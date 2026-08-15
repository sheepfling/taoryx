"""Common model-to-mission authoring and automation helpers.

This module is deliberately provider-neutral.  It consumes only the portable
Mission Composition advertisement plus optional plug-in registrations for
family adapters and controller-tuning campaigns.  Users can therefore author
initial states and segments with ordinary Python/YAML values while Taoryx
still validates the exact provider schema, units, fidelity, realization, and
mission identity before execution.
"""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal, NotRequired, TypeAlias, TypedDict

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .local_direct_wrench_screen_registry import resolve_local_direct_wrench_screen_advertisement
from .trajectory.configuration_contract import (
    ConfigurableTrajectoryProvider,
    ConfigurableTrajectoryProviderRegistry,
    ConfigurationChoiceSchema,
    ConfigurationChoiceValue,
    ConfigurationGroupSchema,
    ConfigurationGroupValue,
    ConfigurationNode,
    ConfigurationNodeValue,
    ConfigurationOptionalSchema,
    ConfigurationOptionalValue,
    ConfigurationParameterSchema,
    ConfigurationParameterValue,
    ConfigurationSequenceSchema,
    ConfigurationSequenceTemplate,
    ConfigurationSequenceValue,
    PreparedTrajectoryConfiguration,
    TrajectoryConfigurationInstance,
    TrajectoryMissionOperationMetadata,
    TrajectoryMissionTemplateMetadata,
    TrajectoryModelMetadata,
    TrajectoryRealizationMetadata,
)
from .trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
    MissionCompositionRunResponse,
    RunnableMissionCompositionProvider,
)
from .vehicle_catalog_resources import vehicle_catalog_resources

if TYPE_CHECKING:
    from .controller_tuning_registry import (
        ControllerTuningCampaignRegistration,
        ControllerTuningCampaignRegistry,
    )
    from .family_adapter_registry import FamilyAdapterRegistry
    from .local_controller_screen_advertisements import LocalControllerScreenAdvertisementRegistry
    from .plugins import PluginCatalog

AUTHORING_DRAFT_SCHEMA = "taoryx.model-authoring-draft/v1"
REQUIRED_VALUE = "<REQUIRED>"
SELECTION_REQUIRED = "<SELECT>"
AuthoringPlanStatus = Literal["ready_to_author", "selection_required"]
AuthoringScalar: TypeAlias = str | int | float | bool | None
AuthoringValue: TypeAlias = AuthoringScalar | list["AuthoringValue"] | dict[str, "AuthoringValue"]
AuthoringPlanRecord: TypeAlias = dict[str, AuthoringValue]


class ModelAuthoringSelectionProjection(TypedDict):
    """Portable selection identity embedded in an authoring-plan projection."""

    provider_id: str
    model_id: str
    model_version: str
    model_metadata_fingerprint: str
    family_id: str | None
    physical_family: str | None
    fidelity: str
    realization_id: str | None
    mission_template_id: str | None
    sources: dict[str, str]

    ####


class ModelAuthoringPlanProjection(TypedDict):
    """Stable JSON projection returned by :func:`build_model_authoring_plan`.

    The plan is a reviewer- and CLI-facing discovery document, rather than an
    executable configuration.  Its named sections deliberately remain plain
    JSON records so it can be written directly to JSON, YAML, or a UI client;
    every record is normalized through :func:`normalize_authoring_values`
    before it leaves this owner.
    """

    schema: Literal["taoryx.model-authoring-plan/v1"]
    status: AuthoringPlanStatus
    selection: ModelAuthoringSelectionProjection
    selection_gaps: list[str]
    data_contract: AuthoringPlanRecord
    controller_automation: AuthoringPlanRecord
    execution_advertisement: AuthoringPlanRecord
    maturity_advertisement: AuthoringPlanRecord
    focused_endpoint_verification: AuthoringPlanRecord
    navigation_automation: AuthoringPlanRecord
    mode_automation: AuthoringPlanRecord
    segment_automation: AuthoringPlanRecord
    configuration_draft: AuthoringPlanRecord
    workflow: list[str]
    claim_boundary: str

    ####


class AuthoringChoiceProjection(TypedDict):
    """Canonical plain-value envelope for a configuration choice."""

    selected: str
    values: dict[str, AuthoringValue]

    ####


class AuthoringSegmentOccurrenceProjection(AuthoringChoiceProjection):
    """A choice projection with an optional stable sequence occurrence ID."""

    instance_id: NotRequired[str]

    ####


class AuthoringSequenceProjection(TypedDict):
    """Canonical plain-value envelope for a templated or custom sequence."""

    template: str | None
    items: list[AuthoringValue]

    ####


class ModelAuthoringError(ValueError):
    """Stable error raised while scaffolding or compiling a simple draft."""

    def __init__(self, code: str, message: str, *, path: str = "authoring") -> None:
        self.code = code
        self.path = path
        super().__init__(f"{code}: {path}: {message}")
        ####

    ####


def normalize_authoring_values(value: object, *, path: str = "values") -> AuthoringValue:
    """Canonicalize portable authoring values before schema-specific compilation.

    Provider schemas still decide which leaf fields, variants, units, and
    ranges are valid.  This boundary handles the separate transport concern:
    a public authoring draft must contain only ordinary JSON-shaped values, not
    mutable custom mappings, framework objects, or opaque Python instances.
    """

    if isinstance(value, BaseModel):
        return normalize_authoring_values(value.model_dump(mode="python", by_alias=True), path=path)
    if value is None or isinstance(value, str | bool):
        return value
    if isinstance(value, int | float):
        return value
    if isinstance(value, Mapping):
        normalized: dict[str, AuthoringValue] = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise ModelAuthoringError(
                    "nonportable-mapping-key",
                    f"expected string key, received {type(key).__name__}",
                    path=path,
                )
            normalized[key] = normalize_authoring_values(item, path=f"{path}.{key}")
        return normalized
    if isinstance(value, Sequence) and not isinstance(value, str | bytes):
        return [normalize_authoring_values(item, path=f"{path}[{index}]") for index, item in enumerate(value)]
    raise ModelAuthoringError(
        "nonportable-authoring-value",
        f"expected a JSON-shaped value, received {type(value).__name__}",
        path=path,
    )
    ####


def _authoring_mapping(value: object, *, path: str) -> dict[str, AuthoringValue]:
    """Normalize an authoring object that must remain a string-keyed mapping."""

    normalized = normalize_authoring_values(value, path=path)
    if not isinstance(normalized, dict):
        raise ModelAuthoringError("invalid-projection-values", "expected a mapping", path=path)
    return normalized
    ####


def _authoring_plan_record(value: object, *, path: str) -> AuthoringPlanRecord:
    """Normalize one named public plan section into canonical JSON values."""

    return _authoring_mapping(value, path=path)
    ####


def _authoring_plan_text(value: object, *, path: str) -> str:
    """Require one non-empty text field in a public plan projection."""

    if not isinstance(value, str) or not value.strip():
        raise ModelAuthoringError("invalid-plan-projection", "expected non-empty text", path=path)
    return value
    ####


def _authoring_plan_optional_text(value: object, *, path: str) -> str | None:
    """Require an optional text field in a public plan projection."""

    if value is None:
        return None
    return _authoring_plan_text(value, path=path)
    ####


def _authoring_plan_text_list(value: object, *, path: str) -> list[str]:
    """Require a canonical ordered text list in a public plan projection."""

    if not isinstance(value, list):
        raise ModelAuthoringError("invalid-plan-projection", "expected a list", path=path)
    return [_authoring_plan_text(item, path=f"{path}[{index}]") for index, item in enumerate(value)]
    ####


def _model_authoring_selection_projection(value: object) -> ModelAuthoringSelectionProjection:
    """Validate the stable identity section of a public authoring plan."""

    payload = _authoring_plan_record(value, path="model_authoring_plan.selection")
    raw_sources = payload.get("sources")
    sources = _authoring_plan_record(raw_sources, path="model_authoring_plan.selection.sources")
    return {
        "provider_id": _authoring_plan_text(payload.get("provider_id"), path="model_authoring_plan.selection.provider_id"),
        "model_id": _authoring_plan_text(payload.get("model_id"), path="model_authoring_plan.selection.model_id"),
        "model_version": _authoring_plan_text(payload.get("model_version"), path="model_authoring_plan.selection.model_version"),
        "model_metadata_fingerprint": _authoring_plan_text(
            payload.get("model_metadata_fingerprint"),
            path="model_authoring_plan.selection.model_metadata_fingerprint",
        ),
        "family_id": _authoring_plan_optional_text(payload.get("family_id"), path="model_authoring_plan.selection.family_id"),
        "physical_family": _authoring_plan_optional_text(
            payload.get("physical_family"),
            path="model_authoring_plan.selection.physical_family",
        ),
        "fidelity": _authoring_plan_text(payload.get("fidelity"), path="model_authoring_plan.selection.fidelity"),
        "realization_id": _authoring_plan_optional_text(
            payload.get("realization_id"),
            path="model_authoring_plan.selection.realization_id",
        ),
        "mission_template_id": _authoring_plan_optional_text(
            payload.get("mission_template_id"),
            path="model_authoring_plan.selection.mission_template_id",
        ),
        "sources": {key: _authoring_plan_text(item, path=f"model_authoring_plan.selection.sources.{key}") for key, item in sources.items()},
    }
    ####


def _model_authoring_plan_projection(value: object) -> ModelAuthoringPlanProjection:
    """Validate and normalize the public authoring-plan envelope once at its owner."""

    payload = _authoring_plan_record(value, path="model_authoring_plan")
    if payload.get("schema") != "taoryx.model-authoring-plan/v1":
        raise ModelAuthoringError(
            "invalid-plan-projection",
            "expected schema taoryx.model-authoring-plan/v1",
            path="model_authoring_plan.schema",
        )
    raw_status = payload.get("status")
    if raw_status == "ready_to_author":
        status: AuthoringPlanStatus = "ready_to_author"
    elif raw_status == "selection_required":
        status = "selection_required"
    else:
        raise ModelAuthoringError(
            "invalid-plan-projection",
            "expected ready_to_author or selection_required status",
            path="model_authoring_plan.status",
        )
    return {
        "schema": "taoryx.model-authoring-plan/v1",
        "status": status,
        "selection": _model_authoring_selection_projection(payload.get("selection")),
        "selection_gaps": _authoring_plan_text_list(payload.get("selection_gaps"), path="model_authoring_plan.selection_gaps"),
        "data_contract": _authoring_plan_record(payload.get("data_contract"), path="model_authoring_plan.data_contract"),
        "controller_automation": _authoring_plan_record(
            payload.get("controller_automation"),
            path="model_authoring_plan.controller_automation",
        ),
        "execution_advertisement": _authoring_plan_record(
            payload.get("execution_advertisement"),
            path="model_authoring_plan.execution_advertisement",
        ),
        "maturity_advertisement": _authoring_plan_record(
            payload.get("maturity_advertisement"),
            path="model_authoring_plan.maturity_advertisement",
        ),
        "focused_endpoint_verification": _authoring_plan_record(
            payload.get("focused_endpoint_verification"),
            path="model_authoring_plan.focused_endpoint_verification",
        ),
        "navigation_automation": _authoring_plan_record(
            payload.get("navigation_automation"),
            path="model_authoring_plan.navigation_automation",
        ),
        "mode_automation": _authoring_plan_record(payload.get("mode_automation"), path="model_authoring_plan.mode_automation"),
        "segment_automation": _authoring_plan_record(
            payload.get("segment_automation"),
            path="model_authoring_plan.segment_automation",
        ),
        "configuration_draft": _authoring_plan_record(
            payload.get("configuration_draft"),
            path="model_authoring_plan.configuration_draft",
        ),
        "workflow": _authoring_plan_text_list(payload.get("workflow"), path="model_authoring_plan.workflow"),
        "claim_boundary": _authoring_plan_text(payload.get("claim_boundary"), path="model_authoring_plan.claim_boundary"),
    }
    ####


class ModelAuthoringDraft(BaseModel):
    """Editable plain-value configuration bound to one advertised schema."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.model-authoring-draft/v1"] = Field(
        default="taoryx.model-authoring-draft/v1",
        alias="schema",
        serialization_alias="schema",
    )
    draft_id: str = Field(min_length=1)
    configuration_id: str = Field(min_length=1)
    provider_id: str = Field(min_length=1)
    model_id: str = Field(min_length=1)
    model_version: str = Field(min_length=1)
    schema_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    fidelity: str = Field(min_length=1)
    realization_id: str | None = None
    mission_template_id: str | None = None
    selection_sources: dict[str, str] = Field(default_factory=dict)
    values: Any
    claim_boundary: str = (
        "An authoring draft is editable input in canonical advertised units. "
        "It is not executable until the exact provider validates it, and it "
        "does not qualify the selected model, controller, or mission."
    )

    @model_validator(mode="before")
    @classmethod
    def normalize_plain_values(cls, payload: object) -> object:
        """Keep the serialized draft at a portable, canonical projection boundary."""

        if not isinstance(payload, Mapping):
            return payload
        values = dict(payload)
        if "values" in values:
            values["values"] = normalize_authoring_values(values["values"])
        return values
        ####

    @property
    def unresolved_inputs(self) -> tuple[str, ...]:
        """Return every placeholder that still requires an author decision."""

        paths: list[str] = []
        _collect_placeholders(self.values, path="values", result=paths)
        return tuple(paths)
        ####

    def public_dict(self, *, include_diagnostics: bool = True) -> dict[str, Any]:
        """Return a portable draft and optional current completion state."""

        payload = self.model_dump(mode="json", by_alias=True)
        if include_diagnostics:
            payload["completion"] = {
                "status": "inputs_required" if self.unresolved_inputs else "complete",
                "unresolved_inputs": list(self.unresolved_inputs),
            }
        return payload
        ####

    ####


@dataclass(frozen=True, slots=True)
class ModelAuthoringSelection:
    """Resolved provider/model selections and their declared provenance."""

    provider_id: str
    model: TrajectoryModelMetadata
    fidelity: str
    realization: TrajectoryRealizationMetadata | None
    mission: TrajectoryMissionTemplateMetadata | None
    sources: Mapping[str, str]

    def public_dict(self) -> ModelAuthoringSelectionProjection:
        """Return the selected identities without embedding full schemas."""

        return {
            "provider_id": self.provider_id,
            "model_id": self.model.id,
            "model_version": self.model.version,
            "model_metadata_fingerprint": self.model.metadata_fingerprint,
            "family_id": self.model.family_id,
            "physical_family": self.model.physical_family,
            "fidelity": self.fidelity,
            "realization_id": self.realization.id if self.realization is not None else None,
            "mission_template_id": self.mission.id if self.mission is not None else None,
            "sources": dict(self.sources),
        }
        ####

    ####


def select_variant(
    variant_id: str,
    values: Mapping[str, Any] | None = None,
    **parameters: Any,
) -> AuthoringChoiceProjection:
    """Build the plain authoring form for one advertised choice variant."""

    if not variant_id.strip():
        raise ValueError("choice variant ID must be non-empty")
    merged = dict(values or {})
    overlap = sorted(set(merged) & set(parameters))
    if overlap:
        raise ValueError(f"choice values were supplied twice: {overlap!r}")
    merged.update(parameters)
    return {
        "selected": variant_id,
        "values": _authoring_mapping(merged, path=f"choice[{variant_id}].values"),
    }
    ####


def sequence_template(
    template_id: str,
    *items: Mapping[str, Any],
) -> AuthoringSequenceProjection:
    """Build a concise ordered segment value using an advertised template."""

    if not template_id.strip():
        raise ValueError("sequence template ID must be non-empty")
    return {
        "template": template_id,
        "items": [normalize_authoring_values(item, path=f"sequence[{template_id}].items[{index}]") for index, item in enumerate(items)],
    }
    ####


def segment_occurrence(
    segment_id: str,
    values: Mapping[str, Any] | None = None,
    *,
    instance_id: str | None = None,
    **parameters: Any,
) -> AuthoringSegmentOccurrenceProjection:
    """Build one explicitly typed occurrence for an open segment sequence."""

    projection = select_variant(segment_id, values, **parameters)
    occurrence: AuthoringSegmentOccurrenceProjection = {
        "selected": projection["selected"],
        "values": projection["values"],
    }
    if instance_id is not None:
        if not instance_id.strip():
            raise ValueError("segment instance ID must be non-empty when supplied")
        occurrence["instance_id"] = instance_id
    return occurrence
    ####


def custom_sequence(*items: Mapping[str, Any]) -> AuthoringSequenceProjection:
    """Build an open ordered sequence from explicit segment occurrences."""

    return {
        "template": None,
        "items": [normalize_authoring_values(item, path=f"custom_sequence.items[{index}]") for index, item in enumerate(items)],
    }
    ####


def resolve_model_authoring_selection(
    providers: ConfigurableTrajectoryProviderRegistry,
    provider_id: str,
    model_id: str,
    *,
    fidelity: str | None = None,
    realization_id: str | None = None,
    mission_template_id: str | None = None,
    allow_blocked_local_design: bool = False,
) -> ModelAuthoringSelection:
    """Resolve advertised selections, optionally for a registered local design.

    Ordinary authoring, compilation, and execution remain limited to available
    realizations.  The explicit opt-in is for the controller-tuning host only:
    a model-owned registered campaign may legitimately screen a source-local
    plant while its end-to-end mission realization is still blocked.
    """

    model = providers.model(provider_id, model_id)
    sources: dict[str, str] = {}
    selected_fidelity = fidelity
    if selected_fidelity is None:
        selected_fidelity = model.presentation.default_fidelity_id
        if selected_fidelity is not None:
            sources["fidelity"] = "advertised_presentation_default"
        else:
            declared = tuple(item.id for item in model.fidelities if item.declared)
            if len(declared) != 1:
                raise ModelAuthoringError(
                    "fidelity-selection-required",
                    f"model advertises fidelities {list(declared)!r}; select one explicitly",
                    path="fidelity",
                )
            selected_fidelity = declared[0]
            sources["fidelity"] = "only_declared_fidelity"
    else:
        sources["fidelity"] = "caller"
    known_fidelities = {item.id for item in model.fidelities if item.declared}
    if selected_fidelity not in known_fidelities:
        raise ModelAuthoringError(
            "unsupported-fidelity",
            f"expected one of {sorted(known_fidelities)!r}",
            path="fidelity",
        )

    mission = _resolve_mission(model, mission_template_id, sources)
    if mission is not None and selected_fidelity not in mission.compatible_fidelities:
        compatible = tuple(identifier for identifier in mission.compatible_fidelities if identifier in known_fidelities)
        if fidelity is None and compatible:
            selected_fidelity = compatible[0]
            sources["fidelity"] = (
                "only_fidelity_compatible_with_advertised_mission" if len(compatible) == 1 else "first_advertised_fidelity_compatible_with_advertised_mission"
            )
        elif mission_template_id is None:
            replacement = _first_available_mission_for_fidelity(model, selected_fidelity)
            if replacement is not None:
                mission = replacement
                sources["mission"] = "first_available_fidelity_compatible_mission"
            else:
                raise ModelAuthoringError(
                    "mission-fidelity-mismatch",
                    (f"mission {mission.id!r} supports {list(compatible)!r}; select a compatible mission explicitly"),
                    path="mission_template_id",
                )
        else:
            raise ModelAuthoringError(
                "mission-fidelity-mismatch",
                (f"mission {mission.id!r} supports {list(compatible)!r}; select a compatible fidelity explicitly"),
                path="fidelity",
            )
    realization = _resolve_realization(
        model,
        selected_fidelity,
        realization_id,
        mission,
        sources,
        allow_blocked_local_design=allow_blocked_local_design,
    )
    return ModelAuthoringSelection(
        provider_id,
        model,
        selected_fidelity,
        realization,
        mission,
        sources,
    )
    ####


def scaffold_model_authoring_draft(
    providers: ConfigurableTrajectoryProviderRegistry,
    provider_id: str,
    model_id: str,
    *,
    fidelity: str | None = None,
    realization_id: str | None = None,
    mission_template_id: str | None = None,
    draft_id: str | None = None,
    configuration_id: str | None = None,
) -> ModelAuthoringDraft:
    """Generate an editable draft from one model's portable advertisement."""

    selection = resolve_model_authoring_selection(
        providers,
        provider_id,
        model_id,
        fidelity=fidelity,
        realization_id=realization_id,
        mission_template_id=mission_template_id,
    )
    schema = providers.get_model_schema(provider_id, model_id)
    values = _scaffold_node(
        schema.root,
        fidelity=selection.fidelity,
        mission=selection.mission,
    )
    suffix = selection.mission.id if selection.mission is not None else selection.fidelity
    generated_id = f"{model_id}-{suffix}"
    return ModelAuthoringDraft(
        draft_id=draft_id or f"{generated_id}-draft",
        configuration_id=configuration_id or generated_id,
        provider_id=provider_id,
        model_id=model_id,
        model_version=schema.model_version,
        schema_fingerprint=schema.fingerprint,
        fidelity=selection.fidelity,
        realization_id=selection.realization.id if selection.realization is not None else None,
        mission_template_id=selection.mission.id if selection.mission is not None else None,
        selection_sources=dict(selection.sources),
        values=values,
    )
    ####


def compile_model_authoring_draft(
    providers: ConfigurableTrajectoryProviderRegistry,
    draft: ModelAuthoringDraft,
) -> PreparedTrajectoryConfiguration:
    """Compile plain values and validate through the exact selected provider."""

    if draft.unresolved_inputs:
        raise ModelAuthoringError(
            "unresolved-inputs",
            "replace every generated placeholder: " + ", ".join(draft.unresolved_inputs),
            path="values",
        )
    model = providers.model(draft.provider_id, draft.model_id)
    schema = providers.get_model_schema(draft.provider_id, draft.model_id)
    if draft.model_version != model.version or draft.model_version != schema.model_version:
        raise ModelAuthoringError(
            "model-version-mismatch",
            f"draft targets {draft.model_version!r}, installed model is {model.version!r}",
            path="model_version",
        )
    if draft.schema_fingerprint != schema.fingerprint:
        raise ModelAuthoringError(
            "schema-fingerprint-mismatch",
            "the installed provider advertisement changed; regenerate or migrate the draft",
            path="schema_fingerprint",
        )
    root = _compile_node(
        schema.root,
        draft.values,
        fidelity=draft.fidelity,
        path="values",
    )
    configuration = TrajectoryConfigurationInstance(
        configuration_id=draft.configuration_id,
        model_id=draft.model_id,
        model_version=draft.model_version,
        schema_fingerprint=draft.schema_fingerprint,
        fidelity=draft.fidelity,
        realization_id=draft.realization_id,
        mission_template_id=draft.mission_template_id,
        root=root,
    )
    return providers.validate_configuration(draft.provider_id, configuration)
    ####


def run_prepared_mission_composition(
    providers: ConfigurableTrajectoryProviderRegistry,
    provider_id: str,
    prepared: PreparedTrajectoryConfiguration,
    *,
    request_id: str | None = None,
    output: MissionCompositionOutputSelection | None = None,
) -> MissionCompositionRunResponse:
    """Run one prepared configuration through its provider-owned batch runner.

    Prepared configurations deliberately omit provider identity so that their
    configuration data remains portable. Callers must therefore choose the
    provider at dispatch time. The configuration is revalidated first, which
    prevents a serialized record from silently crossing a provider or schema
    update.
    """

    provider = providers.provider(provider_id)
    current = provider.validate_configuration(prepared.configuration)
    if current.fingerprint != prepared.fingerprint:
        raise ModelAuthoringError(
            "prepared-configuration-stale",
            "the serialized prepared configuration differs from the current provider validation result",
            path="prepared_configuration.fingerprint",
        )
    if not isinstance(provider, RunnableMissionCompositionProvider):
        raise ModelAuthoringError(
            "execution-runner-unavailable",
            f"provider {provider_id!r} does not publish a common batch runner for this model",
            path="provider_id",
        )
    runner = provider.build_runner()
    if not isinstance(runner, MissionCompositionRunnerRegistry):
        raise ModelAuthoringError(
            "invalid-execution-runner",
            f"provider {provider_id!r} returned an object outside the common Mission Composition runner contract",
            path="provider_id",
        )
    return runner.run(
        MissionCompositionRunRequest(
            request_id=request_id or prepared.configuration.configuration_id,
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=current,
            output=output or MissionCompositionOutputSelection(),
        )
    )
    ####


def author_configuration(
    provider: ConfigurableTrajectoryProvider,
    *,
    configuration_id: str,
    model_id: str,
    fidelity: str,
    values: Any,
    realization_id: str | None = None,
    mission_template_id: str | None = None,
) -> PreparedTrajectoryConfiguration:
    """Author and validate one configuration from concise Python values.

    Numeric values are interpreted in the canonical units explicitly declared
    by the provider schema.  No conversion, clipping, or default invention is
    performed by this helper.
    """

    schema = provider.get_model_schema(model_id)
    draft = ModelAuthoringDraft(
        draft_id=f"{configuration_id}-draft",
        configuration_id=configuration_id,
        provider_id=provider.metadata.id,
        model_id=model_id,
        model_version=schema.model_version,
        schema_fingerprint=schema.fingerprint,
        fidelity=fidelity,
        realization_id=realization_id,
        mission_template_id=mission_template_id,
        selection_sources={
            "fidelity": "caller",
            "realization": "caller" if realization_id is not None else "not_selected",
            "mission": "caller" if mission_template_id is not None else "not_selected",
        },
        values=values,
    )
    registry = ConfigurableTrajectoryProviderRegistry((provider,))
    return compile_model_authoring_draft(registry, draft)
    ####


def build_model_authoring_plan(
    providers: ConfigurableTrajectoryProviderRegistry,
    tuning_campaigns: ControllerTuningCampaignRegistry,
    provider_id: str,
    model_id: str,
    *,
    family_adapters: FamilyAdapterRegistry | None = None,
    local_controller_screens: LocalControllerScreenAdvertisementRegistry | None = None,
    fidelity: str | None = None,
    realization_id: str | None = None,
    mission_template_id: str | None = None,
) -> ModelAuthoringPlanProjection:
    """Join advertised data, controls, tuning, navigation, and segment work."""

    selection = resolve_model_authoring_selection(
        providers,
        provider_id,
        model_id,
        fidelity=fidelity,
        realization_id=realization_id,
        mission_template_id=mission_template_id,
    )
    schema = providers.get_model_schema(provider_id, model_id)
    draft = scaffold_model_authoring_draft(
        providers,
        provider_id,
        model_id,
        fidelity=selection.fidelity,
        realization_id=selection.realization.id if selection.realization is not None else None,
        mission_template_id=selection.mission.id if selection.mission is not None else None,
    )
    campaign_matches = tuning_campaigns.matching(
        provider_id=provider_id,
        model_id=model_id,
        fidelity=selection.fidelity,
        realization_id=selection.realization.id if selection.realization is not None else None,
        mission_template_id=selection.mission.id if selection.mission is not None else None,
    )
    adapter_record = _family_adapter_record(
        family_adapters,
        family_id=selection.model.family_id,
        fidelity=selection.fidelity,
    )
    controller = _controller_plan(
        selection,
        campaign_matches,
        adapter_record,
        local_controller_screens=local_controller_screens,
    )
    execution_advertisement = _execution_advertisement(selection)
    maturity_advertisement = _maturity_advertisement(selection)
    provider_catalog = getattr(providers.provider(provider_id), "plugin_catalog", None)
    focused_endpoint_verification = _focused_endpoint_verification_advertisement(
        selection,
        plugin_catalog=provider_catalog,
    )
    segment_instances = _segment_instances(schema.root, selection.mission)
    navigation_parameters = [item for item in _parameter_records(schema.root) if _is_navigation_parameter(item)]
    selection_gaps: list[str] = []
    if selection.realization is None:
        selection_gaps.append("select_realization")
    if selection.model.mission_templates and selection.mission is None:
        selection_gaps.append("select_mission_template")
    status: AuthoringPlanStatus = "selection_required" if selection_gaps else "ready_to_author"
    return _model_authoring_plan_projection(
        {
            "schema": "taoryx.model-authoring-plan/v1",
            "status": status,
            "selection": selection.public_dict(),
            "selection_gaps": selection_gaps,
            "data_contract": {
                "model_kind": selection.model.model_kind,
                "source_refs": list(selection.model.source_refs),
                "provenance": selection.model.provenance,
                "properties": [item.model_dump(mode="json") for item in selection.model.presentation.properties],
                "reference_frames": [item.model_dump(mode="json") for item in selection.model.reference_frames],
                "configuration_schema_id": selection.model.configuration_schema_id,
                "configuration_schema_fingerprint": selection.model.configuration_schema_fingerprint,
                "output_schema_id": selection.model.output_schema_id,
                "output_schema_fingerprint": selection.model.output_schema_fingerprint,
                "core_output_channels": [item.id for item in selection.model.output_schema.core_channels],
                "telemetry_groups": [item.id for item in selection.model.output_schema.telemetry_groups],
            },
            "controller_automation": controller,
            "execution_advertisement": execution_advertisement,
            "maturity_advertisement": maturity_advertisement,
            "focused_endpoint_verification": focused_endpoint_verification,
            "navigation_automation": {
                "mission_template": selection.mission.model_dump(mode="json") if selection.mission is not None else None,
                "navigation_parameters": navigation_parameters,
                "policy": (
                    "Route and waypoint geometry come from declared mission parameters or a registered "
                    "capability compiler. The authoring layer never invents coordinates or envelope limits."
                ),
            },
            "mode_automation": {
                "initialization_modes": list(selection.model.capabilities.initialization_modes),
                "segment_types": list(selection.model.capabilities.segment_types),
                "termination_modes": list(selection.model.capabilities.termination_modes),
                "selected_realization": selection.realization.id if selection.realization is not None else None,
                "default_authority": (selection.realization.controls.default_authority_id if selection.realization is not None else None),
                "control_intents": (
                    [item.model_dump(mode="json") for item in selection.realization.controls.intents] if selection.realization is not None else []
                ),
            },
            "segment_automation": {
                "supports_custom_segments": selection.model.capabilities.supports_custom_segments,
                "instances": segment_instances,
                "plain_value_api": {
                    "python": "taoryx.model_authoring.author_configuration",
                    "choice_helper": "taoryx.model_authoring.select_variant",
                    "sequence_helper": "taoryx.model_authoring.sequence_template",
                    "segment_helper": "taoryx.model_authoring.segment_occurrence",
                    "custom_sequence_helper": "taoryx.model_authoring.custom_sequence",
                    "canonical_unit_rule": "plain numeric values use the units advertised by the selected parameter schema",
                },
            },
            "configuration_draft": {
                "schema": AUTHORING_DRAFT_SCHEMA,
                "unresolved_input_count": len(draft.unresolved_inputs),
                "unresolved_inputs": list(draft.unresolved_inputs),
            },
            "workflow": [
                f"taoryx model scaffold {provider_id} {model_id} --fidelity {selection.fidelity}"
                + (f" --mission {selection.mission.id}" if selection.mission is not None else "")
                + " --output <draft.yaml>",
                "edit <draft.yaml> and replace every <REQUIRED>/<SELECT> placeholder",
                "taoryx model compile <draft.yaml> --output <prepared-configuration.json>",
                *(
                    [
                        f"taoryx model tune {provider_id} {model_id} --fidelity {selection.fidelity} "
                        f"--campaign {campaign_matches[0].id} --output <tuning-report.json>"
                    ]
                    if len(campaign_matches) == 1
                    else []
                ),
            ],
            "claim_boundary": (
                "This plan automates discovery, draft generation, schema validation, and registered local tuning campaigns. "
                "It does not infer missing physics, operating points, authority, route geometry, or qualification evidence."
            ),
        }
    )
    ####


@lru_cache(maxsize=1)
def _vehicle_maturity_records() -> tuple[Mapping[str, object], ...]:
    """Load the small planning ledger once for public plan advertisements."""

    records: list[Mapping[str, object]] = []
    for source in vehicle_catalog_resources("verification/vehicle_maturity_registry.yaml"):
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping) or payload.get("registry_id") != "taoryx_vehicle_maturity_v1":
            raise ValueError("invalid vehicle maturity registry")
        fragment_records = payload.get("records")
        if not isinstance(fragment_records, list):
            raise ValueError("vehicle maturity registry must contain records")
        records.extend(record for record in fragment_records if isinstance(record, Mapping))
    identifiers = [record.get("id") for record in records]
    if len(identifiers) != len(set(identifiers)):
        raise ValueError("vehicle maturity registry has duplicate record IDs across plug-in fragments")
    return tuple(records)
    ####


def _maturity_advertisement(selection: ModelAuthoringSelection) -> dict[str, object]:
    """Project family-level maturity separately from exact endpoint availability."""

    family_id = selection.model.family_id or selection.model.id
    record = next(
        (candidate for candidate in _vehicle_maturity_records() if candidate.get("composition_family_id") == family_id or candidate.get("id") == family_id),
        None,
    )
    claim_boundary = (
        "This is the planning-ledger maturity of the selected vehicle family. It is separate from the exact selected "
        "endpoint's operation matrix, controller evidence, physical-allocation evidence, and qualification status."
    )
    if record is None:
        return {
            "status": "not_declared",
            "family_id": family_id,
            "claim_boundary": claim_boundary,
        }
    return {
        "status": "declared",
        "record_id": record["id"],
        "family_id": family_id,
        "composition_family_id": record.get("composition_family_id"),
        "maturity": record["maturity"],
        "evidence_strength": record["evidence_strength"],
        "current_status": record["status"],
        "next_gate": record["next_gate"],
        "claim_boundary": claim_boundary,
    }
    ####


def _execution_advertisement(selection: ModelAuthoringSelection) -> dict[str, object]:
    """Project exact selected mission operations into a portable plan boundary.

    Provider metadata owns operation availability, executor identity, and
    blockers.  The common authoring plan only selects the exact
    mission/fidelity/realization row and makes it visible beside controls and
    segment metadata; it never creates an endpoint or substitutes an
    incompatible realization.
    """

    mission = selection.mission
    if mission is None:
        return {
            "status": "not_advertised",
            "mission_template_id": None,
            "operation_records": [],
            "available_operations": [],
            "blocked_operations": [],
            "endpoint_maturity": "not_advertised",
            "claim_boundary": (
                "The selected model has no mission-template operation matrix. No batch or interactive endpoint is "
                "inferred from model discovery or schema availability."
            ),
        }

    realization_id = selection.realization.id if selection.realization is not None else None
    records_by_operation: dict[str, TrajectoryMissionOperationMetadata] = {}
    for record in mission.operations:
        if record.fidelity != selection.fidelity:
            continue
        if record.realization_id not in {None, realization_id}:
            continue
        existing = records_by_operation.get(record.operation)
        # An exact realization-specific declaration overrides a generic
        # mission/fidelity declaration for that operation.
        if existing is None or record.realization_id == realization_id:
            records_by_operation[record.operation] = record
    operation_order = ("validate", "batch", "step")
    records = tuple(records_by_operation[operation] for operation in operation_order if operation in records_by_operation)
    if not records:
        return {
            "status": "not_advertised",
            "mission_template_id": mission.id,
            "operation_records": [],
            "available_operations": [],
            "blocked_operations": [],
            "endpoint_maturity": "not_advertised",
            "claim_boundary": (
                "The selected mission has no operation advertisement for this exact fidelity and realization. "
                "No endpoint is inferred from a neighboring fidelity or realization."
            ),
        }

    available = [record.operation for record in records if record.status == "available"]
    blocked = [record.model_dump(mode="json") for record in records if record.status == "blocked"]
    executable = {operation for operation in available if operation in {"batch", "step"}}
    endpoint_maturity = (
        "batch_and_step_ready"
        if executable == {"batch", "step"}
        else "batch_ready"
        if executable == {"batch"}
        else "step_ready"
        if executable == {"step"}
        else "declared_execution_gap"
    )
    return {
        "status": "runnable" if executable else "validation_only",
        "mission_template_id": mission.id,
        "operation_records": [record.model_dump(mode="json") for record in records],
        "available_operations": available,
        "blocked_operations": blocked,
        "endpoint_maturity": endpoint_maturity,
        "claim_boundary": (
            "This is the exact selected mission operation advertisement. It does not execute the endpoint, establish "
            "batch/step parity, create a controller, or promote qualification."
        ),
    }
    ####


def _focused_endpoint_verification_advertisement(
    selection: ModelAuthoringSelection,
    *,
    plugin_catalog: PluginCatalog | None = None,
) -> dict[str, object]:
    """Advertise checked-in vertical proofs without conflating them with a plan.

    Physical vehicle endpoints and provider-owned workflow endpoints have
    distinct verifiers because they make different evidence claims. The
    authoring plan joins both catalogs only by their published identity and
    reports exactly how closely each proof matches the current selection. It
    never treats a nearby mission or fidelity as evidence for the selected
    configuration.
    """

    # These imports are intentionally local: the workflow verifier consumes
    # authoring helpers, while this public plan needs to expose the verifier
    # only after the module has finished importing.
    from .mission_workflow_endpoint import load_mission_workflow_endpoint_catalog
    from .vehicle_endpoint_spec import load_vehicle_endpoint_spec_catalog

    selected_mission_id = selection.mission.id if selection.mission is not None else None
    selected_realization_id = selection.realization.id if selection.realization is not None else None
    endpoints: list[dict[str, object]] = []

    if selection.model.model_kind == "canonical_vehicle_family":
        for vehicle_endpoint in load_vehicle_endpoint_spec_catalog().endpoints:
            if vehicle_endpoint.model_id != selection.model.id or selection.provider_id not in vehicle_endpoint.provider_ids:
                continue
            matches_selected_mission_and_fidelity = vehicle_endpoint.mission_id == selected_mission_id and vehicle_endpoint.fidelity == selection.fidelity
            endpoints.append(
                {
                    "id": vehicle_endpoint.id,
                    "kind": "vehicle_composition",
                    "maturity_record_id": vehicle_endpoint.maturity_record_id,
                    "matches_selected_mission_and_fidelity": matches_selected_mission_and_fidelity,
                    "match_scope": "provider_id, model_id, mission_id, fidelity",
                    "command": f"taoryx vehicle verify {vehicle_endpoint.id}",
                    "execute_command": f"taoryx vehicle verify {vehicle_endpoint.id} --execute",
                }
            )

    # The provider/model identity is the workflow-proof boundary. New
    # nonphysical kinds (for example, analytical or contract fixtures) do not
    # need a core-code allowlist merely to advertise their package-owned
    # endpoint evidence.
    if selection.model.model_kind != "canonical_vehicle_family":
        for workflow_endpoint in load_mission_workflow_endpoint_catalog(plugins=plugin_catalog).endpoints:
            if selection.provider_id not in workflow_endpoint.provider_ids or workflow_endpoint.model_id != selection.model.id:
                continue
            matches_selected_configuration = (
                workflow_endpoint.fidelity == selection.fidelity
                and workflow_endpoint.realization_id == selected_realization_id
                and workflow_endpoint.mission_template_id == selected_mission_id
            )
            endpoints.append(
                {
                    "id": workflow_endpoint.id,
                    "kind": "mission_workflow",
                    "maturity_record_id": workflow_endpoint.maturity_record_id,
                    "matches_selected_configuration": matches_selected_configuration,
                    "match_scope": "provider_id (or declared alias), model_id, mission_template_id, fidelity, realization_id",
                    "command": f"taoryx model verify {workflow_endpoint.id}",
                    "execute_command": f"taoryx model verify {workflow_endpoint.id} --execute",
                }
            )

    endpoints.sort(key=lambda item: (str(item["kind"]), str(item["id"])))
    selected_endpoint_ids = [
        str(item["id"]) for item in endpoints if item.get("matches_selected_mission_and_fidelity") is True or item.get("matches_selected_configuration") is True
    ]
    if not endpoints:
        status = "not_declared"
        selected_status = "not_declared"
    elif selected_mission_id is None:
        status = "available"
        selected_status = "selection_required"
    elif selected_endpoint_ids:
        status = "available"
        selected_status = "matching_endpoint_available"
    else:
        status = "available"
        selected_status = "different_declared_endpoint"
    return {
        "schema": "taoryx.focused-endpoint-advertisement/v1alpha1",
        "status": status,
        "selected_endpoint_status": selected_status,
        "selected_endpoint_ids": selected_endpoint_ids,
        "endpoints": endpoints,
        "claim_boundary": (
            "A focused verifier proves only its checked-in endpoint contract. A matching identity does not execute "
            "the verifier, establish robustness beyond its declared screen, or qualify the selected model."
        ),
    }
    ####


def build_model_automation_assessment(
    providers: ConfigurableTrajectoryProviderRegistry,
    tuning_campaigns: ControllerTuningCampaignRegistry,
    *,
    family_adapters: FamilyAdapterRegistry | None = None,
    local_controller_screens: LocalControllerScreenAdvertisementRegistry | None = None,
    provider_id: str | None = None,
) -> dict[str, object]:
    """Return a compact, all-model readiness matrix from advertisements.

    This is deliberately an inventory, rather than a qualification report.
    It makes the common authoring and controller seams visible for every
    realization, including the descriptor of each registered campaign-owned
    adapter. It does not run trim, linearization, a numerical campaign, or
    invent a lowering path.
    """

    selected_providers = providers.providers
    if provider_id is not None:
        selected_providers = (providers.provider(provider_id),)

    provider_records: list[dict[str, object]] = []
    for provider in selected_providers:
        model_records: list[dict[str, object]] = []
        for model in provider.list_models():
            schema = providers.get_model_schema(provider.metadata.id, model.id)
            advertisement = _advertisement_exercise_record(
                providers,
                tuning_campaigns,
                provider_id=provider.metadata.id,
                model=model,
                family_adapters=family_adapters,
                local_controller_screens=local_controller_screens,
            )
            realization_records = [
                _automation_realization_record(
                    provider_id=provider.metadata.id,
                    model=model,
                    realization=realization,
                    tuning_campaigns=tuning_campaigns,
                    family_adapters=family_adapters,
                    local_controller_screens=local_controller_screens,
                )
                for realization in model.realizations
            ]
            model_records.append(
                {
                    "id": model.id,
                    "version": model.version,
                    "display_name": model.presentation.display_name,
                    "family_id": model.family_id,
                    "model_kind": model.model_kind,
                    "status": model.status,
                    "advertisement": {
                        "status": "complete" if advertisement["status"] == "pass" else "incomplete",
                        "configuration_schema_id": model.configuration_schema_id,
                        "configuration_schema_fingerprint": model.configuration_schema_fingerprint,
                        "output_schema_id": model.output_schema_id,
                        "output_schema_fingerprint": model.output_schema_fingerprint,
                        "property_count": len(model.presentation.properties),
                        "reference_frame_ids": [item.id for item in model.reference_frames],
                        "core_output_channels": [item.id for item in model.output_schema.core_channels],
                        "telemetry_group_ids": [item.id for item in model.output_schema.telemetry_groups],
                        "source_ref_count": len(model.source_refs),
                        "schema_root_id": schema.root.id,
                        "plan_exercise": advertisement,
                        "claim_boundary": (
                            "Complete means the typed advertisement validates and its common authoring plan exposes "
                            "data, controls, navigation, modes, and segment APIs. It does not qualify a physical "
                            "plant or mission result."
                        ),
                    },
                    "generic_authoring": {
                        "status": "advertised",
                        "operations": ["plan", "scaffold", "compile"],
                        "supports_custom_segments": model.capabilities.supports_custom_segments,
                        "segment_types": list(model.capabilities.segment_types),
                        "claim_boundary": (
                            "The host can generate and validate plain-value drafts against this advertised schema; "
                            "it does not invent routes, segments, or envelope limits."
                        ),
                    },
                    "realizations": realization_records,
                }
            )
        provider_records.append(
            {
                "id": provider.metadata.id,
                "display_name": provider.metadata.presentation.display_name,
                "models": model_records,
            }
        )

    readiness_summary = _controller_automation_summary(provider_records)
    advertisement_summary = _advertisement_readiness_summary(provider_records)
    return {
        "schema": "taoryx.model-automation-assessment/v1",
        "providers": provider_records,
        "advertisement_readiness_summary": advertisement_summary,
        "controller_readiness_summary": readiness_summary,
        "rslqr": {
            "status": "deferred",
            "reason": (
                "RSLQR/adaptive controller synthesis is not an executable common backend yet. "
                "A realization must not be presented as RSLQR-ready from its metadata alone."
            ),
        },
        "claim_boundary": (
            "This matrix reports installed typed advertisements, common authoring availability, family-adapter "
            "registrations, and tuning-campaign registration. It does not assert executable fidelity lowering, "
            "controller qualification, actuator allocation, or RSLQR availability."
        ),
    }
    ####


def _advertisement_readiness_summary(provider_records: Sequence[Mapping[str, object]]) -> dict[str, object]:
    """Summarize whether the shared plan consumes every model advertisement."""

    incomplete: list[dict[str, str]] = []
    model_count = 0
    for provider in provider_records:
        provider_id = provider.get("id")
        models = provider.get("models")
        if not isinstance(provider_id, str) or not isinstance(models, list):
            raise TypeError("model automation provider records must contain an ID and model list")
        for model in models:
            if not isinstance(model, Mapping):
                raise TypeError("model automation provider model records must be mappings")
            model_count += 1
            advertisement = model.get("advertisement")
            model_id = model.get("id")
            if not isinstance(model_id, str) or not isinstance(advertisement, Mapping):
                raise TypeError("model automation records must contain an advertisement")
            if advertisement.get("status") != "complete":
                incomplete.append({"provider_id": provider_id, "model_id": model_id})
    return {
        "status": "complete" if not incomplete else "incomplete",
        "model_count": model_count,
        "complete_model_count": model_count - len(incomplete),
        "incomplete_models": incomplete,
        "claim_boundary": (
            "Complete means the common plan/scaffold join consumed each typed advertisement. It does not execute a "
            "runtime, controller campaign, or qualification procedure."
        ),
    }
    ####


def _advertisement_exercise_record(
    providers: ConfigurableTrajectoryProviderRegistry,
    tuning_campaigns: ControllerTuningCampaignRegistry,
    *,
    provider_id: str,
    model: TrajectoryModelMetadata,
    family_adapters: FamilyAdapterRegistry | None,
    local_controller_screens: LocalControllerScreenAdvertisementRegistry | None,
) -> dict[str, object]:
    """Exercise one advertised model through the portable planning surface.

    This is deliberately limited to discovery and authoring metadata.  It
    creates no runtime, controller campaign, or qualification result.  The
    record lets ``model assess`` distinguish a structurally valid Pydantic
    advertisement from one that the shared plan/scaffold path can actually
    consume.
    """

    try:
        plan = build_model_authoring_plan(
            providers,
            tuning_campaigns,
            provider_id,
            model.id,
            family_adapters=family_adapters,
            local_controller_screens=local_controller_screens,
        )
    except (KeyError, TypeError, ValueError) as error:
        return {
            "status": "fail",
            "sections": [],
            "findings": [f"common authoring plan failed: {error}"],
            "claim_boundary": ("An exercise failure is an advertisement integration gap. It does not identify a runtime or qualification failure."),
        }

    required_sections = {
        "data_contract": {
            "model_kind",
            "source_refs",
            "provenance",
            "properties",
            "reference_frames",
            "configuration_schema_id",
            "configuration_schema_fingerprint",
            "output_schema_id",
            "output_schema_fingerprint",
            "core_output_channels",
            "telemetry_groups",
        },
        "controller_automation": {
            "status",
            "family_adapter",
            "campaigns",
            "tuner_selection",
            "tuning_cache",
            "common_pipeline",
        },
        "navigation_automation": {"mission_template", "navigation_parameters", "policy"},
        "mode_automation": {"initialization_modes", "segment_types", "termination_modes", "control_intents"},
        "segment_automation": {"supports_custom_segments", "instances", "plain_value_api"},
        "execution_advertisement": {
            "status",
            "mission_template_id",
            "operation_records",
            "available_operations",
            "blocked_operations",
            "endpoint_maturity",
        },
        "maturity_advertisement": {"status", "family_id", "claim_boundary"},
        "focused_endpoint_verification": {
            "schema",
            "status",
            "selected_endpoint_status",
            "selected_endpoint_ids",
            "endpoints",
            "claim_boundary",
        },
    }
    findings: list[str] = []
    for section, required_fields in required_sections.items():
        payload = plan.get(section)
        if not isinstance(payload, Mapping):
            findings.append(f"{section}: missing mapping")
            continue
        missing = sorted(required_fields - set(payload))
        if missing:
            findings.append(f"{section}: missing {', '.join(missing)}")
    selection = plan.get("selection")
    if not isinstance(selection, Mapping):
        findings.append("selection: missing mapping")
    else:
        for field in ("provider_id", "model_id", "fidelity", "sources"):
            if field not in selection:
                findings.append(f"selection: missing {field}")
        if selection.get("provider_id") != provider_id:
            findings.append("selection: provider_id disagrees with assessed provider")
        if selection.get("model_id") != model.id:
            findings.append("selection: model_id disagrees with assessed model")
    workflow = plan.get("workflow")
    if not isinstance(workflow, list) or not workflow or not all(isinstance(item, str) and item for item in workflow):
        findings.append("workflow: missing actionable authoring commands")

    return {
        "status": "pass" if not findings else "fail",
        "plan_status": plan.get("status"),
        "sections": list(required_sections),
        "findings": findings,
        "claim_boundary": (
            "This exercises portable plan/scaffold metadata only. It does not run a model, synthesize a controller, or promote qualification evidence."
        ),
    }
    ####


def _controller_automation_summary(provider_records: Sequence[Mapping[str, object]]) -> dict[str, object]:
    """Summarize common controller coverage without claiming qualification.

    The detailed assessment retains every realization and fidelity record. This
    compact view lets a plug-in author immediately identify the only
    advertisement gap that should block external control automation: an
    available realization with externally supplied controls but no matching
    common tuning campaign.
    """

    advertised_fidelity_count = 0
    campaign_backed_fidelity_count = 0
    externally_controllable_fidelity_count = 0
    externally_tunable_fidelity_count = 0
    provider_managed_fidelity_count = 0
    not_applicable_fidelity_count = 0
    explicitly_blocked_fidelity_count = 0
    gaps: list[dict[str, str]] = []

    for provider in provider_records:
        provider_id = str(provider["id"])
        models = provider["models"]
        if not isinstance(models, list):
            raise TypeError("model automation assessment provider records must contain a model list")
        for model in models:
            if not isinstance(model, Mapping):
                raise TypeError("model automation assessment model records must be mappings")
            model_id = str(model["id"])
            realizations = model["realizations"]
            if not isinstance(realizations, list):
                raise TypeError("model automation assessment model records must contain realizations")
            for realization in realizations:
                if not isinstance(realization, Mapping):
                    raise TypeError("model automation assessment realization records must be mappings")
                realization_id = str(realization["id"])
                realization_status = str(realization["status"])
                control = realization["control"]
                fidelities = realization["fidelities"]
                if not isinstance(control, Mapping) or not isinstance(fidelities, list):
                    raise TypeError("model automation assessment realizations must contain control and fidelity records")
                control_status = str(control["status"])
                for fidelity in fidelities:
                    if not isinstance(fidelity, Mapping):
                        raise TypeError("model automation assessment fidelity records must be mappings")
                    advertised_fidelity_count += 1
                    fidelity_id = str(fidelity["id"])
                    automation = fidelity["controller_automation"]
                    if not isinstance(automation, Mapping):
                        raise TypeError("model automation assessment fidelity automation records must be mappings")
                    automation_status = str(automation["status"])
                    if automation_status in {"campaign_registered", "provider_managed_with_campaign"}:
                        campaign_backed_fidelity_count += 1
                    if automation_status == "provider_managed":
                        provider_managed_fidelity_count += 1
                    if automation_status == "not_applicable":
                        not_applicable_fidelity_count += 1
                    if realization_status != "available" or control_status == "blocked":
                        explicitly_blocked_fidelity_count += 1
                    if (
                        realization_status == "available"
                        and control_status == "available"
                        and automation_status in {"campaign_registered", "campaign_registration_required"}
                    ):
                        externally_controllable_fidelity_count += 1
                        if automation_status == "campaign_registered":
                            externally_tunable_fidelity_count += 1
                        else:
                            gaps.append(
                                {
                                    "provider_id": provider_id,
                                    "model_id": model_id,
                                    "realization_id": realization_id,
                                    "fidelity": fidelity_id,
                                    "automation_status": automation_status,
                                }
                            )

    return {
        "status": "complete" if not gaps else "external_control_gaps_declared",
        "advertised_fidelity_count": advertised_fidelity_count,
        "campaign_backed_fidelity_count": campaign_backed_fidelity_count,
        "externally_controllable_fidelity_count": externally_controllable_fidelity_count,
        "externally_tunable_fidelity_count": externally_tunable_fidelity_count,
        "provider_managed_fidelity_count": provider_managed_fidelity_count,
        "not_applicable_fidelity_count": not_applicable_fidelity_count,
        "explicitly_blocked_fidelity_count": explicitly_blocked_fidelity_count,
        "external_control_gaps": gaps,
        "claim_boundary": (
            "Complete means every currently available realization with externally supplied controls has a registered "
            "common tuning campaign. It does not qualify gains, lower blocked tiers, or require provider-managed and "
            "uncontrolled realizations to expose a tunable external controller."
        ),
    }
    ####


def build_model_automation_readiness_summary(assessment: Mapping[str, object]) -> dict[str, object]:
    """Condense a full assessment into one author-facing readiness inventory.

    The full assessment intentionally embeds the advertised schema, controls,
    metadata, and adapter operations for every model. This companion view
    keeps the decisions a plug-in author needs while selecting work: whether a
    realization is runnable, how its controls are supplied, whether a campaign
    owns its tuner, and which campaign/adapters are registered. It neither
    evaluates a campaign nor changes the claim boundary of the full matrix.
    """

    providers = assessment.get("providers")
    advertisement_summary = assessment.get("advertisement_readiness_summary")
    controller_summary = assessment.get("controller_readiness_summary")
    rslqr = assessment.get("rslqr")
    if not isinstance(providers, list):
        raise TypeError("model automation assessment must contain a provider list")
    if not isinstance(advertisement_summary, Mapping):
        raise TypeError("model automation assessment must contain an advertisement readiness summary")
    if not isinstance(controller_summary, Mapping):
        raise TypeError("model automation assessment must contain a controller readiness summary")
    if not isinstance(rslqr, Mapping):
        raise TypeError("model automation assessment must contain RSLQR readiness")

    provider_summaries: list[dict[str, object]] = []
    for provider in providers:
        if not isinstance(provider, Mapping):
            raise TypeError("model automation assessment providers must be mappings")
        models = provider.get("models")
        if not isinstance(models, list):
            raise TypeError("model automation assessment providers must contain a model list")
        model_summaries: list[dict[str, object]] = []
        for model in models:
            if not isinstance(model, Mapping):
                raise TypeError("model automation assessment models must be mappings")
            realizations = model.get("realizations")
            if not isinstance(realizations, list):
                raise TypeError("model automation assessment models must contain realization records")
            realization_summaries: list[dict[str, object]] = []
            for realization in realizations:
                if not isinstance(realization, Mapping):
                    raise TypeError("model automation assessment realizations must be mappings")
                control = realization.get("control")
                fidelities = realization.get("fidelities")
                if not isinstance(control, Mapping) or not isinstance(fidelities, list):
                    raise TypeError("model automation assessment realizations must contain controls and fidelities")
                fidelity_summaries: list[dict[str, object]] = []
                for fidelity in fidelities:
                    if not isinstance(fidelity, Mapping):
                        raise TypeError("model automation assessment fidelities must be mappings")
                    automation = fidelity.get("controller_automation")
                    if not isinstance(automation, Mapping):
                        raise TypeError("model automation assessment fidelities must contain controller automation")
                    adapter = automation.get("tuning_adapter")
                    if not isinstance(adapter, Mapping):
                        raise TypeError("model automation controller automation must contain tuning adapter readiness")
                    adapters = adapter.get("adapters")
                    if not isinstance(adapters, list):
                        raise TypeError("model automation tuning adapter readiness must contain adapters")
                    adapter_ids: list[str] = []
                    for adapter_record in adapters:
                        if not isinstance(adapter_record, Mapping):
                            raise TypeError("model automation tuning adapters must be mappings")
                        descriptor = adapter_record.get("descriptor")
                        if not isinstance(descriptor, Mapping):
                            raise TypeError("model automation tuning adapters must contain descriptors")
                        adapter_ids.append(str(descriptor["adapter_id"]))
                    fidelity_summaries.append(
                        {
                            "id": str(fidelity["id"]),
                            "automation_status": str(automation["status"]),
                            "campaign_ids": list(automation["campaign_ids"]),
                            "tuning_adapter_status": str(adapter["status"]),
                            "tuning_adapter_ids": adapter_ids,
                        }
                    )
                realization_summaries.append(
                    {
                        "id": str(realization["id"]),
                        "label": str(realization["label"]),
                        "status": str(realization["status"]),
                        "control_status": str(control["status"]),
                        "control_channels": list(control["channels"]),
                        "fidelities": fidelity_summaries,
                    }
                )
            model_summaries.append(
                {
                    "id": str(model["id"]),
                    "display_name": str(model["display_name"]),
                    "family_id": model["family_id"],
                    "status": str(model["status"]),
                    "advertisement_status": str(model["advertisement"]["status"]),
                    "realizations": realization_summaries,
                }
            )
        provider_summaries.append(
            {
                "id": str(provider["id"]),
                "display_name": str(provider["display_name"]),
                "models": model_summaries,
            }
        )

    return {
        "schema": "taoryx.model-automation-readiness-summary/v1",
        "advertisement_readiness_summary": dict(advertisement_summary),
        "controller_readiness_summary": dict(controller_summary),
        "rslqr": dict(rslqr),
        "providers": provider_summaries,
        "claim_boundary": (
            "This compact inventory reports the advertised control and tuning seam for each realization. "
            "It does not execute a campaign, qualify a gain, lower a fidelity, or make RSLQR available."
        ),
    }
    ####


def _automation_realization_record(
    *,
    provider_id: str,
    model: TrajectoryModelMetadata,
    realization: TrajectoryRealizationMetadata,
    tuning_campaigns: ControllerTuningCampaignRegistry,
    family_adapters: FamilyAdapterRegistry | None,
    local_controller_screens: LocalControllerScreenAdvertisementRegistry | None,
) -> dict[str, object]:
    """Project one realization into the non-executable readiness matrix."""

    fidelity_records: list[dict[str, object]] = []
    for fidelity in realization.fidelity_aliases:
        adapter_record = _family_adapter_record(
            family_adapters,
            family_id=model.family_id,
            fidelity=fidelity,
        )
        campaigns = tuning_campaigns.matching(
            provider_id=provider_id,
            model_id=model.id,
            fidelity=fidelity,
            realization_id=realization.id,
        )
        selection = ModelAuthoringSelection(
            provider_id=provider_id,
            model=model,
            fidelity=fidelity,
            realization=realization,
            mission=None,
            sources={"fidelity": "advertised_realization_alias", "realization": "explicit"},
        )
        controller = _controller_plan(
            selection,
            campaigns,
            adapter_record,
            local_controller_screens=local_controller_screens,
        )
        fidelity_records.append(
            {
                "id": fidelity,
                "family_adapter": adapter_record,
                "controller_automation": {
                    "status": controller["status"],
                    "campaign_ids": [item.id for item in campaigns],
                    "tuning_adapter": controller["tuning_adapter"],
                    "common_pipeline": controller["common_pipeline"],
                    "claim_boundary": (
                        "Registered means the plug-in has opted into the common local tuning campaign; "
                        "it does not establish controller qualification or an operating envelope."
                    ),
                },
            }
        )

    controls = realization.controls
    blockers = list(realization.blockers)
    if realization.status != "available":
        blockers.append(f"realization status is {realization.status}")
    for record in fidelity_records:
        controller_record = record["controller_automation"]
        assert isinstance(controller_record, dict)
        if controller_record["status"] == "campaign_registration_required":
            blockers.append(f"{record['id']}: no common tuning campaign is registered")
        adapter = record["family_adapter"]
        # A matching tuning campaign owns its own tier-specific adapter.  The
        # broader family adapter is still useful execution metadata, but its
        # supported tiers must not incorrectly block a registered lower-tier
        # guidance or response-law campaign.
        if adapter is None and not controller_record["campaign_ids"]:
            blockers.append(f"{record['id']}: no family adapter registration")
        elif isinstance(adapter, dict) and not adapter["fidelity_supported"] and not controller_record["campaign_ids"]:
            blockers.append(f"{record['id']}: family adapter does not support this fidelity")

    return {
        "id": realization.id,
        "label": realization.label,
        "status": realization.status,
        "operations": list(realization.operations),
        "dynamics_fidelities": list(realization.dynamics_fidelities),
        "fidelity_aliases": list(realization.fidelity_aliases),
        "input_realization": realization.input_realization,
        "actuator_types": list(realization.actuator_types),
        "actuation_class": _actuation_class(realization),
        "control": {
            "status": controls.status,
            "channel_count": len(controls.channels),
            "channels": [item.id for item in controls.channels],
            "authority_ids": [item.id for item in controls.authorities],
            "control_scheme_ids": list(dict.fromkeys(item.scheme_id for item in model.control_scheme_support if item.realization_id == realization.id)),
            "intent_ids": [item.id for item in controls.intents],
            "default_authority_id": controls.default_authority_id,
        },
        "lowering": {
            "status": "not_advertised",
            "fidelity_selection": "preserved",
            "reason": (
                "The common authoring layer preserves an explicitly selected advertised fidelity. "
                "Automatic lowering requires separate executable evidence and is not inferred here."
            ),
        },
        "fidelities": fidelity_records,
        "blockers": list(dict.fromkeys(blockers)),
        "claim_boundary": realization.claim_boundary,
    }
    ####


def _actuation_class(realization: TrajectoryRealizationMetadata) -> str:
    """Name the advertised actuation path without overclaiming its behavior."""

    if realization.input_realization == "actuator_allocated":
        if "rigid_body_6dof" in realization.dynamics_fidelities:
            return "actuated_6dof"
        return "actuator_allocated"
    if realization.input_realization == "direct_wrench":
        return "direct_wrench"
    if realization.input_realization == "guidance_command":
        return "guidance_command"
    return realization.input_realization
    ####


def load_model_authoring_draft(path: str | Path) -> ModelAuthoringDraft:
    """Load one JSON or YAML draft."""

    draft_path = Path(path)
    payload = (
        json.loads(draft_path.read_text(encoding="utf-8"))
        if draft_path.suffix.casefold() == ".json"
        else yaml.safe_load(draft_path.read_text(encoding="utf-8"))
    )
    return ModelAuthoringDraft.model_validate(payload)
    ####


def write_model_authoring_draft(draft: ModelAuthoringDraft, path: str | Path) -> Path:
    """Write an editable JSON or YAML draft without derived diagnostics."""

    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = draft.model_dump(mode="json", by_alias=True)
    if output.suffix.casefold() == ".json":
        output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    else:
        output.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return output
    ####


def _first_available_mission_for_fidelity(
    model: TrajectoryModelMetadata,
    fidelity: str,
) -> TrajectoryMissionTemplateMetadata | None:
    """Choose the first runnable mission compatible with an explicit fidelity.

    A presentation default is a preferred pairing, not permission to attach a
    physical controller screen to a caller-selected reduced fidelity.  Retain
    the caller's tier and select its first runnable mission, falling back to a
    compatible declared mission only when no runtime endpoint is advertised.
    """

    candidates = tuple(item for item in model.mission_templates if fidelity in item.compatible_fidelities)
    if not candidates:
        return None
    for mission in candidates:
        if any(item.fidelity == fidelity and item.operation in {"batch", "step"} and item.status == "available" for item in mission.operations):
            return mission
    return candidates[0]
    ####


def _resolve_mission(
    model: TrajectoryModelMetadata,
    mission_template_id: str | None,
    sources: dict[str, str],
) -> TrajectoryMissionTemplateMetadata | None:
    missions = {item.id: item for item in model.mission_templates}
    selected_id = mission_template_id
    if selected_id is not None:
        sources["mission"] = "caller"
    elif model.presentation.default_mission_template_id is not None:
        selected_id = model.presentation.default_mission_template_id
        sources["mission"] = "advertised_presentation_default"
    elif len(missions) == 1:
        selected_id = next(iter(missions))
        sources["mission"] = "only_advertised_mission"
    else:
        sources["mission"] = "selection_required" if missions else "not_applicable"
        return None
    try:
        return missions[selected_id]
    except KeyError as error:
        raise ModelAuthoringError(
            "unknown-mission-template",
            f"expected one of {sorted(missions)!r}",
            path="mission_template_id",
        ) from error
    ####


def _resolve_realization(
    model: TrajectoryModelMetadata,
    fidelity: str,
    realization_id: str | None,
    mission: TrajectoryMissionTemplateMetadata | None,
    sources: dict[str, str],
    *,
    allow_blocked_local_design: bool = False,
) -> TrajectoryRealizationMetadata | None:
    realizations = {item.id: item for item in model.realizations}
    if realization_id is not None:
        try:
            realization = realizations[realization_id]
        except KeyError as error:
            raise ModelAuthoringError(
                "unknown-realization",
                f"expected one of {sorted(realizations)!r}",
                path="realization_id",
            ) from error
        _validate_realization_selection(
            realization,
            fidelity,
            mission,
            allow_blocked_local_design=allow_blocked_local_design,
        )
        sources["realization"] = "caller"
        if realization.status != "available":
            sources["realization_availability"] = "blocked_local_design_allowed_for_registered_campaign"
        return realization
    candidates = tuple(
        item
        for item in model.realizations
        if item.status == "available"
        and fidelity in item.fidelity_aliases
        and (mission is None or not item.mission_template_ids or mission.id in item.mission_template_ids)
    )
    exact = tuple(item for item in candidates if item.id == fidelity)
    if len(exact) == 1:
        sources["realization"] = "advertised_fidelity_identity"
        return exact[0]
    if len(candidates) == 1:
        sources["realization"] = "only_compatible_realization"
        return candidates[0]
    sources["realization"] = "selection_required"
    return None
    ####


def _validate_realization_selection(
    realization: TrajectoryRealizationMetadata,
    fidelity: str,
    mission: TrajectoryMissionTemplateMetadata | None,
    *,
    allow_blocked_local_design: bool = False,
) -> None:
    if realization.status != "available" and not (allow_blocked_local_design and realization.status == "blocked"):
        raise ModelAuthoringError(
            "realization-unavailable",
            f"realization status is {realization.status!r}: {list(realization.blockers)!r}",
            path="realization_id",
        )
    if fidelity not in realization.fidelity_aliases:
        raise ModelAuthoringError(
            "realization-fidelity-mismatch",
            f"realization supports {list(realization.fidelity_aliases)!r}",
            path="realization_id",
        )
    if mission is not None and realization.mission_template_ids and mission.id not in realization.mission_template_ids:
        raise ModelAuthoringError(
            "realization-mission-mismatch",
            f"realization supports missions {list(realization.mission_template_ids)!r}",
            path="realization_id",
        )
    ####


_OMIT = object()


def _scaffold_node(
    node: ConfigurationNode,
    *,
    fidelity: str,
    mission: TrajectoryMissionTemplateMetadata | None,
) -> Any:
    if isinstance(node, ConfigurationParameterSchema):
        if node.default_declared:
            return node.default
        return REQUIRED_VALUE if node.required else _OMIT
    if isinstance(node, ConfigurationGroupSchema):
        values: dict[str, Any] = {}
        for child in node.children:
            value = _scaffold_node(child, fidelity=fidelity, mission=mission)
            if value is not _OMIT:
                values[child.id] = value
        return values
    if isinstance(node, ConfigurationChoiceSchema):
        compatible = tuple(item for item in node.variants if not item.compatible_fidelities or fidelity in item.compatible_fidelities)
        preferred: tuple[str, ...] = ()
        if mission is not None and node.id in {"initialization", "initial_state", "launch_state"}:
            preferred = tuple(item for item in mission.initialization_variants if item in {variant.id for variant in compatible})
        selected = preferred[0] if len(preferred) == 1 else (compatible[0].id if len(compatible) == 1 else SELECTION_REQUIRED)
        if selected == SELECTION_REQUIRED:
            return {
                "selected": f"{SELECTION_REQUIRED}:{'|'.join(item.id for item in compatible)}",
                "values": {},
            }
        variant = next(item for item in compatible if item.id == selected)
        return {
            "selected": selected,
            "values": _scaffold_node(variant.node, fidelity=fidelity, mission=mission),
        }
    if isinstance(node, ConfigurationSequenceSchema):
        template = _select_sequence_template(node, fidelity=fidelity, mission=mission)
        if template is None:
            choices = "|".join(item.id for item in node.templates)
            return {
                "template": f"{SELECTION_REQUIRED}:{choices}" if choices else None,
                "items": [],
            }
        if not isinstance(node.item, ConfigurationChoiceSchema):
            raise ModelAuthoringError(
                "unsupported-template-shape",
                "an advertised sequence template requires choice items",
                path=node.id,
            )
        variants = {item.id: item for item in node.item.variants}
        items = [_scaffold_node(variants[variant_id].node, fidelity=fidelity, mission=mission) for variant_id in template.item_variants]
        return {"template": template.id, "items": items}
    if isinstance(node, ConfigurationOptionalSchema):
        return None
    raise TypeError(f"unsupported configuration node {type(node).__name__}")
    ####


def _select_sequence_template(
    node: ConfigurationSequenceSchema,
    *,
    fidelity: str,
    mission: TrajectoryMissionTemplateMetadata | None,
) -> ConfigurationSequenceTemplate | None:
    compatible = tuple(item for item in node.templates if not item.compatible_fidelities or fidelity in item.compatible_fidelities)
    if mission is not None:
        by_id = tuple(item for item in compatible if item.id == mission.id)
        if len(by_id) == 1:
            return by_id[0]
        by_sequence = tuple(item for item in compatible if item.item_variants == mission.segment_sequence)
        if len(by_sequence) == 1:
            return by_sequence[0]
    return compatible[0] if len(compatible) == 1 else None
    ####


def _compile_node(
    node: ConfigurationNode,
    raw: Any,
    *,
    fidelity: str,
    path: str,
) -> ConfigurationNodeValue:
    if isinstance(node, ConfigurationParameterSchema):
        return ConfigurationParameterValue(value=raw, unit=node.canonical_unit)
    if isinstance(node, ConfigurationGroupSchema):
        if not isinstance(raw, Mapping):
            raise ModelAuthoringError("invalid-group", "expected a mapping", path=path)
        children = {item.id: item for item in node.children}
        unknown = sorted(str(item) for item in set(raw) - set(children))
        if unknown:
            raise ModelAuthoringError("unknown-field", f"group does not advertise {unknown!r}", path=path)
        return ConfigurationGroupValue(
            values={
                str(identifier): _compile_node(
                    children[str(identifier)],
                    value,
                    fidelity=fidelity,
                    path=f"{path}.{identifier}",
                )
                for identifier, value in raw.items()
            }
        )
    if isinstance(node, ConfigurationChoiceSchema):
        if not isinstance(raw, Mapping):
            raise ModelAuthoringError("invalid-choice", "expected selected/values mapping", path=path)
        selected = raw.get("selected")
        if not isinstance(selected, str):
            raise ModelAuthoringError("choice-selection-required", "choice requires string selected", path=path)
        variant = next((item for item in node.variants if item.id == selected), None)
        if variant is None:
            raise ModelAuthoringError(
                "unknown-choice",
                f"expected one of {sorted(item.id for item in node.variants)!r}",
                path=f"{path}.selected",
            )
        return ConfigurationChoiceValue(
            selected=selected,
            instance_id=_optional_string(raw.get("instance_id"), path=f"{path}.instance_id"),
            value=_compile_node(
                variant.node,
                raw.get("values", {}),
                fidelity=fidelity,
                path=f"{path}.{selected}",
            ),
        )
    if isinstance(node, ConfigurationSequenceSchema):
        return _compile_sequence(node, raw, fidelity=fidelity, path=path)
    if isinstance(node, ConfigurationOptionalSchema):
        if raw is None:
            return ConfigurationOptionalValue(enabled=False)
        return ConfigurationOptionalValue(
            enabled=True,
            value=_compile_node(node.item, raw, fidelity=fidelity, path=f"{path}.value"),
        )
    raise TypeError(f"unsupported configuration node {type(node).__name__}")
    ####


def _compile_sequence(
    node: ConfigurationSequenceSchema,
    raw: Any,
    *,
    fidelity: str,
    path: str,
) -> ConfigurationSequenceValue:
    if not isinstance(raw, Mapping):
        raise ModelAuthoringError("invalid-sequence", "expected template/items mapping", path=path)
    raw_items = raw.get("items")
    if not isinstance(raw_items, Sequence) or isinstance(raw_items, str | bytes):
        raise ModelAuthoringError("invalid-sequence", "items must be a list", path=f"{path}.items")
    template_id = raw.get("template")
    if template_id is None:
        return ConfigurationSequenceValue(
            items=tuple(_compile_node(node.item, item, fidelity=fidelity, path=f"{path}.items[{index}]") for index, item in enumerate(raw_items))
        )
    if not isinstance(template_id, str):
        raise ModelAuthoringError("invalid-template", "template must be a string", path=f"{path}.template")
    template = next((item for item in node.templates if item.id == template_id), None)
    if template is None:
        raise ModelAuthoringError(
            "unknown-template",
            f"expected one of {sorted(item.id for item in node.templates)!r}",
            path=f"{path}.template",
        )
    if template.compatible_fidelities and fidelity not in template.compatible_fidelities:
        raise ModelAuthoringError(
            "incompatible-template",
            f"template supports {list(template.compatible_fidelities)!r}",
            path=f"{path}.template",
        )
    if len(raw_items) != len(template.item_variants):
        raise ModelAuthoringError(
            "template-item-count",
            f"template requires {len(template.item_variants)} items, received {len(raw_items)}",
            path=f"{path}.items",
        )
    if not isinstance(node.item, ConfigurationChoiceSchema):
        raise ModelAuthoringError(
            "unsupported-template-shape",
            "templated sequence item is not a choice",
            path=path,
        )
    variants = {item.id: item for item in node.item.variants}
    items: list[ConfigurationChoiceValue] = []
    for index, (variant_id, item_raw) in enumerate(zip(template.item_variants, raw_items, strict=True), start=1):
        variant = variants[variant_id]
        values = item_raw
        instance_id = f"{index:02d}-{variant_id}"
        if isinstance(item_raw, Mapping) and "selected" in item_raw:
            if item_raw.get("selected") != variant_id:
                raise ModelAuthoringError(
                    "template-choice-mismatch",
                    f"template requires {variant_id!r}",
                    path=f"{path}.items[{index - 1}].selected",
                )
            values = item_raw.get("values", {})
            instance_id = (
                _optional_string(
                    item_raw.get("instance_id"),
                    path=f"{path}.items[{index - 1}].instance_id",
                )
                or instance_id
            )
        items.append(
            ConfigurationChoiceValue(
                selected=variant_id,
                instance_id=instance_id,
                value=_compile_node(
                    variant.node,
                    values,
                    fidelity=fidelity,
                    path=f"{path}.items[{index - 1}].{variant_id}",
                ),
            )
        )
    return ConfigurationSequenceValue(items=tuple(items))
    ####


def _optional_string(value: object, *, path: str) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str) or not value.strip():
        raise ModelAuthoringError("invalid-string", "expected a non-empty string", path=path)
    return value
    ####


def _collect_placeholders(value: Any, *, path: str, result: list[str]) -> None:
    if isinstance(value, str) and (value == REQUIRED_VALUE or value == SELECTION_REQUIRED or value.startswith(f"{SELECTION_REQUIRED}:")):
        result.append(path)
        return
    if isinstance(value, Mapping):
        for key, item in value.items():
            _collect_placeholders(item, path=f"{path}.{key}", result=result)
    elif isinstance(value, Sequence) and not isinstance(value, str | bytes):
        for index, item in enumerate(value):
            _collect_placeholders(item, path=f"{path}[{index}]", result=result)
    ####


def _parameter_records(node: ConfigurationNode, *, path: str = "values") -> list[dict[str, object]]:
    records: list[dict[str, object]] = []
    if isinstance(node, ConfigurationParameterSchema):
        payload = node.model_dump(mode="json")
        payload["path"] = path
        records.append(payload)
    elif isinstance(node, ConfigurationGroupSchema):
        for child in node.children:
            records.extend(_parameter_records(child, path=f"{path}.{child.id}"))
    elif isinstance(node, ConfigurationChoiceSchema):
        for variant in node.variants:
            records.extend(_parameter_records(variant.node, path=f"{path}.{node.id}.{variant.id}"))
    elif isinstance(node, ConfigurationSequenceSchema):
        records.extend(_parameter_records(node.item, path=f"{path}.{node.id}[]"))
    elif isinstance(node, ConfigurationOptionalSchema):
        records.extend(_parameter_records(node.item, path=f"{path}.{node.id}"))
    return records
    ####


def _is_navigation_parameter(record: Mapping[str, object]) -> bool:
    identifier = str(record.get("id", "")).casefold()
    control = ""
    presentation = record.get("presentation")
    if isinstance(presentation, Mapping):
        control = str(presentation.get("control", ""))
    tokens = (
        "waypoint",
        "target",
        "gate",
        "position",
        "north",
        "east",
        "altitude",
        "heading",
        "course",
        "range",
        "crossrange",
        "latitude",
        "longitude",
        "location",
    )
    return control == "coordinate_picker" or any(token in identifier for token in tokens)
    ####


def _segment_instances(
    root: ConfigurationNode,
    mission: TrajectoryMissionTemplateMetadata | None,
) -> list[dict[str, object]]:
    sequence = _find_sequence(root, "segments")
    if sequence is None:
        return []
    variants = {item.id: item for item in sequence.item.variants} if isinstance(sequence.item, ConfigurationChoiceSchema) else {}
    selected = mission.segment_sequence if mission is not None else ()
    return [
        {
            "occurrence_index": index,
            "instance_id": f"{index:02d}-{segment_id}",
            "segment_id": segment_id,
            "parameters": (_parameter_records(variants[segment_id].node, path=f"segments[{index - 1}]") if segment_id in variants else []),
        }
        for index, segment_id in enumerate(selected, start=1)
    ]
    ####


def _find_sequence(node: ConfigurationNode, identifier: str) -> ConfigurationSequenceSchema | None:
    if isinstance(node, ConfigurationSequenceSchema) and node.id == identifier:
        return node
    if isinstance(node, ConfigurationGroupSchema):
        for child in node.children:
            match = _find_sequence(child, identifier)
            if match is not None:
                return match
    if isinstance(node, ConfigurationChoiceSchema):
        for variant in node.variants:
            match = _find_sequence(variant.node, identifier)
            if match is not None:
                return match
    if isinstance(node, ConfigurationSequenceSchema):
        return _find_sequence(node.item, identifier)
    if isinstance(node, ConfigurationOptionalSchema):
        return _find_sequence(node.item, identifier)
    return None
    ####


def _family_adapter_record(
    registry: FamilyAdapterRegistry | None,
    *,
    family_id: str | None,
    fidelity: str,
) -> dict[str, object] | None:
    if registry is None or family_id is None:
        return None
    match = next((item for item in registry.registrations if item.family_id == family_id), None)
    if match is None:
        return None
    return {
        "family_id": match.family_id,
        "adapter_id": match.adapter_id,
        "status": match.status,
        "fidelity_supported": fidelity in match.supported_tiers,
        "supported_tiers": list(match.supported_tiers),
        "note": match.note,
    }
    ####


def _controller_plan(
    selection: ModelAuthoringSelection,
    campaigns: Sequence[ControllerTuningCampaignRegistration],
    adapter_record: dict[str, object] | None,
    *,
    local_controller_screens: LocalControllerScreenAdvertisementRegistry | None,
) -> dict[str, object]:
    realization = selection.realization
    if realization is None:
        return {
            "status": "realization_selection_required",
            "common_pipeline": [],
            "family_adapter": adapter_record,
            "campaigns": [],
            "tuner_selection": _tuner_selection_advertisement(()),
            "tuning_cache": _tuning_cache_advertisement(()),
            "control_scheme_support": [],
        }
    controls = realization.controls
    default_authority = next(
        (item for item in controls.authorities if item.id == controls.default_authority_id),
        None,
    )
    physical_model_property = next(
        (item for item in selection.model.presentation.properties if item.id == "physical_model"),
        None,
    )
    explicitly_nonphysical = physical_model_property is not None and physical_model_property.value_declared and physical_model_property.value is False
    if realization.input_realization in {"uncontrolled", "source_replay"}:
        status = "not_applicable"
        pipeline = ["declare open-loop or replay semantics", "evaluate truth objectives"]
    elif explicitly_nonphysical:
        status = "not_applicable"
        pipeline = [
            "exercise typed control transport and lowering",
            "retain the non-physical debug claim boundary",
        ]
    elif controls.status == "internally_generated" or (default_authority is not None and default_authority.command_owner != "caller"):
        status = "provider_managed_with_campaign" if campaigns else "provider_managed"
        pipeline = [
            "resolve mission capability and route",
            "exercise provider-internal control intent",
            "evaluate commanded-versus-achieved telemetry",
        ]
        if campaigns:
            pipeline.extend(
                (
                    "run the optional source-local tuning campaign",
                    "retain provider-owned execution and qualification boundaries",
                )
            )
    else:
        status = "campaign_registered" if campaigns else "campaign_registration_required"
        pipeline = [
            "declare operating points and canonical scales",
            "trim",
            "two-step derivative consistency",
            "authority preflight",
            "normalized controller candidate grid",
            "nonlinear response screen",
            "allocation and actuator screen when applicable",
            "mission truth-objective gates",
        ]
    public_campaigns = [item.public_dict() for item in campaigns]
    campaign_ids = [item.id for item in campaigns]
    direct_wrench_screen = resolve_local_direct_wrench_screen_advertisement(
        family_id=selection.model.family_id,
        mission_id=selection.mission.id if selection.mission is not None else None,
        fidelity=selection.fidelity,
    )
    campaign_screens = tuple(
        screen
        for campaign in campaigns
        for screen in campaign.local_controller_screen_advertisements(selection.mission.id if selection.mission is not None else None)
    )
    if len(campaign_screens) > 1:
        raise RuntimeError("one selected composition endpoint has multiple campaign local-controller-screen advertisements")
    registered_screen = campaign_screens[0] if campaign_screens else None
    plug_in_screens = (
        ()
        if local_controller_screens is None
        else local_controller_screens.matching(
            provider_id=selection.provider_id,
            model_id=selection.model.id,
            family_id=selection.model.family_id,
            fidelity=selection.fidelity,
            realization_id=selection.realization.id if selection.realization is not None else None,
            mission_template_id=selection.mission.id if selection.mission is not None else None,
        )
    )
    if len(plug_in_screens) > 1:
        raise RuntimeError("one selected composition endpoint has multiple plug-in local-controller-screen advertisements")
    plug_in_screen = plug_in_screens[0].public_advertisement() if plug_in_screens else None
    screens = tuple(item for item in (direct_wrench_screen, registered_screen, plug_in_screen) if item is not None)
    if len(screens) > 1:
        raise RuntimeError("one selected composition endpoint has multiple local-controller-screen advertisements")
    has_live_switchable_profiles = (
        sum(item.availability in {"available", "available_in_batch"} and item.switching_policy == "explicit_bumpless" for item in controls.authorities) > 1
    )
    default_channel_ids = (
        {item.id for item in controls.channels} if default_authority is None or not has_live_switchable_profiles else set(default_authority.channel_ids)
    )
    default_channels = tuple(item for item in controls.channels if item.id in default_channel_ids)
    control_scheme_support = tuple(
        item for item in selection.model.control_scheme_support if item.realization_id == realization.id and selection.fidelity in item.fidelity_ids
    )
    return {
        "status": status,
        "input_realization": realization.input_realization,
        "control_status": controls.status,
        # For a live-switchable multi-profile realization, ``channels`` is the
        # immediately usable default projection. Legacy locked realizations
        # retain their complete realization catalog here for compatibility.
        # In both cases discovery clients can inspect every coordinate through
        # ``available_channels`` and join it to ``authorities`` below.
        "channels": [item.model_dump(mode="json") for item in default_channels],
        "available_channels": [item.model_dump(mode="json") for item in controls.channels],
        "default_authority_id": controls.default_authority_id,
        "channel_projection": ("default_authority" if default_authority is not None and has_live_switchable_profiles else "complete_realization"),
        "authorities": [item.model_dump(mode="json") for item in controls.authorities],
        "control_scheme_support": [item.model_dump(mode="json") for item in control_scheme_support],
        "intents": [item.model_dump(mode="json") for item in controls.intents],
        "family_adapter": adapter_record,
        "tuning_adapter": {
            "status": "campaign_owned" if campaigns else "not_registered",
            "campaign_ids": campaign_ids,
            "adapters": [item.adapter_advertisement() for item in campaigns],
            "claim_boundary": (
                "When registered, the campaign constructs and validates its own exact tier-specific adapter at tune time. "
                "The general family-adapter record remains an execution-capability advertisement and does not replace that adapter."
            ),
        },
        "campaigns": public_campaigns,
        "tuner_selection": _tuner_selection_advertisement(campaign_ids),
        "tuning_cache": _tuning_cache_advertisement(campaign_ids),
        "local_controller_screen": screens[0] if screens else None,
        "common_pipeline": pipeline,
        "non_tunable_rule": (
            "Missing authority, effectors, frames, trim, or state derivatives are model-integration blockers; "
            "the tuner must not compensate for them with gains."
        ),
    }
    ####


def _tuner_selection_advertisement(campaign_ids: Sequence[str]) -> dict[str, object]:
    """Describe the explicit, non-heuristic campaign selection rule."""

    identifiers = list(campaign_ids)
    return {
        "status": "available" if identifiers else "not_available",
        "campaign_ids": identifiers,
        "cli_argument": "--campaign <campaign-id>",
        "selection_rule": (
            "The sole matching campaign is inferred. When multiple campaigns match the selected model, "
            "fidelity, realization, and mission, callers must provide --campaign; core never chooses by name, "
            "method, or apparent controller quality."
            if identifiers
            else "No common controller-tuning campaign matches this exact selection."
        ),
        "claim_boundary": (
            "Campaign selection chooses a declared local design screen only. It does not select a runtime controller "
            "for a mission, create an operating-point schedule, or establish qualification."
        ),
    }
    ####


def _tuning_cache_advertisement(campaign_ids: Sequence[str]) -> dict[str, object]:
    """Expose the common tuning-result reuse contract to every authoring plan."""

    identifiers = list(campaign_ids)
    return {
        "schema": "taoryx.controller-tuning-cache/v1alpha1",
        "status": "available" if identifiers else "not_available",
        "campaign_ids": identifiers,
        "default_directory": "build/controller-cache",
        "cli_options": {
            "directory": "--cache-dir <path>",
            "bypass": "--no-cache",
        },
        "identity_inputs": [
            "installed_plugin_catalog_fingerprint",
            "campaign_registration_advertisement",
            "campaign_adapter_descriptor",
            "campaign_declaration",
        ],
        "reuse_rule": (
            "The host reuses a report only when all content-addressed identity inputs match; otherwise it reruns the "
            "declared campaign and atomically replaces that exact cache artifact."
            if identifiers
            else "No cache artifact is applicable because this exact selection has no registered campaign."
        ),
        "claim_boundary": (
            "A cache hit reuses local candidate-design evidence only. It does not skip endpoint preflight, nonlinear "
            "validation, allocation, mission truth-objective checks, or qualification gates."
        ),
    }
    ####


def __getattr__(name: str) -> object:
    """Resolve the historical maturity-registry path only for compatibility users."""

    if name != "VEHICLE_MATURITY_REGISTRY":
        raise AttributeError(name)
    from .compatibility.vehicle_catalog_resources import legacy_vehicle_catalog_resource

    value = legacy_vehicle_catalog_resource("verification/vehicle_maturity_registry.yaml")
    globals()[name] = value
    return value
    ####


__all__ = [
    "AUTHORING_DRAFT_SCHEMA",
    "AuthoringPlanRecord",
    "ModelAuthoringDraft",
    "ModelAuthoringError",
    "ModelAuthoringPlanProjection",
    "ModelAuthoringSelection",
    "ModelAuthoringSelectionProjection",
    "REQUIRED_VALUE",
    "SELECTION_REQUIRED",
    "author_configuration",
    "build_model_automation_assessment",
    "build_model_automation_readiness_summary",
    "build_model_authoring_plan",
    "compile_model_authoring_draft",
    "custom_sequence",
    "load_model_authoring_draft",
    "resolve_model_authoring_selection",
    "scaffold_model_authoring_draft",
    "segment_occurrence",
    "select_variant",
    "sequence_template",
    "write_model_authoring_draft",
]
####
