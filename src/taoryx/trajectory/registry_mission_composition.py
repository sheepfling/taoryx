"""Registry-backed Mission Composition discovery and configuration provider.

This module projects the canonical vehicle-composition, fidelity, value-space,
and execution-binding authorities into the portable provider configuration
contract.  It does not create another vehicle catalog and it does not execute
or qualify a trajectory.
"""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Mapping, Sequence
from dataclasses import replace
from typing import TYPE_CHECKING, Literal, cast

from ..episode_channel_value_space import episode_channel_value_space
from ..fidelity_contracts import CANONICAL_FIDELITY_TIERS
from ..value_space import ValueSpaceSpec, default_value_space_for_value_type
from ..vehicle_composition_registry import (
    ParameterSpec,
    ResolvedVehicleComposition,
    ResolvedVehicleCompositionCatalog,
    VariantParameterBinding,
    load_resolved_vehicle_composition_catalog,
)
from ..vehicle_interface import InterfaceChannel, interface_contract_for_composition
from .configuration_contract import (
    ConfigurationBound,
    ConfigurationChoiceSchema,
    ConfigurationChoiceVariant,
    ConfigurationContractError,
    ConfigurationGroupSchema,
    ConfigurationInterval,
    ConfigurationOptionalSchema,
    ConfigurationParameterSchema,
    ConfigurationPeriodicity,
    ConfigurationSequenceSchema,
    ConfigurationSequenceTemplate,
    ConfigurationValueSpace,
    PreparedTrajectoryConfiguration,
    TrajectoryConfigurationInstance,
    TrajectoryConfigurationSchema,
    TrajectoryControlAdvertisement,
    TrajectoryControlAuthorityMetadata,
    TrajectoryControlAvailability,
    TrajectoryControlChannelMetadata,
    TrajectoryControlIntentMetadata,
    TrajectoryControlIntentResolution,
    TrajectoryControlNativeBindingMetadata,
    TrajectoryControlStatus,
    TrajectoryDeploymentMetadata,
    TrajectoryEntityOutputMetadata,
    TrajectoryFidelityMetadata,
    TrajectoryFidelityTransition,
    TrajectoryMissionOperationMetadata,
    TrajectoryMissionTemplateMetadata,
    TrajectoryModelCapabilities,
    TrajectoryModelMetadata,
    TrajectoryModelPresentationMetadata,
    TrajectoryModelPropertyMetadata,
    TrajectoryOutputChannelMetadata,
    TrajectoryOutputSchema,
    TrajectoryProviderMetadata,
    TrajectoryProviderPresentationMetadata,
    TrajectoryRealizationMetadata,
    TrajectoryReferenceFrameMetadata,
    TrajectoryTelemetryGroupMetadata,
    ValuePresentationMetadata,
    validate_configuration_instance,
)
from .execution_contract import MissionCompositionRunnerRegistry
from .native_output_contract import (
    native_output_bindings,
    native_output_channel_metadata,
    native_output_reference_frames,
)

if TYPE_CHECKING:
    from ..plugins.discovery import PluginCatalog

_EXECUTOR_ID = "taoryx.registry.native-batch.v1"
_SESSION_EXECUTOR_ID = "taoryx.registry.native-session.v1"
_FIDELITY_LABELS = {
    "point_mass_3dof": "Point-Mass 3-DOF",
    "pseudo_6dof": "Pseudo 6-DOF",
    "rigid_body_6dof_direct_wrench": "Rigid-Body 6-DOF, Direct Wrench",
    "rigid_body_6dof_surface_allocated": "Rigid-Body 6-DOF, Surface Allocated",
}
_QUANTITY_BY_UNIT = {
    "dimensionless": "dimensionless",
    "m": "length",
    "m/s": "speed",
    "m/s^2": "acceleration",
    "s": "time",
    "deg": "angle",
    "rad": "angle",
    "deg/s": "angular_rate",
    "rad/s": "angular_rate",
    "kg": "mass",
    "Pa": "pressure",
    "J/kg": "specific_energy",
    "g": "load_factor",
    "N": "force",
    "N*m": "moment",
}


def _scope_catalog_mission_templates(
    catalog: ResolvedVehicleCompositionCatalog,
    allowed_mission_template_ids: Mapping[str, Sequence[str]],
) -> ResolvedVehicleCompositionCatalog:
    """Return a provider-local view with only explicitly owned mission endpoints.

    A family package can retain source declarations for an optional overlay
    owned by another plug-in.  Its focused provider must not expose the
    overlay's initialization, segment, or mission choices unless that owner
    is part of the selected provider surface.  The aggregate provider leaves
    this argument unset and therefore retains the complete canonical view.
    """

    known_families = {item.family.family_id for item in catalog.vehicles}
    unknown_families = sorted(set(allowed_mission_template_ids) - known_families)
    if unknown_families:
        raise ValueError(f"mission-template scope names unknown families: {unknown_families}")

    scoped: list[ResolvedVehicleComposition] = []
    for vehicle in catalog.vehicles:
        selected_ids = allowed_mission_template_ids.get(vehicle.family.family_id)
        if selected_ids is None:
            scoped.append(vehicle)
            continue
        identifiers = (selected_ids,) if isinstance(selected_ids, str) else selected_ids
        allowed_ids = frozenset(str(item).strip() for item in identifiers if str(item).strip())
        if not allowed_ids:
            raise ValueError(f"mission-template scope for {vehicle.family.family_id!r} must not be empty")
        declared_ids = {item.id for item in vehicle.declaration.mission_templates}
        unknown_ids = sorted(allowed_ids - declared_ids)
        if unknown_ids:
            raise ValueError(
                f"mission-template scope for {vehicle.family.family_id!r} names undeclared missions: {unknown_ids}"
            )
        missions = tuple(item for item in vehicle.declaration.mission_templates if item.id in allowed_ids)
        initialization_ids = {identifier for mission in missions for identifier in mission.initialization_contracts}
        segment_ids = {identifier for mission in missions for identifier in mission.segment_sequence}
        compatible_fidelities = {fidelity for mission in missions for fidelity in mission.compatible_fidelities}
        declaration = vehicle.declaration.model_copy(
            update={
                "initialization_contracts": tuple(
                    item for item in vehicle.declaration.initialization_contracts if item.id in initialization_ids
                ),
                "segment_contracts": tuple(item for item in vehicle.declaration.segment_contracts if item.id in segment_ids),
                "mission_templates": missions,
                "variant_parameters": tuple(
                    item
                    for item in vehicle.declaration.variant_parameters
                    if item.target_initialization_id in initialization_ids
                    and bool(set(item.compatible_fidelities) & compatible_fidelities)
                ),
            }
        )
        scoped.append(replace(vehicle, declaration=declaration))
    return ResolvedVehicleCompositionCatalog(tuple(scoped))
    ####


class RegistryMissionCompositionProvider:
    """Compatibility aggregate provider for canonical families and workflows.

    Discovery, validation, registered batch operations, and exact native
    episodes are exposed through the common runner and stateful session APIs.
    New direct vehicle packages must use ``CatalogMissionCompositionProvider``
    with an explicit resolved catalog so they cannot widen to this aggregate.
    """

    def __init__(
        self,
        catalog: ResolvedVehicleCompositionCatalog | None = None,
        *,
        provider_id: str = "taoryx.registry.mission-composition",
        provider_name: str = "TAORYX Mission Composition Registry",
        provider_short_name: str = "TAORYX Registry",
        provider_summary: str = "Canonical vehicle families and configuration-ready workflows exposed through one portable catalog.",
        provider_version: str = "1",
        plugin_catalog: PluginCatalog | None = None,
        include_builtin_workflows: bool = True,
        allowed_mission_template_ids: Mapping[str, Sequence[str]] | None = None,
    ) -> None:
        """Build a provider over one explicit vehicle catalog or the legacy aggregate.

        Direct vehicle plug-ins use ``CatalogMissionCompositionProvider`` with
        ``include_builtin_workflows=False`` so a family can be installed
        without pulling unrelated Simple Aero or dual-launch workflows. The
        omitted-catalog fallback is retained only for historical aggregate
        consumers.
        """

        if not provider_version.strip():
            raise ValueError("Mission Composition provider version must not be empty")
        self._plugin_catalog = plugin_catalog
        resolved_catalog = catalog or load_resolved_vehicle_composition_catalog()
        self._catalog = (
            _scope_catalog_mission_templates(resolved_catalog, allowed_mission_template_ids)
            if allowed_mission_template_ids is not None
            else resolved_catalog
        )
        vehicle_schemas = tuple(_configuration_schema(item) for item in self._catalog.vehicles)
        self._simple_aero_model_id: str | None = None
        extra_models: tuple[TrajectoryModelMetadata, ...]
        if include_builtin_workflows:
            from .dual_launch_mission_composition import dual_launch_configuration_schema, dual_launch_model_metadata
            from .simple_aero_mission_composition import (
                SIMPLE_AERO_MODEL_ID,
                simple_aero_configuration_schema,
                simple_aero_model_metadata,
            )

            simple_aero_schema = simple_aero_configuration_schema()
            dual_launch_schema = dual_launch_configuration_schema()
            schemas = (*vehicle_schemas, simple_aero_schema, dual_launch_schema)
            extra_models = (
                simple_aero_model_metadata(simple_aero_schema),
                dual_launch_model_metadata(dual_launch_schema),
            )
            self._simple_aero_model_id = SIMPLE_AERO_MODEL_ID
        else:
            schemas = vehicle_schemas
            extra_models = ()
        self._schemas = {item.model_id: item for item in schemas}
        if len(self._schemas) != len(schemas):
            raise ValueError("Mission Composition registry has duplicate model IDs")
        vehicle_models = tuple(_model_metadata(vehicle, self._schemas[_model_id(vehicle)]) for vehicle in self._catalog.vehicles)
        self._models = (*vehicle_models, *extra_models)
        self._model_by_id = {item.id: item for item in self._models}
        self._metadata = TrajectoryProviderMetadata(
            id=provider_id,
            name=provider_name,
            version=provider_version,
            description=(
                "Portable discovery and configuration schemas projected from TAORYX's canonical vehicle-family authorities "
                "the Simple Aero workflow contract, and the dual-launch glider composition proof."
            ),
            presentation=TrajectoryProviderPresentationMetadata(
                display_name=provider_name,
                short_name=provider_short_name,
                summary=provider_summary,
                organization="TAORYX",
                categories=("Canonical Models", "Trajectory Workflows"),
            ),
            status="common_batch_and_session_execution",
            tags=("trajectory", "mission-composition", "registry-backed", "workflow-aware")
            if include_builtin_workflows
            else ("trajectory", "mission-composition", "registry-backed", "family-owned"),
            execution_contract="taoryx.vehicle-execution-bindings/v1alpha1",
            model_count=len(self._models),
            provenance="verification/vehicle_composition_registry.yaml; verification/alpha2_family_catalog.yaml",
            claim_boundary=(
                "This provider validates portable configurations and exposes exact registered batch and stateful session "
                "bindings. It does not qualify or substitute a vehicle, mission, realization, or fidelity."
            ),
        )
        ####

    @property
    def metadata(self) -> TrajectoryProviderMetadata:
        """Return provider identity without embedding the model schemas."""

        return self._metadata
        ####

    @property
    def plugin_catalog(self) -> PluginCatalog | None:
        """Return the selected host catalog retained by a focused provider.

        ``None`` retains the historical aggregate-discovery behavior for
        callers that construct the registry provider directly.
        """

        return self._plugin_catalog
        ####

    def list_models(self) -> tuple[TrajectoryModelMetadata, ...]:
        """Return common metadata for every resolved canonical family."""

        return self._models
        ####

    def get_model_schema(self, model_id: str) -> TrajectoryConfigurationSchema:
        """Return one model's complete portable configuration grammar."""

        try:
            return self._schemas[model_id]
        except KeyError as error:
            raise KeyError(f"unknown trajectory model {model_id!r}") from error
        ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        """Return one model's required core state and selectable telemetry."""

        return self.model(model_id).output_schema
        ####

    def model(self, model_id: str) -> TrajectoryModelMetadata:
        """Return one common model descriptor."""

        try:
            return self._model_by_id[model_id]
        except KeyError as error:
            raise KeyError(f"unknown trajectory model {model_id!r}") from error
        ####

    def validate_configuration(
        self,
        configuration: TrajectoryConfigurationInstance,
    ) -> PreparedTrajectoryConfiguration:
        """Validate a typed configuration and all selected discovery identities."""

        prepared = validate_configuration_instance(self.get_model_schema(configuration.model_id), configuration)
        model = self.model(configuration.model_id)
        realization = None
        if configuration.realization_id is not None:
            realization = next((item for item in model.realizations if item.id == configuration.realization_id), None)
            if realization is None:
                raise ConfigurationContractError(
                    "unknown-realization",
                    f"expected one of {sorted(item.id for item in model.realizations)!r}",
                    path="configuration.realization_id",
                )
            if _dynamics_fidelity(configuration.fidelity) not in realization.dynamics_fidelities:
                raise ConfigurationContractError(
                    "realization-fidelity-incompatible",
                    f"realization {realization.id!r} does not support fidelity {configuration.fidelity!r}",
                    path="configuration.realization_id",
                )
            if "validate" not in realization.operations:
                raise ConfigurationContractError(
                    "realization-not-configurable",
                    f"realization {realization.id!r} is not available for configuration validation",
                    path="configuration.realization_id",
                )

        root = _prepared_root(prepared)
        initialization = (
            _prepared_choice(root, "initialization") if "initialization" in root else _prepared_choice(root, "launch_mode") if "launch_mode" in root else None
        )
        segments = _prepared_segments(root) if isinstance(root.get("segments"), list) else None
        if configuration.mission_template_id is not None:
            mission = next(
                (item for item in model.mission_templates if item.id == configuration.mission_template_id),
                None,
            )
            if mission is None:
                raise ConfigurationContractError(
                    "unknown-mission-template",
                    f"expected one of {sorted(item.id for item in model.mission_templates)!r}",
                    path="configuration.mission_template_id",
                )
            if configuration.fidelity not in mission.compatible_fidelities:
                raise ConfigurationContractError(
                    "mission-fidelity-incompatible",
                    f"mission {mission.id!r} does not support fidelity {configuration.fidelity!r}",
                    path="configuration.mission_template_id",
                )
            if initialization is not None and initialization[0] not in mission.initialization_variants:
                raise ConfigurationContractError(
                    "mission-initialization-incompatible",
                    f"mission {mission.id!r} does not accept initialization {initialization[0]!r}",
                    path="configuration.root.initialization",
                )
            if segments is not None and not mission.accepts_segment_sequence(segments):
                if mission.open_segment_sequence is not None:
                    grammar = mission.open_segment_sequence
                    raise ConfigurationContractError(
                        "mission-open-sequence-mismatch",
                        (
                            f"mission {mission.id!r} accepts {grammar.minimum_items} through "
                            f"{grammar.maximum_items if grammar.maximum_items is not None else 'unbounded'} occurrences "
                            f"from {list(grammar.allowed_segment_ids)!r}"
                        ),
                        path=f"configuration.root.{grammar.configuration_node_id}",
                    )
                raise ConfigurationContractError(
                    "mission-sequence-mismatch",
                    f"mission {mission.id!r} requires segment sequence {list(mission.segment_sequence)!r}",
                    path="configuration.root.segments",
                )
            if realization is not None and mission.id not in realization.mission_template_ids:
                raise ConfigurationContractError(
                    "realization-mission-incompatible",
                    f"realization {realization.id!r} does not support mission {mission.id!r}",
                    path="configuration.realization_id",
                )

        if configuration.model_id == "tumbling_body" and configuration.realization_id in _TUMBLING_BODY_SHAPES:
            if initialization is None:
                raise ConfigurationContractError(
                    "missing-initialization-selection",
                    "named tumbling realizations require an initialization choice",
                    path="configuration.root.initialization",
                )
            selected_shape = initialization[1].get("body_shape")
            if selected_shape != configuration.realization_id:
                raise ConfigurationContractError(
                    "realization-configuration-mismatch",
                    f"realization {configuration.realization_id!r} requires body_shape={configuration.realization_id!r}",
                    path="configuration.root.initialization.body_shape",
                )
        return prepared
        ####

    def build_model_default_configuration(
        self,
        model_id: str,
        *,
        configuration_id: str,
    ) -> TrajectoryConfigurationInstance:
        """Return one checked-in runnable default without inventing schema defaults.

        Canonical vehicle families use their existing exact batch witnesses.
        The two package-owned workflows retain their own configuration builders.
        This keeps the user-facing default path traceable to the same evidence
        that proves the advertised native binding, while leaving authoring
        schemas free to require real mission choices.
        """

        self.model(model_id)
        if model_id == "simple_aero":
            from .simple_aero_mission_composition import build_simple_aero_example_configuration

            configuration = build_simple_aero_example_configuration(schema=self.get_model_schema(model_id))
        elif model_id == "dual_launch_glider":
            from .dual_launch_mission_composition import build_dual_launch_example_configuration

            configuration = build_dual_launch_example_configuration(schema=self.get_model_schema(model_id))
        else:
            from ..vehicle_catalog_resources import vehicle_catalog_resource
            from ..vehicle_composition import load_vehicle_composition_request
            from ..vehicle_execution_witnesses import load_vehicle_execution_witness_catalog
            from .native_mission_composition import configuration_instance_from_vehicle_request

            configuration = None
            witnesses = load_vehicle_execution_witness_catalog(plugins=self._plugin_catalog).witnesses
            for witness in witnesses:
                if witness.operation != "batch":
                    continue
                request = load_vehicle_composition_request(
                    vehicle_catalog_resource(witness.composition, plugins=self._plugin_catalog)
                )
                if request.vehicle == model_id:
                    configuration = configuration_instance_from_vehicle_request(self, request)
                    break
            if configuration is None:
                raise ValueError(
                    f"model {model_id!r} has no checked-in batch execution witness for a runnable default"
                )
        return configuration.model_copy(update={"configuration_id": configuration_id})
        ####

    def open_session_episode(
        self,
        prepared: PreparedTrajectoryConfiguration,
        *,
        seed: int | None = None,
        integration_step_s: float = 0.02,
    ) -> object:
        """Dispatch noncanonical stateful workflows with exact registered factories."""

        model = self.model(prepared.configuration.model_id)
        if self._simple_aero_model_id is not None and model.id == self._simple_aero_model_id:
            from .simple_aero_mission_composition import open_simple_aero_session_episode

            return open_simple_aero_session_episode(
                model,
                prepared,
                seed=seed,
                integration_step_s=integration_step_s,
            )
        raise TypeError(f"model {model.id!r} has no noncanonical session episode factory")
        ####

    def build_runner(self) -> MissionCompositionRunnerRegistry:
        """Build the exact common batch runner advertised by this registry.

        The native bridge is imported only at the execution boundary because it
        lowers typed portable configurations back into model-specific runtime
        artifacts.  Discovery and authoring therefore stay importable without
        constructing execution factories.
        """

        from .native_mission_composition import build_registry_mission_composition_runner

        return build_registry_mission_composition_runner(self)
        ####


_TUMBLING_BODY_SHAPES = ("cylinder", "sphere", "cone", "triaxial_ellipsoid")


def _prepared_root(prepared: PreparedTrajectoryConfiguration) -> Mapping[str, object]:
    if not isinstance(prepared.resolved, Mapping):
        raise ConfigurationContractError("invalid-prepared-root", "resolved root must be a mapping")
    return prepared.resolved
    ####


def _prepared_choice(root: Mapping[str, object], identifier: str) -> tuple[str, Mapping[str, object]]:
    raw = root.get(identifier)
    if not isinstance(raw, Mapping) or not isinstance(raw.get("selected"), str) or not isinstance(raw.get("value"), Mapping):
        raise ConfigurationContractError(
            "invalid-prepared-choice",
            "resolved choice must contain selected and value",
            path=f"configuration.root.{identifier}",
        )
    return str(raw["selected"]), cast(Mapping[str, object], raw["value"])
    ####


def _prepared_segments(root: Mapping[str, object]) -> tuple[str, ...]:
    raw = root.get("segments")
    if not isinstance(raw, list):
        raise ConfigurationContractError(
            "invalid-prepared-sequence",
            "resolved segment sequence must be a list",
            path="configuration.root.segments",
        )
    result: list[str] = []
    for index, item in enumerate(raw):
        if not isinstance(item, Mapping) or not isinstance(item.get("selected"), str):
            raise ConfigurationContractError(
                "invalid-prepared-sequence-item",
                "resolved segment must contain a selected variant",
                path=f"configuration.root.segments[{index}]",
            )
        result.append(str(item["selected"]))
    return tuple(result)
    ####


def _model_id(vehicle: ResolvedVehicleComposition) -> str:
    return vehicle.family.family.vehicle_registry_id or vehicle.family.family_id
    ####


def _configuration_schema(vehicle: ResolvedVehicleComposition) -> TrajectoryConfigurationSchema:
    declaration = vehicle.declaration
    initialization = ConfigurationChoiceSchema(
        id="initialization",
        label="Initial State",
        description="Select exactly one advertised initial-state contract.",
        variants=tuple(
            ConfigurationChoiceVariant(
                id=item.id,
                label=_label(item.id),
                description=item.description,
                compatible_fidelities=item.compatible_fidelities,
                node=ConfigurationGroupSchema(
                    id=f"{item.id}_parameters",
                    label=f"{_label(item.id)} Parameters",
                    description=item.description,
                    children=tuple(_parameter_schema(parameter, role="initialization", availability=item.status) for parameter in item.parameters),
                ),
            )
            for item in declaration.initialization_contracts
        ),
    )
    segment_choice = ConfigurationChoiceSchema(
        id="segment",
        label="Segment Type",
        description="Select one advertised semantic segment for this sequence position.",
        variants=tuple(
            ConfigurationChoiceVariant(
                id=item.id,
                label=_label(item.id),
                description=item.description,
                compatible_fidelities=item.compatible_fidelities,
                node=ConfigurationGroupSchema(
                    id=f"{item.id}_parameters",
                    label=f"{_label(item.id)} Parameters",
                    description=item.description,
                    children=tuple(_parameter_schema(parameter, role="segment", availability=item.status) for parameter in item.parameters),
                ),
            )
            for item in declaration.segment_contracts
        ),
    )
    sequence_lengths = tuple(len(item.segment_sequence) for item in declaration.mission_templates)
    segments = ConfigurationSequenceSchema(
        id="segments",
        label="Mission Segments",
        description=(
            "Ordered segment sequence. The canonical registry currently permits the exact advertised mission templates; "
            "providers may publish open custom sequences in the same grammar."
        ),
        item=segment_choice,
        minimum_items=min(sequence_lengths),
        maximum_items=max(sequence_lengths),
        templates=tuple(
            ConfigurationSequenceTemplate(
                id=item.id,
                label=_label(item.id),
                description=item.description,
                item_variants=item.segment_sequence,
                compatible_fidelities=item.compatible_fidelities,
            )
            for item in declaration.mission_templates
        ),
        allow_custom=False,
    )
    children: list[ConfigurationChoiceSchema | ConfigurationSequenceSchema | ConfigurationOptionalSchema] = [initialization, segments]
    variant_node = _variant_configuration(vehicle)
    if variant_node is not None:
        children.append(variant_node)
    supported_fidelities = tuple(tier for tier in CANONICAL_FIDELITY_TIERS if vehicle.family.family.tiers[tier].profile_id is not None)
    return TrajectoryConfigurationSchema(
        model_id=_model_id(vehicle),
        model_version=vehicle.declaration.metadata.model_version,
        supported_fidelities=supported_fidelities,
        root=ConfigurationGroupSchema(
            id="mission",
            label="Mission Configuration",
            description="Initial state, ordered segments, and optional vehicle-variant inputs.",
            children=tuple(children),
        ),
        claim_boundary=(
            "Schema validation proves structural, unit, value-space, fidelity, and advertised-sequence compatibility. "
            "It does not establish trim convergence, mission feasibility, runtime availability, or qualification."
        ),
    )
    ####


def _variant_configuration(vehicle: ResolvedVehicleComposition) -> ConfigurationOptionalSchema | None:
    bindings = vehicle.declaration.variant_parameters
    if not bindings:
        return None
    by_group: dict[str, list[VariantParameterBinding]] = defaultdict(list)
    for binding in bindings:
        by_group[binding.coupling_group or binding.id].append(binding)
    groups: list[ConfigurationOptionalSchema] = []
    for group_id, group_bindings in sorted(by_group.items()):
        policies = {item.coupling_policy for item in group_bindings}
        item: ConfigurationChoiceSchema | ConfigurationGroupSchema
        if policies == {"exclusive"}:
            item = ConfigurationChoiceSchema(
                id=f"{group_id}_choice",
                label=f"{_label(group_id)} Choice",
                description="At most one modifier in this coupling group may be selected.",
                variants=tuple(
                    ConfigurationChoiceVariant(
                        id=binding.id,
                        label=_label(binding.id),
                        description=binding.description,
                        compatible_fidelities=binding.compatible_fidelities,
                        node=ConfigurationGroupSchema(
                            id=f"{binding.id}_value",
                            label=f"{_label(binding.id)} Value",
                            children=(_variant_parameter_schema(binding),),
                        ),
                    )
                    for binding in group_bindings
                ),
            )
        else:
            item = ConfigurationGroupSchema(
                id=f"{group_id}_values",
                label=f"{_label(group_id)} Values",
                description="Composable modifiers in one declared coupling group.",
                children=tuple(
                    ConfigurationOptionalSchema(
                        id=binding.id,
                        label=_label(binding.id),
                        description=binding.description,
                        item=_variant_parameter_schema(binding),
                    )
                    for binding in group_bindings
                ),
            )
        groups.append(
            ConfigurationOptionalSchema(
                id=group_id,
                label=_label(group_id),
                description="Optional vehicle-variant coupling group.",
                item=item,
            )
        )
    return ConfigurationOptionalSchema(
        id="variant_configuration",
        label="Vehicle Variant",
        description="Optional source-owned vehicle-variant inputs.",
        item=ConfigurationGroupSchema(
            id="variant_groups",
            label="Variant Groups",
            children=tuple(groups),
        ),
    )
    ####


def _variant_parameter_schema(binding: VariantParameterBinding) -> ConfigurationParameterSchema:
    payload = binding.public_dict()
    return _parameter_schema_from_mapping(payload, role="variant", availability=binding.status)
    ####


def _parameter_schema(
    parameter: ParameterSpec,
    *,
    role: Literal["initialization", "segment"],
    availability: str,
) -> ConfigurationParameterSchema:
    return _parameter_schema_from_mapping(parameter.public_dict(), role=role, availability=availability)
    ####


def _parameter_schema_from_mapping(
    payload: Mapping[str, object],
    *,
    role: Literal["initialization", "segment", "variant"],
    availability: str,
) -> ConfigurationParameterSchema:
    parameter_id = str(payload["id"])
    unit = payload.get("canonical_unit")
    canonical_unit = str(unit) if unit is not None else None
    raw_value_type = str(payload.get("value_type", "scalar"))
    value_type = {
        "scalar": "number",
        "vector3": "vector3",
        "vector4": "vector4",
        "enum": "enum",
    }[raw_value_type]
    value_space_payload = payload.get("value_space")
    if not isinstance(value_space_payload, Mapping):
        raise ValueError(f"parameter {parameter_id!r} has no portable value-space mapping")
    period = value_space_payload.get("period")
    periodicity = None
    if period is not None:
        periodicity = ConfigurationPeriodicity(
            period=float(period),
            canonical_minimum=-180.0 if parameter_id == "longitude_deg" else 0.0,
        )
    hard_lower = _optional_float(payload.get("hard_lower"))
    hard_upper = _optional_float(payload.get("hard_upper"))
    interval = _interval(hard_lower, hard_upper) if value_type in {"number", "integer"} else None
    choices = _string_tuple(payload.get("options"), field=f"{parameter_id}.options")
    compatible = _string_tuple(
        payload.get("compatible_fidelities"),
        field=f"{parameter_id}.compatible_fidelities",
    )
    transform = str(payload.get("transform", "identity"))
    if transform not in {"identity", "log", "logit", "categorical"}:
        raise ValueError(f"parameter {parameter_id!r} has unsupported transform {transform!r}")
    projection = str(payload.get("projection_policy", payload.get("resolution_policy", "reject_invalid")))
    if projection not in {"reject_invalid", "project_to_valid"}:
        raise ValueError(f"parameter {parameter_id!r} has unsupported projection policy {projection!r}")
    return ConfigurationParameterSchema(
        id=parameter_id,
        label=_label(parameter_id),
        description=str(payload.get("description") or f"Configuration value {parameter_id}."),
        value_type=value_type,  # type: ignore[arg-type]
        quantity=None if canonical_unit is None else _QUANTITY_BY_UNIT.get(canonical_unit),
        canonical_unit=canonical_unit,
        display_unit=canonical_unit,
        required=bool(payload.get("required", role == "variant")),
        default=payload.get("default"),
        default_declared=bool(payload.get("default_declared", False)),
        interval=interval,
        qualified_interval=_declared_interval(payload, "qualified_lower", "qualified_upper"),
        safe_extended_interval=_declared_interval(payload, "safe_extended_lower", "safe_extended_upper"),
        periodicity=periodicity,
        choices=choices,
        role=role,
        availability=availability,
        compatible_fidelities=compatible,
        transform=transform,  # type: ignore[arg-type]
        projection_policy=projection,  # type: ignore[arg-type]
        visibility_note=_optional_string(payload.get("conditional_visibility")),
        coupling_group=_optional_string(payload.get("coupling_group")),
        derivation=_optional_string(payload.get("derivation")),
        invalidations=_string_tuple(payload.get("invalidations"), field=f"{parameter_id}.invalidations"),
        frame=_optional_string(payload.get("frame")),
        value_space=ConfigurationValueSpace.from_mapping(value_space_payload),
        provenance=str(payload.get("provenance") or "verification/vehicle_composition_registry.yaml"),
    )
    ####


def _model_metadata(
    vehicle: ResolvedVehicleComposition,
    schema: TrajectoryConfigurationSchema,
) -> TrajectoryModelMetadata:
    descriptor = vehicle.as_dict()
    fidelity_records = descriptor["fidelities"]
    execution_bindings = descriptor["execution_bindings"]
    if not isinstance(fidelity_records, Mapping) or not isinstance(execution_bindings, list):
        raise ValueError(f"resolved family {vehicle.family.family_id!r} has invalid discovery metadata")
    declared_mission_ids = {item.id for item in vehicle.declaration.mission_templates}
    execution_bindings = [
        item
        for item in execution_bindings
        if isinstance(item, Mapping) and item.get("mission") in declared_mission_ids
    ]
    runnable_operations = {
        "batch" if item.get("operation") == "batch" else "step"
        for item in execution_bindings
        if isinstance(item, Mapping) and item.get("status") == "runnable" and item.get("operation") in {"batch", "episode"}
    }
    operations = cast(
        tuple[Literal["discover", "validate", "batch", "step"], ...],
        tuple(item for item in ("discover", "validate", "batch", "step") if item in {"discover", "validate", *runnable_operations}),
    )
    common_runner_operations = cast(
        tuple[Literal["batch", "step"], ...],
        tuple(item for item in ("batch", "step") if item in runnable_operations),
    )
    fidelities = tuple(_fidelity_metadata(vehicle.family.family_id, tier, fidelity_records[tier], execution_bindings) for tier in CANONICAL_FIDELITY_TIERS)
    source_refs = tuple(
        item
        for item in (
            "verification/vehicle_composition_registry.yaml",
            "verification/horizontal_fidelity_registry.yaml",
            "verification/vehicle_execution_bindings.yaml",
            vehicle.family.source_manifest_path,
        )
        if item is not None
    )
    display_name = str(descriptor["display_name"])
    realizations = _realization_metadata(vehicle, fidelities, execution_bindings)
    mission_templates = _mission_templates(vehicle, execution_bindings, realizations)
    available_output_operations = cast(
        tuple[Literal["batch", "step"], ...],
        tuple(item for item in ("batch", "step") if item in operations),
    )
    presentation = TrajectoryModelPresentationMetadata(
        display_name=display_name,
        short_name=display_name,
        summary=f"Canonical {vehicle.family.family.physical_family} family projected from registry authorities.",
        category="Canonical Vehicle Families",
        subcategory=vehicle.family.family.physical_family,
        sort_key=f"{vehicle.family.family.physical_family}:{display_name}",
        badges=tuple(item for item in ("Canonical", "Native Binding" if runnable_operations else "Declared")),
        default_fidelity_id=next((item.id for item in fidelities if item.declared), fidelities[0].id),
        default_mission_template_id=mission_templates[0].id,
        properties=(
            _model_property(
                "physical_family",
                "Physical Family",
                "Canonical physical-family classification.",
                semantic_role="identity",
                value_type="string",
                value=vehicle.family.family.physical_family,
                group="identity",
                order=10,
            ),
            _model_property(
                "automatic_lowering",
                "Automatic Lowering",
                "Whether the family authority permits validated automatic lowering to a lower fidelity.",
                semantic_role="capability",
                value_type="boolean",
                value=vehicle.family.family.automatic_lowering,
                group="fidelity",
                order=20,
            ),
            _model_property(
                "declared_fidelity_count",
                "Declared Fidelity Tiers",
                "Number of fidelity tiers explicitly declared by this family.",
                semantic_role="capability",
                value_type="integer",
                value=sum(item.declared for item in fidelities),
                group="fidelity",
                order=30,
            ),
            _model_property(
                "mission_template_count",
                "Mission Templates",
                "Number of ordered mission recipes advertised by the family declaration.",
                semantic_role="capability",
                value_type="integer",
                value=len(mission_templates),
                group="capabilities",
                order=40,
            ),
        ),
    )
    deployments = _deployment_metadata(vehicle)
    output_schema = _registry_output_schema(
        schema,
        fidelities,
        available_output_operations,
        deployments,
    )
    return TrajectoryModelMetadata(
        id=schema.model_id,
        name=display_name,
        version=schema.model_version,
        description=f"Mission Composition model for {display_name} ({vehicle.family.family.physical_family}).",
        presentation=presentation,
        family_id=vehicle.family.family_id,
        physical_family=vehicle.family.family.physical_family,
        model_kind="canonical_vehicle_family",
        status="common_runner_ready" if common_runner_operations else "declared",
        tags=(vehicle.family.family.physical_family, "canonical-family"),
        execution_capability_profile="taoryx_universal",
        operations=operations,
        common_runner_operations=common_runner_operations,
        capabilities=_model_capabilities(vehicle, operations, deployments),
        realizations=realizations,
        mission_templates=mission_templates,
        deployments=deployments,
        reference_frames=_registry_reference_frames(schema),
        output_schema=output_schema,
        output_schema_id=output_schema.schema_id,
        output_schema_fingerprint=output_schema.fingerprint,
        configuration_schema_id=schema.schema_id,
        configuration_schema_fingerprint=schema.fingerprint,
        fidelities=fidelities,
        fidelity_transitions=_fidelity_transitions(vehicle, fidelity_records),
        source_refs=source_refs,
        provenance="registry projection; no duplicated plant data",
        claim_boundary=(
            "Capabilities reflect exact registry and execution-binding records. A declared model or tier is not automatically "
            "qualified, runnable for every mission, or interchangeable with another fidelity."
        ),
    )
    ####


def _model_property(
    property_id: str,
    label: str,
    description: str,
    *,
    semantic_role: Literal["identity", "geometry", "mass", "performance", "capability", "evidence", "implementation"],
    value_type: Literal["number", "integer", "boolean", "string", "enum", "vector3", "vector4"],
    value: object,
    group: str,
    order: int,
) -> TrajectoryModelPropertyMetadata:
    return TrajectoryModelPropertyMetadata(
        id=property_id,
        label=label,
        description=description,
        semantic_role=semantic_role,
        value_type=value_type,
        value_kind="declared",
        value=value,
        value_declared=True,
        presentation=ValuePresentationMetadata(group=group, order=order),
        provenance="canonical vehicle-composition registry projection",
        claim_boundary="Registry declaration only; this property does not imply execution or qualification.",
    )
    ####


def _model_capabilities(
    vehicle: ResolvedVehicleComposition,
    operations: tuple[Literal["discover", "validate", "batch", "step"], ...],
    deployments: tuple[TrajectoryDeploymentMetadata, ...],
) -> TrajectoryModelCapabilities:
    family_id = vehicle.family.family_id
    transition_events = tuple(dict.fromkeys(event for segment in vehicle.declaration.segment_contracts for event in segment.permitted_transition_events))
    return TrajectoryModelCapabilities(
        initialization_modes=tuple(item.id for item in vehicle.declaration.initialization_contracts),
        segment_types=tuple(item.id for item in vehicle.declaration.segment_contracts),
        termination_modes=transition_events or ("mission_template_terminal",),
        operations=operations,
        supports_custom_segments=False,
        supports_deployment=bool(deployments),
        supports_staging=family_id in {"x15", "hl20_mod_k", "reference_nesc_two_stage_rocket"},
        supports_dynamic_child_generation=any(item.lifecycle == "independently_propagated" for item in deployments),
        supports_multiple_stages=family_id == "reference_nesc_two_stage_rocket",
        supports_submodels=bool(deployments),
    )
    ####


def _control_advertisement(
    vehicle: ResolvedVehicleComposition,
    *,
    fidelity_aliases: tuple[str, ...],
    input_realization: str,
    realization_status: Literal["available", "blocked", "unsupported"],
    operations: tuple[Literal["validate", "batch", "step"], ...],
    mission_ids: tuple[str, ...],
) -> TrajectoryControlAdvertisement:
    """Project the static vehicle-interface authority into composer metadata."""

    representative_fidelity = next(
        (item for item in fidelity_aliases if item in CANONICAL_FIDELITY_TIERS),
        "point_mass_3dof",
    )
    contract = interface_contract_for_composition(vehicle, representative_fidelity)  # type: ignore[arg-type]
    status = _control_status(
        input_realization=input_realization,
        realization_status=realization_status,
        operations=operations,
        channels=contract.action_channels,
    )
    channels = tuple(
        _interface_control_channel(item, status=status, realization_operations=operations, order=index)
        for index, item in enumerate((*contract.action_channels, *contract.effector_channels), start=1)
    )
    if representative_fidelity == "rigid_body_6dof_surface_allocated":
        known_channel_ids = {item.id for item in channels}
        channels = (
            *channels,
            *tuple(
                item for item in _source_manifest_control_channels(vehicle, status=status, start_order=len(channels) + 1) if item.id not in known_channel_ids
            ),
        )
    authorities = tuple(_control_authority(item, status=status, realization_operations=operations) for item in contract.authority_profiles)
    if channels and not any(item.channel_ids for item in authorities):
        authorities = (
            *authorities,
            TrajectoryControlAuthorityMetadata(
                id="declared_effectors",
                authority="effector",
                availability="planned" if status in {"blocked", "unsupported"} else "available_in_batch",
                channel_ids=tuple(item.id for item in channels),
                operations=() if status in {"blocked", "unsupported"} else ("batch",),
                description="Source-declared control coordinates retained as explicit effector candidates.",
                command_owner="source_program",
                selection_scope="batch",
                switching_policy="locked",
                scheme_id="effector.direct",
                lowering_chain=("source_declared_control_coordinate",),
                source_refs=tuple(item for item in (vehicle.family.source_manifest_path,) if item is not None),
                provenance="source family manifest",
                claim_boundary=(
                    "Declared source coordinates are not physical allocation evidence until a runtime binding publishes "
                    "commanded, achieved, limited, and achieved-wrench channels."
                ),
            ),
        )
    intents = _control_intents(
        vehicle,
        fidelity_aliases=fidelity_aliases,
        mission_ids=mission_ids,
        control_status=status,
        operations=operations,
        channels=channels,
    )
    active_authority = (
        contract.default_authority_profile_id
        if contract.default_authority_profile_id in {item.id for item in authorities}
        else next(
            (item.id for item in authorities if item.availability in {"available", "available_in_batch"}),
            None,
        )
    )
    return TrajectoryControlAdvertisement(
        status=status,
        channels=channels,
        authorities=authorities,
        intents=intents,
        default_authority_id=active_authority,
        claim_boundary=(
            "Control channels, semantic intent, and authority are advertised independently from dynamics fidelity. "
            "Native bindings are exact adapter coordinates and never imply actuator allocation or qualification."
        ),
    )
    ####


def _control_status(
    *,
    input_realization: str,
    realization_status: Literal["available", "blocked", "unsupported"],
    operations: tuple[Literal["validate", "batch", "step"], ...],
    channels: tuple[InterfaceChannel, ...],
) -> TrajectoryControlStatus:
    if realization_status == "blocked":
        return "blocked"
    if realization_status == "unsupported":
        return "unsupported"
    if input_realization == "uncontrolled":
        return "uncontrolled"
    if "step" in operations and any(item.availability == "available" for item in channels):
        return "available"
    return "internally_generated"
    ####


def _interface_control_channel(
    channel: InterfaceChannel,
    *,
    status: TrajectoryControlStatus,
    realization_operations: tuple[Literal["validate", "batch", "step"], ...],
    order: int,
) -> TrajectoryControlChannelMetadata:
    availability, operations = _control_availability_and_operations(
        channel.availability,
        status=status,
        realization_operations=realization_operations,
    )
    data_type, shape = cast(
        tuple[
            Literal["float64", "int64", "boolean", "string", "json"],
            tuple[int | Literal["variable"], ...],
        ],
        {
            "scalar": ("float64", ()),
            "vector3": ("float64", (3,)),
            "vector4": ("float64", (4,)),
            "boolean": ("boolean", ()),
            "enum": ("string", ()),
            "event": ("string", ()),
        }[channel.value_type],
    )
    binding = dict(channel.binding)
    native_channel_id = binding.get("native_action") or binding.get("native_effector")
    interval = (
        ConfigurationInterval(
            minimum=ConfigurationBound(value=channel.lower) if channel.lower is not None else None,
            maximum=ConfigurationBound(value=channel.upper) if channel.upper is not None else None,
        )
        if channel.lower is not None or channel.upper is not None
        else None
    )
    if channel.value_space is None:
        raise ValueError(f"vehicle-interface control channel {channel.id!r} has no value-space metadata")
    native_value_space = (
        episode_channel_value_space(
            str(native_channel_id),
            channel.canonical_unit,
            channel.lower,
            channel.upper,
        )
        if isinstance(native_channel_id, str)
        else None
    )
    native_data_type, native_shape = _control_storage(native_value_space) if native_value_space is not None else (data_type, shape)
    quantity = "boolean" if data_type == "boolean" else _QUANTITY_BY_UNIT.get(channel.canonical_unit or "")
    return TrajectoryControlChannelMetadata(
        id=channel.id,
        label=_label(channel.id),
        description=channel.description,
        channel_kind=channel.kind,  # type: ignore[arg-type]
        quantity=quantity,
        canonical_unit=None if data_type in {"boolean", "string"} else channel.canonical_unit,
        display_unit=None if data_type in {"boolean", "string"} else channel.canonical_unit,
        data_type=data_type,
        shape=shape,
        interval=interval,
        frame=channel.frame,
        sampling_semantics=channel.sampling,  # type: ignore[arg-type]
        value_space=ConfigurationValueSpace.from_mapping(channel.value_space.as_dict()),
        availability=availability,
        operations=operations,
        native_channel_id=str(native_channel_id) if isinstance(native_channel_id, str) else None,
        native_binding=(
            TrajectoryControlNativeBindingMetadata(
                id=str(native_channel_id),
                quantity="boolean" if native_data_type == "boolean" else _QUANTITY_BY_UNIT.get(channel.canonical_unit or ""),
                canonical_unit=None if native_data_type in {"boolean", "string"} else channel.canonical_unit,
                data_type=native_data_type,
                shape=native_shape,
                interval=interval,
                value_space=ConfigurationValueSpace.from_mapping(native_value_space.as_dict()),
                provider_binding=binding,
            )
            if isinstance(native_channel_id, str) and native_value_space is not None
            else None
        ),
        provider_binding=binding,
        presentation=ValuePresentationMetadata(
            group="controls" if channel.kind == "action" else "effectors",
            order=order * 10,
            control="toggle" if data_type == "boolean" else "vector_editor" if shape else "slider",
        ),
        source_refs=("src/taoryx/vehicle_interface.py",),
        provenance=channel.provenance,
        claim_boundary=channel.claim_boundary or "Declared channel; no execution or qualification claim.",
    )
    ####


def _control_storage(
    value_space: ValueSpaceSpec,
) -> tuple[Literal["float64", "int64", "boolean", "string", "json"], tuple[int | Literal["variable"], ...]]:
    """Mirror the public session representation rules without opening a plant."""

    if value_space.topology == "boolean":
        return "boolean", ()
    if value_space.topology in {"finite_set", "event"}:
        return ("float64", ()) if "code" in value_space.representation else ("string", ())
    if value_space.representation.startswith("vector"):
        size = int(value_space.representation.removeprefix("vector").split(maxsplit=1)[0])
        return "float64", (size,)
    if value_space.topology == "product":
        return "float64", (len(value_space.components),)
    return "float64", ()
    ####


def _control_availability_and_operations(
    availability: TrajectoryControlAvailability,
    *,
    status: TrajectoryControlStatus,
    realization_operations: tuple[Literal["validate", "batch", "step"], ...],
) -> tuple[TrajectoryControlAvailability, tuple[Literal["batch", "step"], ...]]:
    if status == "blocked":
        return (
            "unavailable_at_runtime" if availability in {"available", "available_in_batch"} else availability,
            (),
        )
    if status == "unsupported":
        return ("not_available" if availability in {"available", "available_in_batch"} else availability, ())
    if status == "uncontrolled":
        return ("not_applicable" if availability in {"available", "available_in_batch"} else availability, ())
    if availability == "available" and "step" in realization_operations:
        return "available", ("step",)
    if availability in {"available", "available_in_batch"} and "batch" in realization_operations:
        return "available_in_batch", ("batch",)
    if availability in {"available", "available_in_batch"}:
        return "unavailable_at_runtime", ()
    return availability, ()
    ####


def _control_authority(
    authority: object,
    *,
    status: TrajectoryControlStatus,
    realization_operations: tuple[Literal["validate", "batch", "step"], ...],
) -> TrajectoryControlAuthorityMetadata:
    availability, operations = _control_availability_and_operations(
        authority.availability,  # type: ignore[attr-defined]
        status=status,
        realization_operations=realization_operations,
    )
    return TrajectoryControlAuthorityMetadata(
        id=str(authority.id),  # type: ignore[attr-defined]
        authority=authority.authority,  # type: ignore[attr-defined]
        availability=availability,
        channel_ids=tuple(authority.action_ids),  # type: ignore[attr-defined]
        operations=operations,
        description=str(authority.description),  # type: ignore[attr-defined]
        command_owner=authority.command_owner,  # type: ignore[attr-defined]
        selection_scope=authority.selection_scope,  # type: ignore[attr-defined]
        switching_policy=authority.switching_policy,  # type: ignore[attr-defined]
        applicable_phase_ids=tuple(authority.applicable_phase_ids),  # type: ignore[attr-defined]
        lowering_chain=tuple(authority.lowering_chain),  # type: ignore[attr-defined]
        scheme_id=authority.scheme_id,  # type: ignore[attr-defined]
        scheme_layer=authority.scheme_layer,  # type: ignore[attr-defined]
        consumer_roles=tuple(authority.consumer_roles),  # type: ignore[attr-defined]
        streaming_preference=authority.streaming_preference,  # type: ignore[attr-defined]
        ui_order=authority.ui_order,  # type: ignore[attr-defined]
        source_refs=("src/taoryx/vehicle_interface.py",),
        provenance="vehicle-interface authority profile",
        claim_boundary=str(authority.claim_boundary),  # type: ignore[attr-defined]
    )
    ####


def _source_manifest_control_channels(
    vehicle: ResolvedVehicleComposition,
    *,
    status: TrajectoryControlStatus,
    start_order: int,
) -> tuple[TrajectoryControlChannelMetadata, ...]:
    manifest = vehicle.family.source_manifest
    if manifest is None:
        return ()
    availability: TrajectoryControlAvailability = "planned" if status != "unsupported" else "not_available"
    value_space = ConfigurationValueSpace.from_mapping(default_value_space_for_value_type("scalar").as_dict())
    return tuple(
        TrajectoryControlChannelMetadata(
            id=item.id,
            label=_label(item.id),
            description=item.description or f"Source-declared {item.id} control coordinate.",
            channel_kind="effector" if item.semantic_level == "effector" else "action",
            quantity=_QUANTITY_BY_UNIT.get(item.unit or ""),
            canonical_unit=item.unit,
            display_unit=item.unit,
            interval=(
                ConfigurationInterval(
                    minimum=ConfigurationBound(value=item.minimum) if item.minimum is not None else None,
                    maximum=ConfigurationBound(value=item.maximum) if item.maximum is not None else None,
                )
                if item.minimum is not None or item.maximum is not None
                else None
            ),
            sampling_semantics="held_action",
            value_space=value_space,
            availability=availability,
            operations=(),
            native_channel_id=item.id,
            native_binding=TrajectoryControlNativeBindingMetadata(
                id=item.id,
                quantity=_QUANTITY_BY_UNIT.get(item.unit or ""),
                canonical_unit=item.unit,
                interval=(
                    ConfigurationInterval(
                        minimum=ConfigurationBound(value=item.minimum) if item.minimum is not None else None,
                        maximum=ConfigurationBound(value=item.maximum) if item.maximum is not None else None,
                    )
                    if item.minimum is not None or item.maximum is not None
                    else None
                ),
                value_space=value_space,
                provider_binding={"source_control_id": item.id},
            ),
            provider_binding={"source_control_id": item.id},
            presentation=ValuePresentationMetadata(group="effectors", order=(start_order + index) * 10),
            source_refs=tuple(value for value in (vehicle.family.source_manifest_path,) if value is not None),
            provenance=f"source family manifest; evidence={item.evidence_grade}",
            claim_boundary=(
                "Source control-coordinate declaration only; runtime allocation, actuator dynamics, limits, and achieved wrench remain separately gated."
            ),
        )
        for index, item in enumerate(manifest.controls)
    )
    ####


def _control_intents(
    vehicle: ResolvedVehicleComposition,
    *,
    fidelity_aliases: tuple[str, ...],
    mission_ids: tuple[str, ...],
    control_status: TrajectoryControlStatus,
    operations: tuple[Literal["validate", "batch", "step"], ...],
    channels: tuple[TrajectoryControlChannelMetadata, ...],
) -> tuple[TrajectoryControlIntentMetadata, ...]:
    compatible_fidelities = set(fidelity_aliases)
    segments = tuple(item for item in vehicle.declaration.segment_contracts if set(item.compatible_fidelities) & compatible_fidelities)
    intent_ids = tuple(dict.fromkeys(intent for segment in segments for intent in segment.required_control_intents))
    resolution = cast(
        TrajectoryControlIntentResolution,
        {
            "available": "external_channel",
            "internally_generated": "provider_internal",
            "uncontrolled": "open_loop",
            "blocked": "blocked",
            "unsupported": "unsupported",
        }[control_status],
    )
    run_operations = cast(
        tuple[Literal["batch", "step"], ...],
        tuple(item for item in operations if item in {"batch", "step"}) if resolution not in {"blocked", "unsupported"} else (),
    )
    channel_ids = {item.id for item in channels}
    intent_channel_candidates = {
        "speed": ("guidance.speed.command",),
        "flight_path": ("guidance.flight_path_angle.command",),
        "heading": ("guidance.heading.command",),
        "bank": ("guidance.bank.command",),
        "yaw": ("attitude.yaw.command",),
        "throttle": ("propulsion.command.fraction",),
        "wrench": ("wrench.force.command", "wrench.moment.command"),
    }
    mission_by_id = {item.id: item for item in vehicle.declaration.mission_templates}
    return tuple(
        TrajectoryControlIntentMetadata(
            id=intent,
            label=_label(intent),
            description=f"Semantic {intent.replace('_', ' ')} intent required by compatible mission segments.",
            resolution=resolution,
            segment_ids=tuple(item.id for item in segments if intent in item.required_control_intents),
            mission_template_ids=tuple(
                mission_id
                for mission_id in mission_ids
                if mission_id in mission_by_id
                and set(mission_by_id[mission_id].segment_sequence) & {item.id for item in segments if intent in item.required_control_intents}
            ),
            channel_ids=tuple(item for item in intent_channel_candidates.get(intent, ()) if item in channel_ids),
            operations=run_operations,
            source_refs=("verification/vehicle_composition_registry.yaml",),
            provenance="canonical segment required_control_intents projection",
            claim_boundary=(
                "Intent compatibility names the semantic demand and its resolution boundary; it does not prove control "
                "performance, allocation, or mission feasibility."
            ),
        )
        for intent in intent_ids
    )
    ####


def _realization_metadata(
    vehicle: ResolvedVehicleComposition,
    fidelities: tuple[TrajectoryFidelityMetadata, ...],
    execution_bindings: Sequence[object],
) -> tuple[TrajectoryRealizationMetadata, ...]:
    family_id = vehicle.family.family_id
    mission_ids = tuple(item.id for item in vehicle.declaration.mission_templates)
    result: list[TrajectoryRealizationMetadata] = []
    for fidelity in fidelities:
        bindings = tuple(
            item for item in execution_bindings if isinstance(item, Mapping) and item.get("fidelity") == fidelity.id and item.get("status") == "runnable"
        )
        native_operations = {"batch" if item.get("operation") == "batch" else "step" for item in bindings if item.get("operation") in {"batch", "episode"}}
        operations = cast(
            tuple[Literal["validate", "batch", "step"], ...],
            tuple(item for item in ("validate", "batch", "step") if fidelity.declared and item in {"validate", *native_operations}),
        )
        status: Literal["available", "blocked", "unsupported"]
        if not fidelity.declared:
            status = "unsupported"
        elif native_operations:
            status = "available"
        else:
            status = "blocked"
        blockers = () if status == "available" else fidelity.blockers or ("no exact native execution binding is registered",)
        realization_mission_ids = tuple(dict.fromkeys(str(item["mission"]) for item in bindings if isinstance(item.get("mission"), str)))
        result.append(
            TrajectoryRealizationMetadata(
                id=fidelity.id,
                label=fidelity.label,
                description=(
                    f"Compatibility realization for legacy fidelity selector {fidelity.id!r}; normalized dynamics, "
                    "input realization, and actuator type are published independently."
                ),
                status=status,
                dynamics_fidelities=(fidelity.dynamics_fidelity,),
                input_realization=fidelity.input_realization,
                actuator_types=fidelity.actuator_types,
                controls=_control_advertisement(
                    vehicle,
                    fidelity_aliases=(fidelity.id,),
                    input_realization=fidelity.input_realization,
                    realization_status=status,
                    operations=operations if status == "available" else ("validate",) if fidelity.declared else (),
                    mission_ids=realization_mission_ids,
                ),
                fidelity_aliases=(fidelity.id,),
                mission_template_ids=realization_mission_ids,
                operations=operations if status == "available" else ("validate",) if fidelity.declared else (),
                native_factory_ids=tuple(dict.fromkeys(str(item["factory_id"]) for item in bindings if item.get("factory_id") is not None)),
                blockers=blockers,
                source_refs=("verification/horizontal_fidelity_registry.yaml", "verification/vehicle_execution_bindings.yaml"),
                claim_boundary=(
                    "This realization separates dynamics fidelity from input and actuator realization. Native factory "
                    "presence is not by itself a common-runner availability claim."
                ),
            )
        )

    if family_id == "a320_openap_3dof":
        result.append(
            TrajectoryRealizationMetadata(
                id="jsbsim_surrogate_composite_pseudo6dof",
                label="A320 JSBSim Surrogate Composite Pseudo-6-DOF",
                description=(
                    "OpenAP performance plus the qualified JSBSim rotational surrogate executed through the exact "
                    "A320 pseudo-6DOF response-law mission binding."
                ),
                status="available",
                dynamics_fidelities=("pseudo_6dof",),
                input_realization="guidance_command",
                controls=_control_advertisement(
                    vehicle,
                    fidelity_aliases=("pseudo_6dof",),
                    input_realization="guidance_command",
                    realization_status="available",
                    operations=("validate", "batch", "step"),
                    mission_ids=mission_ids,
                ),
                fidelity_aliases=("pseudo_6dof",),
                mission_template_ids=mission_ids,
                operations=("validate", "batch", "step"),
                native_factory_ids=("reduced_fixed_wing_openap.v1", "reduced_fixed_wing_a320_episode.v1"),
                blockers=(),
                source_refs=(
                    "verification/daveml_family_layer_dispositions.yaml",
                    "verification/vehicle_execution_bindings.yaml",
                ),
                claim_boundary=(
                    "This exact response-law route uses OpenAP translational performance and the JSBSim-derived "
                    "rotational surrogate. It does not establish physical actuator allocation, rigid-body dynamics, "
                    "or manufacturer-aircraft qualification."
                ),
            )
        )
    if family_id in {"f16_s119", "hl20_mod_k"}:
        result.append(
            TrajectoryRealizationMetadata(
                id="source_daveml_rigid_body",
                label="Source DAVE-ML Rigid-Body Plant",
                description="Source-grounded rigid-body plant realization retained independently from downstream controls.",
                status="blocked",
                dynamics_fidelities=("rigid_body_6dof",),
                input_realization="provider_defined",
                controls=_control_advertisement(
                    vehicle,
                    fidelity_aliases=("rigid_body_6dof_surface_allocated",),
                    input_realization="provider_defined",
                    realization_status="blocked",
                    operations=("validate",),
                    mission_ids=(),
                ),
                fidelity_aliases=("rigid_body_6dof_direct_wrench", "rigid_body_6dof_surface_allocated"),
                mission_template_ids=(),
                operations=("validate",),
                blockers=("complete_source_plant_mission_adapter",),
                source_refs=(
                    "verification/daveml_family_layer_dispositions.yaml",
                    f"families/reference_{'f16_s119' if family_id == 'f16_s119' else 'hl20_mod_k'}/family.yaml",
                ),
                claim_boundary="Source plant availability is not a complete guidance, control, or mission execution claim.",
            )
        )
    if family_id == "tumbling_body":
        native_shape_operations = tuple(
            item
            for item in ("validate", "batch")
            if item == "validate"
            or any(
                isinstance(binding, Mapping) and binding.get("status") == "runnable" and binding.get("operation") == "batch" for binding in execution_bindings
            )
        )
        for shape in _TUMBLING_BODY_SHAPES:
            available = "batch" in native_shape_operations
            result.append(
                TrajectoryRealizationMetadata(
                    id=shape,
                    label=f"{_label(shape)} Passive Body",
                    description=f"Named passive-tumbling {shape.replace('_', ' ')} geometry realization.",
                    status="available" if available else "blocked",
                    dynamics_fidelities=("point_mass_3dof", "pseudo_6dof"),
                    input_realization="uncontrolled",
                    controls=_control_advertisement(
                        vehicle,
                        fidelity_aliases=("point_mass_3dof", "pseudo_6dof"),
                        input_realization="uncontrolled",
                        realization_status="available" if available else "blocked",
                        operations=cast(
                            tuple[Literal["validate", "batch", "step"], ...],
                            native_shape_operations if available else ("validate",),
                        ),
                        mission_ids=mission_ids,
                    ),
                    fidelity_aliases=("point_mass_3dof", "pseudo_6dof"),
                    mission_template_ids=mission_ids,
                    operations=cast(tuple[Literal["validate", "batch", "step"], ...], native_shape_operations if available else ("validate",)),
                    native_factory_ids=("passive_tumbling_direct_release.v1",) if available else (),
                    blockers=() if available else ("passive_tumbling_batch_binding",),
                    source_refs=(
                        "src/taoryx/vehicle.py",
                        "verification/alpha3_tumbling_body/fidelity_ladder/comparison.json",
                    ),
                    claim_boundary="Shape physics evidence is independent from common composition translator availability.",
                )
            )
    return tuple(result)
    ####


def _dynamics_fidelity(tier: str) -> Literal["point_mass_3dof", "pseudo_6dof", "rigid_body_6dof"]:
    if tier == "point_mass_3dof":
        return "point_mass_3dof"
    if tier == "pseudo_6dof":
        return "pseudo_6dof"
    return "rigid_body_6dof"
    ####


def _input_realization(
    family_id: str,
    tier: str,
) -> Literal["uncontrolled", "guidance_command", "direct_wrench", "actuator_allocated", "source_replay", "provider_defined"]:
    if tier == "rigid_body_6dof_direct_wrench":
        return "direct_wrench"
    if tier == "rigid_body_6dof_surface_allocated":
        return "actuator_allocated"
    if family_id == "tumbling_body" or family_id == "x15":
        return "uncontrolled"
    if family_id in {"reference_nesc_two_stage_rocket", "hl20_mod_k"}:
        return "source_replay"
    return "guidance_command"
    ####


def _actuator_types(
    family_id: str,
    tier: str,
) -> tuple[Literal["not_applicable", "aerodynamic_surfaces", "rotors", "thrust_vectoring", "gimbals", "rcs", "mixed", "provider_defined"], ...]:
    if tier != "rigid_body_6dof_surface_allocated":
        return ("not_applicable",)
    if family_id == "hummingbird":
        return ("rotors",)
    if family_id == "x15":
        return ("mixed",)
    if family_id == "reference_nesc_two_stage_rocket":
        return ("gimbals",)
    if family_id == "tumbling_body":
        return ("provider_defined",)
    return ("aerodynamic_surfaces",)
    ####


def _registry_output_channels(
    model_id: str,
    fidelities: tuple[TrajectoryFidelityMetadata, ...],
    operations: tuple[Literal["batch", "step"], ...],
) -> tuple[TrajectoryOutputChannelMetadata, ...]:
    native = native_output_channel_metadata(model_id)
    if native:
        return native
    declared = tuple(item.id for item in fidelities if item.declared)
    specifications = (
        ("position.altitude.geodetic", "Geodetic Altitude", "length", "m", "linear"),
        ("position.latitude.geodetic", "Geodetic Latitude", "angle", "deg", "linear"),
        ("position.longitude", "Longitude", "angle", "deg", "periodic"),
        ("mass.total", "Total Mass", "mass", "kg", "linear"),
        ("aerodynamics.dynamic_pressure", "Dynamic Pressure", "pressure", "Pa", "linear"),
        ("aerodynamics.angle_of_attack", "Angle of Attack", "angle", "deg", "linear"),
    )
    result: list[TrajectoryOutputChannelMetadata] = []
    for order, (channel_id, label, quantity, unit, interpolation) in enumerate(specifications, start=1):
        periodicity = ConfigurationPeriodicity(period=360.0, canonical_minimum=-180.0) if interpolation == "periodic" else None
        result.append(
            TrajectoryOutputChannelMetadata(
                id=channel_id,
                label=label,
                description=f"Potential {label.casefold()} channel normalized by the common runtime artifact bridge.",
                quantity=quantity,
                canonical_unit=unit,
                display_unit=unit,
                interpolation=cast(Literal["linear", "step", "periodic", "slerp", "event"], interpolation),
                periodicity=periodicity,
                availability="runtime_reported",
                compatible_fidelities=declared,
                operations=operations,
                presentation=ValuePresentationMetadata(group=channel_id.split(".", maxsplit=1)[0], order=order * 10),
                provenance="src/taoryx/output_catalog.py and runtime artifact projection",
                claim_boundary="The executor reports exact channel presence at runtime; publication does not guarantee this channel for every mission.",
            )
        )
    return tuple(result)
    ####


def _registry_output_schema(
    schema: TrajectoryConfigurationSchema,
    fidelities: tuple[TrajectoryFidelityMetadata, ...],
    operations: tuple[Literal["batch", "step"], ...],
    deployments: tuple[TrajectoryDeploymentMetadata, ...],
) -> TrajectoryOutputSchema:
    channels = _registry_output_channels(schema.model_id, fidelities, operations)
    exact_bindings = native_output_bindings(schema.model_id)
    core_ids = (
        {item.id for item in exact_bindings if item.channel_class == "core_state"}
        if exact_bindings
        else {
            "position.altitude.geodetic",
            "position.latitude.geodetic",
            "position.longitude",
        }
    )
    core = tuple(
        item.model_copy(
            update={
                "availability": "guaranteed",
                "claim_boundary": ("Required core state projected by the registered common batch adapter for each compatible exact mission/fidelity tuple."),
            }
        )
        for item in channels
        if item.id in core_ids
    )
    telemetry = tuple(item for item in channels if item.id not in core_ids)
    telemetry_ids = {item.id for item in telemetry}
    group_ids = tuple(dict.fromkeys(item.telemetry_group for item in exact_bindings if item.channel_class == "telemetry" and item.telemetry_group is not None))
    if group_ids:
        groups = tuple(
            TrajectoryTelemetryGroupMetadata(
                id=group_id,
                label=_label(group_id),
                description=f"Requestable {group_id.replace('_', ' ')} telemetry emitted by exact native bindings.",
                channel_ids=tuple(item.id for item in exact_bindings if item.channel_class == "telemetry" and item.telemetry_group == group_id),
                presentation=ValuePresentationMetadata(group="telemetry", order=(index + 1) * 10),
            )
            for index, group_id in enumerate(group_ids)
        )
    else:
        groups = tuple(
            group
            for group in (
                TrajectoryTelemetryGroupMetadata(
                    id="resources",
                    label="Resources",
                    description="Mass and consumable-resource state reported by the selected realization.",
                    channel_ids=tuple(item for item in ("mass.total",) if item in telemetry_ids),
                    presentation=ValuePresentationMetadata(group="telemetry", order=10),
                ),
                TrajectoryTelemetryGroupMetadata(
                    id="aerodynamics",
                    label="Aerodynamics",
                    description="Model aerodynamic state and derived flow quantities.",
                    channel_ids=tuple(item for item in ("aerodynamics.dynamic_pressure", "aerodynamics.angle_of_attack") if item in telemetry_ids),
                    presentation=ValuePresentationMetadata(group="telemetry", order=20),
                ),
            )
            if group.channel_ids
        )
    dynamic = tuple(item for item in deployments if item.lifecycle == "independently_propagated")
    relationship_kinds: list[Literal["release", "separation", "deployment"]] = []
    for deployment in dynamic:
        text = " ".join((deployment.id, *deployment.trigger_event_kinds)).casefold()
        kind: Literal["release", "separation", "deployment"]
        if "separation" in text:
            kind = "separation"
        elif "release" in text:
            kind = "release"
        else:
            kind = "deployment"
        if kind not in relationship_kinds:
            relationship_kinds.append(kind)
    return TrajectoryOutputSchema(
        model_id=schema.model_id,
        model_version=schema.model_version,
        core_channels=core,
        telemetry_channels=telemetry,
        telemetry_groups=groups,
        entity_output=TrajectoryEntityOutputMetadata(
            supports_multiple_entities=bool(dynamic),
            supports_dynamic_spawning=bool(dynamic),
            maximum_descendant_depth=1 if dynamic else 0,
            relationship_kinds=tuple(relationship_kinds),
            child_output_schema_policy="runtime_reported" if dynamic else "not_applicable",
            includes_spawn_initial_state=bool(dynamic),
            includes_lifecycle_events=True,
        ),
        claim_boundary=(
            "Core channels are the normalized positional subset currently published by the registry bridge. "
            "Telemetry availability remains realization- and runtime-dependent; the schema is not a qualification claim."
        ),
    )
    ####


def _registry_reference_frames(schema: TrajectoryConfigurationSchema) -> tuple[TrajectoryReferenceFrameMetadata, ...]:
    frame_ids: set[str] = set()

    def visit(node: object) -> None:
        if isinstance(node, ConfigurationParameterSchema):
            if node.frame is not None:
                frame_ids.add(node.frame)
        elif isinstance(node, ConfigurationGroupSchema):
            for child in node.children:
                visit(child)
        elif isinstance(node, ConfigurationChoiceSchema):
            for variant in node.variants:
                visit(variant.node)
        elif isinstance(node, ConfigurationSequenceSchema):
            visit(node.item)
        elif isinstance(node, ConfigurationOptionalSchema):
            visit(node.item)
        ####

    visit(schema.root)
    records: list[TrajectoryReferenceFrameMetadata] = []
    for frame_id in sorted(frame_ids):
        if frame_id == "NED":
            records.append(
                TrajectoryReferenceFrameMetadata(
                    id="NED",
                    name="Local North-East-Down",
                    description="Local tangent frame used by canonical family configuration parameters.",
                    frame_kind="local_tangent",
                    axes=("north", "east", "down"),
                    handedness="right",
                    origin="mission-selected local datum",
                    orientation="north-east-down",
                    provenance="canonical vehicle-family parameter declarations",
                )
            )
        elif frame_id == "body":
            records.append(
                TrajectoryReferenceFrameMetadata(
                    id="body",
                    name="Body Frame",
                    description="Vehicle-fixed frame used by body-state configuration values.",
                    frame_kind="body",
                    axes=("forward", "right", "down"),
                    handedness="right",
                    origin="vehicle reference point",
                    orientation="forward-right-down",
                    provenance="canonical vehicle-family parameter declarations",
                )
            )
        else:
            records.append(
                TrajectoryReferenceFrameMetadata(
                    id=frame_id,
                    name=_label(frame_id),
                    description="Provider-defined frame or transform representation used by a configuration parameter.",
                    frame_kind="provider_defined",
                    axes=("provider_defined",),
                    handedness="not_applicable",
                    origin="defined by the canonical family parameter contract",
                    orientation="defined by the parameter value-space metadata",
                    provenance="canonical vehicle-family parameter declarations",
                )
            )
    by_id = {item.id: item for item in records}
    for frame in native_output_reference_frames(schema.model_id):
        by_id.setdefault(frame.id, frame)
    return tuple(by_id[item] for item in sorted(by_id))
    ####


def _mission_templates(
    vehicle: ResolvedVehicleComposition,
    execution_bindings: Sequence[object],
    realizations: tuple[TrajectoryRealizationMetadata, ...],
) -> tuple[TrajectoryMissionTemplateMetadata, ...]:
    """Project only mission/realization pairs a caller can actually select.

    The canonical declaration may retain a development recipe whose segment
    semantics cover a broad fidelity set.  That does not make every pair a
    selectable provider endpoint.  This public projection is deliberately
    narrower: runnable and blocked realization state remains visible on the
    realization, while mission operation rows exist only for exact selectable
    combinations.
    """

    def selectable(realization: TrajectoryRealizationMetadata, mission_id: str, fidelity: str) -> bool:
        return (
            realization.status == "available"
            and fidelity in realization.fidelity_aliases
            and (not realization.mission_template_ids or mission_id in realization.mission_template_ids)
        )
        ####

    templates: list[TrajectoryMissionTemplateMetadata] = []
    for mission in vehicle.declaration.mission_templates:
        compatible_fidelities = tuple(
            fidelity for fidelity in mission.compatible_fidelities if any(selectable(realization, mission.id, fidelity) for realization in realizations)
        )
        if not compatible_fidelities:
            continue
        operations: list[TrajectoryMissionOperationMetadata] = []
        for fidelity in compatible_fidelities:
            identity_realization = next((item for item in realizations if item.id == fidelity), None)
            identity_selectable = identity_realization is not None and selectable(identity_realization, mission.id, fidelity)
            if identity_selectable:
                operations.append(
                    TrajectoryMissionOperationMetadata(
                        fidelity=fidelity,
                        realization_id=fidelity,
                        operation="validate",
                        status="available",
                        execution_mode="portable_schema_validation",
                        claim_boundary="Structural configuration validation only; no runtime or feasibility claim.",
                    )
                )
                for operation, binding_operation in (("batch", "batch"), ("step", "episode")):
                    matches = tuple(
                        item
                        for item in execution_bindings
                        if isinstance(item, Mapping)
                        and item.get("mission") == mission.id
                        and item.get("fidelity") == fidelity
                        and item.get("operation") == binding_operation
                    )
                    if len(matches) > 1:
                        raise ValueError(f"mission {mission.id!r}/{fidelity!r}/{binding_operation!r} has duplicate execution bindings")
                    binding = matches[0] if matches else None
                    available = binding is not None and binding.get("status") == "runnable"
                    blockers = (
                        tuple(str(item) for item in binding.get("blockers", ()))
                        if binding is not None
                        else (f"no exact {binding_operation} execution binding is registered",)
                    )
                    operations.append(
                        TrajectoryMissionOperationMetadata(
                            fidelity=fidelity,
                            realization_id=fidelity,
                            operation=operation,  # type: ignore[arg-type]
                            status="available" if available else "blocked",
                            execution_mode=str(binding["execution_mode"]) if binding is not None else None,
                            availability_scope="provider_interface" if available else "native_runtime_binding",
                            common_runner_status=("registered" if available else "not_available"),
                            executor_id=(
                                _EXECUTOR_ID if available and operation == "batch" else _SESSION_EXECUTOR_ID if available and operation == "step" else None
                            ),
                            blockers=() if available else blockers,
                            claim_boundary=(
                                str(binding["claim_boundary"])
                                if binding is not None
                                else "Absence is not permission to substitute another family, mission, fidelity, or executor."
                            ),
                        )
                    )
            if vehicle.family.family_id == "tumbling_body":
                selectable_shapes = tuple(
                    shape for shape in _TUMBLING_BODY_SHAPES if selectable(next(item for item in realizations if item.id == shape), mission.id, fidelity)
                )
                batch_available = any(
                    isinstance(item, Mapping)
                    and item.get("mission") == mission.id
                    and item.get("fidelity") == fidelity
                    and item.get("operation") == "batch"
                    and item.get("status") == "runnable"
                    for item in execution_bindings
                )
                for shape in selectable_shapes:
                    operations.extend(
                        (
                            TrajectoryMissionOperationMetadata(
                                fidelity=fidelity,
                                realization_id=shape,
                                operation="validate",
                                status="available",
                                execution_mode="portable_schema_validation",
                                claim_boundary=("Structural validation binds the named body_shape parameter to the selected realization."),
                            ),
                            TrajectoryMissionOperationMetadata(
                                fidelity=fidelity,
                                realization_id=shape,
                                operation="batch",
                                status="available" if batch_available else "blocked",
                                execution_mode="passive_tumbling_direct_release" if batch_available else None,
                                availability_scope="provider_interface" if batch_available else "native_runtime_binding",
                                common_runner_status="registered" if batch_available else "not_available",
                                executor_id=_EXECUTOR_ID if batch_available else None,
                                blockers=() if batch_available else ("no exact passive-tumbling batch binding is registered",),
                                claim_boundary=(
                                    "The named geometry is compiled into the native passive-body kernel; fidelity semantics "
                                    "remain independently selected and are never inferred from shape."
                                ),
                            ),
                        )
                    )
            if (
                vehicle.family.family_id == "a320_openap_3dof"
                and fidelity == "pseudo_6dof"
                and selectable(
                    next(item for item in realizations if item.id == "jsbsim_surrogate_composite_pseudo6dof"),
                    mission.id,
                    fidelity,
                )
            ):
                for operation, binding_operation in (("batch", "batch"), ("step", "episode")):
                    binding = next(
                        (
                            item
                            for item in execution_bindings
                            if isinstance(item, Mapping)
                            and item.get("mission") == mission.id
                            and item.get("fidelity") == fidelity
                            and item.get("operation") == binding_operation
                        ),
                        None,
                    )
                    available = binding is not None and binding.get("status") == "runnable"
                    operations.append(
                        TrajectoryMissionOperationMetadata(
                            fidelity=fidelity,
                            realization_id="jsbsim_surrogate_composite_pseudo6dof",
                            operation=operation,  # type: ignore[arg-type]
                            status="available" if available else "blocked",
                            execution_mode=str(binding["execution_mode"]) if binding is not None else None,
                            availability_scope="provider_interface" if available else "native_runtime_binding",
                            common_runner_status="registered" if available else "not_available",
                            executor_id=(_EXECUTOR_ID if available and operation == "batch" else _SESSION_EXECUTOR_ID if available else None),
                            blockers=() if available else ("a320_jsbsim_surrogate_execution_binding",),
                            claim_boundary=(
                                "The exact A320 pseudo-6DOF response-law binding uses the named OpenAP/JSBSim "
                                "surrogate composite. It does not establish physical actuator allocation or a "
                                "rigid-body A320 claim."
                            ),
                        )
                    )
        if not operations:
            continue
        templates.append(
            TrajectoryMissionTemplateMetadata(
                id=mission.id,
                name=_label(mission.id),
                description=mission.description,
                status=mission.status,
                initialization_variants=mission.initialization_contracts,
                segment_sequence=mission.segment_sequence,
                compatible_fidelities=compatible_fidelities,
                operations=tuple(operations),
                provenance="verification/vehicle_composition_registry.yaml; verification/vehicle_execution_bindings.yaml",
                claim_boundary=(
                    "The ordered recipe and per-fidelity operation matrix are exact declarations. "
                    "An available operation does not promote fidelity evidence or prove mission feasibility."
                ),
            )
        )
    return tuple(templates)
    ####


def _deployment_metadata(vehicle: ResolvedVehicleComposition) -> tuple[TrajectoryDeploymentMetadata, ...]:
    """Project the currently evidenced parent-to-child capabilities."""

    family_id = vehicle.family.family_id
    advertised_segments = {segment.id for segment in vehicle.declaration.segment_contracts}
    if family_id == "x15" and "booster_coast_release" in advertised_segments:
        return (
            TrajectoryDeploymentMetadata(
                id="spent_booster_release",
                name="Spent Booster Release",
                description="Emit and independently propagate the passive spent booster at the accepted release boundary.",
                status="available",
                trigger_segment_ids=("booster_coast_release",),
                trigger_event_kinds=("booster_release",),
                child_role="spent_booster",
                child_model_id="x15-spent-booster",
                child_model_scope="provider_generated",
                child_model_kind="passive_aero_ballistic",
                minimum_children=1,
                maximum_children=1,
                compatible_fidelities=("point_mass_3dof", "pseudo_6dof"),
                operations=("batch",),
                availability_scope="provider_interface",
                common_runner_status="registered",
                executor_id=_EXECUTOR_ID,
                state_initialization="inherited_at_accepted_boundary",
                fidelity_policy="provider_mapped",
                lifecycle="independently_propagated",
                source_refs=(
                    "verification/vehicle_composition_registry.yaml",
                    "verification/vehicle_execution_bindings.yaml",
                    "src/taoryx/x15_reachability.py",
                ),
                provenance="source-pinned staged reachability plus passive deployment witness",
                claim_boundary=(
                    "The emitted child is a passive spent-booster witness. Its result does not promote the parent X-15 "
                    "model, establish physical separation dynamics, or advertise a guided child."
                ),
            ),
        )
    if family_id == "hl20_mod_k" and "source_booster_release" in advertised_segments:
        return (
            TrajectoryDeploymentMetadata(
                id="synthetic_spent_booster_release",
                name="Synthetic Spent Booster Release",
                description="Emit and independently propagate the synthetic passive booster used by the source replay witness.",
                status="available",
                trigger_segment_ids=("source_booster_release",),
                trigger_event_kinds=("booster-release",),
                child_role="spent_booster",
                child_model_id="hl20-synthetic-spent-booster",
                child_model_scope="provider_generated",
                child_model_kind="passive_aero_ballistic",
                minimum_children=1,
                maximum_children=1,
                compatible_fidelities=("point_mass_3dof", "pseudo_6dof"),
                operations=("batch",),
                availability_scope="provider_interface",
                common_runner_status="registered",
                executor_id=_EXECUTOR_ID,
                state_initialization="inherited_at_accepted_boundary",
                fidelity_policy="provider_mapped",
                lifecycle="independently_propagated",
                source_refs=(
                    "verification/vehicle_composition_registry.yaml",
                    "verification/vehicle_execution_bindings.yaml",
                    "verification/daveml_alpha3_deployment_evidence.json",
                    "src/taoryx/hl20_reachability.py",
                ),
                provenance="source-aerodynamic parent replay plus explicitly synthetic booster child",
                claim_boundary=(
                    "The child is synthetic and separately qualified from the source-grounded HL-20 parent. Child evidence "
                    "does not alter or broaden parent qualification."
                ),
            ),
        )
    if family_id == "reference_nesc_two_stage_rocket" and "stage_separation" in advertised_segments:
        return (
            TrajectoryDeploymentMetadata(
                id="stage_separation_lineage",
                name="Stage Separation Lineage",
                description=(
                    "Publish the ordered cutoff, separation, and upper-stage ignition event chain, with an optional "
                    "explicit binding to the independently installable synthetic passive-cylinder child runtime."
                ),
                status="available",
                trigger_segment_ids=("stage_separation",),
                trigger_event_kinds=("stage_cutoff", "stage_separation", "upper_stage_ignition"),
                child_role="synthetic_released_body",
                child_model_id="nesc-synthetic-cylinder",
                child_model_scope="external",
                child_plugin_id="taoryx.passive-bodies",
                child_runtime_id="taoryx.passive-bodies.local-atmosphere-release.v1",
                child_model_kind="synthetic_passive_aero_ballistic",
                minimum_children=0,
                maximum_children=1,
                compatible_fidelities=(
                    "point_mass_3dof",
                    "pseudo_6dof",
                ),
                operations=("batch",),
                availability_scope="native_runtime_binding",
                common_runner_status="adapter_required",
                executor_id="taoryx.passive-bodies.local-atmosphere-release.v1",
                state_initialization="inherited_at_accepted_boundary",
                fidelity_policy="explicit_child",
                lifecycle="independently_propagated",
                source_refs=(
                    "verification/vehicle_composition_registry.yaml",
                    "verification/daveml_nesc_staging_lineage.json",
                    "families/reference_nesc_two_stage_rocket/deployment/synthetic-passive-cylinder-v1.yaml",
                ),
                provenance="source-history staging lineage plus explicit cross-plugin synthetic child binding",
                claim_boundary=(
                    "A child trajectory exists only when a composition explicitly selects the listed external runtime. "
                    "The parent supplies a source-replay kinematic projection at the accepted event; it does not establish "
                    "a physical separation impulse, source-exact spent-stage identity, child aerodynamics, or parent promotion."
                ),
            ),
        )
    return ()
    ####


def _fidelity_metadata(
    family_id: str,
    tier: str,
    record: object,
    execution_bindings: Sequence[object],
) -> TrajectoryFidelityMetadata:
    if not isinstance(record, Mapping):
        raise ValueError(f"fidelity {tier!r} has an invalid metadata record")
    runnable = {
        "batch" if item.get("operation") == "batch" else "step"
        for item in execution_bindings
        if isinstance(item, Mapping) and item.get("fidelity") == tier and item.get("status") == "runnable" and item.get("operation") in {"batch", "episode"}
    }
    declared = record.get("declared") is True
    operations = cast(
        tuple[Literal["validate", "batch", "step"], ...],
        tuple(item for item in ("validate", "batch", "step") if declared and item in {"validate", *runnable}),
    )
    return TrajectoryFidelityMetadata(
        id=tier,
        label=_FIDELITY_LABELS[tier],
        rank=CANONICAL_FIDELITY_TIERS.index(tier),
        declared=declared,
        dynamics_fidelity=_dynamics_fidelity(tier),
        input_realization=_input_realization(family_id, tier),
        actuator_types=_actuator_types(family_id, tier),
        compatibility_aliases=(tier,),
        runtime_fidelity=str(record["runtime_fidelity"]),
        control_realization=str(record["control_realization"]),
        promotion_status=str(record["promotion_status"]),
        operations=operations,
        profile_id=_optional_string(record.get("profile_id")),
        blockers=tuple(str(item) for item in record.get("blockers", ())),
        required_operations=tuple(str(item) for item in record.get("required_operations", ())),
        claim_boundary=("Declaration, promotion evidence, and exact runnable operations are reported independently; none may be inferred from another."),
    )
    ####


def _fidelity_transitions(
    vehicle: ResolvedVehicleComposition,
    records: Mapping[object, object],
) -> tuple[TrajectoryFidelityTransition, ...]:
    transitions: list[TrajectoryFidelityTransition] = []
    for lower, upper in zip(CANONICAL_FIDELITY_TIERS[:-1], CANONICAL_FIDELITY_TIERS[1:], strict=True):
        transitions.append(_fidelity_transition(vehicle, records, lower, upper, direction="step_up"))
        transitions.append(_fidelity_transition(vehicle, records, upper, lower, direction="step_down"))
    return tuple(transitions)
    ####


def _fidelity_transition(
    vehicle: ResolvedVehicleComposition,
    records: Mapping[object, object],
    source: str,
    target: str,
    *,
    direction: Literal["step_up", "step_down"],
) -> TrajectoryFidelityTransition:
    source_record = records[source]
    target_record = records[target]
    if not isinstance(source_record, Mapping) or not isinstance(target_record, Mapping):
        raise ValueError("fidelity transition records must be mappings")
    both_declared = source_record.get("declared") is True and target_record.get("declared") is True
    if not both_declared:
        status = "not_available"
    elif target_record.get("promotion_status") == "qualified":
        status = "available"
    else:
        status = "conditional"
    automatic = direction == "step_down" and vehicle.family.family.automatic_lowering and both_declared
    requirements: tuple[str, ...]
    if direction == "step_up":
        policy = "explicit_upgrade_only"
        requirements = (
            "submit a new explicit target-fidelity request",
            "satisfy the target tier's profile, adapter-operation, and evidence gates",
        )
    elif automatic:
        policy = "validated_lower_only"
        requirements = (
            "caller opts into validated lowering",
            "selected lower tier has qualified profile evidence and required operations",
        )
    else:
        policy = "exact_only"
        requirements = ("submit a new explicit lower-fidelity request",)
    target_blockers = tuple(str(item) for item in target_record.get("blockers", ()))
    if not both_declared:
        requirements = (*requirements, "both adjacent fidelity profiles must be declared")
    return TrajectoryFidelityTransition(
        from_fidelity=source,
        to_fidelity=target,
        direction=direction,
        status=status,  # type: ignore[arg-type]
        automatic=automatic,
        selection_policy=policy,  # type: ignore[arg-type]
        requirements=(*requirements, *target_blockers),
        state_transfer="not_advertised; start a newly prepared run at the selected target fidelity",
        claim_boundary=(
            "This edge describes request-time fidelity selection. It does not advertise live state transfer, "
            "automatic model promotion, or equivalence between adjacent tiers."
        ),
    )
    ####


def _interval(lower: float | None, upper: float | None) -> ConfigurationInterval:
    return ConfigurationInterval(
        minimum=ConfigurationBound(value=lower) if lower is not None else None,
        maximum=ConfigurationBound(value=upper) if upper is not None else None,
    )
    ####


def _declared_interval(payload: Mapping[str, object], lower_key: str, upper_key: str) -> ConfigurationInterval | None:
    lower = _optional_float(payload.get(lower_key))
    upper = _optional_float(payload.get(upper_key))
    return _interval(lower, upper) if lower is not None or upper is not None else None
    ####


def _optional_float(value: object) -> float | None:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"expected a numeric interval bound, received {value!r}")
    return float(value)
    ####


def _optional_string(value: object) -> str | None:
    return None if value is None else str(value)
    ####


def _string_tuple(value: object, *, field: str) -> tuple[str, ...]:
    if value is None:
        return ()
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{field} must be a sequence")
    return tuple(str(item) for item in value)
    ####


def _label(identifier: str) -> str:
    return identifier.replace("_", " ").title()
    ####


__all__ = ["RegistryMissionCompositionProvider"]
