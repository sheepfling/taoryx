"""Evidence-bounded model cards for installed Taoryx plug-in models.

The model-authoring plan is intentionally selection-specific: it answers how
to author one fidelity, realization, and mission.  This module complements it
with a model-wide card that joins the declared identity, provenance, fidelity
ladder, registered tuning contracts, missions, and editable configuration
grammar without running a plant or a tuning campaign.
"""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from typing import TYPE_CHECKING

from .trajectory.configuration_contract import (
    ConfigurationChoiceSchema,
    ConfigurationGroupSchema,
    ConfigurationNode,
    ConfigurationOptionalSchema,
    ConfigurationParameterSchema,
    ConfigurationSequenceSchema,
    TrajectoryConfigurationSchema,
    TrajectoryFidelityMetadata,
    TrajectoryMissionTemplateMetadata,
    TrajectoryModelMetadata,
    TrajectoryRealizationMetadata,
)

if TYPE_CHECKING:
    from .controller_tuning_registry import ControllerTuningCampaignRegistry
    from .plugins.discovery import PluginCatalog
    from .trajectory.configuration_contract import ConfigurableTrajectoryProvider, ConfigurableTrajectoryProviderRegistry


MODEL_OVERVIEW_SCHEMA = "taoryx.model-overview/v1"


def build_model_overview_catalog(
    plugins: PluginCatalog,
    providers: ConfigurableTrajectoryProviderRegistry,
    tuning_campaigns: ControllerTuningCampaignRegistry,
    *,
    provider_id: str | None = None,
    model_id: str | None = None,
) -> dict[str, object]:
    """Build model cards from the exact installed plug-in advertisements.

    A card is an inspection projection.  It may construct a tuning campaign's
    immutable declaration so it can describe the registered method and
    operating points, but it never constructs the campaign adapter, solves a
    trim point, synthesizes gains, or executes a vehicle.
    """

    selected_providers: Sequence[ConfigurableTrajectoryProvider] = providers.providers
    if provider_id is not None:
        selected_providers = (providers.provider(provider_id),)

    provider_cards: list[dict[str, object]] = []
    matching_model_count = 0
    for provider in selected_providers:
        models = tuple(model for model in provider.list_models() if model_id is None or model.id == model_id)
        matching_model_count += len(models)
        if model_id is not None and not models and provider_id is not None:
            raise KeyError(f"provider {provider.metadata.id!r} does not advertise model {model_id!r}")
        if not models:
            continue
        provider_cards.append(
            _provider_card(
                plugins,
                providers,
                tuning_campaigns,
                provider,
                models,
            )
        )

    if model_id is not None and matching_model_count == 0:
        raise KeyError(f"no installed Mission Composition provider advertises model {model_id!r}")

    return {
        "schema": MODEL_OVERVIEW_SCHEMA,
        "plugin_catalog_fingerprint": plugins.fingerprint,
        "provider_catalog_fingerprint": providers.fingerprint,
        "providers": provider_cards,
        "claim_boundary": (
            "A model card is a projection of installed metadata and immutable tuning declarations. It does not execute "
            "a plant, validate source data, establish trim or controller performance, or promote a fidelity tier."
        ),
    }
    ####


def render_model_overview_markdown(report: Mapping[str, object]) -> str:
    """Render a concise, human-readable view of a model-overview payload."""

    lines = [
        "# Taoryx vehicle model cards",
        "",
        "These cards are generated from the installed plug-in advertisements, configuration schemas, and registered "
        "tuning declarations. They report declared scope and evidence boundaries; they are not certification, source "
        "validation, or a record of a successful controller-design run.",
        "",
    ]
    for provider_card in _mapping_records(report.get("providers")):
        provider = _mapping(provider_card.get("provider"))
        provider_metadata = _mapping(provider.get("metadata"))
        plugin = _mapping(provider_card.get("plugin"))
        plugin_metadata = _mapping(plugin.get("metadata"))
        presentation = _mapping(provider_metadata.get("presentation"))
        display_name = _text(presentation, "display_name", fallback=_text(provider_metadata.get("name")))
        provider_summary = _text(presentation.get("summary"), fallback="")
        provider_description = _text(provider_metadata.get("description"), fallback="")
        categories = _code_list(presentation.get("categories"))
        links = _mapping_records(presentation.get("links"))
        lines.extend(
            (
                f"## {display_name}",
                "",
                f"- Provider: `{_text(provider_metadata.get('id'))}`",
                f"- Plug-in: `{_text(plugin_metadata.get('id'), fallback='not declared')}` "
                f"({_text(plugin_metadata.get('package'), fallback='unknown package')} "
                f"{_text(plugin_metadata.get('version'), fallback='unknown version')})",
                f"- Provider status: `{_text(provider_metadata.get('status'))}`",
                "",
                "#### Provider scope",
                "",
            )
        )
        if provider_summary:
            lines.extend((provider_summary, ""))
        if provider_description:
            lines.extend((provider_description, ""))
        if categories != "—":
            lines.append(f"- Categories: {categories}")
        if links:
            rendered_links = ", ".join(
                f"[{_text(link.get('label'))}]({_text(link.get('uri'))})"
                for link in links
                if _text(link.get("label"), fallback="") and _text(link.get("uri"), fallback="")
            )
            if rendered_links:
                lines.append(f"- Documentation: {rendered_links}")
        lines.append("")
        for card in _mapping_records(provider_card.get("models")):
            _render_model_card_markdown(lines, card)

    lines.extend(("## Scope", "", _text(report.get("claim_boundary")), ""))
    return "\n".join(lines)
    ####


def _provider_card(
    plugins: PluginCatalog,
    providers: ConfigurableTrajectoryProviderRegistry,
    tuning_campaigns: ControllerTuningCampaignRegistry,
    provider: ConfigurableTrajectoryProvider,
    models: Sequence[TrajectoryModelMetadata],
) -> dict[str, object]:
    """Build every selected model card from one owning provider."""

    owner = next(
        (item for item in plugins.records("mission_composition_provider") if item.id == provider.metadata.id),
        None,
    )
    if owner is None:
        plugin: dict[str, object] = {
            "status": "not_declared",
            "metadata": {},
            "revision": None,
        }
    else:
        plugin = {
            "status": "declared",
            "metadata": owner.plugin.public_dict(),
            "revision": plugins.plugin_revision(owner.plugin.id).public_dict(),
        }
    return {
        "provider": {
            "metadata": provider.metadata.model_dump(mode="json", by_alias=True),
            "revision": providers.provider_revision(provider.metadata.id),
        },
        "plugin": plugin,
        "models": [
            _model_card(
                providers,
                tuning_campaigns,
                provider_id=provider.metadata.id,
                model=model,
            )
            for model in models
        ],
    }
    ####


def _model_card(
    providers: ConfigurableTrajectoryProviderRegistry,
    tuning_campaigns: ControllerTuningCampaignRegistry,
    *,
    provider_id: str,
    model: TrajectoryModelMetadata,
) -> dict[str, object]:
    """Join one model's discovery, configuration, and campaign declarations."""

    schema = providers.get_model_schema(provider_id, model.id)
    configuration = _configuration_card(schema.root, schema)
    return {
        "identity": {
            "provider_id": provider_id,
            "id": model.id,
            "name": model.name,
            "version": model.version,
            "metadata_fingerprint": model.metadata_fingerprint,
            "family_id": model.family_id,
            "physical_family": model.physical_family,
            "model_kind": model.model_kind,
            "status": model.status,
            "tags": list(model.tags),
            "operations": list(model.operations),
            "common_runner_operations": list(model.common_runner_operations),
        },
        "presentation": model.presentation.model_dump(mode="json"),
        "description": model.description,
        "composition": model.composition_advertisement.model_dump(mode="json", by_alias=True),
        "fidelity_ladder": [_fidelity_card(item) for item in model.fidelities],
        "realizations": [_realization_card(item) for item in model.realizations],
        "missions": [_mission_card(item) for item in model.mission_templates],
        "configuration": configuration,
        "provenance": _provenance_card(model, configuration),
        "tuning": _tuning_card(
            tuning_campaigns,
            provider_id=provider_id,
            model_id=model.id,
        ),
        "claim_boundary": model.claim_boundary,
    }
    ####


def _fidelity_card(fidelity: TrajectoryFidelityMetadata) -> dict[str, object]:
    """Project the public fields that distinguish one fidelity tier."""

    return {
        "id": fidelity.id,
        "label": fidelity.label,
        "rank": fidelity.rank,
        "declared": fidelity.declared,
        "dynamics_fidelity": fidelity.dynamics_fidelity,
        "input_realization": fidelity.input_realization,
        "actuator_types": list(fidelity.actuator_types),
        "compatibility_aliases": list(fidelity.compatibility_aliases),
        "runtime_fidelity": fidelity.runtime_fidelity,
        "control_realization": fidelity.control_realization,
        "promotion_status": fidelity.promotion_status,
        "operations": list(fidelity.operations),
        "profile_id": fidelity.profile_id,
        "blockers": list(fidelity.blockers),
        "required_operations": list(fidelity.required_operations),
        "claim_boundary": fidelity.claim_boundary,
    }
    ####


def _realization_card(realization: TrajectoryRealizationMetadata) -> dict[str, object]:
    """Keep the execution and control declaration intact for one realization."""

    return {
        "id": realization.id,
        "label": realization.label,
        "description": realization.description,
        "status": realization.status,
        "dynamics_fidelities": list(realization.dynamics_fidelities),
        "input_realization": realization.input_realization,
        "actuator_types": list(realization.actuator_types),
        "fidelity_ids": list(realization.fidelity_aliases),
        "mission_template_ids": list(realization.mission_template_ids),
        "operations": list(realization.operations),
        "native_factory_ids": list(realization.native_factory_ids),
        "blockers": list(realization.blockers),
        "source_refs": list(realization.source_refs),
        "control": realization.controls.model_dump(mode="json"),
        "claim_boundary": realization.claim_boundary,
    }
    ####


def _mission_card(mission: TrajectoryMissionTemplateMetadata) -> dict[str, object]:
    """Describe one fixed or open segment recipe without choosing input values."""

    if mission.open_segment_sequence is None:
        segment_sequence: dict[str, object] = {
            "kind": "fixed",
            "segment_ids": list(mission.segment_sequence),
        }
    else:
        segment_sequence = {
            "kind": "open",
            **mission.open_segment_sequence.model_dump(mode="json"),
        }
    return {
        "id": mission.id,
        "name": mission.name,
        "description": mission.description,
        "status": mission.status,
        "initialization_variants": list(mission.initialization_variants),
        "segment_sequence": segment_sequence,
        "compatible_fidelities": list(mission.compatible_fidelities),
        "operations": [item.model_dump(mode="json") for item in mission.operations],
        "provenance": mission.provenance,
        "claim_boundary": mission.claim_boundary,
    }
    ####


def _configuration_card(root: ConfigurationNode, schema: TrajectoryConfigurationSchema) -> dict[str, object]:
    """Flatten editable parameters while retaining initialization and segment shape."""

    parameters = _configuration_parameter_records(root)
    initialization_choices = [_choice_card(node, path=()) for node in _nodes_named(root, "initialization") if isinstance(node, ConfigurationChoiceSchema)]
    segment_sequences = [_sequence_card(node, path=()) for node in _nodes_named(root, "segments") if isinstance(node, ConfigurationSequenceSchema)]
    provenance_count = sum(bool(_text(item.get("provenance"))) for item in parameters)
    return {
        "schema_id": schema.schema_id,
        "schema_fingerprint": schema.fingerprint,
        "supported_fidelities": list(schema.supported_fidelities),
        "root_id": root.id,
        "parameter_count": len(parameters),
        "required_parameter_count": sum(bool(item["required"]) for item in parameters),
        "defaulted_parameter_count": sum(bool(item["default_declared"]) for item in parameters),
        "parameter_provenance_coverage": {
            "parameters_with_declared_provenance": provenance_count,
            "parameters_without_declared_provenance": len(parameters) - provenance_count,
            "claim_boundary": (
                "These are metadata-field counts only. A populated provenance field does not independently validate the source or qualify the parameter value."
            ),
        },
        "parameters": parameters,
        "initialization_choices": initialization_choices,
        "segment_sequences": segment_sequences,
        "claim_boundary": (
            "The configuration card records the exact advertised input grammar. It does not select values, infer a "
            "default where none is declared, or establish that a parameter is valid outside its declared bounds."
        ),
    }
    ####


def _configuration_parameter_records(
    node: ConfigurationNode,
    *,
    path: tuple[str, ...] = (),
) -> list[dict[str, object]]:
    """Flatten all leaf parameter declarations while preserving choice context."""

    if isinstance(node, ConfigurationParameterSchema):
        return [_parameter_card(node, path=(*path, node.id))]
    if isinstance(node, ConfigurationGroupSchema):
        return [record for child in node.children for record in _configuration_parameter_records(child, path=(*path, node.id))]
    if isinstance(node, ConfigurationChoiceSchema):
        return [record for variant in node.variants for record in _configuration_parameter_records(variant.node, path=(*path, node.id, variant.id))]
    if isinstance(node, ConfigurationSequenceSchema):
        return _configuration_parameter_records(node.item, path=(*path, f"{node.id}[]"))
    if isinstance(node, ConfigurationOptionalSchema):
        return _configuration_parameter_records(node.item, path=(*path, node.id))
    raise TypeError(f"unsupported configuration node {type(node)!r}")
    ####


def _parameter_card(parameter: ConfigurationParameterSchema, *, path: tuple[str, ...]) -> dict[str, object]:
    """Serialize the editable facts needed to author one parameter safely."""

    return {
        "path": ".".join(path),
        "id": parameter.id,
        "label": parameter.label,
        "description": parameter.description,
        "role": parameter.role,
        "value_type": parameter.value_type,
        "quantity": parameter.quantity,
        "canonical_unit": parameter.canonical_unit,
        "display_unit": parameter.display_unit,
        "required": parameter.required,
        "default_declared": parameter.default_declared,
        "default": parameter.default if parameter.default_declared else None,
        "hard_interval": _json_model(parameter.interval),
        "qualified_interval": _json_model(parameter.qualified_interval),
        "safe_extended_interval": _json_model(parameter.safe_extended_interval),
        "periodicity": _json_model(parameter.periodicity),
        "choices": list(parameter.choices),
        "availability": parameter.availability,
        "compatible_fidelities": list(parameter.compatible_fidelities),
        "transform": parameter.transform,
        "projection_policy": parameter.projection_policy,
        "visibility_note": parameter.visibility_note,
        "coupling_group": parameter.coupling_group,
        "derivation": parameter.derivation,
        "invalidations": list(parameter.invalidations),
        "frame": parameter.frame,
        "value_space": _json_model(parameter.value_space),
        "provenance": parameter.provenance,
    }
    ####


def _choice_card(choice: ConfigurationChoiceSchema, *, path: tuple[str, ...]) -> dict[str, object]:
    """Render a choice and every variant's editable parameters."""

    choice_path = (*path, choice.id)
    return {
        "id": choice.id,
        "label": choice.label,
        "description": choice.description,
        "variants": [
            {
                "id": variant.id,
                "label": variant.label,
                "description": variant.description,
                "compatible_fidelities": list(variant.compatible_fidelities),
                "parameters": _configuration_parameter_records(variant.node, path=(*choice_path, variant.id)),
            }
            for variant in choice.variants
        ],
    }
    ####


def _sequence_card(sequence: ConfigurationSequenceSchema, *, path: tuple[str, ...]) -> dict[str, object]:
    """Render a segment sequence, its templates, and the variant vocabulary."""

    item_path = (*path, f"{sequence.id}[]")
    variants: list[dict[str, object]] = []
    if isinstance(sequence.item, ConfigurationChoiceSchema):
        variants = [
            {
                "id": variant.id,
                "label": variant.label,
                "description": variant.description,
                "compatible_fidelities": list(variant.compatible_fidelities),
                "parameters": _configuration_parameter_records(variant.node, path=(*item_path, variant.id)),
            }
            for variant in sequence.item.variants
        ]
    return {
        "id": sequence.id,
        "label": sequence.label,
        "description": sequence.description,
        "minimum_items": sequence.minimum_items,
        "maximum_items": sequence.maximum_items,
        "allow_custom": sequence.allow_custom,
        "templates": [item.model_dump(mode="json") for item in sequence.templates],
        "item_parameters": _configuration_parameter_records(sequence.item, path=item_path),
        "variants": variants,
    }
    ####


def _nodes_named(node: ConfigurationNode, identifier: str) -> tuple[ConfigurationNode, ...]:
    """Return every configuration node with an exact stable identifier."""

    matches: list[ConfigurationNode] = [node] if node.id == identifier else []
    if isinstance(node, ConfigurationGroupSchema):
        for child in node.children:
            matches.extend(_nodes_named(child, identifier))
    elif isinstance(node, ConfigurationChoiceSchema):
        for variant in node.variants:
            matches.extend(_nodes_named(variant.node, identifier))
    elif isinstance(node, ConfigurationSequenceSchema | ConfigurationOptionalSchema):
        matches.extend(_nodes_named(node.item, identifier))
    return tuple(matches)
    ####


def _provenance_card(model: TrajectoryModelMetadata, configuration: Mapping[str, object]) -> dict[str, object]:
    """Collect declared sources without elevating them into qualification claims."""

    records: list[dict[str, object]] = []

    def add_record(scope: str, *, source_refs: Sequence[str] = (), provenance: str = "") -> None:
        if source_refs or provenance:
            records.append(
                {
                    "scope": scope,
                    "source_refs": list(source_refs),
                    "provenance": provenance,
                }
            )
        ####

    add_record("model", source_refs=model.source_refs, provenance=model.provenance)
    for property_metadata in model.presentation.properties:
        add_record(
            f"property:{property_metadata.id}",
            source_refs=property_metadata.source_refs,
            provenance=property_metadata.provenance,
        )
    for frame in model.reference_frames:
        add_record(
            f"reference_frame:{frame.id}",
            source_refs=frame.source_refs,
            provenance=frame.provenance,
        )
    for realization in model.realizations:
        add_record(
            f"realization:{realization.id}",
            source_refs=realization.source_refs,
            provenance="",
        )
    for mission in model.mission_templates:
        add_record(f"mission:{mission.id}", provenance=mission.provenance)

    return {
        "records": records,
        "configuration_parameter_provenance": configuration["parameter_provenance_coverage"],
        "claim_boundary": (
            "Source references and provenance labels state what the installed model advertises. They do not establish "
            "source authority, data completeness, correlation, numerical parity, or vehicle certification."
        ),
    }
    ####


def _tuning_card(
    tuning_campaigns: ControllerTuningCampaignRegistry,
    *,
    provider_id: str,
    model_id: str,
) -> dict[str, object]:
    """Describe registered tuning inputs without running their numerical screens."""

    registrations = tuple(
        item for item in tuning_campaigns.registrations if item.model_id == model_id and provider_id in (item.provider_id, *item.provider_aliases)
    )
    campaigns: list[dict[str, object]] = []
    for registration in registrations:
        definition = registration.campaign_factory().as_dict()
        if definition["campaign_id"] != registration.id:
            raise ValueError(f"tuning registration {registration.id!r} returned campaign {definition['campaign_id']!r}")
        campaigns.append(
            {
                "registration": registration.public_dict(),
                "campaign_definition": definition,
                "command": (f"taoryx model tune {provider_id} {model_id} --fidelity {registration.fidelity} --campaign {registration.id}"),
            }
        )
    return {
        "status": "registered" if campaigns else "not_registered",
        "campaigns": campaigns,
        "claim_boundary": (
            "A registered campaign publishes a reproducible local design-screen method and inputs. It does not mean "
            "the campaign has run, that it produced candidate gains, or that a resulting controller is qualified."
        ),
    }
    ####


def _render_model_card_markdown(lines: list[str], card: Mapping[str, object]) -> None:
    """Append one complete human-readable card to a Markdown document."""

    identity = _mapping(card.get("identity"))
    presentation = _mapping(card.get("presentation"))
    title = _text(presentation.get("display_name"), fallback=_text(identity.get("id")))
    lines.extend(
        (
            f"### {title}",
            "",
            _text(card.get("description")),
            "",
            f"- Model ID: `{_text(identity.get('id'))}`",
            f"- Version: `{_text(identity.get('version'))}`",
            f"- Family: `{_text(identity.get('family_id'), fallback='not declared')}`",
            f"- Kind / status: `{_text(identity.get('model_kind'))}` / `{_text(identity.get('status'))}`",
            f"- Model metadata fingerprint: `{_text(identity.get('metadata_fingerprint'))}`",
            "",
            "#### Composition surface",
            "",
        )
    )
    composition = _mapping(card.get("composition"))
    composition_features = _mapping_records(composition.get("features"))
    active_composition_features = tuple(item for item in composition_features if _text(item.get("status")) != "not_available")
    composition_rows = [
        (
            _text(item.get("category")),
            _text(item.get("id")),
            _text(item.get("status")),
            _code_list(item.get("operations")),
            _code_list(item.get("mutation_timing")),
        )
        for item in active_composition_features
    ]
    lines.extend(
        _markdown_table(
            ("Category", "Feature", "Status", "Operations", "Mutation timing"),
            composition_rows,
        )
    )
    lines.extend(
        (
            "",
            f"- Advertisement: `{_text(composition.get('schema'))}`; "
            f"{len(active_composition_features)} usable/declared/blocked features and "
            f"{len(composition_features) - len(active_composition_features)} explicitly unavailable features.",
            "",
            "#### Fidelity ladder",
            "",
        )
    )
    fidelity_rows = [
        (
            _text(item.get("id")),
            _text(item.get("dynamics_fidelity")),
            _text(item.get("control_realization")),
            _text(item.get("promotion_status")),
            _code_list(item.get("operations")),
            _code_list(item.get("blockers")),
        )
        for item in _mapping_records(card.get("fidelity_ladder"))
    ]
    lines.extend(_markdown_table(("Tier", "Dynamics", "Control realization", "Promotion", "Operations", "Blockers"), fidelity_rows))
    lines.extend(("", "#### Data provenance", ""))
    provenance = _mapping(card.get("provenance"))
    provenance_rows = [
        (
            _text(item.get("scope")),
            _text(item.get("provenance"), fallback="—"),
            _code_list(item.get("source_refs")),
        )
        for item in _mapping_records(provenance.get("records"))
    ]
    lines.extend(_markdown_table(("Scope", "Declared provenance", "Source references"), provenance_rows))
    parameter_coverage = _mapping(provenance.get("configuration_parameter_provenance"))
    lines.extend(
        (
            "",
            "- Configuration parameter provenance: "
            f"{_text(parameter_coverage.get('parameters_with_declared_provenance'), fallback='0')} declared / "
            f"{_text(parameter_coverage.get('parameters_without_declared_provenance'), fallback='0')} undeclared.",
            "",
            "#### Tuning",
            "",
        )
    )
    tuning = _mapping(card.get("tuning"))
    campaigns = _mapping_records(tuning.get("campaigns"))
    if not campaigns:
        lines.extend(("No common tuning campaign is registered for this provider/model selection.", ""))
    for campaign_record in campaigns:
        registration = _mapping(campaign_record.get("registration"))
        definition = _mapping(campaign_record.get("campaign_definition"))
        lines.extend(
            (
                f"- `{_text(registration.get('id'))}` — {_text(registration.get('description'))}",
                f"  - Fidelity: `{_text(registration.get('fidelity'))}`; strategy: `{_text(definition.get('strategy_id'))}`.",
                f"  - Command: `{_text(campaign_record.get('command'))}`",
            )
        )
        tuning_rows = [
            (
                _text(node.get("node_id")),
                _text(node.get("controller_method")),
                _compact_json(node.get("trim_target")),
                _code_list(node.get("design_state_names")),
                _code_list(node.get("design_control_names")),
            )
            for node in _mapping_records(definition.get("nodes"))
        ]
        lines.extend(("", *_markdown_table(("Operating point", "Method", "Trim target", "Design states", "Controls"), tuning_rows), ""))

    lines.extend(("#### Missions and segments", ""))
    mission_rows = [
        (
            _text(mission.get("id")),
            _text(mission.get("status")),
            _sequence_summary(_mapping(mission.get("segment_sequence"))),
            _code_list(mission.get("compatible_fidelities")),
            _operation_summary(mission.get("operations")),
        )
        for mission in _mapping_records(card.get("missions"))
    ]
    lines.extend(_markdown_table(("Mission", "Status", "Segments", "Fidelities", "Operations"), mission_rows))

    configuration = _mapping(card.get("configuration"))
    lines.extend(
        (
            "",
            "#### Configuration parameters",
            "",
            f"Schema `{_text(configuration.get('schema_id'))}`; "
            f"{_text(configuration.get('parameter_count'), fallback='0')} parameters, "
            f"{_text(configuration.get('required_parameter_count'), fallback='0')} required, and "
            f"{_text(configuration.get('defaulted_parameter_count'), fallback='0')} with declared defaults.",
            "",
        )
    )
    parameter_rows = [
        (
            _text(parameter.get("path")),
            _text(parameter.get("role")),
            _text(parameter.get("canonical_unit"), fallback="—"),
            _parameter_default(parameter),
            _interval_summary(parameter.get("hard_interval")),
            _text(parameter.get("provenance"), fallback="—"),
        )
        for parameter in _mapping_records(configuration.get("parameters"))
    ]
    lines.extend(_markdown_table(("Path", "Role", "Unit", "Required / default", "Hard bounds", "Provenance"), parameter_rows))

    for sequence in _mapping_records(configuration.get("segment_sequences")):
        lines.extend(("", f"##### Segment vocabulary: `{_text(sequence.get('id'))}`", ""))
        variant_rows = [
            (
                _text(variant.get("id")),
                _text(variant.get("description")),
                _parameter_brief(_mapping_records(variant.get("parameters"))),
            )
            for variant in _mapping_records(sequence.get("variants"))
        ]
        lines.extend(_markdown_table(("Segment", "Description", "Parameters"), variant_rows))

    lines.extend(
        (
            "",
            f"> Claim boundary: {_text(card.get('claim_boundary'))}",
            "",
        )
    )
    ####


def _markdown_table(headers: Sequence[str], rows: Sequence[Sequence[str]]) -> list[str]:
    """Render a compact CommonMark table or an explicit no-record message."""

    if not rows:
        return ["_None declared._"]
    escaped_headers = [_markdown_cell(item) for item in headers]
    rendered = [
        "| " + " | ".join(escaped_headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    rendered.extend("| " + " | ".join(_markdown_cell(item) for item in row) + " |" for row in rows)
    return rendered
    ####


def _parameter_default(parameter: Mapping[str, object]) -> str:
    """Render required/default status without implying an absent default."""

    if bool(parameter.get("default_declared")):
        return f"default `{_compact_json(parameter.get('default'))}`"
    return "required" if bool(parameter.get("required")) else "not declared"
    ####


def _parameter_brief(parameters: Sequence[Mapping[str, object]]) -> str:
    """Create a compact segment-parameter list for a Markdown table cell."""

    if not parameters:
        return "—"
    items = []
    for parameter in parameters:
        suffix = _text(parameter.get("canonical_unit"), fallback="")
        default = _parameter_default(parameter)
        items.append(f"`{_text(parameter.get('id'))}` ({suffix or default}; {default})")
    return "; ".join(items)
    ####


def _interval_summary(value: object) -> str:
    """Render the serialized explicit hard interval in conventional notation."""

    interval = _mapping(value)
    if not interval:
        return "—"
    minimum = _mapping(interval.get("minimum"))
    maximum = _mapping(interval.get("maximum"))
    minimum_value = minimum.get("value") if minimum else None
    maximum_value = maximum.get("value") if maximum else None
    left = "[" if minimum.get("inclusive", True) else "("
    right = "]" if maximum.get("inclusive", True) else ")"
    lower = "−∞" if minimum_value is None else _text(minimum_value)
    upper = "∞" if maximum_value is None else _text(maximum_value)
    return f"{left}{lower}, {upper}{right}"
    ####


def _sequence_summary(sequence: Mapping[str, object]) -> str:
    """Summarize fixed or open mission sequence metadata in one table cell."""

    if sequence.get("kind") == "fixed":
        return _code_list(sequence.get("segment_ids"))
    vocabulary = _code_list(sequence.get("allowed_segment_ids"))
    minimum = _text(sequence.get("minimum_items"), fallback="0")
    maximum = _text(sequence.get("maximum_items"), fallback="unbounded")
    return f"open {vocabulary} ({minimum}–{maximum})"
    ####


def _operation_summary(value: object) -> str:
    """Summarize mission operation availability without dropping blocked rows."""

    return "; ".join(f"`{_text(item.get('operation'))}`:{_text(item.get('status'))}" for item in _mapping_records(value)) or "—"
    ####


def _code_list(value: object) -> str:
    """Render a list of stable IDs safely inside a Markdown table cell."""

    if not isinstance(value, Sequence) or isinstance(value, str):
        return "—"
    return ", ".join(f"`{_text(item)}`" for item in value) if value else "—"
    ####


def _compact_json(value: object) -> str:
    """Serialize a small public value in a deterministic inline form."""

    if value is None:
        return "—"
    return json.dumps(value, sort_keys=True, separators=(",", ":"))
    ####


def _json_model(value: object) -> dict[str, object] | None:
    """Serialize an optional Pydantic metadata object without using ``Any`` externally."""

    if value is None:
        return None
    model_dump = getattr(value, "model_dump", None)
    if not callable(model_dump):
        raise TypeError(f"expected a Pydantic metadata object, received {type(value)!r}")
    payload = model_dump(mode="json")
    if not isinstance(payload, dict):
        raise TypeError("Pydantic metadata did not serialize to a mapping")
    return dict(payload)
    ####


def _mapping(value: object) -> Mapping[str, object]:
    """Return a safe mapping projection for JSON-shaped report content."""

    return value if isinstance(value, Mapping) else {}
    ####


def _mapping_records(value: object) -> list[Mapping[str, object]]:
    """Return mapping records from one JSON-shaped list field."""

    if not isinstance(value, Sequence) or isinstance(value, str):
        return []
    return [item for item in value if isinstance(item, Mapping)]
    ####


def _text(value: object, key: str | None = None, *, fallback: str = "—") -> str:
    """Convert a scalar or a mapping field to a compact display string."""

    if key is not None:
        value = _mapping(value).get(key)
    if value is None or value == "":
        return fallback
    return str(value)
    ####


def _markdown_cell(value: str) -> str:
    """Keep a generated cell on one CommonMark row."""

    return value.replace("|", "\\|").replace("\n", "<br>")
    ####


__all__ = [
    "MODEL_OVERVIEW_SCHEMA",
    "build_model_overview_catalog",
    "render_model_overview_markdown",
]
