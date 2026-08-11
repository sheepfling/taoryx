"""Control metadata projected from exact CADAC configuration boundaries."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Literal

from taoryx.trajectory.configuration_contract import (
    ConfigurationGroupSchema,
    ConfigurationParameterSchema,
    ConfigurationValueSpace,
    ControlCommandSemantics,
    TrajectoryConfigurationSchema,
    TrajectoryControlAdvertisement,
    TrajectoryControlAuthorityMetadata,
    TrajectoryControlChannelMetadata,
    TrajectoryControlIntentMetadata,
    TrajectoryControlNativeBindingMetadata,
)

BatchControlAuthority = Literal["effector", "wrench", "native_bridge", "mission"]


@dataclass(frozen=True, slots=True)
class CadacControlOutputEvidence:
    """Requested/realized standard-output evidence for one external control.

    Component indices retain the exact scalar-to-vector projection without
    creating a second set of scalar telemetry channels. A direct-wrench input
    may name force or moment as its realized evidence; the quantities need not
    match because that mapping is the plant response being made explicit.
    """

    requested_channel_id: str
    realized_channel_ids: tuple[str, ...]
    requested_component: int | None = None
    realized_components: tuple[int | None, ...] = ()

    def __post_init__(self) -> None:
        if not self.requested_channel_id.strip() or not self.realized_channel_ids:
            raise ValueError("CADAC control output evidence requires requested and realized channel IDs")
        if any(not channel_id.strip() for channel_id in self.realized_channel_ids):
            raise ValueError("CADAC control output evidence contains an empty realized channel ID")
        if self.requested_component is not None and self.requested_component < 0:
            raise ValueError("CADAC requested output component must be nonnegative")
        if self.realized_components and len(self.realized_components) != len(self.realized_channel_ids):
            raise ValueError("CADAC realized output components must align with realized channel IDs")
        if any(component is not None and component < 0 for component in self.realized_components):
            raise ValueError("CADAC realized output components must be nonnegative")
        ####

    def as_dict(self) -> dict[str, object]:
        """Return JSON-safe evidence embedded in the native control binding."""

        components = self.realized_components or (None,) * len(self.realized_channel_ids)
        return {
            "requested": {
                "channel_id": self.requested_channel_id,
                "component": self.requested_component,
            },
            "realized": [
                {"channel_id": channel_id, "component": component} for channel_id, component in zip(self.realized_channel_ids, components, strict=True)
            ],
        }
        ####

    ####


def batch_configuration_control_advertisement(
    schema: TrajectoryConfigurationSchema,
    *,
    channels: Mapping[str, tuple[str, str]],
    output_evidence: Mapping[str, CadacControlOutputEvidence],
    authority_id: str,
    authority: BatchControlAuthority,
    authority_description: str,
    intent_id: str,
    intent_label: str,
    intent_description: str,
    mission_ids: tuple[str, ...],
    source_refs: tuple[str, ...],
    claim_boundary: str,
) -> TrajectoryControlAdvertisement:
    """Publish caller-owned, fixed batch commands from exact schema leaves.

    CADAC source cases commonly run their own controller.  This helper is only
    for a reconstructed plant that explicitly accepts a caller value at its
    source command boundary.  The action is fixed for the submitted batch;
    no step/session authority is implied.
    """

    if set(output_evidence) != set(channels):
        missing = tuple(sorted(set(channels) - set(output_evidence)))
        unknown = tuple(sorted(set(output_evidence) - set(channels)))
        raise ValueError(f"CADAC external controls require exact requested/realized output evidence; missing={missing!r}, unknown={unknown!r}")
    ####

    records = tuple(
        _batch_control_channel(
            semantic_id=semantic_id,
            parameter=_parameter_at_path(schema, path),
            path=path,
            output_evidence=output_evidence[semantic_id],
            order=index,
            source_refs=source_refs,
            claim_boundary=claim_boundary,
        )
        for index, (semantic_id, path) in enumerate(channels.items(), start=1)
    )
    channel_ids = tuple(item.id for item in records)
    return TrajectoryControlAdvertisement(
        status="available",
        channels=records,
        authorities=(
            TrajectoryControlAuthorityMetadata(
                id=authority_id,
                authority=authority,
                availability="available_in_batch",
                channel_ids=channel_ids,
                operations=("batch",),
                description=authority_description,
                source_refs=source_refs,
                provenance="CADAC schema-to-batch-control projection",
                claim_boundary=claim_boundary,
            ),
        ),
        intents=(
            TrajectoryControlIntentMetadata(
                id=intent_id,
                label=intent_label,
                description=intent_description,
                resolution="external_channel",
                segment_ids=(),
                mission_template_ids=mission_ids,
                channel_ids=channel_ids,
                operations=("batch",),
                source_refs=source_refs,
                provenance="CADAC source command boundary",
                claim_boundary=claim_boundary,
            ),
        ),
        default_authority_id=authority_id,
        claim_boundary=claim_boundary,
    )
    ####


def _parameter_at_path(
    schema: TrajectoryConfigurationSchema,
    path: tuple[str, str],
) -> ConfigurationParameterSchema:
    """Resolve one public root-group configuration parameter exactly."""

    group_id, parameter_id = path
    if not isinstance(schema.root, ConfigurationGroupSchema):
        raise ValueError("CADAC batch controls require a root configuration group")
    ####
    for child in schema.root.children:
        if not isinstance(child, ConfigurationGroupSchema) or child.id != group_id:
            continue
        ####
        for parameter in child.children:
            if isinstance(parameter, ConfigurationParameterSchema) and parameter.id == parameter_id:
                return parameter
            ####
        ####
        break
    ####
    raise ValueError(f"CADAC control binding {group_id!r}/{parameter_id!r} does not identify a public configuration parameter")
    ####


def _batch_control_channel(
    *,
    semantic_id: str,
    parameter: ConfigurationParameterSchema,
    path: tuple[str, str],
    output_evidence: CadacControlOutputEvidence,
    order: int,
    source_refs: tuple[str, ...],
    claim_boundary: str,
) -> TrajectoryControlChannelMetadata:
    """Project one typed configuration leaf into a batch action/native pair."""

    data_type, shape = _control_shape(parameter)
    value_space = parameter.value_space or _value_space(parameter)
    semantics = ControlCommandSemantics(value_domain=_value_domain(parameter))
    native_channel_id = f"cadac.configuration.{path[0]}.{path[1]}"
    presentation = parameter.presentation.model_copy(
        update={
            "group": "controls",
            "order": order * 10,
            "control": _presentation_control(parameter),
        }
    )
    quantity = parameter.quantity or _quantity_for_parameter(parameter, semantic_id)
    native = TrajectoryControlNativeBindingMetadata(
        id=native_channel_id,
        quantity=quantity,
        canonical_unit=parameter.canonical_unit,
        data_type=data_type,
        shape=shape,
        interval=parameter.interval,
        value_space=value_space,
        semantics=semantics,
        provider_binding={
            "configuration_path": list(path),
            "write_phase": "before_batch_execution",
            "update_semantics": "fixed_for_batch",
            "output_evidence": output_evidence.as_dict(),
        },
    )
    return TrajectoryControlChannelMetadata(
        id=semantic_id,
        label=parameter.label,
        description=parameter.description,
        channel_kind="action",
        quantity=quantity,
        canonical_unit=parameter.canonical_unit,
        display_unit=parameter.display_unit,
        data_type=data_type,
        shape=shape,
        interval=parameter.interval,
        choices=parameter.choices,
        frame=parameter.frame,
        sampling_semantics="held_action",
        value_space=value_space,
        semantics=semantics,
        availability="available_in_batch",
        operations=("batch",),
        native_channel_id=native_channel_id,
        native_binding=native,
        provider_binding={
            "configuration_path": list(path),
            "source_parameter_id": parameter.id,
            "output_evidence": output_evidence.as_dict(),
        },
        presentation=presentation,
        source_refs=source_refs,
        provenance=parameter.provenance or "CADAC configuration schema",
        claim_boundary=claim_boundary,
    )
    ####


def _control_shape(
    parameter: ConfigurationParameterSchema,
) -> tuple[Literal["float64", "int64", "boolean", "string", "json"], tuple[int, ...]]:
    """Return the transport type/shape for one configuration leaf."""

    mapping: dict[str, tuple[Literal["float64", "int64", "boolean", "string", "json"], tuple[int, ...]]] = {
        "number": ("float64", ()),
        "integer": ("int64", ()),
        "boolean": ("boolean", ()),
        "string": ("string", ()),
        "enum": ("string", ()),
        "vector3": ("float64", (3,)),
        "vector4": ("float64", (4,)),
    }
    try:
        return mapping[parameter.value_type]
    except KeyError as error:
        raise ValueError(f"unsupported CADAC control value type {parameter.value_type!r}") from error
    ####


def _value_domain(parameter: ConfigurationParameterSchema) -> str:
    """Keep batch action semantics aligned with the configuration value type."""

    if parameter.value_type == "boolean":
        return "boolean"
    ####
    if parameter.value_type == "enum":
        return "enum"
    ####
    if parameter.value_type in {"vector3", "vector4"}:
        return "vector"
    ####
    return "continuous"
    ####


def _value_space(parameter: ConfigurationParameterSchema) -> ConfigurationValueSpace:
    """Construct explicit control topology where legacy CADAC schema leaves omit it."""

    if parameter.value_type == "boolean":
        return ConfigurationValueSpace(
            topology="boolean",
            representation="boolean",
            error_rule="exact_match",
            interpolation_rule="step",
        )
    ####
    if parameter.value_type == "enum":
        return ConfigurationValueSpace(
            topology="finite_set",
            representation="enum",
            error_rule="exact_match",
            interpolation_rule="step",
        )
    ####
    if parameter.value_type in {"vector3", "vector4"}:
        if parameter.id.endswith("unit_body"):
            return ConfigurationValueSpace(
                topology="unit_direction",
                representation=parameter.value_type,
                error_rule="angular_difference",
                interpolation_rule="spherical_linear",
                normalization_rule="normalize_nonzero_vector_at_provider_boundary",
            )
        ####
        component = ConfigurationValueSpace(
            topology="real_line",
            representation="scalar",
            error_rule="componentwise_difference",
            interpolation_rule="linear",
        )
        size = 3 if parameter.value_type == "vector3" else 4
        return ConfigurationValueSpace(
            topology="product",
            representation=parameter.value_type,
            error_rule="componentwise_difference",
            interpolation_rule="linear",
            components=(component,) * size,
        )
    ####
    return ConfigurationValueSpace(
        topology="real_line",
        representation="scalar",
        error_rule="difference",
        interpolation_rule="linear",
    )
    ####


def _presentation_control(parameter: ConfigurationParameterSchema) -> Literal["number_input", "slider", "toggle", "select", "vector_editor"]:
    """Supply a compact UI hint without changing the semantic value contract."""

    if parameter.value_type == "boolean":
        return "toggle"
    ####
    if parameter.value_type == "enum":
        return "select"
    ####
    if parameter.value_type in {"vector3", "vector4"}:
        return "vector_editor"
    ####
    if parameter.interval is not None:
        return "slider"
    ####
    return "number_input"
    ####


def _quantity_for_unit(unit: str | None) -> str | None:
    """Retain common CADAC physical dimensions when the source leaf omits a quantity."""

    return {
        "deg": "angle",
        "rad": "angle",
        "deg/s": "angular_velocity",
        "rad/s": "angular_velocity",
        "g": "acceleration",
        "m/s": "speed",
        "m/s^2": "acceleration",
        "N": "force",
        "N*m": "moment",
        "s": "time",
    }.get(unit)
    ####


def _quantity_for_parameter(parameter: ConfigurationParameterSchema, semantic_id: str) -> str:
    """Return a complete semantic quantity for a public CADAC control."""

    unit_quantity = _quantity_for_unit(parameter.canonical_unit)
    if unit_quantity is not None:
        return unit_quantity
    ####
    if parameter.value_type == "boolean":
        return "boolean"
    ####
    if parameter.value_type in {"enum", "string"}:
        return "categorical"
    ####
    if "unit_body" in parameter.id or ".direction" in semantic_id:
        return "direction"
    ####
    return "dimensionless"


####


def source_managed_control_advertisement(
    *,
    mission_ids: tuple[str, ...],
    source_refs: tuple[str, ...],
    claim_boundary: str,
    operations: tuple[Literal["batch", "step"], ...] = ("batch",),
) -> TrajectoryControlAdvertisement:
    """Describe source-managed control without inventing an external channel."""

    if not operations or len(operations) != len(set(operations)):
        raise ValueError("source-managed control operations must be nonempty and unique")
    availability = "available_in_batch" if operations == ("batch",) else "available"

    return TrajectoryControlAdvertisement(
        status="internally_generated",
        channels=(),
        authorities=(
            TrajectoryControlAuthorityMetadata(
                id="source_program_control",
                authority="mission",
                availability=availability,
                channel_ids=(),
                operations=operations,
                description="The source program resolves its own controller, guidance, or fixed trajectory command path at its declared source boundary.",
                source_refs=source_refs,
                provenance="CADAC source-order compatibility boundary",
                claim_boundary=claim_boundary,
            ),
        ),
        intents=(
            TrajectoryControlIntentMetadata(
                id="source_managed_control",
                label="Source-managed control",
                description="The CADAC source program supplies control at its declared module boundary.",
                resolution="provider_internal",
                segment_ids=(),
                mission_template_ids=mission_ids,
                channel_ids=(),
                operations=operations,
                source_refs=source_refs,
                provenance="CADAC source-order compatibility boundary",
                claim_boundary=claim_boundary,
            ),
        ),
        default_authority_id="source_program_control",
        claim_boundary=claim_boundary,
    )
    ####


def blocked_control_advertisement(*, claim_boundary: str) -> TrajectoryControlAdvertisement:
    """Describe an unavailable realization without pretending its controls exist."""

    return TrajectoryControlAdvertisement(
        status="blocked",
        channels=(),
        authorities=(),
        intents=(),
        claim_boundary=claim_boundary,
    )
    ####


__all__ = [
    "CadacControlOutputEvidence",
    "batch_configuration_control_advertisement",
    "blocked_control_advertisement",
    "source_managed_control_advertisement",
]
####
