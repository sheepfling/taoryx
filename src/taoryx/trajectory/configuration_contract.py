"""Portable configuration grammar for self-describing trajectory providers.

The schema in this module is deliberately separate from simulation execution.
A provider publishes a typed configuration tree; a consumer builds a matching
configuration instance; validation produces an immutable, fingerprinted
handoff.  No node contains Python callbacks or provider-local classes.
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping, Sequence
from typing import Annotated, Any, Literal, Protocol

from pydantic import BaseModel, ConfigDict, Field, model_validator

ConfigurationValueType = Literal["number", "integer", "boolean", "string", "enum", "vector3", "vector4"]
ConfigurationRole = Literal["initialization", "segment", "constraint", "variant", "output"]
FidelityTransitionDirection = Literal["step_up", "step_down"]
FidelityTransitionStatus = Literal["available", "conditional", "blocked", "not_available"]
TrajectoryDynamicsFidelity = Literal["point_mass_3dof", "pseudo_6dof", "rigid_body_6dof"]
TrajectoryInputRealization = Literal[
    "uncontrolled",
    "guidance_command",
    "direct_wrench",
    "actuator_allocated",
    "source_replay",
    "provider_defined",
]
TrajectoryActuatorType = Literal[
    "not_applicable",
    "aerodynamic_surfaces",
    "rotors",
    "thrust_vectoring",
    "gimbals",
    "rcs",
    "mixed",
    "provider_defined",
]
TrajectoryOutputDataType = Literal["float64", "int64", "boolean", "string", "json"]
TrajectorySamplingSemantics = Literal[
    "continuous_sample",
    "discrete_sample",
    "event",
    "interval",
    "static",
    "provider_reported",
]


class NumericPresentationMetadata(BaseModel):
    """Non-authoritative numeric formatting hints for generic consumers."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    style: Literal["automatic", "fixed", "scientific", "engineering", "percent"] = "automatic"
    decimal_places: int | None = Field(default=None, ge=0, le=12)
    significant_digits: int | None = Field(default=None, ge=1, le=15)
    use_grouping: bool = False
    trim_trailing_zeroes: bool = True

    @model_validator(mode="after")
    def validate_precision(self) -> NumericPresentationMetadata:
        if self.decimal_places is not None and self.significant_digits is not None:
            raise ValueError("numeric presentation cannot set both decimal_places and significant_digits")
        return self
        ####

    ####


class ValuePresentationMetadata(BaseModel):
    """Portable display hints that never change validation or canonical units."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    group: str = "general"
    order: int = 0
    visibility: Literal["primary", "advanced", "expert", "debug"] = "primary"
    control: Literal[
        "automatic",
        "number_input",
        "slider",
        "toggle",
        "select",
        "text",
        "vector_editor",
        "coordinate_picker",
    ] = "automatic"
    format: NumericPresentationMetadata | None = None
    placeholder: str | None = None
    enum_labels: dict[str, str] = Field(default_factory=dict)


class PresentationLinkMetadata(BaseModel):
    """One provider-authored link suitable for a discovery interface."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    relation: Literal["documentation", "source", "support", "homepage", "license"]
    label: str = Field(min_length=1)
    uri: str = Field(min_length=1)
    media_type: str | None = None


class TrajectoryProviderPresentationMetadata(BaseModel):
    """Provider-level names and navigation hints for presentation layers."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    display_name: str = Field(min_length=1)
    short_name: str = Field(min_length=1)
    summary: str = Field(min_length=1)
    organization: str | None = None
    categories: tuple[str, ...] = ()
    links: tuple[PresentationLinkMetadata, ...] = ()


class ConfigurationContractError(ValueError):
    """Stable fail-closed diagnostic for configuration validation."""

    def __init__(self, code: str, message: str, *, path: str = "configuration") -> None:
        self.code = code
        self.path = path
        super().__init__(f"{code}: {path}: {message}")
        ####

    ####


class ConfigurationBound(BaseModel):
    """One interval endpoint; a null value represents signed infinity."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    value: float | None = None
    inclusive: bool = True

    @model_validator(mode="after")
    def validate_value(self) -> ConfigurationBound:
        if self.value is not None and not math.isfinite(self.value):
            raise ValueError("configuration bounds must be finite or null")
        return self
        ####

    ####


class ConfigurationInterval(BaseModel):
    """Explicit bounded or unbounded numeric interval."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    minimum: ConfigurationBound | None = None
    maximum: ConfigurationBound | None = None

    @model_validator(mode="after")
    def validate_interval(self) -> ConfigurationInterval:
        minimum = self.minimum
        maximum = self.maximum
        lower = None if minimum is None else minimum.value
        upper = None if maximum is None else maximum.value
        if minimum is not None and maximum is not None and lower is not None and upper is not None:
            if lower > upper:
                raise ValueError("configuration interval has inverted bounds")
            if lower == upper and (not minimum.inclusive or not maximum.inclusive):
                raise ValueError("an open zero-width configuration interval is empty")
        return self
        ####

    ####


class ConfigurationPeriodicity(BaseModel):
    """Principal interval for a periodic scalar value."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    period: float = Field(gt=0.0)
    canonical_minimum: float = 0.0

    @model_validator(mode="after")
    def validate_periodicity(self) -> ConfigurationPeriodicity:
        if not math.isfinite(self.period) or not math.isfinite(self.canonical_minimum):
            raise ValueError("periodicity values must be finite")
        return self
        ####

    ####


class ConfigurationValueSpace(BaseModel):
    """Portable topology and interpolation semantics for a public value."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    topology: str = Field(min_length=1)
    representation: str = Field(min_length=1)
    error_rule: str = Field(min_length=1)
    interpolation_rule: str = Field(min_length=1)
    normalization_rule: str | None = None
    period: float | None = None
    equivalence: str | None = None
    coordinate_chart: str | None = None
    components: tuple[ConfigurationValueSpace, ...] = ()

    @classmethod
    def from_mapping(cls, payload: Mapping[str, object]) -> ConfigurationValueSpace:
        """Build metadata from the repository's shared value-space payload."""

        return cls.model_validate(payload)
        ####

    ####


class ConfigurationParameterSchema(BaseModel):
    """One typed leaf in a provider's configuration grammar."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["parameter"] = "parameter"
    id: str = Field(min_length=1)
    label: str = Field(min_length=1)
    description: str = Field(min_length=1)
    value_type: ConfigurationValueType = "number"
    quantity: str | None = None
    canonical_unit: str | None = None
    display_unit: str | None = None
    required: bool = False
    default: Any = None
    default_declared: bool = False
    interval: ConfigurationInterval | None = None
    qualified_interval: ConfigurationInterval | None = None
    safe_extended_interval: ConfigurationInterval | None = None
    periodicity: ConfigurationPeriodicity | None = None
    choices: tuple[str, ...] = ()
    role: ConfigurationRole
    availability: str = "available"
    compatible_fidelities: tuple[str, ...] = ()
    transform: Literal["identity", "log", "logit", "categorical"] = "identity"
    projection_policy: Literal["reject_invalid", "project_to_valid"] = "reject_invalid"
    visibility_note: str | None = None
    coupling_group: str | None = None
    derivation: str | None = None
    invalidations: tuple[str, ...] = ()
    frame: str | None = None
    value_space: ConfigurationValueSpace | None = None
    presentation: ValuePresentationMetadata = Field(default_factory=ValuePresentationMetadata)
    provenance: str = ""

    @model_validator(mode="after")
    def validate_parameter(self) -> ConfigurationParameterSchema:
        if self.value_type == "enum" and not self.choices:
            raise ValueError(f"enum parameter {self.id!r} requires choices")
        if self.value_type != "enum" and self.choices:
            raise ValueError(f"non-enum parameter {self.id!r} cannot declare choices")
        if self.periodicity is not None and self.value_type not in {"number", "integer"}:
            raise ValueError(f"periodic parameter {self.id!r} must be numeric")
        if self.transform == "categorical" and self.value_type != "enum":
            raise ValueError(f"categorical parameter {self.id!r} must use enum representation")
        if self.default_declared:
            _validate_parameter_value(self, self.default, path=f"schema.{self.id}.default", supplied_unit=self.canonical_unit)
        return self
        ####

    ####


class ConfigurationGroupSchema(BaseModel):
    """Named collection of configuration nodes that apply together."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["group"] = "group"
    id: str = Field(min_length=1)
    label: str = Field(min_length=1)
    description: str = ""
    children: tuple[ConfigurationNode, ...]

    @model_validator(mode="after")
    def validate_children(self) -> ConfigurationGroupSchema:
        _require_unique_node_ids(self.children, f"group {self.id!r}")
        return self
        ####

    ####


class ConfigurationChoiceVariant(BaseModel):
    """One structurally distinct variant of a choice node."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    label: str = Field(min_length=1)
    description: str = ""
    node: ConfigurationNode
    compatible_fidelities: tuple[str, ...] = ()


class ConfigurationChoiceSchema(BaseModel):
    """Exactly one selected configuration shape."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["choice"] = "choice"
    id: str = Field(min_length=1)
    label: str = Field(min_length=1)
    description: str = ""
    variants: tuple[ConfigurationChoiceVariant, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_variants(self) -> ConfigurationChoiceSchema:
        ids = tuple(item.id for item in self.variants)
        if len(ids) != len(set(ids)):
            raise ValueError(f"choice {self.id!r} has duplicate variants")
        return self
        ####

    ####


class ConfigurationSequenceTemplate(BaseModel):
    """One advertised, ordered recipe through a sequence choice grammar."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    label: str = Field(min_length=1)
    description: str = ""
    item_variants: tuple[str, ...] = Field(min_length=1)
    compatible_fidelities: tuple[str, ...] = ()


class ConfigurationSequenceSchema(BaseModel):
    """Variable-length ordered collection of one advertised item grammar."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["sequence"] = "sequence"
    id: str = Field(min_length=1)
    label: str = Field(min_length=1)
    description: str = ""
    item: ConfigurationNode
    minimum_items: int = Field(default=0, ge=0)
    maximum_items: int | None = Field(default=None, ge=0)
    templates: tuple[ConfigurationSequenceTemplate, ...] = ()
    allow_custom: bool = True

    @model_validator(mode="after")
    def validate_cardinality(self) -> ConfigurationSequenceSchema:
        if self.maximum_items is not None and self.maximum_items < self.minimum_items:
            raise ValueError(f"sequence {self.id!r} has maximum_items below minimum_items")
        if not self.allow_custom and not self.templates:
            raise ValueError(f"closed sequence {self.id!r} requires at least one template")
        template_ids = tuple(item.id for item in self.templates)
        if len(template_ids) != len(set(template_ids)):
            raise ValueError(f"sequence {self.id!r} has duplicate template IDs")
        if self.templates:
            if not isinstance(self.item, ConfigurationChoiceSchema):
                raise ValueError(f"templated sequence {self.id!r} requires a choice item")
            variant_ids = {item.id for item in self.item.variants}
            for template in self.templates:
                unknown = sorted(set(template.item_variants) - variant_ids)
                if unknown:
                    raise ValueError(f"sequence template {template.id!r} references unknown variants {unknown!r}")
                count = len(template.item_variants)
                if count < self.minimum_items or (self.maximum_items is not None and count > self.maximum_items):
                    raise ValueError(f"sequence template {template.id!r} violates sequence cardinality")
        return self
        ####

    ####


class ConfigurationOptionalSchema(BaseModel):
    """Explicitly present or absent configuration subtree."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["optional"] = "optional"
    id: str = Field(min_length=1)
    label: str = Field(min_length=1)
    description: str = ""
    item: ConfigurationNode


ConfigurationNode = Annotated[
    ConfigurationParameterSchema | ConfigurationGroupSchema | ConfigurationChoiceSchema | ConfigurationSequenceSchema | ConfigurationOptionalSchema,
    Field(discriminator="kind"),
]


class ConfigurationParameterValue(BaseModel):
    """One typed leaf value and its explicit canonical unit."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["parameter"] = "parameter"
    value: Any
    unit: str | None = None


class ConfigurationGroupValue(BaseModel):
    """Values keyed by the stable IDs of a group schema's children."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["group"] = "group"
    values: dict[str, ConfigurationNodeValue] = Field(default_factory=dict)


class ConfigurationChoiceValue(BaseModel):
    """One selected choice variant and its typed value subtree."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["choice"] = "choice"
    selected: str = Field(min_length=1)
    instance_id: str | None = Field(default=None, min_length=1)
    value: ConfigurationNodeValue


class ConfigurationSequenceValue(BaseModel):
    """Ordered values for a sequence schema."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["sequence"] = "sequence"
    items: tuple[ConfigurationNodeValue, ...] = ()

    @model_validator(mode="after")
    def validate_instance_ids(self) -> ConfigurationSequenceValue:
        instance_ids = tuple(
            item.instance_id
            for item in self.items
            if isinstance(item, ConfigurationChoiceValue) and item.instance_id is not None
        )
        if len(instance_ids) != len(set(instance_ids)):
            raise ValueError("sequence choice instance IDs must be unique")
        return self
        ####

    ####


class ConfigurationOptionalValue(BaseModel):
    """Explicit optional-node selection."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["optional"] = "optional"
    enabled: bool
    value: ConfigurationNodeValue | None = None

    @model_validator(mode="after")
    def validate_selection(self) -> ConfigurationOptionalValue:
        if self.enabled != (self.value is not None):
            raise ValueError("enabled optional values require a value and disabled values require null")
        return self
        ####

    ####


ConfigurationNodeValue = Annotated[
    ConfigurationParameterValue | ConfigurationGroupValue | ConfigurationChoiceValue | ConfigurationSequenceValue | ConfigurationOptionalValue,
    Field(discriminator="kind"),
]


class TrajectoryModelPropertyMetadata(BaseModel):
    """One typed, read-only fact published about a trajectory model.

    Properties use the same quantity/unit/value-space vocabulary as mutable
    configuration parameters, but they are descriptive facts rather than run
    inputs.  A property may publish either a scalar value or a range.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    label: str = Field(min_length=1)
    description: str = Field(min_length=1)
    semantic_role: Literal["identity", "geometry", "mass", "performance", "capability", "evidence", "implementation"]
    value_type: ConfigurationValueType
    value_kind: Literal["exact", "nominal", "representative", "limit", "range", "declared", "unknown"]
    value: Any = None
    value_declared: bool = False
    interval: ConfigurationInterval | None = None
    choices: tuple[str, ...] = ()
    quantity: str | None = None
    canonical_unit: str | None = None
    display_unit: str | None = None
    periodicity: ConfigurationPeriodicity | None = None
    frame: str | None = None
    value_space: ConfigurationValueSpace | None = None
    presentation: ValuePresentationMetadata = Field(default_factory=ValuePresentationMetadata)
    source_refs: tuple[str, ...] = ()
    provenance: str = ""
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_property(self) -> TrajectoryModelPropertyMetadata:
        if self.value_type == "enum" and not self.choices:
            raise ValueError(f"enum model property {self.id!r} requires choices")
        if self.value_type != "enum" and self.choices:
            raise ValueError(f"non-enum model property {self.id!r} cannot declare choices")
        if self.display_unit is not None and self.canonical_unit is None:
            raise ValueError(f"model property {self.id!r} cannot advertise a display unit without a canonical unit")
        if self.value_type in {"boolean", "string", "enum"} and (self.canonical_unit is not None or self.display_unit is not None):
            raise ValueError(f"non-numeric model property {self.id!r} cannot advertise units")
        if self.value_kind == "range" and self.interval is None:
            raise ValueError(f"range model property {self.id!r} requires an interval")
        if self.value_kind == "unknown" and (self.value_declared or self.interval is not None):
            raise ValueError(f"unknown model property {self.id!r} cannot declare a value or interval")
        if self.value_kind != "unknown" and not self.value_declared and self.interval is None:
            raise ValueError(f"model property {self.id!r} requires a value or interval")
        if self.value_declared:
            descriptor = ConfigurationParameterSchema(
                id=self.id,
                label=self.label,
                description=self.description,
                value_type=self.value_type,
                quantity=self.quantity,
                canonical_unit=self.canonical_unit,
                display_unit=self.display_unit,
                interval=self.interval,
                periodicity=self.periodicity,
                choices=self.choices,
                role="output",
                frame=self.frame,
                value_space=self.value_space,
            )
            _validate_parameter_value(descriptor, self.value, path=f"model_property.{self.id}", supplied_unit=self.canonical_unit)
        return self
        ####

    ####


class TrajectoryModelPresentationMetadata(BaseModel):
    """Model-level discovery and default-view hints for generic consumers."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    display_name: str = Field(min_length=1)
    short_name: str = Field(min_length=1)
    summary: str = Field(min_length=1)
    category: str = Field(min_length=1)
    subcategory: str | None = None
    sort_key: str = Field(min_length=1)
    badges: tuple[str, ...] = ()
    default_fidelity_id: str | None = None
    default_mission_template_id: str | None = None
    default_output_channel_ids: tuple[str, ...] = ()
    properties: tuple[TrajectoryModelPropertyMetadata, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_properties(self) -> TrajectoryModelPresentationMetadata:
        property_ids = tuple(item.id for item in self.properties)
        if len(property_ids) != len(set(property_ids)):
            raise ValueError("model presentation contains duplicate property IDs")
        if len(self.default_output_channel_ids) != len(set(self.default_output_channel_ids)):
            raise ValueError("model presentation contains duplicate default output channels")
        return self
        ####

    ####


class TrajectoryReferenceFrameMetadata(BaseModel):
    """Discoverable coordinate-frame semantics used by parameters or outputs."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    name: str = Field(min_length=1)
    description: str = Field(min_length=1)
    frame_kind: Literal["inertial", "earth_fixed", "local_tangent", "body", "wind", "geodetic", "provider_defined"]
    axes: tuple[str, ...] = Field(min_length=1)
    handedness: Literal["right", "left", "not_applicable"]
    origin: str = Field(min_length=1)
    orientation: str = Field(min_length=1)
    source_refs: tuple[str, ...] = ()
    provenance: str = ""


class TrajectoryOutputChannelMetadata(BaseModel):
    """One output channel a model may emit through the common result shape."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    label: str = Field(min_length=1)
    description: str = Field(min_length=1)
    quantity: str | None = None
    canonical_unit: str | None = None
    display_unit: str | None = None
    data_type: TrajectoryOutputDataType = "float64"
    shape: tuple[int | Literal["variable"], ...] = ()
    frame: str | None = None
    sampling_semantics: TrajectorySamplingSemantics = "continuous_sample"
    interpolation: Literal["linear", "step", "periodic", "slerp", "event"] = "linear"
    periodicity: ConfigurationPeriodicity | None = None
    value_space: ConfigurationValueSpace | None = None
    availability: Literal["guaranteed", "conditional", "runtime_reported"] = "conditional"
    compatible_fidelities: tuple[str, ...] = ()
    compatible_realizations: tuple[str, ...] = ()
    compatible_mission_templates: tuple[str, ...] = ()
    operations: tuple[Literal["batch", "step"], ...] = ("batch", "step")
    presentation: ValuePresentationMetadata = Field(default_factory=ValuePresentationMetadata)
    source_refs: tuple[str, ...] = ()
    provenance: str = ""
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_channel(self) -> TrajectoryOutputChannelMetadata:
        if self.display_unit is not None and self.canonical_unit is None:
            raise ValueError(f"output channel {self.id!r} cannot advertise a display unit without a canonical unit")
        if any(item != "variable" and item <= 0 for item in self.shape):
            raise ValueError(f"output channel {self.id!r} has a non-positive shape dimension")
        if self.data_type in {"boolean", "string", "json"} and self.canonical_unit is not None:
            raise ValueError(f"non-numeric output channel {self.id!r} cannot advertise canonical units")
        if self.data_type in {"boolean", "string", "json"} and self.interpolation not in {"step", "event"}:
            raise ValueError(f"non-numeric output channel {self.id!r} requires step or event interpolation")
        if self.interpolation == "periodic" and self.periodicity is None:
            raise ValueError(f"periodic output channel {self.id!r} requires periodicity metadata")
        if self.interpolation == "event" and self.sampling_semantics != "event":
            raise ValueError(f"event output channel {self.id!r} requires event sampling semantics")
        if self.sampling_semantics == "event" and self.interpolation != "event":
            raise ValueError(f"event-sampled output channel {self.id!r} requires event interpolation")
        if len(self.operations) != len(set(self.operations)):
            raise ValueError(f"output channel {self.id!r} has duplicate operations")
        return self
    ####

    ####


class TrajectoryTelemetryGroupMetadata(BaseModel):
    """Named, requestable collection of model-specific telemetry channels."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    label: str = Field(min_length=1)
    description: str = Field(min_length=1)
    channel_ids: tuple[str, ...] = Field(min_length=1)
    default_selected: bool = False
    presentation: ValuePresentationMetadata = Field(default_factory=ValuePresentationMetadata)

    @model_validator(mode="after")
    def validate_channels(self) -> TrajectoryTelemetryGroupMetadata:
        if len(self.channel_ids) != len(set(self.channel_ids)):
            raise ValueError(f"telemetry group {self.id!r} contains duplicate channel IDs")
        return self
        ####

    ####


class TrajectoryEntityOutputMetadata(BaseModel):
    """Advertised entity-graph behavior of one model's output contract."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    supports_multiple_entities: bool = False
    supports_dynamic_spawning: bool = False
    supports_recursive_spawning: bool = False
    maximum_descendant_depth: int | None = Field(default=0, ge=0)
    relationship_kinds: tuple[Literal["release", "separation", "deployment"], ...] = ()
    child_output_schema_policy: Literal[
        "not_applicable",
        "same_as_parent",
        "child_model_schema",
        "runtime_reported",
    ] = "not_applicable"
    includes_spawn_initial_state: bool = False
    includes_lifecycle_events: bool = True

    @model_validator(mode="after")
    def validate_entity_capabilities(self) -> TrajectoryEntityOutputMetadata:
        if len(self.relationship_kinds) != len(set(self.relationship_kinds)):
            raise ValueError("entity output metadata contains duplicate relationship kinds")
        if self.supports_dynamic_spawning and not self.supports_multiple_entities:
            raise ValueError("dynamic spawning requires multi-entity output support")
        if self.supports_recursive_spawning and not self.supports_dynamic_spawning:
            raise ValueError("recursive spawning requires dynamic spawning")
        if self.supports_dynamic_spawning and not self.relationship_kinds:
            raise ValueError("dynamic spawning requires at least one relationship kind")
        if self.supports_dynamic_spawning and not self.includes_spawn_initial_state:
            raise ValueError("dynamic spawning requires an advertised spawn initial-state record")
        if not self.supports_multiple_entities and self.child_output_schema_policy != "not_applicable":
            raise ValueError("a single-entity output schema cannot advertise a child output-schema policy")
        if not self.supports_dynamic_spawning and self.maximum_descendant_depth != 0:
            raise ValueError("a non-spawning output schema must declare zero descendant depth")
        if self.supports_dynamic_spawning and self.maximum_descendant_depth == 0:
            raise ValueError("dynamic spawning requires a positive or unbounded descendant depth")
        if self.supports_recursive_spawning and self.maximum_descendant_depth == 1:
            raise ValueError("recursive spawning requires at least two descendant generations")
        if not self.supports_recursive_spawning and self.maximum_descendant_depth not in {0, 1}:
            raise ValueError("non-recursive spawning cannot advertise more than one descendant generation")
        return self
        ####

    ####


class TrajectoryOutputSchema(BaseModel):
    """Portable description of required core state and selectable telemetry."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-provider-output-schema/v1"] = Field(
        default="taoryx.trajectory-provider-output-schema/v1",
        alias="schema",
        serialization_alias="schema",
    )
    model_id: str = Field(min_length=1)
    model_version: str = Field(min_length=1)
    core_channels: tuple[TrajectoryOutputChannelMetadata, ...] = Field(min_length=1)
    telemetry_channels: tuple[TrajectoryOutputChannelMetadata, ...] = ()
    telemetry_groups: tuple[TrajectoryTelemetryGroupMetadata, ...] = ()
    entity_output: TrajectoryEntityOutputMetadata = Field(default_factory=TrajectoryEntityOutputMetadata)
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_output_contract(self) -> TrajectoryOutputSchema:
        nonguaranteed_core = sorted(item.id for item in self.core_channels if item.availability != "guaranteed")
        if nonguaranteed_core:
            raise ValueError(
                f"output schema {self.model_id!r} has non-guaranteed core channels {nonguaranteed_core!r}"
            )
        channel_ids = tuple(item.id for item in self.channels)
        if len(channel_ids) != len(set(channel_ids)):
            raise ValueError(f"output schema {self.model_id!r} contains duplicate channel IDs")
        group_ids = tuple(item.id for item in self.telemetry_groups)
        if len(group_ids) != len(set(group_ids)):
            raise ValueError(f"output schema {self.model_id!r} contains duplicate telemetry group IDs")
        telemetry_ids = {item.id for item in self.telemetry_channels}
        grouped_ids: list[str] = []
        for group in self.telemetry_groups:
            unknown = sorted(set(group.channel_ids) - telemetry_ids)
            if unknown:
                raise ValueError(f"telemetry group {group.id!r} references non-telemetry channels {unknown!r}")
            grouped_ids.extend(group.channel_ids)
        ungrouped = sorted(telemetry_ids - set(grouped_ids))
        if ungrouped:
            raise ValueError(f"output schema {self.model_id!r} has ungrouped telemetry channels {ungrouped!r}")
        repeated = sorted({item for item in grouped_ids if grouped_ids.count(item) > 1})
        if repeated:
            raise ValueError(f"output schema {self.model_id!r} assigns telemetry channels to multiple groups {repeated!r}")
        return self
        ####

    @property
    def channels(self) -> tuple[TrajectoryOutputChannelMetadata, ...]:
        """Return required core channels followed by optional telemetry channels."""

        return (*self.core_channels, *self.telemetry_channels)
        ####

    @property
    def fingerprint(self) -> str:
        """Return a stable identity for the complete advertised output surface."""

        payload = self.model_dump(mode="json", by_alias=True)
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()
        ####

    ####


class TrajectoryFidelityMetadata(BaseModel):
    """One advertised realization tier and its evidence/execution boundary."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    label: str = Field(min_length=1)
    rank: int = Field(ge=0)
    declared: bool
    dynamics_fidelity: TrajectoryDynamicsFidelity
    input_realization: TrajectoryInputRealization
    actuator_types: tuple[TrajectoryActuatorType, ...] = ("not_applicable",)
    compatibility_aliases: tuple[str, ...] = ()
    runtime_fidelity: str = Field(min_length=1)
    control_realization: str = Field(min_length=1)
    promotion_status: str = Field(min_length=1)
    operations: tuple[Literal["validate", "batch", "step"], ...] = ("validate",)
    profile_id: str | None = None
    blockers: tuple[str, ...] = ()
    required_operations: tuple[str, ...] = ()
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_normalized_fidelity(self) -> TrajectoryFidelityMetadata:
        if len(self.actuator_types) != len(set(self.actuator_types)):
            raise ValueError(f"fidelity {self.id!r} has duplicate actuator types")
        if len(self.compatibility_aliases) != len(set(self.compatibility_aliases)):
            raise ValueError(f"fidelity {self.id!r} has duplicate compatibility aliases")
        if self.input_realization != "actuator_allocated" and self.actuator_types != ("not_applicable",):
            raise ValueError(f"fidelity {self.id!r} advertises actuators without actuator-allocated input")
        if self.input_realization == "actuator_allocated" and self.actuator_types == ("not_applicable",):
            raise ValueError(f"actuator-allocated fidelity {self.id!r} requires an actuator type")
        return self
        ####

    ####


class TrajectoryRealizationMetadata(BaseModel):
    """One selectable model realization, separate from dynamics fidelity."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    label: str = Field(min_length=1)
    description: str = Field(min_length=1)
    status: Literal["available", "blocked", "unsupported"]
    dynamics_fidelities: tuple[TrajectoryDynamicsFidelity, ...] = Field(min_length=1)
    input_realization: TrajectoryInputRealization
    actuator_types: tuple[TrajectoryActuatorType, ...] = ("not_applicable",)
    fidelity_aliases: tuple[str, ...] = Field(min_length=1)
    mission_template_ids: tuple[str, ...] = ()
    operations: tuple[Literal["validate", "batch", "step"], ...] = ("validate",)
    native_factory_ids: tuple[str, ...] = ()
    blockers: tuple[str, ...] = ()
    source_refs: tuple[str, ...] = ()
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_realization(self) -> TrajectoryRealizationMetadata:
        if len(self.dynamics_fidelities) != len(set(self.dynamics_fidelities)):
            raise ValueError(f"realization {self.id!r} has duplicate dynamics fidelities")
        if len(self.actuator_types) != len(set(self.actuator_types)):
            raise ValueError(f"realization {self.id!r} has duplicate actuator types")
        if len(self.fidelity_aliases) != len(set(self.fidelity_aliases)):
            raise ValueError(f"realization {self.id!r} has duplicate fidelity aliases")
        if len(self.operations) != len(set(self.operations)):
            raise ValueError(f"realization {self.id!r} has duplicate operations")
        if self.status != "available" and any(item in {"batch", "step"} for item in self.operations):
            raise ValueError(f"unavailable realization {self.id!r} cannot advertise execution operations")
        if self.status == "blocked" and not self.blockers:
            raise ValueError(f"blocked realization {self.id!r} requires blockers")
        if self.input_realization != "actuator_allocated" and self.actuator_types != ("not_applicable",):
            raise ValueError(f"realization {self.id!r} advertises actuators without actuator allocation")
        if self.input_realization == "actuator_allocated" and self.actuator_types == ("not_applicable",):
            raise ValueError(f"actuator-allocated realization {self.id!r} requires an actuator type")
        return self
        ####

    ####


class TrajectoryModelCapabilities(BaseModel):
    """Explicit structural capabilities that consumers must never infer."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    initialization_modes: tuple[str, ...] = Field(min_length=1)
    segment_types: tuple[str, ...] = Field(min_length=1)
    termination_modes: tuple[str, ...] = Field(min_length=1)
    operations: tuple[Literal["discover", "validate", "batch", "step"], ...] = ("discover", "validate")
    supports_custom_segments: bool = False
    supports_deployment: bool = False
    supports_staging: bool = False
    supports_dynamic_child_generation: bool = False
    supports_multiple_stages: bool = False
    supports_submodels: bool = False

    @model_validator(mode="after")
    def validate_capabilities(self) -> TrajectoryModelCapabilities:
        for name in ("initialization_modes", "segment_types", "termination_modes", "operations"):
            values = getattr(self, name)
            if len(values) != len(set(values)):
                raise ValueError(f"model capabilities contain duplicate {name}")
        if self.supports_dynamic_child_generation and not self.supports_deployment:
            raise ValueError("dynamic child generation requires deployment support")
        if self.supports_multiple_stages and not self.supports_staging:
            raise ValueError("multiple-stage support requires staging support")
        return self
        ####

    ####


class TrajectoryFidelityTransition(BaseModel):
    """Explicit adjacent movement through a model's fidelity ladder."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    from_fidelity: str = Field(min_length=1)
    to_fidelity: str = Field(min_length=1)
    direction: FidelityTransitionDirection
    status: FidelityTransitionStatus
    automatic: bool = False
    selection_policy: Literal["exact_only", "validated_lower_only", "explicit_upgrade_only"]
    requirements: tuple[str, ...] = ()
    state_transfer: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_transition(self) -> TrajectoryFidelityTransition:
        if self.from_fidelity == self.to_fidelity:
            raise ValueError("a fidelity transition must change tiers")
        if self.direction == "step_up" and self.automatic:
            raise ValueError("fidelity upgrades must be explicitly requested")
        if self.automatic and self.selection_policy != "validated_lower_only":
            raise ValueError("automatic transitions require validated_lower_only policy")
        return self
        ####

    ####


class TrajectoryMissionOperationMetadata(BaseModel):
    """Exact operation availability for one mission-template/fidelity pair."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    fidelity: str = Field(min_length=1)
    realization_id: str | None = None
    operation: Literal["validate", "batch", "step"]
    status: Literal["available", "blocked"]
    execution_mode: str | None = None
    availability_scope: Literal["provider_interface", "native_runtime_binding"] = "provider_interface"
    common_runner_status: Literal["registered", "adapter_required", "not_available"] = "not_available"
    executor_id: str | None = None
    blockers: tuple[str, ...] = ()
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_availability(self) -> TrajectoryMissionOperationMetadata:
        if self.status == "blocked" and not self.blockers:
            raise ValueError("a blocked mission operation requires at least one blocker")
        if self.status == "blocked" and self.common_runner_status != "not_available":
            raise ValueError("a blocked mission operation cannot advertise common-runner dispatch")
        if self.common_runner_status == "registered" and self.status != "available":
            raise ValueError("a registered common-runner operation must be available")
        if self.executor_id is not None and self.common_runner_status == "not_available":
            raise ValueError("an unavailable common-runner operation cannot advertise an executor ID")
        return self
        ####

    ####


class TrajectoryMissionTemplateMetadata(BaseModel):
    """One advertised ordered mission recipe and its exact operation matrix."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    name: str = Field(min_length=1)
    description: str = Field(min_length=1)
    status: str = Field(min_length=1)
    initialization_variants: tuple[str, ...] = Field(min_length=1)
    segment_sequence: tuple[str, ...] = Field(min_length=1)
    compatible_fidelities: tuple[str, ...] = Field(min_length=1)
    operations: tuple[TrajectoryMissionOperationMetadata, ...] = Field(min_length=1)
    provenance: str = ""
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_operation_keys(self) -> TrajectoryMissionTemplateMetadata:
        keys = tuple((item.fidelity, item.realization_id, item.operation) for item in self.operations)
        if len(keys) != len(set(keys)):
            raise ValueError(f"mission template {self.id!r} has duplicate operation records")
        unknown = sorted({item.fidelity for item in self.operations} - set(self.compatible_fidelities))
        if unknown:
            raise ValueError(f"mission template {self.id!r} has operations for incompatible fidelities {unknown!r}")
        return self
        ####

    ####


class TrajectoryDeploymentMetadata(BaseModel):
    """One advertised parent-to-child emission or deployment capability."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    name: str = Field(min_length=1)
    description: str = Field(min_length=1)
    status: Literal["available", "declared", "blocked", "not_available"]
    trigger_segment_ids: tuple[str, ...] = Field(min_length=1)
    trigger_event_kinds: tuple[str, ...] = Field(min_length=1)
    child_role: str = Field(min_length=1)
    child_model_id: str | None = None
    child_model_scope: Literal["provider_catalog", "provider_generated", "external"]
    child_model_kind: str = Field(min_length=1)
    minimum_children: int = Field(default=0, ge=0)
    maximum_children: int | None = Field(default=None, ge=1)
    compatible_fidelities: tuple[str, ...] = Field(min_length=1)
    operations: tuple[Literal["batch", "step"], ...] = ()
    availability_scope: Literal["provider_interface", "native_runtime_binding"] = "provider_interface"
    common_runner_status: Literal["registered", "adapter_required", "not_available"] = "not_available"
    executor_id: str | None = None
    state_initialization: Literal[
        "inherited_at_accepted_boundary",
        "provider_defined_at_accepted_boundary",
        "external",
        "not_advertised",
    ]
    fidelity_policy: Literal["inherit_parent", "explicit_child", "provider_mapped", "external"]
    lifecycle: Literal["independently_propagated", "attached_only", "event_only", "external"]
    blockers: tuple[str, ...] = ()
    source_refs: tuple[str, ...] = ()
    provenance: str = ""
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_deployment(self) -> TrajectoryDeploymentMetadata:
        if self.maximum_children is not None and self.maximum_children < self.minimum_children:
            raise ValueError(f"deployment {self.id!r} has maximum_children below minimum_children")
        if len(self.trigger_segment_ids) != len(set(self.trigger_segment_ids)):
            raise ValueError(f"deployment {self.id!r} has duplicate trigger segment IDs")
        if len(self.trigger_event_kinds) != len(set(self.trigger_event_kinds)):
            raise ValueError(f"deployment {self.id!r} has duplicate trigger event kinds")
        if len(self.operations) != len(set(self.operations)):
            raise ValueError(f"deployment {self.id!r} has duplicate operations")
        if self.status == "available" and not self.operations:
            raise ValueError(f"available deployment {self.id!r} requires an operation")
        if self.status in {"declared", "blocked", "not_available"} and self.operations:
            raise ValueError(f"unavailable deployment {self.id!r} cannot advertise operations")
        if self.status == "blocked" and not self.blockers:
            raise ValueError(f"blocked deployment {self.id!r} requires blockers")
        if self.child_model_scope == "provider_catalog" and self.child_model_id is None:
            raise ValueError(f"catalog child deployment {self.id!r} requires child_model_id")
        if self.lifecycle == "independently_propagated" and self.child_model_id is None:
            raise ValueError(f"independently propagated deployment {self.id!r} requires child_model_id")
        if self.lifecycle == "event_only" and self.operations:
            raise ValueError(f"event-only deployment {self.id!r} cannot advertise child propagation")
        if self.status != "available" and self.common_runner_status != "not_available":
            raise ValueError(f"unavailable deployment {self.id!r} cannot advertise common-runner dispatch")
        if self.common_runner_status == "registered" and not self.operations:
            raise ValueError(f"registered deployment {self.id!r} requires an operation")
        if self.executor_id is not None and self.common_runner_status == "not_available":
            raise ValueError(f"deployment {self.id!r} cannot advertise an unavailable executor")
        return self
        ####

    ####


class TrajectoryModelMetadata(BaseModel):
    """Common provider-independent model identity and capability metadata."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    name: str = Field(min_length=1)
    version: str = Field(min_length=1)
    description: str = Field(min_length=1)
    presentation: TrajectoryModelPresentationMetadata
    family_id: str | None = None
    physical_family: str | None = None
    model_kind: str = Field(min_length=1)
    status: str = Field(min_length=1)
    tags: tuple[str, ...] = ()
    operations: tuple[Literal["discover", "validate", "batch", "step"], ...] = ("discover", "validate")
    common_runner_operations: tuple[Literal["batch", "step"], ...] = ()
    capabilities: TrajectoryModelCapabilities
    realizations: tuple[TrajectoryRealizationMetadata, ...] = Field(min_length=1)
    mission_templates: tuple[TrajectoryMissionTemplateMetadata, ...] = ()
    deployments: tuple[TrajectoryDeploymentMetadata, ...] = ()
    reference_frames: tuple[TrajectoryReferenceFrameMetadata, ...] = ()
    output_schema: TrajectoryOutputSchema
    output_schema_id: str = Field(min_length=1)
    output_schema_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    configuration_schema_id: str = Field(min_length=1)
    configuration_schema_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    fidelities: tuple[TrajectoryFidelityMetadata, ...] = Field(min_length=1)
    fidelity_transitions: tuple[TrajectoryFidelityTransition, ...] = ()
    source_refs: tuple[str, ...] = ()
    provenance: str = ""
    claim_boundary: str = Field(min_length=1)

    @property
    def output_channels(self) -> tuple[TrajectoryOutputChannelMetadata, ...]:
        """Compatibility view of every channel in the model's output schema."""

        return self.output_schema.channels
        ####

    @model_validator(mode="after")
    def validate_fidelity_graph(self) -> TrajectoryModelMetadata:
        fidelity_ids = tuple(item.id for item in self.fidelities)
        if len(fidelity_ids) != len(set(fidelity_ids)):
            raise ValueError(f"model {self.id!r} has duplicate fidelity metadata")
        known = set(fidelity_ids)
        realization_ids = tuple(item.id for item in self.realizations)
        if len(realization_ids) != len(set(realization_ids)):
            raise ValueError(f"model {self.id!r} has duplicate realization metadata")
        known_realizations = set(realization_ids)
        if len(self.operations) != len(set(self.operations)):
            raise ValueError(f"model {self.id!r} has duplicate operations")
        if len(self.common_runner_operations) != len(set(self.common_runner_operations)):
            raise ValueError(f"model {self.id!r} has duplicate common-runner operations")
        unavailable_common = sorted(
            {str(item) for item in self.common_runner_operations} - {str(item) for item in self.operations}
        )
        if unavailable_common:
            raise ValueError(f"model {self.id!r} exposes unavailable common-runner operations {unavailable_common!r}")
        for transition in self.fidelity_transitions:
            if transition.from_fidelity not in known or transition.to_fidelity not in known:
                raise ValueError(f"model {self.id!r} fidelity transition references an unknown tier")
        for mission in self.mission_templates:
            unknown = sorted(set(mission.compatible_fidelities) - known)
            if unknown:
                raise ValueError(f"model {self.id!r} mission {mission.id!r} references unknown fidelities {unknown!r}")
            unknown_realizations = sorted(
                {item.realization_id for item in mission.operations if item.realization_id is not None}
                - known_realizations
            )
            if unknown_realizations:
                raise ValueError(
                    f"model {self.id!r} mission {mission.id!r} references unknown realizations {unknown_realizations!r}"
                )
        deployment_ids = tuple(item.id for item in self.deployments)
        if len(deployment_ids) != len(set(deployment_ids)):
            raise ValueError(f"model {self.id!r} has duplicate deployment metadata")
        for deployment in self.deployments:
            unknown = sorted(set(deployment.compatible_fidelities) - known)
            if unknown:
                raise ValueError(f"model {self.id!r} deployment {deployment.id!r} references unknown fidelities {unknown!r}")
        independently_propagated = tuple(
            item for item in self.deployments if item.lifecycle == "independently_propagated"
        )
        if bool(independently_propagated) != self.output_schema.entity_output.supports_dynamic_spawning:
            raise ValueError(
                f"model {self.id!r} deployment publication and dynamic entity-output capability disagree"
            )
        frame_ids = tuple(item.id for item in self.reference_frames)
        if len(frame_ids) != len(set(frame_ids)):
            raise ValueError(f"model {self.id!r} has duplicate reference-frame metadata")
        channel_ids = tuple(item.id for item in self.output_channels)
        if len(channel_ids) != len(set(channel_ids)):
            raise ValueError(f"model {self.id!r} has duplicate output-channel metadata")
        for channel in self.output_channels:
            unknown = sorted(set(channel.compatible_fidelities) - known)
            if unknown:
                raise ValueError(f"model {self.id!r} output channel {channel.id!r} references unknown fidelities {unknown!r}")
            unknown_realizations = sorted(set(channel.compatible_realizations) - known_realizations)
            if unknown_realizations:
                raise ValueError(
                    f"model {self.id!r} output channel {channel.id!r} references unknown realizations {unknown_realizations!r}"
                )
            if channel.frame is not None and channel.frame not in set(frame_ids):
                raise ValueError(f"model {self.id!r} output channel {channel.id!r} references unknown frame {channel.frame!r}")
        if self.presentation.default_fidelity_id is not None and self.presentation.default_fidelity_id not in known:
            raise ValueError(f"model {self.id!r} presentation references an unknown default fidelity")
        mission_ids = {item.id for item in self.mission_templates}
        if self.presentation.default_mission_template_id is not None and self.presentation.default_mission_template_id not in mission_ids:
            raise ValueError(f"model {self.id!r} presentation references an unknown default mission template")
        unknown_default_channels = sorted(set(self.presentation.default_output_channel_ids) - set(channel_ids))
        if unknown_default_channels:
            raise ValueError(f"model {self.id!r} presentation references unknown default output channels {unknown_default_channels!r}")
        if self.output_schema.model_id != self.id or self.output_schema.model_version != self.version:
            raise ValueError(f"model {self.id!r} output schema identity does not match model metadata")
        if self.output_schema_id != self.output_schema.schema_id:
            raise ValueError(f"model {self.id!r} output schema ID does not match model metadata")
        if self.output_schema_fingerprint != self.output_schema.fingerprint:
            raise ValueError(f"model {self.id!r} output schema fingerprint does not match model metadata")
        if set(self.capabilities.operations) != set(self.operations):
            raise ValueError(f"model {self.id!r} capability and top-level operation declarations disagree")
        if self.capabilities.supports_dynamic_child_generation != self.output_schema.entity_output.supports_dynamic_spawning:
            raise ValueError(f"model {self.id!r} capability and output child-generation declarations disagree")
        return self
        ####

    ####


class TrajectoryProviderMetadata(BaseModel):
    """Common provider identity, independent from its model catalog."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-provider-metadata/v1"] = Field(
        default="taoryx.trajectory-provider-metadata/v1",
        alias="schema",
        serialization_alias="schema",
    )
    id: str = Field(min_length=1)
    name: str = Field(min_length=1)
    version: str = Field(min_length=1)
    description: str = Field(min_length=1)
    presentation: TrajectoryProviderPresentationMetadata
    status: str = Field(min_length=1)
    tags: tuple[str, ...] = ()
    configuration_contract: str = "taoryx.trajectory-provider-configuration-schema/v1"
    output_contract: str = "taoryx.trajectory-provider-output-schema/v1"
    run_request_contract: str = "taoryx.mission-composition-run-request/v1"
    session_contract: str = "taoryx.mission-composition-session/v1"
    trajectory_result_contract: str = "taoryx.mission-composition-trajectory/v1"
    failure_contract: str = "taoryx.mission-composition-failure/v1"
    execution_contract: str | None = None
    model_count: int = Field(ge=0)
    provenance: str = ""
    claim_boundary: str = Field(min_length=1)


class TrajectoryConfigurationSchema(BaseModel):
    """Versioned portable configuration grammar for one trajectory model."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-provider-configuration-schema/v1"] = Field(
        default="taoryx.trajectory-provider-configuration-schema/v1",
        alias="schema",
        serialization_alias="schema",
    )
    contract_version: Literal["1"] = "1"
    model_id: str = Field(min_length=1)
    model_version: str = Field(min_length=1)
    supported_fidelities: tuple[str, ...] = Field(min_length=1)
    root: ConfigurationNode
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_fidelity_references(self) -> TrajectoryConfigurationSchema:
        if len(self.supported_fidelities) != len(set(self.supported_fidelities)):
            raise ValueError(f"configuration schema {self.model_id!r} has duplicate supported fidelities")
        unknown = sorted(_node_fidelity_references(self.root) - set(self.supported_fidelities))
        if unknown:
            raise ValueError(f"configuration schema {self.model_id!r} references unsupported fidelities {unknown!r}")
        return self
        ####

    @property
    def fingerprint(self) -> str:
        """Return a stable identity for the complete advertised grammar."""

        payload = self.model_dump(mode="json", by_alias=True)
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()
        ####

    ####


class TrajectoryConfigurationInstance(BaseModel):
    """One caller-authored selection matching an advertised schema tree."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-provider-configuration/v1"] = Field(
        default="taoryx.trajectory-provider-configuration/v1",
        alias="schema",
        serialization_alias="schema",
    )
    configuration_id: str = Field(min_length=1)
    model_id: str = Field(min_length=1)
    model_version: str = Field(min_length=1)
    schema_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    fidelity: str = Field(min_length=1)
    realization_id: str | None = None
    mission_template_id: str | None = None
    root: ConfigurationNodeValue


class PreparedTrajectoryConfiguration(BaseModel):
    """Immutable, validated provider-independent configuration handoff."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-provider-prepared-configuration/v1"] = Field(
        default="taoryx.trajectory-provider-prepared-configuration/v1",
        alias="schema",
        serialization_alias="schema",
    )
    configuration: TrajectoryConfigurationInstance
    resolved: Any
    fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")

    @model_validator(mode="after")
    def validate_fingerprint(self) -> PreparedTrajectoryConfiguration:
        expected = _prepared_configuration_fingerprint(self.configuration, self.resolved)
        if self.fingerprint != expected:
            raise ValueError("prepared configuration fingerprint does not match its contents")
        return self
        ####

    ####


class ConfigurableTrajectoryProvider(Protocol):
    """Discovery/configuration surface independent from trajectory execution."""

    @property
    def metadata(self) -> TrajectoryProviderMetadata:
        """Return provider identity without embedding every model schema."""
        ...

    def list_models(self) -> tuple[TrajectoryModelMetadata, ...]:
        """Return common metadata without executing any model."""
        ...

    def get_model_schema(self, model_id: str) -> TrajectoryConfigurationSchema:
        """Return one complete portable configuration grammar."""
        ...

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        """Return one complete portable output-capability grammar."""
        ...

    def validate_configuration(self, configuration: TrajectoryConfigurationInstance) -> PreparedTrajectoryConfiguration:
        """Validate one typed configuration tree without running a model."""
        ...

    ####


def validate_configuration_instance(
    schema: TrajectoryConfigurationSchema,
    configuration: TrajectoryConfigurationInstance,
) -> PreparedTrajectoryConfiguration:
    """Validate one configuration against its exact advertised schema."""

    if configuration.model_id != schema.model_id:
        raise ConfigurationContractError("model-mismatch", f"expected model {schema.model_id!r}")
    if configuration.model_version != schema.model_version:
        raise ConfigurationContractError("model-version-mismatch", f"expected model version {schema.model_version!r}")
    if configuration.schema_fingerprint != schema.fingerprint:
        raise ConfigurationContractError("schema-fingerprint-mismatch", "configuration was authored against another schema")
    if configuration.fidelity not in schema.supported_fidelities:
        raise ConfigurationContractError(
            "unsupported-fidelity",
            f"expected one of {list(schema.supported_fidelities)!r}",
            path="configuration.fidelity",
        )
    resolved = _validate_node(
        schema.root,
        configuration.root,
        path="configuration.root",
        fidelity=configuration.fidelity,
    )
    return PreparedTrajectoryConfiguration(
        configuration=configuration,
        resolved=resolved,
        fingerprint=_prepared_configuration_fingerprint(configuration, resolved),
    )
    ####


def _prepared_configuration_fingerprint(configuration: TrajectoryConfigurationInstance, resolved: Any) -> str:
    identity = {
        "configuration": configuration.model_dump(mode="json", by_alias=True),
        "resolved": resolved,
    }
    encoded = json.dumps(identity, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
    ####


def render_configuration_schema(schema: TrajectoryConfigurationSchema) -> str:
    """Render a provider-independent plain-text view of a schema AST."""

    lines = [f"{schema.model_id} ({schema.model_version})", f"fidelities: {', '.join(schema.supported_fidelities)}"]
    _render_node(schema.root, lines, prefix="")
    return "\n".join(lines)
    ####


def _render_node(node: ConfigurationNode, lines: list[str], *, prefix: str) -> None:
    marker = {
        "parameter": "parameter",
        "group": "group",
        "choice": "choice",
        "sequence": "sequence",
        "optional": "optional",
    }[node.kind]
    lines.append(f"{prefix}{node.id}: {node.label} [{marker}]")
    child_prefix = prefix + "  "
    if isinstance(node, ConfigurationGroupSchema):
        for child in node.children:
            _render_node(child, lines, prefix=child_prefix)
    elif isinstance(node, ConfigurationChoiceSchema):
        for variant in node.variants:
            lines.append(f"{child_prefix}{variant.id}: {variant.label} [variant]")
            _render_node(variant.node, lines, prefix=child_prefix + "  ")
    elif isinstance(node, ConfigurationSequenceSchema):
        mode = "custom allowed" if node.allow_custom else "advertised templates only"
        lines.append(f"{child_prefix}sequence policy: {mode}")
        for template in node.templates:
            lines.append(f"{child_prefix}template {template.id}: {' -> '.join(template.item_variants)}")
        _render_node(node.item, lines, prefix=child_prefix)
    elif isinstance(node, ConfigurationOptionalSchema):
        _render_node(node.item, lines, prefix=child_prefix)
    ####


def _validate_node(
    schema: ConfigurationNode,
    value: ConfigurationNodeValue,
    *,
    path: str,
    fidelity: str,
) -> Any:
    if schema.kind != value.kind:
        raise ConfigurationContractError("node-kind-mismatch", f"expected {schema.kind!r}, received {value.kind!r}", path=path)
    if isinstance(schema, ConfigurationParameterSchema) and isinstance(value, ConfigurationParameterValue):
        if schema.compatible_fidelities and fidelity not in schema.compatible_fidelities:
            raise ConfigurationContractError(
                "incompatible-fidelity",
                f"parameter supports {list(schema.compatible_fidelities)!r}",
                path=path,
            )
        return _validate_parameter_value(schema, value.value, path=path, supplied_unit=value.unit)
    if isinstance(schema, ConfigurationGroupSchema) and isinstance(value, ConfigurationGroupValue):
        children = {item.id: item for item in schema.children}
        unknown = sorted(set(value.values) - set(children))
        if unknown:
            raise ConfigurationContractError("unknown-field", f"group does not advertise {unknown!r}", path=path)
        resolved: dict[str, Any] = {}
        for child_id, child_schema in children.items():
            child_value = value.values.get(child_id)
            if child_value is None:
                if isinstance(child_schema, ConfigurationParameterSchema):
                    if child_schema.default_declared:
                        resolved[child_id] = child_schema.default
                        continue
                    if not child_schema.required:
                        continue
                if isinstance(child_schema, ConfigurationOptionalSchema):
                    resolved[child_id] = None
                    continue
                raise ConfigurationContractError("missing-field", "required configuration node was not supplied", path=f"{path}.{child_id}")
            resolved[child_id] = _validate_node(
                child_schema,
                child_value,
                path=f"{path}.{child_id}",
                fidelity=fidelity,
            )
        return resolved
    if isinstance(schema, ConfigurationChoiceSchema) and isinstance(value, ConfigurationChoiceValue):
        variants = {item.id: item for item in schema.variants}
        selected = variants.get(value.selected)
        if selected is None:
            raise ConfigurationContractError("unknown-choice", f"expected one of {sorted(variants)!r}", path=path)
        if selected.compatible_fidelities and fidelity not in selected.compatible_fidelities:
            raise ConfigurationContractError(
                "incompatible-fidelity",
                f"variant {value.selected!r} supports {list(selected.compatible_fidelities)!r}",
                path=path,
            )
        resolved_choice = {
            "selected": value.selected,
            "value": _validate_node(
                selected.node,
                value.value,
                path=f"{path}.{value.selected}",
                fidelity=fidelity,
            ),
        }
        if value.instance_id is not None:
            resolved_choice["instance_id"] = value.instance_id
        return resolved_choice
    if isinstance(schema, ConfigurationSequenceSchema) and isinstance(value, ConfigurationSequenceValue):
        count = len(value.items)
        if count < schema.minimum_items:
            raise ConfigurationContractError("sequence-too-short", f"minimum_items is {schema.minimum_items}", path=path)
        if schema.maximum_items is not None and count > schema.maximum_items:
            raise ConfigurationContractError("sequence-too-long", f"maximum_items is {schema.maximum_items}", path=path)
        if schema.templates:
            if not all(isinstance(item, ConfigurationChoiceValue) for item in value.items):
                raise ConfigurationContractError(
                    "invalid-sequence-item",
                    "templated sequences require choice values",
                    path=path,
                )
            selected_variants = tuple(item.selected for item in value.items if isinstance(item, ConfigurationChoiceValue))
            matching_templates = tuple(
                template
                for template in schema.templates
                if template.item_variants == selected_variants and (not template.compatible_fidelities or fidelity in template.compatible_fidelities)
            )
            if not schema.allow_custom and not matching_templates:
                raise ConfigurationContractError(
                    "unsupported-sequence",
                    "selected segment order is not an advertised template for this fidelity",
                    path=path,
                )
        return [_validate_node(schema.item, item, path=f"{path}[{index}]", fidelity=fidelity) for index, item in enumerate(value.items)]
    if isinstance(schema, ConfigurationOptionalSchema) and isinstance(value, ConfigurationOptionalValue):
        if not value.enabled:
            return None
        if value.value is None:
            raise ConfigurationContractError("missing-optional-value", "enabled optional node has no value", path=path)
        return _validate_node(schema.item, value.value, path=f"{path}.value", fidelity=fidelity)
    raise ConfigurationContractError("node-type-mismatch", "schema and value nodes are incompatible", path=path)
    ####


def _validate_parameter_value(
    schema: ConfigurationParameterSchema,
    value: Any,
    *,
    path: str,
    supplied_unit: str | None,
) -> Any:
    if supplied_unit != schema.canonical_unit:
        raise ConfigurationContractError(
            "unit-mismatch",
            f"expected {schema.canonical_unit!r}, received {supplied_unit!r}",
            path=path,
        )
    if schema.value_type == "number":
        if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
            raise ConfigurationContractError("invalid-value", "expected a finite number", path=path)
    elif schema.value_type == "integer":
        if isinstance(value, bool) or not isinstance(value, int):
            raise ConfigurationContractError("invalid-value", "expected an integer", path=path)
    elif schema.value_type == "boolean":
        if not isinstance(value, bool):
            raise ConfigurationContractError("invalid-value", "expected a boolean", path=path)
    elif schema.value_type in {"string", "enum"}:
        if not isinstance(value, str):
            raise ConfigurationContractError("invalid-value", "expected a string", path=path)
        if schema.value_type == "enum" and value not in schema.choices:
            raise ConfigurationContractError("invalid-choice", f"expected one of {list(schema.choices)!r}", path=path)
    elif schema.value_type in {"vector3", "vector4"}:
        expected_length = 3 if schema.value_type == "vector3" else 4
        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != expected_length:
            raise ConfigurationContractError("invalid-value", f"expected a numeric vector of length {expected_length}", path=path)
        if any(isinstance(item, bool) or not isinstance(item, int | float) or not math.isfinite(float(item)) for item in value):
            raise ConfigurationContractError("invalid-value", f"expected a finite numeric vector of length {expected_length}", path=path)
    if isinstance(value, int | float) and not isinstance(value, bool):
        numeric = float(value)
        _validate_interval(schema.interval, numeric, path=path)
        if schema.periodicity is not None:
            lower = schema.periodicity.canonical_minimum
            upper = lower + schema.periodicity.period
            if numeric < lower or numeric >= upper:
                raise ConfigurationContractError(
                    "noncanonical-periodic-value",
                    f"expected canonical interval [{lower}, {upper})",
                    path=path,
                )
    return value
    ####


def _validate_interval(interval: ConfigurationInterval | None, value: float, *, path: str) -> None:
    if interval is None:
        return
    if interval.minimum is not None and interval.minimum.value is not None:
        lower = interval.minimum.value
        if value < lower or (value == lower and not interval.minimum.inclusive):
            operator = ">=" if interval.minimum.inclusive else ">"
            raise ConfigurationContractError("out-of-bounds", f"expected value {operator} {lower}", path=path)
    if interval.maximum is not None and interval.maximum.value is not None:
        upper = interval.maximum.value
        if value > upper or (value == upper and not interval.maximum.inclusive):
            operator = "<=" if interval.maximum.inclusive else "<"
            raise ConfigurationContractError("out-of-bounds", f"expected value {operator} {upper}", path=path)
    ####


def _require_unique_node_ids(nodes: Sequence[ConfigurationNode], scope: str) -> None:
    ids = tuple(item.id for item in nodes)
    if len(ids) != len(set(ids)):
        raise ValueError(f"{scope} has duplicate node IDs")
    ####


def _node_fidelity_references(node: ConfigurationNode) -> set[str]:
    references: set[str] = set()
    if isinstance(node, ConfigurationParameterSchema):
        references.update(node.compatible_fidelities)
    elif isinstance(node, ConfigurationGroupSchema):
        for child in node.children:
            references.update(_node_fidelity_references(child))
    elif isinstance(node, ConfigurationChoiceSchema):
        for variant in node.variants:
            references.update(variant.compatible_fidelities)
            references.update(_node_fidelity_references(variant.node))
    elif isinstance(node, ConfigurationSequenceSchema):
        for template in node.templates:
            references.update(template.compatible_fidelities)
        references.update(_node_fidelity_references(node.item))
    elif isinstance(node, ConfigurationOptionalSchema):
        references.update(_node_fidelity_references(node.item))
    return references
    ####


for _schema_model in (
    ConfigurationGroupSchema,
    ConfigurationChoiceVariant,
    ConfigurationChoiceSchema,
    ConfigurationSequenceSchema,
    ConfigurationOptionalSchema,
):
    _schema_model.model_rebuild(_types_namespace={"ConfigurationNode": ConfigurationNode})

for _value_model in (
    ConfigurationGroupValue,
    ConfigurationChoiceValue,
    ConfigurationSequenceValue,
    ConfigurationOptionalValue,
):
    _value_model.model_rebuild(_types_namespace={"ConfigurationNodeValue": ConfigurationNodeValue})


__all__ = [
    "ConfigurableTrajectoryProvider",
    "ConfigurationBound",
    "ConfigurationChoiceSchema",
    "ConfigurationChoiceValue",
    "ConfigurationChoiceVariant",
    "ConfigurationContractError",
    "ConfigurationGroupSchema",
    "ConfigurationGroupValue",
    "ConfigurationInterval",
    "ConfigurationNode",
    "ConfigurationNodeValue",
    "ConfigurationOptionalSchema",
    "ConfigurationOptionalValue",
    "ConfigurationParameterSchema",
    "ConfigurationParameterValue",
    "ConfigurationPeriodicity",
    "ConfigurationSequenceSchema",
    "ConfigurationSequenceTemplate",
    "ConfigurationSequenceValue",
    "ConfigurationValueSpace",
    "NumericPresentationMetadata",
    "PreparedTrajectoryConfiguration",
    "PresentationLinkMetadata",
    "TrajectoryConfigurationInstance",
    "TrajectoryConfigurationSchema",
    "TrajectoryDeploymentMetadata",
    "TrajectoryActuatorType",
    "TrajectoryDynamicsFidelity",
    "TrajectoryEntityOutputMetadata",
    "TrajectoryFidelityMetadata",
    "TrajectoryFidelityTransition",
    "TrajectoryMissionOperationMetadata",
    "TrajectoryMissionTemplateMetadata",
    "TrajectoryModelCapabilities",
    "TrajectoryModelMetadata",
    "TrajectoryModelPresentationMetadata",
    "TrajectoryModelPropertyMetadata",
    "TrajectoryOutputChannelMetadata",
    "TrajectoryOutputDataType",
    "TrajectoryOutputSchema",
    "TrajectoryProviderMetadata",
    "TrajectoryProviderPresentationMetadata",
    "TrajectoryRealizationMetadata",
    "TrajectoryReferenceFrameMetadata",
    "TrajectoryInputRealization",
    "TrajectorySamplingSemantics",
    "TrajectoryTelemetryGroupMetadata",
    "ValuePresentationMetadata",
    "render_configuration_schema",
    "validate_configuration_instance",
]
