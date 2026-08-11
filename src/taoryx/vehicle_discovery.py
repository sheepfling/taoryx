"""Shared discovery taxonomy for vehicle families and composition inputs.

The physics registries remain authoritative for numeric model data.  This
module owns only stable, provider-neutral vocabulary used to explain those
models to authors, UIs, and agents.  A family may extend the vocabulary with
an inline typed segment taxonomy, but the common catalogue keeps ordinary
vehicle and mission concepts comparable across plug-ins.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

VehicleClass = Literal[
    "powered_fixed_wing_aircraft",
    "rocket_aircraft",
    "multirotor",
    "lifting_body",
    "staged_launch_vehicle",
    "passive_atmospheric_body",
]
OperatingDomain = Literal["atmospheric", "near_space", "space"]
PropulsionKind = Literal[
    "propeller",
    "turbofan",
    "scheduled_thrust_model",
    "jet",
    "rocket",
    "electric_rotor",
    "unpowered",
    "staged_rocket",
    "none",
]
ModelBasis = Literal[
    "public_research_surrogate",
    "source_grounded_reference",
    "source_derived_composite",
    "analytical_geometry_model",
]

SegmentCategory = Literal[
    "local_control_screen",
    "hold",
    "powered_flight",
    "takeoff",
    "translation",
    "turn",
    "energy_management",
    "coast",
    "release",
    "staging",
    "descent",
    "landing",
    "terminal",
]
SegmentExecutionStyle = Literal[
    "closed_loop",
    "open_loop",
    "scheduled_replay",
    "passive",
    "evaluation_only",
]
SegmentLifecyclePhase = Literal[
    "local_validation",
    "departure",
    "enroute",
    "transition",
    "arrival",
    "terminal",
]

ParameterSemanticRole = Literal[
    "position",
    "geodetic_position",
    "altitude",
    "speed",
    "heading",
    "attitude",
    "angular_rate",
    "mass",
    "time",
    "event_time",
    "flight_path",
    "control_command",
    "control_limit",
    "environment_limit",
    "energy",
    "geometry",
    "terminal_condition",
    "mode_selection",
    "custom",
]


class VehicleMetadata(BaseModel):
    """Human-facing identity and taxonomy that do not duplicate plant data."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    summary: str = Field(min_length=1)
    vehicle_class: VehicleClass
    operating_domains: tuple[OperatingDomain, ...] = Field(min_length=1)
    propulsion: tuple[PropulsionKind, ...] = Field(min_length=1)
    model_basis: ModelBasis
    roles: tuple[str, ...] = Field(min_length=1)
    tags: tuple[str, ...] = ()
    claim_boundary: str = Field(min_length=1)


class FixedReferenceGeometry(BaseModel):
    """One source-owned fixed reference geometry record."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["fixed_reference_geometry"] = "fixed_reference_geometry"
    area_m2: float | None = Field(default=None, gt=0.0)
    reference_length_m: float | None = Field(default=None, gt=0.0)
    mean_aerodynamic_chord_m: float | None = Field(default=None, gt=0.0)
    span_m: float | None = Field(default=None, gt=0.0)
    provenance: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)


class VariantGeometry(BaseModel):
    """Geometry selected at composition time rather than fixed for a family."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["variant_geometry"] = "variant_geometry"
    selector_parameter_id: str = Field(min_length=1)
    selectable_variant_ids: tuple[str, ...] = Field(min_length=1)
    provenance: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)


class NotRepresentedGeometry(BaseModel):
    """An explicit declaration that common geometry data has no owner."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["not_represented"] = "not_represented"
    reason: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)


GeometryCharacteristic = Annotated[
    FixedReferenceGeometry | VariantGeometry | NotRepresentedGeometry,
    Field(discriminator="kind"),
]


class PrincipalInertia(BaseModel):
    """Principal body-axis inertia values in a declared x/y/z order."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    x_kg_m2: float = Field(gt=0.0)
    y_kg_m2: float = Field(gt=0.0)
    z_kg_m2: float = Field(gt=0.0)
    axis_order: Literal["x_y_z"] = "x_y_z"


class FixedMassProperties(BaseModel):
    """One fixed-mass source binding with optional nominal operating mass."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["fixed_mass_properties"] = "fixed_mass_properties"
    dry_mass_kg: float = Field(gt=0.0)
    nominal_mass_kg: float | None = Field(default=None, gt=0.0)
    inertia: PrincipalInertia
    binding_status: str | None = None
    perturbation_policy: str | None = None
    provenance: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)


class ScheduledMassProperties(BaseModel):
    """A variable-mass binding governed by a named source schedule."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["scheduled_mass_properties"] = "scheduled_mass_properties"
    schedule_id: str = Field(min_length=1)
    schedule_evidence: str = Field(min_length=1)
    includes_staging_events: bool
    provenance: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)


class NotRepresentedMassProperties(BaseModel):
    """An explicit declaration that common mass data has no owner."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["not_represented"] = "not_represented"
    reason: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)


MassCharacteristic = Annotated[
    FixedMassProperties | ScheduledMassProperties | NotRepresentedMassProperties,
    Field(discriminator="kind"),
]


class EnvelopeAxis(BaseModel):
    """One bounded model-validity axis, never a performance assertion."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    canonical_unit: str | None = None
    lower: float | None = None
    upper: float | None = None
    description: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_bound(self) -> EnvelopeAxis:
        if self.lower is None and self.upper is None:
            raise ValueError(f"envelope axis {self.id!r} requires a lower or upper bound")
        if self.lower is not None and self.upper is not None and self.lower > self.upper:
            raise ValueError(f"envelope axis {self.id!r} has inverted bounds")
        return self
        ####


class DeclaredValidityEnvelope(BaseModel):
    """A set of source-owned model-validity axes."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["declared_validity_envelope"] = "declared_validity_envelope"
    axes: tuple[EnvelopeAxis, ...] = Field(min_length=1)
    notes: tuple[str, ...] = ()
    provenance: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_axis_ids(self) -> DeclaredValidityEnvelope:
        identifiers = [axis.id for axis in self.axes]
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("validity envelope axis IDs must be unique")
        return self
        ####


class NotRepresentedValidityEnvelope(BaseModel):
    """An explicit declaration that a common validity envelope is absent."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["not_represented"] = "not_represented"
    reason: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)


ValidityEnvelopeCharacteristic = Annotated[
    DeclaredValidityEnvelope | NotRepresentedValidityEnvelope,
    Field(discriminator="kind"),
]


class DeclaredEffector(BaseModel):
    """One source or registry declared command/effectuator coordinate."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    canonical_unit: str | None = None
    lower: float | None = None
    upper: float | None = None
    default: float | None = None
    provenance: str = Field(min_length=1)
    description: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_bounds(self) -> DeclaredEffector:
        if self.lower is not None and self.upper is not None and self.lower > self.upper:
            raise ValueError(f"effector {self.id!r} has inverted bounds")
        return self
        ####


class DeclaredEffectors(BaseModel):
    """A source-owned set of potentially controllable effectors."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["declared_effectors"] = "declared_effectors"
    effectors: tuple[DeclaredEffector, ...] = Field(min_length=1)
    provenance: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_effector_ids(self) -> DeclaredEffectors:
        identifiers = [effector.id for effector in self.effectors]
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("effector IDs must be unique")
        return self
        ####


class NotApplicableEffectors(BaseModel):
    """A model that deliberately has no externally controllable effectors."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["not_applicable"] = "not_applicable"
    reason: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)


class NotRepresentedEffectors(BaseModel):
    """A model whose common source does not own an effector contract."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["not_represented"] = "not_represented"
    reason: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)


EffectorCharacteristic = Annotated[
    DeclaredEffectors | NotApplicableEffectors | NotRepresentedEffectors,
    Field(discriminator="kind"),
]


class VehiclePhysicalCharacteristics(BaseModel):
    """Complete typed physical-characteristics card for one vehicle family."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    reference_geometry: GeometryCharacteristic
    mass_properties: MassCharacteristic
    validity_envelope: ValidityEnvelopeCharacteristic
    effectors: EffectorCharacteristic
    claim_boundary: str = Field(min_length=1)

    def availability_by_field(self) -> dict[str, Literal["available", "not_applicable", "not_represented"]]:
        """Return a small typed coverage report for UI and validation clients."""

        def availability(characteristic: BaseModel) -> Literal["available", "not_applicable", "not_represented"]:
            kind = getattr(characteristic, "kind")
            if kind == "not_represented":
                return "not_represented"
            if kind == "not_applicable":
                return "not_applicable"
            return "available"

        return {
            "reference_geometry": availability(self.reference_geometry),
            "mass_properties": availability(self.mass_properties),
            "validity_envelope": availability(self.validity_envelope),
            "effectors": availability(self.effectors),
        }
        ####

    def public_dict(self) -> dict[str, object]:
        """Serialize only at the discovery/API boundary."""

        coverage = self.availability_by_field()
        available_fields = [identifier for identifier, state in coverage.items() if state != "not_represented"]
        unavailable_fields = [identifier for identifier, state in coverage.items() if state == "not_represented"]
        return {
            **self.model_dump(mode="json"),
            "status": "source_owned_values_available" if available_fields else "not_represented_in_common_catalog",
            "available_fields": available_fields,
            "unavailable_fields": unavailable_fields,
            "availability_by_field": coverage,
        }
        ####


class PointMassTierProfile(BaseModel):
    """Typed profile card for the reduced translational tier."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["point_mass_profile"] = "point_mass_profile"
    profile_id: str = Field(min_length=1)
    control_realization: Literal["force_model"] = "force_model"


class AxisResponseLimit(BaseModel):
    """One declared pseudo-6DOF attitude-response axis."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    axis: Literal["roll", "pitch", "yaw"]
    time_constant_s: float = Field(gt=0.0)
    damping_ratio: float = Field(gt=0.0, le=2.0)
    maximum_rate_rad_s: float = Field(gt=0.0)
    maximum_acceleration_rad_s2: float = Field(gt=0.0)


class PhaseResponseLimits(BaseModel):
    """A named response schedule phase with all attitude axes present."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    phase: str = Field(min_length=1)
    axes: tuple[AxisResponseLimit, ...] = Field(min_length=3, max_length=3)

    @model_validator(mode="after")
    def validate_axes(self) -> PhaseResponseLimits:
        if {axis.axis for axis in self.axes} != {"roll", "pitch", "yaw"}:
            raise ValueError("a phase response must declare roll, pitch, and yaw exactly once")
        return self
        ####


class PseudoSixDofTierProfile(BaseModel):
    """Typed profile card for a response-law or rigid-body-reuse pseudo tier."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["pseudo_6dof_profile"] = "pseudo_6dof_profile"
    profile_id: str = Field(min_length=1)
    model_kind: str = Field(min_length=1)
    control_realization: Literal["response_law", "surface_allocated", "uncontrolled"]
    evidence_grade: str = Field(min_length=1)
    default_response: tuple[AxisResponseLimit, ...] = ()
    phase_response: tuple[PhaseResponseLimits, ...] = ()
    unsupported_claims: tuple[str, ...] = ()

    @model_validator(mode="after")
    def validate_response(self) -> PseudoSixDofTierProfile:
        if self.model_kind == "rigid_body_reuse":
            if self.default_response or self.phase_response:
                raise ValueError("rigid_body_reuse pseudo profile cannot advertise surrogate response limits")
            return self
        if {axis.axis for axis in self.default_response} != {"roll", "pitch", "yaw"}:
            raise ValueError("pseudo profile must declare roll, pitch, and yaw response limits")
        return self
        ####


class DirectWrenchTierProfile(BaseModel):
    """Typed generalized-force/moment profile card."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["direct_wrench_profile"] = "direct_wrench_profile"
    profile_id: str | None = None
    force_axes: tuple[str, ...] = ()
    moment_axes: tuple[str, ...] = ()
    unsupported_claims: tuple[str, ...] = ()


class SurfaceAllocatedTierProfile(BaseModel):
    """Typed named-effector allocation profile card."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["surface_allocated_profile"] = "surface_allocated_profile"
    profile_id: str | None = None
    allocator_id: str | None = None
    declared_effector_channels: tuple[str, ...] = ()
    unsupported_claims: tuple[str, ...] = ()


TierModelProfile = Annotated[
    PointMassTierProfile | PseudoSixDofTierProfile | DirectWrenchTierProfile | SurfaceAllocatedTierProfile,
    Field(discriminator="kind"),
]


class SegmentTaxonomy(BaseModel):
    """Comparable semantic classification for one composable segment."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    category: SegmentCategory
    execution_style: SegmentExecutionStyle
    lifecycle_phase: SegmentLifecyclePhase
    description: str = Field(min_length=1)
    additional_capabilities: tuple[SegmentCategory, ...] = ()


def _segment(
    category: SegmentCategory,
    execution_style: SegmentExecutionStyle,
    lifecycle_phase: SegmentLifecyclePhase,
    description: str,
    additional_capabilities: tuple[SegmentCategory, ...] = (),
) -> SegmentTaxonomy:
    return SegmentTaxonomy(
        category=category,
        execution_style=execution_style,
        lifecycle_phase=lifecycle_phase,
        description=description,
        additional_capabilities=additional_capabilities,
    )
    ####


_LOCAL_CLOSED_LOOP = {
    "condition3_source_table_physical_lqi_screen",
    "condition3_source_table_physical_lqr_screen",
    "individual_rotor_horizontal_translation_lqi_screen",
    "individual_rotor_hover_lqi_screen",
    "individual_rotor_vertical_translation_lqi_screen",
    "local_physical_control_screen",
    "local_physical_surface_lqi_schedule_interior_screen",
    "local_physical_surface_lqi_screen",
    "local_physical_surface_lqr_schedule_interior_screen",
    "local_physical_surface_lqr_schedule_transition_screen",
    "local_wrench_lqi_recovery_screen",
    "local_wrench_recovery_screen",
    "native_coordinate_attitude_lqi_screen",
    "source_surface_attitude_rate_lqi_recovery_screen",
    "source_table_physical_lqi_long_recovery_screen",
    "source_table_physical_lqi_screen",
    "source_table_physical_lqr_screen",
}
_LOCAL_EVALUATION = {
    "source_surface_pitch_authority_allocation_screen",
    "source_surface_three_axis_authority_allocation_screen",
}

COMMON_SEGMENT_TAXONOMY: Mapping[str, SegmentTaxonomy] = {
    **{
        identifier: _segment(
            "local_control_screen",
            "closed_loop",
            "local_validation",
            "Bounded local feedback-control evaluation; not a navigation mission segment.",
        )
        for identifier in _LOCAL_CLOSED_LOOP
    },
    **{
        identifier: _segment(
            "local_control_screen",
            "evaluation_only",
            "local_validation",
            "Bounded control-authority or allocation evaluation without mission propagation.",
        )
        for identifier in _LOCAL_EVALUATION
    },
    "trim_hold": _segment("hold", "closed_loop", "departure", "Maintain a declared trim or operating condition."),
    "hover_dwell": _segment("hold", "closed_loop", "enroute", "Maintain position, altitude, and heading for a bounded dwell."),
    "trim_capture": _segment("hold", "closed_loop", "departure", "Capture a declared post-release trim condition."),
    "ignition_ascent": _segment("powered_flight", "scheduled_replay", "departure", "Execute a bounded powered-ascent phase."),
    "powered_climb": _segment("powered_flight", "closed_loop", "departure", "Execute a controlled powered climb to a declared cutoff or gate."),
    "booster_powered": _segment("powered_flight", "open_loop", "departure", "Replay a declared attached-booster powered interval."),
    "rotor_spool_takeoff": _segment("takeoff", "closed_loop", "departure", "Spool the rotor system and climb from the ground to a hover gate."),
    "waypoint_translation": _segment("translation", "closed_loop", "enroute", "Translate to a spatial waypoint under bounded speed and attitude intent."),
    "fly_by_turn": _segment("turn", "closed_loop", "enroute", "Follow a bounded horizontal turn toward a fly-by gate."),
    "glide_bank_reversal": _segment("turn", "closed_loop", "enroute", "Reverse bank to manage crossrange during glide."),
    "source_scheduled_bank": _segment("turn", "scheduled_replay", "enroute", "Replay one source-declared fixed bank interval."),
    "yaw_scan": _segment("turn", "closed_loop", "enroute", "Rotate heading through a bounded yaw scan."),
    "climb_level_gate": _segment("energy_management", "closed_loop", "enroute", "Manage speed and altitude through a climb-to-level gate."),
    "energy_climb": _segment("energy_management", "closed_loop", "enroute", "Increase specific energy within declared load and envelope limits."),
    "energy_handoff": _segment("energy_management", "closed_loop", "arrival", "Reach a declared terminal energy and geometry corridor."),
    "energy_recovery_gate": _segment("energy_management", "closed_loop", "arrival", "Recover to a declared energy state before arrival."),
    "atmospheric_handoff": _segment("energy_management", "closed_loop", "arrival", "Manage atmospheric energy to a named handoff corridor."),
    "unpowered_glide_handoff": _segment("energy_management", "open_loop", "arrival", "Propagate an unpowered glide to a declared handoff witness."),
    "coast_apogee": _segment("coast", "passive", "enroute", "Coast through apogee within a declared energy corridor."),
    "coast_terminal_state": _segment(
        "coast",
        "passive",
        "arrival",
        "Coast toward a declared terminal state.",
        ("terminal",),
    ),
    "passive_coast": _segment("coast", "passive", "enroute", "Propagate a passive ballistic or aerodynamic coast."),
    "booster_coast_release": _segment("release", "open_loop", "transition", "Coast to and execute a declared booster release event."),
    "separation": _segment("release", "closed_loop", "transition", "Execute carrier separation and clearance."),
    "source_booster_release": _segment("release", "scheduled_replay", "transition", "Replay a source-declared booster release event."),
    "stage_separation": _segment("staging", "scheduled_replay", "transition", "Execute a declared cutoff, separation, and ignition chain."),
    "atmospheric_descent": _segment(
        "descent",
        "passive",
        "arrival",
        "Propagate atmospheric descent toward a terminal condition.",
        ("terminal",),
    ),
    "descent_level_gate": _segment("descent", "closed_loop", "arrival", "Manage descent to a bounded level-flight gate."),
    "touchdown_settle_disarm": _segment("landing", "closed_loop", "terminal", "Descend, settle, and disarm at a declared pad."),
    "impact_witness": _segment("terminal", "passive", "terminal", "Observe a passive terminal impact event."),
    "source_ground_contact": _segment("terminal", "scheduled_replay", "terminal", "Replay to the source-declared ground-contact event."),
    "terminal_state_gate": _segment("terminal", "closed_loop", "terminal", "Evaluate arrival against a declared terminal state corridor."),
}


VEHICLE_CLASS_SEGMENT_EXPECTATIONS: Mapping[VehicleClass, tuple[SegmentCategory, ...]] = {
    "powered_fixed_wing_aircraft": ("hold", "energy_management", "turn", "descent", "terminal"),
    "rocket_aircraft": ("release", "powered_flight", "coast", "energy_management", "terminal"),
    "multirotor": ("takeoff", "hold", "translation", "turn", "landing"),
    "lifting_body": ("release", "hold", "turn", "energy_management", "terminal"),
    "staged_launch_vehicle": ("powered_flight", "staging", "coast", "terminal"),
    "passive_atmospheric_body": ("coast", "descent", "terminal"),
}


_PARAMETER_ROLE_IDS: Mapping[ParameterSemanticRole, frozenset[str]] = {
    "position": frozenset({"north_m", "east_m", "pad_north_m", "pad_east_m", "gate_center_ned_m", "target_ned_m", "pad_ned_m"}),
    "geodetic_position": frozenset({"latitude_deg", "longitude_deg"}),
    "altitude": frozenset({"altitude_m", "initial_altitude_m", "target_altitude_m", "impact_plane_altitude_m"}),
    "speed": frozenset({"speed_m_s", "initial_speed_m_s", "target_speed_m_s", "maximum_speed_m_s", "climb_rate_m_s", "descent_rate_m_s", "maximum_sink_rate_m_s"}),
    "heading": frozenset({"heading_deg", "initial_heading_deg", "target_heading_deg", "launch_azimuth_deg"}),
    "attitude": frozenset({"quaternion_wxyz"}),
    "angular_rate": frozenset({"body_rates_rad_s", "yaw_rate_limit_deg_s"}),
    "mass": frozenset({"mass_kg", "initial_mass_kg", "operating_mass_kg", "grounded_operating_mass_kg"}),
    "time": frozenset({"duration_s", "dwell_s", "horizon_s", "settle_s", "clearance_time_s"}),
    "event_time": frozenset({"cutoff_time_s", "release_time_s", "separation_delay_s"}),
    "flight_path": frozenset({"launch_elevation_deg", "pitch_program"}),
    "control_command": frozenset({"bank_command_deg", "glide_bank_deg"}),
    "control_limit": frozenset({"bank_limit_deg", "load_factor_limit"}),
    "environment_limit": frozenset({"max_dynamic_pressure_pa", "corridor_m"}),
    "energy": frozenset({"mach", "target_mach", "target_energy"}),
    "geometry": frozenset({"turn_radius_m", "target_crossrange_m"}),
    "terminal_condition": frozenset({"terminal_kind"}),
    "mode_selection": frozenset({"area_policy", "body_shape", "cutoff_condition", "turn_direction"}),
}


def parameter_semantic_role(identifier: str) -> ParameterSemanticRole:
    """Resolve a common composition parameter into a stable semantic role."""

    for role, identifiers in _PARAMETER_ROLE_IDS.items():
        if identifier in identifiers:
            return role
    return "custom"
    ####


def resolve_segment_taxonomy(identifier: str, declared: SegmentTaxonomy | None = None) -> SegmentTaxonomy:
    """Resolve an inline taxonomy or one shared common-segment definition."""

    if declared is not None:
        return declared
    try:
        return COMMON_SEGMENT_TAXONOMY[identifier]
    except KeyError as error:
        raise ValueError(
            f"segment {identifier!r} has no common taxonomy; declare an inline taxonomy before advertising it"
        ) from error
    ####


__all__ = [
    "AxisResponseLimit",
    "COMMON_SEGMENT_TAXONOMY",
    "DeclaredEffector",
    "DeclaredEffectors",
    "DeclaredValidityEnvelope",
    "DirectWrenchTierProfile",
    "EffectorCharacteristic",
    "EnvelopeAxis",
    "FixedMassProperties",
    "FixedReferenceGeometry",
    "GeometryCharacteristic",
    "MassCharacteristic",
    "ModelBasis",
    "NotApplicableEffectors",
    "NotRepresentedEffectors",
    "NotRepresentedGeometry",
    "NotRepresentedMassProperties",
    "NotRepresentedValidityEnvelope",
    "OperatingDomain",
    "ParameterSemanticRole",
    "PhaseResponseLimits",
    "PointMassTierProfile",
    "PrincipalInertia",
    "PropulsionKind",
    "PseudoSixDofTierProfile",
    "ScheduledMassProperties",
    "SegmentCategory",
    "SegmentExecutionStyle",
    "SegmentLifecyclePhase",
    "SegmentTaxonomy",
    "SurfaceAllocatedTierProfile",
    "TierModelProfile",
    "VEHICLE_CLASS_SEGMENT_EXPECTATIONS",
    "ValidityEnvelopeCharacteristic",
    "VehicleClass",
    "VehicleMetadata",
    "VehiclePhysicalCharacteristics",
    "VariantGeometry",
    "parameter_semantic_role",
    "resolve_segment_taxonomy",
]
