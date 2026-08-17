"""Small, evidence-aware authoring surface for parametric interceptors."""

from __future__ import annotations

import hashlib
import json
import math
import re
from collections.abc import Mapping, Sequence
from enum import StrEnum
from pathlib import Path
from typing import Any, Literal, Self

import yaml
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from .aerodynamics import DragCoefficientSchedule
from .control_authority import CONTROL_ALLOCATION_POLICIES
from .thrust_curve import AbsoluteThrustCurve, DualPulseThrustProgram
from .thrust_schedule import ThrustProfileSchedule

EvidenceScalar = float | int | str | bool | tuple[str, ...]


class ValueOrigin(StrEnum):
    """Why one profile value exists."""

    OBSERVED = "observed"
    REPORTED = "reported"
    DERIVED = "derived"
    INFERRED = "inferred"
    ARCHETYPE_ASSUMPTION = "archetype_assumption"
    CALIBRATED = "calibrated"
    SIMULATION_ASSUMPTION = "simulation_assumption"


####


class EvidenceConfidence(StrEnum):
    """Coarse confidence carried from research into simulation."""

    UNKNOWN = "unknown"
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"


####


class AssumptionCase(StrEnum):
    """Named, reproducible surrogate uncertainty cases."""

    CONSERVATIVE = "conservative"
    NOMINAL = "nominal"
    OPTIMISTIC = "optimistic"


####


class ResolutionStatus(StrEnum):
    """How strongly a runnable surrogate depends on archetype assumptions."""

    UNASSESSED = "unassessed"
    SOURCE_DOMINANT = "source_dominant"
    MIXED = "mixed"
    ARCHETYPE_DOMINANT = "archetype_dominant"
    EXACT_VARIANT_REQUIRED = "exact_variant_required"


####


class EvidenceSourceValue(BaseModel):
    """Original source value retained when a canonical value was converted."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    value: float | int | str
    unit: str | None = None

    ####


class EvidenceGap(BaseModel):
    """A deliberately unavailable value that must not be filled as evidence."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    parameter_id: str = Field(pattern=r"^[a-z0-9]+(?:[._-][a-z0-9]+)*$")
    status: str = Field(pattern=r"^[a-z0-9]+(?:[._-][a-z0-9]+)*$")
    unit: str | None = None
    source_record_ids: tuple[str, ...] = ()
    note: str = ""

    ####


class EvidenceInterval(BaseModel):
    """Variant-level numeric range retained without selecting a false point value."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    parameter_id: str = Field(pattern=r"^[a-z0-9]+(?:[._-][a-z0-9]+)*$")
    minimum_value: float
    maximum_value: float
    unit: str
    origin: str = "observed_variant_interval"
    source_record_ids: tuple[str, ...] = ()
    source_minimum_value: float | None = None
    source_maximum_value: float | None = None
    source_unit: str | None = None
    note: str = ""

    @model_validator(mode="after")
    def validate_interval(self) -> Self:
        numeric = [self.minimum_value, self.maximum_value]
        if self.source_minimum_value is not None:
            numeric.append(self.source_minimum_value)
        if self.source_maximum_value is not None:
            numeric.append(self.source_maximum_value)
        if any(not math.isfinite(value) for value in numeric):
            raise ValueError("evidence interval bounds must be finite")
        if self.minimum_value > self.maximum_value:
            raise ValueError("evidence interval minimum must not exceed maximum")
        source_fields = (
            self.source_minimum_value,
            self.source_maximum_value,
            self.source_unit,
        )
        if any(item is not None for item in source_fields) and any(item is None for item in source_fields):
            raise ValueError("evidence interval source bounds and unit must be supplied together")
        if self.source_minimum_value is not None and self.source_maximum_value is not None and self.source_minimum_value > self.source_maximum_value:
            raise ValueError("evidence interval source minimum must not exceed source maximum")
        return self
        ####

    ####


class EvidenceValue(BaseModel):
    """One value plus its factual or modeling status.

    A raw value passed to :func:`interceptor` becomes a simulation assumption.
    Use :func:`observed`, :func:`reported`, or :func:`assumed` when the origin
    should be explicit in authoring code.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    value: EvidenceScalar
    origin: ValueOrigin
    unit: str | None = None
    source_record_ids: tuple[str, ...] = ()
    confidence: EvidenceConfidence = EvidenceConfidence.UNKNOWN
    method: str | None = None
    source_value: EvidenceSourceValue | None = None

    @model_validator(mode="after")
    def validate_evidence(self) -> Self:
        if isinstance(self.value, float) and not math.isfinite(self.value):
            raise ValueError("evidence values must be finite")
        if self.origin in {ValueOrigin.OBSERVED, ValueOrigin.REPORTED} and not self.source_record_ids:
            raise ValueError(f"{self.origin.value} evidence requires at least one source_record_id")
        if self.origin is ValueOrigin.DERIVED and not self.method:
            raise ValueError("derived evidence requires a method")
        return self
        ####

    ####


def observed(
    value: EvidenceScalar,
    *,
    source_record_id: str | None = None,
    source_record_ids: Sequence[str] = (),
    unit: str | None = None,
    confidence: EvidenceConfidence | str = EvidenceConfidence.HIGH,
    method: str | None = None,
    source_value: EvidenceSourceValue | Mapping[str, object] | None = None,
) -> EvidenceValue:
    """Mark a directly observed or cataloged value."""

    normalized_source_value = EvidenceSourceValue.model_validate(source_value) if isinstance(source_value, Mapping) else source_value
    return EvidenceValue(
        value=value,
        origin=ValueOrigin.OBSERVED,
        unit=unit,
        source_record_ids=_normalize_source_record_ids(source_record_id, source_record_ids),
        confidence=EvidenceConfidence(confidence),
        method=method,
        source_value=normalized_source_value,
    )
    ####


def reported(
    value: EvidenceScalar,
    *,
    source_record_id: str | None = None,
    source_record_ids: Sequence[str] = (),
    unit: str | None = None,
    confidence: EvidenceConfidence | str = EvidenceConfidence.MEDIUM,
) -> EvidenceValue:
    """Mark a public performance claim or secondary-source value."""

    return EvidenceValue(
        value=value,
        origin=ValueOrigin.REPORTED,
        unit=unit,
        source_record_ids=_normalize_source_record_ids(source_record_id, source_record_ids),
        confidence=EvidenceConfidence(confidence),
    )
    ####


def assumed(
    value: EvidenceScalar,
    *,
    unit: str | None = None,
    method: str | None = None,
) -> EvidenceValue:
    """Mark a deliberate developer-supplied simulation assumption."""

    return EvidenceValue(
        value=value,
        origin=ValueOrigin.SIMULATION_ASSUMPTION,
        unit=unit,
        confidence=EvidenceConfidence.LOW,
        method=method,
    )
    ####


def calibrated(
    value: EvidenceScalar,
    *,
    unit: str | None = None,
    method: str,
) -> EvidenceValue:
    """Mark a simulation-only value produced by a reproducible fit receipt."""

    return EvidenceValue(
        value=value,
        origin=ValueOrigin.CALIBRATED,
        unit=unit,
        confidence=EvidenceConfidence.MEDIUM,
        method=method,
    )
    ####


def derived(
    value: EvidenceScalar,
    *,
    method: str,
    unit: str | None = None,
    source_record_id: str | None = None,
    source_record_ids: Sequence[str] = (),
    confidence: EvidenceConfidence | str = EvidenceConfidence.MEDIUM,
) -> EvidenceValue:
    """Mark a value computed from stated inputs without presenting it as observed."""

    return EvidenceValue(
        value=value,
        origin=ValueOrigin.DERIVED,
        unit=unit,
        source_record_ids=_normalize_source_record_ids(source_record_id, source_record_ids),
        confidence=EvidenceConfidence(confidence),
        method=method,
    )
    ####


def inferred(
    value: EvidenceScalar,
    *,
    method: str,
    unit: str | None = None,
    source_record_id: str | None = None,
    source_record_ids: Sequence[str] = (),
    confidence: EvidenceConfidence | str = EvidenceConfidence.LOW,
) -> EvidenceValue:
    """Mark a source-informed interpretation that was not directly observed."""

    return EvidenceValue(
        value=value,
        origin=ValueOrigin.INFERRED,
        unit=unit,
        source_record_ids=_normalize_source_record_ids(source_record_id, source_record_ids),
        confidence=EvidenceConfidence(confidence),
        method=method,
    )
    ####


def unavailable(
    parameter_id: str,
    status: str,
    *,
    unit: str | None = None,
    source_record_id: str | None = None,
    source_record_ids: Sequence[str] = (),
    note: str = "",
) -> EvidenceGap:
    """Create a typed classified, missing, or otherwise unavailable evidence record."""

    return EvidenceGap(
        parameter_id=parameter_id,
        status=status,
        unit=unit,
        source_record_ids=_normalize_source_record_ids(source_record_id, source_record_ids),
        note=note,
    )
    ####


def variant_interval(
    parameter_id: str,
    minimum_value: float,
    maximum_value: float,
    *,
    unit: str,
    origin: str = "observed_variant_interval",
    source_record_id: str | None = None,
    source_record_ids: Sequence[str] = (),
    source_minimum_value: float | None = None,
    source_maximum_value: float | None = None,
    source_unit: str | None = None,
    note: str = "",
) -> EvidenceInterval:
    """Create a family/variant range without silently selecting a point value."""

    return EvidenceInterval(
        parameter_id=parameter_id,
        minimum_value=minimum_value,
        maximum_value=maximum_value,
        unit=unit,
        origin=origin,
        source_record_ids=_normalize_source_record_ids(source_record_id, source_record_ids),
        source_minimum_value=source_minimum_value,
        source_maximum_value=source_maximum_value,
        source_unit=source_unit,
        note=note,
    )
    ####


def _normalize_source_record_ids(
    source_record_id: str | None,
    source_record_ids: Sequence[str],
) -> tuple[str, ...]:
    """Normalize singular/plural source arguments without discarding ambiguity."""

    if isinstance(source_record_ids, str):
        raise TypeError("source_record_ids must be a sequence of complete identifiers, not one string")
    plural = tuple(source_record_ids)
    if source_record_id is not None and plural:
        raise ValueError("choose source_record_id or source_record_ids, not both")
    normalized = (source_record_id,) if source_record_id is not None else plural
    if any(not isinstance(item, str) or not item or item != item.strip() for item in normalized):
        raise ValueError("source record IDs must be non-empty strings without surrounding whitespace")
    if len(normalized) != len(set(normalized)):
        raise ValueError("source record IDs must be unique")
    return normalized
    ####


class InterceptorEvidenceProfile(BaseModel):
    """Sparse ontology-to-simulation profile.

    Only identity is structurally required. Missing engineering values are
    filled by a versioned archetype and remain visibly marked as assumptions.
    That keeps early catalog intake easy without laundering defaults into facts.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    interceptor_id: str = Field(pattern=r"^[a-z0-9]+(?:[._-][a-z0-9]+)*$")
    variant: str = "baseline"
    parameter_set_version: str = "0.1.0"
    surrogate_archetype_id: str = "generic_slender_sam_v1"
    default_assumption_case: AssumptionCase = AssumptionCase.NOMINAL
    calibration_reference_model_id: str | None = None
    catalogue_interceptor_id: str | None = None
    variant_basis: str | None = None
    application_context: str | None = None
    resolution_status: ResolutionStatus = ResolutionStatus.UNASSESSED
    required_diagnostics: tuple[str, ...] = ()
    evidence_gaps: tuple[EvidenceGap, ...] = ()
    evidence_intervals: tuple[EvidenceInterval, ...] = ()

    launch_mass_kg: EvidenceValue | None = None
    burnout_mass_kg: EvidenceValue | None = None
    propellant_fraction: EvidenceValue | None = None
    effective_specific_impulse_s: EvidenceValue | None = None
    length_m: EvidenceValue | None = None
    body_diameter_m: EvidenceValue | None = None
    wingspan_m: EvidenceValue | None = None
    reference_area_m2: EvidenceValue | None = None
    reference_length_m: EvidenceValue | None = None
    slenderness_ratio: EvidenceValue | None = None
    propulsion_architecture: EvidenceValue | None = None
    motor_designation: EvidenceValue | None = None
    burn_time_class: EvidenceValue | None = None
    burn_time_s: EvidenceValue | None = None
    thrust_profile_class: EvidenceValue | None = None
    thrust_profile_schedule: ThrustProfileSchedule | None = None
    second_pulse_thrust_profile_schedule: ThrustProfileSchedule | None = None
    thrust_time_curve: AbsoluteThrustCurve | None = None
    dual_pulse_thrust_program: DualPulseThrustProgram | None = None
    nominal_thrust_n: EvidenceValue | None = None
    thrust_scale: EvidenceValue | None = None
    drag_scale: EvidenceValue | None = None
    maneuverability_scale: EvidenceValue | None = None
    guidance_time_constant_scale: EvidenceValue | None = None
    first_pulse_burn_time_s: EvidenceValue | None = None
    inter_pulse_coast_time_s: EvidenceValue | None = None
    second_pulse_burn_time_s: EvidenceValue | None = None
    second_pulse_thrust_ratio: EvidenceValue | None = None
    second_pulse_propellant_fraction: EvidenceValue | None = None
    aero_archetype: EvidenceValue | None = None
    drag_class: EvidenceValue | None = None
    drag_coefficient_schedule: DragCoefficientSchedule | None = None
    maneuver_drag_factor: EvidenceValue | None = None
    maneuverability_class: EvidenceValue | None = None
    control_configuration: EvidenceValue | None = None
    control_allocation_policy: EvidenceValue | None = None
    normal_force_coefficient_limit: EvidenceValue | None = None
    max_thrust_vector_angle_rad: EvidenceValue | None = None
    control_bandwidth_class: EvidenceValue | None = None
    attitude_bandwidth_rad_s: EvidenceValue | None = None
    attitude_damping_ratio: EvidenceValue | None = None
    max_body_rate_rad_s: EvidenceValue | None = None
    max_body_acceleration_rad_s2: EvidenceValue | None = None
    max_bank_angle_rad: EvidenceValue | None = None
    guidance_family: EvidenceValue | None = None
    guidance_archetype: EvidenceValue | None = None
    navigation_constant: EvidenceValue | None = None
    control_features: EvidenceValue | None = None
    intercept_mechanism: EvidenceValue | None = None
    applicability_altitude_min_m: EvidenceValue | None = None
    applicability_altitude_max_m: EvidenceValue | None = None
    applicability_mach_min: EvidenceValue | None = None
    applicability_mach_max: EvidenceValue | None = None
    reported_max_speed_mps: EvidenceValue | None = None
    reported_max_range_m: EvidenceValue | None = None
    reported_max_altitude_m: EvidenceValue | None = None
    target_classes: EvidenceValue | None = None
    source_record_ids: tuple[str, ...] = ()
    model_notes: str = ""

    @field_validator(
        "launch_mass_kg",
        "burnout_mass_kg",
        "length_m",
        "body_diameter_m",
        "wingspan_m",
        "reference_area_m2",
        "reference_length_m",
        "slenderness_ratio",
        "effective_specific_impulse_s",
        "reported_max_speed_mps",
        "reported_max_range_m",
        "reported_max_altitude_m",
        "burn_time_s",
        "first_pulse_burn_time_s",
        "second_pulse_burn_time_s",
        "second_pulse_thrust_ratio",
        "nominal_thrust_n",
        "thrust_scale",
        "drag_scale",
        "maneuverability_scale",
        "guidance_time_constant_scale",
        "attitude_bandwidth_rad_s",
        "attitude_damping_ratio",
        "max_body_rate_rad_s",
        "max_body_acceleration_rad_s2",
        "max_bank_angle_rad",
        "navigation_constant",
        "normal_force_coefficient_limit",
        "max_thrust_vector_angle_rad",
    )
    @classmethod
    def positive_values(cls, item: EvidenceValue | None) -> EvidenceValue | None:
        if item is not None and _number(item) <= 0.0:
            raise ValueError("physical and reported scalar values must be positive")
        return item
        ####

    @field_validator("propellant_fraction")
    @classmethod
    def valid_fraction(cls, item: EvidenceValue | None) -> EvidenceValue | None:
        if item is not None and not 0.0 < _number(item) < 1.0:
            raise ValueError("propellant_fraction must lie strictly between zero and one")
        return item
        ####

    @field_validator("inter_pulse_coast_time_s")
    @classmethod
    def nonnegative_coast_time(cls, item: EvidenceValue | None) -> EvidenceValue | None:
        if item is not None and _number(item) < 0.0:
            raise ValueError("inter_pulse_coast_time_s must be nonnegative")
        return item
        ####

    @field_validator(
        "applicability_altitude_min_m",
        "applicability_altitude_max_m",
        "applicability_mach_min",
        "applicability_mach_max",
    )
    @classmethod
    def nonnegative_applicability_bounds(cls, item: EvidenceValue | None) -> EvidenceValue | None:
        if item is not None and _number(item) < 0.0:
            raise ValueError("applicability bounds must be nonnegative")
        return item
        ####

    @field_validator("maneuver_drag_factor")
    @classmethod
    def nonnegative_maneuver_drag_factor(cls, item: EvidenceValue | None) -> EvidenceValue | None:
        if item is not None and _number(item) < 0.0:
            raise ValueError("maneuver_drag_factor must be nonnegative")
        return item
        ####

    @field_validator("second_pulse_propellant_fraction")
    @classmethod
    def valid_second_pulse_fraction(cls, item: EvidenceValue | None) -> EvidenceValue | None:
        if item is not None and not 0.0 < _number(item) < 1.0:
            raise ValueError("second_pulse_propellant_fraction must lie strictly between zero and one")
        return item
        ####

    @field_validator("attitude_damping_ratio")
    @classmethod
    def valid_attitude_damping(cls, item: EvidenceValue | None) -> EvidenceValue | None:
        if item is not None and _number(item) > 2.0:
            raise ValueError("attitude_damping_ratio must not exceed 2")
        return item
        ####

    @field_validator("max_bank_angle_rad")
    @classmethod
    def valid_bank_angle(cls, item: EvidenceValue | None) -> EvidenceValue | None:
        if item is not None and _number(item) > math.pi / 2.0:
            raise ValueError("max_bank_angle_rad must not exceed pi/2")
        return item
        ####

    @field_validator("max_thrust_vector_angle_rad")
    @classmethod
    def valid_thrust_vector_angle(cls, item: EvidenceValue | None) -> EvidenceValue | None:
        if item is not None and _number(item) >= math.pi / 2.0:
            raise ValueError("max_thrust_vector_angle_rad must be below pi/2")
        return item
        ####

    @model_validator(mode="after")
    def validate_mass_order(self) -> Self:
        if self.launch_mass_kg is not None and self.burnout_mass_kg is not None:
            if _number(self.burnout_mass_kg) >= _number(self.launch_mass_kg):
                raise ValueError("burnout_mass_kg must be below launch_mass_kg")
        return self
        ####

    @model_validator(mode="after")
    def validate_specific_impulse_mass_authority(self) -> Self:
        if self.effective_specific_impulse_s is None:
            return self
        conflicts = tuple(
            name
            for name in (
                "burnout_mass_kg",
                "propellant_fraction",
                "second_pulse_propellant_fraction",
            )
            if getattr(self, name) is not None
        )
        if conflicts:
            raise ValueError(f"effective_specific_impulse_s derives burnout mass and propellant allocation; remove these overlapping fields: {conflicts!r}")
        if self.launch_mass_kg is None:
            raise ValueError("effective_specific_impulse_s requires an explicit launch_mass_kg")
        explicit_amplitude = any(
            item is not None
            for item in (
                self.nominal_thrust_n,
                self.thrust_time_curve,
                self.dual_pulse_thrust_program,
            )
        )
        if not explicit_amplitude:
            raise ValueError("effective_specific_impulse_s requires nominal_thrust_n, thrust_time_curve, or dual_pulse_thrust_program")
        explicit_timing = any(
            item is not None
            for item in (
                self.burn_time_s,
                self.thrust_time_curve,
                self.dual_pulse_thrust_program,
            )
        ) or (self.first_pulse_burn_time_s is not None and self.second_pulse_burn_time_s is not None)
        if not explicit_timing:
            raise ValueError("effective_specific_impulse_s requires burn_time_s, both pulse durations, thrust_time_curve, or dual_pulse_thrust_program")
        return self
        ####

    @model_validator(mode="after")
    def validate_drag_authority(self) -> Self:
        if self.drag_coefficient_schedule is not None:
            conflicting = tuple(name for name in ("aero_archetype", "drag_class") if getattr(self, name) is not None)
            if conflicting:
                raise ValueError(f"drag_coefficient_schedule replaces these coarse aerodynamic selectors: {conflicting!r}")
        return self
        ####

    @model_validator(mode="after")
    def validate_thrust_authority(self) -> Self:
        if self.thrust_time_curve is not None and self.dual_pulse_thrust_program is not None:
            raise ValueError("choose exactly one of thrust_time_curve or dual_pulse_thrust_program")
        if self.thrust_time_curve is not None:
            conflicting = tuple(
                name
                for name in (
                    "burn_time_class",
                    "burn_time_s",
                    "thrust_profile_class",
                    "thrust_profile_schedule",
                    "nominal_thrust_n",
                    "first_pulse_burn_time_s",
                    "inter_pulse_coast_time_s",
                    "second_pulse_burn_time_s",
                    "second_pulse_thrust_ratio",
                    "second_pulse_propellant_fraction",
                )
                if getattr(self, name) is not None
            )
            if conflicting:
                raise ValueError(
                    f"thrust_time_curve derives burn duration, nominal mean thrust, and normalized shape; remove these overlapping fields: {conflicting!r}"
                )
        if self.dual_pulse_thrust_program is not None:
            conflicting = tuple(
                name
                for name in (
                    "burn_time_class",
                    "burn_time_s",
                    "thrust_profile_class",
                    "thrust_profile_schedule",
                    "second_pulse_thrust_profile_schedule",
                    "nominal_thrust_n",
                    "first_pulse_burn_time_s",
                    "inter_pulse_coast_time_s",
                    "second_pulse_burn_time_s",
                    "second_pulse_thrust_ratio",
                )
                if getattr(self, name) is not None
            )
            if conflicting:
                raise ValueError(
                    "dual_pulse_thrust_program derives architecture, pulse timing, mean thrust, ratio, and shapes; "
                    f"remove these overlapping fields: {conflicting!r}"
                )
            if self.propulsion_architecture is not None and self.propulsion_architecture.value != "dual_pulse_solid":
                raise ValueError("dual_pulse_thrust_program requires propulsion_architecture dual_pulse_solid when explicitly supplied")
        if self.thrust_profile_schedule is not None and self.nominal_thrust_n is not None and self.thrust_profile_class is not None:
            raise ValueError(
                "thrust_profile_schedule plus nominal_thrust_n fully replace an explicitly supplied thrust_profile_class; omit the unused coarse selector"
            )
        return self
        ####

    @model_validator(mode="after")
    def validate_applicability_order(self) -> Self:
        pairs = (
            ("altitude", self.applicability_altitude_min_m, self.applicability_altitude_max_m),
            ("Mach", self.applicability_mach_min, self.applicability_mach_max),
        )
        for label, lower, upper in pairs:
            if lower is not None and upper is not None and _number(lower) >= _number(upper):
                raise ValueError(f"applicability {label} minimum must be below its maximum")
        return self
        ####

    @property
    def model_id(self) -> str:
        """Return the stable Composition model ID."""

        suffix = re.sub(r"[^a-z0-9]+", "-", self.variant.lower()).strip("-")
        return self.interceptor_id if suffix in {"", "baseline"} else f"{self.interceptor_id}-{suffix}"
        ####

    @property
    def fingerprint(self) -> str:
        """Return a stable identity for sparse evidence and authoring inputs."""

        payload = self.model_dump(mode="json")
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        return hashlib.sha256(encoded).hexdigest()
        ####

    ####


class ResolvedParameter(BaseModel):
    """A complete numerical or categorical value with its resolution trace."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    value: EvidenceScalar
    origin: ValueOrigin
    unit: str | None = None
    source_record_ids: tuple[str, ...] = ()
    confidence: EvidenceConfidence = EvidenceConfidence.UNKNOWN
    method: str
    archetype_id: str | None = None
    assumption_case: AssumptionCase | None = None
    depends_on: tuple[str, ...] = ()
    source_value: EvidenceSourceValue | None = None


class ResolvedInterceptorProfile(BaseModel):
    """Complete immutable case with executable and non-executable values.

    Use :func:`interceptor_parameter_usage` to distinguish direct dynamics
    inputs from active/inactive resolver selectors, runtime advisories,
    calibration targets, derived summaries, and evidence-only metadata.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    interceptor_id: str
    model_id: str
    variant: str
    parameter_set_version: str
    surrogate_archetype_id: str
    assumption_case: AssumptionCase
    calibration_reference_model_id: str | None = None
    catalogue_interceptor_id: str | None = None
    variant_basis: str | None = None
    application_context: str | None = None
    resolution_status: ResolutionStatus = ResolutionStatus.UNASSESSED
    required_diagnostics: tuple[str, ...] = ()
    evidence_gaps: tuple[EvidenceGap, ...] = ()
    evidence_intervals: tuple[EvidenceInterval, ...] = ()
    drag_coefficient_schedule: DragCoefficientSchedule
    thrust_profile_schedule: ThrustProfileSchedule
    thrust_profile_schedule_depends_on: tuple[str, ...] = ()
    second_pulse_thrust_profile_schedule: ThrustProfileSchedule | None = None
    thrust_time_curve: AbsoluteThrustCurve | None = None
    dual_pulse_thrust_program: DualPulseThrustProgram | None = None
    parameters: dict[str, ResolvedParameter]
    source_record_ids: tuple[str, ...] = ()
    model_notes: str = ""

    def number(self, name: str) -> float:
        """Return one resolved numeric value."""

        value = self.parameters[name].value
        if isinstance(value, bool) or not isinstance(value, int | float):
            raise TypeError(f"resolved parameter {name!r} is not numeric")
        return float(value)
        ####

    def text(self, name: str) -> str:
        """Return one resolved categorical value."""

        value = self.parameters[name].value
        if not isinstance(value, str):
            raise TypeError(f"resolved parameter {name!r} is not text")
        return value
        ####

    @property
    def fingerprint(self) -> str:
        """Return a reproducible identity for the fully resolved case."""

        payload = self.model_dump(mode="json")
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        return hashlib.sha256(encoded).hexdigest()
        ####

    ####


PARAMETER_USAGE_CONTRACT: Literal["taoryx.parametric-interceptors.parameter-usage/v1"] = "taoryx.parametric-interceptors.parameter-usage/v1"
ParameterUsageClass = Literal[
    "dynamics_input",
    "resolution_input",
    "inactive_resolution_input",
    "runtime_advisory",
    "calibration_target",
    "derived_summary",
    "evidence_only",
]


class InterceptorParameterUsage(BaseModel):
    """One fail-closed record of how a resolved value is actually consumed."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    contract: Literal["taoryx.parametric-interceptors.parameter-usage/v1"] = PARAMETER_USAGE_CONTRACT
    parameter_id: str = Field(min_length=1)
    usage_class: ParameterUsageClass
    directly_consumed: bool
    affects_dynamics: bool
    consumer_ids: tuple[str, ...]
    resolved_sink_ids: tuple[str, ...]
    description: str = Field(min_length=1)


####


_COMMON_RUNTIME_CONSUMERS = (
    "runtime.point_mass_3dof",
    "runtime.attitude_response_pseudo_6dof",
)
_PSEUDO6_RUNTIME_CONSUMERS = (
    "runtime.attitude_response_pseudo_6dof",
    "analysis.pseudo6_response",
)
_OPERATING_POINT_RESPONSE_CONSUMERS = (
    *_COMMON_RUNTIME_CONSUMERS,
    "analysis.pseudo6_operating_point_response",
)
_OPERATING_POINT_RESPONSE_INPUTS = frozenset(
    {
        "reference_area_m2",
        "max_lateral_acceleration_mps2",
        "control_configuration",
        "normal_force_coefficient_limit",
        "max_thrust_vector_angle_rad",
    }
)
_COMMON_DYNAMICS_INPUTS = frozenset(
    {
        "launch_mass_kg",
        "burnout_mass_kg",
        "reference_area_m2",
        "propulsion_architecture",
        "thrust_profile_schedule",
        "second_pulse_thrust_profile_schedule",
        "first_pulse_burn_time_s",
        "inter_pulse_coast_time_s",
        "second_pulse_burn_time_s",
        "second_pulse_thrust_ratio",
        "second_pulse_propellant_fraction",
        "thrust_n",
        "drag_coefficient_schedule",
        "maneuver_drag_factor",
        "max_lateral_acceleration_mps2",
        "control_configuration",
        "control_allocation_policy",
        "normal_force_coefficient_limit",
        "max_thrust_vector_angle_rad",
        "guidance_time_constant_s",
        "guidance_archetype",
        "navigation_constant",
    }
)
_PSEUDO6_DYNAMICS_INPUTS = frozenset(
    {
        "attitude_bandwidth_rad_s",
        "attitude_damping_ratio",
        "max_body_rate_rad_s",
        "max_body_acceleration_rad_s2",
        "max_bank_angle_rad",
    }
)
_RUNTIME_ADVISORY_INPUTS = frozenset(
    {
        "applicability_altitude_min_m",
        "applicability_altitude_max_m",
        "applicability_mach_min",
        "applicability_mach_max",
    }
)
_CALIBRATION_TARGET_PARAMETERS = frozenset(
    {
        "reported_max_speed_mps",
        "reported_max_range_m",
        "reported_max_altitude_m",
    }
)
_DERIVED_SUMMARY_PARAMETERS = frozenset(
    {
        "reference_length_m",
        "slenderness_ratio",
        "mass_flow_kg_s",
        "drag_coefficient",
        "active_burn_total_impulse_n_s",
    }
)
_EVIDENCE_ONLY_PARAMETERS = frozenset(
    {
        "length_m",
        "wingspan_m",
        "motor_designation",
        "guidance_family",
        "intercept_mechanism",
        "target_classes",
    }
)
_POTENTIAL_RESOLUTION_INPUTS = frozenset(
    {
        "body_diameter_m",
        "propellant_fraction",
        "effective_specific_impulse_s",
        "burn_time_class",
        "burn_time_s",
        "aero_archetype",
        "drag_class",
        "maneuverability_class",
        "control_bandwidth_class",
        "thrust_scale",
        "drag_scale",
        "maneuverability_scale",
        "guidance_time_constant_scale",
        "control_features",
    }
)


def interceptor_parameter_usage(
    profile: ResolvedInterceptorProfile,
) -> dict[str, InterceptorParameterUsage]:
    """Return an exact usage record for every resolved value and schedule.

    Classification is derived from direct runtime sinks plus the resolved
    ``depends_on`` graph. Any new unconsumed parameter must be assigned an
    explicit non-executable role or this function fails, preventing silent
    baggage from entering discovery metadata.
    """

    direct_consumers = {
        **{identifier: _COMMON_RUNTIME_CONSUMERS for identifier in _COMMON_DYNAMICS_INPUTS},
        **{identifier: _OPERATING_POINT_RESPONSE_CONSUMERS for identifier in _OPERATING_POINT_RESPONSE_INPUTS},
        **{identifier: _PSEUDO6_RUNTIME_CONSUMERS for identifier in _PSEUDO6_DYNAMICS_INPUTS},
        **{identifier: ("runtime.applicability_advisory",) for identifier in _RUNTIME_ADVISORY_INPUTS},
    }
    downstream: dict[str, set[str]] = {}
    for dependent_id, parameter in profile.parameters.items():
        for dependency_id in parameter.depends_on:
            downstream.setdefault(dependency_id, set()).add(dependent_id)
    for dependency_id in ("aero_archetype", "drag_class", "drag_scale"):
        if dependency_id in profile.parameters:
            downstream.setdefault(dependency_id, set()).add("drag_coefficient_schedule")
    for dependency_id in profile.thrust_profile_schedule_depends_on:
        if dependency_id in profile.parameters:
            downstream.setdefault(dependency_id, set()).add("thrust_profile_schedule")
    if profile.thrust_time_curve is not None:
        for dependent_id in (
            "burn_time_s",
            "nominal_thrust_n",
            "thrust_profile_schedule",
        ):
            downstream.setdefault("thrust_time_curve", set()).add(dependent_id)
    if profile.dual_pulse_thrust_program is not None:
        for dependent_id in (
            "propulsion_architecture",
            "burn_time_s",
            "nominal_thrust_n",
            "first_pulse_burn_time_s",
            "inter_pulse_coast_time_s",
            "second_pulse_burn_time_s",
            "second_pulse_thrust_ratio",
            "thrust_profile_schedule",
            "second_pulse_thrust_profile_schedule",
        ):
            downstream.setdefault("dual_pulse_thrust_program", set()).add(dependent_id)

    def resolved_sinks(parameter_id: str, visiting: frozenset[str] = frozenset()) -> set[str]:
        if parameter_id in visiting:
            raise ValueError(f"resolved interceptor parameter dependency cycle includes {parameter_id!r}")
        sinks = {parameter_id} if parameter_id in direct_consumers else set()
        next_visiting = visiting | {parameter_id}
        for dependent_id in downstream.get(parameter_id, ()):
            sinks.update(resolved_sinks(dependent_id, next_visiting))
        return sinks
        ####

    records: dict[str, InterceptorParameterUsage] = {}
    structured_parameter_ids = (
        "drag_coefficient_schedule",
        "thrust_profile_schedule",
        *(("second_pulse_thrust_profile_schedule",) if profile.second_pulse_thrust_profile_schedule is not None else ()),
        *(("thrust_time_curve",) if profile.thrust_time_curve is not None else ()),
        *(("dual_pulse_thrust_program",) if profile.dual_pulse_thrust_program is not None else ()),
    )
    parameter_ids = (*profile.parameters, *structured_parameter_ids)
    for parameter_id in parameter_ids:
        sinks = tuple(sorted(resolved_sinks(parameter_id)))
        consumers = tuple(sorted({consumer for sink in sinks for consumer in direct_consumers[sink]}))
        if parameter_id in _COMMON_DYNAMICS_INPUTS | _PSEUDO6_DYNAMICS_INPUTS:
            usage_class: ParameterUsageClass = "dynamics_input"
            directly_consumed = True
            affects_dynamics = True
            description = "Read directly by the advertised runtime/analysis consumers."
        elif parameter_id in _RUNTIME_ADVISORY_INPUTS:
            usage_class = "runtime_advisory"
            directly_consumed = True
            affects_dynamics = False
            description = "Read at runtime for advisory applicability status; it does not alter forces or state propagation."
        elif parameter_id in _CALIBRATION_TARGET_PARAMETERS:
            usage_class = "calibration_target"
            directly_consumed = False
            affects_dynamics = False
            consumers = ("calibration.scenario_target",)
            description = "Scenario-qualified comparison target; never inserted into the force or guidance equations."
        elif parameter_id in _DERIVED_SUMMARY_PARAMETERS:
            usage_class = "derived_summary"
            directly_consumed = False
            affects_dynamics = False
            consumers = ("composition.metadata",)
            description = "Resolved reference/summary value published for inspection but not consumed by runtime dynamics."
        elif sinks:
            usage_class = "resolution_input"
            directly_consumed = False
            affects_dynamics = True
            description = f"Consumed during resolution and reaches active executable inputs: {', '.join(sinks)}."
        elif parameter_id in _POTENTIAL_RESOLUTION_INPUTS:
            usage_class = "inactive_resolution_input"
            directly_consumed = False
            affects_dynamics = False
            consumers = ("resolver.inactive_or_overridden",)
            description = "Coarse resolver input retained for provenance but overridden or inactive in this resolved case."
        elif parameter_id in _EVIDENCE_ONLY_PARAMETERS:
            usage_class = "evidence_only"
            directly_consumed = False
            affects_dynamics = False
            consumers = ("composition.metadata",)
            description = "Evidence/context value only; the current low-fidelity kernels do not consume it."
        else:
            raise ValueError(f"resolved interceptor parameter {parameter_id!r} has no declared runtime, resolver, calibration, summary, or evidence-only usage")
        records[parameter_id] = InterceptorParameterUsage(
            parameter_id=parameter_id,
            usage_class=usage_class,
            directly_consumed=directly_consumed,
            affects_dynamics=affects_dynamics,
            consumer_ids=consumers,
            resolved_sink_ids=sinks,
            description=description,
        )
    return records
    ####


_STRUCTURED_FIELDS = {
    "drag_coefficient_schedule",
    "thrust_profile_schedule",
    "second_pulse_thrust_profile_schedule",
    "thrust_time_curve",
    "dual_pulse_thrust_program",
}
_EVIDENCE_FIELDS = (
    set(InterceptorEvidenceProfile.model_fields)
    - _STRUCTURED_FIELDS
    - {
        "interceptor_id",
        "variant",
        "parameter_set_version",
        "surrogate_archetype_id",
        "default_assumption_case",
        "calibration_reference_model_id",
        "catalogue_interceptor_id",
        "variant_basis",
        "application_context",
        "resolution_status",
        "required_diagnostics",
        "evidence_gaps",
        "evidence_intervals",
        "source_record_ids",
        "model_notes",
    }
)


def interceptor(interceptor_id: str, /, **values: Any) -> InterceptorEvidenceProfile:
    """Create a sparse profile from plain Python/YAML-shaped values.

    Raw engineering values are deliberately tagged ``simulation_assumption``.
    This makes the shortest authoring form safe while the evidence helpers let
    research-backed callers add provenance one field at a time.
    """

    normalized = dict(values)
    schedule = normalized.get("drag_coefficient_schedule")
    if schedule is not None and not isinstance(schedule, DragCoefficientSchedule):
        normalized["drag_coefficient_schedule"] = DragCoefficientSchedule.model_validate(schedule)
    thrust_schedule = normalized.get("thrust_profile_schedule")
    if thrust_schedule is not None and not isinstance(thrust_schedule, ThrustProfileSchedule):
        normalized["thrust_profile_schedule"] = ThrustProfileSchedule.model_validate(thrust_schedule)
    second_thrust_schedule = normalized.get("second_pulse_thrust_profile_schedule")
    if second_thrust_schedule is not None and not isinstance(second_thrust_schedule, ThrustProfileSchedule):
        normalized["second_pulse_thrust_profile_schedule"] = ThrustProfileSchedule.model_validate(second_thrust_schedule)
    thrust_curve = normalized.get("thrust_time_curve")
    if thrust_curve is not None and not isinstance(thrust_curve, AbsoluteThrustCurve):
        normalized["thrust_time_curve"] = AbsoluteThrustCurve.model_validate(thrust_curve)
    dual_program = normalized.get("dual_pulse_thrust_program")
    if dual_program is not None and not isinstance(dual_program, DualPulseThrustProgram):
        normalized["dual_pulse_thrust_program"] = DualPulseThrustProgram.model_validate(dual_program)
    for name in _EVIDENCE_FIELDS & normalized.keys():
        value = normalized[name]
        if value is not None and not isinstance(value, EvidenceValue):
            if isinstance(value, Mapping) and "value" in value:
                value = EvidenceValue.model_validate(value)
            elif isinstance(value, Mapping):
                raise ValueError(f"evidence mapping for {name!r} requires value and origin fields")
            elif name in {"target_classes", "control_features"} and isinstance(value, list):
                value = tuple(str(item) for item in value)
            if not isinstance(value, EvidenceValue):
                if isinstance(value, tuple):
                    if not all(isinstance(item, str) for item in value):
                        raise ValueError(f"tuple profile value {name!r} must contain strings")
                    raw: EvidenceScalar = value
                elif isinstance(value, float | int | str | bool):
                    raw = value
                else:
                    raise ValueError(f"unsupported profile value for {name!r}: {type(value).__name__}")
                value = assumed(raw, unit=_DEFAULT_UNITS.get(name))
            normalized[name] = value
    return InterceptorEvidenceProfile.model_validate({"interceptor_id": interceptor_id, **normalized})
    ####


def interceptor_from_mapping(payload: Mapping[str, Any]) -> InterceptorEvidenceProfile:
    """Build a profile from a JSON/YAML-shaped mapping."""

    values = dict(payload)
    interceptor_id = values.pop("interceptor_id", None)
    if not isinstance(interceptor_id, str) or not interceptor_id:
        raise ValueError("interceptor profile mapping requires a non-empty interceptor_id")
    return interceptor(interceptor_id, **values)
    ####


def load_interceptor_profile(path: str | Path) -> InterceptorEvidenceProfile:
    """Load one ergonomic YAML profile without a provider-specific toolchain."""

    source = Path(path)
    try:
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"could not load interceptor profile {source}: {error}") from error
    if not isinstance(payload, Mapping):
        raise ValueError(f"interceptor profile {source} must contain one mapping")
    return interceptor_from_mapping(payload)
    ####


def flat_interceptor_profile_schema() -> dict[str, Any]:
    """Return JSON Schema for the scalar-or-evidence flat YAML surface."""

    schema = InterceptorEvidenceProfile.model_json_schema()
    schema["$id"] = "https://taoryx.dev/schemas/parametric-interceptor-flat/v1"
    schema["title"] = "Taoryx flat parametric interceptor profile"
    schema["description"] = (
        "File-first sparse interceptor profile. Raw field values are simulation assumptions; evidence objects retain explicit origin and provenance."
    )
    properties = schema.get("properties")
    if not isinstance(properties, dict):
        raise RuntimeError("interceptor profile JSON Schema is missing properties")
    for name in _EVIDENCE_FIELDS:
        field_schema = properties.get(name)
        if not isinstance(field_schema, dict):
            raise RuntimeError(f"interceptor profile JSON Schema is missing {name!r}")
        if name in {"target_classes", "control_features"}:
            value_schema: dict[str, object] = {"type": "array", "items": {"type": "string"}}
        elif name == "control_allocation_policy":
            value_schema = {"type": "string", "enum": list(CONTROL_ALLOCATION_POLICIES)}
        elif name in _DEFAULT_UNITS:
            value_schema = {"type": "number"}
        else:
            value_schema = {"type": "string"}
        evidence_schema = {
            "allOf": [
                {"$ref": "#/$defs/EvidenceValue"},
                {"type": "object", "properties": {"value": value_schema}},
            ]
        }
        field_schema["anyOf"] = [value_schema, evidence_schema, {"type": "null"}]
    return schema
    ####


def _number(item: EvidenceValue) -> float:
    value = item.value
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError("expected numeric evidence value")
    return float(value)
    ####


_DEFAULT_UNITS: dict[str, str] = {
    "launch_mass_kg": "kg",
    "burnout_mass_kg": "kg",
    "propellant_fraction": "1",
    "length_m": "m",
    "body_diameter_m": "m",
    "wingspan_m": "m",
    "reference_area_m2": "m^2",
    "reference_length_m": "m",
    "slenderness_ratio": "1",
    "reported_max_speed_mps": "m/s",
    "reported_max_range_m": "m",
    "reported_max_altitude_m": "m",
    "applicability_altitude_min_m": "m",
    "applicability_altitude_max_m": "m",
    "applicability_mach_min": "1",
    "applicability_mach_max": "1",
    "burn_time_s": "s",
    "first_pulse_burn_time_s": "s",
    "inter_pulse_coast_time_s": "s",
    "second_pulse_burn_time_s": "s",
    "second_pulse_thrust_ratio": "1",
    "second_pulse_propellant_fraction": "1",
    "nominal_thrust_n": "N",
    "thrust_scale": "1",
    "drag_scale": "1",
    "maneuverability_scale": "1",
    "guidance_time_constant_scale": "1",
    "attitude_bandwidth_rad_s": "rad/s",
    "attitude_damping_ratio": "1",
    "max_body_rate_rad_s": "rad/s",
    "max_body_acceleration_rad_s2": "rad/s^2",
    "max_bank_angle_rad": "rad",
    "normal_force_coefficient_limit": "1",
    "max_thrust_vector_angle_rad": "rad",
    "maneuver_drag_factor": "1",
    "navigation_constant": "1",
}


__all__ = [
    "AssumptionCase",
    "EvidenceGap",
    "EvidenceConfidence",
    "EvidenceInterval",
    "EvidenceSourceValue",
    "EvidenceValue",
    "InterceptorEvidenceProfile",
    "InterceptorParameterUsage",
    "PARAMETER_USAGE_CONTRACT",
    "ParameterUsageClass",
    "ResolvedInterceptorProfile",
    "ResolvedParameter",
    "ResolutionStatus",
    "ValueOrigin",
    "assumed",
    "calibrated",
    "derived",
    "flat_interceptor_profile_schema",
    "interceptor",
    "interceptor_parameter_usage",
    "inferred",
    "interceptor_from_mapping",
    "load_interceptor_profile",
    "observed",
    "reported",
    "unavailable",
    "variant_interval",
]
####
