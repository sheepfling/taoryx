"""Versioned archetype resolution for sparse interceptor evidence profiles."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Final, cast

from .aerodynamics import DragCoefficientPoint, DragCoefficientSchedule
from .control_authority import CONTROL_ALLOCATION_POLICIES, ControlConfiguration
from .profile import (
    AssumptionCase,
    EvidenceConfidence,
    EvidenceValue,
    InterceptorEvidenceProfile,
    ResolutionStatus,
    ResolvedInterceptorProfile,
    ResolvedParameter,
    ValueOrigin,
)
from .thrust_curve import AbsoluteThrustCurve, DualPulseThrustProgram
from .thrust_schedule import ThrustProfilePoint, ThrustProfileSchedule, ThrustScheduleInterpolation

_ARCHETYPE_ID: Final = "generic_slender_sam_v1"
_ARCHETYPE_DEFAULTS: Final[dict[str, dict[str, tuple[object, str | None]]]] = {
    _ARCHETYPE_ID: {
        "launch_mass_kg": (350.0, "kg"),
        "propellant_fraction": (0.45, "1"),
        "length_m": (5.0, "m"),
        "body_diameter_m": (0.35, "m"),
        "propulsion_architecture": ("single_stage", None),
        "burn_time_class": ("medium", None),
        "thrust_profile_class": ("boost_sustain", None),
        "aero_archetype": ("generic_slender_supersonic", None),
        "drag_class": ("nominal", None),
        "maneuverability_class": ("moderate", None),
        "control_bandwidth_class": ("nominal", None),
        "guidance_family": ("command", None),
        "guidance_archetype": ("waypoint_pursuit", None),
        "navigation_constant": (3.0, "1"),
        "target_classes": (("air_breathing",), None),
    },
    "compact_high_agility_sam_v1": {
        "launch_mass_kg": (85.0, "kg"),
        "propellant_fraction": (0.45, "1"),
        "length_m": (3.0, "m"),
        "body_diameter_m": (0.13, "m"),
        "propulsion_architecture": ("single_stage_solid", None),
        "burn_time_class": ("short", None),
        "thrust_profile_class": ("progressive", None),
        "aero_archetype": ("compact_supersonic", None),
        "drag_class": ("nominal", None),
        "maneuverability_class": ("high", None),
        "control_bandwidth_class": ("fast", None),
        "guidance_family": ("infrared_homing_with_datalink", None),
        "guidance_archetype": ("proportional_navigation", None),
        "navigation_constant": (3.5, "1"),
        "target_classes": (("air_breathing",), None),
    },
    "medium_range_active_rf_sam_v1": {
        "launch_mass_kg": (160.0, "kg"),
        "propellant_fraction": (0.44, "1"),
        "length_m": (3.7, "m"),
        "body_diameter_m": (0.18, "m"),
        "propulsion_architecture": ("single_stage_solid", None),
        "burn_time_class": ("medium", None),
        "thrust_profile_class": ("boost_sustain", None),
        "aero_archetype": ("medium_range_supersonic", None),
        "drag_class": ("nominal", None),
        "maneuverability_class": ("moderate", None),
        "control_bandwidth_class": ("nominal", None),
        "guidance_family": ("active_radar", None),
        "guidance_archetype": ("proportional_navigation", None),
        "navigation_constant": (3.0, "1"),
        "target_classes": (("air_breathing", "cruise_missile"), None),
    },
    "high_energy_dual_pulse_interceptor_v1": {
        "launch_mass_kg": (320.0, "kg"),
        "propellant_fraction": (0.52, "1"),
        "length_m": (5.0, "m"),
        "body_diameter_m": (0.29, "m"),
        "propulsion_architecture": ("dual_pulse_solid", None),
        "burn_time_class": ("long", None),
        "thrust_profile_class": ("boost_sustain", None),
        "aero_archetype": ("high_energy_slender_supersonic", None),
        "drag_class": ("nominal", None),
        "maneuverability_class": ("high", None),
        "control_bandwidth_class": ("fast", None),
        "guidance_family": ("active_radar", None),
        "guidance_archetype": ("proportional_navigation", None),
        "navigation_constant": (4.0, "1"),
        "target_classes": (("air_breathing", "ballistic"), None),
    },
}
_CANONICAL_UNITS: Final[dict[str, str]] = {
    "launch_mass_kg": "kg",
    "burnout_mass_kg": "kg",
    "propellant_fraction": "1",
    "effective_specific_impulse_s": "s",
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
    "attitude_bandwidth_rad_s": "rad/s",
    "attitude_damping_ratio": "1",
    "max_body_rate_rad_s": "rad/s",
    "max_body_acceleration_rad_s2": "rad/s^2",
    "max_bank_angle_rad": "rad",
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
    "navigation_constant": "1",
    "normal_force_coefficient_limit": "1",
    "max_thrust_vector_angle_rad": "rad",
    "maneuver_drag_factor": "1",
}
_BURN_TIME_S: Final[dict[str, float]] = {"short": 4.0, "medium": 8.0, "long": 14.0}
_THRUST_TO_WEIGHT: Final[dict[str, float]] = {
    "regressive": 5.0,
    "neutral": 6.0,
    "boost_sustain": 7.5,
    "progressive": 8.0,
}
_THRUST_PROFILE_SHAPES: Final[dict[str, tuple[str, tuple[tuple[float, float], ...]]]] = {
    "neutral": ("linear", ((0.0, 1.0), (1.0, 1.0))),
    "regressive": ("linear", ((0.0, 1.2), (1.0, 0.8))),
    "progressive": ("linear", ((0.0, 0.8), (1.0, 1.2))),
    "boost_sustain": (
        "step_previous",
        ((0.0, 1.6), (0.2, 0.85), (1.0, 0.85)),
    ),
}
_DRAG_COEFFICIENT: Final[dict[str, float]] = {"low": 0.28, "nominal": 0.36, "high": 0.48}
_DRAG_SHAPE_BY_AERO_ARCHETYPE: Final[dict[str, tuple[tuple[float, float], ...]]] = {
    "generic_slender_supersonic": (
        (0.0, 0.82),
        (0.8, 0.90),
        (1.0, 1.35),
        (1.2, 1.20),
        (2.0, 1.00),
        (4.0, 0.88),
        (6.0, 0.84),
    ),
    "compact_supersonic": (
        (0.0, 0.90),
        (0.8, 0.98),
        (1.0, 1.45),
        (1.2, 1.25),
        (2.0, 1.00),
        (4.0, 0.90),
        (6.0, 0.86),
    ),
    "medium_range_supersonic": (
        (0.0, 0.82),
        (0.8, 0.90),
        (1.0, 1.30),
        (1.2, 1.15),
        (2.0, 1.00),
        (4.0, 0.88),
        (6.0, 0.84),
    ),
    "high_energy_slender_supersonic": (
        (0.0, 0.78),
        (0.8, 0.86),
        (1.0, 1.28),
        (1.2, 1.12),
        (2.0, 1.00),
        (4.0, 0.86),
        (6.0, 0.80),
    ),
}
_MANEUVER_LIMIT_MPS2: Final[dict[str, float]] = {"low": 4.0 * 9.80665, "moderate": 10.0 * 9.80665, "high": 20.0 * 9.80665}
_NORMAL_FORCE_COEFFICIENT_LIMIT: Final[dict[str, float]] = {"low": 3.0, "moderate": 6.0, "high": 10.0}
_THRUST_VECTOR_ANGLE_RAD: Final[dict[str, float]] = {
    "low": math.radians(5.0),
    "moderate": math.radians(10.0),
    "high": math.radians(15.0),
}
_CONTROL_CONFIGURATION_BY_ARCHETYPE: Final[dict[str, ControlConfiguration]] = {
    "generic_slender_sam_v1": "aerodynamic",
    "compact_high_agility_sam_v1": "mixed",
    "medium_range_active_rf_sam_v1": "aerodynamic",
    "high_energy_dual_pulse_interceptor_v1": "aerodynamic",
}
_GUIDANCE_TIME_CONSTANT_S: Final[dict[str, float]] = {"slow": 1.2, "nominal": 0.65, "fast": 0.3}
_ATTITUDE_BANDWIDTH_RAD_S: Final[dict[str, float]] = {"slow": 3.0, "nominal": 6.0, "fast": 10.0}
_MAX_BODY_RATE_RAD_S: Final[dict[str, float]] = {"low": 1.5, "moderate": 2.5, "high": 4.0}
_MAX_BODY_ACCELERATION_RAD_S2: Final[dict[str, float]] = {"low": 5.0, "moderate": 10.0, "high": 20.0}
_MAX_BANK_ANGLE_RAD: Final[dict[str, float]] = {
    "low": math.radians(60.0),
    "moderate": math.radians(75.0),
    "high": math.radians(85.0),
}
_CASE_FACTORS: Final[dict[AssumptionCase, Mapping[str, float]]] = {
    AssumptionCase.CONSERVATIVE: {"thrust": 0.90, "drag": 1.10, "maneuver": 0.85, "bandwidth": 1.15},
    AssumptionCase.NOMINAL: {"thrust": 1.0, "drag": 1.0, "maneuver": 1.0, "bandwidth": 1.0},
    AssumptionCase.OPTIMISTIC: {"thrust": 1.10, "drag": 0.90, "maneuver": 1.15, "bandwidth": 0.85},
}


def resolve_interceptor(
    profile: InterceptorEvidenceProfile,
    *,
    assumption_case: AssumptionCase | str | None = None,
) -> ResolvedInterceptorProfile:
    """Resolve sparse evidence into one complete and auditable model case."""

    profile = _compile_dual_pulse_thrust_program(_compile_absolute_thrust_curve(profile))
    case = profile.default_assumption_case if assumption_case is None else AssumptionCase(assumption_case)
    try:
        archetype_defaults = _ARCHETYPE_DEFAULTS[profile.surrogate_archetype_id]
    except KeyError as error:
        raise ValueError(f"unknown surrogate_archetype_id {profile.surrogate_archetype_id!r}; available archetypes: {tuple(_ARCHETYPE_DEFAULTS)!r}") from error
    values: dict[str, ResolvedParameter] = {}
    for name, (default, unit) in archetype_defaults.items():
        evidence = getattr(profile, name)
        values[name] = _provided(name, evidence) if evidence is not None else _archetype(name, default, unit, profile, case)

    for name in InterceptorEvidenceProfile.model_fields:
        evidence = getattr(profile, name)
        if isinstance(evidence, EvidenceValue) and name not in values:
            values[name] = _provided(name, evidence)

    if profile.thrust_time_curve is not None:
        architecture = _resolved_text(values, "propulsion_architecture")
        if architecture not in {"single_stage", "single_stage_solid"}:
            raise ValueError(
                "thrust_time_curve currently represents one continuous active-burn interval; "
                "dual_pulse_solid requires explicit pulse timing and allocation fields"
            )
    if profile.second_pulse_thrust_profile_schedule is not None:
        architecture = _resolved_text(values, "propulsion_architecture")
        if architecture != "dual_pulse_solid":
            raise ValueError("second_pulse_thrust_profile_schedule requires dual_pulse_solid propulsion")

    _resolve_geometry_and_mass(profile, values, case)
    drag_schedule, thrust_schedule, thrust_schedule_depends_on = _resolve_kernel_parameters(profile, values, case)
    resolution_status = _resolution_status(profile, values)
    return ResolvedInterceptorProfile(
        interceptor_id=profile.interceptor_id,
        model_id=profile.model_id,
        variant=profile.variant,
        parameter_set_version=profile.parameter_set_version,
        surrogate_archetype_id=profile.surrogate_archetype_id,
        assumption_case=case,
        calibration_reference_model_id=profile.calibration_reference_model_id,
        catalogue_interceptor_id=profile.catalogue_interceptor_id,
        variant_basis=profile.variant_basis,
        application_context=profile.application_context,
        resolution_status=resolution_status,
        required_diagnostics=profile.required_diagnostics,
        evidence_gaps=profile.evidence_gaps,
        evidence_intervals=profile.evidence_intervals,
        drag_coefficient_schedule=drag_schedule,
        thrust_profile_schedule=thrust_schedule,
        thrust_profile_schedule_depends_on=thrust_schedule_depends_on,
        second_pulse_thrust_profile_schedule=profile.second_pulse_thrust_profile_schedule,
        thrust_time_curve=profile.thrust_time_curve,
        dual_pulse_thrust_program=profile.dual_pulse_thrust_program,
        parameters=values,
        source_record_ids=profile.source_record_ids,
        model_notes=profile.model_notes,
    )
    ####


def _compile_absolute_thrust_curve(
    profile: InterceptorEvidenceProfile,
) -> InterceptorEvidenceProfile:
    """Compile one source-shaped curve into existing executable inputs."""

    curve: AbsoluteThrustCurve | None = profile.thrust_time_curve
    if curve is None:
        return profile
    if curve.origin in {"observed", "reported", "derived"}:
        derived_origin = ValueOrigin.DERIVED
    else:
        derived_origin = ValueOrigin(curve.origin)
    confidence = EvidenceConfidence(curve.confidence)
    burn_time = EvidenceValue(
        value=curve.duration_s,
        origin=derived_origin,
        unit="s",
        source_record_ids=curve.source_record_ids,
        confidence=confidence,
        method=f"last time ordinate of absolute curve {curve.curve_id}; {curve.method}",
    )
    nominal_thrust = EvidenceValue(
        value=curve.mean_thrust_n,
        origin=derived_origin,
        unit="N",
        source_record_ids=curve.source_record_ids,
        confidence=confidence,
        method=f"integrated impulse / active-burn duration from absolute curve {curve.curve_id}; {curve.method}",
    )
    return profile.model_copy(
        update={
            "burn_time_s": burn_time,
            "nominal_thrust_n": nominal_thrust,
            "thrust_profile_schedule": curve.normalized_schedule(),
        }
    )
    ####


def _compile_dual_pulse_thrust_program(
    profile: InterceptorEvidenceProfile,
) -> InterceptorEvidenceProfile:
    """Compile two source-shaped pulses into the shared dual-pulse inputs."""

    program: DualPulseThrustProgram | None = profile.dual_pulse_thrust_program
    if program is None:
        return profile
    first = program.first_pulse
    second = program.second_pulse
    curve_origin = _combined_curve_origin(first, second)
    curve_sources = tuple(
        dict.fromkeys(
            (
                *first.source_record_ids,
                *second.source_record_ids,
            )
        )
    )
    curve_confidence = _combined_curve_confidence(first, second)
    curve_method = f"compiled from dual-pulse program {program.program_id}; first={first.curve_id}; second={second.curve_id}"
    architecture_origin = _derived_curve_origin(program.origin)
    architecture = profile.propulsion_architecture or EvidenceValue(
        value="dual_pulse_solid",
        origin=architecture_origin,
        source_record_ids=program.source_record_ids,
        confidence=EvidenceConfidence(program.confidence),
        method=f"dual-pulse architecture declared by program {program.program_id}; {program.method}",
    )
    updates: dict[str, object] = {
        "propulsion_architecture": architecture,
        "burn_time_s": EvidenceValue(
            value=program.active_burn_time_s,
            origin=curve_origin,
            unit="s",
            source_record_ids=curve_sources,
            confidence=curve_confidence,
            method=f"sum of source-curve pulse durations; {curve_method}",
        ),
        "nominal_thrust_n": EvidenceValue(
            value=program.mean_active_burn_thrust_n,
            origin=curve_origin,
            unit="N",
            source_record_ids=curve_sources,
            confidence=curve_confidence,
            method=f"total two-pulse impulse / total active burn time; {curve_method}",
        ),
        "first_pulse_burn_time_s": EvidenceValue(
            value=first.duration_s,
            origin=_derived_curve_origin(first.origin),
            unit="s",
            source_record_ids=first.source_record_ids,
            confidence=EvidenceConfidence(first.confidence),
            method=f"last time ordinate of first-pulse curve {first.curve_id}; {first.method}",
        ),
        "inter_pulse_coast_time_s": EvidenceValue(
            value=program.inter_pulse_coast_time_s,
            origin=_derived_curve_origin(program.origin),
            unit="s",
            source_record_ids=program.source_record_ids,
            confidence=EvidenceConfidence(program.confidence),
            method=f"coast duration from dual-pulse program {program.program_id}; {program.method}",
        ),
        "second_pulse_burn_time_s": EvidenceValue(
            value=second.duration_s,
            origin=_derived_curve_origin(second.origin),
            unit="s",
            source_record_ids=second.source_record_ids,
            confidence=EvidenceConfidence(second.confidence),
            method=f"last time ordinate of second-pulse curve {second.curve_id}; {second.method}",
        ),
        "second_pulse_thrust_ratio": EvidenceValue(
            value=program.second_to_first_mean_thrust_ratio,
            origin=curve_origin,
            unit="1",
            source_record_ids=curve_sources,
            confidence=curve_confidence,
            method=f"second-pulse mean / first-pulse mean; {curve_method}",
        ),
        "thrust_profile_schedule": first.normalized_schedule(),
        "second_pulse_thrust_profile_schedule": second.normalized_schedule(),
    }
    return profile.model_copy(update=updates)
    ####


def _derived_curve_origin(origin: str) -> ValueOrigin:
    if origin in {"observed", "reported", "derived"}:
        return ValueOrigin.DERIVED
    return ValueOrigin(origin)
    ####


def _combined_curve_origin(
    first: AbsoluteThrustCurve,
    second: AbsoluteThrustCurve,
) -> ValueOrigin:
    origins = {_derived_curve_origin(first.origin), _derived_curve_origin(second.origin)}
    for origin in (
        ValueOrigin.CALIBRATED,
        ValueOrigin.ARCHETYPE_ASSUMPTION,
        ValueOrigin.SIMULATION_ASSUMPTION,
        ValueOrigin.INFERRED,
        ValueOrigin.DERIVED,
    ):
        if origin in origins:
            return origin
    return ValueOrigin.DERIVED
    ####


def _combined_curve_confidence(
    first: AbsoluteThrustCurve,
    second: AbsoluteThrustCurve,
) -> EvidenceConfidence:
    ranking = {
        EvidenceConfidence.UNKNOWN: 0,
        EvidenceConfidence.LOW: 1,
        EvidenceConfidence.MEDIUM: 2,
        EvidenceConfidence.HIGH: 3,
    }
    return min(
        (EvidenceConfidence(first.confidence), EvidenceConfidence(second.confidence)),
        key=ranking.__getitem__,
    )
    ####


def _resolve_geometry_and_mass(
    profile: InterceptorEvidenceProfile,
    values: dict[str, ResolvedParameter],
    case: AssumptionCase,
) -> None:
    diameter = _resolved_number(values, "body_diameter_m")
    launch_mass = _resolved_number(values, "launch_mass_kg")
    if profile.reference_area_m2 is not None:
        values["reference_area_m2"] = _provided("reference_area_m2", profile.reference_area_m2)
    else:
        values["reference_area_m2"] = _dependent(
            math.pi * diameter**2 / 4.0,
            values=values,
            profile=profile,
            unit="m^2",
            method="circular body frontal area: pi * body_diameter_m^2 / 4",
            depends_on=("body_diameter_m",),
            case=case,
        )
    if profile.reference_length_m is not None:
        values["reference_length_m"] = _provided("reference_length_m", profile.reference_length_m)
    else:
        values["reference_length_m"] = _dependent(
            _resolved_number(values, "length_m"),
            values=values,
            profile=profile,
            unit="m",
            method="reference length equals overall length for generic slender archetype",
            depends_on=("length_m",),
            case=case,
        )
    if profile.slenderness_ratio is not None:
        values["slenderness_ratio"] = _provided("slenderness_ratio", profile.slenderness_ratio)
    else:
        values["slenderness_ratio"] = _dependent(
            _resolved_number(values, "length_m") / diameter,
            values=values,
            profile=profile,
            unit="1",
            method="length_m / body_diameter_m",
            depends_on=("length_m", "body_diameter_m"),
            case=case,
        )
    if profile.effective_specific_impulse_s is not None:
        values.pop("propellant_fraction", None)
    else:
        propellant_fraction = _resolved_number(values, "propellant_fraction")
        if profile.burnout_mass_kg is not None:
            values["burnout_mass_kg"] = _provided("burnout_mass_kg", profile.burnout_mass_kg)
        else:
            values["burnout_mass_kg"] = _dependent(
                launch_mass * (1.0 - propellant_fraction),
                values=values,
                profile=profile,
                unit="kg",
                method="launch_mass_kg * (1 - propellant_fraction)",
                depends_on=("launch_mass_kg", "propellant_fraction"),
                case=case,
            )
        if _resolved_number(values, "burnout_mass_kg") >= launch_mass:
            raise ValueError("resolved burnout mass must be below launch mass")
    ####


def _resolve_kernel_parameters(
    profile: InterceptorEvidenceProfile,
    values: dict[str, ResolvedParameter],
    case: AssumptionCase,
) -> tuple[DragCoefficientSchedule, ThrustProfileSchedule, tuple[str, ...]]:
    factors = _CASE_FACTORS[case]
    burn_class = _resolved_text(values, "burn_time_class")
    thrust_class = _resolved_text(values, "thrust_profile_class")
    maneuver_class = _resolved_text(values, "maneuverability_class")
    bandwidth_class = _resolved_text(values, "control_bandwidth_class")
    guidance_archetype = _resolved_text(values, "guidance_archetype")
    if guidance_archetype not in {"waypoint_pursuit", "proportional_navigation"}:
        raise ValueError(f"unsupported guidance_archetype {guidance_archetype!r}; choose one of ('waypoint_pursuit', 'proportional_navigation')")
    for name in (
        "thrust_scale",
        "drag_scale",
        "maneuverability_scale",
        "guidance_time_constant_scale",
    ):
        if getattr(profile, name) is None:
            values[name] = _program_archetype_parameter(
                1.0,
                unit="1",
                profile=profile,
                case=case,
                method=f"neutral default for calibration scale {name}",
                depends_on=(),
            )
    if profile.maneuver_drag_factor is None:
        values["maneuver_drag_factor"] = _program_archetype_parameter(
            0.1,
            unit="1",
            profile=profile,
            case=case,
            method="neutral reduced-order Cdm = k * Cn^2 maneuver-drag factor",
            depends_on=(),
        )
    _resolve_control_authority_parameters(profile, values, case)
    if profile.burn_time_s is None:
        values["burn_time_s"] = _derived(
            _class_value(_BURN_TIME_S, burn_class, "burn_time_class"),
            unit="s",
            method=f"versioned burn-time class map {profile.surrogate_archetype_id}; {case.value} case factors applied",
            depends_on=("burn_time_class",),
            case=case,
            origin=ValueOrigin.ARCHETYPE_ASSUMPTION,
            archetype_id=profile.surrogate_archetype_id,
        )
    if profile.thrust_time_curve is not None or profile.dual_pulse_thrust_program is not None:
        values.pop("burn_time_class", None)
    _resolve_propulsion_program(profile, values, case)
    burn_time_s = _resolved_number(values, "burn_time_s")
    launch_mass = _resolved_number(values, "launch_mass_kg")
    if profile.nominal_thrust_n is None:
        values["nominal_thrust_n"] = _dependent(
            launch_mass * 9.80665 * _class_value(_THRUST_TO_WEIGHT, thrust_class, "thrust_profile_class"),
            values=values,
            profile=profile,
            unit="N",
            method="launch mass * standard gravity * backward-compatible archetype thrust-to-weight",
            depends_on=("launch_mass_kg", "thrust_profile_class"),
            case=case,
        )
    _resolve_specific_impulse_mass(profile, values, case)
    thrust_schedule, thrust_schedule_depends_on = _resolve_thrust_profile_schedule(profile, thrust_class)
    if profile.nominal_thrust_n is not None and profile.thrust_profile_schedule is not None:
        values.pop("thrust_profile_class", None)
    drag_schedule = _resolve_drag_schedule(profile, values, case)
    maneuver_limit = _class_value(_MANEUVER_LIMIT_MPS2, maneuver_class, "maneuverability_class") * factors["maneuver"]
    guidance_time_constant = (
        _class_value(_GUIDANCE_TIME_CONSTANT_S, bandwidth_class, "control_bandwidth_class")
        * factors["bandwidth"]
        * _resolved_number(values, "guidance_time_constant_scale")
    )
    attitude_bandwidth = _class_value(_ATTITUDE_BANDWIDTH_RAD_S, bandwidth_class, "control_bandwidth_class") / factors["bandwidth"]
    burnout_mass = _resolved_number(values, "burnout_mass_kg")
    method_suffix = f"; {case.value} case factors applied"
    values.update(
        {
            "thrust_n": _resolve_executable_thrust(profile, values, case),
            "max_lateral_acceleration_mps2": _dependent(
                maneuver_limit,
                values=values,
                profile=profile,
                unit="m/s^2",
                method=f"versioned maneuverability-class map {profile.surrogate_archetype_id}{method_suffix}",
                depends_on=("maneuverability_class",),
                case=case,
            ),
            "guidance_time_constant_s": _dependent(
                guidance_time_constant,
                values=values,
                profile=profile,
                unit="s",
                method=f"versioned control-bandwidth map {profile.surrogate_archetype_id}{method_suffix}",
                depends_on=("control_bandwidth_class", "guidance_time_constant_scale"),
                case=case,
            ),
        }
    )
    values["mass_flow_kg_s"] = _dependent(
        (launch_mass - burnout_mass) / burn_time_s,
        values=values,
        profile=profile,
        unit="kg/s",
        method="(launch_mass_kg - burnout_mass_kg) / burn_time_s",
        depends_on=("launch_mass_kg", "burnout_mass_kg", "burn_time_s"),
        case=case,
    )
    values["active_burn_total_impulse_n_s"] = _dependent(
        _resolved_number(values, "thrust_n") * burn_time_s,
        values=values,
        profile=profile,
        unit="N*s",
        method="executable mean active-burn thrust * total active burn time",
        depends_on=("thrust_n", "burn_time_s"),
        case=case,
    )
    response_defaults = {
        "attitude_bandwidth_rad_s": (
            attitude_bandwidth,
            "rad/s",
            "control_bandwidth_class",
            "versioned attitude-bandwidth class map",
        ),
        "attitude_damping_ratio": (0.9, "1", "control_bandwidth_class", "generic damped response-law default"),
        "max_body_rate_rad_s": (
            _class_value(_MAX_BODY_RATE_RAD_S, maneuver_class, "maneuverability_class") * factors["maneuver"],
            "rad/s",
            "maneuverability_class",
            "versioned body-rate class map",
        ),
        "max_body_acceleration_rad_s2": (
            _class_value(_MAX_BODY_ACCELERATION_RAD_S2, maneuver_class, "maneuverability_class") * factors["maneuver"],
            "rad/s^2",
            "maneuverability_class",
            "versioned angular-acceleration class map",
        ),
        "max_bank_angle_rad": (
            _class_value(_MAX_BANK_ANGLE_RAD, maneuver_class, "maneuverability_class"),
            "rad",
            "maneuverability_class",
            "versioned bank-authority class map",
        ),
    }
    for name, (default, unit, dependency, method) in response_defaults.items():
        evidence = getattr(profile, name)
        values[name] = (
            _provided(name, evidence)
            if evidence is not None
            else _derived(
                default,
                unit=unit,
                method=f"{method} {profile.surrogate_archetype_id}{method_suffix}",
                depends_on=(dependency,),
                case=case,
                origin=ValueOrigin.ARCHETYPE_ASSUMPTION,
                archetype_id=profile.surrogate_archetype_id,
            )
        )
    values["drag_coefficient"] = ResolvedParameter(
        value=drag_schedule.coefficient_at(2.0),
        origin=ValueOrigin(drag_schedule.origin),
        unit="1",
        source_record_ids=drag_schedule.source_record_ids,
        confidence=EvidenceConfidence(drag_schedule.confidence),
        method=f"reference Mach 2 projection of {drag_schedule.schedule_id}; {drag_schedule.method}",
        archetype_id=(profile.surrogate_archetype_id if drag_schedule.origin == ValueOrigin.ARCHETYPE_ASSUMPTION.value else None),
        assumption_case=case,
        depends_on=("drag_coefficient_schedule", "drag_scale"),
    )
    return drag_schedule, thrust_schedule, thrust_schedule_depends_on
    ####


def _resolve_specific_impulse_mass(
    profile: InterceptorEvidenceProfile,
    values: dict[str, ResolvedParameter],
    case: AssumptionCase,
) -> None:
    """Lower an explicit effective Isp into mass and pulse allocation.

    The calculation uses the unscaled nominal active-burn impulse. Assumption-
    case and fitted thrust multipliers therefore do not silently change the
    vehicle's resolved wet or dry mass.
    """

    if profile.effective_specific_impulse_s is None:
        return
    standard_gravity_mps2 = 9.80665
    launch_mass = _resolved_number(values, "launch_mass_kg")
    specific_impulse = _resolved_number(values, "effective_specific_impulse_s")
    burn_time = _resolved_number(values, "burn_time_s")
    nominal_thrust = _resolved_number(values, "nominal_thrust_n")
    nominal_impulse = nominal_thrust * burn_time
    propellant_mass = nominal_impulse / (specific_impulse * standard_gravity_mps2)
    if propellant_mass <= 0.0 or propellant_mass >= launch_mass:
        raise ValueError("effective specific impulse and nominal active-burn impulse must imply propellant mass strictly between zero and launch mass")
    common_dependencies = (
        "launch_mass_kg",
        "effective_specific_impulse_s",
        "nominal_thrust_n",
        "burn_time_s",
    )
    values["propellant_fraction"] = _dependent_with_source_lineage(
        propellant_mass / launch_mass,
        values=values,
        profile=profile,
        unit="1",
        method=("nominal_thrust_n * burn_time_s / (effective_specific_impulse_s * standard_gravity * launch_mass_kg)"),
        depends_on=common_dependencies,
        case=case,
    )
    values["burnout_mass_kg"] = _dependent_with_source_lineage(
        launch_mass - propellant_mass,
        values=values,
        profile=profile,
        unit="kg",
        method=("launch_mass_kg - nominal_thrust_n * burn_time_s / (effective_specific_impulse_s * standard_gravity)"),
        depends_on=common_dependencies,
        case=case,
    )
    if _resolved_text(values, "propulsion_architecture") == "dual_pulse_solid":
        first_duration = _resolved_number(values, "first_pulse_burn_time_s")
        second_duration = _resolved_number(values, "second_pulse_burn_time_s")
        second_ratio = _resolved_number(values, "second_pulse_thrust_ratio")
        second_impulse_weight = second_duration * second_ratio
        values["second_pulse_propellant_fraction"] = _dependent_with_source_lineage(
            second_impulse_weight / (first_duration + second_impulse_weight),
            values=values,
            profile=profile,
            unit="1",
            method=(
                "second-pulse nominal impulse share under one common effective_specific_impulse_s: "
                "second_pulse_burn_time_s * second_pulse_thrust_ratio / "
                "(first_pulse_burn_time_s + second_pulse_burn_time_s * second_pulse_thrust_ratio)"
            ),
            depends_on=(
                "effective_specific_impulse_s",
                "first_pulse_burn_time_s",
                "second_pulse_burn_time_s",
                "second_pulse_thrust_ratio",
            ),
            case=case,
        )
    ####


def _resolve_control_authority_parameters(
    profile: InterceptorEvidenceProfile,
    values: dict[str, ResolvedParameter],
    case: AssumptionCase,
) -> None:
    maneuver_class = _resolved_text(values, "maneuverability_class")
    maneuver_scale = _resolved_number(values, "maneuverability_scale")
    case_factor = _CASE_FACTORS[case]["maneuver"]
    features = _resolved_text_tuple(values, "control_features") if "control_features" in values else ()

    if "control_allocation_policy" not in values:
        values["control_allocation_policy"] = ResolvedParameter(
            value="aerodynamic_first",
            origin=ValueOrigin.ARCHETYPE_ASSUMPTION,
            confidence=EvidenceConfidence.LOW,
            method="default reduced-order lateral-force allocation policy",
            archetype_id=profile.surrogate_archetype_id,
            assumption_case=case,
            depends_on=("surrogate_archetype_id",),
        )
    allocation_policy_text = _resolved_text(values, "control_allocation_policy")
    if allocation_policy_text not in CONTROL_ALLOCATION_POLICIES:
        raise ValueError(f"unsupported control_allocation_policy {allocation_policy_text!r}; choose one of {CONTROL_ALLOCATION_POLICIES!r}")
    if "control_configuration" not in values:
        if "thrust_vectoring" in features:
            feature = values["control_features"]
            values["control_configuration"] = ResolvedParameter(
                value="mixed",
                origin=ValueOrigin.INFERRED,
                confidence=feature.confidence,
                method="mixed reduced-order authority inferred from declared thrust_vectoring control feature",
                source_record_ids=feature.source_record_ids,
                depends_on=("control_features",),
            )
        else:
            archetype_configuration = _CONTROL_CONFIGURATION_BY_ARCHETYPE[profile.surrogate_archetype_id]
            values["control_configuration"] = ResolvedParameter(
                value=archetype_configuration,
                origin=ValueOrigin.ARCHETYPE_ASSUMPTION,
                confidence=EvidenceConfidence.LOW,
                method="versioned control-configuration archetype map",
                archetype_id=profile.surrogate_archetype_id,
                assumption_case=case,
                depends_on=("surrogate_archetype_id",),
            )
    configuration_text = _resolved_text(values, "control_configuration")
    if configuration_text not in {"aerodynamic", "thrust_assisted", "mixed"}:
        raise ValueError(f"unsupported control_configuration {configuration_text!r}; choose one of ('aerodynamic', 'thrust_assisted', 'mixed')")
    configuration = cast(ControlConfiguration, configuration_text)

    if configuration == "thrust_assisted":
        if profile.normal_force_coefficient_limit is not None:
            raise ValueError("thrust_assisted control cannot define normal_force_coefficient_limit")
        values["normal_force_coefficient_limit"] = _derived(
            0.0,
            unit="1",
            method="pure thrust-assisted configuration disables aerodynamic authority",
            depends_on=("control_configuration",),
            case=case,
        )
    else:
        _resolve_normal_force_coefficient(
            profile,
            values,
            case,
            maneuver_class=maneuver_class,
            scale=maneuver_scale * case_factor,
        )

    if configuration == "aerodynamic":
        if profile.max_thrust_vector_angle_rad is not None:
            raise ValueError("aerodynamic control cannot define max_thrust_vector_angle_rad")
        values["max_thrust_vector_angle_rad"] = _derived(
            0.0,
            unit="rad",
            method="aerodynamic configuration disables thrust-vector authority",
            depends_on=("control_configuration",),
            case=case,
        )
    else:
        if profile.max_thrust_vector_angle_rad is None:
            values["max_thrust_vector_angle_rad"] = _derived(
                _class_value(_THRUST_VECTOR_ANGLE_RAD, maneuver_class, "maneuverability_class") * case_factor,
                unit="rad",
                method=f"versioned thrust-vector-angle class map {profile.surrogate_archetype_id}; {case.value} case factor applied",
                depends_on=("maneuverability_class", "control_configuration"),
                case=case,
                origin=ValueOrigin.ARCHETYPE_ASSUMPTION,
                archetype_id=profile.surrogate_archetype_id,
            )
    ####


def _resolve_normal_force_coefficient(
    profile: InterceptorEvidenceProfile,
    values: dict[str, ResolvedParameter],
    case: AssumptionCase,
    *,
    maneuver_class: str,
    scale: float,
) -> None:
    evidence = profile.normal_force_coefficient_limit
    if evidence is None:
        values["normal_force_coefficient_limit"] = _derived(
            _class_value(_NORMAL_FORCE_COEFFICIENT_LIMIT, maneuver_class, "maneuverability_class") * scale,
            unit="1",
            method=f"versioned normal-force authority class map {profile.surrogate_archetype_id}; {case.value} case and maneuverability_scale applied",
            depends_on=("maneuverability_class", "maneuverability_scale"),
            case=case,
            origin=(ValueOrigin.CALIBRATED if values["maneuverability_scale"].origin is ValueOrigin.CALIBRATED else ValueOrigin.ARCHETYPE_ASSUMPTION),
            archetype_id=profile.surrogate_archetype_id,
        )
        return
    base_value = _resolved_number(values, "normal_force_coefficient_limit")
    if math.isclose(scale, 1.0, rel_tol=0.0, abs_tol=1.0e-15):
        return
    scale_origin = values["maneuverability_scale"].origin
    values["normal_force_coefficient_limit"] = ResolvedParameter(
        value=base_value * scale,
        origin=(ValueOrigin.CALIBRATED if scale_origin is ValueOrigin.CALIBRATED else ValueOrigin.SIMULATION_ASSUMPTION),
        unit="1",
        source_record_ids=evidence.source_record_ids,
        confidence=(EvidenceConfidence.MEDIUM if scale_origin is ValueOrigin.CALIBRATED else EvidenceConfidence.LOW),
        method=f"authored coefficient × {case.value} case factor × maneuverability_scale; source value remains unchanged",
        assumption_case=case,
        depends_on=("maneuverability_scale",),
    )
    ####


def _resolve_executable_thrust(
    profile: InterceptorEvidenceProfile,
    values: Mapping[str, ResolvedParameter],
    case: AssumptionCase,
) -> ResolvedParameter:
    """Apply case and calibration scaling without overwriting source thrust."""

    nominal = values["nominal_thrust_n"]
    nominal_value = _resolved_number(values, "nominal_thrust_n")
    scale = _resolved_number(values, "thrust_scale")
    case_factor = _CASE_FACTORS[case]["thrust"]
    explicit_scale = profile.thrust_scale is not None
    nonnominal_case = case is not AssumptionCase.NOMINAL
    scale_origin = values["thrust_scale"].origin

    if scale_origin is ValueOrigin.CALIBRATED or nominal.origin is ValueOrigin.CALIBRATED:
        origin = ValueOrigin.CALIBRATED
        confidence = EvidenceConfidence.MEDIUM
    elif explicit_scale:
        origin = ValueOrigin.SIMULATION_ASSUMPTION
        confidence = EvidenceConfidence.LOW
    elif nominal.origin is ValueOrigin.ARCHETYPE_ASSUMPTION:
        origin = ValueOrigin.ARCHETYPE_ASSUMPTION
        confidence = EvidenceConfidence.LOW
    elif nominal.origin is ValueOrigin.SIMULATION_ASSUMPTION:
        origin = ValueOrigin.SIMULATION_ASSUMPTION
        confidence = EvidenceConfidence.LOW
    elif nonnominal_case:
        origin = ValueOrigin.SIMULATION_ASSUMPTION
        confidence = EvidenceConfidence.LOW
    elif nominal.origin in {ValueOrigin.OBSERVED, ValueOrigin.REPORTED, ValueOrigin.DERIVED}:
        origin = ValueOrigin.DERIVED
        confidence = nominal.confidence
    else:
        origin = nominal.origin
        confidence = nominal.confidence

    return ResolvedParameter(
        value=nominal_value * case_factor * scale,
        origin=origin,
        unit="N",
        source_record_ids=nominal.source_record_ids,
        confidence=confidence,
        method=("nominal_thrust_n * assumption-case thrust factor * thrust_scale; the evidence-bearing nominal value remains unchanged"),
        archetype_id=profile.surrogate_archetype_id if origin is ValueOrigin.ARCHETYPE_ASSUMPTION else None,
        assumption_case=case,
        depends_on=("nominal_thrust_n", "thrust_scale"),
    )
    ####


def _resolve_drag_schedule(
    profile: InterceptorEvidenceProfile,
    values: dict[str, ResolvedParameter],
    case: AssumptionCase,
) -> DragCoefficientSchedule:
    drag_scale = _resolved_number(values, "drag_scale")
    case_factor = _CASE_FACTORS[case]["drag"]
    scale = drag_scale * case_factor
    authored = profile.drag_coefficient_schedule
    if authored is None:
        aero_archetype = _resolved_text(values, "aero_archetype")
        drag_class = _resolved_text(values, "drag_class")
        try:
            normalized_shape = _DRAG_SHAPE_BY_AERO_ARCHETYPE[aero_archetype]
        except KeyError as error:
            raise ValueError(f"unsupported aero_archetype {aero_archetype!r}; choose one of {tuple(_DRAG_SHAPE_BY_AERO_ARCHETYPE)!r}") from error
        reference_coefficient = _class_value(_DRAG_COEFFICIENT, drag_class, "drag_class")
        return DragCoefficientSchedule(
            schedule_id=f"{aero_archetype}-drag-v1",
            points=tuple(
                DragCoefficientPoint(
                    mach=mach,
                    coefficient=reference_coefficient * multiplier * scale,
                )
                for mach, multiplier in normalized_shape
            ),
            origin="archetype_assumption",
            confidence="low",
            method=(f"versioned {aero_archetype} normalized Mach shape × {drag_class} drag-class amplitude × {case.value} case factor × drag_scale"),
        )

    values.pop("aero_archetype", None)
    values.pop("drag_class", None)
    source_origin = ValueOrigin(authored.origin)
    scale_origin = values["drag_scale"].origin
    if math.isclose(scale, 1.0, rel_tol=0.0, abs_tol=1.0e-15):
        resolved_origin = source_origin
        resolved_confidence = authored.confidence
    elif scale_origin is ValueOrigin.CALIBRATED:
        resolved_origin = ValueOrigin.CALIBRATED
        resolved_confidence = EvidenceConfidence.MEDIUM.value
    else:
        resolved_origin = ValueOrigin.SIMULATION_ASSUMPTION
        resolved_confidence = EvidenceConfidence.LOW.value
    return DragCoefficientSchedule(
        schedule_id=authored.schedule_id,
        points=tuple(
            DragCoefficientPoint(
                mach=item.mach,
                coefficient=item.coefficient * scale,
            )
            for item in authored.points
        ),
        origin=resolved_origin.value,
        source_record_ids=authored.source_record_ids,
        confidence=resolved_confidence,
        method=(f"{authored.method}; resolved coefficients multiply authored values by {case.value} case factor and drag_scale"),
    )
    ####


def _resolve_thrust_profile_schedule(
    profile: InterceptorEvidenceProfile,
    thrust_profile_class: str,
) -> tuple[ThrustProfileSchedule, tuple[str, ...]]:
    """Resolve a coarse class or accept one explicit unit-mean pulse shape."""

    authored = profile.thrust_profile_schedule
    if authored is not None:
        return authored, ()
    try:
        interpolation, points = _THRUST_PROFILE_SHAPES[thrust_profile_class]
    except KeyError as error:
        raise ValueError(f"unsupported thrust_profile_class {thrust_profile_class!r}; choose one of {tuple(_THRUST_PROFILE_SHAPES)!r}") from error
    return (
        ThrustProfileSchedule(
            schedule_id=f"{thrust_profile_class}-thrust-profile-v1",
            points=tuple(ThrustProfilePoint(burn_fraction=burn_fraction, multiplier=multiplier) for burn_fraction, multiplier in points),
            interpolation=cast(ThrustScheduleInterpolation, interpolation),
            origin="archetype_assumption",
            confidence="low",
            method=f"versioned {thrust_profile_class} unit-mean active-burn shape",
        ),
        ("thrust_profile_class",),
    )
    ####


def _resolve_propulsion_program(
    profile: InterceptorEvidenceProfile,
    values: dict[str, ResolvedParameter],
    case: AssumptionCase,
) -> None:
    """Resolve one- or two-pulse timing without hiding archetype choices."""

    architecture = _resolved_text(values, "propulsion_architecture")
    if architecture in {"single_stage", "single_stage_solid"}:
        _resolve_single_pulse(profile, values, case)
        return
    if architecture == "dual_pulse_solid":
        _resolve_dual_pulse(profile, values, case)
        return
    raise ValueError(
        f"unsupported propulsion_architecture {architecture!r}; the executable surrogate supports single_stage, single_stage_solid, and dual_pulse_solid"
    )
    ####


def _resolve_single_pulse(
    profile: InterceptorEvidenceProfile,
    values: dict[str, ResolvedParameter],
    case: AssumptionCase,
) -> None:
    pulse_two_fields = (
        "inter_pulse_coast_time_s",
        "second_pulse_burn_time_s",
        "second_pulse_thrust_ratio",
        "second_pulse_propellant_fraction",
    )
    supplied = tuple(name for name in pulse_two_fields if getattr(profile, name) is not None)
    if supplied:
        raise ValueError(f"single-pulse propulsion cannot define second-pulse fields: {supplied!r}")

    if profile.first_pulse_burn_time_s is not None:
        first = _resolved_number(values, "first_pulse_burn_time_s")
        if profile.burn_time_s is not None and not math.isclose(first, _resolved_number(values, "burn_time_s"), rel_tol=1.0e-9):
            raise ValueError("single-pulse first_pulse_burn_time_s must equal burn_time_s")
        if profile.burn_time_s is None:
            values["burn_time_s"] = _dependent(
                first,
                values=values,
                profile=profile,
                unit="s",
                method="single-pulse active burn time equals first_pulse_burn_time_s",
                depends_on=("first_pulse_burn_time_s",),
                case=case,
            )
    else:
        values["first_pulse_burn_time_s"] = _dependent(
            _resolved_number(values, "burn_time_s"),
            values=values,
            profile=profile,
            unit="s",
            method="single-pulse duration equals total active burn time",
            depends_on=("burn_time_s", "propulsion_architecture"),
            case=case,
        )
    _set_inactive_second_pulse(values, profile, case)
    ####


def _set_inactive_second_pulse(
    values: dict[str, ResolvedParameter],
    profile: InterceptorEvidenceProfile,
    case: AssumptionCase,
) -> None:
    for name, unit in (
        ("inter_pulse_coast_time_s", "s"),
        ("second_pulse_burn_time_s", "s"),
        ("second_pulse_thrust_ratio", "1"),
        ("second_pulse_propellant_fraction", "1"),
    ):
        values[name] = _dependent(
            0.0,
            values=values,
            profile=profile,
            unit=unit,
            method=f"{name} is zero for single-pulse propulsion",
            depends_on=("propulsion_architecture",),
            case=case,
        )
    ####


def _resolve_dual_pulse(
    profile: InterceptorEvidenceProfile,
    values: dict[str, ResolvedParameter],
    case: AssumptionCase,
) -> None:
    first_supplied = profile.first_pulse_burn_time_s is not None
    second_supplied = profile.second_pulse_burn_time_s is not None
    total_supplied = profile.burn_time_s is not None
    total = _resolved_number(values, "burn_time_s")

    if first_supplied and second_supplied:
        first = _resolved_number(values, "first_pulse_burn_time_s")
        second = _resolved_number(values, "second_pulse_burn_time_s")
        if total_supplied and not math.isclose(first + second, total, rel_tol=1.0e-9):
            raise ValueError("dual-pulse burn_time_s must equal the sum of first and second pulse burn times")
        if not total_supplied:
            values["burn_time_s"] = _dependent(
                first + second,
                values=values,
                profile=profile,
                unit="s",
                method="sum of developer-supplied dual-pulse active durations",
                depends_on=("first_pulse_burn_time_s", "second_pulse_burn_time_s"),
                case=case,
            )
            total = first + second
    elif first_supplied:
        first = _resolved_number(values, "first_pulse_burn_time_s")
        second = total - first
        _require_positive_remainder(second)
        values["second_pulse_burn_time_s"] = _dependent(
            second,
            values=values,
            profile=profile,
            unit="s",
            method="total active burn time minus supplied first-pulse duration",
            depends_on=("burn_time_s", "first_pulse_burn_time_s"),
            case=case,
        )
    elif second_supplied:
        second = _resolved_number(values, "second_pulse_burn_time_s")
        first = total - second
        _require_positive_remainder(first)
        values["first_pulse_burn_time_s"] = _dependent(
            first,
            values=values,
            profile=profile,
            unit="s",
            method="total active burn time minus supplied second-pulse duration",
            depends_on=("burn_time_s", "second_pulse_burn_time_s"),
            case=case,
        )
    else:
        values["first_pulse_burn_time_s"] = _program_archetype_parameter(
            total * 0.45,
            unit="s",
            profile=profile,
            case=case,
            method="dual-pulse archetype assigns 45 percent of active burn to pulse one",
            depends_on=("burn_time_s", "propulsion_architecture"),
        )
        values["second_pulse_burn_time_s"] = _program_archetype_parameter(
            total * 0.55,
            unit="s",
            profile=profile,
            case=case,
            method="dual-pulse archetype assigns 55 percent of active burn to pulse two",
            depends_on=("burn_time_s", "propulsion_architecture"),
        )

    defaults = {
        "inter_pulse_coast_time_s": (4.0, "s", "dual-pulse archetype inter-pulse coast duration"),
        "second_pulse_thrust_ratio": (0.8, "1", "dual-pulse archetype second-to-first pulse thrust ratio"),
        "second_pulse_propellant_fraction": (0.5, "1", "dual-pulse archetype propellant allocation to pulse two"),
    }
    for name, (default, unit, method) in defaults.items():
        if getattr(profile, name) is None:
            values[name] = _program_archetype_parameter(
                default,
                unit=unit,
                profile=profile,
                case=case,
                method=method,
                depends_on=("propulsion_architecture",),
            )
    ####


def _program_archetype_parameter(
    value: float,
    *,
    unit: str,
    profile: InterceptorEvidenceProfile,
    case: AssumptionCase,
    method: str,
    depends_on: tuple[str, ...],
) -> ResolvedParameter:
    return _derived(
        value,
        unit=unit,
        method=f"{method} {profile.surrogate_archetype_id}",
        depends_on=depends_on,
        case=case,
        origin=ValueOrigin.ARCHETYPE_ASSUMPTION,
        archetype_id=profile.surrogate_archetype_id,
    )
    ####


def _require_positive_remainder(value: float) -> None:
    if value <= 0.0:
        raise ValueError("each dual-pulse burn duration must be shorter than total burn_time_s")
    ####


def _provided(name: str, evidence: EvidenceValue) -> ResolvedParameter:
    canonical = _CANONICAL_UNITS.get(name)
    if canonical is not None and evidence.unit not in {None, canonical}:
        raise ValueError(f"{name} must use canonical unit {canonical!r}, got {evidence.unit!r}")
    return ResolvedParameter(
        value=evidence.value,
        origin=evidence.origin,
        unit=canonical or evidence.unit,
        source_record_ids=evidence.source_record_ids,
        confidence=evidence.confidence,
        method=evidence.method or "developer-supplied profile value",
        source_value=evidence.source_value,
    )
    ####


def _archetype(
    name: str,
    value: object,
    unit: str | None,
    profile: InterceptorEvidenceProfile,
    case: AssumptionCase,
) -> ResolvedParameter:
    if not isinstance(value, float | int | str | bool | tuple):
        raise TypeError(f"invalid archetype value for {name!r}")
    return ResolvedParameter(
        value=value,
        origin=ValueOrigin.ARCHETYPE_ASSUMPTION,
        unit=unit,
        confidence=EvidenceConfidence.LOW,
        method=f"missing value filled by versioned archetype field {name}",
        archetype_id=profile.surrogate_archetype_id,
        assumption_case=case,
    )
    ####


def _derived(
    value: float,
    *,
    unit: str,
    method: str,
    depends_on: tuple[str, ...],
    case: AssumptionCase,
    origin: ValueOrigin = ValueOrigin.DERIVED,
    archetype_id: str | None = None,
) -> ResolvedParameter:
    return ResolvedParameter(
        value=value,
        origin=origin,
        unit=unit,
        confidence=(
            EvidenceConfidence.LOW
            if origin in {ValueOrigin.ARCHETYPE_ASSUMPTION, ValueOrigin.SIMULATION_ASSUMPTION, ValueOrigin.INFERRED}
            else EvidenceConfidence.MEDIUM
        ),
        method=method,
        archetype_id=archetype_id,
        assumption_case=case,
        depends_on=depends_on,
    )
    ####


def _dependent(
    value: float,
    *,
    values: Mapping[str, ResolvedParameter],
    profile: InterceptorEvidenceProfile,
    unit: str,
    method: str,
    depends_on: tuple[str, ...],
    case: AssumptionCase,
) -> ResolvedParameter:
    """Derive a value while conservatively propagating assumption lineage."""

    origins = {values[name].origin for name in depends_on if name in values}
    if ValueOrigin.CALIBRATED in origins:
        origin = ValueOrigin.CALIBRATED
    elif ValueOrigin.ARCHETYPE_ASSUMPTION in origins:
        origin = ValueOrigin.ARCHETYPE_ASSUMPTION
    elif ValueOrigin.SIMULATION_ASSUMPTION in origins:
        origin = ValueOrigin.SIMULATION_ASSUMPTION
    elif ValueOrigin.INFERRED in origins:
        origin = ValueOrigin.INFERRED
    else:
        origin = ValueOrigin.DERIVED
    return _derived(
        value,
        unit=unit,
        method=method,
        depends_on=depends_on,
        case=case,
        origin=origin,
        archetype_id=profile.surrogate_archetype_id if origin is ValueOrigin.ARCHETYPE_ASSUMPTION else None,
    )
    ####


def _dependent_with_source_lineage(
    value: float,
    *,
    values: Mapping[str, ResolvedParameter],
    profile: InterceptorEvidenceProfile,
    unit: str,
    method: str,
    depends_on: tuple[str, ...],
    case: AssumptionCase,
) -> ResolvedParameter:
    """Derive a value and retain the source records of its exact operands."""

    parameter = _dependent(
        value,
        values=values,
        profile=profile,
        unit=unit,
        method=method,
        depends_on=depends_on,
        case=case,
    )
    source_record_ids = tuple(
        dict.fromkeys(source_id for dependency in depends_on if dependency in values for source_id in values[dependency].source_record_ids)
    )
    return parameter.model_copy(update={"source_record_ids": source_record_ids})
    ####


def _resolved_number(values: Mapping[str, ResolvedParameter], name: str) -> float:
    value = values[name].value
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"resolved parameter {name!r} must be numeric")
    return float(value)
    ####


def _resolved_text(values: Mapping[str, ResolvedParameter], name: str) -> str:
    value = values[name].value
    if not isinstance(value, str):
        raise ValueError(f"resolved parameter {name!r} must be a string")
    return value
    ####


def _resolved_text_tuple(values: Mapping[str, ResolvedParameter], name: str) -> tuple[str, ...]:
    value = values[name].value
    if not isinstance(value, tuple) or not all(isinstance(item, str) for item in value):
        raise ValueError(f"resolved parameter {name!r} must be a text tuple")
    return value
    ####


def _class_value(values: Mapping[str, float], selected: str, field_name: str) -> float:
    try:
        return values[selected]
    except KeyError as error:
        raise ValueError(f"unsupported {field_name} {selected!r}; choose one of {tuple(values)!r}") from error
    ####


def _resolution_status(
    profile: InterceptorEvidenceProfile,
    values: Mapping[str, ResolvedParameter],
) -> ResolutionStatus:
    if profile.resolution_status is not ResolutionStatus.UNASSESSED:
        return profile.resolution_status
    source_values = sum(item.origin in {ValueOrigin.OBSERVED, ValueOrigin.REPORTED} for item in values.values())
    archetype_values = sum(item.origin is ValueOrigin.ARCHETYPE_ASSUMPTION for item in values.values())
    if source_values >= archetype_values:
        return ResolutionStatus.SOURCE_DOMINANT
    if source_values == 0 or source_values * 2 < archetype_values:
        return ResolutionStatus.ARCHETYPE_DOMINANT
    return ResolutionStatus.MIXED
    ####


__all__ = ["resolve_interceptor"]
####
