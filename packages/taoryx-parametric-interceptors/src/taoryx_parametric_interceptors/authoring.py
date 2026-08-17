"""File-first authoring helpers for parametric interceptor surrogates."""

from __future__ import annotations

import json
import shlex
from collections import Counter
from collections.abc import Mapping
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field

from taoryx.trajectory.configuration_contract import ConfigurationGroupSchema, ConfigurationParameterSchema

from .catalogue import catalogue_interceptor_record_schema, interceptor_from_catalogue_record
from .profile import (
    AssumptionCase,
    InterceptorEvidenceProfile,
    InterceptorParameterUsage,
    ResolutionStatus,
    ResolvedInterceptorProfile,
    ValueOrigin,
    flat_interceptor_profile_schema,
    interceptor,
    interceptor_from_mapping,
    interceptor_parameter_usage,
)
from .provider import (
    DIRECT_ACCELERATION_MISSION_TEMPLATE_ID,
    PACKAGE_VERSION,
    POINT_MASS_FIDELITY_ID,
    PROVIDER_ID,
    PSEUDO6_FIDELITY_ID,
    TARGET_TRACK_MISSION_TEMPLATE_ID,
    ParametricInterceptorMissionCompositionProvider,
)

InterceptorInputFormat = Literal["auto", "flat", "catalogue"]
DetectedInterceptorInputFormat = Literal["flat", "catalogue"]
InterceptorSchemaSelection = Literal["flat", "catalogue", "all"]

_CALIBRATION_TARGET_IDS = (
    "reported_max_speed_mps",
    "reported_max_range_m",
    "reported_max_altitude_m",
)


class InterceptorAuthoringReadiness(BaseModel):
    """Honest readiness summary for one successfully resolved profile."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    execution_status: Literal["runnable_surrogate", "resolver_only"]
    execution_gate_reason_codes: tuple[str, ...]
    performance_claim_status: Literal["unqualified_surrogate"] = "unqualified_surrogate"
    evidence_status: str
    review_required: bool
    assumption_parameter_ids: tuple[str, ...]
    evidence_gap_ids: tuple[str, ...]
    calibration_target_ids: tuple[str, ...]
    missing_calibration_target_ids: tuple[str, ...]
    inactive_resolution_parameter_ids: tuple[str, ...]
    evidence_only_parameter_ids: tuple[str, ...]
    required_diagnostics: tuple[str, ...]


class InterceptorControllerAnalysisSummary(BaseModel):
    """Discoverable local response-analysis routes for the pseudo-6DOF tier.

    This remains a surrogate response-law inspection surface.  It deliberately
    advertises neither a physical controller nor a stability qualification.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    fidelity_id: Literal["attitude_response_pseudo_6dof"] = PSEUDO6_FIDELITY_ID
    response_analysis_status: str
    response_analysis_contract: str
    operating_point_analysis_status: str
    operating_point_comparison_status: str
    axes: tuple[str, ...]
    provider_operations: tuple[str, ...]
    cli_commands: tuple[str, ...]
    claim_boundary: str = (
        "These are local reduced-order response metrics using a frozen support fraction or one explicit force "
        "operating point. They do not qualify a physical controller, actuator, airframe, gain schedule, or robustness."
    )


class InterceptorCompositionSummary(BaseModel):
    """Small Composition inventory useful to tools without flattening metadata."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    provider_id: str
    provider_version: str
    model_id: str
    model_version: str
    model_metadata_fingerprint: str
    fidelity_ids: tuple[str, ...]
    realization_ids: tuple[str, ...]
    mission_template_ids: tuple[str, ...]
    authority_ids: tuple[str, ...]
    control_channel_ids: tuple[str, ...]
    configuration_parameter_ids: tuple[str, ...]
    environment_model_ids: tuple[str, ...]
    gravity_model_ids: tuple[str, ...]
    sensor_suite_ids: tuple[str, ...]
    output_channel_ids: tuple[str, ...]
    controller_analysis: InterceptorControllerAnalysisSummary


class InterceptorAuthoringReport(BaseModel):
    """Stable machine-readable bridge from evidence file to Composition model."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.parametric-interceptor-authoring-report/v1"] = Field(
        default="taoryx.parametric-interceptor-authoring-report/v1",
        alias="schema",
        serialization_alias="schema",
    )
    source_path: str
    input_format: DetectedInterceptorInputFormat
    profile_fingerprint: str
    resolved_profile: ResolvedInterceptorProfile
    origin_counts: dict[str, int]
    parameters_by_origin: dict[str, tuple[str, ...]]
    usage_counts: dict[str, int]
    parameters_by_usage: dict[str, tuple[str, ...]]
    parameter_usage: dict[str, InterceptorParameterUsage]
    readiness: InterceptorAuthoringReadiness
    composition: InterceptorCompositionSummary
    next_commands: tuple[str, ...]
    claim_boundary: str = (
        "This report proves schema validation, evidence-preserving resolution, and Composition advertisement only. "
        "It does not qualify interceptor performance, a seeker, a controller, or a real weapon variant."
    )


def load_interceptor_authoring_profile(
    path: str | Path,
    *,
    input_format: InterceptorInputFormat = "auto",
) -> tuple[InterceptorEvidenceProfile, DetectedInterceptorInputFormat]:
    """Load either supported YAML shape with deterministic automatic detection."""

    source = Path(path)
    try:
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"could not load interceptor profile {source}: {error}") from error
    if not isinstance(payload, Mapping):
        raise ValueError(f"interceptor profile {source} must contain one mapping")
    detected: DetectedInterceptorInputFormat = "catalogue" if payload.get("kind") == "interceptor_evidence_record" else "flat"
    selected = detected if input_format == "auto" else input_format
    if selected == "catalogue":
        return interceptor_from_catalogue_record(payload), "catalogue"
    return interceptor_from_mapping(payload), "flat"
    ####


def build_interceptor_authoring_report(
    path: str | Path,
    *,
    input_format: InterceptorInputFormat = "auto",
    assumption_case: AssumptionCase | str | None = None,
) -> InterceptorAuthoringReport:
    """Validate, resolve, and advertise one file without running a trajectory."""

    source = Path(path)
    profile, detected = load_interceptor_authoring_profile(source, input_format=input_format)
    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(
        profile,
        assumption_case=assumption_case,
    )
    resolved = provider.resolved_profile(profile.model_id)
    model = provider.model(profile.model_id)
    origins: dict[str, list[str]] = {}
    for parameter_id, parameter in resolved.parameters.items():
        origins.setdefault(parameter.origin.value, []).append(parameter_id)
    origins.setdefault(resolved.drag_coefficient_schedule.origin, []).append("drag_coefficient_schedule")
    origins.setdefault(resolved.thrust_profile_schedule.origin, []).append("thrust_profile_schedule")
    if resolved.second_pulse_thrust_profile_schedule is not None:
        origins.setdefault(resolved.second_pulse_thrust_profile_schedule.origin, []).append("second_pulse_thrust_profile_schedule")
    if resolved.thrust_time_curve is not None:
        origins.setdefault(resolved.thrust_time_curve.origin, []).append("thrust_time_curve")
    if resolved.dual_pulse_thrust_program is not None:
        origins.setdefault(resolved.dual_pulse_thrust_program.origin, []).append("dual_pulse_thrust_program")
        origins.setdefault(resolved.dual_pulse_thrust_program.first_pulse.origin, []).append("dual_pulse_thrust_program.first_pulse")
        origins.setdefault(resolved.dual_pulse_thrust_program.second_pulse.origin, []).append("dual_pulse_thrust_program.second_pulse")
    parameters_by_origin = {origin: tuple(sorted(parameter_ids)) for origin, parameter_ids in sorted(origins.items())}
    origin_counts = dict(sorted(Counter(origin for origin, parameter_ids in origins.items() for _ in parameter_ids).items()))
    parameter_usage = interceptor_parameter_usage(resolved)
    usage_groups: dict[str, list[str]] = {}
    for parameter_id, usage in parameter_usage.items():
        usage_groups.setdefault(usage.usage_class, []).append(parameter_id)
    parameters_by_usage = {usage: tuple(sorted(parameter_ids)) for usage, parameter_ids in sorted(usage_groups.items())}
    usage_counts = {usage: len(parameter_ids) for usage, parameter_ids in parameters_by_usage.items()}
    assumption_ids = [
        parameter_id
        for parameter_id, parameter in resolved.parameters.items()
        if parameter.origin in {ValueOrigin.ARCHETYPE_ASSUMPTION, ValueOrigin.SIMULATION_ASSUMPTION}
    ]
    if resolved.drag_coefficient_schedule.origin in {
        ValueOrigin.ARCHETYPE_ASSUMPTION.value,
        ValueOrigin.SIMULATION_ASSUMPTION.value,
    }:
        assumption_ids.append("drag_coefficient_schedule")
    if resolved.thrust_profile_schedule.origin in {
        ValueOrigin.ARCHETYPE_ASSUMPTION.value,
        ValueOrigin.SIMULATION_ASSUMPTION.value,
    }:
        assumption_ids.append("thrust_profile_schedule")
    if resolved.second_pulse_thrust_profile_schedule is not None and resolved.second_pulse_thrust_profile_schedule.origin in {
        ValueOrigin.ARCHETYPE_ASSUMPTION.value,
        ValueOrigin.SIMULATION_ASSUMPTION.value,
    }:
        assumption_ids.append("second_pulse_thrust_profile_schedule")
    if resolved.thrust_time_curve is not None and resolved.thrust_time_curve.origin in {
        ValueOrigin.ARCHETYPE_ASSUMPTION.value,
        ValueOrigin.SIMULATION_ASSUMPTION.value,
    }:
        assumption_ids.append("thrust_time_curve")
    if resolved.dual_pulse_thrust_program is not None:
        structured_origins = (
            ("dual_pulse_thrust_program", resolved.dual_pulse_thrust_program.origin),
            (
                "dual_pulse_thrust_program.first_pulse",
                resolved.dual_pulse_thrust_program.first_pulse.origin,
            ),
            (
                "dual_pulse_thrust_program.second_pulse",
                resolved.dual_pulse_thrust_program.second_pulse.origin,
            ),
        )
        assumption_ids.extend(
            identifier
            for identifier, origin in structured_origins
            if origin
            in {
                ValueOrigin.ARCHETYPE_ASSUMPTION.value,
                ValueOrigin.SIMULATION_ASSUMPTION.value,
            }
        )
    assumptions = tuple(sorted(assumption_ids))
    calibration_targets = tuple(identifier for identifier in _CALIBRATION_TARGET_IDS if identifier in resolved.parameters)
    missing_targets = tuple(identifier for identifier in _CALIBRATION_TARGET_IDS if identifier not in resolved.parameters)
    execution_status = interceptor_execution_status(resolved)
    execution_gate_reasons = tuple(resolved.required_diagnostics) if execution_status == "resolver_only" else ()
    authorities = tuple(dict.fromkeys(authority.id for realization in model.realizations for authority in realization.controls.authorities))
    control_channels = tuple(dict.fromkeys(channel.id for realization in model.realizations for channel in realization.controls.channels))
    output_channels = tuple(channel.id for channel in (*model.output_schema.core_channels, *model.output_schema.telemetry_channels))
    presentation_properties = {item.id: item.value for item in model.presentation.properties}
    configuration_root = provider.get_model_schema(model.id).root
    if not isinstance(configuration_root, ConfigurationGroupSchema):
        raise RuntimeError("parametric interceptor configuration root must be a group")
    configuration_parameters = {item.id: item for item in configuration_root.children if isinstance(item, ConfigurationParameterSchema)}
    quoted_source = shlex.quote(str(source))
    case_option = "" if assumption_case is None else f" --assumption-case {AssumptionCase(assumption_case).value}"
    controller_analysis = InterceptorControllerAnalysisSummary(
        response_analysis_status=str(presentation_properties["response_analysis_status"]),
        response_analysis_contract=str(presentation_properties["response_analysis_contract"]),
        operating_point_analysis_status=str(presentation_properties["response_operating_point_analysis_status"]),
        operating_point_comparison_status=str(presentation_properties["response_operating_point_comparison_status"]),
        axes=tuple(part.strip() for part in str(presentation_properties["response_analysis_axes"]).split(",")),
        provider_operations=(
            "analyze_pseudo6_response",
            "analyze_pseudo6_response_at_operating_point",
            "compare_pseudo6_responses",
            "compare_pseudo6_responses_at_operating_point",
            "run_pseudo6_attitude_step",
        ),
        cli_commands=(
            f"taoryx-interceptor analyze-response {quoted_source}{case_option}",
            (
                f"taoryx-interceptor analyze-operating-response {quoted_source}{case_option} "
                "response-operating-point.yaml"
            ),
        ),
    )
    return InterceptorAuthoringReport(
        source_path=str(source),
        input_format=detected,
        profile_fingerprint=profile.fingerprint,
        resolved_profile=resolved,
        origin_counts=origin_counts,
        parameters_by_origin=parameters_by_origin,
        usage_counts=usage_counts,
        parameters_by_usage=parameters_by_usage,
        parameter_usage=parameter_usage,
        readiness=InterceptorAuthoringReadiness(
            execution_status=execution_status,
            execution_gate_reason_codes=execution_gate_reasons,
            evidence_status=resolved.resolution_status.value,
            review_required=bool(assumptions or resolved.evidence_gaps or resolved.required_diagnostics),
            assumption_parameter_ids=assumptions,
            evidence_gap_ids=tuple(sorted({item.parameter_id for item in resolved.evidence_gaps})),
            calibration_target_ids=calibration_targets,
            missing_calibration_target_ids=missing_targets,
            inactive_resolution_parameter_ids=parameters_by_usage.get("inactive_resolution_input", ()),
            evidence_only_parameter_ids=parameters_by_usage.get("evidence_only", ()),
            required_diagnostics=resolved.required_diagnostics,
        ),
        composition=InterceptorCompositionSummary(
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            model_id=model.id,
            model_version=model.version,
            model_metadata_fingerprint=model.metadata_fingerprint,
            fidelity_ids=tuple(item.id for item in model.fidelities),
            realization_ids=tuple(item.id for item in model.realizations),
            mission_template_ids=tuple(item.id for item in model.mission_templates),
            authority_ids=authorities,
            control_channel_ids=control_channels,
            configuration_parameter_ids=tuple(configuration_parameters),
            environment_model_ids=configuration_parameters["runtime.environment_model_id"].choices,
            gravity_model_ids=configuration_parameters["runtime.gravity_model_id"].choices,
            sensor_suite_ids=configuration_parameters["runtime.sensor_suite_id"].choices,
            output_channel_ids=output_channels,
            controller_analysis=controller_analysis,
        ),
        next_commands=_next_commands(
            quoted_source,
            case_option,
            execution_status,
        ),
    )
    ####


def interceptor_authoring_schema_bundle(
    selection: InterceptorSchemaSelection = "all",
) -> dict[str, object]:
    """Return the exact file-ingestion schemas for editors, agents, and CI."""

    if selection not in {"flat", "catalogue", "all"}:
        raise ValueError("authoring schema selection must be 'flat', 'catalogue', or 'all'")
    formats: dict[str, object] = {}
    if selection in {"flat", "all"}:
        formats["flat"] = flat_interceptor_profile_schema()
    if selection in {"catalogue", "all"}:
        formats["catalogue"] = catalogue_interceptor_record_schema()
    return {
        "schema": "taoryx.parametric-interceptor-authoring-schema-bundle/v1",
        "provider_id": PROVIDER_ID,
        "provider_version": PACKAGE_VERSION,
        "formats": formats,
    }
    ####


def interceptor_execution_status(
    resolved: ResolvedInterceptorProfile,
) -> Literal["runnable_surrogate", "resolver_only"]:
    """Gate explicitly sparse named variants without blocking generic studies."""

    if resolved.resolution_status is ResolutionStatus.ARCHETYPE_DOMINANT and resolved.required_diagnostics:
        return "resolver_only"
    return "runnable_surrogate"
    ####


def write_interceptor_authoring_template(
    path: str | Path,
    *,
    interceptor_id: str = "my-interceptor",
    input_format: DetectedInterceptorInputFormat = "flat",
    overwrite: bool = False,
) -> Path:
    """Write one valid, copy-ready template while refusing accidental overwrite."""

    if input_format == "flat":
        interceptor(interceptor_id)
        model_id = interceptor_id
    else:
        model_id = interceptor_from_catalogue_record(
            {
                "kind": "interceptor_evidence_record",
                "interceptor_id": interceptor_id,
            }
        ).model_id
    destination = Path(path)
    if destination.exists() and not overwrite:
        raise FileExistsError(f"refusing to overwrite existing interceptor profile {destination}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    template = _flat_template(interceptor_id) if input_format == "flat" else _catalogue_template(interceptor_id, model_id)
    destination.write_text(template, encoding="utf-8")
    return destination
    ####


def report_json(report: InterceptorAuthoringReport) -> str:
    """Serialize a report deterministically for CLI, agents, and golden tests."""

    return (
        json.dumps(
            report.model_dump(mode="json", by_alias=True),
            indent=2,
            sort_keys=True,
        )
        + "\n"
    )
    ####


def _next_commands(
    quoted_source: str,
    case_option: str,
    execution_status: Literal["runnable_surrogate", "resolver_only"],
) -> tuple[str, ...]:
    inspect = f"taoryx-interceptor inspect {quoted_source}{case_option}"
    acknowledgement = " --allow-unqualified" if execution_status == "resolver_only" else ""
    return (
        inspect,
        f"taoryx-interceptor run {quoted_source}{case_option}{acknowledgement} --fidelity {POINT_MASS_FIDELITY_ID}",
        f"taoryx-interceptor run {quoted_source}{case_option}{acknowledgement} --fidelity {PSEUDO6_FIDELITY_ID}",
        (
            f"taoryx-interceptor run {quoted_source}{case_option}{acknowledgement} "
            f"--mission-template {TARGET_TRACK_MISSION_TEMPLATE_ID} --fidelity {POINT_MASS_FIDELITY_ID}"
        ),
        (
            f"taoryx-interceptor run {quoted_source}{case_option}{acknowledgement} "
            f"--mission-template {DIRECT_ACCELERATION_MISSION_TEMPLATE_ID} "
            f"--set control.lateral_acceleration.local.east.command=10 "
            f"--fidelity {POINT_MASS_FIDELITY_ID}"
        ),
        f"taoryx-interceptor calibrate {quoted_source} calibration-scenario.yaml{case_option}{acknowledgement}",
        f"taoryx-interceptor compare-cases {quoted_source} calibration-scenario.yaml{acknowledgement}",
        (
            f"taoryx-interceptor fit {quoted_source} fit-campaign.yaml{acknowledgement} "
            "--output fit-receipt.json --fitted-profile-output fitted-interceptor.yaml"
        ),
    )
    ####


def _flat_template(interceptor_id: str) -> str:
    return f"""\
# Fast-start profile. Raw values are simulation assumptions, never observations.
interceptor_id: {interceptor_id}
parameter_set_version: 0.1.0
surrogate_archetype_id: generic_slender_sam_v1

launch_mass_kg: 350.0
propellant_fraction: 0.45
length_m: 5.0
body_diameter_m: 0.35
propulsion_architecture: single_stage
burn_time_class: medium
thrust_profile_class: boost_sustain
# Optional absolute active-burn mean, before assumption-case/thrust_scale.
# nominal_thrust_n: 25000.0
# Optional explicit unit-mean pulse shape. Raw multipliers are normalized for you;
# if nominal_thrust_n is also supplied, remove the now-unused coarse class.
# thrust_profile_schedule:
#   schedule_id: developer-boost-sustain-v1
#   interpolation: step_previous  # linear | step_previous
#   points:
#     - {{burn_fraction: 0.0, multiplier: 1.6}}
#     - {{burn_fraction: 0.2, multiplier: 0.85}}
#     - {{burn_fraction: 1.0, multiplier: 0.85}}
#   origin: simulation_assumption
#   method: developer-authored relative thrust-time shape
# Alternatively, replace burn_time_class, thrust_profile_class, nominal_thrust_n,
# and thrust_profile_schedule with one source-shaped single-pulse table:
# thrust_time_curve:
#   curve_id: developer-static-motor-curve-v1
#   time_unit: ms
#   thrust_unit: kN
#   points:
#     - {{time: 0.0, thrust: 0.0}}
#     - {{time: 500.0, thrust: 50.0}}
#     - {{time: 4000.0, thrust: 0.0}}
#   origin: simulation_assumption
# For two independent source curves plus a coast, copy
# examples/parametric_interceptors/dual_pulse_thrust_curves_sam.yaml.
# When launch mass, absolute thrust, and burn timing are explicit, optional
# effective_specific_impulse_s derives burnout mass and dual-pulse mass split.
aero_archetype: generic_slender_supersonic
drag_class: nominal
maneuverability_class: moderate
# Optional explicit reduced-order authority; otherwise resolved from the archetype.
# control_configuration: aerodynamic  # aerodynamic | thrust_assisted | mixed
# control_allocation_policy: aerodynamic_first  # aerodynamic_first | thrust_vector_first | proportional
# normal_force_coefficient_limit: 6.0
# max_thrust_vector_angle_rad: 0.174533  # required by thrust_assisted or mixed
# maneuver_drag_factor: 0.1  # Cdm = factor * achieved_Cn^2
control_bandwidth_class: nominal
guidance_family: command
guidance_archetype: waypoint_pursuit
# navigation_constant: 3.0  # used by proportional_navigation
target_classes: [air_breathing]

# Optional advisory model-domain bounds. Missing bounds remain undeclared.
# applicability_altitude_min_m: 100.0
# applicability_altitude_max_m: 25000.0
# applicability_mach_min: 0.1
# applicability_mach_max: 5.0

# Add reported calibration targets only when the scenario and source are known.
# reported_max_speed_mps:
#   value: 1200.0
#   unit: m/s
#   origin: reported
#   source_record_ids: [source:replace-me]

model_notes: >-
  Developer-started surrogate. Replace assumptions with observed, reported, or
  derived evidence records as public-source research becomes available.
"""
    ####


def _catalogue_template(interceptor_id: str, model_id: str) -> str:
    return f"""\
kind: interceptor_evidence_record
interceptor_id: {interceptor_id}
model_id: {model_id}
variant_basis: replace_with_exact_variant
resolution_status: unassessed

# Null fields remain evidence gaps and are never converted into source facts.
evidence:
  launch_mass: {{value: null, unit: kg, status: not_yet_sourced}}
  length: {{value: null, unit: m, status: not_yet_sourced}}
  body_diameter: {{value: null, unit: m, status: not_yet_sourced}}
  reported_speed: {{value: null, unit: m/s, status: not_yet_sourced}}
  reported_range: {{value: null, unit: m, status: not_yet_sourced}}
  reported_altitude: {{value: null, unit: m, status: not_yet_sourced}}

derived: {{}}
resolver_assumptions:
  surrogate_archetype_id: generic_slender_sam_v1
  assumption_case: nominal
  unresolved_parameters:
    - launch_mass
    - length
    - body_diameter
    - thrust_time_profile
    - drag_coefficient_model
    - lateral_acceleration_limit

model_notes: >-
  Evidence-intake scaffold. Archetype-selected values remain simulation
  assumptions and this record is not a qualified performance model.
"""
    ####


__all__ = [
    "DetectedInterceptorInputFormat",
    "InterceptorAuthoringReadiness",
    "InterceptorAuthoringReport",
    "InterceptorCompositionSummary",
    "InterceptorInputFormat",
    "InterceptorSchemaSelection",
    "build_interceptor_authoring_report",
    "interceptor_authoring_schema_bundle",
    "interceptor_execution_status",
    "load_interceptor_authoring_profile",
    "report_json",
    "write_interceptor_authoring_template",
]
