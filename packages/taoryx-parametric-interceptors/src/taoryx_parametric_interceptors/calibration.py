"""Scenario-qualified calibration screens for parametric interceptors."""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Literal, Self

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from taoryx.runtime.environment_runtime import EnvironmentProvider
from taoryx.trajectory.evaluation import (
    EvaluationGate,
    EvaluationMetric,
    EvidenceChannel,
    TrajectoryEvaluation,
)

from .kernel import PointMassMission, PointMassRun, PointMassSample, run_point_mass_interceptor
from .profile import (
    AssumptionCase,
    InterceptorEvidenceProfile,
    ResolvedInterceptorProfile,
    ValueOrigin,
)
from .pseudo6 import Pseudo6Run, Pseudo6Sample, run_pseudo6_interceptor
from .resolver import resolve_interceptor
from .sensor_suite import (
    DEFAULT_SENSOR_SUITE_ID,
    DEFAULT_SENSOR_SUITE_VERSION,
    InterceptorSensorSuite,
    standard_interceptor_sensor_suite,
)

CalibrationMetricId = Literal[
    "peak_speed_mps",
    "maximum_altitude_m",
    "horizontal_distance_m",
    "elapsed_time_s",
    "terminal_waypoint_range_m",
    "propellant_remaining_kg",
    "waypoint_captured",
]
CalibrationFidelity = Literal["point_mass_3dof", "attitude_response_pseudo_6dof"]

_METRIC_UNITS: dict[str, str] = {
    "peak_speed_mps": "m/s",
    "maximum_altitude_m": "m",
    "horizontal_distance_m": "m",
    "elapsed_time_s": "s",
    "terminal_waypoint_range_m": "m",
    "propellant_remaining_kg": "kg",
    "waypoint_captured": "1",
}
_REPORTED_PARAMETER_METRICS: dict[str, CalibrationMetricId] = {
    "reported_max_speed_mps": "peak_speed_mps",
    "reported_max_altitude_m": "maximum_altitude_m",
    "reported_max_range_m": "horizontal_distance_m",
}
DEFAULT_ENVIRONMENT_MODEL_ID = "taoryx.environment.exponential-atmosphere.standard-earth-v1"
DEFAULT_GRAVITY_MODEL_ID = "taoryx.equations.inverse-square-gravity.standard-earth-v1"
DEFAULT_SENSOR_SUITE_FINGERPRINT = standard_interceptor_sensor_suite().fingerprint


class CalibrationTarget(BaseModel):
    """One scenario-qualified observable target and its evidence lineage."""

    model_config = ConfigDict(frozen=True, extra="forbid", str_strip_whitespace=True)

    metric_id: CalibrationMetricId
    target: float | bool
    tolerance: float | None = Field(default=None, gt=0.0)
    unit: str = Field(min_length=1)
    severity: Literal["required", "advisory"] = "advisory"
    source_parameter_id: str | None = None
    source_origin: ValueOrigin | None = None
    source_record_ids: tuple[str, ...] = ()
    source_basis: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_target(self) -> Self:
        expected_unit = _METRIC_UNITS[self.metric_id]
        if self.unit != expected_unit:
            raise ValueError(f"calibration metric {self.metric_id!r} requires unit {expected_unit!r}")
        if self.metric_id == "waypoint_captured":
            if not isinstance(self.target, bool) or self.tolerance is not None:
                raise ValueError("waypoint_captured requires a boolean target without tolerance")
        else:
            if isinstance(self.target, bool) or not math.isfinite(float(self.target)):
                raise ValueError(f"calibration metric {self.metric_id!r} requires a finite numeric target")
            if self.tolerance is None:
                raise ValueError(f"calibration metric {self.metric_id!r} requires a positive tolerance")
        return self
        ####

    ####


class InterceptorCalibrationScenario(BaseModel):
    """Immutable mission and target contract used for one calibration screen."""

    model_config = ConfigDict(frozen=True, extra="forbid", str_strip_whitespace=True)

    scenario_id: str = Field(pattern=r"^[a-z0-9]+(?:[._-][a-z0-9]+)*$")
    mission: PointMassMission = Field(default_factory=PointMassMission)
    fidelity: CalibrationFidelity = "point_mass_3dof"
    targets: tuple[CalibrationTarget, ...] = Field(min_length=1)
    unavailable_target_parameter_ids: tuple[str, ...] = ()
    environment_model_id: str = Field(
        default=DEFAULT_ENVIRONMENT_MODEL_ID,
        min_length=1,
    )
    gravity_model_id: str = Field(
        default=DEFAULT_GRAVITY_MODEL_ID,
        min_length=1,
    )
    sensor_suite_id: str = Field(default=DEFAULT_SENSOR_SUITE_ID, min_length=1)
    sensor_suite_version: str = Field(default=DEFAULT_SENSOR_SUITE_VERSION, min_length=1)
    sensor_suite_fingerprint: str = Field(
        default=DEFAULT_SENSOR_SUITE_FINGERPRINT,
        pattern=r"^[0-9a-f]{64}$",
    )
    scenario_basis: str = Field(min_length=1)
    claim_boundary: str = (
        "Scenario-qualified surrogate comparison only; it does not establish weapon performance, source fidelity, or controller qualification."
    )

    @model_validator(mode="after")
    def validate_scenario(self) -> Self:
        target_ids = tuple(item.metric_id for item in self.targets)
        if len(target_ids) != len(set(target_ids)):
            raise ValueError("calibration scenario target metric IDs must be unique")
        if len(self.unavailable_target_parameter_ids) != len(set(self.unavailable_target_parameter_ids)):
            raise ValueError("unavailable target parameter IDs must be unique")
        return self
        ####

    @property
    def fingerprint(self) -> str:
        payload = self.model_dump(mode="json")
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        return hashlib.sha256(encoded).hexdigest()
        ####

    ####


class InterceptorCalibrationResult(BaseModel):
    """One resolved profile evaluated against one immutable scenario."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    contract: Literal["taoryx.parametric-interceptors.calibration-result/v1"] = "taoryx.parametric-interceptors.calibration-result/v1"
    schema_version: int = 1
    model_id: str
    profile_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    assumption_case: AssumptionCase
    scenario_id: str
    scenario_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    scenario: InterceptorCalibrationScenario
    fidelity: CalibrationFidelity
    sensor_suite_id: str
    sensor_suite_version: str
    sensor_suite_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    navigation_sensor_provider_kind: str
    target_track_sensor_provider_kind: str
    termination: str
    observables: dict[str, float | bool]
    aggregate_normalized_error: float = Field(ge=0.0)
    evaluation: TrajectoryEvaluation
    claim_boundary: str = (
        "Calibration scores compare surrogate observables with declared targets; they do not convert assumptions into observations or qualify the interceptor."
    )

    @model_validator(mode="after")
    def validate_scenario_identity(self) -> Self:
        if self.scenario_id != self.scenario.scenario_id:
            raise ValueError("calibration result scenario ID does not match its embedded scenario")
        if self.scenario_fingerprint != self.scenario.fingerprint:
            raise ValueError("calibration result scenario fingerprint does not match its embedded scenario")
        if self.fidelity != self.scenario.fidelity:
            raise ValueError("calibration result fidelity does not match its embedded scenario")
        if self.sensor_suite_id != self.scenario.sensor_suite_id:
            raise ValueError("calibration result sensor-suite ID does not match its embedded scenario")
        if self.sensor_suite_version != self.scenario.sensor_suite_version:
            raise ValueError("calibration result sensor-suite version does not match its embedded scenario")
        if self.sensor_suite_fingerprint != self.scenario.sensor_suite_fingerprint:
            raise ValueError("calibration result sensor-suite fingerprint does not match its embedded scenario")
        return self
        ####

    ####


class AssumptionCaseScore(BaseModel):
    """Sortable sensitivity score for one resolver assumption case."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    assumption_case: AssumptionCase
    aggregate_normalized_error: float = Field(ge=0.0)
    required_gates_pass: bool

    ####


class AssumptionCaseComparison(BaseModel):
    """Non-optimizing sensitivity comparison across named assumption cases."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    contract: Literal["taoryx.parametric-interceptors.assumption-case-comparison/v1"] = "taoryx.parametric-interceptors.assumption-case-comparison/v1"
    schema_version: Literal[1] = 1
    model_id: str
    source_profile_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    scenario_id: str
    scenario_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    results: tuple[InterceptorCalibrationResult, ...] = Field(min_length=1)
    ranking: tuple[AssumptionCaseScore, ...] = Field(min_length=1)
    best_case: AssumptionCase
    claim_boundary: str = (
        "Ranking is a sensitivity screen over predefined archetype cases, not parameter estimation, "
        "model validation, or evidence that the lowest-score case is physically correct."
    )

    @model_validator(mode="after")
    def validate_cases(self) -> Self:
        result_cases = tuple(item.assumption_case for item in self.results)
        ranking_cases = tuple(item.assumption_case for item in self.ranking)
        if len(result_cases) != len(set(result_cases)):
            raise ValueError("assumption-case comparison results must contain unique cases")
        if set(result_cases) != set(ranking_cases):
            raise ValueError("assumption-case ranking must cover every result exactly once")
        if self.best_case != self.ranking[0].assumption_case:
            raise ValueError("best_case must match the first ranked assumption case")
        if any(item.model_id != self.model_id for item in self.results):
            raise ValueError("assumption-case results must match the comparison model ID")
        if any(item.scenario_fingerprint != self.scenario_fingerprint for item in self.results):
            raise ValueError("assumption-case results must match the comparison scenario fingerprint")
        return self
        ####

    ####


def calibration_scenario_from_reported_profile(
    profile: ResolvedInterceptorProfile,
    scenario_id: str,
    *,
    scenario_basis: str,
    mission: PointMassMission | None = None,
    fidelity: CalibrationFidelity = "point_mass_3dof",
    relative_tolerance: float = 0.20,
    include: Sequence[str] = tuple(_REPORTED_PARAMETER_METRICS),
    severity: Literal["required", "advisory"] = "advisory",
    environment_model_id: str = DEFAULT_ENVIRONMENT_MODEL_ID,
    gravity_model_id: str = DEFAULT_GRAVITY_MODEL_ID,
    sensor_suite_id: str = DEFAULT_SENSOR_SUITE_ID,
    sensor_suite_version: str = DEFAULT_SENSOR_SUITE_VERSION,
    sensor_suite_fingerprint: str = DEFAULT_SENSOR_SUITE_FINGERPRINT,
) -> InterceptorCalibrationScenario:
    """Promote available reported values only after declaring scenario basis.

    The helper records requested-but-unavailable profile parameters rather than
    filling them. A profile with no available requested targets cannot produce
    a scored calibration scenario.
    """

    if not math.isfinite(relative_tolerance) or relative_tolerance <= 0.0:
        raise ValueError("relative_tolerance must be finite and positive")
    unknown = tuple(name for name in include if name not in _REPORTED_PARAMETER_METRICS)
    if unknown:
        raise ValueError(f"unsupported reported calibration parameters: {unknown!r}")
    targets: list[CalibrationTarget] = []
    unavailable: list[str] = []
    for parameter_id in include:
        parameter = profile.parameters.get(parameter_id)
        if parameter is None:
            unavailable.append(parameter_id)
            continue
        value = parameter.value
        if isinstance(value, bool) or not isinstance(value, int | float):
            raise TypeError(f"reported calibration parameter {parameter_id!r} must be numeric")
        metric_id = _REPORTED_PARAMETER_METRICS[parameter_id]
        numeric = float(value)
        targets.append(
            CalibrationTarget(
                metric_id=metric_id,
                target=numeric,
                tolerance=max(abs(numeric) * relative_tolerance, 1.0e-12),
                unit=_METRIC_UNITS[metric_id],
                severity=severity,
                source_parameter_id=parameter_id,
                source_origin=parameter.origin,
                source_record_ids=parameter.source_record_ids,
                source_basis=scenario_basis,
            )
        )
    if not targets:
        raise ValueError(f"none of the requested reported calibration parameters are available; unavailable={tuple(unavailable)!r}")
    return InterceptorCalibrationScenario(
        scenario_id=scenario_id,
        mission=mission or PointMassMission(),
        fidelity=fidelity,
        targets=tuple(targets),
        unavailable_target_parameter_ids=tuple(unavailable),
        environment_model_id=environment_model_id,
        gravity_model_id=gravity_model_id,
        sensor_suite_id=sensor_suite_id,
        sensor_suite_version=sensor_suite_version,
        sensor_suite_fingerprint=sensor_suite_fingerprint,
        scenario_basis=scenario_basis,
    )
    ####


def load_interceptor_calibration_scenario(path: str | Path) -> InterceptorCalibrationScenario:
    """Load one copy-ready YAML calibration scenario."""

    source = Path(path)
    try:
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"could not load interceptor calibration scenario {source}: {error}") from error
    if not isinstance(payload, Mapping):
        raise ValueError(f"interceptor calibration scenario {source} must contain one mapping")
    return InterceptorCalibrationScenario.model_validate(payload)
    ####


def evaluate_interceptor_calibration(
    profile: ResolvedInterceptorProfile,
    scenario: InterceptorCalibrationScenario,
    *,
    environment: EnvironmentProvider | None = None,
    gravity_acceleration: Callable[[float], float] | None = None,
    sensor_suite: InterceptorSensorSuite | None = None,
) -> InterceptorCalibrationResult:
    """Run one selected fidelity and score only declared scenario targets."""

    if environment is None and scenario.environment_model_id != DEFAULT_ENVIRONMENT_MODEL_ID:
        raise ValueError(f"calibration scenario names nonstandard environment {scenario.environment_model_id!r}; supply its registered provider explicitly")
    if gravity_acceleration is None and scenario.gravity_model_id != DEFAULT_GRAVITY_MODEL_ID:
        raise ValueError(f"calibration scenario names nonstandard gravity model {scenario.gravity_model_id!r}; supply its registered implementation explicitly")
    selected_sensor_suite = sensor_suite or standard_interceptor_sensor_suite()
    _validate_sensor_suite_identity(scenario, selected_sensor_suite)
    run: PointMassRun | Pseudo6Run
    if scenario.fidelity == "attitude_response_pseudo_6dof":
        run = run_pseudo6_interceptor(
            profile,
            scenario.mission,
            environment=environment,
            gravity_acceleration=gravity_acceleration,
            imu_adapter=selected_sensor_suite.build_pseudo6_sensor(seed=0),
            target_track_adapter=selected_sensor_suite.build_target_track_sensor(seed=0),
        )
    else:
        run = run_point_mass_interceptor(
            profile,
            scenario.mission,
            environment=environment,
            gravity_acceleration=gravity_acceleration,
            translation_acceleration_adapter=selected_sensor_suite.build_point_mass_sensor(seed=0),
            target_track_adapter=selected_sensor_suite.build_target_track_sensor(seed=0),
        )
    observables, times = _observables(run, scenario.mission)
    metrics = tuple(_score_target(target, observables[target.metric_id], times[target.metric_id]) for target in scenario.targets)
    normalized_errors = tuple(item.normalized_error for item in metrics if item.normalized_error is not None)
    aggregate_error = sum(normalized_errors) / len(normalized_errors) if normalized_errors else 0.0
    evaluation = TrajectoryEvaluation(
        scenario_id=scenario.scenario_id,
        scenario_contract_sha256=scenario.fingerprint,
        validity="valid",
        qualification="unqualified",
        feasibility=_feasibility(run.termination),
        outcome=_outcome(run.termination),
        metrics=metrics,
        gates=_evaluation_gates(profile, scenario, run),
        requested_controls=_requested_controls(scenario.mission),
        resources=(
            EvidenceChannel(
                id="resource.propellant.remaining",
                value=float(observables["propellant_remaining_kg"]),
                unit="kg",
                source="resource",
                status="available",
                time_s=run.samples[-1].time_s,
                provenance="taoryx-parametric-interceptors shared propulsion program",
            ),
        ),
        events=(
            EvidenceChannel(
                id="event.waypoint.capture",
                value=bool(observables["waypoint_captured"]),
                unit=None,
                source="event",
                status="available",
                time_s=run.samples[-1].time_s,
                provenance=f"kernel termination={run.termination}",
            ),
        ),
        claim_boundary=scenario.claim_boundary,
    )
    return InterceptorCalibrationResult(
        model_id=profile.model_id,
        profile_fingerprint=profile.fingerprint,
        assumption_case=profile.assumption_case,
        scenario_id=scenario.scenario_id,
        scenario_fingerprint=scenario.fingerprint,
        scenario=scenario,
        fidelity=scenario.fidelity,
        sensor_suite_id=selected_sensor_suite.id,
        sensor_suite_version=selected_sensor_suite.version,
        sensor_suite_fingerprint=selected_sensor_suite.fingerprint,
        navigation_sensor_provider_kind=(
            selected_sensor_suite.pseudo6_provider.kind
            if scenario.fidelity == "attitude_response_pseudo_6dof"
            else selected_sensor_suite.point_mass_provider.kind
        ),
        target_track_sensor_provider_kind=selected_sensor_suite.target_track_provider.kind,
        termination=run.termination,
        observables=observables,
        aggregate_normalized_error=aggregate_error,
        evaluation=evaluation,
    )
    ####


def compare_assumption_cases(
    profile: InterceptorEvidenceProfile,
    scenario: InterceptorCalibrationScenario,
    *,
    cases: Sequence[AssumptionCase | str] = tuple(AssumptionCase),
    environment: EnvironmentProvider | None = None,
    gravity_acceleration: Callable[[float], float] | None = None,
    sensor_suite: InterceptorSensorSuite | None = None,
) -> AssumptionCaseComparison:
    """Evaluate the same scenario across named resolver uncertainty cases."""

    selected = tuple(AssumptionCase(item) for item in cases)
    if not selected or len(selected) != len(set(selected)):
        raise ValueError("assumption-case comparison requires one or more unique cases")
    results = tuple(
        evaluate_interceptor_calibration(
            resolve_interceptor(profile, assumption_case=case),
            scenario,
            environment=environment,
            gravity_acceleration=gravity_acceleration,
            sensor_suite=sensor_suite,
        )
        for case in selected
    )
    ranked = tuple(
        AssumptionCaseScore(
            assumption_case=item.assumption_case,
            aggregate_normalized_error=item.aggregate_normalized_error,
            required_gates_pass=item.evaluation.required_gates_pass,
        )
        for item in sorted(results, key=lambda item: (item.aggregate_normalized_error, item.assumption_case.value))
    )
    return AssumptionCaseComparison(
        model_id=profile.model_id,
        source_profile_fingerprint=profile.fingerprint,
        scenario_id=scenario.scenario_id,
        scenario_fingerprint=scenario.fingerprint,
        results=results,
        ranking=ranked,
        best_case=ranked[0].assumption_case,
    )
    ####


def _observables(
    run: PointMassRun | Pseudo6Run,
    mission: PointMassMission,
) -> tuple[dict[str, float | bool], dict[str, float]]:
    samples: Sequence[PointMassSample | Pseudo6Sample] = run.samples
    if not samples:
        raise ValueError("calibration run emitted no samples")
    peak_speed = max(samples, key=lambda item: item.speed_mps)
    maximum_altitude = max(samples, key=lambda item: item.altitude_m)
    horizontal = tuple(math.hypot(item.north_m - mission.launch_north_m, item.east_m - mission.launch_east_m) for item in samples)
    horizontal_index = max(range(len(samples)), key=horizontal.__getitem__)
    terminal = samples[-1]
    observables: dict[str, float | bool] = {
        "peak_speed_mps": peak_speed.speed_mps,
        "maximum_altitude_m": maximum_altitude.altitude_m,
        "horizontal_distance_m": horizontal[horizontal_index],
        "elapsed_time_s": terminal.time_s,
        "terminal_waypoint_range_m": terminal.waypoint_range_m,
        "propellant_remaining_kg": terminal.propellant_remaining_kg,
        "waypoint_captured": run.termination in {"waypoint_capture", "target_intercept"},
    }
    times = {
        "peak_speed_mps": peak_speed.time_s,
        "maximum_altitude_m": maximum_altitude.time_s,
        "horizontal_distance_m": samples[horizontal_index].time_s,
        "elapsed_time_s": terminal.time_s,
        "terminal_waypoint_range_m": terminal.time_s,
        "propellant_remaining_kg": terminal.time_s,
        "waypoint_captured": terminal.time_s,
    }
    return observables, times
    ####


def _score_target(target: CalibrationTarget, actual: float | bool, time_s: float) -> EvaluationMetric:
    source = target.source_basis
    if target.source_parameter_id is not None:
        origin = target.source_origin.value if target.source_origin is not None else "unknown"
        source = f"{target.source_parameter_id}; origin={origin}; {source}"
    if target.source_record_ids:
        source = f"{source}; source_record_ids={','.join(target.source_record_ids)}"
    if isinstance(target.target, bool):
        passed = bool(actual) is target.target
        return EvaluationMetric(
            id=f"calibration.{target.metric_id}",
            actual=bool(actual),
            target=target.target,
            unit=target.unit,
            normalized_error=0.0 if passed else 1.0,
            status="pass" if passed else "fail",
            severity=target.severity,
            source=source,
            time_s=time_s,
        )
    numeric_actual = float(actual)
    numeric_target = float(target.target)
    tolerance = target.tolerance
    if tolerance is None:
        raise ValueError("numeric calibration target unexpectedly lacks tolerance")
    error = abs(numeric_actual - numeric_target)
    return EvaluationMetric(
        id=f"calibration.{target.metric_id}",
        actual=numeric_actual,
        target=numeric_target,
        tolerance=tolerance,
        slack=tolerance - error,
        normalized_error=error / tolerance,
        unit=target.unit,
        status="pass" if error <= tolerance else "fail",
        severity=target.severity,
        source=source,
        time_s=time_s,
    )
    ####


def _evaluation_gates(
    profile: ResolvedInterceptorProfile,
    scenario: InterceptorCalibrationScenario,
    run: PointMassRun | Pseudo6Run,
) -> tuple[EvaluationGate, ...]:
    archetype_count = sum(item.origin is ValueOrigin.ARCHETYPE_ASSUMPTION for item in profile.parameters.values())
    unavailable = scenario.unavailable_target_parameter_ids
    return (
        EvaluationGate(
            id="runtime-completed",
            status="pass",
            message=f"kernel produced {len(run.samples)} accepted samples; termination={run.termination}",
        ),
        EvaluationGate(
            id="runtime-dependency-identity",
            status="pass",
            message=(
                f"environment_model_id={scenario.environment_model_id}; gravity_model_id={scenario.gravity_model_id}; "
                f"sensor_suite_id={scenario.sensor_suite_id}; sensor_suite_version={scenario.sensor_suite_version}; "
                f"sensor_suite_fingerprint={scenario.sensor_suite_fingerprint}"
            ),
        ),
        EvaluationGate(
            id="profile-assumption-lineage",
            status="advisory" if archetype_count else "pass",
            message=f"resolved profile contains {archetype_count} archetype-assumption parameters",
        ),
        EvaluationGate(
            id="reported-target-coverage",
            status="advisory" if unavailable else "pass",
            message=(f"requested profile targets unavailable: {unavailable!r}" if unavailable else "all requested reported profile targets were available"),
        ),
    )
    ####


def _validate_sensor_suite_identity(
    scenario: InterceptorCalibrationScenario,
    sensor_suite: InterceptorSensorSuite,
) -> None:
    expected = (
        scenario.sensor_suite_id,
        scenario.sensor_suite_version,
        scenario.sensor_suite_fingerprint,
    )
    actual = (sensor_suite.id, sensor_suite.version, sensor_suite.fingerprint)
    if actual != expected:
        raise ValueError(f"calibration sensor suite does not match the scenario-bound identity: expected id/version/fingerprint={expected!r}, got {actual!r}")
    ####


def _requested_controls(mission: PointMassMission) -> tuple[EvidenceChannel, ...]:
    if mission.objective_kind == "constant_velocity_target":
        return (
            EvidenceChannel(id="requested.target.position.north", value=mission.target_north_m, unit="m", frame="local_ned", source="requested"),
            EvidenceChannel(id="requested.target.position.east", value=mission.target_east_m, unit="m", frame="local_ned", source="requested"),
            EvidenceChannel(id="requested.target.position.altitude", value=mission.target_altitude_m, unit="m", frame="local_ned", source="requested"),
            EvidenceChannel(id="requested.target.velocity.north", value=mission.target_north_velocity_mps, unit="m/s", frame="local_ned", source="requested"),
            EvidenceChannel(id="requested.target.velocity.east", value=mission.target_east_velocity_mps, unit="m/s", frame="local_ned", source="requested"),
            EvidenceChannel(
                id="requested.target.velocity.vertical", value=mission.target_vertical_velocity_mps, unit="m/s", frame="local_ned", source="requested"
            ),
            EvidenceChannel(id="requested.target.capture_radius", value=mission.target_capture_radius_m, unit="m", frame="local_ned", source="requested"),
        )
    return (
        EvidenceChannel(id="requested.waypoint.north", value=mission.waypoint_north_m, unit="m", frame="local_ned", source="requested"),
        EvidenceChannel(id="requested.waypoint.east", value=mission.waypoint_east_m, unit="m", frame="local_ned", source="requested"),
        EvidenceChannel(id="requested.waypoint.altitude", value=mission.waypoint_altitude_m, unit="m", frame="local_ned", source="requested"),
        EvidenceChannel(id="requested.waypoint.capture_radius", value=mission.capture_radius_m, unit="m", frame="local_ned", source="requested"),
    )
    ####


def _feasibility(termination: str) -> Literal["likely_feasible", "unknown", "likely_infeasible"]:
    if termination in {"waypoint_capture", "target_intercept"}:
        return "likely_feasible"
    if termination == "ground_impact":
        return "likely_infeasible"
    return "unknown"
    ####


def _outcome(termination: str) -> Literal["completed", "partial", "time_limited"]:
    if termination in {"waypoint_capture", "target_intercept"}:
        return "completed"
    if termination == "duration":
        return "time_limited"
    return "partial"
    ####


__all__ = [
    "AssumptionCaseComparison",
    "AssumptionCaseScore",
    "CalibrationFidelity",
    "CalibrationMetricId",
    "CalibrationTarget",
    "DEFAULT_SENSOR_SUITE_FINGERPRINT",
    "InterceptorCalibrationResult",
    "InterceptorCalibrationScenario",
    "calibration_scenario_from_reported_profile",
    "compare_assumption_cases",
    "evaluate_interceptor_calibration",
    "load_interceptor_calibration_scenario",
]
####
