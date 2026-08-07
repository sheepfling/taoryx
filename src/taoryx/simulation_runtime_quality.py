"""Numerical-quality and fidelity-boundary gates for Simulation Runtime artifacts.

The Simulation Runtime run manifest answers what was executed.  This module answers
the narrower question of whether the emitted result has the declared
numerical evidence.  It deliberately does not infer quality from a completed
status, a plot, or a fidelity label.
"""

from __future__ import annotations

import hashlib
import json
import math
from bisect import bisect_right
from collections.abc import Mapping, Sequence
from typing import TYPE_CHECKING, Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from taoryx.outputs import RunArtifact, VehicleTelemetry
from taoryx.validation import (
    continuity_audit,
    energy_balance_residual,
    integral_mass_balance_error,
)

if TYPE_CHECKING:
    from taoryx.simulation_runtime_catalog import SimulationRuntimeScenario


QualityDisposition = Literal["pass", "development", "blocked", "bounded-failure"]
InvariantKind = Literal["mass_balance", "energy_balance"]


class SimulationRuntimeFidelityContract(BaseModel):
    """Explicit fidelity and claim-boundary metadata for one scenario."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    lane: str = Field(pattern=r"^[a-z0-9][a-z0-9-]*$")
    claim_boundary: str = Field(min_length=1)
    response_law: str | None = None
    omitted_physics: tuple[str, ...] = ()
    controls: tuple[str, ...] = ()
    envelope: str = Field(min_length=1)
    operation_availability: tuple[str, ...] = Field(min_length=1)
    ####


class SimulationRuntimeRepeatabilitySpec(BaseModel):
    """Fixed-input repeatability policy for one canonical scenario."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    enabled: bool = True
    channels: tuple[str, ...] = ()
    absolute_tolerance: float = Field(default=0.0, ge=0.0)
    relative_tolerance: float = Field(default=0.0, ge=0.0)

    @model_validator(mode="after")
    def require_channels_when_enabled(self) -> SimulationRuntimeRepeatabilitySpec:
        if self.enabled and not self.channels:
            raise ValueError("enabled repeatability checks must name at least one channel")
        return self
        ####
    ####


class SimulationRuntimeRefinementSpec(BaseModel):
    """Fixed-step and integrator refinement policy."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    enabled: bool = False
    coarse_step_multiplier: float = Field(default=2.0, gt=0.0)
    fine_step_multiplier: float = Field(default=1.0, gt=0.0)
    coarse_integrator: str = "rk4"
    fine_integrator: str = "rk4"
    channels: tuple[str, ...] = ()
    max_absolute_error: float = Field(default=0.0, ge=0.0)
    max_event_time_error_s: float = Field(default=1.0e-9, ge=0.0)

    @model_validator(mode="after")
    def require_channels_when_enabled(self) -> SimulationRuntimeRefinementSpec:
        if self.enabled and not self.channels:
            raise ValueError("enabled refinement checks must name at least one channel")
        if not self.coarse_integrator.strip() or not self.fine_integrator.strip():
            raise ValueError("refinement integrators must not be empty")
        return self
        ####
    ####


class SimulationRuntimeInvariantSpec(BaseModel):
    """One explicitly declared conservation or closure invariant."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(pattern=r"^[a-z0-9][a-z0-9-]*$")
    kind: InvariantKind
    required: bool = True
    tolerance: float = Field(ge=0.0)
    mass_channel: str = "mass.total"
    mass_rate_channel: str = "propulsion.mass_flow"
    altitude_channel: str = "position.altitude.geodetic"
    speed_channel: str = "taos.vel"
    drag_force_channel: str = "aerodynamics.drag_force"
    thrust_force_channel: str = "propulsion.thrust"
    ####


class SimulationRuntimeQualitySpec(BaseModel):
    """M4 gate declarations carried by a Simulation Runtime catalog entry."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    fidelity: SimulationRuntimeFidelityContract
    artifact_required: bool = True
    finite_telemetry: bool = True
    finite_channels: tuple[str, ...] = ()
    event_ordering: bool = True
    continuity_channels: dict[str, float] = Field(default_factory=dict)
    repeatability: SimulationRuntimeRepeatabilitySpec = Field(default_factory=SimulationRuntimeRepeatabilitySpec)
    refinement: SimulationRuntimeRefinementSpec = Field(default_factory=SimulationRuntimeRefinementSpec)
    invariants: tuple[SimulationRuntimeInvariantSpec, ...] = ()

    @model_validator(mode="after")
    def validate_continuity_tolerances(self) -> SimulationRuntimeQualitySpec:
        if any(value < 0.0 or not math.isfinite(value) for value in self.continuity_channels.values()):
            raise ValueError("continuity tolerances must be finite and nonnegative")
        identifiers = [item.id for item in self.invariants]
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("quality invariant identifiers must be unique")
        return self
        ####
    ####


class SimulationRuntimeQualityCheck(BaseModel):
    """One machine-readable M4 gate result."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(pattern=r"^[a-z0-9][a-z0-9-]*$")
    disposition: QualityDisposition
    required: bool = True
    message: str = Field(min_length=1)
    observed: dict[str, Any] = Field(default_factory=dict)
    ####


class SimulationRuntimeNumericalQualityReport(BaseModel):
    """Complete M4 report for one canonical scenario."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_id: str = Field(
        default="taoryx.simulation-runtime-numerical-quality/v1alpha1",
        alias="schema",
        serialization_alias="schema",
    )
    schema_version: int = 1
    scenario_id: str = Field(min_length=1)
    disposition: QualityDisposition
    fidelity: str = Field(min_length=1)
    realization: str = Field(min_length=1)
    fidelity_lane: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)
    checks: tuple[SimulationRuntimeQualityCheck, ...]

    @property
    def status(self) -> QualityDisposition:
        """Expose the report disposition under the common status spelling."""

        return self.disposition
        ####

    def as_dict(self) -> dict[str, object]:
        """Return the stable JSON projection used by tools and bundles."""

        return self.model_dump(mode="json", by_alias=True)
        ####
    ####


def evaluate_simulation_runtime_quality(
    scenario: SimulationRuntimeScenario,
    *,
    primary: RunArtifact | None = None,
    repeat: RunArtifact | None = None,
    coarse: RunArtifact | None = None,
    fine: RunArtifact | None = None,
) -> SimulationRuntimeNumericalQualityReport:
    """Evaluate declared M4 checks without promoting nominal execution.

    Missing artifacts are reported as ``blocked`` when the scenario requires
    a common artifact, and as ``development`` for showcase lanes that have a
    family-owned artifact contract but no normalized Simulation Runtime projection.
    """

    quality = scenario.quality
    checks: list[SimulationRuntimeQualityCheck] = []
    checks.append(_fidelity_declaration_check(scenario))
    checks.append(_claim_boundary_check(scenario))

    if primary is None:
        missing_disposition: QualityDisposition = "blocked" if quality.artifact_required else "development"
        message = (
            "no normalized run artifact was supplied for the declared numerical gates"
            if quality.artifact_required
            else "family-owned evidence is outside the normalized Simulation Runtime artifact gate"
        )
        checks.append(_check("artifact-evidence", missing_disposition, quality.artifact_required, message))
        checks.extend(_missing_artifact_checks(quality, missing_disposition))
    else:
        checks.extend(_audit_artifact(primary, quality))

    if quality.repeatability.enabled:
        if primary is None or repeat is None:
            checks.append(
                _check(
                    "repeatability",
                    "blocked" if quality.artifact_required else "development",
                    quality.artifact_required,
                    "repeatability requires two artifacts from identical inputs and configuration",
                )
            )
        else:
            checks.append(_repeatability_check(primary, repeat, quality.repeatability))
    else:
        checks.append(_check("repeatability", "development", False, "repeatability is not declared for this scenario"))

    if quality.refinement.enabled:
        if coarse is None or fine is None:
            checks.append(
                _check(
                    "refinement",
                    "blocked" if quality.artifact_required else "development",
                    quality.artifact_required,
                    "refinement requires coarse and fine artifacts with explicit integration configurations",
                )
            )
        else:
            checks.append(_refinement_check(coarse, fine, quality.refinement))
    else:
        checks.append(_check("refinement", "development", False, "step/integrator refinement is not selected for this scenario"))

    disposition = _report_disposition(checks, scenario.expected_disposition.value)
    return SimulationRuntimeNumericalQualityReport(
        scenario_id=scenario.id,
        disposition=disposition,
        fidelity=scenario.fidelity,
        realization=scenario.realization,
        fidelity_lane=quality.fidelity.lane,
        claim_boundary=scenario.claim_boundary,
        checks=tuple(checks),
    )
    ####


def validate_simulation_runtime_quality_catalog(scenarios: Sequence[SimulationRuntimeScenario]) -> tuple[str, ...]:
    """Validate cross-scenario lane separation and M4 declaration coverage."""

    errors: list[str] = []
    lanes_by_family: dict[str, set[str]] = {}
    refinement_scenarios: list[str] = []
    for scenario in scenarios:
        contract = scenario.quality.fidelity
        if scenario.operation not in contract.operation_availability:
            errors.append(f"{scenario.id}: operation {scenario.operation!r} is absent from fidelity availability")
        if not contract.claim_boundary.strip():
            errors.append(f"{scenario.id}: fidelity claim boundary is empty")
        lanes_by_family.setdefault(scenario.family, set()).add(contract.lane)
        if scenario.quality.refinement.enabled:
            refinement_scenarios.append(scenario.id)
        if "pseudo_6dof" in scenario.fidelity.casefold():
            missing = {
                name: value
                for name, value in (
                    ("response_law", contract.response_law),
                    ("omitted_physics", contract.omitted_physics),
                    ("controls", contract.controls),
                    ("envelope", contract.envelope),
                    ("operation_availability", contract.operation_availability),
                )
                if not value
            }
            if missing:
                errors.append(f"{scenario.id}: pseudo-6DOF fidelity declaration is incomplete: {', '.join(sorted(missing))}")
    required_lane_families = {
        "reference_nesc_two_stage_rocket": "nesc",
        "x15": "x15",
        "hl20_mod_k": "hl20",
        "synthetic_california_hawaii": "synthetic-ca-hi",
    }
    for family, expected_lane in required_lane_families.items():
        observed = {lane for lane in lanes_by_family.get(family, set()) if lane == expected_lane}
        if not observed:
            errors.append(f"{family}: required isolated fidelity lane {expected_lane!r} is missing")
    if len(refinement_scenarios) < 2:
        errors.append("M4 requires at least two canonical scenarios with step/integrator refinement declarations")
    return tuple(errors)
    ####


def artifact_digest(artifact: RunArtifact) -> str:
    """Return the deterministic digest used as repeatability evidence."""

    encoded = json.dumps(artifact.model_dump(mode="json"), sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
    ####


def _fidelity_declaration_check(scenario: SimulationRuntimeScenario) -> SimulationRuntimeQualityCheck:
    contract = scenario.quality.fidelity
    return _check(
        "fidelity-declaration",
        "pass",
        True,
        "fidelity tier, lane, response law, omitted physics, controls, envelope, and operation are explicit",
        {
            "lane": contract.lane,
            "response_law": contract.response_law,
            "omitted_physics": list(contract.omitted_physics),
            "controls": list(contract.controls),
            "envelope": contract.envelope,
            "operation_availability": list(contract.operation_availability),
        },
    )
    ####


def _claim_boundary_check(scenario: SimulationRuntimeScenario) -> SimulationRuntimeQualityCheck:
    contract = scenario.quality.fidelity
    return _check(
        "claim-boundary",
        "pass" if scenario.claim_boundary.strip() and contract.claim_boundary.strip() else "bounded-failure",
        True,
        "positive result and nonclaim text are recorded separately from numerical status",
        {"scenario_boundary": scenario.claim_boundary, "fidelity_boundary": contract.claim_boundary},
    )
    ####


def _missing_artifact_checks(quality: SimulationRuntimeQualitySpec, disposition: QualityDisposition) -> tuple[SimulationRuntimeQualityCheck, ...]:
    checks: list[SimulationRuntimeQualityCheck] = []
    if quality.finite_telemetry:
        checks.append(_check("finite-telemetry", disposition, quality.artifact_required, "finite telemetry cannot be evaluated without an artifact"))
    if quality.event_ordering:
        checks.append(_check("event-ordering", disposition, quality.artifact_required, "event ordering cannot be evaluated without an artifact"))
    if quality.continuity_channels:
        checks.append(_check("state-continuity", disposition, quality.artifact_required, "state continuity cannot be evaluated without an artifact"))
    for invariant in quality.invariants:
        checks.append(_check(f"invariant-{invariant.id}", disposition, invariant.required, "declared invariant cannot be evaluated without an artifact"))
    return tuple(checks)
    ####


def _audit_artifact(artifact: RunArtifact, quality: SimulationRuntimeQualitySpec) -> tuple[SimulationRuntimeQualityCheck, ...]:
    checks: list[SimulationRuntimeQualityCheck] = []
    if quality.finite_telemetry:
        checks.append(_finite_telemetry_check(artifact, quality.finite_channels))
    if quality.event_ordering:
        checks.append(_event_ordering_check(artifact))
    if quality.continuity_channels:
        checks.append(_continuity_check(artifact, quality.continuity_channels))
    for invariant in quality.invariants:
        checks.append(_invariant_check(artifact, invariant))
    return tuple(checks)
    ####


def _finite_telemetry_check(artifact: RunArtifact, selected_channels: Sequence[str] = ()) -> SimulationRuntimeQualityCheck:
    nonfinite: list[dict[str, object]] = []
    missing_channels: list[dict[str, object]] = []
    invalid_times: list[dict[str, object]] = []
    sample_count = 0
    for vehicle_id, vehicle in artifact.vehicles.items():
        previous: float | None = None
        for index, time_s in enumerate(vehicle.times):
            sample_count += 1
            if not math.isfinite(float(time_s)) or (previous is not None and float(time_s) <= previous):
                invalid_times.append({"vehicle": vehicle_id, "index": index, "time_s": time_s})
            previous = float(time_s)
        channels = (
            {name: vehicle.channels.get(name) for name in selected_channels}
            if selected_channels
            else vehicle.channels
        )
        for channel_name, channel in channels.items():
            if channel is None:
                missing_channels.append({"vehicle": vehicle_id, "channel": channel_name})
                continue
            for index, value in enumerate(channel.values):
                if value is None or not math.isfinite(float(value)):
                    nonfinite.append({"vehicle": vehicle_id, "channel": channel_name, "index": index, "value": value})
    passed = not nonfinite and not missing_channels and not invalid_times and bool(artifact.vehicles)
    return _check(
        "finite-telemetry",
        "pass" if passed else "bounded-failure",
        True,
        "all declared telemetry channels are finite and strictly time-ordered" if passed else "telemetry contains a missing/non-finite value, missing declared channel, or invalid time order",
        {"vehicle_count": len(artifact.vehicles), "sample_count": sample_count, "selected_channels": list(selected_channels), "missing_channels": missing_channels[:10], "nonfinite": nonfinite[:10], "invalid_times": invalid_times[:10]},
    )
    ####


def _event_ordering_check(artifact: RunArtifact) -> SimulationRuntimeQualityCheck:
    violations: list[dict[str, object]] = []
    event_count = 0
    event_streams: list[tuple[str, Sequence[object]]] = [
        ("artifact", artifact.events),
        *tuple((f"vehicle:{vehicle_id}", vehicle.events) for vehicle_id, vehicle in artifact.vehicles.items()),
    ]
    for source, events in event_streams:
        previous: float | None = None
        for index, event in enumerate(events):
            raw_time = event.get("time_s", event.get("time")) if isinstance(event, Mapping) else getattr(event, "time", None)
            try:
                time_s = float(raw_time) if isinstance(raw_time, (int, float, str)) else float("nan")
            except ValueError:
                time_s = float("nan")
            event_count += 1
            if not math.isfinite(time_s) or (previous is not None and time_s < previous):
                violations.append({"source": source, "index": index, "time_s": raw_time})
            previous = time_s
    return _check(
        "event-ordering",
        "pass" if not violations else "bounded-failure",
        True,
        "events are finite and nondecreasing in each recorded stream" if not violations else "event stream contains an invalid or out-of-order timestamp",
        {"event_count": event_count, "violations": violations[:10]},
    )
    ####


def _continuity_check(artifact: RunArtifact, channels: Mapping[str, float]) -> SimulationRuntimeQualityCheck:
    reports: list[dict[str, object]] = []
    missing: list[str] = []
    violations: list[dict[str, object]] = []
    for channel_name, tolerance in channels.items():
        found = False
        for vehicle_id, vehicle in artifact.vehicles.items():
            channel = vehicle.channels.get(channel_name)
            if channel is None:
                continue
            found = True
            samples = tuple(
                {"time_s": time_s, channel_name: value}
                for time_s, value in zip(vehicle.times, channel.values, strict=True)
                if value is not None
            )
            event_times = tuple(event.time for event in vehicle.events)
            try:
                report = continuity_audit(samples, {channel_name: tolerance}, event_times=event_times)
            except (AssertionError, ValueError) as error:
                reports.append({"vehicle": vehicle_id, "channel": channel_name, "error": str(error)})
                violations.append({"vehicle": vehicle_id, "channel": channel_name, "error": str(error)})
            else:
                reports.append({"vehicle": vehicle_id, "channel": channel_name, "passed": report["passed"], "maximum_abs_delta": report["maximum_abs_delta"]})
                unexplained = report.get("unexplained_violations", [])
                if isinstance(unexplained, list):
                    violations.extend({"vehicle": vehicle_id, **item} for item in unexplained if isinstance(item, Mapping))
        if not found:
            missing.append(channel_name)
    passed = not missing and not violations and bool(reports)
    return _check(
        "state-continuity",
        "pass" if passed else "bounded-failure",
        True,
        "declared state channels are continuous except at declared events" if passed else "state continuity has missing channels or unexplained jumps",
        {"reports": reports, "missing_channels": missing, "violations": violations[:10]},
    )
    ####


def _invariant_check(artifact: RunArtifact, invariant: SimulationRuntimeInvariantSpec) -> SimulationRuntimeQualityCheck:
    reports: list[dict[str, object]] = []
    missing: list[str] = []
    for vehicle_id, vehicle in artifact.vehicles.items():
        required: tuple[str, ...]
        if invariant.kind == "mass_balance":
            required = (invariant.mass_channel, invariant.mass_rate_channel)
        else:
            required = (
                invariant.altitude_channel,
                invariant.speed_channel,
                invariant.mass_channel,
                invariant.drag_force_channel,
                invariant.thrust_force_channel,
            )
        if any(name not in vehicle.channels for name in required):
            missing.extend(f"{vehicle_id}:{name}" for name in required if name not in vehicle.channels)
            continue
        rows = _vehicle_rows(vehicle, required)
        try:
            residual = (
                integral_mass_balance_error(rows, mass_channel=invariant.mass_channel, mass_rate_channel=invariant.mass_rate_channel)
                if invariant.kind == "mass_balance"
                else energy_balance_residual(
                    rows,
                    mass_channel=invariant.mass_channel,
                    altitude_channel=invariant.altitude_channel,
                    speed_channel=invariant.speed_channel,
                    drag_force_channel=invariant.drag_force_channel,
                    thrust_force_channel=invariant.thrust_force_channel,
                )
            )
        except (AssertionError, ValueError, ZeroDivisionError) as error:
            reports.append({"vehicle": vehicle_id, "error": str(error)})
            continue
        reports.append({"vehicle": vehicle_id, "residual": residual, "tolerance": invariant.tolerance})
    if missing and not reports:
        return _check(
            f"invariant-{invariant.id}",
            "blocked" if invariant.required else "development",
            invariant.required,
            "declared invariant channels are absent from the artifact",
            {"missing_channels": missing},
        )
    residuals = [float(value) for item in reports if isinstance((value := item.get("residual")), (int, float))]
    passed = bool(residuals) and not missing and max(abs(value) for value in residuals) <= invariant.tolerance
    return _check(
        f"invariant-{invariant.id}",
        "pass" if passed else "bounded-failure",
        invariant.required,
        "declared invariant residual is within tolerance" if passed else "declared invariant residual exceeds tolerance",
        {"reports": reports, "missing_channels": missing},
    )
    ####


def _repeatability_check(first: RunArtifact, second: RunArtifact, spec: SimulationRuntimeRepeatabilitySpec) -> SimulationRuntimeQualityCheck:
    errors: list[dict[str, object]] = []
    maximum_abs = 0.0
    for vehicle_id in sorted(set(first.vehicles) | set(second.vehicles)):
        left = first.vehicles.get(vehicle_id)
        right = second.vehicles.get(vehicle_id)
        if left is None or right is None:
            errors.append({"vehicle": vehicle_id, "reason": "vehicle set differs"})
            continue
        for channel_name in spec.channels:
            left_channel = left.channels.get(channel_name)
            right_channel = right.channels.get(channel_name)
            if left_channel is None or right_channel is None:
                errors.append({"vehicle": vehicle_id, "channel": channel_name, "reason": "channel missing"})
                continue
            if len(left.times) != len(right.times) or any(not math.isclose(a, b, abs_tol=1.0e-12, rel_tol=0.0) for a, b in zip(left.times, right.times, strict=False)):
                errors.append({"vehicle": vehicle_id, "channel": channel_name, "reason": "sample times differ"})
                continue
            if len(left_channel.values) != len(right_channel.values):
                errors.append({"vehicle": vehicle_id, "channel": channel_name, "reason": "sample count differs"})
                continue
            for index, (left_value, right_value) in enumerate(zip(left_channel.values, right_channel.values, strict=True)):
                if left_value is None or right_value is None:
                    if left_value != right_value:
                        errors.append({"vehicle": vehicle_id, "channel": channel_name, "index": index, "reason": "missingness differs"})
                    continue
                delta = abs(float(left_value) - float(right_value))
                maximum_abs = max(maximum_abs, delta)
                if not math.isclose(float(left_value), float(right_value), abs_tol=spec.absolute_tolerance, rel_tol=spec.relative_tolerance):
                    errors.append({"vehicle": vehicle_id, "channel": channel_name, "index": index, "delta": delta})
    if first.scenario_identity != second.scenario_identity:
        errors.append({"reason": "scenario identity differs", "first": first.scenario_identity, "second": second.scenario_identity})
    first_model_fingerprint = first.visualization.get("model_fingerprint")
    second_model_fingerprint = second.visualization.get("model_fingerprint")
    if first_model_fingerprint != second_model_fingerprint:
        errors.append({"reason": "model fingerprint differs", "first": first_model_fingerprint, "second": second_model_fingerprint})
    passed = not errors
    return _check(
        "repeatability",
        "pass" if passed else "bounded-failure",
        True,
        "identical inputs and fixed configuration reproduce the selected telemetry" if passed else "repeated execution differs beyond the declared tolerance",
        {"first_digest": artifact_digest(first), "second_digest": artifact_digest(second), "maximum_abs_error": maximum_abs, "errors": errors[:10]},
    )
    ####


def _refinement_check(coarse: RunArtifact, fine: RunArtifact, spec: SimulationRuntimeRefinementSpec) -> SimulationRuntimeQualityCheck:
    errors: list[dict[str, object]] = []
    maximum_abs = 0.0
    for vehicle_id in sorted(set(coarse.vehicles) | set(fine.vehicles)):
        coarse_vehicle = coarse.vehicles.get(vehicle_id)
        fine_vehicle = fine.vehicles.get(vehicle_id)
        if coarse_vehicle is None or fine_vehicle is None:
            errors.append({"vehicle": vehicle_id, "reason": "vehicle set differs"})
            continue
        for channel_name in spec.channels:
            coarse_channel = coarse_vehicle.channels.get(channel_name)
            fine_channel = fine_vehicle.channels.get(channel_name)
            if coarse_channel is None or fine_channel is None:
                errors.append({"vehicle": vehicle_id, "channel": channel_name, "reason": "channel missing"})
                continue
            try:
                differences = _interpolated_differences(coarse_vehicle, coarse_channel.values, fine_vehicle, fine_channel.values)
            except (AssertionError, ValueError) as error:
                errors.append({"vehicle": vehicle_id, "channel": channel_name, "reason": str(error)})
                continue
            channel_error = max(differences, default=0.0)
            maximum_abs = max(maximum_abs, channel_error)
            if channel_error > spec.max_absolute_error:
                errors.append({"vehicle": vehicle_id, "channel": channel_name, "maximum_abs_error": channel_error, "tolerance": spec.max_absolute_error})
        coarse_events = _event_times(coarse_vehicle)
        fine_events = _event_times(fine_vehicle)
        if len(coarse_events) != len(fine_events):
            errors.append({"vehicle": vehicle_id, "reason": "event count differs", "coarse": len(coarse_events), "fine": len(fine_events)})
        else:
            for index, (coarse_event, fine_event) in enumerate(zip(coarse_events, fine_events, strict=True)):
                if coarse_event[0] != fine_event[0] or abs(coarse_event[1] - fine_event[1]) > spec.max_event_time_error_s:
                    errors.append({"vehicle": vehicle_id, "event_index": index, "coarse": coarse_event, "fine": fine_event})
    configurations_differ = (
        not math.isclose(spec.coarse_step_multiplier, spec.fine_step_multiplier, rel_tol=0.0, abs_tol=0.0)
        or spec.coarse_integrator != spec.fine_integrator
    )
    if not configurations_differ:
        errors.append({"reason": "coarse and fine integration configurations are identical"})
    passed = not errors
    return _check(
        "refinement",
        "pass" if passed else "bounded-failure",
        True,
        "coarse/fine telemetry and event times remain within the declared refinement bound" if passed else "step or integrator refinement exceeds the declared bound",
        {
            "coarse_step_multiplier": spec.coarse_step_multiplier,
            "fine_step_multiplier": spec.fine_step_multiplier,
            "coarse_integrator": spec.coarse_integrator,
            "fine_integrator": spec.fine_integrator,
            "maximum_abs_error": maximum_abs,
            "errors": errors[:10],
        },
    )
    ####


def _interpolated_differences(
    coarse_vehicle: VehicleTelemetry,
    coarse_values: Sequence[float | None],
    fine_vehicle: VehicleTelemetry,
    fine_values: Sequence[float | None],
) -> tuple[float, ...]:
    if len(coarse_vehicle.times) != len(coarse_values) or len(fine_vehicle.times) != len(fine_values):
        raise AssertionError("refinement channels must align with their vehicle times")
    fine_times = tuple(float(value) for value in fine_vehicle.times)
    if any(right <= left for left, right in zip(fine_times, fine_times[1:], strict=False)):
        raise AssertionError("fine refinement times must increase")
    differences: list[float] = []
    for time_s, value in zip(coarse_vehicle.times, coarse_values, strict=True):
        if value is None:
            raise AssertionError("coarse refinement channel contains a missing value")
        if float(time_s) < fine_times[0] - 1.0e-9 or float(time_s) > fine_times[-1] + 1.0e-9:
            # A spawned child can begin at a slightly different accepted
            # event time after refinement.  The event-time comparison is the
            # gate for that boundary; compare the overlapping child history.
            continue
        interpolated = _linear_interpolate(fine_times, fine_values, float(time_s))
        differences.append(abs(float(value) - interpolated))
    if not differences:
        raise ValueError("refinement histories have no overlapping samples")
    return tuple(differences)
    ####


def _linear_interpolate(times: Sequence[float], values: Sequence[float | None], target: float) -> float:
    if target < times[0] - 1.0e-9 or target > times[-1] + 1.0e-9:
        raise ValueError(f"refinement histories do not overlap at time {target}")
    index = bisect_right(times, target)
    if index == 0:
        index = 1
    if index >= len(times):
        index = len(times) - 1
    left_time, right_time = times[index - 1], times[index]
    left_value, right_value = values[index - 1], values[index]
    if left_value is None or right_value is None:
        raise AssertionError("fine refinement channel contains a missing value")
    if abs(target - left_time) <= 1.0e-10:
        return float(left_value)
    if abs(target - right_time) <= 1.0e-10:
        return float(right_value)
    fraction = (target - left_time) / (right_time - left_time)
    return float(left_value) + fraction * (float(right_value) - float(left_value))
    ####


def _vehicle_rows(vehicle: VehicleTelemetry, selected_channels: Sequence[str] | None = None) -> tuple[dict[str, float], ...]:
    rows: list[dict[str, float]] = []
    names = tuple(selected_channels) if selected_channels is not None else tuple(vehicle.channels)
    for index, time_s in enumerate(vehicle.times):
        row: dict[str, float] = {"time_s": float(time_s)}
        for name in names:
            channel = vehicle.channels[name]
            value = channel.values[index]
            if value is None:
                raise AssertionError(f"channel {name!r} contains a missing value")
            row[name] = float(value)
        rows.append(row)
    return tuple(rows)
    ####


def _event_times(vehicle: VehicleTelemetry) -> tuple[tuple[str, float], ...]:
    return tuple((event.name, float(event.time)) for event in vehicle.events)
    ####


def _report_disposition(checks: Sequence[SimulationRuntimeQualityCheck], expected: str) -> QualityDisposition:
    dispositions = {check.disposition for check in checks if check.required}
    if "bounded-failure" in dispositions:
        return "bounded-failure"
    if "blocked" in dispositions:
        return "blocked"
    if "development" in dispositions or expected != "passed":
        return "development"
    return "pass"
    ####


def _check(
    identifier: str,
    disposition: QualityDisposition,
    required: bool,
    message: str,
    observed: Mapping[str, Any] | None = None,
) -> SimulationRuntimeQualityCheck:
    return SimulationRuntimeQualityCheck(
        id=identifier,
        disposition=disposition,
        required=required,
        message=message,
        observed=dict(observed or {}),
    )
    ####


__all__ = [
    "InvariantKind",
    "SimulationRuntimeFidelityContract",
    "SimulationRuntimeInvariantSpec",
    "SimulationRuntimeNumericalQualityReport",
    "SimulationRuntimeQualityCheck",
    "SimulationRuntimeQualitySpec",
    "SimulationRuntimeRefinementSpec",
    "SimulationRuntimeRepeatabilitySpec",
    "QualityDisposition",
    "artifact_digest",
    "evaluate_simulation_runtime_quality",
    "validate_simulation_runtime_quality_catalog",
]
####
