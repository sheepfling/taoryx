"""Typed controller-realization and preflight contracts.

The controller design catalog identifies an intended controller.  This module
records what actually produced a command for a resolved case.  It deliberately
contains no simulator imports so the same contract can be used by native,
reduced-order, and external trajectory backends.
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping, Sequence
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .fidelity_contracts import LegacyFidelityTier

ControllerRole = Literal[
    "guidance",
    "attitude",
    "rate",
    "local_regulator",
    "integral_regulator",
    "allocator",
    "actuator",
    "fallback",
    "baseline",
    "unspecified",
]
ControllerImplementation = Literal[
    "lqr",
    "lqi",
    "rslqr",
    "gain_scheduled_lqr",
    "gain_scheduled_rslqr",
    "pid",
    "mpc",
    "pole_placement",
    "dynamic_inversion",
    "rule_based",
    "external",
    "unspecified",
]
ControllerFidelity = LegacyFidelityTier
InterpolationMethod = Literal["nearest", "linear", "cubic", "hold"]
ControllerQualificationStatus = Literal["design", "wiring_verified", "local_stability_verified", "qualified"]
ControllerEvidenceTier = Literal[
    "T0_structural",
    "T1_trimmed",
    "T2_linearized",
    "T3_linearly_controlled",
    "T4_physically_allocated",
    "T5_nonlinearly_validated",
    "T6_envelope_validated",
]
ControllerControlPath = Literal[
    "unspecified",
    "direct_wrench_screen",
    "direct_wrench_bridge",
    "unconstrained_effector_allocation",
    "constrained_effector_allocation",
    "nonlinear_effector_validation",
    "scheduled_nonlinear_validation",
]

_EVIDENCE_TIER_ORDER: dict[ControllerEvidenceTier, int] = {
    "T0_structural": 0,
    "T1_trimmed": 1,
    "T2_linearized": 2,
    "T3_linearly_controlled": 3,
    "T4_physically_allocated": 4,
    "T5_nonlinearly_validated": 5,
    "T6_envelope_validated": 6,
}


class ControllerChannel(BaseModel):
    """One ordered controller state or input with physical metadata."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    name: str = Field(min_length=1)
    order: int = Field(ge=0)
    unit: str = Field(min_length=1)
    frame: str = Field(min_length=1)
    scale: float = Field(gt=0.0)
    lower: float | None = None
    upper: float | None = None

    @model_validator(mode="after")
    def validate_bounds(self) -> ControllerChannel:
        if self.lower is not None and self.upper is not None and self.lower > self.upper:
            raise ValueError(f"controller channel {self.name!r} lower bound exceeds upper bound")
        return self
        ####
    ####


class ClosedLoopPole(BaseModel):
    """JSON-safe representation of one continuous or discrete pole."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    real: float
    imaginary: float = 0.0


class ControllerSchedule(BaseModel):
    """Operating-point schedule metadata for a realized controller."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    coordinates: tuple[str, ...] = ()
    interpolation: InterpolationMethod = "hold"
    domains: dict[str, tuple[float, float]] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_domains(self) -> ControllerSchedule:
        unknown = set(self.domains) - set(self.coordinates)
        if unknown:
            raise ValueError(f"controller schedule domains are not coordinates: {sorted(unknown)}")
        for name, bounds in self.domains.items():
            if bounds[0] > bounds[1]:
                raise ValueError(f"controller schedule domain {name!r} lower bound exceeds upper bound")
        return self
        ####
    ####


class ControllerProvenance(BaseModel):
    """Runtime-facing provenance summary for one active controller."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    architecture: ControllerImplementation
    design_id: str = Field(min_length=1)
    design_version: str = Field(min_length=1)
    design_hash: str = Field(pattern=r"^[0-9a-f]{64}$")
    runtime_backend: str = Field(min_length=1)
    active_trim_id: str = Field(min_length=1)
    trim_source: str = Field(min_length=1)
    linear_model_id: str = Field(min_length=1)
    active_schedule_id: str | None = None
    schedule_variables: tuple[str, ...] = ()
    matrix_hashes: dict[str, str] = Field(default_factory=dict)
    state_scaling_id: str = Field(min_length=1)
    control_scaling_id: str = Field(min_length=1)
    integral_states: tuple[str, ...] = ()
    auxiliary_loops: tuple[str, ...] = ()
    scenario_gain_overrides: bool = False
    qualification_status: ControllerQualificationStatus = "design"
    evidence_tier: ControllerEvidenceTier = "T0_structural"
    control_realization_path: ControllerControlPath = "unspecified"

    @model_validator(mode="after")
    def validate_hashes_and_claim(self) -> ControllerProvenance:
        for name, digest in self.matrix_hashes.items():
            if not digest or len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
                raise ValueError(f"controller provenance matrix hash {name!r} must be a lowercase SHA-256 digest")
        if self.qualification_status == "qualified" and self.scenario_gain_overrides:
            raise ValueError("qualified controller provenance cannot contain scenario gain overrides")
        _validate_evidence_path(self.evidence_tier, self.control_realization_path)
        return self
        ####
    ####
####


class GuidanceReference(BaseModel):
    """Named guidance/reference values before primary regulation."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    time_s: float = Field(ge=0.0)
    segment_id: str = Field(min_length=1)
    source: str = Field(min_length=1)
    values: dict[str, float] = Field(default_factory=dict)
    units: dict[str, str] = Field(default_factory=dict)
    frames: dict[str, str] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_channels(self) -> GuidanceReference:
        if set(self.units) - set(self.values):
            raise ValueError("guidance reference units must identify declared values")
        if set(self.frames) - set(self.values):
            raise ValueError("guidance reference frames must identify declared values")
        _validate_finite_mapping(self.values, "guidance reference")
        return self
        ####
    ####


class GeneralizedControlRequest(BaseModel):
    """Controller output before allocation or actuator limits."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    time_s: float = Field(ge=0.0)
    source: str = Field(min_length=1)
    values: dict[str, float] = Field(default_factory=dict)
    units: dict[str, str] = Field(default_factory=dict)
    frames: dict[str, str] = Field(default_factory=dict)
    saturated: bool = False

    @model_validator(mode="after")
    def validate_channels(self) -> GeneralizedControlRequest:
        if set(self.units) - set(self.values):
            raise ValueError("generalized-control units must identify declared values")
        if set(self.frames) - set(self.values):
            raise ValueError("generalized-control frames must identify declared values")
        _validate_finite_mapping(self.values, "generalized control")
        return self
        ####
    ####


class AllocationResult(BaseModel):
    """Requested, allocated, and achieved control values at one sample."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    allocator_id: str = Field(min_length=1)
    requested: dict[str, float] = Field(default_factory=dict)
    allocated: dict[str, float] = Field(default_factory=dict)
    achieved: dict[str, float] = Field(default_factory=dict)
    residual: dict[str, float] = Field(default_factory=dict)
    wrench_weights: dict[str, float] = Field(default_factory=dict)
    controlled_wrench_axes: tuple[str, ...] = ()
    uncontrolled_wrench_axes: tuple[str, ...] = ()
    saturated_channels: tuple[str, ...] = ()
    rate_limited_channels: tuple[str, ...] = ()
    allocation_status: str = "unspecified"
    commanded_effectors: dict[str, float] = Field(default_factory=dict)
    actual_effectors: dict[str, float] = Field(default_factory=dict)
    effector_rates: dict[str, float] = Field(default_factory=dict)
    actual_residual: dict[str, float] = Field(default_factory=dict)
    effectiveness_matrix: tuple[tuple[float, ...], ...] = ()
    effectiveness_rank: int | None = Field(default=None, ge=0)
    allocator_iterations: int | None = Field(default=None, ge=0)
    solver_message: str = ""

    @model_validator(mode="after")
    def validate_values(self) -> AllocationResult:
        for label, values in (
            ("requested", self.requested),
            ("allocated", self.allocated),
            ("achieved", self.achieved),
            ("residual", self.residual),
            ("commanded effectors", self.commanded_effectors),
            ("actual effectors", self.actual_effectors),
            ("effector rates", self.effector_rates),
            ("actual residual", self.actual_residual),
        ):
            _validate_finite_mapping(values, f"allocation {label}")
        if any(not math.isfinite(float(value)) for row in self.effectiveness_matrix for value in row):
            raise ValueError("allocation effectiveness matrix must contain only finite values")
        _validate_finite_mapping(self.wrench_weights, "allocation wrench weights")
        if any(value < 0.0 for value in self.wrench_weights.values()):
            raise ValueError("allocation wrench weights must be nonnegative")
        wrench_axes = set(self.requested) | set(self.allocated) | set(self.achieved) | set(self.residual)
        if set(self.controlled_wrench_axes) & set(self.uncontrolled_wrench_axes):
            raise ValueError("allocation controlled and uncontrolled wrench axes must be disjoint")
        if set(self.controlled_wrench_axes) - wrench_axes:
            raise ValueError("controlled wrench axes must identify allocation wrench values")
        if set(self.uncontrolled_wrench_axes) - wrench_axes:
            raise ValueError("uncontrolled wrench axes must identify allocation wrench values")
        effector_channels = set(self.commanded_effectors) | set(self.actual_effectors) | set(self.effector_rates)
        allowed_limit_channels = wrench_axes | effector_channels
        if set(self.saturated_channels) - allowed_limit_channels:
            raise ValueError("allocation saturation channels must identify wrench or effector values")
        if set(self.rate_limited_channels) - allowed_limit_channels:
            raise ValueError("allocation rate-limit channels must identify wrench or effector values")
        return self
        ####
    ####


class ControllerRuntimeState(BaseModel):
    """Machine-readable active controller state for showcase telemetry."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    time_s: float = Field(ge=0.0)
    provenance: ControllerProvenance
    active_mode: str = Field(min_length=1)
    active_trim_id: str = Field(min_length=1)
    schedule_coordinates: dict[str, float] = Field(default_factory=dict)
    state_error: dict[str, float] = Field(default_factory=dict)
    reference: GuidanceReference
    requested_control: GeneralizedControlRequest
    allocation: AllocationResult | None = None
    fallback_active: bool = False
    transition_events: tuple[str, ...] = ()

    @model_validator(mode="after")
    def validate_state(self) -> ControllerRuntimeState:
        _validate_finite_mapping(self.schedule_coordinates, "controller schedule")
        _validate_finite_mapping(self.state_error, "controller state error")
        if set(self.schedule_coordinates) - set(self.provenance.schedule_variables):
            raise ValueError("controller schedule coordinates must be declared by controller provenance")
        if self.reference.time_s != self.time_s or self.requested_control.time_s != self.time_s:
            raise ValueError("controller runtime channels must share the committed sample time")
        if self.active_trim_id != self.provenance.active_trim_id:
            raise ValueError("active trim must agree with controller provenance")
        if self.fallback_active and not self.provenance.scenario_gain_overrides and not self.transition_events:
            raise ValueError("fallback activation must be represented by provenance or a transition event")
        return self
        ####
    ####


class ControllerRealization(BaseModel):
    """Immutable evidence of the controller that produced runtime commands.

    Matrix hashes are required for LQR/LQI realizations.  ``state`` and
    ``inputs`` are ordered contracts, not sets: changing their order changes
    the meaning of every matrix and gain entry and must therefore fail closed.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: int = Field(default=1, ge=1)
    id: str = Field(min_length=1)
    role: ControllerRole
    implementation: ControllerImplementation
    implementation_version: str = Field(min_length=1)
    fidelity: ControllerFidelity
    design_id: str = Field(min_length=1)
    states: tuple[ControllerChannel, ...] = Field(min_length=1)
    inputs: tuple[ControllerChannel, ...] = Field(min_length=1)
    reference_bounds: dict[str, tuple[float, float]] = Field(default_factory=dict)
    plant_source: str = Field(min_length=1)
    linearization_source: str = Field(min_length=1)
    operating_point: dict[str, float | str] = Field(default_factory=dict)
    schedule: ControllerSchedule | None = None
    state_scale_id: str = Field(min_length=1)
    control_scale_id: str = Field(min_length=1)
    q_id: str = Field(min_length=1)
    r_id: str = Field(min_length=1)
    a_sha256: str | None = Field(default=None, pattern=r"^[0-9a-f]{64}$")
    b_sha256: str | None = Field(default=None, pattern=r"^[0-9a-f]{64}$")
    q_sha256: str | None = Field(default=None, pattern=r"^[0-9a-f]{64}$")
    r_sha256: str | None = Field(default=None, pattern=r"^[0-9a-f]{64}$")
    k_sha256: str | None = Field(default=None, pattern=r"^[0-9a-f]{64}$")
    integral_states: tuple[str, ...] = ()
    closed_loop_poles: tuple[ClosedLoopPole, ...] = ()
    closed_loop_max_real_pole: float | None = None
    allocator_id: str = Field(min_length=1)
    actuator_bindings: tuple[str, ...] = ()
    control_path: tuple[str, ...] = ()
    fallback_controller_id: str | None = None
    scenario_overrides_allowed: bool = False
    claim_status: ControllerQualificationStatus = "design"
    evidence_tier: ControllerEvidenceTier = "T0_structural"
    control_realization_path: ControllerControlPath = "unspecified"
    provenance: dict[str, str] = Field(default_factory=dict)
    scenario_gain_overrides: tuple[str, ...] = ()

    @model_validator(mode="after")
    def validate_contract(self) -> ControllerRealization:
        _validate_ordered_channels(self.states, "state")
        _validate_ordered_channels(self.inputs, "input")
        state_names = {item.name for item in self.states}
        input_names = {item.name for item in self.inputs}
        if state_names & input_names:
            raise ValueError("controller state and input channels must be disjoint")
        if set(self.reference_bounds) - input_names:
            unknown = sorted(set(self.reference_bounds) - input_names)
            raise ValueError(f"reference bounds name unknown controller inputs: {unknown}")
        if set(self.integral_states) - state_names:
            unknown = sorted(set(self.integral_states) - state_names)
            raise ValueError(f"integral state name unknown controller states: {unknown}")
        if self.implementation in {"lqr", "lqi", "rslqr", "gain_scheduled_lqr", "gain_scheduled_rslqr"}:
            required = {
                "a_sha256": self.a_sha256,
                "b_sha256": self.b_sha256,
                "q_sha256": self.q_sha256,
                "r_sha256": self.r_sha256,
                "k_sha256": self.k_sha256,
            }
            missing = sorted(name for name, value in required.items() if value is None)
            if missing:
                raise ValueError(f"{self.implementation} realization requires matrix hashes: {', '.join(missing)}")
        if self.scenario_gain_overrides and self.scenario_overrides_allowed:
            raise ValueError("controller cannot both allow and record scenario gain overrides in one realization")
        if self.claim_status == "qualified" and (self.scenario_overrides_allowed or self.scenario_gain_overrides):
            raise ValueError("qualified controller realizations cannot allow or record scenario gain overrides")
        if self.closed_loop_poles and self.closed_loop_max_real_pole is None:
            raise ValueError("closed_loop_max_real_pole is required when poles are recorded")
        _validate_evidence_path(self.evidence_tier, self.control_realization_path)
        for name, bounds in self.reference_bounds.items():
            if bounds[0] > bounds[1]:
                raise ValueError(f"reference bound {name!r} lower bound exceeds upper bound")
        return self
        ####
    ####

    def canonical_payload(self) -> dict[str, Any]:
        """Return the realization without a self-referential identity field."""

        return self.model_dump(mode="json")
        ####

    def digest(self) -> str:
        """Return a stable fingerprint for the realization contract."""

        encoded = json.dumps(self.canonical_payload(), sort_keys=True, separators=(",", ":")).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()
        ####


class ControllerPreflightIssue(BaseModel):
    """One fail-closed controller realization diagnostic."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    code: str = Field(min_length=1)
    message: str = Field(min_length=1)
    field: str | None = None


class ControllerPreflightReport(BaseModel):
    """Result of checking a realization against a resolved runtime context."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    controller_id: str
    passed: bool
    issues: tuple[ControllerPreflightIssue, ...] = ()
    realization_sha256: str

    def require_pass(self) -> None:
        """Raise a stable error if the controller cannot be activated."""

        if not self.passed:
            details = "; ".join(f"{item.code}: {item.message}" for item in self.issues)
            raise ValueError(f"controller preflight failed for {self.controller_id!r}: {details}")
        ####
    ####


def preflight_controller_realization(
    realization: ControllerRealization,
    *,
    expected_state_names: Sequence[str] | None = None,
    expected_input_names: Sequence[str] | None = None,
    controllability_rank: int | None = None,
    schedule_point: Mapping[str, float] | None = None,
    connected_outputs: Sequence[str] = (),
    required_outputs: Sequence[str] = (),
    fallback_active: bool = False,
    scenario_overrides: Sequence[str] = (),
) -> ControllerPreflightReport:
    """Validate a realization before it is allowed to command a plant.

    This is intentionally a structural preflight.  Numerical matrix tests
    remain in the LQR solver and plant-specific evidence tools, while this
    function checks that the result is the controller the case declared.
    """

    issues: list[ControllerPreflightIssue] = []

    def issue(code: str, message: str, field: str | None = None) -> None:
        issues.append(ControllerPreflightIssue(code=code, message=message, field=field))
        ####

    if realization.role == "unspecified":
        issue("controller-role-missing", "active controller must declare a role", "role")
    if realization.implementation_version == "unversioned":
        issue("controller-version-missing", "active controller must declare an implementation version", "implementation_version")
    if not realization.plant_source.strip() or realization.plant_source == "unspecified":
        issue("plant-source-missing", "controller must identify the plant or plant family used for synthesis", "plant_source")
    if realization.implementation in {"lqr", "lqi", "rslqr", "gain_scheduled_lqr", "gain_scheduled_rslqr"} and realization.linearization_source in {"", "unspecified"}:
        issue("linearization-source-missing", "LQR-family controllers must identify the linearization source", "linearization_source")
    if expected_state_names is not None and tuple(expected_state_names) != tuple(item.name for item in realization.states):
        issue("state-order-mismatch", "realization state order differs from the resolved plant contract", "states")
    if expected_input_names is not None and tuple(expected_input_names) != tuple(item.name for item in realization.inputs):
        issue("input-order-mismatch", "realization input order differs from the resolved control contract", "inputs")
    if controllability_rank is not None and controllability_rank < len(realization.states):
        issue("uncontrollable-plant", f"controllability rank {controllability_rank} is below state dimension {len(realization.states)}", "states")
    if schedule_point is not None:
        if realization.schedule is None:
            issue("schedule-missing", "an operating-point schedule point was supplied but no schedule is declared", "schedule")
        else:
            for coordinate in realization.schedule.coordinates:
                if coordinate not in schedule_point:
                    issue("schedule-coordinate-missing", f"schedule point lacks coordinate {coordinate!r}", f"schedule.{coordinate}")
            for coordinate, bounds in realization.schedule.domains.items():
                if coordinate in schedule_point and not bounds[0] <= float(schedule_point[coordinate]) <= bounds[1]:
                    issue("schedule-out-of-domain", f"{coordinate}={schedule_point[coordinate]} lies outside [{bounds[0]}, {bounds[1]}]", f"schedule.{coordinate}")
    missing_outputs = sorted(set(required_outputs) - set(connected_outputs))
    for output in missing_outputs:
        issue("disconnected-output", f"declared controller output {output!r} is not connected to the runtime path", "control_path")
    if fallback_active and realization.fallback_controller_id is None:
        issue("hidden-fallback", "a fallback is active but no fallback controller is declared", "fallback_controller_id")
    declared_overrides = tuple(sorted(set(scenario_overrides).union(realization.scenario_gain_overrides)))
    if declared_overrides and not realization.scenario_overrides_allowed:
        issue("scenario-gain-override", f"scenario overrides are not allowed: {', '.join(declared_overrides)}", "scenario_overrides_allowed")
    if realization.claim_status == "qualified" and (realization.scenario_overrides_allowed or declared_overrides):
        issue("qualified-controller-override", "qualified controller realizations cannot allow or record scenario gain overrides", "claim_status")
    required_path = {"allocator", "actuator", "plant"}
    if realization.implementation in {
        "lqr",
        "lqi",
        "rslqr",
        "gain_scheduled_lqr",
        "gain_scheduled_rslqr",
        "pid",
        "mpc",
        "pole_placement",
        "dynamic_inversion",
        "rule_based",
    }:
        missing_path = sorted(required_path - set(realization.control_path))
        if missing_path:
            issue("control-path-incomplete", f"control path omits required stages: {', '.join(missing_path)}", "control_path")
    return ControllerPreflightReport(
        controller_id=realization.id,
        passed=not issues,
        issues=tuple(issues),
        realization_sha256=realization.digest(),
    )
    ####


def _validate_ordered_channels(channels: Sequence[ControllerChannel], kind: str) -> None:
    """Require contiguous order indices and unique names for a channel list."""

    names = [channel.name for channel in channels]
    orders = [channel.order for channel in channels]
    if len(set(names)) != len(names):
        raise ValueError(f"controller {kind} names must be unique")
    if orders != list(range(len(channels))):
        raise ValueError(f"controller {kind} order must be contiguous and match tuple order")
    ####


def _validate_finite_mapping(values: Mapping[str, float], label: str) -> None:
    """Reject NaN/Inf values in machine-readable runtime channels."""

    if any(not math.isfinite(float(value)) for value in values.values()):
        raise ValueError(f"{label} must contain only finite values")
    ####


def _validate_evidence_path(
    evidence_tier: ControllerEvidenceTier,
    control_realization_path: ControllerControlPath,
) -> None:
    """Prevent a controller evidence record from claiming a higher tier.

    Direct wrench and unconstrained matrix-inversion paths remain useful
    stabilizability screens, but they cannot promote a vehicle to physically
    allocated evidence.  Promotion is monotonic and must identify the path
    that actually produced the plant loads.
    """

    level = _EVIDENCE_TIER_ORDER[evidence_tier]
    if control_realization_path in {"direct_wrench_screen", "direct_wrench_bridge", "unconstrained_effector_allocation"} and level > 3:
        raise ValueError(f"{control_realization_path} cannot claim evidence beyond T3_linearly_controlled")
    if level >= 4 and control_realization_path not in {
        "constrained_effector_allocation",
        "nonlinear_effector_validation",
        "scheduled_nonlinear_validation",
    }:
        raise ValueError("T4+ evidence requires constrained physical effector allocation")
    if level >= 5 and control_realization_path not in {"nonlinear_effector_validation", "scheduled_nonlinear_validation"}:
        raise ValueError("T5+ evidence requires nonlinear physical-effector validation")
    if level >= 6 and control_realization_path != "scheduled_nonlinear_validation":
        raise ValueError("T6 evidence requires scheduled nonlinear validation")
    ####


__all__ = [
    "ClosedLoopPole",
    "ControllerChannel",
    "ControllerFidelity",
    "ControllerControlPath",
    "ControllerEvidenceTier",
    "ControllerImplementation",
    "ControllerPreflightIssue",
    "ControllerPreflightReport",
    "ControllerProvenance",
    "ControllerRealization",
    "ControllerRole",
    "ControllerRuntimeState",
    "ControllerQualificationStatus",
    "ControllerSchedule",
    "GuidanceReference",
    "GeneralizedControlRequest",
    "AllocationResult",
    "preflight_controller_realization",
]
