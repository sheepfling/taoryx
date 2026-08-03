"""Generic executable probes for the horizontal family-adapter contract.

The probes deliberately exercise the adapter seam, not a family-specific
mission.  They establish that a declared operation is callable and returns a
finite, structurally valid result; they do not promote a vehicle to a
qualification tier or replace source-specific trim and response evidence.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any, Literal

from .family_adapter import AdapterOperation, StandardFamilyAdapter, TrimFragmentResult

ProbeStatus = Literal["pass", "not_applicable", "blocked"]


@dataclass(frozen=True, slots=True)
class AdapterProbeCase:
    """Family-supplied operating-point inputs for the generic probe."""

    state: Mapping[str, float]
    effectors: Mapping[str, float]
    environment: Mapping[str, float | str] = field(default_factory=dict)
    resources: Mapping[str, float] = field(default_factory=dict)
    events: Sequence[Mapping[str, Any]] = ()
    trim_target: Mapping[str, float] | None = None
    trim_initial_guess: Mapping[str, float] | None = None
    trim_fragment_request: Mapping[str, float | str] = field(default_factory=dict)
    linearize_options: Mapping[str, float | str] = field(
        default_factory=lambda: {
            "state_step": 1.0e-4,
            "control_step": 1.0e-3,
            "comparison_factor": 0.5,
            "maximum_relative_difference": 0.10,
            "comparison_absolute_floor": 1.0e-8,
        }
    )
    desired_wrench: Mapping[str, float] | None = None
    previous_effectors: Mapping[str, float] | None = None
    replay_request: Mapping[str, Any] = field(default_factory=dict)
    dt_s: float = 0.01

    def __post_init__(self) -> None:
        if not math.isfinite(self.dt_s) or self.dt_s <= 0.0:
            raise ValueError("adapter probe dt_s must be finite and positive")
        _finite_values(self.state, "state")
        _finite_values(self.effectors, "effectors")
        _finite_values(self.resources, "resources")
        if self.trim_target is not None:
            _finite_values(self.trim_target, "trim_target")
        if self.trim_initial_guess is not None:
            _finite_values(self.trim_initial_guess, "trim_initial_guess")
        for value in self.trim_fragment_request.values():
            if not isinstance(value, str) and not math.isfinite(float(value)):
                raise ValueError("trim_fragment_request contains a non-finite value")
        if self.desired_wrench is not None:
            _finite_values(self.desired_wrench, "desired_wrench")
        if self.previous_effectors is not None:
            _finite_values(self.previous_effectors, "previous_effectors")
        ####
    ####


@dataclass(frozen=True, slots=True)
class AdapterProbeOperation:
    """Result of one generic adapter operation probe."""

    operation: AdapterOperation
    status: ProbeStatus
    message: str
    details: Mapping[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "operation": self.operation,
            "status": self.status,
            "message": self.message,
            "details": dict(self.details),
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class AdapterProbeReport:
    """Complete operation-level probe report for one family/tier adapter."""

    family_id: str
    adapter_id: str
    tier: str
    status: ProbeStatus
    operations: tuple[AdapterProbeOperation, ...]

    @property
    def blocked(self) -> tuple[AdapterProbeOperation, ...]:
        return tuple(item for item in self.operations if item.status == "blocked")
        ####

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema": "taoryx.family-adapter-probe/v1alpha1",
            "family_id": self.family_id,
            "adapter_id": self.adapter_id,
            "tier": self.tier,
            "status": self.status,
            "operations": [item.as_dict() for item in self.operations],
            "blocked_count": len(self.blocked),
        }
        ####
    ####


def run_adapter_probe(adapter: StandardFamilyAdapter, case: AdapterProbeCase) -> AdapterProbeReport:
    """Exercise all declared operations using one family-supplied case."""

    operations: list[AdapterProbeOperation] = []
    capabilities = adapter.capability_report()
    state = dict(case.state)
    effectors = dict(case.effectors)
    environment = dict(case.environment)
    trim_result: Any = None
    effectiveness: Any = None

    state_derivative = capabilities.capability("state_derivative")
    if state_derivative.status == "available":
        try:
            result = adapter.state_derivative(state, effectors, environment)
            _finite_values(result, "state derivative")
            missing = set(adapter.state_names) - set(result)
            if missing:
                raise ValueError("missing derivative channels: " + ", ".join(sorted(missing)))
            operations.append(AdapterProbeOperation("state_derivative", "pass", "finite derivative returned", {"channel_count": len(result)}))
        except Exception as error:  # noqa: BLE001 - probe must report, not hide, adapter failures
            operations.append(AdapterProbeOperation("state_derivative", "blocked", str(error)))
    else:
        operations.append(AdapterProbeOperation("state_derivative", "not_applicable", state_derivative.reason))

    trim_capability = capabilities.capability("trim")
    if trim_capability.status == "available":
        try:
            target = dict(case.trim_target or state)
            initial_guess = dict(case.trim_initial_guess or effectors)
            trim_result = adapter.trim(target, initial_guess)
            if not trim_result.success:
                raise ValueError("trim did not converge")
            _finite_values(trim_result.state, "trim state")
            _finite_values(trim_result.controls, "trim controls")
            operations.append(AdapterProbeOperation("trim", "pass", "physical trim returned an accepted result"))
        except Exception as error:  # noqa: BLE001 - see state probe rationale
            operations.append(AdapterProbeOperation("trim", "blocked", str(error)))
    else:
        operations.append(AdapterProbeOperation("trim", "not_applicable", trim_capability.reason))

    trim_fragment_capability = capabilities.capability("trim_fragment")
    if trim_fragment_capability.status == "available":
        try:
            fragment = adapter.trim_fragment(case.trim_fragment_request)
            if not isinstance(fragment, TrimFragmentResult):
                raise ValueError("trim_fragment provider did not return TrimFragmentResult")
            if fragment.status == "failed":
                raise ValueError(f"trim fragment reported failure: {fragment.fragment_id}")
            operations.append(
                AdapterProbeOperation(
                    "trim_fragment",
                    "pass",
                    "source-backed trim fragment returned",
                    {
                        "fragment_id": fragment.fragment_id,
                        "status": fragment.status,
                        "max_residual": fragment.max_residual,
                        "claim_boundary": fragment.claim_boundary,
                    },
                )
            )
        except Exception as error:  # noqa: BLE001 - probe must report, not hide, adapter failures
            operations.append(AdapterProbeOperation("trim_fragment", "blocked", str(error)))
    else:
        operations.append(AdapterProbeOperation("trim_fragment", "not_applicable", trim_fragment_capability.reason))

    linearize_capability = capabilities.capability("linearize")
    if linearize_capability.status == "available" and trim_result is not None:
        try:
            linearization = adapter.linearize(trim_result, case.linearize_options)
            if linearization.primary.a_matrix.size == 0 or linearization.primary.b_matrix.size == 0:
                raise ValueError("linearization returned an empty A or B matrix")
            operations.append(
                AdapterProbeOperation(
                    "linearize",
                    "pass",
                    "provenance-bearing A/B matrices returned",
                    {
                        "state_dimension": int(linearization.primary.a_matrix.shape[0]),
                        "control_dimension": int(linearization.primary.b_matrix.shape[1]),
                    },
                )
            )
        except Exception as error:  # noqa: BLE001 - see state probe rationale
            operations.append(AdapterProbeOperation("linearize", "blocked", str(error)))
    elif linearize_capability.status == "available":
        operations.append(AdapterProbeOperation("linearize", "blocked", "linearization requires a passing trim probe"))
    else:
        operations.append(AdapterProbeOperation("linearize", "not_applicable", linearize_capability.reason))

    effectiveness_capability = capabilities.capability("effectiveness")
    if effectiveness_capability.status == "available":
        try:
            effectiveness = adapter.effectiveness(state, effectors)
            if not effectiveness.wrench_names or not effectiveness.effector_names:
                raise ValueError("effectiveness returned no wrench or effector channels")
            operations.append(
                AdapterProbeOperation(
                    "effectiveness",
                    "pass",
                    "finite local effectiveness returned",
                    {"wrench_channels": len(effectiveness.wrench_names), "effector_channels": len(effectiveness.effector_names)},
                )
            )
        except Exception as error:  # noqa: BLE001 - see state probe rationale
            operations.append(AdapterProbeOperation("effectiveness", "blocked", str(error)))
    else:
        operations.append(AdapterProbeOperation("effectiveness", "not_applicable", effectiveness_capability.reason))

    allocate_capability = capabilities.capability("allocate")
    if allocate_capability.status == "available" and effectiveness is not None:
        try:
            desired = dict(case.desired_wrench or effectiveness.reference_wrench)
            previous = dict(case.previous_effectors or effectors)
            allocation = adapter.allocate(state, desired, previous, case.dt_s)
            _finite_values(allocation.achieved_wrench, "achieved wrench")
            _finite_values(allocation.achieved_residual_wrench, "allocation residual")
            operations.append(AdapterProbeOperation("allocate", "pass", "bounded allocation returned achieved wrench", {"status": allocation.allocation.status}))
        except Exception as error:  # noqa: BLE001 - see state probe rationale
            operations.append(AdapterProbeOperation("allocate", "blocked", str(error)))
    elif allocate_capability.status == "available":
        operations.append(AdapterProbeOperation("allocate", "blocked", "allocation requires a passing effectiveness probe"))
    else:
        operations.append(AdapterProbeOperation("allocate", "not_applicable", allocate_capability.reason))

    resource_capability = capabilities.capability("resource_rates")
    if resource_capability.status == "available":
        try:
            rates = adapter.resource_rates(state, effectors)
            _finite_values(rates, "resource rates")
            operations.append(AdapterProbeOperation("resource_rates", "pass", "finite resource rates returned", {"channel_count": len(rates)}))
        except Exception as error:  # noqa: BLE001 - see state probe rationale
            operations.append(AdapterProbeOperation("resource_rates", "blocked", str(error)))
    else:
        operations.append(AdapterProbeOperation("resource_rates", "not_applicable", resource_capability.reason))

    observe_capability = capabilities.capability("observe")
    if observe_capability.status == "available":
        try:
            observation = adapter.observe(state, effectors, case.resources, case.events)
            if not isinstance(observation, Mapping):
                raise ValueError("observation provider did not return a mapping")
            operations.append(AdapterProbeOperation("observe", "pass", "observation mapping returned", {"channel_count": len(observation)}))
        except Exception as error:  # noqa: BLE001 - see state probe rationale
            operations.append(AdapterProbeOperation("observe", "blocked", str(error)))
    else:
        operations.append(AdapterProbeOperation("observe", "not_applicable", observe_capability.reason))

    replay_capability = capabilities.capability("replay")
    if replay_capability.status == "available":
        try:
            replay = adapter.replay(case.replay_request)
            if not isinstance(replay, Mapping):
                raise ValueError("replay provider did not return a mapping")
            rows = replay.get("rows", replay.get("history"))
            if not isinstance(rows, Sequence) or isinstance(rows, (str, bytes)) or not rows:
                raise ValueError("replay provider returned no rows")
            operations.append(
                AdapterProbeOperation(
                    "replay",
                    "pass",
                    "source replay returned a non-empty artifact",
                    {"row_count": len(rows), "keys": sorted(str(key) for key in replay)},
                )
            )
        except Exception as error:  # noqa: BLE001 - probe must report, not hide, replay failures
            operations.append(AdapterProbeOperation("replay", "blocked", str(error)))
    else:
        operations.append(AdapterProbeOperation("replay", "not_applicable", replay_capability.reason))

    status: ProbeStatus = "blocked" if any(item.status == "blocked" for item in operations) else "pass"
    descriptor = adapter.describe()
    return AdapterProbeReport(descriptor.family_id, descriptor.adapter_id, descriptor.tier, status, tuple(operations))
    ####


def _finite_values(values: Mapping[str, float], label: str) -> None:
    if any(not math.isfinite(float(value)) for value in values.values()):
        raise ValueError(f"{label} contains a non-finite value")
    ####


__all__ = [
    "AdapterProbeCase",
    "AdapterProbeOperation",
    "AdapterProbeReport",
    "ProbeStatus",
    "run_adapter_probe",
]
