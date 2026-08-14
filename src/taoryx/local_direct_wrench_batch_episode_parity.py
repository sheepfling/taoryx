"""Exact batch/episode parity for declared local direct-wrench bridges.

Each installed local screen has both a public batch factory and an
accepted-truth episode. Their common claim is deliberately narrow: a
caller-owned total body wrench is projected through the declared local
authority bridge and then integrated through the source-local velocity/rate
derivative. This module replays an episode's semantic trace through a fresh
batch propagation loop; it does not relabel that bridge as physical control-
surface allocation.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

from .direct_wrench import DIRECT_WRENCH_NAMES, DirectWrenchProjection
from .local_direct_wrench import LocalDirectWrenchScreenConfig
from .local_direct_wrench_screen_registry import resolve_local_direct_wrench_screen_definition
from .vehicle_composition import CompiledVehicleComposition, resolve_vehicle_composition_interface_contract
from .vehicle_execution_bindings import batch_episode_parity_record, resolve_vehicle_execution_binding
from .vehicle_interface import VehicleInterfaceContract, project_committed_status_values, validate_projected_status_values

_X15_ADAPTER_ID = "taoryx.x15.local_direct_wrench_batch_episode_parity.v1"
_HL20_ADAPTER_ID = "taoryx.hl20.local_direct_wrench_batch_episode_parity.v1"
_GENERIC_ADAPTER_ID = "taoryx.local_direct_wrench_batch_episode_parity.v1"
_SUPPORTED_ADAPTER_IDS = {_HL20_ADAPTER_ID, _X15_ADAPTER_ID, _GENERIC_ADAPTER_ID}
_TOLERANCE = 1.0e-12


@dataclass(frozen=True, slots=True)
class LocalDirectWrenchBatchEpisodeParityStep:
    """One replayed direct-wrench action interval."""

    index: int
    time_start_s: float
    time_end_s: float
    status: str
    mismatches: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        return {
            "index": self.index,
            "time_start_s": self.time_start_s,
            "time_end_s": self.time_end_s,
            "status": self.status,
            "mismatches": list(self.mismatches),
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class LocalDirectWrenchBatchEpisodeParityReport:
    """Scoped direct-wrench bridge parity evidence."""

    adapter_id: str
    composition_id: str
    composition_identity_sha256: str
    interface_id: str
    interface_fingerprint_sha256: str
    batch_factory_id: str
    authority_profile_id: str
    integration_step_s: float
    status: str
    steps: tuple[LocalDirectWrenchBatchEpisodeParityStep, ...]
    claim_boundary: str

    def as_dict(self) -> dict[str, object]:
        return {
            "schema": "taoryx.local-direct-wrench-batch-episode-parity/v1alpha1",
            "adapter_id": self.adapter_id,
            "composition_id": self.composition_id,
            "composition_identity_sha256": self.composition_identity_sha256,
            "interface_id": self.interface_id,
            "interface_fingerprint_sha256": self.interface_fingerprint_sha256,
            "batch_factory_id": self.batch_factory_id,
            "episode_authority_profile_id": self.authority_profile_id,
            "integration_step_s": self.integration_step_s,
            "status": self.status,
            "steps": [step.as_dict() for step in self.steps],
            "claim_boundary": self.claim_boundary,
        }
        ####
    ####


def verify_serialized_local_direct_wrench_batch_episode_parity(
    composition: CompiledVehicleComposition,
    payload: Mapping[str, object],
) -> LocalDirectWrenchBatchEpisodeParityReport:
    """Replay one declared direct-wrench trace through the source-local batch path."""

    if _text(payload, "schema") != "taoryx.composition-policy-trace/v1alpha1":
        raise ValueError("unsupported policy trace schema")
    definition = resolve_local_direct_wrench_screen_definition(composition)
    if definition is None:
        raise ValueError("no local direct-wrench parity adapter is registered for this composition")
    parity = batch_episode_parity_record(composition.family_id, composition.mission, composition.fidelity)
    adapter_id = parity.get("adapter_id")
    if parity.get("availability") != "registered" or adapter_id not in _SUPPORTED_ADAPTER_IDS:
        raise ValueError("no declared local direct-wrench parity adapter is registered for this composition")
    if _text(payload, "composition_id") != composition.id or _text(payload, "composition_identity_sha256") != composition.identity_sha256:
        raise ValueError("policy trace composition identity disagrees with the requested parity composition")
    if _text(payload, "authority_profile_id") != "direct_wrench":
        raise ValueError("local direct-wrench parity requires the direct_wrench authority profile")

    batch_binding = resolve_vehicle_execution_binding(composition, "batch")
    episode_binding = resolve_vehicle_execution_binding(composition, "episode")
    if batch_binding.factory_id not in {
        "hl20_local_direct_wrench_screen.v1",
        "local_direct_wrench_screen.v1",
        "x15_local_direct_wrench_screen.v1",
    } or episode_binding.factory_id not in {
        "hl20_local_direct_wrench_episode.v1",
        "x15_local_direct_wrench_episode.v1",
        "local_direct_wrench_episode.v1",
    }:
        raise ValueError("local direct-wrench parity does not recognize the selected execution factories")
    contract = resolve_vehicle_composition_interface_contract(composition)
    if _text(payload, "interface_id") != contract.id or _text(payload, "interface_fingerprint_sha256") != contract.fingerprint:
        raise ValueError("policy trace interface identity disagrees with the parity composition")

    config = definition.config_factory()
    state = _named_state(config.initial_state, config.state_names, "initial state")
    requested = _named_wrench(config.balancing_wrench(config.reference_state), "initial balancing wrench")
    previous = {name: 0.0 for name in DIRECT_WRENCH_NAMES}
    projection = config.limits.project(requested, previous, config.dt_s)
    time_s = 0.0
    records: list[LocalDirectWrenchBatchEpisodeParityStep] = []
    for index, raw_step in enumerate(_sequence(payload.get("steps"), "steps")):
        step = _mapping(raw_step, f"steps[{index}]")
        frame = _mapping(step.get("action_frame"), f"steps[{index}].action_frame")
        _validate_frame(contract, frame, index)
        if abs(_finite(step.get("time_start_s"), f"steps[{index}].time_start_s") - time_s) > _TOLERANCE:
            raise ValueError(f"policy trace step {index} begins outside the previous committed boundary")
        duration_s = _positive_finite(frame.get("duration_s"), f"steps[{index}].action_frame.duration_s")
        if time_s + duration_s > config.duration_s + _TOLERANCE:
            raise ValueError(f"policy trace step {index} exceeds the declared {definition.family_id} local-screen horizon")
        requested = _requested_wrench(_mapping(frame.get("values"), f"steps[{index}].action_frame.values"))
        state, projection, time_s = _advance_batch(config, state, projection.achieved, requested, time_s, duration_s)
        expected_status = _mapping(step.get("status_frame"), f"steps[{index}].status_frame")
        actual_status = _status_values(contract, config, state, projection, time_s, config.duration_s)
        mismatches = _mismatches(_mapping(expected_status.get("values"), f"steps[{index}].status_frame.values"), actual_status, "status")
        if abs(_finite(step.get("time_end_s"), f"steps[{index}].time_end_s") - time_s) > _TOLERANCE:
            mismatches.append("time_end_s differs from batch committed boundary")
        records.append(LocalDirectWrenchBatchEpisodeParityStep(index, _finite(step.get("time_start_s"), f"steps[{index}].time_start_s"), time_s, "pass" if not mismatches else "fail", tuple(mismatches)))

    final_status = _mapping(payload.get("final_status"), "final_status")
    final_mismatches = _mismatches(
        _mapping(final_status.get("values"), "final_status.values"),
        _status_values(contract, config, state, projection, time_s, config.duration_s),
        "final_status",
    )
    if final_mismatches:
        records.append(LocalDirectWrenchBatchEpisodeParityStep(len(records), time_s, time_s, "fail", tuple(final_mismatches)))
    return LocalDirectWrenchBatchEpisodeParityReport(
        str(adapter_id),
        composition.id,
        composition.identity_sha256,
        contract.id,
        contract.fingerprint,
        batch_binding.factory_id,
        "direct_wrench",
        config.dt_s,
        "pass" if all(item.status == "pass" for item in records) else "fail",
        tuple(records),
        (
            "This compares one declared total-wrench action trace through the selected source-local batch and "
            "episode bridge paths at committed boundaries. It is not a flight mission, physical effector allocation, "
            "trim, robustness, or vehicle qualification result. "
            + definition.claim_boundary
        ),
    )
    ####


def _advance_batch(
    config: LocalDirectWrenchScreenConfig,
    state: dict[str, float],
    previous: Mapping[str, float],
    requested: Mapping[str, float],
    time_s: float,
    duration_s: float,
) -> tuple[dict[str, float], DirectWrenchProjection, float]:
    """Advance one held wrench through the exact source-local Euler bridge."""

    remaining = duration_s
    projection = config.limits.project(requested, previous, config.dt_s)
    while remaining > _TOLERANCE:
        step_s = min(config.dt_s, remaining)
        projection = config.limits.project(requested, projection.achieved, step_s)
        derivative = _named_state(config.source_derivative(state, projection.achieved), config.state_names, "source derivative")
        state = {name: state[name] + step_s * derivative[name] for name in config.state_names}
        if any(not math.isfinite(value) for value in state.values()):
            raise RuntimeError("local direct-wrench batch replay produced a nonfinite state")
        time_s += step_s
        remaining -= step_s
    return state, projection, time_s
    ####


def _status_values(
    contract: VehicleInterfaceContract,
    config: LocalDirectWrenchScreenConfig,
    state: Mapping[str, float],
    projection: DirectWrenchProjection,
    time_s: float,
    horizon_s: float,
) -> dict[str, object]:
    requested = projection.requested
    achieved = projection.achieved
    residual = projection.residual
    raw_values = {
        "body_velocity_m_s": [state[name] for name in ("u_m_s", "v_m_s", "w_m_s")],
        "body_rate_rad_s": [state[name] for name in ("p_rad_s", "q_rad_s", "r_rad_s")],
        "requested_force_body_n": [requested[name] for name in ("force_x_n", "force_y_n", "force_z_n")],
        "requested_moment_body_nm": [requested[name] for name in ("moment_x_nm", "moment_y_nm", "moment_z_nm")],
        "achieved_force_body_n": [achieved[name] for name in ("force_x_n", "force_y_n", "force_z_n")],
        "achieved_moment_body_nm": [achieved[name] for name in ("moment_x_nm", "moment_y_nm", "moment_z_nm")],
        "residual_force_body_n": [residual[name] for name in ("force_x_n", "force_y_n", "force_z_n")],
        "residual_moment_body_nm": [residual[name] for name in ("moment_x_nm", "moment_y_nm", "moment_z_nm")],
        "wrench_residual_norm": projection.residual_norm,
        "feedback_norm": _feedback_norm(config, state),
        "wrench_status": projection.status,
        "wrench_saturated": bool(projection.position_saturated or projection.rate_limited),
        "control_realization": "direct_wrench_screen",
        "physical_effector_allocation": False,
        **{str(name): float(value) for name, value in config.resource_values.items()},
    }
    values = project_committed_status_values(
        contract,
        time_s=time_s,
        execution_status="completed" if time_s >= horizon_s - _TOLERANCE else "active",
        raw_values=raw_values,
    )
    validate_projected_status_values(contract, values, context=f"local direct-wrench batch parity t={time_s:.12g} s")
    return values
    ####


def _feedback_norm(config: LocalDirectWrenchScreenConfig, state: Mapping[str, float]) -> float:
    """Match the local episode's declared normalized feedback diagnostic."""

    names = config.assessment_state_names or config.state_names
    scales = dict(zip(config.state_names, config.state_scales, strict=True))
    return math.sqrt(
        sum(
            ((float(state[name]) - float(config.reference_state[name])) / scales[name]) ** 2
            for name in names
        )
    )
    ####


def _validate_frame(contract: VehicleInterfaceContract, frame: Mapping[str, object], index: int) -> None:
    if _text(frame, "interface_id") != contract.id or _text(frame, "interface_fingerprint_sha256") != contract.fingerprint:
        raise ValueError(f"policy trace step {index} action frame interface identity disagrees with composition")
    if _text(frame, "authority_profile_id") != "direct_wrench":
        raise ValueError(f"policy trace step {index} has no compatible direct-wrench action frame")
    values = _mapping(frame.get("values"), f"steps[{index}].action_frame.values")
    if set(values) != {"wrench.force.command", "wrench.moment.command"}:
        raise ValueError(f"policy trace step {index} must declare both direct-wrench action vectors")
    ####


def _requested_wrench(values: Mapping[str, object]) -> dict[str, float]:
    force = _vector(values.get("wrench.force.command"), "wrench.force.command")
    moment = _vector(values.get("wrench.moment.command"), "wrench.moment.command")
    return dict(zip(DIRECT_WRENCH_NAMES, (*force, *moment), strict=True))
    ####


def _named_state(values: Mapping[str, float], names: tuple[str, ...], label: str) -> dict[str, float]:
    if set(values) != set(names):
        raise ValueError(f"{label} axes disagree with the declared source-local state")
    result = {name: float(values[name]) for name in names}
    if any(not math.isfinite(value) for value in result.values()):
        raise ValueError(f"{label} values must be finite")
    return result
    ####


def _named_wrench(values: Mapping[str, float], label: str) -> dict[str, float]:
    if set(values) != set(DIRECT_WRENCH_NAMES):
        raise ValueError(f"{label} axes disagree with the canonical direct wrench")
    result = {name: float(values[name]) for name in DIRECT_WRENCH_NAMES}
    if any(not math.isfinite(value) for value in result.values()):
        raise ValueError(f"{label} values must be finite")
    return result
    ####


def _mismatches(expected: Mapping[str, object], actual: Mapping[str, object], label: str) -> list[str]:
    mismatches: list[str] = []
    if set(expected) != set(actual):
        return [f"{label} channel IDs differ"]
    for name, expected_value in expected.items():
        actual_value = actual[name]
        if isinstance(expected_value, list) and isinstance(actual_value, list):
            if len(expected_value) != len(actual_value) or any(
                not _close(left, right) for left, right in zip(expected_value, actual_value, strict=True)
            ):
                mismatches.append(f"{label}.{name} differs")
        elif not _close(expected_value, actual_value):
            mismatches.append(f"{label}.{name} differs")
    return mismatches
    ####


def _close(left: object, right: object) -> bool:
    if isinstance(left, bool) or isinstance(right, bool):
        return left is right
    if isinstance(left, int | float) and isinstance(right, int | float):
        return math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=_TOLERANCE)
    return left == right
    ####


def _vector(value: object, label: str) -> tuple[float, float, float]:
    if not isinstance(value, list | tuple) or len(value) != 3:
        raise ValueError(f"{label} must be a three-vector")
    result = tuple(_finite(item, label) for item in value)
    return result[0], result[1], result[2]
    ####


def _sequence(value: object, label: str) -> tuple[object, ...]:
    if not isinstance(value, list | tuple):
        raise ValueError(f"{label} must be a sequence")
    return tuple(value)
    ####


def _mapping(value: object, label: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value
    ####


def _text(values: Mapping[str, object], name: str) -> str:
    value = values.get(name)
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be a nonempty string")
    return value
    ####


def _finite(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"{label} must be finite numeric")
    return float(value)
    ####


def _positive_finite(value: object, label: str) -> float:
    result = _finite(value, label)
    if result <= 0.0:
        raise ValueError(f"{label} must be positive")
    return result
    ####


__all__ = [
    "LocalDirectWrenchBatchEpisodeParityReport",
    "LocalDirectWrenchBatchEpisodeParityStep",
    "verify_serialized_local_direct_wrench_batch_episode_parity",
]
