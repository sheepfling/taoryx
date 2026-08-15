"""Exact action-trace parity witness for the F-16 reduced source stepper."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Literal

from taoryx.composition_episode import (
    ActionFrame,
    EpisodeObservation,
    StatusFrame,
    native_action_for_frame,
    status_frame_for_observation,
)
from taoryx.composition_policy import CompositionPolicyTraceRecord, parse_composition_policy_trace_record
from taoryx.vehicle_composition import CompiledVehicleComposition, resolve_vehicle_composition_interface_contract
from taoryx.vehicle_execution_bindings import batch_episode_parity_record, resolve_vehicle_execution_binding
from taoryx.vehicle_interface import VehicleInterfaceContract

from .f16_composition_episode import _f16_guidance_override, _f16_stepper_for_composition
from .trajectory.f16_reduced_racetrack import F16ReducedRacetrackStepper

_ADAPTER_ID = "taoryx.reduced_fixed_wing.f16_action_trace_batch_episode_parity.v1"
_BATCH_FACTORY_ID = "reduced_fixed_wing_f16_source.v1"
_EPISODE_FACTORY_ID = "reduced_fixed_wing_f16_episode.v1"
_TOLERANCE = 1.0e-12


@dataclass(frozen=True, slots=True)
class F16ReducedBatchEpisodeParityStep:
    """One committed action-boundary comparison."""

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
class F16ReducedBatchEpisodeParityReport:
    """One narrowly scoped F-16 reduced-kernel/status parity report."""

    composition_id: str
    composition_identity_sha256: str
    interface_id: str
    interface_fingerprint_sha256: str
    batch_factory_id: str
    authority_profile_id: str
    adapter_id: str
    status: str
    steps: tuple[F16ReducedBatchEpisodeParityStep, ...]

    def as_dict(self) -> dict[str, object]:
        return {
            "schema": "taoryx.composition-batch-episode-parity/v1alpha1",
            "adapter_id": self.adapter_id,
            "composition_id": self.composition_id,
            "composition_identity_sha256": self.composition_identity_sha256,
            "interface_id": self.interface_id,
            "interface_fingerprint_sha256": self.interface_fingerprint_sha256,
            "batch_factory_id": self.batch_factory_id,
            "episode_authority_profile_id": self.authority_profile_id,
            "integration_step_s": 0.2,
            "status": self.status,
            "steps": [step.as_dict() for step in self.steps],
            "claim_boundary": (
                "This compares one semantic kinematic-guidance trace through separately constructed F-16 source-owned "
                "reduced batch-stepper and episode paths at committed boundaries. It is kernel/status parity only—not "
                "physical control-surface realization, moment balance, robustness, mission qualification, or generic "
                "fixed-wing parity evidence."
            ),
        }
        ####

    ####


def verify_serialized_f16_reduced_batch_episode_parity(
    composition: CompiledVehicleComposition,
    payload: object,
) -> F16ReducedBatchEpisodeParityReport:
    """Replay a persisted F-16 policy trace through a fresh source stepper."""

    trace = parse_composition_policy_trace_record(payload)
    _validate_identity(composition, trace)
    contract = resolve_vehicle_composition_interface_contract(composition)
    if trace["interface_id"] != contract.id or trace["interface_fingerprint_sha256"] != contract.fingerprint:
        raise ValueError("policy trace interface identity disagrees with the F-16 parity composition")
    authority_profile_id = trace["authority_profile_id"]
    if authority_profile_id != "kinematic_guidance":
        raise ValueError("F-16 reduced parity requires the kinematic_guidance authority profile")
    batch_binding = resolve_vehicle_execution_binding(composition, "batch")
    episode_binding = resolve_vehicle_execution_binding(composition, "episode")
    if batch_binding.factory_id != _BATCH_FACTORY_ID or episode_binding.factory_id != _EPISODE_FACTORY_ID:
        raise ValueError("F-16 reduced parity does not recognize the selected execution factories")

    stepper = _f16_stepper_for_composition(composition)
    held_native: dict[str, float] = {}
    records: list[F16ReducedBatchEpisodeParityStep] = []
    for index, item in enumerate(trace["steps"]):
        expected = _mapping(item, f"steps[{index}]")
        frame = _action_frame(_mapping(expected.get("action_frame"), f"steps[{index}].action_frame"))
        if frame.authority_profile_id != authority_profile_id:
            raise ValueError(f"policy trace step {index} has no compatible semantic action frame")
        start_time = _finite(expected.get("time_start_s"), f"steps[{index}].time_start_s")
        if abs(start_time - stepper.state.time_s) > _TOLERANCE:
            raise ValueError(f"policy trace step {index} begins outside the previous committed boundary")
        native = native_action_for_frame(contract, frame)
        held_native.update({key: _finite(value, f"native action {key}") for key, value in native.items()})
        stepper.step(frame.duration_s, _f16_guidance_override(held_native))
        actual = _status(stepper, contract)
        expected_status = _mapping(expected.get("status_frame"), f"steps[{index}].status_frame")
        mismatches = _mismatches(_mapping(expected_status.get("values"), "status values"), actual.values, "status")
        expected_end = _finite(expected.get("time_end_s"), f"steps[{index}].time_end_s")
        if abs(expected_end - actual.time_s) > _TOLERANCE:
            mismatches.append("time_end_s differs from batch committed boundary")
        records.append(F16ReducedBatchEpisodeParityStep(index, start_time, actual.time_s, "pass" if not mismatches else "fail", tuple(mismatches)))
    expected_final = trace["final_status"]
    final_mismatches = _mismatches(_mapping(expected_final.get("values"), "final status values"), _status(stepper, contract).values, "final_status")
    if final_mismatches:
        now = stepper.state.time_s
        records.append(F16ReducedBatchEpisodeParityStep(len(records), now, now, "fail", tuple(final_mismatches)))
    return F16ReducedBatchEpisodeParityReport(
        composition.id,
        composition.identity_sha256,
        contract.id,
        contract.fingerprint,
        batch_binding.factory_id,
        authority_profile_id,
        _ADAPTER_ID,
        "pass" if all(item.status == "pass" for item in records) else "fail",
        tuple(records),
    )
    ####


def _validate_identity(composition: CompiledVehicleComposition, payload: CompositionPolicyTraceRecord) -> None:
    if (
        composition.family_id != "f16_s119"
        or composition.mission != "powered_fixed_wing_racetrack_v1"
        or composition.fidelity not in {"point_mass_3dof", "pseudo_6dof"}
    ):
        raise ValueError("no F-16 reduced batch/episode parity adapter is registered for this composition")
    parity = batch_episode_parity_record(composition.family_id, composition.mission, composition.fidelity)
    if parity.availability != "registered" or parity.adapter_id != _ADAPTER_ID:
        raise ValueError("no F-16 reduced batch/episode parity adapter is registered for this composition")
    if _text(payload, "composition_id") != composition.id or _text(payload, "composition_identity_sha256") != composition.identity_sha256:
        raise ValueError("policy trace composition identity disagrees with the requested parity composition")
    ####


def _status(stepper: F16ReducedRacetrackStepper, contract: VehicleInterfaceContract) -> StatusFrame:
    row = stepper.current_row(_f16_guidance_override({}))
    lifecycle: Literal["active", "completed"] = "completed" if stepper.completed else "active"
    return status_frame_for_observation(contract, EpisodeObservation(stepper.state.time_s, _status_values(row), lifecycle))
    ####


def _status_values(row: Mapping[str, object]) -> dict[str, object]:
    """Enrich F-16 telemetry with portable reduced-order vectors."""

    values = dict(row)
    attitude_keys = ("route_bank_achieved_deg", "route_pitch_achieved_deg", "route_heading_achieved_deg")
    rate_keys = ("p_rad_s", "q_rad_s", "r_rad_s")
    if all(key in values for key in attitude_keys):
        values["attitude_euler_deg"] = [values[key] for key in attitude_keys]
    if all(key in values for key in rate_keys):
        values["body_rate_rad_s"] = [values[key] for key in rate_keys]
    values["control_realization"] = "response_law" if values.get("control_path") == "pseudo_6dof_kinematic_bridge" else "force_model"
    return values
    ####


def _action_frame(payload: Mapping[str, object]) -> ActionFrame:
    return ActionFrame(
        _text(payload, "interface_id"),
        _text(payload, "interface_fingerprint_sha256"),
        _text(payload, "authority_profile_id"),
        _mapping(payload.get("values"), "action_frame.values"),
        _finite(payload.get("duration_s"), "action_frame.duration_s"),
    )
    ####


def _mismatches(expected: Mapping[str, object], actual: Mapping[str, object], prefix: str) -> list[str]:
    result: list[str] = []
    if set(expected) != set(actual):
        result.append(f"{prefix} channel keys differ")
    for key in sorted(set(expected) & set(actual)):
        if not _same(expected[key], actual[key]):
            result.append(f"{prefix}.{key} differs")
    return result
    ####


def _same(left: object, right: object) -> bool:
    if isinstance(left, bool) or isinstance(right, bool):
        return left is right
    if isinstance(left, int | float) and isinstance(right, int | float):
        return math.isclose(float(left), float(right), abs_tol=_TOLERANCE, rel_tol=0.0)
    if isinstance(left, Sequence) and not isinstance(left, str | bytes) and isinstance(right, Sequence) and not isinstance(right, str | bytes):
        return len(left) == len(right) and all(_same(a, b) for a, b in zip(left, right, strict=True))
    return left == right
    ####


def _mapping(value: object, label: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value
    ####


def _sequence(value: object, label: str) -> Sequence[object]:
    if not isinstance(value, Sequence) or isinstance(value, str | bytes):
        raise ValueError(f"{label} must be an ordered sequence")
    return value
    ####


def _text(payload: Mapping[str, object], key: str) -> str:
    value = payload.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"{key} must be a non-empty string")
    return value
    ####


def _finite(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"{label} must be finite numeric")
    return float(value)
    ####


__all__ = [
    "F16ReducedBatchEpisodeParityReport",
    "F16ReducedBatchEpisodeParityStep",
    "verify_serialized_f16_reduced_batch_episode_parity",
]
