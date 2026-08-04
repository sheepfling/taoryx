"""Committed-boundary action-trace parity for the A320 reduced-flight stepper.

This is intentionally an exact A320 witness.  It constructs a fresh
source-owned stepper rather than reusing an episode object or accepting a
neighboring fixed-wing plant.  It proves kernel/status parity only; the
underlying reduced fidelity still has its declared nonclaims.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from .composition_episode import (
    ActionFrame,
    EpisodeObservation,
    StatusFrame,
    _a320_guidance_override,
    _a320_status_values,
    _a320_stepper_for_composition,
    _native_action_for_frame,
    _status_frame,
)
from .trajectory import A320RacetrackStepper
from .vehicle_composition import CompiledVehicleComposition, resolve_vehicle_composition_interface_contract
from .vehicle_execution_bindings import batch_episode_parity_record, resolve_vehicle_execution_binding
from .vehicle_interface import VehicleInterfaceContract

_ADAPTER_ID = "taoryx.reduced_fixed_wing.a320_action_trace_batch_episode_parity.v1"
_TOLERANCE = 1.0e-12


@dataclass(frozen=True, slots=True)
class ReducedFixedWingBatchEpisodeParityStep:
    """One action-boundary comparison through a fresh reduced batch stepper."""

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
class ReducedFixedWingBatchEpisodeParityReport:
    """One A320 action-trace kernel/status parity artifact."""

    composition_id: str
    composition_identity_sha256: str
    interface_id: str
    interface_fingerprint_sha256: str
    batch_factory_id: str
    authority_profile_id: str
    status: str
    steps: tuple[ReducedFixedWingBatchEpisodeParityStep, ...]

    def as_dict(self) -> dict[str, object]:
        return {
            "schema": "taoryx.composition-batch-episode-parity/v1alpha1",
            "adapter_id": _ADAPTER_ID,
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
                "This compares one semantic kinematic-guidance trace through separately constructed A320 reduced "
                "batch-stepper and episode paths at committed boundaries. It is kernel/status parity only—not "
                "physical control-surface realization, moment balance, robustness, mission qualification, or a "
                "generic fixed-wing parity assertion."
            ),
        }
        ####
    ####


def verify_serialized_reduced_fixed_wing_batch_episode_parity(
    composition: CompiledVehicleComposition,
    payload: Mapping[str, object],
) -> ReducedFixedWingBatchEpisodeParityReport:
    """Replay one persisted A320 semantic trace through a fresh batch stepper."""

    _validate_identity(composition, payload)
    contract = resolve_vehicle_composition_interface_contract(composition)
    if _text(payload, "interface_id") != contract.id or _text(payload, "interface_fingerprint_sha256") != contract.fingerprint:
        raise ValueError("policy trace interface identity disagrees with the parity composition")
    authority_profile_id = _text(payload, "authority_profile_id")
    if authority_profile_id != "kinematic_guidance":
        raise ValueError("A320 reduced parity requires the kinematic_guidance authority profile")
    batch_binding = resolve_vehicle_execution_binding(composition, "batch")
    episode_binding = resolve_vehicle_execution_binding(composition, "episode")
    if batch_binding.factory_id != "reduced_fixed_wing_openap.v1":
        raise ValueError("A320 reduced parity does not recognize the selected batch factory")
    if episode_binding.factory_id != "reduced_fixed_wing_a320_episode.v1":
        raise ValueError("A320 reduced parity does not recognize the selected episode factory")

    stepper = _a320_stepper_for_composition(composition)
    held_native: dict[str, float] = {}
    records: list[ReducedFixedWingBatchEpisodeParityStep] = []
    for index, value in enumerate(_sequence(payload.get("steps"), "steps")):
        expected = _mapping(value, f"steps[{index}]")
        frame = _action_frame(_mapping(expected.get("action_frame"), f"steps[{index}].action_frame"))
        if frame.authority_profile_id != authority_profile_id:
            raise ValueError(f"policy trace step {index} has no compatible semantic action frame")
        start_time = _finite(expected.get("time_start_s"), f"steps[{index}].time_start_s")
        if abs(start_time - stepper.state.time_s) > _TOLERANCE:
            raise ValueError(f"policy trace step {index} begins outside the previous committed boundary")
        native = _native_action_for_frame(contract, frame)
        held_native.update({name: _finite(number, f"native action {name}") for name, number in native.items()})
        stepper.step(frame.duration_s, _a320_guidance_override(held_native))
        actual = _status(stepper, contract)
        expected_status = _mapping(expected.get("status_frame"), f"steps[{index}].status_frame")
        mismatches = _mismatches(
            _mapping(expected_status.get("values"), f"steps[{index}].status_frame.values"),
            actual.values,
            "status",
        )
        expected_end = _finite(expected.get("time_end_s"), f"steps[{index}].time_end_s")
        if abs(expected_end - actual.time_s) > _TOLERANCE:
            mismatches.append("time_end_s differs from batch committed boundary")
        records.append(
            ReducedFixedWingBatchEpisodeParityStep(
                index,
                start_time,
                actual.time_s,
                "pass" if not mismatches else "fail",
                tuple(mismatches),
            )
        )
    actual_final = _status(stepper, contract)
    expected_final = _mapping(payload.get("final_status"), "final_status")
    final_mismatches = _mismatches(
        _mapping(expected_final.get("values"), "final_status.values"),
        actual_final.values,
        "final_status",
    )
    if final_mismatches:
        records.append(
            ReducedFixedWingBatchEpisodeParityStep(
                len(records), actual_final.time_s, actual_final.time_s, "fail", tuple(final_mismatches)
            )
        )
    return ReducedFixedWingBatchEpisodeParityReport(
        composition.id,
        composition.identity_sha256,
        contract.id,
        contract.fingerprint,
        batch_binding.factory_id,
        authority_profile_id,
        "pass" if all(record.status == "pass" for record in records) else "fail",
        tuple(records),
    )
    ####


def _validate_identity(composition: CompiledVehicleComposition, payload: Mapping[str, object]) -> None:
    """Reject traces outside an exact registered A320 composition witness."""

    if _text(payload, "schema") != "taoryx.composition-policy-trace/v1alpha1":
        raise ValueError("unsupported policy trace schema")
    if (
        composition.family_id != "a320_openap_3dof"
        or composition.mission != "powered_fixed_wing_racetrack_v1"
        or composition.fidelity not in {"point_mass_3dof", "pseudo_6dof"}
    ):
        raise ValueError("no A320 reduced batch/episode parity adapter is registered for this composition")
    parity = batch_episode_parity_record(composition.family_id, composition.mission, composition.fidelity)
    if parity.get("availability") != "registered" or parity.get("adapter_id") != _ADAPTER_ID:
        raise ValueError("no A320 reduced batch/episode parity adapter is registered for this composition")
    if _text(payload, "composition_id") != composition.id or _text(payload, "composition_identity_sha256") != composition.identity_sha256:
        raise ValueError("policy trace composition identity disagrees with the requested parity composition")
    ####


def _status(stepper: A320RacetrackStepper, contract: VehicleInterfaceContract) -> StatusFrame:
    """Project fresh A320 stepper truth through the public status contract."""

    row = stepper.current_row(_a320_guidance_override({}))
    state = stepper.state
    lifecycle = "completed" if stepper.completed else "active"
    return _status_frame(contract, EpisodeObservation(state.time_s, _a320_status_values(row), lifecycle))
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
    mismatches: list[str] = []
    if set(expected) != set(actual):
        mismatches.append(f"{prefix} channel keys differ")
    for key in sorted(set(expected) & set(actual)):
        if not _same(expected[key], actual[key]):
            mismatches.append(f"{prefix}.{key} differs")
    return mismatches
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
    "ReducedFixedWingBatchEpisodeParityReport",
    "ReducedFixedWingBatchEpisodeParityStep",
    "verify_serialized_reduced_fixed_wing_batch_episode_parity",
]
