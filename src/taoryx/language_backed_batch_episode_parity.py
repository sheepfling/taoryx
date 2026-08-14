"""Committed-boundary action-trace parity for language-backed fixed-wing paths.

The language runtime has an autonomous batch mission runner and an accepted
truth interactive wrapper.  This module supplies the missing third seam: a
fresh source-materialized batch session that replays an externally produced
semantic action trace.  It is deliberately scoped to the one registered X8
composition; no fixed-wing family fallback is allowed.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import cast

from .committed_boundary_sensor import CommittedBoundarySensor
from .composition_episode import (
    ActionFrame,
    EpisodeObservation,
    StatusFrame,
    _compile_language_backed_scenario,
    _episode_status,
    _native_action_for_frame,
    _next_sensor_boundary,
    _sensor_for_contract,
    _status_frame,
)
from .language_backed_racetrack import LanguageBackedRacetrackAssets
from .plugins import PluginCatalog
from .runtime.interactive import InteractiveSession, InteractiveStatus
from .vehicle_composition import CompiledVehicleComposition, resolve_vehicle_composition_interface_contract
from .vehicle_execution_bindings import batch_episode_parity_record, resolve_vehicle_execution_binding
from .vehicle_interface import VehicleInterfaceContract

_ADAPTER_ID = "taoryx.language_backed.action_trace_batch_episode_parity.v1"
_TOLERANCE = 1.0e-12


@dataclass(frozen=True, slots=True)
class LanguageBackedBatchEpisodeParityStep:
    """One trace boundary independently replayed through a batch session."""

    index: int
    time_start_s: float
    time_end_s: float
    status: str
    mismatches: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        """Return a portable step comparison record."""

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
class LanguageBackedBatchEpisodeParityReport:
    """One source-runtime kernel/status parity result—not a mission verdict."""

    composition_id: str
    composition_identity_sha256: str
    interface_id: str
    interface_fingerprint_sha256: str
    batch_factory_id: str
    authority_profile_id: str
    status: str
    steps: tuple[LanguageBackedBatchEpisodeParityStep, ...]

    def as_dict(self) -> dict[str, object]:
        """Return a common parity artifact with narrow evidence boundaries."""

        return {
            "schema": "taoryx.composition-batch-episode-parity/v1alpha1",
            "adapter_id": _ADAPTER_ID,
            "composition_id": self.composition_id,
            "composition_identity_sha256": self.composition_identity_sha256,
            "interface_id": self.interface_id,
            "interface_fingerprint_sha256": self.interface_fingerprint_sha256,
            "batch_factory_id": self.batch_factory_id,
            "episode_authority_profile_id": self.authority_profile_id,
            "integration_step_s": None,
            "status": self.status,
            "steps": [step.as_dict() for step in self.steps],
            "claim_boundary": (
                "This compares one semantic action trace through separately constructed language-runtime batch and "
                "episode sessions at committed status boundaries. Both use the same declared source runtime, so this "
                "is kernel/status parity only—not independent plant validation, physical effector realization, "
                "robustness, or mission qualification."
            ),
        }
        ####
    ####


def verify_serialized_language_backed_batch_episode_parity(
    composition: CompiledVehicleComposition,
    payload: Mapping[str, object],
    *,
    assets: LanguageBackedRacetrackAssets | None = None,
    plugins: PluginCatalog | None = None,
) -> LanguageBackedBatchEpisodeParityReport:
    """Replay one registered public trace through a fresh language-runtime session."""

    _validate_identity(composition, payload)
    contract = resolve_vehicle_composition_interface_contract(composition)
    if _text(payload, "interface_id") != contract.id or _text(payload, "interface_fingerprint_sha256") != contract.fingerprint:
        raise ValueError("policy trace interface identity disagrees with the parity composition")
    authority_profile_id = _text(payload, "authority_profile_id")
    if authority_profile_id != "native_control_bridge":
        raise ValueError("language-backed fixed-wing parity requires the native_control_bridge authority profile")
    batch_binding = resolve_vehicle_execution_binding(composition, "batch")
    episode_binding = resolve_vehicle_execution_binding(composition, "episode")
    if batch_binding.factory_id != "language_backed_powered_fixed_wing.v1":
        raise ValueError("language-backed fixed-wing parity does not recognize the selected batch factory")
    if episode_binding.factory_id != "language_backed_interactive.v1":
        raise ValueError("language-backed fixed-wing parity does not recognize the selected episode factory")

    session = _compile_language_backed_scenario(
        composition,
        seed=None,
        assets=assets,
        plugins=plugins,
    ).interactive_session()
    sensor = _sensor_for_contract(contract, seed=None)
    if sensor is not None:
        initial = _status(session, contract)
        sensor.advance(initial.time_s, initial.values)
    records: list[LanguageBackedBatchEpisodeParityStep] = []
    for index, value in enumerate(_sequence(payload.get("steps"), "steps")):
        expected = _mapping(value, f"steps[{index}]")
        frame = _action_frame(_mapping(expected.get("action_frame"), f"steps[{index}].action_frame"))
        if frame.authority_profile_id != authority_profile_id:
            raise ValueError(f"policy trace step {index} has no compatible semantic action frame")
        start_time = _finite(expected.get("time_start_s"), f"steps[{index}].time_start_s")
        if abs(start_time - session.time) > _TOLERANCE:
            raise ValueError(f"policy trace step {index} begins outside the previous committed boundary")
        native_action = _native_action_for_frame(contract, frame)
        _advance(session, native_action, frame.duration_s, sensor, contract)
        actual = _status(session, contract)
        expected_status = _mapping(expected.get("status_frame"), f"steps[{index}].status_frame")
        mismatches = _mismatches(_mapping(expected_status.get("values"), f"steps[{index}].status_frame.values"), actual.values, "status")
        expected_end = _finite(expected.get("time_end_s"), f"steps[{index}].time_end_s")
        if abs(expected_end - actual.time_s) > _TOLERANCE:
            mismatches.append("time_end_s differs from batch committed boundary")
        records.append(
            LanguageBackedBatchEpisodeParityStep(
                index,
                start_time,
                actual.time_s,
                "pass" if not mismatches else "fail",
                tuple(mismatches),
            )
        )
    actual_final = _status(session, contract)
    expected_final = _mapping(payload.get("final_status"), "final_status")
    final_mismatches = _mismatches(_mapping(expected_final.get("values"), "final_status.values"), actual_final.values, "final_status")
    if final_mismatches:
        records.append(
            LanguageBackedBatchEpisodeParityStep(
                len(records), actual_final.time_s, actual_final.time_s, "fail", tuple(final_mismatches)
            )
        )
    return LanguageBackedBatchEpisodeParityReport(
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
    """Reject a trace or composition outside a declared language-backed witness."""

    if _text(payload, "schema") != "taoryx.composition-policy-trace/v1alpha1":
        raise ValueError("unsupported policy trace schema")
    if (
        composition.family_id not in {"skywalker_x8", "b747"}
        or composition.mission != "powered_fixed_wing_racetrack_v1"
        or composition.fidelity not in {"point_mass_3dof", "pseudo_6dof"}
    ):
        raise ValueError("no language-backed batch/episode parity adapter is registered for this composition")
    parity = batch_episode_parity_record(composition.family_id, composition.mission, composition.fidelity)
    if parity.get("availability") != "registered" or parity.get("adapter_id") != _ADAPTER_ID:
        raise ValueError("no language-backed batch/episode parity adapter is registered for this composition")
    if _text(payload, "composition_id") != composition.id or _text(payload, "composition_identity_sha256") != composition.identity_sha256:
        raise ValueError("policy trace composition identity disagrees with the requested parity composition")
    ####


def _advance(
    session: InteractiveSession,
    native_action: Mapping[str, object],
    duration_s: float,
    sensor: CommittedBoundarySensor | None,
    contract: VehicleInterfaceContract,
) -> None:
    """Hold a semantic action while respecting the same truth boundaries as an episode."""

    commands = {name: _finite(value, f"native action {name}") for name, value in native_action.items()}
    target_time = session.time + duration_s
    while session.time < target_time - _TOLERANCE:
        boundary = _next_sensor_boundary(sensor, session.time, target_time)
        interval = (boundary if boundary is not None else target_time) - session.time
        session.step(interval, commands)
        if sensor is not None:
            status = _status(session, contract)
            sensor.advance(status.time_s, status.values)
        if session.status in {InteractiveStatus.COMPLETED, InteractiveStatus.FAILED}:
            break
    ####


def _status(session: InteractiveSession, contract: VehicleInterfaceContract) -> StatusFrame:
    """Project fresh batch-session truth through the public status schema."""

    values: dict[str, object] = {}
    for vehicle in session.problem.vehicles.values():
        values[vehicle.name] = {
            **dict(zip(vehicle.state.value_names, vehicle.state.values, strict=False)),
            **dict(vehicle.state.named),
        }
    observation = EpisodeObservation(session.time, values, _episode_status(session.status, False))
    return _status_frame(contract, observation)
    ####


def _action_frame(payload: Mapping[str, object]) -> ActionFrame:
    """Read a semantic action without trusting its prior episode result."""

    return ActionFrame(
        _text(payload, "interface_id"),
        _text(payload, "interface_fingerprint_sha256"),
        _text(payload, "authority_profile_id"),
        _mapping(payload.get("values"), "action_frame.values"),
        _finite(payload.get("duration_s"), "action_frame.duration_s"),
    )
    ####


def _finite(value: object, label: str) -> float:
    """Read one finite scalar from untrusted trace evidence."""

    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"language-backed parity {label} must be finite numeric")
    return float(value)
    ####


def _mapping(value: object, label: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"language-backed parity {label} must be an object")
    return cast(Mapping[str, object], value)
    ####


def _sequence(value: object, label: str) -> tuple[object, ...]:
    if not isinstance(value, list):
        raise ValueError(f"language-backed parity {label} must be an array")
    return tuple(value)
    ####


def _text(payload: Mapping[str, object], key: str) -> str:
    value = payload.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"language-backed parity {key} must be nonempty text")
    return value
    ####


def _mismatches(expected: object, actual: object, path: str) -> list[str]:
    if isinstance(expected, Mapping) and isinstance(actual, Mapping):
        result = [f"{path}: key set differs"] if set(expected) != set(actual) else []
        for key in sorted(set(expected) & set(actual)):
            result.extend(_mismatches(expected[key], actual[key], f"{path}.{key}"))
        return result
    if isinstance(expected, list | tuple) and isinstance(actual, list | tuple):
        if len(expected) != len(actual):
            return [f"{path}: sequence length differs"]
        return [mismatch for index, pair in enumerate(zip(expected, actual, strict=True)) for mismatch in _mismatches(pair[0], pair[1], f"{path}[{index}]")]
    if isinstance(expected, int | float) and not isinstance(expected, bool) and isinstance(actual, int | float) and not isinstance(actual, bool):
        return [] if math.isclose(float(expected), float(actual), rel_tol=0.0, abs_tol=_TOLERANCE) else [f"{path}: numeric value differs"]
    return [] if expected == actual else [f"{path}: value differs"]
    ####


__all__ = ["LanguageBackedBatchEpisodeParityReport", "LanguageBackedBatchEpisodeParityStep", "verify_serialized_language_backed_batch_episode_parity"]
