"""Declared Hummingbird batch/episode action-trace parity."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import cast

from .committed_boundary_sensor import CommittedBoundarySensor
from .composition_policy import CompositionPolicyTrace, parse_composition_policy_trace_record
from .trajectory.hummingbird_pseudo6dof import HummingbirdPseudo6DOFCommand, HummingbirdPseudo6DOFModel, HummingbirdPseudo6DOFState
from .vehicle_composition import CompiledVehicleComposition, resolve_vehicle_composition_interface_contract
from .vehicle_execution_bindings import batch_episode_parity_record, resolve_vehicle_execution_binding
from .vehicle_interface import (
    CommittedNativeStatusValues,
    VehicleInterfaceContract,
    project_committed_status_values,
    validate_projected_status_values,
)

_TOLERANCE = 1.0e-12
_ADAPTER_ID = "taoryx.hummingbird.aggregate_thrust_batch_episode_parity.v1"


@dataclass(frozen=True, slots=True)
class BatchEpisodeParityStep:
    """One episode action boundary compared against batch propagation."""

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
class BatchEpisodeParityReport:
    """Immutable, scoped parity evidence—not a mission or qualification result."""

    composition_id: str
    composition_identity_sha256: str
    interface_id: str
    interface_fingerprint_sha256: str
    batch_factory_id: str
    authority_profile_id: str
    integration_step_s: float
    status: str
    steps: tuple[BatchEpisodeParityStep, ...]

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
            "integration_step_s": self.integration_step_s,
            "status": self.status,
            "steps": [step.as_dict() for step in self.steps],
            "claim_boundary": (
                "This compares one declared semantic action trace through the family-owned batch and episode "
                "paths at committed truth boundaries. It is not a mission run, physical motor allocation, "
                "robustness, or qualification result."
            ),
        }
        ####

    ####


def verify_composition_batch_episode_parity(
    composition: CompiledVehicleComposition,
    trace: CompositionPolicyTrace,
) -> BatchEpisodeParityReport:
    """Replay a Hummingbird episode trace through a separate batch loop.

    No generic fallback is permitted: the composition, declared factories,
    authority profile, interface identity, integration step, and every action
    frame must all match the single source-owned Hummingbird witness.
    """

    return verify_serialized_composition_batch_episode_parity(composition, trace.as_dict())
    ####


def verify_serialized_composition_batch_episode_parity(
    composition: CompiledVehicleComposition,
    payload: object,
) -> BatchEpisodeParityReport:
    """Compare a persisted semantic trace through the registered batch path.

    The serialized trace remains untrusted expected evidence: this function
    replays each declared semantic frame through a separate batch loop and
    compares the newly projected committed status values at every boundary.
    It deliberately supports only one registered family witness; callers do
    not receive a plausible-looking parity result from substituted dynamics.
    """

    trace = parse_composition_policy_trace_record(payload)
    if composition.family_id != "hummingbird" or composition.fidelity != "pseudo_6dof" or composition.mission != "multirotor_pad_box_yaw_recovery_land_v1":
        raise ValueError("no batch/episode action-trace parity adapter is registered for this composition")
    parity = batch_episode_parity_record(composition.family_id, composition.mission, composition.fidelity)
    if parity.availability != "registered" or parity.adapter_id != _ADAPTER_ID:
        raise ValueError("no batch/episode action-trace parity adapter is registered for this composition")
    if trace["composition_id"] != composition.id or trace["composition_identity_sha256"] != composition.identity_sha256:
        raise ValueError("policy trace composition identity disagrees with the requested parity composition")
    authority_profile_id = trace["authority_profile_id"]
    if authority_profile_id != "body_motion_response":
        raise ValueError("Hummingbird parity requires the body_motion_response authority profile")
    integration_step_s = _positive_finite(trace["integration_step_s"], "integration_step_s")

    batch_binding = resolve_vehicle_execution_binding(composition, "batch")
    episode_binding = resolve_vehicle_execution_binding(composition, "episode")
    if batch_binding.factory_id != "hummingbird_aggregate_thrust_pseudo_batch.v1":
        raise ValueError("Hummingbird parity does not recognize the selected batch factory")
    if episode_binding.factory_id != "hummingbird_aggregate_thrust_episode.v1":
        raise ValueError("Hummingbird parity does not recognize the selected episode factory")
    contract = resolve_vehicle_composition_interface_contract(composition)
    if trace["interface_id"] != contract.id or trace["interface_fingerprint_sha256"] != contract.fingerprint:
        raise ValueError("policy trace interface identity disagrees with the parity composition")

    model = HummingbirdPseudo6DOFModel(mass_kg=_initial_mass_kg(composition))
    state = _initial_state(composition, model)
    action = _initial_action(state, model)
    sensor = _sensor(contract, composition)
    _advance_sensor(sensor, _status_values(contract, state, model, "ready"))
    records: list[BatchEpisodeParityStep] = []
    for index, serialized_step in enumerate(trace["steps"]):
        episode_step = _mapping(serialized_step, f"steps[{index}]")
        frame = _mapping(episode_step.get("action_frame"), f"steps[{index}].action_frame")
        if _text(frame, "interface_id") != contract.id or _text(frame, "interface_fingerprint_sha256") != contract.fingerprint:
            raise ValueError(f"policy trace step {index} action frame interface identity disagrees with composition")
        if _text(frame, "authority_profile_id") != authority_profile_id:
            raise ValueError(f"policy trace step {index} has no compatible semantic action frame")
        if abs(_finite(episode_step.get("time_start_s"), f"steps[{index}].time_start_s") - state.time_s) > _TOLERANCE:
            raise ValueError(f"policy trace step {index} begins outside the previous committed boundary")
        action = _apply_semantic_action(action, _mapping(frame.get("values"), f"steps[{index}].action_frame.values"))
        state = _advance_batch(
            state,
            action,
            model,
            contract,
            sensor,
            _positive_finite(frame.get("duration_s"), f"steps[{index}].action_frame.duration_s"),
            integration_step_s,
        )
        expected_status = _mapping(episode_step.get("status_frame"), f"steps[{index}].status_frame")
        mismatches = _mismatches(
            _mapping(expected_status.get("values"), f"steps[{index}].status_frame.values"),
            _status_values(contract, state, model, "active"),
            "status",
        )
        if abs(_finite(episode_step.get("time_end_s"), f"steps[{index}].time_end_s") - state.time_s) > _TOLERANCE:
            mismatches.append("time_end_s differs from batch committed boundary")
        records.append(
            BatchEpisodeParityStep(
                index,
                _finite(episode_step.get("time_start_s"), f"steps[{index}].time_start_s"),
                state.time_s,
                "pass" if not mismatches else "fail",
                tuple(mismatches),
            )
        )
    final_status = trace["final_status"]
    final_mismatches = _mismatches(
        _mapping(final_status.get("values"), "final_status.values"),
        _status_values(contract, state, model, "active"),
        "final_status",
    )
    if final_mismatches:
        records.append(BatchEpisodeParityStep(len(records), state.time_s, state.time_s, "fail", tuple(final_mismatches)))
    return BatchEpisodeParityReport(
        composition.id,
        composition.identity_sha256,
        contract.id,
        contract.fingerprint,
        batch_binding.factory_id,
        authority_profile_id,
        integration_step_s,
        "pass" if all(record.status == "pass" for record in records) else "fail",
        tuple(records),
    )
    ####


def _initial_mass_kg(composition: CompiledVehicleComposition) -> float:
    if composition.initialization.id != "grounded_idle":
        return 0.5
    value = composition.initialization.inputs.get("mass_kg")
    result = 0.5 if value is None else _finite(value.value, "mass_kg")
    if result <= 0.0:
        raise ValueError("Hummingbird parity mass_kg must be positive")
    return result
    ####


def _initial_state(composition: CompiledVehicleComposition, model: HummingbirdPseudo6DOFModel) -> HummingbirdPseudo6DOFState:
    values = composition.initialization.inputs
    altitude = _input(values, "altitude_m", 0.0)
    north = _input(values, "north_m", _input(values, "pad_north_m", 0.0))
    east = _input(values, "east_m", _input(values, "pad_east_m", 0.0))
    heading = math.radians(_input(values, "heading_deg", 0.0))
    state = model.initial_state(altitude_m=altitude)
    return HummingbirdPseudo6DOFState(
        state.time_s,
        (north, east, altitude),
        state.velocity_m_s,
        (0.0, 0.0, heading),
        state.attitude_rate_rad_s,
        state.battery_fraction,
        state.thrust_n,
        altitude == 0.0,
    )
    ####


def _input(values: Mapping[str, object], identifier: str, default: float) -> float:
    value = values.get(identifier)
    return default if value is None else _finite(getattr(value, "value", None), identifier)
    ####


def _initial_action(state: HummingbirdPseudo6DOFState, model: HummingbirdPseudo6DOFModel) -> dict[str, object]:
    return {
        "roll_rad": 0.0,
        "pitch_rad": 0.0,
        "yaw_rad": state.attitude_rad[2],
        "thrust_ratio": model.mass_kg * model.gravity_m_s2 / model.maximum_thrust_n,
        "motors_enabled": True,
    }
    ####


def _apply_semantic_action(previous: Mapping[str, object], values: Mapping[str, object]) -> dict[str, object]:
    mapping = {
        "attitude.roll.command": "roll_rad",
        "attitude.pitch.command": "pitch_rad",
        "attitude.yaw.command": "yaw_rad",
        "propulsion.command.fraction": "thrust_ratio",
        "propulsion.enable": "motors_enabled",
    }
    unknown = sorted(set(values) - set(mapping))
    if unknown:
        raise ValueError("Hummingbird parity trace contains unsupported semantic action(s): " + ", ".join(unknown))
    action = dict(previous)
    for semantic, native in mapping.items():
        if semantic in values:
            action[native] = values[semantic]
    limits = (("roll_rad", math.pi / 2.0), ("pitch_rad", math.pi / 2.0), ("yaw_rad", math.pi))
    for name, limit in limits:
        action[name] = max(-limit, min(limit, _finite(action.get(name), name)))
    action["thrust_ratio"] = max(0.0, min(1.0, _finite(action.get("thrust_ratio"), "thrust_ratio")))
    if not isinstance(action.get("motors_enabled"), bool):
        raise ValueError("Hummingbird parity motors_enabled must be boolean")
    return action
    ####


def _advance_batch(
    state: HummingbirdPseudo6DOFState,
    action: Mapping[str, object],
    model: HummingbirdPseudo6DOFModel,
    contract: VehicleInterfaceContract,
    sensor: CommittedBoundarySensor | None,
    duration_s: float,
    step_s: float,
) -> HummingbirdPseudo6DOFState:
    remaining = duration_s
    while remaining > _TOLERANCE:
        boundary = None if sensor is None else sensor.next_required_boundary(state.time_s, state.time_s + remaining)
        dt_s = min(step_s, remaining, remaining if boundary is None else max(0.0, boundary - state.time_s))
        if dt_s <= _TOLERANCE:
            _advance_sensor(sensor, _status_values(contract, state, model, "active"))
            continue
        state, _ = model.step(
            state,
            HummingbirdPseudo6DOFCommand(
                roll_rad=_finite(action.get("roll_rad"), "roll_rad"),
                pitch_rad=_finite(action.get("pitch_rad"), "pitch_rad"),
                yaw_rad=_finite(action.get("yaw_rad"), "yaw_rad"),
                thrust_ratio=_finite(action.get("thrust_ratio"), "thrust_ratio"),
                motors_enabled=bool(action["motors_enabled"]),
                thrust_frame="body_euler",
            ),
            dt_s,
        )
        remaining -= dt_s
        _advance_sensor(sensor, _status_values(contract, state, model, "active"))
    return state
    ####


def _sensor(contract: VehicleInterfaceContract, composition: CompiledVehicleComposition) -> CommittedBoundarySensor | None:
    profile = contract.observation_profile(composition.observation.profile_id)
    if profile.source != "sensor":
        return None
    if profile.cadence_s is None or profile.latency_s is None:
        raise ValueError("Hummingbird parity sensor profile has incomplete timing")
    return CommittedBoundarySensor(
        profile_id=profile.id,
        channel_ids=profile.channel_ids,
        cadence_s=profile.cadence_s,
        latency_s=profile.latency_s,
        channel_errors=profile.channel_errors,
        seed=0,
    )
    ####


def _advance_sensor(sensor: CommittedBoundarySensor | None, values: Mapping[str, object]) -> None:
    if sensor is not None:
        sensor.advance(_finite(values.get("execution.time"), "execution.time"), values)
    ####


def _status_values(
    contract: VehicleInterfaceContract,
    state: HummingbirdPseudo6DOFState,
    model: HummingbirdPseudo6DOFModel,
    execution_status: str,
) -> dict[str, object]:
    position = state.position_m
    velocity = state.velocity_m_s
    raw = {
        "position_ned_m": [position[0], position[1], -position[2]],
        "velocity_ned_m_s": [velocity[0], velocity[1], -velocity[2]],
        "attitude_rad": list(state.attitude_rad),
        "body_rate_rad_s": list(state.attitude_rate_rad_s),
        "battery_fraction": state.battery_fraction,
        "mass_kg": model.mass_kg,
        "aggregate_thrust_n": state.thrust_n,
        "aggregate_thrust_fraction": state.thrust_n / model.maximum_thrust_n,
        "motors_enabled": state.thrust_n > 0.0,
        "velocity_horizontal_speed_m_s": math.hypot(velocity[0], velocity[1]),
        "contact": state.contact,
        "guidance_waypoint_range_m": 0.0,
        "guidance_waypoint_captured": False,
        "guidance_waypoint_status": "inactive",
        "response_profile_id": model.profile_id,
        "control_realization": "aggregate_thrust_vector_surrogate",
        "controller_method": "attitude_response_law",
        "physical_motor_allocation": False,
    }
    values = project_committed_status_values(
        contract,
        time_s=state.time_s,
        execution_status=execution_status,
        raw_values=CommittedNativeStatusValues(raw),
    )
    validate_projected_status_values(contract, values, context=f"Hummingbird batch parity t={state.time_s:.12g} s")
    return values
    ####


def _finite(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"Hummingbird parity {label} must be finite numeric")
    return float(value)
    ####


def _positive_finite(value: object, label: str) -> float:
    """Read a strict positive number from untrusted persisted evidence."""

    result = _finite(value, label)
    if result <= 0.0:
        raise ValueError(f"Hummingbird parity {label} must be positive")
    return result
    ####


def _mapping(value: object, label: str) -> Mapping[str, object]:
    """Return a JSON object or reject malformed persisted evidence."""

    if not isinstance(value, Mapping):
        raise ValueError(f"Hummingbird parity {label} must be an object")
    return cast(Mapping[str, object], value)
    ####


def _sequence(value: object, label: str) -> tuple[object, ...]:
    """Return a JSON array or reject malformed persisted evidence."""

    if not isinstance(value, list):
        raise ValueError(f"Hummingbird parity {label} must be an array")
    return tuple(value)
    ####


def _text(payload: Mapping[str, object], key: str) -> str:
    """Read a required nonempty text field from persisted evidence."""

    value = payload.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"Hummingbird parity {key} must be nonempty text")
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
        return [mismatch for index, (left, right) in enumerate(zip(expected, actual, strict=True)) for mismatch in _mismatches(left, right, f"{path}[{index}]")]
    if isinstance(expected, int | float) and not isinstance(expected, bool) and isinstance(actual, int | float) and not isinstance(actual, bool):
        return [] if math.isclose(float(expected), float(actual), rel_tol=0.0, abs_tol=_TOLERANCE) else [f"{path}: numeric value differs"]
    return [] if expected == actual else [f"{path}: value differs"]
    ####


__all__ = [
    "BatchEpisodeParityReport",
    "BatchEpisodeParityStep",
    "verify_composition_batch_episode_parity",
    "verify_serialized_composition_batch_episode_parity",
]
