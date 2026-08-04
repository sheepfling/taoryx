"""Exact declared action-trace parity between a batch kernel and an episode.

This is deliberately narrower than trajectory or qualification parity.  A
family earns an action-trace parity report only when its batch replay uses the
same declared semantic frames, response model, action limits, and committed
truth/sensor boundaries as its public episode.  It is a fail-closed extension
point: no neighboring-family or approximate re-simulation fallback exists.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

from .committed_boundary_sensor import CommittedBoundarySensor
from .composition_policy import CompositionPolicyTrace
from .trajectory import HummingbirdPseudo6DOFCommand, HummingbirdPseudo6DOFModel, HummingbirdPseudo6DOFState
from .vehicle_composition import CompiledVehicleComposition, resolve_vehicle_composition_interface_contract
from .vehicle_execution_bindings import resolve_vehicle_execution_binding
from .vehicle_interface import VehicleInterfaceContract, project_committed_status_values, validate_projected_status_values

_TOLERANCE = 1.0e-12


@dataclass(frozen=True, slots=True)
class BatchEpisodeParityStep:
    """One public action boundary compared against independent batch replay."""

    index: int
    time_start_s: float
    time_end_s: float
    status: str
    mismatches: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        """Return a portable comparison row."""

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
    """A precise action-trace parity result for one immutable composition."""

    adapter_id: str
    composition_id: str
    composition_identity_sha256: str
    interface_id: str
    interface_fingerprint_sha256: str
    batch_factory_id: str
    episode_authority_profile_id: str
    integration_step_s: float
    status: str
    steps: tuple[BatchEpisodeParityStep, ...]

    def as_dict(self) -> dict[str, object]:
        """Return the report without turning parity into qualification evidence."""

        return {
            "schema": "taoryx.composition-batch-episode-parity/v1alpha1",
            "adapter_id": self.adapter_id,
            "composition_id": self.composition_id,
            "composition_identity_sha256": self.composition_identity_sha256,
            "interface_id": self.interface_id,
            "interface_fingerprint_sha256": self.interface_fingerprint_sha256,
            "batch_factory_id": self.batch_factory_id,
            "episode_authority_profile_id": self.episode_authority_profile_id,
            "integration_step_s": self.integration_step_s,
            "status": self.status,
            "steps": [step.as_dict() for step in self.steps],
            "claim_boundary": (
                "This report compares one declared semantic action trace at committed truth boundaries through "
                "the family-owned batch and episode paths. It does not establish a full mission run, independent "
                "truth-objective pass, physical motor allocation, robustness, or qualification."
            ),
        }
        ####
    ####


def verify_composition_batch_episode_parity(
    composition: CompiledVehicleComposition,
    trace: CompositionPolicyTrace,
) -> BatchEpisodeParityReport:
    """Replay an episode action trace through the exact supported batch kernel.

    The first adapter is intentionally Hummingbird-only.  It retains the
    aggregate thrust-vector pseudo-6DOF evidence boundary and requires both
    declared factories to be runnable.  Future family adapters must opt in
    rather than being approximated by this model.
    """

    if composition.family_id != "hummingbird" or composition.fidelity != "pseudo_6dof":
        raise ValueError("no batch/episode action-trace parity adapter is registered for this composition")
    if composition.mission != "multirotor_pad_box_yaw_recovery_land_v1":
        raise ValueError("Hummingbird parity requires the declared multirotor mission")
    if trace.composition_id != composition.id or trace.composition_identity_sha256 != composition.identity_sha256:
        raise ValueError("policy trace composition identity disagrees with the requested batch parity composition")
    if trace.authority_profile_id != "body_motion_response":
        raise ValueError("Hummingbird parity requires the body_motion_response authority profile")
    if trace.integration_step_s is None:
        raise ValueError("Hummingbird parity requires a trace with declared integration_step_s")
    if trace.integration_step_s <= 0.0 or not math.isfinite(trace.integration_step_s):
        raise ValueError("Hummingbird parity trace integration_step_s must be positive and finite")

    batch_binding = resolve_vehicle_execution_binding(composition, "batch")
    episode_binding = resolve_vehicle_execution_binding(composition, "episode")
    if batch_binding.factory_id != "hummingbird_aggregate_thrust_pseudo_batch.v1":
        raise ValueError("Hummingbird parity does not recognize the selected batch factory")
    if episode_binding.factory_id != "hummingbird_aggregate_thrust_episode.v1":
        raise ValueError("Hummingbird parity does not recognize the selected episode factory")

    contract = resolve_vehicle_composition_interface_contract(composition)
    if trace.interface_id != contract.id or trace.interface_fingerprint_sha256 != contract.fingerprint:
        raise ValueError("policy trace interface identity disagrees with the resolved parity interface")
    model = HummingbirdPseudo6DOFModel(mass_kg=_initial_mass_kg(composition))
    state = _initial_state(composition, model)
    action = _initial_native_action(state, model)
    sensor = _declared_sensor(contract, composition)
    _advance_sensor(sensor, _status_values(contract, state, model, "ready"))

    records: list[BatchEpisodeParityStep] = []
    for index, episode_step in enumerate(trace.steps):
        frame = episode_step.action_frame
        if frame is None:
            raise ValueError(f"policy trace step {index} has no semantic action frame")
        if frame.authority_profile_id != trace.authority_profile_id:
            raise ValueError(f"policy trace step {index} uses a different authority profile")
        if abs(episode_step.time_start_s - state.time_s) > _TOLERANCE:
            raise ValueError(f"policy trace step {index} does not begin at the previous committed truth boundary")
        action = _apply_semantic_action(action, frame.values)
        state = _advance_hummingbird_batch(
            state,
            action,
            model,
            contract,
            sensor,
            frame.duration_s,
            trace.integration_step_s,
        )
        actual_status = _status_values(contract, state, model, "active")
        expected_status = episode_step.status_frame
        if expected_status is None:
            raise ValueError(f"policy trace step {index} has no committed status frame")
        mismatches = _payload_mismatches(expected_status.values, actual_status, "status")
        if abs(episode_step.time_end_s - state.time_s) > _TOLERANCE:
            mismatches.append("time_end_s differs from batch committed boundary")
        records.append(
            BatchEpisodeParityStep(
                index=index,
                time_start_s=episode_step.time_start_s,
                time_end_s=state.time_s,
                status="pass" if not mismatches else "fail",
                mismatches=tuple(mismatches),
            )
        )

    final_mismatches = _payload_mismatches(trace.final_status.values, _status_values(contract, state, model, "active"), "final_status")
    if final_mismatches:
        records.append(
            BatchEpisodeParityStep(
                index=len(records),
                time_start_s=state.time_s,
                time_end_s=state.time_s,
                status="fail",
                mismatches=tuple(final_mismatches),
            )
        )
    return BatchEpisodeParityReport(
        adapter_id="taoryx.hummingbird.aggregate_thrust_batch_episode_parity.v1",
        composition_id=composition.id,
        composition_identity_sha256=composition.identity_sha256,
        interface_id=contract.id,
        interface_fingerprint_sha256=contract.fingerprint,
        batch_factory_id=batch_binding.factory_id,
        episode_authority_profile_id=trace.authority_profile_id,
        integration_step_s=trace.integration_step_s,
        status="pass" if all(item.status == "pass" for item in records) else "fail",
        steps=tuple(records),
    )
    ####


def _initial_mass_kg(composition: CompiledVehicleComposition) -> float:
    if composition.initialization.id != "grounded_idle":
        return 0.5
    value = composition.initialization.inputs.get("mass_kg")
    mass_kg = 0.5 if value is None else float(value.value)
    if not math.isfinite(mass_kg) or mass_kg <= 0.0:
        raise ValueError("Hummingbird parity initial mass must be positive and finite")
    return mass_kg
    ####


def _initial_state(composition: CompiledVehicleComposition, model: HummingbirdPseudo6DOFModel) -> HummingbirdPseudo6DOFState:
    values = composition.initialization.inputs
    altitude_m = _input_number(values, "altitude_m", default=0.0)
    north_m = _input_number(values, "north_m", default=_input_number(values, "pad_north_m", default=0.0))
    east_m = _input_number(values, "east_m", default=_input_number(values, "pad_east_m", default=0.0))
    heading_rad = math.radians(_input_number(values, "heading_deg", default=0.0))
    state = model.initial_state(altitude_m=altitude_m)
    return HummingbirdPseudo6DOFState(
        state.time_s,
        (north_m, east_m, altitude_m),
        state.velocity_m_s,
        (0.0, 0.0, heading_rad),
        state.attitude_rate_rad_s,
        state.battery_fraction,
        state.thrust_n,
        altitude_m == 0.0,
    )
    ####


def _input_number(values: Mapping[str, object], identifier: str, *, default: float) -> float:
    value = values.get(identifier)
    if value is None:
        return default
    raw = getattr(value, "value", None)
    if isinstance(raw, bool) or not isinstance(raw, int | float) or not math.isfinite(float(raw)):
        raise ValueError(f"Hummingbird parity initialization {identifier!r} must be finite numeric")
    return float(raw)
    ####


def _initial_native_action(state: HummingbirdPseudo6DOFState, model: HummingbirdPseudo6DOFModel) -> dict[str, object]:
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
    return _bounded_native_action(action)
    ####


def _bounded_native_action(action: Mapping[str, object]) -> dict[str, object]:
    result: dict[str, object] = {}
    for identifier, limit in (("roll_rad", math.pi / 2.0), ("pitch_rad", math.pi / 2.0), ("yaw_rad", math.pi)):
        value = _finite_number(action.get(identifier), identifier)
        result[identifier] = max(-limit, min(limit, value))
    result["thrust_ratio"] = max(0.0, min(1.0, _finite_number(action.get("thrust_ratio"), "thrust_ratio")))
    motors = action.get("motors_enabled")
    if not isinstance(motors, bool):
        raise ValueError("Hummingbird parity motors_enabled must be boolean")
    result["motors_enabled"] = motors
    return result
    ####


def _advance_hummingbird_batch(
    state: HummingbirdPseudo6DOFState,
    action: Mapping[str, object],
    model: HummingbirdPseudo6DOFModel,
    contract: VehicleInterfaceContract,
    sensor: CommittedBoundarySensor | None,
    duration_s: float,
    integration_step_s: float,
) -> HummingbirdPseudo6DOFState:
    remaining = duration_s
    while remaining > _TOLERANCE:
        upper_time_s = state.time_s + remaining
        boundary = None if sensor is None else sensor.next_required_boundary(state.time_s, upper_time_s)
        step_s = min(
            integration_step_s,
            remaining,
            remaining if boundary is None else max(0.0, boundary - state.time_s),
        )
        if step_s <= _TOLERANCE:
            _advance_sensor(sensor, _status_values(contract, state, model, "active"))
            continue
        command = HummingbirdPseudo6DOFCommand(
            roll_rad=_finite_number(action.get("roll_rad"), "roll_rad"),
            pitch_rad=_finite_number(action.get("pitch_rad"), "pitch_rad"),
            yaw_rad=_finite_number(action.get("yaw_rad"), "yaw_rad"),
            thrust_ratio=_finite_number(action.get("thrust_ratio"), "thrust_ratio"),
            motors_enabled=bool(action.get("motors_enabled")),
            thrust_frame="body_euler",
        )
        state, _ = model.step(state, command, step_s)
        remaining -= step_s
        _advance_sensor(sensor, _status_values(contract, state, model, "active"))
    return state
    ####


def _declared_sensor(
    contract: VehicleInterfaceContract,
    composition: CompiledVehicleComposition,
) -> CommittedBoundarySensor | None:
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


def _advance_sensor(sensor: CommittedBoundarySensor | None, status_values: Mapping[str, object]) -> None:
    if sensor is not None:
        time_s = status_values.get("execution.time")
        if isinstance(time_s, bool) or not isinstance(time_s, int | float):
            raise ValueError("Hummingbird parity status has invalid committed time")
        sensor.advance(float(time_s), status_values)
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
        "aggregate_thrust_n": state.thrust_n,
        "contact": state.contact,
        "response_profile_id": model.profile_id,
        "control_realization": "aggregate_thrust_vector_surrogate",
        "physical_motor_allocation": False,
    }
    values = project_committed_status_values(
        contract,
        time_s=state.time_s,
        execution_status=execution_status,
        raw_values=raw,
    )
    validate_projected_status_values(contract, values, context=f"Hummingbird batch parity t={state.time_s:.12g} s")
    return values
    ####


def _finite_number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"Hummingbird parity {label} must be finite numeric")
    return float(value)
    ####


def _payload_mismatches(expected: object, actual: object, path: str) -> list[str]:
    if isinstance(expected, Mapping) and isinstance(actual, Mapping):
        mismatches: list[str] = []
        if set(expected) != set(actual):
            mismatches.append(f"{path}: key set differs")
        for key in sorted(set(expected) & set(actual)):
            mismatches.extend(_payload_mismatches(expected[key], actual[key], f"{path}.{key}"))
        return mismatches
    if isinstance(expected, list | tuple) and isinstance(actual, list | tuple):
        if len(expected) != len(actual):
            return [f"{path}: sequence length differs"]
        return [
            mismatch
            for index, (left, right) in enumerate(zip(expected, actual, strict=True))
            for mismatch in _payload_mismatches(left, right, f"{path}[{index}]")
        ]
    if isinstance(expected, int | float) and not isinstance(expected, bool) and isinstance(actual, int | float) and not isinstance(actual, bool):
        return [] if math.isclose(float(expected), float(actual), abs_tol=_TOLERANCE, rel_tol=0.0) else [f"{path}: numeric value differs"]
    return [] if expected == actual else [f"{path}: value differs"]
    ####


__all__ = [
    "BatchEpisodeParityReport",
    "BatchEpisodeParityStep",
    "verify_composition_batch_episode_parity",
]
