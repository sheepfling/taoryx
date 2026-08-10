"""Shared racetrack execution for the F-16 reduced-fidelity tiers.

The reduced runners are intentionally separate from the rigid-body runner.
They preserve the same route geometry and truth-objective channels, while
making the omitted physics explicit:

* point-mass mode integrates translational kinematics with bounded speed,
  flight-path, and heading response;
* pseudo-6DOF mode adds a named first-order attitude/rate response bridge.

Neither mode creates physical moments or surface activity.  The source plant
is still used for the local force/resource observables and provenance, but the
response law is the executable reduced-model authority.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Literal

from ..racetrack_guidance import RacetrackGuidanceReference, racetrack_reference_at_time
from ..racetrack_template import ResolvedRacetrack
from ..trim import TrimResult
from .f16_reductions import F16AttitudeResponsePseudo6DOFModel, F16PointMass3DOFModel
from .pseudo6dof_profiles import Pseudo6DOFProfile
from .response_laws import AxisResponseState, step_bounded_axis_response

F16ReducedRacetrackMode = Literal["point_mass_3dof", "pseudo_6dof_kinematic_bridge"]
_Model = F16PointMass3DOFModel | F16AttitudeResponsePseudo6DOFModel


@dataclass(frozen=True, slots=True)
class F16ReducedRacetrackRun:
    """Telemetry and execution status for one reduced-fidelity run."""

    mode: F16ReducedRacetrackMode
    rows: tuple[dict[str, float | int | str], ...]
    numerical_valid: bool
    failure: str | None


@dataclass(frozen=True, slots=True)
class F16GuidanceOverride:
    """One held reduced-fidelity F-16 guidance request in SI/radians."""

    speed_m_s: float | None = None
    flight_path_angle_rad: float | None = None
    heading_rad: float | None = None
    bank_angle_rad: float | None = None

    def __post_init__(self) -> None:
        values = (
            self.speed_m_s,
            self.flight_path_angle_rad,
            self.heading_rad,
            self.bank_angle_rad,
        )
        if not all(value is None or math.isfinite(value) for value in values):
            raise ValueError("F-16 guidance overrides must be finite when specified")
        if self.speed_m_s is not None and self.speed_m_s <= 0.0:
            raise ValueError("F-16 guidance speed override must be positive")
        ####
    ####


@dataclass(slots=True)
class F16ReducedRacetrackStepperState:
    """Mutable state owned by :class:`F16ReducedRacetrackStepper` only."""

    time_s: float
    north_m: float
    east_m: float
    altitude_m: float
    speed_m_s: float
    heading_rad: float
    flight_path_angle_rad: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    p_rad_s: float
    q_rad_s: float
    r_rad_s: float
    controls: dict[str, float]
    numerical_valid: bool = True
    failure: str | None = None
    ####


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))
    ####


def _wrap(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
    ####


def _slew(current: float, target: float, time_constant_s: float, dt_s: float) -> float:
    if time_constant_s <= 0.0:
        return target
    return current + (target - current) * _clamp(dt_s / time_constant_s, 0.0, 1.0)
    ####


@dataclass(slots=True)
class F16ReducedRacetrackRunner:
    """Execute the shared route through a declared F-16 reduction."""

    model: _Model
    trim: TrimResult
    route: ResolvedRacetrack
    mode: F16ReducedRacetrackMode
    dt_s: float = 0.5
    speed_time_constant_s: float = 3.0
    heading_time_constant_s: float = 4.0
    flight_path_time_constant_s: float = 3.0
    attitude_time_constant_s: float = 1.5
    maximum_speed_acceleration_mps2: float = 8.0
    maximum_turn_rate_rad_s: float = math.radians(8.0)
    maximum_flight_path_rate_rad_s: float = math.radians(6.0)
    response_profile: Pseudo6DOFProfile | None = None
    initial_north_offset_m: float = 0.0
    initial_east_offset_m: float = 0.0
    initial_altitude_offset_m: float = 0.0
    initial_speed_offset_m_s: float = 0.0
    initial_heading_offset_rad: float = 0.0
    initial_flight_path_offset_rad: float = 0.0
    initial_bank_offset_rad: float = 0.0

    def __post_init__(self) -> None:
        if self.mode == "point_mass_3dof" and not isinstance(self.model, F16PointMass3DOFModel):
            raise ValueError("point-mass mode requires F16PointMass3DOFModel")
        if self.mode == "pseudo_6dof_kinematic_bridge" and not isinstance(
            self.model, F16AttitudeResponsePseudo6DOFModel
        ):
            raise ValueError("pseudo-6DOF mode requires F16AttitudeResponsePseudo6DOFModel")
        if self.response_profile is not None and self.mode != "pseudo_6dof_kinematic_bridge":
            raise ValueError("response profiles apply only to pseudo-6DOF mode")
        if not math.isfinite(self.dt_s) or self.dt_s <= 0.0:
            raise ValueError("reduced F-16 racetrack step must be finite and positive")
        values = (
            self.speed_time_constant_s,
            self.heading_time_constant_s,
            self.flight_path_time_constant_s,
            self.attitude_time_constant_s,
            self.maximum_speed_acceleration_mps2,
            self.maximum_turn_rate_rad_s,
            self.maximum_flight_path_rate_rad_s,
            self.initial_north_offset_m,
            self.initial_east_offset_m,
            self.initial_altitude_offset_m,
            self.initial_speed_offset_m_s,
            self.initial_heading_offset_rad,
            self.initial_flight_path_offset_rad,
            self.initial_bank_offset_rad,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values[:7]):
            raise ValueError("reduced F-16 response parameters must be finite and positive")
        if not all(math.isfinite(value) for value in values[7:]):
            raise ValueError("reduced F-16 initial offsets must be finite")
        ####

    @property
    def trim_pitch_rad(self) -> float:
        """Return the source trim pitch used by both reductions."""

        return math.atan2(float(self.trim.state["w_m_s"]), float(self.trim.state["u_m_s"]))
        ####

    def _reference(
        self,
        time_s: float,
        north_m: float,
        east_m: float,
    ) -> RacetrackGuidanceReference:
        reference = racetrack_reference_at_time(self.route, time_s, trim_pitch_rad=self.trim_pitch_rad)
        if time_s >= self.route.declared_duration_s:
            terminal_heading = math.atan2(-east_m, -north_m)
            reference = replace(
                reference,
                heading_rad=terminal_heading,
                flight_path_angle_rad=0.0,
                bank_rad=0.0,
            )
        return reference
        ####

    def _source_observables(
        self,
        speed_m_s: float,
        flight_path_angle_rad: float,
        controls: dict[str, float],
        altitude_m: float,
    ) -> dict[str, float]:
        body = {
            "u_m_s": speed_m_s * math.cos(flight_path_angle_rad),
            "v_m_s": 0.0,
            "w_m_s": speed_m_s * math.sin(flight_path_angle_rad),
            "p_rad_s": 0.0,
            "q_rad_s": 0.0,
            "r_rad_s": 0.0,
        }
        loads = self.model.source.evaluate_loads(body, controls, altitude_m=altitude_m)
        return {
            "source_force_x_n": float(loads["total_force_x_n"]),
            "source_force_y_n": float(loads["total_force_y_n"]),
            "source_force_z_n": float(loads["total_force_z_n"]),
            "source_dynamic_pressure_pa": float(loads["dynamic_pressure_pa"]),
            "source_mach": float(loads["mach"]),
            "source_alpha_deg": float(loads["alpha_deg"]),
            "source_beta_deg": float(loads["beta_deg"]),
        }
        ####

    def _sample(
        self,
        time_s: float,
        north_m: float,
        east_m: float,
        altitude_m: float,
        speed_m_s: float,
        heading_rad: float,
        flight_path_angle_rad: float,
        reference: RacetrackGuidanceReference,
        source_observables: dict[str, float],
        controls: dict[str, float],
        *,
        roll_rad: float = 0.0,
        pitch_rad: float | None = None,
        yaw_rad: float | None = None,
        p_rad_s: float = 0.0,
        q_rad_s: float = 0.0,
        r_rad_s: float = 0.0,
    ) -> dict[str, float | int | str]:
        actual_pitch = self.trim_pitch_rad + flight_path_angle_rad if pitch_rad is None else pitch_rad
        actual_yaw = heading_rad if yaw_rad is None else yaw_rad
        return {
            "time_s": time_s,
            "north_m": north_m,
            "east_m": east_m,
            "altitude_m": altitude_m,
            "speed_m_s": speed_m_s,
            "mass_kg": self.model.source.mass_kg,
            "phase_index": reference.phase_index,
            "route_leg_index": reference.route_leg_index,
            "phase": reference.phase,
            "route_north_command_m": reference.north_m,
            "route_east_command_m": reference.east_m,
            "route_altitude_command_m": reference.altitude_m,
            "route_speed_command_m_s": reference.speed_m_s,
            "route_heading_command_deg": math.degrees(reference.heading_rad),
            "route_flight_path_command_deg": math.degrees(reference.flight_path_angle_rad),
            "route_bank_command_deg": math.degrees(reference.bank_rad),
            "route_bank_achieved_deg": math.degrees(roll_rad),
            "route_pitch_achieved_deg": math.degrees(actual_pitch),
            "route_heading_achieved_deg": math.degrees(actual_yaw),
            "heading_rad": heading_rad,
            "flight_path_angle_rad": flight_path_angle_rad,
            "p_rad_s": p_rad_s,
            "q_rad_s": q_rad_s,
            "r_rad_s": r_rad_s,
            "source_force_x_n": source_observables["source_force_x_n"],
            "source_force_y_n": source_observables["source_force_y_n"],
            "source_force_z_n": source_observables["source_force_z_n"],
            "dynamic_pressure_pa": source_observables["source_dynamic_pressure_pa"],
            "mach": source_observables["source_mach"],
            "aero_alpha_deg": source_observables["source_alpha_deg"],
            "aero_beta_deg": source_observables["source_beta_deg"],
            "control_path": self.mode,
            "response_profile_id": self.response_profile.id if self.response_profile is not None else "legacy_runner_defaults",
            "elevator_deg": controls["elevator_deg"],
            "aileron_deg": controls["aileron_deg"],
            "rudder_deg": controls["rudder_deg"],
            "throttle_fraction": controls["throttle_fraction"],
            "allocation_status": "reduced_response_law",
            "allocation_residual_norm": 0.0,
            "saturation_count": 0,
        }
        ####

    def run(self, *, duration_s: float | None = None) -> F16ReducedRacetrackRun:
        """Execute a reduced route and return truth-compatible telemetry."""

        horizon = self.route.horizon_s if duration_s is None else float(duration_s)
        if not math.isfinite(horizon) or horizon <= 0.0:
            raise ValueError("reduced F-16 racetrack horizon must be finite and positive")
        controls = dict(self.trim.controls)
        north_m = self.initial_north_offset_m
        east_m = self.initial_east_offset_m
        altitude_m = float(self.route.low_altitude_m) + self.initial_altitude_offset_m
        speed_m_s = max(1.0, float(self.route.speed_m_s) + self.initial_speed_offset_m_s)
        heading_rad = _wrap(math.pi / 2.0 + self.initial_heading_offset_rad)
        flight_path_angle_rad = _clamp(self.initial_flight_path_offset_rad, -0.8, 0.8)
        roll_rad = self.initial_bank_offset_rad if self.mode == "pseudo_6dof_kinematic_bridge" else 0.0
        pitch_rad = self.trim_pitch_rad + flight_path_angle_rad
        yaw_rad = heading_rad
        p_rad_s = q_rad_s = r_rad_s = 0.0
        rows: list[dict[str, float | int | str]] = []
        numerical_valid = True
        failure: str | None = None
        steps = int(math.ceil(horizon / self.dt_s))
        time_s = 0.0
        for _ in range(steps + 1):
            try:
                dt = min(self.dt_s, horizon - time_s)
                if dt <= 0.0:
                    break
                reference = self._reference(time_s, north_m, east_m)
                source_observables = self._source_observables(
                    speed_m_s,
                    flight_path_angle_rad,
                    controls,
                    altitude_m,
                )
                speed_error = reference.speed_m_s - speed_m_s
                speed_acceleration = _clamp(
                    speed_error / self.speed_time_constant_s,
                    -self.maximum_speed_acceleration_mps2,
                    self.maximum_speed_acceleration_mps2,
                )
                speed_next = max(1.0, speed_m_s + speed_acceleration * dt)
                heading_error = _wrap(reference.heading_rad - heading_rad)
                heading_rate = _clamp(
                    heading_error / self.heading_time_constant_s,
                    -self.maximum_turn_rate_rad_s,
                    self.maximum_turn_rate_rad_s,
                )
                gamma_error = reference.flight_path_angle_rad - flight_path_angle_rad
                gamma_rate = _clamp(
                    gamma_error / self.flight_path_time_constant_s,
                    -self.maximum_flight_path_rate_rad_s,
                    self.maximum_flight_path_rate_rad_s,
                )
                heading_next = _wrap(heading_rad + heading_rate * dt)
                gamma_next = _clamp(flight_path_angle_rad + gamma_rate * dt, -0.8, 0.8)
                altitude_next = altitude_m + speed_m_s * math.sin(flight_path_angle_rad) * dt
                north_next = north_m + speed_m_s * math.cos(flight_path_angle_rad) * math.cos(heading_rad) * dt
                east_next = east_m + speed_m_s * math.cos(flight_path_angle_rad) * math.sin(heading_rad) * dt
                controls["throttle_fraction"] = _clamp(
                    float(self.trim.controls["throttle_fraction"]) + 0.01 * speed_error,
                    0.0,
                    1.0,
                )
                if self.mode == "pseudo_6dof_kinematic_bridge":
                    roll_target = reference.bank_rad
                    pitch_target = self.trim_pitch_rad + reference.flight_path_angle_rad
                    yaw_target = reference.heading_rad
                    if self.response_profile is None:
                        roll_rate_target = _wrap(roll_target - roll_rad) / self.attitude_time_constant_s
                        pitch_rate_target = _wrap(pitch_target - pitch_rad) / self.attitude_time_constant_s
                        yaw_rate_target = _wrap(yaw_target - yaw_rad) / self.attitude_time_constant_s
                        p_rad_s = _slew(p_rad_s, _clamp(roll_rate_target, -0.5, 0.5), self.attitude_time_constant_s, dt)
                        q_rad_s = _slew(q_rad_s, _clamp(pitch_rate_target, -0.5, 0.5), self.attitude_time_constant_s, dt)
                        r_rad_s = _slew(r_rad_s, _clamp(yaw_rate_target, -0.5, 0.5), self.attitude_time_constant_s, dt)
                        roll_rad += p_rad_s * dt
                        pitch_rad += q_rad_s * dt
                        yaw_rad = _wrap(yaw_rad + r_rad_s * dt)
                    else:
                        roll_state = step_bounded_axis_response(
                            self.response_profile.response["roll"],
                            AxisResponseState(roll_rad, p_rad_s),
                            roll_target,
                            dt,
                        )
                        pitch_state = step_bounded_axis_response(
                            self.response_profile.response["pitch"],
                            AxisResponseState(pitch_rad, q_rad_s),
                            pitch_target,
                            dt,
                        )
                        yaw_state = step_bounded_axis_response(
                            self.response_profile.response["yaw"],
                            AxisResponseState(yaw_rad, r_rad_s),
                            yaw_target,
                            dt,
                        )
                        roll_rad, p_rad_s = roll_state.angle_rad, roll_state.rate_rad_s
                        pitch_rad, q_rad_s = pitch_state.angle_rad, pitch_state.rate_rad_s
                        yaw_rad, r_rad_s = _wrap(yaw_state.angle_rad), yaw_state.rate_rad_s
                rows.append(
                    self._sample(
                        time_s,
                        north_m,
                        east_m,
                        altitude_m,
                        speed_m_s,
                        heading_rad,
                        flight_path_angle_rad,
                        reference,
                        source_observables,
                        controls,
                        roll_rad=roll_rad,
                        pitch_rad=pitch_rad,
                        yaw_rad=yaw_rad if self.mode == "pseudo_6dof_kinematic_bridge" else None,
                        p_rad_s=p_rad_s,
                        q_rad_s=q_rad_s,
                        r_rad_s=r_rad_s,
                    )
                )
                if time_s >= horizon:
                    break
                time_s += dt
                north_m, east_m, altitude_m = north_next, east_next, altitude_next
                speed_m_s, heading_rad, flight_path_angle_rad = speed_next, heading_next, gamma_next
                if not all(math.isfinite(float(value)) for value in (time_s, north_m, east_m, altitude_m, speed_m_s, heading_rad, flight_path_angle_rad)):
                    raise FloatingPointError("reduced F-16 racetrack state became non-finite")
            except (FloatingPointError, ValueError, KeyError) as error:
                numerical_valid = False
                failure = str(error)
                break
        return F16ReducedRacetrackRun(self.mode, tuple(rows), numerical_valid, failure)
        ####

class F16ReducedRacetrackStepper:
    """Stateful source-owned F-16 reduced-flight stepper.

    The existing runner remains the autonomous batch reference. This class
    owns the equivalent trim-derived state for accepted external action
    boundaries. It accepts kinematic guidance only and never turns the F-16
    source controls sampled for diagnostics into physical allocation evidence.
    """

    def __init__(self, runner: F16ReducedRacetrackRunner, *, horizon_s: float | None = None) -> None:
        self.runner = runner
        self.horizon_s = runner.route.horizon_s if horizon_s is None else float(horizon_s)
        if not math.isfinite(self.horizon_s) or self.horizon_s <= 0.0:
            raise ValueError("F-16 stepper horizon must be finite and positive")
        self._state = self._initial_state()
        ####

    @property
    def state(self) -> F16ReducedRacetrackStepperState:
        """Return a copy of committed state without mutable control aliases."""

        return replace(self._state, controls=dict(self._state.controls))
        ####

    @property
    def completed(self) -> bool:
        return not self._state.numerical_valid or self._state.time_s >= self.horizon_s - 1.0e-12
        ####

    def reset(self) -> F16ReducedRacetrackStepperState:
        self._state = self._initial_state()
        return self.state
        ####

    def restore(self, snapshot: F16ReducedRacetrackStepperState) -> F16ReducedRacetrackStepperState:
        """Restore a validated composition-owned state snapshot."""

        numeric = (
            snapshot.time_s,
            snapshot.north_m,
            snapshot.east_m,
            snapshot.altitude_m,
            snapshot.speed_m_s,
            snapshot.heading_rad,
            snapshot.flight_path_angle_rad,
            snapshot.roll_rad,
            snapshot.pitch_rad,
            snapshot.yaw_rad,
            snapshot.p_rad_s,
            snapshot.q_rad_s,
            snapshot.r_rad_s,
            *snapshot.controls.values(),
        )
        if not 0.0 <= snapshot.time_s <= self.horizon_s or not all(math.isfinite(value) for value in numeric):
            raise ValueError("F-16 stepper checkpoint is outside the declared finite state space")
        self._state = replace(snapshot, controls=dict(snapshot.controls))
        return self.state
        ####

    def current_row(self, override: F16GuidanceOverride | None = None) -> dict[str, float | int | str]:
        """Return current committed telemetry without advancing or interpolation."""

        state = self._state
        reference = self._reference(override)
        observables = self.runner._source_observables(
            state.speed_m_s,
            state.flight_path_angle_rad,
            state.controls,
            state.altitude_m,
        )
        return self.runner._sample(
            state.time_s,
            state.north_m,
            state.east_m,
            state.altitude_m,
            state.speed_m_s,
            state.heading_rad,
            state.flight_path_angle_rad,
            reference,
            observables,
            state.controls,
            roll_rad=state.roll_rad,
            pitch_rad=state.pitch_rad,
            yaw_rad=state.yaw_rad if self.runner.mode == "pseudo_6dof_kinematic_bridge" else None,
            p_rad_s=state.p_rad_s,
            q_rad_s=state.q_rad_s,
            r_rad_s=state.r_rad_s,
        )
        ####

    def step(
        self,
        duration_s: float,
        override: F16GuidanceOverride | None = None,
    ) -> tuple[dict[str, float | int | str], ...]:
        """Advance held guidance through exact source-owned inner steps."""

        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError("F-16 step duration must be finite and positive")
        if self.completed:
            return ()
        target_time = min(self.horizon_s, self._state.time_s + duration_s)
        rows: list[dict[str, float | int | str]] = []
        while self._state.time_s < target_time - 1.0e-12 and self._state.numerical_valid:
            rows.append(self.current_row(override))
            self._advance_one(min(self.runner.dt_s, target_time - self._state.time_s), override)
        return tuple(rows)
        ####

    def _initial_state(self) -> F16ReducedRacetrackStepperState:
        return F16ReducedRacetrackStepperState(
            0.0,
            self.runner.initial_north_offset_m,
            self.runner.initial_east_offset_m,
            float(self.runner.route.low_altitude_m) + self.runner.initial_altitude_offset_m,
            max(1.0, float(self.runner.route.speed_m_s) + self.runner.initial_speed_offset_m_s),
            _wrap(math.pi / 2.0 + self.runner.initial_heading_offset_rad),
            _clamp(self.runner.initial_flight_path_offset_rad, -0.8, 0.8),
            self.runner.initial_bank_offset_rad if self.runner.mode == "pseudo_6dof_kinematic_bridge" else 0.0,
            self.runner.trim_pitch_rad + _clamp(self.runner.initial_flight_path_offset_rad, -0.8, 0.8),
            _wrap(math.pi / 2.0 + self.runner.initial_heading_offset_rad),
            0.0,
            0.0,
            0.0,
            dict(self.runner.trim.controls),
        )
        ####

    def _reference(self, override: F16GuidanceOverride | None) -> RacetrackGuidanceReference:
        state = self._state
        reference = self.runner._reference(state.time_s, state.north_m, state.east_m)
        if override is None:
            return reference
        return replace(
            reference,
            speed_m_s=reference.speed_m_s if override.speed_m_s is None else override.speed_m_s,
            flight_path_angle_rad=(
                reference.flight_path_angle_rad
                if override.flight_path_angle_rad is None
                else _clamp(override.flight_path_angle_rad, -0.8, 0.8)
            ),
            heading_rad=reference.heading_rad if override.heading_rad is None else _wrap(override.heading_rad),
            bank_rad=reference.bank_rad if override.bank_angle_rad is None else _clamp(override.bank_angle_rad, -1.2, 1.2),
        )
        ####

    def _advance_one(self, dt_s: float, override: F16GuidanceOverride | None) -> None:
        state = self._state
        try:
            reference = self._reference(override)
            speed_error = reference.speed_m_s - state.speed_m_s
            speed_acceleration = _clamp(
                speed_error / self.runner.speed_time_constant_s,
                -self.runner.maximum_speed_acceleration_mps2,
                self.runner.maximum_speed_acceleration_mps2,
            )
            speed_next = max(1.0, state.speed_m_s + speed_acceleration * dt_s)
            heading_error = _wrap(reference.heading_rad - state.heading_rad)
            heading_rate = _clamp(
                heading_error / self.runner.heading_time_constant_s,
                -self.runner.maximum_turn_rate_rad_s,
                self.runner.maximum_turn_rate_rad_s,
            )
            gamma_error = reference.flight_path_angle_rad - state.flight_path_angle_rad
            gamma_rate = _clamp(
                gamma_error / self.runner.flight_path_time_constant_s,
                -self.runner.maximum_flight_path_rate_rad_s,
                self.runner.maximum_flight_path_rate_rad_s,
            )
            heading_next = _wrap(state.heading_rad + heading_rate * dt_s)
            gamma_next = _clamp(state.flight_path_angle_rad + gamma_rate * dt_s, -0.8, 0.8)
            altitude_next = state.altitude_m + state.speed_m_s * math.sin(state.flight_path_angle_rad) * dt_s
            north_next = state.north_m + state.speed_m_s * math.cos(state.flight_path_angle_rad) * math.cos(state.heading_rad) * dt_s
            east_next = state.east_m + state.speed_m_s * math.cos(state.flight_path_angle_rad) * math.sin(state.heading_rad) * dt_s
            controls = dict(state.controls)
            controls["throttle_fraction"] = _clamp(
                float(self.runner.trim.controls["throttle_fraction"]) + 0.01 * speed_error,
                0.0,
                1.0,
            )
            roll_rad, pitch_rad, yaw_rad = state.roll_rad, state.pitch_rad, state.yaw_rad
            p_rad_s, q_rad_s, r_rad_s = state.p_rad_s, state.q_rad_s, state.r_rad_s
            if self.runner.mode == "pseudo_6dof_kinematic_bridge":
                roll_rad, pitch_rad, yaw_rad, p_rad_s, q_rad_s, r_rad_s = self._advance_pseudo_response(
                    reference,
                    dt_s,
                    roll_rad,
                    pitch_rad,
                    yaw_rad,
                    p_rad_s,
                    q_rad_s,
                    r_rad_s,
                )
            time_s = min(self.horizon_s, state.time_s + dt_s)
            numeric = (
                time_s,
                north_next,
                east_next,
                altitude_next,
                speed_next,
                heading_next,
                gamma_next,
                roll_rad,
                pitch_rad,
                yaw_rad,
                p_rad_s,
                q_rad_s,
                r_rad_s,
            )
            if not all(math.isfinite(value) for value in numeric):
                raise FloatingPointError("reduced F-16 stepper state became non-finite")
            self._state = F16ReducedRacetrackStepperState(
                time_s,
                north_next,
                east_next,
                altitude_next,
                speed_next,
                heading_next,
                gamma_next,
                roll_rad,
                pitch_rad,
                yaw_rad,
                p_rad_s,
                q_rad_s,
                r_rad_s,
                controls,
            )
        except (FloatingPointError, ValueError, KeyError) as error:
            self._state = replace(state, numerical_valid=False, failure=str(error))
        ####

    def _advance_pseudo_response(
        self,
        reference: RacetrackGuidanceReference,
        dt_s: float,
        roll_rad: float,
        pitch_rad: float,
        yaw_rad: float,
        p_rad_s: float,
        q_rad_s: float,
        r_rad_s: float,
    ) -> tuple[float, float, float, float, float, float]:
        roll_target = reference.bank_rad
        pitch_target = self.runner.trim_pitch_rad + reference.flight_path_angle_rad
        yaw_target = reference.heading_rad
        if self.runner.response_profile is None:
            roll_rate_target = _wrap(roll_target - roll_rad) / self.runner.attitude_time_constant_s
            pitch_rate_target = _wrap(pitch_target - pitch_rad) / self.runner.attitude_time_constant_s
            yaw_rate_target = _wrap(yaw_target - yaw_rad) / self.runner.attitude_time_constant_s
            p_rad_s = _slew(p_rad_s, _clamp(roll_rate_target, -0.5, 0.5), self.runner.attitude_time_constant_s, dt_s)
            q_rad_s = _slew(q_rad_s, _clamp(pitch_rate_target, -0.5, 0.5), self.runner.attitude_time_constant_s, dt_s)
            r_rad_s = _slew(r_rad_s, _clamp(yaw_rate_target, -0.5, 0.5), self.runner.attitude_time_constant_s, dt_s)
            return (
                roll_rad + p_rad_s * dt_s,
                pitch_rad + q_rad_s * dt_s,
                _wrap(yaw_rad + r_rad_s * dt_s),
                p_rad_s,
                q_rad_s,
                r_rad_s,
            )
        roll_state = step_bounded_axis_response(
            self.runner.response_profile.response["roll"], AxisResponseState(roll_rad, p_rad_s), roll_target, dt_s
        )
        pitch_state = step_bounded_axis_response(
            self.runner.response_profile.response["pitch"], AxisResponseState(pitch_rad, q_rad_s), pitch_target, dt_s
        )
        yaw_state = step_bounded_axis_response(
            self.runner.response_profile.response["yaw"], AxisResponseState(yaw_rad, r_rad_s), yaw_target, dt_s
        )
        return (
            roll_state.angle_rad,
            pitch_state.angle_rad,
            _wrap(yaw_state.angle_rad),
            roll_state.rate_rad_s,
            pitch_state.rate_rad_s,
            yaw_state.rate_rad_s,
        )
        ####
    ####


__all__ = [
    "F16GuidanceOverride",
    "F16ReducedRacetrackMode",
    "F16ReducedRacetrackRun",
    "F16ReducedRacetrackRunner",
    "F16ReducedRacetrackStepper",
    "F16ReducedRacetrackStepperState",
]
####
