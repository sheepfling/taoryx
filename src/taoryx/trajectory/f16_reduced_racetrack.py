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

F16ReducedRacetrackMode = Literal["point_mass_3dof", "pseudo_6dof_kinematic_bridge"]
_Model = F16PointMass3DOFModel | F16AttitudeResponsePseudo6DOFModel


@dataclass(frozen=True, slots=True)
class F16ReducedRacetrackRun:
    """Telemetry and execution status for one reduced-fidelity run."""

    mode: F16ReducedRacetrackMode
    rows: tuple[dict[str, float | int | str], ...]
    numerical_valid: bool
    failure: str | None


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

    def __post_init__(self) -> None:
        if self.mode == "point_mass_3dof" and not isinstance(self.model, F16PointMass3DOFModel):
            raise ValueError("point-mass mode requires F16PointMass3DOFModel")
        if self.mode == "pseudo_6dof_kinematic_bridge" and not isinstance(
            self.model, F16AttitudeResponsePseudo6DOFModel
        ):
            raise ValueError("pseudo-6DOF mode requires F16AttitudeResponsePseudo6DOFModel")
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
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("reduced F-16 response parameters must be finite and positive")
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
        north_m = 0.0
        east_m = 0.0
        altitude_m = float(self.route.low_altitude_m)
        speed_m_s = float(self.route.speed_m_s)
        heading_rad = math.pi / 2.0
        flight_path_angle_rad = 0.0
        roll_rad = 0.0
        pitch_rad = self.trim_pitch_rad
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
                if dt < 0.0:
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
                    roll_rate_target = _wrap(roll_target - roll_rad) / self.attitude_time_constant_s
                    pitch_rate_target = _wrap(pitch_target - pitch_rad) / self.attitude_time_constant_s
                    yaw_rate_target = _wrap(yaw_target - yaw_rad) / self.attitude_time_constant_s
                    p_rad_s = _slew(p_rad_s, _clamp(roll_rate_target, -0.5, 0.5), self.attitude_time_constant_s, dt)
                    q_rad_s = _slew(q_rad_s, _clamp(pitch_rate_target, -0.5, 0.5), self.attitude_time_constant_s, dt)
                    r_rad_s = _slew(r_rad_s, _clamp(yaw_rate_target, -0.5, 0.5), self.attitude_time_constant_s, dt)
                    roll_rad += p_rad_s * dt
                    pitch_rad += q_rad_s * dt
                    yaw_rad = _wrap(yaw_rad + r_rad_s * dt)
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

__all__ = ["F16ReducedRacetrackMode", "F16ReducedRacetrackRun", "F16ReducedRacetrackRunner"]
####
