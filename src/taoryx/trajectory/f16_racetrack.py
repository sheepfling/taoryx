"""Source-aware F-16 execution of the shared powered-fixed-wing racetrack.

The runner is intentionally local and explicit.  It connects the existing
source DAVE-ML plant, plant-derived wrench LQR, physical allocator, and shared
racetrack reference without claiming that the current F-16 package contains a
full production flight-control system.  Direct-wrench execution is retained
as a labelled comparison path; surface execution is the physically
accountable path.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, replace
from typing import Literal

import numpy as np

from ..contracts import Vector3
from ..modes import Quaternion
from ..physical_lqr import PhysicalWrenchLqrDesign
from ..racetrack_guidance import RacetrackGuidanceReference, racetrack_reference_at_time
from ..racetrack_template import ResolvedRacetrack
from ..trim import TrimResult
from .f16_reference import F16ReferencePhysicalPlant, F16ReferencePlant

F16RacetrackMode = Literal["direct_wrench", "surface_allocated"]
STATE_NAMES: tuple[str, ...] = ("u_m_s", "v_m_s", "w_m_s", "p_rad_s", "q_rad_s", "r_rad_s")


@dataclass(frozen=True, slots=True)
class F16RacetrackNavigationState:
    """Committed local navigation and source body state."""

    time_s: float
    body: Mapping[str, float]
    attitude: Quaternion
    north_m: float
    east_m: float
    altitude_m: float


@dataclass(frozen=True, slots=True)
class F16RacetrackRun:
    """Telemetry and run metadata from one local racetrack execution."""

    mode: F16RacetrackMode
    rows: tuple[dict[str, float | int | str], ...]
    numerical_valid: bool
    failure: str | None


def _wrap(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
    ####


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))
    ####


def _euler_from_quaternion(attitude: Quaternion) -> tuple[float, float, float]:
    """Return 3-2-1 roll, pitch, yaw from a body-to-NED quaternion."""

    q = attitude.normalized()
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch_argument = _clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0)
    pitch = math.asin(pitch_argument)
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw
    ####


def _quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Construct a body-to-NED ZYX Euler quaternion."""

    half_roll = 0.5 * roll
    half_pitch = 0.5 * pitch
    half_yaw = 0.5 * yaw
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return Quaternion(
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr,
    ).normalized()
    ####


def _desired_body_reference(
    route_reference: RacetrackGuidanceReference,
    attitude: Quaternion,
    trim: TrimResult,
    *,
    north_position_error_m: float = 0.0,
    east_position_error_m: float = 0.0,
    position_capture_gain: float = 0.0,
    position_capture_max_correction_mps: float = 0.0,
) -> dict[str, float]:
    """Convert route kinematics and attitude errors to a local body target."""

    horizontal_speed = route_reference.speed_m_s * math.cos(route_reference.flight_path_angle_rad)
    velocity_navigation = Vector3(
        horizontal_speed * math.cos(route_reference.heading_rad),
        horizontal_speed * math.sin(route_reference.heading_rad),
        -route_reference.speed_m_s * math.sin(route_reference.flight_path_angle_rad),
    )
    correction_north = _clamp(
        position_capture_gain * north_position_error_m,
        -position_capture_max_correction_mps,
        position_capture_max_correction_mps,
    )
    correction_east = _clamp(
        position_capture_gain * east_position_error_m,
        -position_capture_max_correction_mps,
        position_capture_max_correction_mps,
    )
    velocity_navigation = Vector3(
        velocity_navigation.x + correction_north,
        velocity_navigation.y + correction_east,
        velocity_navigation.z,
    )
    roll, pitch, yaw = _euler_from_quaternion(attitude)
    # The outer path loop commands a bank and flight-path angle.  Preserve
    # the source-trim angle of attack while changing flight path, then express
    # the desired inertial velocity in that commanded body frame.  Using the
    # current attitude here would ask the inner loop to erase the trim angle
    # of attack whenever a climb or descent begins.
    trim_pitch = math.atan2(float(trim.state["w_m_s"]), float(trim.state["u_m_s"]))
    desired_attitude = _quaternion_from_euler(
        route_reference.bank_rad,
        trim_pitch + route_reference.flight_path_angle_rad,
        route_reference.heading_rad,
    )
    velocity_body = desired_attitude.conjugate().rotate(velocity_navigation)
    pitch_target = trim_pitch + route_reference.flight_path_angle_rad
    roll_error = _wrap(route_reference.bank_rad - roll)
    pitch_error = _wrap(pitch_target - pitch)
    heading_error = _wrap(route_reference.heading_rad - yaw)
    return {
        "u_m_s": velocity_body.x,
        "v_m_s": velocity_body.y,
        "w_m_s": velocity_body.z,
        "p_rad_s": _clamp(0.10 * roll_error, -0.05, 0.05),
        "q_rad_s": _clamp(0.60 * pitch_error, -0.10, 0.10),
        # In a coordinated fixed-wing turn, bank is the primary turn command;
        # an independent yaw-rate demand would fight the coordinated response
        # and feed the local LQR's roll/yaw cross-coupling.  Keep yaw as a
        # low-bandwidth heading-hold correction only.
        "r_rad_s": _clamp(0.03 * heading_error, -0.02, 0.02),
    }
    ####


def _gravity_components(gravity: float, roll: float, pitch: float) -> tuple[float, float, float]:
    return (
        -gravity * math.sin(pitch),
        gravity * math.sin(roll) * math.cos(pitch),
        gravity * math.cos(roll) * math.cos(pitch),
    )
    ####


def _body_derivative(
    source: F16ReferencePlant,
    body: Mapping[str, float],
    controls: Mapping[str, float],
    *,
    altitude_m: float,
    attitude: Quaternion,
    direct_wrench: Mapping[str, float] | None,
) -> dict[str, float]:
    """Evaluate source dynamics, replacing only the explicitly direct axes."""

    roll, pitch, _ = _euler_from_quaternion(attitude)
    loads = source.evaluate_loads(body, controls, altitude_m=altitude_m)
    force_x = loads["total_force_x_n"] if direct_wrench is None else float(direct_wrench["total_force_x_n"])
    force_y = loads["total_force_y_n"]
    force_z = loads["total_force_z_n"]
    moments: tuple[float, float, float] = (
        loads["total_moment_x_nm"],
        loads["total_moment_y_nm"],
        loads["total_moment_z_nm"],
    )
    if direct_wrench is not None:
        moments = (
            float(direct_wrench["total_moment_x_nm"]),
            float(direct_wrench["total_moment_y_nm"]),
            float(direct_wrench["total_moment_z_nm"]),
        )
    gravity = _gravity_components(source.gravity_m_s2, roll, pitch)
    u, v, w = (float(body[name]) for name in ("u_m_s", "v_m_s", "w_m_s"))
    p, q, r = (float(body[name]) for name in ("p_rad_s", "q_rad_s", "r_rad_s"))
    linear = (
        force_x / source.mass_kg + gravity[0] - q * w + r * v,
        force_y / source.mass_kg + gravity[1] - r * u + p * w,
        force_z / source.mass_kg + gravity[2] - p * v + q * u,
    )
    inertia = np.asarray(source.inertia_matrix_kg_m2, dtype=float)
    omega = np.asarray((p, q, r), dtype=float)
    angular = np.linalg.solve(inertia, np.asarray(moments, dtype=float) - np.cross(omega, inertia @ omega))
    return {
        **dict(zip(("u_m_s", "v_m_s", "w_m_s"), linear, strict=True)),
        **dict(zip(("p_rad_s", "q_rad_s", "r_rad_s"), angular, strict=True)),
    }
    ####


def _rk4_body_step(
    source: F16ReferencePlant,
    body: Mapping[str, float],
    controls: Mapping[str, float],
    *,
    altitude_m: float,
    attitude: Quaternion,
    dt_s: float,
    direct_wrench: Mapping[str, float] | None,
) -> dict[str, float]:
    """Advance the source body state with held controls and direct demand."""

    base = np.asarray([float(body[name]) for name in STATE_NAMES], dtype=float)

    def evaluate(values: np.ndarray) -> np.ndarray:
        candidate = dict(zip(STATE_NAMES, values, strict=True))
        derivative = _body_derivative(
            source,
            candidate,
            controls,
            altitude_m=altitude_m,
            attitude=attitude,
            direct_wrench=direct_wrench,
        )
        return np.asarray([float(derivative[name]) for name in STATE_NAMES], dtype=float)

    first = evaluate(base)
    second = evaluate(base + 0.5 * dt_s * first)
    third = evaluate(base + 0.5 * dt_s * second)
    fourth = evaluate(base + dt_s * third)
    result = base + dt_s * (first + 2.0 * second + 2.0 * third + fourth) / 6.0
    if not np.all(np.isfinite(result)):
        raise FloatingPointError("F-16 racetrack body integration produced non-finite state")
    return {name: float(value) for name, value in zip(STATE_NAMES, result, strict=True)}
    ####


@dataclass(slots=True)
class F16RacetrackRunner:
    """Execute a shared racetrack through one declared F-16 realization."""

    source: F16ReferencePlant
    trim: TrimResult
    design: PhysicalWrenchLqrDesign
    route: ResolvedRacetrack
    mode: F16RacetrackMode
    physical_adapter: F16ReferencePhysicalPlant | None = None
    dt_s: float = 0.05
    outer_roll_kp_s2: float = 0.5
    outer_roll_kd_s: float = 2.5
    outer_pitch_kp_s2: float = 0.5
    outer_pitch_kd_s: float = 1.0
    outer_speed_kp_s: float = 0.12
    outer_yaw_rate_kp_s: float = 0.8
    initial_north_offset_m: float = 0.0
    initial_east_offset_m: float = 0.0
    initial_altitude_offset_m: float = 0.0
    initial_speed_offset_m_s: float = 0.0
    initial_bank_offset_rad: float = 0.0

    def __post_init__(self) -> None:
        if self.mode == "surface_allocated" and self.physical_adapter is None:
            raise ValueError("surface-allocated F-16 racetrack requires a physical adapter")
        if not math.isfinite(self.dt_s) or self.dt_s <= 0.0:
            raise ValueError("F-16 racetrack step must be finite and positive")
        if not all(
            math.isfinite(value) and value >= 0.0
            for value in (
                self.outer_roll_kp_s2,
                self.outer_roll_kd_s,
                self.outer_pitch_kp_s2,
                self.outer_pitch_kd_s,
                self.outer_speed_kp_s,
                self.outer_yaw_rate_kp_s,
            )
        ) or not all(
            math.isfinite(value)
            for value in (
                self.initial_north_offset_m,
                self.initial_east_offset_m,
                self.initial_altitude_offset_m,
                self.initial_speed_offset_m_s,
                self.initial_bank_offset_rad,
            )
        ):
            raise ValueError("F-16 racetrack outer attitude gains must be finite and nonnegative")
        if set(self.design.projection.state_names) != set(STATE_NAMES):
            raise ValueError("F-16 racetrack controller state names do not match the source plant")
        ####

    def initial_state(self) -> F16RacetrackNavigationState:
        """Construct an eastbound local start state from the source trim."""

        trim_pitch = math.atan2(float(self.trim.state["w_m_s"]), float(self.trim.state["u_m_s"]))
        attitude = _quaternion_from_euler(self.initial_bank_offset_rad, trim_pitch, math.pi / 2.0)
        body = dict(self.trim.state)
        body["u_m_s"] += self.initial_speed_offset_m_s
        return F16RacetrackNavigationState(
            time_s=0.0,
            body=body,
            attitude=attitude,
            north_m=self.initial_north_offset_m,
            east_m=self.initial_east_offset_m,
            altitude_m=max(self.route.low_altitude_m, float(self.route.low_altitude_m) + self.initial_altitude_offset_m),
        )
        ####

    def _sample(
        self,
        state: F16RacetrackNavigationState,
        reference: RacetrackGuidanceReference,
        requested_wrench: Mapping[str, float],
        controls: Mapping[str, float],
        commanded_controls: Mapping[str, float],
        achieved_wrench: Mapping[str, float],
        allocation_status: str,
        allocation_residual_norm: float,
        saturation_count: int,
    ) -> dict[str, float | int | str]:
        loads = self.source.evaluate_loads(state.body, controls, altitude_m=state.altitude_m)
        roll, pitch, yaw = _euler_from_quaternion(state.attitude)
        speed = math.sqrt(sum(float(state.body[name]) ** 2 for name in ("u_m_s", "v_m_s", "w_m_s")))
        return {
            "time_s": state.time_s,
            "north_m": state.north_m,
            "east_m": state.east_m,
            "altitude_m": state.altitude_m,
            "speed_m_s": speed,
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
            "route_bank_achieved_deg": math.degrees(roll),
            "route_pitch_achieved_deg": math.degrees(pitch),
            "route_heading_achieved_deg": math.degrees(yaw),
            "local_roll_deg": math.degrees(roll),
            "local_pitch_deg": math.degrees(pitch),
            "local_heading_deg": math.degrees(yaw),
            "aero_alpha_deg": loads["alpha_deg"],
            "aero_sideslip_deg": loads["beta_deg"],
            "mach": loads["mach"],
            "dynamic_pressure_pa": loads["dynamic_pressure_pa"],
            "source_aerodynamic_force_x_n": loads["aerodynamic_force_x_n"],
            "source_aerodynamic_force_y_n": loads["aerodynamic_force_y_n"],
            "source_aerodynamic_force_z_n": loads["aerodynamic_force_z_n"],
            "source_propulsion_force_x_n": loads["propulsion_force_x_n"],
            "source_total_force_x_n": loads["total_force_x_n"],
            "source_total_force_y_n": loads["total_force_y_n"],
            "source_total_force_z_n": loads["total_force_z_n"],
            "source_total_moment_x_nm": loads["total_moment_x_nm"],
            "source_total_moment_y_nm": loads["total_moment_y_nm"],
            "source_total_moment_z_nm": loads["total_moment_z_nm"],
            "u_m_s": state.body["u_m_s"],
            "v_m_s": state.body["v_m_s"],
            "w_m_s": state.body["w_m_s"],
            "p_rad_s": state.body["p_rad_s"],
            "q_rad_s": state.body["q_rad_s"],
            "r_rad_s": state.body["r_rad_s"],
            "requested_force_x_n": float(requested_wrench["total_force_x_n"]),
            "requested_moment_x_nm": float(requested_wrench["total_moment_x_nm"]),
            "requested_moment_y_nm": float(requested_wrench["total_moment_y_nm"]),
            "requested_moment_z_nm": float(requested_wrench["total_moment_z_nm"]),
            "achieved_force_x_n": float(achieved_wrench["total_force_x_n"]),
            "achieved_moment_x_nm": float(achieved_wrench["total_moment_x_nm"]),
            "achieved_moment_y_nm": float(achieved_wrench["total_moment_y_nm"]),
            "achieved_moment_z_nm": float(achieved_wrench["total_moment_z_nm"]),
            "allocation_status": allocation_status,
            "allocation_residual_norm": float(allocation_residual_norm),
            "saturation_count": int(saturation_count),
            "elevator_deg": float(controls["elevator_deg"]),
            "aileron_deg": float(controls["aileron_deg"]),
            "rudder_deg": float(controls["rudder_deg"]),
            "throttle_fraction": float(controls["throttle_fraction"]),
            "commanded_elevator_deg": float(commanded_controls["elevator_deg"]),
            "commanded_aileron_deg": float(commanded_controls["aileron_deg"]),
            "commanded_rudder_deg": float(commanded_controls["rudder_deg"]),
            "commanded_throttle_fraction": float(commanded_controls["throttle_fraction"]),
            "elevator_position_error_deg": float(commanded_controls["elevator_deg"] - controls["elevator_deg"]),
            "aileron_position_error_deg": float(commanded_controls["aileron_deg"] - controls["aileron_deg"]),
            "rudder_position_error_deg": float(commanded_controls["rudder_deg"] - controls["rudder_deg"]),
            "throttle_position_error_fraction": float(commanded_controls["throttle_fraction"] - controls["throttle_fraction"]),
        }
        ####

    def run(self, *, duration_s: float | None = None) -> F16RacetrackRun:
        """Execute until the route horizon or an integration failure."""

        horizon = self.route.horizon_s if duration_s is None else float(duration_s)
        if not math.isfinite(horizon) or horizon <= 0.0:
            raise ValueError("F-16 racetrack horizon must be finite and positive")
        state = self.initial_state()
        actual_effectors = dict(self.trim.controls)
        rows: list[dict[str, float | int | str]] = []
        failure: str | None = None
        numerical_valid = True
        steps = int(math.ceil(horizon / self.dt_s))
        for _ in range(steps + 1):
            try:
                reference = racetrack_reference_at_time(self.route, state.time_s, trim_pitch_rad=0.0)
                if reference.phase == "inbound-descent" and state.east_m > self.route.gates[1].east_m:
                    # The timing estimate is a planning aid, not a permission
                    # to abandon the left-turn exit gate early.  Hold the
                    # high-altitude level contract until truth geometry has
                    # crossed the exit plane, then let the descent controller
                    # take over.  The independent evaluator still decides
                    # whether the gate was actually satisfied.
                    reference = replace(
                        reference,
                        altitude_m=self.route.high_altitude_m,
                        flight_path_angle_rad=0.0,
                    )
                if state.time_s >= self.route.declared_duration_s:
                    terminal_heading = math.atan2(-state.east_m, -state.north_m)
                    _, current_pitch, current_heading = _euler_from_quaternion(state.attitude)
                    terminal_heading_error = _wrap(terminal_heading - current_heading)
                    terminal_bank = _clamp(0.6 * terminal_heading_error, -math.radians(10.0), math.radians(10.0))
                    reference = replace(
                        reference,
                        heading_rad=terminal_heading,
                        bank_rad=terminal_bank,
                        flight_path_angle_rad=_clamp(-0.15 * current_pitch, -0.15, 0.15),
                    )
                altitude_error = reference.altitude_m - state.altitude_m
                commanded_vertical_rate = (
                    reference.speed_m_s * math.sin(reference.flight_path_angle_rad)
                    + self.route.altitude_capture_gain_per_s * altitude_error
                )
                commanded_vertical_rate = _clamp(
                    commanded_vertical_rate,
                    -self.route.altitude_capture_max_mps,
                    self.route.altitude_capture_max_mps,
                )
                reference = replace(
                    reference,
                    flight_path_angle_rad=math.asin(
                        _clamp(commanded_vertical_rate / reference.speed_m_s, -0.8, 0.8)
                    ),
                )
                if reference.phase == "inbound-level":
                    cross_track_bank_correction = _clamp(
                        -self.route.position_capture_bank_gain_rad_per_m
                        * (state.north_m - reference.north_m),
                        -math.radians(self.route.position_capture_max_bank_correction_deg),
                        math.radians(self.route.position_capture_max_bank_correction_deg),
                    )
                    reference = replace(reference, bank_rad=reference.bank_rad + cross_track_bank_correction)
                position_capture_enabled = reference.phase in {"outbound-level", "inbound-level"}
                state_reference = _desired_body_reference(
                    reference,
                    state.attitude,
                    self.trim,
                    north_position_error_m=(reference.north_m - state.north_m) if position_capture_enabled else 0.0,
                    east_position_error_m=(reference.east_m - state.east_m) if position_capture_enabled else 0.0,
                    position_capture_gain=self.route.position_capture_gain if position_capture_enabled else 0.0,
                    position_capture_max_correction_mps=(
                        self.route.position_capture_max_correction_mps if position_capture_enabled else 0.0
                    ),
                )
                turn_rate_reference = (
                    -self.route.turn_rate_command_scale * reference.speed_m_s / self.route.turn_radius_m
                    if reference.phase in {"left-turn", "right-turn"}
                    else 0.0
                )
                state_reference["r_rad_s"] = turn_rate_reference
                requested, _ = self.design.requested_wrench_for_reference(state.body, state_reference)
                roll, pitch, _ = _euler_from_quaternion(state.attitude)
                roll_error = _wrap(reference.bank_rad - roll)
                roll_acceleration_command = self.outer_roll_kp_s2 * roll_error - self.outer_roll_kd_s * float(
                    state.body["p_rad_s"]
                )
                trim_pitch = math.atan2(float(self.trim.state["w_m_s"]), float(self.trim.state["u_m_s"]))
                pitch_error = _wrap(trim_pitch + reference.flight_path_angle_rad - pitch)
                pitch_acceleration_command = self.outer_pitch_kp_s2 * pitch_error - self.outer_pitch_kd_s * float(
                    state.body["q_rad_s"]
                )
                speed = math.sqrt(sum(float(state.body[name]) ** 2 for name in ("u_m_s", "v_m_s", "w_m_s")))
                speed_force_command = self.source.mass_kg * self.outer_speed_kp_s * (reference.speed_m_s - speed)
                requested["total_force_x_n"] += speed_force_command
                yaw_rate_acceleration_command = self.outer_yaw_rate_kp_s * (
                    turn_rate_reference - float(state.body["r_rad_s"])
                )
                requested["total_moment_x_nm"] += self.source.inertia_matrix_kg_m2[0][0] * roll_acceleration_command
                requested["total_moment_y_nm"] += self.source.inertia_matrix_kg_m2[1][1] * pitch_acceleration_command
                requested["total_moment_z_nm"] += self.source.inertia_matrix_kg_m2[2][2] * yaw_rate_acceleration_command
                allocation_status = "direct_wrench"
                allocation_residual_norm = 0.0
                saturation_count = 0
                if self.mode == "surface_allocated":
                    assert self.physical_adapter is not None
                    active_adapter = replace(self.physical_adapter, altitude_m=state.altitude_m)
                    allocation = active_adapter.allocate(state.body, requested, actual_effectors, self.dt_s)
                    commanded_controls = dict(allocation.actuator.commanded_positions)
                    actual_effectors = dict(allocation.actuator.actual_positions)
                    controls = actual_effectors
                    achieved = self.source.evaluate_loads(state.body, controls, altitude_m=state.altitude_m)
                    achieved_wrench = {name: float(achieved[name]) for name in self.physical_adapter.wrench_names}
                    allocation_status = allocation.allocation.status
                    allocation_residual_norm = allocation.achieved_controlled_residual_norm
                    saturation_count = len(
                        set(allocation.allocation.position_saturated)
                        | set(allocation.allocation.rate_limited)
                        | set(allocation.actuator.position_saturated)
                        | set(allocation.actuator.rate_limited)
                    )
                    direct_wrench = None
                else:
                    controls = dict(self.trim.controls)
                    commanded_controls = dict(controls)
                    achieved_wrench = dict(requested)
                    direct_wrench = requested
                rows.append(
                    self._sample(
                        state,
                        reference,
                        requested,
                        controls,
                        commanded_controls,
                        achieved_wrench,
                        allocation_status,
                        allocation_residual_norm,
                        saturation_count,
                    )
                )
                if state.time_s >= horizon:
                    break
                body_next = _rk4_body_step(
                    self.source,
                    state.body,
                    controls,
                    altitude_m=state.altitude_m,
                    attitude=state.attitude,
                    dt_s=min(self.dt_s, horizon - state.time_s),
                    direct_wrench=direct_wrench,
                )
                dt = min(self.dt_s, horizon - state.time_s)
                attitude_next = state.attitude.integrate_body_rate(
                    Vector3(state.body["p_rad_s"], state.body["q_rad_s"], state.body["r_rad_s"]),
                    dt,
                )
                velocity_navigation = state.attitude.rotate(Vector3(state.body["u_m_s"], state.body["v_m_s"], state.body["w_m_s"]))
                state = F16RacetrackNavigationState(
                    time_s=state.time_s + dt,
                    body=body_next,
                    attitude=attitude_next,
                    north_m=state.north_m + velocity_navigation.x * dt,
                    east_m=state.east_m + velocity_navigation.y * dt,
                    altitude_m=max(
                        self.route.low_altitude_m,
                        state.altitude_m - velocity_navigation.z * dt,
                    ),
                )
                if not all(math.isfinite(float(value)) for value in (*body_next.values(), state.north_m, state.east_m, state.altitude_m)):
                    raise FloatingPointError("F-16 racetrack navigation state became non-finite")
            except (FloatingPointError, ValueError, KeyError, np.linalg.LinAlgError) as error:
                numerical_valid = False
                failure = f"{type(error).__name__}: {error}"
                break
        return F16RacetrackRun(self.mode, tuple(rows), numerical_valid, failure)
        ####


__all__ = [
    "F16RacetrackMode",
    "F16RacetrackNavigationState",
    "F16RacetrackRun",
    "F16RacetrackRunner",
]
####
