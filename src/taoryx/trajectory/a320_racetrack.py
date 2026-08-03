"""Shared racetrack execution for the A320 integration lanes.

The A320 OpenAP product is a performance model, not a complete aircraft
navigation plant.  This adapter therefore keeps the mission claim narrow:
the point-mass lane integrates OpenAP speed, altitude, mass, and fuel-flow
channels while a bounded kinematic navigation law realizes the shared course;
the pseudo-6DOF lane adds the named A320 rotational surrogate and its declared
surface-command overlay.  Neither lane claims manufacturer flight dynamics or
source-exact actuator behavior.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Literal

from ..racetrack_guidance import RacetrackGuidanceReference, racetrack_reference_at_time
from ..racetrack_template import ResolvedRacetrack
from ..trim import TrimResult
from .a320_openap import A320OpenAPModel, A320OpenAPOperatingPoint
from .a320_pseudo6dof import A320Pseudo6DOFModel, A320Pseudo6DOFOperatingPoint
from .pseudo6dof_profiles import Pseudo6DOFProfile
from .response_laws import AxisResponseState, step_bounded_axis_response

A320RacetrackMode = Literal["point_mass_3dof", "pseudo_6dof_kinematic_bridge"]
_Model = A320OpenAPModel | A320Pseudo6DOFModel


@dataclass(frozen=True, slots=True)
class A320RacetrackRun:
    """Telemetry and execution status for one A320 reduced run."""

    mode: A320RacetrackMode
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
class A320RacetrackRunner:
    """Execute the common racetrack through one declared A320 reduction."""

    model: _Model
    trim: TrimResult
    route: ResolvedRacetrack
    mode: A320RacetrackMode
    dt_s: float = 0.5
    speed_time_constant_s: float = 4.0
    heading_time_constant_s: float = 2.0
    flight_path_time_constant_s: float = 4.0
    attitude_time_constant_s: float = 2.0
    maximum_speed_acceleration_mps2: float = 3.0
    maximum_turn_rate_rad_s: float = math.radians(8.0)
    maximum_flight_path_rate_rad_s: float = math.radians(3.0)
    response_profile: Pseudo6DOFProfile | None = None
    operating_point: A320OpenAPOperatingPoint | None = None

    def __post_init__(self) -> None:
        if self.mode == "point_mass_3dof" and not isinstance(self.model, A320OpenAPModel):
            raise ValueError("point-mass mode requires A320OpenAPModel")
        if self.mode == "pseudo_6dof_kinematic_bridge" and not isinstance(self.model, A320Pseudo6DOFModel):
            raise ValueError("pseudo-6DOF mode requires A320Pseudo6DOFModel")
        if self.response_profile is not None and self.mode != "pseudo_6dof_kinematic_bridge":
            raise ValueError("response profiles apply only to pseudo-6DOF mode")
        values = (
            self.dt_s,
            self.speed_time_constant_s,
            self.heading_time_constant_s,
            self.flight_path_time_constant_s,
            self.attitude_time_constant_s,
            self.maximum_speed_acceleration_mps2,
            self.maximum_turn_rate_rad_s,
            self.maximum_flight_path_rate_rad_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("A320 racetrack response parameters must be finite and positive")
        ####

    @property
    def _trim_throttle(self) -> float:
        return float(self.trim.controls["throttle_ratio"])
        ####

    @property
    def _trim_gamma(self) -> float:
        return float(self.trim.controls["flight_path_angle_rad"])
        ####

    @property
    def _trim_alpha(self) -> float:
        return float(self.trim.state.get("alpha_rad", 0.0))
        ####

    def _reference(self, time_s: float, north_m: float, east_m: float) -> RacetrackGuidanceReference:
        """Return the route reference, then explicitly acquire the terminal gate.

        The terminal period is not another indefinite low-level leg.  Once the
        nominal racetrack has completed, steer toward the start/finish gate so
        its crossing can be independently evaluated and the existing hold
        logic can freeze the terminal state.  This also makes longer
        capability-scaled routes behave like the catalog baseline rather than
        drifting past the finish line during their simulation margin.
        """

        reference = racetrack_reference_at_time(self.route, time_s, trim_pitch_rad=self._trim_alpha)
        if time_s >= self.route.declared_duration_s:
            reference = replace(
                reference,
                heading_rad=math.atan2(-east_m, -north_m),
                flight_path_angle_rad=0.0,
                bank_rad=0.0,
            )
        return reference
        ####

    def _point_observables(self, state: dict[str, float], controls: dict[str, float]) -> dict[str, float]:
        if self.mode == "point_mass_3dof":
            model = self.model
            assert isinstance(model, A320OpenAPModel)
            result_openap = model.evaluate(
                A320OpenAPOperatingPoint(
                    altitude_m=state["altitude_m"],
                    mach=state["mach"],
                    mass_kg=state["mass_kg"],
                    vertical_speed_mps=abs(controls["vertical_speed_mps"]),
                    thrust_mode="climb" if abs(controls["vertical_speed_mps"]) > 1.0e-6 else "cruise",
                    throttle_ratio=controls["throttle_ratio"],
                )
            )
            return {
                "true_airspeed_mps": result_openap.true_airspeed_mps,
                "dynamic_pressure_pa": result_openap.dynamic_pressure_pa,
                "lift_n": result_openap.lift_n,
                "drag_n": result_openap.drag_n,
                "thrust_n": result_openap.thrust_n,
                "required_thrust_n": result_openap.required_thrust_n,
                "fuel_flow_kg_s": result_openap.fuel_flow_at_throttle_kg_s,
                "thrust_margin_n": result_openap.thrust_margin_n,
                "alpha_rad": self._trim_alpha,
                "side_force_n": 0.0,
                "roll_moment_nm": 0.0,
                "pitch_moment_nm": 0.0,
                "yaw_moment_nm": 0.0,
            }
        model = self.model
        assert isinstance(model, A320Pseudo6DOFModel)
        result_pseudo = model.evaluate(
            A320Pseudo6DOFOperatingPoint(
                A320OpenAPOperatingPoint(
                    altitude_m=state["altitude_m"],
                    mach=state["mach"],
                    mass_kg=state["mass_kg"],
                    vertical_speed_mps=abs(controls["vertical_speed_mps"]),
                    thrust_mode="climb" if abs(controls["vertical_speed_mps"]) > 1.0e-6 else "cruise",
                    throttle_ratio=controls["throttle_ratio"],
                ),
                alpha_rad=state["alpha_rad"],
                beta_rad=state["beta_rad"],
                roll_rate_rad_s=state["roll_rate_rad_s"],
                pitch_rate_rad_s=state["pitch_rate_rad_s"],
                yaw_rate_rad_s=state["yaw_rate_rad_s"],
                aileron_rad=controls["aileron_rad"],
                elevator_rad=controls["elevator_rad"],
                rudder_rad=controls["rudder_rad"],
                bank_angle_rad=state["roll_rad"],
            )
        )
        return {
            "true_airspeed_mps": result_pseudo.performance.true_airspeed_mps,
            "dynamic_pressure_pa": result_pseudo.performance.dynamic_pressure_pa,
            "lift_n": result_pseudo.performance.lift_n,
            "drag_n": result_pseudo.performance.drag_n,
            "thrust_n": result_pseudo.performance.thrust_n,
            "required_thrust_n": result_pseudo.performance.required_thrust_n,
            "fuel_flow_kg_s": result_pseudo.performance.fuel_flow_at_throttle_kg_s,
            "thrust_margin_n": result_pseudo.performance.thrust_margin_n,
            "alpha_rad": state["alpha_rad"],
            "side_force_n": result_pseudo.side_force_n,
            "roll_moment_nm": result_pseudo.roll_moment_nm,
            "pitch_moment_nm": result_pseudo.pitch_moment_nm,
            "yaw_moment_nm": result_pseudo.yaw_moment_nm,
        }
        ####

    def _sample(
        self,
        time_s: float,
        state: dict[str, float],
        north_m: float,
        east_m: float,
        speed_m_s: float,
        reference: RacetrackGuidanceReference,
        observables: dict[str, float],
        controls: dict[str, float],
    ) -> dict[str, float | int | str]:
        row: dict[str, float | int | str] = {
            "time_s": time_s,
            "north_m": north_m,
            "east_m": east_m,
            "altitude_m": state["altitude_m"],
            "speed_m_s": speed_m_s,
            "mach": state["mach"],
            "mass_kg": state["mass_kg"],
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
            "route_bank_achieved_deg": math.degrees(state.get("roll_rad", 0.0)),
            "route_pitch_achieved_deg": math.degrees(state.get("pitch_rad", reference.flight_path_angle_rad)),
            "route_heading_achieved_deg": math.degrees(state.get("yaw_rad", state["heading_rad"])),
            "heading_rad": state["heading_rad"],
            "flight_path_angle_rad": state["flight_path_angle_rad"],
            "p_rad_s": state.get("roll_rate_rad_s", 0.0),
            "q_rad_s": state.get("pitch_rate_rad_s", 0.0),
            "r_rad_s": state.get("yaw_rate_rad_s", 0.0),
            "dynamic_pressure_pa": observables["dynamic_pressure_pa"],
            "lift_n": observables["lift_n"],
            "drag_n": observables["drag_n"],
            "thrust_n": observables["thrust_n"],
            "required_thrust_n": observables["required_thrust_n"],
            "fuel_flow_kg_s": observables["fuel_flow_kg_s"],
            "thrust_margin_n": observables["thrust_margin_n"],
            "side_force_n": observables["side_force_n"],
            "roll_moment_nm": observables["roll_moment_nm"],
            "pitch_moment_nm": observables["pitch_moment_nm"],
            "yaw_moment_nm": observables["yaw_moment_nm"],
            "control_path": self.mode,
            "response_profile_id": self.response_profile.id if self.response_profile is not None else "legacy_runner_defaults",
            "allocation_status": "not_applicable" if self.mode == "point_mass_3dof" else "surrogate_policy_overlay",
            "allocation_residual_norm": 0.0,
            "saturation_count": 0,
            "throttle_ratio": controls["throttle_ratio"],
            "flight_path_angle_command_rad": controls["flight_path_angle_rad"],
        }
        if self.mode == "pseudo_6dof_kinematic_bridge":
            row.update(
                {
                    "aileron_rad": controls["aileron_rad"],
                    "elevator_rad": controls["elevator_rad"],
                    "rudder_rad": controls["rudder_rad"],
                    "control_effectivity_source": "jsbsim-1.3.1-a320-coefficient-surrogate",
                }
            )
        else:
            row["control_effectivity_source"] = "not_applicable_point_mass"
        return row
        ####

    def run(self, *, duration_s: float | None = None) -> A320RacetrackRun:
        """Execute and return truth-compatible A320 mission telemetry."""

        horizon = self.route.horizon_s if duration_s is None else float(duration_s)
        if not math.isfinite(horizon) or horizon <= 0.0:
            raise ValueError("A320 racetrack horizon must be finite and positive")
        initial_point = self.operating_point or A320OpenAPOperatingPoint(self.route.low_altitude_m, 0.78, 60000.0)
        state: dict[str, float] = {
            "altitude_m": initial_point.altitude_m,
            "mach": initial_point.mach,
            "mass_kg": initial_point.mass_kg,
            "range_m": 0.0,
            "heading_rad": math.pi / 2.0,
            "flight_path_angle_rad": self._trim_gamma,
        }
        if self.mode == "pseudo_6dof_kinematic_bridge":
            state.update(
                {
                    "alpha_rad": self._trim_alpha,
                    "beta_rad": 0.0,
                    "roll_rate_rad_s": 0.0,
                    "pitch_rate_rad_s": 0.0,
                    "yaw_rate_rad_s": 0.0,
                    "roll_rad": 0.0,
                    "pitch_rad": self._trim_alpha,
                    "yaw_rad": math.pi / 2.0,
                }
            )
        controls: dict[str, float] = {
            "throttle_ratio": self._trim_throttle,
            "flight_path_angle_rad": self._trim_gamma,
            "vertical_speed_mps": 0.0,
        }
        if self.mode == "pseudo_6dof_kinematic_bridge":
            controls.update(
                {
                    "aileron_rad": float(self.trim.controls.get("aileron_rad", 0.0)),
                    "elevator_rad": float(self.trim.controls.get("elevator_rad", 0.0)),
                    "rudder_rad": float(self.trim.controls.get("rudder_rad", 0.0)),
                    "bank_angle_rad": 0.0,
                }
            )
        north_m = 0.0
        east_m = 0.0
        terminal_hold = False
        rows: list[dict[str, float | int | str]] = []
        numerical_valid = True
        failure: str | None = None
        time_s = 0.0
        while time_s <= horizon + 1.0e-9:
            try:
                reference = self._reference(time_s, north_m, east_m)
                if self.mode == "point_mass_3dof":
                    model = self.model
                    assert isinstance(model, A320OpenAPModel)
                    current = model.evaluate(
                        A320OpenAPOperatingPoint(
                            state["altitude_m"],
                            state["mach"],
                            state["mass_kg"],
                            vertical_speed_mps=abs(controls["vertical_speed_mps"]),
                            thrust_mode="climb" if abs(controls["vertical_speed_mps"]) > 1.0e-6 else "cruise",
                            throttle_ratio=controls["throttle_ratio"],
                        )
                    )
                else:
                    model = self.model
                    assert isinstance(model, A320Pseudo6DOFModel)
                    current = model.openap.evaluate(
                        A320OpenAPOperatingPoint(
                            state["altitude_m"],
                            state["mach"],
                            state["mass_kg"],
                            vertical_speed_mps=abs(controls["vertical_speed_mps"]),
                            thrust_mode="climb" if abs(controls["vertical_speed_mps"]) > 1.0e-6 else "cruise",
                            throttle_ratio=controls["throttle_ratio"],
                        )
                    )
                speed_m_s = current.true_airspeed_mps
                speed_error = reference.speed_m_s - speed_m_s
                controls["throttle_ratio"] = _clamp(float(current.required_throttle_ratio) + 0.03 * speed_error, 0.0, 1.0)
                gamma_error = reference.flight_path_angle_rad - state["flight_path_angle_rad"]
                gamma_rate = _clamp(gamma_error / self.flight_path_time_constant_s, -self.maximum_flight_path_rate_rad_s, self.maximum_flight_path_rate_rad_s)
                heading_error = _wrap(reference.heading_rad - state["heading_rad"])
                heading_rate = _clamp(heading_error / self.heading_time_constant_s, -self.maximum_turn_rate_rad_s, self.maximum_turn_rate_rad_s)
                controls["flight_path_angle_rad"] = state["flight_path_angle_rad"]
                controls["vertical_speed_mps"] = speed_m_s * math.sin(state["flight_path_angle_rad"])
                dt = min(self.dt_s, horizon - time_s)
                observables = self._point_observables(state, controls)
                rows.append(self._sample(time_s, state, north_m, east_m, speed_m_s, reference, observables, controls))
                if dt <= 0.0:
                    break
                state["heading_rad"] = _wrap(state["heading_rad"] + heading_rate * dt)
                state["flight_path_angle_rad"] = _clamp(state["flight_path_angle_rad"] + gamma_rate * dt, -0.4, 0.4)
                if self.mode == "point_mass_3dof":
                    point_model = self.model
                    assert isinstance(point_model, A320OpenAPModel)
                    derivatives = point_model.point_mass_derivatives(
                        state,
                        {"throttle_ratio": controls["throttle_ratio"], "flight_path_angle_rad": state["flight_path_angle_rad"]},
                        thrust_mode="climb" if abs(controls["vertical_speed_mps"]) > 1.0e-6 else "cruise",
                    )
                else:
                    model = self.model
                    assert isinstance(model, A320Pseudo6DOFModel)
                    target_roll = reference.bank_rad
                    target_pitch = reference.flight_path_angle_rad + self._trim_alpha
                    target_yaw = reference.heading_rad
                    controls["aileron_rad"] = _clamp(float(self.trim.controls.get("aileron_rad", 0.0)) + 0.08 * (target_roll - state["roll_rad"]) - 0.04 * state["roll_rate_rad_s"], -0.05, 0.05)
                    controls["elevator_rad"] = _clamp(float(self.trim.controls.get("elevator_rad", 0.0)) + 0.08 * (target_pitch - state["pitch_rad"]) - 0.04 * state["pitch_rate_rad_s"], -0.05, 0.05)
                    controls["rudder_rad"] = _clamp(float(self.trim.controls.get("rudder_rad", 0.0)) + 0.08 * _wrap(target_yaw - state["yaw_rad"]) - 0.04 * state["yaw_rate_rad_s"], -0.05, 0.05)
                    controls["bank_angle_rad"] = state["roll_rad"]
                    pseudo_controls = dict(controls)
                    pseudo_controls["vertical_speed_mps"] = abs(pseudo_controls["vertical_speed_mps"])
                    # Evaluate the source-scaled rotational channels for the
                    # telemetry path, but use the declared pseudo-6DOF
                    # attitude-response law for mission propagation.  This is
                    # the same boundary used by the other reduced runners: it
                    # prevents a surrogate moment pulse from being mistaken
                    # for a stable source-exact aircraft controller.
                    _ = model.six_dof_derivatives(
                        state,
                        pseudo_controls,
                        thrust_mode="climb" if abs(controls["vertical_speed_mps"]) > 1.0e-6 else "cruise",
                    )
                    derivatives = model.openap.point_mass_derivatives(
                        state,
                        {"throttle_ratio": controls["throttle_ratio"], "flight_path_angle_rad": state["flight_path_angle_rad"]},
                        thrust_mode="climb" if abs(controls["vertical_speed_mps"]) > 1.0e-6 else "cruise",
                    )
                previous_east = east_m
                if not terminal_hold:
                    north_m += speed_m_s * math.cos(state["flight_path_angle_rad"]) * math.cos(state["heading_rad"]) * dt
                    east_m += speed_m_s * math.cos(state["flight_path_angle_rad"]) * math.sin(state["heading_rad"]) * dt
                    if (
                        time_s >= self.route.declared_duration_s
                        and previous_east < 0.0 <= east_m
                        and abs(north_m) <= self.route.gate_corridor_m
                    ):
                        terminal_hold = True
                state["altitude_m"] += derivatives["altitude_m"] * dt
                state["mach"] += derivatives["mach"] * dt
                state["mass_kg"] = max(1.0, state["mass_kg"] + derivatives["mass_kg"] * dt)
                state["range_m"] += derivatives["range_m"] * dt
                if self.mode == "pseudo_6dof_kinematic_bridge":
                    target_roll = reference.bank_rad
                    target_pitch = reference.flight_path_angle_rad + self._trim_alpha
                    target_yaw = reference.heading_rad
                    if self.response_profile is None:
                        rate_targets = {
                            "roll_rate_rad_s": _clamp(_wrap(target_roll - state["roll_rad"]) / self.attitude_time_constant_s, -0.25, 0.25),
                            "pitch_rate_rad_s": _clamp((target_pitch - state["pitch_rad"]) / self.attitude_time_constant_s, -0.25, 0.25),
                            "yaw_rate_rad_s": _clamp(_wrap(target_yaw - state["yaw_rad"]) / self.attitude_time_constant_s, -0.25, 0.25),
                        }
                        for name, target in rate_targets.items():
                            state[name] = _slew(state[name], target, self.attitude_time_constant_s, dt)
                        state["roll_rad"] += state["roll_rate_rad_s"] * dt
                        state["pitch_rad"] += state["pitch_rate_rad_s"] * dt
                        state["yaw_rad"] = _wrap(state["yaw_rad"] + state["yaw_rate_rad_s"] * dt)
                    else:
                        roll_state = step_bounded_axis_response(
                            self.response_profile.response["roll"],
                            AxisResponseState(state["roll_rad"], state["roll_rate_rad_s"]),
                            target_roll,
                            dt,
                        )
                        pitch_state = step_bounded_axis_response(
                            self.response_profile.response["pitch"],
                            AxisResponseState(state["pitch_rad"], state["pitch_rate_rad_s"]),
                            target_pitch,
                            dt,
                        )
                        yaw_state = step_bounded_axis_response(
                            self.response_profile.response["yaw"],
                            AxisResponseState(state["yaw_rad"], state["yaw_rate_rad_s"]),
                            target_yaw,
                            dt,
                        )
                        state["roll_rad"], state["roll_rate_rad_s"] = roll_state.angle_rad, roll_state.rate_rad_s
                        state["pitch_rad"], state["pitch_rate_rad_s"] = pitch_state.angle_rad, pitch_state.rate_rad_s
                        state["yaw_rad"], state["yaw_rate_rad_s"] = _wrap(yaw_state.angle_rad), yaw_state.rate_rad_s
                    state["alpha_rad"] = _slew(state["alpha_rad"], self._trim_alpha, self.attitude_time_constant_s, dt)
                    state["beta_rad"] = _slew(state["beta_rad"], 0.0, self.attitude_time_constant_s, dt)
                time_s = min(horizon, time_s + dt)
                if not all(math.isfinite(value) for value in state.values()) or not all(math.isfinite(value) for value in (north_m, east_m, time_s)):
                    raise FloatingPointError("A320 racetrack state became non-finite")
            except (FloatingPointError, ValueError, KeyError) as error:
                numerical_valid = False
                failure = str(error)
                break
        return A320RacetrackRun(self.mode, tuple(rows), numerical_valid, failure)
        ####


__all__ = ["A320RacetrackMode", "A320RacetrackRun", "A320RacetrackRunner"]
####
