"""Source-backed operating-point resolution for the F-16 S-119 family.

The public F-16 package supplies a nonlinear aerodynamic/propulsion plant, not
a complete flight-control schedule.  This module therefore resolves bounded
steady, level-flight fixtures directly against that plant.  The fixtures are
useful for local linearization and allocator evidence; they do not imply a
scheduled controller between points.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from ..trim import TrimResult, TrimSpec, solve_trim
from .f16_reference import F16ReferencePlant


@dataclass(frozen=True, slots=True)
class F16OperatingPoint:
    """Resolved source-backed fixed-altitude, fixed-airspeed equilibrium."""

    point_id: str
    altitude_m: float
    true_airspeed_m_s: float
    trim: TrimResult
    trim_pitch_rad: float
    body_state: dict[str, float]
    controls: dict[str, float]
    residuals: dict[str, float]

    @property
    def mach(self) -> float:
        """Return the atmosphere-derived Mach number at this point."""

        return self.true_airspeed_m_s / self._speed_of_sound_m_s
        ####

    _speed_of_sound_m_s: float

    @property
    def max_residual(self) -> float:
        """Return the largest unscaled equilibrium residual."""

        return max(abs(value) for value in self.residuals.values())
        ####

    def as_dict(self) -> dict[str, Any]:
        """Return a stable, JSON-compatible operating-point record."""

        return {
            "id": self.point_id,
            "environment": {
                "geometric_altitude_m": self.altitude_m,
                "true_airspeed_m_s": self.true_airspeed_m_s,
                "mach": self.mach,
                "gravity_m_s2": self.trim.spec.operating_point.get("gravity_m_s2", 9.80665),
            },
            "state": {
                "alpha_deg": float(self.trim.state["alpha_deg"]),
                "body_velocity_m_s": {
                    "u": self.body_state["u_m_s"],
                    "v": self.body_state["v_m_s"],
                    "w": self.body_state["w_m_s"],
                },
                "body_rates_rad_s": {
                    "p": self.body_state["p_rad_s"],
                    "q": self.body_state["q_rad_s"],
                    "r": self.body_state["r_rad_s"],
                },
                "attitude": {
                    "roll_rad": 0.0,
                    "pitch_rad": self.trim_pitch_rad,
                    "yaw_rad": 0.0,
                },
            },
            "controls": {
                "elevator_deg": self.controls["elevator_deg"],
                "aileron_deg": self.controls["aileron_deg"],
                "rudder_deg": self.controls["rudder_deg"],
                "throttle_fraction": self.controls["throttle_fraction"],
                "source_powerLeverAngle_pct": 100.0 * self.controls["throttle_fraction"],
            },
            "residuals": dict(self.residuals),
            "max_residual": self.max_residual,
            "scaled_residual_norm": float(self.trim.scaled_residual_norm),
            "solver": {
                "success": self.trim.success,
                "status": self.trim.status,
                "iterations": self.trim.iterations,
                "message": self.trim.message,
            },
            "validity": "source_backed_local_equilibrium",
            "claim_boundary": (
                "fixed-altitude, fixed-airspeed source equilibrium for local plant and control evidence; "
                "not a scheduled controller or flight-envelope qualification"
            ),
        }
        ####


def solve_f16_source_trim(
    source: F16ReferencePlant,
    *,
    point_id: str,
    altitude_m: float,
    true_airspeed_m_s: float,
    initial_alpha_deg: float,
    initial_elevator_deg: float,
    initial_throttle_fraction: float,
    gravity_m_s2: float | None = None,
) -> F16OperatingPoint:
    """Solve a bounded level-flight trim using the runtime source plant."""

    if not point_id.strip():
        raise ValueError("F-16 operating-point id must not be blank")
    if not math.isfinite(altitude_m) or altitude_m < 0.0:
        raise ValueError("F-16 operating-point altitude must be finite and nonnegative")
    if not math.isfinite(true_airspeed_m_s) or true_airspeed_m_s <= 0.0:
        raise ValueError("F-16 operating-point airspeed must be finite and positive")
    gravity = source.gravity_m_s2 if gravity_m_s2 is None else gravity_m_s2
    if not math.isfinite(gravity) or gravity <= 0.0:
        raise ValueError("F-16 operating-point gravity must be finite and positive")

    def evaluate(state: Mapping[str, float], controls: Mapping[str, float]) -> dict[str, float]:
        alpha_rad = math.radians(state["alpha_deg"])
        body = {
            "u_m_s": true_airspeed_m_s * math.cos(alpha_rad),
            "v_m_s": 0.0,
            "w_m_s": true_airspeed_m_s * math.sin(alpha_rad),
            "p_rad_s": 0.0,
            "q_rad_s": 0.0,
            "r_rad_s": 0.0,
        }
        full_controls = {
            "elevator_deg": controls["elevator_deg"],
            "aileron_deg": 0.0,
            "rudder_deg": 0.0,
            "throttle_fraction": controls["throttle_fraction"],
        }
        loads = source.evaluate_loads(body, full_controls, altitude_m=altitude_m)
        return {
            "force_x_n": loads["total_force_x_n"] - source.mass_kg * gravity * math.sin(alpha_rad),
            "force_z_n": loads["total_force_z_n"] + source.mass_kg * gravity * math.cos(alpha_rad),
            "moment_y_nm": loads["total_moment_y_nm"],
        }
        ####

    spec = TrimSpec(
        state_names=("alpha_deg",),
        control_names=("elevator_deg", "throttle_fraction"),
        residual_names=("force_x_n", "force_z_n", "moment_y_nm"),
        state_initial={"alpha_deg": initial_alpha_deg},
        control_initial={
            "elevator_deg": initial_elevator_deg,
            "throttle_fraction": initial_throttle_fraction,
        },
        state_lower={"alpha_deg": -10.0},
        state_upper={"alpha_deg": 15.0},
        control_lower={"elevator_deg": -24.0, "throttle_fraction": 0.0},
        control_upper={"elevator_deg": 24.0, "throttle_fraction": 1.0},
        residual_scales={"force_x_n": 1.0e4, "force_z_n": 1.0e4, "moment_y_nm": 1.0e5},
        operating_point={
            "altitude_m": altitude_m,
            "true_airspeed_m_s": true_airspeed_m_s,
            "gravity_m_s2": gravity,
        },
    )
    result = solve_trim(
        spec,
        evaluate,
        max_nfev=500,
        residual_tolerance=1.0e-7,
        acceptance_tolerance=1.0e-6,
    )
    if not result.success:
        raise ValueError(f"F-16 operating-point trim failed for {point_id}: {result.message}")
    alpha_rad = math.radians(float(result.state["alpha_deg"]))
    body_state = {
        "u_m_s": true_airspeed_m_s * math.cos(alpha_rad),
        "v_m_s": 0.0,
        "w_m_s": true_airspeed_m_s * math.sin(alpha_rad),
        "p_rad_s": 0.0,
        "q_rad_s": 0.0,
        "r_rad_s": 0.0,
    }
    controls = {
        "elevator_deg": float(result.controls["elevator_deg"]),
        "aileron_deg": 0.0,
        "rudder_deg": 0.0,
        "throttle_fraction": float(result.controls["throttle_fraction"]),
    }
    residuals = {name: float(value) for name, value in result.residuals.items()}
    return F16OperatingPoint(
        point_id=point_id,
        altitude_m=altitude_m,
        true_airspeed_m_s=true_airspeed_m_s,
        trim=result,
        trim_pitch_rad=alpha_rad,
        body_state=body_state,
        controls=controls,
        residuals=residuals,
        _speed_of_sound_m_s=float(source.atmosphere.evaluate(altitude_m)["speed_of_sound_m_s"]),
    )
    ####


def runtime_trim_result(point: F16OperatingPoint) -> TrimResult:
    """Promote a solved fixture into the full runtime state/control contract."""

    state = dict(point.body_state)
    controls = dict(point.controls)
    spec = TrimSpec(
        state_names=tuple(state),
        control_names=tuple(controls),
        residual_names=tuple(state),
        state_initial=state,
        control_initial=controls,
        operating_point={
            "altitude_m": point.altitude_m,
            "true_airspeed_m_s": point.true_airspeed_m_s,
            "trim_pitch_rad": point.trim_pitch_rad,
        },
    )
    return TrimResult(
        spec,
        state,
        controls,
        {name: 0.0 for name in state},
        0.0,
        True,
        1,
        "source-backed operating point",
        0,
        0.0,
    )
    ####


__all__ = ["F16OperatingPoint", "runtime_trim_result", "solve_f16_source_trim"]
####
