"""NESC source-translation plus scheduled-attitude pseudo-6DOF bridge.

The retained NESC evidence contains a qualified translation replay and stage
history, but no independent attitude-response comparison channel.  This
bridge therefore composes the verified translation history with the declared
Alpha 3 response law.  It is executable development evidence, not a source-
exact attitude or gimbal model.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

from taoryx_reference_models.resources import model_resource_root

from .pseudo6dof_profiles import Pseudo6DOFProfile, load_pseudo6dof_catalog
from .response_laws import AxisResponseState, step_bounded_axis_response

Vector3 = tuple[float, float, float]
DEFAULT_REDUCTION = model_resource_root() / "verification/daveml_nesc_reduction_qualification.json"


@dataclass(frozen=True, slots=True)
class NESCCompositePseudo6DOFResult:
    """Self-contained source/reduced pseudo-6DOF comparison artifact."""

    profile_id: str
    source_artifact: str
    rows: tuple[dict[str, object], ...]
    checks: tuple[tuple[str, bool], ...]
    response_time_scale: float = 1.0
    initial_attitude_rad: Vector3 = (0.0, 0.0, 0.0)

    @property
    def passed(self) -> bool:
        """Return whether the surrogate stayed numerically and structurally valid."""

        return all(result for _, result in self.checks)
        ####

    def as_dict(self) -> dict[str, object]:
        """Return a deterministic machine-readable result."""

        return {
            "schema": "taoryx.nesc-composite-pseudo6dof/v1alpha1",
            "family_id": "reference_nesc_two_stage_rocket",
            "profile_id": self.profile_id,
            "source_artifact": self.source_artifact,
            "claim_boundary": "qualified source translation replay plus scheduled attitude-response surrogate; not source-exact attitude or gimbal equivalence",
            "checks": {name: result for name, result in self.checks},
            "status": "development_pass" if self.passed else "failed",
            "rows": list(self.rows),
            "response_time_scale": self.response_time_scale,
            "initial_attitude_rad": list(self.initial_attitude_rad),
        }
        ####


def _profile() -> Pseudo6DOFProfile:
    """Resolve the canonical NESC profile."""

    _, profile = load_pseudo6dof_catalog().for_family("reference_nesc_two_stage_rocket")
    return profile
    ####


def _scaled_profile(profile: Pseudo6DOFProfile, response_time_scale: float) -> Pseudo6DOFProfile:
    """Return a bounded response-law variant for development sensitivity tests."""

    if not math.isfinite(response_time_scale) or response_time_scale <= 0.0:
        raise ValueError("NESC response_time_scale must be finite and positive")
    if response_time_scale == 1.0:
        return profile
    response = {
        name: axis.model_copy(update={"time_constant_s": axis.time_constant_s * response_time_scale})
        for name, axis in profile.response.items()
    }
    phase_response = {
        phase: {
            name: axis.model_copy(update={"time_constant_s": axis.time_constant_s * response_time_scale})
            for name, axis in axes.items()
        }
        for phase, axes in profile.phase_response.items()
    }
    return profile.model_copy(update={"response": response, "phase_response": phase_response})
    ####


def _norm(value: Vector3) -> float:
    return math.sqrt(sum(component * component for component in value))
    ####


def _attitude_target(velocity: Vector3) -> Vector3:
    horizontal = math.hypot(velocity[0], velocity[1])
    return (0.0, math.atan2(velocity[2], max(horizontal, 1.0e-12)), math.atan2(velocity[1], velocity[0]))
    ####


def _finite_vector(value: object) -> Vector3:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError("NESC translation vector must contain three values")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError("NESC translation vector must be finite")
    return result  # type: ignore[return-value]
    ####


def build_nesc_composite_pseudo6dof(
    source_artifact: str | Path = DEFAULT_REDUCTION,
    *,
    profile: Pseudo6DOFProfile | None = None,
    response_time_scale: float = 1.0,
    initial_attitude_rad: Vector3 = (0.0, 0.0, 0.0),
) -> NESCCompositePseudo6DOFResult:
    """Compose source translation with the declared bounded response law."""

    selected = _scaled_profile(_profile() if profile is None else profile, response_time_scale)
    if selected.id != "nesc_rocket.attitude_response_p6dof.v1":
        raise ValueError("NESC composite requires the canonical NESC pseudo profile")
    payload = json.loads(Path(source_artifact).read_text(encoding="utf-8"))
    source_rows = payload.get("history")
    if not isinstance(source_rows, list) or not source_rows:
        raise ValueError("NESC reduction artifact has no source history")
    if not all(math.isfinite(value) for value in initial_attitude_rad):
        raise ValueError("NESC initial_attitude_rad must be finite")
    attitude: Vector3 = initial_attitude_rad
    rates: Vector3 = (0.0, 0.0, 0.0)
    rows: list[dict[str, object]] = []
    previous_time = float(source_rows[0]["time_s"])
    for source in source_rows:
        if not isinstance(source, dict):
            raise ValueError("NESC source history row must be a mapping")
        time_s = float(source["time_s"])
        velocity = _finite_vector(source["source_velocity_eci_mps"])
        dt_s = max(0.0, time_s - previous_time)
        response_phase, response_axes = selected.response_for_phase(str(source["phase"]))
        axes = (response_axes["roll"], response_axes["pitch"], response_axes["yaw"])
        target = _attitude_target(velocity)
        if dt_s > 0.0:
            response = tuple(
                step_bounded_axis_response(axis, AxisResponseState(angle, rate), command, dt_s)
                for axis, angle, rate, command in zip(axes, attitude, rates, target, strict=True)
            )
            roll_response, pitch_response, yaw_response = response
            attitude = (roll_response.angle_rad, pitch_response.angle_rad, yaw_response.angle_rad)
            rates = (roll_response.rate_rad_s, pitch_response.rate_rad_s, yaw_response.rate_rad_s)
        rows.append(
            {
                "time_s": time_s,
                "phase": str(source["phase"]),
                "position_eci_m": list(_finite_vector(source["source_position_eci_m"])),
                "velocity_eci_mps": list(velocity),
                "mass_kg": float(source["mass_kg"]),
                "commanded_attitude_rad": list(target),
                "achieved_attitude_rad": list(attitude),
                "body_rate_rad_s": list(rates),
                "response_profile_id": selected.id,
                "response_phase": response_phase,
                "response_schedule_applied": response_phase != "default",
                "translation_source_replay": True,
                "attitude_source": "scheduled_response_surrogate",
                "physical_gimbal_allocation": False,
            }
        )
        previous_time = time_s
    stage_sequence = [str(row["phase"]) for row in rows]
    def attitude_row_is_finite(row: dict[str, object]) -> bool:
        attitude = row["achieved_attitude_rad"]
        rates = row["body_rate_rad_s"]
        if not isinstance(attitude, list) or not isinstance(rates, list):
            return False
        return all(math.isfinite(float(value)) for value in (*attitude, *rates))
        ####

    def translation_row_is_valid(row: dict[str, object]) -> bool:
        try:
            speed = _norm(_finite_vector(row["velocity_eci_mps"]))
        except (TypeError, ValueError):
            return False
        return speed >= 0.0
        ####

    checks = (
        ("source_rows_retained", len(rows) == len(source_rows)),
        ("stage_sequence_retained", stage_sequence[0] == "stage1_burn" and stage_sequence[-1] == "orbit_coast"),
        ("attitude_rows_finite", all(attitude_row_is_finite(row) for row in rows)),
        ("profile_identity_retained", all(row["response_profile_id"] == selected.id for row in rows)),
        ("translation_speed_retained", all(translation_row_is_valid(row) for row in rows)),
    )
    return NESCCompositePseudo6DOFResult(
        selected.id,
        str(Path(source_artifact)),
        tuple(rows),
        checks,
        response_time_scale,
        initial_attitude_rad,
    )
    ####


__all__ = ["NESCCompositePseudo6DOFResult", "build_nesc_composite_pseudo6dof"]
