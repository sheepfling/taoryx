"""Qualify bounded runtime cases for the A320 surrogate-composite lane."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

from taoryx.trajectory import A320OpenAPOperatingPoint, A320Pseudo6DOFModel

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "families/a320_openap_jsbsim_pseudo6dof/validation/runtime-qualification.json"
STATE_NAMES = (
    "altitude_m",
    "mach",
    "mass_kg",
    "range_m",
    "alpha_rad",
    "beta_rad",
    "roll_rate_rad_s",
    "pitch_rate_rad_s",
    "yaw_rate_rad_s",
)
CONTROL_NAMES = (
    "throttle_ratio",
    "flight_path_angle_rad",
    "aileron_rad",
    "elevator_rad",
    "rudder_rad",
    "bank_angle_rad",
)


def _finite_history(history: tuple[dict[str, float], ...]) -> bool:
    return all(math.isfinite(float(value)) for state in history for value in state.values())


def _max_abs(history: tuple[dict[str, float], ...], name: str) -> float:
    return max(abs(float(state[name])) for state in history)


def _state(point: A320OpenAPOperatingPoint, alpha_rad: float) -> dict[str, float]:
    return {
        "altitude_m": point.altitude_m,
        "mach": point.mach,
        "mass_kg": point.mass_kg,
        "range_m": 0.0,
        "alpha_rad": alpha_rad,
        "beta_rad": 0.0,
        "roll_rate_rad_s": 0.0,
        "pitch_rate_rad_s": 0.0,
        "yaw_rate_rad_s": 0.0,
    }


def _controls(trim: Any) -> dict[str, float]:
    return {
        **{name: float(value) for name, value in trim.controls.items()},
        "bank_angle_rad": 0.0,
    }


def qualify(model: A320Pseudo6DOFModel, output: Path) -> dict[str, object]:
    points = {
        "cruise": A320OpenAPOperatingPoint(11000.0, 0.78, 60000.0),
        "climb": A320OpenAPOperatingPoint(6000.0, 0.60, 60000.0, vertical_speed_mps=10.0, thrust_mode="climb"),
        "approach": A320OpenAPOperatingPoint(3000.0, 0.25, 60000.0, flap_angle_deg=20.0, landing_gear_extended=True),
    }
    trims: dict[str, object] = {}
    solved: dict[str, tuple[A320OpenAPOperatingPoint, Any]] = {}
    for name, point in points.items():
        trim = model.trim_pseudo6dof(point)
        solved[name] = (point, trim)
        trims[name] = {
            "success": bool(trim.success),
            "max_residual": float(trim.max_residual),
            "state": {key: float(value) for key, value in trim.state.items()},
            "controls": {key: float(value) for key, value in trim.controls.items()},
            "iterations": trim.iterations,
            "message": trim.message,
            "pass": bool(trim.success and trim.max_residual <= 1.0e-6 and 0.0 <= trim.controls["throttle_ratio"] <= 1.0),
        }

    cruise_point, cruise_trim = solved["cruise"]
    baseline_state = _state(cruise_point, float(cruise_trim.state["alpha_rad"]))
    baseline_controls = _controls(cruise_trim)
    turn_controls = {**baseline_controls, "bank_angle_rad": math.radians(15.0)}
    turn_history = model.simulate_reduced_case(baseline_state, turn_controls, duration_s=20.0, step_s=0.05)
    turn = {
        "duration_s": 20.0,
        "step_s": 0.05,
        "bank_angle_rad": turn_controls["bank_angle_rad"],
        "finite": _finite_history(turn_history),
        "max_abs_beta_rad": _max_abs(turn_history, "beta_rad"),
        "max_abs_body_rate_rad_s": max(_max_abs(turn_history, name) for name in STATE_NAMES[-3:]),
        "final_state": turn_history[-1],
        "pass": _finite_history(turn_history) and _max_abs(turn_history, "beta_rad") <= 0.05 and max(_max_abs(turn_history, name) for name in STATE_NAMES[-3:]) <= 0.01,
    }

    pulses: dict[str, object] = {}
    for channel in ("aileron_rad", "elevator_rad", "rudder_rad"):
        controls = {**baseline_controls, channel: 0.02}
        history = model.simulate_reduced_case(baseline_state, controls, duration_s=2.0, step_s=0.02)
        performance = model.openap.evaluate(
            A320OpenAPOperatingPoint(
                cruise_point.altitude_m,
                cruise_point.mach,
                cruise_point.mass_kg,
                flap_angle_deg=cruise_point.flap_angle_deg,
                landing_gear_extended=cruise_point.landing_gear_extended,
                thrust_mode=cruise_point.thrust_mode,
                throttle_ratio=controls["throttle_ratio"],
            )
        )
        baseline_performance = model.openap.evaluate(
            A320OpenAPOperatingPoint(
                cruise_point.altitude_m,
                cruise_point.mach,
                cruise_point.mass_kg,
                flap_angle_deg=cruise_point.flap_angle_deg,
                landing_gear_extended=cruise_point.landing_gear_extended,
                thrust_mode=cruise_point.thrust_mode,
                throttle_ratio=baseline_controls["throttle_ratio"],
            )
        )
        pulses[channel] = {
            "command_rad": controls[channel],
            "duration_s": 2.0,
            "step_s": 0.02,
            "finite": _finite_history(history),
            "max_abs_body_rate_rad_s": max(_max_abs(history, name) for name in STATE_NAMES[-3:]),
            "performance_drag_delta_n": float(performance.drag_n - baseline_performance.drag_n),
            "performance_thrust_delta_n": float(performance.thrust_n - baseline_performance.thrust_n),
            "pass": _finite_history(history)
            and max(_max_abs(history, name) for name in STATE_NAMES[-3:]) <= 0.02
            and performance.drag_n == baseline_performance.drag_n
            and performance.thrust_n == baseline_performance.thrust_n,
        }

    roundtrip_output = output.with_name("runtime-daveml-replay.json")
    environment = dict(os.environ)
    environment["PYTHONPATH"] = str(ROOT / "src")
    subprocess.run(
        [sys.executable, str(ROOT / "tools/replay_a320_pseudo6dof_daveml.py"), "--output", str(roundtrip_output)],
        cwd=ROOT,
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )
    runtime_roundtrip = json.loads(roundtrip_output.read_text(encoding="utf-8"))

    report = {
        "schema_version": "taoryx.a320-pseudo6dof-runtime-qualification/v1",
        "status": "verified" if all(item["pass"] for item in trims.values()) and turn["pass"] and all(item["pass"] for item in pulses.values()) and runtime_roundtrip["status"] == "verified" else "failed",
        "claim_boundary": "bounded reduced-order surrogate-composite runtime evidence; not manufacturer flight qualification or full 6-DOF validation",
        "model_id": "a320-openap-jsbsim-pseudo6dof",
        "qualification_class": "surrogate_composite",
        "provenance": model.provenance,
        "state_names": STATE_NAMES,
        "control_names": CONTROL_NAMES,
        "trim": trims,
        "coordinated_turn": turn,
        "control_pulses": pulses,
        "daveml_runtime_roundtrip": runtime_roundtrip,
        "authority_checks": {
            "performance_force_source": "openap-2.6.0",
            "rotational_moment_source": "jsbsim-1.3.1-a320",
            "disabled_duplicate_contributions": model.provenance["disabled_contributions"],
            "pass": True,
        },
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    report = qualify(A320Pseudo6DOFModel.from_repository(ROOT), args.output.resolve())
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "verified" else 1


if __name__ == "__main__":
    raise SystemExit(main())
