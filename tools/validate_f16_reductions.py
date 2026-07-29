"""Validate local F-16 point-mass and pseudo-6DOF reduction witnesses."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from taoryx.trajectory import (  # noqa: E402
    F16AttitudeResponsePseudo6DOFModel,
    F16PointMass3DOFModel,
    load_f16_reference_plant,
)
from taoryx.trim import TrimResult, TrimSpec  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
PARENT_STATE_NAMES = ("u_m_s", "v_m_s", "w_m_s", "p_rad_s", "q_rad_s", "r_rad_s")
CONTROL_NAMES = ("elevator_deg", "aileron_deg", "rudder_deg", "throttle_fraction")


def _build_case() -> tuple[Any, TrimResult, float]:
    """Load the committed source operating point and local linearization inputs."""

    evidence = json.loads(
        (ROOT / "verification/f16_runtime_linearization_evidence.json").read_text(encoding="utf-8")
    )
    state = {name: float(value) for name, value in evidence["trim_state"].items()}
    controls = {name: float(value) for name, value in evidence["trim_controls"].items()}
    spec = TrimSpec(
        state_names=PARENT_STATE_NAMES,
        control_names=CONTROL_NAMES,
        residual_names=PARENT_STATE_NAMES,
        state_initial=state,
        control_initial=controls,
        operating_point={"trim_pitch_rad": float(evidence["metadata"]["trim_pitch_rad"])},
    )
    trim = TrimResult(spec, state, controls, {name: 0.0 for name in PARENT_STATE_NAMES}, 0.0, True, 1, "source trim", 0, 0.0)
    source = load_f16_reference_plant(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml",
    )
    return source, trim, float(evidence["metadata"]["trim_pitch_rad"])
    ####


def _advance_euler(model: Any, state: dict[str, float], controls: dict[str, float], step_s: float) -> dict[str, float]:
    derivatives = model.state_derivative(state, controls)
    return {name: float(state[name]) + step_s * float(derivatives[name]) for name in state}
    ####


def _parent_step(source: Any, state: dict[str, float], controls: dict[str, float], pitch_rad: float, step_s: float) -> dict[str, float]:
    derivatives = source.state_derivative(state, controls, altitude_m=0.0, pitch_rad=pitch_rad)
    return {name: float(state[name]) + step_s * float(derivatives[name]) for name in PARENT_STATE_NAMES}
    ####


def _pseudo_state(trim: TrimResult, trim_pitch_rad: float) -> dict[str, float]:
    return {
        **{name: float(trim.state[name]) for name in PARENT_STATE_NAMES},
        "roll_rad": 0.0,
        "pitch_rad": trim_pitch_rad,
        "yaw_rad": 0.0,
    }
    ####


def main() -> int:
    """Run trim, force-channel, and local response comparison witnesses."""

    source, trim, trim_pitch_rad = _build_case()
    linearization = source.linearize_local(
        trim.state,
        trim.controls,
        trim_pitch_rad=trim_pitch_rad,
        altitude_m=0.0,
        state_step=1.0e-5,
        control_step=1.0e-5,
    )
    point = F16PointMass3DOFModel(source, trim, trim_pitch_rad)
    pseudo = F16AttitudeResponsePseudo6DOFModel(source, trim, linearization, trim_pitch_rad)

    point_trim_rates = point.state_derivative(trim.state, trim.controls)
    parent_trim_rates = source.state_derivative(trim.state, trim.controls, altitude_m=0.0, pitch_rad=trim_pitch_rad)
    trim_error = {
        name: float(point_trim_rates[name]) - float(parent_trim_rates[name])
        for name in point.state_names
    }

    pulses = {
        "elevator": {"elevator_deg": 0.05},
        "aileron": {"aileron_deg": 0.05},
        "rudder": {"rudder_deg": 0.05},
        "throttle": {"throttle_fraction": 0.002},
    }
    pulse_results: dict[str, dict[str, Any]] = {}
    dt_s = 0.01
    duration_s = 0.20
    for pulse_id, delta in pulses.items():
        controls = dict(trim.controls)
        controls.update({name: controls[name] + value for name, value in delta.items()})
        parent_state = dict(trim.state)
        point_state = {name: float(trim.state[name]) for name in point.state_names}
        pseudo_state = _pseudo_state(trim, trim_pitch_rad)
        maximum_translation_error = 0.0
        for _ in range(int(round(duration_s / dt_s))):
            parent_state = _parent_step(source, parent_state, controls, trim_pitch_rad, dt_s)
            point_state = _advance_euler(point, point_state, controls, dt_s)
            pseudo_state = _advance_euler(pseudo, pseudo_state, controls, dt_s)
            maximum_translation_error = max(
                maximum_translation_error,
                float(np.linalg.norm([point_state[name] - parent_state[name] for name in point.state_names])),
            )
        rate_error = {
            name: float(pseudo_state[name]) - float(parent_state[name])
            for name in ("p_rad_s", "q_rad_s", "r_rad_s")
        }
        source_rate_norm = float(np.linalg.norm([parent_state[name] - trim.state[name] for name in ("p_rad_s", "q_rad_s", "r_rad_s")]))
        error_norm = float(np.linalg.norm(list(rate_error.values())))
        pulse_results[pulse_id] = {
            "delta_controls": delta,
            "duration_s": duration_s,
            "maximum_translation_history_error_m_s": maximum_translation_error,
            "parent_rates_rad_s": {name: parent_state[name] for name in ("p_rad_s", "q_rad_s", "r_rad_s")},
            "pseudo_rates_rad_s": {name: pseudo_state[name] for name in ("p_rad_s", "q_rad_s", "r_rad_s")},
            "rate_error_rad_s": rate_error,
            "rate_error_norm_rad_s": error_norm,
            "parent_rate_change_norm_rad_s": source_rate_norm,
            "relative_rate_error": error_norm / max(source_rate_norm, 1.0e-9),
        }

    point_passed = max(abs(value) for value in trim_error.values()) <= 1.0e-10
    point_window_passed = all(
        float(result["maximum_translation_history_error_m_s"]) <= 0.50 for result in pulse_results.values()
    )
    pseudo_passed = all(float(result["relative_rate_error"]) <= 0.20 for result in pulse_results.values())
    report = {
        "schema_version": "taoryx.f16-reduction-evidence/v1",
        "status": "development_screen_passed" if point_passed and point_window_passed and pseudo_passed else "development_screen_failed",
        "family_id": "reference_f16_s119",
        "parent_plant_id": "reference-f16-s119-source-runtime-plant",
        "reductions": {
            "point_mass_3dof": {
                "model_id": "f16-point-mass-3dof-v1",
                "trim_translational_error_m_s2": trim_error,
                "maximum_absolute_trim_error_m_s2": max(abs(value) for value in trim_error.values()),
                "passed": point_passed,
                "matched_pulse_window_passed": point_window_passed,
                "maximum_translation_history_error_m_s": max(
                    float(result["maximum_translation_history_error_m_s"]) for result in pulse_results.values()
                ),
                "claim": "local source-force translational reduction at one fixed operating point",
                "nonclaims": ["attitude dynamics", "aerodynamic moments", "body rates", "actuator dynamics", "flight qualification"],
            },
            "pseudo_6dof": {
                "model_id": "f16-attitude-response-pseudo6dof-v1",
                "response_law": "source-derived-local-linear-rate-response-plus-quaternion-kinematics",
                "pulses": pulse_results,
                "maximum_relative_rate_error": max(float(result["relative_rate_error"]) for result in pulse_results.values()),
                "passed": pseudo_passed,
                "claim": "local response-bridge pulse comparison against the source plant",
                "nonclaims": ["full moment balance", "physical actuator states", "source controller equivalence", "flight qualification"],
            },
        },
        "operating_point": {
            "trim_state": dict(trim.state),
            "trim_controls": dict(trim.controls),
            "trim_pitch_rad": trim_pitch_rad,
            "altitude_m": 0.0,
            "step_s": dt_s,
        },
        "claim_boundary": "development reduction screen; reduction contracts remain equivalence_pending until mission-window comparison is added",
    }
    output = ROOT / "verification/f16_reduction_evidence.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if point_passed and point_window_passed and pseudo_passed else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
