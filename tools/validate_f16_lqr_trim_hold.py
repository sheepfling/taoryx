"""Generate the bounded local F-16 LQR trim-hold development artifact."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from taoryx.control_allocation import EffectorLimits, advance_actuators
from taoryx.runtime.lqr import LqrController, solve_scaled_continuous_lqr
from taoryx.trajectory import load_f16_reference_plant

ROOT = Path(__file__).resolve().parents[1]
STATE_NAMES = ("u_m_s", "v_m_s", "w_m_s", "p_rad_s", "q_rad_s", "r_rad_s")
CONTROL_NAMES = ("elevator_deg", "aileron_deg", "rudder_deg", "throttle_fraction")
CONTROL_CONFIG_NAMES = {
    "elevator": "elevator_deg",
    "aileron": "aileron_deg",
    "rudder": "rudder_deg",
    "throttle": "throttle_fraction",
}


def _rk4_step(plant: Any, state: dict[str, float], controls: dict[str, float], pitch_rad: float, step_s: float) -> dict[str, float]:
    """Advance the local nonlinear source plant with fixed achieved effectors."""

    def derivative(candidate: dict[str, float]) -> dict[str, float]:
        return dict(plant.state_derivative(candidate, controls, pitch_rad=pitch_rad))

    def add(candidate: dict[str, float], rates: dict[str, float], fraction: float) -> dict[str, float]:
        return {name: candidate[name] + fraction * rates[name] for name in STATE_NAMES}

    first = derivative(state)
    second = derivative(add(state, first, 0.5 * step_s))
    third = derivative(add(state, second, 0.5 * step_s))
    fourth = derivative(add(state, third, step_s))
    return {
        name: state[name] + step_s * (first[name] + 2.0 * second[name] + 2.0 * third[name] + fourth[name]) / 6.0
        for name in STATE_NAMES
    }
    ####


def _limits(payload: dict[str, Any]) -> dict[str, EffectorLimits]:
    """Resolve the explicit bounded actuator overlay into runtime limits."""

    result: dict[str, EffectorLimits] = {}
    for source_name, values in payload["limits"].items():
        canonical = CONTROL_CONFIG_NAMES[source_name]
        result[canonical] = EffectorLimits(
            canonical,
            float(values["lower"]),
            float(values["upper"]),
            str(values["unit"]),
            float(values["rate_limit_per_s"]),
            float(values["time_constant_s"]),
        )
    return result
    ####


def main() -> int:
    """Synthesize, bound, and run one local nonlinear trim-hold case."""

    controller_config = yaml.safe_load(
        (ROOT / "families/reference_f16_s119/controllers/local-lqr-trim-hold-v1.yaml").read_text(encoding="utf-8")
    )
    actuator_config = yaml.safe_load(
        (ROOT / "families/reference_f16_s119/actuators/reference-first-order-v1.yaml").read_text(encoding="utf-8")
    )
    evidence = json.loads(
        (ROOT / "verification/f16_runtime_linearization_evidence.json").read_text(encoding="utf-8")
    )
    plant = load_f16_reference_plant(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml",
    )
    state_trim = {name: float(evidence["trim_state"][name]) for name in STATE_NAMES}
    control_trim = {name: float(evidence["trim_controls"][name]) for name in CONTROL_NAMES}
    limits = _limits(actuator_config)
    lqr = solve_scaled_continuous_lqr(
        evidence["a_matrix"],
        evidence["b_matrix"],
        np.diag(controller_config["q_diagonal"]),
        np.diag(controller_config["r_diagonal"]),
        state_scales=controller_config["state_scales"],
        control_scales=controller_config["control_scales"],
        state_names=STATE_NAMES,
        control_names=CONTROL_NAMES,
    )
    controller = LqrController(
        lqr,
        state_trim=state_trim,
        control_trim=control_trim,
        lower={name: limits[name].lower for name in CONTROL_NAMES},
        upper={name: limits[name].upper for name in CONTROL_NAMES},
    )
    state = dict(state_trim)
    state["u_m_s"] += 5.0
    state["w_m_s"] += 1.0
    state["q_rad_s"] += 0.02
    actual = dict(control_trim)
    step_s = 0.01
    horizon_s = 20.0
    saturation_samples = 0
    rate_limited_samples = 0
    maximum_effector_rate = 0.0
    maximum_state_error = 0.0
    sample_count = int(round(horizon_s / step_s))
    for _ in range(sample_count):
        command = controller.command(state)
        actuator = advance_actuators(limits, command.controls, actual, step_s)
        actual = dict(actuator.actual_positions)
        saturation_samples += int(bool(actuator.position_saturated or actuator.rate_limited))
        rate_limited_samples += int(bool(actuator.rate_limited))
        maximum_effector_rate = max(maximum_effector_rate, *(abs(value) for value in actuator.rates_per_s.values()))
        state = _rk4_step(
            plant,
            state,
            actual,
            float(evidence["metadata"]["trim_pitch_rad"]),
            step_s,
        )
        maximum_state_error = max(maximum_state_error, *(abs(state[name] - state_trim[name]) for name in STATE_NAMES))
    final_error = {name: state[name] - state_trim[name] for name in STATE_NAMES}
    final_error_norm = float(np.linalg.norm(np.asarray(tuple(final_error.values()), dtype=float)))
    saturation_fraction = saturation_samples / sample_count
    report = {
        "schema_version": "taoryx.f16-lqr-trim-hold-evidence/v1",
        "status": "development_screen_passed" if lqr.hurwitz and final_error_norm < 0.5 else "development_screen_failed",
        "claim_boundary": (
            "local nonlinear trim-hold screen using plant-derived LQR, direct physical effector commands, "
            "and bounded engineering actuator overlays; not scheduled control, wrench allocation, or flight qualification"
        ),
        "family_id": "reference_f16_s119",
        "plant_id": evidence["plant_id"],
        "controller_id": controller_config["controller_id"],
        "actuator_profile_id": actuator_config["profile_id"],
        "control_path": "lqr_to_bounded_direct_effectors_to_source_nonlinear_plant",
        "linearization_artifact": "verification/f16_runtime_linearization_evidence.json",
        "closed_loop": {
            "hurwitz": lqr.hurwitz,
            "maximum_real_pole": lqr.maximum_real_pole,
            "condition_number": lqr.condition_number,
            "a_sha256": lqr.a_sha256,
            "b_sha256": lqr.b_sha256,
            "q_sha256": lqr.q_sha256,
            "r_sha256": lqr.r_sha256,
            "k_sha256": lqr.k_sha256,
            "poles": [{"real": float(value.real), "imag": float(value.imag)} for value in lqr.closed_loop_eigenvalues],
        },
        "initial_perturbation": {"u_m_s": 5.0, "w_m_s": 1.0, "q_rad_s": 0.02},
        "trim_state": state_trim,
        "trim_controls": control_trim,
        "final_state": state,
        "final_error": final_error,
        "metrics": {
            "horizon_s": horizon_s,
            "step_s": step_s,
            "final_error_norm_mixed_units": final_error_norm,
            "final_error_limit_mixed_units": 0.5,
            "maximum_state_error_mixed_units": maximum_state_error,
            "saturation_fraction": saturation_fraction,
            "rate_limited_fraction": rate_limited_samples / sample_count,
            "maximum_effector_rate": maximum_effector_rate,
        },
        "actuator_limits": actuator_config["limits"],
        "evidence": {
            "linearization": evidence["claim_boundary"],
            "actuators": actuator_config["evidence"],
            "controller": controller_config["evidence"],
        },
    }
    output = ROOT / "verification/f16_lqr_trim_hold_evidence.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "development_screen_passed" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
