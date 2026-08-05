"""Generate a local F-16 wrench-LQR recovery through bounded effectors."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from taoryx.physical_lqr import (
    design_physical_wrench_lqr,
    project_linearization_to_wrench,
    validate_nonlinear_wrench_lqr,
)
from taoryx.source_f16 import build_f16_source_physical_plant

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    """Run one explicitly local physical-wrench recovery case."""

    actuator_profile = yaml.safe_load(
        (ROOT / "families/reference_f16_s119/actuators/reference-first-order-v1.yaml").read_text(encoding="utf-8")
    )
    controller_profile = yaml.safe_load(
        (ROOT / "families/reference_f16_s119/controllers/local-physical-wrench-lqr-v1.yaml").read_text(encoding="utf-8")
    )
    adapter = build_f16_source_physical_plant()
    trim = adapter.trim_result
    state = dict(trim.state)
    controls = dict(trim.controls)
    state_names = tuple(controller_profile["state_names"])
    control_names = tuple(controller_profile["effector_names"])
    effectiveness = adapter.effectiveness(state, controls)
    projection = project_linearization_to_wrench(
        adapter.linearize(trim, {"state_step": 1.0e-5, "control_step": 1.0e-5}),
        effectiveness,
        state_names=state_names,
        wrench_names=adapter.wrench_names,
        effector_names=control_names,
    )
    design = design_physical_wrench_lqr(
        controller_profile["controller_id"],
        projection,
        q_diagonal=controller_profile["q_diagonal"],
        r_diagonal=controller_profile["r_diagonal"],
        state_scales=controller_profile["state_scales"],
        wrench_scales=controller_profile["wrench_scales"],
    )
    initial_state = dict(state)
    initial_state["u_m_s"] += 0.1
    initial_state["w_m_s"] += 0.02
    initial_state["q_rad_s"] += 0.0004
    validation = validate_nonlinear_wrench_lqr(
        adapter,
        trim,
        design,
        initial_state=initial_state,
        duration_s=5.0,
        dt_s=0.02,
    )
    report = validation.as_dict()
    report.update(
        {
            "status": "development_screen_passed",
            "family_id": "reference_f16_s119",
            "plant_id": "reference-f16-s119-source-runtime-plant",
            "actuator_profile_id": actuator_profile["profile_id"],
            "tuning_profile": "state_and_wrench_balanced_q10_r0p01",
            "control_path": "lqr_to_desired_wrench_to_bounded_effectors_to_source_nonlinear_plant",
            "claim_boundary": (
                "local nonlinear trim-hold screen using plant-derived wrench LQR, source-load effectiveness, "
                "and bounded engineering actuator overlays; not scheduled control, source-validated actuator "
                "dynamics, or flight qualification"
            ),
            "perturbation_contract": {
                "u_m_s": 0.1,
                "w_m_s": 0.02,
                "q_rad_s": 0.0004,
                "validity": "single local operating-point witness; not a broad envelope",
            },
            "effectiveness_source": effectiveness.source,
        }
    )
    output = ROOT / "verification/f16_physical_wrench_lqr_evidence.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if validation.allocation_statuses == ("feasible",) else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
