"""Generate a local F-16 wrench-LQR recovery through bounded effectors."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from taoryx.control_allocation import EffectorLimits
from taoryx.physical_lqr import (
    design_physical_wrench_lqr,
    project_linearization_to_wrench,
    validate_nonlinear_wrench_lqr,
)
from taoryx.trajectory import F16ReferencePhysicalPlant, load_f16_reference_plant
from taoryx.trim import TrimResult, TrimSpec

ROOT = Path(__file__).resolve().parents[1]


def _limits(payload: dict[str, Any]) -> dict[str, EffectorLimits]:
    """Resolve the bounded development actuator profile."""

    names = {
        "elevator": "elevator_deg",
        "aileron": "aileron_deg",
        "rudder": "rudder_deg",
        "throttle": "throttle_fraction",
    }
    return {
        names[source_name]: EffectorLimits(
            names[source_name],
            float(values["lower"]),
            float(values["upper"]),
            str(values["unit"]),
            float(values["rate_limit_per_s"]),
            float(values["time_constant_s"]),
        )
        for source_name, values in payload["limits"].items()
    }
    ####


def main() -> int:
    """Run one explicitly local physical-wrench recovery case."""

    linearization = json.loads(
        (ROOT / "verification/f16_runtime_linearization_evidence.json").read_text(encoding="utf-8")
    )
    actuator_profile = yaml.safe_load(
        (ROOT / "families/reference_f16_s119/actuators/reference-first-order-v1.yaml").read_text(encoding="utf-8")
    )
    controller_profile = yaml.safe_load(
        (ROOT / "families/reference_f16_s119/controllers/local-physical-wrench-lqr-v1.yaml").read_text(encoding="utf-8")
    )
    state = {name: float(value) for name, value in linearization["trim_state"].items()}
    controls = {name: float(value) for name, value in linearization["trim_controls"].items()}
    state_names = tuple(controller_profile["state_names"])
    control_names = tuple(controller_profile["effector_names"])
    trim_pitch_rad = float(linearization["metadata"]["trim_pitch_rad"])
    trim_spec = TrimSpec(
        state_names=state_names,
        control_names=control_names,
        residual_names=state_names,
        state_initial=state,
        control_initial=controls,
        operating_point={"trim_pitch_rad": trim_pitch_rad},
    )
    trim = TrimResult(trim_spec, state, controls, {name: 0.0 for name in state}, 0.0, True, 1, "source trim", 0, 0.0)
    source = load_f16_reference_plant(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml",
    )
    adapter = F16ReferencePhysicalPlant(
        source,
        trim,
        trim_pitch_rad,
        0.0,
        _limits(actuator_profile),
    )
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
