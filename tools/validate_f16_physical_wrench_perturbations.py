"""Run the bounded local F-16 wrench-LQR perturbation matrix."""

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


def _build_case() -> tuple[F16ReferencePhysicalPlant, TrimResult, Any]:
    """Build the shared source plant, trim, and physical wrench LQR."""

    evidence = json.loads(
        (ROOT / "verification/f16_runtime_linearization_evidence.json").read_text(encoding="utf-8")
    )
    actuator_profile = yaml.safe_load(
        (ROOT / "families/reference_f16_s119/actuators/reference-first-order-v1.yaml").read_text(encoding="utf-8")
    )
    controller_profile = yaml.safe_load(
        (ROOT / "families/reference_f16_s119/controllers/local-physical-wrench-lqr-v1.yaml").read_text(encoding="utf-8")
    )
    state = {name: float(value) for name, value in evidence["trim_state"].items()}
    controls = {name: float(value) for name, value in evidence["trim_controls"].items()}
    names = {
        "elevator": "elevator_deg",
        "aileron": "aileron_deg",
        "rudder": "rudder_deg",
        "throttle": "throttle_fraction",
    }
    dynamics = actuator_profile.get("dynamics", {})
    default_time_constant = float(dynamics.get("time_constant_s", 0.0))
    limits = {}
    for source_name, values in actuator_profile["limits"].items():
        position = values.get("position")
        if position is None:
            position = [values["lower"], values["upper"]]
        rate = values.get("rate_per_s", values.get("rate_limit_per_s"))
        unit = values.get("unit", "fraction" if source_name == "throttle" else "deg")
        limits[names[source_name]] = EffectorLimits(
            names[source_name],
            float(position[0]),
            float(position[1]),
            str(unit),
            float(rate) if rate is not None else None,
            float(values.get("time_constant_s", default_time_constant)),
        )
    state_names = tuple(controller_profile["state_names"])
    control_names = tuple(controller_profile["effector_names"])
    trim_pitch_rad = float(evidence["metadata"]["trim_pitch_rad"])
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
    adapter = F16ReferencePhysicalPlant(source, trim, trim_pitch_rad, 0.0, limits)
    projection = project_linearization_to_wrench(
        adapter.linearize(trim, {"state_step": 1.0e-5, "control_step": 1.0e-5}),
        adapter.effectiveness(state, controls),
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
    return adapter, trim, design
    ####


def main() -> int:
    """Run isolated, reversal, and coupled local perturbation witnesses."""

    adapter, trim, design = _build_case()
    cases = {
        "u_plus": {"u_m_s": 0.1},
        "w_plus": {"w_m_s": 0.02},
        "q_plus": {"q_rad_s": 0.0004},
        "coupled_reversal": {"u_m_s": -0.1, "w_m_s": -0.02, "q_rad_s": -0.0004},
        "lateral_coupled": {"v_m_s": 0.1, "p_rad_s": 0.0002, "r_rad_s": -0.0002},
    }
    results: dict[str, dict[str, Any]] = {}
    passed = True
    for case_id, perturbation in cases.items():
        initial_state = dict(trim.state)
        initial_state.update({name: initial_state[name] + value for name, value in perturbation.items()})
        validation = validate_nonlinear_wrench_lqr(
            adapter,
            trim,
            design,
            initial_state=initial_state,
            duration_s=5.0,
            dt_s=0.02,
        )
        case_passed = (
            validation.allocation_statuses == ("feasible",)
            and validation.saturation_fraction == 0.0
            and validation.final_normalized_feedback_error_norm
            < validation.initial_normalized_feedback_error_norm * 0.2
        )
        passed = passed and case_passed
        results[case_id] = {
            "perturbation": perturbation,
            "passed": case_passed,
            "initial_normalized_feedback_error_norm": validation.initial_normalized_feedback_error_norm,
            "final_normalized_feedback_error_norm": validation.final_normalized_feedback_error_norm,
            "allocation_statuses": list(validation.allocation_statuses),
            "saturation_fraction": validation.saturation_fraction,
            "maximum_controlled_actual_residual": validation.maximum_controlled_actual_residual,
            "maximum_continuous_saturation_duration_s": validation.maximum_continuous_saturation_duration_s,
        }
    report = {
        "schema_version": "taoryx.f16-physical-wrench-perturbation-evidence/v1",
        "status": "development_screen_passed" if passed else "development_screen_failed",
        "family_id": "reference_f16_s119",
        "plant_id": "reference-f16-s119-source-runtime-plant",
        "controller_id": design.id,
        "tuning_profile": "state_and_wrench_balanced_q10_r0p01",
        "control_path": "lqr_to_desired_wrench_to_bounded_effectors_to_source_nonlinear_plant",
        "horizon_s": 5.0,
        "step_s": 0.02,
        "cases": results,
        "claim_boundary": (
            "local fixed-operating-point perturbation screen; not scheduled control, broad envelope validation, "
            "source-validated actuator dynamics, or flight qualification"
        ),
    }
    output = ROOT / "verification/f16_physical_wrench_perturbation_evidence.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if passed else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
