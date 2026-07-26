"""Generate source-backed DaveML operating-point evidence.

This first artifact is intentionally narrow: it certifies the F-16 source
pitch-channel trim and its local residual sensitivity.  It does not claim a
full equations-of-motion equilibrium until mass, gravity, and atmosphere
bindings are explicitly supplied.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.trajectory import load_daveml_trim_binding
from taoryx.trim import TrimSpec, finite_difference_linearization, solve_trim


def build_f16_pitch_binding():
    return load_daveml_trim_binding(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        role="aerodynamics",
        state_inputs={},
        control_inputs={"elevator_deg": "el"},
        residual_outputs={"pitch_cm": "cm"},
        fixed_inputs={
            "vt": 500.0,
            "alpha": 2.0,
            "beta": 0.0,
            "p": 0.0,
            "q": 0.0,
            "r": 0.0,
            "ail": 0.0,
            "rdr": 0.0,
            "xcg": 0.35,
        },
    )


def build_hl20_pitch_binding():
    return load_daveml_trim_binding(
        ROOT / "families/reference_hl20_mod_k/plant/daveml-import.json",
        role="aerodynamics",
        state_inputs={"alpha_deg": "ALP_UNLIM"},
        control_inputs={},
        residual_outputs={"pitch_cm": "CM"},
        fixed_inputs={
            "BETA": 0.0,
            "XMACH": 1.0,
            "PB": 0.0,
            "QB": 0.0,
            "RB": 0.0,
            "VRW": 100.0,
            "H_rwy": 0.0,
            "DBFUL": 0.0,
            "DBFUR": 0.0,
            "DBFLL": 0.0,
            "DBFLR": 0.0,
            "DWFL": 0.0,
            "DWFR": 0.0,
            "DRUD": 0.0,
            "DLG": 0.0,
        },
    )


def solve_pitch_evidence(binding, spec: TrimSpec) -> dict[str, object]:
    result = solve_trim(spec, binding.as_evaluator(), max_nfev=100, residual_tolerance=1.0e-10)
    residual_jacobian_a, residual_jacobian_b = finite_difference_linearization(
        spec, binding.as_evaluator(), result, state_step=1.0e-5, control_step=1.0e-5
    )
    return {
        "state": dict(result.state),
        "controls": dict(result.controls),
        "residuals": dict(result.residuals),
        "max_residual": result.max_residual,
        "scaled_residual_norm": result.scaled_residual_norm,
        "success": result.success,
        "iterations": result.iterations,
        "solver_status": result.status,
        "solver_message": result.message,
        "local_residual_jacobian": {
            "state_names": list(spec.state_names),
            "control_names": list(spec.control_names),
            "residual_names": list(spec.residual_names),
            "a": residual_jacobian_a.tolist(),
            "b": residual_jacobian_b.tolist(),
            "method": "central_finite_difference",
        },
    }


def main() -> int:
    binding = build_f16_pitch_binding()
    spec = TrimSpec(
        state_names=(),
        control_names=("elevator_deg",),
        residual_names=("pitch_cm",),
        state_initial={},
        control_initial={"elevator_deg": 0.0},
        control_lower={"elevator_deg": -24.0},
        control_upper={"elevator_deg": 24.0},
    )
    result = solve_trim(spec, binding.as_evaluator(), max_nfev=100, residual_tolerance=1.0e-10)
    residual_jacobian_a, residual_jacobian_b = finite_difference_linearization(
        spec,
        binding.as_evaluator(),
        result,
        control_step=1.0e-5,
    )
    report = {
        "schema_version": "taoryx.daveml-trim-evidence/v1",
        "status": "verified" if result.success else "failed",
        "claim_boundary": "source-bounded pitch-channel trim; not full 6-DOF equilibrium",
        "family_id": "reference_f16_s119",
        "model_role": "aerodynamics",
        "source": {
            "package_member": binding.graph.package_member,
            "document_sha256": binding.graph.document_sha256,
            "package_sha256": binding.graph.package_sha256,
        },
        "trim": {
            "state": dict(result.state),
            "controls": dict(result.controls),
            "residuals": dict(result.residuals),
            "max_residual": result.max_residual,
            "scaled_residual_norm": result.scaled_residual_norm,
            "success": result.success,
            "iterations": result.iterations,
            "solver_status": result.status,
            "solver_message": result.message,
            "bounds": {"elevator_deg": [-24.0, 24.0]},
        },
        "local_residual_jacobian": {
            "state_names": list(spec.state_names),
            "control_names": list(spec.control_names),
            "residual_names": list(spec.residual_names),
            "a": residual_jacobian_a.tolist(),
            "b": residual_jacobian_b.tolist(),
            "method": "central_finite_difference",
            "control_step": 1.0e-5,
        },
    }
    output = ROOT / "verification/daveml_f16_trim_evidence.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    hl20_binding = build_hl20_pitch_binding()
    hl20_spec = TrimSpec(
        state_names=("alpha_deg",),
        control_names=(),
        residual_names=("pitch_cm",),
        state_initial={"alpha_deg": 5.0},
        control_initial={},
        state_lower={"alpha_deg": 0.0},
        state_upper={"alpha_deg": 15.0},
    )
    hl20_trim = solve_pitch_evidence(hl20_binding, hl20_spec)
    hl20_report = {
        "schema_version": "taoryx.daveml-trim-evidence/v1",
        "status": "verified" if hl20_trim["success"] else "failed",
        "claim_boundary": "source-bounded pitch-channel trim; not full 6-DOF equilibrium",
        "family_id": "reference_hl20_mod_k",
        "model_role": "aerodynamics",
        "operating_point": {"mach": 1.0, "true_airspeed_f_s": 100.0},
        "source": {
            "package_member": hl20_binding.graph.package_member,
            "document_sha256": hl20_binding.graph.document_sha256,
            "package_sha256": hl20_binding.graph.package_sha256,
        },
        "trim": hl20_trim,
    }
    (ROOT / "verification/daveml_hl20_trim_evidence.json").write_text(
        json.dumps(hl20_report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    print(json.dumps(hl20_report, indent=2, sort_keys=True))
    return 0 if result.success and bool(hl20_trim["success"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
