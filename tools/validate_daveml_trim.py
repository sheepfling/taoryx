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
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if result.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
