"""Generate source-linked HL-20 pitch-channel linearization evidence."""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.trajectory import load_daveml_trim_binding
from taoryx.trim import TrimSpec, finite_difference_linearization, solve_trim


def main() -> int:
    binding = load_daveml_trim_binding(
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
    spec = TrimSpec(
        state_names=("alpha_deg",),
        control_names=(),
        residual_names=("pitch_cm",),
        state_initial={"alpha_deg": 5.0},
        control_initial={},
        state_lower={"alpha_deg": 0.0},
        state_upper={"alpha_deg": 15.0},
    )
    trim = solve_trim(spec, binding.as_evaluator(), max_nfev=100, residual_tolerance=1.0e-10)
    a_matrix, b_matrix = finite_difference_linearization(
        spec, binding.as_evaluator(), trim, state_step=1.0e-5, control_step=1.0e-5
    )
    report = {
        "schema_version": "taoryx.daveml-hl20-linearization-evidence/v1",
        "status": "verified" if trim.success else "failed",
        "claim_boundary": "source-bounded HL-20 pitch residual Jacobian; not full 6-DOF dynamics or controller qualification",
        "family_id": "reference_hl20_mod_k",
        "state_names": list(spec.state_names),
        "control_names": list(spec.control_names),
        "residual_names": list(spec.residual_names),
        "trim": {
            "state": dict(trim.state),
            "controls": dict(trim.controls),
            "residuals": dict(trim.residuals),
            "max_residual": trim.max_residual,
            "success": trim.success,
            "iterations": trim.iterations,
        },
        "perturbations": {"state_step": 1.0e-5, "control_step": 1.0e-5, "method": "central_finite_difference"},
        "a_matrix": a_matrix.tolist(),
        "b_matrix": b_matrix.tolist(),
        "provenance": {
            "document_sha256": binding.graph.document_sha256,
            "package_sha256": binding.graph.package_sha256,
        },
    }
    output = ROOT / "verification/daveml_hl20_linearization_evidence.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if trim.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
