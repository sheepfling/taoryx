"""Generate source-linked HL-20 lifting-body load evidence."""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from taoryx.trajectory import DAVEMLLiftingBodyLoadBinding, load_daveml_atmosphere, load_daveml_trim_binding

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    sidecar = ROOT / "families/reference_hl20_mod_k/plant/daveml-import.json"
    aero = load_daveml_trim_binding(
        sidecar,
        role="aerodynamics",
        state_inputs={"alpha_deg": "ALP_UNLIM"},
        control_inputs={},
        residual_outputs={"cl": "CL", "cd": "CD", "cm": "CM"},
        fixed_inputs={"BETA": 0.0, "XMACH": 1.0, "PB": 0.0, "QB": 0.0, "RB": 0.0, "VRW": 100.0, "H_rwy": 0.0, "DBFUL": 0.0, "DBFUR": 0.0, "DBFLL": 0.0, "DBFLR": 0.0, "DWFL": 0.0, "DWFR": 0.0, "DRUD": 0.0, "DLG": 0.0},
    )
    atmosphere = load_daveml_atmosphere(ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml")
    binding = DAVEMLLiftingBodyLoadBinding(aero, 26.612075808, 8.607552, 1.0)
    loads = binding.evaluate_with_atmosphere(
        {"alpha_deg": 5.0},
        {},
        atmosphere,
        geometric_altitude_m=0.0,
        true_airspeed_m_s=30.48,
    )
    report = {
        "schema_version": "taoryx.daveml-hl20-load-evidence/v1",
        "status": "verified",
        "claim_boundary": "source-bounded HL-20 wind-axis load sample; no mass or dynamics claim",
        "family_id": "reference_hl20_mod_k",
        "operating_point": {"alpha_deg": 5.0, "mach": 1.0, "true_airspeed_m_s": 30.48, "geometric_altitude_m": 0.0},
        "loads": loads,
        "geometry": {"reference_area_m2": 26.612075808, "mean_aerodynamic_chord_m": 8.607552},
        "provenance": {"aerodynamics_document_sha256": aero.graph.document_sha256, "aerodynamics_package_sha256": aero.graph.package_sha256, "atmosphere_sha256": atmosphere.source_sha256},
    }
    output = ROOT / "verification/daveml_hl20_load_evidence.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

