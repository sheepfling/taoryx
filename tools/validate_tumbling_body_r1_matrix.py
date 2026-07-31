#!/usr/bin/env python3
"""Index the fixed passive tumbling-body shape/uncertainty matrix.

No controller is synthesized.  This wrapper makes the existing passive
deployment qualification consumable by the Alpha 3 readiness matrix while
preserving the rigid-body-reuse pseudo boundary.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT / "verification/alpha3_tumbling_body/qualification.json"
DEFAULT_OUTPUT = ROOT / "verification/alpha3_tumbling_body/r1_matrix"
FIDELITIES = ("point_mass_3dof", "pseudo_6dof")


def run_matrix(source_path: Path, output: Path) -> dict[str, object]:
    """Validate every declared shape at both passive fidelity interfaces."""

    source = json.loads(source_path.read_text(encoding="utf-8"))
    assert isinstance(source, dict)
    shapes = source.get("shapes")
    assert isinstance(shapes, list)
    output.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, Any]] = []
    for shape in shapes:
        assert isinstance(shape, dict)
        shape_id = str(shape["shape"])
        nominal = shape.get("nominal")
        assert isinstance(nominal, dict)
        for fidelity in FIDELITIES:
            tier = nominal.get(fidelity)
            assert isinstance(tier, dict)
            passed = tier.get("classification") == "impact"
            records.append(
                {
                    "shape": shape_id,
                    "fidelity": fidelity,
                    "truth_result": "PASS" if passed else "FAIL",
                    "classification": tier.get("classification"),
                    "terminal_state": tier.get("terminal_state"),
                    "footprint_m": tier.get("footprint_m"),
                    "source_artifact": str(source_path.relative_to(ROOT)),
                }
            )
    passed = sum(record["truth_result"] == "PASS" for record in records)
    report: dict[str, object] = {
        "schema": "taoryx.tumbling-body-passive-r1-matrix/v1alpha1",
        "status": "PASSIVE_R1_MATRIX_COMPLETE",
        "claim": "Fixed passive shape and uncertainty evidence for averaged-area 3DOF and native rigid-body-reuse pseudo interfaces.",
        "claim_boundary": "This is passive-body evidence, not controller robustness. The pseudo tier reuses native rigid-body equations; no prescribed tumble, control surface, wheel, thruster, or authority claim is introduced.",
        "case_count": len(records),
        "passed_case_count": passed,
        "failed_case_count": len(records) - passed,
        "fidelities": list(FIDELITIES),
        "records": records,
        "uncertainty_axes": source.get("uncertainty_axes", {}),
        "source_status": source.get("status"),
        "reproduction": "PYTHONPATH=src python3 tools/validate_tumbling_body_r1_matrix.py",
    }
    (output / "manifest.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(str(report["reproduction"]) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    """Run and write the passive-body R1 matrix."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    print(json.dumps(run_matrix(arguments.source, arguments.output), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
