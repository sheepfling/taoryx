"""Regenerate and validate the HL-20 G0-G6 qualification bundle."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

from taoryx.hl20_qualification import DEFAULT_ARTIFACT_DIR, DEFAULT_QUALITY_GATES, validate_hl20_artifacts
from taoryx.hl20_reachability import write_hl20_fidelity_bundle


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact-dir", type=Path, default=ROOT / DEFAULT_ARTIFACT_DIR)
    parser.add_argument("--quality-gates", type=Path, default=ROOT / DEFAULT_QUALITY_GATES)
    parser.add_argument("--output", type=Path, default=ROOT / "verification/hl20_ca_hi_qualification.json")
    parser.add_argument("--validate-only", action="store_true")
    arguments = parser.parse_args()
    if not arguments.validate_only:
        write_hl20_fidelity_bundle(arguments.artifact_dir)
    report = validate_hl20_artifacts(arguments.artifact_dir, arguments.quality_gates)
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"HL-20 qualification: {report.verdict}")
    for gate in report.gates:
        print(f"{gate.gate_id}: {gate.status} | {gate.message}")
    return 0 if report.verdict == "qualified_through_hl20_g6" else 1


if __name__ == "__main__":
    raise SystemExit(main())
