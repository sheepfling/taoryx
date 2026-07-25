"""Generate the machine-readable Alpha 2 future-family interface report."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import yaml

REQUIRED_FIELDS = {
    "capabilities",
    "component_slots",
    "controls",
    "observations",
    "resources",
    "allocations",
    "mode_transitions",
    "evidence",
}
REQUIRED_FIDELITIES = {"point_mass_3dof", "pseudo_6dof", "rigid_body_6dof"}


def build_report(matrix: dict[str, Any]) -> dict[str, Any]:
    """Validate the contract-only matrix and return a deterministic report."""

    findings: list[dict[str, Any]] = []
    for probe in matrix.get("probes", []):
        missing = sorted(REQUIRED_FIELDS - set(probe))
        missing_fidelities = sorted(REQUIRED_FIDELITIES - set(probe.get("fidelities", [])))
        diagnostics = []
        if missing:
            diagnostics.append(f"missing-fields:{','.join(missing)}")
        if missing_fidelities:
            diagnostics.append(f"missing-fidelities:{','.join(missing_fidelities)}")
        findings.append(
            {
                "id": probe.get("id"),
                "archetype": probe.get("archetype"),
                "status": "pass" if not diagnostics else "fail",
                "fidelities": list(probe.get("fidelities", [])),
                "contract_counts": {field: len(probe.get(field, [])) for field in REQUIRED_FIELDS if field != "capabilities" and field != "evidence"},
                "capabilities": probe.get("capabilities", {}),
                "evidence": probe.get("evidence", {}),
                "provider_status": "interface_only",
                "diagnostics": diagnostics,
            }
        )
    ####
    status = "pass" if findings and all(item["status"] == "pass" for item in findings) else "fail"
    return {
        "schema_version": 1,
        "matrix_id": matrix.get("matrix_id"),
        "status": status,
        "probe_count": len(findings),
        "claim_boundary": "contract coverage only; no provider qualification",
        "findings": findings,
    }
    ####


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix", type=Path, default=Path("verification/alpha2_interface_stress_matrix.yaml"))
    parser.add_argument("--output", type=Path, default=Path("verification/generated/alpha2_interface_compatibility_report.json"))
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    matrix = yaml.safe_load(args.matrix.read_text(encoding="utf-8"))
    report = build_report(matrix)
    if args.check:
        expected = json.loads(args.output.read_text(encoding="utf-8"))
        if expected != report:
            raise SystemExit("Alpha 2 interface compatibility report is stale")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Alpha 2 interface compatibility report: {report['status']} ({report['probe_count']} probes)")
    return 0 if report["status"] == "pass" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
