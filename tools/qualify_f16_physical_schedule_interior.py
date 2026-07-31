#!/usr/bin/env python3
"""Qualify the conservative interior of the F-16 physical schedule witness.

The source schedule envelope intentionally contains authority-boundary cases.
This tool computes the intersection of cases that pass at every validated node
and records that smaller schedule-wide interior separately from the boundary
matrix.  It does not promote the F-16 to full-envelope or operational status.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "verification/alpha3_f16_physical_schedule_envelope/manifest.json"
DEFAULT_OUTPUT = ROOT / "verification/alpha3_f16_physical_schedule_envelope/interior_qualification.json"
####


def _load(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected mapping in {path}")
    return payload
    ####


def qualify(output: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    """Build the schedule-wide interior qualification record."""

    source = _load(SOURCE)
    nodes = source.get("nodes")
    if not isinstance(nodes, list) or not nodes:
        raise ValueError("F-16 schedule envelope has no nodes")
    node_cases = {
        str(node["point_id"]): {
            str(case_id): case
            for case_id, case in node.get("cases", {}).items()
            if isinstance(case, dict)
        }
        for node in nodes
        if isinstance(node, dict) and isinstance(node.get("cases"), dict)
    }
    common_case_ids = set.intersection(
        *(set(case_id for case_id, case in cases.items() if case.get("mission_pass") is True) for cases in node_cases.values())
    )
    all_case_ids = set().union(*(set(cases) for cases in node_cases.values()))
    boundary_case_ids = sorted(all_case_ids.difference(common_case_ids))
    interior_cases = {
        case_id: {
            "perturbation": next(
                dict(node_cases[point_id][case_id].get("perturbation", {}))
                for point_id in node_cases
                if case_id in node_cases[point_id]
            ),
            "node_results": {
                point_id: {
                    "mission_pass": bool(cases[case_id].get("mission_pass")),
                    "recovery_ratio": cases[case_id].get("recovery_ratio"),
                    "allocation_statuses": cases[case_id].get("allocation_statuses", []),
                    "saturation_fraction": cases[case_id].get("saturation_fraction"),
                }
                for point_id, cases in node_cases.items()
            },
        }
        for case_id in sorted(common_case_ids)
    }
    boundary_cases = {
        point_id: {
            case_id: {
                "mission_pass": bool(case.get("mission_pass")),
                "failure_reason": case.get("failure_reason"),
                "perturbation": case.get("perturbation", {}),
            }
            for case_id, case in sorted(cases.items())
            if case_id in boundary_case_ids
        }
        for point_id, cases in node_cases.items()
    }
    schedule = source.get("schedule_envelope", {})
    report: dict[str, Any] = {
        "schema": "taoryx.f16-physical-schedule-interior-qualification/v1alpha1",
        "status": "F16_physical_schedule_interior_qualified" if common_case_ids else "F16_physical_schedule_interior_missing",
        "family_id": "reference_f16_s119",
        "source_artifact": "verification/alpha3_f16_physical_schedule_envelope/manifest.json",
        "direct_body_moment_injection": source.get("direct_body_moment_injection"),
        "control_path": source.get("control_path"),
        "claim_boundary": "Local schedule-wide interior physical-effector evidence only. This does not claim beta/high-rate authority outside the retained boundary cases, a full Mach/dynamic-pressure envelope, wind robustness, servo certification, statistical reliability, or operational flight control.",
        "validated_interior": {
            "node_count": len(node_cases),
            "altitude_m": schedule.get("altitude_m"),
            "true_airspeed_m_s": schedule.get("true_airspeed_m_s"),
            "case_ids": sorted(common_case_ids),
            "case_count": len(common_case_ids) * len(node_cases),
            "all_cases_pass_at_all_nodes": all(
                result["mission_pass"]
                for case in interior_cases.values()
                for result in case["node_results"].values()
            ),
            "cases": interior_cases,
        },
        "boundary_witnesses": {
            "case_ids": boundary_case_ids,
            "schedule_wide_case_count": len(boundary_case_ids) * len(node_cases),
            "node_results": boundary_cases,
            "interpretation": "A case is schedule-wide boundary evidence when it fails at any validated source node; partial authority is retained and is not clipped into a pass.",
        },
        "summary": {
            "node_count": len(node_cases),
            "source_case_count": sum(len(cases) for cases in node_cases.values()),
            "interior_case_id_count": len(common_case_ids),
            "interior_case_count": len(common_case_ids) * len(node_cases),
            "schedule_wide_boundary_case_count": len(boundary_case_ids) * len(node_cases),
        },
        "reproduction": "PYTHONPATH=src python3 tools/qualify_f16_physical_schedule_interior.py",
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    report = qualify(arguments.output)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "F16_physical_schedule_interior_qualified" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
