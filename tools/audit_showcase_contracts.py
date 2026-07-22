"""Audit comparability contracts for an already-rendered fidelity showcase."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
LADDER = ROOT / "verification/fidelity_ladder.yaml"
####


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
####


def _contract(problem: Path, tables: tuple[Path, ...], tier: str, units: str, duration: float | None) -> dict[str, Any]:
    return {
        "problem_sha256": _sha256(problem),
        "table_sha256": [_sha256(path) for path in tables],
        "dynamics_tier": tier,
        "unit_system": units,
        "duration_s": duration,
    }
####


def _compare(left: dict[str, Any], right: dict[str, Any]) -> dict[str, Any]:
    fields = ("problem_sha256", "table_sha256", "unit_system", "duration_s")
    mismatches = [field for field in fields if left[field] != right[field]]
    return {"comparable": not mismatches, "mismatches": mismatches}
####


def audit(showcase_dir: Path) -> Path:
    showcase_dir = showcase_dir.resolve()
    manifest = json.loads((showcase_dir / "manifest.json").read_text(encoding="utf-8"))
    ladder = yaml.safe_load(LADDER.read_text(encoding="utf-8"))
    durations = {item["id"]: item["durations_s"] for item in manifest["families"]}
    reports: list[dict[str, Any]] = []
    for family in ladder["families"]:
        identifier = str(family["id"])
        point_problem = ROOT / str(family.get("showcase_point_mass_problem", family["point_mass_problem"]))
        rigid_problem = ROOT / str(family.get("showcase_rigid_body_problem", family["rigid_body_problem"]))
        point_tables = tuple(ROOT / str(path) for path in family["tables"])
        rigid_tables = tuple(ROOT / str(path) for path in family["rigid_tables"])
        unit_point = str(family["point_mass_unit_system"])
        unit_rigid = str(family["rigid_body_unit_system"])
        family_contracts = {
            "3dof": _contract(point_problem, point_tables, "point-mass-3dof", unit_point, durations[identifier]["3dof"]),
            "bridge": _contract(point_problem, point_tables, "kinematic-3-plus-3-dof", unit_point, durations[identifier]["bridge"]),
            "6dof": _contract(rigid_problem, rigid_tables, "rigid-body-6dof", unit_rigid, durations[identifier]["6dof"]),
        }
        reports.append({
            "id": identifier,
            "contracts": family_contracts,
            "comparisons": {
                "3dof_vs_bridge": _compare(family_contracts["3dof"], family_contracts["bridge"]),
                "3dof_vs_6dof": _compare(family_contracts["3dof"], family_contracts["6dof"]),
            },
        })
    report = {
        "schema_version": 1,
        "showcase_manifest": str((showcase_dir / "manifest.json").relative_to(ROOT)),
        "claim": "comparability audit; not engineering validity",
        "families": reports,
    }
    destination = showcase_dir / "scenario_contract_report.json"
    destination.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return destination
####


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("showcase_dir", type=Path)
    print(audit(parser.parse_args().showcase_dir))
    ####


if __name__ == "__main__":
    main()
