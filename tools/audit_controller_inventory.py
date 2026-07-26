"""Audit controller-path inventory before LQR migration or qualification."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from taoryx.controller_design import load_controller_catalog
from taoryx.controller_inventory import ControllerInventory

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INVENTORY = ROOT / "verification/controller_inventory.yaml"
DEFAULT_DESIGNS = ROOT / "verification/controller_designs.yaml"
ESTABLISHED_FAMILIES = ("b747", "skywalker_x8", "hummingbird", "x15")
####


def audit(inventory_path: Path = DEFAULT_INVENTORY, design_path: Path = DEFAULT_DESIGNS) -> dict[str, object]:
    """Validate coverage of active paths and declared LQR design artifacts."""

    inventory = ControllerInventory(**yaml.safe_load(inventory_path.read_text(encoding="utf-8")))
    designs = load_controller_catalog(design_path)
    entries_by_design = {entry.design_id: entry for entry in inventory.entries if entry.design_id is not None}
    missing_designs = [design.id for design in designs.designs if design.id not in entries_by_design]
    active_families = {entry.family for entry in inventory.entries if entry.active_runtime}
    missing_families = [family for family in ESTABLISHED_FAMILIES if family not in active_families]
    qualification_blockers = [
        {
            "id": entry.id,
            "reason": "scenario gain overrides are present",
        }
        for entry in inventory.entries
        if entry.qualification_eligible and entry.scenario_gain_overrides
    ]
    report = {
        "schema_version": inventory.schema_version,
        "inventory_id": inventory.id,
        "entry_count": len(inventory.entries),
        "active_entry_count": sum(entry.active_runtime for entry in inventory.entries),
        "baseline_entry_count": sum(entry.legacy_baseline for entry in inventory.entries),
        "design_count": len(designs.designs),
        "missing_design_inventory": missing_designs,
        "missing_active_families": missing_families,
        "qualification_blockers": qualification_blockers,
        "status": "pass" if not missing_designs and not missing_families and not qualification_blockers else "blocked",
        "claim_boundary": inventory.claim_boundary,
    }
    return report
    ####


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inventory", type=Path, default=DEFAULT_INVENTORY)
    parser.add_argument("--designs", type=Path, default=DEFAULT_DESIGNS)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = audit(args.inventory, args.designs)
    serialized = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output is None:
        print(serialized, end="")
    else:
        args.output.write_text(serialized, encoding="utf-8")
        print(args.output)
    if report["status"] != "pass":
        raise SystemExit(2)
    ####


if __name__ == "__main__":
    main()
####
