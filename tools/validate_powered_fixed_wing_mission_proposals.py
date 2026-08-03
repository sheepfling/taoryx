"""Validate the capability-scaled powered-fixed-wing mission proposal catalog."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from taoryx.powered_fixed_wing_mission_compiler import (
    compare_compiled_racetrack_to_baseline,
    compile_powered_fixed_wing_racetrack,
    load_powered_fixed_wing_mission_profiles,
)
from taoryx.racetrack_template import RACETRACK_FIDELITIES, load_racetrack_template_catalog

ROOT = Path(__file__).resolve().parents[1]
PROFILES = ROOT / "verification/powered_fixed_wing_mission_profiles.yaml"
BASELINE_CATALOG = ROOT / "verification/racetrack_templates.yaml"
OUTPUT = ROOT / "verification/generated/powered_fixed_wing_mission_proposals.json"


def build_catalog() -> dict[str, object]:
    """Build the deterministic planning proposal catalog."""

    profiles = load_powered_fixed_wing_mission_profiles(PROFILES)
    raw_profiles = yaml.safe_load(PROFILES.read_text(encoding="utf-8"))
    raw_profile_entries = raw_profiles.get("profiles", {}) if isinstance(raw_profiles, dict) else {}
    baseline_catalog = load_racetrack_template_catalog(BASELINE_CATALOG)
    compiled: dict[str, object] = {}
    for profile_id, profile in sorted(profiles.items()):
        raw_entry = raw_profile_entries.get(profile_id) if isinstance(raw_profile_entries, dict) else None
        bindings = raw_entry.get("realization_bindings") if isinstance(raw_entry, dict) else None
        if not isinstance(bindings, dict) or not bindings:
            raise ValueError(f"profile {profile_id!r} has no realization bindings")
        realizations: dict[str, object] = {}
        for fidelity, baseline_id in sorted(bindings.items()):
            if fidelity not in RACETRACK_FIDELITIES:
                raise ValueError(f"profile {profile_id!r} has unsupported fidelity {fidelity!r}")
            if not isinstance(baseline_id, str):
                raise ValueError(f"profile {profile_id!r} has invalid baseline binding for {fidelity!r}")
            proposal = compile_powered_fixed_wing_racetrack(
                *profile,
                binding_id=f"{profile_id}-{fidelity}-compiled",
                fidelity=fidelity,
            )
            manifest = proposal.manifest()
            manifest["baseline_comparison"] = compare_compiled_racetrack_to_baseline(proposal, baseline_catalog.get(baseline_id))
            realizations[fidelity] = manifest
        capability, intent = profile
        compiled[profile_id] = {
            "capability": {
                "vehicle_id": capability.vehicle_id,
                "operating_point_id": capability.operating_point_id,
                "provenance": capability.provenance,
                "evidence_class": capability.evidence_class,
            },
            "intent_id": intent.id,
            "realization_count": len(realizations),
            "realizations": realizations,
        }
    return {
        "schema_version": "taoryx.powered-fixed-wing-mission-proposals/v1",
        "profile_count": len(compiled),
        "profiles": compiled,
        "nonclaim": "These capability-scaled routes are planning proposals; run the bound controller and independent truth evaluator before qualification.",
    }
    ####


def main() -> int:
    """Write or verify the checked-in planning proposal catalog."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    catalog = build_catalog()
    rendered = json.dumps(catalog, indent=2, sort_keys=True) + "\n"
    if args.check:
        if not OUTPUT.is_file() or OUTPUT.read_text(encoding="utf-8") != rendered:
            raise SystemExit("powered fixed-wing mission proposal catalog is stale; rerun tools/validate_powered_fixed_wing_mission_proposals.py")
        print(json.dumps({"profile_count": catalog["profile_count"], "status": "pass"}, sort_keys=True))
        return 0
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(rendered, encoding="utf-8")
    print(OUTPUT)
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
