"""Compile capability-scaled first-mission proposals for airbreathing vehicles."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from taoryx.powered_fixed_wing_mission_compiler import (
    compare_compiled_racetrack_to_baseline,
    compile_powered_fixed_wing_racetrack,
    load_powered_fixed_wing_mission_profiles,
    resolve_powered_fixed_wing_mission_profile,
)
from taoryx.racetrack_template import load_racetrack_template_catalog

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILES = ROOT / "verification/powered_fixed_wing_mission_profiles.yaml"
DEFAULT_BASELINE_CATALOG = ROOT / "verification/racetrack_templates.yaml"


def main() -> int:
    """Compile selected mission proposals and write an auditable JSON artifact."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profiles", type=Path, default=DEFAULT_PROFILES)
    parser.add_argument("--profile", action="append", dest="selected_profiles")
    parser.add_argument("--fidelity", default="point_mass_3dof")
    parser.add_argument("--baseline-catalog", type=Path, default=DEFAULT_BASELINE_CATALOG)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    profiles = load_powered_fixed_wing_mission_profiles(args.profiles)
    raw_profiles = yaml.safe_load(args.profiles.read_text(encoding="utf-8"))
    baselines = raw_profiles.get("baselines", {}) if isinstance(raw_profiles, dict) else {}
    baseline_catalog = load_racetrack_template_catalog(args.baseline_catalog)
    selected = args.selected_profiles or sorted(profiles)
    unknown = sorted(set(selected).difference(profiles))
    if unknown:
        parser.error(f"unknown profile(s): {', '.join(unknown)}")
    compiled = {}
    for profile_id in selected:
        proposal = compile_powered_fixed_wing_racetrack(
            *resolve_powered_fixed_wing_mission_profile(args.profiles, profile_id, args.fidelity),
            binding_id=f"{profile_id}-compiled",
            fidelity=args.fidelity,
        )
        manifest = proposal.manifest()
        baseline_id = baselines.get(profile_id) if isinstance(baselines, dict) else None
        if isinstance(baseline_id, str):
            manifest["baseline_comparison"] = compare_compiled_racetrack_to_baseline(proposal, baseline_catalog.get(baseline_id))
        compiled[profile_id] = manifest
    artifact = {
        "schema_version": "taoryx.powered-fixed-wing-mission-proposals/v1",
        "profile_count": len(compiled),
        "profiles": compiled,
        "nonclaim": "These capability-scaled routes are planning proposals; run the bound controller and independent truth evaluator before qualification.",
    }
    rendered = json.dumps(artifact, indent=2, sort_keys=True) + "\n"
    if args.output is None:
        print(rendered, end="")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
        print(args.output)
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
