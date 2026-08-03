"""Assess whether a capability-scaled route candidate may replace a baseline."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from taoryx.mission_promotion import assess_mission_promotion
from taoryx.powered_fixed_wing_mission_compiler import (
    compile_powered_fixed_wing_racetrack,
    load_powered_fixed_wing_mission_profiles,
    resolve_powered_fixed_wing_mission_profile,
)

ROOT = Path(__file__).resolve().parents[1]
PROFILES = ROOT / "verification/powered_fixed_wing_mission_profiles.yaml"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--fidelity", required=True)
    parser.add_argument("--execution", type=Path)
    args = parser.parse_args()
    profiles = load_powered_fixed_wing_mission_profiles(PROFILES)
    if args.profile not in profiles:
        parser.error(f"unknown profile: {args.profile}")
    execution = json.loads(args.execution.read_text(encoding="utf-8")) if args.execution else None
    proposal = compile_powered_fixed_wing_racetrack(
        *resolve_powered_fixed_wing_mission_profile(PROFILES, args.profile, args.fidelity),
        binding_id=f"{args.profile}-{args.fidelity}-candidate",
        fidelity=args.fidelity,
    )
    print(json.dumps(assess_mission_promotion(proposal, execution).as_dict(), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
