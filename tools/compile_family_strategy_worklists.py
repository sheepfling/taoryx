#!/usr/bin/env python3
"""Compile the generic tuning and integration worklist for every family tier."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.family_strategy import build_family_strategy_worklist


def main() -> int:
    """Write a checked worklist without treating blocked tiers as an error."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("verification/family_strategy_worklists.json"))
    parser.add_argument("--check", action="store_true", help="fail only when strategy-to-family conformance fails")
    args = parser.parse_args()

    report = build_family_strategy_worklist()
    payload = report.as_dict()
    destination = args.output if args.output.is_absolute() else ROOT / args.output
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "status": payload["status"],
                "family_count": payload["family_count"],
                "tier_count": payload["tier_count"],
                "status_counts": payload["status_counts"],
            },
            sort_keys=True,
        )
    )
    return 1 if args.check and payload["status"] != "pass" else 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
