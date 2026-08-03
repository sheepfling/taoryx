#!/usr/bin/env python3
"""Generate the joined family/tier readiness matrix."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.horizontal_readiness import READINESS_ARTIFACT, build_horizontal_readiness_report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=READINESS_ARTIFACT)
    parser.add_argument("--adapter-artifact", type=Path)
    parser.add_argument("--check", action="store_true", help="fail on manifest-join errors")
    args = parser.parse_args()
    report = build_horizontal_readiness_report(adapter_artifact=args.adapter_artifact)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"status": report.status, "family_count": report.family_count, "tier_count": report.tier_count, "blocked_count": len(report.blockers)}))
    return 1 if args.check and report.status == "blocked" else 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
