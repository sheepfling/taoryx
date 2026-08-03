#!/usr/bin/env python3
"""Preflight one family/tier showcase against the joined readiness matrix."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import cast

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.fidelity_contracts import CANONICAL_FIDELITY_TIERS, FidelityTier
from taoryx.horizontal_readiness import (
    build_horizontal_readiness_report,
    preflight_horizontal_showcase,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("family_id")
    parser.add_argument("tier", choices=CANONICAL_FIDELITY_TIERS)
    parser.add_argument("--adapter-artifact", type=Path)
    parser.add_argument(
        "--require-operation",
        action="append",
        default=[],
        help="adapter operation that must have a passing probe; repeat as needed",
    )
    args = parser.parse_args()
    report = build_horizontal_readiness_report(adapter_artifact=args.adapter_artifact)
    preflight = preflight_horizontal_showcase(
        args.family_id,
        cast(FidelityTier, args.tier),
        report=report,
        required_operations=tuple(args.require_operation),
    )
    print(json.dumps(preflight.as_dict(), indent=2, sort_keys=True))
    return 0 if preflight.allowed else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
