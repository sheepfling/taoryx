#!/usr/bin/env python3
"""Validate and optionally write the horizontal fidelity manifest."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.horizontal_fidelity import validate_horizontal_fidelity


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("verification/alpha3_horizontal_fidelity/manifest.json"),
        help="machine-readable report destination",
    )
    parser.add_argument("--check", action="store_true", help="return nonzero when conformance fails")
    args = parser.parse_args()
    report = validate_horizontal_fidelity()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"status": report.status, "family_count": report.family_count, "errors": len(report.errors)}))
    return 1 if args.check and report.errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
