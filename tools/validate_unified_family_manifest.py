#!/usr/bin/env python3
"""Validate the unified nine-family integration manifest join."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.family_manifest import load_unified_family_manifest_catalog


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("verification/alpha3_horizontal_fidelity/unified_manifest.json"),
    )
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    catalog = load_unified_family_manifest_catalog()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(catalog.as_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"status": "pass" if not catalog.errors else "fail", "family_count": len(catalog.families), "errors": len(catalog.errors)}))
    return 1 if args.check and catalog.errors else 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
