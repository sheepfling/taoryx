#!/usr/bin/env python3
"""Validate checked-in composition witnesses for every runnable endpoint."""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.vehicle_execution_witnesses import validate_vehicle_execution_witnesses


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--execute-batch",
        action="store_true",
        help="run every batch witness through the public compose-to-run entry point",
    )
    arguments = parser.parse_args()
    report = validate_vehicle_execution_witnesses(execute_batch=arguments.execute_batch)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "pass" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
