#!/usr/bin/env python3
"""Validate checked-in composition witnesses for every runnable endpoint."""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.vehicle_execution_parity_witnesses import validate_vehicle_execution_parity_witnesses
from taoryx.vehicle_execution_witnesses import validate_vehicle_execution_witnesses, validate_vehicle_variant_witnesses


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--execute-batch",
        action="store_true",
        help="run every batch witness through the public compose-to-run entry point",
    )
    parser.add_argument(
        "--variants-only",
        action="store_true",
        help="validate only runtime-bound variant composition witnesses without opening endpoints",
    )
    parser.add_argument(
        "--execute-parity",
        action="store_true",
        help="replay every checked-in trace for pairs with registered batch/episode parity evidence",
    )
    arguments = parser.parse_args()
    if arguments.variants_only and (arguments.execute_batch or arguments.execute_parity):
        parser.error("--variants-only cannot be combined with --execute-batch or --execute-parity")
    if arguments.variants_only:
        report = validate_vehicle_variant_witnesses()
    else:
        report = validate_vehicle_execution_witnesses(execute_batch=arguments.execute_batch)
        if arguments.execute_parity:
            parity = validate_vehicle_execution_parity_witnesses()
            report["batch_episode_parity_witnesses"] = parity
            if parity["status"] != "pass":
                report["status"] = "fail"
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "pass" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
