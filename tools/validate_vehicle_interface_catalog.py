"""Validate the complete resolved vehicle-interface catalog."""

from __future__ import annotations

import argparse
import json
from collections.abc import Sequence

from taoryx.vehicle_interface import build_vehicle_interface_catalog_report


def build_report() -> dict[str, object]:
    """Return the static interface report used by CI and the public CLI."""

    return build_vehicle_interface_catalog_report()
    ####


def main(argv: Sequence[str] | None = None) -> int:
    """Print a compact result and fail when interface declarations drift."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if any resolved interface has a conformance finding")
    args = parser.parse_args(argv)
    report = build_report()
    summary = {
        "schema": report["schema"],
        "status": report["status"],
        "family_count": report["family_count"],
        "interface_count": report["interface_count"],
        "error_count": report["error_count"],
    }
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if report["status"] == "pass" or not args.check else 2
    ####


if __name__ == "__main__":
    raise SystemExit(main())
