from __future__ import annotations

import csv
import json
import sys
from pathlib import Path
from typing import Any


def count_rows(path: Path) -> int:
    with path.open("r", newline="", encoding="utf-8") as handle:
        return sum(1 for _ in csv.DictReader(handle))
####
####


def main() -> int:
    root = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    checks: dict[str, Any] = {
        "b747_static": count_rows(root / "jet_b747/aero/static_six_axis_grid.csv"),
        "x8_static": count_rows(root / "cruise_class_uav_skywalker_x8/aero/static_airframe_grid.csv"),
        "hummingbird_wrench": count_rows(root / "quadcopter_hummingbird/aero/common_speed_wrench_grid.csv"),
    }
    failures = []
    if checks["b747_static"] != 650:
        failures.append("B747 static row count")
    if checks["x8_static"] != 840:
        failures.append("X8 static row count")
    if checks["hummingbird_wrench"] != 270:
        failures.append("Hummingbird wrench row count")
    result = {"status": "PASS" if not failures else "FAIL", "checks": checks, "failures": failures}
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if not failures else 1
####
####


if __name__ == "__main__":
    raise SystemExit(main())
####
