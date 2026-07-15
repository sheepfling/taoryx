from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

def rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))
####
####


def validate(root: Path) -> dict[str, Any]:
    failures: list[str] = []
    static = rows(root / "aero/grids/static_grid_zero_controls.csv")
    de = rows(root / "aero/grids/symmetric_stabilator_grid.csv")
    da = rows(root / "aero/grids/differential_stabilator_grid.csv")
    dr = rows(root / "aero/grids/rudder_grid.csv")
    mass = rows(root / "mass/source_native_full_throttle_mass_cg_inertia_schedule.csv")
    atmosphere = rows(root / "environment/us_standard_atmosphere_1976.csv")

    expected_static = 27 * 4 * 7 * 5
    if len(static) != expected_static:
        failures.append(f"static row count: {len(static)} != {expected_static}")
    if len(de) != expected_static * 9:
        failures.append("symmetric control row count")
    if len(da) != expected_static * 9:
        failures.append("differential control row count")
    if len(dr) != expected_static * 7:
        failures.append("rudder row count")

    for row in static:
        for key in ("cx","cy","cz","cmx","cmy","cmz"):
            if not math.isfinite(float(row[key])):
                failures.append(f"non-finite {key}")
                break
        ####
    ####

    masses = [float(row["total_mass_lbm"]) for row in mass]
    if any(next_mass > current_mass + 1e-9 for current_mass, next_mass in zip(masses[:-1], masses[1:], strict=True)):
        failures.append("mass is not monotone")
    ####

    minimum_eigenvalue = min(float(row["minimum_inertia_eigenvalue_slug_ft2"]) for row in mass)
    if minimum_eigenvalue <= 0.0:
        failures.append("inertia is not positive definite")

    sea_level = atmosphere[0]
    if abs(float(sea_level["temperature_K"]) - 288.15) > 1e-6:
        failures.append("sea-level temperature")
    if abs(float(sea_level["pressure_Pa"]) - 101325.0) > 1e-3:
        failures.append("sea-level pressure")

    return {
        "status": "PASS" if not failures else "FAIL",
        "failures": failures,
        "counts": {
            "static": len(static),
            "symmetric_stabilator": len(de),
            "differential_stabilator": len(da),
            "rudder": len(dr),
            "mass_schedule": len(mass),
            "atmosphere": len(atmosphere),
        },
        "minimum_inertia_eigenvalue_slug_ft2": minimum_eigenvalue,
        "initial_mass_lbm": masses[0],
        "final_mass_lbm": masses[-1],
    }
####
####


def main() -> int:
    root = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    result = validate(root)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1
####
####


if __name__ == "__main__":
    raise SystemExit(main())
####
