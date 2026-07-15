"""Generate TAORYX coefficient and propulsion tables from the X-15 fixture."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests/fixtures/x15_coherent_6dof_public_research_v1"
GRID = FIXTURE / "aero/grids"
TABLES = FIXTURE / "tables"
COEFFICIENTS = ("cx", "cy", "cz", "cmx", "cmy", "cmz")
COMMON_AXES = ("mach", "altitude_m", "alpha", "beta")
DEGREE_AXES = {"alpha_deg": "alpha", "beta_deg": "beta", "symmetric_stabilator_deg": "symmetric_stabilator", "differential_stabilator_deg": "differential_stabilator", "rudder_deg": "rudder"}


def _number(value: float) -> str:
    return f"{value:.12g}"
    ####


def _axis_value(column: str, value: str) -> float:
    if column == "altitude_ft":
        return float(value) * 0.3048
    if column.endswith("_deg"):
        return math.radians(float(value))
    return float(value)
    ####


def _write_grid(source: Path, destination: Path, control_column: str | None = None) -> None:
    rows = list(csv.DictReader(source.open(encoding="utf-8", newline="")))
    source_axes = ["mach", "altitude_ft", "alpha_deg", "beta_deg"] + ([control_column] if control_column else [])
    axis_names = ["mach", "altitude_m", "alpha", "beta"] + ([DEGREE_AXES[control_column]] if control_column else [])
    axes = {name: sorted({_axis_value(column, row[column]) for row in rows}) for name, column in zip(axis_names, source_axes, strict=True)}
    indexed = {
        tuple(_axis_value(column, row[column]) for column in source_axes): row
        for row in rows
    }
    values_by_coefficient: dict[str, list[float]] = {name: [] for name in COEFFICIENTS}
    import itertools

    for point in itertools.product(*(axes[name] for name in axis_names)):
        row = indexed[point]
        for coefficient in COEFFICIENTS:
            values_by_coefficient[coefficient].append(float(row[coefficient]))
    header = f"# Derived from source/{source.relative_to(FIXTURE)}; angular axes are radians and altitude is meters.\n"
    text = header
    for coefficient in COEFFICIENTS:
        text += f"({coefficient})\ntable {coefficient}({','.join(axis_names)}) no-extrap sref=18.580608\n"
        for name in axis_names:
            text += f"{name}={','.join(_number(value) for value in axes[name])}\n"
        text += f"{coefficient}={','.join(_number(value) for value in values_by_coefficient[coefficient])}\n\n"
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(text, encoding="utf-8")
    ####


def main() -> None:
    TABLES.mkdir(parents=True, exist_ok=True)
    _write_grid(GRID / "static_grid_zero_controls.csv", TABLES / "x15_static_6axis.tbl")
    for stem, control_column in (
        ("symmetric_stabilator", "symmetric_stabilator_deg"),
        ("differential_stabilator", "differential_stabilator_deg"),
        ("rudder", "rudder_deg"),
    ):
        _write_grid(GRID / f"{stem}_grid.csv", TABLES / f"x15_{stem}_6axis.tbl", control_column)
    with (FIXTURE / "propulsion/source_native_full_throttle_step_history.csv").open(encoding="utf-8", newline="") as handle:
        propulsion_rows = list(csv.DictReader(handle))
    TABLES.joinpath("x15_xlr99_thrust_mdot.tbl").write_text(
        "# Derived from propulsion/source_native_full_throttle_step_history.csv; flow converted from lbm/s to kg/s.\n"
        "(x15-thrust)\n"
        "table thrust(time) no-extrap units=n\n"
        f"time={','.join(_number(float(row['time_s'])) for row in propulsion_rows)}\n"
        f"thrust={','.join(_number(float(row['ideal_vacuum_thrust_N'])) for row in propulsion_rows)}\n\n"
        "(x15-mdot)\n"
        "table mdot(time) no-extrap units=kg/sec\n"
        f"time={','.join(_number(float(row['time_s'])) for row in propulsion_rows)}\n"
        f"mdot={','.join(_number((float(row['total_flow_lbm_s']) * 0.45359237)) for row in propulsion_rows)}\n",
        encoding="utf-8",
    )
    metadata = {
        "source_validation": "checks/validation_report.json",
        "reference_geometry": "geometry/reference_geometry.csv",
        "generated_tables": sorted(path.name for path in TABLES.glob("*.tbl")),
        "body_axes": {"x": "forward", "y": "right", "z": "down"},
        "static_axes": ["mach", "altitude_m", "alpha", "beta"],
        "control_axes": ["symmetric_stabilator", "differential_stabilator", "rudder"],
        "fidelity": "vehicle-specific public 6-DOF research surrogate",
        "limitations": ["no certified thermal model", "zero wind baseline", "source marked BETA"],
    }
    (TABLES / "manifest.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    ####


if __name__ == "__main__":
    main()
