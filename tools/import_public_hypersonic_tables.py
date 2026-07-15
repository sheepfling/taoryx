"""Generate traceable TAORYX tables from the public hypersonic fixture."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests/fixtures/public_hypersonic_research_v1"
SOURCE = FIXTURE / "source/aero/langley_winged_cone_hypersonic_static.csv"
OUTPUT = FIXTURE / "tables/langley_winged_cone_6axis_static.tbl"
PROPULSION = ROOT / "tests/fixtures/public_hypersonic_research_v1/source/propulsion"


def main() -> None:
    rows = list(csv.DictReader(SOURCE.open(encoding="utf-8", newline="")))
    mach = sorted({float(row["mach"]) for row in rows})
    alpha_deg = sorted({float(row["alpha_deg"]) for row in rows})
    by_point = {(float(row["mach"]), float(row["alpha_deg"])): row for row in rows}

    def values(column: str, *, default: float = 0.0) -> list[float]:
        return [float(by_point[m, a].get(column, default)) for m in mach for a in alpha_deg]

    def line(name: str, column: str, *, default: float = 0.0, table_output: str | None = None) -> str:
        rendered = ",".join(f"{value:.12g}" for value in values(column, default=default))
        output = table_output or name
        return f"({name})\ntable {output}(mach,alpha) no-extrap\nmach={','.join(f'{value:g}' for value in mach)}\nalpha={','.join(f'{math.radians(value):.12g}' for value in alpha_deg)}\n{output}={rendered}\n\n"

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(
        "# Derived from source/langley_winged_cone_hypersonic_static.csv.\n"
        "# alpha is radians; the source deck is longitudinal-only and has no beta or controls.\n"
        + line("cx", "CX_body_x_forward")
        + line("cy", "CY", default=0.0)
        + line("cz", "CZ_body_z_down")
        + line("cmx", "cmx")
        + line("cmy", "cmy")
        + line("cmz", "cmz"),
        encoding="utf-8",
    )
    for motor in ("orion38", "orion50xl", "orion50sxl"):
        source = PROPULSION / f"{motor}_thrust_curve_surrogate.csv"
        motor_rows = list(csv.DictReader(source.open(encoding="utf-8", newline="")))
        time = [float(row["time_s"]) for row in motor_rows]
        thrust = [float(row["thrust_N"]) for row in motor_rows]
        mdot = [float(row["propellant_mass_flow_lbm_s"]) * 0.45359237 for row in motor_rows]
        motor_output = FIXTURE / f"tables/{motor}_thrust_mdot.tbl"
        motor_output.write_text(
            f"# Derived from source/propulsion/{source.name}; mdot converted to kg/s.\n"
            f"(thrust)\ntable thrust(time) no-extrap units=n\n"
            f"time={','.join(f'{value:.12g}' for value in time)}\n"
            f"thrust={','.join(f'{value:.12g}' for value in thrust)}\n\n"
            f"(mdot)\ntable mdot(time) no-extrap units=kg/sec\n"
            f"time={','.join(f'{value:.12g}' for value in time)}\n"
            f"mdot={','.join(f'{value:.12g}' for value in mdot)}\n",
            encoding="utf-8",
        )
    metadata = {
        "source": "source/aero/langley_winged_cone_hypersonic_static.csv",
        "generated": [
            "tables/langley_winged_cone_6axis_static.tbl",
            "tables/orion38_thrust_mdot.tbl",
            "tables/orion50xl_thrust_mdot.tbl",
            "tables/orion50sxl_thrust_mdot.tbl",
        ],
        "rows": len(rows),
        "axes": {"mach": mach, "alpha_deg": alpha_deg, "alpha_table_units": "radian"},
        "coverage": "longitudinal-only",
        "control_effectiveness": False,
        "sideslip": False,
        "fidelity": "EQUATION_RECONSTRUCTION",
        "source_id": "NASA_CR_194987",
        "reference_area_m2": 334.450944,
        "reference_length_m": 24.384,
    }
    (FIXTURE / "tables/manifest.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    ####


if __name__ == "__main__":
    main()
