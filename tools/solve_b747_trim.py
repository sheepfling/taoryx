"""Solve the first B747 source-anchor force trim using the TAORYX runtime.

This is deliberately a plant-level research tool.  It calls ``run_files`` for
each candidate, so the residual is produced by the same table, gravity, frame,
and rigid-body path used by the trajectory runner.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import tempfile
from pathlib import Path

try:
    from tools.rotating_earth_trim import rotating_fixture_source
except ModuleNotFoundError:
    from rotating_earth_trim import rotating_fixture_source

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import run_files
from taoryx.trim import solve_trim
from taoryx.trim_catalog import load_trim_catalog
from taoryx.vehicle_registry import vehicle_definition, vehicle_status_line

ROOT = Path(__file__).resolve().parents[1]
TABLE = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/b747_nominal_elevator_6axis.tbl"
OUTPUT = ROOT / "artifacts/golden_plants/b747_condition3_trim_report.json"
SOURCE_GRID = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/jet_b747/aero/static_six_axis_grid.csv"
BASE_QUATERNION = (0.5, -0.5, -0.5, 0.5)
B747 = vehicle_definition("b747")


def _quaternion_for_alpha(alpha_rad: float) -> tuple[float, float, float, float]:
    """Return the canonical east-flight quaternion at actual alpha."""

    half = alpha_rad / 2.0
    w, x, y, z = BASE_QUATERNION
    pitch = (math.cos(half), 0.0, math.sin(half), 0.0)
    return (
        w * pitch[0] - x * pitch[1] - y * pitch[2] - z * pitch[3],
        w * pitch[1] + x * pitch[0] + y * pitch[3] - z * pitch[2],
        w * pitch[2] - x * pitch[3] + y * pitch[0] + z * pitch[1],
        w * pitch[3] + x * pitch[2] - y * pitch[1] + z * pitch[0],
    )
    ####


def _problem(alpha_offset_deg: float, thrust_n: float, elevator_deg: float, earth_omega_rad_s: float) -> str:
    """Build one standard problem-file candidate around condition 3."""

    actual_alpha = math.radians(3.1 + alpha_offset_deg)
    qw, qx, qy, qz = _quaternion_for_alpha(actual_alpha)
    problem = f"""(b747-condition3-trim-candidate)
*title B747 condition 3 trim candidate
*mode rigid-body-6dof
*atmos standard
*earth wgs-84 omega=0
{vehicle_status_line("b747")}
*runtime control elevator-deg vehicle=1 default={elevator_deg:.16g} lower=-10 upper=10
*runtime status actuator maximum-moment=100000000 maximum-body-rate-deg-s=90
*runtime status thermal policy=none
*trajectory 1 b747 start on 1
  *initial ecic x=6378237.0 y=0.0 z=0.0 xdt=0.0 ydt=153.0096 zdt=0.0 qw={qw:.16g} qx={qx:.16g} qy={qy:.16g} qz={qz:.16g} time=0.0 mass=288756.9 propellant_mass=0.0
  *segment 1 condition3-trim
    *integ dtprnt=0.05 dt=0.005
    *prop thrust={thrust_n:.16g} mdot=0
    *aero cx=(cx) cy=(cy) cz=(cz) cmx=(cmx) cmy=(cmy) cmz=(cmz)
    *when time>0.005 stop
*end
"""
    return rotating_fixture_source(problem, earth_omega_rad_s)
    ####


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def _residual(state_values: dict[str, float], control_values: dict[str, float], work: Path, earth_omega_rad_s: float) -> dict[str, float]:
    """Return normalized axial and vertical force residuals."""

    alpha_offset_deg = state_values["alpha_offset_deg"]
    thrust_n = control_values["thrust_n"]
    elevator_deg = control_values["elevator_deg"]
    problem = work / "candidate.prb"
    problem.write_text(_problem(alpha_offset_deg, thrust_n, elevator_deg, earth_omega_rad_s), encoding="utf-8")
    report = run_files(problem, (TABLE,), output_dir=work / "run", max_steps=2, integrator="rk4", profile=GrammarProfile.TAORYX)
    if not report.results:
        raise RuntimeError("trim candidate failed before producing a state")
    state = report.results[0].states["1"][0].named
    scale = max(float(state["mass"]) * 9.80665, 1.0)
    moment_scale = max(float(state["mass"]) * 9.80665 * float(B747["reference_length_m"]), 1.0)
    return {
        "body_x_force": float(state["total_force_body_x_n"]) / scale,
        "body_z_force": float(state["total_force_body_z_n"]) / scale,
        "pitch_moment": float(state["total_moment_body_y_nm"]) / moment_scale,
    }
    ####


def main() -> None:
    """Solve and write the machine-readable B747 trim report."""

    earth_omega_rad_s = float(os.environ.get("TAORYX_TRIM_EARTH_OMEGA", "0.0"))
    with tempfile.TemporaryDirectory(prefix="taoryx-b747-trim-") as directory:
        work = Path(directory)
        spec = load_trim_catalog(ROOT / "verification/trim_specs.yaml").get("b747-condition3-trim-v1").to_spec()
        result = solve_trim(spec, lambda state, controls: _residual(dict(state), dict(controls), work, earth_omega_rad_s), max_nfev=100, residual_tolerance=1.0e-11)
        parameters = (result.state["alpha_offset_deg"], result.controls["thrust_n"], result.controls["elevator_deg"])
        residual = result.residuals
        payload = {
            "vehicle": "b747",
            "earth_omega_rad_s": earth_omega_rad_s,
            "operating_point_mode": "zero_rate_source_parity" if earth_omega_rad_s == 0.0 else "rotating_earth_representative",
            "source_anchor": "NASA CR-2144 condition 3 / Mach 0.45",
            "claim": "runtime force trim using corrected source-transcoded static deck",
            "parameters": {"alpha_offset_deg": parameters[0], "thrust_n": parameters[1], "elevator_deg": parameters[2]},
            "residual_normalized": {"body_x": residual["body_x_force"], "body_z": residual["body_z_force"], "moment_y": residual["pitch_moment"]},
            "trim_diagnostics": [diagnostic.as_dict() for diagnostic in result.diagnostics],
            "residual_norm_l2": math.sqrt(sum(value * value for value in residual.values())),
            "acceptance_gate": {
                "translation_norm_lt": 0.01,
                "rotation_norm_lt": 0.001,
                "passed": max(abs(residual["body_x_force"]), abs(residual["body_z_force"])) < 0.01 and abs(residual["pitch_moment"]) < 0.001,
            },
            "provenance": {
                "source_grid": str(SOURCE_GRID.relative_to(ROOT)),
                "source_grid_sha256": _sha256(SOURCE_GRID),
                "runtime_table": str(TABLE.relative_to(ROOT)),
                "runtime_table_sha256": _sha256(TABLE),
            },
            "solver": {"success": result.success, "status": result.status, "message": result.message, "nfev": result.iterations},
        }
    output = Path(os.environ.get("TAORYX_TRIM_OUTPUT", str(OUTPUT)))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output)
    ####


if __name__ == "__main__":
    main()
