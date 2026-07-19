"""Solve the first B747 source-anchor force trim using the TAORYX runtime.

This is deliberately a plant-level research tool.  It calls ``run_files`` for
each candidate, so the residual is produced by the same table, gravity, frame,
and rigid-body path used by the trajectory runner.
"""

from __future__ import annotations

import hashlib
import json
import math
import tempfile
from pathlib import Path

from scipy.optimize import least_squares

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[1]
TABLE = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/b747_nominal_elevator_6axis.tbl"
OUTPUT = ROOT / "artifacts/golden_plants/b747_condition3_trim_report.json"
SOURCE_GRID = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/jet_b747/aero/static_six_axis_grid.csv"
BASE_QUATERNION = (0.5, -0.5, -0.5, 0.5)


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


def _problem(alpha_offset_deg: float, thrust_n: float, elevator_deg: float) -> str:
    """Build one standard problem-file candidate around condition 3."""

    actual_alpha = math.radians(3.1 + alpha_offset_deg)
    qw, qx, qy, qz = _quaternion_for_alpha(actual_alpha)
    return f"""(b747-condition3-trim-candidate)
*title B747 condition 3 trim candidate
*mode rigid-body-6dof
*atmos standard
*earth wgs-84 omega=0
*runtime status vehicle reference-area=510.96672 reference-length=8.324088 dry-mass-kg=288756.9 inertia-x=24675886.7 inertia-y=44877574.1 inertia-z=67384152.0 envelope-min-forward-speed=20 envelope-max-mach=0.9 envelope-max-alpha-deg=4 envelope-max-beta-deg=5 aero-alpha-reference-deg=3.1
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
    ####


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def _residual(parameters: tuple[float, float, float], work: Path) -> tuple[float, float, float]:
    """Return normalized axial and vertical force residuals."""

    alpha_offset_deg, thrust_n, elevator_deg = parameters
    problem = work / "candidate.prb"
    problem.write_text(_problem(alpha_offset_deg, thrust_n, elevator_deg), encoding="utf-8")
    report = run_files(problem, (TABLE,), output_dir=work / "run", max_steps=2, integrator="rk4", profile=GrammarProfile.TAORYX)
    if not report.results:
        raise RuntimeError("trim candidate failed before producing a state")
    state = report.results[0].states["1"][0].named
    scale = max(float(state["mass"]) * 9.80665, 1.0)
    moment_scale = max(float(state["mass"]) * 9.80665 * 8.324088, 1.0)
    return (
        float(state["total_force_body_x_n"]) / scale,
        float(state["total_force_body_z_n"]) / scale,
        float(state["total_moment_body_y_nm"]) / moment_scale,
    )
    ####


def main() -> None:
    """Solve and write the machine-readable B747 trim report."""

    with tempfile.TemporaryDirectory(prefix="taoryx-b747-trim-") as directory:
        work = Path(directory)
        result = least_squares(
            lambda values: _residual((float(values[0]), float(values[1]), float(values[2])), work),
            x0=(0.0, 122_000.0, 0.0),
            bounds=([-3.9, 0.0, -10.0], [3.9, 1_000_000.0, 10.0]),
            xtol=1.0e-11,
            ftol=1.0e-11,
            gtol=1.0e-11,
            max_nfev=100,
        )
        parameters = (float(result.x[0]), float(result.x[1]), float(result.x[2]))
        residual = _residual(parameters, work)
        payload = {
            "vehicle": "b747",
            "source_anchor": "NASA CR-2144 condition 3 / Mach 0.45",
            "claim": "runtime force trim using corrected source-transcoded static deck",
            "parameters": {"alpha_offset_deg": parameters[0], "thrust_n": parameters[1], "elevator_deg": parameters[2]},
            "residual_normalized": {"body_x": residual[0], "body_z": residual[1], "moment_y": residual[2]},
            "residual_norm_l2": math.sqrt(sum(value * value for value in residual)),
            "acceptance_gate": {
                "translation_norm_lt": 0.01,
                "rotation_norm_lt": 0.001,
                "passed": max(abs(residual[0]), abs(residual[1])) < 0.01 and abs(residual[2]) < 0.001,
            },
            "provenance": {
                "source_grid": str(SOURCE_GRID.relative_to(ROOT)),
                "source_grid_sha256": _sha256(SOURCE_GRID),
                "runtime_table": str(TABLE.relative_to(ROOT)),
                "runtime_table_sha256": _sha256(TABLE),
            },
            "solver": {"success": bool(result.success), "status": int(result.status), "message": str(result.message), "nfev": int(result.nfev)},
        }
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(OUTPUT)
    ####


if __name__ == "__main__":
    main()
