"""Solve a bounded powered Skywalker X8 research-surrogate trim.

The first X8 trim deliberately uses the unique static six-axis deck and the
published simplified thrust map.  The separate collective/differential control
decks are not silently combined because the current problem binding requires a
unique runtime table name for each coefficient family.
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
STATIC_TABLE = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/skywalker_x8_static_6axis.tbl"
COLLECTIVE_TABLE = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/skywalker_x8_collective_elevon_6axis.tbl"
THRUST_TABLE = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/skywalker_x8_thrust.tbl"
SOURCE_GRID = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/cruise_class_uav_skywalker_x8/aero/static_airframe_grid.csv"
OUTPUT = ROOT / "artifacts/golden_plants/x8_powered_trim_report.json"
BASE_QUATERNION = (0.5, -0.5, -0.5, 0.5)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def _quaternion_for_alpha(alpha_rad: float) -> tuple[float, float, float, float]:
    """Return the east-flight quaternion at the requested body alpha."""

    half = alpha_rad / 2.0
    pitch = (math.cos(half), 0.0, math.sin(half), 0.0)
    w, x, y, z = BASE_QUATERNION
    return (
        w * pitch[0] - x * pitch[1] - y * pitch[2] - z * pitch[3],
        w * pitch[1] + x * pitch[0] + y * pitch[3] - z * pitch[2],
        w * pitch[2] - x * pitch[3] + y * pitch[0] + z * pitch[1],
        w * pitch[3] + x * pitch[2] - y * pitch[1] + z * pitch[0],
    )
    ####


def _problem(alpha_deg: float, throttle: float, collective_deg: float) -> str:
    qw, qx, qy, qz = _quaternion_for_alpha(math.radians(alpha_deg))
    return f"""(x8-powered-trim-candidate)
*title Skywalker X8 powered static-deck trim candidate
*mode rigid-body-6dof
*atmos standard
*earth wgs-84 omega=0
*runtime status vehicle reference-area=0.75 reference-length=0.36 dry-mass-kg=3.364 inertia-x=0.325 inertia-y=0.140 inertia-z=0.400 envelope-min-forward-speed=2 envelope-max-speed=27 envelope-max-alpha-deg=12 envelope-max-beta-deg=5
*runtime control throttle vehicle=1 default={throttle:.16g} lower=0 upper=1
*runtime control collective-elevon-deg vehicle=1 default={collective_deg:.16g} lower=-20 upper=20
*runtime status actuator maximum-moment=20 maximum-body-rate-deg-s=360
*runtime status thermal policy=none
*trajectory 1 x8 start on 1
  *initial ecic x=6378315.0 y=0.0 z=0.0 xdt=0.0 ydt=17.9 zdt=0.0 qw={qw:.16g} qx={qx:.16g} qy={qy:.16g} qz={qz:.16g} time=0.0 mass=3.364 propellant_mass=0.0
  *segment 1 powered-trim
    *integ dtprnt=0.05 dt=0.005
    *prop thrust=(thrust) mdot=0
    *aero cx=(cx) cy=(cy) cz=(cz) cmx=(cmx) cmy=(cmy) cmz=(cmz)
    *when time>0.005 stop
*end
"""
    ####


def _residual(parameters: tuple[float, float, float], work: Path) -> tuple[float, float, float]:
    alpha_deg, throttle, collective_deg = parameters
    problem = work / "candidate.prb"
    problem.write_text(_problem(alpha_deg, throttle, collective_deg), encoding="utf-8")
    report = run_files(
        problem,
        (COLLECTIVE_TABLE, THRUST_TABLE),
        output_dir=work / "run",
        max_steps=2,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
    if not report.results:
        raise RuntimeError([(item.code, item.message) for item in report.diagnostics])
    state = report.results[0].states["1"][0].named
    force_scale = max(float(state["mass_kg"]) * 9.80665, 1.0)
    moment_scale = max(force_scale * 0.36, 1.0)
    return (
        float(state["total_force_body_x_n"]) / force_scale,
        float(state["total_force_body_z_n"]) / force_scale,
        float(state["total_moment_body_y_nm"]) / moment_scale,
    )
    ####


def main() -> None:
    """Solve and write the X8 powered trim evidence report."""

    with tempfile.TemporaryDirectory(prefix="taoryx-x8-trim-") as directory:
        work = Path(directory)
        result = least_squares(
            lambda values: _residual((float(values[0]), float(values[1]), float(values[2])), work),
            x0=(7.9, 0.44, -2.35),
            bounds=([0.0, 0.0, -20.0], [12.0, 1.0, 20.0]),
            xtol=1.0e-11,
            ftol=1.0e-11,
            gtol=1.0e-11,
            max_nfev=100,
        )
        parameters = (float(result.x[0]), float(result.x[1]), float(result.x[2]))
        residual = _residual(parameters, work)
    payload = {
        "vehicle": "skywalker-x8",
        "source_anchor": "flight-identified published trim neighborhood",
        "claim": "powered trim using unique static six-axis and simplified thrust decks",
        "control_decks": "collective-elevon family bound as the active coefficient deck; differential family remains separate",
        "parameters": {"alpha_deg": parameters[0], "throttle": parameters[1], "collective_elevon_deg": parameters[2]},
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
            "runtime_tables": {str(path.relative_to(ROOT)): _sha256(path) for path in (COLLECTIVE_TABLE, THRUST_TABLE)},
        },
        "solver": {"success": bool(result.success), "status": int(result.status), "message": str(result.message), "nfev": int(result.nfev)},
    }
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(OUTPUT)
    ####


if __name__ == "__main__":
    main()
