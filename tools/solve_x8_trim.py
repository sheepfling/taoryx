"""Solve a bounded powered Skywalker X8 research-surrogate trim.

The X8 trim uses the source-composed static, collective, and differential
six-axis decks plus the published simplified thrust map.  The resulting
candidate closes all three body moments, not only the pitch moment.
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
STATIC_TABLE = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/skywalker_x8_static_6axis.tbl"
COLLECTIVE_TABLE = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/skywalker_x8_collective_elevon_6axis.tbl"
DIFFERENTIAL_TABLE = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/skywalker_x8_differential_elevon_6axis.tbl"
THRUST_TABLE = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/skywalker_x8_thrust.tbl"
SOURCE_GRID = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/cruise_class_uav_skywalker_x8/aero/static_airframe_grid.csv"
OUTPUT = ROOT / "artifacts/golden_plants/x8_powered_trim_report.json"
BASE_QUATERNION = (0.5, -0.5, -0.5, 0.5)
X8 = vehicle_definition("skywalker_x8")


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


def _problem(alpha_deg: float, throttle: float, collective_deg: float, differential_deg: float, earth_omega_rad_s: float) -> str:
    qw, qx, qy, qz = _quaternion_for_alpha(math.radians(alpha_deg))
    problem = f"""(x8-powered-trim-candidate)
*title Skywalker X8 powered static-deck trim candidate
*mode rigid-body-6dof
*atmos standard
*earth wgs-84 omega=0
{vehicle_status_line("skywalker_x8")}
*runtime control throttle vehicle=1 default={throttle:.16g} lower=0 upper=1
*runtime control collective-elevon-deg vehicle=1 default={collective_deg:.16g} lower=-20 upper=20
*runtime control differential-elevon-deg vehicle=1 default={differential_deg:.16g} lower=-20 upper=20
*runtime status actuator maximum-moment=20 maximum-body-rate-deg-s=360
*runtime status thermal policy=none
*define x8-cx
  x8-cx=(cx-static)+(cx-collective)-(cx-static)+(cx-differential)-(cx-static);
*define x8-cy
  x8-cy=(cy-static)+(cy-collective)-(cy-static)+(cy-differential)-(cy-static);
*define x8-cz
  x8-cz=(cz-static)+(cz-collective)-(cz-static)+(cz-differential)-(cz-static);
*define x8-cmx
  x8-cmx=(cmx-static)+(cmx-collective)-(cmx-static)+(cmx-differential)-(cmx-static);
*define x8-cmy
  x8-cmy=(cmy-static)+(cmy-collective)-(cmy-static)+(cmy-differential)-(cmy-static);
*define x8-cmz
  x8-cmz=(cmz-static)+(cmz-collective)-(cmz-static)+(cmz-differential)-(cmz-static);
*trajectory 1 x8 start on 1
  *initial ecic x=6378315.0 y=0.0 z=0.0 xdt=0.0 ydt=17.9 zdt=0.0 qw={qw:.16g} qx={qx:.16g} qy={qy:.16g} qz={qz:.16g} time=0.0 mass=3.364 propellant_mass=0.0
  *segment 1 powered-trim
    *integ dtprnt=0.05 dt=0.005
    *prop thrust=(thrust) mdot=0
    *aero cx=x8-cx cy=x8-cy cz=x8-cz cmx=x8-cmx cmy=x8-cmy cmz=x8-cmz
    *when time>0.005 stop
*end
"""
    return rotating_fixture_source(problem, earth_omega_rad_s)
    ####


def _residual(state_values: dict[str, float], control_values: dict[str, float], work: Path, earth_omega_rad_s: float) -> dict[str, float]:
    alpha_deg = state_values["alpha_deg"]
    throttle = control_values["throttle"]
    collective_deg = control_values["collective_elevon_deg"]
    differential_deg = control_values["differential_elevon_deg"]
    problem = work / "candidate.prb"
    problem.write_text(_problem(alpha_deg, throttle, collective_deg, differential_deg, earth_omega_rad_s), encoding="utf-8")
    report = run_files(
        problem,
        (STATIC_TABLE, COLLECTIVE_TABLE, DIFFERENTIAL_TABLE, THRUST_TABLE),
        output_dir=work / "run",
        max_steps=2,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
    if not report.results:
        raise RuntimeError([(item.code, item.message) for item in report.diagnostics])
    state = report.results[0].states["1"][0].named
    force_scale = max(float(state["mass_kg"]) * 9.80665, 1.0)
    moment_scale = max(force_scale * float(X8["reference_length_m"]), 1.0)
    return {
        "body_x_force": float(state["total_force_body_x_n"]) / force_scale,
        "body_z_force": float(state["total_force_body_z_n"]) / force_scale,
        "roll_moment": float(state["total_moment_body_x_nm"]) / moment_scale,
        "pitch_moment": float(state["total_moment_body_y_nm"]) / moment_scale,
        "yaw_moment": float(state["total_moment_body_z_nm"]) / moment_scale,
    }
    ####


def main() -> None:
    """Solve and write the X8 powered trim evidence report."""

    earth_omega_rad_s = float(os.environ.get("TAORYX_TRIM_EARTH_OMEGA", "0.0"))
    with tempfile.TemporaryDirectory(prefix="taoryx-x8-trim-") as directory:
        work = Path(directory)
        spec = load_trim_catalog(ROOT / "verification/trim_specs.yaml").get("x8-powered-trim-v1").to_spec()
        result = solve_trim(
            spec,
            lambda state, controls: _residual(dict(state), dict(controls), work, earth_omega_rad_s),
            max_nfev=100,
            residual_tolerance=1.0e-11,
            acceptance_tolerance=1.0e-3,
        )
        parameters = (result.state["alpha_deg"], result.controls["throttle"], result.controls["collective_elevon_deg"], result.controls["differential_elevon_deg"])
        residual = result.residuals
    payload = {
        "vehicle": "skywalker-x8",
        "earth_omega_rad_s": earth_omega_rad_s,
        "operating_point_mode": "zero_rate_source_parity" if earth_omega_rad_s == 0.0 else "rotating_earth_representative",
        "source_anchor": "flight-identified published trim neighborhood",
        "claim": "powered trim using composed static, collective, and differential six-axis decks plus simplified thrust",
        "control_decks": "static, collective-elevon, and differential-elevon increments composed at the source-table boundary",
        "parameters": {"alpha_deg": parameters[0], "throttle": parameters[1], "collective_elevon_deg": parameters[2], "differential_elevon_deg": parameters[3]},
        "residual_normalized": {"body_x": residual["body_x_force"], "body_z": residual["body_z_force"], "moment_x": residual["roll_moment"], "moment_y": residual["pitch_moment"], "moment_z": residual["yaw_moment"]},
        "trim_diagnostics": [diagnostic.as_dict() for diagnostic in result.diagnostics],
        "residual_norm_l2": math.sqrt(sum(value * value for value in residual.values())),
        "acceptance_gate": {
            "translation_norm_lt": 0.01,
            "rotation_norm_lt": 0.001,
            "passed": max(abs(residual["body_x_force"]), abs(residual["body_z_force"])) < 0.01 and max(abs(residual["roll_moment"]), abs(residual["pitch_moment"]), abs(residual["yaw_moment"])) < 0.001,
        },
        "provenance": {
            "source_grid": str(SOURCE_GRID.relative_to(ROOT)),
            "source_grid_sha256": _sha256(SOURCE_GRID),
            "runtime_tables": {str(path.relative_to(ROOT)): _sha256(path) for path in (STATIC_TABLE, COLLECTIVE_TABLE, DIFFERENTIAL_TABLE, THRUST_TABLE)},
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
