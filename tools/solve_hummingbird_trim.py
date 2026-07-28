"""Solve the Hummingbird equal-rotor hover trim through the native plant."""

from __future__ import annotations

import hashlib
import json
import os
import re
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

ROOT = Path(__file__).resolve().parents[1]
PROBLEM = ROOT / "examples/mission_families/slower_hummingbird/SV05_hover_validation_6dof.prb"
TABLE_ROOT = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables"
TABLES = tuple(TABLE_ROOT / name for name in ("hummingbird_cx.tbl", "hummingbird_cy.tbl", "hummingbird_cz.tbl", "hummingbird_cmx.tbl", "hummingbird_cmy.tbl", "hummingbird_cmz.tbl"))
OUTPUT = ROOT / "artifacts/golden_plants/hummingbird_hover_trim_report.json"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def _candidate(source: str, rotor_speed: float, earth_omega_rad_s: float) -> str:
    updated = re.sub(
        r"(\*runtime control rotor-speed vehicle=1 default=)[0-9.eE+-]+",
        rf"\g<1>{rotor_speed:.16g}",
        source,
    )
    return rotating_fixture_source(updated, earth_omega_rad_s)
    ####


def _residual(state_values: dict[str, float], control_values: dict[str, float], work: Path, earth_omega_rad_s: float) -> dict[str, float]:
    rotor_speed = control_values["equal_rotor_speed_rad_s"]
    problem = work / "candidate.prb"
    problem.write_text(_candidate(PROBLEM.read_text(encoding="utf-8"), rotor_speed, earth_omega_rad_s), encoding="utf-8")
    report = run_files(problem, TABLES, output_dir=work / "run", max_steps=2, integrator="rk4", profile=GrammarProfile.TAORYX)
    if not report.results:
        raise RuntimeError([(item.code, item.message) for item in report.diagnostics])
    state = report.results[0].states["1"][0].named
    force_scale = max(float(state["mass_kg"]) * 9.80665, 1.0)
    moment_scale = max(force_scale * 0.34, 1.0)
    return {
        "body_x_force": float(state["total_force_body_x_n"]) / force_scale,
        "body_z_force": float(state["total_force_body_z_n"]) / force_scale,
        "yaw_moment": float(state["total_moment_body_z_nm"]) / moment_scale,
    }
    ####


def main() -> None:
    """Solve and write the Hummingbird hover trim evidence report."""

    earth_omega_rad_s = float(os.environ.get("TAORYX_TRIM_EARTH_OMEGA", "0.0"))
    with tempfile.TemporaryDirectory(prefix="taoryx-hummingbird-trim-") as directory:
        work = Path(directory)
        spec = load_trim_catalog(ROOT / "verification/trim_specs.yaml").get("hummingbird-hover-v1").to_spec()
        result = solve_trim(spec, lambda state, controls: _residual(dict(state), dict(controls), work, earth_omega_rad_s), max_nfev=50, residual_tolerance=1.0e-12)
        rotor_speed = result.controls["equal_rotor_speed_rad_s"]
        residual = result.residuals
    payload = {
        "vehicle": "asctec-hummingbird",
        "earth_omega_rad_s": earth_omega_rad_s,
        "operating_point_mode": "zero_rate_source_parity" if earth_omega_rad_s == 0.0 else "rotating_earth_representative",
        "source_anchor": "RotorPy-derived equal-rotor hover",
        "claim": "open-loop hover trim using the native direct-wrench plant",
        "parameters": {"equal_rotor_speed_rad_s": rotor_speed},
        "residual_normalized": {"body_x": residual["body_x_force"], "body_z": residual["body_z_force"], "moment_z": residual["yaw_moment"]},
        "trim_diagnostics": [diagnostic.as_dict() for diagnostic in result.diagnostics],
        "residual_norm_l2": sum(value * value for value in residual.values()) ** 0.5,
        "acceptance_gate": {
            "translation_norm_lt": 0.01,
            "rotation_norm_lt": 0.001,
            "passed": max(abs(residual["body_x_force"]), abs(residual["body_z_force"])) < 0.01 and abs(residual["yaw_moment"]) < 0.001,
        },
        "provenance": {
            "problem": str(PROBLEM.relative_to(ROOT)),
            "problem_sha256": _sha256(PROBLEM),
            "runtime_tables": {str(path.relative_to(ROOT)): _sha256(path) for path in TABLES},
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
