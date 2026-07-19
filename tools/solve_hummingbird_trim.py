"""Solve the Hummingbird equal-rotor hover trim through the native plant."""

from __future__ import annotations

import hashlib
import json
import re
import tempfile
from pathlib import Path

from scipy.optimize import least_squares

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[1]
PROBLEM = ROOT / "examples/mission_families/slower_hummingbird/SV05_hover_validation_6dof.prb"
TABLE_ROOT = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables"
TABLES = tuple(TABLE_ROOT / name for name in ("hummingbird_cx.tbl", "hummingbird_cy.tbl", "hummingbird_cz.tbl", "hummingbird_cmx.tbl", "hummingbird_cmy.tbl", "hummingbird_cmz.tbl"))
OUTPUT = ROOT / "artifacts/golden_plants/hummingbird_hover_trim_report.json"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def _candidate(source: str, rotor_speed: float) -> str:
    return re.sub(
        r"(\*runtime control rotor-speed vehicle=1 default=)[0-9.eE+-]+",
        rf"\g<1>{rotor_speed:.16g}",
        source,
    )
    ####


def _residual(values: tuple[float], work: Path) -> tuple[float, float, float]:
    rotor_speed = values[0]
    problem = work / "candidate.prb"
    problem.write_text(_candidate(PROBLEM.read_text(encoding="utf-8"), rotor_speed), encoding="utf-8")
    report = run_files(problem, TABLES, output_dir=work / "run", max_steps=2, integrator="rk4", profile=GrammarProfile.TAORYX)
    if not report.results:
        raise RuntimeError([(item.code, item.message) for item in report.diagnostics])
    state = report.results[0].states["1"][0].named
    force_scale = max(float(state["mass_kg"]) * 9.80665, 1.0)
    moment_scale = max(force_scale * 0.34, 1.0)
    return (
        float(state["total_force_body_x_n"]) / force_scale,
        float(state["total_force_body_z_n"]) / force_scale,
        float(state["total_moment_body_z_nm"]) / moment_scale,
    )
    ####


def main() -> None:
    """Solve and write the Hummingbird hover trim evidence report."""

    with tempfile.TemporaryDirectory(prefix="taoryx-hummingbird-trim-") as directory:
        work = Path(directory)
        result = least_squares(
            lambda values: _residual((float(values[0]),), work),
            x0=(469.124102661955,),
            bounds=([0.0], [1500.0]),
            xtol=1.0e-12,
            ftol=1.0e-12,
            gtol=1.0e-12,
            max_nfev=50,
        )
        rotor_speed = float(result.x[0])
        residual = _residual((rotor_speed,), work)
    payload = {
        "vehicle": "asctec-hummingbird",
        "source_anchor": "RotorPy-derived equal-rotor hover",
        "claim": "open-loop hover trim using the native direct-wrench plant",
        "parameters": {"equal_rotor_speed_rad_s": rotor_speed},
        "residual_normalized": {"body_x": residual[0], "body_z": residual[1], "moment_z": residual[2]},
        "residual_norm_l2": sum(value * value for value in residual) ** 0.5,
        "acceptance_gate": {
            "translation_norm_lt": 0.01,
            "rotation_norm_lt": 0.001,
            "passed": max(abs(residual[0]), abs(residual[1])) < 0.01 and abs(residual[2]) < 0.001,
        },
        "provenance": {
            "problem": str(PROBLEM.relative_to(ROOT)),
            "problem_sha256": _sha256(PROBLEM),
            "runtime_tables": {str(path.relative_to(ROOT)): _sha256(path) for path in TABLES},
        },
        "solver": {"success": bool(result.success), "status": int(result.status), "message": str(result.message), "nfev": int(result.nfev)},
    }
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(OUTPUT)
    ####


if __name__ == "__main__":
    main()
