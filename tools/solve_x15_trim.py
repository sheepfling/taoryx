"""Solve the X-15 source-trim candidate through the common trim contract."""

from __future__ import annotations

import hashlib
import json
import math
import re
import tempfile
from pathlib import Path

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import run_files
from taoryx.trim import solve_trim
from taoryx.trim_catalog import load_trim_catalog
from taoryx.vehicle_registry import vehicle_definition

ROOT = Path(__file__).resolve().parents[1]
PROBLEM = ROOT / "examples/showcases/x15_rocket_to_hawaii/source_trim_hold_6dof.prb"
TABLE_ROOT = ROOT / "tests/fixtures/x15_coherent_6dof_public_research_v1/tables"
TABLES = tuple(TABLE_ROOT / name for name in ("x15_static_6axis.tbl", "x15_symmetric_stabilator_6axis.tbl", "x15_differential_stabilator_6axis.tbl", "x15_rudder_6axis.tbl"))
OUTPUT = ROOT / "artifacts/golden_plants/x15_release_glide_trim_report.json"
X15 = vehicle_definition("x15")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def _candidate(source: str, state: dict[str, float], controls: dict[str, float]) -> str:
    updated = source
    updated = re.sub(r"(\*fly alpha=)[0-9.eE+-]+", rf"\g<1>{state['alpha_deg']:.16g}", updated, count=1)
    for name, value in controls.items():
        updated, count = re.subn(
            rf"(\*runtime control {re.escape(name.replace('_', '-'))}\b[^\n]*?\bdefault=)[0-9.eE+-]+",
            rf"\g<1>{value:.16g}",
            updated,
            count=1,
        )
        if count != 1:
            raise ValueError(f"could not locate X-15 control {name!r}")
    updated = updated.replace("rudder-hold-gain-deg-per-deg=1.0", "rudder-hold-gain-deg-per-deg=0.0")
    updated = re.sub(r"\*when time>[^\n]+ stop", "*when time>0.005 stop", updated, count=1)
    return updated
    ####


def _residual(state_values: dict[str, float], control_values: dict[str, float], work: Path) -> dict[str, float]:
    candidate = work / "candidate.prb"
    candidate.write_text(_candidate(PROBLEM.read_text(encoding="utf-8"), state_values, control_values), encoding="utf-8")
    report = run_files(candidate, TABLES, output_dir=work / "run", max_steps=10, integrator="rk4", profile=GrammarProfile.TAORYX)
    if not report.results:
        raise RuntimeError([(item.code, item.message) for item in report.diagnostics])
    states = report.results[0].states
    state = next(iter(states.values()))[0].named
    force_scale = max(float(state["mass_kg"]) * 9.80665, 1.0)
    moment_scale = max(force_scale * float(X15["reference_length_m"]), 1.0)
    return {
        "body_x_force": float(state["total_force_body_x_n"]) / force_scale,
        "body_z_force": float(state["total_force_body_z_n"]) / force_scale,
        "roll_moment": float(state["total_moment_body_x_nm"]) / moment_scale,
        "pitch_moment": float(state["total_moment_body_y_nm"]) / moment_scale,
        "yaw_moment": float(state["total_moment_body_z_nm"]) / moment_scale,
    }
    ####


def main() -> None:
    """Solve and write the X-15 source-trim report."""

    with tempfile.TemporaryDirectory(prefix="taoryx-x15-trim-") as directory:
        work = Path(directory)
        spec = load_trim_catalog(ROOT / "verification/trim_specs.yaml").get("x15-release-glide-v1").to_spec()
        result = solve_trim(
            spec,
            lambda state, controls: _residual(dict(state), dict(controls), work),
            max_nfev=100,
            residual_tolerance=1.0e-10,
            acceptance_tolerance=1.0e-3,
        )
    payload = {
        "vehicle": "x15",
        "source_anchor": "source-trimmed release glide",
        "claim": "local source-trim candidate through the common trim solver",
        "status": "pass" if result.success else "blocked",
        "state": dict(result.state),
        "controls": dict(result.controls),
        "residual_normalized": dict(result.residuals),
        "residual_norm_l2": math.sqrt(sum(value * value for value in result.residuals.values())),
        "acceptance_gate": {"translation_norm_lt": 0.01, "rotation_norm_lt": 0.001, "passed": result.success},
        "provenance": {
            "problem": str(PROBLEM.relative_to(ROOT)),
            "problem_sha256": _sha256(PROBLEM),
            "runtime_tables": {str(path.relative_to(ROOT)): _sha256(path) for path in TABLES},
        },
        "solver": {"success": result.success, "status": result.status, "message": result.message, "nfev": result.iterations},
    }
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(OUTPUT)
    ####


if __name__ == "__main__":
    main()
