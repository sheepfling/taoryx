"""Solve the X-15 source-trim candidate through the common trim contract."""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import tempfile
from pathlib import Path

from taoryx.contracts import Vector3
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.modes import Quaternion
from taoryx.runtime.common import RuntimeState, RuntimeVehicle
from taoryx.runtime.program import LoadedProgram
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


def _state_at_alpha(base: RuntimeState, base_alpha_deg: float, alpha_deg: float) -> RuntimeState:
    """Apply an alpha perturbation about the source release attitude.

    The source problem carries a rigid-body attitude and inertial velocity;
    alpha is therefore represented by a body-pitch perturbation, not by
    rewriting a guidance placeholder.  This keeps the trim variable tied to
    the same air-data calculation used by the rigid-body plant.
    """

    delta = math.radians(alpha_deg - base_alpha_deg)
    current = Quaternion(
        float(base.named["qw"]),
        float(base.named["qx"]),
        float(base.named["qy"]),
        float(base.named["qz"]),
    )
    perturbation = Quaternion(math.cos(delta / 2.0), 0.0, math.sin(delta / 2.0), 0.0)
    attitude = current.multiply(perturbation).normalized()
    values = list(base.values)
    for name, value in zip(("qw", "qx", "qy", "qz"), (attitude.w, attitude.x, attitude.y, attitude.z), strict=True):
        values[base.value_names.index(name)] = value
    return base.with_values(values)
    ####


def _residual(
    state_values: dict[str, float],
    control_values: dict[str, float],
    vehicle: RuntimeVehicle,
    base_state: RuntimeState,
    base_alpha_deg: float,
) -> dict[str, float]:
    """Evaluate one cached source-bound plant residual in memory."""

    alpha_deg = float(state_values["alpha_deg"])
    candidate_state = _state_at_alpha(base_state, base_alpha_deg, alpha_deg)
    vehicle.state = candidate_state
    vehicle.history[0] = candidate_state
    for name, value in control_values.items():
        degree_name = name.replace("_", "-")
        numeric_value = float(value)
        vehicle.control_values[degree_name] = numeric_value
        # The rigid-body table adapters query the canonical radian aliases;
        # retain the degree command as well so diagnostics and source-facing
        # controls remain auditable.
        vehicle.control_values[degree_name.removesuffix("-deg")] = math.radians(numeric_value)
    # Keep the candidate namespace coherent for both the generic runtime
    # evaluator and the rigid-body aerodynamic control provider.
    candidate_named = {**candidate_state.named, **vehicle.control_values}
    candidate_state = RuntimeState(
        candidate_state.time,
        candidate_state.values,
        candidate_state.frame,
        candidate_named,
        candidate_state.value_names,
        candidate_state.segment_endpoints,
    )
    vehicle.state = candidate_state
    vehicle.history[0] = candidate_state
    if vehicle.environment_evaluator is None:
        raise RuntimeError("X-15 cached program has no rigid-body observable evaluator")
    # The evaluator consumes one coherent namespace containing both the
    # candidate rigid-body state and the candidate runtime controls.  Passing
    # only ``candidate_state.named`` silently left the closure's original
    # source controls active while the solver varied ``control_values``.
    observed = vehicle.environment_evaluator({**candidate_state.named, **vehicle.control_values})
    force_scale = max(float(candidate_state.named["mass"]) * 9.80665, 1.0)
    moment_scale = max(force_scale * float(X15["reference_length_m"]), 1.0)
    current = Quaternion(
        float(candidate_state.named["qw"]),
        float(candidate_state.named["qx"]),
        float(candidate_state.named["qy"]),
        float(candidate_state.named["qz"]),
    )
    velocity_body = current.conjugate().rotate(
        Vector3(
            float(candidate_state.named.get("xdt", candidate_state.named.get("xdot", candidate_state.named.get("xdot_ecic", 0.0)))),
            float(candidate_state.named.get("ydt", candidate_state.named.get("ydot", candidate_state.named.get("ydot_ecic", 0.0)))),
            float(candidate_state.named.get("zdt", candidate_state.named.get("zdot", candidate_state.named.get("zdot_ecic", 0.0)))),
        )
    )
    total_force = Vector3(
        float(observed["total_force_body_x_n"]),
        float(observed["total_force_body_y_n"]),
        float(observed["total_force_body_z_n"]),
    )
    force_velocity_cross = total_force.cross(velocity_body)
    force_velocity_scale = max(force_scale * max(velocity_body.norm(), 1.0), 1.0)
    return {
        "force_velocity_cross_y": float(force_velocity_cross.y) / force_velocity_scale,
        "force_velocity_cross_z": float(force_velocity_cross.z) / force_velocity_scale,
        "roll_moment": float(observed["total_moment_body_x_nm"]) / moment_scale,
        "pitch_moment": float(observed["total_moment_body_y_nm"]) / moment_scale,
        "yaw_moment": float(observed["total_moment_body_z_nm"]) / moment_scale,
    }
    ####


def main() -> None:
    """Solve and write the X-15 source-trim report."""

    with tempfile.TemporaryDirectory(prefix="taoryx-x15-trim-") as directory:
        work = Path(directory)
        spec = load_trim_catalog(ROOT / "verification/trim_specs.yaml").get("x15-release-glide-v1").to_spec()
        candidate = work / "candidate.prb"
        candidate.write_text(
            _candidate(
                PROBLEM.read_text(encoding="utf-8"),
                dict(spec.state_initial),
                dict(spec.control_initial),
            ),
            encoding="utf-8",
        )
        program = LoadedProgram.load(candidate, TABLES, profile=GrammarProfile.TAORYX)
        vehicle = program.case().vehicles["1"]
        if vehicle.environment_evaluator is None:
            raise RuntimeError("X-15 source candidate did not produce a runtime evaluator")
        base_state = vehicle.state
        base_observables = vehicle.environment_evaluator(base_state.named)
        base_alpha_deg = float(base_observables["aero_alpha_deg"])
        result = solve_trim(
            spec,
            lambda state, controls: _residual(dict(state), dict(controls), vehicle, base_state, base_alpha_deg),
            max_nfev=100,
            residual_tolerance=1.0e-10,
            acceptance_tolerance=1.0e-3,
        )
    payload = {
        "vehicle": "x15",
        "earth_omega_rad_s": 7.2921151467e-5,
        "operating_point_mode": "rotating_earth_representative",
        "source_anchor": "source-trimmed release glide",
        "claim": "local source-trim candidate through the common trim solver and cached rigid-body glide residual adapter",
        "status": "pass" if result.success else "blocked",
        "state": dict(result.state),
        "controls": dict(result.controls),
        "residual_normalized": dict(result.residuals),
        "trim_diagnostics": [diagnostic.as_dict() for diagnostic in result.diagnostics],
        "residual_norm_l2": math.sqrt(sum(value * value for value in result.residuals.values())),
        "acceptance_gate": {
            "force_velocity_cross_norm_lt": 1.0e-3,
            "moment_norm_lt": 1.0e-3,
            "passed": result.success,
        },
        "diagnostic": {
            "source_only_preserved": not result.success,
            "reason": (
                "bounded glide residual solve did not satisfy force-velocity alignment and zero-moment equilibrium; "
                "the source release state is not silently reclassified as a trim"
                if not result.success
                else "bounded glide residual solve satisfied force-velocity alignment and zero-moment equilibrium"
            ),
            "state_bound_hit": bool(
                result.state.get("alpha_deg") == result.spec.state_lower.get("alpha_deg")
                if result.spec.state_lower
                else False
            ),
            "source_release_is_equilibrium_claim": False,
        },
        "provenance": {
            "problem": str(PROBLEM.relative_to(ROOT)),
            "problem_sha256": _sha256(PROBLEM),
            "runtime_tables": {str(path.relative_to(ROOT)): _sha256(path) for path in TABLES},
        },
        "solver": {
            "success": result.success,
            "status": result.status,
            "message": result.message,
            "nfev": result.iterations,
            "evaluation_path": "loaded-program-in-memory",
            "tables_rebound_per_evaluation": False,
        },
    }
    output = Path(os.environ.get("TAORYX_TRIM_OUTPUT", str(OUTPUT)))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output)
    ####


if __name__ == "__main__":
    main()
