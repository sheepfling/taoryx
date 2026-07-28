"""Attempt a source-bounded X-15 full-thrust local trim at the table epoch.

The public XLR99 deck is a time-indexed full-thrust history, not a throttle
map.  This utility therefore searches a single frozen-time operating point;
it does not claim throttle authority, engine dynamics, or a sustained powered
trim after the source thrust history evolves.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import tempfile
from pathlib import Path
from typing import Any

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.modes import Quaternion
from taoryx.runtime.common import RuntimeState, RuntimeVehicle
from taoryx.runtime.program import LoadedProgram
from taoryx.trim import TrimSpec, solve_trim
from taoryx.vehicle_registry import vehicle_definition

ROOT = Path(__file__).resolve().parents[1]
PROBLEM = ROOT / "examples/showcases/x15_rocket_to_hawaii/source_trim_hold_6dof.prb"
TABLE_ROOT = ROOT / "tests/fixtures/x15_coherent_6dof_public_research_v1/tables"
TABLES = tuple(
    TABLE_ROOT / name
    for name in (
        "x15_static_6axis.tbl",
        "x15_symmetric_stabilator_6axis.tbl",
        "x15_differential_stabilator_6axis.tbl",
        "x15_rudder_6axis.tbl",
        "x15_xlr99_thrust_mdot.tbl",
    )
)
OUTPUT = ROOT / "artifacts/golden_plants/x15_powered_full_thrust_trim_report.json"
X15 = vehicle_definition("x15")
_CONTROL_LIMITS = {
    "symmetric_stabilator_deg": (-14.89690267, 34.9504255),
    "differential_stabilator_deg": (-20.05352283, 20.05352283),
    "rudder_deg": (-29.79380535, 29.79380535),
}


def _sha256(path: Path) -> str:
    """Return a stable digest for one source artifact."""

    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def _candidate(source: str) -> str:
    """Enable only the supplied full-thrust history and disable hold laws."""

    updated, count = re.subn(r"\*prop thrust=0 mdot=0", "*prop thrust=(x15-thrust) mdot=(x15-mdot)", source, count=1)
    if count != 1:
        raise ValueError("could not enable the X-15 source full-thrust profile")
    updated = updated.replace("rudder-hold-gain-deg-per-deg=1.0", "rudder-hold-gain-deg-per-deg=0.0")
    updated = re.sub(r"\*when time>[^\n]+ stop", "*when time>0.005 stop", updated, count=1)
    return updated
    ####


def _steady_seed(base: RuntimeState) -> RuntimeState:
    """Clear source-release body rates before testing a local equilibrium."""

    values = list(base.values)
    for name in ("wx", "wy", "wz"):
        values[base.value_names.index(name)] = 0.0
    return base.with_values(values)
    ####


def _state_at_offsets(
    base: RuntimeState,
    pitch_offset_deg: float,
    yaw_offset_deg: float,
    speed_scale: float,
) -> RuntimeState:
    """Construct a physical attitude/velocity candidate without a guidance proxy."""

    current = Quaternion(
        float(base.named["qw"]),
        float(base.named["qx"]),
        float(base.named["qy"]),
        float(base.named["qz"]),
    )
    pitch = math.radians(pitch_offset_deg)
    yaw = math.radians(yaw_offset_deg)
    attitude = current.multiply(
        Quaternion(math.cos(pitch / 2.0), 0.0, math.sin(pitch / 2.0), 0.0)
    ).multiply(
        Quaternion(math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
    ).normalized()
    values = list(base.values)
    for name, value in zip(("qw", "qx", "qy", "qz"), (attitude.w, attitude.x, attitude.y, attitude.z), strict=True):
        values[base.value_names.index(name)] = value
    for name in ("vx", "vy", "vz"):
        values[base.value_names.index(name)] *= speed_scale
    return base.with_values(values)
    ####


def _apply_controls(vehicle: RuntimeVehicle, controls: dict[str, float]) -> None:
    """Apply source-coordinate controls in both grammar and table aliases."""

    for name, value in controls.items():
        degree_name = name.replace("_", "-")
        numeric_value = float(value)
        vehicle.control_values[degree_name] = numeric_value
        vehicle.control_values[degree_name.removesuffix("-deg")] = math.radians(numeric_value)
        vehicle.control_values[degree_name.removesuffix("-deg").replace("-", "_")] = math.radians(numeric_value)
    ####


def _residual(
    state_values: dict[str, float],
    controls: dict[str, float],
    vehicle: RuntimeVehicle,
    base: RuntimeState,
) -> dict[str, float]:
    """Evaluate true body force/moment residuals at the frozen source epoch."""

    candidate = _state_at_offsets(
        base,
        float(state_values["pitch_offset_deg"]),
        float(state_values["yaw_offset_deg"]),
        float(state_values["speed_scale"]),
    )
    _apply_controls(vehicle, controls)
    named = {**candidate.named, **vehicle.control_values}
    candidate = RuntimeState(
        candidate.time,
        candidate.values,
        candidate.frame,
        named,
        candidate.value_names,
        candidate.segment_endpoints,
    )
    vehicle.state = candidate
    vehicle.history[0] = candidate
    if vehicle.environment_evaluator is None:
        raise RuntimeError("X-15 candidate has no rigid-body observable evaluator")
    try:
        observed = vehicle.environment_evaluator({**candidate.named, **vehicle.control_values})
    except ValueError as error:
        # The optimizer is not allowed to step through a no-extrapolation
        # table boundary.  Return a deterministic, plainly failed residual so
        # the result records an infeasible source condition rather than
        # treating the exception as a successful surrogate extrapolation.
        if "outside its declared envelope" not in str(error):
            raise
        return {
            "body_x_force_n": 1.0e9,
            "body_y_force_n": 1.0e9,
            "body_z_force_n": 1.0e9,
            "roll_moment_nm": 1.0e9,
            "pitch_moment_nm": 1.0e9,
            "yaw_moment_nm": 1.0e9,
        }
    return {
        "body_x_force_n": float(observed["total_force_body_x_n"]),
        "body_y_force_n": float(observed["total_force_body_y_n"]),
        "body_z_force_n": float(observed["total_force_body_z_n"]),
        "roll_moment_nm": float(observed["total_moment_body_x_nm"]),
        "pitch_moment_nm": float(observed["total_moment_body_y_nm"]),
        "yaw_moment_nm": float(observed["total_moment_body_z_nm"]),
    }
    ####


def build_report() -> dict[str, Any]:
    """Attempt one frozen-time source full-thrust trim and retain its result."""

    with tempfile.TemporaryDirectory(prefix="taoryx-x15-powered-trim-") as directory:
        candidate_path = Path(directory) / "candidate.prb"
        candidate_path.write_text(_candidate(PROBLEM.read_text(encoding="utf-8")), encoding="utf-8")
        program = LoadedProgram.load(candidate_path, TABLES, profile=GrammarProfile.TAORYX)
        vehicle = program.case().vehicles["1"]
        source_release_state = vehicle.state
        base = _steady_seed(source_release_state)
        force_scale = max(float(base.named["mass"]) * 9.80665, 1.0)
        moment_scale = max(force_scale * float(X15["reference_length_m"]), 1.0)
        spec = TrimSpec(
            state_names=("pitch_offset_deg", "yaw_offset_deg", "speed_scale"),
            control_names=tuple(_CONTROL_LIMITS),
            residual_names=(
                "body_x_force_n",
                "body_y_force_n",
                "body_z_force_n",
                "roll_moment_nm",
                "pitch_moment_nm",
                "yaw_moment_nm",
            ),
            state_initial={"pitch_offset_deg": 0.0, "yaw_offset_deg": 0.0, "speed_scale": 1.0},
            control_initial={
                "symmetric_stabilator_deg": 0.0,
                "differential_stabilator_deg": 0.0,
                "rudder_deg": 0.0,
            },
            state_lower={"pitch_offset_deg": -3.0, "yaw_offset_deg": -5.0, "speed_scale": 0.70},
            state_upper={"pitch_offset_deg": 5.0, "yaw_offset_deg": 5.0, "speed_scale": 1.10},
            control_lower={name: bounds[0] for name, bounds in _CONTROL_LIMITS.items()},
            control_upper={name: bounds[1] for name, bounds in _CONTROL_LIMITS.items()},
            residual_scales={
                "body_x_force_n": force_scale,
                "body_y_force_n": force_scale,
                "body_z_force_n": force_scale,
                "roll_moment_nm": moment_scale,
                "pitch_moment_nm": moment_scale,
                "yaw_moment_nm": moment_scale,
            },
        )
        result = solve_trim(
            spec,
            lambda state, controls: _residual(dict(state), dict(controls), vehicle, base),
            max_nfev=500,
            residual_tolerance=1.0e-10,
            acceptance_tolerance=1.0e-3,
        )
    payload = {
        "vehicle": "x15",
        "operating_condition": "frozen source full-thrust table epoch at t=0",
        "status": "pass" if result.success else "blocked",
        "state": dict(result.state),
        "controls": dict(result.controls),
        "residual_normalized": dict(result.residuals),
        "residual_norm_l2": math.sqrt(sum(value * value for value in result.residuals.values())),
        "claim": (
            "A frozen-time source full-thrust trim attempt using actual source stabilator/rudder table coordinates; "
            "not a throttle-controlled or time-persistent equilibrium."
        ),
        "nonclaims": [
            "The XLR99 source table is time-indexed full thrust, not a Mach/altitude/throttle map.",
            "No throttling, engine-spool, RCS, or scheduled powered-flight claim follows from this attempt.",
        ],
        "provenance": {
            "source_problem": str(PROBLEM.relative_to(ROOT)),
            "source_problem_sha256": _sha256(PROBLEM),
            "tables": {str(path.relative_to(ROOT)): _sha256(path) for path in TABLES},
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
    return payload
    ####


def write_report(output: Path = OUTPUT) -> Path:
    """Write the deterministic frozen-time full-thrust diagnostic."""

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(build_report(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return output
    ####


def main() -> None:
    """Attempt and record the frozen-time full-thrust local trim."""

    print(write_report())
    ####


if __name__ == "__main__":
    main()
