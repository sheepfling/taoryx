"""Run and truth-evaluate the source-grounded F-16 racetrack binding.

This is a development qualification tool.  It deliberately reports direct
wrench and physically allocated surface runs separately; neither path is
promoted to a family qualification claim until the independent objective
gates, actuator evidence, envelope checks, and numerical checks pass.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections.abc import Mapping
from pathlib import Path
from typing import Any, cast

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT))

from taoryx.mission_objectives import TruthObjectiveSpec, evaluate_truth_objectives
from taoryx.racetrack_template import ResolvedRacetrack, load_racetrack_template_catalog
from taoryx.trajectory import (
    F16AttitudeResponsePseudo6DOFModel,
    F16PointMass3DOFModel,
    F16RacetrackMode,
    F16RacetrackRun,
    F16RacetrackRunner,
    F16ReducedRacetrackMode,
    F16ReducedRacetrackRun,
    F16ReducedRacetrackRunner,
)
from tools.validate_f16_physical_wrench_perturbations import _build_case
from tools.validate_f16_reductions import _build_case as _build_reduction_case

CATALOG = ROOT / "verification/racetrack_templates.yaml"
F16_VALIDITY_ENVELOPE = json.loads(
    (ROOT / "families/reference_f16_s119/plant/daveml-import.json").read_text(encoding="utf-8")
)["package"]["validity_envelope"]


def _envelope_violations(rows: list[dict[str, float | int | str]]) -> list[dict[str, object]]:
    """Check telemetry against the source package's declared validity bounds."""

    bounds = {
        "altitude_m": (float(F16_VALIDITY_ENVELOPE["altitude_min_m"]), float(F16_VALIDITY_ENVELOPE["altitude_max_m"])),
        "mach": (float(F16_VALIDITY_ENVELOPE["mach_min"]), float(F16_VALIDITY_ENVELOPE["mach_max"])),
        "alpha_deg": (
            math.degrees(float(F16_VALIDITY_ENVELOPE["alpha_min_rad"])),
            math.degrees(float(F16_VALIDITY_ENVELOPE["alpha_max_rad"])),
        ),
        "beta_deg": (
            math.degrees(float(F16_VALIDITY_ENVELOPE["beta_min_rad"])),
            math.degrees(float(F16_VALIDITY_ENVELOPE["beta_max_rad"])),
        ),
    }
    channel_aliases = {"alpha_deg": "aero_alpha_deg", "beta_deg": "aero_beta_deg"}
    violations: list[dict[str, object]] = []
    for index, row in enumerate(rows):
        for channel, (lower, upper) in bounds.items():
            telemetry_channel = channel_aliases.get(channel, channel)
            if channel == "beta_deg" and telemetry_channel not in row:
                telemetry_channel = "aero_sideslip_deg"
            if telemetry_channel not in row:
                continue
            value = float(row[telemetry_channel])
            if not math.isfinite(value) or value < lower or value > upper:
                violations.append({
                    "sample_index": index,
                    "time_s": float(row.get("time_s", index)),
                    "channel": telemetry_channel,
                    "value": value,
                    "lower": lower,
                    "upper": upper,
                })
    return violations
    ####


def _objective_specs(route: ResolvedRacetrack) -> tuple[TruthObjectiveSpec, ...]:
    """Build ordered gates from the resolved route, not controller events."""

    gates = route.gates
    windows = {window.name: window for window in route.phase_windows}
    specs: list[TruthObjectiveSpec] = []
    for gate in gates:
        window = windows[gate.phase]
        target = gate.target(route.speed_m_s)
        specs.append(
            TruthObjectiveSpec(
                id=gate.id,
                objective_type="fly_by_gate",
                target=target,
                tolerance={
                    "corridor_m": route.gate_corridor_m,
                    "altitude_m": route.gate_altitude_tolerance_m,
                    "speed_m_s": route.gate_speed_tolerance_mps,
                },
                gate_normal=cast(tuple[float, float, float], tuple(gate.gate_normal())),
                crossing_direction=1,
                window_start_s=max(0.0, window.start_s - 12.0),
                window_end_s=(
                    route.horizon_s
                    if gate.id == "terminal-start-finish-gate"
                    else window.end_s + 12.0
                ),
            )
        )
    return tuple(specs)
    ####


def _finite_rows(rows: list[dict[str, float | int | str]]) -> bool:
    """Return whether all numeric telemetry fields remain finite."""

    for row in rows:
        for value in row.values():
            if isinstance(value, (int, float)) and not math.isfinite(float(value)):
                return False
    return True
    ####


def _render_board(
    output: Path,
    rows: list[dict[str, float | int | str]],
    evaluation: dict[str, Any],
    route: ResolvedRacetrack,
    mode: str,
) -> None:
    """Render a compact development board from the same telemetry/evaluation."""

    times = [float(row["time_s"]) for row in rows]
    north = [float(row["north_m"]) for row in rows]
    east = [float(row["east_m"]) for row in rows]
    altitude = [float(row["altitude_m"]) for row in rows]
    speed = [float(row["speed_m_s"]) for row in rows]
    bank_command = [float(row["route_bank_command_deg"]) for row in rows]
    bank_actual = [float(row["route_bank_achieved_deg"]) for row in rows]
    reduced = mode in {"point_mass_3dof", "pseudo_6dof_kinematic_bridge"}
    requested_force = [
        float(row["requested_force_x_n"]) if "requested_force_x_n" in row else float(row["source_force_x_n"])
        for row in rows
    ]
    achieved_force = [
        float(row["achieved_force_x_n"]) if "achieved_force_x_n" in row else float(row["source_force_x_n"])
        for row in rows
    ]
    figure, axes = plt.subplots(2, 2, figsize=(16, 10), constrained_layout=True)
    axes[0, 0].plot(east, north, color="#1d4ed8", linewidth=1.5, label="truth path")
    for gate in route.gates:
        axes[0, 0].scatter(gate.east_m, gate.north_m, s=45, label=gate.id)
        axes[0, 0].annotate(gate.id, (gate.east_m, gate.north_m), fontsize=7)
    axes[0, 0].set_title("F-16 racetrack geometry")
    axes[0, 0].set_xlabel("east (m)")
    axes[0, 0].set_ylabel("north (m)")
    axes[0, 0].axis("equal")
    axes[0, 0].grid(alpha=0.25)

    axes[0, 1].plot(times, altitude, label="truth altitude")
    axes[0, 1].plot(
        times,
        [float(row["route_altitude_command_m"]) for row in rows],
        linestyle=":",
        label="route altitude",
    )
    axes[0, 1].plot(times, speed, label="truth speed")
    axes[0, 1].plot(
        times,
        [float(row["route_speed_command_m_s"]) for row in rows],
        linestyle=":",
        label="route speed",
    )
    axes[0, 1].set_title("Altitude and speed")
    axes[0, 1].set_xlabel("time (s)")
    axes[0, 1].grid(alpha=0.25)
    axes[0, 1].legend(fontsize=8)

    axes[1, 0].plot(times, bank_command, linestyle=":", label="bank command")
    axes[1, 0].plot(times, bank_actual, label="bank truth")
    axes[1, 0].plot(
        times,
        [float(row["route_pitch_achieved_deg"]) for row in rows],
        label="pitch truth",
    )
    axes[1, 0].set_title("Attitude response")
    axes[1, 0].set_xlabel("time (s)")
    axes[1, 0].set_ylabel("degrees")
    axes[1, 0].grid(alpha=0.25)
    axes[1, 0].legend(fontsize=8)

    axes[1, 1].plot(times, requested_force, linestyle=":", label="force requested")
    axes[1, 1].plot(times, achieved_force, label="force achieved")
    axes[1, 1].plot(
        times,
        [float(row["allocation_residual_norm"]) for row in rows],
        label="allocation residual norm",
    )
    axes[1, 1].set_title("Source-force diagnostics" if reduced else "Control-path evidence")
    axes[1, 1].set_xlabel("time (s)")
    axes[1, 1].grid(alpha=0.25)
    axes[1, 1].legend(fontsize=8)

    status = "PASS" if evaluation["mission_pass"] else "PENDING / FAILED GATE"
    figure.suptitle(f"reference F-16 S-119 — {mode} — {status}", fontsize=16, fontweight="bold")
    figure.text(
        0.01,
        0.005,
        (
            "Independent truth evaluator; reduced modes use a named response law and source-force diagnostics. "
            "They do not establish physical effector realization."
            if reduced
            else "Independent truth evaluator; direct-wrench path is comparison-only. "
            "A plotted command does not establish physical effector realization."
        ),
        fontsize=8,
        color="#334155",
    )
    figure.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(figure)
    ####


def run_case(
    mode: F16RacetrackMode | F16ReducedRacetrackMode,
    duration_s: float | None,
    dt_s: float,
    perturbation: Mapping[str, float] | None = None,
) -> tuple[dict[str, Any], list[dict[str, float | int | str]]]:
    """Execute one F-16 route realization and evaluate its truth gates."""

    result: F16RacetrackRun | F16ReducedRacetrackRun
    if mode in {"point_mass_3dof", "pseudo_6dof_kinematic_bridge"}:
        source, trim, trim_pitch_rad = _build_reduction_case()
        catalog = load_racetrack_template_catalog(CATALOG)
        binding_id = {
            "point_mass_3dof": "f16-s119-point-mass",
            "pseudo_6dof_kinematic_bridge": "f16-s119-pseudo-6dof",
        }[mode]
        route = catalog.get(binding_id)
        reduced_mode = mode
        model: F16PointMass3DOFModel | F16AttitudeResponsePseudo6DOFModel
        if reduced_mode == "point_mass_3dof":
            model = F16PointMass3DOFModel(source, trim, trim_pitch_rad)
        else:
            linearization = source.linearize_local(
                trim.state,
                trim.controls,
                trim_pitch_rad=trim_pitch_rad,
                altitude_m=0.0,
                state_step=1.0e-5,
                control_step=1.0e-5,
            )
            model = F16AttitudeResponsePseudo6DOFModel(source, trim, linearization, trim_pitch_rad)
        result = F16ReducedRacetrackRunner(
            model,
            trim,
            route,
            reduced_mode,
            dt_s=dt_s,
        ).run(duration_s=duration_s)
        rows = [dict(row) for row in result.rows]
        allocations_ok = True
        claim_boundary = (
            "The reduced route preserves shared geometry and truth objectives. "
            "Point-mass mode uses bounded translational kinematics; pseudo-6DOF mode adds a named "
            "attitude/rate response bridge. Source forces are diagnostic observables only; neither mode "
            "claims physical moments, control-surface allocation, or actuator realization."
        )
    else:
        adapter, trim, design = _build_case()
        catalog = load_racetrack_template_catalog(CATALOG)
        binding_id = "f16-s119-surfaces" if mode == "surface_allocated" else "f16-s119-direct-wrench"
        route = catalog.get(binding_id)
        runner = F16RacetrackRunner(
            adapter.source,
            trim,
            design,
            route,
            cast(F16RacetrackMode, mode),
            adapter if mode == "surface_allocated" else None,
            dt_s=dt_s,
            initial_north_offset_m=float((perturbation or {}).get("initial_north_offset_m", 0.0)),
            initial_east_offset_m=float((perturbation or {}).get("initial_east_offset_m", 0.0)),
            initial_altitude_offset_m=float((perturbation or {}).get("initial_altitude_offset_m", 0.0)),
            initial_speed_offset_m_s=float((perturbation or {}).get("initial_speed_offset_m_s", 0.0)),
            initial_bank_offset_rad=float((perturbation or {}).get("initial_bank_offset_rad", 0.0)),
        )
        result = runner.run(duration_s=duration_s)
        rows = [dict(row) for row in result.rows]
        allocations_ok = mode == "direct_wrench" or all(
            str(row["allocation_status"]) in {"feasible", "feasible_near_limit"} and int(row["saturation_count"]) == 0
            for row in rows
        )
        claim_boundary = (
            "Source-grounded local F-16 body-load execution through the shared racetrack reference. "
            "The direct-wrench run is a labelled comparison screen. The surface run is physically accountable "
            "only where allocation status, actuator limits, truth objectives, and envelope checks pass."
        )
    envelope_violations = _envelope_violations(rows)
    hard_gates_passed = result.numerical_valid and _finite_rows(rows) and allocations_ok and not envelope_violations
    evaluation = evaluate_truth_objectives(
        _objective_specs(route),
        rows,
        hard_gates_passed=hard_gates_passed,
    )
    packet = {
        "schema_version": "taoryx.f16-racetrack-evidence/v1",
        "status": "nominal_case_pass" if evaluation["mission_pass"] else "development_screen_pending",
        "family_id": "reference_f16_s119",
        "vehicle_id": "reference_f16_s119",
        "binding_id": binding_id,
        "fidelity": route.fidelity,
        "mode": mode,
        "perturbation": dict(perturbation or {}),
        "route": {
            "template_id": route.template_id,
            "source_realization": route.source_realization,
            "timing": route.timing_manifest(),
            "gates": [
                gate.__dict__
                if hasattr(gate, "__dict__")
                else {
                    "id": gate.id,
                    "north_m": gate.north_m,
                    "east_m": gate.east_m,
                    "altitude_m": gate.altitude_m,
                    "gate_normal_north": gate.gate_normal_north,
                    "gate_normal_east": gate.gate_normal_east,
                    "phase": gate.phase,
                }
                for gate in route.gates
            ],
        },
        "runtime": {
            "dt_s": dt_s,
            "duration_s": float(rows[-1]["time_s"]) if rows else 0.0,
            "numerical_valid": result.numerical_valid,
            "failure": result.failure,
            "hard_gates_passed": hard_gates_passed,
            "envelope_violations": envelope_violations,
        },
        "evaluation": evaluation,
        "claim_boundary": claim_boundary,
    }
    return packet, rows
    ####


def _write_csv(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    """Write flat telemetry with a stable column order."""

    if not rows:
        path.write_text("\n", encoding="utf-8")
        return
    fields = list(rows[0])
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    ####


def main() -> int:
    """Run one requested mode and write reproducible evidence artifacts."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=("point_mass_3dof", "pseudo_6dof_kinematic_bridge", "direct_wrench", "surface_allocated"),
        default="surface_allocated",
    )
    parser.add_argument("--duration-s", type=float, default=None)
    parser.add_argument("--dt-s", type=float, default=0.2)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "verification/f16_racetrack")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    packet, rows = run_case(args.mode, args.duration_s, args.dt_s)
    stem = f"{args.mode}"
    (args.output_dir / f"{stem}_evidence.json").write_text(json.dumps(packet, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _write_csv(args.output_dir / f"{stem}_telemetry.csv", rows)
    route = load_racetrack_template_catalog(CATALOG).get(str(packet["binding_id"]))
    _render_board(args.output_dir / f"{stem}_board.png", rows, packet["evaluation"], route, args.mode)
    print(json.dumps(packet, indent=2, sort_keys=True))
    return 0 if packet["evaluation"]["mission_pass"] else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
