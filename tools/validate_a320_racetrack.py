"""Run and truth-evaluate the A320 shared powered-fixed-wing racetrack.

This tool is the executable mission stage for the A320 integration pilots.  It
does not promote the two lanes to aircraft qualification: the point-mass run
is performance-backed kinematic evidence, while the pseudo-6DOF run adds the
named rotational response bridge and policy control overlay.
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

from taoryx.mission_objectives import TruthObjectiveSpec, evaluate_truth_objectives  # noqa: E402
from taoryx.racetrack_template import ResolvedRacetrack, load_racetrack_template_catalog  # noqa: E402
from taoryx.trajectory import (  # noqa: E402
    A320OpenAPModel,
    A320OpenAPOperatingPoint,
    A320Pseudo6DOFModel,
    A320RacetrackMode,
    A320RacetrackRun,
    A320RacetrackRunner,
    load_pseudo6dof_catalog,
)

CATALOG = ROOT / "verification/racetrack_templates.yaml"


def _objective_specs(route: ResolvedRacetrack) -> tuple[TruthObjectiveSpec, ...]:
    """Resolve oriented truth gates from the shared route binding."""

    windows = {window.name: window for window in route.phase_windows}
    specs: list[TruthObjectiveSpec] = []
    for gate in route.gates:
        window = windows[gate.phase]
        specs.append(
            TruthObjectiveSpec(
                id=gate.id,
                objective_type="fly_by_gate",
                target=gate.target(route.speed_m_s),
                tolerance={
                    "corridor_m": route.gate_corridor_m,
                    "altitude_m": route.gate_altitude_tolerance_m,
                    "speed_m_s": route.gate_speed_tolerance_mps,
                },
                gate_normal=cast(tuple[float, float, float], tuple(gate.gate_normal())),
                crossing_direction=1,
                window_start_s=max(0.0, window.start_s - 12.0),
                window_end_s=route.horizon_s if gate.id == "terminal-start-finish-gate" else window.end_s + 12.0,
            )
        )
    return tuple(specs)
    ####


def _finite_rows(rows: list[dict[str, float | int | str]]) -> bool:
    for row in rows:
        for value in row.values():
            if isinstance(value, (int, float)) and not math.isfinite(float(value)):
                return False
    return True
    ####


def _build_run(
    mode: A320RacetrackMode,
    dt_s: float,
    operating_point: A320OpenAPOperatingPoint | None = None,
) -> tuple[ResolvedRacetrack, A320RacetrackRun, dict[str, Any]]:
    catalog = load_racetrack_template_catalog(CATALOG)
    binding_id = "a320-openap-3dof" if mode == "point_mass_3dof" else "a320-openap-pseudo6dof"
    route = catalog.get(binding_id)
    operating_point = operating_point or A320OpenAPOperatingPoint(10500.0, 0.78, 60000.0)
    model: A320OpenAPModel | A320Pseudo6DOFModel
    if mode == "point_mass_3dof":
        model = A320OpenAPModel.from_repository(ROOT)
        trim = model.trim_level_flight(operating_point)
    else:
        model = A320Pseudo6DOFModel.from_repository(ROOT)
        trim = model.trim_pseudo6dof(operating_point)
    response_profile = None
    if mode == "pseudo_6dof_kinematic_bridge":
        _, response_profile = load_pseudo6dof_catalog(ROOT / "verification/pseudo6dof_profiles.yaml").for_family("a320_openap_3dof")
    result = A320RacetrackRunner(model, trim, route, mode, dt_s=dt_s, response_profile=response_profile).run()
    return route, result, {
        "binding_id": binding_id,
        "trim": trim.as_dict(),
        "model_provenance": model.provenance,
    }
    ####


def run_case(
    mode: A320RacetrackMode,
    dt_s: float,
    operating_point: A320OpenAPOperatingPoint | None = None,
) -> tuple[dict[str, Any], list[dict[str, float | int | str]]]:
    """Execute one A320 lane and evaluate it only from truth telemetry."""

    route, result, metadata = _build_run(mode, dt_s, operating_point)
    rows = [dict(row) for row in result.rows]
    hard_gates_passed = result.numerical_valid and _finite_rows(rows)
    evaluation = evaluate_truth_objectives(_objective_specs(route), rows, hard_gates_passed=hard_gates_passed)
    claim_boundary = (
        "Exact OpenAP point-mass performance channels plus a bounded kinematic navigation realization; "
        "no physical attitude, moment, surface, actuator, or manufacturer-aircraft claim."
        if mode == "point_mass_3dof"
        else "OpenAP performance plus the declared JSBSim rotational surrogate and Taoryx kinematic attitude-response law; "
        "no source-exact actuator or manufacturer flight-dynamics claim."
    )
    packet: dict[str, Any] = {
        "schema_version": "taoryx.a320-racetrack-evidence/v1",
        "status": "nominal_case_pass" if evaluation["mission_pass"] else "development_screen_pending",
        "family_id": "a320_openap_3dof" if mode == "point_mass_3dof" else "a320_openap_jsbsim_pseudo6dof",
        "vehicle_id": "a320_openap",
        "binding_id": metadata["binding_id"],
        "fidelity": mode,
        "claim_boundary": claim_boundary,
        "route": {
            "template_id": route.template_id,
            "source_realization": route.source_realization,
            "timing": route.timing_manifest(),
            "gates": [
                {
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
        "trim": metadata["trim"],
        "operating_point": {
            "altitude_m": (operating_point.altitude_m if operating_point is not None else 10500.0),
            "mach": operating_point.mach if operating_point is not None else 0.78,
            "mass_kg": operating_point.mass_kg if operating_point is not None else 60000.0,
        },
        "model_provenance": metadata["model_provenance"],
        "runtime": {
            "dt_s": dt_s,
            "duration_s": float(rows[-1]["time_s"]) if rows else 0.0,
            "numerical_valid": result.numerical_valid,
            "failure": result.failure,
            "hard_gates_passed": hard_gates_passed,
        },
        "evaluation": evaluation,
        "nonclaims": [
            "manufacturer validation",
            "source-exact aircraft control law",
            "physical actuator qualification",
            "full rigid-body 6DOF equivalence",
        ],
    }
    return packet, rows
    ####


def _write_csv(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    if not rows:
        path.write_text("\n", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    ####


def _render_board(path: Path, rows: list[dict[str, float | int | str]], packet: Mapping[str, Any]) -> None:
    """Render a compact evidence board from the same telemetry and report."""

    route = packet["route"]
    evaluation = packet["evaluation"]
    times = [float(row["time_s"]) for row in rows]
    figure, axes = plt.subplots(2, 2, figsize=(14, 8), layout="constrained")
    figure.suptitle(
        f"A320 shared racetrack — {packet['fidelity']} — {packet['status']}",
        fontsize=14,
        fontweight="bold",
        x=0.02,
        ha="left",
    )
    axis = axes[0, 0]
    axis.plot([float(row["east_m"]) for row in rows], [float(row["north_m"]) for row in rows], color="#2563eb", label="truth path")
    axis.scatter([0.0], [0.0], color="#16a34a", marker="o", label="start/finish")
    for gate in route["gates"]:
        axis.scatter([float(gate["east_m"])], [float(gate["north_m"])], marker="x", color="#dc2626")
        axis.text(float(gate["east_m"]), float(gate["north_m"]), str(gate["id"]), fontsize=7)
    axis.set_title("Complete route and truth gates", loc="left", fontweight="bold")
    axis.set_xlabel("east (m)")
    axis.set_ylabel("north (m)")
    axis.axis("equal")
    axis.grid(True, color="#cbd5e1", linewidth=0.7)
    axis.legend(fontsize="small")

    axis = axes[0, 1]
    altitude = [float(row["altitude_m"]) for row in rows]
    speed_axis = axis.twinx()
    axis.plot(times, altitude, color="#2563eb", label="altitude truth")
    axis.plot(times, [float(row["route_altitude_command_m"]) for row in rows], color="#2563eb", linestyle=":", label="altitude command")
    speed_axis.plot(times, [float(row["speed_m_s"]) for row in rows], color="#7c3aed", label="speed truth")
    speed_axis.plot(times, [float(row["route_speed_command_m_s"]) for row in rows], color="#7c3aed", linestyle=":", label="speed command")
    axis.set_title("Altitude and speed truth versus command", loc="left", fontweight="bold")
    axis.set_xlabel("time (s)")
    axis.set_ylabel("altitude (m)")
    speed_axis.set_ylabel("speed (m/s)")
    axis.grid(True, axis="y", color="#cbd5e1", linewidth=0.7)
    lines, labels = axis.get_legend_handles_labels()
    lines2, labels2 = speed_axis.get_legend_handles_labels()
    axis.legend(lines + lines2, labels + labels2, fontsize="small", loc="best")

    axis = axes[1, 0]
    axis.plot(times, [float(row["route_bank_command_deg"]) for row in rows], color="#0f766e", linestyle=":", label="bank command")
    axis.plot(times, [float(row["route_bank_achieved_deg"]) for row in rows], color="#0f766e", label="bank achieved")
    axis.plot(times, [float(row["route_heading_command_deg"]) for row in rows], color="#ea580c", linestyle=":", label="heading command")
    axis.plot(times, [float(row["route_heading_achieved_deg"]) for row in rows], color="#ea580c", label="heading achieved")
    axis.set_title("Attitude/response channels", loc="left", fontweight="bold")
    axis.set_xlabel("time (s)")
    axis.set_ylabel("degrees")
    axis.grid(True, color="#cbd5e1", linewidth=0.7)
    axis.legend(fontsize="small", ncol=2)

    axis = axes[1, 1]
    results = evaluation["results"]
    labels = [str(item["id"]).replace("-", "\n") for item in results]
    values = [1.0 if item["status"] == "pass" else 0.0 for item in results]
    axis.bar(range(len(labels)), values, color=["#16a34a" if value else "#dc2626" for value in values])
    axis.set_ylim(0.0, 1.2)
    axis.set_xticks(range(len(labels)), labels, fontsize=7)
    axis.set_yticks([0.0, 1.0], ["FAIL", "PASS"])
    axis.set_title("Independent truth-gate results", loc="left", fontweight="bold")
    axis.grid(True, axis="y", color="#cbd5e1", linewidth=0.7)
    figure.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(figure)
    ####


def main(argv: list[str] | None = None) -> int:
    """Run one or both A320 integration mission lanes."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("all", "point_mass_3dof", "pseudo_6dof_kinematic_bridge"), default="all")
    parser.add_argument("--dt-s", type=float, default=0.2)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "verification/a320_racetrack")
    args = parser.parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    modes: tuple[A320RacetrackMode, ...] = ("point_mass_3dof", "pseudo_6dof_kinematic_bridge") if args.mode == "all" else (cast(A320RacetrackMode, args.mode),)
    packets: list[dict[str, Any]] = []
    all_passed = True
    for mode in modes:
        packet, rows = run_case(mode, args.dt_s)
        all_passed = all_passed and bool(packet["evaluation"]["mission_pass"])
        stem = mode
        (args.output_dir / f"{stem}_evidence.json").write_text(json.dumps(packet, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        _write_csv(args.output_dir / f"{stem}_telemetry.csv", rows)
        _render_board(args.output_dir / f"{stem}_board.png", rows, packet)
        packets.append(packet)
        print(json.dumps(packet, indent=2, sort_keys=True))
    (args.output_dir / "summary.json").write_text(json.dumps({"schema_version": "taoryx.a320-racetrack-summary/v1", "mission_pass": all_passed, "packets": packets}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if all_passed else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
