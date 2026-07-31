#!/usr/bin/env python3
"""Probe bounded HL-20 source-load allocation without claiming flight control.

The pinned DAVE-ML graph exposes seven surface inputs and six aerodynamic
force/moment outputs. This witness turns those source directions into a local
effectiveness matrix and solves bounded least-squares requests. It is useful
for automatic onboarding and allocator validation, but it is not a trim,
closed-loop, or controlled-trajectory qualification.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from taoryx.control_allocation import EffectorLimits, allocate_and_advance_wrench
from taoryx.hl20_controls import HL20_SOURCE_SURFACE_BOUNDS_DEG, HL20_SURFACE_NAMES, HL20ActuatorProfile
from taoryx.reachability_aerodynamics import HL20DavemlAerodynamics, build_hl20_source_effectiveness

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_hl20_source_allocation"
OPERATING_POINTS = ((0.5, 5.0), (1.0, 5.0), (2.0, 5.0), (3.0, 5.0))
WRENCH_NAMES = ("force_x_n", "force_y_n", "force_z_n", "moment_x_nm", "moment_y_nm", "moment_z_nm")


def _velocity(provider: HL20DavemlAerodynamics, mach: float, alpha_deg: float) -> tuple[float, float, float]:
    speed = mach * provider.speed_of_sound_m_s
    alpha_rad = math.radians(alpha_deg)
    return speed * math.cos(alpha_rad), 0.0, -speed * math.sin(alpha_rad)
    ####


def _wrench(loads: Any) -> np.ndarray:
    return np.asarray((*loads.force_body_n, *loads.moment_body_nm), dtype=float)
    ####


def _probe_point(provider: HL20DavemlAerodynamics, mach: float, alpha_deg: float) -> dict[str, Any]:
    velocity = _velocity(provider, mach, alpha_deg)
    zero_controls = {name: 0.0 for name in HL20_SURFACE_NAMES}
    baseline = _wrench(provider.evaluate(velocity, 0.0, controls=zero_controls))
    effectiveness = build_hl20_source_effectiveness(provider, velocity, 0.0, controls=zero_controls)
    matrix = effectiveness.array
    limits = {
        name: EffectorLimits(
            name=name,
            lower=HL20_SOURCE_SURFACE_BOUNDS_DEG[name][0],
            upper=HL20_SOURCE_SURFACE_BOUNDS_DEG[name][1],
            unit="deg",
            rate_limit_per_s=60.0,
            time_constant_s=0.15,
        )
        for name in HL20_SURFACE_NAMES
    }
    known_commands = np.asarray([-3.0, 3.0, -2.0, 2.0, 4.0, -4.0, 3.0], dtype=float)
    feasible_target = baseline + matrix @ known_commands
    feasible = allocate_and_advance_wrench(
        effectiveness,
        limits,
        dict(zip(WRENCH_NAMES, feasible_target, strict=True)),
        zero_controls,
        1.0,
        preferred_effectors=zero_controls,
        regularization=1.0e-8,
    )
    stress_target = 20.0 * feasible_target
    stress = allocate_and_advance_wrench(
        effectiveness,
        limits,
        dict(zip(WRENCH_NAMES, stress_target, strict=True)),
        zero_controls,
        1.0,
        preferred_effectors=zero_controls,
        regularization=1.0e-8,
    )
    profile = HL20ActuatorProfile(profile_id="hl20.source_allocation_probe_first_order.v1", mode="first_order")
    trace = profile.realize(dict(zip(HL20_SURFACE_NAMES, known_commands, strict=True)), duration_s=0.05)
    achieved_controls = dict(trace.achieved_deg)
    achieved_wrench = _wrench(provider.evaluate(velocity, 0.0, controls=achieved_controls)) - baseline
    static_residual = feasible.allocation.controlled_residual_norm
    realized_residual = feasible.achieved_controlled_residual_norm
    stress_residual = stress.allocation.controlled_residual_norm
    stress_realized_residual = stress.achieved_controlled_residual_norm
    return {
        "mach": mach,
        "alpha_deg": alpha_deg,
        "wrench_names": list(WRENCH_NAMES),
        "surface_names": list(HL20_SURFACE_NAMES),
        "effectiveness_matrix_n_or_nm_per_deg": matrix.tolist(),
        "matrix_rank": int(np.linalg.matrix_rank(matrix, tol=1.0e-10)),
        "singular_values": np.linalg.svd(matrix, compute_uv=False).tolist(),
        "bounds_deg": {name: [float(limits[name].lower), float(limits[name].upper)] for name in HL20_SURFACE_NAMES},
        "feasible_request": {
            "requested_wrench": feasible_target.tolist(),
            "known_command_seed_deg": known_commands.tolist(),
            "allocated_command_deg": dict(feasible.actuator.commanded_positions),
            "residual_norm": static_residual,
            "realized_residual_norm": realized_residual,
            "solver_success": feasible.allocation.status in {"feasible", "feasible_near_limit"},
            "allocation_status": feasible.allocation.status,
            "achieved_command_deg": dict(feasible.actuator.actual_positions),
            "achieved_wrench": dict(feasible.achieved_wrench),
        },
        "infeasible_request": {
            "requested_wrench": stress_target.tolist(),
            "allocated_command_deg": dict(stress.actuator.commanded_positions),
            "residual_norm": stress_residual,
            "realized_residual_norm": stress_realized_residual,
            "solver_success": stress.allocation.status not in {"solver_failure", "numerically_singular"},
            "allocation_status": stress.allocation.status,
            "achieved_command_deg": dict(stress.actuator.actual_positions),
            "achieved_wrench": dict(stress.achieved_wrench),
            "boundary_recorded": stress_residual > max(static_residual * 10.0, 1.0e-8),
        },
        "actuator_probe": {
            "requested_command_deg": dict(trace.requested_deg),
            "achieved_command_deg": achieved_controls,
            "rate_deg_s": dict(trace.rate_deg_s),
            "rate_limited": list(trace.rate_limited),
            "saturated": list(trace.saturated),
            "achieved_source_wrench_delta": achieved_wrench.tolist(),
            "actuated_request_residual_norm": float(np.linalg.norm(achieved_wrench - feasible_target)),
        },
        "allocator": {
            "implementation": "taoryx.control_allocation.allocate_and_advance_wrench",
            "effectiveness_source": effectiveness.source,
            "actuator_model": "bounded_position_rate_first_order",
        },
    }
    ####


def _plot(report: dict[str, Any], output: Path) -> None:
    points = report["operating_points"]
    labels = [f"M{float(point['mach']):.1f}" for point in points]
    ranks = [int(point["matrix_rank"]) for point in points]
    static = [float(point["feasible_request"]["residual_norm"]) for point in points]
    stress = [float(point["infeasible_request"]["residual_norm"]) for point in points]
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), constrained_layout=True)
    axes[0].plot(labels, ranks, marker="o", color="#4c78a8", label="rank")
    axes[0].axhline(6.0, color="#59a14f", linestyle="--", label="six-axis target")
    axes[0].set_ylim(0.0, 7.0)
    axes[0].set_ylabel("effectiveness matrix rank")
    axes[0].set_title("Source-load authority rank")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)
    axes[1].semilogy(labels, [max(value, 1.0e-12) for value in static], marker="o", label="feasible request")
    axes[1].semilogy(labels, [max(value, 1.0e-12) for value in stress], marker="s", label="20x stress request")
    axes[1].set_ylabel("six-axis wrench residual norm")
    axes[1].set_title("Bounded allocation and retained infeasibility")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)
    fig.suptitle("HL-20 pinned DAVE-ML source-load allocation witness", fontsize=14)
    fig.savefig(output / "source_allocation_board.png", dpi=160)
    plt.close(fig)
    ####


def build_probe(output: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    """Write source-load effectiveness and bounded-allocation evidence."""

    output.mkdir(parents=True, exist_ok=True)
    provider = HL20DavemlAerodynamics()
    points = [_probe_point(provider, mach, alpha) for mach, alpha in OPERATING_POINTS]
    feasible_residuals = [float(point["feasible_request"]["residual_norm"]) for point in points]
    stress_boundary_count = sum(bool(point["infeasible_request"]["boundary_recorded"]) for point in points)
    report: dict[str, Any] = {
        "schema": "taoryx.hl20-source-allocation/v1alpha1",
        "status": "open_loop_source_allocation_pass_with_boundary" if stress_boundary_count else "open_loop_source_allocation_pass",
        "family_id": "hl20_mod_k",
        "operating_points": points,
        "source_provenance": provider.provenance,
        "control_path": "source aerodynamic graph -> finite-difference six-axis effectiveness -> bounded least-squares surface allocation -> source-load replay",
        "claims": [
            "the seven declared source surface channels are converted into six-axis local force/moment effectiveness matrices",
            "feasible local wrench requests are solved with explicit surface bounds",
            "an intentionally over-demanded request is retained as an authority boundary rather than silently clipped into a pass",
            "the declared HL-20 first-order actuator profile is exercised for a source-load replay probe",
        ],
        "nonclaims": [
            "closed-loop trim or trajectory qualification",
            "physical force/moment closure beyond the pinned source graph",
            "source-exact guidance or flight-control law",
            "family envelope, landing, thermal, or statistical reliability qualification",
        ],
        "summary": {
            "operating_point_count": len(points),
            "minimum_matrix_rank": min(int(point["matrix_rank"]) for point in points),
            "maximum_feasible_static_residual_norm": max(feasible_residuals),
            "stress_boundary_count": stress_boundary_count,
            "all_feasible_solver_requests_succeeded": all(bool(point["feasible_request"]["solver_success"]) for point in points),
        },
        "next_gate": "Use this source effectivity and allocator in a source-bound pitch/bank/alpha/energy response adapter; retain the Mach-3 rudder boundary and do not call this closed-loop flight evidence.",
        "reproduction": "PYTHONPATH=src python3 tools/validate_hl20_source_allocation_probe.py",
    }
    (output / "source_allocation_probe.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _plot(report, output)
    manifest = {
        "schema": "taoryx.hl20-source-allocation-manifest/v1alpha1",
        "status": report["status"],
        "artifact": "source_allocation_probe.json",
        "plot": "source_allocation_board.png",
        "reproduction": report["reproduction"],
        "claim_boundary": "Open-loop source-load allocation evidence only; no trim or controlled-flight qualification.",
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    """Run the HL-20 source-load allocation probe."""

    print(json.dumps(build_probe(), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
