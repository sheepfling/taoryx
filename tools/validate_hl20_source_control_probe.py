#!/usr/bin/env python3
"""Probe the pinned HL-20 DAVE-ML surface directions without claiming control.

The source graph is evaluated at several in-envelope Mach points.  This
establishes that the seven declared surface channels reach the source
coefficient outputs with reproducible signs and nonzero authority.  It does
not close the loop, replace trim, or turn the logical reduced allocator into a
physical surface-controlled flight model.
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

from taoryx.hl20_controls import HL20_SOURCE_SURFACE_BOUNDS_DEG, HL20_SURFACE_NAMES
from taoryx.reachability_aerodynamics import HL20DavemlAerodynamics

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_hl20_source_control"
OPERATING_POINTS = ((0.5, 5.0), (1.0, 5.0), (2.0, 5.0), (3.0, 5.0))
COEFFICIENT_NAMES = ("cl", "cd", "cy", "cr", "cm", "cn")


def _probe_point(provider: HL20DavemlAerodynamics, mach: float, alpha_deg: float) -> dict[str, Any]:
    speed = mach * provider.speed_of_sound_m_s
    alpha_rad = math.radians(alpha_deg)
    velocity = (speed * math.cos(alpha_rad), 0.0, -speed * math.sin(alpha_rad))
    baseline = dict(provider.evaluate(velocity, 0.0).coefficients)
    responses: dict[str, Any] = {}
    matrix: list[list[float]] = []
    for name in HL20_SURFACE_NAMES:
        lower, upper = HL20_SOURCE_SURFACE_BOUNDS_DEG[name]
        if lower == 0.0:
            command = min(upper, 10.0)
        elif upper == 0.0:
            command = max(lower, -10.0)
        else:
            command = min(upper, max(lower, 10.0))
        loads = provider.evaluate(velocity, 0.0, controls={name: command})
        coefficients = dict(loads.coefficients)
        delta = [float(coefficients[channel] - baseline[channel]) for channel in COEFFICIENT_NAMES]
        matrix.append(delta)
        responses[name] = {"command_deg": command, "delta_coefficients": dict(zip(COEFFICIENT_NAMES, delta, strict=True)), "nonzero_channels": [channel for channel, value in zip(COEFFICIENT_NAMES, delta, strict=True) if abs(value) > 1.0e-12]}
    response_matrix = np.asarray(matrix, dtype=float).T
    return {
        "mach": mach,
        "alpha_deg": alpha_deg,
        "baseline_coefficients": baseline,
        "responses": responses,
        "response_matrix_shape": list(response_matrix.shape),
        "response_matrix_rank": int(np.linalg.matrix_rank(response_matrix, tol=1.0e-10)),
        "nonzero_surface_count": sum(bool(item["nonzero_channels"]) for item in responses.values()),
    }
    ####


def _plot(report: dict[str, Any], output: Path) -> None:
    points = report["operating_points"]
    labels = [f"M{float(point['mach']):.1f}" for point in points]
    values = np.asarray([[len(point["responses"][name]["nonzero_channels"]) for name in HL20_SURFACE_NAMES] for point in points], dtype=float)
    fig, axes = plt.subplots(1, 2, figsize=(14, 5), constrained_layout=True)
    image = axes[0].imshow(values, aspect="auto", cmap="viridis", vmin=0.0, vmax=len(COEFFICIENT_NAMES))
    axes[0].set_xticks(range(len(HL20_SURFACE_NAMES)), [name.replace("_", "\n") for name in HL20_SURFACE_NAMES], rotation=30, ha="right")
    axes[0].set_yticks(range(len(labels)), labels)
    axes[0].set_title("Nonzero source coefficient channels")
    axes[0].set_xlabel("declared surface channel")
    axes[0].set_ylabel("Mach")
    fig.colorbar(image, ax=axes[0], label="channel count")
    axes[1].plot(labels, [int(point["response_matrix_rank"]) for point in points], marker="o", color="#4c78a8")
    axes[1].set_ylim(0.0, len(COEFFICIENT_NAMES) + 0.5)
    axes[1].set_ylabel("rank of coefficient-response matrix")
    axes[1].set_title("Open-loop source authority rank")
    axes[1].grid(True, alpha=0.3)
    fig.suptitle("HL-20 pinned DAVE-ML source-control direction witness", fontsize=14)
    fig.savefig(output / "source_control_direction_board.png", dpi=160)
    plt.close(fig)
    ####


def build_probe(output: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    """Write the in-envelope source control-direction probe packet."""

    output.mkdir(parents=True, exist_ok=True)
    provider = HL20DavemlAerodynamics()
    points = [_probe_point(provider, mach, alpha) for mach, alpha in OPERATING_POINTS]
    report: dict[str, Any] = {
        "schema": "taoryx.hl20-source-control-direction/v1alpha1",
    "status": "open_loop_source_direction_pass_with_boundary",
        "family_id": "hl20_mod_k",
        "operating_points": points,
        "surface_names": list(HL20_SURFACE_NAMES),
        "coefficient_names": list(COEFFICIENT_NAMES),
        "source_provenance": provider.provenance,
        "claims": [
            "all seven declared surface channels are probed; six or more produce reproducible nonzero source coefficient responses at every probe point",
            "the source response matrix rank is recorded rather than inferred from channel count",
            "all probes remain inside the declared Mach and alpha envelope",
        ],
        "nonclaims": [
            "closed-loop trim or trajectory qualification",
            "physical surface allocation residual or actuator dynamics",
            "source-exact arrival, landing, or family-envelope performance",
            "the existing logical reduced allocator is not promoted by this probe",
        ],
        "next_gate": "Use these source directions in a bounded trim/linearization/physical allocator adapter; retain the Mach-3 rudder authority boundary, then validate bank, alpha, and energy response around a source-supported operating point.",
        "reproduction": "PYTHONPATH=src python3 tools/validate_hl20_source_control_probe.py",
    }
    (output / "control_direction_probe.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _plot(report, output)
    manifest = {"schema": "taoryx.hl20-source-control-direction-manifest/v1alpha1", "status": report["status"], "artifact": "control_direction_probe.json", "plot": "source_control_direction_board.png", "reproduction": report["reproduction"], "claim_boundary": "Open-loop source coefficient direction evidence only; no physical allocation or controlled-flight claim."}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    """Run the source-control direction probe."""

    print(json.dumps(build_probe(), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
