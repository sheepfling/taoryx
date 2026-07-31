#!/usr/bin/env python3
"""Render the Alpha 3 passive tumbling-body fidelity evidence board.

The board uses the existing nominal cylinder telemetry as the representative
trajectory and the four-shape comparison record for terminal spread.  It
does not invent a controller: the pseudo tier is native rigid-body reuse and
the 3DOF tier uses an orientation-averaged projected-area policy.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_tumbling_body"
FIDELITY_FILES = {
    "point_mass_3dof": DEFAULT_OUTPUT / "nominal-cylinder-point_mass_3dof.json",
    "pseudo_6dof": DEFAULT_OUTPUT / "nominal-cylinder-pseudo_6dof.json",
}
COMPARISON_FILE = DEFAULT_OUTPUT / "fidelity_ladder/comparison.json"
COLORS = {"point_mass_3dof": "#4c78a8", "pseudo_6dof": "#f58518"}
####


def _load(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected mapping in {path}")
    return payload
    ####


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def _artifact_path(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return path.name
    ####


def _telemetry(payload: dict[str, Any]) -> list[dict[str, Any]]:
    samples = payload.get("samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError("tumbling-body packet has no samples")
    spawned = samples[0].get("spawned_bodies")
    if not isinstance(spawned, list) or not spawned:
        raise ValueError("tumbling-body packet has no spawned child")
    telemetry = spawned[0].get("telemetry")
    if not isinstance(telemetry, list) or not telemetry:
        raise ValueError("tumbling-body packet has no child telemetry")
    return [row for row in telemetry if isinstance(row, dict)]
    ####


def render(output: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    """Render and manifest the passive-body paired-fidelity witness."""

    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    packets = {fidelity: _load(path) for fidelity, path in FIDELITY_FILES.items()}
    rows = {fidelity: _telemetry(payload) for fidelity, payload in packets.items()}
    comparison = _load(COMPARISON_FILE)
    output.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(2, 3, figsize=(16.0, 9.0), constrained_layout=True)

    for fidelity, samples in rows.items():
        color = COLORS[fidelity]
        label = fidelity.replace("_", " ")
        time = [float(row["time_s"]) for row in samples]
        downrange = [float(row["position_m"][0]) for row in samples]
        altitude = [float(row["position_m"][2]) for row in samples]
        speed = [float(row["air_relative_speed_m_s"]) for row in samples]
        angular_rate = [float(row.get("angular_rate_norm_rad_s", 0.0)) for row in samples]
        area = [float(row["projected_area_m2"]) for row in samples]
        drag = [float(row["drag_force_n"]) for row in samples]
        axes[0, 0].plot(downrange, altitude, color=color, label=label)
        axes[0, 1].plot(time, speed, color=color, label=label)
        axes[0, 2].plot(time, angular_rate, color=color, label=label)
        axes[1, 0].plot(time, area, color=color, label=label)
        axes[1, 1].plot(time, drag, color=color, label=label)

    axes[0, 0].set(xlabel="downrange (m)", ylabel="altitude (m)", title="Passive child trajectory: cylinder")
    axes[0, 1].set(xlabel="time (s)", ylabel="air-relative speed (m/s)", title="Speed")
    axes[0, 2].set(xlabel="time (s)", ylabel="angular-rate norm (rad/s)", title="Rotational state")
    axes[1, 0].set(xlabel="time (s)", ylabel="projected area (m²)", title="Area policy")
    axes[1, 1].set(xlabel="time (s)", ylabel="drag force (N)", title="Aerodynamic drag")
    for axis in axes.flat[:5]:
        axis.grid(alpha=0.25)
        axis.legend(fontsize=7, loc="best")

    shapes = [str(record["shape"]) for record in comparison["records"]]
    deltas = [float(record["terminal_delta"]["downrange_m"]) for record in comparison["records"]]
    axes[1, 2].bar(shapes, deltas, color="#72b7b2")
    axes[1, 2].axhline(0.0, color="black", linewidth=0.8)
    axes[1, 2].set(xlabel="shape", ylabel="pseudo − 3DOF (m)", title="Terminal downrange difference")
    axes[1, 2].tick_params(axis="x", rotation=25)
    axes[1, 2].grid(axis="y", alpha=0.25)

    figure.suptitle(
        "Passive tumbling body | averaged-area 3DOF versus native rigid-body pseudo-6DOF",
        fontsize=15,
        fontweight="bold",
    )
    figure.text(
        0.01,
        0.01,
        "No controller or physical effector is claimed. 3DOF uses orientation-averaged area; pseudo reuses native rigid-body rotation.",
        fontsize=8,
    )
    board = output / "tumbling_body_fidelity_evidence_board.png"
    figure.savefig(board, dpi=160, bbox_inches="tight")
    plt.close(figure)

    manifest = {
        "schema": "taoryx.tumbling-body-fidelity-evidence-board/v1alpha1",
        "status": "verified",
        "family_id": "tumbling_body",
        "representative_shape": "cylinder",
        "fidelity_records": [
            {
                "fidelity": fidelity,
                "artifact": _artifact_path(path),
                "sha256": _sha256(path),
                "classification": packets[fidelity]["samples"][0]["spawned_bodies"][0]["classification"],
            }
            for fidelity, path in FIDELITY_FILES.items()
        ],
        "comparison_artifact": {"path": _artifact_path(COMPARISON_FILE), "sha256": _sha256(COMPARISON_FILE)},
        "board": {"path": _artifact_path(board), "sha256": _sha256(board)},
        "claim_boundary": {
            "proves": "The declared passive cylinder deployment produces an impact witness and exposes the difference between orientation-averaged translation and native rigid-body rotation.",
            "nonclaims": [
                "No controller, actuator, guidance, or control-authority claim.",
                "No source-exact child aerodynamics or impact certification.",
                "No parent capability or controller-effective reachability claim.",
            ],
        },
        "reproduction": "PYTHONPATH=src python3 tools/render_tumbling_body_fidelity_board.py",
    }
    (output / "tumbling_body_fidelity_evidence_board_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return manifest
    ####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    print(json.dumps(render(arguments.output), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
