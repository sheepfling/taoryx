#!/usr/bin/env python3
"""Render the scoped Alpha 3 X-15 fidelity evidence board.

The board is generated only from the checked-in 3DOF and pseudo-6DOF evidence
packets.  It is a visual evidence product for the staged event/energy/impact
witness, not a controlled terminal-handoff or physical-effector claim.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/x15_fidelity_ladder"
FIDELITY_FILES = {
    "point_mass_3dof": DEFAULT_OUTPUT / "point_mass_3dof_evidence.json",
    "pseudo_6dof": DEFAULT_OUTPUT / "pseudo_6dof_evidence.json",
}
COLORS = {"point_mass_3dof": "#4c78a8", "pseudo_6dof": "#f58518"}
####


def _load(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected mapping in {path}")
    return payload
    ####


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()
    ####


def _artifact_path(path: Path) -> str:
    """Return a stable manifest path for repository or isolated output."""

    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return path.name
    ####


def _event_rows(payload: dict[str, Any]) -> list[dict[str, Any]]:
    rows = payload.get("telemetry")
    if not isinstance(rows, list):
        raise ValueError("X-15 evidence packet has no telemetry list")
    return [row for row in rows if isinstance(row, dict)]
    ####


def render(output: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    """Render and manifest the current X-15 staged witness."""

    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    packets = {fidelity: _load(path) for fidelity, path in FIDELITY_FILES.items()}
    rows = {fidelity: _event_rows(payload) for fidelity, payload in packets.items()}
    output.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(2, 3, figsize=(16.0, 9.0), constrained_layout=True)

    for fidelity, samples in rows.items():
        color = COLORS[fidelity]
        label = fidelity.replace("_", " ")
        time = [float(row["time_s"]) for row in samples]
        altitude = [float(row["position_m"][2]) / 1_000.0 for row in samples]
        downrange = [float(row["position_m"][0]) / 1_000.0 for row in samples]
        speed = [float(row["air_relative_speed_m_s"]) for row in samples]
        energy = [float(row["specific_energy_j_per_kg"]) / 1_000.0 for row in samples]
        bank = [float(row.get("mission_bank_command_deg", 0.0)) for row in samples]
        axes[0, 0].plot(downrange, altitude, color=color, label=label)
        axes[0, 1].plot(time, speed, color=color, label=label)
        axes[0, 2].plot(time, energy, color=color, label=label)
        axes[1, 0].plot(time, bank, color=color, label=label)

        handoff = next(
            (
                item
                for item in packets[fidelity]["mission"]["required_objectives"]
                if item.get("id") == "atmospheric_terminal_handoff"
            ),
            None,
        )
        actual = handoff.get("actual") if isinstance(handoff, dict) else None
        handoff_time = handoff.get("truth_time_s") if isinstance(handoff, dict) else None
        if isinstance(actual, dict) and isinstance(handoff_time, (int, float)):
            handoff_row = next(
                row for row in samples if abs(float(row["time_s"]) - float(handoff_time)) < 1.0e-9
            )
            axes[0, 0].scatter(
                float(handoff_row["position_m"][0]) / 1_000.0,
                float(actual["altitude_m"]) / 1_000.0,
                color=color,
                marker="D",
                s=34,
                label=f"{label}: handoff",
            )
            axes[0, 1].axvline(float(handoff_time), color=color, linestyle=":", alpha=0.45)

        phase_indices: dict[str, list[float]] = {}
        for row in samples:
            phase = str(row.get("phase", "unknown"))
            phase_indices.setdefault(phase, []).append(float(row["time_s"]))
        for phase, phase_times in phase_indices.items():
            axes[1, 1].plot(
                [min(phase_times), max(phase_times)],
                [label, label],
                color=color,
                linewidth=8,
                solid_capstyle="butt",
                label=f"{label}: {phase}",
            )

    axes[0, 0].set(xlabel="downrange (km)", ylabel="altitude (km)", title="Staged trajectory")
    axes[0, 1].set(xlabel="time (s)", ylabel="air-relative speed (m/s)", title="Energy state: speed")
    axes[0, 2].set(xlabel="time (s)", ylabel="specific energy (kJ/kg)", title="Energy state: specific energy")
    axes[1, 0].set(xlabel="time (s)", ylabel="bank command (deg)", title="Declared mission command")
    axes[1, 1].set(xlabel="time (s)", title="Active phase")
    axes[1, 1].set_yticks(["point mass 3dof", "pseudo 6dof"])
    axes[1, 2].axis("off")
    for axis in axes.flat[:5]:
        axis.grid(alpha=0.25)
        axis.legend(fontsize=7, loc="best")

    reference = packets["point_mass_3dof"]
    claim = reference["claim"]
    evaluation = reference["evaluation"]
    nonclaims = claim.get("nonclaims", []) if isinstance(claim, dict) else []
    text = [
        "ALPHA 3 X-15 STAGED WITNESS",
        "Nominal paired 3DOF / pseudo-6DOF evidence",
        f"Mission pass: {evaluation.get('mission_pass')}",
        "",
        "Proves:",
        str(claim.get("proves", "")) if isinstance(claim, dict) else "",
        "",
        "Nonclaims:",
        *[f"• {item}" for item in nonclaims],
    ]
    axes[1, 2].text(0.0, 1.0, "\n".join(text), va="top", ha="left", fontsize=8, wrap=True)
    figure.suptitle("X-15 | staged boost, coast, glide, terminal handoff, and impact evidence", fontsize=15, fontweight="bold")
    board = output / "x15_fidelity_evidence_board.png"
    figure.savefig(board, dpi=160, bbox_inches="tight")
    plt.close(figure)

    manifest = {
        "schema": "taoryx.x15-fidelity-evidence-board/v1alpha1",
        "status": "verified",
        "family_id": "x15",
        "fidelity_records": [
            {
                "fidelity": fidelity,
                "artifact": str(path.relative_to(ROOT)),
                "sha256": _sha256(path),
                "mission_pass": packets[fidelity].get("evaluation", {}).get("mission_pass"),
            }
            for fidelity, path in FIDELITY_FILES.items()
        ],
        "board": {"path": _artifact_path(board), "sha256": _sha256(board)},
        "claim_boundary": claim,
        "reproduction": "PYTHONPATH=src python3 tools/render_x15_fidelity_board.py",
    }
    (output / "x15_fidelity_evidence_board_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return manifest
    ####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    print(json.dumps(render(args.output), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
