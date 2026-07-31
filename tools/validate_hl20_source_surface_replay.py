#!/usr/bin/env python3
"""Validate the HL-20 source-surface open-loop replay seam.

This is a short native rigid-body witness. Explicit seven-surface commands
enter the pinned DAVE-ML load graph; no velocity-alignment or direct control
moment is injected. It is deliberately not a trim, closed-loop, or mission
qualification result.
"""

from __future__ import annotations

import json
from pathlib import Path

from taoryx.hl20_reachability import hl20_source_surface_replay_vehicle
from taoryx.reachability_envelope import LaunchCommand, ReachabilityFidelity, run_reachability_envelope

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_hl20_source_surface_replay"
SURFACE_COMMANDS = (0.0, 0.0, 0.0, 0.0, 30.0, 30.0, 0.0)


def build_replay(output: Path = DEFAULT_OUTPUT) -> dict[str, object]:
    """Run and write the bounded source-surface replay artifact."""

    output.mkdir(parents=True, exist_ok=True)
    command = LaunchCommand(0.0, 0.0, surface_commands_deg=SURFACE_COMMANDS)
    envelope = run_reachability_envelope(
        hl20_source_surface_replay_vehicle(),
        (command,),
        fidelity=ReachabilityFidelity.RIGID_BODY_6DOF_SURFACE_ALLOCATED,
        step_size_s=0.01,
        horizon_s=0.5,
        spawn_children=False,
    )
    payload = envelope.as_dict(include_trajectories=True)
    sample = payload["samples"][0]
    assert isinstance(sample, dict)
    telemetry = sample["telemetry"]
    assert isinstance(telemetry, list)
    source_rows = [row for row in telemetry if isinstance(row, dict) and "source_aerodynamics" in row]
    direct_injection_values = {float(row["direct_body_moment_injection"]) for row in telemetry if "direct_body_moment_injection" in row}
    surface_modes = {str(row["surface_allocation_mode"]) for row in telemetry if "surface_allocation_mode" in row}
    mission_pass = (
        sample["classification"] == "feasible"
        and bool(source_rows)
        and direct_injection_values == {0.0}
        and surface_modes == {"source_open_loop_replay"}
    )
    report: dict[str, object] = {
        "schema": "taoryx.hl20-source-surface-replay/v1alpha1",
        "status": "source_surface_replay_pass_with_open_loop_boundary" if mission_pass else "source_surface_replay_failed",
        "family_id": "hl20_mod_k",
        "fidelity": ReachabilityFidelity.RIGID_BODY_6DOF_SURFACE_ALLOCATED.value,
        "mission_pass": mission_pass,
        "claim": {
            "proves": [
                "explicit seven-surface commands reach the pinned DAVE-ML aerodynamic load graph",
                "the native rigid-body runtime records source forces and moments from those effectors",
                "the replay path injects no synthetic direct control moment",
            ],
            "nonclaims": [
                "trim or equilibrium",
                "closed-loop guidance or stabilization",
                "surface allocation quality beyond the supplied open-loop command",
                "bank, alpha, energy, landing, or family-envelope qualification",
            ],
        },
        "control_path": "explicit surface commands -> source DAVE-ML loads -> native rigid-body integration",
        "surface_commands_deg": list(SURFACE_COMMANDS),
        "source_query_count": len(source_rows),
        "direct_body_moment_injection_values": sorted(direct_injection_values),
        "surface_allocation_modes": sorted(surface_modes),
        "classification": sample["classification"],
        "failure_reasons": sample["failure_reasons"],
        "runtime": {
            "step_size_s": 0.01,
            "horizon_s": 0.5,
            "initial_altitude_m": 5_000.0,
            "initial_speed_m_s": 340.294,
            "source_validity_policy": "fail_closed",
        },
        "source_provenance": payload["provenance"],
        "reproduction": "PYTHONPATH=src MPLCONFIGDIR=/tmp/taoryx-mpl python3 tools/validate_hl20_source_surface_replay.py",
    }
    (output / "source_surface_replay.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest = {
        "schema": "taoryx.hl20-source-surface-replay-manifest/v1alpha1",
        "status": report["status"],
        "artifact": "source_surface_replay.json",
        "reproduction": report["reproduction"],
        "claim_boundary": "Short open-loop source-surface replay only; no trim, controller, or mission qualification.",
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def main() -> int:
    """Run the open-loop source-surface replay."""

    report = build_replay()
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["mission_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
