"""Tests for disposable language-backed capability-scaled route inputs."""

from __future__ import annotations

from pathlib import Path

import yaml

from taoryx.language_backed_racetrack import materialize_language_backed_racetrack
from taoryx.powered_fixed_wing_mission_compiler import (
    compile_powered_fixed_wing_racetrack,
    load_powered_fixed_wing_mission_profiles,
)
from taoryx.racetrack_template import load_racetrack_template_catalog

ROOT = Path(__file__).resolve().parents[2]


def test_x8_candidate_materialization_preserves_baseline_and_retimes_gates(tmp_path: Path) -> None:
    """Generated route inputs live beside the run and leave baseline files unchanged."""

    profiles = load_powered_fixed_wing_mission_profiles(
        ROOT / "verification/powered_fixed_wing_mission_profiles.yaml"
    )
    proposal = compile_powered_fixed_wing_racetrack(
        *profiles["x8-cruise"],
        binding_id="x8-cruise-pseudo_6dof_kinematic_bridge-candidate",
        fidelity="pseudo_6dof_kinematic_bridge",
    )
    baseline = ROOT / "examples/mission_families/slower_x8/SV03_racetrack_altitude_turns_pseudo_6dof.prb"
    baseline_text = baseline.read_text(encoding="utf-8")

    materialized = materialize_language_backed_racetrack(
        proposal,
        "x8-racetrack-altitude-turns-pseudo-6dof-v1",
        tmp_path,
    )

    assert baseline.read_text(encoding="utf-8") == baseline_text
    assert (
        f"racetrack-length-m={proposal.route.route_attributes()['racetrack-length-m']}"
        in materialized.problem.read_text(encoding="utf-8")
    )
    catalog = load_racetrack_template_catalog(materialized.racetrack_config)
    assert catalog.get(proposal.route.binding_id).route_attributes() == proposal.route.route_attributes()
    payload = yaml.safe_load(materialized.mission_config.read_text(encoding="utf-8"))
    mission = next(item for item in payload["missions"] if item["id"].endswith("-candidate"))
    assert mission["racetrack_binding"] == proposal.route.binding_id
    high_gate = next(item for item in mission["objectives"] if item["id"] == "high-altitude-level-gate")
    high_phase = next(item for item in proposal.route.phase_windows if item.name == "outbound-level")
    assert high_gate["window_start_s"] < high_phase.end_s < high_gate["window_end_s"]
    left_bank = next(item for item in mission["objectives"] if item["id"] == "left-turn-bank-response")
    left_phase = next(item for item in proposal.route.phase_windows if item.name == "left-turn")
    assert left_bank["window_start_s"] == left_phase.start_s
    assert left_bank["window_end_s"] == left_phase.end_s
    ####
