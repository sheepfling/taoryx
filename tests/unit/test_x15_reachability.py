from __future__ import annotations

import json
import math
from dataclasses import replace
from pathlib import Path

from taoryx.reachability_envelope import LaunchCommand, ReachabilityFidelity, simulate_rocket_glide
from taoryx.vehicle_registry import vehicle_definition
from taoryx.x15_native_replay import write_x15_native_boundary_replay
from taoryx.x15_reachability import (
    run_x15_reachability_tiers,
    write_x15_reachability_bundle,
    x15_integration_preflight,
    x15_reachability_commands,
    x15_source_staging_contract,
    x15_surrogate_vehicle,
)


def test_x15_preflight_matches_source_staging_contract() -> None:
    source = x15_source_staging_contract()
    report = x15_integration_preflight(x15_surrogate_vehicle())

    assert report.passed
    assert report.declared_propellant_discrepancy_kg == 7_000.0
    assert source["initial_speed_m_s"] == 1_555.6349186151906
    assert dict(report.surrogate)["consumed_booster_propellant_kg"] == 2_000.0


def test_x15_preflight_rejects_a_released_state_disguised_as_staged() -> None:
    vehicle = x15_surrogate_vehicle()
    released_state = replace(vehicle, initial_speed_m_s=1_283.0)

    try:
        x15_integration_preflight(released_state)
    except ValueError as error:
        assert "initial_speed" in str(error)
    else:
        raise AssertionError("X-15 preflight accepted a mismatched initial state")


def test_x15_surrogate_uses_registry_mass_and_geometry() -> None:
    definition = vehicle_definition("x15")
    vehicle = x15_surrogate_vehicle()

    assert vehicle.vehicle_id == "x15-generic-reachability-surrogate-v1"
    assert vehicle.dry_mass_kg == 14_641.0545
    assert vehicle.release_mass_kg == 14_641.0545
    assert vehicle.initial_mass_kg == 30_000.0
    assert vehicle.reference_area_m2 == definition["reference_area_m2"]
    ####


def test_x15_surrogate_exposes_boost_coast_release_and_glide() -> None:
    vehicle = x15_surrogate_vehicle()
    result = simulate_rocket_glide(
        vehicle,
        LaunchCommand(0.0, math.radians(45.0)),
        fidelity=ReachabilityFidelity.POINT_MASS_3DOF,
        step_size_s=5.0,
        horizon_s=60.0,
    )

    phases = {state.phase for state in result.states}
    assert phases == {"boost", "coast", "glide"}
    assert result.states[0].mass_kg == 30_000.0
    coast_state = next(state for state in result.states if state.phase == "coast")
    assert coast_state.mass_kg == 28_000.0
    release_state = next(state for state in result.states if state.phase == "glide")
    assert release_state.mass_kg == vehicle.release_mass_kg
    ####


def test_x15_tiers_share_search_and_record_claim_boundary() -> None:
    envelopes = run_x15_reachability_tiers(commands=x15_reachability_commands()[:6], horizon_s=4.0, step_size_s=0.5)

    assert tuple(envelope.fidelity for envelope in envelopes) == (
        ReachabilityFidelity.POINT_MASS_3DOF,
        ReachabilityFidelity.PSEUDO_6DOF,
    )
    assert all(len(envelope.samples) == 6 for envelope in envelopes)
    assert envelopes[0].search_space == envelopes[1].search_space
    assert envelopes[0].as_dict()["provenance"]["source_vehicle_id"] == "x15"
    assert "not a native X-15 rigid-body batch provider" in envelopes[0].as_dict()["provenance"]["claim_boundary"]
    assert envelopes[0].as_dict()["provenance"]["integration_preflight"]["status"] == "PASS"
    ####


def test_x15_bundle_writes_comparable_artifacts_and_plots(tmp_path: Path) -> None:
    bundle = write_x15_reachability_bundle(
        tmp_path,
        commands=x15_reachability_commands()[:6],
        horizon_s=2.0,
        step_size_s=0.5,
        dpi=90,
    )

    manifest = json.loads(bundle.manifest_path.read_text(encoding="utf-8"))
    assert manifest["vehicle_id"] == "x15"
    assert {path.name for path in bundle.plot_report.plot_paths} == {
        "flown-trajectories.png",
        "search-coverage.png",
        "terminal-capability.png",
        "fidelity-progression.png",
    }
    assert (tmp_path / "point_mass_3dof.json").exists()
    assert (tmp_path / "pseudo_6dof.json").exists()
    assert all(path.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n" for path in bundle.plot_report.plot_paths)
    ####


def test_x15_native_replay_runs_one_safe_boundary_checkpoint(tmp_path: Path) -> None:
    envelope_dir = tmp_path / "envelope"
    write_x15_reachability_bundle(
        envelope_dir,
        commands=x15_reachability_commands()[:6],
        horizon_s=120.0,
        step_size_s=1.0,
        dpi=90,
    )

    replay = write_x15_native_boundary_replay(
        envelope_dir / "pseudo_6dof.json",
        tmp_path / "native-replay",
        max_points=1,
    )

    assert len(replay.records) == 1
    assert replay.records[0].completed
    assert replay.records[0].native_artifact is not None
    assert replay.records[0].comparison["native_replay_terminal_speed_m_s"] is not None
    assert replay.records[0].comparison["native_replay_terminal_mass_kg"] == 14_641.0545
