from __future__ import annotations

import json
import math
from pathlib import Path

from taoryx.reachability_envelope import (
    LaunchCommand,
    PointMass3DofState,
    Pseudo6DofState,
    ReachabilityFidelity,
    RocketGlideVehicle,
    TerminalCriteria,
    generate_launch_grid,
    rerun_timed_out_envelope,
    run_reachability_envelope,
    simulate_rocket_glide,
)
from taoryx.reachability_visualization import render_reachability_plot_bundle
from taoryx.runtime.cli import main


def _commands() -> tuple[LaunchCommand, ...]:
    return generate_launch_grid(
        (math.radians(-20.0), math.radians(20.0)),
        (math.radians(45.0),),
        (math.radians(-15.0), math.radians(15.0)),
    )
    ####


def test_point_mass_trajectory_crosses_boost_to_glide_boundary() -> None:
    result = simulate_rocket_glide(
        RocketGlideVehicle(),
        LaunchCommand(0.0, math.radians(45.0)),
        fidelity=ReachabilityFidelity.POINT_MASS_3DOF,
        step_size_s=0.25,
        horizon_s=10.0,
    )

    assert isinstance(result.states[0], PointMass3DofState)
    assert result.states[0].phase == "boost"
    assert result.states[-1].phase == "glide"
    assert result.terminal.mass_kg == RocketGlideVehicle().dry_mass_kg
    assert result.terminal.position_m[2] > result.states[0].position_m[2]
    ####


def test_pseudo_six_dof_adds_filtered_attitude_state() -> None:
    result = simulate_rocket_glide(
        RocketGlideVehicle(),
        LaunchCommand(math.radians(20.0), math.radians(45.0), math.radians(15.0)),
        fidelity=ReachabilityFidelity.PSEUDO_6DOF,
        step_size_s=0.1,
        horizon_s=1.0,
    )

    assert isinstance(result.terminal, Pseudo6DofState)
    assert result.terminal.attitude_rad != (0.0, 0.0, 0.0)
    assert result.terminal.attitude_rate_rad_s != (0.0, 0.0, 0.0)
    ####


def test_serial_and_spawned_envelopes_preserve_sample_order() -> None:
    vehicle = RocketGlideVehicle()
    commands = _commands()
    serial = run_reachability_envelope(vehicle, commands, horizon_s=2.0, step_size_s=0.25, workers=1)
    parallel = run_reachability_envelope(vehicle, commands, horizon_s=2.0, step_size_s=0.25, workers=2)

    assert [sample.index for sample in serial.samples] == [0, 1, 2, 3]
    serial_payload = serial.as_dict()
    parallel_payload = parallel.as_dict()
    serial_payload["workers"] = parallel_payload["workers"]
    serial_payload["execution"] = parallel_payload["execution"]
    assert serial_payload == parallel_payload
    assert serial.bounds is not None
    ####


def test_envelope_summary_is_json_compatible() -> None:
    result = run_reachability_envelope(RocketGlideVehicle(), _commands()[:1], horizon_s=1.0, step_size_s=0.25)
    encoded = json.dumps(result.as_dict(include_trajectories=True))

    assert "trajectory" in encoded
    assert result.samples[0].trajectory.terminal.time_s == 1.0
    ####


def test_impact_radius_and_speed_goals_are_explicit() -> None:
    vehicle = RocketGlideVehicle(
        thrust_n=1.0,
        burn_time_s=0.1,
        initial_speed_m_s=0.0,
        initial_altitude_m=1.0,
    )
    result = run_reachability_envelope(
        vehicle,
        (LaunchCommand(0.0, math.radians(45.0)),),
        horizon_s=2.0,
        step_size_s=0.1,
        criteria=TerminalCriteria(
            require_ground_contact=True,
            max_impact_radius_m=1.0,
            min_impact_speed_m_s=0.0,
            max_impact_speed_m_s=100.0,
        ),
    )

    sample = result.samples[0]
    assert sample.feasible
    assert sample.trajectory.termination.value == "ground_contact"
    margins = dict(sample.terminal_margins)
    assert margins["impact_radius_margin_m"] >= 0.0
    assert margins["impact_speed_max_margin_m_s"] >= 0.0


def test_timed_out_candidates_are_reported_and_can_be_rerun() -> None:
    vehicle = RocketGlideVehicle()
    result = run_reachability_envelope(vehicle, _commands(), horizon_s=0.25, step_size_s=0.25)

    assert len(result.timed_out_samples) == 4
    payload = result.as_dict()
    assert payload["summary"]["timed_out_query_ids"] == [
        "query-000000",
        "query-000001",
        "query-000002",
        "query-000003",
    ]
    rerun = rerun_timed_out_envelope(vehicle, result, horizon_s=1.0)
    assert len(rerun.samples) == 4
    assert dict(rerun.provenance)["rerun_reason"] == "timed_out_candidates"
    assert rerun.horizon_s == 1.0
    ####


def test_result_format_records_search_space_outcomes_and_plot_rows() -> None:
    result = run_reachability_envelope(
        RocketGlideVehicle(),
        _commands(),
        horizon_s=1.0,
        step_size_s=0.25,
        criteria=TerminalCriteria(min_speed_m_s=10_000.0),
    )
    payload = result.as_dict(include_trajectories=True)
    sample = payload["samples"][0]

    assert payload["schema"] == "trajectory.reachability-envelope/v1alpha1"
    assert payload["search_space"]["composition"] == "cartesian_product"
    assert payload["search_space"]["candidate_count"] == 4
    assert payload["study"]["vehicle_parameters"]["burn_time_s"] == 8.0
    assert payload["summary"]["classification_counts"] == {"infeasible": 4}
    assert payload["summary"]["failure_reason_counts"] == {"terminal_speed_low": 4}
    assert payload["summary"]["successful_query_ids"] == []
    assert payload["summary"]["unsuccessful_query_ids"] == [
        "query-000000",
        "query-000001",
        "query-000002",
        "query-000003",
    ]
    assert sample["classification"] == "infeasible"
    assert sample["failure_reasons"] == ["terminal_speed_low"]
    assert sample["path_metrics"]["duration_s"] == 1.0
    assert sample["trajectory"]["fields"][-1] == "phase"
    assert len(sample["trajectory"]["rows"]) == 5
    ####


def test_reachability_cli_runs_fixture_and_writes_summary(tmp_path: Path) -> None:
    output = tmp_path / "envelope.json"
    assert (
        main(
            [
                "reachability",
                "run",
                "--azimuth-deg",
                "0",
                "--elevation-deg",
                "45",
                "--bank-deg",
                "0",
                "--horizon-s",
                "1",
                "--step-size-s",
                "0.25",
                "--output",
                str(output),
            ]
        )
        == 0
    )

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["fidelity"] == "point_mass_3dof"
    assert payload["sample_count"] == 1
    assert payload["feasible_count"] == 1
    rerun_output = tmp_path / "timeout-rerun.json"
    assert main(
        [
            "reachability",
            "rerun-timeouts",
            str(output),
            "--horizon-s",
            "2",
            "--output",
            str(rerun_output),
        ]
    ) == 0
    rerun_payload = json.loads(rerun_output.read_text(encoding="utf-8"))
    assert rerun_payload["study"]["study_id"].endswith("_timeout_rerun")
    assert rerun_payload["provenance"]["rerun_reason"] == "timed_out_candidates"
    ####


def test_reachability_plot_bundle_renders_trajectories_coverage_and_capability(tmp_path: Path) -> None:
    point_mass = run_reachability_envelope(RocketGlideVehicle(), _commands(), horizon_s=1.0, step_size_s=0.25)
    pseudo = run_reachability_envelope(
        RocketGlideVehicle(),
        _commands(),
        fidelity=ReachabilityFidelity.PSEUDO_6DOF,
        horizon_s=1.0,
        step_size_s=0.25,
    )
    artifact_path = tmp_path / "point-mass.json"
    point_mass.write_json(artifact_path)

    report = render_reachability_plot_bundle(
        json.loads(artifact_path.read_text(encoding="utf-8")),
        tmp_path / "plots",
        comparison_sources=(pseudo,),
        dpi=90,
    )

    assert {path.name for path in report.plot_paths} == {
        "flown-trajectories.png",
        "search-coverage.png",
        "terminal-capability.png",
        "fidelity-progression.png",
    }
    assert all(path.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n" for path in report.plot_paths)
    manifest = json.loads(report.manifest_path.read_text(encoding="utf-8"))
    assert manifest["schema"] == "trajectory.reachability-plot-bundle/v1alpha1"
    cli_output = tmp_path / "cli-plots"
    assert main(["reachability", "plot", str(artifact_path), "--output-dir", str(cli_output), "--dpi", "90"]) == 0
    assert (cli_output / "terminal-capability.png").exists()
    ####
