from __future__ import annotations

import json
import math
import os
from pathlib import Path

import pytest

from taoryx.contracts import Vector3
from taoryx.reachability_envelope import (
    LaunchCommand,
    PointMass3DofState,
    Pseudo6DofState,
    ReachabilityFidelity,
    RigidBody6DofReachabilityState,
    RocketGlideVehicle,
    RocketStageSpec,
    StagedRocketSpec,
    StageSeparationSpec,
    TerminalCriteria,
    generate_launch_grid,
    rerun_timed_out_envelope,
    run_reachability_envelope,
    simulate_rocket_glide,
)
from taoryx.reachability_visualization import render_reachability_plot_bundle
from taoryx.runtime.cli import main
from taoryx.vehicle import DetachedBodyDefinition, ImpulseFrame, PropellantType, PropulsionCapabilities, SeparationMechanism


def _commands() -> tuple[LaunchCommand, ...]:
    return generate_launch_grid(
        (math.radians(-20.0), math.radians(20.0)),
        (math.radians(45.0),),
        (math.radians(-15.0), math.radians(15.0)),
    )
    ####


def _norm_difference(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right, strict=True)))


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
    try:
        os.sysconf("SC_SEM_NSEMS_MAX")
    except (AttributeError, OSError, PermissionError, ValueError):
        pytest.skip("spawned process pools are unavailable in this sandbox")
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
    assert result.as_dict()["execution"]["deployment_composition"] == "standalone"
    ####


def test_deployment_artifact_has_top_level_passive_child_contract() -> None:
    from taoryx.x15_reachability import x15_surrogate_vehicle

    result = run_reachability_envelope(
        x15_surrogate_vehicle(),
        (LaunchCommand(0.0, math.radians(45.0)),),
        fidelity=ReachabilityFidelity.POINT_MASS_3DOF,
        horizon_s=60.0,
        step_size_s=1.0,
        spawn_children=True,
    )

    deployment = result.as_dict()["deployment"]

    assert deployment["kind"] == "passive_aero_ballistic"
    assert deployment["enabled"] is True
    assert deployment["configured"] is True
    assert deployment["accepted_event_count"] == 1
    assert deployment["spawned_child_count"] == 1
    assert deployment["child_models"][0]["shape"] == "cylinder"
    assert deployment["child_classification_counts"]
    ####


def test_staged_spec_closes_mass_flow_and_separation_without_parallel_scalars() -> None:
    spent_booster = DetachedBodyDefinition.cylinder(
        "spent-booster",
        mass_kg=50.0,
        radius_m=0.5,
        length_m=2.0,
        inertia_kg_m2=Vector3(10.0, 20.0, 20.0),
    )
    spec = StagedRocketSpec(
        core_stage=RocketStageSpec("glider", dry_mass_kg=100.0),
        attached_stages=(
            RocketStageSpec(
                "booster",
                dry_mass_kg=50.0,
                propellant_mass_kg=20.0,
                thrust_n=1_000.0,
                burn_time_s=10.0,
                mass_flow_kg_s=2.0,
                declared_propellant_mass_kg=30.0,
            ),
        ),
        separation_events=(StageSeparationSpec("booster", time_s=15.0, detached_body=spent_booster),),
    )

    vehicle = RocketGlideVehicle.from_staged_spec(spec)

    assert spec.initial_mass_kg == 170.0
    assert spec.retained_mass_kg == 100.0
    assert spec.ejected_mass_kg == 50.0
    assert spec.declared_propellant_discrepancy_kg == 10.0
    assert spec.separation_events[0].detached_body is spent_booster
    assert vehicle.initial_mass_kg == 170.0
    assert vehicle.release_mass_kg == 100.0
    ####


def test_four_canonical_detached_body_profiles_have_stable_geometry_contracts() -> None:
    bodies = (
        DetachedBodyDefinition.sphere("sphere", mass_kg=10.0, radius_m=0.5),
        DetachedBodyDefinition.cylinder("cylinder", mass_kg=10.0, radius_m=0.5, length_m=2.0, inertia_kg_m2=Vector3(1.0, 2.0, 2.0)),
        DetachedBodyDefinition.spheroid(
            "spheroid",
            mass_kg=10.0,
            axial_semi_axis_m=1.5,
            transverse_semi_axis_m=0.5,
            inertia_kg_m2=Vector3(1.0, 2.0, 2.0),
        ),
        DetachedBodyDefinition.cone(
            "cone",
            mass_kg=10.0,
            base_radius_m=0.5,
            height_m=2.0,
            inertia_kg_m2=Vector3(1.0, 2.0, 2.0),
        ),
    )

    assert [body.shape.value for body in bodies] == ["sphere", "cylinder", "spheroid", "cone"]
    assert all(body.reference_area_m2 > 0.0 for body in bodies)
    for body in bodies:
        result = simulate_rocket_glide(
            RocketGlideVehicle(
                dry_mass_kg=100.0,
                propellant_mass_kg=10.0,
                thrust_n=1_000.0,
                burn_time_s=2.0,
                initial_altitude_m=100.0,
                booster_dry_mass_kg=10.0,
                booster_propellant_mass_kg=5.0,
                booster_thrust_n=1_000.0,
                booster_burn_time_s=1.0,
                booster_release_time_s=1.5,
                booster_detached_body=body,
            ),
            LaunchCommand(0.0, math.radians(45.0)),
            horizon_s=2.0,
            step_size_s=0.25,
            spawn_children=True,
        )
        assert len(result.spawned_bodies) == 1
        assert result.spawned_bodies[0].shape == body.shape.value
        assert all(float(row["projected_area_m2"]) > 0.0 for row in result.spawned_bodies[0].telemetry)


def test_triaxial_ellipsoid_detached_body_has_orientation_dependent_area() -> None:
    body = DetachedBodyDefinition.triaxial_ellipsoid(
        "elliptic-tank",
        mass_kg=10.0,
        semi_axis_x_m=1.5,
        semi_axis_y_m=0.75,
        semi_axis_z_m=0.5,
        inertia_kg_m2=Vector3(1.0, 2.0, 3.0),
        initial_angular_rate_body_rad_s=Vector3(0.25, 0.4, 0.6),
    )
    vehicle = RocketGlideVehicle(
        dry_mass_kg=100.0,
        propellant_mass_kg=10.0,
        thrust_n=1_000.0,
        burn_time_s=2.0,
        initial_altitude_m=100.0,
        booster_dry_mass_kg=10.0,
        booster_propellant_mass_kg=5.0,
        booster_thrust_n=1_000.0,
        booster_burn_time_s=1.0,
        booster_release_time_s=1.5,
        booster_detached_body=body,
    )

    result = simulate_rocket_glide(
        vehicle,
        LaunchCommand(0.0, math.radians(45.0)),
        fidelity=ReachabilityFidelity.PSEUDO_6DOF,
        horizon_s=3.0,
        step_size_s=0.1,
        spawn_children=True,
    )

    child = result.spawned_bodies[0]
    areas = [float(row["projected_area_m2"]) for row in child.telemetry]
    assert child.shape == "triaxial_ellipsoid"
    assert max(areas) > min(areas)
    assert all("attitude_rate_rad_s" in row for row in child.telemetry)


def test_reachability_can_spawn_and_serialize_a_detached_ballistic_witness(tmp_path: Path) -> None:
    body = DetachedBodyDefinition.cylinder(
        "spent-booster",
        mass_kg=5.0,
        radius_m=0.2,
        length_m=1.0,
        inertia_kg_m2=Vector3(1.0, 1.0, 1.0),
    )
    vehicle = RocketGlideVehicle(
        dry_mass_kg=100.0,
        propellant_mass_kg=10.0,
        thrust_n=1_000.0,
        burn_time_s=2.0,
        initial_altitude_m=100.0,
        booster_dry_mass_kg=5.0,
        booster_propellant_mass_kg=5.0,
        booster_thrust_n=1_000.0,
        booster_burn_time_s=1.0,
        booster_release_time_s=1.5,
        booster_detached_body=body,
    )

    trajectory = simulate_rocket_glide(
        vehicle,
        LaunchCommand(0.0, math.radians(45.0)),
        horizon_s=2.0,
        step_size_s=0.25,
        spawn_children=True,
    )
    envelope = run_reachability_envelope(
        vehicle,
        (LaunchCommand(0.0, math.radians(45.0)),),
        horizon_s=2.0,
        step_size_s=0.25,
        spawn_children=True,
    )

    assert len(trajectory.spawned_bodies) == 1
    assert trajectory.spawned_bodies[0].body_id == "spent-booster"
    assert trajectory.deployment_events[0]["event_id"] == "booster-release"
    payload = envelope.as_dict(include_trajectories=True)
    assert payload["execution"]["spawn_children"] is True
    assert payload["samples"][0]["spawned_bodies"][0]["shape"] == "cylinder"
    assert payload["samples"][0]["spawned_bodies"][0]["classification"] in {"impact", "timeout", "invalid"}
    assert sum(payload["summary"]["child_classification_counts"].values()) == 1
    json.dumps(payload)
    report = render_reachability_plot_bundle(envelope, tmp_path / "plots")
    assert (tmp_path / "plots" / "deployment-timeline.png") in report.plot_paths
    assert (tmp_path / "plots" / "trajectory_parent.png") in report.plot_paths
    assert (tmp_path / "plots" / "trajectory_children.png") in report.plot_paths
    assert (tmp_path / "plots" / "event_timeline.png") in report.plot_paths
    assert (tmp_path / "plots" / "projected_area.png") in report.plot_paths
    assert (tmp_path / "plots" / "telemetry_parent.csv") in report.artifact_paths
    assert (tmp_path / "plots" / "telemetry_spent-booster.csv") in report.artifact_paths
    manifest = json.loads(report.manifest_path.read_text(encoding="utf-8"))
    assert manifest["deployment_event_count"] == 1
    assert manifest["source_configuration_hash"]


def test_detached_body_aerodynamics_use_declared_crosswind() -> None:
    body = DetachedBodyDefinition.cylinder(
        "winded-stage",
        mass_kg=5.0,
        radius_m=0.2,
        length_m=1.0,
        inertia_kg_m2=Vector3(1.0, 1.0, 1.0),
    )
    vehicle = RocketGlideVehicle(
        dry_mass_kg=100.0,
        propellant_mass_kg=10.0,
        thrust_n=1_000.0,
        burn_time_s=2.0,
        initial_altitude_m=100.0,
        wind_velocity_m_s=Vector3(0.0, 10.0, 0.0),
        booster_dry_mass_kg=5.0,
        booster_propellant_mass_kg=5.0,
        booster_thrust_n=1_000.0,
        booster_burn_time_s=1.0,
        booster_release_time_s=1.5,
        booster_detached_body=body,
    )

    result = simulate_rocket_glide(
        vehicle,
        LaunchCommand(0.0, math.radians(45.0)),
        horizon_s=2.0,
        step_size_s=0.25,
        spawn_children=True,
    )
    row = result.spawned_bodies[0].telemetry[0]
    velocity = row["velocity_m_s"]
    air_velocity = row["air_relative_velocity_m_s"]

    assert air_velocity == pytest.approx([velocity[0], velocity[1] - 10.0, velocity[2]])
    assert row["air_relative_speed_m_s"] == pytest.approx(math.sqrt(sum(value * value for value in air_velocity)))
    assert row["air_relative_speed_m_s"] != pytest.approx(math.sqrt(sum(value * value for value in velocity)))


def test_pseudo_sixdof_passive_tumble_changes_projected_area_and_records_rates() -> None:
    body = DetachedBodyDefinition.cylinder(
        "tumbling-stage",
        mass_kg=5.0,
        radius_m=0.2,
        length_m=2.0,
        inertia_kg_m2=Vector3(1.0, 2.0, 3.0),
        initial_angular_rate_body_rad_s=Vector3(0.25, 0.4, 0.6),
    )
    vehicle = RocketGlideVehicle(
        dry_mass_kg=100.0,
        propellant_mass_kg=10.0,
        thrust_n=1_000.0,
        burn_time_s=2.0,
        initial_altitude_m=100.0,
        booster_dry_mass_kg=5.0,
        booster_propellant_mass_kg=5.0,
        booster_thrust_n=1_000.0,
        booster_burn_time_s=1.0,
        booster_release_time_s=1.5,
        booster_detached_body=body,
    )

    result = simulate_rocket_glide(
        vehicle,
        LaunchCommand(0.0, math.radians(45.0)),
        fidelity=ReachabilityFidelity.PSEUDO_6DOF,
        horizon_s=3.0,
        step_size_s=0.1,
        spawn_children=True,
    )
    child = result.spawned_bodies[0]
    areas = [float(row["projected_area_m2"]) for row in child.telemetry]
    rates = [tuple(row["attitude_rate_rad_s"]) for row in child.telemetry if "attitude_rate_rad_s" in row]

    assert max(areas) > min(areas)
    assert rates[0] != rates[-1]
    assert all("drag_force_n" in row and "event_id" in row for row in child.telemetry)


def test_rigid_body_tier_uses_native_state_and_records_force_moment_telemetry() -> None:
    body = DetachedBodyDefinition.cylinder(
        "rigid-stage",
        mass_kg=5.0,
        radius_m=0.2,
        length_m=1.0,
        inertia_kg_m2=Vector3(1.0, 2.0, 3.0),
        initial_angular_rate_body_rad_s=Vector3(0.05, 0.1, 0.15),
    )
    vehicle = RocketGlideVehicle(
        dry_mass_kg=100.0,
        propellant_mass_kg=10.0,
        thrust_n=1_000.0,
        burn_time_s=2.0,
        initial_altitude_m=100.0,
        booster_dry_mass_kg=5.0,
        booster_propellant_mass_kg=5.0,
        booster_thrust_n=1_000.0,
        booster_burn_time_s=1.0,
        booster_release_time_s=1.5,
        booster_detached_body=body,
    )

    result = simulate_rocket_glide(
        vehicle,
        LaunchCommand(0.0, math.radians(45.0)),
        fidelity=ReachabilityFidelity.RIGID_BODY_6DOF,
        horizon_s=3.0,
        step_size_s=0.25,
        spawn_children=True,
    )

    child = result.spawned_bodies[0]
    assert isinstance(child.states[0], RigidBody6DofReachabilityState)
    assert all(abs(math.sqrt(sum(value * value for value in row["attitude_quaternion"])) - 1.0) < 1.0e-9 for row in child.telemetry)
    assert all("moment_body_x_nm" in row and "total_force_ecic_n" in row for row in child.telemetry)
    assert float(result.deployment_events[0]["momentum_residual_kg_m_s"]) == pytest.approx(0.0, abs=1.0e-9)

    refined = simulate_rocket_glide(
        vehicle,
        LaunchCommand(0.0, math.radians(45.0)),
        fidelity=ReachabilityFidelity.RIGID_BODY_6DOF,
        horizon_s=3.0,
        step_size_s=0.1,
        spawn_children=True,
    )
    refined_terminal = refined.spawned_bodies[0].terminal
    assert _norm_difference(child.terminal.position_m, refined_terminal.position_m) < 25.0
    assert child.terminal.speed_m_s == pytest.approx(refined_terminal.speed_m_s, abs=2.0)


def test_detached_body_requires_inertia_for_tumbling() -> None:
    with pytest.raises(ValueError, match="require inertia"):
        DetachedBodyDefinition.spheroid(
            "spent-tank",
            mass_kg=10.0,
            axial_semi_axis_m=1.5,
            transverse_semi_axis_m=0.5,
        )


def test_propulsion_capabilities_guard_solid_cutoff_and_throttle_requests() -> None:
    liquid = PropulsionCapabilities.defaults_for(PropellantType.LIQUID)
    liquid.validate_throttle(0.4)
    liquid.validate_cutoff(True)

    solid = PropulsionCapabilities.defaults_for(PropellantType.SOLID)
    with pytest.raises(ValueError, match="cannot throttle"):
        solid.validate_throttle(0.4)
    with pytest.raises(ValueError, match="cannot be commanded off"):
        solid.validate_cutoff(True)
    ####


def test_stage_separation_records_body_kick_and_converts_it_to_delta_v() -> None:
    event = StageSeparationSpec(
        "booster",
        time_s=15.0,
        mechanism=SeparationMechanism.PYROTECHNIC,
        impulse_body_n_s=Vector3(0.0, 0.0, -100.0),
        separation_energy_j=250.0,
    )

    assert event.impulse_magnitude_n_s == 100.0
    assert event.impulse_direction_body == Vector3(0.0, 0.0, -1.0)
    assert event.retained_delta_v_m_s(100.0) == Vector3(0.0, 0.0, -1.0)
    ####


def test_stage_separation_keeps_inertial_impulses_distinct_from_body_direction() -> None:
    event = StageSeparationSpec(
        "booster",
        time_s=15.0,
        mechanism=SeparationMechanism.SPRING,
        impulse_body_n_s=Vector3(100.0, 0.0, 0.0),
        impulse_frame=ImpulseFrame.INERTIAL,
    )

    assert event.impulse_direction_body is None
    assert event.impulse_direction == Vector3(1.0, 0.0, 0.0)


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
