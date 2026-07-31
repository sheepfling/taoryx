from __future__ import annotations

import math

import pytest

from taoryx.contracts import Vector3
from taoryx.hl20_controls import HL20ActuatorProfile
from taoryx.hl20_reachability import (
    hl20_ca_hi_terminal_criteria,
    hl20_source_release_vehicle,
    hl20_source_surface_replay_vehicle,
    run_hl20_source_release,
)
from taoryx.reachability_aerodynamics import (
    HL20_SOURCE_MODEL_ID,
    HL20DavemlAerodynamics,
    probe_hl20_control_directions,
)
from taoryx.reachability_envelope import (
    LaunchCommand,
    ReachabilityFidelity,
    RocketGlideVehicle,
    run_reachability_envelope,
    timed_out_commands_from_artifact,
)


def test_hl20_source_provider_returns_six_load_channels_and_provenance() -> None:
    alpha = math.radians(5.0)
    speed = 340.294
    loads = HL20DavemlAerodynamics().evaluate((speed * math.cos(alpha), 0.0, -speed * math.sin(alpha)), 0.0)

    assert loads.force_body_n[0] < 0.0
    assert loads.force_body_n[2] < 0.0
    assert set(dict(loads.coefficients)) == {"cl", "cd", "cm", "cn", "cr", "cy"}
    assert dict(loads.coefficients)["cl"] == pytest.approx(0.17524959583333333)
    assert dict(loads.coefficients)["cd"] == pytest.approx(0.12083715666666667)
    assert dict(loads.coefficients)["cm"] == pytest.approx(0.007435285000000005)
    assert dict(loads.operating_point)["mach"] == pytest.approx(1.0)
    assert loads.as_dict()["operating_point"]
    assert HL20DavemlAerodynamics().provenance["source_package_sha256"]


def test_hl20_actuator_profile_reports_position_and_rate_limits() -> None:
    profile = HL20ActuatorProfile()
    trace = profile.realize(
        {"upper_left_body_flap": 10.0, "lower_left_body_flap": 20.0},
        duration_s=0.1,
    )

    assert "upper_left_body_flap" in trace.saturated
    assert "lower_left_body_flap" not in trace.saturated
    assert dict(trace.rate_deg_s)["lower_left_body_flap"] == pytest.approx(profile.rate_limit_deg_s)
    assert trace.as_dict()["achieved_deg"]


def test_hl20_source_provider_fails_closed_outside_source_envelope() -> None:
    with pytest.raises(ValueError, match="outside its validity envelope"):
        HL20DavemlAerodynamics().evaluate((50.0, 0.0, -1.0), 0.0)

    with pytest.raises(ValueError, match="surface query"):
        HL20DavemlAerodynamics().evaluate(
            (340.294, 0.0, 0.0),
            0.0,
            controls={"upper_left_body_flap": 1.0},
        )


def test_hl20_source_control_direction_probe_is_reproducible_and_bounded() -> None:
    report = probe_hl20_control_directions()

    assert set(report["responses"]) == {
        "upper_left_body_flap",
        "lower_left_body_flap",
        "upper_right_body_flap",
        "lower_right_body_flap",
        "left_wing_flap",
        "right_wing_flap",
        "rudder",
    }
    for response in report["responses"].values():
        assert response["nonzero_channels"]
        assert response["command_deg"] != 0.0
    assert report["provenance"]["source_package_sha256"]


def test_source_reachability_selector_is_serializable_and_uses_source_graph() -> None:
    vehicle = RocketGlideVehicle(
        dry_mass_kg=8_664.1,
        propellant_mass_kg=0.0,
        thrust_n=0.0,
        burn_time_s=0.0,
        initial_speed_m_s=300.0,
        initial_altitude_m=1_000.0,
        reference_area_m2=26.612075808,
        aerodynamic_model_id=HL20_SOURCE_MODEL_ID,
        actuator_profile_id="hl20.reference_first_order.v1",
    )
    source_result = run_reachability_envelope(
        vehicle,
        (LaunchCommand(0.0, 0.0, surface_commands_deg=(-5.0, 5.0, -5.0, 5.0, 2.0, -2.0, 0.0)),),
        fidelity=ReachabilityFidelity.POINT_MASS_3DOF,
        step_size_s=0.5,
        horizon_s=1.0,
        spawn_children=False,
    )
    payload = source_result.as_dict(include_trajectories=True)
    assert payload["study"]["model"]["force_model"][-2:] == [
        "hl20_daveml_force_moment_graph",
        "source_lift_and_side_force",
    ]
    assert payload["provenance"]["aerodynamic_model"] == HL20_SOURCE_MODEL_ID
    assert any("source_aerodynamics" in row for row in payload["samples"][0]["telemetry"])
    assert payload["samples"][0]["decision_vector"]["surface_commands_deg"] == [-5.0, 5.0, -5.0, 5.0, 2.0, -2.0, 0.0]
    assert timed_out_commands_from_artifact(payload)[0].surface_commands_deg == (-5.0, 5.0, -5.0, 5.0, 2.0, -2.0, 0.0)
    assert any("actuator" in row["source_aerodynamics"] for row in payload["samples"][0]["telemetry"] if "source_aerodynamics" in row)


def test_hl20_source_mission_profile_records_energy_managed_segments() -> None:
    result = run_hl20_source_release(
        fidelity=ReachabilityFidelity.PSEUDO_6DOF,
        horizon_s=100.0,
        step_size_s=0.5,
        spawn_children=False,
    )

    nominal = result.samples[0]
    segments = [str(event["segment"]) for event in nominal.trajectory.mission_events]
    assert segments == ["stabilization", "glide_trim", "right_bank", "left_bank", "energy_descent"]
    assert all("specific_energy_j_per_kg" in row for row in nominal.trajectory.telemetry)
    assert all("mission_bank_command_deg" in row for row in nominal.trajectory.telemetry)

    rigid = run_hl20_source_release(
        commands=(LaunchCommand(0.0, math.radians(60.0)),),
        fidelity=ReachabilityFidelity.RIGID_BODY_6DOF,
        horizon_s=36.0,
        step_size_s=0.5,
        spawn_children=False,
    )
    rigid_rows = rigid.samples[0].trajectory.telemetry
    bank_rows = [row for row in rigid_rows if float(row["time_s"]) >= 35.0]
    assert bank_rows
    assert max(float(row["mission_bank_command_deg"]) for row in bank_rows) == pytest.approx(30.0)
    assert all("attitude_control_error_magnitude_rad" in row for row in bank_rows)


def test_hl20_ca_hi_terminal_contract_round_trips_explicit_target_semantics() -> None:
    criteria = hl20_ca_hi_terminal_criteria()
    restored = type(criteria).from_dict(criteria.as_dict())

    assert restored.target_id == "honolulu_terminal_reference"
    assert restored.target_frame == "local_ecic_tangent_from_california_origin"
    assert restored.max_impact_radius_m == pytest.approx(40_000.0)
    assert restored.min_impact_speed_m_s == pytest.approx(1_000.0)


def test_source_vehicle_variant_serializes_wind_and_explicit_mass_properties() -> None:
    vehicle = hl20_source_release_vehicle(
        configuration_variant_id="crosswind_probe",
        wind_velocity_m_s=Vector3(0.0, 10.0, 0.0),
    )
    assert vehicle.pseudo6dof_profile_id == "hl20.attitude_response_p6dof.v1"
    result = run_reachability_envelope(
        vehicle,
        (LaunchCommand(0.0, math.radians(60.0)),),
        fidelity=ReachabilityFidelity.POINT_MASS_3DOF,
        step_size_s=0.5,
        horizon_s=30.0,
        spawn_children=False,
    )
    payload = result.as_dict(include_trajectories=True)
    parameters = payload["study"]["vehicle_parameters"]
    assert parameters["configuration_variant_id"] == "crosswind_probe"
    assert parameters["pseudo6dof_profile_id"] == "hl20.attitude_response_p6dof.v1"
    assert parameters["wind_velocity_m_s"] == [0.0, 10.0, 0.0]
    assert parameters["inertia_body_kg_m2"]
    source_rows = [row["source_aerodynamics"] for row in payload["samples"][0]["telemetry"] if "source_aerodynamics" in row]
    assert source_rows
    assert abs(float(dict(source_rows[0]["operating_point"])["beta_deg"])) > 0.0


def test_hl20_surface_replay_uses_source_loads_without_direct_control_moment() -> None:
    vehicle = hl20_source_surface_replay_vehicle()
    command = LaunchCommand(0.0, 0.0, surface_commands_deg=(0.0, 0.0, 0.0, 0.0, 30.0, 30.0, 0.0))
    result = run_reachability_envelope(
        vehicle,
        (command,),
        fidelity=ReachabilityFidelity.RIGID_BODY_6DOF_SURFACE_ALLOCATED,
        step_size_s=0.01,
        horizon_s=0.5,
        spawn_children=False,
    )
    sample = result.samples[0]
    assert sample.feasible
    assert sample.failure_reasons == ()
    assert all(row.get("direct_body_moment_injection") == 0.0 for row in sample.trajectory.telemetry)
    assert all(row.get("surface_allocation_mode") == "source_open_loop_replay" for row in sample.trajectory.telemetry)
    assert all("source_aerodynamics" in row for row in sample.trajectory.telemetry)
