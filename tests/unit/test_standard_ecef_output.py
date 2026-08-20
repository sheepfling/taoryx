"""Regression coverage for the universal Mission Composition ECEF pose."""

from __future__ import annotations

import math

import pytest
from taoryx_trajectory_contracts import StandardEcefState as PublicStandardEcefState

from taoryx.contracts import Frame, FrameVector3, Vector3
from taoryx.modes import DynamicsMode, Kinematic6DofState, Quaternion
from taoryx.runtime.common import RuntimeState, RuntimeVehicle
from taoryx.runtime.observations import observe_vehicle
from taoryx.trajectory.execution_contract import (
    MissionCompositionTrajectoryResult,
    TrajectoryChannelMetadata,
    TrajectoryObject,
    TrajectorySample,
)
from taoryx.trajectory.providers import SessionState, TrajectoryResult
from taoryx.trajectory.session_contract import MissionCompositionSessionObservation
from taoryx.trajectory.standard_output import StandardEcefState, project_standard_ecef_samples


def test_standard_ecef_type_is_shared_with_the_standalone_contract_package() -> None:
    assert StandardEcefState is PublicStandardEcefState
    ####


def test_geodetic_and_local_samples_project_to_explicit_wgs84_ecef() -> None:
    states = project_standard_ecef_samples(
        (
            (
                0.0,
                {
                    "position.geodetic.latitude": 0.0,
                    "position.geodetic.longitude": 0.0,
                    "position.geodetic.altitude": 10.0,
                },
            ),
            (
                1.0,
                {
                    "position.geodetic.latitude": 0.0,
                    "position.geodetic.longitude": 0.0,
                    "position.geodetic.altitude": 11.0,
                },
            ),
        ),
        channel_units={
            "position.geodetic.latitude": "deg",
            "position.geodetic.longitude": "deg",
            "position.geodetic.altitude": "m",
        },
    )

    assert states[0].frame_id == "ecfc"
    assert states[0].position_projection == "geodetic_wgs84"
    assert states[0].position_ecef_m == pytest.approx((6_378_147.0, 0.0, 0.0))
    assert states[0].velocity_ecef_mps == pytest.approx((1.0, 0.0, 0.0))
    assert states[0].acceleration_ecef_mps2 == pytest.approx((0.0, 0.0, 0.0))

    local = project_standard_ecef_samples(
        ((0.0, {"position.local.north": 2.0, "position.local.east": 3.0, "position.geometric.altitude": 4.0}),),
        channel_frames={
            "position.local.north": "local_ned",
            "position.local.east": "local_ned",
            "position.geometric.altitude": "local_ned",
        },
    )[0]
    assert local.position_projection == "local_ned_wgs84_equatorial_embedding"
    assert local.position_ecef_m == pytest.approx((6_378_141.0, 3.0, 2.0))
    assert local.orientation_kind == "local_ned_reference"
    ####


def test_ecic_velocity_becomes_earth_relative_ecef_velocity() -> None:
    radius_m = 6_378_137.0
    earth_rotation_radps = 7.2921150e-5
    state = project_standard_ecef_samples(
        (
            (
                0.0,
                {
                    "position.eci.x": radius_m,
                    "position.eci.y": 0.0,
                    "position.eci.z": 0.0,
                    "velocity.eci.x": 0.0,
                    "velocity.eci.y": earth_rotation_radps * radius_m,
                    "velocity.eci.z": 0.0,
                },
            ),
        )
    )[0]

    assert state.position_projection == "ecic_to_ecfc_zero_epoch"
    assert state.position_ecef_m == pytest.approx((radius_m, 0.0, 0.0))
    assert state.velocity_ecef_mps == pytest.approx((0.0, 0.0, 0.0), abs=1.0e-12)
    assert math.isclose(sum(value * value for value in state.ecef_from_body_wxyz), 1.0, abs_tol=1.0e-12)
    ####


def test_advertised_inertial_and_local_ned_vectors_keep_native_attitude_truth() -> None:
    inertial = project_standard_ecef_samples(
        (
            (
                0.0,
                {
                    "position_inertial_m": (6_378_137.0, 0.0, 0.0),
                    "velocity_inertial_mps": (0.0, 7.2921150e-5 * 6_378_137.0, 0.0),
                    "quaternion_wxyz": (1.0, 0.0, 0.0, 0.0),
                },
            ),
        ),
        channel_frames={
            "position_inertial_m": "cadac.earth_inertial",
            "velocity_inertial_mps": "cadac.earth_inertial",
        },
    )[0]
    assert inertial.position_projection == "ecic_to_ecfc_zero_epoch"
    assert inertial.orientation_kind == "native_inertial_to_body"
    assert inertial.ecef_from_body_wxyz == pytest.approx((1.0, 0.0, 0.0, 0.0))

    local_ned = project_standard_ecef_samples(
        (
            (
                0.0,
                {
                    "position_ned_m": (0.0, 0.0, 0.0),
                    "velocity_ned_mps": (1.0, 0.0, 0.0),
                    "quaternion_wxyz": (1.0, 0.0, 0.0, 0.0),
                    "body_rates_rad_s": (0.1, 0.2, 0.3),
                },
            ),
        )
    )[0]
    assert local_ned.orientation_kind == "native_body_from_local_ned"
    assert local_ned.ecef_from_body_wxyz == pytest.approx((math.sqrt(0.5), 0.0, -math.sqrt(0.5), 0.0))
    assert local_ned.angular_velocity_kind == "native_body_rate"
    assert local_ned.angular_velocity_body_radps == pytest.approx((0.1, 0.2, 0.3))
    ####


def test_angular_velocity_is_derived_from_standardized_quaternion_history() -> None:
    states = project_standard_ecef_samples(
        (
            (0.0, {"position_ned_m": (0.0, 0.0, 0.0), "quaternion_wxyz": (1.0, 0.0, 0.0, 0.0)}),
            (1.0, {"position_ned_m": (0.0, 0.0, 0.0), "quaternion_wxyz": (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))}),
        )
    )
    assert all(state.angular_velocity_kind == "orientation_finite_difference" for state in states)
    assert any(abs(component) > 1.0 for component in states[0].angular_velocity_body_radps)
    ####


def test_native_ecef_kinematics_and_world_from_body_quaternion_are_preserved() -> None:
    state = project_standard_ecef_samples(
        (
            (
                3.0,
                {
                    "position_ecfc_m": (6_378_140.0, 2.0, 1.0),
                    "velocity_ecfc_mps": (6.0, 5.0, 4.0),
                    "acceleration_ecfc_mps2": (3.0, 2.0, 1.0),
                    "ecef_from_body_wxyz": (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)),
                    "angular_velocity_body_radps": (0.1, 0.2, 0.3),
                },
            ),
        )
    )[0]

    assert state.position_projection == "native_ecfc"
    assert state.acceleration_kind == "native_earth_relative_acceleration"
    assert state.acceleration_ecef_mps2 == pytest.approx((3.0, 2.0, 1.0))
    assert state.orientation_kind == "native_ecef_from_body"
    assert state.ecef_from_body_wxyz == pytest.approx((math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)))
    assert state.angular_velocity_kind == "native_body_rate"
    assert state.angular_velocity_body_radps == pytest.approx((0.1, 0.2, 0.3))
    ####


def test_legacy_provider_and_runtime_observations_carry_the_same_standard_minimum() -> None:
    legacy = TrajectoryResult(
        provider_id="test.legacy",
        case_id="case",
        status="completed",
        samples=(
            SessionState(0.0, {"position.downrange_m": 0.0, "position.altitude_m": 10.0, "velocity.m_s": 2.0}),
            SessionState(1.0, {"position.downrange_m": 3.0, "position.altitude_m": 10.0, "velocity.m_s": 4.0}),
        ),
        applied_controls=(),
    )
    assert legacy.samples[0].standard_ecef.position_projection == "local_cartesian_wgs84_equatorial_embedding"
    assert legacy.samples[0].standard_ecef.velocity_ecef_mps == pytest.approx((0.0, 0.0, 2.0))
    assert legacy.samples[0].standard_ecef.acceleration_ecef_mps2 == pytest.approx((0.0, 0.0, 2.0))
    assert legacy.to_dict()["samples"][0]["standard_ecef"]["frame_id"] == "ecfc"

    position = FrameVector3(Vector3(6_378_140.0, 2.0, 1.0), Frame.ECFC)
    velocity = FrameVector3(Vector3(6.0, 5.0, 4.0), Frame.ECFC)
    vehicle = RuntimeVehicle(
        "runtime",
        RuntimeState(
            3.0,
            (),
            named={
                "x_ecfc": position.vector.x,
                "y_ecfc": position.vector.y,
                "z_ecfc": position.vector.z,
                "xdot_ecfc": velocity.vector.x,
                "ydot_ecfc": velocity.vector.y,
                "zdot_ecfc": velocity.vector.z,
            },
        ),
        derivative=lambda _: (0.0, 0.0, 0.0, 3.0, 2.0, 1.0),
        dynamics_mode=DynamicsMode.KINEMATIC_6DOF,
        kinematic_state=Kinematic6DofState(3.0, position, velocity, Quaternion.identity()),
        body_rate_provider=lambda _: Vector3(0.1, 0.2, 0.3),
    )
    runtime = observe_vehicle(vehicle)
    standard = runtime.standard.standard_ecef
    assert standard.position_ecef_m == pytest.approx((6_378_140.0, 2.0, 1.0))
    assert standard.velocity_ecef_mps == pytest.approx((6.0, 5.0, 4.0))
    assert standard.acceleration_ecef_mps2 == pytest.approx((3.0, 2.0, 1.0))
    assert standard.ecef_from_body_wxyz == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert standard.angular_velocity_body_radps == pytest.approx((0.1, 0.2, 0.3))
    assert runtime.as_dict()["standard"]["standard_ecef"]["frame_id"] == "ecfc"
    ####


def test_finalized_result_and_session_observation_always_carry_standard_ecef() -> None:
    channels = (
        TrajectoryChannelMetadata(id="position.local.north", quantity="length", unit="m", frame="local_ned"),
        TrajectoryChannelMetadata(id="position.local.east", quantity="length", unit="m", frame="local_ned"),
        TrajectoryChannelMetadata(id="position.geometric.altitude", quantity="length", unit="m", frame="local_ned"),
        TrajectoryChannelMetadata(id="velocity.local.north", quantity="speed", unit="m/s", frame="local_ned"),
        TrajectoryChannelMetadata(id="velocity.local.east", quantity="speed", unit="m/s", frame="local_ned"),
        TrajectoryChannelMetadata(id="velocity.local.vertical", quantity="speed", unit="m/s", frame="local_ned"),
    )
    values = {
        "position.local.north": 1.0,
        "position.local.east": 2.0,
        "position.geometric.altitude": 3.0,
        "velocity.local.north": 4.0,
        "velocity.local.east": 5.0,
        "velocity.local.vertical": 6.0,
    }
    object_result = TrajectoryObject(
        object_id="primary",
        model_id="example",
        realization_id="point-mass",
        name="Example",
        role="primary_vehicle",
        fidelity="point_mass_3dof",
        status="completed",
        active_from_s=0.0,
        active_to_s=0.0,
        channels=channels,
        samples=(TrajectorySample(time_s=0.0, values=values),),
        claim_boundary="test fixture",
    )
    direct_sample = object_result.samples[0]
    assert direct_sample.standard_ecef.frame_id == "ecfc"
    assert direct_sample.standard_ecef.acceleration_ecef_mps2 == pytest.approx((0.0, 0.0, 0.0))
    assert direct_sample.standard_ecef.angular_velocity_body_radps == pytest.approx((0.0, 0.0, 0.0))
    result = MissionCompositionTrajectoryResult(
        provider_id="test.provider",
        provider_version="1.0",
        request_id="request",
        configuration_fingerprint="0" * 64,
        primary_model_id="example",
        primary_object_id="primary",
        status="completed",
        objects=(object_result,),
        claim_boundary="test fixture",
    )

    sample = result.objects[0].samples[0]
    assert sample.values == values
    assert sample.standard_ecef is not None
    assert sample.standard_ecef.position_ecef_m == pytest.approx((6_378_140.0, 2.0, 1.0))
    assert sample.standard_ecef.velocity_ecef_mps == pytest.approx((6.0, 5.0, 4.0))
    assert sample.standard_ecef.acceleration_ecef_mps2 == pytest.approx((0.0, 0.0, 0.0))
    assert sample.standard_ecef.angular_velocity_body_radps == pytest.approx((0.0, 0.0, 0.0))
    assert sample.standard_ecef.orientation_kind == "kinematic_velocity_aligned"

    session = MissionCompositionSessionObservation(
        session_id="session",
        sequence=0,
        time_s=0.0,
        lifecycle="ready",
        values=values,
    )
    assert session.standard_ecef is not None
    assert session.standard_ecef.frame_id == "ecfc"
    ####


def test_standard_ecef_is_a_required_structural_minimum() -> None:
    sample_schema = TrajectorySample.model_json_schema()
    state_schema = MissionCompositionSessionObservation.model_json_schema()
    assert "standard_ecef" in sample_schema["required"]
    assert "standard_ecef" in state_schema["required"]
    standard_schema = sample_schema["$defs"]["StandardEcefState"]
    assert {
        "position_ecef_m",
        "velocity_ecef_mps",
        "acceleration_ecef_mps2",
        "angular_velocity_body_radps",
        "ecef_from_body_wxyz",
    } <= set(standard_schema["required"])
    ####
