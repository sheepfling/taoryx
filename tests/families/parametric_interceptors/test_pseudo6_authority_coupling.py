"""Focused witnesses for pseudo-6DOF response-authority coupling."""

from __future__ import annotations

import math

import pytest
from taoryx_parametric_interceptors import (
    ControlAuthorityEvaluation,
    PointMassWaypoint,
    Pseudo6Kernel,
    Pseudo6State,
    TargetTrackTelemetry,
    interceptor,
    resolve_interceptor,
)

from taoryx.contracts import Frame, FrameVector3, Vector3
from taoryx.runtime.environment_runtime import EnvironmentSample, StaticEnvironmentProvider


def _environment(
    density_kg_m3: float,
    *,
    wind_ecfc_mps: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> StaticEnvironmentProvider:
    return StaticEnvironmentProvider(
        EnvironmentSample(
            density=density_kg_m3,
            pressure=0.0 if density_kg_m3 == 0.0 else 100_000.0,
            temperature=250.0,
            speed_of_sound=300.0,
            wind=FrameVector3(Vector3(*wind_ecfc_mps), Frame.ECFC),
        )
    )
    ####


def _state(
    *,
    time_s: float = 0.0,
    north_velocity_mps: float = 100.0,
    pitch_rad: float = 0.0,
    yaw_rad: float = 0.0,
    yaw_rate_rad_s: float = 0.0,
) -> Pseudo6State:
    return Pseudo6State(
        time_s=time_s,
        north_m=0.0,
        east_m=0.0,
        altitude_m=100.0,
        north_velocity_mps=north_velocity_mps,
        east_velocity_mps=0.0,
        vertical_velocity_mps=0.0,
        roll_rad=0.0,
        pitch_rad=pitch_rad,
        yaw_rad=yaw_rad,
        roll_rate_rad_s=0.0,
        pitch_rate_rad_s=0.0,
        yaw_rate_rad_s=yaw_rate_rad_s,
    )
    ####


def _east_waypoint(*, objective_kind: str = "fixed_waypoint") -> PointMassWaypoint:
    return PointMassWaypoint(
        north_m=0.0,
        east_m=1_000.0,
        altitude_m=100.0,
        capture_radius_m=10.0,
        objective_kind=objective_kind,  # type: ignore[arg-type]
    )
    ####


def test_commanded_support_fraction_is_bounded_and_explicit_for_zero_demand() -> None:
    authority = ControlAuthorityEvaluation(
        configuration="aerodynamic",
        aerodynamic_mps2=10.0,
        thrust_vector_mps2=0.0,
        combined_unclipped_mps2=10.0,
        available_mps2=10.0,
        structural_limit_mps2=20.0,
        structural_limit_active=False,
    )

    assert authority.commanded_support_fraction(20.0) == 0.5
    assert authority.commanded_support_fraction(5.0) == 1.0
    assert authority.commanded_support_fraction(0.0) == 1.0
    with pytest.raises(ValueError, match="finite and nonnegative"):
        authority.commanded_support_fraction(-1.0)
    ####


def test_aerodynamic_response_requires_dynamic_pressure_and_preserves_ballistic_rate() -> None:
    profile = resolve_interceptor(
        interceptor(
            "pseudo6-aerodynamic-authority",
            control_configuration="aerodynamic",
            reference_area_m2=0.1,
            normal_force_coefficient_limit=6.0,
            guidance_archetype="waypoint_pursuit",
            control_bandwidth_class="fast",
        )
    )
    vacuum_kernel = Pseudo6Kernel(
        profile,
        environment=_environment(0.0),
        gravity_acceleration=lambda _: 0.0,
    )
    vacuum = vacuum_kernel.evaluate(_state(), _east_waypoint())

    assert vacuum.lateral_acceleration_command_mps2 > 0.0
    assert math.sqrt(sum(value**2 for value in vacuum.lateral_acceleration_command_vector_mps2)) == pytest.approx(vacuum.lateral_acceleration_command_mps2)
    assert vacuum.lateral_acceleration_achieved_vector_mps2 == (0.0, 0.0, 0.0)
    assert vacuum.lateral_acceleration_achievement_fraction == 0.0
    assert not vacuum.lateral_acceleration_direction_error_valid
    assert vacuum.lateral_acceleration_available_mps2 == 0.0
    assert not vacuum.attitude_response_authority_available
    assert vacuum.attitude_response_command_support_fraction == 0.0
    assert vacuum.attitude_response_authority_limited
    assert vacuum.euler_accelerations_rad_s2 == (0.0, 0.0, 0.0)
    assert vacuum.body_accelerations_rad_s2 == (0.0, 0.0, 0.0)
    assert "aerodynamic_authority_saturation" in vacuum.control_limit_reason
    stationary = vacuum_kernel.advance(_state(), vacuum, 0.1)
    assert (stationary.roll_rad, stationary.pitch_rad, stationary.yaw_rad) == (0.0, 0.0, 0.0)
    assert (stationary.roll_rate_rad_s, stationary.pitch_rate_rad_s, stationary.yaw_rate_rad_s) == (0.0, 0.0, 0.0)

    rotating_state = _state(yaw_rate_rad_s=0.2)
    rotating = vacuum_kernel.evaluate(rotating_state, _east_waypoint())
    assert rotating.euler_accelerations_rad_s2 == (0.0, 0.0, 0.0)
    coasted = vacuum_kernel.advance(rotating_state, rotating, 0.1)
    assert coasted.yaw_rate_rad_s == pytest.approx(0.2)
    assert coasted.yaw_rad == pytest.approx(0.02)

    aerodynamic = Pseudo6Kernel(
        profile,
        environment=_environment(1.0),
        gravity_acceleration=lambda _: 0.0,
    )
    aerodynamic_initial = aerodynamic.evaluate(_state(), _east_waypoint())
    aerodynamic_state = aerodynamic.advance(_state(), aerodynamic_initial, 0.1)
    aerodynamic = aerodynamic.evaluate(aerodynamic_state, _east_waypoint())
    assert aerodynamic.attitude_response_authority_available
    assert 0.0 < aerodynamic.attitude_response_command_support_fraction <= 1.0
    assert any(abs(value) > 0.0 for value in aerodynamic.euler_accelerations_rad_s2)
    assert aerodynamic_initial.lateral_acceleration_command_mps2 > 0.0
    assert aerodynamic_initial.lateral_acceleration_achieved_vector_mps2 == (0.0, 0.0, 0.0)
    assert aerodynamic_initial.lateral_acceleration_achievement_fraction == 0.0
    assert not aerodynamic_initial.lateral_acceleration_direction_error_valid
    assert math.sqrt(sum(value**2 for value in aerodynamic.lateral_acceleration_achieved_vector_mps2)) == pytest.approx(
        aerodynamic.lateral_acceleration_achieved_mps2
    )
    assert aerodynamic.lateral_acceleration_achieved_mps2 > 0.0
    assert aerodynamic.lateral_acceleration_achievement_fraction > 0.0
    assert aerodynamic.lateral_acceleration_direction_error_valid
    ####


def test_thrust_vector_response_exists_in_powered_vacuum_and_stops_at_burnout() -> None:
    profile = resolve_interceptor(
        interceptor(
            "pseudo6-thrust-vector-authority",
            control_configuration="thrust_assisted",
            max_thrust_vector_angle_rad=0.2,
            guidance_archetype="waypoint_pursuit",
            control_bandwidth_class="fast",
        )
    )
    kernel = Pseudo6Kernel(
        profile,
        environment=_environment(0.0),
        gravity_acceleration=lambda _: 0.0,
    )

    powered = kernel.evaluate(_state(time_s=0.0), _east_waypoint())
    assert powered.propulsion_available
    assert powered.thrust_vector_lateral_authority_mps2 > 0.0
    assert powered.attitude_response_authority_available
    assert powered.attitude_response_command_support_fraction > 0.0
    assert any(abs(value) > 0.0 for value in powered.euler_accelerations_rad_s2)

    burnout = kernel.evaluate(_state(time_s=9.0), _east_waypoint())
    assert not burnout.propulsion_available
    assert burnout.thrust_vector_lateral_authority_mps2 == 0.0
    assert not burnout.attitude_response_authority_available
    assert burnout.attitude_response_command_support_fraction == 0.0
    assert burnout.attitude_response_authority_limited
    assert burnout.euler_accelerations_rad_s2 == (0.0, 0.0, 0.0)
    ####


def test_unavailable_target_track_cannot_command_attitude_from_truth_position() -> None:
    profile = resolve_interceptor(
        interceptor(
            "pseudo6-no-target-truth-fallback",
            control_configuration="aerodynamic",
            guidance_archetype="proportional_navigation",
        )
    )
    kernel = Pseudo6Kernel(
        profile,
        environment=_environment(1.0),
        gravity_acceleration=lambda _: 0.0,
    )
    unavailable = TargetTrackTelemetry(
        applicable=True,
        valid=False,
        invalid_reason="outside-range",
    )
    east_truth = kernel.evaluate(
        _state(),
        _east_waypoint(objective_kind="constant_velocity_target"),
        guidance_waypoint=None,
        target_track=unavailable,
    )
    west_truth = kernel.evaluate(
        _state(),
        PointMassWaypoint(
            north_m=0.0,
            east_m=-1_000.0,
            altitude_m=100.0,
            capture_radius_m=10.0,
            objective_kind="constant_velocity_target",
        ),
        guidance_waypoint=None,
        target_track=unavailable,
    )

    assert not east_truth.guidance_available
    assert east_truth.guidance_mode == "target_track_unavailable"
    assert east_truth.lateral_acceleration_command_mps2 == 0.0
    assert east_truth.lateral_acceleration_command_vector_mps2 == (0.0, 0.0, 0.0)
    assert east_truth.lateral_acceleration_achieved_vector_mps2 == (0.0, 0.0, 0.0)
    assert east_truth.lateral_acceleration_achievement_fraction == 1.0
    assert not east_truth.lateral_acceleration_direction_error_valid
    assert east_truth.lateral_acceleration_direction_error_rad == 0.0
    assert east_truth.attitude_command_rad == (0.0, 0.0, 0.0)
    assert west_truth.attitude_command_rad == east_truth.attitude_command_rad
    assert west_truth.euler_accelerations_rad_s2 == east_truth.euler_accelerations_rad_s2
    ####


def test_pseudo6_reports_air_relative_flow_in_forward_right_down_body_axes() -> None:
    profile = resolve_interceptor(
        interceptor(
            "pseudo6-body-flow-angles",
            control_configuration="aerodynamic",
        )
    )
    pitch = math.radians(10.0)
    pitched = Pseudo6Kernel(
        profile,
        environment=_environment(1.0, wind_ecfc_mps=(0.0, 0.0, 20.0)),
        gravity_acceleration=lambda _: 0.0,
    ).evaluate(_state(pitch_rad=pitch), _east_waypoint())

    assert pitched.airspeed_mps == pytest.approx(80.0)
    assert pitched.flow_angles_valid
    assert pitched.air_relative_velocity_body_mps == pytest.approx((80.0 * math.cos(pitch), 0.0, 80.0 * math.sin(pitch)))
    assert pitched.angle_of_attack_rad == pytest.approx(pitch)
    assert pitched.sideslip_angle_rad == pytest.approx(0.0)

    yaw = math.radians(10.0)
    yawed = Pseudo6Kernel(
        profile,
        environment=_environment(1.0),
        gravity_acceleration=lambda _: 0.0,
    ).evaluate(_state(yaw_rad=yaw), _east_waypoint())
    assert yawed.air_relative_velocity_body_mps == pytest.approx((100.0 * math.cos(yaw), -100.0 * math.sin(yaw), 0.0))
    assert yawed.angle_of_attack_rad == pytest.approx(0.0)
    assert yawed.sideslip_angle_rad == pytest.approx(-yaw)

    stopped = Pseudo6Kernel(
        profile,
        environment=_environment(1.0),
        gravity_acceleration=lambda _: 0.0,
    ).evaluate(_state(north_velocity_mps=0.0), _east_waypoint())
    assert not stopped.flow_angles_valid
    assert stopped.air_relative_velocity_body_mps == (0.0, 0.0, 0.0)
    assert stopped.angle_of_attack_rad == 0.0
    assert stopped.sideslip_angle_rad == 0.0
    ####
