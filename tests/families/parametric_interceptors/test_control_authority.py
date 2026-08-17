"""Focused resolver-to-Composition witnesses for reduced-order control authority."""

from __future__ import annotations

import math

import pytest
from taoryx_parametric_interceptors import (
    CONTROL_ALLOCATION_POLICIES,
    ParametricInterceptorMissionCompositionProvider,
    PointMassKernel,
    PointMassState,
    PointMassWaypoint,
    Pseudo6Kernel,
    Pseudo6State,
    ValueOrigin,
    aim9x_block2_profile,
    aim120_c5_c7_profile,
    allocate_control_authority,
    calibrated,
    canonicalize_interceptor_value,
    evaluate_control_authority,
    interceptor,
    load_interceptor_profile,
    resolve_interceptor,
)

from taoryx.contracts import Frame, FrameVector3, Vector3
from taoryx.runtime.environment_runtime import EnvironmentSample, StaticEnvironmentProvider
from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunRequest,
    MissionCompositionTrajectoryResponse,
)


def _static_environment(density_kg_m3: float = 1.0) -> StaticEnvironmentProvider:
    return StaticEnvironmentProvider(
        EnvironmentSample(
            density=density_kg_m3,
            pressure=100_000.0 if density_kg_m3 > 0.0 else 0.0,
            temperature=250.0,
            speed_of_sound=300.0,
            wind=FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
        )
    )
    ####


def _mixed_profile():
    return resolve_interceptor(
        interceptor(
            "mixed-authority-witness",
            launch_mass_kg=100.0,
            propellant_fraction=0.4,
            reference_area_m2=0.01,
            maneuverability_class="moderate",
            control_configuration="mixed",
            normal_force_coefficient_limit=2.0,
            max_thrust_vector_angle_rad=0.1,
            guidance_archetype="waypoint_pursuit",
            control_bandwidth_class="fast",
        )
    )
    ####


def test_force_authority_components_add_then_clip_to_the_structural_envelope() -> None:
    evaluation = evaluate_control_authority(
        configuration="mixed",
        dynamic_pressure_pa=1_000.0,
        reference_area_m2=0.2,
        normal_force_coefficient_limit=5.0,
        thrust_n=1_000.0,
        mass_kg=100.0,
        max_thrust_vector_angle_rad=math.radians(30.0),
        structural_limit_mps2=12.0,
    )

    assert evaluation.aerodynamic_mps2 == pytest.approx(10.0)
    assert evaluation.thrust_vector_mps2 == pytest.approx(5.0)
    assert evaluation.combined_unclipped_mps2 == pytest.approx(15.0)
    assert evaluation.available_mps2 == pytest.approx(12.0)
    assert evaluation.structural_limit_active
    assert evaluation.saturation_reason == "combined_authority_saturation"

    allocation = allocate_control_authority(
        evaluation,
        requested_mps2=12.0,
        policy="aerodynamic_first",
        thrust_n=1_000.0,
        mass_kg=100.0,
    )
    assert allocation.aerodynamic_achieved_mps2 == pytest.approx(10.0)
    assert allocation.thrust_vector_achieved_mps2 == pytest.approx(2.0)
    assert allocation.achieved_mps2 == pytest.approx(12.0)
    assert allocation.thrust_vector_angle_rad == pytest.approx(math.asin(0.2))
    assert allocation.axial_thrust_n == pytest.approx(math.sqrt(1_000.0**2 - 200.0**2))

    aerodynamic_only_demand = allocate_control_authority(
        evaluation,
        requested_mps2=4.0,
        policy="aerodynamic_first",
        thrust_n=1_000.0,
        mass_kg=100.0,
    )
    assert aerodynamic_only_demand.aerodynamic_achieved_mps2 == 4.0
    assert aerodynamic_only_demand.thrust_vector_achieved_mps2 == 0.0
    assert aerodynamic_only_demand.axial_thrust_n == 1_000.0

    thrust_first = allocate_control_authority(
        evaluation,
        requested_mps2=12.0,
        policy="thrust_vector_first",
        thrust_n=1_000.0,
        mass_kg=100.0,
    )
    assert thrust_first.aerodynamic_achieved_mps2 == pytest.approx(7.0)
    assert thrust_first.thrust_vector_achieved_mps2 == pytest.approx(5.0)
    assert thrust_first.axial_thrust_n == pytest.approx(math.sqrt(1_000.0**2 - 500.0**2))

    proportional = allocate_control_authority(
        evaluation,
        requested_mps2=12.0,
        policy="proportional",
        thrust_n=1_000.0,
        mass_kg=100.0,
    )
    assert proportional.aerodynamic_achieved_mps2 == pytest.approx(8.0)
    assert proportional.thrust_vector_achieved_mps2 == pytest.approx(4.0)
    assert proportional.axial_thrust_n == pytest.approx(math.sqrt(1_000.0**2 - 400.0**2))

    coast = evaluate_control_authority(
        configuration="thrust_assisted",
        dynamic_pressure_pa=1_000.0,
        reference_area_m2=0.2,
        normal_force_coefficient_limit=0.0,
        thrust_n=0.0,
        mass_kg=100.0,
        max_thrust_vector_angle_rad=math.radians(30.0),
        structural_limit_mps2=12.0,
    )
    assert coast.available_mps2 == 0.0
    assert coast.thrust_vector_mps2 == 0.0
    ####


def test_point_mass_vector_reports_unbounded_command_and_zero_realization_without_authority() -> None:
    profile = resolve_interceptor(
        interceptor(
            "point-mass-vector-no-authority",
            control_configuration="aerodynamic",
            reference_area_m2=0.1,
            normal_force_coefficient_limit=6.0,
            guidance_archetype="waypoint_pursuit",
        )
    )
    evaluation = (
        PointMassKernel(
            profile,
            environment=_static_environment(0.0),
            gravity_acceleration=lambda _: 0.0,
        )
        .evaluate(
            PointMassState(
                time_s=0.0,
                north_m=0.0,
                east_m=0.0,
                altitude_m=100.0,
                north_velocity_mps=100.0,
                east_velocity_mps=0.0,
                vertical_velocity_mps=0.0,
            ),
            PointMassWaypoint(
                north_m=0.0,
                east_m=1_000.0,
                altitude_m=100.0,
                capture_radius_m=10.0,
            ),
        )
        .sample
    )

    assert evaluation.lateral_acceleration_command_mps2 > 0.0
    assert evaluation.lateral_acceleration_command_vector_mps2[0] == pytest.approx(0.0)
    assert evaluation.lateral_acceleration_command_vector_mps2[1] > 0.0
    assert math.sqrt(sum(value**2 for value in evaluation.lateral_acceleration_command_vector_mps2)) == pytest.approx(
        evaluation.lateral_acceleration_command_mps2
    )
    assert evaluation.lateral_acceleration_available_mps2 == 0.0
    assert evaluation.lateral_acceleration_achieved_mps2 == 0.0
    assert evaluation.lateral_acceleration_achieved_vector_mps2 == (0.0, 0.0, 0.0)
    assert evaluation.lateral_acceleration_achievement_fraction == 0.0
    assert not evaluation.lateral_acceleration_direction_error_valid
    assert evaluation.lateral_acceleration_direction_error_rad == 0.0
    ####


def test_witnesses_resolve_control_features_without_laundering_them_into_controller_claims() -> None:
    aim9x = resolve_interceptor(aim9x_block2_profile())
    aim120 = resolve_interceptor(aim120_c5_c7_profile())

    assert aim9x.text("control_configuration") == "mixed"
    assert aim9x.parameters["control_configuration"].origin is ValueOrigin.INFERRED
    assert aim9x.parameters["control_configuration"].source_record_ids == (
        "int:us-aim9x-block2",
        "source:navair-aim9x-public-description",
    )
    assert aim9x.number("normal_force_coefficient_limit") == 10.0
    assert aim9x.number("max_thrust_vector_angle_rad") == pytest.approx(math.radians(15.0))
    assert aim9x.text("control_allocation_policy") == "aerodynamic_first"
    assert aim9x.parameters["control_allocation_policy"].origin is ValueOrigin.ARCHETYPE_ASSUMPTION

    assert aim120.text("control_configuration") == "aerodynamic"
    assert aim120.parameters["control_configuration"].origin is ValueOrigin.ARCHETYPE_ASSUMPTION
    assert aim120.number("normal_force_coefficient_limit") == 6.0
    assert aim120.number("max_thrust_vector_angle_rad") == 0.0
    ####


def test_explicit_authority_is_easy_to_author_and_rejects_incompatible_fields() -> None:
    example = resolve_interceptor(load_interceptor_profile("examples/parametric_interceptors/custom_control_authority_sam.yaml"))
    assert example.text("control_configuration") == "mixed"
    assert example.text("control_allocation_policy") == "proportional"
    assert example.number("normal_force_coefficient_limit") == 8.0
    assert example.number("max_thrust_vector_angle_rad") == pytest.approx(0.1745329252)

    tuned = resolve_interceptor(
        interceptor(
            "calibrated-authority",
            control_configuration="aerodynamic",
            maneuverability_class="moderate",
            maneuverability_scale=calibrated(1.2, unit="1", method="focused lateral-response fit"),
        )
    )
    assert tuned.number("normal_force_coefficient_limit") == pytest.approx(7.2)
    assert tuned.parameters["normal_force_coefficient_limit"].origin is ValueOrigin.CALIBRATED
    assert tuned.number("max_lateral_acceleration_mps2") == pytest.approx(10.0 * 9.80665)
    assert tuned.parameters["max_lateral_acceleration_mps2"].depends_on == ("maneuverability_class",)

    with pytest.raises(ValueError, match="aerodynamic control cannot define max_thrust_vector_angle_rad"):
        resolve_interceptor(
            interceptor(
                "invalid-aerodynamic-authority",
                control_configuration="aerodynamic",
                max_thrust_vector_angle_rad=0.1,
            )
        )
    with pytest.raises(ValueError, match="thrust_assisted control cannot define normal_force_coefficient_limit"):
        resolve_interceptor(
            interceptor(
                "invalid-thrust-authority",
                control_configuration="thrust_assisted",
                normal_force_coefficient_limit=4.0,
            )
        )
    with pytest.raises(ValueError, match="unsupported control_allocation_policy"):
        resolve_interceptor(
            interceptor(
                "invalid-allocation-policy",
                control_allocation_policy="opaque_allocator",
            )
        )

    for policy in CONTROL_ALLOCATION_POLICIES:
        resolved = resolve_interceptor(
            interceptor(
                f"valid-{policy}",
                control_configuration="mixed",
                control_allocation_policy=policy,
                normal_force_coefficient_limit=4.0,
                max_thrust_vector_angle_rad=0.1,
            )
        )
        assert resolved.text("control_allocation_policy") == policy

    converted = canonicalize_interceptor_value("max_thrust_vector_angle_rad", 10.0, "deg")
    assert converted.canonical_value == pytest.approx(math.radians(10.0))
    ####


@pytest.mark.parametrize("time_s", (0.0, 9.0))
def test_point_mass_and_pseudo6_share_instantaneous_authority_through_burnout(time_s: float) -> None:
    profile = _mixed_profile()
    environment = _static_environment()
    point = PointMassKernel(profile, environment=environment, gravity_acceleration=lambda _: 0.0)
    pseudo = Pseudo6Kernel(profile, environment=environment, gravity_acceleration=lambda _: 0.0)
    waypoint = PointMassWaypoint(
        north_m=0.0,
        east_m=1_000.0,
        altitude_m=100.0,
        capture_radius_m=10.0,
    )
    point_evaluation = point.evaluate(
        PointMassState(
            time_s=time_s,
            north_m=0.0,
            east_m=0.0,
            altitude_m=100.0,
            north_velocity_mps=100.0,
            east_velocity_mps=0.0,
            vertical_velocity_mps=0.0,
        ),
        waypoint,
    ).sample
    pseudo_evaluation = pseudo.evaluate(
        Pseudo6State(
            time_s=time_s,
            north_m=0.0,
            east_m=0.0,
            altitude_m=100.0,
            north_velocity_mps=100.0,
            east_velocity_mps=0.0,
            vertical_velocity_mps=0.0,
            roll_rad=0.0,
            pitch_rad=0.0,
            yaw_rad=math.pi / 2.0,
            roll_rate_rad_s=0.0,
            pitch_rate_rad_s=0.0,
            yaw_rate_rad_s=0.0,
        ),
        waypoint,
    )

    assert point_evaluation.control_configuration == "mixed"
    assert point_evaluation.aerodynamic_lateral_authority_mps2 == pytest.approx(pseudo_evaluation.aerodynamic_lateral_authority_mps2)
    assert point_evaluation.thrust_vector_lateral_authority_mps2 == pytest.approx(pseudo_evaluation.thrust_vector_lateral_authority_mps2)
    assert point_evaluation.lateral_acceleration_available_mps2 == pytest.approx(pseudo_evaluation.lateral_acceleration_available_mps2)
    assert point_evaluation.lateral_acceleration_achieved_mps2 == pytest.approx(pseudo_evaluation.lateral_acceleration_achieved_mps2)
    assert math.sqrt(sum(value**2 for value in point_evaluation.lateral_acceleration_command_vector_mps2)) == pytest.approx(
        point_evaluation.lateral_acceleration_command_mps2
    )
    assert math.sqrt(sum(value**2 for value in point_evaluation.lateral_acceleration_achieved_vector_mps2)) == pytest.approx(
        point_evaluation.lateral_acceleration_achieved_mps2
    )
    assert math.sqrt(sum(value**2 for value in pseudo_evaluation.lateral_acceleration_command_vector_mps2)) == pytest.approx(
        pseudo_evaluation.lateral_acceleration_command_mps2
    )
    assert math.sqrt(sum(value**2 for value in pseudo_evaluation.lateral_acceleration_achieved_vector_mps2)) == pytest.approx(
        pseudo_evaluation.lateral_acceleration_achieved_mps2
    )
    assert point_evaluation.lateral_acceleration_command_vector_mps2[0] == pytest.approx(0.0)
    assert point_evaluation.lateral_acceleration_achieved_vector_mps2[0] == pytest.approx(0.0)
    assert point_evaluation.lateral_acceleration_achievement_fraction == pytest.approx(
        point_evaluation.lateral_acceleration_achieved_mps2 / point_evaluation.lateral_acceleration_command_mps2
    )
    assert point_evaluation.lateral_acceleration_direction_error_valid
    assert point_evaluation.lateral_acceleration_direction_error_rad == pytest.approx(0.0)
    assert pseudo_evaluation.lateral_acceleration_achievement_fraction == pytest.approx(
        pseudo_evaluation.lateral_acceleration_achieved_mps2 / pseudo_evaluation.lateral_acceleration_command_mps2
    )
    assert pseudo_evaluation.lateral_acceleration_direction_error_valid
    assert pseudo_evaluation.lateral_acceleration_direction_error_rad == pytest.approx(0.0)
    assert point_evaluation.aerodynamic_lateral_acceleration_achieved_mps2 == pytest.approx(pseudo_evaluation.aerodynamic_lateral_acceleration_achieved_mps2)
    assert point_evaluation.thrust_vector_lateral_acceleration_achieved_mps2 == pytest.approx(
        pseudo_evaluation.thrust_vector_lateral_acceleration_achieved_mps2
    )
    assert point_evaluation.thrust_vector_angle_achieved_rad == pytest.approx(pseudo_evaluation.thrust_vector_angle_achieved_rad)
    assert point_evaluation.axial_thrust_n == pytest.approx(pseudo_evaluation.axial_thrust_n)
    assert point_evaluation.control_allocation_policy == "aerodynamic_first"
    assert point_evaluation.base_drag_n == pytest.approx(pseudo_evaluation.base_drag_n)
    assert point_evaluation.maneuver_drag_n == pytest.approx(pseudo_evaluation.maneuver_drag_n)
    assert point_evaluation.drag_n == pytest.approx(pseudo_evaluation.drag_n)
    assert point_evaluation.drag_n == pytest.approx(point_evaluation.base_drag_n + point_evaluation.maneuver_drag_n)
    assert point_evaluation.maneuver_drag_n > 0.0
    assert point_evaluation.total_drag_coefficient == pytest.approx(point_evaluation.drag_coefficient + point_evaluation.maneuver_drag_coefficient)
    assert point_evaluation.lateral_acceleration_achieved_mps2 <= point_evaluation.lateral_acceleration_available_mps2
    assert pseudo_evaluation.lateral_acceleration_achieved_mps2 <= pseudo_evaluation.lateral_acceleration_available_mps2
    assert "combined_authority_saturation" in point_evaluation.control_limit_reason
    assert "combined_authority_saturation" in pseudo_evaluation.control_limit_reason
    if time_s == 0.0:
        assert point_evaluation.thrust_vector_lateral_authority_mps2 > 0.0
        assert point_evaluation.thrust_vector_lateral_acceleration_achieved_mps2 > 0.0
        lateral_thrust_n = point_evaluation.thrust_vector_lateral_acceleration_achieved_mps2 * point_evaluation.mass_kg
        assert math.hypot(point_evaluation.axial_thrust_n, lateral_thrust_n) == pytest.approx(point_evaluation.thrust_n)
        assert point_evaluation.axial_thrust_n < point_evaluation.thrust_n
        assert point_evaluation.propulsion_available
    else:
        assert point_evaluation.thrust_vector_lateral_authority_mps2 == 0.0
        assert point_evaluation.thrust_vector_lateral_acceleration_achieved_mps2 == 0.0
        assert point_evaluation.thrust_vector_angle_achieved_rad == 0.0
        assert point_evaluation.axial_thrust_n == point_evaluation.thrust_n == 0.0
        assert not point_evaluation.propulsion_available
        assert point_evaluation.aerodynamic_lateral_authority_mps2 > 0.0
    ####


@pytest.mark.parametrize("policy", CONTROL_ALLOCATION_POLICIES)
def test_allocation_policy_flows_through_both_runtime_tiers(policy: str) -> None:
    profile = resolve_interceptor(
        interceptor(
            f"runtime-{policy}",
            launch_mass_kg=100.0,
            propellant_fraction=0.4,
            reference_area_m2=0.01,
            maneuverability_class="moderate",
            control_configuration="mixed",
            control_allocation_policy=policy,
            normal_force_coefficient_limit=2.0,
            max_thrust_vector_angle_rad=0.1,
            guidance_archetype="waypoint_pursuit",
            control_bandwidth_class="slow",
        )
    )
    environment = _static_environment()
    waypoint = PointMassWaypoint(
        north_m=1_000.0,
        east_m=10.0,
        altitude_m=100.0,
        capture_radius_m=1.0,
    )
    point = (
        PointMassKernel(profile, environment=environment, gravity_acceleration=lambda _: 0.0)
        .evaluate(
            PointMassState(
                time_s=0.0,
                north_m=0.0,
                east_m=0.0,
                altitude_m=100.0,
                north_velocity_mps=100.0,
                east_velocity_mps=0.0,
                vertical_velocity_mps=0.0,
            ),
            waypoint,
        )
        .sample
    )
    pseudo = Pseudo6Kernel(profile, environment=environment, gravity_acceleration=lambda _: 0.0).evaluate(
        Pseudo6State(
            time_s=0.0,
            north_m=0.0,
            east_m=0.0,
            altitude_m=100.0,
            north_velocity_mps=100.0,
            east_velocity_mps=0.0,
            vertical_velocity_mps=0.0,
            roll_rad=0.0,
            pitch_rad=0.0,
            yaw_rad=math.atan2(10.0, 1_000.0),
            roll_rate_rad_s=0.0,
            pitch_rate_rad_s=0.0,
            yaw_rate_rad_s=0.0,
        ),
        waypoint,
    )

    assert point.control_allocation_policy == pseudo.control_allocation_policy == policy
    assert point.lateral_acceleration_achieved_mps2 == pytest.approx(pseudo.lateral_acceleration_achieved_mps2)
    assert point.aerodynamic_lateral_acceleration_achieved_mps2 == pytest.approx(pseudo.aerodynamic_lateral_acceleration_achieved_mps2)
    assert point.thrust_vector_lateral_acceleration_achieved_mps2 == pytest.approx(pseudo.thrust_vector_lateral_acceleration_achieved_mps2)
    if policy == "aerodynamic_first":
        assert point.thrust_vector_lateral_acceleration_achieved_mps2 == 0.0
    elif policy == "thrust_vector_first":
        assert point.aerodynamic_lateral_acceleration_achieved_mps2 == 0.0
    else:
        assert point.aerodynamic_lateral_acceleration_achieved_mps2 > 0.0
        assert point.thrust_vector_lateral_acceleration_achieved_mps2 > 0.0
    ####


def test_composition_advertises_structural_and_instantaneous_authority_separately() -> None:
    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(aim9x_block2_profile())
    model = provider.model("aim9x-block2")
    properties = {item.id: item for item in model.presentation.properties}
    outputs = {item.id: item for item in model.output_schema.telemetry_channels}
    frames = {item.id: item for item in model.reference_frames}

    assert properties["control_configuration"].value == "mixed"
    assert properties["control_configuration"].provenance.startswith("origin=inferred")
    assert properties["control_allocation_policy"].value == "aerodynamic_first"
    assert properties["control_allocation_policy.supported"].value == ", ".join(CONTROL_ALLOCATION_POLICIES)
    assert outputs["guidance.lateral_acceleration.limit"].canonical_unit == "m/s^2"
    assert "structural maneuver envelope" in outputs["guidance.lateral_acceleration.limit"].description
    vector_ids = (
        "guidance.lateral_acceleration.commanded.local.north",
        "guidance.lateral_acceleration.commanded.local.east",
        "guidance.lateral_acceleration.commanded.local.vertical",
        "guidance.lateral_acceleration.achieved.local.north",
        "guidance.lateral_acceleration.achieved.local.east",
        "guidance.lateral_acceleration.achieved.local.vertical",
    )
    assert all(outputs[identifier].canonical_unit == "m/s^2" for identifier in vector_ids)
    assert all(outputs[identifier].frame == "local_neu" for identifier in vector_ids)
    assert frames["local_neu"].axes == ("north", "east", "up")
    assert frames["local_neu"].handedness == "left"
    assert "unbounded guidance-law" in outputs[vector_ids[0]].description
    assert "actually realized" in outputs[vector_ids[3]].description
    assert outputs["guidance.lateral_acceleration.achievement_fraction"].canonical_unit == "1"
    assert "[0, 1]" in outputs["guidance.lateral_acceleration.achievement_fraction"].description
    assert outputs["guidance.lateral_acceleration.direction_error.valid"].data_type == "boolean"
    assert outputs["guidance.lateral_acceleration.direction_error"].canonical_unit == "rad"
    assert "meaningful only" in outputs["guidance.lateral_acceleration.direction_error"].description
    assert outputs["control.authority.available"].canonical_unit == "m/s^2"
    assert outputs["control.authority.configuration"].data_type == "string"
    assert outputs["control.authority.structural_limit_active"].data_type == "boolean"
    assert "q*S*Cn/m" in outputs["control.authority.aerodynamic"].description
    assert "thrust*sin" in outputs["control.authority.thrust_vector"].description
    assert outputs["propulsion.thrust.axial"].canonical_unit == "N"
    assert outputs["control.allocation.policy"].data_type == "string"
    assert outputs["control.allocation.thrust_vector_angle"].canonical_unit == "rad"

    prepared = provider.validate_configuration(
        provider.configuration(
            "aim9x-block2",
            configuration_id="aim9x-authority-composition",
            launch_altitude_m=100.0,
            launch_speed_mps=100.0,
            launch_heading_deg=0.0,
            launch_flight_path_deg=0.0,
            navigation_waypoint_north_command=0.0,
            navigation_waypoint_east_command=1_000.0,
            navigation_waypoint_altitude_command=100.0,
            runtime_duration_s=0.1,
            runtime_time_step_s=0.1,
        )
    )
    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id="aim9x-authority-composition",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )
    assert isinstance(response, MissionCompositionTrajectoryResponse)
    values = response.result.objects[0].samples[0].values
    assert values["control.authority.configuration"] == "mixed"
    assert values["control.allocation.policy"] == "aerodynamic_first"
    assert float(values["propulsion.thrust.axial"]) <= float(values["propulsion.thrust"])
    assert float(values["control.allocation.aerodynamic_achieved"]) + float(values["control.allocation.thrust_vector_achieved"]) == pytest.approx(
        float(values["guidance.lateral_acceleration.achieved"])
    )
    assert float(values["aerodynamics.drag.base"]) + float(values["aerodynamics.drag.maneuver"]) == pytest.approx(float(values["aerodynamics.drag"]))
    assert float(values["control.authority.available"]) <= float(values["guidance.lateral_acceleration.limit"])
    assert 0.0 <= float(values["control.authority.utilization"]) <= 1.0
    commanded_vector = tuple(float(values[identifier]) for identifier in vector_ids[:3])
    achieved_vector = tuple(float(values[identifier]) for identifier in vector_ids[3:])
    assert math.sqrt(sum(value**2 for value in commanded_vector)) == pytest.approx(float(values["guidance.lateral_acceleration.commanded"]))
    assert math.sqrt(sum(value**2 for value in achieved_vector)) == pytest.approx(float(values["guidance.lateral_acceleration.achieved"]))
    assert float(values["guidance.lateral_acceleration.achievement_fraction"]) == pytest.approx(
        float(values["guidance.lateral_acceleration.achieved"]) / float(values["guidance.lateral_acceleration.commanded"])
    )
    assert values["guidance.lateral_acceleration.direction_error.valid"] is True
    assert float(values["guidance.lateral_acceleration.direction_error"]) == pytest.approx(0.0)
    ####
