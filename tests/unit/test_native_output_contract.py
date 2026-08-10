"""Fast structural checks for native batch-output advertisements."""

from __future__ import annotations

from taoryx.trajectory.native_output_contract import (
    NativeOutputBinding,
    extract_native_channel,
    native_output_bindings,
    native_output_channel_metadata,
)


def _applicable_ids(model_id: str, fidelity: str, mission: str) -> set[str]:
    """Return every exact native channel selected by one batch endpoint."""

    return {
        item.id
        for item in native_output_bindings(model_id)
        if fidelity in item.fidelities and (not item.mission_templates or mission in item.mission_templates)
    }
    ####


def test_native_output_maps_have_unique_public_ids_and_legal_core_groups() -> None:
    """A source map must be representable by the public output schema."""

    for model_id in (
        "skywalker_x8",
        "b747",
        "a320_openap_3dof",
        "f16_s119",
        "hummingbird",
        "reference_nesc_two_stage_rocket",
        "x15",
        "hl20_mod_k",
        "tumbling_body",
    ):
        bindings: tuple[NativeOutputBinding, ...] = native_output_bindings(model_id)
        assert len({item.id for item in bindings}) == len(bindings)
        assert all(item.telemetry_group is None for item in bindings if item.channel_class == "core_state")
        assert len(native_output_channel_metadata(model_id)) == len(bindings)
    ####


def test_f16_and_hummingbird_physical_screens_advertise_emitted_state_controls_and_resources() -> None:
    """Physical control evidence reaches the common result instead of a sidecar only."""

    f16_common = {
        "velocity.body.x",
        "velocity.body.y",
        "velocity.body.z",
        "angular_rate.body.x",
        "angular_rate.body.y",
        "angular_rate.body.z",
        "mass.total",
        "control.wrench.requested.force.x",
        "control.wrench.achieved.force.x",
        "control.wrench.residual.force.x",
        "control.wrench.residual.moment.x",
        "control.wrench.residual.moment.y",
        "control.wrench.residual.moment.z",
        "diagnostics.allocation_residual_norm",
        "diagnostics.saturation_count",
    }
    for fidelity, mission in (
        ("rigid_body_6dof_direct_wrench", "f16_local_physical_control_screen_v1"),
        ("rigid_body_6dof_surface_allocated", "f16_local_physical_control_screen_v1"),
        ("rigid_body_6dof_surface_allocated", "f16_local_physical_surface_lqi_screen_v1"),
        ("rigid_body_6dof_surface_allocated", "f16_local_physical_surface_lqr_schedule_interior_screen_v1"),
        ("rigid_body_6dof_surface_allocated", "f16_local_physical_surface_lqr_schedule_transition_screen_v1"),
    ):
        assert f16_common <= _applicable_ids("f16_s119", fidelity, mission)

    hummingbird_ids = _applicable_ids(
        "hummingbird",
        "rigid_body_6dof_surface_allocated",
        "hummingbird_local_individual_rotor_lqi_screen_v1",
    )
    assert {
        "attitude.local.roll_error",
        "attitude.local.pitch_error",
        "attitude.local.yaw_error",
        "angular_rate.body.x",
        "angular_rate.body.y",
        "angular_rate.body.z",
        "mass.total",
        "control.wrench.requested.moment.x",
        "control.wrench.achieved.moment.x",
        "diagnostics.allocation_residual_norm",
        "diagnostics.saturation_count",
        "control.lqi.integral_error.roll",
        "actuator.rotor.1.speed.actual",
        "actuator.rotor.4.speed.actual",
    } <= hummingbird_ids
    horizontal_hummingbird_ids = _applicable_ids(
        "hummingbird",
        "rigid_body_6dof_surface_allocated",
        "hummingbird_local_horizontal_translation_lqi_screen_v1",
    )
    assert {
        "position.local.x",
        "position.local.y",
        "position.local.z",
        "velocity.local.x",
        "velocity.local.y",
        "velocity.local.z",
        "control.lqi.integral_error.yaw",
        "actuator.rotor.4.speed.actual",
    } <= horizontal_hummingbird_ids
    vertical_hummingbird_ids = _applicable_ids(
        "hummingbird",
        "rigid_body_6dof_surface_allocated",
        "hummingbird_local_vertical_translation_lqi_screen_v1",
    )
    assert {
        "position.local.z",
        "velocity.local.z",
        "control.wrench.requested.force.z",
        "control.wrench.achieved.force.z",
        "control.wrench.residual.force.z",
        "control.wrench.requested.moment.x",
        "control.wrench.achieved.moment.z",
        "control.lqi.integral_error.vertical_speed",
        "actuator.rotor.4.speed.actual",
    } <= vertical_hummingbird_ids
    hummingbird_direct_ids = _applicable_ids(
        "hummingbird",
        "rigid_body_6dof_direct_wrench",
        "hummingbird_local_direct_wrench_screen_v1",
    )
    assert {
        "mass.total",
        "control.wrench.residual.force.x",
        "control.wrench.residual.force.y",
        "control.wrench.residual.force.z",
        "control.wrench.residual.moment.x",
        "control.wrench.residual.moment.y",
        "control.wrench.residual.moment.z",
        "control.wrench.residual_norm",
        "control.feedback_norm",
    } <= hummingbird_direct_ids
    ####


def test_a320_native_coordinate_lqi_screen_advertises_its_named_control_trace() -> None:
    """A named-control LQI result must not hide its source controls in a sidecar."""

    ids = _applicable_ids(
        "a320_openap_3dof",
        "pseudo_6dof",
        "a320_local_native_coordinate_lqi_screen_v1",
    )
    assert {
        "position.local.x",
        "position.local.y",
        "position.local.z",
        "velocity.speed",
        "attitude.local.roll_error",
        "angular_rate.body.x",
        "angular_rate.body.y",
        "angular_rate.body.z",
        "mass.total",
        "aerodynamics.angle_of_attack",
        "aerodynamics.sideslip",
        "control.native.aileron.requested",
        "control.native.aileron.applied",
        "control.native.elevator.requested",
        "control.native.elevator.applied",
        "control.native.rudder.requested",
        "control.native.rudder.applied",
        "control.lqi.integral_error.roll",
        "diagnostics.feedback_error_norm",
        "diagnostics.saturation_count",
    } <= ids
    ####


def test_x15_direct_wrench_screens_advertise_their_source_release_mass() -> None:
    """The bounded X-15 screen retains its fixed source mass as a resource."""

    for mission in (
        "x15_local_direct_wrench_screen_v1",
        "x15_local_direct_wrench_lqi_screen_v1",
    ):
        ids = _applicable_ids("x15", "rigid_body_6dof_direct_wrench", mission)
        assert "mass.total" in ids
        assert {
            "control.wrench.residual.force.x",
            "control.wrench.residual.force.y",
            "control.wrench.residual.force.z",
            "control.wrench.residual.moment.x",
            "control.wrench.residual.moment.y",
            "control.wrench.residual.moment.z",
            "control.wrench.residual_norm",
            "control.feedback_norm",
        } <= ids
    ####


def test_hl20_direct_wrench_screens_advertise_their_fixed_source_mass() -> None:
    """Both local HL-20 screens retain their source mass as a resource."""

    for mission in (
        "hl20_local_direct_wrench_screen_v1",
        "hl20_local_direct_wrench_lqi_screen_v1",
    ):
        ids = _applicable_ids("hl20_mod_k", "rigid_body_6dof_direct_wrench", mission)
        assert "mass.total" in ids
        assert {
            "control.wrench.residual.force.x",
            "control.wrench.residual.force.y",
            "control.wrench.residual.force.z",
            "control.wrench.residual.moment.x",
            "control.wrench.residual.moment.y",
            "control.wrench.residual.moment.z",
            "control.wrench.residual_norm",
            "control.feedback_norm",
        } <= ids
    ####


def test_hl20_source_surface_authority_screen_advertises_actual_named_surfaces_and_boundaries() -> None:
    """The source allocation proof is visible through common output metadata."""

    ids = _applicable_ids(
        "hl20_mod_k",
        "rigid_body_6dof_surface_allocated",
        "hl20_source_surface_pitch_authority_screen_v1",
    )
    assert {
        "mass.total",
        "aerodynamics.pitch_coefficient",
        "control.pitch_moment.requested",
        "control.pitch_moment.achieved",
        "control.pitch_moment.residual",
        "control.allocation.residual_norm",
        "control.allocation.status",
        "control.source_effectiveness_rank",
        "actuator.surface.upper_left_body_flap.actual",
        "actuator.surface.lower_right_body_flap.actual",
        "actuator.surface.left_wing_flap.actual",
        "actuator.surface.rudder.actual",
    } <= ids
    ####


def test_x15_source_surface_authority_screen_advertises_actual_named_surfaces_and_moments() -> None:
    """The X-15 source allocation proof is visible through common output metadata."""

    ids = _applicable_ids(
        "x15",
        "rigid_body_6dof_surface_allocated",
        "x15_source_surface_authority_screen_v1",
    )
    assert {
        "mass.total",
        "source_fixture.mach",
        "source_fixture.alpha",
        "source_fixture.beta",
        "control.moment.x.requested",
        "control.moment.y.achieved",
        "control.moment.z.achieved",
        "control.allocation.residual_norm",
        "control.allocation.status",
        "control.source_effectiveness_rank",
        "actuator.surface.symmetric_stabilator.actual",
        "actuator.surface.differential_stabilator.actual",
        "actuator.surface.rudder.actual",
    } <= ids
    ####


def test_source_surface_lqi_screens_advertise_exact_state_and_allocation_evidence() -> None:
    """Both source-surface LQI runners make their propagated state and physical allocation selectable."""

    x15_ids = _applicable_ids(
        "x15",
        "rigid_body_6dof_surface_allocated",
        "x15_source_surface_attitude_rate_lqi_screen_v1",
    )
    assert {
        "attitude.local.roll_error",
        "attitude.local.pitch_error",
        "attitude.local.yaw_error",
        "angular_rate.body.x",
        "angular_rate.body.y",
        "angular_rate.body.z",
        "source_fixture.velocity.body.u",
        "source_fixture.velocity.body.v",
        "source_fixture.velocity.body.w",
        "mass.total",
        "control.moment.x.requested",
        "control.moment.y.achieved",
        "control.moment.z.residual",
        "control.allocation.residual_norm",
        "control.allocation.saturation_count",
        "actuator.surface.symmetric_stabilator.actual",
        "actuator.surface.rudder.actual",
    } <= x15_ids

    hl20_ids = _applicable_ids(
        "hl20_mod_k",
        "rigid_body_6dof_surface_allocated",
        "hl20_source_surface_attitude_rate_lqi_screen_v1",
    )
    assert {
        "attitude.local.roll_error",
        "attitude.local.pitch_error",
        "attitude.local.yaw_error",
        "angular_rate.body.x",
        "angular_rate.body.y",
        "angular_rate.body.z",
        "source_fixture.velocity.body.x",
        "source_fixture.velocity.body.y",
        "source_fixture.velocity.body.z",
        "mass.total",
        "aerodynamics.pitch_coefficient",
        "control.pitch_moment.requested",
        "control.pitch_moment.achieved",
        "control.pitch_moment.residual",
        "control.allocation.residual_norm",
        "control.allocation.saturation_count",
        "actuator.surface.upper_left_body_flap.actual",
        "actuator.surface.rudder.actual",
    } <= hl20_ids
    ####


def test_source_surface_lqi_core_bindings_extract_the_emitted_flattened_truth_rows() -> None:
    """Fixed source velocities and propagated rates use the exact rows emitted by both LQI runners."""

    x15 = {item.id: item for item in native_output_bindings("x15")}
    x15_row: dict[str, object] = {
        "roll_error_rad": 0.1,
        "pitch_error_rad": -0.2,
        "yaw_error_rad": 0.3,
        "p_rad_s": 0.01,
        "q_rad_s": -0.02,
        "r_rad_s": 0.03,
        "body_velocity_m_s": [101.0, 2.0, -3.0],
    }
    assert extract_native_channel(x15_row, x15["attitude.local.roll_error"]) == 0.1
    assert extract_native_channel(x15_row, x15["angular_rate.body.y"]) == -0.02
    assert extract_native_channel(x15_row, x15["source_fixture.velocity.body.w"]) == -3.0

    hl20 = {item.id: item for item in native_output_bindings("hl20_mod_k")}
    hl20_row: dict[str, object] = {
        "roll_error_rad": -0.1,
        "pitch_error_rad": 0.2,
        "yaw_error_rad": -0.3,
        "p_rad_s": -0.01,
        "q_rad_s": 0.02,
        "r_rad_s": -0.03,
        "body_velocity_m_s": [201.0, -4.0, 5.0],
    }
    assert extract_native_channel(hl20_row, hl20["attitude.local.yaw_error"]) == -0.3
    assert extract_native_channel(hl20_row, hl20["angular_rate.body.x"]) == -0.01
    assert extract_native_channel(hl20_row, hl20["source_fixture.velocity.body.y"]) == -4.0
    ####


def test_x8_and_b747_physical_screens_advertise_mass_and_allocation_diagnostics() -> None:
    """Source-table physical screens retain source mass and observable allocation health."""

    for model_id, missions in (
        (
            "skywalker_x8",
            (
                "x8_local_physical_surface_lqr_screen_v1",
                "x8_local_physical_surface_lqi_screen_v1",
                "x8_local_physical_surface_lqi_long_recovery_screen_v1",
            ),
        ),
        (
            "b747",
            (
                "b747_condition3_local_physical_surface_lqr_screen_v1",
                "b747_condition3_local_physical_surface_lqi_screen_v1",
            ),
        ),
    ):
        for mission in missions:
            ids = _applicable_ids(model_id, "rigid_body_6dof_surface_allocated", mission)
            assert {
                "mass.total",
                "control.wrench.residual.moment.x",
                "control.wrench.residual.moment.y",
                "control.wrench.residual.moment.z",
                "diagnostics.allocation_residual_norm",
                "diagnostics.saturation_count",
            } <= ids
    ####
