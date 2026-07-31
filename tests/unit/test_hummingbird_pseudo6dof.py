"""Tests for the aggregate thrust-vector Hummingbird pseudo tier."""

from __future__ import annotations

import math

from taoryx.trajectory import HummingbirdPseudo6DOFCommand, HummingbirdPseudo6DOFModel


def test_hummingbird_pseudo_profile_is_bounded_and_explicit() -> None:
    model = HummingbirdPseudo6DOFModel()
    state = model.initial_state(altitude_m=2.0)
    state, row = model.step(
        state,
        HummingbirdPseudo6DOFCommand(roll_rad=math.radians(20.0), pitch_rad=math.radians(-15.0), yaw_rad=math.radians(30.0), thrust_ratio=1.0),
        0.02,
    )

    assert row["response_profile_id"] == "hummingbird.attitude_response_p6dof.v1"
    assert row["allocation_mode"] == "aggregate_thrust_vector_surrogate"
    assert row["physical_motor_allocation"] is False
    assert all(abs(rate) <= limit.maximum_rate_rad_s + 1.0e-12 for rate, limit in zip(state.attitude_rate_rad_s, (model.profile.response["roll"], model.profile.response["pitch"], model.profile.response["yaw"]), strict=True))
    assert 0.0 <= state.battery_fraction <= 1.0
    ####


def test_hummingbird_pseudo_exercises_yaw_and_translation() -> None:
    model = HummingbirdPseudo6DOFModel()
    commands = tuple(
        HummingbirdPseudo6DOFCommand(
            yaw_rad=math.radians(45.0) if index >= 10 else 0.0,
            pitch_rad=math.radians(8.0) if index >= 20 else 0.0,
            thrust_ratio=1.0,
        )
        for index in range(40)
    )
    terminal, rows = model.run(commands, dt_s=0.02)

    assert len(rows) == 40
    assert terminal.attitude_rad[2] > 0.0
    assert terminal.position_m[0] > 0.0
    assert all(row["response_profile_id"] == "hummingbird.attitude_response_p6dof.v1" for row in rows)
    ####


def test_hummingbird_pseudo_reports_contact_and_motor_shutdown() -> None:
    model = HummingbirdPseudo6DOFModel()
    state = model.initial_state(altitude_m=0.02)
    state, row = model.step(state, HummingbirdPseudo6DOFCommand(thrust_ratio=0.0), 0.1)

    assert state.contact is True
    assert state.position_m[2] == 0.0
    assert state.velocity_m_s[2] == 0.0
    assert row["contact_state"] is True

    state, row = model.step(state, HummingbirdPseudo6DOFCommand(motors_enabled=False), 0.1)
    assert state.contact is True
    assert row["shutdown"] is True
    assert row["achieved_thrust_n"] == 0.0
    ####


def test_hummingbird_pseudo_declares_body_frame_thrust_mapping() -> None:
    model = HummingbirdPseudo6DOFModel()
    state = model.initial_state(altitude_m=2.0)
    world_command = HummingbirdPseudo6DOFCommand(pitch_rad=math.radians(10.0), yaw_rad=math.radians(90.0), thrust_ratio=0.3)
    body_command = HummingbirdPseudo6DOFCommand(pitch_rad=math.radians(10.0), yaw_rad=math.radians(90.0), thrust_ratio=0.3, thrust_frame="body_euler")
    _, world_rows = model.run(tuple(world_command for _ in range(200)), dt_s=0.02, initial_state=state)
    _, body_rows = model.run(tuple(body_command for _ in range(200)), dt_s=0.02, initial_state=state)
    world_row = world_rows[-1]
    body_row = body_rows[-1]
    assert world_row["thrust_vector_frame"] == "world_euler"
    assert body_row["thrust_vector_frame"] == "body_euler"
    assert abs(float(body_row["achieved_thrust_vector_n"][1])) > abs(float(body_row["achieved_thrust_vector_n"][0]))
    assert abs(float(world_row["achieved_thrust_vector_n"][0])) > abs(float(world_row["achieved_thrust_vector_n"][1]))
    ####
