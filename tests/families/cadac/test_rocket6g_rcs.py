from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.rocket6g_rcs import (
    Rocket6gRcsConfig,
    Rocket6gRcsRuntimeInput,
    Rocket6gRcsState,
    load_rocket6g_rcs_source_definition,
    rocket6g_rcs_proportional,
    rocket6g_rcs_schmitt,
    rocket6g_rcs_step,
)

_INPUT = """TITLE ROCKET6G RCS fixture
MONTE 1 1234
OPTIONS y_plot
MODULES
kinematics def,init,exec
rcs def,exec
forces def,exec
newton def,init,exec
euler def,init,exec
END
TIMING
plot_step 0.1
int_step 0.001
END
VEHICLES 1
HYPER6 SLV
GAUSS sensor_bias 0 3
MARKOV sensor_noise 0.25 100
RAYL dvae 5
mrcs_moment 21
mrcs_force 0
roll_mom_max 100
pitch_mom_max 200000
yaw_mom_max 200000
dead_zone 0.4
hysteresis 0.1
rcs_tau 1
thtbdcomx 80
psibdcomx -83
END
ENDTIME 20
STOP
"""


def _case(tmp_path: Path) -> Path:
    path = tmp_path / "input.asc"
    path.write_text(_INPUT, encoding="utf-8")
    return path


####


def test_rocket6g_rcs_lowering_preserves_direct_wrench_boundary(tmp_path: Path) -> None:
    definition = load_rocket6g_rcs_source_definition(_case(tmp_path))
    assert definition.taoryx_tier == "rigid_body_6dof_direct_wrench"
    assert definition.control_realization == "direct_wrench"
    assert definition.config.moment_mode == 21
    assert definition.config.control_mode == 1
    assert definition.config.moment_type == 2


####


def test_rocket6g_proportional_limiter_matches_source() -> None:
    assert rocket6g_rcs_proportional(25.0, 100.0) == 25.0
    assert rocket6g_rcs_proportional(125.0, 100.0) == 100.0
    assert rocket6g_rcs_proportional(-125.0, 100.0) == -100.0


####


def test_rocket6g_schmitt_preserves_previous_input_semantics() -> None:
    assert rocket6g_rcs_schmitt(-10.0, 0.0, 0.4, 0.1) == 0
    assert rocket6g_rcs_schmitt(-10.0, -10.0, 0.4, 0.1) == -1
    assert rocket6g_rcs_schmitt(10.0, 10.0, 0.4, 0.1) == 1


####


def test_rocket6g_on_off_moment_thruster_has_source_one_pass_trigger_lag() -> None:
    config = Rocket6gRcsConfig(
        moment_mode=21,
        force_mode=0,
        dead_zone=0.4,
        hysteresis=0.1,
        time_slope_s=1.0,
        roll_moment_limit_nm=100.0,
        pitch_moment_limit_nm=200000.0,
        yaw_moment_limit_nm=200000.0,
        proportional_damping=0.0,
        proportional_frequency_rad_s=0.0,
        acceleration_gain_n_per_mps2=0.0,
        side_force_limit_n=0.0,
        pitch_command_deg=80.0,
        yaw_command_deg=-83.0,
    )
    runtime = Rocket6gRcsRuntimeInput(
        inertia_diagonal_kgm2=(1000.0, 1000.0, 1000.0),
        geodetic_angles_deg=(0.0, 90.0, -83.0),
    )
    first = rocket6g_rcs_step(config, runtime)
    second = rocket6g_rcs_step(config, runtime, first.state)

    assert first.moment_body_nm == (0.0, 0.0, 0.0)
    assert first.state.pitch_saved_error == -10.0
    assert second.moment_body_nm == (0.0, -200000.0, 0.0)
    assert second.state.pitch_output == -1
    assert second.state.pitch_switch_count == 1


####


def test_rocket6g_proportional_rcs_maps_angle_error_to_bounded_direct_wrench() -> None:
    config = Rocket6gRcsConfig(
        moment_mode=11,
        force_mode=1,
        dead_zone=0.0,
        hysteresis=0.0,
        time_slope_s=0.0,
        roll_moment_limit_nm=100.0,
        pitch_moment_limit_nm=150.0,
        yaw_moment_limit_nm=200.0,
        proportional_damping=0.7,
        proportional_frequency_rad_s=2.0,
        acceleration_gain_n_per_mps2=100.0,
        side_force_limit_n=50.0,
        roll_command_deg=10.0,
        pitch_command_deg=5.0,
        yaw_command_deg=-5.0,
    )
    runtime = Rocket6gRcsRuntimeInput(
        inertia_diagonal_kgm2=(100.0, 200.0, 300.0),
        specific_force_body_mps2=(0.0, 0.0, 0.0),
        acceleration_commands_g=(1.0, -1.0),
    )
    step = rocket6g_rcs_step(config, runtime, Rocket6gRcsState())
    assert step.moment_body_nm == pytest.approx((100.0, 150.0, -200.0))
    assert step.force_body_n == pytest.approx((0.0, 50.0, -50.0))


####
