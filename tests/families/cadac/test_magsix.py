from __future__ import annotations

import math
from pathlib import Path

import pytest
from taoryx.families.cadac.magsix import load_magsix_source_definition, run_magsix_trajectory_source_compatibility

_INPUT = """TITLE MAGSIX trajectory fixture
OPTIONS y_plot
MODULES
environment def,exec
trajectory def,init,exec,term
END
TIMING
scrn_step 2
plot_step 0.01
int_step 0.001
END
VEHICLES 1
ROTOR RECT.MR1
sbel1 0
sbel2 0
hbe 1000
hbg 0
dvbe 16.6
psivlx 0
thtvlx -77
omega_rpm 850
mass 1.5
moi_spin .004
ref_area .0468
ref_length .0625
cd 1.31
cmdw -.45
clw 2.51
cma .508
END
ENDTIME 50
STOP
"""


def _case(tmp_path: Path) -> Path:
    path = tmp_path / "input.asc"
    path.write_text(_INPUT, encoding="utf-8")
    return path


####


def test_magsix_trajectory_only_lowers_as_point_mass(tmp_path: Path) -> None:
    definition = load_magsix_source_definition(_case(tmp_path))
    assert definition.source_model == "ROTOR"
    assert definition.taoryx_tier == "point_mass_3dof"
    assert definition.control_realization == "force_model"
    assert definition.module_order == ("environment", "trajectory")
    assert definition.integration_step_dnt == pytest.approx(0.001)
    assert definition.plot_step_dnt == pytest.approx(0.01)
    assert definition.source_artifacts[0].sha256


####


def test_magsix_time_zero_sample_is_post_first_source_step(tmp_path: Path) -> None:
    definition = load_magsix_source_definition(_case(tmp_path))
    run = run_magsix_trajectory_source_compatibility(definition, end_time_dnt=0.001, sample_step_dnt=0.001)
    sample = run.samples[0]
    assert sample.time_s == 0.0
    assert sample.source_time_dnt == 0.0
    assert sample.speed_mps != pytest.approx(definition.initial_state.speed_mps)
    assert sample.position_ned_m[0] > definition.initial_state.north_m
    assert sample.altitude_m < definition.initial_state.altitude_m
    assert all(math.isfinite(value) for value in (*sample.position_ned_m, *sample.velocity_ned_mps))


####


def test_magsix_source_dynamics_evolve_speed_glide_and_spin(tmp_path: Path) -> None:
    definition = load_magsix_source_definition(_case(tmp_path))
    run = run_magsix_trajectory_source_compatibility(definition, end_time_dnt=0.05, sample_step_dnt=0.01)
    first = run.samples[0]
    last = run.samples[-1]
    assert run.status == "completed"
    assert last.time_s > first.time_s
    assert last.speed_mps != pytest.approx(first.speed_mps)
    assert last.flight_path_deg != pytest.approx(first.flight_path_deg)
    assert last.spin_rpm != pytest.approx(first.spin_rpm)
    assert last.dynamic_time_scale_s > 0.0
    assert last.dynamic_pressure_pa > 0.0
    assert last.mach > 0.0


####


def test_magsix_reported_real_time_is_monotonic(tmp_path: Path) -> None:
    definition = load_magsix_source_definition(_case(tmp_path))
    run = run_magsix_trajectory_source_compatibility(definition, end_time_dnt=0.1, sample_step_dnt=0.01)
    times = tuple(sample.time_s for sample in run.samples)
    assert times == tuple(sorted(times))
    assert len(set(times)) == len(times)


####
