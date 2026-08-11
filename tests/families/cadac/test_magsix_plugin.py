from __future__ import annotations

from pathlib import Path

from taoryx.families.cadac.magsix_plugin import MagsixPluginOverrides, MagsixVehiclePlugin

_INPUT = """TITLE MAGSIX trajectory fixture
OPTIONS y_plot
MODULES
environment def,exec
trajectory def,init,exec,term
END
TIMING
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


def test_magsix_plugin_is_runnable_trajectory_only(tmp_path: Path) -> None:
    plugin = MagsixVehiclePlugin(_case(tmp_path))
    descriptor = plugin.descriptor
    assert descriptor.status == "runnable"
    assert descriptor.default_phase_id == "trajectory_only"
    assert descriptor.batch_factory_id == "cadac.magsix.trajectory.batch"
    assert plugin.validate_installation() == ()


####


def test_magsix_plugin_overrides_do_not_mutate_source_definition(tmp_path: Path) -> None:
    plugin = MagsixVehiclePlugin(_case(tmp_path))
    original = plugin.source_definition()
    run = plugin.run_batch(
        MagsixPluginOverrides(
            altitude_m=1200.0,
            speed_mps=18.0,
            spin_rpm=900.0,
            end_time_dnt=0.01,
            sample_step_dnt=0.01,
        )
    )
    assert run.samples[0].altitude_m < 1200.0
    assert original.initial_state.altitude_m == 1000.0
    assert original.initial_state.speed_mps == 16.6
    assert original.initial_state.spin_rpm == 850.0
    assert original.end_time_dnt == 50.0


####
