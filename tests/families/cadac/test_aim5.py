from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from taoryx.families.cadac.aim5 import (
    Aim5SourceError,
    atmosphere76,
    load_aim5_source_definition,
    mat2tr,
    run_aim5_source_compatibility,
)

INPUT = """\
TITLE synthetic_aim5.asc Head-on compatibility fixture
OPTIONS n_scrn
MODULES
    environment def,exec
    kinematics def,init,exec
    aerodynamics def,exec
    propulsion def,exec
    seeker def,exec
    guidance def,exec
    control def,init,exec
    forces def,exec
    newton def,init,exec
    intercept def,exec
END
TIMING
    int_step 0.01
END
VEHICLES 2
    AIM5 Missile
        sael1 0
        sael2 -1000
        sael3 -1000
        psivlx 90
        thtvlx 0
        alphax 0
        betax 0
        dvae 250
        AERO_DECK aero.asc
        area 0.02
        alpmax 40
        PROP_DECK prop.asc
        mprop 1
        mass 60
        aexit 0
        mseek 1
        mguid 1
        gnav 4
        tr 0.1
        ta 2
        gacp 40
    END
    AIRCRAFT3 Target
        sael1 0
        sael2 0
        sael3 -1000
        psivlx -90
        thtvlx 0
        dvae 200
        acft_option 0
        clalpha 0.05
        wingloading 3000
        philimx 60
        alplimx 12
    END
ENDTIME 4
STOP
"""

AERO = """\
TITLE synthetic aero
2DIM cl_aim_vs_alpha_mach
NX1 2 NX2 2
0 0.5 0 0
40 5.0 8 8
2DIM cd_aim_on_vs_alpha_mach
NX1 2 NX2 2
0 0.5 0.2 0.2
40 5.0 1.0 1.0
2DIM cd_aim_off_vs_alpha_mach
NX1 2 NX2 2
0 0.5 0.25 0.25
40 5.0 1.1 1.1
"""

PROP = """\
TITLE synthetic prop
1DIM mass_vs_time
NX1 3
0 60
3 50
4 50
1DIM thrust_vs_time
NX1 3
0 12000
3 12000
4 0
"""


def _write_case(tmp_path: Path, *, target_first: bool = False) -> Path:
    text = INPUT
    if target_first:
        missile_start = text.index("    AIM5 Missile")
        target_start = text.index("    AIRCRAFT3 Target")
        endtime_start = text.index("ENDTIME 4")
        missile_block = text[missile_start:target_start]
        target_block = text[target_start:endtime_start]
        text = text[:missile_start] + target_block + missile_block + text[endtime_start:]
    ####
    path = tmp_path / "input.asc"
    path.write_text(text, encoding="utf-8")
    (tmp_path / "aero.asc").write_text(AERO, encoding="utf-8")
    (tmp_path / "prop.asc").write_text(PROP, encoding="utf-8")
    return path


####


def test_atmosphere76_matches_source_sea_level_reference() -> None:
    rho, pressure, temperature = atmosphere76(0.0)

    assert rho == pytest.approx(1.225)
    assert pressure == pytest.approx(101_325.0)
    assert temperature == pytest.approx(288.15)


####


def test_mat2tr_maps_north_to_vehicle_axis_at_zero_heading() -> None:
    matrix = mat2tr(0.0, 0.0)

    np.testing.assert_allclose(matrix, np.eye(3))
    east_heading = mat2tr(math.pi / 2.0, 0.0)
    assert east_heading[0, 1] == pytest.approx(1.0)
    assert east_heading[1, 0] == pytest.approx(-1.0)


####


def test_lowering_declares_source_execution_semantics(tmp_path: Path) -> None:
    definition = load_aim5_source_definition(_write_case(tmp_path))

    assert definition.integration_step_s == pytest.approx(0.01)
    assert definition.module_order[0] == "environment"
    assert definition.module_order[-1] == "intercept"
    assert definition.missile.target_number == 1
    assert definition.actor_fidelity[0].taoryx_tier == "pseudo_6dof"
    assert definition.actor_fidelity[1].taoryx_tier == "point_mass_3dof"


####


def test_lowering_rejects_vehicle_reordering_that_changes_combus_semantics(tmp_path: Path) -> None:
    with pytest.raises(Aim5SourceError, match="AIM5 before AIRCRAFT3"):
        load_aim5_source_definition(_write_case(tmp_path, target_first=True))
    ####


####


def test_closed_loop_source_compatibility_runner_reaches_intercept(tmp_path: Path) -> None:
    definition = load_aim5_source_definition(_write_case(tmp_path))
    result = run_aim5_source_compatibility(definition, sample_step_s=0.1, trace_steps=1)

    assert result.terminated_reason == "intercept"
    assert result.intercept is not None
    assert result.intercept.time_s < definition.end_time_s
    assert result.intercept.miss_distance_m < 500.0
    assert result.samples[-1].range_m == pytest.approx(result.intercept.miss_distance_m)
    assert result.execution_semantics.vehicle_order == ("AIM5", "AIRCRAFT3")
    assert result.execution_semantics.target_snapshot_seen_by_missile == "previous_target_vehicle_pass"
    assert result.executed_steps > 0
    assert [item.module for item in result.module_trace[:10]] == list(definition.module_order)
    assert {item.vehicle_model for item in result.module_trace} == {"AIM5", "AIRCRAFT3"}


####


def test_plot_projection_can_align_to_requested_source_epochs(tmp_path: Path) -> None:
    from taoryx.families.cadac.aim5 import aim5_plot_projection

    definition = load_aim5_source_definition(_write_case(tmp_path))
    result = run_aim5_source_compatibility(definition, sample_step_s=0.1)
    projection = aim5_plot_projection(result, times_s=(0.0, 0.1, 0.2))

    assert projection["time"] == pytest.approx((0.0, 0.1, 0.2))
    assert len(projection["mach"]) == 3


####


def test_source_event_mutates_aim5_runtime_before_module_pass(tmp_path: Path) -> None:
    event_input = INPUT.replace(
        "        gacp 40\n    END\n    AIRCRAFT3 Target",
        "        gacp 40\n        IF time > 0.1\n            gnav 0\n        ENDIF\n    END\n    AIRCRAFT3 Target",
    )
    path = tmp_path / "input.asc"
    path.write_text(event_input, encoding="utf-8")
    (tmp_path / "aero.asc").write_text(AERO, encoding="utf-8")
    (tmp_path / "prop.asc").write_text(PROP, encoding="utf-8")

    definition = load_aim5_source_definition(path)
    assert len(definition.missile_events) == 1

    result = run_aim5_source_compatibility(definition, sample_step_s=0.05)

    assert len(result.event_trace) == 1
    event = result.event_trace[0]
    assert event.vehicle_model == "AIM5"
    assert event.time_s == pytest.approx(0.11)
    assert event.watch_variable == "time"
    assert event.updated_values == (("gnav", 0.0),)


####
