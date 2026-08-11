from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.aim5 import load_aim5_source_definition, run_aim5_source_compatibility
from taoryx.families.cadac.aim5_scenario import (
    load_aim5_scenario_source_definition,
    run_aim5_scenario_source_compatibility,
)
from test_aim5 import AERO, INPUT, PROP

MULTI_INPUT = """\
TITLE synthetic_multi.asc Two independent AIM5 engagements
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
    plot_step 0.05
    int_step 0.01
END
VEHICLES 4
    AIM5 Missile_1
        tgt_num 1
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
    AIM5 Missile_2
        tgt_num 2
        sael1 -1000
        sael2 0
        sael3 -1000
        psivlx 0
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
    AIRCRAFT3 Target_1
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
    AIRCRAFT3 Target_2
        sael1 0
        sael2 0
        sael3 -1000
        psivlx 180
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


def _write_multi_case(tmp_path: Path) -> Path:
    path = tmp_path / "input_multi.asc"
    path.write_text(MULTI_INPUT, encoding="utf-8")
    (tmp_path / "aero.asc").write_text(AERO, encoding="utf-8")
    (tmp_path / "prop.asc").write_text(PROP, encoding="utf-8")
    return path


####


def test_multi_lowering_preserves_repeated_actor_identity_and_target_assignment(tmp_path: Path) -> None:
    definition = load_aim5_scenario_source_definition(_write_multi_case(tmp_path))

    assert [item.object_id for item in definition.missiles] == ["m1", "m2"]
    assert [item.object_id for item in definition.targets] == ["a1", "a2"]
    assert [item.object_id for item in definition.vehicle_order] == ["m1", "m2", "a1", "a2"]
    assert [item.config.target_number for item in definition.missiles] == [1, 2]
    assert definition.missiles[0].aerodynamic_deck == definition.missiles[1].aerodynamic_deck


####


def test_multi_runner_executes_two_independent_intercepts_with_source_order_trace(tmp_path: Path) -> None:
    definition = load_aim5_scenario_source_definition(_write_multi_case(tmp_path))
    result = run_aim5_scenario_source_compatibility(definition, sample_step_s=0.05, trace_steps=1)

    assert result.terminated_reason == "all_missiles_terminated"
    assert len(result.engagements) == 2
    assert [item.target_object_id for item in result.engagements] == ["a1", "a2"]
    assert all(item.intercept is not None for item in result.engagements)
    assert all(item.intercept is not None and item.intercept.miss_distance_m < 500.0 for item in result.engagements)
    assert [item.object_id for item in result.module_trace[:20:10]] == ["m1", "m2"]
    assert {item.object_id for item in result.module_trace} == {"m1", "m2", "a1", "a2"}
    assert [item.target_object_id for item in result.targets] == ["a1", "a2"]
    assert all(item.samples for item in result.targets)
    assert all(item.samples[0].time_s == pytest.approx(0.0) for item in result.targets)
    assert all(item.samples[-1].alive is False for item in result.targets)


####


def test_scenario_runner_matches_single_runner_for_one_engagement(tmp_path: Path) -> None:
    single_path = tmp_path / "single"
    single_path.mkdir()
    path = single_path / "input.asc"
    path.write_text(INPUT, encoding="utf-8")
    (single_path / "aero.asc").write_text(AERO, encoding="utf-8")
    (single_path / "prop.asc").write_text(PROP, encoding="utf-8")

    single = run_aim5_source_compatibility(load_aim5_source_definition(path), sample_step_s=0.05)
    scenario = run_aim5_scenario_source_compatibility(
        load_aim5_scenario_source_definition(path),
        sample_step_s=0.05,
    )

    assert single.intercept is not None
    assert scenario.engagements[0].intercept is not None
    assert scenario.engagements[0].intercept.time_s == pytest.approx(single.intercept.time_s)
    assert scenario.engagements[0].intercept.miss_distance_m == pytest.approx(single.intercept.miss_distance_m)


####
