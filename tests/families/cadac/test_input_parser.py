from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.input_ast import CadacDeckKind, CadacModuleStage
from taoryx.families.cadac.input_parser import CadacInputParseError, parse_cadac_input

CASE = """    TITLE prototype.asc Demonstration engagement
OPTIONS y_plot y_traj
MODULES
    environment def,exec
    control def,init,exec
    newton def,init,exec
END
TIMING
    plot_step 0.1
    int_step 0.01
END
VEHICLES 2
    AIM5 Missile
        north_m 0 // initial north coordinate
        speed_mps 250.0
        AERO_DECK demo_aero.asc
    END
    AIRCRAFT3 Target
        north_m 5000
        speed_mps 220
    END
ENDTIME 8
STOP
"""


def test_parse_case_preserves_module_and_vehicle_order() -> None:
    case = parse_cadac_input(CASE, source_name="fixture/input.asc")

    assert case.description == "Demonstration engagement"
    assert [module.name for module in case.modules] == ["environment", "control", "newton"]
    assert case.modules[1].stages == (
        CadacModuleStage.DEFINE,
        CadacModuleStage.INITIALIZE,
        CadacModuleStage.EXECUTE,
    )
    assert case.timing_values == {"plot_step": 0.1, "int_step": 0.01}
    assert [vehicle.model_name for vehicle in case.vehicles] == ["AIM5", "AIRCRAFT3"]
    assert case.vehicle("aim5").parameter("speed_mps") == 250.0
    assert case.vehicle("AIM5").deck_references[0].kind is CadacDeckKind.AERODYNAMIC
    assert case.vehicle("AIM5").deck_references[0].keyword == "AERO_DECK"
    assert case.vehicle("AIM5").assignments[0].comment == "initial north coordinate"


####


def test_parse_case_rejects_vehicle_count_mismatch() -> None:
    malformed = CASE.replace("VEHICLES 2", "VEHICLES 3")

    with pytest.raises(CadacInputParseError, match="declared VEHICLES count"):
        parse_cadac_input(malformed)
    ####


####


def test_parse_case_rejects_unknown_module_stage() -> None:
    malformed = CASE.replace("environment def,exec", "environment def,unknown")

    with pytest.raises(CadacInputParseError, match="unknown module stage"):
        parse_cadac_input(malformed)
    ####


####


def test_parse_source_ordered_event_blocks() -> None:
    case = parse_cadac_input(
        """\
TITLE events.asc Event fixture
OPTIONS y_events
MODULES
    newton def,init,exec
END
TIMING
    int_step 0.01
END
VEHICLES 1
    PLANE6 Aircraft
        wp_flag 0
        swel1 5000
        IF wp_flag = -1
            swel1 12000
            altcom 1500
        ENDIF
        IF wp_flag = -1
            swel1 16000
            altcom 2000
        ENDIF
    END
ENDTIME 20
STOP
"""
    )

    vehicle = case.vehicles[0]
    assert len(vehicle.events) == 2
    assert vehicle.parameter("swel1") == 5000
    assert vehicle.events[0].condition.variable == "wp_flag"
    assert vehicle.events[0].condition.operator.value == "="
    assert [item.name for item in vehicle.events[0].assignments] == ["swel1", "altcom"]
    assert vehicle.events[1].assignments[0].value == 16000


####


def test_parser_preserves_monte_carlo_and_stochastic_declarations() -> None:
    parsed = parse_cadac_input(
        """TITLE stochastic case
MONTE 4 1234
OPTIONS n_plot
MODULES
environment def,exec
END
TIMING
int_step 0.01
END
VEHICLES 1
HYPER6 Rocket
GAUSS bias 0 3
MARKOV noise 0.25 100
RAYL wind 5
value 2
END
ENDTIME 1
STOP
"""
    )
    assert parsed.monte_carlo is not None
    assert parsed.monte_carlo.runs == 4
    assert parsed.monte_carlo.seed == 1234
    vehicle = parsed.vehicles[0]
    assert vehicle.parameter("value") == 2
    assert tuple(item.kind.value for item in vehicle.stochastic_assignments) == ("GAUSS", "MARKOV", "RAYL")
    assert vehicle.stochastic_assignments[1].parameters == (0.25, 100.0)


####


def test_parser_accepts_zero_mutation_event_as_epoch_reset_watchpoint() -> None:
    parsed = parse_cadac_input(
        """TITLE zero-event fixture
OPTIONS y_events
MODULES
propulsion def,exec
END
TIMING
int_step 0.01
END
VEHICLES 1
HYPER6 Rocket
thrust 1
IF thrust = 0
ENDIF
END
ENDTIME 2
STOP
"""
    )
    event = parsed.vehicles[0].events[0]
    assert event.condition.variable == "thrust"
    assert event.assignments == ()
    ####


def test_parser_accepts_legacy_compact_event_condition_and_vehicle_stop_parameter() -> None:
    parsed = parse_cadac_input(
        """TITLE compact-event
OPTIONS y_plot
MODULES
environment def,exec
END
TIMING
int_step 0.1
END
VEHICLES 1
MISSILE6 direct_plant
stop 0
IF fuel<5000
mode 3
ENDIF
END
ENDTIME 1
STOP
"""
    )

    vehicle = parsed.vehicles[0]
    assert vehicle.parameter("stop") == 0
    event = vehicle.events[0]
    assert (event.condition.variable, event.condition.operator, event.condition.value) == ("fuel", "<", 5000)
    assert event.assignments[0].name == "mode"
    ####


def test_parser_preserves_historical_noop_stray_endif_after_a_completed_event() -> None:
    parsed = parse_cadac_input(
        """TITLE stray-endif
OPTIONS y_plot
MODULES
environment def,exec
END
TIMING
int_step 0.1
END
VEHICLES 1
MISSILE6 source_controller
IF time > 5
mode 3
ENDIF
ENDIF
END
ENDTIME 10
STOP
"""
    )

    vehicle = parsed.vehicles[0]
    assert len(vehicle.events) == 1
    assert vehicle.events[0].assignments[0].name == "mode"


####


def test_vehicle_duplicate_initial_assignments_preserve_source_order_and_last_value_wins(tmp_path: Path) -> None:
    source = tmp_path / "duplicate-assignment.asc"
    source.write_text(
        """TITLE duplicate source assignment\nOPTIONS y_traj\nMODULES\n environment def,exec\nEND\nTIMING\n int_step 0.01\nEND\nVEHICLES 1\n MISSILE6 SAM\n  gain_rf 10\n  gain_rf 5\n END\nENDTIME 1\nSTOP\n""",
        encoding="utf-8",
    )

    case = parse_cadac_input(source.read_text(encoding="utf-8"), source_name=source.as_posix())
    vehicle = case.vehicle("MISSILE6")

    assert tuple(assignment.value for assignment in vehicle.assignments if assignment.name == "gain_rf") == (10, 5)
    assert vehicle.parameter("GAIN_RF") == 5


####
