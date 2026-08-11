from __future__ import annotations

from taoryx.families.cadac.events import CadacEventCursor
from taoryx.families.cadac.input_ast import CadacEventBlock
from taoryx.families.cadac.input_parser import parse_cadac_input


def _events() -> tuple[CadacEventBlock, ...]:
    case = parse_cadac_input(
        """\
TITLE event_runtime.asc Event runtime
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
        altcom 1000
        IF wp_flag = -1
            wp_flag 0
            altcom 1500
        ENDIF
        IF wp_flag = -1
            altcom 2000
        ENDIF
    END
ENDTIME 10
STOP
"""
    )
    return case.vehicles[0].events


####


def test_event_cursor_evaluates_only_next_event() -> None:
    values: dict[str, int | float] = {"wp_flag": 0, "altcom": 1000.0}
    cursor = CadacEventCursor.from_events(_events())

    assert cursor.evaluate_and_apply(values) is None
    values["wp_flag"] = -1
    first = cursor.evaluate_and_apply(values)
    assert first is not None
    assert first.event_index == 0
    assert values == {"wp_flag": 0, "altcom": 1500.0}
    assert cursor.next_index == 1

    assert cursor.evaluate_and_apply(values) is None
    values["wp_flag"] = -1
    second = cursor.evaluate_and_apply(values)
    assert second is not None
    assert second.event_index == 1
    assert values["altcom"] == 2000.0
    assert cursor.complete is True


####
