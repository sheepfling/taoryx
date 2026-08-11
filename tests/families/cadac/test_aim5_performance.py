from __future__ import annotations

from pathlib import Path
from time import perf_counter

from taoryx.families.cadac.aim5_plugin import Aim5VehiclePlugin
from taoryx.families.cadac.aim5_scenario import Aim5ScenarioSession
from test_aim5 import AERO, INPUT, PROP


def _case(tmp_path: Path) -> Path:
    path = tmp_path / "input.asc"
    path.write_text(INPUT, encoding="utf-8")
    (tmp_path / "aero.asc").write_text(AERO, encoding="utf-8")
    (tmp_path / "prop.asc").write_text(PROP, encoding="utf-8")
    return path


####


def test_aim5_persistent_source_loop_stays_within_interactive_smoke_budget(tmp_path: Path) -> None:
    """Catch accidental batch replay or per-step heavyweight sensor setup.

    This is intentionally a broad CI smoke budget rather than a claim about
    target-machine throughput.  The behavioral assertions make the guard
    meaningful even on slow runners: every source step must execute through
    one persistent state owner, and the full loop must remain responsive.
    """

    definition = Aim5VehiclePlugin(_case(tmp_path)).source_definition()
    far_target = definition.targets[0].model_copy(
        update={"config": definition.targets[0].config.model_copy(update={"position_ned_m": (0.0, 100_000.0, -1_000.0)})}
    )
    definition = definition.model_copy(update={"end_time_s": 2.5, "targets": (far_target,)})
    session = Aim5ScenarioSession(definition)
    started = perf_counter()
    for _ in range(250):
        session.advance(0.01)
    elapsed_s = perf_counter() - started

    assert session.executed_steps == 250
    assert session.completed
    assert elapsed_s < 2.0


####
