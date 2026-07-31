from __future__ import annotations

import json
from importlib import import_module
from pathlib import Path
from typing import Any, cast

_tool: Any = import_module("tools.validate_four_vehicle_control_evidence")
OUTPUT = cast(Path, _tool.OUTPUT)
build = _tool.build


def test_four_vehicle_control_evidence_keeps_tiers_and_blockers_distinct() -> None:
    """A direct screen must not conceal the X-15 trim prerequisite failure."""

    report = build()

    assert report["status"] == "tiered_evidence_complete"
    assert report["claim"]["direct_wrench_results_are_screen_only"] is True
    assert report["promotion_summary"] == {
        "T5_local_nonlinear": ["hummingbird", "b747"],
        "T4_source_coordinate_physical": ["skywalker_x8"],
        "T0_blocked_before_trim": ["x15"],
    }
    for vehicle in report["vehicles"].values():
        assert vehicle["direct_body_moment_injection"] is False
        assert vehicle["reproduction"]
    assert report["vehicles"]["x15"]["status"] == "blocked_before_T1_trim"
    ####


def test_four_vehicle_control_evidence_ledger_is_generated_and_current() -> None:
    """Keep the release ledger exactly reproducible from its source packets."""

    expected = build()
    actual = json.loads(OUTPUT.read_text(encoding="utf-8"))

    assert actual == expected
    ####
