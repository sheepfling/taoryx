from __future__ import annotations

import json
from importlib import import_module
from pathlib import Path
from typing import Any, cast

_tuning_tool: Any = import_module("tools.tune_reference_aircraft")
FAMILIES = cast(tuple[str, ...], _tuning_tool.FAMILIES)
OUTPUT = cast(Path, _tuning_tool.OUTPUT)
build = _tuning_tool.build


def test_reference_aircraft_tuning_report_covers_all_four_families() -> None:
    report = build()

    assert tuple(report["families"]) == FAMILIES
    for vehicle_id in FAMILIES:
        family = report["families"][vehicle_id]
        assert family["trim_spec"]
        assert family["linearization_mode"]
        assert family["control_realization"]
        assert family["evidence_tier"] == "T0_structural"
        assert family["source_linearization_status"] in {"blocked", "estimated", "available"}
        assert family["tuning_stage"] == "generic_lqr_screen"
        assert family["qualification_status"] == "screen_only"
        assert family["plant_backed_linearization"] is False
        assert family["gap"]
        assert family["authority_preflight"]["reason"].startswith(f"{vehicle_id}-attitude-screen-authority")
        assert family["authority_preflight"]["status"] == "passed"
        assert family["authority_preflight"]["metrics"]["controllability_rank"] == 6.0
        assert family["report"]["best_profile_id"] is not None
        assert family["report"]["status"] == "safe"
    ####


def test_reference_aircraft_tuning_report_is_generated_and_current() -> None:
    expected = build()
    actual = json.loads(Path(OUTPUT).read_text(encoding="utf-8"))

    assert actual == expected
    ####
