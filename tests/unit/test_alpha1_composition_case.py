from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "run_alpha1_composition_case",
    ROOT / "tools/run_alpha1_composition_case.py",
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_alpha1_composition_case_is_data_driven() -> None:
    case = MODULE._load_case(ROOT / "verification/alpha1_composition_case.yaml")
    builder = MODULE._build(case)
    scenario = builder.build()

    assert builder.duration_s == 20.0
    assert [segment.id for segment in scenario.segments] == ["trim", "heading"]
    assert all(segment.goal is not None for segment in scenario.segments)
    assert scenario.segments[0].goal.kind == "trim_hold"
    assert scenario.segments[1].goal.kind == "heading_capture"
    assert "trim_hold" in builder.registry.names()
####
