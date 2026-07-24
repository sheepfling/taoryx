from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

from taoryx.contracts import Frame
from taoryx.rigid_body import RIGID_BODY_STATE_NAMES
from taoryx.runtime.common import RuntimeState

ROOT = Path(__file__).parents[2]
SPEC = importlib.util.spec_from_file_location("solve_x15_trim", ROOT / "tools/solve_x15_trim.py")
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _source_state() -> RuntimeState:
    values = (
        1.0,
        2.0,
        3.0,
        100.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        14641.0545,
        0.0,
        0.0,
        0.0,
    )
    return RuntimeState(
        time=0.0,
        values=values,
        frame=Frame.ECIC,
        named=dict(zip(RIGID_BODY_STATE_NAMES, values, strict=True)),
        value_names=RIGID_BODY_STATE_NAMES,
    )
    ####


def test_x15_alpha_adapter_perturbs_body_attitude_not_guidance_text() -> None:
    base = _source_state()

    perturbed = MODULE._state_at_alpha(base, 0.0, 10.0)

    assert perturbed.named["qw"] == pytest.approx(0.9961946981)
    assert perturbed.named["qy"] == pytest.approx(0.0871557427)
    assert perturbed.named["qx"] == pytest.approx(0.0)
    assert perturbed.named["qz"] == pytest.approx(0.0)
    assert perturbed.named["vx"] == pytest.approx(base.named["vx"])
    assert perturbed.named["mass"] == pytest.approx(base.named["mass"])
    ####


def test_x15_trim_report_preserves_source_only_diagnostic() -> None:
    report_path = ROOT / "artifacts/golden_plants/x15_release_glide_trim_report.json"
    if not report_path.is_file():
        pytest.skip("generated X-15 trim evidence is not present")
    import json

    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["status"] == "blocked"
    assert report["diagnostic"]["source_only_preserved"] is True
    assert report["diagnostic"]["source_release_is_equilibrium_claim"] is False
    assert report["solver"]["tables_rebound_per_evaluation"] is False
    ####
