from __future__ import annotations

import json
from pathlib import Path

import pytest

from taoryx.language import GrammarProfile
from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[2]
MATRIX = json.loads((ROOT / "tests/fixtures/dof_robustness_v1/manifest.json").read_text(encoding="utf-8"))
AERO = ROOT / "examples/showcases/california_to_hawaii/aero.tbl"
THREE_DOF_IDS = {case["id"] for case in MATRIX["cases"]}


@pytest.mark.dof6
@pytest.mark.slow
@pytest.mark.parametrize("case", MATRIX["six_dof_cases"], ids=lambda item: item["id"])
def test_six_dof_robustness_baseline(case: dict[str, str], tmp_path: Path) -> None:
    """Run reusable rigid-body family problems through the source runner."""

    assert case["reference_3dof"] in THREE_DOF_IDS
    problem = ROOT / case["path"]
    tables = (ROOT / case["table"],) if case["kind"] in {"route", "saturation", "wind", "pitch", "bank"} else ((AERO,) if case["kind"] == "aero" else ())
    max_steps = 3_000 if case["kind"] in {"route", "wind"} else (2_000 if case["kind"] == "saturation" else (200 if case["kind"] == "ballistic" else 100))
    report = run_files(problem, tables, output_dir=tmp_path / case["id"], max_steps=max_steps, profile=GrammarProfile.TAORYX)

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    assert report.results and report.results[0].completed
    history = report.results[0].states["1"]
    assert len(history) >= 2
    final = history[-1]
    quaternion_norm = sum(final.named[name] ** 2 for name in ("qw", "qx", "qy", "qz"))
    assert quaternion_norm == pytest.approx(1.0, abs=1.0e-9)
    assert all(value == value and abs(value) != float("inf") for state in history for value in state.values)
    if case["kind"] == "route":
        assert final.named["range_to_target_m"] >= 0.0
        assert final.named["pro_nav_los_range_m"] >= 0.0
    elif case["kind"] == "saturation":
        assert any(state.named["attitude_controller_saturated"] == pytest.approx(1.0) for state in history)
        assert all(abs(state.named["wz"]) <= 1.0e6 for state in history)
    elif case["kind"] == "wind":
        assert final.named["aero_active"] == pytest.approx(1.0)
        assert max(abs(state.named.get("aero_sideslip_deg", 0.0)) for state in history) > 1.0e-6
        assert max(abs(state.named.get("aero_moment_body_z_nm", 0.0)) for state in history) > 0.0
    elif case["kind"] == "pitch":
        assert max(abs(state.named["aero_alpha_deg"]) for state in history) >= 4.0
        assert max(abs(state.named["aero_moment_body_y_nm"]) for state in history) > 0.0
    elif case["kind"] == "bank":
        assert max(abs(state.named["aero_moment_body_x_nm"]) for state in history) > 0.0
    elif case["kind"] == "propulsion":
        assert final.named["propellant_mass"] < history[0].named["propellant_mass"]
    elif case["kind"] == "ballistic":
        assert final.named["altitude_m"] >= 0.0
    else:
        assert final.named["aero_active"] == pytest.approx(1.0)
        assert "attitude_controller_saturated" in final.named
    ####
