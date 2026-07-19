from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.language import GrammarProfile
from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[2]
PROBLEM = ROOT / "examples/mission_families/slower_hummingbird/SV05_rate_damped_hover_6dof.prb"
TABLES = tuple(ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables" / name for name in (
    "hummingbird_cx.tbl", "hummingbird_cy.tbl", "hummingbird_cz.tbl",
    "hummingbird_cmx.tbl", "hummingbird_cmy.tbl", "hummingbird_cmz.tbl",
))


@pytest.mark.slow
@pytest.mark.dof6
def test_hummingbird_rate_damping_uses_individual_rotor_commands(tmp_path: Path) -> None:
    report = run_files(PROBLEM, TABLES, output_dir=tmp_path, max_steps=700, profile=GrammarProfile.TAORYX)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    assert report.results and report.results[0].completed
    history = report.results[0].states["1"]
    initial_rate = sum(history[0].named[name] ** 2 for name in ("wx", "wy", "wz")) ** 0.5
    final_rate = sum(history[-1].named[name] ** 2 for name in ("wx", "wy", "wz")) ** 0.5
    assert final_rate < initial_rate
    assert max(abs(state.named["aero_moment_body_x_nm"]) for state in history) > 1.0e-4
    assert max(abs(state.named["aero_moment_body_y_nm"]) for state in history) > 1.0e-4
    assert max(abs(state.named["aero_moment_body_z_nm"]) for state in history) > 1.0e-4
    assert max(abs(state.named["aero_query_rotor-1-speed"] - state.named["aero_query_rotor-2-speed"]) for state in history) > 1.0e-3
    assert all(0.0 <= state.named[f"aero_query_rotor-{index}-speed"] <= 1500.0 for state in history for index in range(1, 5))
    ####
