from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.language import GrammarProfile
from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[2]
PROBLEM = ROOT / "examples/mission_families/slower_hummingbird/SV05_differential_allocation_6dof.prb"
TABLES = tuple(ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables" / name for name in (
    "hummingbird_cx.tbl",
    "hummingbird_cy.tbl",
    "hummingbird_cz.tbl",
    "hummingbird_cmx.tbl",
    "hummingbird_cmy.tbl",
    "hummingbird_cmz.tbl",
))


@pytest.mark.slow
@pytest.mark.dof6
def test_hummingbird_individual_rotor_allocation_reaches_runtime_wrench(tmp_path: Path) -> None:
    report = run_files(PROBLEM, TABLES, output_dir=tmp_path, max_steps=500, profile=GrammarProfile.TAORYX)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    assert report.results and report.results[0].completed
    history = report.results[0].states["1"]
    assert max(abs(state.named["aero_moment_body_x_nm"]) for state in history) > 1.0e-4
    assert max(abs(state.named["aero_moment_body_y_nm"]) for state in history) < 1.0e-8
    assert max(abs(state.named["aero_moment_body_z_nm"]) for state in history) < 1.0e-8
    assert history[0].named["translation_equation_residual_normalized"] < 1.0e-8
    assert history[0].named["rotation_equation_residual_normalized"] < 1.0e-8
    ####
