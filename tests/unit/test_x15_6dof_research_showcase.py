from __future__ import annotations

from pathlib import Path

from taoryx.language import GrammarProfile
from taoryx.runtime.runner import run_files

ROOT = Path("tests/fixtures/x15_coherent_6dof_public_research_v1/tables")
MISSION = Path("examples/showcases/x15_6dof_research/mission.prb")


def test_x15_standard_problem_runs_native_6dof_with_source_tables(tmp_path: Path) -> None:
    report = run_files(
        MISSION,
        (
            ROOT / "x15_symmetric_stabilator_6axis.tbl",
            ROOT / "x15_xlr99_thrust_mdot.tbl",
        ),
        output_dir=tmp_path,
        max_steps=300,
        profile=GrammarProfile.TAORYX,
    )

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    assert report.results[0].completed
    final = report.results[0].states["1"][-1]
    assert final.time == 1.0
    assert final.named["aero_active"] == 1.0
    assert final.named["aero_mach"] <= 6.7
    assert final.named["aero_density_kg_m3"] >= 0.0
    assert final.named["force_body_x_n"] != 0.0
    assert final.named["moment_body_y_nm"] != 0.0
    assert report.metadata[0]["native_pipeline"]["integration_frame"] == "ecic"
    assert report.metadata[0]["native_pipeline"]["table_binding"] == "explicit-segment-aero"
    ####
