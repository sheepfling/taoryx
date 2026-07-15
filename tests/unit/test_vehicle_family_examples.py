from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from taoryx.language.ingest import ingest_file
from taoryx.language.semantic_validation import validate_problem, validate_table_file
from taoryx.language.table_parser import table_type_catalog
from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[1].parent / "examples" / "vehicle_families"
FAMILY = ROOT / "staged_rocket" / "two_stage_demo"

pytestmark = pytest.mark.table


def test_two_stage_family_declares_scope_and_sources() -> None:
    metadata = yaml.safe_load((FAMILY / "family.yaml").read_text(encoding="utf-8"))

    assert metadata["provenance"] == "synthetic-smoke-test"
    assert metadata["mission"] == "mission.prb"
    assert set(metadata["tables"]) == {"aero.tbl", "propulsion.tbl"}
    assert "six-degree-of-freedom rigid-body dynamics" in metadata["not_claimed"]
####


def test_two_stage_family_files_are_lossless_and_semantically_linked() -> None:
    table_documents = [ingest_file(FAMILY / filename) for filename in ("aero.tbl", "propulsion.tbl")]
    table_types = {}
    for result in table_documents:
        assert result.source.render_bytes() == (FAMILY / Path(result.source.source_path).name).read_bytes()
        assert not [item for item in validate_table_file(result.document) if item.severity.value == "error"]
        table_types.update(table_type_catalog(result.document))
    ####

    problem = ingest_file(FAMILY / "mission.prb", available_tables=table_types)
    assert problem.source.render_bytes() == (FAMILY / "mission.prb").read_bytes()
    assert not [item for item in validate_problem(problem.document, available_tables=table_types) if item.severity.value == "error"]
    trajectories = problem.document.problems[0].trajectories
    assert {trajectory.number for trajectory in trajectories} == {1, 2}
    assert trajectories[1].start_segment == 1
    assert any(block.source_trajectory == 1 and block.source_segment == 20 for block in trajectories[1].blocks if hasattr(block, "source_trajectory"))
    ####


def test_two_stage_family_executes_stage_transitions_and_inheritance(tmp_path: Path) -> None:
    report = run_files(
        FAMILY / "mission.prb",
        (FAMILY / "aero.tbl", FAMILY / "propulsion.tbl"),
        output_dir=tmp_path,
        max_steps=5_000,
    )

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    assert len(report.results) == 1
    result = report.results[0]
    assert result.completed
    assert set(result.states) == {"1", "2"}
    stack = result.states["1"]
    booster = result.states["2"]
    assert max(state.named["wt"] for state in stack) > min(state.named["wt"] for state in stack)
    assert stack[-1].named["wt"] == pytest.approx(492.0)
    assert booster[0].named["wt"] == pytest.approx(250.0)
    assert booster[-1].time > booster[0].time
    ####
