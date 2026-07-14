from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from taoryx.language.ingest import ingest_file
from taoryx.language.semantic_validation import validate_problem, validate_table_file
from taoryx.language.table_parser import table_type_catalog

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
