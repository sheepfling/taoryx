from pathlib import Path

import pytest

from taoryx.language.models import OptimizeBlock, WhenBlock
from taoryx.language.problem_parser import parse_problem_file
from taoryx.language.semantic_validation import validate_problem

FIXTURES = sorted(Path("examples/chapter04").glob("*.prb"))


@pytest.mark.parametrize("path", FIXTURES)
def test_manual_problem_fixture_parses(path: Path) -> None:
    document = parse_problem_file(path)
    assert len(document.problems) == 1
    assert not [item for item in document.diagnostics if item.severity == "error"]
    assert document.problems[0].ended
####


def test_air_intercept_has_typed_optimize_and_when_blocks() -> None:
    document = parse_problem_file(Path("examples/chapter04/air-launched-intercept.prb"))
    problem = document.problems[0]
    assert any(isinstance(block, OptimizeBlock) for block in problem.blocks)
    assert any(isinstance(block, WhenBlock) for trajectory in problem.trajectories for segment in trajectory.segments for block in segment.blocks)
####


@pytest.mark.parametrize("path", FIXTURES)
def test_manual_problem_fixture_semantics(path: Path) -> None:
    document = parse_problem_file(path)
    errors = [item for item in validate_problem(document) if item.severity == "error"]
    assert not errors, [item.model_dump() for item in errors]
####
