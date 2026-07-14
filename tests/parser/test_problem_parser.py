from pathlib import Path

import pytest

from taoryx.language.ingest import FileKind, ingest_text
from taoryx.language.models import OptimizeBlock, WhenBlock
from taoryx.language.problem_parser import parse_problem_file, parse_problem_text
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


def test_survey_placeholder_must_match_declared_survey() -> None:
    document = parse_problem_text(
        """(survey-check)
*survey 1 altitude lo=0 hi=10 inc=5
*trajectory 1 Test start on 1
  *initial geodetic
    long=0 lat=0 alt=surv-2 vel=1 gama=0 psi=0 time=0 wt=1
  *segment 1
    *when time>1 stop
*end
"""
    )

    diagnostics = validate_problem(document)

    matches = [item for item in diagnostics if item.code == "unknown-survey-reference"]
    assert len(matches) == 1
    assert matches[0].location.line == 5
####


def test_declared_survey_placeholder_is_valid() -> None:
    document = parse_problem_text(
        """(survey-check)
*survey 1 altitude lo=0 hi=10 inc=5
*trajectory 1 Test start on 1
  *initial geodetic
    long=0 lat=0 alt=surv-1 vel=1 gama=0 psi=0 time=0 wt=1
  *segment 1
    *when time>1 stop
*end
"""
    )

    diagnostics = validate_problem(document)

    assert not [item for item in diagnostics if item.code == "unknown-survey-reference"]


def test_when_rejects_compound_conditions_but_preserves_typed_header() -> None:
    document = parse_problem_text(
        """(when-check)
*trajectory 1 Test start on 1
  *initial geodetic
    long=0 lat=0 alt=0 vel=1 gama=0 psi=0 time=0 wt=1
  *segment 1
    *when alt<200000&&time>1 goto 2
  *segment 2
    *when time>2 stop
*end
"""
    )

    diagnostic = next(item for item in document.diagnostics if item.code == "unsupported-when-condition")
    assert diagnostic.location.line == 6
    when = document.problems[0].trajectories[0].segments[0].blocks[0]
    assert isinstance(when, WhenBlock)
    assert when.condition is not None
    assert when.action == "goto"
    assert when.target_segment == 2
    assert any(record.code == "unsupported-when-condition" for record in document.recovered_records)
    ####
####


def test_search_placeholder_requires_matching_search_block_through_ingestion() -> None:
    result = ingest_text(
        """(search-check)
*trajectory 1 Test start on 1
  *initial geodetic
    long=0 lat=0 alt=0 vel=1 gama=0 psi=0 time=0 wt=1
  *segment 1
    *fly alpha=srch-1
    *when time>1 stop
*end
""",
        kind=FileKind.PROBLEM,
    )

    diagnostic = next(item for item in result.diagnostics if item.code == "unknown-search-reference")
    assert diagnostic.location.line == 6
    assert "srch-1" in result.source.render_text()
    block = result.document.problems[0].trajectories[0].segments[0].blocks[0]
    assert block.value.family == "search"
    assert block.value.index == 1
####


def test_declared_search_placeholder_is_valid_through_ingestion() -> None:
    result = ingest_text(
        """(search-check)
*trajectory 1 Test start on 1
  *initial geodetic
    long=0 lat=0 alt=0 vel=1 gama=0 psi=0 time=0 wt=1
  *segment 1
    *fly alpha=srch-1
    *when time>1 stop
*search 1 vary alpha until alt=0 on segment 1, trajectory 1
  xlo=0 xhi=1 xest=0.5 dx=0.1
*end
""",
        kind=FileKind.PROBLEM,
    )

    assert not [item for item in result.diagnostics if item.code == "unknown-search-reference"]
####


def test_complete_search_requires_the_four_mandatory_controls() -> None:
    result = ingest_text(
        """(search-check)
*search 1 vary alpha until alt=0 on segment 1, trajectory 1
  xlo=0 xest=0.5
*trajectory 1 Test start on 1
  *initial geodetic
    long=0 lat=0 alt=0 vel=1 gama=0 psi=0 time=0 wt=1
  *segment 1
    *when time>1 stop
*end
""",
        kind=FileKind.PROBLEM,
    )

    diagnostics = [item for item in result.diagnostics if item.code == "missing-search-control"]
    assert {required for required in ("xhi", "dx") if f"'{required}'" in " ".join(item.message for item in diagnostics)} == {"xhi", "dx"}
    assert all(item.location.line == 2 for item in diagnostics)


def test_semantic_validation_rejects_unknown_search_endpoint_segment() -> None:
    document = parse_problem_text(
        """(endpoint-check)
*search 1 vary alpha until alt=10 on segment 9, trajectory 1
  xlo=0 xhi=1 xest=0.5 dx=0.1
*trajectory 1 Test start on 1
  *initial geodetic
    long=0 lat=0 alt=0 vel=1 gama=0 psi=0 time=0 wt=1
  *segment 1
    *when time>1 stop
*end
"""
    )

    diagnostics = validate_problem(document)
    assert any(item.code == "unknown-endpoint-segment" for item in diagnostics)


def test_search_accepts_every_documented_control_variable() -> None:
    result = ingest_text(
        """(search-controls)
*search 1 vary alpha until alt=30000 on segment 3, trajectory 1
  xlo=0.5 xhi=10.0 xest=5.0 dx=1.0 tol=0.001
  xref=1.0 fref=10000 maxitr=20 integ=0 print=1
*trajectory 1 Test start on 1
  *initial geodetic
    long=0 lat=0 alt=0 vel=1 gama=0 psi=0 time=0 wt=1
  *segment 1
    *when time>1 stop
*end
""",
        kind=FileKind.PROBLEM,
    )

    search = result.document.problems[0].blocks[0]
    assert {assignment.name for assignment in search.controls} == {
        "xlo", "xhi", "xest", "dx", "tol", "xref", "fref", "maxitr", "integ", "print"
    }
    assert not [item for item in result.diagnostics if item.code == "unsupported-search-control"]
####


def test_complete_optimize_requires_initial_parameters_for_optional_bounds() -> None:
    result = ingest_text(
        """(optimize-check)
*optimize a for alt=max on segment 1, trajectory 1
  lo-1=0 hi-2=10 ref-3=1
*trajectory 1 Test start on 1
  *initial geodetic
    long=0 lat=0 alt=0 vel=1 gama=0 psi=0 time=0 wt=1
  *segment 1
    *when time>1 stop
*end
""",
        kind=FileKind.PROBLEM,
    )

    codes = [item.code for item in result.diagnostics]
    assert codes.count("missing-optimize-parameters") == 1
    assert codes.count("orphan-optimize-parameter-bound") == 3
    assert all(item.location.line == 2 for item in result.diagnostics if item.code.startswith(("missing-optimize", "orphan-optimize")))
####


def test_optimize_parameter_numbers_must_be_sequential() -> None:
    result = ingest_text(
        """(optimize-check)
*optimize a for alt=max on segment 1, trajectory 1
  par-1=0 par-3=1
*trajectory 1 Test start on 1
  *initial geodetic
    long=0 lat=0 alt=0 vel=1 gama=0 psi=0 time=0 wt=1
  *segment 1
    *when time>1 stop
*end
""",
        kind=FileKind.PROBLEM,
    )

    assert any(item.code == "nonsequential-optimize-parameters" for item in result.diagnostics)
####


def test_duplicate_optimize_loops_are_ambiguous_and_preserved() -> None:
    result = ingest_text(
        """(optimize-check)
*optimize a for alt=max on segment 1, trajectory 1
  par-1=0
*optimize A for range=min on segment 2, trajectory 1
  par-1=1
*trajectory 1 Test start on 1
  *initial geodetic
    long=0 lat=0 alt=0 vel=1 gama=0 psi=0 time=0 wt=1
  *segment 1
    *fly alpha=opta-1
    *when time>1 stop
*end
""",
        kind=FileKind.PROBLEM,
    )

    assert len(result.document.problems[0].blocks) == 2
    assert any(item.code == "duplicate-optimize-loop" for item in result.diagnostics)
    assert any(item.code == "unknown-optimize-loop" for item in result.diagnostics)
    ####


def test_duplicate_optimize_controls_are_diagnosed_without_overwriting_source() -> None:
    result = ingest_text(
        """(optimize-check)
*optimize a for alt=max on segment 1, trajectory 1
  par-1=0 par-1=1 fref=1 fref=2
*trajectory 1 Test start on 1
  *initial geodetic
    long=0 lat=0 alt=0 vel=1 gama=0 psi=0 time=0 wt=1
  *segment 1
    *when time>1 stop
*end
""",
        kind=FileKind.PROBLEM,
    )

    assert [assignment.name for assignment in result.document.problems[0].blocks[0].controls] == [
        "par-1", "par-1", "fref", "fref"
    ]
    assert sum(item.code == "duplicate-optimize-control" for item in result.diagnostics) == 2
    ####


def test_optimization_placeholder_requires_matching_loop_and_parameter() -> None:
    result = ingest_text(
        """(optimize-check)
*optimize a for alt=max on segment 1, trajectory 1
  par-1=0
*trajectory 1 Test start on 1
  *initial geodetic
    long=0 lat=0 alt=0 vel=1 gama=0 psi=0 time=0 wt=1
  *segment 1
    *fly alpha=optb-1
    *when time>opta-2 stop
*end
""",
        kind=FileKind.PROBLEM,
    )

    codes = {item.code for item in result.diagnostics}
    assert {"unknown-optimize-loop", "unknown-optimize-parameter"} <= codes
    assert all(item.location.line in {8, 9} for item in result.diagnostics if item.code.startswith("unknown-optimize"))
####


@pytest.mark.parametrize("path", FIXTURES)
def test_manual_problem_fixture_semantics(path: Path) -> None:
    document = parse_problem_file(path)
    errors = [item for item in validate_problem(document) if item.severity == "error"]
    assert not errors, [item.model_dump() for item in errors]
####
