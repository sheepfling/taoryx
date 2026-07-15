from __future__ import annotations

import pytest

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language.problem_parser import parse_problem_text
from taoryx.language.semantic_validation import validate_problem


def _problem(intervals: tuple[tuple[int, int], ...]) -> str:
    max_segment = max(end for _, end in intervals)
    lines = [
        "(search-topology)",
        "*atmos none",
        "*earth spherical gm=0 omega=0",
        "*trajectory 1 vehicle start on 1",
        "  *initial ecfc x=0 y=0 z=0 xdt=0 ydt=0 zdt=0 time=0 mass=1",
    ]
    for segment_number in range(1, max_segment + 1):
        search_id = _search_for_start(intervals, segment_number)
        lines.append(f"  *segment {segment_number}")
        if search_id:
            lines.append(f"    *fly alpha=srch-{search_id}")
        lines.append("    *when time=0 stop")
    for search_id, (_, end) in enumerate(intervals, start=1):
        lines.extend(
            (
                f"*search {search_id} vary alpha until alt=0 on segment {end}, trajectory 1",
                "  xlo=0 xhi=10 xest=5 dx=1 tol=0.001",
            )
        )
    lines.append("*end")
    return "\n".join(lines) + "\n"
####


def _search_for_start(intervals: tuple[tuple[int, int], ...], segment: int) -> int | None:
    for search_id, (start, _) in enumerate(intervals, start=1):
        if start == segment:
            return search_id
    return None
####


@pytest.mark.parametrize("intervals", [((1, 1), (3, 3), (5, 5)), ((1, 5), (2, 4), (3, 3))])
def test_three_searches_accept_disjoint_and_wholly_nested_topologies(intervals: tuple[tuple[int, int], ...]) -> None:
    document = parse_problem_text(_problem(intervals))
    diagnostics = validate_problem(document)

    assert not [diagnostic for diagnostic in diagnostics if diagnostic.severity.value == "error"]
####


def test_three_searches_reject_chained_partial_overlaps() -> None:
    document = parse_problem_text(_problem(((1, 3), (2, 4), (3, 5))))

    diagnostics = [diagnostic for diagnostic in validate_problem(document) if diagnostic.code == "partially-overlapping-searches"]
    assert len(diagnostics) == 2
    assert all(diagnostic.severity.value == "error" for diagnostic in diagnostics)
####


def test_partial_search_overlaps_are_a_taoryx_extension() -> None:
    document = parse_problem_text(_problem(((1, 3), (2, 4), (3, 5))), profile=GrammarProfile.TAORYX)

    diagnostics = validate_problem(document)

    assert not [diagnostic for diagnostic in diagnostics if diagnostic.code == "partially-overlapping-searches"]
####
