"""Scoped, context-bounded parsers for raw ``.prb`` manual displays."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel

from taoryx.language.diagnostics import Diagnostic, SourceLocation
from taoryx.language.models import OptimizeBlock, OptimizeBodyFragment, ProblemFragmentDocument
from taoryx.language.problem_parser import parse_problem_text

_CONTEXTUAL_FRAGMENT_DIAGNOSTICS = frozenset(
    {
        "inertial-body-first-segment",
        "missing-egs-summary-survey",
        "missing-egs-summary-variable",
        "missing-segment-when",
        "missing-trajectory-initial",
        "missing-trajectory-segment",
        "wildcard-fly-in-first-segment",
    }
)


def _shift_value(value, offset: int):
    if isinstance(value, SourceLocation):
        return value.model_copy(update={"line": max(1, value.line - offset)})
    if isinstance(value, BaseModel):
        return value.model_copy(
            update={field: _shift_value(getattr(value, field), offset) for field in type(value).model_fields}
        )
    if isinstance(value, list):
        return [_shift_value(item, offset) for item in value]
    if isinstance(value, tuple):
        return tuple(_shift_value(item, offset) for item in value)
    if isinstance(value, dict):
        return {key: _shift_value(item, offset) for key, item in value.items()}
    return value
####


def _first_content_line(text: str) -> str:
    return next((line for line in text.splitlines() if line.strip()), "")
####


def _wrapper_for_scope(text: str, scope: Literal["problem", "trajectory", "segment"]) -> tuple[str, int]:
    first = _first_content_line(text).lstrip().casefold()
    if scope == "problem":
        prefix = "(fragment)\n"
    elif scope == "trajectory":
        prefix = "(fragment)\n"
        if not first.startswith("*trajectory"):
            prefix += "*trajectory 1 fragment start on 1\n"
    else:
        prefix = "(fragment)\n*trajectory 1 fragment start on 1\n"
        if not first.startswith("*segment"):
            prefix += "*segment 1 fragment\n"
    ####
    return prefix + text + "\n*end\n", prefix.count("\n")
####


def _select_blocks(document, scope: Literal["problem", "trajectory", "segment"]):
    if not document.problems:
        return []
    problem = document.problems[0]
    if scope == "problem":
        return list(problem.blocks)
    if not problem.trajectories:
        return []
    trajectory = problem.trajectories[0]
    if scope == "trajectory":
        return list(trajectory.blocks)
    return list(trajectory.segments[0].blocks) if trajectory.segments else []
####


def parse_problem_fragment(
    text: str,
    *,
    scope: Literal["problem", "trajectory", "segment"],
    path: str = "<memory>",
) -> ProblemFragmentDocument:
    """Parse a raw problem-language block with an explicitly supplied scope.

    The temporary framing is never exposed. Locations and source lines are
    shifted back to the fragment, and diagnostics that require omitted
    trajectory/segment context are suppressed rather than guessed.
    """

    wrapped, offset = _wrapper_for_scope(text, scope)
    parsed = parse_problem_text(wrapped, path)
    diagnostics: list[Diagnostic] = []
    recovered = []
    deferred_diagnostics: list[Diagnostic] = []
    deferred_recovered = []
    for diagnostic in parsed.diagnostics:
        if diagnostic.code in _CONTEXTUAL_FRAGMENT_DIAGNOSTICS:
            deferred_diagnostics.append(_shift_value(diagnostic, offset))
            continue
        diagnostics.append(_shift_value(diagnostic, offset))
    ####
    for record in parsed.recovered_records:
        if record.code in _CONTEXTUAL_FRAGMENT_DIAGNOSTICS:
            deferred_recovered.append(_shift_value(record, offset))
            continue
        recovered.append(_shift_value(record, offset))
    ####
    diagnostics.sort(key=lambda item: (item.location.line if item.location else 0, item.location.column if item.location else 0))
    recovered.sort(key=lambda item: (item.location.line, item.location.column))
    deferred_diagnostics.sort(key=lambda item: (item.location.line if item.location else 0, item.location.column if item.location else 0))
    deferred_recovered.sort(key=lambda item: (item.location.line, item.location.column))
    blocks = [_shift_value(block, offset) for block in _select_blocks(parsed, scope)]
    return ProblemFragmentDocument(
        source_text=text,
        scope=scope,
        blocks=blocks,
        diagnostics=diagnostics,
        recovered_records=recovered,
        deferred_diagnostics=deferred_diagnostics,
        deferred_recovered_records=deferred_recovered,
    )
####


def parse_optimize_body_fragment(text: str, path: str = "<memory>") -> OptimizeBodyFragment:
    """Parse optimize constraints/controls without fabricating an optimize header."""

    prefix = "(fragment)\n*optimize a for vel=max on segment 1, trajectory 1\n"
    parsed = parse_problem_text(prefix + text + "\n*end\n", path)
    diagnostics = [_shift_value(item, 2) for item in parsed.diagnostics]
    recovered = [_shift_value(item, 2) for item in parsed.recovered_records]
    block = next(
        (item for item in parsed.problems[0].blocks if isinstance(item, OptimizeBlock)),
        None,
    ) if parsed.problems else None
    return OptimizeBodyFragment(
        source_text=text,
        constraints=_shift_value(block.constraints, 2) if block is not None else [],
        controls=_shift_value(block.controls, 2) if block is not None else [],
        diagnostics=diagnostics,
        recovered_records=recovered,
    )
####
