from __future__ import annotations

from collections import Counter

from taoryx.language.diagnostics import Diagnostic, Severity
from taoryx.language.expressions import (
    IndexedExpression,
    ParameterExpression,
    TableReferenceExpression,
)
from taoryx.language.models import ProblemDocument, TableDocument, WhenBlock


def _walk_expression(expression):
    yield expression
    if hasattr(expression, "operand"):
        yield from _walk_expression(expression.operand)
    ####
    if hasattr(expression, "left"):
        yield from _walk_expression(expression.left)
        yield from _walk_expression(expression.right)
    ####
    if hasattr(expression, "arguments"):
        for argument in expression.arguments:
            yield from _walk_expression(argument)
        ####
    ####
####


def validate_problem(document: ProblemDocument, available_tables: set[str] | None = None) -> list[Diagnostic]:
    diagnostics = list(document.diagnostics)
    available_tables = available_tables or set()
    for problem in document.problems:
        trajectory_numbers = [trajectory.number for trajectory in problem.trajectories]
        for number, count in Counter(trajectory_numbers).items():
            if count > 1:
                diagnostics.append(Diagnostic(severity=Severity.ERROR, code="duplicate-trajectory", message=f"Trajectory {number} is declared more than once.", location=problem.location))
            ####
        ####
        trajectory_set = set(trajectory_numbers)
        for trajectory in problem.trajectories:
            segment_numbers = [segment.number for segment in trajectory.segments]
            segment_set = set(segment_numbers)
            if trajectory.start_segment not in segment_set:
                diagnostics.append(Diagnostic(severity=Severity.ERROR, code="missing-start-segment", message=f"Trajectory {trajectory.number} starts on undefined segment {trajectory.start_segment}.", location=trajectory.location))
            ####
            for number, count in Counter(segment_numbers).items():
                if count > 1:
                    diagnostics.append(Diagnostic(severity=Severity.ERROR, code="duplicate-segment", message=f"Segment {number} is declared more than once in trajectory {trajectory.number}.", location=trajectory.location))
                ####
            ####
            for block in trajectory.blocks:
                if block.keyword == "initial" and block.source_trajectory is not None and block.source_trajectory not in trajectory_set:
                    diagnostics.append(Diagnostic(severity=Severity.ERROR, code="unknown-initial-trajectory", message=f"Initial state references undefined trajectory {block.source_trajectory}.", location=block.location))
                ####
            ####
            for segment in trajectory.segments:
                for block in segment.blocks:
                    if isinstance(block, WhenBlock) and block.action == "goto" and block.target_segment not in segment_set:
                        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="unknown-goto-segment", message=f"Segment {segment.number} jumps to undefined segment {block.target_segment}.", location=block.location))
                    ####
                    for assignment in block.assignments:
                        for node in _walk_expression(assignment.value):
                            if isinstance(node, TableReferenceExpression) and available_tables and node.name not in available_tables:
                                diagnostics.append(Diagnostic(severity=Severity.WARNING, code="external-table-reference", message=f"Table {node.name!r} is not present in the supplied table set.", location=assignment.location))
                            elif isinstance(node, IndexedExpression) and node.index not in trajectory_set:
                                diagnostics.append(Diagnostic(severity=Severity.WARNING, code="unknown-indexed-trajectory", message=f"Indexed variable {node.name}[{node.index}] references an undefined trajectory.", location=assignment.location))
                            elif isinstance(node, ParameterExpression) and node.index <= 0:
                                diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-parameter-index", message="TAOS parameter indices must be positive.", location=assignment.location))
                            ####
                        ####
                    ####
                ####
            ####
        ####
        if not problem.ended:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="missing-end", message=f"Problem {problem.name!r} does not end with '*end'.", location=problem.location))
        ####
    ####
    return diagnostics
####


def validate_table_file(document: TableDocument) -> list[Diagnostic]:
    diagnostics = list(document.diagnostics)
    names = [table.name for table in document.tables]
    for name, count in Counter(names).items():
        if count > 1:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="duplicate-table", message=f"Table {name!r} is declared more than once."))
        ####
    ####
    return diagnostics
####
