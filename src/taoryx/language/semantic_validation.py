from __future__ import annotations

from collections import Counter
from collections.abc import Collection, Iterator, Mapping

from pydantic import BaseModel

from taoryx.language.diagnostics import Diagnostic, Severity
from taoryx.language.expressions import (
    CallExpression,
    Expression,
    IndexedExpression,
    NameExpression,
    ParameterExpression,
    TableReferenceExpression,
)
from taoryx.language.grammar_contracts import DOCUMENTED_STATE_VARIABLES, GrammarProfile
from taoryx.language.models import (
    DefineBlock,
    EgsBlock,
    FileBlock,
    OptimizeBlock,
    OptimizeEndpoint,
    PrintBlock,
    ProblemDocument,
    SearchBlock,
    SurveyBlock,
    TableDocument,
    UnitsFormatBlock,
    WhenBlock,
)
from taoryx.runtime.units import variable_dimension

TABLE_TYPE_REFERENCES = frozenset(
    {
        "ca", "cn", "cl", "cd", "cs", "cx", "cy", "cz",
        "thrust", "tvec1", "tvec2", "mdot", "cg",
        "windv", "windh", "winde", "windn", "windd",
    }
)

_USER_VARIABLE_BLOCK_FIXED_NAMES = {
    "aero": frozenset({"ca", "cn", "cl", "cd", "cs", "cx", "cy", "cz", "sref"}),
    "cg": frozenset({"cg"}),
    "constants": frozenset(),
    "prop": frozenset({"thrust", "mdot", "ep1", "ep2", "thr_units", "mdt_units"}),
}


def _source_order(diagnostics: list[Diagnostic]) -> list[Diagnostic]:
    """Return diagnostics in stable physical source order.

    Parser diagnostics already arrive in this order, but semantic validation
    appends findings while traversing the typed model.  Sorting at the shared
    validation boundary keeps the aggregate route deterministic without
    changing the encounter order of diagnostics on the same source position.
    """
    return sorted(
        diagnostics,
        key=lambda diagnostic: (
            diagnostic.location.line if diagnostic.location is not None else 2**31,
            diagnostic.location.column if diagnostic.location is not None else 2**31,
        ),
    )
####


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


def _walk_model_expressions(value: object, location: object) -> Iterator[tuple[Expression, object]]:
    """Yield expressions with the nearest owning model location.

    Expression nodes do not carry source locations themselves.  Their owning
    assignment or block does, so retaining that location gives semantic
    diagnostics a useful source anchor without inventing expression offsets.
    """
    if isinstance(value, Expression):
        for expression in _walk_expression(value):
            yield expression, location
        ####
        return
    ####
    if isinstance(value, BaseModel):
        model_location = getattr(value, "location", location)
        for child in value.__dict__.values():
            yield from _walk_model_expressions(child, model_location)
        ####
        return
    ####
    if isinstance(value, (list, tuple, set)):
        for child in value:
            yield from _walk_model_expressions(child, location)
        ####
    ####


def _normalize_variable_name(name: str) -> str:
    return name.casefold().split("[", 1)[0]
####


def _validate_search_topology(problem, search_blocks: list[SearchBlock], diagnostics: list[Diagnostic]) -> None:
    """Reject partially intersecting search loops on the same trajectory."""

    intervals: list[tuple[int, int, int, SearchBlock]] = []
    for search in search_blocks:
        if search.search_id is None or search.objective is None or search.objective.left.segment is None:
            continue
        trajectory_number = search.objective.left.trajectory or search.objective.left.trajectory_subscript
        if trajectory_number is None:
            continue
        trajectory = next((item for item in problem.trajectories if item.number == trajectory_number), None)
        if trajectory is None:
            continue
        start_segment = _search_start_segment(trajectory, search.search_id)
        if start_segment is None:
            continue
        end_segment = search.objective.left.segment
        if start_segment > end_segment:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="invalid-search-interval",
                    message=f"Search {search.search_id} starts on segment {start_segment} after its objective segment {end_segment}.",
                    location=search.location,
                )
            )
            continue
        intervals.append((trajectory_number, start_segment, end_segment, search))
    ####
    for index, (trajectory, first_start, first_end, first_search) in enumerate(intervals):
        for _, second_start, second_end, second_search in intervals[index + 1 :]:
            if trajectory != _search_trajectory(second_search):
                continue
            if _search_intervals_partially_overlap(first_start, first_end, second_start, second_end):
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="partially-overlapping-searches",
                        message=(
                            f"Searches {first_search.search_id} and {second_search.search_id} partially overlap "
                            f"on trajectory {trajectory}; search intervals must be disjoint or wholly nested."
                        ),
                        location=second_search.location,
                    )
                )
            ####
        ####
    ####
####


def _search_start_segment(trajectory, search_id: int) -> int | None:
    """Return the first segment containing a search placeholder."""

    for segment in sorted(trajectory.segments, key=lambda item: item.location.line):
        if any(
            isinstance(expression, ParameterExpression)
            and expression.family == "search"
            and expression.index == search_id
            for expression, _ in _walk_model_expressions(segment, segment.location)
        ):
            return segment.number
        ####
    ####
    if any(
        isinstance(expression, ParameterExpression)
        and expression.family == "search"
        and expression.index == search_id
        for block in trajectory.blocks
        for expression, _ in _walk_model_expressions(block, block.location)
    ):
        return trajectory.start_segment
    return None
####


def _search_trajectory(search: SearchBlock) -> int | None:
    if search.objective is None:
        return None
    return search.objective.left.trajectory or search.objective.left.trajectory_subscript
####


def _search_intervals_partially_overlap(first_start: int, first_end: int, second_start: int, second_end: int) -> bool:
    """Return true for overlap without containment."""

    return first_start < second_start < first_end < second_end or second_start < first_start < second_end < first_end
####


def _collect_known_output_variable_names(problem) -> set[str]:
    known = {
        _normalize_variable_name(block.variable)
        for block in problem.blocks
        if isinstance(block, DefineBlock) and block.variable is not None
    }
    for block in problem.blocks:
        if isinstance(block, (FileBlock, EgsBlock, PrintBlock)):
            known.update(_normalize_variable_name(variable) for variable in block.variables)
        ####
    for trajectory in problem.trajectories:
        for block in trajectory.blocks:
            if isinstance(block, DefineBlock) and block.variable is not None:
                known.add(_normalize_variable_name(block.variable))
            elif isinstance(block, (FileBlock, EgsBlock, PrintBlock)):
                known.update(_normalize_variable_name(variable) for variable in block.variables)
            ####
        for segment in trajectory.segments:
            for block in segment.blocks:
                if isinstance(block, DefineBlock) and block.variable is not None:
                    known.add(_normalize_variable_name(block.variable))
                elif isinstance(block, (FileBlock, EgsBlock, PrintBlock)):
                    known.update(_normalize_variable_name(variable) for variable in block.variables)
                ####
            ####
        ####
    ####
    return known
####
####


def validate_problem(
    document: ProblemDocument,
    available_tables: set[str] | Mapping[str, str] | None = None,
    available_table_variables: Mapping[str, Collection[str]] | None = None,
) -> list[Diagnostic]:
    diagnostics = list(document.diagnostics)
    available_table_types = (
        {name.casefold(): table_type.casefold() for name, table_type in available_tables.items()}
        if isinstance(available_tables, Mapping)
        else {}
    )
    available_table_names = (
        set(available_table_types)
        if isinstance(available_tables, Mapping)
        else {name.casefold() for name in (available_tables or set())}
    )
    normalized_table_variables = {
        name.casefold(): {variable.casefold() for variable in variables}
        for name, variables in (available_table_variables or {}).items()
    }
    for problem in document.problems:
        known_output_variables = _collect_known_output_variable_names(problem)
        for expression, location in _walk_model_expressions(problem, problem.location):
            table_name = None
            if isinstance(expression, TableReferenceExpression):
                table_name = expression.name
            elif (
                isinstance(expression, CallExpression)
                and expression.function.casefold() == "table"
                and expression.arguments
                and isinstance(expression.arguments[0], NameExpression)
            ):
                table_name = expression.arguments[0].name
            if table_name is not None and available_tables is not None and table_name.casefold() not in available_table_names:
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.WARNING,
                        code="external-table-reference",
                        message=f"Table {table_name!r} is not present in the supplied table set.",
                        location=location,
                    )
                )
            ####
        ####
        for block in problem.blocks:
            if not isinstance(block, UnitsFormatBlock):
                continue
            for setting in block.settings:
                variable = _normalize_variable_name(setting.variable)
                if variable_dimension(setting.variable) is None and variable not in known_output_variables:
                    diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="unknown-units-format-variable",
                            message=(
                                f"Units/format target {setting.variable!r} does not match a documented output variable "
                                "or user-defined variable in this problem."
                            ),
                            location=setting.location,
                        )
                    )
                ####
            ####
        ####
        survey_blocks = [block for block in problem.blocks if isinstance(block, SurveyBlock) and block.survey_id is not None]
        survey_counts = Counter(block.survey_id for block in survey_blocks)
        for block in survey_blocks:
            if block.survey_id is None:
                continue
            if survey_counts[block.survey_id] > 1:
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="duplicate-survey-id",
                        message=f"Survey number {block.survey_id} is declared more than once; its settings are ambiguous.",
                        location=block.location,
                    )
                )
        survey_ids = {survey_id for survey_id, count in survey_counts.items() if count == 1}
        for expression, location in _walk_model_expressions(problem, problem.location):
            if (
                isinstance(expression, ParameterExpression)
                and expression.family == "survey"
                and expression.index not in survey_ids
            ):
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="unknown-survey-reference",
                        message=f"Survey placeholder surv-{expression.index} has no matching '*survey {expression.index}' block.",
                        location=location,
                    )
                )
            ####
        ####
        search_blocks = [block for block in problem.blocks if isinstance(block, SearchBlock) and block.search_id is not None]
        search_counts = Counter(block.search_id for block in search_blocks)
        for block in search_blocks:
            if block.search_id is None:
                continue
            if search_counts[block.search_id] > 1:
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="duplicate-search-id",
                        message=f"Search number {block.search_id} is declared more than once; its objective and controls are ambiguous.",
                        location=block.location,
                    )
                )
        search_ids = {search_id for search_id, count in search_counts.items() if count == 1}
        for block in problem.blocks:
            if not isinstance(block, SearchBlock):
                continue
            control_names = {assignment.name.casefold() for assignment in block.controls}
            for required in ("xlo", "xhi", "xest", "dx"):
                if required not in control_names:
                    diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="missing-search-control",
                            message=f"A '*search' block requires the '{required}' control documented by the manual.",
                            location=block.location,
                        )
                    )
                ####
            ####
        ####
        if document.grammar_profile is GrammarProfile.TAOS96:
            _validate_search_topology(problem, search_blocks, diagnostics)
        trajectory_by_number = {trajectory.number: trajectory for trajectory in problem.trajectories}

        def validate_endpoint(endpoint, location: object, *, default_trajectory: int | None = None) -> None:
            """Validate an explicitly qualified search/optimization endpoint."""
            trajectory_number = endpoint.trajectory or endpoint.trajectory_subscript or default_trajectory
            if trajectory_number is None or endpoint.segment is None:
                return
            trajectory = trajectory_by_number.get(trajectory_number)
            if trajectory is None:
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="unknown-endpoint-trajectory",
                        message=f"Endpoint references undefined trajectory {trajectory_number}.",
                        location=location,
                    )
                )
                return
            if endpoint.segment not in {segment.number for segment in trajectory.segments}:
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="unknown-endpoint-segment",
                        message=(
                            f"Endpoint references undefined segment {endpoint.segment} "
                            f"in trajectory {trajectory_number}."
                        ),
                        location=location,
                    )
                )
            ####
        ####
        for block in problem.blocks:
            if isinstance(block, SearchBlock) and block.objective is not None:
                validate_endpoint(block.objective.left, block.location)
                if block.objective.right is not None:
                    validate_endpoint(block.objective.right, block.location)
            elif isinstance(block, OptimizeBlock):
                validate_endpoint(
                    OptimizeEndpoint(
                        text=block.objective_variable or "",
                        segment=block.segment,
                        trajectory=block.trajectory,
                    ),
                    block.location,
                )
                for constraint in block.constraints:
                    validate_endpoint(constraint.left, constraint.location, default_trajectory=block.trajectory)
                    validate_endpoint(constraint.right, constraint.location, default_trajectory=block.trajectory)
                ####
            ####
        ####
        optimize_loop_counts: dict[str, int] = {}
        optimize_blocks = [block for block in problem.blocks if isinstance(block, OptimizeBlock)]
        for block in optimize_blocks[5:]:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="too-many-optimize-loops",
                    message="A problem may contain at most five optimization loops, identified by letters a through e.",
                    location=block.location,
                )
            )
        ####
        for block in problem.blocks:
            if not isinstance(block, OptimizeBlock):
                continue
            if block.loop is not None:
                loop = block.loop.casefold()
                optimize_loop_counts[loop] = optimize_loop_counts.get(loop, 0) + 1
            ####
            parameter_names = {assignment.name.casefold() for assignment in block.controls}
            parameter_ids = {
                int(name.split("-", 1)[1])
                for name in parameter_names
                if name.startswith("par-") and name.split("-", 1)[1].isdigit()
            }
            if not parameter_ids:
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="missing-optimize-parameters",
                        message="An '*optimize' block requires at least one initial par-N value.",
                        location=block.location,
                    )
                )
            ####
            elif parameter_ids != set(range(1, max(parameter_ids) + 1)):
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="nonsequential-optimize-parameters",
                        message="Optimization par-N parameter numbers must be sequential starting at par-1.",
                        location=block.location,
                    )
                )
            ####
            for prefix in ("lo", "hi", "ref"):
                for name in parameter_names:
                    if not name.startswith(f"{prefix}-"):
                        continue
                    suffix = name.split("-", 1)[1]
                    if suffix.isdigit() and int(suffix) not in parameter_ids:
                        diagnostics.append(
                            Diagnostic(
                                severity=Severity.ERROR,
                                code="orphan-optimize-parameter-bound",
                                message=f"Optimization control {name!r} has no matching par-{suffix} initial value.",
                                location=block.location,
                            )
                        )
                    ####
                ####
            ####
        ####
        for loop, count in optimize_loop_counts.items():
            if count > 1:
                duplicate = next(
                    block
                    for block in problem.blocks
                    if isinstance(block, OptimizeBlock) and block.loop is not None and block.loop.casefold() == loop
                )
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="duplicate-optimize-loop",
                        message=f"Optimization loop {loop!r} is defined more than once; its parameters and objective are ambiguous.",
                        location=duplicate.location,
                    )
                )
            ####
        ####
        optimize_parameters = {
            block.loop: {
                int(assignment.name.casefold().split("-", 1)[1])
                for assignment in block.controls
                if assignment.name.casefold().startswith("par-")
                and assignment.name.casefold().split("-", 1)[1].isdigit()
            }
            for block in problem.blocks
            if isinstance(block, OptimizeBlock)
            and block.loop is not None
            and optimize_loop_counts.get(block.loop.casefold(), 0) == 1
        }
        for expression, location in _walk_model_expressions(problem, problem.location):
            if not isinstance(expression, ParameterExpression) or expression.family != "optimize":
                continue
            if expression.loop not in optimize_parameters:
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="unknown-optimize-loop",
                        message=f"Optimization placeholder opt{expression.loop}-{expression.index} has no matching '*optimize {expression.loop}' block.",
                        location=location,
                    )
                )
            elif expression.index not in optimize_parameters[expression.loop]:
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="unknown-optimize-parameter",
                        message=f"Optimization placeholder opt{expression.loop}-{expression.index} has no matching par-{expression.index} initial value.",
                        location=location,
                    )
                )
            ####
        ####
        for expression, location in _walk_model_expressions(problem, problem.location):
            if (
                isinstance(expression, ParameterExpression)
                and expression.family == "search"
                and expression.index not in search_ids
            ):
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="unknown-search-reference",
                        message=f"Search placeholder srch-{expression.index} has no matching '*search {expression.index}' block.",
                        location=location,
                    )
                )
            ####
        ####
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
                if block.keyword != "initial" or block.source_trajectory is None:
                    continue
                source = trajectory_by_number.get(block.source_trajectory)
                if source is None:
                    diagnostics.append(Diagnostic(severity=Severity.ERROR, code="unknown-initial-trajectory", message=f"Initial state references undefined trajectory {block.source_trajectory}.", location=block.location))
                elif block.source_segment is not None and block.source_segment not in {segment.number for segment in source.segments}:
                    diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="unknown-initial-segment",
                            message=(
                                f"Initial state references undefined segment {block.source_segment} "
                                f"in trajectory {block.source_trajectory}."
                            ),
                            location=block.location,
                        )
                    )
                ####
            ####
            for segment in trajectory.segments:
                for block in segment.blocks:
                    if isinstance(block, WhenBlock) and block.action == "goto" and block.target_segment not in segment_set:
                        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="unknown-goto-segment", message=f"Segment {segment.number} jumps to undefined segment {block.target_segment}.", location=block.location))
                    ####
                    referenced_table_names = {
                        node.name.casefold()
                        for assignment in block.assignments
                        for node in _walk_expression(assignment.value)
                        if isinstance(node, TableReferenceExpression)
                    }
                    if block.keyword in {"aero", "cg", "prop"} and normalized_table_variables and referenced_table_names:
                        table_variables = set().union(
                            *(normalized_table_variables.get(name, set()) for name in referenced_table_names)
                        )
                        fixed_names = {
                            "aero": {"ca", "cn", "cl", "cd", "cs", "cx", "cy", "cz", "sref"},
                            "cg": {"cg"},
                            "prop": {"thrust", "mdot", "ep1", "ep2", "thr_units", "mdt_units"},
                        }[block.keyword]
                        for assignment in block.assignments:
                            name = assignment.name.casefold()
                            if name not in fixed_names and name not in table_variables:
                                diagnostics.append(
                                    Diagnostic(
                                        severity=Severity.ERROR,
                                        code="unknown-table-variable-assignment",
                                        message=(
                                            f"User-defined variable {assignment.name!r} in '*{block.keyword}' "
                                            "does not occur in the referenced table data."
                                        ),
                                        location=assignment.location,
                                    )
                                )
                        ####
                    if block.keyword in _USER_VARIABLE_BLOCK_FIXED_NAMES:
                        fixed_names = _USER_VARIABLE_BLOCK_FIXED_NAMES[block.keyword]
                        for assignment in block.assignments:
                            name = assignment.name.casefold()
                            if name in DOCUMENTED_STATE_VARIABLES and name not in fixed_names:
                                diagnostics.append(
                                    Diagnostic(
                                        severity=Severity.ERROR,
                                        code="reserved-user-variable",
                                        message=(
                                            f"Assignment name {assignment.name!r} in '*{block.keyword}' "
                                            "is reserved for a documented TAOS state variable and cannot "
                                            "be used as a user-defined variable."
                                        ),
                                        location=assignment.location,
                                    )
                                )
                            ####
                        ####
                    ####
                    for assignment in block.assignments:
                        for node in _walk_expression(assignment.value):
                            if isinstance(node, TableReferenceExpression) and available_table_types and node.name.casefold() in available_table_types:
                                expected_type = assignment.name.casefold() if block.keyword in {"aero", "cg", "prop", "wind"} else None
                                actual_type = available_table_types[node.name.casefold()]
                                # ``output`` is the documented generic value
                                # table form used by synthetic/example decks;
                                # it may feed a typed aero/prop assignment,
                                # while a conflicting typed table remains an
                                # ingestion error.
                                if expected_type in TABLE_TYPE_REFERENCES and actual_type not in {expected_type, "output"}:
                                    diagnostics.append(
                                        Diagnostic(
                                            severity=Severity.ERROR,
                                            code="table-type-mismatch",
                                            message=(
                                                f"Table {node.name!r} has type {actual_type!r}, but '*{block.keyword}' "
                                                f"assignment {assignment.name!r} requires table type {expected_type!r}."
                                            ),
                                            location=assignment.location,
                                        )
                                    )
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
    return _source_order(diagnostics)
####


def validate_table_file(document: TableDocument) -> list[Diagnostic]:
    diagnostics = list(document.diagnostics)
    seen: dict[str, object] = {}
    for table in document.tables:
        key = table.name.casefold()
        if key in seen:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="duplicate-table",
                    message=f"Table {table.name!r} is declared more than once (table names are case-insensitive).",
                    location=table.location,
                )
            )
        ####
        else:
            seen[key] = table
        ####
    ####
    return _source_order(diagnostics)
####
