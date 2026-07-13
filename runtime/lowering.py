"""Lower parsed TAOS documents into executable runtime cases and outputs."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

from taoryx.language.expressions import ExpressionType, IndexedExpression, ParameterExpression, TableReferenceExpression
from taoryx.language.models import (
    EgsBlock,
    FileBlock,
    InitialBlock,
    IntegrationBlock,
    Problem,
    ProblemDocument,
    PrintBlock,
    TableDocument,
    WhenBlock,
)
from taoryx.tables import ExtrapolationMode, PreparedTable, prepare_table

from .common import EventCondition, RuntimeProblem, RuntimeState, RuntimeVehicle
from .engine import ExecutionResult, compute_trajectories
from .expressions import evaluate_expression
from .summaries import evaluate_summary
from .surveys import generate_survey_cases


@dataclass(frozen=True, slots=True)
class RuntimeTable:
    """A parsed simple table paired with its prepared interpolation object."""

    name: str
    table_type: str
    independent_variables: tuple[str, ...]
    output_variable: str
    prepared: PreparedTable

    def evaluate(self, values: Mapping[str, float]) -> float:
        from taoryx.tables import interpolate_nd

        return interpolate_nd(self.prepared, tuple(values[name] for name in self.independent_variables))
    ####
####


@dataclass(frozen=True, slots=True)
class RuntimeCase:
    """One deterministic survey expansion and its lowered problem."""

    index: int
    parameters: Mapping[str, float]
    problem: RuntimeProblem


@dataclass(frozen=True, slots=True)
class LoweredDocument:
    """Executable cases plus output and summary specifications."""

    cases: tuple[RuntimeCase, ...]
    tables: Mapping[str, RuntimeTable]
    print_variables: tuple[str, ...]
    output_files: tuple[tuple[str, tuple[str, ...]], ...]
    summaries: tuple[tuple[str, tuple[tuple[str, str | None], ...]], ...]


def lower_tables(document: TableDocument) -> dict[str, RuntimeTable]:
    """Convert parsed simple table definitions to prepared interpolation tables."""

    tables: dict[str, RuntimeTable] = {}
    for definition in document.tables:
        if definition.format != "simple":
            raise ValueError(f"table {definition.name!r} is not a simple table")
        independent = tuple(definition.independent_variables)
        assignments = {item.name.casefold(): tuple(item.values) for item in definition.assignments}
        missing = [name for name in independent if name.casefold() not in assignments]
        outputs = [item for item in definition.assignments if item.name.casefold() not in {name.casefold() for name in independent}]
        if missing or len(outputs) != 1:
            raise ValueError(f"table {definition.name!r} needs independent axes and one output assignment")
        mode = ExtrapolationMode.CLAMP if any(str(value).casefold() == "no-extrap" for value in definition.options.values()) else ExtrapolationMode.LINEAR
        prepared = prepare_table(tuple(assignments[name.casefold()] for name in independent), outputs[0].values, extrapolation=mode)
        tables[definition.name.casefold()] = RuntimeTable(definition.name, definition.table_type, independent, outputs[0].name, prepared)
    return tables
####


def lower_problem_document(document: ProblemDocument, tables: Mapping[str, RuntimeTable] = {}) -> LoweredDocument:
    """Lower the first parsed problem and all survey combinations."""

    if len(document.problems) != 1:
        raise ValueError("runtime execution requires exactly one problem per document")
    problem = document.problems[0]
    surveys = _survey_parameters(problem)
    cases = generate_survey_cases(surveys) if surveys else ({},)
    print_variables = _print_variables(problem)
    output_files = _output_files(problem)
    summaries = _summary_specs(problem)
    lowered = tuple(_lower_case(problem, parameters, index, tables) for index, parameters in enumerate(cases, start=1))
    return LoweredDocument(lowered, tables, print_variables, output_files, summaries)
####


def execute_lowered(document: LoweredDocument, *, output_dir: str | Path = ".", max_steps: int = 100000) -> tuple[ExecutionResult, ...]:
    """Execute every lowered case and write print/file/summary products."""

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    results: list[ExecutionResult] = []
    for case in document.cases:
        result = compute_trajectories(case.problem, max_steps=max_steps)
        results.append(result)
        _write_case_outputs(case.index, result, document, destination)
    return tuple(results)
####


def _lower_case(problem: Problem, parameters: Mapping[str, float], index: int, tables: Mapping[str, RuntimeTable]) -> RuntimeCase:
    vehicles: list[RuntimeVehicle] = []
    for trajectory in problem.trajectories:
        initial = next((block for block in trajectory.blocks if isinstance(block, InitialBlock)), None)
        if initial is None:
            raise ValueError(f"trajectory {trajectory.number} has no initial block")
        names, values, named, start_time = _initial_values(initial, parameters)
        integration = next((block for segment in trajectory.segments for block in segment.blocks if isinstance(block, IntegrationBlock)), None)
        step = _assignment_value(integration, "dt", parameters, default=1.0) if integration is not None else 1.0
        final_time = _assignment_value(integration, "tf", parameters, default=None) if integration is not None else None
        conditions = tuple(
            EventCondition(f"trajectory-{trajectory.number}-when-{segment.number}", lambda state, expression=block.condition: _condition_value(expression, state.named, parameters), block.action or "stop")
            for segment in trajectory.segments
            for block in segment.blocks
            if isinstance(block, WhenBlock) and block.condition is not None
        )

        def derivative(state: RuntimeState, *, state_names=names, state_tables=tables) -> tuple[float, ...]:
            rates = {name: 0.0 for name in state_names}
            velocity = state.named.get("vel", 0.0)
            gamma = math.radians(state.named.get("gama", 0.0))
            if "alt" in rates:
                rates["alt"] = velocity * math.sin(gamma) if "gama" in state.named else velocity
            if "range" in rates:
                rates["range"] = velocity * math.cos(gamma)
            for table in state_tables.values():
                if table.output_variable.casefold() in rates and table.independent_variables[0] in state.named:
                    rates[table.output_variable.casefold()] = table.evaluate(state.named)
            return tuple(rates[name] for name in state_names)
        ####

        def stop_when(state: RuntimeState, event_conditions=conditions) -> bool:
            return any(condition.function(state) >= 0.0 for condition in event_conditions)
        ####

        vehicle = RuntimeVehicle(str(trajectory.number), RuntimeState(start_time, values, named=named, value_names=names), derivative, step)
        vehicle.history[0] = vehicle.state
        if conditions:
            vehicle.active = True
        vehicle._taos_stop_when = stop_when  # type: ignore[attr-defined]
        vehicles.append(vehicle)
    runtime = RuntimeProblem({vehicle.name: vehicle for vehicle in vehicles}, final_time=None)
    runtime.metadata["parameters"] = dict(parameters)
    runtime.metadata["tables"] = tables
    runtime.metadata["stop_conditions"] = tuple(getattr(vehicle, "_taos_stop_when", lambda state: False) for vehicle in vehicles)
    return RuntimeCase(index, parameters, runtime)
####


def _initial_values(block: InitialBlock, parameters: Mapping[str, float]) -> tuple[tuple[str, ...], tuple[float, ...], dict[str, float], float]:
    named: dict[str, float] = {}
    for assignment in block.assignments:
        named[assignment.name.casefold()] = evaluate_expression(assignment.value, named, parameters)
    start_time = named.pop("time", 0.0)
    names = tuple(named)
    return names, tuple(named[name] for name in names), {**named, "time": start_time}, start_time
####


def _survey_parameters(problem: Problem) -> dict[str, Sequence[float] | tuple[float, float, float]]:
    result: dict[str, Sequence[float] | tuple[float, float, float]] = {}
    for block in problem.blocks:
        if not hasattr(block, "settings") or getattr(block, "survey_id", None) is None:
            continue
        settings = {setting.name: tuple(float(value) for value in setting.values) for setting in block.settings}
        if "vals" in settings:
            result[f"survey-{block.survey_id}"] = settings["vals"]
        elif {"lo", "hi", "inc"} <= settings.keys():
            result[f"survey-{block.survey_id}"] = (settings["lo"][0], settings["hi"][0], settings["inc"][0])
    return result
####


def _print_variables(problem: Problem) -> tuple[str, ...]:
    return tuple(variable for trajectory in problem.trajectories for block in trajectory.blocks if isinstance(block, PrintBlock) for variable in block.variables)
####


def _output_files(problem: Problem) -> tuple[tuple[str, tuple[str, ...]], ...]:
    return tuple((block.filename, tuple(block.variables)) for block in problem.blocks if isinstance(block, (FileBlock, EgsBlock)) and block.filename)
####


def _summary_specs(problem: Problem) -> tuple[tuple[str, tuple[tuple[str, str | None], ...]], ...]:
    return tuple((block.name or "summary", tuple((operation.operation, operation.operand.text if operation.operand else None) for operation in block.operations)) for block in problem.blocks if hasattr(block, "operations"))
####


def _assignment_value(block: IntegrationBlock | None, name: str, parameters: Mapping[str, float], *, default: float | None) -> float | None:
    if block is None:
        return default
    assignment = next((item for item in block.assignments if item.name.casefold() == name.casefold()), None)
    return default if assignment is None else evaluate_expression(assignment.value, {}, parameters)
####


def _condition_value(expression: ExpressionType, values: Mapping[str, float], parameters: Mapping[str, float]) -> float:
    return evaluate_expression(expression, values, parameters)
####


def _write_case_outputs(index: int, result: ExecutionResult, document: LoweredDocument, destination: Path) -> None:
    for name, history in result.states.items():
        if not history:
            continue
        variables = document.print_variables or tuple(history[-1].value_names)
        rows = [" ".join(("time", *variables))]
        rows.extend(" ".join(str(state.time if variable == "time" else state.named.get(variable.casefold(), float("nan"))) for variable in variables) for state in history)
        for filename, file_variables in document.output_files:
            chosen = file_variables or variables
            path = destination / (filename if index == 1 else f"{Path(filename).stem}-case-{index}{Path(filename).suffix}")
            path.write_text(" ".join(("time", *chosen)) + "\n" + "\n".join(" ".join(str(state.time if variable == "time" else state.named.get(variable.casefold(), float("nan"))) for variable in chosen) for state in history) + "\n", encoding="utf-8")
        if document.print_variables:
            (destination / (f"case-{index}-{name}.print" if len(result.states) > 1 or index > 1 else f"{name}.print")).write_text("\n".join(rows) + "\n", encoding="utf-8")
    summary_values = {name: [state.named.get(name.casefold(), float("nan")) for state in history] for name, history in result.states.items() for name in history[-1].named}
    if document.summaries:
        payload = {summary: _evaluate_summary_chain(operations, summary_values) for summary, operations in document.summaries}
        (destination / f"case-{index}-summaries.json").write_text(__import__("json").dumps(payload, indent=2) + "\n", encoding="utf-8")
####


def _evaluate_summary_chain(operations: Sequence[tuple[str, str | None]], values: Mapping[str, Sequence[float]]) -> float:
    accumulator = 0.0
    for operation, operand in operations:
        if operand is not None:
            name = operand.split("(")[-1].rstrip(")").split("[")[0]
            samples = values.get(name.casefold(), ())
            operand_value = evaluate_summary(samples, "max" if "max(" in operand else "min" if "min(" in operand else "last") if samples else 0.0
        else:
            operand_value = None
        if operation == "add":
            accumulator += operand_value or 0.0
        elif operation == "neg":
            accumulator = -accumulator
    return accumulator
####
