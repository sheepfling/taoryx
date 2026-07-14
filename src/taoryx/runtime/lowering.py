"""Lower parsed TAOS documents into executable runtime cases."""

from __future__ import annotations

import json
import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import cast

from taoryx.aerodynamics import maximum_lift_to_drag
from taoryx.attitude import EulerAngles, euler_angles_to_body_basis
from taoryx.contracts import Angle, Basis3, Frame, Latitude, Longitude, Vector3
from taoryx.coordinates import geocentric_unit_vectors, geodetic_unit_vectors
from taoryx.equations.geodesy import CartesianVector3
from taoryx.equations.gravity import gravity_full_geocentric_components
from taoryx.guidance import predictive_intercept, proportional_navigation, range_insensitive_axis, solve_guidance
from taoryx.language.expressions import (
    BinaryExpression,
    ExpressionType,
    IndexedExpression,
    NameExpression,
    NumberExpression,
    ParameterExpression,
    TableReferenceExpression,
    WildcardExpression,
)
from taoryx.language.models import (
    AeroBlock,
    Assignment,
    AtmosBlock,
    CgBlock,
    ConstantsBlock,
    DefineAssignmentStatement,
    DefineBlock,
    DefineControlStatement,
    DownrangeCrossrangeBlock,
    EarthBlock,
    EgsBlock,
    FileBlock,
    FlyBlock,
    IipBlock,
    IncrementBlock,
    InertialBlock,
    InitialBlock,
    IntegrationBlock,
    Limit,
    LimitsBlock,
    OptimizeBlock,
    OptimizeConstraint,
    OptimizeEndpoint,
    PrintBlock,
    Problem,
    ProblemDocument,
    PropulsionBlock,
    RadarBlock,
    RailBlock,
    ResetBlock,
    SearchBlock,
    Segment,
    SummarizeBlock,
    SummaryOperation,
    SurveyBlock,
    TableAssignment,
    TableCall,
    TableDefinition,
    TableDocument,
    TableOperation,
    TangentBlock,
    UnitsFormatBlock,
    WhenBlock,
    WindBlock,
)
from taoryx.numeric import DifferenceMode
from taoryx.optimization import build_optimization_problem, redistribute_control_history
from taoryx.searches import golden_section_minimize, parabolic_minimize, parabolic_root, secant_bracketed_root
from taoryx.tables import (
    ExtrapolationMode,
    PreparedSkewedTable,
    PreparedTable,
    SkewedTableSlice,
    TableEvaluationContext,
    evaluate_full_table,
    prepare_skewed_table,
    prepare_table,
)

from .common import EventCondition, RuntimeProblem, RuntimeState, RuntimeVehicle
from .engine import ExecutionResult, compute_trajectories
from .expressions import evaluate_definition_program, evaluate_expression
from .optimization_runtime import resolve_optimize_block
from .summaries import evaluate_summary
from .surveys import generate_survey_cases
from .units import format_number, from_internal, selected_setting, to_internal

_TABLE_EVALUATOR_CACHE: dict[int, tuple[Mapping[str, RuntimeTable], dict[str, Callable[[Mapping[str, float]], float]]]] = {}
PlatformBasis = tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]


@dataclass(frozen=True, slots=True)
class RuntimeTable:
    """A parsed simple table paired with its prepared interpolation object."""

    name: str
    table_type: str
    independent_variables: tuple[str, ...]
    output_variable: str
    prepared: PreparedTable | None
    operations: tuple[TableOperation, ...] = ()
    reference_area: float | None = None
    skewed: PreparedSkewedTable | None = None

    def evaluate(self, values: Mapping[str, float], tables: Mapping[str, RuntimeTable] | None = None) -> float:
        if self.skewed is not None:
            from taoryx.tables import interpolate_skewed

            return interpolate_skewed(self.skewed, tuple(values[name] for name in self.independent_variables))
        if self.prepared is None:
            prepared_tables = {name: table.prepared for name, table in (tables or {}).items() if table.prepared is not None}
            evaluators = {
                name: _RuntimeTableEvaluator(table, tables or {})
                for name, table in (tables or {}).items()
            }
            return evaluate_full_table(self.operations, TableEvaluationContext.from_values(values, prepared_tables, evaluators)).value
        from taoryx.tables import interpolate_nd

        return interpolate_nd(self.prepared, tuple(values[name] for name in self.independent_variables))
    ####
####


@dataclass(frozen=True, slots=True)
class _RuntimeTableEvaluator:
    """Callable table adapter with explicit argumented lookup support."""

    table: RuntimeTable
    tables: Mapping[str, RuntimeTable]

    def __call__(self, values: Mapping[str, float]) -> float:
        return self.table.evaluate(values, self.tables)
    ####

    def evaluate_call(self, values: Mapping[str, float], arguments: Sequence[float]) -> float:
        if not self.table.independent_variables:
            if arguments:
                raise ValueError(f"full table {self.table.name!r} does not accept lookup arguments")
            return self.table.evaluate(values, self.tables)
        if len(arguments) != len(self.table.independent_variables):
            raise ValueError(
                f"table {self.table.name!r} requires {len(self.table.independent_variables)} lookup arguments"
            )
        query_values = dict(values)
        query_values.update(zip(self.table.independent_variables, arguments, strict=True))
        return self.table.evaluate(query_values, self.tables)
    ####
####


@dataclass(frozen=True, slots=True)
class RuntimeCase:
    index: int
    parameters: Mapping[str, float]
    problem: RuntimeProblem


@dataclass(frozen=True, slots=True)
class LoweredDocument:
    cases: tuple[RuntimeCase, ...]
    tables: Mapping[str, RuntimeTable]
    print_variables: tuple[str, ...]
    output_files: tuple[tuple[str, tuple[str, ...], int | None, int], ...]
    summaries: tuple[tuple[str, tuple[SummaryOperation, ...]], ...]
    unsupported_features: tuple[str, ...] = ()
    searches: tuple[SearchBlock, ...] = ()
    optimizations: tuple[OptimizeBlock, ...] = ()
    source_problem: Problem | None = None
    print_scopes: tuple[tuple[tuple[str, ...], int | None], ...] = ()
    problem_case_counts: tuple[int, ...] = ()
    case_problem_indices: tuple[int, ...] = ()
    summary_egs_files: tuple[tuple[str, int, tuple[str, ...]], ...] = ()
    unit_settings: Mapping[str, str | None] = field(default_factory=dict)
    output_formats: Mapping[str, str | None] = field(default_factory=dict)


def lower_tables(document: TableDocument, unit_settings: Mapping[str, str | None] = {}) -> dict[str, RuntimeTable]:
    """Convert parsed simple tables to prepared interpolation tables."""

    tables: dict[str, RuntimeTable] = {}
    for definition in document.tables:
        if definition.format == "full":
            skewed = _prepare_skewed_runtime_table(definition, unit_settings)
            if skewed is not None:
                independent_variables, output_variable, prepared = skewed
                tables[definition.name.casefold()] = RuntimeTable(
                    definition.name,
                    definition.table_type,
                    independent_variables,
                    output_variable,
                    None,
                    (),
                    _table_reference_area(definition.options),
                    prepared,
                )
                continue
            tables[definition.name.casefold()] = RuntimeTable(
                definition.name,
                definition.table_type,
                (),
                definition.name,
                None,
                tuple(definition.operations),
                _table_reference_area(definition.options),
            )
            continue
        independent = tuple(name.casefold() for name in definition.independent_variables)
        assignments = {
            item.name.casefold(): tuple(to_internal(value, item.name, unit_settings) for value in item.values)
            for item in definition.assignments
        }
        output_names = {name.casefold() for name in independent}
        outputs = [item for item in definition.assignments if item.name.casefold() not in output_names]
        missing = [name for name in independent if name.casefold() not in assignments]
        if missing or len(outputs) != 1:
            raise ValueError(f"table {definition.name!r} needs axes and one output assignment")
        mode = ExtrapolationMode.CLAMP if any(str(value).casefold() == "no-extrap" for value in definition.options.values()) else ExtrapolationMode.LINEAR
        tables[definition.name.casefold()] = RuntimeTable(
            definition.name,
            definition.table_type,
            independent,
            outputs[0].name,
            prepare_table(
                tuple(assignments[name.casefold()] for name in independent),
                assignments[outputs[0].name.casefold()],
                extrapolation=mode,
            ),
            (),
            _table_reference_area(definition.options),
        )
    return tables


def _prepare_skewed_runtime_table(
    definition: TableDefinition,
    unit_settings: Mapping[str, str | None],
) -> tuple[tuple[str, ...], str, PreparedSkewedTable] | None:
    """Lower a full-table ``add table(...)`` body into skewed interpolation data."""

    operations = definition.operations
    operation = next(
        (
            item
            for item in operations
            if item.operator.casefold() == "add"
            and isinstance(item.operand, TableCall)
            and len(item.operand.arguments) >= 2
            and item.assignments
        ),
        None,
    )
    if operation is None:
        return None
    assert isinstance(operation.operand, TableCall)
    arguments = tuple(argument.casefold() for argument in operation.operand.arguments)
    output_name = operation.operand.name.casefold()
    groups: list[list[TableAssignment]] = []
    current: list[TableAssignment] = []
    for assignment in operation.assignments:
        current.append(assignment)
        if assignment.name.casefold() == output_name:
            groups.append(current)
            current = []
    if current or not groups:
        return None
    slices: list[SkewedTableSlice] = []
    for group in groups:
        by_name = {assignment.name.casefold(): assignment for assignment in group}
        if any(name not in by_name for name in arguments):
            return None
        if any(len(by_name[name].values) != 1 for name in arguments[:-1]):
            return None
        inner = by_name[arguments[-1]]
        slices.append(
            SkewedTableSlice(
                tuple(to_internal(by_name[name].values[0], name, unit_settings) for name in arguments[:-1]),
                tuple(to_internal(value, arguments[-1], unit_settings) for value in inner.values),
                tuple(to_internal(value, output_name, unit_settings) for value in by_name[output_name].values),
            )
        )
    mode = ExtrapolationMode.CLAMP if (
        operation.extrapolation == "no-extrap"
        or any(str(value).casefold() == "no-extrap" for value in definition.options.values())
    ) else ExtrapolationMode.LINEAR
    return arguments, output_name, prepare_skewed_table(slices, extrapolation=mode)
####


def problem_unit_settings(document: ProblemDocument) -> tuple[dict[str, str | None], dict[str, str | None]]:
    """Collect the latest units and output format setting for each variable."""

    units: dict[str, str | None] = {}
    formats: dict[str, str | None] = {}
    for problem in document.problems:
        for block in problem.blocks:
            if not isinstance(block, UnitsFormatBlock):
                continue
            for setting in block.settings:
                key = setting.variable.casefold()
                units[key] = setting.unit
                formats[key] = setting.format
    return units, formats
####


def lower_problem_document(document: ProblemDocument, tables: Mapping[str, RuntimeTable] | None = None) -> LoweredDocument:
    """Lower every parsed problem and its survey combinations in source order."""

    if not document.problems:
        raise ValueError("runtime requires at least one problem")
    if len(document.problems) > 1 and any(
        isinstance(block, (SearchBlock, OptimizeBlock))
        for problem in document.problems
        for block in problem.blocks
    ):
        raise ValueError("search and optimize blocks are not supported across multiple problems")
    cases: list[RuntimeCase] = []
    print_variables: list[str] = []
    output_files: list[tuple[str, tuple[str, ...], int | None, int]] = []
    summaries: list[tuple[str, tuple[SummaryOperation, ...]]] = []
    unsupported: list[str] = []
    problem_searches: list[SearchBlock] = []
    problem_optimizations: list[OptimizeBlock] = []
    print_scopes: list[tuple[tuple[str, ...], int | None]] = []
    problem_case_counts: list[int] = []
    case_problem_indices: list[int] = []
    summary_egs_files: list[tuple[str, int, tuple[str, ...]]] = []
    unit_settings: dict[str, str | None] = {}
    output_formats: dict[str, str | None] = {}
    case_index = 1
    for problem in document.problems:
        for block in problem.blocks:
            if isinstance(block, UnitsFormatBlock):
                for setting in block.settings:
                    key = setting.variable.casefold()
                    unit_settings[key] = setting.unit
                    output_formats[key] = setting.format
        surveys = _survey_parameters(problem)
        survey_cases = generate_survey_cases(surveys) if surveys else ({},)
        problem_case_counts.append(len(survey_cases))
        case_problem_indices.extend([len(problem_case_counts) - 1] * len(survey_cases))
        search_seed = _search_seed_parameters(problem)
        optimize_seed = _optimize_seed_parameters(problem)
        cases.extend(
            _lower_case(problem, {**search_seed, **optimize_seed, **parameters}, index, tables or {}, unit_settings)
            for index, parameters in enumerate(survey_cases, start=case_index)
        )
        case_index += len(survey_cases)
        print_variables.extend(_print_variables(problem))
        output_files.extend(_output_files(problem, len(problem_case_counts) - 1))
        summaries.extend(_summary_specs(problem))
        summary_egs_files.extend(_summary_egs_specs(problem, len(problem_case_counts) - 1))
        unsupported.extend(_unsupported_features(problem))
        problem_searches.extend(block for block in problem.blocks if isinstance(block, SearchBlock))
        problem_optimizations.extend(block for block in problem.blocks if isinstance(block, OptimizeBlock))
        print_scopes.extend(_print_scopes(problem))
    source_problem = document.problems[0] if len(document.problems) == 1 else None
    return LoweredDocument(
        tuple(cases),
        tables or {},
        tuple(print_variables),
        tuple(output_files),
        tuple(summaries),
        tuple(dict.fromkeys(unsupported)),
        tuple(problem_searches),
        tuple(problem_optimizations),
        source_problem,
        tuple(print_scopes),
        tuple(problem_case_counts),
        tuple(case_problem_indices),
        tuple(summary_egs_files),
        unit_settings,
        output_formats,
    )
####


def execute_lowered(document: LoweredDocument, *, output_dir: str = ".", max_steps: int = 100000) -> tuple[ExecutionResult, ...]:
    """Execute cases in order and emit declared output products."""

    from pathlib import Path

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    results: list[ExecutionResult] = []
    summary_rows: dict[int, list[tuple[Mapping[str, float], Mapping[str, float]]]] = {}
    survey_optima: dict[int, dict[str, float]] = {}
    for case_position, case in enumerate(document.cases):
        problem_index = document.case_problem_indices[case_position] if document.case_problem_indices else 0
        executable_case = case
        if any(
            _control_value(optimize.controls, "surveys", case.parameters, 0.0) != 0.0
            for optimize in document.optimizations
        ) and problem_index in survey_optima:
            carried = {**case.parameters, **survey_optima[problem_index]}
            if document.source_problem is None:
                raise ValueError("survey optimization requires one source problem")
            executable_case = _lower_case(
                document.source_problem,
                carried,
                case.index,
                document.tables,
                document.unit_settings,
            )
        for search in document.searches:
            search_trials: list[ExecutionResult] = []
            executable_case = _resolve_search_case(executable_case, document, search, max_steps=max_steps, trial_results=search_trials)
            if _control_value(search.controls, "print", executable_case.parameters, 0.0) != 0.0:
                _write_search_trials(executable_case.index, search.search_id or 0, search_trials, document, destination)
        for optimize in document.optimizations:
            executable_case = _resolve_optimize_case(executable_case, document, optimize, max_steps=max_steps)
            if _control_value(optimize.controls, "surveys", executable_case.parameters, 0.0) != 0.0:
                survey_optima[problem_index] = {
                    name: value
                    for name, value in executable_case.parameters.items()
                    if name.startswith("optimize-")
                }
        result = compute_trajectories(executable_case.problem, max_steps=max_steps)
        results.append(result)
        _write_outputs(executable_case.index, result, document, destination, problem_index)
        if document.summary_egs_files and document.summaries:
            payload = {name: _evaluate_summary_operations(operations, result) for name, operations in document.summaries}
            summary_rows.setdefault(problem_index, []).append((executable_case.parameters, payload))
    for filename, problem_index, survey_names in document.summary_egs_files:
        _write_summary_egs(filename, survey_names, document.summaries, summary_rows.get(problem_index, ()), destination)
    return tuple(results)
####


def _lower_case(
    problem: Problem,
    parameters: Mapping[str, float],
    index: int,
    tables: Mapping[str, RuntimeTable],
    unit_settings: Mapping[str, str | None] = {},
) -> RuntimeCase:
    earth_mu, earth_omega, earth_j2, earth_coefficients = _earth_parameters(problem, parameters)
    environment_evaluator = _atmosphere_evaluator(problem)
    trajectories = {trajectory.number: trajectory for trajectory in problem.trajectories}
    vehicles: list[RuntimeVehicle] = []
    for trajectory in problem.trajectories:
        initial = next((block for block in trajectory.blocks if isinstance(block, InitialBlock)), None)
        if initial is None:
            raise ValueError(f"trajectory {trajectory.number} has no initial block")
        source_trajectory = trajectories.get(initial.source_trajectory) if initial.source_trajectory is not None else None
        source_initial = next((block for block in source_trajectory.blocks if isinstance(block, InitialBlock)), None) if source_trajectory is not None else None
        names, values, named, start_time = _initial_values(source_initial or initial, parameters, tables, unit_settings)
        inherited = source_trajectory is not None and source_initial is not None
        integral_blocks = tuple(block for block in trajectory.blocks if isinstance(block, DefineBlock) and block.integral and block.variable)
        for integral_block in integral_blocks:
            assert integral_block.variable is not None
            if integral_block.variable.casefold() not in named:
                initial_value = 0.0 if integral_block.initial_value is None else evaluate_expression(
                    integral_block.initial_value,
                    named,
                    parameters,
                    tables=_table_evaluators(tables),
                )
                named[integral_block.variable.casefold()] = initial_value
        for rate_state in _rate_guidance_state_names(
            tuple(block for segment in trajectory.segments for block in segment.blocks)
        ):
            named.setdefault(rate_state, 0.0)
        names = tuple(named)
        values = tuple(named[name] for name in names)
        if environment_evaluator is not None:
            named.update(environment_evaluator(named))
        definition_blocks = tuple(
            block
            for block in problem.blocks
            if isinstance(block, DefineBlock) and not block.integral
        ) + tuple(
            block
            for block in trajectory.blocks
            if isinstance(block, DefineBlock) and not block.integral
        )
        definitions = {
            assignment.name.casefold(): assignment.value
            for block in definition_blocks
            if isinstance(block, DefineBlock) and not block.integral
            for assignment in block.assignments
        }
        definition_controls = tuple(
            statement
            for block in definition_blocks
            for statement in block.typed_statements
            if isinstance(statement, DefineControlStatement)
        )
        condition_list: list[EventCondition] = []
        segment_events: dict[int, tuple[EventCondition, ...]] = {}
        segment_by_number = {segment.number: segment for segment in trajectory.segments}
        active_segment = {"number": trajectory.start_segment}
        initial_segment = segment_by_number[trajectory.start_segment]
        step = _segment_step_size(initial_segment, parameters, 1.0)
        guidance_interval = _segment_guidance_interval(initial_segment, parameters, step)
        platform_state: dict[str, object] = {}
        _align_inertial_platform(platform_state, initial_segment, named, parameters, tables, earth_omega)
        event_targets: dict[str, int | None] = {}
        vehicle_environment = _vehicle_environment_evaluator(
            environment_evaluator,
            segment_by_number,
            active_segment,
            tables,
            parameters,
            definition_blocks,
            vehicles,
            str(trajectory.number),
            guidance_interval,
            earth_mu,
            trajectory.blocks,
            tuple(block for block in problem.blocks if isinstance(block, RadarBlock)),
            tuple(block for block in problem.blocks if isinstance(block, WindBlock)),
            platform_state,
            earth_omega,
        )
        for segment in trajectory.segments:
            segment_conditions: list[EventCondition] = []
            for segment_block in segment.blocks:
                if isinstance(segment_block, WhenBlock) and segment_block.condition is not None:
                    expression = segment_block.condition

                    def condition_function(state: RuntimeState, expression: ExpressionType = expression) -> float:
                        return _event_residual(expression, state.named, parameters, tables)
                    ####

                    def condition_predicate(state: RuntimeState, expression: ExpressionType = expression) -> bool:
                        return _event_satisfied(expression, state.named, parameters, tables)
                    ####

                    event = EventCondition(
                        f"trajectory-{trajectory.number}-when-{segment.number}-{len(segment_conditions) + 1}",
                        condition_function,
                        segment_block.action or "stop",
                        condition_predicate,
                    )
                    condition_list.append(event)
                    segment_conditions.append(event)
                    event_targets[event.name] = segment_block.target_segment if segment_block.action == "goto" else None
            ####
            segment_events[segment.number] = tuple(segment_conditions)
        ####
        conditions = segment_events.get(trajectory.start_segment, ())

        vehicle_ref: list[RuntimeVehicle] = []
        event_handlers: dict[str, Callable[[RuntimeState], RuntimeState]] = {}
        for segment in trajectory.segments:
            for event in segment_events[segment.number]:
                def handler(
                    state: RuntimeState,
                    segment: Segment = segment,
                    target: int | None = event_targets.get(event.name),
                    event_segments: Mapping[int, tuple[EventCondition, ...]] = segment_events,
                    segments: Mapping[int, Segment] = segment_by_number,
                    reference: list[RuntimeVehicle] = vehicle_ref,
                    active_segment_ref: dict[str, int] = active_segment,
                    platform: dict[str, object] = platform_state,
                ) -> RuntimeState:
                    if target is not None and reference:
                        source_endpoint = state
                        updated = _apply_segment_updates(
                            state,
                            segments[target],
                            parameters,
                            tables,
                        )
                        _align_inertial_platform(platform, segments[target], updated.named, parameters, tables, earth_omega)
                        reference[0].events = event_segments.get(target, ())
                        reference[0].segment_number = target
                        next_step = _segment_step_size(segments[target], parameters, reference[0].step_size)
                        reference[0].step_size = next_step
                        reference[0].max_step_size = next_step
                        active_segment_ref["number"] = target
                        named = dict(updated.named)
                        named["tseg"] = 0.0
                        values = list(updated.values)
                        if "tseg" in updated.value_names:
                            values[updated.value_names.index("tseg")] = 0.0
                        endpoints = dict(updated.segment_endpoints)
                        endpoints[segment.number] = source_endpoint
                        updated = RuntimeState(updated.time, tuple(values), updated.frame, named, updated.value_names, endpoints)
                        return updated
                    return state
                ####

                event_handlers[event.name] = handler
            ####
        ####

        def derivative(
            state: RuntimeState,
            *,
            state_names: tuple[str, ...] = names,
            state_tables: Mapping[str, RuntimeTable] = tables,
            segments: Mapping[int, Segment] = segment_by_number,
            active_segment_ref: dict[str, int] = active_segment,
            guidance_interval: float = guidance_interval,
        ) -> tuple[float, ...]:
            rates = {name: 0.0 for name in state_names}
            _assemble_ecfc_rates(rates, state.named, earth_mu, earth_omega, earth_j2, earth_coefficients)
            velocity = state.named.get("vel", 0.0)
            gamma = math.radians(state.named.get("gama", 0.0))
            if "alt" in rates:
                rates["alt"] = velocity * math.sin(gamma) if "gama" in state.named else velocity
            if "range" in rates:
                rates["range"] = velocity * math.cos(gamma)
            _assemble_geodetic_rates(rates, state.named, velocity, gamma)
            if "plength" in rates:
                rates["plength"] = abs(velocity)
            if "tseg" in rates:
                rates["tseg"] = 1.0
            if "tmark" in rates:
                rates["tmark"] = 1.0
            for block in integral_blocks:
                assert block.variable is not None
                if block.variable.casefold() in rates:
                    rates[block.variable.casefold()] = _evaluate_integral_rate(block, state.named, parameters, state_tables)
            segment = segments.get(active_segment_ref["number"])
            segment_step = _segment_step_size(segment, parameters, step) if segment is not None else step
            segment_guidance_interval = _segment_guidance_interval(segment, parameters, segment_step) if segment is not None else guidance_interval
            thrust = 0.0
            mass_rate = 0.0
            if segment is not None:
                for prop in (block for block in segment.blocks if isinstance(block, PropulsionBlock)):
                    thrust_assignment = next((item for item in prop.assignments if item.name.casefold() == "thrust"), None)
                    mdot_assignment = next((item for item in prop.assignments if item.name.casefold() == "mdot"), None)
                    if thrust_assignment is not None:
                        thrust += evaluate_expression(thrust_assignment.value, state.named, parameters, tables=_table_evaluators(state_tables))
                    if mdot_assignment is not None:
                        mass_rate += evaluate_expression(mdot_assignment.value, state.named, parameters, tables=_table_evaluators(state_tables))
            weight_state = "wt" in state.value_names and "mass" not in state.value_names and segment is not None and any(isinstance(block, RailBlock) for block in segment.blocks)
            raw_mass = abs(state.named.get("wt", state.named.get("mass", 1.0)))
            mass = max(raw_mass / 32.174 if weight_state else raw_mass, 1e-12)
            if segment is not None and all(name in state.named for name in ("xdt", "ydt", "zdt")):
                aero_acceleration = _ecfc_aerodynamic_acceleration(segment, state.named, state_tables, parameters, mass)
            else:
                aero_acceleration = _aerodynamic_acceleration(segment, state.named, state_tables, parameters, mass) if segment is not None else (0.0, 0.0, 0.0)
            ecfc_propulsive_acceleration = _ecfc_propulsive_acceleration(segment, state.named, state_tables, parameters, mass) if segment is not None else (0.0, 0.0, 0.0)
            geodetic_propulsive_acceleration = _geodetic_propulsive_acceleration(segment, state.named, state_tables, parameters, mass) if segment is not None else None
            geodetic_force_rates = _geodetic_force_rates(
                segment,
                state.named,
                state_tables,
                parameters,
                thrust,
                mass,
                earth_mu,
                earth_omega,
                earth_j2,
                earth_coefficients,
                geodetic_propulsive_acceleration,
                (
                    state.named.get("_guidance_ax", 0.0),
                    state.named.get("_guidance_ay", 0.0),
                    state.named.get("_guidance_az", 0.0),
                ),
            )
            ecfc_total_acceleration = tuple(left + right for left, right in zip(aero_acceleration, ecfc_propulsive_acceleration, strict=True))
            ecfc_total_acceleration = tuple(
                value + state.named.get(f"_guidance_a{axis}", 0.0)
                for value, axis in zip(ecfc_total_acceleration, ("x", "y", "z"), strict=True)
            )
            available_acceleration = geodetic_force_rates[0] if geodetic_force_rates is not None else ecfc_total_acceleration[0]
            rail_correction = _rail_acceleration_correction(segment, state.named, available_acceleration, parameters) if segment is not None else 0.0
            if geodetic_force_rates is not None:
                if "vel" in rates:
                    rates["vel"] = geodetic_force_rates[0] + rail_correction
                if "gama" in rates:
                    rates["gama"] = geodetic_force_rates[1]
                if "psi" in rates:
                    rates["psi"] = geodetic_force_rates[2]
            elif "vel" in rates:
                rates["vel"] = available_acceleration + rail_correction
            for name, value in zip(("xdt", "ydt", "zdt"), aero_acceleration, strict=True):
                if name in rates and geodetic_force_rates is None:
                    rates[name] += ecfc_total_acceleration[({"xdt": 0, "ydt": 1, "zdt": 2})[name]] + (rail_correction if name == "xdt" else 0.0)
            if "wt" in rates:
                rates["wt"] = -mass_rate
            if "mass" in rates:
                rates["mass"] = -mass_rate
            _apply_rate_guidance(rates, state.named)
            if "vel" in rates and "_command_mach" in state.named and "_guidance_solved" not in state.named and state.named.get("sndspd", 0.0) > 0.0:
                target_speed = state.named["_command_mach"] * state.named["sndspd"]
                rates["vel"] = (target_speed - velocity) / max(segment_guidance_interval, 1e-12)
            if "gama" in rates and "_command_gamgd" in state.named and "_guidance_solved" not in state.named:
                rates["gama"] = (state.named["_command_gamgd"] - state.named.get("gama", 0.0)) / max(segment_guidance_interval, 1e-12)
            if "psi" in rates and "_command_psi" in state.named and "_guidance_solved" not in state.named:
                heading_error = (state.named["_command_psi"] - state.named.get("psi", 0.0) + 180.0) % 360.0 - 180.0
                rates["psi"] = heading_error / max(segment_guidance_interval, 1e-12)
            for table in state_tables.values():
                if table.output_variable.casefold() in rates and table.independent_variables:
                    rates[table.output_variable.casefold()] = table.evaluate(state.named, state_tables)
            return tuple(rates[name] for name in state_names)
        ####

        def stop_when(state: RuntimeState, reference: list[RuntimeVehicle] = vehicle_ref) -> bool:
            # Segment transitions replace the vehicle event set; do not keep
            # evaluating conditions from the segment that was left.
            event_conditions = reference[0].events if reference else conditions
            return any(
                condition.predicate(state) if condition.predicate is not None else condition.function(state) >= 0.0
                for condition in event_conditions
            )
        ####

        def definition_evaluator(
            values: Mapping[str, float],
            controls: tuple[DefineControlStatement, ...] = definition_controls,
        ) -> Mapping[str, float]:
            working = dict(values)
            for control in controls:
                for name in _define_control_names(control):
                    working.setdefault(name, 0.0)
            _apply_define_control_sequence(controls, working, parameters, tables)
            return {name: working[name] for control in controls for name in _define_control_names(control) if name in working}
        ####

        initial_state = RuntimeState(start_time, values, named=named, value_names=names)
        initial_state = _apply_segment_updates(
            initial_state,
            segment_by_number[trajectory.start_segment],
            parameters,
            tables,
        )

        def activation_handler(
            state: RuntimeState,
            target: int = trajectory.start_segment,
            segments: Mapping[int, Segment] = segment_by_number,
        ) -> RuntimeState:
            _align_inertial_platform(platform_state, segments[target], state.named, parameters, tables, earth_omega)
            return _apply_segment_updates(state, segments[target], parameters, tables)
        ####

        vehicle = RuntimeVehicle(
            str(trajectory.number),
            initial_state,
            derivative,
            step,
            dependencies=(str(initial.source_trajectory),) if inherited and initial.source_trajectory is not None else (),
            dependency_segments={str(initial.source_trajectory): initial.source_segment or 1} if inherited and initial.source_trajectory is not None else {},
            active=not inherited,
            segment_number=trajectory.start_segment,
            stop_when=None,
            events=conditions,
            integrator="rkf45",
            max_step_size=step,
            derived_definitions=definitions,
            definition_evaluator=definition_evaluator if definition_controls else None,
            parameters=parameters,
            table_evaluators=_table_evaluators(tables),
            environment_evaluator=vehicle_environment,
            event_handlers=event_handlers,
            activation_handler=activation_handler,
        )
        vehicle_ref.append(vehicle)
        initial_named = dict(vehicle.state.named)
        try:
            initial_named.update(vehicle_environment(initial_named))
        except KeyError:
            # Relative guidance may target a vehicle declared later in the file;
            # the normal integration refresh runs after the complete graph exists.
            pass
        vehicle.state = RuntimeState(vehicle.state.time, vehicle.state.values, named=initial_named, value_names=names)
        vehicle.history[0] = vehicle.state
        if definitions:
            try:
                derived = evaluate_definition_program(definitions, initial_named, parameters=parameters, table_evaluators=_table_evaluators(tables))
            except KeyError:
                # Indexed problem-scope definitions may need a later trajectory.
                derived = initial_named
            vehicle.state = RuntimeState(vehicle.state.time, vehicle.state.values, named=derived, value_names=names)
            vehicle.history[0] = vehicle.state
        if definition_controls:
            initial_named = dict(vehicle.state.named)
            initial_named.update(definition_evaluator(initial_named))
            vehicle.state = RuntimeState(vehicle.state.time, vehicle.state.values, named=initial_named, value_names=names)
            vehicle.history[0] = vehicle.state
        vehicles.append(vehicle)
    # Indexed problem references cannot resolve until the complete graph exists.
    for vehicle in vehicles:
        named = dict(vehicle.state.named)
        if vehicle.environment_evaluator is not None:
            named.update(vehicle.environment_evaluator(named))
        if vehicle.derived_definitions:
            named = evaluate_definition_program(
                vehicle.derived_definitions,
                {key: value for key, value in named.items() if key not in vehicle.derived_definitions},
                parameters=vehicle.parameters,
                table_evaluators=vehicle.table_evaluators,
            )
        named["time"] = vehicle.state.time
        vehicle.state = RuntimeState(
            vehicle.state.time,
            vehicle.state.values,
            vehicle.state.frame,
            named,
            vehicle.state.value_names,
            vehicle.state.segment_endpoints,
        )
        vehicle.history[0] = vehicle.state
    runtime = RuntimeProblem({vehicle.name: vehicle for vehicle in vehicles})
    runtime.metadata["parameters"] = dict(parameters)
    runtime.metadata["tables"] = tables
    return RuntimeCase(index, parameters, runtime)
####


def _initial_values(
    block: InitialBlock,
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable] | None = None,
    unit_settings: Mapping[str, str | None] = {},
) -> tuple[tuple[str, ...], tuple[float, ...], dict[str, float], float]:
    # ``time`` is available as the zero-time default even when the source
    # assigns it later in the initial block.
    named: dict[str, float] = {"time": 0.0}
    for assignment in block.assignments:
        value = evaluate_expression(
            assignment.value,
            named,
            parameters,
            tables=_table_evaluators(tables or {}),
        )
        named[assignment.name.casefold()] = to_internal(value, assignment.name, unit_settings)
    start_time = named.pop("time", 0.0)
    _normalize_initial_coordinates(named, block.coordinate_system)
    named.setdefault("rho", 0.0)
    if "vel" not in named and any(name in named for name in ("xdt", "ydt", "zdt")):
        named["vel"] = math.sqrt(sum(named.get(name, 0.0) ** 2 for name in ("xdt", "ydt", "zdt")))
    named.setdefault("mach", 0.0)
    named.setdefault("dynprs", 0.0)
    named.setdefault("nx", 0.0)
    named.setdefault("ny", 0.0)
    named.setdefault("nz", 0.0)
    named.setdefault("ntotal", 0.0)
    named.setdefault("plength", 0.0)
    named.setdefault("tseg", 0.0)
    named.setdefault("tmark", 0.0)
    names = tuple(named)
    return names, tuple(named[name] for name in names), {**named, "time": start_time}, start_time
####


def _normalize_initial_coordinates(named: dict[str, float], coordinate_system: str | None) -> None:
    """Populate Cartesian and geodetic state aliases at initialization."""

    radius = 20925646.3255
    if coordinate_system == "geodetic" and {"long", "lat", "alt", "vel", "gama", "psi"} <= named.keys():
        named["_geodetic_state"] = 1.0
        longitude = math.radians(named["long"])
        latitude = math.radians(named["lat"])
        gamma = math.radians(named["gama"])
        heading = math.radians(named["psi"])
        radial = radius + named["alt"]
        named.update(
            {
                "x": radial * math.cos(latitude) * math.cos(longitude),
                "y": radial * math.cos(latitude) * math.sin(longitude),
                "z": radial * math.sin(latitude),
            }
        )
        east = named["vel"] * math.cos(gamma) * math.sin(heading)
        north = named["vel"] * math.cos(gamma) * math.cos(heading)
        up = named["vel"] * math.sin(gamma)
        named["xdt"] = up * math.cos(latitude) * math.cos(longitude) - north * math.sin(latitude) * math.cos(longitude) - east * math.sin(longitude)
        named["ydt"] = up * math.cos(latitude) * math.sin(longitude) - north * math.sin(latitude) * math.sin(longitude) + east * math.cos(longitude)
        named["zdt"] = up * math.sin(latitude) + north * math.cos(latitude)
    elif coordinate_system == "ecfc" and {"x", "y", "z"} <= named.keys():
        named["_geodetic_state"] = 0.0
        longitude = math.atan2(named["y"], named["x"])
        radial = math.sqrt(named["x"] ** 2 + named["y"] ** 2 + named["z"] ** 2)
        latitude = math.asin(named["z"] / radial) if radial > 0.0 else 0.0
        named["long"] = math.degrees(longitude)
        named["lat"] = math.degrees(latitude)
        named["latgd"] = named["lat"]
        named["alt"] = radial - radius
        if {"xdt", "ydt", "zdt"} <= named.keys():
            east = -math.sin(longitude) * named["xdt"] + math.cos(longitude) * named["ydt"]
            north = -math.sin(latitude) * math.cos(longitude) * named["xdt"] - math.sin(latitude) * math.sin(longitude) * named["ydt"] + math.cos(latitude) * named["zdt"]
            up = math.cos(latitude) * math.cos(longitude) * named["xdt"] + math.cos(latitude) * math.sin(longitude) * named["ydt"] + math.sin(latitude) * named["zdt"]
            horizontal = math.hypot(east, north)
            named["vel"] = math.sqrt(east * east + north * north + up * up)
            named["gama"] = math.degrees(math.atan2(up, horizontal))
            named["gamgd"] = named["gama"]
            named["psi"] = math.degrees(math.atan2(east, north))
            named["psigd"] = named["psi"]
    ####


def _evaluate_integral_rate(
    block: DefineBlock,
    values: Mapping[str, float],
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
) -> float:
    """Evaluate one integral variable's conditional assignment program."""

    if block.variable is None:
        return 0.0
    working = dict(values)
    statements = tuple(block.typed_statements)
    _apply_define_statements(statements, working, parameters, tables)
    return working.get(block.variable.casefold(), values.get(block.variable.casefold(), 0.0))
####


def _evaluate_definition_blocks(
    blocks: Sequence[DefineBlock],
    values: Mapping[str, float],
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
) -> dict[str, float]:
    """Evaluate ordinary ``*define`` assignments and control programs."""

    working = dict(values)
    targets: list[str] = []
    for block in blocks:
        if block.variable is not None:
            targets.append(block.variable.casefold())
        if block.typed_statements:
            for statement in block.typed_statements:
                targets.extend(_define_control_names(statement) if isinstance(statement, DefineControlStatement) else (statement.assignment.name.casefold(),))
            _apply_define_statements(tuple(block.typed_statements), working, parameters, tables)
        else:
            for assignment in block.assignments:
                _apply_define_assignment(assignment, working, parameters, tables)
                targets.append(assignment.name.casefold())
    return {name: working[name] for name in dict.fromkeys(targets) if name in working}
####


def _apply_define_statements(
    statements: Sequence[DefineAssignmentStatement | DefineControlStatement],
    values: dict[str, float],
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
) -> None:
    """Apply typed define statements in source order, pairing adjacent branches."""

    index = 0
    while index < len(statements):
        statement = statements[index]
        if isinstance(statement, DefineControlStatement):
            next_statement = statements[index + 1] if index + 1 < len(statements) else None
            if statement.kind == "if" and isinstance(next_statement, DefineControlStatement) and next_statement.kind == "else":
                _apply_define_control_sequence((statement, next_statement), values, parameters, tables)
                index += 2
                continue
            _apply_define_control(statement, values, parameters, tables)
        else:
            _apply_define_assignment(statement.assignment, values, parameters, tables)
        index += 1
    ####


def _apply_define_control(
    control: DefineControlStatement,
    values: dict[str, float],
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
) -> None:
    if control.kind == "else":
        if control.assignment is not None:
            _apply_define_assignment(control.assignment, values, parameters, tables)
        for assignment in control.body:
            _apply_define_assignment(assignment, values, parameters, tables)
        _apply_define_control_sequence(control.body_controls, values, parameters, tables)
        return
    condition = control.condition is not None and evaluate_expression(control.condition, values, parameters, tables=_table_evaluators(tables)) != 0.0
    assignments = control.body if condition else control.else_body
    controls = control.body_controls if condition else control.else_controls
    if condition and control.assignment is not None:
        _apply_define_assignment(control.assignment, values, parameters, tables)
    for assignment in assignments:
        _apply_define_assignment(assignment, values, parameters, tables)
    _apply_define_control_sequence(controls, values, parameters, tables)
####


def _apply_define_control_sequence(
    controls: Sequence[DefineControlStatement],
    values: dict[str, float],
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
) -> None:
    """Apply ordered if/else controls without executing an orphaned branch."""

    index = 0
    while index < len(controls):
        control = controls[index]
        if control.kind == "if" and index + 1 < len(controls) and controls[index + 1].kind == "else":
            condition = control.condition is not None and evaluate_expression(control.condition, values, parameters, tables=_table_evaluators(tables)) != 0.0
            _apply_define_control(control if condition else controls[index + 1], values, parameters, tables)
            index += 2
            continue
        _apply_define_control(control, values, parameters, tables)
        index += 1
    ####


def _define_control_names(control: DefineControlStatement) -> tuple[str, ...]:
    """Return variables assigned by both branches of a define control."""

    names = [assignment.name.casefold() for assignment in (*control.body, *control.else_body)]
    for nested in (*control.body_controls, *control.else_controls):
        names.extend(_define_control_names(nested))
    if control.assignment is not None:
        names.append(control.assignment.name.casefold())
    if control.nested is not None:
        names.extend(_define_control_names(control.nested))
    return tuple(dict.fromkeys(names))
####


def _apply_define_assignment(
    assignment: Assignment,
    values: dict[str, float],
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
) -> None:
    name = assignment.name.casefold()
    value = evaluate_expression(assignment.value, values, parameters, tables=_table_evaluators(tables))
    previous = values.get(name, 0.0)
    if assignment.operator == "=":
        values[name] = value
    elif assignment.operator == "+=":
        values[name] = previous + value
    elif assignment.operator == "-=":
        values[name] = previous - value
    elif assignment.operator == "*=":
        values[name] = previous * value
    elif assignment.operator == "/=":
        values[name] = previous / value if value != 0.0 else _raise_zero_division(name)
    else:
        values[name] = value
####


def _raise_zero_division(name: str) -> float:
    raise ZeroDivisionError(f"integral definition division by zero for {name}")
####


def _evaluate_guidance_table(
    block: FlyBlock,
    values: Mapping[str, float],
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
) -> float:
    """Evaluate a parsed ``*fly variable vrs reference`` point table."""

    if block.reference is None or not block.points:
        raise ValueError("guidance table has no reference or points")
    independent = values.get(block.reference.casefold())
    if independent is None:
        independent = evaluate_expression(NameExpression(name=block.reference.casefold()), values, parameters, tables=_table_evaluators(tables))
    points = sorted(
        (
            evaluate_expression(point.value, values, parameters, tables=_table_evaluators(tables)),
            evaluate_expression(point.independent, values, parameters, tables=_table_evaluators(tables)),
        )
        for point in block.points
    )
    if len(points) == 1 or independent <= points[0][0]:
        return points[0][1]
    if independent >= points[-1][0]:
        return points[-1][1]
    upper_index = next(index for index in range(1, len(points)) if independent <= points[index][0])
    lower_independent, lower_value = points[upper_index - 1]
    upper_independent, upper_value = points[upper_index]
    fraction = (independent - lower_independent) / (upper_independent - lower_independent)
    interpolation = (block.interpolation or "interp-1").casefold()
    if interpolation == "interp-2":
        delta = (upper_value - lower_value + 180.0) % 360.0 - 180.0
        return lower_value + fraction * delta
    if interpolation == "interp-3":
        return _guidance_polynomial_interpolation(points, independent)
    return lower_value + fraction * (upper_value - lower_value)
####


def _guidance_polynomial_interpolation(points: Sequence[tuple[float, float]], independent: float) -> float:
    """Evaluate the documented polynomial guidance interpolation through all points."""

    total = 0.0
    for index, (x_value, y_value) in enumerate(points):
        basis = 1.0
        for other_index, (other_x, _) in enumerate(points):
            if other_index != index:
                denominator = x_value - other_x
                if abs(denominator) <= 1e-12:
                    raise ValueError("polynomial guidance points require distinct independent values")
                basis *= (independent - other_x) / denominator
        total += y_value * basis
    return total
####


def _evaluate_ld_max(
    segment: Segment,
    values: Mapping[str, float],
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
) -> dict[str, float]:
    """Resolve ``*fly l/d-max`` from the active ``cl`` and ``cd`` rules."""

    lift_expression: ExpressionType | None = None
    drag_expression: ExpressionType | None = None
    bounds = (-20.0, 20.0)
    for block in segment.blocks:
        if not isinstance(block, AeroBlock):
            continue
        assignments = {assignment.name.casefold(): assignment.value for assignment in block.assignments}
        lift_expression = assignments.get("cl", lift_expression)
        drag_expression = assignments.get("cd", drag_expression)
        for expression in (lift_expression, drag_expression):
            if not isinstance(expression, TableReferenceExpression):
                continue
            table = tables.get(expression.name.casefold())
            if table is None or table.prepared is None or "alpha" not in table.independent_variables:
                continue
            axis = table.prepared.axes[table.independent_variables.index("alpha")]
            if len(axis) > 1:
                bounds = (axis[0], axis[-1])
    if lift_expression is None or drag_expression is None:
        return {}
    evaluators = _table_evaluators(tables)

    def coefficients(angle: float) -> tuple[float, float]:
        candidate = {**values, "alpha": angle}
        return (
            evaluate_expression(lift_expression, candidate, parameters, tables=evaluators),
            evaluate_expression(drag_expression, candidate, parameters, tables=evaluators),
        )
    ####

    result = maximum_lift_to_drag(coefficients, bounds, 1e-6)
    return {"alpha": result.angle, "alpha_l/d": result.angle, "l/d": result.lift_to_drag_ratio}
####


def _earth_parameters(problem: Problem, parameters: Mapping[str, float]) -> tuple[float, float, float, Mapping[tuple[int, int], tuple[float, float]]]:
    """Resolve ``gm``, ``omega``, and the selected zonal ``J2`` coefficient."""

    earth = next((block for block in problem.blocks if isinstance(block, EarthBlock)), None)
    if earth is None:
        return 0.0, 0.0, 0.0, {}
    values = {
        assignment.name.casefold(): evaluate_expression(assignment.value, {}, parameters)
        for assignment in earth.assignments
    }
    model = (earth.model or "").casefold()
    nominal_gm = 1.407646463e16
    nominal_omega = 7.2921151467e-5
    nominal_j2 = 1.08262668e-3
    coefficients: dict[tuple[int, int], tuple[float, float]] = {}
    for degree in range(2, 5):
        for order in range(degree + 1):
            cosine = values.get(f"c{degree}{order}")
            sine = values.get(f"s{degree}{order}", 0.0)
            if cosine is not None or sine != 0.0:
                coefficients[(degree, order)] = (cosine or 0.0, sine)
    if model in {"wgs-72", "wgs-84", "wgs-84-full", "gem-t1-full", "tsap-72", "tsap-84"}:
        return values.get("gm", nominal_gm), values.get("omega", nominal_omega), values.get("j2", nominal_j2), coefficients
    return values.get("gm", 0.0), values.get("omega", 0.0), values.get("j2", 0.0), coefficients
####


def _table_reference_area(options: Mapping[str, str | float]) -> float | None:
    value = options.get("sref")
    return None if value is None else float(value)
####


def _aero_reference_area(
    block: AeroBlock,
    coefficient: ExpressionType | None,
    named: Mapping[str, float],
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
) -> float:
    """Resolve direct ``*aero sref`` before a referenced table area."""

    assignments = {item.name.casefold(): item for item in block.assignments}
    area_assignment = assignments.get("sref")
    if area_assignment is not None:
        return evaluate_expression(area_assignment.value, named, parameters, tables=_table_evaluators(tables))
    if isinstance(coefficient, TableReferenceExpression):
        return tables.get(coefficient.name.casefold(), RuntimeTable("", "", (), "", None)).reference_area or 1.0
    return 1.0
####


def _aerodynamic_acceleration(
    segment: Segment,
    named: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
    parameters: Mapping[str, float],
    mass: float,
) -> tuple[float, float, float]:
    """Resolve axial drag from active ``ca`` tables in the current segment."""

    speed = math.sqrt(sum(named.get(name, 0.0) ** 2 for name in ("xdt", "ydt", "zdt")))
    scalar_speed = abs(named.get("vel", 0.0))
    speed = speed if speed > 0.0 else scalar_speed
    if speed <= 0.0:
        return 0.0, 0.0, 0.0
    density = max(named.get("rho", 0.0), 0.0)
    dynamic_pressure = named.get("dynprs", 0.5 * density * speed * speed)
    acceleration = 0.0
    for block in segment.blocks:
        if not isinstance(block, AeroBlock):
            continue
        assignment = next((item for item in block.assignments if item.name.casefold() == "ca"), None)
        if assignment is None:
            continue
        coefficient = evaluate_expression(assignment.value, named, parameters, tables=_table_evaluators(tables))
        reference_area = _aero_reference_area(block, assignment.value, named, parameters, tables)
        acceleration += dynamic_pressure * reference_area * coefficient / mass
    if "xdt" in named or "ydt" in named or "zdt" in named:
        components = tuple(named.get(name, 0.0) for name in ("xdt", "ydt", "zdt"))
        norm = math.sqrt(sum(value * value for value in components))
        if norm > 0.0:
            return (-acceleration * components[0] / norm, -acceleration * components[1] / norm, -acceleration * components[2] / norm)
    return (-acceleration if named.get("vel", 0.0) >= 0.0 else acceleration, 0.0, 0.0)
####


def _ecfc_propulsive_acceleration(
    segment: Segment | None,
    named: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
    parameters: Mapping[str, float],
    mass: float,
) -> tuple[float, float, float]:
    """Assemble thrust vectors in ECFC axes from active propulsion blocks."""

    if segment is None:
        return 0.0, 0.0, 0.0
    reference_basis = Basis3(
        Vector3(1.0, 0.0, 0.0),
        Vector3(0.0, 1.0, 0.0),
        Vector3(0.0, 0.0, 1.0),
        Frame.ECFC,
        Frame.INERTIAL_PLATFORM,
    )
    body_basis = euler_angles_to_body_basis(
        reference_basis,
        EulerAngles(
            Angle(math.radians(named.get("yawi", named.get("yawgc", 0.0)))),
            Angle(math.radians(named.get("pitchi", named.get("pitchgc", 0.0)))),
            Angle(math.radians(named.get("rolli", named.get("rollgc", 0.0)))),
        ),
    )
    total = Vector3(0.0, 0.0, 0.0)
    evaluators = _table_evaluators(tables)
    for block in segment.blocks:
        if not isinstance(block, PropulsionBlock):
            continue
        assignments = {item.name.casefold(): item for item in block.assignments}
        thrust_assignment = assignments.get("thrust")
        if thrust_assignment is None:
            continue
        thrust = evaluate_expression(thrust_assignment.value, named, parameters, tables=evaluators)
        ep1 = evaluate_expression(assignments["ep1"].value, named, parameters, tables=evaluators) if "ep1" in assignments else 0.0
        ep2 = evaluate_expression(assignments["ep2"].value, named, parameters, tables=evaluators) if "ep2" in assignments else 0.0
        first = math.radians(ep1)
        second = math.radians(ep2)
        body_vector = Vector3(
            thrust * math.cos(first),
            -thrust * math.sin(first) * math.cos(second),
            -thrust * math.sin(first) * math.sin(second),
        )
        total = total + body_basis.first.scaled(body_vector.x) + body_basis.second.scaled(body_vector.y) + body_basis.third.scaled(body_vector.z)
    return total.x / mass, total.y / mass, total.z / mass
####


def _ecfc_aerodynamic_acceleration(
    segment: Segment,
    named: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
    parameters: Mapping[str, float],
    mass: float,
) -> tuple[float, float, float]:
    """Accumulate aerodynamic coefficient families in ECFC components."""

    velocity = Vector3(named.get("xdt", 0.0), named.get("ydt", 0.0), named.get("zdt", 0.0))
    speed = velocity.norm()
    if speed <= 0.0:
        return 0.0, 0.0, 0.0
    reference_basis = Basis3(
        Vector3(1.0, 0.0, 0.0),
        Vector3(0.0, 1.0, 0.0),
        Vector3(0.0, 0.0, 1.0),
        Frame.ECFC,
        Frame.INERTIAL_PLATFORM,
    )
    body_basis = euler_angles_to_body_basis(
        reference_basis,
        EulerAngles(
            Angle(math.radians(named.get("yawi", named.get("yawgc", 0.0)))),
            Angle(math.radians(named.get("pitchi", named.get("pitchgc", 0.0)))),
            Angle(math.radians(named.get("rolli", named.get("rollgc", 0.0)))),
        ),
    )
    forward = velocity.scaled(1.0 / speed)
    side_candidate = body_basis.second - forward.scaled(forward.dot(body_basis.second))
    if side_candidate.norm() <= 1e-12:
        side_candidate = body_basis.third - forward.scaled(forward.dot(body_basis.third))
    if side_candidate.norm() <= 1e-12:
        return 0.0, 0.0, 0.0
    side = side_candidate.scaled(1.0 / side_candidate.norm())
    down = forward.cross(side)
    dynamic_pressure = named.get("dynprs", 0.5 * max(named.get("rho", 0.0), 0.0) * speed * speed)
    total = Vector3(0.0, 0.0, 0.0)
    evaluators = _table_evaluators(tables)
    for block in segment.blocks:
        if not isinstance(block, AeroBlock):
            continue
        assignments = {item.name.casefold(): item for item in block.assignments}
        present = set(assignments)
        family: tuple[str, ...]
        if present & {"ca", "cn"}:
            family = tuple(name for name in ("ca", "cn") if name in present)
        else:
            family = next(
                (tuple(name for name in candidate if name in present) for candidate in (("cl", "cd", "cs"), ("cx", "cy", "cz")) if present & set(candidate)),
                (),
            )
        coefficients: dict[str, float] = {}
        reference_area = _aero_reference_area(block, assignments[family[0]].value if family else None, named, parameters, tables)
        for name in family:
            assignment = assignments[name]
            coefficients[name] = evaluate_expression(assignment.value, named, parameters, tables=evaluators)
        scale = dynamic_pressure * reference_area
        force = Vector3(0.0, 0.0, 0.0)
        if "ca" in coefficients:
            force = force + body_basis.first.scaled(-coefficients["ca"])
        if "cn" in coefficients:
            projection_y = forward.dot(body_basis.second)
            projection_z = forward.dot(body_basis.third)
            meridian = body_basis.second.scaled(projection_y) + body_basis.third.scaled(projection_z)
            if meridian.norm() > 1e-12:
                force = force + meridian.scaled(-coefficients["cn"] / meridian.norm())
        if "cl" in coefficients:
            force = force + down.scaled(-coefficients["cl"])
        if "cd" in coefficients:
            force = force + forward.scaled(-coefficients["cd"])
        if "cs" in coefficients:
            force = force + side.scaled(coefficients["cs"])
        if "cx" in coefficients:
            force = force + body_basis.first.scaled(coefficients["cx"])
        if "cy" in coefficients:
            force = force + body_basis.second.scaled(coefficients["cy"])
        if "cz" in coefficients:
            force = force + body_basis.third.scaled(coefficients["cz"])
        total = total + force.scaled(scale / mass)
    return total.x, total.y, total.z
####


def _geodetic_force_rates(
    segment: Segment | None,
    named: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
    parameters: Mapping[str, float],
    thrust: float,
    mass: float,
    gravitational_parameter: float,
    rotation_rate: float = 0.0,
    j2_coefficient: float = 0.0,
    harmonic_coefficients: Mapping[tuple[int, int], tuple[float, float]] | None = None,
    propulsive_acceleration: tuple[float, float, float] | None = None,
    guidance_acceleration: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> tuple[float, float, float] | None:
    """Project geodetic force, gravity, and rotating-frame terms into rates."""

    longitude_name = "long" if "long" in named else "lon" if "lon" in named else None
    if segment is None or longitude_name is None or named.get("_geodetic_state", 0.0) < 0.5 or not {"lat", "vel", "gama", "psi"}.issubset(named):
        return None
    basis = geodetic_unit_vectors(Longitude(math.radians(named[longitude_name])), Latitude(math.radians(named["lat"])))
    north, east, up = basis.first, basis.second, basis.third.scaled(-1.0)
    gamma = math.radians(named["gama"])
    heading = math.radians(named["psi"])
    horizontal = north.scaled(math.cos(heading)) + east.scaled(math.sin(heading))
    forward = horizontal.scaled(math.cos(gamma)) + up.scaled(math.sin(gamma))
    normal = horizontal.scaled(-math.sin(gamma)) + up.scaled(math.cos(gamma))
    side = north.scaled(-math.sin(heading)) + east.scaled(math.cos(heading))

    pitch = math.radians(named.get("pitchi", named.get("pitchgd", named["gama"])))
    yaw = math.radians(named.get("yawi", named.get("yawgd", named["psi"])))
    if propulsive_acceleration is None:
        commanded_horizontal = north.scaled(math.cos(yaw)) + east.scaled(math.sin(yaw))
        commanded = commanded_horizontal.scaled(math.cos(pitch)) + up.scaled(math.sin(pitch))
        total = commanded.scaled(thrust / mass)
    else:
        total = Vector3(*propulsive_acceleration)
    total = total + Vector3(*guidance_acceleration)
    position = Vector3(
        named.get("x", 0.0),
        named.get("y", 0.0),
        named.get("z", 0.0),
    )
    velocity_vector = Vector3(
        named.get("xdt", 0.0),
        named.get("ydt", 0.0),
        named.get("zdt", 0.0),
    )
    radius_squared = position.dot(position)
    if gravitational_parameter > 0.0 and radius_squared > 0.0:
        radius = math.sqrt(radius_squared)
        if harmonic_coefficients:
            gravity_components = _full_gravity_ecfc(
                (position.x, position.y, position.z),
                harmonic_coefficients,
                gravitational_parameter,
            )
            gravity_vector = Vector3(*gravity_components)
        else:
            gravity_scale = -gravitational_parameter / (radius_squared * radius)
            radius_ratio_squared = (20_902_646.3255 / radius) ** 2
            z_ratio_squared = (position.z / radius) ** 2
            j2_scale = 1.5 * j2_coefficient * radius_ratio_squared
            gravity_vector = Vector3(
                gravity_scale * position.x * (1.0 - j2_scale * (5.0 * z_ratio_squared - 1.0)),
                gravity_scale * position.y * (1.0 - j2_scale * (5.0 * z_ratio_squared - 1.0)),
                gravity_scale * position.z * (1.0 - j2_scale * (5.0 * z_ratio_squared - 3.0)),
            )
        coriolis = Vector3(
            2.0 * rotation_rate * velocity_vector.y,
            -2.0 * rotation_rate * velocity_vector.x,
            0.0,
        )
        centrifugal = Vector3(
            rotation_rate * rotation_rate * position.x,
            rotation_rate * rotation_rate * position.y,
            0.0,
        )
        total = total + gravity_vector + coriolis + centrifugal
    speed = abs(named["vel"])
    dynamic_pressure = named.get("dynprs", 0.5 * max(named.get("rho", 0.0), 0.0) * speed * speed)
    for block in segment.blocks:
        if not isinstance(block, AeroBlock):
            continue
        assignments = {item.name.casefold(): item for item in block.assignments}
        present = set(assignments)
        family: tuple[str, ...]
        if present & {"ca", "cn"}:
            family = tuple(name for name in ("ca", "cn") if name in present)
        else:
            families = (("cl", "cd", "cs"), ("cx", "cy", "cz"))
            family = next((tuple(name for name in candidate if name in present) for candidate in families if present & set(candidate)), ())
        for name in family:
            assignment = assignments[name]
            coefficient = evaluate_expression(assignment.value, named, parameters, tables=_table_evaluators(tables))
            reference_area = _aero_reference_area(block, assignment.value, named, parameters, tables)
            force_acceleration = dynamic_pressure * reference_area * coefficient / mass
            total = total + {
                "ca": forward.scaled(-force_acceleration),
                "cn": normal.scaled(force_acceleration),
                "cd": forward.scaled(-force_acceleration),
                "cl": normal.scaled(force_acceleration),
                "cs": side.scaled(force_acceleration),
                "cx": forward.scaled(force_acceleration),
                "cy": side.scaled(force_acceleration),
                "cz": up.scaled(-force_acceleration),
            }[name]
        ####
    ####
    speed_rate = total.dot(forward)
    if speed <= 1e-8:
        # Flight-path and heading rates are undefined at rest; the rail
        # constraint supplies the launch acceleration until a direction exists.
        gamma_rate = 0.0
        heading_rate = 0.0
    else:
        gamma_rate = total.dot(normal) / speed * 180.0 / math.pi
        heading_rate = total.dot(side) / max(speed * abs(math.cos(gamma)), 1e-12) * 180.0 / math.pi
    return speed_rate, gamma_rate, heading_rate
####


def _geodetic_propulsive_acceleration(
    segment: Segment | None,
    named: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
    parameters: Mapping[str, float],
    mass: float,
) -> tuple[float, float, float] | None:
    """Resolve propulsion vectors in the active geodetic-horizon basis."""

    if segment is None:
        return None
    longitude_name = "long" if "long" in named else "lon" if "lon" in named else None
    if longitude_name is None or named.get("_geodetic_state", 0.0) < 0.5 or not {"lat", "gama", "psi"}.issubset(named):
        return None
    basis = geodetic_unit_vectors(Longitude(math.radians(named[longitude_name])), Latitude(math.radians(named["lat"])))
    north, east, up = basis.first, basis.second, basis.third.scaled(-1.0)
    yaw = math.radians(named.get("yawi", named.get("yawgd", named["psi"])))
    pitch = math.radians(named.get("pitchi", named.get("pitchgd", named["gama"])))
    roll = math.radians(named.get("rolli", named.get("rollgd", 0.0)))
    horizontal = north.scaled(math.cos(yaw)) + east.scaled(math.sin(yaw))
    forward = horizontal.scaled(math.cos(pitch)) + up.scaled(math.sin(pitch))
    side = north.scaled(-math.sin(yaw)) + east.scaled(math.cos(yaw))
    normal = horizontal.scaled(-math.sin(pitch)) + up.scaled(math.cos(pitch))
    body_y = side.scaled(math.cos(roll)) + normal.scaled(math.sin(roll))
    body_z = side.scaled(-math.sin(roll)) + normal.scaled(math.cos(roll))
    total = Vector3(0.0, 0.0, 0.0)
    evaluators = _table_evaluators(tables)
    for block in segment.blocks:
        if not isinstance(block, PropulsionBlock):
            continue
        assignments = {item.name.casefold(): item for item in block.assignments}
        thrust_assignment = assignments.get("thrust")
        if thrust_assignment is None:
            continue
        thrust = evaluate_expression(thrust_assignment.value, named, parameters, tables=evaluators)
        ep1 = math.radians(evaluate_expression(assignments["ep1"].value, named, parameters, tables=evaluators)) if "ep1" in assignments else 0.0
        ep2 = math.radians(evaluate_expression(assignments["ep2"].value, named, parameters, tables=evaluators)) if "ep2" in assignments else 0.0
        body_vector = (
            forward.scaled(thrust * math.cos(ep1))
            + body_y.scaled(-thrust * math.sin(ep1) * math.cos(ep2))
            + body_z.scaled(-thrust * math.sin(ep1) * math.sin(ep2))
        )
        total = total + body_vector
    return total.x / mass, total.y / mass, total.z / mass
####


def _specific_load_observables(
    segment: Segment,
    values: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
    parameters: Mapping[str, float],
) -> dict[str, float]:
    """Project non-gravitational acceleration into the current ECFC body basis."""

    if not {"x", "y", "z", "xdt", "ydt", "zdt"}.issubset(values):
        return {}
    raw_mass = abs(values.get("wt", values.get("mass", 1.0)))
    mass = max(raw_mass, 1e-12)
    aerodynamic = _ecfc_aerodynamic_acceleration(segment, values, tables, parameters, mass)
    propulsive = _ecfc_propulsive_acceleration(segment, values, tables, parameters, mass)
    specific = tuple(
        left + right + values.get(f"_guidance_a{axis}", 0.0)
        for left, right, axis in zip(aerodynamic, propulsive, ("x", "y", "z"), strict=True)
    )
    body_basis = euler_angles_to_body_basis(
        Basis3(
            Vector3(1.0, 0.0, 0.0),
            Vector3(0.0, 1.0, 0.0),
            Vector3(0.0, 0.0, 1.0),
            Frame.ECFC,
            Frame.INERTIAL_PLATFORM,
        ),
        EulerAngles(
            Angle(math.radians(values.get("yawi", values.get("yawgc", 0.0)))),
            Angle(math.radians(values.get("pitchi", values.get("pitchgc", 0.0)))),
            Angle(math.radians(values.get("rolli", values.get("rollgc", 0.0)))),
        ),
    )
    vector = Vector3(*specific)
    components = (vector.dot(body_basis.first), vector.dot(body_basis.second), vector.dot(body_basis.third))
    return {
        "nx": components[0],
        "ny": components[1],
        "nz": components[2],
        "ntotal": vector.norm(),
    }
####


def _align_inertial_platform(
    platform: dict[str, object],
    segment: Segment,
    values: Mapping[str, float],
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
    earth_rotation_rate: float,
) -> None:
    """Capture a segment-start platform origin and basis in ECIC coordinates."""

    block = next((item for item in segment.blocks if isinstance(item, InertialBlock)), None)
    alignment = block.alignment if block is not None and block.alignment is not None else "geodetic"
    assignments = {
        item.name.casefold(): evaluate_expression(item.value, values, parameters, tables=_table_evaluators(tables))
        for item in (block.assignments if block is not None else ())
    }
    longitude = assignments.get("long", values.get("long", values.get("lon", 0.0)))
    latitude = assignments.get("lat", values.get("lat", values.get("latgd", 0.0)))
    if alignment == "geocentric":
        basis: PlatformBasis = _basis_vectors(geocentric_unit_vectors(Longitude(math.radians(longitude)), Latitude(math.radians(latitude))))
    elif alignment == "geodetic":
        basis = _basis_vectors(geodetic_unit_vectors(Longitude(math.radians(longitude)), Latitude(math.radians(latitude))))
    elif alignment == "ecfc":
        east = _platform_assignment_vector(assignments, "east", (0.0, 1.0, 0.0))
        down = _platform_assignment_vector(assignments, "down", (0.0, 0.0, -1.0))
        east = _normalize_tuple(east)
        down = _normalize_tuple(down)
        north = _normalize_tuple(_cross_tuple(down, east))
        basis = (north, east, down)
    elif alignment == "body":
        basis = _body_platform_basis(values)
    else:
        basis = _velocity_platform_basis(values, alignment == "wind")
    basis_vectors: PlatformBasis = tuple(
        _rotate_ecfc_to_ecic(vector, earth_rotation_rate, assignments.get("time", values.get("time", 0.0)))
        for vector in _basis_vectors(basis)
    )  # type: ignore[assignment]
    position = (values.get("x", 0.0), values.get("y", 0.0), values.get("z", 0.0))
    if alignment in {"geocentric", "geodetic"} and {"long", "lat"}.issubset(assignments):
        radius = assignments.get(
            "rcm",
            20_925_646.3255 + assignments.get("alt", 0.0),
        )
        longitude_radians = math.radians(assignments["long"])
        latitude_radians = math.radians(assignments["lat"])
        position = (
            radius * math.cos(latitude_radians) * math.cos(longitude_radians),
            radius * math.cos(latitude_radians) * math.sin(longitude_radians),
            radius * math.sin(latitude_radians),
        )
    alignment_time = assignments.get("time", values.get("time", 0.0))
    platform["origin"] = _rotate_ecfc_to_ecic(position, earth_rotation_rate, alignment_time)
    platform["basis"] = basis_vectors
####


def _inertial_platform_observables(
    values: Mapping[str, float],
    platform: Mapping[str, object],
    earth_rotation_rate: float,
) -> dict[str, float]:
    """Project current ECFC position and velocity into the fixed platform."""

    origin_value = platform.get("origin")
    basis_value = platform.get("basis")
    if not isinstance(origin_value, tuple) or not isinstance(basis_value, tuple) or len(origin_value) != 3 or len(basis_value) != 3:
        return {}
    origin = cast(tuple[float, float, float], origin_value)
    basis = cast(PlatformBasis, basis_value)
    time = values.get("time", 0.0)
    position_ecic = _rotate_ecfc_to_ecic((values.get("x", 0.0), values.get("y", 0.0), values.get("z", 0.0)), earth_rotation_rate, time)
    velocity_ecic = _rotate_ecfc_velocity_to_ecic(
        (values.get("x", 0.0), values.get("y", 0.0), values.get("z", 0.0)),
        (values.get("xdt", 0.0), values.get("ydt", 0.0), values.get("zdt", 0.0)),
        earth_rotation_rate,
        time,
    )
    relative = _subtract_tuple(position_ecic, origin)
    result = {
        "xip": _dot_tuple(relative, basis[0]),
        "yip": _dot_tuple(relative, basis[1]),
        "zip": _dot_tuple(relative, basis[2]),
    }
    result.update(_platform_velocity_components(velocity_ecic, basis))
    return result
####


def _platform_velocity_components(vector: tuple[float, float, float], basis: PlatformBasis) -> dict[str, float]:
    return {name: _dot_tuple(vector, axis) for name, axis in zip(("xipdt", "yipdt", "zipdt"), basis, strict=True)}
####


def _basis_vectors(basis: Basis3 | PlatformBasis) -> PlatformBasis:
    if isinstance(basis, tuple):
        return basis
    return (
        (basis.first.x, basis.first.y, basis.first.z),
        (basis.second.x, basis.second.y, basis.second.z),
        (basis.third.x, basis.third.y, basis.third.z),
    )
####


def _body_platform_basis(values: Mapping[str, float]) -> PlatformBasis:
    reference = Basis3(
        Vector3(1.0, 0.0, 0.0),
        Vector3(0.0, 1.0, 0.0),
        Vector3(0.0, 0.0, 1.0),
        Frame.ECFC,
        Frame.INERTIAL_PLATFORM,
    )
    body = euler_angles_to_body_basis(
        reference,
        EulerAngles(
            Angle(math.radians(values.get("yawi", values.get("yawgc", 0.0)))),
            Angle(math.radians(values.get("pitchi", values.get("pitchgc", 0.0)))),
            Angle(math.radians(values.get("rolli", values.get("rollgc", 0.0)))),
        ),
    )
    return _basis_vectors(body)
####


def _velocity_platform_basis(values: Mapping[str, float], wind: bool) -> PlatformBasis:
    vector = (values.get("xdt", 0.0), values.get("ydt", 0.0), values.get("zdt", 0.0))
    if wind:
        vector = (
            vector[0] - values.get("windx", 0.0),
            vector[1] - values.get("windy", 0.0),
            vector[2] - values.get("windz", 0.0),
        )
    norm = math.sqrt(sum(component * component for component in vector))
    if norm <= 1e-12:
        return _basis_vectors(geodetic_unit_vectors(Longitude(0.0), Latitude(0.0)))
    x_axis = (vector[0] / norm, vector[1] / norm, vector[2] / norm)
    down = _basis_vectors(geodetic_unit_vectors(Longitude(math.radians(values.get("long", 0.0))), Latitude(math.radians(values.get("lat", 0.0)))))[2]
    z_candidate = _subtract_tuple(down, _scale_tuple(x_axis, _dot_tuple(down, x_axis)))
    if math.sqrt(sum(component * component for component in z_candidate)) <= 1e-12:
        return _basis_vectors(geodetic_unit_vectors(Longitude(0.0), Latitude(0.0)))
    z_axis = _normalize_tuple(z_candidate)
    y_axis = _normalize_tuple(_cross_tuple(z_axis, x_axis))
    return (x_axis, y_axis, z_axis)
####


def _platform_assignment_vector(assignments: Mapping[str, float], prefix: str, default: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        assignments.get(f"{prefix}x", default[0]),
        assignments.get(f"{prefix}y", default[1]),
        assignments.get(f"{prefix}z", default[2]),
    )
####


def _rotate_ecfc_to_ecic(vector: tuple[float, float, float], rotation_rate: float, time: float) -> tuple[float, float, float]:
    angle = rotation_rate * time
    cosine, sine = math.cos(angle), math.sin(angle)
    return (vector[0] * cosine - vector[1] * sine, vector[0] * sine + vector[1] * cosine, vector[2])
####


def _rotate_ecfc_velocity_to_ecic(
    position: tuple[float, float, float],
    velocity: tuple[float, float, float],
    rotation_rate: float,
    time: float,
) -> tuple[float, float, float]:
    angle = rotation_rate * time
    cosine, sine = math.cos(angle), math.sin(angle)
    return (
        velocity[0] * cosine - velocity[1] * sine - rotation_rate * (position[0] * sine + position[1] * cosine),
        velocity[0] * sine + velocity[1] * cosine + rotation_rate * (position[0] * cosine - position[1] * sine),
        velocity[2],
    )
####


def _dot_tuple(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
    return sum(a * b for a, b in zip(left, right, strict=True))
####


def _cross_tuple(left: tuple[float, float, float], right: tuple[float, float, float]) -> tuple[float, float, float]:
    return (left[1] * right[2] - left[2] * right[1], left[2] * right[0] - left[0] * right[2], left[0] * right[1] - left[1] * right[0])
####


def _subtract_tuple(left: tuple[float, float, float], right: tuple[float, float, float]) -> tuple[float, float, float]:
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])
####


def _scale_tuple(vector: tuple[float, float, float], factor: float) -> tuple[float, float, float]:
    return (vector[0] * factor, vector[1] * factor, vector[2] * factor)
####


def _normalize_tuple(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    norm = math.sqrt(sum(value * value for value in vector))
    if norm <= 1e-12:
        raise ValueError("inertial platform axis must be nonzero")
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm)
####


def _rail_acceleration_correction(segment: Segment, named: Mapping[str, float], available_acceleration: float, parameters: Mapping[str, float]) -> float:
    """Apply static/sliding rail resistance in the scalar flight direction."""

    rail = next((block for block in segment.blocks if isinstance(block, RailBlock)), None)
    if rail is None:
        return 0.0
    controls = {
        assignment.name.casefold(): evaluate_expression(assignment.value, named, parameters)
        for assignment in rail.assignments
    }
    coefficient = controls.get("cfstat", 0.0) if abs(named.get("vel", 0.0)) <= 1e-12 else controls.get("cfslid", controls.get("cfstat", 0.0))
    resistance = max(0.0, coefficient) * 32.174
    if abs(named.get("vel", 0.0)) <= 1e-12:
        return -available_acceleration if available_acceleration <= resistance else -resistance
    return -math.copysign(resistance, named.get("vel", 0.0))
####


def _vehicle_environment_evaluator(
    base_evaluator: Callable[[Mapping[str, float]], Mapping[str, float]] | None,
    segments: Mapping[int, Segment],
    active_segment: Mapping[str, int],
    tables: Mapping[str, RuntimeTable],
    parameters: Mapping[str, float],
    definition_blocks: Sequence[DefineBlock] = (),
    vehicles: Sequence[RuntimeVehicle] = (),
    vehicle_name: str = "",
    guidance_interval: float = 1.0,
    gravitational_parameter: float = 0.0,
    trajectory_blocks: Sequence[object] = (),
    radar_blocks: Sequence[RadarBlock] = (),
    wind_blocks: Sequence[WindBlock] = (),
    platform_state: dict[str, object] | None = None,
    earth_rotation_rate: float = 0.0,
) -> Callable[[Mapping[str, float]], Mapping[str, float]]:
    """Combine atmosphere refresh with active-segment force observables."""

    def evaluate(values: Mapping[str, float]) -> Mapping[str, float]:
        result = dict(base_evaluator(values) if base_evaluator is not None else {})
        if platform_state is not None:
            result.update(_inertial_platform_observables({**values, **result}, platform_state, earth_rotation_rate))
        result.update(_trajectory_reference_values(vehicles))
        result.update(_wind_observables(values, result, wind_blocks, parameters, tables))
        if "vair" in result and result.get("sndspd", 0.0) > 0.0:
            result["mach"] = result["vair"] / result["sndspd"]
            result["dynprs"] = 0.5 * result.get("rho", 0.0) * result["vair"] ** 2
        segment = segments.get(active_segment["number"])
        if segment is None:
            return result
        segment_step = _segment_step_size(segment, parameters, guidance_interval)
        segment_guidance_interval = _segment_guidance_interval(segment, parameters, segment_step)
        result["_segment"] = float(segment.number)
        coefficients: dict[str, list[float]] = {name: [] for name in ("ca", "cn", "cl", "cd", "cs", "cx", "cy", "cz")}
        thrust = 0.0
        mdot = 0.0
        for block in segment.blocks:
            if not isinstance(block, FlyBlock) or block.guidance_variable is None:
                continue
            if isinstance(block.value, WildcardExpression):
                result[block.guidance_variable.casefold()] = values.get(block.guidance_variable.casefold(), result.get(block.guidance_variable.casefold(), 0.0))
            elif block.value is not None:
                try:
                    guidance_value = evaluate_expression(
                        block.value,
                        {**values, **result},
                        parameters,
                        tables=_table_evaluators(tables),
                    )
                    variable = block.guidance_variable.casefold()
                    result[variable] = guidance_value
                    if variable in {"mach", "gamgd", "psigc", "psigd"}:
                        result[f"_command_{variable}"] = guidance_value
                    if variable in {"psigc", "psigd"}:
                        result["_command_psi"] = guidance_value
                except (KeyError, TypeError, ValueError):
                    # Unresolved guidance is rejected by the lowering feature audit.
                    pass
            elif block.reference is not None and block.points:
                try:
                    result[block.guidance_variable.casefold()] = _evaluate_guidance_table(block, {**values, **result}, parameters, tables)
                except (KeyError, TypeError, ValueError):
                    # A table may depend on a value that is not available yet.
                    pass
            elif block.guidance_variable.casefold() == "l/d-max":
                try:
                    result.update(_evaluate_ld_max(segment, {**values, **result}, parameters, tables))
                except (KeyError, TypeError, ValueError):
                    # A special rule without resolvable aerodynamic data is inert.
                    pass
        result.update(_evaluate_relative_guidance(result, vehicles, vehicle_name))
        result.update(_evaluate_range_insensitive_guidance(segment, {**values, **result}, parameters, gravitational_parameter))
        result.setdefault("alpha", values.get("alpha", 0.0))
        result.setdefault("power", values.get("power", 0.0))
        if "mass" in values or "mass" in result:
            if "wt" in values and "mass" in values and values["wt"] != values["mass"]:
                result["wt"] = values["wt"]
            else:
                result["wt"] = result.get("mass", values.get("mass", 0.0))
            result["fuel"] = result["wt"]
        elif "wt" in values or "wt" in result:
            result["mass"] = result.get("wt", values.get("wt", 0.0))
            result["fuel"] = result["mass"]
        if "alt" in values or "alt" in result:
            result["rcm"] = 20925646.3255 + result.get("alt", values.get("alt", 0.0))
        result.setdefault("alpha", values.get("alpha", 0.0))
        result.setdefault("alphat", abs(result["alpha"]))
        for alias, source in (("latgd", "lat"), ("gamgd", "gama"), ("psigd", "psi")):
            if alias not in result and (source in values or source in result):
                result[alias] = result.get(source, values.get(source, 0.0))
        for alias, source in (("pitchgd", "gama"), ("yawgd", "psi")):
            if alias not in result and (source in values or source in result):
                result[alias] = result.get(source, values.get(source, 0.0))
        result.update(_solve_indirect_guidance(segment, {**values, **result}, parameters, tables, segment_guidance_interval, gravitational_parameter))
        _apply_guidance_limits(segment, result, {**values, **result}, parameters, tables)
        for block in segment.blocks:
            if isinstance(block, (ConstantsBlock, CgBlock)):
                for assignment in block.assignments:
                    result[assignment.name.casefold()] = evaluate_expression(
                        assignment.value,
                        {**values, **result},
                        parameters,
                        tables=_table_evaluators(tables),
                    )
            elif isinstance(block, AeroBlock):
                for name in coefficients:
                    aero_assignment = next((item for item in block.assignments if item.name.casefold() == name), None)
                    if aero_assignment is not None:
                        coefficients[name].append(evaluate_expression(aero_assignment.value, {**values, **result}, parameters, tables=_table_evaluators(tables)))
            elif isinstance(block, PropulsionBlock):
                for assignment in block.assignments:
                    assignment_name = assignment.name.casefold()
                    if assignment_name == "thrust":
                        thrust += evaluate_expression(assignment.value, {**values, **result}, parameters, tables=_table_evaluators(tables))
                    elif assignment_name == "mdot":
                        mdot += evaluate_expression(assignment.value, {**values, **result}, parameters, tables=_table_evaluators(tables))
                    elif assignment_name in {"ep1", "ep2"}:
                        result[assignment_name] = evaluate_expression(assignment.value, {**values, **result}, parameters, tables=_table_evaluators(tables))
        for name, values_for_name in coefficients.items():
            if values_for_name:
                result[name] = sum(values_for_name)
        if thrust:
            result["thrust"] = thrust
        if mdot:
            result["mdot"] = mdot
        result.update(_specific_load_observables(segment, {**values, **result}, tables, parameters))
        result.update(_evaluate_definition_blocks(definition_blocks, {**values, **result}, parameters, tables))
        result.update(_trajectory_observables(trajectory_blocks, {**values, **result}, parameters, gravitational_parameter, tables))
        result.update(_relative_observables(values, vehicles, vehicle_name, radar_blocks, parameters))
        return result
    ####

    return evaluate
####


def _solve_indirect_guidance(
    segment: Segment,
    values: Mapping[str, float],
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
    guidance_interval: float,
    gravitational_parameter: float,
) -> dict[str, float]:
    """Solve documented indirect ``*fly`` rules against live force rates."""

    targets: dict[str, float] = {}
    direct_controls: set[str] = set()
    for block in segment.blocks:
        if not isinstance(block, FlyBlock) or block.guidance_variable is None:
            continue
        name = block.guidance_variable.casefold()
        if block.value is None:
            continue
        try:
            value = evaluate_expression(block.value, values, parameters, tables=_table_evaluators(tables))
        except (KeyError, TypeError, ValueError):
            continue
        if name in {"gamgd", "gamgc", "mach", "dynprs", "cl", "cd", "cs", "l/d", "thrust", "vel", "alt", "nx", "ny", "nz"}:
            targets[name] = value
        elif name in {"alpha", "alphat", "beta", "betae", "bankgc", "bankgd", "power"}:
            direct_controls.add(name)
    if not targets:
        return {}
    residual_names = [name for name in ("gamgd", "mach", "dynprs", "cl", "cd", "cs", "l/d", "thrust", "vel", "alt", "nx", "ny", "nz") if name in targets]
    load_targets = {name for name in ("nx", "ny", "nz") if name in targets}
    flight_condition_targets = {"gamgd", "gamgc", "mach", "dynprs", "vel", "alt"}
    requested_controls: tuple[str, ...]
    if load_targets:
        if len(residual_names) == 1:
            requested_controls = ("betae",) if "ny" in load_targets else ("alpha",)
        else:
            requested_controls = ("alpha", "power")
    elif {"gamgd", "mach"}.issubset(residual_names):
        requested_controls = ("alpha", "power")
    elif set(residual_names) & {"gamgd", "gamgc"}:
        requested_controls = ("alpha",)
    elif set(residual_names) & flight_condition_targets:
        requested_controls = ("power",)
    else:
        requested_controls = ("power",) if residual_names == ["thrust"] else ("alpha",)
    controls = [name for name in requested_controls if name not in direct_controls]
    if not controls or len(controls) != len(residual_names):
        return {}
    initial = tuple(values.get(name, 0.0) for name in controls)
    limit_bounds = _guidance_limit_bounds(segment, values, parameters, tables)
    bounds = tuple(_guidance_control_bounds(name, segment, tables, limit_bounds) for name in controls)
    increments = tuple(max((upper - lower) * 1e-4, 1e-6) for lower, upper in bounds)

    def residual(candidate: tuple[float, ...]) -> tuple[float, ...]:
        candidate_values = {**values, **dict(zip(controls, candidate, strict=True))}
        coefficients, thrust = _guidance_force_inputs(segment, candidate_values, parameters, tables)
        force_rates = _geodetic_force_rates(segment, candidate_values, tables, parameters, thrust, max(abs(candidate_values.get("wt", candidate_values.get("mass", 1.0))), 1e-12), gravitational_parameter)
        residuals: list[float] = []
        for target in residual_names:
            if target in {"gamgd", "mach", "dynprs", "vel", "alt"} and force_rates is not None:
                speed_rate, gamma_rate, _ = force_rates
                if target in {"gamgd", "gamgc"}:
                    residuals.append(gamma_rate - (targets[target] - candidate_values.get("gama", 0.0)) / max(guidance_interval, 1e-12))
                elif target == "mach":
                    sound_speed = max(candidate_values.get("sndspd", 0.0), 1e-12)
                    residuals.append(speed_rate - (targets[target] * sound_speed - candidate_values.get("vel", 0.0)) / max(guidance_interval, 1e-12))
                elif target == "vel":
                    residuals.append(speed_rate - (targets[target] - candidate_values.get("vel", 0.0)) / max(guidance_interval, 1e-12))
                elif target == "alt":
                    residuals.append(candidate_values.get("gama", 0.0) - 0.0)
                else:
                    dynamic_pressure_rate = candidate_values.get("rho", 0.0) * candidate_values.get("vel", 0.0) * speed_rate
                    residuals.append(dynamic_pressure_rate - (targets[target] - candidate_values.get("dynprs", 0.0)) / max(guidance_interval, 1e-12))
            elif target in {"cl", "cd", "cs"}:
                residuals.append(coefficients.get(target, 0.0) - targets[target])
            elif target == "l/d":
                residuals.append(coefficients.get("cl", 0.0) / max(abs(coefficients.get("cd", 0.0)), 1e-12) - targets[target])
            elif target == "thrust":
                residuals.append(thrust - targets[target])
            elif target in {"nx", "ny", "nz"}:
                loads = _specific_load_observables(segment, candidate_values, tables, parameters)
                if target not in loads:
                    return tuple(1e6 for _ in residual_names)
                residuals.append(loads[target] - targets[target])
            else:
                residuals.append(0.0)
        return tuple(residuals)
    ####

    try:
        solved = solve_guidance(residual, initial, bounds, increments, 1e-6, max_iterations=20)
    except (KeyError, RuntimeError, TypeError, ValueError, ZeroDivisionError):
        return {}
    if not solved.converged:
        return {}
    return {**dict(zip(controls, solved.controls, strict=True)), "_guidance_solved": 1.0}
####


def _trajectory_observables(
    blocks: Sequence[object],
    values: Mapping[str, float],
    parameters: Mapping[str, float],
    gravitational_parameter: float,
    tables: Mapping[str, RuntimeTable] = {},
) -> dict[str, float]:
    """Expose finite downrange, local velocity, and ballistic IIP observables."""

    assignments: dict[str, float] = {}
    for block in blocks:
        if not isinstance(block, (DownrangeCrossrangeBlock, TangentBlock, IipBlock)):
            continue
        for assignment in block.assignments:
            assignments[assignment.name.casefold()] = evaluate_expression(
                assignment.value,
                values,
                parameters,
                tables=_table_evaluators(tables),
            )
    result: dict[str, float] = {}
    radius = 20925646.3255
    latitude = values.get("lat", values.get("latgd", 0.0))
    longitude = values.get("long", values.get("lon", 0.0))
    speed = abs(values.get("vel", 0.0))
    gamma = math.radians(values.get("gama", values.get("gamgd", 0.0)))
    heading = math.radians(values.get("psi", values.get("azm", 0.0)))
    horizontal_speed = speed * math.cos(gamma)
    result["east"] = horizontal_speed * math.sin(heading)
    result["north"] = horizontal_speed * math.cos(heading)
    if "long" in values or "lat" in values or "latgd" in values:
        result["range"] = radius * math.sqrt(
            math.radians(longitude) ** 2 * math.cos(math.radians(latitude)) ** 2
            + math.radians(latitude) ** 2
        )

    if any(isinstance(block, DownrangeCrossrangeBlock) for block in blocks):
        reference_latitude = assignments.get("latgd", 0.0)
        reference_longitude = assignments.get("long", 0.0)
        reference_azimuth = math.radians(assignments.get("azm", 0.0))
        east = math.radians(longitude - reference_longitude) * radius * math.cos(math.radians(reference_latitude))
        north = math.radians(latitude - reference_latitude) * radius
        result["dwnrng"] = east * math.sin(reference_azimuth) + north * math.cos(reference_azimuth)
        result["crsrng"] = east * math.cos(reference_azimuth) - north * math.sin(reference_azimuth)

    if any(isinstance(block, IipBlock) for block in blocks):
        altitude = values.get("alt", 0.0)
        gravity = gravitational_parameter / max((radius + altitude) ** 2, 1.0) if gravitational_parameter else 0.0
        vertical_speed = speed * math.sin(gamma)
        discriminant = max(vertical_speed * vertical_speed + 2.0 * gravity * max(altitude, 0.0), 0.0)
        impact_time = (vertical_speed + math.sqrt(discriminant)) / gravity if gravity > 0.0 else 0.0
        impact_range = horizontal_speed * impact_time
        result.update(
            {
                "iip_rng": impact_range,
                "iip_azm": math.degrees(heading),
                "iip_time": impact_time,
                "iip_long": longitude + math.degrees(impact_range * math.sin(heading) / max(radius * math.cos(math.radians(latitude)), 1.0)),
                "iip_latgd": latitude + math.degrees(impact_range * math.cos(heading) / radius),
                "xtp": impact_range * math.sin(heading),
                "ytp": impact_range * math.cos(heading),
                "ztp": 0.0,
            }
        )
    return result
####


def _guidance_control_bounds(
    name: str,
    segment: Segment,
    tables: Mapping[str, RuntimeTable],
    limit_bounds: Mapping[str, tuple[float | None, float | None]] = {},
) -> tuple[float, float]:
    """Infer a bounded free-control interval from referenced tables."""

    default = (-20.0, 20.0) if name in {"alpha", "beta", "betae"} else (0.0, 1.0)
    lower, upper = default
    for block in segment.blocks:
        assignments = block.assignments if isinstance(block, (AeroBlock, PropulsionBlock)) else ()
        for assignment in assignments:
            if not isinstance(assignment.value, TableReferenceExpression):
                continue
            table = tables.get(assignment.value.name.casefold())
            if table is None or table.prepared is None or name not in table.independent_variables:
                continue
            axis = table.prepared.axes[table.independent_variables.index(name)]
            if len(axis) > 1:
                lower, upper = axis[0], axis[-1]
    limited_lower, limited_upper = limit_bounds.get(name.casefold(), (None, None))
    if limited_lower is not None:
        lower = max(lower, limited_lower)
    if limited_upper is not None:
        upper = min(upper, limited_upper)
    if lower > upper:
        raise ValueError(f"guidance limits leave no feasible interval for {name}")
    return lower, upper
####


def _guidance_limit_bounds(
    segment: Segment,
    values: Mapping[str, float],
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
) -> dict[str, tuple[float | None, float | None]]:
    """Resolve ``*limits`` into lower/upper bounds for guidance controls."""

    bounds: dict[str, tuple[float | None, float | None]] = {}
    for block in segment.blocks:
        if not isinstance(block, LimitsBlock):
            continue
        for limit in block.limits:
            variable = limit.variable.casefold()
            bound = evaluate_expression(limit.value, values, parameters, tables=_table_evaluators(tables))
            lower, upper = bounds.get(variable, (None, None))
            if limit.operator == ">":
                lower = bound if lower is None else max(lower, bound)
            else:
                upper = bound if upper is None else min(upper, bound)
            bounds[variable] = (lower, upper)
            if variable == "alphat" and limit.operator == "<":
                alpha_lower, alpha_upper = bounds.get("alpha", (None, None))
                alpha_lower = -bound if alpha_lower is None else max(alpha_lower, -bound)
                alpha_upper = bound if alpha_upper is None else min(alpha_upper, bound)
                bounds["alpha"] = (alpha_lower, alpha_upper)
    return bounds
####


def _rate_guidance_state_names(blocks: Sequence[object]) -> tuple[str, ...]:
    """Return state names required by documented direct rate guidance rules."""

    state_names = {
        "psigddt": "psi",
        "psigcdt": "psi",
        "gamgddt": "gama",
        "gamgcdt": "gama",
        "powerdt": "power",
    }
    return tuple(
        dict.fromkeys(
            state_names[block.guidance_variable.casefold()]
            for block in blocks
            if isinstance(block, FlyBlock)
            and block.guidance_variable is not None
            and block.guidance_variable.casefold() in state_names
        )
    )
####


def _apply_rate_guidance(rates: dict[str, float], values: Mapping[str, float]) -> None:
    """Apply direct ``*fly ...dt`` commands to matching integrated states."""

    rate_states = {
        "psigddt": "psi",
        "psigcdt": "psi",
        "gamgddt": "gama",
        "gamgcdt": "gama",
        "powerdt": "power",
    }
    for command, state_name in rate_states.items():
        if state_name in rates and command in values:
            rates[state_name] = float(values[command])
####


def _apply_guidance_limits(
    segment: Segment,
    result: dict[str, float],
    values: Mapping[str, float],
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
) -> None:
    """Clamp resolved direct and special guidance commands to active limits."""

    bounds = _guidance_limit_bounds(segment, values, parameters, tables)
    for variable, (lower, upper) in bounds.items():
        if variable == "alphat":
            continue
        if variable in result:
            if lower is not None:
                result[variable] = max(result[variable], lower)
            if upper is not None:
                result[variable] = min(result[variable], upper)
    if "alpha" in result:
        result["alphat"] = abs(result["alpha"])
####


def _guidance_force_inputs(
    segment: Segment,
    values: Mapping[str, float],
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
) -> tuple[dict[str, float], float]:
    """Evaluate aerodynamic coefficient and thrust inputs for one trial control."""

    coefficients: dict[str, float] = {}
    thrust = 0.0
    evaluators = _table_evaluators(tables)
    for block in segment.blocks:
        if isinstance(block, AeroBlock):
            for assignment in block.assignments:
                name = assignment.name.casefold()
                if name in {"cl", "cd", "cs"}:
                    coefficients[name] = evaluate_expression(assignment.value, values, parameters, tables=evaluators)
        elif isinstance(block, PropulsionBlock):
            thrust_assignment = next((item for item in block.assignments if item.name.casefold() == "thrust"), None)
            if thrust_assignment is not None:
                thrust += evaluate_expression(thrust_assignment.value, values, parameters, tables=evaluators)
    return coefficients, thrust
####


def _evaluate_relative_guidance(
    values: Mapping[str, float],
    vehicles: Sequence[RuntimeVehicle],
    vehicle_name: str,
) -> dict[str, float]:
    """Resolve predictive-intercept and proportional-navigation fly controls."""

    mode = "propnav" if "propnav" in values else "intercept" if "intercept" in values else None
    if mode is None:
        return {}
    target_number = int(values[mode])
    target = next((vehicle for vehicle in vehicles if vehicle.name == str(target_number)), None)
    if target is None or target.name == vehicle_name:
        raise KeyError(f"guidance target trajectory {target_number} is unavailable")
    source_position = CartesianVector3(*(target_component(values, name) for name in ("x", "y", "z")))
    source_velocity = CartesianVector3(*(target_component(values, name) for name in ("xdt", "ydt", "zdt")))
    target_position = CartesianVector3(*(target_component(target.state.named, name) for name in ("x", "y", "z")))
    target_velocity = CartesianVector3(*(target_component(target.state.named, name) for name in ("xdt", "ydt", "zdt")))
    if mode == "intercept":
        result = predictive_intercept(source_position, source_velocity, target_position, target_velocity)
        commanded_alpha = math.degrees(result.flight_path_angle_radians) - float(values.get("gama", 0.0))
        return {
            "yawi": math.degrees(result.heading_radians),
            "pitchi": math.degrees(result.flight_path_angle_radians),
            "alpha": commanded_alpha,
            "alphat": abs(commanded_alpha),
            "relrng[2]": _vector_distance(target_position, source_position),
            "relvel[2]": _closure_velocity(
                target_position,
                source_position,
                target_velocity,
                source_velocity,
            ),
        }
    navigation = proportional_navigation(source_position, source_velocity, target_position, target_velocity, 3.0)
    return {
        "yawi": math.degrees(navigation.yaw_radians),
        "pitchi": math.degrees(navigation.pitch_radians),
        "relrng[2]": _vector_distance(target_position, source_position),
        "relvel[2]": _closure_velocity(
            target_position,
            source_position,
            target_velocity,
            source_velocity,
        ),
        "_guidance_ax": navigation.ecfc_acceleration.x,
        "_guidance_ay": navigation.ecfc_acceleration.y,
        "_guidance_az": navigation.ecfc_acceleration.z,
    }
####


def _evaluate_range_insensitive_guidance(
    segment: Segment,
    values: Mapping[str, float],
    parameters: Mapping[str, float],
    gravitational_parameter: float,
) -> dict[str, float]:
    """Bind ``downria`` and ``upria`` to the local ballistic IIP sensitivities."""

    rule = next(
        (
            block
            for block in segment.blocks
            if isinstance(block, FlyBlock)
            and (block.guidance_variable or "").casefold() in {"downria", "upria"}
        ),
        None,
    )
    if rule is None or rule.value is None:
        return {}
    rule_name = (rule.guidance_variable or "").casefold()
    impact_altitude = evaluate_expression(rule.value, values, parameters)
    radius = 20925646.3255
    altitude = float(values.get("alt", 0.0))
    speed = abs(float(values.get("vel", 0.0)))
    gravity = gravitational_parameter / max((radius + altitude) ** 2, 1.0) if gravitational_parameter else 0.0
    if speed <= 1e-9 or gravity <= 1e-12 or altitude <= impact_altitude:
        return {}
    heading = math.radians(float(values.get("psi", values.get("azm", 0.0))))
    flight_path = math.radians(float(values.get("gama", values.get("gamgd", 0.0))))
    altitude_change = altitude - impact_altitude

    def impact_projection(yaw: float, pitch: float) -> tuple[float, float, float]:
        vertical_speed = speed * math.sin(pitch)
        discriminant = max(vertical_speed * vertical_speed + 2.0 * gravity * altitude_change, 0.0)
        impact_time = (vertical_speed + math.sqrt(discriminant)) / gravity
        impact_range = speed * math.cos(pitch) * impact_time
        return impact_range * math.sin(yaw), impact_range * math.cos(yaw), impact_time
    ####

    step = 1e-5
    base = impact_projection(heading, flight_path)
    yaw_plus = impact_projection(heading + step, flight_path)
    pitch_plus = impact_projection(heading, flight_path + step)
    position_sensitivity = (
        ((yaw_plus[0] - base[0]) / step, (pitch_plus[0] - base[0]) / step),
        ((yaw_plus[1] - base[1]) / step, (pitch_plus[1] - base[1]) / step),
    )
    time_sensitivity = ((yaw_plus[2] - base[2]) / step, (pitch_plus[2] - base[2]) / step)
    selected = range_insensitive_axis(position_sensitivity, time_sensitivity, delta_v=min(speed, 10.0))
    desired_time_sign = 1.0 if rule_name == "upria" else -1.0
    direction = -1.0 if selected.time_sensitivity * desired_time_sign < 0.0 else 1.0
    angular_scale = selected.delta_v / speed
    return {
        "yawi": math.degrees(heading + direction * angular_scale * math.cos(selected.yaw_radians)),
        "pitchi": math.degrees(flight_path + direction * angular_scale * math.sin(selected.yaw_radians)),
        "_ria_position_sensitivity": selected.position_sensitivity_norm,
        "_ria_time_sensitivity": direction * selected.time_sensitivity,
    }
####


def target_component(values: Mapping[str, float], name: str) -> float:
    """Read a Cartesian component while keeping relative guidance total."""

    return float(values.get(name, 0.0))
####


def _trajectory_reference_values(vehicles: Sequence[RuntimeVehicle]) -> dict[str, float]:
    """Expose indexed trajectory state aliases to definitions and outputs."""

    references: dict[str, float] = {}
    aliases = {
        "xecfc": "x",
        "yecfc": "y",
        "zecfc": "z",
        "xecfcdt": "xdt",
        "yecfcdt": "ydt",
        "zecfcdt": "zdt",
    }
    for vehicle in vehicles:
        for name, value in vehicle.state.named.items():
            if "[" in name:
                continue
            references[f"{name}[{vehicle.name}]"] = float(value)
        for name, value in vehicle.state.named.items():
            if name.count("[") == 1 and name.casefold().split("[", 1)[0] in {"relrng", "relvel", "radrng", "radrngdt"}:
                references[f"{name}[{vehicle.name}]"] = float(value)
        for alias, name in aliases.items():
            if name in vehicle.state.named:
                references[f"{alias}[{vehicle.name}]"] = float(vehicle.state.named[name])
        references[f"time[{vehicle.name}]"] = vehicle.state.time
    return references
####


def _relative_observables(
    values: Mapping[str, float],
    vehicles: Sequence[RuntimeVehicle],
    vehicle_name: str,
    radar_blocks: Sequence[RadarBlock],
    parameters: Mapping[str, float],
) -> dict[str, float]:
    """Resolve indexed relative-vehicle and radar observables."""

    current = next((vehicle for vehicle in vehicles if vehicle.name == vehicle_name), None)
    if current is None:
        return {}
    result: dict[str, float] = {}
    position = CartesianVector3(*(float(values.get(name, 0.0)) for name in ("x", "y", "z")))
    velocity = CartesianVector3(*(float(values.get(name, 0.0)) for name in ("xdt", "ydt", "zdt")))
    for target in vehicles:
        if target.name == vehicle_name:
            continue
        target_position = CartesianVector3(*(float(target.state.named.get(name, 0.0)) for name in ("x", "y", "z")))
        target_velocity = CartesianVector3(*(float(target.state.named.get(name, 0.0)) for name in ("xdt", "ydt", "zdt")))
        result[f"relrng[{target.name}]"] = _vector_distance(target_position, position)
        result[f"relvel[{target.name}]"] = _closure_velocity(
            target_position,
            position,
            target_velocity,
            velocity,
        )
    radius = 20925646.3255
    for radar in radar_blocks:
        assignments = {assignment.name.casefold(): assignment.value for assignment in radar.assignments}
        latitude = evaluate_expression(assignments["latgd"], values, parameters) if "latgd" in assignments else 0.0
        longitude = evaluate_expression(assignments["long"], values, parameters) if "long" in assignments else 0.0
        altitude = evaluate_expression(assignments["alt"], values, parameters) if "alt" in assignments else 0.0
        lat_radians = math.radians(latitude)
        lon_radians = math.radians(longitude)
        station = CartesianVector3(
            (radius + altitude) * math.cos(lat_radians) * math.cos(lon_radians),
            (radius + altitude) * math.cos(lat_radians) * math.sin(lon_radians),
            (radius + altitude) * math.sin(lat_radians),
        )
        relative = CartesianVector3(position.x - station.x, position.y - station.y, position.z - station.z)
        range_value = math.sqrt(relative.x * relative.x + relative.y * relative.y + relative.z * relative.z)
        range_rate = (relative.x * velocity.x + relative.y * velocity.y + relative.z * velocity.z) / range_value if range_value > 1e-12 else 0.0
        radar_id = radar.radar_id or 1
        result[f"radrng[{radar_id}]"] = range_value
        result[f"radrngdt[{radar_id}]"] = range_rate
    return result
####


def _wind_observables(
    values: Mapping[str, float],
    environment: Mapping[str, float],
    wind_blocks: Sequence[WindBlock],
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
) -> dict[str, float]:
    """Resolve scalar air-relative speed for documented wind forms."""

    speed = float(values.get("vel", 0.0))
    if not wind_blocks:
        return {"vair": abs(speed)}
    block = wind_blocks[-1]
    assignments = {assignment.name.casefold(): assignment.value for assignment in block.assignments}
    wind_speed = evaluate_expression(assignments["windv"], values, parameters, tables=_table_evaluators(tables)) if "windv" in assignments else 0.0
    wind_heading = math.radians(evaluate_expression(assignments["windh"], values, parameters, tables=_table_evaluators(tables))) if "windh" in assignments else 0.0
    wind_down = evaluate_expression(assignments["windd"], values, parameters, tables=_table_evaluators(tables)) if "windd" in assignments else 0.0
    heading = math.radians(values.get("psi", 0.0))
    gamma = math.radians(values.get("gama", 0.0))
    velocity = (
        speed * math.cos(gamma) * math.sin(heading),
        speed * math.cos(gamma) * math.cos(heading),
        speed * math.sin(gamma),
    )
    wind_vector = (
        wind_speed * math.cos(wind_down) * math.sin(wind_heading),
        wind_speed * math.cos(wind_down) * math.cos(wind_heading),
        wind_speed * math.sin(wind_down),
    )
    relative = tuple(left - right for left, right in zip(velocity, wind_vector, strict=True))
    return {"vair": math.sqrt(sum(component * component for component in relative))}
####


def _vector_distance(left: CartesianVector3, right: CartesianVector3) -> float:
    return math.sqrt((left.x - right.x) ** 2 + (left.y - right.y) ** 2 + (left.z - right.z) ** 2)
####


def _closure_velocity(
    target_position: CartesianVector3,
    source_position: CartesianVector3,
    target_velocity: CartesianVector3,
    source_velocity: CartesianVector3,
) -> float:
    """Return signed target/source closure velocity from the manual convention."""

    relative_position = CartesianVector3(
        target_position.x - source_position.x,
        target_position.y - source_position.y,
        target_position.z - source_position.z,
    )
    relative_velocity = CartesianVector3(
        target_velocity.x - source_velocity.x,
        target_velocity.y - source_velocity.y,
        target_velocity.z - source_velocity.z,
    )
    range_value = math.sqrt(
        relative_position.x * relative_position.x
        + relative_position.y * relative_position.y
        + relative_position.z * relative_position.z
    )
    if range_value <= 1e-12:
        return 0.0
    return -(
        relative_velocity.x * relative_position.x
        + relative_velocity.y * relative_position.y
        + relative_velocity.z * relative_position.z
    ) / range_value
####


def _atmosphere_evaluator(problem: Problem) -> Callable[[Mapping[str, float]], Mapping[str, float]] | None:
    """Build a tabulated user-atmosphere evaluator for runtime state refresh."""

    atmosphere = next((block for block in problem.blocks if isinstance(block, AtmosBlock)), None)
    if atmosphere is None:
        return None
    if atmosphere.model == "standard":
        def standard(values: Mapping[str, float]) -> Mapping[str, float]:
            altitude = max(0.0, float(values.get("alt", 0.0)))
            temperature, pressure, density, sound_speed = _standard_atmosphere_properties(altitude)
            speed = _environment_speed(values)
            return {
                "temp": temperature,
                "pres": pressure,
                "rho": density,
                "sndspd": sound_speed,
                "nu": 0.000157,
                "mach": speed / sound_speed,
                "dynprs": 0.5 * density * speed * speed,
            }
        ####

        return standard
    if atmosphere.model not in {"user", "site"} or not atmosphere.rows or not atmosphere.columns:
        return None
    columns = tuple(column.casefold() for column in atmosphere.columns)
    if "alt" not in columns:
        raise ValueError("user atmosphere requires an alt column")
    column_index = {name: index for index, name in enumerate(columns)}
    rows = tuple(tuple(float(value) for value in row) for row in atmosphere.rows)
    altitude_index = column_index["alt"]
    if any(len(row) != len(columns) for row in rows):
        raise ValueError("user atmosphere rows must match their column count")
    ordered = tuple(sorted(rows, key=lambda row: row[altitude_index]))
    if any(left[altitude_index] >= right[altitude_index] for left, right in zip(ordered, ordered[1:], strict=False)):
        raise ValueError("user atmosphere altitude rows must be strictly increasing")

    def evaluate(values: Mapping[str, float]) -> Mapping[str, float]:
        altitude = float(values.get("alt", ordered[0][altitude_index]))
        lower, upper = _bracket_atmosphere_rows(ordered, altitude, altitude_index)
        lower_altitude = lower[altitude_index]
        upper_altitude = upper[altitude_index]
        fraction = 0.0 if upper_altitude == lower_altitude else (altitude - lower_altitude) / (upper_altitude - lower_altitude)
        result = {
            name: lower[index] + fraction * (upper[index] - lower[index])
            for name, index in column_index.items()
            if name != "alt"
        }
        speed = _environment_speed(values)
        density = result.get("rho", values.get("rho", 0.0))
        sound_speed = result.get("sndspd", values.get("sndspd", 0.0))
        if "temp" not in result:
            result["temp"] = max(389.97, 518.67 - 0.00356616 * min(max(altitude, 0.0), 36089.0))
        if sound_speed <= 0.0:
            sound_speed = math.sqrt(1.4 * 1716.0 * result["temp"])
            result["sndspd"] = sound_speed
        result.setdefault("nu", 0.000157)
        if sound_speed > 0.0:
            result["mach"] = speed / sound_speed
        result["dynprs"] = 0.5 * density * speed * speed
        return result
    ####

    return evaluate
####


def _standard_atmosphere_properties(altitude_feet: float) -> tuple[float, float, float, float]:
    """Evaluate 1976 standard-atmosphere layers and return TAOS English units."""

    # Geopotential layer bases and lapse rates from the 1976 standard model.
    layers = (
        (0.0, 288.15, 101325.0, -0.0065),
        (11_000.0, 216.65, 22632.06, 0.0),
        (20_000.0, 216.65, 5474.889, 0.0010),
        (32_000.0, 228.65, 868.0187, 0.0028),
        (47_000.0, 270.65, 110.9063, 0.0),
        (51_000.0, 270.65, 66.93887, -0.0028),
        (71_000.0, 214.65, 3.956420, -0.0020),
        (84_852.0, 186.946, 0.373400, 0.0),
    )
    altitude_meters = altitude_feet * 0.3048
    layer_index = max(index for index, layer in enumerate(layers) if altitude_meters >= layer[0])
    base_altitude, base_temperature, base_pressure, lapse_rate = layers[layer_index]
    delta_altitude = altitude_meters - base_altitude
    gas_constant = 287.05287
    gravity = 9.80665
    if lapse_rate == 0.0:
        temperature_kelvin = base_temperature
        pressure_pascal = base_pressure * math.exp(-gravity * delta_altitude / (gas_constant * base_temperature))
    else:
        temperature_kelvin = base_temperature + lapse_rate * delta_altitude
        temperature_kelvin = max(temperature_kelvin, 1.0)
        pressure_pascal = base_pressure * (temperature_kelvin / base_temperature) ** (-gravity / (lapse_rate * gas_constant))
    density_si = pressure_pascal / (gas_constant * temperature_kelvin)
    sound_speed_si = math.sqrt(1.4 * gas_constant * temperature_kelvin)
    return (
        temperature_kelvin * 1.8,
        pressure_pascal / 47.88025898,
        density_si * 0.001940326502,
        sound_speed_si * 3.280839895,
    )
####


def _bracket_atmosphere_rows(rows: Sequence[tuple[float, ...]], altitude: float, altitude_index: int) -> tuple[tuple[float, ...], tuple[float, ...]]:
    if altitude <= rows[0][altitude_index]:
        return rows[0], rows[0]
    if altitude >= rows[-1][altitude_index]:
        return rows[-1], rows[-1]
    index = next(index for index in range(len(rows) - 1) if rows[index][altitude_index] <= altitude <= rows[index + 1][altitude_index])
    return rows[index], rows[index + 1]
####


def _environment_speed(values: Mapping[str, float]) -> float:
    """Resolve airspeed from scalar or ECFC Cartesian velocity state."""

    if "vair" in values:
        return abs(values["vair"])
    if "vel" in values:
        return abs(values["vel"])
    return math.sqrt(sum(values.get(name, 0.0) ** 2 for name in ("xdt", "ydt", "zdt")))
####


def _assemble_ecfc_rates(
    rates: dict[str, float],
    named: Mapping[str, float],
    gravitational_parameter: float,
    rotation_rate: float,
    j2_coefficient: float = 0.0,
    harmonic_coefficients: Mapping[tuple[int, int], tuple[float, float]] | None = None,
) -> None:
    """Add ECFC Cartesian kinematics and selected point-mass/J2 acceleration."""

    position_names = ("x", "y", "z")
    velocity_names = ("xdt", "ydt", "zdt")
    if not all(name in named for name in (*position_names, *velocity_names)):
        return
    for position_name, velocity_name in zip(position_names, velocity_names, strict=True):
        if position_name in rates:
            rates[position_name] = named[velocity_name]
    position = (named["x"], named["y"], named["z"])
    velocity = (named["xdt"], named["ydt"], named["zdt"])
    radius_squared = sum(component * component for component in position)
    gravity_scale = -gravitational_parameter / (radius_squared ** 1.5) if gravitational_parameter and radius_squared > 0.0 else 0.0
    radius = math.sqrt(radius_squared) if radius_squared > 0.0 else 0.0
    if harmonic_coefficients and radius > 0.0:
        gravity = _full_gravity_ecfc(position, harmonic_coefficients, gravitational_parameter)
    else:
        radius_ratio_squared = (20_902_646.3255 / radius) ** 2 if radius > 0.0 else 0.0
        z_ratio_squared = (position[2] / radius) ** 2 if radius > 0.0 else 0.0
        j2_scale = 1.5 * j2_coefficient * radius_ratio_squared
        gravity = (
            gravity_scale * position[0] * (1.0 - j2_scale * (5.0 * z_ratio_squared - 1.0)),
            gravity_scale * position[1] * (1.0 - j2_scale * (5.0 * z_ratio_squared - 1.0)),
            gravity_scale * position[2] * (1.0 - j2_scale * (5.0 * z_ratio_squared - 3.0)),
        )
    coriolis = (2.0 * rotation_rate * velocity[1], -2.0 * rotation_rate * velocity[0], 0.0)
    centrifugal = (rotation_rate * rotation_rate * position[0], rotation_rate * rotation_rate * position[1], 0.0)
    for velocity_name, value in zip(velocity_names, (gravity[0] + coriolis[0] + centrifugal[0], gravity[1] + coriolis[1] + centrifugal[1], gravity[2]), strict=True):
        if velocity_name in rates:
            rates[velocity_name] = value
####


def _assemble_geodetic_rates(rates: dict[str, float], named: Mapping[str, float], velocity: float, gamma: float) -> None:
    """Assemble spherical geodetic longitude/latitude rates in manual units."""

    if not {"alt", "gama", "psi"}.issubset(named):
        return
    radius = 20_902_646.3255
    altitude = named.get("alt", 0.0)
    heading = math.radians(named.get("psi", 0.0))
    latitude = math.radians(named.get("lat", named.get("latgd", 0.0)))
    horizontal = velocity * math.cos(gamma)
    denominator = max(radius + altitude, 1.0)
    if "alt" in rates:
        rates["alt"] = velocity * math.sin(gamma)
    if "range" in rates:
        rates["range"] = horizontal
    latitude_rate = horizontal * math.cos(heading) / denominator * 180.0 / math.pi
    if "lat" in rates:
        rates["lat"] = latitude_rate
    if "latgd" in rates:
        rates["latgd"] = latitude_rate
    longitude_name = "long" if "long" in rates else "lon" if "lon" in rates else None
    if longitude_name is not None:
        rates[longitude_name] = horizontal * math.sin(heading) / max(denominator * math.cos(latitude), 1.0) * 180.0 / math.pi
####


def _apply_segment_updates(
    state: RuntimeState,
    segment: Segment,
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
) -> RuntimeState:
    values = list(state.values)
    named = dict(state.named)
    named.pop("_segment_discontinuity", None)
    discontinuity = False
    # TAOS applies every reset before every increment, while preserving the
    # source order within each block family.
    for block_type in (ResetBlock, IncrementBlock):
        for block in segment.blocks:
            if not isinstance(block, block_type):
                continue
            for assignment in block.assignments:
                discontinuity = True
                name = assignment.name.casefold()
                value = evaluate_expression(assignment.value, named, parameters, tables=_table_evaluators(tables))
                if block_type is IncrementBlock:
                    value += named.get(name, 0.0)
                named[name] = value
                if name in state.value_names:
                    values[state.value_names.index(name)] = value
            ####
        ####
    integration = _segment_integration(segment)
    if integration is not None:
        default_interval = named.get("_dtprnt", _segment_step_size(segment, parameters, 1.0))
        named["_dtprnt"] = _assignment_value(integration, "dtprnt", parameters, default_interval)
    if discontinuity:
        named["_segment_discontinuity"] = 1.0
    ####
    return RuntimeState(state.time, tuple(values), state.frame, named, state.value_names, state.segment_endpoints)
####


def _full_gravity_ecfc(
    position: tuple[float, float, float],
    coefficients: Mapping[tuple[int, int], tuple[float, float]],
    gravitational_parameter: float,
    reference_radius: float = 20_902_646.3255,
) -> tuple[float, float, float]:
    """Evaluate configured degree-four harmonics and convert back to English units."""

    radius = math.sqrt(sum(component * component for component in position))
    longitude = math.atan2(position[1], position[0])
    latitude = math.asin(position[2] / radius)
    scale = 0.3048
    local = gravity_full_geocentric_components(
        gravitational_parameter * scale**3,
        reference_radius * scale,
        radius * scale,
        latitude,
        longitude,
        coefficients,
    )
    basis = geocentric_unit_vectors(Longitude(longitude), Latitude(latitude))
    vector = basis.first.scaled(local.x) + basis.second.scaled(local.y) + basis.third.scaled(local.z)
    return vector.x / scale, vector.y / scale, vector.z / scale
####


def _event_residual(
    expression: ExpressionType,
    values: Mapping[str, float],
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable] = {},
) -> float:
    if isinstance(expression, BinaryExpression) and expression.operator in {"<", ">", "<=", ">=", "=", "==", "!="}:
        left = evaluate_expression(expression.left, values, parameters, tables=_table_evaluators(tables))
        right = evaluate_expression(expression.right, values, parameters, tables=_table_evaluators(tables))
        if expression.operator == "<":
            return right - left
        return left - right
    return evaluate_expression(expression, values, parameters, tables=_table_evaluators(tables))
####


def _event_satisfied(
    expression: ExpressionType,
    values: Mapping[str, float],
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable] = {},
) -> bool:
    """Evaluate the documented ``*when`` relationship at one state."""

    if not isinstance(expression, BinaryExpression):
        return evaluate_expression(expression, values, parameters, tables=_table_evaluators(tables)) >= 0.0
    left = evaluate_expression(expression.left, values, parameters, tables=_table_evaluators(tables))
    right = evaluate_expression(expression.right, values, parameters, tables=_table_evaluators(tables))
    if expression.operator == "<":
        return left < right
    if expression.operator == "<=":
        return left <= right or math.isclose(left, right, rel_tol=0.0, abs_tol=1e-9)
    if expression.operator == ">":
        return left > right
    if expression.operator == ">=":
        return left >= right or math.isclose(left, right, rel_tol=0.0, abs_tol=1e-9)
    if expression.operator in {"=", "=="}:
        return math.isclose(left, right, rel_tol=0.0, abs_tol=1e-9)
    if expression.operator == "!=":
        return not math.isclose(left, right, rel_tol=0.0, abs_tol=1e-9)
    return False
####


def _limit_residual(limit: Limit, values: Mapping[str, float], parameters: Mapping[str, float], tables: Mapping[str, RuntimeTable]) -> float:
    """Return a non-negative residual when a ``*limits`` bound is violated."""

    current = values.get(limit.variable.casefold())
    if current is None:
        raise KeyError(f"undefined limited variable: {limit.variable}")
    bound = evaluate_expression(limit.value, values, parameters, tables=_table_evaluators(tables))
    return current - bound if limit.operator == "<" else bound - current
####


def _table_evaluators(tables: Mapping[str, RuntimeTable]) -> dict[str, Callable[[Mapping[str, float]], float]]:
    evaluators: dict[str, Callable[[Mapping[str, float]], float]] = {}
    cached = _TABLE_EVALUATOR_CACHE.get(id(tables))
    if cached is not None and cached[0] is tables:
        return cached[1]
    for name, table in tables.items():
        evaluators[name] = _RuntimeTableEvaluator(table, tables)
    _TABLE_EVALUATOR_CACHE[id(tables)] = (tables, evaluators)
    return evaluators
####


def _survey_parameters(problem: Problem) -> dict[str, Sequence[float] | tuple[float, float, float]]:
    result: dict[str, Sequence[float] | tuple[float, float, float]] = {}
    for block in problem.blocks:
        if not isinstance(block, SurveyBlock):
            continue
        settings = {setting.name: tuple(float(value) for value in setting.values) for setting in block.settings}
        if "vals" in settings:
            result[f"survey-{block.survey_id}"] = settings["vals"]
        elif {"lo", "hi", "inc"} <= settings.keys():
            result[f"survey-{block.survey_id}"] = (settings["lo"][0], settings["hi"][0], settings["inc"][0])
    return result
####


def _print_variables(problem: Problem) -> tuple[str, ...]:
    return tuple(variable for variables, trajectory in _print_scopes(problem) if trajectory is not None for variable in variables)


def _print_scopes(problem: Problem) -> tuple[tuple[tuple[str, ...], int | None], ...]:
    """Retain whether each print block is problem- or trajectory-scoped."""

    scopes: list[tuple[tuple[str, ...], int | None]] = []
    for block in problem.blocks:
        if isinstance(block, PrintBlock):
            scopes.append((tuple(block.variables), None))
    for trajectory in problem.trajectories:
        variables: list[str] = []
        for trajectory_block in trajectory.blocks:
            if isinstance(trajectory_block, PrintBlock):
                variables.extend(trajectory_block.variables)
        for segment in trajectory.segments:
            for segment_block in segment.blocks:
                if isinstance(segment_block, PrintBlock):
                    variables.extend(segment_block.variables)
        if variables:
            scopes.append((tuple(variables), trajectory.number))
    return tuple(scopes)
####
####


def _output_files(problem: Problem, problem_index: int = 0) -> tuple[tuple[str, tuple[str, ...], int | None, int], ...]:
    problem_outputs = ((block.filename, tuple(block.variables), None, problem_index) for block in problem.blocks if isinstance(block, (FileBlock, EgsBlock)) and block.filename)
    trajectory_outputs = (
        (block.filename, tuple(block.variables), trajectory.number, problem_index)
        for trajectory in problem.trajectories
        for block in trajectory.blocks
        if isinstance(block, (FileBlock, EgsBlock)) and block.filename
    )
    return tuple((*problem_outputs, *trajectory_outputs))
####


def _summary_specs(problem: Problem) -> tuple[tuple[str, tuple[SummaryOperation, ...]], ...]:
    return tuple((block.name or "summary", tuple(block.operations)) for block in problem.blocks if isinstance(block, SummarizeBlock))
####


def _summary_egs_specs(problem: Problem, problem_index: int) -> tuple[tuple[str, int, tuple[str, ...]], ...]:
    """Describe EGS summary files and their survey-level dimensions."""

    survey_names = tuple(
        block.name or f"surv-{block.survey_id}"
        for block in problem.blocks
        if isinstance(block, SurveyBlock)
    )
    return tuple(
        (block.filename, problem_index, survey_names)
        for block in problem.blocks
        if isinstance(block, EgsBlock) and block.summary and block.filename
    )
####


def _unsupported_features(problem: Problem) -> tuple[str, ...]:
    """List parsed control blocks that the current runtime cannot execute."""

    features: list[str] = []
    if any(not _is_supported_search(block) for block in problem.blocks if isinstance(block, SearchBlock)):
        features.append("search")
    if any(not _is_supported_optimization(block) for block in problem.blocks if isinstance(block, OptimizeBlock)):
        features.append("optimize")
    return tuple(dict.fromkeys(features))
####


def _search_seed_parameters(problem: Problem) -> dict[str, float]:
    """Seed search placeholders with their documented initial estimates."""

    parameters: dict[str, float] = {}
    for block in problem.blocks:
        if not isinstance(block, SearchBlock) or block.search_id is None:
            continue
        controls = {assignment.name.casefold(): assignment.value for assignment in block.controls}
        if "xest" in controls:
            parameters[f"search-{block.search_id}"] = evaluate_expression(controls["xest"], {}, {})
    return parameters
####


def _optimize_seed_parameters(problem: Problem) -> dict[str, float]:
    """Seed optimization placeholders with the documented parameter values."""

    parameters: dict[str, float] = {}
    for block in problem.blocks:
        if not isinstance(block, OptimizeBlock):
            continue
        for assignment in block.controls:
            name = assignment.name.casefold()
            if name.startswith("par-"):
                index = name.removeprefix("par-")
                loop = block.loop.casefold() if block.loop is not None else ""
                parameters[f"optimize-{loop}-{index}"] = evaluate_expression(assignment.value, {}, {})
                parameters.setdefault(f"optimize-{index}", parameters[f"optimize-{loop}-{index}"])
    return parameters
####


def _is_supported_optimization(block: OptimizeBlock) -> bool:
    """Return whether an optimization has the currently executable shape."""

    names = {assignment.name.casefold() for assignment in block.controls}
    parameter_indices = {name.removeprefix("par-") for name in names if name.startswith("par-")}
    return bool(parameter_indices) and block.objective_variable is not None and block.objective_mode in {"min", "max"} and all({f"lo-{index}", f"hi-{index}"}.issubset(names) for index in parameter_indices)
####


def _is_supported_search(block: SearchBlock) -> bool:
    """Return whether a search has the bounded root controls implemented here."""

    controls = {assignment.name.casefold() for assignment in block.controls}
    return (
        block.search_id is not None
        and block.objective is not None
        and block.objective.operator in {"=", "<", ">"}
        and block.objective.left.expression is not None
        and block.objective.right is not None
        and {"xlo", "xhi", "tol", "maxitr"}.issubset(controls)
    )
####


def _resolve_search_case(
    case: RuntimeCase,
    document: LoweredDocument,
    search: SearchBlock,
    *,
    max_steps: int,
    trial_results: list[ExecutionResult] | None = None,
) -> RuntimeCase:
    """Find one bounded search boundary using the parameters resolved before it."""

    source_problem = document.source_problem
    if source_problem is None:
        raise ValueError("search requires one source problem")
    objective = search.objective
    if search.search_id is None or objective is None or objective.operator not in {"=", "<", ">"}:
        raise ValueError("runtime search requires one equality or inequality objective")
    controls = {assignment.name.casefold(): evaluate_expression(assignment.value, {}, case.parameters) for assignment in search.controls}
    required = ("xlo", "xhi", "xest", "dx", "tol", "maxitr")
    if any(name not in controls for name in required):
        raise ValueError(f"search requires controls {required!r}")
    max_iterations = int(controls["maxitr"])
    if max_iterations < 0:
        raise ValueError("search maxitr must be non-negative")
    if max_iterations == 0:
        # TAOS uses maxitr=0 as a diagnostic mode: execute only the seeded
        # xest trajectory and do not attempt a search update.
        return case
    x_reference = abs(controls.get("xref", 1.0))
    function_reference = abs(controls.get("fref", 1.0))
    if x_reference == 0.0:
        raise ValueError("search xref must be nonzero")
    if function_reference == 0.0:
        raise ValueError("search fref must be nonzero")
    normalized_bounds = (controls["xlo"] / x_reference, controls["xhi"] / x_reference)
    normalized_estimate = controls["xest"] / x_reference
    normalized_spacing = abs(controls["dx"]) / x_reference
    target = objective.right
    if target is None or objective.left.expression is None:
        raise ValueError("search objective is incomplete")
    endpoint_requirements = _optimization_endpoint_requirements(objective.left, target)
    integration_mode = int(controls.get("integ", 1.0))

    def execute_trial(parameters: Mapping[str, float]) -> ExecutionResult:
        trial = RuntimeCase(
            case.index,
            parameters,
            _lower_case(source_problem, parameters, case.index, document.tables, document.unit_settings).problem,
        )
        _apply_trial_integration_mode(trial.problem, endpoint_requirements, integration_mode)
        result = compute_trajectories(
            trial.problem,
            max_steps=max_steps,
            stop_when=_optimization_endpoint_stop_when(endpoint_requirements),
        )
        if trial_results is not None:
            trial_results.append(result)
        return result
    ####

    target_name = target.expression.name.casefold() if isinstance(target.expression, NameExpression) else None
    if target_name in {"min", "max"}:
        def objective_value(normalized_candidate: float) -> float:
            candidate = normalized_candidate * x_reference
            parameters = {**case.parameters, f"search-{search.search_id}": candidate}
            result = execute_trial(parameters)
            value = _endpoint_value(objective.left, result, parameters)
            normalized = value / function_reference
            return -normalized if target_name == "max" else normalized
        ####

        minimum = parabolic_minimize(
            objective_value,
            normalized_estimate,
            normalized_spacing,
            normalized_bounds,
            controls["tol"],
            max_iterations=max_iterations,
        )
        if not minimum.converged:
            minimum = golden_section_minimize(
                objective_value,
                normalized_bounds,
                controls["tol"] / x_reference,
                max_iterations=max_iterations,
            )
        if not minimum.converged:
            raise RuntimeError(f"search {search.search_id} did not converge: {minimum.status}")
        parameters = {**case.parameters, f"search-{search.search_id}": minimum.minimum * x_reference}
        return RuntimeCase(
            case.index,
            parameters,
            _lower_case(source_problem, parameters, case.index, document.tables, document.unit_settings).problem,
        )

    def residual(normalized_candidate: float) -> float:
        candidate = normalized_candidate * x_reference
        parameters = {**case.parameters, f"search-{search.search_id}": candidate}
        result = execute_trial(parameters)
        left = _endpoint_value(objective.left, result, parameters)
        right = _endpoint_value(target, result, parameters)
        return (left - right) / function_reference
    ####

    parabolic = parabolic_root(
        residual,
        normalized_estimate,
        normalized_spacing,
        normalized_bounds,
        controls["tol"],
        max_iterations=max_iterations,
    )
    if parabolic.converged:
        root_value = parabolic.root
    else:
        secant = secant_bracketed_root(
            residual,
            normalized_bounds,
            controls["tol"],
            max_iterations=max_iterations,
        )
        if not secant.converged:
            raise RuntimeError(f"search {search.search_id} did not converge: {secant.status}")
        root_value = secant.root
    parameters = {**case.parameters, f"search-{search.search_id}": root_value * x_reference}
    return RuntimeCase(
        case.index,
        parameters,
        _lower_case(source_problem, parameters, case.index, document.tables, document.unit_settings).problem,
    )
####


def _resolve_optimize_case(case: RuntimeCase, document: LoweredDocument, optimize: OptimizeBlock, *, max_steps: int) -> RuntimeCase:
    """Find a bounded optimum by recomputing the trajectory for each candidate."""

    source_problem = document.source_problem
    if source_problem is None:
        raise ValueError("optimization requires a source problem")
    if not _is_supported_optimization(optimize) or optimize.objective_variable is None or optimize.objective_mode is None:
        raise ValueError("optimization requires bounded parameter controls and an objective")
    controls = {assignment.name.casefold(): evaluate_expression(assignment.value, {}, case.parameters) for assignment in optimize.controls}
    indices = tuple(sorted((name.removeprefix("par-") for name in controls if name.startswith("par-")), key=_optimization_parameter_index))
    objective_name = optimize.objective_variable
    objective_reference = controls.get("fref", 1.0)
    if objective_reference == 0.0:
        raise ValueError("optimization fref must be nonzero")
    bounds = tuple((controls[f"lo-{index}"], controls[f"hi-{index}"]) for index in indices)
    equality_text: list[str] = []
    inequality_text: list[str] = []
    for constraint in optimize.constraints:
        text = f"{constraint.left.text}{constraint.operator}{constraint.right.text}"
        (equality_text if constraint.operator == "=" else inequality_text).append(text)
    program = build_optimization_problem(
        objective_name,
        equality_text,
        inequality_text,
        parameters=tuple(f"par-{index}" for index in indices),
        bounds=bounds,
        references=(objective_reference,),
    )
    case.problem.metadata["optimization_program"] = program
    derivative_step = controls.get("dx", 1.0e-6)
    if derivative_step <= 0.0:
        raise ValueError("optimization dx must be positive")
    max_iterations = int(controls.get("maxitr", 100))
    if max_iterations < 0:
        raise ValueError("optimization maxitr must be non-negative")
    restart_count = int(controls.get("restarts", 0.0))
    if restart_count < 0:
        raise ValueError("optimization restarts must be non-negative")
    if max_iterations == 0:
        # TAOS diagnostic mode evaluates only the seeded trajectory. Do not
        # run refinement or mutate optimization placeholders in this mode.
        return case
    derivative_method = int(controls.get("derivs", 1.0))
    if derivative_method not in {0, 1}:
        raise ValueError("optimization derivs must be 0 (central) or 1 (forward)")
    difference_mode = DifferenceMode.CENTRAL if derivative_method == 0 else DifferenceMode.FORWARD
    objective_endpoint = OptimizeEndpoint(text=objective_name, segment=optimize.segment, trajectory=optimize.trajectory)
    endpoint_requirements = _optimization_endpoint_requirements(
        objective_endpoint,
        tuple(
            endpoint
            for constraint in optimize.constraints
            for endpoint in _qualified_constraint_endpoints(constraint)
        ),
    )
    integration_mode = int(controls.get("integ", 1.0))
    candidate_cache: dict[tuple[float, ...], tuple[ExecutionResult, Mapping[str, float]]] = {}
    base_parameters: dict[str, float] = dict(case.parameters)

    def execute_candidate(candidate: Sequence[float]) -> tuple[ExecutionResult, Mapping[str, float]]:
        point = tuple(float(value) for value in candidate)
        cached = candidate_cache.get(point)
        if cached is not None:
            return cached
        loop = optimize.loop.casefold() if optimize.loop is not None else ""
        updates = {f"optimize-{loop}-{index}": value for index, value in zip(indices, point, strict=True)}
        updates.update({f"optimize-{index}": value for index, value in zip(indices, point, strict=True)})
        parameters = {**base_parameters, **updates}
        trial = RuntimeCase(
            case.index,
            parameters,
            _lower_case(source_problem, parameters, case.index, document.tables, document.unit_settings).problem,
        )
        _apply_trial_integration_mode(trial.problem, endpoint_requirements, integration_mode)
        cached = (
            compute_trajectories(
                trial.problem,
                max_steps=max_steps,
                stop_when=_optimization_endpoint_stop_when(endpoint_requirements),
            ),
            parameters,
        )
        candidate_cache[point] = cached
        return cached

    def objective(candidate: tuple[float, ...]) -> float:
        result, parameters = execute_candidate(candidate)
        try:
            value = _endpoint_value(objective_endpoint, result, parameters, prefer_boundary=True)
        except RuntimeError:
            # A bounded trial may fail to reach a qualified objective segment;
            # keep the optimizer alive and make that candidate unattractive.
            return 1e30
        normalized = value / objective_reference
        return -normalized if optimize.objective_mode == "max" else normalized
    ####

    equality_constraints: list[Callable[[tuple[float, ...]], float]] = []
    inequality_constraints: list[Callable[[tuple[float, ...]], float]] = []
    for constraint in optimize.constraints:
        left_endpoint, right_endpoint = _qualified_constraint_endpoints(constraint)

        def constraint_function(
            candidate: tuple[float, ...],
            constraint: OptimizeConstraint = constraint,
            left_endpoint: OptimizeEndpoint = left_endpoint,
            right_endpoint: OptimizeEndpoint = right_endpoint,
        ) -> float:
            result, candidate_parameters = execute_candidate(candidate)
            try:
                left = _endpoint_value(left_endpoint, result, candidate_parameters, prefer_boundary=True)
                right = _endpoint_value(right_endpoint, result, candidate_parameters, prefer_boundary=True)
            except RuntimeError:
                return 1e6 if constraint.operator == "=" else -1e6
            scale = max(abs(left), abs(right), 1.0)
            residual = left - right if constraint.operator == "=" else right - left if constraint.operator == "<" else left - right
            residual /= scale
            if constraint.reference is not None:
                reference = evaluate_expression(constraint.reference, {}, candidate_parameters)
                if reference == 0.0:
                    raise ValueError("optimization constraint reference must be nonzero")
                residual /= reference
            return residual
        ####

        (equality_constraints if constraint.operator == "=" else inequality_constraints).append(constraint_function)
    ####

    loop = optimize.loop.casefold() if optimize.loop is not None else ""
    initial = tuple(
        base_parameters.get(f"optimize-{loop}-{index}", controls[f"par-{index}"])
        for index in indices
    )
    optimizer = resolve_optimize_block(
        objective,
        bounds,
        equality_constraints=equality_constraints,
        inequality_constraints=inequality_constraints,
        tolerance=controls.get("tol", 1e-7),
        derivative_step=derivative_step,
        difference_mode=difference_mode,
    )
    def merit(point: tuple[float, ...]) -> float:
        equality_penalty = sum(function(point) ** 2 for function in equality_constraints)
        inequality_penalty = sum(max(0.0, -function(point)) ** 2 for function in inequality_constraints)
        return objective(point) + 1000.0 * (equality_penalty + inequality_penalty)
    ####

    def refinement_score(point: tuple[float, ...]) -> tuple[float, float]:
        violation = max(
            (*tuple(abs(function(point)) for function in equality_constraints),
             *tuple(max(0.0, -function(point)) for function in inequality_constraints),
             0.0),
        )
        return violation, merit(point)
    ####

    candidate = initial
    for restart_index in range(restart_count + 1):
        iteration_budget = max_iterations
        if restart_count > 0 and restart_index == restart_count:
            iteration_budget *= 2
        if len(indices) > 4:
            # Finite-differencing every control requires one complete trajectory
            # per coordinate. Coordinate search remains deterministic across
            # restart attempts while refining from the current solution.
            candidate = _coordinate_search(
                candidate,
                bounds,
                merit,
                max_sweeps=iteration_budget,
                tolerance=controls.get("tol", 1e-7),
            )
            converged = False
        else:
            result = optimizer.run(candidate, max_iterations=iteration_budget)
            # A bounded backend may report a stationary or boundary result as stalled;
            # a permitted restart retries from that current solution.
            candidate = result.parameters
            converged = result.converged
        if converged:
            break
        if restart_index < restart_count and int(controls.get("adjust", 0.0)) != 0:
            updates = {f"optimize-{loop}-{index}": value for index, value in zip(indices, candidate, strict=True)}
            updates.update({f"optimize-{index}": value for index, value in zip(indices, candidate, strict=True)})
            base_parameters.update(updates)
            base_parameters = _adjust_guidance_history_parameters(source_problem, base_parameters)
            candidate = tuple(base_parameters.get(f"optimize-{loop}-{index}", value) for index, value in zip(indices, candidate, strict=True))
            candidate_cache.clear()
    if len(indices) <= 2:
        for _ in range(2):
            for position, index in enumerate(indices):
                lower, upper = controls[f"lo-{index}"], controls[f"hi-{index}"]
                grid = tuple(lower + (upper - lower) * step / 20.0 for step in range(21))
                candidate = min(
                    (candidate[:position] + (value,) + candidate[position + 1:] for value in grid),
                    key=refinement_score if equality_constraints or inequality_constraints else merit,
                )
            ####
        ####
    updates = {f"optimize-{loop}-{index}": value for index, value in zip(indices, candidate, strict=True)}
    updates.update({f"optimize-{index}": value for index, value in zip(indices, candidate, strict=True)})
    constraint_tolerance = max(10.0 * controls.get("tol", 1e-7), 1e-6)
    equality_residuals = tuple(function(candidate) for function in equality_constraints)
    inequality_residuals = tuple(function(candidate) for function in inequality_constraints)
    if any(abs(residual) > constraint_tolerance for residual in equality_residuals):
        raise RuntimeError(
            f"optimization {loop or '<unnamed>'} did not satisfy equality constraints: {equality_residuals!r}"
        )
    if any(residual < -constraint_tolerance for residual in inequality_residuals):
        raise RuntimeError(
            f"optimization {loop or '<unnamed>'} did not satisfy inequality constraints: {inequality_residuals!r}"
        )
    parameters = {**case.parameters, **updates}
    resolved = _lower_case(source_problem, parameters, case.index, document.tables, document.unit_settings).problem
    resolved.metadata["optimization_program"] = program
    return RuntimeCase(
        case.index,
        parameters,
        resolved,
    )
####


def _qualified_constraint_endpoints(constraint: OptimizeConstraint) -> tuple[OptimizeEndpoint, OptimizeEndpoint]:
    """Apply one-sided ``on segment`` qualifiers to both constraint endpoints."""

    left, right = constraint.left, constraint.right
    if left.segment is None and right.segment is not None:
        left = left.model_copy(update={"segment": right.segment})
    if right.segment is None and left.segment is not None:
        right = right.model_copy(update={"segment": left.segment})
    if left.trajectory is None and right.trajectory is not None and left.trajectory_subscript is None:
        left = left.model_copy(update={"trajectory": right.trajectory})
    if right.trajectory is None and left.trajectory is not None and right.trajectory_subscript is None:
        right = right.model_copy(update={"trajectory": left.trajectory})
    return left, right
####


def _coordinate_search(
    initial: Sequence[float],
    bounds: Sequence[tuple[float, float]],
    objective: Callable[[tuple[float, ...]], float],
    *,
    max_sweeps: int,
    tolerance: float = 1e-7,
) -> tuple[float, ...]:
    """Run bounded coordinate search with deterministic tolerance refinement."""

    candidate = tuple(
        min(max(float(value), float(lower)), float(upper))
        for value, (lower, upper) in zip(initial, bounds, strict=True)
    )
    if max_sweeps <= 0 or tolerance <= 0.0:
        return candidate
    steps = [max((float(upper) - float(lower)) / 4.0, tolerance) for lower, upper in bounds]
    current = objective(candidate)
    for _ in range(max_sweeps):
        changed = False
        for position, (lower, upper) in enumerate(bounds):
            span = steps[position]
            if span <= tolerance:
                continue
            points = {
                candidate,
                candidate[:position] + (min(max(candidate[position] - span, float(lower)), float(upper)),) + candidate[position + 1:],
                candidate[:position] + (min(max(candidate[position] + span, float(lower)), float(upper)),) + candidate[position + 1:],
            }
            best = min(points, key=objective)
            score = objective(best)
            if score < current:
                candidate, current = best, score
                changed = True
            else:
                steps[position] *= 0.5
        if not changed:
            if max(steps, default=0.0) <= tolerance:
                break
    return candidate
####


def _optimization_parameter_index(index: str) -> tuple[int, int | str]:
    """Order numbered optimization controls numerically, not lexicographically."""

    try:
        return (0, int(index))
    except ValueError:
        return (1, index)
####


def _optimization_endpoint_requirements(
    *endpoints: OptimizeEndpoint | Sequence[OptimizeEndpoint],
) -> tuple[tuple[str, int], ...]:
    """Collect trajectory/segment pairs needed to score one optimization trial."""

    flattened: list[OptimizeEndpoint] = []
    for item in endpoints:
        if isinstance(item, OptimizeEndpoint):
            flattened.append(item)
        else:
            flattened.extend(item)
    return tuple(
        dict.fromkeys(
            (
                str(endpoint.trajectory or endpoint.trajectory_subscript or 1),
                int(endpoint.segment),
            )
            for endpoint in flattened
            if endpoint.segment is not None
        )
    )
####


def _optimization_endpoint_stop_when(
    requirements: Sequence[tuple[str, int]],
) -> Callable[[RuntimeProblem], bool] | None:
    """Stop a candidate trial after all scored segment endpoints are sealed."""

    if not requirements:
        return None

    def stop_when(problem: RuntimeProblem) -> bool:
        sealed: set[tuple[str, int]] = set()
        for trajectory, segment in requirements:
            vehicle = problem.vehicles.get(trajectory)
            if vehicle is None:
                continue
            observed = any(
                math.isclose(state.named.get("_segment", float("nan")), float(segment), abs_tol=1e-9)
                for state in vehicle.history
            )
            if observed and (not vehicle.active or vehicle.segment_number != segment):
                sealed.add((trajectory, segment))
        for trajectory in {name for name, _ in requirements}:
            vehicle = problem.vehicles.get(trajectory)
            vehicle_requirements = {(name, segment) for name, segment in requirements if name == trajectory}
            if vehicle is not None and vehicle_requirements.issubset(sealed):
                vehicle.active = False
                vehicle.activation_pending = False
        return sealed == set(requirements)
    ####

    return stop_when
####


def _assignment_value(block: IntegrationBlock | None, name: str, parameters: Mapping[str, float], default: float) -> float:
    if block is None:
        return default
    assignment = next((item for item in block.assignments if item.name.casefold() == name.casefold()), None)
    return default if assignment is None else evaluate_expression(assignment.value, {}, parameters)
####


def _control_value(
    assignments: Sequence[Assignment],
    name: str,
    parameters: Mapping[str, float],
    default: float,
) -> float:
    """Resolve an optional search/optimization control value."""

    assignment = next((item for item in assignments if item.name.casefold() == name.casefold()), None)
    return default if assignment is None else evaluate_expression(assignment.value, {}, parameters)
####


def _adjust_guidance_history_parameters(problem: Problem, parameters: Mapping[str, float]) -> dict[str, float]:
    """Redistribute ``vrs tseg`` guidance controls across the current time span."""

    adjusted = dict(parameters)
    for trajectory in problem.trajectories:
        for segment in trajectory.segments:
            for block in segment.blocks:
                if not isinstance(block, FlyBlock) or (block.reference or "").casefold() != "tseg" or len(block.points) < 2:
                    continue
                times = tuple(evaluate_expression(point.independent, adjusted, adjusted) for point in block.points)
                controls = tuple((evaluate_expression(point.value, adjusted, adjusted),) for point in block.points)
                if any(left >= right for left, right in zip(times, times[1:], strict=False)):
                    raise ValueError("adjust requires strictly increasing vrs tseg guidance points")
                new_times = tuple(
                    times[0] + (times[-1] - times[0]) * index / (len(times) - 1)
                    for index in range(len(times))
                )
                redistributed = redistribute_control_history(times, controls, new_times)
                for point, new_time, new_control in zip(block.points, new_times, redistributed, strict=True):
                    if isinstance(point.independent, ParameterExpression):
                        adjusted[_parameter_key(point.independent)] = new_time
                    if isinstance(point.value, ParameterExpression):
                        adjusted[_parameter_key(point.value)] = new_control[0]
    return adjusted
####


def _parameter_key(expression: ParameterExpression) -> str:
    """Return the runtime parameter key for a typed placeholder."""

    if expression.family == "optimize" and expression.loop is not None:
        return f"optimize-{expression.loop.casefold()}-{expression.index}"
    return f"{expression.family}-{expression.index}"
####


def _apply_trial_integration_mode(
    problem: RuntimeProblem,
    requirements: Sequence[tuple[str, int]],
    integration_mode: int,
) -> None:
    """Limit ``integ=0`` candidate trials to trajectories needed by the objective."""

    if integration_mode != 0 or not requirements:
        return
    required = {trajectory for trajectory, _ in requirements}
    changed = True
    while changed:
        changed = False
        for name in tuple(required):
            vehicle = problem.vehicles.get(name)
            if vehicle is None:
                continue
            for dependency in vehicle.dependencies:
                if dependency not in required:
                    required.add(dependency)
                    changed = True
    for name, vehicle in problem.vehicles.items():
        if name not in required:
            vehicle.active = False
            vehicle.activation_pending = False
####


def _segment_integration(segment: Segment | None) -> IntegrationBlock | None:
    """Return the integration controls declared by one segment."""

    if segment is None:
        return None
    return next((block for block in segment.blocks if isinstance(block, IntegrationBlock)), None)
####


def _segment_step_size(segment: Segment | None, parameters: Mapping[str, float], default: float) -> float:
    """Resolve a segment's local integration step, falling back safely."""

    return _assignment_value(_segment_integration(segment), "dt", parameters, default)
####


def _segment_guidance_interval(segment: Segment | None, parameters: Mapping[str, float], default: float) -> float:
    """Resolve the segment-local guidance refresh interval."""

    return _assignment_value(_segment_integration(segment), "dtguid", parameters, default)
####


def _write_outputs(index: int, result: ExecutionResult, document: LoweredDocument, destination: Path, problem_index: int = 0) -> None:
    root = Path(destination)
    trajectory_output_files = tuple(item for item in document.output_files if item[2] is not None and item[3] == problem_index)
    problem_output_files = tuple(item for item in document.output_files if item[2] is None and item[3] == problem_index)
    if document.print_scopes:
        for variables, trajectory in document.print_scopes:
            if trajectory is None:
                output_name = f"case-{index}-problem.print" if index > 1 else "problem.print"
                (root / output_name).write_text(
                    _problem_history_text_with_settings(result, variables, document.unit_settings, document.output_formats),
                    encoding="utf-8",
                )
                continue
            name = str(trajectory)
            history = result.states.get(name, ())
            if not history:
                continue
            output_name = f"case-{index}-{name}.print" if index > 1 else f"{name}.print"
            (root / output_name).write_text(
                _history_text_with_settings(history, variables, document.unit_settings, document.output_formats),
                encoding="utf-8",
            )
    else:
        for name, history in result.states.items():
            if not history:
                continue
            variables = history[-1].value_names
            path = root / (f"case-{index}-{name}.print" if index > 1 else f"{name}.print")
            path.write_text(_history_text(history, variables), encoding="utf-8")
    for name, history in result.states.items():
        if not history:
            continue
        variables = history[-1].value_names
        for filename, file_variables, trajectory, _ in trajectory_output_files:
            if str(trajectory) != name:
                continue
            selected = file_variables or variables
            output_name = _case_output_name(filename, index, document, problem_index)
            (root / output_name).write_text(
                _history_text_with_settings(history, selected, document.unit_settings, document.output_formats),
                encoding="utf-8",
            )
    for filename, file_variables, _, _ in problem_output_files:
        selected = file_variables or document.print_variables
        output_name = _case_output_name(filename, index, document, problem_index)
        (root / output_name).write_text(
            _problem_history_text_with_settings(result, selected, document.unit_settings, document.output_formats),
            encoding="utf-8",
        )
    if document.summaries:
        payload = {name: _evaluate_summary_operations(operations, result) for name, operations in document.summaries}
        (root / f"case-{index}-summaries.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
####


def _write_search_trials(
    case_index: int,
    search_id: int,
    trials: Sequence[ExecutionResult],
    document: LoweredDocument,
    destination: Path,
) -> None:
    """Write deterministic diagnostic histories for ``*search print=1``."""

    for trial_index, result in enumerate(trials, start=1):
        for trajectory, history in sorted(result.states.items()):
            if not history:
                continue
            filename = f"search-{search_id}-case-{case_index}-trial-{trial_index:03d}-{trajectory}.print"
            variables = history[-1].value_names
            (destination / filename).write_text(
                _history_text_with_settings(history, variables, document.unit_settings, document.output_formats),
                encoding="utf-8",
            )
####


def _case_output_name(filename: str, index: int, document: LoweredDocument, problem_index: int) -> str:
    """Suffix repeated survey outputs without renaming independent problem files."""

    occurrences = sum(1 for candidate, _, _, _ in document.output_files if candidate == filename)
    repeated_case = document.problem_case_counts[problem_index] > 1 if problem_index < len(document.problem_case_counts) else False
    if problem_index == 0 and index == 1:
        return filename
    if occurrences <= 1 and not repeated_case:
        return filename
    suffix = f"-problem-{problem_index + 1}" if problem_index > 0 else ""
    if repeated_case or problem_index > 0:
        suffix += f"-case-{index}"
    return f"{Path(filename).stem}{suffix}{Path(filename).suffix}"
####


def _evaluate_summary_operations(operations: Sequence[SummaryOperation], result: ExecutionResult) -> float:
    """Evaluate a typed ``*summarize`` operation chain over runtime history."""

    accumulator = 0.0
    for operation in operations:
        operand = _summary_operand_value(operation.operand, result) if operation.operand is not None else None
        name = operation.operation.casefold()
        if name == "add":
            accumulator += _require_summary_operand(name, operand)
        elif name == "sub":
            accumulator -= _require_summary_operand(name, operand)
        elif name == "mult":
            accumulator *= _require_summary_operand(name, operand)
        elif name == "div":
            divisor = _require_summary_operand(name, operand)
            if divisor == 0.0:
                raise ZeroDivisionError("summary division by zero")
            accumulator /= divisor
        elif name == "idiv":
            numerator = _require_summary_operand(name, operand)
            if accumulator == 0.0:
                raise ZeroDivisionError("summary inverse division by zero")
            accumulator = numerator / accumulator
        elif name == "exp":
            accumulator = accumulator ** _require_summary_operand(name, operand)
        elif name == "iexp":
            base = _require_summary_operand(name, operand)
            accumulator = base**accumulator
        else:
            accumulator = _summary_unary(name, accumulator)
    return accumulator
####


def _summary_operand_value(operand: object, result: ExecutionResult) -> float:
    trajectory = getattr(operand, "trajectory", None)
    segment = getattr(operand, "segment", None)
    function = getattr(operand, "function", None)
    expression = getattr(operand, "expression", None)
    if isinstance(expression, NumberExpression):
        return evaluate_expression(expression, {}, {})
    if isinstance(expression, IndexedExpression):
        variable = expression.name.casefold()
        trajectory = expression.index if trajectory is None else trajectory
    elif isinstance(expression, NameExpression):
        variable = expression.name.casefold()
    else:
        variable = str(getattr(operand, "text", "")).casefold()
    histories = result.states if trajectory is None else {str(trajectory): result.states.get(str(trajectory), ())}
    series: list[float] = []
    for history in histories.values():
        states = _summary_history_states(history, segment)
        for state in states:
            value = _output_value(state, variable)
            if not math.isfinite(value):
                raise ValueError(f"summary operand {variable!r} is unavailable in trajectory history")
            series.append(value)
    if not series:
        raise ValueError(f"summary operand {variable!r} has no trajectory history")
    return evaluate_summary(series, function or "last")
####


def _summary_history_states(history: Sequence[RuntimeState], segment: int | None) -> tuple[RuntimeState, ...]:
    """Return summary samples, including exact boundaries for a qualified segment."""

    if segment is None:
        return tuple(history)
    selected = [
        state
        for state in history
        if math.isclose(state.named.get("_segment", float("nan")), float(segment), abs_tol=1.0e-9)
    ]
    boundary: RuntimeState | None = None
    for state in reversed(history):
        boundary = state.segment_endpoints.get(segment)
        if boundary is not None:
            break
    if boundary is not None and not any(math.isclose(state.time, boundary.time, rel_tol=0.0, abs_tol=1.0e-12) for state in selected):
        selected.append(boundary)
        selected.sort(key=lambda state: state.time)
    return tuple(selected)
####


def _require_summary_operand(operation: str, value: float | None) -> float:
    if value is None:
        raise ValueError(f"summary operation {operation!r} requires an operand")
    return value
####


def _summary_unary(operation: str, value: float) -> float:
    import math

    functions = {
        "abs": abs,
        "neg": lambda item: -item,
        "sqr": lambda item: item * item,
        "sqrt": math.sqrt,
        "ln": math.log,
        "log": math.log10,
        "e": math.exp,
        "sin": lambda item: math.sin(math.radians(item)),
        "cos": lambda item: math.cos(math.radians(item)),
        "tan": lambda item: math.tan(math.radians(item)),
        "asin": lambda item: math.degrees(math.asin(item)),
        "acos": lambda item: math.degrees(math.acos(item)),
        "atan": lambda item: math.degrees(math.atan(item)),
    }
    try:
        return float(functions[operation](value))
    except KeyError as exc:
        raise ValueError(f"unsupported summary operation: {operation}") from exc
####


def _write_summary_egs(
    filename: str,
    survey_names: Sequence[str],
    summaries: Sequence[tuple[str, tuple[SummaryOperation, ...]]],
    rows: Sequence[tuple[Mapping[str, float], Mapping[str, float]]],
    destination: Path,
) -> None:
    """Write the deterministic tabular subset of the EGS summary format."""

    summary_names = tuple(name for name, _ in summaries)
    lines = ["*EGS SUMMARY DATA FILE", "*VARIABLES"]
    if survey_names:
        lines.append(f"  LEVEL 1 {' '.join(survey_names)}")
    lines.append(f"  LEVEL 2 {' '.join(summary_names)}")
    lines.append("*TABLE")
    for parameters, payload in rows:
        survey_values = [str(parameters.get(f"survey-{index}", 0.0)) for index in range(1, len(survey_names) + 1)]
        values = [str(payload[name]) for name in summary_names]
        lines.append("  " + " ".join((*survey_values, *values)))
    (destination / filename).write_text("\n".join(lines) + "\n", encoding="utf-8")
####


def _history_text(history: Sequence[RuntimeState], variables: Sequence[str]) -> str:
    return _history_text_with_settings(history, variables, {}, {})
####


def _history_text_with_settings(
    history: Sequence[RuntimeState],
    variables: Sequence[str],
    unit_settings: Mapping[str, str | None],
    output_formats: Mapping[str, str | None],
) -> str:
    history = _sample_history(_history_with_segment_boundaries(history))
    selected = tuple(variable for variable in variables if variable.casefold().split("[", 1)[0] != "time")
    rows = [" ".join(("time", *selected))]
    for state in history:
        rows.append(
            " ".join(
                (
                    _format_named_output(state.time, "time", unit_settings, output_formats),
                    *(_format_named_output(_output_value(state, variable), variable, unit_settings, output_formats) for variable in selected),
                )
            )
        )
    return "\n".join(rows) + "\n"
####


def _problem_history_text(result: ExecutionResult, variables: Sequence[str]) -> str:
    return _problem_history_text_with_settings(result, variables, {}, {})
####


def _problem_history_text_with_settings(
    result: ExecutionResult,
    variables: Sequence[str],
    unit_settings: Mapping[str, str | None],
    output_formats: Mapping[str, str | None],
) -> str:
    """Render a problem-scope output with indexed cross-trajectory values."""

    selected = tuple(variable for variable in variables if variable.casefold().split("[", 1)[0] != "time")
    rows = [" ".join(("time", *selected))]
    histories = tuple(_sample_history(_history_with_segment_boundaries(history)) for history in result.states.values() if history)
    if not histories:
        return "\n".join(rows) + "\n"
    for index in range(max(len(history) for history in histories)):
        snapshots = {
            name: history[min(index, len(history) - 1)]
            for name, history in result.states.items()
            if history
        }
        state = next(iter(snapshots.values()))
        values = dict(state.named)
        aliases = {"xecfc": "x", "yecfc": "y", "zecfc": "z", "xecfcdt": "xdt", "yecfcdt": "ydt", "zecfcdt": "zdt"}
        for variable, source in aliases.items():
            if source in state.named:
                values[variable] = state.named[source]
        for name, snapshot in snapshots.items():
            values[f"time[{name}]"] = snapshot.time
            for variable, source in aliases.items():
                if source in snapshot.named:
                    values[f"{variable}[{name}]"] = snapshot.named[source]
            for variable, value in snapshot.named.items():
                values[f"{variable}[{name}]"] = value
        rows.append(
            " ".join(
                (
                    _format_named_output(state.time, "time", unit_settings, output_formats),
                    *(
                        _format_named_output(
                            _output_reference_value(variable, values), variable, unit_settings, output_formats
                        )
                        for variable in selected
                    ),
                )
            )
        )
    return "\n".join(rows) + "\n"
####


def _format_named_output(
    value: float,
    variable: str,
    unit_settings: Mapping[str, str | None],
    output_formats: Mapping[str, str | None],
) -> str:
    key = variable.casefold().split("[", 1)[0]
    return format_number(from_internal(value, variable, unit_settings), selected_setting(key, output_formats))
####


def _sample_history(history: Sequence[RuntimeState]) -> tuple[RuntimeState, ...]:
    """Interpolate scheduled print states without changing integration history."""

    if len(history) <= 1:
        return tuple(history)
    if history[0].named.get("_dtprnt", 0.0) <= 0.0:
        return tuple(history)
    sampled: list[RuntimeState] = [history[0]]
    previous = history[0]
    interval = previous.named.get("_dtprnt")
    next_tick = _next_print_tick(previous.time, interval)
    for index, state in enumerate(history[1:], start=1):
        segment_changed = state.named.get("_segment") != previous.named.get("_segment")
        if segment_changed:
            if sampled[-1] is not previous:
                sampled.append(previous)
            sampled.append(state)
            interval = state.named.get("_dtprnt")
            next_tick = _next_print_tick(state.time, interval)
        else:
            interval = state.named.get("_dtprnt", interval)
            while interval is not None and interval > 0.0 and next_tick < state.time - 1.0e-12:
                if next_tick > previous.time + 1.0e-12:
                    fraction = (next_tick - previous.time) / (state.time - previous.time)
                    sampled.append(_interpolate_output_state(previous, state, fraction, next_tick))
                next_tick += interval
            if interval is not None and interval > 0.0 and math.isclose(state.time, next_tick, rel_tol=0.0, abs_tol=1.0e-9 * max(1.0, interval)):
                sampled.append(state)
                next_tick += interval
            elif index == len(history) - 1:
                sampled.append(state)
        previous = state
    return tuple(sampled)


def _history_with_segment_boundaries(history: Sequence[RuntimeState]) -> tuple[RuntimeState, ...]:
    """Expose exact pre-transition states before their target-segment samples."""

    expanded: list[RuntimeState] = []
    emitted_segments: set[int] = set()
    for state in history:
        boundaries = sorted(
            (
                (segment, boundary)
                for segment, boundary in state.segment_endpoints.items()
                if segment not in emitted_segments
                and boundary.time <= state.time + 1.0e-12
                and state.named.get("_segment_discontinuity", 0.0) > 0.0
            ),
            key=lambda item: (item[1].time, item[0]),
        )
        for segment, boundary in boundaries:
            if not any(
                candidate is boundary
                or (
                    math.isclose(candidate.time, boundary.time, rel_tol=0.0, abs_tol=1.0e-12)
                    and math.isclose(candidate.named.get("_segment", float("nan")), float(segment), abs_tol=1.0e-9)
                )
                for candidate in expanded
            ):
                expanded.append(boundary)
            emitted_segments.add(segment)
        expanded.append(state)
    return tuple(expanded)
####


def _next_print_tick(time: float, interval: float | None) -> float:
    """Return the first print tick strictly after a state time."""

    if interval is None or interval <= 0.0:
        return math.inf
    return (math.floor(time / interval + 1.0e-10) + 1) * interval


def _interpolate_output_state(start: RuntimeState, end: RuntimeState, fraction: float, time: float) -> RuntimeState:
    """Linearly interpolate numeric output values between accepted states."""

    values = tuple(left + fraction * (right - left) for left, right in zip(start.values, end.values, strict=True))
    names = set(start.named) | set(end.named)
    named = {
        name: start.named.get(name, end.named.get(name, 0.0)) + fraction * (end.named.get(name, start.named.get(name, 0.0)) - start.named.get(name, end.named.get(name, 0.0)))
        for name in names
    }
    named["time"] = time
    return RuntimeState(time, values, end.frame, named, end.value_names, end.segment_endpoints)
####


def _format_output_number(value: float) -> str:
    """Remove insignificant floating-point tails without changing useful precision."""

    numeric = float(value)
    rounded = round(numeric, 12)
    if abs(numeric - rounded) <= 1e-12 * max(1.0, abs(numeric)):
        return str(rounded)
    return str(numeric)
####


def _output_reference_value(variable: str, values: Mapping[str, float]) -> float:
    """Resolve indexed aliases such as ``yecfc[2]`` from a named snapshot."""

    key = variable.casefold()
    if key in values:
        return values[key]
    return float("nan")
####


def _endpoint_state(endpoint: OptimizeEndpoint, result: ExecutionResult, *, prefer_boundary: bool = False) -> RuntimeState:
    """Select the requested trajectory endpoint from retained runtime history."""

    trajectory = str(endpoint.trajectory or endpoint.trajectory_subscript or 1)
    history = result.states.get(trajectory, ())
    if not history:
        raise RuntimeError(f"optimization trajectory {trajectory} produced no history")
    if endpoint.segment is None:
        return history[-1]
    segment = float(endpoint.segment)
    if prefer_boundary:
        for state in reversed(history):
            boundary = state.segment_endpoints.get(endpoint.segment)
            if boundary is not None:
                return boundary
    matches = tuple(
        state
        for state in history
        if math.isclose(state.named.get("_segment", float("nan")), segment, abs_tol=1e-9)
    )
    if not matches:
        raise RuntimeError(f"trajectory {trajectory} produced no history for segment {endpoint.segment}")
    return matches[-1]
####


def _endpoint_value(
    endpoint: OptimizeEndpoint,
    result: ExecutionResult,
    parameters: Mapping[str, float],
    *,
    prefer_boundary: bool = False,
) -> float:
    """Evaluate a qualified search or optimization endpoint."""

    state = _endpoint_state(endpoint, result, prefer_boundary=prefer_boundary)
    if isinstance(endpoint.expression, NameExpression):
        return _output_value(state, endpoint.expression.name)
    if endpoint.expression is not None:
        trajectory = str(endpoint.trajectory or endpoint.trajectory_subscript or 1)
        values = dict(state.named)
        aliases = {
            "xecfc": "x",
            "yecfc": "y",
            "zecfc": "z",
            "xecfcdt": "xdt",
            "yecfcdt": "ydt",
            "zecfcdt": "zdt",
        }
        for alias, source in aliases.items():
            if source in state.named:
                values[alias] = state.named[source]
        values.update({f"{name}[{trajectory}]": value for name, value in state.named.items()})
        return evaluate_expression(endpoint.expression, values, parameters)
    return _output_value(state, endpoint.text)
####


def _output_value(state: RuntimeState, variable: str) -> float:
    """Resolve historical output aliases and reject unavailable output fields."""

    aliases = {"xecfc": "x", "yecfc": "y", "zecfc": "z", "xecfcdt": "xdt", "yecfcdt": "ydt", "zecfcdt": "zdt"}
    name = aliases.get(variable.casefold(), variable.casefold())
    value = state.named.get(name)
    if value is None or not math.isfinite(value):
        raise KeyError(f"output variable {variable!r} is unavailable at time {state.time:g}")
    return value
####


def _summary_chain(operations: Sequence[tuple[str, str | None]], values: Mapping[str, Sequence[float]]) -> float:
    accumulator = 0.0
    for operation, operand in operations:
        operand_value = 0.0
        if operand:
            function = "max" if "max(" in operand else "min" if "min(" in operand else "last"
            variable = operand.split("(")[-1].rstrip(")").casefold()
            if values.get(variable):
                operand_value = evaluate_summary(values[variable], function)
        if operation == "add":
            accumulator += operand_value
        elif operation == "sub":
            accumulator -= operand_value
        elif operation == "mult":
            accumulator *= operand_value
        elif operation in {"div", "idiv"}:
            if operand_value == 0.0:
                raise ZeroDivisionError("summary division by zero")
            accumulator /= operand_value
        elif operation == "neg":
            accumulator = -accumulator
    return accumulator
####
