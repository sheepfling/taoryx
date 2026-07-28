"""Lower parsed TAOS documents into executable runtime cases."""

from __future__ import annotations

import json
import math
import random
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, TypeAlias, cast

from pydantic import BaseModel

from taoryx.aerodynamics import maximum_lift_to_drag
from taoryx.attitude import EulerAngles, euler_angles_to_body_basis
from taoryx.contracts import Angle, Basis3, EarthModel, Frame, FrameVector3, Latitude, Longitude, Quantity, Unit, Vector3
from taoryx.control_allocation import EffectorEffectiveness, EffectorLimits, allocate_and_advance_wrench
from taoryx.controller_realization import ClosedLoopPole, ControllerChannel, ControllerImplementation, ControllerRealization, ControllerRole
from taoryx.coordinates import geocentric_unit_vectors, geodetic_unit_vectors
from taoryx.dynamics import ConstraintMode, apply_rail_constraint
from taoryx.equations.geodesy import CartesianVector3
from taoryx.equations.gravity import gravity_full_geocentric_components
from taoryx.guidance import predictive_intercept, proportional_navigation, range_insensitive_axis, solve_guidance
from taoryx.integration import IntegratorName, normalize_integrator
from taoryx.language.expressions import (
    BinaryExpression,
    ExpressionType,
    IndexedExpression,
    NameExpression,
    NumberExpression,
    ParameterExpression,
    TableReferenceExpression,
    WildcardExpression,
    parse_expression,
)
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language.models import (
    AeroBlock,
    Assignment,
    AtmosBlock,
    CgBlock,
    ConstantsBlock,
    DefineAssignmentStatement,
    DefineBlock,
    DefineControlStatement,
    DofDirectiveBlock,
    DownrangeCrossrangeBlock,
    EarthBlock,
    EgsBlock,
    ExtensionBlock,
    FileBlock,
    FlyBlock,
    IipBlock,
    IncrementBlock,
    InertialBlock,
    InitialBlock,
    IntegrationBlock,
    Limit,
    LimitsBlock,
    ModeBlock,
    OptimizeBlock,
    OptimizeConstraint,
    OptimizeEndpoint,
    PrintBlock,
    Problem,
    ProblemDocument,
    PropulsionBlock,
    RadarBlock,
    RailBlock,
    RandomBlock,
    ResetBlock,
    RuntimeBlock,
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
from taoryx.modes import DynamicsMode, FidelitySetupError, Kinematic6DofState, Quaternion
from taoryx.numeric import DifferenceMode
from taoryx.optimization import build_optimization_problem, redistribute_control_history
from taoryx.output_catalog import output_channel_spec
from taoryx.rigid_body import RIGID_BODY_STATE_NAMES, RigidBody6DofModel, RigidBody6DofState, RigidBodyForceMoment
from taoryx.rigid_body_frames import EarthRotationAdapter
from taoryx.rotorcraft import QuadRotorAllocation
from taoryx.runtime.sensor_clock import SensorClockSpec, SensorRatePolicy, SensorSampleMode, SensorTruthPolicy
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
from taoryx.vehicle import AeroQueryContext, DirectWrenchTableModel, PreparedAerodynamicCoefficients, TableAerodynamicModel

from .common import EventCondition, RuntimeProblem, RuntimeState, RuntimeVehicle
from .engine import ExecutionResult, compute_trajectories
from .environment_runtime import ExponentialAtmosphereProvider, WindFieldEnvironmentProvider, evaluate_wind
from .expressions import _call, evaluate_definition_program, evaluate_expression
from .guidance_control import CoordinatedTurnController, allocate_alpha_bank, limit_vector_norm
from .lqr import (
    GainScheduledLqrController,
    LqrController,
    LqrUncertaintySpec,
    assess_lqr_robustness,
    solve_continuous_lqr,
    solve_scaled_continuous_lqr,
)
from .optimization_runtime import resolve_optimize_block
from .rigid_body import bounded_attitude_moment, rigid_body_vehicle
from .summaries import evaluate_summary
from .surveys import generate_survey_cases
from .truth import KinematicTruthProvider, TranslationTruthProvider
from .units import format_number, from_internal, selected_setting, to_internal

_TABLE_EVALUATOR_CACHE: dict[int, tuple[Mapping[str, RuntimeTable], dict[str, Callable[[Mapping[str, float]], float]]]] = {}
CartesianTriple: TypeAlias = tuple[float, float, float]
PlatformBasis: TypeAlias = tuple[CartesianTriple, CartesianTriple, CartesianTriple]
SurveySpan: TypeAlias = CartesianTriple
####


def _contains_indexed_expression(value: object) -> bool:
    """Return whether a parsed block tree requires indexed trajectory aliases."""

    if isinstance(value, IndexedExpression):
        return True
    if isinstance(value, BaseModel):
        return any(_contains_indexed_expression(child) for child in value.__dict__.values())
    if isinstance(value, Mapping):
        return any(_contains_indexed_expression(child) for child in value.values())
    if isinstance(value, (list, tuple, set, frozenset)):
        return any(_contains_indexed_expression(child) for child in value)
    return False
####


def _contains_named_expression(value: object, names: frozenset[str]) -> bool:
    """Return whether a typed block tree refers to one of the requested names."""

    if isinstance(value, str):
        return value.casefold() in names
    if isinstance(value, NameExpression):
        return value.name.casefold() in names
    if isinstance(value, BaseModel):
        return any(_contains_named_expression(child, names) for child in value.__dict__.values())
    if isinstance(value, Mapping):
        return any(_contains_named_expression(child, names) for child in value.values())
    if isinstance(value, (list, tuple, set, frozenset)):
        return any(_contains_named_expression(child, names) for child in value)
    return False
####


def _contains_derived_rate_reference(value: object) -> bool:
    """Return whether a typed block tree requests a non-state rate alias."""

    if isinstance(value, str):
        name = value.casefold()
        return name.endswith("dt") and name not in {"dt", "dtprnt", "xdt", "ydt", "zdt"}
    if isinstance(value, BaseModel):
        return any(_contains_derived_rate_reference(child) for child in value.__dict__.values())
    if isinstance(value, Mapping):
        return any(_contains_derived_rate_reference(child) for child in value.values())
    if isinstance(value, (list, tuple, set, frozenset)):
        return any(_contains_derived_rate_reference(child) for child in value)
    return False
####


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

    def evaluate(
        self,
        values: Mapping[str, float],
        tables: Mapping[str, RuntimeTable] | None = None,
        *,
        strict_no_extrap: bool = False,
    ) -> float:
        query_values = _table_query_values(values, self.independent_variables)
        if strict_no_extrap and self.prepared is not None and self.prepared.extrapolation is ExtrapolationMode.CLAMP:
            _reject_outside_runtime_table_envelope(self, query_values)
        if self.skewed is not None:
            from taoryx.tables import interpolate_skewed

            if strict_no_extrap and self.skewed.extrapolation is ExtrapolationMode.CLAMP:
                _reject_outside_skewed_runtime_table_envelope(self, query_values)
            return interpolate_skewed(self.skewed, tuple(query_values[name] for name in self.independent_variables))
        if self.prepared is None:
            prepared_tables = {name: table.prepared for name, table in (tables or {}).items() if table.prepared is not None}
            evaluators = {
                name: _RuntimeTableEvaluator(table, tables or {}, strict_no_extrap)
                for name, table in (tables or {}).items()
            }
            return evaluate_full_table(self.operations, TableEvaluationContext.from_values(values, prepared_tables, evaluators)).value
        from taoryx.tables import interpolate_nd

        return interpolate_nd(self.prepared, tuple(query_values[name] for name in self.independent_variables))
    ####
####


@dataclass(frozen=True, slots=True)
class _RuntimeTableEvaluator:
    """Callable table adapter with explicit argumented lookup support."""

    table: RuntimeTable
    tables: Mapping[str, RuntimeTable]
    strict_no_extrap: bool = False

    def __call__(self, values: Mapping[str, float]) -> float:
        return self.table.evaluate(values, self.tables, strict_no_extrap=self.strict_no_extrap)
    ####

    def evaluate_call(self, values: Mapping[str, float], arguments: Sequence[float]) -> float:
        if not self.table.independent_variables:
            if arguments:
                raise ValueError(f"full table {self.table.name!r} does not accept lookup arguments")
            return self.table.evaluate(values, self.tables, strict_no_extrap=self.strict_no_extrap)
        if len(arguments) != len(self.table.independent_variables):
            raise ValueError(
                f"table {self.table.name!r} requires {len(self.table.independent_variables)} lookup arguments"
            )
        query_values = dict(values)
        query_values.update(zip(self.table.independent_variables, arguments, strict=True))
        return self.table.evaluate(query_values, self.tables, strict_no_extrap=self.strict_no_extrap)
    ####
####


def _reject_outside_runtime_table_envelope(
    table: RuntimeTable,
    query_values: Mapping[str, float],
) -> None:
    """Reject a ``no-extrap`` query instead of silently clamping it.

    General TAOS table evaluation retains its historical clamping behavior.
    The rigid-body aerodynamic boundary is stricter: a source coefficient
    deck must not manufacture loads outside its declared evidence envelope.
    """

    assert table.prepared is not None
    for axis_name, axis in zip(table.independent_variables, table.prepared.axes, strict=True):
        value = float(query_values[axis_name])
        lower, upper = min(axis), max(axis)
        tolerance = 1.0e-12 * max(1.0, abs(lower), abs(upper))
        if value < lower - tolerance or value > upper + tolerance:
            distance = min(abs(value - lower), abs(value - upper))
            raise ValueError(
                f"coefficient table {table.name!r} query "
                f"{tuple(float(query_values[name]) for name in table.independent_variables)!r} "
                f"is outside its declared envelope on axis {axis_name!r}: "
                f"value={value}, range=[{lower}, {upper}], "
                f"distance_to_boundary={distance}"
            )
        ####
    ####


def _reject_outside_skewed_runtime_table_envelope(
    table: RuntimeTable,
    query_values: Mapping[str, float],
) -> None:
    """Reject strict queries against both outer and inner skewed axes."""

    assert table.skewed is not None
    axes = table.skewed.outer_axes + (
        tuple(value for item in table.skewed.slices for value in item.axis),
    )
    for axis_name, axis in zip(table.independent_variables, axes, strict=True):
        value = float(query_values[axis_name])
        lower, upper = min(axis), max(axis)
        tolerance = 1.0e-12 * max(1.0, abs(lower), abs(upper))
        if value < lower - tolerance or value > upper + tolerance:
            distance = min(abs(value - lower), abs(value - upper))
            raise ValueError(
                f"coefficient table {table.name!r} query "
                f"{tuple(float(query_values[name]) for name in table.independent_variables)!r} "
                f"is outside its declared envelope on axis {axis_name!r}: "
                f"value={value}, range=[{lower}, {upper}], "
                f"distance_to_boundary={distance}"
            )
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


def _random_case_seed(problem: Problem, case_index: int, seed_override: int | None) -> int:
    """Pick the deterministic seed for one lowered case."""

    if seed_override is not None:
        return seed_override + case_index - 1
    seed = next((block.seed for block in reversed(problem.blocks) if isinstance(block, RandomBlock) and block.seed is not None), 0)
    return seed + case_index - 1


def _sample_random_call(function: str, arguments: Sequence[float], rng: random.Random) -> float:
    """Evaluate one supported stochastic distribution."""

    name = function.casefold().replace("-", "_")
    if name == "uniform":
        if len(arguments) != 2:
            raise ValueError("uniform() requires two arguments")
        return rng.uniform(arguments[0], arguments[1])
    if name in {"normal", "gaussian"}:
        if len(arguments) != 2:
            raise ValueError("normal() requires two arguments")
        return rng.normalvariate(arguments[0], arguments[1])
    if name in {"exponential", "exp"}:
        if len(arguments) != 1:
            raise ValueError("exponential() requires one argument")
        scale = arguments[0]
        if scale <= 0.0:
            raise ValueError("exponential() requires a positive scale")
        return rng.expovariate(1.0 / scale)
    if name == "rayleigh":
        if len(arguments) != 1:
            raise ValueError("rayleigh() requires one argument")
        scale = arguments[0]
        if scale <= 0.0:
            raise ValueError("rayleigh() requires a positive scale")
        return scale * math.sqrt(-2.0 * math.log(max(1.0 - rng.random(), 1.0e-15)))
    if name in {"student_t", "studentt", "student-t", "t"}:
        if len(arguments) not in {1, 2, 3}:
            raise ValueError("student_t() requires one to three arguments")
        df = arguments[0]
        loc = arguments[1] if len(arguments) >= 2 else 0.0
        scale = arguments[2] if len(arguments) >= 3 else 1.0
        if df <= 0.0:
            raise ValueError("student_t() requires a positive degrees-of-freedom value")
        if scale <= 0.0:
            raise ValueError("student_t() requires a positive scale")
        z = rng.normalvariate(0.0, 1.0)
        chi_square = rng.gammavariate(df / 2.0, 2.0)
        return loc + scale * z / math.sqrt(chi_square / df)
    raise ValueError(f"unsupported random function: {function}")


def _sample_problem_random_parameters(
    problem: Problem,
    parameters: Mapping[str, float],
    case_index: int,
    tables: Mapping[str, RuntimeTable],
    *,
    seed: int | None = None,
) -> dict[str, float]:
    """Apply any problem-level stochastic declarations once for one case."""

    random_blocks = tuple(block for block in problem.blocks if isinstance(block, RandomBlock))
    if not random_blocks:
        return dict(parameters)

    rng = random.Random(_random_case_seed(problem, case_index, seed))
    working = dict(parameters)
    table_evaluators = _table_evaluators(tables)

    def call_handler(function: str, arguments: Sequence[float]) -> float:
        return _sample_random_call(function, arguments, rng)

    for block in random_blocks:
        for assignment in block.assignments:
            working[assignment.name.casefold()] = evaluate_expression(
                assignment.value,
                working,
                working,
                tables=table_evaluators,
                call_handler=call_handler,
            )
    return working
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


def lower_problem_document(
    document: ProblemDocument,
    tables: Mapping[str, RuntimeTable] | None = None,
    *,
    seed: int | None = None,
    parameter_overrides: Mapping[str, float] | None = None,
) -> LoweredDocument:
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
        search_seed = _search_seed_parameters(problem)
        declared_parameters = _runtime_parameters(problem)
        resolved_parameters = {**declared_parameters, **(parameter_overrides or {}), **search_seed}
        resolved_parameters.update(_definition_parameter_values(problem, resolved_parameters))
        optimize_seed = _optimize_seed_parameters(problem, resolved_parameters)
        resolved_parameters.update(optimize_seed)
        explicit_cases = _case_parameters(problem, resolved_parameters)
        case_parameters = explicit_cases if explicit_cases else tuple(survey_cases)
        problem_case_counts.append(len(case_parameters))
        case_problem_indices.extend([len(problem_case_counts) - 1] * len(case_parameters))
        cases.extend(
            _lower_case(
                problem,
                _sample_problem_random_parameters(
                    problem,
                    {**resolved_parameters, **parameters},
                    index,
                    tables or {},
                    seed=seed,
                ),
                index,
                tables or {},
                unit_settings,
            )
            for index, parameters in enumerate(case_parameters, start=case_index)
        )
        case_index += len(case_parameters)
        print_variables.extend(_print_variables(problem))
        output_files.extend(_output_files(problem, len(problem_case_counts) - 1))
        summaries.extend(_summary_specs(problem))
        summary_egs_files.extend(_summary_egs_specs(problem, len(problem_case_counts) - 1))
        unsupported.extend(_unsupported_features(problem, document.grammar_profile))
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


def execute_lowered(
    document: LoweredDocument,
    *,
    output_dir: str = ".",
    max_steps: int = 100000,
    integrator: str | None = None,
) -> tuple[ExecutionResult, ...]:
    """Execute cases in order and emit declared output products."""

    from pathlib import Path

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    results: list[ExecutionResult] = []
    summary_rows: dict[int, list[tuple[Mapping[str, float], Mapping[str, float]]]] = {}
    survey_optima: dict[int, dict[str, float]] = {}
    normalized_integrator = normalize_integrator(integrator) if integrator is not None else None
    for case_position, case in enumerate(document.cases):
        problem_index = document.case_problem_indices[case_position] if document.case_problem_indices else 0
        executable_case = _apply_integrator_override(case, normalized_integrator)
        if any(
            _control_value(optimize.controls, "surveys", case.parameters, 0.0) != 0.0
            for optimize in document.optimizations
        ) and problem_index in survey_optima:
            carried = {**case.parameters, **survey_optima[problem_index]}
            if document.source_problem is None:
                raise ValueError("survey optimization requires one source problem")
            executable_case = _apply_integrator_override(_lower_case(
                document.source_problem,
                carried,
                case.index,
                document.tables,
                document.unit_settings,
            ), normalized_integrator)
        for search in document.searches:
            search_trials: list[ExecutionResult] = []
            executable_case = _resolve_search_case(
                executable_case,
                document,
                search,
                max_steps=max_steps,
                trial_results=search_trials,
                integrator=normalized_integrator,
            )
            if _control_value(search.controls, "print", executable_case.parameters, 0.0) != 0.0:
                _write_search_trials(executable_case.index, search.search_id or 0, search_trials, document, destination)
        for optimize in document.optimizations:
            try:
                executable_case = _resolve_optimize_case(
                    executable_case,
                    document,
                    optimize,
                    max_steps=max_steps,
                    integrator=normalized_integrator,
                )
            except RuntimeError as error:
                if "consecutive trials without reaching" not in str(error):
                    raise
                # A bounded source example may request an endpoint beyond the
                # caller's smoke budget. Preserve the seeded executable case
                # and let the normal incomplete-result warning describe the
                # bounded run instead of converting it into a parser/runtime
                # failure.
                executable_case.problem.metadata["optimization_status"] = "endpoint-not-reached-within-budget"
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
    if _dynamics_mode(problem) is DynamicsMode.RIGID_BODY_6DOF:
        return _lower_rigid_body_case(problem, parameters, index, tables, unit_settings)
    ####
    parameters = {**parameters, **_definition_parameter_values(problem, parameters)}
    earth_mu, earth_omega, earth_j2, earth_coefficients = _earth_parameters(problem, parameters)
    environment_evaluator = _atmosphere_evaluator(problem)
    vehicle_attributes = _runtime_attributes(problem, "vehicle")
    alpha_reference_degrees = float(vehicle_attributes.get("aero-alpha-reference-deg", "0.0"))
    point_mass_si_contract = (
        (unit_settings.get("vel") or "").casefold() in {"m/sec", "m/s"}
        and (unit_settings.get("mass") or "").casefold() == "kg"
    )
    route_attributes = _runtime_attributes(problem, "route")
    target_attributes = _runtime_attributes(problem, "target")
    trajectories = {trajectory.number: trajectory for trajectory in problem.trajectories}
    vehicles: list[RuntimeVehicle] = []
    for trajectory in problem.trajectories:
        initial = next((block for block in trajectory.blocks if isinstance(block, InitialBlock)), None)
        deployment = next(
            (block for block in trajectory.blocks if isinstance(block, ExtensionBlock) and block.keyword == "deployed"),
            None,
        )
        deployment_source = (
            trajectories.get(deployment.source_trajectory)
            if deployment is not None and deployment.source_trajectory is not None
            else None
        )
        deployment_initial = (
            next((block for block in deployment_source.blocks if isinstance(block, InitialBlock)), None)
            if deployment_source is not None
            else None
        )
        if initial is None:
            if deployment_initial is None:
                raise ValueError(f"trajectory {trajectory.number} has no initial block")
            initial = deployment_initial
        source_trajectory = (
            deployment_source
            if deployment is not None
            else trajectories.get(initial.source_trajectory)
            if initial.source_trajectory is not None
            else None
        )
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
        # Controls are part of the point-mass aero-table query contract. Make
        # their declared defaults available before initial environment and
        # derived-definition evaluation.
        control_values = _runtime_control_values(problem, str(trajectory.number))
        named.update(control_values)
        named.setdefault("_aero_alpha_reference_deg", alpha_reference_degrees)
        named.setdefault("_point_mass_si_contract", 1.0 if point_mass_si_contract else 0.0)
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
        surface_reference = _surface_helper_context(definition_blocks, named, parameters)
        surface_call_handler = _surface_call_handler(surface_reference)
        definition_controls = tuple(
            statement
            for block in definition_blocks
            for statement in block.typed_statements
            if isinstance(statement, DefineControlStatement)
        )
        # A conditional definition may assign different names in its two
        # branches (for example ``late`` in the true branch and ``early`` in
        # the false branch).  Keep every assigned output visible to file
        # writers and live evaluators; filtering to the block header alone
        # makes the first, inactive branch look undefined.
        definition_output_names = set(definitions)
        for block in definition_blocks:
            for statement in block.typed_statements:
                if isinstance(statement, DefineControlStatement):
                    definition_output_names.update(_define_control_names(statement))
                else:
                    definition_output_names.add(statement.assignment.name.casefold())
        condition_list: list[EventCondition] = []
        segment_events: dict[int, tuple[EventCondition, ...]] = {}
        runtime_events = _runtime_event_conditions(problem, parameters, tables)
        # Point-mass trajectories use geodetic altitude directly and do not
        # pass through the rigid-body safety guard. Add the same fail-closed
        # ground boundary here, while allowing an initial altitude of exactly
        # zero for launch/release fixtures.
        has_rail_launch = any(
            isinstance(block, RailBlock)
            for segment in trajectory.segments
            for block in segment.blocks
        )
        if (initial.coordinate_system or "").casefold() == "geodetic" and not has_rail_launch:
            ground_guard_armed = {"value": float(named.get("alt", named.get("altitude_m", 0.0))) > 1.0e-9}

            def point_mass_altitude(state: RuntimeState) -> float:
                # ``alt`` is the integrated geodetic state. Prefer it over
                # derived Cartesian altitude, whose chord geometry can be
                # slightly below the ellipsoid during horizontal flight.
                return float(state.named.get("alt", state.named.get("altitude_m", 0.0)))
            ####

            def point_mass_ground_residual(state: RuntimeState) -> float:
                altitude = point_mass_altitude(state)
                if altitude > 1.0e-9:
                    ground_guard_armed["value"] = True
                return altitude if ground_guard_armed["value"] else 1.0
            ####

            point_ground_event = EventCondition(
                f"trajectory-{trajectory.number}-earth-intersection",
                point_mass_ground_residual,
                "stop",
                lambda state: ground_guard_armed["value"] and point_mass_altitude(state) < 0.0,
                signal="earth-intersection",
                source="taoryx-point-mass-safety-guard",
            )
            runtime_events = (*runtime_events, point_ground_event)
        segment_by_number = {segment.number: segment for segment in trajectory.segments}
        active_segment = {"number": trajectory.start_segment}
        initial_segment = segment_by_number[trajectory.start_segment]
        step = _segment_step_size(initial_segment, parameters, 1.0)
        guidance_interval = _segment_guidance_interval(initial_segment, parameters, step)
        platform_state: dict[str, object] = {}
        _align_inertial_platform(platform_state, initial_segment, named, parameters, tables, earth_omega)
        event_targets: dict[str, int | None] = {}
        radar_blocks = tuple(block for block in problem.blocks if isinstance(block, RadarBlock))
        reference_blocks = tuple(problem.blocks) + tuple(trajectory.blocks) + tuple(
            block for segment in trajectory.segments for block in segment.blocks
        )
        include_trajectory_references = bool(radar_blocks) or _contains_indexed_expression(reference_blocks)
        derivative_blocks = tuple(
            block
            for block in reference_blocks
            if not isinstance(block, (EgsBlock, FileBlock, PrintBlock, SummarizeBlock, SurveyBlock))
        )
        include_specific_loads_in_derivative = _contains_named_expression(
            derivative_blocks,
            frozenset({"nx", "ny", "nz", "ntotal"}),
        )
        publish_derived_rates = _contains_derived_rate_reference(reference_blocks)
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
            radar_blocks,
            tuple(block for block in problem.blocks if isinstance(block, WindBlock)),
            platform_state,
            earth_omega,
            include_trajectory_references,
            include_specific_loads_in_derivative,
            runtime_controls=control_values,
            route_attributes=route_attributes,
            target_attributes=target_attributes,
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
                        source=f"{segment_block.location.path}:{segment_block.location.line}",
                    )
                    condition_list.append(event)
                    segment_conditions.append(event)
                    event_targets[event.name] = segment_block.target_segment if segment_block.action == "goto" else None
            ####
            segment_events[segment.number] = (*runtime_events, *segment_conditions)
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
                        thrust += _evaluate_propulsion_thrust(
                            thrust_assignment.value,
                            _point_mass_aero_query_values(state.named),
                            parameters,
                            state_tables,
                            1.0,
                        )
                    if mdot_assignment is not None:
                        mass_rate += evaluate_expression(mdot_assignment.value, state.named, parameters, tables=_table_evaluators(state_tables))
            throttle = state.named.get("throttle", 1.0)
            mass_rate *= throttle
            weight_state = "wt" in state.value_names and "mass" not in state.value_names and segment is not None and any(isinstance(block, RailBlock) for block in segment.blocks)
            raw_mass = abs(state.named.get("wt", state.named.get("mass", 1.0)))
            mass = max(raw_mass / 32.174 if weight_state else raw_mass, 1e-12)
            if segment is not None and all(name in state.named for name in ("xdt", "ydt", "zdt")):
                aero_acceleration = _ecfc_aerodynamic_acceleration(segment, state.named, state_tables, parameters, mass)
            else:
                aero_acceleration = _aerodynamic_acceleration(segment, state.named, state_tables, parameters, mass) if segment is not None else (0.0, 0.0, 0.0)
            ecfc_propulsive_acceleration = _ecfc_propulsive_acceleration(segment, state.named, state_tables, parameters, mass, throttle) if segment is not None else (0.0, 0.0, 0.0)
            geodetic_propulsive_acceleration = _geodetic_propulsive_acceleration(segment, state.named, state_tables, parameters, mass, throttle) if segment is not None else None
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
                derived_expressions=definitions,
            )
            ecfc_total_acceleration = tuple(left + right for left, right in zip(aero_acceleration, ecfc_propulsive_acceleration, strict=True))
            ecfc_total_acceleration = tuple(
                value + state.named.get(f"_guidance_a{axis}", 0.0)
                for value, axis in zip(ecfc_total_acceleration, ("x", "y", "z"), strict=True)
            )
            if segment is not None:
                platform_axes = _body_platform_basis(state.named)
                body_axes = (Vector3(*platform_axes[0]), Vector3(*platform_axes[1]), Vector3(*platform_axes[2]))
                constrained = _apply_runtime_rail_constraint(
                    segment,
                    state.named,
                    Vector3(*ecfc_total_acceleration),
                    parameters,
                    body_axes,
                )
                ecfc_total_acceleration = (constrained.x, constrained.y, constrained.z)
            available_acceleration = geodetic_force_rates[0] if geodetic_force_rates is not None else ecfc_total_acceleration[0]
            if geodetic_force_rates is not None:
                if "vel" in rates:
                    rates["vel"] = geodetic_force_rates[0]
                if "gama" in rates:
                    rates["gama"] = geodetic_force_rates[1]
                if "psi" in rates:
                    rates["psi"] = geodetic_force_rates[2]
            elif "vel" in rates:
                rates["vel"] = available_acceleration
            for name, value in zip(("xdt", "ydt", "zdt"), aero_acceleration, strict=True):
                if name in rates and geodetic_force_rates is None:
                    rates[name] = ecfc_total_acceleration[({"xdt": 0, "ydt": 1, "zdt": 2})[name]]
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
            if "vel" in rates and "_command_vel" in state.named and "_guidance_solved" not in state.named:
                rates["vel"] = (state.named["_command_vel"] - velocity) / max(segment_guidance_interval, 1.0e-12)
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

        def stall_detector(
            state: RuntimeState,
            segments: Mapping[int, Segment] = segment_by_number,
            active_segment_ref: dict[str, int] = active_segment,
            state_names: tuple[str, ...] = names,
            derivative_ref: Callable[[RuntimeState], tuple[float, ...]] = derivative,
        ) -> bool:
            """Detect an impossible stationary rail launch before timeout."""

            segment = segments.get(active_segment_ref["number"])
            rail = next((block for block in segment.blocks if isinstance(block, RailBlock)), None) if segment is not None else None
            if rail is None or (rail.mode or "").casefold() != "launch":
                return False
            geodetic = "_geodetic_state" in state.named
            speed = abs(state.named.get("vel", 0.0))
            if not geodetic and {"xdt", "ydt", "zdt"}.issubset(state.named):
                speed = math.sqrt(sum(state.named[name] ** 2 for name in ("xdt", "ydt", "zdt")))
            if speed > 0.001:
                return False
            rates = derivative_ref(state)
            if geodetic:
                if "vel" not in state.value_names:
                    return False
                return rates[state.value_names.index("vel")] <= 0.0
            return math.sqrt(sum(rates[state_names.index(name)] ** 2 for name in ("xdt", "ydt", "zdt") if name in state_names)) <= 0.0
        ####

        def definition_evaluator(
            values: Mapping[str, float],
        ) -> Mapping[str, float]:
            # Re-evaluate all ordinary definitions at the live point-mass
            # table-query boundary.  This prevents a table-backed definition
            # from being frozen in the initialization units (for example,
            # degrees where the source table declares radians) and also keeps
            # control-program definitions on the same path.
            clean_values = {
                key: value
                for key, value in values.items()
                if key.casefold() not in definitions
            }
            working = _point_mass_aero_query_values(clean_values)
            resolved = _evaluate_definition_blocks(definition_blocks, working, parameters, tables)
            return {
                name: resolved[name]
                for name in resolved
                if name in definition_output_names
            }
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

        if _dynamics_mode(problem) is DynamicsMode.KINEMATIC_6DOF:
            initial_named = {
                **initial_state.named,
                "qw": 1.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
            }
            initial_state = RuntimeState(initial_state.time, initial_state.values, initial_state.frame, initial_named, initial_state.value_names, initial_state.segment_endpoints)
        dynamics_mode = _dynamics_mode(problem)
        base_truth_provider = TranslationTruthProvider(
            earth_mu,
            earth_omega,
            length_scale_to_m=0.3048 if point_mass_si_contract else 1.0,
        )
        truth_provider = KinematicTruthProvider(base_truth_provider) if dynamics_mode is DynamicsMode.KINEMATIC_6DOF else base_truth_provider
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
            definition_evaluator=definition_evaluator if definitions or definition_controls else None,
            definition_call_handler=surface_call_handler,
            parameters=parameters,
            control_values=control_values,
            table_evaluators=_table_evaluators(tables),
            environment_evaluator=vehicle_environment,
            event_handlers=event_handlers,
            activation_handler=activation_handler,
            dynamics_mode=dynamics_mode,
            body_rate_provider=_build_kinematic_body_rate_provider(problem) if dynamics_mode is DynamicsMode.KINEMATIC_6DOF else None,
            publish_derived_rates=publish_derived_rates,
            kinematic_state=_kinematic_state(initial_state) if dynamics_mode is DynamicsMode.KINEMATIC_6DOF else None,
            stall_detector=stall_detector,
            truth_provider=truth_provider,
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
                derived = evaluate_definition_program(
                    definitions,
                    initial_named,
                    parameters=parameters,
                    table_evaluators=_table_evaluators(tables),
                    call_handler=surface_call_handler,
                )
            except KeyError:
                # Indexed problem-scope definitions may need a later trajectory.
                derived = initial_named
            vehicle.state = RuntimeState(vehicle.state.time, vehicle.state.values, named=derived, value_names=names)
            vehicle.history[0] = vehicle.state
        if definition_evaluator is not None:
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
                call_handler=vehicle.definition_call_handler,
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
    runtime = RuntimeProblem(
        {vehicle.name: vehicle for vehicle in vehicles},
        sensor_clocks=_runtime_sensor_clocks(problem),
    )
    runtime.metadata["dynamics_mode"] = _dynamics_mode(problem).value
    runtime.metadata["parameters"] = dict(parameters)
    runtime.metadata["tables"] = tables
    _register_sensor_clock_metadata(runtime)
    runtime.metadata["runtime_model_bindings"] = _runtime_model_bindings(problem)
    runtime.metadata["coupled_trajectories"] = any(
        isinstance(block, RadarBlock)
        for block in problem.blocks
    ) or any(
        isinstance(block, FlyBlock) and (block.guidance_variable or "").casefold() in {"intercept", "propnav"}
        for trajectory in problem.trajectories
        for segment in trajectory.segments
        for block in segment.blocks
    )
    return RuntimeCase(index, parameters, runtime)
####


def _lower_rigid_body_case(
    problem: Problem,
    parameters: Mapping[str, float],
    index: int,
    tables: Mapping[str, RuntimeTable],
    unit_settings: Mapping[str, str | None],
) -> RuntimeCase:
    """Lower the first executable TAORYX rigid-body problem subset.

    This deliberately starts with explicit ECIC initial conditions and a
    body-x propulsion assignment. It establishes the runtime seam without
    guessing how historical geodetic or body-aerodynamic syntax should map to
    the successor model.
    """

    earth_mu, _, _, _ = _earth_parameters(problem, parameters)
    earth_mu = _rigid_body_gravitational_parameter(problem, earth_mu)
    if not problem.trajectories:
        raise FidelitySetupError(
            "missing-trajectory",
            "rigid-body mode requires at least one trajectory",
            "add a *trajectory block with an initial state and at least one segment",
            field="trajectory",
        )
    trajectory = problem.trajectories[0]
    initial = next((block for block in trajectory.blocks if isinstance(block, InitialBlock)), None)
    if initial is None:
        raise FidelitySetupError(
            "missing-initial-state",
            "rigid-body mode requires an initial block",
            "add an ECIC or geodetic *initial declaration containing position, velocity, and mass",
            field="initial",
        )
    _, _, named, start_time = _initial_values(initial, parameters, tables, unit_settings)
    if initial.coordinate_system == "geodetic":
        # The point-mass normalizer uses the historical spherical radius. The
        # rigid-body kernel has an explicit WGS-84 Earth surface, so convert
        # geodetic source syntax at this boundary rather than rejecting a
        # valid successor example.
        radius = 6_378_137.0 + named.get("alt", 0.0)
        longitude = math.radians(named.get("long", 0.0))
        latitude = math.radians(named.get("lat", 0.0))
        named["x"] = radius * math.cos(latitude) * math.cos(longitude)
        named["y"] = radius * math.cos(latitude) * math.sin(longitude)
        named["z"] = radius * math.sin(latitude)
        gamma = math.radians(named.get("gama", 0.0))
        heading = math.radians(named.get("psi", 0.0))
        east = named.get("vel", 0.0) * math.cos(gamma) * math.sin(heading)
        north = named.get("vel", 0.0) * math.cos(gamma) * math.cos(heading)
        up = named.get("vel", 0.0) * math.sin(gamma)
        named["xdt"] = up * math.cos(latitude) * math.cos(longitude) - north * math.sin(latitude) * math.cos(longitude) - east * math.sin(longitude)
        named["ydt"] = up * math.cos(latitude) * math.sin(longitude) - north * math.sin(latitude) * math.sin(longitude) + east * math.cos(longitude)
        named["zdt"] = up * math.sin(latitude) + north * math.cos(latitude)
    if "mass" not in named and "wt" in named:
        named["mass"] = named["wt"]
    required = ("x", "y", "z", "xdt", "ydt", "zdt", "mass")
    missing = tuple(name for name in required if name not in named)
    if missing:
        raise FidelitySetupError(
            "missing-rigid-state-channel",
            f"rigid-body initial state is missing: {', '.join(missing)}",
            "declare x, y, z, xdt, ydt, zdt, and mass, or use a supported geodetic initial form",
            field="initial",
        )
    body_rate = Vector3(named.get("wx", 0.0), named.get("wy", 0.0), named.get("wz", 0.0))
    initial_state = RigidBody6DofState(
        start_time,
        FrameVector3(Vector3(named["x"], named["y"], named["z"]), Frame.ECIC),
        FrameVector3(Vector3(named["xdt"], named["ydt"], named["zdt"]), Frame.ECIC),
        Quaternion(
            named.get("qw", 1.0),
            named.get("qx", 0.0),
            named.get("qy", 0.0),
            named.get("qz", 0.0),
        ).normalized(),
        body_rate,
        named["mass"],
        named.get("propellant_mass", named["mass"]),
        named.get("heat_load", 0.0),
        named.get("peak_heat_rate", 0.0),
    )
    segments = {item.number: item for item in trajectory.segments}
    if not segments:
        raise FidelitySetupError(
            "missing-segment",
            "rigid-body mode requires at least one segment",
            "add a *segment block with an integration step and stop/transition condition",
            field="segment",
        )
    active_segment = {"number": trajectory.start_segment}
    control_values = _runtime_control_values(problem, str(trajectory.number))
    vehicle_attributes = _runtime_attributes(problem, "vehicle")
    actuator_attributes = _runtime_attributes(problem, "actuator")
    target_attributes = _runtime_attributes(problem, "target")
    guidance_attributes = _runtime_attributes(problem, "guidance")
    route_attributes = _runtime_attributes(problem, "route")
    thermal_attributes = _runtime_attributes(problem, "thermal")
    aero_load_mode = vehicle_attributes.get("aero-load-mode", "coefficient")
    rotor_allocation = _runtime_rotor_allocation(vehicle_attributes)
    minimum_air_data_speed_m_s = max(0.0, float(vehicle_attributes.get("minimum-air-data-speed-m-s", "0.1")))
    inertia = Vector3(
        float(vehicle_attributes.get("inertia-x", "1.0")),
        float(vehicle_attributes.get("inertia-y", "1.0")),
        float(vehicle_attributes.get("inertia-z", "1.0")),
    )
    inertia_provider = _runtime_inertia_provider(vehicle_attributes, inertia, named["mass"])
    attitude_lqr = _build_attitude_lqr(problem, inertia, actuator_attributes, tables, initial_mass_kg=named["mass"])
    rate_lqr = _build_attitude_lqr(
        problem,
        inertia,
        actuator_attributes,
        tables,
        initial_mass_kg=named["mass"],
        controller_name="rate",
    )
    def controller_state_provider() -> object | None:
        return getattr(vehicle, "controller_state", None)

    segment = segments[trajectory.start_segment]
    step = _segment_step_size(segment, parameters, 0.01)
    earth_omega = _earth_parameters(problem, parameters)[1]
    attitude_earth = EarthRotationAdapter(
        EarthModel(
            Quantity(6_378_137.0, Unit.METER),
            0.0,
            Quantity(max(earth_mu, 1.0), Unit.METER_CUBED_PER_SECOND_SQUARED),
            Quantity(earth_omega, Unit.RADIAN_PER_SECOND),
        )
    )
    aerodynamic_model = _rigid_body_aerodynamic_model(
        problem,
        tables,
        earth_mu,
        earth_omega,
        control_values,
        target_attributes=target_attributes,
        guidance_attributes=guidance_attributes,
        route_attributes=route_attributes,
        vehicle_attributes=vehicle_attributes,
        actuator_attributes=actuator_attributes,
        reference_area=float(vehicle_attributes.get("reference-area", "1.0")),
        reference_length=float(vehicle_attributes.get("reference-length", "1.0")),
        aero_load_mode=aero_load_mode,
        aero_wrench_frame=vehicle_attributes.get("aero-wrench-frame", "taoryx"),
        alpha_reference_degrees=float(vehicle_attributes.get("aero-alpha-reference-deg", "0.0")),
        parameters=parameters,
        wind_blocks=tuple(block for block in problem.blocks if isinstance(block, WindBlock)),
        rotor_allocation=rotor_allocation,
        rate_controller=rate_lqr,
        inertia_provider=inertia_provider,
        controller_state_provider=controller_state_provider,
    )
    controller_saturated = {"value": False}
    ground_contact_mode = vehicle_attributes.get("ground-contact-mode", "none").casefold()
    ground_contact_segment_text = vehicle_attributes.get("ground-contact-segment")
    ground_contact_segment = int(ground_contact_segment_text) if ground_contact_segment_text is not None else None
    if ground_contact_mode not in {"none", "", "settle", "static", "clamp"}:
        raise ValueError(f"unsupported ground-contact-mode {ground_contact_mode!r}")
    if ground_contact_mode not in {"none", ""} and ground_contact_segment not in segments:
        raise ValueError("ground-contact-segment must identify an existing rigid-body segment")
    coordinated_turn_enabled = _runtime_coordinated_turn_enabled(guidance_attributes, route_attributes)
    coordinated_turn_controller = CoordinatedTurnController(
        heading_gain=float(guidance_attributes.get("rectangle-heading-gain-nm-per-rad", "0.0")),
        bank_gain=float(guidance_attributes.get("rectangle-bank-gain-nm-per-rad", "0.0")),
        heading_rate_damping=float(guidance_attributes.get("rectangle-heading-rate-damping-nm-s-per-rad", "0.0")),
        bank_rate_damping=float(guidance_attributes.get("rectangle-bank-rate-damping-nm-s-per-rad", "0.0")),
        maximum_moment=float(actuator_attributes["maximum-moment"]) if "maximum-moment" in actuator_attributes else None,
    )
    def force_moment(state: RigidBody6DofState) -> RigidBodyForceMoment:
        control_values["_rotor_guidance_moment_x"] = 0.0
        control_values["_rotor_guidance_moment_y"] = 0.0
        control_values["_rotor_guidance_moment_z"] = 0.0
        # Keep the direct-moment contract explicit.  These channels are
        # reset for every derivative evaluation because the RK4 stages share
        # the runtime control dictionary; stale controller requests must not
        # be mistaken for an applied command at a later accepted state.
        control_values["direct_moment_active"] = 0.0
        control_values["direct_moment_source_cancellation_active"] = 0.0
        control_values["direct_local_alpha_error_deg"] = 0.0
        control_values["direct_local_beta_error_deg"] = 0.0
        control_values["direct_local_heading_error_deg"] = 0.0
        control_values["direct_force_source_side_cancellation_active"] = 0.0
        control_values["direct_force_source_side_n"] = 0.0
        control_values["direct_force_beta_error_deg"] = 0.0
        control_values["direct_force_beta_correction_n"] = 0.0
        control_values["direct_force_beta_damping_n"] = 0.0
        for axis in ("x", "y", "z"):
            control_values[f"direct_moment_source_moment_{axis}_nm"] = 0.0
        for axis in ("x", "y", "z"):
            control_values[f"direct_moment_controller_request_{axis}_nm"] = 0.0
            control_values[f"direct_moment_commanded_{axis}_nm"] = 0.0
            control_values[f"direct_moment_applied_{axis}_nm"] = 0.0
        if ground_contact_segment is not None and active_segment["number"] == ground_contact_segment:
            # The contact segment is deliberately a small, explicit static-pad
            # model.  It cancels gravity with a normal reaction and damps any
            # residual in-plane velocity/rate after the collision impulse.  It
            # is not a landing-gear, tire, or ground-effect model.
            gravity_ecic = gravity(state)
            contact_damping = max(float(vehicle_attributes.get("ground-contact-damping-per-s", "8.0")), 0.0)
            contact_force_ecic = gravity_ecic.scaled(-state.mass) - state.velocity.vector.scaled(state.mass * contact_damping)
            contact_force_body = state.attitude.conjugate().rotate(contact_force_ecic)
            contact_moment = state.body_rate.scaled(-contact_damping)
            return RigidBodyForceMoment(
                contact_force_body,
                contact_moment,
                0.0,
                0.0,
                propulsion_force_body=contact_force_body,
                propulsion_moment_body=contact_moment,
            )
        selected = None if controller_state_provider is None else controller_state_provider()
        if isinstance(selected, RigidBody6DofState):
            state = selected
        airspeed_target_text = guidance_attributes.get("airspeed-hold-target-mps")
        airspeed_gain_text = guidance_attributes.get("airspeed-hold-gain-throttle-per-mps")
        if airspeed_target_text is not None and airspeed_gain_text is not None:
            if aerodynamic_model is not None:
                actual_airspeed = aerodynamic_model.air_velocity_body(state).norm()
            else:
                actual_airspeed = state.velocity.vector.norm()
            base_throttle = control_values.setdefault("_airspeed-hold-base-throttle", control_values.get("throttle", 1.0))
            commanded_throttle = base_throttle + float(airspeed_gain_text) * (float(airspeed_target_text) - actual_airspeed)
            lower_throttle = float(guidance_attributes.get("airspeed-hold-min-throttle", "0.0"))
            upper_throttle = float(guidance_attributes.get("airspeed-hold-max-throttle", "1.0"))
            control_values["throttle"] = max(lower_throttle, min(upper_throttle, commanded_throttle))
            control_values["airspeed_hold_error_mps"] = float(airspeed_target_text) - actual_airspeed
        thrust_vector = Vector3(0.0, 0.0, 0.0)
        mass_rate = 0.0
        current_segment = segments[active_segment["number"]]
        # A table-backed engine deck may be scheduled by Mach and airspeed in
        # addition to altitude and throttle.  Build that query from the same
        # air-data model used by the aerodynamic loads instead of silently
        # substituting inertial speed or requiring a hand-written constant
        # thrust.  Retain the sampled result below so propulsion and aero use
        # one accepted truth-state air-data evaluation for this force commit.
        aero_sample = (
            aerodynamic_model.evaluate(state)
            if aerodynamic_model is not None and any(isinstance(block, AeroBlock) for block in current_segment.blocks)
            else None
        )
        propulsion_speed_mps = aerodynamic_model.air_velocity_body(state).norm() if aerodynamic_model is not None else state.velocity.vector.norm()
        propulsion_query = {
            "time": state.time,
            "mass": state.mass,
            "altitude_m": max(state.position.vector.norm() - 6_378_137.0, 0.0),
            "airspeed_m_s": aero_sample.airspeed_m_s if aero_sample is not None else state.velocity.vector.norm(),
            "mach": aero_sample.mach if aero_sample is not None else 0.0,
            "speed_of_sound_m_s": aero_sample.speed_of_sound_m_s if aero_sample is not None else 0.0,
            # Propulsion maps are calibrated against vehicle airspeed, not
            # inertial transport speed introduced by a rotating Earth.  Keep
            # the air-data fields above for Mach-scheduled decks while using
            # the same sampled airspeed for the generic velocity key.
            "velocity_m_s": aero_sample.airspeed_m_s if aero_sample is not None else propulsion_speed_mps,
            "throttle": control_values.get("throttle", 1.0),
        }
        for block in current_segment.blocks:
            if not isinstance(block, PropulsionBlock):
                continue
            assignments = {assignment.name.casefold(): assignment.value for assignment in block.assignments}
            if "thrust" in assignments:
                thrust_value = _evaluate_propulsion_thrust(
                    assignments["thrust"],
                    propulsion_query,
                    parameters,
                    tables,
                    control_values.get("throttle", 1.0),
                )
                ep1 = math.radians(evaluate_expression(assignments["ep1"], propulsion_query, parameters, tables=_table_evaluators(tables))) if "ep1" in assignments else 0.0
                ep2 = math.radians(evaluate_expression(assignments["ep2"], propulsion_query, parameters, tables=_table_evaluators(tables))) if "ep2" in assignments else 0.0
                thrust_vector = thrust_vector + Vector3(
                    thrust_value * math.cos(ep1),
                    -thrust_value * math.sin(ep1) * math.cos(ep2),
                    -thrust_value * math.sin(ep1) * math.sin(ep2),
                )
            if "mdot" in assignments:
                mass_rate += evaluate_expression(assignments["mdot"], propulsion_query, parameters, tables=_table_evaluators(tables))
        mass_rate *= control_values.get("throttle", 1.0)
        standard_propnav = _segment_uses_guidance(current_segment, "propnav")
        route_velocity = _runtime_route_velocity(route_attributes, target_attributes, state, earth_omega) if standard_propnav else None
        flight_path_target_text = guidance_attributes.get("flight-path-hold-target-deg")
        if flight_path_target_text is not None and aerodynamic_model is not None and not (aero_load_mode.casefold() == "direct-wrench" and rotor_allocation is not None):
            air_velocity_body = aerodynamic_model.air_velocity_body(state)
            air_velocity_ecic = state.attitude.rotate(air_velocity_body)
            radial = state.position.vector.scaled(1.0 / max(state.position.vector.norm(), 1.0e-12))
            airspeed = max(air_velocity_ecic.norm(), 1.0e-12)
            radial_speed = air_velocity_ecic.dot(radial)
            if route_velocity is None:
                horizontal_velocity = air_velocity_ecic - radial.scaled(radial_speed)
            else:
                # Preserve the horizontal route command while replacing only
                # its radial component.  This lets a native flight-path hold
                # compose with great-circle/pro-nav guidance instead of
                # silently disabling the route whenever both are present.
                route_radial_speed = route_velocity.dot(radial)
                horizontal_velocity = route_velocity - radial.scaled(route_radial_speed)
            horizontal_speed = max(horizontal_velocity.norm(), 1.0e-12)
            target_gamma_deg = float(flight_path_target_text)
            transition_text = guidance_attributes.get("flight-path-hold-transition-s")
            if transition_text is not None and state.time >= float(transition_text):
                target_gamma_deg = float(
                    guidance_attributes.get(
                        "flight-path-hold-after-target-deg",
                        target_gamma_deg,
                    )
                )
            altitude_target_text = target_attributes.get("altitude-m")
            if transition_text is not None and state.time >= float(transition_text):
                altitude_target_text = guidance_attributes.get(
                    "flight-path-hold-after-altitude-m",
                    altitude_target_text,
                )
            altitude_gain_text = guidance_attributes.get("flight-path-hold-altitude-gain-deg-per-m")
            if altitude_target_text is not None and altitude_gain_text is not None:
                current_altitude = state.position.vector.norm() - 6_378_137.0
                target_gamma_deg += float(altitude_gain_text) * (float(altitude_target_text) - current_altitude)
                gamma_limit_text = guidance_attributes.get("flight-path-hold-max-target-deg")
                if gamma_limit_text is not None:
                    gamma_limit = abs(float(gamma_limit_text))
                    target_gamma_deg = max(-gamma_limit, min(gamma_limit, target_gamma_deg))
            target_gamma_rad = math.radians(target_gamma_deg)
            route_velocity = (
                horizontal_velocity.scaled(math.cos(target_gamma_rad) * airspeed / horizontal_speed)
                + radial.scaled(math.sin(target_gamma_rad) * airspeed)
            )
            control_values["flight_path_angle_deg"] = math.degrees(math.asin(max(-1.0, min(1.0, radial_speed / airspeed))))
            control_values["flight_path_error_deg"] = target_gamma_deg - control_values["flight_path_angle_deg"]
        if (standard_propnav and route_velocity is None) or _runtime_pure_propnav_active(route_attributes, state.time):
            command = _runtime_propnav_command(
                state,
                target_attributes,
                float(guidance_attributes.get("propnav-gain", "0.0")),
                earth_mu,
                earth_omega,
            )
            demand = cast(Vector3, command["demand"])
            horizon = max(float(route_attributes.get("propnav-attitude-horizon-s", "1.0")), 0.01)
            route_velocity = state.velocity.vector + demand.scaled(horizon)
        thrust_moment = Vector3(0.0, 0.0, 0.0)
        route_attitude_error = Vector3(0.0, 0.0, 0.0)
        surface_authority = guidance_attributes.get("fixed-wing-control-authority", "direct-moment").casefold()
        surface_inversion = guidance_attributes.get("surface-control-inversion", "false").casefold() in {"1", "true", "yes"}
        rotorcraft_guidance = aero_load_mode.casefold() == "direct-wrench" and rotor_allocation is not None
        if route_velocity is not None and not rotorcraft_guidance:
            route_direction = route_velocity.scaled(1.0 / max(route_velocity.norm(), 1.0e-12))
            coordinated_turn = _runtime_coordinated_turn_enabled(guidance_attributes, route_attributes)
            fixed_wing_surface_law = guidance_attributes.get(
                "fixed-wing-surface-control-law",
                "",
            ).casefold()
            rectangle_bank = (
                _runtime_rectangle_bank_angle(route_attributes, state)
                if attitude_lqr is not None or not coordinated_turn or fixed_wing_surface_law in {"bank-pitch-pd", "bank_pitch_pd"}
                else None
            )
            desired_body_x_ecic = route_direction
            fixed_wing_alpha = _runtime_fixed_wing_alpha_command_rad(guidance_attributes)
            fixed_wing_beta = _runtime_fixed_wing_beta_command_rad(guidance_attributes)
            if abs(fixed_wing_alpha) > 0.0 or abs(fixed_wing_beta) > 0.0:
                radial = state.position.vector.scaled(1.0 / max(state.position.vector.norm(), 1.0e-12))
                horizontal_route = route_direction - radial.scaled(route_direction.dot(radial))
                horizontal_route = horizontal_route.scaled(1.0 / max(horizontal_route.norm(), 1.0e-12))
                lateral_route = radial.cross(horizontal_route)
                lateral_route = lateral_route.scaled(1.0 / max(lateral_route.norm(), 1.0e-12))
                body_heading = horizontal_route.scaled(math.cos(fixed_wing_beta)) + lateral_route.scaled(math.sin(fixed_wing_beta))
                body_heading = body_heading.scaled(1.0 / max(body_heading.norm(), 1.0e-12))
                flight_path_angle = math.asin(max(-1.0, min(1.0, route_direction.dot(radial))))
                body_pitch = flight_path_angle + fixed_wing_alpha
                desired_body_x_ecic = body_heading.scaled(math.cos(body_pitch)) + radial.scaled(math.sin(body_pitch))
                desired_body_x_ecic = desired_body_x_ecic.scaled(1.0 / max(desired_body_x_ecic.norm(), 1.0e-12))
            if rectangle_bank is not None:
                radial = state.position.vector.scaled(1.0 / max(state.position.vector.norm(), 1.0e-12))
                lateral_ecic = radial.cross(desired_body_x_ecic)
                lateral_ecic = lateral_ecic.scaled(1.0 / max(lateral_ecic.norm(), 1.0e-12))
                down_perpendicular = Vector3(0.0, 0.0, 0.0) - radial
                down_perpendicular = down_perpendicular - desired_body_x_ecic.scaled(
                    down_perpendicular.dot(desired_body_x_ecic)
                )
                down_perpendicular = down_perpendicular.scaled(1.0 / max(down_perpendicular.norm(), 1.0e-12))
                # ``local_roll_deg`` uses the local NED body-axis convention
                # below (body +Y toward local down is positive roll).  The
                # previous subtraction generated the opposite-sign local
                # roll target, which made a direct-moment racetrack request
                # a positive bank and then report/track a negative bank.
                desired_body_z_ecic = down_perpendicular.scaled(math.cos(rectangle_bank)) + lateral_ecic.scaled(math.sin(rectangle_bank))
                desired_body_y_ecic = desired_body_z_ecic.cross(desired_body_x_ecic)
                desired_body_axes = (
                    state.attitude.conjugate().rotate(desired_body_x_ecic),
                    state.attitude.conjugate().rotate(desired_body_y_ecic),
                    state.attitude.conjugate().rotate(desired_body_z_ecic),
                )
                current_body = (Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0), Vector3(0.0, 0.0, 1.0))
                attitude_error = sum(
                    (current.cross(target) for current, target in zip(current_body, desired_body_axes, strict=True)),
                    Vector3(0.0, 0.0, 0.0),
                ).scaled(0.5)
            else:
                desired_body = state.attitude.conjugate().rotate(desired_body_x_ecic)
                attitude_error = Vector3(1.0, 0.0, 0.0).cross(desired_body)
            route_attitude_error = attitude_error
            maximum_sideslip_text = guidance_attributes.get("max-sideslip-deg")
            if maximum_sideslip_text is not None and rectangle_bank is None:
                desired_body = _limit_body_direction_sideslip(
                    desired_body,
                    math.radians(abs(float(maximum_sideslip_text))),
                )
            maximum_pitch_text = guidance_attributes.get("max-pitch-deg")
            if maximum_pitch_text is not None and rectangle_bank is None:
                desired_body = _limit_body_direction_pitch(
                    desired_body,
                    math.radians(abs(float(maximum_pitch_text))),
                )
            # Recompute the control error after command projections.  The
            # sideslip/pitch limits are controller contracts: retaining the
            # pre-projection error would make the declared limits telemetry
            # only and would continue steering toward the rejected direction.
            if rectangle_bank is None:
                attitude_error = Vector3(1.0, 0.0, 0.0).cross(desired_body)
                route_attitude_error = attitude_error
            attitude_gain = float(guidance_attributes.get("attitude-gain", "10000.0"))
            rate_damping = float(guidance_attributes.get("rate-damping", "2000.0"))
            maximum_moment_text = actuator_attributes.get("maximum-moment", guidance_attributes.get("maximum-moment"))
            maximum_moment = float(maximum_moment_text) if maximum_moment_text is not None else None
            maximum_body_rate_text = actuator_attributes.get("maximum-body-rate-deg-s")
            maximum_body_rate = math.radians(float(maximum_body_rate_text)) if maximum_body_rate_text is not None else None
            if surface_authority in {"surfaces", "control-surfaces", "aero-surfaces"}:
                # Surface authority is deliberately opt-in.  It leaves the
                # source aerodynamic control derivatives in the force/moment
                # path and avoids treating an arbitrary direct moment as a
                # vehicle actuator.
                thrust_moment = Vector3(0.0, 0.0, 0.0)
            elif attitude_lqr is None:
                direct_control_law = guidance_attributes.get("fixed-wing-direct-control-law", "").casefold()
                if direct_control_law in {"local-bank-pitch-pd", "local_bank_pitch_pd"} and rectangle_bank is not None:
                    # A transport's source-table aerodynamic roll moment can
                    # be of the same order as the direct-wrench request.  A
                    # generic three-axis attitude cross-product controller can
                    # therefore settle with the wrong local bank sign even
                    # though its global attitude error is small.  This opt-in
                    # direct-wrench law closes the declared local bank and
                    # pitch channels explicitly, while retaining the generic
                    # controller for heading/yaw and all other vehicles.
                    local_attitude = _rigid_body_local_attitude_observables(state, attitude_earth)
                    actual_bank = math.radians(local_attitude["local_roll_deg"])
                    actual_pitch = math.radians(local_attitude["local_pitch_deg"])
                    radial = state.position.vector.scaled(1.0 / max(state.position.vector.norm(), 1.0e-12))
                    flight_path_angle = math.asin(max(-1.0, min(1.0, route_direction.dot(radial))))
                    desired_pitch = flight_path_angle + fixed_wing_alpha
                    bank_error = rectangle_bank - actual_bank
                    pitch_error = desired_pitch - actual_pitch
                    bank_gain = float(guidance_attributes.get("direct-bank-gain-nm-per-rad", str(attitude_gain)))
                    bank_rate_damping = float(
                        guidance_attributes.get("direct-bank-rate-damping-nm-s-per-rad", str(rate_damping))
                    )
                    pitch_gain = float(guidance_attributes.get("direct-pitch-gain-nm-per-rad", str(attitude_gain)))
                    pitch_rate_damping = float(
                        guidance_attributes.get("direct-pitch-rate-damping-nm-s-per-rad", str(rate_damping))
                    )
                    alpha_error = 0.0
                    alpha_gain = float(guidance_attributes.get("direct-alpha-gain-nm-per-rad", "0.0"))
                    if alpha_gain != 0.0 and aerodynamic_model is not None:
                        air_velocity_body = aerodynamic_model.air_velocity_body(state)
                        actual_alpha = math.atan2(
                            air_velocity_body.z,
                            max(abs(air_velocity_body.x), 1.0e-12),
                        )
                        alpha_error = fixed_wing_alpha - actual_alpha
                    beta_error = 0.0
                    beta_gain = float(guidance_attributes.get("direct-beta-gain-nm-per-rad", "0.0"))
                    beta_rate_damping = float(
                        guidance_attributes.get("direct-beta-rate-damping-nm-s-per-rad", "0.0")
                    )
                    if beta_gain != 0.0 and aerodynamic_model is not None:
                        air_velocity_body = aerodynamic_model.air_velocity_body(state)
                        actual_beta = math.atan2(
                            air_velocity_body.y,
                            max(math.hypot(air_velocity_body.x, air_velocity_body.z), 1.0e-12),
                        )
                        beta_error = fixed_wing_beta - actual_beta
                    heading_error = 0.0
                    heading_gain = float(guidance_attributes.get("direct-heading-gain-nm-per-rad", "0.0"))
                    heading_rate_damping = float(
                        guidance_attributes.get("direct-heading-rate-damping-nm-s-per-rad", "0.0")
                    )
                    if heading_gain != 0.0 and route_velocity is not None:
                        heading_error, _ = _runtime_rectangle_turn_errors(
                            route_attributes,
                            state,
                            route_velocity,
                        )
                    yaw_controller = bounded_attitude_moment(
                        attitude_error,
                        state.body_rate,
                        attitude_gain=attitude_gain,
                        rate_damping=rate_damping,
                        maximum_moment=maximum_moment,
                        maximum_body_rate=maximum_body_rate,
                    )
                    yaw_moment = yaw_controller.moment_body.z
                    if heading_gain != 0.0:
                        yaw_moment = heading_gain * heading_error - heading_rate_damping * state.body_rate.z
                    if beta_gain != 0.0:
                        yaw_moment += beta_gain * beta_error - beta_rate_damping * state.body_rate.z
                    requested_moment = Vector3(
                        bank_gain * bank_error - bank_rate_damping * state.body_rate.x,
                        pitch_gain * pitch_error
                        + alpha_gain * alpha_error
                        - pitch_rate_damping * state.body_rate.y,
                        yaw_moment,
                    )
                    thrust_moment = (
                        limit_vector_norm(requested_moment, maximum_moment)
                        if maximum_moment is not None
                        else requested_moment
                    )
                    controller_saturated["value"] = yaw_controller.saturated or thrust_moment != requested_moment
                    control_values["direct_local_bank_error_deg"] = math.degrees(bank_error)
                    control_values["direct_local_pitch_error_deg"] = math.degrees(pitch_error)
                    control_values["direct_local_bank_command_deg"] = math.degrees(rectangle_bank)
                    control_values["direct_local_pitch_command_deg"] = math.degrees(desired_pitch)
                    control_values["direct_local_alpha_error_deg"] = math.degrees(alpha_error)
                    control_values["direct_local_beta_error_deg"] = math.degrees(beta_error)
                    control_values["direct_local_heading_error_deg"] = math.degrees(heading_error)
                else:
                    controller = bounded_attitude_moment(
                        attitude_error,
                        state.body_rate,
                        attitude_gain=attitude_gain,
                        rate_damping=rate_damping,
                        maximum_moment=maximum_moment,
                        maximum_body_rate=maximum_body_rate,
                    )
                    thrust_moment = controller.moment_body
                    controller_saturated["value"] = controller.saturated
                control_values["direct_moment_active"] = 1.0
                for axis, value in zip(("x", "y", "z"), (thrust_moment.x, thrust_moment.y, thrust_moment.z), strict=True):
                    control_values[f"direct_moment_controller_request_{axis}_nm"] = value
                    control_values[f"direct_moment_commanded_{axis}_nm"] = value
            else:
                current_inertia = inertia_provider(state)
                lqr_command = attitude_lqr.command({
                    "attitude-error-x": -attitude_error.x,
                    "attitude-error-y": -attitude_error.y,
                    "attitude-error-z": -attitude_error.z,
                    "wx": state.body_rate.x,
                    "wy": state.body_rate.y,
                    "wz": state.body_rate.z,
                }, mass_kg=state.mass, inertia=(current_inertia.x, current_inertia.y, current_inertia.z))
                if maximum_body_rate is not None and state.body_rate.norm() > maximum_body_rate:
                    thrust_moment = state.body_rate.scaled(-rate_damping)
                    if maximum_moment is not None and thrust_moment.norm() > maximum_moment:
                        thrust_moment = thrust_moment.scaled(maximum_moment / thrust_moment.norm())
                    controller_saturated["value"] = True
                else:
                    thrust_moment = Vector3(
                        lqr_command.controls["moment-x"],
                        lqr_command.controls["moment-y"],
                        lqr_command.controls["moment-z"],
                    )
                control_values["direct_moment_active"] = 1.0
                for axis, name in zip(("x", "y", "z"), ("moment-x", "moment-y", "moment-z"), strict=True):
                    control_values[f"direct_moment_controller_request_{axis}_nm"] = float(lqr_command.unsaturated[name])
                    control_values[f"direct_moment_commanded_{axis}_nm"] = float(getattr(thrust_moment, axis))
                controller_saturated["value"] = bool(lqr_command.saturated)
            if thrust_vector.norm() > 0.0:
                # Route guidance commands the attitude; propulsion remains a
                # body-X load. This keeps thrust-vector steering inside the
                # integrated attitude/moment path rather than injecting ECIC
                # acceleration directly.
                thrust_vector = Vector3(thrust_vector.norm(), 0.0, 0.0)
        if surface_authority in {"surfaces", "control-surfaces", "aero-surfaces"} and route_velocity is not None and not surface_inversion:
            roll_gain = float(guidance_attributes.get("surface-roll-gain-deg-per-rad", "0.0"))
            pitch_gain = float(guidance_attributes.get("surface-pitch-gain-deg-per-rad", "0.0"))
            if "differential-elevon-deg" in control_values:
                base = float(
                    control_values.setdefault(
                        "_surface-base-differential-elevon-deg",
                        control_values["differential-elevon-deg"],
                    )
                )
                lower = float(guidance_attributes.get("differential-elevon-min-deg", "-20.0"))
                upper = float(guidance_attributes.get("differential-elevon-max-deg", "20.0"))
                control_values["differential-elevon-deg"] = min(upper, max(lower, base + roll_gain * route_attitude_error.x))
                control_values["differential_elevon"] = math.radians(control_values["differential-elevon-deg"])
            if "collective-elevon-deg" in control_values:
                base = float(
                    control_values.setdefault(
                        "_surface-base-collective-elevon-deg",
                        control_values["collective-elevon-deg"],
                    )
                )
                lower = float(guidance_attributes.get("collective-elevon-min-deg", "-20.0"))
                upper = float(guidance_attributes.get("collective-elevon-max-deg", "20.0"))
                control_values["collective-elevon-deg"] = min(upper, max(lower, base + pitch_gain * route_attitude_error.y))
                control_values["collective_elevon"] = math.radians(control_values["collective-elevon-deg"])
            if "symmetric-stabilator-deg" in control_values:
                base = float(
                    control_values.setdefault(
                        "_surface-base-symmetric-stabilator-deg",
                        control_values["symmetric-stabilator-deg"],
                    )
                )
                lower = float(guidance_attributes.get("symmetric-stabilator-min-deg", "-14.9"))
                upper = float(guidance_attributes.get("symmetric-stabilator-max-deg", "34.9"))
                control_values["symmetric-stabilator-deg"] = min(
                    upper,
                    max(lower, base + pitch_gain * route_attitude_error.y),
                )
                control_values["symmetric_stabilator"] = math.radians(control_values["symmetric-stabilator-deg"])
            if "differential-stabilator-deg" in control_values:
                base = float(
                    control_values.setdefault(
                        "_surface-base-differential-stabilator-deg",
                        control_values["differential-stabilator-deg"],
                    )
                )
                lower = float(guidance_attributes.get("differential-stabilator-min-deg", "-20.05"))
                upper = float(guidance_attributes.get("differential-stabilator-max-deg", "20.05"))
                control_values["differential-stabilator-deg"] = min(
                    upper,
                    max(lower, base + roll_gain * route_attitude_error.x),
                )
                control_values["differential_stabilator"] = math.radians(control_values["differential-stabilator-deg"])
            if "rudder-deg" in control_values:
                base = float(control_values.setdefault("_surface-base-rudder-deg", control_values["rudder-deg"]))
                lower = float(guidance_attributes.get("rudder-min-deg", "-29.79"))
                upper = float(guidance_attributes.get("rudder-max-deg", "29.79"))
                sideslip_feedback = 0.0
                if aerodynamic_model is not None:
                    air_velocity_body = aerodynamic_model.air_velocity_body(state)
                    sideslip_feedback = math.atan2(
                        air_velocity_body.y,
                        max(math.hypot(air_velocity_body.x, air_velocity_body.z), 1.0e-12),
                    )
                sideslip_gain = float(guidance_attributes.get("sideslip-surface-gain-deg-per-rad", "0.0"))
                control_values["rudder-deg"] = min(
                    upper,
                    max(
                        lower,
                        base
                        + float(guidance_attributes.get("surface-yaw-gain-deg-per-rad", "0.0")) * route_attitude_error.z
                        - sideslip_gain * sideslip_feedback,
                    ),
                )
                control_values["rudder"] = math.radians(control_values["rudder-deg"])
        if (
            surface_authority in {"surfaces", "control-surfaces", "aero-surfaces"}
            and surface_inversion
            and route_velocity is not None
            and isinstance(aerodynamic_model, TableAerodynamicModel)
        ):
            fixed_wing_surface_law = guidance_attributes.get(
                "fixed-wing-surface-control-law",
                "",
            ).casefold()
            if fixed_wing_surface_law in {"bank-pitch-pd", "bank_pitch_pd"}:
                radial = state.position.vector.scaled(1.0 / max(state.position.vector.norm(), 1.0e-12))
                flight_path_angle = math.asin(
                    max(-1.0, min(1.0, route_velocity.dot(radial) / max(route_velocity.norm(), 1.0e-12)))
                )
                desired_pitch = flight_path_angle + fixed_wing_alpha
                attitude_observables = _rigid_body_local_attitude_observables(
                    state,
                    attitude_earth,
                )
                desired_bank = _runtime_rectangle_bank_angle(route_attributes, state) or 0.0
                bank_error = desired_bank - math.radians(float(attitude_observables["local_roll_deg"]))
                pitch_error = desired_pitch - math.radians(float(attitude_observables["local_pitch_deg"]))
                bank_gain = float(guidance_attributes.get("surface-bank-gain-nm-per-rad", "0.25"))
                bank_damping = float(guidance_attributes.get("surface-bank-rate-damping-nm-s-per-rad", "0.08"))
                pitch_gain = float(guidance_attributes.get("surface-pitch-gain-nm-per-rad", "0.25"))
                pitch_damping = float(guidance_attributes.get("surface-pitch-rate-damping-nm-s-per-rad", "0.08"))
                inversion_target = Vector3(
                    bank_gain * bank_error - bank_damping * state.body_rate.x,
                    pitch_gain * pitch_error - pitch_damping * state.body_rate.y,
                    0.0,
                )
                control_values["fixed_wing_surface_bank_error_deg"] = math.degrees(bank_error)
                control_values["fixed_wing_surface_pitch_error_deg"] = math.degrees(pitch_error)
            elif attitude_lqr is not None:
                current_inertia = inertia_provider(state)
                lqr_command = attitude_lqr.command(
                    {
                    "attitude-error-x": -route_attitude_error.x,
                    "attitude-error-y": -route_attitude_error.y,
                    "attitude-error-z": -route_attitude_error.z,
                        "wx": state.body_rate.x,
                        "wy": state.body_rate.y,
                        "wz": state.body_rate.z,
                    },
                    mass_kg=state.mass,
                    inertia=(current_inertia.x, current_inertia.y, current_inertia.z),
                )
                inversion_target = Vector3(
                    lqr_command.controls["moment-x"],
                    lqr_command.controls["moment-y"],
                    lqr_command.controls["moment-z"],
                )
                controller_saturated["value"] = controller_saturated["value"] or bool(lqr_command.saturated)
            else:
                inversion_controller = bounded_attitude_moment(
                    route_attitude_error,
                    state.body_rate,
                    attitude_gain=float(guidance_attributes.get("attitude-gain", "10000.0")),
                    rate_damping=float(guidance_attributes.get("rate-damping", "2000.0")),
                    maximum_moment=maximum_moment,
                    maximum_body_rate=maximum_body_rate,
                )
                inversion_target = inversion_controller.moment_body
                controller_saturated["value"] = controller_saturated["value"] or inversion_controller.saturated
            if coordinated_turn_enabled:
                heading_error, bank_error = _runtime_rectangle_turn_errors(
                    route_attributes,
                    state,
                    route_velocity,
                )
                turn_command = coordinated_turn_controller.command(
                    heading_error,
                    bank_error,
                    state.body_rate,
                )
                # A surface-authority realization must allocate the turn
                # demand through the declared effectors.  It must not append
                # this command later as an unobservable direct moment.
                inversion_target = inversion_target + turn_command.moment_body
                controller_saturated["value"] = controller_saturated["value"] or turn_command.saturated
            sideslip_gain = float(guidance_attributes.get("sideslip-gain", "0.0"))
            sideslip_rate_damping = float(guidance_attributes.get("sideslip-rate-damping", "0.0"))
            if abs(sideslip_gain) > 0.0:
                air_velocity_body = aerodynamic_model.air_velocity_body(state)
                sideslip_angle = math.atan2(
                    air_velocity_body.y,
                    max(math.hypot(air_velocity_body.x, air_velocity_body.z), 1.0e-12),
                )
                inversion_target = inversion_target + Vector3(
                    0.0,
                    0.0,
                    -sideslip_gain * sideslip_angle - sideslip_rate_damping * state.body_rate.z,
                )
            controller_saturated["value"] = controller_saturated["value"] or _apply_surface_control_inversion(
                aerodynamic_model,
                state,
                control_values,
                inversion_target,
                guidance_attributes,
            )
        rudder_hold_gain = float(guidance_attributes.get("rudder-hold-gain-deg-per-deg", "0.0"))
        if rudder_hold_gain != 0.0 and "rudder-deg" in control_values and aerodynamic_model is not None:
            # A rudder beta hold is useful before route guidance is active:
            # the vehicle must first establish a source-backed trimmed plant.
            # This is deliberately separate from route yaw feedback so a
            # trim segment can use the same surface contract without inventing
            # a route or target.
            air_velocity_body = aerodynamic_model.air_velocity_body(state)
            sideslip_feedback = math.atan2(
                air_velocity_body.y,
                max(math.hypot(air_velocity_body.x, air_velocity_body.z), 1.0e-12),
            )
            target_sideslip_deg = float(guidance_attributes.get("rudder-hold-target-deg", "0.0"))
            base = float(control_values.setdefault("_surface-base-rudder-hold-deg", control_values["rudder-deg"]))
            lower = float(guidance_attributes.get("rudder-min-deg", "-29.79"))
            upper = float(guidance_attributes.get("rudder-max-deg", "29.79"))
            commanded = base + rudder_hold_gain * (target_sideslip_deg - math.degrees(sideslip_feedback))
            control_values["rudder-deg"] = min(upper, max(lower, commanded))
            control_values["rudder"] = math.radians(control_values["rudder-deg"])
            control_values["rudder_hold_controller_saturated"] = float(commanded < lower or commanded > upper)
        sideslip_gain = float(guidance_attributes.get("sideslip-gain", "0.0"))
        if surface_authority not in {"surfaces", "control-surfaces", "aero-surfaces"} and abs(sideslip_gain) > 0.0 and aerodynamic_model is not None and any(isinstance(block, AeroBlock) for block in current_segment.blocks):
            air_velocity_body = aerodynamic_model.air_velocity_body(state)
            sideslip_angle = math.atan2(air_velocity_body.y, max(math.hypot(air_velocity_body.x, air_velocity_body.z), 1.0e-12))
            sideslip_rate_damping = float(guidance_attributes.get("sideslip-rate-damping", "0.0"))
            # Positive gain is a restoring yaw moment for the canonical
            # +Y-right body convention.  Accepting a signed value keeps the
            # extension explicit while making a negative gain unambiguously
            # available for source conventions that define opposite beta.
            sideslip_moment = -sideslip_gain * sideslip_angle - sideslip_rate_damping * state.body_rate.z
            thrust_moment = thrust_moment + Vector3(0.0, 0.0, sideslip_moment)
        body_rate_damping = float(guidance_attributes.get("body-rate-damping-nm-s-per-rad", "0.0"))
        if body_rate_damping > 0.0 and surface_authority not in {"surfaces", "control-surfaces", "aero-surfaces"}:
            # Generalized damping is a direct-moment convenience only for
            # direct-authority realizations.  Surface-authority vehicles must
            # place any declared roll/pitch damping in their surface demand so
            # the aero tables, not propulsion, produce the actual moment.
            thrust_moment = thrust_moment + state.body_rate.scaled(-body_rate_damping)
        if (
            route_attributes.get("mode", "").casefold() in {"rectangle", "racetrack", "figure-eight", "figure8"}
            and coordinated_turn_enabled
            and attitude_lqr is None
            and surface_authority not in {"surfaces", "control-surfaces", "aero-surfaces"}
        ):
            heading_error, bank_error = _runtime_rectangle_turn_errors(route_attributes, state, route_velocity or Vector3(1.0, 0.0, 0.0))
            turn_command = coordinated_turn_controller.command(heading_error, bank_error, state.body_rate)
            thrust_moment = thrust_moment + turn_command.moment_body
            controller_saturated["value"] = controller_saturated["value"] or turn_command.saturated
        if control_values.get("direct_moment_active", 0.0) > 0.5:
            # A direct-wrench realization may either add its requested body
            # moment to the source aerodynamic moment, or explicitly request
            # the *total* moment after cancelling that source contribution.
            # The latter is useful for source-table plants whose uncontrolled
            # dimensional moments are larger than the reduced controller
            # request.  It remains an abstract direct-wrench model: no
            # physical surface or control allocation is implied.
            direct_moment_commanded = thrust_moment
            source_moment = Vector3(0.0, 0.0, 0.0)
            cancel_source_moment = guidance_attributes.get(
                "direct-wrench-cancel-source-moment",
                "false",
            ).casefold() in {"1", "true", "yes"}
            cancel_axes = set(
                guidance_attributes.get(
                    "direct-wrench-cancel-source-axes",
                    "xyz" if cancel_source_moment else "",
                ).casefold()
            )
            cancel_source_side_force = guidance_attributes.get(
                "direct-wrench-cancel-source-side-force",
                "false",
            ).casefold() in {"1", "true", "yes"}
            if (cancel_source_moment or cancel_source_side_force) and aerodynamic_model is not None:
                source_aero = aerodynamic_model.evaluate(state)
                source_moment = source_aero.moment_body_nm
                cancelled_source_moment = Vector3(
                    source_moment.x if "x" in cancel_axes else 0.0,
                    source_moment.y if "y" in cancel_axes else 0.0,
                    source_moment.z if "z" in cancel_axes else 0.0,
                )
                thrust_moment = thrust_moment - cancelled_source_moment
                if cancel_source_moment:
                    control_values["direct_moment_source_cancellation_active"] = 1.0
                if cancel_source_side_force:
                    beta_force_gain = float(
                        guidance_attributes.get("direct-beta-force-gain-n-per-rad", "0.0")
                    )
                    beta_force_damping = float(
                        guidance_attributes.get("direct-beta-force-damping-n-s-per-m", "0.0")
                    )
                    beta_error = 0.0
                    beta_force_correction = 0.0
                    beta_force_damping_term = 0.0
                    if beta_force_gain != 0.0:
                        beta_error = _runtime_fixed_wing_beta_command_rad(guidance_attributes) - source_aero.sideslip_rad
                        lateral_speed_m_s = aerodynamic_model.air_velocity_body(state).y
                        beta_force_damping_term = -beta_force_damping * lateral_speed_m_s
                        beta_force_correction = beta_force_gain * beta_error + beta_force_damping_term
                    thrust_vector = Vector3(
                        thrust_vector.x,
                        thrust_vector.y - source_aero.force_body_n.y + beta_force_correction,
                        thrust_vector.z,
                    )
                    control_values["direct_force_source_side_cancellation_active"] = 1.0
                    control_values["direct_force_source_side_n"] = source_aero.force_body_n.y
                    control_values["direct_force_beta_error_deg"] = math.degrees(beta_error)
                    control_values["direct_force_beta_correction_n"] = beta_force_correction
                    control_values["direct_force_beta_damping_n"] = beta_force_damping_term
            control_values["direct_moment_source_moment_x_nm"] = source_moment.x
            control_values["direct_moment_source_moment_y_nm"] = source_moment.y
            control_values["direct_moment_source_moment_z_nm"] = source_moment.z
            for axis, value in zip(
                ("x", "y", "z"),
                (direct_moment_commanded.x, direct_moment_commanded.y, direct_moment_commanded.z),
                strict=True,
            ):
                control_values[f"direct_moment_commanded_{axis}_nm"] = value
            for axis, value in zip(("x", "y", "z"), (thrust_moment.x, thrust_moment.y, thrust_moment.z), strict=True):
                control_values[f"direct_moment_applied_{axis}_nm"] = value
        propulsion = RigidBodyForceMoment(
            thrust_vector,
            thrust_moment,
            mass_rate,
            0.0,
            propulsion_force_body=thrust_vector,
            propulsion_moment_body=thrust_moment,
        )
        if aerodynamic_model is None or not any(isinstance(block, AeroBlock) for block in current_segment.blocks):
            return propulsion
        aero = aero_sample if aero_sample is not None else aerodynamic_model.evaluate(state)
        heat_rate_coefficient = max(0.0, float(thermal_attributes.get("heat-rate-coefficient", "0.002")))
        return RigidBodyForceMoment(
            propulsion.force_body + aero.force_body_n,
            propulsion.moment_body
            + Vector3(
                control_values.get("_rotor_guidance_moment_x", 0.0),
                control_values.get("_rotor_guidance_moment_y", 0.0),
                control_values.get("_rotor_guidance_moment_z", 0.0),
            )
            + aero.moment_body_nm,
            propulsion.propellant_mass_rate,
            aero.dynamic_pressure_pa * aero.airspeed_m_s * heat_rate_coefficient,
            aero_force_body=aero.force_body_n,
            propulsion_force_body=propulsion.force_body,
            aero_moment_body=aero.moment_body_nm,
            propulsion_moment_body=propulsion.moment_body
            + Vector3(
                control_values.get("_rotor_guidance_moment_x", 0.0),
                control_values.get("_rotor_guidance_moment_y", 0.0),
                control_values.get("_rotor_guidance_moment_z", 0.0),
            ),
        )
    ####

    def gravity(state: RigidBody6DofState) -> Vector3:
        radius = state.position.vector.norm()
        return state.position.vector.scaled(-earth_mu / max(radius**3, 1.0))
    ####

    model = RigidBody6DofModel(
        inertia,
        force_moment,
        gravity,
        dry_mass=float(vehicle_attributes.get("dry-mass-kg", str(named["mass"] - named.get("propellant_mass", 0.0)))),
        inertia_provider=inertia_provider,
    )
    vehicle = rigid_body_vehicle(str(trajectory.number), initial_state, model, step_size=step, integrator="rk4")
    # A release directive applies to the initial release state as well as to
    # later segment transitions.  Without this initial application, a
    # single-segment payload-release problem silently ignores the declared
    # airflow alignment and can enter a source table outside its sideslip
    # envelope before any event handler has a chance to run.
    if vehicle_attributes.get("release-attitude", "").casefold() == "airflow" and aerodynamic_model is not None:
        vehicle.state = _align_release_attitude_to_airflow(vehicle.state, aerodynamic_model)
        vehicle.history[0] = vehicle.state
    vehicle.control_values = control_values
    base_observables = vehicle.environment_evaluator

    def observables(values: Mapping[str, float]) -> dict[str, float]:
        # RK4 evaluates the derivative at four stage states.  The rigid-body
        # derivative consumes the force/moment model directly; it does not
        # need the full telemetry projection below.  Avoid evaluating every
        # coefficient table a second time at those internal stages.  Accepted
        # states still receive the complete observable set for evidence.
        if values.get("_runtime_derivative_stage", 0.0) >= 0.5:
            result = dict(base_observables(values)) if base_observables is not None else {}
            result["_segment"] = float(active_segment["number"])
            return result
        result = dict(base_observables(values)) if base_observables is not None else {}
        state = RigidBody6DofState.from_values(float(values.get("time", start_time)), tuple(values[name] for name in RIGID_BODY_STATE_NAMES))
        result.update(_rigid_body_guidance_observables(state, target_attributes, guidance_attributes, earth_mu, _earth_parameters(problem, parameters)[1]))
        active_segment_has_aero = any(isinstance(block, AeroBlock) for block in segments[active_segment["number"]].blocks)
        result.update(
            _rigid_body_aero_observables(
                state,
                aerodynamic_model if active_segment_has_aero else None,
                minimum_air_data_speed_m_s=minimum_air_data_speed_m_s,
            )
        )
        aero_force_body = Vector3(
            float(result.get("aero_force_body_x_n", 0.0)),
            float(result.get("aero_force_body_y_n", 0.0)),
            float(result.get("aero_force_body_z_n", 0.0)),
        )
        aero_acceleration_ecic = state.attitude.rotate(aero_force_body).scaled(1.0 / max(state.mass, 1.0))
        command_ecic = Vector3(
            float(result.get("pro_nav_command_ecfc_x_m_s2", 0.0)),
            float(result.get("pro_nav_command_ecfc_y_m_s2", 0.0)),
            float(result.get("pro_nav_command_ecfc_z_m_s2", 0.0)),
        )
        result["pro_nav_achieved_aero_acceleration_m_s2"] = aero_acceleration_ecic.norm()
        result["pro_nav_acceleration_response_residual_m_s2"] = (command_ecic - aero_acceleration_ecic).norm()
        result.update(_rigid_body_position_observables(state, target_attributes, route_attributes, earth_mu, earth_omega))
        # Historical print/file names remain part of the semantic runtime
        # contract even when the canonical rigid-body state is Cartesian.
        radius = max(state.position.vector.norm(), 1.0)
        result.setdefault("alt", radius - 6_378_137.0)
        result.setdefault("altitude_m", radius - 6_378_137.0)
        result.setdefault("vel", state.velocity.vector.norm())
        result.setdefault("speed_m_s", state.velocity.vector.norm())
        result.setdefault("wt", state.mass)
        result.setdefault("range", result.get("range_to_target_m", 0.0))
        result.setdefault("latgd", result.get("latitude_deg", 0.0))
        result.setdefault("long", result.get("longitude_deg", 0.0))
        result.setdefault("gamgd", 0.0)
        result.setdefault("psigd", 0.0)
        result.setdefault("yawgd", result.get("local_heading_deg", 0.0))
        result.setdefault("pitchgd", result.get("local_pitch_deg", 0.0))
        result.setdefault("rollgd", result.get("local_roll_deg", 0.0))
        result.setdefault("dynprs", result.get("aero_dynamic_pressure_pa", 0.0))
        result.setdefault("p", state.body_rate.x)
        result.setdefault("q", state.body_rate.y)
        result.setdefault("r", state.body_rate.z)
        result.setdefault("alpha", result.get("aero_alpha_deg", 0.0))
        result.setdefault("beta", result.get("aero_sideslip_deg", 0.0))
        result.setdefault("plength", 0.0)
        result.setdefault("q_rail", 0.0)
        result.setdefault("azm", 0.0)
        result.setdefault("elev", 0.0)
        result.setdefault("winde", 0.0)
        result.setdefault("windn", 0.0)
        result.setdefault("windd", 0.0)
        result.setdefault("relrng[2]", 0.0)
        result.setdefault("relvel[2]", 0.0)
        result.update(_rigid_body_local_attitude_observables(state, attitude_earth))
        if route_attributes.get("mode", "").casefold() in {"rectangle", "racetrack", "figure-eight", "figure8"}:
            result["route_alpha_command_deg"] = math.degrees(
                _runtime_fixed_wing_alpha_command_rad(guidance_attributes)
            )
            result["route_beta_command_deg"] = math.degrees(
                _runtime_fixed_wing_beta_command_rad(guidance_attributes)
            )
            result["route_beta_tracking_error_deg"] = result["route_beta_command_deg"] - result.get(
                "aero_sideslip_deg",
                0.0,
            )
        rectangle_bank = _runtime_rectangle_bank_angle(route_attributes, state)
        if rectangle_bank is not None:
            result["route_bank_command_deg"] = math.degrees(rectangle_bank)
            # This compatibility channel reports the achieved local body roll
            # used by the route controller, not an independent aerodynamic
            # bank measurement.
            result["route_bank_achieved_deg"] = result["local_roll_deg"]
            result["route_bank_tracking_error_deg"] = result["route_bank_command_deg"] - result["route_bank_achieved_deg"]
        if route_attributes.get("mode", "").casefold() in {"figure-eight", "figure8"}:
            duration = max(float(route_attributes.get("duration-s", "1.0")), 1.0)
            theta = 2.0 * math.pi * max(0.0, min(duration, state.time)) / duration
            route_pitch = _runtime_figure_eight_pitch_angle(route_attributes, theta)
            result["route_flight_path_command_deg"] = math.degrees(route_pitch)
            result["route_pitch_command_deg"] = math.degrees(route_pitch) + math.degrees(
                _runtime_fixed_wing_alpha_command_rad(guidance_attributes)
            )
            result["route_pitch_achieved_deg"] = result["local_pitch_deg"]
            result["route_pitch_tracking_error_deg"] = result["route_pitch_command_deg"] - result["route_pitch_achieved_deg"]
        elif route_attributes.get("mode", "").casefold() == "racetrack":
            local = _runtime_racetrack_local_reference(route_attributes, state.time)
            if local is not None:
                _, _, _, _, _, vertical_rate, _ = local
                speed = max(abs(float(route_attributes.get("racetrack-speed-mps", "1.0"))), 1.0e-6)
                result["route_flight_path_command_deg"] = math.degrees(math.atan2(vertical_rate, speed))
                result["route_pitch_command_deg"] = result["route_flight_path_command_deg"] + math.degrees(
                    _runtime_fixed_wing_alpha_command_rad(guidance_attributes)
                )
                result["route_pitch_achieved_deg"] = result["local_pitch_deg"]
                result["route_pitch_tracking_error_deg"] = result["route_pitch_command_deg"] - result["route_pitch_achieved_deg"]
        for name in (
            "direct_moment_active",
            "direct_moment_controller_request_x_nm",
            "direct_moment_controller_request_y_nm",
            "direct_moment_controller_request_z_nm",
            "direct_moment_commanded_x_nm",
            "direct_moment_commanded_y_nm",
            "direct_moment_commanded_z_nm",
            "direct_moment_applied_x_nm",
            "direct_moment_applied_y_nm",
            "direct_moment_applied_z_nm",
            "direct_moment_source_cancellation_active",
            "direct_moment_source_moment_x_nm",
            "direct_moment_source_moment_y_nm",
            "direct_moment_source_moment_z_nm",
            "direct_local_alpha_error_deg",
            "direct_local_beta_error_deg",
            "direct_local_heading_error_deg",
            "direct_force_source_side_cancellation_active",
            "direct_force_source_side_n",
            "direct_force_beta_error_deg",
            "direct_force_beta_correction_n",
            "direct_force_beta_damping_n",
            "rate_lqr_request_moment_x_nm",
            "rate_lqr_request_moment_y_nm",
            "rate_lqr_request_moment_z_nm",
            "rate_lqr_saturated",
            "rotor_command_saturated",
            "rotorcraft_yaw_target_deg",
            "rotorcraft_yaw_achieved_deg",
            "rotorcraft_yaw_error_deg",
            "rotorcraft_yaw_moment_nm",
            "rotorcraft_yaw_control_active",
            "surface_allocation_requested_moment_x_nm",
            "surface_allocation_requested_moment_y_nm",
            "surface_allocation_requested_moment_z_nm",
            "surface_allocation_linearized_achieved_moment_x_nm",
            "surface_allocation_linearized_achieved_moment_y_nm",
            "surface_allocation_linearized_achieved_moment_z_nm",
            "surface_allocation_actual_achieved_moment_x_nm",
            "surface_allocation_actual_achieved_moment_y_nm",
            "surface_allocation_actual_achieved_moment_z_nm",
            "surface_allocation_residual_nm",
            "surface_allocation_controlled_residual_nm",
            "surface_allocation_actual_residual_nm",
            "surface_allocation_actual_controlled_residual_nm",
            "surface_allocation_saturated",
            "surface_allocation_rate_limited",
            "surface_allocation_delta_limit_deg",
            "surface_allocation_active_surface_count",
            "surface_allocation_controlled_axis_count",
            "surface_allocation_uncontrolled_axis_count",
            "surface_allocation_effectiveness_rank",
            "surface_allocation_status_code",
            "surface_allocation_collective_elevon_commanded_deg",
            "surface_allocation_collective_elevon_achieved_deg",
            "surface_allocation_collective_elevon_rate_deg_s",
            "surface_allocation_differential_elevon_commanded_deg",
            "surface_allocation_differential_elevon_achieved_deg",
            "surface_allocation_differential_elevon_rate_deg_s",
            "surface_allocation_elevator_commanded_deg",
            "surface_allocation_elevator_achieved_deg",
            "surface_allocation_elevator_rate_deg_s",
            "surface_allocation_aileron_commanded_deg",
            "surface_allocation_aileron_achieved_deg",
            "surface_allocation_aileron_rate_deg_s",
            "fixed_wing_surface_bank_error_deg",
            "fixed_wing_surface_pitch_error_deg",
            "surface_allocation_symmetric_stabilator_commanded_deg",
            "surface_allocation_symmetric_stabilator_achieved_deg",
            "surface_allocation_symmetric_stabilator_rate_deg_s",
            "surface_allocation_differential_stabilator_commanded_deg",
            "surface_allocation_differential_stabilator_achieved_deg",
            "surface_allocation_differential_stabilator_rate_deg_s",
            "surface_allocation_rudder_commanded_deg",
            "surface_allocation_rudder_achieved_deg",
            "surface_allocation_rudder_rate_deg_s",
        ):
            if name in control_values:
                result[name] = float(control_values[name])
        for block in segments[active_segment["number"]].blocks:
            if not isinstance(block, FlyBlock) or block.guidance_variable is None:
                continue
            if block.guidance_variable.casefold() not in {"bankgc", "bankgd"}:
                continue
            commanded_bank = values.get(block.guidance_variable.casefold(), 0.0)
            if block.value is not None:
                try:
                    commanded_bank = evaluate_expression(
                        block.value,
                        values,
                        parameters,
                        tables=_table_evaluators(tables),
                    )
                except (KeyError, TypeError, ValueError):
                    pass
            result["bank_command_deg"] = float(commanded_bank)
            # Preserve the historical channel name while making its meaning
            # explicit: achieved local roll, not raw table-query bank.
            result["bank_achieved_deg"] = result["local_roll_deg"]
        # Derive source-facing channels from the same total load and aero
        # sample; do not introduce a second force evaluation path.
        total_force_body = Vector3(
            float(result.get("force_body_x_n", 0.0)),
            float(result.get("force_body_y_n", 0.0)),
            float(result.get("force_body_z_n", 0.0)),
        )
        total_moment_body = Vector3(
            float(result.get("moment_body_x_nm", 0.0)),
            float(result.get("moment_body_y_nm", 0.0)),
            float(result.get("moment_body_z_nm", 0.0)),
        )
        aero_moment_body = Vector3(
            float(result.get("aero_moment_body_x_nm", 0.0)),
            float(result.get("aero_moment_body_y_nm", 0.0)),
            float(result.get("aero_moment_body_z_nm", 0.0)),
        )
        propulsion_force = total_force_body - aero_force_body
        control_moment = total_moment_body - aero_moment_body
        if result.get("surface_allocation_active_surface_count", 0.0) > 0.0:
            requested_surface_moment = Vector3(
                float(result.get("surface_allocation_requested_moment_x_nm", 0.0)),
                float(result.get("surface_allocation_requested_moment_y_nm", 0.0)),
                float(result.get("surface_allocation_requested_moment_z_nm", 0.0)),
            )
            actual_surface_error = aero_moment_body - requested_surface_moment
            result.update(
                {
                    "surface_allocation_actual_moment_error_x_nm": actual_surface_error.x,
                    "surface_allocation_actual_moment_error_y_nm": actual_surface_error.y,
                    "surface_allocation_actual_moment_error_z_nm": actual_surface_error.z,
                    "surface_allocation_actual_residual_nm": actual_surface_error.norm(),
                }
            )
        if result.get("direct_moment_active", 0.0) > 0.5:
            direct_aero = aero_moment_body
            direct_propulsion = Vector3(
                float(result.get("propulsion_moment_body_x_nm", control_moment.x)),
                float(result.get("propulsion_moment_body_y_nm", control_moment.y)),
                float(result.get("propulsion_moment_body_z_nm", control_moment.z)),
            )
            result.update(
                {
                    "direct_moment_aero_x_nm": direct_aero.x,
                    "direct_moment_aero_y_nm": direct_aero.y,
                    "direct_moment_aero_z_nm": direct_aero.z,
                    "direct_moment_propulsion_x_nm": direct_propulsion.x,
                    "direct_moment_propulsion_y_nm": direct_propulsion.y,
                    "direct_moment_propulsion_z_nm": direct_propulsion.z,
                    "direct_moment_total_x_nm": total_moment_body.x,
                    "direct_moment_total_y_nm": total_moment_body.y,
                    "direct_moment_total_z_nm": total_moment_body.z,
                    "direct_moment_cancellation_residual_nm": total_moment_body.norm(),
                }
            )
        result.update(
            {
                "mass_kg": state.mass,
                "propellant_mass_kg": state.propellant_mass,
                "thrust_n": propulsion_force.norm(),
                "thrust_force_n": propulsion_force.norm(),
                "drag_force_n": max(0.0, -aero_force_body.x),
                "control_force_n": math.hypot(aero_force_body.y, aero_force_body.z),
                "total_force_n": float(result.get("total_force_ecic_n", total_force_body.norm())),
                "control_torque_nm": control_moment.norm(),
                "mach": float(result.get("aero_mach", 0.0)),
            }
        )
        result["attitude_controller_saturated"] = 1.0 if controller_saturated["value"] else 0.0
        contact_active = ground_contact_segment is not None and active_segment["number"] == ground_contact_segment
        result["ground_contact_state"] = 1.0 if contact_active else 0.0
        result["ground_reaction_n"] = state.mass * earth_mu / max(state.position.vector.norm() ** 2, 1.0) if contact_active else 0.0
        result["ground_contact_impulse_n_s"] = float(values.get("ground_contact_impulse_n_s", result.get("ground_contact_impulse_n_s", 0.0)))
        result["ground_contact_mode"] = 1.0 if contact_active else 0.0
        shutdown_time_text = actuator_attributes.get("motor-shutdown-time-s")
        result["motor_shutdown"] = 1.0 if shutdown_time_text is not None and state.time >= float(shutdown_time_text) else 0.0
        result["_segment"] = float(active_segment["number"])
        return result
    ####

    vehicle.environment_evaluator = observables
    if control_values:
        vehicle.state = RuntimeState(vehicle.state.time, vehicle.state.values, vehicle.state.frame, {**vehicle.state.named, **control_values}, vehicle.state.value_names, vehicle.state.segment_endpoints)
        vehicle.history[0] = vehicle.state
    initial_observables = {
        "alt": vehicle.state.named.get("x", 0.0) - 6_378_137.0,
        "vel": math.sqrt(sum(vehicle.state.named.get(name, 0.0) ** 2 for name in ("vx", "vy", "vz"))),
        "wt": vehicle.state.named.get("mass", 0.0),
        "p": vehicle.state.named.get("wx", 0.0),
        "q": vehicle.state.named.get("wy", 0.0),
        "r": vehicle.state.named.get("wz", 0.0),
        "plength": 0.0,
        "q_rail": 0.0,
        "azm": 0.0,
        "elev": 0.0,
        "winde": 0.0,
        "windn": 0.0,
        "windd": 0.0,
        "relrng[2]": 0.0,
        "relvel[2]": 0.0,
    }
    try:
        initial_observables.update(observables({**vehicle.state.named, **control_values}))
    except (KeyError, TypeError, ValueError):
        # Some guidance declarations are undefined exactly at the source
        # origin (for example zero target range). Keep the basic historical
        # channels available; the live evaluator will publish the full set
        # once the first accepted integration state exists.
        pass
    vehicle.state = RuntimeState(
        vehicle.state.time,
        vehicle.state.values,
        vehicle.state.frame,
        {**vehicle.state.named, **initial_observables},
        vehicle.state.value_names,
        vehicle.state.segment_endpoints,
    )
    vehicle.history[0] = vehicle.state
    segment_events: dict[int, tuple[EventCondition, ...]] = {}
    event_targets: dict[str, int | None] = {}
    runtime_events = _runtime_event_conditions(problem, parameters, tables)
    for current_segment in segments.values():
        safety_event_name = f"trajectory-{trajectory.number}-earth-intersection-{current_segment.number}"
        contact_transition = (
            ground_contact_mode not in {"none", ""}
            and ground_contact_segment is not None
            and current_segment.number != ground_contact_segment
        )
        event_action = "goto" if contact_transition else "stop"
        event_targets[safety_event_name] = ground_contact_segment if contact_transition else None
        events: list[EventCondition] = []
        if not (ground_contact_segment is not None and current_segment.number == ground_contact_segment):
            events.append(
                EventCondition(
                    safety_event_name,
                    lambda state: _rigid_body_altitude_residual(state),
                    event_action,
                    lambda state: _rigid_body_altitude_residual(state) <= 0.0,
                    signal="ground-contact" if contact_transition else "earth-intersection",
                    source="taoryx-rigid-body-ground-contact" if contact_transition else "taoryx-rigid-body-safety-guard",
                )
            )
        events.extend(runtime_events)
        if thermal_attributes.get("policy", "none").casefold() == "stop":
            thermal_limits = (
                ("heat-rate", "heat_rate_w_m2", "maximum-heat-rate-w-m2"),
                ("heat-load", "heat_load", "maximum-heat-load-j-m2"),
            )
            for label, state_name, limit_name in thermal_limits:
                limit_text = thermal_attributes.get(limit_name)
                if limit_text is None:
                    continue
                limit = float(limit_text)
                event_name = f"trajectory-{trajectory.number}-thermal-{label}-{current_segment.number}"

                def thermal_residual(state: RuntimeState, state_name: str = state_name, limit: float = limit) -> float:
                    return limit - state.named.get(state_name, 0.0)
                ####

                def thermal_predicate(state: RuntimeState, residual: Callable[[RuntimeState], float] = thermal_residual) -> bool:
                    return residual(state) <= 0.0
                ####

                events.append(
                    EventCondition(
                        event_name,
                        thermal_residual,
                        "stop",
                        thermal_predicate,
                        signal=f"thermal-{label}-limit",
                        source=f"{problem.location.path}:runtime thermal policy",
                    )
                )
                event_targets[event_name] = None
            ####
        for block in current_segment.blocks:
            if not isinstance(block, WhenBlock) or block.condition is None:
                continue
            expression = block.condition
            event_name = f"trajectory-{trajectory.number}-when-{current_segment.number}-{len(events) + 1}"

            def condition_function(state: RuntimeState, expression: ExpressionType = expression) -> float:
                return _event_residual(expression, state.named, parameters, tables)
            ####

            def condition_predicate(state: RuntimeState, expression: ExpressionType = expression) -> bool:
                return _event_satisfied(expression, state.named, parameters, tables)
            ####

            events.append(
                EventCondition(
                    event_name,
                    condition_function,
                    block.action or "stop",
                    condition_predicate,
                    source=f"{block.location.path}:{block.location.line}",
                )
            )
            event_targets[event_name] = block.target_segment if block.action == "goto" else None
        segment_events[current_segment.number] = tuple(events)
    ####

    vehicle.events = segment_events[trajectory.start_segment]

    def transition_handler(state: RuntimeState, *, target: int | None = None) -> RuntimeState:
        if target is None:
            return state
        if target not in segments:
            raise ValueError(f"rigid-body event targets missing segment {target}")
        if ground_contact_segment is not None and target == ground_contact_segment:
            state = _apply_ground_contact_state(
                state,
                restitution=float(vehicle_attributes.get("ground-contact-restitution", "0.0")),
                static_friction=vehicle_attributes.get("ground-contact-friction", "static").casefold() in {"static", "clamp", "1", "true", "yes"},
                contact_step_s=max(float(vehicle_attributes.get("ground-contact-step-s", str(step))), 1.0e-6),
            )
        # Segment reset/increment blocks are the native problem-file seam for
        # staging and release events.  Apply them before switching the force
        # sources so mass and propellant discontinuities are visible to the
        # next rigid-body derivative evaluation.
        state = _apply_segment_updates(state, segments[target], parameters, tables)
        if vehicle_attributes.get("release-attitude", "").casefold() == "airflow" and aerodynamic_model is not None:
            state = _align_release_attitude_to_airflow(state, aerodynamic_model)
        active_segment["number"] = target
        vehicle.events = segment_events[target]
        vehicle.segment_number = target
        vehicle.step_size = _segment_step_size(segments[target], parameters, vehicle.step_size)
        vehicle.max_step_size = vehicle.step_size
        return state
    ####

    def make_transition_handler(target: int) -> Callable[[RuntimeState], RuntimeState]:
        def handler(state: RuntimeState) -> RuntimeState:
            return transition_handler(state, target=target)
        ####

        return handler
    ####

    vehicle.event_handlers = {
        event.name: make_transition_handler(target)
        for events in segment_events.values()
        for event in events
        if (target := event_targets[event.name]) is not None
    }
    runtime = RuntimeProblem({vehicle.name: vehicle}, sensor_clocks=_runtime_sensor_clocks(problem))
    runtime.metadata["dynamics_mode"] = DynamicsMode.RIGID_BODY_6DOF.value
    runtime.metadata["parameters"] = dict(parameters)
    runtime.metadata["vehicle"] = dict(vehicle_attributes)
    runtime.metadata["actuator"] = dict(actuator_attributes)
    runtime.metadata["target"] = dict(_runtime_attributes(problem, "target"))
    runtime.metadata["route"] = dict(route_attributes)
    runtime.metadata["thermal"] = dict(thermal_attributes)
    # Keep the historical flat attitude declaration for existing consumers;
    # the role-indexed view is the extensible contract for mixed controllers.
    runtime.metadata["lqr"] = _runtime_lqr_attributes(problem, "attitude")
    runtime.metadata["lqr_by_role"] = {
        "attitude": _runtime_lqr_attributes(problem, "attitude"),
        "rate": _runtime_lqr_attributes(problem, "rate"),
    }
    runtime.metadata["controller_realization"] = _runtime_controller_realization_metadata(attitude_lqr or rate_lqr)
    runtime.metadata["controller_realizations"] = {
        "attitude": _runtime_controller_realization_metadata(attitude_lqr),
        "rate": _runtime_controller_realization_metadata(rate_lqr),
    }
    runtime.metadata["telemetry"] = dict(_runtime_attributes(problem, "telemetry"))
    runtime.metadata["controls"] = dict(control_values)
    _register_sensor_clock_metadata(runtime)
    runtime.metadata["runtime_model_bindings"] = _runtime_model_bindings(problem)
    runtime.metadata["native_pipeline"] = {
        "integration_frame": "ecic",
        "environment_frame": "ecfc",
        "load_sources": ("gravity", "propulsion") + (("aerodynamics", "thermal") if aerodynamic_model is not None else ()),
        "guidance": "proportional-navigation" if float(guidance_attributes.get("propnav-gain", "0.0")) > 0.0 else "none",
        "route_guidance": route_attributes.get("mode", "none"),
        "terminal_guidance_mode": route_attributes.get("terminal-guidance-mode", "route"),
        "terminal_guidance_owner": "pure-propnav" if route_attributes.get("terminal-guidance-mode", "route").casefold() in {"propnav", "pure-propnav"} else "route",
        "table_binding": "explicit-segment-aero" if aerodynamic_model is not None else "none",
    }
    return RuntimeCase(index, parameters, runtime)
####


def _limit_body_direction_sideslip(direction_body: Vector3, limit_radians: float) -> Vector3:
    """Limit the lateral component of an attitude direction in body axes.

    This is an attitude-command projection, not an aerodynamic lookup clamp.
    It preserves the requested pitch-plane direction while limiting the body-Y
    component that would create sideslip during a coordinated route turn.
    """

    if not math.isfinite(limit_radians) or limit_radians >= math.pi / 2.0:
        return direction_body
    limit = max(0.0, math.sin(limit_radians))
    norm = max(direction_body.norm(), 1.0e-12)
    unit = direction_body.scaled(1.0 / norm)
    lateral = max(-limit, min(limit, unit.y))
    pitch_plane_norm = math.hypot(unit.x, unit.z)
    if pitch_plane_norm <= 1.0e-12:
        return Vector3(math.sqrt(max(0.0, 1.0 - lateral * lateral)), lateral, 0.0)
    scale = math.sqrt(max(0.0, 1.0 - lateral * lateral)) / pitch_plane_norm
    return Vector3(unit.x * scale, lateral, unit.z * scale)
####


def _limit_body_direction_pitch(direction_body: Vector3, limit_radians: float) -> Vector3:
    """Limit the vertical component of an attitude direction command."""

    if not math.isfinite(limit_radians) or limit_radians >= math.pi / 2.0:
        return direction_body
    limit = max(0.0, math.sin(limit_radians))
    unit = direction_body.scaled(1.0 / max(direction_body.norm(), 1.0e-12))
    vertical = max(-limit, min(limit, unit.z))
    horizontal_norm = math.hypot(unit.x, unit.y)
    if horizontal_norm <= 1.0e-12:
        return Vector3(math.sqrt(max(0.0, 1.0 - vertical * vertical)), 0.0, vertical)
    scale = math.sqrt(max(0.0, 1.0 - vertical * vertical)) / horizontal_norm
    return Vector3(unit.x * scale, unit.y * scale, vertical)
####


def _align_release_attitude_to_airflow(state: RuntimeState, aerodynamic_model: TableAerodynamicModel | DirectWrenchTableModel) -> RuntimeState:
    """Acquire a coordinated zero-sideslip attitude at a release boundary."""

    rigid_state = RigidBody6DofState.from_values(state.time, tuple(state.named[name] for name in RIGID_BODY_STATE_NAMES))
    air_body = aerodynamic_model.air_velocity_body(rigid_state)
    air_ecic = rigid_state.attitude.rotate(air_body)
    air_norm = air_ecic.norm()
    if air_norm <= 1.0e-12:
        return state
    body_x = air_ecic.scaled(1.0 / air_norm)
    radial = rigid_state.position.vector.scaled(1.0 / max(rigid_state.position.vector.norm(), 1.0e-12))
    body_y = radial.cross(body_x)
    if body_y.norm() <= 1.0e-12:
        body_y = rigid_state.attitude.rotate(Vector3(0.0, 1.0, 0.0))
    body_y = body_y.scaled(1.0 / max(body_y.norm(), 1.0e-12))
    cross = body_x.cross(body_y)
    body_z = cross.scaled(1.0 / max(cross.norm(), 1.0e-12))
    trace = body_x.x + body_y.y + body_z.z
    if trace > 0.0:
        scale = 0.5 / math.sqrt(trace + 1.0)
        attitude = Quaternion(0.25 / scale, (body_y.z - body_z.y) * scale, (body_z.x - body_x.z) * scale, (body_x.y - body_y.x) * scale)
    elif body_x.x > body_y.y and body_x.x > body_z.z:
        scale = 2.0 * math.sqrt(1.0 + body_x.x - body_y.y - body_z.z)
        attitude = Quaternion((body_y.z - body_z.y) / scale, 0.25 * scale, (body_y.x + body_x.y) / scale, (body_z.x + body_x.z) / scale)
    elif body_y.y > body_z.z:
        scale = 2.0 * math.sqrt(1.0 + body_y.y - body_x.x - body_z.z)
        attitude = Quaternion((body_z.x - body_x.z) / scale, (body_y.x + body_x.y) / scale, 0.25 * scale, (body_z.y + body_y.z) / scale)
    else:
        scale = 2.0 * math.sqrt(1.0 + body_z.z - body_x.x - body_y.y)
        attitude = Quaternion((body_x.y - body_y.x) / scale, (body_z.x + body_x.z) / scale, (body_z.y + body_y.z) / scale, 0.25 * scale)
    attitude = attitude.normalized()
    values = list(state.values)
    for name, value in zip(("qw", "qx", "qy", "qz"), (attitude.w, attitude.x, attitude.y, attitude.z), strict=True):
        values[state.value_names.index(name)] = value
    named = {**state.named, "qw": attitude.w, "qx": attitude.x, "qy": attitude.y, "qz": attitude.z, "release_attitude_aligned": 1.0}
    return RuntimeState(state.time, tuple(values), state.frame, named, state.value_names, state.segment_endpoints)
####


def _rigid_body_altitude_residual(state: RuntimeState) -> float:
    """Return geocentric altitude above the native Earth collision surface."""

    radius = math.sqrt(sum(state.named.get(name, 0.0) ** 2 for name in ("x", "y", "z")))
    return radius - 6_378_137.0


def _apply_ground_contact_state(
    state: RuntimeState,
    *,
    restitution: float,
    static_friction: bool,
    contact_step_s: float,
) -> RuntimeState:
    """Commit a declared rigid-body ground-contact impulse.

    This is intentionally a small pad/contact contract for showcase and
    terminal-state work.  It clamps the body to the spherical Earth surface,
    applies a bounded normal restitution, and optionally consumes tangential
    velocity as static pad friction.  The pre/post state is retained by the
    runtime transition ledger, so this is an explicit state discontinuity—not
    a hidden renderer-side reset.
    """

    radius_vector = Vector3(state.named.get("x", 0.0), state.named.get("y", 0.0), state.named.get("z", 0.0))
    radius = max(radius_vector.norm(), 1.0e-12)
    normal = radius_vector.scaled(1.0 / radius)
    position = normal.scaled(6_378_137.0)
    velocity = Vector3(state.named.get("xdt", 0.0), state.named.get("ydt", 0.0), state.named.get("zdt", 0.0))
    normal_velocity = velocity.dot(normal)
    tangential_velocity = velocity - normal.scaled(normal_velocity)
    bounded_restitution = max(0.0, min(1.0, restitution))
    post_normal_velocity = -bounded_restitution * min(normal_velocity, 0.0)
    post_tangential_velocity = Vector3(0.0, 0.0, 0.0) if static_friction else tangential_velocity
    post_velocity = normal.scaled(post_normal_velocity) + post_tangential_velocity
    mass = float(state.named.get("mass", 0.0))
    impulse = (post_velocity - velocity).scaled(mass)
    values = list(state.values)
    for name, value in zip(
        ("x", "y", "z", "vx", "vy", "vz", "wx", "wy", "wz"),
        (position.x, position.y, position.z, post_velocity.x, post_velocity.y, post_velocity.z, 0.0, 0.0, 0.0),
        strict=True,
    ):
        # The rigid-body state indices are stable and the explicit assignment
        # avoids changing the canonical state vector for a contact-only field.
        values[RIGID_BODY_STATE_NAMES.index(name)] = value
    named = {
        **state.named,
        "ground_contact_state": 1.0,
        "ground_contact_impulse_n_s": impulse.norm(),
        "ground_reaction_n": impulse.norm() / contact_step_s,
        "ground_contact_normal_velocity_pre_m_s": normal_velocity,
        "ground_contact_tangential_speed_pre_m_s": tangential_velocity.norm(),
        "ground_contact_restitution": bounded_restitution,
    }
    return RuntimeState(
        state.time,
        tuple(values),
        state.frame,
        named,
        RIGID_BODY_STATE_NAMES,
        state.segment_endpoints,
    )
####


def _runtime_point_mass_route_commands(
    route_attributes: Mapping[str, str],
    target_attributes: Mapping[str, str],
    values: Mapping[str, float],
) -> dict[str, float]:
    """Resolve the shared route declaration into geodetic 3-DOF commands.

    Point-mass states use geodetic scalars and native feet/second units,
    whereas the rigid-body route adapter uses ECIC vectors.  This boundary
    adapter preserves one problem-file route contract without pretending that
    a point-mass run contains attitude or actuator dynamics.
    """

    mode = route_attributes.get("mode", "").casefold()
    if mode not in {"great-circle", "rectangle", "racetrack"} or not {"lat", "long", "alt", "vel"}.issubset(values):
        return {}
    duration = max(float(route_attributes.get("duration-s", "1.0")), 1.0)
    current_time = float(values.get("time", 0.0))
    latitude = math.radians(float(values["lat"]))
    longitude = math.radians(float(values["long"]))
    current_altitude_m = float(values["alt"]) * 0.3048
    target_altitude_m = current_altitude_m
    if mode == "great-circle":
        required = ("start-latitude-deg", "start-longitude-deg", "duration-s")
        if any(name not in route_attributes for name in required) or not {"latitude-deg", "longitude-deg"}.issubset(target_attributes):
            return {}
        target_latitude = math.radians(float(target_attributes["latitude-deg"]))
        target_longitude = math.radians(float(target_attributes["longitude-deg"]))
        delta_east = (target_longitude - longitude) * 6_378_137.0 * math.cos(latitude)
        delta_north = (target_latitude - latitude) * 6_378_137.0
        target_altitude_m = float(target_attributes.get("altitude-m", str(current_altitude_m)))
    elif mode == "racetrack":
        local = _runtime_racetrack_local_reference(route_attributes, current_time)
        if local is None:
            return {}
        east_offset, north_offset, target_altitude_m, tangent_east, tangent_north, vertical_rate_mps, _phase = local
        altitude_rate_mps = vertical_rate_mps
        start_latitude = math.radians(float(route_attributes.get("start-latitude-deg", "0.0")))
        start_longitude = math.radians(float(route_attributes.get("start-longitude-deg", "0.0")))
        radius = 6_378_137.0 + float(route_attributes.get("racetrack-low-altitude-m", "0.0"))
        east = (longitude - start_longitude) * radius * math.cos(start_latitude)
        north = (latitude - start_latitude) * radius
        delta_east = east_offset - east
        delta_north = north_offset - north
        speed_mps = abs(float(route_attributes.get("racetrack-speed-mps", "0.0")))
        if speed_mps <= 0.0:
            return {}
        capture_gain = max(0.0, float(route_attributes.get("position-capture-gain", "0.0")))
        correction_east = delta_east * capture_gain
        correction_north = delta_north * capture_gain
        correction_limit = max(
            0.0,
            float(route_attributes.get("position-capture-max-correction-mps", str(max(speed_mps * 0.25, 1.0)))),
        )
        correction_norm = math.hypot(correction_east, correction_north)
        if correction_norm > correction_limit > 0.0:
            correction_scale = correction_limit / correction_norm
            correction_east *= correction_scale
            correction_north *= correction_scale
        commanded_east = tangent_east * speed_mps + correction_east
        commanded_north = tangent_north * speed_mps + correction_north
        commanded_horizontal_speed = max(math.hypot(commanded_east, commanded_north), 1.0e-12)
        delta_altitude = target_altitude_m - current_altitude_m
        altitude_gain = max(0.0, float(route_attributes.get("racetrack-altitude-capture-gain-per-s", "0.0")))
        altitude_limit = max(0.0, float(route_attributes.get("racetrack-altitude-capture-max-mps", "0.0")))
        altitude_correction = max(-altitude_limit, min(altitude_limit, altitude_gain * delta_altitude))
        altitude_rate_mps += altitude_correction
        return {
            "_command_vel": speed_mps / 0.3048,
            "_command_psi": math.degrees(math.atan2(commanded_east, commanded_north)),
            "_command_gamgd": math.degrees(math.atan2(altitude_rate_mps, commanded_horizontal_speed)),
        }
    else:
        required = ("rectangle-length-m", "rectangle-width-m", "duration-s")
        if any(name not in route_attributes for name in required):
            return {}
        radius = 6_378_137.0 + float(route_attributes.get("start-altitude-m", "0.0"))
        start_latitude = math.radians(float(route_attributes.get("start-latitude-deg", "0.0")))
        start_longitude = math.radians(float(route_attributes.get("start-longitude-deg", "0.0")))
        east = (longitude - start_longitude) * radius * math.cos(start_latitude)
        north = (latitude - start_latitude) * radius
        leg_duration = duration / 4.0
        phase = min(3, max(0, int(float(values.get("time", 0.0)) / max(leg_duration, 1.0e-12))))
        start_altitude = float(route_attributes.get("start-altitude-m", "0.0"))
        elevated_altitude = float(route_attributes.get("elevated-corner-altitude-m", str(start_altitude)))
        corners = (
            (float(route_attributes["rectangle-length-m"]), 0.0, start_altitude),
            (float(route_attributes["rectangle-length-m"]), float(route_attributes["rectangle-width-m"]), elevated_altitude),
            (0.0, float(route_attributes["rectangle-width-m"]), start_altitude),
            (0.0, 0.0, start_altitude),
        )
        target_east, target_north, target_altitude_m = corners[phase]
        delta_east = target_east - east
        delta_north = target_north - north
    distance = math.hypot(delta_east, delta_north)
    altitude_profile = route_attributes.get("altitude-profile", "linear-target").casefold()
    if altitude_profile == "mission":
        powered_end = float(route_attributes.get("powered-end-s", "360.0"))
        terminal_start = float(route_attributes.get("terminal-start-s", str(duration * 0.75)))
        apogee = float(route_attributes.get("apogee-altitude-m", "220000.0"))
        if current_time < powered_end:
            altitude_rate_mps = float(route_attributes.get("powered-climb-rate-mps", "0.0"))
            speed_mps = abs(float(route_attributes.get("powered-speed-mps", "0.0")))
        elif current_time < terminal_start:
            altitude_rate_mps = 0.0
            speed_mps = abs(float(route_attributes.get("glide-speed-mps", "0.0")))
            target_altitude_m = apogee
        else:
            descent_duration = max(duration - terminal_start, 1.0)
            altitude_rate_mps = -apogee / descent_duration
            speed_mps = abs(float(route_attributes.get("terminal-speed-mps", "0.0")))
            target_altitude_m = max(0.0, apogee + altitude_rate_mps * (current_time - terminal_start))
    else:
        altitude_rate_mps = (target_altitude_m - current_altitude_m) / duration
        speed_key = "rectangle-speed-mps" if mode == "rectangle" else "powered-speed-mps"
        speed_mps = abs(float(route_attributes.get(speed_key, "0.0")))
    if speed_mps <= 0.0:
        speed_mps = distance / duration
    return {
        "_command_vel": speed_mps / 0.3048,
        "_command_psi": math.degrees(math.atan2(delta_east, delta_north)),
        "_command_gamgd": math.degrees(math.atan2(altitude_rate_mps, max(speed_mps, 1.0e-6))),
    }
    ####


def _runtime_route_velocity(
    route_attributes: Mapping[str, str],
    target_attributes: Mapping[str, str],
    state: RigidBody6DofState,
    earth_omega: float,
) -> Vector3 | None:
    """Resolve a declared great-circle route into an ECIC velocity direction.

    This is a native extension guidance reference, not a force injection. It
    supplies a desired velocity direction to the thrust-vector/attitude
    controller; the rigid-body model still integrates the resulting body
    thrust, moments, gravity, mass flow, and any active aerodynamic loads.
    """

    if route_attributes.get("mode", "").casefold() in {"rectangle", "racetrack", "figure-eight", "figure8"}:
        return _runtime_rectangle_route_velocity(route_attributes, state, earth_omega)
    if route_attributes.get("mode", "").casefold() != "great-circle":
        return None
    required = ("start-latitude-deg", "start-longitude-deg", "duration-s")
    if any(name not in route_attributes for name in required) or "latitude-deg" not in target_attributes or "longitude-deg" not in target_attributes:
        return None
    start_lat = math.radians(float(route_attributes["start-latitude-deg"]))
    start_lon = math.radians(float(route_attributes["start-longitude-deg"]))
    target_lat = math.radians(float(target_attributes["latitude-deg"]))
    target_lon = math.radians(float(target_attributes["longitude-deg"]))
    start = Vector3(math.cos(start_lat) * math.cos(start_lon), math.cos(start_lat) * math.sin(start_lon), math.sin(start_lat))
    target = Vector3(math.cos(target_lat) * math.cos(target_lon), math.cos(target_lat) * math.sin(target_lon), math.sin(target_lat))
    cosine = max(-1.0, min(1.0, start.dot(target)))
    route_angle = math.acos(cosine)
    duration = max(float(route_attributes["duration-s"]), 1.0)
    fraction = max(0.0, min(1.0, state.time / duration))
    if route_angle <= 1.0e-12:
        route_unit = start
    else:
        sine = math.sin(route_angle)
        route_unit = (start.scaled(math.sin((1.0 - fraction) * route_angle) / sine) + target.scaled(math.sin(fraction * route_angle) / sine))
        route_unit = route_unit.scaled(1.0 / max(route_unit.norm(), 1.0e-12))
    tangent = target - route_unit.scaled(route_unit.dot(target))
    tangent = tangent.scaled(1.0 / max(tangent.norm(), 1.0e-12))
    apogee = float(route_attributes.get("apogee-altitude-m", "220000.0"))
    terminal_start = float(route_attributes.get("terminal-start-s", "1650.0"))
    terminal_descent_end = max(float(route_attributes.get("terminal-descent-end-s", str(duration))), terminal_start + 1.0)
    powered_end = float(route_attributes.get("powered-end-s", "360.0"))
    altitude_profile = route_attributes.get("altitude-profile", "mission").casefold()
    if altitude_profile == "linear-target":
        start_altitude = float(route_attributes.get("start-altitude-m", "0.0"))
        target_altitude = float(target_attributes.get("altitude-m", str(start_altitude)))
        fraction = max(0.0, min(1.0, state.time / duration))
        desired_altitude = start_altitude + (target_altitude - start_altitude) * fraction
        altitude_rate = (target_altitude - start_altitude) / duration
        speed = float(route_attributes.get("powered-speed-mps", "3500.0"))
    elif state.time < powered_end:
        altitude_rate = float(route_attributes.get("powered-climb-rate-mps", str(apogee / max(powered_end, 1.0))))
        speed = float(route_attributes.get("powered-speed-mps", "3500.0"))
        desired_altitude = max(0.0, apogee * state.time / max(powered_end, 1.0))
    elif state.time < terminal_start:
        altitude_rate = 0.0
        speed = float(route_attributes.get("glide-speed-mps", "6000.0"))
        desired_altitude = apogee
    else:
        altitude_rate = -apogee / max(terminal_descent_end - terminal_start, 1.0)
        speed = float(route_attributes.get("terminal-speed-mps", "2000.0"))
        desired_altitude = max(0.0, apogee + altitude_rate * (state.time - terminal_start))
    rotation = Vector3(0.0, 0.0, earth_omega)
    ecic_tangent = Vector3(
        tangent.x * math.cos(earth_omega * state.time) - tangent.y * math.sin(earth_omega * state.time),
        tangent.x * math.sin(earth_omega * state.time) + tangent.y * math.cos(earth_omega * state.time),
        tangent.z,
    )
    ecic_radial = Vector3(
        route_unit.x * math.cos(earth_omega * state.time) - route_unit.y * math.sin(earth_omega * state.time),
        route_unit.x * math.sin(earth_omega * state.time) + route_unit.y * math.cos(earth_omega * state.time),
        route_unit.z,
    )
    desired_position = ecic_radial.scaled(6_378_137.0 + desired_altitude)
    if state.time >= terminal_start and route_attributes.get("terminal-capture", "false").casefold() in {"1", "true", "yes"}:
        target_ecic_radial = Vector3(
            target.x * math.cos(earth_omega * state.time) - target.y * math.sin(earth_omega * state.time),
            target.x * math.sin(earth_omega * state.time) + target.y * math.cos(earth_omega * state.time),
            target.z,
        )
        desired_position = target_ecic_radial.scaled(6_378_137.0 + desired_altitude)
        time_floor = max(float(route_attributes.get("terminal-time-floor-s", "1.0")), 1.0)
        time_to_go = max(terminal_descent_end - state.time, time_floor)
        capture_gain = float(route_attributes.get("terminal-capture-gain", "1.0"))
        radial_now = state.position.vector.scaled(1.0 / max(state.position.vector.norm(), 1.0e-12))
        position_error = desired_position - state.position.vector
        current_altitude = state.position.vector.norm() - 6_378_137.0
        radial_speed = state.velocity.vector.dot(radial_now)
        altitude_error = desired_altitude - current_altitude
        altitude_gain = float(route_attributes.get("terminal-altitude-gain", "0.001"))
        radial_damping = float(route_attributes.get("terminal-radial-damping", "0.8"))
        radial_rate = altitude_gain * altitude_error - radial_damping * radial_speed
        descent_limit = abs(float(route_attributes.get("terminal-descent-rate-mps", "1000.0")))
        final_descent_limit = abs(float(route_attributes.get("terminal-descent-rate-final-mps", str(descent_limit))))
        descent_fraction = max(0.0, min(1.0, (state.time - terminal_start) / max(terminal_descent_end - terminal_start, 1.0)))
        descent_limit += (final_descent_limit - descent_limit) * descent_fraction
        climb_limit = abs(float(route_attributes.get("terminal-climb-rate-mps", str(descent_limit))))
        radial_rate = max(-descent_limit, min(climb_limit, radial_rate))
        horizontal_error = position_error - radial_now.scaled(position_error.dot(radial_now))
        horizontal_state_velocity = state.velocity.vector - radial_now.scaled(radial_speed)
        velocity_damping = max(0.0, float(route_attributes.get("terminal-velocity-damping", "0.0")))
        horizontal_gain = float(route_attributes.get("terminal-horizontal-gain", str(capture_gain)))
        horizontal_velocity = horizontal_error.scaled(horizontal_gain / time_to_go) - horizontal_state_velocity.scaled(velocity_damping)
        commanded_velocity = horizontal_velocity + radial_now.scaled(radial_rate) + rotation.cross(state.position.vector)
        command_speed = abs(float(route_attributes.get("terminal-command-speed-mps", route_attributes.get("terminal-speed-mps", "2000.0"))))
        if command_speed > 0.0 and commanded_velocity.norm() > command_speed:
            commanded_velocity = commanded_velocity.scaled(command_speed / commanded_velocity.norm())
        return commanded_velocity
    position_capture_gain = float(route_attributes.get("position-capture-gain", "0.0"))
    position_correction = desired_position - state.position.vector
    return (
        ecic_tangent.scaled(speed)
        + ecic_radial.scaled(altitude_rate)
        + position_correction.scaled(position_capture_gain)
        + rotation.cross(state.position.vector)
    )
####


def _runtime_rectangle_route_velocity(
    route_attributes: Mapping[str, str],
    state: RigidBody6DofState,
    earth_omega: float,
) -> Vector3 | None:
    """Resolve a small local closed rectangle into a native ECIC velocity.

    The course is expressed in metres from the declared start point: east,
    then north, then west, then south back to the start.  The second corner
    is the elevated corner.  This is a reusable route shape, not a vehicle
    or mission-specific grammar construct.
    """

    required = ("start-latitude-deg", "start-longitude-deg", "duration-s", "rectangle-length-m", "rectangle-width-m")
    mode = route_attributes.get("mode", "").casefold()
    if mode in {"figure-eight", "figure8"}:
        return _runtime_figure_eight_route_velocity(route_attributes, state, earth_omega)
    if mode == "racetrack":
        return _runtime_racetrack_route_velocity(route_attributes, state, earth_omega)
    if mode != "rectangle" or any(name not in route_attributes for name in required):
        return None
    latitude = math.radians(float(route_attributes["start-latitude-deg"]))
    longitude = math.radians(float(route_attributes["start-longitude-deg"]))
    start_altitude = float(route_attributes.get("start-altitude-m", "0.0"))
    elevated_altitude = float(route_attributes.get("elevated-corner-altitude-m", str(start_altitude)))
    duration = max(float(route_attributes["duration-s"]), 1.0)
    leg_duration = duration / 4.0
    leg = min(3, max(0, int(state.time / leg_duration)))
    fraction = max(0.0, min(1.0, (state.time - leg * leg_duration) / leg_duration))
    radius = 6_378_137.0 + start_altitude
    radial = Vector3(math.cos(latitude) * math.cos(longitude), math.cos(latitude) * math.sin(longitude), math.sin(latitude))
    east = Vector3(-math.sin(longitude), math.cos(longitude), 0.0)
    north = Vector3(-math.sin(latitude) * math.cos(longitude), -math.sin(latitude) * math.sin(longitude), math.cos(latitude))
    length = float(route_attributes["rectangle-length-m"])
    width = float(route_attributes["rectangle-width-m"])
    corners = (
        radial.scaled(radius),
        radial.scaled(radius) + east.scaled(length),
        radial.scaled(radius) + east.scaled(length) + north.scaled(width) + radial.scaled(elevated_altitude - start_altitude),
        radial.scaled(radius) + north.scaled(width),
        radial.scaled(radius),
    )
    origin = corners[leg]
    destination = corners[leg + 1]
    desired_position = origin + (destination - origin).scaled(fraction)
    leg_vector = destination - origin
    leg_direction = leg_vector.scaled(1.0 / max(leg_vector.norm(), 1.0e-12))
    corner_window = min(leg_duration / 2.0, max(0.0, float(route_attributes.get("rectangle-corner-window-s", "0.0"))))
    if corner_window > 0.0 and state.time - leg * leg_duration < corner_window and leg > 0:
        previous_vector = corners[leg] - corners[leg - 1]
        previous_direction = previous_vector.scaled(1.0 / max(previous_vector.norm(), 1.0e-12))
        blend = (state.time - leg * leg_duration) / corner_window
        leg_direction = (previous_direction.scaled(1.0 - blend) + leg_direction.scaled(blend)).scaled(
            1.0 / max((previous_direction.scaled(1.0 - blend) + leg_direction.scaled(blend)).norm(), 1.0e-12)
        )
    elif corner_window > 0.0 and leg < 3 and leg_duration - (state.time - leg * leg_duration) < corner_window:
        next_vector = corners[leg + 2] - corners[leg + 1]
        next_direction = next_vector.scaled(1.0 / max(next_vector.norm(), 1.0e-12))
        blend = (leg_duration - (state.time - leg * leg_duration)) / corner_window
        leg_direction = (leg_direction.scaled(blend) + next_direction.scaled(1.0 - blend)).scaled(
            1.0 / max((leg_direction.scaled(blend) + next_direction.scaled(1.0 - blend)).norm(), 1.0e-12)
        )
    speed = abs(float(route_attributes.get("rectangle-speed-mps", "0.0")))
    if speed <= 0.0:
        speed = leg_vector.norm() / leg_duration
    capture_gain = float(route_attributes.get("position-capture-gain", "0.0"))
    position_correction = (desired_position - state.position.vector).scaled(capture_gain)
    configured_limit = float(
        route_attributes.get("position-capture-max-correction-mps", str(max(speed * 0.25, 1.0)))
    )
    position_correction = limit_vector_norm(position_correction, max(configured_limit, 0.0))
    rotation = Vector3(0.0, 0.0, earth_omega)
    return (
        leg_direction.scaled(speed)
        + position_correction
        + rotation.cross(state.position.vector)
    )
####


def _runtime_racetrack_phase_profile(
    route_attributes: Mapping[str, str],
) -> tuple[tuple[float, ...], float, float, float, float, float, float] | None:
    """Return cumulative phase ends and the derived racetrack parameters."""

    required = (
        "start-latitude-deg",
        "start-longitude-deg",
        "racetrack-length-m",
        "racetrack-turn-radius-m",
        "racetrack-speed-mps",
        "racetrack-low-altitude-m",
        "racetrack-high-altitude-m",
        "racetrack-climb-rate-mps",
        "racetrack-descent-rate-mps",
    )
    if route_attributes.get("mode", "").casefold() != "racetrack" or any(
        name not in route_attributes for name in required
    ):
        return None
    length = max(float(route_attributes["racetrack-length-m"]), 1.0)
    turn_radius = max(float(route_attributes["racetrack-turn-radius-m"]), 1.0)
    speed = max(abs(float(route_attributes["racetrack-speed-mps"])), 1.0e-6)
    low_altitude = float(route_attributes["racetrack-low-altitude-m"])
    high_altitude = float(route_attributes["racetrack-high-altitude-m"])
    altitude_delta = high_altitude - low_altitude
    climb_rate = max(abs(float(route_attributes["racetrack-climb-rate-mps"])), 1.0e-6)
    descent_rate = max(abs(float(route_attributes["racetrack-descent-rate-mps"])), 1.0e-6)
    straight_time = length / speed
    turn_time = math.pi * turn_radius / speed
    climb_time = min(straight_time, max(0.0, altitude_delta / climb_rate))
    descent_time = min(straight_time, max(0.0, altitude_delta / descent_rate))
    outbound_level_time = max(0.0, straight_time - climb_time)
    inbound_level_time = max(0.0, straight_time - descent_time)
    durations = (
        climb_time,
        outbound_level_time,
        turn_time,
        descent_time,
        inbound_level_time,
        turn_time,
    )
    total = sum(durations)
    declared_duration = route_attributes.get("duration-s")
    if declared_duration is not None:
        total = max(float(declared_duration), total)
    cumulative: list[float] = []
    elapsed = 0.0
    for duration in durations:
        elapsed += duration
        cumulative.append(elapsed)
    if total > cumulative[-1] and cumulative:
        cumulative[-1] = total
    return tuple(cumulative), length, turn_radius, speed, low_altitude, high_altitude, total
    ####


def _runtime_racetrack_local_reference(
    route_attributes: Mapping[str, str],
    state_time: float,
) -> tuple[float, float, float, float, float, float, int] | None:
    """Return local east/north position, tangent, vertical rate, and phase."""

    profile = _runtime_racetrack_phase_profile(route_attributes)
    if profile is None:
        return None
    cumulative, length, turn_radius, speed, low_altitude, high_altitude, total = profile
    if state_time > total:
        # The route closes at the gate, then continues a short distance along
        # the finish course so an independent fly-by evaluator can observe an
        # actual crossing rather than a sample exactly on the plane.
        return 0.0, 0.0, low_altitude, 1.0, 0.0, 0.0, 6
    durations = (
        cumulative[0],
        cumulative[1] - cumulative[0],
        cumulative[2] - cumulative[1],
        cumulative[3] - cumulative[2],
        cumulative[4] - cumulative[3],
        cumulative[5] - cumulative[4],
    )
    phase = min(5, max(0, next((index for index, end in enumerate(cumulative) if state_time < end), 5)))
    phase_start = 0.0 if phase == 0 else cumulative[phase - 1]
    local_time = max(0.0, min(state_time, total) - phase_start)
    climb_time = durations[0]
    descent_time = durations[3]
    climb_distance = speed * climb_time
    descent_distance = speed * descent_time
    delta_altitude = high_altitude - low_altitude
    if phase == 0:
        x, y = speed * local_time, 0.0
        tangent_x, tangent_y, vertical_rate = 1.0, 0.0, delta_altitude / max(climb_time, 1.0e-6)
        altitude = low_altitude + delta_altitude * local_time / max(climb_time, 1.0e-6)
    elif phase == 1:
        x, y = climb_distance + speed * local_time, 0.0
        tangent_x, tangent_y, vertical_rate, altitude = 1.0, 0.0, 0.0, high_altitude
    elif phase == 2:
        theta = -math.pi / 2.0 + math.pi * local_time / max(durations[2], 1.0e-6)
        x, y = length + turn_radius * math.cos(theta), turn_radius + turn_radius * math.sin(theta)
        tangent_x, tangent_y, vertical_rate, altitude = -math.sin(theta), math.cos(theta), 0.0, high_altitude
    elif phase == 3:
        x, y = length - speed * local_time, 2.0 * turn_radius
        tangent_x, tangent_y, vertical_rate = -1.0, 0.0, -delta_altitude / max(descent_time, 1.0e-6)
        altitude = high_altitude - delta_altitude * local_time / max(descent_time, 1.0e-6)
    elif phase == 4:
        x, y = length - descent_distance - speed * local_time, 2.0 * turn_radius
        tangent_x, tangent_y, vertical_rate, altitude = -1.0, 0.0, 0.0, low_altitude
    else:
        theta = math.pi / 2.0 + math.pi * local_time / max(durations[5], 1.0e-6)
        x, y = turn_radius * math.cos(theta), turn_radius + turn_radius * math.sin(theta)
        tangent_x, tangent_y, vertical_rate, altitude = -math.sin(theta), math.cos(theta), 0.0, low_altitude
    return x, y, altitude, tangent_x, tangent_y, vertical_rate, phase
    ####


def _runtime_racetrack_route_velocity(
    route_attributes: Mapping[str, str],
    state: RigidBody6DofState,
    earth_omega: float,
) -> Vector3 | None:
    """Resolve the reusable climb/turn/descent racetrack into ECIC velocity."""

    local = _runtime_racetrack_local_reference(route_attributes, state.time)
    if local is None:
        return None
    x, y, altitude, tangent_x, tangent_y, vertical_rate, _ = local
    latitude = math.radians(float(route_attributes["start-latitude-deg"]))
    longitude = math.radians(float(route_attributes["start-longitude-deg"]))
    radius = 6_378_137.0
    radial = Vector3(math.cos(latitude) * math.cos(longitude), math.cos(latitude) * math.sin(longitude), math.sin(latitude))
    east = Vector3(-math.sin(longitude), math.cos(longitude), 0.0)
    north = Vector3(-math.sin(latitude) * math.cos(longitude), -math.sin(latitude) * math.sin(longitude), math.cos(latitude))
    desired_position = radial.scaled(radius + altitude) + east.scaled(x) + north.scaled(y)
    speed = max(abs(float(route_attributes["racetrack-speed-mps"])), 1.0e-6)
    tangent = east.scaled(tangent_x * speed) + north.scaled(tangent_y * speed) + radial.scaled(vertical_rate)
    altitude_error = altitude - (state.position.vector.norm() - radius)
    altitude_gain = max(0.0, float(route_attributes.get("racetrack-altitude-capture-gain-per-s", "0.0")))
    altitude_limit = max(0.0, float(route_attributes.get("racetrack-altitude-capture-max-mps", "0.0")))
    if altitude_gain > 0.0 and altitude_limit > 0.0:
        altitude_correction = max(-altitude_limit, min(altitude_limit, altitude_gain * altitude_error))
        tangent = tangent + radial.scaled(altitude_correction)
    correction_gain = max(0.0, float(route_attributes.get("position-capture-gain", "0.0")))
    correction = limit_vector_norm(
        (desired_position - state.position.vector).scaled(correction_gain),
        max(float(route_attributes.get("position-capture-max-correction-mps", str(max(speed * 0.25, 1.0)))), 0.0),
    )
    return tangent + correction + Vector3(0.0, 0.0, earth_omega).cross(state.position.vector)
    ####


def _runtime_rectangle_waypoint_position(
    route_attributes: Mapping[str, str],
    state: RigidBody6DofState,
) -> Vector3 | None:
    """Return the active smooth-route reference for route diagnostics."""

    required = ("start-latitude-deg", "start-longitude-deg", "duration-s", "rectangle-length-m", "rectangle-width-m")
    if route_attributes.get("mode", "").casefold() in {"figure-eight", "figure8"}:
        return _runtime_figure_eight_waypoint_position(route_attributes, state)
    if route_attributes.get("mode", "").casefold() == "racetrack":
        local = _runtime_racetrack_local_reference(route_attributes, state.time)
        if local is None:
            return None
        x, y, altitude, _, _, _, _ = local
        latitude = math.radians(float(route_attributes["start-latitude-deg"]))
        longitude = math.radians(float(route_attributes["start-longitude-deg"]))
        radial = Vector3(math.cos(latitude) * math.cos(longitude), math.cos(latitude) * math.sin(longitude), math.sin(latitude))
        east = Vector3(-math.sin(longitude), math.cos(longitude), 0.0)
        north = Vector3(-math.sin(latitude) * math.cos(longitude), -math.sin(latitude) * math.sin(longitude), math.cos(latitude))
        return radial.scaled(6_378_137.0 + altitude) + east.scaled(x) + north.scaled(y)
    if route_attributes.get("mode", "").casefold() != "rectangle" or any(name not in route_attributes for name in required):
        return None
    latitude = math.radians(float(route_attributes["start-latitude-deg"]))
    longitude = math.radians(float(route_attributes["start-longitude-deg"]))
    start_altitude = float(route_attributes.get("start-altitude-m", "0.0"))
    elevated_altitude = float(route_attributes.get("elevated-corner-altitude-m", str(start_altitude)))
    duration = max(float(route_attributes["duration-s"]), 1.0)
    leg_duration = duration / 4.0
    leg = min(3, max(0, int(state.time / leg_duration)))
    fraction = max(0.0, min(1.0, (state.time - leg * leg_duration) / leg_duration))
    radius = 6_378_137.0 + start_altitude
    radial = Vector3(math.cos(latitude) * math.cos(longitude), math.cos(latitude) * math.sin(longitude), math.sin(latitude))
    east = Vector3(-math.sin(longitude), math.cos(longitude), 0.0)
    north = Vector3(-math.sin(latitude) * math.cos(longitude), -math.sin(latitude) * math.sin(longitude), math.cos(latitude))
    length = float(route_attributes["rectangle-length-m"])
    width = float(route_attributes["rectangle-width-m"])
    corners = (
        radial.scaled(radius),
        radial.scaled(radius) + east.scaled(length),
        radial.scaled(radius) + east.scaled(length) + north.scaled(width) + radial.scaled(elevated_altitude - start_altitude),
        radial.scaled(radius) + north.scaled(width),
        radial.scaled(radius),
    )
    return corners[leg] + (corners[leg + 1] - corners[leg]).scaled(fraction)


def _runtime_figure_eight_waypoint_position(
    route_attributes: Mapping[str, str],
    state: RigidBody6DofState,
) -> Vector3 | None:
    """Return the current smooth figure-eight reference position."""

    required = ("start-latitude-deg", "start-longitude-deg", "duration-s", "figure-eight-length-m", "figure-eight-width-m")
    if any(name not in route_attributes for name in required):
        return None
    latitude = math.radians(float(route_attributes["start-latitude-deg"]))
    longitude = math.radians(float(route_attributes["start-longitude-deg"]))
    start_altitude = float(route_attributes.get("start-altitude-m", "0.0"))
    duration = max(float(route_attributes["duration-s"]), 1.0)
    theta = 2.0 * math.pi * max(0.0, min(duration, state.time)) / duration
    radius = 6_378_137.0 + start_altitude
    radial = Vector3(math.cos(latitude) * math.cos(longitude), math.cos(latitude) * math.sin(longitude), math.sin(latitude))
    east = Vector3(-math.sin(longitude), math.cos(longitude), 0.0)
    north = Vector3(-math.sin(latitude) * math.cos(longitude), -math.sin(latitude) * math.sin(longitude), math.cos(latitude))
    altitude_offset = _runtime_figure_eight_altitude_offset(route_attributes, theta)
    return (
        radial.scaled(radius)
        + east.scaled(0.5 * float(route_attributes["figure-eight-length-m"]) * math.sin(theta))
        + north.scaled(0.5 * float(route_attributes["figure-eight-width-m"]) * math.sin(theta) * math.cos(theta))
        + radial.scaled(altitude_offset)
    )
    ####


def _runtime_figure_eight_altitude_offset(route_attributes: Mapping[str, str], theta: float) -> float:
    """Return the declared smooth altitude excursion for a figure-eight."""

    amplitude = float(route_attributes.get("figure-eight-altitude-amplitude-m", "0.0"))
    return amplitude * math.sin(theta)
    ####


def _runtime_figure_eight_pitch_angle(route_attributes: Mapping[str, str], theta: float) -> float:
    """Return the signed path-pitch command scheduled over the two lobes."""

    return math.radians(float(route_attributes.get("figure-eight-pitch-deg", "0.0"))) * math.sin(theta)
    ####


def _runtime_rectangle_corner_positions(route_attributes: Mapping[str, str]) -> tuple[Vector3, ...] | None:
    """Return the fixed five-point rectangle used by route diagnostics."""

    required = ("start-latitude-deg", "start-longitude-deg", "rectangle-length-m", "rectangle-width-m")
    if route_attributes.get("mode", "").casefold() != "rectangle" or any(
        name not in route_attributes for name in required
    ):
        return None
    latitude = math.radians(float(route_attributes["start-latitude-deg"]))
    longitude = math.radians(float(route_attributes["start-longitude-deg"]))
    start_altitude = float(route_attributes.get("start-altitude-m", "0.0"))
    elevated_altitude = float(route_attributes.get("elevated-corner-altitude-m", str(start_altitude)))
    radius = 6_378_137.0 + start_altitude
    radial = Vector3(math.cos(latitude) * math.cos(longitude), math.cos(latitude) * math.sin(longitude), math.sin(latitude))
    east = Vector3(-math.sin(longitude), math.cos(longitude), 0.0)
    north = Vector3(-math.sin(latitude) * math.cos(longitude), -math.sin(latitude) * math.sin(longitude), math.cos(latitude))
    length = float(route_attributes["rectangle-length-m"])
    width = float(route_attributes["rectangle-width-m"])
    return (
        radial.scaled(radius),
        radial.scaled(radius) + east.scaled(length),
        radial.scaled(radius) + east.scaled(length) + north.scaled(width) + radial.scaled(elevated_altitude - start_altitude),
        radial.scaled(radius) + north.scaled(width),
        radial.scaled(radius),
    )
    ####


def _runtime_route_tracking_geometry(
    route_attributes: Mapping[str, str],
    route_target: Vector3,
    state: RigidBody6DofState,
    earth_omega: float,
) -> dict[str, float]:
    """Return path-relative errors for smooth and waypoint routes.

    ``route_target_error_m`` is distance to the moving reference point.  The
    cross/along-track channels below instead measure error relative to the
    instantaneous route tangent, so a phase lag is not misreported as an
    unstable path.  The calculation is diagnostic only and does not steer the
    vehicle.
    """

    reference_velocity = _runtime_route_velocity(route_attributes, {}, state, earth_omega)
    if reference_velocity is None or reference_velocity.norm() <= 1.0e-12:
        return {}
    radial = state.position.vector.scaled(1.0 / max(state.position.vector.norm(), 1.0e-12))
    tangent = reference_velocity - radial.scaled(reference_velocity.dot(radial))
    if tangent.norm() <= 1.0e-12:
        return {}
    tangent = tangent.scaled(1.0 / tangent.norm())
    lateral = radial.cross(tangent)
    if lateral.norm() <= 1.0e-12:
        return {}
    lateral = lateral.scaled(1.0 / lateral.norm())
    position_error = state.position.vector - route_target
    actual_velocity = state.velocity.vector - radial.scaled(state.velocity.vector.dot(radial))
    heading_error = 0.0
    if actual_velocity.norm() > 1.0e-12:
        actual_direction = actual_velocity.scaled(1.0 / actual_velocity.norm())
        heading_error = math.atan2(radial.dot(tangent.cross(actual_direction)), tangent.dot(actual_direction))
    return {
        "route_cross_track_error_m": position_error.dot(lateral),
        "route_along_track_error_m": position_error.dot(tangent),
        "route_heading_error_deg": math.degrees(heading_error),
    }
    ####


def _runtime_figure_eight_route_velocity(
    route_attributes: Mapping[str, str],
    state: RigidBody6DofState,
    earth_omega: float,
) -> Vector3 | None:
    """Resolve a smooth local figure-eight course into an ECIC velocity.

    The course is a reusable guidance geometry, not a vehicle-specific
    controller: east/north coordinates follow ``(A sin(theta),
    B sin(theta) cos(theta))`` over one duration.  The path is continuous at
    the crossing and its tangent reverses turn sense between lobes.
    """

    required = ("start-latitude-deg", "start-longitude-deg", "duration-s", "figure-eight-length-m", "figure-eight-width-m")
    if any(name not in route_attributes for name in required):
        return None
    latitude = math.radians(float(route_attributes["start-latitude-deg"]))
    longitude = math.radians(float(route_attributes["start-longitude-deg"]))
    start_altitude = float(route_attributes.get("start-altitude-m", "0.0"))
    duration = max(float(route_attributes["duration-s"]), 1.0)
    theta = 2.0 * math.pi * max(0.0, min(duration, state.time)) / duration
    radius = 6_378_137.0 + start_altitude
    radial = Vector3(math.cos(latitude) * math.cos(longitude), math.cos(latitude) * math.sin(longitude), math.sin(latitude))
    terminal_start = float(route_attributes.get("terminal-start-s", str(duration)))
    if (
        route_attributes.get("terminal-capture", "false").casefold() in {"1", "true", "yes"}
        and state.time >= terminal_start
    ):
        # The figure-eight is a maneuver reference, not a terminal contract.
        # Once its declared window is complete, switch to an explicit return
        # velocity so the aircraft closes a real recovery phase rather than
        # being judged against home while still flying the final tangent.
        terminal_altitude = float(route_attributes.get("terminal-target-altitude-m", str(start_altitude)))
        desired_position = radial.scaled(6_378_137.0 + terminal_altitude)
        radial_now = state.position.vector.scaled(1.0 / max(state.position.vector.norm(), 1.0e-12))
        position_error = desired_position - state.position.vector
        radial_speed = state.velocity.vector.dot(radial_now)
        horizontal_error = position_error - radial_now.scaled(position_error.dot(radial_now))
        horizontal_velocity = state.velocity.vector - radial_now.scaled(radial_speed)
        latitude_now = math.asin(max(-1.0, min(1.0, radial_now.z)))
        longitude_now = math.atan2(radial_now.y, radial_now.x)
        east_now = Vector3(-math.sin(longitude_now), math.cos(longitude_now), 0.0)
        north_now = Vector3(
            -math.sin(latitude_now) * math.cos(longitude_now),
            -math.sin(latitude_now) * math.sin(longitude_now),
            math.cos(latitude_now),
        )
        capture_gain = max(0.0, float(route_attributes.get("terminal-capture-gain", "0.8")))
        damping = max(0.0, float(route_attributes.get("terminal-velocity-damping", "0.8")))
        terminal_speed = abs(float(route_attributes.get("terminal-speed-mps", "0.0")))
        terminal_heading = math.radians(float(route_attributes.get("terminal-heading-deg", "90.0")))
        terminal_course = (
            north_now.scaled(terminal_speed * math.cos(terminal_heading))
            + east_now.scaled(terminal_speed * math.sin(terminal_heading))
        )
        terminal_recovery_mode = route_attributes.get("terminal-recovery-mode", "point").casefold()
        if terminal_recovery_mode in {"gate", "line", "fly-by", "fly_by"}:
            lateral_now = radial_now.cross(terminal_course.scaled(1.0 / max(terminal_speed, 1.0e-12)))
            lateral_error = horizontal_error.dot(lateral_now)
            lateral_velocity = horizontal_velocity.dot(lateral_now)
            lateral_gain = max(0.0, float(route_attributes.get("terminal-cross-track-gain", str(capture_gain))))
            lateral_damping = max(0.0, float(route_attributes.get("terminal-cross-track-damping", str(damping))))
            intercept_distance = max(0.0, float(route_attributes.get("terminal-intercept-distance-m", "0.0")))
            intercept_capture = max(0.0, float(route_attributes.get("terminal-intercept-capture-m", "100.0")))
            intercept_point = desired_position - terminal_course.scaled(intercept_distance / max(terminal_speed, 1.0e-12))
            intercept_error = intercept_point - state.position.vector
            intercept_horizontal_error = intercept_error - radial_now.scaled(intercept_error.dot(radial_now))
            if intercept_horizontal_error.norm() > intercept_capture:
                # First align with a point behind the gate.  This is a finite
                # intercept stage, not an attractor at the terminal itself.
                desired_velocity = terminal_course + intercept_horizontal_error.scaled(capture_gain) - horizontal_velocity.scaled(damping)
            else:
                # Once aligned, a fixed-wing terminal is a gate crossing, not
                # a hover-like point capture. Correct only cross-track error
                # while preserving the declared forward course; attracting the
                # full position vector creates an orbit/limit cycle.
                desired_velocity = terminal_course + lateral_now.scaled(
                    -lateral_gain * lateral_error - lateral_damping * lateral_velocity
                )
        else:
            desired_velocity = terminal_course + horizontal_error.scaled(capture_gain) - horizontal_velocity.scaled(damping)
        altitude_error = terminal_altitude - (state.position.vector.norm() - 6_378_137.0)
        desired_velocity = desired_velocity + radial_now.scaled(
            float(route_attributes.get("terminal-altitude-gain", "0.8")) * altitude_error
            - float(route_attributes.get("terminal-radial-damping", "0.8")) * radial_speed
        )
        command_speed = abs(float(route_attributes.get("terminal-command-speed-mps", "20.0")))
        if command_speed > 0.0 and desired_velocity.norm() > command_speed:
            desired_velocity = desired_velocity.scaled(command_speed / desired_velocity.norm())
        return desired_velocity + Vector3(0.0, 0.0, earth_omega).cross(state.position.vector)
    east = Vector3(-math.sin(longitude), math.cos(longitude), 0.0)
    north = Vector3(-math.sin(latitude) * math.cos(longitude), -math.sin(latitude) * math.sin(longitude), math.cos(latitude))
    half_length = 0.5 * float(route_attributes["figure-eight-length-m"])
    half_width = 0.5 * float(route_attributes["figure-eight-width-m"])
    east_offset = half_length * math.sin(theta)
    north_offset = half_width * math.sin(theta) * math.cos(theta)
    east_rate = half_length * math.cos(theta)
    north_rate = half_width * math.cos(2.0 * theta)
    altitude_offset = _runtime_figure_eight_altitude_offset(route_attributes, theta)
    desired_position = radial.scaled(radius + altitude_offset) + east.scaled(east_offset) + north.scaled(north_offset)
    tangent = east.scaled(east_rate) + north.scaled(north_rate)
    tangent = tangent.scaled(1.0 / max(tangent.norm(), 1.0e-12))
    speed = abs(float(route_attributes.get("figure-eight-speed-mps", "0.0")))
    if speed <= 0.0:
        speed = math.hypot(east_rate, north_rate) * 2.0 * math.pi / duration
    pitch = _runtime_figure_eight_pitch_angle(route_attributes, theta)
    horizontal_speed = speed * math.cos(pitch)
    altitude_rate = float(route_attributes.get("figure-eight-altitude-amplitude-m", "0.0")) * (2.0 * math.pi / duration) * math.cos(theta)
    correction = desired_position - state.position.vector
    correction_gain = float(route_attributes.get("position-capture-gain", "0.0"))
    correction = limit_vector_norm(correction.scaled(correction_gain), max(float(route_attributes.get("position-capture-max-correction-mps", str(max(speed * 0.25, 1.0)))), 0.0))
    return tangent.scaled(horizontal_speed) + radial.scaled(speed * math.sin(pitch) + altitude_rate) + correction + Vector3(0.0, 0.0, earth_omega).cross(state.position.vector)
    ####
####


def _runtime_rectangle_bank_angle(
    route_attributes: Mapping[str, str],
    state: RigidBody6DofState,
    earth_omega: float = 0.0,
) -> float | None:
    """Return a scheduled bank command with optional bounded path feedback."""

    if route_attributes.get("mode", "").casefold() in {"figure-eight", "figure8"}:
        duration = max(float(route_attributes.get("duration-s", "1.0")), 1.0)
        terminal_start = float(route_attributes.get("terminal-start-s", str(duration)))
        if route_attributes.get("terminal-capture", "false").casefold() in {"1", "true", "yes"} and state.time >= terminal_start:
            # Return-to-terminal guidance is a velocity/heading problem, not
            # another figure-eight bank command.  Releasing the scheduled bank
            # here prevents the recovery turn from fighting the terminal gate.
            return None
        theta = 2.0 * math.pi * max(0.0, min(duration, state.time)) / duration
        scheduled = math.radians(float(route_attributes.get("figure-eight-bank-deg", "20.0"))) * math.sin(theta)
        maximum = abs(float(route_attributes.get("route-max-bank-deg", route_attributes.get("figure-eight-bank-deg", "20.0"))))
    elif route_attributes.get("mode", "").casefold() == "racetrack":
        profile = _runtime_racetrack_phase_profile(route_attributes)
        if profile is not None and state.time >= profile[-1]:
            # The finish course is a terminal-evaluation reference.  Do not
            # continue steering bank after the declared racetrack duration;
            # otherwise a heading-capture correction can drive a finished
            # course back toward a table boundary before the stop condition
            # is observed.
            return None
        local = _runtime_racetrack_local_reference(route_attributes, state.time)
        if local is None:
            return None
        phase = local[-1]
        left_bank = math.radians(float(route_attributes.get("racetrack-left-bank-deg", "12.0")))
        right_bank = math.radians(float(route_attributes.get("racetrack-right-bank-deg", "-12.0")))
        scheduled = left_bank if phase == 2 else right_bank if phase == 5 else 0.0
        maximum = abs(float(route_attributes.get("route-max-bank-deg", str(max(abs(math.degrees(left_bank)), abs(math.degrees(right_bank)))))))
    else:
        if route_attributes.get("mode", "").casefold() != "rectangle" or "rectangle-bank-deg" not in route_attributes:
            return None
        try:
            duration = max(float(route_attributes["duration-s"]), 1.0)
            length = float(route_attributes["rectangle-length-m"])
            width = float(route_attributes["rectangle-width-m"])
        except (KeyError, ValueError):
            return None
        leg_duration = duration / 4.0
        leg = min(3, max(0, int(state.time / leg_duration)))
        local_time = state.time - leg * leg_duration
        window = min(leg_duration / 2.0, max(0.0, float(route_attributes.get("rectangle-corner-window-s", "0.0"))))
        strength = 0.0
        if window > 0.0 and local_time < window and leg > 0:
            strength = 1.0 - local_time / window
        elif window > 0.0 and leg < 3 and leg_duration - local_time < window:
            strength = 1.0 - (leg_duration - local_time) / window
        _ = length, width
        scheduled = -math.radians(float(route_attributes["rectangle-bank-deg"])) * strength
        maximum = abs(float(route_attributes.get("route-max-bank-deg", route_attributes["rectangle-bank-deg"])))

    route_target = None
    feedback_gain = float(route_attributes.get("route-cross-track-bank-gain-deg-per-m", "0.0"))
    heading_feedback_gain = float(route_attributes.get("route-heading-bank-gain-deg-per-deg", "0.0"))
    if feedback_gain != 0.0 or heading_feedback_gain != 0.0:
        route_target = _runtime_rectangle_waypoint_position(route_attributes, state)
        if route_target is not None:
            geometry = _runtime_route_tracking_geometry(route_attributes, route_target, state, earth_omega)
            scheduled += math.radians(feedback_gain * geometry.get("route_cross_track_error_m", 0.0))
            heading_correction_deg = heading_feedback_gain * geometry.get("route_heading_error_deg", 0.0)
            heading_correction_limit_deg = abs(float(route_attributes.get("route-heading-bank-max-deg", "0.0")))
            if heading_correction_limit_deg > 0.0:
                heading_correction_deg = max(-heading_correction_limit_deg, min(heading_correction_limit_deg, heading_correction_deg))
            scheduled += math.radians(heading_correction_deg)
    return max(-math.radians(maximum), min(math.radians(maximum), scheduled))
####


def _runtime_coordinated_turn_enabled(
    guidance_attributes: Mapping[str, str],
    route_attributes: Mapping[str, str],
) -> bool:
    """Resolve the shared coordinated-turn switch for smooth and polygonal routes."""

    route_mode = route_attributes.get("mode", "").casefold()
    names: tuple[str, ...] = ("rectangle-coordinated-turn", "racetrack-coordinated-turn", "figure-eight-coordinated-turn")
    if route_mode in {"figure-eight", "figure8"}:
        names = ("figure-eight-coordinated-turn", "rectangle-coordinated-turn")
    elif route_mode == "racetrack":
        names = ("racetrack-coordinated-turn", "rectangle-coordinated-turn")
    return any(guidance_attributes.get(name, "false").casefold() in {"1", "true", "yes"} for name in names)
####


def _runtime_fixed_wing_alpha_command_rad(guidance_attributes: Mapping[str, str]) -> float:
    """Return an explicit fixed-wing body-axis angle-of-attack command.

    A route tangent is a flight-path direction, not the aircraft body-X
    direction.  Airbreathers normally trim at a non-zero angle of attack, so
    the route attitude reference must add that declared alpha before the
    surface allocator is asked to produce a moment.  The default remains zero
    to preserve the point-mass/kinematic route contract for models that do not
    declare a fixed-wing alpha realization.
    """

    return math.radians(float(guidance_attributes.get("fixed-wing-alpha-command-deg", "0.0")))
    ####


def _runtime_fixed_wing_beta_command_rad(guidance_attributes: Mapping[str, str]) -> float:
    """Return an explicit fixed-wing trim sideslip command in radians."""

    return math.radians(float(guidance_attributes.get("fixed-wing-beta-command-deg", "0.0")))
    ####


def _runtime_rectangle_turn_errors(
    route_attributes: Mapping[str, str],
    state: RigidBody6DofState,
    route_direction: Vector3,
) -> tuple[float, float]:
    """Return heading and bank errors for a local coordinated rectangle turn."""

    radial = state.position.vector.scaled(1.0 / max(state.position.vector.norm(), 1.0e-12))
    latitude = math.asin(max(-1.0, min(1.0, radial.z)))
    longitude = math.atan2(radial.y, radial.x)
    east = Vector3(-math.sin(longitude), math.cos(longitude), 0.0)
    north = Vector3(-math.sin(latitude) * math.cos(longitude), -math.sin(latitude) * math.sin(longitude), math.cos(latitude))
    body_x = state.attitude.rotate(Vector3(1.0, 0.0, 0.0))
    body_y = state.attitude.rotate(Vector3(0.0, 1.0, 0.0))
    body_z = state.attitude.rotate(Vector3(0.0, 0.0, 1.0))
    current_horizontal = body_x - radial.scaled(body_x.dot(radial))
    desired_horizontal = route_direction - radial.scaled(route_direction.dot(radial))
    current_horizontal = current_horizontal.scaled(1.0 / max(current_horizontal.norm(), 1.0e-12))
    desired_horizontal = desired_horizontal.scaled(1.0 / max(desired_horizontal.norm(), 1.0e-12))
    current_heading = math.atan2(current_horizontal.dot(north), current_horizontal.dot(east))
    desired_heading = math.atan2(desired_horizontal.dot(north), desired_horizontal.dot(east))
    heading_error = math.atan2(math.sin(desired_heading - current_heading), math.cos(desired_heading - current_heading))
    current_bank = math.atan2(body_y.dot(radial.scaled(-1.0)), body_z.dot(radial.scaled(-1.0)))
    desired_bank = _runtime_rectangle_bank_angle(route_attributes, state) or 0.0
    return heading_error, desired_bank - current_bank
####


def _rotate_ecic_vector_to_ecfc(vector: Vector3, angle_radians: float) -> Vector3:
    """Resolve an ECIC vector in ECFC without applying transport velocity."""

    cosine = math.cos(angle_radians)
    sine = math.sin(angle_radians)
    return Vector3(
        cosine * vector.x + sine * vector.y,
        -sine * vector.x + cosine * vector.y,
        vector.z,
    )
    ####


def _rigid_body_local_attitude_observables(state: RigidBody6DofState, earth: EarthRotationAdapter) -> dict[str, float]:
    """Publish body attitude relative to the local geocentric NED frame.

    ECIC Euler angles are retained for historical/debugging continuity, but
    they are not aircraft roll, pitch, and heading.  This independent basis
    resolves the integrated body axes into local north/east/down components.
    ``local_roll_deg`` is body Euler roll; it is deliberately distinct from
    TAOS ``bankgc``/``bankgd`` aerodynamic bank about the velocity vector.
    """

    position_ecfc, _ = earth.ecic_to_ecfc(state.position, state.velocity, time_seconds=state.time)
    radius = max(position_ecfc.vector.norm(), 1.0e-12)
    longitude = Longitude(math.atan2(position_ecfc.vector.y, position_ecfc.vector.x))
    latitude = Latitude(math.asin(max(-1.0, min(1.0, position_ecfc.vector.z / radius))))
    basis = geocentric_unit_vectors(longitude, latitude)
    body_axes_ecfc = tuple(
        _rotate_ecic_vector_to_ecfc(state.attitude.rotate(axis), earth.angle(state.time))
        for axis in (Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0), Vector3(0.0, 0.0, 1.0))
    )
    body_x, body_y, body_z = body_axes_ecfc
    r11, r21, r31 = body_x.dot(basis.first), body_x.dot(basis.second), body_x.dot(basis.third)
    r32, r33 = body_y.dot(basis.third), body_z.dot(basis.third)
    local_pitch = math.asin(max(-1.0, min(1.0, -r31)))
    local_roll = math.atan2(r32, r33)
    local_heading = math.atan2(r21, r11)
    return {
        "local_roll_deg": math.degrees(local_roll),
        "local_pitch_deg": math.degrees(local_pitch),
        "local_heading_deg": math.degrees(local_heading),
    }
    ####


def _rigid_body_aero_observables(
    state: RigidBody6DofState,
    model: TableAerodynamicModel | DirectWrenchTableModel | None,
    *,
    minimum_air_data_speed_m_s: float = 0.1,
) -> dict[str, float]:
    """Publish table-aero channels without creating a second load path.

    The evaluator is called only for telemetry. The derivative still obtains
    its loads from the same ``TableAerodynamicModel`` instance in
    ``force_moment``. An inactive model returns explicit zero channels so
    consumers can use one stable schema across propulsion-only and aero cases.
    """

    if model is None:
        return {
            "aero_active": 0.0,
            "aero_density_kg_m3": 0.0,
            "aero_dynamic_pressure_pa": 0.0,
            "aero_airspeed_m_s": 0.0,
            "aero_mach": 0.0,
            "aero_alpha_deg": 0.0,
            "aero_sideslip_deg": 0.0,
            "aero_force_body_x_n": 0.0,
            "aero_force_body_y_n": 0.0,
            "aero_force_body_z_n": 0.0,
            "aero_moment_body_x_nm": 0.0,
            "aero_moment_body_y_nm": 0.0,
            "aero_moment_body_z_nm": 0.0,
            "aero_table_valid": 0.0,
            "aero_table_min_margin": math.nan,
            "aero_table_min_normalized_margin": math.nan,
            "aero_model_uncertainty_fraction": math.nan,
            "aero_model_estimated": 1.0,
            "aero_table_operational_margin": math.nan,
        }
    output = model.evaluate(state)
    air_data_valid = output.airspeed_m_s >= minimum_air_data_speed_m_s
    alpha_deg = math.degrees(output.angle_of_attack_rad) if air_data_valid else math.nan
    beta_deg = math.degrees(output.sideslip_rad) if air_data_valid else math.nan
    margins = {
        f"aero_table_margin_{name}": value for name, value in output.table_margins.items()
    }
    margins.update(
        {
            f"aero_table_normalized_margin_{name}": value
            for name, value in output.table_normalized_margins.items()
        }
    )
    margins["aero_table_min_margin"] = min(output.table_margins.values(), default=math.nan)
    margins["aero_table_min_normalized_margin"] = min(output.table_normalized_margins.values(), default=math.nan)
    margins["aero_table_valid"] = 1.0
    margins["aero_model_uncertainty_fraction"] = output.model_uncertainty_fraction
    margins["aero_model_estimated"] = 1.0 if output.source_quality.casefold() != "verified" else 0.0
    margins["aero_table_operational_margin"] = margins["aero_table_min_normalized_margin"] - output.model_uncertainty_fraction
    # Preserve the source-qualified margin view used by the vehicle-family
    # evidence harness. The canonical force/moment names above remain the
    # stable runtime contract; these aliases make composed decks auditable by
    # coefficient family without changing table lookup semantics.
    for name, value in output.table_margins.items():
        parts = name.split(".")
        if len(parts) == 3:
            _kind, coefficient, axis = parts
            margins[f"aero_table_margin_table.{coefficient}-static.{axis}"] = value
        elif len(parts) == 4:
            _kind, control_family, coefficient, axis = parts
            margins[f"aero_table_margin_table.{coefficient}-{control_family}.{axis}"] = value
    return {
        "aero_active": 1.0,
        "aero_density_kg_m3": output.density_kg_m3,
        "aero_dynamic_pressure_pa": output.dynamic_pressure_pa,
        "aero_airspeed_m_s": output.airspeed_m_s,
        "aero_mach": output.mach,
        "aero_alpha_deg": alpha_deg,
        "aero_sideslip_deg": beta_deg,
        "aero_air_data_valid": 1.0 if air_data_valid else 0.0,
        "aero_force_body_x_n": output.force_body_n.x,
        "aero_force_body_y_n": output.force_body_n.y,
        "aero_force_body_z_n": output.force_body_n.z,
        "aero_moment_body_x_nm": output.moment_body_nm.x,
        "aero_moment_body_y_nm": output.moment_body_nm.y,
        "aero_moment_body_z_nm": output.moment_body_nm.z,
        **{f"aero_query_{name}": value for name, value in output.query_values.items()},
        **margins,
    }
####


def _rigid_body_position_observables(
    state: RigidBody6DofState,
    target_attributes: Mapping[str, str],
    route_attributes: Mapping[str, str],
    earth_mu: float,
    earth_omega: float,
) -> dict[str, float]:
    """Publish Earth-fixed route coordinates and target range."""

    earth = EarthModel(
        Quantity(6_378_137.0, Unit.METER),
        0.0,
        Quantity(max(earth_mu, 1.0), Unit.METER_CUBED_PER_SECOND_SQUARED),
        Quantity(earth_omega, Unit.RADIAN_PER_SECOND),
    )
    position_ecfc, _ = EarthRotationAdapter(earth).ecic_to_ecfc(state.position, state.velocity, time_seconds=state.time)
    position = position_ecfc.vector
    radius = max(position.norm(), 1.0)
    target = _runtime_target_position(target_attributes, earth, EarthRotationAdapter(earth), state.time)
    result = {
        "latitude_deg": math.degrees(math.asin(max(-1.0, min(1.0, position.z / radius)))),
        "longitude_deg": math.degrees(math.atan2(position.y, position.x)),
        "altitude_m": radius - earth.equatorial_radius.si_value,
        "speed_m_s": state.velocity.vector.norm(),
        "range_to_target_m": 0.0 if target is None else (target - state.position.vector).norm(),
    }
    if route_attributes.get("mode", "").casefold() == "great-circle":
        route_geometry = _runtime_great_circle_geometry(route_attributes, target_attributes, position, state.velocity.vector)
        if route_geometry is not None:
            result.update(route_geometry)
    route_target = _runtime_rectangle_waypoint_position(route_attributes, state)
    if route_target is not None:
        route_mode = route_attributes.get("mode", "").casefold()
        if route_mode == "racetrack":
            local = _runtime_racetrack_local_reference(route_attributes, state.time)
            phase_index = float(local[-1]) if local is not None else 0.0
        else:
            duration = max(float(route_attributes.get("duration-s", "1.0")), 1.0)
            phase_index = float(min(3, max(0, int(state.time / (duration / 4.0)))))
        if route_mode == "rectangle":
            result["route_leg_index"] = phase_index
        elif route_mode in {"racetrack", "figure-eight", "figure8"}:
            # A smooth figure-eight has no physical waypoint corners.  Keep
            # its lobe/phase transitions distinct from square-course legs.
            result["route_phase_index"] = phase_index
        result["route_target_error_m"] = (route_target - state.position.vector).norm()
        if route_attributes.get("mode", "").casefold() in {"rectangle", "racetrack", "figure-eight", "figure8"}:
            result.update(_runtime_route_tracking_geometry(route_attributes, route_target, state, earth_omega))
        corners = _runtime_rectangle_corner_positions(route_attributes)
        if corners is not None:
            result.update(
                {
                    f"route_corner_{index}_error_m": (corner - state.position.vector).norm()
                    for index, corner in enumerate(corners[:-1])
                }
            )
    return result
####


def _runtime_great_circle_geometry(
    route_attributes: Mapping[str, str],
    target_attributes: Mapping[str, str],
    position_ecfc: Vector3,
    velocity_ecic: Vector3,
) -> dict[str, float] | None:
    """Return signed cross-track and local heading errors for a declared route.

    The route contract is geodesic, while the rigid-body state is propagated
    in ECIC.  This helper intentionally performs the diagnostic in ECFC
    coordinates and removes the radial component before comparing heading.
    It reports geometry only; it does not steer the vehicle or inject force.
    """

    earth_radius = 6_378_137.0
    required = ("start-latitude-deg", "start-longitude-deg")
    if any(name not in route_attributes for name in required):
        return None
    if "latitude-deg" not in target_attributes or "longitude-deg" not in target_attributes:
        return None
    start_lat = math.radians(float(route_attributes["start-latitude-deg"]))
    start_lon = math.radians(float(route_attributes["start-longitude-deg"]))
    target_lat = math.radians(float(target_attributes["latitude-deg"]))
    target_lon = math.radians(float(target_attributes["longitude-deg"]))
    start = Vector3(math.cos(start_lat) * math.cos(start_lon), math.cos(start_lat) * math.sin(start_lon), math.sin(start_lat))
    target = Vector3(math.cos(target_lat) * math.cos(target_lon), math.cos(target_lat) * math.sin(target_lon), math.sin(target_lat))
    radial = position_ecfc.scaled(1.0 / max(position_ecfc.norm(), 1.0e-12))
    route_normal = start.cross(target)
    normal_norm = route_normal.norm()
    if normal_norm <= 1.0e-12:
        return None
    route_normal = route_normal.scaled(1.0 / normal_norm)
    cross_track_angle = math.asin(max(-1.0, min(1.0, radial.dot(route_normal))))
    tangent = target - radial.scaled(radial.dot(target))
    tangent_norm = tangent.norm()
    if tangent_norm <= 1.0e-12:
        return {"route_cross_track_error_m": earth_radius * cross_track_angle, "route_heading_error_deg": 0.0}
    tangent = tangent.scaled(1.0 / tangent_norm)
    horizontal_velocity = velocity_ecic - radial.scaled(velocity_ecic.dot(radial))
    horizontal_speed = horizontal_velocity.norm()
    if horizontal_speed <= 1.0e-9:
        heading_error = 0.0
    else:
        velocity_direction = horizontal_velocity.scaled(1.0 / horizontal_speed)
        heading_error = math.atan2(radial.dot(tangent.cross(velocity_direction)), tangent.dot(velocity_direction))
    return {
        "route_cross_track_error_m": earth_radius * cross_track_angle,
        "route_heading_error_deg": math.degrees(heading_error),
    }
####


def _rigid_body_aerodynamic_model(
    problem: Problem,
    tables: Mapping[str, RuntimeTable],
    earth_mu: float,
    earth_omega: float,
    control_values: Mapping[str, float] = {},
    *,
    target_attributes: Mapping[str, str] = {},
    guidance_attributes: Mapping[str, str] = {},
    route_attributes: Mapping[str, str] = {},
    vehicle_attributes: Mapping[str, str] = {},
    actuator_attributes: Mapping[str, str] = {},
    reference_area: float = 1.0,
    reference_length: float = 1.0,
    aero_load_mode: str = "coefficient",
    aero_wrench_frame: str = "taoryx",
    alpha_reference_degrees: float = 0.0,
    parameters: Mapping[str, float] = {},
    wind_blocks: Sequence[WindBlock] = (),
    rotor_allocation: QuadRotorAllocation | None = None,
    rate_controller: GainScheduledLqrController | None = None,
    inertia_provider: Callable[[RigidBody6DofState], Vector3] | None = None,
    controller_state_provider: Callable[[], object | None] | None = None,
) -> TableAerodynamicModel | DirectWrenchTableModel | None:
    """Build the explicit TAORYX table-aero bridge for rigid-body cases.

    This is intentionally opt-in by table content: a rigid-body problem with
    no complete ``cx/cy/cz`` family retains its propulsion-only lowering. The
    bridge gives extension problems a deterministic atmosphere, ECIC/ECFC
    conversion, force/moment lookup, and named control-surface inputs.
    """

    aero_assignments = {
        assignment.name.casefold(): assignment.value
        for trajectory in problem.trajectories
        for segment in trajectory.segments
        for block in segment.blocks
        if isinstance(block, AeroBlock)
        for assignment in block.assignments
    }
    if not aero_assignments:
        return None
    force_expressions = {name: aero_assignments.get(name) for name in ("cx", "cy", "cz")}
    if any(expression is None for expression in force_expressions.values()):
        return None
    derived_expressions: dict[str, ExpressionType] = {
        assignment.name.casefold(): assignment.value
        for block in problem.blocks
        if isinstance(block, DefineBlock) and not block.integral
        for assignment in block.assignments
    }
    for trajectory in problem.trajectories:
        derived_expressions.update(
            {
                assignment.name.casefold(): assignment.value
                for block in trajectory.blocks
                if isinstance(block, DefineBlock) and not block.integral
                for assignment in block.assignments
            }
        )
    selected_tables: dict[str, RuntimeTable] = {}
    requested_names = {name for name in ("cx", "cy", "cz", "cmx", "cmy", "cmz") if name in aero_assignments}
    for alias, table in tables.items():
        output_name = table.output_variable.casefold()
        if output_name in requested_names:
            selected_tables[alias] = table
    # Preserve the explicit-reference behavior for a uniquely named table,
    # while allowing qualified static/control aliases to compose when a
    # vehicle supplies several coefficient families.
    for name in requested_names:
        reference = aero_assignments.get(name)
        if isinstance(reference, TableReferenceExpression):
            candidate = tables.get(reference.name.casefold())
            if candidate is not None:
                selected_tables.setdefault(reference.name.casefold(), candidate)
    coefficients = PreparedAerodynamicCoefficients.from_runtime_tables(cast(Mapping[str, Any], selected_tables)) if selected_tables else None
    earth = EarthModel(
        Quantity(6_378_137.0, Unit.METER),
        0.0,
        Quantity(max(earth_mu, 1.0), Unit.METER_CUBED_PER_SECOND_SQUARED),
        Quantity(earth_omega, Unit.RADIAN_PER_SECOND),
    )
    atmosphere = ExponentialAtmosphereProvider(
        earth.equatorial_radius.si_value,
        wind=FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
    )
    environment = _rigid_body_environment(atmosphere, wind_blocks, parameters=parameters, tables=tables)
    resolved_area = next((table.reference_area for table in tables.values() if table.reference_area is not None), reference_area)
    default_inertia = Vector3(
        float(vehicle_attributes.get("inertia-x", "1.0")),
        float(vehicle_attributes.get("inertia-y", "1.0")),
        float(vehicle_attributes.get("inertia-z", "1.0")),
    )

    target_position = _runtime_target_position(target_attributes, earth, EarthRotationAdapter(earth), 0.0)
    navigation_gain = float(guidance_attributes.get("propnav-gain", "0.0"))
    rotorcraft_guidance = aero_load_mode.casefold() == "direct-wrench" and rotor_allocation is not None
    control_state = cast(dict[str, float], control_values)
    segment_blocks = {
        segment.number: segment.blocks
        for trajectory in problem.trajectories
        for segment in trajectory.segments
    }

    def controls(state: RigidBody6DofState) -> dict[str, float]:
        selected = None if controller_state_provider is None else controller_state_provider()
        if isinstance(selected, RigidBody6DofState):
            state = selected
        values = dict(control_values)

        def apply_surface_allocation_override() -> None:
            """Apply a probe or achieved surface command after hold laws."""

            for degree_name, radian_name in (
                ("elevator-deg", "elevator"),
                ("aileron-deg", "aileron"),
                ("collective-elevon-deg", "collective_elevon"),
                ("differential-elevon-deg", "differential_elevon"),
                ("symmetric-stabilator-deg", "symmetric_stabilator"),
                ("differential-stabilator-deg", "differential_stabilator"),
                ("rudder-deg", "rudder"),
            ):
                override_name = f"_surface-allocation-override-{degree_name}"
                if override_name not in control_values:
                    continue
                override = float(control_values[override_name])
                values[degree_name] = override
                values[radian_name] = math.radians(override)
            ####
        ####

        # The grammar-facing controls are degree-labelled, while coefficient
        # tables use the canonical radian ``alpha``/``bank`` axes. Keep both
        # names in the query context rather than making table data guess.
        if "alpha-deg" in values:
            values["alpha"] = math.radians(values["alpha-deg"])
        if "bank-deg" in values:
            values["bank"] = math.radians(values["bank-deg"])
        # Bank is an optional extension axis.  A table may still declare it
        # even when the problem supplies no bank control; the neutral value is
        # the documented zero-bank default.
        values.setdefault("bank", 0.0)
        values.setdefault("fin_pitch", 0.0)
        values.setdefault("symmetric_stabilator", math.radians(values.get("symmetric-stabilator-deg", 0.0)))
        values.setdefault("differential_stabilator", math.radians(values.get("differential-stabilator-deg", 0.0)))
        values.setdefault("elevator", math.radians(values.get("elevator-deg", 0.0)))
        values.setdefault("aileron", math.radians(values.get("aileron-deg", 0.0)))
        values.setdefault("rudder", math.radians(values.get("rudder-deg", 0.0)))
        values.setdefault("collective_elevon", math.radians(values.get("collective-elevon-deg", 0.0)))
        values.setdefault("differential_elevon", math.radians(values.get("differential-elevon-deg", 0.0)))
        values.setdefault("rotor_speed", values.get("rotor-speed", 469.124102661955))
        shutdown_time_text = actuator_attributes.get("motor-shutdown-time-s")
        motor_shutdown = shutdown_time_text is not None and state.time >= float(shutdown_time_text)
        if motor_shutdown:
            values["rotor_speed"] = 0.0
            for index in range(1, 5):
                values[f"rotor-{index}-speed"] = 0.0
        values["motor_shutdown"] = 1.0 if motor_shutdown else 0.0
        altitude_hold_gain = float(guidance_attributes.get("altitude-hold-gain-rad-s-per-m", "0.0"))
        altitude_target = target_attributes.get("altitude-m")
        if altitude_hold_gain != 0.0 and altitude_target is not None and not rotorcraft_guidance:
            current_altitude = state.position.vector.norm() - earth.equatorial_radius.si_value
            correction = altitude_hold_gain * (float(altitude_target) - current_altitude)
            correction_limit = abs(float(guidance_attributes.get("altitude-hold-max-delta-rad-s", "100.0")))
            values["rotor_speed"] = max(0.0, min(1500.0, values["rotor_speed"] + max(-correction_limit, min(correction_limit, correction))))
            values["altitude_hold_error_m"] = float(altitude_target) - current_altitude
        if rotor_allocation is not None:
            if rate_controller is not None:
                current_inertia = inertia_provider(state) if inertia_provider is not None else default_inertia
                rate_command = rate_controller.command(
                    {
                        "wx": state.body_rate.x,
                        "wy": state.body_rate.y,
                        "wz": state.body_rate.z,
                    },
                    mass_kg=state.mass,
                    inertia=(current_inertia.x, current_inertia.y, current_inertia.z),
                )
                canonical_moment = Vector3(
                    rate_command.controls["moment-x"],
                    rate_command.controls["moment-y"],
                    rate_command.controls["moment-z"],
                )
                if aero_wrench_frame.casefold() == "source-z-up":
                    requested_moment = Vector3(
                        -canonical_moment.x,
                        -canonical_moment.y,
                        canonical_moment.z,
                    )
                else:
                    requested_moment = canonical_moment
                commands = rotor_allocation.allocate(float(values["rotor_speed"]), requested_moment)
                for index, speed in enumerate(commands.values, start=1):
                    values[f"rotor-{index}-speed"] = speed
                control_state["rate_lqr_request_moment_x_nm"] = canonical_moment.x
                control_state["rate_lqr_request_moment_y_nm"] = canonical_moment.y
                control_state["rate_lqr_request_moment_z_nm"] = canonical_moment.z
                control_state["rate_lqr_saturated"] = float(bool(rate_command.saturated))
                control_state["rotor_command_saturated"] = float(
                    any(
                        speed in {rotor_allocation.minimum_speed_rad_s, rotor_allocation.maximum_speed_rad_s}
                        for speed in commands.values
                    )
                )
            else:
                rotor_rate_damping = float(guidance_attributes.get("rotor-rate-damping-nm-s-per-rad", "0.0"))
            if rate_controller is None and rotor_rate_damping > 0.0:
                source_sign = 1.0 if aero_wrench_frame.casefold() == "source-z-up" else -1.0
                requested_moment = Vector3(
                    source_sign * rotor_rate_damping * state.body_rate.x,
                    source_sign * rotor_rate_damping * state.body_rate.y,
                    -source_sign * rotor_rate_damping * state.body_rate.z,
                )
                commands = rotor_allocation.allocate(float(values["rotor_speed"]), requested_moment)
                for index, speed in enumerate(commands.values, start=1):
                    values[f"rotor-{index}-speed"] = speed
                values["rotor_command_saturated"] = float(
                    any(speed in {rotor_allocation.minimum_speed_rad_s, rotor_allocation.maximum_speed_rad_s} for speed in commands.values)
                )
            if rotorcraft_guidance and target_position is not None:
                # Rotorcraft use the source +Z thrust axis, not the fixed-wing
                # body-X route axis.  Build a bounded acceleration demand from
                # local position/velocity error and align the actual thrust
                # direction with it through the ordinary rigid-body moment
                # path.  The coefficients and rotor allocator remain the only
                # force-producing path.
                radial = state.position.vector.scaled(1.0 / max(state.position.vector.norm(), 1.0e-12))
                target_ecic = _runtime_rectangle_waypoint_position(route_attributes, state)
                if target_ecic is None:
                    target_ecic = _runtime_target_position(target_attributes, earth, EarthRotationAdapter(earth), state.time)
                if target_ecic is None:
                    return values
                position_error = target_ecic - state.position.vector
                horizontal_error = position_error - radial.scaled(position_error.dot(radial))
                horizontal_velocity = state.velocity.vector - radial.scaled(state.velocity.vector.dot(radial))
                horizontal_gain = max(0.0, float(guidance_attributes.get("rotorcraft-waypoint-gain-mps2-per-m", "1.0")))
                horizontal_damping = max(0.0, float(guidance_attributes.get("rotorcraft-velocity-damping-per-s", "1.5")))
                waypoint_altitude = target_ecic.norm() - earth.equatorial_radius.si_value
                altitude_error = waypoint_altitude - (state.position.vector.norm() - earth.equatorial_radius.si_value)
                radial_speed = state.velocity.vector.dot(radial)
                altitude_gain = max(0.0, float(guidance_attributes.get("rotorcraft-altitude-gain-mps2-per-m", "1.5")))
                altitude_damping = max(0.0, float(guidance_attributes.get("rotorcraft-altitude-damping-per-s", "2.0")))
                demanded_acceleration = (
                    horizontal_error.scaled(horizontal_gain)
                    - horizontal_velocity.scaled(horizontal_damping)
                    + radial.scaled(altitude_gain * altitude_error - altitude_damping * radial_speed)
                )
                gravity_ecic = state.position.vector.scaled(
                    -earth_mu / max(state.position.vector.norm() ** 3, 1.0)
                )
                desired_force_body = state.attitude.conjugate().rotate(
                    gravity_ecic.scaled(-state.mass) + demanded_acceleration.scaled(state.mass)
                )
                desired_force_norm = max(desired_force_body.norm(), 1.0e-12)
                desired_thrust_body = desired_force_body.scaled(1.0 / desired_force_norm)
                # Direct-wrench source +Z becomes canonical body -Z after the
                # frame adapter.  Cross-product ordering is therefore chosen
                # against the canonical thrust direction, then passed through
                # the normal bounded moment controller.
                thrust_axis_body = Vector3(0.0, 0.0, -1.0)
                rotor_attitude_error = thrust_axis_body.cross(desired_thrust_body)
                maximum_moment_text = actuator_attributes.get("maximum-moment", guidance_attributes.get("maximum-moment"))
                maximum_moment = float(maximum_moment_text) if maximum_moment_text is not None else None
                maximum_body_rate_text = actuator_attributes.get("maximum-body-rate-deg-s")
                maximum_body_rate = math.radians(float(maximum_body_rate_text)) if maximum_body_rate_text is not None else None
                rotor_controller = bounded_attitude_moment(
                    rotor_attitude_error,
                    state.body_rate,
                    attitude_gain=float(guidance_attributes.get("rotorcraft-attitude-gain-nm-per-rad", "0.05")),
                    rate_damping=(
                        0.0
                        if rate_controller is not None
                        else float(guidance_attributes.get("rotorcraft-rate-damping-nm-s-per-rad", "0.01"))
                    ),
                    maximum_moment=maximum_moment,
                    maximum_body_rate=maximum_body_rate,
                )
                control_state["_rotor_guidance_moment_x"] = rotor_controller.moment_body.x
                control_state["_rotor_guidance_moment_y"] = rotor_controller.moment_body.y
                control_state["_rotor_guidance_moment_z"] = rotor_controller.moment_body.z
                control_state["_rotor_controller_saturated"] = float(rotor_controller.saturated)
                yaw_target_text = guidance_attributes.get("rotorcraft-yaw-target-deg")
                if yaw_target_text is not None:
                    actual_heading_deg = _rigid_body_local_attitude_observables(state, EarthRotationAdapter(earth))["local_heading_deg"]
                    yaw_target_deg = float(yaw_target_text)
                    yaw_error = math.atan2(
                        math.sin(math.radians(yaw_target_deg - actual_heading_deg)),
                        math.cos(math.radians(yaw_target_deg - actual_heading_deg)),
                    )
                    yaw_moment = (
                        float(guidance_attributes.get("rotorcraft-yaw-gain-nm-per-rad", "0.05")) * yaw_error
                        - float(guidance_attributes.get("rotorcraft-yaw-rate-damping-nm-s-per-rad", "0.01")) * state.body_rate.z
                    )
                    yaw_moment_limit_text = guidance_attributes.get("rotorcraft-yaw-maximum-moment", maximum_moment_text)
                    yaw_moment_limit = None if yaw_moment_limit_text is None else abs(float(yaw_moment_limit_text))
                    if yaw_moment_limit is not None:
                        yaw_moment = max(-yaw_moment_limit, min(yaw_moment_limit, yaw_moment))
                    control_state["_rotor_guidance_moment_z"] += yaw_moment
                    control_state["rotorcraft_yaw_target_deg"] = yaw_target_deg
                    control_state["rotorcraft_yaw_achieved_deg"] = actual_heading_deg
                    control_state["rotorcraft_yaw_error_deg"] = math.degrees(yaw_error)
                    control_state["rotorcraft_yaw_moment_nm"] = yaw_moment
                    control_state["rotorcraft_yaw_control_active"] = 1.0
                else:
                    control_state["rotorcraft_yaw_control_active"] = 0.0
                collective_gain = float(guidance_attributes.get("rotorcraft-collective-gain-rad-s-per-mps2", "20.0"))
                tilt_compensation = max(0.0, desired_force_norm / max(abs(gravity_ecic.norm() * state.mass), 1.0e-12) - 1.0)
                values["rotor_speed"] = max(
                    0.0,
                    min(1500.0, values["rotor_speed"] + collective_gain * (altitude_gain * altitude_error - altitude_damping * radial_speed + tilt_compensation)),
                )
        alpha_hold_gain = float(guidance_attributes.get("alpha-hold-gain-deg-per-deg", "0.0"))
        if alpha_hold_gain != 0.0 and "elevator-deg" in values:
            position_ecfc, _ = EarthRotationAdapter(earth).ecic_to_ecfc(state.position, state.velocity, time_seconds=state.time)
            sample = environment.sample(time=state.time, position=position_ecfc)
            air_velocity_ecic = EarthRotationAdapter(earth).air_relative_velocity_ecic(
                state.position, state.velocity, sample.wind, time_seconds=state.time,
            ).vector
            velocity_body = state.attitude.conjugate().rotate(air_velocity_ecic)
            actual_alpha_deg = math.degrees(math.atan2(velocity_body.z, max(abs(velocity_body.x), 1.0e-12)))
            target_alpha_deg = float(guidance_attributes.get("alpha-hold-target-deg", str(alpha_reference_degrees)))
            rectangle_bank = _runtime_rectangle_bank_angle(route_attributes, state)
            lift_compensation = float(guidance_attributes.get("rectangle-lift-compensation-deg", "0.0"))
            if rectangle_bank is not None and lift_compensation != 0.0:
                target_alpha_deg += lift_compensation * (1.0 / max(math.cos(abs(rectangle_bank)), 1.0e-6) - 1.0)
            base_elevator_deg = float(values.get("elevator-deg", 0.0))
            commanded_elevator_deg = base_elevator_deg + alpha_hold_gain * (target_alpha_deg - actual_alpha_deg)
            lower = float(guidance_attributes.get("elevator-min-deg", "-10.0"))
            upper = float(guidance_attributes.get("elevator-max-deg", "10.0"))
            values["elevator-deg"] = min(upper, max(lower, commanded_elevator_deg))
            values["elevator"] = math.radians(values["elevator-deg"])
            values["elevator_controller_saturated"] = float(commanded_elevator_deg < lower or commanded_elevator_deg > upper)
        if alpha_hold_gain != 0.0 and "symmetric-stabilator-deg" in values and "elevator-deg" not in values:
            # Reuse the documented alpha-hold contract for source decks whose
            # pitch actuator is a symmetric stabilator rather than an
            # elevator.  The selected actuator is determined by the problem
            # file's declared control, not by a vehicle-family special case.
            position_ecfc, _ = EarthRotationAdapter(earth).ecic_to_ecfc(state.position, state.velocity, time_seconds=state.time)
            sample = environment.sample(time=state.time, position=position_ecfc)
            air_velocity_ecic = EarthRotationAdapter(earth).air_relative_velocity_ecic(
                state.position, state.velocity, sample.wind, time_seconds=state.time,
            ).vector
            velocity_body = state.attitude.conjugate().rotate(air_velocity_ecic)
            actual_alpha_deg = math.degrees(math.atan2(velocity_body.z, max(abs(velocity_body.x), 1.0e-12)))
            target_alpha_deg = float(guidance_attributes.get("alpha-hold-target-deg", str(alpha_reference_degrees)))
            base_stabilator_deg = float(values.get("symmetric-stabilator-deg", 0.0))
            commanded_stabilator_deg = base_stabilator_deg + alpha_hold_gain * (target_alpha_deg - actual_alpha_deg)
            lower = float(guidance_attributes.get("symmetric-stabilator-min-deg", "-14.9"))
            upper = float(guidance_attributes.get("symmetric-stabilator-max-deg", "34.9"))
            values["symmetric-stabilator-deg"] = min(upper, max(lower, commanded_stabilator_deg))
            values["symmetric_stabilator"] = math.radians(values["symmetric-stabilator-deg"])
            values["symmetric_stabilator_controller_saturated"] = float(commanded_stabilator_deg < lower or commanded_stabilator_deg > upper)
        terminal_capture_active = (
            route_attributes.get("terminal-capture", "false").casefold() in {"1", "true", "yes"}
            and state.time >= float(route_attributes.get("terminal-start-s", "inf"))
        )
        elevon_hold_gain = float(guidance_attributes.get("collective-elevon-hold-gain-deg-per-deg", "0.0"))
        if elevon_hold_gain != 0.0 and not terminal_capture_active:
            position_ecfc, _ = EarthRotationAdapter(earth).ecic_to_ecfc(state.position, state.velocity, time_seconds=state.time)
            sample = environment.sample(time=state.time, position=position_ecfc)
            air_velocity_ecic = EarthRotationAdapter(earth).air_relative_velocity_ecic(
                state.position, state.velocity, sample.wind, time_seconds=state.time,
            ).vector
            velocity_body = state.attitude.conjugate().rotate(air_velocity_ecic)
            actual_alpha_deg = math.degrees(math.atan2(velocity_body.z, max(abs(velocity_body.x), 1.0e-12)))
            target_alpha_deg = float(guidance_attributes.get("collective-elevon-hold-target-deg", "7.9"))
            base_elevon_deg = float(values.get("collective-elevon-deg", 0.0))
            commanded_elevon_deg = base_elevon_deg + elevon_hold_gain * (target_alpha_deg - actual_alpha_deg)
            lower = float(guidance_attributes.get("collective-elevon-min-deg", "-20.0"))
            upper = float(guidance_attributes.get("collective-elevon-max-deg", "20.0"))
            values["collective-elevon-deg"] = min(upper, max(lower, commanded_elevon_deg))
            values["collective_elevon"] = math.radians(values["collective-elevon-deg"])
            values["collective_elevon_controller_saturated"] = float(commanded_elevon_deg < lower or commanded_elevon_deg > upper)
        sideslip_hold_gain = float(guidance_attributes.get("differential-elevon-hold-gain-deg-per-deg", "0.0"))
        if sideslip_hold_gain != 0.0 and "differential-elevon-deg" in values and not terminal_capture_active:
            position_ecfc, _ = EarthRotationAdapter(earth).ecic_to_ecfc(state.position, state.velocity, time_seconds=state.time)
            sample = environment.sample(time=state.time, position=position_ecfc)
            air_velocity_ecic = EarthRotationAdapter(earth).air_relative_velocity_ecic(
                state.position, state.velocity, sample.wind, time_seconds=state.time,
            ).vector
            velocity_body = state.attitude.conjugate().rotate(air_velocity_ecic)
            actual_beta_deg = math.degrees(math.atan2(velocity_body.y, max(math.hypot(velocity_body.x, velocity_body.z), 1.0e-12)))
            target_beta_deg = float(guidance_attributes.get("differential-elevon-hold-target-deg", "0.0"))
            base_differential_deg = float(values.get("differential-elevon-deg", 0.0))
            commanded_differential_deg = base_differential_deg + sideslip_hold_gain * (target_beta_deg - actual_beta_deg)
            lower = float(guidance_attributes.get("differential-elevon-min-deg", "-20.0"))
            upper = float(guidance_attributes.get("differential-elevon-max-deg", "20.0"))
            values["differential-elevon-deg"] = min(upper, max(lower, commanded_differential_deg))
            values["differential_elevon"] = math.radians(values["differential-elevon-deg"])
            values["differential_elevon_controller_saturated"] = float(commanded_differential_deg < lower or commanded_differential_deg > upper)
        differential_stabilator_gain = float(
            guidance_attributes.get("differential-stabilator-hold-gain-deg-per-deg", str(sideslip_hold_gain))
        )
        if (
            differential_stabilator_gain != 0.0
            and "differential-stabilator-deg" in values
            and "differential-elevon-deg" not in values
            and not terminal_capture_active
        ):
            # The same sideslip-hold idea applies to a differential
            # stabilator.  A separate target name is accepted so legacy
            # differential-elevon fragments remain reusable unchanged.
            position_ecfc, _ = EarthRotationAdapter(earth).ecic_to_ecfc(state.position, state.velocity, time_seconds=state.time)
            sample = environment.sample(time=state.time, position=position_ecfc)
            air_velocity_ecic = EarthRotationAdapter(earth).air_relative_velocity_ecic(
                state.position, state.velocity, sample.wind, time_seconds=state.time,
            ).vector
            velocity_body = state.attitude.conjugate().rotate(air_velocity_ecic)
            actual_beta_deg = math.degrees(math.atan2(velocity_body.y, max(math.hypot(velocity_body.x, velocity_body.z), 1.0e-12)))
            target_beta_deg = float(
                guidance_attributes.get(
                    "differential-stabilator-hold-target-deg",
                    guidance_attributes.get("differential-elevon-hold-target-deg", "0.0"),
                )
            )
            base_differential_deg = float(values.get("differential-stabilator-deg", 0.0))
            commanded_differential_deg = base_differential_deg + differential_stabilator_gain * (target_beta_deg - actual_beta_deg)
            lower = float(guidance_attributes.get("differential-stabilator-min-deg", "-20.05"))
            upper = float(guidance_attributes.get("differential-stabilator-max-deg", "20.05"))
            values["differential-stabilator-deg"] = min(upper, max(lower, commanded_differential_deg))
            values["differential_stabilator"] = math.radians(values["differential-stabilator-deg"])
            values["differential_stabilator_controller_saturated"] = float(commanded_differential_deg < lower or commanded_differential_deg > upper)
        active_segment_number = int(values.get("_segment", min(segment_blocks, default=1)))
        active_segment_guidance = any(
            isinstance(block, FlyBlock)
            and (block.guidance_variable or "").casefold() in {"propnav", "intercept"}
            for block in segment_blocks.get(active_segment_number, ())
        )
        if target_position is None or navigation_gain <= 0.0 or not (active_segment_guidance or _runtime_pure_propnav_active(route_attributes, state.time)):
            apply_surface_allocation_override()
            return values
        target_ecic = _runtime_target_position(target_attributes, earth, EarthRotationAdapter(earth), state.time)
        if target_ecic is None:
            apply_surface_allocation_override()
            return values
        demand = cast(Vector3, _runtime_propnav_command(state, target_attributes, navigation_gain, earth_mu, earth_omega)["demand"])
        allocation = allocate_alpha_bank(
            demand,
            lift_acceleration_per_radian=float(guidance_attributes.get("lift-acceleration-per-radian", "100.0")),
            maximum_angle_of_attack_radians=math.radians(float(guidance_attributes.get("max-alpha-deg", "20.0"))),
            maximum_bank_radians=math.radians(abs(float(guidance_attributes.get("max-bank-deg", "180.0")))),
        )
        values["alpha-deg"] = math.degrees(allocation.angle_of_attack_radians)
        values["bank-deg"] = math.degrees(allocation.bank_radians)
        values["alpha"] = allocation.angle_of_attack_radians
        values["bank"] = allocation.bank_radians
        values["fin_pitch"] = allocation.angle_of_attack_radians / math.radians(float(guidance_attributes.get("max-alpha-deg", "20.0")))
        values["pro-nav-acceleration-m-s2"] = demand.norm()
        if guidance_attributes.get("energy-management", "").casefold() == "alpha-drag":
            position_ecfc, _ = EarthRotationAdapter(earth).ecic_to_ecfc(
                state.position,
                state.velocity,
                time_seconds=state.time,
            )
            sample = environment.sample(time=state.time, position=position_ecfc)
            air_velocity_ecic = EarthRotationAdapter(earth).air_relative_velocity_ecic(
                state.position,
                state.velocity,
                sample.wind,
                time_seconds=state.time,
            ).vector
            airspeed = air_velocity_ecic.norm()
            target_speed = float(guidance_attributes.get("energy-target-speed-mps", "0.0"))
            alpha_gain = math.radians(float(guidance_attributes.get("energy-alpha-gain-deg-per-mps", "0.01")))
            energy_alpha_limit = math.radians(float(guidance_attributes.get("energy-max-alpha-deg", "20.0")))
            energy_alpha = max(0.0, (airspeed - target_speed) * alpha_gain) if target_speed > 0.0 else 0.0
            values["alpha"] = min(energy_alpha_limit, max(allocation.angle_of_attack_radians, energy_alpha))
            values["alpha-deg"] = math.degrees(values["alpha"])
            values["fin_pitch"] = values["alpha"] / max(energy_alpha_limit, 1.0e-12)
            values["energy_speed_mps"] = airspeed
            values["energy_alpha_deg"] = values["alpha-deg"]
        apply_surface_allocation_override()
        return values
    ####

    if aero_load_mode.casefold() == "direct-wrench":
        if coefficients is None:
            raise ValueError("direct-wrench aerodynamic mode requires direct runtime coefficient tables")
        return DirectWrenchTableModel(
            environment,
            EarthRotationAdapter(earth),
            coefficients,
            controls,
            source_z_up=aero_wrench_frame.casefold() == "source-z-up",
            rotor_allocation=rotor_allocation,
            model_uncertainty_fraction=float(vehicle_attributes.get("aero-uncertainty-fraction", "0.0")),
            source_quality=vehicle_attributes.get("aero-source-quality", "estimated"),
        )
    # Aerodynamic source decks are evidence-bounded.  Do not let their
    # ``no-extrap`` axes silently clamp during force/moment evaluation.
    evaluators = _strict_table_evaluators(tables)

    def resolve_derived(name: str, values: Mapping[str, float], active: frozenset[str] = frozenset()) -> float:
        key = name.casefold()
        if key not in derived_expressions:
            raise KeyError(f"undefined variable: {name}")
        if key in active:
            raise ValueError(f"cyclic derived aerodynamic definition involving {name!r}")
        return evaluate_expression(
            derived_expressions[key],
            values,
            parameters,
            resolver=lambda nested: resolve_derived(nested, values, active | {key}),
            tables=evaluators,
        )
    ####

    def evaluate_force(context: AeroQueryContext) -> Vector3:
        values = context.values
        return Vector3(*(evaluate_expression(cast(ExpressionType, force_expressions[name]), values, parameters, resolver=lambda name: resolve_derived(name, values), tables=evaluators) for name in ("cx", "cy", "cz")))
    ####

    moment_expressions = {name: aero_assignments.get(name) for name in ("cmx", "cmy", "cmz")}
    has_complete_moment_expression = all(expression is not None for expression in moment_expressions.values())

    def evaluate_moment(context: AeroQueryContext) -> Vector3:
        values = context.values
        return Vector3(*(evaluate_expression(cast(ExpressionType, moment_expressions[name]), values, parameters, resolver=lambda name: resolve_derived(name, values), tables=evaluators) for name in ("cmx", "cmy", "cmz")))
    ####

    simple_force_references = all(isinstance(force_expressions[name], TableReferenceExpression) for name in ("cx", "cy", "cz"))
    simple_moment_references = all(isinstance(moment_expressions[name], TableReferenceExpression) for name in ("cmx", "cmy", "cmz"))

    fallback_force_provider: Callable[[float, float, float], Vector3]
    if coefficients is not None:
        fallback_force_provider = coefficients.force_provider()
    else:
        fallback_force_provider = lambda _mach, _alpha, _beta: Vector3(0.0, 0.0, 0.0)
    fallback_moment_provider: Callable[[float, float, float], Vector3] | None
    if coefficients is not None and not has_complete_moment_expression:
        fallback_moment_provider = coefficients.moment_provider()
    else:
        fallback_moment_provider = None

    return TableAerodynamicModel(
        environment,
        EarthRotationAdapter(earth),
        resolved_area,
        reference_length,
        fallback_force_provider,
        moment_coefficients=fallback_moment_provider,
        context_coefficients=(
            coefficients.context_force_provider()
            if coefficients is not None and coefficients.control_force_tables and simple_force_references
            else evaluate_force
        ),
        context_moment_coefficients=(
            coefficients.context_moment_provider()
            if coefficients is not None and coefficients.control_moment_tables and has_complete_moment_expression and simple_moment_references
            else evaluate_moment
            if has_complete_moment_expression
            else None
        ),
        control_provider=controls,
        alpha_reference_rad=math.radians(alpha_reference_degrees),
        table_margin_provider=coefficients.table_margins if coefficients is not None else lambda values: _runtime_table_margins(tables, values),
        table_normalized_margin_provider=(
            coefficients.table_normalized_margins
            if coefficients is not None
            else lambda values: _runtime_table_normalized_margins(tables, values)
        ),
        model_uncertainty_fraction=float(vehicle_attributes.get("aero-uncertainty-fraction", "0.0")),
        source_quality=vehicle_attributes.get("aero-source-quality", "estimated"),
    )
####


def _rigid_body_environment(
    atmosphere: ExponentialAtmosphereProvider,
    wind_blocks: Sequence[WindBlock],
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
) -> ExponentialAtmosphereProvider | WindFieldEnvironmentProvider:
    """Build the generic rigid-body atmosphere/wind bridge from ``*wind``."""

    if not wind_blocks:
        return atmosphere
    block = wind_blocks[-1]
    assignments = {assignment.name.casefold(): assignment.value for assignment in block.assignments}
    evaluators = _table_evaluators(tables)

    def resolve(time: float, position: FrameVector3) -> FrameVector3:
        values = {"time": time, "x": position.vector.x, "y": position.vector.y, "z": position.vector.z}
        if "winde" in assignments or "windn" in assignments:
            east = evaluate_expression(assignments.get("winde", NumberExpression(value=0.0)), values, parameters, tables=evaluators)
            north = evaluate_expression(assignments.get("windn", NumberExpression(value=0.0)), values, parameters, tables=evaluators)
            down = evaluate_expression(assignments.get("windd", NumberExpression(value=0.0)), values, parameters, tables=evaluators)
            return evaluate_wind(east=east, north=north, down=down, longitude=math.atan2(position.vector.y, position.vector.x), latitude=math.asin(max(-1.0, min(1.0, position.vector.z / max(position.vector.norm(), 1.0))))).ecfc
        speed = evaluate_expression(assignments.get("windv", NumberExpression(value=0.0)), values, parameters, tables=evaluators)
        heading = math.radians(evaluate_expression(assignments.get("windh", NumberExpression(value=0.0)), values, parameters, tables=evaluators))
        down = math.radians(evaluate_expression(assignments.get("windd", NumberExpression(value=0.0)), values, parameters, tables=evaluators))
        return evaluate_wind(magnitude=speed, heading=heading, down=down, longitude=math.atan2(position.vector.y, position.vector.x), latitude=math.asin(max(-1.0, min(1.0, position.vector.z / max(position.vector.norm(), 1.0))))).ecfc
    ####

    return WindFieldEnvironmentProvider(atmosphere, resolve)
####


def _runtime_target_position(
    attributes: Mapping[str, str],
    earth: EarthModel,
    adapter: EarthRotationAdapter,
    time_seconds: float,
) -> Vector3 | None:
    """Resolve a declared fixed ground target into ECIC coordinates."""

    if "latitude-deg" not in attributes or "longitude-deg" not in attributes:
        return None
    latitude = math.radians(float(attributes["latitude-deg"]))
    longitude = math.radians(float(attributes["longitude-deg"]))
    altitude = float(attributes.get("altitude-m", "0.0"))
    radius = earth.equatorial_radius.si_value + altitude
    ecfc = FrameVector3(
        Vector3(
            radius * math.cos(latitude) * math.cos(longitude),
            radius * math.cos(latitude) * math.sin(longitude),
            radius * math.sin(latitude),
        ),
        Frame.ECFC,
    )
    return adapter.ecfc_to_ecic(ecfc, FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC), time_seconds=time_seconds)[0].vector
####


def _runtime_propnav_command(
    state: RigidBody6DofState,
    target_attributes: Mapping[str, str],
    navigation_gain: float,
    earth_mu: float,
    earth_omega: float,
) -> dict[str, float | Vector3]:
    """Evaluate one native ECIC ProNav command and its LOS observables."""

    earth = EarthModel(
        Quantity(6_378_137.0, Unit.METER),
        0.0,
        Quantity(max(earth_mu, 1.0), Unit.METER_CUBED_PER_SECOND_SQUARED),
        Quantity(earth_omega, Unit.RADIAN_PER_SECOND),
    )
    target = _runtime_target_position(target_attributes, earth, EarthRotationAdapter(earth), state.time)
    if target is None or navigation_gain <= 0.0:
        return {
            "demand": Vector3(0.0, 0.0, 0.0),
            "range": 0.0,
            "closing": 0.0,
            "azimuth": 0.0,
            "elevation": 0.0,
            "azimuth_rate": 0.0,
            "elevation_rate": 0.0,
        }
    target_velocity = Vector3(0.0, 0.0, earth_omega).cross(target)
    relative = target - state.position.vector
    relative_velocity = target_velocity - state.velocity.vector
    range_m = max(relative.norm(), 1.0e-12)
    horizontal = math.hypot(relative.x, relative.y)
    horizontal_rate = (relative.x * relative_velocity.y - relative.y * relative_velocity.x) / max(horizontal * horizontal, 1.0e-12)
    horizontal_rate_projection = (relative.x * relative_velocity.x + relative.y * relative_velocity.y) / max(horizontal, 1.0e-12)
    elevation_rate = (horizontal * relative_velocity.z - relative.z * horizontal_rate_projection) / max(range_m * range_m, 1.0e-12)
    navigation = proportional_navigation(
        CartesianVector3(state.position.vector.x, state.position.vector.y, state.position.vector.z),
        CartesianVector3(state.velocity.vector.x, state.velocity.vector.y, state.velocity.vector.z),
        CartesianVector3(target.x, target.y, target.z),
        CartesianVector3(target_velocity.x, target_velocity.y, target_velocity.z),
        navigation_gain,
    )
    return {
        "demand": Vector3(navigation.ecfc_acceleration.x, navigation.ecfc_acceleration.y, navigation.ecfc_acceleration.z),
        "range": range_m,
        "closing": -relative.dot(relative_velocity) / range_m,
        "azimuth": math.atan2(relative.y, relative.x),
        "elevation": math.atan2(relative.z, max(horizontal, 1.0e-12)),
        "azimuth_rate": horizontal_rate,
        "elevation_rate": elevation_rate,
    }
####


def _runtime_pure_propnav_active(route_attributes: Mapping[str, str], state_time: float) -> bool:
    """Return whether terminal attitude steering is owned by pure ProNav."""

    mode = route_attributes.get("terminal-guidance-mode", "route").casefold()
    terminal_start = float(route_attributes.get("terminal-start-s", "1650.0"))
    return mode in {"propnav", "pure-propnav"} and state_time >= terminal_start
####


def _segment_uses_guidance(segment: Segment, guidance_name: str) -> bool:
    """Return whether a standard segment ``*fly`` selects a guidance law."""

    expected = guidance_name.casefold()
    return any(
        isinstance(block, FlyBlock)
        and (block.guidance_variable or "").casefold() == expected
        for block in segment.blocks
    )
####


def _rigid_body_guidance_observables(
    state: RigidBody6DofState,
    target_attributes: Mapping[str, str],
    guidance_attributes: Mapping[str, str],
    earth_mu: float,
    earth_omega: float,
) -> dict[str, float]:
    """Publish native ProNav demand and allocation channels for one state."""

    result = {
        "pro_nav_active": 0.0,
        "pro_nav_acceleration_m_s2": 0.0,
        "pro_nav_command_ecfc_x_m_s2": 0.0,
        "pro_nav_command_ecfc_y_m_s2": 0.0,
        "pro_nav_command_ecfc_z_m_s2": 0.0,
        "pro_nav_los_range_m": 0.0,
        "pro_nav_closing_velocity_m_s": 0.0,
        "pro_nav_los_azimuth_deg": 0.0,
        "pro_nav_los_elevation_deg": 0.0,
        "pro_nav_los_azimuth_rate_deg_s": 0.0,
        "pro_nav_los_elevation_rate_deg_s": 0.0,
        "alpha_command_deg": 0.0,
        "bank_command_deg": 0.0,
        "acceleration_residual_m_s2": 0.0,
    }
    gain = float(guidance_attributes.get("propnav-gain", "0.0"))
    if gain <= 0.0:
        return result
    command = _runtime_propnav_command(state, target_attributes, gain, earth_mu, earth_omega)
    demand = cast(Vector3, command["demand"])
    range_m = float(cast(Any, command["range"]))
    if range_m <= 0.0:
        return result
    allocation = allocate_alpha_bank(
        demand,
        lift_acceleration_per_radian=float(guidance_attributes.get("lift-acceleration-per-radian", "100.0")),
        maximum_angle_of_attack_radians=math.radians(float(guidance_attributes.get("max-alpha-deg", "20.0"))),
        maximum_bank_radians=math.radians(abs(float(guidance_attributes.get("max-bank-deg", "180.0")))),
    )
    result.update(
        {
            "pro_nav_active": 1.0,
            "pro_nav_acceleration_m_s2": demand.norm(),
            "pro_nav_command_ecfc_x_m_s2": demand.x,
            "pro_nav_command_ecfc_y_m_s2": demand.y,
            "pro_nav_command_ecfc_z_m_s2": demand.z,
            "pro_nav_los_range_m": range_m,
            "pro_nav_closing_velocity_m_s": float(cast(Any, command["closing"])),
            "pro_nav_los_azimuth_deg": math.degrees(float(cast(Any, command["azimuth"]))),
            "pro_nav_los_elevation_deg": math.degrees(float(cast(Any, command["elevation"]))),
            "pro_nav_los_azimuth_rate_deg_s": math.degrees(float(cast(Any, command["azimuth_rate"]))),
            "pro_nav_los_elevation_rate_deg_s": math.degrees(float(cast(Any, command["elevation_rate"]))),
            "alpha_command_deg": math.degrees(allocation.angle_of_attack_radians),
            "bank_command_deg": math.degrees(allocation.bank_radians),
            "acceleration_residual_m_s2": allocation.residual_acceleration.norm(),
        }
    )
    return result
####


def _runtime_attributes(problem: Problem, name: str) -> Mapping[str, str]:
    """Resolve one named TAORYX runtime status declaration."""

    attributes: dict[str, str] = {}
    for block in problem.blocks:
        if isinstance(block, RuntimeBlock) and block.declaration == "status" and block.name is not None and block.name.casefold() == name.casefold():
            attributes.update(block.attributes)
    return attributes
    ####


def _build_kinematic_body_rate_provider(problem: Problem) -> Callable[[RuntimeState], Vector3]:
    """Build the generic prescribed/lagged attitude bridge controller.

    The kinematic bridge deliberately commands body rates instead of moments.
    This keeps it between the point-mass and rigid-body models: translation is
    force-integrated, while attitude follows a declared Euler-angle target with
    an optional first-order lag and rate limit.
    """

    attributes = _runtime_attributes(problem, "attitude")
    mode = attributes.get("mode", "prescribed").casefold()
    if mode not in {"prescribed", "lag", "rate"}:
        raise FidelitySetupError(
            "invalid-kinematic-attitude-mode",
            "kinematic attitude mode must be prescribed, lag, or rate",
            "set mode=prescribed, mode=lag, or mode=rate on the attitude runtime status",
            field="attitude.mode",
        )
    target = Vector3(
        math.radians(float(attributes.get("roll-deg", "0.0"))),
        math.radians(float(attributes.get("pitch-deg", "0.0"))),
        math.radians(float(attributes.get("yaw-deg", "0.0"))),
    )
    commanded_rate = Vector3(
        math.radians(float(attributes.get("roll-rate-deg-s", "0.0"))),
        math.radians(float(attributes.get("pitch-rate-deg-s", "0.0"))),
        math.radians(float(attributes.get("yaw-rate-deg-s", "0.0"))),
    )
    lag_s = float(attributes.get("lag-s", "0.25"))
    maximum_rate = math.radians(float(attributes.get("max-rate-deg-s", "360.0")))
    if lag_s <= 0.0 or not math.isfinite(lag_s):
        raise FidelitySetupError(
            "invalid-kinematic-lag",
            "kinematic attitude lag-s must be positive and finite",
            "set lag-s to a positive time constant in seconds; it is used only for prescribed/lag modes",
            field="attitude.lag-s",
        )
    if maximum_rate <= 0.0 or not math.isfinite(maximum_rate):
        raise FidelitySetupError(
            "invalid-kinematic-rate-limit",
            "kinematic attitude max-rate-deg-s must be positive and finite",
            "set max-rate-deg-s to a positive finite angular-rate limit",
            field="attitude.max-rate-deg-s",
        )

    def provider(state: RuntimeState) -> Vector3:
        if mode == "rate":
            return commanded_rate
        current = Quaternion(
            float(state.named.get("qw", 1.0)),
            float(state.named.get("qx", 0.0)),
            float(state.named.get("qy", 0.0)),
            float(state.named.get("qz", 0.0)),
        ).normalized()
        target_attitude = _quaternion_from_euler(target)
        error = target_attitude.multiply(current.conjugate()).normalized()
        sign = -1.0 if error.w < 0.0 else 1.0
        rate = Vector3(
            sign * 2.0 * error.x / lag_s,
            sign * 2.0 * error.y / lag_s,
            sign * 2.0 * error.z / lag_s,
        )
        magnitude = rate.norm()
        if magnitude > maximum_rate:
            rate = rate.scaled(maximum_rate / magnitude)
        return rate
        ####

    return provider
    ####


def _quaternion_from_euler(angles: Vector3) -> Quaternion:
    """Return the body-to-reference quaternion for roll, pitch, yaw radians."""

    half_roll = angles.x * 0.5
    half_pitch = angles.y * 0.5
    half_yaw = angles.z * 0.5
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return Quaternion(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ).normalized()
    ####


def _runtime_lqr_attributes(problem: Problem, name: str) -> dict[str, str]:
    """Resolve one named runtime LQR declaration."""

    for block in problem.blocks:
        if isinstance(block, RuntimeBlock) and block.declaration == "lqr" and block.name is not None and block.name.casefold() == name.casefold():
            return dict(block.attributes)
    return {}
    ####


def _runtime_controller_realization_metadata(controller: object | None) -> dict[str, object]:
    """Expose the active controller contract in the resolved runtime metadata."""

    if controller is None:
        return {"status": "not_configured"}
    realization = getattr(controller, "realization", None)
    if realization is None:
        nominal = getattr(controller, "nominal", None)
        realization = getattr(nominal, "realization", None)
    if realization is None:
        return {"status": "configured_without_realization"}
    return {"status": "configured", **realization.model_dump(mode="json")}
    ####


def _runtime_sensor_clocks(problem: Problem) -> tuple[SensorClockSpec, ...]:
    """Lower declarative sensor clocks without inventing measurements."""

    clocks: list[SensorClockSpec] = []
    for block in problem.blocks:
        if not isinstance(block, RuntimeBlock) or block.declaration != "sensor" or block.name is None:
            continue
        attributes = block.attributes
        clocks.append(
            SensorClockSpec(
                name=block.name,
                kind=attributes["kind"],
                cadence_s=float(attributes["cadence-s"]),
                phase_s=float(attributes.get("phase-s", "0")),
                sample_mode=cast(SensorSampleMode, attributes.get("sample", "instantaneous")),
                delivery_s=float(attributes.get("delivery-s", "0")),
                truth_policy=cast(SensorTruthPolicy, attributes.get("truth", "boundary")),
                rate_policy=cast(SensorRatePolicy, attributes.get("rate-policy", "split")),
            )
        )
    return tuple(clocks)
    ####


def _register_sensor_clock_metadata(runtime: RuntimeProblem) -> None:
    """Expose sensor clocks for inspection and make the execution boundary explicit."""

    runtime.metadata["sensor_clocks"] = [clock.to_metadata() for clock in runtime.sensor_clocks]
    runtime.metadata["sensor_execution"] = (
        "clock-only; measurement providers consume committed boundaries or accepted segments"
        if runtime.sensor_clocks
        else "none"
    )
    ####


def _runtime_model_bindings(problem: Problem) -> list[dict[str, object]]:
    """Expose declarative observation/navigation/feedback references."""

    return [
        {
            "declaration": block.declaration,
            "name": block.name,
            "attributes": dict(block.attributes),
        }
        for block in problem.blocks
        if isinstance(block, RuntimeBlock)
        and block.declaration in {"observation", "navigation", "feedback"}
        and block.name is not None
    ]


def _runtime_inertia_provider(
    vehicle_attributes: Mapping[str, str],
    reference_inertia: Vector3,
    reference_mass_kg: float,
) -> Callable[[RigidBody6DofState], Vector3]:
    """Build an explicit constant or source-declared mass/inertia model.

    Mass loss alone does not determine inertia.  The optional linear model
    therefore requires explicit dry-mass inertia endpoints; it never guesses a
    scaling exponent from total mass.  The endpoint interpolation is a
    declared approximation and should be replaced by a source-backed schedule
    when tank or payload geometry is available.
    """

    mode = vehicle_attributes.get("inertia-mass-model", "constant").casefold()
    if mode in {"constant", "fixed"}:
        return lambda _state: reference_inertia
    if mode not in {"linear-dry-mass", "linear-dry"}:
        raise ValueError(f"unsupported inertia-mass-model {mode!r}")
    dry_mass = float(vehicle_attributes.get("dry-mass-kg", "0.0"))
    if not math.isfinite(dry_mass) or dry_mass <= 0.0 or dry_mass >= reference_mass_kg:
        raise ValueError("linear-dry-mass inertia model requires dry mass below the initial mass")
    dry_values = tuple(
        float(vehicle_attributes[name])
        for name in ("inertia-x-dry", "inertia-y-dry", "inertia-z-dry")
        if name in vehicle_attributes
    )
    if len(dry_values) != 3 or not all(math.isfinite(value) and value > 0.0 for value in dry_values):
        raise ValueError("linear-dry-mass inertia model requires positive inertia-x/y/z-dry values")
    if any(float(value) <= 0.0 for value in dry_values):
        raise ValueError("linear-dry-mass inertia endpoints must be positive")
    dry_inertia = Vector3(*dry_values)

    def provider(state: RigidBody6DofState) -> Vector3:
        fraction = max(0.0, min(1.0, (state.mass - dry_mass) / max(reference_mass_kg - dry_mass, 1.0e-12)))
        return Vector3(
            dry_inertia.x + fraction * (reference_inertia.x - dry_inertia.x),
            dry_inertia.y + fraction * (reference_inertia.y - dry_inertia.y),
            dry_inertia.z + fraction * (reference_inertia.z - dry_inertia.z),
        )
        ####

    return provider
    ####


def _build_attitude_lqr(
    problem: Problem,
    inertia: Vector3,
    actuator_attributes: Mapping[str, str],
    tables: Mapping[str, RuntimeTable],
    *,
    initial_mass_kg: float | None = None,
    controller_name: str = "attitude",
) -> GainScheduledLqrController | None:
    """Build a declared rigid-body attitude or rate LQR when present."""

    controller_name = controller_name.casefold()
    if controller_name not in {"attitude", "rate"}:
        raise ValueError("runtime LQR controller name must be attitude or rate")
    attributes = _runtime_lqr_attributes(problem, controller_name)
    if not attributes:
        return None
    profile_id = attributes.get("profile")
    if profile_id is not None:
        from taoryx.vehicle_registry import lqr_profile_attributes

        profile_attributes = lqr_profile_attributes(profile_id)
        attributes = {**profile_attributes, **attributes}
    import numpy as np

    state_names = (
        ("attitude-error-x", "attitude-error-y", "attitude-error-z", "wx", "wy", "wz")
        if controller_name == "attitude"
        else ("wx", "wy", "wz")
    )
    control_names = ("moment-x", "moment-y", "moment-z")
    state_dimension = len(state_names)
    a_matrix = _lqr_matrix_source(tables, attributes.get("a-table"), (state_dimension, state_dimension))
    b_matrix = _lqr_matrix_source(tables, attributes.get("b-table"), (state_dimension, 3))
    b_table_supplied = b_matrix is not None
    if a_matrix is None:
        a_matrix = np.zeros((state_dimension, state_dimension), dtype=float)
        if controller_name == "attitude":
            a_matrix[:3, 3:] = np.eye(3)
    angle_weight = float(attributes.get("q-angle", "1.0"))
    rate_weight = float(attributes.get("q-rate", "1.0"))
    moment_weight = float(attributes.get("r-moment", "1.0"))
    q_diagonal = (
        (angle_weight, angle_weight, angle_weight, rate_weight, rate_weight, rate_weight)
        if controller_name == "attitude"
        else (rate_weight, rate_weight, rate_weight)
    )
    q_matrix = _lqr_weight_source(tables, attributes.get("q-table"), (state_dimension, state_dimension), q_diagonal)
    r_matrix = _lqr_weight_source(tables, attributes.get("r-table"), (3, 3), (moment_weight, moment_weight, moment_weight))
    state_angle_scale = attributes.get("state-angle-scale-rad")
    state_rate_scale = attributes.get("state-rate-scale-rad-s")
    control_moment_scale = attributes.get("control-moment-scale-nm")
    mass_scale_text = attributes.get("mass-scale-kg")
    mass_scaling = attributes.get("mass-scaling", "none").casefold()
    scaled = any(value is not None for value in (state_angle_scale, state_rate_scale, control_moment_scale))
    if scaled:
        required_scales = (state_rate_scale, control_moment_scale) if controller_name == "rate" else (state_angle_scale, state_rate_scale, control_moment_scale)
        if any(value is None for value in required_scales):
            controller_label = "rate" if controller_name == "rate" else "attitude"
            raise ValueError(
                f"scaled {controller_label} LQR requires "
                + (
                    "state-rate-scale-rad-s and control-moment-scale-nm"
                    if controller_name == "rate"
                    else "state-angle-scale-rad, state-rate-scale-rad-s, and control-moment-scale-nm"
                )
            )
        if controller_name == "attitude":
            assert state_angle_scale is not None
        assert state_rate_scale is not None
        assert control_moment_scale is not None
    angle_scale = float(state_angle_scale) if state_angle_scale is not None else 1.0
    rate_scale = float(state_rate_scale) if state_rate_scale is not None else 1.0
    moment_scale = float(control_moment_scale) if control_moment_scale is not None else 1.0
    mass_scale = float(mass_scale_text) if mass_scale_text is not None else None
    if mass_scale is not None and (not math.isfinite(mass_scale) or mass_scale <= 0.0):
        raise ValueError("scaled attitude LQR mass-scale-kg must be finite and positive")
    maximum_moment_text = actuator_attributes.get("maximum-moment")
    maximum_moment = abs(float(maximum_moment_text)) if maximum_moment_text is not None else None
    uncertainty = LqrUncertaintySpec(
        a_fraction=float(attributes.get("a-uncertainty-fraction", "0.0")),
        b_fraction=float(attributes.get("b-uncertainty-fraction", "0.0")),
        samples=int(attributes.get("uncertainty-samples", "9")),
        seed=int(attributes.get("uncertainty-seed", "1995")),
    )
    update = attributes.get("update", "initial").casefold()
    scheduled_implementation: ControllerImplementation = "gain_scheduled_lqr" if update in {"mass", "operating-point", "schedule"} else "lqr"

    def build(current_mass_kg: float | None, current_inertia: tuple[float, float, float] | None) -> LqrController:
        current = Vector3(*(current_inertia or (inertia.x, inertia.y, inertia.z)))
        effective_b = cast(Any, b_matrix)
        if effective_b is None:
            effective_b = np.zeros((state_dimension, 3), dtype=float)
            row = 3 if controller_name == "attitude" else 0
            effective_b[row:, :] = np.diag((1.0 / current.x, 1.0 / current.y, 1.0 / current.z))
        if scaled:
            effective_moment_scale = moment_scale
            if mass_scaling == "nominal-ratio":
                if mass_scale is None:
                    raise ValueError("nominal-ratio mass scaling requires mass-scale-kg")
                if current_mass_kg is not None:
                    if not math.isfinite(current_mass_kg) or current_mass_kg <= 0.0:
                        raise ValueError("nominal-ratio mass scaling requires a positive runtime mass")
                    effective_moment_scale *= current_mass_kg / mass_scale
            result = solve_scaled_continuous_lqr(
                cast(Sequence[Sequence[float]], a_matrix),
                cast(Sequence[Sequence[float]], effective_b),
                cast(Sequence[Sequence[float]], q_matrix),
                cast(Sequence[Sequence[float]], r_matrix),
                state_scales=(
                    (angle_scale,) * 3 + (rate_scale,) * 3
                    if controller_name == "attitude"
                    else (rate_scale,) * 3
                ),
                control_scales=(effective_moment_scale,) * 3,
                state_names=state_names,
                control_names=control_names,
            )
        else:
            result = solve_continuous_lqr(
                cast(Sequence[Sequence[float]], a_matrix),
                cast(Sequence[Sequence[float]], effective_b),
                cast(Sequence[Sequence[float]], q_matrix),
                cast(Sequence[Sequence[float]], r_matrix),
                state_names=state_names,
                control_names=control_names,
            )
        robustness = assess_lqr_robustness(
            cast(Sequence[Sequence[float]], a_matrix),
            cast(Sequence[Sequence[float]], effective_b),
            result,
            uncertainty,
        ) if uncertainty.a_fraction > 0.0 or uncertainty.b_fraction > 0.0 else None
        if not result.hurwitz:
            raise ValueError(
                f"{controller_name} LQR closed-loop poles are not strictly stable: "
                f"maximum real pole={result.maximum_real_pole:.6g}"
            )
        if robustness is not None and not robustness.stable and attributes.get("uncertainty-policy", "fail-closed").casefold() == "fail-closed":
            raise ValueError(
                f"{controller_name} LQR is not stable across the declared derivative uncertainty envelope: "
                f"worst maximum real pole={robustness.worst_max_real_pole:.6g}"
            )
        limits = {
            name: -maximum_moment
            for name in control_names
        } if maximum_moment is not None else {}
        uppers = {
            name: maximum_moment
            for name in control_names
        } if maximum_moment is not None else {}
        state_unit = "rad" if controller_name == "attitude" else "rad/s"
        state_scale = angle_scale if controller_name == "attitude" else rate_scale
        realization_role: ControllerRole = "attitude" if controller_name == "attitude" else "rate"
        realization_id = f"runtime-{controller_name}-lqr:{profile_id or 'inline'}"
        design_id = profile_id or f"inline-{controller_name}-lqr"
        realization = ControllerRealization(
            id=realization_id,
            role=realization_role,
            implementation=scheduled_implementation,
            implementation_version="runtime-scaled-lqr-v1" if scaled else "runtime-lqr-v1",
            fidelity="rigid_body_6dof",
            design_id=design_id,
            states=tuple(
                ControllerChannel(
                    name=name,
                    order=index,
                    unit=state_unit if controller_name == "rate" or index >= 3 else "rad",
                    frame="body",
                    scale=state_scale if controller_name == "rate" or index >= 3 else angle_scale,
                )
                for index, name in enumerate(state_names)
            ),
            inputs=tuple(
                ControllerChannel(
                    name=name,
                    order=index,
                    unit="N*m",
                    frame="body",
                    scale=effective_moment_scale if scaled else 1.0,
                    lower=limits.get(name),
                    upper=uppers.get(name),
                )
                for index, name in enumerate(control_names)
            ),
            plant_source=f"problem:{problem.name}",
            linearization_source=f"runtime-{controller_name}-linearization",
            operating_point={"mass_kg": current_mass_kg if current_mass_kg is not None else "initial"},
            state_scale_id=profile_id or "runtime-state-scales-v1",
            control_scale_id=profile_id or "runtime-control-scales-v1",
            q_id=attributes.get("q-table", "inline-q"),
            r_id=attributes.get("r-table", "inline-r"),
            a_sha256=result.a_sha256,
            b_sha256=result.b_sha256,
            q_sha256=result.q_sha256,
            r_sha256=result.r_sha256,
            k_sha256=result.k_sha256,
            closed_loop_poles=tuple(ClosedLoopPole(real=float(value.real), imaginary=float(value.imag)) for value in result.closed_loop_eigenvalues),
            closed_loop_max_real_pole=result.maximum_real_pole,
            allocator_id=actuator_attributes.get("allocator", f"runtime-{controller_name}-moment"),
            control_path=(
                ("guidance", "reference_shaping", "attitude_lqr", "allocator", "actuator", "plant")
                if controller_name == "attitude"
                else ("guidance", "rate_lqr", "allocator", "actuator", "plant")
            ),
            fallback_controller_id=attributes.get("fallback-controller"),
            scenario_overrides_allowed=attributes.get("allow-scenario-gain-override", "false").casefold() == "true",
            claim_status="design",
            evidence_tier="T0_structural",
            # The runtime's default B is an inertia/direct-moment bridge.
            # A later table allocator may realize its request through surfaces,
            # but that does not retroactively make this LQR plant-derived.
            control_realization_path="direct_wrench_screen",
            provenance={
                "problem": problem.name,
                "profile": profile_id or "inline",
                "update": attributes.get("update", "initial"),
                "controller": controller_name,
                "screen_only": "true",
                "b_matrix_source": "declared-table" if b_table_supplied else "inertia-direct-moment-bridge",
            },
        )
        return LqrController(result, lower=limits, upper=uppers, robustness=robustness, realization=realization)

    nominal = build(initial_mass_kg, (inertia.x, inertia.y, inertia.z))
    if update in {"mass", "operating-point", "schedule"} and b_table_supplied:
        raise ValueError(
            f"mass-scheduled {controller_name} LQR requires a runtime-generated or per-operating-point B matrix; "
            "a fixed b-table cannot claim mass scheduling"
        )
    if update in {"mass", "operating-point", "schedule"}:
        mass_tolerance = max(1.0e-6, 1.0e-4 * mass_scale) if mass_scale is not None else 1.0e-6
        return GainScheduledLqrController(nominal, builder=build, mass_tolerance_kg=mass_tolerance)
    return GainScheduledLqrController(nominal)
####


def _lqr_matrix_source(tables: Mapping[str, RuntimeTable], name: str | None, shape: tuple[int, int]) -> Any:
    """Load a flattened matrix from a prepared output table."""

    if name is None:
        return None
    table = tables.get(name.casefold())
    if table is None or table.prepared is None:
        raise ValueError(f"LQR matrix table {name!r} must resolve to a prepared table")
    import numpy as np

    values = table.prepared.values
    expected = shape[0] * shape[1]
    if len(values) != expected:
        raise ValueError(f"LQR matrix table {name!r} contains {len(values)} values; expected {expected}")
    return np.asarray(values, dtype=float).reshape(shape)
####


def _lqr_weight_source(
    tables: Mapping[str, RuntimeTable],
    name: str | None,
    shape: tuple[int, int],
    diagonal: tuple[float, ...],
) -> Any:
    """Load a full or diagonal LQR weight matrix from a prepared table."""

    if name is None:
        import numpy as np

        return np.diag(diagonal)
    table = tables.get(name.casefold())
    if table is None or table.prepared is None:
        raise ValueError(f"LQR weight table {name!r} must resolve to a prepared table")
    import numpy as np

    values = table.prepared.values
    if len(values) == len(diagonal):
        return np.diag(np.asarray(values, dtype=float))
    expected = shape[0] * shape[1]
    if len(values) != expected:
        raise ValueError(f"LQR weight table {name!r} contains {len(values)} values; expected {len(diagonal)} or {expected}")
    return np.asarray(values, dtype=float).reshape(shape)
####


def _runtime_parameters(problem: Problem) -> dict[str, float]:
    """Resolve one-time parameter declarations from a problem file."""

    parameters: dict[str, float] = {}
    for block in problem.blocks:
        if not isinstance(block, RuntimeBlock) or block.declaration != "parameter" or block.name is None:
            continue
        raw_value = block.attributes.get("value", block.attributes.get("default"))
        if raw_value is None:
            raise ValueError(f"runtime parameter {block.name!r} requires value= or default=")
        parameters[block.name.casefold()] = float(raw_value)
    return parameters
####


def _apply_integrator_override(case: RuntimeCase, integrator: IntegratorName | None) -> RuntimeCase:
    """Apply one runtime-selected integrator to every vehicle in a case."""

    if integrator is not None:
        for vehicle in case.problem.vehicles.values():
            vehicle.integrator = integrator.value
    return case
####


def _lower_case_for_runtime(
    problem: Problem,
    parameters: Mapping[str, float],
    index: int,
    tables: Mapping[str, RuntimeTable],
    unit_settings: Mapping[str, str | None],
    integrator: IntegratorName | None,
) -> RuntimeCase:
    """Lower a candidate and apply the caller-selected integrator."""

    return _apply_integrator_override(_lower_case(problem, parameters, index, tables, unit_settings), integrator)
####


def _initial_values(
    block: InitialBlock,
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable] | None = None,
    unit_settings: Mapping[str, str | None] = {},
) -> tuple[tuple[str, ...], tuple[float, ...], dict[str, float], float]:
    # ``time`` is available as the zero-time default even when the source
    # assigns it later in the initial block.
    named: dict[str, float] = dict(parameters)
    named["time"] = 0.0
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


def _runtime_rotor_allocation(vehicle_attributes: Mapping[str, str]) -> QuadRotorAllocation | None:
    """Resolve the generic quadrotor allocation contract from vehicle metadata."""

    if vehicle_attributes.get("rotor-allocation", "").casefold() not in {"quad-x", "quadrotor-x"}:
        return None
    force_model = vehicle_attributes.get("rotor-force-model", "common-speed-table").casefold()
    individual_source = force_model in {"individual-rotor-source", "individual-rotor-source-v1"}
    if force_model not in {"common-speed-table", "individual-rotor-source", "individual-rotor-source-v1"}:
        raise ValueError(f"unsupported rotor-force-model {force_model!r}")

    def source_value(name: str) -> float | None:
        if not individual_source:
            return None
        value = vehicle_attributes.get(name)
        if value is None:
            raise ValueError(f"individual-rotor source model requires vehicle attribute {name!r}")
        return float(value)

    frame_drag_x = source_value("rotor-frame-drag-x-coefficient")
    frame_drag_y = source_value("rotor-frame-drag-y-coefficient")
    frame_drag_z = source_value("rotor-frame-drag-z-coefficient")
    return QuadRotorAllocation(
        arm_m=float(vehicle_attributes.get("rotor-arm-m", "0.17")),
        thrust_coefficient_n_per_rad_s2=float(vehicle_attributes.get("rotor-thrust-coefficient", "5.57e-6")),
        reaction_torque_coefficient_nm_per_rad_s2=float(vehicle_attributes.get("rotor-reaction-torque-coefficient", "1.36e-7")),
        minimum_speed_rad_s=float(vehicle_attributes.get("rotor-speed-min", "0.0")),
        maximum_speed_rad_s=float(vehicle_attributes.get("rotor-speed-max", "1500.0")),
        rotor_drag_xy_coefficient_n_s_per_m=source_value("rotor-drag-xy-coefficient"),
        rotor_drag_z_coefficient_n_s_per_m=source_value("rotor-drag-z-coefficient"),
        translational_lift_coefficient_n_s2_per_m2=source_value("rotor-translational-lift-coefficient"),
        frame_drag_coefficients_n_s2_per_m2=(
            None
            if frame_drag_x is None or frame_drag_y is None or frame_drag_z is None
            else Vector3(frame_drag_x, frame_drag_y, frame_drag_z)
        ),
    )
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
    if any(block.helper_calls for block in blocks) and all(
        not any(isinstance(statement, DefineControlStatement) for statement in block.typed_statements)
        for block in blocks
    ):
        expressions = {
            assignment.name.casefold(): assignment.value
            for block in blocks
            for assignment in block.assignments
        }
        resolved = evaluate_definition_program(
            expressions,
            working,
            parameters=parameters,
            table_evaluators=_table_evaluators(tables),
            call_handler=_surface_call_handler(_surface_helper_context(blocks, working, parameters)),
        )
        return {name: resolved[name] for name in expressions if name in resolved}
    # Conditional branches are allowed to assign different output names.  A
    # file may request both names even when only one branch is active, so make
    # the inactive branch explicitly available as the historical zero default
    # before executing the ordered program.
    declared: list[str] = []
    for block in blocks:
        if block.variable is not None:
            declared.append(block.variable.casefold())
        if block.typed_statements:
            for statement in block.typed_statements:
                declared.extend(
                    _define_control_names(statement)
                    if isinstance(statement, DefineControlStatement)
                    else (statement.assignment.name.casefold(),)
                )
        else:
            declared.extend(assignment.name.casefold() for assignment in block.assignments)
    for name in dict.fromkeys(declared):
        working.setdefault(name, 0.0)
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


def _rigid_body_gravitational_parameter(problem: Problem, value: float) -> float:
    """Return SI GM for the native rigid-body kernel.

    The historical TAOS Earth defaults are stored in the point-mass
    implementation's customary-unit convention.  Native rigid-body states
    are SI ECIC states, so an omitted ``gm`` must be converted once at this
    boundary.  An explicit ``gm`` is treated as an already-SI override.
    """

    earth = next((block for block in problem.blocks if isinstance(block, EarthBlock)), None)
    if earth is None or any(assignment.name.casefold() == "gm" for assignment in earth.assignments):
        return value
    return value * 0.3048**3
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
) -> CartesianTriple:
    """Resolve axial drag from active ``ca`` tables in the current segment."""

    speed = math.sqrt(sum(named.get(name, 0.0) ** 2 for name in ("xdt", "ydt", "zdt")))
    scalar_speed = abs(named.get("vel", 0.0))
    speed = speed if speed > 0.0 else scalar_speed
    if speed <= 0.0:
        return 0.0, 0.0, 0.0
    density = max(named.get("rho", 0.0), 0.0)
    query_values = _point_mass_aero_query_values(named)
    dynamic_pressure = named.get("dynprs", 0.5 * density * speed * speed)
    acceleration = 0.0
    for block in segment.blocks:
        if not isinstance(block, AeroBlock):
            continue
        assignment = next((item for item in block.assignments if item.name.casefold() == "ca"), None)
        if assignment is None:
            continue
        coefficient = evaluate_expression(assignment.value, query_values, parameters, tables=_table_evaluators(tables))
        reference_area = _aero_reference_area(block, assignment.value, query_values, parameters, tables)
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
    throttle: float = 1.0,
) -> CartesianTriple:
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
    query_values = _point_mass_aero_query_values(named)
    for block in segment.blocks:
        if not isinstance(block, PropulsionBlock):
            continue
        assignments = {item.name.casefold(): item for item in block.assignments}
        thrust_assignment = assignments.get("thrust")
        if thrust_assignment is None:
            continue
        thrust = _evaluate_propulsion_thrust(thrust_assignment.value, query_values, parameters, tables, throttle)
        ep1 = evaluate_expression(assignments["ep1"].value, query_values, parameters, tables=evaluators) if "ep1" in assignments else 0.0
        ep2 = evaluate_expression(assignments["ep2"].value, query_values, parameters, tables=evaluators) if "ep2" in assignments else 0.0
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
) -> CartesianTriple:
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
    query_values = _point_mass_aero_query_values(named)
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
        reference_area = _aero_reference_area(block, assignments[family[0]].value if family else None, query_values, parameters, tables)
        for name in family:
            assignment = assignments[name]
            coefficients[name] = evaluate_expression(assignment.value, query_values, parameters, tables=evaluators)
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


def _point_mass_aero_query_values(named: Mapping[str, float]) -> dict[str, float]:
    """Expose canonical table axes to the point-mass reduction.

    Vehicle coefficient decks use descriptive axes such as ``velocity_m_s``
    and ``altitude_m`` while the historical point-mass state uses ``vel`` and
    ``alt``.  This adapter keeps the problem-file state compact and lets the
    3-DOF reduction query the same verified tables as the rigid-body plant.
    """

    values = dict(named)
    si_contract = values.get("_point_mass_si_contract", 0.0) >= 0.5
    speed = math.sqrt(sum(values.get(name, 0.0) ** 2 for name in ("xdt", "ydt", "zdt")))
    # The point-mass runtime retains TAOS canonical English units internally;
    # these source tables are SI.  Convert only at the table-query boundary.
    scale = 0.3048 if si_contract else 1.0
    values["velocity_m_s"] = (speed if speed > 0.0 else abs(values.get("vel", 0.0))) * scale
    values["altitude_m"] = values.get("alt", 0.0) * scale
    if "alpha" in values:
        values.setdefault("alpha_rad", math.radians(values["alpha"]))
        # Source-backed SI vehicle tables declare angular axes in radians.
        # Legacy TAOS tables may deliberately use degree-valued axes; retain
        # their native query value unless the problem explicitly selected the
        # SI source-vehicle contract.
        if si_contract:
            values["alpha"] = values["alpha_rad"]
    else:
        values.setdefault("alpha_rad", 0.0)
    if "beta" in values:
        values.setdefault("beta_rad", math.radians(values["beta"]))
        if si_contract:
            values["beta"] = values["beta_rad"]
    else:
        values.setdefault("beta_rad", 0.0)
    values.setdefault("velocity_x", values.get("xdt", 0.0))
    values.setdefault("velocity_y", values.get("ydt", 0.0))
    values.setdefault("velocity_z", values.get("zdt", 0.0))
    # Point-mass table decks use underscore axis names while runtime controls
    # retain the TAOS-style hyphenated names with explicit degree suffixes.
    for axis in ("collective_elevon", "differential_elevon", "elevator", "aileron", "rudder"):
        values[axis] = math.radians(values.get(f"{axis.replace('_', '-')}-deg", 0.0))
    # The point-mass Hummingbird reduction uses the source's documented
    # symmetric-hover rotor speed when no explicit rotor state exists.
    values.setdefault("rotor_speed", values.get("rotor-speed", 469.124102661955))
    return values
####


def _evaluate_propulsion_thrust(
    expression: ExpressionType,
    values: Mapping[str, float],
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
    throttle: float,
) -> float:
    """Evaluate thrust without scaling a table that already has a throttle axis.

    A literal ``*prop thrust=...`` is a throttle-scaled command.  A direct
    table reference whose independent variables include ``throttle`` is
    already the commanded thrust, so multiplying it again would apply the
    control twice.  Keeping that distinction here makes table-backed
    propulsion reusable by both point-mass and rigid-body modes.
    """

    value = evaluate_expression(expression, values, parameters, tables=_table_evaluators(tables))
    if isinstance(expression, TableReferenceExpression):
        table = tables.get(expression.name.casefold())
        if table is not None and "throttle" in table.independent_variables:
            return value
    return value * throttle
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
    propulsive_acceleration: CartesianTriple | None = None,
    guidance_acceleration: CartesianTriple = (0.0, 0.0, 0.0),
    derived_expressions: Mapping[str, ExpressionType] = {},
) -> CartesianTriple | None:
    """Project geodetic force, gravity, and rotating-frame terms into rates."""

    longitude_name = "long" if "long" in named else "lon" if "lon" in named else None
    if segment is None or longitude_name is None or named.get("_geodetic_state", 0.0) < 0.5 or not {"lat", "vel", "gama", "psi"}.issubset(named):
        return None
    si_contract = named.get("_point_mass_si_contract", 0.0) >= 0.5
    effective_mass = mass * 0.45359237 if si_contract else mass
    basis = geodetic_unit_vectors(Longitude(math.radians(named[longitude_name])), Latitude(math.radians(named["lat"])))
    north, east, up = basis.first, basis.second, basis.third.scaled(-1.0)
    gamma = math.radians(named["gama"])
    heading = math.radians(named["psi"])
    horizontal = north.scaled(math.cos(heading)) + east.scaled(math.sin(heading))
    forward = horizontal.scaled(math.cos(gamma)) + up.scaled(math.sin(gamma))
    normal = horizontal.scaled(-math.sin(gamma)) + up.scaled(math.cos(gamma))
    side = north.scaled(-math.sin(heading)) + east.scaled(math.cos(heading))

    body_axis_aero = any(
        isinstance(block, AeroBlock)
        and any(
            item.name.casefold() in {"cx", "cy", "cz"}
            or item.name.casefold().endswith(("-cx", "-cy", "-cz"))
            for item in block.assignments
        )
        for block in segment.blocks
    )
    if "pitchi" in named:
        pitch = math.radians(named["pitchi"])
    elif si_contract and body_axis_aero and "alpha" in named:
        # Body-axis force coefficients are defined in the body frame.  In the
        # point-mass reduction, the body x-axis therefore leads the velocity
        # vector by alpha: pitch = flight-path angle + angle of attack.
        # Lift/drag coefficient families remain projected in the wind frame.
        effective_alpha = named["alpha"] + named.get("_aero_alpha_reference_deg", 0.0)
        pitch = math.radians(named["gama"] + effective_alpha)
    else:
        pitch = math.radians(named.get("pitchgd", named["gama"]))
    yaw = math.radians(named.get("yawi", named.get("yawgd", named["psi"])))
    roll = math.radians(named.get("rolli", named.get("rollgd", 0.0)))
    body_horizontal = north.scaled(math.cos(yaw)) + east.scaled(math.sin(yaw))
    body_forward = body_horizontal.scaled(math.cos(pitch)) + up.scaled(math.sin(pitch))
    body_normal = body_horizontal.scaled(-math.sin(pitch)) + up.scaled(math.cos(pitch))
    body_side = north.scaled(-math.sin(yaw)) + east.scaled(math.cos(yaw))
    body_y = body_side.scaled(math.cos(roll)) + body_normal.scaled(math.sin(roll))
    # TAOS body Z is positive down; ``body_normal`` is positive up.
    body_z = body_side.scaled(math.sin(roll)) - body_normal.scaled(math.cos(roll))
    aero_forward = body_forward if si_contract else forward
    aero_side = body_y if si_contract else side
    aero_down = body_z if si_contract else up.scaled(-1.0)
    if propulsive_acceleration is None:
        commanded_horizontal = north.scaled(math.cos(yaw)) + east.scaled(math.sin(yaw))
        commanded = commanded_horizontal.scaled(math.cos(pitch)) + up.scaled(math.sin(pitch))
        total = commanded.scaled(thrust / effective_mass)
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
        if si_contract:
            total = total + (gravity_vector + coriolis + centrifugal).scaled(0.3048)
        else:
            total = total + gravity_vector + coriolis + centrifugal
    speed_native = abs(named["vel"])
    speed_for_rates = speed_native * 0.3048 if si_contract else speed_native
    if si_contract and "rho" in named:
        density_kg_m3 = max(named["rho"], 0.0) * 515.3788184
        dynamic_pressure = 0.5 * density_kg_m3 * speed_for_rates * speed_for_rates
    elif si_contract:
        dynamic_pressure = max(named.get("dynprs", 0.0), 0.0) * 47.88025898
    else:
        dynamic_pressure = named.get("dynprs", 0.5 * max(named.get("rho", 0.0), 0.0) * speed_native * speed_native)
    query_values = _point_mass_aero_query_values(named)
    # Table-backed ``*define`` expressions must be resolved against the
    # current aerodynamic query, not the initialization snapshot.  Remove
    # cached derived values from the lookup context so the resolver below
    # cannot accidentally shadow the live table composition.
    live_query_values = {
        key: value
        for key, value in query_values.items()
        if key.casefold() not in derived_expressions
    }

    def resolve_derived(name: str, active: frozenset[str] = frozenset()) -> float:
        key = name.casefold()
        if key not in derived_expressions:
            raise KeyError(f"undefined variable: {name}")
        if key in active:
            raise ValueError(f"cyclic derived aerodynamic definition involving {name!r}")
        return evaluate_expression(
            derived_expressions[key],
            live_query_values,
            parameters,
            resolver=lambda nested: resolve_derived(nested, active | {key}),
            tables=_table_evaluators(tables),
        )
    ####

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
            coefficient = evaluate_expression(
                assignment.value,
                live_query_values,
                parameters,
                resolver=resolve_derived,
                tables=_table_evaluators(tables),
            )
            reference_area = _aero_reference_area(block, assignment.value, live_query_values, parameters, tables)
            force_acceleration = dynamic_pressure * reference_area * coefficient / effective_mass
            total = total + {
                "ca": forward.scaled(-force_acceleration),
                "cn": normal.scaled(force_acceleration),
                "cd": forward.scaled(-force_acceleration),
                "cl": normal.scaled(force_acceleration),
                "cs": side.scaled(force_acceleration),
                "cx": aero_forward.scaled(force_acceleration),
                "cy": aero_side.scaled(force_acceleration),
                "cz": aero_down.scaled(force_acceleration),
            }[name]
        ####
    ####
    total = _apply_runtime_rail_constraint(segment, named, total, parameters, (forward, body_y, body_z))
    speed_rate = total.dot(forward) / 0.3048 if si_contract else total.dot(forward)
    if speed_for_rates <= 1e-8:
        # Flight-path and heading rates are undefined at rest; the rail
        # constraint supplies the launch acceleration until a direction exists.
        gamma_rate = 0.0
        heading_rate = 0.0
    else:
        gamma_rate = total.dot(normal) / speed_for_rates * 180.0 / math.pi
        heading_rate = total.dot(side) / max(speed_for_rates * abs(math.cos(gamma)), 1e-12) * 180.0 / math.pi
    return speed_rate, gamma_rate, heading_rate
####


def _geodetic_propulsive_acceleration(
    segment: Segment | None,
    named: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
    parameters: Mapping[str, float],
    mass: float,
    throttle: float = 1.0,
) -> CartesianTriple | None:
    """Resolve propulsion vectors in the active geodetic-horizon basis."""

    if segment is None:
        return None
    longitude_name = "long" if "long" in named else "lon" if "lon" in named else None
    if longitude_name is None or named.get("_geodetic_state", 0.0) < 0.5 or not {"lat", "gama", "psi"}.issubset(named):
        return None
    si_contract = named.get("_point_mass_si_contract", 0.0) >= 0.5
    effective_mass = mass * 0.45359237 if si_contract else mass
    basis = geodetic_unit_vectors(Longitude(math.radians(named[longitude_name])), Latitude(math.radians(named["lat"])))
    north, east, up = basis.first, basis.second, basis.third.scaled(-1.0)
    yaw = math.radians(named.get("yawi", named.get("yawgd", named["psi"])))
    if "pitchi" in named:
        pitch = math.radians(named["pitchi"])
    elif si_contract and "alpha" in named:
        # A body-axis propulsion vector follows the body x-axis, which is
        # offset from the velocity vector by the prescribed angle of attack.
        effective_alpha = named["alpha"] + named.get("_aero_alpha_reference_deg", 0.0)
        pitch = math.radians(named["gama"] + effective_alpha)
    else:
        pitch = math.radians(named.get("pitchgd", named["gama"]))
    roll = math.radians(named.get("rolli", named.get("rollgd", 0.0)))
    horizontal = north.scaled(math.cos(yaw)) + east.scaled(math.sin(yaw))
    forward = horizontal.scaled(math.cos(pitch)) + up.scaled(math.sin(pitch))
    side = north.scaled(-math.sin(yaw)) + east.scaled(math.cos(yaw))
    normal = horizontal.scaled(-math.sin(pitch)) + up.scaled(math.cos(pitch))
    body_y = side.scaled(math.cos(roll)) + normal.scaled(math.sin(roll))
    body_z = side.scaled(-math.sin(roll)) + normal.scaled(math.cos(roll))
    total = Vector3(0.0, 0.0, 0.0)
    evaluators = _table_evaluators(tables)
    query_values = _point_mass_aero_query_values(named)
    for block in segment.blocks:
        if not isinstance(block, PropulsionBlock):
            continue
        assignments = {item.name.casefold(): item for item in block.assignments}
        thrust_assignment = assignments.get("thrust")
        if thrust_assignment is None:
            continue
        thrust = _evaluate_propulsion_thrust(thrust_assignment.value, query_values, parameters, tables, throttle)
        ep1 = math.radians(evaluate_expression(assignments["ep1"].value, query_values, parameters, tables=evaluators)) if "ep1" in assignments else 0.0
        ep2 = math.radians(evaluate_expression(assignments["ep2"].value, query_values, parameters, tables=evaluators)) if "ep2" in assignments else 0.0
        body_vector = (
            forward.scaled(thrust * math.cos(ep1))
            + body_y.scaled(-thrust * math.sin(ep1) * math.cos(ep2))
            + body_z.scaled(-thrust * math.sin(ep1) * math.sin(ep2))
        )
        total = total + body_vector
    return total.x / effective_mass, total.y / effective_mass, total.z / effective_mass
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
    propulsive = _ecfc_propulsive_acceleration(segment, values, tables, parameters, mass, values.get("throttle", 1.0))
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
    origin = cast(CartesianTriple, origin_value)
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


def _platform_velocity_components(vector: CartesianTriple, basis: PlatformBasis) -> dict[str, float]:
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


def _platform_assignment_vector(assignments: Mapping[str, float], prefix: str, default: CartesianTriple) -> CartesianTriple:
    return (
        assignments.get(f"{prefix}x", default[0]),
        assignments.get(f"{prefix}y", default[1]),
        assignments.get(f"{prefix}z", default[2]),
    )
####


def _rotate_ecfc_to_ecic(vector: CartesianTriple, rotation_rate: float, time: float) -> CartesianTriple:
    angle = rotation_rate * time
    cosine, sine = math.cos(angle), math.sin(angle)
    return (vector[0] * cosine - vector[1] * sine, vector[0] * sine + vector[1] * cosine, vector[2])
####


def _rotate_ecfc_velocity_to_ecic(
    position: CartesianTriple,
    velocity: CartesianTriple,
    rotation_rate: float,
    time: float,
) -> CartesianTriple:
    angle = rotation_rate * time
    cosine, sine = math.cos(angle), math.sin(angle)
    return (
        velocity[0] * cosine - velocity[1] * sine - rotation_rate * (position[0] * sine + position[1] * cosine),
        velocity[0] * sine + velocity[1] * cosine + rotation_rate * (position[0] * cosine - position[1] * sine),
        velocity[2],
    )
####


def _dot_tuple(left: CartesianTriple, right: CartesianTriple) -> float:
    return sum(a * b for a, b in zip(left, right, strict=True))
####


def _cross_tuple(left: CartesianTriple, right: CartesianTriple) -> CartesianTriple:
    return (left[1] * right[2] - left[2] * right[1], left[2] * right[0] - left[0] * right[2], left[0] * right[1] - left[1] * right[0])
####


def _subtract_tuple(left: CartesianTriple, right: CartesianTriple) -> CartesianTriple:
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])
####


def _scale_tuple(vector: CartesianTriple, factor: float) -> CartesianTriple:
    return (vector[0] * factor, vector[1] * factor, vector[2] * factor)
####


def _normalize_tuple(vector: CartesianTriple) -> CartesianTriple:
    norm = math.sqrt(sum(value * value for value in vector))
    if norm <= 1e-12:
        raise ValueError("inertial platform axis must be nonzero")
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm)
####


def _apply_runtime_rail_constraint(
    segment: Segment,
    named: Mapping[str, float],
    total_acceleration: Vector3,
    parameters: Mapping[str, float],
    body_axes: tuple[Vector3, Vector3, Vector3],
) -> Vector3:
    """Apply the manual's vector rail/sled constraint in the runtime path."""

    rail = next((block for block in segment.blocks if isinstance(block, RailBlock)), None)
    if rail is None:
        return total_acceleration
    controls = {
        assignment.name.casefold(): evaluate_expression(assignment.value, named, parameters)
        for assignment in rail.assignments
    }
    speed = abs(named.get("vel", 0.0))
    if {"xdt", "ydt", "zdt"}.issubset(named):
        speed = math.sqrt(named["xdt"] ** 2 + named["ydt"] ** 2 + named["zdt"] ** 2)
    basis = Basis3(body_axes[0], body_axes[1], body_axes[2], Frame.ECFC, Frame.BODY)
    result = apply_rail_constraint(
        FrameVector3(total_acceleration, Frame.ECFC),
        basis,
        speed,
        max(0.0, controls.get("cfstat", 0.0)),
        max(0.0, controls.get("cfslid", controls.get("cfstat", 0.0))),
        mode=ConstraintMode.SLED if (rail.mode or "").casefold() == "sled" else ConstraintMode.RAIL,
        static_speed_threshold=0.001,
    )
    return result.acceleration.vector
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
    include_trajectory_references: bool = True,
    include_specific_loads_in_derivative: bool = True,
    runtime_controls: Mapping[str, float] = {},
    route_attributes: Mapping[str, str] = {},
    target_attributes: Mapping[str, str] = {},
) -> Callable[[Mapping[str, float]], Mapping[str, float]]:
    """Combine atmosphere refresh with active-segment force observables."""

    def evaluate(values: Mapping[str, float]) -> Mapping[str, float]:
        result = dict(base_evaluator(values) if base_evaluator is not None else {})
        result.setdefault("rotor_speed", values.get("rotor_speed", 469.124102661955))
        guidance_controls = {
            name: value
            for name, value in runtime_controls.items()
            if name in {"alpha", "alphat", "bankgc", "bankgd", "beta", "betae", "gamgd", "mach", "power", "psigc", "psigd"}
        }
        result.update(guidance_controls)
        if "mach" in guidance_controls:
            result["_command_mach"] = guidance_controls["mach"]
        if "gamgd" in guidance_controls:
            result["_command_gamgd"] = guidance_controls["gamgd"]
        if "psigd" in guidance_controls:
            result["_command_psigd"] = guidance_controls["psigd"]
            result["_command_psi"] = guidance_controls["psigd"]
        if "psigc" in guidance_controls:
            result["_command_psigc"] = guidance_controls["psigc"]
            result["_command_psi"] = guidance_controls["psigc"]
        if platform_state is not None:
            result.update(_inertial_platform_observables({**values, **result}, platform_state, earth_rotation_rate))
        if include_trajectory_references:
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
        result.update(guidance_controls)
        result.update(_evaluate_relative_guidance(result, vehicles, vehicle_name))
        result.update(_evaluate_range_insensitive_guidance(segment, {**values, **result}, parameters, gravitational_parameter))
        result.update(_runtime_point_mass_route_commands(route_attributes, target_attributes, {**values, **result}))
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
        # Constants and CG blocks establish table-query inputs such as flaps,
        # configuration, and mass before definitions are evaluated.  This
        # preserves the documented block order while allowing a point-mass
        # reduction to share the rigid-body coefficient deck.
        for block in segment.blocks:
            if isinstance(block, (ConstantsBlock, CgBlock)):
                for assignment in block.assignments:
                    result[assignment.name.casefold()] = evaluate_expression(
                        assignment.value,
                        _point_mass_aero_query_values({**values, **result}),
                        parameters,
                        tables=_table_evaluators(tables),
                    )
        # Point-mass aero expressions may use the same problem-scope derived
        # coefficients as a rigid-body case (for example a control-increment
        # sum). Resolve those definitions after those query inputs exist.
        result.update(
            _evaluate_definition_blocks(
                definition_blocks,
                _point_mass_aero_query_values({**values, **result}),
                parameters,
                tables,
            )
        )
        evaluation_values = _point_mass_aero_query_values({**values, **result})
        for block in segment.blocks:
            if isinstance(block, (ConstantsBlock, CgBlock)):
                for assignment in block.assignments:
                    result[assignment.name.casefold()] = evaluate_expression(
                        assignment.value,
                        evaluation_values,
                        parameters,
                        tables=_table_evaluators(tables),
                    )
            elif isinstance(block, AeroBlock):
                for name in coefficients:
                    aero_assignment = next((item for item in block.assignments if item.name.casefold() == name), None)
                    if aero_assignment is not None:
                        coefficients[name].append(evaluate_expression(aero_assignment.value, evaluation_values, parameters, tables=_table_evaluators(tables)))
            elif isinstance(block, PropulsionBlock):
                for assignment in block.assignments:
                    assignment_name = assignment.name.casefold()
                    if assignment_name == "thrust":
                        thrust += evaluate_expression(assignment.value, evaluation_values, parameters, tables=_table_evaluators(tables))
                    elif assignment_name == "mdot":
                        mdot += evaluate_expression(assignment.value, evaluation_values, parameters, tables=_table_evaluators(tables))
                    elif assignment_name in {"ep1", "ep2"}:
                        result[assignment_name] = evaluate_expression(assignment.value, evaluation_values, parameters, tables=_table_evaluators(tables))
        for name, values_for_name in coefficients.items():
            if values_for_name:
                result[name] = sum(values_for_name)
        # Publish transient propulsion observables every sample so coast and
        # post-burn states cannot inherit stale values from the prior segment.
        result["thrust"] = thrust
        result["mdot"] = mdot
        if include_specific_loads_in_derivative or values.get("_runtime_derivative_stage", 0.0) < 0.5:
            result.update(_specific_load_observables(segment, {**values, **result}, tables, parameters))
        result.update(
            _evaluate_definition_blocks(
                definition_blocks,
                _point_mass_aero_query_values({**values, **result}),
                parameters,
                tables,
            )
        )
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

    def impact_projection(yaw: float, pitch: float) -> CartesianTriple:
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
    evaluators = _table_evaluators(tables)
    if "winde" in assignments or "windn" in assignments:
        if "winde" not in assignments or "windn" not in assignments:
            raise ValueError("component wind requires both winde and windn")
        east = evaluate_expression(assignments["winde"], values, parameters, tables=evaluators)
        north = evaluate_expression(assignments["windn"], values, parameters, tables=evaluators)
        wind_vector = (east, north, evaluate_expression(assignments["windd"], values, parameters, tables=evaluators) if "windd" in assignments else 0.0)
    else:
        wind_down = math.radians(evaluate_expression(assignments["windd"], values, parameters, tables=evaluators)) if "windd" in assignments else 0.0
        wind_speed = evaluate_expression(assignments["windv"], values, parameters, tables=evaluators) if "windv" in assignments else 0.0
        wind_heading = math.radians(evaluate_expression(assignments["windh"], values, parameters, tables=evaluators)) if "windh" in assignments else 0.0
        wind_vector = (
            wind_speed * math.cos(wind_down) * math.sin(wind_heading),
            wind_speed * math.cos(wind_down) * math.cos(wind_heading),
            wind_speed * math.sin(wind_down),
        )
    heading = math.radians(values.get("psi", 0.0))
    gamma = math.radians(values.get("gama", 0.0))
    velocity = (
        speed * math.cos(gamma) * math.sin(heading),
        speed * math.cos(gamma) * math.cos(heading),
        speed * math.sin(gamma),
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
    position: CartesianTriple,
    coefficients: Mapping[tuple[int, int], tuple[float, float]],
    gravitational_parameter: float,
    reference_radius: float = 20_902_646.3255,
) -> CartesianTriple:
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


def _runtime_event_conditions(
    problem: Problem,
    parameters: Mapping[str, float],
    tables: Mapping[str, RuntimeTable],
) -> tuple[EventCondition, ...]:
    """Lower declarative TAORYX runtime events into the batch event engine."""

    events: list[EventCondition] = []
    for block in problem.blocks:
        if not isinstance(block, RuntimeBlock) or block.declaration != "event":
            continue
        condition_text = block.attributes.get("condition")
        if not condition_text or block.name is None:
            continue
        expression = parse_expression(condition_text)
        action = block.attributes.get("action", "signal").casefold()
        if action not in {"stop", "signal"}:
            raise ValueError(f"batch runtime event {block.name!r} does not support action {action!r}; use stop or signal")

        def condition_function(state: RuntimeState, expression: ExpressionType = expression) -> float:
            return _event_residual(expression, {**state.named, "time": state.time}, parameters, tables)
        ####

        def condition_predicate(state: RuntimeState, expression: ExpressionType = expression) -> bool:
            return _event_satisfied(expression, {**state.named, "time": state.time}, parameters, tables)
        ####

        events.append(
            EventCondition(
                f"runtime-{block.name}",
                condition_function,
                action,
                condition_predicate,
                signal=block.attributes.get("signal") or block.name,
                source=f"{block.location.path}:{block.location.line}",
            )
        )
    return tuple(events)
    ####


def _runtime_control_values(problem: Problem, vehicle_name: str) -> dict[str, float]:
    """Resolve bounded declarative control defaults for one batch vehicle."""

    values: dict[str, float] = {}
    for block in problem.blocks:
        if not isinstance(block, RuntimeBlock) or block.declaration != "control" or block.name is None:
            continue
        selected_vehicle = block.attributes.get("vehicle")
        if selected_vehicle is not None and selected_vehicle != vehicle_name:
            continue
        value = float(block.attributes.get("default", "0"))
        lower = float(block.attributes["lower"]) if "lower" in block.attributes else None
        upper = float(block.attributes["upper"]) if "upper" in block.attributes else None
        if lower is not None and upper is not None and lower > upper:
            raise ValueError(f"runtime control {block.name!r} has lower bound above upper bound")
        if lower is not None and value < lower or upper is not None and value > upper:
            raise ValueError(f"runtime control {block.name!r} default lies outside its bounds")
        values[block.name.casefold()] = value
    return values
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


def _strict_table_evaluators(
    tables: Mapping[str, RuntimeTable],
) -> dict[str, Callable[[Mapping[str, float]], float]]:
    """Build strict evaluators for source-backed aerodynamic tables."""

    return {
        name: _RuntimeTableEvaluator(table, tables, strict_no_extrap=True)
        for name, table in tables.items()
    }


def _apply_surface_control_inversion(
    aerodynamic_model: TableAerodynamicModel,
    state: RigidBody6DofState,
    control_values: dict[str, float],
    target_moment: Vector3,
    guidance_attributes: Mapping[str, str],
) -> bool:
    """Allocate a moment request through bounded physical table coordinates.

    The controller supplies a generalized moment request.  This function
    differentiates the active source tables, solves the common constrained
    allocator, advances declared actuator state, and evaluates the resulting
    nonlinear aerodynamic moment.  It therefore never appends a direct body
    moment merely because the requested wrench was infeasible.
    """

    candidates = (
        ("elevator-deg", "elevator", "elevator-min-deg", "elevator-max-deg"),
        ("aileron-deg", "aileron", "aileron-min-deg", "aileron-max-deg"),
        ("collective-elevon-deg", "collective_elevon", "collective-elevon-min-deg", "collective-elevon-max-deg"),
        ("differential-elevon-deg", "differential_elevon", "differential-elevon-min-deg", "differential-elevon-max-deg"),
        ("symmetric-stabilator-deg", "symmetric_stabilator", "symmetric-stabilator-min-deg", "symmetric-stabilator-max-deg"),
        ("differential-stabilator-deg", "differential_stabilator", "differential-stabilator-min-deg", "differential-stabilator-max-deg"),
        ("rudder-deg", "rudder", "rudder-min-deg", "rudder-max-deg"),
    )
    active = tuple(candidate for candidate in candidates if candidate[0] in control_values)
    if not active:
        return False
    update_period = max(0.0, float(guidance_attributes.get("surface-inversion-update-s", "0.0")))
    last_update = control_values.get("_surface-allocation-last-time")
    elapsed_since_update = (
        None
        if last_update is None
        else max(0.0, state.time - float(last_update))
    )
    if (
        update_period > 0.0
        and last_update is not None
        and state.time > float(last_update)
        and state.time - float(last_update) < update_period
    ):
        return bool(control_values.get("surface_allocation_saturated", 0.0))
    control_values["_surface-allocation-last-time"] = state.time
    raw_values = {
        degree_name: float(control_values[degree_name])
        for degree_name, _, _, _ in active
    }
    for degree_name, _, _, _ in active:
        control_values.pop(f"_surface-allocation-override-{degree_name}", None)
    controller_values = (
        dict(aerodynamic_model.control_provider(state))
        if aerodynamic_model.control_provider is not None
        else {}
    )
    base_values: dict[str, float] = {}
    for degree_name, radian_name, _, _ in active:
        # Allocate around the command after the shared hold laws.  Keep the
        # raw semantic command separate so the hold law is not cumulatively
        # applied on later allocator cycles.
        base_values[degree_name] = float(controller_values.get(degree_name, raw_values[degree_name]))
        control_values[degree_name] = raw_values[degree_name]
        control_values[radian_name] = math.radians(raw_values[degree_name])
        control_values[f"_surface-allocation-override-{degree_name}"] = base_values[degree_name]
    ####

    baseline = aerodynamic_model.evaluate(state).moment_body_nm
    step_degrees = abs(float(guidance_attributes.get("surface-inversion-step-deg", "1.0")))
    if step_degrees <= 0.0:
        raise ValueError("surface-inversion-step-deg must be positive")
    columns: list[tuple[float, float, float]] = []
    for degree_name, radian_name, _, _ in active:
        control_values[f"_surface-allocation-override-{degree_name}"] = base_values[degree_name] + step_degrees
        plus = aerodynamic_model.evaluate(state).moment_body_nm
        control_values[f"_surface-allocation-override-{degree_name}"] = base_values[degree_name] - step_degrees
        minus = aerodynamic_model.evaluate(state).moment_body_nm
        control_values[f"_surface-allocation-override-{degree_name}"] = base_values[degree_name]
        columns.append(
            (
                (plus.x - minus.x) / (2.0 * step_degrees),
                (plus.y - minus.y) / (2.0 * step_degrees),
                (plus.z - minus.z) / (2.0 * step_degrees),
            )
        )
    ####

    axis_weights = (
        max(0.0, float(guidance_attributes.get("surface-inversion-axis-weight-x", "1.0"))),
        max(0.0, float(guidance_attributes.get("surface-inversion-axis-weight-y", "1.0"))),
        max(0.0, float(guidance_attributes.get("surface-inversion-axis-weight-z", "1.0"))),
    )
    maximum_delta = abs(float(guidance_attributes.get("surface-inversion-max-delta-deg", "5.0")))
    rate_limit_text = guidance_attributes.get("surface-inversion-rate-deg-s")
    rate_limit = None if rate_limit_text is None else max(0.0, float(rate_limit_text))
    response_time = max(0.0, float(guidance_attributes.get("surface-inversion-time-constant-s", "0.0")))
    effectors: dict[str, EffectorLimits] = {}
    previous_effectors: dict[str, float] = {}
    for degree_name, _, minimum_name, maximum_name in active:
        lower = max(float(guidance_attributes.get(minimum_name, "-20.0")), base_values[degree_name] - maximum_delta)
        upper = min(float(guidance_attributes.get(maximum_name, "20.0")), base_values[degree_name] + maximum_delta)
        if lower > upper:
            raise ValueError(f"surface allocation has empty bounds for {degree_name!r}")
        # A zero configured rate is a declared frozen actuator, not an
        # implicit unlimited actuator.  Positive rates are enforced between
        # committed controller updates; absent data remains an explicit ideal
        # response assumption.
        available = rate_limit is None or rate_limit > 0.0
        effectors[degree_name] = EffectorLimits(
            degree_name,
            lower,
            upper,
            "deg",
            rate_limit_per_s=rate_limit if available else None,
            time_constant_s=response_time,
            available=available,
        )
        surface_name = degree_name.removesuffix("-deg").replace("-", "_")
        previous_effectors[degree_name] = float(
            control_values.get(
                f"surface_allocation_{surface_name}_achieved_deg",
                base_values[degree_name],
            )
        )
    effectiveness = EffectorEffectiveness(
        wrench_names=("moment_x_nm", "moment_y_nm", "moment_z_nm"),
        effector_names=tuple(effectors),
        matrix=tuple(tuple(column[axis] for column in columns) for axis in range(3)),
        reference_wrench={
            "moment_x_nm": baseline.x,
            "moment_y_nm": baseline.y,
            "moment_z_nm": baseline.z,
        },
        reference_effectors=base_values,
        source="centered-table-aerodynamic-difference",
    )
    allocation = allocate_and_advance_wrench(
        effectiveness,
        effectors,
        {
            "moment_x_nm": target_moment.x,
            "moment_y_nm": target_moment.y,
            "moment_z_nm": target_moment.z,
        },
        previous_effectors,
        0.0 if elapsed_since_update is None else elapsed_since_update,
        preferred_effectors=base_values,
        wrench_weights={
            "moment_x_nm": axis_weights[0],
            "moment_y_nm": axis_weights[1],
            "moment_z_nm": axis_weights[2],
        },
        # The legacy regularizer was expressed against a radian-coordinate
        # Jacobian.  Convert it to the degree-coordinate matrix passed to the
        # common allocator so existing scenario values retain their meaning.
        regularization=max(0.0, float(guidance_attributes.get("surface-inversion-regularization", "0.01")))
        * math.radians(1.0) ** 2,
        feasibility_tolerance=max(
            1.0e-12,
            float(guidance_attributes.get("surface-inversion-feasibility-tolerance-nm", "1.0e-6")),
        ),
    )
    for degree_name, radian_name, _, _ in active:
        actual = allocation.actuator.actual_positions[degree_name]
        control_values[degree_name] = raw_values[degree_name]
        control_values[radian_name] = math.radians(raw_values[degree_name])
        control_values[f"_surface-allocation-override-{degree_name}"] = actual
        surface_name = degree_name.removesuffix("-deg").replace("-", "_")
        control_values[f"surface_allocation_{surface_name}_commanded_deg"] = allocation.actuator.commanded_positions[degree_name]
        control_values[f"surface_allocation_{surface_name}_achieved_deg"] = actual
        control_values[f"surface_allocation_{surface_name}_rate_deg_s"] = allocation.actuator.rates_per_s[degree_name]
    actual_moment = aerodynamic_model.evaluate(state).moment_body_nm
    linearized_achieved = allocation.allocation.predicted_wrench
    actual_residual = {
        "moment_x_nm": target_moment.x - actual_moment.x,
        "moment_y_nm": target_moment.y - actual_moment.y,
        "moment_z_nm": target_moment.z - actual_moment.z,
    }
    control_values["surface_allocation_requested_moment_x_nm"] = target_moment.x
    control_values["surface_allocation_requested_moment_y_nm"] = target_moment.y
    control_values["surface_allocation_requested_moment_z_nm"] = target_moment.z
    control_values["surface_allocation_linearized_achieved_moment_x_nm"] = linearized_achieved["moment_x_nm"]
    control_values["surface_allocation_linearized_achieved_moment_y_nm"] = linearized_achieved["moment_y_nm"]
    control_values["surface_allocation_linearized_achieved_moment_z_nm"] = linearized_achieved["moment_z_nm"]
    control_values["surface_allocation_actual_achieved_moment_x_nm"] = actual_moment.x
    control_values["surface_allocation_actual_achieved_moment_y_nm"] = actual_moment.y
    control_values["surface_allocation_actual_achieved_moment_z_nm"] = actual_moment.z
    control_values["surface_allocation_residual_nm"] = allocation.allocation.residual_norm
    control_values["surface_allocation_controlled_residual_nm"] = allocation.allocation.controlled_residual_norm
    control_values["surface_allocation_actual_residual_nm"] = math.sqrt(sum(value * value for value in actual_residual.values()))
    control_values["surface_allocation_actual_controlled_residual_nm"] = math.sqrt(
        sum(actual_residual[name] * actual_residual[name] for name in allocation.allocation.controlled_wrench_axes)
    )
    status_codes = {
        "feasible": 0.0,
        "feasible_near_limit": 1.0,
        "partially_achievable": 2.0,
        "infeasible": 3.0,
        "numerically_singular": 4.0,
        "solver_failure": 5.0,
    }
    saturated = (
        allocation.allocation.status != "feasible"
        or bool(allocation.allocation.position_saturated)
        or bool(allocation.allocation.rate_limited)
        or bool(allocation.actuator.position_saturated)
        or bool(allocation.actuator.rate_limited)
        or bool(allocation.actuator.unavailable_effectors)
    )
    control_values["surface_allocation_saturated"] = 1.0 if saturated else 0.0
    control_values["surface_allocation_rate_limited"] = 1.0 if (
        allocation.allocation.rate_limited or allocation.actuator.rate_limited
    ) else 0.0
    control_values["surface_allocation_delta_limit_deg"] = maximum_delta
    control_values["surface_allocation_active_surface_count"] = float(len(active))
    control_values["surface_allocation_controlled_axis_count"] = float(len(allocation.allocation.controlled_wrench_axes))
    control_values["surface_allocation_uncontrolled_axis_count"] = float(len(allocation.allocation.uncontrolled_wrench_axes))
    control_values["surface_allocation_effectiveness_rank"] = float(allocation.allocation.effectiveness_rank)
    control_values["surface_allocation_status_code"] = status_codes[allocation.allocation.status]
    return saturated
    ####


def _runtime_table_margins(tables: Mapping[str, RuntimeTable], values: Mapping[str, float]) -> dict[str, float]:
    """Report interpolation margins for every prepared table in a composed deck."""

    result: dict[str, float] = {}
    query = dict(values)
    if "velocity_m_s" not in query and "airspeed_m_s" in query:
        query["velocity_m_s"] = query["airspeed_m_s"]
    if "airspeed_m_s" not in query and "velocity_m_s" in query:
        query["airspeed_m_s"] = query["velocity_m_s"]
    for name, table in tables.items():
        if table.prepared is None:
            continue
        for axis_name, axis in zip(table.independent_variables, table.prepared.axes, strict=True):
            if axis_name not in query:
                continue
            value = float(query[axis_name])
            result[f"table.{name}.{axis_name}"] = min(abs(value - axis[0]), abs(axis[-1] - value))
    return result
####


def _runtime_table_normalized_margins(tables: Mapping[str, RuntimeTable], values: Mapping[str, float]) -> dict[str, float]:
    """Report table margins normalized by each prepared axis span."""

    result: dict[str, float] = {}
    query = dict(values)
    if "velocity_m_s" not in query and "airspeed_m_s" in query:
        query["velocity_m_s"] = query["airspeed_m_s"]
    if "airspeed_m_s" not in query and "velocity_m_s" in query:
        query["airspeed_m_s"] = query["velocity_m_s"]
    for name, table in tables.items():
        if table.prepared is None:
            continue
        for axis_name, axis in zip(table.independent_variables, table.prepared.axes, strict=True):
            if axis_name not in query:
                continue
            span = max(abs(axis[-1] - axis[0]), 1.0e-30)
            value = float(query[axis_name])
            result[f"table.{name}.{axis_name}"] = min(abs(value - axis[0]), abs(axis[-1] - value)) / span
    return result
####
####


def _table_query_values(values: Mapping[str, float], independent_variables: Sequence[str]) -> dict[str, float]:
    """Add only explicit degree-suffixed aliases used by analysis datasets.

    The TAOS aerodynamic vocabulary names angle of attack ``alpha``. Some
    TAORYX analysis packs use ``alpha_deg`` to make units visible in source
    data. This boundary alias preserves that source naming without changing
    the canonical runtime state or silently guessing arbitrary variable names.
    """

    query = dict(values)
    for name in independent_variables:
        if name == "alpha" and "alpha_rad" in query:
            query[name] = query["alpha_rad"]
        elif name == "beta" and "beta_rad" in query:
            query[name] = query["beta_rad"]
        elif name == "collective_elevon" and "collective_elevon" not in query and "collective-elevon-deg" in query:
            query[name] = math.radians(query["collective-elevon-deg"])
        elif name == "differential_elevon" and "differential_elevon" not in query and "differential-elevon-deg" in query:
            query[name] = math.radians(query["differential-elevon-deg"])
        elif name in query:
            continue
        elif name == "alpha_deg" and "alpha" in query:
            query[name] = query["alpha"]
        elif name == "alpha" and "alpha_deg" in query:
            query[name] = query["alpha_deg"]
        elif name in {"velocity_m_s", "airspeed_m_s"} and "airspeed_m_s" in query:
            query[name] = query["airspeed_m_s"]
        elif name == "velocity_m_s":
            query[name] = math.sqrt(sum(query.get(axis, 0.0) ** 2 for axis in ("xdt", "ydt", "zdt")))
            if query[name] == 0.0:
                query[name] = abs(query.get("vel", 0.0))
        elif name == "altitude_m" and "alt" in query:
            query[name] = query["alt"]
        elif name == "beta" and "beta_rad" not in query:
            query[name] = 0.0
        elif name == "throttle":
            query[name] = 1.0
        elif name == "speed_of_sound_m_s" and "speed_of_sound_m_s" in query:
            query[name] = query["speed_of_sound_m_s"]
    return query
####


def _survey_parameters(problem: Problem) -> dict[str, Sequence[float] | SurveySpan]:
    result: dict[str, Sequence[float] | SurveySpan] = {}
    for block in problem.blocks:
        if not isinstance(block, SurveyBlock):
            continue
        settings = {setting.name: tuple(float(value) for value in setting.values) for setting in block.settings}
        if "vals" in settings:
            result[f"survey-{block.survey_id}"] = settings["vals"]
        elif {"lo", "hi", "inc"} <= settings.keys():
            result[f"survey-{block.survey_id}"] = (settings["lo"][0], settings["hi"][0], settings["inc"][0])
    return result


def _case_parameters(problem: Problem, base: Mapping[str, float]) -> tuple[dict[str, float], ...]:
    """Expand a TAORYX ``*cases`` block into deterministic parameter rows."""

    blocks = tuple(block for block in problem.blocks if isinstance(block, ExtensionBlock) and block.keyword == "cases")
    if not blocks:
        return ()
    rows: list[dict[str, float]] = []
    for block in blocks:
        columns = tuple(column.casefold() for column in block.columns if column.casefold() not in {"case", "id", "index"})
        for row in block.rows:
            if len(row) < len(columns):
                raise ValueError(f"*cases row has {len(row)} values; expected at least {len(columns)}")
            values = dict(base)
            for name, raw in zip(columns, row[-len(columns):], strict=True):
                values[name] = float(evaluate_expression(parse_expression(raw), values, values))
            definition_names = {
                assignment.name.casefold()
                for candidate in problem.blocks
                if isinstance(candidate, DefineBlock) and not candidate.integral
                for assignment in candidate.assignments
            }
            definition_inputs = {name: value for name, value in values.items() if name not in definition_names}
            definition_inputs.update({name: values[name] for name in columns})
            derived = _definition_parameter_values(problem, definition_inputs)
            values.update({name: value for name, value in derived.items() if name not in columns})
            rows.append(values)
    return tuple(rows)


def _definition_parameter_values(problem: Problem, parameters: Mapping[str, float]) -> dict[str, float]:
    """Resolve constant problem definitions needed by initial/case syntax."""

    blocks = tuple(block for block in problem.blocks if isinstance(block, DefineBlock) and not block.integral)
    expressions = {
        assignment.name.casefold(): assignment.value
        for block in blocks
        for assignment in block.assignments
    }
    if not expressions:
        return {}
    try:
        resolved = evaluate_definition_program(
            expressions,
            parameters,
            parameters=parameters,
            call_handler=_surface_call_handler(_surface_helper_context(blocks, {}, parameters)),
        )
    except (KeyError, TypeError, ValueError):
        return {}
    return {name: value for name, value in resolved.items() if name in expressions}


def _surface_helper_context(
    blocks: Sequence[DefineBlock],
    values: Mapping[str, float],
    parameters: Mapping[str, float],
) -> dict[str, float]:
    """Resolve source ``surface_ref`` calls before dependent definitions."""

    known = dict(parameters)
    known.update(values)
    assignments = {
        assignment.name.casefold(): assignment.value
        for block in blocks
        for assignment in block.assignments
    }
    for _ in range(len(assignments) + 1):
        changed = False
        for name, expression in assignments.items():
            if name in known:
                continue
            try:
                known[name] = evaluate_expression(expression, known, known)
            except (KeyError, TypeError, ValueError):
                continue
            changed = True
        if not changed:
            break
    reference: dict[str, float] = {}
    for block in blocks:
        for raw in block.helper_calls:
            function, _, argument_text = raw.partition("(")
            if function.casefold().strip() != "surface_ref":
                continue
            arguments = argument_text.rsplit(")", 1)[0].split(",")
            if len(arguments) != 2:
                raise ValueError("surface_ref requires latitude and longitude")
            reference["lat"] = evaluate_expression(parse_expression(arguments[0].strip()), known, known)
            reference["long"] = evaluate_expression(parse_expression(arguments[1].strip()), known, known)
    return reference


def _surface_call_handler(reference: Mapping[str, float]) -> Callable[[str, Sequence[float]], float]:
    """Return the runtime implementation of the Taoryx surface helpers."""

    def call(function: str, arguments: Sequence[float]) -> float:
        name = function.casefold()
        if name == "surface_ref":
            raise ValueError("surface_ref is a declaration, not an expression")
        if name not in {"surface_azm", "surface_dist"}:
            return _call(function, arguments)
        if len(arguments) != 2 or not {"lat", "long"} <= reference.keys():
            raise ValueError(f"{function} requires a preceding surface_ref and two coordinates")
        lat1 = math.radians(reference["lat"])
        lon1 = math.radians(reference["long"])
        lat2 = math.radians(arguments[0])
        lon2 = math.radians(arguments[1])
        delta_lon = lon2 - lon1
        if name == "surface_azm":
            east = math.cos(lat2) * math.sin(delta_lon)
            north = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
            return math.degrees(math.atan2(east, north))
        north = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
        central_angle = math.atan2(
            math.sqrt((math.cos(lat2) * math.sin(delta_lon)) ** 2 + north**2),
            math.sin(lat1) * math.sin(lat2) + math.cos(lat1) * math.cos(lat2) * math.cos(delta_lon),
        )
        return 6_378_137.0 * central_angle
    return call
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


def _unsupported_features(problem: Problem, profile: GrammarProfile = GrammarProfile.TAOS96) -> tuple[str, ...]:
    """List parsed control blocks that the current runtime cannot execute."""

    features: list[str] = []
    if any(not _is_supported_search(block) for block in problem.blocks if isinstance(block, SearchBlock)):
        features.append("search")
    if any(not _is_supported_optimization(block) for block in problem.blocks if isinstance(block, OptimizeBlock)):
        features.append("optimize")
    has_successor_directive = any(isinstance(block, DofDirectiveBlock) for block in problem.blocks)
    if profile is GrammarProfile.TAOS96 and _dynamics_mode(problem) is DynamicsMode.RIGID_BODY_6DOF and not has_successor_directive:
        features.append("mode rigid-body-6dof")
    return tuple(dict.fromkeys(features))


def _dynamics_mode(problem: Problem) -> DynamicsMode:
    """Return the selected mode, defaulting to manual-compatible point mass."""

    selected = next((block.mode for block in problem.blocks if isinstance(block, ModeBlock) and block.mode), None)
    directive = next((block.mode for block in problem.blocks if isinstance(block, DofDirectiveBlock)), None)
    selected = directive or selected
    return DynamicsMode(selected or DynamicsMode.POINT_MASS)
####


def _kinematic_state(state: RuntimeState) -> Kinematic6DofState:
    """Build the kinematic attitude sidecar from an ECFC Cartesian state."""

    required = ("x", "y", "z", "xdt", "ydt", "zdt")
    if not all(name in state.named for name in required):
        missing = tuple(name for name in required if name not in state.named)
        raise FidelitySetupError(
            "missing-kinematic-state-channel",
            f"kinematic-6dof mode requires x, y, z, xdt, ydt, and zdt state variables; missing: {', '.join(missing)}",
            "declare all ECFC position and velocity channels in the initial state",
            field="initial",
        )
    return Kinematic6DofState(
        time=state.time,
        position=FrameVector3(Vector3(*(state.named[name] for name in ("x", "y", "z"))), Frame.ECFC),
        velocity=FrameVector3(Vector3(*(state.named[name] for name in ("xdt", "ydt", "zdt"))), Frame.ECFC),
    )
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


def _optimize_seed_parameters(problem: Problem, values: Mapping[str, float] = {}) -> dict[str, float]:
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
                try:
                    value = evaluate_expression(assignment.value, values, values)
                except KeyError:
                    # The normal optimizer will resolve this expression again
                    # after the case has been lowered.  Keeping the seed absent
                    # here preserves the source expression instead of inventing
                    # a numeric starting point.
                    continue
                parameters[f"optimize-{loop}-{index}"] = value
                parameters.setdefault(f"optimize-{index}", value)
    return parameters
####


def _is_supported_optimization(block: OptimizeBlock) -> bool:
    """Return whether an optimization has the currently executable shape."""

    names = {assignment.name.casefold() for assignment in block.controls}
    parameter_indices = {name.removeprefix("par-") for name in names if name.startswith("par-")}
    return bool(parameter_indices) and block.objective_variable is not None and block.objective_mode in {"min", "max"} and all({f"lo-{index}", f"hi-{index}"}.issubset(names) for index in parameter_indices)
####


def _is_supported_search(block: SearchBlock) -> bool:
    """Return whether a search has the bounded root controls implemented here.

    The manual requires only the initial estimate, interval, and bounds;
    convergence and reporting controls have documented defaults.
    """

    controls = {assignment.name.casefold() for assignment in block.controls}
    return (
        block.search_id is not None
        and block.objective is not None
        and block.objective.operator in {"=", "<", ">"}
        and block.objective.left.expression is not None
        and block.objective.right is not None
        and {"xlo", "xhi", "xest", "dx"}.issubset(controls)
    )
####


def _resolve_search_case(
    case: RuntimeCase,
    document: LoweredDocument,
    search: SearchBlock,
    *,
    max_steps: int,
    trial_results: list[ExecutionResult] | None = None,
    integrator: IntegratorName | None = None,
) -> RuntimeCase:
    """Find one bounded search boundary using the parameters resolved before it."""

    source_problem = document.source_problem
    if source_problem is None:
        raise ValueError("search requires one source problem")
    objective = search.objective
    if search.search_id is None or objective is None or objective.operator not in {"=", "<", ">"}:
        raise ValueError("runtime search requires one equality or inequality objective")
    controls = {assignment.name.casefold(): evaluate_expression(assignment.value, {}, case.parameters) for assignment in search.controls}
    for name, default in (
        ("tol", 1.0e-6),
        ("maxitr", 20.0),
        ("xref", 1.0),
        ("fref", 1.0),
        ("integ", 1.0),
        ("print", 0.0),
    ):
        controls.setdefault(name, default)
    ####
    required = ("xlo", "xhi", "xest", "dx")
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
            _lower_case_for_runtime(source_problem, parameters, case.index, document.tables, document.unit_settings, integrator).problem,
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
            _lower_case_for_runtime(source_problem, parameters, case.index, document.tables, document.unit_settings, integrator).problem,
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
        _lower_case_for_runtime(source_problem, parameters, case.index, document.tables, document.unit_settings, integrator).problem,
    )
####


def _resolve_optimize_case(
    case: RuntimeCase,
    document: LoweredDocument,
    optimize: OptimizeBlock,
    *,
    max_steps: int,
    integrator: IntegratorName | None = None,
) -> RuntimeCase:
    """Find a bounded optimum by recomputing the trajectory for each candidate."""

    source_problem = document.source_problem
    if source_problem is None:
        raise ValueError("optimization requires a source problem")
    if not _is_supported_optimization(optimize) or optimize.objective_variable is None or optimize.objective_mode is None:
        raise ValueError("optimization requires bounded parameter controls and an objective")
    controls = {
        assignment.name.casefold(): evaluate_expression(assignment.value, case.parameters, case.parameters)
        for assignment in optimize.controls
    }
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
    derivative_step = controls.get("dx", 1.0e-8)
    if derivative_step <= 0.0:
        raise ValueError("optimization dx must be positive")
    max_iterations = int(controls.get("maxitr", 50))
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
    static_trajectory_names = _optimization_static_trajectory_names(source_problem, endpoint_requirements) if integration_mode == 0 and _can_integrate_optimization_independently(case.problem) else frozenset()
    static_requirements = tuple(item for item in endpoint_requirements if item[0] in static_trajectory_names)
    dynamic_requirements = tuple(item for item in endpoint_requirements if item[0] not in static_trajectory_names)
    static_result: ExecutionResult | None = None
    adaptive_trial_steps: int | None = None
    consecutive_incomplete_trials = 0
    if static_requirements:
        fixed_trial = RuntimeCase(
            case.index,
            base_parameters,
            _lower_case_for_runtime(source_problem, base_parameters, case.index, document.tables, document.unit_settings, integrator).problem,
        )
        _apply_trial_integration_mode(fixed_trial.problem, static_requirements, integration_mode)
        fixed_problem = _restrict_optimization_problem(fixed_trial.problem, static_requirements)
        static_result = compute_trajectories(
            fixed_problem,
            max_steps=max_steps,
            stop_when=_optimization_endpoint_stop_when(static_requirements),
        )
        if not static_result.completed:
            raise RuntimeError("fixed optimization trajectory did not reach its qualified endpoint")

    def execute_candidate(candidate: Sequence[float], *, accurate: bool = False) -> tuple[ExecutionResult, Mapping[str, float]]:
        nonlocal adaptive_trial_steps, consecutive_incomplete_trials
        point = tuple(float(value) for value in candidate)
        if not accurate:
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
            _lower_case_for_runtime(source_problem, parameters, case.index, document.tables, document.unit_settings, integrator).problem,
        )
        _apply_trial_integration_mode(trial.problem, dynamic_requirements or endpoint_requirements, integration_mode)
        dynamic_requirements_for_run = dynamic_requirements or endpoint_requirements
        dynamic_problem = trial.problem
        if not dynamic_problem.metadata.get("coupled_trajectories", False):
            dynamic_problem = _restrict_optimization_problem(dynamic_problem, dynamic_requirements_for_run)
        if not accurate:
            # Optimization trials only rank nearby candidates. Use the existing
            # fixed-step integrator for those repeated evaluations; the chosen
            # point is rerun below with the production RKF45 path.
            for vehicle in dynamic_problem.vehicles.values():
                if vehicle.integrator.casefold() == "rkf45":
                    vehicle.integrator = "rk4"
        trial_steps = max_steps if adaptive_trial_steps is None else min(max_steps, adaptive_trial_steps)
        dynamic_result = compute_trajectories(
            dynamic_problem,
            max_steps=trial_steps,
            stop_when=_optimization_endpoint_stop_when(dynamic_requirements_for_run),
            synchronize_vehicles=not _can_integrate_optimization_independently(dynamic_problem),
        )
        if dynamic_result.completed and adaptive_trial_steps is None:
            observed_steps = max((len(states) for states in dynamic_result.states.values()), default=0)
            adaptive_trial_steps = min(max_steps, observed_steps + 256)
        if dynamic_result.completed:
            consecutive_incomplete_trials = 0
        else:
            consecutive_incomplete_trials += 1
            if consecutive_incomplete_trials >= 5:
                raise RuntimeError("optimization produced five consecutive trials without reaching its qualified endpoint")
        ####
        if static_result is None:
            result = dynamic_result
        else:
            result = ExecutionResult(
                states={**static_result.states, **dynamic_result.states},
                completed=static_result.completed and dynamic_result.completed,
                stop_reason=dynamic_result.stop_reason,
            )
        cached = (result, parameters)
        if not accurate:
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
            if result.stop_reason == "state_stall":
                raise RuntimeError("optimization trial stalled before reaching a qualified endpoint")
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
    # Calibrate the candidate budget from the documented starting trajectory;
    # SciPy is free to request another point before evaluating its nominal x0.
    execute_candidate(initial)
    optimizer = resolve_optimize_block(
        objective,
        bounds,
        equality_constraints=equality_constraints,
        inequality_constraints=inequality_constraints,
        tolerance=controls.get("tol", 1e-6),
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
                tolerance=controls.get("tol", 1e-6),
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
    # Re-evaluate the selected point with the production integrator before
    # checking constraints or returning the resolved case.
    candidate_cache.pop(tuple(candidate), None)
    candidate_cache[tuple(candidate)] = execute_candidate(candidate, accurate=True)
    updates = {f"optimize-{loop}-{index}": value for index, value in zip(indices, candidate, strict=True)}
    updates.update({f"optimize-{index}": value for index, value in zip(indices, candidate, strict=True)})
    constraint_tolerance = max(10.0 * controls.get("tol", 1e-6), 1e-6)
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
    resolved = _lower_case_for_runtime(source_problem, parameters, case.index, document.tables, document.unit_settings, integrator).problem
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
    score_cache: dict[tuple[float, ...], float] = {}

    def score(point: tuple[float, ...]) -> float:
        cached = score_cache.get(point)
        if cached is None:
            cached = objective(point)
            score_cache[point] = cached
        return cached
    ####

    current = score(candidate)
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
            best = min(points, key=score)
            best_score = score(best)
            if best_score < current:
                candidate, current = best, best_score
                changed = True
            else:
                steps[position] *= 0.5
        if not changed:
            if max(steps, default=0.0) <= tolerance:
                break
    return candidate
####


def _can_integrate_optimization_independently(problem: RuntimeProblem) -> bool:
    """Guard the optimizer-only independent execution path.

    Cross-trajectory dependencies and guidance/radar evaluators require the
    synchronized engine because their derivatives can observe another vehicle.
    """

    return not any(vehicle.dependencies for vehicle in problem.vehicles.values()) and not problem.metadata.get("coupled_trajectories", False)
####


def _restrict_optimization_problem(
    problem: RuntimeProblem,
    requirements: Sequence[tuple[str, int]],
) -> RuntimeProblem:
    """Keep only endpoint trajectories and their activation dependencies."""

    required = {name for name, _ in requirements}
    while True:
        dependencies = {
            dependency
            for name in required
            for dependency in problem.vehicles[name].dependencies
            if dependency in problem.vehicles
        }
        expanded = required | dependencies
        if expanded == required:
            break
        required = expanded
    ####
    if len(required) == len(problem.vehicles):
        return problem
    return RuntimeProblem(
        vehicles={name: problem.vehicles[name] for name in required},
        print_times=problem.print_times,
        table_knots=problem.table_knots,
        final_time=problem.final_time,
        metadata=dict(problem.metadata),
        event_history=list(problem.event_history),
        required_truth_times=problem.required_truth_times,
        sensor_clocks=problem.sensor_clocks,
        transition_history=list(problem.transition_history),
        sensor_bus=problem.sensor_bus,
    )
####


def _contains_optimization_parameter(value: object) -> bool:
    """Find optimize placeholders in a typed model without flattening expressions."""

    if isinstance(value, ParameterExpression):
        return value.family.casefold() == "optimize"
    if isinstance(value, BaseModel):
        return any(_contains_optimization_parameter(child) for child in value.__dict__.values())
    if isinstance(value, Mapping):
        return any(_contains_optimization_parameter(child) for child in value.values())
    if isinstance(value, (list, tuple, set, frozenset)):
        return any(_contains_optimization_parameter(child) for child in value)
    return False
####


def _optimization_static_trajectory_names(
    problem: Problem,
    requirements: Sequence[tuple[str, int]],
) -> frozenset[str]:
    """Return qualified endpoint trajectories independent of optimization inputs."""

    trajectories = {str(trajectory.number): trajectory for trajectory in problem.trajectories}
    return frozenset(
        name
        for name, _ in requirements
        if name in trajectories and not _contains_optimization_parameter(trajectories[name])
    )
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
    root.mkdir(parents=True, exist_ok=True)
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

    functions: dict[str, Callable[[float], float]] = {
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
    """Interpolate scheduled outputs according to their channel semantics."""

    values = tuple(left + fraction * (right - left) for left, right in zip(start.values, end.values, strict=True))
    names = set(start.named) | set(end.named)
    named = {name: _interpolate_named_value(name, start.named, end.named, fraction) for name in names}
    named["time"] = time
    return RuntimeState(time, values, end.frame, named, end.value_names, end.segment_endpoints)
####


def _interpolate_named_value(name: str, start: Mapping[str, float], end: Mapping[str, float], fraction: float) -> float:
    """Apply step or shortest-angle semantics to a named output channel."""

    left = start.get(name, end.get(name, 0.0))
    right = end.get(name, start.get(name, 0.0))
    interpolation = _output_interpolation_kind(name)
    if interpolation in {"step", "event"}:
        return left if fraction < 1.0 else right
    if interpolation == "angle":
        delta = (right - left + math.pi) % (2.0 * math.pi) - math.pi
        return _wrap_angle_radians(left + fraction * delta)
    return left + fraction * (right - left)
####


def _output_interpolation_kind(name: str) -> str:
    spec = output_channel_spec(name)
    if spec is not None:
        return spec.interpolation
    lowered = name.casefold()
    return "angle" if lowered in {"alphat", "betae", "bankgc", "bankgd", "gamgc", "gamgd", "yawgc", "yawgd"} else "linear"
####


def _wrap_angle_radians(value: float) -> float:
    wrapped = ((value + math.pi) % (2.0 * math.pi)) - math.pi
    return math.pi if math.isclose(wrapped, -math.pi, rel_tol=0.0, abs_tol=1.0e-12) else wrapped
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
