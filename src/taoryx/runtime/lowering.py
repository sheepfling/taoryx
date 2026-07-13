"""Lower parsed TAOS documents into executable runtime cases."""

from __future__ import annotations

import json
import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

from taoryx.language.expressions import BinaryExpression, ExpressionType, NameExpression, TableReferenceExpression
from taoryx.language.models import (
    AeroBlock,
    AtmosBlock,
    DefineBlock,
    EarthBlock,
    EgsBlock,
    FileBlock,
    FlyBlock,
    IncrementBlock,
    InitialBlock,
    IntegrationBlock,
    OptimizeBlock,
    OptimizeConstraint,
    PrintBlock,
    Problem,
    ProblemDocument,
    PropulsionBlock,
    ResetBlock,
    SearchBlock,
    Segment,
    SummarizeBlock,
    SurveyBlock,
    TableDocument,
    TableOperation,
    WhenBlock,
)
from taoryx.optimization import han_powell_rqp
from taoryx.searches import secant_bracketed_root
from taoryx.tables import ExtrapolationMode, PreparedTable, TableEvaluationContext, evaluate_full_table, prepare_table

from .common import EventCondition, RuntimeProblem, RuntimeState, RuntimeVehicle
from .engine import ExecutionResult, compute_trajectories
from .expressions import evaluate_definition_program, evaluate_expression
from .summaries import evaluate_summary
from .surveys import generate_survey_cases


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

    def evaluate(self, values: Mapping[str, float], tables: Mapping[str, RuntimeTable] | None = None) -> float:
        if self.prepared is None:
            prepared_tables = {name: table.prepared for name, table in (tables or {}).items() if table.prepared is not None}
            return evaluate_full_table(self.operations, TableEvaluationContext.from_values(values, prepared_tables)).value
        from taoryx.tables import interpolate_nd

        return interpolate_nd(self.prepared, tuple(values[name] for name in self.independent_variables))
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
    output_files: tuple[tuple[str, tuple[str, ...]], ...]
    summaries: tuple[tuple[str, tuple[tuple[str, str | None], ...]], ...]
    unsupported_features: tuple[str, ...] = ()
    searches: tuple[SearchBlock, ...] = ()
    optimizations: tuple[OptimizeBlock, ...] = ()
    source_problem: Problem | None = None


def lower_tables(document: TableDocument) -> dict[str, RuntimeTable]:
    """Convert parsed simple tables to prepared interpolation tables."""

    tables: dict[str, RuntimeTable] = {}
    for definition in document.tables:
        if definition.format == "full":
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
        independent = tuple(definition.independent_variables)
        assignments = {item.name.casefold(): tuple(item.values) for item in definition.assignments}
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
            prepare_table(tuple(assignments[name.casefold()] for name in independent), outputs[0].values, extrapolation=mode),
            (),
            _table_reference_area(definition.options),
        )
    return tables
####


def lower_problem_document(document: ProblemDocument, tables: Mapping[str, RuntimeTable] | None = None) -> LoweredDocument:
    """Lower one parsed problem and all survey combinations."""

    if len(document.problems) != 1:
        raise ValueError("runtime execution requires exactly one problem")
    problem = document.problems[0]
    surveys = _survey_parameters(problem)
    cases = generate_survey_cases(surveys) if surveys else ({},)
    search_seed = _search_seed_parameters(problem)
    optimize_seed = _optimize_seed_parameters(problem)
    return LoweredDocument(
        tuple(_lower_case(problem, {**search_seed, **optimize_seed, **parameters}, index, tables or {}) for index, parameters in enumerate(cases, start=1)),
        tables or {},
        _print_variables(problem),
        _output_files(problem),
        _summary_specs(problem),
        _unsupported_features(problem),
        tuple(block for block in problem.blocks if isinstance(block, SearchBlock)),
        tuple(block for block in problem.blocks if isinstance(block, OptimizeBlock)),
        problem,
    )
####


def execute_lowered(document: LoweredDocument, *, output_dir: str = ".", max_steps: int = 100000) -> tuple[ExecutionResult, ...]:
    """Execute cases in order and emit declared output products."""

    from pathlib import Path

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    results: list[ExecutionResult] = []
    for case in document.cases:
        executable_case = _resolve_search_case(case, document, max_steps=max_steps) if document.searches else case
        if document.optimizations:
            executable_case = _resolve_optimize_case(executable_case, document, max_steps=max_steps)
        result = compute_trajectories(executable_case.problem, max_steps=max_steps)
        results.append(result)
        _write_outputs(executable_case.index, result, document, destination)
    return tuple(results)
####


def _lower_case(problem: Problem, parameters: Mapping[str, float], index: int, tables: Mapping[str, RuntimeTable]) -> RuntimeCase:
    earth_mu, earth_omega = _earth_parameters(problem, parameters)
    environment_evaluator = _atmosphere_evaluator(problem)
    trajectories = {trajectory.number: trajectory for trajectory in problem.trajectories}
    vehicles: list[RuntimeVehicle] = []
    for trajectory in problem.trajectories:
        initial = next((block for block in trajectory.blocks if isinstance(block, InitialBlock)), None)
        if initial is None:
            raise ValueError(f"trajectory {trajectory.number} has no initial block")
        source_trajectory = trajectories.get(initial.source_trajectory) if initial.source_trajectory is not None else None
        source_initial = next((block for block in source_trajectory.blocks if isinstance(block, InitialBlock)), None) if source_trajectory is not None else None
        names, values, named, start_time = _initial_values(source_initial or initial, parameters)
        inherited = source_trajectory is not None and source_initial is not None
        if environment_evaluator is not None:
            named.update(environment_evaluator(named))
        definitions = {
            assignment.name.casefold(): assignment.value
            for block in trajectory.blocks
            if isinstance(block, DefineBlock) and not block.integral
            for assignment in block.assignments
        }
        integration = next((block for segment in trajectory.segments for block in segment.blocks if isinstance(block, IntegrationBlock)), None)
        step = _assignment_value(integration, "dt", parameters, 1.0)
        condition_list: list[EventCondition] = []
        segment_events: dict[int, tuple[EventCondition, ...]] = {}
        segment_by_number = {segment.number: segment for segment in trajectory.segments}
        active_segment = {"number": trajectory.start_segment}
        vehicle_environment = _vehicle_environment_evaluator(environment_evaluator, segment_by_number, active_segment, tables, parameters)
        for segment in trajectory.segments:
            segment_conditions: list[EventCondition] = []
            for block in segment.blocks:
                if not isinstance(block, WhenBlock) or block.condition is None:
                    continue
                expression = block.condition

                def condition_function(state: RuntimeState, expression: ExpressionType = expression) -> float:
                    return _event_residual(expression, state.named, parameters)
                ####

                event = EventCondition(f"trajectory-{trajectory.number}-when-{segment.number}-{len(segment_conditions) + 1}", condition_function, block.action or "stop")
                condition_list.append(event)
                segment_conditions.append(event)
            ####
            segment_events[segment.number] = tuple(segment_conditions)
        ####
        conditions = segment_events.get(trajectory.start_segment, ())

        vehicle_ref: list[RuntimeVehicle] = []
        event_handlers: dict[str, Callable[[RuntimeState], RuntimeState]] = {}
        for segment in trajectory.segments:
            target = next((block.target_segment for block in segment.blocks if isinstance(block, WhenBlock) and block.action == "goto"), None)
            for event in segment_events[segment.number]:
                def handler(
                    state: RuntimeState,
                    segment: Segment = segment,
                    target: int | None = target,
                    event_segments: Mapping[int, tuple[EventCondition, ...]] = segment_events,
                    reference: list[RuntimeVehicle] = vehicle_ref,
                    active_segment_ref: dict[str, int] = active_segment,
                ) -> RuntimeState:
                    updated = _apply_segment_updates(state, segment, parameters)
                    if target is not None and reference:
                        reference[0].events = event_segments.get(target, ())
                        reference[0].segment_number = target
                        active_segment_ref["number"] = target
                        named = dict(updated.named)
                        named["tseg"] = 0.0
                        updated = RuntimeState(updated.time, updated.values, updated.frame, named, updated.value_names)
                    return updated
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
        ) -> tuple[float, ...]:
            rates = {name: 0.0 for name in state_names}
            _assemble_ecfc_rates(rates, state.named, earth_mu, earth_omega)
            velocity = state.named.get("vel", 0.0)
            gamma = math.radians(state.named.get("gama", 0.0))
            if "alt" in rates:
                rates["alt"] = velocity * math.sin(gamma) if "gama" in state.named else velocity
            if "range" in rates:
                rates["range"] = velocity * math.cos(gamma)
            if "plength" in rates:
                rates["plength"] = abs(velocity)
            if "tseg" in rates:
                rates["tseg"] = 1.0
            if "tmark" in rates:
                rates["tmark"] = 1.0
            segment = segments.get(active_segment_ref["number"])
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
            mass = max(abs(state.named.get("wt", state.named.get("mass", 1.0))), 1e-12)
            aero_acceleration = _aerodynamic_acceleration(segment, state.named, state_tables, parameters, mass) if segment is not None else (0.0, 0.0, 0.0)
            if "vel" in rates:
                rates["vel"] = thrust / mass + aero_acceleration[0]
            for name, value in zip(("xdt", "ydt", "zdt"), aero_acceleration, strict=True):
                if name in rates:
                    rates[name] += value + (thrust / mass if name == "xdt" else 0.0)
            if "wt" in rates:
                rates["wt"] = -mass_rate
            if "mass" in rates:
                rates["mass"] = -mass_rate
            for table in state_tables.values():
                if table.output_variable.casefold() in rates and table.independent_variables:
                    rates[table.output_variable.casefold()] = table.evaluate(state.named, state_tables)
            return tuple(rates[name] for name in state_names)
        ####

        def stop_when(state: RuntimeState, event_conditions: tuple[EventCondition, ...] = conditions) -> bool:
            return any(condition.function(state) >= 0.0 for condition in event_conditions)
        ####

        vehicle = RuntimeVehicle(
            str(trajectory.number),
            RuntimeState(start_time, values, named=named, value_names=names),
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
            parameters=parameters,
            table_evaluators=_table_evaluators(tables),
            environment_evaluator=vehicle_environment,
            event_handlers=event_handlers,
        )
        vehicle_ref.append(vehicle)
        if definitions:
            derived = evaluate_definition_program(definitions, named, parameters=parameters, table_evaluators=_table_evaluators(tables))
            vehicle.state = RuntimeState(start_time, values, named=derived, value_names=names)
            vehicle.history[0] = vehicle.state
        vehicles.append(vehicle)
    runtime = RuntimeProblem({vehicle.name: vehicle for vehicle in vehicles})
    runtime.metadata["parameters"] = dict(parameters)
    runtime.metadata["tables"] = tables
    return RuntimeCase(index, parameters, runtime)
####


def _initial_values(block: InitialBlock, parameters: Mapping[str, float]) -> tuple[tuple[str, ...], tuple[float, ...], dict[str, float], float]:
    named: dict[str, float] = {}
    for assignment in block.assignments:
        named[assignment.name.casefold()] = evaluate_expression(assignment.value, named, parameters)
    start_time = named.pop("time", 0.0)
    named.setdefault("rho", 0.0)
    named.setdefault("mach", 0.0)
    named.setdefault("dynprs", 0.0)
    named.setdefault("nx", 0.0)
    named.setdefault("plength", 0.0)
    named.setdefault("tseg", 0.0)
    named.setdefault("tmark", 0.0)
    names = tuple(named)
    return names, tuple(named[name] for name in names), {**named, "time": start_time}, start_time
####


def _earth_parameters(problem: Problem, parameters: Mapping[str, float]) -> tuple[float, float]:
    """Resolve the scalar ``gm`` and ``omega`` settings used by ECFC dynamics."""

    earth = next((block for block in problem.blocks if isinstance(block, EarthBlock)), None)
    if earth is None:
        return 0.0, 0.0
    values = {
        assignment.name.casefold(): evaluate_expression(assignment.value, {}, parameters)
        for assignment in earth.assignments
    }
    return values.get("gm", 0.0), values.get("omega", 0.0)
####


def _table_reference_area(options: Mapping[str, str | float]) -> float | None:
    value = options.get("sref")
    return None if value is None else float(value)
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
        reference_area = 1.0
        if isinstance(assignment.value, TableReferenceExpression):
            reference_area = tables.get(assignment.value.name.casefold(), RuntimeTable("", "", (), "", None)).reference_area or 1.0
        acceleration += dynamic_pressure * reference_area * coefficient / mass
    if "xdt" in named or "ydt" in named or "zdt" in named:
        components = tuple(named.get(name, 0.0) for name in ("xdt", "ydt", "zdt"))
        norm = math.sqrt(sum(value * value for value in components))
        if norm > 0.0:
            return (-acceleration * components[0] / norm, -acceleration * components[1] / norm, -acceleration * components[2] / norm)
    return (-acceleration if named.get("vel", 0.0) >= 0.0 else acceleration, 0.0, 0.0)
####


def _vehicle_environment_evaluator(
    base_evaluator: Callable[[Mapping[str, float]], Mapping[str, float]] | None,
    segments: Mapping[int, Segment],
    active_segment: Mapping[str, int],
    tables: Mapping[str, RuntimeTable],
    parameters: Mapping[str, float],
) -> Callable[[Mapping[str, float]], Mapping[str, float]]:
    """Combine atmosphere refresh with active-segment force observables."""

    def evaluate(values: Mapping[str, float]) -> Mapping[str, float]:
        result = dict(base_evaluator(values) if base_evaluator is not None else {})
        segment = segments.get(active_segment["number"])
        if segment is None:
            return result
        coefficients: list[float] = []
        thrust = 0.0
        mdot = 0.0
        for block in segment.blocks:
            if isinstance(block, AeroBlock):
                assignment = next((item for item in block.assignments if item.name.casefold() == "ca"), None)
                if assignment is not None:
                    coefficients.append(evaluate_expression(assignment.value, {**values, **result}, parameters, tables=_table_evaluators(tables)))
            elif isinstance(block, PropulsionBlock):
                for assignment in block.assignments:
                    if assignment.name.casefold() == "thrust":
                        thrust += evaluate_expression(assignment.value, {**values, **result}, parameters, tables=_table_evaluators(tables))
                    elif assignment.name.casefold() == "mdot":
                        mdot += evaluate_expression(assignment.value, {**values, **result}, parameters, tables=_table_evaluators(tables))
            elif isinstance(block, FlyBlock) and block.guidance_variable is not None and block.value is not None:
                try:
                    result[block.guidance_variable.casefold()] = evaluate_expression(block.value, {**values, **result}, parameters)
                except (KeyError, TypeError, ValueError):
                    # Wildcard and indirect guidance are resolved by later control loops.
                    pass
        if coefficients:
            result["ca"] = sum(coefficients)
        if thrust:
            result["thrust"] = thrust
        if mdot:
            result["mdot"] = mdot
        return result
    ####

    return evaluate
####


def _atmosphere_evaluator(problem: Problem) -> Callable[[Mapping[str, float]], Mapping[str, float]] | None:
    """Build a tabulated user-atmosphere evaluator for runtime state refresh."""

    atmosphere = next((block for block in problem.blocks if isinstance(block, AtmosBlock)), None)
    if atmosphere is None or atmosphere.model != "user" or not atmosphere.rows or not atmosphere.columns:
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
        speed = abs(values.get("vair", values.get("vel", 0.0)))
        density = result.get("rho", values.get("rho", 0.0))
        sound_speed = result.get("sndspd", values.get("sndspd", 0.0))
        if sound_speed > 0.0:
            result["mach"] = speed / sound_speed
        result["dynprs"] = 0.5 * density * speed * speed
        return result
    ####

    return evaluate
####


def _bracket_atmosphere_rows(rows: Sequence[tuple[float, ...]], altitude: float, altitude_index: int) -> tuple[tuple[float, ...], tuple[float, ...]]:
    if altitude <= rows[0][altitude_index]:
        return rows[0], rows[0]
    if altitude >= rows[-1][altitude_index]:
        return rows[-1], rows[-1]
    index = next(index for index in range(len(rows) - 1) if rows[index][altitude_index] <= altitude <= rows[index + 1][altitude_index])
    return rows[index], rows[index + 1]
####


def _assemble_ecfc_rates(rates: dict[str, float], named: Mapping[str, float], gravitational_parameter: float, rotation_rate: float) -> None:
    """Add ECFC Cartesian kinematics and rotating point-mass acceleration."""

    position_names = ("x", "y", "z")
    velocity_names = ("xdt", "ydt", "zdt")
    if not all(name in named for name in (*position_names, *velocity_names)):
        return
    for position_name, velocity_name in zip(position_names, velocity_names, strict=True):
        if position_name in rates:
            rates[position_name] = named[velocity_name]
    position = tuple(named[name] for name in position_names)
    velocity = tuple(named[name] for name in velocity_names)
    radius_squared = sum(component * component for component in position)
    gravity_scale = -gravitational_parameter / (radius_squared ** 1.5) if gravitational_parameter and radius_squared > 0.0 else 0.0
    gravity = tuple(gravity_scale * component for component in position)
    coriolis = (2.0 * rotation_rate * velocity[1], -2.0 * rotation_rate * velocity[0], 0.0)
    centrifugal = (rotation_rate * rotation_rate * position[0], rotation_rate * rotation_rate * position[1], 0.0)
    for velocity_name, value in zip(velocity_names, (gravity[0] + coriolis[0] + centrifugal[0], gravity[1] + coriolis[1] + centrifugal[1], gravity[2]), strict=True):
        if velocity_name in rates:
            rates[velocity_name] = value
####


def _apply_segment_updates(state: RuntimeState, segment: Segment, parameters: Mapping[str, float]) -> RuntimeState:
    values = list(state.values)
    named = dict(state.named)
    for block in segment.blocks:
        if not isinstance(block, (IncrementBlock, ResetBlock)):
            continue
        for assignment in block.assignments:
            name = assignment.name.casefold()
            value = evaluate_expression(assignment.value, named, parameters)
            if isinstance(block, IncrementBlock):
                value += named.get(name, 0.0)
            named[name] = value
            if name in state.value_names:
                values[state.value_names.index(name)] = value
        ####
    ####
    return RuntimeState(state.time, tuple(values), state.frame, named, state.value_names)
####


def _event_residual(expression: ExpressionType, values: Mapping[str, float], parameters: Mapping[str, float]) -> float:
    if isinstance(expression, BinaryExpression) and expression.operator in {"<", ">", "<=", ">=", "=", "==", "!="}:
        left = evaluate_expression(expression.left, values, parameters)
        right = evaluate_expression(expression.right, values, parameters)
        return left - right
    return evaluate_expression(expression, values, parameters)
####


def _table_evaluators(tables: Mapping[str, RuntimeTable]) -> dict[str, Callable[[Mapping[str, float]], float]]:
    evaluators: dict[str, Callable[[Mapping[str, float]], float]] = {}
    for name, table in tables.items():
        def evaluate(values: Mapping[str, float], table: RuntimeTable = table) -> float:
            return table.evaluate(values, tables)
        ####

        evaluators[name] = evaluate
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
    return tuple(variable for trajectory in problem.trajectories for block in trajectory.blocks if isinstance(block, PrintBlock) for variable in block.variables)
####


def _output_files(problem: Problem) -> tuple[tuple[str, tuple[str, ...]], ...]:
    problem_outputs = ((block.filename, tuple(block.variables)) for block in problem.blocks if isinstance(block, (FileBlock, EgsBlock)) and block.filename)
    trajectory_outputs = (
        (block.filename, tuple(block.variables))
        for trajectory in problem.trajectories
        for block in trajectory.blocks
        if isinstance(block, (FileBlock, EgsBlock)) and block.filename
    )
    return tuple((*problem_outputs, *trajectory_outputs))
####


def _summary_specs(problem: Problem) -> tuple[tuple[str, tuple[tuple[str, str | None], ...]], ...]:
    return tuple((block.name or "summary", tuple((operation.operation, operation.operand.text if operation.operand else None) for operation in block.operations)) for block in problem.blocks if isinstance(block, SummarizeBlock))
####


def _unsupported_features(problem: Problem) -> tuple[str, ...]:
    """List parsed control blocks that the current runtime cannot execute."""

    features: list[str] = []
    if any(not _is_supported_optimization(block) for block in problem.blocks if isinstance(block, OptimizeBlock)):
        features.append("optimize")
    return tuple(features)
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
                parameters[f"optimize-{name.removeprefix('par-')}"] = evaluate_expression(assignment.value, {}, {})
    return parameters
####


def _is_supported_optimization(block: OptimizeBlock) -> bool:
    """Return whether an optimization has the currently executable shape."""

    names = {assignment.name.casefold() for assignment in block.controls}
    parameter_indices = {name.removeprefix("par-") for name in names if name.startswith("par-")}
    return bool(parameter_indices) and block.objective_variable is not None and block.objective_mode in {"min", "max"} and all({f"lo-{index}", f"hi-{index}"}.issubset(names) for index in parameter_indices)
####


def _resolve_search_case(case: RuntimeCase, document: LoweredDocument, *, max_steps: int) -> RuntimeCase:
    """Find one equality-search root by rebuilding and executing each candidate."""

    source_problem = document.source_problem
    if source_problem is None or len(document.searches) != 1:
        raise ValueError("runtime currently supports one search loop per case")
    search = document.searches[0]
    objective = search.objective
    if search.search_id is None or objective is None or objective.operator != "=":
        raise ValueError("runtime search requires one equality objective")
    controls = {assignment.name.casefold(): evaluate_expression(assignment.value, {}, case.parameters) for assignment in search.controls}
    required = ("xlo", "xhi", "tol", "maxitr")
    if any(name not in controls for name in required):
        raise ValueError(f"search requires controls {required!r}")
    target = objective.right
    if target is None or target.expression is None or objective.left.expression is None:
        raise ValueError("search objective is incomplete")
    target_expression = target.expression

    def residual(candidate: float) -> float:
        parameters = {**case.parameters, f"search-{search.search_id}": candidate}
        trial = RuntimeCase(case.index, parameters, _lower_case(source_problem, parameters, case.index, document.tables).problem)
        result = compute_trajectories(trial.problem, max_steps=max_steps)
        trajectory_name = str(objective.left.trajectory or 1)
        history = result.states.get(trajectory_name, ())
        if not history:
            raise RuntimeError(f"search trajectory {trajectory_name} produced no history")
        state = history[-1]
        left = _output_value(state, objective.left.text)
        right = evaluate_expression(target_expression, state.named, parameters)
        return left - right
    ####

    root = secant_bracketed_root(
        residual,
        (controls["xlo"], controls["xhi"]),
        controls["tol"],
        max_iterations=int(controls["maxitr"]),
    )
    if not root.converged:
        raise RuntimeError(f"search {search.search_id} did not converge: {root.status}")
    parameters = {**case.parameters, f"search-{search.search_id}": root.root}
    return RuntimeCase(case.index, parameters, _lower_case(source_problem, parameters, case.index, document.tables).problem)
####


def _resolve_optimize_case(case: RuntimeCase, document: LoweredDocument, *, max_steps: int) -> RuntimeCase:
    """Find a bounded optimum by recomputing the trajectory for each candidate."""

    source_problem = document.source_problem
    if source_problem is None or len(document.optimizations) != 1:
        raise ValueError("runtime currently supports one scalar optimization per case")
    optimize = document.optimizations[0]
    if not _is_supported_optimization(optimize) or optimize.objective_variable is None or optimize.objective_mode is None:
        raise ValueError("optimization requires bounded parameter controls and an objective")
    controls = {assignment.name.casefold(): evaluate_expression(assignment.value, {}, case.parameters) for assignment in optimize.controls}
    indices = tuple(sorted(name.removeprefix("par-") for name in controls if name.startswith("par-")))
    objective_name = optimize.objective_variable

    def execute_candidate(candidate: Sequence[float]) -> ExecutionResult:
        parameters = {**case.parameters, **{f"optimize-{index}": value for index, value in zip(indices, candidate, strict=True)}}
        trial = RuntimeCase(case.index, parameters, _lower_case(source_problem, parameters, case.index, document.tables).problem)
        return compute_trajectories(trial.problem, max_steps=max_steps)

    def endpoint_value(endpoint: object, result: ExecutionResult, parameters: Mapping[str, float]) -> float:
        trajectory = str(getattr(endpoint, "trajectory", None) or getattr(endpoint, "trajectory_subscript", None) or 1)
        history = result.states.get(trajectory, ())
        if not history:
            raise RuntimeError(f"optimization trajectory {trajectory} produced no history")
        state = history[-1]
        expression = getattr(endpoint, "expression", None)
        if isinstance(expression, NameExpression):
            return _output_value(state, expression.name)
        if expression is not None:
            return evaluate_expression(expression, state.named, parameters)
        return _output_value(state, str(getattr(endpoint, "text", "")))
    ####

    def objective(candidate: tuple[float, ...]) -> float:
        result = execute_candidate(candidate)
        history = result.states.get(str(optimize.trajectory or 1), ())
        if not history:
            raise RuntimeError(f"optimization trajectory {optimize.trajectory or 1} produced no history")
        value = _output_value(history[-1], objective_name)
        return -value if optimize.objective_mode == "max" else value
    ####

    equality_constraints: list[Callable[[tuple[float, ...]], float]] = []
    inequality_constraints: list[Callable[[tuple[float, ...]], float]] = []
    for constraint in optimize.constraints:
        def constraint_function(candidate: tuple[float, ...], constraint: OptimizeConstraint = constraint) -> float:
            parameters = {**case.parameters, **{f"optimize-{index}": value for index, value in zip(indices, candidate, strict=True)}}
            result = execute_candidate(candidate)
            left = endpoint_value(constraint.left, result, parameters)
            right = endpoint_value(constraint.right, result, parameters)
            return left - right if constraint.operator == "=" else right - left if constraint.operator == "<" else left - right
        ####

        (equality_constraints if constraint.operator == "=" else inequality_constraints).append(constraint_function)
    ####

    result = han_powell_rqp(
        objective,
        tuple(controls[f"par-{index}"] for index in indices),
        tuple((controls[f"lo-{index}"], controls[f"hi-{index}"]) for index in indices),
        equality_constraints=equality_constraints,
        inequality_constraints=inequality_constraints,
        tolerance=controls.get("tol", 1e-7),
        max_iterations=int(controls.get("maxitr", 100)),
    )
    candidate = result.parameters
    if len(indices) <= 2:
        def merit(point: tuple[float, ...]) -> float:
            equality_penalty = sum(function(point) ** 2 for function in equality_constraints)
            inequality_penalty = sum(max(0.0, -function(point)) ** 2 for function in inequality_constraints)
            return objective(point) + 1000.0 * (equality_penalty + inequality_penalty)
        ####

        for _ in range(2):
            for position, index in enumerate(indices):
                lower, upper = controls[f"lo-{index}"], controls[f"hi-{index}"]
                grid = tuple(lower + (upper - lower) * step / 20.0 for step in range(21))
                candidate = min(
                    (candidate[:position] + (value,) + candidate[position + 1:] for value in grid),
                    key=merit,
                )
            ####
        ####
    parameters = {**case.parameters, **{f"optimize-{index}": value for index, value in zip(indices, candidate, strict=True)}}
    return RuntimeCase(case.index, parameters, _lower_case(source_problem, parameters, case.index, document.tables).problem)
####


def _assignment_value(block: IntegrationBlock | None, name: str, parameters: Mapping[str, float], default: float) -> float:
    if block is None:
        return default
    assignment = next((item for item in block.assignments if item.name.casefold() == name.casefold()), None)
    return default if assignment is None else evaluate_expression(assignment.value, {}, parameters)
####


def _write_outputs(index: int, result: ExecutionResult, document: LoweredDocument, destination: Path) -> None:
    root = Path(destination)
    for name, history in result.states.items():
        if not history:
            continue
        variables = document.print_variables or history[-1].value_names
        if document.print_variables:
            path = root / (f"case-{index}-{name}.print" if index > 1 else f"{name}.print")
            path.write_text(_history_text(history, variables), encoding="utf-8")
        for filename, file_variables in document.output_files:
            selected = file_variables or variables
            output_name = filename if index == 1 else f"{Path(filename).stem}-case-{index}{Path(filename).suffix}"
            (root / output_name).write_text(_history_text(history, selected), encoding="utf-8")
    if document.summaries:
        values = _summary_values(result)
        payload = {name: _summary_chain(operations, values) for name, operations in document.summaries}
        (root / f"case-{index}-summaries.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
####


def _summary_values(result: ExecutionResult) -> dict[str, list[float]]:
    values: dict[str, list[float]] = {}
    for trajectory, history in result.states.items():
        if not history:
            continue
        for variable in history[-1].named:
            key = variable.casefold()
            series = [_output_value(state, key) for state in history]
            values.setdefault(key, []).extend(series)
            values[f"{key}[{trajectory}]"] = series
        for alias in ("xecfc", "yecfc", "zecfc", "xecfcdt", "yecfcdt", "zecfcdt"):
            series = [_output_value(state, alias) for state in history]
            values.setdefault(alias, []).extend(series)
            values[f"{alias}[{trajectory}]"] = series
    return values
####


def _history_text(history: Sequence[RuntimeState], variables: Sequence[str]) -> str:
    selected = tuple(variable for variable in variables if variable.casefold() != "time")
    rows = [" ".join(("time", *selected))]
    for state in history:
        rows.append(" ".join((str(state.time), *(str(_output_value(state, variable)) for variable in selected))))
    return "\n".join(rows) + "\n"
####


def _output_value(state: RuntimeState, variable: str) -> float:
    """Resolve historical output aliases such as ``xecfc`` to state names."""

    aliases = {"xecfc": "x", "yecfc": "y", "zecfc": "z", "xecfcdt": "xdt", "yecfcdt": "ydt", "zecfcdt": "zdt"}
    return state.named.get(aliases.get(variable.casefold(), variable.casefold()), float("nan"))
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
