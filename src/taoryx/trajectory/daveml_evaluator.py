"""Bounded DAVE-ML checkData evaluator for direct and regular-grid functions."""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from functools import lru_cache


@dataclass(frozen=True, slots=True)
class DAVEMLCheckResult:
    """One evaluated static DAVE-ML check shot."""

    case_id: str
    output_id: str
    expected: float
    actual: float | None
    absolute_error: float | None
    relative_error: float | None
    status: str
    reason: str | None = None
    absolute_tolerance: float | None = None

    def to_dict(self) -> dict[str, object]:
        """Return a JSON-safe result."""

        return {
            "case_id": self.case_id,
            "output_id": self.output_id,
            "expected": self.expected,
            "actual": self.actual,
            "absolute_error": self.absolute_error,
            "relative_error": self.relative_error,
            "status": self.status,
            "reason": self.reason,
            "absolute_tolerance": self.absolute_tolerance,
        }
        ####
####


@dataclass(frozen=True, slots=True)
class DAVEMLGraph:
    """Typed runtime view of one scalar DAVE-ML document."""

    document_id: str
    functions: Mapping[str, dict[str, object]]
    variables: Mapping[str, ET.Element]
    variable_names: Mapping[str, str]
    variable_units: Mapping[str, str]

    def evaluate(self, inputs: Mapping[str, float], outputs: Sequence[str]) -> dict[str, float]:
        """Evaluate named scalar outputs against explicit graph inputs."""

        mapped_inputs: dict[str, float] = {}
        for identifier, value in inputs.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                raise ValueError(f"DAVE-ML graph input {identifier!r} must be a finite scalar")
            mapped_inputs[self.variable_names.get(identifier, identifier)] = float(value)
        result: dict[str, float] = {}
        for output in outputs:
            identifier = self.variable_names.get(output, output)
            result[output] = _evaluate_variable(
                identifier,
                mapped_inputs,
                self.functions,
                self.variables,
                self.variable_names,
            )
        return result
        ####

    def unit_for(self, identifier: str) -> str | None:
        """Return the declared source unit for an ID or display name."""

        canonical_id = self.variable_names.get(identifier, identifier)
        return self.variable_units.get(canonical_id)
        ####

    def dimension_for(self, identifier: str) -> str | None:
        """Return the deterministic dimension signature for a source unit."""

        unit = self.unit_for(identifier)
        if unit is None:
            return None
        dimensions = {
            "nd": "1",
            "deg": "angle",
            "rad": "angle",
            "deg_rad": "angle",
            "rad_s": "angle/time",
            "f_s": "length/time",
            "ft_s": "length/time",
            "f": "length",
            "ft": "length",
            "f2": "length^2",
            "fracMAC": "1",
            "slug": "mass",
            "slug_ft2": "mass*length^2",
            "lb": "force",
            "lbf": "force",
            "s": "time",
            "s_rad": "time/angle",
        }
        return dimensions.get(unit.strip(), "unknown")
        ####

    def evaluate_vectors(
        self,
        inputs: Mapping[str, Sequence[float]],
        outputs: Sequence[str],
    ) -> dict[str, tuple[float, ...]]:
        """Evaluate vector constants and explicit vector inputs fail-closed."""

        mapped_inputs: dict[str, tuple[float, ...]] = {}
        for identifier, values in inputs.items():
            vector = tuple(float(value) for value in values)
            if not vector or not all(math.isfinite(value) for value in vector):
                raise ValueError(f"DAVE-ML vector input {identifier!r} must be finite and non-empty")
            mapped_inputs[self.variable_names.get(identifier, identifier)] = vector
        result: dict[str, tuple[float, ...]] = {}
        for output in outputs:
            identifier = self.variable_names.get(output, output)
            if identifier in mapped_inputs:
                result[output] = mapped_inputs[identifier]
                continue
            variable = self.variables.get(identifier)
            initial = variable.attrib.get("initialValue") if variable is not None else None
            values = _numbers(initial) if initial is not None else []
            if len(values) > 1 and identifier not in self.functions:
                result[output] = tuple(values)
                continue
            raise ValueError(f"DAVE-ML vector output {identifier!r} has no supported vector source")
        return result
        ####
    ####


def load_daveml_graph(payload: bytes, *, document_id: str = "daveml-document") -> DAVEMLGraph:
    """Parse one DAVE-ML document into a reusable typed graph view."""

    root = ET.fromstring(payload)
    variables = {
        str(element.attrib.get("varID", "")): element
        for element in _elements(root, "variableDef")
        if element.attrib.get("varID")
    }
    variable_names: dict[str, str] = {}
    for element in variables.values():
        name = str(element.attrib.get("name", ""))
        identifier = str(element.attrib.get("varID", ""))
        if name and identifier and (name not in variable_names or _first_child(element, "isInput") is not None):
            variable_names[name] = identifier
    variable_units = {
        identifier: str(element.attrib["units"])
        for identifier, element in variables.items()
        if element.attrib.get("units")
    }
    return DAVEMLGraph(document_id, _function_index(root), variables, variable_names, variable_units)
    ####


def evaluate_daveml_checkdata(
    payload: bytes,
    *,
    absolute_tolerance: float = 1.0e-5,
    relative_tolerance: float = 1.0e-5,
) -> tuple[DAVEMLCheckResult, ...]:
    """Evaluate supported static shots from one DAVE-ML document.

    Supported functions are direct point-pair functions and regular gridded
    tables. Other graph forms are returned as quarantined check results.
    """

    if absolute_tolerance < 0.0 or relative_tolerance < 0.0:
        raise ValueError("numeric tolerances must be nonnegative")
    root = ET.fromstring(payload)
    graph = load_daveml_graph(payload)
    variable_names = graph.variable_names
    results: list[DAVEMLCheckResult] = []
    for shot in _children_by_local(root, "checkData", recursive=True):
        for case in _children_by_local(shot, "staticShot"):
            case_id = case.attrib.get("name", "staticShot")
            raw_inputs = _signals(case, "checkInputs")
            inputs = _map_signal_ids(raw_inputs, variable_names)
            outputs = _output_signals(case)
            for output_id, (expected, declared_tolerance) in outputs.items():
                resolved_output_id = variable_names.get(output_id, output_id)
                try:
                    actual = graph.evaluate(inputs, (resolved_output_id,))[resolved_output_id]
                except ValueError as error:
                    results.append(_unsupported(case_id, output_id, expected, str(error), declared_tolerance))
                    continue
                absolute_error = abs(actual - expected)
                relative_error = absolute_error / max(abs(expected), 1.0e-30)
                effective_tolerance = absolute_tolerance if declared_tolerance is None else declared_tolerance
                status = "passed" if absolute_error <= effective_tolerance + relative_tolerance * max(abs(expected), abs(actual)) else "failed"
                results.append(
                    DAVEMLCheckResult(
                        case_id,
                        output_id,
                        expected,
                        actual,
                        absolute_error,
                        relative_error,
                        status,
                        absolute_tolerance=effective_tolerance,
                    )
                )
    return tuple(results)
    ####


def _function_index(root: ET.Element) -> dict[str, dict[str, object]]:
    """Index supported function definitions by dependent variable ID."""

    breakpoints = {
        str(element.attrib.get("bpID", "")).strip(): _numbers(_text_of_first(element, "bpVals"))
        for element in _elements(root, "breakpointDef") + _elements(root, "breakpoint")
    }
    tables = {str(element.attrib.get("gtID", "")).strip(): element for element in _elements(root, "griddedTableDef")}
    tables.update({str(element.attrib.get("gtID", "")).strip(): element for element in _elements(root, "griddedTable") if element.attrib.get("gtID")})
    tables.update({str(element.attrib.get("utID", "")).strip(): element for element in _elements(root, "ungriddedTableDef")})
    functions: dict[str, dict[str, object]] = {}
    for function in _elements(root, "function"):
        independent = [str(item.attrib.get("varID", "")) for item in _children_by_local(function, "independentVarRef")]
        dependent = [str(item.attrib.get("varID", "")) for item in _children_by_local(function, "dependentVarRef")]
        point_y = _first_child(function, "dependentVarPts")
        if not dependent and point_y is not None and point_y.attrib.get("varID"):
            dependent = [str(point_y.attrib["varID"])]
        if len(dependent) != 1:
            continue
        point_x = _first_child(function, "independentVarPts")
        if point_x is not None and point_y is not None:
            functions[dependent[0]] = {"kind": "points", "independent": independent or [point_x.attrib.get("varID", "")], "x": _numbers(point_x.text), "y": _numbers(point_y.text)}
            continue
        definition = _first_child(function, "functionDefn")
        if definition is None:
            continue
        table = next(
            (
                child
                for child in list(definition)
                if _local(child.tag) in {"griddedTable", "griddedTableDef", "ungriddedTable", "ungriddedTableDef"}
            ),
            None,
        )
        if table is None:
            reference = next(
                (
                    child
                    for child in list(definition)
                    if _local(child.tag) in {"griddedTableRef", "ungriddedTableRef"}
                ),
                None,
            )
            if reference is not None:
                table_id = str(reference.attrib.get("gtID", reference.attrib.get("utID", ""))).strip()
                table = tables.get(table_id)
        if table is None:
            continue
        functions[dependent[0]] = {
            "kind": "ungridded" if _local(table.tag) == "ungriddedTableDef" else "grid",
            "independent": independent,
            "table": table,
            "axes": _table_axes(table, breakpoints),
        }
    return functions
    ####


def _evaluate_variable(
    variable_id: str,
    inputs: Mapping[str, float],
    functions: Mapping[str, dict[str, object]],
    variables: Mapping[str, ET.Element],
    variable_names: Mapping[str, str],
) -> float:
    """Resolve one variable through inputs, calculations, or functions."""

    visiting: set[str] = set()

    @lru_cache(maxsize=None)
    def resolve(identifier: str) -> float:
        canonical_id = variable_names.get(identifier, identifier)
        if canonical_id in inputs:
            return inputs[canonical_id]
        if canonical_id in visiting:
            raise ValueError(f"cyclic variable dependency at {canonical_id!r}")
        visiting.add(canonical_id)
        try:
            variable = variables.get(canonical_id)
            function = functions.get(canonical_id)
            if function is not None:
                return _evaluate_function(function, resolve)
            calculation = _first_child(variable, "calculation") if variable is not None else None
            if calculation is not None:
                math_node = _first_child(calculation, "math")
                expression = next(iter(list(math_node)), None) if math_node is not None else None
                if expression is not None:
                    return _evaluate_math(expression, resolve)
            if variable is not None and variable.attrib.get("initialValue") is not None:
                values = _numbers(variable.attrib.get("initialValue"))
                if len(values) == 1:
                    return values[0]
            raise ValueError(f"variable {canonical_id!r} has no supported source")
        finally:
            visiting.remove(canonical_id)

    return resolve(variable_id)
    ####


def _evaluate_function(function: dict[str, object], resolve: Callable[[str], float]) -> float:
    """Evaluate one indexed function."""

    independent_values = function.get("independent", [])
    if not isinstance(independent_values, list):
        raise ValueError("function independent variables are unavailable")
    independent = [str(item) for item in independent_values]
    try:
        query = [float(resolve(name)) for name in independent]
    except (KeyError, ValueError) as error:
        raise ValueError("check shot omits a required independent variable") from error
    if function.get("kind") == "points":
        x_values = function.get("x")
        y_values = function.get("y")
        if not isinstance(x_values, list) or not isinstance(y_values, list):
            raise ValueError("point function coordinates are unavailable")
        return _linear(query[0], [float(item) for item in x_values], [float(item) for item in y_values])
    table = function.get("table")
    if not isinstance(table, ET.Element):
        raise ValueError("table definition is unavailable")
    if function.get("kind") == "ungridded":
        return _ungridded(query, table)
    axes = function.get("axes", [])
    if not isinstance(axes, list):
        raise ValueError("table breakpoints are unavailable")
    values = _numbers(_text_of_first(table, "dataTable"))
    if len(axes) != len(query):
        raise ValueError("gridded table dimension does not match check shot")
    expected_size = math.prod(len(axis) for axis in axes)
    if len(values) != expected_size:
        raise ValueError("gridded table data size does not match breakpoints")
    return _multilinear(query, axes, values)
    ####


def _evaluate_math(element: ET.Element, resolve: Callable[[str], float]) -> float:
    """Evaluate the bounded MathML subset used by the reference corpus."""

    tag = _local(element.tag)
    if tag == "ci":
        identifier = " ".join(element.itertext()).strip()
        if not identifier:
            raise ValueError("MathML ci has no variable identifier")
        return resolve(identifier)
    if tag == "cn":
        values = _numbers(" ".join(element.itertext()))
        if len(values) != 1:
            raise ValueError("MathML cn is not a scalar")
        return values[0]
    if tag == "apply":
        children = list(element)
        if not children:
            raise ValueError("MathML apply has no operator")
        operator = _local(children[0].tag)
        operands = children[1:]
        if operator == "piecewise":
            return _evaluate_math(children[0], resolve)
        values = [_evaluate_math(child, resolve) for child in operands]
        if operator == "plus":
            return sum(values)
        if operator == "times":
            return math.prod(values)
        if operator == "minus":
            if len(values) == 1:
                return -values[0]
            if len(values) == 2:
                return values[0] - values[1]
        if operator == "divide" and len(values) == 2:
            return values[0] / values[1]
        if operator == "power" and len(values) == 2:
            return values[0] ** values[1]
        if operator == "abs" and len(values) == 1:
            return abs(values[0])
        if operator == "min" and values:
            return min(values)
        if operator == "max" and values:
            return max(values)
        raise ValueError(f"unsupported MathML operator {operator!r}")
    if tag == "piecewise":
        for piece in _children_by_local(element, "piece"):
            children = list(piece)
            if len(children) == 2 and _evaluate_condition(children[1], resolve):
                return _evaluate_math(children[0], resolve)
        otherwise = _first_child(element, "otherwise")
        if otherwise is not None:
            expression = next(iter(list(otherwise)), None)
            if expression is not None:
                return _evaluate_math(expression, resolve)
        raise ValueError("MathML piecewise has no true branch")
    raise ValueError(f"unsupported MathML element {tag!r}")
    ####


def _evaluate_condition(element: ET.Element, resolve: Callable[[str], float]) -> bool:
    """Evaluate a comparison or Boolean MathML condition."""

    tag = _local(element.tag)
    children = list(element)
    if tag == "apply" and children:
        operator = _local(children[0].tag)
        operands = children[1:]
    else:
        operator = tag
        operands = children
    if operator in {"lt", "leq", "gt", "geq", "eq"} and len(operands) == 2:
        left, right = (_evaluate_math(child, resolve) for child in operands)
        return {
            "lt": left < right,
            "leq": left <= right,
            "gt": left > right,
            "geq": left >= right,
            "eq": left == right,
        }[operator]
    if operator == "not" and len(operands) == 1:
        return not _evaluate_condition(operands[0], resolve)
    if operator == "and" and operands:
        return all(_evaluate_condition(child, resolve) for child in operands)
    if operator == "or" and operands:
        return any(_evaluate_condition(child, resolve) for child in operands)
    raise ValueError(f"unsupported MathML condition {operator!r}")
    ####


def _table_axes(table: ET.Element, breakpoints: dict[str, list[float]]) -> list[list[float]]:
    """Resolve breakpoint vectors for an inline regular table."""

    axes: list[list[float]] = []
    refs = _first_child(table, "breakpointRefs")
    if refs is None:
        refs = table
    for reference in _children_by_local(refs, "bpRef"):
        bp_id = str(reference.attrib.get("bpID", ""))
        axis = breakpoints.get(bp_id.strip())
        if not axis:
            raise ValueError(f"breakpoint reference {bp_id!r} is unresolved")
        axes.append(axis)
    if axes:
        return axes
    inline = _children_by_local(table, "bpVals")
    return [_numbers(element.text) for element in inline]
    ####


def _ungridded(query: list[float], table: ET.Element) -> float:
    """Evaluate an ungridded table using slice interpolation when available."""

    points = [_numbers(element.text) for element in _children_by_local(table, "dataPoint")]
    points = [point for point in points if len(point) == len(query) + 1]
    if not points:
        raise ValueError("ungridded table has no compatible data points")
    if len(query) == 2:
        rows: dict[float, list[tuple[float, float]]] = {}
        for point in points:
            rows.setdefault(point[0], []).append((point[1], point[2]))
        if len(rows) >= 2 and all(len(row) >= 2 for row in rows.values()):
            row_values = [(coordinate, _linear(query[1], [item[0] for item in sorted(row)], [item[1] for item in sorted(row)])) for coordinate, row in sorted(rows.items())]
            return _linear(query[0], [item[0] for item in row_values], [item[1] for item in row_values])
    scales = [max(point[index] for point in points) - min(point[index] for point in points) for index in range(len(query))]
    distances: list[tuple[float, float]] = []
    for point in points:
        distance = math.sqrt(sum(((query[index] - point[index]) / (scales[index] or 1.0)) ** 2 for index in range(len(query))))
        if distance == 0.0:
            return point[-1]
        distances.append((distance, point[-1]))
    weights = [1.0 / distance for distance, _ in distances]
    return sum(weight * value for weight, (_, value) in zip(weights, distances, strict=True)) / sum(weights)
    ####


def _multilinear(query: list[float], axes: list[list[float]], values: list[float]) -> float:
    """Evaluate a row-major regular grid using DAVEtools endpoint clamping."""

    strides: list[int] = []
    stride = 1
    for axis in reversed(axes[1:]):
        stride *= len(axis)
        strides.insert(0, stride)
    strides.append(1)
    lower: list[int] = []
    fractions: list[float] = []
    for axis, coordinate in zip(axes, query, strict=True):
        if not axis:
            raise ValueError("gridded table has an empty breakpoint axis")
        if len(axis) == 1:
            lower.append(0)
            fractions.append(0.0)
            continue
        if coordinate <= axis[0]:
            index = 0
        elif coordinate >= axis[-1]:
            index = len(axis) - 2
        else:
            index = next(i for i in range(len(axis) - 1) if coordinate <= axis[i + 1])
        lower.append(index)
        bounded_coordinate = min(axis[-1], max(axis[0], coordinate))
        fractions.append((bounded_coordinate - axis[index]) / (axis[index + 1] - axis[index]) if axis[index + 1] != axis[index] else 0.0)
    result = 0.0
    for corner in range(1 << len(axes)):
        weight = 1.0
        flat = 0
        for dimension, index in enumerate(lower):
            high = (corner >> dimension) & 1
            weight *= fractions[dimension] if high else 1.0 - fractions[dimension]
            flat += (index + high) * strides[dimension]
        result += weight * values[flat]
    return result
    ####


def _linear(value: float, x: list[float], y: list[float]) -> float:
    """Evaluate a bounded piecewise-linear point function."""

    if len(x) != len(y) or len(x) < 2:
        raise ValueError("point function requires paired vectors")
    if value < x[0] or value > x[-1]:
        raise ValueError("query is outside point-function bounds")
    index = min(len(x) - 2, max(0, next((i for i in range(len(x) - 1) if value <= x[i + 1]), len(x) - 2)))
    fraction = (value - x[index]) / (x[index + 1] - x[index]) if x[index + 1] != x[index] else 0.0
    return y[index] + fraction * (y[index + 1] - y[index])
    ####


def _signals(case: ET.Element, group: str) -> dict[str, float]:
    """Read static-shot signal IDs and values."""

    result: dict[str, float] = {}
    parent = _first_child(case, group)
    if parent is None:
        return result
    for signal in _children_by_local(parent, "signal"):
        identifier = _signal_identifier(signal)
        values = _numbers(_text_of_first(signal, "signalValue"))
        if identifier and len(values) == 1:
            result[identifier] = values[0]
    return result
    ####


def _output_signals(case: ET.Element) -> dict[str, tuple[float, float | None]]:
    """Read expected output values and optional source tolerances."""

    result: dict[str, tuple[float, float | None]] = {}
    parent = _first_child(case, "checkOutputs")
    if parent is None:
        return result
    for signal in _children_by_local(parent, "signal"):
        identifier = _signal_identifier(signal)
        values = _numbers(_text_of_first(signal, "signalValue"))
        tolerance_values = _numbers(_text_of_first(signal, "tol"))
        if identifier and len(values) == 1:
            result[identifier] = (values[0], tolerance_values[0] if len(tolerance_values) == 1 else None)
    return result
    ####


def _signal_identifier(signal: ET.Element) -> str:
    """Return either the modern signal ID or the legacy signal name."""

    return (_text_of_first(signal, "signalID") or _text_of_first(signal, "signalName")).strip()
    ####


def _map_signal_ids(signals: Mapping[str, float], variable_names: Mapping[str, str]) -> dict[str, float]:
    """Add variable-ID aliases for legacy checkData signal names."""

    result = dict(signals)
    for identifier, value in signals.items():
        canonical_id = variable_names.get(identifier)
        if canonical_id is not None:
            result[canonical_id] = value
    return result
    ####


def _resolve_calculated_inputs(root: ET.Element, inputs: dict[str, float]) -> dict[str, float]:
    """Resolve initial values and simple additive variable calculations."""

    resolved = dict(inputs)
    for variable in _elements(root, "variableDef"):
        identifier = str(variable.attrib.get("varID", ""))
        if identifier and identifier not in resolved and variable.attrib.get("initialValue") is not None:
            values = _numbers(variable.attrib.get("initialValue"))
            if len(values) == 1:
                resolved[identifier] = values[0]
    for variable in _elements(root, "variableDef"):
        identifier = str(variable.attrib.get("varID", ""))
        calculation = _first_child(variable, "calculation")
        if not identifier or calculation is None:
            continue
        math_node = _first_child(calculation, "math")
        apply_node = _first_child(math_node, "apply") if math_node is not None else None
        first_child = next(iter(apply_node), None) if apply_node is not None else None
        if apply_node is None or first_child is None or _local(first_child.tag) != "plus":
            continue
        names = [" ".join(item.itertext()).strip() for item in _children_by_local(apply_node, "ci")]
        if names and all(name in resolved for name in names):
            resolved[identifier] = sum(resolved[name] for name in names)
    return resolved
    ####


def _unsupported(
    case_id: str,
    output_id: str,
    expected: float,
    reason: str,
    absolute_tolerance: float | None = None,
) -> DAVEMLCheckResult:
    """Create an explicit quarantined check result."""

    return DAVEMLCheckResult(case_id, output_id, expected, None, None, None, "unsupported", reason, absolute_tolerance)
    ####


def _elements(root: ET.Element, name: str) -> list[ET.Element]:
    return [element for element in root.iter() if _local(element.tag) == name]


def _children_by_local(root: ET.Element, name: str, *, recursive: bool = False) -> list[ET.Element]:
    source = root.iter() if recursive else list(root)
    return [element for element in source if element is not root and _local(element.tag) == name]


def _first_child(root: ET.Element, name: str) -> ET.Element | None:
    return next(iter(_children_by_local(root, name)), None)


def _text_of_first(root: ET.Element, name: str) -> str:
    element = _first_child(root, name)
    return "" if element is None else " ".join(element.itertext())


def _numbers(value: str | None) -> list[float]:
    if not value:
        return []
    return [float(token) for token in re.findall(r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?", value)]


def _local(tag: object) -> str:
    return str(tag).rsplit("}", 1)[-1]


__all__ = ["DAVEMLCheckResult", "evaluate_daveml_checkdata"]
