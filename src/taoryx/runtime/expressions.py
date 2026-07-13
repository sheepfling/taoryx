"""Evaluation of the typed expression AST used by define blocks."""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping, Sequence

from taoryx.language.expressions import (
    BinaryExpression,
    CallExpression,
    ExpressionType,
    IndexedExpression,
    NameExpression,
    NumberExpression,
    ParameterExpression,
    TableReferenceExpression,
    UnaryExpression,
)


def evaluate_definition_program(
    expressions: Mapping[str, ExpressionType],
    values: Mapping[str, float] | None = None,
    *,
    parameters: Mapping[str, float] | None = None,
    tables: Mapping[str, float] | None = None,
    table_evaluators: Mapping[str, Callable[[Mapping[str, float]], float]] | None = None,
) -> dict[str, float]:
    """Resolve a define program with memoized, cycle-checked dependencies.

    This implements ``TAOS-ALG-PRB-001``. The supplied AST is deliberately
    independent of parser block models so callers can use it for trajectory or
    problem scope.
    """

    result = dict(values or {})
    context = dict(parameters or {})
    context.update(tables or {})
    resolving: set[str] = set()

    def resolve(name: str) -> float:
        if name in result:
            return result[name]
        if name in resolving:
            raise ValueError(f"circular definition: {name}")
        if name not in expressions:
            raise KeyError(f"undefined variable: {name}")
        resolving.add(name)
        result[name] = evaluate_expression(expressions[name], result, context, resolve, table_evaluators)
        resolving.remove(name)
        return result[name]

    for name in expressions:
        resolve(name)
    return result
####


def evaluate_expression(
    expression: ExpressionType,
    values: Mapping[str, float],
    parameters: Mapping[str, float] = {},
    resolver: Callable[[str], float] | None = None,
    tables: Mapping[str, Callable[[Mapping[str, float]], float]] | None = None,
) -> float:
    """Evaluate one parser expression with TAOS scalar operators."""

    if isinstance(expression, NumberExpression):
        return expression.value
    if isinstance(expression, NameExpression):
        if expression.name in values:
            return float(values[expression.name])
        if resolver is not None:
            return float(resolver(expression.name))
        raise KeyError(f"undefined variable: {expression.name}")
    if isinstance(expression, IndexedExpression):
        return float(values[f"{expression.name}[{expression.index}]"])
    if isinstance(expression, ParameterExpression):
        key = f"{expression.family}-{expression.index}"
        return float(parameters[key])
    if isinstance(expression, TableReferenceExpression):
        if tables is not None and expression.name.casefold() in tables:
            return float(tables[expression.name.casefold()](values))
        return float(parameters[expression.name])
    if isinstance(expression, UnaryExpression):
        value = evaluate_expression(expression.operand, values, parameters, resolver, tables)
        return -value if expression.operator == "-" else (not value if expression.operator == "!" else value)
    if isinstance(expression, BinaryExpression):
        left = evaluate_expression(expression.left, values, parameters, resolver, tables)
        right = evaluate_expression(expression.right, values, parameters, resolver, tables)
        return _binary(expression.operator, left, right)
    if isinstance(expression, CallExpression):
        if expression.function.casefold() == "table":
            if tables is None or len(expression.arguments) != 1 or not isinstance(expression.arguments[0], NameExpression):
                raise ValueError("table() requires one configured table name")
            table_name = expression.arguments[0].name.casefold()
            if table_name not in tables:
                raise KeyError(f"undefined table: {table_name}")
            return float(tables[table_name](values))
        arguments = [evaluate_expression(arg, values, parameters, resolver, tables) for arg in expression.arguments]
        return _call(expression.function, arguments)
    raise TypeError(f"unsupported expression: {type(expression).__name__}")
####


def integral_variable_derivatives(
    definitions: Mapping[str, ExpressionType],
    values: Mapping[str, float],
    *,
    parameters: Mapping[str, float] | None = None,
) -> dict[str, float]:
    """Evaluate derivative definitions for integral variables."""

    resolved = evaluate_definition_program(definitions, values, parameters=parameters)
    return {name: resolved[name] for name in definitions}
####


def _binary(operator: str, left: float, right: float) -> float:
    if operator in {"=", "=="}:
        return float(left == right)
    if operator == "!=":
        return float(left != right)
    if operator == "<":
        return float(left < right)
    if operator == ">":
        return float(left > right)
    if operator == "<=":
        return float(left <= right)
    if operator == ">=":
        return float(left >= right)
    if operator == "+":
        return left + right
    if operator == "-":
        return left - right
    if operator == "*":
        return left * right
    if operator == "/":
        if right == 0.0:
            raise ZeroDivisionError("definition division by zero")
        return left / right
    if operator == "^":
        return left**right
    if operator in {"&&", "and"}:
        return float(bool(left) and bool(right))
    if operator in {"||", "or"}:
        return float(bool(left) or bool(right))
    raise ValueError(f"unsupported operator: {operator}")
####


def _call(function: str, arguments: Sequence[float]) -> float:
    name = function.casefold()
    if len(arguments) != 1:
        raise ValueError(f"function {function} requires one argument")
    value = arguments[0]
    functions: dict[str, Callable[[float], float]] = {
        "abs": float.__abs__, "sqrt": math.sqrt, "ln": math.log, "log": math.log10,
        "sin": math.sin, "cos": math.cos, "tan": math.tan, "asin": math.asin,
        "acos": math.acos, "atan": math.atan, "exp": math.exp,
    }
    try:
        return float(functions[name](value))
    except KeyError as error:
        raise ValueError(f"unsupported function: {function}") from error
    ####
####
