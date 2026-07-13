from taoryx.language.expressions import (
    BinaryExpression,
    IndexedExpression,
    ParameterExpression,
    TableReferenceExpression,
    parse_expression,
)


def test_operator_precedence() -> None:
    expression = parse_expression("alt/3280.84+2")
    assert isinstance(expression, BinaryExpression)
    assert expression.operator == "+"
    assert isinstance(expression.left, BinaryExpression)
    assert expression.left.operator == "/"
####


def test_indexed_and_parameter_expressions() -> None:
    indexed = parse_expression("relvel[2]")
    parameter = parse_expression("opta-13")
    assert isinstance(indexed, IndexedExpression)
    assert indexed.index == 2
    assert isinstance(parameter, ParameterExpression)
    assert parameter.loop == "a"
    assert parameter.index == 13
####


def test_table_reference_is_supported_by_problem_parser_value_layer() -> None:
    value = TableReferenceExpression(name="thrust1")
    assert value.name == "thrust1"
####
