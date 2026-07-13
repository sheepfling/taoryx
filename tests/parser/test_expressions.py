from taoryx.language.expressions import (
    BinaryExpression,
    IndexedExpression,
    NumberExpression,
    ParameterExpression,
    TableReferenceExpression,
    parse_expression,
    tokenize_expression,
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


def test_fortran_d_exponent_is_a_number() -> None:
    value = parse_expression("3.0D+02 + 1")

    assert isinstance(value, BinaryExpression)
    assert isinstance(value.left, NumberExpression)
    assert value.left.value == 300.0
####


def test_compound_boolean_expression_is_supported() -> None:
    value = parse_expression("alt > 100 && vel < 2")

    assert isinstance(value, BinaryExpression)
    assert value.operator == "&&"


def test_compound_assignment_operator_is_tokenized() -> None:
    tokens = tokenize_expression("x += 1")

    assert [token.value for token in tokens] == ["x", "+=", "1"]
