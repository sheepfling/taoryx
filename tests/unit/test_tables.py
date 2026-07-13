from __future__ import annotations

import pytest

from taoryx.language.models import SourceLocation, TableOperation
from taoryx.tables import (
    ExtrapolationMode,
    SkewedTableSlice,
    TableEvaluationContext,
    accumulate_table_values,
    evaluate_full_table,
    interpolate_nd,
    interpolate_skewed,
    prepare_skewed_table,
    prepare_table,
)


def test_interpolate_nd_reproduces_affine_two_dimensional_table() -> None:
    table = prepare_table(
        ((0.0, 1.0), (10.0, 20.0)),
        (10.0, 20.0, 11.0, 21.0),
    )

    assert interpolate_nd(table, (0.25, 15.0)) == pytest.approx(15.25)
    assert interpolate_nd(table, (1.0, 20.0)) == pytest.approx(21.0)
####


def test_interpolate_nd_supports_descending_axes_and_clamped_queries() -> None:
    table = prepare_table(
        ((2.0, 1.0, 0.0),),
        (4.0, 2.0, 0.0),
        extrapolation=ExtrapolationMode.CLAMP,
    )

    assert interpolate_nd(table, (1.5,)) == pytest.approx(3.0)
    assert interpolate_nd(table, (4.0,)) == pytest.approx(4.0)
    assert interpolate_nd(table, (-1.0,)) == pytest.approx(0.0)
####


def test_prepare_table_rejects_duplicate_axes_and_wrong_value_count() -> None:
    with pytest.raises(ValueError, match="monotonic"):
        prepare_table(((0.0, 1.0, 1.0),), (0.0, 1.0, 1.0))
    with pytest.raises(ValueError, match="requires 4"):
        prepare_table(((0.0, 1.0), (0.0, 1.0)), (0.0, 1.0, 2.0))
####


def _operation(operator: str, operand: str | float | None = None, *, label: str | None = None) -> TableOperation:
    return TableOperation(
        operator=operator,
        operand=operand,
        label=label,
        location=SourceLocation(path="<test>", line=1),
    )


def test_full_table_accumulator_dispatch_and_storage() -> None:
    context = TableEvaluationContext.from_values({"alpha": 30.0})
    result = evaluate_full_table(
        (
            _operation("add", "alpha"),
            _operation("sin"),
            _operation("sqr"),
            _operation("csto", "saved"),
            _operation("add", "saved"),
            _operation("end"),
        ),
        context,
    )

    assert result.value == pytest.approx(0.25)
    assert result.storage[0][0] == "saved"
    assert result.storage[0][1] == pytest.approx(0.25)


def test_full_table_goto_and_multi_table_accumulation() -> None:
    context = TableEvaluationContext.from_values({})
    result = evaluate_full_table(
        (
            _operation("set", 2.0, label="start"),
            _operation("goto", "finish"),
            _operation("set", 99.0),
            _operation("end", label="finish"),
        ),
        context,
    )

    assert result.value == pytest.approx(2.0)
    assert accumulate_table_values((1.0, 2.5, -0.5)) == pytest.approx(3.0)


def test_skewed_interpolation_evaluates_each_inner_slice_before_outer_mix() -> None:
    table = prepare_skewed_table(
        (
            SkewedTableSlice((0.0,), (0.0, 1.0), (0.0, 1.0)),
            SkewedTableSlice((1.0,), (0.0, 2.0), (10.0, 12.0)),
        )
    )

    assert interpolate_skewed(table, (0.5, 0.5)) == pytest.approx(5.5)
