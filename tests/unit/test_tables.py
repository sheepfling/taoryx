from __future__ import annotations

import pytest

from taoryx.language.models import SourceLocation, TableCall, TableOperation
from taoryx.tables import (
    ExtrapolationMode,
    SkewedTableSlice,
    TableEvaluationContext,
    accumulate_table_values,
    apply_table_operation,
    clear_and_store,
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


def test_clear_and_store_has_explicit_runtime_contract() -> None:
    storage: dict[str, float] = {}

    assert clear_and_store(storage, "Saved", 3.5) == 0.0
    assert storage == {"saved": 3.5}
    with pytest.raises(ValueError, match="duplicate storage"):
        clear_and_store(storage, "SAVED", 4.0)
####


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


def test_full_table_can_resolve_a_nested_runtime_table_evaluator() -> None:
    context = TableEvaluationContext.from_values({}, evaluators={"inner": lambda _: 4.0})
    result = evaluate_full_table(
        (
            _operation("add", TableCall(name="inner")),
            _operation("mult", 3.0),
            _operation("end"),
        ),
        context,
    )

    assert result.value == pytest.approx(12.0)
    ####


def test_full_table_min_and_max_select_the_named_extremum() -> None:
    assert apply_table_operation(3.0, "max", 5.0) == pytest.approx(5.0)
    assert apply_table_operation(3.0, "max", 2.0) == pytest.approx(3.0)
    assert apply_table_operation(3.0, "min", 5.0) == pytest.approx(3.0)
    assert apply_table_operation(3.0, "min", 2.0) == pytest.approx(2.0)
    ####


def test_skewed_interpolation_evaluates_each_inner_slice_before_outer_mix() -> None:
    table = prepare_skewed_table(
        (
            SkewedTableSlice((0.0,), (0.0, 1.0), (0.0, 1.0)),
            SkewedTableSlice((1.0,), (0.0, 2.0), (10.0, 12.0)),
        )
    )

    assert interpolate_skewed(table, (0.5, 0.5)) == pytest.approx(5.5)


def test_skewed_interpolation_supports_sparse_nested_outer_groups() -> None:
    table = prepare_skewed_table(
        (
            SkewedTableSlice((0.0, 0.0), (0.0, 5.0), (0.10, 0.20)),
            SkewedTableSlice((0.0, 5.0), (0.0, 5.0), (0.20, 0.30)),
            SkewedTableSlice((50000.0, 2.0), (0.0, 5.0), (0.09, 0.10)),
            SkewedTableSlice((50000.0, 8.0), (0.0, 5.0), (0.06, 0.07)),
        )
    )

    assert interpolate_skewed(table, (50000.0, 5.0, 5.0)) == pytest.approx(0.085)
