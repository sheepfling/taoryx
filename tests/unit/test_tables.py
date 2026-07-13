from __future__ import annotations

import pytest

from taoryx.tables import ExtrapolationMode, interpolate_nd, prepare_table


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
