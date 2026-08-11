from __future__ import annotations

import pytest
from taoryx.families.cadac.deck import CadacDeckParseError, parse_cadac_deck

DECK = """    TITLE Synthetic prototype deck

1DIM thrust_vs_time
NX1 3 // thrust - N
0 100
1 80
2 0

2DIM coefficient_vs_alpha_mach
NX1 2 NX2 3 // synthetic coefficient
0 0.5 0 1 2
10 1.0 10 11 12
   2.0
"""


def test_parse_one_and_two_dimensional_tables() -> None:
    deck = parse_cadac_deck(DECK, source_name="fixture/deck.asc")

    assert deck.title == "Synthetic prototype deck"
    assert [table.name for table in deck.tables] == ["thrust_vs_time", "coefficient_vs_alpha_mach"]
    assert deck.table("thrust_vs_time").axes == ((0.0, 1.0, 2.0),)
    assert deck.table("coefficient_vs_alpha_mach").axis_sizes == (2, 3)
    assert deck.table("coefficient_vs_alpha_mach").axes == ((0.0, 10.0), (0.5, 1.0, 2.0))
    assert deck.table("coefficient_vs_alpha_mach").values == (0.0, 1.0, 2.0, 10.0, 11.0, 12.0)


####


def test_cadac_boundaries_are_lower_linear_and_upper_clamped() -> None:
    table = parse_cadac_deck(DECK).table("thrust_vs_time")

    assert table.interpolate((0.5,)) == pytest.approx(90.0)
    assert table.interpolate((-1.0,)) == pytest.approx(120.0)
    assert table.interpolate((3.0,)) == pytest.approx(0.0)


####


def test_bilinear_interpolation_uses_row_major_values() -> None:
    table = parse_cadac_deck(DECK).table("coefficient_vs_alpha_mach")

    assert table.interpolate((5.0, 1.5)) == pytest.approx(6.5)
    assert table.interpolate((-5.0, 1.0)) == pytest.approx(-4.0)
    assert table.interpolate((15.0, 3.0)) == pytest.approx(12.0)


####


def test_rejects_uncharacterized_higher_dimensional_layout() -> None:
    with pytest.raises(CadacDeckParseError, match="4DIM tables are not supported"):
        parse_cadac_deck("TITLE x\n4DIM table\nNX1 1 NX2 1 NX3 1 NX4 1\n0 0 0 0 1")
    ####


####


def test_boundary_policy_uses_numeric_domain_for_descending_axes() -> None:
    deck = parse_cadac_deck("""        TITLE descending
    1DIM value_vs_axis
    NX1 2
    10 100
    0 0
    """)
    table = deck.table("value_vs_axis")

    assert table.interpolate((-5.0,)) == pytest.approx(-50.0)
    assert table.interpolate((15.0,)) == pytest.approx(100.0)


####


def test_rejects_malformed_table_declaration() -> None:
    with pytest.raises(CadacDeckParseError, match="malformed table declaration"):
        parse_cadac_deck("TITLE bad\n2DIM\nNX1 1 NX2 1\n0 0 1")
    ####


####


THREE_DIM = """\
TITLE synthetic 3D deck
3DIM ff
NX1 2 NX2 2 NX3 2 // X1 rows, X2 blocks, X3 columns
0 0 10   0 1 2 3
1 5 20   4 5 6 7
"""


def test_parse_3dim_source_layout_and_trilinear_interpolation() -> None:
    deck = parse_cadac_deck(THREE_DIM, source_name="ff.asc")
    table = deck.table("ff")

    assert table.dimension == 3
    assert table.axis_sizes == (2, 2, 2)
    assert table.axes == ((0.0, 1.0), (0.0, 5.0), (10.0, 20.0))
    assert table.values == (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
    assert table.interpolate((0.0, 0.0, 10.0)) == pytest.approx(0.0)
    assert table.interpolate((1.0, 5.0, 20.0)) == pytest.approx(7.0)
    assert table.interpolate((0.5, 2.5, 15.0)) == pytest.approx(3.5)


####


def test_3dim_keeps_linear_lower_and_clamped_upper_boundary_policy() -> None:
    table = parse_cadac_deck(THREE_DIM).table("ff")

    assert table.interpolate((-1.0, 0.0, 10.0)) == pytest.approx(-4.0)
    assert table.interpolate((2.0, 5.0, 20.0)) == pytest.approx(7.0)


####


def test_duplicate_identical_1d_axis_rows_are_collapsed_for_source_weather_decks() -> None:
    deck = parse_cadac_deck(
        """TITLE duplicate weather row
1DIM temperature
NX1 4
0 18
30000 -46.5
30000 -46.5
40000 -25
"""
    )
    table = deck.table("temperature")
    assert table.axis_sizes == (3,)
    assert table.axes == ((0.0, 30000.0, 40000.0),)
    assert table.values == (18.0, -46.5, -25.0)


####


def test_duplicate_conflicting_1d_axis_rows_fail_closed() -> None:
    with pytest.raises(CadacDeckParseError, match="duplicate 1DIM axis value"):
        parse_cadac_deck(
            """TITLE conflicting duplicate
1DIM temperature
NX1 3
0 18
30000 -46.5
30000 -45
"""
        )
    ####


####


def test_legacy_adjacent_signed_table_values_are_tokenized_without_rewriting_source() -> None:
    deck = parse_cadac_deck(
        """TITLE compact signed values
2DIM correction
NX1 1 NX2 2
0 0 1.25-2.5
1
"""
    )

    table = deck.table("correction")
    assert table.axes == ((0.0,), (0.0, 1.0))
    assert table.values == (1.25, -2.5)
    ####
