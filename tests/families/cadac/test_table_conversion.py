from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest
from taoryx.families.cadac.__main__ import main
from taoryx.families.cadac.deck import parse_cadac_deck
from taoryx.families.cadac.table_conversion import (
    convert_cadac_deck,
    convert_cadac_deck_file_to_table_bundle,
    convert_cadac_source_bundle_to_table_bundle,
    convert_cadac_tree_to_table_bundle,
    load_converted_cadac_deck,
)
from taoryx.families.cadac.table_resources import TableAxisDirection, TableUnitEvidence, read_table_bundle

SYNTHETIC_DECK = """TITLE Canonical conversion fixture
1DIM thrust_vs_time
NX1 3 // thrust - N
0 100
1 80
2 0

2DIM coefficient_vs_alpha_mach
NX1 2 NX2 3 // coefficient - ND
0 0.5 0 1 2
10 1.0 10 11 12
   2.0

3DIM ff_vs_thrust_alt_mach
NX1 2 NX2 2 NX3 2 // flow
100 0 0.5 0 1 2 3
200 1000 1.0 4 5 6 7
"""

DESCENDING_2D = """TITLE Descending axes
2DIM value_vs_alpha_mach
NX1 2 NX2 2 // synthetic
10 0 100 120
0 20 0 20
"""


def _sha() -> str:
    return "1" * 64


####


def test_conversion_preserves_1d_2d_3d_lookup_equivalence() -> None:
    deck = parse_cadac_deck(SYNTHETIC_DECK, source_name="fixture.asc")
    canonical = convert_cadac_deck(deck, source_path="fixture.asc", source_sha256=_sha())
    queries = {
        "thrust_vs_time": ((0.5,), (-1.0,), (3.0,)),
        "coefficient_vs_alpha_mach": ((5.0, 1.5), (-5.0, 1.0), (15.0, 3.0)),
        "ff_vs_thrust_alt_mach": ((150.0, 500.0, 0.75), (50.0, 0.0, 0.5), (250.0, 2000.0, 2.0)),
    }
    for name, points in queries.items():
        source_table = deck.table(name)
        canonical_table = canonical.table(name)
        for point in points:
            assert canonical_table.lookup(*point) == pytest.approx(source_table.interpolate(point))
        ####
    ####


####


def test_descending_axes_are_normalized_and_tensor_is_reordered() -> None:
    deck = parse_cadac_deck(DESCENDING_2D, source_name="descending.asc")
    source_table = deck.table("value_vs_alpha_mach")
    table = convert_cadac_deck(deck, source_path="descending.asc", source_sha256=_sha()).table("value_vs_alpha_mach")

    assert table.axes[0].values == (0.0, 10.0)
    assert table.axes[0].source_direction is TableAxisDirection.DESCENDING
    assert table.axes[1].values == (0.0, 20.0)
    assert table.values == (0.0, 20.0, 100.0, 120.0)
    assert table.lookup(5.0, 10.0) == pytest.approx(source_table.interpolate((5.0, 10.0)))
    assert table.lookup(-5.0, 10.0) == pytest.approx(source_table.interpolate((-5.0, 10.0)))
    assert table.lookup(15.0, 30.0) == pytest.approx(source_table.interpolate((15.0, 30.0)))


####


def test_axis_and_value_unit_evidence_are_explicit() -> None:
    deck = parse_cadac_deck(SYNTHETIC_DECK)
    canonical = convert_cadac_deck(deck, source_path="fixture.asc", source_sha256=_sha())
    thrust = canonical.table("thrust_vs_time")
    unknown = convert_cadac_deck(
        parse_cadac_deck("TITLE unknown\n1DIM mystery_vs_foo\nNX1 2 // whatever\n0 1\n1 2\n"),
        source_path="unknown.asc",
        source_sha256=_sha(),
    ).table("mystery_vs_foo")

    assert thrust.axes[0].unit.symbol == "s"
    assert thrust.axes[0].unit.evidence is TableUnitEvidence.NAME_INFERRED
    assert thrust.value_unit.symbol == "N"
    assert thrust.value_unit.evidence is TableUnitEvidence.SOURCE_COMMENT
    assert unknown.axes[0].unit.symbol is None
    assert unknown.axes[0].unit.evidence is TableUnitEvidence.UNKNOWN
    assert unknown.value_unit.symbol is None


####


def test_deck_file_conversion_writes_deterministic_canonical_bundle(tmp_path: Path) -> None:
    source = tmp_path / "physical.asc"
    source.write_text(SYNTHETIC_DECK, encoding="utf-8")
    first = tmp_path / "first"
    second = tmp_path / "second"

    first_manifest = convert_cadac_deck_file_to_table_bundle(source, first)
    second_manifest = convert_cadac_deck_file_to_table_bundle(source, second)
    first_files = {path.relative_to(first): path.read_bytes() for path in first.rglob("*") if path.is_file()}
    second_files = {path.relative_to(second): path.read_bytes() for path in second.rglob("*") if path.is_file()}

    assert first_manifest == second_manifest
    assert first_files == second_files
    assert len(first_manifest.tables) == 3
    loaded = read_table_bundle(first)
    expected_sha256 = hashlib.sha256(source.read_bytes()).hexdigest()
    assert all(table.provenance.source_sha256 == expected_sha256 for table in loaded.tables)


####


def test_converted_deck_can_replace_cadac_lookup_surface(tmp_path: Path) -> None:
    source = tmp_path / "physical.asc"
    source.write_text(SYNTHETIC_DECK, encoding="utf-8")
    output = tmp_path / "bundle"
    convert_cadac_deck_file_to_table_bundle(source, output)
    converted = load_converted_cadac_deck(output)

    assert converted.table("thrust_vs_time").lookup(0.5) == pytest.approx(90.0)
    assert converted.table("coefficient_vs_alpha_mach").lookup(5.0, 1.5) == pytest.approx(6.5)


####


def test_source_bundle_conversion_deduplicates_shared_physical_decks(tmp_path: Path) -> None:
    deck = tmp_path / "shared.asc"
    deck.write_text("TITLE shared\n1DIM value_vs_time\nNX1 2 // value - N\n0 1\n1 2\n", encoding="utf-8")
    case = tmp_path / "input.asc"
    case.write_text(
        """TITLE shared source case
OPTIONS n_scrn n_plot
MODULES
    aerodynamics def,exec
END
TIMING
    int_step 0.1
END
VEHICLES 2
    AIM5 Missile_1
        AERO_DECK shared.asc
    END
    AIM5 Missile_2
        SAM_DECK shared.asc
    END
ENDTIME 1
STOP
""",
        encoding="utf-8",
    )
    output = tmp_path / "canonical"

    manifest = convert_cadac_source_bundle_to_table_bundle(case, output)
    loaded = read_table_bundle(output)

    assert len(manifest.tables) == 1
    assert len(loaded.tables) == 1
    bindings = manifest.metadata["deck_bindings"]
    assert isinstance(bindings, list)
    assert len(bindings) == 2


####


def test_source_tree_conversion_discovers_all_table_bearing_asc_files(tmp_path: Path) -> None:
    source_root = tmp_path / "CADAC"
    package_a = source_root / "A"
    package_b = source_root / "B"
    package_a.mkdir(parents=True)
    package_b.mkdir(parents=True)
    (package_a / "aero_deck.asc").write_text(
        "TITLE A\n1DIM value_vs_time\nNX1 2 // value - N\n0 1\n1 2\n",
        encoding="utf-8",
    )
    (package_b / "prop_deck.asc").write_text(
        "TITLE B\n2DIM value_vs_alpha_mach\nNX1 2 NX2 2 // value - ND\n0 0.5 0 1\n10 1.0 10 11\n",
        encoding="utf-8",
    )
    (source_root / "input.asc").write_text("TITLE not a deck\nOPTIONS n_scrn\n", encoding="utf-8")
    output = tmp_path / "canonical"

    manifest = convert_cadac_tree_to_table_bundle(source_root, output)
    loaded = read_table_bundle(output)

    assert manifest.metadata["deck_count"] == 2
    assert len(loaded.tables) == 2
    assert {table.dimension for table in loaded.tables} == {1, 2}


####


def test_conversion_cli_and_validation_cli(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    source = tmp_path / "physical.asc"
    source.write_text(SYNTHETIC_DECK, encoding="utf-8")
    output = tmp_path / "canonical"

    assert main(["convert-deck", str(source), "--output", str(output)]) == 0
    convert_payload = json.loads(capsys.readouterr().out)
    assert convert_payload["schema"] == "taoryx.table_bundle.v1"
    assert main(["validate-table", str(output)]) == 0
    validate_payload = json.loads(capsys.readouterr().out)
    assert validate_payload["table_count"] == 3


####


def test_convert_tree_cli(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    source_root = tmp_path / "CADAC"
    source_root.mkdir()
    (source_root / "deck.asc").write_text(SYNTHETIC_DECK, encoding="utf-8")
    output = tmp_path / "canonical-tree"

    assert main(["convert-tree", str(source_root), "--output", str(output)]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["metadata"]["conversion_kind"] == "source_tree"
    assert payload["metadata"]["deck_count"] == 1


####
