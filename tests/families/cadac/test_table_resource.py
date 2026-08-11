from __future__ import annotations

import json
from pathlib import Path

import pytest
from taoryx.families.cadac.table_resources import (
    TableAxis,
    TableAxisDirection,
    TableBoundaryMode,
    TableExtrapolation,
    TableProvenance,
    TableResource,
    TableResourceError,
    TableUnit,
    TableUnitEvidence,
    canonical_json_bytes,
    read_table_bundle,
    read_table_resource,
    write_table_bundle,
)


def _table(table_id: str = "example.table", name: str = "value_vs_x") -> TableResource:
    return TableResource(
        table_id=table_id,
        name=name,
        dimension=1,
        axes=(
            TableAxis(
                name="x",
                source_name="X1",
                values=(0.0, 1.0, 2.0),
                unit=TableUnit(symbol="s", evidence=TableUnitEvidence.DECLARED),
                source_direction=TableAxisDirection.ASCENDING,
            ),
        ),
        values=(0.0, 10.0, 20.0),
        extrapolation=TableExtrapolation(
            lower=TableBoundaryMode.LINEAR,
            upper=TableBoundaryMode.CLAMP,
        ),
        provenance=TableProvenance(
            provider="fixture",
            source_path="fixture.asc",
            source_sha256="0" * 64,
            source_table_name=name,
            source_dimension=1,
            source_axis_order=("X1",),
            source_line=1,
            converter="fixture",
            converter_version="1",
        ),
    )


####


def test_canonical_runtime_preserves_per_side_boundary_behavior() -> None:
    table = _table()

    assert table.lookup(0.5) == pytest.approx(5.0)
    assert table.lookup(-1.0) == pytest.approx(-10.0)
    assert table.lookup(3.0) == pytest.approx(20.0)


####


def test_canonical_json_is_deterministic() -> None:
    table = _table()

    assert canonical_json_bytes(table) == canonical_json_bytes(table)
    assert canonical_json_bytes(table).endswith(b"\n")


####


def test_bundle_round_trip_verifies_resource_hashes(tmp_path: Path) -> None:
    manifest = write_table_bundle(
        tmp_path / "bundle",
        bundle_id="fixture-bundle",
        tables=(_table(),),
        metadata={"purpose": "test"},
    )
    loaded = read_table_bundle(tmp_path / "bundle")

    assert loaded.manifest == manifest
    assert loaded.table("example.table").lookup(1.5) == pytest.approx(15.0)
    assert loaded.manifest.metadata["purpose"] == "test"


####


def test_bundle_file_names_are_collision_safe(tmp_path: Path) -> None:
    first = _table("collision:a", "first")
    second = _table("collision-a", "second")
    manifest = write_table_bundle(
        tmp_path / "bundle",
        bundle_id="collisions",
        tables=(first, second),
    )

    assert len({entry.resource for entry in manifest.tables}) == 2
    assert all((tmp_path / "bundle" / entry.resource).is_file() for entry in manifest.tables)


####


def test_resource_checksum_mismatch_fails_closed(tmp_path: Path) -> None:
    manifest = write_table_bundle(tmp_path / "bundle", bundle_id="fixture", tables=(_table(),))
    entry = manifest.tables[0]
    resource_path = tmp_path / "bundle" / entry.resource
    payload = json.loads(resource_path.read_text(encoding="utf-8"))
    payload["values"][0] = 99.0
    resource_path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(TableResourceError, match="checksum mismatch"):
        read_table_bundle(tmp_path / "bundle")
    ####


####


def test_malformed_resource_fails_schema_validation(tmp_path: Path) -> None:
    path = tmp_path / "bad.table.json"
    path.write_text('{"schema":"taoryx.table.v1","table_id":"bad"}', encoding="utf-8")

    with pytest.raises(TableResourceError, match="invalid canonical table resource"):
        read_table_resource(path)
    ####


####
