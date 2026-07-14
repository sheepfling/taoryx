from __future__ import annotations

from pathlib import Path

import yaml

from taoryx.language.ingest import ingest_file
from taoryx.language.table_parser import parse_table_file, table_type_catalog, table_variable_catalog

TABLE_ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "table_examples_v1"
HARNESS_ROOT = TABLE_ROOT / "harnesses"


def _load_table_catalogs() -> tuple[dict[str, str], dict[str, set[str]]]:
    table_types: dict[str, str] = {}
    table_variables: dict[str, set[str]] = {}
    for path in sorted(TABLE_ROOT.rglob("*.tbl")):
        document = parse_table_file(path)
        table_types.update(table_type_catalog(document))
        table_variables.update(table_variable_catalog(document))
    return table_types, table_variables


def test_table_example_harness_manifest_lists_every_problem_file() -> None:
    manifest = yaml.safe_load((HARNESS_ROOT / "manifest.yaml").read_text(encoding="utf-8"))
    listed = sorted(manifest["problem_files"])
    actual = sorted(path.name for path in HARNESS_ROOT.glob("*.prb"))

    assert listed == actual


def test_table_example_harnesses_ingest_cleanly_with_example_tables() -> None:
    table_types, table_variables = _load_table_catalogs()

    for path in sorted(HARNESS_ROOT.glob("*.prb")):
        result = ingest_file(path, available_tables=table_types, available_table_variables=table_variables)
        assert result.source.render_bytes() == path.read_bytes(), path
        assert result.valid, path

