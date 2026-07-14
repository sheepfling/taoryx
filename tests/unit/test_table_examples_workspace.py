from __future__ import annotations

from pathlib import Path

import yaml

from taoryx.language.ingest import ingest_file
from taoryx.language.semantic_validation import validate_table_file
from taoryx.language.table_parser import parse_table_file

ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "table_examples_v1"


def test_table_examples_manifest_lists_every_example_table() -> None:
    manifest = yaml.safe_load((ROOT / "manifest.yaml").read_text(encoding="utf-8"))
    listed = sorted(manifest["tables"])
    actual = sorted(path.relative_to(ROOT).as_posix() for path in ROOT.rglob("*.tbl"))

    assert listed == actual


def test_table_examples_parse_cleanly() -> None:
    for path in sorted(ROOT.rglob("*.tbl")):
        document = parse_table_file(path)
        assert document.tables, path
        assert not [diagnostic for diagnostic in document.diagnostics if diagnostic.severity == "error"], path
        assert not [diagnostic for diagnostic in validate_table_file(document) if diagnostic.severity == "error"], path


def test_table_examples_ingest_cleanly() -> None:
    for path in sorted(ROOT.rglob("*.tbl")):
        result = ingest_file(path)
        assert result.source.render_bytes() == path.read_bytes(), path
        assert result.valid, path
