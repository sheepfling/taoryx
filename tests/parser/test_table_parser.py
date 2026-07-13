from pathlib import Path

import pytest

from taoryx.language.semantic_validation import validate_table_file
from taoryx.language.table_parser import parse_table_file

FIXTURES = sorted(Path("examples/chapter03").glob("*.tbl")) + sorted(Path("examples/chapter04").glob("*.tbl"))


@pytest.mark.parametrize("path", FIXTURES)
def test_manual_table_fixture_parses(path: Path) -> None:
    document = parse_table_file(path)
    assert document.tables
    assert not [item for item in validate_table_file(document) if item.severity == "error"]
####
