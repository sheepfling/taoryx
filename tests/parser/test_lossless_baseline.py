from pathlib import Path

import pytest

from taoryx.language.lossless import parse_lossless_bytes, parse_lossless_file

BASELINE_FIXTURES = sorted(Path("examples/chapter03").glob("*.tbl")) + sorted(
    Path("examples/chapter04").glob("*.prb")
) + sorted(Path("examples/chapter04").glob("*.tbl"))


@pytest.mark.parametrize("path", BASELINE_FIXTURES, ids=lambda path: str(path))
def test_baseline_fixture_round_trips_byte_for_byte(path: Path) -> None:
    document = parse_lossless_file(path)

    assert document.render_bytes() == path.read_bytes()
    assert [record.source.line_start for record in document.records] == list(
        range(1, len(document.records) + 1)
    )
    assert all(record.source.path == str(path) for record in document.records)


def test_lossless_parser_preserves_mixed_newlines_and_comments() -> None:
    source = b"# comment\r\n\r\nvalue = 1\nlast = 2\r"
    parsed = parse_lossless_bytes(source, source_path="fixture.prb")

    assert parsed.newline == "mixed"
    assert parsed.final_newline is True
    assert [record.kind for record in parsed.records] == ["comment", "blank", "content", "content"]
    assert parsed.render_bytes() == source
