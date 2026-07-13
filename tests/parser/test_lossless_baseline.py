from pathlib import Path
from random import Random

import pytest

from taoryx.language.ingest import ingest_file
from taoryx.language.lossless import parse_lossless_bytes, parse_lossless_file
from taoryx.language.problem_parser import parse_problem_file
from taoryx.language.table_parser import parse_table_file

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


@pytest.mark.parametrize(
    ("suffix", "source"),
    [
        (".prb", b"(demo)\n*unknown \x80\n*end\n"),
        (".tbl", b"not-a-table \x80\n"),
    ],
)
def test_file_ingestion_preserves_non_utf8_source_bytes(tmp_path: Path, suffix: str, source: bytes) -> None:
    path = tmp_path / f"non-utf8{suffix}"
    path.write_bytes(source)

    result = ingest_file(path)

    assert result.source.render_bytes() == source
    assert result.diagnostics
    assert all(diagnostic.location is not None for diagnostic in result.diagnostics)
    assert all(record.location is not None for record in result.document.recovered_records)
    ####


@pytest.mark.parametrize("suffix, parser", [(".prb", parse_problem_file), (".tbl", parse_table_file)])
def test_direct_file_parsers_use_lossless_surrogateescape(tmp_path: Path, suffix: str, parser) -> None:
    path = tmp_path / f"direct{suffix}"
    source = b"(demo)\n*unknown \x80\n*end\n" if suffix == ".prb" else b"not-a-table \x80\n"
    path.write_bytes(source)

    document = parser(path)

    assert document.diagnostics
    assert all(diagnostic.location is not None for diagnostic in document.diagnostics)
    assert all(record.location is not None for record in document.recovered_records)
    ####


def test_file_ingestion_recovers_deterministic_arbitrary_bytes_without_exceptions(tmp_path: Path) -> None:
    randomizer = Random(1995)

    for suffix in (".prb", ".tbl"):
        for index in range(100):
            source = bytes(randomizer.randrange(256) for _ in range(randomizer.randrange(180)))
            path = tmp_path / f"arbitrary-{suffix[1:]}-{index}{suffix}"
            path.write_bytes(source)

            result = ingest_file(path)

            assert result.source.render_bytes() == source
            assert all(diagnostic.location is not None for diagnostic in result.diagnostics)
            source_lines = source.decode("utf-8", errors="surrogateescape").splitlines()
            assert all(record.text == source_lines[record.location.line - 1] for record in result.document.recovered_records)
    ####
