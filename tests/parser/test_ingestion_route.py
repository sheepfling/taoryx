from pathlib import Path

import pytest

from taoryx.language.ingest import FileKind, UnsupportedFileKindError, ingest_file, kind_for_path

BASELINE_FIXTURES = sorted(Path("examples/chapter03").glob("*.tbl")) + sorted(
    Path("examples/chapter04").glob("*.prb")
) + sorted(Path("examples/chapter04").glob("*.tbl"))


@pytest.mark.parametrize("path", BASELINE_FIXTURES, ids=lambda path: str(path))
def test_ingest_route_preserves_and_validates_baseline(path: Path) -> None:
    result = ingest_file(path)

    assert result.kind is (FileKind.PROBLEM if path.suffix == ".prb" else FileKind.TABLE)
    assert result.source.render_bytes() == path.read_bytes()
    assert result.valid


def test_kind_detection_and_unsupported_extension() -> None:
    assert kind_for_path("example.prb") is FileKind.PROBLEM
    assert kind_for_path("example.tbl") is FileKind.TABLE
    with pytest.raises(UnsupportedFileKindError):
        kind_for_path("example.txt")
