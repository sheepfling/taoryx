from __future__ import annotations

import csv
import json
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CSV_PATH = ROOT / "metadata" / "equations_provenance.csv"
JSON_PATH = ROOT / "metadata" / "equations_provenance.json"


def expected_equations() -> list[str]:
    return (
        ["1-1", "1-2"]
        + [f"2-{number}" for number in range(1, 316)]
        + ["3-1"]
        + [f"4-{number}" for number in range(1, 9)]
    )
####


def test_equation_provenance_is_complete_and_ordered() -> None:
    with CSV_PATH.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    ####
    assert [row["equation"] for row in rows] == expected_equations()
    assert len(rows) == 326
    assert len({row["latex_label"] for row in rows}) == 326
    assert len({row["latex_snippet_sha256"] for row in rows}) >= 320
    assert Counter(row["transcription_status"] for row in rows) == {
        "visually_verified": 326
    }
####


def test_equation_provenance_has_source_and_tex_locations() -> None:
    with CSV_PATH.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    ####
    for row in rows:
        assert 1 <= int(row["source_pdf_page"]) <= 307
        assert row["source_manual_page"]
        assert row["tex_file"].endswith(".tex")
        assert (ROOT / row["tex_file"]).exists()
        assert int(row["tex_line_start"]) <= int(row["tex_label_line"]) <= int(
            row["tex_line_end"]
        )
        assert len(row["source_pdf_sha256"]) == 64
        assert len(row["latex_snippet_sha256"]) == 64
    ####
####


def test_equation_json_declares_implementation_boundary() -> None:
    payload = json.loads(JSON_PATH.read_text(encoding="utf-8"))
    assert payload["summary"]["equations_total"] == 326
    assert payload["summary"]["coverage"] == {
        "canonical_sequence_complete": True,
        "metadata_to_aux_one_to_one": True,
        "aux_to_tex_one_to_one": True,
        "source_page_labels_verified": True,
        "latex_snippet_hashes_recorded": True,
    }
    assert "not yet all implemented" in payload["summary"]["implementation_boundary"]
####
