from __future__ import annotations

import csv
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "qa" / "source_text_fidelity.json"
MARKDOWN = ROOT / "qa" / "source_text_fidelity.md"
CSV_REPORT = ROOT / "qa" / "source_sentence_matches.csv"
RECONSTRUCTION_NOTE = ROOT / "frontmatter" / "reconstruction_note.tex"
README = ROOT / "README.md"
EQUATIONS = ROOT / "metadata" / "equations.csv"


def main() -> None:
    for path in (REPORT, MARKDOWN, CSV_REPORT, RECONSTRUCTION_NOTE):
        if not path.exists() or path.stat().st_size == 0:
            raise SystemExit(f"Missing source-fidelity artifact: {path.relative_to(ROOT)}")
        ####
    ####
    payload = json.loads(REPORT.read_text(encoding="utf-8"))
    if payload["source_pdf_pages"] != 307:
        raise SystemExit("Source-fidelity report must cover 307 source pages.")
    ####
    if payload["source_sentence_count"] < 2000:
        raise SystemExit("Source-fidelity report contains too few narrative sentences.")
    ####
    structural = payload["structural"]
    if structural["source_pages_mapped"] != 307:
        raise SystemExit("Source-fidelity structural coverage is incomplete.")
    ####
    if structural["equations_complete_or_verified"] != structural["equation_records"]:
        raise SystemExit("One or more equation records remain below visually verified status.")
    ####
    expected_chapters = {
        "Front matter",
        "Chapter 1",
        "Chapter 2",
        "Chapter 3",
        "Chapter 4",
        "Appendix",
        "References",
    }
    if not expected_chapters.issubset(payload["chapter_metrics"]):
        raise SystemExit("Source-fidelity report is missing required chapter metrics.")
    ####
    note = RECONSTRUCTION_NOTE.read_text(encoding="utf-8")
    for phrase in ("not a page facsimile", "word-for-word", "have not been executed"):
        if phrase not in note:
            raise SystemExit(f"Reconstruction note is missing required disclosure: {phrase!r}.")
        ####
    ####
    readme = README.read_text(encoding="utf-8")
    if "not a diplomatic transcription" not in readme:
        raise SystemExit("README must state that the edition is not a diplomatic transcription.")
    ####
    with EQUATIONS.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    ####
    incomplete = [row["equation"] for row in rows if row["status"] not in {"visually_verified", "complete"}]
    if incomplete:
        raise SystemExit("Equation registry has incomplete entries: " + ", ".join(incomplete))
    ####
    print("Source-text fidelity artifact checks passed.")
####


if __name__ == "__main__":
    main()
####
