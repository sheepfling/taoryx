from __future__ import annotations

import csv
import json

from pypdf import PdfReader

from .audit_equation_provenance import ROOT, asdict, build_records

SOURCE_PDF = ROOT / "TAOS_manual_1995.pdf"
AUX_PATH = ROOT / "build" / "manual.aux"
CSV_PATH = ROOT / "metadata" / "equations_provenance.csv"
JSON_PATH = ROOT / "metadata" / "equations_provenance.json"
MARKDOWN_PATH = ROOT / "qa" / "equation_provenance_audit_v21.md"
PDF_PATH = ROOT / "qa" / "TAOS_equation_provenance_audit_v21.pdf"
SOURCE_RENDER_DIR = ROOT / "qa" / "equation-source-pages"
SOURCE_PDF_AVAILABLE = SOURCE_PDF.exists()


def main() -> None:
    records, summary = build_records(SOURCE_PDF, AUX_PATH)
    expected_rows = [asdict(record) for record in records]

    with CSV_PATH.open(newline="", encoding="utf-8") as handle:
        actual_csv_rows = list(csv.DictReader(handle))
    ####
    if len(actual_csv_rows) != 326:
        raise SystemExit(f"Expected 326 CSV equation records, found {len(actual_csv_rows)}")
    ####
    for expected, actual in zip(expected_rows, actual_csv_rows, strict=True):
        normalized = {key: str(value) for key, value in expected.items()}
        if actual != normalized:
            raise SystemExit(
                f"Stale equation CSV record for {expected['equation']}:\n"
                f"expected={normalized}\nactual={actual}"
            )
        ####
    ####

    payload = json.loads(JSON_PATH.read_text(encoding="utf-8"))
    if payload["equations"] != expected_rows:
        raise SystemExit("metadata/equations_provenance.json is stale.")
    ####
    for key in (
        "source_pdf_sha256",
        "source_pdf_pages",
        "equations_total",
        "unique_latex_labels",
        "unique_source_pages_with_equations",
    ):
        if payload["summary"].get(key) != summary.get(key):
            raise SystemExit(f"Stale equation JSON summary field: {key}")
        ####
    ####

    markdown = MARKDOWN_PATH.read_text(encoding="utf-8")
    required_phrases = (
        "Canonical numbered equations: **326**",
        "2-1 through 2-315",
        "not yet all implemented as executable Python functions",
    )
    for phrase in required_phrases:
        if phrase not in markdown:
            raise SystemExit(f"Equation audit Markdown is missing: {phrase}")
        ####
    ####

    reader = PdfReader(PDF_PATH, strict=True)
    if len(reader.pages) < 2:
        raise SystemExit("Equation provenance PDF is unexpectedly short.")
    ####

    if SOURCE_PDF_AVAILABLE:
        expected_render_names = {
            f"page-{record.source_pdf_page:03d}.png" for record in records
        }
        actual_render_names = {path.name for path in SOURCE_RENDER_DIR.glob("page-*.png")}
        if actual_render_names != expected_render_names:
            raise SystemExit(
                "Equation source-page render set is stale: "
                f"missing={sorted(expected_render_names - actual_render_names)}, "
                f"extra={sorted(actual_render_names - expected_render_names)}"
            )
    ####

    print(
        "Equation provenance checks passed: 326 canonical equations, "
        "one-to-one metadata/AUX/TeX mapping, audit PDF, "
        + ("and source-page renders." if SOURCE_PDF_AVAILABLE else "and cached source context.")
    )
####


if __name__ == "__main__":
    main()
####
