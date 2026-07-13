from __future__ import annotations

import csv
import subprocess
from pathlib import Path

import fitz

ROOT = Path(__file__).resolve().parents[1]
LISTING_FIGURES = {"4-5", "4-6", "4-8", "4-9", "4-10", "4-12"}


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))
####


def check_source_coverage() -> None:
    rows = read_csv(ROOT / "metadata" / "source_page_coverage.csv")
    pages = [int(row["source_pdf_page"]) for row in rows]
    if len(rows) != 307 or pages != list(range(1, 308)):
        raise RuntimeError("Source-page coverage must contain exactly pages 1 through 307.")
    ####
    if any(row["status"] != "complete" for row in rows):
        raise RuntimeError("Every source-page coverage record must be complete.")
    ####
####


def check_figure_registry() -> None:
    rows = read_csv(ROOT / "metadata" / "figures.csv")
    if len(rows) != 72:
        raise RuntimeError(f"Expected 72 numbered figures, found {len(rows)}.")
    ####
    if any(row["status"] != "visually_verified" for row in rows):
        raise RuntimeError("Every figure must be visually verified.")
    ####
    expected_formats = {"tikz": 53, "pgfplots": 12, "latex": 1, "latex-listing": 6}
    format_counts: dict[str, int] = {}
    editable_count = 0
    for row in rows:
        figure = row["figure"]
        target_format = row["target_format"]
        format_counts[target_format] = format_counts.get(target_format, 0) + 1
        if figure in LISTING_FIGURES:
            continue
        ####
        chapter = int(figure.split("-")[0])
        figure_path = ROOT / "figures" / f"chapter{chapter:02d}" / f"fig-{figure}.tex"
        if not figure_path.exists():
            raise RuntimeError(f"Missing editable figure source: {figure_path}")
        ####
        editable_count += 1
    ####
    if format_counts != expected_formats:
        raise RuntimeError(f"Unexpected figure-format counts: {format_counts}")
    ####
    if editable_count != 66:
        raise RuntimeError(f"Expected 66 editable semantic figure sources, found {editable_count}.")
    ####
####


def check_no_raster_references() -> None:
    roots = [ROOT / "frontmatter", ROOT / "chapters", ROOT / "backmatter"]
    offenders: list[str] = []
    for directory in roots:
        for path in directory.rglob("*.tex"):
            if "\\includegraphics" in path.read_text(encoding="utf-8"):
                offenders.append(str(path.relative_to(ROOT)))
            ####
        ####
    ####
    if offenders:
        raise RuntimeError(f"Raster includegraphics commands remain in manual sources: {offenders}")
    ####
####


def check_built_pdf() -> None:
    pdf_path = ROOT / "build" / "manual.pdf"
    if not pdf_path.exists():
        raise RuntimeError("build/manual.pdf does not exist.")
    ####
    document = fitz.open(pdf_path)
    if len(document) < 180:
        raise RuntimeError(f"Unexpectedly short manual PDF: {len(document)} pages.")
    ####
    image_list = subprocess.run(
        ["pdfimages", "-list", str(pdf_path)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()[2:]
    if any(line.strip() for line in image_list):
        raise RuntimeError("The final manual PDF still contains raster image objects.")
    ####
    log_text = (ROOT / "build" / "manual.log").read_text(encoding="utf-8", errors="replace")
    forbidden = ["Overfull \\hbox", "There were undefined references", "multiply defined"]
    for marker in forbidden:
        if marker in log_text:
            raise RuntimeError(f"LaTeX log contains forbidden marker: {marker}")
        ####
    ####
####


def check_qa_outputs() -> None:
    report = ROOT / "qa" / "build" / "figure_comparison.pdf"
    if not report.exists() or len(fitz.open(report)) != 73:
        raise RuntimeError("The visual figure comparison report must contain 73 pages.")
    ####
    comparison_tex = (ROOT / "qa" / "figure_comparison.tex").read_text(encoding="utf-8")
    if comparison_tex.count(r"Status: visually\_verified") != 72:
        raise RuntimeError("The figure-comparison source must contain 72 visually verified records.")
    ####
    qa_log_path = ROOT / "build" / "figure_comparison.log"
    if not qa_log_path.exists():
        raise RuntimeError("The figure-comparison LaTeX log is missing.")
    ####
    qa_log = qa_log_path.read_text(encoding="utf-8", errors="replace")
    forbidden = [r"Overfull \hbox", "There were undefined references", "multiply defined"]
    for marker in forbidden:
        if marker in qa_log:
            raise RuntimeError(f"Figure-comparison log contains forbidden marker: {marker}")
        ####
    ####
    contact_sheets = sorted((ROOT / "qa" / "page-contact-sheets").glob("contact-*.jpg"))
    expected_contact_sheets = (len(fitz.open(ROOT / "build" / "manual.pdf")) + 15) // 16
    if len(contact_sheets) != expected_contact_sheets:
        raise RuntimeError(
            f"Expected {expected_contact_sheets} final-manual contact sheets, "
            f"found {len(contact_sheets)}."
        )
    ####
####


def main() -> None:
    check_source_coverage()
    check_figure_registry()
    check_no_raster_references()
    check_built_pdf()
    check_qa_outputs()
    print("Visual/vector sweep checks passed.")
####


if __name__ == "__main__":
    main()
####
