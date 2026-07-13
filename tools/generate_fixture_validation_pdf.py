from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import inch
from reportlab.platypus import (
    BaseDocTemplate,
    Frame,
    PageBreak,
    PageTemplate,
    Paragraph,
    Spacer,
    Table,
    TableStyle,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate the TAOS fixture parser-validation PDF from JSON."
    )
    parser.add_argument("input_json", type=Path)
    parser.add_argument("output_pdf", type=Path)
    return parser.parse_args()
####


def issue_text(counts: dict[str, int]) -> str:
    return (
        f"{counts.get('error', 0)} error, "
        f"{counts.get('warning', 0)} warning, "
        f"{counts.get('note', 0)} note"
    )
####


def make_styles() -> dict[str, ParagraphStyle]:
    base = getSampleStyleSheet()
    return {
        "title": ParagraphStyle(
            "TitleCustom",
            parent=base["Title"],
            fontName="Helvetica-Bold",
            fontSize=17,
            leading=21,
            spaceAfter=10,
            alignment=TA_LEFT,
        ),
        "heading": ParagraphStyle(
            "HeadingCustom",
            parent=base["Heading2"],
            fontName="Helvetica-Bold",
            fontSize=12.5,
            leading=15,
            spaceBefore=8,
            spaceAfter=7,
        ),
        "body": ParagraphStyle(
            "BodyCustom",
            parent=base["BodyText"],
            fontName="Times-Roman",
            fontSize=9.5,
            leading=12.5,
            spaceAfter=6,
        ),
        "small": ParagraphStyle(
            "SmallCustom",
            parent=base["BodyText"],
            fontName="Times-Roman",
            fontSize=8.3,
            leading=10.2,
        ),
        "cell": ParagraphStyle(
            "CellCustom",
            parent=base["BodyText"],
            fontName="Helvetica",
            fontSize=7.6,
            leading=9.2,
        ),
        "cell_mono": ParagraphStyle(
            "CellMonoCustom",
            parent=base["BodyText"],
            fontName="Courier",
            fontSize=6.8,
            leading=8.4,
            wordWrap="CJK",
        ),
        "header": ParagraphStyle(
            "HeaderCustom",
            parent=base["BodyText"],
            fontName="Helvetica-Bold",
            fontSize=7.5,
            leading=9,
        ),
        "bullet": ParagraphStyle(
            "BulletCustom",
            parent=base["BodyText"],
            fontName="Times-Roman",
            fontSize=9.1,
            leading=12,
            leftIndent=13,
            firstLineIndent=-7,
            bulletIndent=4,
            spaceAfter=4,
        ),
    }
####


def p(text: str, style: ParagraphStyle) -> Paragraph:
    safe = (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
    )
    return Paragraph(safe, style)
####


def build_summary_table(data: dict[str, Any], styles: dict[str, ParagraphStyle]) -> Table:
    summary = data["summary"]
    issues = summary["issue_counts"]
    rows: list[list[Any]] = [
        [p("Metric", styles["header"]), p("Result", styles["header"])],
        [p(".tbl fixtures parsed", styles["cell"]), str(summary["table_fixture_count"])],
        [p(".prb fixtures parsed", styles["cell"]), str(summary["problem_fixture_count"])],
        [p("Tables represented", styles["cell"]), str(summary["table_count"])],
        [p("Trajectories represented", styles["cell"]), str(summary["trajectory_count"])],
        [p("Segments represented", styles["cell"]), str(summary["segment_count"])],
        [p("Parser errors", styles["cell"]), str(issues["error"])],
        [p("Warnings", styles["cell"]), str(issues["warning"])],
        [p("Informational notes", styles["cell"]), str(issues["note"])],
    ]
    table = Table(rows, colWidths=[2.35 * inch, 0.8 * inch], hAlign="LEFT")
    table.setStyle(
        TableStyle(
            [
                ("LINEABOVE", (0, 0), (-1, 0), 0.8, colors.black),
                ("LINEBELOW", (0, 0), (-1, 0), 0.5, colors.black),
                ("LINEBELOW", (0, -1), (-1, -1), 0.8, colors.black),
                ("ALIGN", (1, 1), (1, -1), "RIGHT"),
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("LEFTPADDING", (0, 0), (-1, -1), 5),
                ("RIGHTPADDING", (0, 0), (-1, -1), 5),
                ("TOPPADDING", (0, 0), (-1, -1), 2.5),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 2.5),
            ]
        )
    )
    return table
####


def build_table_fixture_table(
    data: dict[str, Any], styles: dict[str, ParagraphStyle]
) -> Table:
    rows: list[list[Any]] = [
        [
            p("Fixture", styles["header"]),
            p("Parsed table", styles["header"]),
            p("Status", styles["header"]),
            p("Issues", styles["header"]),
        ]
    ]
    for fixture in data["table_fixtures"]:
        table_items = ", ".join(
            f"{item['name']} ({item['format']})" for item in fixture["tables"]
        )
        status = (
            "executable-complete"
            if fixture["executable_complete"]
            else "documentation excerpt"
        )
        rows.append(
            [
                p(fixture["path"], styles["cell_mono"]),
                p(table_items, styles["cell_mono"]),
                p(status, styles["cell"]),
                p(issue_text(fixture["issue_counts"]), styles["cell"]),
            ]
        )
    table = Table(
        rows,
        colWidths=[2.30 * inch, 1.20 * inch, 1.35 * inch, 1.25 * inch],
        repeatRows=1,
        hAlign="LEFT",
    )
    table.setStyle(common_table_style())
    return table
####


def build_problem_fixture_table(
    data: dict[str, Any], styles: dict[str, ParagraphStyle]
) -> Table:
    rows: list[list[Any]] = [
        [
            p("Fixture", styles["header"]),
            p("Problem", styles["header"]),
            p("Traj.", styles["header"]),
            p("Seg.", styles["header"]),
            p("Table refs.", styles["header"]),
            p("Issues", styles["header"]),
        ]
    ]
    for fixture in data["problem_fixtures"]:
        rows.append(
            [
                p(fixture["path"], styles["cell_mono"]),
                p(fixture["problem_name"], styles["cell_mono"]),
                str(fixture["trajectory_count"]),
                str(fixture["segment_count"]),
                str(len(fixture["table_references"])),
                p(issue_text(fixture["issue_counts"]), styles["cell"]),
            ]
        )
    table = Table(
        rows,
        colWidths=[2.40 * inch, 0.95 * inch, 0.45 * inch, 0.45 * inch, 0.65 * inch, 1.20 * inch],
        repeatRows=1,
        hAlign="LEFT",
    )
    style = common_table_style()
    style.add("ALIGN", (2, 1), (4, -1), "RIGHT")
    table.setStyle(style)
    return table
####


def common_table_style() -> TableStyle:
    return TableStyle(
        [
            ("LINEABOVE", (0, 0), (-1, 0), 0.8, colors.black),
            ("LINEBELOW", (0, 0), (-1, 0), 0.5, colors.black),
            ("LINEBELOW", (0, -1), (-1, -1), 0.8, colors.black),
            ("VALIGN", (0, 0), (-1, -1), "TOP"),
            ("LEFTPADDING", (0, 0), (-1, -1), 3.5),
            ("RIGHTPADDING", (0, 0), (-1, -1), 3.5),
            ("TOPPADDING", (0, 0), (-1, -1), 3),
            ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
        ]
    )
####


def add_page_number(canvas: Any, doc: BaseDocTemplate) -> None:
    canvas.saveState()
    canvas.setFont("Helvetica", 8)
    canvas.drawCentredString(letter[0] / 2, 0.32 * inch, str(doc.page))
    canvas.restoreState()
####


def build_pdf(data: dict[str, Any], output_path: Path) -> None:
    styles = make_styles()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    doc = BaseDocTemplate(
        str(output_path),
        pagesize=letter,
        leftMargin=0.55 * inch,
        rightMargin=0.55 * inch,
        topMargin=0.48 * inch,
        bottomMargin=0.52 * inch,
        title="TAOS fixture parser-validation report",
        author="TAOS Manual Reconstruction Project",
    )
    frame = Frame(
        doc.leftMargin,
        doc.bottomMargin,
        doc.width,
        doc.height,
        id="normal",
    )
    doc.addPageTemplates(
        [PageTemplate(id="main", frames=[frame], onPage=add_page_number)]
    )

    story: list[Any] = [
        p("TAOS fixture parser-validation report", styles["title"]),
        p(
            "This report is generated by the repository's line-aware TAOS table and "
            "problem parsers. It validates the extracted documentation fixtures "
            "structurally and semantically; it does not claim runtime execution "
            "against the historical TAOS 96.0 binary.",
            styles["body"],
        ),
        p("Summary", styles["heading"]),
        build_summary_table(data, styles),
        Spacer(1, 7),
        p("Table fixtures", styles["heading"]),
        build_table_fixture_table(data, styles),
        PageBreak(),
        p("Problem fixtures", styles["heading"]),
        build_problem_fixture_table(data, styles),
        Spacer(1, 8),
        p("Source-preserved findings", styles["heading"]),
        Paragraph(
            "• The Orbus <font name='Courier'>tmark</font> array contains consecutive "
            "<font name='Courier'>0.00</font> entries. This is present in the printed "
            "source on manual page 3-26, even though Section 3.4 says duplicate "
            "independent values are not allowed. The parser reports it as a warning "
            "rather than silently changing the fixture.",
            styles["bullet"],
        ),
        Paragraph(
            "• <font name='Courier'>reentry-ca-excerpt.tbl</font> contains the manual's "
            "explicit ellipsis. The parser accepts the syntax as a documentation "
            "excerpt but marks it non-executable-complete.",
            styles["bullet"],
        ),
        Paragraph(
            "• Table references not bundled with a worked example are treated as "
            "external dependencies, not syntax failures.",
            styles["bullet"],
        ),
        p("Validation boundary", styles["heading"]),
        p(
            "The parsers check hierarchy, token structure, table cardinality, "
            "independent-variable ordering, trajectory and segment uniqueness, "
            "start and goto targets, inherited initial-state references, indexed "
            "trajectory references, survey and optimization parameter definitions, "
            "and *when conditions and actions. Numerical trajectory results remain "
            "unverified without the historical executable or an independent "
            "compatible implementation.",
            styles["body"],
        ),
    ]
    doc.build(story)
####


def main() -> None:
    args = parse_args()
    data = json.loads(args.input_json.read_text(encoding="utf-8"))
    build_pdf(data, args.output_pdf)
####


if __name__ == "__main__":
    main()
####
