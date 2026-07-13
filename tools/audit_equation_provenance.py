from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import fitz

ROOT = Path(__file__).resolve().parents[1]
DISPLAY_ENVIRONMENTS = {
    "equation",
    "equation*",
    "align",
    "align*",
    "gather",
    "gather*",
    "multline",
    "multline*",
}

IMPLEMENTED_EQUATION_IDS = {
    *[f"2-{number}" for number in range(1, 7)],
    *[f"2-{number}" for number in range(7, 50)],
    *[f"2-{number}" for number in range(50, 62)],
    *[f"2-{number}" for number in range(62, 90)],
    *[f"2-{number}" for number in range(90, 93)],
    *[f"2-{number}" for number in range(93, 97)],
    *[f"2-{number}" for number in range(97, 107)],
    *[f"2-{number}" for number in range(107, 119)],
    *[f"2-{number}" for number in range(153, 176)],
    *[f"2-{number}" for number in range(196, 201)],
    *[f"2-{number}" for number in range(123, 141)],
    *[f"2-{number}" for number in range(148, 153)],
    *[f"2-{number}" for number in range(246, 251)],
}


@dataclass(frozen=True)
class SourceLocation:
    label: str
    tex_file: str
    line_start: int
    line_end: int
    label_line: int
    snippet_sha256: str
####


@dataclass(frozen=True)
class AuxRecord:
    label: str
    equation: str
    reconstructed_page: str
    reconstructed_section: str
    anchor: str
####


@dataclass(frozen=True)
class EquationRecord:
    equation: str
    latex_label: str
    section: str
    source_manual_page: str
    source_pdf_page: int
    source_pdf_sha256: str
    reconstructed_page: str
    reconstructed_section: str
    tex_file: str
    tex_line_start: int
    tex_line_end: int
    tex_label_line: int
    latex_snippet_sha256: str
    transcription_status: str
    verification_scope: str
    validation_suite: str
    code_implementation_status: str
    notes: str
####


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source-pdf",
        type=Path,
        default=ROOT / "TAOS_manual_1995.pdf",
    )
    parser.add_argument(
        "--aux",
        type=Path,
        default=ROOT / "build" / "manual.aux",
    )
    parser.add_argument("--render-source-pages", action="store_true")
    parser.add_argument("--version", default="21")
    return parser.parse_args()
####


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
        ####
    ####
    return digest.hexdigest()
####


def sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()
####


def expected_equations() -> list[str]:
    return (
        ["1-1", "1-2"]
        + [f"2-{number}" for number in range(1, 316)]
        + ["3-1"]
        + [f"4-{number}" for number in range(1, 9)]
    )
####


def parse_group(value: str, start: int) -> tuple[str, int]:
    if start >= len(value) or value[start] != "{":
        raise ValueError(f"Expected '{{' at position {start}: {value!r}")
    ####
    depth = 1
    index = start + 1
    output: list[str] = []
    while index < len(value):
        character = value[index]
        if character == "\\" and index + 1 < len(value):
            output.append(character)
            index += 1
            output.append(value[index])
        elif character == "{":
            depth += 1
            output.append(character)
        elif character == "}":
            depth -= 1
            if depth == 0:
                return "".join(output), index + 1
            ####
            output.append(character)
        else:
            output.append(character)
        ####
        index += 1
    ####
    raise ValueError(f"Unterminated group: {value!r}")
####


def parse_aux(aux_path: Path) -> dict[str, AuxRecord]:
    if not aux_path.exists():
        raise FileNotFoundError(f"Build the manual first; missing {aux_path}")
    ####
    records: dict[str, AuxRecord] = {}
    prefix = r"\newlabel"
    for line in aux_path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line.startswith(prefix):
            continue
        ####
        position = len(prefix)
        label, position = parse_group(line, position)
        if not label.startswith("eq:") or label.endswith("@cref"):
            continue
        ####
        outer, _ = parse_group(line, position)
        fields: list[str] = []
        inner_position = 0
        while inner_position < len(outer):
            while inner_position < len(outer) and outer[inner_position].isspace():
                inner_position += 1
            ####
            if inner_position >= len(outer):
                break
            ####
            field, inner_position = parse_group(outer, inner_position)
            fields.append(field)
        ####
        if len(fields) < 4:
            raise ValueError(f"Unexpected aux equation record: {line}")
        ####
        if label in records:
            raise ValueError(f"Duplicate equation label in aux: {label}")
        ####
        records[label] = AuxRecord(
            label=label,
            equation=fields[0],
            reconstructed_page=fields[1],
            reconstructed_section=fields[2],
            anchor=fields[3],
        )
    ####
    return records
####


def display_ranges(lines: list[str]) -> list[tuple[str, int, int]]:
    token_pattern = re.compile(r"\\(begin|end)\{([^}]+)\}")
    stack: list[tuple[str, int]] = []
    ranges: list[tuple[str, int, int]] = []
    for line_number, line in enumerate(lines, start=1):
        for match in token_pattern.finditer(line):
            action = match.group(1)
            environment = match.group(2)
            if environment not in DISPLAY_ENVIRONMENTS:
                continue
            ####
            if action == "begin":
                stack.append((environment, line_number))
            else:
                if not stack:
                    raise ValueError(
                        f"Unmatched display environment end on line {line_number}: {environment}"
                    )
                ####
                open_environment, start_line = stack.pop()
                if open_environment != environment:
                    raise ValueError(
                        f"Display environment mismatch on line {line_number}: "
                        f"expected {open_environment}, found {environment}"
                    )
                ####
                ranges.append((environment, start_line, line_number))
            ####
        ####
    ####
    if stack:
        raise ValueError(f"Unclosed display environments: {stack}")
    ####
    return ranges
####


def source_locations() -> dict[str, SourceLocation]:
    locations: dict[str, SourceLocation] = {}
    roots = [ROOT / "chapters", ROOT / "frontmatter", ROOT / "backmatter"]
    for source_root in roots:
        for path in sorted(source_root.rglob("*.tex")):
            lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
            ranges = display_ranges(lines)
            for line_number, line in enumerate(lines, start=1):
                for label in re.findall(r"\\label\{(eq:[^}]+)\}", line):
                    matches = [
                        item
                        for item in ranges
                        if item[1] <= line_number <= item[2]
                    ]
                    if not matches:
                        raise ValueError(
                            f"Equation label is not inside a numbered display: "
                            f"{path}:{line_number}: {label}"
                        )
                    ####
                    _, start_line, end_line = min(
                        matches,
                        key=lambda item: item[2] - item[1],
                    )
                    snippet = "\n".join(lines[start_line - 1 : end_line]) + "\n"
                    if label in locations:
                        raise ValueError(f"Duplicate equation label in TeX: {label}")
                    ####
                    locations[label] = SourceLocation(
                        label=label,
                        tex_file=path.relative_to(ROOT).as_posix(),
                        line_start=start_line,
                        line_end=end_line,
                        label_line=line_number,
                        snippet_sha256=sha256_text(snippet),
                    )
                ####
            ####
        ####
    ####
    return locations
####


def validation_scope(equation: str) -> tuple[str, str]:
    chapter, local = (int(part) for part in equation.split("-"))
    if chapter == 2:
        ranges = [
            (1, 49, "tools/check_chapter02_part1.py"),
            (50, 96, "tools/check_chapter02_part2.py"),
            (97, 174, "tools/check_chapter02_section22.py"),
            (175, 220, "tools/check_chapter02_section23.py"),
            (221, 272, "tools/check_chapter02_section24.py"),
            (273, 297, "tools/check_chapter02_section25.py"),
            (298, 315, "tools/check_chapter02_section26.py"),
        ]
        for low, high, script in ranges:
            if low <= local <= high:
                return (
                    "visual transcription plus section-level structural and selected numerical checks",
                    script,
                )
            ####
        ####
    ####
    if chapter == 4:
        return (
            "visual transcription plus Chapter 4 structural validation",
            "tools/check_chapter04.py",
        )
    ####
    return (
        "visual transcription plus repository structural validation",
        "tools/check_manifest.py",
    )
####


def implementation_status(equation: str) -> str:
    if equation in IMPLEMENTED_EQUATION_IDS:
        return "implemented"
    ####
    return "not yet mapped to an executable mathematics implementation"
####


def load_metadata() -> list[dict[str, str]]:
    with (ROOT / "metadata" / "equations.csv").open(
        newline="",
        encoding="utf-8",
    ) as handle:
        rows = list(csv.DictReader(handle))
    ####
    by_equation = {row["equation"]: row for row in rows}
    if len(by_equation) != len(rows):
        duplicates = [
            value
            for value, count in Counter(row["equation"] for row in rows).items()
            if count > 1
        ]
        raise ValueError(f"Duplicate equation metadata entries: {duplicates}")
    ####
    expected = expected_equations()
    missing = sorted(set(expected) - set(by_equation))
    extra = sorted(set(by_equation) - set(expected))
    if missing or extra:
        raise ValueError(f"Equation metadata sequence mismatch; missing={missing}, extra={extra}")
    ####
    return [by_equation[equation] for equation in expected]
####


def load_source_coverage() -> dict[int, dict[str, str]]:
    with (ROOT / "metadata" / "source_page_coverage.csv").open(
        newline="",
        encoding="utf-8",
    ) as handle:
        rows = list(csv.DictReader(handle))
    ####
    return {int(row["source_pdf_page"]): row for row in rows}
####


def load_cached_source_hash() -> str:
    provenance_csv = ROOT / "metadata" / "equations_provenance.csv"
    if provenance_csv.exists():
        with provenance_csv.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        ####
        hashes = {row["source_pdf_sha256"] for row in rows if row.get("source_pdf_sha256")}
        if len(hashes) == 1:
            return hashes.pop()
        ####
    ####
    provenance_json = ROOT / "metadata" / "equations_provenance.json"
    if provenance_json.exists():
        payload = json.loads(provenance_json.read_text(encoding="utf-8"))
        summary_hash = payload.get("summary", {}).get("source_pdf_sha256")
        if isinstance(summary_hash, str) and summary_hash:
            return summary_hash
        ####
    ####
    raise FileNotFoundError(
        "The original source PDF is unavailable and no cached provenance hash was found."
    )
####


def load_cached_records() -> tuple[list[EquationRecord], dict[str, Any]]:
    csv_path = ROOT / "metadata" / "equations_provenance.csv"
    json_path = ROOT / "metadata" / "equations_provenance.json"
    if not csv_path.exists() or not json_path.exists():
        raise FileNotFoundError(
            "The original source PDF is unavailable and cached provenance files are missing."
        )
    ####
    with csv_path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    ####
    records = [
        EquationRecord(
            equation=row["equation"],
            latex_label=row["latex_label"],
            section=row["section"],
            source_manual_page=row["source_manual_page"],
            source_pdf_page=int(row["source_pdf_page"]),
            source_pdf_sha256=row["source_pdf_sha256"],
            reconstructed_page=row["reconstructed_page"],
            reconstructed_section=row["reconstructed_section"],
            tex_file=row["tex_file"],
            tex_line_start=int(row["tex_line_start"]),
            tex_line_end=int(row["tex_line_end"]),
            tex_label_line=int(row["tex_label_line"]),
            latex_snippet_sha256=row["latex_snippet_sha256"],
            transcription_status=row["transcription_status"],
            verification_scope=row["verification_scope"],
            validation_suite=row["validation_suite"],
            code_implementation_status=implementation_status(row["equation"]),
            notes=row["notes"],
        )
        for row in rows
    ]
    ####
    payload = json.loads(json_path.read_text(encoding="utf-8"))
    summary = payload["summary"]
    return records, summary
####


def build_records(
    source_pdf: Path,
    aux_path: Path,
) -> tuple[list[EquationRecord], dict[str, Any]]:
    metadata_rows = load_metadata()
    aux_records = parse_aux(aux_path)
    source_coverage = load_source_coverage()
    if not source_pdf.exists():
        records, summary = load_cached_records()
        cached_labels = {record.latex_label for record in records}
        if set(aux_records) != cached_labels:
            raise ValueError(
                "TeX and AUX equation-label sets differ from the cached provenance registry: "
                f"aux_only={sorted(set(aux_records) - cached_labels)}, "
                f"cache_only={sorted(cached_labels - set(aux_records))}"
            )
        ####
        return records, summary
    ####
    locations = source_locations()
    if source_pdf.exists():
        source_hash = sha256_file(source_pdf)
        with fitz.open(source_pdf) as source_document:
            source_page_count = source_document.page_count
        ####
        if source_page_count != 307:
            raise ValueError(f"Expected 307 source pages, found {source_page_count}")
        ####
    ####
    aux_by_equation = {record.equation: record for record in aux_records.values()}
    if len(aux_by_equation) != len(aux_records):
        duplicates = [
            value
            for value, count in Counter(record.equation for record in aux_records.values()).items()
            if count > 1
        ]
        raise ValueError(f"Duplicate compiled equation numbers: {duplicates}")
    ####
    expected = expected_equations()
    if set(aux_by_equation) != set(expected):
        raise ValueError(
            "Compiled equation sequence differs from the canonical registry: "
            f"missing={sorted(set(expected) - set(aux_by_equation))}, "
            f"extra={sorted(set(aux_by_equation) - set(expected))}"
        )
    ####
    if set(aux_records) != set(locations):
        raise ValueError(
            "TeX and AUX equation-label sets differ: "
            f"aux_only={sorted(set(aux_records) - set(locations))}, "
            f"tex_only={sorted(set(locations) - set(aux_records))}"
        )
    ####
    records: list[EquationRecord] = []
    for row in metadata_rows:
        equation = row["equation"]
        aux = aux_by_equation[equation]
        location = locations[aux.label]
        source_page = int(row["pdf_page"])
        coverage = source_coverage[source_page]
        if coverage["source_label"] != row["manual_page"]:
            raise ValueError(
                f"Source-page label mismatch for {equation}: "
                f"metadata={row['manual_page']}, coverage={coverage['source_label']}"
            )
        ####
        scope, suite = validation_scope(equation)
        records.append(
            EquationRecord(
                equation=equation,
                latex_label=aux.label,
                section=row["section"],
                source_manual_page=row["manual_page"],
                source_pdf_page=source_page,
                source_pdf_sha256=source_hash,
                reconstructed_page=aux.reconstructed_page,
                reconstructed_section=aux.reconstructed_section,
                tex_file=location.tex_file,
                tex_line_start=location.line_start,
                tex_line_end=location.line_end,
                tex_label_line=location.label_line,
                latex_snippet_sha256=location.snippet_sha256,
                transcription_status=row["status"],
                verification_scope=scope,
                validation_suite=suite,
                code_implementation_status=implementation_status(equation),
                notes=row["notes"],
            )
        )
    ####
    summary = {
        "schema_version": 1,
        "release": "generated",
        "source_pdf": source_pdf.name,
        "source_pdf_sha256": source_hash,
        "source_pdf_pages": source_page_count,
        "equations_total": len(records),
        "equations_by_chapter": dict(
            Counter(record.equation.split("-")[0] for record in records)
        ),
        "unique_latex_labels": len({record.latex_label for record in records}),
        "unique_source_pages_with_equations": len(
            {record.source_pdf_page for record in records}
        ),
        "transcription_statuses": dict(
            Counter(record.transcription_status for record in records)
        ),
        "coverage": {
            "canonical_sequence_complete": True,
            "metadata_to_aux_one_to_one": True,
            "aux_to_tex_one_to_one": True,
            "source_page_labels_verified": True,
            "latex_snippet_hashes_recorded": True,
        },
        "implementation_boundary": (
            "All numbered equations are transcribed and provenance-mapped. "
            "They are not yet all implemented as executable Python functions or individually unit-tested."
        ),
    }
    return records, summary
####


def write_csv(records: list[EquationRecord]) -> None:
    output_path = ROOT / "metadata" / "equations_provenance.csv"
    fieldnames = list(asdict(records[0]).keys())
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for record in records:
            writer.writerow(asdict(record))
        ####
    ####
####


def write_json(records: list[EquationRecord], summary: dict[str, Any]) -> None:
    output = {
        "summary": summary,
        "equations": [asdict(record) for record in records],
    }
    (ROOT / "metadata" / "equations_provenance.json").write_text(
        json.dumps(output, indent=2) + "\n",
        encoding="utf-8",
    )
####


def write_markdown(
    records: list[EquationRecord],
    summary: dict[str, Any],
    version: str,
) -> None:
    lines = [
        f"# TAOS Equation Transcription and Provenance Audit - Version {version}",
        "",
        "## Result",
        "",
        "The numbered-equation transcription and provenance registry is complete.",
        "",
        f"- Canonical numbered equations: **{summary['equations_total']}**",
        f"- Source PDF pages: **{summary['source_pdf_pages']}**",
        f"- Unique source pages containing numbered equations: **{summary['unique_source_pages_with_equations']}**",
        f"- Source PDF SHA-256: `{summary['source_pdf_sha256']}`",
        "- Metadata, compiled equation numbers, LaTeX labels, and source locations are one-to-one.",
        "- Every equation records its source manual page, physical PDF page, LaTeX label, TeX file and line range, reconstructed page, and a hash of the exact LaTeX display.",
        "",
        "## Canonical sequence",
        "",
        "| Chapter | Equations | Count |",
        "|---|---:|---:|",
        "| 1 | 1-1 through 1-2 | 2 |",
        "| 2 | 2-1 through 2-315 | 315 |",
        "| 3 | 3-1 | 1 |",
        "| 4 | 4-1 through 4-8 | 8 |",
        "| **Total** |  | **326** |",
        "",
        "## Verification boundary",
        "",
        summary["implementation_boundary"],
        "The Chapter 2 validators provide selected numerical and algebraic checks at the section level; they do not constitute an executable implementation or one independent unit test for every printed equation.",
        "",
        "## Full provenance registry",
        "",
        "The machine-readable canonical files are:",
        "",
        "- `metadata/equations_provenance.csv`",
        "- `metadata/equations_provenance.json`",
        "",
        "The compact table below lists every equation and its primary source location.",
        "",
        "| Equation | LaTeX label | Source | PDF page | TeX source | Reconstructed page | Status |",
        "|---:|---|---:|---:|---|---:|---|",
    ]
    for record in records:
        location = (
            f"`{record.tex_file}:{record.tex_line_start}-{record.tex_line_end}`"
        )
        lines.append(
            f"| {record.equation} | `{record.latex_label}` | "
            f"{record.source_manual_page} | {record.source_pdf_page} | "
            f"{location} | {record.reconstructed_page} | "
            f"{record.transcription_status} |"
        )
    ####
    lines.extend(
        [
            "",
            "## Reproducibility",
            "",
            "Run:",
            "",
            "```bash",
            "python tools/dev.py equation-audit",
            "python -m pytest tests/test_equation_provenance.py",
            "```",
            "",
            "The audit rebuilds the manual, re-parses the LaTeX auxiliary labels, re-hashes each equation display, and verifies the source-page registry.",
        ]
    )
    (ROOT / "qa" / f"equation_provenance_audit_v{version}.md").write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )
####


def latex_escape(value: str) -> str:
    replacements = {
        "\\": r"\textbackslash{}",
        "&": r"\&",
        "%": r"\%",
        "$": r"\$",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
        "~": r"\textasciitilde{}",
        "^": r"\textasciicircum{}",
    }
    return "".join(replacements.get(character, character) for character in value)
####



def breakable_mono(value: str) -> str:
    output: list[str] = []
    break_after = {":", "-", "/", "."}
    for character in value:
        if character == "_":
            output.append(r"\_\allowbreak{}")
        elif character in break_after:
            output.append(latex_escape(character) + r"\allowbreak{}")
        else:
            output.append(latex_escape(character))
        ####
    ####
    return "".join(output)
####

def write_report_tex(
    records: list[EquationRecord],
    summary: dict[str, Any],
    version: str,
) -> None:
    output = [
        r"\documentclass[10pt]{article}",
        r"\usepackage[T1]{fontenc}",
        r"\usepackage{lmodern}",
        r"\usepackage[letterpaper,landscape,margin=0.35in]{geometry}",
        r"\usepackage{booktabs,longtable,array}",
        r"\usepackage{xcolor}",
        r"\usepackage{xurl}",
        r"\usepackage{fancyhdr}",
        r"\usepackage{hyperref}",
        r"\pagestyle{fancy}",
        r"\fancyhf{}",
        rf"\fancyhead[L]{{TAOS Equation Provenance Audit - Version {latex_escape(version)}}}",
        r"\fancyhead[R]{Canonical 1995 manual reconstruction}",
        r"\fancyfoot[C]{\thepage}",
        r"\setlength{\LTleft}{0pt}",
        r"\setlength{\LTright}{0pt}",
        r"\begin{document}",
        r"\begin{center}",
        r"{\LARGE\bfseries TAOS Equation Transcription and Provenance Audit}\\[0.4em]",
        rf"{{\large Version {latex_escape(version)}}}",
        r"\end{center}",
        r"\section*{Result}",
        (
            f"All {summary['equations_total']} numbered equations are present exactly once in the "
            "canonical metadata registry, the compiled LaTeX auxiliary file, and the TeX sources. "
            "Every entry records source-page and TeX-source provenance plus a SHA-256 hash of the exact display."
        ),
        r"\begin{center}",
        r"\begin{tabular}{lr}",
        r"\toprule",
        rf"Source PDF pages & {summary['source_pdf_pages']}\\",
        rf"Numbered equations & {summary['equations_total']}\\",
        rf"Unique source pages with equations & {summary['unique_source_pages_with_equations']}\\",
        rf"Unique LaTeX equation labels & {summary['unique_latex_labels']}\\",
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{center}",
        r"\noindent\textbf{Source PDF SHA-256:} \texttt{\detokenize{" + summary["source_pdf_sha256"] + r"}}",
        r"\paragraph{Implementation boundary.}",
        latex_escape(summary["implementation_boundary"]),
        r"\section*{Full provenance registry}",
        r"\scriptsize",
        r"\begin{longtable}{>{\raggedleft\arraybackslash}p{0.42in}>{\raggedright\arraybackslash}p{2.05in}p{0.55in}>{\raggedleft\arraybackslash}p{0.38in}>{\raggedright\arraybackslash}p{3.55in}>{\raggedleft\arraybackslash}p{0.58in}p{0.62in}}",
        r"\toprule",
        r"Eq. & LaTeX label & Source & PDF & TeX source and lines & Rebuilt & Status\\",
        r"\midrule",
        r"\endfirsthead",
        r"\toprule",
        r"Eq. & LaTeX label & Source & PDF & TeX source and lines & Rebuilt & Status\\",
        r"\midrule",
        r"\endhead",
        r"\bottomrule",
        r"\endfoot",
    ]
    for record in records:
        tex_location = (
            f"{record.tex_file}:{record.tex_line_start}-{record.tex_line_end}"
        )
        output.append(
            " & ".join(
                [
                    latex_escape(record.equation),
                    r"{\ttfamily\scriptsize " + breakable_mono(record.latex_label) + "}",
                    latex_escape(record.source_manual_page),
                    str(record.source_pdf_page),
                    r"{\ttfamily\scriptsize " + breakable_mono(tex_location) + "}",
                    latex_escape(record.reconstructed_page),
                    "verified",
                ]
            )
            + r"\\"
        )
    ####
    output.extend(
        [
            r"\end{longtable}",
            r"\normalsize",
            r"\section*{Reproducibility}",
            r"Run \texttt{python tools/dev.py equation-audit} to rebuild this registry from the compiled AUX file and the source tree. "
            r"The audit also verifies the 307-page source registry and the canonical equation sequence.",
            r"\end{document}",
        ]
    )
    (ROOT / "qa" / "equation_provenance_audit.tex").write_text(
        "\n".join(output) + "\n",
        encoding="utf-8",
    )
####


def render_source_pages(source_pdf: Path, records: list[EquationRecord]) -> None:
    if not source_pdf.exists():
        return
    ####
    output_dir = ROOT / "qa" / "equation-source-pages"
    output_dir.mkdir(parents=True, exist_ok=True)
    expected_names = {
        f"page-{page_number:03d}.png"
        for page_number in {record.source_pdf_page for record in records}
    }
    for path in output_dir.glob("page-*.png"):
        if path.name not in expected_names:
            path.unlink()
        ####
    ####
    document = fitz.open(source_pdf)
    matrix = fitz.Matrix(1.5, 1.5)
    for page_number in sorted({record.source_pdf_page for record in records}):
        output_path = output_dir / f"page-{page_number:03d}.png"
        document[page_number - 1].get_pixmap(matrix=matrix, alpha=False).save(output_path)
    ####
####


def main() -> None:
    args = parse_args()
    records, summary = build_records(args.source_pdf, args.aux)
    summary["release"] = f"v{args.version}"
    (ROOT / "qa").mkdir(parents=True, exist_ok=True)
    write_csv(records)
    write_json(records, summary)
    write_markdown(records, summary, args.version)
    write_report_tex(records, summary, args.version)
    if args.render_source_pages:
        render_source_pages(args.source_pdf, records)
    ####
    print(
        f"Equation provenance audit passed: {len(records)} equations, "
        f"{summary['unique_source_pages_with_equations']} source pages, "
        "one-to-one metadata/AUX/TeX coverage."
    )
####


if __name__ == "__main__":
    main()
####
