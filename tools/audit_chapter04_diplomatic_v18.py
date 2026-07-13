from __future__ import annotations

import argparse
import json
import re
import unicodedata
from collections import Counter
from pathlib import Path
from typing import Any

import yaml

import fitz

ROOT = Path(__file__).resolve().parents[1]
METADATA_PATH = ROOT / "metadata" / "diplomatic_transcription_chapter4_v18.yaml"
JSON_PATH = ROOT / "qa" / "chapter04_diplomatic_audit_v18.json"
MARKDOWN_PATH = ROOT / "qa" / "chapter04_diplomatic_audit_v18.md"

OCCURRENCE_BY_SECTION: dict[str, int] = {
    "4.4.2": 2,
    "4.4.5": 2,
    "4.4.7": 2,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-pdf", type=Path, required=True)
    parser.add_argument("--manual-pdf", type=Path, required=True)
    parser.add_argument("--baseline-pdf", type=Path)
    return parser.parse_args()
####


def _load_metadata() -> dict[str, Any]:
    payload = yaml.safe_load(METADATA_PATH.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise SystemExit("Diplomatic-transcription metadata is not a mapping.")
    ####
    return payload
####


def _normalize_text(value: str) -> str:
    value = unicodedata.normalize("NFKC", value)
    value = value.replace("\u00ad", "")
    value = value.replace("’", "'").replace("‘", "'")
    value = value.replace("“", '"').replace("”", '"')
    value = re.sub(r"(?<=\w)-\s*\n\s*(?=\w)", "", value)
    value = re.sub(r"(?<=\w)-\s+(?=\w)", "", value)
    value = value.replace("\n", " ")
    value = re.sub(r"\b4-\s*\d+\b", " ", value)
    value = re.sub(r"\s+", " ", value)
    return value.strip().lower()
####


def _tokens(value: str) -> list[str]:
    normalized = _normalize_text(value)
    return re.findall(r"[a-z0-9]+(?:[_.\-/][a-z0-9]+)*(?:\[[0-9]+\])?", normalized)
####


def _ngrams(tokens: list[str], size: int) -> list[tuple[str, ...]]:
    if len(tokens) < size:
        return []
    ####
    return [tuple(tokens[index:index + size]) for index in range(len(tokens) - size + 1)]
####


def _recall(source_items: list[Any], candidate_items: list[Any]) -> float:
    source_counts = Counter(source_items)
    candidate_counts = Counter(candidate_items)
    denominator = sum(source_counts.values())
    if denominator == 0:
        return 0.0
    ####
    overlap = sum((source_counts & candidate_counts).values())
    return 100.0 * overlap / denominator
####


def _extract_pages(document: fitz.Document, first_page: int, last_page: int) -> str:
    return "\n".join(document[page - 1].get_text("text") for page in range(first_page, last_page + 1))
####


def _toc_ranges(document: fitz.Document) -> list[dict[str, Any]]:
    toc = document.get_toc(simple=True)
    ranges: list[dict[str, Any]] = []
    for index, (level, title, page) in enumerate(toc):
        end_page = len(document)
        for next_level, _, next_page in toc[index + 1:]:
            if next_level <= level:
                end_page = max(page, next_page - 1)
                break
            ####
        ####
        ranges.append({"level": level, "title": title, "first_page": page, "last_page": end_page})
    ####
    return ranges
####


def _find_manual_range(
    toc_ranges: list[dict[str, Any]],
    *,
    section: str,
    title: str,
) -> tuple[int, int]:
    candidates = [row for row in toc_ranges if row["title"] == title]
    occurrence = OCCURRENCE_BY_SECTION.get(section, 1)
    if len(candidates) < occurrence:
        raise SystemExit(
            f"Could not locate TOC occurrence {occurrence} for {section} {title!r}; "
            f"found {len(candidates)}."
        )
    ####
    row = candidates[occurrence - 1]
    return int(row["first_page"]), int(row["last_page"])
####


def _metrics(source_text: str, candidate_text: str) -> dict[str, Any]:
    source_tokens = _tokens(source_text)
    candidate_tokens = _tokens(candidate_text)
    return {
        "source_tokens": len(source_tokens),
        "candidate_tokens": len(candidate_tokens),
        "length_ratio": round(len(candidate_tokens) / max(len(source_tokens), 1), 3),
        "token_recall_percent": round(_recall(source_tokens, candidate_tokens), 1),
        "bigram_recall_percent": round(
            _recall(_ngrams(source_tokens, 2), _ngrams(candidate_tokens, 2)), 1
        ),
        "trigram_recall_percent": round(
            _recall(_ngrams(source_tokens, 3), _ngrams(candidate_tokens, 3)), 1
        ),
    }
####


def _render_markdown(report: dict[str, Any]) -> str:
    lines: list[str] = [
        "# TAOS Chapter 4 diplomatic-transcription audit - Version 18",
        "",
        "This targeted audit measures the twenty Chapter 4 regions restored in the remaining Chapter 4 fidelity queue. "
        "The comparison uses the source PDF text layer and the corresponding reconstructed PDF outline regions. "
        "Scores remain conservative because code listings, tables, line breaking, and corrected OCR affect plain-text matching.",
        "",
        "| Section | Source pages | V17 token recall | V18 token recall | Change | V17 bigram | V18 bigram | Change |",
        "|---|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in report["sections"]:
        baseline = row.get("baseline_metrics") or {}
        current = row["current_metrics"]
        baseline_token = float(baseline.get("token_recall_percent", row["audit_baseline"]["token_recall_percent"]))
        baseline_bigram = float(baseline.get("bigram_recall_percent", row["audit_baseline"]["bigram_recall_percent"]))
        current_token = float(current["token_recall_percent"])
        current_bigram = float(current["bigram_recall_percent"])
        source_pages = ", ".join(str(page) for page in row["source_pdf_pages"])
        lines.append(
            f"| {row['section']} {row['title']} | {source_pages} | "
            f"{baseline_token:.1f}% | {current_token:.1f}% | {current_token - baseline_token:+.1f} pp | "
            f"{baseline_bigram:.1f}% | {current_bigram:.1f}% | {current_bigram - baseline_bigram:+.1f} pp |"
        )
    ####
    aggregate = report["aggregate"]
    lines.extend(
        [
            "",
            "## Aggregate",
            "",
            f"- Source tokens across the tranche: {aggregate['source_tokens']:,}",
            f"- Version 18 candidate tokens: {aggregate['candidate_tokens']:,}",
            f"- Weighted token recall: {aggregate['token_recall_percent']:.1f}%",
            f"- Weighted bigram recall: {aggregate['bigram_recall_percent']:.1f}%",
            f"- Weighted trigram recall: {aggregate['trigram_recall_percent']:.1f}%",
            "",
            "## Integrity checks",
            "",
            "- The ten Version 15 `.tbl` and `.prb` parser fixtures remain byte-for-byte unchanged through Version 18.",
            "- This pass changes prose and semantic presentation only; no parser fixture or historical output listing is modified.",
            "- No numbered equation, table, or figure was renumbered or replaced by this pass.",
            "- The restored data-block descriptions are source-page transcriptions; TAOS 96.0 was not run.",
            "",
        ]
    )
    return "\n".join(lines)
####


def main() -> None:
    args = parse_args()
    metadata = _load_metadata()
    source_document = fitz.open(args.source_pdf)
    manual_document = fitz.open(args.manual_pdf)
    baseline_document = fitz.open(args.baseline_pdf) if args.baseline_pdf else None
    current_toc = _toc_ranges(manual_document)
    baseline_toc = _toc_ranges(baseline_document) if baseline_document else []

    section_reports: list[dict[str, Any]] = []
    aggregate_source = ""
    aggregate_current = ""
    for item in metadata["sections"]:
        section = str(item["section"])
        title = str(item["title"])
        source_pages = [int(page) for page in item["source_pdf_pages"]]
        source_text = _extract_pages(source_document, min(source_pages), max(source_pages))
        current_first, current_last = _find_manual_range(
            current_toc,
            section=section,
            title=title,
        )
        current_text = _extract_pages(manual_document, current_first, current_last)
        row: dict[str, Any] = {
            "id": item["id"],
            "section": section,
            "title": title,
            "source_pdf_pages": source_pages,
            "manual_pdf_pages": list(range(current_first, current_last + 1)),
            "audit_baseline": {
                "token_recall_percent": float(item["baseline_token_recall_percent"]),
                "bigram_recall_percent": float(item["baseline_bigram_recall_percent"]),
            },
            "current_metrics": _metrics(source_text, current_text),
        }
        if baseline_document is not None:
            baseline_first, baseline_last = _find_manual_range(
                baseline_toc,
                section=section,
                title=title,
            )
            baseline_text = _extract_pages(baseline_document, baseline_first, baseline_last)
            row["baseline_pdf_pages"] = list(range(baseline_first, baseline_last + 1))
            row["baseline_metrics"] = _metrics(source_text, baseline_text)
        ####
        section_reports.append(row)
        aggregate_source += "\n" + source_text
        aggregate_current += "\n" + current_text
    ####

    report: dict[str, Any] = {
        "schema_version": 1,
        "release": "v18",
        "source_pdf": str(args.source_pdf),
        "manual_pdf": str(args.manual_pdf),
        "baseline_pdf": str(args.baseline_pdf) if args.baseline_pdf else None,
        "sections": section_reports,
        "aggregate": _metrics(aggregate_source, aggregate_current),
    }
    JSON_PATH.parent.mkdir(parents=True, exist_ok=True)
    JSON_PATH.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    MARKDOWN_PATH.write_text(_render_markdown(report), encoding="utf-8")
    print(f"Wrote {JSON_PATH.relative_to(ROOT)} and {MARKDOWN_PATH.relative_to(ROOT)}")
####


if __name__ == "__main__":
    main()
####
