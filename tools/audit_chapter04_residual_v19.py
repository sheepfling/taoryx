from __future__ import annotations

import argparse
import csv
import json
import re
import subprocess
import unicodedata
from collections import Counter
from pathlib import Path
from typing import Any

import yaml

import fitz

ROOT = Path(__file__).resolve().parents[1]
METADATA_PATH = ROOT / "metadata" / "diplomatic_transcription_chapter4_v19.yaml"
JSON_PATH = ROOT / "qa" / "chapter04_residual_audit_v19.json"
MARKDOWN_PATH = ROOT / "qa" / "chapter04_residual_audit_v19.md"
SENTENCE_MATCHES_PATH = ROOT / "qa" / "source_sentence_matches.csv"
FIDELITY_PATH = ROOT / "qa" / "source_text_fidelity.json"
BASELINE_FIDELITY_PATH = ROOT / "qa" / "source_text_fidelity_v18.json"
LOW_SCORE_THRESHOLD = 70.0
FLY_SOURCE_PAGES = set(range(178, 185))
LAYOUT_SENSITIVE_PAGES = {
    166,
    167,
    168,
    172,
    173,
    178,
    179,
    180,
    181,
    182,
    183,
    184,
    198,
    199,
    202,
    203,
    207,
    209,
    210,
    211,
    212,
    214,
    218,
    220,
    222,
    224,
    225,
    226,
    228,
    229,
    230,
    231,
    232,
    233,
    234,
    235,
    236,
    237,
    238,
    239,
    240,
    241,
    242,
    243,
    244,
    245,
    246,
    247,
    248,
    249,
    250,
    251,
    252,
    253,
    254,
    255,
    257,
    258,
    259,
    260,
    261,
    262,
    263,
    264,
    265,
    266,
    267,
    268,
    269,
    270,
    271,
    272,
    273,
    274,
    275,
    276,
    277,
    278,
    279,
    280,
}
OCR_DAMAGE_PATTERNS = (
    r"\ufffe",
    r"opera€ed",
    r"\b(?:frrst|defrned|defmed|fmed|fured|inpul|trajecoty|trajectoty|"
    r"dowment|itlegible|hags|kilomew|grams?mour|alphu|gawgd|mch|"
    r"teh|tbe|tbis|wsec|degreelsecond)\b",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-pdf", type=Path, required=True)
    return parser.parse_args()
####


def _load_metadata() -> dict[str, Any]:
    payload = yaml.safe_load(METADATA_PATH.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise SystemExit("Version 19 diplomatic metadata is not a mapping.")
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
    value = re.sub(r"\b(?:sec|fig|tab|eq):[a-z0-9_.:-]+\b", " ", value, flags=re.I)
    value = re.sub(
        r"\b(?:figure|table|lstlisting|tabularx|longtable|toprule|midrule|bottomrule|"
        r"endfirsthead|endhead|htbp)\b",
        " ",
        value,
        flags=re.I,
    )
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


def _extract_source_pages(document: fitz.Document, pages: list[int]) -> str:
    return "\n".join(document[page - 1].get_text("text") for page in pages)
####


def _extract_tex(relative_path: str) -> str:
    path = ROOT / relative_path
    process = subprocess.run(
        ["detex", str(path)],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=True,
    )
    return process.stdout
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


def _looks_ocr_damaged(sentence: str) -> bool:
    lowered = sentence.lower()
    if any(re.search(pattern, lowered, flags=re.I) for pattern in OCR_DAMAGE_PATTERNS):
        return True
    ####
    unusual = sum(
        not (character.isascii() or character in "°–—’‘“”×·")
        for character in sentence
    )
    return unusual >= 2
####


def _looks_table_equation_or_listing(sentence: str) -> bool:
    numeric_tokens = re.findall(r"(?<![A-Za-z])[-+]?\d+(?:\.\d+)?(?:e[-+]?\d+)?", sentence, flags=re.I)
    syntax_marks = sum(sentence.count(mark) for mark in ("*", "=", "<", ">", "[", "]", "_"))
    lowered = sentence.lower()
    heading_run = bool(
        re.match(
            r"^(?:\*?[a-z0-9_/.-]+\s+){2,}(?:data block|variable|units?|description|example)",
            lowered,
        )
    )
    return (
        syntax_marks >= 2
        or len(numeric_tokens) >= 5
        or heading_run
        or " data block " in f" {lowered} " and len(sentence.split()) > 55
        or lowered.startswith(("table ", "figure ", "equation "))
    )
####


def _looks_layout_sensitive(page: int, sentence: str) -> bool:
    word_count = len(sentence.split())
    return page in LAYOUT_SENSITIVE_PAGES and (
        word_count >= 45
        or sentence.count("  ") >= 2
        or bool(re.search(r"\b(?:printout|listing|flowchart|table|figure|data block)\b", sentence, flags=re.I))
    )
####


def _classify_low_sentence(row: dict[str, str]) -> str:
    page = int(row["source_page"])
    sentence = row["source_sentence"]
    if page in FLY_SOURCE_PAGES:
        return "restored_fly_region"
    ####
    if _looks_ocr_damaged(sentence):
        return "ocr_damaged"
    ####
    if _looks_table_equation_or_listing(sentence):
        return "table_equation_or_listing"
    ####
    if _looks_layout_sensitive(page, sentence):
        return "layout_sensitive"
    ####
    return "remaining_prose_review"
####


def _load_chapter4_residual_queue() -> dict[str, Any]:
    if not SENTENCE_MATCHES_PATH.exists():
        raise SystemExit("Run tools/audit_source_fidelity.py before the Version 19 residual audit.")
    ####
    with SENTENCE_MATCHES_PATH.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    ####
    low_rows: list[dict[str, Any]] = []
    for row in rows:
        if row.get("chapter") != "Chapter 4" or float(row["score"]) >= LOW_SCORE_THRESHOLD:
            continue
        ####
        enriched: dict[str, Any] = {
            "source_page": int(row["source_page"]),
            "score": float(row["score"]),
            "source_sentence": row["source_sentence"],
            "reconstructed_sentence": row["reconstructed_sentence"],
        }
        enriched["classification"] = _classify_low_sentence(row)
        low_rows.append(enriched)
    ####
    category_counts = Counter(str(row["classification"]) for row in low_rows)
    examples: dict[str, list[dict[str, Any]]] = {}
    for category in sorted(category_counts):
        category_rows = [row for row in low_rows if row["classification"] == category]
        category_rows.sort(key=lambda row: (float(row["score"]), int(row["source_page"])))
        examples[category] = category_rows[:8]
    ####
    current_metrics: dict[str, Any] = {}
    baseline_metrics: dict[str, Any] = {}
    if FIDELITY_PATH.exists():
        current_metrics = json.loads(FIDELITY_PATH.read_text(encoding="utf-8")).get("chapter_metrics", {}).get("Chapter 4", {})
    ####
    if BASELINE_FIDELITY_PATH.exists():
        baseline_metrics = json.loads(BASELINE_FIDELITY_PATH.read_text(encoding="utf-8")).get("chapter_metrics", {}).get("Chapter 4", {})
    ####
    return {
        "threshold": LOW_SCORE_THRESHOLD,
        "low_sentence_count": len(low_rows),
        "classification_counts": dict(sorted(category_counts.items())),
        "examples": examples,
        "baseline_chapter4_metrics": baseline_metrics,
        "current_chapter4_metrics": current_metrics,
    }
####


def _render_markdown(report: dict[str, Any]) -> str:
    lines: list[str] = [
        "# TAOS Chapter 4 residual fidelity and figure review - Version 19",
        "",
        "This audit covers the six residual Chapter 4 prose regions restored after the three ranked diplomatic batches. "
        "It compares the source PDF text layer with the semantic LaTeX source, so scores remain conservative where "
        "tables, listings, equations, cross-references, and OCR damage affect plain-text matching.",
        "",
        "| Section | Source pages | V18 token | V19 token | Gain | V18 bigram | V19 bigram | Gain |",
        "|---|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in report["sections"]:
        baseline = row["baseline_metrics"]
        current = row["current_metrics"]
        source_pages = ", ".join(str(page) for page in row["source_pdf_pages"])
        token_gain = current["token_recall_percent"] - baseline["token_recall_percent"]
        bigram_gain = current["bigram_recall_percent"] - baseline["bigram_recall_percent"]
        lines.append(
            f"| {row['section']} {row['title']} | {source_pages} | "
            f"{baseline['token_recall_percent']:.1f}% | {current['token_recall_percent']:.1f}% | {token_gain:+.1f} pp | "
            f"{baseline['bigram_recall_percent']:.1f}% | {current['bigram_recall_percent']:.1f}% | {bigram_gain:+.1f} pp |"
        )
    ####
    aggregate = report["aggregate"]
    lines.extend(
        [
            "",
            "## Aggregate",
            "",
            f"- Source tokens across the tranche: {aggregate['source_tokens']:,}",
            f"- Version 19 candidate tokens: {aggregate['candidate_tokens']:,}",
            f"- Weighted token recall: {aggregate['token_recall_percent']:.1f}%",
            f"- Weighted bigram recall: {aggregate['bigram_recall_percent']:.1f}%",
            f"- Weighted trigram recall: {aggregate['trigram_recall_percent']:.1f}%",
            "",
            "## Classified residual Chapter 4 queue",
            "",
        ]
    )
    residual = report["residual_queue"]
    baseline = residual.get("baseline_chapter4_metrics", {})
    current = residual.get("current_chapter4_metrics", {})
    if baseline and current:
        lines.extend(
            [
                f"- Version 18 Chapter 4 review share: {float(baseline.get('review_percent', 0.0)):.1f}%",
                f"- Version 19 Chapter 4 review share: {float(current.get('review_percent', 0.0)):.1f}%",
                f"- Version 18 Chapter 4 median sentence score: {float(baseline.get('median_score', 0.0)):.1f}",
                f"- Version 19 Chapter 4 median sentence score: {float(current.get('median_score', 0.0)):.1f}",
            ]
        )
    ####
    lines.append(f"- Sentences below the diagnostic score threshold of {residual['threshold']:.0f}: {residual['low_sentence_count']}")
    lines.extend(["", "| Classification | Count | Interpretation |", "|---|---:|---|"])
    interpretations = {
        "restored_fly_region": "The source span was restored in Version 19; remaining low scores are largely caused by tables, examples, symbols, and reflow.",
        "table_equation_or_listing": "The OCR sentence merges a table, equation, or input/output listing with prose; plain-sentence matching is not authoritative.",
        "ocr_damaged": "The source text layer contains obvious OCR corruption or non-text artifacts.",
        "layout_sensitive": "The sentence crosses a figure, table, listing, heading, or page-layout boundary.",
        "remaining_prose_review": "A coherent narrative sentence remains a candidate for a future diplomatic pass.",
    }
    for category, count in residual["classification_counts"].items():
        lines.append(f"| `{category}` | {count} | {interpretations[category]} |")
    ####
    lines.extend(
        [
            "",
            "The classifier is a triage tool, not a proof of completeness. The machine-readable JSON retains representative low-score examples for each category so that future work can focus on coherent prose rather than repeatedly editing OCR-damaged tables and listings.",
            "",
            "## Figure review",
            "",
        ]
    )
    for figure in report["figure_review"]:
        lines.append(
            f"- **Figure {figure['figure']} - {figure['title']}:** {figure['action']}"
        )
    ####
    lines.extend(
        [
            "",
            "## Integrity statements",
            "",
            "- Existing `.tbl` and `.prb` parser fixtures are unchanged.",
            "- No numbered equation, table, or figure is renumbered by this pass.",
            "- The reviewed figures remain editable semantic LaTeX/TikZ rather than embedded raster artwork.",
            "- The historical TAOS 96.0 executable was not run.",
            "",
        ]
    )
    return "\n".join(lines)
####


def main() -> None:
    args = parse_args()
    metadata = _load_metadata()
    source_document = fitz.open(args.source_pdf)
    rows: list[dict[str, Any]] = []
    aggregate_source = ""
    aggregate_candidate = ""
    for item in metadata["sections"]:
        pages = [int(page) for page in item["source_pdf_pages"]]
        source_text = _extract_source_pages(source_document, pages)
        candidate_text = _extract_tex(str(item["tex_file"]))
        current = _metrics(source_text, candidate_text)
        baseline = {
            "token_recall_percent": float(item["baseline_token_recall_percent"]),
            "bigram_recall_percent": float(item["baseline_bigram_recall_percent"]),
        }
        rows.append(
            {
                "id": item["id"],
                "section": str(item["section"]),
                "title": str(item["title"]),
                "tex_file": str(item["tex_file"]),
                "source_pdf_pages": pages,
                "baseline_metrics": baseline,
                "current_metrics": current,
            }
        )
        aggregate_source += "\n" + source_text
        aggregate_candidate += "\n" + candidate_text
    ####
    report: dict[str, Any] = {
        "schema_version": 2,
        "release": "v19",
        "source_pdf": str(args.source_pdf),
        "sections": rows,
        "aggregate": _metrics(aggregate_source, aggregate_candidate),
        "residual_queue": _load_chapter4_residual_queue(),
        "figure_review": metadata["figure_review"]["figures"],
    }
    JSON_PATH.parent.mkdir(parents=True, exist_ok=True)
    JSON_PATH.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    MARKDOWN_PATH.write_text(_render_markdown(report), encoding="utf-8")
    print(f"Wrote {JSON_PATH.relative_to(ROOT)} and {MARKDOWN_PATH.relative_to(ROOT)}")
####


if __name__ == "__main__":
    main()
####
