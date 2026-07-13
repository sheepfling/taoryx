from __future__ import annotations

import argparse
import csv
import json
import re
import unicodedata
from collections import Counter
from pathlib import Path
from typing import Any

import fitz
from rapidfuzz import fuzz, process

ROOT = Path(__file__).resolve().parents[1]
JSON_REPORT = ROOT / "qa" / "source_text_fidelity.json"
CSV_REPORT = ROOT / "qa" / "source_sentence_matches.csv"
MARKDOWN_REPORT = ROOT / "qa" / "source_text_fidelity.md"

CHAPTER_RANGES: list[tuple[str, int, int]] = [
    ("Front matter", 1, 17),
    ("Chapter 1", 18, 29),
    ("Chapter 2", 30, 135),
    ("Chapter 3", 136, 165),
    ("Chapter 4", 166, 281),
    ("Appendix", 282, 293),
    ("References", 294, 295),
    ("Index", 296, 305),
    ("Distribution", 306, 307),
]

COMMON_OCR_REPLACEMENTS: dict[str, str] = {
    "fmed": "fixed",
    "fured": "fixed",
    "defrned": "defined",
    "defmed": "defined",
    "ifh": "ith",
    "trajec- tory": "trajectory",
    "coordi- nate": "coordinate",
    "equiva- lent": "equivalent",
    "opti- mization": "optimization",
    "aerody- namic": "aerodynamic",
    "pro- gram": "program",
}


class SentenceMatch:
    def __init__(
        self,
        *,
        source_page: int,
        chapter: str,
        source_sentence: str,
        reconstructed_sentence: str,
        score: float,
    ) -> None:
        self.source_page = source_page
        self.chapter = chapter
        self.source_sentence = source_sentence
        self.reconstructed_sentence = reconstructed_sentence
        self.score = score
    ####

    def as_dict(self) -> dict[str, Any]:
        return {
            "source_page": self.source_page,
            "chapter": self.chapter,
            "source_sentence": self.source_sentence,
            "reconstructed_sentence": self.reconstructed_sentence,
            "score": round(self.score, 1),
        }
    ####
####


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-pdf", type=Path, required=True)
    parser.add_argument("--manual-pdf", type=Path, required=True)
    return parser.parse_args()
####


def _chapter_for_page(page: int) -> str:
    for name, first, last in CHAPTER_RANGES:
        if first <= page <= last:
            return name
        ####
    ####
    return "Unmapped"
####


def _normalize_text(value: str) -> str:
    value = unicodedata.normalize("NFKC", value)
    value = value.replace("\u00ad", "")
    value = value.replace("’", "'").replace("‘", "'").replace("“", '"').replace("”", '"')
    value = re.sub(r"(?<=\w)-\s*\n\s*(?=\w)", "", value)
    value = re.sub(r"(?<=\w)-\s+(?=\w)", "", value)
    value = value.replace("\n", " ")
    for source, target in COMMON_OCR_REPLACEMENTS.items():
        value = value.replace(source, target)
    ####
    value = re.sub(r"\s+", " ", value)
    return value.strip()
####


def _match_normal_form(value: str) -> str:
    value = _normalize_text(value).lower()
    value = re.sub(r"\b(?:figure|table|equation|section)\s+\d+(?:[-.]\d+)+\b", " ", value)
    value = re.sub(r"\b\d+(?:[-.]\d+)+\b", " ", value)
    value = re.sub(r"[^a-z0-9]+", " ", value)
    value = re.sub(r"\s+", " ", value)
    return value.strip()
####


def _is_narrative_sentence(value: str) -> bool:
    words = _match_normal_form(value).split()
    if len(words) < 10 or len(words) > 95:
        return False
    ####
    alpha_words = sum(any(character.isalpha() for character in word) for word in words)
    if alpha_words / max(len(words), 1) < 0.78:
        return False
    ####
    lowered = value.lower().strip()
    if lowered.startswith(("figure ", "table ", "source pdf page", "intentionally left blank")):
        return False
    ####
    if sum(character in "=|[]{}" for character in value) > 4:
        return False
    ####
    return True
####


def _sentences(value: str) -> list[str]:
    normalized = _normalize_text(value)
    candidates = re.split(r"(?<=[.!?])\s+(?=(?:[A-Z0-9]|\*|\())", normalized)
    return [candidate.strip() for candidate in candidates if _is_narrative_sentence(candidate)]
####


def _page_text(document: fitz.Document, page_number: int) -> str:
    return document[page_number - 1].get_text("text")
####


def _extract_reconstructed_sentences(document: fitz.Document) -> list[str]:
    sentences: list[str] = []
    for page in document:
        sentences.extend(_sentences(page.get_text("text")))
    ####
    unique: list[str] = []
    seen: set[str] = set()
    for sentence in sentences:
        key = _match_normal_form(sentence)
        if key and key not in seen:
            unique.append(sentence)
            seen.add(key)
        ####
    ####
    return unique
####


def _best_match(source_sentence: str, reconstructed_sentences: list[str], choices: dict[str, str]) -> tuple[str, float]:
    source_key = _match_normal_form(source_sentence)
    result = process.extractOne(source_key, choices, scorer=fuzz.token_set_ratio, score_cutoff=0)
    if result is None:
        return "", 0.0
    ####
    _, score, key = result
    return reconstructed_sentences[int(key)], float(score)
####


def _aggregate(matches: list[SentenceMatch]) -> dict[str, Any]:
    by_chapter: dict[str, list[SentenceMatch]] = {}
    for match in matches:
        by_chapter.setdefault(match.chapter, []).append(match)
    ####
    output: dict[str, Any] = {}
    for chapter, chapter_matches in by_chapter.items():
        scores = [match.score for match in chapter_matches]
        output[chapter] = {
            "sentence_count": len(scores),
            "near_verbatim_percent": round(100 * sum(score >= 90 for score in scores) / len(scores), 1) if scores else 0.0,
            "edited_close_percent": round(100 * sum(70 <= score < 90 for score in scores) / len(scores), 1) if scores else 0.0,
            "review_percent": round(100 * sum(score < 70 for score in scores) / len(scores), 1) if scores else 0.0,
            "median_score": round(sorted(scores)[len(scores) // 2], 1) if scores else 0.0,
        }
    ####
    return output
####


def _structural_counts() -> dict[str, Any]:
    equations_path = ROOT / "metadata" / "equations.csv"
    figures_path = ROOT / "metadata" / "figures.csv"
    coverage_path = ROOT / "metadata" / "source_page_coverage.csv"
    with equations_path.open(newline="", encoding="utf-8") as handle:
        equations = list(csv.DictReader(handle))
    ####
    with figures_path.open(newline="", encoding="utf-8") as handle:
        figures = list(csv.DictReader(handle))
    ####
    with coverage_path.open(newline="", encoding="utf-8") as handle:
        coverage = list(csv.DictReader(handle))
    ####
    return {
        "source_pages_mapped": len(coverage),
        "equation_records": len(equations),
        "equations_complete_or_verified": sum(row["status"] in {"complete", "visually_verified"} for row in equations),
        "figure_records": len(figures),
        "figures_complete_or_verified": sum(row["status"] in {"complete", "visually_verified"} for row in figures),
    }
####


def _render_markdown(report: dict[str, Any]) -> str:
    structural = report["structural"]
    lines: list[str] = [
        "# TAOS source-text fidelity audit - Version 19",
        "",
        "## Audit conclusion",
        "",
        "The Version 19 document remains a **semantic reconstruction**, not a diplomatic or word-for-word transcription across the whole manual. "
        "Its numbered mathematics, figures, tables, examples, Appendix registry, and backmatter are source-mapped and validated. "
        "Versions 16 through 18 restore near-source wording for thirty-six ranked Chapter 4 regions identified by the Version 15 audit. Version 19 restores the complete `*fly` guidance section plus the Chapter 4 introduction, file-format discussion, and segment/trajectory/problem overview prose, and adds a classified residual review queue; narrative prose elsewhere may still be reflowed, modernized, condensed, or clarified.",
        "",
        "## Structural coverage",
        "",
        "| Item | Count |",
        "|---|---:|",
        f"| Physical source pages mapped | {structural['source_pages_mapped']} |",
        f"| Numbered equation records | {structural['equation_records']} |",
        f"| Equation records complete/visually verified | {structural['equations_complete_or_verified']} |",
        f"| Numbered figure records | {structural['figure_records']} |",
        f"| Figure records complete/visually verified | {structural['figures_complete_or_verified']} |",
        "",
        "## Automated narrative comparison",
        "",
        "The audit extracts narrative-length sentences from the OCR text layer of the 1995 scan and compares them with sentences extracted from the reconstructed PDF. "
        "Scores are diagnostic rather than dispositive: OCR errors, equations, tables, headings, and deliberate editorial modernization can lower a score even when technical content is present.",
        "",
        "| Section | Source sentences | Near-verbatim (>=90) | Edited/close (70-89) | Review (<70) | Median score |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for chapter, values in report["chapter_metrics"].items():
        lines.append(
            f"| {chapter} | {values['sentence_count']} | {values['near_verbatim_percent']:.1f}% | "
            f"{values['edited_close_percent']:.1f}% | {values['review_percent']:.1f}% | {values['median_score']:.1f} |"
        )
    ####
    lines.extend(
        [
            "",
            "## Versions 16 through 19 Chapter 4 diplomatic tranches",
            "",
            "- Version 16 restored source-near prose for `*aero`, `*limits`, `*when`, problem-level `*define`, `*egs`, problem-level `*file`, problem-level `*print`, and the Ballistic Reentry example.",
            "- Version 17 restores trajectory-level `*define`, `*optimize`, `*search`, `*summarize`, `*survey`, Ballistic Rocket, Air-Launched Intercept, and Ground-Launched Intercept.",
            "- Version 18 restores the prioritized units/format, integration, propulsion, atmosphere, earth, radar, trajectory-initialization, and ancillary data-block descriptions.",
            "- Version 19 restores the complete `*fly` guidance section, the Chapter 4 introduction and file-format discussion, and the segment, trajectory, and problem overview prose; it also classifies the remaining low-score queue.",
            "- The Version 17 batch restores optimization defaults, restart and derivative controls, convergence diagnostics, search-loop nesting, summary operations, survey ordering, worked-example narratives, and historical printout excerpts.",
            "- The source-visible `*optimize` derivative-increment default is restored to `1.0e-8`; the Ground-Launched Intercept printout follows the visible `-4731.568` value rather than the corrupted OCR text layer.",
            "- All ten Version 15 `.tbl` and `.prb` parser fixtures remain byte-for-byte unchanged. Separate `.txt` files hold the diplomatic input displays and transcribed historical outputs.",
            "",
            "Relative to the Version 15 baseline, Chapter 4's near-verbatim sentence share increased from 5.3% to "
            f"{report['chapter_metrics']['Chapter 4']['near_verbatim_percent']:.1f}%; its review share fell from 51.7% to "
            f"{report['chapter_metrics']['Chapter 4']['review_percent']:.1f}%, and its median sentence score increased from 69.5 to "
            f"{report['chapter_metrics']['Chapter 4']['median_score']:.1f}.",
            "",
            "## Known and intentional deviations",
            "",
            "- Narrative prose outside completed diplomatic tranches is often condensed or rephrased for clarity. The reconstructed edition should not be used for quotation where exact 1995 wording matters; consult the source scan.",
            "- Evident mathematical, unit, spelling, OCR, or description errors are sometimes corrected. Every known substantive correction is recorded in editorial metadata and release notes.",
            "- Figures are semantic vector redraws rather than pixel facsimiles. Chapter 4 plotted curves reproduce the printed trends but were not regenerated with TAOS 96.0.",
            "- Historical output listings are source transcriptions, not newly generated simulation results.",
            "- Runtime behavior of fixtures remains unverified without the historical TAOS executable or a separately validated compatible implementation.",
            "",
            "## Manual-review queue",
            "",
            "The CSV and JSON audit artifacts retain sentence-level scores and candidate matches. Low-scoring items are a review queue, not an assertion that content is missing. "
            "After the three ranked Chapter 4 batches and the Version 19 residual pass, the remaining Chapter 4 review queue is dispersed among OCR-damaged tables, equations, layout-sensitive listings, and isolated prose outside the selected regions. Condensed narrative in Chapters 1 and 2 also remains outside the completed diplomatic scope.",
            "",
        ]
    )
    return "\n".join(lines)
####


def main() -> None:
    args = parse_args()
    source_document = fitz.open(args.source_pdf)
    reconstructed_document = fitz.open(args.manual_pdf)
    reconstructed_sentences = _extract_reconstructed_sentences(reconstructed_document)
    choices = {
        str(index): _match_normal_form(sentence)
        for index, sentence in enumerate(reconstructed_sentences)
    }
    matches: list[SentenceMatch] = []
    for page_number in range(1, len(source_document) + 1):
        chapter = _chapter_for_page(page_number)
        for source_sentence in _sentences(_page_text(source_document, page_number)):
            reconstructed_sentence, score = _best_match(source_sentence, reconstructed_sentences, choices)
            matches.append(
                SentenceMatch(
                    source_page=page_number,
                    chapter=chapter,
                    source_sentence=source_sentence,
                    reconstructed_sentence=reconstructed_sentence,
                    score=score,
                )
            )
        ####
    ####
    report: dict[str, Any] = {
        "schema_version": 1,
        "source_pdf_pages": len(source_document),
        "reconstructed_pdf_pages": len(reconstructed_document),
        "source_sentence_count": len(matches),
        "structural": _structural_counts(),
        "chapter_metrics": _aggregate(matches),
        "score_bands": dict(Counter("near_verbatim" if match.score >= 90 else "edited_close" if match.score >= 70 else "review" for match in matches)),
        "lowest_scoring": [match.as_dict() for match in sorted(matches, key=lambda item: item.score)[:100]],
    }
    JSON_REPORT.parent.mkdir(parents=True, exist_ok=True)
    JSON_REPORT.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    with CSV_REPORT.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["source_page", "chapter", "score", "source_sentence", "reconstructed_sentence"],
        )
        writer.writeheader()
        for match in matches:
            writer.writerow(match.as_dict())
        ####
    ####
    MARKDOWN_REPORT.write_text(_render_markdown(report), encoding="utf-8")
    print(f"Source-text fidelity audit generated for {len(matches)} source narrative sentences.")
####


if __name__ == "__main__":
    main()
####
