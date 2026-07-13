from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
METADATA_PATH = ROOT / "metadata" / "diplomatic_transcription_chapter4_v19.yaml"
AUDIT_PATH = ROOT / "qa" / "chapter04_residual_audit_v19.json"
EXPECTED_SECTION_COUNT = 6
EXPECTED_FIGURES = {"1-5", "1-6", "2-1", "4-2"}
REQUIRED_PHRASES: dict[str, tuple[str, ...]] = {
    "manual/chapters/chapter04/00_introduction.tex": (
        "Problem files have the hierarchical structure",
        "Each trajectory in a problem is identified",
        "This set of information",
    ),
    "manual/chapters/chapter04/01_file_format.tex": (
        "Survey, Search, and Optimization Parameters",
        "Surveys, searches, and optimization loops all",
        "Comments begin with the special character",
    ),
    "manual/chapters/chapter04/02_segment_overview.tex": (
        "Segment data blocks provide the information required",
        "A segment block extends from its",
        "The following segment is an example",
    ),
    "manual/chapters/chapter04/02_04_fly.tex": (
        "Body-Attitude Guidance",
        "Flight-Condition Guidance",
        "Special Guidance Rules",
        "Guidance Tables",
    ),
    "manual/chapters/chapter04/03_trajectory_overview.tex": (
        "Trajectory data blocks contain information",
        "Every trajectory definition must contain",
        "The following outline illustrates",
    ),
    "manual/chapters/chapter04/04_problem_overview.tex": (
        "It is important to group data blocks",
        "A problem begins with an identifier",
        "the problem ends with",
    ),
}


def _load_yaml(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise SystemExit(f"Expected mapping in {path.relative_to(ROOT)}")
    ####
    return payload
####


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()
####


def _check_phrases() -> None:
    for relative_path, phrases in REQUIRED_PHRASES.items():
        text = (ROOT / relative_path).read_text(encoding="utf-8")
        for phrase in phrases:
            if phrase not in text:
                raise SystemExit(f"Missing required Version 19 phrase in {relative_path}: {phrase}")
            ####
        ####
    ####
####


def _check_figures(metadata: dict[str, Any]) -> None:
    figure_rows = metadata.get("figure_review", {}).get("figures", [])
    if not isinstance(figure_rows, list):
        raise SystemExit("Version 19 figure review is missing.")
    ####
    identifiers = {str(row.get("figure")) for row in figure_rows if isinstance(row, dict)}
    if identifiers != EXPECTED_FIGURES:
        raise SystemExit(f"Unexpected Version 19 figure set: {sorted(identifiers)}")
    ####
    for row in figure_rows:
        if not isinstance(row, dict):
            raise SystemExit("Malformed Version 19 figure row.")
        ####
        for key in ("source_file", "reconstruction_file"):
            path = ROOT / str(row[key])
            if not path.exists():
                raise SystemExit(f"Missing Version 19 figure asset: {path.relative_to(ROOT)}")
            ####
        ####
    ####
    fig42 = (ROOT / "manual/figures/chapter04/fig-4-2.tex").read_text(encoding="utf-8")
    if fig42.count("\\begin{tikzpicture}") != 1 or fig42.count("\\end{tikzpicture}") != 1:
        raise SystemExit("Figure 4-2 must contain exactly one TikZ picture.")
    ####
####


def _check_guidance_keyword() -> None:
    fly_text = (ROOT / "manual/chapters/chapter04/02_04_fly.tex").read_text(encoding="utf-8")
    guidance_text = (ROOT / "metadata/guidance_rules_chapter4.yaml").read_text(encoding="utf-8")
    if "l/d_max" in fly_text or "l/d_max" in guidance_text:
        raise SystemExit("Obsolete l/d_max spelling remains; the source keyword is l/d-max.")
    ####
    if "l/d-max" not in fly_text or "l/d-max" not in guidance_text:
        raise SystemExit("Canonical l/d-max keyword is missing.")
    ####
####


def _check_fixture_invariance(metadata: dict[str, Any]) -> int:
    fixture_invariance = metadata.get("fixture_invariance")
    expected = fixture_invariance.get("sha256") if isinstance(fixture_invariance, dict) else None
    if not isinstance(expected, dict):
        raise SystemExit("Version 19 fixture-invariance hashes are missing.")
    ####
    for relative_path, expected_hash in expected.items():
        path = ROOT / str(relative_path)
        if not path.exists():
            raise SystemExit(f"Missing fixture: {relative_path}")
        ####
        actual_hash = _sha256(path)
        if actual_hash != str(expected_hash):
            raise SystemExit(f"Parser fixture changed since Version 15: {relative_path}")
        ####
    ####
    return len(expected)
####

def _check_audit(metadata: dict[str, Any]) -> None:
    if not AUDIT_PATH.exists():
        raise SystemExit("Run tools/audit_chapter04_residual_v19.py before validation.")
    ####
    report = json.loads(AUDIT_PATH.read_text(encoding="utf-8"))
    rows = report.get("sections")
    if not isinstance(rows, list) or len(rows) != EXPECTED_SECTION_COUNT:
        raise SystemExit("Version 19 audit section count is incorrect.")
    ####
    for row in rows:
        baseline = float(row["baseline_metrics"]["token_recall_percent"])
        current = float(row["current_metrics"]["token_recall_percent"])
        if current <= baseline:
            raise SystemExit(f"Version 19 token recall did not improve for {row['section']}")
        ####
    ####
    metadata_ids = {str(row["id"]) for row in metadata["sections"]}
    audit_ids = {str(row["id"]) for row in rows}
    if metadata_ids != audit_ids:
        raise SystemExit("Version 19 audit and metadata section identifiers differ.")
    ####
    residual = report.get("residual_queue")
    if not isinstance(residual, dict):
        raise SystemExit("Version 19 residual queue is missing.")
    ####
    counts = residual.get("classification_counts")
    if not isinstance(counts, dict) or sum(int(value) for value in counts.values()) != int(residual.get("low_sentence_count", -1)):
        raise SystemExit("Version 19 residual-queue category counts are inconsistent.")
    ####
    expected_categories = {
        "restored_fly_region",
        "table_equation_or_listing",
        "ocr_damaged",
        "layout_sensitive",
        "remaining_prose_review",
    }
    if set(counts) != expected_categories:
        raise SystemExit(f"Unexpected residual-queue categories: {sorted(counts)}")
    ####
####


def main() -> None:
    metadata = _load_yaml(METADATA_PATH)
    sections = metadata.get("sections")
    if not isinstance(sections, list) or len(sections) != EXPECTED_SECTION_COUNT:
        raise SystemExit(f"Expected {EXPECTED_SECTION_COUNT} Version 19 diplomatic sections.")
    ####
    _check_phrases()
    _check_figures(metadata)
    _check_guidance_keyword()
    fixture_count = _check_fixture_invariance(metadata)
    _check_audit(metadata)
    print(
        "Chapter 4 Version 19 residual-fidelity checks passed: "
        f"{len(sections)} sections improved, four figures reviewed, and "
        f"{fixture_count} parser fixtures unchanged."
    )
####


if __name__ == "__main__":
    main()
####
