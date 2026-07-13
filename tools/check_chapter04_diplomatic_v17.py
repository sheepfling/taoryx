from __future__ import annotations

import hashlib
import re
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
METADATA_PATH = ROOT / "metadata" / "diplomatic_transcription_chapter4_v17.yaml"
EDITORIAL_PATH = ROOT / "metadata" / "editorial_notes_chapter4.yaml"

EXPECTED_SECTION_COUNT = 8
REQUIRED_PHRASES: dict[str, tuple[str, ...]] = {
    "manual/chapters/chapter04/03_01_define.tex": (
        "These variables are always computed and are available for printout and plotting",
        "A temporary variable must be assigned before it is used",
        "While loops, for loops, and other C control statements cannot be used",
    ),
    "manual/chapters/chapter04/04_06_optimize.tex": (
        "A problem may contain as many as five optimization loops",
        "Derivative increment (default $1.0\\times10^{-8}$)",
        "Trajectory optimization is more an art than a science",
    ),
    "manual/chapters/chapter04/04_09_search.tex": (
        "Each search is one-dimensional",
        "values \\texttt{xest-dx} and \\texttt{xest+dx} should bracket the solution",
        "Search loops may not partially intersect",
    ),
    "manual/chapters/chapter04/04_10_summarize.tex": (
        "Summary variables are used with surveys to perform tradeoff and sensitivity studies",
        "uses that sample and its two neighbors to fit a parabola",
        "calculates the difference in final range between two trajectories",
    ),
    "manual/chapters/chapter04/04_11_survey.tex": (
        "This is a tradeoff or sensitivity study",
        "the lowest-numbered survey is innermost",
        "inserted in the survey in order from lowest to highest",
    ),
    "manual/chapters/chapter04/05_02_ballistic_rocket.tex": (
        "This example simulates a two-stage ballistic rocket trajectory",
        "The source problem converged in 12 iterations",
        "The solid curve is the main vehicle",
    ),
    "manual/chapters/chapter04/05_03_air_intercept.tex": (
        "This example simulates an air-launched interceptor",
        "The source problem converged after 50 iterations",
        "velocity at intercept is approximately 13,450 ft/s",
    ),
    "manual/chapters/chapter04/05_04_ground_intercept.tex": (
        "using the predictive guidance algorithm",
        "The source problem required approximately 1.5 seconds",
        "angle of attack has reached its $15^\\circ$ limit",
    ),
}

EXPECTED_DISPLAY_LISTINGS: dict[str, str] = {
    "manual/chapters/chapter04/05_02_ballistic_rocket.tex":
        "examples/chapter04/ballistic-rocket-source.prb.txt",
    "manual/chapters/chapter04/05_03_air_intercept.tex":
        "examples/chapter04/air-launched-intercept-source.prb.txt",
    "manual/chapters/chapter04/05_04_ground_intercept.tex":
        "examples/chapter04/ground-launched-intercept-source.prb.txt",
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
        ####
    ####
    return digest.hexdigest()
####


def _load_yaml(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise SystemExit(f"Expected a mapping in {path.relative_to(ROOT)}.")
    ####
    return payload
####


def _check_required_phrases() -> None:
    for relative_path, phrases in REQUIRED_PHRASES.items():
        path = ROOT / relative_path
        if not path.exists() or path.stat().st_size == 0:
            raise SystemExit(f"Missing diplomatic transcription file: {relative_path}")
        ####
        normalized_text = re.sub(r"\s+", " ", path.read_text(encoding="utf-8"))
        for phrase in phrases:
            normalized_phrase = re.sub(r"\s+", " ", phrase)
            if normalized_phrase not in normalized_text:
                raise SystemExit(f"Missing required source phrase in {relative_path}: {phrase!r}")
            ####
        ####
    ####
####


def _check_listings(metadata: dict[str, Any]) -> tuple[int, int]:
    supporting = metadata.get("supporting_files")
    if not isinstance(supporting, list) or len(supporting) != 3:
        raise SystemExit("Expected three Version 17 historical output listings.")
    ####
    for relative_path in supporting:
        path = ROOT / str(relative_path)
        if not path.exists() or path.stat().st_size < 1000:
            raise SystemExit(f"Missing or undersized Version 17 output listing: {relative_path}")
        ####
    ####

    source_listings = metadata.get("source_listing_files")
    if not isinstance(source_listings, list) or len(source_listings) != 3:
        raise SystemExit("Expected three Version 17 source-display problem listings.")
    ####
    for relative_path in source_listings:
        path = ROOT / str(relative_path)
        if not path.exists() or path.stat().st_size < 1000:
            raise SystemExit(f"Missing or undersized source-display listing: {relative_path}")
        ####
    ####

    for tex_path, listing_path in EXPECTED_DISPLAY_LISTINGS.items():
        text = (ROOT / tex_path).read_text(encoding="utf-8")
        if listing_path not in text:
            raise SystemExit(f"Diplomatic example does not use its source-display listing: {tex_path}")
        ####
    ####

    ground_output = (ROOT / "examples/chapter04/ground-intercept-printout.txt").read_text(
        encoding="utf-8"
    )
    if "107129.87  -4731.568" not in ground_output:
        raise SystemExit("Ground-intercept printout does not preserve the visible source value -4731.568.")
    ####
    return len(supporting), len(source_listings)
####


def _check_fixture_invariance(metadata: dict[str, Any]) -> int:
    fixture_invariance = metadata.get("fixture_invariance")
    expected_hashes = fixture_invariance.get("sha256") if isinstance(fixture_invariance, dict) else None
    if not isinstance(expected_hashes, dict) or len(expected_hashes) != 10:
        raise SystemExit("Expected SHA-256 records for ten parser fixtures.")
    ####
    for relative_path, expected_digest in expected_hashes.items():
        path = ROOT / str(relative_path)
        if not path.exists():
            raise SystemExit(f"Parser fixture disappeared: {relative_path}")
        ####
        actual_digest = _sha256(path)
        if actual_digest != expected_digest:
            raise SystemExit(
                f"Parser fixture changed during the Version 17 diplomatic pass: {relative_path}\n"
                f"expected {expected_digest}\nactual   {actual_digest}"
            )
        ####
    ####
    return len(expected_hashes)
####


def _check_editorial_notes(editorial: dict[str, Any]) -> None:
    notes = editorial.get("notes")
    note_ids = {
        str(item.get("id"))
        for item in notes
        if isinstance(notes, list) and isinstance(item, dict)
    }
    expected_note_ids = {
        "chapter4-v17-example-output-transcriptions",
        "chapter4-v17-fixture-invariance",
        "chapter4-v17-source-listing-separation",
        "chapter4-optimize-central-difference",
        "chapter4-optimize-dx-default",
        "chapter4-ground-intercept-first-relative-velocity",
    }
    if not expected_note_ids.issubset(note_ids):
        missing = sorted(expected_note_ids - note_ids)
        raise SystemExit(f"Chapter 4 editorial metadata is missing Version 17 notes: {missing}")
    ####
####


def main() -> None:
    metadata = _load_yaml(METADATA_PATH)
    editorial = _load_yaml(EDITORIAL_PATH)

    sections = metadata.get("sections")
    if not isinstance(sections, list) or len(sections) != EXPECTED_SECTION_COUNT:
        raise SystemExit(f"Expected {EXPECTED_SECTION_COUNT} Version 17 diplomatic sections.")
    ####
    section_files = {str(item.get("tex_file")) for item in sections if isinstance(item, dict)}
    if section_files != set(REQUIRED_PHRASES):
        missing = sorted(set(REQUIRED_PHRASES) - section_files)
        extra = sorted(section_files - set(REQUIRED_PHRASES))
        raise SystemExit(f"Version 17 section-file mismatch. Missing={missing}; extra={extra}")
    ####

    _check_required_phrases()
    output_count, source_listing_count = _check_listings(metadata)
    fixture_count = _check_fixture_invariance(metadata)
    _check_editorial_notes(editorial)

    print(
        "Chapter 4 Version 17 diplomatic checks passed: "
        f"{len(sections)} sections restored, {source_listing_count} source listings and "
        f"{output_count} output listings separated, and {fixture_count} parser fixtures unchanged."
    )
####


if __name__ == "__main__":
    main()
####
