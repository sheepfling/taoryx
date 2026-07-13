from __future__ import annotations

import hashlib
import re
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
METADATA_PATH = ROOT / "metadata" / "diplomatic_transcription_chapter4.yaml"
EDITORIAL_PATH = ROOT / "metadata" / "editorial_notes_chapter4.yaml"

EXPECTED_SECTION_COUNT = 8
REQUIRED_PHRASES: dict[str, tuple[str, ...]] = {
    "manual/chapters/chapter04/02_01_aero.tex": (
        "It is optional; if it is not input, all aerodynamic coefficients are set",
        "Coefficients from different sets cannot be mixed",
        "Once a value has been given for a user-defined variable, it remains set",
    ),
    "manual/chapters/chapter04/02_08_limits.tex": (
        "being a three-degree-of-freedom simulation",
        "Greater-than and less-than signs are used",
        "help control convergence during these searches",
    ),
    "manual/chapters/chapter04/02_12_when.tex": (
        "otherwise, the segment never terminates",
        "east[2]=east[1] goto 7",
        "This prevents infinite loops",
    ),
    "manual/chapters/chapter04/04_02_define.tex": (
        "cannot be integral variables",
        "They cannot be evaluated outside of a trajectory",
        "Equations must end with a semicolon like in C",
    ),
    "manual/chapters/chapter04/04_04_egs.tex": (
        "special format called the EGS database format",
        "overwrites the \\problemblock{*egs} database files without warning",
        "requires at least one survey loop and at least one summary variable",
    ),
    "manual/chapters/chapter04/04_05_file.tex": (
        "Each output line is limited to 400 characters",
        "about 30",
        "12 or fewer",
    ),
    "manual/chapters/chapter04/04_07_print.tex": (
        "132 characters per line and 60 lines per page",
        "maximum of 10 output variables",
        "related by mission time",
    ),
    "manual/chapters/chapter04/05_01_ballistic_reentry.tex": (
        "This problem calculates 16 trajectories",
        "a little over 0.5 seconds to compute each trajectory",
        "The summary variables are tabulated as a function of the survey variables",
    ),
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


def main() -> None:
    metadata = _load_yaml(METADATA_PATH)
    editorial = _load_yaml(EDITORIAL_PATH)

    sections = metadata.get("sections")
    if not isinstance(sections, list) or len(sections) != EXPECTED_SECTION_COUNT:
        raise SystemExit(
            f"Expected {EXPECTED_SECTION_COUNT} diplomatic sections; got "
            f"{len(sections) if isinstance(sections, list) else 'invalid metadata'}."
        )
    ####

    section_files = {str(item.get("tex_file")) for item in sections if isinstance(item, dict)}
    if section_files != set(REQUIRED_PHRASES):
        missing = sorted(set(REQUIRED_PHRASES) - section_files)
        extra = sorted(section_files - set(REQUIRED_PHRASES))
        raise SystemExit(f"Diplomatic section-file mismatch. Missing={missing}; extra={extra}")
    ####

    for relative_path, phrases in REQUIRED_PHRASES.items():
        path = ROOT / relative_path
        if not path.exists() or path.stat().st_size == 0:
            raise SystemExit(f"Missing diplomatic transcription file: {relative_path}")
        ####
        text = path.read_text(encoding="utf-8")
        normalized_text = re.sub(r"\s+", " ", text)
        for phrase in phrases:
            normalized_phrase = re.sub(r"\s+", " ", phrase)
            if normalized_phrase not in normalized_text:
                raise SystemExit(f"Missing required source phrase in {relative_path}: {phrase!r}")
            ####
        ####
    ####

    supporting = metadata.get("supporting_files")
    if not isinstance(supporting, list) or len(supporting) != 2:
        raise SystemExit("Expected two supporting Ballistic Reentry output listings.")
    ####
    for relative_path in supporting:
        path = ROOT / str(relative_path)
        if not path.exists() or path.stat().st_size < 200:
            raise SystemExit(f"Missing or undersized supporting listing: {relative_path}")
        ####
    ####

    source_listings = metadata.get("source_listing_files")
    if not isinstance(source_listings, list) or len(source_listings) != 2:
        raise SystemExit("Expected two source-faithful Ballistic Reentry input listings.")
    ####
    for relative_path in source_listings:
        path = ROOT / str(relative_path)
        if not path.exists() or path.stat().st_size < 100:
            raise SystemExit(f"Missing or undersized source input listing: {relative_path}")
        ####
    ####

    fixture_invariance = metadata.get("fixture_invariance")
    if not isinstance(fixture_invariance, dict):
        raise SystemExit("Missing fixture-invariance metadata.")
    ####
    expected_hashes = fixture_invariance.get("sha256")
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
                f"Parser fixture changed during diplomatic pass: {relative_path}\n"
                f"expected {expected_digest}\nactual   {actual_digest}"
            )
        ####
    ####

    notes = editorial.get("notes")
    if not isinstance(notes, list) or len(notes) < 4:
        raise SystemExit("Chapter 4 editorial metadata is incomplete.")
    ####
    note_ids = {str(item.get("id")) for item in notes if isinstance(item, dict)}
    expected_note_ids = {
        "chapter4-limits-prop-name",
        "chapter4-when-east-example",
        "chapter4-problem-define-identifiers",
        "chapter4-ballistic-output-transcription",
        "chapter4-ballistic-fixture-separation",
    }
    if not expected_note_ids.issubset(note_ids):
        raise SystemExit("Chapter 4 editorial metadata is missing required notes.")
    ####

    print(
        "Chapter 4 diplomatic-transcription checks passed: "
        f"{len(sections)} sections restored, {len(source_listings)} source listings separated, "
        f"and {len(expected_hashes)} parser fixtures unchanged."
    )
####


if __name__ == "__main__":
    main()
####
