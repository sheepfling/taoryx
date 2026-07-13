from __future__ import annotations

import hashlib
import re
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
METADATA_PATH = ROOT / "metadata" / "diplomatic_transcription_chapter4_v18.yaml"
EDITORIAL_PATH = ROOT / "metadata" / "editorial_notes_chapter4.yaml"
EXPECTED_SECTION_COUNT = 20

REQUIRED_PHRASES: dict[str, tuple[str, ...]] = {
    "manual/chapters/chapter04/02_02_constants.tex": (
        "The \\problemblock{*constants} data block is optional",
        "remains set until changed in a subsequent data block",
    ),
    "manual/chapters/chapter04/02_03_cg.tex": (
        "center of gravity defaults to zero",
        "TAOS treats the value as nondimensional",
    ),
    "manual/chapters/chapter04/02_05_increment.tex": (
        "values are reset first and then incremented",
        "A problem with one trajectory can contain a discontinuity in absolute time",
        "Object Deployment",
    ),
    "manual/chapters/chapter04/02_06_inertial.tex": (
        "The default alignment is geodetic",
        "dummy segment",
        "right-handed system",
    ),
    "manual/chapters/chapter04/02_07_integ.tex": (
        "The default step size is 0.10~s",
        "The minimum allowable integration step is $1\\times10^{-6}$~s",
        "The default value of 1~s works well for most guidance rules",
    ),
    "manual/chapters/chapter04/02_09_prop.tex": (
        "If no \\problemblock{*prop} blocks are given, thrust and mass flow are set to zero",
        "override those in the tables",
        "in future versions they would be control variables",
    ),
    "manual/chapters/chapter04/02_10_rail.tex": (
        "Static friction is used while velocity is less than 0.001~ft/s",
        "Negative acceleration in the rail direction is therefore ignored",
    ),
    "manual/chapters/chapter04/02_11_reset.tex": (
        "Rocket-motor thrust in propulsion tables is often made a function of \\statevar{tmark}",
        "sets the weight to 1100~lb regardless of what has happened previously in the trajectory",
    ),
    "manual/chapters/chapter04/03_02_dwn_crs.tex": (
        "The reference longitude, latitude, and heading default to the beginning of the trajectory",
        "All of these distances use the same reference point",
    ),
    "manual/chapters/chapter04/03_03_file.tex": (
        "If a file already exists, TAOS overwrites it",
        "Any number of trajectory-level \\problemblock{*file} blocks can be included",
    ),
    "manual/chapters/chapter04/03_04_iip.tex": (
        "Both values default to zero",
        "range and azimuth from the origin of the tangent-plane coordinate system",
    ),
    "manual/chapters/chapter04/03_05_initial.tex": (
        "Geodetic coordinates are the default",
        "One may replace the other, but both cannot be entered",
        "Initial Conditions from a Trajectory",
    ),
    "manual/chapters/chapter04/03_06_print.tex": (
        "a maximum of ten trajectory variables",
        "Units and formats are controlled by \\problemblock{*units/fmt}",
    ),
    "manual/chapters/chapter04/03_07_tangent.tex": (
        "By default, the origin is on the earth's surface directly below",
        "The initial-impact-point range and azimuth are also measured from the tangent-plane origin",
    ),
    "manual/chapters/chapter04/04_01_atmos.tex": (
        "The default is the 1976 U.S. Standard Atmosphere",
        "The models extend to 1000~km. Above 146~km",
        "different trajectories cannot use different atmosphere models",
    ),
    "manual/chapters/chapter04/04_03_earth.tex": (
        "TAOS requires an earth model to generate a trajectory",
        "selects WGS-84 but makes the earth nonrotating, a simplification often used in conceptual-design studies",
        "neglected gravitational effects of the sun and moon",
    ),
    "manual/chapters/chapter04/04_08_radar.tex": (
        "The presence of one or more radar output variables in a problem triggers the radar calculation",
        "do not follow earth curvature",
        "A radar block may override that shape for radar geometry only",
    ),
    "manual/chapters/chapter04/04_12_title.tex": (
        "There is no formal limit on the number of title lines",
        "extends to the next valid data-block keyword",
    ),
    "manual/chapters/chapter04/04_13_units_fmt.tex": (
        "they change everywhere on both input and output",
        "Units must be dimensionally consistent",
        "TAOS cannot convert its units because it does not know the variable's original dimensions",
    ),
    "manual/chapters/chapter04/04_14_wind.tex": (
        "If no block is supplied, all wind velocities are zero",
        "Wind heading is the direction \\emph{from which} the wind blows",
        "The speed/heading set and the east/north component set cannot be mixed",
    ),
}

EXPECTED_NOTE_IDS = {
    "chapter4-v18-source-normalization",
    "chapter4-v18-units-table-corrections",
    "chapter4-v18-fixture-invariance",
    "chapter4-v18-earth-model-table-preservation",
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
            raise SystemExit(f"Missing Version 18 diplomatic transcription file: {relative_path}")
        ####
        text = re.sub(r"\s+", " ", path.read_text(encoding="utf-8"))
        for phrase in phrases:
            normalized_phrase = re.sub(r"\s+", " ", phrase)
            if normalized_phrase not in text:
                raise SystemExit(f"Missing required source-near phrase in {relative_path}: {phrase!r}")
            ####
        ####
    ####
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
                f"Parser fixture changed during the Version 18 diplomatic pass: {relative_path}\n"
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
    if not EXPECTED_NOTE_IDS.issubset(note_ids):
        missing = sorted(EXPECTED_NOTE_IDS - note_ids)
        raise SystemExit(f"Chapter 4 editorial metadata is missing Version 18 notes: {missing}")
    ####
####


def _check_unescaped_texttt_underscores() -> None:
    pattern = re.compile(r"\\texttt\{([^{}]*)\}")
    for relative_path in REQUIRED_PHRASES:
        text = (ROOT / relative_path).read_text(encoding="utf-8")
        for match in pattern.finditer(text):
            value = match.group(1)
            if re.search(r"(?<!\\)_", value):
                raise SystemExit(
                    f"Unescaped underscore in \\texttt{{...}} in {relative_path}: {value!r}"
                )
            ####
        ####
    ####
####


def main() -> None:
    metadata = _load_yaml(METADATA_PATH)
    editorial = _load_yaml(EDITORIAL_PATH)
    sections = metadata.get("sections")
    if not isinstance(sections, list) or len(sections) != EXPECTED_SECTION_COUNT:
        raise SystemExit(f"Expected {EXPECTED_SECTION_COUNT} Version 18 diplomatic sections.")
    ####
    section_files = {str(item.get("tex_file")) for item in sections if isinstance(item, dict)}
    if section_files != set(REQUIRED_PHRASES):
        missing = sorted(set(REQUIRED_PHRASES) - section_files)
        extra = sorted(section_files - set(REQUIRED_PHRASES))
        raise SystemExit(f"Version 18 section-file mismatch. Missing={missing}; extra={extra}")
    ####
    _check_required_phrases()
    _check_unescaped_texttt_underscores()
    fixture_count = _check_fixture_invariance(metadata)
    _check_editorial_notes(editorial)
    print(
        "Chapter 4 Version 18 diplomatic checks passed: "
        f"{len(sections)} sections restored and {fixture_count} parser fixtures unchanged."
    )
####


if __name__ == "__main__":
    main()
####
