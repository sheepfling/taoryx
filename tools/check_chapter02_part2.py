from __future__ import annotations

import csv
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CHAPTER = ROOT / "chapters" / "chapter02"
WRAPPER = ROOT / "chapters" / "02_methods.tex"

EXPECTED_SECTION_FILES: tuple[str, ...] = (
    "01_07_body_fixed.tex",
    "01_08_velocity.tex",
    "01_09_wind.tex",
    "01_10_inertial_platform.tex",
    "01_11_tangent_plane.tex",
    "01_12_coordinate_transformations.tex",
)
EXPECTED_FIGURE_NUMBERS: set[int] = set(range(8, 16))
EXPECTED_EQUATION_NUMBERS: set[int] = set(range(50, 97))


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))
    ####
####


def _chapter_number(value: str) -> int | None:
    match = re.fullmatch(r"2-(\d+)", value)
    return int(match.group(1)) if match else None
####


def main() -> None:
    missing_sections = [name for name in EXPECTED_SECTION_FILES if not (CHAPTER / name).exists()]
    if missing_sections:
        raise SystemExit(
            "Missing Chapter 2 Part II section files: " + ", ".join(missing_sections)
        )
    ####

    wrapper = WRAPPER.read_text()
    missing_inputs = [
        name
        for name in EXPECTED_SECTION_FILES
        if f"chapter02/{Path(name).stem}" not in wrapper
    ]
    if missing_inputs:
        raise SystemExit("Chapter 2 wrapper omits: " + ", ".join(missing_inputs))
    ####

    completed_text = "\n".join((CHAPTER / name).read_text() for name in EXPECTED_SECTION_FILES)
    if r"\mbox{}\par" in completed_text:
        raise SystemExit("Completed Chapter 2 Part II files still contain skeleton placeholders.")
    ####

    counts = {
        "figures": len(set(re.findall(r"\\label\{fig:[^}]+\}", completed_text))),
        "equations": len(set(re.findall(r"\\label\{eq:[^}]+\}", completed_text))),
    }
    expected_counts = {"figures": 8, "equations": 47}
    if counts != expected_counts:
        raise SystemExit(
            f"Unexpected Chapter 2 Part II semantic counts: {counts}; expected {expected_counts}"
        )
    ####

    figure_rows = [
        row
        for row in _read_csv(ROOT / "metadata" / "figures.csv")
        if _chapter_number(row["figure"]) in EXPECTED_FIGURE_NUMBERS
    ]
    figure_numbers = {_chapter_number(row["figure"]) for row in figure_rows}
    if figure_numbers != EXPECTED_FIGURE_NUMBERS:
        raise SystemExit("Figure metadata does not contain Figures 2-8 through 2-15.")
    ####
    if any(row["status"] != "visually_verified" for row in figure_rows):
        raise SystemExit("One or more Chapter 2 Part II figures have an invalid status.")
    ####

    equation_rows = [
        row
        for row in _read_csv(ROOT / "metadata" / "equations.csv")
        if _chapter_number(row["equation"]) in EXPECTED_EQUATION_NUMBERS
    ]
    equation_numbers = {_chapter_number(row["equation"]) for row in equation_rows}
    if equation_numbers != EXPECTED_EQUATION_NUMBERS:
        raise SystemExit("Equation metadata does not contain each equation from 2-50 through 2-96.")
    ####
    if any(row["status"] != "visually_verified" for row in equation_rows):
        raise SystemExit("One or more Chapter 2 Part II equations have an invalid status.")
    ####

    figure_dir = ROOT / "figures" / "chapter02"
    missing_figure_assets: list[str] = []
    for number in EXPECTED_FIGURE_NUMBERS:
        for relative in (
            Path(f"fig-2-{number}.tex"),
            Path("source-crops") / f"fig-2-{number}.png",
        ):
            if not (figure_dir / relative).exists():
                missing_figure_assets.append(str(relative))
            ####
        ####
    ####
    if missing_figure_assets:
        raise SystemExit("Missing Chapter 2 Part II figure assets: " + ", ".join(missing_figure_assets))
    ####

    required_metadata: tuple[str, ...] = (
        "coordinate_transforms_chapter2.yaml",
        "aerodynamic_angles_chapter2.yaml",
        "rotation_matrices_chapter2.yaml",
        "editorial_notes_chapter2.yaml",
        "coordinate_systems_chapter2.yaml",
        "state_variables_chapter2_coordinates.yaml",
    )
    missing_metadata = [
        name for name in required_metadata if not (ROOT / "metadata" / name).exists()
    ]
    if missing_metadata:
        raise SystemExit("Missing Chapter 2 Part II metadata: " + ", ".join(missing_metadata))
    ####

    rotation_library = ROOT / "math" / "matrices" / "rotations.tex"
    if not rotation_library.exists():
        raise SystemExit("Missing reusable TAOS rotation-matrix library.")
    ####
    rotation_text = rotation_library.read_text()
    required_macros = (
        r"\TaosRollMatrix",
        r"\TaosPitchMatrix",
        r"\TaosYawMatrix",
        r"\TaosEulerSideslipMatrix",
        r"\TaosTransform",
    )
    missing_macros = [macro for macro in required_macros if macro not in rotation_text]
    if missing_macros:
        raise SystemExit("Rotation library omits: " + ", ".join(missing_macros))
    ####

    state_metadata = ROOT / "metadata" / "state_variables_chapter2_coordinates.yaml"
    expected_state_variables: set[str] = {
        "yawgc", "yawgd", "yawi",
        "pitchgc", "pitchgd", "pitchi",
        "rollgc", "rollgd", "rolli",
        "accebx", "acceby", "accebz",
        "accibx", "acciby", "accibz",
        "velebx", "veleby", "velebz",
        "velibx", "veliby", "velibz",
        "vair",
        "alpha", "alphat", "bankgc", "bankgd", "beta", "betae", "phi",
        "xip", "yip", "zip", "xipdt", "yipdt", "zipdt",
        "xipdt2", "yipdt2", "zipdt2",
        "xtp", "ytp", "ztp", "xtpdt", "ytpdt", "ztpdt",
        "xtpdt2", "ytpdt2", "ztpdt2",
    }
    metadata_text = state_metadata.read_text()
    observed_state_variables = set(re.findall(r"name:\s*([a-z0-9]+)", metadata_text))
    missing_state_variables = sorted(expected_state_variables - observed_state_variables)
    if missing_state_variables:
        raise SystemExit(
            "Chapter 2 Part II state metadata omits: " + ", ".join(missing_state_variables)
        )
    ####

    pdf_path = ROOT / "build" / "manual.pdf"
    if not pdf_path.exists():
        raise SystemExit("Compiled PDF is missing; run `python tools/dev.py manual` before validation.")
    ####

    log_path = ROOT / "build" / "manual.log"
    if log_path.exists():
        log = log_path.read_text(errors="replace")
        forbidden = (
            "undefined references",
            "multiply defined",
            "Overfull \\hbox",
            "Overfull \\vbox",
        )
        found = [token for token in forbidden if token in log]
        if found:
            raise SystemExit(
                "Build log contains layout/reference failures: " + ", ".join(found)
            )
        ####
    ####

    print(
        "Chapter 2 Part II check passed: "
        f"{len(EXPECTED_SECTION_FILES)} source sections, "
        f"{counts['figures']} figures, and {counts['equations']} equations."
    )
####


if __name__ == "__main__":
    main()
####
