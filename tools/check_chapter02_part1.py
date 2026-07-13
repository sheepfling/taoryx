from __future__ import annotations

import csv
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CHAPTER = ROOT / "chapters" / "chapter02"
WRAPPER = ROOT / "chapters" / "02_methods.tex"

EXPECTED_SECTION_FILES: tuple[str, ...] = (
    "00_introduction.tex",
    "01_coordinate_systems_overview.tex",
    "01_01_ecfc.tex",
    "01_02_ecic.tex",
    "01_03_local_geocentric.tex",
    "01_04_geocentric.tex",
    "01_05_local_geodetic.tex",
    "01_06_geodetic.tex",
)


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
        raise SystemExit(f"Missing Chapter 2 Part I section files: {', '.join(missing_sections)}")
    ####

    wrapper = WRAPPER.read_text()
    missing_inputs = [
        name
        for name in EXPECTED_SECTION_FILES
        if f"chapter02/{Path(name).stem}" not in wrapper
    ]
    if missing_inputs:
        raise SystemExit(f"Chapter 2 wrapper omits: {', '.join(missing_inputs)}")
    ####

    completed_text = "\n".join((CHAPTER / name).read_text() for name in EXPECTED_SECTION_FILES)
    if r"\mbox{}\par" in completed_text:
        raise SystemExit("Completed Chapter 2 Part I files still contain skeleton placeholders.")
    ####

    counts = {
        "figures": len(set(re.findall(r"\\label\{fig:[^}]+\}", completed_text))),
        "tables": len(set(re.findall(r"\\label\{tab:[^}]+\}", completed_text))),
        "equations": len(set(re.findall(r"\\label\{eq:[^}]+\}", completed_text))),
    }
    expected_counts = {"figures": 7, "tables": 1, "equations": 49}
    if counts != expected_counts:
        raise SystemExit(
            f"Unexpected Chapter 2 Part I semantic counts: {counts}; expected {expected_counts}"
        )
    ####

    figure_rows = [
        row
        for row in _read_csv(ROOT / "metadata" / "figures.csv")
        if _chapter_number(row["figure"]) in range(1, 8)
    ]
    if len(figure_rows) != 7 or any(row["status"] != "visually_verified" for row in figure_rows):
        raise SystemExit("Figure metadata for Chapter 2 Figures 2-1 through 2-7 is incomplete.")
    ####

    equation_rows = [
        row
        for row in _read_csv(ROOT / "metadata" / "equations.csv")
        if _chapter_number(row["equation"]) in range(1, 50)
    ]
    equation_numbers = {_chapter_number(row["equation"]) for row in equation_rows}
    if equation_numbers != set(range(1, 50)):
        raise SystemExit("Equation metadata does not contain each equation from 2-1 through 2-49.")
    ####
    if any(row["status"] != "visually_verified" for row in equation_rows):
        raise SystemExit("One or more Chapter 2 Part I equations are not visually verified.")
    ####

    table_rows = [
        row for row in _read_csv(ROOT / "metadata" / "tables.csv") if row["table"] == "2-1"
    ]
    if len(table_rows) != 1 or table_rows[0]["status"] != "complete":
        raise SystemExit("Table 2-1 metadata is missing or incomplete.")
    ####

    figure_dir = ROOT / "figures" / "chapter02"
    missing_figure_assets: list[str] = []
    for number in range(1, 8):
        for relative in (Path(f"fig-2-{number}.tex"), Path("source-crops") / f"fig-2-{number}.png"):
            if not (figure_dir / relative).exists():
                missing_figure_assets.append(str(relative))
            ####
        ####
    ####
    if missing_figure_assets:
        raise SystemExit(f"Missing Chapter 2 figure assets: {', '.join(missing_figure_assets)}")
    ####

    required_metadata = ROOT / "metadata" / "coordinate_systems_chapter2.yaml"
    if not required_metadata.exists():
        raise SystemExit("Missing coordinate-system semantic metadata.")
    ####

    state_metadata = ROOT / "metadata" / "state_variables_chapter2_coordinates.yaml"
    if not state_metadata.exists():
        raise SystemExit("Missing coordinate state-variable semantic metadata.")
    ####
    expected_state_variables: set[str] = {
        "xecfc",
        "yecfc",
        "zecfc",
        "xecfcdt",
        "yecfcdt",
        "zecfcdt",
        "xecfcdt2",
        "yecfcdt2",
        "zecfcdt2",
        "xecic",
        "yecic",
        "zecic",
        "xecicdt",
        "yecicdt",
        "zecicdt",
        "xecicdt2",
        "yecicdt2",
        "zecicdt2",
        "rcm",
        "long",
        "latgc",
        "vel",
        "gamgc",
        "psigc",
        "alt",
        "latgd",
        "gamgd",
        "psigd",
    }
    metadata_text = state_metadata.read_text()
    observed_state_variables = set(re.findall(r"name:\s*([a-z0-9]+)", metadata_text))
    missing_state_variables = sorted(expected_state_variables - observed_state_variables)
    if missing_state_variables:
        raise SystemExit(
            "Coordinate state-variable metadata omits: " + ", ".join(missing_state_variables)
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
            raise SystemExit(f"Build log contains layout/reference failures: {', '.join(found)}")
        ####
    ####

    print(
        "Chapter 2 Part I check passed: "
        f"{len(EXPECTED_SECTION_FILES)} source sections, "
        f"{counts['figures']} figures, {counts['tables']} table, "
        f"and {counts['equations']} equations."
    )
####


if __name__ == "__main__":
    main()
####
