from __future__ import annotations

import csv
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CHAPTER = ROOT / "chapters" / "chapter04"
WRAPPER = ROOT / "chapters" / "04_problem_files.tex"

EXPECTED_SECTION_FILES: tuple[str, ...] = (
    "00_introduction.tex",
    "01_file_format.tex",
    "02_segment_overview.tex",
    "02_01_aero.tex",
    "02_02_constants.tex",
    "02_03_cg.tex",
    "02_04_fly.tex",
    "02_05_increment.tex",
    "02_06_inertial.tex",
    "02_07_integ.tex",
    "02_08_limits.tex",
    "02_09_prop.tex",
    "02_10_rail.tex",
    "02_11_reset.tex",
    "02_12_when.tex",
    "03_trajectory_overview.tex",
    "03_01_define.tex",
    "03_02_dwn_crs.tex",
    "03_03_file.tex",
    "03_04_iip.tex",
    "03_05_initial.tex",
    "03_06_print.tex",
    "03_07_tangent.tex",
    "04_problem_overview.tex",
    "04_01_atmos.tex",
    "04_02_define.tex",
    "04_03_earth.tex",
    "04_04_egs.tex",
    "04_05_file.tex",
    "04_06_optimize.tex",
    "04_07_print.tex",
    "04_08_radar.tex",
    "04_09_search.tex",
    "04_10_summarize.tex",
    "04_11_survey.tex",
    "04_12_title.tex",
    "04_13_units_fmt.tex",
    "04_14_wind.tex",
    "05_examples_intro.tex",
    "05_01_ballistic_reentry.tex",
    "05_02_ballistic_rocket.tex",
    "05_03_air_intercept.tex",
    "05_04_ground_intercept.tex",
)

EXPECTED_EXAMPLES: tuple[str, ...] = (
    "ballistic-reentry.tbl",
    "ballistic-reentry.prb",
    "ballistic-rocket.prb",
    "air-launched-intercept.prb",
    "ground-launched-intercept.prb",
)


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))
    ####
####


def main() -> None:
    missing_sections = [name for name in EXPECTED_SECTION_FILES if not (CHAPTER / name).exists()]
    if missing_sections:
        raise SystemExit(f"Missing Chapter 4 section files: {', '.join(missing_sections)}")
    ####

    wrapper = WRAPPER.read_text()
    missing_inputs = [
        name
        for name in EXPECTED_SECTION_FILES
        if f"chapter04/{Path(name).stem}" not in wrapper
    ]
    if missing_inputs:
        raise SystemExit(f"Chapter 4 wrapper omits: {', '.join(missing_inputs)}")
    ####
    if r"\mbox{}\par" in wrapper:
        raise SystemExit("Chapter 4 wrapper still contains skeleton placeholders.")
    ####

    example_dir = ROOT / "examples" / "chapter04"
    missing_examples = [name for name in EXPECTED_EXAMPLES if not (example_dir / name).exists()]
    if missing_examples:
        raise SystemExit(f"Missing Chapter 4 example fixtures: {', '.join(missing_examples)}")
    ####

    chapter_text = "\n".join(path.read_text() for path in sorted(CHAPTER.glob("*.tex")))
    counts = {
        "figures": len(set(re.findall(r"\\label\{fig:[^}]+\}", chapter_text))),
        "tables": len(set(re.findall(r"\\label\{tab:[^}]+\}", chapter_text))),
        "equations": len(set(re.findall(r"\\label\{eq:[^}]+\}", chapter_text))),
    }
    expected_counts = {"figures": 21, "tables": 24, "equations": 8}
    if counts != expected_counts:
        raise SystemExit(f"Unexpected Chapter 4 semantic counts: {counts}; expected {expected_counts}")
    ####

    figure_rows = [r for r in _read_csv(ROOT / "metadata" / "figures.csv") if r["figure"].startswith("4-")]
    table_rows = [r for r in _read_csv(ROOT / "metadata" / "tables.csv") if r["table"].startswith("4-")]
    equation_rows = [r for r in _read_csv(ROOT / "metadata" / "equations.csv") if r["equation"].startswith("4-")]
    metadata_counts = {
        "figures": len(figure_rows),
        "tables": len(table_rows),
        "equations": len(equation_rows),
    }
    if metadata_counts != expected_counts:
        raise SystemExit(
            f"Unexpected Chapter 4 metadata counts: {metadata_counts}; expected {expected_counts}"
        )
    ####

    log_path = ROOT / "build" / "manual.log"
    if log_path.exists():
        log = log_path.read_text(errors="replace")
        forbidden = ("undefined references", "multiply defined", "Overfull \\hbox", "Overfull \\vbox")
        found = [token for token in forbidden if token in log]
        if found:
            raise SystemExit(f"Build log contains layout/reference failures: {', '.join(found)}")
        ####
    ####

    print(
        "Chapter 4 check passed: "
        f"{len(EXPECTED_SECTION_FILES)} source sections, "
        f"{counts['figures']} figures, {counts['tables']} tables, "
        f"{counts['equations']} equations, and {len(EXPECTED_EXAMPLES)} example fixtures."
    )
####


if __name__ == "__main__":
    main()
####
