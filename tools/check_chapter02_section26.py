from __future__ import annotations

import csv
import math
import re
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from numpy.typing import NDArray

ROOT = Path(__file__).resolve().parents[1]
CHAPTER = ROOT / "chapters" / "chapter02"
WRAPPER = ROOT / "chapters" / "02_methods.tex"

EXPECTED_SECTION_FILES = [
    "06_00_trajectory_calculations.tex",
    "06_01_derivative_calculations.tex",
    "06_02_search_methods.tex",
    "06_03_optimization.tex",
]
EXPECTED_EQUATION_NUMBERS = {f"2-{number}" for number in range(298, 316)}
EXPECTED_FIGURES = {f"2-{number}" for number in range(31, 40)}
EXPECTED_EDITORIAL_IDS = {
    "section26_newton_reciprocal_derivative_form",
    "section26_altitude_integral_differential",
    "section26_altitude_constraint_strict_inequality",
    "section26_figure_reconstruction_strategy",
}


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))
    ####
####


def _load_yaml(path: Path) -> Any:
    with path.open() as handle:
        return yaml.safe_load(handle)
    ####
####


def _chapter_number(value: str) -> str:
    return value.strip()
####


def _parabolic_coefficients(
    x_values: NDArray[np.float64],
    f_values: NDArray[np.float64],
) -> tuple[float, float, float]:
    x1, x2, x3 = (float(value) for value in x_values)
    f1, f2, f3 = (float(value) for value in f_values)
    denominator = (x2**2 - x3**2) * (x1 - x3) - (x1**2 - x3**2) * (x2 - x3)
    b_value = (
        (x2**2 - x3**2) * (f1 - f3)
        - (x1**2 - x3**2) * (f2 - f3)
    ) / denominator
    a_value = (f2 - f3 - b_value * (x2 - x3)) / (x2**2 - x3**2)
    c_value = f2 - a_value * x2**2 - b_value * x2
    return a_value, b_value, c_value
####


def _validate_search_equations() -> None:
    function = lambda value: value**2 - 2.0
    derivative = lambda value: 2.0 * value

    x_n = 1.5
    newton = x_n - function(x_n) / derivative(x_n)
    if not math.isclose(newton, 17.0 / 12.0, rel_tol=1.0e-14):
        raise SystemExit("Newton-Raphson update failed its numerical check.")
    ####

    x_lo, x_hi = 1.0, 2.0
    secant = x_lo - function(x_lo) * (x_hi - x_lo) / (function(x_hi) - function(x_lo))
    if not math.isclose(secant, 4.0 / 3.0, rel_tol=1.0e-14):
        raise SystemExit("Secant update failed its numerical check.")
    ####

    x_values: NDArray[np.float64] = np.array([-1.0, 0.5, 2.0], dtype=float)
    coefficients = (1.75, -0.4, -2.2)
    f_values: NDArray[np.float64] = np.array(
        [coefficients[0] * x**2 + coefficients[1] * x + coefficients[2] for x in x_values],
        dtype=float,
    )
    recovered = _parabolic_coefficients(x_values, f_values)
    if not np.allclose(recovered, coefficients, atol=1.0e-13):
        raise SystemExit("Parabolic coefficient equations failed to recover a test polynomial.")
    ####

    a_value, b_value, c_value = recovered
    roots = np.roots([a_value, b_value, c_value])
    for root in roots:
        if abs(a_value * root**2 + b_value * root + c_value) > 1.0e-11:
            raise SystemExit("Parabolic root equation failed its residual check.")
        ####
    ####

    alpha = (math.sqrt(5.0) - 1.0) / 2.0
    x1 = x_hi - alpha * (x_hi - x_lo)
    x2 = x_lo + alpha * (x_hi - x_lo)
    if not (x_lo < x1 < x2 < x_hi):
        raise SystemExit("Golden-section interior points are not ordered correctly.")
    ####
    if not math.isclose(x1 - x_lo, x_hi - x2, rel_tol=1.0e-14):
        raise SystemExit("Golden-section interior points are not symmetric.")
    ####

    minimum = -b_value / (2.0 * a_value)
    expected = 0.4 / (2.0 * 1.75)
    if not math.isclose(minimum, expected, rel_tol=1.0e-14):
        raise SystemExit("Parabolic minimum equation failed its numerical check.")
    ####
####


def _validate_quadratic_subproblem() -> None:
    b_matrix: NDArray[np.float64] = np.array([[4.0, 1.0], [1.0, 3.0]], dtype=float)
    gradient: NDArray[np.float64] = np.array([-1.0, 2.0], dtype=float)
    constraint_matrix: NDArray[np.float64] = np.array([[1.0, 1.0]], dtype=float)
    constraint_value: NDArray[np.float64] = np.array([-0.5], dtype=float)
    kkt = np.block(
        [
            [b_matrix, constraint_matrix.T],
            [constraint_matrix, np.zeros((1, 1), dtype=float)],
        ]
    )
    rhs = np.concatenate((-gradient, -constraint_value))
    solution = np.linalg.solve(kkt, rhs)
    delta_x = solution[:2]
    multiplier = solution[2:]
    stationarity = b_matrix @ delta_x + gradient + constraint_matrix.T @ multiplier
    feasibility = constraint_matrix @ delta_x + constraint_value
    if not np.allclose(stationarity, np.zeros(2), atol=1.0e-13):
        raise SystemExit("Quadratic subproblem failed the KKT stationarity check.")
    ####
    if not np.allclose(feasibility, np.zeros(1), atol=1.0e-13):
        raise SystemExit("Quadratic subproblem failed the linearized constraint check.")
    ####

    altitude = np.array([99_000.0, 101_000.0, 102_000.0], dtype=float)
    derivative = np.where(altitude > 100_000.0, (altitude - 100_000.0) ** 2, 0.0)
    integral = float(np.trapezoid(derivative, dx=1.0))
    if integral <= 0.0:
        raise SystemExit("Path-violation integral did not accumulate a positive violation.")
    ####
####


def main() -> None:
    missing_sections = [name for name in EXPECTED_SECTION_FILES if not (CHAPTER / name).exists()]
    if missing_sections:
        raise SystemExit("Missing Section 2.6 source files: " + ", ".join(missing_sections))
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
        raise SystemExit("Completed Section 2.6 files still contain skeleton placeholders.")
    ####
    if r"\cref{tab:taos-program-modules}" not in completed_text:
        raise SystemExit("Section 2.6 does not link back to Table 2-1's module context.")
    ####

    equation_labels = set(re.findall(r"\\label\{eq:[^}]+\}", completed_text))
    if len(equation_labels) != 18:
        raise SystemExit(
            f"Unexpected Section 2.6 equation-label count: {len(equation_labels)}; expected 18."
        )
    ####

    equation_rows = [
        row
        for row in _read_csv(ROOT / "metadata" / "equations.csv")
        if _chapter_number(row["equation"]) in EXPECTED_EQUATION_NUMBERS
    ]
    equation_numbers = {_chapter_number(row["equation"]) for row in equation_rows}
    if equation_numbers != EXPECTED_EQUATION_NUMBERS:
        raise SystemExit("Equation metadata does not contain every equation from 2-298 through 2-315.")
    ####
    if any(row["status"] != "visually_verified" for row in equation_rows):
        raise SystemExit("One or more Section 2.6 equations are not visually verified.")
    ####

    figure_rows = {row["figure"]: row for row in _read_csv(ROOT / "metadata" / "figures.csv")}
    for figure_number in EXPECTED_FIGURES:
        row = figure_rows.get(figure_number)
        if row is None or row["status"] != "visually_verified":
            raise SystemExit(f"Figure metadata is missing or incomplete for {figure_number}.")
        ####
        tikz = ROOT / "figures" / "chapter02" / f"fig-{figure_number}.tex"
        raster = ROOT / "figures" / "chapter02" / "source-crops" / f"fig-{figure_number}.png"
        if not tikz.exists() or not raster.exists():
            raise SystemExit(f"Figure {figure_number} lacks its TikZ reconstruction or source crop.")
        ####
    ####

    for metadata_name in (
        "trajectory_execution_chapter2.yaml",
        "search_algorithms_chapter2.yaml",
        "optimization_model_chapter2.yaml",
        "editorial_notes_chapter2.yaml",
    ):
        _load_yaml(ROOT / "metadata" / metadata_name)
    ####

    editorial_text = (ROOT / "metadata" / "editorial_notes_chapter2.yaml").read_text()
    missing_editorial = [item for item in EXPECTED_EDITORIAL_IDS if item not in editorial_text]
    if missing_editorial:
        raise SystemExit("Missing Section 2.6 editorial notes: " + ", ".join(missing_editorial))
    ####

    progress = _load_yaml(ROOT / "metadata" / "progress.yaml")
    if progress["chapter_2_parts"]["trajectory_calculations"]["status"] != "complete":
        raise SystemExit("Section 2.6 is not marked complete in the progress manifest.")
    ####

    _validate_search_equations()
    _validate_quadratic_subproblem()
    print("Chapter 2 Section 2.6 validation passed.")
####


if __name__ == "__main__":
    main()
####
