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
EXPECTED_SECTION_FILES: tuple[str, ...] = (
    "05_00_guidance_rules.tex",
    "05_01_control_variable_solution.tex",
    "05_02_intercepts_proportional_navigation.tex",
    "05_03_range_insensitive_axis.tex",
    "05_04_flight_path_limits.tex",
)
EXPECTED_EQUATION_NUMBERS: set[int] = set(range(273, 298))
EXPECTED_FIGURES: set[str] = {f"2-{number}" for number in range(25, 31)}
EXPECTED_TABLES: set[str] = {"2-2", "2-3"}


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))
    ####
####


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open() as handle:
        loaded = yaml.safe_load(handle)
    ####
    if not isinstance(loaded, dict):
        raise SystemExit(f"Expected a mapping in {path.name}.")
    ####
    return loaded
####


def _chapter_number(value: str) -> int | None:
    match = re.fullmatch(r"2-(\d+)", value)
    return int(match.group(1)) if match else None
####


def _validate_parabolic_transition() -> None:
    time = 7.0
    delta_time = 3.5
    current = math.radians(11.0)
    desired = math.radians(5.0)
    desired_rate = math.radians(-0.15)
    coefficient_a = (current - desired + desired_rate * delta_time) / delta_time**2
    coefficient_b = desired_rate - 2.0 * coefficient_a * (time + delta_time)
    coefficient_c = current - coefficient_a * time**2 - coefficient_b * time

    def state(at_time: float) -> float:
        return coefficient_a * at_time**2 + coefficient_b * at_time + coefficient_c
    ####

    def rate(at_time: float) -> float:
        return 2.0 * coefficient_a * at_time + coefficient_b
    ####

    if not math.isclose(state(time), current, rel_tol=0.0, abs_tol=1.0e-14):
        raise SystemExit("Parabolic guidance transition does not pass through the current state.")
    ####
    if not math.isclose(state(time + delta_time), desired, rel_tol=0.0, abs_tol=1.0e-14):
        raise SystemExit("Parabolic guidance transition does not reach the desired state.")
    ####
    if not math.isclose(rate(time + delta_time), desired_rate, rel_tol=0.0, abs_tol=1.0e-14):
        raise SystemExit("Parabolic guidance transition does not reach the desired rate.")
    ####
####


def _validate_newton_system() -> None:
    jacobian: NDArray[np.float64] = np.array(
        [[3.0, -1.0], [2.0, 4.0]],
        dtype=float,
    )
    residual: NDArray[np.float64] = np.array([5.0, -2.0], dtype=float)
    increment = np.linalg.solve(jacobian, -residual)
    if not np.allclose(jacobian @ increment, -residual, rtol=0.0, atol=1.0e-13):
        raise SystemExit("Guidance Newton-Raphson linear system failed.")
    ####
####


def _validate_predictive_intercept() -> None:
    position_1: NDArray[np.float64] = np.array([0.0, 0.0, 0.0], dtype=float)
    position_2: NDArray[np.float64] = np.array([100.0, 0.0, 0.0], dtype=float)
    velocity_1: NDArray[np.float64] = np.array([30.0, 0.0, 0.0], dtype=float)
    velocity_2: NDArray[np.float64] = np.array([10.0, 0.0, 0.0], dtype=float)
    time_to_intercept = float(
        np.linalg.norm(position_2 - position_1) / np.linalg.norm(velocity_1 - velocity_2)
    )
    intercept_1 = position_1 + time_to_intercept * velocity_1
    intercept_2 = position_2 + time_to_intercept * velocity_2
    if not np.allclose(intercept_1, intercept_2, rtol=0.0, atol=1.0e-13):
        raise SystemExit("Predictive intercept equations do not meet for a collinear test case.")
    ####
####


def _los_angles(relative_position: NDArray[np.float64]) -> tuple[float, float]:
    x_value, y_value, z_value = relative_position
    yaw = math.atan2(y_value, x_value)
    pitch = math.atan2(z_value, math.hypot(x_value, y_value))
    return yaw, pitch
####


def _validate_proportional_navigation() -> None:
    relative_position: NDArray[np.float64] = np.array([12_000.0, 4_500.0, 2_000.0])
    relative_velocity: NDArray[np.float64] = np.array([-650.0, 85.0, -35.0])
    x_value, y_value, z_value = relative_position
    vx_value, vy_value, vz_value = relative_velocity
    horizontal_squared = x_value**2 + y_value**2
    range_squared = horizontal_squared + z_value**2
    yaw_rate = (x_value * vy_value - y_value * vx_value) / horizontal_squared
    pitch_rate = (
        vz_value * horizontal_squared
        - z_value * (x_value * vx_value + y_value * vy_value)
    ) / (math.sqrt(horizontal_squared) * range_squared)

    dt = 1.0e-5
    yaw_0, pitch_0 = _los_angles(relative_position)
    yaw_1, pitch_1 = _los_angles(relative_position + dt * relative_velocity)
    if not math.isclose(yaw_rate, (yaw_1 - yaw_0) / dt, rel_tol=3.0e-6):
        raise SystemExit("Line-of-sight yaw-rate equation failed a finite-difference check.")
    ####
    if not math.isclose(pitch_rate, (pitch_1 - pitch_0) / dt, rel_tol=3.0e-6):
        raise SystemExit("Line-of-sight pitch-rate equation failed a finite-difference check.")
    ####

    closure_velocity = -float(relative_velocity @ relative_position) / float(
        np.linalg.norm(relative_position)
    )
    if closure_velocity <= 0.0:
        raise SystemExit("Closing test geometry unexpectedly produced nonpositive closure velocity.")
    ####

    lambda_y = 0.27
    lambda_p = -0.18
    cosine_y, sine_y = math.cos(lambda_y), math.sin(lambda_y)
    cosine_p, sine_p = math.cos(lambda_p), math.sin(lambda_p)
    source_transform: NDArray[np.float64] = np.array(
        [
            [cosine_p * cosine_y, -sine_p * cosine_y, -sine_y],
            [sine_p, cosine_p, 0.0],
            [cosine_p * sine_y, -sine_p * sine_y, cosine_y],
        ],
        dtype=float,
    )
    if not np.allclose(source_transform.T @ source_transform, np.eye(3), atol=1.0e-13):
        raise SystemExit("Printed line-of-sight transformation matrix is not orthogonal.")
    ####
####


def main() -> None:
    missing_sections = [name for name in EXPECTED_SECTION_FILES if not (CHAPTER / name).exists()]
    if missing_sections:
        raise SystemExit("Missing Section 2.5 source files: " + ", ".join(missing_sections))
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
        raise SystemExit("Completed Section 2.5 files still contain skeleton placeholders.")
    ####

    equation_labels = set(re.findall(r"\\label\{eq:[^}]+\}", completed_text))
    if len(equation_labels) != 25:
        raise SystemExit(
            f"Unexpected Section 2.5 equation-label count: {len(equation_labels)}; expected 25."
        )
    ####

    equation_rows = [
        row
        for row in _read_csv(ROOT / "metadata" / "equations.csv")
        if _chapter_number(row["equation"]) in EXPECTED_EQUATION_NUMBERS
    ]
    equation_numbers = {_chapter_number(row["equation"]) for row in equation_rows}
    if equation_numbers != EXPECTED_EQUATION_NUMBERS:
        raise SystemExit("Equation metadata does not contain every equation from 2-273 through 2-297.")
    ####
    if any(row["status"] != "visually_verified" for row in equation_rows):
        raise SystemExit("One or more Section 2.5 equations are not visually verified.")
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

    table_rows = {row["table"]: row for row in _read_csv(ROOT / "metadata" / "tables.csv")}
    for table_number in EXPECTED_TABLES:
        row = table_rows.get(table_number)
        if row is None or row["status"] != "complete":
            raise SystemExit(f"Table metadata is missing or incomplete for {table_number}.")
        ####
    ####

    for metadata_name in (
        "guidance_models_chapter2.yaml",
        "state_variables_chapter2_guidance.yaml",
        "editorial_notes_chapter2.yaml",
    ):
        _load_yaml(ROOT / "metadata" / metadata_name)
    ####

    editorial_text = (ROOT / "metadata" / "editorial_notes_chapter2.yaml").read_text()
    required_editorial_ids = {
        "section25_los_rotation_angle_labels",
        "section25_split_acceleration_display",
        "section25_figure_reconstruction_strategy",
    }
    observed_editorial_ids = set(re.findall(r"id:\s*([a-z0-9_]+)", editorial_text))
    if not required_editorial_ids <= observed_editorial_ids:
        missing = sorted(required_editorial_ids - observed_editorial_ids)
        raise SystemExit("Section 2.5 editorial metadata omits: " + ", ".join(missing))
    ####

    _validate_parabolic_transition()
    _validate_newton_system()
    _validate_predictive_intercept()
    _validate_proportional_navigation()

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
            raise SystemExit("LaTeX log contains: " + ", ".join(found))
        ####
    ####

    print("Chapter 2 Section 2.5 validation passed.")
####


if __name__ == "__main__":
    main()
####
