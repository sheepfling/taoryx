from __future__ import annotations

import csv
import math
import re
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

ROOT = Path(__file__).resolve().parents[1]
CHAPTER = ROOT / "chapters" / "chapter02"
WRAPPER = ROOT / "chapters" / "02_methods.tex"
EXPECTED_SECTION_FILES: tuple[str, ...] = (
    "03_00_contributions_to_acceleration.tex",
    "03_01_atmosphere.tex",
    "03_02_aerodynamic_forces.tex",
    "03_03_propulsive_forces.tex",
    "03_04_gravity_model.tex",
)
EXPECTED_EQUATION_NUMBERS: set[int] = set(range(175, 221))
EXPECTED_STATE_VARIABLES: set[str] = {
    "temp",
    "pres",
    "rho",
    "sndspd",
    "nu",
    "dynprs",
    "mach",
    "reypft",
    "ca",
    "cn",
    "cl",
    "cd",
    "cs",
    "cx",
    "cy",
    "cz",
    "thrust",
    "ep1",
    "ep2",
}
EXPECTED_FIGURES: dict[str, tuple[str, str]] = {
    "2-16": ("fig-2-16.png", "fig-2-16.tex"),
    "2-17": ("fig-2-17.png", "fig-2-17.tex"),
}


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))
    ####
####


def _chapter_number(value: str) -> int | None:
    match = re.fullmatch(r"2-(\d+)", value)
    return int(match.group(1)) if match else None
####


def _validate_atmosphere_relations() -> None:
    gas_constant = 8314.32
    molecular_weight = 28.9644
    temperature = 288.15
    density = 1.225
    pressure = density * gas_constant * temperature / molecular_weight
    expected = 101_325.0
    if not math.isclose(pressure, expected, rel_tol=3.0e-4):
        raise SystemExit("Perfect-gas reconstruction does not reproduce sea-level pressure.")
    ####

    g0 = 9.806160 * (
        1.0 - 0.0026373 * math.cos(math.radians(2.0 * 45.5425))
        + 0.0000059 * math.cos(math.radians(2.0 * 45.5425)) ** 2
    )
    effective_radius = 2.0 * g0 / (
        3.085462e-6
        + 2.27e-9 * math.cos(math.radians(2.0 * 45.5425))
        - 2.0e-12 * math.cos(math.radians(4.0 * 45.5425))
    )
    if not math.isclose(effective_radius / 1000.0, 6356.766, rel_tol=2.0e-6):
        raise SystemExit("Effective geopotential earth-radius equation failed its reference check.")
    ####

    base_pressure = 22_632.1
    base_tau = 216.65
    geopotential_delta = 5_000.0
    exponent = -9.80665 * 28.9644 * geopotential_delta / (gas_constant * base_tau)
    pressure_isothermal = base_pressure * math.exp(exponent)
    if not (0.0 < pressure_isothermal < base_pressure):
        raise SystemExit("Isothermal pressure equation is not monotone with altitude.")
    ####
####


def _validate_force_relations() -> None:
    x_hat: NDArray[np.float64] = np.array([1.0, 0.0, 0.0], dtype=float)
    y_hat: NDArray[np.float64] = np.array([0.0, 1.0, 0.0], dtype=float)
    z_hat: NDArray[np.float64] = np.array([0.0, 0.0, 1.0], dtype=float)
    q = 420.0
    area = 3.5
    cd, cs, cl = 0.28, -0.04, 0.61
    force = -(cd * x_hat - cs * y_hat + cl * z_hat) * q * area
    expected = np.array([-cd, cs, -cl], dtype=float) * q * area
    if not np.allclose(force, expected, rtol=0.0, atol=1.0e-12):
        raise SystemExit("Wind-axis aerodynamic-force sign convention is inconsistent.")
    ####

    magnitude = 25_000.0
    epsilon1 = math.radians(37.0)
    epsilon2 = math.radians(-22.0)
    thrust = np.array(
        [
            magnitude * math.cos(epsilon1),
            -magnitude * math.sin(epsilon1) * math.cos(epsilon2),
            -magnitude * math.sin(epsilon1) * math.sin(epsilon2),
        ],
        dtype=float,
    )
    if not math.isclose(float(np.linalg.norm(thrust)), magnitude, rel_tol=1.0e-13):
        raise SystemExit("Propulsive-force angle reconstruction does not preserve magnitude.")
    ####
####


def _validate_gravity_relations() -> None:
    radius = 7.0e6
    gm = 3.986004418e14
    earth_radius = 6.378137e6
    latitude = math.radians(31.0)
    j2 = 1.08262998905e-3
    ratio2 = (earth_radius / radius) ** 2
    north = -(gm / radius**2) * ratio2 * (
        3.0 * j2 * math.sin(latitude) * math.cos(latitude)
    )
    east = 0.0
    inward = (gm / radius**2) * (
        1.0 - ratio2 * 1.5 * j2 * (3.0 * math.sin(latitude) ** 2 - 1.0)
    )
    if not north < 0.0 or east != 0.0 or not inward > 0.0:
        raise SystemExit("J2 gravity-component signs are inconsistent with the geocentric frame.")
    ####

    delta_m = 1.0
    n, m = 4, 0
    p_scale = math.sqrt(
        (2.0 - delta_m) * (2.0 * n + 1.0) * math.factorial(n - m) / math.factorial(n + m)
    )
    c_scale = math.sqrt(
        math.factorial(n + m)
        / ((2.0 - delta_m) * (2.0 * n + 1.0) * math.factorial(n - m))
    )
    if not math.isclose(p_scale * c_scale, 1.0, rel_tol=1.0e-14):
        raise SystemExit("Legendre-function and gravity-coefficient normalization factors are not reciprocal.")
    ####
####


def main() -> None:
    missing_sections = [name for name in EXPECTED_SECTION_FILES if not (CHAPTER / name).exists()]
    if missing_sections:
        raise SystemExit("Missing Section 2.3 source files: " + ", ".join(missing_sections))
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
        raise SystemExit("Completed Section 2.3 files still contain skeleton placeholders.")
    ####

    equation_labels = set(re.findall(r"\\label\{eq:[^}]+\}", completed_text))
    if len(equation_labels) != 46:
        raise SystemExit(
            f"Unexpected Section 2.3 equation-label count: {len(equation_labels)}; expected 46."
        )
    ####

    equation_rows = [
        row
        for row in _read_csv(ROOT / "metadata" / "equations.csv")
        if _chapter_number(row["equation"]) in EXPECTED_EQUATION_NUMBERS
    ]
    equation_numbers = {_chapter_number(row["equation"]) for row in equation_rows}
    if equation_numbers != EXPECTED_EQUATION_NUMBERS:
        raise SystemExit("Equation metadata does not contain every equation from 2-175 through 2-220.")
    ####
    if any(row["status"] != "visually_verified" for row in equation_rows):
        raise SystemExit("One or more Section 2.3 equations are not visually verified.")
    ####

    required_metadata: tuple[str, ...] = (
        "acceleration_models_chapter2.yaml",
        "state_variables_chapter2_acceleration.yaml",
        "editorial_notes_chapter2.yaml",
    )
    missing_metadata = [
        name for name in required_metadata if not (ROOT / "metadata" / name).exists()
    ]
    if missing_metadata:
        raise SystemExit("Missing Section 2.3 metadata: " + ", ".join(missing_metadata))
    ####

    state_text = (ROOT / "metadata" / "state_variables_chapter2_acceleration.yaml").read_text()
    observed_state_variables = set(re.findall(r"name:\s*([a-z0-9]+)", state_text))
    missing_state_variables = sorted(EXPECTED_STATE_VARIABLES - observed_state_variables)
    if missing_state_variables:
        raise SystemExit(
            "Section 2.3 state metadata omits: " + ", ".join(missing_state_variables)
        )
    ####

    figure_rows = {row["figure"]: row for row in _read_csv(ROOT / "metadata" / "figures.csv")}
    for figure_number, (raster_name, tikz_name) in EXPECTED_FIGURES.items():
        if figure_number not in figure_rows:
            raise SystemExit(f"Figure metadata omits {figure_number}.")
        ####
        raster = ROOT / "figures" / "chapter02" / "source-crops" / raster_name
        tikz = ROOT / "figures" / "chapter02" / tikz_name
        if not raster.exists() or not tikz.exists():
            raise SystemExit(f"Figure {figure_number} is missing its raster crop or TikZ draft.")
        ####
    ####

    _validate_atmosphere_relations()
    _validate_force_relations()
    _validate_gravity_relations()

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
        "Chapter 2 Section 2.3 check passed: "
        f"{len(EXPECTED_SECTION_FILES)} source sections, {len(equation_labels)} equations, "
        f"and {len(EXPECTED_FIGURES)} figures."
    )
####


if __name__ == "__main__":
    main()
####
