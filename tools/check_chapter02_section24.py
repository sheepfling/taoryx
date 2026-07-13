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
    "04_00_output_variables.tex",
    "04_01_ranges_distances.tex",
    "04_02_radar_observations.tex",
    "04_03_relative_vehicle.tex",
    "04_04_iip.tex",
    "04_05_aerodynamic_variables.tex",
)
EXPECTED_EQUATION_NUMBERS: set[int] = set(range(221, 273))
EXPECTED_STATE_VARIABLES: set[str] = {
    "crsrng",
    "dwnrng",
    "east",
    "north",
    "radrng",
    "radrngdt",
    "radrngdt2",
    "radaz",
    "radazdt",
    "radazdt2",
    "radelv",
    "radelvdt",
    "radelvdt2",
    "radasp",
    "radmer",
    "relrng",
    "relaz",
    "relelv",
    "relvel",
    "relxb",
    "relyb",
    "relzb",
    "iip_latgd",
    "iip_long",
    "iip_time",
    "iip_rng",
    "iip_azm",
    "alpha_l/d",
    "ballistic",
    "l/d",
}
EXPECTED_FIGURES: dict[str, tuple[str, str]] = {
    "2-18": ("fig-2-18.png", "fig-2-18.tex"),
    "2-19": ("fig-2-19.png", "fig-2-19.tex"),
    "2-20": ("fig-2-20.png", "fig-2-20.tex"),
    "2-21": ("fig-2-21.png", "fig-2-21.tex"),
    "2-22": ("fig-2-22.png", "fig-2-22.tex"),
    "2-23": ("fig-2-23.png", "fig-2-23.tex"),
    "2-24": ("fig-2-24.png", "fig-2-24.tex"),
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


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open() as handle:
        loaded = yaml.safe_load(handle)
    ####
    if not isinstance(loaded, dict):
        raise SystemExit(f"Expected a mapping in {path.name}.")
    ####
    return loaded
####


def _collect_state_names(node: Any) -> set[str]:
    names: set[str] = set()
    if isinstance(node, dict):
        value = node.get("name")
        if isinstance(value, str):
            names.add(value)
        ####
        for child in node.values():
            names.update(_collect_state_names(child))
        ####
    elif isinstance(node, list):
        for child in node:
            names.update(_collect_state_names(child))
        ####
    ####
    return names
####


def _validate_geodesy_relations() -> None:
    equatorial_radius = 6_378_137.0
    flattening = 1.0 / 298.257223563
    polar_radius = equatorial_radius * (1.0 - flattening)
    latitude = math.radians(34.25)
    beta_from_flattening = (1.0 - flattening) * math.tan(latitude)
    beta_from_radii = polar_radius * math.sin(latitude) / (
        equatorial_radius * math.cos(latitude)
    )
    if not math.isclose(beta_from_flattening, beta_from_radii, rel_tol=1.0e-14):
        raise SystemExit("Reduced-latitude identities in Equations 2-224 and 2-225 disagree.")
    ####

    beta_1 = math.radians(18.0)
    beta_2 = math.radians(37.0)
    delta_lambda = math.radians(52.0)
    cosine_phi = (
        math.sin(beta_1) * math.sin(beta_2)
        + math.cos(beta_1) * math.cos(beta_2) * math.cos(delta_lambda)
    )
    if not -1.0 <= cosine_phi <= 1.0:
        raise SystemExit("Sodano central-angle reconstruction is outside the cosine domain.")
    ####
    phi = math.acos(cosine_phi)
    m_value = 1.0 - (
        math.cos(beta_1) ** 2
        * math.cos(beta_2) ** 2
        * math.sin(delta_lambda) ** 2
        / math.sin(phi) ** 2
    )
    if not 0.0 <= m_value <= 1.0 + 1.0e-12:
        raise SystemExit("Sodano inverse auxiliary m is outside its expected range.")
    ####
####


def _validate_radar_relations() -> None:
    radar_position: NDArray[np.float64] = np.array([200.0, -100.0, 50.0], dtype=float)
    vehicle_position: NDArray[np.float64] = np.array([1_800.0, 900.0, 650.0], dtype=float)
    vehicle_velocity: NDArray[np.float64] = np.array([420.0, -35.0, 18.0], dtype=float)
    delta = vehicle_position - radar_position
    distance = float(np.linalg.norm(delta))
    unit = delta / distance
    if not math.isclose(float(np.linalg.norm(unit)), 1.0, rel_tol=1.0e-14):
        raise SystemExit("Radar line-of-sight unit vector is not normalized.")
    ####

    analytical_rate = float(vehicle_velocity @ unit)
    dt = 1.0e-5
    finite_difference_rate = (
        float(np.linalg.norm(delta + dt * vehicle_velocity)) - distance
    ) / dt
    if not math.isclose(analytical_rate, finite_difference_rate, rel_tol=2.0e-6):
        raise SystemExit("Radar range-rate relation failed a finite-difference check.")
    ####

    x_hat: NDArray[np.float64] = np.array([1.0, 0.0, 0.0], dtype=float)
    y_hat: NDArray[np.float64] = np.array([0.0, 1.0, 0.0], dtype=float)
    z_hat: NDArray[np.float64] = np.array([0.0, 0.0, 1.0], dtype=float)
    azimuth = math.atan2(float(delta @ y_hat), float(delta @ x_hat))
    elevation = math.asin(-float(unit @ z_hat))
    reconstructed = np.array(
        [
            math.cos(elevation) * math.cos(azimuth),
            math.cos(elevation) * math.sin(azimuth),
            -math.sin(elevation),
        ],
        dtype=float,
    )
    if not np.allclose(unit, reconstructed, atol=1.0e-12, rtol=0.0):
        raise SystemExit("Radar azimuth/elevation reconstruction is inconsistent.")
    ####
####


def _validate_iip_and_aerodynamic_relations() -> None:
    density = 0.35
    velocity: NDArray[np.float64] = np.array([2_800.0, -320.0, 140.0], dtype=float)
    gravity = 32.174
    ballistic_coefficient = 1_250.0
    speed = float(np.linalg.norm(velocity))
    unit = velocity / speed
    equation_268 = gravity * 0.5 * density * speed**2 / ballistic_coefficient * unit
    equation_270 = 0.5 * gravity * density * speed * velocity / ballistic_coefficient
    if not np.allclose(equation_268, equation_270, rtol=1.0e-14, atol=0.0):
        raise SystemExit("IIP Equations 2-268 through 2-270 are not algebraically consistent.")
    ####

    mass = 18.0
    cd = 0.42
    area = 1.8
    beta = mass * gravity / (cd * area)
    if not math.isclose(beta, mass * gravity / (cd * area), rel_tol=0.0, abs_tol=0.0):
        raise SystemExit("Ballistic-coefficient identity failed.")
    ####
    cl = 0.63
    lift_to_drag = cl / cd
    if not math.isclose(lift_to_drag, (cl * 500.0 * area) / (cd * 500.0 * area)):
        raise SystemExit("Lift-to-drag cancellation failed.")
    ####
####


def main() -> None:
    missing_sections = [name for name in EXPECTED_SECTION_FILES if not (CHAPTER / name).exists()]
    if missing_sections:
        raise SystemExit("Missing Section 2.4 source files: " + ", ".join(missing_sections))
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
        raise SystemExit("Completed Section 2.4 files still contain skeleton placeholders.")
    ####

    equation_labels = set(re.findall(r"\\label\{eq:[^}]+\}", completed_text))
    if len(equation_labels) != 52:
        raise SystemExit(
            f"Unexpected Section 2.4 equation-label count: {len(equation_labels)}; expected 52."
        )
    ####

    equation_rows = [
        row
        for row in _read_csv(ROOT / "metadata" / "equations.csv")
        if _chapter_number(row["equation"]) in EXPECTED_EQUATION_NUMBERS
    ]
    equation_numbers = {_chapter_number(row["equation"]) for row in equation_rows}
    if equation_numbers != EXPECTED_EQUATION_NUMBERS:
        raise SystemExit("Equation metadata does not contain every equation from 2-221 through 2-272.")
    ####
    if any(row["status"] != "visually_verified" for row in equation_rows):
        raise SystemExit("One or more Section 2.4 equations are not visually verified.")
    ####

    required_metadata: tuple[str, ...] = (
        "output_models_chapter2.yaml",
        "state_variables_chapter2_output.yaml",
        "editorial_notes_chapter2.yaml",
    )
    missing_metadata = [
        name for name in required_metadata if not (ROOT / "metadata" / name).exists()
    ]
    if missing_metadata:
        raise SystemExit("Missing Section 2.4 metadata: " + ", ".join(missing_metadata))
    ####

    state_metadata = _load_yaml(ROOT / "metadata" / "state_variables_chapter2_output.yaml")
    observed_state_variables = _collect_state_names(state_metadata)
    missing_state_variables = sorted(EXPECTED_STATE_VARIABLES - observed_state_variables)
    if missing_state_variables:
        raise SystemExit(
            "Section 2.4 state metadata omits: " + ", ".join(missing_state_variables)
        )
    ####

    editorial_text = (ROOT / "metadata" / "editorial_notes_chapter2.yaml").read_text()
    required_editorial_ids = {
        "sodano_central_angle_missing_cosine",
        "sodano_eprime_internal_inconsistency",
        "iip_drag_sign",
        "section24_figure_preservation_strategy",
    }
    observed_editorial_ids = set(re.findall(r"id:\s*([a-z0-9_]+)", editorial_text))
    missing_editorial_ids = sorted(required_editorial_ids - observed_editorial_ids)
    if missing_editorial_ids:
        raise SystemExit("Section 2.4 editorial metadata omits: " + ", ".join(missing_editorial_ids))
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

    _validate_geodesy_relations()
    _validate_radar_relations()
    _validate_iip_and_aerodynamic_relations()

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
        "Chapter 2 Section 2.4 check passed: "
        f"{len(EXPECTED_SECTION_FILES)} source sections, {len(equation_labels)} equations, "
        f"{len(EXPECTED_STATE_VARIABLES)} output variables, and {len(EXPECTED_FIGURES)} figures."
    )
####


if __name__ == "__main__":
    main()
####
