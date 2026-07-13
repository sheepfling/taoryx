from __future__ import annotations

import csv
import re
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

ROOT = Path(__file__).resolve().parents[1]
CHAPTER = ROOT / "chapters" / "chapter02"
WRAPPER = ROOT / "chapters" / "02_methods.tex"
EXPECTED_SECTION_FILES: tuple[str, ...] = (
    "02_00_equations_of_motion.tex",
    "02_01_numerical_integration.tex",
    "02_02_rail_launches.tex",
    "02_03_state_rates.tex",
    "02_04_ground_speed.tex",
    "02_05_flight_path_rates.tex",
    "02_06_specific_load.tex",
)
EXPECTED_EQUATION_NUMBERS: set[int] = set(range(97, 175))
EXPECTED_STATE_VARIABLES: set[str] = {
    "fuel",
    "mass",
    "mdt",
    "wt",
    "plength",
    "plmark",
    "plseg",
    "longdt",
    "latgcdt",
    "latgddt",
    "altdt",
    "range",
    "grseg",
    "grmark",
    "vgr",
    "psigcdt",
    "psigddt",
    "gamgcdt",
    "gamgddt",
    "nx",
    "ny",
    "nz",
    "ntotal",
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


def _validate_rotating_frame_components() -> None:
    position: NDArray[np.float64] = np.array([2.1e7, -1.3e7, 8.0e6], dtype=float)
    velocity: NDArray[np.float64] = np.array([1200.0, 17500.0, -900.0], dtype=float)
    force: NDArray[np.float64] = np.array([3100.0, -4200.0, 1700.0], dtype=float)
    mass = 850.0
    omega = 7.292115e-5
    omega_vector: NDArray[np.float64] = np.array([0.0, 0.0, omega], dtype=float)

    vector_form = (
        force / mass
        - 2.0 * np.cross(omega_vector, velocity)
        - np.cross(omega_vector, np.cross(omega_vector, position))
    )
    component_form: NDArray[np.float64] = np.array(
        [
            force[0] / mass + omega * (2.0 * velocity[1] + omega * position[0]),
            force[1] / mass + omega * (-2.0 * velocity[0] + omega * position[1]),
            force[2] / mass,
        ],
        dtype=float,
    )
    if not np.allclose(vector_form, component_form, rtol=1e-13, atol=1e-13):
        raise SystemExit("Rotating-frame vector and ECFC component equations disagree.")
    ####
####


def _validate_runge_kutta_formula() -> None:
    step = 0.1
    state = 1.0

    def derivative(_time: float, value: float) -> float:
        return value
    ####

    k1 = derivative(0.0, state)
    k2 = derivative(0.5 * step, state + 0.5 * step * k1)
    k3 = derivative(0.5 * step, state + 0.5 * step * k2)
    k4 = derivative(step, state + step * k3)
    integrated = state + step * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
    expected = float(np.exp(step))
    if abs(integrated - expected) > 1.0e-6:
        raise SystemExit("Fourth-order Runge-Kutta reconstruction failed its reference check.")
    ####
####


def main() -> None:
    missing_sections = [name for name in EXPECTED_SECTION_FILES if not (CHAPTER / name).exists()]
    if missing_sections:
        raise SystemExit("Missing Section 2.2 source files: " + ", ".join(missing_sections))
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
        raise SystemExit("Completed Section 2.2 files still contain skeleton placeholders.")
    ####

    equation_labels = set(re.findall(r"\\label\{eq:[^}]+\}", completed_text))
    if len(equation_labels) != 78:
        raise SystemExit(
            f"Unexpected Section 2.2 equation-label count: {len(equation_labels)}; expected 78."
        )
    ####

    equation_rows = [
        row
        for row in _read_csv(ROOT / "metadata" / "equations.csv")
        if _chapter_number(row["equation"]) in EXPECTED_EQUATION_NUMBERS
    ]
    equation_numbers = {_chapter_number(row["equation"]) for row in equation_rows}
    if equation_numbers != EXPECTED_EQUATION_NUMBERS:
        raise SystemExit("Equation metadata does not contain every equation from 2-97 through 2-174.")
    ####
    if any(row["status"] != "visually_verified" for row in equation_rows):
        raise SystemExit("One or more Section 2.2 equations are not visually verified.")
    ####

    required_metadata: tuple[str, ...] = (
        "equations_of_motion_chapter2.yaml",
        "state_variables_chapter2_dynamics.yaml",
        "editorial_notes_chapter2.yaml",
    )
    missing_metadata = [
        name for name in required_metadata if not (ROOT / "metadata" / name).exists()
    ]
    if missing_metadata:
        raise SystemExit("Missing Section 2.2 metadata: " + ", ".join(missing_metadata))
    ####

    state_text = (ROOT / "metadata" / "state_variables_chapter2_dynamics.yaml").read_text()
    observed_state_variables = set(re.findall(r"name:\s*([a-z0-9]+)", state_text))
    missing_state_variables = sorted(EXPECTED_STATE_VARIABLES - observed_state_variables)
    if missing_state_variables:
        raise SystemExit(
            "Section 2.2 state metadata omits: " + ", ".join(missing_state_variables)
        )
    ####

    _validate_rotating_frame_components()
    _validate_runge_kutta_formula()

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
        "Chapter 2 Section 2.2 check passed: "
        f"{len(EXPECTED_SECTION_FILES)} source sections and {len(equation_labels)} equations."
    )
####


if __name__ == "__main__":
    main()
####
