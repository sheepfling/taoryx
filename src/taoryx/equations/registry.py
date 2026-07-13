"""Stable identifiers for equations referenced by the manual and examples.

The canonical equation catalog lives in ``metadata/equations_provenance.csv``.
This module loads that registry into a small typed in-memory index so the rest
of the package can look up equations by canonical TAOS number or LaTeX label.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_PROVENANCE_PATH = ROOT / "metadata" / "equations_provenance.csv"


@dataclass(frozen=True, slots=True)
class Equation:
    """Metadata for one numbered equation."""

    identifier: str
    latex_label: str
    section: str
    source_manual_page: str
    source_pdf_page: int
    source_pdf_sha256: str
    reconstructed_page: str
    reconstructed_section: str
    tex_file: str
    tex_line_start: int
    tex_line_end: int
    tex_label_line: int
    latex_snippet_sha256: str
    transcription_status: str
    verification_scope: str
    validation_suite: str
    code_implementation_status: str
    notes: str = ""
####

    @property
    def is_implemented(self) -> bool:
        """Return whether the registry marks this equation as executable."""

        return self.code_implementation_status.startswith("implemented")
    ####


class EquationRegistry:
    """An in-memory registry with duplicate and missing-ID protection."""

    def __init__(self) -> None:
        self._equations: dict[str, Equation] = {}
        self._equations_by_label: dict[str, Equation] = {}
    ####

    def register(self, equation: Equation) -> None:
        if equation.identifier in self._equations:
            raise ValueError(f"equation already registered: {equation.identifier}")
        ####
        if equation.latex_label in self._equations_by_label:
            raise ValueError(f"equation label already registered: {equation.latex_label}")
        ####
        self._equations[equation.identifier] = equation
        self._equations_by_label[equation.latex_label] = equation
    ####

    def get(self, identifier: str) -> Equation:
        try:
            return self._equations[identifier]
        except KeyError as error:
            raise KeyError(f"unknown equation: {identifier}") from error
        ####
    ####

    def get_by_label(self, latex_label: str) -> Equation:
        try:
            return self._equations_by_label[latex_label]
        except KeyError as error:
            raise KeyError(f"unknown equation label: {latex_label}") from error
        ####
    ####

    def all(self) -> tuple[Equation, ...]:
        return tuple(self._equations.values())
    ####

    def __len__(self) -> int:
        return len(self._equations)
    ####

    def __contains__(self, identifier: object) -> bool:
        return identifier in self._equations
    ####


def _parse_equation_row(row: dict[str, str]) -> Equation:
    return Equation(
        identifier=row["equation"],
        latex_label=row["latex_label"],
        section=row["section"],
        source_manual_page=row["source_manual_page"],
        source_pdf_page=int(row["source_pdf_page"]),
        source_pdf_sha256=row["source_pdf_sha256"],
        reconstructed_page=row["reconstructed_page"],
        reconstructed_section=row["reconstructed_section"],
        tex_file=row["tex_file"],
        tex_line_start=int(row["tex_line_start"]),
        tex_line_end=int(row["tex_line_end"]),
        tex_label_line=int(row["tex_label_line"]),
        latex_snippet_sha256=row["latex_snippet_sha256"],
        transcription_status=row["transcription_status"],
        verification_scope=row["verification_scope"],
        validation_suite=row["validation_suite"],
        code_implementation_status=row["code_implementation_status"],
        notes=row["notes"],
    )
####


def load_equation_registry(provenance_path: Path = DEFAULT_PROVENANCE_PATH) -> EquationRegistry:
    """Load the canonical equation registry from the provenance CSV."""

    registry = EquationRegistry()
    if not provenance_path.exists():
        return registry
    ####
    with provenance_path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            registry.register(_parse_equation_row(row))
        ####
    ####
    return registry
####


registry = load_equation_registry()
####
