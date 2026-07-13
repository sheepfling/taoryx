from __future__ import annotations

import pytest

from taoryx.equations import (
    Equation,
    EquationImplementationStatus,
    EquationRegistry,
    implementation_summary,
    load_equation_registry,
    registry,
)


def test_default_registry_loads_canonical_equation_catalog() -> None:
    assert len(registry) == 326

    first = registry.get("1-1")
    assert first.latex_label == "eq:force-equation"
    assert first.section == "Trajectory Simulation"
    assert first.source_manual_page == "1-2"
    assert registry.get_by_label("eq:force-equation") is first

    last = registry.get("4-8")
    assert last.latex_label == "eq:optimization-central-difference"
    assert last.is_implemented is False
####


def test_registry_rejects_duplicate_identifier_and_label() -> None:
    registry = EquationRegistry()
    first = Equation(
        identifier="1-1",
        latex_label="eq:force-equation",
        section="Trajectory Simulation",
        source_manual_page="1-2",
        source_pdf_page=19,
        source_pdf_sha256="sha",
        reconstructed_page="1-1",
        reconstructed_section="Trajectory Simulation",
        tex_file="manual/chapters/01_introduction.tex",
        tex_line_start=38,
        tex_line_end=44,
        tex_label_line=43,
        latex_snippet_sha256="snippet",
        transcription_status="visually_verified",
        verification_scope="test",
        validation_suite="tests/unit/test_equations.py",
        code_implementation_status="not yet mapped to an executable mathematics implementation",
        notes="",
    )
    second = Equation(
        identifier="1-2",
        latex_label="eq:force-equation",
        section="Trajectory Simulation",
        source_manual_page="1-2",
        source_pdf_page=19,
        source_pdf_sha256="sha",
        reconstructed_page="1-2",
        reconstructed_section="Trajectory Simulation",
        tex_file="manual/chapters/01_introduction.tex",
        tex_line_start=58,
        tex_line_end=61,
        tex_label_line=60,
        latex_snippet_sha256="snippet2",
        transcription_status="visually_verified",
        verification_scope="test",
        validation_suite="tests/unit/test_equations.py",
        code_implementation_status="not yet mapped to an executable mathematics implementation",
        notes="",
    )

    registry.register(first)

    with pytest.raises(ValueError, match="already registered"):
        registry.register(first)

    with pytest.raises(ValueError, match="label already registered"):
        registry.register(second)
####


def test_load_equation_registry_round_trips_provenance_csv() -> None:
    loaded = load_equation_registry()

    assert loaded.get("2-1").tex_file == "manual/chapters/chapter02/01_02_ecic.tex"
    assert loaded.get("2-1").tex_label_line == 14
    assert loaded.get_by_label("eq:earth-rotation-rate").identifier == "2-2"
    assert loaded.all()[0].identifier == "1-1"
    assert loaded.all()[-1].identifier == "4-8"
####


def test_registry_reports_implementation_progress() -> None:
    summary = implementation_summary(registry)

    assert summary["total"] == 326
    assert summary[EquationImplementationStatus.IMPLEMENTED.value] == 254
    assert summary[EquationImplementationStatus.NOT_IMPLEMENTED.value] == 72
    assert summary[EquationImplementationStatus.PARTIAL.value] == 0
    assert summary[EquationImplementationStatus.UNKNOWN.value] == 0
    assert summary["implemented_ratio_numerator"] == 254
    assert summary["implemented_ratio_denominator"] == 326
####


def test_selected_equations_are_marked_implemented() -> None:
    implemented_ids = {
        "2-1",
        "2-6",
        "2-7",
        "2-18",
        "2-22",
        "2-30",
        "2-49",
        "2-50",
        "2-61",
        "2-93",
        "2-96",
        "2-246",
        "2-250",
    }

    assert all(registry.get(identifier).is_implemented for identifier in implemented_ids)
    assert registry.get("2-62").is_implemented
    assert registry.get("2-90").is_implemented
    assert registry.get("2-97").is_implemented
    assert registry.get("2-107").is_implemented
    assert registry.get("2-118").is_implemented
    assert registry.get("2-153").is_implemented
    assert registry.get("2-176").is_implemented
    assert registry.get("2-185").is_implemented
    assert registry.get("2-195").is_implemented
    assert registry.get("2-201").is_implemented
    assert registry.get("2-210").is_implemented
    assert registry.get("2-220").is_implemented
    assert registry.get("2-221").is_implemented
    assert registry.get("2-233").is_implemented
    assert registry.get("2-245").is_implemented
    assert registry.get("2-251").is_implemented
    assert registry.get("2-260").is_implemented
    assert registry.get("2-265").is_implemented
    assert registry.get("2-170").is_implemented
    assert registry.get("2-174").is_implemented
    assert registry.get("2-175").is_implemented
    assert registry.get("2-196").is_implemented
    assert registry.get("2-200").is_implemented
    assert registry.get("2-123").is_implemented
    assert registry.get("2-152").is_implemented
    assert registry.get("2-140").is_implemented
    assert not registry.get("2-141").is_implemented
####
