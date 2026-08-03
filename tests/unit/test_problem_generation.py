from __future__ import annotations

from pathlib import Path

import yaml

from taoryx.language.diagnostics import Severity
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language.problem_parser import parse_problem_file, parse_problem_text
from tools.generate_problem_files import CATALOG, check_catalog, render_catalog, render_scenario


def test_problem_catalog_is_current() -> None:
    """All tracked generated problem files are reproducible from metadata."""

    generated = check_catalog(CATALOG)
    catalog = yaml.safe_load(CATALOG.read_text(encoding="utf-8"))
    assert len(generated) == len(catalog["scenarios"])
    assert all(path.exists() for path in generated)
    assert all(path.suffix == ".prb" for path in generated)
####


def test_problem_generation_is_deterministic(tmp_path: Path) -> None:
    """Rendering the catalog does not depend on process state or timestamps."""

    first = tuple(path.read_text(encoding="utf-8") for path in check_catalog(CATALOG))
    rendered_paths = render_catalog(CATALOG)
    second = tuple(path.read_text(encoding="utf-8") for path in rendered_paths)
    assert first == second
    assert all(path.is_file() for path in rendered_paths)
####


def test_generated_problems_have_no_successor_parse_errors() -> None:
    """Every generated native problem is syntactically valid in its profile."""

    for problem in check_catalog(CATALOG):
        parsed = parse_problem_file(problem, profile=GrammarProfile.TAORYX)
        errors = [
            diagnostic
            for diagnostic in parsed.diagnostics
            if diagnostic.severity is Severity.ERROR
        ]

        assert not errors, (problem, errors)
####


def test_nested_actuator_metadata_is_not_serialized_as_a_problem_expression() -> None:
    """Structured actuator provenance stays in YAML, not scalar `.prb` fields."""

    catalog = yaml.safe_load(CATALOG.read_text(encoding="utf-8"))
    scenario = next(item for item in catalog["scenarios"] if item["id"] == "hummingbird-generated-canonical-hover-6dof")
    rendered = render_scenario(catalog, scenario)

    assert "maximum-moment=1 maximum-body-rate-deg-s=720" in rendered
    assert "dynamics={" not in rendered
    assert "first_order_motor_lag" not in rendered

    parsed = parse_problem_text(rendered, profile=GrammarProfile.TAORYX)

    assert not parsed.diagnostics
####
