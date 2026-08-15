from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import run_files
from taoryx.trajectory import load_case_intent, load_family_catalog, render_fidelity_problems, resolve_case

ROOT = Path(__file__).resolve().parents[2]
CATALOG = ROOT / "verification" / "alpha2_family_catalog.yaml"
CASE = ROOT / "tests" / "fixtures" / "alpha2_case_contracts" / "case-t5-ladder.yaml"

pytestmark = pytest.mark.simple_aero


def _resolved_case():
    """Resolve the checked-in T5 case through the public Alpha2 catalog path."""

    return resolve_case(load_case_intent(CASE), load_family_catalog(CATALOG))
    ####


def test_t5_catalog_exposes_one_coherent_three_tier_family() -> None:
    family = load_family_catalog(CATALOG).family("simple_aero_ladder")

    assert family.fidelities == ("point_mass_3dof", "pseudo_6dof", "rigid_body_6dof")
    assert family.segment_graphs["ladder"]["segments"]
    assert family.parameter_map()["vehicle.mass.initial"].canonical_unit == "kg"
    ####


def test_t5_adapters_generate_parser_compatible_problems_for_all_tiers(tmp_path: Path) -> None:
    case = _resolved_case()
    problems = render_fidelity_problems(case, tmp_path / "problems")

    assert [problem.fidelity for problem in problems] == ["point_mass_3dof", "pseudo_6dof", "rigid_body_6dof"]
    assert problems[0].path.read_text(encoding="utf-8").count("*mode point-mass") == 1
    assert "*mode kinematic-6dof" in problems[1].path.read_text(encoding="utf-8")
    assert "*mode rigid-body-6dof" in problems[2].path.read_text(encoding="utf-8")
    for problem in problems:
        report = run_files(problem.path, output_dir=tmp_path / "runs" / problem.fidelity, max_steps=10_000, profile=GrammarProfile.TAORYX)
        assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
        assert report.results and report.results[0].completed
    ####


def test_t5_pseudo_bridge_preserves_common_translation(tmp_path: Path) -> None:
    problems = render_fidelity_problems(_resolved_case(), tmp_path / "problems")
    reports = [run_files(problem.path, output_dir=tmp_path / "runs" / problem.fidelity, max_steps=10_000, profile=GrammarProfile.TAORYX) for problem in problems[:2]]
    left = reports[0].results[0].states["1"]
    right = reports[1].results[0].states["1"]

    assert len(left) == len(right)
    for point, pseudo in zip(left, right, strict=True):
        for name in ("x", "y", "z", "xdt", "ydt", "zdt", "mass"):
            assert pseudo.named[name] == point.named[name]
    ####
