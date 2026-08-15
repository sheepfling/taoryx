from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.language import parse_problem_text
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import run_files
from taoryx.simple_aero_builder import build_fixed_ld_from_resolved_case
from taoryx.trajectory import load_case_intent, load_family_catalog, resolve_case
from taoryx.trajectory.resolution import ResolutionError

ROOT = Path(__file__).resolve().parents[2]
CATALOG = ROOT / "verification" / "alpha2_family_catalog.yaml"
CASES = ROOT / "tests" / "fixtures" / "alpha2_case_contracts"

pytestmark = pytest.mark.simple_aero


def _case(name: str):
    catalog = load_family_catalog(CATALOG)
    return resolve_case(load_case_intent(CASES / name), catalog)


def test_simple_aero_resolved_case_contains_data_selected_graph_and_cutoff() -> None:
    case = _case("case-t4-baseline.yaml")

    assert [item["id"] for item in case.segment_graph["segments"]] == [
        "powered_ascent",
        "ballistic_coast",
        "bank_maneuver",
        "terminal_pronav",
    ]
    assert case.extensions["cutoff_mode"] == "physical"
    assert case.parameters["vehicle.booster.mass_flow"].unit == "kg/s"
    assert case.recompute_identity() == case.identity_sha256
    ####


@pytest.mark.parametrize(
    ("case_name", "active_comment"),
    (
        ("case-t4-baseline.yaml", "mass-flow depletion is the active source condition"),
        ("case-t4-high-thrust.yaml", "requested speed is the active condition"),
    ),
)
def test_resolved_simple_aero_case_generates_parser_compatible_native_problem(case_name: str, active_comment: str) -> None:
    build = build_fixed_ld_from_resolved_case(_case(case_name))
    document = parse_problem_text(build.problem_text, profile=GrammarProfile.TAORYX)

    assert not [item for item in document.diagnostics if item.severity.value == "error"]
    assert build.problem_text.count("*segment ") == 5
    assert active_comment in build.problem_text
    assert "# event physical_burnout:" in build.problem_text
    assert "# event commanded_cutoff:" in build.problem_text
    ####


def test_commanded_cutoff_case_executes_all_reusable_segments(tmp_path: Path) -> None:
    build = build_fixed_ld_from_resolved_case(_case("case-t4-high-thrust.yaml"))
    problem = tmp_path / "case.prb"
    build.write(problem)
    report = run_files(problem, output_dir=tmp_path / "run", max_steps=10_000, profile=GrammarProfile.TAORYX)

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = report.results[0].states["1"]
    assert sorted({int(float(state.named["_segment"])) for state in history}) == [1, 2, 3, 4]
    assert 15.0 < history[-1].time <= 25.0
    ####


def test_invalid_simple_aero_case_fails_closed_at_resolution() -> None:
    with pytest.raises(ResolutionError, match="out-of-range"):
        _case("case-t4-invalid.yaml")
    ####


def test_resolved_case_generation_is_deterministic() -> None:
    case = _case("case-t4-baseline.yaml")

    left = build_fixed_ld_from_resolved_case(case)
    right = build_fixed_ld_from_resolved_case(case)

    assert left.problem_text == right.problem_text
    assert left.derived.model_dump(mode="json") == right.derived.model_dump(mode="json")
    ####
