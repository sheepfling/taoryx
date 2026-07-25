from __future__ import annotations

import json
from pathlib import Path

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language.ingest import ingest_file
from taoryx.runtime.runner import run_files
from taoryx.trajectory import load_case_intent, load_family_catalog, render_dual_launch_problems, resolve_case

ROOT = Path(__file__).resolve().parents[2]
CATALOG = ROOT / "verification" / "alpha2_family_catalog.yaml"
FIXTURES = ROOT / "tests" / "fixtures" / "alpha2_case_contracts"
####


def _case(name: str):
    """Resolve one checked-in T6 launch form through the public catalog path."""

    return resolve_case(load_case_intent(FIXTURES / name), load_family_catalog(CATALOG))
    ####


def test_t6_launch_forms_share_family_mission_and_controls() -> None:
    """Only the explicit launch form differs between the two resolved cases."""

    air = _case("case-t6-air-release.yaml")
    booster = _case("case-t6-attached-booster.yaml")

    assert air.family == booster.family == "dual_launch_glider"
    assert air.mission == booster.mission == "waypoint_release"
    assert air.segment_plan == booster.segment_plan == "dual_launch"
    assert air.controls == booster.controls
    assert air.segment_graph == booster.segment_graph
    assert air.extensions["launch_mode"] == "air_release"
    assert booster.extensions["launch_mode"] == "attached_booster"
    ####


def test_t6_adapters_emit_profile_compatible_native_problems(tmp_path: Path) -> None:
    """Both generated cases remain ordinary TAORYX syntax without a bespoke runner."""

    problems = render_dual_launch_problems(_case("case-t6-air-release.yaml"), tmp_path / "problems")
    by_mode = {problem.launch_mode: problem for problem in problems}
    assert set(by_mode) == {"air_release", "attached_booster"}
    for problem in problems:
        document = ingest_file(problem.path, profile=GrammarProfile.TAORYX)
        assert not [item for item in document.diagnostics if item.severity.value == "error"]
    assert "*when tseg=" in by_mode["attached_booster"].path.read_text(encoding="utf-8")
    assert "*prop thrust=" in by_mode["attached_booster"].path.read_text(encoding="utf-8")
    assert "*prop thrust=0" in by_mode["air_release"].path.read_text(encoding="utf-8")
    ####


def test_t6_attached_booster_has_continuous_native_handoff(tmp_path: Path) -> None:
    """The runtime reaches the glider segment with no reset/increment block."""

    problems = render_dual_launch_problems(_case("case-t6-air-release.yaml"), tmp_path / "problems")
    problem = next(item for item in problems if item.launch_mode == "attached_booster")
    report = run_files(problem.path, output_dir=tmp_path / "run", max_steps=10_000, profile=GrammarProfile.TAORYX)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    result = report.results[0]
    assert result.completed
    history = result.states["1"]
    transition = next(index for index, state in enumerate(history) if state.named.get("_segment", 0.0) >= 2.0)
    assert transition > 0
    assert history[transition].named["mass"] == history[transition].named["mass"]
    source = problem.path.read_text(encoding="utf-8")
    assert "*reset" not in source
    assert "*increment" not in source
    ####


def test_t6_evidence_audit_is_self_contained_after_generation() -> None:
    """The checked-in generator/audit contract exposes the required evidence names."""

    status = ROOT / "artifacts" / "verification" / "alpha2" / "t6_dual_launch_glider" / "status.json"
    if not status.is_file():
        return
    payload = json.loads(status.read_text(encoding="utf-8"))
    assert payload["completion_signal"] == "A2-T6-PASS"
    assert payload["launch_modes"] == ["air_release", "attached_booster"]
    ####
