"""Common 3-DOF → kinematic 3+3 → rigid-body family ladder."""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

import pytest
import yaml

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import RunReport, run_files

ROOT = Path(__file__).resolve().parents[2]
LADDER = ROOT / "verification/fidelity_ladder.yaml"

pytestmark = pytest.mark.slow


@dataclass(frozen=True, slots=True)
class FidelityCase:
    """One metadata-driven family entry in the staged verification ladder."""

    identifier: str
    display_name: str
    point_mass_problem: Path
    rigid_body_problem: Path
    tables: tuple[Path, ...]
    rigid_tables: tuple[Path, ...]
    max_steps: int
    convergence_position_tolerance_m: float
    convergence_velocity_tolerance_m_s: float


def _cases() -> tuple[FidelityCase, ...]:
    payload = yaml.safe_load(LADDER.read_text(encoding="utf-8"))
    return tuple(
        FidelityCase(
            identifier=str(item["id"]),
            display_name=str(item["display_name"]),
            point_mass_problem=ROOT / str(item["point_mass_problem"]),
            rigid_body_problem=ROOT / str(item["rigid_body_problem"]),
            tables=tuple(ROOT / str(path) for path in item["tables"]),
            rigid_tables=tuple(ROOT / str(path) for path in item["rigid_tables"]),
            max_steps=int(item["max_steps"]),
            convergence_position_tolerance_m=float(item.get("convergence_position_tolerance_m", 1.0e-3)),
            convergence_velocity_tolerance_m_s=float(item.get("convergence_velocity_tolerance_m_s", 1.0e-6)),
        )
        for item in payload["families"]
    )


def _family_marker(identifier: str) -> pytest.MarkDecorator:
    marker = {"skywalker_x8": "x8"}.get(identifier, identifier)
    return getattr(pytest.mark, marker)


def _kinematic_problem(source: Path, destination: Path) -> Path:
    """Derive the bridge case without changing the source 3-DOF fixture."""

    lines = source.read_text(encoding="utf-8").splitlines()
    title_index = next(index for index, line in enumerate(lines) if line.startswith("*title"))
    lines[title_index + 1:title_index + 1] = [
        "*mode kinematic-6dof",
        "*runtime status attitude mode=lag roll-deg=0 pitch-deg=0 yaw-deg=0 lag-s=0.25 max-rate-deg-s=360",
    ]
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return destination


def _scaled_problem(source: Path, destination: Path, factor: float) -> Path:
    """Create a step-refined copy of a problem without changing its events."""

    if factor <= 0.0:
        raise ValueError("integration-step scale must be positive")
    text = source.read_text(encoding="utf-8")
    text = re.sub(
        r"(\bdt=)([0-9.eE+-]+)",
        lambda match: f"{match.group(1)}{float(match.group(2)) * factor:.16g}",
        text,
    )
    destination.write_text(text, encoding="utf-8")
    return destination


def _run(problem: Path, tables: tuple[Path, ...], output: Path, max_steps: int) -> RunReport:
    return run_files(problem, tables, output_dir=output, max_steps=max_steps, profile=GrammarProfile.TAORYX)


@pytest.mark.parametrize("case", tuple(pytest.param(case, id=case.identifier, marks=_family_marker(case.identifier)) for case in _cases()))
def test_each_family_progresses_through_the_three_dynamics_tiers(case: FidelityCase, tmp_path: Path) -> None:
    point_mass = _run(case.point_mass_problem, case.tables, tmp_path / "point-mass", case.max_steps)
    assert point_mass.exit_code == 0, [(item.code, item.message) for item in point_mass.diagnostics]
    assert point_mass.results and point_mass.results[0].completed

    kinematic_problem = _kinematic_problem(case.point_mass_problem, tmp_path / "kinematic.prb")
    kinematic = _run(kinematic_problem, case.tables, tmp_path / "kinematic", case.max_steps)
    assert kinematic.exit_code == 0, [(item.code, item.message) for item in kinematic.diagnostics]
    assert kinematic.results and kinematic.results[0].completed
    kinematic_history = kinematic.results[0].states["1"]
    point_history = point_mass.results[0].states["1"]
    assert len(kinematic_history) == len(point_history)
    for point_state, kinematic_state in zip(point_history, kinematic_history, strict=True):
        for name in ("x", "y", "z", "xdt", "ydt", "zdt"):
            if name in point_state.named and name in kinematic_state.named:
                assert kinematic_state.named[name] == pytest.approx(point_state.named[name], abs=1.0e-9)
    for state in kinematic_history:
        norm = sum(state.named[name] ** 2 for name in ("qw", "qx", "qy", "qz"))
        assert norm == pytest.approx(1.0, abs=1.0e-10)

    rigid = _run(case.rigid_body_problem, case.rigid_tables, tmp_path / "rigid-body", case.max_steps)
    assert rigid.exit_code == 0, [(item.code, item.message) for item in rigid.diagnostics]
    assert rigid.results and rigid.results[0].completed
    assert rigid.artifacts[0].vehicles["1"].dynamics.value == "rigid_body_6dof"


@pytest.mark.parametrize("case", tuple(pytest.param(case, id=case.identifier, marks=_family_marker(case.identifier)) for case in _cases()))
def test_each_family_lower_tiers_converge_under_step_refinement(case: FidelityCase, tmp_path: Path) -> None:
    """The 3-DOF and kinematic bridge remain stable when the step is halved."""

    base_point = _run(case.point_mass_problem, case.tables, tmp_path / "base-point", case.max_steps)
    half_point_problem = _scaled_problem(case.point_mass_problem, tmp_path / "half-point.prb", 0.5)
    half_point = _run(half_point_problem, case.tables, tmp_path / "half-point", case.max_steps * 2)
    base_kinematic_problem = _kinematic_problem(case.point_mass_problem, tmp_path / "base-kinematic.prb")
    half_kinematic_problem = _kinematic_problem(half_point_problem, tmp_path / "half-kinematic.prb")
    base_kinematic = _run(base_kinematic_problem, case.tables, tmp_path / "base-kinematic", case.max_steps)
    half_kinematic = _run(half_kinematic_problem, case.tables, tmp_path / "half-kinematic", case.max_steps * 2)
    for report in (base_point, half_point, base_kinematic, half_kinematic):
        assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
        assert report.results and report.results[0].completed

    for base_report, refined_report in ((base_point, half_point), (base_kinematic, half_kinematic)):
        base = base_report.results[0].states["1"][-1].named
        refined = refined_report.results[0].states["1"][-1].named
        for name in ("x", "y", "z", "xdt", "ydt", "zdt"):
            if name in base and name in refined:
                tolerance = (
                    case.convergence_position_tolerance_m
                    if name in {"x", "y", "z"}
                    else case.convergence_velocity_tolerance_m_s
                )
                assert abs(base[name] - refined[name]) < tolerance, (case.identifier, name, base[name], refined[name])
