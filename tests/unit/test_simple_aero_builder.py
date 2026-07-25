from __future__ import annotations

import json

import pytest

from taoryx.language import parse_problem_text
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import run_files
from taoryx.simple_aero_builder import FixedLD3DOFParameters, SimpleAeroTrajectoryBuilder, build_fixed_ld_3dof


def test_fixed_ld_builder_derives_heading_and_fixed_ld_coefficients() -> None:
    build = build_fixed_ld_3dof(
        vbo_m_s=900.0,
        apogee_altitude_m=20_000.0,
        target_bearing_deg=90.0,
        initial_heading_offset_deg=12.0,
        drag_coefficient=0.025,
        lift_to_drag=5.0,
    )

    assert build.derived.initial_heading_deg == pytest.approx(102.0)
    assert build.derived.lift_coefficient == pytest.approx(0.125)
    assert build.derived.boost_duration_s == pytest.approx(8.5)
    assert "*segment 1 powered-ascent" in build.problem_text
    assert "*segment 4 terminal-pronav" in build.problem_text
    assert "*fly propnav=2" in build.problem_text
    ####


def test_fixed_ld_builder_output_is_parser_compatible_and_writes_manifest(tmp_path) -> None:
    build = SimpleAeroTrajectoryBuilder(
        FixedLD3DOFParameters(
            scenario_id="x8-simple_aero-smoke",
            vehicle_id="skywalker-x8",
            family="crossrange",
            vbo_m_s=250.0,
            apogee_altitude_m=2_000.0,
            pitch_over_angle_deg=35.0,
            target_range_m=5_000.0,
        )
    ).build()

    document = parse_problem_text(build.problem_text, profile=GrammarProfile.TAORYX)
    assert not [diagnostic for diagnostic in document.diagnostics if diagnostic.severity.value == "error"]
    assert len(document.problems[0].trajectories) == 2

    problem_path = tmp_path / "mission.prb"
    manifest_path = tmp_path / "mission.manifest.json"
    build.write(problem_path, manifest_path)

    assert problem_path.read_text(encoding="utf-8") == build.problem_text
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["parameters"]["vbo_m_s"] == 250.0
    assert manifest["derived"]["initial_heading_deg"] == pytest.approx(90.0)
    ####


def test_fixed_ld_builder_runs_through_the_taoryx_point_mass_engine(tmp_path) -> None:
    problem_path = tmp_path / "mission.prb"
    build_fixed_ld_3dof(
        vbo_m_s=250.0,
        apogee_altitude_m=2_000.0,
        pitch_over_angle_deg=35.0,
        target_range_m=5_000.0,
    ).write(problem_path)

    report = run_files(
        problem_path,
        output_dir=tmp_path / "run",
        max_steps=5_000,
        profile=GrammarProfile.TAORYX,
    )

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    assert report.results[0].completed
    assert report.results[0].stop_reason == "stop_condition"
    ####


@pytest.mark.parametrize(
    "overrides",
    (
        {"initial_speed_m_s": 900.0, "vbo_m_s": 900.0},
        {"initial_altitude_m": 2_000.0, "apogee_altitude_m": 2_000.0},
        {"mass_flow_kg_s": 1.0, "thrust_n": 0.0},
    ),
)
def test_fixed_ld_builder_rejects_inconsistent_profile(overrides: dict[str, float]) -> None:
    with pytest.raises(ValueError):
        FixedLD3DOFParameters(**overrides)
    ####
