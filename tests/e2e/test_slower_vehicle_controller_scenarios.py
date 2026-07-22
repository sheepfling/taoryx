from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from taoryx.language import GrammarProfile
from taoryx.runtime.runner import run_files
from taoryx.visualization import render_run_artifact_plots
from tests.e2e.support.golden_plants import GoldenPlantCase, run_golden_plant

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = yaml.safe_load((ROOT / "verification/controller_scenarios.yaml").read_text(encoding="utf-8"))
pytestmark = [pytest.mark.dof6, pytest.mark.slow]


def _cases() -> list[dict[str, object]]:
    return [item for item in MANIFEST["vehicles"] if item.get("status", "ready") == "ready" and item.get("family") == "fixed-wing"]
    ####


def _multirotor_cases() -> list[dict[str, object]]:
    return [
        item
        for item in MANIFEST["vehicles"]
        if item.get("status", "ready") == "ready"
        and item.get("family") == "multirotor"
        and item.get("kind") == "multirotor_waypoint_land"
    ]
    ####


@pytest.mark.parametrize("case", _cases(), ids=lambda item: str(item["id"]))
def test_slower_vehicle_native_controller_response_is_bounded(case: dict[str, object], tmp_path: Path) -> None:
    """Native route attitude control remains inside the verified plant envelope."""

    vehicle_id = str(case["id"])
    problem = ROOT / str(case["problem"])
    tables = tuple(ROOT / str(path) for path in case["tables"])
    run = run_golden_plant(
        GoldenPlantCase(
            vehicle=vehicle_id,
            problem=problem,
            tables=tables,
            max_steps=int(case["max_steps"]),
            controller_configured=True,
            expectations={str(key): float(value) for key, value in dict(case["expectations"]).items()},
        ),
        tmp_path,
    )
    run.require_success()
    run.require_finite()
    run.require_active_aerodynamics()
    run.require_convention_firewall()
    run.require_initial_closure()
    run.require_table_margins()
    run.require_channels("pro_nav_active", "attitude_controller_saturated", "aero_alpha_deg", "aero_sideslip_deg")
    expectations = dict(case["expectations"])
    assert run.final["time_s"] + 1.0e-9 >= float(expectations["min_duration_s"])
    assert max(abs(sample["aero_alpha_deg"]) for sample in run.history) <= float(expectations["max_alpha_deg"])
    assert max(abs(sample["aero_sideslip_deg"]) for sample in run.history) <= float(expectations["max_beta_deg"])
    assert max(abs(sample.get(name, 0.0)) for sample in run.history for name in ("wx", "wy", "wz")) <= float(expectations["max_body_rate_deg_s"]) * 3.141592653589793 / 180.0 + 1.0e-8
    if float(expectations.get("require_guidance", 0.0)) > 0.0:
        assert max(sample["pro_nav_active"] for sample in run.history) == pytest.approx(1.0)
    assert max(sample["attitude_controller_saturated"] for sample in run.history) == pytest.approx(0.0)
    ####


@pytest.mark.artifact
@pytest.mark.parametrize("case", _cases(), ids=lambda item: str(item["id"]))
def test_slower_vehicle_controller_writes_artifacts(case: dict[str, object], artifact_dir: Path, tmp_path: Path) -> None:
    problem = ROOT / str(case["problem"])
    tables = tuple(ROOT / str(path) for path in case["tables"])
    report = run_files(problem, tables, output_dir=tmp_path / str(case["id"]), max_steps=int(case["max_steps"]), profile=GrammarProfile.TAORYX)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    destination = artifact_dir / "slower-vehicle-controllers" / str(case["id"])
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "scenario.json").write_text(json.dumps({"case": case, "report": report.as_dict()}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    for artifact in report.artifacts:
        render_run_artifact_plots(
            artifact,
            destination,
            vehicle_id="1",
            channels=(
                "taos.altitude_m", "taos.speed_m_s", "taos.aero_alpha_deg", "taos.aero_sideslip_deg",
                "taos.wx", "taos.wy", "taos.wz", "taos.pro_nav_acceleration_m_s2",
                "taos.attitude_controller_saturated", "taos.differential-elevon-deg",
            ),
        )
    assert (destination / "scenario.json").stat().st_size > 100
    ####


@pytest.mark.parametrize("case", _multirotor_cases(), ids=lambda item: str(item["id"]))
def test_multirotor_controller_waypoint_and_landing_are_bounded(
    case: dict[str, object],
    tmp_path: Path,
) -> None:
    """The shared controller manifest can execute a rotorcraft waypoint case."""

    problem = ROOT / str(case["problem"])
    tables = tuple(ROOT / str(path) for path in case["tables"])
    report = run_files(
        problem,
        tables,
        output_dir=tmp_path / str(case["id"]),
        max_steps=int(case["max_steps"]),
        profile=GrammarProfile.TAORYX,
    )
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    assert report.results and report.results[0].completed
    states = report.results[0].states["1"]
    history = tuple({"time_s": state.time, **dict(state.named)} for state in states)
    expectations = dict(case["expectations"])
    assert history[-1]["time_s"] >= float(expectations["min_duration_s"])
    assert max(sample["altitude_m"] for sample in history) <= float(expectations["max_altitude_m"])
    assert max(sample["speed_m_s"] for sample in history) <= float(expectations["max_speed_m_s"])
    assert history[-1]["range_to_target_m"] <= float(expectations["final_range_m"])
    assert history[-1]["altitude_m"] <= float(expectations["final_altitude_m"])
    assert history[-1]["motor_shutdown"] == pytest.approx(float(expectations["require_motor_shutdown"]))
    assert max(sample["translation_equation_residual_normalized"] for sample in history) < 1.0e-8
    assert max(sample["rotation_equation_residual_normalized"] for sample in history) < 1.0e-8
    ####


@pytest.mark.artifact
@pytest.mark.parametrize("case", _multirotor_cases(), ids=lambda item: str(item["id"]))
def test_multirotor_controller_writes_artifacts(
    case: dict[str, object],
    artifact_dir: Path,
    tmp_path: Path,
) -> None:
    """Publish the shared-manifest multirotor controller evidence."""

    problem = ROOT / str(case["problem"])
    tables = tuple(ROOT / str(path) for path in case["tables"])
    report = run_files(
        problem,
        tables,
        output_dir=tmp_path / str(case["id"]),
        max_steps=int(case["max_steps"]),
        profile=GrammarProfile.TAORYX,
    )
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    destination = artifact_dir / "slower-vehicle-controllers" / str(case["id"])
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "scenario.json").write_text(
        json.dumps({"case": case, "report": report.as_dict()}, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    for artifact in report.artifacts:
        render_run_artifact_plots(
            artifact,
            destination,
            vehicle_id="1",
            channels=(
                "taos.altitude_m",
                "taos.speed_m_s",
                "taos.range_to_target_m",
                "taos.aero_query_rotor_speed",
                "taos.motor_shutdown",
                "taos.wx",
                "taos.wy",
                "taos.wz",
            ),
        )
    assert (destination / "scenario.json").stat().st_size > 100
    ####
