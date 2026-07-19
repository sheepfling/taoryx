from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from taoryx.visualization import render_run_artifact_plots
from tests.e2e.support.golden_plants import GoldenPlantCase, run_golden_plant

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = yaml.safe_load((ROOT / "verification/vehicle_maneuver_scenarios.yaml").read_text(encoding="utf-8"))
pytestmark = [pytest.mark.dof6, pytest.mark.slow]


def _cases() -> list[dict[str, object]]:
    return list(MANIFEST["vehicles"])
    ####


def _run(case: dict[str, object], output_dir: Path):
    expectations = {str(key): float(value) for key, value in dict(case["expectations"]).items()}
    return run_golden_plant(
        GoldenPlantCase(
            vehicle=str(case["id"]),
            problem=ROOT / str(case["problem"]),
            tables=tuple(ROOT / str(path) for path in case["tables"]),
            max_steps=int(case["max_steps"]),
            controller_configured=True,
            convergence_factors=(1.0,),
            expectations=expectations,
        ),
        output_dir,
    )
    ####


@pytest.mark.parametrize("case", _cases(), ids=lambda item: str(item["id"]))
def test_vehicle_maneuver_reaches_declared_waypoint_and_altitude(case: dict[str, object], tmp_path: Path) -> None:
    """Each slower vehicle performs a bounded native waypoint maneuver."""

    run = _run(case, tmp_path / str(case["id"]))
    run.require_success()
    run.require_finite()
    run.require_active_aerodynamics()
    run.require_convention_firewall()
    run.require_initial_closure()
    run.require_table_margins()
    run.require_channels("range_to_target_m", "altitude_m", "speed_m_s")
    expectations = dict(case["expectations"])
    assert run.final["time_s"] + 1.0e-8 >= float(expectations["min_duration_s"])
    assert run.final["range_to_target_m"] <= float(expectations["max_final_range_m"])
    if "max_final_speed_m_s" in expectations:
        assert run.final["speed_m_s"] <= float(expectations["max_final_speed_m_s"])
    for sample in run.history:
        if sample.get("aero_air_data_valid", 1.0) >= 0.5:
            assert abs(sample["aero_alpha_deg"]) <= float(expectations.get("max_alpha_deg", float("inf")))
            assert abs(sample["aero_sideslip_deg"]) <= float(expectations.get("max_beta_deg", float("inf")))
    origin_altitude = run.initial["altitude_m"]
    assert max(abs(sample["altitude_m"] - origin_altitude) for sample in run.history) <= float(expectations.get("max_altitude_delta_m", float("inf")))
    if "max_speed_delta_m_s" in expectations:
        origin_speed = run.initial["speed_m_s"]
        assert max(abs(sample["speed_m_s"] - origin_speed) for sample in run.history) <= float(expectations["max_speed_delta_m_s"])
    ####


@pytest.mark.artifact
@pytest.mark.parametrize("case", _cases(), ids=lambda item: str(item["id"]))
def test_vehicle_maneuver_writes_evidence_artifacts(case: dict[str, object], artifact_dir: Path, tmp_path: Path) -> None:
    """Publish the common maneuver report and Matplotlib views."""

    run = _run(case, tmp_path / str(case["id"]))
    run.require_success()
    destination = artifact_dir / "slower-vehicle-maneuvers" / str(case["id"])
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "scenario.json").write_text(
        json.dumps({"case": case, "report": run.report.as_dict()}, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    for artifact in run.report.artifacts:
        render_run_artifact_plots(
            artifact,
            destination / "plots",
            vehicle_id="1",
            channels=(
                "taos.altitude_m", "taos.range_to_target_m", "taos.speed_m_s",
                "taos.aero_alpha_deg", "taos.aero_sideslip_deg", "taos.local_pitch_deg",
                "taos.local_roll_deg", "taos.local_heading_deg", "taos.aero_force_body_z_n",
            ),
        )
    assert (destination / "scenario.json").stat().st_size > 100
    ####
