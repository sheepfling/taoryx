from __future__ import annotations

import json
from pathlib import Path

import pytest

from taoryx.visualization import render_run_artifact_plots
from tests.e2e.support.golden_plants import GoldenPlantCase, load_golden_vehicle_catalog, run_golden_plant

ROOT = Path(__file__).resolve().parents[2]
SCENARIOS = load_golden_vehicle_catalog(ROOT / "verification/long_vehicle_scenarios.yaml")


@pytest.mark.slow
@pytest.mark.dof6
@pytest.mark.parametrize("case", SCENARIOS, ids=lambda case: case.vehicle.casefold().replace(" ", "-"))
def test_long_vehicle_scenarios_use_the_verified_plant_contract(case: GoldenPlantCase, tmp_path: Path) -> None:
    """Longer open-loop cases exercise propagation after convention validation."""

    run = run_golden_plant(case, tmp_path / case.vehicle)
    run.require_success()
    run.require_finite()
    run.require_active_aerodynamics()
    run.require_convention_firewall()
    run.require_initial_closure()
    run.require_table_margins()
    run.require_channels("time_s", "altitude_m", "speed_m_s", "aero_alpha_deg", "aero_sideslip_deg")

    expectations = case.expectations
    assert run.final["time_s"] - run.initial["time_s"] >= expectations["min_duration_s"]
    if "max_alpha_deg" in expectations:
        valid_alpha = [abs(sample["aero_alpha_deg"]) for sample in run.history if sample.get("aero_air_data_valid", 1.0) >= 0.5]
        if valid_alpha:
            assert max(valid_alpha) < expectations["max_alpha_deg"]
    if "max_beta_deg" in expectations:
        valid_beta = [abs(sample["aero_sideslip_deg"]) for sample in run.history if sample.get("aero_air_data_valid", 1.0) >= 0.5]
        if valid_beta:
            assert max(valid_beta) < expectations["max_beta_deg"]
    if "max_altitude_delta_m" in expectations:
        run.require_bounded_delta("altitude_m", expectations["max_altitude_delta_m"])
    if "max_speed_delta_m_s" in expectations:
        run.require_bounded_delta("speed_m_s", expectations["max_speed_delta_m_s"])
    if "max_final_mach" in expectations:
        assert run.final["aero_mach"] < expectations["max_final_mach"]
    if "max_final_speed_m_s" in expectations:
        assert run.final["speed_m_s"] < expectations["max_final_speed_m_s"]
    ####


@pytest.mark.artifact
@pytest.mark.dof6
@pytest.mark.parametrize("case", SCENARIOS, ids=lambda case: case.vehicle.casefold().replace(" ", "-"))
def test_long_vehicle_scenarios_write_standard_artifacts(case: GoldenPlantCase, artifact_dir: Path, tmp_path: Path) -> None:
    """Long scenarios publish the same report and Matplotlib views as other runs."""

    run = run_golden_plant(case, tmp_path / case.vehicle)
    run.require_success()
    destination = artifact_dir / "slower-vehicle-long" / case.vehicle.casefold().replace(" ", "-")
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "run-report.json").write_text(
        json.dumps(run.report.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    for artifact in run.report.artifacts:
        render_run_artifact_plots(
            artifact,
            destination / "plots",
            vehicle_id="1",
            channels=(
                "taos.altitude_m", "taos.speed_m_s", "taos.aero_airspeed_m_s",
                "taos.aero_dynamic_pressure_pa", "taos.aero_alpha_deg", "taos.aero_sideslip_deg",
                "taos.aero_force_body_z_n",
            ),
        )
    assert (destination / "run-report.json").stat().st_size > 100
    ####
