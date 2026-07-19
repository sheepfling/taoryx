from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.runtime.runner import RunReport
from tests.e2e.support.golden_plants import GoldenPlantCase, GoldenPlantRun, load_golden_vehicle_catalog

ROOT = Path(__file__).resolve().parents[2]


def _case() -> GoldenPlantCase:
    return GoldenPlantCase(
        vehicle="fixture",
        problem=Path("fixture.prb"),
        tables=(Path("fixture.tbl"),),
        max_steps=10,
    )


def test_golden_plant_case_requires_standard_file_inputs() -> None:
    with pytest.raises(ValueError, match=r"\.prb"):
        GoldenPlantCase("fixture", Path("fixture.txt"), (Path("fixture.tbl"),), 10)
    with pytest.raises(ValueError, match=r"\.tbl"):
        GoldenPlantCase("fixture", Path("fixture.prb"), (Path("fixture.txt"),), 10)
    with pytest.raises(ValueError, match="max_steps"):
        GoldenPlantCase("fixture", Path("fixture.prb"), (Path("fixture.tbl"),), 0)
    ####


def test_vehicle_catalog_is_data_driven_and_complete() -> None:
    cases = load_golden_vehicle_catalog(ROOT / "verification/vehicle_catalog.yaml")
    assert tuple(case.vehicle for case in cases) == ("B747", "Skywalker X8", "AscTec Hummingbird", "X-15")
    assert all(case.problem.suffix == ".prb" for case in cases)
    assert all(all(path.suffix == ".tbl" for path in case.tables) for case in cases)
    ####


def test_golden_plant_run_assertions_use_immutable_telemetry_contract() -> None:
    history = (
        {
            "time_s": 0.0,
            "aero_active": 1.0,
            "aero_air_data_valid": 1.0,
            "aero_alpha_deg": 3.0,
            "aero_sideslip_deg": 0.0,
            "qw": 1.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "mass_kg": 1.0,
            "translation_equation_residual_normalized": 1.0e-12,
            "rotation_equation_residual_normalized": 2.0e-12,
            "aero_table_margin_force.cx.alpha": 0.25,
        },
        {
            "time_s": 1.0,
            "aero_active": 1.0,
            "aero_air_data_valid": 1.0,
            "aero_alpha_deg": 3.1,
            "aero_sideslip_deg": 0.0,
            "qw": 1.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "mass_kg": 1.0,
            "translation_equation_residual_normalized": 1.0e-12,
            "rotation_equation_residual_normalized": 2.0e-12,
            "aero_table_margin_force.cx.alpha": 0.20,
        },
    )
    report = RunReport(
        "fixture.prb",
        (),
        1,
        (),
        (),
        (),
        metadata=({
            "dynamics_mode": "rigid-body-6dof",
            "native_pipeline": {"integration_frame": "ecic", "environment_frame": "ecfc"},
        },),
    )
    run = GoldenPlantRun(_case(), report, history)
    run.require_finite()
    run.require_active_aerodynamics()
    run.require_convention_firewall()
    run.require_initial_values(aero_alpha_deg=3.0, aero_sideslip_deg=0.0)
    run.require_initial_closure(translation_max=1.0e-8, rotation_max=1.0e-8)
    run.require_table_margins(minimum=0.1)
    run.require_bounded_delta("aero_alpha_deg", 0.2)
    assert run.final["time_s"] == 1.0
    ####


def test_golden_plant_run_accepts_explicitly_unavailable_low_speed_air_data() -> None:
    history = (
        {
            "time_s": 0.0,
            "aero_active": 1.0,
            "aero_air_data_valid": 0.0,
            "aero_airspeed_m_s": 0.0,
            "aero_alpha_deg": float("nan"),
            "aero_sideslip_deg": float("nan"),
            "qw": 1.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "mass_kg": 0.5,
            "translation_equation_residual_normalized": 0.0,
            "rotation_equation_residual_normalized": 0.0,
            "aero_table_margin_force.cx.velocity_x": 0.25,
        },
    )
    report = RunReport(
        "fixture.prb",
        (),
        1,
        (),
        (),
        (),
        metadata=({"dynamics_mode": "rigid-body-6dof", "native_pipeline": {"integration_frame": "ecic", "environment_frame": "ecfc"}},),
    )
    run = GoldenPlantRun(_case(), report, history)
    run.require_finite()
    run.require_convention_firewall()
    ####
