from __future__ import annotations

from pathlib import Path

import pytest

from tests.e2e.support.golden_plants import GoldenPlantCase, load_golden_vehicle_catalog, verify_golden_plant

ROOT = Path(__file__).resolve().parents[2]
GOLDEN_CATALOG = load_golden_vehicle_catalog(ROOT / "verification/vehicle_catalog.yaml")
def _family_marker(vehicle: str) -> pytest.MarkDecorator:
    """Map catalog vehicle names to the public family selector marker."""

    normalized = vehicle.casefold()
    if "hummingbird" in normalized:
        family = "hummingbird"
    elif "747" in normalized:
        family = "b747"
    elif "x8" in normalized:
        family = "x8"
    elif "x-15" in normalized or "x15" in normalized:
        family = "x15"
    else:
        raise ValueError(f"No vehicle-family marker mapping for {vehicle!r}")
    return getattr(pytest.mark, family)
    ####


GOLDEN_PLANT_CASES = tuple(
    pytest.param(
        case,
        id=case.vehicle.casefold().replace(" ", "-"),
        marks=_family_marker(case.vehicle),
    )
    for case in GOLDEN_CATALOG
)
X15_CASE = next(case for case in GOLDEN_CATALOG if case.vehicle == "X-15")


@pytest.mark.slow
@pytest.mark.dof6
@pytest.mark.x15
def test_x15_research_anchor_passes_the_convention_firewall(tmp_path: Path) -> None:
    """The X-15 source surrogate is wired before controller work begins."""

    verification = verify_golden_plant(
        X15_CASE,
        tmp_path,
    )
    assert verification.verdict == "plant-golden"
    assert verification.plant_golden is True
    assert [stage.status for stage in verification.stages[:11]] == ["pass"] * 11
    assert verification.stages[11].status == "blocked"
    run = verification.run
    assert run is not None
    run.require_success()
    run.require_finite()
    run.require_active_aerodynamics()
    run.require_convention_firewall()
    run.require_initial_values(aero_alpha_deg=0.0, aero_sideslip_deg=0.0)
    run.require_initial_closure(translation_max=1.0e-10, rotation_max=1.0e-10)
    run.require_table_margins()
    run.require_channels("aero_mach", "aero_dynamic_pressure_pa", "force_body_x_n", "moment_body_y_nm")
    assert run.initial["aero_mach"] == pytest.approx(4.9578637787, abs=1.0e-8)
    assert run.initial["aero_dynamic_pressure_pa"] > 0.0
    assert run.initial["force_body_x_n"] != 0.0
    assert run.initial["moment_body_y_nm"] != 0.0
    ####


@pytest.mark.slow
@pytest.mark.parametrize("case", GOLDEN_PLANT_CASES)
def test_all_four_source_anchored_plants_are_golden(case: GoldenPlantCase, tmp_path: Path) -> None:
    """All four vehicle families pass the complete plant firewall."""

    verification = verify_golden_plant(case, tmp_path / case.vehicle)
    assert verification.plant_golden, verification.as_dict()
    assert verification.verdict == "plant-golden"
    assert all(stage.status == "pass" for stage in verification.stages[:11])
    assert verification.stages[11].status == "blocked"
    closure = verification.stages[8].evidence
    translation = closure["independent_translation_closure"]
    rotation = closure["independent_rotation_closure"]
    assert translation["sample_count"] >= 1.0
    assert rotation["sample_count"] >= 1.0
    assert translation["p99_normalized_residual"] >= 0.0
    assert rotation["p99_normalized_residual"] >= 0.0
    ####
