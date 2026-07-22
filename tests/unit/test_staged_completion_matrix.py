from __future__ import annotations

from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
MATRIX = ROOT / "verification/staged_completion_matrix.yaml"
EXPECTED_VEHICLES = {"b747", "skywalker_x8", "hummingbird", "x15"}
EXPECTED_MODES = {"point_mass_3dof", "kinematic_3_plus_3_dof", "rigid_body_6dof"}
ALLOWED_STATUSES = {
    "pass",
    "pass_local",
    "pass_source_trimmed",
    "pass_bounded_research_case",
    "pass_bounded_directional",
}
####


def test_staged_completion_matrix_covers_all_vehicle_fidelity_modes() -> None:
    """Every standard vehicle has an explicit status at every fidelity tier."""

    document = yaml.safe_load(MATRIX.read_text(encoding="utf-8"))
    vehicles = document["vehicles"]
    assert set(vehicles) == EXPECTED_VEHICLES
    for vehicle in vehicles.values():
        assert set(vehicle["modes"]) == EXPECTED_MODES
        assert all(entry["status"] in ALLOWED_STATUSES for entry in vehicle["modes"].values())
        assert isinstance(vehicle["blockers"], list)
        evidence = vehicle["evidence"]
        assert (ROOT / evidence["source_differential_test"]).is_file()
        for category in ("closure_tests", "convergence_tests"):
            assert evidence[category]
            assert all((ROOT / path).is_file() for path in evidence[category])
        for entry in vehicle["modes"].values():
            problem = entry["problem"]
            if problem != "derived_in_test_from_point_mass_problem":
                assert (ROOT / problem).is_file(), problem
        for section in ("recovery", "long_validation"):
            assert (ROOT / vehicle[section]["problem"]).is_file()
    ####


def test_staged_completion_matrix_preserves_explicit_blockers() -> None:
    """Passing lower tiers do not erase vehicle-specific stress limitations."""

    vehicles = yaml.safe_load(MATRIX.read_text(encoding="utf-8"))["vehicles"]
    assert any("corner" in item for item in vehicles["skywalker_x8"]["blockers"])
    assert any("beta" in item for item in vehicles["x15"]["blockers"])
    assert vehicles["hummingbird"]["blockers"] == []
    ####
