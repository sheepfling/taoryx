from __future__ import annotations

from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
REGISTRY = ROOT / "verification/vehicle_models.yaml"
FAMILIES = ROOT / "verification/vehicle_families.yaml"
CATALOG = ROOT / "verification/vehicle_catalog.yaml"


def test_vehicle_registry_covers_the_standard_four() -> None:
    """All standard vehicles expose one consistent physical/configuration contract."""

    registry = yaml.safe_load(REGISTRY.read_text(encoding="utf-8"))
    families = yaml.safe_load(FAMILIES.read_text(encoding="utf-8"))
    catalog = yaml.safe_load(CATALOG.read_text(encoding="utf-8"))
    assert set(registry["vehicles"]) == {"b747", "skywalker_x8", "hummingbird", "x15"}
    assert set(families["families"]) == {"powered_fixed_wing", "rocket_plane", "multirotor_direct_wrench"}
    assert {entry["model_id"] for entry in catalog["vehicles"]} == set(registry["vehicles"])
    for vehicle_id, vehicle in registry["vehicles"].items():
        family = families["families"][vehicle["family_id"]]
        assert family["dynamics_mode"] == vehicle["mode"]
        assert family["body_frame"] == vehicle["body_frame"]
        assert vehicle["reference_area_m2"] > 0.0
        assert vehicle["reference_length_m"] > 0.0
        assert vehicle["dry_mass_kg"] > 0.0
        assert all(float(value) > 0.0 for value in vehicle["inertia_kg_m2"].values())
        assert vehicle["body_frame"]
        assert vehicle["table_bindings"]
        assert all((ROOT / table).is_file() for table in vehicle["table_bindings"]), vehicle_id
####


def test_vehicle_registry_control_defaults_have_bounds() -> None:
    """Controller defaults are data and must be inside their declared limits."""

    registry = yaml.safe_load(REGISTRY.read_text(encoding="utf-8"))
    for vehicle in registry["vehicles"].values():
        for control in vehicle["controls"]:
            assert control["lower"] <= control["default"] <= control["upper"]
####
