from __future__ import annotations

from pathlib import Path

import yaml

from taoryx.vehicle_registry import derive_lqr_scale_contract, lqr_profile_attributes, vehicle_definition

ROOT = Path(__file__).resolve().parents[2]
REGISTRY = ROOT / "verification/vehicle_models.yaml"
FAMILIES = ROOT / "verification/vehicle_families.yaml"
CATALOG = ROOT / "verification/vehicle_catalog.yaml"
PROFILES = ROOT / "verification/lqr_scaling_profiles.yaml"


def test_vehicle_registry_covers_the_standard_four() -> None:
    """All standard vehicles expose one consistent physical/configuration contract."""

    registry = yaml.safe_load(REGISTRY.read_text(encoding="utf-8"))
    families = yaml.safe_load(FAMILIES.read_text(encoding="utf-8"))
    catalog = yaml.safe_load(CATALOG.read_text(encoding="utf-8"))
    assert set(registry["vehicles"]) == {"b747", "skywalker_x8", "hummingbird", "x15"}
    assert set(families["families"]) == {
        "powered_fixed_wing",
        "rocket_plane",
        "multirotor_direct_wrench",
        "unpowered_lifting_body",
    }
    assert {entry["model_id"] for entry in catalog["vehicles"]} == set(registry["vehicles"])
    for vehicle_id, vehicle in registry["vehicles"].items():
        family = families["families"][vehicle["family_id"]]
        assert family["dynamics_mode"] == vehicle["mode"]
        assert family["body_frame"] == vehicle["body_frame"]
        assert vehicle["reference_area_m2"] > 0.0
        assert vehicle["reference_length_m"] > 0.0
        assert vehicle["dry_mass_kg"] > 0.0
        assert vehicle["nominal_mass_kg"] > 0.0
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


def test_vehicle_registry_references_complete_gentle_standard_aggressive_profiles() -> None:
    """Every standard family exposes the same controller-profile choices."""

    registry = yaml.safe_load(REGISTRY.read_text(encoding="utf-8"))
    profiles = yaml.safe_load(PROFILES.read_text(encoding="utf-8"))["profiles"]
    for vehicle_id, vehicle in registry["vehicles"].items():
        assert vehicle["attitude_lqr_profile"] in profiles
        profile_prefix = vehicle_id.replace("_", "-")
        for style in ("gentle", "standard", "aggressive"):
            profile_id = f"{profile_prefix}-{style}"
            assert profile_id in profiles
            profile = profiles[profile_id]
            assert profile["vehicle"] == vehicle_id
            assert profile["state_angle_scale_rad"] > 0.0
            assert profile["state_rate_scale_rad_s"] > 0.0
            assert profile["control_moment_scale_nm"] > 0.0
            assert set(profile["normalized_weights"]) == {"q_angle", "q_rate", "r_moment"}
####


def test_derived_lqr_scales_are_positive_and_profile_resolved() -> None:
    """Each standard profile resolves scales from its vehicle contract."""

    for vehicle_id in ("b747", "skywalker_x8", "hummingbird", "x15"):
        scales = derive_lqr_scale_contract(vehicle_id)
        assert scales["vehicle"] == vehicle_id
        assert scales["state_angle_scale_rad"] > 0.0
        assert scales["state_rate_scale_rad_s"] > 0.0
        assert scales["control_moment_scale_nm"] > 0.0
        assert scales["mass_scale_kg"] == vehicle_definition(vehicle_id)["nominal_mass_kg"]
        assert scales["inertia_scale_kg_m2"] == max(vehicle_definition(vehicle_id)["inertia_kg_m2"].values())
        assert scales["force_scale_n"] > 0.0
        assert scales["weight_moment_scale_nm"] > 0.0
        assert scales["inertia_moment_scale_nm"] > 0.0
        assert scales["provenance"]
        attributes = lqr_profile_attributes(f"{vehicle_id.replace('_', '-')}-standard")
        assert float(attributes["state-angle-scale-rad"]) == scales["state_angle_scale_rad"]
        assert float(attributes["state-rate-scale-rad-s"]) == scales["state_rate_scale_rad_s"]
        assert float(attributes["control-moment-scale-nm"]) == scales["control_moment_scale_nm"]
        assert float(attributes["mass-scale-kg"]) == scales["mass_scale_kg"]
        assert float(attributes["inertia-scale-kg-m2"]) == scales["inertia_scale_kg_m2"]
        assert attributes["mass-scaling"] == "nominal-ratio"
        assert attributes["update"] == "mass"
    ####
