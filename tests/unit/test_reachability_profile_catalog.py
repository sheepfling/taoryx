from __future__ import annotations

from pathlib import Path

from taoryx.reachability_catalog import load_reachability_catalog

from taoryx.runtime.cli import main

ROOT = Path(__file__).resolve().parents[2]
CATALOG = ROOT / "verification" / "reachability_profile_catalog.yaml"


def test_reachability_profile_catalog_has_unique_profiles_and_family_links() -> None:
    """The reachability registry is a complete, machine-readable runtime catalog."""

    payload = load_reachability_catalog(CATALOG)
    profiles = payload.profiles
    families = payload.vehicle_families
    profile_ids = [profile.id for profile in profiles]
    family_ids = [family.id for family in families]
    assert len(profile_ids) == len(set(profile_ids))
    assert len(family_ids) == len(set(family_ids))
    assert set(payload.study_semantics) >= {
        "first_pass",
        "bounded_time",
        "bounded_resource",
        "bounded_path",
        "eventual_capture",
        "robust_capture",
    }
    assert set(payload.promotion_rungs) == {
        "cataloged",
        "data_ready",
        "3dof_ready",
        "pseudo_6dof_ready",
        "mission_ready",
        "profile_qualified",
    }
    for profile in profiles:
        assert profile.configurations
        assert profile.phases
        assert profile.state_dimensions
        assert profile.products
    for family in families:
        assert family.configurations
        assert family.profiles
        assert all(profile_id in profile_ids for profile_id in family.profiles)
    ####


def test_reachability_cli_lists_profiles(capsys) -> None:
    assert main(["reachability", "list", "profiles"]) == 0
    output = capsys.readouterr().out
    assert "reachability.fixed_wing.first_pass_capture.v1" in output
    assert "reachability.spacecraft.orbit_visibility.v1" in output
