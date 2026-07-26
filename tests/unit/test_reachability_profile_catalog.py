from __future__ import annotations

from pathlib import Path

from taoryx.reachability_catalog import load_reachability_catalog
from taoryx.runtime.cli import main

ROOT = Path(__file__).resolve().parents[2]
CATALOG = ROOT / "verification" / "reachability_profile_catalog.yaml"


def test_reachability_profile_catalog_has_unique_profiles_and_family_links() -> None:
    """The reachability registry is a complete, machine-readable planning surface."""

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


def test_reachability_catalog_covers_current_and_planned_rosters() -> None:
    """Current proof families, Anduril variants, spacecraft, and passive bodies are registered."""

    payload = load_reachability_catalog(CATALOG)
    configurations = {
        configuration
        for family in payload.vehicle_families
        for configuration in family.configurations
    }
    assert {"b747", "skywalker_x8", "hummingbird", "x15"} <= configurations
    assert {"bolt", "ghost_x", "roadrunner", "omen", "thunder"} <= configurations
    assert {"f16_s119", "hl20_mod_k", "reference_a320"} <= configurations
    assert {"spacecraft.6u_observer_rw.standard.v1", "spacecraft.spheres_like_rcs.v1", "spacecraft.marco_like_hybrid.v1"} <= configurations
    assert {"ballistic_rocket", "tumbling_ballistic_body", "kestrel_glider"} <= configurations
    ####


def test_reachability_cli_lists_profiles(capsys) -> None:
    assert main(["reachability", "list", "profiles"]) == 0
    output = capsys.readouterr().out
    assert "reachability.fixed_wing.first_pass_capture.v1" in output
    assert "reachability.spacecraft.orbit_visibility.v1" in output
