from __future__ import annotations

from pathlib import Path

import yaml

from taoryx.specialized_segments import (
    SIMPLE_AERO_FAMILY_SEGMENTS,
    SIMPLE_AERO_SEGMENT_CONTRACTS,
    simple_aero_family_segments,
)

ROOT = Path(__file__).resolve().parents[2]


def test_simple_aero_contracts_cover_every_family_and_preserve_reuse_boundaries() -> None:
    catalog = yaml.safe_load((ROOT / "verification/simple_aero_segment_catalog.yaml").read_text(encoding="utf-8"))
    template_names = {item["name"] for item in catalog["templates"]}
    family_names = {item["name"] for item in catalog["families"]}

    assert family_names == set(SIMPLE_AERO_FAMILY_SEGMENTS)
    assert {contract.name for contract in SIMPLE_AERO_SEGMENT_CONTRACTS} >= template_names | {"moving_target_intercept"}
    assert all(mode in {"point_mass_3dof", "kinematic_3_plus_3", "rigid_body_6dof"} for mode in catalog["supported_dynamics"])
    assert all(item["status"] == "fixture-ready" for item in catalog["families"])

    for family in family_names:
        contracts = simple_aero_family_segments(family)
        catalog_family = next(item for item in catalog["families"] if item["name"] == family)
        assert tuple(contract.name for contract in contracts) == tuple(catalog_family["templates"])


def test_simple_aero_fixture_paths_and_sources_are_present() -> None:
    catalog = yaml.safe_load((ROOT / "verification/simple_aero_segment_catalog.yaml").read_text(encoding="utf-8"))
    for family in catalog["families"]:
        assert (ROOT / "tests/fixtures/problem_file_dumps/simple_aero_trajectories" / family["fixture"]).is_file()
        if family["source"].endswith(".yaml"):
            assert (ROOT / "tests/fixtures/simple_aero_v1" / family["source"]).is_file()
        else:
            assert family["source"] == "p027_proportional_navigation"
