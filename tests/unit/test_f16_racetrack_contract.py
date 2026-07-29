"""Contract tests for the F-16's shared powered-fixed-wing route binding."""

from __future__ import annotations

from pathlib import Path

import yaml

from taoryx.racetrack_template import load_racetrack_template_catalog

ROOT = Path(__file__).resolve().parents[2]


def test_f16_racetrack_bindings_share_geometry_but_not_fidelity_claims() -> None:
    catalog = load_racetrack_template_catalog(ROOT / "verification/racetrack_templates.yaml")
    bindings = tuple(catalog.get(binding_id) for binding_id in (
        "f16-s119-point-mass",
        "f16-s119-pseudo-6dof",
        "f16-s119-direct-wrench",
        "f16-s119-surfaces",
    ))
    assert {binding.fidelity for binding in bindings} == {
        "point_mass_3dof",
        "pseudo_6dof_kinematic_bridge",
        "rigid_body_6dof_direct_wrench",
        "rigid_body_6dof_surface_allocated",
    }
    assert {binding.status for binding in bindings} == {
        "binding_defined_mission_pending",
        "development_route_evidence_pending_qualification",
    }
    assert len({binding.declared_duration_s for binding in bindings}) == 1
    assert all(binding.straight_length_m == 12000.0 for binding in bindings)
    assert all(binding.turn_radius_m == 25000.0 for binding in bindings)
    assert all(binding.left_turn_bank_deg == -6.0 for binding in bindings)
    assert all(binding.right_turn_bank_deg == -6.0 for binding in bindings)
    ####


def test_f16_manifest_points_to_the_shared_racetrack_contract() -> None:
    manifest = yaml.safe_load((ROOT / "families/reference_f16_s119/family.yaml").read_text(encoding="utf-8"))
    mission = manifest["missions"]["baseline"]
    plan = manifest["segment_plans"]["baseline"]
    binding = yaml.safe_load((ROOT / "families/reference_f16_s119/qualification/racetrack-binding.yaml").read_text(encoding="utf-8"))
    assert mission["mission.profile"] == "subsonic_energy_racetrack_contract"
    assert plan["mission.profile"] == "subsonic_energy_racetrack_contract"
    assert binding["template"] == "powered_fixed_wing_racetrack_v1"
    assert len(binding["binding_ids"]) == 4
    assert binding["phase_order"] == [
        "outbound-climb",
        "outbound-level",
        "left-turn",
        "inbound-descent",
        "inbound-level",
        "right-turn",
    ]
    assert binding["status"] == "adapter_available_development_qualification_pending"
    ####
