from __future__ import annotations

import pytest

from taoryx.family_strategy import (
    FamilyTierStrategy,
    build_family_strategy_worklist,
    load_family_strategy_catalog,
    validate_family_strategy_catalog,
)
from taoryx.horizontal_fidelity import load_horizontal_registry


def test_family_strategy_catalog_covers_every_registered_family_and_tier() -> None:
    catalog = load_family_strategy_catalog()
    report = validate_family_strategy_catalog(catalog)
    worklist = build_family_strategy_worklist(catalog=catalog)

    assert report.status == "pass"
    assert not report.errors
    assert len(worklist.items) == 36
    assert len({(item.family_id, item.tier) for item in worklist.items}) == 36


def test_fixed_wing_families_reuse_one_strategy_without_losing_overlays() -> None:
    worklist = build_family_strategy_worklist()
    x8 = next(item for item in worklist.items if item.family_id == "skywalker_x8" and item.tier == "rigid_body_6dof_surface_allocated")
    b747 = next(item for item in worklist.items if item.family_id == "b747" and item.tier == "rigid_body_6dof_surface_allocated")

    assert x8.strategy_id == b747.strategy_id == "powered_fixed_wing.v1"
    assert x8.mission_overlay == "fixed_wing_racetrack"
    assert b747.mission_overlay == "heavy_transport_racetrack"
    assert "aerodynamic_force_model" in x8.family_inputs
    assert x8.stages.index("authority_envelope") < x8.stages.index("attitude_rate_reversal_probe")


def test_worklist_makes_topology_limits_and_passive_non_applicability_explicit() -> None:
    worklist = build_family_strategy_worklist()
    x8_direct = next(item for item in worklist.items if item.family_id == "skywalker_x8" and item.tier == "rigid_body_6dof_direct_wrench")
    tumbling_direct = next(item for item in worklist.items if item.family_id == "tumbling_body" and item.tier == "rigid_body_6dof_direct_wrench")

    assert x8_direct.stages.index("authority_preflight") < x8_direct.stages.index("operating_point_campaign")
    assert tumbling_direct.status == "not_applicable"
    assert tumbling_direct.next_action == "no_control_tuning_required"


def test_existing_reduced_models_clear_adapter_waits_without_false_effector_claims() -> None:
    worklist = build_family_strategy_worklist()

    for family_id in ("hummingbird", "f16_s119"):
        for tier in ("point_mass_3dof", "pseudo_6dof"):
            item = next(candidate for candidate in worklist.items if candidate.family_id == family_id and candidate.tier == tier)
            assert item.status in {"strategy_development", "strategy_probe_ready"}
            assert item.next_action == f"run_strategy_stage:{item.stages[0]}"
            assert item.pending_operations == ()


def test_replay_backed_x15_reduced_tiers_admit_only_their_first_strategy_audit() -> None:
    """A runnable staged witness is visible without becoming an adapter probe."""

    worklist = build_family_strategy_worklist()

    for tier, expected_stage in (
        ("point_mass_3dof", "release_audit"),
        ("pseudo_6dof", "parent_parity_check"),
    ):
        item = next(candidate for candidate in worklist.items if candidate.family_id == "x15" and candidate.tier == tier)

        assert item.status == "strategy_development"
        assert item.next_action == f"run_strategy_stage:{expected_stage}"
        assert item.pending_operations == ("state_derivative", "trim")
        assert item.runnable_batch_execution_modes == ("open_loop_witness",)
        assert item.runnable_batch_missions == ("x15_staged_booster_reachability_v1",)
        assert item.runnable_batch_claim_boundaries


def test_unknown_strategy_mapping_fails_closed() -> None:
    registry = load_horizontal_registry()
    altered = registry.model_copy(
        update={
            "families": (
                registry.families[0].model_copy(update={"strategy_id": "unknown.v1"}),
                *registry.families[1:],
            )
        }
    )

    report = validate_family_strategy_catalog(registry=altered)

    assert report.status == "fail"
    assert report.errors[0].code == "strategy-missing"


def test_linearized_family_strategy_requires_authority_and_campaign_gates() -> None:
    """A newly added controllable family cannot skip the no-tuning-yet gate."""

    with pytest.raises(ValueError, match="operating_point_campaign"):
        FamilyTierStrategy(
            data=("force_moment_model",),
            operations=("state_derivative", "trim", "linearize"),
            calibration_mode="trim_linearize_scaled_lqr",
            stages=("trim_grid", "derivative_consistency", "authority_preflight"),
        )
