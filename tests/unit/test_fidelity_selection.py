from __future__ import annotations

from taoryx.trajectory import FidelitySelectionRequest, select_validated_fidelity
from taoryx.trajectory.reference_families import ReferenceFidelityProfile


def _profiles() -> tuple[ReferenceFidelityProfile, ...]:
    return (
        ReferenceFidelityProfile(
            profile_id="f16.point",
            runtime_fidelity="point_mass_3dof",
            status="planned_derived_from_parent",
            parent_profile="f16.rigid",
        ),
        ReferenceFidelityProfile(
            profile_id="f16.pseudo",
            runtime_fidelity="pseudo_6dof",
            status="planned_derived_from_parent",
            parent_profile="f16.rigid",
        ),
        ReferenceFidelityProfile(
            profile_id="f16.rigid",
            runtime_fidelity="rigid_body_6dof",
            status="runtime_replay_qualification_passed",
        ),
    )
    ####


def test_exact_request_does_not_use_a_pending_reduction() -> None:
    result = select_validated_fidelity(
        _profiles(),
        FidelitySelectionRequest("pseudo_6dof"),
        {"f16.pseudo": {"status": "development_screen_passed"}},
    )
    assert not result.accepted
    assert result.reason == "no_validated_fidelity_in_requested_range"
    assert result.considered[0]["fidelity"] == "pseudo_6dof"
    ####


def test_validated_lower_only_selects_only_an_earned_lower_tier() -> None:
    profiles = _profiles()
    result = select_validated_fidelity(
        profiles,
        FidelitySelectionRequest("pseudo_6dof", "validated_lower_only"),
        {
            "f16.pseudo": {"status": "equivalence_pending"},
            "f16.point": {"status": "equivalence_passed", "artifact": "verification/f16-reduction-evidence.json"},
        },
    )
    assert result.accepted
    assert result.selected == "point_mass_3dof"
    assert result.reason == "validated_lower_fidelity_selected"
    assert result.reduction_artifact == "verification/f16-reduction-evidence.json"
    ####


def test_requested_rigid_tier_is_selected_without_lowering() -> None:
    result = select_validated_fidelity(
        _profiles(),
        FidelitySelectionRequest("rigid_body_6dof", "validated_lower_only"),
        {"f16.rigid": {"status": "runtime_replay_qualification_passed"}},
    )
    assert result.accepted
    assert result.selected == "rigid_body_6dof"
    assert result.reason == "exact_requested_fidelity"
    ####
