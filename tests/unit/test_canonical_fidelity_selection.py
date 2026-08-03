from __future__ import annotations

from taoryx.trajectory import (
    CanonicalFidelitySelectionRequest,
    ReferenceFidelityProfile,
    select_validated_canonical_fidelity,
)


def _profiles() -> tuple[ReferenceFidelityProfile, ...]:
    return (
        ReferenceFidelityProfile(
            profile_id="x8.point",
            runtime_fidelity="point_mass_3dof",
            control_realization="force_model",
            status="qualified",
        ),
        ReferenceFidelityProfile(
            profile_id="x8.pseudo",
            runtime_fidelity="pseudo_6dof",
            control_realization="response_law",
            status="qualified",
        ),
        ReferenceFidelityProfile(
            profile_id="x8.direct",
            runtime_fidelity="rigid_body_6dof",
            control_realization="direct_wrench",
            status="qualified",
        ),
        ReferenceFidelityProfile(
            profile_id="x8.surface",
            runtime_fidelity="rigid_body_6dof",
            control_realization="surface_allocated",
            status="qualified",
        ),
    )
    ####


def test_canonical_selector_preserves_surface_and_direct_tiers() -> None:
    evidence = {profile.profile_id: {"status": "qualified"} for profile in _profiles()}
    result = select_validated_canonical_fidelity(
        _profiles(),
        CanonicalFidelitySelectionRequest("rigid_body_6dof_surface_allocated"),
        evidence,
    )
    assert result.accepted
    assert result.selected == "rigid_body_6dof_surface_allocated"
    assert result.profile_id == "x8.surface"
    ####


def test_canonical_selector_lowers_surface_to_validated_direct_then_pseudo() -> None:
    profiles = _profiles()
    evidence = {
        "x8.surface": {"status": "development"},
        "x8.direct": {"status": "development"},
        "x8.pseudo": {"status": "qualified"},
    }
    result = select_validated_canonical_fidelity(
        profiles,
        CanonicalFidelitySelectionRequest(
            "rigid_body_6dof_surface_allocated",
            fallback_policy="validated_lower_only",
        ),
        evidence,
    )
    assert result.accepted
    assert result.selected == "pseudo_6dof"
    assert result.reason == "validated_lower_fidelity_selected"
    ####


def test_canonical_selector_rejects_ambiguous_legacy_rigid_profile() -> None:
    profile = ReferenceFidelityProfile(
        profile_id="legacy.rigid",
        runtime_fidelity="rigid_body_6dof",
        status="qualified",
    )
    result = select_validated_canonical_fidelity(
        (profile,),
        CanonicalFidelitySelectionRequest("rigid_body_6dof_direct_wrench"),
        {"legacy.rigid": {"status": "qualified"}},
    )
    assert not result.accepted
    assert result.considered[0]["profile_status"] == "missing"
    ####
