from __future__ import annotations

from taoryx.fidelity_lowering import LoweringCandidate, select_canonical_lowering


def test_shared_lowering_prefers_highest_checked_tier() -> None:
    candidates = {
        "point_mass_3dof": LoweringCandidate("point_mass_3dof", "point"),
        "pseudo_6dof": LoweringCandidate("pseudo_6dof", "pseudo", ("point",)),
        "rigid_body_6dof_direct_wrench": LoweringCandidate("rigid_body_6dof_direct_wrench", "direct", ("point",)),
        "rigid_body_6dof_surface_allocated": LoweringCandidate("rigid_body_6dof_surface_allocated", "surface", ("direct",)),
    }
    decision = select_canonical_lowering(
        candidates,
        "rigid_body_6dof_surface_allocated",
        {
            "point": {"status": "qualified"},
            "pseudo": {"status": "qualified"},
            "direct": {"status": "development"},
            "surface": {"status": "qualified"},
        },
        allow_lowering=True,
    )
    assert decision.selected == "pseudo_6dof"
    assert [item["fidelity"] for item in decision.considered] == [
        "rigid_body_6dof_surface_allocated",
        "rigid_body_6dof_direct_wrench",
        "pseudo_6dof",
    ]
    ####


def test_shared_lowering_fails_closed_without_a_profile() -> None:
    decision = select_canonical_lowering(
        {"rigid_body_6dof_surface_allocated": LoweringCandidate("rigid_body_6dof_surface_allocated", None)},
        "rigid_body_6dof_surface_allocated",
        {},
        allow_lowering=False,
    )
    assert decision.selected is None
    assert decision.considered[0]["status"] == "unavailable"
    ####


def test_shared_lowering_can_require_adapter_operations() -> None:
    candidates = {
        "point_mass_3dof": LoweringCandidate("point_mass_3dof", "point"),
        "rigid_body_6dof_direct_wrench": LoweringCandidate(
            "rigid_body_6dof_direct_wrench",
            "direct",
            ("point",),
            required_operations=("state_derivative", "trim"),
        ),
    }
    evidence = {"point": {"status": "qualified"}, "direct": {"status": "qualified"}}
    operation_status = {
        "rigid_body_6dof_direct_wrench": {
            "state_derivative": "pass",
            "trim": "not_applicable",
        }
    }

    blocked = select_canonical_lowering(
        candidates,
        "rigid_body_6dof_direct_wrench",
        evidence,
        allow_lowering=False,
        operation_status=operation_status,
    )
    assert blocked.selected is None
    assert blocked.considered[0]["status"] == "blocked"
    assert blocked.considered[0]["missing_operations"] == ["trim"]

    lowered = select_canonical_lowering(
        candidates,
        "rigid_body_6dof_direct_wrench",
        evidence,
        allow_lowering=True,
        operation_status=operation_status,
    )
    assert lowered.selected == "point_mass_3dof"
    ####
