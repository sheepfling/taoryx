"""Focused contracts for plug-in-owned retained controller-screen metadata."""

from __future__ import annotations

import pytest

from taoryx.local_controller_screen_advertisements import (
    LocalControllerScreenAdvertisement,
    LocalControllerScreenAdvertisementRegistry,
)


def _registration(identifier: str = "example-local-screen") -> LocalControllerScreenAdvertisement:
    """Build one complete minimal opaque-controller advertisement."""

    return LocalControllerScreenAdvertisement(
        id=identifier,
        provider_id="example.provider",
        model_id="example-model",
        family_id="example-family",
        fidelity="rigid_body_6dof_direct_wrench",
        realization_id="rigid_body_6dof_direct_wrench",
        mission_template_id="example_local_screen_v1",
        advertisement={
            "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
            "id": identifier,
            "mission_template_id": "example_local_screen_v1",
            "fidelity": "rigid_body_6dof_direct_wrench",
            "operations": ["batch"],
            "control_realization": "direct_wrench",
            "controller": {"method": "lqr"},
            "claim_boundary": "Opaque local profile only.",
        },
    )
    ####


def test_local_controller_screen_registry_selects_only_the_exact_endpoint() -> None:
    """A static retained-controller record never falls back across selections."""

    registry = LocalControllerScreenAdvertisementRegistry(registrations=(_registration(),))
    matched = registry.matching(
        provider_id="example.provider",
        model_id="example-model",
        family_id="example-family",
        fidelity="rigid_body_6dof_direct_wrench",
        realization_id="rigid_body_6dof_direct_wrench",
        mission_template_id="example_local_screen_v1",
    )

    assert [item.id for item in matched] == ["example-local-screen"]
    public = matched[0].public_advertisement()
    public["controller"]["method"] = "changed"  # type: ignore[index]
    assert matched[0].public_advertisement()["controller"]["method"] == "lqr"  # type: ignore[index]
    assert registry.matching(
        provider_id="example.provider",
        model_id="example-model",
        family_id="example-family",
        fidelity="rigid_body_6dof_direct_wrench",
        realization_id="rigid_body_6dof_direct_wrench",
        mission_template_id="other-screen",
    ) == ()
    ####


def test_local_controller_screen_registry_rejects_duplicate_endpoint_ownership() -> None:
    """Two plug-ins cannot publish competing metadata for one endpoint."""

    with pytest.raises(ValueError, match="duplicate endpoint selections"):
        LocalControllerScreenAdvertisementRegistry(
            registrations=(_registration("first-screen"), _registration("second-screen")),
        )
    ####
