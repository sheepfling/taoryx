"""Focused validation for public local controller-screen advertisements."""

from __future__ import annotations

from typing import Any

import pytest

from taoryx.controller_tuning_registry import ControllerTuningCampaignRegistration


def _registration(controller: dict[str, object]) -> ControllerTuningCampaignRegistration:
    """Create a registration whose constructor checks static screen metadata."""

    return ControllerTuningCampaignRegistration(
        id="example-lqi-campaign-v1",
        provider_id="example.provider",
        model_id="example-model",
        family_id="example-family",
        fidelity="rigid_body_6dof_surface_allocated",
        realization_ids=("rigid_body_6dof_surface_allocated",),
        mission_template_ids=("example_local_screen_v1",),
        description="Example source-local controller campaign.",
        adapter_factory=lambda: None,  # type: ignore[return-value]
        campaign_factory=lambda: None,  # type: ignore[return-value]
        local_controller_screens=(
            {
                "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
                "id": "example-local-screen-v1",
                "mission_template_id": "example_local_screen_v1",
                "fidelity": "rigid_body_6dof_surface_allocated",
                "operations": ["batch"],
                "control_realization": "surface_allocated",
                "controller": controller,
                "claim_boundary": "Example local controller screen only.",
            },
        ),
    )
    ####


def test_local_lqi_advertisement_requires_its_campaign_and_outputs() -> None:
    """An LQI plan cannot be advertised as an untraceable generic controller."""

    registration = _registration(
        {
            "method": "lqi",
            "campaign_id": "example-lqi-campaign-v1",
            "integral_output_names": ["pitch_error_rad"],
            "fixed_cadence_s": 0.02,
            "screen_duration_s": 2.0,
        }
    )

    assert registration.local_controller_screens[0]["controller"] == {
        "method": "lqi",
        "campaign_id": "example-lqi-campaign-v1",
        "integral_output_names": ["pitch_error_rad"],
        "fixed_cadence_s": 0.02,
        "screen_duration_s": 2.0,
    }
    context = registration.public_dict()["tuning_application_context"]
    assert context["status"] == "available_when_candidate_ready"
    assert "applied_gain_fingerprint_sha256" in context["runtime_binding_fields"]
    ####


@pytest.mark.parametrize(
    ("controller", "message"),
    (
        ({"method": "lqi", "integral_output_names": ["pitch_error_rad"]}, "identify this campaign"),
        ({"method": "lqi", "campaign_id": "example-lqi-campaign-v1", "integral_output_names": []}, "integral outputs"),
        ({"method": "lqr", "integral_output_names": ["pitch_error_rad"]}, "cannot advertise integral outputs"),
        ({"method": "pid"}, "must be 'lqr' or 'lqi'"),
        ({"method": "lqr", "fixed_cadence_s": 0.0}, "fixed_cadence_s"),
    ),
)
def test_local_controller_advertisement_rejects_incoherent_method_metadata(
    controller: dict[str, Any],
    message: str,
) -> None:
    """Method-specific metadata must not silently contradict the advertised tuner."""

    with pytest.raises(ValueError, match=message):
        _registration(controller)
    ####
