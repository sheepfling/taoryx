"""Focused contract tests for portable tuning-application contexts."""

from __future__ import annotations

import pytest

from taoryx.composition_result_catalog import _runtime_tuning_binding
from taoryx.tuning_application import tuning_application_contexts_from_campaign_payload


def test_lqi_application_context_carries_resolved_gains_and_requires_exact_coordinates() -> None:
    candidate = {
        "profile_id": "balanced",
        "method": "lqi",
        "state_names": ["pitch_error_rad", "q_rad_s"],
        "control_names": ["elevator_deg"],
        "state_scales": [0.1, 0.2],
        "control_scales": [10.0],
        "weights": {
            "q_diagonal": [1.0, 2.0],
            "r_diagonal": [3.0],
            "integral_q_diagonal": [4.0],
        },
        "integral_output_names": ["pitch_error_rad"],
        "lqi_controller": {
            "output_matrix": [[1.0, 0.0]],
            "state_gain": [[2.0, 3.0]],
            "integral_gain": [[4.0]],
        },
    }
    payload = {
        "schema": "taoryx.tuning-campaign/v1alpha1",
        "status": "candidate_ready",
        "campaign": {"campaign_id": "example-lqi-campaign"},
        "nodes": [
            {
                "node_id": "trim",
                "status": "candidate_ready",
                "lqr": {
                    "best_profile_id": "balanced",
                    "candidates": [candidate],
                },
            }
        ],
    }

    (context,) = tuning_application_contexts_from_campaign_payload(
        payload,
        campaign_id="example-lqi-campaign",
        cache_key="cache-key",
        cache_hit=True,
        cache_persisted=True,
    )

    assert context.cache_disposition == "hit"
    assert context.resolved_gains["integral_gain"] == ((4.0,),)
    binding = context.runtime_binding_after_application(
        controller_method="lqi",
        state_names=("pitch_error_rad", "q_rad_s"),
        control_names=("elevator_deg",),
        integral_output_names=("pitch_error_rad",),
    )
    assert binding["campaign_id"] == "example-lqi-campaign"
    assert binding["cache_key"] == "cache-key"
    assert binding["applied_gain_fingerprint_sha256"] == context.resolved_gain_fingerprint_sha256
    declared = _runtime_tuning_binding({"tuning_binding": binding}, "lqi")
    assert declared["applied_gain_fingerprint_sha256"] == context.resolved_gain_fingerprint_sha256
    with pytest.raises(ValueError, match="state names"):
        context.runtime_binding_after_application(
            controller_method="lqi",
            state_names=("q_rad_s", "pitch_error_rad"),
            control_names=("elevator_deg",),
            integral_output_names=("pitch_error_rad",),
        )
    ####


def test_lqr_application_context_requires_a_serialized_state_gain() -> None:
    candidate = {
        "profile_id": "balanced",
        "method": "lqr",
        "state_names": ["u_m_s"],
        "control_names": ["throttle_fraction"],
        "state_scales": [10.0],
        "control_scales": [1.0],
        "weights": {"q_diagonal": [1.0], "r_diagonal": [2.0], "integral_q_diagonal": []},
        "integral_output_names": [],
    }
    payload = {
        "schema": "taoryx.tuning-campaign/v1alpha1",
        "status": "candidate_ready",
        "campaign": {"campaign_id": "example-lqr-campaign"},
        "nodes": [
            {
                "node_id": "trim",
                "status": "candidate_ready",
                "lqr": {"best_profile_id": "balanced", "candidates": [candidate]},
            }
        ],
    }

    with pytest.raises(ValueError, match="does not expose resolved gains"):
        tuning_application_contexts_from_campaign_payload(
            payload,
            campaign_id="example-lqr-campaign",
            cache_key="cache-key",
            cache_hit=False,
            cache_persisted=True,
        )
    candidate["lqr_controller"] = {"state_gain": [[0.5]]}
    (context,) = tuning_application_contexts_from_campaign_payload(
        payload,
        campaign_id="example-lqr-campaign",
        cache_key="cache-key",
        cache_hit=False,
        cache_persisted=True,
    )
    assert context.resolved_gains["state_gain"] == ((0.5,),)
    ####
