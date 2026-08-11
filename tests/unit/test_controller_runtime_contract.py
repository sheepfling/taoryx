"""Focused checks for typed controller-runtime declarations."""

from __future__ import annotations

import pytest

from taoryx.controller_runtime_contract import ControllerRuntimeDeclaration


def test_lqi_runtime_declaration_keeps_exact_tuning_receipt() -> None:
    declaration = ControllerRuntimeDeclaration.model_validate(
        {
            "controller_method": "lqi",
            "control_realization": "allocated_rotors",
            "integral_output_names": ["position_z_m"],
            "tuning_binding": {
                "campaign_id": "hover-lqi",
                "node_id": "hover",
                "candidate_profile_id": "balanced",
                "candidate_configuration_fingerprint_sha256": "a" * 64,
                "applied_gain_fingerprint_sha256": "b" * 64,
                "controller_method": "lqi",
                "cache_key": "stable-cache-key",
            },
        }
    )

    assert declaration.tuning_binding is not None
    assert declaration.tuning_binding.campaign_id == "hover-lqi"


def test_controller_runtime_declaration_rejects_integral_or_method_mismatch() -> None:
    with pytest.raises(ValueError, match="LQR execution must not identify integral outputs"):
        ControllerRuntimeDeclaration.model_validate(
            {
                "controller_method": "lqr",
                "control_realization": "direct_wrench",
                "integral_output_names": ["position_z_m"],
            }
        )


def test_lqi_runtime_declaration_requires_one_receipt_per_scheduled_node() -> None:
    """A schedule can declare a set only when every receipt is unambiguous."""

    payload = {
        "controller_method": "lqi",
        "control_realization": "surface_allocated",
        "integral_output_names": ["u_m_s"],
        "tuning_bindings": [
            {
                "campaign_id": "schedule-lqi",
                "node_id": "sea-level",
                "candidate_profile_id": "balanced-sea-level",
                "candidate_configuration_fingerprint_sha256": "a" * 64,
                "applied_gain_fingerprint_sha256": "b" * 64,
                "controller_method": "lqi",
            },
            {
                "campaign_id": "schedule-lqi",
                "node_id": "high-altitude",
                "candidate_profile_id": "balanced-high-altitude",
                "candidate_configuration_fingerprint_sha256": "c" * 64,
                "applied_gain_fingerprint_sha256": "d" * 64,
                "controller_method": "lqi",
            },
        ],
    }

    declaration = ControllerRuntimeDeclaration.model_validate(payload)

    assert [receipt.node_id for receipt in declaration.all_tuning_bindings] == ["sea-level", "high-altitude"]

    payload["tuning_bindings"][1]["node_id"] = "sea-level"
    with pytest.raises(ValueError, match="one receipt per node"):
        ControllerRuntimeDeclaration.model_validate(payload)
    with pytest.raises(ValueError, match="controller method disagrees"):
        ControllerRuntimeDeclaration.model_validate(
            {
                "controller_method": "lqr",
                "control_realization": "direct_wrench",
                "tuning_binding": {
                    "campaign_id": "hover-lqi",
                    "node_id": "hover",
                    "candidate_profile_id": "balanced",
                    "candidate_configuration_fingerprint_sha256": "a" * 64,
                    "applied_gain_fingerprint_sha256": "b" * 64,
                    "controller_method": "lqi",
                },
            }
        )
