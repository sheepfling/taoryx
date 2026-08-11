"""CADAC composition-control metadata tests."""

from __future__ import annotations

import pytest
from taoryx.families.cadac.control_metadata import (
    CadacControlOutputEvidence,
    batch_configuration_control_advertisement,
    source_managed_control_advertisement,
)

from taoryx.trajectory.configuration_contract import (
    ConfigurationGroupSchema,
    ConfigurationParameterSchema,
    TrajectoryConfigurationSchema,
)


def _schema() -> TrajectoryConfigurationSchema:
    return TrajectoryConfigurationSchema(
        model_id="cadac.test.direct-plant",
        model_version="0",
        supported_fidelities=("rigid_body_6dof_surface_allocated",),
        root=ConfigurationGroupSchema(
            id="test",
            label="Test",
            children=(
                ConfigurationGroupSchema(
                    id="commands",
                    label="Direct commands",
                    children=(
                        ConfigurationParameterSchema(
                            id="pitch_command_deg",
                            label="Pitch command",
                            description="Direct physical pitch-control coordinate.",
                            canonical_unit="deg",
                            display_unit="deg",
                            default=0.0,
                            default_declared=True,
                            role="segment",
                        ),
                        ConfigurationParameterSchema(
                            id="thrust_vector_unit_body",
                            label="Thrust-vector direction",
                            description="Nonzero body-axis vector normalized by the native plant.",
                            value_type="vector3",
                            default=(1.0, 0.0, 0.0),
                            default_declared=True,
                            role="segment",
                        ),
                    ),
                ),
            ),
        ),
        claim_boundary="Test schema only.",
    )
    ####


def test_batch_configuration_controls_publish_exact_unitful_native_bindings() -> None:
    advertisement = batch_configuration_control_advertisement(
        _schema(),
        channels={
            "tvc.pitch.deflection": ("commands", "pitch_command_deg"),
            "rcs.thrust_vector.direction": ("commands", "thrust_vector_unit_body"),
        },
        output_evidence={
            "tvc.pitch.deflection": CadacControlOutputEvidence("requested_tvc_deg", ("achieved_tvc_deg",), 0, (0,)),
            "rcs.thrust_vector.direction": CadacControlOutputEvidence("requested_direction", ("rcs_force_body_n",)),
        },
        authority_id="direct_commands",
        authority="native_bridge",
        authority_description="Direct native commands fixed for one batch.",
        intent_id="direct_control",
        intent_label="Direct Control",
        intent_description="Set direct native control coordinates.",
        mission_ids=("test_mission",),
        source_refs=("test-source",),
        claim_boundary="Test direct batch boundary.",
    )

    assert advertisement.status == "available"
    assert advertisement.default_authority_id == "direct_commands"
    assert advertisement.authorities[0].operations == ("batch",)
    assert advertisement.intents[0].resolution == "external_channel"
    pitch, direction = advertisement.channels
    assert (pitch.data_type, pitch.canonical_unit, pitch.quantity) == ("float64", "deg", "angle")
    assert pitch.native_binding is not None
    assert pitch.native_binding.provider_binding["configuration_path"] == ["commands", "pitch_command_deg"]
    assert pitch.provider_binding["output_evidence"]["requested"] == {
        "channel_id": "requested_tvc_deg",
        "component": 0,
    }
    assert direction.shape == (3,)
    assert direction.quantity == "direction"
    assert direction.value_space.topology == "unit_direction"
    assert direction.native_binding is not None
    assert direction.native_binding.provider_binding["update_semantics"] == "fixed_for_batch"
    assert advertisement.rl_action_space(operation="batch").shape == (4,)
    ####


def test_batch_configuration_controls_require_output_evidence_for_every_channel() -> None:
    with pytest.raises(ValueError, match="missing=.*tvc.pitch.deflection"):
        batch_configuration_control_advertisement(
            _schema(),
            channels={"tvc.pitch.deflection": ("commands", "pitch_command_deg")},
            output_evidence={},
            authority_id="direct_commands",
            authority="native_bridge",
            authority_description="Direct native command.",
            intent_id="direct_control",
            intent_label="Direct Control",
            intent_description="Set one direct control.",
            mission_ids=("test_mission",),
            source_refs=("test-source",),
            claim_boundary="Test direct batch boundary.",
        )
    ####


def test_source_managed_control_advertises_internal_batch_authority() -> None:
    advertisement = source_managed_control_advertisement(
        mission_ids=("source_mission",),
        source_refs=("test-source",),
        claim_boundary="Test source-managed boundary.",
    )

    assert advertisement.status == "internally_generated"
    assert advertisement.channels == ()
    assert advertisement.default_authority_id == "source_program_control"
    assert advertisement.authorities[0].availability == "available_in_batch"
    assert advertisement.intents[0].resolution == "provider_internal"
    ####
