"""Contracts for control lifecycle semantics and RL action projections."""

from __future__ import annotations

import pytest

from taoryx.trajectory import (
    ConfigurationBound,
    ConfigurationInterval,
    ConfigurationValueSpace,
    ControlCommandSemantics,
    ControlQuantizationMetadata,
    TrajectoryControlAdvertisement,
    TrajectoryControlAuthorityMetadata,
    TrajectoryControlChannelMetadata,
    build_rl_action_space,
    decode_agent_action,
    encode_agent_action,
)
from taoryx.trajectory.contract_probe_mission_composition import (
    CONTRACT_PROBE_MODEL_ID,
    ContractProbeMissionCompositionProvider,
)
from taoryx.trajectory.execution_contract import audit_provider_advertisement


def _interval(lower: float, upper: float) -> ConfigurationInterval:
    return ConfigurationInterval(
        minimum=ConfigurationBound(value=lower),
        maximum=ConfigurationBound(value=upper),
    )
    ####


def _space(topology: str, representation: str = "scalar", *, period: float | None = None) -> ConfigurationValueSpace:
    return ConfigurationValueSpace(
        topology=topology,
        representation=representation,
        error_rule="exact equality" if topology in {"boolean", "finite_set", "event"} else "linear subtraction",
        interpolation_rule="not interpolable" if topology in {"boolean", "finite_set", "event"} else "linear",
        period=period,
    )
    ####


def _channel(
    channel_id: str,
    *,
    data_type: str = "float64",
    value_space: ConfigurationValueSpace,
    interval: ConfigurationInterval | None = None,
    choices: tuple[str, ...] = (),
    semantics: ControlCommandSemantics | None = None,
) -> TrajectoryControlChannelMetadata:
    return TrajectoryControlChannelMetadata(
        id=channel_id,
        label=channel_id,
        description=f"Test channel {channel_id}.",
        channel_kind="action",
        data_type=data_type,
        interval=interval,
        choices=choices,
        value_space=value_space,
        semantics=semantics or ControlCommandSemantics(),
        availability="available",
        operations=("step",),
        native_channel_id=f"native.{channel_id}",
        provider_binding={"test": True},
        native_binding={
            "id": f"native.{channel_id}",
            "data_type": data_type,
            "interval": interval,
            "value_space": value_space,
            "provider_binding": {"test": True},
        },
        claim_boundary="Synthetic contract test only.",
    )
    ####


def _advertisement(*channels: TrajectoryControlChannelMetadata) -> TrajectoryControlAdvertisement:
    return TrajectoryControlAdvertisement(
        status="available",
        channels=channels,
        authorities=(
            TrajectoryControlAuthorityMetadata(
                id="test-authority",
                authority="provider_defined",
                availability="available",
                channel_ids=tuple(channel.id for channel in channels),
                operations=("step",),
                description="Synthetic test authority.",
                claim_boundary="Synthetic contract test only.",
            ),
        ),
        intents=(),
        claim_boundary="Synthetic contract test only.",
    )
    ####


def test_control_semantics_cover_rate_latch_momentary_pulse_and_detents() -> None:
    rate = ControlCommandSemantics(command_mode="rate", rate_unit="rad/s", temporal_semantics="held")
    assert rate.command_mode == "rate"
    assert (
        ControlCommandSemantics(
            temporal_semantics="latched",
            release_behavior="hold",
        ).temporal_semantics
        == "latched"
    )
    assert (
        ControlCommandSemantics(
            temporal_semantics="momentary",
            release_behavior="release_value",
            release_value=False,
        ).release_value
        is False
    )
    assert (
        ControlCommandSemantics(
            value_domain="event",
            command_mode="event",
            temporal_semantics="pulse",
            pulse_duration_s=0.25,
            repeat_policy="once_per_episode",
        ).repeat_policy
        == "once_per_episode"
    )
    detents = ControlQuantizationMetadata(mode="levels", levels=(0.0, 0.5, 1.0))
    assert detents.levels == (0.0, 0.5, 1.0)
    with pytest.raises(ValueError, match="rate_unit"):
        ControlCommandSemantics(command_mode="rate")
    with pytest.raises(ValueError, match="pulse controls"):
        ControlCommandSemantics(command_mode="event", temporal_semantics="pulse")
    ####


def test_rl_projection_mixes_box_discrete_binary_and_event_channels() -> None:
    advertisement = _advertisement(
        _channel(
            "throttle",
            value_space=_space("unit_interval"),
            interval=_interval(0.0, 1.0),
        ),
        _channel(
            "heading",
            value_space=_space("periodic_circle", period=360.0),
            semantics=ControlCommandSemantics(value_domain="periodic"),
        ),
        _channel(
            "stage-mode",
            data_type="string",
            value_space=_space("finite_set"),
            choices=("boost", "coast", "terminal"),
            semantics=ControlCommandSemantics(value_domain="enum"),
        ),
        _channel(
            "engine-enable",
            data_type="boolean",
            value_space=_space("boolean", "boolean"),
            semantics=ControlCommandSemantics(value_domain="boolean", temporal_semantics="momentary", release_behavior="release_value", release_value=False),
        ),
        _channel(
            "stage-separation",
            data_type="string",
            value_space=_space("event"),
            choices=("separate",),
            semantics=ControlCommandSemantics(
                value_domain="event",
                command_mode="event",
                temporal_semantics="sampled",
                repeat_policy="once_per_episode",
            ),
        ),
    )

    space = build_rl_action_space(advertisement)
    assert advertisement.rl_action_space().model_dump() == space.model_dump()

    assert space.kind == "dict"
    assert space.masking == "required"
    assert space.mask_channels == ("stage-separation",)
    assert space.channel_map["throttle"].normalization == "affine"
    assert space.channel_map["stage-mode"].action_values == ("boost", "coast", "terminal")
    assert space.channel_map["engine-enable"].encoding == "multi_binary"
    assert encode_agent_action(space, "throttle", 0.0) == pytest.approx(-1.0)
    assert decode_agent_action(space, "throttle", 1.0) == pytest.approx(1.0)
    assert encode_agent_action(space, "heading", 390.0) == pytest.approx(-5.0 / 6.0)
    assert decode_agent_action(space, "heading", -5.0 / 6.0) == pytest.approx(30.0)
    assert encode_agent_action(space, "stage-mode", "terminal") == 2
    assert decode_agent_action(space, "stage-mode", 1) == "coast"
    assert encode_agent_action(space, "engine-enable", True) == 1
    assert space.model_dump(by_alias=True)["schema"] == "taoryx.control-agent-action-space/v1"
    assert space.torch_spec()["kind"] == "dict"
    ####


def test_rl_projection_exposes_numeric_detents_and_standardization() -> None:
    advertisement = _advertisement(
        _channel(
            "gimbal-detent",
            value_space=_space("bounded_interval"),
            interval=_interval(-1.0, 1.0),
            semantics=ControlCommandSemantics(
                value_domain="discrete_levels",
                quantization=ControlQuantizationMetadata(mode="levels", levels=(-1.0, 0.0, 1.0)),
            ),
        ),
        _channel(
            "pitch-rate",
            value_space=_space("euclidean"),
            semantics=ControlCommandSemantics(
                command_mode="rate",
                rate_unit="rad/s",
                agent_normalization="standardize",
                agent_center=0.0,
                agent_scale=2.0,
            ),
        ),
    )
    space = build_rl_action_space(advertisement)
    assert space.kind == "dict"
    assert space.channel_map["gimbal-detent"].action_values == (-1.0, 0.0, 1.0)
    assert encode_agent_action(space, "pitch-rate", 4.0) == pytest.approx(2.0)
    assert decode_agent_action(space, "pitch-rate", -1.5) == pytest.approx(-3.0)
    assert space.channel_map["pitch-rate"].requires_external_statistics is False
    ####


def test_contract_probe_is_the_exhaustive_vehicle_composition_control_witness() -> None:
    """Keep every public control/agent mode reachable through one debug vehicle."""

    provider = ContractProbeMissionCompositionProvider()
    model = provider.list_models()[0]
    assert model.id == CONTRACT_PROBE_MODEL_ID
    assert model.family_id == "debug.contract-probe"
    assert model.model_kind == "contract_probe"

    controls = next(item.controls for item in model.realizations if item.id == "medium")
    assert {item.semantics.value_domain for item in controls.channels} >= {
        "continuous",
        "periodic",
        "boolean",
        "enum",
        "discrete_levels",
        "event",
        "vector",
        "provider_defined",
    }
    assert {item.semantics.command_mode for item in controls.channels} >= {
        "absolute",
        "rate",
        "increment",
        "event",
    }
    assert {item.semantics.temporal_semantics for item in controls.channels} >= {
        "held",
        "sampled",
        "profile",
        "momentary",
        "latched",
        "pulse",
    }
    assert {item.semantics.release_behavior for item in controls.channels} >= {
        "hold",
        "default",
        "failsafe",
        "release_value",
        "auto_reset",
    }
    assert {item.sampling_semantics for item in controls.channels} >= {
        "held_action",
        "batch_profile",
        "segment_generated",
        "not_sampled",
        "provider_reported",
        "event",
    }
    assert {item.semantics.quantization.mode for item in controls.channels} >= {"none", "step", "levels"}
    assert all(item.native_binding is not None and item.native_binding.semantics == item.semantics for item in controls.channels)

    space = controls.rl_action_space(operation="batch")
    assert space.kind == "dict"
    assert space.masking == "required"
    assert space.mask_channels == ("payload.release.command", "payload.arm.command")
    assert space.requires_external_statistics
    assert {item.normalization for item in space.channels} >= {
        "affine",
        "standardize",
        "identity",
        "periodic_wrap",
        "categorical_index",
        "binary",
    }
    assert space.channel_map["guidance.speed.command"].agent_clip
    assert space.channel_map["guidance.acceleration.increment"].requires_external_statistics
    assert space.channel_map["aerodynamics.spoiler.increment"].action_values == (-1.0, -0.5, 0.0, 0.5, 1.0)
    assert encode_agent_action(space, "guidance.heading.command", 390.0) == pytest.approx(-5.0 / 6.0)
    assert decode_agent_action(space, "guidance.heading.command", -5.0 / 6.0) == pytest.approx(30.0)
    assert encode_agent_action(space, "payload.release.command", "release") == 0
    assert space.torch_spec()["spaces"]["guidance.speed.command"]["clip"] is True

    audit = audit_provider_advertisement(provider, provider.build_runner())
    assert audit.status == "pass", audit.diagnostics
    ####
