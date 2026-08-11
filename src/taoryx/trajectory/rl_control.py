"""Framework-neutral action-space projections for Mission Composition controls.

The projection is intentionally free of a Torch dependency.  It publishes the
same information a Torch or Gymnasium adapter needs: stable channel order,
action encoding, native bounds, normalization policy, discrete values, and
mask/replay requirements.  Consumers can turn the returned dictionary into
``torch.Tensor`` objects without importing provider code or guessing from UI
widgets.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .configuration_contract import (
    ConfigurationInterval,
    ControlCommandSemantics,
    TrajectoryControlAdvertisement,
    TrajectoryControlChannelMetadata,
)

RLActionEncoding = Literal["box", "discrete", "multi_binary"]
RLActionDType = Literal["float32", "int64", "bool", "mixed"]
RLNormalization = Literal["affine", "standardize", "identity", "periodic_wrap", "categorical_index", "binary"]
RLMasking = Literal["none", "optional", "required"]
####


class RLControlChannelSpec(BaseModel):
    """One action-channel projection suitable for a Torch policy head."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    channel_id: str = Field(min_length=1)
    action_index: int = Field(ge=0)
    flat_offset: int = Field(ge=0)
    flat_size: int = Field(gt=0)
    encoding: RLActionEncoding
    dtype: Literal["float32", "int64", "bool"]
    shape: tuple[int, ...] = ()
    normalization: RLNormalization
    native_low: tuple[float | None, ...] = ()
    native_high: tuple[float | None, ...] = ()
    agent_low: tuple[float, ...] = ()
    agent_high: tuple[float, ...] = ()
    action_values: tuple[Any, ...] = ()
    command_mode: str
    temporal_semantics: str
    repeat_policy: str
    masking: RLMasking = "none"
    periodic_period: float | None = None
    standardize_center: float | None = None
    standardize_scale: float | None = None
    agent_clip: bool = False
    requires_external_statistics: bool = False
    description: str = ""

    @model_validator(mode="after")
    def validate_shape(self) -> RLControlChannelSpec:
        if self.flat_size != (math.prod(self.shape) if self.shape else 1):
            raise ValueError(f"RL channel {self.channel_id!r} flat_size disagrees with shape")
        if self.encoding == "box":
            if len(self.native_low) != self.flat_size or len(self.native_high) != self.flat_size:
                raise ValueError(f"RL box channel {self.channel_id!r} must publish native bounds for every component")
            if len(self.agent_low) != self.flat_size or len(self.agent_high) != self.flat_size:
                raise ValueError(f"RL box channel {self.channel_id!r} must publish agent bounds for every component")
        if self.encoding == "discrete" and not self.action_values:
            raise ValueError(f"RL discrete channel {self.channel_id!r} requires at least one action value")
        if self.encoding == "multi_binary" and self.flat_size != 1:
            raise ValueError(f"RL multi-binary channel {self.channel_id!r} must be scalar")
        if self.periodic_period is not None and self.periodic_period <= 0.0:
            raise ValueError(f"RL periodic channel {self.channel_id!r} requires a positive period")
        if (self.standardize_center is None) != (self.standardize_scale is None):
            raise ValueError(f"RL standardized channel {self.channel_id!r} must publish center and scale together")
        if self.standardize_scale is not None and self.standardize_scale <= 0.0:
            raise ValueError(f"RL standardized channel {self.channel_id!r} requires a positive scale")
        return self
        ####

    ####


class RLActionSpaceSpec(BaseModel):
    """Complete mixed action-space description for an advertised realization."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.control-agent-action-space/v1"] = Field(
        default="taoryx.control-agent-action-space/v1",
        alias="schema",
        serialization_alias="schema",
    )
    operation: Literal["batch", "step"]
    kind: Literal["box", "discrete", "multi_discrete", "multi_binary", "dict"]
    dtype: RLActionDType
    shape: tuple[int, ...] = ()
    channels: tuple[RLControlChannelSpec, ...]
    flattening: Literal["advertised_channel_order"] = "advertised_channel_order"
    masking: RLMasking = "none"
    mask_channels: tuple[str, ...] = ()
    requires_external_statistics: bool = False
    claim_boundary: str = Field(min_length=1)

    @property
    def channel_map(self) -> dict[str, RLControlChannelSpec]:
        """Return stable channel lookup for action encoding/decoding."""

        return {item.channel_id: item for item in self.channels}
        ####

    def torch_spec(self) -> dict[str, Any]:
        """Return dependency-free metadata for constructing Torch action heads."""

        def one(channel: RLControlChannelSpec) -> dict[str, Any]:
            return {
                "encoding": channel.encoding,
                "dtype": {"float32": "torch.float32", "int64": "torch.int64", "bool": "torch.bool"}[channel.dtype],
                "shape": list(channel.shape) if channel.shape else [1],
                "low": list(channel.agent_low),
                "high": list(channel.agent_high),
                "action_values": list(channel.action_values),
                "normalization": channel.normalization,
                "clip": channel.agent_clip,
                "masking": channel.masking,
            }

        payload: dict[str, Any] = {
            "schema": self.schema_id,
            "kind": self.kind,
            "dtype": self.dtype,
            "shape": list(self.shape),
            "masking": self.masking,
            "requires_external_statistics": self.requires_external_statistics,
        }
        if self.kind == "dict":
            payload["spaces"] = {item.channel_id: one(item) for item in self.channels}
        elif self.channels:
            payload["space"] = one(self.channels[0]) if len(self.channels) == 1 else [one(item) for item in self.channels]
        return payload
        ####

    ####


def _bound_values(interval: ConfigurationInterval | None) -> tuple[float | None, float | None]:
    """Extract finite-or-unbounded interval endpoints."""

    lower = None if interval is None or interval.minimum is None else interval.minimum.value
    upper = None if interval is None or interval.maximum is None else interval.maximum.value
    return lower, upper
    ####


def _repeat(value: float | None, count: int) -> tuple[float | None, ...]:
    return (value,) * count
    ####


def _detent_values(channel: TrajectoryControlChannelMetadata) -> tuple[float, ...]:
    """Resolve explicit or interval-derived numeric detents for an RL head."""

    quantization = channel.semantics.quantization
    if quantization.mode == "levels":
        return quantization.levels
    if quantization.mode != "step" or quantization.step is None:
        raise ValueError(f"control {channel.id!r} does not publish finite detent values")
    lower, upper = _bound_values(channel.interval)
    if lower is None or upper is None:
        raise ValueError(f"step control {channel.id!r} requires finite interval bounds for a discrete RL action")
    first = math.ceil((lower - quantization.origin) / quantization.step - 1e-12)
    last = math.floor((upper - quantization.origin) / quantization.step + 1e-12)
    count = last - first + 1
    if count < 2:
        raise ValueError(f"step control {channel.id!r} has fewer than two representable detents")
    if count > 4096:
        raise ValueError(f"step control {channel.id!r} has too many detents for a discrete RL action; publish explicit levels")
    return tuple(quantization.origin + index * quantization.step for index in range(first, last + 1))
    ####


def _discrete_channel(
    channel: TrajectoryControlChannelMetadata,
    action_index: int,
    flat_offset: int,
) -> RLControlChannelSpec | None:
    """Build a categorical or binary projection when the value domain requires one."""

    semantics = channel.semantics
    domain = semantics.value_domain
    if domain == "boolean":
        return RLControlChannelSpec(
            channel_id=channel.id,
            action_index=action_index,
            flat_offset=flat_offset,
            flat_size=1,
            encoding="multi_binary",
            dtype="bool",
            shape=(),
            normalization="binary",
            action_values=(False, True),
            command_mode=semantics.command_mode,
            temporal_semantics=semantics.temporal_semantics,
            repeat_policy=semantics.repeat_policy,
            description=channel.description,
        )
    if domain in {"enum", "event"}:
        values: tuple[Any, ...] = channel.choices
        encoding: RLActionEncoding = "discrete"
        masking: RLMasking = "required" if semantics.repeat_policy != "repeatable" else "optional" if domain == "event" else "none"
    elif domain == "discrete_levels":
        values = _detent_values(channel)
        encoding = "discrete"
        masking = "none"
    else:
        return None
    return RLControlChannelSpec(
        channel_id=channel.id,
        action_index=action_index,
        flat_offset=flat_offset,
        flat_size=1,
        encoding=encoding,
        dtype="int64",
        shape=(),
        normalization="categorical_index",
        action_values=values,
        command_mode=semantics.command_mode,
        temporal_semantics=semantics.temporal_semantics,
        repeat_policy=semantics.repeat_policy,
        masking=masking,
        description=channel.description,
    )
    ####


def _box_channel(
    channel: TrajectoryControlChannelMetadata,
    action_index: int,
    flat_offset: int,
) -> RLControlChannelSpec:
    """Build an affine, periodic, standardized, or identity Box projection."""

    semantics: ControlCommandSemantics = channel.semantics
    if any(item == "variable" for item in channel.shape):
        raise ValueError(f"RL action channel {channel.id!r} requires a fixed shape")
    shape: tuple[int, ...] = tuple(int(item) for item in channel.shape)
    flat_size = math.prod(shape) if shape else 1
    lower, upper = _bound_values(channel.interval)
    native_low = _repeat(lower, flat_size)
    native_high = _repeat(upper, flat_size)
    policy = semantics.agent_normalization
    period = channel.value_space.period if semantics.value_domain == "periodic" else None
    if semantics.value_domain == "periodic":
        normalization: RLNormalization = "periodic_wrap"
        agent_low = (-1.0,) * flat_size
        agent_high = (1.0,) * flat_size
    elif policy == "standardize":
        normalization = "standardize"
        agent_low = (float("-inf"),) * flat_size
        agent_high = (float("inf"),) * flat_size
    elif policy == "identity" or lower is None or upper is None:
        normalization = "identity"
        agent_low = tuple(-float("inf") if item is None else item for item in native_low)
        agent_high = tuple(float("inf") if item is None else item for item in native_high)
    else:
        if upper <= lower:
            raise ValueError(f"control {channel.id!r} has an invalid finite RL interval")
        normalization = "affine"
        agent_low = (-1.0,) * flat_size
        agent_high = (1.0,) * flat_size
    return RLControlChannelSpec(
        channel_id=channel.id,
        action_index=action_index,
        flat_offset=flat_offset,
        flat_size=flat_size,
        encoding="box",
        dtype="float32",
        shape=shape,
        normalization=normalization,
        native_low=native_low,
        native_high=native_high,
        agent_low=agent_low,
        agent_high=agent_high,
        command_mode=semantics.command_mode,
        temporal_semantics=semantics.temporal_semantics,
        repeat_policy=semantics.repeat_policy,
        periodic_period=period,
        standardize_center=semantics.agent_center,
        standardize_scale=semantics.agent_scale,
        agent_clip=semantics.agent_clip,
        requires_external_statistics=normalization == "identity" and (lower is None or upper is None) and policy == "auto",
        description=channel.description,
    )
    ####


def build_rl_action_space(
    advertisement: TrajectoryControlAdvertisement,
    *,
    operation: Literal["batch", "step"] = "step",
) -> RLActionSpaceSpec:
    """Project available semantic action channels into a Torch-friendly space."""

    channels = tuple(channel for channel in advertisement.channels if channel.channel_kind == "action" and operation in channel.operations)
    if not channels:
        raise ValueError(f"control advertisement has no action channels for {operation!r}")
    projected: list[RLControlChannelSpec] = []
    flat_offset = 0
    for action_index, channel in enumerate(channels):
        discrete = _discrete_channel(channel, action_index, flat_offset)
        item = discrete if discrete is not None else _box_channel(channel, action_index, flat_offset)
        projected.append(item)
        flat_offset += item.flat_size
    encodings = {item.encoding for item in projected}
    shape: tuple[int, ...]
    if encodings == {"box"}:
        kind: Literal["box", "discrete", "multi_discrete", "multi_binary", "dict"] = "box"
        dtype: RLActionDType = "float32"
        shape = (flat_offset,)
    elif encodings == {"discrete"}:
        kind = "discrete" if len(projected) == 1 else "multi_discrete"
        dtype = "int64"
        shape = (len(projected),)
    elif encodings == {"multi_binary"}:
        kind = "multi_binary"
        dtype = "bool"
        shape = (flat_offset,)
    else:
        kind = "dict"
        dtype = "mixed"
        shape = ()
    mask_channels = tuple(item.channel_id for item in projected if item.masking != "none")
    masking: RLMasking = "required" if any(item.masking == "required" for item in projected) else "optional" if mask_channels else "none"
    return RLActionSpaceSpec(
        operation=operation,
        kind=kind,
        dtype=dtype,
        shape=shape,
        channels=tuple(projected),
        masking=masking,
        mask_channels=mask_channels,
        requires_external_statistics=any(item.requires_external_statistics for item in projected),
        claim_boundary=(
            "Derived agent action encoding from the advertised semantic control channels. "
            "This does not qualify a policy, normalization statistics, or vehicle fidelity."
        ),
    )
    ####


def _as_values(value: Any, size: int) -> tuple[float, ...]:
    """Normalize scalar/vector input into a flat finite numeric tuple."""

    if size == 1 and isinstance(value, int | float) and not isinstance(value, bool):
        values: tuple[float, ...] = (float(value),)
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        values = tuple(float(item) for item in value)
    else:
        raise ValueError("numeric control value must be a scalar or numeric sequence")
    if len(values) != size or any(not math.isfinite(item) for item in values):
        raise ValueError("numeric control value has the wrong shape or contains a non-finite value")
    return values
    ####


def encode_agent_action(space: RLActionSpaceSpec, channel_id: str, native_value: Any) -> Any:
    """Encode a native control value into a policy action value."""

    channel = space.channel_map[channel_id]
    if channel.encoding == "discrete":
        try:
            return channel.action_values.index(native_value)
        except ValueError as error:
            raise ValueError(f"{channel_id!r} value is not one of the advertised action values") from error
    if channel.encoding == "multi_binary":
        if not isinstance(native_value, bool):
            raise ValueError(f"{channel_id!r} expects a boolean action")
        return int(native_value)
    values = _as_values(native_value, channel.flat_size)
    if channel.normalization == "identity":
        return values[0] if channel.flat_size == 1 else values
    if channel.normalization == "standardize":
        if channel.standardize_center is None or channel.standardize_scale is None:
            raise ValueError(f"{channel_id!r} standardization requires provider statistics")
        return_value = tuple((value - channel.standardize_center) / channel.standardize_scale for value in values)
        return return_value[0] if channel.flat_size == 1 else return_value
    if channel.normalization == "periodic_wrap":
        if channel.periodic_period is None:
            raise ValueError(f"{channel_id!r} periodic action has no period")
        normalized_periodic = tuple(2.0 * ((value % channel.periodic_period) / channel.periodic_period) - 1.0 for value in values)
        return normalized_periodic[0] if channel.flat_size == 1 else normalized_periodic
    normalized: list[float] = []
    for value, lower, upper in zip(values, channel.native_low, channel.native_high, strict=True):
        if lower is None or upper is None or upper <= lower:
            raise ValueError(f"{channel_id!r} affine action requires finite bounds")
        result = 2.0 * (value - lower) / (upper - lower) - 1.0
        normalized.append(max(-1.0, min(1.0, result)))
    return normalized[0] if channel.flat_size == 1 else tuple(normalized)
    ####


def decode_agent_action(space: RLActionSpaceSpec, channel_id: str, agent_value: Any) -> Any:
    """Decode a policy action value into the channel's native representation."""

    channel = space.channel_map[channel_id]
    if channel.encoding == "discrete":
        index = int(agent_value)
        if index < 0 or index >= len(channel.action_values):
            raise ValueError(f"{channel_id!r} discrete action index is outside the advertised set")
        return channel.action_values[index]
    if channel.encoding == "multi_binary":
        if int(agent_value) not in {0, 1}:
            raise ValueError(f"{channel_id!r} binary action must be 0 or 1")
        return bool(agent_value)
    values = _as_values(agent_value, channel.flat_size)
    if channel.normalization == "identity":
        return values[0] if channel.flat_size == 1 else values
    if channel.normalization == "standardize":
        if channel.standardize_center is None or channel.standardize_scale is None:
            raise ValueError(f"{channel_id!r} standardization requires provider statistics")
        return_value = tuple(value * channel.standardize_scale + channel.standardize_center for value in values)
        return return_value[0] if channel.flat_size == 1 else return_value
    if channel.normalization == "periodic_wrap":
        if channel.periodic_period is None:
            raise ValueError(f"{channel_id!r} periodic action has no period")
        decoded_periodic = tuple(((value + 1.0) / 2.0) * channel.periodic_period for value in values)
        return decoded_periodic[0] if channel.flat_size == 1 else decoded_periodic
    decoded: list[float] = []
    for value, lower, upper in zip(values, channel.native_low, channel.native_high, strict=True):
        if lower is None or upper is None:
            raise ValueError(f"{channel_id!r} affine action requires finite bounds")
        decoded.append(lower + ((value + 1.0) / 2.0) * (upper - lower))
    return decoded[0] if channel.flat_size == 1 else tuple(decoded)
    ####


__all__ = [
    "RLActionSpaceSpec",
    "RLControlChannelSpec",
    "build_rl_action_space",
    "decode_agent_action",
    "encode_agent_action",
]
####
