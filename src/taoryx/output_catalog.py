"""Canonical semantic metadata for TAOS output channels."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

InterpolationKind = Literal["linear", "angle", "step", "slerp", "event"]


@dataclass(frozen=True, slots=True)
class OutputChannelSpec:
    """Semantic contract for one or more historical TAOS aliases."""

    aliases: tuple[str, ...]
    semantic_name: str
    quantity: str
    interpolation: InterpolationKind = "linear"
    visualization_roles: tuple[str, ...] = ("all",)
####


OUTPUT_CHANNEL_CATALOG: tuple[OutputChannelSpec, ...] = (
    OutputChannelSpec(("alt",), "position.altitude.geodetic", "length"),
    OutputChannelSpec(("long",), "position.longitude", "angle", "angle"),
    OutputChannelSpec(("latgd",), "position.latitude.geodetic", "angle", "angle"),
    OutputChannelSpec(("latgc",), "position.latitude.geocentric", "angle", "angle"),
    OutputChannelSpec(("x", "xecfc"), "position.ecfc.x", "length"),
    OutputChannelSpec(("y", "yecfc"), "position.ecfc.y", "length"),
    OutputChannelSpec(("z", "zecfc"), "position.ecfc.z", "length"),
    OutputChannelSpec(("xdt", "xecfcdt"), "velocity.ecfc.x", "speed"),
    OutputChannelSpec(("ydt", "yecfcdt"), "velocity.ecfc.y", "speed"),
    OutputChannelSpec(("zdt", "zecfcdt"), "velocity.ecfc.z", "speed"),
    OutputChannelSpec(("mass", "wt"), "mass.total", "mass"),
    OutputChannelSpec(("mdt", "wtdt"), "mass.flow", "mass_rate"),
    OutputChannelSpec(("thrust",), "propulsion.thrust", "force", visualization_roles=("rocket", "airbreather")),
    OutputChannelSpec(("power",), "propulsion.throttle_command", "dimensionless", visualization_roles=("airbreather",)),
    OutputChannelSpec(("dynprs",), "aerodynamics.dynamic_pressure", "pressure"),
    OutputChannelSpec(("alpha",), "aerodynamics.angle_of_attack", "angle", "angle"),
    OutputChannelSpec(("beta",), "aerodynamics.sideslip", "angle", "angle"),
    OutputChannelSpec(("segment", "_segment"), "phase.segment", "discrete", "step"),
)

_BY_ALIAS = {alias: spec for spec in OUTPUT_CHANNEL_CATALOG for alias in spec.aliases}


def output_channel_spec(source_name: str) -> OutputChannelSpec | None:
    """Return the semantic registry entry for a historical output alias."""

    return _BY_ALIAS.get(source_name.casefold())
####


def canonical_output_name(source_name: str) -> str:
    """Normalize private runtime aliases to their public historical names."""

    return "segment" if source_name.casefold() == "_segment" else source_name
####
