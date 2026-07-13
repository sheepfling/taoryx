"""Initial-state resolution with explicit coordinate-order contracts."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from taoryx.contracts import FrameQuantityVector3


@dataclass(frozen=True, slots=True)
class InitialState:
    """Resolved position and velocity in ECFC, plus optional inherited source."""

    position: FrameQuantityVector3
    velocity: FrameQuantityVector3
    inherited_from: str | None = None


def resolve_initial_state(
    direct: InitialState | None = None,
    *,
    inherited: Mapping[str, InitialState] | None = None,
    reference: str | None = None,
) -> InitialState:
    """Resolve a direct or inherited normalized ECFC initial state."""

    if direct is not None and reference is not None:
        raise ValueError("initial state cannot be both direct and inherited")
    if direct is not None:
        return direct
    if reference is None or inherited is None:
        raise ValueError("a direct state or inherited reference is required")
    try:
        source = inherited[reference]
    except KeyError as error:
        raise KeyError(f"unknown inherited initial state: {reference}") from error
    return InitialState(source.position, source.velocity, reference)
####
