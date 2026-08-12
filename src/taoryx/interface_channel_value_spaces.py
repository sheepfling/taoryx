"""Versioned mathematical topology for public vehicle interface channels."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping
from functools import lru_cache
from pathlib import Path
from typing import Literal

import yaml

from .plugins.resources import packaged_resource_fallback
from .value_space import (
    ValueSpaceSpec,
    boolean,
    bounded_interval,
    euclidean,
    finite_set,
    periodic_circle,
    positive_half_line,
    product,
    quaternion_so3,
    unit_interval,
)
from .vehicle_registry import ROOT

INTERFACE_CHANNEL_VALUE_SPACE_CATALOG = packaged_resource_fallback(
    ROOT / "verification/interface_channel_value_space_catalog.yaml",
    package="taoryx_reference_models",
    resource="data/verification/interface_channel_value_space_catalog.yaml",
)

InterfaceChannelValueSpaceProfile = Literal[
    "boolean",
    "bounded_interval",
    "euclidean_scalar",
    "euclidean_vector3",
    "event",
    "finite_set",
    "periodic_degrees",
    "periodic_radians",
    "positive_half_line",
    "quaternion_so3",
    "rotation_euler",
    "unit_interval",
]

_INTERFACE_CHANNEL_VALUE_SPACE_PROFILES: frozenset[InterfaceChannelValueSpaceProfile] = frozenset(
    {
        "boolean",
        "bounded_interval",
        "euclidean_scalar",
        "euclidean_vector3",
        "event",
        "finite_set",
        "periodic_degrees",
        "periodic_radians",
        "positive_half_line",
        "quaternion_so3",
        "rotation_euler",
        "unit_interval",
    }
)


def value_space_for_interface_channel_profile(
    profile: InterfaceChannelValueSpaceProfile,
    *,
    canonical_unit: str | None,
) -> ValueSpaceSpec:
    """Return one explicit profile, validating unit-dependent rotations."""

    if profile == "boolean":
        return boolean()
    if profile == "bounded_interval":
        return bounded_interval()
    if profile == "euclidean_scalar":
        return euclidean()
    if profile == "euclidean_vector3":
        return euclidean(3)
    if profile == "event":
        return finite_set(event=True)
    if profile == "finite_set":
        return finite_set()
    if profile == "periodic_degrees":
        if canonical_unit != "deg":
            raise ValueError("periodic_degrees interface profile requires canonical unit 'deg'")
        return periodic_circle(360.0)
    if profile == "periodic_radians":
        if canonical_unit != "rad":
            raise ValueError("periodic_radians interface profile requires canonical unit 'rad'")
        return periodic_circle(2.0 * math.pi)
    if profile == "positive_half_line":
        return positive_half_line()
    if profile == "quaternion_so3":
        return quaternion_so3()
    if profile == "rotation_euler":
        if canonical_unit == "deg":
            period = 360.0
        elif canonical_unit == "rad":
            period = 2.0 * math.pi
        else:
            raise ValueError("rotation_euler interface profile requires canonical unit 'deg' or 'rad'")
        return product(
            bounded_interval(),
            bounded_interval(),
            periodic_circle(period),
            representation="vector3 [roll, pitch, yaw]",
        )
    if profile == "unit_interval":
        return unit_interval()
    raise ValueError(f"unsupported interface channel value-space profile {profile!r}")
    ####


@lru_cache(maxsize=4)
def load_interface_channel_value_space_catalog(
    path: str | Path | None = None,
) -> dict[str, InterfaceChannelValueSpaceProfile]:
    """Load an explicit ID-to-profile mapping, rejecting ambiguous membership."""

    source = Path(path) if path is not None else INTERFACE_CHANNEL_VALUE_SPACE_CATALOG
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{source} must contain a mapping")
    if payload.get("schema") != "taoryx.interface-channel-value-space-catalog/v1alpha1":
        raise ValueError(f"{source} has an unsupported interface value-space schema")
    groups = payload.get("profiles")
    if not isinstance(groups, Mapping) or not groups:
        raise ValueError(f"{source} must declare non-empty profiles")
    contracts: dict[str, InterfaceChannelValueSpaceProfile] = {}
    for profile, identifiers in groups.items():
        if not isinstance(profile, str) or not isinstance(identifiers, list):
            raise ValueError(f"{source} has an invalid profile entry")
        if profile not in _INTERFACE_CHANNEL_VALUE_SPACE_PROFILES:
            raise ValueError(f"{source} has unsupported profile {profile!r}")
        typed_profile = profile
        for identifier in identifiers:
            if not isinstance(identifier, str) or not identifier.strip():
                raise ValueError(f"{source} has an invalid channel identifier")
            if identifier in contracts:
                raise ValueError(f"{source} declares channel {identifier!r} in multiple value-space profiles")
            value_space_for_interface_channel_profile(
                typed_profile,
                canonical_unit=(
                    "deg"
                    if typed_profile == "periodic_degrees"
                    else "rad"
                    if typed_profile in {"periodic_radians", "rotation_euler"}
                    else None
                ),
            )
            contracts[identifier] = typed_profile
    return contracts
    ####


def interface_channel_value_space_profile(identifier: str) -> InterfaceChannelValueSpaceProfile | None:
    """Return the public profile or ``None`` for an ad-hoc non-registry channel."""

    return load_interface_channel_value_space_catalog().get(identifier)
    ####


def validate_interface_channel_value_space_coverage(identifiers: Iterable[str]) -> None:
    """Fail closed when a public resolved interface exposes an unlisted channel."""

    contracts = load_interface_channel_value_space_catalog()
    missing = sorted(set(identifiers) - set(contracts))
    if missing:
        raise ValueError("public interface channels lack explicit value-space contracts: " + ", ".join(missing))
    ####


__all__ = [
    "INTERFACE_CHANNEL_VALUE_SPACE_CATALOG",
    "InterfaceChannelValueSpaceProfile",
    "interface_channel_value_space_profile",
    "load_interface_channel_value_space_catalog",
    "validate_interface_channel_value_space_coverage",
    "value_space_for_interface_channel_profile",
]
