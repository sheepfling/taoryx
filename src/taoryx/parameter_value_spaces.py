"""Explicit topology contracts for Mission Composition parameters.

The public vehicle-composition registry must not infer a mathematical space
from a suffix such as ``_m``.  A coordinate in metres is signed; a duration in
seconds is nonnegative.  This catalog supplies that semantic distinction and
is validated against every advertised initialization and segment parameter at
registry load time.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Literal

import yaml

from .plugins.resources import packaged_resource_fallback
from .value_space import ValueSpaceSpec, bounded_interval, euclidean, finite_set, periodic_circle, positive_half_line, quaternion_so3
from .vehicle_registry import ROOT

PARAMETER_VALUE_SPACE_CATALOG = packaged_resource_fallback(
    ROOT / "verification/parameter_value_space_catalog.yaml",
    package="taoryx",
    resource="data/verification/parameter_value_space_catalog.yaml",
)

ParameterValueType = Literal["scalar", "vector3", "vector4", "enum"]
ParameterValueSpaceProfile = Literal[
    "bounded_interval",
    "euclidean_scalar",
    "euclidean_vector3",
    "finite_set",
    "periodic_degrees",
    "positive_half_line",
    "quaternion_so3",
]


@dataclass(frozen=True, slots=True)
class ParameterValueSpaceContract:
    """One explicit public parameter representation and mathematical space."""

    id: str
    value_type: ParameterValueType
    profile: ParameterValueSpaceProfile
    frame: str | None = None

    @property
    def value_space(self) -> ValueSpaceSpec:
        """Return the immutable value-space contract selected by this profile."""

        return value_space_for_parameter_profile(self.profile)
        ####
    ####


def value_space_for_parameter_profile(profile: ParameterValueSpaceProfile) -> ValueSpaceSpec:
    """Resolve one catalog profile without making an identifier-based guess."""

    profiles: dict[ParameterValueSpaceProfile, ValueSpaceSpec] = {
        "bounded_interval": bounded_interval(),
        "euclidean_scalar": euclidean(),
        "euclidean_vector3": euclidean(3),
        "finite_set": finite_set(),
        "periodic_degrees": periodic_circle(360.0),
        "positive_half_line": positive_half_line(),
        "quaternion_so3": quaternion_so3(),
    }
    return profiles[profile]
    ####


def _validate_contract(contract: ParameterValueSpaceContract) -> None:
    expected_type: dict[ParameterValueSpaceProfile, ParameterValueType] = {
        "bounded_interval": "scalar",
        "euclidean_scalar": "scalar",
        "euclidean_vector3": "vector3",
        "finite_set": "enum",
        "periodic_degrees": "scalar",
        "positive_half_line": "scalar",
        "quaternion_so3": "vector4",
    }
    if contract.value_type != expected_type[contract.profile]:
        raise ValueError(
            f"parameter value-space {contract.id!r} profile {contract.profile!r} requires "
            f"value_type {expected_type[contract.profile]!r}, not {contract.value_type!r}"
        )
    ####


@lru_cache(maxsize=4)
def load_parameter_value_space_catalog(
    path: str | Path | None = None,
) -> dict[str, ParameterValueSpaceContract]:
    """Load the versioned catalog of explicitly declared parameter spaces."""

    source = Path(path) if path is not None else PARAMETER_VALUE_SPACE_CATALOG
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{source} must contain a mapping")
    if payload.get("schema") != "taoryx.parameter-value-space-catalog/v1alpha1":
        raise ValueError(f"{source} has an unsupported parameter value-space schema")
    entries = payload.get("parameters")
    if not isinstance(entries, Mapping) or not entries:
        raise ValueError(f"{source} must declare a non-empty parameters mapping")
    contracts: dict[str, ParameterValueSpaceContract] = {}
    for identifier, raw in entries.items():
        if not isinstance(identifier, str) or not identifier.strip() or not isinstance(raw, Mapping):
            raise ValueError(f"{source} contains an invalid parameter value-space entry")
        try:
            contract = ParameterValueSpaceContract(
                id=identifier,
                value_type=raw["value_type"],
                profile=raw["profile"],
                frame=raw.get("frame"),
            )
        except KeyError as error:
            raise ValueError(f"{source} parameter {identifier!r} omits {error.args[0]!r}") from error
        _validate_contract(contract)
        contracts[identifier] = contract
    return contracts
    ####


def parameter_value_space_contract(identifier: str) -> ParameterValueSpaceContract | None:
    """Return a declared space or ``None`` for an ad-hoc, non-registry value."""

    return load_parameter_value_space_catalog().get(identifier)
    ####


def validate_parameter_value_space_coverage(identifiers: Iterable[str]) -> None:
    """Fail closed if a public registry parameter lacks a catalog contract."""

    contracts = load_parameter_value_space_catalog()
    missing = sorted(set(identifiers) - set(contracts))
    if missing:
        raise ValueError(
            "public vehicle-composition parameters lack explicit value-space contracts: " + ", ".join(missing)
        )
    ####


__all__ = [
    "PARAMETER_VALUE_SPACE_CATALOG",
    "ParameterValueSpaceContract",
    "ParameterValueSpaceProfile",
    "ParameterValueType",
    "load_parameter_value_space_catalog",
    "parameter_value_space_contract",
    "validate_parameter_value_space_coverage",
    "value_space_for_parameter_profile",
]
