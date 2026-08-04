"""Versioned value-space contracts for the declared truth-objective vocabulary."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Literal

import yaml

from .value_space import ValueSpaceSpec, euclidean, periodic_circle
from .vehicle_registry import ROOT

TRUTH_OBJECTIVE_CHANNEL_VALUE_SPACE_CATALOG = ROOT / "verification/truth_objective_channel_value_space_catalog.yaml"

TruthObjectiveValueSpaceProfile = Literal["euclidean_scalar", "periodic_degrees"]


@dataclass(frozen=True, slots=True)
class TruthObjectiveChannelContract:
    """Unit and topology for one generic truth-objective target channel."""

    id: str
    canonical_unit: str
    profile: TruthObjectiveValueSpaceProfile

    @property
    def value_space(self) -> ValueSpaceSpec:
        """Return the declared mathematical space for this target coordinate."""

        if self.profile == "euclidean_scalar":
            return euclidean()
        if self.profile == "periodic_degrees":
            if self.canonical_unit != "deg":
                raise ValueError("periodic-degrees objective channel requires canonical unit 'deg'")
            return periodic_circle(360.0)
        raise ValueError(f"unsupported truth-objective value-space profile {self.profile!r}")
        ####
    ####


@lru_cache(maxsize=4)
def load_truth_objective_channel_value_space_catalog(
    path: str | Path | None = None,
) -> dict[str, TruthObjectiveChannelContract]:
    """Load the generic objective target vocabulary and reject malformed entries."""

    source = Path(path) if path is not None else TRUTH_OBJECTIVE_CHANNEL_VALUE_SPACE_CATALOG
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{source} must contain a mapping")
    if payload.get("schema") != "taoryx.truth-objective-channel-value-space-catalog/v1alpha1":
        raise ValueError(f"{source} has an unsupported truth-objective value-space schema")
    entries = payload.get("channels")
    if not isinstance(entries, Mapping) or not entries:
        raise ValueError(f"{source} must declare non-empty channels")
    contracts: dict[str, TruthObjectiveChannelContract] = {}
    for identifier, raw in entries.items():
        if not isinstance(identifier, str) or not identifier.strip() or not isinstance(raw, Mapping):
            raise ValueError(f"{source} contains an invalid objective channel entry")
        try:
            contract = TruthObjectiveChannelContract(
                id=identifier,
                canonical_unit=raw["canonical_unit"],
                profile=raw["profile"],
            )
        except KeyError as error:
            raise ValueError(f"{source} objective channel {identifier!r} omits {error.args[0]!r}") from error
        contract.value_space
        contracts[identifier] = contract
    return contracts
    ####


def truth_objective_channel_contract(identifier: str) -> TruthObjectiveChannelContract | None:
    """Return a generic contract or ``None`` for a family-owned target channel."""

    return load_truth_objective_channel_value_space_catalog().get(identifier)
    ####


__all__ = [
    "TRUTH_OBJECTIVE_CHANNEL_VALUE_SPACE_CATALOG",
    "TruthObjectiveChannelContract",
    "TruthObjectiveValueSpaceProfile",
    "load_truth_objective_channel_value_space_catalog",
    "truth_objective_channel_contract",
]
