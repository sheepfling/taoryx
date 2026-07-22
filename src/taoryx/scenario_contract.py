"""Canonical scenario identity and parity-comparison contracts."""

from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

DynamicsTier = Literal["3dof", "pseudo_6dof", "6dof"]


class ScenarioContract(BaseModel):
    """Inputs that must match before fidelity histories are compared."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: int = Field(default=1, ge=1)
    scenario_id: str = Field(min_length=1)
    vehicle: str = Field(min_length=1)
    family: str = Field(min_length=1)
    dynamics_tier: DynamicsTier
    initial_state_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    environment_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    vehicle_model_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    propulsion_model_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    mass_model_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    command_history_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    event_schedule_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    termination_policy_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    duration_s: float = Field(gt=0.0)
    unit_system: Literal["si"] = "si"
    integrator: str = Field(min_length=1)
    output_rate_hz: float = Field(gt=0.0)

    def comparison_payload(self) -> dict[str, object]:
        """Return all physical experiment fields except the fidelity tier."""

        payload = self.model_dump(mode="json")
        payload.pop("dynamics_tier", None)
        return payload
        ####

    def digest(self, *, include_tier: bool = True) -> str:
        """Return a deterministic full or parity identity hash."""

        payload: Mapping[str, object]
        if include_tier:
            payload = self.model_dump(mode="json")
        else:
            payload = self.comparison_payload()
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()
        ####

    def write_json(self, path: str | Path) -> Path:
        """Write a canonical contract envelope for packet tooling."""

        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        payload = self.model_dump(mode="json")
        payload["contract_sha256"] = self.digest()
        payload["parity_sha256"] = self.digest(include_tier=False)
        destination.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return destination
        ####

    @classmethod
    def read_json(cls, path: str | Path) -> ScenarioContract:
        """Read a contract and reject a tampered or stale digest."""

        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        expected = payload.pop("contract_sha256", None)
        parity = payload.pop("parity_sha256", None)
        contract = cls.model_validate(payload)
        if expected != contract.digest() or parity != contract.digest(include_tier=False):
            raise ValueError(f"scenario contract digest mismatch: {path}")
        return contract
        ####


def compare_contracts(left: ScenarioContract, right: ScenarioContract) -> dict[str, object]:
    """Compare two runs and refuse mismatched physical experiments."""

    left_payload = left.comparison_payload()
    right_payload = right.comparison_payload()
    mismatches = {
        key: {"left": left_payload[key], "right": right_payload[key]}
        for key in left_payload
        if left_payload[key] != right_payload[key]
    }
    return {
        "comparable": not mismatches,
        "mismatches": mismatches,
        "left_tier": left.dynamics_tier,
        "right_tier": right.dynamics_tier,
        "parity_sha256": left.digest(include_tier=False) if not mismatches else None,
        "left_sha256": left.digest(),
        "right_sha256": right.digest(),
    }
    ####
