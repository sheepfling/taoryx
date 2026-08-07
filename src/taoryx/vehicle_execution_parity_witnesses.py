"""Reproducible action-trace witnesses for registered batch/episode parity.

The execution catalog can declare a pair parity-ready, but that declaration is
not evidence by itself.  This companion manifest supplies one exact composed
vehicle and short semantic action sequence for every *registered* pair.  It
never manufactures a witness for a pair whose parity availability is absent or
``not_registered``.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .composition_episode import open_vehicle_composition_episode
from .composition_policy import PolicyDecision, replay_serialized_composition_policy_trace, run_composition_policy
from .vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request
from .vehicle_execution_bindings import load_vehicle_batch_episode_parity_catalog
from .vehicle_registry import ROOT

VEHICLE_EXECUTION_PARITY_WITNESSES = ROOT / "verification/vehicle_execution_parity_witnesses.yaml"


class ParityWitnessAction(BaseModel):
    """One held public semantic command for a parity witness."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    values: dict[str, object] = Field(min_length=1)
    duration_s: float = Field(gt=0.0)


class VehicleExecutionParityWitness(BaseModel):
    """One exact registered pair and its deterministic policy trace inputs."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    composition: str = Field(min_length=1)
    authority_profile_id: str = Field(min_length=1)
    observation_profile_id: str | None = None
    seed: int | None = Field(default=None, ge=0)
    integration_step_s: float | None = Field(default=None, gt=0.0)
    actions: tuple[ParityWitnessAction, ...] = Field(min_length=1)


class VehicleExecutionParityWitnessCatalog(BaseModel):
    """Versioned parity-witness manifest."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_id: str = Field(alias="schema", min_length=1)
    version: int = Field(ge=1)
    description: str = Field(min_length=1)
    witnesses: tuple[VehicleExecutionParityWitness, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_unique_ids(self) -> VehicleExecutionParityWitnessCatalog:
        identifiers = tuple(item.id for item in self.witnesses)
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("execution parity witness catalog has duplicate IDs")
        return self
        ####
    ####


@dataclass(frozen=True, slots=True)
class _WitnessKey:
    """Stable identity for a concrete batch/episode pair."""

    family_id: str
    mission: str
    fidelity: str


def load_vehicle_execution_parity_witness_catalog(
    path: str | Path | None = None,
) -> VehicleExecutionParityWitnessCatalog:
    """Load checked-in replay inputs for every registered parity pair."""

    source = Path(path) if path is not None else VEHICLE_EXECUTION_PARITY_WITNESSES
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{source} must contain a mapping")
    return VehicleExecutionParityWitnessCatalog.model_validate(payload)
    ####


def validate_vehicle_execution_parity_witnesses(
    catalog: VehicleExecutionParityWitnessCatalog | None = None,
) -> dict[str, object]:
    """Execute every registered parity witness and reject missing/excess rows."""

    selected = catalog or load_vehicle_execution_parity_witness_catalog()
    declared = load_vehicle_batch_episode_parity_catalog()
    expected = {_WitnessKey(item.family_id, item.mission, item.fidelity) for item in declared.bindings}
    observed: dict[_WitnessKey, str] = {}
    records: list[dict[str, object]] = []
    errors: list[str] = []
    for witness in selected.witnesses:
        source = ROOT / witness.composition
        if not source.is_file():
            errors.append(f"{witness.id}: composition request is missing: {witness.composition}")
            continue
        try:
            composition = compile_vehicle_composition(load_vehicle_composition_request(source))
            key = _WitnessKey(composition.family_id, composition.mission, composition.fidelity)
            if key in observed:
                errors.append(f"{witness.id}: duplicates parity witness {observed[key]!r}")
                continue
            observed[key] = witness.id
            if key not in expected:
                errors.append(f"{witness.id}: composition has no registered parity declaration")
                continue
            episode = (
                open_vehicle_composition_episode(composition, seed=witness.seed)
                if witness.integration_step_s is None
                else open_vehicle_composition_episode(
                    composition,
                    seed=witness.seed,
                    integration_step_s=witness.integration_step_s,
                )
            )
            try:
                actions = iter(witness.actions)

                def policy(_observation: object, _contract: object) -> PolicyDecision | None:
                    action = next(actions, None)
                    return None if action is None else PolicyDecision(action.values, action.duration_s)
                    ####

                trace = run_composition_policy(
                    episode,
                    policy,
                    authority_profile_id=witness.authority_profile_id,
                    observation_profile_id=witness.observation_profile_id,
                )
            finally:
                episode.close()
            replay_report = replay_serialized_composition_policy_trace(composition, trace.as_dict())
            replay_payload = replay_report.as_dict()
            parity_payload = replay_payload["batch_episode_parity"]
            if not isinstance(parity_payload, Mapping):
                errors.append(f"{witness.id}: replay report did not include a parity disposition")
                parity_payload = {}
            if replay_report.status != "pass":
                errors.append(f"{witness.id}: replay report returned {replay_report.status}")
            parity_report = parity_payload.get("report")
            if not isinstance(parity_report, Mapping) or parity_report.get("status") != "pass":
                errors.append(f"{witness.id}: parity report was not a passing registered report")
            records.append(
                {
                    "id": witness.id,
                    "composition": witness.composition,
                    "family_id": composition.family_id,
                    "mission": composition.mission,
                    "fidelity": composition.fidelity,
                    "authority_profile_id": witness.authority_profile_id,
                    "action_count": len(witness.actions),
                    "adapter_id": parity_report.get("adapter_id") if isinstance(parity_report, Mapping) else None,
                    "replay_report": replay_payload,
                    "status": replay_report.status,
                }
            )
        except (TypeError, ValueError, RuntimeError) as error:
            errors.append(f"{witness.id}: parity witness failed: {error}")
    for key in sorted(expected, key=lambda item: (item.family_id, item.mission, item.fidelity)):
        if key not in observed:
            errors.append(f"registered parity binding has no replay witness: {key.family_id}/{key.mission}/{key.fidelity}")
    return {
        "schema": "taoryx.vehicle-execution-parity-witness-report/v1alpha1",
        "status": "pass" if not errors else "fail",
        "registered_binding_count": len(expected),
        "witness_count": len(selected.witnesses),
        "records": records,
        "errors": errors,
        "claim_boundary": (
            "This replays one short declared semantic action trace through every registered batch/episode pair. "
            "It does not establish parity for unregistered pairs, physical-effector realization, robustness, or qualification."
        ),
    }
    ####


__all__ = [
    "VEHICLE_EXECUTION_PARITY_WITNESSES",
    "VehicleExecutionParityWitness",
    "VehicleExecutionParityWitnessCatalog",
    "load_vehicle_execution_parity_witness_catalog",
    "validate_vehicle_execution_parity_witnesses",
]
