"""Reproducible action-trace witnesses for registered batch/episode parity.

The execution catalog can declare a pair parity-ready, but that declaration is
not evidence by itself.  This companion manifest supplies one exact composed
vehicle and short semantic action sequence for every *registered* pair.  It
never manufactures a witness for a pair whose parity availability is absent or
``not_registered``.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .composition_episode import open_vehicle_composition_episode
from .composition_policy import PolicyDecision, replay_serialized_composition_policy_trace, run_composition_policy
from .plugins import PluginCatalog
from .vehicle_catalog_resources import vehicle_catalog_resource, vehicle_catalog_resources
from .vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request
from .vehicle_execution_bindings import load_vehicle_batch_episode_parity_catalog, load_vehicle_execution_binding_catalog

# Materialized lazily by ``__getattr__`` only for compatibility consumers.
VEHICLE_EXECUTION_PARITY_WITNESSES: Path

_EMPTY_PARITY_WITNESS_CATALOG: dict[str, object] = {
    "schema": "taoryx.vehicle-execution-parity-witnesses/v1alpha1",
    "version": 1,
    "description": "No selected vehicle plug-in declares a registered batch/episode parity witness.",
    "witnesses": [],
}


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
    witnesses: tuple[VehicleExecutionParityWitness, ...] = ()

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
    *,
    plugins: PluginCatalog | None = None,
) -> VehicleExecutionParityWitnessCatalog:
    """Load replay inputs from the selected family-owned package fragments.

    A selected plug-in with no registered parity pair intentionally produces an
    empty catalog.  It never borrows a witness from an aggregate package; a
    declared parity pair without its package-owned witness still fails the
    validator below.
    """

    if path is not None:
        return _load_vehicle_execution_parity_witness_catalog_sources((str(Path(path)),))
    sources = vehicle_catalog_resources(
        "verification/vehicle_execution_parity_witnesses.yaml",
        plugins=plugins,
        required=False,
    )
    return _load_vehicle_execution_parity_witness_catalog_sources(tuple(str(source) for source in sources))
    ####


@lru_cache(maxsize=32)
def _load_vehicle_execution_parity_witness_catalog_sources(
    source_names: tuple[str, ...],
) -> VehicleExecutionParityWitnessCatalog:
    """Parse one cacheable, exact set of parity-witness fragments."""

    if not source_names:
        return VehicleExecutionParityWitnessCatalog.model_validate(_EMPTY_PARITY_WITNESS_CATALOG)
    payloads: list[Mapping[str, object]] = []
    for source_name in source_names:
        source = Path(source_name)
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError(f"{source} must contain a mapping")
        payloads.append(payload)
    merged = dict(payloads[0])
    rows: list[object] = []
    for payload in payloads:
        witnesses = payload.get("witnesses", [])
        if not isinstance(witnesses, list):
            raise ValueError("vehicle execution parity witness catalog must contain a witnesses list")
        rows.extend(witnesses)
    merged["witnesses"] = rows
    return VehicleExecutionParityWitnessCatalog.model_validate(merged)
    ####


def validate_vehicle_execution_parity_witnesses(
    catalog: VehicleExecutionParityWitnessCatalog | None = None,
    *,
    family_ids: Iterable[str] | None = None,
    plugins: PluginCatalog | None = None,
) -> dict[str, object]:
    """Execute selected registered parity witnesses and reject missing/excess rows.

    A focused ``plugins`` catalog is carried through both episode construction
    and trace replay so this catalog gate never has to rediscover unrelated
    model packages.
    """

    selected = catalog or load_vehicle_execution_parity_witness_catalog(plugins=plugins)
    declared = load_vehicle_batch_episode_parity_catalog(plugins=plugins)
    family_filter = None if family_ids is None else frozenset(str(item).strip() for item in family_ids if str(item).strip())
    if family_filter == frozenset():
        raise ValueError("family_ids must contain at least one non-empty vehicle family ID")
    known_families = {item.family_id for item in load_vehicle_execution_binding_catalog(plugins=plugins).bindings}
    unknown_families = () if family_filter is None else tuple(sorted(family_filter - known_families))
    expected = {
        _WitnessKey(item.family_id, item.mission, item.fidelity)
        for item in declared.bindings
        if family_filter is None or item.family_id in family_filter
    }
    observed: dict[_WitnessKey, str] = {}
    records: list[dict[str, object]] = []
    errors: list[str] = [f"unknown requested vehicle family: {item}" for item in unknown_families]
    selected_witness_count = 0
    for witness in selected.witnesses:
        try:
            source = vehicle_catalog_resource(witness.composition, plugins=plugins)
        except FileNotFoundError:
            errors.append(f"{witness.id}: composition request is missing: {witness.composition}")
            continue
        if not source.is_file():
            if family_filter is None:
                errors.append(f"{witness.id}: composition request is missing: {witness.composition}")
            continue
        try:
            request = load_vehicle_composition_request(source)
            if family_filter is not None and request.vehicle not in family_filter:
                continue
            selected_witness_count += 1
            composition = compile_vehicle_composition(request, plugins=plugins)
            key = _WitnessKey(composition.family_id, composition.mission, composition.fidelity)
            if key in observed:
                errors.append(f"{witness.id}: duplicates parity witness {observed[key]!r}")
                continue
            observed[key] = witness.id
            if key not in expected:
                errors.append(f"{witness.id}: composition has no registered parity declaration")
                continue
            episode = (
                open_vehicle_composition_episode(composition, seed=witness.seed, plugins=plugins)
                if witness.integration_step_s is None
                else open_vehicle_composition_episode(
                    composition,
                    seed=witness.seed,
                    integration_step_s=witness.integration_step_s,
                    plugins=plugins,
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
            replay_report = replay_serialized_composition_policy_trace(
                composition,
                trace.as_dict(),
                plugins=plugins,
            )
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
        "witness_count": selected_witness_count,
        "family_filter": None if family_filter is None else sorted(family_filter),
        "records": records,
        "errors": errors,
        "claim_boundary": (
            "This replays one short declared semantic action trace through every registered batch/episode pair. "
            "It does not establish parity for unregistered pairs, physical-effector realization, robustness, or qualification."
        ),
    }
    ####


def __getattr__(name: str) -> object:
    """Resolve the historical parity-witness path only for compatibility users."""

    if name != "VEHICLE_EXECUTION_PARITY_WITNESSES":
        raise AttributeError(name)
    from .compatibility.vehicle_catalog_resources import legacy_vehicle_catalog_resource

    value = legacy_vehicle_catalog_resource("verification/vehicle_execution_parity_witnesses.yaml")
    globals()[name] = value
    return value
    ####


__all__ = [
    "VEHICLE_EXECUTION_PARITY_WITNESSES",
    "VehicleExecutionParityWitness",
    "VehicleExecutionParityWitnessCatalog",
    "load_vehicle_execution_parity_witness_catalog",
    "validate_vehicle_execution_parity_witnesses",
]
