"""Explicit execution bindings for immutable vehicle compositions.

The composition registry answers what a user may request.  This companion
catalog answers the separate operational question: which selected requests
currently have a source-owned batch runner or accepted-truth episode factory?
It intentionally has no family fallback.  A missing binding is actionable
onboarding work, not permission to run a neighboring vehicle model.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .fidelity_contracts import FidelityTier
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_registry import ROOT

VEHICLE_EXECUTION_BINDINGS = ROOT / "verification/vehicle_execution_bindings.yaml"

ExecutionOperation = Literal["batch", "episode"]
ExecutionBindingStatus = Literal["runnable", "planned"]


class VehicleExecutionBinding(BaseModel):
    """One exact composition-to-executable-factory declaration."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    family_id: str = Field(min_length=1)
    mission: str = Field(min_length=1)
    fidelity: FidelityTier
    operation: ExecutionOperation
    status: ExecutionBindingStatus
    factory_id: str | None = None
    description: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)
    blockers: tuple[str, ...] = ()

    @model_validator(mode="after")
    def validate_factory_contract(self) -> VehicleExecutionBinding:
        if self.status == "runnable" and self.factory_id is None:
            raise ValueError("a runnable execution binding requires factory_id")
        if self.status == "planned" and not self.blockers:
            raise ValueError("a planned execution binding requires at least one blocker")
        return self
        ####
####


class VehicleExecutionBindingCatalog(BaseModel):
    """Versioned execution-binding authority."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_id: str = Field(alias="schema", min_length=1)
    version: int = Field(ge=1)
    description: str = Field(min_length=1)
    bindings: tuple[VehicleExecutionBinding, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_unique_bindings(self) -> VehicleExecutionBindingCatalog:
        keys = tuple((item.family_id, item.mission, item.fidelity, item.operation) for item in self.bindings)
        duplicates = sorted({item for item in keys if keys.count(item) > 1})
        if duplicates:
            raise ValueError(f"execution binding catalog has duplicate keys: {duplicates}")
        return self
        ####
####


class VehicleExecutionBindingError(ValueError):
    """Fail-closed diagnostic for unavailable composition execution."""

    def __init__(
        self,
        composition: CompiledVehicleComposition,
        operation: ExecutionOperation,
        reason: str,
    ) -> None:
        self.composition = composition
        self.operation = operation
        self.reason = reason
        super().__init__(
            f"no {operation} execution binding for {composition.family_id!r}/"
            f"{composition.mission!r}/{composition.fidelity!r}: {reason}"
        )
        ####
    ####


def load_vehicle_execution_binding_catalog(
    path: str | Path | None = None,
) -> VehicleExecutionBindingCatalog:
    """Load the versioned execution capability declaration."""

    source = Path(path) if path is not None else VEHICLE_EXECUTION_BINDINGS
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{source} must contain a mapping")
    return VehicleExecutionBindingCatalog.model_validate(payload)
    ####


def bindings_for_family(
    family_id: str,
    *,
    catalog: VehicleExecutionBindingCatalog | None = None,
) -> tuple[VehicleExecutionBinding, ...]:
    """Return all explicit runnable and planned bindings for one family."""

    selected = catalog or load_vehicle_execution_binding_catalog()
    return tuple(item for item in selected.bindings if item.family_id == family_id)
    ####


def resolve_vehicle_execution_binding(
    composition: CompiledVehicleComposition,
    operation: ExecutionOperation,
    *,
    catalog: VehicleExecutionBindingCatalog | None = None,
) -> VehicleExecutionBinding:
    """Resolve an exact runnable factory binding or return its declared gap."""

    selected = catalog or load_vehicle_execution_binding_catalog()
    matches = tuple(
        item
        for item in selected.bindings
        if (
            item.family_id == composition.family_id
            and item.mission == composition.mission
            and item.fidelity == composition.fidelity
            and item.operation == operation
        )
    )
    if not matches:
        raise VehicleExecutionBindingError(composition, operation, "the catalog has no binding for this exact composition")
    binding = matches[0]
    if binding.status != "runnable":
        raise VehicleExecutionBindingError(composition, operation, "; ".join(binding.blockers))
    return binding
    ####


def execution_binding_records(
    family_id: str,
    *,
    catalog: VehicleExecutionBindingCatalog | None = None,
) -> list[dict[str, object]]:
    """Serialize bindings for discovery without constructing any runtime."""

    return [item.model_dump(mode="json") for item in bindings_for_family(family_id, catalog=catalog)]
    ####


def validate_execution_bindings(
    bindings: Iterable[VehicleExecutionBinding],
    *,
    declarations: Mapping[str, object],
) -> tuple[str, ...]:
    """Check bindings against a compact family/mission/fidelity declaration.

    ``declarations`` uses the simple structure emitted by the composition
    registry: ``family_id -> {mission_id: supported_fidelity_names}``.  Keeping
    this validator independent prevents a registry/catalog import cycle while
    still making stale execution declarations fail in tests and release checks.
    """

    errors: list[str] = []
    for binding in bindings:
        family = declarations.get(binding.family_id)
        if not isinstance(family, Mapping):
            errors.append(f"unknown execution-binding family: {binding.family_id}")
            continue
        supported = family.get(binding.mission)
        if not isinstance(supported, set | frozenset | tuple | list):
            errors.append(f"unknown execution-binding mission: {binding.family_id}/{binding.mission}")
            continue
        if binding.fidelity not in supported:
            errors.append(
                f"execution binding declares unsupported fidelity: "
                f"{binding.family_id}/{binding.mission}/{binding.fidelity}"
            )
    return tuple(errors)
    ####


__all__ = [
    "ExecutionBindingStatus",
    "ExecutionOperation",
    "VEHICLE_EXECUTION_BINDINGS",
    "VehicleExecutionBinding",
    "VehicleExecutionBindingCatalog",
    "VehicleExecutionBindingError",
    "bindings_for_family",
    "execution_binding_records",
    "load_vehicle_execution_binding_catalog",
    "resolve_vehicle_execution_binding",
    "validate_execution_bindings",
]
