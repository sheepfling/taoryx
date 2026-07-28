"""Declarative trim-spec catalog loading.

The catalog describes solver variables and numerical policy.  It deliberately
does not contain vehicle equations; those remain source-backed Python plant
residual adapters.
"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .trim import TrimGate, TrimProcedure, TrimSpec


class TrimProcedurePolicy(BaseModel):
    """Declarative deterministic search policy for a trim entry."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    multi_start: int = Field(default=1, ge=1)
    seed: int = 0
    perturbation_fraction: float = Field(default=0.10, ge=0.0, le=1.0)
    max_nfev: int = Field(default=2000, ge=1)
    residual_tolerance: float = Field(default=1.0e-8, gt=0.0)
    acceptance_tolerance: float | None = Field(default=None, gt=0.0)
    continuation_axis: str | None = None
    continuation_values: tuple[float, ...] = ()

    @model_validator(mode="after")
    def validate_continuation(self) -> TrimProcedurePolicy:
        if self.continuation_values and self.continuation_axis is None:
            raise ValueError("trim continuation values require a continuation axis")
        return self
        ####
    ####


class TrimGateSpec(BaseModel):
    """Declarative metric gate attached to a trim procedure."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    metric: str = Field(min_length=1)
    comparison: Literal["minimum", "maximum", "equal"]
    limit: float
    unit: str = Field(min_length=1)

    def to_gate(self) -> TrimGate:
        """Convert the catalog entry to the solver contract."""

        return TrimGate(self.id, self.metric, self.comparison, self.limit, self.unit)
        ####


class TrimCatalogEntry(BaseModel):
    """One declarative trim specification."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    vehicle: str = Field(min_length=1)
    status: Literal["ready", "source_only"] = "ready"
    state_names: tuple[str, ...] = ()
    control_names: tuple[str, ...] = ()
    residual_names: tuple[str, ...] = Field(min_length=1)
    state_initial: Mapping[str, float] = {}
    control_initial: Mapping[str, float] = {}
    state_lower: Mapping[str, float] = {}
    state_upper: Mapping[str, float] = {}
    control_lower: Mapping[str, float] = {}
    control_upper: Mapping[str, float] = {}
    residual_scales: Mapping[str, float] = {}
    x_scale: Mapping[str, float] = {}
    notes: str = ""
    fidelity: str = "rigid_body_6dof"
    operating_point: Mapping[str, float | str] = {}
    provenance: Mapping[str, str] = {}
    solver: TrimProcedurePolicy = Field(default_factory=TrimProcedurePolicy)
    gates: tuple[TrimGateSpec, ...] = ()

    @model_validator(mode="after")
    def validate_channels(self) -> TrimCatalogEntry:
        if set(self.state_names) & set(self.control_names):
            raise ValueError(f"trim catalog entry {self.id!r} overlaps state and control names")
        for name in self.state_names:
            if name not in self.state_initial:
                raise ValueError(f"trim catalog entry {self.id!r} lacks initial state {name!r}")
        for name in self.control_names:
            if name not in self.control_initial:
                raise ValueError(f"trim catalog entry {self.id!r} lacks initial control {name!r}")
        unknown_scales = set(self.residual_scales) - set(self.residual_names)
        if unknown_scales:
            raise ValueError(f"trim catalog entry {self.id!r} scales unknown residuals: {sorted(unknown_scales)}")
        return self
        ####
    ####

    def to_spec(self) -> TrimSpec:
        """Convert the declarative entry to the solver contract."""

        return TrimSpec(
            state_names=self.state_names,
            control_names=self.control_names,
            residual_names=self.residual_names,
            state_initial=self.state_initial,
            control_initial=self.control_initial,
            state_lower=self.state_lower,
            state_upper=self.state_upper,
            control_lower=self.control_lower,
            control_upper=self.control_upper,
            residual_scales=self.residual_scales,
            x_scale=self.x_scale,
            operating_point=self.operating_point,
        )
        ####

    def to_procedure(self) -> TrimProcedure:
        """Convert the declarative entry to the common procedure contract."""

        return TrimProcedure(
            id=self.id,
            vehicle=self.vehicle,
            fidelity=self.fidelity,
            spec=self.to_spec(),
            operating_point=self.operating_point,
            provenance=self.provenance,
            multi_start=self.solver.multi_start,
            seed=self.solver.seed,
            perturbation_fraction=self.solver.perturbation_fraction,
            max_nfev=self.solver.max_nfev,
            residual_tolerance=self.solver.residual_tolerance,
            acceptance_tolerance=self.solver.acceptance_tolerance,
            continuation_axis=self.solver.continuation_axis,
            continuation_values=self.solver.continuation_values,
            gates=tuple(gate.to_gate() for gate in self.gates),
        )
        ####


class TrimCatalog(BaseModel):
    """Validated collection of declarative trim specifications."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: int = Field(ge=1)
    id: str = Field(min_length=1)
    entries: tuple[TrimCatalogEntry, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def unique_ids(self) -> TrimCatalog:
        ids = [entry.id for entry in self.entries]
        if len(set(ids)) != len(ids):
            raise ValueError("trim catalog IDs must be unique")
        return self
        ####
    ####

    def get(self, identifier: str) -> TrimCatalogEntry:
        """Return one trim entry by stable identifier."""

        for entry in self.entries:
            if entry.id == identifier:
                return entry
        raise KeyError(f"unknown trim catalog entry {identifier!r}")
        ####


def load_trim_catalog(path: Path) -> TrimCatalog:
    """Load and validate a YAML trim catalog."""

    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"trim catalog must be a mapping: {path}")
    return TrimCatalog.model_validate(payload)
    ####
