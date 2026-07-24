"""Selectable X-15 maneuver catalog with explicit segment-quality gates.

The catalog is a low-code menu for focused segment work.  A ``settled`` row
means that the declared fixture has passed its focused segment gates; it does
not promote the row into a complete route or claim flight performance.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

X15ManeuverKind = Literal[
    "powered_ascent",
    "release_coast",
    "bank_energy_management",
    "phugoid_alpha_profile",
    "weave",
    "terminal_pronav",
]
X15ManeuverMode = Literal["point_mass_3dof", "kinematic_3_plus_3", "rigid_body_6dof"]
X15ManeuverStatus = Literal["settled", "candidate", "blocked"]
QualityGateStatus = Literal["pass", "pending", "not_applicable"]


class X15QualityGate(BaseModel):
    """One focused quality gate and its traceable evidence pointer."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    name: str = Field(min_length=1)
    status: QualityGateStatus
    evidence: str = Field(min_length=1)
    ####


class X15ManeuverSpec(BaseModel):
    """A selectable X-15 fixture and the evidence required to use it."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    display_name: str = Field(min_length=1)
    kind: X15ManeuverKind
    mode: X15ManeuverMode
    source_problem: str = Field(min_length=1)
    tables: tuple[str, ...] = Field(min_length=1)
    focused_test: str = Field(min_length=1)
    status: X15ManeuverStatus
    quality_gates: tuple[X15QualityGate, ...] = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_gates(self) -> X15ManeuverSpec:
        names = [gate.name for gate in self.quality_gates]
        if len(names) != len(set(names)):
            raise ValueError(f"maneuver {self.id!r} repeats a quality gate")
        if self.status == "settled" and any(gate.status != "pass" for gate in self.quality_gates):
            raise ValueError(f"settled maneuver {self.id!r} must have only passing quality gates")
        return self
        ####

    @property
    def quality_gates_pass(self) -> bool:
        """Return whether every declared focused gate passed."""

        return all(gate.status == "pass" for gate in self.quality_gates)
        ####


class X15ManeuverCatalog(BaseModel):
    """Typed menu of X-15 maneuvers available to the composition layer."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: int = 1
    id: str = Field(min_length=1)
    vehicle: Literal["x15"] = "x15"
    description: str = Field(min_length=1)
    maneuvers: tuple[X15ManeuverSpec, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def unique_ids(self) -> X15ManeuverCatalog:
        identifiers = [maneuver.id for maneuver in self.maneuvers]
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("X-15 maneuver IDs must be unique")
        return self
        ####

    def select(self, status: X15ManeuverStatus) -> tuple[X15ManeuverSpec, ...]:
        """Return maneuvers matching a lifecycle status."""

        return tuple(maneuver for maneuver in self.maneuvers if maneuver.status == status)
        ####

    def get(self, identifier: str) -> X15ManeuverSpec:
        """Return one maneuver by ID, raising a useful error when absent."""

        for maneuver in self.maneuvers:
            if maneuver.id == identifier:
                return maneuver
        raise KeyError(f"unknown X-15 maneuver {identifier!r}")
        ####


def load_x15_maneuver_catalog(path: str | Path) -> X15ManeuverCatalog:
    """Load and validate a selectable X-15 maneuver catalog from YAML."""

    catalog_path = Path(path)
    payload = yaml.safe_load(catalog_path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"X-15 maneuver catalog must be a mapping: {catalog_path}")
    raw_maneuvers = payload.get("maneuvers", ())
    if not isinstance(raw_maneuvers, Sequence) or isinstance(raw_maneuvers, (str, bytes)):
        raise ValueError(f"X-15 maneuver catalog maneuvers must be a sequence: {catalog_path}")
    if len(raw_maneuvers) != len([item for item in raw_maneuvers if isinstance(item, Mapping)]):
        raise ValueError(f"X-15 maneuver catalog contains a non-mapping maneuver: {catalog_path}")
    vehicle = str(payload.get("vehicle", "x15"))
    if vehicle != "x15":
        raise ValueError(f"X-15 maneuver catalog vehicle must be 'x15': {catalog_path}")
    return X15ManeuverCatalog(
        schema_version=int(payload.get("schema_version", 1)),
        id=str(payload.get("id", catalog_path.stem)),
        vehicle="x15",
        description=str(payload.get("description", "Selectable X-15 maneuvers")),
        maneuvers=tuple(X15ManeuverSpec(**item) for item in raw_maneuvers),
    )
    ####


def settled_x15_maneuvers(path: str | Path) -> tuple[X15ManeuverSpec, ...]:
    """Load the catalog and return only maneuvers ready for composition."""

    return load_x15_maneuver_catalog(path).select("settled")
    ####


__all__ = [
    "QualityGateStatus",
    "X15ManeuverCatalog",
    "X15ManeuverKind",
    "X15ManeuverMode",
    "X15ManeuverSpec",
    "X15ManeuverStatus",
    "X15QualityGate",
    "load_x15_maneuver_catalog",
    "settled_x15_maneuvers",
]
