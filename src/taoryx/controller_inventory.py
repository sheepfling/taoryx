"""Machine-readable inventory of controller paths and qualification status."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

ControllerInventoryClassification = Literal[
    "mission_logic",
    "guidance",
    "primary_regulator",
    "integral_augmentation",
    "allocator_mixer",
    "actuator_servo",
    "safety_limiter",
    "legacy_or_special_case",
]
ControllerInventoryStatus = Literal["baseline", "candidate", "wiring_verified", "qualified", "blocked"]


class ControllerInventoryEntry(BaseModel):
    """One controller-like loop that can influence a vehicle run."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    family: str = Field(min_length=1)
    vehicle_id: str = Field(min_length=1)
    classification: ControllerInventoryClassification
    implementation: str = Field(min_length=1)
    source: str = Field(min_length=1)
    path: tuple[str, ...] = Field(min_length=1)
    design_id: str | None = None
    active_runtime: bool = False
    legacy_baseline: bool = False
    scenario_gain_overrides: bool = False
    qualification_eligible: bool = False
    gain_owner: str = Field(min_length=1)
    declared_gains: tuple[str, ...] = ()
    notes: str = ""

    @model_validator(mode="after")
    def validate_policy(self) -> ControllerInventoryEntry:
        if self.qualification_eligible and self.scenario_gain_overrides:
            raise ValueError(f"inventory entry {self.id!r} cannot qualify with scenario gain overrides")
        if self.legacy_baseline and not self.qualification_eligible:
            return self
        if self.active_runtime and not self.path:
            raise ValueError(f"active inventory entry {self.id!r} must declare a control path")
        return self
        ####
    ####


class ControllerInventory(BaseModel):
    """Validated inventory used before controller migration or qualification."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: int = Field(default=1, ge=1)
    id: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)
    entries: tuple[ControllerInventoryEntry, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_entries(self) -> ControllerInventory:
        ids = [entry.id for entry in self.entries]
        if len(ids) != len(set(ids)):
            raise ValueError("controller inventory IDs must be unique")
        active = [entry for entry in self.entries if entry.active_runtime]
        if not active:
            raise ValueError("controller inventory must identify at least one active runtime path")
        for entry in active:
            if entry.classification == "primary_regulator" and entry.implementation == "unspecified":
                raise ValueError(f"active primary regulator {entry.id!r} is unclassified")
        return self
        ####
    ####


__all__ = [
    "ControllerInventory",
    "ControllerInventoryClassification",
    "ControllerInventoryEntry",
    "ControllerInventoryStatus",
]
####
