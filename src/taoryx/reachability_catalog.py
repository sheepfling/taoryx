"""Typed access to the Alpha 3 reachability profile catalog."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator


class ReachabilityStudySemantic(BaseModel):
    """A declared study semantic label and its meaning."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    meaning: str = Field(min_length=1)
    ####


class ReachabilityCommonObject(BaseModel):
    """The common mathematical contract for all reachability studies."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    mathematical_form: str = Field(min_length=1)
    required_study_identity: tuple[str, ...] = Field(min_length=1)
    required_result_products: tuple[str, ...] = Field(min_length=1)
    default_semantics: str = Field(min_length=1)
    default_policy: str = Field(min_length=1)
    ####


class ReachabilityProfileSpec(BaseModel):
    """One archetype-specific reachability profile."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    archetype: str = Field(min_length=1)
    status: str = Field(min_length=1)
    phases: tuple[str, ...] = Field(min_length=1)
    state_dimensions: tuple[str, ...] = Field(min_length=1)
    decisions: tuple[str, ...] = ()
    constraints: tuple[str, ...] = Field(min_length=1)
    coordinates: tuple[str, ...] = Field(min_length=1)
    products: tuple[str, ...] = Field(min_length=1)
    limiting_factors: tuple[str, ...] = ()
    configurations: tuple[str, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_layout(self) -> ReachabilityProfileSpec:
        if len(self.phases) != len(set(self.phases)):
            raise ValueError(f"reachability profile {self.id!r} repeats a phase")
        if len(self.state_dimensions) != len(set(self.state_dimensions)):
            raise ValueError(f"reachability profile {self.id!r} repeats a state dimension")
        if len(self.decisions) != len(set(self.decisions)):
            raise ValueError(f"reachability profile {self.id!r} repeats a decision")
        if len(self.constraints) != len(set(self.constraints)):
            raise ValueError(f"reachability profile {self.id!r} repeats a constraint")
        if len(self.coordinates) != len(set(self.coordinates)):
            raise ValueError(f"reachability profile {self.id!r} repeats a coordinate system")
        if len(self.products) != len(set(self.products)):
            raise ValueError(f"reachability profile {self.id!r} repeats a product")
        if len(self.limiting_factors) != len(set(self.limiting_factors)):
            raise ValueError(f"reachability profile {self.id!r} repeats a limiting factor")
        if len(self.configurations) != len(set(self.configurations)):
            raise ValueError(f"reachability profile {self.id!r} repeats a configuration")
        return self
        ####
    ####


class ReachabilityFamilySpec(BaseModel):
    """One vehicle family registration row in the reachability roster."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    status: str = Field(min_length=1)
    configurations: tuple[str, ...] = Field(min_length=1)
    profiles: tuple[str, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_layout(self) -> ReachabilityFamilySpec:
        if len(self.configurations) != len(set(self.configurations)):
            raise ValueError(f"reachability family {self.id!r} repeats a configuration")
        if len(self.profiles) != len(set(self.profiles)):
            raise ValueError(f"reachability family {self.id!r} repeats a profile")
        return self
        ####
    ####


class ReachabilityCatalog(BaseModel):
    """Alpha 3 planning surface for reachability envelopes."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: int = Field(ge=1)
    id: str = Field(min_length=1)
    status: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)
    common_object: ReachabilityCommonObject
    study_semantics: dict[str, ReachabilityStudySemantic]
    coordinate_systems: dict[str, tuple[str, ...]]
    limiting_factor_codes: tuple[str, ...] = Field(min_length=1)
    profiles: tuple[ReachabilityProfileSpec, ...] = Field(min_length=1)
    vehicle_families: tuple[ReachabilityFamilySpec, ...] = Field(min_length=1)
    promotion_rungs: tuple[str, ...] = Field(min_length=1)
    qualification_requirements: tuple[str, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_cross_links(self) -> ReachabilityCatalog:
        profile_ids = [profile.id for profile in self.profiles]
        if len(profile_ids) != len(set(profile_ids)):
            raise ValueError("reachability profile IDs must be unique")
        if len(self.limiting_factor_codes) != len(set(self.limiting_factor_codes)):
            raise ValueError("reachability limiting factor codes must be unique")
        if len(self.promotion_rungs) != len(set(self.promotion_rungs)):
            raise ValueError("reachability promotion rungs must be unique")
        family_ids = [family.id for family in self.vehicle_families]
        if len(family_ids) != len(set(family_ids)):
            raise ValueError("reachability vehicle family IDs must be unique")
        profile_set = set(profile_ids)
        for family in self.vehicle_families:
            missing = [profile_id for profile_id in family.profiles if profile_id not in profile_set]
            if missing:
                raise ValueError(f"reachability family {family.id!r} references unknown profiles: {missing}")
        required_semantics = {
            "first_pass",
            "bounded_time",
            "bounded_resource",
            "bounded_path",
            "eventual_capture",
            "robust_capture",
        }
        if not set(self.study_semantics) >= required_semantics:
            raise ValueError("reachability catalog is missing required study semantics")
        return self
        ####

    def profile(self, identifier: str) -> ReachabilityProfileSpec:
        """Return one reachability profile by stable identifier."""

        for profile in self.profiles:
            if profile.id == identifier:
                return profile
        raise KeyError(f"unknown reachability profile {identifier!r}")
        ####

    def family(self, identifier: str) -> ReachabilityFamilySpec:
        """Return one reachability family roster entry by stable identifier."""

        for family in self.vehicle_families:
            if family.id == identifier:
                return family
        raise KeyError(f"unknown reachability family {identifier!r}")
        ####

    def semantic(self, identifier: str) -> ReachabilityStudySemantic:
        """Return one declared study semantic by name."""

        try:
            return self.study_semantics[identifier]
        except KeyError as error:
            raise KeyError(f"unknown reachability semantic {identifier!r}") from error
        ####


def load_reachability_catalog(path: str | Path) -> ReachabilityCatalog:
    """Load and validate the Alpha 3 reachability roster from YAML."""

    source = Path(path)
    try:
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    except OSError as error:
        raise ValueError(f"reachability catalog cannot be read: {source}: {error}") from error
    if not isinstance(payload, Mapping):
        raise ValueError(f"reachability catalog must be a mapping: {source}")
    try:
        return ReachabilityCatalog.model_validate(payload)
    except ValueError as error:
        raise ValueError(f"invalid reachability catalog {source}: {error}") from error
    ####


__all__ = [
    "ReachabilityCatalog",
    "ReachabilityCommonObject",
    "ReachabilityFamilySpec",
    "ReachabilityProfileSpec",
    "ReachabilityStudySemantic",
    "load_reachability_catalog",
]
####
