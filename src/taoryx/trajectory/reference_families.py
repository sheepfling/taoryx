"""Source-grounded reference-family manifests.

Reference plants need a little more metadata than an ordinary synthetic
``FamilyPackage``: source locks, named reduction profiles, and evidence for
each composable layer.  This module keeps that metadata provider-neutral and
projects the manifest into the existing Alpha 2 family contract for case
resolution.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field

from .contracts import (
    ControlSchema,
    FamilyCatalog,
    FamilyPackage,
    FidelityProfile,
    ObservationSchema,
    ParameterSchema,
)
from .daveml_import import load_daveml_family_import


class ReferenceSourceLock(BaseModel):
    """Immutable source and package identity for a reference family."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["daveml"]
    package: str = Field(min_length=1)
    package_relative_path: str = Field(min_length=1)
    package_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    aerodynamic_source: str = Field(min_length=1)
    aerodynamics_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    corpus_archive: str = Field(min_length=1)
    corpus_archive_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    availability: str = Field(min_length=1)
    license_and_notices: str = Field(min_length=1)
####


class ReferenceGeometry(BaseModel):
    """Reference dimensions copied from the accepted package manifest."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    area_m2: float = Field(gt=0.0)
    mean_aerodynamic_chord_m: float = Field(gt=0.0)
    span_m: float = Field(gt=0.0)
####


class ReferenceValidityEnvelope(BaseModel):
    """Closed validity bounds for the source plant's advertised axes."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    mach_min: float
    mach_max: float
    alpha_min_rad: float
    alpha_max_rad: float
    beta_min_rad: float
    beta_max_rad: float
    altitude_min_m: float
    altitude_max_m: float

    def validate_ordering(self) -> None:
        """Reject inverted source-envelope bounds."""

        pairs = (
            ("mach", self.mach_min, self.mach_max),
            ("alpha", self.alpha_min_rad, self.alpha_max_rad),
            ("beta", self.beta_min_rad, self.beta_max_rad),
            ("altitude", self.altitude_min_m, self.altitude_max_m),
        )
        for name, lower, upper in pairs:
            if lower > upper:
                raise ValueError(f"reference envelope has inverted {name} bounds")
        ####
    ####


class ReferencePlantBinding(BaseModel):
    """Native plant boundary without importing a provider implementation."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    fidelity: Literal["rigid_body_6dof"]
    body_frame: str = Field(min_length=1)
    navigation_frame: str = Field(min_length=1)
    quaternion_order: Literal["wxyz", "xyzw"]
    package_schema_version: str = Field(min_length=1)
    reference_geometry: ReferenceGeometry
    validity_envelope: ReferenceValidityEnvelope
    qualification: str = Field(min_length=1)
    source_payload_required_for_execution: bool = True

    def validate_envelope(self) -> None:
        """Validate the plant's package-derived envelope."""

        self.validity_envelope.validate_ordering()
        ####
####


class ReferenceFidelityProfile(BaseModel):
    """A named fidelity realization and its explicit reduction boundary."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    profile_id: str = Field(min_length=1)
    runtime_fidelity: FidelityProfile
    equations: tuple[str, ...] = ()
    status: str = Field(min_length=1)
    parent_profile: str | None = None
    data_contract: tuple[str, ...] = ()
    omitted_physics: tuple[str, ...] = ()
####


class ReferenceLayer(BaseModel):
    """Evidence and version for one overlay in the family stack."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    version: str = Field(min_length=1)
    evidence: str = Field(min_length=1)
####


class ReferenceFamilyManifest(BaseModel):
    """Standardized library shape for a source-grounded vehicle family."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: int = Field(default=1, ge=1)
    manifest_type: Literal["taoryx.reference-family/v1alpha1"]
    family_id: str = Field(min_length=1)
    version: str = Field(min_length=1)
    display_name: str = Field(min_length=1)
    role: Literal["source-grounded-reference-anchor"]
    source: ReferenceSourceLock
    daveml_import: str | None = None
    plant: ReferencePlantBinding
    fidelity_profiles: tuple[ReferenceFidelityProfile, ...]
    parameters: tuple[ParameterSchema, ...] = ()
    variants: dict[str, dict[str, Any]] = Field(default_factory=dict)
    loadouts: dict[str, dict[str, Any]] = Field(default_factory=dict)
    missions: dict[str, dict[str, Any]] = Field(default_factory=dict)
    segment_plans: dict[str, dict[str, Any]] = Field(default_factory=dict)
    segment_graphs: dict[str, dict[str, Any]] = Field(default_factory=dict)
    controls: tuple[ControlSchema, ...] = ()
    observations: tuple[ObservationSchema, ...] = ()
    layers: dict[str, ReferenceLayer]
    bindings: tuple[str, ...] = ()
    claims: tuple[str, ...] = ()
    nonclaims: tuple[str, ...] = ()
    next_gate: str = Field(min_length=1)

    def fidelity_map(self) -> dict[str, ReferenceFidelityProfile]:
        """Return named fidelity profiles and reject duplicate IDs."""

        result = {profile.profile_id: profile for profile in self.fidelity_profiles}
        if len(result) != len(self.fidelity_profiles):
            raise ValueError(f"family {self.family_id!r} has duplicate fidelity profile IDs")
        return result
        ####

    def to_family_package(self) -> FamilyPackage:
        """Project the reference manifest into the neutral case contract."""

        self.fidelity_map()
        runtime_fidelities = tuple(dict.fromkeys(profile.runtime_fidelity for profile in self.fidelity_profiles))
        if self.plant.fidelity not in runtime_fidelities:
            raise ValueError(f"family {self.family_id!r} does not advertise its source plant fidelity")
        return FamilyPackage(
            family_id=self.family_id,
            version=self.version,
            display_name=self.display_name,
            fidelities=runtime_fidelities,
            parameters=self.parameters,
            variants=self.variants,
            loadouts=self.loadouts,
            missions=self.missions,
            segment_plans=self.segment_plans,
            segment_graphs=self.segment_graphs,
            controls=self.controls,
            observations=self.observations,
            provenance=(
                f"source-grounded DAVE-ML reference family; package={self.source.package}; "
                f"package_sha256={self.source.package_sha256}; next_gate={self.next_gate}"
            ),
        )
        ####
####


def load_reference_family_manifest(path: str | Path) -> ReferenceFamilyManifest:
    """Load one source-grounded family manifest from YAML."""

    source = Path(path)
    try:
        payload: Any = yaml.safe_load(source.read_text(encoding="utf-8"))
    except OSError as error:
        raise ValueError(f"reference family manifest cannot be read: {source}: {error}") from error
    if not isinstance(payload, dict):
        raise ValueError(f"reference family manifest must be a mapping: {source}")
    try:
        manifest = ReferenceFamilyManifest.model_validate(payload)
        manifest.fidelity_map()
        manifest.plant.validate_envelope()
        if manifest.daveml_import is not None:
            import_record = load_daveml_family_import(source.parent / manifest.daveml_import)
            if import_record.family_id != manifest.family_id:
                raise ValueError(
                    f"DAVE-ML import family mismatch: record={import_record.family_id!r}, "
                    f"manifest={manifest.family_id!r}"
                )
            if import_record.package.sha256 != manifest.source.package_sha256:
                raise ValueError(
                    f"DAVE-ML import package hash mismatch: record={import_record.package.sha256!r}, "
                    f"manifest={manifest.source.package_sha256!r}"
                )
        return manifest
    except ValueError as error:
        raise ValueError(f"invalid reference family manifest {source}: {error}") from error
    ####


def load_reference_family_catalog(path: str | Path) -> tuple[ReferenceFamilyManifest, ...]:
    """Load a catalog of repository-relative reference-family manifests."""

    source = Path(path)
    try:
        payload: Any = yaml.safe_load(source.read_text(encoding="utf-8"))
    except OSError as error:
        raise ValueError(f"reference family catalog cannot be read: {source}: {error}") from error
    if not isinstance(payload, dict) or payload.get("manifest_type") != "taoryx.reference-family-catalog/v1alpha1":
        raise ValueError(f"invalid reference family catalog header: {source}")
    entries = payload.get("families")
    if not isinstance(entries, list) or not all(isinstance(entry, str) for entry in entries):
        raise ValueError(f"reference family catalog families must be a list of paths: {source}")
    manifests = tuple(load_reference_family_manifest(source.parent / entry) for entry in entries)
    ids = [manifest.family_id for manifest in manifests]
    if len(ids) != len(set(ids)):
        raise ValueError(f"reference family catalog contains duplicate family IDs: {source}")
    return manifests
    ####


def as_family_catalog(manifests: tuple[ReferenceFamilyManifest, ...]) -> FamilyCatalog:
    """Convert loaded reference manifests into the common family catalog."""

    return FamilyCatalog(families=tuple(manifest.to_family_package() for manifest in manifests))
    ####


__all__ = [
    "ReferenceFamilyManifest",
    "ReferenceFidelityProfile",
    "ReferenceLayer",
    "ReferenceGeometry",
    "ReferencePlantBinding",
    "ReferenceSourceLock",
    "ReferenceValidityEnvelope",
    "as_family_catalog",
    "load_reference_family_catalog",
    "load_reference_family_manifest",
]
