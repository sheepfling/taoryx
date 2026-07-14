"""Typed manifest models for source datasets."""

from __future__ import annotations

from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field, field_validator


class DatasetTableSpec(BaseModel):
    """One dependent variable emitted from a long-form source file."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    id: str = Field(min_length=1)
    type: str = Field(min_length=1)
    source: str = Field(min_length=1)
    axes: tuple[str, ...] = Field(min_length=1, max_length=5)
    value_column: str = Field(min_length=1)
    units: str | None = None
    extrapolation: str = "no-extrap"

    @field_validator("axes")
    @classmethod
    def unique_axes(cls, value: tuple[str, ...]) -> tuple[str, ...]:
        if len(set(value)) != len(value):
            raise ValueError("table axes must be unique")
        return value
    ####
####


class DatasetConventions(BaseModel):
    """Coordinate and interpolation conventions recorded with a dataset."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    body_axes: str
    angle_unit: str
    mdot_sign: str
    coefficients: str
    interpolation: str = "multilinear"
####


class DatasetProvenance(BaseModel):
    """Human-readable provenance classification for generated artifacts."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    generated_by: str
    source_kind: str
    flight_qualified: bool = False
    notes: tuple[str, ...] = ()
####


class DatasetManifest(BaseModel):
    """Manifest describing a reproducible source-data pack."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    schema_version: int = Field(ge=1)
    dataset_id: str = Field(min_length=1)
    title: str = Field(min_length=1)
    classification: str = Field(min_length=1)
    target_format: str = "taoryx_tbl"
    target_unit_system: str = "taoryx_legacy"
    conventions: DatasetConventions
    tables: tuple[DatasetTableSpec, ...] = Field(min_length=1)
    provenance: DatasetProvenance

    @field_validator("tables")
    @classmethod
    def unique_table_ids(cls, value: tuple[DatasetTableSpec, ...]) -> tuple[DatasetTableSpec, ...]:
        ids = [item.id.casefold() for item in value]
        if len(set(ids)) != len(ids):
            raise ValueError("dataset table identifiers must be unique")
        return value
    ####

    def resolve_source(self, manifest_path: Path, source: str) -> Path:
        """Resolve a manifest-relative source path."""

        return (manifest_path.parent / source).resolve()
    ####
####
