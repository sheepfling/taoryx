"""Provider-neutral persisted table resource schemas for Taoryx."""

from __future__ import annotations

import math
from enum import StrEnum
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, JsonValue, model_validator

TABLE_SCHEMA_ID = "taoryx.table.v1"
TABLE_BUNDLE_SCHEMA_ID = "taoryx.table_bundle.v1"


class TableModel(BaseModel):
    """Frozen strict base model for canonical table resources."""

    model_config = ConfigDict(extra="forbid", frozen=True, populate_by_name=True)


####


class TableBoundaryMode(StrEnum):
    """Behavior outside one numeric side of a table axis."""

    LINEAR = "linear"
    CLAMP = "clamp"
    ERROR = "error"


####


class TableInterpolationMode(StrEnum):
    """Interpolation algorithms supported by the v1 resource."""

    MULTILINEAR = "multilinear"


####


class TableAxisDirection(StrEnum):
    """Ordering found in the provider source before normalization."""

    ASCENDING = "ascending"
    DESCENDING = "descending"


####


class TableUnitEvidence(StrEnum):
    """How a unit annotation was obtained."""

    DECLARED = "declared"
    SOURCE_COMMENT = "source_comment"
    NAME_INFERRED = "name_inferred"
    UNKNOWN = "unknown"


####


class TableUnit(TableModel):
    """Unit annotation with explicit evidence quality."""

    symbol: str | None = None
    evidence: TableUnitEvidence = TableUnitEvidence.UNKNOWN

    @model_validator(mode="after")
    def validate_evidence(self) -> "TableUnit":
        if self.symbol is None and self.evidence is not TableUnitEvidence.UNKNOWN:
            raise ValueError("unit evidence must be unknown when no unit symbol is present")
        ####
        if self.symbol is not None and not self.symbol.strip():
            raise ValueError("unit symbol cannot be blank")
        ####
        return self

    ####


####


class TableAxis(TableModel):
    """One normalized independent axis in ascending canonical order."""

    name: str = Field(min_length=1)
    source_name: str = Field(min_length=1)
    values: tuple[float, ...] = Field(min_length=1)
    unit: TableUnit = Field(default_factory=TableUnit)
    source_direction: TableAxisDirection

    @model_validator(mode="after")
    def validate_values(self) -> "TableAxis":
        if any(not math.isfinite(value) for value in self.values):
            raise ValueError("axis values must be finite")
        ####
        if len(self.values) > 1 and not all(left < right for left, right in zip(self.values, self.values[1:], strict=False)):
            raise ValueError("canonical axis values must be strictly ascending")
        ####
        return self

    ####


####


class TableExtrapolation(TableModel):
    """Per-side behavior for all independent axes."""

    lower: TableBoundaryMode
    upper: TableBoundaryMode


####


class TableProvenance(TableModel):
    """Identity of the provider source from which one resource was converted."""

    provider: str = Field(min_length=1)
    source_path: str = Field(min_length=1)
    source_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    source_table_name: str = Field(min_length=1)
    source_dimension: int = Field(ge=1)
    source_axis_order: tuple[str, ...] = Field(min_length=1)
    source_line: int | None = Field(default=None, ge=1)
    converter: str = Field(min_length=1)
    converter_version: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_axis_order(self) -> "TableProvenance":
        if len(self.source_axis_order) != self.source_dimension:
            raise ValueError("source_axis_order must match source_dimension")
        ####
        return self

    ####


####


class TableResource(TableModel):
    """Canonical provider-neutral Taoryx rectilinear table resource."""

    schema_id: Literal["taoryx.table.v1"] = Field(default=TABLE_SCHEMA_ID, alias="schema")
    table_id: str = Field(min_length=1)
    name: str = Field(min_length=1)
    dimension: int = Field(ge=1)
    axes: tuple[TableAxis, ...] = Field(min_length=1)
    values: tuple[float, ...] = Field(min_length=1)
    value_unit: TableUnit = Field(default_factory=TableUnit)
    interpolation: TableInterpolationMode = TableInterpolationMode.MULTILINEAR
    extrapolation: TableExtrapolation
    description: str | None = None
    provenance: TableProvenance

    @model_validator(mode="after")
    def validate_shape(self) -> "TableResource":
        if len(self.axes) != self.dimension:
            raise ValueError("axis count must match dimension")
        ####
        expected = math.prod(len(axis.values) for axis in self.axes)
        if len(self.values) != expected:
            raise ValueError(f"table requires {expected} values, received {len(self.values)}")
        ####
        if any(not math.isfinite(value) for value in self.values):
            raise ValueError("table values must be finite")
        ####
        if self.provenance.source_dimension != self.dimension:
            raise ValueError("provenance source dimension must match resource dimension")
        ####
        return self

    ####

    def lookup(self, *query: float) -> float:
        """Evaluate this table using its persisted numerical policy."""

        from .runtime import lookup_table

        return lookup_table(self, query)

    ####

    def interpolate(self, query: tuple[float, ...] | list[float]) -> float:
        """Compatibility surface for model kernels that consume rectilinear tables."""

        from .runtime import lookup_table

        return lookup_table(self, query)

    ####


####


class TableBundleEntry(TableModel):
    """One table file referenced by a canonical bundle manifest."""

    table_id: str = Field(min_length=1)
    name: str = Field(min_length=1)
    resource: str = Field(min_length=1)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")


####


class TableBundleManifest(TableModel):
    """Manifest for a deterministic directory of canonical table resources."""

    schema_id: Literal["taoryx.table_bundle.v1"] = Field(default=TABLE_BUNDLE_SCHEMA_ID, alias="schema")
    bundle_id: str = Field(min_length=1)
    tables: tuple[TableBundleEntry, ...] = Field(min_length=1)
    metadata: dict[str, JsonValue] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_entries(self) -> "TableBundleManifest":
        ids = [entry.table_id.casefold() for entry in self.tables]
        resources = [entry.resource.casefold() for entry in self.tables]
        if len(ids) != len(set(ids)):
            raise ValueError("bundle contains duplicate table ids")
        ####
        if len(resources) != len(set(resources)):
            raise ValueError("bundle contains duplicate table resources")
        ####
        return self

    ####


####


__all__ = [
    "TABLE_BUNDLE_SCHEMA_ID",
    "TABLE_SCHEMA_ID",
    "TableAxis",
    "TableAxisDirection",
    "TableBoundaryMode",
    "TableBundleEntry",
    "TableBundleManifest",
    "TableExtrapolation",
    "TableInterpolationMode",
    "TableProvenance",
    "TableResource",
    "TableUnit",
    "TableUnitEvidence",
]
