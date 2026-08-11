"""Canonical persisted rectilinear table resources for Taoryx."""

from .io import (
    LoadedTableBundle,
    TableResourceError,
    canonical_json_bytes,
    read_table_bundle,
    read_table_resource,
    sha256_bytes,
    write_table_bundle,
    write_table_resource,
)
from .runtime import lookup_table
from .schema import (
    TABLE_BUNDLE_SCHEMA_ID,
    TABLE_SCHEMA_ID,
    TableAxis,
    TableAxisDirection,
    TableBoundaryMode,
    TableBundleEntry,
    TableBundleManifest,
    TableExtrapolation,
    TableInterpolationMode,
    TableProvenance,
    TableResource,
    TableUnit,
    TableUnitEvidence,
)

__all__ = [
    "LoadedTableBundle",
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
    "TableResourceError",
    "TableUnit",
    "TableUnitEvidence",
    "canonical_json_bytes",
    "lookup_table",
    "read_table_bundle",
    "read_table_resource",
    "sha256_bytes",
    "write_table_bundle",
    "write_table_resource",
]
