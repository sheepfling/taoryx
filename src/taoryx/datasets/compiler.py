"""Compile long-form source data into deterministic TAORYX tables."""

from __future__ import annotations

import csv
import hashlib
import json
from collections.abc import Mapping
from itertools import product
from pathlib import Path
from typing import Any

import yaml

from .models import DatasetManifest, DatasetTableSpec


def load_manifest(path: str | Path) -> DatasetManifest:
    """Load and validate a YAML dataset manifest."""

    manifest_path = Path(path)
    payload = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"dataset manifest must contain a mapping: {manifest_path}")
    return DatasetManifest.model_validate(payload)
####


def flatten_rectangular_grid(
    axis_names: tuple[str, ...],
    axes: Mapping[str, tuple[float, ...]],
    samples: Mapping[tuple[float, ...], float],
) -> tuple[float, ...]:
    """Flatten a grid with the final declared axis varying fastest."""

    flattened: list[float] = []
    for coordinate in product(*(axes[name] for name in axis_names)):
        normalized = tuple(float(value) for value in coordinate)
        if normalized not in samples:
            raise ValueError(f"missing table value at {normalized!r}")
        flattened.append(float(samples[normalized]))
    return tuple(flattened)
####


def _read_source(path: Path, spec: DatasetTableSpec) -> tuple[dict[str, tuple[float, ...]], dict[tuple[float, ...], float]]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"source file is empty: {path}")
    required = (*spec.axes, spec.value_column)
    missing_columns = [name for name in required if name not in rows[0]]
    if missing_columns:
        raise ValueError(f"{path} is missing columns: {', '.join(missing_columns)}")
    axis_values: dict[str, set[float]] = {name: set() for name in spec.axes}
    samples: dict[tuple[float, ...], float] = {}
    for row_number, row in enumerate(rows, start=2):
        try:
            coordinate = tuple(float(row[name]) for name in spec.axes)
            value = float(row[spec.value_column])
        except (TypeError, ValueError) as error:
            raise ValueError(f"non-numeric source value at {path}:{row_number}") from error
        if coordinate in samples:
            raise ValueError(f"duplicate table coordinate at {path}:{row_number}: {coordinate!r}")
        for name, coordinate_value in zip(spec.axes, coordinate, strict=True):
            axis_values[name].add(coordinate_value)
        samples[coordinate] = value
    axes = {name: tuple(sorted(values)) for name, values in axis_values.items()}
    expected = 1
    for axis in axes.values():
        expected *= len(axis)
    if len(samples) != expected:
        raise ValueError(f"source grid for {spec.id!r} is incomplete: expected {expected}, found {len(samples)}")
    return axes, samples
####


def _format_values(values: tuple[float, ...]) -> str:
    return ",".join(f"{value:.12g}" for value in values)
####


def _compile_table(spec: DatasetTableSpec, source_path: Path) -> tuple[str, dict[str, Any]]:
    axes, samples = _read_source(source_path, spec)
    values = flatten_rectangular_grid(spec.axes, axes, samples)
    options = [spec.extrapolation]
    if spec.units:
        options.append(f"units={spec.units}")
    lines = [f"({spec.id})", f"table {spec.type}({','.join(spec.axes)}) {' '.join(options)}"]
    lines.extend(f"{name}={_format_values(axes[name])}" for name in spec.axes)
    lines.append(f"{spec.type}={_format_values(values)}")
    lines.append("")
    return "\n".join(lines), {
        "id": spec.id,
        "type": spec.type,
        "source": source_path.name,
        "axes": list(spec.axes),
        "value_column": spec.value_column,
        "units": spec.units,
        "sample_count": len(values),
    }
####


def compile_dataset(manifest_path: str | Path, output_dir: str | Path) -> dict[str, Path]:
    """Compile a manifest and write `.tbl` files plus a provenance sidecar."""

    manifest_file = Path(manifest_path).resolve()
    manifest = load_manifest(manifest_file)
    output_root = Path(output_dir)
    output_root.mkdir(parents=True, exist_ok=True)
    generated: dict[str, Path] = {}
    table_records: list[dict[str, Any]] = []
    source_hashes: dict[str, str] = {}
    for spec in manifest.tables:
        source_path = manifest.resolve_source(manifest_file, spec.source)
        source_hashes[spec.source] = hashlib.sha256(source_path.read_bytes()).hexdigest()
        text, record = _compile_table(spec, source_path)
        output_path = output_root / f"{spec.id}.tbl"
        output_path.write_text(text, encoding="utf-8")
        generated[spec.id] = output_path
        table_records.append(record)
    provenance = {
        "schema_version": 1,
        "dataset_id": manifest.dataset_id,
        "title": manifest.title,
        "classification": manifest.classification,
        "target": {"format": manifest.target_format, "unit_system": manifest.target_unit_system},
        "conventions": manifest.conventions.model_dump(mode="json"),
        "source_hashes": source_hashes,
        "tables": table_records,
        "generator": manifest.provenance.model_dump(mode="json"),
    }
    provenance_path = output_root / "provenance.json"
    provenance_path.write_text(json.dumps(provenance, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    generated["provenance"] = provenance_path
    return generated
####
