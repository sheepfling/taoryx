"""Conversion from CADAC source decks into canonical Taoryx table resources."""

from __future__ import annotations

import hashlib
import re
from pathlib import Path

import numpy as np
from pydantic import JsonValue

from .bundle import CadacDeckBinding, load_cadac_source_bundle
from .deck import CADAC_TABLE_POLICY, CadacBoundaryMode, CadacDeck, CadacDeckTable, parse_cadac_deck_file
from .input_ast import source_name_for
from .table_resources import (
    LoadedTableBundle,
    TableAxis,
    TableAxisDirection,
    TableBoundaryMode,
    TableBundleManifest,
    TableExtrapolation,
    TableInterpolationMode,
    TableProvenance,
    TableResource,
    TableUnit,
    TableUnitEvidence,
    read_table_bundle,
    write_table_bundle,
)

CADAC_TABLE_CONVERTER_VERSION = "0.13.0"

_AXIS_UNITS: dict[str, str] = {
    "alpha": "deg",
    "alphax": "deg",
    "alpp": "deg",
    "alppx": "deg",
    "alt": "m",
    "altitude": "m",
    "beta": "deg",
    "betax": "deg",
    "mach": "1",
    "mass": "kg",
    "q": "Pa",
    "pdynmc": "Pa",
    "throttle": "1",
    "thrtl": "1",
    "thrust": "N",
    "time": "s",
}
_VALUE_UNIT_ALIASES: dict[str, str] = {
    "deg": "deg",
    "kg": "kg",
    "kg*m^2": "kg*m^2",
    "kgm^2": "kg*m^2",
    "m": "m",
    "m/s": "m/s",
    "m/s^2": "m/s^2",
    "n": "N",
    "n*m": "N*m",
    "nm": "N*m",
    "pa": "Pa",
    "rad": "rad",
    "s": "s",
    "sec": "s",
    "nd": "1",
    "1/deg": "1/deg",
    "1/rad": "1/rad",
}


class CadacCanonicalDeck:
    """Canonical in-memory replacement for one CADAC deck lookup surface."""

    def __init__(self, source_name: str, tables: tuple[TableResource, ...]) -> None:
        self.source_name = source_name
        self.tables = tables

    ####

    def table(self, name: str) -> TableResource:
        """Resolve one converted table by its original CADAC name."""

        key = name.casefold()
        for table in self.tables:
            if table.name.casefold() == key:
                return table
            ####
        ####
        raise KeyError(name)

    ####


####


def convert_cadac_deck(
    deck: CadacDeck,
    *,
    source_path: str,
    source_sha256: str,
) -> CadacCanonicalDeck:
    """Convert a parsed CADAC deck into canonical in-memory table resources."""

    tables = tuple(
        convert_cadac_table(
            table,
            source_path=source_path,
            source_sha256=source_sha256,
        )
        for table in deck.tables
    )
    return CadacCanonicalDeck(source_name=deck.source_name, tables=tables)


####


def convert_cadac_table(
    table: CadacDeckTable,
    *,
    source_path: str,
    source_sha256: str,
) -> TableResource:
    """Normalize one CADAC table into the provider-neutral v1 table schema."""

    axis_names = _axis_names(table)
    normalized_axes: list[TableAxis] = []
    flip_axes: list[int] = []
    for axis_index, (axis_name, source_axis) in enumerate(zip(axis_names, table.axes, strict=True)):
        descending = len(source_axis) > 1 and source_axis[0] > source_axis[-1]
        source_direction = TableAxisDirection.DESCENDING if descending else TableAxisDirection.ASCENDING
        canonical_values = tuple(reversed(source_axis)) if descending else source_axis
        if descending:
            flip_axes.append(axis_index)
        ####
        unit_symbol = _axis_unit(axis_name)
        normalized_axes.append(
            TableAxis(
                name=axis_name,
                source_name=f"X{axis_index + 1}",
                values=canonical_values,
                unit=TableUnit(
                    symbol=unit_symbol,
                    evidence=TableUnitEvidence.NAME_INFERRED if unit_symbol is not None else TableUnitEvidence.UNKNOWN,
                ),
                source_direction=source_direction,
            )
        )
    ####
    tensor = np.asarray(table.values, dtype=float).reshape(table.axis_sizes)
    for axis_index in flip_axes:
        tensor = np.flip(tensor, axis=axis_index)
    ####
    value_unit = _value_unit_from_description(table.description)
    table_id = _table_id(source_path, source_sha256, table.name)
    return TableResource(
        table_id=table_id,
        name=table.name,
        dimension=table.dimension,
        axes=tuple(normalized_axes),
        values=tuple(float(value) for value in tensor.reshape(-1)),
        value_unit=value_unit,
        interpolation=TableInterpolationMode.MULTILINEAR,
        extrapolation=TableExtrapolation(
            lower=_convert_boundary_mode(CADAC_TABLE_POLICY.lower),
            upper=_convert_boundary_mode(CADAC_TABLE_POLICY.upper),
        ),
        description=table.description,
        provenance=TableProvenance(
            provider="cadac",
            source_path=source_path,
            source_sha256=source_sha256,
            source_table_name=table.name,
            source_dimension=table.dimension,
            source_axis_order=tuple(f"X{index + 1}" for index in range(table.dimension)),
            source_line=table.source_line,
            converter="taoryx.families.cadac.table_conversion",
            converter_version=CADAC_TABLE_CONVERTER_VERSION,
        ),
    )


####


def convert_cadac_deck_file_to_table_bundle(
    deck_path: str | Path,
    output_dir: str | Path,
) -> TableBundleManifest:
    """Convert one physical CADAC deck file into a canonical Taoryx bundle."""

    source = Path(deck_path).resolve()
    payload = source.read_bytes()
    source_sha256 = hashlib.sha256(payload).hexdigest()
    deck = parse_cadac_deck_file(source)
    canonical = convert_cadac_deck(
        deck,
        source_path=source_name_for(source),
        source_sha256=source_sha256,
    )
    metadata: dict[str, JsonValue] = {
        "provider": "cadac",
        "conversion_kind": "deck",
        "source_path": source_name_for(source),
        "source_sha256": source_sha256,
        "source_title": deck.title,
        "converter_version": CADAC_TABLE_CONVERTER_VERSION,
    }
    return write_table_bundle(
        output_dir,
        bundle_id=f"cadac-deck-{source.stem}-{source_sha256[:12]}",
        tables=canonical.tables,
        metadata=metadata,
    )


####


def convert_cadac_source_bundle_to_table_bundle(
    input_path: str | Path,
    output_dir: str | Path,
) -> TableBundleManifest:
    """Convert all unique physical decks referenced by one CADAC input case."""

    bundle = load_cadac_source_bundle(input_path)
    tables: list[TableResource] = []
    converted_artifacts: set[tuple[str, str]] = set()
    for binding in bundle.deck_bindings:
        artifact_key = (binding.artifact.resolved_path, binding.artifact.sha256)
        if artifact_key in converted_artifacts:
            continue
        ####
        converted = convert_cadac_deck(
            binding.deck,
            source_path=binding.artifact.resolved_path,
            source_sha256=binding.artifact.sha256,
        )
        tables.extend(converted.tables)
        converted_artifacts.add(artifact_key)
    ####
    metadata: dict[str, JsonValue] = {
        "provider": "cadac",
        "conversion_kind": "source_bundle",
        "input_path": bundle.input_artifact.resolved_path,
        "input_sha256": bundle.input_artifact.sha256,
        "converter_version": CADAC_TABLE_CONVERTER_VERSION,
        "deck_bindings": [_binding_metadata(binding) for binding in bundle.deck_bindings],
    }
    return write_table_bundle(
        output_dir,
        bundle_id=f"cadac-case-{Path(input_path).stem}-{bundle.input_artifact.sha256[:12]}",
        tables=tuple(tables),
        metadata=metadata,
    )


####


def convert_cadac_tree_to_table_bundle(
    source_root: str | Path,
    output_dir: str | Path,
) -> TableBundleManifest:
    """Convert every table-bearing ``.asc`` deck under one CADAC source tree."""

    root = Path(source_root).resolve()
    if not root.is_dir():
        raise NotADirectoryError(root)
    ####
    candidates = tuple(path for path in sorted(root.rglob("*.asc")) if _looks_like_cadac_deck(path))
    if not candidates:
        raise FileNotFoundError(f"no CADAC table decks found under {root}")
    ####
    tables: list[TableResource] = []
    deck_metadata: list[dict[str, JsonValue]] = []
    for source in candidates:
        payload = source.read_bytes()
        source_sha256 = hashlib.sha256(payload).hexdigest()
        deck = parse_cadac_deck_file(source)
        converted = convert_cadac_deck(
            deck,
            source_path=source_name_for(source),
            source_sha256=source_sha256,
        )
        tables.extend(converted.tables)
        deck_metadata.append(
            {
                "source_path": source_name_for(source),
                "source_sha256": source_sha256,
                "table_count": len(converted.tables),
                "table_names": [table.name for table in converted.tables],
            }
        )
    ####
    root_identity = hashlib.sha256("\n".join(f"{item['source_path']}:{item['source_sha256']}" for item in deck_metadata).encode("utf-8")).hexdigest()
    metadata: dict[str, JsonValue] = {
        "provider": "cadac",
        "conversion_kind": "source_tree",
        "source_root": source_name_for(root),
        "source_tree_identity_sha256": root_identity,
        "converter_version": CADAC_TABLE_CONVERTER_VERSION,
        "deck_count": len(deck_metadata),
        "source_decks": deck_metadata,
    }
    return write_table_bundle(
        output_dir,
        bundle_id=f"cadac-tree-{root.name}-{root_identity[:12]}",
        tables=tuple(tables),
        metadata=metadata,
    )


####


def load_converted_cadac_deck(bundle_path: str | Path, *, source_path: str | None = None) -> CadacCanonicalDeck:
    """Load one converted deck view from a canonical bundle."""

    loaded = read_table_bundle(bundle_path)
    selected = tuple(table for table in loaded.tables if source_path is None or table.provenance.source_path == source_path)
    if not selected:
        raise KeyError(source_path or "<all>")
    ####
    source_names = {table.provenance.source_path for table in selected}
    if len(source_names) != 1:
        raise ValueError("bundle contains multiple source decks; select source_path explicitly")
    ####
    return CadacCanonicalDeck(source_name=next(iter(source_names)), tables=selected)


####


def validate_converted_table_bundle(path: str | Path) -> LoadedTableBundle:
    """Load a canonical bundle with full schema and SHA-256 verification."""

    return read_table_bundle(path)


####


def _looks_like_cadac_deck(path: Path) -> bool:
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    for index, raw_line in enumerate(lines):
        content = raw_line.partition("//")[0].strip()
        if re.fullmatch(r"\d+DIM\s+\S+", content, flags=re.IGNORECASE) is None:
            continue
        ####
        for following in lines[index + 1 :]:
            following_content = following.partition("//")[0].strip()
            if not following_content:
                continue
            ####
            return re.match(r"NX1\s+\d+", following_content, flags=re.IGNORECASE) is not None
        ####
    ####
    return False


####


def _axis_names(table: CadacDeckTable) -> tuple[str, ...]:
    marker = "_vs_"
    if marker not in table.name.casefold():
        return tuple(f"x{index + 1}" for index in range(table.dimension))
    ####
    suffix = table.name[table.name.casefold().index(marker) + len(marker) :]
    comma_parts = tuple(part.strip() for part in suffix.split(",") if part.strip())
    if len(comma_parts) == table.dimension:
        return tuple(_normalize_axis_name(part) for part in comma_parts)
    ####
    underscore_parts = tuple(part.strip() for part in suffix.split("_") if part.strip())
    if len(underscore_parts) == table.dimension:
        return tuple(_normalize_axis_name(part) for part in underscore_parts)
    ####
    if table.dimension == 1:
        return (_normalize_axis_name(suffix),)
    ####
    return tuple(f"x{index + 1}" for index in range(table.dimension))


####


def _normalize_axis_name(name: str) -> str:
    normalized = re.sub(r"[^a-zA-Z0-9_]+", "_", name.strip()).strip("_").lower()
    return normalized or "axis"


####


def _axis_unit(axis_name: str) -> str | None:
    key = axis_name.casefold()
    direct = _AXIS_UNITS.get(key)
    if direct is not None:
        return direct
    ####
    if key.endswith("time") or key.endswith("_time"):
        return "s"
    ####
    if key.endswith("alt") or key.endswith("altitude"):
        return "m"
    ####
    return None


####


def _value_unit_from_description(description: str | None) -> TableUnit:
    if description is None:
        return TableUnit()
    ####
    normalized = description.strip().lower().replace("²", "^2")
    candidates = re.findall(r"(?:-|\b)(kg\*m\^2|kgm\^2|m/s\^2|m/s|n\*m|1/deg|1/rad|deg|kg|pa|rad|sec|s|nm|n|nd|m)\b", normalized)
    if not candidates:
        return TableUnit()
    ####
    symbol = _VALUE_UNIT_ALIASES.get(candidates[-1])
    if symbol is None:
        return TableUnit()
    ####
    return TableUnit(symbol=symbol, evidence=TableUnitEvidence.SOURCE_COMMENT)


####


def _table_id(source_path: str, source_sha256: str, table_name: str) -> str:
    source_stem = Path(source_path).stem.lower()
    normalized_name = re.sub(r"[^a-zA-Z0-9_.-]+", "-", table_name).strip("-").lower()
    path_digest = hashlib.sha256(source_path.encode("utf-8")).hexdigest()[:8]
    return f"cadac.{source_stem}.{normalized_name}.{source_sha256[:12]}.{path_digest}"


####


def _convert_boundary_mode(mode: CadacBoundaryMode) -> TableBoundaryMode:
    if mode is CadacBoundaryMode.LINEAR:
        return TableBoundaryMode.LINEAR
    ####
    if mode is CadacBoundaryMode.CLAMP:
        return TableBoundaryMode.CLAMP
    ####
    raise ValueError(f"unsupported CADAC boundary mode {mode}")


####


def _binding_metadata(binding: CadacDeckBinding) -> JsonValue:
    return {
        "vehicle_model": binding.vehicle_model,
        "vehicle_role": binding.vehicle_role,
        "kind": binding.kind.value,
        "keyword": binding.keyword,
        "reference": binding.reference,
        "source_line": binding.source_line,
        "source_path": binding.artifact.resolved_path,
        "source_sha256": binding.artifact.sha256,
    }


####


__all__ = [
    "CADAC_TABLE_CONVERTER_VERSION",
    "CadacCanonicalDeck",
    "convert_cadac_deck",
    "convert_cadac_deck_file_to_table_bundle",
    "convert_cadac_source_bundle_to_table_bundle",
    "convert_cadac_tree_to_table_bundle",
    "convert_cadac_table",
    "load_converted_cadac_deck",
    "validate_converted_table_bundle",
]
