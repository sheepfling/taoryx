"""Deterministic disk I/O for canonical Taoryx table resources."""

from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path

from pydantic import JsonValue, ValidationError

from .schema import TableBundleEntry, TableBundleManifest, TableResource


class TableResourceError(ValueError):
    """Malformed or integrity-invalid canonical table resource."""


####


class LoadedTableBundle:
    """Loaded manifest plus immutable table resources."""

    def __init__(self, manifest: TableBundleManifest, tables: tuple[TableResource, ...]) -> None:
        self.manifest = manifest
        self.tables = tables

    ####

    def table(self, table_id: str) -> TableResource:
        """Resolve one table by canonical id."""

        key = table_id.casefold()
        for table in self.tables:
            if table.table_id.casefold() == key:
                return table
            ####
        ####
        raise KeyError(table_id)

    ####

    def tables_named(self, name: str) -> tuple[TableResource, ...]:
        """Return all tables sharing one provider/source display name."""

        key = name.casefold()
        return tuple(table for table in self.tables if table.name.casefold() == key)

    ####


####


def canonical_json_bytes(model: TableResource | TableBundleManifest) -> bytes:
    """Serialize one canonical model deterministically as UTF-8 JSON."""

    payload = model.model_dump(mode="json", by_alias=True)
    text = json.dumps(payload, ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":"))
    return (text + "\n").encode("utf-8")


####


def sha256_bytes(payload: bytes) -> str:
    """Return a lowercase SHA-256 digest."""

    return hashlib.sha256(payload).hexdigest()


####


def write_table_resource(table: TableResource, path: str | Path) -> str:
    """Write one deterministic table resource and return its SHA-256."""

    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = canonical_json_bytes(table)
    target.write_bytes(payload)
    return sha256_bytes(payload)


####


def read_table_resource(path: str | Path, *, expected_sha256: str | None = None) -> TableResource:
    """Load and validate one canonical table resource."""

    source = Path(path)
    payload = source.read_bytes()
    digest = sha256_bytes(payload)
    if expected_sha256 is not None and digest != expected_sha256:
        raise TableResourceError(f"table resource checksum mismatch for {source}")
    ####
    try:
        raw = json.loads(payload)
        return TableResource.model_validate(raw)
    except (json.JSONDecodeError, ValidationError) as error:
        raise TableResourceError(f"invalid canonical table resource {source}: {error}") from error
    ####


####


def write_table_bundle(
    output_dir: str | Path,
    *,
    bundle_id: str,
    tables: tuple[TableResource, ...],
    metadata: dict[str, JsonValue] | None = None,
) -> TableBundleManifest:
    """Write a deterministic bundle directory and manifest."""

    root = Path(output_dir)
    tables_dir = root / "tables"
    if tables_dir.exists():
        shutil.rmtree(tables_dir)
    ####
    tables_dir.mkdir(parents=True, exist_ok=True)
    entries: list[TableBundleEntry] = []
    used_names: set[str] = set()
    for table in tables:
        file_name = _resource_file_name(table.table_id, used_names)
        resource_path = Path("tables") / file_name
        digest = write_table_resource(table, root / resource_path)
        entries.append(
            TableBundleEntry(
                table_id=table.table_id,
                name=table.name,
                resource=resource_path.as_posix(),
                sha256=digest,
            )
        )
    ####
    manifest = TableBundleManifest(
        bundle_id=bundle_id,
        tables=tuple(entries),
        metadata=dict(metadata or {}),
    )
    (root / "manifest.json").write_bytes(canonical_json_bytes(manifest))
    return manifest


####


def read_table_bundle(path: str | Path) -> LoadedTableBundle:
    """Load a bundle manifest and verify all referenced table hashes."""

    root = Path(path)
    manifest_path = root / "manifest.json" if root.is_dir() else root
    try:
        manifest = TableBundleManifest.model_validate(json.loads(manifest_path.read_bytes()))
    except (json.JSONDecodeError, ValidationError) as error:
        raise TableResourceError(f"invalid table bundle manifest {manifest_path}: {error}") from error
    ####
    base = manifest_path.parent
    tables: list[TableResource] = []
    for entry in manifest.tables:
        resource_path = (base / entry.resource).resolve()
        try:
            resource_path.relative_to(base.resolve())
        except ValueError as error:
            raise TableResourceError(f"table resource escapes bundle directory: {entry.resource}") from error
        ####
        table = read_table_resource(resource_path, expected_sha256=entry.sha256)
        if table.table_id != entry.table_id or table.name != entry.name:
            raise TableResourceError(f"manifest identity does not match resource {entry.resource}")
        ####
        tables.append(table)
    ####
    return LoadedTableBundle(manifest=manifest, tables=tuple(tables))


####


def _resource_file_name(table_id: str, used_names: set[str]) -> str:
    slug = "".join(character.lower() if character.isalnum() else "-" for character in table_id).strip("-")
    slug = "-".join(part for part in slug.split("-") if part)
    if not slug:
        slug = "table"
    ####
    candidate = f"{slug}.table.json"
    if candidate.casefold() in used_names:
        suffix = hashlib.sha256(table_id.encode("utf-8")).hexdigest()[:8]
        candidate = f"{slug}-{suffix}.table.json"
    ####
    counter = 2
    while candidate.casefold() in used_names:
        candidate = f"{slug}-{counter}.table.json"
        counter += 1
    ####
    used_names.add(candidate.casefold())
    return candidate


####


__all__ = [
    "LoadedTableBundle",
    "TableResourceError",
    "canonical_json_bytes",
    "read_table_bundle",
    "read_table_resource",
    "sha256_bytes",
    "write_table_bundle",
    "write_table_resource",
]
