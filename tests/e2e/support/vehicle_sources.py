"""Canonical vehicle/table bindings for source-backed integration tests."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[3]
REGISTRY_PATH = ROOT / "verification/vehicle_models.yaml"
CATALOG_PATH = ROOT / "verification/vehicle_catalog.yaml"


def _registry() -> dict[str, dict[str, Any]]:
    payload = yaml.safe_load(REGISTRY_PATH.read_text(encoding="utf-8"))
    return {str(key): value for key, value in payload["vehicles"].items()}
####


def vehicle_definition(vehicle_id: str) -> dict[str, Any]:
    """Return one canonical vehicle definition for a test."""

    try:
        return _registry()[vehicle_id]
    except KeyError as error:
        raise KeyError(f"unknown test vehicle: {vehicle_id}") from error
####


def vehicle_tables(vehicle_id: str) -> tuple[Path, ...]:
    """Return every source table bound to a canonical vehicle."""

    return tuple(ROOT / path for path in vehicle_definition(vehicle_id)["table_bindings"])
####


def source_table(vehicle_id: str, filename: str) -> Path:
    """Return one table after checking it is a declared source binding."""

    tables = vehicle_tables(vehicle_id)
    matches = tuple(path for path in tables if path.name == filename)
    if len(matches) != 1:
        raise ValueError(f"{filename!r} is not a unique binding for {vehicle_id!r}")
    return matches[0]
####


def canonical_problem(vehicle_id: str) -> Path:
    """Return the generated canonical problem from the public catalog."""

    payload = yaml.safe_load(CATALOG_PATH.read_text(encoding="utf-8"))
    matches = tuple(entry for entry in payload["vehicles"] if entry["model_id"] == vehicle_id)
    if len(matches) != 1:
        raise ValueError(f"{vehicle_id!r} is not a unique public catalog entry")
    return ROOT / matches[0]["problem"]
####
