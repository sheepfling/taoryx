"""YAML loading for Alpha 2 vehicle-family packages."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from .contracts import FamilyCatalog


def load_family_catalog(path: str | Path) -> FamilyCatalog:
    """Load and validate a versioned family catalog from YAML."""

    source = Path(path)
    try:
        payload: Any = yaml.safe_load(source.read_text(encoding="utf-8"))
    except OSError as error:
        raise ValueError(f"family catalog cannot be read: {source}: {error}") from error
    if not isinstance(payload, dict):
        raise ValueError(f"family catalog must be a mapping: {source}")
    try:
        return FamilyCatalog.model_validate(payload)
    except ValueError as error:
        raise ValueError(f"invalid family catalog {source}: {error}") from error
    ####


__all__ = ["load_family_catalog"]
####
