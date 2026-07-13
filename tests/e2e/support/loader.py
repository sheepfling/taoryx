from __future__ import annotations

import json
from pathlib import Path

import yaml

from .models import CaseSpec, MetamorphicSpec


def repository_root() -> Path:
    """Return the checked-in v23 corpus root."""
    return Path(__file__).resolve().parents[3] / "tests" / "fixtures" / "taos_e2e_v23"
####


def load_manifest(root: Path | None = None) -> list[CaseSpec]:
    base = root or repository_root()
    payload = json.loads((base / "manifest.json").read_text(encoding="utf-8"))
    return [CaseSpec.model_validate(item) for item in payload["cases"]]
####


def load_metamorphic(root: Path | None = None) -> list[MetamorphicSpec]:
    base = root or repository_root()
    payload = yaml.safe_load((base / "metamorphic/groups.yaml").read_text(encoding="utf-8"))
    return [MetamorphicSpec.model_validate(item) for item in payload["groups"]]
####


def case_directory(case: CaseSpec, root: Path | None = None) -> Path:
    return (root or repository_root()) / case.path
####
