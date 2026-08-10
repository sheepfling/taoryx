"""Source-hash-pinned compatibility overlays for legacy DAVE-ML documents."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True, slots=True)
class DAVEMLCompatibilityOverlay:
    """Validated semantic repairs that leave the source bytes unchanged."""

    overlay_version: str
    source_sha256: str
    repairs: tuple[dict[str, Any], ...]

    def permits_legacy_ungridded_reference(self, table_id: str) -> bool:
        """Return whether this overlay repairs the named legacy table reference."""

        for repair in self.repairs:
            observed = repair.get("observed", {})
            replacement = repair.get("replacement", {})
            if (
                observed.get("element") == "griddedTableRef"
                and observed.get("attribute") == "gtID"
                and str(observed.get("value", "")).strip() == table_id
                and replacement.get("element") == "ungriddedTableRef"
                and replacement.get("attribute") == "utID"
                and str(replacement.get("value", "")).strip() == table_id
                and repair.get("source_mutated") is False
            ):
                return True
        return False
        ####


def load_compatibility_overlay(path: str | Path, payload: bytes) -> DAVEMLCompatibilityOverlay:
    """Load and validate an overlay against the immutable source payload."""

    source_sha256 = hashlib.sha256(payload).hexdigest()
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    declared = str(raw.get("source_sha256", "")).lower()
    if declared != source_sha256:
        raise ValueError(f"compatibility overlay source hash mismatch: expected {source_sha256}, got {declared}")
    if raw.get("overlay_version") != "1.0":
        raise ValueError("unsupported DAVE-ML compatibility overlay version")
    repairs = raw.get("repairs")
    if not isinstance(repairs, list) or not repairs or not all(isinstance(repair, dict) for repair in repairs):
        raise ValueError("compatibility overlay must declare at least one repair")
    return DAVEMLCompatibilityOverlay("1.0", source_sha256, tuple(repairs))
    ####


def load_compatibility_overlay_for_payload(payload: bytes, directory: str | Path) -> DAVEMLCompatibilityOverlay | None:
    """Load the overlay named by a source payload hash, if one exists."""

    source_sha256 = hashlib.sha256(payload).hexdigest()
    path = Path(directory) / f"{source_sha256}.json"
    if not path.is_file():
        return None
    return load_compatibility_overlay(path, payload)
    ####


def load_quarantine_for_payload(payload: bytes, directory: str | Path, policy: str) -> dict[str, Any] | None:
    """Load a case-level quarantine pinned to the source hash and policy."""

    source_sha256 = hashlib.sha256(payload).hexdigest()
    path = Path(directory) / f"{source_sha256}.json"
    if not path.is_file():
        return None
    raw = json.loads(path.read_text(encoding="utf-8"))
    if str(raw.get("source_sha256", "")).lower() != source_sha256:
        raise ValueError("DAVE-ML quarantine source hash mismatch")
    if raw.get("policy") != policy:
        raise ValueError("DAVE-ML quarantine policy does not match the selected interpolation policy")
    if raw.get("expires_on_source_change") is not True or raw.get("expires_on_policy_change") is not True:
        raise ValueError("DAVE-ML quarantine must expire on source or policy change")
    return raw
    ####
