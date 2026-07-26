"""Validate the shared operational contract registry for DaveML families."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
CONTRACTS = ROOT / "verification/daveml_operational_contracts.yaml"
REPORT = ROOT / "verification/daveml_operational_contracts.json"
QUALIFICATION_CLASSES = {"reference_exact", "derived_exact", "surrogate_composite", "synthetic"}
APPLICABILITY = {"required", "not_applicable"}
ENVELOPE_PAIRS = (
    ("mach_min", "mach_max"),
    ("alpha_min_rad", "alpha_max_rad"),
    ("beta_min_rad", "beta_max_rad"),
    ("altitude_min_m", "altitude_max_m"),
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
####


def _load_manifest(path: Path) -> dict[str, Any]:
    if path.suffix.lower() == ".json":
        value = json.loads(path.read_text(encoding="utf-8"))
    else:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"family manifest must be a mapping: {path}")
    return value
####


def _validate_envelope(family_id: str, envelope: dict[str, Any]) -> None:
    for low_key, high_key in ENVELOPE_PAIRS:
        if low_key not in envelope or high_key not in envelope:
            raise ValueError(f"{family_id}: validity envelope requires {low_key}/{high_key}")
        low = float(envelope[low_key])
        high = float(envelope[high_key])
        if not math.isfinite(low) or not math.isfinite(high) or low > high:
            raise ValueError(f"{family_id}: invalid validity envelope {low_key}/{high_key}")
    ####
####


def _manifest_identity(manifest: dict[str, Any], family_id: str) -> tuple[str, str]:
    manifest_family = str(manifest.get("family_id", ""))
    if manifest_family != family_id:
        raise ValueError(f"{family_id}: manifest family_id is {manifest_family!r}")
    if "collection_type" in manifest:
        qualification = str(manifest.get("qualification_class", ""))
        return "collection", qualification
    if manifest.get("manifest_type") != "taoryx.reference-family/v1alpha1":
        raise ValueError(f"{family_id}: unsupported family manifest type")
    source = manifest.get("source")
    if not isinstance(source, dict) or not str(source.get("package_sha256", "")):
        raise ValueError(f"{family_id}: reference manifest has no pinned package hash")
    return "reference", "reference_exact"
####


def validate() -> dict[str, Any]:
    raw = yaml.safe_load(CONTRACTS.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("schema_version") != 1:
        raise ValueError("operational contract registry schema_version must be 1")
    families = raw.get("families")
    if not isinstance(families, list) or not families:
        raise ValueError("operational contract registry requires families")
    seen: set[str] = set()
    entries: list[dict[str, Any]] = []
    for contract in families:
        if not isinstance(contract, dict):
            raise ValueError("operational contract family must be a mapping")
        family_id = str(contract.get("id", ""))
        if not family_id or family_id in seen:
            raise ValueError(f"duplicate or missing operational family id: {family_id!r}")
        seen.add(family_id)
        qualification = str(contract.get("qualification_class", ""))
        if qualification not in QUALIFICATION_CLASSES:
            raise ValueError(f"{family_id}: invalid qualification class {qualification!r}")
        manifest_path = ROOT / str(contract.get("manifest", ""))
        if not manifest_path.is_file():
            raise ValueError(f"{family_id}: missing manifest {manifest_path}")
        manifest = _load_manifest(manifest_path)
        manifest_kind, manifest_qualification = _manifest_identity(manifest, family_id)
        if manifest_qualification and manifest_qualification != qualification:
            raise ValueError(f"{family_id}: contract and manifest qualification classes differ")
        frames = contract.get("frames")
        if not isinstance(frames, dict) or not all(str(frames.get(key, "")).strip() for key in ("body", "navigation", "quaternion_order")):
            raise ValueError(f"{family_id}: incomplete frame contract")
        geometry = contract.get("reference_geometry")
        if not isinstance(geometry, dict) or not all(float(geometry.get(key, 0.0)) > 0.0 for key in ("area_m2", "mean_aerodynamic_chord_m", "span_m")):
            raise ValueError(f"{family_id}: incomplete reference geometry")
        envelope = contract.get("validity_envelope")
        if not isinstance(envelope, dict):
            raise ValueError(f"{family_id}: missing validity envelope")
        _validate_envelope(family_id, envelope)
        bindings = contract.get("bindings")
        if not isinstance(bindings, list) or not bindings:
            raise ValueError(f"{family_id}: requires at least one binding")
        binding_ids: set[str] = set()
        for binding in bindings:
            if not isinstance(binding, dict):
                raise ValueError(f"{family_id}: binding must be a mapping")
            binding_id = str(binding.get("id", ""))
            if not binding_id or binding_id in binding_ids or not bool(binding.get("required")):
                raise ValueError(f"{family_id}: bindings must have unique required ids")
            binding_ids.add(binding_id)
            if not str(binding.get("source_role", "")).strip() or not isinstance(binding.get("outputs"), list) or not binding["outputs"]:
                raise ValueError(f"{family_id}: binding {binding_id!r} lacks source role or outputs")
        applicability = contract.get("applicability")
        if not isinstance(applicability, dict) or set(applicability) != {"trim", "linearization", "tuning", "objectives", "scenarios"}:
            raise ValueError(f"{family_id}: incomplete applicability declaration")
        if any(str(value) not in APPLICABILITY for value in applicability.values()):
            raise ValueError(f"{family_id}: invalid applicability value")
        nonclaims = contract.get("nonclaims")
        if not isinstance(nonclaims, list) or not nonclaims or not all(str(item).strip() for item in nonclaims):
            raise ValueError(f"{family_id}: nonclaims are required")
        entries.append(
            {
                "family_id": family_id,
                "manifest": str(manifest_path.relative_to(ROOT)).replace("\\", "/"),
                "manifest_kind": manifest_kind,
                "manifest_sha256": _sha256(manifest_path),
                "qualification_class": qualification,
                "binding_count": len(bindings),
                "applicability": dict(applicability),
                "nonclaim_count": len(nonclaims),
                "status": "verified",
            }
        )
    ####
    return {
        "schema_version": 1,
        "report_type": "taoryx.daveml-operational-contracts/v1",
        "contract_registry": str(CONTRACTS.relative_to(ROOT)).replace("\\", "/"),
        "contract_registry_sha256": _sha256(CONTRACTS),
        "claim_boundary": raw["claim_boundary"],
        "family_count": len(entries),
        "families": entries,
        "status": "verified",
    }
####


def main() -> int:
    report = validate()
    REPORT.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0
####


if __name__ == "__main__":
    raise SystemExit(main())
