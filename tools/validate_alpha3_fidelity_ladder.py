#!/usr/bin/env python3
"""Verify that every Alpha 3 target family has a paired fidelity packet."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_fidelity_ladder"
PROMOTION_MANIFEST = ROOT / "verification/alpha3_fidelity_evidence.yaml"

DEVELOPMENT_MANIFESTS = {
    "x15": ROOT / "verification/x15_fidelity_ladder/manifest.json",
    "hummingbird": ROOT / "verification/alpha3_hummingbird_development/manifest.json",
    "hl20_mod_k": ROOT / "verification/alpha3_hl20_development/manifest.json",
    "reference_nesc_two_stage_rocket": ROOT / "verification/alpha3_nesc_development/manifest.json",
    "tumbling_body": ROOT / "verification/alpha3_tumbling_body/fidelity_ladder/manifest.json",
}

FAMILY_IDS = ("skywalker_x8", "b747", "a320", "f16_s119", *DEVELOPMENT_MANIFESTS)


def _load_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert isinstance(payload, dict)
    return payload
    ####


def _promotion_records() -> list[dict[str, Any]]:
    payload = yaml.safe_load(PROMOTION_MANIFEST.read_text(encoding="utf-8"))
    assert isinstance(payload, dict)
    records = payload.get("records")
    assert isinstance(records, list)
    return [record for record in records if isinstance(record, dict)]
    ####


def _family_key(profile_id: str) -> str:
    family = profile_id.split(".", 1)[0]
    if family.startswith("a320"):
        return "a320"
    if family == "hl20":
        return "hl20_mod_k"
    if family == "nesc_rocket":
        return "reference_nesc_two_stage_rocket"
    return family
    ####


def build_summary() -> dict[str, Any]:
    """Collect promotion and development packets into one deterministic index."""

    # A promoted record supersedes the same row in its development manifest.
    # Keep one canonical row per family/fidelity so cross-fidelity reports do
    # not see duplicate tiers after promotion.
    records_by_key: dict[tuple[str, str], dict[str, Any]] = {}
    for record in _promotion_records():
        artifact = ROOT / str(record["artifact"])
        family_id = _family_key(str(record["profile_id"]))
        fidelity = str(record["fidelity"])
        records_by_key[(family_id, fidelity)] = {
            "family_id": family_id,
            "fidelity": fidelity,
            "status": record["status"],
            "mission_pass": _load_json(artifact).get("evaluation", {}).get("mission_pass", True),
            "artifact": artifact.relative_to(ROOT).as_posix(),
            "claim_boundary": record.get("claim_boundary"),
        }
    for family_id, manifest_path in DEVELOPMENT_MANIFESTS.items():
        manifest = _load_json(manifest_path)
        for record in manifest.get("records", []):
            assert isinstance(record, dict)
            artifact_name = str(record["artifact"])
            artifact = ROOT / artifact_name if artifact_name.startswith("verification/") else manifest_path.parent / artifact_name
            key = (family_id, str(record["fidelity"]))
            if key not in records_by_key:
                records_by_key[key] = {
                    "family_id": family_id,
                    "fidelity": record["fidelity"],
                    "status": record["status"],
                    "mission_pass": record["mission_pass"],
                    "artifact": artifact.relative_to(ROOT).as_posix(),
                    "claim_boundary": manifest.get("promotion_blocker"),
                }
    records = list(records_by_key.values())
    seen_families = {str(record["family_id"]) for record in records}
    missing_families = sorted(set(FAMILY_IDS) - seen_families)
    paired: dict[str, set[str]] = {}
    for record in records:
        paired.setdefault(str(record["family_id"]), set()).add(str(record["fidelity"]))
    incomplete_pairs = sorted(family for family in seen_families if not {"point_mass_3dof", "pseudo_6dof"}.issubset(paired[family]) and family != "tumbling_body")
    return {
        "schema": "taoryx.alpha3-fidelity-ladder-index/v1alpha1",
        "status": "pass" if not missing_families and not incomplete_pairs and all(record["mission_pass"] for record in records) else "fail",
        "family_count": len(seen_families),
        "families": sorted(seen_families),
        "records": sorted(records, key=lambda record: (str(record["family_id"]), str(record["fidelity"]))),
        "missing_families": missing_families,
        "incomplete_pairs": incomplete_pairs,
        "cross_fidelity_manifest": "verification/alpha3_cross_fidelity/manifest.json" if (ROOT / "verification/alpha3_cross_fidelity/manifest.json").is_file() else None,
        "interpretation": "Development records prove executable nominal witnesses; nominal_case_pass records are the only records currently authorized for automatic lowering.",
    }
    ####


def write_summary(output: Path) -> dict[str, Any]:
    """Write the Alpha 3 fidelity index."""

    output.mkdir(parents=True, exist_ok=True)
    summary = build_summary()
    path = output / "manifest.json"
    path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary
    ####


def main() -> int:
    """Verify and write the Alpha 3 fidelity index."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    summary = write_summary(arguments.output)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["status"] == "pass" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
