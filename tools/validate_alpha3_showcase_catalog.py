#!/usr/bin/env python3
"""Verify the complete Alpha 3 family showcase catalog.

This is a catalog/artifact gate, not a vehicle-qualification gate.  It proves
that every target family has a recipe, paired fidelity evidence, a passing
cross-fidelity report, and a canonical visual board with reproducible hashes.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import yaml

from taoryx.showcase import ShowcaseArchetypeCatalog

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_showcase_catalog"
CATALOG = ROOT / "verification/showcase_archetype_catalog.yaml"
READINESS = ROOT / "verification/alpha3_robustness_matrix/manifest.json"
DIRECT_WRENCH_CONTRACT = ROOT / "verification/alpha3_direct_wrench_contract/manifest.json"

FAMILY_BINDINGS: dict[str, dict[str, Any]] = {
    "skywalker_x8": {
        "recipe": "x8-fixed-wing-racetrack-response-v1",
        "board": "verification/alpha3_family_qualification/x8-racetrack-altitude-turns-pseudo-6dof-v1/qualification_board.png",
        "supplemental_evidence": ("verification/alpha3_x8_direct_wrench/manifest.json",),
    },
    "b747": {
        "recipe": "b747-transport-energy-arrival-v1",
        "board": "verification/alpha3_family_qualification/b747-racetrack-altitude-turns-pseudo-6dof-v1/qualification_board.png",
        "supplemental_evidence": (
            "artifacts/showcases/airbreathing-racetrack-fidelity-ladder/b747-racetrack-altitude-turns-6dof-v1/summary.json",
        ),
    },
    "a320": {
        "recipe": "a320-derived-versus-surrogate-v1",
        "board": "verification/a320_racetrack/pseudo_6dof_kinematic_bridge_board.png",
        "supplemental_evidence": ("verification/alpha3_a320_direct_wrench/manifest.json",),
    },
    "f16_s119": {
        "recipe": "f16-maneuver-trim-evidence-v1",
        "board": "verification/f16_racetrack/pseudo_6dof_kinematic_bridge_board.png",
        "supplemental_evidence": (
            "verification/alpha3_f16_direct_wrench/manifest.json",
            "artifacts/showcases/f16-s119-racetrack-fidelity-ladder/6dof-direct-wrench/evidence.json",
        ),
    },
    "x15": {
        "recipe": "x15-boost-glide-storyboard-v1",
        "board": "verification/x15_fidelity_ladder/x15_fidelity_evidence_board.png",
        "supplemental_evidence": (
            "verification/alpha3_regime_direct_wrench_readiness/manifest.json",
            "verification/alpha3_x15_direct_wrench/manifest.json",
        ),
    },
    "hummingbird": {
        "recipe": "hummingbird-multirotor-hover-yaw-contact-v1",
        "board": "verification/alpha3_hummingbird_directional/directional_translation_evidence_board.png",
        "auxiliary_evidence": "verification/alpha3_hummingbird_directional/manifest.json",
        "supplemental_evidence": (
            "verification/alpha3_hummingbird_native_horizontal/manifest.json",
            "verification/alpha3_hummingbird_native_vertical/manifest.json",
            "verification/alpha3_hummingbird_native_pad_to_pad/hummingbird-pad-to-pad-altitude-yaw-individual-rotor-v1/summary.json",
            "verification/alpha3_regime_direct_wrench_readiness/manifest.json",
        ),
    },
    "hl20_mod_k": {
        "recipe": "hl20-glide-entry-evidence-v1",
        "board": "verification/alpha3_hl20_fidelity/composites/hl20-ca-hi-flight-composite.png",
        "supplemental_evidence": (
            "verification/alpha3_hl20_direct_wrench/manifest.json",
            "verification/alpha3_hl20_source_allocation/manifest.json",
            "verification/alpha3_hl20_source_surface_replay/manifest.json",
            "verification/alpha3_regime_direct_wrench_readiness/manifest.json",
        ),
    },
    "reference_nesc_two_stage_rocket": {
        "recipe": "nesc-two-stage-launch-lineage-v1",
        "board": "artifacts/showcases/daveml-families/nesc-two-stage-launch-lineage-v1/evidence-board.png",
        "supplemental_evidence": (
            "verification/alpha3_regime_direct_wrench_readiness/manifest.json",
            "verification/alpha3_nesc_direct_wrench/manifest.json",
        ),
    },
    "tumbling_body": {
        "recipe": "tumbling-body-passive-deployment-v1",
        "board": "verification/alpha3_tumbling_body/tumbling_body_fidelity_evidence_board.png",
    },
}
####


def _load(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected mapping in {path}")
    return payload
    ####


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return path.name
    ####


def build_catalog(output: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    """Build the deterministic nine-family showcase catalog manifest."""

    catalog = ShowcaseArchetypeCatalog(**yaml.safe_load(CATALOG.read_text(encoding="utf-8")))
    readiness = _load(READINESS)
    direct_wrench_contract = _load(DIRECT_WRENCH_CONTRACT)
    recipes = {recipe.id: recipe for recipe in catalog.recipes}
    readiness_by_family = {family["family_id"]: family for family in readiness["families"]}
    records: list[dict[str, Any]] = []
    failures: list[str] = []
    if direct_wrench_contract.get("status") != "contract_verified":
        failures.append("direct-wrench contract ledger is not verified")

    for family_id, binding in FAMILY_BINDINGS.items():
        recipe_id = binding["recipe"]
        board = ROOT / binding["board"]
        family = readiness_by_family.get(family_id)
        recipe = recipes.get(recipe_id)
        if recipe is None:
            failures.append(f"{family_id}: missing recipe {recipe_id}")
        if family is None:
            failures.append(f"{family_id}: missing readiness record")
        if not board.is_file():
            failures.append(f"{family_id}: missing board {binding['board']}")

        fidelity_records = [] if family is None else family.get("fidelity_records", [])
        pair_pass = len(fidelity_records) == 2 and all(bool(record.get("mission_pass")) for record in fidelity_records)
        cross_pass = family is not None and family.get("cross_fidelity_status") == "pass"
        if not pair_pass:
            failures.append(f"{family_id}: paired fidelity evidence is incomplete or not passing")
        if not cross_pass:
            failures.append(f"{family_id}: cross-fidelity report is not passing")

        records.append(
            {
                "family_id": family_id,
                "recipe_id": recipe_id,
                "recipe_family": None if recipe is None else recipe.family,
                "recipe_priority": None if recipe is None else recipe.priority,
                "board": {"path": binding["board"], "sha256": _sha256(board) if board.is_file() else None},
                "auxiliary_evidence": (
                    {
                        "path": binding["auxiliary_evidence"],
                        "sha256": _sha256(ROOT / binding["auxiliary_evidence"]),
                    }
                    if binding.get("auxiliary_evidence") and (ROOT / binding["auxiliary_evidence"]).is_file()
                    else None
                ),
                "supplemental_evidence": [
                    {
                        "path": relative_path,
                        "sha256": _sha256(ROOT / relative_path),
                    }
                    for relative_path in binding.get("supplemental_evidence", ())
                    if (ROOT / relative_path).is_file()
                ],
                "paired_fidelity": {
                    "status": None if family is None else family.get("pair_outcome"),
                    "mission_pass": pair_pass,
                    "records": [
                        {
                            "fidelity": record.get("fidelity"),
                            "artifact": record.get("artifact"),
                            "sha256": _sha256(ROOT / str(record["artifact"])) if ROOT.joinpath(str(record["artifact"])).is_file() else None,
                        }
                        for record in fidelity_records
                    ],
                },
                "cross_fidelity_status": "pass" if cross_pass else "not_passed",
                "claim_boundary": family.get("fidelity_records", [{}])[0].get("claim_boundary") if family else "missing readiness record",
            }
        )

    report: dict[str, Any] = {
        "schema": "taoryx.alpha3-showcase-catalog/v1alpha1",
        "status": "development_catalog_verified" if not failures else "catalog_incomplete",
        "claim_boundary": "This verifies artifact composition and declared evidence boundaries; it does not promote a family to physical-effector, source-exact, operational, or full-envelope qualification.",
        "catalog": {"path": _relative(CATALOG), "sha256": _sha256(CATALOG), "recipe_count": len(catalog.recipes)},
        "readiness": {"path": _relative(READINESS), "sha256": _sha256(READINESS)},
        "direct_wrench_contract": {
            "path": _relative(DIRECT_WRENCH_CONTRACT),
            "sha256": _sha256(DIRECT_WRENCH_CONTRACT),
            "status": direct_wrench_contract.get("status"),
            "record_count": len(direct_wrench_contract.get("records", [])),
        },
        "family_count": len(records),
        "board_count": sum(record["board"]["sha256"] is not None for record in records),
        "paired_fidelity_count": sum(record["paired_fidelity"]["mission_pass"] for record in records),
        "cross_fidelity_pass_count": sum(record["cross_fidelity_status"] == "pass" for record in records),
        "families": records,
        "failures": failures,
        "reproduction": "PYTHONPATH=src python3 tools/validate_alpha3_showcase_catalog.py",
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "manifest.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(str(report["reproduction"]) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    report = build_catalog(arguments.output)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "development_catalog_verified" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
