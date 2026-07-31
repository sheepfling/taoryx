"""Build the Alpha 3 direct-wrench family evidence matrix."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "verification/alpha3_direct_wrench_matrix"

DIRECT_ARTIFACTS = {
    "skywalker_x8": "verification/alpha3_x8_direct_wrench/manifest.json",
    "b747": "artifacts/showcases/airbreathing-racetrack-fidelity-ladder/b747-racetrack-altitude-turns-6dof-v1/summary.json",
    "a320_openap_3dof": "verification/alpha3_a320_direct_wrench/manifest.json",
    "f16_s119": "verification/alpha3_f16_direct_wrench/manifest.json",
    "x15": "verification/alpha3_x15_direct_wrench/manifest.json",
    "hummingbird": "verification/alpha3_hummingbird_direct_wrench_debug/manifest.json",
    "hl20_mod_k": "verification/alpha3_hl20_direct_wrench/manifest.json",
    "reference_nesc_two_stage_rocket": "verification/alpha3_nesc_direct_wrench/manifest.json",
}

EXCEPTIONS: dict[str, dict[str, object]] = {
    "tumbling_body": {
        "status": "not_applicable_native_uncontrolled",
        "blocker": "no controller or effector exists to realize a requested wrench",
        "available_lineage": "native passive rigid-body equations with averaged-area 3DOF reduction",
    },
}


def _load(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected object in {path}")
    return payload
    ####


def build() -> dict[str, object]:
    records: list[dict[str, object]] = []
    for family_id, relative_path in DIRECT_ARTIFACTS.items():
        payload = _load(ROOT / relative_path)
        claim = payload.get("claim", {})
        evaluation = payload.get("truth_evaluation", payload.get("evaluation", payload.get("metrics", {})))
        if not isinstance(claim, dict):
            claim = {}
        if not isinstance(evaluation, dict):
            evaluation = {}
        status = str(payload.get("status", claim.get("status", "available")))
        records.append(
            {
                "family_id": family_id,
                "status": status,
                "artifact": relative_path,
                "fidelity": payload.get("fidelity", payload.get("fidelity_tier")),
                "mission_pass": payload.get("mission_pass", evaluation.get("mission_pass", status == "nominal_case_pass")),
                "evidence_tier": payload.get("evidence_level", claim.get("evidence_tier", "T3_screen_only")),
                "direct_body_moment_injection": payload.get("direct_body_moment_injection", claim.get("direct_body_moment_injection", True)),
                "physical_effector_allocation": payload.get("physical_effector_allocation", claim.get("physical_effector_allocation", False)),
                "claim_boundary": payload.get("nonclaims", claim.get("nonclaims", payload.get("claim_boundary"))),
            }
        )
    records.extend({"family_id": family_id, **details} for family_id, details in EXCEPTIONS.items())
    result: dict[str, object] = {
        "schema": "taoryx.alpha3-direct-wrench-matrix/v1alpha1",
        "status": "development_direct_wrench_matrix",
        "claim_boundary": "This matrix inventories direct-wrench evidence, physical-path precedence, and blockers. It does not promote a family to full-envelope or hardware qualification.",
        "records": records,
        "summary": {
            "family_count": len(records),
            "nominal_direct_wrench_pass_count": sum(bool(record.get("mission_pass")) for record in records),
            "direct_wrench_artifact_count": len(DIRECT_ARTIFACTS),
            "direct_wrench_bridge_pass_count": sum(
                bool(record.get("mission_pass")) for record in records if record["family_id"] != "tumbling_body"
            ),
            "blocked_or_exception_count": len(EXCEPTIONS),
        },
        "reproduction": "PYTHONPATH=src python3 tools/build_alpha3_direct_wrench_matrix.py",
    }
    OUTPUT.mkdir(parents=True, exist_ok=True)
    (OUTPUT / "manifest.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (OUTPUT / "reproduction.txt").write_text(str(result["reproduction"]) + "\n", encoding="utf-8")
    return result
    ####


def main() -> int:
    result = build()
    print(json.dumps(result["summary"], indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
