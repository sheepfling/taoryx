#!/usr/bin/env python3
"""Build the normalized Alpha 3 direct-wrench contract ledger.

The family-specific run artifacts predate one another and therefore do not
share identical field names.  This ledger is the authoritative cross-family
record for the four evidence dimensions that must never be implicit:
trim/release, force/moment composition, resources, and operating envelope.
It also carries the exact control realization and claim boundary for each
family, including the passive uncontrolled-body exception.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "verification/alpha3_direct_wrench_contract"


FAMILY_CONTRACTS: dict[str, dict[str, Any]] = {
    "skywalker_x8": {
        "artifact": "verification/alpha3_x8_direct_wrench/manifest.json",
        "trim": {"status": "source_trim_recorded", "source": "table-backed local trim; boundary findings retained"},
        "force_moment": {"source_loads": "table-backed nonlinear loads", "control_loads": "bounded direct body wrench", "composition": "source loads plus achieved injected wrench"},
        "resources": {"status": "fixed_mass_local_screen", "modeled": ["mass"], "not_modeled": ["fuel burn", "actuator energy"]},
        "envelope": {"status": "local_source_table_domain", "limits": ["declared source attitude/rate bounds", "local table domain"]},
        "promotion": "T3_direct_wrench_nominal_local",
    },
    "b747": {
        "artifact": "artifacts/showcases/airbreathing-racetrack-fidelity-ladder/b747-racetrack-altitude-turns-6dof-v1/summary.json",
        "trim": {"status": "source_table_transport_trim_or_local_balance", "source": "declared transport operating condition"},
        "force_moment": {"source_loads": "source-table aerodynamic loads", "control_loads": "bounded direct body wrench", "composition": "source loads plus logged direct-wrench control load"},
        "resources": {"status": "explicit_fixed_mass_zero_propellant", "modeled": ["fixed mass"], "not_modeled": ["fuel burn", "surface actuator energy"]},
        "envelope": {"status": "nominal_transport_racetrack", "limits": ["source table domain", "declared beta and rate limits", "route terminal gates"]},
        "promotion": "T3_direct_wrench_nominal_mission",
    },
    "a320": {
        "artifact": "verification/alpha3_a320_direct_wrench/manifest.json",
        "trim": {"status": "operating_point_trim_recorded", "source": "OpenAP/JSBSim-derived local operating point"},
        "force_moment": {"source_loads": "source-calibrated surrogate loads", "control_loads": "bounded direct body wrench", "composition": "local source response plus achieved injected wrench"},
        "resources": {"status": "fixed_mass_local_screen", "modeled": ["mass"], "not_modeled": ["fuel burn", "surface actuator energy"]},
        "envelope": {"status": "single_local_operating_point", "limits": ["declared Mach/altitude point", "local alpha/beta/rate bounds"]},
        "promotion": "T3_direct_wrench_nominal_local",
    },
    "f16_s119": {
        "artifact": "verification/alpha3_f16_direct_wrench/manifest.json",
        "trim": {"status": "source_trim_recorded", "source": "F-16 S-119 source-plant local trim"},
        "force_moment": {"source_loads": "source nonlinear loads", "control_loads": "bounded direct body wrench", "composition": "source loads plus achieved injected wrench"},
        "resources": {"status": "fixed_mass_local_screen", "modeled": ["mass"], "not_modeled": ["fuel burn", "physical surface energy"]},
        "envelope": {"status": "local_subsonic_source_point", "limits": ["source alpha/beta domain", "declared local rate limits", "route boundary retained separately"]},
        "promotion": "T3_direct_wrench_nominal_local",
    },
    "x15": {
        "artifact": "verification/alpha3_x15_direct_wrench/manifest.json",
        "trim": {"status": "source_trim_blocked_before_T1", "source": "source-trim readiness artifact; direct balance bias is not trim"},
        "force_moment": {"source_loads": "retained X-15 source rigid-body loads", "control_loads": "bounded direct body wrench", "composition": "source loads plus explicit local load-balance bias and feedback wrench"},
        "resources": {"status": "release_glide_screen_only", "modeled": ["fixed release mass"], "not_modeled": ["powered propellant flow", "phase-dependent actuator energy"]},
        "envelope": {"status": "unpowered_release_glide_local", "limits": ["source aerodynamic table domain", "local Mach/alpha/beta region", "no terminal handoff"]},
        "promotion": "T3_direct_wrench_bridge_local_source_trim_blocked",
    },
    "hummingbird": {
        "artifact": "verification/alpha3_hummingbird_direct_wrench_debug/manifest.json",
        "trim": {"status": "native_rotor_trim_preferred", "source": "debug comparator local trim; native rotor packet is authoritative physical path"},
        "force_moment": {"source_loads": "RotorPy-derived local loads", "control_loads": "bounded direct body wrench for comparator only", "composition": "debug source loads plus achieved injected wrench; native rotor allocation separate"},
        "resources": {"status": "native_path_resource_model_preferred", "modeled": ["mass in comparator"], "not_modeled": ["motor-level battery draw in direct comparator"]},
        "envelope": {"status": "hover_local_debug", "limits": ["local hover response envelope", "direct comparator bounds"]},
        "promotion": "T3_direct_wrench_bridge_native_rotor_preferred",
    },
    "hl20_mod_k": {
        "artifact": "verification/alpha3_hl20_direct_wrench/manifest.json",
        "trim": {"status": "source_trim_pending", "source": "source-load operating point; local balance bias is not source trim"},
        "force_moment": {"source_loads": "pinned DAVE-ML source loads", "control_loads": "bounded direct body wrench", "composition": "source loads plus explicit local balance bias and feedback wrench"},
        "resources": {"status": "fixed_mass_release_screen", "modeled": ["fixed mass"], "not_modeled": ["booster/propellant transients", "surface actuator energy"]},
        "envelope": {"status": "Mach/alpha/altitude_single_point", "limits": ["DAVE-ML source domain", "declared alpha/beta point", "no closed-loop bank/energy mission"]},
        "promotion": "T3_direct_wrench_bridge_source_surface_preferred",
    },
    "reference_nesc_two_stage_rocket": {
        "artifact": "verification/alpha3_nesc_direct_wrench/manifest.json",
        "trim": {"status": "source_attitude_trim_unavailable", "source": "retained translation/staging history only"},
        "force_moment": {"source_loads": "translation-derived force; source force/moment channels unavailable", "control_loads": "bounded direct body wrench", "composition": "engineering translation extension plus achieved injected wrench"},
        "resources": {"status": "source_staging_history", "modeled": ["mass/stage history"], "not_modeled": ["attitude propellant allocation", "gimbal energy"]},
        "envelope": {"status": "translation_extension_only", "limits": ["source translation history", "declared engineering inertia", "ECI/body alignment assumption"]},
        "promotion": "T3_direct_wrench_bridge_engineering_translation",
    },
    "tumbling_body": {
        "artifact": "verification/alpha3_tumbling_body/fidelity_ladder/manifest.json",
        "trim": {"status": "passive_initial_release", "source": "native uncontrolled rigid-body release"},
        "force_moment": {"source_loads": "native passive aerodynamic/gravity loads", "control_loads": "none", "composition": "uncontrolled rigid-body equations"},
        "resources": {"status": "passive_no_consumable", "modeled": ["mass"], "not_modeled": ["controller resource use"]},
        "envelope": {"status": "native_rigid_body_and_averaged_area_reduction", "limits": ["declared shape/rate/area policy", "passive deployment domain"]},
        "promotion": "native_uncontrolled_6dof; direct_wrench_not_applicable",
    },
}


def _load(relative_path: str) -> dict[str, Any]:
    path = ROOT / relative_path
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected object in {relative_path}")
    return payload
    ####


def build(output: Path = OUTPUT) -> dict[str, object]:
    """Validate and write the normalized family contract ledger."""

    records: list[dict[str, object]] = []
    failures: list[str] = []
    for family_id, contract in FAMILY_CONTRACTS.items():
        artifact_path = str(contract["artifact"])
        try:
            payload = _load(artifact_path)
        except (FileNotFoundError, json.JSONDecodeError, ValueError) as exc:
            failures.append(f"{family_id}: cannot load evidence artifact: {exc}")
            payload = {}
        claim = payload.get("claim", {})
        if not isinstance(claim, dict):
            claim = {}
        evaluation = payload.get("evaluation", payload.get("metrics", {}))
        if not isinstance(evaluation, dict):
            evaluation = {}
        evidence_status = payload.get("status", claim.get("status"))
        if evidence_status is None and evaluation.get("mission_pass") is True:
            evidence_status = "nominal_case_pass"
        if evidence_status is None:
            failures.append(f"{family_id}: evidence artifact has no status")
        control_realization = str(payload.get("control_realization", ""))
        fidelity_text = str(payload.get("fidelity", payload.get("fidelity_tier", "")))
        direct_declared = control_realization in {"direct_wrench", "direct_wrench_debug_bypass"} or "direct_wrench" in fidelity_text
        if family_id != "tumbling_body" and not direct_declared:
            failures.append(f"{family_id}: direct-wrench artifact does not declare control_realization=direct_wrench")
        for dimension in ("trim", "force_moment", "resources", "envelope"):
            if not isinstance(contract.get(dimension), dict):
                failures.append(f"{family_id}: missing normalized {dimension} contract")
        records.append(
            {
                "family_id": family_id,
                "evidence_artifact": artifact_path,
                "evidence_status": evidence_status,
                "fidelity": payload.get("fidelity", payload.get("fidelity_tier", "native_rigid_body_6dof")),
                "control_realization": control_realization or ("direct_wrench" if direct_declared else "native_uncontrolled"),
                "evidence_tier": claim.get("evidence_tier", payload.get("evidence_level", "passive_native")),
                "trim": contract["trim"],
                "force_moment": contract["force_moment"],
                "resources": contract["resources"],
                "envelope": contract["envelope"],
                "promotion_status": contract["promotion"],
                "proves": claim.get("proves", "") if family_id != "tumbling_body" else "Passive native rigid-body and averaged-area reduction evidence.",
                "nonclaims": payload.get("nonclaims", claim.get("nonclaims", [payload.get("claim_boundary", "Claim boundary is declared in the source artifact.")])),
            }
        )
    result: dict[str, object] = {
        "schema": "taoryx.alpha3-direct-wrench-contract/v1alpha1",
        "status": "contract_verified" if not failures else "contract_incomplete",
        "claim_boundary": "This ledger normalizes the direct-wrench bridge contract; it does not promote generalized-wrench evidence to physical-effector qualification.",
        "required_dimensions": ["trim", "force_moment", "resources", "envelope", "claim_boundary"],
        "records": records,
        "failures": failures,
        "reproduction": "PYTHONPATH=src python3 tools/build_alpha3_direct_wrench_contract.py",
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "manifest.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(str(result["reproduction"]) + "\n", encoding="utf-8")
    return result
    ####


def main() -> int:
    result = build()
    print(json.dumps({"status": result["status"], "record_count": len(result["records"]), "failures": result["failures"]}, indent=2, sort_keys=True))
    return 0 if result["status"] == "contract_verified" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
    ####
