"""Build the fail-closed direct-wrench readiness records for regime families.

The fixed-wing direct-wrench witnesses already have executable local recovery
artifacts.  This builder records the remaining regime-specific decisions in
the same machine-readable form without treating an open-loop replay,
response-law surrogate, or missing actuator contract as a controller pass.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "verification/alpha3_regime_direct_wrench_readiness"


def _load(relative_path: str) -> dict[str, Any]:
    payload = json.loads((ROOT / relative_path).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected an object in {relative_path}")
    return payload
    ####


def _record(
    family_id: str,
    *,
    status: str,
    fidelity: str,
    evidence_tier: str,
    direct_wrench_mode: str,
    direct_body_moment_injection: bool,
    physical_effector_allocation: bool,
    blocker: str,
    available_evidence: list[str],
    required_inputs: list[str],
    claim: str,
    nonclaims: list[str],
) -> dict[str, object]:
    return {
        "family_id": family_id,
        "status": status,
        "fidelity": fidelity,
        "evidence_tier": evidence_tier,
        "direct_wrench_mode": direct_wrench_mode,
        "mission_pass": None,
        "direct_body_moment_injection": direct_body_moment_injection,
        "physical_effector_allocation": physical_effector_allocation,
        "claim": claim,
        "claim_boundary": nonclaims,
        "blocker": blocker,
        "available_evidence": available_evidence,
        "required_inputs_to_promote": required_inputs,
    }
    ####


def build() -> dict[str, object]:
    """Return and write the regime-specific readiness matrix."""

    x15 = _load("verification/generated/x15_physical_lqr_readiness.json")
    nesc = _load("verification/alpha3_nesc_attitude_data_contract/manifest.json")
    hl20_surface = _load("verification/alpha3_hl20_source_surface_replay/manifest.json")
    hl20_allocation = _load("verification/alpha3_hl20_source_allocation/manifest.json")
    hummingbird = _load(
        "verification/alpha3_hummingbird_native_pad_to_pad/hummingbird-pad-to-pad-altitude-yaw-individual-rotor-v1/summary.json"
    )

    records = [
        _record(
            "x15",
            status="direct_wrench_bridge_available_source_trim_blocked",
            fidelity="rigid_body_6dof_direct_wrench",
            evidence_tier="T0_structural",
            direct_wrench_mode="bridge_available",
            direct_body_moment_injection=True,
            physical_effector_allocation=False,
            blocker="source-bounded trim must pass before a direct-wrench recovery screen can be promoted beyond structural readiness",
            available_evidence=[
                "verification/alpha3_x15_direct_wrench/manifest.json",
                "verification/generated/x15_physical_lqr_readiness.json",
                "tests/fixtures/x15_coherent_6dof_public_research_v1/tables/x15_static_6axis.tbl",
                "tests/fixtures/x15_coherent_6dof_public_research_v1/controls/control_coordinate_mapping.csv",
            ],
            required_inputs=[
                "source-bounded powered or glide trim with residual contract",
                "resolved symmetric/differential stabilator gearing and signs",
                "propulsion command contract or explicit coast-only scope",
                "RCS geometry and impulse data before low-authority blending",
            ],
            claim="The X-15 source control and actuator contract is inventoried and a local direct-wrench bridge is reproducible, while source-bounded T1 trim remains the next promotion gate.",
            nonclaims=[
                "No direct-wrench recovery result is claimed before T1 trim.",
                "No physical-effector LQR or powered/coast authority transition is claimed.",
            ],
        ),
        _record(
            "hl20_mod_k",
            status="surface_path_preferred_direct_wrench_debug_not_promoted",
            fidelity="rigid_body_6dof_direct_wrench",
            evidence_tier="T0_source_effectivity",
            direct_wrench_mode="bridge_available",
            direct_body_moment_injection=True,
            physical_effector_allocation=False,
            blocker="the direct-wrench bridge is available, but source surface replay and bounded source-load allocation remain more informative until source-bound trim is connected",
            available_evidence=[
                "verification/alpha3_hl20_direct_wrench/manifest.json",
                "verification/alpha3_hl20_source_surface_replay/manifest.json",
                "verification/alpha3_hl20_source_allocation/manifest.json",
                "families/reference_hl20_mod_k/family.yaml",
            ],
            required_inputs=[
                "source-bound equilibrium with seven surface commands",
                "nonlinear source-load residual after trim",
                "surface actuator dynamics and rate limits",
                "closed-loop bank, alpha, and energy mission contract",
            ],
            claim="The HL-20 source surfaces and source-load direction/allocation probes are available for the physical path.",
            nonclaims=[
                "The open-loop allocation probe is not a direct-wrench controller pass.",
                "No source-exact trim, closed-loop controller, or terminal arrival is claimed.",
            ],
        ),
        _record(
            "reference_nesc_two_stage_rocket",
            status="direct_wrench_bridge_available_gimbal_blocked",
            fidelity="rigid_body_6dof_direct_wrench",
            evidence_tier="T0_translation_only",
            direct_wrench_mode="bridge_available",
            direct_body_moment_injection=True,
            physical_effector_allocation=False,
            blocker="the retained NESC package is a translation/mass/staging replay and declares no participating attitude or gimbal input",
            available_evidence=[
                "verification/alpha3_nesc_direct_wrench/manifest.json",
                "verification/alpha3_nesc_attitude_data_contract/manifest.json",
                "verification/daveml_nesc_reduction_qualification.json",
                "families/reference_nesc_two_stage_rocket/family.yaml",
            ],
            required_inputs=[
                "gimbal or thrust-vector input channels and signs",
                "control-dependent force and moment outputs or effectiveness deck",
                "stage-dependent actuator limits and rates",
                "event-aligned attitude and body-rate history",
            ],
            claim="The NESC source-data boundary is audited and explicitly blocks invented direct-wrench authority.",
            nonclaims=[
                "No source-exact attitude, gimbal, or direct-wrench recovery result is claimed.",
                "Static aerodynamic moments are not treated as control derivatives.",
            ],
        ),
        _record(
            "hummingbird",
            status="native_rotor_path_preferred_direct_wrench_debug_only",
            fidelity="rigid_body_6dof_direct_wrench",
            evidence_tier="T5_nonlinearly_validated_native_rotor",
            direct_wrench_mode="physical_preferred",
            direct_body_moment_injection=True,
            physical_effector_allocation=True,
            blocker="none for the declared native rotor path; direct wrench remains a diagnostic bypass and cannot replace rotor allocation",
            available_evidence=[
                "verification/alpha3_hummingbird_native_pad_to_pad/hummingbird-pad-to-pad-altitude-yaw-individual-rotor-v1/summary.json",
                "verification/alpha3_hummingbird_native_horizontal/manifest.json",
                "verification/alpha3_hummingbird_native_vertical/manifest.json",
                "verification/alpha3_hummingbird_direct_wrench_debug/manifest.json",
            ],
            required_inputs=[
                "rotor-resolved wind and voltage envelope before broader promotion",
                "explicit motor failure and battery authority cases",
            ],
            claim="The Hummingbird native individual-rotor path remains the preferred 6DOF control realization.",
            nonclaims=[
                "A direct-wrench bypass is not evidence of motor or rotor allocation.",
                "The native packet does not claim full rotorcraft aerodynamics beyond its declared model.",
            ],
        ),
    ]
    result: dict[str, object] = {
        "schema": "taoryx.alpha3-regime-direct-wrench-readiness/v1alpha1",
        "status": "development_fail_closed",
        "claim_boundary": "Readiness and promotion-boundary inventory; no blocked family is promoted by this artifact.",
        "records": records,
        "source_statuses": {
            "x15": x15.get("status"),
            "hl20_surface_replay": hl20_surface.get("status"),
            "hl20_source_allocation": hl20_allocation.get("status"),
            "nesc_attitude_contract": nesc.get("status"),
            "hummingbird_native": hummingbird.get("status"),
        },
        "summary": {
            "record_count": len(records),
            "direct_wrench_bridge_count": sum(
                record["direct_wrench_mode"] in {"bridge_available", "physical_preferred"} for record in records
            ),
            "promoted_direct_wrench_count": sum(record["mission_pass"] is True for record in records),
            "blocked_or_debug_count": sum(record["mission_pass"] is None for record in records),
        },
        "reproduction": "PYTHONPATH=src python3 tools/build_alpha3_regime_direct_wrench_readiness.py",
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
    ####
