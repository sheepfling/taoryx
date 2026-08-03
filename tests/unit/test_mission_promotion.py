from __future__ import annotations

from pathlib import Path

from taoryx.mission_promotion import assess_mission_promotion, mission_proposal_fingerprint
from taoryx.powered_fixed_wing_mission_compiler import compile_powered_fixed_wing_racetrack, load_powered_fixed_wing_mission_profiles

ROOT = Path(__file__).resolve().parents[2]


def _proposal():
    capability, intent = load_powered_fixed_wing_mission_profiles(ROOT / "verification/powered_fixed_wing_mission_profiles.yaml")["x8-cruise"]
    return compile_powered_fixed_wing_racetrack(capability, intent, binding_id="x8-candidate", fidelity="pseudo_6dof_kinematic_bridge")


def test_candidate_without_execution_stays_pending() -> None:
    assessment = assess_mission_promotion(_proposal(), None)
    assert assessment.status == "candidate_pending_execution"
    assert not assessment.checks["execution_artifact_present"]
    ####


def test_matching_truth_evaluated_execution_is_promotion_eligible() -> None:
    proposal = _proposal()
    assessment = assess_mission_promotion(
        proposal,
        {
            "mission_proposal_fingerprint": mission_proposal_fingerprint(proposal),
            "binding_id": proposal.route.binding_id,
            "fidelity": proposal.route.fidelity,
            "evaluation": {"mission_pass": True, "required_passed": 4, "required_objectives": 4, "hard_gates_passed": True},
            "runtime": {
                "numerical_valid": True,
                "semantic_gate_convergence_pass": True,
                "batch_step_parity_pass": True,
            },
        },
    )
    assert assessment.status == "promotion_eligible"
    assert all(assessment.checks.values())
    ####


def test_stale_or_incomplete_execution_cannot_promote_a_route() -> None:
    proposal = _proposal()
    assessment = assess_mission_promotion(
        proposal,
        {
            "mission_proposal_fingerprint": "stale",
            "binding_id": proposal.route.binding_id,
            "fidelity": proposal.route.fidelity,
            "evaluation": {"mission_pass": True, "required_passed": 3, "required_objectives": 4, "hard_gates_passed": True},
            "runtime": {
                "numerical_valid": True,
                "semantic_gate_convergence_pass": True,
                "batch_step_parity_pass": True,
            },
        },
    )
    assert assessment.status == "candidate_execution_rejected"
    assert not assessment.checks["proposal_fingerprint_matches"]
    assert not assessment.checks["truth_objectives_pass"]
    ####


def test_execution_without_numerical_replay_evidence_promotes_only_the_candidate_route() -> None:
    proposal = _proposal()
    assessment = assess_mission_promotion(
        proposal,
        {
            "mission_proposal_fingerprint": mission_proposal_fingerprint(proposal),
            "binding_id": proposal.route.binding_id,
            "fidelity": proposal.route.fidelity,
            "evaluation": {"mission_pass": True, "required_passed": 4, "required_objectives": 4, "hard_gates_passed": True},
            "runtime": {
                "numerical_valid": True,
                "semantic_gate_convergence_pass": False,
                "batch_step_parity_pass": False,
            },
        },
    )
    assert assessment.status == "promotion_eligible"
    assert not assessment.checks["semantic_gate_convergence_pass"]
    assert not assessment.checks["batch_step_parity_pass"]
    assert "additional qualification evidence pending: semantic_gate_convergence_pass" in assessment.findings
    assert "additional qualification evidence pending: batch_step_parity_pass" in assessment.findings
    ####


def test_proposal_fingerprint_binds_the_resolved_fidelity_and_route_identity() -> None:
    capability, intent = load_powered_fixed_wing_mission_profiles(
        ROOT / "verification/powered_fixed_wing_mission_profiles.yaml"
    )["x8-cruise"]
    point_mass = compile_powered_fixed_wing_racetrack(
        capability,
        intent,
        binding_id="x8-point-mass-candidate",
        fidelity="point_mass_3dof",
    )
    pseudo_six = compile_powered_fixed_wing_racetrack(
        capability,
        intent,
        binding_id="x8-pseudo-six-candidate",
        fidelity="pseudo_6dof_kinematic_bridge",
    )

    assert mission_proposal_fingerprint(point_mass) != mission_proposal_fingerprint(pseudo_six)
    ####
