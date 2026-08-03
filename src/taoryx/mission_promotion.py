"""Fail-closed promotion contract for capability-scaled mission proposals."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from typing import Any, Mapping

from taoryx.powered_fixed_wing_mission_compiler import CapabilityScaledRacetrack


@dataclass(frozen=True, slots=True)
class MissionPromotionAssessment:
    """Independent decision about whether a candidate route may be promoted."""

    proposal_fingerprint: str
    candidate_binding_id: str
    fidelity: str
    status: str
    checks: Mapping[str, bool]
    findings: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        """Return a portable promotion record."""

        return {
            "schema_version": "taoryx.mission-promotion/v1",
            "proposal_fingerprint": self.proposal_fingerprint,
            "candidate_binding_id": self.candidate_binding_id,
            "fidelity": self.fidelity,
            "status": self.status,
            "checks": dict(self.checks),
            "findings": list(self.findings),
            "claim_boundary": (
                "Promotion eligibility only. This record does not replace a family evidence tier, "
                "robustness qualification, or physical-effector claim."
            ),
        }
        ####
    ####


def mission_proposal_fingerprint(proposal: CapabilityScaledRacetrack) -> str:
    """Hash the complete resolved proposal used by a candidate execution."""

    canonical = json.dumps(proposal.manifest(), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()
    ####


def assess_mission_promotion(
    proposal: CapabilityScaledRacetrack,
    execution: Mapping[str, Any] | None,
) -> MissionPromotionAssessment:
    """Require independent execution evidence before route promotion.

    An execution artifact must bind itself to the exact proposal fingerprint;
    otherwise a passing run could be attributed to changed geometry.  Its truth
    evaluator, hard gates, and numerical status must all pass.  Candidate
    promotion establishes only that this exact geometry has an executable
    nominal witness.  Step-refinement and batch/step replay are retained as
    explicit evidence checks, but belong to later numerical/qualification
    promotion gates rather than blocking a route from becoming a versioned
    candidate baseline.
    """

    fingerprint = mission_proposal_fingerprint(proposal)
    route = proposal.route
    base_checks = {
        "proposal_is_capability_feasible": proposal.status == "capability_feasible",
        "execution_artifact_present": execution is not None,
        "proposal_fingerprint_matches": False,
        "candidate_binding_matches": False,
        "fidelity_matches": False,
        "truth_objectives_pass": False,
        "hard_gates_pass": False,
        "numerical_valid": False,
        "semantic_gate_convergence_pass": False,
        "batch_step_parity_pass": False,
    }
    findings: list[str] = []
    if execution is None:
        findings.append("candidate has no executable truth-evaluation artifact")
        return MissionPromotionAssessment(fingerprint, route.binding_id, route.fidelity, "candidate_pending_execution", base_checks, tuple(findings))

    checks = dict(base_checks)
    checks["proposal_fingerprint_matches"] = execution.get("mission_proposal_fingerprint") == fingerprint
    checks["candidate_binding_matches"] = execution.get("binding_id") == route.binding_id
    checks["fidelity_matches"] = execution.get("fidelity") == route.fidelity
    evaluation = execution.get("evaluation")
    runtime = execution.get("runtime")
    if isinstance(evaluation, Mapping):
        checks["truth_objectives_pass"] = bool(evaluation.get("mission_pass")) and (
            evaluation.get("required_passed") == evaluation.get("required_objectives")
        )
        checks["hard_gates_pass"] = bool(evaluation.get("hard_gates_passed"))
    else:
        findings.append("execution artifact has no independent evaluation mapping")
    if isinstance(runtime, Mapping):
        checks["numerical_valid"] = bool(runtime.get("numerical_valid"))
        checks["semantic_gate_convergence_pass"] = bool(runtime.get("semantic_gate_convergence_pass"))
        checks["batch_step_parity_pass"] = bool(runtime.get("batch_step_parity_pass"))
    else:
        findings.append("execution artifact has no runtime/numerical mapping")
    required_checks = (
        "proposal_is_capability_feasible",
        "execution_artifact_present",
        "proposal_fingerprint_matches",
        "candidate_binding_matches",
        "fidelity_matches",
        "truth_objectives_pass",
        "hard_gates_pass",
        "numerical_valid",
    )
    supplemental_checks = (
        "semantic_gate_convergence_pass",
        "batch_step_parity_pass",
    )
    for name in required_checks:
        if not checks[name]:
            findings.append(f"promotion requirement failed: {name}")
    for name in supplemental_checks:
        if not checks[name]:
            findings.append(f"additional qualification evidence pending: {name}")
    status = "promotion_eligible" if all(checks[name] for name in required_checks) else "candidate_execution_rejected"
    return MissionPromotionAssessment(fingerprint, route.binding_id, route.fidelity, status, checks, tuple(findings))
    ####


__all__ = ["MissionPromotionAssessment", "assess_mission_promotion", "mission_proposal_fingerprint"]
####
