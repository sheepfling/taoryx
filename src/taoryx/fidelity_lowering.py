"""Shared evidence-based lowering for the canonical four-tier ladder."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Literal

from .fidelity_contracts import (
    CANONICAL_FIDELITY_TIERS,
    FIDELITY_TIER_RANK,
    QUALIFIED_FIDELITY_STATUSES,
    FidelityTier,
)

LoweringStatus = Literal["eligible", "blocked", "unavailable"]


@dataclass(frozen=True, slots=True)
class LoweringCandidate:
    """One canonical tier candidate and its evidence prerequisites."""

    tier: FidelityTier
    profile_id: str | None
    parent_profile_ids: tuple[str, ...] = ()
    prerequisite: str = "qualified profile evidence"
    required_operations: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class LoweringDecision:
    """Auditable result of the shared lowering algorithm."""

    requested: FidelityTier
    selected: FidelityTier | None
    first_blocker: str | None
    considered: tuple[dict[str, object], ...]


def _evidence_status(profile_id: str, evidence: Mapping[str, Mapping[str, object]]) -> str:
    record = evidence.get(profile_id, {})
    return str(record.get("status", "missing"))
    ####


def _qualified(profile_id: str | None, evidence: Mapping[str, Mapping[str, object]]) -> tuple[bool, str]:
    if profile_id is None:
        return False, "no profile was supplied for this fidelity tier"
    status = _evidence_status(profile_id, evidence)
    if status in QUALIFIED_FIDELITY_STATUSES:
        return True, f"evidence status {status!r} for {profile_id!r} is qualified"
    return False, f"no qualified evidence for {profile_id!r} (evidence status {status!r})"
    ####


def select_canonical_lowering(
    candidates: Mapping[FidelityTier, LoweringCandidate],
    requested: FidelityTier,
    evidence: Mapping[str, Mapping[str, object]],
    *,
    allow_lowering: bool,
    minimum_acceptable: FidelityTier = "point_mass_3dof",
    operation_status: Mapping[FidelityTier, Mapping[str, str]] | None = None,
) -> LoweringDecision:
    """Select the highest checked candidate without crossing claim boundaries.

    When ``operation_status`` is supplied, candidates may additionally declare
    ``required_operations``.  Such a candidate is eligible only when every
    required operation is reported as ``pass`` or ``available``.  Existing
    profile-only callers remain unchanged when the optional mapping is absent.
    """

    start = FIDELITY_TIER_RANK[requested]
    stop = FIDELITY_TIER_RANK[minimum_acceptable]
    end = stop if allow_lowering else start
    tiers = tuple(CANONICAL_FIDELITY_TIERS[index] for index in range(start, end - 1, -1))
    considered: list[dict[str, object]] = []
    first_blocker: str | None = None
    for tier in tiers:
        candidate = candidates.get(tier, LoweringCandidate(tier, None))
        own_ok, own_reason = _qualified(candidate.profile_id, evidence)
        reason = own_reason
        parent_ok = True
        for parent_id in candidate.parent_profile_ids:
            parent_ok, parent_reason = _qualified(parent_id, evidence)
            if not parent_ok:
                if tier == "pseudo_6dof":
                    reason = f"qualified pseudo-6DOF profile but parent 3-DOF evidence is missing: {parent_reason}"
                elif tier == "rigid_body_6dof_surface_allocated":
                    reason = f"surface profile is checked but its direct-wrench parent is not: {parent_reason}"
                else:
                    reason = f"qualified {tier} profile but parent 3-DOF evidence is missing: {parent_reason}"
                break
        operation_ok = True
        missing_operations: tuple[str, ...] = ()
        if own_ok and parent_ok and candidate.required_operations:
            statuses = (operation_status or {}).get(tier)
            if statuses is None:
                operation_ok = False
                missing_operations = candidate.required_operations
                reason = "adapter operation status was not supplied for required operations: " + ", ".join(missing_operations)
            else:
                missing_operations = tuple(
                    operation
                    for operation in candidate.required_operations
                    if statuses.get(operation) not in {"pass", "available"}
                )
                if missing_operations:
                    operation_ok = False
                    reason = "required adapter operations are not available: " + ", ".join(missing_operations)
        eligible = own_ok and parent_ok and operation_ok
        status: LoweringStatus = "eligible" if eligible else ("unavailable" if candidate.profile_id is None else "blocked")
        if not eligible and first_blocker is None:
            first_blocker = reason
        considered.append(
            {
                "fidelity": tier,
                "profile_id": candidate.profile_id,
                "status": status,
                "prerequisite": candidate.prerequisite,
                "required_operations": list(candidate.required_operations),
                "missing_operations": list(missing_operations),
                "reason": reason,
            }
        )
        if eligible:
            return LoweringDecision(requested, tier, None, tuple(considered))
    return LoweringDecision(requested, None, first_blocker, tuple(considered))
    ####


__all__ = [
    "LoweringCandidate",
    "LoweringDecision",
    "LoweringStatus",
    "select_canonical_lowering",
]
