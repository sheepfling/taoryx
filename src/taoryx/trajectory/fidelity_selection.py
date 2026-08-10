"""Provider-neutral validated fidelity selection for reference families."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal, cast

from ..fidelity_contracts import (
    CANONICAL_FIDELITY_TIERS,
    FIDELITY_TIER_RANK,
    FidelityTier,
    LegacyFidelityTier,
    canonical_tier_for_runtime,
)
from ..fidelity_lowering import LoweringCandidate, select_canonical_lowering

if TYPE_CHECKING:
    from .reference_families import ReferenceFidelityProfile

FallbackPolicy = Literal["exact_only", "validated_lower_only"]
FidelityName = LegacyFidelityTier
QUALIFIED_EVIDENCE_STATUSES = frozenset(
    {
        "equivalence_passed",
        "multi_fidelity_qualified",
        "qualified",
        "runtime_replay_qualification_passed",
    }
)
_ORDER: tuple[FidelityName, ...] = (
    "point_mass_3dof",
    "pseudo_6dof",
    "rigid_body_6dof",
)


@dataclass(frozen=True, slots=True)
class FidelitySelectionRequest:
    """Requested tier and explicit lowering policy."""

    requested: FidelityName
    fallback_policy: FallbackPolicy = "exact_only"
    minimum_acceptable: FidelityName = "point_mass_3dof"

    def __post_init__(self) -> None:
        if _ORDER.index(self.minimum_acceptable) > _ORDER.index(self.requested):
            raise ValueError("minimum_acceptable cannot be higher than requested fidelity")
        ####
    ####


@dataclass(frozen=True, slots=True)
class FidelitySelectionResult:
    """Auditable selected tier or structured rejection."""

    requested: FidelityName
    selected: FidelityName | None
    fallback_policy: FallbackPolicy
    minimum_acceptable: FidelityName
    reason: str
    considered: tuple[dict[str, object], ...]
    profile_id: str | None = None
    reduction_artifact: str | None = None

    @property
    def accepted(self) -> bool:
        """Return whether a validated executable tier was selected."""

        return self.selected is not None
        ####
    ####

    def as_dict(self) -> dict[str, object]:
        """Return a stable resolved-case record."""

        return {
            "requested": self.requested,
            "selected": self.selected,
            "fallback_policy": self.fallback_policy,
            "minimum_acceptable": self.minimum_acceptable,
            "reason": self.reason,
            "considered": list(self.considered),
            "profile_id": self.profile_id,
            "reduction_artifact": self.reduction_artifact,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class CanonicalFidelitySelectionRequest:
    """Requested canonical tier and explicit lowering policy."""

    requested: FidelityTier
    fallback_policy: FallbackPolicy = "exact_only"
    minimum_acceptable: FidelityTier = "point_mass_3dof"

    def __post_init__(self) -> None:
        if FIDELITY_TIER_RANK[self.minimum_acceptable] > FIDELITY_TIER_RANK[self.requested]:
            raise ValueError("minimum_acceptable cannot be higher than requested fidelity")
        ####
    ####


@dataclass(frozen=True, slots=True)
class CanonicalFidelitySelectionResult:
    """Auditable selection result for the four-tier fidelity contract."""

    requested: FidelityTier
    selected: FidelityTier | None
    fallback_policy: FallbackPolicy
    minimum_acceptable: FidelityTier
    reason: str
    considered: tuple[dict[str, object], ...]
    profile_id: str | None = None
    reduction_artifact: str | None = None

    @property
    def accepted(self) -> bool:
        """Return whether a canonical executable tier was selected."""

        return self.selected is not None
        ####

    def as_dict(self) -> dict[str, object]:
        """Return a stable resolved-case record."""

        return {
            "requested": self.requested,
            "selected": self.selected,
            "fallback_policy": self.fallback_policy,
            "minimum_acceptable": self.minimum_acceptable,
            "reason": self.reason,
            "considered": list(self.considered),
            "profile_id": self.profile_id,
            "reduction_artifact": self.reduction_artifact,
        }
        ####
    ####


def _canonical_profile_tier(profile: ReferenceFidelityProfile) -> FidelityTier | None:
    """Return a profile's explicit tier, or ``None`` for an ambiguous legacy profile."""

    try:
        return canonical_tier_for_runtime(
            profile.runtime_fidelity,
            control_realization=profile.control_realization,
        )
    except ValueError:
        return None
    ####


def select_validated_canonical_fidelity(
    profiles: Sequence[ReferenceFidelityProfile],
    request: CanonicalFidelitySelectionRequest,
    evidence: Mapping[str, Mapping[str, object]],
) -> CanonicalFidelitySelectionResult:
    """Select a validated tier without collapsing direct and surface control paths.

    Legacy rigid-body profiles remain consumable by the old selector, but they
    are intentionally ineligible here until their control realization is
    declared.  This makes automatic lowering fail closed at the claim boundary.
    """

    profiles_by_tier: dict[FidelityTier, list[ReferenceFidelityProfile]] = {}
    for profile in profiles:
        tier = _canonical_profile_tier(profile)
        if tier is None:
            continue
        profiles_by_tier.setdefault(tier, []).append(profile)
    candidates: dict[FidelityTier, LoweringCandidate] = {}
    metadata: dict[FidelityTier, tuple[ReferenceFidelityProfile | None, bool, int]] = {}
    for tier in CANONICAL_FIDELITY_TIERS:
        tier_profiles = profiles_by_tier.get(tier, [])
        eligible_profiles = [
            profile
            for profile in tier_profiles
            if str(evidence.get(profile.profile_id, {}).get("status", profile.status)) in QUALIFIED_EVIDENCE_STATUSES
        ]
        ambiguous = len(eligible_profiles) > 1
        selected_profile: ReferenceFidelityProfile | None = (
            eligible_profiles[0] if len(eligible_profiles) == 1 else (tier_profiles[0] if tier_profiles else None)
        )
        candidate_profile_id = None if ambiguous or selected_profile is None else selected_profile.profile_id
        candidates[tier] = LoweringCandidate(tier, candidate_profile_id)
        metadata[tier] = (selected_profile, ambiguous, len(tier_profiles))

    decision = select_canonical_lowering(
        candidates,
        request.requested,
        evidence,
        allow_lowering=request.fallback_policy == "validated_lower_only",
        minimum_acceptable=request.minimum_acceptable,
    )
    considered: list[dict[str, object]] = []
    for item in decision.considered:
        tier = cast(FidelityTier, item["fidelity"])
        selected_profile, ambiguous, candidate_count = metadata[tier]
        record = dict(evidence.get(selected_profile.profile_id, {})) if selected_profile is not None else {}
        considered.append(
            {
                **item,
                "profile_status": selected_profile.status if selected_profile is not None else "missing",
                "evidence_status": str(record.get("status", selected_profile.status if selected_profile is not None else "missing")),
                "eligible": item["status"] == "eligible",
                "candidate_count": candidate_count,
                "ambiguous": ambiguous,
            }
        )
    selected_profile_id = candidates[decision.selected].profile_id if decision.selected is not None else None
    selected_record = dict(evidence.get(selected_profile_id, {})) if selected_profile_id is not None else {}
    return CanonicalFidelitySelectionResult(
        request.requested,
        decision.selected,
        request.fallback_policy,
        request.minimum_acceptable,
        (
            "exact_requested_fidelity"
            if decision.selected == request.requested
            else "validated_lower_fidelity_selected"
            if decision.selected is not None
            else "no_validated_fidelity_in_requested_range"
        ),
        tuple(considered),
        selected_profile_id,
        str(selected_record["artifact"]) if "artifact" in selected_record else None,
    )
    ####


def select_validated_fidelity(
    profiles: Sequence[ReferenceFidelityProfile],
    request: FidelitySelectionRequest,
    evidence: Mapping[str, Mapping[str, object]],
) -> FidelitySelectionResult:
    """Select the highest eligible tier without treating plans as evidence.

    ``evidence`` is keyed by profile ID and must contain an explicit
    qualification status.  A profile with a development screen, a pending
    reduction, or no evidence is never eligible for automatic lowering.
    """

    profile_by_fidelity = {profile.runtime_fidelity: profile for profile in profiles}
    start = _ORDER.index(request.requested)
    stop = _ORDER.index(request.minimum_acceptable)
    candidates = tuple(_ORDER[index] for index in range(start, stop - 1, -1)) if start >= stop else (request.requested,)
    considered: list[dict[str, object]] = []
    for fidelity in candidates:
        profile = profile_by_fidelity.get(fidelity)
        record = dict(evidence.get(profile.profile_id, {})) if profile is not None else {}
        status = str(record.get("status", profile.status if profile is not None else "missing"))
        eligible = profile is not None and status in QUALIFIED_EVIDENCE_STATUSES
        considered.append(
            {
                "fidelity": fidelity,
                "profile_id": profile.profile_id if profile is not None else None,
                "profile_status": profile.status if profile is not None else "missing",
                "evidence_status": status,
                "eligible": eligible,
            }
        )
        if eligible and profile is not None:
            artifact = record.get("artifact")
            return FidelitySelectionResult(
                request.requested,
                fidelity,
                request.fallback_policy,
                request.minimum_acceptable,
                "exact_requested_fidelity" if fidelity == request.requested else "validated_lower_fidelity_selected",
                tuple(considered),
                profile.profile_id,
                str(artifact) if artifact is not None else None,
            )
        if request.fallback_policy == "exact_only":
            break
    return FidelitySelectionResult(
        request.requested,
        None,
        request.fallback_policy,
        request.minimum_acceptable,
        "no_validated_fidelity_in_requested_range",
        tuple(considered),
    )
    ####


__all__ = [
    "CanonicalFidelitySelectionRequest",
    "CanonicalFidelitySelectionResult",
    "FidelitySelectionRequest",
    "FidelitySelectionResult",
    "select_validated_canonical_fidelity",
    "select_validated_fidelity",
]
