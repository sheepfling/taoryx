"""Provider-neutral validated fidelity selection for reference families."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Literal

from .reference_families import ReferenceFidelityProfile

FallbackPolicy = Literal["exact_only", "validated_lower_only"]
FidelityName = Literal["point_mass_3dof", "pseudo_6dof", "rigid_body_6dof"]
QUALIFIED_EVIDENCE_STATUSES = frozenset(
    {
        "equivalence_passed",
        "multi_fidelity_qualified",
        "qualified",
        "runtime_replay_qualification_passed",
    }
)
_ORDER: tuple[FidelityName, ...] = ("point_mass_3dof", "pseudo_6dof", "rigid_body_6dof")


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


__all__ = ["FidelitySelectionRequest", "FidelitySelectionResult", "select_validated_fidelity"]
