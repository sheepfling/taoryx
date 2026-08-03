"""Canonical horizontal fidelity and control-realization contracts.

This module is the single vocabulary shared by readiness, lowering, controller
provenance, and family integration code.  Family-specific adapters still own
the equations and source data; this module owns the meaning and ordering of
the advertised realization tiers.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Literal, cast

FidelityTier = Literal[
    "point_mass_3dof",
    "pseudo_6dof",
    "rigid_body_6dof_direct_wrench",
    "rigid_body_6dof_surface_allocated",
]
RuntimeFidelity = Literal[
    "point_mass_3dof",
    "pseudo_6dof",
    "rigid_body_6dof",
]
LegacyFidelityTier = Literal[
    FidelityTier,
    "rigid_body_6dof",
]
ControlRealization = Literal[
    "force_model",
    "response_law",
    "direct_wrench",
    "surface_allocated",
    "uncontrolled",
    "unspecified",
]

CANONICAL_FIDELITY_TIERS: tuple[FidelityTier, ...] = (
    "point_mass_3dof",
    "pseudo_6dof",
    "rigid_body_6dof_direct_wrench",
    "rigid_body_6dof_surface_allocated",
)
LEGACY_FIDELITY_ORDER: tuple[str, ...] = (
    "point_mass_3dof",
    "pseudo_6dof",
    "rigid_body_6dof",
)
FIDELITY_TIER_RANK: Mapping[FidelityTier, int] = {
    tier: index for index, tier in enumerate(CANONICAL_FIDELITY_TIERS)
}
RUNTIME_FIDELITY_BY_TIER: Mapping[FidelityTier, RuntimeFidelity] = {
    "point_mass_3dof": "point_mass_3dof",
    "pseudo_6dof": "pseudo_6dof",
    "rigid_body_6dof_direct_wrench": "rigid_body_6dof",
    "rigid_body_6dof_surface_allocated": "rigid_body_6dof",
}
FidelityStatus = Literal[
    "planned",
    "development",
    "nominal_case_pass",
    "debug_comparator_pass",
    "promoted",
    "not_applicable",
    "not_available",
]
QUALIFIED_FIDELITY_STATUSES = frozenset(
    {
        "equivalence_passed",
        "multi_fidelity_qualified",
        "qualified",
        "runtime_replay_qualification_passed",
        "nominal_case_pass",
        "debug_comparator_pass",
        "promoted",
    }
)


def canonicalize_fidelity(
    value: str,
    *,
    control_realization: str | None = None,
) -> FidelityTier:
    """Normalize one tier name and reject ambiguous legacy rigid-body names.

    Existing records used ``rigid_body_6dof`` for both direct-wrench and
    physical-effector runs.  New records must select the explicit tier.  A
    legacy value is accepted only when its control realization disambiguates
    the mapping, so compatibility cannot silently change a claim boundary.
    """

    if value in CANONICAL_FIDELITY_TIERS:
        return cast(FidelityTier, value)
    if value != "rigid_body_6dof":
        raise ValueError(f"unknown fidelity tier: {value!r}")
    if control_realization in {"direct_wrench", "direct_wrench_bridge"}:
        return "rigid_body_6dof_direct_wrench"
    if control_realization in {"surface_allocated", "constrained_effector_allocation", "nonlinear_effector_validation"}:
        return "rigid_body_6dof_surface_allocated"
    raise ValueError(
        "legacy 'rigid_body_6dof' is ambiguous; declare control_realization "
        "as direct_wrench or surface_allocated"
    )
    ####


def canonical_tier_for_runtime(
    value: str,
    *,
    control_realization: str | None = None,
) -> FidelityTier:
    """Resolve a runtime or canonical name to one explicit advertised tier."""

    return canonicalize_fidelity(value, control_realization=control_realization)
    ####


def parent_fidelity(tier: FidelityTier) -> FidelityTier | None:
    """Return the immediately lower semantic tier, if one exists."""

    index = FIDELITY_TIER_RANK[tier]
    return CANONICAL_FIDELITY_TIERS[index - 1] if index else None
    ####


def control_realization_for(tier: FidelityTier) -> ControlRealization:
    """Return the canonical control realization for a fidelity tier."""

    realization_by_tier: Mapping[FidelityTier, ControlRealization] = {
        "point_mass_3dof": "force_model",
        "pseudo_6dof": "response_law",
        "rigid_body_6dof_direct_wrench": "direct_wrench",
        "rigid_body_6dof_surface_allocated": "surface_allocated",
    }
    return realization_by_tier[tier]
    ####


def runtime_fidelity_for(tier: FidelityTier) -> RuntimeFidelity:
    """Return the native runtime mode for a canonical advertised tier.

    The two rigid-body tiers intentionally share one native integration mode;
    their control realization is distinguished by ``control_realization_for``.
    Keeping this mapping here prevents readiness and adapter code from
    independently re-encoding the same compatibility relationship.
    """

    return RUNTIME_FIDELITY_BY_TIER[tier]
    ####


__all__ = [
    "CANONICAL_FIDELITY_TIERS",
    "ControlRealization",
    "FIDELITY_TIER_RANK",
    "FidelityStatus",
    "FidelityTier",
    "LEGACY_FIDELITY_ORDER",
    "LegacyFidelityTier",
    "QUALIFIED_FIDELITY_STATUSES",
    "RUNTIME_FIDELITY_BY_TIER",
    "RuntimeFidelity",
    "canonicalize_fidelity",
    "canonical_tier_for_runtime",
    "control_realization_for",
    "parent_fidelity",
    "runtime_fidelity_for",
]
