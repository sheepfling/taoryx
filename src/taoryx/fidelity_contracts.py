"""Canonical horizontal fidelity and control-realization contracts.

This module is the single vocabulary shared by readiness, lowering, controller
provenance, and family integration code.  Family-specific adapters still own
the equations and source data; this module owns the meaning and ordering of
the advertised realization tiers.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Literal

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
FIDELITY_TIER_RANK: Mapping[FidelityTier, int] = {tier: index for index, tier in enumerate(CANONICAL_FIDELITY_TIERS)}
RUNTIME_FIDELITY_BY_TIER: Mapping[FidelityTier, RuntimeFidelity] = {
    "point_mass_3dof": "point_mass_3dof",
    "pseudo_6dof": "pseudo_6dof",
    "rigid_body_6dof_direct_wrench": "rigid_body_6dof",
    "rigid_body_6dof_surface_allocated": "rigid_body_6dof",
}


@dataclass(frozen=True, slots=True)
class FidelityTierMetadata:
    """Stable explanatory metadata for one canonical fidelity tier."""

    display_name: str
    model_kind: str
    translational_degrees_of_freedom: int
    attitude_representation: str
    control_boundary: str
    description: str

    def as_dict(self) -> dict[str, object]:
        return {
            "display_name": self.display_name,
            "model_kind": self.model_kind,
            "translational_degrees_of_freedom": self.translational_degrees_of_freedom,
            "attitude_representation": self.attitude_representation,
            "control_boundary": self.control_boundary,
            "description": self.description,
        }
        ####
    ####


FIDELITY_TIER_METADATA: Mapping[FidelityTier, FidelityTierMetadata] = {
    "point_mass_3dof": FidelityTierMetadata(
        "Point-mass 3DOF",
        "translational_force_model",
        3,
        "not_dynamically_propagated",
        "force_or_guidance_intent",
        "Translational motion with attitude and physical moment balance omitted.",
    ),
    "pseudo_6dof": FidelityTierMetadata(
        "Pseudo-6DOF response model",
        "translation_plus_attitude_response",
        3,
        "named_response_law_or_declared_rigid_body_reuse",
        "attitude_or_guidance_intent",
        "Translational motion plus a declared attitude-response realization; it is not automatically a physical moment or effector model.",
    ),
    "rigid_body_6dof_direct_wrench": FidelityTierMetadata(
        "Rigid-body 6DOF — direct wrench",
        "newton_euler_rigid_body",
        3,
        "integrated_rigid_body_attitude",
        "generalized_body_force_and_moment",
        "Rigid-body translation and rotation driven through a bounded generalized-wrench boundary without a physical-effector claim.",
    ),
    "rigid_body_6dof_surface_allocated": FidelityTierMetadata(
        "Rigid-body 6DOF — allocated effectors",
        "newton_euler_rigid_body_with_allocation",
        3,
        "integrated_rigid_body_attitude",
        "named_bounded_effectors",
        "Rigid-body translation and rotation with commands realized through declared bounded physical or logical effectors.",
    ),
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
        return value
    if value != "rigid_body_6dof":
        raise ValueError(f"unknown fidelity tier: {value!r}")
    if control_realization in {"direct_wrench", "direct_wrench_bridge"}:
        return "rigid_body_6dof_direct_wrench"
    if control_realization in {"surface_allocated", "constrained_effector_allocation", "nonlinear_effector_validation"}:
        return "rigid_body_6dof_surface_allocated"
    raise ValueError("legacy 'rigid_body_6dof' is ambiguous; declare control_realization as direct_wrench or surface_allocated")
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


def fidelity_tier_metadata(tier: FidelityTier) -> FidelityTierMetadata:
    """Return the common explanatory card for a canonical tier."""

    return FIDELITY_TIER_METADATA[tier]
    ####


__all__ = [
    "CANONICAL_FIDELITY_TIERS",
    "ControlRealization",
    "FIDELITY_TIER_RANK",
    "FIDELITY_TIER_METADATA",
    "FidelityTierMetadata",
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
    "fidelity_tier_metadata",
    "parent_fidelity",
    "runtime_fidelity_for",
]
