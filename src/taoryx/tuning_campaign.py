"""Reusable operating-point controller-design campaigns.

Vehicle integration should not begin with an unbounded sequence of manual gain
changes.  A family adapter supplies the nonlinear plant, trim definition, and
effectors or lower-tier semantic controls.  This module supplies the same
ordered campaign for every controlled topology:

``trim -> derivative consistency -> authority -> scaled candidates``.

The result is intentionally a *design-screen* artifact.  A candidate-ready
node has a trimmed, locally controllable, derivative-consistent linear model
and a bounded LQR candidate.  It does not prove nonlinear mission behavior or
physical effector realization; those remain separate, tier-appropriate gates.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Literal

import numpy as np

from .family_adapter import AdapterCapabilityError, AdapterOperation, StandardFamilyAdapter
from .generic_tuning import (
    AuthorityPreflightReport,
    GenericLqrProfile,
    GenericLqrReport,
    LinearAuthorityRequirement,
    NormalizedLqrProfileGrid,
    linear_authority_preflight,
    tune_lqi_profiles,
    tune_lqr_profiles,
)
from .trim import TrimResult

TuningNodeStatus = Literal[
    "candidate_ready",
    "operation_unavailable",
    "trim_failed",
    "linearization_failed",
    "derivative_inconsistent",
    "design_subsystem_coupled",
    "authority_blocked",
    "candidate_rejected",
]
TuningCampaignStatus = Literal["candidate_ready", "blocked"]


@dataclass(frozen=True, slots=True)
class TuningCampaignNode:
    """One declared operating point in a family-independent design campaign.

    The family declares target/initial values and the physically meaningful
    scaling contract.  The campaign owns the numerical sequence and reports
    why it stopped.  This preserves vehicle-specific physics without allowing
    an integration to skip structural diagnostics and go directly to gains.
    """

    node_id: str
    trim_target: Mapping[str, float]
    trim_initial_guess: Mapping[str, float]
    state_scales: tuple[float, ...]
    control_scales: tuple[float, ...]
    authority_requirement: LinearAuthorityRequirement
    profiles: tuple[GenericLqrProfile, ...] = ()
    profile_grid: NormalizedLqrProfileGrid | None = None
    design_state_names: tuple[str, ...] | None = None
    design_control_names: tuple[str, ...] | None = None
    controller_method: Literal["lqr", "lqi"] = "lqr"
    integral_output_names: tuple[str, ...] = ()
    integral_q_diagonal: tuple[float, ...] = ()
    maximum_omitted_state_coupling: float = 1.0e-8
    linearization_options: Mapping[str, float | str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.node_id.strip():
            raise ValueError("tuning campaign nodes require a non-empty ID")
        if not self.state_scales or not self.control_scales:
            raise ValueError("tuning campaign nodes require state and control scales")
        if bool(self.profiles) == (self.profile_grid is not None):
            raise ValueError("tuning campaign nodes require exactly one of explicit profiles or a normalized profile grid")
        if self.design_state_names is not None and (
            not self.design_state_names or len(set(self.design_state_names)) != len(self.design_state_names)
        ):
            raise ValueError("campaign design-state names must be non-empty and unique when supplied")
        if self.design_control_names is not None and (
            not self.design_control_names or len(set(self.design_control_names)) != len(self.design_control_names)
        ):
            raise ValueError("campaign design-control names must be non-empty and unique when supplied")
        if self.controller_method == "lqi":
            if not self.integral_output_names or len(set(self.integral_output_names)) != len(self.integral_output_names):
                raise ValueError("LQI campaign nodes require non-empty unique integral outputs")
            if self.design_state_names is not None and set(self.integral_output_names) - set(self.design_state_names):
                raise ValueError("LQI campaign integral outputs must identify design states")
            if len(self.integral_q_diagonal) != len(self.integral_output_names):
                raise ValueError("LQI campaign integral weights must match integral outputs")
            if any(value <= 0.0 for value in self.integral_q_diagonal):
                raise ValueError("LQI campaign integral weights must be positive")
        elif self.integral_output_names or self.integral_q_diagonal:
            raise ValueError("LQR campaign nodes cannot declare LQI integral outputs or weights")
        if self.maximum_omitted_state_coupling < 0.0:
            raise ValueError("maximum omitted-state coupling must be nonnegative")
        ####

    def as_dict(self) -> dict[str, object]:
        """Return reproducible input data without numerical results."""

        return {
            "node_id": self.node_id,
            "trim_target": dict(self.trim_target),
            "trim_initial_guess": dict(self.trim_initial_guess),
            "state_scales": list(self.state_scales),
            "control_scales": list(self.control_scales),
            "profiles": [
                {
                    "id": profile.id,
                    "q_diagonal": list(profile.q_diagonal),
                    "r_diagonal": list(profile.r_diagonal),
                }
                for profile in self.profiles
            ],
            "profile_grid": self.profile_grid.as_dict() if self.profile_grid is not None else None,
            "authority_requirement": {
                "id": self.authority_requirement.id,
                "required_state_names": list(self.authority_requirement.required_state_names),
                "minimum_controllability_rank": self.authority_requirement.minimum_controllability_rank,
                "maximum_uncontrolled_fraction": self.authority_requirement.maximum_uncontrolled_fraction,
                "maximum_controllability_condition": self.authority_requirement.maximum_controllability_condition,
            },
            "design_state_names": list(self.design_state_names) if self.design_state_names is not None else None,
            "design_control_names": list(self.design_control_names) if self.design_control_names is not None else None,
            "controller_method": self.controller_method,
            "integral_output_names": list(self.integral_output_names),
            "integral_q_diagonal": list(self.integral_q_diagonal),
            "maximum_omitted_state_coupling": self.maximum_omitted_state_coupling,
            "linearization_options": dict(self.linearization_options),
        }
        ####

    def resolved_profiles(self, state_count: int, control_count: int) -> tuple[GenericLqrProfile, ...]:
        """Return explicit candidates or generate the declared normalized lattice."""

        return self.profiles if self.profile_grid is None else self.profile_grid.profiles(state_count, control_count)
        ####
    ####

@dataclass(frozen=True, slots=True)
class TuningCampaign:
    """A named family/tier campaign composed from declared operating points."""

    campaign_id: str
    family_id: str
    tier: str
    strategy_id: str
    nodes: tuple[TuningCampaignNode, ...]

    def __post_init__(self) -> None:
        if not all(value.strip() for value in (self.campaign_id, self.family_id, self.tier, self.strategy_id)):
            raise ValueError("tuning campaigns require non-empty identity fields")
        if not self.nodes:
            raise ValueError("tuning campaigns require at least one operating point")
        ids = tuple(node.node_id for node in self.nodes)
        if len(set(ids)) != len(ids):
            raise ValueError("tuning campaign operating-point IDs must be unique")
        ####

    def as_dict(self) -> dict[str, object]:
        """Return the immutable campaign contract."""

        return {
            "campaign_id": self.campaign_id,
            "family_id": self.family_id,
            "tier": self.tier,
            "strategy_id": self.strategy_id,
            "nodes": [node.as_dict() for node in self.nodes],
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class TuningCampaignNodeResult:
    """One stop-at-first-blocker result from an operating-point campaign."""

    node_id: str
    status: TuningNodeStatus
    message: str
    blockers: tuple[str, ...] = ()
    trim: TrimResult | None = None
    derivative_consistent: bool | None = None
    derivative_metrics: Mapping[str, float] = field(default_factory=dict)
    authority_preflight: AuthorityPreflightReport | None = None
    lqr: GenericLqrReport | None = None

    def as_dict(self) -> dict[str, object]:
        """Return a self-contained tuning-node audit record."""

        return {
            "node_id": self.node_id,
            "status": self.status,
            "message": self.message,
            "blockers": list(self.blockers),
            "trim": self.trim.as_dict() if self.trim is not None else None,
            "derivative_consistent": self.derivative_consistent,
            "derivative_metrics": dict(self.derivative_metrics),
            "authority_preflight": self.authority_preflight.as_dict() if self.authority_preflight is not None else None,
            "lqr": self.lqr.as_dict() if self.lqr is not None else None,
        }
        ####
    ####

@dataclass(frozen=True, slots=True)
class TuningCampaignReport:
    """Complete candidate-design screen for one family/tier campaign."""

    campaign: TuningCampaign
    status: TuningCampaignStatus
    nodes: tuple[TuningCampaignNodeResult, ...]
    claim_boundary: str = (
        "Candidate-design evidence only: passing nodes have trim, derivative, "
        "and authority evidence plus an LQR candidate, not nonlinear mission "
        "or physical-effector qualification."
    )

    def as_dict(self) -> dict[str, object]:
        """Return the reusable campaign evidence artifact."""

        return {
            "schema": "taoryx.tuning-campaign/v1alpha1",
            "status": self.status,
            "campaign": self.campaign.as_dict(),
            "nodes": [node.as_dict() for node in self.nodes],
            "claim_boundary": self.claim_boundary,
        }
        ####
    ####


def run_tuning_campaign(
    adapter: StandardFamilyAdapter,
    campaign: TuningCampaign,
) -> TuningCampaignReport:
    """Run the common no-manual-gain-search workflow for one family/tier.

    Every node stops at the first prerequisite that fails.  Consequently a
    failed trim, inconsistent finite-difference derivative, or unreachable
    declared state cannot be mislabeled as a controller-tuning problem.
    """

    descriptor = adapter.describe()
    if descriptor.family_id != campaign.family_id:
        raise ValueError(
            f"campaign family {campaign.family_id!r} does not match adapter family {descriptor.family_id!r}"
        )
    if descriptor.tier != campaign.tier:
        raise ValueError(f"campaign tier {campaign.tier!r} does not match adapter tier {descriptor.tier!r}")

    capability_report = adapter.capability_report()
    required_operations: tuple[AdapterOperation, ...] = ("trim", "linearize")
    unavailable = tuple(
        operation
        for operation in required_operations
        if not capability_report.capability(operation).usable
    )
    if unavailable:
        message = "campaign requires unavailable adapter operations: " + ", ".join(unavailable)
        results = tuple(
            TuningCampaignNodeResult(
                node.node_id,
                "operation_unavailable",
                message,
                tuple(f"operation_unavailable:{operation}" for operation in unavailable),
            )
            for node in campaign.nodes
        )
        return TuningCampaignReport(campaign, "blocked", results)

    results = tuple(_run_node(adapter, campaign, node) for node in campaign.nodes)
    status: TuningCampaignStatus = "candidate_ready" if all(node.status == "candidate_ready" for node in results) else "blocked"
    return TuningCampaignReport(campaign, status, results)
    ####


def _run_node(
    adapter: StandardFamilyAdapter,
    campaign: TuningCampaign,
    node: TuningCampaignNode,
) -> TuningCampaignNodeResult:
    """Run one declared node without recovering past a structural failure."""

    try:
        trim = adapter.trim(node.trim_target, node.trim_initial_guess)
    except (AdapterCapabilityError, KeyError, TypeError, ValueError, RuntimeError) as error:
        return TuningCampaignNodeResult(
            node.node_id,
            "trim_failed",
            f"trim could not be evaluated: {error}",
            ("trim_evaluation_failed",),
        )
    if not trim.success:
        return TuningCampaignNodeResult(
            node.node_id,
            "trim_failed",
            trim.message,
            ("trim_not_converged",),
            trim=trim,
        )

    try:
        linearization = adapter.linearize(trim, node.linearization_options)
    except (AdapterCapabilityError, KeyError, TypeError, ValueError, RuntimeError) as error:
        return TuningCampaignNodeResult(
            node.node_id,
            "linearization_failed",
            f"linearization could not be evaluated: {error}",
            ("linearization_evaluation_failed",),
            trim=trim,
        )

    provenance = linearization.provenance
    derivative_metrics = {
        "maximum_relative_difference": provenance.maximum_relative_difference,
        "maximum_absolute_difference": provenance.maximum_absolute_difference,
        "state_step": provenance.state_step,
        "control_step": provenance.control_step,
    }
    if not provenance.derivative_consistent:
        return TuningCampaignNodeResult(
            node.node_id,
            "derivative_inconsistent",
            "finite-difference derivatives changed beyond the declared comparison tolerance",
            ("derivative_consistency_failed",),
            trim=trim,
            derivative_consistent=False,
            derivative_metrics=derivative_metrics,
        )

    try:
        design_state_names, design_control_names, design_a_matrix, design_b_matrix, omitted_coupling = _project_design_model(
            node,
            state_names=linearization.primary.state_names,
            control_names=linearization.primary.control_names,
            a_matrix=linearization.primary.a_matrix,
            b_matrix=linearization.primary.b_matrix,
        )
    except ValueError as error:
        return TuningCampaignNodeResult(
            node.node_id,
            "design_subsystem_coupled",
            str(error),
            ("design_subsystem_not_closed",),
            trim=trim,
            derivative_consistent=True,
            derivative_metrics=derivative_metrics,
        )
    derivative_metrics["maximum_omitted_state_coupling"] = omitted_coupling

    try:
        authority = linear_authority_preflight(
            node.authority_requirement,
            state_names=design_state_names,
            a_matrix=design_a_matrix,
            b_matrix=design_b_matrix,
        )
    except (TypeError, ValueError, RuntimeError) as error:
        return TuningCampaignNodeResult(
            node.node_id,
            "authority_blocked",
            f"authority preflight could not be evaluated: {error}",
            ("authority_preflight_invalid",),
            trim=trim,
            derivative_consistent=True,
            derivative_metrics=derivative_metrics,
        )
    if authority.status != "passed":
        return TuningCampaignNodeResult(
            node.node_id,
            "authority_blocked",
            authority.reason,
            authority.blockers,
            trim=trim,
            derivative_consistent=True,
            derivative_metrics=derivative_metrics,
            authority_preflight=authority,
        )

    try:
        profiles = node.resolved_profiles(len(design_state_names), len(design_control_names))
        report_args = {
            "state_names": design_state_names,
            "control_names": design_control_names,
            "state_scales": node.state_scales,
            "control_scales": node.control_scales,
            "profiles": profiles,
            "design_source": (
                f"{campaign.campaign_id}:{node.node_id}:"
                f"{provenance.nonlinear_plant_id}@{provenance.nonlinear_plant_revision}"
            ),
        }
        report = (
            tune_lqi_profiles(
                campaign.family_id,
                design_a_matrix,
                design_b_matrix,
                output_names=node.integral_output_names,
                integral_q_diagonal=node.integral_q_diagonal,
                **report_args,
            )
            if node.controller_method == "lqi"
            else tune_lqr_profiles(campaign.family_id, design_a_matrix, design_b_matrix, **report_args)
        )
    except (TypeError, ValueError, RuntimeError) as error:
        return TuningCampaignNodeResult(
            node.node_id,
            "candidate_rejected",
            f"candidate synthesis could not be evaluated: {error}",
            ("candidate_synthesis_failed",),
            trim=trim,
            derivative_consistent=True,
            derivative_metrics=derivative_metrics,
            authority_preflight=authority,
        )
    if report.best is None:
        blockers = tuple(
            sorted(
                {
                    violation
                    for candidate in report.candidates
                    for violation in candidate.violations
                }
            )
        ) or ("no_safe_candidate",)
        return TuningCampaignNodeResult(
            node.node_id,
            "candidate_rejected",
            f"no bounded {node.controller_method.upper()} candidate passed the declared design screen",
            blockers,
            trim=trim,
            derivative_consistent=True,
            derivative_metrics=derivative_metrics,
            authority_preflight=authority,
            lqr=report,
        )
    return TuningCampaignNodeResult(
        node.node_id,
        "candidate_ready",
        f"trim, derivative consistency, authority preflight, and a bounded {node.controller_method.upper()} candidate passed",
        trim=trim,
        derivative_consistent=True,
        derivative_metrics=derivative_metrics,
        authority_preflight=authority,
        lqr=report,
    )
    ####


def _project_design_model(
    node: TuningCampaignNode,
    *,
    state_names: Sequence[str],
    control_names: Sequence[str],
    a_matrix: np.ndarray,
    b_matrix: np.ndarray,
) -> tuple[tuple[str, ...], tuple[str, ...], tuple[tuple[float, ...], ...], tuple[tuple[float, ...], ...], float]:
    """Project a declared nested-loop subsystem while checking closure.

    A smaller inner-loop LQR is legitimate only when the omitted parent-state
    coordinates do not materially drive its selected derivatives at the
    linearization point.  This prevents a developer from making an awkward
    full model look controllable by silently discarding a coupled state.
    """

    full_states = tuple(state_names)
    full_controls = tuple(control_names)
    design_states = node.design_state_names or full_states
    design_controls = node.design_control_names or full_controls
    missing_states = tuple(name for name in design_states if name not in full_states)
    missing_controls = tuple(name for name in design_controls if name not in full_controls)
    if missing_states or missing_controls:
        details = []
        if missing_states:
            details.append("unknown design states: " + ", ".join(missing_states))
        if missing_controls:
            details.append("unknown design controls: " + ", ".join(missing_controls))
        raise ValueError("; ".join(details))

    state_indices = tuple(full_states.index(name) for name in design_states)
    control_indices = tuple(full_controls.index(name) for name in design_controls)
    omitted_indices = tuple(index for index, name in enumerate(full_states) if name not in design_states)
    matrix_a = np.asarray(a_matrix, dtype=float)
    matrix_b = np.asarray(b_matrix, dtype=float)
    if omitted_indices:
        omitted_coupling = float(np.max(np.abs(matrix_a[np.ix_(state_indices, omitted_indices)])))
    else:
        omitted_coupling = 0.0
    if omitted_coupling > node.maximum_omitted_state_coupling:
        raise ValueError(
            "declared nested-loop subsystem is not closed: omitted-state coupling "
            f"{omitted_coupling:.6g} exceeds {node.maximum_omitted_state_coupling:.6g}"
        )
    projected_a = matrix_a[np.ix_(state_indices, state_indices)]
    projected_b = matrix_b[np.ix_(state_indices, control_indices)]
    return (
        tuple(design_states),
        tuple(design_controls),
        tuple(tuple(float(value) for value in row) for row in projected_a),
        tuple(tuple(float(value) for value in row) for row in projected_b),
        omitted_coupling,
    )
    ####


__all__ = [
    "TuningCampaign",
    "TuningCampaignNode",
    "TuningCampaignNodeResult",
    "TuningCampaignReport",
    "TuningCampaignStatus",
    "TuningNodeStatus",
    "run_tuning_campaign",
]
