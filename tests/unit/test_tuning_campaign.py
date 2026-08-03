"""Tests for the reusable stop-at-first-blocker tuning campaign."""

from __future__ import annotations

from collections.abc import Mapping

from taoryx.family_adapter import StandardFamilyAdapter, descriptor_from_control_plant
from taoryx.generic_tuning import GenericLqrProfile, LinearAuthorityRequirement
from taoryx.trajectory.reduced_control_plant import ReducedOrderControlPlant, declared_equilibrium_trim
from taoryx.trim import TrimResult
from taoryx.tuning_campaign import TuningCampaign, TuningCampaignNode, run_tuning_campaign


def _plant(*, with_yaw: bool) -> ReducedOrderControlPlant:
    state_names = ("roll_rate", "pitch_rate", "yaw_rate")
    control_names = ("roll_command", "pitch_command")

    def derivative(state: Mapping[str, float], controls: Mapping[str, float], environment: Mapping[str, float | str]) -> Mapping[str, float]:
        del environment
        return {
            "roll_rate": -state["roll_rate"] + controls["roll_command"],
            "pitch_rate": -state["pitch_rate"] + controls["pitch_command"],
            "yaw_rate": (
                state["roll_rate"] - state["yaw_rate"]
                if with_yaw
                else -state["yaw_rate"]
            ),
        }

    def trim(target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        state = {name: float(target.get(name, 0.0)) for name in state_names}
        controls = {name: float(initial_guess.get(name, 0.0)) for name in control_names}
        return declared_equilibrium_trim(
            state_names=state_names,
            control_names=control_names,
            state=state,
            controls=controls,
            residuals=derivative(state, controls, {}),
            operating_point={"synthetic": "tuning-campaign"},
        )

    return ReducedOrderControlPlant(
        state_names=state_names,
        control_names=control_names,
        derivative_evaluator=derivative,
        trim_provider=trim,
        nonlinear_plant_id="synthetic.tuning-campaign",
        nonlinear_plant_revision="v1",
        state_units={name: "rad/s" for name in state_names},
        control_units={name: "rad/s2" for name in control_names},
        claim_boundary="synthetic test response law",
    )
    ####


def _adapter(*, with_yaw: bool) -> StandardFamilyAdapter:
    plant = _plant(with_yaw=with_yaw)
    descriptor = descriptor_from_control_plant(
        plant,
        family_id="synthetic_attitude",
        adapter_id="synthetic.attitude.v1",
        physical_family="test",
        tier="pseudo_6dof",
        state_units=plant.state_units,
        control_units=plant.control_units,
    )
    return StandardFamilyAdapter.from_control_plant(descriptor, plant)
    ####


def _campaign(*, required_states: tuple[str, ...]) -> TuningCampaign:
    profile = GenericLqrProfile("standard", (1.0, 1.0, 1.0), (1.0, 1.0))
    return TuningCampaign(
        campaign_id="synthetic-attitude-campaign-v1",
        family_id="synthetic_attitude",
        tier="pseudo_6dof",
        strategy_id="synthetic-attitude.v1",
        nodes=(
            TuningCampaignNode(
                node_id="trim",
                trim_target={},
                trim_initial_guess={},
                state_scales=(1.0, 1.0, 1.0),
                control_scales=(1.0, 1.0),
                profiles=(profile,),
                authority_requirement=LinearAuthorityRequirement("synthetic-attitude-authority", required_states),
            ),
        ),
    )
    ####


def _nested_campaign() -> TuningCampaign:
    profile = GenericLqrProfile("inner-loop", (1.0, 1.0), (1.0, 1.0))
    return TuningCampaign(
        campaign_id="synthetic-nested-campaign-v1",
        family_id="synthetic_attitude",
        tier="pseudo_6dof",
        strategy_id="synthetic-attitude.v1",
        nodes=(
            TuningCampaignNode(
                node_id="inner-roll-pitch",
                trim_target={},
                trim_initial_guess={},
                state_scales=(1.0, 1.0),
                control_scales=(1.0, 1.0),
                profiles=(profile,),
                authority_requirement=LinearAuthorityRequirement("inner-roll-pitch", ("roll_rate", "pitch_rate")),
                design_state_names=("roll_rate", "pitch_rate"),
                design_control_names=("roll_command", "pitch_command"),
            ),
        ),
    )
    ####


def test_campaign_selects_a_bounded_candidate_after_all_generic_gates_pass() -> None:
    result = run_tuning_campaign(_adapter(with_yaw=True), _campaign(required_states=("roll_rate", "pitch_rate", "yaw_rate")))

    assert result.status == "candidate_ready"
    node = result.nodes[0]
    assert node.status == "candidate_ready"
    assert node.derivative_consistent is True
    assert node.authority_preflight is not None
    assert node.authority_preflight.status == "passed"
    assert node.lqr is not None
    assert node.lqr.best is not None
    ####


def test_campaign_stops_at_authority_before_trying_controller_candidates() -> None:
    result = run_tuning_campaign(_adapter(with_yaw=False), _campaign(required_states=("roll_rate", "pitch_rate", "yaw_rate")))

    assert result.status == "blocked"
    node = result.nodes[0]
    assert node.status == "authority_blocked"
    assert "uncontrolled_required_state:yaw_rate" in node.blockers
    assert node.lqr is None
    ####


def test_campaign_supports_a_declared_closed_nested_subsystem() -> None:
    result = run_tuning_campaign(_adapter(with_yaw=False), _nested_campaign())

    assert result.status == "candidate_ready"
    node = result.nodes[0]
    assert node.status == "candidate_ready"
    assert node.derivative_metrics["maximum_omitted_state_coupling"] == 0.0
    assert node.lqr is not None
    assert node.lqr.best is not None
    ####


def test_campaign_fails_closed_when_the_tier_has_no_trim_linearization_operations() -> None:
    adapter = StandardFamilyAdapter.passive(
        descriptor_from_control_plant(
            _plant(with_yaw=True),
            family_id="synthetic_attitude",
            adapter_id="synthetic.attitude.v1",
            physical_family="test",
            tier="pseudo_6dof",
        )
    )
    result = run_tuning_campaign(adapter, _campaign(required_states=("roll_rate",)))

    assert result.status == "blocked"
    assert result.nodes[0].status == "operation_unavailable"
    assert set(result.nodes[0].blockers) == {"operation_unavailable:trim", "operation_unavailable:linearize"}
    ####
