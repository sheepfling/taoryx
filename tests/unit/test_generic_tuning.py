from __future__ import annotations

from collections.abc import Mapping

import numpy as np

from taoryx.generic_tuning import (
    AuthorityPreflightReport,
    GenericLqrProfile,
    LinearAuthorityRequirement,
    NormalizedLqrProfileGrid,
    linear_authority_preflight,
    linear_authority_preflight_evaluator,
    trim_linearize_and_tune,
    tune_lqr_profiles,
)
from taoryx.trim import TrimSpec


def _profile() -> GenericLqrProfile:
    return GenericLqrProfile("standard", (1.0, 1.0), (1.0,))


def test_normalized_lqr_profile_grid_generates_a_bounded_deterministic_lattice() -> None:
    grid = NormalizedLqrProfileGrid(
        "synthetic",
        state_weight_multipliers=(0.5, 1.0),
        control_effort_multipliers=(2.0, 1.0, 0.5),
        state_base_weights=(2.0, 1.0),
        control_base_weights=(4.0,),
    )

    profiles = grid.profiles(2, 1)

    assert len(profiles) == 6
    assert profiles[0].id == "synthetic.tracking-0.5.effort-2"
    assert profiles[0].q_diagonal == (1.0, 0.5)
    assert profiles[0].r_diagonal == (8.0,)
    assert profiles[-1].id == "synthetic.tracking-1.effort-0.5"
    assert profiles[-1].q_diagonal == (2.0, 1.0)
    assert profiles[-1].r_diagonal == (2.0,)


def test_generic_lqr_tuning_accepts_arbitrary_state_and_control_dimensions() -> None:
    report = tune_lqr_profiles(
        "synthetic_second_order_vehicle",
        ((0.0, 1.0), (-1.0, 0.0)),
        ((0.0,), (1.0,)),
        state_names=("position", "velocity"),
        control_names=("force",),
        state_scales=(10.0, 2.0),
        control_scales=(5.0,),
        profiles=(_profile(),),
    )

    assert report.best is not None
    assert report.best.lqr is not None
    assert report.best.lqr.state_names == ("position", "velocity")
    assert report.best.lqr.control_names == ("force",)
    assert report.best.safe


def test_trim_linearize_and_tune_stops_when_trim_fails() -> None:
    spec = TrimSpec(
        state_names=("position", "velocity"),
        control_names=("force",),
        residual_names=("position", "velocity"),
        state_initial={"position": 1.0, "velocity": 0.0},
        control_initial={"force": 0.0},
        state_lower={"position": 1.0, "velocity": 0.0},
        state_upper={"position": 1.0, "velocity": 0.0},
        control_lower={"force": 0.0},
        control_upper={"force": 0.0},
    )

    def evaluator(state: Mapping[str, float], controls: Mapping[str, float]) -> Mapping[str, float]:
        del controls
        return {"position": state["position"], "velocity": 0.0}

    result = trim_linearize_and_tune(
        "synthetic_second_order_vehicle",
        spec,
        evaluator,
        evaluator,
        state_scales=(1.0, 1.0),
        control_scales=(1.0,),
        profiles=(_profile(),),
    )

    assert result.status == "trim_adapter_invalid"
    assert result.trim is None
    assert result.linearization is None
    assert result.lqr is None


def test_trim_linearize_and_tune_produces_plant_backed_report() -> None:
    spec = TrimSpec(
        state_names=("position", "velocity"),
        control_names=("force",),
        residual_names=("position", "velocity"),
        state_initial={"position": 0.0, "velocity": 0.0},
        control_initial={"force": 0.0},
    )

    def evaluator(state: Mapping[str, float], controls: Mapping[str, float]) -> Mapping[str, float]:
        return {"position": state["velocity"], "velocity": -state["position"] + controls["force"]}

    result = trim_linearize_and_tune(
        "synthetic_second_order_vehicle",
        spec,
        evaluator,
        evaluator,
        state_scales=(1.0, 1.0),
        control_scales=(1.0,),
        profiles=(_profile(),),
    )

    assert result.status == "tuned"
    assert result.linearization is not None
    np.testing.assert_allclose(result.linearization.a_matrix, ((0.0, 1.0), (-1.0, 0.0)), atol=1.0e-5)
    assert result.lqr is not None
    assert result.lqr.best is not None


def test_authority_preflight_prevents_gain_search_for_a_structural_blocker() -> None:
    spec = TrimSpec(
        state_names=("position", "velocity"),
        control_names=("force",),
        residual_names=("position", "velocity"),
        state_initial={"position": 0.0, "velocity": 0.0},
        control_initial={"force": 0.0},
    )

    def evaluator(state: Mapping[str, float], controls: Mapping[str, float]) -> Mapping[str, float]:
        return {"position": state["velocity"], "velocity": -state["position"] + controls["force"]}

    def blocked_authority(*_: object) -> AuthorityPreflightReport:
        return AuthorityPreflightReport(
            "blocked",
            "the declared topology lacks independent yaw authority",
            {"effectiveness_rank": 2.0, "required_axis_count": 3.0},
            ("uncontrolled_required_axis:yaw",),
        )

    result = trim_linearize_and_tune(
        "synthetic_rank_limited_vehicle",
        spec,
        evaluator,
        evaluator,
        state_scales=(1.0, 1.0),
        control_scales=(1.0,),
        profiles=(_profile(),),
        authority_preflight=blocked_authority,
    )

    assert result.status == "authority_preflight_blocked"
    assert result.linearization is not None
    assert result.lqr is None
    assert result.authority_preflight is not None
    assert result.authority_preflight.blockers == ("uncontrolled_required_axis:yaw",)


def test_linear_authority_preflight_reports_required_state_rank_loss() -> None:
    """A missing yaw axis is a topology blocker, not an LQR-weight problem."""

    report = linear_authority_preflight(
        LinearAuthorityRequirement("two-elevon-roll-pitch", ("roll_rate", "pitch_rate", "yaw_rate")),
        state_names=("roll_rate", "pitch_rate", "yaw_rate"),
        a_matrix=((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        b_matrix=((1.0, 0.0), (0.0, 1.0), (0.0, 0.0)),
    )

    assert report.status == "blocked"
    assert report.metrics["controllability_rank"] == 2.0
    assert report.metrics["uncontrolled_fraction.yaw_rate"] == 1.0
    assert report.blockers == (
        "controllability_rank_below_requirement",
        "uncontrolled_required_state:yaw_rate",
    )


def test_linear_authority_preflight_evaluator_admits_trimmed_second_order_plant() -> None:
    spec = TrimSpec(
        state_names=("position", "velocity"),
        control_names=("force",),
        residual_names=("position", "velocity"),
        state_initial={"position": 0.0, "velocity": 0.0},
        control_initial={"force": 0.0},
    )

    def evaluator(state: Mapping[str, float], controls: Mapping[str, float]) -> Mapping[str, float]:
        return {"position": state["velocity"], "velocity": -state["position"] + controls["force"]}

    result = trim_linearize_and_tune(
        "synthetic_second_order_vehicle",
        spec,
        evaluator,
        evaluator,
        state_scales=(1.0, 1.0),
        control_scales=(1.0,),
        profiles=(_profile(),),
        authority_preflight=linear_authority_preflight_evaluator(
            LinearAuthorityRequirement("second-order-force-authority", ("position", "velocity"))
        ),
    )

    assert result.status == "tuned"
    assert result.authority_preflight is not None
    assert result.authority_preflight.status == "passed"
