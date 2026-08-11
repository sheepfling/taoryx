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
    tune_lqi_profiles,
    tune_lqr_profiles,
    validate_nonlinear_native_coordinate_lqi,
)
from taoryx.trim import TrimResult, TrimSpec


def _profile() -> GenericLqrProfile:
    return GenericLqrProfile("standard", (1.0, 1.0), (1.0,))


class _SecondOrderNativeControlPlant:
    """Analytic native-control plant with no physical allocation claim."""

    state_names = ("position", "velocity")
    control_names = ("force",)

    def state_derivative(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        return {
            "position": float(state["velocity"]),
            "velocity": -float(state["position"]) + float(controls["force"]) + float(environment.get("bias", 0.0)),
        }
        ####

    ####


def _native_trim() -> TrimResult:
    specification = TrimSpec(
        state_names=("position", "velocity"),
        control_names=("force",),
        residual_names=("position_residual", "velocity_residual"),
        state_initial={"position": 0.0, "velocity": 0.0},
        control_initial={"force": 0.0},
    )
    return TrimResult(
        specification,
        {"position": 0.0, "velocity": 0.0},
        {"force": 0.0},
        {"position_residual": 0.0, "velocity_residual": 0.0},
        0.0,
        True,
        1,
        "analytic native trim",
        1,
        0.0,
    )
    ####


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


def test_normalized_lqr_profile_grid_can_vary_lqi_integral_priority() -> None:
    """Offset-free candidate grids retain the selected integral priority in artifacts."""

    grid = NormalizedLqrProfileGrid(
        "synthetic-lqi",
        state_weight_multipliers=(1.0,),
        control_effort_multipliers=(1.0,),
        integral_weight_multipliers=(0.5, 4.0),
    )

    profiles = grid.profiles(2, 1)
    report = tune_lqi_profiles(
        "synthetic_integral_grid",
        ((0.0, 1.0), (-1.0, 0.0)),
        ((0.0,), (1.0,)),
        state_names=("position", "velocity"),
        control_names=("force",),
        state_scales=(0.2, 1.0),
        control_scales=(1.0,),
        profiles=profiles,
        output_names=("position",),
        integral_q_diagonal=(20.0,),
    )

    assert [profile.id for profile in profiles] == [
        "synthetic-lqi.tracking-1.effort-1.integral-0.5",
        "synthetic-lqi.tracking-1.effort-1.integral-4",
    ]
    assert [candidate.integral_q_diagonal for candidate in report.candidates] == [(10.0,), (80.0,)]
    assert [candidate.as_dict()["integral_weight_multiplier"] for candidate in report.candidates] == [0.5, 4.0]


def test_generic_lqi_prefers_nominal_integral_priority_when_linear_scores_tie() -> None:
    """Automatic selection must not depend on the declared profile-list order."""

    profiles = NormalizedLqrProfileGrid(
        "synthetic-lqi-nominal",
        state_weight_multipliers=(1.0,),
        control_effort_multipliers=(1.0,),
        integral_weight_multipliers=(0.1, 1.0, 10.0),
    ).profiles(2, 1)
    report = tune_lqi_profiles(
        "synthetic_nominal_integral_selection",
        ((0.0, 1.0), (-1.0, 0.0)),
        ((0.0,), (1.0,)),
        state_names=("position", "velocity"),
        control_names=("force",),
        state_scales=(0.2, 1.0),
        control_scales=(1.0,),
        profiles=profiles,
        output_names=("position",),
        integral_q_diagonal=(20.0,),
    )

    assert report.best is not None
    assert report.best.integral_weight_multiplier == 1.0
    ####


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


def test_generic_lqi_retains_integral_gains_and_validates_native_control_coordinates() -> None:
    """Generic LQI can be replayed through model controls without an allocator fiction."""

    report = tune_lqi_profiles(
        "synthetic_native_second_order",
        ((0.0, 1.0), (-1.0, 0.0)),
        ((0.0,), (1.0,)),
        state_names=("position", "velocity"),
        control_names=("force",),
        state_scales=(0.2, 1.0),
        control_scales=(1.0,),
        profiles=(GenericLqrProfile("offset-free", (10.0, 1.0), (1.0,)),),
        output_names=("position",),
        integral_q_diagonal=(20.0,),
    )
    candidate = report.best
    assert candidate is not None
    assert candidate.lqi is not None
    assert candidate.lqi.output_names == ("position",)
    assert candidate.as_dict()["lqi_controller"] is not None

    validation = validate_nonlinear_native_coordinate_lqi(
        _SecondOrderNativeControlPlant(),
        _native_trim(),
        candidate,
        initial_state={"position": 0.02, "velocity": 0.0},
        duration_s=8.0,
        dt_s=0.01,
        control_lower={"force": -1.0},
        control_upper={"force": 1.0},
        integral_lower={"position": -0.5},
        integral_upper={"position": 0.5},
        environment={"bias": 0.01},
    )

    assert validation.integrators_exercised
    assert validation.final_normalized_feedback_error_norm < validation.initial_normalized_feedback_error_norm * 0.05
    assert validation.control_saturation_fraction == 0.0
    payload = validation.as_dict()
    assert payload["control_realization"] == "native_named_coordinates"
    assert payload["environment"] == {"bias": 0.01}
    ####


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
