from __future__ import annotations

from collections.abc import Mapping

import numpy as np

from taoryx.generic_tuning import GenericLqrProfile, trim_linearize_and_tune, tune_lqr_profiles
from taoryx.trim import TrimSpec


def _profile() -> GenericLqrProfile:
    return GenericLqrProfile("standard", (1.0, 1.0), (1.0,))


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
