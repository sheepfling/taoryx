from __future__ import annotations

import numpy as np
import pytest

from taoryx.controller_autotune import AutoTuneLimits, auto_tune_lqr_profiles, default_attitude_linearization
from taoryx.runtime import LqrUncertaintySpec
from taoryx.trim import DynamicsLinearization
from taoryx.vehicle_registry import derive_lqr_scale_contract


@pytest.mark.algorithms
def test_registered_vehicles_get_three_stable_profile_candidates() -> None:
    """The common synthesis path covers every standard vehicle family."""

    for vehicle_id in ("b747", "skywalker_x8", "hummingbird", "x15"):
        report = auto_tune_lqr_profiles(vehicle_id)
        assert report.design_source == "runtime-inertia-attitude-bridge"
        assert len(report.candidates) == 3
        assert report.best is not None
        assert all(candidate.safe for candidate in report.candidates)
        assert all(candidate.lqr is not None for candidate in report.candidates)
        assert all(candidate.scales["control_moment_scale_nm"] > 0.0 for candidate in report.candidates)
        contract = derive_lqr_scale_contract(vehicle_id)
        assert all(candidate.scales["mass_scale_kg"] == contract["mass_scale_kg"] for candidate in report.candidates)
        assert all(candidate.scales["inertia_scale_kg_m2"] == contract["inertia_scale_kg_m2"] for candidate in report.candidates)
        assert all(candidate.scales["weight_moment_scale_nm"] == contract["weight_moment_scale_nm"] for candidate in report.candidates)
        assert all(candidate.scales["inertia_moment_scale_nm"] == contract["inertia_moment_scale_nm"] for candidate in report.candidates)
        assert {"mass", "inertia", "weight_moment", "inertia_moment"}.issubset(
            report.candidates[0].scale_provenance
        )
    ####


@pytest.mark.algorithms
def test_synthesis_report_is_json_ready_and_preserves_poles() -> None:
    """Serialized synthesis output contains profile provenance diagnostics."""

    report = auto_tune_lqr_profiles("skywalker_x8")
    payload = report.as_dict()
    assert payload["status"] == "safe"
    assert payload["best_profile_id"] == "skywalker-x8-gentle"
    assert len(payload["candidates"]) == 3
    assert payload["candidates"][0]["closed_loop_eigenvalues"]
    assert payload["candidates"][0]["scales"]["state_angle_scale_rad"] > 0.0
    ####


@pytest.mark.algorithms
def test_operational_evaluator_can_reject_stable_candidate() -> None:
    """A stable pole set does not bypass table or saturation gates."""

    def evaluator(_result: object) -> dict[str, float]:
        return {
            "table_margin_min_normalized": 0.04,
            "control_saturation_fraction": 0.25,
            "control_rate_abs_max": 2.0,
            "tracking_error": 9.0,
        }

    report = auto_tune_lqr_profiles(
        "b747",
        limits=AutoTuneLimits(
            minimum_table_margin=0.10,
            maximum_saturation_fraction=0.0,
            maximum_control_rate=1.0,
            maximum_tracking_error=1.0,
        ),
        evaluator=evaluator,
    )
    assert report.best is None
    for candidate in report.candidates:
        assert candidate.status == "unsafe"
        assert "table-margin-limit" in candidate.violations
        assert "control-saturation-limit" in candidate.violations
        assert "control-rate-limit" in candidate.violations
        assert "tracking-error-limit" in candidate.violations
    ####


@pytest.mark.algorithms
def test_linearization_matrices_must_be_supplied_as_a_pair() -> None:
    """The adapter cannot silently fall back when only half a plant is given."""

    a_matrix, _ = default_attitude_linearization("hummingbird")
    with pytest.raises(ValueError, match="supplied together"):
        auto_tune_lqr_profiles("hummingbird", a_matrix=a_matrix)
    ####


@pytest.mark.algorithms
def test_autotune_can_gate_candidates_against_derivative_uncertainty() -> None:
    """Profile synthesis keeps estimated aero derivatives inside its claim boundary."""

    report = auto_tune_lqr_profiles(
        "hummingbird",
        limits=AutoTuneLimits(
            derivative_uncertainty=LqrUncertaintySpec(a_fraction=0.15, b_fraction=0.15, samples=13),
        ),
    )
    assert all(candidate.robustness is not None for candidate in report.candidates)
    assert all("worst_uncertain_real_pole" in candidate.metrics for candidate in report.candidates)


@pytest.mark.algorithms
def test_autotune_accepts_source_trim_dynamics_linearization() -> None:
    """The autotuner can consume a true state-derivative Jacobian artifact."""

    linearization = DynamicsLinearization(
        ("attitude-error-x", "attitude-error-y", "attitude-error-z", "p", "q", "r"),
        ("moment-x", "moment-y", "moment-z"),
        np.block([[np.zeros((3, 3)), np.eye(3)], [np.zeros((3, 6))]]),
        np.vstack((np.zeros((3, 3)), np.eye(3))),
        {name: 0.0 for name in ("attitude-error-x", "attitude-error-y", "attitude-error-z", "p", "q", "r")},
        {name: 0.0 for name in ("moment-x", "moment-y", "moment-z")},
        {"source_quality": "estimated"},
    )
    report = auto_tune_lqr_profiles("hummingbird", linearization=linearization)
    assert report.design_source == "source-trim-dynamics-linearization"
