from __future__ import annotations

from collections.abc import Mapping

import numpy as np
import pytest

from taoryx.control_allocation import (
    DerivativeProvenance,
    EffectorEffectiveness,
    EffectorLimits,
    PhysicalAllocationStep,
    ProvenancedLinearization,
    allocate_and_advance_wrench,
)
from taoryx.physical_lqr import (
    design_physical_wrench_lqr,
    project_linearization_to_wrench,
    validate_nonlinear_wrench_lqr,
)
from taoryx.trim import DynamicsLinearization, TrimResult, TrimSpec


class _SecondOrderPhysicalPlant:
    """Analytic plant proving the generic demand-to-effector validation path."""

    state_names = ("angle_rad", "rate_rad_s")
    control_names = ("elevon_rad",)

    def __init__(self) -> None:
        self._limits = {"elevon_rad": EffectorLimits("elevon_rad", -1.0, 1.0, "rad")}
        ####

    def state_derivative(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        del environment
        return {
            "angle_rad": float(state["rate_rad_s"]),
            "rate_rad_s": float(effectors["elevon_rad"]),
        }
        ####

    def allocate(
        self,
        state: Mapping[str, float],
        desired_wrench: Mapping[str, float],
        previous_effectors: Mapping[str, float],
        dt_s: float,
    ) -> PhysicalAllocationStep:
        del state
        return allocate_and_advance_wrench(
            _effectiveness(),
            self._limits,
            desired_wrench,
            previous_effectors,
            dt_s,
        )
        ####

    def trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        """Return the analytic fixture equilibrium through the common protocol."""

        del target, initial_guess
        return _trim()
        ####

    def linearize(self, trim: TrimResult, options: Mapping[str, float | str]) -> ProvenancedLinearization:
        """Return the analytic fixture derivative through the common protocol."""

        del trim, options
        return _linearization()
        ####

    def effectiveness(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> EffectorEffectiveness:
        """Return the analytic physical effector contract."""

        del state, effectors
        return _effectiveness()
        ####


def _effectiveness() -> EffectorEffectiveness:
    return EffectorEffectiveness(
        wrench_names=("pitch_moment_nm",),
        effector_names=("elevon_rad",),
        matrix=((1.0,),),
        reference_wrench={"pitch_moment_nm": 0.0},
        reference_effectors={"elevon_rad": 0.0},
        source="analytic-second-order-fixture",
    )
    ####


def _trim() -> TrimResult:
    specification = TrimSpec(
        state_names=("angle_rad", "rate_rad_s"),
        control_names=("elevon_rad",),
        residual_names=("angle_residual", "rate_residual"),
        state_initial={"angle_rad": 0.0, "rate_rad_s": 0.0},
        control_initial={"elevon_rad": 0.0},
    )
    return TrimResult(
        specification,
        {"angle_rad": 0.0, "rate_rad_s": 0.0},
        {"elevon_rad": 0.0},
        {"angle_residual": 0.0, "rate_residual": 0.0},
        0.0,
        True,
        1,
        "analytic trim",
        1,
        0.0,
    )
    ####


def _linearization() -> ProvenancedLinearization:
    trim = _trim()
    primary = DynamicsLinearization(
        trim.spec.state_names,
        trim.spec.control_names,
        np.asarray(((0.0, 1.0), (0.0, 0.0))),
        np.asarray(((0.0,), (1.0,))),
        trim.state,
        trim.controls,
        {"method": "analytic"},
    )
    provenance = DerivativeProvenance(
        nonlinear_plant_id="analytic-second-order-fixture",
        nonlinear_plant_revision="test-v1",
        state_names=primary.state_names,
        control_names=primary.control_names,
        method="analytic-fixture",
        state_step=1.0e-4,
        control_step=1.0e-4,
        comparison_state_step=5.0e-5,
        comparison_control_step=5.0e-5,
        maximum_relative_difference=0.0,
        maximum_absolute_difference=0.0,
        comparison_absolute_floor=0.0,
        derivative_consistent=True,
        state_units={"angle_rad": "rad", "rate_rad_s": "rad/s"},
        control_units={"elevon_rad": "rad"},
    )
    return ProvenancedLinearization(primary, primary, provenance)
    ####


def test_physical_wrench_lqr_projects_real_effectors_and_recovers_the_nonlinear_plant() -> None:
    trim = _trim()
    projection = project_linearization_to_wrench(
        _linearization(),
        _effectiveness(),
        state_names=("angle_rad", "rate_rad_s"),
        wrench_names=("pitch_moment_nm",),
        effector_names=("elevon_rad",),
    )
    design = design_physical_wrench_lqr(
        "analytic-physical-wrench-lqr",
        projection,
        q_diagonal=(10.0, 1.0),
        r_diagonal=(1.0,),
        state_scales=(0.2, 1.0),
        wrench_scales=(1.0,),
    )
    result = validate_nonlinear_wrench_lqr(
        _SecondOrderPhysicalPlant(),
        trim,
        design,
        initial_state={"angle_rad": 0.02, "rate_rad_s": 0.0},
        duration_s=3.0,
        dt_s=0.01,
    )

    assert design.result.hurwitz
    assert design.result.controllable
    assert result.final_feedback_error_norm < result.initial_feedback_error_norm * 0.05
    assert result.initial_normalized_feedback_error_norm == pytest.approx(0.1)
    assert result.final_normalized_feedback_error_norm < result.initial_normalized_feedback_error_norm * 0.05
    assert result.final_controlled_actual_residual < 1.0e-8
    assert result.allocation_statuses == ("feasible",)
    assert result.saturation_fraction == pytest.approx(0.0)
    ####


def test_wrench_projection_rejects_an_underactuated_requested_axis() -> None:
    trim = _trim()
    effectiveness = EffectorEffectiveness(
        wrench_names=("roll_moment_nm", "pitch_moment_nm"),
        effector_names=("elevon_rad",),
        matrix=((1.0,), (0.0,)),
        reference_wrench={"roll_moment_nm": 0.0, "pitch_moment_nm": 0.0},
        reference_effectors={"elevon_rad": 0.0},
        source="rank-deficient-fixture",
    )
    with pytest.raises(ValueError, match="cannot independently realize"):
        project_linearization_to_wrench(
            _linearization(),
            effectiveness,
            state_names=trim.spec.state_names,
            wrench_names=("roll_moment_nm", "pitch_moment_nm"),
            effector_names=("elevon_rad",),
        )
    ####
