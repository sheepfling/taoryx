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
    PhysicalWrenchLqrSchedule,
    PhysicalWrenchLqrScheduleNode,
    design_physical_wrench_lqi,
    design_physical_wrench_lqr,
    project_linearization_to_wrench,
    run_scheduled_physical_wrench_transition,
    validate_nonlinear_wrench_lqi,
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
        bias = float(environment.get("pitch_bias_rad_s2", 0.0))
        return {
            "angle_rad": float(state["rate_rad_s"]),
            "rate_rad_s": float(effectors["elevon_rad"]) + bias,
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


def test_physical_wrench_lqr_accepts_explicit_local_state_reference() -> None:
    """A maneuver reference changes feedback demand without bypassing allocation."""

    trim = _trim()
    projection = project_linearization_to_wrench(
        _linearization(),
        _effectiveness(),
        state_names=("angle_rad", "rate_rad_s"),
        wrench_names=("pitch_moment_nm",),
        effector_names=("elevon_rad",),
    )
    design = design_physical_wrench_lqr(
        "reference-test",
        projection,
        q_diagonal=(10.0, 1.0),
        r_diagonal=(1.0,),
        state_scales=(0.2, 1.0),
        wrench_scales=(1.0,),
    )
    requested, increment = design.requested_wrench_for_reference(
        trim.state,
        {"angle_rad": 0.0, "rate_rad_s": 0.1},
    )

    assert requested["pitch_moment_nm"] == pytest.approx(increment["pitch_moment_nm"])
    assert increment["pitch_moment_nm"] > 0.0
    ####


def test_physical_wrench_lqi_integrates_an_explicit_output_without_bypassing_allocation() -> None:
    """LQI retains the wrench/effector boundary while removing constant angle bias."""

    projection = project_linearization_to_wrench(
        _linearization(),
        _effectiveness(),
        state_names=("angle_rad", "rate_rad_s"),
        wrench_names=("pitch_moment_nm",),
        effector_names=("elevon_rad",),
    )
    design = design_physical_wrench_lqi(
        "analytic-physical-wrench-lqi",
        projection,
        output_names=("angle_rad",),
        q_diagonal=(10.0, 1.0),
        r_diagonal=(1.0,),
        integral_q_diagonal=(20.0,),
        state_scales=(0.2, 1.0),
        wrench_scales=(1.0,),
    )
    controller = design.build_controller(
        wrench_lower={"pitch_moment_nm": -1.0},
        wrench_upper={"pitch_moment_nm": 1.0},
        integral_lower={"angle_rad": -0.5},
        integral_upper={"angle_rad": 0.5},
    )
    command = controller.command(
        {"angle_rad": 0.02, "rate_rad_s": 0.0},
        {"angle_rad": 0.0},
        dt=0.01,
    )

    assert design.result.hurwitz
    assert design.result.design.controllable
    assert command.controls["pitch_moment_nm"] < 0.0
    assert controller.integral_error["angle_rad"] == pytest.approx(0.0002)
    assert design.as_dict()["method"] == "lqi"
    ####


def test_physical_wrench_lqi_validates_integral_actions_through_the_real_allocator() -> None:
    """The reusable LQI runner retains integral and achieved-wrench evidence."""

    trim = _trim()
    projection = project_linearization_to_wrench(
        _linearization(),
        _effectiveness(),
        state_names=("angle_rad", "rate_rad_s"),
        wrench_names=("pitch_moment_nm",),
        effector_names=("elevon_rad",),
    )
    design = design_physical_wrench_lqi(
        "analytic-physical-wrench-lqi-validation",
        projection,
        output_names=("angle_rad",),
        q_diagonal=(10.0, 1.0),
        r_diagonal=(1.0,),
        integral_q_diagonal=(20.0,),
        state_scales=(0.2, 1.0),
        wrench_scales=(1.0,),
    )
    result = validate_nonlinear_wrench_lqi(
        _SecondOrderPhysicalPlant(),
        trim,
        design,
        initial_state={"angle_rad": 0.02, "rate_rad_s": 0.0},
        duration_s=4.0,
        dt_s=0.01,
        wrench_lower={"pitch_moment_nm": -1.0},
        wrench_upper={"pitch_moment_nm": 1.0},
        integral_lower={"angle_rad": -0.5},
        integral_upper={"angle_rad": 0.5},
    )

    payload = result.as_dict()
    assert result.integrators_exercised is True
    assert result.final_feedback_error_norm < result.initial_feedback_error_norm
    assert result.final_controlled_actual_residual < 1.0e-8
    assert result.allocation_statuses == ("feasible",)
    assert payload["schema"] == "taoryx.physical-lqi-validation/v1alpha1"
    assert payload["samples"][0]["lqi_integral_error"]["angle_rad"] == pytest.approx(0.0002)
    assert "lqr_wrench_increment" not in payload["samples"][0]
    ####


def test_physical_wrench_lqi_rejects_a_constant_matched_disturbance_through_allocation() -> None:
    """A declared derivative bias exercises the offset-free path without injection."""

    trim = _trim()
    projection = project_linearization_to_wrench(
        _linearization(),
        _effectiveness(),
        state_names=("angle_rad", "rate_rad_s"),
        wrench_names=("pitch_moment_nm",),
        effector_names=("elevon_rad",),
    )
    design = design_physical_wrench_lqi(
        "analytic-physical-wrench-lqi-bias-rejection",
        projection,
        output_names=("angle_rad",),
        q_diagonal=(10.0, 1.0),
        r_diagonal=(1.0,),
        integral_q_diagonal=(20.0,),
        state_scales=(0.2, 1.0),
        wrench_scales=(1.0,),
    )
    result = validate_nonlinear_wrench_lqi(
        _SecondOrderPhysicalPlant(),
        trim,
        design,
        initial_state={"angle_rad": 0.0, "rate_rad_s": 0.0},
        duration_s=8.0,
        dt_s=0.01,
        wrench_lower={"pitch_moment_nm": -1.0},
        wrench_upper={"pitch_moment_nm": 1.0},
        integral_lower={"angle_rad": -0.5},
        integral_upper={"angle_rad": 0.5},
        environment={"pitch_bias_rad_s2": 0.01},
    )

    assert result.environment == {"pitch_bias_rad_s2": 0.01}
    assert result.integrators_exercised is True
    assert abs(result.final_state["angle_rad"]) < 0.002
    assert result.final_controlled_actual_residual < 1.0e-8
    assert result.allocation_statuses == ("feasible",)
    ####


def test_physical_wrench_lqr_schedule_interpolates_without_bypassing_allocator() -> None:
    """A scheduled demand is continuous and remains a wrench request only."""

    projection = project_linearization_to_wrench(
        _linearization(),
        _effectiveness(),
        state_names=("angle_rad", "rate_rad_s"),
        wrench_names=("pitch_moment_nm",),
        effector_names=("elevon_rad",),
    )
    design = design_physical_wrench_lqr(
        "schedule-node",
        projection,
        q_diagonal=(10.0, 1.0),
        r_diagonal=(1.0,),
        state_scales=(0.2, 1.0),
        wrench_scales=(1.0,),
    )
    schedule = PhysicalWrenchLqrSchedule(
        (
            PhysicalWrenchLqrScheduleNode(0.0, design),
            PhysicalWrenchLqrScheduleNode(10.0, design),
        )
    )
    command = schedule.command({"angle_rad": 0.02, "rate_rad_s": 0.0}, 5.0)
    assert command.lower_node == design.id
    assert command.upper_node == design.id
    assert command.interpolation_fraction == pytest.approx(0.5)
    assert set(command.requested_wrench) == {"pitch_moment_nm"}
    assert not hasattr(command, "effectors")
    left = schedule.command({"angle_rad": 0.02, "rate_rad_s": 0.0}, -1.0)
    right = schedule.command({"angle_rad": 0.02, "rate_rad_s": 0.0}, 11.0)
    assert left.interpolation_fraction == pytest.approx(0.0)
    assert right.interpolation_fraction == pytest.approx(0.0)
    ####


def test_scheduled_transition_runner_reuses_bounded_physical_path() -> None:
    """The shared transition runner integrates a plant through real effectors."""

    projection = project_linearization_to_wrench(
        _linearization(),
        _effectiveness(),
        state_names=("angle_rad", "rate_rad_s"),
        wrench_names=("pitch_moment_nm",),
        effector_names=("elevon_rad",),
    )
    design = design_physical_wrench_lqr(
        "transition-runner-node",
        projection,
        q_diagonal=(10.0, 1.0),
        r_diagonal=(1.0,),
        state_scales=(0.2, 1.0),
        wrench_scales=(1.0,),
    )
    schedule = PhysicalWrenchLqrSchedule(
        (
            PhysicalWrenchLqrScheduleNode(0.0, design),
            PhysicalWrenchLqrScheduleNode(10.0, design),
        )
    )
    result = run_scheduled_physical_wrench_transition(
        schedule,
        start_coordinate=0.0,
        end_coordinate=10.0,
        initial_state={"angle_rad": 0.02, "rate_rad_s": 0.0},
        initial_effectors={"elevon_rad": 0.0},
        plant_for_coordinate=lambda _coordinate: _SecondOrderPhysicalPlant(),
        state_scales=(0.2, 1.0),
        duration_s=3.0,
        dt_s=0.01,
        sample_stride_steps=100,
    )

    assert result["passed"] is True
    assert result["direct_wrench_injection"] is False
    assert result["allocation_statuses"] == ["feasible"]
    assert result["saturation_steps"] == 0
    assert result["final_normalized_error"] < result["initial_normalized_error"] * 0.05
    assert result["samples"][-1]["lower_node"] == design.id
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
