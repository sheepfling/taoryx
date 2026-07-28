from __future__ import annotations

import math
from collections.abc import Mapping

import pytest

from taoryx.control import ControlDemand, VehicleObservation
from taoryx.control_allocation import (
    ConstrainedWrenchAllocator,
    EffectorEffectiveness,
    EffectorLimits,
    advance_actuators,
    allocate_and_advance_wrench,
    allocate_bounded_weighted_wrench,
    finite_difference_linearization_with_provenance,
)
from taoryx.trim import TrimSpec, solve_trim


def _effectiveness(
    matrix: tuple[tuple[float, ...], ...],
    *,
    wrench_names: tuple[str, ...] = ("moment_x_nm", "moment_y_nm"),
    effector_names: tuple[str, ...] = ("left", "right"),
) -> EffectorEffectiveness:
    return EffectorEffectiveness(
        wrench_names=wrench_names,
        effector_names=effector_names,
        matrix=matrix,
        reference_wrench={name: 0.0 for name in wrench_names},
        reference_effectors={name: 0.0 for name in effector_names},
        source="analytic-test-fixture",
    )
    ####


def _limits(*, available_right: bool = True, rate: float | None = None) -> dict[str, EffectorLimits]:
    return {
        "left": EffectorLimits("left", -1.0, 1.0, "rad", rate_limit_per_s=rate),
        "right": EffectorLimits("right", -1.0, 1.0, "rad", rate_limit_per_s=rate, available=available_right),
    }
    ####


def test_bounded_allocator_recovers_a_feasible_wrench_exactly() -> None:
    result = allocate_bounded_weighted_wrench(
        _effectiveness(((1.0, 0.0), (0.0, 1.0))),
        _limits(),
        {"moment_x_nm": 0.3, "moment_y_nm": -0.4},
        {"left": 0.0, "right": 0.0},
        0.1,
    )

    assert result.status == "feasible"
    assert result.effector_commands == pytest.approx({"left": 0.3, "right": -0.4})
    assert result.predicted_wrench == pytest.approx({"moment_x_nm": 0.3, "moment_y_nm": -0.4})
    assert result.residual_norm < 1.0e-10
    assert result.controlled_wrench_axes == ("moment_x_nm", "moment_y_nm")
    assert result.uncontrolled_wrench_axes == ()
    ####


def test_allocator_exposes_an_infeasible_bound_residual() -> None:
    result = allocate_bounded_weighted_wrench(
        _effectiveness(((1.0,),), wrench_names=("moment_x_nm",), effector_names=("elevon",)),
        {"elevon": EffectorLimits("elevon", -1.0, 1.0, "rad")},
        {"moment_x_nm": 3.0},
        {"elevon": 0.0},
        0.1,
    )

    assert result.status == "partially_achievable"
    assert result.effector_commands["elevon"] == pytest.approx(1.0)
    assert result.residual_wrench["moment_x_nm"] == pytest.approx(2.0)
    assert result.position_saturated == ("elevon",)
    ####


def test_allocator_enforces_rate_bounds_before_actuator_response() -> None:
    result = allocate_bounded_weighted_wrench(
        _effectiveness(((1.0,),), wrench_names=("moment_x_nm",), effector_names=("elevon",)),
        {"elevon": EffectorLimits("elevon", -1.0, 1.0, "rad", rate_limit_per_s=2.0)},
        {"moment_x_nm": 1.0},
        {"elevon": 0.0},
        0.1,
    )

    assert result.status == "partially_achievable"
    assert result.effector_commands["elevon"] == pytest.approx(0.2)
    assert result.rate_limited == ("elevon",)
    assert result.residual_wrench["moment_x_nm"] == pytest.approx(0.8)
    ####


def test_allocator_handles_rank_deficiency_and_disabled_effectors_without_hiding_error() -> None:
    rank_deficient = allocate_bounded_weighted_wrench(
        _effectiveness(((1.0, 1.0), (2.0, 2.0))),
        _limits(),
        {"moment_x_nm": 1.0, "moment_y_nm": 0.0},
        {"left": 0.0, "right": 0.0},
        0.1,
    )
    disabled = allocate_bounded_weighted_wrench(
        _effectiveness(((1.0, 0.0), (0.0, 1.0))),
        _limits(available_right=False),
        {"moment_x_nm": 0.0, "moment_y_nm": 0.5},
        {"left": 0.0, "right": 0.0},
        0.1,
    )

    assert rank_deficient.effectiveness_rank == 1
    assert rank_deficient.status in {"infeasible", "partially_achievable"}
    assert disabled.status == "partially_achievable"
    assert disabled.unavailable_effectors == ("right",)
    assert disabled.effector_commands["right"] == pytest.approx(0.0)
    assert disabled.residual_wrench["moment_y_nm"] == pytest.approx(0.5)
    ####


def test_allocator_reports_explicitly_uncontrolled_axes_without_hiding_them() -> None:
    result = allocate_bounded_weighted_wrench(
        _effectiveness(
            ((1.0, 0.0), (0.0, 1.0), (0.5, 0.5)),
            wrench_names=("roll", "pitch", "yaw"),
        ),
        _limits(),
        {"roll": 0.2, "pitch": -0.3, "yaw": 0.0},
        {"left": 0.0, "right": 0.0},
        0.1,
        wrench_weights={"roll": 1.0, "pitch": 1.0, "yaw": 0.0},
    )

    assert result.status == "feasible"
    assert result.controlled_wrench_axes == ("roll", "pitch")
    assert result.uncontrolled_wrench_axes == ("yaw",)
    assert result.residual_wrench["yaw"] != 0.0
    assert result.controlled_residual_norm < 1.0e-10
    ####


def test_zero_effectiveness_is_reported_as_numerically_singular() -> None:
    result = allocate_bounded_weighted_wrench(
        _effectiveness(((0.0,),), wrench_names=("moment_x_nm",), effector_names=("elevon",)),
        {"elevon": EffectorLimits("elevon", -1.0, 1.0, "rad")},
        {"moment_x_nm": 1.0},
        {"elevon": 0.0},
        0.1,
    )

    assert result.status == "numerically_singular"
    assert result.effectiveness_rank == 0
    ####


def test_actuator_dynamics_expose_lag_and_rate_limits() -> None:
    effectors = {
        "elevon": EffectorLimits("elevon", -1.0, 1.0, "rad", rate_limit_per_s=1.0, time_constant_s=1.0),
    }
    result = advance_actuators(effectors, {"elevon": 1.0}, {"elevon": 0.0}, 0.5)

    assert result.actual_positions["elevon"] == pytest.approx(1.0 - math.exp(-0.5))
    assert result.lag_active == ("elevon",)
    assert result.rate_limited == ()
    ####


def test_physical_step_reports_actual_wrench_after_lag() -> None:
    effectors = {"elevon": EffectorLimits("elevon", -1.0, 1.0, "rad", time_constant_s=1.0)}
    step = allocate_and_advance_wrench(
        _effectiveness(((1.0,),), wrench_names=("moment_x_nm",), effector_names=("elevon",)),
        effectors,
        {"moment_x_nm": 1.0},
        {"elevon": 0.0},
        1.0,
    )

    assert step.allocation.status == "feasible_near_limit"
    assert step.achieved_wrench["moment_x_nm"] < 1.0
    assert step.achieved_residual_wrench["moment_x_nm"] > 0.0
    ####


def test_physical_step_exports_requested_and_achieved_controller_telemetry() -> None:
    step = allocate_and_advance_wrench(
        _effectiveness(((1.0,),), wrench_names=("moment_x_nm",), effector_names=("elevon",)),
        {"elevon": EffectorLimits("elevon", -1.0, 1.0, "rad", rate_limit_per_s=1.0)},
        {"moment_x_nm": 1.0},
        {"elevon": 0.0},
        0.1,
    )

    telemetry = step.as_controller_result("test-elevon-allocator")

    assert telemetry.allocator_id == "test-elevon-allocator"
    assert telemetry.requested["moment_x_nm"] == pytest.approx(1.0)
    assert telemetry.actual_effectors["elevon"] == pytest.approx(0.1)
    assert telemetry.rate_limited_channels == ("elevon",)
    assert telemetry.effectiveness_matrix == ((1.0,),)
    ####


def test_stateful_allocator_is_deterministic_and_uses_committed_sample_time() -> None:
    allocator = ConstrainedWrenchAllocator(
        _effectiveness(((1.0, 0.0), (0.0, 1.0))),
        _limits(rate=10.0),
    )
    demand = ControlDemand({"moment_x_nm": 0.5, "moment_y_nm": -0.5}, source="test")
    first = allocator.allocate(demand, VehicleObservation(0.0, {}))
    second = allocator.allocate(demand, VehicleObservation(0.1, {}))

    # The first committed sample owns the trim actuator state.  The command
    # changes the plant over the following accepted interval rather than
    # inventing an interpolated pre-sample actuator state.
    assert first.values == pytest.approx({"left": 0.0, "right": 0.0})
    assert second.values == pytest.approx({"left": 0.5, "right": -0.5})
    assert allocator.last_step is not None
    with pytest.raises(ValueError, match="monotonic"):
        allocator.allocate(demand, VehicleObservation(0.05, {}))
    ####


def test_derivative_provenance_compares_two_actual_plant_linearizations() -> None:
    specification = TrimSpec(
        state_names=("position", "velocity"),
        control_names=("force",),
        residual_names=("position", "velocity"),
        state_initial={"position": 0.0, "velocity": 0.0},
        control_initial={"force": 0.0},
    )

    def dynamics(state: Mapping[str, float], controls: Mapping[str, float]) -> Mapping[str, float]:
        return {"position": state["velocity"], "velocity": -state["position"] + controls["force"]}
        ####

    trim = solve_trim(specification, dynamics)
    result = finite_difference_linearization_with_provenance(
        specification,
        dynamics,
        trim,
        nonlinear_plant_id="analytic-double-integrator",
        nonlinear_plant_revision="test-v1",
        state_step=1.0e-4,
        control_step=1.0e-4,
        maximum_relative_difference=1.0e-4,
        state_units={"position": "m", "velocity": "m/s"},
        control_units={"force": "N"},
    )

    assert result.provenance.derivative_consistent
    assert result.provenance.maximum_relative_difference < 1.0e-8
    assert result.provenance.maximum_absolute_difference < 1.0e-8
    assert result.provenance.comparison_absolute_floor == 0.0
    assert result.primary.state_names == ("position", "velocity")
    ####
