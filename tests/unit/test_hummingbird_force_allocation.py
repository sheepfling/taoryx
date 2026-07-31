from __future__ import annotations

import math

from tools.validate_hummingbird_physical_lqr import build_plant


def test_hummingbird_source_plant_exposes_six_axis_physical_effectiveness() -> None:
    plant = build_plant()
    effectiveness = plant.force_moment_effectiveness(plant.source_local_state, plant.source_effectors)

    assert effectiveness.wrench_names == (
        "force_x_n",
        "force_y_n",
        "force_z_n",
        "moment_x_nm",
        "moment_y_nm",
        "moment_z_nm",
    )
    assert effectiveness.effector_names == tuple(plant.control_names)
    assert effectiveness.array.shape == (6, 4)
    assert all(math.isfinite(value) for row in effectiveness.matrix for value in row)
    assert all(value < 0.0 for value in effectiveness.matrix[2])
    ####


def test_hummingbird_force_request_is_allocated_through_rotors_and_reports_residual() -> None:
    plant = build_plant()
    wrench_names = plant.force_moment_effectiveness(
        plant.source_local_state,
        plant.source_effectors,
    ).wrench_names
    desired = {name: 0.0 for name in wrench_names}
    desired["force_z_n"] = -0.05
    weights = {name: float(name == "force_z_n") for name in wrench_names}

    step = plant.allocate_force_moment(
        plant.source_local_state,
        desired,
        plant.source_effectors,
        0.02,
        wrench_weights=weights,
    )

    assert step.allocation.status in {"feasible", "feasible_near_limit", "partially_achievable"}
    assert set(step.actuator.actual_positions) == set(plant.control_names)
    assert all(name.startswith("rotor-") for name in step.actuator.actual_positions)
    assert step.achieved_wrench["force_z_n"] < 0.0
    assert math.isfinite(step.achieved_residual_wrench["force_z_n"])
    assert abs(step.achieved_residual_wrench["force_z_n"]) > 1.0e-9
    assert math.isclose(
        step.achieved_residual_wrench["force_z_n"],
        desired["force_z_n"] - step.achieved_wrench["force_z_n"],
        rel_tol=1.0e-12,
        abs_tol=1.0e-12,
    )
    assert "direct_force" not in step.actuator.actual_positions
    assert "direct_moment" not in step.actuator.actual_positions
    ####
