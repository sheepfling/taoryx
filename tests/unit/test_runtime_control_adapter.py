from __future__ import annotations

import math
from pathlib import Path

from taoryx.contracts import Vector3
from taoryx.control_allocation import EffectorLimits
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.program import LoadedProgram
from taoryx.runtime_control_adapter import local_rigid_body_plant_from_vehicle

ROOT = Path(__file__).resolve().parents[2]
PROBLEM = ROOT / "examples/generated/vehicles/skywalker_x8_table_coordinate_trim_6dof.prb"
TABLE_ROOT = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables"
TABLES = tuple(
    TABLE_ROOT / name
    for name in (
        "skywalker_x8_static_6axis.tbl",
        "skywalker_x8_collective_elevon_6axis.tbl",
        "skywalker_x8_differential_elevon_6axis.tbl",
        "skywalker_x8_thrust.tbl",
    )
)


def _plant():
    program = LoadedProgram.load(PROBLEM, TABLES, profile=GrammarProfile.TAORYX)
    return local_rigid_body_plant_from_vehicle(
        "skywalker-x8-table-plant",
        "source-trim-local-v1",
        program.case().vehicles["1"],
        inertia_kg_m2=Vector3(0.325, 0.140, 0.400),
        reference_length_m=0.36,
        effector_limits={
            "collective-elevon-deg": EffectorLimits(
                "collective-elevon-deg",
                -20.0,
                20.0,
                "deg",
                rate_limit_per_s=120.0,
                time_constant_s=0.05,
            ),
            "differential-elevon-deg": EffectorLimits(
                "differential-elevon-deg",
                -20.0,
                20.0,
                "deg",
                rate_limit_per_s=120.0,
                time_constant_s=0.05,
            ),
            "throttle": EffectorLimits("throttle", 0.0, 1.0, "fraction", time_constant_s=0.2),
        },
        effectiveness_steps={
            "collective-elevon-deg": 0.1,
            "differential-elevon-deg": 0.1,
            "throttle": 0.005,
        },
        allocation_wrench_weights={
            "moment_x_nm": 1.0,
            "moment_y_nm": 1.0,
            "moment_z_nm": 0.0,
        },
    )
    ####


def test_x8_local_adapter_uses_runtime_table_plant_for_state_and_effectiveness() -> None:
    plant = _plant()
    state = plant.source_local_state
    controls = plant.source_effectors

    derivative = plant.state_derivative(state, controls, {})
    effectiveness = plant.effectiveness(state, controls)

    assert set(derivative) == set(plant.state_names)
    assert all(math.isfinite(value) for value in derivative.values())
    assert effectiveness.effector_names == (
        "collective-elevon-deg",
        "differential-elevon-deg",
        "throttle",
    )
    assert effectiveness.wrench_names == ("moment_x_nm", "moment_y_nm", "moment_z_nm")
    assert effectiveness.array.shape == (3, 3)
    assert max(abs(value) for row in effectiveness.matrix for value in row) > 0.0
    ####


def test_x8_local_adapter_trim_and_linearization_are_derived_from_actual_effectors() -> None:
    plant = _plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)

    assert trim.success, trim.as_dict()
    assert trim.scaled_residual_norm < 1.0e-3
    linearization = plant.linearize(trim, {"state_step": 1.0e-4, "control_step": 1.0e-3})

    assert linearization.primary.state_names == plant.state_names
    assert linearization.primary.control_names == plant.control_names
    assert linearization.provenance.nonlinear_plant_id == "skywalker-x8-table-plant"
    assert linearization.provenance.derivative_consistent
    ####


def test_x8_local_adapter_maps_a_requested_moment_through_actual_elevons() -> None:
    plant = _plant()
    state = plant.source_local_state
    controls = plant.source_effectors
    baseline = plant.effectiveness(state, controls).reference_wrench
    step = plant.allocate(
        state,
        {
            "moment_x_nm": baseline["moment_x_nm"] + 0.05,
            "moment_y_nm": baseline["moment_y_nm"] - 0.02,
            "moment_z_nm": baseline["moment_z_nm"],
        },
        controls,
        0.1,
    )

    assert step.allocation.status in {"feasible", "feasible_near_limit"}
    assert step.allocation.controlled_wrench_axes == ("moment_x_nm", "moment_y_nm")
    assert step.allocation.uncontrolled_wrench_axes == ("moment_z_nm",)
    assert set(step.actuator.actual_positions) == set(plant.control_names)
    assert all(math.isfinite(value) for value in step.achieved_wrench.values())
    assert step.achieved_residual_norm >= 0.0
    assert "direct_moment" not in step.actuator.actual_positions
    ####
