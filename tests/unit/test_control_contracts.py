from __future__ import annotations

import pytest

from taoryx.contracts import Vector3
from taoryx.control import (
    ControlCommand,
    ControlDemand,
    DirectControlAllocator,
    ProportionalHoldController,
    SegmentController,
    SegmentPlan,
    SegmentSchedule,
    VehicleObservation,
    schedule_from_rows,
)
from taoryx.control_adapters import FunctionalPlantAdapter, QuadRotorControlAllocator
from taoryx.rotorcraft import QuadRotorAllocation


def test_segment_controller_resets_and_allocates_at_boundaries() -> None:
    schedule = SegmentSchedule(
        (
            SegmentPlan("hold", 0.0, 1.0, {"altitude_m": 10.0}),
            SegmentPlan("climb", 1.0, 2.0, {"altitude_m": 20.0}),
        )
    )
    controller = ProportionalHoldController({"altitude_m": "throttle"}, {"altitude_m": 0.5})
    runner = SegmentController(controller, schedule, DirectControlAllocator({"throttle": (0.0, 1.0)}))
    first = runner.step(VehicleObservation(0.5, {"altitude_m": 8.0}, Vector3(0.0, 0.0, 0.0)), 0.1)
    second = runner.step(VehicleObservation(1.5, {"altitude_m": 18.0}), 0.1)
    assert first.values == {"throttle": 1.0}
    assert second.values == {"throttle": 1.0}
    assert controller._reset_count == 2


def test_schedule_from_rows_is_vehicle_agnostic() -> None:
    schedule = schedule_from_rows(
        [
            {"id": "takeoff", "start_s": 0, "end_s": 2, "target": {"altitude_m": 2}},
            {"id": "loiter", "start_s": 2, "end_s": 10, "target": {"altitude_m": 2}, "controller": "hold"},
        ]
    )
    assert schedule.at(3.0).controller_name == "hold"
    assert schedule.at(3.0).target["altitude_m"] == 2.0


def test_schedule_rejects_overlap_and_unowned_time() -> None:
    with pytest.raises(ValueError, match="overlapping"):
        SegmentSchedule((SegmentPlan("a", 0, 2), SegmentPlan("b", 1, 3)))
    schedule = SegmentSchedule((SegmentPlan("a", 0, 1),))
    with pytest.raises(ValueError, match="no segment"):
        schedule.at(1.0)


def test_functional_plant_adapter_preserves_common_result() -> None:
    from taoryx.control import PlantEvaluation

    adapter = FunctionalPlantAdapter(
        lambda observation, command: PlantEvaluation(
            diagnostics={"time": observation.time_s, "commanded": bool(command.values)}
        )
    )
    result = adapter.evaluate(VehicleObservation(0.5, {"altitude_m": 2.0}), ControlCommand({"throttle": 0.5}))
    assert result.diagnostics == {"time": 0.5, "commanded": True}


def test_quadrotor_allocator_emits_individual_rotor_controls() -> None:
    allocator = QuadRotorControlAllocator(QuadRotorAllocation(0.17, 5.57e-6, 1.36e-7))
    command = allocator.allocate(
        ControlDemand({"collective_speed_rad_s": 469.0, "moment_x_nm": 0.0, "moment_y_nm": 0.0, "moment_z_nm": 0.0}),
        VehicleObservation(0.0, {}),
    )
    assert set(command.values) == {"rotor-1-speed", "rotor-2-speed", "rotor-3-speed", "rotor-4-speed"}
    assert all(value == pytest.approx(469.0) for value in command.values.values())
