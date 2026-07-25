from __future__ import annotations

import pytest

from taoryx.language import GrammarProfile, parse_problem_text
from taoryx.language.models import RuntimeBlock
from taoryx.runtime import SensorClockSpec
from taoryx.runtime.common import RuntimeProblem, RuntimeState, RuntimeVehicle
from taoryx.runtime.engine import get_next_time_step
from taoryx.runtime.lowering import lower_problem_document


def test_sensor_declarations_are_typed_and_profile_bound() -> None:
    source = """\
(sensor-contract)
*runtime sensor imu kind=imu cadence-s=0.01 sample=instantaneous delivery-s=0 truth=boundary rate-policy=split
*runtime sensor camera kind=camera cadence-s=0.1 phase-s=0.05 sample=interval delivery-s=0.05 truth=accepted-segment rate-policy=accumulate
*end
"""
    document = parse_problem_text(source, profile=GrammarProfile.TAORYX)
    assert not [item for item in document.diagnostics if item.severity.value == "error"]
    sensors = [block for block in document.problems[0].blocks if isinstance(block, RuntimeBlock) and block.declaration == "sensor"]
    assert [block.name for block in sensors] == ["imu", "camera"]
    assert sensors[0].attributes["cadence-s"] == "0.01"

    historical = parse_problem_text(source, profile=GrammarProfile.TAOS96)
    assert any(item.code == "taoryx-extension-requires-profile" for item in historical.diagnostics)


@pytest.mark.parametrize(
    ("header", "code"),
    (
        ("*runtime sensor imu kind=imu sample=instantaneous truth=boundary rate-policy=split", "missing-sensor-attribute"),
        ("*runtime sensor imu kind=imu cadence-s=0 sample=instantaneous truth=boundary rate-policy=split", "invalid-sensor-timing"),
        ("*runtime sensor imu kind=imu cadence-s=0.01 sample=instantaneous truth=accepted-segment rate-policy=split", "inconsistent-sensor-timing-policy"),
        ("*runtime sensor imu kind=imu cadence-s=0.01 sample=instantaneous truth=boundary rate-policy=accumulate", "inconsistent-sensor-rate-policy"),
    ),
)
def test_sensor_declarations_fail_with_located_contract_diagnostics(header: str, code: str) -> None:
    document = parse_problem_text(f"(invalid-sensor)\n{header}\n*end\n", profile=GrammarProfile.TAORYX)
    assert any(item.code == code and item.location is not None for item in document.diagnostics)


def test_sensor_clock_is_the_next_truth_boundary_not_a_posthoc_sample() -> None:
    clock = SensorClockSpec("imu", "imu", cadence_s=0.01)
    vehicle = RuntimeVehicle("vehicle", RuntimeState(0.0, (0.0,)), step_size=0.1)
    problem = RuntimeProblem({"vehicle": vehicle}, sensor_clocks=(clock,))
    assert clock.next_truth_time(0.0) == pytest.approx(0.01)
    assert clock.next_truth_time(0.01) == pytest.approx(0.02)
    assert get_next_time_step(problem, 0.1) == pytest.approx(0.01)


def test_sensor_clock_contract_rejects_invalid_modes() -> None:
    with pytest.raises(ValueError, match="instantaneous sensors require"):
        SensorClockSpec("bad", "imu", 0.01, sample_mode="instantaneous", truth_policy="accepted-segment")


def test_sensor_language_lowers_to_runtime_clock_metadata() -> None:
    source = """\
(sensor-runtime)
*3dof
*runtime sensor imu kind=imu cadence-s=0.01 sample=instantaneous delivery-s=0 truth=boundary rate-policy=split
*trajectory 1 vehicle start on 1
*initial geodetic alt=100 lat=0 long=0 vel=10 gamma=0 psi=0 mass=1 time=0
*segment 1
*integ dt=0.1
*when time>0.3 stop
*end
"""
    document = parse_problem_text(source, profile=GrammarProfile.TAORYX)
    case = lower_problem_document(document).cases[0].problem
    assert case.metadata["sensor_execution"].startswith("clock-only")
    assert case.metadata["sensor_clocks"] == [case.sensor_clocks[0].to_metadata()]
    assert get_next_time_step(case, 0.1) == pytest.approx(0.01)
