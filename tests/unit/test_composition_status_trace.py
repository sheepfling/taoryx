"""Regression coverage for portable committed batch-status traces."""

from __future__ import annotations

from pathlib import Path
from typing import cast

import pytest

from taoryx.composition_resource_ledger import build_committed_resource_ledger, validate_committed_resource_ledger
from taoryx.composition_sensor_trace import BatchTruthSample
from taoryx.composition_status_trace import build_committed_status_trace, validate_committed_status_trace
from taoryx.vehicle_composition import CompiledVehicleComposition, compile_vehicle_composition, load_vehicle_composition_request

ROOT = Path(__file__).resolve().parents[2]


def _composition(name: str) -> CompiledVehicleComposition:
    return compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / name))
    ####


def _x15_sample(time_s: float, *, phase: str) -> BatchTruthSample:
    return BatchTruthSample(
        time_s=time_s,
        execution_status="completed" if phase == "glide" else "active",
        raw_values={
            "time_s": time_s,
            "position_m": [100.0 + time_s, 200.0, 10_000.0 - time_s],
            "velocity_m_s": [500.0, 0.0, -5.0],
            "mass_kg": 10_000.0 - time_s,
            "phase": phase,
            "attitude_rad": [0.1, 0.2, 0.3],
            "attitude_rate_rad_s": [0.01, 0.02, 0.03],
        },
    )
    ####


def test_batch_status_trace_projects_declared_x15_resources_without_interpolation() -> None:
    trace = build_committed_status_trace(
        _composition("x15_staged_booster_reachability_pseudo6dof_compose.yaml"),
        (_x15_sample(0.0, phase="boost"), _x15_sample(25.0, phase="glide")),
    )

    assert trace["schema"] == "taoryx.composition-status-trace/v1alpha1"
    assert trace["sampling"] == "committed_truth_boundary_only"
    channels = trace["channels"]
    assert isinstance(channels, list)
    assert "resources.booster.propellant.consumed" in channels
    samples = cast(list[dict[str, object]], trace["samples"])
    terminal = cast(dict[str, object], samples[-1]["values"])
    assert terminal["resources.booster.attached"] is False
    assert terminal["resources.booster.propellant.consumed"] == pytest.approx(2000.0)
    assert terminal["velocity.speed"] == pytest.approx((500.0**2 + 5.0**2) ** 0.5)
    validate_committed_status_trace(_composition("x15_staged_booster_reachability_pseudo6dof_compose.yaml"), trace)
    ####


def test_batch_status_trace_fails_when_an_advertised_channel_has_no_committed_binding() -> None:
    with pytest.raises(ValueError, match="cannot resolve declared batch channel"):
        build_committed_status_trace(
            _composition("x15_staged_booster_reachability_3dof_compose.yaml"),
            (BatchTruthSample(time_s=0.0, raw_values={}),),
        )
    ####


def test_batch_status_trace_fails_when_a_declared_scalar_has_the_wrong_shape() -> None:
    sample = _x15_sample(0.0, phase="boost")
    raw_values = dict(sample.raw_values)
    raw_values["mass_kg"] = "not-a-mass"

    with pytest.raises(ValueError, match="resources.mass.total.*finite scalar"):
        build_committed_status_trace(
            _composition("x15_staged_booster_reachability_pseudo6dof_compose.yaml"),
            (BatchTruthSample(time_s=sample.time_s, execution_status=sample.execution_status, raw_values=raw_values),),
        )
    ####


def test_batch_status_trace_validator_rejects_a_missing_declared_channel() -> None:
    composition = _composition("x15_staged_booster_reachability_pseudo6dof_compose.yaml")
    trace = build_committed_status_trace(composition, (_x15_sample(0.0, phase="boost"),))
    samples = trace["samples"]
    assert isinstance(samples, list)
    first_sample = samples[0]
    assert isinstance(first_sample, dict)
    values = first_sample["values"]
    assert isinstance(values, dict)
    del values["resources.booster.propellant.consumed"]

    with pytest.raises(ValueError, match="omits declared channel"):
        validate_committed_status_trace(composition, trace)
    ####


def test_batch_status_trace_validator_rejects_a_malformed_declared_value() -> None:
    composition = _composition("x15_staged_booster_reachability_pseudo6dof_compose.yaml")
    trace = build_committed_status_trace(composition, (_x15_sample(0.0, phase="boost"),))
    samples = trace["samples"]
    assert isinstance(samples, list)
    first_sample = samples[0]
    assert isinstance(first_sample, dict)
    values = first_sample["values"]
    assert isinstance(values, dict)
    values["resources.booster.propellant.consumed"] = "not-a-scalar"

    with pytest.raises(ValueError, match="resources.booster.propellant.consumed.*finite scalar"):
        validate_committed_status_trace(composition, trace)
    ####


def test_committed_resource_ledger_copies_declared_samples_without_resource_inference() -> None:
    composition = _composition("x15_staged_booster_reachability_pseudo6dof_compose.yaml")
    status_trace = build_committed_status_trace(
        composition,
        (_x15_sample(0.0, phase="boost"), _x15_sample(25.0, phase="glide")),
    )

    ledger = build_committed_resource_ledger(composition, status_trace)

    assert ledger["schema"] == "taoryx.composition-resource-ledger/v1alpha1"
    assert ledger["sampling"] == "committed_truth_boundary_only"
    summaries = cast(list[dict[str, object]], ledger["summaries"])
    propellant = next(item for item in summaries if item["id"] == "resources.booster.propellant.consumed")
    assert propellant["trend"] == "nondecreasing"
    assert propellant["initial_value"] == pytest.approx(0.0)
    assert propellant["final_value"] == pytest.approx(2000.0)
    validate_committed_resource_ledger(composition, ledger, status_trace=status_trace)
    ####


def test_committed_resource_ledger_rejects_a_sample_detached_from_status_truth() -> None:
    composition = _composition("x15_staged_booster_reachability_pseudo6dof_compose.yaml")
    status_trace = build_committed_status_trace(composition, (_x15_sample(0.0, phase="boost"),))
    ledger = build_committed_resource_ledger(composition, status_trace)
    samples = cast(list[dict[str, object]], ledger["samples"])
    values = cast(dict[str, object], samples[0]["values"])
    values["resources.mass.total"] = -1.0

    with pytest.raises(ValueError, match="disagrees with the composition status trace"):
        validate_committed_resource_ledger(composition, ledger, status_trace=status_trace)
    ####
