"""Tests for adapter-consumption evidence on selected composition variants."""

from __future__ import annotations

from pathlib import Path

from taoryx.variant_runtime_evidence import build_variant_runtime_evidence
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request

ROOT = Path(__file__).resolve().parents[2]


def _composition(name: str):
    return compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / name))
    ####


def test_adapter_consumption_and_committed_status_relation_must_both_match() -> None:
    composition = _composition("a320_racetrack_mass_variant_3dof_compose.yaml")
    trace = {
        "schema": "taoryx.composition-status-trace/v1alpha1",
        "samples": [
            {"time_s": 0.0, "values": {"resources.mass.total": 66000.0}},
            {"time_s": 1.0, "values": {"resources.mass.total": 66000.0}},
        ],
    }

    passed = build_variant_runtime_evidence(
        composition,
        trace,
        consumed_native_inputs={"A320OpenAPOperatingPoint.mass_kg": 66000.0},
    )
    failed = build_variant_runtime_evidence(
        composition,
        trace,
        consumed_native_inputs={"A320OpenAPOperatingPoint.mass_kg": 65000.0},
    )

    assert passed["status"] == "pass"
    assert failed["status"] == "fail"
    bindings = failed["bindings"]
    assert isinstance(bindings, list)
    assert bindings[0]["consumed_native_input_match"] is False
    ####


def test_nonvariant_composition_is_explicitly_not_applicable() -> None:
    composition = _composition("x8_racetrack_compose.yaml")
    evidence = build_variant_runtime_evidence(composition, {"samples": [{"values": {}}]}, consumed_native_inputs={})

    assert evidence["status"] == "not_applicable"
    assert evidence["bindings"] == []
    ####
