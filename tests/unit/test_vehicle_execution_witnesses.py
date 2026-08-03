"""Regression coverage for the public composition endpoint witness matrix."""

from __future__ import annotations

from taoryx.vehicle_execution_bindings import load_vehicle_execution_binding_catalog
from taoryx.vehicle_execution_witnesses import (
    load_vehicle_execution_witness_catalog,
    validate_vehicle_execution_witnesses,
)


def test_every_runnable_execution_binding_has_a_checked_in_compilation_witness() -> None:
    catalog = load_vehicle_execution_witness_catalog()
    report = validate_vehicle_execution_witnesses(catalog)
    binding_catalog = load_vehicle_execution_binding_catalog()
    runnable_count = sum(binding.status == "runnable" for binding in binding_catalog.bindings)

    assert report["status"] == "pass"
    assert report["runnable_binding_count"] == runnable_count
    assert report["witness_count"] == runnable_count
    records = report["records"]
    assert isinstance(records, list)
    assert any(
        item["family_id"] == "x15"
        and item["mission"] == "x15_local_direct_wrench_screen_v1"
        and item["fidelity"] == "rigid_body_6dof_direct_wrench"
        and item["operation"] == "batch"
        for item in records
    )
    assert all(item["preflight_status"] == "translation_ready" for item in records)
    assert all(
        item["episode_opened"] is True
        for item in records
        if item["operation"] == "episode"
    )
    ####


def test_execution_witness_gate_fails_closed_when_an_advertised_endpoint_is_missing() -> None:
    catalog = load_vehicle_execution_witness_catalog()
    incomplete = catalog.model_copy(update={"witnesses": catalog.witnesses[:1]})

    report = validate_vehicle_execution_witnesses(incomplete)

    assert report["status"] == "fail"
    errors = report["errors"]
    assert isinstance(errors, list)
    assert any("b747/powered_fixed_wing_racetrack_v1/point_mass_3dof/batch" in item for item in errors)
    ####
