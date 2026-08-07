"""Regression coverage for the public composition endpoint witness matrix."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

import taoryx.composition_result_catalog as result_catalog_module
import taoryx.vehicle_execution_witnesses as witness_module
from taoryx.vehicle_composition import (
    compile_vehicle_composition,
    load_vehicle_composition_request,
)
from taoryx.vehicle_execution_bindings import (
    load_vehicle_execution_binding_catalog,
    resolve_vehicle_execution_binding,
)
from taoryx.vehicle_execution_witnesses import (
    _execute_batch_witness,
    _validate_batch_action_trace_artifact,
    _validate_batch_release_packet,
    _validate_batch_reproduction_artifact,
    _validate_batch_resource_ledger_artifact,
    _validate_batch_result_catalog,
    load_vehicle_execution_witness_catalog,
    validate_vehicle_execution_witnesses,
    validate_vehicle_graph_extension_witnesses,
    validate_vehicle_variant_witnesses,
)

ROOT = Path(__file__).resolve().parents[2]


def test_every_runnable_execution_binding_has_a_checked_in_compilation_witness() -> None:
    catalog = load_vehicle_execution_witness_catalog()
    report = validate_vehicle_execution_witnesses(catalog)
    binding_catalog = load_vehicle_execution_binding_catalog()
    runnable_count = sum(binding.status == "runnable" for binding in binding_catalog.bindings)

    assert report["status"] == "pass"
    assert report["runnable_binding_count"] == runnable_count
    assert report["witness_count"] == runnable_count
    assert report["runnable_variant_count"] == 2
    assert report["variant_witness_count"] == 2
    assert report["graph_extension_witness_count"] == 1
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
    capability_preflights = [item["capability_preflight"] for item in records]
    assert all(isinstance(item, dict) for item in capability_preflights)
    assert all(item["adapter_id"] for item in capability_preflights)
    assert all(item["derived_mission_sha256"] for item in capability_preflights)
    assert all(item["lowering_status"] in {"adapter_bound", "factory_bound"} for item in records)
    assert all(
        item["batch_action_trace_disposition"]
        in {
            "emits_committed_interval_trace",
            "not_emitted",
            "not_applicable",
            "planned",
        }
        for item in records
    )
    assert all(item["episode_opened"] is True for item in records if item["operation"] == "episode")
    episode_contracts = [item["episode_contract"] for item in records if item["operation"] == "episode"]
    assert all(isinstance(item, dict) and item["status"] == "pass" for item in episode_contracts)
    variant_records = report["variant_records"]
    assert isinstance(variant_records, list)
    assert {item["family_id"] for item in variant_records} == {"a320_openap_3dof", "hummingbird"}
    assert all(item["preflight_status"] == "translation_ready" for item in variant_records)
    assert all(isinstance(item["capability_preflight"], dict) for item in variant_records)
    assert all(item["lowering_status"] == "factory_bound" for item in variant_records)
    ####


def test_family_graph_extension_witness_is_composed_without_borrowing_a_generic_branch() -> None:
    report = validate_vehicle_graph_extension_witnesses()

    assert report["status"] == "pass"
    assert report["graph_extension_witness_count"] == 1
    records = report["records"]
    assert isinstance(records, list)
    assert len(records) == 1
    record = records[0]
    assert record["id"] == "hummingbird-timeout-recovery-graph-pseudo6dof"
    assert record["family_id"] == "hummingbird"
    assert record["mission"] == "multirotor_pad_box_yaw_recovery_land_v1"
    assert record["fidelity"] == "pseudo_6dof"
    assert record["graph_status"] == "authored_graph_not_lowered"
    assert record["required_transition_kind"] == "timeout"
    assert record["supported_transition_kinds"] == ["success", "timeout"]
    assert record["preflight_status"] == "translation_ready"
    assert record["lowering_status"] == "factory_bound"
    assert record["batch_factory_id"] == "hummingbird_aggregate_thrust_pseudo_batch.v1"
    assert record["batch_execution"] is None
    capability = record["capability_preflight"]
    assert isinstance(capability, dict)
    assert capability["adapter_id"] == "taoryx.multirotor_hover_translation.capability.v1"
    ####


def test_graph_extension_witness_fails_closed_for_an_undeclared_branch() -> None:
    catalog = load_vehicle_execution_witness_catalog()
    witness = catalog.graph_extension_witnesses[0].model_copy(
        update={"required_transition_kind": "abort"}
    )

    report = validate_vehicle_graph_extension_witnesses(
        catalog.model_copy(update={"graph_extension_witnesses": (witness,)})
    )

    assert report["status"] == "fail"
    errors = report["errors"]
    assert isinstance(errors, list)
    assert any("does not declare required 'abort' transition support" in item for item in errors)
    assert any("has no declared 'abort' transition" in item for item in errors)
    ####


def test_graph_extension_witness_public_batch_run_retains_observed_dispatches() -> None:
    report = validate_vehicle_graph_extension_witnesses(execute_batch=True)

    assert report["status"] == "pass"
    records = report["records"]
    assert isinstance(records, list)
    execution = records[0]["batch_execution"]
    assert isinstance(execution, dict)
    assert execution["status"] == "pass"
    graph_execution = execution["graph_execution"]
    assert isinstance(graph_execution, dict)
    assert graph_execution["observation_status"] == "observed"
    assert graph_execution["graph_status"] == "authored_graph_not_lowered"
    assert graph_execution["dispatch_count"] == 7
    assert graph_execution["outcomes"] == ["success"] * 7
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


def test_execution_witness_gate_fails_closed_when_a_runnable_variant_has_no_witness() -> None:
    catalog = load_vehicle_execution_witness_catalog()
    incomplete = catalog.model_copy(update={"variant_witnesses": catalog.variant_witnesses[:1]})

    report = validate_vehicle_variant_witnesses(incomplete)

    assert report["status"] == "fail"
    errors = report["errors"]
    assert isinstance(errors, list)
    assert any("hummingbird/grounded_operating_mass_kg" in item for item in errors)
    ####


def test_runtime_variant_witnesses_are_composed_and_lowered_without_running_a_vehicle() -> None:
    report = validate_vehicle_variant_witnesses()

    assert report["status"] == "pass"
    assert report["runnable_variant_count"] == 2
    assert report["variant_witness_count"] == 2
    records = report["records"]
    assert isinstance(records, list)
    assert {item["family_id"] for item in records} == {"a320_openap_3dof", "hummingbird"}
    assert all(item["preflight_status"] == "translation_ready" for item in records)
    assert all(isinstance(item["capability_preflight"], dict) for item in records)
    assert all(item["lowering_status"] == "factory_bound" for item in records)
    assert {item["family_id"]: item["batch_factory_id"] for item in records} == {
        "a320_openap_3dof": "reduced_fixed_wing_openap.v1",
        "hummingbird": "hummingbird_aggregate_thrust_pseudo_batch.v1",
    }
    json.dumps(report)
    ####


def test_variant_witness_batch_smoke_uses_the_exact_public_binding(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[tuple[str, str, str]] = []

    def fake_batch_witness(
        witness_id: str,
        composition: object,
        *,
        binding: object,
    ) -> dict[str, object]:
        family_id = str(getattr(composition, "family_id"))
        factory_id = str(getattr(binding, "factory_id"))
        calls.append((witness_id, family_id, factory_id))
        return {"status": "pass", "detail": "fixture batch execution"}
        ####

    monkeypatch.setattr(witness_module, "_execute_batch_witness", fake_batch_witness)

    report = validate_vehicle_variant_witnesses(execute_batch=True)

    assert report["status"] == "pass"
    assert report["batch_execution_smoke"] is True
    assert {(identifier, family_id) for identifier, family_id, _factory in calls} == {
        ("a320-operating-mass-3dof", "a320_openap_3dof"),
        ("hummingbird-grounded-mass-pseudo6dof", "hummingbird"),
    }
    records = report["records"]
    assert isinstance(records, list)
    assert all(item["batch_execution"] == {"status": "pass", "detail": "fixture batch execution"} for item in records)
    ####


def test_runtime_variant_witness_batch_smoke_validates_consumption_packets() -> None:
    report = validate_vehicle_variant_witnesses(execute_batch=True)

    assert report["status"] == "pass"
    assert report["batch_execution_smoke"] is True
    records = report["records"]
    assert isinstance(records, list)
    for record in records:
        execution = record["batch_execution"]
        assert isinstance(execution, dict)
        assert execution["status"] == "pass"
        normalized = execution["result_catalog"]
        assert isinstance(normalized, dict)
        assert normalized["status"] == "pass"
        assert normalized["record_kind"] == "mission_evaluation"
    ####


def test_batch_witness_smoke_validates_its_declared_committed_action_trace() -> None:
    """A promoted endpoint cannot pass its smoke without its trace artifact."""

    composition = compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml"))
    binding = resolve_vehicle_execution_binding(composition, "batch")

    report = _execute_batch_witness("x8-action-trace", composition, binding=binding)

    assert report["status"] == "pass"
    evidence = report["action_trace"]
    assert isinstance(evidence, dict)
    assert evidence == {
        "status": "pass",
        "disposition": "emits_committed_interval_trace",
        "artifact": "semantic_action_trace.json",
    }
    normalized = report["result_catalog"]
    assert isinstance(normalized, dict)
    assert normalized["status"] == "pass"
    assert normalized["record_kind"] == "mission_evaluation"
    assert normalized["graph_observation_status"] == "unobserved"
    reproduction = report["reproduction"]
    assert isinstance(reproduction, dict)
    assert reproduction["status"] == "pass"
    assert "taoryx vehicle run" in str(reproduction["command"])
    release_packet = report["release_packet"]
    assert isinstance(release_packet, dict)
    assert release_packet["status"] == "pass"
    assert release_packet["reproduction_identity"] == "verified"
    ####


def test_batch_witness_smoke_evaluates_a_local_controller_screen_by_its_declared_screen_contract() -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x15_local_direct_wrench_screen_compose.yaml")
    )
    binding = resolve_vehicle_execution_binding(composition, "batch")

    report = _execute_batch_witness("x15-local-screen", composition, binding=binding)

    assert report["status"] == "pass"
    assert report["mode"] == "local_controller_screen"
    assert report["screen_pass"] is True
    action_trace = report["action_trace"]
    assert isinstance(action_trace, dict)
    assert action_trace["status"] == "pass"
    normalized = report["result_catalog"]
    assert isinstance(normalized, dict)
    assert normalized["record_kind"] == "local_controller_screen"
    reproduction = report["reproduction"]
    assert isinstance(reproduction, dict)
    assert reproduction["status"] == "pass"
    release_packet = report["release_packet"]
    assert isinstance(release_packet, dict)
    assert release_packet["status"] == "pass"
    ####


def test_batch_witness_action_trace_gate_fails_closed_for_a_missing_promoted_artifact(tmp_path: Path) -> None:
    """Registry promotion cannot survive a missing or substituted trace."""

    composition = compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml"))

    report = _validate_batch_action_trace_artifact(
        composition,
        tmp_path,
        "emits_committed_interval_trace",
    )

    assert report["status"] == "fail"
    assert "omitted semantic_action_trace.json" in report["detail"]
    ####


def test_batch_resource_ledger_gate_fails_closed_for_a_missing_artifact(tmp_path: Path) -> None:
    composition = compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml"))

    report = _validate_batch_resource_ledger_artifact(composition, tmp_path)

    assert report["status"] == "fail"
    assert "omitted resource_ledger.json" in report["detail"]
    ####


def test_batch_reproduction_gate_fails_closed_for_missing_or_mismatched_identity(tmp_path: Path) -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml")
    )
    binding = resolve_vehicle_execution_binding(composition, "batch")

    missing = _validate_batch_reproduction_artifact(composition, tmp_path, binding=binding)

    assert missing["status"] == "fail"
    assert "omitted reproduction.txt" in missing["detail"]

    (tmp_path / "reproduction.txt").write_text(
        "\n".join(
            (
                "# TAORYX Mission Composition public batch reproduction record",
                f"# composition_id: {composition.id}",
                "# composition_identity_sha256: " + "0" * 64,
                f"# execution_factory_id: {binding.factory_id}",
                f"# execution_mode: {binding.execution_mode}",
                "taoryx vehicle run composition.json --output-dir execution",
                "",
            )
        ),
        encoding="utf-8",
    )

    mismatched = _validate_batch_reproduction_artifact(composition, tmp_path, binding=binding)

    assert mismatched["status"] == "fail"
    assert "composition_identity_sha256" in mismatched["detail"]
    ####


def test_batch_release_packet_gate_fails_closed_without_a_normalized_packet(tmp_path: Path) -> None:
    report = _validate_batch_release_packet(tmp_path / "execution")

    assert report["status"] == "fail"
    assert "not release-catalog ready" in report["detail"]
    ####


def test_batch_result_catalog_gate_fails_closed_without_a_normalized_packet(tmp_path: Path) -> None:
    composition = compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml"))
    binding = resolve_vehicle_execution_binding(composition, "batch")

    report = _validate_batch_result_catalog(tmp_path / "execution", binding)

    assert report["status"] == "fail"
    assert "failed normalized result-catalog validation" in report["detail"]
    ####


def test_batch_result_catalog_gate_requires_verified_graph_execution_evidence(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A valid normalized result cannot hide absent graph-transition evidence."""

    composition = compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml"))
    binding = resolve_vehicle_execution_binding(composition, "batch")
    monkeypatch.setattr(
        result_catalog_module,
        "index_composition_results",
        lambda _directory: {
            "status": "pass",
            "records": [
                {
                    "record_kind": "mission_evaluation",
                    "status": "valid",
                    "outcome": "completed",
                    "graph_execution_evidence": {"status": "missing"},
                }
            ],
        },
    )

    report = _validate_batch_result_catalog(tmp_path / "execution", binding)

    assert report["status"] == "fail"
    assert "mission-graph execution evidence" in report["detail"]
    ####
