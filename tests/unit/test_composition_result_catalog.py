"""Tests for discovery-only normalized Mission Composition result catalogs."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import taoryx.composition_result_catalog as result_catalog
from taoryx.composition_control_trace import BatchControlSample, build_committed_control_trace
from taoryx.composition_result_catalog import (
    build_composition_release_catalog,
    index_composition_results,
    validate_composition_release_catalog,
    write_composition_release_catalog,
)
from taoryx.runtime.cli import main
from taoryx.trajectory.evaluation import EvaluationGate, EvaluationMetric, EvidenceChannel, TrajectoryEvaluation
from taoryx.vehicle_composition import (
    compile_vehicle_composition,
    load_vehicle_composition_request,
    resolve_vehicle_composition_interface_contract,
)
from taoryx.vehicle_execution_bindings import resolve_vehicle_execution_binding
from taoryx.vehicle_execution_preflight import preflight_vehicle_composition

ROOT = Path(__file__).resolve().parents[2]


def _evaluation(identifier: str) -> TrajectoryEvaluation:
    return TrajectoryEvaluation(
        scenario_id=identifier,
        scenario_contract_sha256="a" * 64,
        validity="valid",
        qualification="unqualified",
        feasibility="feasible",
        outcome="completed",
        claim_boundary="result-index fixture",
    )
    ####


def _graph_metadata(composition: object) -> dict[str, object]:
    graph = getattr(composition, "mission_graph")
    assert graph is not None
    return {
        "graph_status": graph.status,
        "entry_instance_id": graph.entry_instance_id,
        "expected_instance_ids": [node.instance_id for node in graph.nodes],
    }
    ####


def test_result_catalog_indexes_valid_evaluations_without_recomputing_them(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    first = tmp_path / "x8" / "evaluation.json"
    first.parent.mkdir()
    first.write_text(json.dumps(_evaluation("x8-fixture").as_dict()), encoding="utf-8")
    (first.parent / "objective_report.json").write_text("{}", encoding="utf-8")

    report = index_composition_results(tmp_path)

    assert report["status"] == "pass"
    assert report["result_count"] == 1
    records = report["records"]
    assert isinstance(records, list)
    assert records[0]["scenario_id"] == "x8-fixture"
    assert records[0]["artifact_paths"]["objective_report.json"] == "x8/objective_report.json"
    assert records[0]["composition_provenance"]["status"] == "missing"
    assert records[0]["graph_execution_evidence"]["status"] == "missing"
    assert records[0]["batch_episode_parity"]["availability"] == "not_available"

    assert main(["vehicle", "results", str(tmp_path)]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["status"] == "pass"
    ####


def test_release_catalog_hashes_valid_packets_without_fabricating_missing_release_evidence(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    packet = tmp_path / "x8"
    packet.mkdir()
    (packet / "evaluation.json").write_text(json.dumps(_evaluation("x8-release").as_dict()), encoding="utf-8")
    (packet / "objective_report.json").write_text("{}", encoding="utf-8")
    (packet / "robustness_report.json").write_text(
        json.dumps({"status": "not_applicable", "claim": "fixture has no robustness campaign"}), encoding="utf-8"
    )
    (packet / "reproduction.txt").write_text("taoryx vehicle run composition.json\n", encoding="utf-8")

    manifest = build_composition_release_catalog(tmp_path)

    assert manifest["status"] == "ready"
    assert manifest["result_packet_count"] == 1
    assert set(manifest["artifact_sha256"]) == {
        "x8/evaluation.json",
        "x8/objective_report.json",
        "x8/reproduction.txt",
        "x8/robustness_report.json",
    }
    coverage = manifest["release_evidence_coverage"]
    assert coverage["robustness_report.json"] == {"present_packet_count": 1, "missing_packet_count": 0}
    assert coverage["convergence_report.json"] == {"present_packet_count": 0, "missing_packet_count": 1}
    packet_evidence = manifest["packets"][0]["release_evidence"]
    assert packet_evidence["robustness_report.json"] == {
        "status": "verified",
        "format": "json",
        "reported_status": "not_applicable",
    }
    assert packet_evidence["reproduction.txt"]["status"] == "verified"
    assert validate_composition_release_catalog(tmp_path, manifest) == ()

    output = tmp_path / "release-catalog.json"
    written = write_composition_release_catalog(tmp_path, output)
    assert output.is_file()
    assert written == manifest
    assert main(["vehicle", "release-catalog", str(tmp_path), "--output", str(output)]) == 0
    assert json.loads(capsys.readouterr().out)["schema"] == "taoryx.vehicle-composition-release-catalog/v1alpha1"

    (packet / "objective_report.json").write_text('{"changed": true}', encoding="utf-8")
    assert validate_composition_release_catalog(tmp_path, manifest) == (
        "release catalog artifact SHA-256 ledger disagrees with current validated packets",
    )
    ####


def test_release_catalog_rejects_tampered_derived_packet_and_coverage_summaries(tmp_path: Path) -> None:
    """Release verification must bind its summary, not only its artifact hashes."""

    packet = tmp_path / "x8"
    packet.mkdir()
    (packet / "evaluation.json").write_text(json.dumps(_evaluation("x8-release-summary").as_dict()), encoding="utf-8")
    (packet / "objective_report.json").write_text("{}", encoding="utf-8")

    manifest = build_composition_release_catalog(tmp_path)
    tampered_packets = {**manifest, "packets": []}
    assert validate_composition_release_catalog(tmp_path, tampered_packets) == (
        "release catalog packets disagrees with current validated packets",
    )

    tampered_coverage = {**manifest, "release_evidence_coverage": {}}
    assert validate_composition_release_catalog(tmp_path, tampered_coverage) == (
        "release catalog release_evidence_coverage disagrees with current validated packets",
    )
    ####


def test_release_catalog_rejects_a_present_but_structurally_invalid_optional_sidecar(tmp_path: Path) -> None:
    packet = tmp_path / "invalid-sidecar"
    packet.mkdir()
    (packet / "evaluation.json").write_text(json.dumps(_evaluation("invalid-sidecar").as_dict()), encoding="utf-8")
    (packet / "robustness_report.json").write_text("{}", encoding="utf-8")

    with pytest.raises(ValueError, match="invalid optional release evidence"):
        build_composition_release_catalog(tmp_path)
    ####


def test_result_catalog_projects_typed_evaluation_evidence_without_inventing_missing_channels(tmp_path: Path) -> None:
    output_directory = tmp_path / "summary"
    output_directory.mkdir()
    evaluation = TrajectoryEvaluation(
        scenario_id="summary-fixture",
        scenario_contract_sha256="b" * 64,
        validity="valid",
        qualification="unqualified",
        feasibility="feasible",
        outcome="partial",
        metrics=(
            EvaluationMetric(
                id="objective.first.normalized_error",
                actual=0.25,
                target=0.0,
                tolerance=1.0,
                normalized_error=0.25,
                unit="1",
                status="pass",
                source="independent_truth_telemetry",
            ),
            EvaluationMetric(
                id="objective.second.normalized_error",
                actual=1.4,
                target=0.0,
                tolerance=1.0,
                normalized_error=1.4,
                unit="1",
                status="fail",
                source="independent_truth_telemetry",
            ),
        ),
        gates=(
            EvaluationGate(id="independent-truth-objectives", status="fail"),
            EvaluationGate(id="envelope", status="pass"),
        ),
        requested_controls=(
            EvidenceChannel(id="propulsion.command.fraction", value=0.6, unit="1", source="requested"),
        ),
        achieved_controls=(
            EvidenceChannel(id="control.physical_effector_allocation", value=False, source="achieved"),
        ),
        claim_boundary="summary-fixture",
    )
    (output_directory / "evaluation.json").write_text(json.dumps(evaluation.as_dict()), encoding="utf-8")

    report = index_composition_results(tmp_path)

    records = report["records"]
    assert isinstance(records, list)
    summary = records[0]["evaluation_summary"]
    assert summary["required_objectives"] == {
        "count": 2,
        "passed": 1,
        "failed": 1,
        "blocked": 0,
        "worst_normalized_error": 1.4,
    }
    assert summary["gates"]["failed"] == 1
    assert summary["evidence_channels"]["requested_controls"] == {
        "count": 1,
        "available": 1,
        "unavailable": 0,
        "invalid": 0,
    }
    assert summary["evidence_channels"]["resources"]["count"] == 0
    assert "does not infer a terminal state" in summary["claim_boundary"]
    ####


def test_result_catalog_retains_invalid_artifacts_as_failures(tmp_path: Path) -> None:
    invalid = tmp_path / "broken" / "evaluation.json"
    invalid.parent.mkdir()
    invalid.write_text("not-json", encoding="utf-8")

    report = index_composition_results(tmp_path)

    assert report["status"] == "fail"
    assert report["valid_result_count"] == 0
    records = report["records"]
    assert isinstance(records, list)
    assert records[0]["status"] == "invalid"
    assert report["errors"]
    ####


def test_result_catalog_rejects_a_colocated_composition_with_a_different_identity(tmp_path: Path) -> None:
    output_directory = tmp_path / "mismatched"
    output_directory.mkdir()
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml")
    )
    evaluation = TrajectoryEvaluation(
        scenario_id="expected-scenario",
        scenario_contract_sha256=composition.identity_sha256,
        validity="valid",
        qualification="unqualified",
        feasibility="feasible",
        outcome="completed",
        claim_boundary="mismatched-composition fixture",
    )
    (output_directory / "evaluation.json").write_text(json.dumps(evaluation.as_dict()), encoding="utf-8")
    (output_directory / "composition.json").write_text(
        json.dumps(composition.model_dump(mode="json", by_alias=True)), encoding="utf-8"
    )

    report = index_composition_results(tmp_path)

    assert report["status"] == "fail"
    records = report["records"]
    assert isinstance(records, list)
    assert records[0]["status"] == "invalid"
    provenance = records[0]["composition_provenance"]
    assert isinstance(provenance, dict)
    assert provenance["status"] == "invalid"
    assert "scenario ID mismatch" in provenance["error"]
    ####


def test_result_catalog_binds_interface_and_variant_provenance_to_the_composition(tmp_path: Path) -> None:
    output_directory = tmp_path / "x8"
    output_directory.mkdir()
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml")
    )
    evaluation = TrajectoryEvaluation(
        scenario_id=composition.id,
        scenario_contract_sha256=composition.identity_sha256,
        validity="valid",
        qualification="unqualified",
        feasibility="feasible",
        outcome="completed",
        claim_boundary="bound-interface fixture",
    )
    interface = resolve_vehicle_composition_interface_contract(composition)
    (output_directory / "evaluation.json").write_text(json.dumps(evaluation.as_dict()), encoding="utf-8")
    (output_directory / "composition.json").write_text(
        json.dumps(composition.model_dump(mode="json", by_alias=True)), encoding="utf-8"
    )
    (output_directory / "vehicle_interface.json").write_text(
        json.dumps({**interface.as_dict(), "validation": {"status": "pass", "findings": []}}),
        encoding="utf-8",
    )
    binding = resolve_vehicle_execution_binding(composition, "batch")
    (output_directory / "reproduction.txt").write_text(
        "\n".join(
            (
                "# TAORYX Mission Composition public batch reproduction record",
                f"# composition_id: {composition.id}",
                f"# composition_identity_sha256: {composition.identity_sha256}",
                f"# execution_factory_id: {binding.factory_id}",
                f"# execution_mode: {binding.execution_mode}",
                "taoryx vehicle run composition.json --output-dir x8",
                "",
            )
        ),
        encoding="utf-8",
    )

    report = index_composition_results(tmp_path)

    assert report["status"] == "pass"
    records = report["records"]
    assert isinstance(records, list)
    record = records[0]
    composition_provenance = record["composition_provenance"]
    assert composition_provenance["family_id"] == "skywalker_x8"
    assert composition_provenance["control_realization"] == "force_model"
    assert composition_provenance["variant"]["qualification"] == "baseline"
    interface_provenance = record["interface_provenance"]
    assert interface_provenance["status"] == "verified"
    assert interface_provenance["interface_id"] == "skywalker_x8/point_mass_3dof"
    assert interface_provenance["execution_bindings"][0]["factory_id"] == "language_backed_powered_fixed_wing.v1"
    assert interface_provenance["execution_bindings"][0]["execution_mode"] == "closed_loop_controller"
    reproduction = record["reproduction_evidence"]
    assert reproduction["status"] == "verified"
    assert reproduction["execution_factory_id"] == "language_backed_powered_fixed_wing.v1"
    release = build_composition_release_catalog(tmp_path)
    identity = release["packets"][0]["execution_identity"]
    assert identity["composition"]["vehicle_id"] == "skywalker_x8"
    assert identity["composition"]["fidelity"] == "point_mass_3dof"
    assert identity["interface"]["execution_bindings"][0]["execution_mode"] == "closed_loop_controller"
    assert identity["capability_preflight"]["status"] == "missing"
    assert identity["reproduction"]["status"] == "verified"
    assert record["graph_execution_evidence"]["status"] == "missing"
    assert record["batch_episode_parity"]["availability"] == "registered"

    (output_directory / "reproduction.txt").write_text(
        (output_directory / "reproduction.txt").read_text(encoding="utf-8").replace(
            f"# composition_identity_sha256: {composition.identity_sha256}",
            "# composition_identity_sha256: " + "0" * 64,
        ),
        encoding="utf-8",
    )
    invalid = index_composition_results(tmp_path)
    assert invalid["status"] == "fail"
    invalid_records = invalid["records"]
    assert isinstance(invalid_records, list)
    assert invalid_records[0]["reproduction_evidence"]["status"] == "invalid"
    ####


def test_result_catalog_binds_concrete_capability_preflight_to_the_composition(tmp_path: Path) -> None:
    output_directory = tmp_path / "x8-capability-preflight"
    output_directory.mkdir()
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml")
    )
    evaluation = TrajectoryEvaluation(
        scenario_id=composition.id,
        scenario_contract_sha256=composition.identity_sha256,
        validity="valid",
        qualification="unqualified",
        feasibility="feasible",
        outcome="completed",
        claim_boundary="capability-preflight fixture",
    )
    preflight = preflight_vehicle_composition(composition)
    assert preflight.status == "translation_ready"
    (output_directory / "evaluation.json").write_text(json.dumps(evaluation.as_dict()), encoding="utf-8")
    (output_directory / "composition.json").write_text(
        json.dumps(composition.model_dump(mode="json", by_alias=True)), encoding="utf-8"
    )
    (output_directory / "preflight.json").write_text(json.dumps(preflight.as_dict()), encoding="utf-8")

    report = index_composition_results(tmp_path)

    assert report["status"] == "pass"
    records = report["records"]
    assert isinstance(records, list)
    evidence = records[0]["capability_preflight_evidence"]
    assert evidence["status"] == "verified"
    assert evidence["adapter_id"] == "taoryx.powered_fixed_wing_racetrack.capability_scaled.v1"
    assert evidence["feasibility"] == "feasible"

    payload = json.loads((output_directory / "preflight.json").read_text(encoding="utf-8"))
    payload["capability_estimate"]["derived_mission_sha256"] = "0" * 64
    (output_directory / "preflight.json").write_text(json.dumps(payload), encoding="utf-8")

    invalid = index_composition_results(tmp_path)

    assert invalid["status"] == "fail"
    invalid_records = invalid["records"]
    assert isinstance(invalid_records, list)
    assert invalid_records[0]["capability_preflight_evidence"]["status"] == "invalid"
    ####


def test_result_catalog_binds_graph_execution_evidence_to_the_compiled_composition(tmp_path: Path) -> None:
    output_directory = tmp_path / "observed-graph"
    output_directory.mkdir()
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml")
    )
    evaluation = TrajectoryEvaluation(
        scenario_id=composition.id,
        scenario_contract_sha256=composition.identity_sha256,
        validity="valid",
        qualification="unqualified",
        feasibility="feasible",
        outcome="completed",
        claim_boundary="graph-evidence fixture",
    )
    (output_directory / "evaluation.json").write_text(json.dumps(evaluation.as_dict()), encoding="utf-8")
    (output_directory / "composition.json").write_text(
        json.dumps(composition.model_dump(mode="json", by_alias=True)), encoding="utf-8"
    )
    (output_directory / "mission_graph_execution.json").write_text(
        json.dumps(
            {
                "schema": "taoryx.mission-graph-execution/v1alpha1",
                "composition_id": composition.id,
                "composition_identity_sha256": composition.identity_sha256,
                **_graph_metadata(composition),
                "observation_status": "unobserved",
                "dispatches": [],
                "completed_nominal_success_path": None,
                "unobserved_reason": "fixture does not model controller graph dispatch",
            }
        ),
        encoding="utf-8",
    )

    report = index_composition_results(tmp_path)

    assert report["status"] == "pass"
    records = report["records"]
    assert isinstance(records, list)
    graph_evidence = records[0]["graph_execution_evidence"]
    assert graph_evidence["status"] == "verified"
    assert graph_evidence["observation_status"] == "unobserved"
    assert graph_evidence["completed_nominal_success_path"] is None
    ####


def test_result_catalog_rejects_graph_execution_evidence_from_another_composition(tmp_path: Path) -> None:
    output_directory = tmp_path / "wrong-graph"
    output_directory.mkdir()
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml")
    )
    evaluation = TrajectoryEvaluation(
        scenario_id=composition.id,
        scenario_contract_sha256=composition.identity_sha256,
        validity="valid",
        qualification="unqualified",
        feasibility="feasible",
        outcome="completed",
        claim_boundary="wrong-graph fixture",
    )
    (output_directory / "evaluation.json").write_text(json.dumps(evaluation.as_dict()), encoding="utf-8")
    (output_directory / "composition.json").write_text(
        json.dumps(composition.model_dump(mode="json", by_alias=True)), encoding="utf-8"
    )
    (output_directory / "mission_graph_execution.json").write_text(
        json.dumps(
            {
                "schema": "taoryx.mission-graph-execution/v1alpha1",
                "composition_id": composition.id,
                "composition_identity_sha256": "0" * 64,
                **_graph_metadata(composition),
                "observation_status": "unobserved",
                "dispatches": [],
                "completed_nominal_success_path": None,
                "unobserved_reason": "fixture does not model controller graph dispatch",
            }
        ),
        encoding="utf-8",
    )

    report = index_composition_results(tmp_path)

    assert report["status"] == "fail"
    records = report["records"]
    assert isinstance(records, list)
    assert records[0]["graph_execution_evidence"]["status"] == "invalid"
    ####


def test_result_catalog_rejects_observed_dispatch_that_is_not_a_declared_graph_edge(tmp_path: Path) -> None:
    """A retained graph artifact cannot rewrite the selected transition semantics."""

    output_directory = tmp_path / "wrong-graph-dispatch"
    output_directory.mkdir()
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml")
    )
    graph = composition.mission_graph
    assert graph is not None
    evaluation = TrajectoryEvaluation(
        scenario_id=composition.id,
        scenario_contract_sha256=composition.identity_sha256,
        validity="valid",
        qualification="unqualified",
        feasibility="feasible",
        outcome="completed",
        claim_boundary="graph-dispatch fixture",
    )
    dispatches = []
    for index, node in enumerate(graph.nodes):
        next_instance_id = None if index + 1 == len(graph.nodes) else graph.nodes[index + 1].instance_id
        dispatches.append(
            {
                "instance_id": node.instance_id,
                "segment_id": node.segment_id,
                "outcome": "success",
                "committed_time_s": float(index + 1),
                "transition_status": "declared_terminal" if next_instance_id is None else "transitioned",
                "next_instance_id": next_instance_id,
                "state_transfer": None if next_instance_id is None else "previous_terminal_truth_state",
            }
        )
    dispatches[0]["transition_status"] = "declared_terminal"
    (output_directory / "evaluation.json").write_text(json.dumps(evaluation.as_dict()), encoding="utf-8")
    (output_directory / "composition.json").write_text(
        json.dumps(composition.model_dump(mode="json", by_alias=True)), encoding="utf-8"
    )
    (output_directory / "mission_graph_execution.json").write_text(
        json.dumps(
            {
                "schema": "taoryx.mission-graph-execution/v1alpha1",
                "composition_id": composition.id,
                "composition_identity_sha256": composition.identity_sha256,
                **_graph_metadata(composition),
                "observation_status": "observed",
                "executed_instance_ids": [item["instance_id"] for item in dispatches],
                "dispatches": dispatches,
                "completed_nominal_success_path": True,
                "unobserved_reason": None,
            }
        ),
        encoding="utf-8",
    )

    report = index_composition_results(tmp_path)

    assert report["status"] == "fail"
    records = report["records"]
    assert isinstance(records, list)
    evidence = records[0]["graph_execution_evidence"]
    assert evidence["status"] == "invalid"
    assert "disagrees with its declared" in str(evidence["error"])
    ####


def test_result_catalog_rejects_variant_runtime_evidence_from_another_composition(tmp_path: Path) -> None:
    output_directory = tmp_path / "wrong-variant-runtime"
    output_directory.mkdir()
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/a320_racetrack_mass_variant_3dof_compose.yaml")
    )
    evaluation = TrajectoryEvaluation(
        scenario_id=composition.id,
        scenario_contract_sha256=composition.identity_sha256,
        validity="valid",
        qualification="unqualified",
        feasibility="feasible",
        outcome="completed",
        claim_boundary="wrong-variant-runtime fixture",
    )
    (output_directory / "evaluation.json").write_text(json.dumps(evaluation.as_dict()), encoding="utf-8")
    (output_directory / "composition.json").write_text(
        json.dumps(composition.model_dump(mode="json", by_alias=True)), encoding="utf-8"
    )
    (output_directory / "variant_runtime_evidence.json").write_text(
        json.dumps(
            {
                "schema": "taoryx.variant-runtime-evidence/v1alpha1",
                "composition_id": composition.id,
                "composition_identity_sha256": "0" * 64,
                "status": "pass",
                "bindings": [{"id": "operating_mass_kg"}],
            }
        ),
        encoding="utf-8",
    )

    report = index_composition_results(tmp_path)

    assert report["status"] == "fail"
    records = report["records"]
    assert isinstance(records, list)
    assert records[0]["variant_runtime_evidence"]["status"] == "invalid"
    ####


def test_result_catalog_rejects_semantic_action_trace_from_another_composition(tmp_path: Path) -> None:
    output_directory = tmp_path / "wrong-action-trace"
    output_directory.mkdir()
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml")
    )
    evaluation = TrajectoryEvaluation(
        scenario_id=composition.id,
        scenario_contract_sha256=composition.identity_sha256,
        validity="valid",
        qualification="unqualified",
        feasibility="feasible",
        outcome="completed",
        claim_boundary="wrong-action-trace fixture",
    )
    trace = build_committed_control_trace(
        composition,
        (
            BatchControlSample(
                0.0,
                0.1,
                {
                    "propulsion.command.fraction": 0.5,
                    "control.longitudinal.bridge.command": 0.0,
                    "control.lateral.bridge.command": 0.0,
                },
                {},
            ),
        ),
    )
    trace["composition_identity_sha256"] = "0" * 64
    (output_directory / "evaluation.json").write_text(json.dumps(evaluation.as_dict()), encoding="utf-8")
    (output_directory / "composition.json").write_text(
        json.dumps(composition.model_dump(mode="json", by_alias=True)), encoding="utf-8"
    )
    (output_directory / "semantic_action_trace.json").write_text(json.dumps(trace), encoding="utf-8")

    report = index_composition_results(tmp_path)

    assert report["status"] == "fail"
    records = report["records"]
    assert isinstance(records, list)
    assert records[0]["semantic_action_trace_evidence"]["status"] == "invalid"
    ####


def test_result_catalog_rejects_trace_when_the_selected_binding_does_not_declare_it(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """An optional artifact cannot upgrade an endpoint that did not earn it."""

    output_directory = tmp_path / "unexpected-action-trace"
    output_directory.mkdir()
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml")
    )
    evaluation = TrajectoryEvaluation(
        scenario_id=composition.id,
        scenario_contract_sha256=composition.identity_sha256,
        validity="valid",
        qualification="unqualified",
        feasibility="feasible",
        outcome="completed",
        claim_boundary="unexpected-action-trace fixture",
    )
    trace = build_committed_control_trace(
        composition,
        (
            BatchControlSample(
                0.0,
                0.1,
                {
                    "propulsion.command.fraction": 0.5,
                    "control.longitudinal.bridge.command": 0.0,
                    "control.lateral.bridge.command": 0.0,
                },
                {},
            ),
        ),
    )
    (output_directory / "evaluation.json").write_text(json.dumps(evaluation.as_dict()), encoding="utf-8")
    (output_directory / "composition.json").write_text(
        json.dumps(composition.model_dump(mode="json", by_alias=True)), encoding="utf-8"
    )
    (output_directory / "semantic_action_trace.json").write_text(json.dumps(trace), encoding="utf-8")
    monkeypatch.setattr(
        result_catalog,
        "resolve_vehicle_execution_binding",
        lambda _composition, _operation: SimpleNamespace(batch_action_trace="not_emitted"),
    )

    report = index_composition_results(tmp_path)

    assert report["status"] == "fail"
    records = report["records"]
    assert isinstance(records, list)
    evidence = records[0]["semantic_action_trace_evidence"]
    assert evidence["status"] == "invalid"
    assert "declares 'not_emitted'" in evidence["error"]
    ####


def test_result_catalog_rejects_interface_sidecar_with_wrong_fidelity(tmp_path: Path) -> None:
    output_directory = tmp_path / "wrong-interface"
    output_directory.mkdir()
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml")
    )
    evaluation = TrajectoryEvaluation(
        scenario_id=composition.id,
        scenario_contract_sha256=composition.identity_sha256,
        validity="valid",
        qualification="unqualified",
        feasibility="feasible",
        outcome="completed",
        claim_boundary="wrong-interface fixture",
    )
    (output_directory / "evaluation.json").write_text(json.dumps(evaluation.as_dict()), encoding="utf-8")
    (output_directory / "composition.json").write_text(
        json.dumps(composition.model_dump(mode="json", by_alias=True)), encoding="utf-8"
    )
    (output_directory / "vehicle_interface.json").write_text(
        json.dumps(
            {
                "interface_id": "skywalker_x8/pseudo_6dof",
                "fingerprint_sha256": "a" * 64,
                "family_id": "skywalker_x8",
                "fidelity": "pseudo_6dof",
                "control_realization": "response_law",
                "execution_records": [],
                "validation": {"status": "pass", "findings": []},
            }
        ),
        encoding="utf-8",
    )

    report = index_composition_results(tmp_path)

    assert report["status"] == "fail"
    records = report["records"]
    assert isinstance(records, list)
    assert records[0]["interface_provenance"]["status"] == "invalid"
    assert "fidelity mismatch" in records[0]["error"]
    ####


def test_result_catalog_indexes_local_direct_wrench_screen_without_promoting_a_mission(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    output_directory = tmp_path / "x15-local-screen"
    output_directory.mkdir()
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x15_local_direct_wrench_screen_compose.yaml")
    )
    (output_directory / "composition.json").write_text(
        json.dumps(composition.model_dump(mode="json", by_alias=True)), encoding="utf-8"
    )
    (output_directory / "preflight.json").write_text(
        json.dumps(preflight_vehicle_composition(composition).as_dict()), encoding="utf-8"
    )
    (output_directory / "local_screen.json").write_text(
        json.dumps(
            {
                "schema": "taoryx.local-direct-wrench-screen/v1alpha1",
                "id": "x15-source-release-glide-local-direct-wrench-v1",
                "plant_id": "x15-source-direct-wrench-local-plant",
                "fidelity": "rigid_body_6dof_direct_wrench",
                "control_realization": "direct_wrench",
                "physical_effector_allocation": False,
                "evaluation": {"mission_pass": True},
            }
        ),
        encoding="utf-8",
    )

    report = index_composition_results(tmp_path)

    assert report["status"] == "pass"
    assert report["result_count"] == 1
    assert report["mission_result_count"] == 0
    assert report["local_controller_screen_count"] == 1
    records = report["records"]
    assert isinstance(records, list)
    record = records[0]
    assert record["record_kind"] == "local_controller_screen"
    assert record["outcome"] == "local_screen_pass"
    assert record["composition_provenance"]["status"] == "verified"
    assert record["capability_preflight_evidence"]["status"] == "verified"
    assert record["graph_execution_evidence"]["status"] == "missing"
    assert record["batch_episode_parity"]["availability"] == "registered"
    assert "does not establish an X-15 flight mission" in record["batch_episode_parity"]["claim_boundary"]
    assert "not a mission completion" in record["claim_boundary"]

    assert main(["vehicle", "result", str(output_directory)]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["record_kind"] == "local_controller_screen"
    assert payload["result"]["outcome"] == "local_screen_pass"
    assert "not a normalized mission" in payload["claim_boundary"]
    ####


def test_result_catalog_rejects_local_screen_without_a_composition_sidecar(tmp_path: Path) -> None:
    screen = tmp_path / "local_screen.json"
    screen.write_text(
        json.dumps(
            {
                "schema": "taoryx.local-direct-wrench-screen/v1alpha1",
                "id": "screen",
                "plant_id": "plant",
                "fidelity": "rigid_body_6dof_direct_wrench",
                "control_realization": "direct_wrench",
                "physical_effector_allocation": False,
                "evaluation": {"mission_pass": True},
            }
        ),
        encoding="utf-8",
    )

    report = index_composition_results(tmp_path)

    assert report["status"] == "fail"
    records = report["records"]
    assert isinstance(records, list)
    assert records[0]["record_kind"] == "local_controller_screen"
    assert records[0]["status"] == "invalid"
    assert "composition provenance" in records[0]["error"]
    ####
