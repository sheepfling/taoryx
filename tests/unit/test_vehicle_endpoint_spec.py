"""Focused regression coverage for typed vehicle endpoint contracts."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from taoryx.composition_result_catalog import _runtime_tuning_binding, index_composition_results
from taoryx.runtime.cli import main
from taoryx.vehicle_endpoint_spec import (
    VehicleEndpointSpec,
    _build_endpoint_acceptance_gates,
    _resolve_tuning_binding,
    _verify_disturbance_or_mass_evidence,
    _verify_emitted_output_contract,
    _verify_output_contract,
    load_vehicle_endpoint_spec_catalog,
    vehicle_endpoint_spec_list,
    verify_vehicle_endpoint,
)

_ENDPOINT_IDS = (
    "hummingbird-source-hover-rotor-lqi",
    "hummingbird-source-vertical-translation-rotor-lqi",
    "hummingbird-source-horizontal-translation-rotor-lqi",
    "x8-source-surface-roll-pitch-lqi",
    "b747-condition3-source-surface-attitude-rate-lqi",
    "a320-pseudo6dof-native-coordinate-lqi",
    "nesc-source-history-replay-pseudo6dof",
    "tumbling-body-passive-release-pseudo6dof",
    "f16-source-node-surface-lqr-schedule-transition",
    "f16-source-node-surface-lqr-schedule-interior",
    "f16-source-trim-surface-velocity-lqi",
    "f16-source-node-surface-velocity-lqi-schedule-interior",
    "x15-source-release-direct-wrench-lqr",
    "x15-source-surface-attitude-rate-lqi",
    "hl20-source-surface-attitude-rate-lqi",
)


def test_endpoint_catalog_lists_the_selected_control_endpoints() -> None:
    catalog = load_vehicle_endpoint_spec_catalog()

    assert tuple(item.id for item in catalog.endpoints) == _ENDPOINT_IDS
    assert {item.execution_mode for item in catalog.endpoints} == {
        "closed_loop_controller",
        "local_direct_wrench_screen",
        "local_native_coordinate_lqi_screen",
        "source_history_replay",
        "passive_uncontrolled",
    }
    controller_endpoints = [item for item in catalog.endpoints if item.controller_campaign_id is not None]
    controller_free_endpoints = [item for item in catalog.endpoints if item.controller_campaign_id is None]
    assert all(item.required_tuning_operations for item in controller_endpoints)
    assert all(not item.required_tuning_operations for item in controller_free_endpoints)
    x8 = catalog.endpoint("x8-source-surface-roll-pitch-lqi")
    assert x8.required_tuning_operations == ("state_derivative", "trim", "linearize")
    hummingbird = catalog.endpoint("hummingbird-source-hover-rotor-lqi")
    assert hummingbird.provider_ids == (
        "taoryx.registry.mission-composition",
        "taoryx.hummingbird.mission-composition",
    )
    required_robustness = [item for item in catalog.endpoints if item.robustness_requirement.disposition == "required"]
    not_applicable_robustness = [
        item for item in catalog.endpoints if item.robustness_requirement.disposition == "not_applicable"
    ]
    assert all(item.robustness_screens for item in required_robustness)
    assert all(screen.cases and screen.metrics for item in required_robustness for screen in item.robustness_screens)
    assert [item.id for item in not_applicable_robustness] == [
        "hummingbird-source-horizontal-translation-rotor-lqi",
        "a320-pseudo6dof-native-coordinate-lqi",
        "nesc-source-history-replay-pseudo6dof",
        "tumbling-body-passive-release-pseudo6dof",
        "f16-source-node-surface-lqr-schedule-interior",
        "f16-source-trim-surface-velocity-lqi",
        "f16-source-node-surface-velocity-lqi-schedule-interior",
        "x15-source-release-direct-wrench-lqr",
    ]
    assert all(not item.robustness_screens and item.robustness_requirement.reason for item in not_applicable_robustness)
    listed = vehicle_endpoint_spec_list()
    listed_endpoints = listed["endpoints"]
    assert isinstance(listed_endpoints, list)
    assert [item["id"] for item in listed_endpoints if isinstance(item, dict)] == list(_ENDPOINT_IDS)
    ####


def test_endpoint_rejects_an_unknown_campaign_operation() -> None:
    """Campaign prerequisites must use the shared adapter-operation vocabulary."""

    endpoint = load_vehicle_endpoint_spec_catalog().endpoint("x8-source-surface-roll-pitch-lqi")
    payload = endpoint.model_dump(mode="json")
    payload["required_tuning_operations"] = ["state_derivative", "unknown_operation"]

    with pytest.raises(ValueError, match="unknown_operation"):
        VehicleEndpointSpec.model_validate(payload)
    ####


def test_declared_robustness_screen_contract_fails_closed_and_accepts_its_standard_artifact(tmp_path: Path) -> None:
    endpoint = load_vehicle_endpoint_spec_catalog().endpoint("hummingbird-source-hover-rotor-lqi")
    screen = endpoint.robustness_screens[0]
    planned_screen = screen.model_copy(update={"execution_readiness": "planned"})
    planned_endpoint = endpoint.model_copy(update={"robustness_screens": (planned_screen,)})
    errors: list[str] = []

    planned = _verify_disturbance_or_mass_evidence(planned_endpoint, "planned-robustness", tmp_path, errors)
    assert planned["status"] == "not_executed"
    assert errors == []

    unavailable = _verify_disturbance_or_mass_evidence(endpoint, "missing-available-robustness", tmp_path, errors)
    assert unavailable["status"] == "fail"
    assert any(screen.artifact_filename in item for item in errors)
    errors.clear()

    payload = {
        "schema": "taoryx.endpoint-robustness-screen/v1alpha1",
        "id": screen.id,
        "kind": screen.kind,
        "pass": True,
        "cases": [
            {
                "id": case.id,
                "parameters": dict(case.parameters),
                "status": "pass",
                "metrics": {
                    metric.id: metric.minimum if metric.minimum is not None else metric.maximum
                    for metric in screen.metrics
                },
            }
            for case in screen.cases
        ],
    }
    (tmp_path / screen.artifact_filename).write_text(json.dumps(payload), encoding="utf-8")
    passed = _verify_disturbance_or_mass_evidence(endpoint, "declared-robustness", tmp_path, errors)

    assert passed["status"] == "pass"
    assert passed["evidence"][0]["cases"][0]["status"] == "pass"
    payload["cases"][0]["metrics"]["saturation_fraction"] = 1.0
    (tmp_path / screen.artifact_filename).write_text(json.dumps(payload), encoding="utf-8")
    failed = _verify_disturbance_or_mass_evidence(endpoint, "failed-robustness", tmp_path, errors)
    assert failed["status"] == "fail"
    assert any("saturation_fraction" in item for item in errors)
    ####


@pytest.mark.slow
@pytest.mark.parametrize("endpoint_id", _ENDPOINT_IDS)
def test_selected_endpoint_contracts_exercise_their_cross_catalog_wiring(endpoint_id: str) -> None:
    report = verify_vehicle_endpoint(endpoint_id)
    endpoint = load_vehicle_endpoint_spec_catalog().endpoint(endpoint_id)

    assert report["status"] == "pass", report["errors"]
    records = report["records"]
    assert isinstance(records, dict)
    assert records["interface"]["status"] == "pass"
    assert records["outputs"]["status"] == "pass"
    if endpoint.controller_campaign_id is None:
        assert records["controller"]["status"] == "not_applicable"
    else:
        assert records["controller"]["status"] == "pass"
        assert records["controller"]["missing_tuning_operations"] == []
    witnesses = records["witnesses"]
    assert isinstance(witnesses, list)
    assert all(item["compile_status"] == "pass" for item in witnesses)
    assert all(item["preflight_status"] == "translation_ready" for item in witnesses)
    assert all(item["capability_advertisement"]["status"] == "pass" for item in witnesses)
    ####


def test_endpoint_verifier_fails_closed_when_a_required_public_output_drifts() -> None:
    catalog = load_vehicle_endpoint_spec_catalog()
    original = catalog.endpoint("hummingbird-source-hover-rotor-lqi")
    changed = original.model_copy(
        update={"required_core_output_ids": (*original.required_core_output_ids, "state.not.advertised")}
    )
    errors: list[str] = []
    record = _verify_output_contract(changed, errors)

    assert record["status"] == "fail"
    assert any("state.not.advertised" in item for item in errors)
    ####


def test_endpoint_verifier_fails_closed_when_an_advertised_output_is_not_emitted(tmp_path: Path) -> None:
    endpoint = load_vehicle_endpoint_spec_catalog().endpoint("hummingbird-source-hover-rotor-lqi")
    errors: list[str] = []

    record = _verify_emitted_output_contract(endpoint, "missing-output-witness", tmp_path, errors)

    assert record["status"] == "fail"
    assert record["missing_required_output_ids"] == sorted(
        (*endpoint.required_core_output_ids, *endpoint.required_telemetry_output_ids)
    )
    assert any("no truth_telemetry.csv" in item for item in errors)
    ####


def test_candidate_binding_requires_the_exact_selected_configuration_fingerprint() -> None:
    controller_record = {
        "campaign_id": "example-lqi-campaign",
        "tuning_status": "pass",
        "candidate_status": "candidate_ready",
        "candidate_methods": ["lqi"],
        "selected_candidate_configurations": [
            {
                "node_id": "hover",
                "profile_id": "gentle",
                "method": "lqi",
                "candidate_configuration_fingerprint_sha256": "a" * 64,
            }
        ],
        "tuning_application_contexts": [
            {
                "node_id": "hover",
                "candidate_profile_id": "gentle",
                "candidate_configuration_fingerprint_sha256": "a" * 64,
                "resolved_gain_fingerprint_sha256": "c" * 64,
            }
        ],
    }
    evidence = {
        "status": "verified",
        "method": "lqi",
        "tuning_binding": {
            "status": "declared",
            "campaign_id": "example-lqi-campaign",
            "node_id": "hover",
            "candidate_profile_id": "gentle",
            "candidate_configuration_fingerprint_sha256": "a" * 64,
            "applied_gain_fingerprint_sha256": "c" * 64,
            "controller_method": "lqi",
        },
    }

    assert _resolve_tuning_binding(controller_record, evidence)["status"] == "candidate_bound"
    evidence["tuning_binding"]["candidate_configuration_fingerprint_sha256"] = "b" * 64
    assert _resolve_tuning_binding(controller_record, evidence)["status"] == "candidate_binding_mismatch"
    evidence["tuning_binding"]["candidate_configuration_fingerprint_sha256"] = "a" * 64
    evidence["tuning_binding"]["applied_gain_fingerprint_sha256"] = "d" * 64
    assert _resolve_tuning_binding(controller_record, evidence)["status"] == "candidate_binding_mismatch"
    ####


def test_candidate_binding_requires_every_selected_schedule_node() -> None:
    """A multi-node campaign cannot be promoted from one applied node."""

    controller_record = {
        "campaign_id": "schedule-lqi",
        "tuning_status": "pass",
        "candidate_status": "candidate_ready",
        "candidate_methods": ["lqi"],
        "selected_candidate_configurations": [
            {
                "node_id": "sea-level",
                "profile_id": "sea-level-profile",
                "candidate_configuration_fingerprint_sha256": "a" * 64,
            },
            {
                "node_id": "high-altitude",
                "profile_id": "high-altitude-profile",
                "candidate_configuration_fingerprint_sha256": "b" * 64,
            },
        ],
        "tuning_application_contexts": [
            {
                "node_id": "sea-level",
                "candidate_profile_id": "sea-level-profile",
                "candidate_configuration_fingerprint_sha256": "a" * 64,
                "resolved_gain_fingerprint_sha256": "c" * 64,
            },
            {
                "node_id": "high-altitude",
                "candidate_profile_id": "high-altitude-profile",
                "candidate_configuration_fingerprint_sha256": "b" * 64,
                "resolved_gain_fingerprint_sha256": "d" * 64,
            },
        ],
    }
    bindings = [
        {
            "campaign_id": "schedule-lqi",
            "node_id": "sea-level",
            "candidate_profile_id": "sea-level-profile",
            "candidate_configuration_fingerprint_sha256": "a" * 64,
            "applied_gain_fingerprint_sha256": "c" * 64,
            "controller_method": "lqi",
        },
        {
            "campaign_id": "schedule-lqi",
            "node_id": "high-altitude",
            "candidate_profile_id": "high-altitude-profile",
            "candidate_configuration_fingerprint_sha256": "b" * 64,
            "applied_gain_fingerprint_sha256": "d" * 64,
            "controller_method": "lqi",
        },
    ]
    evidence = {
        "status": "verified",
        "method": "lqi",
        "tuning_binding": {"status": "declared_set", "bindings": bindings},
    }

    bound = _resolve_tuning_binding(controller_record, evidence)
    assert bound["status"] == "candidate_bound"
    assert bound["node_count"] == 2

    bindings.pop()
    assert _resolve_tuning_binding(controller_record, evidence)["status"] == "candidate_binding_mismatch"
    ####


def test_endpoint_acceptance_uses_the_declared_batch_witness_for_artifact_evidence() -> None:
    """An interactive bridge must not be treated as a missing batch sidecar."""

    endpoint = load_vehicle_endpoint_spec_catalog().endpoint("x15-source-release-direct-wrench-lqr")
    batch = {
        "id": "batch",
        "operation": "batch",
        "compile_status": "pass",
        "identity_status": "pass",
        "resolved_binding_status": "pass",
        "preflight_status": "translation_ready",
        "capability_advertisement": {"status": "pass"},
        "execution_status": "pass",
        "tracking_evidence": {"status": "pass"},
        "disturbance_or_mass_evidence": {"status": "not_applicable"},
        "tuning_provenance": {"binding": {"status": "candidate_bound"}},
    }
    episode = {
        **batch,
        "id": "episode",
        "operation": "episode",
        "tracking_evidence": {"status": "not_requested"},
        "disturbance_or_mass_evidence": {"status": "not_requested"},
        "tuning_provenance": {"status": "not_requested"},
    }
    records = {
        "selection": {"status": "pass"},
        "interface": {"status": "pass"},
        "outputs": {"status": "pass"},
        "maturity": {"status": "pass"},
        "controller": {"status": "pass", "tuning_status": "pass"},
        "execution": [{"operation": "batch", "status": "pass"}, {"operation": "episode", "status": "pass"}],
        "witnesses": [batch, episode],
    }

    gates = _build_endpoint_acceptance_gates(records, endpoint=endpoint, execute=True, tune=True)["gates"]

    assert gates["tracking_passed"]["status"] == "pass"
    assert gates["tracking_passed"]["witness_count"] == 1
    assert gates["tuner_bound"]["status"] == "pass"
    assert gates["disturbance_or_mass_screened"]["screen_statuses"] == ["not_applicable"]
    ####


def test_runtime_tuning_binding_rejects_a_method_that_disagrees_with_execution() -> None:
    runtime = {
        "tuning_binding": {
            "campaign_id": "example-lqi-campaign",
            "node_id": "hover",
            "candidate_profile_id": "gentle",
            "candidate_configuration_fingerprint_sha256": "a" * 64,
            "applied_gain_fingerprint_sha256": "c" * 64,
            "controller_method": "lqi",
        }
    }

    declared = _runtime_tuning_binding(runtime, "lqi")
    assert declared["status"] == "declared"
    runtime["tuning_binding"].pop("applied_gain_fingerprint_sha256")
    with pytest.raises(ValueError, match="applied_gain_fingerprint"):
        _runtime_tuning_binding(runtime, "lqi")
    runtime["tuning_binding"]["applied_gain_fingerprint_sha256"] = "c" * 64
    with pytest.raises(ValueError, match="controller method disagrees"):
        _runtime_tuning_binding(runtime, "lqr")
    ####


@pytest.mark.slow
def test_retained_hummingbird_endpoint_packet_checks_emitted_outputs_and_tuning_provenance(tmp_path: Path) -> None:
    report = verify_vehicle_endpoint(
        "hummingbird-source-hover-rotor-lqi",
        execute=True,
        tune=True,
        cache_dir=tmp_path / "cache",
        results_dir=tmp_path / "results",
    )

    assert report["status"] == "pass", report["errors"]
    records = report["records"]
    assert isinstance(records, dict)
    witness = records["witnesses"][0]
    assert witness["emitted_outputs"]["status"] == "pass"
    assert all(count > 0 for count in witness["emitted_outputs"]["emitted_sample_count_by_id"].values())
    assert witness["controller_execution_evidence"]["status"] == "verified"
    assert witness["tracking_evidence"]["status"] == "pass"
    provenance = witness["tuning_provenance"]
    assert provenance["binding"]["status"] == "candidate_bound"
    artifact = provenance["artifact_path"]
    assert isinstance(artifact, str) and Path(artifact).is_file()
    payload = json.loads(Path(artifact).read_text(encoding="utf-8"))
    assert payload["tuning"]["cache_key"] == records["controller"]["cache_key"]
    assert payload["execution"]["controller_method"] == "lqi"
    indexed = index_composition_results(Path(artifact).parent)
    assert indexed["status"] == "pass"
    assert "controller_tuning_provenance.json" in indexed["records"][0]["artifact_paths"]
    acceptance = records["acceptance_gates"]
    assert acceptance["gates"]["contract_valid"]["status"] == "pass"
    assert acceptance["gates"]["batch_executed"]["status"] == "pass"
    assert acceptance["gates"]["tracking_passed"]["status"] == "pass"
    assert acceptance["gates"]["tuner_bound"]["status"] == "pass"
    assert acceptance["gates"]["disturbance_or_mass_screened"]["status"] == "pass"
    assert acceptance["finalization_status"] == "ready"
    performance = records["performance"]
    assert performance["cache"]["disposition"] in {"hit", "miss"}
    assert performance["phase_durations_s"]["witness.batch_execution_s"] > 0.0
    assert performance["phase_durations_s"]["witness.result_catalog_index_s"] >= 0.0
    ####


@pytest.mark.slow
def test_x15_endpoint_packet_accepts_its_emitted_matched_offset_evidence() -> None:
    """The generic endpoint verifier reads X-15's allocator-backed offset artifact."""

    report = verify_vehicle_endpoint("x15-source-surface-attitude-rate-lqi", execute=True)

    assert report["status"] == "pass", report["errors"]
    records = report["records"]
    assert isinstance(records, dict)
    witness = records["witnesses"][0]
    assert witness["tracking_evidence"]["status"] == "pass"
    robustness = witness["disturbance_or_mass_evidence"]
    assert robustness["status"] == "pass"
    evidence = robustness["evidence"][0]
    assert evidence["status"] == "pass"
    assert [case["id"] for case in evidence["cases"]] == [
        "nominal",
        "positive-pitch-offset",
        "negative-pitch-offset",
    ]
    assert all(case["status"] == "pass" for case in evidence["cases"])
    acceptance = records["acceptance_gates"]
    assert acceptance["gates"]["disturbance_or_mass_screened"]["status"] == "pass"
    assert acceptance["finalization_status"] == "incomplete"
    ####


@pytest.mark.slow
def test_x8_endpoint_packet_accepts_its_emitted_matched_offset_evidence() -> None:
    """The generic endpoint verifier reads the source-table allocator-backed X8 artifact."""

    report = verify_vehicle_endpoint("x8-source-surface-roll-pitch-lqi", execute=True)

    assert report["status"] == "pass", report["errors"]
    records = report["records"]
    assert isinstance(records, dict)
    witness = records["witnesses"][0]
    assert witness["tracking_evidence"]["status"] == "pass"
    robustness = witness["disturbance_or_mass_evidence"]
    assert robustness["status"] == "pass"
    evidence = robustness["evidence"][0]
    assert evidence["status"] == "pass"
    assert [case["id"] for case in evidence["cases"]] == [
        "nominal",
        "positive-pitch-offset",
        "negative-pitch-offset",
    ]
    assert all(case["status"] == "pass" for case in evidence["cases"])
    acceptance = records["acceptance_gates"]
    assert acceptance["gates"]["disturbance_or_mass_screened"]["status"] == "pass"
    assert acceptance["finalization_status"] == "incomplete"
    ####


@pytest.mark.slow
def test_b747_endpoint_packet_accepts_its_emitted_matched_offset_evidence() -> None:
    """The shared verifier consumes B747's typed physical-LQI sidecar."""

    report = verify_vehicle_endpoint("b747-condition3-source-surface-attitude-rate-lqi", execute=True)

    assert report["status"] == "pass", report["errors"]
    records = report["records"]
    assert isinstance(records, dict)
    witness = records["witnesses"][0]
    assert witness["tracking_evidence"]["status"] == "pass"
    robustness = witness["disturbance_or_mass_evidence"]
    assert robustness["status"] == "pass"
    evidence = robustness["evidence"][0]
    assert evidence["status"] == "pass"
    assert [case["id"] for case in evidence["cases"]] == [
        "nominal",
        "positive-pitch-offset",
        "negative-pitch-offset",
    ]
    assert all(case["status"] == "pass" for case in evidence["cases"])
    acceptance = records["acceptance_gates"]
    assert acceptance["gates"]["disturbance_or_mass_screened"]["status"] == "pass"
    assert acceptance["finalization_status"] == "incomplete"
    ####


@pytest.mark.slow
def test_a320_native_coordinate_endpoint_records_its_nominal_convergence_boundary() -> None:
    """A nonphysical endpoint must state, not omit, its lack of a robustness seam."""

    report = verify_vehicle_endpoint("a320-pseudo6dof-native-coordinate-lqi", execute=True)

    assert report["status"] == "pass", report["errors"]
    records = report["records"]
    assert isinstance(records, dict)
    witness = records["witnesses"][0]
    assert witness["execution_status"] == "pass"
    assert witness["emitted_outputs"]["status"] == "pass"
    assert witness["result_catalog"]["status"] == "pass"
    assert witness["tracking_evidence"]["status"] == "pass"
    robustness = witness["disturbance_or_mass_evidence"]
    assert robustness["status"] == "not_applicable"
    assert robustness["required_screen_count"] == 0
    assert "no mass, wind, or persistent-offset input seam" in robustness["reason"]
    acceptance = records["acceptance_gates"]
    assert acceptance["gates"]["disturbance_or_mass_screened"] == {
        "status": "not_applicable",
        "required_for_finalization": False,
        "screen_statuses": ["not_applicable"],
        "reason": robustness["reason"],
    }
    assert acceptance["finalization_status"] == "incomplete"
    ####


@pytest.mark.slow
@pytest.mark.parametrize(
    "endpoint_id",
    ("nesc-source-history-replay-pseudo6dof", "tumbling-body-passive-release-pseudo6dof"),
)
def test_controller_free_endpoints_record_their_explicit_boundary(endpoint_id: str) -> None:
    """Source replay and passive execution must not fabricate controller or robustness evidence."""

    report = verify_vehicle_endpoint(endpoint_id, execute=True)

    assert report["status"] == "pass", report["errors"]
    records = report["records"]
    assert isinstance(records, dict)
    assert records["controller"]["status"] == "not_applicable"
    witness = records["witnesses"][0]
    assert witness["execution_status"] == "pass"
    assert witness["emitted_outputs"]["status"] == "pass"
    assert witness["result_catalog"]["status"] == "pass"
    assert witness["controller_execution_evidence"]["status"] == "not_applicable"
    assert witness["tracking_evidence"]["status"] == "pass"
    assert witness["disturbance_or_mass_evidence"]["status"] == "not_applicable"
    acceptance = records["acceptance_gates"]
    assert acceptance["gates"]["tuner_bound"]["status"] == "not_applicable"
    assert acceptance["gates"]["tuner_bound"]["required_for_finalization"] is False
    assert acceptance["gates"]["disturbance_or_mass_screened"]["status"] == "not_applicable"
    assert acceptance["gates"]["disturbance_or_mass_screened"]["required_for_finalization"] is False
    assert acceptance["finalization_status"] == "ready"
    ####


def test_vehicle_endpoint_specs_cli_is_discoverable(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["vehicle", "endpoint-specs"]) == 0

    payload = json.loads(capsys.readouterr().out)
    assert payload["schema"] == "taoryx.vehicle-endpoint-spec-list/v1alpha1"
    assert [item["id"] for item in payload["endpoints"]] == list(_ENDPOINT_IDS)
    ####


def test_vehicle_endpoint_results_directory_requires_execution(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    assert main(
        [
            "vehicle",
            "verify",
            "hummingbird-source-hover-rotor-lqi",
            "--results-dir",
            str(tmp_path / "results"),
        ]
    ) == 2
    assert "--results-dir requires --execute" in capsys.readouterr().out
    ####
