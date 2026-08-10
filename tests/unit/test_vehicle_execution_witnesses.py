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
    _validate_batch_controller_metadata,
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
    assert all(isinstance(item["capability_advertisement"], dict) for item in capability_preflights)
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
    assert all(isinstance(item["capability_preflight"]["capability_advertisement"], dict) for item in variant_records)
    assert {
        item["family_id"]: item["lowering_status"]
        for item in variant_records
    } == {
        "a320_openap_3dof": "adapter_bound",
        "hummingbird": "factory_bound",
    }
    ####


def test_family_scoped_execution_witnesses_keep_exact_endpoint_coverage() -> None:
    """A focused smoke validates one family without borrowing global witnesses."""

    report = validate_vehicle_execution_witnesses(family_ids=("hl20_mod_k",))

    assert report["status"] == "pass"
    assert report["family_filter"] == ["hl20_mod_k"]
    assert report["runnable_binding_count"] == 7
    assert report["witness_count"] == 7
    assert report["runnable_variant_count"] == 0
    assert report["variant_witness_count"] == 0
    assert report["graph_extension_witness_count"] == 0
    records = report["records"]
    assert isinstance(records, list)
    assert {
        (item["mission"], item["fidelity"], item["operation"])
        for item in records
    } == {
            ("hl20_local_direct_wrench_screen_v1", "rigid_body_6dof_direct_wrench", "batch"),
            ("hl20_local_direct_wrench_screen_v1", "rigid_body_6dof_direct_wrench", "episode"),
            ("hl20_local_direct_wrench_lqi_screen_v1", "rigid_body_6dof_direct_wrench", "batch"),
            ("hl20_source_surface_attitude_rate_lqi_screen_v1", "rigid_body_6dof_surface_allocated", "batch"),
            ("hl20_source_surface_pitch_authority_screen_v1", "rigid_body_6dof_surface_allocated", "batch"),
        ("hl20_source_booster_release_replay_v1", "point_mass_3dof", "batch"),
        ("hl20_source_booster_release_replay_v1", "pseudo_6dof", "batch"),
    }
    ####


def test_family_scoped_execution_witnesses_reject_an_unknown_family() -> None:
    """A typo must not turn a focused endpoint check into a false pass."""

    report = validate_vehicle_execution_witnesses(family_ids=("no_such_vehicle",))

    assert report["status"] == "fail"
    assert report["runnable_binding_count"] == 0
    assert report["witness_count"] == 0
    assert report["errors"] == ["unknown requested vehicle family: no_such_vehicle"]
    ####


def test_witness_scoped_execution_witness_runs_one_exact_endpoint() -> None:
    """One endpoint smoke does not claim family variants or graph extensions."""

    report = validate_vehicle_execution_witnesses(
        witness_ids=("hl20-local-direct-wrench-screen-batch",),
    )

    assert report["status"] == "pass"
    assert report["family_filter"] is None
    assert report["witness_filter"] == ["hl20-local-direct-wrench-screen-batch"]
    assert report["runnable_binding_count"] == 1
    assert report["witness_count"] == 1
    assert report["runnable_variant_count"] == 0
    assert report["variant_witness_count"] == 0
    assert report["graph_extension_witness_count"] == 0
    records = report["records"]
    assert isinstance(records, list)
    assert [(item["family_id"], item["mission"], item["fidelity"], item["operation"]) for item in records] == [
        ("hl20_mod_k", "hl20_local_direct_wrench_screen_v1", "rigid_body_6dof_direct_wrench", "batch"),
    ]
    ####


def test_witness_scoped_execution_witnesses_reject_an_unknown_witness() -> None:
    """A typo must not turn an endpoint-only smoke into a false pass."""

    report = validate_vehicle_execution_witnesses(witness_ids=("no-such-witness",))

    assert report["status"] == "fail"
    assert report["runnable_binding_count"] == 0
    assert report["witness_count"] == 0
    assert report["errors"] == ["unknown requested execution witness: no-such-witness"]
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
    lowering_statuses = {item["family_id"]: item["lowering_status"] for item in records}
    assert lowering_statuses == {
        "a320_openap_3dof": "adapter_bound",
        "hummingbird": "factory_bound",
    }
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
    control_evidence = normalized["control_execution_evidence"]
    controller_evidence = normalized["controller_execution_evidence"]
    source_action_evidence = normalized["semantic_action_trace_evidence"]
    assert isinstance(control_evidence, dict)
    assert isinstance(controller_evidence, dict)
    assert isinstance(source_action_evidence, dict)
    assert control_evidence["status"] == "not_applicable"
    assert controller_evidence["status"] == "not_applicable"
    assert source_action_evidence["status"] == "verified"
    assert source_action_evidence["sample_count"] > 0
    interface_trace = report["interface_trace"]
    assert isinstance(interface_trace, dict)
    assert interface_trace["status"] == "pass"
    assert interface_trace["sample_count"] > 0
    assert interface_trace["channel_count"] > 0
    coverage = interface_trace["batch_visible_channels"]
    assert isinstance(coverage, dict)
    assert coverage["status"]
    assert coverage["resources"]
    assert coverage["diagnostics"]
    assert "control.controller.method" in coverage["diagnostics"]
    reproduction = report["reproduction"]
    assert isinstance(reproduction, dict)
    assert reproduction["status"] == "pass"
    assert "taoryx vehicle run" in str(reproduction["command"])
    release_packet = report["release_packet"]
    assert isinstance(release_packet, dict)
    assert release_packet["status"] == "pass"
    assert release_packet["reproduction_identity"] == "verified"
    ####


def test_batch_witness_can_retain_a_release_ready_result_corpus(tmp_path: Path) -> None:
    """A user-selected output root preserves one normalized public packet."""

    destination = tmp_path / "retained-witness-results"
    report = validate_vehicle_execution_witnesses(
        execute_batch=True,
        witness_ids=("x8-3dof-batch",),
        retained_results_directory=destination,
    )

    assert report["status"] == "pass"
    retained = report["retained_result_corpus"]
    assert isinstance(retained, dict)
    assert retained["status"] == "pass"
    assert retained["valid_result_count"] == 1
    assert retained["release_packet_count"] == 1
    assert (destination / "x8-3dof-batch" / "execution" / "evaluation.json").is_file()
    release_path = destination / "release-catalog.json"
    assert release_path.is_file()
    release_catalog = json.loads(release_path.read_text(encoding="utf-8"))
    assert result_catalog_module.validate_composition_release_catalog(destination, release_catalog) == ()
    indexed = result_catalog_module.index_composition_results(destination)
    assert indexed["status"] == "pass"
    assert indexed["valid_result_count"] == 1
    ####


def test_retained_batch_result_corpus_requires_an_empty_destination(tmp_path: Path) -> None:
    destination = tmp_path / "nonempty-results"
    destination.mkdir()
    (destination / "existing.txt").write_text("preserve", encoding="utf-8")

    with pytest.raises(ValueError, match="must be empty"):
        validate_vehicle_execution_witnesses(
            execute_batch=True,
            witness_ids=("x8-3dof-batch",),
            retained_results_directory=destination,
        )
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
    controller_metadata = report["controller_metadata"]
    assert isinstance(controller_metadata, dict)
    assert controller_metadata["status"] == "pass"
    assert controller_metadata["method"] == "lqi"
    assert controller_metadata["status_trace_method"] == "lqi"
    action_trace = report["action_trace"]
    assert isinstance(action_trace, dict)
    assert action_trace["status"] == "pass"
    normalized = report["result_catalog"]
    assert isinstance(normalized, dict)
    assert normalized["record_kind"] == "local_controller_screen"
    interface_trace = report["interface_trace"]
    assert isinstance(interface_trace, dict)
    assert interface_trace["status"] == "pass"
    coverage = interface_trace["batch_visible_channels"]
    assert isinstance(coverage, dict)
    assert coverage["status"]
    assert coverage["resources"]
    assert coverage["diagnostics"]
    reproduction = report["reproduction"]
    assert isinstance(reproduction, dict)
    assert reproduction["status"] == "pass"
    release_packet = report["release_packet"]
    assert isinstance(release_packet, dict)
    assert release_packet["status"] == "pass"
    ####


@pytest.mark.parametrize(
    "composition_name",
    (
        "a320_local_native_coordinate_lqi_screen_compose.yaml",
        "b747_condition3_local_physical_surface_lqi_screen_compose.yaml",
        "f16_local_physical_direct_wrench_screen_compose.yaml",
        "f16_local_physical_surface_screen_compose.yaml",
        "f16_local_physical_surface_lqi_screen_compose.yaml",
        "f16_local_physical_surface_lqr_schedule_interior_screen_compose.yaml",
        "f16_local_physical_surface_lqi_schedule_interior_screen_compose.yaml",
        "f16_local_physical_surface_lqr_schedule_transition_screen_compose.yaml",
        "x8_local_physical_surface_lqi_long_recovery_screen_compose.yaml",
        "x15_source_surface_attitude_rate_lqi_screen_compose.yaml",
        "hl20_source_surface_attitude_rate_lqi_screen_compose.yaml",
        "hummingbird_local_individual_rotor_lqi_screen_compose.yaml",
        "hummingbird_local_vertical_translation_lqi_screen_compose.yaml",
    ),
)
def test_batch_witness_smoke_normalizes_every_local_controller_screen(
    composition_name: str,
) -> None:
    """Local screen result packets use their declared screen pass, not mission-only fields."""

    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / composition_name)
    )
    binding = resolve_vehicle_execution_binding(composition, "batch")

    report = _execute_batch_witness("local-screen-normalization", composition, binding=binding)

    assert report["status"] == "pass"
    assert report["mode"] == "local_controller_screen"
    assert report["screen_pass"] is True
    controller_metadata = report["controller_metadata"]
    assert isinstance(controller_metadata, dict)
    assert controller_metadata["status"] == "pass"
    assert controller_metadata["method"] in {"lqr", "lqi"}
    assert controller_metadata["status_trace_method"] == controller_metadata["method"]
    result_catalog = report["result_catalog"]
    assert isinstance(result_catalog, dict)
    assert result_catalog["record_kind"] in {"local_controller_screen", "mission_evaluation"}
    assert result_catalog["qualification"] in {None, "unqualified"}
    control_evidence = result_catalog["control_execution_evidence"]
    assert isinstance(control_evidence, dict)
    assert control_evidence["status"] == "verified"
    assert control_evidence["execution_control_realization"] == controller_metadata["control_realization"]
    controller_evidence = result_catalog["controller_execution_evidence"]
    assert isinstance(controller_evidence, dict)
    assert controller_evidence["status"] == "verified"
    assert controller_evidence["method"] == controller_metadata["method"]
    assert controller_evidence["status_trace_sample_count"] > 0
    ####


@pytest.mark.parametrize(
    ("composition_name", "control_realization"),
    (
        ("x15_source_surface_authority_screen_compose.yaml", "source_surface_three_axis_authority_allocation"),
        ("hl20_source_surface_pitch_authority_screen_compose.yaml", "source_surface_pitch_authority_allocation"),
    ),
)
def test_batch_witness_normalizes_source_surface_authority_execution(
    composition_name: str,
    control_realization: str,
) -> None:
    """Physical source authority proves allocation without inventing a controller."""

    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / composition_name)
    )
    binding = resolve_vehicle_execution_binding(composition, "batch")

    report = _execute_batch_witness("source-surface-authority-normalization", composition, binding=binding)

    assert report["status"] == "pass"
    assert report["mode"] == "source_surface_authority_screen"
    result_catalog = report["result_catalog"]
    assert isinstance(result_catalog, dict)
    control_evidence = result_catalog["control_execution_evidence"]
    assert isinstance(control_evidence, dict)
    assert control_evidence["status"] == "verified"
    assert control_evidence["execution_control_realization"] == control_realization
    assert control_evidence["physical_effector_allocation"] is True
    assert control_evidence["physical_allocation_trace_channel"] == "control.physical_effector_allocation"
    assert control_evidence["full_state_trim"]["status"] == "not_available"
    controller_evidence = result_catalog["controller_execution_evidence"]
    assert isinstance(controller_evidence, dict)
    assert controller_evidence["status"] == "not_applicable"
    ####


@pytest.mark.parametrize(
    ("family_id", "expected_witness_count"),
    (
        ("reference_nesc_two_stage_rocket", 2),
        ("tumbling_body", 2),
    ),
)
def test_family_batch_witnesses_preserve_explicit_controller_free_evidence(
    family_id: str,
    expected_witness_count: int,
) -> None:
    """Replay and passive endpoints remain consumable without inventing control."""

    report = validate_vehicle_execution_witnesses(
        execute_batch=True,
        family_ids=(family_id,),
    )

    assert report["status"] == "pass"
    records = report["records"]
    assert isinstance(records, list)
    assert len(records) == expected_witness_count
    for record in records:
        execution = record["batch_execution"]
        assert isinstance(execution, dict)
        assert execution["status"] == "pass"
        result_catalog = execution["result_catalog"]
        assert isinstance(result_catalog, dict)
        control_evidence = result_catalog["control_execution_evidence"]
        controller_evidence = result_catalog["controller_execution_evidence"]
        assert isinstance(control_evidence, dict)
        assert isinstance(controller_evidence, dict)
        assert control_evidence["status"] == "not_applicable"
        assert controller_evidence["status"] == "not_applicable"
    ####


@pytest.mark.parametrize(
(
    "payload", "detail"
),
(
    (
        {
            "runtime": {"controller_method": "lqr", "control_realization": "surface_allocated"},
            "control_screen": {"controller_method": "lqi"},
        },
        "runtime and control-screen controller methods disagree",
    ),
    (
        {
            "runtime": {"controller_method": "lqi"},
            "control_screen": {"controller_method": "lqi"},
        },
        "runtime controller metadata omitted control realization",
    ),
    (
        {"control_screen": {"controller_method": "lqr"}},
        "controller screen omitted runtime controller metadata",
    ),
),
)
def test_batch_controller_metadata_gate_fails_closed_for_incoherent_evidence(
    payload: dict[str, object],
    detail: str,
) -> None:
    report = _validate_batch_controller_metadata(payload)

    assert report == {"status": "fail", "detail": detail}
    ####


def test_batch_controller_metadata_gate_requires_committed_trace_agreement() -> None:
    payload = {
        "runtime": {"controller_method": "lqi", "control_realization": "surface_allocated"},
        "control_screen": {"controller_method": "lqi"},
    }
    report = _validate_batch_controller_metadata(
        payload,
        status_trace={"samples": [{"values": {"control.controller.method": "lqr"}}]},
    )

    assert report == {
        "status": "fail",
        "detail": "runtime controller method disagrees with committed status trace",
    }
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
