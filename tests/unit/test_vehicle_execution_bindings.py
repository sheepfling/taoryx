"""Validation of the explicit compose-to-execution binding catalog."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Literal, cast

import pytest
from taoryx.hl20_source_release_composition_execution import execute_hl20_source_booster_release_composition
from taoryx.hummingbird_composition_execution import execute_hummingbird_pseudo_composition
from taoryx.nesc_composition_execution import execute_nesc_source_replay_composition
from taoryx.passive_tumbling_composition_execution import execute_passive_tumbling_composition
from taoryx.x15_staged_composition_execution import execute_x15_staged_reachability_composition

from taoryx.composition_control_trace import validate_committed_control_trace
from taoryx.composition_episode import (
    HummingbirdPseudoCompositionEpisode,
    open_vehicle_composition_episode,
    registered_episode_factory_ids,
)
from taoryx.composition_result_catalog import index_composition_results
from taoryx.language_backed_execution import execute_powered_fixed_wing_composition
from taoryx.local_direct_wrench_composition_execution import execute_local_direct_wrench_composition
from taoryx.runtime.cli import main
from taoryx.vehicle_composition import (
    CompiledVehicleComposition,
    CompositionValue,
    MissionGraphNodeSelection,
    MissionGraphSelection,
    MissionTransitionSelection,
    SegmentSelection,
    compile_vehicle_composition,
    load_vehicle_composition_request,
)
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog
from taoryx.vehicle_execution_bindings import (
    VehicleExecutionBinding,
    VehicleExecutionBindingError,
    batch_episode_parity_record,
    batch_episode_parity_records,
    load_vehicle_batch_episode_parity_catalog,
    load_vehicle_execution_binding_catalog,
    resolve_vehicle_execution_binding,
    validate_batch_episode_parity_bindings,
    validate_execution_bindings,
)
from taoryx.vehicle_execution_preflight import preflight_vehicle_composition
from taoryx.vehicle_runtime_lowering import lower_vehicle_composition

ROOT = Path(__file__).resolve().parents[2]


def _composition(name: str) -> CompiledVehicleComposition:
    return compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / name))
    ####


def _assert_unobserved_graph_execution(output_dir: Path) -> None:
    """Assert that a non-dispatching executor does not imply graph progress."""

    payload = json.loads((output_dir / "mission_graph_execution.json").read_text(encoding="utf-8"))
    assert payload["schema"] == "taoryx.mission-graph-execution/v1alpha1"
    assert payload["observation_status"] == "unobserved"
    assert payload["completed_nominal_success_path"] is None
    assert payload["dispatches"] == []
    ####


def _declared_missions() -> dict[str, dict[str, set[str]]]:
    catalog = load_resolved_vehicle_composition_catalog()
    return {
        vehicle.family.family_id: {mission.id: set(mission.compatible_fidelities) for mission in vehicle.declaration.mission_templates}
        for vehicle in catalog.vehicles
    }
    ####


def test_execution_binding_catalog_references_declared_family_missions_and_tiers() -> None:
    catalog = load_vehicle_execution_binding_catalog()

    assert validate_execution_bindings(catalog.bindings, declarations=_declared_missions()) == ()
    declared_dispositions = {item.batch_action_trace for item in catalog.bindings}
    assert declared_dispositions <= {
        "emits_committed_interval_trace",
        "committed_interval_history_missing",
        "not_emitted",
        "not_applicable",
        "planned",
    }
    assert {"emits_committed_interval_trace", "not_applicable", "planned"} <= declared_dispositions
    assert "not_emitted" not in declared_dispositions
    assert all(item.batch_action_trace == "not_applicable" for item in catalog.bindings if item.operation == "episode")
    assert all(
        item.batch_action_trace == "emits_committed_interval_trace"
        for item in catalog.bindings
        if item.operation == "batch" and item.status == "runnable"
    )
    assert all(item.batch_action_trace == "planned" for item in catalog.bindings if item.status == "planned")
    assert all(item.execution_mode == "planned" for item in catalog.bindings if item.status == "planned")
    assert all(item.execution_mode != "planned" for item in catalog.bindings if item.status == "runnable")
    assert {
        "closed_loop_controller",
        "source_history_replay",
        "source_scheduled_replay",
        "open_loop_witness",
        "local_direct_wrench_screen",
        "source_surface_authority_screen",
        "passive_uncontrolled",
        "planned",
    } <= {item.execution_mode for item in catalog.bindings}
    runnable_a320_episodes = {
        (item.family_id, item.fidelity): item.factory_id
        for item in catalog.bindings
        if item.operation == "episode" and item.status == "runnable" and item.family_id == "a320_openap_3dof"
    }
    assert runnable_a320_episodes == {
        ("a320_openap_3dof", "point_mass_3dof"): "reduced_fixed_wing_a320_episode.v1",
        ("a320_openap_3dof", "pseudo_6dof"): "reduced_fixed_wing_a320_episode.v1",
    }
    runnable_f16_episodes = {
        (item.family_id, item.fidelity): item.factory_id
        for item in catalog.bindings
        if item.operation == "episode" and item.status == "runnable" and item.family_id == "f16_s119"
    }
    assert runnable_f16_episodes == {
        ("f16_s119", "point_mass_3dof"): "reduced_fixed_wing_f16_episode.v1",
        ("f16_s119", "pseudo_6dof"): "reduced_fixed_wing_f16_episode.v1",
    }
    ####


def test_every_advertised_runnable_episode_factory_has_one_runtime_constructor() -> None:
    """The public endpoint catalog and fail-closed episode registry stay aligned."""

    catalog = load_vehicle_execution_binding_catalog()
    advertised = {item.factory_id for item in catalog.bindings if item.operation == "episode" and item.status == "runnable" and item.factory_id is not None}

    assert advertised == set(registered_episode_factory_ids())
    ####


@pytest.mark.parametrize(
    ("update", "message"),
    (
        (
            {"execution_mode": "source_history_replay", "operation": "episode", "batch_action_trace": "not_applicable"},
            "supports batch replay/release only",
        ),
        (
            {"execution_mode": "source_scheduled_replay", "operation": "episode", "batch_action_trace": "not_applicable"},
            "supports batch replay/release only",
        ),
        (
            {"execution_mode": "passive_uncontrolled", "operation": "episode", "batch_action_trace": "not_applicable"},
            "supports batch replay/release only",
        ),
        (
            {"execution_mode": "local_direct_wrench_screen", "fidelity": "pseudo_6dof"},
            "requires rigid_body_6dof_direct_wrench fidelity",
        ),
        (
            {"execution_mode": "source_surface_authority_screen", "operation": "episode", "batch_action_trace": "not_applicable"},
            "supports batch execution only",
        ),
    ),
)
def test_execution_mode_rejects_an_incompatible_endpoint_contract(update: dict[str, object], message: str) -> None:
    payload: dict[str, object] = {
        "family_id": "fixture",
        "mission": "fixture_mission",
        "fidelity": "point_mass_3dof",
        "operation": "batch",
        "status": "runnable",
        "execution_mode": "closed_loop_controller",
        "factory_id": "fixture.v1",
        "batch_action_trace": "emits_committed_interval_trace",
        "description": "Fixture endpoint.",
        "claim_boundary": "Fixture only.",
    }

    with pytest.raises(ValueError, match=message):
        VehicleExecutionBinding.model_validate({**payload, **update})
    ####


def test_batch_episode_parity_registry_is_explicit_and_never_inferred() -> None:
    parity_catalog = load_vehicle_batch_episode_parity_catalog()
    execution_catalog = load_vehicle_execution_binding_catalog()

    assert validate_batch_episode_parity_bindings(parity_catalog.bindings, execution_bindings=execution_catalog.bindings) == ()

    hummingbird = batch_episode_parity_record("hummingbird", "multirotor_pad_box_yaw_recovery_land_v1", "pseudo_6dof", parity_catalog=parity_catalog)
    hummingbird_physical = batch_episode_parity_record("hummingbird", "hummingbird_local_individual_rotor_lqi_screen_v1", "rigid_body_6dof_surface_allocated", parity_catalog=parity_catalog)
    hummingbird_horizontal = batch_episode_parity_record("hummingbird", "hummingbird_local_horizontal_translation_lqi_screen_v1", "rigid_body_6dof_surface_allocated", parity_catalog=parity_catalog)
    hummingbird_vertical = batch_episode_parity_record("hummingbird", "hummingbird_local_vertical_translation_lqi_screen_v1", "rigid_body_6dof_surface_allocated", parity_catalog=parity_catalog)
    hummingbird_direct = batch_episode_parity_record("hummingbird", "hummingbird_local_direct_wrench_screen_v1", "rigid_body_6dof_direct_wrench", parity_catalog=parity_catalog)
    x8 = batch_episode_parity_record("skywalker_x8", "powered_fixed_wing_racetrack_v1", "point_mass_3dof", parity_catalog=parity_catalog)
    x8_pseudo = batch_episode_parity_record("skywalker_x8", "powered_fixed_wing_racetrack_v1", "pseudo_6dof", parity_catalog=parity_catalog)
    a320 = batch_episode_parity_record("a320_openap_3dof", "powered_fixed_wing_racetrack_v1", "pseudo_6dof", parity_catalog=parity_catalog)
    f16 = batch_episode_parity_record("f16_s119", "powered_fixed_wing_racetrack_v1", "pseudo_6dof", parity_catalog=parity_catalog)
    x15 = batch_episode_parity_record("x15", "x15_local_direct_wrench_screen_v1", "rigid_body_6dof_direct_wrench", parity_catalog=parity_catalog)
    hl20 = batch_episode_parity_record("hl20_mod_k", "hl20_local_direct_wrench_screen_v1", "rigid_body_6dof_direct_wrench", parity_catalog=parity_catalog)

    assert hummingbird["availability"] == "registered"
    assert hummingbird["adapter_id"] == "taoryx.hummingbird.aggregate_thrust_batch_episode_parity.v1"
    assert hummingbird_physical["availability"] == "not_available"
    assert hummingbird_physical["runnable_operations"] == ["batch"]
    assert hummingbird_horizontal["availability"] == "not_available"
    assert hummingbird_horizontal["runnable_operations"] == ["batch"]
    assert hummingbird_vertical["availability"] == "not_available"
    assert hummingbird_vertical["runnable_operations"] == ["batch"]
    assert hummingbird_direct["availability"] == "not_available"
    assert hummingbird_direct["runnable_operations"] == ["batch"]
    assert x8["availability"] == "registered"
    assert x8["adapter_id"] == "taoryx.language_backed.action_trace_batch_episode_parity.v1"
    assert x8_pseudo["availability"] == "registered"
    assert x8_pseudo["adapter_id"] == "taoryx.language_backed.action_trace_batch_episode_parity.v1"
    assert a320["availability"] == "registered"
    assert a320["adapter_id"] == "taoryx.reduced_fixed_wing.a320_action_trace_batch_episode_parity.v1"
    assert f16["availability"] == "registered"
    assert f16["adapter_id"] == "taoryx.reduced_fixed_wing.f16_action_trace_batch_episode_parity.v1"
    assert x15["availability"] == "registered"
    assert x15["adapter_id"] == "taoryx.x15.local_direct_wrench_batch_episode_parity.v1"
    assert hl20["availability"] == "registered"
    assert hl20["adapter_id"] == "taoryx.local_direct_wrench_batch_episode_parity.v1"
    assert batch_episode_parity_records("hummingbird", parity_catalog=parity_catalog) == [
        hummingbird_direct,
        hummingbird_horizontal,
        hummingbird_physical,
        hummingbird_vertical,
        hummingbird,
    ]
    ####


@pytest.mark.parametrize(
    ("request_name", "operation", "factory_id"),
    (
        ("x8_racetrack_capability_3dof_compose.yaml", "batch", "language_backed_powered_fixed_wing.v1"),
        ("b747_racetrack_capability_3dof_compose.yaml", "batch", "language_backed_powered_fixed_wing.v1"),
        ("b747_racetrack_capability_pseudo6dof_compose.yaml", "episode", "language_backed_interactive.v1"),
        ("a320_racetrack_capability_3dof_compose.yaml", "batch", "reduced_fixed_wing_openap.v1"),
        ("f16_racetrack_capability_pseudo6dof_compose.yaml", "batch", "reduced_fixed_wing_f16_source.v1"),
        ("nesc_staged_source_replay_pseudo6dof_compose.yaml", "batch", "nesc_source_replay.v1"),
        ("hl20_source_booster_release_replay_3dof_compose.yaml", "batch", "hl20_source_booster_release_replay.v1"),
        ("x15_staged_booster_reachability_3dof_compose.yaml", "batch", "x15_staged_reachability.v1"),
        ("x15_local_direct_wrench_screen_compose.yaml", "batch", "local_direct_wrench_screen.v1"),
        ("tumbling_body_direct_release_pseudo6dof_compose.yaml", "batch", "passive_tumbling_direct_release.v1"),
        ("hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml", "episode", "hummingbird_aggregate_thrust_episode.v1"),
        ("hummingbird_local_individual_rotor_lqi_screen_compose.yaml", "batch", "hummingbird_local_individual_rotor_lqi_screen.v1"),
    ),
)
def test_execution_bindings_select_an_exact_family_mission_tier_and_operation(
    request_name: str,
    operation: Literal["batch", "episode"],
    factory_id: str,
) -> None:
    binding = resolve_vehicle_execution_binding(_composition(request_name), operation)

    assert binding.status == "runnable"
    assert binding.factory_id == factory_id
    ####


def test_execution_binding_refuses_an_undeclared_interactive_fallback() -> None:
    composition = _composition("nesc_staged_source_replay_pseudo6dof_compose.yaml")

    with pytest.raises(VehicleExecutionBindingError, match="no episode execution binding"):
        resolve_vehicle_execution_binding(composition, "episode")
    ####


def test_x8_batch_execution_accepts_an_authored_graph_that_preserves_the_template_sequence(tmp_path: Path) -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml")
    instance_ids = tuple(segment.instance_id or f"{index:02d}-{segment.id}" for index, segment in enumerate(request.segments, start=1))
    graph = MissionGraphSelection(
        entry_instance_id=instance_ids[0],
        nodes=tuple(
            MissionGraphNodeSelection(
                instance_id=instance_ids[index],
                success_transition=(None if index + 1 == len(request.segments) else MissionTransitionSelection(target_instance_id=instance_ids[index + 1])),
            )
            for index in range(len(request.segments))
        ),
    )
    composition = compile_vehicle_composition(request.model_copy(update={"mission_graph": graph}))

    result = execute_powered_fixed_wing_composition(composition, tmp_path / "x8-authored-linear-graph")

    assert result.mission_pass is True
    assert result.preflight.status == "translation_ready"
    assert result.composition.mission_graph is not None
    assert result.composition.mission_graph.status == "authored_linear_sequence_lowered"
    ####


def test_vehicle_example_validator_distinguishes_compilation_from_required_preflight(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    source = ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml"
    valid = tmp_path / source.name
    valid.write_text(source.read_text(encoding="utf-8"), encoding="utf-8")

    assert main(["vehicle", "validate-examples", "--directory", str(tmp_path), "--require-preflight"]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["status"] == "pass"
    assert report["examples"][0]["preflight_status"] == "translation_ready"
    assert report["examples"][0]["lowering_status"] == "factory_bound"
    assert report["examples"][0]["lowering_execution_binding"]["factory_id"] == "language_backed_powered_fixed_wing.v1"

    blocked = tmp_path / "x8-racetrack-legacy.yaml"
    blocked.write_text(
        (ROOT / "examples/vehicle_composition/x8_racetrack_compose.yaml").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    assert main(["vehicle", "validate-examples", "--directory", str(tmp_path), "--require-preflight"]) == 2
    strict_report = json.loads(capsys.readouterr().out)
    blocked_records = [record for record in strict_report["examples"] if record["path"].endswith(blocked.name)]
    assert blocked_records[0]["preflight_status"] == "blocked"
    assert blocked_records[0]["composition_status"] == "fail"
    ####


def test_hummingbird_pseudo_composition_runs_with_truth_objectives_and_contact_finality(tmp_path: Path) -> None:
    result = execute_hummingbird_pseudo_composition(
        _composition("hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml"),
        tmp_path / "hummingbird-execution",
    )

    assert result.mission_pass is True
    assert result.preflight.status == "translation_ready"
    assert result.runtime["physical_motor_allocation"] is False
    margins = cast(dict[str, float], result.envelope["minimum_margins"])
    assert margins["translation_speed_m_s"] >= 0.0
    assert result.truth_evaluation["required_passed"] == 7
    assert result.truth_evaluation["terminal_pass"] is True
    assert result.controller_transitions[-1]["reason"] == "EVENT_COMPLETE"
    assert (result.output_dir / "truth_telemetry.csv").is_file()
    assert result.status_trace["schema"] == "taoryx.composition-status-trace/v1alpha1"
    status_channels = cast(list[str], result.status_trace["channels"])
    assert "resources.battery.fraction_remaining" in status_channels
    assert "resources.mass.total" in status_channels
    first_status = cast(dict[str, object], cast(list[dict[str, object]], result.status_trace["samples"])[0]["values"])
    assert first_status["resources.mass.total"] == 0.5
    assert (result.output_dir / "status_trace.json").is_file()
    action_trace = json.loads((result.output_dir / "semantic_action_trace.json").read_text(encoding="utf-8"))
    validate_committed_control_trace(result.composition, action_trace)
    assert action_trace["requested_action_channels"] == [
        "attitude.pitch.command",
        "attitude.roll.command",
        "attitude.yaw.command",
        "propulsion.command.fraction",
        "propulsion.enable",
    ]
    assert action_trace["achieved_effector_channels"] == []
    assert (result.output_dir / "resource_ledger.json").is_file()
    assert (result.output_dir / "semantic_action_trace.json").is_file()
    assert (result.output_dir / "mission_graph_execution.json").is_file()
    assert (result.output_dir / "objective_report.json").is_file()
    assert (result.output_dir / "variant_runtime_evidence.json").is_file()
    assert (result.output_dir / "controller_transitions.json").is_file()
    evaluation = json.loads((result.output_dir / "evaluation.json").read_text(encoding="utf-8"))
    assert evaluation["outcome"] == "completed"
    assert evaluation["qualification"] == "unqualified"
    assert evaluation["gates"][1]["id"] == "independent-truth-objectives"
    assert evaluation["gates"][1]["status"] == "pass"
    requested_controls = {item["id"]: item for item in evaluation["requested_controls"]}
    assert requested_controls["requested.propulsion.enable"]["status"] == "available"
    assert requested_controls["requested.propulsion.enable"]["value"] is False
    result_catalog = index_composition_results(result.output_dir)
    assert result_catalog["status"] == "pass"
    records = cast(list[dict[str, object]], result_catalog["records"])
    provenance = cast(dict[str, object], records[0]["composition_provenance"])
    assert provenance["status"] == "verified"
    resource_ledger_evidence = cast(dict[str, object], records[0]["resource_ledger_evidence"])
    assert resource_ledger_evidence["status"] == "verified"
    assert resource_ledger_evidence["resource_count"] == 2
    assert provenance["composition_identity_sha256"] == result.composition.identity_sha256
    action_trace = cast(dict[str, object], records[0]["semantic_action_trace_evidence"])
    assert action_trace["status"] == "verified"
    assert action_trace["requested_action_channel_count"] == 5
    variant_runtime = cast(dict[str, object], records[0]["variant_runtime_evidence"])
    assert variant_runtime["status"] == "verified"
    assert variant_runtime["evidence_status"] == "not_applicable"
    ####


def test_hummingbird_timeout_recovery_graph_runs_the_nominal_path_without_promoting_timeout(tmp_path: Path) -> None:
    """The first branch-capable translator accepts only safe timeout recovery.

    This nominal witness follows the success chain.  The graph retains the
    timeout edge so preflight/lowering prove that the Hummingbird translator
    has explicitly opted into it; a real timeout would still fail the skipped
    required objective table and therefore cannot pass the mission.
    """

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml")
    instance_ids = tuple(segment.instance_id or f"{index:02d}-{segment.id}" for index, segment in enumerate(request.segments, start=1))
    touchdown_instance_id = instance_ids[-1]
    graph = MissionGraphSelection(
        entry_instance_id=instance_ids[0],
        nodes=tuple(
            MissionGraphNodeSelection(
                instance_id=instance_ids[index],
                success_transition=(None if index + 1 == len(request.segments) else MissionTransitionSelection(target_instance_id=instance_ids[index + 1])),
                timeout_transition=(MissionTransitionSelection(target_instance_id=touchdown_instance_id) if index == 2 else None),
            )
            for index in range(len(request.segments))
        ),
    )
    composition = compile_vehicle_composition(request.model_copy(update={"mission_graph": graph}))

    assert composition.mission_graph is not None
    assert composition.mission_graph.status == "authored_graph_not_lowered"
    assert preflight_vehicle_composition(composition).status == "translation_ready"
    assert lower_vehicle_composition(composition).status == "factory_bound"

    result = execute_hummingbird_pseudo_composition(composition, tmp_path / "hummingbird-timeout-recovery-graph")

    assert result.mission_pass is True
    graph_execution = cast(dict[str, object], result.runtime["mission_graph_execution"])
    assert graph_execution["schema"] == "taoryx.mission-graph-execution/v1alpha1"
    assert graph_execution["graph_status"] == "authored_graph_not_lowered"
    assert graph_execution["observation_status"] == "observed"
    assert graph_execution["completed_nominal_success_path"] is True
    graph_manifest = cast(dict[str, object], result.plan.manifest()["mission_graph"])
    assert graph_manifest["translator_execution_status"] == "hummingbird_timeout_recovery_lowered"
    timed_node = next(item for item in result.controller_transitions if item["segment_instance_id"] == instance_ids[2])
    assert timed_node["graph_outcome"] == "success"
    assert timed_node["next_instance_id"] == instance_ids[3]
    ####


def test_hummingbird_timeout_recovery_graph_executes_safe_landing_branch_and_fails_mission(tmp_path: Path) -> None:
    """An actual controller timeout follows the declared landing edge, never success."""

    request = load_vehicle_composition_request(
        ROOT / "examples/vehicle_composition/hummingbird_timeout_recovery_graph_pseudo6dof_compose.yaml"
    )
    timed_segment = request.segments[2]
    forced_timeout = SegmentSelection(
        id=timed_segment.id,
        instance_id=timed_segment.instance_id,
        inputs={
            **timed_segment.inputs,
            "target_ned_m": CompositionValue(value=[1000.0, 0.0, -2.0], unit="m"),
        },
    )
    composition = compile_vehicle_composition(
        request.model_copy(update={"segments": (*request.segments[:2], forced_timeout, *request.segments[3:])})
    )

    result = execute_hummingbird_pseudo_composition(composition, tmp_path / "hummingbird-timeout-recovery-executed")

    assert result.mission_pass is False
    graph_execution = cast(dict[str, object], result.runtime["mission_graph_execution"])
    assert graph_execution["observation_status"] == "observed"
    assert graph_execution["completed_nominal_success_path"] is False
    dispatches = cast(list[dict[str, object]], graph_execution["dispatches"])
    timed_dispatch = next(item for item in dispatches if item["instance_id"] == "forward-translation")
    assert timed_dispatch["outcome"] == "timeout"
    assert timed_dispatch["transition_status"] == "transitioned"
    assert timed_dispatch["next_instance_id"] == "07-touchdown_settle_disarm"
    assert [item["instance_id"] for item in dispatches] == [
        "01-rotor_spool_takeoff",
        "02-hover_dwell",
        "forward-translation",
        "07-touchdown_settle_disarm",
    ]
    assert all(item["segment_instance_id"] not in {"04-yaw_scan", "lateral-return", "06-hover_dwell"} for item in result.controller_transitions)
    evaluation = cast(dict[str, object], result.truth_evaluation)
    assert evaluation["mission_pass"] is False
    ####


def test_hummingbird_graph_rejects_unsupported_abort_semantics() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml")
    instance_ids = tuple(segment.instance_id or f"{index:02d}-{segment.id}" for index, segment in enumerate(request.segments, start=1))
    graph = MissionGraphSelection(
        entry_instance_id=instance_ids[0],
        nodes=tuple(
            MissionGraphNodeSelection(
                instance_id=instance_ids[index],
                success_transition=(None if index + 1 == len(request.segments) else MissionTransitionSelection(target_instance_id=instance_ids[index + 1])),
                abort_transition=(MissionTransitionSelection(target_instance_id=instance_ids[-1]) if index == 0 else None),
            )
            for index in range(len(request.segments))
        ),
    )
    composition = compile_vehicle_composition(request.model_copy(update={"mission_graph": graph}))

    preflight = preflight_vehicle_composition(composition)

    assert preflight.status == "blocked"
    assert "abort" in preflight.diagnostics[0]
    ####


def test_hummingbird_grounded_mass_reset_is_consumed_by_the_batch_model(tmp_path: Path) -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml")
    grounded = request.initialization.model_copy(
        update={
            "id": "grounded_idle",
            "inputs": {
                "pad_north_m": CompositionValue(value=0.0, unit="m"),
                "pad_east_m": CompositionValue(value=0.0, unit="m"),
                "heading_deg": CompositionValue(value=0.0, unit="deg"),
                "mass_kg": CompositionValue(value=0.55, unit="kg"),
            },
        }
    )
    composition = compile_vehicle_composition(request.model_copy(update={"initialization": grounded}))
    result = execute_hummingbird_pseudo_composition(composition, tmp_path / "hummingbird-grounded-mass")

    assert result.mission_pass is True
    assert result.runtime["initial_mass_kg"] == 0.55
    assert result.preflight.derived_mission is not None
    assert result.preflight.derived_mission["initial_mass_kg"] == 0.55
    episode = open_vehicle_composition_episode(composition)
    try:
        assert isinstance(episode, HummingbirdPseudoCompositionEpisode)
        assert episode.model.mass_kg == 0.55
    finally:
        episode.close()
    ####


def test_hummingbird_grounded_mass_variant_is_bounded_and_consumed_by_batch_and_episode(tmp_path: Path) -> None:
    composition = _composition("hummingbird_grounded_mass_variant_pseudo6dof_compose.yaml")

    assert composition.initialization.inputs["mass_kg"].value == 0.55
    assert composition.variant.inputs["grounded_operating_mass_kg"].value == 0.55
    binding = composition.variant.runtime_bindings["grounded_operating_mass_kg"]
    assert binding.runtime_input_path == "HummingbirdPseudo6DOFModel.mass_kg"
    assert binding.status_derivation_relation == "equal_to_target"
    assert binding.resource_derivation == "not_represented"
    assert composition.variant.invalidations == ("requalification:mass_kg",)
    assert composition.variant.qualification == "extended"
    assert "no declared qualified range" in composition.variant.qualification_findings[0]

    result = execute_hummingbird_pseudo_composition(composition, tmp_path / "hummingbird-grounded-mass-variant")

    assert result.mission_pass is True
    assert result.runtime["initial_mass_kg"] == 0.55
    variant_evidence = cast(dict[str, object], result.runtime["variant_runtime_evidence"])
    assert variant_evidence["status"] == "pass"
    bindings = cast(list[dict[str, object]], variant_evidence["bindings"])
    assert bindings[0]["consumed_native_input_match"] is True
    channels = cast(list[dict[str, object]], bindings[0]["status_channels"])
    assert channels[0]["relation_match"] is True
    assert (result.output_dir / "variant_runtime_evidence.json").is_file()
    episode = open_vehicle_composition_episode(composition)
    try:
        assert isinstance(episode, HummingbirdPseudoCompositionEpisode)
        assert episode.model.mass_kg == 0.55
        assert episode.status_frame().values["resources.mass.total"] == 0.55
    finally:
        episode.close()
    ####


def test_hummingbird_sensor_composition_emits_only_boundary_held_observations(tmp_path: Path) -> None:
    result = execute_hummingbird_pseudo_composition(
        _composition("hummingbird_hover_yaw_sensor_episode_pseudo6dof_compose.yaml"),
        tmp_path / "hummingbird-sensor-execution",
    )

    assert result.mission_pass is True
    assert result.sensor_trace is not None
    assert result.sensor_trace["interpolation"] == "forbidden"
    samples = cast(list[dict[str, object]], result.sensor_trace["samples"])
    assert samples[0]["source_time_s"] is None
    assert samples[1]["source_time_s"] == pytest.approx(0.0)
    assert (result.output_dir / "sensor_observations.json").is_file()
    ####


@pytest.mark.parametrize(
    ("request_name", "expected_control_realization"),
    (
        ("nesc_staged_source_replay_3dof_compose.yaml", "uncontrolled_source_replay"),
        ("nesc_staged_source_replay_pseudo6dof_compose.yaml", "response_law"),
    ),
)
def test_nesc_source_replay_composition_runs_with_stage_truth_and_terminal_state(
    tmp_path: Path,
    request_name: str,
    expected_control_realization: str,
) -> None:
    result = execute_nesc_source_replay_composition(
        _composition(request_name),
        tmp_path / "nesc-source-replay",
    )

    assert result.mission_pass is True
    assert result.preflight.status == "translation_ready"
    assert result.runtime["participating_nonlinear_plant"] is False
    assert result.runtime["physical_gimbal_allocation"] is False
    assert result.runtime["control_realization"] == expected_control_realization
    assert result.truth_evaluation["required_passed"] == 4
    objectives = cast(list[dict[str, object]], result.truth_evaluation["results"])
    assert objectives[-1]["status"] == "pass"
    margins = cast(dict[str, float], result.envelope["minimum_margins"])
    assert margins["replay_position_error_margin_m"] >= 0.0
    assert result.envelope["phase_order"] == ["stage1_burn", "stack_coast", "stage2_burn", "orbit_coast"]
    status_channels = cast(list[str], result.status_trace["channels"])
    assert "resources.mass.total" in status_channels
    status_samples = cast(list[dict[str, object]], result.status_trace["samples"])
    first_status = cast(dict[str, object], status_samples[0]["values"])
    assert first_status["phase.mode"] == "stage1_burn"
    assert (result.output_dir / "status_trace.json").is_file()
    action_trace = json.loads((result.output_dir / "semantic_action_trace.json").read_text(encoding="utf-8"))
    validate_committed_control_trace(result.composition, action_trace)
    assert action_trace["requested_action_channels"] == []
    assert action_trace["achieved_effector_channels"] == []
    assert (result.output_dir / "resource_ledger.json").is_file()
    assert (result.output_dir / "source_provenance.json").is_file()
    _assert_unobserved_graph_execution(result.output_dir)
    ####


@pytest.mark.parametrize(
    ("request_name", "expected_control_realization"),
    (
        ("hl20_source_booster_release_replay_3dof_compose.yaml", "source_scheduled_force_model"),
        ("hl20_source_booster_release_replay_pseudo6dof_compose.yaml", "source_scheduled_named_attitude_response"),
    ),
)
def test_hl20_source_release_replay_runs_without_promoting_guidance_or_effectors(
    tmp_path: Path,
    request_name: str,
    expected_control_realization: str,
) -> None:
    result = execute_hl20_source_booster_release_composition(
        _composition(request_name),
        tmp_path / "hl20-source-release",
    )

    assert result.mission_pass is True
    assert result.preflight.status == "translation_ready"
    assert result.runtime["source_aerodynamic_graph"] is True
    assert result.runtime["participating_guidance_controller"] is False
    assert result.runtime["physical_effector_allocation"] is False
    assert result.runtime["direct_wrench_injection"] is False
    assert result.runtime["control_realization"] == expected_control_realization
    assert result.truth_evaluation["required_passed"] == 4
    action_trace = json.loads((result.output_dir / "semantic_action_trace.json").read_text(encoding="utf-8"))
    validate_committed_control_trace(result.composition, action_trace)
    assert action_trace["requested_action_channels"] == []
    assert action_trace["achieved_effector_channels"] == []
    assert (result.output_dir / "status_trace.json").is_file()
    provenance = json.loads((result.output_dir / "source_provenance.json").read_text(encoding="utf-8"))
    assert provenance["source_model_id"] == "hl20_mod_k_daveml_source_v1"
    assert provenance["source_package_sha256"]
    assert provenance["source_aerodynamics_sha256"]
    _assert_unobserved_graph_execution(result.output_dir)
    ####


def test_nesc_source_replay_preflight_rejects_an_unreplayable_launch_mass() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/nesc_staged_source_replay_3dof_compose.yaml")
    initialization = request.initialization.model_copy(
        update={
            "inputs": {
                **request.initialization.inputs,
                "initial_mass_kg": CompositionValue(value=313999.0, unit="kg"),
            }
        }
    )
    incompatible = compile_vehicle_composition(request.model_copy(update={"initialization": initialization}))

    result = preflight_vehicle_composition(incompatible)

    assert result.status == "blocked"
    assert "initial_mass_kg" in result.diagnostics[0]
    ####


@pytest.mark.parametrize(
    ("request_name", "expected_control_realization"),
    (
        ("x15_staged_booster_reachability_3dof_compose.yaml", "open_loop"),
        ("x15_staged_booster_reachability_pseudo6dof_compose.yaml", "response_law"),
    ),
)
def test_x15_staged_reachability_composition_preserves_source_staging_without_promoting_the_native_bridge(
    tmp_path: Path,
    request_name: str,
    expected_control_realization: str,
) -> None:
    result = execute_x15_staged_reachability_composition(
        _composition(request_name),
        tmp_path / "x15-staged-reachability",
    )

    assert result.mission_pass is True
    assert result.preflight.status == "translation_ready"
    assert result.runtime["control_realization"] == expected_control_realization
    assert result.runtime["participating_native_x15_plant"] is False
    assert result.runtime["direct_wrench_injection"] is False
    assert result.runtime["physical_effector_allocation"] is False
    assert result.envelope["phase_order"] == ["boost", "coast", "glide"]
    assert result.envelope["deployment_event_count"] == 1
    assert result.truth_evaluation["required_passed"] == 6
    assert result.truth_evaluation["terminal_pass"] is True
    assert all(item["reason"] == "EVENT_COMPLETE" for item in result.controller_transitions)
    evaluation = json.loads((result.output_dir / "evaluation.json").read_text(encoding="utf-8"))
    assert evaluation["outcome"] == "completed"
    assert all(metric["status"] == "pass" for metric in evaluation["metrics"])
    status_channels = cast(list[str], result.status_trace["channels"])
    assert "resources.booster.propellant.consumed" in status_channels
    status_samples = cast(list[dict[str, object]], result.status_trace["samples"])
    terminal = cast(dict[str, object], status_samples[-1]["values"])
    assert terminal["resources.booster.attached"] is False
    assert terminal["resources.booster.propellant.consumed"] == pytest.approx(2000.0)
    assert (result.output_dir / "status_trace.json").is_file()
    action_trace = json.loads((result.output_dir / "semantic_action_trace.json").read_text(encoding="utf-8"))
    validate_committed_control_trace(result.composition, action_trace)
    assert action_trace["requested_action_channels"] == []
    assert action_trace["achieved_effector_channels"] == []
    assert (result.output_dir / "resource_ledger.json").is_file()
    assert (result.output_dir / "source_provenance.json").is_file()
    assert (result.output_dir / "objective_report.json").is_file()
    assert "not prove a California-to-Hawaii trajectory" in result.claim_boundary
    _assert_unobserved_graph_execution(result.output_dir)
    ####


def test_x15_staged_reachability_preflight_rejects_a_nearby_unqualified_launch_elevation() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x15_staged_booster_reachability_3dof_compose.yaml")
    initialization = request.initialization.model_copy(
        update={
            "inputs": {
                **request.initialization.inputs,
                "launch_elevation_deg": CompositionValue(value=46.0, unit="deg"),
            }
        }
    )
    incompatible = compile_vehicle_composition(request.model_copy(update={"initialization": initialization}))

    result = preflight_vehicle_composition(incompatible)

    assert result.status == "blocked"
    assert "launch_elevation_deg" in result.diagnostics[0]
    ####


def test_x15_local_direct_wrench_screen_runs_through_composition_without_becoming_a_flight_mission(
    tmp_path: Path,
) -> None:
    result = execute_local_direct_wrench_composition(
        _composition("x15_local_direct_wrench_screen_compose.yaml"),
        tmp_path / "x15-local-direct-wrench-screen",
    )

    assert result.screen_pass is True
    assert result.preflight.status == "translation_ready"
    assert result.preflight.translator_id == "taoryx.x15_local_direct_wrench_screen.capability.v1"
    lowering = lower_vehicle_composition(result.composition, preflight_result=result.preflight)
    assert lowering.status == "adapter_bound"
    assert lowering.translator_status == "translation_ready"
    assert lowering.execution_binding is not None
    assert lowering.execution_binding["factory_id"] == "local_direct_wrench_screen.v1"
    assert result.screen.observed_statuses == ("feasible",)
    assert result.screen.equilibrium_pass is True
    assert "not prove X-15 flight trim" in result.claim_boundary
    channels = cast(list[str], result.status_trace["channels"])
    assert "control.wrench.achieved.moment" in channels
    assert "control.wrench.residual.force" in channels
    samples = cast(list[dict[str, object]], result.status_trace["samples"])
    terminal = cast(dict[str, object], samples[-1]["values"])
    assert terminal["control.wrench.status"] == "feasible"
    assert terminal["control.wrench.saturated"] is False
    action_trace = json.loads((result.output_dir / "semantic_action_trace.json").read_text(encoding="utf-8"))
    validate_committed_control_trace(result.composition, action_trace)
    assert action_trace["requested_action_channels"] == ["wrench.force.command", "wrench.moment.command"]
    first_action = cast(dict[str, object], cast(list[dict[str, object]], action_trace["samples"])[0]["requested_actions"])
    assert len(cast(list[object], first_action["wrench.force.command"])) == 3
    assert len(cast(list[object], first_action["wrench.moment.command"])) == 3
    assert (result.output_dir / "local_screen.json").is_file()
    assert (result.output_dir / "objective_report.json").is_file()
    objective_report = json.loads((result.output_dir / "objective_report.json").read_text(encoding="utf-8"))
    requirements = cast(list[dict[str, object]], objective_report["requirements"])
    assert {str(item["id"]): item["passed"] for item in requirements}["equilibrium_wrench_feasible"] is True
    assert {str(item["id"]): item["passed"] for item in requirements}["equilibrium_reference_derivative"] is True
    assert (result.output_dir / "status_trace.json").is_file()
    assert (result.output_dir / "resource_ledger.json").is_file()
    _assert_unobserved_graph_execution(result.output_dir)
    ####


def test_hl20_local_direct_wrench_screen_reuses_the_explicit_local_bridge_without_promoting_glide_evidence(
    tmp_path: Path,
) -> None:
    result = execute_local_direct_wrench_composition(
        _composition("hl20_local_direct_wrench_screen_compose.yaml"),
        tmp_path / "hl20-local-direct-wrench-screen",
    )

    assert result.screen_pass is True
    assert result.preflight.status == "translation_ready"
    assert result.preflight.translator_id == "taoryx.hl20_local_direct_wrench_screen.capability.v1"
    assert result.screen.equilibrium_pass is True
    assert result.screen.equilibrium_projection.status == "feasible"
    assert "not prove flight trim" in result.claim_boundary
    assert "high-energy performance" in result.claim_boundary
    objective_report = json.loads((result.output_dir / "objective_report.json").read_text(encoding="utf-8"))
    requirements = cast(list[dict[str, object]], objective_report["requirements"])
    assert all(item["passed"] is True for item in requirements)
    _assert_unobserved_graph_execution(result.output_dir)
    ####


@pytest.mark.parametrize(
    ("request_name", "expected_realized_fidelity", "expected_area_policy"),
    (
        ("tumbling_body_direct_release_3dof_compose.yaml", "point_mass_3dof", "orientation_averaged_projected_area"),
        (
            "tumbling_body_direct_release_pseudo6dof_compose.yaml",
            "rigid_body_6dof",
            "native_rigid_body_reuse_instantaneous_projected_area",
        ),
    ),
)
def test_passive_tumbling_composition_runs_without_a_controller_or_hidden_booster(
    tmp_path: Path,
    request_name: str,
    expected_realized_fidelity: str,
    expected_area_policy: str,
) -> None:
    result = execute_passive_tumbling_composition(
        _composition(request_name),
        tmp_path / "passive-tumbling",
    )

    assert result.mission_pass is True
    assert result.preflight.status == "translation_ready"
    assert result.runtime["control_realization"] == "uncontrolled"
    assert result.runtime["direct_wrench_injection"] is False
    assert result.runtime["physical_effector_allocation"] is False
    assert result.envelope["realized_fidelity"] == expected_realized_fidelity
    assert result.envelope["area_policy"] == expected_area_policy
    assert result.truth_evaluation["required_passed"] == 3
    assert result.truth_evaluation["terminal_pass"] is True
    assert all(item["reason"] == "EVENT_COMPLETE" for item in result.transitions)
    status_samples = cast(list[dict[str, object]], result.status_trace["samples"])
    first_status = cast(dict[str, object], status_samples[0]["values"])
    assert first_status["phase.mode"] == "ballistic"
    status_channels = cast(list[str], result.status_trace["channels"])
    assert "resources.mass.total" in status_channels
    assert (result.output_dir / "status_trace.json").is_file()
    action_trace = json.loads((result.output_dir / "semantic_action_trace.json").read_text(encoding="utf-8"))
    validate_committed_control_trace(result.composition, action_trace)
    assert action_trace["requested_action_channels"] == []
    assert action_trace["achieved_effector_channels"] == []
    assert (result.output_dir / "resource_ledger.json").is_file()
    assert (result.output_dir / "truth_telemetry.csv").is_file()
    assert (result.output_dir / "objective_report.json").is_file()
    assert (result.output_dir / "execution.json").is_file()
    _assert_unobserved_graph_execution(result.output_dir)
    ####


def test_passive_tumbling_preflight_rejects_an_area_policy_from_the_wrong_fidelity() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/tumbling_body_direct_release_3dof_compose.yaml")
    initialization = request.initialization.model_copy(
        update={
            "inputs": {
                **request.initialization.inputs,
                "area_policy": CompositionValue(value="native_rigid_body_reuse_instantaneous_projected_area"),
            }
        }
    )
    incompatible = compile_vehicle_composition(request.model_copy(update={"initialization": initialization}))

    result = preflight_vehicle_composition(incompatible)

    assert result.status == "blocked"
    assert "area_policy" in result.diagnostics[0]
    ####


def test_vehicle_run_cli_executes_the_x15_staged_witness_and_writes_its_interface_contract(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    composition = _composition("x15_staged_booster_reachability_pseudo6dof_compose.yaml")
    compiled = tmp_path / "x15-staged-composition.json"
    output = tmp_path / "x15-staged-run"
    composition.write_json(compiled)

    assert main(["vehicle", "run", str(compiled), "--output-dir", str(output)]) == 0
    payload = cast(dict[str, Any], json.loads(capsys.readouterr().out))

    assert payload["mission_pass"] is True
    assert payload["runtime"]["control_realization"] == "response_law"
    assert payload["vehicle_interface_artifact"]["path"].endswith("vehicle_interface.json")
    assert payload["vehicle_interface_artifact"]["interface_id"] == "x15/pseudo_6dof"
    assert (output / "vehicle_interface.json").is_file()
    ####


def test_vehicle_endpoints_cli_exposes_hummingbird_batch_and_episode_bindings(
    capsys: pytest.CaptureFixture[str],
) -> None:
    assert main(["vehicle", "endpoints", "hummingbird"]) == 0
    payload = cast(dict[str, Any], json.loads(capsys.readouterr().out))

    assert payload["family_id"] == "hummingbird"
    bindings = cast(list[dict[str, object]], payload["execution_bindings"])
    parity = cast(list[dict[str, object]], payload["batch_episode_parity"])
    assert {item["operation"] for item in bindings if item["status"] == "runnable"} == {"batch", "episode"}
    assert all(item["factory_id"] is not None for item in bindings if item["status"] == "runnable")
    pseudo = next(item for item in parity if item["fidelity"] == "pseudo_6dof")
    assert pseudo["availability"] == "registered"
    ####
