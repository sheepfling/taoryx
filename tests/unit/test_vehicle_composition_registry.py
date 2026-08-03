from __future__ import annotations

import json
from pathlib import Path
from typing import Any, cast

import pytest

from taoryx.language_backed_execution import execute_powered_fixed_wing_composition
from taoryx.reduced_fixed_wing_execution import execute_reduced_fixed_wing_composition
from taoryx.runtime.cli import main
from taoryx.vehicle_composition import (
    VehicleCompositionError,
    compile_vehicle_composition,
    load_vehicle_composition_request,
)
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog
from taoryx.vehicle_execution_preflight import preflight_vehicle_composition
from taoryx.vehicle_runtime_lowering import lower_vehicle_composition

ROOT = Path(__file__).resolve().parents[2]


def test_composition_registry_covers_every_unified_family() -> None:
    catalog = load_resolved_vehicle_composition_catalog()

    assert len(catalog.vehicles) == 9
    assert {item.family.family_id for item in catalog.vehicles} == {
        "skywalker_x8",
        "b747",
        "a320_openap_3dof",
        "f16_s119",
        "x15",
        "hummingbird",
        "hl20_mod_k",
        "reference_nesc_two_stage_rocket",
        "tumbling_body",
    }
    ####


def test_x8_composition_exposes_four_tiers_and_racetrack_recipe() -> None:
    x8 = cast(dict[str, Any], load_resolved_vehicle_composition_catalog().vehicle("skywalker_x8").as_dict())

    assert set(x8["fidelities"]) == {
        "point_mass_3dof",
        "pseudo_6dof",
        "rigid_body_6dof_direct_wrench",
        "rigid_body_6dof_surface_allocated",
    }
    assert x8["fidelities"]["rigid_body_6dof_direct_wrench"]["control_realization"] == "direct_wrench"
    assert x8["fidelities"]["rigid_body_6dof_surface_allocated"]["control_realization"] == "surface_allocated"
    assert x8["interfaces"]["pseudo_6dof"]["available_authority_profiles"] == ["native_control_bridge"]
    assert x8["interfaces"]["pseudo_6dof"]["available_observation_profiles"] == ["truth_debug"]
    assert x8["initialization_contracts"][0]["id"] == "airborne_trim"
    racetrack = x8["mission_templates"][0]
    assert racetrack["id"] == "powered_fixed_wing_racetrack_v1"
    assert racetrack["segment_sequence"][-1] == "terminal_state_gate"
    ####


def test_passive_tumbling_body_hides_inapplicable_actuator_tiers() -> None:
    tumbling = cast(dict[str, Any], load_resolved_vehicle_composition_catalog().vehicle("tumbling_body").as_dict())

    assert tumbling["fidelities"]["rigid_body_6dof_direct_wrench"]["declared"] is False
    assert tumbling["fidelities"]["rigid_body_6dof_surface_allocated"]["declared"] is False
    assert tumbling["fidelities"]["pseudo_6dof"]["control_realization"] == "uncontrolled"
    assert tumbling["mission_templates"][0]["compatible_fidelities"] == ["point_mass_3dof", "pseudo_6dof"]
    ####


def test_vehicle_cli_lists_and_exports_composition_sections(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["vehicle", "list"]) == 0
    listing = json.loads(capsys.readouterr().out)
    assert len(listing) == 9
    assert any(item["family_id"] == "hummingbird" for item in listing)

    assert main(["vehicle", "schema", "b747", "segments"]) == 0
    schema = json.loads(capsys.readouterr().out)
    assert schema["family_id"] == "b747"
    assert any(item["id"] == "fly_by_turn" for item in schema["segment_contracts"])
    ####


def test_compose_x8_racetrack_creates_immutable_semantic_adapter_handoff() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_compose.yaml")
    compiled = compile_vehicle_composition(request)

    assert compiled.vehicle_id == "skywalker_x8"
    assert compiled.fidelity == "pseudo_6dof"
    assert compiled.composition_status == "development"
    assert compiled.segments[2].instance_id == "left-turn"
    assert compiled.segments[2].state_transfer == "previous_terminal_truth_state"
    assert compiled.native_adapter_handoff["adapter_id"] == "taoryx.fixed_wing.source_table.v1"
    assert compiled.native_adapter_handoff["handoff_status"] == "semantic_compiled_runtime_lowering_pending"
    assert len(compiled.identity_sha256) == 64
    ####


def test_compose_rejects_missing_or_reordered_required_segments() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_compose.yaml")
    missing_input = request.model_copy(update={"initialization": request.initialization.model_copy(update={"inputs": {}})})
    with pytest.raises(VehicleCompositionError, match="missing-input"):
        compile_vehicle_composition(missing_input)

    reordered = request.model_copy(update={"segments": tuple(reversed(request.segments))})
    with pytest.raises(VehicleCompositionError, match="segment-sequence-mismatch"):
        compile_vehicle_composition(reordered)
    ####


def test_vehicle_compose_cli_writes_reproducible_output(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    output = tmp_path / "composition.json"

    assert main(["vehicle", "compose", str(ROOT / "examples/vehicle_composition/x8_racetrack_compose.yaml"), "--output", str(output)]) == 0
    emitted = json.loads(capsys.readouterr().out)
    written = json.loads(output.read_text(encoding="utf-8"))
    assert written == emitted
    assert emitted["mission"] == "powered_fixed_wing_racetrack_v1"
    ####


def test_vehicle_materialize_requires_preflight_ready_geometry(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    composition_path = tmp_path / "composition.json"
    output_dir = tmp_path / "native-inputs"
    request = ROOT / "examples/vehicle_composition/x8_racetrack_capability_compose.yaml"
    assert main(["vehicle", "compose", str(request), "--output", str(composition_path)]) == 0
    _ = capsys.readouterr()

    assert main(["vehicle", "materialize", str(composition_path), "--output-dir", str(output_dir)]) == 0
    payload = json.loads(capsys.readouterr().out)

    assert payload["source_mission_id"] == "x8-racetrack-altitude-turns-pseudo-6dof-v1"
    assert payload["proposal"]["derived"]["selected_straight_length_m"] == 1000.0
    assert Path(payload["inputs"]["problem"]).is_file()
    assert Path(payload["inputs"]["mission_config"]).is_file()
    assert Path(payload["inputs"]["racetrack_config"]).is_file()
    ####


def test_vehicle_run_executes_x8_translation_ready_composition(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    composition_path = tmp_path / "composition.json"
    output_dir = tmp_path / "execution"
    request = ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml"

    assert main(["vehicle", "compose", str(request), "--output", str(composition_path)]) == 0
    _ = capsys.readouterr()
    assert main(["vehicle", "run", str(composition_path), "--output-dir", str(output_dir)]) == 0
    payload = json.loads(capsys.readouterr().out)

    assert payload["mission_pass"] is True
    assert payload["preflight"]["status"] == "translation_ready"
    assert payload["truth_evaluation"]["mission_pass"] is True
    assert (output_dir / "truth_telemetry.csv").is_file()
    assert (output_dir / "objective_report.json").is_file()
    assert (output_dir / "execution.json").is_file()
    interface = json.loads((output_dir / "vehicle_interface.json").read_text(encoding="utf-8"))
    assert payload["vehicle_interface_artifact"]["fingerprint_sha256"] == interface["fingerprint_sha256"]
    assert interface["schema"] == "taoryx.vehicle-interface/v1alpha1"
    assert interface["validation"] == {"status": "pass", "findings": []}
    ####


def test_vehicle_run_executes_b747_point_mass_translation_ready_composition(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    composition_path = tmp_path / "b747-composition.json"
    output_dir = tmp_path / "b747-execution"
    request = ROOT / "examples/vehicle_composition/b747_racetrack_capability_3dof_compose.yaml"

    assert main(["vehicle", "compose", str(request), "--output", str(composition_path)]) == 0
    _ = capsys.readouterr()
    assert main(["vehicle", "run", str(composition_path), "--output-dir", str(output_dir)]) == 0
    payload = json.loads(capsys.readouterr().out)

    assert payload["mission_pass"] is True
    assert payload["composition"]["fidelity"] == "point_mass_3dof"
    assert payload["preflight"]["status"] == "translation_ready"
    assert payload["truth_evaluation"]["mission_pass"] is True
    assert (output_dir / "truth_telemetry.csv").is_file()
    assert (output_dir / "objective_report.json").is_file()
    interface = json.loads((output_dir / "vehicle_interface.json").read_text(encoding="utf-8"))
    assert interface["fidelity"] == "point_mass_3dof"
    assert interface["validation"] == {"status": "pass", "findings": []}
    ####


def test_x8_sensor_composition_writes_a_no_interpolation_batch_trace(tmp_path: Path) -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_sensor_episode_3dof_compose.yaml")
    )

    result = execute_powered_fixed_wing_composition(
        composition,
        tmp_path / "x8-sensor-execution",
        max_steps=3,
    )

    assert result.sensor_trace is not None
    assert result.sensor_trace["interpolation"] == "forbidden"
    samples = cast(list[dict[str, object]], result.sensor_trace["samples"])
    assert samples[0]["source_time_s"] is None
    assert samples[1]["source_time_s"] == pytest.approx(0.0)
    assert (result.output_dir / "sensor_observations.json").is_file()
    ####


def test_vehicle_run_executes_a320_reduced_composition(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    composition_path = tmp_path / "composition.json"
    output_dir = tmp_path / "execution"
    request = ROOT / "examples/vehicle_composition/a320_racetrack_capability_pseudo6dof_compose.yaml"

    assert main(["vehicle", "compose", str(request), "--output", str(composition_path)]) == 0
    _ = capsys.readouterr()
    assert main(["vehicle", "run", str(composition_path), "--output-dir", str(output_dir)]) == 0
    payload = json.loads(capsys.readouterr().out)

    assert payload["mission_pass"] is True
    assert payload["runtime"]["adapter_id"] == "taoryx.fixed_wing.openap.v1"
    assert "trim" in payload
    assert "provenance" in payload
    assert (output_dir / "trim.json").is_file()
    assert (output_dir / "model_provenance.json").is_file()
    ####


@pytest.mark.parametrize(
    ("request_name", "expected_family"),
    (
        ("a320_racetrack_capability_3dof_compose.yaml", "a320_openap_3dof"),
        ("a320_racetrack_capability_pseudo6dof_compose.yaml", "a320_openap_3dof"),
        ("f16_racetrack_capability_3dof_compose.yaml", "f16_s119"),
        ("f16_racetrack_capability_pseudo6dof_compose.yaml", "f16_s119"),
    ),
)
def test_reduced_airbreather_compositions_execute_the_common_racetrack(
    tmp_path: Path,
    request_name: str,
    expected_family: str,
) -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / request_name)
    )

    result = execute_reduced_fixed_wing_composition(composition, tmp_path / request_name.removesuffix(".yaml"))

    assert result.composition.family_id == expected_family
    assert result.mission_pass is True
    assert result.truth_evaluation["required_passed"] == 4
    status_channels = cast(list[str], result.status_trace["channels"])
    assert {"position.north", "position.east", "position.altitude", "velocity.speed", "resources.mass.total"} <= set(status_channels)
    status_samples = cast(list[dict[str, object]], result.status_trace["samples"])
    first_status = cast(dict[str, object], status_samples[0]["values"])
    assert float(first_status["velocity.speed"]) > 0.0
    assert float(first_status["resources.mass.total"]) > 0.0
    if composition.fidelity == "pseudo_6dof":
        assert "attitude.euler" in first_status
        assert "body_rate" in first_status
    assert (result.output_dir / "truth_telemetry.csv").is_file()
    assert (result.output_dir / "execution.json").is_file()
    ####


def test_runtime_lowering_never_falls_back_to_another_family_adapter() -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_compose.yaml")
    )

    result = lower_vehicle_composition(composition)

    assert result.status == "blocked"
    assert result.adapter_descriptor is None
    assert "skywalker_x8" in result.diagnostics[0]
    ####


def test_x8_legacy_semantic_example_is_blocked_when_its_route_does_not_match_the_native_translator() -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_compose.yaml")
    )

    result = preflight_vehicle_composition(composition)

    assert result.status == "blocked"
    failed = {check.id for check in result.checks if not check.passed}
    assert "initialization.heading" in failed
    assert "left_turn.radius" in failed
    assert "right_turn.exit_gate" in failed
    ####


def test_x8_capability_derived_composition_is_ready_for_native_route_translation() -> None:
    for path in (
        ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml",
        ROOT / "examples/vehicle_composition/x8_racetrack_capability_compose.yaml",
    ):
        composition = compile_vehicle_composition(load_vehicle_composition_request(path))
        result = preflight_vehicle_composition(composition)

        assert result.status == "translation_ready"
        assert all(check.passed for check in result.checks)
        assert result.translator_id == "taoryx.powered_fixed_wing_racetrack.capability_scaled.v1"
    ####


def test_b747_capability_derived_composition_reuses_the_fixed_wing_translator(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/b747_racetrack_capability_pseudo6dof_compose.yaml")
    )

    result = preflight_vehicle_composition(composition)
    assert result.status == "translation_ready"
    assert all(check.passed for check in result.checks)
    assert result.derived_mission is not None
    assert result.derived_mission["resolved_route"]["vehicle_id"] == "b747"

    output_dir = tmp_path / "native-inputs"
    assert main(["vehicle", "compose", str(ROOT / "examples/vehicle_composition/b747_racetrack_capability_pseudo6dof_compose.yaml"), "--output", str(tmp_path / "composition.json")]) == 0
    _ = capsys.readouterr()
    assert main(["vehicle", "materialize", str(tmp_path / "composition.json"), "--output-dir", str(output_dir)]) == 0
    materialized = json.loads(capsys.readouterr().out)
    assert materialized["source_mission_id"] == "b747-racetrack-altitude-turns-pseudo-6dof-v1"
    assert materialized["proposal"]["derived"]["selected_straight_length_m"] == 30000.0
    assert Path(materialized["inputs"]["problem"]).is_file()
    ####


def test_runtime_lowering_binds_an_available_source_adapter() -> None:
    x15_composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x15_direct_wrench_compose.yaml")
    )

    result = lower_vehicle_composition(x15_composition)

    assert result.status == "adapter_bound"
    assert result.adapter_descriptor is not None
    assert result.adapter_descriptor["adapter_id"] == "taoryx.high_energy.fixed_wing.v1"
    assert result.translator_status == "pending"
    ####
