"""Validation of the explicit compose-to-execution binding catalog."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Literal, cast

import pytest

from taoryx.hummingbird_composition_execution import execute_hummingbird_pseudo_composition
from taoryx.local_direct_wrench_composition_execution import execute_local_direct_wrench_composition
from taoryx.nesc_composition_execution import execute_nesc_source_replay_composition
from taoryx.passive_tumbling_composition_execution import execute_passive_tumbling_composition
from taoryx.runtime.cli import main
from taoryx.vehicle_composition import (
    CompiledVehicleComposition,
    CompositionValue,
    compile_vehicle_composition,
    load_vehicle_composition_request,
)
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog
from taoryx.vehicle_execution_bindings import (
    VehicleExecutionBindingError,
    load_vehicle_execution_binding_catalog,
    resolve_vehicle_execution_binding,
    validate_execution_bindings,
)
from taoryx.vehicle_execution_preflight import preflight_vehicle_composition
from taoryx.x15_staged_composition_execution import execute_x15_staged_reachability_composition

ROOT = Path(__file__).resolve().parents[2]


def _composition(name: str) -> CompiledVehicleComposition:
    return compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / name))
    ####


def _declared_missions() -> dict[str, dict[str, set[str]]]:
    catalog = load_resolved_vehicle_composition_catalog()
    return {
        vehicle.family.family_id: {
            mission.id: set(mission.compatible_fidelities) for mission in vehicle.declaration.mission_templates
        }
        for vehicle in catalog.vehicles
    }
    ####


def test_execution_binding_catalog_references_declared_family_missions_and_tiers() -> None:
    catalog = load_vehicle_execution_binding_catalog()

    assert validate_execution_bindings(catalog.bindings, declarations=_declared_missions()) == ()
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
        ("x15_staged_booster_reachability_3dof_compose.yaml", "batch", "x15_staged_reachability.v1"),
        ("x15_local_direct_wrench_screen_compose.yaml", "batch", "local_direct_wrench_screen.v1"),
        ("tumbling_body_direct_release_pseudo6dof_compose.yaml", "batch", "passive_tumbling_direct_release.v1"),
        ("hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml", "episode", "hummingbird_aggregate_thrust_episode.v1"),
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
    composition = _composition("a320_racetrack_capability_3dof_compose.yaml")

    with pytest.raises(VehicleExecutionBindingError, match="no episode execution binding"):
        resolve_vehicle_execution_binding(composition, "episode")
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
    assert (result.output_dir / "status_trace.json").is_file()
    assert (result.output_dir / "objective_report.json").is_file()
    assert (result.output_dir / "controller_transitions.json").is_file()
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
    assert (result.output_dir / "source_provenance.json").is_file()
    ####


def test_nesc_source_replay_preflight_rejects_an_unreplayable_launch_mass() -> None:
    request = load_vehicle_composition_request(
        ROOT / "examples/vehicle_composition/nesc_staged_source_replay_3dof_compose.yaml"
    )
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
    status_channels = cast(list[str], result.status_trace["channels"])
    assert "resources.booster.propellant.consumed" in status_channels
    status_samples = cast(list[dict[str, object]], result.status_trace["samples"])
    terminal = cast(dict[str, object], status_samples[-1]["values"])
    assert terminal["resources.booster.attached"] is False
    assert terminal["resources.booster.propellant.consumed"] == pytest.approx(2000.0)
    assert (result.output_dir / "status_trace.json").is_file()
    assert (result.output_dir / "source_provenance.json").is_file()
    assert (result.output_dir / "objective_report.json").is_file()
    assert "not prove a California-to-Hawaii trajectory" in result.claim_boundary
    ####


def test_x15_staged_reachability_preflight_rejects_a_nearby_unqualified_launch_elevation() -> None:
    request = load_vehicle_composition_request(
        ROOT / "examples/vehicle_composition/x15_staged_booster_reachability_3dof_compose.yaml"
    )
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
    assert result.preflight.translator_id == "taoryx.local_direct_wrench_screen.v1"
    assert result.screen.observed_statuses == ("feasible",)
    assert "not prove X-15 flight trim" in result.claim_boundary
    channels = cast(list[str], result.status_trace["channels"])
    assert "control.wrench.achieved.moment" in channels
    assert "control.wrench.residual.force" in channels
    samples = cast(list[dict[str, object]], result.status_trace["samples"])
    terminal = cast(dict[str, object], samples[-1]["values"])
    assert terminal["control.wrench.status"] == "feasible"
    assert terminal["control.wrench.saturated"] is False
    assert (result.output_dir / "local_screen.json").is_file()
    assert (result.output_dir / "objective_report.json").is_file()
    assert (result.output_dir / "status_trace.json").is_file()
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
    assert (result.output_dir / "truth_telemetry.csv").is_file()
    assert (result.output_dir / "objective_report.json").is_file()
    assert (result.output_dir / "execution.json").is_file()
    ####


def test_passive_tumbling_preflight_rejects_an_area_policy_from_the_wrong_fidelity() -> None:
    request = load_vehicle_composition_request(
        ROOT / "examples/vehicle_composition/tumbling_body_direct_release_3dof_compose.yaml"
    )
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
    assert {item["operation"] for item in bindings if item["status"] == "runnable"} == {"batch", "episode"}
    assert all(item["factory_id"] is not None for item in bindings if item["status"] == "runnable")
    ####
