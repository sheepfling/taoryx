from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.ads6_engagement import (
    Ads6EngagementRunConfig,
    load_ads6_engagement_source_definition,
    run_ads6_engagement,
)
from taoryx.families.cadac.ads6_sam import (
    Ads6SamDirectCommand,
    Ads6SamPlantStepper,
    lower_ads6_sam_actor,
)
from taoryx.families.cadac.ads6_sam_controller import (
    Ads6SamControllerContext,
    Ads6SamSourceController,
    ads6_sam_aerodynamic_derivatives,
    lower_ads6_sam_controller_actor,
)
from taoryx.families.cadac.bundle import load_cadac_source_bundle
from test_ads6_engagement import _write_source_controller_aircraft_engagement


def test_ads6_sam_controller_lowers_source_modes_and_explicit_evidence_boundary(tmp_path: Path) -> None:
    source_path = _write_source_controller_aircraft_engagement(tmp_path)
    bundle = load_cadac_source_bundle(source_path)
    vehicle = bundle.case.vehicles_named("MISSILE6")[0]
    plant = lower_ads6_sam_actor(bundle, vehicle)
    controller = lower_ads6_sam_controller_actor(
        bundle,
        vehicle,
        plant,
        selected_phase="fin_control",
    )

    assert controller.sensor.initial_mode == 12
    assert controller.sensor.target_kind == "aircraft"
    assert controller.sensor.rf_tracking_gain_per_s == pytest.approx(5.0)
    assert controller.guidance.initial_mode == 0
    assert controller.autopilot.initial_mode == 2
    assert tuple(event.condition.variable for event in controller.events) == ("msl_time", "mseek")
    assert "truth-aligned INS" in controller.claim_boundary
    assert "RF glint/thermal-noise" in controller.claim_boundary


####


def test_ads6_sam_controller_consumes_the_physical_plants_aerodynamic_derivative_ledger(tmp_path: Path) -> None:
    source_path = _write_source_controller_aircraft_engagement(tmp_path)
    bundle = load_cadac_source_bundle(source_path)
    vehicle = bundle.case.vehicles_named("MISSILE6")[0]
    plant_definition = lower_ads6_sam_actor(bundle, vehicle)
    plant = Ads6SamPlantStepper(plant_definition)
    plant.step(0.0, Ads6SamDirectCommand(phase="fin_control"))
    observation = plant.observation(0.0)

    derivatives = ads6_sam_aerodynamic_derivatives(plant_definition, observation)

    assert derivatives.normal_alpha_mps2 == pytest.approx(observation.coefficients.normal_alpha_derivative_mps2)
    assert derivatives.pitch_control_rad_s2 == pytest.approx(observation.coefficients.pitch_control_derivative_rad_s2)
    assert derivatives.yaw_control_rad_s2 == pytest.approx(observation.coefficients.yaw_control_derivative_rad_s2)
    assert abs(derivatives.normal_alpha_mps2) > 0.0
    assert abs(derivatives.pitch_control_rad_s2) > 0.0


####


def test_ads6_sam_controller_interleaves_source_events_sensor_guidance_and_physical_fins(tmp_path: Path) -> None:
    source_path = _write_source_controller_aircraft_engagement(tmp_path)
    bundle = load_cadac_source_bundle(source_path)
    vehicle = bundle.case.vehicles_named("MISSILE6")[0]
    plant_definition = lower_ads6_sam_actor(bundle, vehicle)
    controller_definition = lower_ads6_sam_controller_actor(
        bundle,
        vehicle,
        plant_definition,
        selected_phase="fin_control",
    )
    plant = Ads6SamPlantStepper(plant_definition)
    controller = Ads6SamSourceController(controller_definition, plant_definition)
    context = Ads6SamControllerContext(
        missile_index=1,
        target_actor_id="a1",
        target_kind="aircraft",
        target_position_ned_m=(0.0, 1000.0, -1000.0),
        target_velocity_ned_mps=(0.0, -250.0, 0.0),
        target_packet_epoch_s=0.0,
        intercept_point_ned_m=(0.0, 1000.0, -1000.0),
        radar_packet_epoch_s=0.0,
    )

    for index in range(5):
        time_s = index * plant_definition.integration_step_s
        plant.step_controlled(time_s, time_s, controller, context)
    ####

    controller_sample = controller.sample()
    plant_sample = plant.sample(0.04)
    assert controller_sample.control_mode == 3
    assert controller_sample.sensor_mode == 14
    assert controller_sample.guidance_mode == 7
    assert controller_sample.source_event_count == 2
    assert controller_sample.requested_control_deg == pytest.approx(plant_sample.requested_control_deg)
    assert max(abs(value) for value in plant_sample.achieved_fins_deg) > 0.0
    assert controller_sample.target_actor_id == "a1"
    assert controller_sample.rf_measurement_boundary == "deterministic_no_glint_or_thermal_noise"


####


def test_ads6_package_source_controller_preserves_packet_latency_and_mode_transitions(tmp_path: Path) -> None:
    definition = load_ads6_engagement_source_definition(_write_source_controller_aircraft_engagement(tmp_path))
    result = run_ads6_engagement(
        definition,
        Ads6EngagementRunConfig(
            end_time_s=0.10,
            sample_step_s=0.01,
            command_law="source_controller",
        ),
    )

    controller_events = tuple(event for event in result.events if event.kind == "sam_controller_event")
    mode_events = tuple(event for event in result.events if event.kind == "sam_mode_transition")
    seeker_lock = next(event for event in result.events if event.kind == "seeker_lock")
    missile = result.objects[0]
    terminal_sample = missile.samples[-1]

    assert tuple(event.details["watch_variable"] for event in controller_events) == (
        "msl_time",
        "mseek",
    )
    assert any(event.details["subsystem"] == "sensor" and event.details["mode"] == 14 for event in mode_events)
    assert any(event.details["subsystem"] == "guidance" and event.details["mode"] == 7 for event in mode_events)
    assert seeker_lock.time_s == pytest.approx(0.03)
    terminal_event = next(event for event in controller_events if event.details["watch_variable"] == "mseek")
    assert terminal_event.time_s == pytest.approx(0.04)
    assert terminal_event.time_s == pytest.approx(seeker_lock.time_s + definition.integration_step_s)
    assert terminal_sample.telemetry["sensor_mode"] == 14
    assert terminal_sample.telemetry["guidance_mode"] == 7
    assert terminal_sample.telemetry["controller_mode"] == 3
    assert terminal_sample.telemetry["source_controller_event_count"] == 2
    assert terminal_sample.telemetry["controller_requested_control_deg"] == pytest.approx(terminal_sample.telemetry["requested_control_deg"])
    assert float(terminal_sample.telemetry["target_packet_epoch_s"]) < terminal_sample.time_s
    assert max(abs(value) for value in terminal_sample.telemetry["achieved_control_deg"]) > 0.0


####


def test_ads6_ir_source_controller_locks_and_switches_to_compensated_terminal_pronav(tmp_path: Path) -> None:
    source_path = _write_source_controller_aircraft_engagement(tmp_path)
    source_text = source_path.read_text(encoding="utf-8")
    source_text = source_text.replace(
        "  mseek 12\n  skr_dyn 1\n  racq_rf 5000\n  dtimac_rf 0.01\n  forlim_rfx 180\n  fovlim_rfx 180\n  gain_rf 10\n  gain_rf 5\n",
        "  mseek 22\n"
        "  skr_dyn 1\n"
        "  racq_ir 5000\n"
        "  dtimac_ir 0.01\n"
        "  fovyaw_ir 3.14\n"
        "  fovpitch_ir 3.14\n"
        "  gk 10\n"
        "  wnk 50\n"
        "  zetak 0.9\n"
        "  trtht 3.14\n"
        "  trthtd 100\n"
        "  trphid 100\n"
        "  trate 100\n",
    )
    source_text = source_text.replace(
        "  IF mseek = 14\n   mguide 7\n",
        "  IF mseek = 24\n   mguide 6\n",
    )
    ir_source_path = tmp_path / "input-ir-source-controller.asc"
    ir_source_path.write_text(source_text, encoding="utf-8")

    definition = load_ads6_engagement_source_definition(ir_source_path)
    result = run_ads6_engagement(
        definition,
        Ads6EngagementRunConfig(
            end_time_s=0.10,
            sample_step_s=0.01,
            command_law="source_controller",
        ),
    )

    seeker_lock = next(event for event in result.events if event.kind == "seeker_lock")
    terminal_event = next(event for event in result.events if event.kind == "sam_controller_event" and event.details["watch_variable"] == "mseek")
    terminal_sample = result.objects[0].samples[-1]

    assert seeker_lock.time_s == pytest.approx(0.03)
    assert seeker_lock.details["sensor_kind"] == "ir"
    assert terminal_event.time_s == pytest.approx(0.04)
    assert terminal_sample.telemetry["sensor_mode"] == 24
    assert terminal_sample.telemetry["guidance_mode"] == 6
    assert terminal_sample.telemetry["ir_measurement_boundary"] == ("deterministic_no_focal_plane_or_aimpoint_corruption")
    assert max(abs(value) for value in terminal_sample.telemetry["achieved_control_deg"]) > 0.0


####


def test_ads6_rf_kinematic_seeker_locks_after_time_delay_without_dynamic_for_gate(tmp_path: Path) -> None:
    source_path = _write_source_controller_aircraft_engagement(tmp_path)
    bundle = load_cadac_source_bundle(source_path)
    vehicle = bundle.case.vehicles_named("MISSILE6")[0]
    plant_definition = lower_ads6_sam_actor(bundle, vehicle)
    original = lower_ads6_sam_controller_actor(
        bundle,
        vehicle,
        plant_definition,
        selected_phase="fin_control",
    )
    sensor = original.sensor.model_copy(
        update={
            "dynamic_mode": 0,
            "rf_acquisition_time_s": 0.0,
            "rf_field_of_regard_deg": 0.0,
        }
    )
    definition = original.model_copy(update={"sensor": sensor, "events": ()})
    plant = Ads6SamPlantStepper(plant_definition)
    controller = Ads6SamSourceController(definition, plant_definition)
    context = Ads6SamControllerContext(
        missile_index=1,
        target_actor_id="a1",
        target_kind="aircraft",
        target_position_ned_m=(0.0, 1000.0, -1000.0),
        target_velocity_ned_mps=(0.0, -250.0, 0.0),
        target_packet_epoch_s=0.0,
        intercept_point_ned_m=(0.0, 1000.0, -1000.0),
        radar_packet_epoch_s=0.0,
    )

    plant.step_controlled(0.0, 0.0, controller, context)
    plant.step_controlled(0.01, 0.01, controller, context)

    sample = controller.sample()
    assert sample.sensor_mode == 14
    assert sample.tracking_error_pitch_yaw_rad == pytest.approx((0.0, 0.0))


####


def test_ads6_rate_controller_ignores_declared_but_unused_source_rate_commands(tmp_path: Path) -> None:
    source_path = _write_source_controller_aircraft_engagement(tmp_path)
    bundle = load_cadac_source_bundle(source_path)
    vehicle = bundle.case.vehicles_named("MISSILE6")[0]
    plant_definition = lower_ads6_sam_actor(bundle, vehicle)
    base_definition = lower_ads6_sam_controller_actor(
        bundle,
        vehicle,
        plant_definition,
        selected_phase="fin_control",
    )
    commanded_autopilot = base_definition.autopilot.model_copy(
        update={
            "pitch_rate_command_deg_s": 999.0,
            "yaw_rate_command_deg_s": -999.0,
        }
    )
    commanded_definition = base_definition.model_copy(update={"autopilot": commanded_autopilot})
    plant = Ads6SamPlantStepper(plant_definition)
    plant.step(0.0, Ads6SamDirectCommand(phase="fin_control"))
    observation = plant.observation(0.0)
    derivatives = ads6_sam_aerodynamic_derivatives(plant_definition, observation)
    base = Ads6SamSourceController(base_definition, plant_definition)
    commanded = Ads6SamSourceController(commanded_definition, plant_definition)

    assert commanded._rate_control(observation, derivatives) == pytest.approx(base._rate_control(observation, derivatives))


####


def test_ads6_acceleration_controller_reuses_pitch_pole_pair_for_yaw_axis(tmp_path: Path) -> None:
    source_path = _write_source_controller_aircraft_engagement(tmp_path)
    bundle = load_cadac_source_bundle(source_path)
    vehicle = bundle.case.vehicles_named("MISSILE6")[0]
    plant_definition = lower_ads6_sam_actor(bundle, vehicle)
    definition = lower_ads6_sam_controller_actor(
        bundle,
        vehicle,
        plant_definition,
        selected_phase="fin_control",
    )
    plant = Ads6SamPlantStepper(plant_definition)
    plant.step(0.0, Ads6SamDirectCommand(phase="fin_control"))
    observation = plant.observation(0.0)
    derivatives = ads6_sam_aerodynamic_derivatives(plant_definition, observation)
    altered = derivatives.model_copy(
        update={
            "yaw_real_root_1_rad_s": derivatives.yaw_real_root_1_rad_s * 1000.0 + 123.0,
            "yaw_real_root_2_rad_s": derivatives.yaw_real_root_2_rad_s * 1000.0 - 321.0,
        }
    )
    source = Ads6SamSourceController(definition, plant_definition)
    changed = Ads6SamSourceController(definition, plant_definition)
    source._normal_command_g = 1.0
    source._lateral_command_g = 1.0
    changed._normal_command_g = 1.0
    changed._lateral_command_g = 1.0

    assert changed._acceleration_control(observation, altered) == pytest.approx(source._acceleration_control(observation, derivatives))


####


def test_ads6_ir_kinematic_terminal_controller_drives_physical_fins(tmp_path: Path) -> None:
    source_path = _write_source_controller_aircraft_engagement(tmp_path)
    bundle = load_cadac_source_bundle(source_path)
    vehicle = bundle.case.vehicles_named("MISSILE6")[0]
    plant_definition = lower_ads6_sam_actor(bundle, vehicle)
    original = lower_ads6_sam_controller_actor(
        bundle,
        vehicle,
        plant_definition,
        selected_phase="fin_control",
    )
    sensor = original.sensor.model_copy(
        update={
            "initial_mode": 22,
            "dynamic_mode": 0,
            "target_kind": "srbm",
            "ir_acquisition_range_m": 5000.0,
            "ir_acquisition_time_s": 0.0,
        }
    )
    guidance = original.guidance.model_copy(
        update={
            "initial_mode": 6,
            "navigation_gain": 3.5,
            "navigation_gain_state": 3.5,
            "navigation_gain_time_constant_s": 0.0,
        }
    )
    autopilot = original.autopilot.model_copy(update={"initial_mode": 3})
    definition = original.model_copy(
        update={
            "sensor": sensor,
            "guidance": guidance,
            "autopilot": autopilot,
            "events": (),
        }
    )
    plant = Ads6SamPlantStepper(plant_definition)
    controller = Ads6SamSourceController(definition, plant_definition)
    context = Ads6SamControllerContext(
        missile_index=1,
        target_actor_id="r1",
        target_kind="srbm",
        target_position_ned_m=(100.0, 1200.0, -800.0),
        target_velocity_ned_mps=(-50.0, -300.0, 25.0),
        target_packet_epoch_s=0.0,
        intercept_point_ned_m=(100.0, 1200.0, -800.0),
        radar_packet_epoch_s=0.0,
    )

    plant.step_controlled(0.0, 0.0, controller, context)
    plant.step_controlled(0.01, 0.01, controller, context)
    plant.step_controlled(0.02, 0.02, controller, context)

    controller_sample = controller.sample()
    plant_sample = plant.sample(0.02)
    assert controller_sample.sensor_mode == 24
    assert controller_sample.sensor_kind == "ir"
    assert controller_sample.guidance_mode == 6
    assert max(abs(value) for value in plant_sample.achieved_fins_deg) > 0.0
    assert controller_sample.ir_measurement_boundary == ("deterministic_no_focal_plane_or_aimpoint_corruption")


####
