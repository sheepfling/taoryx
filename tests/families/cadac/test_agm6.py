from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from taoryx.families.cadac.agm6 import (
    Agm6ActuatorConfig,
    Agm6ControlCommand,
    Agm6FinActuatorState,
    Agm6PropulsionStep,
    _ActorPacket,
    _AircraftPacket,
    _initialize_missile,
    _missile_datalink,
    _missile_guidance,
    _missile_intercept,
    _TrackPacket,
    agm6_actuator_step,
    agm6_initial_quaternion,
    agm6_propulsion_step,
    load_agm6_source_definition,
    run_agm6_source_compatibility,
)


def _write_agm6_case(root: Path) -> Path:
    input_path = root / "input.asc"
    input_path.write_text(
        """TITLE synthetic AGM6
MONTE 1 12345
OPTIONS y_events y_plot y_traj
MODULES
 environment def,init,exec
 kinematics def,init,exec
 aerodynamics def,init,exec
 propulsion def,init,exec
 forces def,exec
 ins def,init,exec
 datalink def,exec
 sensor def,exec
 guidance def,exec
 control def,exec
 actuator def,exec
 euler def,exec
 newton def,init,exec
 intercept def,exec
END
TIMING
 plot_step 0.01
 traj_step 0.02
 int_step 0.001
END
VEHICLES 3
 MISSILE6 SyntheticAGM6
  tgt_num 1
  sbel1 0
  sbel2 0
  sbel3 -100
  dvbe 293
  psiblx 0
  thtblx 0
  phiblx 0
  alpha0x 0
  beta0x 0
  mair 212
  WEATHER_DECK weather.asc
  RAYL dvae 5
  twind 0.1
  turb_length 100
  turb_sigma 0.2
  AERO_DECK aero.asc
  alplimx 20
  ai11 42.5
  ai33 2632
  vmass0 1360
  fmass0 250
  mprop 0
  aexit 0.02
  spi 210
  thrsl 10000
  throtl 1
  mact 2
  dlimx 20
  ddlimx 600
  wnact 62.8
  zetact 0.7
  maut 3
  alimit 3
  dqlimx 25
  drlimx 25
  dplimx 25
  phicomx 0
  zrcl 0.9
  wrcl 5
  zetlagr 0.9
  wacl 2
  zacl 0.7
  pacl 10
  mins 1
  mseek 0
  skr_dyn 1
  racq 1000
  fovyaw 0.2
  fovpitch 0.2
  dtimac 0.005
  dblind 1
  gk 10
  wnk 30
  zetak 0.9
  trate 2
  trtht 1.2
  trthtd 20
  trphid 20
  IF time > 0.02
   mnav 3
   mprop 1
   mseek 2
   mguid 30
   grav_bias 1.5
   gnav 3
  ENDIF
  IF mseek = 4
   mguid 6
   gnav 3
  ENDIF
 END
 TARGET3 SyntheticGroundTarget
  sael1 200
  sael2 20
  sael3 -100
  dvae 5
  psivlx -90
  acc_latx 0.01
 END
 AIRCRAFT3 SyntheticTrackingAircraft
  sael1 0
  sael2 0
  sael3 -100
  psivlx 0
  thtvlx 0
  dvae 293
  acft_option 0
  gturn 1.5
  tphi 0.5
  tanx 0.5
  clalpha 0.0523
  wingloading 3247
  philimx 60
  alplimx 12
  track_step 0.01
  dat_sigma 0
  azat_sigma 0
  elat_sigma 0
  vel_sigma 0
 END
ENDTIME 0.08
STOP
""",
        encoding="utf-8",
    )
    one_dimensional = {
        "ca0_vs_mach": (0.2, 0.2),
        "caa_vs_mach": (0.0, 0.0),
        "cad_vs_mach": (0.0001, 0.0001),
        "cndq_vs_mach": (0.02, 0.02),
        "cllap_vs_mach": (0.0, 0.0),
        "cllp_vs_mach": (-0.1, -0.1),
        "clldp_vs_mach": (0.1, 0.1),
        "clmq_vs_mach": (-5.0, -5.0),
        "clmdq_vs_mach": (-0.05, -0.05),
    }
    two_dimensional = {
        "cyp_vs_mach_alpha": (0.0, 0.0, 0.0, 0.0),
        "cn0_vs_mach_alpha": (0.0, 0.5, 0.0, 0.5),
        "cnp_vs_mach_alpha": (0.0, 0.0, 0.0, 0.0),
        "clm0_vs_mach_alpha": (0.0, -0.2, 0.0, -0.2),
        "clmp_vs_mach_alpha": (0.0, 0.0, 0.0, 0.0),
        "clnp_vs_mach_alpha": (0.0, 0.0, 0.0, 0.0),
    }
    aero_lines = ["TITLE synthetic AGM6 aero"]
    for name, values in one_dimensional.items():
        aero_lines.extend((f"1DIM {name}", "NX1 2", f"0 {values[0]}", f"5 {values[1]}"))
    ####
    for name, values in two_dimensional.items():
        aero_lines.extend(
            (
                f"2DIM {name}",
                "NX1 2 NX2 2",
                f"0 0 {values[0]} {values[1]}",
                f"5 20 {values[2]} {values[3]}",
            )
        )
    ####
    (root / "aero.asc").write_text("\n".join(aero_lines) + "\n", encoding="utf-8")
    (root / "weather.asc").write_text(
        """TITLE synthetic weather
1DIM density
NX1 2
0 1.225
10000 0.4135
1DIM pressure
NX1 2
0 101325
10000 26436
1DIM temperature
NX1 2
0 15
10000 -50
1DIM speed
NX1 2
0 8
10000 12
1DIM direction
NX1 2
0 90
10000 90
""",
        encoding="utf-8",
    )
    return input_path


####


def test_agm6_source_lowering_preserves_three_actor_bus_and_t4_effector_contract(tmp_path: Path) -> None:
    definition = load_agm6_source_definition(_write_agm6_case(tmp_path))
    assert definition.vehicle_order == ("MISSILE6", "TARGET3", "AIRCRAFT3")
    assert definition.module_order[5:9] == ("ins", "datalink", "sensor", "guidance")
    assert definition.actuator.mode == 2
    assert definition.environment.atmosphere_mode == 2
    assert definition.environment.turbulence_mode == 1
    assert definition.environment.wind_mode == 2
    assert definition.ins_mode_requested == 1
    assert definition.taoryx_tier == "rigid_body_6dof_surface_allocated"
    assert tuple(event.condition.variable for event in definition.missile_events) == ("time", "mseek")


####


def test_agm6_source_lowering_defaults_ground_target_flight_path_to_zero(tmp_path: Path) -> None:
    definition = load_agm6_source_definition(_write_agm6_case(tmp_path))
    assert definition.target.flight_path_deg == 0.0


####


def test_agm6_source_intercept_sphere_is_the_source_literal_not_critmax(tmp_path: Path) -> None:
    input_path = _write_agm6_case(tmp_path)
    text = input_path.read_text(encoding="utf-8")
    input_path.write_text(text.replace("  mseek 0", "  critmax 7\n  mseek 0"), encoding="utf-8")
    definition = load_agm6_source_definition(input_path)
    assert definition.target_sphere_radius_m == 100.0


####


def test_agm6_initial_quaternion_is_unit_norm(tmp_path: Path) -> None:
    definition = load_agm6_source_definition(_write_agm6_case(tmp_path))
    quaternion = np.asarray(agm6_initial_quaternion(definition.initial_state))
    assert np.linalg.norm(quaternion) == pytest.approx(1.0)


####


def test_agm6_reuses_source_identical_four_fin_effector_primitive() -> None:
    config = Agm6ActuatorConfig(
        mode=2,
        position_limit_deg=20.0,
        rate_limit_deg_s=600.0,
        natural_frequency_rad_s=62.8,
        damping_ratio=0.7,
    )
    command = Agm6ControlCommand(roll_deg=1.0, pitch_deg=2.0, yaw_deg=-3.0)
    first = agm6_actuator_step(config, Agm6FinActuatorState(), command, 0.001)
    second = agm6_actuator_step(config, first.state, command, 0.001)
    assert first.requested_fins.vector() == pytest.approx((4.0, -2.0, 6.0, 0.0))
    assert first.achieved_fins.vector() == pytest.approx((0.0, 0.0, 0.0, 0.0))
    assert any(abs(value) > 0.0 for value in second.achieved_fins.vector())


####


def test_agm6_propulsion_advances_continuous_fuel_and_pressure_corrected_thrust(tmp_path: Path) -> None:
    definition = load_agm6_source_definition(_write_agm6_case(tmp_path))
    initial = Agm6PropulsionStep(
        mode=1,
        thrust_n=0.0,
        mass_kg=definition.airframe.launch_mass_kg,
        fuel_expended_kg=0.0,
        fuel_remaining_kg=definition.propulsion.initial_fuel_mass_kg,
        fuel_flow_kg_s=0.0,
    )
    step = agm6_propulsion_step(
        definition,
        initial,
        pressure_pa=90_000.0,
        dt_s=0.01,
        mode=1,
    )
    assert step.thrust_n > definition.propulsion.sea_level_thrust_n
    assert step.fuel_expended_kg > 0.0
    assert step.mass_kg < definition.airframe.launch_mass_kg


####


def test_agm6_datalink_uses_source_position_norm_change_and_preserves_update_flag(tmp_path: Path) -> None:
    definition = load_agm6_source_definition(_write_agm6_case(tmp_path))
    runtime = _initialize_missile(definition, definition.initial_state, np.random.default_rng(3))
    target = _ActorPacket(
        position_ned_m=np.asarray((100.0, 0.0, -100.0), dtype=np.float64),
        velocity_ned_mps=np.zeros(3, dtype=np.float64),
        heading_deg=0.0,
        flight_path_deg=0.0,
        alive=True,
    )
    aircraft = _AircraftPacket(
        position_ned_m=np.zeros(3, dtype=np.float64),
        velocity_ned_mps=np.zeros(3, dtype=np.float64),
        heading_deg=0.0,
        flight_path_deg=0.0,
        alive=True,
        track=_TrackPacket(
            position_ned_m=np.asarray((3.0, 4.0, 0.0), dtype=np.float64),
            velocity_ned_mps=np.asarray((1.0, 0.0, 0.0), dtype=np.float64),
            update_sequence=1,
            time_s=1.0,
        ),
    )

    _missile_datalink(runtime, aircraft)
    assert runtime.datalink_update_mode == 3
    assert runtime.datalink_target_position_norm_m == pytest.approx(5.0)

    runtime.guidance_mode = 30
    runtime.launch_time_s = 1.5
    _missile_guidance(runtime, definition.guidance, target, aircraft)
    assert runtime.target_update_epoch_s == pytest.approx(1.5)
    assert runtime.datalink_update_mode == 3

    aircraft.track = _TrackPacket(
        position_ned_m=np.asarray((0.0, 5.0, 0.0), dtype=np.float64),
        velocity_ned_mps=np.asarray((2.0, 0.0, 0.0), dtype=np.float64),
        update_sequence=2,
        time_s=2.0,
    )
    _missile_datalink(runtime, aircraft)
    assert runtime.datalink_update_mode == 0
    assert runtime.datalink_track_sequence == 2

    aircraft.track = _TrackPacket(
        position_ned_m=np.asarray((0.0, 6.0, 0.0), dtype=np.float64),
        velocity_ned_mps=np.asarray((3.0, 0.0, 0.0), dtype=np.float64),
        update_sequence=3,
        time_s=3.0,
    )
    _missile_datalink(runtime, aircraft)
    assert runtime.datalink_update_mode == 3
    assert runtime.datalink_target_position_norm_m == pytest.approx(6.0)


####


def test_agm6_target_plane_intercept_uses_source_previous_sample_interpolation(tmp_path: Path) -> None:
    definition = load_agm6_source_definition(_write_agm6_case(tmp_path))
    runtime = _initialize_missile(definition, definition.initial_state, np.random.default_rng(4))
    runtime.guidance_mode = 40
    runtime.position_ned_m = np.asarray((2.0, 4.0, 1.0), dtype=np.float64)
    runtime.previous_target_plane_position_m = np.asarray((0.0, 2.0, -1.0), dtype=np.float64)
    runtime.previous_time_s = 1.0
    target = _ActorPacket(
        position_ned_m=np.zeros(3, dtype=np.float64),
        velocity_ned_mps=np.zeros(3, dtype=np.float64),
        heading_deg=0.0,
        flight_path_deg=0.0,
        alive=True,
    )

    intercept = _missile_intercept(runtime, definition, target, sim_time=1.1, dt_s=0.1)
    assert intercept is not None
    assert intercept.time_s == pytest.approx(1.05)
    assert intercept.miss_vector_target_plane_m == pytest.approx((1.0, 3.0, 0.0))
    assert intercept.miss_distance_m == pytest.approx(math.sqrt(10.0))


####


def test_agm6_run_executes_events_datalink_sensor_and_physical_fins(tmp_path: Path) -> None:
    definition = load_agm6_source_definition(_write_agm6_case(tmp_path))
    result = run_agm6_source_compatibility(definition, random_seed=12345)
    assert result.terminated_reason in {"end_time", "intercept", "ground_impact"}
    assert len(result.samples) >= 5
    assert len(result.target_samples) == len(result.samples)
    assert len(result.aircraft_samples) == len(result.samples)
    assert len(result.track_samples) >= 2
    assert tuple(trace.watch_variable for trace in result.event_trace) == ("time", "mseek")
    assert any(sample.propulsion_mode == 1 for sample in result.samples)
    assert any(sample.guidance_mode == 30 for sample in result.samples)
    assert any(sample.guidance_mode == 6 for sample in result.samples)
    assert any(sample.sensor_mode == 4 for sample in result.samples)
    assert any(any(abs(value) > 0.0 for value in sample.requested_fins_deg) for sample in result.samples)
    assert any(any(abs(value) > 0.0 for value in sample.achieved_fins_deg) for sample in result.samples)
    assert all(sample.ins_mode_requested == 1 for sample in result.samples)
    assert all(sample.ins_model_effective == "truth_aligned" for sample in result.samples)


####


def test_agm6_seeded_weather_and_tracking_are_deterministic(tmp_path: Path) -> None:
    definition = load_agm6_source_definition(_write_agm6_case(tmp_path))
    first = run_agm6_source_compatibility(definition, random_seed=77)
    second = run_agm6_source_compatibility(definition, random_seed=77)
    assert first == second
    assert first.samples[-1].wind_ned_mps == second.samples[-1].wind_ned_mps


####


def test_agm6_result_states_claim_boundary_without_overclaiming_real_ins_or_full_iir(tmp_path: Path) -> None:
    result = run_agm6_source_compatibility(
        load_agm6_source_definition(_write_agm6_case(tmp_path)),
        random_seed=11,
    )
    assert "truth-aligned" in result.claim_boundary
    assert "focal-plane" in result.claim_boundary
    assert "C-rand" in result.claim_boundary


####


def test_agm6_rejects_non_source_actor_order(tmp_path: Path) -> None:
    input_path = _write_agm6_case(tmp_path)
    text = input_path.read_text(encoding="utf-8")
    missile_start = text.index(" MISSILE6")
    target_start = text.index(" TARGET3")
    aircraft_start = text.index(" AIRCRAFT3")
    end_time_start = text.index("ENDTIME")
    missile = text[missile_start:target_start]
    target = text[target_start:aircraft_start]
    aircraft = text[aircraft_start:end_time_start]
    input_path.write_text(
        text[:missile_start] + aircraft + missile + target + text[end_time_start:],
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="source-compatible bus lag"):
        load_agm6_source_definition(input_path)
    ####


####


def test_agm6_all_sample_channels_remain_finite(tmp_path: Path) -> None:
    result = run_agm6_source_compatibility(
        load_agm6_source_definition(_write_agm6_case(tmp_path)),
        random_seed=5,
    )
    for sample in result.samples:
        values = (
            *sample.position_ned_m,
            *sample.velocity_ned_mps,
            *sample.quaternion_wxyz,
            *sample.body_rates_rad_s,
            *sample.force_body_n,
            *sample.moment_body_nm,
            sample.mass_kg,
            sample.target_range_m,
        )
        assert all(math.isfinite(value) for value in values)
    ####


####
