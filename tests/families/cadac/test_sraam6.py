from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from taoryx.families.cadac.sraam6 import (
    Sraam6ActuatorConfig,
    Sraam6ControlCommand,
    Sraam6FinActuatorState,
    load_sraam6_source_definition,
    run_sraam6_source_compatibility,
    sraam6_actuator_step,
    sraam6_initial_quaternion,
    sraam6_mix_fin_commands,
    sraam6_propulsion_step,
    sraam6_unmix_fin_positions,
)


def _write_sraam6_case(root: Path) -> Path:
    input_path = root / "input.asc"
    input_path.write_text(
        """TITLE synthetic SRAAM6
OPTIONS y_events y_plot
MODULES
 environment def,exec
 kinematics def,init,exec
 aerodynamics def,init,exec
 propulsion def,exec
 seeker def,exec
 guidance def,exec
 control def,exec
 actuator def,exec
 tvc def,exec
 forces def,exec
 euler def,init,exec
 newton def,init,exec
 intercept def,exec
END
TIMING
 plot_step 0.02
 int_step 0.01
END
VEHICLES 2
 MISSILE6 SyntheticSRAAM
  tgt_num 1
  sbel1 0
  sbel2 0
  sbel3 -1000
  dvbe 250
  psiblx 0
  thtblx 0
  phiblx 0
  alpha0x 0
  beta0x 0
  AERO_DECK aero.asc
  alplimx 46
  mprop 1
  aexit 0.0125
  PROP_DECK prop.asc
  mact 2
  dlimx 28
  ddlimx 600
  wnact 100
  zetact 0.7
  maut 2
  alimit 50
  dqlimx 28
  drlimx 28
  dplimx 28
  phicomx 0
  wrcl 20
  zrcl 0.9
  zetlagr 0.6
  mseek 2
  ms1dyn 1
  racq 99999
  dblind 3
  dtimac 0.02
  gk 10
  zetak 0.9
  wnk 60
  fovyaw 0.0314
  fovpitch 0.0314
  mnav 3
  gnav 3.75
  mtvc 0
  IF time > 0.05
   maut 3
   mguid 3
  ENDIF
 END
 TARGET3 SyntheticTarget
  msl_num 1
  tgt_option 1
  gturn 3
  guid_gain 3
  sael1 1000
  sael2 0
  sael3 -1000
  psialx 180
  thtalx 0
  dvae 250
 END
ENDTIME 0.2
STOP
""",
        encoding="utf-8",
    )
    one_dimensional = {
        "ca0_vs_mach": (0.2, 0.2),
        "caa_vs_mach": (0.0, 0.0),
        "cad_vs_mach": (0.0, 0.0),
        "caoff_vs_mach": (0.0, 0.0),
        "clmq_vs_mach": (-5.0, -5.0),
    }
    two_dimensional = {
        "cn0_vs_mach_alpha": (0.0, 1.0, 0.0, 1.0),
        "cnp_vs_mach_alpha": (0.0, 0.0, 0.0, 0.0),
        "clm0_vs_mach_alpha": (0.0, -0.2, 0.0, -0.2),
        "clmp_vs_mach_alpha": (0.0, 0.0, 0.0, 0.0),
        "cyp_vs_mach_alpha": (0.0, 0.0, 0.0, 0.0),
        "cndq_vs_mach_alpha": (0.1, 0.1, 0.1, 0.1),
        "clmdq_vs_mach_alpha": (-0.1, -0.1, -0.1, -0.1),
        "clnp_vs_mach_alpha": (0.0, 0.0, 0.0, 0.0),
        "cllap_vs_mach_alpha": (0.0, 0.0, 0.0, 0.0),
        "cllp_vs_mach_alpha": (-0.1, -0.1, -0.1, -0.1),
        "clldp_vs_mach_alpha": (0.1, 0.1, 0.1, 0.1),
    }
    aero_lines = ["TITLE synthetic SRAAM6 aero"]
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
    (root / "prop.asc").write_text(
        """TITLE synthetic SRAAM6 prop
1DIM mass_vs_time
NX1 3
0 92
0.1 90
3 56
1DIM thrust_vs_time
NX1 3
0 0
0.1 30000
3 0
1DIM moipitch_vs_time
NX1 3
0 60
0.1 59
3 44
1DIM moiroll_vs_time
NX1 3
0 0.31
0.1 0.30
3 0.20
1DIM cg_vs_time
NX1 3
0 1.536
0.1 1.52
3 1.22
""",
        encoding="utf-8",
    )
    return input_path


####


def test_sraam6_source_lowering_recovers_physical_fin_and_target_contract(tmp_path: Path) -> None:
    definition = load_sraam6_source_definition(_write_sraam6_case(tmp_path))
    assert definition.vehicle_order == ("MISSILE6", "TARGET3")
    assert definition.actuator.mode == 2
    assert definition.seeker.dynamic_mode == 1
    assert definition.target.aircraft_option == 1
    assert definition.taoryx_tier == "rigid_body_6dof_surface_allocated"
    assert tuple(event.condition.variable for event in definition.missile_events) == ("time",)


####


def test_sraam6_initial_quaternion_is_unit_norm(tmp_path: Path) -> None:
    definition = load_sraam6_source_definition(_write_sraam6_case(tmp_path))
    quaternion = np.asarray(sraam6_initial_quaternion(definition.initial_state))
    assert np.linalg.norm(quaternion) == pytest.approx(1.0)


####


def test_sraam6_four_fin_mixer_round_trips_control_coordinates() -> None:
    command = Sraam6ControlCommand(roll_deg=1.0, pitch_deg=2.0, yaw_deg=-3.0)
    fins = sraam6_mix_fin_commands(command)
    assert fins.vector() == pytest.approx((4.0, -2.0, 6.0, 0.0))
    assert sraam6_unmix_fin_positions(fins).vector() == pytest.approx(command.vector())


####


def test_sraam6_second_order_fin_actuator_exposes_requested_and_achieved() -> None:
    config = Sraam6ActuatorConfig(
        mode=2,
        position_limit_deg=28.0,
        rate_limit_deg_s=600.0,
        natural_frequency_rad_s=100.0,
        damping_ratio=0.7,
    )
    command = Sraam6ControlCommand(pitch_deg=5.0)
    first = sraam6_actuator_step(config, Sraam6FinActuatorState(), command, 0.001)
    second = sraam6_actuator_step(config, first.state, command, 0.001)
    assert first.requested_fins.vector() == pytest.approx((5.0, 5.0, 5.0, 5.0))
    assert first.achieved_fins.vector() == pytest.approx((0.0, 0.0, 0.0, 0.0))
    assert all(value > 0.0 for value in second.achieved_fins.vector())
    assert second.achieved_control.pitch_deg > 0.0


####


def test_sraam6_second_order_actuator_preserves_source_preintegration_travel_limit_order() -> None:
    config = Sraam6ActuatorConfig(
        mode=2,
        position_limit_deg=28.0,
        rate_limit_deg_s=600.0,
        natural_frequency_rad_s=100.0,
        damping_ratio=0.7,
    )
    state = Sraam6FinActuatorState(
        position_derivative_deg_s=(600.0, 0.0, 0.0, 0.0),
        position_deg=(27.9, 0.0, 0.0, 0.0),
        rate_derivative_deg_s2=(0.0, 0.0, 0.0, 0.0),
        rate_deg_s=(600.0, 0.0, 0.0, 0.0),
    )
    step = sraam6_actuator_step(
        config,
        state,
        Sraam6ControlCommand(),
        0.001,
    )
    assert step.position_limited == (False, False, False, False)
    assert step.achieved_fins.fin1_deg == pytest.approx(28.5)
    assert step.achieved_fins.fin1_deg > config.position_limit_deg


####


def test_sraam6_mode_zero_actuator_does_not_reseed_dynamic_states() -> None:
    config = Sraam6ActuatorConfig(
        mode=0,
        position_limit_deg=28.0,
        rate_limit_deg_s=600.0,
        natural_frequency_rad_s=100.0,
        damping_ratio=0.7,
    )
    state = Sraam6FinActuatorState(
        position_derivative_deg_s=(1.0, 2.0, 3.0, 4.0),
        position_deg=(0.5, 1.0, 1.5, 2.0),
        rate_derivative_deg_s2=(5.0, 6.0, 7.0, 8.0),
        rate_deg_s=(9.0, 10.0, 11.0, 12.0),
    )
    step = sraam6_actuator_step(
        config,
        state,
        Sraam6ControlCommand(pitch_deg=40.0),
        0.001,
    )
    assert step.achieved_fins.vector() == pytest.approx((28.0, 28.0, 28.0, 28.0))
    assert step.state == state


####


def test_sraam6_propulsion_uses_time_deck_mass_and_pressure_correction(tmp_path: Path) -> None:
    definition = load_sraam6_source_definition(_write_sraam6_case(tmp_path))
    step = sraam6_propulsion_step(definition, time_s=0.1, pressure_pa=90_000.0, mode=1)
    assert step.mass_kg == pytest.approx(90.0)
    assert step.thrust_n == pytest.approx(30_000.0 + (101_325.0 - 90_000.0) * 0.0125)
    assert step.roll_inertia_kg_m2 == pytest.approx(0.30)


####


def test_sraam6_source_run_switches_rate_to_acceleration_control_and_is_finite(tmp_path: Path) -> None:
    definition = load_sraam6_source_definition(_write_sraam6_case(tmp_path))
    result = run_sraam6_source_compatibility(definition, end_time_s=0.2, sample_step_s=0.02)
    assert result.executed_steps > 0
    assert result.event_trace[0].updated_values == (("maut", 3), ("mguid", 3))
    assert {sample.autopilot_mode for sample in result.samples} == {2, 3}
    assert 3 in {sample.seeker_mode for sample in result.samples}
    assert 4 in {sample.seeker_mode for sample in result.samples}
    assert 6 in {sample.guidance_mode for sample in result.samples}
    assert any(abs(sample.seeker_los_rate_pitch_rad_s) > 0.0 or abs(sample.seeker_los_rate_yaw_rad_s) > 0.0 for sample in result.samples)
    assert all(math.isfinite(value) for sample in result.samples for value in sample.position_ned_m)
    assert all(len(sample.achieved_fins_deg) == 4 for sample in result.samples)
    assert result.target_samples[-1].position_ned_m != result.target_samples[0].position_ned_m


####


def test_sraam6_target_events_use_source_time_and_mutate_supported_controls(tmp_path: Path) -> None:
    input_path = _write_sraam6_case(tmp_path)
    source = input_path.read_text(encoding="utf-8")
    input_path.write_text(
        source.replace(
            "  dvae 250\n END\nENDTIME",
            "  dvae 250\n  IF time > 0.03\n   gturn -2\n  ENDIF\n END\nENDTIME",
        ),
        encoding="utf-8",
    )
    definition = load_sraam6_source_definition(input_path)
    result = run_sraam6_source_compatibility(definition, end_time_s=0.08, sample_step_s=0.01)
    target_events = tuple(trace for trace in result.event_trace if trace.actor == "TARGET3")
    assert len(target_events) == 1
    assert target_events[0].updated_values == (("gturn", -2.0),)


####


def test_sraam6_unsupported_event_mutation_fails_closed(tmp_path: Path) -> None:
    input_path = _write_sraam6_case(tmp_path)
    source = input_path.read_text(encoding="utf-8")
    input_path.write_text(
        source.replace(
            "  ENDIF\n END\n TARGET3",
            "  ENDIF\n  IF time > 0.06\n   thrust 1\n  ENDIF\n END\n TARGET3",
        ),
        encoding="utf-8",
    )
    definition = load_sraam6_source_definition(input_path)
    with pytest.raises(ValueError, match="readable but not mutable"):
        run_sraam6_source_compatibility(definition, end_time_s=0.1, sample_step_s=0.01)
    ####


####
