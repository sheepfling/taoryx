from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from taoryx.families.cadac.ads6_sam import (
    Ads6SamControlCommand,
    Ads6SamDirectCommand,
    Ads6SamFinActuatorConfig,
    Ads6SamFinActuatorState,
    Ads6SamRcsConfig,
    Ads6SamRcsRuntimeInput,
    Ads6SamRcsState,
    Ads6SamTvcConfig,
    Ads6SamTvcState,
    ads6_sam_fin_actuator_step,
    ads6_sam_initial_quaternion,
    ads6_sam_mix_fin_commands,
    ads6_sam_rcs_step,
    ads6_sam_tvc_step,
    ads6_sam_unmix_fin_positions,
    load_ads6_sam_source_definition,
    run_ads6_sam_physical_plant,
)


def _write_ads6_sam_case(root: Path) -> Path:
    input_path = root / "input.asc"
    input_path.write_text(
        """TITLE synthetic ADS6 SAM
OPTIONS y_plot
MODULES
 environment def,exec
 kinematics def,init,exec
 propulsion def,exec
 aerodynamics def,init,exec
 ins def,init,exec
 sensor def,exec
 guidance def,exec
 control def,exec
 actuator def,exec
 tvc def,exec
 rcs def,exec
 forces def,exec
 euler def,exec
 newton def,init,exec
 intercept def,exec
END
TIMING
 plot_step 0.02
 int_step 0.001
END
VEHICLES 1
 MISSILE6 SyntheticSAM
  sbel1 0
  sbel2 0
  sbel3 -1000
  dvbe 300
  psiblx 0
  thtblx 0
  phiblx 0
  alpha0x 0
  beta0x 0
  AERO_DECK aero.asc
  xcgref 2.5
  alplimx 40
  alimitx 50
  PROP_DECK prop.asc
  aexit 0.0314
  mact 2
  dlimx 28
  ddlimx 600
  wnact 100
  zetact 0.7
  mtvc 2
  tvclimx 8
  dtvclimx 200
  wntvc 100
  zettvc 0.7
  pdynmc_gtvc36 100000
  gtvc0 0.5
  parm 5
  mrcs_moment 21
  mrcs_force 2
  roll_mom_max 100
  pitch_mom_max 300
  yaw_mom_max 300
  rcs_zeta 0.7
  rcs_freq 1
  dead_zone 0.4
  hysteresis 0.1
  rcs_tau 1
  rcs_arm 4
  rcs_isp 200
  rate_gain_rcs 100
  acc_gain 1000
  rcs_thrust 1000
  phibdcomx 0
  thtbdcomx 0
  psibdcomx 0
 END
ENDTIME 0.05
STOP
""",
        encoding="utf-8",
    )
    three_dimensional = (
        "ca0_vs_mach,betax,alphax",
        "cy0_vs_mach,betax,alphax",
        "cydr_vs_mach,betax,alphax",
        "cn0_vs_mach,betax,alphax",
        "cndq_vs_mach,betax,alphax",
        "cll0_vs_mach,betax,alphax",
        "clldp_vs_mach,betax,alphax",
        "clm0_vs_mach,betax,alphax",
        "clmdq_vs_mach,betax,alphax",
        "cln0_vs_mach,betax,alphax",
        "clndr_vs_mach,betax,alphax",
    )
    one_dimensional = (
        "cad_vs_mach",
        "cab_vs_mach",
        "cllp_vs_mach",
        "clmq_vs_mach",
        "clnr_vs_mach",
    )
    aero_lines = ["TITLE synthetic ADS6 SAM aero"]
    for name in three_dimensional:
        if name.startswith("ca0"):
            values = (0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2)
        elif name.startswith("cn0"):
            values = (0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0)
        elif name.startswith("clm0"):
            values = (0.0, -0.2, 0.0, -0.2, 0.0, -0.2, 0.0, -0.2)
        elif name.startswith(("cndq", "cydr", "clldp")):
            values = (0.1,) * 8
        elif name.startswith(("clmdq", "clndr")):
            values = (-0.1,) * 8
        else:
            values = (0.0,) * 8
        ####
        aero_lines.extend(
            (
                f"3DIM {name}",
                "NX1 2 NX2 2 NX3 2",
                f"0 -10 0 {values[0]} {values[1]} {values[2]} {values[3]}",
                f"5 10 40 {values[4]} {values[5]} {values[6]} {values[7]}",
            )
        )
    ####
    for name in one_dimensional:
        value = -0.1 if name in {"cllp_vs_mach", "clmq_vs_mach", "clnr_vs_mach"} else 0.01
        aero_lines.extend((f"1DIM {name}", "NX1 2", f"0 {value}", f"5 {value}"))
    ####
    (root / "aero.asc").write_text("\n".join(aero_lines) + "\n", encoding="utf-8")
    (root / "prop.asc").write_text(
        """TITLE synthetic ADS6 SAM prop
1DIM thrust_vs_time
NX1 3
0 20000
0.1 20000
60 0
1DIM mass_vs_time
NX1 3
0 300
0.1 299
60 200
1DIM cg_vs_time
NX1 3
0 2.9
0.1 2.89
60 2.5
1DIM moipitch_vs_time
NX1 3
0 440
0.1 439
60 300
1DIM moiroll_vs_time
NX1 3
0 2.9
0.1 2.89
60 2
""",
        encoding="utf-8",
    )
    return input_path


####


def test_ads6_sam_source_lowering_recovers_all_realization_subsystems(tmp_path: Path) -> None:
    definition = load_ads6_sam_source_definition(_write_ads6_sam_case(tmp_path))
    assert definition.fin_actuator.mode == 2
    assert definition.tvc.mode == 2
    assert definition.rcs.moment_mode == 21
    assert definition.rcs.force_mode == 2
    assert definition.aerodynamic_deck.table("ca0_vs_mach,betax,alphax").dimension == 3
    assert len(definition.source_artifacts) == 3


####


def test_ads6_sam_initial_quaternion_is_unit_norm(tmp_path: Path) -> None:
    definition = load_ads6_sam_source_definition(_write_ads6_sam_case(tmp_path))
    quaternion = np.asarray(ads6_sam_initial_quaternion(definition.initial_state))
    assert np.linalg.norm(quaternion) == pytest.approx(1.0)


####


def test_ads6_cross_fin_mixer_round_trips_control_coordinates() -> None:
    command = Ads6SamControlCommand(roll_deg=1.0, pitch_deg=2.0, yaw_deg=-3.0)
    fins = ads6_sam_mix_fin_commands(command)
    assert fins.vector() == pytest.approx((2.0, 1.0, -4.0, -3.0))
    assert ads6_sam_unmix_fin_positions(fins).vector() == pytest.approx(command.vector())


####


def test_ads6_second_order_fin_actuator_exposes_requested_and_achieved() -> None:
    config = Ads6SamFinActuatorConfig(
        mode=2,
        position_limit_deg=28.0,
        rate_limit_deg_s=600.0,
        natural_frequency_rad_s=100.0,
        damping_ratio=0.7,
    )
    command = Ads6SamControlCommand(pitch_deg=5.0)
    first = ads6_sam_fin_actuator_step(config, Ads6SamFinActuatorState(), command, 0.001)
    second = ads6_sam_fin_actuator_step(config, first.state, command, 0.001)
    assert first.requested_fins.vector() == pytest.approx((0.0, 5.0, 0.0, -5.0))
    assert first.achieved_fins.vector() == pytest.approx((0.0, 0.0, 0.0, 0.0))
    assert second.achieved_control.pitch_deg > 0.0


####


def test_ads6_tvc_variable_gain_and_achieved_wrench() -> None:
    config = Ads6SamTvcConfig(
        mode=3,
        position_limit_deg=8.0,
        rate_limit_deg_s=200.0,
        natural_frequency_rad_s=100.0,
        damping_ratio=0.7,
        pressure_for_36_percent_gain_pa=100_000.0,
        initial_gain=0.5,
        propulsion_arm_from_nose_m=5.0,
    )
    command = Ads6SamControlCommand(pitch_deg=4.0, yaw_deg=2.0)
    first = ads6_sam_tvc_step(
        config,
        Ads6SamTvcState(),
        command,
        dynamic_pressure_pa=100_000.0,
        thrust_n=20_000.0,
        center_of_gravity_m=3.0,
        dt_s=0.001,
    )
    second = ads6_sam_tvc_step(
        config,
        first.state,
        command,
        dynamic_pressure_pa=100_000.0,
        thrust_n=20_000.0,
        center_of_gravity_m=3.0,
        dt_s=0.001,
    )
    assert first.effective_gain == pytest.approx(0.5 * np.exp(-1.0))
    assert second.achieved_pitch_yaw_deg[0] > 0.0
    assert second.moment_body_nm[1] < 0.0


####


def test_ads6_rcs_side_force_overwrites_transverse_moments_with_parasitic_arm() -> None:
    config = Ads6SamRcsConfig(
        moment_mode=21,
        force_mode=2,
        dead_zone=0.4,
        hysteresis=0.1,
        time_slope_s=1.0,
        roll_moment_limit_nm=100.0,
        pitch_moment_limit_nm=300.0,
        yaw_moment_limit_nm=300.0,
        proportional_damping=0.7,
        proportional_frequency_rad_s=1.0,
        location_from_nose_m=4.0,
        specific_impulse_s=200.0,
        acceleration_gain_n_per_mps2=1000.0,
        side_force_limit_n=1000.0,
    )
    runtime = Ads6SamRcsRuntimeInput(
        dt_s=0.01,
        body_rates_rad_s=(0.0, 0.0, 0.0),
        body_angles_deg=(0.0, 0.0, 0.0),
        incidence_deg=(0.0, 0.0),
        acceleration_commands_g=(1.0, 1.0),
        specific_force_body_mps2=(0.0, 0.0, 0.0),
        inertia_diagonal_kgm2=(3.0, 440.0, 440.0),
        center_of_gravity_m=3.0,
    )
    first = ads6_sam_rcs_step(config, runtime, Ads6SamRcsState())
    second = ads6_sam_rcs_step(config, runtime, first.state)
    assert first.force_body_n == pytest.approx((0.0, 0.0, 0.0))
    assert second.force_body_n == pytest.approx((0.0, 1000.0, -1000.0))
    assert second.moment_body_nm[1:] == pytest.approx((-1000.0, -1000.0))
    assert second.fuel_mass_expended_kg > 0.0


####


@pytest.mark.parametrize(
    ("phase", "expected_fidelity"),
    (
        ("fin_control", "rigid_body_6dof_surface_allocated"),
        ("tvc_control", "rigid_body_6dof_surface_allocated"),
        ("aggregate_rcs", "rigid_body_6dof_direct_wrench"),
    ),
)
def test_ads6_sam_runner_reports_phase_specific_fidelity(
    tmp_path: Path,
    phase: str,
    expected_fidelity: str,
) -> None:
    definition = load_ads6_sam_source_definition(_write_ads6_sam_case(tmp_path))
    command = Ads6SamDirectCommand(
        phase=phase,
        control=Ads6SamControlCommand(pitch_deg=1.0),
        tvc_mode=2 if phase == "tvc_control" else None,
        rcs_moment_mode=21 if phase == "aggregate_rcs" else None,
        rcs_force_mode=0 if phase == "aggregate_rcs" else None,
        pitch_attitude_command_deg=1.0,
    )
    result = run_ads6_sam_physical_plant(
        definition,
        command,
        end_time_s=0.01,
        sample_step_s=0.005,
    )
    assert result.samples[-1].source_phase == phase
    assert result.samples[-1].fidelity == expected_fidelity
    assert result.terminated_reason == "end_time"


####
