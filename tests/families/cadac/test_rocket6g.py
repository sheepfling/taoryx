from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from taoryx.families.cadac.rocket6g import (
    Rocket6gDirectCommand,
    Rocket6gTvcConfig,
    Rocket6gTvcState,
    load_rocket6g_source_definition,
    rocket6g_tvc_step,
    run_rocket6g_phase_aware_plant,
)


def _write_rocket_case(root: Path) -> Path:
    input_path = root / "input.asc"
    input_path.write_text(
        """TITLE synthetic ROCKET6G phase program
OPTIONS y_events y_plot
MODULES
 kinematics def,init,exec
 environment def,init,exec
 propulsion def,init,exec
 aerodynamics def,init,exec
 gps def,exec
 startrack def,exec
 ins def,init,exec
 guidance def,exec
 control def,exec
 rcs def,exec
 actuator def,exec
 tvc def,exec
 forces def,exec
 newton def,init,exec
 euler def,init,exec
 intercept def,exec
END
TIMING
 plot_step 0.1
 int_step 0.1
END
VEHICLES 1
 HYPER6 SyntheticSLV
  lonx -120
  latx 35
  alt 100
  dvbe 1
  phibdx 0
  thtbdx 90
  psibdx -83
  alpha0x 0
  beta0x 0
  ppx 0
  qqx 0
  rrx 0
  mair 0
  maero 13
  AERO_DECK aero.asc
  xcg_ref 8
  refa 1
  refd 1
  alplimx 20
  alimitx 5
  mprop 3
  vmass0 100
  fmass0 20
  xcg_0 4
  xcg_1 3
  moi_roll_0 100
  moi_roll_1 80
  moi_trans_0 200
  moi_trans_1 120
  spi 100
  fuel_flow_rate 10
  mtvc 0
  gtvc 1
  parm 8
  tvclimx 10
  dtvclimx 50
  zettvc 0.7
  wntvc 10
  mrcs_moment 21
  roll_mom_max 10
  pitch_mom_max 20
  yaw_mom_max 20
  dead_zone 0.4
  hysteresis 0.1
  rcs_tau 1
  thtbdcomx 80
  psibdcomx -83
  IF time > 0.5
   mtvc 2
   mrcs_moment 20
  ENDIF
  IF thrust = 0
  ENDIF
  IF event_time > 0.2
   maero 12
   xcg_ref 5
   mtvc 0
   mrcs_moment 22
   mprop 4
   vmass0 60
   fmass0 10
   fmasse 0
   xcg_0 3
   xcg_1 2.5
   moi_roll_0 60
   moi_roll_1 40
   moi_trans_0 100
   moi_trans_1 70
   spi 100
   fuel_flow_rate 10
  ENDIF
  IF event_time > 1.2
   maero 11
   xcg_ref 3
   roll_mom_max 5
   pitch_mom_max 5
   yaw_mom_max 5
   mprop 4
   vmass0 30
   fmass0 10
   fmasse 0
   xcg_0 2
   xcg_1 1.5
   moi_roll_0 30
   moi_roll_1 20
   moi_trans_0 50
   moi_trans_1 35
   spi 100
   fuel_flow_rate 10
  ENDIF
  IF beco_flag = 1
   mprop 0
  ENDIF
 END
ENDTIME 5
STOP
""",
        encoding="utf-8",
    )
    one_dimensional = (
        "ca0slv3_vs_mach",
        "caaslv3_vs_mach",
        "ca0bslv3_vs_mach",
        "clmqslv3_vs_mach",
        "ca0slv2_vs_mach",
        "caaslv2_vs_mach",
        "ca0bslv2_vs_mach",
        "clmqslv2_vs_mach",
        "clmqslv1_vs_mach",
    )
    two_dimensional = (
        "cn0slv3_vs_mach_alpha",
        "clm0slv3_vs_mach_alpha",
        "cn0slv2_vs_mach_alpha",
        "clm0slv2_vs_mach_alpha",
        "cn0slv1_vs_mach_alpha",
        "clm0slv1_vs_mach_alpha",
    )
    lines = ["TITLE synthetic ROCKET6G aero"]
    for name in one_dimensional:
        lines.extend((f"1DIM {name}", "NX1 2", "0 0", "20 0"))
    ####
    for name in two_dimensional:
        lines.extend((f"2DIM {name}", "NX1 2 NX2 2", "0 0 0 0", "20 20 0 0"))
    ####
    (root / "aero.asc").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return input_path


####


def test_rocket6g_source_lowering_recovers_three_stages_and_empty_burnout_event(tmp_path: Path) -> None:
    definition = load_rocket6g_source_definition(_write_rocket_case(tmp_path))
    assert definition.integration_step_s == pytest.approx(0.1)
    assert tuple(stage.stage_number for stage in definition.stages) == (1, 2, 3)
    assert tuple(stage.initial_mass_kg for stage in definition.stages) == pytest.approx((100.0, 60.0, 30.0))
    assert definition.events[1].condition.variable == "thrust"
    assert definition.events[1].assignments == ()
    assert definition.control_realization == "mixed_effector"


####


def test_rocket6g_second_order_tvc_preserves_requested_and_achieved_nozzle_state() -> None:
    config = Rocket6gTvcConfig(
        mode=2,
        command_gain=1.0,
        propulsion_arm_from_nose_m=8.0,
        position_limit_deg=10.0,
        rate_limit_deg_s=50.0,
        natural_frequency_rad_s=10.0,
        damping_ratio=0.7,
    )
    first = rocket6g_tvc_step(
        config,
        Rocket6gTvcState(),
        control_pitch_deg=5.0,
        control_yaw_deg=-2.0,
        thrust_n=1000.0,
        dynamic_pressure_pa=1000.0,
        center_of_gravity_m=3.0,
        dt_s=0.01,
    )
    second = rocket6g_tvc_step(
        config,
        first.state,
        control_pitch_deg=5.0,
        control_yaw_deg=-2.0,
        thrust_n=1000.0,
        dynamic_pressure_pa=1000.0,
        center_of_gravity_m=3.0,
        dt_s=0.01,
    )
    assert first.requested_nozzle_deg == pytest.approx((5.0, -2.0))
    assert first.achieved_nozzle_deg == pytest.approx((0.0, 0.0))
    assert second.achieved_nozzle_deg[0] > 0.0
    assert second.achieved_nozzle_deg[1] < 0.0
    assert second.force_body_n[2] < 0.0
    assert second.moment_body_nm[1] < 0.0
    assert second.force_body_n[1] < 0.0
    assert second.moment_body_nm[2] > 0.0


####


def test_rocket6g_phase_aware_run_transitions_rcs_tvc_rcs_and_stages(tmp_path: Path) -> None:
    definition = load_rocket6g_source_definition(_write_rocket_case(tmp_path))
    result = run_rocket6g_phase_aware_plant(
        definition,
        Rocket6gDirectCommand(
            tvc_pitch_command_deg=1.0,
            thrust_vector_unit_body=(1.0, 0.01, -0.01),
            boost_cutoff_time_s=4.2,
        ),
        end_time_s=4.8,
        sample_step_s=0.1,
    )
    phases = {sample.source_phase for sample in result.samples}
    stages = {sample.active_stage for sample in result.samples}
    assert "aggregate_rcs" in phases
    assert "mixed_tvc_rcs" in phases
    assert stages == {1, 2, 3}
    assert tuple(event.event_index for event in result.events) == (0, 1, 2, 3, 4)
    assert result.events[1].updated_values == ()
    assert any(sample.runtime_fidelity == "rigid_body_6dof_surface_allocated" for sample in result.samples)
    assert any(sample.runtime_fidelity == "rigid_body_6dof_direct_wrench" for sample in result.samples)
    assert result.samples[-1].propulsion_mode == 0


####


def test_rocket6g_run_is_finite_and_preserves_physical_tvc_request_vs_achievement(tmp_path: Path) -> None:
    definition = load_rocket6g_source_definition(_write_rocket_case(tmp_path))
    result = run_rocket6g_phase_aware_plant(
        definition,
        Rocket6gDirectCommand(tvc_pitch_command_deg=2.0),
        end_time_s=1.2,
        sample_step_s=0.1,
    )
    assert result.terminated_reason == "end_time"
    assert all(math.isfinite(value) for sample in result.samples for value in sample.position_inertial_m)
    mixed = next(sample for sample in result.samples if sample.source_phase == "mixed_tvc_rcs")
    assert mixed.requested_nozzle_deg[0] == pytest.approx(2.0)
    assert mixed.achieved_nozzle_deg[0] != pytest.approx(mixed.requested_nozzle_deg[0])
    quaternion_norms = [np.linalg.norm(sample.quaternion_wxyz) for sample in result.samples]
    assert quaternion_norms == pytest.approx([1.0] * len(quaternion_norms), abs=1.0e-9)


####
