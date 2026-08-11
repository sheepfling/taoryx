from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from taoryx.families.cadac.ghame6 import (
    Ghame6DirectCommand,
    Ghame6SourceError,
    Ghame6SurfaceActuatorConfig,
    Ghame6SurfaceState,
    ghame6_atmosphere,
    ghame6_surface_step,
    load_ghame6_source_definition,
    run_ghame6_phase_aware_mission,
)

_AERO_TABLES = (
    "cd0_vs_alpha_mach",
    "cda_vs_alpha_mach",
    "cl0_vs_alpha_mach",
    "cla_vs_alpha_mach",
    "clde_vs_alpha_mach",
    "cyb_vs_alpha_mach",
    "cyda_vs_alpha_mach",
    "cydr_vs_alpha_mach",
    "cllb_vs_alpha_mach",
    "cllda_vs_alpha_mach",
    "clldr_vs_alpha_mach",
    "cllp_vs_alpha_mach",
    "cllr_vs_alpha_mach",
    "cm0_vs_alpha_mach",
    "cma_vs_alpha_mach",
    "cmde_vs_alpha_mach",
    "cmq_vs_alpha_mach",
    "clnb_vs_alpha_mach",
    "clnda_vs_alpha_mach",
    "clndr_vs_alpha_mach",
    "clnp_vs_alpha_mach",
    "clnr_vs_alpha_mach",
)


def _write_ghame6_case(root: Path, *, include_tvc: bool = False) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    tvc_module = " tvc def,exec\n" if include_tvc else ""
    input_path = root / "input.asc"
    input_path.write_text(
        f"""TITLE synthetic GHAME6 phase program
OPTIONS y_events y_plot
MODULES
 kinematics def,init,exec
 environment def,init,exec
 aerodynamics def,init,exec
 propulsion def,init,exec
 gps def,exec
 startrack def,exec
 ins def,init,exec
 datalink def,exec
 seeker def,exec
 guidance def,exec
 control def,exec
 actuator def,exec
 rcs def,exec
{tvc_module} forces def,exec
 newton def,init,exec
 euler def,init,exec
 intercept def,exec
END
TIMING
 plot_step 0.1
 int_step 0.05
END
VEHICLES 3
 HYPER6 SyntheticGHAME
  minit 0
  lonx -80
  latx 28
  alt 100000
  dvbe 100
  phibdx 0
  thtbdx 5
  psibdx 45
  alpha0x 2
  beta0x 0
  ppx 0
  qqx 0
  rrx 0
  mair 100
  maero 1
  AERO_DECK aero.asc
  PROP_DECK prop.asc
  alpplimx 21
  alpnlimx -3
  strct_pos_limitx 3
  strct_neg_limitx -2
  mprop 0
  vmass0 1000
  fmass0 100
  qhold 58000
  tq 1
  acowl 10
  throttle 0.1
  thrtl_idle 0.05
  thrtl_max 2
  mact 2
  dlimx 20
  ddlimx 400
  wnact 20
  zetact 0.7
  mguide 4
  maut 24
  mseek 0
  mrcs_moment 0
  mrcs_force 0
  rcs_zeta 0.7
  rcs_freq 0.4
  roll_mom_max 0
  pitch_mom_max 0
  yaw_mom_max 0
  side_force_max 0
  dead_zone 0.4
  hysteresis 0.1
  rcs_tau 1
  thtbdcomx 5
  psibdcomx 45
  racq 10000000
  dtimac 0.1
  dblind 1
  fovlimx 85
  IF event_time > 0.1
   qhold 60000
  ENDIF
  IF alt > 100
   mguide 0
   mact 0
   maut 0
   maero 2
   mprop 4
   vmass0 10
   fmass0 2
   fmasse 0
   moi_roll_exo_0 20
   moi_roll_exo_1 10
   moi_trans_exo_0 40
   moi_trans_exo_1 20
   isp_fuel 100
   fuel_flow_rate 1
   mrcs_moment 21
   rcs_zeta 0.7
   rcs_freq 0.4
   roll_mom_max 5
   pitch_mom_max 10
   yaw_mom_max 10
   side_force_max 100
   dead_zone 0.4
   hysteresis 0.1
   rcs_tau 1
  ENDIF
  IF fmassr < 1.99
   mrcs_moment 12
   mguide 5
   fmasse 0
   vmass0 8
   fmass0 1
   moi_roll_exo_0 10
   moi_roll_exo_1 8
   moi_trans_exo_0 20
   moi_trans_exo_1 16
   isp_fuel 100
   fuel_flow_rate 0.5
  ENDIF
  IF beco_flag = 1
   mseek 2
   mprop 4
   vmass0 5
   fmass0 1
   fmasse 0
   moi_roll_exo_0 5
   moi_roll_exo_1 4
   moi_trans_exo_0 10
   moi_trans_exo_1 8
   isp_fuel 100
   fuel_flow_rate 0.1
   mrcs_moment 22
   roll_mom_max 2
   pitch_mom_max 3
   yaw_mom_max 3
   mguide 8
  ENDIF
  IF mseek = 4
   int_step_new 0.025
   mguide 6
   mprop 4
   mrcs_moment 12
   mrcs_force 1
   roll_mom_max 1
   pitch_mom_max 2
   yaw_mom_max 2
   fuel_flow_rate 0.05
   side_force_max 50
  ENDIF
 END
 SAT3 Satellite
  minit 1
  semi 7000000
  ecc 0.01
  inclx 30
  lon_anodex 20
  arg_perix 10
  true_anomx 0
  sat_thrust 0
  sat_mass 100
 END
 RADAR0 GroundRadar
  radar_on 1
  track_step 0.1
  lonx -25
  latx 37
  alt 0
  dat_sigma 0
  azat_sigma 0
  elat_sigma 0
  vel_sigma 0
 END
ENDTIME 2
STOP
""",
        encoding="utf-8",
    )
    aero_lines = ["TITLE synthetic GHAME6 aero"]
    for name in _AERO_TABLES:
        aero_lines.extend((f"2DIM {name}", "NX1 2 NX2 2", "-10 0 0 0", "20 20 0 0"))
    ####
    (root / "aero.asc").write_text("\n".join(aero_lines) + "\n", encoding="utf-8")
    (root / "prop.asc").write_text(
        """TITLE synthetic GHAME6 propulsion
2DIM spi_vs_throttle_mach
NX1 2 NX2 2
0 0 100 100
2 20 100 100
2DIM ca_vs_alpha_mach
NX1 2 NX2 2
-10 0 1 1
20 20 1 1
""",
        encoding="utf-8",
    )
    return input_path


####


def test_ghame6_lowering_preserves_three_actor_source_order_and_no_tvc(tmp_path: Path) -> None:
    definition = load_ghame6_source_definition(_write_ghame6_case(tmp_path))

    assert definition.actor_order == ("HYPER6", "SAT3", "RADAR0")
    assert "tvc" not in definition.module_order
    assert len(definition.events) == 5
    assert definition.initial_state.altitude_m == pytest.approx(100_000.0)
    assert definition.radar.enabled
    assert definition.taoryx_tier == "rigid_body_6dof_surface_allocated"


####


def test_ghame6_rejects_invented_tvc_source_realization(tmp_path: Path) -> None:
    with pytest.raises(Ghame6SourceError, match="must not be promoted to a TVC"):
        load_ghame6_source_definition(_write_ghame6_case(tmp_path, include_tvc=True))
    ####


####


def test_ghame6_nasa_atmosphere_matches_source_breakpoint_values() -> None:
    density, pressure, temperature, speed_of_sound = ghame6_atmosphere(100, 100_000.0)

    assert pressure == pytest.approx(0.032011, rel=1.0e-12)
    assert temperature == pytest.approx(195.08134433524688)
    assert density == pytest.approx(5.604006613716597e-7)
    assert speed_of_sound > 0.0
    assert ghame6_atmosphere(0, 100_000.0)[:3] == pytest.approx((0.0, 0.0, 186.946))


####


def test_ghame6_surface_mixer_preserves_requested_and_achieved_physical_states() -> None:
    config = Ghame6SurfaceActuatorConfig(
        mode=2,
        position_limit_deg=20.0,
        rate_limit_deg_s=400.0,
        natural_frequency_rad_s=20.0,
        damping_ratio=0.7,
    )
    command = Ghame6DirectCommand(aileron_command_deg=1.0, elevator_command_deg=2.0, rudder_command_deg=-0.5)
    first = ghame6_surface_step(config, Ghame6SurfaceState(), command, dt_s=0.01)
    second = ghame6_surface_step(config, first.state, command, dt_s=0.01)

    assert first.requested_surfaces_deg == pytest.approx((3.0, 1.0, -0.5))
    assert first.achieved_surfaces_deg == pytest.approx((0.0, 0.0, 0.0))
    assert second.achieved_surfaces_deg[0] > second.achieved_surfaces_deg[1] > 0.0
    assert second.achieved_surfaces_deg[2] < 0.0
    assert second.achieved_control_deg[0] > 0.0
    assert second.active

    direct = ghame6_surface_step(config, Ghame6SurfaceState(), command, dt_s=0.01, mode_override=0)
    assert direct.active
    assert direct.achieved_surfaces_deg == pytest.approx((3.0, 1.0, -0.5))


####


def test_ghame6_phase_program_transitions_t4_to_all_source_rcs_phases(tmp_path: Path) -> None:
    definition = load_ghame6_source_definition(_write_ghame6_case(tmp_path))
    result = run_ghame6_phase_aware_mission(
        definition,
        Ghame6DirectCommand(
            aileron_command_deg=1.0,
            elevator_command_deg=2.0,
            rudder_command_deg=0.5,
            boost_cutoff_time_s=0.65,
            terminal_lock_time_s=0.85,
        ),
        end_time_s=1.2,
        sample_step_s=0.05,
        random_seed=7,
    )

    assert result.terminated_reason == "end_time"
    assert tuple(event.event_index for event in result.events) == (0, 1, 2, 3, 4)
    assert {sample.source_phase for sample in result.samples} == {
        "atmospheric_surfaces",
        "transfer_angle_rcs",
        "transfer_vector_rcs",
        "interceptor_glideslope_rcs",
        "interceptor_terminal_rcs",
    }
    assert {sample.runtime_fidelity for sample in result.samples} == {
        "rigid_body_6dof_surface_allocated",
        "rigid_body_6dof_direct_wrench",
    }
    assert result.final_integration_step_s == pytest.approx(0.025)
    assert result.radar_update_count >= 10
    assert len(result.samples) == len(result.satellite_samples) == len(result.radar_samples)
    assert all(math.isfinite(value) for sample in result.samples for value in sample.position_inertial_m)
    assert [np.linalg.norm(sample.quaternion_wxyz) for sample in result.samples] == pytest.approx(
        [1.0] * len(result.samples),
        abs=1.0e-9,
    )
    assert result.samples[0].requested_surfaces_deg == pytest.approx((3.0, 1.0, 0.5))
    assert result.samples[-1].rcs_force_mode == 1
    assert any(sample.thrust_n > 0.0 for sample in result.samples if sample.propulsion_mode in {3, 4})
    assert result.samples[-1].remaining_fuel_kg < 1.0


####
