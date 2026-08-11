from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.ads6_engagement import (
    Ads6EngagementRunConfig,
    Ads6EngagementSession,
    load_ads6_engagement_source_definition,
    run_ads6_engagement,
)


def _write_sam_decks(root: Path) -> None:
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
    lines = ["TITLE synthetic ADS6 SAM aero"]
    for name in three_dimensional:
        if name.startswith("ca0"):
            values = (0.2,) * 8
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
        lines.extend(
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
        lines.extend((f"1DIM {name}", "NX1 2", f"0 {value}", f"5 {value}"))
    ####
    (root / "sam_aero.asc").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (root / "sam_prop.asc").write_text(
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


####


def _write_srbm_decks(root: Path) -> None:
    (root / "srbm_aero.asc").write_text(
        """TITLE synthetic SRBM deck
2DIM cltgt_vs_alpha_mach
NX1 4 NX2 4
0 0.5 0.0 0.0 0.0 0.0
10 1.0 1.0 1.2 1.4 1.6
20 2.0 2.0 2.4 2.8 3.2
30 4.0 3.0 3.6 4.2 4.8
2DIM cdtgt_vs_alpha_mach
NX1 4 NX2 4
0 0.5 0.2 0.3 0.4 0.5
10 1.0 0.3 0.4 0.5 0.6
20 2.0 0.5 0.6 0.7 0.8
30 4.0 0.8 0.9 1.0 1.1
""",
        encoding="utf-8",
    )
    (root / "sam_traj.asc").write_text(
        """TITLE synthetic SAM trajectory
1DIM time_vs_ascent_altitude
NX1 3
0 0
15000 0.02
30000 0.04
1DIM alt_vs_launch_time
NX1 3
0 0
0.02 15000
0.04 30000
""",
        encoding="utf-8",
    )
    (root / "srbm_traj.asc").write_text(
        """TITLE synthetic SRBM trajectory
1DIM apotime_vs_descent_altitude
NX1 3
0 0.1
15000 0.06
30000 0.02
1DIM x_vs_launch_time
NX1 3
0 0
0.1 1000
0.2 2000
1DIM y_vs_launch_time
NX1 3
0 0
0.1 100
0.2 200
1DIM z_vs_launch_time
NX1 3
0 0
0.1 -30000
0.2 0
""",
        encoding="utf-8",
    )


####


def _sam_block(role: str = "SAM1") -> tuple[str, ...]:
    return (
        f" MISSILE6 {role}",
        "  sbel1 0",
        "  sbel2 0",
        "  sbel3 0",
        "  dvbe 16",
        "  psiblx 0",
        "  thtblx 45",
        "  phiblx 0",
        "  alpha0x 0",
        "  beta0x 0",
        "  AERO_DECK sam_aero.asc",
        "  PROP_DECK sam_prop.asc",
        "  xcgref 2.5",
        "  mact 2",
        "  dlimx 28",
        "  ddlimx 600",
        "  wnact 100",
        "  zetact 0.7",
        "  mtvc 0",
        "  mrcs_moment 0",
        "  mrcs_force 0",
        " END",
    )


####


def _source_controller_sam_block(role: str = "SAM1") -> tuple[str, ...]:
    """Return a short source-shaped RF/line-guidance/terminal-PN SAM actor."""

    base = list(_sam_block(role))
    if base[-1] != " END":
        raise AssertionError("synthetic ADS6 SAM block lost its closing END")
    ####
    base.pop()
    base.extend(
        (
            "  mtarget 2",
            "  mins 0",
            "  mseek 12",
            "  skr_dyn 1",
            "  racq_rf 5000",
            "  dtimac_rf 0.01",
            "  forlim_rfx 180",
            "  fovlim_rfx 180",
            "  gain_rf 10",
            "  gain_rf 5",
            "  dblind 1",
            "  maut 2",
            "  alimitx 50",
            "  dplimx 28",
            "  dqlimx 28",
            "  drlimx 28",
            "  zrcl 0.9",
            "  tp 0.1",
            "  zetlagr 1.2",
            "  IF msl_time > 0.01",
            "   maut 3",
            "   mguide 20",
            "   line_gain 1",
            "   nl_gain_fact 0.5",
            "   decrement 700",
            "   thtflx 0",
            "  ENDIF",
            "  IF mseek = 14",
            "   mguide 7",
            "   gnav 3.1",
            "   tgnav 0",
            "  ENDIF",
            " END",
        )
    )
    return tuple(base)


####


def _modules() -> tuple[str, ...]:
    return (
        " environment def,exec",
        " kinematics def,init,exec",
        " propulsion def,exec",
        " aerodynamics def,init,exec",
        " ins def,init,exec",
        " sensor def,exec",
        " guidance def,exec",
        " control def,exec",
        " actuator def,exec",
        " tvc def,exec",
        " rcs def,exec",
        " forces def,exec",
        " euler def,exec",
        " newton def,init,exec",
        " intercept def,exec",
    )


####


def _write_aircraft_engagement(root: Path, *, target_east_m: float = 10_000.0) -> Path:
    _write_sam_decks(root)
    path = root / "input.asc"
    path.write_text(
        "\n".join(
            (
                "TITLE synthetic ADS6 SAM-aircraft package",
                "OPTIONS y_traj",
                "MODULES",
                *_modules(),
                "END",
                "TIMING",
                " int_step 0.01",
                " traj_step 0.01",
                "END",
                "VEHICLES 3",
                *_sam_block(),
                " AIRCRAFT3 AC1",
                "  sael1 0",
                f"  sael2 {target_east_m}",
                "  sael3 -1000",
                "  psivlx -90",
                "  thtvlx 0",
                "  dvae 250",
                "  acft_option 0",
                "  clalpha 0.0523",
                "  wingloading 3247",
                "  philimx 80",
                "  alplimx 12",
                " END",
                " RADAR0 Radar",
                "  mtrack 2",
                "  lethal_rng 20000",
                "  srel1 0",
                "  srel2 0",
                "  srel3 0",
                "  track_step 0.01",
                "  dat_sigma 0",
                "  azat_sigma 0",
                "  elat_sigma 0",
                "  vel_sigma 0",
                " END",
                "ENDTIME 0.04",
                "STOP",
                "",
            )
        ),
        encoding="utf-8",
    )
    return path


####


def _write_source_controller_aircraft_engagement(root: Path) -> Path:
    _write_sam_decks(root)
    path = root / "input-source-controller.asc"
    path.write_text(
        "\n".join(
            (
                "TITLE synthetic ADS6 source-controller package",
                "OPTIONS y_traj y_events",
                "MODULES",
                *_modules(),
                "END",
                "TIMING",
                " int_step 0.01",
                " traj_step 0.01",
                "END",
                "VEHICLES 3",
                *_source_controller_sam_block(),
                " AIRCRAFT3 AC1",
                "  sael1 0",
                "  sael2 1000",
                "  sael3 -1000",
                "  psivlx -90",
                "  thtvlx 0",
                "  dvae 250",
                "  acft_option 0",
                "  clalpha 0.0523",
                "  wingloading 3247",
                "  philimx 80",
                "  alplimx 12",
                " END",
                " RADAR0 Radar",
                "  mtrack 2",
                "  lethal_rng 20000",
                "  srel1 0",
                "  srel2 0",
                "  srel3 0",
                "  track_step 0.01",
                "  dat_sigma 0",
                "  azat_sigma 0",
                "  elat_sigma 0",
                "  vel_sigma 0",
                " END",
                "ENDTIME 0.10",
                "STOP",
                "",
            )
        ),
        encoding="utf-8",
    )
    return path


####


def _write_srbm_engagement(root: Path) -> Path:
    _write_sam_decks(root)
    _write_srbm_decks(root)
    path = root / "input.asc"
    path.write_text(
        "\n".join(
            (
                "TITLE synthetic ADS6 SAM-SRBM package",
                "OPTIONS y_traj",
                "MODULES",
                *_modules(),
                "END",
                "TIMING",
                " int_step 0.01",
                " traj_step 0.01",
                "END",
                "VEHICLES 3",
                *_sam_block(),
                " ROCKET5 SRBM1",
                "  sael1 0",
                "  sael2 1000",
                "  sael3 -15000",
                "  psivlx 0",
                "  thtvlx -10",
                "  dvae 300",
                "  alpha_t0x 5",
                "  beta_t0x 0",
                "  AERO_DECK srbm_aero.asc",
                "  alpmax 28",
                "  mprop 0",
                "  maut 1",
                "  alt_endo 30000",
                "  ancomx_bias 0",
                " END",
                " RADAR0 Radar",
                "  mtrack 1",
                "  alt_engage 15000",
                "  srel1 0",
                "  srel2 0",
                "  srel3 0",
                "  track_step 0.01",
                "  dat_sigma 0",
                "  azat_sigma 0",
                "  elat_sigma 0",
                "  vel_sigma 0",
                "  SAM_DECK sam_traj.asc",
                "  SRBM_DECK srbm_traj.asc",
                " END",
                "ENDTIME 0.05",
                "STOP",
                "",
            )
        ),
        encoding="utf-8",
    )
    return path


####


def test_ads6_aircraft_package_preserves_source_order_and_next_epoch_launch_response(tmp_path: Path) -> None:
    definition = load_ads6_engagement_source_definition(_write_aircraft_engagement(tmp_path))
    result = run_ads6_engagement(
        definition,
        Ads6EngagementRunConfig(end_time_s=0.03, sample_step_s=0.01, command_law="hold"),
    )

    assert result.target_kind == "aircraft"
    assert result.source_order == ("m1", "a1", "f1")
    launch_command = next(event for event in result.events if event.kind == "launch_command")
    missile_launch = next(event for event in result.events if event.kind == "missile_launch")
    assert launch_command.time_s == pytest.approx(0.0)
    assert missile_launch.time_s == pytest.approx(0.01)
    assert missile_launch.time_s - launch_command.time_s == pytest.approx(definition.integration_step_s)
    first_missile_trace = next(item for item in result.packet_traces if item.actor_id == "m1")
    assert first_missile_trace.active is False
    assert tuple(item.actor_id for item in result.objects) == ("m1", "a1", "f1")
    assert result.objects[0].samples[0].held is True
    assert result.objects[0].samples[-1].held is False


####


def test_ads6_persistent_package_session_matches_the_exact_batch_scheduler(tmp_path: Path) -> None:
    """A session consumes the batch scheduler's epochs without replay or reordering."""

    definition = load_ads6_engagement_source_definition(_write_source_controller_aircraft_engagement(tmp_path))
    config = Ads6EngagementRunConfig(end_time_s=0.04, sample_step_s=0.01)
    expected = run_ads6_engagement(definition, config)
    session = Ads6EngagementSession(definition, config)

    while not session.completed:
        session.advance(definition.integration_step_s)
    ####

    assert session.run_result == expected
    assert session.executed_epochs == expected.executed_epochs
    assert session.terminated_reason == expected.terminated_reason


####


def test_ads6_radar_observes_target_packet_published_earlier_in_same_source_epoch(tmp_path: Path) -> None:
    definition = load_ads6_engagement_source_definition(_write_aircraft_engagement(tmp_path))
    result = run_ads6_engagement(
        definition,
        Ads6EngagementRunConfig(end_time_s=0.01, sample_step_s=0.01, command_law="hold"),
    )
    radar_trace = next(item for item in result.packet_traces if item.actor_id == "f1" and item.time_s == 0.01)
    observed = dict(radar_trace.observed_packet_epochs)

    assert observed["m1"] == pytest.approx(0.01)
    assert observed["a1"] == pytest.approx(0.01)


####


def test_ads6_srbm_package_uses_apogee_trajectory_decks_and_keeps_mixed_fidelity_roots(tmp_path: Path) -> None:
    definition = load_ads6_engagement_source_definition(_write_srbm_engagement(tmp_path))
    result = run_ads6_engagement(
        definition,
        Ads6EngagementRunConfig(end_time_s=0.04, sample_step_s=0.01, command_law="hold"),
    )

    assert result.target_kind == "srbm"
    assert result.source_order == ("m1", "r1", "f1")
    command = next(event for event in result.events if event.kind == "launch_command")
    assert command.details["reason"] == "apogee_prediction"
    assert {item.fidelity for item in result.objects} == {
        "rigid_body_6dof_surface_allocated",
        "pseudo_6dof",
        "static_sensor",
    }
    assert result.radar_launch_commands


####


def test_ads6_package_lowering_rejects_mismatched_target_count(tmp_path: Path) -> None:
    path = _write_aircraft_engagement(tmp_path)
    text = path.read_text(encoding="utf-8")
    text = text.replace("VEHICLES 3", "VEHICLES 2").replace(
        " AIRCRAFT3 AC1\n",
        "",
        1,
    )
    path.write_text(text, encoding="utf-8")
    with pytest.raises(Exception):
        load_ads6_engagement_source_definition(path)
    ####


####


def _write_three_aircraft_engagement(root: Path) -> Path:
    _write_sam_decks(root)
    aircraft_blocks: list[str] = []
    for index, east_m in enumerate((10_000.0, 11_000.0, 12_000.0), start=1):
        aircraft_blocks.extend(
            (
                f" AIRCRAFT3 AC{index}",
                f"  sael1 {1_000.0 * (index - 1)}",
                f"  sael2 {east_m}",
                "  sael3 -1000",
                "  psivlx -90",
                "  thtvlx 0",
                "  dvae 250",
                "  acft_option 0",
                "  clalpha 0.0523",
                "  wingloading 3247",
                "  philimx 80",
                "  alplimx 12",
                " END",
            )
        )
    ####
    path = root / "input-three-aircraft.asc"
    path.write_text(
        "\n".join(
            (
                "TITLE synthetic ADS6 three-aircraft package",
                "OPTIONS y_traj",
                "MODULES",
                *_modules(),
                "END",
                "TIMING",
                " int_step 0.01",
                " traj_step 0.01",
                "END",
                "VEHICLES 7",
                *_sam_block("SAM1"),
                *_sam_block("SAM2"),
                *_sam_block("SAM3"),
                *aircraft_blocks,
                " RADAR0 Radar",
                "  mtrack 2",
                "  lethal_rng 20000",
                "  srel1 0",
                "  srel2 0",
                "  srel3 0",
                "  track_step 0.01",
                "  dat_sigma 0",
                "  azat_sigma 0",
                "  elat_sigma 0",
                "  vel_sigma 0",
                "  lnch_dly_bias2 0.02",
                "  lnch_dly_bias3 0.04",
                " END",
                "ENDTIME 0.06",
                "STOP",
                "",
            )
        ),
        encoding="utf-8",
    )
    return path


####


def _write_three_srbm_engagement(root: Path) -> Path:
    _write_sam_decks(root)
    _write_srbm_decks(root)
    rocket_blocks: list[str] = []
    for index, launch_delay_s in enumerate((0.0, 0.01, 0.02), start=1):
        rocket_blocks.extend(
            (
                f" ROCKET5 SRBM{index}",
                f"  launch_delay {launch_delay_s}",
                f"  sael1 {1_000.0 * (index - 1)}",
                "  sael2 1000",
                "  sael3 -15000",
                "  psivlx 0",
                "  thtvlx -10",
                "  dvae 300",
                "  alpha_t0x 5",
                "  beta_t0x 0",
                "  AERO_DECK srbm_aero.asc",
                "  alpmax 28",
                "  mprop 0",
                "  maut 1",
                "  alt_endo 30000",
                "  ancomx_bias 0",
                " END",
            )
        )
    ####
    path = root / "input-three-srbm.asc"
    path.write_text(
        "\n".join(
            (
                "TITLE synthetic ADS6 three-SRBM package",
                "OPTIONS y_traj",
                "MODULES",
                *_modules(),
                "END",
                "TIMING",
                " int_step 0.01",
                " traj_step 0.01",
                "END",
                "VEHICLES 7",
                *_sam_block("SAM1"),
                *_sam_block("SAM2"),
                *_sam_block("SAM3"),
                *rocket_blocks,
                " RADAR0 Radar",
                "  mtrack 1",
                "  alt_engage 15000",
                "  srel1 0",
                "  srel2 0",
                "  srel3 0",
                "  track_step 0.01",
                "  dat_sigma 0",
                "  azat_sigma 0",
                "  elat_sigma 0",
                "  vel_sigma 0",
                "  SAM_DECK sam_traj.asc",
                "  SRBM_DECK srbm_traj.asc",
                "  lnch_dly_bias2 0.01",
                "  lnch_dly_bias3 0.02",
                " END",
                "ENDTIME 0.08",
                "STOP",
                "",
            )
        ),
        encoding="utf-8",
    )
    return path


####


def test_ads6_three_aircraft_package_preserves_tail_number_pairing_and_launch_biases(tmp_path: Path) -> None:
    definition = load_ads6_engagement_source_definition(_write_three_aircraft_engagement(tmp_path))
    result = run_ads6_engagement(
        definition,
        Ads6EngagementRunConfig(end_time_s=0.06, sample_step_s=0.01, command_law="hold"),
    )

    assert result.source_order == ("m1", "m2", "m3", "a1", "a2", "a3", "f1")
    commands = tuple(event for event in result.events if event.kind == "launch_command")
    launches = tuple(event for event in result.events if event.kind == "missile_launch")
    assert tuple(event.related_actor_id for event in commands) == ("m1", "m2", "m3")
    assert tuple(event.details["target_actor_id"] for event in commands) == ("a1", "a2", "a3")
    assert tuple(float(event.details["launch_time_s"]) for event in commands) == pytest.approx((0.0, 0.02, 0.04))
    assert tuple(event.actor_id for event in launches) == ("m1", "m2", "m3")
    assert tuple(event.time_s for event in launches) == pytest.approx((0.01, 0.02, 0.04))
    assert len(result.objects) == 7
    assert len(result.radar_tracks) == 21


####


def test_ads6_three_srbm_package_uses_one_apogee_epoch_with_per_missile_biases(tmp_path: Path) -> None:
    definition = load_ads6_engagement_source_definition(_write_three_srbm_engagement(tmp_path))
    result = run_ads6_engagement(
        definition,
        Ads6EngagementRunConfig(end_time_s=0.08, sample_step_s=0.01, command_law="hold"),
    )

    assert result.source_order == ("m1", "m2", "m3", "r1", "r2", "r3", "f1")
    commands = tuple(event for event in result.events if event.kind == "launch_command")
    assert tuple(event.related_actor_id for event in commands) == ("m1", "m2", "m3")
    assert tuple(event.details["target_actor_id"] for event in commands) == ("r1", "r2", "r3")
    assert tuple(float(event.details["launch_time_s"]) for event in commands) == pytest.approx((0.04, 0.05, 0.06))
    launches = tuple(event for event in result.events if event.kind == "missile_launch")
    assert tuple(event.actor_id for event in launches) == ("m1", "m2", "m3")
    assert tuple(event.time_s for event in launches) == pytest.approx((0.04, 0.05, 0.06))
    assert len(result.objects) == 7


####
