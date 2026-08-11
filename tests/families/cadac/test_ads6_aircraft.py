from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.ads6_aircraft import (
    Ads6AircraftSourceError,
    Ads6AircraftThreatTrack,
    load_ads6_aircraft_source_definition,
    run_ads6_aircraft_source_compatibility,
)


def _write_ads6_aircraft_case(
    tmp_path: Path,
    *,
    guidance_option: int = 0,
    guidance_gain: float = 0.0,
    turn_load_g: float = 0.0,
    maneuver_start_s: float = 0.0,
    maneuver_stop_s: float = 0.0,
    bank_time_constant_s: float = 0.0,
    bank_limit_deg: float = 80.0,
    load_factor_time_constant_s: float = 0.0,
    alpha_limit_deg: float = 12.0,
    lift_slope_per_deg: float = 0.0523,
    wing_loading_n_m2: float = 3247.0,
    longitudinal_acceleration_g: float = 0.0,
    position_ned_m: tuple[float, float, float] = (0.0, -30_000.0, -10_000.0),
    speed_mps: float = 250.0,
    heading_deg: float = 90.0,
    flight_path_deg: float = 0.0,
    integration_step_s: float = 0.01,
    trajectory_step_s: float = 0.01,
    end_time_s: float = 0.1,
    extra_actor: bool = False,
    actor_event: bool = False,
    stochastic: bool = False,
) -> Path:
    maneuver_lines: list[str] = []
    if guidance_option > 0:
        maneuver_lines.extend(
            (
                f"  man_start {maneuver_start_s}",
                f"  man_stop {maneuver_stop_s}",
            )
        )
    ####
    event_lines = (
        (
            "  IF time > 0.02",
            "   gturn 0",
            "  ENDIF",
        )
        if actor_event
        else ()
    )
    stochastic_lines = ("  GAUSS gturn 0 0.1",) if stochastic else ()
    actor_lines = (
        " AIRCRAFT3 AC1",
        f"  sael1 {position_ned_m[0]}",
        f"  sael2 {position_ned_m[1]}",
        f"  sael3 {position_ned_m[2]}",
        f"  psivlx {heading_deg}",
        f"  thtvlx {flight_path_deg}",
        f"  dvae {speed_mps}",
        f"  acft_option {guidance_option}",
        f"  guid_gain {guidance_gain}",
        f"  gturn {turn_load_g}",
        *maneuver_lines,
        f"  tphi {bank_time_constant_s}",
        f"  tanx {load_factor_time_constant_s}",
        f"  clalpha {lift_slope_per_deg}",
        f"  wingloading {wing_loading_n_m2}",
        f"  philimx {bank_limit_deg}",
        f"  alplimx {alpha_limit_deg}",
        f"  acc_longx {longitudinal_acceleration_g}",
        *stochastic_lines,
        *event_lines,
        " END",
    )
    extra_lines = (
        (
            " AIRCRAFT3 AC2",
            "  sael1 0",
            "  sael2 0",
            "  sael3 -1000",
            "  psivlx 0",
            "  thtvlx 0",
            "  dvae 200",
            "  acft_option 0",
            "  clalpha 0.0523",
            "  wingloading 3247",
            "  philimx 80",
            "  alplimx 12",
            " END",
        )
        if extra_actor
        else ()
    )
    path = tmp_path / "input.asc"
    path.write_text(
        "\n".join(
            (
                "TITLE ADS6 AIRCRAFT3 synthetic source case",
                "OPTIONS y_traj",
                "MODULES",
                " environment def,exec",
                " kinematics def,init,exec",
                " guidance def,exec",
                " control def,exec",
                " forces def,exec",
                " newton def,init,exec",
                "END",
                "TIMING",
                f" int_step {integration_step_s}",
                f" traj_step {trajectory_step_s}",
                "END",
                f"VEHICLES {2 if extra_actor else 1}",
                *actor_lines,
                *extra_lines,
                f"ENDTIME {end_time_s}",
                "STOP",
                "",
            )
        ),
        encoding="utf-8",
    )
    return path


####


def test_ads6_aircraft_lowering_preserves_point_mass_force_model_claim(tmp_path: Path) -> None:
    definition = load_ads6_aircraft_source_definition(_write_ads6_aircraft_case(tmp_path))

    assert definition.source_model == "AIRCRAFT3"
    assert definition.taoryx_tier == "point_mass_3dof"
    assert definition.runtime_fidelity == "point_mass_3dof"
    assert definition.control_realization == "force_model"
    assert definition.module_order == (
        "environment",
        "kinematics",
        "guidance",
        "control",
        "forces",
        "newton",
    )
    assert len(definition.source_artifacts) == 1


####


def test_ads6_aircraft_requires_exactly_one_actor(tmp_path: Path) -> None:
    with pytest.raises(Ads6AircraftSourceError, match="exactly one AIRCRAFT3"):
        load_ads6_aircraft_source_definition(_write_ads6_aircraft_case(tmp_path, extra_actor=True))
    ####


####


def test_ads6_aircraft_rejects_unexecuted_source_events_and_stochastic_inputs(tmp_path: Path) -> None:
    with pytest.raises(Ads6AircraftSourceError, match="does not yet execute actor-local source events"):
        load_ads6_aircraft_source_definition(_write_ads6_aircraft_case(tmp_path, actor_event=True))
    ####
    stochastic_root = tmp_path / "stochastic"
    stochastic_root.mkdir()
    with pytest.raises(Ads6AircraftSourceError, match="does not yet materialize"):
        load_ads6_aircraft_source_definition(_write_ads6_aircraft_case(stochastic_root, stochastic=True))
    ####


####


def test_ads6_aircraft_steady_source_holds_level_speed_and_heading(tmp_path: Path) -> None:
    definition = load_ads6_aircraft_source_definition(_write_ads6_aircraft_case(tmp_path, end_time_s=1.0, trajectory_step_s=0.5))
    result = run_ads6_aircraft_source_compatibility(definition)

    assert result.terminated_reason == "end_time"
    assert result.samples[-1].altitude_m == pytest.approx(10_000.0, abs=1.0e-8)
    assert result.samples[-1].speed_mps == pytest.approx(250.0, rel=1.0e-10)
    assert result.samples[-1].heading_deg == pytest.approx(90.0, abs=1.0e-10)
    assert result.samples[-1].normal_load_factor_g == pytest.approx(1.0)
    assert result.samples[-1].fidelity == "point_mass_3dof"


####


def test_ads6_aircraft_strict_g_turn_window_and_response_states(tmp_path: Path) -> None:
    definition = load_ads6_aircraft_source_definition(
        _write_ads6_aircraft_case(
            tmp_path,
            guidance_option=1,
            turn_load_g=1.5,
            maneuver_start_s=0.02,
            maneuver_stop_s=0.05,
            bank_time_constant_s=0.02,
            load_factor_time_constant_s=0.02,
            end_time_s=0.08,
        )
    )
    result = run_ads6_aircraft_source_compatibility(definition)
    by_time = {round(sample.time_s, 2): sample for sample in result.samples}

    assert by_time[0.02].maneuver_active is False
    assert by_time[0.03].maneuver_active is True
    assert by_time[0.03].mode == "g_turn"
    assert by_time[0.05].maneuver_active is False
    assert tuple((round(item.time_s, 2), item.active) for item in result.maneuver_transitions) == (
        (0.03, True),
        (0.05, False),
    )
    assert max(sample.bank_deg for sample in result.samples) > 0.0
    assert max(sample.normal_load_factor_g for sample in result.samples) > 1.0
    assert result.samples[-1].heading_deg > result.samples[0].heading_deg


####


def test_ads6_aircraft_load_and_bank_limits_are_source_telemetry(tmp_path: Path) -> None:
    definition = load_ads6_aircraft_source_definition(
        _write_ads6_aircraft_case(
            tmp_path,
            guidance_option=1,
            turn_load_g=5.0,
            maneuver_start_s=0.0,
            maneuver_stop_s=0.2,
            bank_limit_deg=10.0,
            alpha_limit_deg=0.2,
            bank_time_constant_s=0.0,
            load_factor_time_constant_s=0.0,
            end_time_s=0.1,
        )
    )
    result = run_ads6_aircraft_source_compatibility(definition)
    active = next(sample for sample in result.samples if sample.maneuver_active)

    assert active.bank_limited is True
    assert abs(active.bank_deg) == pytest.approx(10.0)
    assert active.load_factor_limited is True
    assert abs(active.normal_load_factor_g) == pytest.approx(active.load_factor_limit_g)


####


def test_ads6_aircraft_escape_mode_requires_and_consumes_external_threat(tmp_path: Path) -> None:
    definition = load_ads6_aircraft_source_definition(
        _write_ads6_aircraft_case(
            tmp_path,
            guidance_option=2,
            guidance_gain=2.0,
            maneuver_start_s=0.0,
            maneuver_stop_s=0.2,
            end_time_s=0.1,
        )
    )
    with pytest.raises(Ads6AircraftSourceError, match="requires an external threat track"):
        run_ads6_aircraft_source_compatibility(definition)
    ####
    result = run_ads6_aircraft_source_compatibility(
        definition,
        threat_track=Ads6AircraftThreatTrack(
            position_ned_m=(10_000.0, -30_000.0, -10_000.0),
            velocity_ned_mps=(200.0, 0.0, 0.0),
        ),
    )
    active = next(sample for sample in result.samples if sample.maneuver_active)

    assert active.mode == "escape"
    assert active.threat_range_m is not None
    assert active.threat_range_m > 0.0
    assert active.commanded_bank_deg != 0.0


####


def test_ads6_aircraft_ground_impact_and_runtime_validation(tmp_path: Path) -> None:
    definition = load_ads6_aircraft_source_definition(
        _write_ads6_aircraft_case(
            tmp_path,
            position_ned_m=(0.0, 0.0, -1.0),
            speed_mps=100.0,
            heading_deg=0.0,
            flight_path_deg=-80.0,
            end_time_s=1.0,
        )
    )
    result = run_ads6_aircraft_source_compatibility(definition)

    assert result.terminated_reason == "ground_impact"
    assert result.samples[-1].altitude_m < 0.0
    with pytest.raises(ValueError, match="end_time_s"):
        run_ads6_aircraft_source_compatibility(definition, end_time_s=0.0)
    ####
    with pytest.raises(ValueError, match="sample_step_s"):
        run_ads6_aircraft_source_compatibility(definition, sample_step_s=0.0)
    ####


####
