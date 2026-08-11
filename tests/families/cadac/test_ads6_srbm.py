from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.ads6_srbm import (
    Ads6SrbmSourceError,
    load_ads6_srbm_source_definition,
    run_ads6_srbm_source_compatibility,
)
from taoryx.families.cadac.source_environment import cadac_source_inverse_square_gravity_mps2


def _write_ads6_srbm_case(
    tmp_path: Path,
    *,
    include_sensor: bool = False,
    seeker_mode: int = 0,
    guidance_mode: int = 0,
    altitude_m: float = 1_000.0,
    speed_mps: float = 300.0,
    flight_path_deg: float = 20.0,
    endo_boundary_m: float = 30_000.0,
    end_time_s: float = 0.2,
    target_position_ned_m: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Path:
    modules = [
        " environment def,exec",
        " kinematics def,init,exec",
        " propulsion def,exec",
        " aerodynamics def,exec",
    ]
    if include_sensor:
        modules.append(" sensor def,exec")
    ####
    modules.extend(
        (
            " guidance def,exec",
            " control def,exec",
            " forces def,exec",
            " newton def,init,exec",
            " intercept def,exec",
        )
    )
    input_path = tmp_path / "input.asc"
    input_path.write_text(
        "\n".join(
            (
                "TITLE ADS6 SRBM synthetic source case",
                "OPTIONS y_plot",
                "MODULES",
                *modules,
                "END",
                "TIMING",
                " int_step 0.01",
                " plot_step 0.05",
                " traj_step 0.05",
                "END",
                "VEHICLES 1",
                " ROCKET5 SRBM",
                "  sael1 0",
                "  sael2 0",
                f"  sael3 {-altitude_m}",
                f"  dvae {speed_mps}",
                "  psivlx 0",
                f"  thtvlx {flight_path_deg}",
                "  alpha_t0x 5",
                "  beta_t0x 0",
                "  AERO_DECK srbm_aero.asc",
                "  area 0.636",
                "  alpmax 28",
                "  mprop 1",
                "  mass_launch 6000",
                "  mass_fuel 4000",
                "  isp 230",
                "  thrust_sl 128600",
                "  aexit 0.282",
                "  maut 1",
                f"  alt_endo {endo_boundary_m}",
                "  ancomx_bias 0.5",
                f"  mseek {seeker_mode}",
                f"  mguide {guidance_mode}",
                "  gnav 2.5",
                f"  stel1 {target_position_ned_m[0]}",
                f"  stel2 {target_position_ned_m[1]}",
                f"  stel3 {target_position_ned_m[2]}",
                "  tgo_manvr 20",
                "  amp_manvr 1",
                "  frq_manvr 1",
                "  tgo63_manvr 5",
                " END",
                f"ENDTIME {end_time_s}",
                "STOP",
                "",
            )
        ),
        encoding="utf-8",
    )
    (tmp_path / "srbm_aero.asc").write_text(
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
    return input_path


####


def test_ads6_srbm_lowering_preserves_roket5_response_law_claim(tmp_path: Path) -> None:
    definition = load_ads6_srbm_source_definition(_write_ads6_srbm_case(tmp_path))

    assert definition.source_model == "ROCKET5"
    assert definition.taoryx_tier == "pseudo_6dof"
    assert definition.control_realization == "response_law"
    assert definition.initial_state.position_ned_m == pytest.approx((0.0, 0.0, -1_000.0))
    assert definition.aerodynamic_deck.table("cltgt_vs_alpha_mach").dimension == 2
    assert len(definition.source_artifacts) == 2


####


def test_ads6_srbm_requires_exactly_one_rocket5_actor(tmp_path: Path) -> None:
    source_path = _write_ads6_srbm_case(tmp_path)
    source_path.write_text(source_path.read_text(encoding="utf-8").replace("ROCKET5", "AIRCRAFT3"), encoding="utf-8")

    with pytest.raises(Ads6SrbmSourceError, match="exactly one ROCKET5"):
        load_ads6_srbm_source_definition(source_path)
    ####


####


def test_ads6_srbm_seeker_mode_requires_sensor_module(tmp_path: Path) -> None:
    source_path = _write_ads6_srbm_case(tmp_path, seeker_mode=1, guidance_mode=1)

    with pytest.raises(ValueError, match="requires the sensor module"):
        load_ads6_srbm_source_definition(source_path)
    ####


####


def test_ads6_srbm_endo_response_states_are_executable(tmp_path: Path) -> None:
    definition = load_ads6_srbm_source_definition(_write_ads6_srbm_case(tmp_path))
    result = run_ads6_srbm_source_compatibility(definition, end_time_s=0.1, sample_step_s=0.05)

    assert result.terminated_reason == "end_time"
    assert len(result.samples) == 3
    assert {sample.phase for sample in result.samples} == {"endo_ascent"}
    assert result.samples[-1].alpha_deg != pytest.approx(result.samples[0].alpha_deg)
    assert result.samples[-1].pitch_response_rate_rad_s != 0.0
    assert result.samples[-1].fidelity == "pseudo_6dof"


####


def test_ads6_srbm_publishes_realized_specific_force_separately_from_source_commands(tmp_path: Path) -> None:
    definition = load_ads6_srbm_source_definition(_write_ads6_srbm_case(tmp_path))
    result = run_ads6_srbm_source_compatibility(definition, end_time_s=0.1, sample_step_s=0.1)
    sample = result.samples[-1]
    gravity_mps2 = cadac_source_inverse_square_gravity_mps2(sample.altitude_m)

    assert sample.normal_acceleration_g == pytest.approx(-sample.specific_force_body_mps2[2] / gravity_mps2)
    assert sample.lateral_acceleration_g == pytest.approx(sample.specific_force_body_mps2[1] / gravity_mps2)


####


####


def test_ads6_srbm_propulsion_decreases_mass_and_preserves_pressure_correction(tmp_path: Path) -> None:
    definition = load_ads6_srbm_source_definition(_write_ads6_srbm_case(tmp_path))
    result = run_ads6_srbm_source_compatibility(definition, end_time_s=0.1, sample_step_s=0.1)

    assert result.samples[-1].mass_kg < result.samples[0].mass_kg
    assert result.samples[0].thrust_n > definition.propulsion.sea_level_thrust_n
    assert result.samples[-1].propulsion_mode == 1


####


def test_ads6_srbm_sticky_exo_flag_yields_exo_then_reentry(tmp_path: Path) -> None:
    definition = load_ads6_srbm_source_definition(
        _write_ads6_srbm_case(
            tmp_path,
            altitude_m=1_010.0,
            speed_mps=200.0,
            flight_path_deg=-80.0,
            endo_boundary_m=1_000.0,
            end_time_s=0.15,
        )
    )
    result = run_ads6_srbm_source_compatibility(definition, sample_step_s=0.01)

    phases = tuple(transition.phase for transition in result.phase_transitions)
    assert phases[0] == "exo_ballistic"
    assert "endo_reentry" in phases
    exo_sample = next(sample for sample in result.samples if sample.phase == "exo_ballistic")
    reentry_sample = next(sample for sample in result.samples if sample.phase == "endo_reentry")
    assert exo_sample.exo_flag is True
    assert exo_sample.alpha_deg == 0.0
    assert exo_sample.pitch_response_rate_rad_s == 0.0
    assert reentry_sample.exo_flag is True


####


def test_ads6_srbm_guided_source_exposes_fixed_target_geometry(tmp_path: Path) -> None:
    definition = load_ads6_srbm_source_definition(
        _write_ads6_srbm_case(
            tmp_path,
            include_sensor=True,
            seeker_mode=1,
            guidance_mode=11,
            altitude_m=1_010.0,
            speed_mps=200.0,
            flight_path_deg=-80.0,
            endo_boundary_m=1_000.0,
            target_position_ned_m=(100.0, 25.0, 0.0),
            end_time_s=0.15,
        )
    )
    result = run_ads6_srbm_source_compatibility(definition, sample_step_s=0.01)

    reentry = next(sample for sample in result.samples if sample.phase == "endo_reentry" and sample.range_to_target_m > 0.0)
    assert reentry.range_to_target_m > 0.0
    assert reentry.time_to_go_s >= 0.0
    assert reentry.control_realization == "response_law"


####


def test_ads6_srbm_ground_impact_is_reported(tmp_path: Path) -> None:
    definition = load_ads6_srbm_source_definition(
        _write_ads6_srbm_case(
            tmp_path,
            altitude_m=2.0,
            speed_mps=100.0,
            flight_path_deg=-80.0,
            endo_boundary_m=1.0,
            end_time_s=1.0,
        )
    )
    result = run_ads6_srbm_source_compatibility(definition, sample_step_s=0.05)

    assert result.terminated_reason == "ground_impact"
    assert result.impact is not None
    assert result.impact.kind == "ground_impact"
    assert result.samples[-1].altitude_m < 0.0


####


def test_ads6_srbm_rejects_nonpositive_runtime_requests(tmp_path: Path) -> None:
    definition = load_ads6_srbm_source_definition(_write_ads6_srbm_case(tmp_path))

    with pytest.raises(ValueError, match="end_time_s"):
        run_ads6_srbm_source_compatibility(definition, end_time_s=0.0)
    ####
    with pytest.raises(ValueError, match="sample_step_s"):
        run_ads6_srbm_source_compatibility(definition, sample_step_s=0.0)
    ####


####
