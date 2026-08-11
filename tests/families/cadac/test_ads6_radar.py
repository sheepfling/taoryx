from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.ads6_radar import (
    Ads6RadarMissileTruth,
    Ads6RadarRuntime,
    Ads6RadarSourceDefinition,
    Ads6RadarTargetTruth,
    lower_ads6_radar_actor,
)
from taoryx.families.cadac.bundle import load_cadac_source_bundle


def _write_radar_case(tmp_path: Path, *, track_mode: int) -> Path:
    extra = (
        ("  lethal_rng 20000",)
        if track_mode == 2
        else (
            "  alt_engage 15000",
            "  SAM_DECK SAM_traj_deck.asc",
            "  SRBM_DECK SRBM_traj_deck.asc",
        )
    )
    path = tmp_path / "input.asc"
    path.write_text(
        "\n".join(
            (
                "TITLE ADS6 RADAR0 synthetic package",
                "OPTIONS y_traj",
                "MODULES",
                " sensor def,exec",
                "END",
                "TIMING",
                " int_step 0.01",
                "END",
                "VEHICLES 1",
                " RADAR0 Radar",
                f"  mtrack {track_mode}",
                "  srel1 0",
                "  srel2 0",
                "  srel3 0",
                "  track_step 0.1",
                "  dat_sigma 0",
                "  azat_sigma 0",
                "  elat_sigma 0",
                "  vel_sigma 0",
                "  lnch_dly_bias2 2",
                *extra,
                " END",
                "ENDTIME 10",
                "STOP",
                "",
            )
        ),
        encoding="utf-8",
    )
    if track_mode == 1:
        (tmp_path / "SAM_traj_deck.asc").write_text(
            "\n".join(
                (
                    "TITLE synthetic SAM trajectory",
                    "1DIM time_vs_ascent_altitude",
                    "NX1 3",
                    "0 0",
                    "15000 20",
                    "30000 40",
                    "1DIM alt_vs_launch_time",
                    "NX1 3",
                    "0 0",
                    "20 15000",
                    "40 30000",
                    "",
                )
            ),
            encoding="utf-8",
        )
        (tmp_path / "SRBM_traj_deck.asc").write_text(
            "\n".join(
                (
                    "TITLE synthetic SRBM trajectory",
                    "1DIM apotime_vs_descent_altitude",
                    "NX1 3",
                    "0 100",
                    "15000 60",
                    "30000 20",
                    "1DIM x_vs_launch_time",
                    "NX1 3",
                    "0 0",
                    "100 100000",
                    "200 200000",
                    "1DIM y_vs_launch_time",
                    "NX1 3",
                    "0 0",
                    "100 1000",
                    "200 2000",
                    "1DIM z_vs_launch_time",
                    "NX1 3",
                    "0 0",
                    "100 -30000",
                    "200 0",
                    "",
                )
            ),
            encoding="utf-8",
        )
    ####
    return path


####


def _definition(tmp_path: Path, *, track_mode: int) -> Ads6RadarSourceDefinition:
    bundle = load_cadac_source_bundle(_write_radar_case(tmp_path, track_mode=track_mode))
    return lower_ads6_radar_actor(bundle, bundle.case.vehicle("RADAR0"))


####


def test_ads6_radar_aircraft_lethal_range_latches_launch_and_obeys_track_cadence(tmp_path: Path) -> None:
    runtime = Ads6RadarRuntime(_definition(tmp_path, track_mode=2), seed=7)
    outside = Ads6RadarTargetTruth(
        actor_id="a1",
        position_ned_m=(0.0, -30_000.0, -10_000.0),
        velocity_ned_mps=(0.0, 250.0, 0.0),
    )
    first = runtime.step(0.0, (outside,))
    skipped = runtime.step(0.05, (outside,))
    inside = outside.model_copy(update={"position_ned_m": (0.0, -15_000.0, -10_000.0)})
    second = runtime.step(0.1, (inside,))

    assert first.tracked is True
    assert first.launch_commands[0].launch_time_s == pytest.approx(9_999.0)
    assert first.launch_commands[0].newly_latched is False
    assert skipped.tracked is False
    assert second.launch_commands[0].newly_latched is True
    assert second.launch_commands[0].launch_time_s == pytest.approx(0.1)
    assert runtime.launch_times_s[0] == pytest.approx(0.1)
    assert runtime.intercept_points_ned_m[0] == pytest.approx(inside.position_ned_m)


####


def test_ads6_radar_aircraft_pairing_applies_per_missile_launch_bias(tmp_path: Path) -> None:
    runtime = Ads6RadarRuntime(_definition(tmp_path, track_mode=2))
    targets = (
        Ads6RadarTargetTruth(actor_id="a1", position_ned_m=(0.0, 10_000.0, 0.0), velocity_ned_mps=(0.0, 0.0, 0.0)),
        Ads6RadarTargetTruth(actor_id="a2", position_ned_m=(0.0, 10_000.0, 0.0), velocity_ned_mps=(0.0, 0.0, 0.0)),
    )
    result = runtime.step(3.0, targets)

    assert tuple(item.missile_index for item in result.launch_commands) == (1, 2)
    assert result.launch_commands[0].launch_time_s == pytest.approx(3.0)
    assert result.launch_commands[1].launch_time_s == pytest.approx(5.0)


####


def test_ads6_radar_srbm_apogee_prediction_and_intercept_point_refinement(tmp_path: Path) -> None:
    runtime = Ads6RadarRuntime(_definition(tmp_path, track_mode=1))
    ascending = Ads6RadarTargetTruth(
        actor_id="r1",
        position_ned_m=(50_000.0, 500.0, -30_000.0),
        velocity_ned_mps=(500.0, 0.0, -100.0),
    )
    before = runtime.step(10.0, (ascending,))
    descending = ascending.model_copy(update={"velocity_ned_mps": (500.0, 0.0, 100.0)})
    apogee = runtime.step(10.1, (descending,))

    assert before.launch_commands == ()
    command = apogee.launch_commands[0]
    assert command.newly_latched is True
    assert command.launch_time_s == pytest.approx(50.1)
    assert command.intercept_point_ned_m == pytest.approx((70_100.0, 701.0, -15_000.0))

    after = runtime.step(
        50.2,
        (descending.model_copy(update={"position_ned_m": (60_000.0, 600.0, -14_000.0)}),),
        missiles=(
            Ads6RadarMissileTruth(
                actor_id="m1",
                position_ned_m=(0.0, 0.0, -10_000.0),
                velocity_ned_mps=(0.0, 0.0, -500.0),
                launched=True,
            ),
        ),
    )
    assert after.launch_commands[0].newly_latched is False
    assert after.launch_commands[0].intercept_point_ned_m != command.intercept_point_ned_m


####


def test_ads6_radar_source_lowering_rejects_missing_srbm_trajectory_decks(tmp_path: Path) -> None:
    path = _write_radar_case(tmp_path, track_mode=1)
    (tmp_path / "SAM_traj_deck.asc").unlink()
    with pytest.raises(FileNotFoundError, match="SAM_traj_deck"):
        load_cadac_source_bundle(path)
    ####


####
