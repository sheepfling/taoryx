"""Same-plant parity checks for the Skywalker X8 point-mass reduction."""

from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import run_files

pytestmark = pytest.mark.x8

ROOT = Path(__file__).resolve().parents[2]
TABLE_ROOT = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables"
TABLES = tuple(
    TABLE_ROOT / name
    for name in (
        "skywalker_x8_static_6axis.tbl",
        "skywalker_x8_collective_elevon_6axis.tbl",
        "skywalker_x8_differential_elevon_6axis.tbl",
        "skywalker_x8_thrust.tbl",
    )
)
REDUCTION = ROOT / "examples/generated/vehicles/skywalker_x8_source_trim_reduction_3dof.prb"
RIGID_BODY = ROOT / "examples/mission_families/slower_x8/SV03_source_trim_hold_30_6dof.prb"
RIGID_PARITY = RIGID_BODY


def test_x8_point_mass_reduction_uses_the_rigid_body_source_deck() -> None:
    """The two modes agree at the common initial aerodynamic query.

    The comparison deliberately converts the point-mass state from TAOS's
    canonical English units to the SI units used by the source tables.  This
    catches the exact class of boundary error that previously made a 17.9
    m/s X8 case query the tables at 58.7 as though it were m/s.
    """

    reduction = run_files(REDUCTION, TABLES, max_steps=1, profile=GrammarProfile.TAORYX)
    rigid_body = run_files(RIGID_BODY, TABLES, max_steps=1, profile=GrammarProfile.TAORYX)
    assert reduction.results, [(item.code, item.message) for item in reduction.diagnostics]
    assert rigid_body.results, [(item.code, item.message) for item in rigid_body.diagnostics]

    point = reduction.results[0].states["1"][0].named
    rigid = rigid_body.results[0].states["1"][0].named

    assert point["vel"] * 0.3048 == pytest.approx(rigid["aero_airspeed_m_s"], abs=1e-10)
    assert point["alt"] * 0.3048 == pytest.approx(rigid["aero_query_altitude_m"], abs=1e-10)
    q_s = rigid["aero_dynamic_pressure_pa"] * 0.75
    assert q_s > 0.0
    for point_name, force_name in (
        ("cx", "aero_force_body_x_n"),
        ("cy", "aero_force_body_y_n"),
        ("cz", "aero_force_body_z_n"),
    ):
        assert point[point_name] == pytest.approx(rigid[force_name] / q_s, abs=3e-9)
    # The composed coefficient names are part of the evidence contract.  They
    # must be live table evaluations, not initialization snapshots, and the
    # active *aero assignment must consume the same values.
    for component in ("cx", "cy", "cz"):
        assert point[f"x8-{component}"] == pytest.approx(point[component], abs=1e-12)
    assert point["thrust"] == pytest.approx(rigid["thrust_force_n"], abs=1e-10)
    # The reduction applies the solved source command directly; the rigid
    # plant reports the achieved actuator state after its first controller /
    # actuator realization.  Keep this as a tight engineering tolerance, but
    # do not confuse two valid representations with bitwise identity.
    assert point["collective-elevon-deg"] == pytest.approx(
        rigid["aero_query_collective-elevon-deg"], abs=1e-6
    )
    assert point["differential-elevon-deg"] == pytest.approx(
        rigid["aero_query_differential-elevon-deg"], abs=1e-6
    )
    ####


def test_x8_point_mass_and_rigid_body_short_histories_remain_in_parity() -> None:
    """The source-composed reduction tracks the rigid plant over 100 ms."""

    reduction = run_files(REDUCTION, TABLES, max_steps=20, profile=GrammarProfile.TAORYX)
    rigid_body = run_files(RIGID_PARITY, TABLES, max_steps=20, profile=GrammarProfile.TAORYX)
    assert reduction.results and rigid_body.results
    point_history = reduction.results[0].states["1"]
    rigid_history = rigid_body.results[0].states["1"]
    assert len(point_history) == len(rigid_history)
    for point, rigid in zip(point_history, rigid_history, strict=True):
        assert point.time == pytest.approx(rigid.time, abs=1.0e-12)
        assert point.named["vel"] * 0.3048 == pytest.approx(rigid.named["speed_m_s"], abs=0.03)
        # The 6DOF attitude is free to respond to the source moment while the
        # reduction prescribes alpha; this gate intentionally covers only the
        # common 100 ms phase window before that assumption diverges.
        assert point.named["alt"] * 0.3048 == pytest.approx(rigid.named["altitude_m"], abs=0.06)
    ####
