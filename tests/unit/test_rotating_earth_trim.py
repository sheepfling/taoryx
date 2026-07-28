from __future__ import annotations

import json
from pathlib import Path
from tempfile import TemporaryDirectory

import pytest

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import run_files
from taoryx.trim import TrimSpec, solve_trim
from tools.rotating_earth_trim import NOMINAL_EARTH_RATE_RAD_S, rotating_fixture_source
from tools.solve_x8_trim import COLLECTIVE_TABLE, DIFFERENTIAL_TABLE, STATIC_TABLE, THRUST_TABLE, _problem


def test_rotating_fixture_source_adds_equatorial_transport_to_local_velocity() -> None:
    source = """*earth wgs-84 omega=0
  *initial ecic x=6378137.0 y=0.0 z=0.0 xdt=0.0 ydt=17.9 zdt=0.0
"""

    transformed = rotating_fixture_source(source, NOMINAL_EARTH_RATE_RAD_S)

    assert "omega=7.2921151467e-05" in transformed
    assert "ydt=483.0010942542769" in transformed


def test_trim_result_preserves_operating_point_metadata() -> None:
    spec = TrimSpec(
        state_names=(),
        control_names=("u",),
        residual_names=("error",),
        state_initial={},
        control_initial={"u": 0.0},
        operating_point={"latitude_deg": 34.0, "earth_omega_rad_s": NOMINAL_EARTH_RATE_RAD_S},
    )
    result = solve_trim(spec, lambda _state, controls: {"error": controls["u"] - 1.0})

    assert result.success
    assert result.as_dict()["operating_point"] == {
        "latitude_deg": 34.0,
        "earth_omega_rad_s": NOMINAL_EARTH_RATE_RAD_S,
    }
    assert json.loads(json.dumps(result.as_dict()))["operating_point"]["latitude_deg"] == pytest.approx(34.0)


def test_rotating_x8_propulsion_query_uses_air_relative_speed() -> None:
    with TemporaryDirectory(prefix="taoryx-rotating-x8-test-") as directory:
        root = Path(directory)
        problem = root / "candidate.prb"
        problem.write_text(_problem(7.9, 0.44, -2.35, -2.16, NOMINAL_EARTH_RATE_RAD_S), encoding="utf-8")
        report = run_files(
            problem,
            (STATIC_TABLE, COLLECTIVE_TABLE, DIFFERENTIAL_TABLE, THRUST_TABLE),
            output_dir=root / "run",
            max_steps=2,
            integrator="rk4",
            profile=GrammarProfile.TAORYX,
        )

    assert report.results
    state = report.results[0].states["1"][0].named
    assert state["aero_mach"] == pytest.approx(0.0527077631265, rel=1.0e-8)
    assert state["propulsion_force_body_x_n"] > 3.0
