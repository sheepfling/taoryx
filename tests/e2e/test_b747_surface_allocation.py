from __future__ import annotations

import math
from pathlib import Path

import pytest

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[2]
SURFACE_PROBLEM = ROOT / "examples/mission_families/slower_b747/SV01_racetrack_surface_allocation_6dof.prb"
TABLE_ROOT = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables"
TABLES = tuple(
    TABLE_ROOT / name
    for name in (
        "b747_nominal_static_6axis.tbl",
        "b747_nominal_elevator_6axis.tbl",
        "b747_nominal_aileron_6axis.tbl",
        "b747_nominal_rudder_6axis.tbl",
        "b747_jt9d_thrust.tbl",
    )
)

pytestmark = [pytest.mark.slow, pytest.mark.b747]


def test_b747_racetrack_allocates_bounded_surface_commands(tmp_path: Path) -> None:
    """The B747 surface candidate routes three source-backed controls.

    This is an allocation and telemetry seam test.  It deliberately does not
    assert that the resulting trajectory is flight-control qualified.
    """

    report = run_files(
        SURFACE_PROBLEM,
        TABLES,
        output_dir=tmp_path / "run",
        max_steps=3_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    assert report.results
    history = tuple(
        {"time_s": state.time, **dict(state.named)}
        for state in report.results[0].states["1"]
    )
    assert history[-1]["time_s"] == pytest.approx(20.0, abs=1.0e-10)
    assert {int(sample["surface_allocation_active_surface_count"]) for sample in history} == {3}
    for sample in history:
        assert math.isfinite(sample["surface_allocation_residual_nm"])
        assert -10.0 <= sample["surface_allocation_elevator_achieved_deg"] <= 10.0
        assert -10.0 <= sample["surface_allocation_aileron_achieved_deg"] <= 10.0
        assert -10.0 <= sample["surface_allocation_rudder_achieved_deg"] <= 10.0
    requested_moment_excursions = {
        axis: max(abs(sample[f"surface_allocation_requested_moment_{axis}_nm"]) for sample in history)
        for axis in ("x", "y", "z")
    }
    assert max(requested_moment_excursions.values()) > 1.0e-4, requested_moment_excursions
    ####
