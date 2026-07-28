from __future__ import annotations

import math
from pathlib import Path

import pytest

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[2]
SOURCE_PROBLEM = ROOT / "examples/mission_families/slower_b747/SV01_racetrack_altitude_turns_6dof.prb"
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

    problem = tmp_path / "b747-surface-candidate.prb"
    source = SOURCE_PROBLEM.read_text(encoding="utf-8")
    source = source.replace(
        "*runtime control elevator-deg vehicle=1 default=0.3861602391 lower=-10 upper=10",
        "\n".join(
            (
                "*runtime control elevator-deg vehicle=1 default=0.3861602391 lower=-10 upper=10",
                "*runtime control aileron-deg vehicle=1 default=0 lower=-10 upper=10",
                "*runtime control rudder-deg vehicle=1 default=0 lower=-10 upper=10",
            )
        ),
    )
    source = source.replace(
        "fixed-wing-direct-control-law=local-bank-pitch-pd direct-bank-gain-nm-per-rad=4000000 direct-bank-rate-damping-nm-s-per-rad=3000000 direct-pitch-gain-nm-per-rad=5000000 direct-pitch-rate-damping-nm-s-per-rad=3000000 direct-alpha-gain-nm-per-rad=50000000 direct-beta-gain=0.0 direct-beta-rate-damping-nm-s-per-rad=0.0 direct-beta-force-gain-n-per-rad=100000000 direct-beta-force-damping-n-s-per-m=100000 direct-wrench-cancel-source-moment=true direct-wrench-cancel-source-axes=xyz direct-wrench-cancel-source-side-force=true max-sideslip-deg=4.0 sideslip-gain=0.0 sideslip-rate-damping=0.0 alpha-hold-gain-deg-per-deg=-0.2 alpha-hold-target-deg=3.1 fixed-wing-alpha-command-deg=3.1 fixed-wing-beta-command-deg=0.0 fixed-wing-control-authority=direct-moment maximum-moment=100000000",
        "fixed-wing-surface-control-law=bank-pitch-pd surface-bank-gain-nm-per-rad=4000000 surface-bank-rate-damping-nm-s-per-rad=3000000 surface-pitch-gain-nm-per-rad=5000000 surface-pitch-rate-damping-nm-s-per-rad=3000000 surface-inversion-step-deg=1.0 surface-inversion-max-delta-deg=0.5 surface-inversion-rate-deg-s=40 surface-inversion-regularization=1.0 fixed-wing-alpha-command-deg=3.1 fixed-wing-beta-command-deg=0.0 fixed-wing-control-authority=surfaces surface-control-inversion=true maximum-moment=100000000",
    )
    source = source.replace(
        "duration-s=654.5780264679122",
        "duration-s=20.0",
    ).replace(
        "*when time>664.5780264679122 stop",
        "*when time>20 stop",
    )
    problem.write_text(source, encoding="utf-8")

    report = run_files(
        problem,
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
