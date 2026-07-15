from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[1].parent / "examples" / "vehicle_families" / "rigid_body_matrix"
AERO = Path(__file__).resolve().parents[1].parent / "examples" / "showcases" / "california_to_hawaii" / "aero.tbl"


@pytest.mark.parametrize("family", ("rocket", "glider", "drone"))
def test_rigid_body_vehicle_families_use_the_same_source_runner(family: str, tmp_path: Path) -> None:
    report = run_files(
        ROOT / f"{family}.prb",
        (AERO,) if family != "rocket" else (),
        output_dir=tmp_path / family,
        max_steps=100,
        profile=GrammarProfile.TAORYX,
    )

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    assert report.cases == 1
    assert report.results[0].completed
    assert report.results[0].states["1"][-1].time == pytest.approx(0.2)
    artifact = report.artifacts[0]
    vehicle = artifact.vehicles["1"]
    assert vehicle.dynamics.value == "rigid_body_6dof"
    source_names = {channel.source_name for channel in vehicle.channels.values()}
    assert "mass_kg" in source_names
    assert "attitude_controller_saturated" in source_names
    if family == "rocket":
        propellant = next(channel for channel in vehicle.channels.values() if channel.source_name == "propellant_mass_kg")
        assert propellant.values[-1] < propellant.values[0]
    else:
        aero_active = next(channel for channel in vehicle.channels.values() if channel.source_name == "aero_active")
        assert aero_active.values[-1] == pytest.approx(1.0)
