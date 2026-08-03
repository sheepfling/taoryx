import json
from pathlib import Path

import pytest

from examples.showcases.orbital_insertion_coast_reentry.run_showcase import generate as generate_orbital
from examples.showcases.quadcopter_drone_racetrack.run_showcase import generate as generate_quadcopter
from examples.showcases.suborbital_ballistic_return.run_showcase import generate as generate_suborbital
from taoryx.showcase import validate_showcase_run_artifact_boundary

pytestmark = pytest.mark.artifact


@pytest.mark.parametrize(
    ("generator", "family", "required_panels"),
    [
        (generate_orbital, "orbital", {"phase_timeline", "orbit_state", "orbital_elements", "ground_track", "reentry"}),
        (generate_suborbital, "suborbital", {"phase_timeline", "ascent_return", "speed_mach", "return_corridor", "mass_drag"}),
        (generate_quadcopter, "quadcopter", {"phase_timeline", "ground_track", "altitude_hold", "airspeed_wind", "attitude_rates", "controls"}),
    ],
)
def test_extended_showcase_generators_emit_standard_artifacts(tmp_path: Path, generator, family: str, required_panels: set[str]) -> None:
    reports = generator(tmp_path)

    assert len(reports) == 1
    report = reports[0]
    assert report.family.value == family
    assert report.log_path.exists()
    assert required_panels <= {path.stem for path in report.plot_paths}
    assert (tmp_path / "telemetry.json").exists()
    assert (tmp_path / "telemetry.sqlite").exists()
    manifest = json.loads((tmp_path / "manifest.json").read_text(encoding="utf-8"))
    assert len(manifest["run_artifacts"]) == 1
    validate_showcase_run_artifact_boundary(manifest["run_artifacts"][0])
    payload = json.loads(report.log_path.read_text(encoding="utf-8"))
    assert payload["problem"]
    assert all(panel["rendered"] for panel in payload["panels"])
####
