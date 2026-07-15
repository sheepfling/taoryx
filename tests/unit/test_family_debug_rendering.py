import json

from taoryx.family_debug_rendering import render_family_debug_artifacts
from taoryx.family_debugging import DebugFamily
from taoryx.outputs import DynamicsKind, EventRecord, RunArtifact, TelemetryChannel, VehicleKind, VehicleTelemetry


def _artifact() -> RunArtifact:
    times = [0.0, 1.0, 2.0]
    channels = {
        "position.altitude.geodetic": TelemetryChannel(source_name="alt", semantic_name="position.altitude.geodetic", unit="m", values=[0.0, 10.0, 20.0]),
        "mass.total": TelemetryChannel(source_name="mass", semantic_name="mass.total", unit="kg", values=[10.0, 9.0, 8.0]),
        "propulsion.thrust": TelemetryChannel(source_name="thrust", semantic_name="propulsion.thrust", unit="N", values=[0.0, 5.0, 0.0]),
    }
    vehicle = VehicleTelemetry(
        vehicle_id="demo",
        name="Demo",
        kind=VehicleKind.ROCKET,
        dynamics=DynamicsKind.POINT_MASS_3DOF,
        times=times,
        channels=channels,
        events=[EventRecord(time=1.0, vehicle="demo", name="stage-separation", kind="segment_transition")],
    )
    return RunArtifact(problem="demo.prb", vehicles={"demo": vehicle})
####


def test_family_renderer_writes_pngs_and_structured_log(tmp_path) -> None:
    reports = render_family_debug_artifacts(_artifact(), tmp_path, DebugFamily.ROCKET)

    assert len(reports) == 1
    report = reports[0]
    assert {path.name for path in report.plot_paths} == {"phase_timeline.png", "altitude_range.png", "mass_propulsion.png"}
    assert all(path.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n" for path in report.plot_paths)
    payload = json.loads(report.log_path.read_text(encoding="utf-8"))
    assert payload["family"] == "rocket"
    assert "stage-separation" in {event["name"] for event in payload["events"]}
    assert payload["panels"][0]["rendered"] is True
####


def test_family_renderer_logs_missing_required_channels_without_plot(tmp_path) -> None:
    artifact = _artifact().model_copy(update={"vehicles": {"demo": _artifact().vehicles["demo"].model_copy(update={"channels": {}})}})
    reports = render_family_debug_artifacts(artifact, tmp_path, DebugFamily.ROCKET)

    payload = json.loads(reports[0].log_path.read_text(encoding="utf-8"))
    altitude = next(panel for panel in payload["panels"] if panel["panel_id"] == "altitude_range")
    assert altitude["renderable"] is False
    assert "position.altitude.geodetic" in altitude["missing_required"]
    assert not (tmp_path / "rocket" / "demo" / "altitude_range.png").exists()
####
