from __future__ import annotations

from taoryx.family_debugging import DebugFamily, build_family_debug_plan, family_profile
from taoryx.outputs import DynamicsKind, EventRecord, RunArtifact, TelemetryChannel, VehicleKind, VehicleTelemetry


def _artifact() -> RunArtifact:
    channels = {
        "position.altitude.geodetic": TelemetryChannel(source_name="alt", semantic_name="position.altitude.geodetic", values=[0.0, 100.0]),
        "mass.total": TelemetryChannel(source_name="wt", semantic_name="mass.total", values=[100.0, 90.0]),
        "propulsion.thrust": TelemetryChannel(source_name="thrust", semantic_name="propulsion.thrust", values=[10.0, 0.0]),
    }
    vehicle = VehicleTelemetry(
        vehicle_id="1", name="demo", kind=VehicleKind.ROCKET, dynamics=DynamicsKind.POINT_MASS_3DOF,
        times=[0.0, 1.0], channels=channels,
        events=[EventRecord(time=1.0, vehicle="1", name="stage-separation", kind="stage_separation")],
    )
    return RunArtifact(problem="demo.prb", vehicles={"1": vehicle})


def test_family_profile_resolves_required_and_optional_channels() -> None:
    plan = build_family_debug_plan(_artifact(), DebugFamily.ROCKET)

    mass_panel = next(panel for panel in plan.panels if panel.spec.panel_id == "mass_propulsion")
    assert mass_panel.renderable
    assert mass_panel.available_channels == ("mass.total", "propulsion.thrust")
    assert "mass.flow" in mass_panel.missing_optional
    assert plan.events[0].kind == "stage_separation"
    ####


def test_orbital_and_quadcopter_profiles_are_distinct() -> None:
    assert {panel.panel_id for panel in family_profile(DebugFamily.ORBITAL)} != {panel.panel_id for panel in family_profile(DebugFamily.QUADCOPTER)}
    assert "orbital_elements" in {panel.panel_id for panel in family_profile("orbital")}
    assert "controls" in {panel.panel_id for panel in family_profile("quadcopter")}
    ####
