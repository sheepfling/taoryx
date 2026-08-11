"""Consumer-oriented authoring proofs for the analytical reference fixtures."""

from __future__ import annotations

import pytest

from taoryx import trajectory as trajectory_api
from taoryx.trajectory import (
    MissionCompositionOutputSelection,
    MissionCompositionRunRequest,
    MissionCompositionTrajectoryResult,
    PreparedTrajectoryConfiguration,
    ReferenceBallisticLaunch,
    ReferenceMissionCompositionProvider,
    ReferenceWaypoint,
    ReferenceWaypointCourseStart,
)


def _run(
    provider: ReferenceMissionCompositionProvider,
    prepared: PreparedTrajectoryConfiguration,
) -> MissionCompositionTrajectoryResult:
    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id="reference-authoring-proof",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="core", cadence_s=1.0),
        )
    )
    assert response.kind == "trajectory"
    return response.result
    ####


def test_ballistic_builder_normalizes_heading_and_preserves_coast_identity() -> None:
    """The common simple case needs no generic configuration-tree assembly."""

    assert trajectory_api.ReferenceBallisticLaunch is ReferenceBallisticLaunch
    provider = ReferenceMissionCompositionProvider()
    prepared = provider.prepare_ballistic(
        ReferenceBallisticLaunch(
            altitude_m=1000.0,
            speed_m_s=150.0,
            heading_deg=360.0,
            flight_path_angle_deg=20.0,
        ),
        coast_durations_s=(2.0, 3.0),
        configuration_id="friendly-ballistic",
    )

    assert prepared.configuration.model_id == "reference_ballistic_3dof"
    assert prepared.configuration.mission_template_id == "reference_ballistic_3dof_repeatable_sequence_v1"
    assert prepared.resolved["initialization"]["value"]["heading_deg"] == pytest.approx(0.0)
    assert [item["instance_id"] for item in prepared.resolved["segments"]] == ["coast-01", "coast-02"]

    result = _run(provider, prepared)
    assert result.objects[0].segments[-1].instance_id == "coast-02"
    assert result.objects[0].samples[-1].time_s == pytest.approx(5.0)
    ####


def test_waypoint_course_builder_derives_leg_times_and_runs_every_leg() -> None:
    """A multi-leg route retains caller identities and returns to its start."""

    assert trajectory_api.ReferenceWaypoint is ReferenceWaypoint
    provider = ReferenceMissionCompositionProvider()
    prepared = provider.prepare_waypoint_course(
        ReferenceWaypointCourseStart(
            altitude_m=1000.0,
            speed_m_s=100.0,
            heading_deg=-270.0,
        ),
        waypoints=(
            ReferenceWaypoint(north_m=0.0, east_m=500.0, altitude_m=1000.0, instance_id="outbound"),
            ReferenceWaypoint(north_m=0.0, east_m=0.0, altitude_m=1000.0, instance_id="return"),
        ),
        configuration_id="friendly-waypoint-course",
    )

    assert prepared.resolved["initialization"]["value"]["heading_deg"] == pytest.approx(90.0)
    assert [item["value"]["duration_s"] for item in prepared.resolved["segments"]] == pytest.approx([5.0, 5.0])

    result = _run(provider, prepared)
    primary = result.objects[0]
    assert [item.instance_id for item in primary.segments] == ["outbound", "return"]
    assert primary.samples[-1].values["position.north_m"] == pytest.approx(0.0)
    assert primary.samples[-1].values["position.east_m"] == pytest.approx(0.0)
    assert [item.segment_instance_id for item in result.events if item.kind == "waypoint_captured"] == ["outbound", "return"]
    ####


def test_waypoint_course_builder_rejects_an_empty_route() -> None:
    provider = ReferenceMissionCompositionProvider()

    with pytest.raises(ValueError, match="at least one waypoint"):
        provider.build_waypoint_course_configuration(
            ReferenceWaypointCourseStart(
                altitude_m=1000.0,
                speed_m_s=100.0,
                heading_deg=0.0,
            ),
            (),
        )
    ####


def test_waypoint_builder_accounts_for_an_explicit_short_preceding_leg() -> None:
    """Automatic timing continues from the planned endpoint of a timed leg."""

    provider = ReferenceMissionCompositionProvider()
    prepared = provider.prepare_waypoint_course(
        ReferenceWaypointCourseStart(
            altitude_m=1000.0,
            speed_m_s=100.0,
            heading_deg=0.0,
        ),
        waypoints=(
            ReferenceWaypoint(north_m=1000.0, east_m=0.0, altitude_m=1000.0, duration_s=2.0),
            ReferenceWaypoint(north_m=0.0, east_m=0.0, altitude_m=1000.0),
        ),
    )

    assert [item["value"]["duration_s"] for item in prepared.resolved["segments"]] == pytest.approx([2.0, 2.0])
    ####
