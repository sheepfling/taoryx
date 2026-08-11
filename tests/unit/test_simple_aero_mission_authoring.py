"""Consumer-oriented authoring proofs for the ergonomic Simple Aero surface."""

from __future__ import annotations

import math

import pytest
from taoryx.trajectory.registry_mission_composition import RegistryMissionCompositionProvider
from taoryx.trajectory.simple_aero_mission_composition import build_simple_aero_prepared_configuration

from taoryx import trajectory as trajectory_api
from taoryx.trajectory import (
    MissionCompositionOutputSelection,
    MissionCompositionRunRequest,
    SimpleAeroCheckpoints,
    SimpleAeroEndpointState,
    SimpleAeroGeodeticAimpoint,
    SimpleAeroLaunch,
    SimpleAeroMission,
    SimpleAeroRuntime,
    SimpleAeroSegment,
    SimpleAeroSurrogate,
    build_registry_mission_composition_runner,
    prepare_simple_aero_mission,
)


def _all_features_mission() -> SimpleAeroMission:
    """Return a compact custom sequence covering every published source segment."""

    return SimpleAeroMission(
        configuration_id="simple-aero-all-feature-authoring",
        launch=SimpleAeroLaunch(
            latitude_deg=35.8766,
            longitude_deg=540.0,
            altitude_m=500.0,
            speed_m_s=75.0,
            pitch_over_angle_deg=20.0,
            initial_heading_offset_deg=10.0,
        ),
        endpoint=SimpleAeroGeodeticAimpoint(latitude_deg=36.000975, longitude_deg=181.0),
        endpoint_state=SimpleAeroEndpointState(altitude_m=200.0, speed_m_s=50.0, heading_deg=540.0),
        surrogate=SimpleAeroSurrogate(
            vehicle_id="ergonomic-simple-aero",
            initial_mass_kg=1200.0,
            thrust_n=90_000.0,
            mass_flow_kg_s=15.0,
            drag_coefficient=0.03,
            lift_to_drag=3.5,
        ),
        checkpoints=SimpleAeroCheckpoints(burnout_speed_m_s=900.0, apogee_altitude_m=20_000.0),
        runtime=SimpleAeroRuntime(time_step_s=0.05, output_interval_s=0.1, earth_model="vacuum_spherical"),
        segments=(
            SimpleAeroSegment.powered_ascent(duration_s=0.2, cutoff_condition="commanded_burnout_speed"),
            SimpleAeroSegment.ballistic_coast(duration_s=0.2, alpha_deg=2.0),
            SimpleAeroSegment.bank_maneuver(duration_s=0.2, bank_deg=-12.0),
            SimpleAeroSegment.cbcr(
                go_left=False,
                maneuver_altitude_start_m=10_000.0,
                duration_s=0.2,
                minimum_time_to_go_s=0.2,
            ),
            SimpleAeroSegment.crossrange(initial_heading_error_deg=8.0, minimum_time_to_go_s=0.2),
            SimpleAeroSegment.marv(maneuver_begin_time_to_go_s=0.2, minimum_time_to_go_s=0.2),
            SimpleAeroSegment.phugoid(
                start_range_to_go_m=100.0,
                amplitude_deg=1.0,
                frequency_hz=2.0,
                maneuver_roll_deg=20.0,
            ),
            SimpleAeroSegment.range_extension(minimum_time_to_go_s=0.2),
            SimpleAeroSegment.skip(maneuver_begin_time_to_go_s=0.2),
            SimpleAeroSegment.slalom(
                start_range_to_go_m=100.0,
                end_range_to_go_m=50.0,
                minimum_time_to_go_s=0.2,
            ),
            SimpleAeroSegment.weave(end_range_to_go_m=25.0, minimum_time_to_go_s=0.2),
            SimpleAeroSegment.terminal_pronav(duration_s=0.2, capture_range_m=5.0),
        ),
    )
    ####


def test_simple_aero_named_authoring_maps_every_feature_to_the_published_tree() -> None:
    """Typed convenience objects retain all schema choices, optionals, and occurrences."""

    assert trajectory_api.SimpleAeroMission is SimpleAeroMission
    assert trajectory_api.SimpleAeroSegment is SimpleAeroSegment
    mission = _all_features_mission()
    prepared = prepare_simple_aero_mission(mission)
    source_ids = [item["selected"] for item in prepared.resolved["segments"]]

    assert isinstance(mission.endpoint, SimpleAeroGeodeticAimpoint)
    assert mission.launch.longitude_deg == pytest.approx(-180.0)
    assert mission.endpoint.longitude_deg == pytest.approx(-179.0)
    assert mission.endpoint_state.heading_deg == pytest.approx(-180.0)
    assert prepared.configuration.mission_template_id == "custom_composition"
    assert source_ids == [
        "powered_ascent",
        "ballistic_coast",
        "bank_maneuver",
        "cbcr",
        "crossrange",
        "marv",
        "phugoid",
        "range_extension",
        "skip",
        "slalom",
        "weave",
        "terminal_pronav",
    ]
    assert [item["instance_id"] for item in prepared.resolved["segments"]] == [
        f"{identifier}-{index:02d}" for index, identifier in enumerate(source_ids, start=1)
    ]
    assert prepared.resolved["segments"][6]["value"]["start_range_to_go_m"] == pytest.approx(100.0)
    assert prepared.resolved["segments"][9]["value"]["start_range_to_go_m"] == pytest.approx(100.0)
    assert prepared.resolved["segments"][10]["value"]["end_range_to_go_m"] == pytest.approx(25.0)

    build = build_simple_aero_prepared_configuration(prepared)
    assert build.derived.total_duration_s > 0.0
    assert all(f"source Simple Aero segment {identifier}" in build.problem_text for identifier in source_ids)
    ####


def test_simple_aero_template_authoring_materializes_every_advertised_sequence() -> None:
    """Published templates are available without raw configuration-tree construction."""

    provider = RegistryMissionCompositionProvider()
    template_ids = tuple(item.id for item in provider.model("simple_aero").mission_templates if item.id != "custom_composition")

    for template_id in template_ids:
        mission = SimpleAeroMission.template(template_id, configuration_id=f"friendly-{template_id}")
        prepared = prepare_simple_aero_mission(mission, schema=provider.get_model_schema("simple_aero"))

        assert prepared.configuration.mission_template_id == template_id
        assert [item["selected"] for item in prepared.resolved["segments"]]
        assert build_simple_aero_prepared_configuration(prepared).derived.total_duration_s > 0.0
    ####


def test_simple_aero_all_feature_sequence_executes_through_the_registered_batch_binding() -> None:
    """Custom occurrence order remains visible to the executable fixed-L/D lowering."""

    provider = RegistryMissionCompositionProvider()
    prepared = provider.validate_configuration(prepare_simple_aero_mission(_all_features_mission()).configuration)
    response = build_registry_mission_composition_runner(provider).run(
        MissionCompositionRunRequest(
            request_id="simple-aero-all-feature-authoring",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=7),
        )
    )

    assert response.kind == "trajectory", response.model_dump(mode="json", by_alias=True)
    assert response.result.status == "completed"
    primary = response.result.objects[0]
    assert len(primary.samples) == 7
    assert all(math.isfinite(float(sample.values["velocity.speed"])) for sample in primary.samples)
    ####


def test_simple_aero_mission_rejects_duplicate_explicit_segment_identity() -> None:
    """Repeated source variants remain legal but their supplied identities must be unique."""

    with pytest.raises(ValueError, match="instance_id values must be unique"):
        SimpleAeroMission(
            segments=(
                SimpleAeroSegment.ballistic_coast(instance_id="coast"),
                SimpleAeroSegment.ballistic_coast(instance_id="coast"),
            )
        )
    ####
