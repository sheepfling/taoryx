from __future__ import annotations

import json
from pathlib import Path

import pytest

from taoryx import trajectory as trajectory_api
from taoryx.trajectory import (
    ExampleMissionCompositionProvider,
    MissionCompositionError,
    MissionCompositionOutputRequest,
    MissionCompositionParameterValue,
    MissionCompositionProviderRegistry,
    MissionCompositionSegmentRequest,
    MissionCompositionTrajectoryRequest,
)


def _waypoint_request(*, segments: tuple[MissionCompositionSegmentRequest, ...] | None = None) -> MissionCompositionTrajectoryRequest:
    """Build the public request used by the Mission Composition contract tests."""

    return MissionCompositionTrajectoryRequest(
        request_id="waypoint-demo",
        vehicle_id="reference_constant_velocity_waypoint_3dof",
        fidelity="point_mass_3dof",
        initialization_id="initial_state",
        initialization={
            "altitude_m": MissionCompositionParameterValue(value=1000.0, unit="m"),
            "speed_m_s": MissionCompositionParameterValue(value=100.0, unit="m/s"),
            "heading_deg": MissionCompositionParameterValue(value=90.0, unit="deg"),
        },
        segments=segments
        or (
            MissionCompositionSegmentRequest(
                id="waypoint_leg",
                parameters={
                    "duration_s": MissionCompositionParameterValue(value=5.0, unit="s"),
                    "waypoint_north_m": MissionCompositionParameterValue(value=0.0, unit="m"),
                    "waypoint_east_m": MissionCompositionParameterValue(value=500.0, unit="m"),
                    "waypoint_altitude_m": MissionCompositionParameterValue(value=1000.0, unit="m"),
                },
            ),
            MissionCompositionSegmentRequest(
                id="waypoint_leg",
                parameters={
                    "duration_s": MissionCompositionParameterValue(value=5.0, unit="s"),
                    "waypoint_north_m": MissionCompositionParameterValue(value=0.0, unit="m"),
                    "waypoint_east_m": MissionCompositionParameterValue(value=0.0, unit="m"),
                    "waypoint_altitude_m": MissionCompositionParameterValue(value=1000.0, unit="m"),
                },
            ),
        ),
        output=MissionCompositionOutputRequest(cadence_s=0.5),
    )
    ####


def test_mission_composition_names_are_exported_from_the_canonical_surface() -> None:
    assert trajectory_api.ExampleMissionCompositionProvider is ExampleMissionCompositionProvider
    assert trajectory_api.MissionCompositionProviderRegistry is MissionCompositionProviderRegistry
    assert trajectory_api.MissionCompositionTrajectoryRequest is MissionCompositionTrajectoryRequest

    provider = ExampleMissionCompositionProvider()
    registry = MissionCompositionProviderRegistry((provider,))
    assert registry.provider(provider.metadata.provider_id) is provider
    ####


def test_discovery_publishes_vehicle_capabilities_and_parameter_metadata() -> None:
    provider = ExampleMissionCompositionProvider()
    registry = MissionCompositionProviderRegistry((provider,))

    catalog = registry.catalog()
    assert catalog["schema"] == "taoryx.mission-composition-provider-catalog/v1"
    publication = catalog["providers"][0]
    assert publication["provider_id"] == "taoryx.example.mission-composition"
    assert {item["vehicle_id"] for item in publication["vehicles"]} == {
        "reference_ballistic_3dof",
        "reference_constant_velocity_waypoint_3dof",
    }
    waypoint_model = provider.metadata.vehicle("reference_constant_velocity_waypoint_3dof")
    waypoint = waypoint_model.segment("waypoint_leg")
    arrival = next(item for item in waypoint.parameters if item.id == "arrival_tolerance_m")
    assert arrival.canonical_unit == "m"
    assert arrival.minimum == pytest.approx(0.1)
    assert arrival.default == pytest.approx(25.0)
    ballistic_model = provider.metadata.vehicle("reference_ballistic_3dof")
    assert ballistic_model.execution_mode == "native"
    assert ballistic_model.model_kind == "ballistic_3dof"
    assert waypoint_model.execution_mode == "native"
    assert waypoint_model.fidelities == ("point_mass_3dof",)
    assert waypoint.allowed_next == ("waypoint_leg",)
    assert waypoint.repeatable
    ####


def test_prepare_resolves_defaults_and_assigns_stable_instances() -> None:
    provider = ExampleMissionCompositionProvider()
    prepared = provider.prepare(_waypoint_request())

    assert prepared.segments[0].instance_id == "01-waypoint_leg"
    assert prepared.segments[1].instance_id == "02-waypoint_leg"
    assert prepared.segments[0].parameters["arrival_tolerance_m"] == pytest.approx(25.0)
    assert len(prepared.request_fingerprint) == 64
    assert prepared.request_fingerprint == provider.prepare(_waypoint_request()).request_fingerprint
    ####


def test_run_returns_standard_trajectory_with_segment_spans_and_events(tmp_path: Path) -> None:
    provider = ExampleMissionCompositionProvider()
    prepared = provider.prepare(_waypoint_request())
    result = provider.run(prepared)

    assert result.schema_id == "taoryx.mission-composition-trajectory/v1"
    assert result.status == "completed"
    assert result.vehicle_id == "reference_constant_velocity_waypoint_3dof"
    assert len(result.samples) > 1
    assert result.samples[0].time_s == pytest.approx(0.0)
    assert result.samples[-1].time_s == pytest.approx(10.0)
    assert [item.id for item in result.segments] == ["waypoint_leg", "waypoint_leg"]
    assert result.segments[0].end_time_s == pytest.approx(5.0)
    assert result.segments[1].start_time_s == pytest.approx(5.0)
    assert result.channel_units["maneuver.load_factor_g"] == "g0"
    assert all(sample.values["maneuver.load_factor_g"] == pytest.approx(1.0) for sample in result.samples)
    assert result.samples[-1].values["position.east_m"] == pytest.approx(0.0)
    assert all(sample.standard_ecef.frame_id == "ecfc" for sample in result.samples)
    assert all(
        len(sample.standard_ecef.position_ecef_m) == 3
        and len(sample.standard_ecef.velocity_ecef_mps) == 3
        and len(sample.standard_ecef.acceleration_ecef_mps2) == 3
        and len(sample.standard_ecef.angular_velocity_body_radps) == 3
        and len(sample.standard_ecef.ecef_from_body_wxyz) == 4
        for sample in result.samples
    )
    assert [item.segment_instance_id for item in result.events if item.kind == "waypoint_captured"] == [
        "01-waypoint_leg",
        "02-waypoint_leg",
    ]
    destination = tmp_path / "trajectory.json"
    result.write_json(destination)
    payload = json.loads(destination.read_text(encoding="utf-8"))
    assert payload["schema"] == "taoryx.mission-composition-trajectory/v1"
    assert payload["request_fingerprint"] == prepared.request_fingerprint
    ####


def test_ballistic_model_supports_repeatable_coast_segments() -> None:
    provider = ExampleMissionCompositionProvider()
    request = MissionCompositionTrajectoryRequest(
        request_id="ballistic-demo",
        vehicle_id="reference_ballistic_3dof",
        fidelity="point_mass_3dof",
        initialization_id="launch_state",
        initialization={
            "altitude_m": MissionCompositionParameterValue(value=1000.0, unit="m"),
            "speed_m_s": MissionCompositionParameterValue(value=150.0, unit="m/s"),
            "heading_deg": MissionCompositionParameterValue(value=0.0, unit="deg"),
        },
        segments=(
            MissionCompositionSegmentRequest(
                id="ballistic_coast",
                parameters={"duration_s": MissionCompositionParameterValue(value=1.0, unit="s")},
            ),
            MissionCompositionSegmentRequest(
                id="ballistic_coast",
                parameters={"duration_s": MissionCompositionParameterValue(value=1.0, unit="s")},
            ),
        ),
    )

    result = provider.run(provider.prepare(request))
    assert result.status == "completed"
    assert len(result.segments) == 2
    assert result.samples[-1].values["position.north_m"] > 0.0
    assert all(sample.values["velocity.north_m_s"] >= 0.0 for sample in result.samples)
    ####


@pytest.mark.parametrize(
    ("request_change", "code"),
    [
        (
            {"initialization": {"altitude_m": MissionCompositionParameterValue(value=1000.0, unit="kg")}},
            "unit-mismatch",
        ),
        (
            {"initialization": {"altitude_m": MissionCompositionParameterValue(value=1000.0, unit="m"), "speed_m_s": MissionCompositionParameterValue(value=100.0, unit="m/s"), "heading_deg": MissionCompositionParameterValue(value=90.0, unit="deg"), "unknown": MissionCompositionParameterValue(value=1.0)}},
            "unknown-parameter",
        ),
    ],
)
def test_prepare_fails_closed_for_invalid_parameter_input(
    request_change: dict[str, object], code: str
) -> None:
    provider = ExampleMissionCompositionProvider()
    base = _waypoint_request()
    request = base.model_copy(update=request_change)
    with pytest.raises(MissionCompositionError, match=code):
        provider.prepare(request)
    ####


def test_prepare_rejects_unknown_segment() -> None:
    provider = ExampleMissionCompositionProvider()
    request = _waypoint_request(
        segments=(
            MissionCompositionSegmentRequest(
                id="not_advertised",
                parameters={
                    "duration_s": MissionCompositionParameterValue(value=1.0, unit="s"),
                },
            ),
        )
    )
    with pytest.raises(MissionCompositionError, match="unknown-segment"):
        provider.prepare(request)
    ####


def test_prepare_enforces_requested_output_sample_limit() -> None:
    provider = ExampleMissionCompositionProvider()
    request = _waypoint_request().model_copy(update={"output": MissionCompositionOutputRequest(cadence_s=0.1, max_samples=10)})

    with pytest.raises(MissionCompositionError, match="output-sample-limit"):
        provider.prepare(request)
    ####
