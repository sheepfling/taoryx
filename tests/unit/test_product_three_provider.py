from __future__ import annotations

import json
from pathlib import Path

import pytest

from taoryx.trajectory import (
    ExampleProductThreeProvider,
    ProductThreeError,
    ProductThreeOutputRequest,
    ProductThreeParameterValue,
    ProductThreeProviderRegistry,
    ProductThreeSegmentRequest,
    ProductThreeTrajectoryRequest,
)


def _guided_request(*, segments: tuple[ProductThreeSegmentRequest, ...] | None = None) -> ProductThreeTrajectoryRequest:
    """Build the public request used by the Product 3 contract tests."""

    return ProductThreeTrajectoryRequest(
        request_id="guided-demo",
        vehicle_id="reference_guided_point_mass",
        fidelity="point_mass_3dof",
        initialization_id="airborne_state",
        initialization={
            "altitude_m": ProductThreeParameterValue(value=1000.0, unit="m"),
            "speed_m_s": ProductThreeParameterValue(value=100.0, unit="m/s"),
            "heading_deg": ProductThreeParameterValue(value=90.0, unit="deg"),
        },
        segments=segments
        or (
            ProductThreeSegmentRequest(
                id="waypoint_leg",
                parameters={
                    "duration_s": ProductThreeParameterValue(value=5.0, unit="s"),
                    "waypoint_north_m": ProductThreeParameterValue(value=0.0, unit="m"),
                    "waypoint_east_m": ProductThreeParameterValue(value=500.0, unit="m"),
                    "waypoint_altitude_m": ProductThreeParameterValue(value=1000.0, unit="m"),
                    "max_load_factor_g": ProductThreeParameterValue(value=3.0, unit="g0"),
                },
            ),
            ProductThreeSegmentRequest(
                id="bank_maneuver",
                parameters={
                    "duration_s": ProductThreeParameterValue(value=2.0, unit="s"),
                    "target_heading_deg": ProductThreeParameterValue(value=180.0, unit="deg"),
                },
            ),
        ),
        output=ProductThreeOutputRequest(cadence_s=0.5),
    )
    ####


def test_product_three_discovery_publishes_vehicle_capabilities_and_parameter_metadata() -> None:
    provider = ExampleProductThreeProvider()
    registry = ProductThreeProviderRegistry((provider,))

    catalog = registry.catalog()
    assert catalog["schema"] == "taoryx.product-three-provider-catalog/v1"
    publication = catalog["providers"][0]
    assert publication["provider_id"] == "taoryx.example.product3"
    assert {item["vehicle_id"] for item in publication["vehicles"]} == {
        "reference_ballistic",
        "reference_guided_point_mass",
    }
    guided = provider.metadata.vehicle("reference_guided_point_mass")
    waypoint = guided.segment("waypoint_leg")
    max_g = next(item for item in waypoint.parameters if item.id == "max_load_factor_g")
    assert max_g.canonical_unit == "g0"
    assert max_g.minimum == pytest.approx(1.0)
    assert max_g.maximum == pytest.approx(6.0)
    assert max_g.qualified_maximum == pytest.approx(4.0)
    assert waypoint.allowed_next == ("waypoint_leg", "bank_maneuver", "coast")
    assert waypoint.repeatable
    ####


def test_product_three_prepare_resolves_defaults_and_assigns_stable_instances() -> None:
    provider = ExampleProductThreeProvider()
    prepared = provider.prepare(_guided_request())

    assert prepared.segments[0].instance_id == "01-waypoint_leg"
    assert prepared.segments[1].instance_id == "02-bank_maneuver"
    assert prepared.segments[0].parameters["max_load_factor_g"] == pytest.approx(3.0)
    assert prepared.segments[0].parameters["arrival_tolerance_m"] == pytest.approx(25.0)
    assert len(prepared.request_fingerprint) == 64
    assert prepared.request_fingerprint == provider.prepare(_guided_request()).request_fingerprint
    ####


def test_product_three_run_returns_standard_trajectory_with_segment_spans_and_events(tmp_path: Path) -> None:
    provider = ExampleProductThreeProvider()
    prepared = provider.prepare(_guided_request())
    result = provider.run(prepared)

    assert result.schema_id == "taoryx.product-three-trajectory/v1"
    assert result.status == "completed"
    assert result.vehicle_id == "reference_guided_point_mass"
    assert len(result.samples) > 1
    assert result.samples[0].time_s == pytest.approx(0.0)
    assert result.samples[-1].time_s == pytest.approx(7.0)
    assert [item.id for item in result.segments] == ["waypoint_leg", "bank_maneuver"]
    assert result.segments[0].end_time_s == pytest.approx(5.0)
    assert result.segments[1].start_time_s == pytest.approx(5.0)
    assert result.channel_units["maneuver.load_factor_g"] == "g0"
    assert max(sample.values["maneuver.load_factor_g"] for sample in result.samples) <= 3.0
    destination = tmp_path / "trajectory.json"
    result.write_json(destination)
    payload = json.loads(destination.read_text(encoding="utf-8"))
    assert payload["schema"] == "taoryx.product-three-trajectory/v1"
    assert payload["request_fingerprint"] == prepared.request_fingerprint
    ####


def test_product_three_ballistic_provider_supports_repeatable_coast_segments() -> None:
    provider = ExampleProductThreeProvider()
    request = ProductThreeTrajectoryRequest(
        request_id="ballistic-demo",
        vehicle_id="reference_ballistic",
        fidelity="point_mass_3dof",
        initialization_id="launch_state",
        initialization={
            "altitude_m": ProductThreeParameterValue(value=1000.0, unit="m"),
            "speed_m_s": ProductThreeParameterValue(value=150.0, unit="m/s"),
            "heading_deg": ProductThreeParameterValue(value=0.0, unit="deg"),
        },
        segments=(
            ProductThreeSegmentRequest(
                id="ballistic_coast",
                parameters={"duration_s": ProductThreeParameterValue(value=1.0, unit="s")},
            ),
            ProductThreeSegmentRequest(
                id="ballistic_coast",
                parameters={"duration_s": ProductThreeParameterValue(value=1.0, unit="s")},
            ),
        ),
    )

    result = provider.run(provider.prepare(request))
    assert result.status == "completed"
    assert len(result.segments) == 2
    assert result.samples[-1].values["position.north_m"] > 0.0
    assert all(sample.values["maneuver.load_factor_g"] == pytest.approx(1.0) for sample in result.samples)
    ####


@pytest.mark.parametrize(
    ("request_change", "code"),
    [
        (
            {"initialization": {"altitude_m": ProductThreeParameterValue(value=1000.0, unit="kg")}},
            "unit-mismatch",
        ),
        (
            {"initialization": {"altitude_m": ProductThreeParameterValue(value=1000.0, unit="m"), "speed_m_s": ProductThreeParameterValue(value=100.0, unit="m/s"), "heading_deg": ProductThreeParameterValue(value=90.0, unit="deg"), "unknown": ProductThreeParameterValue(value=1.0)}},
            "unknown-parameter",
        ),
    ],
)
def test_product_three_prepare_fails_closed_for_invalid_parameter_input(
    request_change: dict[str, object], code: str
) -> None:
    provider = ExampleProductThreeProvider()
    base = _guided_request()
    request = base.model_copy(update=request_change)
    with pytest.raises(ProductThreeError, match=code):
        provider.prepare(request)
    ####


def test_product_three_prepare_rejects_invalid_segment_order() -> None:
    provider = ExampleProductThreeProvider()
    request = _guided_request(
        segments=(
            ProductThreeSegmentRequest(
                id="bank_maneuver",
                parameters={
                    "duration_s": ProductThreeParameterValue(value=1.0, unit="s"),
                    "target_heading_deg": ProductThreeParameterValue(value=180.0, unit="deg"),
                },
            ),
            ProductThreeSegmentRequest(
                id="waypoint_leg",
                parameters={
                    "duration_s": ProductThreeParameterValue(value=1.0, unit="s"),
                    "waypoint_north_m": ProductThreeParameterValue(value=0.0, unit="m"),
                    "waypoint_east_m": ProductThreeParameterValue(value=100.0, unit="m"),
                    "waypoint_altitude_m": ProductThreeParameterValue(value=1000.0, unit="m"),
                },
            ),
        )
    )
    with pytest.raises(ProductThreeError, match="invalid-sequence"):
        provider.prepare(request)
    ####


def test_product_three_prepare_enforces_requested_output_sample_limit() -> None:
    provider = ExampleProductThreeProvider()
    request = _guided_request().model_copy(update={"output": ProductThreeOutputRequest(cadence_s=0.1, max_samples=10)})

    with pytest.raises(ProductThreeError, match="output-sample-limit"):
        provider.prepare(request)
    ####
