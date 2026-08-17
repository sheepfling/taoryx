"""Direct public-source unit ingestion and provenance tests."""

from __future__ import annotations

import math
from pathlib import Path

import pytest
from taoryx_parametric_interceptors import (
    EvidenceConfidence,
    ParametricInterceptorMissionCompositionProvider,
    canonical_unit_for_interceptor_parameter,
    canonicalize_interceptor_value,
    interceptor_from_catalogue_record,
    load_catalogue_interceptor_record,
    resolve_interceptor,
    supported_interceptor_units,
)

ROOT = Path(__file__).resolve().parents[3]
EXAMPLE = ROOT / "examples/parametric_interceptors/mixed_unit_authoring_catalogue_seed.yaml"


def test_mixed_source_units_resolve_to_canonical_values_with_originals_retained() -> None:
    profile = load_catalogue_interceptor_record(EXAMPLE)
    resolved = resolve_interceptor(profile)

    expected = {
        "launch_mass_kg": (356.0 * 0.45359237, "kg", 356, "lb"),
        "length_m": (12.0 * 0.3048, "m", 12, "ft"),
        "body_diameter_m": (7.0 * 0.0254, "m", 7, "in"),
        "wingspan_m": (19.0 * 0.0254, "m", 19, "in"),
        "propellant_fraction": (0.44, "1", 44, "%"),
        "burn_time_s": (8.0, "s", 8000, "ms"),
        "max_body_rate_rad_s": (math.pi, "rad/s", 180, "deg/s"),
        "max_body_acceleration_rad_s2": (5.0 * math.pi, "rad/s^2", 900, "deg/s^2"),
        "max_bank_angle_rad": (math.radians(75.0), "rad", 75, "deg"),
        "reported_max_speed_mps": (2500.0 / 3.6, "m/s", 2500, "km/h"),
        "reference_area_m2": (0.02483, "m^2", 248.3, "cm^2"),
    }
    for parameter_id, (value, unit, source_value, source_unit) in expected.items():
        parameter = resolved.parameters[parameter_id]
        assert parameter.value == pytest.approx(value)
        assert parameter.unit == unit
        assert parameter.source_value is not None
        assert parameter.source_value.value == source_value
        assert parameter.source_value.unit == source_unit
        assert "multiply" in parameter.method

    assert resolved.parameters["launch_mass_kg"].confidence is EvidenceConfidence.HIGH
    assert resolved.number("maneuver_drag_factor") == 0.12
    assert resolved.parameters["maneuver_drag_factor"].unit == "1"
    interval = resolved.evidence_intervals[0]
    assert interval.minimum_value == pytest.approx(348.0 * 0.45359237)
    assert interval.maximum_value == pytest.approx(358.0 * 0.45359237)
    assert interval.unit == "kg"
    assert interval.source_minimum_value == 348
    assert interval.source_maximum_value == 358
    assert interval.source_unit == "lb"
    assert "Canonical interval converted" in interval.note
    ####


def test_composition_advertises_canonical_values_and_retained_source_units() -> None:
    provider = ParametricInterceptorMissionCompositionProvider.from_catalogue_yaml(EXAMPLE)
    properties = {item.id: item for item in provider.model("example-mixed-unit-sam").presentation.properties}

    assert properties["launch_mass_kg"].canonical_unit == "kg"
    assert properties["launch_mass_kg"].value == pytest.approx(356.0 * 0.45359237)
    assert "source_value=356 lb" in properties["launch_mass_kg"].provenance
    assert "source=[348, 358] lb" in str(properties["evidence_intervals"].value)
    ####


def test_public_unit_inspection_api_is_small_and_deterministic() -> None:
    conversion = canonicalize_interceptor_value("reported_max_range_m", 25, "nmi")

    assert conversion.canonical_value == 46_300.0
    assert conversion.canonical_unit == "m"
    assert conversion.source_value == 25.0
    assert conversion.source_unit == "nmi"
    assert conversion.converted is True
    assert canonical_unit_for_interceptor_parameter("launch_mass_kg") == "kg"
    assert supported_interceptor_units("launch_mass_kg") == ("kg", "g", "lb", "oz")
    ####


def test_incompatible_or_ambiguous_source_units_fail_before_resolution() -> None:
    with pytest.raises(ValueError, match="requires a unit compatible with 'kg'"):
        interceptor_from_catalogue_record(
            {
                "interceptor_id": "int:invalid-mass-unit",
                "evidence": {
                    "launch_mass": {"value": 100, "unit": "m", "origin": "observed"},
                },
            }
        )

    with pytest.raises(ValueError, match="supply the source scalar once"):
        interceptor_from_catalogue_record(
            {
                "interceptor_id": "int:ambiguous-mass-source",
                "evidence": {
                    "launch_mass": {
                        "value": 356,
                        "unit": "lb",
                        "origin": "observed_converted",
                        "source_value": {"value": 356, "unit": "lb"},
                    },
                },
            }
        )
    ####
