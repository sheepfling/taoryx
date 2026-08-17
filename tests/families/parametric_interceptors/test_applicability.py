"""Focused evidence-to-runtime tests for advisory interceptor applicability."""

from __future__ import annotations

import pytest
from pydantic import ValidationError
from taoryx_parametric_interceptors import (
    APPLICABILITY_CONTRACT,
    APPLICABILITY_ENFORCEMENT,
    InterceptorApplicabilityEnvelope,
    ParametricInterceptorMissionCompositionProvider,
    PointMassKernel,
    PointMassState,
    PointMassWaypoint,
    Pseudo6Kernel,
    Pseudo6State,
    ValueOrigin,
    interceptor,
    interceptor_from_catalogue_record,
    resolve_interceptor,
)

from taoryx.contracts import Frame, FrameVector3, Vector3
from taoryx.runtime.environment_runtime import EnvironmentSample, StaticEnvironmentProvider
from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunRequest,
    MissionCompositionTrajectoryResponse,
)


def _environment() -> StaticEnvironmentProvider:
    return StaticEnvironmentProvider(
        EnvironmentSample(
            density=1.0,
            pressure=100_000.0,
            temperature=250.0,
            speed_of_sound=100.0,
            wind=FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
        )
    )
    ####


def _profile():
    return resolve_interceptor(
        interceptor(
            "applicability-witness",
            applicability_altitude_min_m=500.0,
            applicability_altitude_max_m=2_000.0,
            applicability_mach_min=2.0,
            applicability_mach_max=4.0,
        )
    )
    ####


def test_optional_envelope_distinguishes_undeclared_inside_and_outside_states() -> None:
    undeclared = InterceptorApplicabilityEnvelope().evaluate(altitude_m=100.0, mach=1.0)
    assert not undeclared.declared
    assert undeclared.status == "not_declared"
    assert undeclared.reason == "not_declared"

    envelope = InterceptorApplicabilityEnvelope(
        altitude_min_m=500.0,
        altitude_max_m=2_000.0,
        mach_min=2.0,
        mach_max=4.0,
    )
    assert envelope.declared_bounds == (
        "altitude_min_m",
        "altitude_max_m",
        "mach_min",
        "mach_max",
    )
    inside = envelope.evaluate(altitude_m=1_000.0, mach=3.0)
    assert inside.status == "within_declared_envelope"
    assert inside.reason == "none"

    outside = envelope.evaluate(altitude_m=100.0, mach=5.0)
    assert outside.status == "outside_declared_envelope"
    assert outside.reason == "below_declared_altitude+above_declared_mach"
    ####


def test_profile_ordering_and_catalogue_operating_aliases_are_strict_and_evidence_aware() -> None:
    with pytest.raises(ValidationError, match="altitude minimum must be below"):
        interceptor(
            "invalid-altitude-envelope",
            applicability_altitude_min_m=2_000.0,
            applicability_altitude_max_m=500.0,
        )
    with pytest.raises(ValidationError, match="Mach minimum must be below"):
        interceptor(
            "invalid-mach-envelope",
            applicability_mach_min=4.0,
            applicability_mach_max=2.0,
        )

    profile = interceptor_from_catalogue_record(
        {
            "kind": "interceptor_evidence_record",
            "interceptor_id": "int:test-applicability",
            "model_id": "catalogue-applicability",
            "source_record_ids": ["source:public-operating-domain"],
            "resolution_status": "mixed",
            "evidence": {
                "operating_altitude_min": {"value": 1_000.0, "unit": "ft", "origin": "observed"},
                "operating_altitude_max": {"value": 60_000.0, "unit": "ft", "origin": "observed"},
                "operating_mach_min": {"value": 0.2, "unit": "1", "origin": "observed"},
                "operating_mach_max": {"value": 5.0, "unit": "1", "origin": "observed"},
            },
            "resolver_assumptions": {
                "surrogate_archetype_id": "generic_slender_sam_v1",
            },
        }
    )
    resolved = resolve_interceptor(profile)
    lower = resolved.parameters["applicability_altitude_min_m"]
    assert lower.value == pytest.approx(304.8)
    assert lower.origin is ValueOrigin.OBSERVED
    assert lower.source_value is not None
    assert (lower.source_value.value, lower.source_value.unit) == (1_000.0, "ft")
    assert lower.source_record_ids == ("source:public-operating-domain",)
    assert resolved.number("applicability_altitude_max_m") == pytest.approx(18_288.0)
    assert resolved.number("applicability_mach_min") == 0.2
    assert resolved.number("applicability_mach_max") == 5.0
    ####


@pytest.mark.parametrize(
    ("altitude_m", "speed_mps", "expected_status", "expected_reason"),
    (
        (100.0, 150.0, "outside_declared_envelope", "below_declared_altitude+below_declared_mach"),
        (1_000.0, 300.0, "within_declared_envelope", "none"),
    ),
)
def test_both_fidelity_tiers_share_advisory_applicability(
    altitude_m: float,
    speed_mps: float,
    expected_status: str,
    expected_reason: str,
) -> None:
    profile = _profile()
    environment = _environment()
    waypoint = PointMassWaypoint(
        north_m=10_000.0,
        east_m=0.0,
        altitude_m=altitude_m,
        capture_radius_m=10.0,
    )
    point = (
        PointMassKernel(profile, environment=environment, gravity_acceleration=lambda _: 0.0)
        .evaluate(
            PointMassState(
                time_s=0.0,
                north_m=0.0,
                east_m=0.0,
                altitude_m=altitude_m,
                north_velocity_mps=speed_mps,
                east_velocity_mps=0.0,
                vertical_velocity_mps=0.0,
            ),
            waypoint,
        )
        .sample
    )
    pseudo = Pseudo6Kernel(profile, environment=environment, gravity_acceleration=lambda _: 0.0).evaluate(
        Pseudo6State(
            time_s=0.0,
            north_m=0.0,
            east_m=0.0,
            altitude_m=altitude_m,
            north_velocity_mps=speed_mps,
            east_velocity_mps=0.0,
            vertical_velocity_mps=0.0,
            roll_rad=0.0,
            pitch_rad=0.0,
            yaw_rad=0.0,
            roll_rate_rad_s=0.0,
            pitch_rate_rad_s=0.0,
            yaw_rate_rad_s=0.0,
        ),
        waypoint,
    )

    assert point.applicability_declared
    assert point.applicability_status == pseudo.applicability_status == expected_status
    assert point.applicability_reason == pseudo.applicability_reason == expected_reason
    assert point.operational
    assert pseudo.operational
    ####


def test_composition_advertises_and_reports_advisory_applicability_without_gating() -> None:
    profile = interceptor(
        "composition-applicability",
        applicability_altitude_min_m=500.0,
        applicability_altitude_max_m=2_000.0,
        applicability_mach_max=0.01,
    )
    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(profile)
    model = provider.model(profile.model_id)
    properties = {item.id: item for item in model.presentation.properties}
    outputs = {item.id: item for item in model.output_schema.telemetry_channels}
    groups = {item.id: item for item in model.output_schema.telemetry_groups}

    assert properties["applicability.contract"].value == APPLICABILITY_CONTRACT
    assert properties["applicability.enforcement"].value == APPLICABILITY_ENFORCEMENT == "advisory"
    assert properties["applicability.declared"].value is True
    assert "altitude_min_m=500.0" in str(properties["applicability.bounds"].value)
    assert outputs["model.applicability.declared"].data_type == "boolean"
    assert "does not silently terminate" in outputs["model.applicability.status"].description
    assert groups["applicability"].default_selected

    prepared = provider.validate_configuration(
        provider.configuration(
            profile.model_id,
            configuration_id="composition-applicability",
            launch_altitude_m=100.0,
            launch_speed_mps=100.0,
            launch_flight_path_deg=0.0,
            navigation_waypoint_north_command=10_000.0,
            navigation_waypoint_altitude_command=100.0,
            runtime_duration_s=0.1,
            runtime_time_step_s=0.1,
        )
    )
    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id="composition-applicability",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )
    assert isinstance(response, MissionCompositionTrajectoryResponse)
    final = response.result.objects[0].samples[-1]
    assert final.time_s == 0.1
    assert final.values["model.applicability.declared"] is True
    assert final.values["model.applicability.status"] == "outside_declared_envelope"
    assert "below_declared_altitude" in str(final.values["model.applicability.reason"])
    assert final.values["vehicle.operational"] is True
    assert final.values["guidance.available"] is True
    ####
