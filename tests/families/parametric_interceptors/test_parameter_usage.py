"""Focused no-hidden-baggage witnesses for resolved interceptor parameters."""

from __future__ import annotations

import json

import pytest
from taoryx_parametric_interceptors import (
    DragCoefficientPoint,
    DragCoefficientSchedule,
    ParametricInterceptorMissionCompositionProvider,
    ResolvedParameter,
    ValueOrigin,
    aim9x_block2_profile,
    build_interceptor_authoring_report,
    interceptor,
    interceptor_parameter_usage,
    reported,
    resolve_interceptor,
)


def test_usage_manifest_classifies_every_resolved_value_and_virtual_schedule() -> None:
    resolved = resolve_interceptor(
        interceptor(
            "parameter-usage-generic",
            applicability_mach_max=5.0,
            reported_max_speed_mps=reported(
                1_200.0,
                unit="m/s",
                source_record_id="source:public-speed-claim",
            ),
        )
    )
    usage = interceptor_parameter_usage(resolved)

    assert set(usage) == {
        *resolved.parameters,
        "drag_coefficient_schedule",
        "thrust_profile_schedule",
    }
    assert usage["reference_area_m2"].usage_class == "dynamics_input"
    assert usage["reference_area_m2"].directly_consumed
    assert usage["reference_area_m2"].affects_dynamics
    assert usage["body_diameter_m"].usage_class == "resolution_input"
    assert usage["body_diameter_m"].resolved_sink_ids == ("reference_area_m2",)
    assert usage["length_m"].usage_class == "evidence_only"
    assert not usage["length_m"].affects_dynamics
    assert usage["drag_coefficient"].usage_class == "derived_summary"
    assert usage["reported_max_speed_mps"].usage_class == "calibration_target"
    assert usage["applicability_mach_max"].usage_class == "runtime_advisory"
    assert usage["drag_coefficient_schedule"].usage_class == "dynamics_input"
    assert usage["thrust_profile_schedule"].usage_class == "dynamics_input"
    assert usage["thrust_profile_class"].usage_class == "resolution_input"
    assert set(usage["thrust_profile_class"].resolved_sink_ids) == {
        "thrust_n",
        "thrust_profile_schedule",
    }
    assert set(usage["attitude_bandwidth_rad_s"].consumer_ids) == {
        "analysis.pseudo6_response",
        "runtime.attitude_response_pseudo_6dof",
    }
    assert set(usage["reference_area_m2"].consumer_ids) == {
        "analysis.pseudo6_operating_point_response",
        "runtime.attitude_response_pseudo_6dof",
        "runtime.point_mass_3dof",
    }
    assert "analysis.pseudo6_operating_point_response" in usage["control_configuration"].consumer_ids
    assert "analysis.pseudo6_operating_point_response" not in usage["thrust_n"].consumer_ids
    ####


def test_explicit_overrides_leave_coarse_selectors_visibly_inactive() -> None:
    resolved = resolve_interceptor(
        interceptor(
            "parameter-usage-overrides",
            launch_mass_kg=100.0,
            burnout_mass_kg=60.0,
            propellant_fraction=0.4,
            body_diameter_m=0.2,
            reference_area_m2=0.05,
            burn_time_class="short",
            burn_time_s=5.0,
            control_features=("thrust_vectoring",),
            control_configuration="aerodynamic",
            drag_coefficient_schedule=DragCoefficientSchedule(
                schedule_id="usage-authored-drag-v1",
                points=(
                    DragCoefficientPoint(mach=0.0, coefficient=0.4),
                    DragCoefficientPoint(mach=3.0, coefficient=0.3),
                ),
            ),
        )
    )
    usage = interceptor_parameter_usage(resolved)

    assert usage["body_diameter_m"].usage_class == "inactive_resolution_input"
    assert usage["propellant_fraction"].usage_class == "inactive_resolution_input"
    assert usage["burn_time_class"].usage_class == "inactive_resolution_input"
    assert usage["control_features"].usage_class == "inactive_resolution_input"
    assert "aero_archetype" not in resolved.parameters
    assert "drag_class" not in resolved.parameters
    assert usage["drag_scale"].usage_class == "resolution_input"
    assert usage["drag_scale"].resolved_sink_ids == ("drag_coefficient_schedule",)
    ####


def test_usage_classification_fails_closed_for_an_unregistered_parameter() -> None:
    resolved = resolve_interceptor(interceptor("parameter-usage-fail-closed"))
    parameters = dict(resolved.parameters)
    parameters["mystery_baggage"] = ResolvedParameter(
        value=1.0,
        origin=ValueOrigin.SIMULATION_ASSUMPTION,
        unit="1",
        method="test-only undeclared parameter",
    )
    malformed = resolved.model_copy(update={"parameters": parameters})

    with pytest.raises(ValueError, match="has no declared runtime, resolver, calibration"):
        interceptor_parameter_usage(malformed)
    ####


def test_authoring_and_composition_publish_the_same_usage_contract() -> None:
    report = build_interceptor_authoring_report("examples/parametric_interceptors/generic_medium_sam.yaml")
    assert sum(report.usage_counts.values()) == len(report.resolved_profile.parameters) + 2
    assert report.parameter_usage["length_m"].usage_class == "evidence_only"
    assert "length_m" in report.readiness.evidence_only_parameter_ids
    assert report.parameters_by_usage["dynamics_input"]

    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(aim9x_block2_profile())
    properties = {item.id: item for item in provider.model("aim9x-block2").presentation.properties}
    manifest_value = properties["parameter_usage.manifest"].value
    assert isinstance(manifest_value, str)
    manifest = json.loads(manifest_value)

    assert properties["parameter_usage.contract"].value == "taoryx.parametric-interceptors.parameter-usage/v1"
    assert manifest["body_diameter_m"]["usage_class"] == "inactive_resolution_input"
    assert manifest["control_features"]["usage_class"] == "resolution_input"
    assert manifest["guidance_family"]["usage_class"] == "evidence_only"
    assert "usage_class=evidence_only" in properties["guidance_family"].provenance
    assert "does not affect current low-fidelity dynamics" in properties["guidance_family"].claim_boundary
    assert "usage_class=dynamics_input" in properties["reference_area_m2"].provenance
    ####
