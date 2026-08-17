"""Focused evidence, interpolation, and shared-runtime Mach-drag witnesses."""

from __future__ import annotations

import math

import pytest
from pydantic import ValidationError
from taoryx_parametric_interceptors import (
    AssumptionCase,
    DragCoefficientSchedule,
    ParametricInterceptorMissionCompositionProvider,
    PointMassMission,
    ValueOrigin,
    calibrated,
    evaluate_maneuver_drag,
    flat_interceptor_profile_schema,
    interceptor,
    load_interceptor_profile,
    resolve_interceptor,
    run_point_mass_interceptor,
    run_pseudo6_interceptor,
)

from taoryx.contracts import Frame, FrameVector3, Vector3
from taoryx.runtime.environment_runtime import EnvironmentSample, StaticEnvironmentProvider
from taoryx.trajectory.execution_contract import MissionCompositionOutputSelection, MissionCompositionRunRequest


def _authored_schedule(**updates: object) -> DragCoefficientSchedule:
    values: dict[str, object] = {
        "schedule_id": "test-authored-drag-v1",
        "points": (
            {"mach": 0.0, "coefficient": 0.20},
            {"mach": 1.0, "coefficient": 0.40},
            {"mach": 2.0, "coefficient": 0.30},
        ),
        "origin": "simulation_assumption",
        "method": "focused authored drag schedule",
    }
    values.update(updates)
    return DragCoefficientSchedule.model_validate(values)
    ####


def test_drag_schedule_interpolates_and_holds_without_hidden_extrapolation() -> None:
    schedule = _authored_schedule()

    assert schedule.coefficient_at(0.0) == pytest.approx(0.20)
    assert schedule.coefficient_at(0.5) == pytest.approx(0.30)
    assert schedule.coefficient_at(1.5) == pytest.approx(0.35)
    assert schedule.coefficient_at(8.0) == pytest.approx(0.30)
    assert len(schedule.fingerprint) == 64
    with pytest.raises(ValueError, match="finite and nonnegative"):
        schedule.coefficient_at(-0.1)
    ####


def test_drag_schedule_contract_rejects_ambiguous_or_untraceable_authoring() -> None:
    with pytest.raises(ValidationError, match="strictly increasing"):
        _authored_schedule(
            points=(
                {"mach": 1.0, "coefficient": 0.4},
                {"mach": 0.8, "coefficient": 0.3},
            )
        )
    with pytest.raises(ValidationError, match="observed drag schedule requires"):
        _authored_schedule(origin="observed")
    with pytest.raises(ValidationError, match="replaces these coarse aerodynamic selectors"):
        interceptor(
            "ambiguous-aero-authority",
            drag_class="nominal",
            drag_coefficient_schedule=_authored_schedule(),
        )
    ####


def test_maneuver_drag_uses_only_achieved_aerodynamic_normal_force() -> None:
    evaluation = evaluate_maneuver_drag(
        dynamic_pressure_pa=1_000.0,
        reference_area_m2=0.2,
        mass_kg=100.0,
        aerodynamic_lateral_acceleration_mps2=10.0,
        maneuver_drag_factor=0.1,
    )

    assert evaluation.normal_force_coefficient == pytest.approx(5.0)
    assert evaluation.drag_coefficient == pytest.approx(2.5)
    assert evaluation.drag_n == pytest.approx(500.0)

    unloaded = evaluate_maneuver_drag(
        dynamic_pressure_pa=1_000.0,
        reference_area_m2=0.2,
        mass_kg=100.0,
        aerodynamic_lateral_acceleration_mps2=0.0,
        maneuver_drag_factor=0.1,
    )
    assert unloaded.normal_force_coefficient == 0.0
    assert unloaded.drag_coefficient == 0.0
    assert unloaded.drag_n == 0.0

    with pytest.raises(ValueError, match="requires positive dynamic pressure"):
        evaluate_maneuver_drag(
            dynamic_pressure_pa=0.0,
            reference_area_m2=0.2,
            mass_kg=100.0,
            aerodynamic_lateral_acceleration_mps2=1.0,
            maneuver_drag_factor=0.1,
        )
    ####


def test_archetype_drag_schedule_preserves_case_order_and_reference_coefficient() -> None:
    profile = interceptor("archetype-drag-witness")
    resolved = {case: resolve_interceptor(profile, assumption_case=case) for case in AssumptionCase}

    conservative = resolved[AssumptionCase.CONSERVATIVE]
    nominal = resolved[AssumptionCase.NOMINAL]
    optimistic = resolved[AssumptionCase.OPTIMISTIC]
    assert conservative.drag_coefficient_schedule.origin == "archetype_assumption"
    assert nominal.drag_coefficient_schedule.coefficient_at(1.0) > nominal.drag_coefficient_schedule.coefficient_at(2.0)
    for mach in (0.0, 0.9, 1.0, 1.5, 3.0, 6.0):
        assert (
            conservative.drag_coefficient_schedule.coefficient_at(mach)
            > nominal.drag_coefficient_schedule.coefficient_at(mach)
            > optimistic.drag_coefficient_schedule.coefficient_at(mach)
        )
    assert nominal.number("drag_coefficient") == pytest.approx(nominal.drag_coefficient_schedule.coefficient_at(2.0))
    assert nominal.number("maneuver_drag_factor") == 0.1
    assert nominal.parameters["maneuver_drag_factor"].origin is ValueOrigin.ARCHETYPE_ASSUMPTION
    assert len({item.drag_coefficient_schedule.fingerprint for item in resolved.values()}) == 3
    ####


def test_authored_schedule_replaces_coarse_selectors_and_retains_lineage() -> None:
    schedule = _authored_schedule(
        origin="observed",
        confidence="high",
        source_record_ids=("source:wind-tunnel-table",),
    )
    resolved = resolve_interceptor(
        interceptor(
            "observed-drag-schedule",
            drag_coefficient_schedule=schedule,
        )
    )

    assert "aero_archetype" not in resolved.parameters
    assert "drag_class" not in resolved.parameters
    assert resolved.drag_coefficient_schedule.origin == "observed"
    assert resolved.drag_coefficient_schedule.source_record_ids == ("source:wind-tunnel-table",)
    assert resolved.parameters["drag_coefficient"].origin is ValueOrigin.OBSERVED
    assert resolved.parameters["drag_coefficient"].source_record_ids == ("source:wind-tunnel-table",)

    calibrated_resolved = resolve_interceptor(
        interceptor(
            "calibrated-drag-schedule",
            drag_coefficient_schedule=_authored_schedule(),
            drag_scale=calibrated(1.2, unit="1", method="focused drag fit"),
        )
    )
    assert calibrated_resolved.drag_coefficient_schedule.origin == "calibrated"
    assert calibrated_resolved.drag_coefficient_schedule.coefficient_at(1.0) == pytest.approx(0.48)
    assert calibrated_resolved.parameters["drag_coefficient"].origin is ValueOrigin.CALIBRATED
    ####


def test_both_runtime_tiers_share_active_schedule_and_composition_advertises_it() -> None:
    schedule = _authored_schedule()
    resolved = resolve_interceptor(
        interceptor(
            "runtime-drag-schedule",
            reference_area_m2=1.0,
            drag_coefficient_schedule=schedule,
        )
    )
    environment = StaticEnvironmentProvider(
        EnvironmentSample(
            density=1.0,
            pressure=100_000.0,
            temperature=250.0,
            speed_of_sound=100.0,
            wind=FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
        )
    )
    mission = PointMassMission(
        launch_altitude_m=100.0,
        launch_speed_mps=150.0,
        launch_flight_path_deg=0.0,
        waypoint_north_m=10_000.0,
        waypoint_altitude_m=100.0,
        duration_s=0.1,
        time_step_s=0.1,
    )
    point = run_point_mass_interceptor(resolved, mission, environment=environment, gravity_acceleration=lambda _: 0.0)
    pseudo = run_pseudo6_interceptor(resolved, mission, environment=environment, gravity_acceleration=lambda _: 0.0)
    expected_coefficient = schedule.coefficient_at(1.5)
    expected_drag = 0.5 * 150.0**2 * expected_coefficient

    assert point.samples[0].mach == pytest.approx(1.5)
    assert point.samples[0].drag_coefficient == pytest.approx(expected_coefficient)
    assert pseudo.samples[0].drag_coefficient == pytest.approx(expected_coefficient)
    assert point.samples[0].drag_n == pytest.approx(expected_drag)
    assert pseudo.samples[0].drag_n == pytest.approx(expected_drag)
    assert point.samples[0].base_drag_n == pytest.approx(expected_drag)
    assert point.samples[0].maneuver_drag_n == 0.0
    assert point.samples[0].total_drag_coefficient == pytest.approx(expected_coefficient)

    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(resolved)
    model = provider.model(resolved.model_id)
    properties = {item.id: item for item in model.presentation.properties}
    outputs = {item.id: item for item in model.output_schema.telemetry_channels}
    assert properties["drag_schedule.fingerprint"].value == resolved.drag_coefficient_schedule.fingerprint
    assert properties["drag_schedule.point_count"].value == 3
    assert properties["drag_schedule.contract"].value == "taoryx.parametric-interceptors.drag-schedule/v1"
    assert properties["drag_schedule.method"].value == resolved.drag_coefficient_schedule.method
    assert outputs["aerodynamics.drag_coefficient"].canonical_unit == "1"
    assert outputs["aerodynamics.maneuver.drag_coefficient"].canonical_unit == "1"
    assert outputs["aerodynamics.drag.base"].canonical_unit == "N"
    assert outputs["aerodynamics.drag.maneuver"].canonical_unit == "N"
    configuration = provider.configuration(
        resolved.model_id,
        runtime_duration_s=0.1,
        runtime_time_step_s=0.1,
    )
    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id="drag-schedule-composition",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=provider.validate_configuration(configuration),
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )
    values = response.result.objects[0].samples[0].values  # type: ignore[union-attr]
    assert math.isclose(
        float(values["aerodynamics.drag_coefficient"]),
        resolved.drag_coefficient_schedule.coefficient_at(float(values["aerodynamics.mach"])),
    )
    ####


def test_flat_schema_exposes_structured_drag_schedule_instead_of_scalar_alias() -> None:
    schema = flat_interceptor_profile_schema()
    schedule = schema["properties"]["drag_coefficient_schedule"]

    assert "$ref" in schedule["anyOf"][0]
    assert "DragCoefficientSchedule" in schedule["anyOf"][0]["$ref"]
    example = load_interceptor_profile("examples/parametric_interceptors/custom_mach_drag_sam.yaml")
    assert example.drag_coefficient_schedule is not None
    assert example.drag_coefficient_schedule.schedule_id == "custom-mach-drag-sam-v1"
    ####
