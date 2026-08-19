import math

import pytest
from taoryx.passive_tumbling_mission_translation import compile_passive_tumbling_mission
from taoryx.trajectory.dual_launch_mission_composition import DUAL_LAUNCH_MODEL_ID
from taoryx.trajectory.simple_aero_mission_composition import build_simple_aero_example_configuration

from taoryx.trajectory.configuration_contract import ConfigurationParameterValue
from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunRequest,
    parse_mission_composition_response,
    resolve_output_selection,
)
from taoryx.trajectory.native_mission_composition import (
    build_registry_mission_composition_runner,
    compile_prepared_vehicle_composition,
    configuration_instance_from_vehicle_request,
)
from taoryx.trajectory.registry_mission_composition import RegistryMissionCompositionProvider
from taoryx.vehicle_batch_execution import registered_vehicle_batch_factory_ids
from taoryx.vehicle_composition import load_vehicle_composition_request
from taoryx.vehicle_execution_bindings import load_vehicle_execution_binding_catalog
from taoryx.vehicle_execution_witnesses import load_vehicle_execution_witness_catalog
from taoryx.vehicle_registry import ROOT

PROVIDER = RegistryMissionCompositionProvider()
RUNNER = build_registry_mission_composition_runner(PROVIDER)
BATCH_WITNESSES = tuple(item for item in load_vehicle_execution_witness_catalog().witnesses if item.operation == "batch")
_SLOW_BATCH_WITNESS_IDS = frozenset(
    {
        "b747-3dof-batch",
        "b747-direct-wrench-batch",
        "b747-pseudo6dof-batch",
        "b747-condition3-local-physical-surface-lqi-screen-batch",
        "f16-local-surface-lqr-schedule-transition-screen-batch",
        "hl20-source-surface-attitude-rate-lqi-screen-batch",
        "x15-source-surface-attitude-rate-lqi-screen-batch",
        "x8-direct-wrench-batch",
        "x8-local-physical-surface-lqi-screen-batch",
    }
)


def _batch_witness_parameters() -> tuple[object, ...]:
    """Keep long transport and controller-screen proofs out of the inner loop.

    The marked witnesses still execute their exact advertised common-runner
    paths in the complete suite. Their separate marker prevents full routes
    and schedule-transition screens from turning focused native-output edits
    into a many-minute test run.
    """

    return tuple(
        pytest.param(
            witness,
            id=witness.id,
            marks=pytest.mark.slow if witness.id in _SLOW_BATCH_WITNESS_IDS else (),
        )
        for witness in BATCH_WITNESSES
    )
    ####


def _request(witness: object, *, mode: str = "all") -> tuple[MissionCompositionRunRequest, object]:
    composition_path = ROOT / str(getattr(witness, "composition"))
    native_request = load_vehicle_composition_request(composition_path)
    configuration = configuration_instance_from_vehicle_request(PROVIDER, native_request)
    prepared = PROVIDER.validate_configuration(configuration)
    request = MissionCompositionRunRequest(
        request_id=f"common-{getattr(witness, 'id')}",
        provider_id=PROVIDER.metadata.id,
        provider_version=PROVIDER.metadata.version,
        prepared_configuration=prepared,
        output=MissionCompositionOutputSelection(mode=mode, maximum_samples_per_object=17),
    )
    return request, compile_prepared_vehicle_composition(PROVIDER, prepared)
    ####


def _named_tumbling_request(shape: str, fidelity: str) -> MissionCompositionRunRequest:
    witness_id = "tumbling-3dof-batch" if fidelity == "point_mass_3dof" else "tumbling-pseudo6dof-batch"
    witness = next(item for item in BATCH_WITNESSES if getattr(item, "id") == witness_id)
    native_request = load_vehicle_composition_request(ROOT / str(getattr(witness, "composition")))
    configuration = configuration_instance_from_vehicle_request(PROVIDER, native_request)
    root = configuration.root
    initialization = root.values["initialization"]
    parameter_group = initialization.value
    parameter_group = parameter_group.model_copy(
        update={
            "values": {
                **parameter_group.values,
                "body_shape": ConfigurationParameterValue(value=shape),
            }
        }
    )
    initialization = initialization.model_copy(update={"value": parameter_group})
    configuration = configuration.model_copy(
        update={
            "realization_id": shape,
            "root": root.model_copy(update={"values": {**root.values, "initialization": initialization}}),
        }
    )
    return MissionCompositionRunRequest(
        request_id=f"common-tumbling-{shape}-{fidelity}",
        provider_id=PROVIDER.metadata.id,
        provider_version=PROVIDER.metadata.version,
        prepared_configuration=PROVIDER.validate_configuration(configuration),
        output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=11),
    )
    ####


def test_batch_factory_registry_exactly_covers_native_catalog() -> None:
    catalog = load_vehicle_execution_binding_catalog()
    declared = {item.factory_id for item in catalog.bindings if item.operation == "batch" and item.status == "runnable" and item.factory_id is not None}
    assert set(registered_vehicle_batch_factory_ids()) == declared
    ####


def test_common_runner_registrations_match_batch_ready_models() -> None:
    advertised = {(PROVIDER.metadata.id, item.id) for item in PROVIDER.list_models() if "batch" in item.common_runner_operations}
    assert set(RUNNER.registrations()) == advertised
    ####


@pytest.mark.integration
@pytest.mark.parametrize("witness", _batch_witness_parameters())
def test_every_native_batch_tuple_executes_through_common_runner(witness: object) -> None:
    request, composition = _request(witness)
    response = RUNNER.run(request)
    assert response.kind == "trajectory", response.model_dump(mode="json", by_alias=True)
    result = response.result
    assert result.status == "completed"
    assert result.primary_model_id == composition.vehicle_id
    primary = next(item for item in result.objects if item.object_id == result.primary_object_id)
    advertised = resolve_output_selection(
        PROVIDER.get_model_output_schema(composition.vehicle_id),
        request.output,
        fidelity=composition.fidelity,
        operation="batch",
        realization_id=request.prepared_configuration.configuration.realization_id,
        mission_template_id=composition.mission,
    )
    assert {item.id for item in primary.channels} == {item.id for item in advertised}
    assert all(item.values.keys() == {channel.id for channel in primary.channels} for item in primary.samples)
    for item in result.objects:
        for sample in item.samples:
            standard = sample.standard_ecef
            assert standard.frame_id == "ecfc"
            assert len(standard.position_ecef_m) == 3
            assert len(standard.velocity_ecef_mps) == 3
            assert len(standard.acceleration_ecef_mps2) == 3
            assert len(standard.angular_velocity_body_radps) == 3
            assert len(standard.ecef_from_body_wxyz) == 4
            assert all(
                math.isfinite(value)
                for value in (
                    *standard.position_ecef_m,
                    *standard.velocity_ecef_mps,
                    *standard.acceleration_ecef_mps2,
                    *standard.angular_velocity_body_radps,
                    *standard.ecef_from_body_wxyz,
                )
            )
            assert math.isclose(sum(value * value for value in standard.ecef_from_body_wxyz), 1.0, abs_tol=1.0e-9)
    assert any(item.channel_class == "core_state" for item in primary.channels)
    assert parse_mission_composition_response(response.model_dump(mode="json", by_alias=True)) == response
    if composition.vehicle_id in {"x15", "hl20_mod_k"} and composition.fidelity in {
        "point_mass_3dof",
        "pseudo_6dof",
    }:
        assert len(result.objects) == 2
        assert len(result.relationships) == 1
        child = next(item for item in result.objects if item.parent_object_id is not None)
        assert child.samples[0].values == result.relationships[0].initial_state.values
        assert child.spawn_event_id == result.relationships[0].event_id
    else:
        assert len(result.relationships) == 0
    ####


def test_available_batch_operation_matrix_exactly_matches_witnesses() -> None:
    advertised: set[tuple[str, str, str, str | None]] = set()
    for model in PROVIDER.list_models():
        for mission in model.mission_templates:
            for operation in mission.operations:
                if operation.operation == "batch" and operation.common_runner_status == "registered":
                    advertised.add((model.id, mission.id, operation.fidelity, operation.realization_id))
    witnessed: set[tuple[str, str, str, str | None]] = set()
    for witness in BATCH_WITNESSES:
        request, composition = _request(witness, mode="core")
        witnessed.add(
            (
                composition.vehicle_id,
                composition.mission,
                composition.fidelity,
                request.prepared_configuration.configuration.realization_id,
            )
        )
    witnessed.update(
        (
            "tumbling_body",
            "tumbling_body_release_damping_impact_v1",
            fidelity,
            shape,
        )
        for fidelity in ("point_mass_3dof", "pseudo_6dof")
        for shape in ("cylinder", "sphere", "cone", "triaxial_ellipsoid")
    )
    witnessed.update(
        ("simple_aero", mission.id, "point_mass_3dof", "fixed_ld_point_mass")
        for mission in PROVIDER.model("simple_aero").mission_templates
    )
    witnessed.add(
        (
            "a320_openap_3dof",
            "powered_fixed_wing_racetrack_v1",
            "pseudo_6dof",
            "jsbsim_surrogate_composite_pseudo6dof",
        )
    )
    witnessed.add(
        (
            "a320_openap_3dof",
            "a320_local_native_coordinate_lqi_screen_v1",
            "pseudo_6dof",
            "jsbsim_surrogate_composite_pseudo6dof",
        )
    )
    witnessed.update(
        (DUAL_LAUNCH_MODEL_ID, mission_id, "point_mass_3dof", "generated_native_problem")
        for mission_id in ("air_release_waypoint", "attached_booster_waypoint")
    )
    assert advertised == witnessed
    ####


@pytest.mark.parametrize("shape", ("cylinder", "sphere", "cone", "triaxial_ellipsoid"))
@pytest.mark.parametrize("fidelity", ("point_mass_3dof", "pseudo_6dof"))
def test_every_named_tumbling_realization_executes_through_common_runner(shape: str, fidelity: str) -> None:
    request = _named_tumbling_request(shape, fidelity)
    composition = compile_prepared_vehicle_composition(PROVIDER, request.prepared_configuration)
    assert compile_passive_tumbling_mission(composition).body.shape.value == shape
    response = RUNNER.run(request)
    assert response.kind == "trajectory", response.model_dump(mode="json", by_alias=True)
    assert response.result.status == "completed"
    assert response.result.primary_model_id == "tumbling_body"
    assert {item.id for item in response.result.objects[0].channels} == {item.id for item in PROVIDER.get_model_output_schema("tumbling_body").channels}
    ####


def test_simple_aero_registered_baseline_executes_through_common_runner() -> None:
    prepared = PROVIDER.validate_configuration(build_simple_aero_example_configuration(PROVIDER.get_model_schema("simple_aero")))
    request = MissionCompositionRunRequest(
        request_id="common-simple-aero-fixed-ld",
        provider_id=PROVIDER.metadata.id,
        provider_version=PROVIDER.metadata.version,
        prepared_configuration=prepared,
        output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=17),
    )
    response = RUNNER.run(request)
    assert response.kind == "trajectory", response.model_dump(mode="json", by_alias=True)
    assert response.result.status == "completed"
    assert len(response.result.objects) == 1
    primary = response.result.objects[0]
    assert len(primary.samples) == 17
    assert {item.id for item in primary.channels} == {item.id for item in PROVIDER.get_model_output_schema("simple_aero").channels}
    ####
