import pytest
from taoryx.trajectory.native_mission_composition import configuration_instance_from_vehicle_request
from taoryx.trajectory.registry_mission_composition import RegistryMissionCompositionProvider
from taoryx.trajectory.session_contract import (
    MissionCompositionClosedSession,
    MissionCompositionCloseSessionRequest,
    MissionCompositionInspectSessionRequest,
    MissionCompositionOpenSessionRequest,
    MissionCompositionResetSessionRequest,
    MissionCompositionSessionDescriptor,
    MissionCompositionSessionManager,
    MissionCompositionSessionObservation,
    MissionCompositionSessionStepRequest,
    MissionCompositionSessionStepResult,
)

from taoryx.composition_episode import registered_episode_factory_ids
from taoryx.trajectory.execution_contract import MissionCompositionExecutionError
from taoryx.vehicle_composition import load_vehicle_composition_request
from taoryx.vehicle_execution_bindings import load_vehicle_execution_binding_catalog
from taoryx.vehicle_execution_witnesses import load_vehicle_execution_witness_catalog
from taoryx.vehicle_registry import ROOT

PROVIDER = RegistryMissionCompositionProvider()
EPISODE_WITNESSES = tuple(item for item in load_vehicle_execution_witness_catalog().witnesses if item.operation == "episode")


def _prepared(witness: object):
    request = load_vehicle_composition_request(ROOT / str(getattr(witness, "composition")))
    return PROVIDER.validate_configuration(configuration_instance_from_vehicle_request(PROVIDER, request))
    ####


def test_episode_factory_registry_exactly_covers_native_catalog() -> None:
    catalog = load_vehicle_execution_binding_catalog()
    declared = {item.factory_id for item in catalog.bindings if item.operation == "episode" and item.status == "runnable" and item.factory_id is not None}
    assert set(registered_episode_factory_ids()) == declared
    ####


@pytest.mark.parametrize("witness", EPISODE_WITNESSES, ids=lambda item: item.id)
def test_every_native_episode_tuple_uses_common_stateful_session(witness: object) -> None:
    prepared = _prepared(witness)
    manager = MissionCompositionSessionManager(PROVIDER)
    session_id = f"session-{getattr(witness, 'id')}"
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id=session_id,
            provider_id=PROVIDER.metadata.id,
            provider_version=PROVIDER.metadata.version,
            prepared_configuration=prepared,
            seed=37,
            integration_step_s=0.02,
        )
    )
    assert descriptor.state_owner == "provider_session"
    assert descriptor.deterministic_reset
    assert descriptor.initial_observation.sequence == 0
    assert set(descriptor.initial_observation.values) == {item.id for item in descriptor.observation_schema}
    assert all(item.data_type and item.sampling_semantics for item in descriptor.observation_schema)
    configuration = prepared.configuration
    realization_id = configuration.realization_id or configuration.fidelity
    model = PROVIDER.model(configuration.model_id)
    realization = next(item for item in model.realizations if item.id == realization_id)
    advertised_actions = {
        item.native_channel_id: item
        for item in realization.controls.channels
        if item.channel_kind == "action" and "step" in item.operations and item.native_channel_id is not None
    }
    runtime_actions = {item.id: item for item in descriptor.action_schema}
    assert set(advertised_actions) == set(runtime_actions)
    for channel_id, runtime_channel in runtime_actions.items():
        advertised = advertised_actions[channel_id]
        native = advertised.native_binding
        assert native is not None
        minimum = native.interval.minimum.value if native.interval is not None and native.interval.minimum is not None else None
        maximum = native.interval.maximum.value if native.interval is not None and native.interval.maximum is not None else None
        assert native.quantity == runtime_channel.quantity
        assert native.canonical_unit == runtime_channel.unit
        assert native.data_type == runtime_channel.data_type
        assert native.shape == runtime_channel.shape
        assert minimum == runtime_channel.minimum
        assert maximum == runtime_channel.maximum
        assert native.value_space.model_dump(mode="json") == runtime_channel.value_space
        if channel_id == "guidance-override-enabled":
            assert runtime_channel.data_type == "boolean"
            assert runtime_channel.minimum is None
            assert runtime_channel.maximum is None
    assert MissionCompositionSessionDescriptor.model_validate_json(descriptor.model_dump_json(by_alias=True)) == descriptor

    inspect_request = MissionCompositionInspectSessionRequest(session_id=session_id)
    assert MissionCompositionInspectSessionRequest.model_validate_json(inspect_request.model_dump_json(by_alias=True)) == inspect_request
    inspected = manager.inspect(inspect_request)
    assert inspected == descriptor.initial_observation
    stepped = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=session_id,
            action={},
            duration_s=0.02,
            expected_sequence=0,
        )
    )
    assert stepped.sequence == 1
    assert stepped.time_end_s > stepped.time_start_s
    assert stepped.observation.lifecycle in {"active", "completed"}
    assert MissionCompositionSessionStepResult.model_validate_json(stepped.model_dump_json()) == stepped

    reset = manager.reset(MissionCompositionResetSessionRequest(session_id=session_id, seed=37))
    assert reset.sequence == 0
    assert reset.time_s == descriptor.initial_observation.time_s
    assert reset.values == descriptor.initial_observation.values
    assert MissionCompositionSessionObservation.model_validate_json(reset.model_dump_json()) == reset

    close_request = MissionCompositionCloseSessionRequest(session_id=session_id)
    assert MissionCompositionCloseSessionRequest.model_validate_json(close_request.model_dump_json(by_alias=True)) == close_request
    closed = manager.close(close_request)
    assert closed.lifecycle == "closed"
    assert MissionCompositionClosedSession.model_validate_json(closed.model_dump_json()) == closed
    assert manager.inspect(session_id).lifecycle == "closed"
    assert manager.active_session_ids() == ()
    ####


def test_registered_interactive_matrix_exactly_matches_episode_witnesses() -> None:
    advertised: set[tuple[str, str, str, str | None]] = set()
    for model in PROVIDER.list_models():
        for mission in model.mission_templates:
            for operation in mission.operations:
                if operation.operation == "step" and operation.common_runner_status == "registered":
                    advertised.add((model.id, mission.id, operation.fidelity, operation.realization_id))
    witnessed: set[tuple[str, str, str, str | None]] = set()
    for witness in EPISODE_WITNESSES:
        prepared = _prepared(witness)
        configuration = prepared.configuration
        witnessed.add(
            (
                configuration.model_id,
                str(configuration.mission_template_id),
                configuration.fidelity,
                configuration.realization_id,
            )
        )
    witnessed.add(
        (
            "a320_openap_3dof",
            "powered_fixed_wing_racetrack_v1",
            "pseudo_6dof",
            "jsbsim_surrogate_composite_pseudo6dof",
        )
    )
    assert advertised == witnessed
    ####


def test_stateful_session_lifecycle_and_action_failures_are_structured() -> None:
    witness = EPISODE_WITNESSES[0]
    prepared = _prepared(witness)
    manager = MissionCompositionSessionManager(PROVIDER)
    request = MissionCompositionOpenSessionRequest(
        session_id="session-lifecycle-contract",
        provider_id=PROVIDER.metadata.id,
        provider_version=PROVIDER.metadata.version,
        prepared_configuration=prepared,
        seed=91,
        integration_step_s=0.02,
    )
    descriptor = manager.open(request)

    with pytest.raises(MissionCompositionExecutionError) as duplicate:
        manager.open(request)
    assert duplicate.value.diagnostic.code == "session-already-exists"
    assert duplicate.value.diagnostic.phase == "preflight"

    assert descriptor.action_schema
    channel = descriptor.action_schema[0]
    invalid_value: object
    if channel.shape:
        invalid_value = 1.0
    elif channel.data_type == "float64":
        invalid_value = "not-a-number"
    elif channel.data_type == "int64":
        invalid_value = 1.5
    elif channel.data_type == "boolean":
        invalid_value = "not-a-boolean"
    elif channel.data_type == "string":
        invalid_value = 17
    else:
        invalid_value = object()
    with pytest.raises(MissionCompositionExecutionError) as invalid_action:
        manager.step(
            MissionCompositionSessionStepRequest(
                session_id=request.session_id,
                action={channel.id: invalid_value},
                duration_s=0.02,
                expected_sequence=0,
            )
        )
    assert invalid_action.value.diagnostic.code == "invalid-action-value"
    assert invalid_action.value.diagnostic.phase == "preflight"

    manager.step(
        MissionCompositionSessionStepRequest(
            session_id=request.session_id,
            action={},
            duration_s=0.02,
            expected_sequence=0,
        )
    )
    with pytest.raises(MissionCompositionExecutionError) as stale:
        manager.step(
            MissionCompositionSessionStepRequest(
                session_id=request.session_id,
                action={},
                duration_s=0.02,
                expected_sequence=0,
            )
        )
    assert stale.value.diagnostic.code == "stale-session-sequence"

    manager.close(request.session_id)
    with pytest.raises(MissionCompositionExecutionError) as closed:
        manager.step(
            MissionCompositionSessionStepRequest(
                session_id=request.session_id,
                action={},
                duration_s=0.02,
            )
        )
    assert closed.value.diagnostic.code == "session-closed"

    with pytest.raises(MissionCompositionExecutionError) as missing:
        manager.inspect("session-does-not-exist")
    assert missing.value.diagnostic.code == "unknown-session"
    ####
