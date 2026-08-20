"""Validate scoped Mission Composition provider contracts.

This inexpensive, provider-scoped complement to a vehicle vertical test has
two explicit profiles. ``interoperable`` accepts an honest batch-only or
step-only provider and verifies the exact advertised operations. The
``taoryx_universal`` profile is stricter: every TAORYX host-facing model must
publish matching common ``batch`` and ``step`` seams. In both cases the common
result and session models make the standard ECEF kinematic/orientation sidecar
mandatory wherever a provider returns samples.
"""

from __future__ import annotations

import argparse
import json
from collections.abc import Iterable, Sequence
from typing import Any, Literal

from taoryx_trajectory_contracts import (
    BatchOutputSelection,
    BatchRunRequest,
    CloseStreamingSessionRequest,
    OpenStreamingSessionRequest,
    StreamingStepRequest,
    audit_batch_result,
    audit_default_configuration_provider,
    audit_provider_descriptor,
    audit_streaming_descriptor,
    audit_streaming_step,
)
from taoryx_trajectory_contracts import (
    StandardEcefState as PublicStandardEcefState,
)

from taoryx.plugins import PluginCatalog, discover_plugins
from taoryx.trajectory.contracts_adapter import TaoryxTrajectoryContractsAdapter
from taoryx.trajectory.execution_contract import (
    MissionCompositionRunnerRegistry,
    TrajectorySample,
    audit_provider_advertisement,
)
from taoryx.trajectory.session_contract import MissionCompositionSessionObservation
from taoryx.trajectory.standard_output import StandardEcefState

_CORE_BATCH_REPLAY_EXECUTION_MODE = "core_batch_replay"
_CORE_BATCH_REPLAY_SESSION_EXECUTOR_ID = "taoryx.core.batch-replay-session.v1"
ContractProfile = Literal["interoperable", "taoryx_universal"]
_CONTRACT_PROFILES: tuple[ContractProfile, ...] = ("interoperable", "taoryx_universal")
_STANDARD_ECEF_FIELDS = (
    "position_ecef_m",
    "velocity_ecef_mps",
    "acceleration_ecef_mps2",
    "angular_velocity_body_radps",
    "ecef_from_body_wxyz",
    "position_projection",
    "source_frame",
    "orientation_kind",
)


def _normalize_plugin_ids(plugin_ids: Iterable[str]) -> tuple[str, ...]:
    """Return a deterministic non-empty selected plug-in scope."""

    normalized = tuple(sorted({item.strip() for item in plugin_ids if item.strip()}))
    if not normalized:
        raise ValueError("at least one non-empty plug-in ID is required")
    return normalized
    ####


def _standard_ecef_contract() -> dict[str, Any]:
    """Check the core types that impose standard pose data on every provider."""

    errors: list[str] = []
    frame_field = StandardEcefState.model_fields.get("frame_id")
    frame_id = None if frame_field is None else frame_field.default
    if frame_id != "ecfc":
        errors.append("StandardEcefState.frame_id must be fixed to 'ecfc' (ECEF)")
    if StandardEcefState is not PublicStandardEcefState:
        errors.append("TAORYX standard ECEF state must be the public trajectory-contracts type")
    sample_field = TrajectorySample.model_fields.get("standard_ecef")
    if sample_field is None or not sample_field.is_required():
        errors.append("TrajectorySample.standard_ecef must be required")
    observation_field = MissionCompositionSessionObservation.model_fields.get("standard_ecef")
    if observation_field is None or not observation_field.is_required():
        errors.append("MissionCompositionSessionObservation.standard_ecef must be required")
    missing_state_fields = tuple(
        field for field in _STANDARD_ECEF_FIELDS if field not in StandardEcefState.model_fields
    )
    if missing_state_fields:
        errors.append(f"StandardEcefState is missing required fields {missing_state_fields!r}")
    nonrequired_state_fields = tuple(
        field
        for field in _STANDARD_ECEF_FIELDS
        if field in StandardEcefState.model_fields and not StandardEcefState.model_fields[field].is_required()
    )
    if nonrequired_state_fields:
        errors.append(f"StandardEcefState fields must be required: {nonrequired_state_fields!r}")
    return {
        "status": "pass" if not errors else "fail",
        "frame_id": frame_id,
        "result_sample_required": sample_field is not None and sample_field.is_required(),
        "session_observation_required": observation_field is not None and observation_field.is_required(),
        "required_fields": list(_STANDARD_ECEF_FIELDS),
        "errors": errors,
    }
    ####


def _public_contract_projection(
    provider: Any,
    *,
    require_defaults: bool,
) -> dict[str, Any]:
    """Audit the standalone projection and, when required, its defaults.

    This remains provider-scoped and stops before numerical execution.  It
    proves that a host can take each advertised model from discovery through a
    deterministic provider-selected runnable default and normal preparation
    without guessing at required catalog values. Full batch and streaming
    execution remain separate integration checks.
    """

    try:
        adapter = TaoryxTrajectoryContractsAdapter(provider)
        descriptor = adapter.descriptor
        audit = audit_provider_descriptor(descriptor)
        default_audit = audit_default_configuration_provider(adapter) if require_defaults else None
    except Exception as error:  # noqa: BLE001 - preserve a concise plug-in boundary report
        return {
            "status": "fail",
            "model_count": 0,
            "available_batch_tuple_count": 0,
            "available_step_tuple_count": 0,
            "default_configuration_count": 0,
            "defaults": {},
            "errors": [f"public trajectory-contract projection failed: {type(error).__name__}: {error}"],
        }
    source_models = tuple(provider.list_models())
    source_tuples = {
        (
            model.id,
            mission.id,
            operation.fidelity,
            operation.realization_id,
            operation.operation,
            operation.status,
        )
        for model in source_models
        for mission in model.mission_templates
        for operation in mission.operations
    }
    public_tuples = {
        (
            model.id,
            operation.mission_template_id,
            operation.fidelity,
            operation.realization_id,
            operation.operation,
            "available" if operation.availability == "available" else "blocked",
        )
        for model in descriptor.models
        for operation in model.operations
    }
    errors: list[str] = []
    if descriptor.id != provider.metadata.id or descriptor.version != provider.metadata.version:
        errors.append("public trajectory-contract descriptor identity disagrees with provider metadata")
    if source_tuples != public_tuples:
        errors.append("public trajectory-contract operation tuples disagree with the native provider advertisement")
    if audit.status != "pass":
        errors.extend(f"public trajectory-contract audit {item.code}: {item.message}" for item in audit.findings)
    if default_audit is not None and default_audit.status != "pass":
        errors.extend(
            f"public trajectory-contract default audit {item.code}: {item.message}"
            for item in default_audit.findings
        )
    defaults: dict[str, dict[str, object]] = {}
    if require_defaults:
        for model in descriptor.models:
            try:
                configuration = adapter.build_default_configuration(model.id)
                prepared = adapter.prepare_configuration(configuration)
            except Exception as error:  # noqa: BLE001 - retain a model-specific portable construction error
                message = f"default configuration failed: {type(error).__name__}: {error}"
                defaults[model.id] = {
                    "status": "fail",
                    "configuration_id": model.default_configuration_id,
                    "errors": [message],
                }
                errors.append(f"model {model.id}: {message}")
                continue
            default_errors: list[str] = []
            if configuration.configuration_id != model.default_configuration_id:
                default_errors.append(
                    "default configuration ID disagrees with the public model descriptor"
                )
            if prepared.configuration.configuration_id != configuration.configuration_id:
                default_errors.append(
                    "prepared default configuration identity disagrees with its authored configuration"
                )
            defaults[model.id] = {
                "status": "pass" if not default_errors else "fail",
                "configuration_id": configuration.configuration_id,
                "fidelity": configuration.fidelity,
                "realization_id": configuration.realization_id,
                "mission_template_id": configuration.mission_template_id,
                "errors": default_errors,
            }
            errors.extend(f"model {model.id}: {message}" for message in default_errors)
    available = tuple(
        item
        for model in descriptor.models
        for item in model.operations
        if item.availability == "available"
    )
    return {
        "status": "pass" if not errors else "fail",
        "model_count": len(descriptor.models),
        "available_batch_tuple_count": sum(item.operation == "batch" for item in available),
        "available_step_tuple_count": sum(item.operation == "step" for item in available),
        "default_configuration_count": sum(item["status"] == "pass" for item in defaults.values()),
        "defaults": defaults,
        "errors": errors,
    }
    ####


def _registered_operations(model: Any, operation_name: Literal["batch", "step"]) -> tuple[tuple[Any, Any], ...]:
    """Return exact available common tuples for one declared operation."""

    return tuple(
        (mission, operation)
        for mission in model.mission_templates
        for operation in mission.operations
        if (
            operation.operation == operation_name
            and operation.status == "available"
            and operation.common_runner_status == "registered"
        )
    )
    ####


def _matching_operations(
    mission: Any,
    source: Any,
    *,
    operation_name: Literal["batch", "step"],
) -> tuple[Any, ...]:
    """Find exact counterpart tuples without widening realization authority."""

    return tuple(
        operation
        for operation in mission.operations
        if (
            operation.operation == operation_name
            and operation.status == "available"
            and operation.common_runner_status == "registered"
            and operation.fidelity == source.fidelity
            and operation.realization_id in {None, source.realization_id}
        )
    )
    ####


def _capability_shape(batches: tuple[tuple[Any, Any], ...], steps: tuple[tuple[Any, Any], ...]) -> str:
    """Return the truthful host-operation shape for one model."""

    if batches and steps:
        return "batch_and_step"
    if batches:
        return "batch_only"
    if steps:
        return "step_only"
    return "catalog_only"
    ####


def _execute_public_default(
    provider: Any,
    model_id: str,
    *,
    execute_batch: bool,
) -> dict[str, object]:
    """Exercise one prepared default through public step and optional batch interfaces."""

    adapter = TaoryxTrajectoryContractsAdapter(provider)
    configuration = adapter.build_default_configuration(model_id)
    prepared = adapter.prepare_configuration(configuration)
    batch_entity_count: int | None = None
    batch_sample_count: int | None = None
    if execute_batch:
        batch = adapter.run_batch(
            BatchRunRequest(
                request_id=f"contract-default-batch-{model_id}",
                prepared_configuration=prepared,
                output=BatchOutputSelection(
                    include_events=False,
                    include_segments=False,
                    maximum_entities=2,
                ),
            )
        )
        batch_audit = audit_batch_result(batch)
        if batch_audit.status != "pass":
            raise ValueError("public batch result does not satisfy the trajectory-contracts audit")
        batch_entity_count = len(batch.entities)
        batch_sample_count = sum(len(entity.samples) for entity in batch.entities)
    session_id = f"contract-default-stream-{model_id}"
    opened = adapter.open_stream(
        OpenStreamingSessionRequest(
            session_id=session_id,
            prepared_configuration=prepared,
            seed=19,
        )
    )
    try:
        descriptor_audit = audit_streaming_descriptor(opened)
        if descriptor_audit.status != "pass":
            raise ValueError("public streaming descriptor does not satisfy the trajectory-contracts audit")
        stepped = adapter.step_stream(
            StreamingStepRequest(
                session_id=session_id,
                duration_s=0.02,
                expected_sequence=0,
            )
        )
        step_audit = audit_streaming_step(stepped)
        if step_audit.status != "pass":
            raise ValueError("public streaming step does not satisfy the trajectory-contracts audit")
    finally:
        adapter.close_stream(CloseStreamingSessionRequest(session_id=session_id))
    return {
        "status": "pass",
        "configuration_id": configuration.configuration_id,
        "batch_executed": execute_batch,
        "batch_entity_count": batch_entity_count,
        "batch_sample_count": batch_sample_count,
        "stream_session_id": session_id,
        "stream_step_sequence": stepped.sequence,
    }
    ####


def _provider_report(
    *,
    contribution_id: str,
    plugin_id: str,
    provider: Any,
    contract_profile: ContractProfile,
    execute_defaults: bool = False,
    execute_batch_defaults: bool = False,
) -> dict[str, object]:
    """Audit one resolved provider against one explicit host profile."""

    if execute_batch_defaults:
        execute_defaults = True
    errors: list[str] = []
    try:
        metadata = provider.metadata
        provider_id = str(metadata.id)
        provider_version = str(metadata.version)
    except Exception as error:  # noqa: BLE001 - a plug-in is an untrusted boundary
        return {
            "contribution_id": contribution_id,
            "plugin_id": plugin_id,
            "provider_id": None,
            "provider_version": None,
            "contract_profile": contract_profile,
            "status": "fail",
            "runner_type": None,
            "advertisement_audit_status": "not_run",
            "model_count": 0,
            "registered_batch_tuple_count": 0,
            "registered_step_tuple_count": 0,
            "capability_shape_counts": {},
            "models": [],
            "errors": [f"provider construction failed: {type(error).__name__}: {error}"],
        }
    if provider_id != contribution_id:
        errors.append(
            f"provider metadata ID {provider_id!r} does not match registered contribution {contribution_id!r}"
        )
    try:
        provider_models = provider.list_models()
    except Exception as error:  # noqa: BLE001 - preserve a scoped construction/audit error
        errors.append(f"model discovery failed: {type(error).__name__}: {error}")
        provider_models = ()
    require_defaults = contract_profile == "taoryx_universal" or execute_defaults
    public_contract = _public_contract_projection(provider, require_defaults=require_defaults)
    errors.extend(str(item) for item in public_contract["errors"])
    batches_by_model = {
        model.id: _registered_operations(model, "batch")
        for model in provider_models
    }
    requires_batch_runner = any(batches_by_model.values())
    runner: MissionCompositionRunnerRegistry | None = None
    build_runner = getattr(provider, "build_runner", None)
    if requires_batch_runner:
        if not callable(build_runner):
            errors.append("registered batch execution has no callable build_runner()")
        else:
            try:
                candidate = build_runner()
            except Exception as error:  # noqa: BLE001 - preserve an actionable plug-in failure
                errors.append(f"common runner construction failed: {type(error).__name__}: {error}")
            else:
                if not isinstance(candidate, MissionCompositionRunnerRegistry):
                    errors.append(
                        "build_runner() returned "
                        f"{type(candidate).__name__}, not MissionCompositionRunnerRegistry"
                    )
                else:
                    runner = candidate

    audit_status = "not_run"
    try:
        audit = audit_provider_advertisement(provider, runner)
        audit_status = audit.status
        if audit.status != "pass":
            errors.extend(
                f"advertisement audit {diagnostic.code}: {diagnostic.message}"
                for diagnostic in audit.diagnostics
            )
    except Exception as error:  # noqa: BLE001 - preserve a scoped construction/audit error
        errors.append(f"advertisement audit failed: {type(error).__name__}: {error}")
        audit_status = "fail"

    executed_defaults: dict[str, dict[str, object]] = {}
    if execute_defaults:
        for model in provider_models:
            try:
                executed_defaults[model.id] = _execute_public_default(
                    provider,
                    model.id,
                    execute_batch=execute_batch_defaults,
                )
            except Exception as error:  # noqa: BLE001 - report a portable model-specific integration failure
                executed_defaults[model.id] = {
                    "status": "fail",
                    "errors": [f"default execution failed: {type(error).__name__}: {error}"],
                }

    models: list[dict[str, object]] = []
    registered_batch_tuple_count = 0
    registered_step_tuple_count = 0
    for model in provider_models:
        model_errors: list[str] = []
        default = public_contract["defaults"].get(model.id)
        if require_defaults:
            if default is None:
                model_errors.append("public trajectory-contract projection omitted this model's default configuration")
            elif default["status"] != "pass":
                model_errors.extend(str(item) for item in default["errors"])
        executed_default = executed_defaults.get(model.id)
        if executed_default is not None and executed_default["status"] != "pass":
            execution_errors = executed_default.get("errors", ())
            if isinstance(execution_errors, list | tuple):
                model_errors.extend(str(item) for item in execution_errors)
            else:
                model_errors.append("default execution failed without a structured error list")
        batches = batches_by_model[model.id]
        steps = _registered_operations(model, "step")
        registered_batch_tuple_count += len(batches)
        registered_step_tuple_count += len(steps)
        batch_records: list[dict[str, object]] = []
        step_records: list[dict[str, object]] = []
        capability_shape = _capability_shape(batches, steps)
        if batches:
            if runner is None:
                model_errors.append("registered batch execution has no valid common runner")
            elif not runner.has_executor(provider_id, model.id):
                model_errors.append("registered batch execution has no provider/model common runner executor")
        if contract_profile == "taoryx_universal":
            if model.execution_capability_profile != "taoryx_universal":
                model_errors.append(
                    "TAORYX-universal validation requires "
                    "model.execution_capability_profile to equal 'taoryx_universal'"
                )
            if model.common_runner_operations != ("batch", "step"):
                model_errors.append(
                    "TAORYX-universal validation requires model.common_runner_operations "
                    "to equal ('batch', 'step')"
                )
        for mission, batch in batches:
            matching_steps = _matching_operations(mission, batch, operation_name="step")
            if contract_profile == "taoryx_universal" and not matching_steps:
                model_errors.append(
                    "registered batch tuple "
                    f"{mission.id!r}/{batch.fidelity!r}/{batch.realization_id!r} has no registered step tuple"
                )
            for step in matching_steps:
                if step.execution_mode == _CORE_BATCH_REPLAY_EXECUTION_MODE and (
                    step.executor_id != _CORE_BATCH_REPLAY_SESSION_EXECUTOR_ID
                ):
                    model_errors.append(
                        "core_batch_replay step tuple must use "
                        f"{_CORE_BATCH_REPLAY_SESSION_EXECUTOR_ID!r}"
                    )
            batch_records.append(
                {
                    "mission_template_id": mission.id,
                    "fidelity": batch.fidelity,
                    "realization_id": batch.realization_id,
                    "matching_step_count": len(matching_steps),
                    "matching_step_execution_modes": [step.execution_mode for step in matching_steps],
                }
            )
        for mission, step in steps:
            matching_batches = _matching_operations(mission, step, operation_name="batch")
            if contract_profile == "taoryx_universal" and not matching_batches:
                model_errors.append(
                    "registered step tuple "
                    f"{mission.id!r}/{step.fidelity!r}/{step.realization_id!r} has no registered batch tuple"
                )
            step_records.append(
                {
                    "mission_template_id": mission.id,
                    "fidelity": step.fidelity,
                    "realization_id": step.realization_id,
                    "execution_mode": step.execution_mode,
                    "matching_batch_count": len(matching_batches),
                }
            )
        models.append(
            {
                "model_id": model.id,
                "execution_capability_profile": model.execution_capability_profile,
                "capability_shape": capability_shape,
                "status": "pass" if not model_errors else "fail",
                "registered_batch_tuple_count": len(batches),
                "registered_step_tuple_count": len(steps),
                "default_configuration": default,
                "default_execution": executed_default,
                "batches": batch_records,
                "steps": step_records,
                "errors": model_errors,
            }
        )
        errors.extend(f"model {model.id}: {error}" for error in model_errors)
    capability_shape_counts = {
        shape: sum(item["capability_shape"] == shape for item in models)
        for shape in ("batch_and_step", "batch_only", "step_only", "catalog_only")
    }
    return {
        "contribution_id": contribution_id,
        "plugin_id": plugin_id,
        "provider_id": provider_id,
        "provider_version": provider_version,
        "contract_profile": contract_profile,
        "status": "pass" if not errors else "fail",
        "runner_type": None if runner is None else type(runner).__name__,
        "advertisement_audit_status": audit_status,
        "model_count": len(models),
        "registered_batch_tuple_count": registered_batch_tuple_count,
        "registered_step_tuple_count": registered_step_tuple_count,
        "capability_shape_counts": capability_shape_counts,
        "default_execution_enabled": execute_defaults,
        "default_execution_count": sum(item["status"] == "pass" for item in executed_defaults.values()),
        "default_batch_execution_enabled": execute_batch_defaults,
        "default_batch_execution_count": sum(
            item["status"] == "pass" and item.get("batch_executed") is True
            for item in executed_defaults.values()
        ),
        "public_trajectory_contract": public_contract,
        "models": models,
        "errors": errors,
    }
    ####


def build_report(
    *,
    plugin_ids: Iterable[str],
    contract_profile: ContractProfile = "interoperable",
    include_external: bool = False,
    execute_defaults: bool = False,
    execute_batch_defaults: bool = False,
    catalog: PluginCatalog | None = None,
) -> dict[str, Any]:
    """Build a profile-specific report limited to selected plug-in ownership."""

    if contract_profile not in _CONTRACT_PROFILES:
        raise ValueError(f"unknown provider contract profile {contract_profile!r}")
    if execute_batch_defaults:
        execute_defaults = True
    selected = _normalize_plugin_ids(plugin_ids)
    if catalog is None:
        catalog = discover_plugins(
            include_external=include_external,
            selected=selected,
            strict=False,
        )
    errors: list[str] = []
    standard_ecef = _standard_ecef_contract()
    errors.extend(str(item) for item in standard_ecef["errors"])
    loaded_plugin_ids = {item.id for item in catalog.plugins}
    missing_plugins = tuple(plugin_id for plugin_id in selected if plugin_id not in loaded_plugin_ids)
    errors.extend(f"selected plug-in did not load: {plugin_id}" for plugin_id in missing_plugins)
    catalog_diagnostics: list[dict[str, str]] = []
    for diagnostic in catalog.diagnostics:
        if diagnostic.plugin_id not in selected:
            continue
        catalog_diagnostics.append(
            {
                "plugin_id": diagnostic.plugin_id,
                "status": diagnostic.status,
                "message": diagnostic.message,
            }
        )
        if diagnostic.status != "loaded":
            errors.append(f"plug-in discovery {diagnostic.plugin_id}: {diagnostic.status}: {diagnostic.message}")

    records = tuple(
        item
        for item in catalog.records("mission_composition_provider")
        if item.plugin.id in selected
    )
    providers: list[dict[str, Any]] = []
    if records:
        try:
            registry = catalog.build_mission_composition_provider_registry()
        except Exception as error:  # noqa: BLE001 - report one scoped registry construction error
            errors.append(f"Mission Composition registry construction failed: {type(error).__name__}: {error}")
            registry = None
        for record in records:
            if registry is None:
                providers.append(
                    {
                        "contribution_id": record.id,
                        "plugin_id": record.plugin.id,
                        "provider_id": None,
                        "provider_version": None,
                        "contract_profile": contract_profile,
                        "status": "fail",
                        "runner_type": None,
                        "advertisement_audit_status": "not_run",
                        "model_count": 0,
                        "registered_batch_tuple_count": 0,
                        "registered_step_tuple_count": 0,
                        "capability_shape_counts": {},
                        "models": [],
                        "errors": ["Mission Composition registry construction failed"],
                    }
                )
                continue
            try:
                provider = registry.provider(record.id)
            except Exception as error:  # noqa: BLE001 - preserve provider registration failures
                providers.append(
                    {
                        "contribution_id": record.id,
                        "plugin_id": record.plugin.id,
                        "provider_id": None,
                        "provider_version": None,
                        "contract_profile": contract_profile,
                        "status": "fail",
                        "runner_type": None,
                        "advertisement_audit_status": "not_run",
                        "model_count": 0,
                        "registered_batch_tuple_count": 0,
                        "registered_step_tuple_count": 0,
                        "capability_shape_counts": {},
                        "models": [],
                        "errors": [f"provider registry lookup failed: {type(error).__name__}: {error}"],
                    }
                )
                continue
            providers.append(
                _provider_report(
                    contribution_id=record.id,
                    plugin_id=record.plugin.id,
                    provider=provider,
                    contract_profile=contract_profile,
                    execute_defaults=execute_defaults,
                    execute_batch_defaults=execute_batch_defaults,
                )
            )
    errors.extend(
        f"provider {provider['contribution_id']}: {error}"
        for provider in providers
        for error in provider["errors"]
    )
    plugin_reports = [
        {
            "plugin_id": plugin_id,
            "loaded": plugin_id in loaded_plugin_ids,
            "provider_ids": [provider["contribution_id"] for provider in providers if provider["plugin_id"] == plugin_id],
            "provider_count": sum(provider["plugin_id"] == plugin_id for provider in providers),
        }
        for plugin_id in selected
    ]
    return {
        "schema": "taoryx.mission-composition-provider-contract/v1",
        "status": "pass" if not errors else "fail",
        "contract_profile": contract_profile,
        "plugin_ids": list(selected),
        "plugins": plugin_reports,
        "catalog_diagnostics": catalog_diagnostics,
        "standard_ecef_contract": standard_ecef,
        "provider_count": len(providers),
        "model_count": sum(int(provider["model_count"]) for provider in providers),
        "registered_batch_tuple_count": sum(int(provider["registered_batch_tuple_count"]) for provider in providers),
        "registered_step_tuple_count": sum(int(provider["registered_step_tuple_count"]) for provider in providers),
        "default_execution_enabled": execute_defaults,
        "default_execution_count": sum(int(provider.get("default_execution_count", 0)) for provider in providers),
        "default_batch_execution_enabled": execute_batch_defaults,
        "default_batch_execution_count": sum(
            int(provider.get("default_batch_execution_count", 0)) for provider in providers
        ),
        "capability_shape_counts": {
            shape: sum(int(provider["capability_shape_counts"].get(shape, 0)) for provider in providers)
            for shape in ("batch_and_step", "batch_only", "step_only", "catalog_only")
        },
        "error_count": len(errors),
        "errors": errors,
        "providers": providers,
        "claim_boundary": (
            "This is a selected-plug-in structural and provider-construction gate. It verifies the common "
            "ECEF result/session types, metadata schemas, exact advertised operation seams, and a preparable "
            "provider-selected runnable default for every model. The "
            "taoryx_universal profile additionally requires matching batch and step tuples for every "
            "host-facing model. --execute-defaults additionally runs each selected model through one public "
            "streaming step. --execute-batch-defaults also runs the full public batch, which may be intentionally "
            "long for a transport-scale witness; use check-vehicle for a family-specific vertical evidence ladder."
        ),
    }
    ####


def _summary(report: dict[str, Any]) -> dict[str, Any]:
    """Return the concise payload used by inner-loop development commands."""

    return {
        "schema": report["schema"],
        "status": report["status"],
        "contract_profile": report["contract_profile"],
        "plugin_ids": report["plugin_ids"],
        "provider_count": report["provider_count"],
        "model_count": report["model_count"],
        "registered_batch_tuple_count": report["registered_batch_tuple_count"],
        "registered_step_tuple_count": report["registered_step_tuple_count"],
        "default_execution_enabled": report["default_execution_enabled"],
        "default_execution_count": report["default_execution_count"],
        "default_batch_execution_enabled": report["default_batch_execution_enabled"],
        "default_batch_execution_count": report["default_batch_execution_count"],
        "capability_shape_counts": report["capability_shape_counts"],
        "error_count": report["error_count"],
        "errors": report["errors"],
    }
    ####


def main(argv: Sequence[str] | None = None) -> int:
    """Run one scoped interoperable or TAORYX-universal provider-contract gate."""

    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument(
        "--plugin",
        action="append",
        metavar="PLUGIN_ID",
        help="validate one source-tree plug-in; repeat to select several",
    )
    selection.add_argument("--all", action="store_true", help="validate every source-tree plug-in for the release gate")
    parser.add_argument(
        "--contract-profile",
        choices=("interoperable", "taoryx-universal"),
        default="interoperable",
        help="interoperable accepts honest batch-only or step-only providers; taoryx-universal requires both",
    )
    parser.add_argument(
        "--include-external",
        action="store_true",
        help="include installed third-party entry points when validating named plug-ins",
    )
    parser.add_argument(
        "--execute-defaults",
        action="store_true",
        help="run each selected provider-selected default through one public streaming step",
    )
    parser.add_argument(
        "--execute-batch-defaults",
        action="store_true",
        help="also run the full public batch for each selected default; implies --execute-defaults",
    )
    parser.add_argument("--summary", action="store_true", help="print counts and findings instead of each model tuple")
    args = parser.parse_args(argv)
    if args.all:
        if args.include_external:
            parser.error("--all is the source-tree release scope and cannot be combined with --include-external")
        from taoryx.builtin_plugins import source_plugin_entry_points

        plugin_ids = tuple(item.name for item in source_plugin_entry_points())
    else:
        plugin_ids = args.plugin
    contract_profile: ContractProfile = "taoryx_universal" if args.contract_profile == "taoryx-universal" else "interoperable"
    report = build_report(
        plugin_ids=plugin_ids,
        contract_profile=contract_profile,
        include_external=args.include_external,
        execute_defaults=args.execute_defaults,
        execute_batch_defaults=args.execute_batch_defaults,
    )
    print(json.dumps(_summary(report) if args.summary else report, indent=2, sort_keys=True))
    return 0 if report["status"] == "pass" else 2
    ####


if __name__ == "__main__":
    raise SystemExit(main())
