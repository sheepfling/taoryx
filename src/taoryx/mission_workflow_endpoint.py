"""Focused vertical verification for non-vehicle Mission Composition workflows.

Physical vehicle families use :mod:`taoryx.vehicle_endpoint_spec`, which joins
their composition registry, execution binding, and native result packet.  A
trajectory workflow has no such physical-family composition entry.  This
module gives those registered models the equivalent honest vertical seam:
checked-in authored draft, exact provider validation, declared common runner,
and normalized result evidence.
"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import TYPE_CHECKING, Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .model_authoring import (
    ModelAuthoringError,
    compile_model_authoring_draft,
    load_model_authoring_draft,
    run_prepared_mission_composition,
)
from .plugins import discover_plugins
from .plugins.resources import packaged_resource_fallback
from .trajectory.execution_contract import MissionCompositionOutputSelection, RunnableMissionCompositionProvider
from .vehicle_registry import ROOT

if TYPE_CHECKING:
    from .plugins import PluginCatalog

MISSION_WORKFLOW_ENDPOINT_SPECS = packaged_resource_fallback(
    ROOT / "verification/mission_workflow_endpoint_specs.yaml",
    package="taoryx_reference_models",
    resource="data/verification/mission_workflow_endpoint_specs.yaml",
)


class MissionWorkflowEndpointEventSpec(BaseModel):
    """One observable lifecycle event required by a workflow endpoint."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    category: str = Field(min_length=1)
    kind: str = Field(min_length=1)


class MissionWorkflowEndpointSpec(BaseModel):
    """One executable, provider-owned workflow endpoint contract."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    provider_id: str = Field(min_length=1)
    model_id: str = Field(min_length=1)
    model_kind: Literal["trajectory_workflow", "composition_proof_family"]
    fidelity: str = Field(min_length=1)
    realization_id: str = Field(min_length=1)
    mission_template_id: str = Field(min_length=1)
    draft: str = Field(min_length=1)
    batch_executor_id: str = Field(min_length=1)
    blocked_operations: tuple[Literal["step"], ...] = ("step",)
    required_core_output_ids: tuple[str, ...] = Field(min_length=1)
    required_telemetry_output_ids: tuple[str, ...] = ()
    required_events: tuple[MissionWorkflowEndpointEventSpec, ...] = ()
    expected_primary_object_count: int = Field(default=1, ge=1)
    maximum_samples_per_object: int = Field(default=3, ge=1)
    robustness_disposition: Literal["not_applicable"] = "not_applicable"
    robustness_reason: str = Field(min_length=1)
    maturity_record_id: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_endpoint_shape(self) -> MissionWorkflowEndpointSpec:
        if Path(self.draft).is_absolute() or ".." in Path(self.draft).parts:
            raise ValueError(f"workflow endpoint {self.id!r} draft must be a relative checked-in path")
        output_ids = (*self.required_core_output_ids, *self.required_telemetry_output_ids)
        if len(output_ids) != len(set(output_ids)):
            raise ValueError(f"workflow endpoint {self.id!r} repeats a required output ID")
        if any(not item.strip() for item in output_ids):
            raise ValueError(f"workflow endpoint {self.id!r} has a blank required output ID")
        event_keys = tuple((item.category, item.kind) for item in self.required_events)
        if len(event_keys) != len(set(event_keys)):
            raise ValueError(f"workflow endpoint {self.id!r} repeats a required event")
        return self
        ####

    ####


class MissionWorkflowEndpointCatalog(BaseModel):
    """Versioned catalog of focused, non-physical workflow endpoint proofs."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_id: Literal["taoryx.mission-workflow-endpoint-specs/v1alpha1"] = Field(alias="schema")
    version: int = Field(ge=1)
    description: str = Field(min_length=1)
    endpoints: tuple[MissionWorkflowEndpointSpec, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_unique_ids(self) -> MissionWorkflowEndpointCatalog:
        ids = tuple(item.id for item in self.endpoints)
        if len(ids) != len(set(ids)):
            raise ValueError("mission workflow endpoint catalog has duplicate IDs")
        identities = tuple((item.provider_id, item.model_id, item.mission_template_id, item.fidelity) for item in self.endpoints)
        if len(identities) != len(set(identities)):
            raise ValueError("mission workflow endpoint catalog has duplicate provider/model/mission/fidelity identities")
        return self
        ####

    def endpoint(self, identifier: str) -> MissionWorkflowEndpointSpec:
        """Resolve one exact workflow endpoint contract."""

        match = next((item for item in self.endpoints if item.id == identifier), None)
        if match is None:
            raise KeyError(f"unknown mission workflow endpoint {identifier!r}")
        return match
        ####

    ####


def load_mission_workflow_endpoint_catalog(path: str | Path | None = None) -> MissionWorkflowEndpointCatalog:
    """Load the checked-in workflow endpoint catalog."""

    source = Path(path) if path is not None else MISSION_WORKFLOW_ENDPOINT_SPECS
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{source} must contain a mapping")
    return MissionWorkflowEndpointCatalog.model_validate(payload)
    ####


def mission_workflow_endpoint_list() -> dict[str, object]:
    """Return the discoverable workflow endpoint inventory without execution."""

    catalog = load_mission_workflow_endpoint_catalog()
    return {
        "schema": "taoryx.mission-workflow-endpoint-list/v1alpha1",
        "endpoints": [
            {
                "id": item.id,
                "provider_id": item.provider_id,
                "model_id": item.model_id,
                "fidelity": item.fidelity,
                "realization_id": item.realization_id,
                "mission_template_id": item.mission_template_id,
                "draft": item.draft,
                "blocked_operations": list(item.blocked_operations),
                "maturity_record_id": item.maturity_record_id,
            }
            for item in catalog.endpoints
        ],
        "claim_boundary": (
            "This lists workflow endpoint contracts. It does not validate an installed provider or execute a workflow."
        ),
    }
    ####


def verify_mission_workflow_endpoint(
    identifier: str,
    *,
    execute: bool = False,
    catalog: MissionWorkflowEndpointCatalog | None = None,
    plugins: PluginCatalog | None = None,
) -> dict[str, object]:
    """Verify one exact authored workflow through the public provider contract."""

    endpoint = (catalog or load_mission_workflow_endpoint_catalog()).endpoint(identifier)
    installed_plugins = plugins or discover_plugins()
    providers = installed_plugins.build_mission_composition_provider_registry()
    errors: list[str] = []
    records: dict[str, object] = {}
    draft_path = _workflow_draft_path(endpoint.draft)

    try:
        draft = load_model_authoring_draft(draft_path)
    except (OSError, ModelAuthoringError, ValueError) as error:
        errors.append(f"endpoint {endpoint.id}: cannot load draft: {error}")
        records["draft"] = {"status": "fail", "path": endpoint.draft, "detail": str(error)}
        return _verification_report(endpoint, execute=execute, errors=errors, records=records)

    draft_identity = {
        "provider_id": draft.provider_id,
        "model_id": draft.model_id,
        "fidelity": draft.fidelity,
        "realization_id": draft.realization_id,
        "mission_template_id": draft.mission_template_id,
    }
    expected_identity = {
        "provider_id": endpoint.provider_id,
        "model_id": endpoint.model_id,
        "fidelity": endpoint.fidelity,
        "realization_id": endpoint.realization_id,
        "mission_template_id": endpoint.mission_template_id,
    }
    records["draft"] = {
        "status": "pass" if draft_identity == expected_identity else "fail",
        "path": endpoint.draft,
        "identity": draft_identity,
        "unresolved_inputs": list(draft.unresolved_inputs),
    }
    if draft_identity != expected_identity:
        errors.append(f"endpoint {endpoint.id}: checked-in draft identity disagrees with endpoint declaration")
    if draft.unresolved_inputs:
        errors.append(f"endpoint {endpoint.id}: checked-in draft still has unresolved inputs")

    try:
        provider = providers.provider(endpoint.provider_id)
        model = providers.model(endpoint.provider_id, endpoint.model_id)
    except KeyError as error:
        errors.append(f"endpoint {endpoint.id}: provider/model is not installed: {error}")
        records["advertisement"] = {"status": "fail", "detail": str(error)}
        return _verification_report(endpoint, execute=execute, errors=errors, records=records)

    advertised_outputs = {item.id for item in model.output_schema.channels}
    required_outputs = (*endpoint.required_core_output_ids, *endpoint.required_telemetry_output_ids)
    missing_advertised_outputs = sorted(set(required_outputs) - advertised_outputs)
    records["advertisement"] = {
        "status": "pass" if model.model_kind == endpoint.model_kind and not missing_advertised_outputs else "fail",
        "provider_version": provider.metadata.version,
        "model_version": model.version,
        "model_kind": model.model_kind,
        "output_schema_fingerprint": model.output_schema_fingerprint,
        "missing_required_output_ids": missing_advertised_outputs,
    }
    if model.model_kind != endpoint.model_kind:
        errors.append(f"endpoint {endpoint.id}: model kind {model.model_kind!r} is not {endpoint.model_kind!r}")
    if missing_advertised_outputs:
        errors.append(f"endpoint {endpoint.id}: output advertisement misses {missing_advertised_outputs!r}")

    try:
        prepared = compile_model_authoring_draft(providers, draft)
    except (KeyError, ModelAuthoringError, ValueError) as error:
        errors.append(f"endpoint {endpoint.id}: draft compilation/validation failed: {error}")
        records["configuration_preflight"] = {"status": "fail", "detail": str(error)}
        return _verification_report(endpoint, execute=execute, errors=errors, records=records)
    records["configuration_preflight"] = {
        "status": "pass",
        "configuration_fingerprint": prepared.fingerprint,
        "schema_fingerprint": prepared.configuration.schema_fingerprint,
    }

    mission = next((item for item in model.mission_templates if item.id == endpoint.mission_template_id), None)
    batch_operation = (
        next(
            (
                item
                for item in mission.operations
                if item.fidelity == endpoint.fidelity
                and item.realization_id == endpoint.realization_id
                and item.operation == "batch"
            ),
            None,
        )
        if mission is not None
        else None
    )
    blocked_operations = {
        item.operation: item
        for item in (mission.operations if mission is not None else ())
        if item.fidelity == endpoint.fidelity and item.realization_id == endpoint.realization_id
    }
    runner_registered = False
    runner_detail: str | None = None
    if isinstance(provider, RunnableMissionCompositionProvider):
        runner_registered = provider.build_runner().has_executor(endpoint.provider_id, endpoint.model_id)
    else:
        runner_detail = "provider does not implement RunnableMissionCompositionProvider"
    missing_blocked_operations = [
        operation
        for operation in endpoint.blocked_operations
        if operation not in blocked_operations
        or blocked_operations[operation].status != "blocked"
        or blocked_operations[operation].common_runner_status != "not_available"
    ]
    runner_ok = (
        batch_operation is not None
        and batch_operation.status == "available"
        and batch_operation.common_runner_status == "registered"
        and batch_operation.executor_id == endpoint.batch_executor_id
        and runner_registered
        and not missing_blocked_operations
    )
    records["runner"] = {
        "status": "pass" if runner_ok else "fail",
        "registered": runner_registered,
        "batch_operation": None if batch_operation is None else batch_operation.model_dump(mode="json"),
        "missing_blocked_operations": missing_blocked_operations,
        "detail": runner_detail,
    }
    if not runner_ok:
        errors.append(f"endpoint {endpoint.id}: common batch runner advertisement/registration is incomplete")

    if execute:
        response = run_prepared_mission_composition(
            providers,
            endpoint.provider_id,
            prepared,
            request_id=f"workflow-endpoint-{endpoint.id}",
            output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=endpoint.maximum_samples_per_object),
        )
        if response.kind != "trajectory":
            errors.append(f"endpoint {endpoint.id}: batch execution returned a failure response")
            records["execution"] = {"status": "fail", "response": response.model_dump(mode="json", by_alias=True)}
        else:
            primary = response.result.objects[0] if response.result.objects else None
            emitted_outputs = set() if primary is None else {item.id for item in primary.channels}
            missing_emitted_outputs = sorted(set(required_outputs) - emitted_outputs)
            events = {(item.category, item.kind) for item in response.result.events}
            missing_events = [
                item.model_dump(mode="json")
                for item in endpoint.required_events
                if (item.category, item.kind) not in events
            ]
            execution_ok = (
                response.result.status == "completed"
                and len(response.result.objects) == endpoint.expected_primary_object_count
                and not missing_emitted_outputs
                and not missing_events
            )
            records["execution"] = {
                "status": "pass" if execution_ok else "fail",
                "result_status": response.result.status,
                "primary_model_id": response.result.primary_model_id,
                "object_count": len(response.result.objects),
                "missing_required_output_ids": missing_emitted_outputs,
                "missing_required_events": missing_events,
                "event_count": len(response.result.events),
            }
            if not execution_ok:
                errors.append(f"endpoint {endpoint.id}: normalized batch result misses declared evidence")
    else:
        records["execution"] = {"status": "not_requested"}

    records["robustness"] = {
        "status": "not_applicable",
        "reason": endpoint.robustness_reason,
    }
    return _verification_report(endpoint, execute=execute, errors=errors, records=records)
    ####


def _workflow_draft_path(relative_path: str) -> Path:
    """Resolve a checked-in witness from source or the reference-model package."""

    return packaged_resource_fallback(
        ROOT / relative_path,
        package="taoryx_reference_models",
        resource=f"data/{relative_path}",
    )
    ####


def _verification_report(
    endpoint: MissionWorkflowEndpointSpec,
    *,
    execute: bool,
    errors: list[str],
    records: Mapping[str, object],
) -> dict[str, object]:
    """Build one stable public report without widening a workflow claim."""

    return {
        "schema": "taoryx.mission-workflow-endpoint-verification/v1alpha1",
        "endpoint": endpoint.model_dump(mode="json"),
        "status": "pass" if not errors else "fail",
        "execution_requested": execute,
        "records": dict(records),
        "errors": list(errors),
        "claim_boundary": (
            "This verifies one checked-in workflow draft through its advertised provider validation and common batch runner. "
            "It does not promote a workflow into a physical vehicle-composition claim, establish robustness, or qualify a mission."
        ),
    }
    ####


__all__ = [
    "MISSION_WORKFLOW_ENDPOINT_SPECS",
    "MissionWorkflowEndpointCatalog",
    "MissionWorkflowEndpointEventSpec",
    "MissionWorkflowEndpointSpec",
    "load_mission_workflow_endpoint_catalog",
    "mission_workflow_endpoint_list",
    "verify_mission_workflow_endpoint",
]
