"""Typed vertical contracts and focused verification for vehicle endpoints.

An endpoint is the smallest useful unit a vehicle plug-in author should need
to reason about: one family, mission, fidelity, control/tuning seam, public
result surface, and executable witness.  The historical catalogs remain the
authorities during migration, while this contract makes every cross-catalog
join explicit and verifiable through one bounded endpoint command.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import tempfile
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Literal, TypeGuard

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .composition_episode import open_vehicle_composition_episode
from .composition_result_catalog import index_composition_results
from .family_adapter import AdapterOperation
from .fidelity_contracts import FidelityTier
from .plugins import discover_plugins
from .trajectory.native_output_contract import NativeOutputBinding, extract_native_channel, native_output_bindings
from .tuning_application import RuntimeTuningBindingReceipt, TuningApplicationContextSet
from .vehicle_batch_execution import VehicleBatchExecution, execute_vehicle_composition_batch
from .vehicle_catalog_resources import vehicle_catalog_resources
from .vehicle_composition import CompiledVehicleComposition, compile_vehicle_composition, load_vehicle_composition_request
from .vehicle_composition_registry import ResolvedVehicleCompositionCatalog, load_resolved_vehicle_composition_catalog
from .vehicle_execution_bindings import (
    ExecutionMode,
    ExecutionOperation,
    VehicleExecutionBinding,
    load_vehicle_execution_binding_catalog,
    resolve_vehicle_execution_binding,
)
from .vehicle_execution_preflight import preflight_vehicle_composition, validate_public_capability_advertisement
from .vehicle_execution_witnesses import load_vehicle_execution_witness_catalog
from .vehicle_interface import resolve_vehicle_interface_contract, validate_vehicle_interface_contract
from .vehicle_registry import ROOT

_CONTROLLER_TUNING_PROVENANCE_FILENAME = "controller_tuning_provenance.json"

# Materialized lazily by ``__getattr__`` only for compatibility consumers.
VEHICLE_ENDPOINT_SPECS: Path
VEHICLE_MATURITY_REGISTRY: Path


class VehicleEndpointOperationSpec(BaseModel):
    """One exact public operation a vertical endpoint must retain."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    operation: ExecutionOperation
    factory_id: str = Field(min_length=1)
    witness_id: str = Field(min_length=1)


class VehicleEndpointRobustnessCaseSpec(BaseModel):
    """One declared mass, bias, or wind case for an endpoint screen."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    parameters: dict[str, float] = Field(default_factory=dict)
    description: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_case_shape(self) -> VehicleEndpointRobustnessCaseSpec:
        if any(not key.strip() for key in self.parameters):
            raise ValueError(f"robustness case {self.id!r} has an empty parameter name")
        if any(not _finite_number(value) for value in self.parameters.values()):
            raise ValueError(f"robustness case {self.id!r} parameters must be finite")
        return self
        ####


class VehicleEndpointRobustnessMetricSpec(BaseModel):
    """One numeric metric and its explicit pass interval for a screen case."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    minimum: float | None = None
    maximum: float | None = None
    unit: str = Field(min_length=1)
    description: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_metric_shape(self) -> VehicleEndpointRobustnessMetricSpec:
        if self.minimum is None and self.maximum is None:
            raise ValueError(f"robustness metric {self.id!r} needs a minimum and/or maximum threshold")
        if self.minimum is not None and not _finite_number(self.minimum):
            raise ValueError(f"robustness metric {self.id!r} minimum must be finite")
        if self.maximum is not None and not _finite_number(self.maximum):
            raise ValueError(f"robustness metric {self.id!r} maximum must be finite")
        if self.minimum is not None and self.maximum is not None and self.minimum > self.maximum:
            raise ValueError(f"robustness metric {self.id!r} has an inverted threshold interval")
        return self
        ####


class VehicleEndpointRobustnessScreenSpec(BaseModel):
    """A declarative endpoint-level disturbance or mass-screen contract."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    kind: Literal["mass_variation", "constant_offset", "wind_bias"]
    artifact_filename: str = Field(min_length=1)
    execution_readiness: Literal["planned", "available", "blocked"]
    availability_reason: str | None = None
    cases: tuple[VehicleEndpointRobustnessCaseSpec, ...] = Field(min_length=1)
    metrics: tuple[VehicleEndpointRobustnessMetricSpec, ...] = Field(min_length=1)
    required_for_finalization: bool = True
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_screen_shape(self) -> VehicleEndpointRobustnessScreenSpec:
        filename = Path(self.artifact_filename)
        if filename.name != self.artifact_filename or filename.suffix != ".json":
            raise ValueError(f"robustness screen {self.id!r} artifact filename must be one JSON basename")
        case_ids = tuple(item.id for item in self.cases)
        metric_ids = tuple(item.id for item in self.metrics)
        if len(case_ids) != len(set(case_ids)):
            raise ValueError(f"robustness screen {self.id!r} has duplicate case IDs")
        if len(metric_ids) != len(set(metric_ids)):
            raise ValueError(f"robustness screen {self.id!r} has duplicate metric IDs")
        if self.execution_readiness == "blocked" and not self.availability_reason:
            raise ValueError(f"blocked robustness screen {self.id!r} needs an availability reason")
        return self
    ####


class VehicleEndpointRobustnessRequirement(BaseModel):
    """State whether a runnable endpoint owns a robustness evidence seam.

    A nonphysical local-control screen can be complete at its declared
    fidelity without exposing a mass, wind, or persistent-offset input.  The
    disposition makes that absence explicit and reviewable; an empty screen
    list can never silently mean that robustness was forgotten.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    disposition: Literal["required", "not_applicable"] = "required"
    reason: str | None = None

    @model_validator(mode="after")
    def validate_requirement_shape(self) -> VehicleEndpointRobustnessRequirement:
        if self.disposition == "not_applicable" and (self.reason is None or not self.reason.strip()):
            raise ValueError("a not_applicable robustness requirement needs a nonblank reason")
        if self.disposition == "required" and self.reason is not None:
            raise ValueError("a required robustness requirement must not carry a not-applicable reason")
        return self
        ####


class VehicleEndpointSpec(BaseModel):
    """Single author-facing declaration joining one runnable vehicle endpoint."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    family_id: str = Field(min_length=1)
    model_id: str = Field(min_length=1)
    provider_ids: tuple[str, ...] = Field(default=("taoryx.registry.mission-composition",), min_length=1)
    mission_id: str = Field(min_length=1)
    fidelity: FidelityTier
    execution_mode: ExecutionMode
    operations: tuple[VehicleEndpointOperationSpec, ...] = Field(min_length=1)
    controller_campaign_id: str | None = None
    # These are capabilities used to build a campaign candidate.  They are
    # intentionally distinct from the physical allocation exercised by the
    # batch runtime, which is independently checked from the committed control
    # trace by the result catalog.
    required_tuning_operations: tuple[AdapterOperation, ...] = ()
    required_core_output_ids: tuple[str, ...] = ()
    required_telemetry_output_ids: tuple[str, ...] = ()
    robustness_requirement: VehicleEndpointRobustnessRequirement = Field(
        default_factory=VehicleEndpointRobustnessRequirement
    )
    robustness_screens: tuple[VehicleEndpointRobustnessScreenSpec, ...] = ()
    maturity_record_id: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_endpoint_shape(self) -> VehicleEndpointSpec:
        operations = tuple(item.operation for item in self.operations)
        if len(operations) != len(set(operations)):
            raise ValueError(f"endpoint {self.id!r} has duplicate operations")
        if len(self.provider_ids) != len(set(self.provider_ids)) or any(not identifier.strip() for identifier in self.provider_ids):
            raise ValueError(f"endpoint {self.id!r} needs unique nonblank provider IDs")
        if "batch" not in operations:
            raise ValueError(f"endpoint {self.id!r} needs one batch operation for typed evidence artifacts")
        required_outputs = (*self.required_core_output_ids, *self.required_telemetry_output_ids)
        if len(required_outputs) != len(set(required_outputs)):
            raise ValueError(f"endpoint {self.id!r} repeats a required output ID")
        if any(not identifier.strip() for identifier in required_outputs):
            raise ValueError(f"endpoint {self.id!r} has an empty required output ID")
        if len(self.required_tuning_operations) != len(set(self.required_tuning_operations)):
            raise ValueError(f"endpoint {self.id!r} repeats a required tuning operation")
        if self.controller_campaign_id is None and self.required_tuning_operations:
            raise ValueError(f"endpoint {self.id!r} names tuning operations without a controller campaign")
        if self.controller_campaign_id is not None and not self.required_tuning_operations:
            raise ValueError(f"endpoint {self.id!r} needs required tuning operations for its controller campaign")
        robustness_ids = tuple(item.id for item in self.robustness_screens)
        robustness_artifacts = tuple(item.artifact_filename for item in self.robustness_screens)
        if self.robustness_requirement.disposition == "required" and not self.robustness_screens:
            raise ValueError(f"endpoint {self.id!r} requires at least one robustness screen")
        if self.robustness_requirement.disposition == "not_applicable" and self.robustness_screens:
            raise ValueError(f"endpoint {self.id!r} cannot combine a not-applicable robustness requirement with screens")
        if len(robustness_ids) != len(set(robustness_ids)):
            raise ValueError(f"endpoint {self.id!r} has duplicate robustness screen IDs")
        if len(robustness_artifacts) != len(set(robustness_artifacts)):
            raise ValueError(f"endpoint {self.id!r} reuses a robustness artifact filename")
        if self.execution_mode == "planned":
            raise ValueError("a VehicleEndpointSpec must name an executable rather than planned endpoint")
        return self
        ####


class VehicleEndpointSpecCatalog(BaseModel):
    """Versioned source of focused vertical endpoint contracts."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_id: str = Field(alias="schema", min_length=1)
    version: int = Field(ge=1)
    description: str = Field(min_length=1)
    endpoints: tuple[VehicleEndpointSpec, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_unique_endpoint_ids(self) -> VehicleEndpointSpecCatalog:
        identifiers = tuple(item.id for item in self.endpoints)
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("vehicle endpoint spec catalog has duplicate endpoint IDs")
        identities = tuple((item.family_id, item.mission_id, item.fidelity) for item in self.endpoints)
        if len(identities) != len(set(identities)):
            raise ValueError("vehicle endpoint spec catalog has duplicate family/mission/fidelity identities")
        return self
        ####

    def endpoint(self, identifier: str) -> VehicleEndpointSpec:
        """Resolve one exact vertical endpoint contract."""

        match = next((item for item in self.endpoints if item.id == identifier), None)
        if match is None:
            raise KeyError(f"unknown vehicle endpoint spec {identifier!r}")
        return match
        ####


def load_vehicle_endpoint_spec_catalog(path: str | Path | None = None) -> VehicleEndpointSpecCatalog:
    """Load the explicit plug-in authoring contracts for focused endpoints."""

    if path is not None:
        source = Path(path)
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError(f"{source} must contain a mapping")
        return VehicleEndpointSpecCatalog.model_validate(payload)
    payloads = _catalog_payloads("verification/vehicle_endpoint_specs.yaml")
    return VehicleEndpointSpecCatalog.model_validate(_merge_list_catalog(payloads, "endpoints"))
    ####


def _catalog_payloads(relative_path: str) -> list[Mapping[str, object]]:
    """Read every installed non-overlapping vehicle endpoint fragment."""

    payloads: list[Mapping[str, object]] = []
    for source in vehicle_catalog_resources(relative_path):
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError(f"{source} must contain a mapping")
        payloads.append(payload)
    return payloads
    ####


def _merge_list_catalog(payloads: list[Mapping[str, object]], key: str) -> dict[str, object]:
    """Join same-schema plug-in fragments before typed validation."""

    if not payloads:
        raise ValueError("vehicle endpoint catalog has no installed fragments")
    merged = dict(payloads[0])
    rows: list[object] = []
    for payload in payloads:
        value = payload.get(key)
        if not isinstance(value, list):
            raise ValueError(f"vehicle endpoint catalog {key!r} must contain a list")
        rows.extend(value)
    merged[key] = rows
    return merged
    ####


def _maturity_records() -> tuple[Mapping[str, object], ...]:
    """Return the merged family-ledger records owned by installed packages."""

    records: list[Mapping[str, object]] = []
    for payload in _catalog_payloads("verification/vehicle_maturity_registry.yaml"):
        if payload.get("registry_id") != "taoryx_vehicle_maturity_v1":
            raise ValueError("invalid vehicle maturity registry")
        fragment_records = payload.get("records")
        if not isinstance(fragment_records, list):
            raise ValueError("vehicle maturity registry must contain records")
        records.extend(item for item in fragment_records if isinstance(item, Mapping))
    identifiers = [item.get("id") for item in records]
    if len(identifiers) != len(set(identifiers)):
        raise ValueError("vehicle maturity registry has duplicate record IDs across plug-in fragments")
    return tuple(records)
    ####


def vehicle_endpoint_spec_list() -> dict[str, object]:
    """Return compact discovery records without compiling or executing a vehicle."""

    catalog = load_vehicle_endpoint_spec_catalog()
    return {
        "schema": "taoryx.vehicle-endpoint-spec-list/v1alpha1",
        "endpoints": [
            {
                "id": item.id,
                "family_id": item.family_id,
                "model_id": item.model_id,
                "mission_id": item.mission_id,
                "fidelity": item.fidelity,
                "operations": [operation.operation for operation in item.operations],
                "controller_campaign_id": item.controller_campaign_id,
                "required_tuning_operations": list(item.required_tuning_operations),
                "robustness_requirement": item.robustness_requirement.model_dump(mode="json"),
                "robustness_screens": [screen.model_dump(mode="json") for screen in item.robustness_screens],
                "maturity_record_id": item.maturity_record_id,
            }
            for item in catalog.endpoints
        ],
        "claim_boundary": (
            "This lists focused vertical contracts. It does not compile, execute, tune, or qualify an endpoint."
        ),
    }
    ####


def verify_vehicle_endpoint(
    identifier: str,
    *,
    execute: bool = False,
    tune: bool = False,
    cache_dir: str | Path | None = None,
    results_dir: str | Path | None = None,
    catalog: VehicleEndpointSpecCatalog | None = None,
    composition_catalog: ResolvedVehicleCompositionCatalog | None = None,
) -> dict[str, object]:
    """Verify one exact endpoint across composition, runtime, output, and tuning seams.

    The default path is deliberately focused: it compiles the endpoint's
    checked-in witness and runs semantic preflight.  ``execute`` opts into the
    real batch factory; ``tune`` opts into the campaign runner and its
    content-addressed cache. ``results_dir`` retains the exact batch packet,
    including a host-written controller/tuning provenance sidecar. Neither
    option promotes the endpoint beyond its declared claim boundary.
    """

    if results_dir is not None and not execute:
        raise ValueError("vehicle endpoint results_dir requires execute=True")
    verification_started = time.perf_counter()
    phase_durations_s: dict[str, float] = {}
    setup_started = time.perf_counter()
    endpoint_catalog = catalog or load_vehicle_endpoint_spec_catalog()
    endpoint = endpoint_catalog.endpoint(identifier)
    vehicle_catalog = composition_catalog or load_resolved_vehicle_composition_catalog()
    errors: list[str] = []
    records: dict[str, object] = {}
    retained_results_root = _prepare_endpoint_results_directory(results_dir) if results_dir is not None else None
    if retained_results_root is not None:
        records["retained_results_directory"] = str(retained_results_root)
    phase_durations_s["catalog_and_result_setup_s"] = _elapsed_s(setup_started)

    selection_started = time.perf_counter()
    selection_findings: list[str] = []
    try:
        vehicle = vehicle_catalog.vehicle(endpoint.family_id)
    except (KeyError, ValueError) as error:
        detail = str(error)
        errors.append(detail)
        selection_findings.append(detail)
        vehicle = None
    if vehicle is not None:
        vehicle_id = vehicle.family.family.vehicle_registry_id or vehicle.family.family_id
        if vehicle_id != endpoint.model_id:
            detail = (
                f"endpoint {endpoint.id}: model_id {endpoint.model_id!r} disagrees with composition vehicle {vehicle_id!r}"
            )
            errors.append(detail)
            selection_findings.append(detail)
        mission = next((item for item in vehicle.declaration.mission_templates if item.id == endpoint.mission_id), None)
        if mission is None:
            detail = f"endpoint {endpoint.id}: unknown composition mission {endpoint.mission_id!r}"
            errors.append(detail)
            selection_findings.append(detail)
        elif endpoint.fidelity not in mission.compatible_fidelities:
            detail = (
                f"endpoint {endpoint.id}: mission {endpoint.mission_id!r} does not support fidelity {endpoint.fidelity!r}"
            )
            errors.append(detail)
            selection_findings.append(detail)
        try:
            interface = resolve_vehicle_interface_contract(endpoint.family_id, endpoint.fidelity, catalog=vehicle_catalog)
            interface_findings = validate_vehicle_interface_contract(interface)
        except (KeyError, TypeError, ValueError) as error:
            errors.append(f"endpoint {endpoint.id}: interface is unavailable: {error}")
            records["interface"] = {"status": "fail", "findings": [str(error)]}
        else:
            if interface_findings:
                errors.extend(f"endpoint {endpoint.id}: interface: {finding}" for finding in interface_findings)
            records["interface"] = {
                "id": interface.id,
                "fingerprint_sha256": interface.fingerprint,
                "status": "pass" if not interface_findings else "fail",
                "findings": list(interface_findings),
            }
    records["selection"] = {
        "status": "pass" if not selection_findings else "fail",
        "family_id": endpoint.family_id,
        "model_id": endpoint.model_id,
        "mission_id": endpoint.mission_id,
        "fidelity": endpoint.fidelity,
        "findings": selection_findings,
    }
    phase_durations_s["selection_and_interface_s"] = _elapsed_s(selection_started)

    output_started = time.perf_counter()
    output_record = _verify_output_contract(endpoint, errors)
    phase_durations_s["output_contract_s"] = _elapsed_s(output_started)
    controller_started = time.perf_counter()
    controller_record, tuning_context_set = _verify_controller_campaign(endpoint, tune=tune, cache_dir=cache_dir, errors=errors)
    phase_durations_s["controller_campaign_s"] = _elapsed_s(controller_started)
    maturity_started = time.perf_counter()
    maturity_record = _verify_maturity_record(endpoint, errors)
    phase_durations_s["maturity_registry_s"] = _elapsed_s(maturity_started)
    records["outputs"] = output_record
    records["controller"] = controller_record
    records["maturity"] = maturity_record

    witness_started = time.perf_counter()
    execution_catalog = load_vehicle_execution_binding_catalog()
    witness_catalog = load_vehicle_execution_witness_catalog()
    binding_records: list[dict[str, object]] = []
    witness_records: list[dict[str, object]] = []
    witness_by_id = {item.id: item for item in witness_catalog.witnesses}
    for operation_spec in endpoint.operations:
        binding = _find_execution_binding(endpoint, operation_spec.operation, execution_catalog.bindings)
        binding_records.append(_binding_record(endpoint, operation_spec, binding))
        if binding is None:
            errors.append(f"endpoint {endpoint.id}: no binding for operation {operation_spec.operation!r}")
        else:
            if binding.status != "runnable":
                errors.append(f"endpoint {endpoint.id}: operation {operation_spec.operation!r} is not runnable")
            if binding.factory_id != operation_spec.factory_id:
                errors.append(
                    f"endpoint {endpoint.id}: operation {operation_spec.operation!r} factory is {binding.factory_id!r}, "
                    f"expected {operation_spec.factory_id!r}"
                )
            if binding.execution_mode != endpoint.execution_mode:
                errors.append(
                    f"endpoint {endpoint.id}: operation {operation_spec.operation!r} mode is {binding.execution_mode!r}, "
                    f"expected {endpoint.execution_mode!r}"
                )

        witness = witness_by_id.get(operation_spec.witness_id)
        if witness is None:
            errors.append(f"endpoint {endpoint.id}: missing witness {operation_spec.witness_id!r}")
            continue
        if witness.operation != operation_spec.operation:
            errors.append(
                f"endpoint {endpoint.id}: witness {witness.id!r} operation {witness.operation!r} does not match "
                f"{operation_spec.operation!r}"
            )
        witness_records.append(
            _verify_witness(
                endpoint,
                operation_spec,
                witness.id,
                witness.composition,
                execute=execute,
                controller_record=controller_record,
                tuning_context_set=tuning_context_set,
                retained_results_root=retained_results_root,
                vehicle_catalog=vehicle_catalog,
                errors=errors,
            )
        )
    records["execution"] = binding_records
    records["witnesses"] = witness_records
    phase_durations_s["binding_and_witness_s"] = _elapsed_s(witness_started)
    acceptance_started = time.perf_counter()
    records["acceptance_gates"] = _build_endpoint_acceptance_gates(
        records,
        endpoint=endpoint,
        execute=execute,
        tune=tune,
    )
    phase_durations_s["acceptance_gates_s"] = _elapsed_s(acceptance_started)
    records["performance"] = _build_endpoint_performance_record(
        records,
        phase_durations_s=phase_durations_s,
        total_duration_s=_elapsed_s(verification_started),
        execute=execute,
        tune=tune,
    )

    return {
        "schema": "taoryx.vehicle-endpoint-verification/v1alpha1",
        "endpoint": endpoint.model_dump(mode="json"),
        "status": "pass" if not errors else "fail",
        "execution_requested": execute,
        "tuning_requested": tune,
        "records": records,
        "errors": errors,
        "claim_boundary": (
            "This verifies the selected endpoint's declared cross-contract wiring and optional narrow execution/tuning "
            "screens. It does not establish mission-level tracking, robustness, envelope coverage, or qualification beyond "
            "the endpoint's own claim boundary."
        ),
    }
    ####


def _find_execution_binding(
    endpoint: VehicleEndpointSpec,
    operation: ExecutionOperation,
    bindings: tuple[VehicleExecutionBinding, ...],
) -> VehicleExecutionBinding | None:
    matches = tuple(
        item
        for item in bindings
        if (
            item.family_id == endpoint.family_id
            and item.mission == endpoint.mission_id
            and item.fidelity == endpoint.fidelity
            and item.operation == operation
        )
    )
    if len(matches) != 1:
        return None
    return matches[0]
    ####


def _binding_record(
    endpoint: VehicleEndpointSpec,
    operation_spec: VehicleEndpointOperationSpec,
    binding: VehicleExecutionBinding | None,
) -> dict[str, object]:
    """Make each exact runtime link visible in the focused report."""

    findings: list[str] = []
    if binding is None:
        findings.append("no exact binding is registered")
    else:
        if binding.status != "runnable":
            findings.append(f"binding status is {binding.status!r}, not 'runnable'")
        if binding.factory_id != operation_spec.factory_id:
            findings.append(f"factory is {binding.factory_id!r}, expected {operation_spec.factory_id!r}")
        if binding.execution_mode != endpoint.execution_mode:
            findings.append(f"mode is {binding.execution_mode!r}, expected {endpoint.execution_mode!r}")
    return {
        "operation": operation_spec.operation,
        "expected_factory_id": operation_spec.factory_id,
        "witness_id": operation_spec.witness_id,
        "binding": None if binding is None else binding.model_dump(mode="json"),
        "status": "pass" if not findings else "fail",
        "findings": findings,
    }
    ####


def _verify_witness(
    endpoint: VehicleEndpointSpec,
    operation_spec: VehicleEndpointOperationSpec,
    witness_id: str,
    composition_path: str,
    *,
    execute: bool,
    controller_record: Mapping[str, object],
    tuning_context_set: TuningApplicationContextSet | None,
    retained_results_root: Path | None,
    vehicle_catalog: ResolvedVehicleCompositionCatalog,
    errors: list[str],
) -> dict[str, object]:
    """Compile/preflight one exact witness and optionally run its batch factory."""

    timings_s: dict[str, float] = {}
    source = ROOT / composition_path
    record: dict[str, object] = {
        "id": witness_id,
        "operation": operation_spec.operation,
        "composition": composition_path,
        "compile_status": "not_run",
        "identity_status": "not_run",
        "preflight_status": "not_run",
        "capability_advertisement": {"status": "not_run"},
        "resolved_factory_id": None,
        "resolved_binding_status": "not_run",
        "execution_status": "not_requested",
        "emitted_outputs": {"status": "not_requested"},
        "result_catalog": {"status": "not_requested"},
        "controller_execution_evidence": {"status": "not_requested"},
        "tracking_evidence": {"status": "not_requested"},
        "disturbance_or_mass_evidence": {"status": "not_requested"},
        "tuning_provenance": {"status": "not_requested"},
        "timings_s": timings_s,
    }
    compile_started = time.perf_counter()
    if not source.is_file():
        errors.append(f"endpoint {endpoint.id}: witness {witness_id!r} composition is missing: {composition_path}")
        record["compile_status"] = "missing"
        timings_s["compile_s"] = _elapsed_s(compile_started)
        return record
    try:
        composition = compile_vehicle_composition(load_vehicle_composition_request(source), catalog=vehicle_catalog)
    except (TypeError, ValueError) as error:
        errors.append(f"endpoint {endpoint.id}: witness {witness_id!r} does not compile: {error}")
        record["compile_status"] = "fail"
        timings_s["compile_s"] = _elapsed_s(compile_started)
        return record
    timings_s["compile_s"] = _elapsed_s(compile_started)
    record["compile_status"] = "pass"
    actual = (composition.family_id, composition.vehicle_id, composition.mission, composition.fidelity)
    expected = (endpoint.family_id, endpoint.model_id, endpoint.mission_id, endpoint.fidelity)
    record["identity_status"] = "pass" if actual == expected else "fail"
    if actual != expected:
        errors.append(f"endpoint {endpoint.id}: witness {witness_id!r} selects {actual!r}, expected {expected!r}")
    binding_started = time.perf_counter()
    try:
        resolved_binding = resolve_vehicle_execution_binding(composition, operation_spec.operation)
    except (KeyError, TypeError, ValueError) as error:
        record["resolved_binding_status"] = "fail"
        errors.append(
            f"endpoint {endpoint.id}: witness {witness_id!r} cannot resolve {operation_spec.operation!r}: {error}"
        )
    else:
        record["resolved_factory_id"] = resolved_binding.factory_id
        record["resolved_binding_status"] = "pass" if resolved_binding.factory_id == operation_spec.factory_id else "fail"
        if resolved_binding.factory_id != operation_spec.factory_id:
            errors.append(
                f"endpoint {endpoint.id}: witness {witness_id!r} resolves factory {resolved_binding.factory_id!r}, "
                f"expected {operation_spec.factory_id!r}"
            )
    timings_s["binding_resolution_s"] = _elapsed_s(binding_started)
    preflight_started = time.perf_counter()
    preflight = preflight_vehicle_composition(composition)
    timings_s["preflight_s"] = _elapsed_s(preflight_started)
    record["preflight_status"] = preflight.status
    record["capability_adapter_id"] = (
        preflight.capability_estimate.get("adapter_id")
        if isinstance(preflight.capability_estimate, Mapping)
        else None
    )
    if preflight.status != "translation_ready":
        errors.append(f"endpoint {endpoint.id}: witness {witness_id!r} preflight is {preflight.status!r}")
    capability_started = time.perf_counter()
    record["capability_advertisement"] = _verify_capability_advertisement(endpoint, composition, preflight, errors)
    timings_s["capability_advertisement_s"] = _elapsed_s(capability_started)
    if execute and operation_spec.operation == "batch":
        batch_started = time.perf_counter()
        try:
            if retained_results_root is not None:
                output_directory = retained_results_root / witness_id
                batch = _execute_endpoint_batch(
                    composition,
                    output_directory,
                    tuning_context_set=tuning_context_set,
                )
                timings_s["batch_execution_s"] = _elapsed_s(batch_started)
                record["result_directory"] = str(batch.output_dir)
                _record_batch_evidence(
                    endpoint,
                    witness_id,
                    batch.output_dir,
                    batch.passed,
                    controller_record=controller_record,
                    record=record,
                    timings_s=timings_s,
                    errors=errors,
                )
            else:
                with tempfile.TemporaryDirectory(prefix="taoryx-endpoint-") as temporary:
                    batch = _execute_endpoint_batch(
                        composition,
                        Path(temporary) / "result",
                        tuning_context_set=tuning_context_set,
                    )
                    timings_s["batch_execution_s"] = _elapsed_s(batch_started)
                    _record_batch_evidence(
                        endpoint,
                        witness_id,
                        batch.output_dir,
                        batch.passed,
                        controller_record=controller_record,
                        record=record,
                        timings_s=timings_s,
                        errors=errors,
                    )
        except (OSError, RuntimeError, TypeError, ValueError) as error:
            timings_s.setdefault("batch_execution_s", _elapsed_s(batch_started))
            record["execution_status"] = "fail"
            errors.append(f"endpoint {endpoint.id}: witness {witness_id!r} batch execution failed: {error}")
    elif execute and operation_spec.operation == "episode":
        episode_started = time.perf_counter()
        try:
            episode = open_vehicle_composition_episode(composition, seed=0)
            try:
                initial_truth = episode.reset(seed=0)
                record["execution_status"] = "pass"
                record["episode_initial_truth_channel_count"] = len(initial_truth.values)
            finally:
                episode.close()
        except (OSError, RuntimeError, TypeError, ValueError) as error:
            record["execution_status"] = "fail"
            errors.append(f"endpoint {endpoint.id}: witness {witness_id!r} episode execution failed: {error}")
        timings_s["episode_execution_s"] = _elapsed_s(episode_started)
    return record
    ####


def _execute_endpoint_batch(
    composition: CompiledVehicleComposition,
    output_directory: Path,
    *,
    tuning_context_set: TuningApplicationContextSet | None,
) -> VehicleBatchExecution:
    """Pass one local candidate or one complete schedule selection to batch execution."""

    if tuning_context_set is None:
        return execute_vehicle_composition_batch(composition, output_directory)
    singular = tuning_context_set.singular
    if singular is not None:
        return execute_vehicle_composition_batch(
            composition,
            output_directory,
            tuning_context=singular,
        )
    return execute_vehicle_composition_batch(
        composition,
        output_directory,
        tuning_context_set=tuning_context_set,
    )
    ####


def _prepare_endpoint_results_directory(directory: str | Path) -> Path:
    """Create one explicitly requested, otherwise-empty endpoint result root."""

    root = Path(directory)
    if root.exists():
        if not root.is_dir():
            raise ValueError(f"vehicle endpoint results directory is not a directory: {root}")
        if any(root.iterdir()):
            raise ValueError(f"vehicle endpoint results directory must be empty: {root}")
    else:
        root.mkdir(parents=True, exist_ok=False)
    return root
    ####


def _record_batch_evidence(
    endpoint: VehicleEndpointSpec,
    witness_id: str,
    output_directory: Path,
    batch_passed: bool,
    *,
    controller_record: Mapping[str, object],
    record: dict[str, object],
    timings_s: dict[str, float],
    errors: list[str],
) -> None:
    """Validate the packet emitted by an exact batch witness before it is discarded or retained."""

    record["execution_status"] = "pass" if batch_passed else "fail"
    record["batch_result"] = {
        "passed": batch_passed,
        "output_directory": str(output_directory),
    }
    if not batch_passed:
        errors.append(f"endpoint {endpoint.id}: witness {witness_id!r} batch execution did not pass")

    emitted_output_started = time.perf_counter()
    record["emitted_outputs"] = _verify_emitted_output_contract(
        endpoint,
        witness_id,
        output_directory,
        errors,
    )
    timings_s["emitted_output_validation_s"] = _elapsed_s(emitted_output_started)
    result_index_started = time.perf_counter()
    result_catalog = _verify_execution_result_packet(endpoint, witness_id, output_directory, errors)
    timings_s["result_catalog_index_s"] = _elapsed_s(result_index_started)
    record["result_catalog"] = result_catalog
    controller_evidence = result_catalog.get("controller_execution_evidence")
    record["controller_execution_evidence"] = (
        dict(controller_evidence) if isinstance(controller_evidence, Mapping) else {"status": "not_available"}
    )
    tracking_started = time.perf_counter()
    record["tracking_evidence"] = _verify_tracking_evidence(endpoint, witness_id, output_directory, errors)
    timings_s["tracking_validation_s"] = _elapsed_s(tracking_started)
    robustness_started = time.perf_counter()
    record["disturbance_or_mass_evidence"] = _verify_disturbance_or_mass_evidence(
        endpoint,
        witness_id,
        output_directory,
        errors,
    )
    timings_s["robustness_validation_s"] = _elapsed_s(robustness_started)
    tuning_provenance = _build_controller_tuning_provenance(
        endpoint,
        witness_id,
        controller_record,
        record["controller_execution_evidence"],
    )
    provenance_started = time.perf_counter()
    try:
        artifact = _write_controller_tuning_provenance(output_directory, tuning_provenance)
    except (OSError, TypeError, ValueError) as error:
        errors.append(f"endpoint {endpoint.id}: witness {witness_id!r} could not record controller/tuning provenance: {error}")
        tuning_provenance["status"] = "fail"
        tuning_provenance["artifact_path"] = None
    else:
        tuning_provenance["artifact_path"] = str(artifact)
    timings_s["tuning_provenance_write_s"] = _elapsed_s(provenance_started)
    record["tuning_provenance"] = tuning_provenance
    ####


def _verify_emitted_output_contract(
    endpoint: VehicleEndpointSpec,
    witness_id: str,
    output_directory: Path,
    errors: list[str],
) -> dict[str, object]:
    """Read emitted telemetry and prove each required advertised channel has finite samples."""

    telemetry_sources, source_errors = _native_telemetry_sources(output_directory)
    applicable = _applicable_output_bindings(endpoint)
    required_ids = (*endpoint.required_core_output_ids, *endpoint.required_telemetry_output_ids)
    sample_counts: dict[str, int] = {}
    missing: list[str] = []
    for identifier in required_ids:
        binding = applicable.get(identifier)
        if binding is None:
            missing.append(identifier)
            sample_counts[identifier] = 0
            continue
        count = 0
        for _, rows in telemetry_sources:
            for row in rows:
                try:
                    extract_native_channel(row, binding)
                except ValueError:
                    continue
                count += 1
        sample_counts[identifier] = count
        if count == 0:
            missing.append(identifier)
    if source_errors:
        errors.extend(f"endpoint {endpoint.id}: witness {witness_id!r} emitted telemetry: {detail}" for detail in source_errors)
    if missing:
        errors.append(
            f"endpoint {endpoint.id}: witness {witness_id!r} emitted no finite samples for required outputs {sorted(missing)!r}"
        )
    return {
        "status": "pass" if telemetry_sources and not source_errors and not missing else "fail",
        "telemetry_sources": [str(path) for path, _ in telemetry_sources],
        "telemetry_row_count": sum(len(rows) for _, rows in telemetry_sources),
        "required_output_ids": list(required_ids),
        "emitted_sample_count_by_id": sample_counts,
        "missing_required_output_ids": sorted(missing),
        "claim_boundary": (
            "This proves the endpoint's required public channels were emitted as finite native samples. It does not "
            "establish their physical accuracy outside the selected result packet."
        ),
    }
    ####


def _native_telemetry_sources(output_directory: Path) -> tuple[list[tuple[Path, list[dict[str, object]]]], list[str]]:
    """Load common row-oriented telemetry without assuming a family-specific encoding.

    CSV remains the default packet encoding.  Native-coordinate controller
    screens retain nested control and state values, so they emit the same
    row-oriented truth surface as JSON instead of flattening it into an
    untyped CSV string cell.  Both encodings feed the identical normalized
    output extraction contract below.
    """

    sources: list[tuple[Path, list[dict[str, object]]]] = []
    errors: list[str] = []
    for filename in ("truth_telemetry.csv", "telemetry.csv"):
        source = output_directory / filename
        if not source.is_file():
            continue
        try:
            with source.open(encoding="utf-8", newline="") as stream:
                reader = csv.DictReader(stream)
                if reader.fieldnames is None:
                    raise ValueError("CSV has no header row")
                rows = [
                    {str(key): value for key, value in row.items() if key is not None}
                    for row in reader
                ]
        except (OSError, csv.Error, ValueError) as error:
            errors.append(f"could not read {filename}: {error}")
            continue
        if not rows:
            errors.append(f"{filename} has no telemetry rows")
            continue
        sources.append((source, rows))
    json_source = output_directory / "truth_telemetry.json"
    if json_source.is_file():
        try:
            payload = json.loads(json_source.read_text(encoding="utf-8"))
            if not isinstance(payload, list):
                raise ValueError("JSON telemetry must be a list of row mappings")
            rows = []
            for index, item in enumerate(payload):
                if not isinstance(item, Mapping):
                    raise ValueError(f"JSON telemetry row {index} is not a mapping")
                rows.append(dict(item))
        except (OSError, ValueError, json.JSONDecodeError) as error:
            errors.append(f"could not read truth_telemetry.json: {error}")
        else:
            if not rows:
                errors.append("truth_telemetry.json has no telemetry rows")
            else:
                sources.append((json_source, rows))
    if not sources and not errors:
        errors.append("no truth_telemetry.csv, telemetry.csv, or truth_telemetry.json artifact was emitted")
    return sources, errors
    ####


def _verify_execution_result_packet(
    endpoint: VehicleEndpointSpec,
    witness_id: str,
    output_directory: Path,
    errors: list[str],
) -> dict[str, object]:
    """Index the emitted packet and retain the catalog's controller-runtime evidence."""

    try:
        catalog = index_composition_results(output_directory)
    except (OSError, RuntimeError, TypeError, ValueError) as error:
        errors.append(f"endpoint {endpoint.id}: witness {witness_id!r} result packet could not be indexed: {error}")
        return {"status": "fail", "error": str(error), "controller_execution_evidence": {"status": "not_available"}}
    records = catalog.get("records")
    valid_records = [item for item in records if isinstance(item, Mapping) and item.get("status") == "valid"] if isinstance(records, list) else []
    if catalog.get("status") != "pass" or len(valid_records) != 1:
        catalog_errors = catalog.get("errors")
        detail = (
            f"result catalog status is {catalog.get('status')!r} with {len(valid_records)} valid mission result(s), expected one"
        )
        errors.append(f"endpoint {endpoint.id}: witness {witness_id!r} {detail}")
        return {
            "status": "fail",
            "catalog_status": catalog.get("status"),
            "valid_result_count": len(valid_records),
            "errors": list(catalog_errors) if isinstance(catalog_errors, list) else [],
            "controller_execution_evidence": {"status": "not_available"},
        }
    result = valid_records[0]
    controller_evidence = result.get("controller_execution_evidence")
    if not isinstance(controller_evidence, Mapping):
        detail = "controller execution evidence is unavailable"
        errors.append(f"endpoint {endpoint.id}: witness {witness_id!r} {detail}")
        return {
            "status": "fail",
            "catalog_status": catalog.get("status"),
            "valid_result_count": len(valid_records),
            "controller_execution_evidence": {"status": "not_available"},
        }
    controller_status = controller_evidence.get("status")
    expected_controller_status = "not_applicable" if endpoint.controller_campaign_id is None else "verified"
    if controller_status != expected_controller_status:
        detail = (
            f"controller execution evidence is {controller_status!r}, expected {expected_controller_status!r} "
            "for the endpoint controller contract"
        )
        errors.append(f"endpoint {endpoint.id}: witness {witness_id!r} {detail}")
        return {
            "status": "fail",
            "catalog_status": catalog.get("status"),
            "valid_result_count": len(valid_records),
            "controller_execution_evidence": dict(controller_evidence),
        }
    return {
        "status": "pass",
        "catalog_status": catalog.get("status"),
        "valid_result_count": len(valid_records),
        "controller_execution_evidence": dict(controller_evidence),
        "evaluation_path": result.get("evaluation_path"),
        "claim_boundary": (
            "The result catalog validates the common normalized packet and either the declared controller runtime "
            "evidence or the explicit controller-free disposition, without re-executing or retuning the vehicle."
        ),
    }
    ####


def _verify_tracking_evidence(
    endpoint: VehicleEndpointSpec,
    witness_id: str,
    output_directory: Path,
    errors: list[str],
) -> dict[str, object]:
    """Make the exact batch screen's required outcome gates visible and fail closed."""

    source = output_directory / "objective_report.json"
    if not source.is_file():
        errors.append(f"endpoint {endpoint.id}: witness {witness_id!r} has no objective_report.json tracking screen")
        return {"status": "fail", "source_path": None, "reason": "objective report is missing"}
    try:
        screen = _read_json_mapping(source)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        errors.append(f"endpoint {endpoint.id}: witness {witness_id!r} has invalid objective report: {error}")
        return {"status": "fail", "source_path": str(source), "reason": str(error)}
    mission_pass = screen.get("mission_pass")
    raw_results = screen.get("results")
    if not isinstance(mission_pass, bool) or not isinstance(raw_results, list):
        detail = "objective report must contain boolean mission_pass and a results list"
        errors.append(f"endpoint {endpoint.id}: witness {witness_id!r} {detail}")
        return {"status": "fail", "source_path": str(source), "reason": detail}
    required_results = [item for item in raw_results if isinstance(item, Mapping) and item.get("required") is True]
    malformed = [item for item in required_results if not isinstance(item.get("id"), str) or item.get("status") not in {"pass", "fail"}]
    if not required_results or malformed:
        detail = "objective report has no well-formed required result gates"
        errors.append(f"endpoint {endpoint.id}: witness {witness_id!r} {detail}")
        return {"status": "fail", "source_path": str(source), "reason": detail}
    failed_ids = sorted(str(item["id"]) for item in required_results if item.get("status") != "pass")
    status = "pass" if mission_pass and not failed_ids else "fail"
    if status != "pass":
        errors.append(f"endpoint {endpoint.id}: witness {witness_id!r} required tracking/screen gates did not pass")
    return {
        "status": status,
        "source_path": str(source),
        "mission_pass": mission_pass,
        "required_result_count": len(required_results),
        "required_passed_count": len(required_results) - len(failed_ids),
        "failed_required_result_ids": failed_ids,
        "claim_boundary": (
            "This is the endpoint's declared local tracking/control screen, not evidence of general navigation, "
            "route following, or an operating envelope."
        ),
    }
    ####


def _verify_disturbance_or_mass_evidence(
    endpoint: VehicleEndpointSpec,
    witness_id: str,
    output_directory: Path,
    errors: list[str],
) -> dict[str, object]:
    """Verify explicit, per-endpoint robustness contracts without inferring coverage.

    Previous endpoint reports merely looked for a generically named file.
    The typed declaration now makes each proposed mass, offset, or wind case
    visible even before its source-owning model can execute it.  A planned or
    blocked screen remains incomplete rather than becoming a passing result.
    """

    requirement = endpoint.robustness_requirement
    if requirement.disposition == "not_applicable":
        return {
            "status": "not_applicable",
            "evidence": [],
            "required_screen_count": 0,
            "passed_required_screen_count": 0,
            "reason": requirement.reason,
            "claim_boundary": (
                "This endpoint declares no mass, wind, or persistent-disturbance seam at its selected fidelity. "
                "That absence is explicit and is not robustness evidence."
            ),
        }

    evidence: list[dict[str, object]] = []
    for screen in endpoint.robustness_screens:
        source = output_directory / screen.artifact_filename
        summary = _robustness_screen_contract_summary(screen)
        if not source.is_file():
            if screen.execution_readiness == "available":
                detail = f"required robustness artifact {screen.artifact_filename!r} was not emitted"
                errors.append(f"endpoint {endpoint.id}: witness {witness_id!r} {detail}")
                summary.update({"status": "fail", "source_path": None, "findings": [detail]})
            elif screen.execution_readiness == "blocked":
                summary.update(
                    {
                        "status": "blocked",
                        "source_path": None,
                        "reason": screen.availability_reason,
                    }
                )
            else:
                summary.update(
                    {
                        "status": "not_executed",
                        "source_path": None,
                        "reason": "the screen is declared but its source-owning batch factory has not emitted it",
                    }
                )
            evidence.append(summary)
            continue
        try:
            payload = _read_json_mapping(source)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            detail = f"could not read {screen.artifact_filename}: {error}"
            errors.append(f"endpoint {endpoint.id}: witness {witness_id!r} {detail}")
            summary.update({"status": "fail", "source_path": str(source), "findings": [detail]})
            evidence.append(summary)
            continue
        validation = _verify_declared_robustness_screen(screen, payload)
        summary.update(validation)
        summary["source_path"] = str(source)
        if validation["status"] != "pass":
            findings = validation.get("findings")
            detail = "; ".join(str(item) for item in findings) if isinstance(findings, list) else "screen did not satisfy its declaration"
            errors.append(
                f"endpoint {endpoint.id}: witness {witness_id!r} robustness screen {screen.id!r} did not pass: {detail}"
            )
        evidence.append(summary)

    required = [item for item in evidence if item.get("required_for_finalization") is True]
    statuses = [item.get("status") for item in required]
    if statuses and all(status == "pass" for status in statuses):
        status = "pass"
    elif any(status == "fail" for status in statuses):
        status = "fail"
    elif any(status == "blocked" for status in statuses):
        status = "blocked"
    else:
        status = "not_executed"
    return {
        "status": status,
        "evidence": evidence,
        "required_screen_count": len(required),
        "passed_required_screen_count": sum(item.get("status") == "pass" for item in required),
        "claim_boundary": (
            "Each result is checked against the endpoint's declared cases, metrics, and thresholds only. A pass does "
            "not extrapolate to an untested disturbance, mass range, operating point, or qualification envelope."
        ),
    }
    ####


def _robustness_screen_contract_summary(screen: VehicleEndpointRobustnessScreenSpec) -> dict[str, object]:
    """Project one declared screen into an execution-report-friendly record."""

    return {
        "id": screen.id,
        "kind": screen.kind,
        "artifact_filename": screen.artifact_filename,
        "execution_readiness": screen.execution_readiness,
        "required_for_finalization": screen.required_for_finalization,
        "cases": [item.model_dump(mode="json") for item in screen.cases],
        "metrics": [item.model_dump(mode="json") for item in screen.metrics],
        "claim_boundary": screen.claim_boundary,
    }
    ####


def _verify_declared_robustness_screen(
    screen: VehicleEndpointRobustnessScreenSpec,
    payload: Mapping[str, object],
) -> dict[str, object]:
    """Fail closed unless an emitted screen satisfies its generic declared contract."""

    findings: list[str] = []
    if payload.get("schema") != "taoryx.endpoint-robustness-screen/v1alpha1":
        findings.append("schema is not 'taoryx.endpoint-robustness-screen/v1alpha1'")
    if payload.get("id") != screen.id:
        findings.append(f"screen id is {payload.get('id')!r}, expected {screen.id!r}")
    if payload.get("kind") != screen.kind:
        findings.append(f"screen kind is {payload.get('kind')!r}, expected {screen.kind!r}")
    disposition = _screen_disposition(payload)
    if disposition is not True:
        findings.append("screen does not report a passing disposition")

    raw_cases = payload.get("cases")
    cases_by_id: dict[str, Mapping[str, object]] = {}
    if not isinstance(raw_cases, list):
        findings.append("screen cases must be a list")
    else:
        for raw_case in raw_cases:
            if not isinstance(raw_case, Mapping):
                findings.append("screen cases contain a non-mapping entry")
                continue
            case_id = raw_case.get("id")
            if not isinstance(case_id, str) or not case_id.strip():
                findings.append("screen case has no non-empty id")
                continue
            if case_id in cases_by_id:
                findings.append(f"screen repeats case {case_id!r}")
                continue
            cases_by_id[case_id] = raw_case

    case_records: list[dict[str, object]] = []
    for expected_case in screen.cases:
        emitted = cases_by_id.get(expected_case.id)
        case_record: dict[str, object] = {
            "id": expected_case.id,
            "status": "fail",
            "parameters": dict(expected_case.parameters),
            "metrics": [],
        }
        if emitted is None:
            finding = f"required case {expected_case.id!r} is missing"
            findings.append(finding)
            case_record["findings"] = [finding]
            case_records.append(case_record)
            continue
        case_findings: list[str] = []
        emitted_parameters = emitted.get("parameters")
        if not isinstance(emitted_parameters, Mapping):
            case_findings.append("parameters are missing or not a mapping")
        else:
            for parameter, expected_value in expected_case.parameters.items():
                actual = emitted_parameters.get(parameter)
                if not _finite_number(actual) or float(actual) != expected_value:
                    case_findings.append(
                        f"parameter {parameter!r} is {actual!r}, expected {expected_value!r}"
                    )
        if _screen_disposition(emitted) is not True:
            case_findings.append("case does not report a passing disposition")
        emitted_metrics = emitted.get("metrics")
        metric_records: list[dict[str, object]] = []
        if not isinstance(emitted_metrics, Mapping):
            case_findings.append("metrics are missing or not a mapping")
        else:
            for metric in screen.metrics:
                actual = emitted_metrics.get(metric.id)
                metric_findings: list[str] = []
                if not _finite_number(actual):
                    metric_findings.append("value is missing or non-finite")
                else:
                    numeric = float(actual)
                    if metric.minimum is not None and numeric < metric.minimum:
                        metric_findings.append(f"value {numeric!r} is below minimum {metric.minimum!r}")
                    if metric.maximum is not None and numeric > metric.maximum:
                        metric_findings.append(f"value {numeric!r} exceeds maximum {metric.maximum!r}")
                if metric_findings:
                    case_findings.extend(f"metric {metric.id!r}: {item}" for item in metric_findings)
                metric_records.append(
                    {
                        "id": metric.id,
                        "actual": actual,
                        "minimum": metric.minimum,
                        "maximum": metric.maximum,
                        "unit": metric.unit,
                        "status": "pass" if not metric_findings else "fail",
                        "findings": metric_findings,
                    }
                )
        case_record["metrics"] = metric_records
        case_record["status"] = "pass" if not case_findings else "fail"
        case_record["findings"] = case_findings
        findings.extend(f"case {expected_case.id!r}: {item}" for item in case_findings)
        case_records.append(case_record)
    return {
        "status": "pass" if not findings else "fail",
        "screen_disposition": disposition,
        "reported_case_count": len(cases_by_id),
        "required_case_count": len(screen.cases),
        "cases": case_records,
        "findings": findings,
    }
    ####


def _screen_disposition(payload: Mapping[str, object]) -> bool | None:
    """Extract a conservative boolean disposition from common screen artifacts."""

    for key in ("mission_pass", "screen_pass", "pass"):
        value = payload.get(key)
        if isinstance(value, bool):
            return value
    status = payload.get("status")
    if status in {"pass", "passed", "verified"}:
        return True
    if status in {"fail", "failed", "blocked", "invalid"}:
        return False
    return None
    ####


def _finite_number(value: object) -> TypeGuard[int | float]:
    """Return whether a JSON scalar is a finite non-boolean number."""

    return isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(float(value))
    ####


def _read_json_mapping(source: Path) -> dict[str, object]:
    """Load one JSON object with a focused artifact error when its shape is wrong."""

    payload = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{source.name} is not a JSON object")
    return dict(payload)
    ####


def _write_controller_tuning_provenance(output_directory: Path, payload: Mapping[str, object]) -> Path:
    """Write the host-owned provenance sidecar without overwriting a family artifact."""

    destination = output_directory / _CONTROLLER_TUNING_PROVENANCE_FILENAME
    if destination.exists():
        raise ValueError(f"controller/tuning provenance artifact already exists: {destination}")
    encoded = json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n"
    destination.write_text(encoded, encoding="utf-8")
    return destination
    ####


def _build_controller_tuning_provenance(
    endpoint: VehicleEndpointSpec,
    witness_id: str,
    controller_record: Mapping[str, object],
    controller_evidence: object,
) -> dict[str, object]:
    """Bind an executed controller claim to a selected tuning candidate when the runtime declares one."""

    evidence = dict(controller_evidence) if isinstance(controller_evidence, Mapping) else {"status": "not_available"}
    controller_configuration = {
        key: evidence.get(key)
        for key in ("controller_id", "method", "control_realization", "integral_output_names")
    }
    execution = {
        "output_directory": ".",
        "controller_evidence_status": evidence.get("status"),
        "composition_id": evidence.get("composition_id"),
        "composition_identity_sha256": evidence.get("composition_identity_sha256"),
        "controller_id": evidence.get("controller_id"),
        "controller_method": evidence.get("method"),
        "control_realization": evidence.get("control_realization"),
        "integral_output_names": evidence.get("integral_output_names"),
        "controller_configuration_fingerprint_sha256": _canonical_json_fingerprint(controller_configuration),
    }
    tuning: dict[str, object] = {
        "campaign_id": controller_record.get("campaign_id"),
        "campaign_registration_fingerprint_sha256": controller_record.get("campaign_registration_fingerprint_sha256"),
        "tuning_status": controller_record.get("tuning_status"),
    }
    for key in (
        "cache_key",
        "cache_hit",
        "cache_disposition",
        "candidate_status",
        "candidate_payload_fingerprint_sha256",
        "candidate_methods",
        "selected_candidate_configurations",
        "tuning_application_status",
        "tuning_application_contexts",
    ):
        if key in controller_record:
            tuning[key] = controller_record[key]
    binding = _resolve_tuning_binding(controller_record, evidence)
    return {
        "schema": "taoryx.vehicle-endpoint-controller-tuning-provenance/v1alpha1",
        "status": "recorded",
        "endpoint_id": endpoint.id,
        "witness_id": witness_id,
        "execution": execution,
        "tuning": tuning,
        "binding": binding,
        "claim_boundary": (
            "This records the exact batch packet, selected campaign cache identity, and controller-runtime evidence. "
            "A candidate is only marked bound when the runtime declares the same campaign node/profile/configuration "
            "and applied-gain fingerprints; matching controller method alone is not proof of applied gains."
        ),
    }
    ####


def _resolve_tuning_binding(
    controller_record: Mapping[str, object],
    controller_evidence: Mapping[str, object],
) -> dict[str, object]:
    """Compare a runtime-declared candidate binding to the exact tuned candidate fingerprints."""

    tuning_status = controller_record.get("tuning_status")
    if tuning_status == "not_requested":
        return {"status": "not_requested", "reason": "the controller campaign was not run for this batch packet"}
    if tuning_status != "pass" or controller_record.get("candidate_status") != "candidate_ready":
        return {
            "status": "candidate_not_ready",
            "reason": "the controller campaign did not produce a candidate-ready tuning artifact",
        }
    if controller_evidence.get("status") != "verified":
        return {
            "status": "execution_controller_not_verified",
            "reason": "the emitted packet has no verified common controller-runtime evidence",
        }
    runtime_method = controller_evidence.get("method")
    candidate_methods = controller_record.get("candidate_methods")
    if isinstance(candidate_methods, list) and runtime_method not in candidate_methods:
        return {
            "status": "method_mismatch",
            "reason": f"runtime controller method {runtime_method!r} is not among tuned methods {candidate_methods!r}",
        }
    raw_runtime_binding = controller_evidence.get("tuning_binding")
    raw_runtime_bindings = controller_evidence.get("tuning_bindings")
    binding_shape: str
    payloads: list[object]
    if isinstance(raw_runtime_binding, Mapping) and raw_runtime_binding.get("status") == "declared":
        binding_shape = "single"
        single = dict(raw_runtime_binding)
        single.pop("status", None)
        payloads = [single]
    elif isinstance(raw_runtime_binding, Mapping) and raw_runtime_binding.get("status") == "declared_set":
        binding_shape = "set"
        raw_items = raw_runtime_binding.get("bindings")
        if not isinstance(raw_items, list):
            return {
                "status": "candidate_binding_mismatch",
                "reason": "runtime tuning binding set has no receipt list",
                "runtime_bindings": dict(raw_runtime_binding),
            }
        payloads = list(raw_items)
    elif isinstance(raw_runtime_bindings, Mapping) and raw_runtime_bindings.get("status") == "declared_set":
        binding_shape = "set"
        raw_items = raw_runtime_bindings.get("bindings")
        if not isinstance(raw_items, list):
            return {
                "status": "candidate_binding_mismatch",
                "reason": "runtime tuning binding set has no receipt list",
                "runtime_bindings": dict(raw_runtime_bindings),
            }
        payloads = list(raw_items)
    else:
        return {
            "status": "candidate_ready_not_runtime_bound",
            "reason": (
                "the tuned candidate is ready and the controller method is compatible, but the runtime did not declare "
                "a campaign node/profile/configuration and applied-gain fingerprint"
            ),
        }
    try:
        runtime_bindings = tuple(RuntimeTuningBindingReceipt.model_validate(item) for item in payloads)
    except ValueError as error:
        return {
            "status": "candidate_binding_mismatch",
            "reason": f"runtime tuning binding violates the typed receipt contract: {error}",
            "runtime_binding": (
                raw_runtime_binding
                if isinstance(raw_runtime_binding, Mapping)
                else raw_runtime_bindings
            ),
        }
    if not runtime_bindings:
        return {
            "status": "candidate_binding_mismatch",
            "reason": "runtime tuning binding set is empty",
        }

    expected_campaign = controller_record.get("campaign_id")
    contexts = controller_record.get("tuning_application_contexts")
    context_items = contexts if isinstance(contexts, list) else []
    expected_contexts: dict[str, Mapping[str, object]] = {}
    for context in context_items:
        if not isinstance(context, Mapping):
            continue
        node_id = context.get("node_id")
        if isinstance(node_id, str):
            expected_contexts[node_id] = context
    expected_nodes = tuple(expected_contexts)
    observed_nodes = tuple(binding.node_id for binding in runtime_bindings)
    if len(observed_nodes) != len(set(observed_nodes)):
        return {
            "status": "candidate_binding_mismatch",
            "reason": "runtime tuning binding set declares a node more than once",
        }
    if len(expected_nodes) > 1:
        if binding_shape != "set":
            return {
                "status": "candidate_binding_mismatch",
                "reason": "a multi-node tuned campaign requires a runtime binding receipt for every executed node",
            }
        if set(observed_nodes) != set(expected_nodes):
            return {
                "status": "candidate_binding_mismatch",
                "reason": (
                    "runtime tuning binding nodes do not match the selected campaign nodes: "
                    f"expected {sorted(expected_nodes)!r}, got {sorted(observed_nodes)!r}"
                ),
            }
    elif binding_shape != "single":
        return {
            "status": "candidate_binding_mismatch",
            "reason": "a one-node tuned campaign must emit one singular runtime binding receipt",
        }

    configurations = controller_record.get("selected_candidate_configurations")
    configuration_items = configurations if isinstance(configurations, list) else []
    candidate_configurations = {
        (
            item.get("node_id"),
            item.get("profile_id"),
            item.get("candidate_configuration_fingerprint_sha256"),
        )
        for item in configuration_items
        if isinstance(item, Mapping)
    }
    expected_cache_key = controller_record.get("cache_key")
    failures: list[str] = []
    for runtime_binding in runtime_bindings:
        prefix = f"{runtime_binding.node_id}:"
        if runtime_binding.campaign_id != expected_campaign:
            failures.append(prefix + "campaign_id")
        candidate_key = (
            runtime_binding.node_id,
            runtime_binding.candidate_profile_id,
            runtime_binding.candidate_configuration_fingerprint_sha256,
        )
        if candidate_key not in candidate_configurations:
            failures.append(prefix + "candidate_configuration_fingerprint_sha256")
        context = expected_contexts.get(runtime_binding.node_id)
        expected_gain_fingerprint = (
            context.get("resolved_gain_fingerprint_sha256") if isinstance(context, Mapping) else None
        )
        if not isinstance(expected_gain_fingerprint, str):
            failures.append(prefix + "tuning_application_context")
        elif runtime_binding.applied_gain_fingerprint_sha256 != expected_gain_fingerprint:
            failures.append(prefix + "applied_gain_fingerprint_sha256")
        if runtime_binding.controller_method != runtime_method:
            failures.append(prefix + "controller_method")
        if (
            runtime_binding.cache_key is not None
            and expected_cache_key is not None
            and runtime_binding.cache_key != expected_cache_key
        ):
            failures.append(prefix + "cache_key")
    if failures:
        return {
            "status": "candidate_binding_mismatch",
            "reason": "runtime tuning binding disagrees with the tuned candidate: " + ", ".join(failures),
            "runtime_binding": (
                {"status": "declared", **runtime_bindings[0].as_dict()}
                if binding_shape == "single"
                else {"status": "declared_set", "bindings": [item.as_dict() for item in runtime_bindings]}
            ),
        }
    result: dict[str, object] = {
        "status": "candidate_bound",
        "campaign_id": expected_campaign,
        "node_count": len(runtime_bindings),
    }
    if binding_shape == "single":
        runtime_binding = runtime_bindings[0]
        result.update(
            {
                "node_id": runtime_binding.node_id,
                "candidate_profile_id": runtime_binding.candidate_profile_id,
                "candidate_configuration_fingerprint_sha256": runtime_binding.candidate_configuration_fingerprint_sha256,
                "applied_gain_fingerprint_sha256": runtime_binding.applied_gain_fingerprint_sha256,
            }
        )
    else:
        result["bindings"] = [item.as_dict() for item in runtime_bindings]
    return result
    ####


def _canonical_json_fingerprint(payload: object) -> str:
    """Return one stable hash for JSON-compatible controller or campaign evidence."""

    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
    ####


def _verify_capability_advertisement(
    endpoint: VehicleEndpointSpec,
    composition: object,
    preflight: object,
    errors: list[str],
) -> dict[str, object]:
    """Exercise the generic advertisement emitted by the exact compiled witness."""

    capability_estimate = getattr(preflight, "capability_estimate", None)
    if not isinstance(capability_estimate, Mapping):
        errors.append(f"endpoint {endpoint.id}: witness has no concrete capability advertisement")
        return {"status": "fail", "findings": ["capability estimate is missing"]}
    advertisement = capability_estimate.get("capability_advertisement")
    if not isinstance(advertisement, Mapping):
        errors.append(f"endpoint {endpoint.id}: witness has no generic capability advertisement")
        return {"status": "fail", "findings": ["capability advertisement is missing"]}
    expected_selection = {
        "composition_id": getattr(composition, "id", None),
        "composition_identity_sha256": getattr(composition, "identity_sha256", None),
        "vehicle_id": getattr(composition, "vehicle_id", None),
        "family_id": endpoint.family_id,
        "mission_id": endpoint.mission_id,
        "fidelity": endpoint.fidelity,
    }
    findings = validate_public_capability_advertisement(advertisement, expected_selection=expected_selection)
    if findings:
        errors.extend(f"endpoint {endpoint.id}: capability advertisement: {finding}" for finding in findings)
    interface = advertisement.get("interface")
    family_owned = advertisement.get("family_owned")
    return {
        "status": "pass" if not findings else "fail",
        "fingerprint_sha256": advertisement.get("fingerprint_sha256"),
        "interface_id": interface.get("interface_id") if isinstance(interface, Mapping) else None,
        "interface_metadata_fields": sorted(interface) if isinstance(interface, Mapping) else [],
        "family_owned_metadata_fields": sorted(family_owned) if isinstance(family_owned, Mapping) else [],
        "findings": list(findings),
    }
    ####


def _verify_output_contract(endpoint: VehicleEndpointSpec, errors: list[str]) -> dict[str, object]:
    """Ensure required public state and telemetry are selectable for this exact endpoint."""

    applicable = _applicable_output_bindings(endpoint)
    missing_core = sorted(
        identifier
        for identifier in endpoint.required_core_output_ids
        if identifier not in applicable or applicable[identifier].channel_class != "core_state"
    )
    missing_telemetry = sorted(
        identifier
        for identifier in endpoint.required_telemetry_output_ids
        if identifier not in applicable or applicable[identifier].channel_class != "telemetry"
    )
    if missing_core:
        errors.append(f"endpoint {endpoint.id}: missing required core outputs {missing_core!r}")
    if missing_telemetry:
        errors.append(f"endpoint {endpoint.id}: missing required telemetry outputs {missing_telemetry!r}")
    return {
        "available_channel_ids": sorted(applicable),
        "required_core_output_ids": list(endpoint.required_core_output_ids),
        "required_telemetry_output_ids": list(endpoint.required_telemetry_output_ids),
        "missing_core_output_ids": missing_core,
        "missing_telemetry_output_ids": missing_telemetry,
        "status": "pass" if not missing_core and not missing_telemetry else "fail",
    }
    ####


def _applicable_output_bindings(endpoint: VehicleEndpointSpec) -> dict[str, NativeOutputBinding]:
    """Return the native-output bindings selected by one endpoint identity."""

    return {
        item.id: item
        for item in native_output_bindings(endpoint.model_id)
        if endpoint.fidelity in item.fidelities
        and (not item.mission_templates or endpoint.mission_id in item.mission_templates)
    }
    ####


def _verify_controller_campaign(
    endpoint: VehicleEndpointSpec,
    *,
    tune: bool,
    cache_dir: str | Path | None,
    errors: list[str],
) -> tuple[dict[str, object], TuningApplicationContextSet | None]:
    """Verify the optional common tuning seam without inventing a controller."""

    started = time.perf_counter()
    if endpoint.controller_campaign_id is None:
        return {
            "status": "not_applicable",
            "campaign_id": None,
            "tuning_status": "not_requested",
            "timings_s": {"controller_campaign_s": _elapsed_s(started)},
        }, None
    try:
        plugins = discover_plugins()
        providers = plugins.build_mission_composition_provider_registry()
        campaigns = plugins.build_controller_tuning_campaign_registry()
        campaigns.validate_against(providers)
        registration = campaigns.registration(endpoint.controller_campaign_id)
    except (KeyError, TypeError, ValueError) as error:
        errors.append(f"endpoint {endpoint.id}: controller campaign unavailable: {error}")
        return {
            "status": "fail",
            "campaign_id": endpoint.controller_campaign_id,
            "tuning_status": "not_run",
            "timings_s": {"controller_campaign_s": _elapsed_s(started)},
        }, None
    matches = registration.matches(
        provider_id=registration.provider_id,
        model_id=endpoint.model_id,
        fidelity=endpoint.fidelity,
        realization_id=endpoint.fidelity,
        mission_template_id=endpoint.mission_id,
    )
    if not matches or registration.family_id != endpoint.family_id:
        errors.append(f"endpoint {endpoint.id}: controller campaign does not match its family/model/fidelity/mission")
    raw_adapter_advertisement: Mapping[str, object]
    try:
        raw_adapter_advertisement = registration.adapter_advertisement()
    except (RuntimeError, TypeError, ValueError) as error:
        errors.append(f"endpoint {endpoint.id}: controller adapter advertisement is unavailable: {error}")
        raw_adapter_advertisement = {"status": "fail", "findings": [str(error)]}
    if raw_adapter_advertisement.get("status") != "available":
        errors.append(f"endpoint {endpoint.id}: controller adapter advertisement is not available")
    advertised_operations = raw_adapter_advertisement.get("operations")
    if not isinstance(advertised_operations, list) or not advertised_operations:
        errors.append(f"endpoint {endpoint.id}: controller adapter advertisement has no operations")
    usable_operations: set[str] = set()
    if isinstance(advertised_operations, list):
        for item in advertised_operations:
            if not isinstance(item, Mapping) or item.get("usable") is not True:
                continue
            operation = item.get("operation")
            if isinstance(operation, str):
                usable_operations.add(operation)
    missing_tuning_operations = sorted(set(endpoint.required_tuning_operations) - usable_operations)
    if missing_tuning_operations:
        errors.append(
            f"endpoint {endpoint.id}: controller adapter does not provide required tuning operations "
            f"{missing_tuning_operations!r}"
        )
    adapter_advertisement = _controller_adapter_advertisement_summary(raw_adapter_advertisement)
    campaign = registration.public_dict()
    local_screens = campaign.get("local_controller_screens")
    local_controller_screen_ids: list[str] = []
    if isinstance(local_screens, list):
        for item in local_screens:
            if not isinstance(item, Mapping):
                continue
            screen_id = item.get("id")
            if isinstance(screen_id, str):
                local_controller_screen_ids.append(screen_id)
    campaign_summary = {
        "id": campaign.get("id"),
        "provider_id": campaign.get("provider_id"),
        "model_id": campaign.get("model_id"),
        "family_id": campaign.get("family_id"),
        "fidelity": campaign.get("fidelity"),
        "realization_ids": campaign.get("realization_ids"),
        "mission_template_ids": campaign.get("mission_template_ids"),
        "local_controller_screen_ids": sorted(local_controller_screen_ids),
        "tuning_application_context": campaign.get("tuning_application_context"),
    }
    record: dict[str, object] = {
        "status": (
            "pass"
            if matches
            and registration.family_id == endpoint.family_id
            and raw_adapter_advertisement.get("status") == "available"
            and isinstance(advertised_operations, list)
            and bool(advertised_operations)
            and not missing_tuning_operations
            else "fail"
        ),
        "campaign_id": registration.id,
        "campaign": campaign_summary,
        "adapter_advertisement": adapter_advertisement,
        "required_tuning_operations": list(endpoint.required_tuning_operations),
        "missing_tuning_operations": missing_tuning_operations,
        "tuning_status": "not_requested",
        "campaign_registration_fingerprint_sha256": _canonical_json_fingerprint(campaign),
        "tuning_application_status": "not_requested",
    }
    execution_tuning_context_set: TuningApplicationContextSet | None = None
    if tune:
        tuning_started = time.perf_counter()
        try:
            cached = registration.run_cached(cache_dir)
            cache_disposition = "hit" if cached.cache_hit else "miss" if cached.cache_path is not None else "not_persisted"
            candidate_provenance = _candidate_configuration_provenance(cached.payload)
            candidate_status = cached.payload.get("status")
            record["tuning_status"] = "pass" if candidate_status == "candidate_ready" else "fail"
            record["cache_key"] = cached.cache_key
            record["cache_hit"] = cached.cache_hit
            record["cache_path"] = None if cached.cache_path is None else str(cached.cache_path)
            record["cache_disposition"] = cache_disposition
            record["candidate_status"] = candidate_status
            record.update(candidate_provenance)
            application_started = time.perf_counter()
            try:
                contexts = registration.application_contexts(cached)
            except (TypeError, ValueError) as error:
                record["tuning_application_status"] = "fail"
                record["tuning_application_error"] = str(error)
                errors.append(f"endpoint {endpoint.id}: tuned candidate cannot form an application context: {error}")
            else:
                record["tuning_application_status"] = "ready"
                record["tuning_application_contexts"] = [context.as_dict() for context in contexts]
                execution_tuning_context_set = TuningApplicationContextSet(tuple(contexts))
                if execution_tuning_context_set.singular is not None:
                    execution_tuning_context = execution_tuning_context_set.singular
                    record["execution_application_context"] = {
                        "status": "available",
                        "campaign_id": execution_tuning_context.campaign_id,
                        "node_id": execution_tuning_context.node_id,
                        "candidate_profile_id": execution_tuning_context.candidate_profile_id,
                    }
                else:
                    record["execution_application_context_set"] = {
                        "status": "available",
                        "campaign_id": execution_tuning_context_set.campaign_id,
                        "controller_method": execution_tuning_context_set.controller_method,
                        "node_ids": list(execution_tuning_context_set.node_ids),
                    }
            record["application_context_s"] = _elapsed_s(application_started)
            if candidate_status != "candidate_ready":
                errors.append(
                    f"endpoint {endpoint.id}: controller tuning returned {candidate_status!r}, not 'candidate_ready'"
                )
        except (OSError, RuntimeError, TypeError, ValueError) as error:
            record["tuning_status"] = "fail"
            record["tuning_application_status"] = "not_available"
            errors.append(f"endpoint {endpoint.id}: controller tuning failed: {error}")
        record["tuning_execution_s"] = _elapsed_s(tuning_started)
    record["timings_s"] = {"controller_campaign_s": _elapsed_s(started)}
    return record, execution_tuning_context_set
    ####


def _candidate_configuration_provenance(payload: Mapping[str, object]) -> dict[str, object]:
    """Fingerprint each selected campaign candidate so an execution can name it exactly."""

    selected: list[dict[str, object]] = []
    methods: set[str] = set()
    raw_nodes = payload.get("nodes")
    if not isinstance(raw_nodes, list):
        return {
            "candidate_payload_fingerprint_sha256": _canonical_json_fingerprint(payload),
            "candidate_methods": [],
            "selected_candidate_configurations": [],
        }
    for raw_node in raw_nodes:
        if not isinstance(raw_node, Mapping):
            continue
        node_id = raw_node.get("node_id")
        lqr = raw_node.get("lqr")
        if not isinstance(node_id, str) or not isinstance(lqr, Mapping):
            continue
        method = lqr.get("method")
        if isinstance(method, str):
            methods.add(method)
        profile_id = lqr.get("best_profile_id")
        candidates = lqr.get("candidates")
        if not isinstance(profile_id, str) or not isinstance(candidates, list):
            continue
        candidate = next(
            (
                item
                for item in candidates
                if isinstance(item, Mapping) and item.get("profile_id") == profile_id
            ),
            None,
        )
        if not isinstance(candidate, Mapping):
            continue
        selected.append(
            {
                "node_id": node_id,
                "profile_id": profile_id,
                "method": method,
                "candidate_configuration_fingerprint_sha256": _canonical_json_fingerprint(candidate),
            }
        )
    return {
        "candidate_payload_fingerprint_sha256": _canonical_json_fingerprint(payload),
        "candidate_methods": sorted(methods),
        "selected_candidate_configurations": selected,
    }
    ####


def _controller_adapter_advertisement_summary(advertisement: Mapping[str, object]) -> dict[str, object]:
    """Project the controller metadata needed for a readable endpoint report."""

    descriptor = advertisement.get("descriptor")
    operations = advertisement.get("operations")
    operation_records = tuple(item for item in operations if isinstance(item, Mapping)) if isinstance(operations, list) else ()
    return {
        "status": advertisement.get("status"),
        "adapter_id": descriptor.get("adapter_id") if isinstance(descriptor, Mapping) else None,
        "control_realization": descriptor.get("control_realization") if isinstance(descriptor, Mapping) else None,
        "state_channel_count": len(descriptor.get("state_channels", ())) if isinstance(descriptor, Mapping) and isinstance(descriptor.get("state_channels"), list) else 0,
        "control_channel_count": len(descriptor.get("control_channels", ())) if isinstance(descriptor, Mapping) and isinstance(descriptor.get("control_channels"), list) else 0,
        "resource_channel_count": len(descriptor.get("resource_channels", ())) if isinstance(descriptor, Mapping) and isinstance(descriptor.get("resource_channels"), list) else 0,
        "usable_operations": sorted(
            str(item["operation"])
            for item in operation_records
            if item.get("usable") is True and isinstance(item.get("operation"), str)
        ),
        "unavailable_operations": sorted(
            str(item["operation"])
            for item in operation_records
            if item.get("usable") is not True and isinstance(item.get("operation"), str)
        ),
    }
    ####


def _build_endpoint_acceptance_gates(
    records: Mapping[str, object],
    *,
    endpoint: VehicleEndpointSpec,
    execute: bool,
    tune: bool,
) -> dict[str, object]:
    """Report the ordered endpoint readiness chain without silently promoting partial evidence."""

    witness_records = _mapping_records(records.get("witnesses"))
    binding_records = _mapping_records(records.get("execution"))
    contract_findings: list[str] = []
    for name in ("selection", "interface", "outputs", "maturity"):
        if _record_status(records.get(name)) != "pass":
            contract_findings.append(f"{name} is not valid")
    controller_status = _record_status(records.get("controller"))
    if controller_status not in {"pass", "not_applicable"}:
        contract_findings.append("controller campaign advertisement is not valid")
    if not binding_records:
        contract_findings.append("no execution binding record exists")
    for item in binding_records:
        if _record_status(item) != "pass":
            contract_findings.append(f"operation {item.get('operation')!r} binding is not valid")
    if not witness_records:
        contract_findings.append("no executable witness record exists")
    for witness in witness_records:
        witness_id = witness.get("id")
        for field, expected in (
            ("compile_status", "pass"),
            ("identity_status", "pass"),
            ("resolved_binding_status", "pass"),
            ("preflight_status", "translation_ready"),
        ):
            if witness.get(field) != expected:
                contract_findings.append(f"witness {witness_id!r} {field} is not {expected!r}")
        if _record_status(witness.get("capability_advertisement")) != "pass":
            contract_findings.append(f"witness {witness_id!r} capability advertisement is not valid")
    contract_gate = {
        "status": "pass" if not contract_findings else "fail",
        "required_for_finalization": True,
        "findings": contract_findings,
    }

    execution_statuses = [item.get("execution_status") for item in witness_records]
    if not execute:
        batch_gate = {
            "status": "not_requested",
            "required_for_finalization": True,
            "reason": "run the exact endpoint with --execute to establish batch evidence",
        }
    elif execution_statuses and all(status == "pass" for status in execution_statuses):
        batch_gate = {
            "status": "pass",
            "required_for_finalization": True,
            "witness_count": len(witness_records),
        }
    else:
        batch_gate = {
            "status": "fail",
            "required_for_finalization": True,
            "execution_statuses": execution_statuses,
        }

    controller_record = records.get("controller")
    tuning_status = controller_record.get("tuning_status") if isinstance(controller_record, Mapping) else None
    batch_witness_records = _batch_evidence_witness_records(witness_records)
    binding_statuses = _tuning_binding_statuses(batch_witness_records)
    if controller_status == "not_applicable":
        tuner_gate = {
            "status": "not_applicable",
            "required_for_finalization": False,
            "reason": "the endpoint declares no controller campaign or controller runtime",
        }
    elif not tune:
        tuner_gate = {
            "status": "not_requested",
            "required_for_finalization": True,
            "reason": "run the declared campaign with --tune before asking whether the executed controller is bound",
        }
    elif not execute:
        tuner_gate = {
            "status": "not_exercised",
            "required_for_finalization": True,
            "reason": "candidate binding requires both --tune and --execute for the same endpoint report",
        }
    elif tuning_status != "pass":
        tuner_gate = {
            "status": "fail",
            "required_for_finalization": True,
            "reason": f"controller campaign tuning status is {tuning_status!r}",
        }
    elif binding_statuses and all(status == "candidate_bound" for status in binding_statuses):
        tuner_gate = {
            "status": "pass",
            "required_for_finalization": True,
            "binding_statuses": binding_statuses,
        }
    elif any(
        status in {"candidate_binding_mismatch", "candidate_not_ready", "execution_controller_not_verified", "method_mismatch"}
        for status in binding_statuses
    ):
        tuner_gate = {
            "status": "fail",
            "required_for_finalization": True,
            "binding_statuses": binding_statuses,
        }
    else:
        tuner_gate = {
            "status": "candidate_ready_not_runtime_bound",
            "required_for_finalization": True,
            "binding_statuses": binding_statuses,
            "reason": (
                "a campaign candidate exists, but the batch runtime has not declared the exact campaign node/profile/"
                "configuration fingerprint it applied"
            ),
        }

    tracking_statuses = [_record_status(item.get("tracking_evidence")) for item in batch_witness_records]
    if not execute:
        tracking_gate = {
            "status": "not_requested",
            "required_for_finalization": True,
            "reason": "run the exact batch witness to inspect its required local screen gates",
        }
    elif tracking_statuses and all(status == "pass" for status in tracking_statuses):
        tracking_gate = {
            "status": "pass",
            "required_for_finalization": True,
            "witness_count": len(batch_witness_records),
            "claim_boundary": "Pass is limited to the endpoint's declared local tracking/control screen.",
        }
    else:
        tracking_gate = {
            "status": "fail",
            "required_for_finalization": True,
            "tracking_statuses": tracking_statuses,
        }

    disturbance_statuses = [_record_status(item.get("disturbance_or_mass_evidence")) for item in batch_witness_records]
    if endpoint.robustness_requirement.disposition == "not_applicable":
        disturbance_gate = {
            "status": "not_applicable",
            "required_for_finalization": False,
            "screen_statuses": disturbance_statuses,
            "reason": endpoint.robustness_requirement.reason,
        }
    elif not execute:
        disturbance_gate = {
            "status": "not_requested",
            "required_for_finalization": True,
            "reason": "run the exact batch witness before looking for a declared disturbance or mass screen",
        }
    elif any(status == "fail" for status in disturbance_statuses):
        disturbance_gate = {
            "status": "fail",
            "required_for_finalization": True,
            "screen_statuses": disturbance_statuses,
        }
    elif disturbance_statuses and all(status == "pass" for status in disturbance_statuses):
        disturbance_gate = {
            "status": "pass",
            "required_for_finalization": True,
            "witness_count": len(batch_witness_records),
        }
    elif any(status == "blocked" for status in disturbance_statuses):
        disturbance_gate = {
            "status": "blocked",
            "required_for_finalization": True,
            "screen_statuses": disturbance_statuses,
            "reason": "a declared required disturbance or mass screen is blocked by its source-model capability",
        }
    else:
        disturbance_gate = {
            "status": "not_executed",
            "required_for_finalization": True,
            "screen_statuses": disturbance_statuses,
            "reason": "the endpoint's declared disturbance or mass screen has not yet emitted passing evidence",
        }

    gates: dict[str, Mapping[str, object]] = {
        "contract_valid": contract_gate,
        "batch_executed": batch_gate,
        "tuner_bound": tuner_gate,
        "tracking_passed": tracking_gate,
        "disturbance_or_mass_screened": disturbance_gate,
    }
    finalization_ready = all(
        _record_status(gate) == "pass"
        for gate in gates.values()
        if gate.get("required_for_finalization") is True
    )
    return {
        "schema": "taoryx.vehicle-endpoint-acceptance-gates/v1alpha1",
        "gate_order": list(gates),
        "gates": gates,
        "finalization_status": "ready" if finalization_ready else "incomplete",
        "claim_boundary": (
            "An endpoint is ready only when every ordered gate passes. 'not requested', 'not declared', and a "
            "method-only tuning match are intentionally incomplete rather than pass dispositions. A declared but "
            "not-yet-executed robustness contract is likewise incomplete rather than a robustness pass. A typed "
            "not-applicable robustness seam is excluded from finalization rather than counted as robustness evidence."
        ),
    }
    ####


def _mapping_records(value: object) -> list[Mapping[str, object]]:
    """Normalize a report list while excluding malformed entries from acceptance evidence."""

    return [item for item in value if isinstance(item, Mapping)] if isinstance(value, list) else []
    ####


def _record_status(record: object) -> object:
    """Read one mapping status without treating absent evidence as a pass."""

    return record.get("status") if isinstance(record, Mapping) else None
    ####


def _tuning_binding_statuses(witness_records: list[Mapping[str, object]]) -> list[object]:
    """Return every batch witness's explicit candidate-binding disposition."""

    statuses: list[object] = []
    for witness in witness_records:
        provenance = witness.get("tuning_provenance")
        binding = provenance.get("binding") if isinstance(provenance, Mapping) else None
        statuses.append(_record_status(binding))
    return statuses
    ####


def _batch_evidence_witness_records(witness_records: list[Mapping[str, object]]) -> list[Mapping[str, object]]:
    """Select the typed batch witnesses that can emit endpoint evidence.

    Endpoint operations may also include an interactive episode.  Such an
    episode is a separately validated public operation, but it cannot emit the
    batch packet artifacts consumed by tracking, robustness, or tuning-binding
    gates.  Keeping that boundary here prevents a valid episode from being
    mislabeled as missing unrelated evidence.
    """

    return [item for item in witness_records if item.get("operation") == "batch"]
    ####


def _build_endpoint_performance_record(
    records: Mapping[str, object],
    *,
    phase_durations_s: Mapping[str, float],
    total_duration_s: float,
    execute: bool,
    tune: bool,
) -> dict[str, object]:
    """Summarize focused endpoint phase timings and tuning-cache disposition.

    Timings are deliberately observational.  They help a plug-in author see
    whether iteration is dominated by compile/preflight, campaign synthesis,
    the actual batch, or packet indexing without presenting one machine's
    wall-clock numbers as a performance qualification.
    """

    phases = {name: float(value) for name, value in phase_durations_s.items() if _finite_number(value)}
    witness_records = _mapping_records(records.get("witnesses"))
    for witness in witness_records:
        timings = witness.get("timings_s")
        if not isinstance(timings, Mapping):
            continue
        for name, value in timings.items():
            if not isinstance(name, str) or not _finite_number(value):
                continue
            phase = f"witness.{name}"
            phases[phase] = phases.get(phase, 0.0) + float(value)

    controller = records.get("controller")
    cache: dict[str, object] = {
        "disposition": "not_requested" if not tune else "not_available",
        "key": None,
        "hit": None,
    }
    if isinstance(controller, Mapping):
        for source_name, phase_name in (
            ("tuning_execution_s", "controller.tuning_execution_s"),
            ("application_context_s", "controller.application_context_s"),
        ):
            duration = controller.get(source_name)
            if _finite_number(duration):
                phases[phase_name] = float(duration)
        cache["disposition"] = controller.get("cache_disposition", cache["disposition"])
        cache["key"] = controller.get("cache_key")
        cache["hit"] = controller.get("cache_hit")
        cache["path"] = controller.get("cache_path")

    slowest_phase = max(phases, key=phases.__getitem__) if phases else None
    return {
        "schema": "taoryx.vehicle-endpoint-performance/v1alpha1",
        "total_duration_s": total_duration_s,
        "phase_durations_s": dict(sorted(phases.items())),
        "slowest_phase": (
            None
            if slowest_phase is None
            else {"id": slowest_phase, "duration_s": phases[slowest_phase]}
        ),
        "cache": cache,
        "execution_requested": execute,
        "tuning_requested": tune,
        "claim_boundary": (
            "These are local wall-clock observations for the selected endpoint and cache state. They are not a "
            "benchmark, real-time guarantee, or controller-performance claim."
        ),
    }
    ####


def _elapsed_s(started: float) -> float:
    """Return one non-negative elapsed wall-clock duration with report-friendly precision."""

    return round(max(time.perf_counter() - started, 0.0), 6)
    ####


def _verify_maturity_record(endpoint: VehicleEndpointSpec, errors: list[str]) -> dict[str, object]:
    """Keep the planning ledger visible without letting it imply runtime readiness."""

    record = next(
        (item for item in _maturity_records() if item.get("id") == endpoint.maturity_record_id),
        None,
    )
    if not isinstance(record, Mapping):
        errors.append(f"endpoint {endpoint.id}: maturity record {endpoint.maturity_record_id!r} is missing")
        return {"status": "fail", "record_id": endpoint.maturity_record_id}
    family_id = record.get("composition_family_id")
    if family_id not in {None, endpoint.family_id}:
        errors.append(
            f"endpoint {endpoint.id}: maturity record family {family_id!r} does not match {endpoint.family_id!r}"
        )
    return {
        "status": "pass" if family_id in {None, endpoint.family_id} else "fail",
        "record": dict(record),
        "claim_boundary": (
            "Family maturity is a planning ledger. It is not substituted for the endpoint's executable or controller evidence."
        ),
    }
    ####


def __getattr__(name: str) -> object:
    """Resolve historical aggregate path constants only on explicit access."""

    relative_paths = {
        "VEHICLE_ENDPOINT_SPECS": "verification/vehicle_endpoint_specs.yaml",
        "VEHICLE_MATURITY_REGISTRY": "verification/vehicle_maturity_registry.yaml",
    }
    try:
        relative_path = relative_paths[name]
    except KeyError as error:
        raise AttributeError(name) from error
    from .compatibility.vehicle_catalog_resources import legacy_vehicle_catalog_resource

    value = legacy_vehicle_catalog_resource(relative_path)
    globals()[name] = value
    return value
    ####


__all__ = [
    "VEHICLE_ENDPOINT_SPECS",
    "VehicleEndpointOperationSpec",
    "VehicleEndpointRobustnessCaseSpec",
    "VehicleEndpointRobustnessMetricSpec",
    "VehicleEndpointRobustnessScreenSpec",
    "VehicleEndpointSpec",
    "VehicleEndpointSpecCatalog",
    "load_vehicle_endpoint_spec_catalog",
    "vehicle_endpoint_spec_list",
    "verify_vehicle_endpoint",
]
