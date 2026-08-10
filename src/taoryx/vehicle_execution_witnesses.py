"""Executable composition witnesses for every advertised runtime binding.

The execution-binding catalog declares what *could* be selected.  This module
adds the complementary onboarding proof: every currently runnable exact tuple
must have one checked-in composition request that compiles, passes translation
preflight, and resolves that exact factory without borrowing another family.
"""

from __future__ import annotations

import hashlib
import json
import shlex
import tempfile
from collections.abc import Iterable, Mapping
from contextlib import nullcontext, redirect_stdout
from io import StringIO
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .vehicle_composition import CompiledVehicleComposition, compile_vehicle_composition, load_vehicle_composition_request
from .vehicle_composition_registry import (
    ResolvedVehicleCompositionCatalog,
    load_resolved_vehicle_composition_catalog,
    mission_graph_execution_contract,
)
from .vehicle_execution_bindings import (
    ExecutionOperation,
    VehicleExecutionBinding,
    load_vehicle_execution_binding_catalog,
    resolve_vehicle_execution_binding,
)
from .vehicle_execution_preflight import preflight_vehicle_composition, validate_public_capability_advertisement
from .vehicle_registry import ROOT
from .vehicle_runtime_lowering import lower_vehicle_composition

VEHICLE_EXECUTION_WITNESSES = ROOT / "verification/vehicle_execution_witnesses.yaml"

# These executions deliberately have no common control screen.  A release
# packet must preserve that boundary as explicit ``not_applicable`` evidence;
# accepting missing or fabricated controller evidence would make a source
# replay or passive witness look more capable than it is.
_CONTROL_FREE_EXECUTION_MODES = frozenset(
    {
        "open_loop_witness",
        "passive_uncontrolled",
        "source_history_replay",
        "source_scheduled_replay",
    }
)

# An implementation-owned route can execute an autonomous policy and expose
# its accepted-interval semantic action trace without claiming the common
# LQR/LQI control-screen protocol.  It is distinct from a replay or passive
# execution: verified native action-trace evidence is required here.
_NATIVE_AUTONOMOUS_EXECUTION_MODES = frozenset({"source_native_autonomous", "native_autonomous"})


class VehicleExecutionWitness(BaseModel):
    """One checked-in semantic request for an exact public endpoint."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    composition: str = Field(min_length=1)
    operation: ExecutionOperation


class VehicleVariantWitness(BaseModel):
    """One checked-in composition that exercises runtime-bound variants."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    composition: str = Field(min_length=1)
    variant_ids: tuple[str, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_unique_variant_ids(self) -> VehicleVariantWitness:
        if len(self.variant_ids) != len(set(self.variant_ids)):
            raise ValueError(f"variant witness {self.id!r} has duplicate variant IDs")
        return self
        ####


class VehicleGraphExtensionWitness(BaseModel):
    """One family-owned non-success graph capability exercised by a public run.

    This is deliberately separate from the one-witness-per-endpoint matrix.
    It proves a registered endpoint accepts and reports a declared graph
    extension; it neither duplicates an endpoint witness nor turns recovery
    behavior into a successful mission qualification claim.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    composition: str = Field(min_length=1)
    required_transition_kind: Literal["timeout", "abort", "resource_limit", "envelope_limit"]
    expected_observed_outcome: Literal["success", "abort", "resource_limit", "envelope_limit", "timeout"] = "success"


####


####


class VehicleExecutionWitnessCatalog(BaseModel):
    """Versioned witness mapping for the public execution-binding matrix."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_id: str = Field(alias="schema", min_length=1)
    version: int = Field(ge=1)
    description: str = Field(min_length=1)
    witnesses: tuple[VehicleExecutionWitness, ...] = Field(min_length=1)
    variant_witnesses: tuple[VehicleVariantWitness, ...] = ()
    graph_extension_witnesses: tuple[VehicleGraphExtensionWitness, ...] = ()

    @model_validator(mode="after")
    def validate_unique_ids(self) -> VehicleExecutionWitnessCatalog:
        ids = tuple(item.id for item in self.witnesses)
        if len(ids) != len(set(ids)):
            raise ValueError("execution witness catalog has duplicate IDs")
        variant_ids = tuple(item.id for item in self.variant_witnesses)
        if len(variant_ids) != len(set(variant_ids)):
            raise ValueError("execution witness catalog has duplicate variant-witness IDs")
        graph_extension_ids = tuple(item.id for item in self.graph_extension_witnesses)
        if len(graph_extension_ids) != len(set(graph_extension_ids)):
            raise ValueError("execution witness catalog has duplicate graph-extension witness IDs")
        all_ids = (*ids, *variant_ids, *graph_extension_ids)
        if len(all_ids) != len(set(all_ids)):
            raise ValueError("execution witness catalog reuses an ID across witness categories")
        return self
        ####

    ####


def load_vehicle_execution_witness_catalog(
    path: str | Path | None = None,
) -> VehicleExecutionWitnessCatalog:
    """Load the checked-in composition witness catalog."""

    source = Path(path) if path is not None else VEHICLE_EXECUTION_WITNESSES
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{source} must contain a mapping")
    return VehicleExecutionWitnessCatalog.model_validate(payload)
    ####


def validate_vehicle_execution_witnesses(
    catalog: VehicleExecutionWitnessCatalog | None = None,
    *,
    execute_batch: bool = False,
    family_ids: Iterable[str] | None = None,
    witness_ids: Iterable[str] | None = None,
    retained_results_directory: str | Path | None = None,
) -> dict[str, object]:
    """Validate every runnable execution binding has an exact composition witness.

    The validation intentionally stops before batch integration: it is an
    onboarding and endpoint-contract gate, not a substitute for full mission
    execution or qualification. With ``execute_batch=True``, it additionally
    drives the public compose-to-run command for every batch witness. This is
    deliberately opt-in because it is an integration smoke, not a fast static
    registry check. ``retained_results_directory`` additionally keeps every
    generated batch packet in a caller-selected empty directory and writes one
    aggregate, hash-bound release catalog; it does not change evidence scope.
    Interactive witnesses are opened once because their endpoint contract
    includes a concrete accepted-truth episode.
    """

    witness_catalog = catalog or load_vehicle_execution_witness_catalog()
    family_filter = _normalized_family_filter(family_ids)
    witness_filter = _normalized_witness_filter(witness_ids)
    batch_output_root = _prepare_retained_results_directory(retained_results_directory, execute_batch=execute_batch)
    binding_catalog = load_vehicle_execution_binding_catalog()
    composition_catalog = load_resolved_vehicle_composition_catalog()
    known_families = {item.family_id for item in binding_catalog.bindings}
    known_witnesses = {item.id for item in witness_catalog.witnesses}
    requested_unknown_families = () if family_filter is None else tuple(sorted(family_filter - known_families))
    requested_unknown_witnesses = () if witness_filter is None else tuple(sorted(witness_filter - known_witnesses))
    runnable = tuple(
        item
        for item in binding_catalog.bindings
        if item.status == "runnable" and (family_filter is None or item.family_id in family_filter)
    )
    binding_by_key: dict[tuple[str, str, str, ExecutionOperation], VehicleExecutionBinding] = {_binding_key(item): item for item in runnable}
    witness_keys: dict[tuple[str, str, str, ExecutionOperation], str] = {}
    records: list[dict[str, object]] = []
    errors: list[str] = [f"unknown requested vehicle family: {family_id}" for family_id in requested_unknown_families]
    errors.extend(f"unknown requested execution witness: {witness_id}" for witness_id in requested_unknown_witnesses)

    for witness in witness_catalog.witnesses:
        if witness_filter is not None and witness.id not in witness_filter:
            continue
        source = ROOT / witness.composition
        if not source.is_file():
            errors.append(f"{witness.id}: composition request is missing: {witness.composition}")
            continue
        try:
            composition = compile_vehicle_composition(
                load_vehicle_composition_request(source),
                catalog=composition_catalog,
            )
        except (TypeError, ValueError) as error:
            errors.append(f"{witness.id}: composition does not compile: {error}")
            continue
        if family_filter is not None and composition.family_id not in family_filter:
            continue
        key = (composition.family_id, composition.mission, composition.fidelity, witness.operation)
        if key in witness_keys:
            errors.append(f"{witness.id}: duplicates endpoint witness {witness_keys[key]!r}")
            continue
        witness_keys[key] = witness.id
        binding = binding_by_key.get(key)
        if binding is None:
            errors.append(f"{witness.id}: no runnable execution binding for {_format_key(key)}")
            continue
        try:
            resolved = resolve_vehicle_execution_binding(composition, witness.operation)
        except ValueError as error:
            errors.append(f"{witness.id}: endpoint resolution failed: {error}")
            continue
        preflight = preflight_vehicle_composition(composition)
        if preflight.status != "translation_ready":
            errors.append(f"{witness.id}: expected translation_ready preflight, got {preflight.status}")
        capability_preflight = _validate_concrete_capability_preflight(
            preflight,
            composition,
            context=witness.id,
            errors=errors,
        )
        lowering = lower_vehicle_composition(composition, preflight_result=preflight)
        if lowering.status not in {"adapter_bound", "factory_bound"}:
            errors.append(f"{witness.id}: expected a runtime binding after preflight, got {lowering.status}")
        episode_opened = False
        episode_contract: dict[str, object] | None = None
        batch_execution: dict[str, object] | None = None
        if witness.operation == "episode":
            try:
                from .composition_episode import (
                    open_vehicle_composition_episode,
                    validate_vehicle_composition_episode_contract,
                )

                episode = open_vehicle_composition_episode(composition)
                episode_contract = validate_vehicle_composition_episode_contract(episode)
                if episode_contract["status"] != "pass":
                    raw_findings = episode_contract.get("findings", [])
                    findings = raw_findings if isinstance(raw_findings, list | tuple) else [raw_findings]
                    detail = "; ".join(str(item) for item in findings)
                    errors.append(f"{witness.id}: episode semantic-contract validation failed: {detail}")
                episode.close()
                episode_opened = True
            except (TypeError, ValueError) as error:
                errors.append(f"{witness.id}: declared episode did not open: {error}")
        elif execute_batch:
            batch_execution = _run_batch_witness(
                witness.id,
                composition,
                binding=resolved,
                output_root=batch_output_root,
            )
            if batch_execution["status"] != "pass":
                errors.append(f"{witness.id}: public batch execution failed: {batch_execution['detail']}")
        records.append(
            {
                "id": witness.id,
                "composition": witness.composition,
                "family_id": composition.family_id,
                "mission": composition.mission,
                "fidelity": composition.fidelity,
                "operation": witness.operation,
                "factory_id": resolved.factory_id,
                "batch_action_trace_disposition": resolved.batch_action_trace,
                "preflight_status": preflight.status,
                "capability_preflight": capability_preflight,
                "lowering_status": lowering.status,
                "lowering_execution_binding": lowering.execution_binding,
                "episode_opened": episode_opened if witness.operation == "episode" else None,
                "episode_contract": episode_contract,
                "batch_execution": batch_execution,
            }
        )

    if witness_filter is None:
        for binding_key in sorted(binding_by_key):
            if binding_key not in witness_keys:
                errors.append(f"runnable execution binding has no composition witness: {_format_key(binding_key)}")
    for witness_key in sorted(witness_keys):
        if witness_key not in binding_by_key:
            errors.append(f"composition witness targets an unavailable endpoint: {_format_key(witness_key)}")
    variant_report = (
        _empty_variant_witness_report(execute_batch=execute_batch, family_filter=family_filter)
        if witness_filter is not None
        else validate_vehicle_variant_witnesses(
            witness_catalog,
            composition_catalog=composition_catalog,
            execute_batch=execute_batch,
            family_ids=family_filter,
            batch_output_root=batch_output_root,
        )
    )
    variant_errors = variant_report["errors"]
    variant_records = variant_report["records"]
    if not isinstance(variant_errors, list) or not all(isinstance(item, str) for item in variant_errors):
        raise ValueError("variant witness report has invalid errors")
    if not isinstance(variant_records, list):
        raise ValueError("variant witness report has invalid records")
    errors.extend(variant_errors)
    graph_extension_report = (
        _empty_graph_extension_witness_report(execute_batch=execute_batch, family_filter=family_filter)
        if witness_filter is not None
        else validate_vehicle_graph_extension_witnesses(
            witness_catalog,
            composition_catalog=composition_catalog,
            execute_batch=execute_batch,
            family_ids=family_filter,
            batch_output_root=batch_output_root,
        )
    )
    graph_extension_errors = graph_extension_report["errors"]
    graph_extension_records = graph_extension_report["records"]
    if not isinstance(graph_extension_errors, list) or not all(isinstance(item, str) for item in graph_extension_errors):
        raise ValueError("graph-extension witness report has invalid errors")
    if not isinstance(graph_extension_records, list):
        raise ValueError("graph-extension witness report has invalid records")
    errors.extend(graph_extension_errors)
    retained_result_corpus = _finalize_retained_results_directory(batch_output_root)
    if retained_result_corpus is not None and retained_result_corpus["status"] != "pass":
        errors.append(f"retained batch result corpus failed: {retained_result_corpus['detail']}")
    return {
        "schema": "taoryx.vehicle-execution-witness-report/v1alpha1",
        "status": "pass" if not errors else "fail",
        "runnable_binding_count": len(witness_keys) if witness_filter is not None else len(runnable),
        "witness_count": len(records),
        "runnable_variant_count": variant_report["runnable_variant_count"],
        "variant_witness_count": variant_report["variant_witness_count"],
        "graph_extension_witness_count": graph_extension_report["graph_extension_witness_count"],
        "batch_execution_smoke": execute_batch,
        "retained_result_corpus": retained_result_corpus,
        "family_filter": None if family_filter is None else sorted(family_filter),
        "witness_filter": None if witness_filter is None else sorted(witness_filter),
        "records": records,
        "variant_records": variant_records,
        "graph_extension_records": graph_extension_records,
        "errors": errors,
        "claim_boundary": (
            "This verifies exact composition-to-endpoint discoverability, runtime-bound variant witnesses, declared family graph-extension witnesses, semantic preflight, runtime lowering, and interactive endpoint construction. "
            "When batch_execution_smoke is enabled, it also verifies that each batch artifact matches its declared action-trace disposition and can form a hash-bound release packet with a verified reproduction identity. "
            "Neither mode establishes numerical parity or promotes qualification evidence."
        ),
    }
    ####


def validate_vehicle_variant_witnesses(
    catalog: VehicleExecutionWitnessCatalog | None = None,
    *,
    composition_catalog: ResolvedVehicleCompositionCatalog | None = None,
    execute_batch: bool = False,
    family_ids: Iterable[str] | None = None,
    batch_output_root: Path | None = None,
) -> dict[str, object]:
    """Validate every runnable variant has one exact composed witness.

    The default proves composition, semantic preflight, and declared runtime
    handoff for a bounded modifier. With ``execute_batch`` it also runs the
    exact public batch endpoint and requires the generated packet's runtime
    evidence and normalized result-catalog record to validate. Neither mode
    establishes retrim validity or qualification.
    """

    witness_catalog = catalog or load_vehicle_execution_witness_catalog()
    family_filter = _normalized_family_filter(family_ids)
    resolved_catalog = composition_catalog or load_resolved_vehicle_composition_catalog()
    runnable_variants = {
        (vehicle.family.family_id, variant.id)
        for vehicle in resolved_catalog.vehicles
        for variant in vehicle.declaration.variant_parameters
        if variant.status == "runnable" and (family_filter is None or vehicle.family.family_id in family_filter)
    }
    witnessed_variants: dict[tuple[str, str], str] = {}
    records: list[dict[str, object]] = []
    errors: list[str] = []
    for witness in witness_catalog.variant_witnesses:
        source = ROOT / witness.composition
        if not source.is_file():
            errors.append(f"{witness.id}: variant composition request is missing: {witness.composition}")
            continue
        try:
            composition = compile_vehicle_composition(
                load_vehicle_composition_request(source),
                catalog=resolved_catalog,
            )
        except (TypeError, ValueError) as error:
            errors.append(f"{witness.id}: variant composition does not compile: {error}")
            continue
        if family_filter is not None and composition.family_id not in family_filter:
            continue
        declared = tuple(sorted(witness.variant_ids))
        actual = tuple(sorted(composition.variant.inputs))
        if actual != declared:
            errors.append(f"{witness.id}: declared variant IDs {declared!r} do not match composition inputs {actual!r}")
        for identifier in actual:
            variant_key = (composition.family_id, identifier)
            if variant_key in witnessed_variants:
                errors.append(f"{witness.id}: duplicates variant witness {witnessed_variants[variant_key]!r} for {variant_key!r}")
            witnessed_variants[variant_key] = witness.id
            if variant_key not in runnable_variants:
                errors.append(f"{witness.id}: composition selects an undeclared runnable variant {variant_key!r}")
        preflight = preflight_vehicle_composition(composition)
        capability_preflight = _validate_concrete_capability_preflight(
            preflight,
            composition,
            context=witness.id,
            errors=errors,
        )
        lowering = lower_vehicle_composition(composition, preflight_result=preflight)
        if preflight.status != "translation_ready":
            errors.append(f"{witness.id}: variant composition expected translation_ready preflight, got {preflight.status}")
        if lowering.status not in {"adapter_bound", "factory_bound"}:
            errors.append(f"{witness.id}: variant composition expected a runtime binding, got {lowering.status}")
        batch_binding: VehicleExecutionBinding | None = None
        try:
            batch_binding = resolve_vehicle_execution_binding(composition, "batch")
        except ValueError as error:
            errors.append(f"{witness.id}: runnable variant has no public batch endpoint: {error}")
        batch_execution: dict[str, object] | None = None
        if execute_batch and batch_binding is not None:
            try:
                batch_execution = _run_batch_witness(
                    witness.id,
                    composition,
                    binding=batch_binding,
                    output_root=batch_output_root,
                )
                if batch_execution["status"] != "pass":
                    errors.append(f"{witness.id}: variant batch execution failed: {batch_execution['detail']}")
            except ValueError as error:
                errors.append(f"{witness.id}: variant batch execution is invalid: {error}")
        records.append(
            {
                "id": witness.id,
                "composition": witness.composition,
                "family_id": composition.family_id,
                "variant_ids": list(actual),
                "runtime_bindings": {identifier: binding.model_dump(mode="json") for identifier, binding in composition.variant.runtime_bindings.items()},
                "invalidations": list(composition.variant.invalidations),
                "preflight_status": preflight.status,
                "capability_preflight": capability_preflight,
                "lowering_status": lowering.status,
                "batch_factory_id": None if batch_binding is None else batch_binding.factory_id,
                "batch_execution": batch_execution,
            }
        )
    for variant_key in sorted(runnable_variants):
        if variant_key not in witnessed_variants:
            errors.append(f"runnable variant has no composition witness: {variant_key[0]}/{variant_key[1]}")
    return {
        "schema": "taoryx.vehicle-variant-witness-report/v1alpha1",
        "status": "pass" if not errors else "fail",
        "runnable_variant_count": len(runnable_variants),
        "variant_witness_count": len(records),
        "batch_execution_smoke": execute_batch,
        "family_filter": None if family_filter is None else sorted(family_filter),
        "records": records,
        "errors": errors,
        "claim_boundary": (
            "This verifies the composed semantic handoff for every advertised runtime-bound variant. "
            "When batch_execution_smoke is enabled, it also verifies declared native-input consumption/status "
            "evidence and release-packet/reproduction conformance through the selected public batch endpoint. Neither mode establishes retrim validity or qualification."
        ),
    }
    ####


def validate_vehicle_graph_extension_witnesses(
    catalog: VehicleExecutionWitnessCatalog | None = None,
    *,
    composition_catalog: ResolvedVehicleCompositionCatalog | None = None,
    execute_batch: bool = False,
    family_ids: Iterable[str] | None = None,
    batch_output_root: Path | None = None,
) -> dict[str, object]:
    """Validate declared non-success graph extensions without generic branches.

    A graph-extension witness is not another endpoint witness. It proves that
    one family-owned composed graph exposes a declared alternate transition,
    preflights and lowers through that same family translator, and—when batch
    smoke is requested—emits observed graph evidence from the public runner.
    The nominal public run may still follow success edges; the required
    alternate transition remains a declared recovery capability, not a hidden
    timeout-as-success assertion.
    """

    witness_catalog = catalog or load_vehicle_execution_witness_catalog()
    family_filter = _normalized_family_filter(family_ids)
    resolved_catalog = composition_catalog or load_resolved_vehicle_composition_catalog()
    records: list[dict[str, object]] = []
    errors: list[str] = []
    for witness in witness_catalog.graph_extension_witnesses:
        source = ROOT / witness.composition
        if not source.is_file():
            errors.append(f"{witness.id}: graph-extension composition request is missing: {witness.composition}")
            continue
        try:
            composition = compile_vehicle_composition(
                load_vehicle_composition_request(source),
                catalog=resolved_catalog,
            )
        except (TypeError, ValueError) as error:
            errors.append(f"{witness.id}: graph-extension composition does not compile: {error}")
            continue
        if family_filter is not None and composition.family_id not in family_filter:
            continue
        graph = composition.mission_graph
        if graph is None:
            errors.append(f"{witness.id}: graph-extension composition has no compiled mission graph")
            continue
        graph_contract = mission_graph_execution_contract(
            composition.family_id,
            composition.mission,
            composition.fidelity,
        )
        if graph_contract.get("status") != "family_extension_declared":
            errors.append(f"{witness.id}: selected family/mission/fidelity has no declared graph execution extension")
        transition_kinds = graph_contract.get("supported_transition_kinds")
        if not isinstance(transition_kinds, list) or witness.required_transition_kind not in transition_kinds:
            errors.append(
                f"{witness.id}: graph extension does not declare required {witness.required_transition_kind!r} transition support"
            )
        if graph.status in {"linear_sequence_only", "authored_linear_sequence_lowered"}:
            errors.append(f"{witness.id}: graph-extension witness is only a linear success sequence")
        if not any(
            _graph_transition_for_kind(node, witness.required_transition_kind) is not None
            for node in graph.nodes
        ):
            errors.append(
                f"{witness.id}: compiled graph has no declared {witness.required_transition_kind!r} transition"
            )
        preflight = preflight_vehicle_composition(composition)
        capability_preflight = _validate_concrete_capability_preflight(
            preflight,
            composition,
            context=witness.id,
            errors=errors,
        )
        lowering = lower_vehicle_composition(composition, preflight_result=preflight)
        if preflight.status != "translation_ready":
            errors.append(f"{witness.id}: graph-extension composition expected translation_ready preflight, got {preflight.status}")
        if lowering.status not in {"adapter_bound", "factory_bound"}:
            errors.append(f"{witness.id}: graph-extension composition expected a runtime binding, got {lowering.status}")
        batch_binding: VehicleExecutionBinding | None = None
        try:
            batch_binding = resolve_vehicle_execution_binding(composition, "batch")
        except ValueError as error:
            errors.append(f"{witness.id}: graph-extension composition has no public batch endpoint: {error}")
        batch_execution: dict[str, object] | None = None
        if execute_batch and batch_binding is not None:
            batch_execution = _run_batch_witness(
                witness.id,
                composition,
                binding=batch_binding,
                output_root=batch_output_root,
            )
            if batch_execution.get("status") != "pass":
                errors.append(f"{witness.id}: graph-extension batch execution failed: {batch_execution.get('detail')}")
            graph_execution = batch_execution.get("graph_execution")
            if not isinstance(graph_execution, Mapping):
                errors.append(f"{witness.id}: graph-extension batch execution did not retain graph evidence")
            else:
                if graph_execution.get("observation_status") != "observed":
                    errors.append(f"{witness.id}: graph-extension batch execution did not observe graph dispatches")
                outcomes = graph_execution.get("outcomes")
                if not isinstance(outcomes, list) or witness.expected_observed_outcome not in outcomes:
                    errors.append(
                        f"{witness.id}: observed graph outcomes do not include {witness.expected_observed_outcome!r}"
                    )
        records.append(
            {
                "id": witness.id,
                "composition": witness.composition,
                "family_id": composition.family_id,
                "mission": composition.mission,
                "fidelity": composition.fidelity,
                "graph_status": graph.status,
                "required_transition_kind": witness.required_transition_kind,
                "supported_transition_kinds": transition_kinds,
                "preflight_status": preflight.status,
                "capability_preflight": capability_preflight,
                "lowering_status": lowering.status,
                "batch_factory_id": None if batch_binding is None else batch_binding.factory_id,
                "batch_execution": batch_execution,
            }
        )
    return {
        "schema": "taoryx.vehicle-graph-extension-witness-report/v1alpha1",
        "status": "pass" if not errors else "fail",
        "graph_extension_witness_count": len(records),
        "batch_execution_smoke": execute_batch,
        "family_filter": None if family_filter is None else sorted(family_filter),
        "records": records,
        "errors": errors,
        "claim_boundary": (
            "This verifies declared family-owned alternate graph-transition support and observed graph evidence. "
            "It does not turn a recovery edge into mission success or promote the family to qualification."
        ),
    }
    ####


def _validate_concrete_capability_preflight(
    preflight: object,
    composition: CompiledVehicleComposition,
    *,
    context: str,
    errors: list[str],
) -> dict[str, object] | None:
    """Validate the family-owned feasibility evidence behind endpoint readiness.

    ``translation_ready`` is not accepted as a bare boolean for an advertised
    runtime endpoint.  The witness must retain the exact capability adapter,
    feasibility label, selection identity, and fingerprint of the derived
    mission that the selected semantic translator consumed.
    """

    status = getattr(preflight, "status", None)
    if status != "translation_ready":
        return None
    capability = getattr(preflight, "capability_estimate", None)
    derived_mission = getattr(preflight, "derived_mission", None)
    translator_id = getattr(preflight, "translator_id", None)
    if not isinstance(capability, Mapping):
        errors.append(f"{context}: translation-ready preflight omitted concrete capability evidence")
        return None
    if not isinstance(derived_mission, Mapping):
        errors.append(f"{context}: translation-ready preflight omitted its derived mission")
        return None
    expected = {
        "schema": "taoryx.concrete-capability-preflight/v1alpha1",
        "composition_id": composition.id,
        "composition_identity_sha256": composition.identity_sha256,
        "family_id": composition.family_id,
        "mission_id": composition.mission,
        "fidelity": composition.fidelity,
        "semantic_translator_id": translator_id,
    }
    for key, value in expected.items():
        if capability.get(key) != value:
            errors.append(
                f"{context}: concrete capability preflight {key!r} is {capability.get(key)!r}, expected {value!r}"
            )
    feasibility = capability.get("feasibility")
    if not isinstance(capability.get("adapter_id"), str) or not str(capability["adapter_id"]).strip():
        errors.append(f"{context}: concrete capability preflight has no capability adapter ID")
    if feasibility not in {
        "feasible",
        "likely_feasible",
        "unknown",
        "likely_infeasible",
        "certainly_infeasible",
    }:
        errors.append(f"{context}: concrete capability preflight has invalid feasibility {feasibility!r}")
    advertisement = capability.get("capability_advertisement")
    if not isinstance(advertisement, Mapping):
        errors.append(f"{context}: concrete capability preflight omitted its generic capability advertisement")
    else:
        for finding in validate_public_capability_advertisement(
            advertisement,
            expected_selection={
                "composition_id": composition.id,
                "composition_identity_sha256": composition.identity_sha256,
                "vehicle_id": composition.vehicle_id,
                "family_id": composition.family_id,
                "mission_id": composition.mission,
                "fidelity": composition.fidelity,
            },
        ):
            errors.append(f"{context}: {finding}")
    try:
        encoded = json.dumps(
            derived_mission,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
        ).encode("utf-8")
    except (TypeError, ValueError) as error:
        errors.append(f"{context}: derived mission is not fingerprintable: {error}")
    else:
        observed = capability.get("derived_mission_sha256")
        expected_sha256 = hashlib.sha256(encoded).hexdigest()
        if observed != expected_sha256:
            errors.append(
                f"{context}: concrete capability preflight has mismatched derived-mission fingerprint "
                f"{observed!r}, expected {expected_sha256!r}"
            )
    return {
        "adapter_id": capability.get("adapter_id"),
        "feasibility": feasibility,
        "capability_advertisement": dict(advertisement) if isinstance(advertisement, Mapping) else None,
        "derived_mission_sha256": capability.get("derived_mission_sha256"),
    }
    ####


def _binding_key(binding: VehicleExecutionBinding) -> tuple[str, str, str, ExecutionOperation]:
    return (binding.family_id, binding.mission, binding.fidelity, binding.operation)
    ####


def _normalized_family_filter(family_ids: Iterable[str] | None) -> frozenset[str] | None:
    """Return an explicit family filter or ``None`` for the full witness matrix."""

    if family_ids is None:
        return None
    normalized = frozenset(str(family_id).strip() for family_id in family_ids if str(family_id).strip())
    if not normalized:
        raise ValueError("family_ids must contain at least one non-empty vehicle family ID")
    return normalized
    ####


def _normalized_witness_filter(witness_ids: Iterable[str] | None) -> frozenset[str] | None:
    """Return an explicit endpoint-witness filter or ``None`` for the full matrix."""

    if witness_ids is None:
        return None
    normalized = frozenset(str(witness_id).strip() for witness_id in witness_ids if str(witness_id).strip())
    if not normalized:
        raise ValueError("witness_ids must contain at least one non-empty execution witness ID")
    return normalized
    ####


def _empty_variant_witness_report(
    *,
    execute_batch: bool,
    family_filter: frozenset[str] | None,
) -> dict[str, object]:
    """Describe intentionally omitted variant work for an endpoint-only witness run."""

    return {
        "runnable_variant_count": 0,
        "variant_witness_count": 0,
        "batch_execution_smoke": execute_batch,
        "family_filter": None if family_filter is None else sorted(family_filter),
        "records": [],
        "errors": [],
    }
    ####


def _empty_graph_extension_witness_report(
    *,
    execute_batch: bool,
    family_filter: frozenset[str] | None,
) -> dict[str, object]:
    """Describe intentionally omitted graph extensions for an endpoint-only run."""

    return {
        "graph_extension_witness_count": 0,
        "batch_execution_smoke": execute_batch,
        "family_filter": None if family_filter is None else sorted(family_filter),
        "records": [],
        "errors": [],
    }
    ####


def _graph_transition_for_kind(node: object, kind: str) -> object | None:
    """Return only the explicitly declared edge for one graph outcome."""

    field_by_kind = {
        "timeout": "timeout_transition",
        "abort": "abort_transition",
        "resource_limit": "resource_limit_transition",
        "envelope_limit": "envelope_limit_transition",
    }
    field = field_by_kind.get(kind)
    return None if field is None else getattr(node, field, None)
    ####


def _format_key(key: tuple[str, str, str, ExecutionOperation]) -> str:
    family, mission, fidelity, operation = key
    return f"{family}/{mission}/{fidelity}/{operation}"
    ####


def _prepare_retained_results_directory(
    directory: str | Path | None,
    *,
    execute_batch: bool,
) -> Path | None:
    """Create one empty caller-owned root for retained public batch packets."""

    if directory is None:
        return None
    if not execute_batch:
        raise ValueError("retained batch results require execute_batch=True")
    root = Path(directory)
    if root.exists():
        if not root.is_dir():
            raise ValueError(f"retained batch result path is not a directory: {root}")
        if any(root.iterdir()):
            raise ValueError(f"retained batch result directory must be empty: {root}")
    else:
        root.mkdir(parents=True, exist_ok=False)
    return root
    ####


def _run_batch_witness(
    witness_id: str,
    composition: CompiledVehicleComposition,
    *,
    binding: VehicleExecutionBinding,
    output_root: Path | None,
) -> dict[str, object]:
    """Run one witness with temporary or explicitly retained artifacts."""

    if output_root is None:
        return _execute_batch_witness(witness_id, composition, binding=binding)
    return _execute_batch_witness(witness_id, composition, binding=binding, output_root=output_root)
    ####


def _finalize_retained_results_directory(root: Path | None) -> dict[str, object] | None:
    """Index and seal all retained batch packets as one release-ready corpus."""

    if root is None:
        return None
    from .composition_result_catalog import (
        index_composition_results,
        validate_composition_release_catalog,
        write_composition_release_catalog,
    )

    indexed = index_composition_results(root)
    if indexed.get("status") != "pass":
        return {
            "status": "fail",
            "directory": str(root),
            "detail": f"normalized result index returned {indexed.get('status')!r}: {indexed.get('errors')!r}",
        }
    result_count = indexed.get("valid_result_count")
    if not isinstance(result_count, int) or result_count <= 0:
        return {
            "status": "fail",
            "directory": str(root),
            "detail": "retained result directory contains no valid public batch packet",
        }
    release_catalog_path = root / "release-catalog.json"
    try:
        release_catalog = write_composition_release_catalog(root, release_catalog_path)
        release_errors = validate_composition_release_catalog(root, release_catalog)
    except (OSError, ValueError) as error:
        return {
            "status": "fail",
            "directory": str(root),
            "detail": f"could not create aggregate release catalog: {error}",
        }
    if release_errors:
        return {
            "status": "fail",
            "directory": str(root),
            "detail": f"aggregate release catalog is not reproducible: {list(release_errors)!r}",
        }
    return {
        "status": "pass",
        "directory": str(root),
        "valid_result_count": result_count,
        "release_catalog": str(release_catalog_path),
        "release_packet_count": release_catalog.get("result_packet_count"),
        "claim_boundary": (
            "This is a retained, hash-bound inventory of public batch witness packets. It does not create "
            "new control, physical-fidelity, robustness, or qualification evidence."
        ),
    }
    ####


def _execute_batch_witness(
    witness_id: str,
    composition: CompiledVehicleComposition,
    *,
    binding: VehicleExecutionBinding,
    output_root: Path | None = None,
) -> dict[str, object]:
    """Run one exact batch witness through the public CLI without log leakage.

    The source-table fixed-wing interpreter is intentionally exercised only
    through its first eight committed rows. Full transport racetracks are
    long-running mission executions, not suitable as a generic endpoint
    smoke. Other current batch factories retain their small bounded nominal
    witnesses and run to their declared terminal state.
    """

    from .runtime.cli import main

    if output_root is None:
        workspace = tempfile.TemporaryDirectory(prefix=f"taoryx-execution-witness-{witness_id}-")
    else:
        identifier = Path(witness_id)
        if identifier.name != witness_id or witness_id in {".", ".."}:
            raise ValueError(f"retained batch witness ID is not a safe path segment: {witness_id!r}")
        destination = output_root / witness_id
        if destination.exists():
            raise ValueError(f"retained batch witness directory already exists: {destination}")
        destination.mkdir()
        workspace = nullcontext(destination)
    with workspace as temporary:
        root = Path(temporary)
        composition_path = root / "composition.json"
        output_dir = root / "execution"
        composition.write_json(composition_path)
        stdout = StringIO()
        arguments = ["vehicle", "run", str(composition_path), "--output-dir", str(output_dir)]
        bounded_translation_smoke = binding.factory_id == "language_backed_powered_fixed_wing.v1"
        if bounded_translation_smoke:
            arguments.extend(("--max-steps", "8"))
        with redirect_stdout(stdout):
            exit_code = main(arguments)
        execution_path = output_dir / "execution.json"
        interface_path = output_dir / "vehicle_interface.json"
        status_trace_path = output_dir / "status_trace.json"
        resource_ledger_path = output_dir / "resource_ledger.json"
        if not execution_path.is_file() or not interface_path.is_file() or not status_trace_path.is_file() or not resource_ledger_path.is_file():
            return {
                "status": "fail",
                "detail": "vehicle run omitted execution, interface, committed status-trace, or resource-ledger artifact",
            }
        interface_trace_evidence = _validate_batch_interface_trace_artifact(composition, output_dir)
        if interface_trace_evidence["status"] != "pass":
            return {"status": "fail", "detail": str(interface_trace_evidence["detail"])}
        resource_ledger_evidence = _validate_batch_resource_ledger_artifact(composition, output_dir)
        if resource_ledger_evidence["status"] != "pass":
            return {"status": "fail", "detail": str(resource_ledger_evidence["detail"])}
        action_trace_evidence = _validate_batch_action_trace_artifact(
            composition,
            output_dir,
            binding.batch_action_trace,
        )
        if action_trace_evidence["status"] != "pass":
            return {"status": "fail", "detail": str(action_trace_evidence["detail"])}
        result_catalog_evidence = _validate_batch_result_catalog(output_dir, binding)
        if result_catalog_evidence["status"] != "pass":
            return {"status": "fail", "detail": str(result_catalog_evidence["detail"])}
        graph_execution_evidence = _graph_execution_summary(output_dir)
        if graph_execution_evidence["status"] != "pass":
            return {"status": "fail", "detail": str(graph_execution_evidence["detail"])}
        reproduction_evidence = _validate_batch_reproduction_artifact(
            composition,
            output_dir,
            binding=binding,
        )
        if reproduction_evidence["status"] != "pass":
            return {"status": "fail", "detail": str(reproduction_evidence["detail"])}
        release_packet_evidence = _validate_batch_release_packet(output_dir)
        if release_packet_evidence["status"] != "pass":
            return {"status": "fail", "detail": str(release_packet_evidence["detail"])}
        if exit_code != 0 and not bounded_translation_smoke:
            return {"status": "fail", "detail": f"vehicle run returned {exit_code}"}
        if exit_code not in {0, 1}:
            return {"status": "fail", "detail": f"vehicle run returned unexpected code {exit_code}"}
        payload = json.loads(execution_path.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            return {"status": "fail", "detail": "execution artifact is not a JSON object"}
        if bounded_translation_smoke:
            return {
                "status": "pass",
                "detail": "bounded source-table translation smoke emitted execution/interface/status artifacts",
                "mode": "bounded_translation_smoke",
                "max_steps": 8,
                "mission_pass": bool(payload.get("mission_pass")),
                "interface_trace": interface_trace_evidence,
                "action_trace": action_trace_evidence,
                "resource_ledger": resource_ledger_evidence,
                "result_catalog": result_catalog_evidence,
                "graph_execution": graph_execution_evidence,
                "reproduction": reproduction_evidence,
                "release_packet": release_packet_evidence,
            }
        control_screen = payload.get("control_screen")
        declared_screen_pass = payload.get("screen_pass") is True or (
            isinstance(control_screen, Mapping)
            and (control_screen.get("screen_pass") is True or control_screen.get("mission_pass") is True)
        )
        if declared_screen_pass:
            if not isinstance(control_screen, Mapping) or (
                control_screen.get("screen_pass") is not True and control_screen.get("mission_pass") is not True
            ):
                return {"status": "fail", "detail": "batch screen did not pass its declared screen contract"}
            is_source_surface_authority = binding.execution_mode == "source_surface_authority_screen"
            controller_metadata: dict[str, object] | None = None
            if not is_source_surface_authority:
                status_trace = json.loads(status_trace_path.read_text(encoding="utf-8"))
                if not isinstance(status_trace, Mapping):
                    return {"status": "fail", "detail": "committed status trace is not a JSON object"}
                controller_metadata = _validate_batch_controller_metadata(payload, status_trace=status_trace)
                if controller_metadata["status"] != "pass":
                    return {"status": "fail", "detail": str(controller_metadata["detail"])}
            return {
                "status": "pass",
                "detail": (
                    "source-surface authority screen completed and emitted execution/interface/status artifacts"
                    if is_source_surface_authority
                    else "local controller screen completed and emitted execution/interface/status artifacts"
                ),
                "mode": "source_surface_authority_screen" if is_source_surface_authority else "local_controller_screen",
                "screen_pass": True,
                **({"controller_metadata": controller_metadata} if controller_metadata is not None else {}),
                "interface_trace": interface_trace_evidence,
                "action_trace": action_trace_evidence,
                "resource_ledger": resource_ledger_evidence,
                "result_catalog": result_catalog_evidence,
                "graph_execution": graph_execution_evidence,
                "reproduction": reproduction_evidence,
                "release_packet": release_packet_evidence,
            }
        if not bool(payload.get("mission_pass")):
            return {"status": "fail", "detail": "nominal batch execution did not pass its mission contract"}
        return {
            "status": "pass",
            "detail": "public vehicle run completed and emitted execution/interface/status artifacts",
            "mode": "complete_nominal_mission",
            "mission_pass": True,
            "interface_trace": interface_trace_evidence,
            "action_trace": action_trace_evidence,
            "resource_ledger": resource_ledger_evidence,
            "result_catalog": result_catalog_evidence,
            "graph_execution": graph_execution_evidence,
            "reproduction": reproduction_evidence,
            "release_packet": release_packet_evidence,
        }
    ####


def _validate_batch_controller_metadata(
    payload: Mapping[str, object],
    *,
    status_trace: Mapping[str, object] | None = None,
) -> dict[str, object]:
    """Require a completed controller screen to identify its actual method.

    An aggregate family/fidelity interface can legitimately include both
    controller-driven and non-controller operations.  This mission-specific
    gate therefore verifies the executed screen record itself: its runtime
    metadata and control-screen evaluation must both name the same supported
    controller method.  It is evidence of an executed implementation choice,
    not a qualification or robustness claim.
    """

    runtime = payload.get("runtime")
    if not isinstance(runtime, Mapping):
        return {"status": "fail", "detail": "controller screen omitted runtime controller metadata"}
    control_screen = payload.get("control_screen")
    if not isinstance(control_screen, Mapping):
        return {"status": "fail", "detail": "controller screen omitted control-screen metadata"}
    runtime_method = runtime.get("controller_method")
    screen_method = control_screen.get("controller_method")
    if runtime_method not in {"lqr", "lqi"}:
        return {"status": "fail", "detail": "runtime controller metadata must declare lqr or lqi"}
    if screen_method != runtime_method:
        return {
            "status": "fail",
            "detail": "runtime and control-screen controller methods disagree",
        }
    control_realization = runtime.get("control_realization")
    if not isinstance(control_realization, str) or not control_realization:
        return {"status": "fail", "detail": "runtime controller metadata omitted control realization"}
    evidence: dict[str, object] = {
        "status": "pass",
        "method": runtime_method,
        "control_realization": control_realization,
        "integral_output_names": list(runtime.get("integral_output_names", ())),
    }
    if status_trace is None:
        return evidence
    samples = status_trace.get("samples")
    if not isinstance(samples, list) or not samples:
        return {"status": "fail", "detail": "controller screen status trace has no committed samples"}
    for index, sample in enumerate(samples):
        if not isinstance(sample, Mapping) or not isinstance(sample.get("values"), Mapping):
            return {"status": "fail", "detail": f"controller screen status trace sample {index} is malformed"}
        if sample["values"].get("control.controller.method") != runtime_method:
            return {
                "status": "fail",
                "detail": "runtime controller method disagrees with committed status trace",
            }
    evidence["status_trace_method"] = runtime_method
    evidence["status_trace_sample_count"] = len(samples)
    return evidence
    ####


def _validate_batch_interface_trace_artifact(
    composition: CompiledVehicleComposition,
    output_dir: Path,
) -> dict[str, object]:
    """Validate and summarize exercised batch-visible interface categories.

    The committed status trace is the common runtime proof for all advertised
    status, resource, and diagnostic channels.  This helper makes that exact
    coverage visible in a witness report without turning an availability
    declaration into controller or qualification evidence.
    """

    path = output_dir / "status_trace.json"
    try:
        from .composition_status_trace import validate_committed_status_trace
        from .vehicle_composition import resolve_vehicle_composition_interface_contract

        payload = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError("status trace artifact is not a JSON object")
        validate_committed_status_trace(composition, payload)
        contract = resolve_vehicle_composition_interface_contract(composition)
        channel_groups = {
            "status": contract.status_channels,
            "resources": contract.resource_channels,
            "diagnostics": contract.diagnostic_channels,
        }
        coverage = {
            name: [
                channel.id
                for channel in channels
                if channel.availability in {"available", "available_in_batch"}
            ]
            for name, channels in channel_groups.items()
        }
        channels = payload.get("channels")
        samples = payload.get("samples")
        if not isinstance(channels, list) or not isinstance(samples, list):
            raise ValueError("validated status trace has malformed summary fields")
    except (OSError, ValueError, json.JSONDecodeError) as error:
        return {"status": "fail", "detail": f"invalid committed interface trace: {error}"}
    return {
        "status": "pass",
        "artifact": "status_trace.json",
        "interface_id": contract.id,
        "interface_fingerprint_sha256": contract.fingerprint,
        "sample_count": len(samples),
        "channel_count": len(channels),
        "batch_visible_channels": coverage,
        "claim_boundary": (
            "This proves complete batch projection of the selected interface's available status, resource, and "
            "diagnostic channels. Control evidence remains separately bounded by the declared action trace."
        ),
    }
    ####


def _graph_execution_summary(output_dir: Path) -> dict[str, object]:
    """Project already-validated graph evidence for a public witness report."""

    path = output_dir / "mission_graph_execution.json"
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError("mission graph execution artifact is not a JSON object")
        observation_status = payload.get("observation_status")
        graph_status = payload.get("graph_status")
        dispatches = payload.get("dispatches")
        if observation_status not in {"observed", "unobserved"}:
            raise ValueError("mission graph execution artifact has invalid observation status")
        if not isinstance(graph_status, str) or not graph_status:
            raise ValueError("mission graph execution artifact has invalid graph status")
        if not isinstance(dispatches, list):
            raise ValueError("mission graph execution artifact has invalid dispatches")
        outcomes: list[str] = []
        for index, dispatch in enumerate(dispatches):
            if not isinstance(dispatch, Mapping) or not isinstance(dispatch.get("outcome"), str):
                raise ValueError(f"mission graph execution dispatch {index} has invalid outcome")
            outcomes.append(str(dispatch["outcome"]))
    except (OSError, ValueError, json.JSONDecodeError) as error:
        return {"status": "fail", "detail": f"invalid mission graph execution artifact: {error}"}
    return {
        "status": "pass",
        "artifact": "mission_graph_execution.json",
        "observation_status": observation_status,
        "graph_status": graph_status,
        "dispatch_count": len(dispatches),
        "outcomes": outcomes,
    }
    ####


def _validate_batch_reproduction_artifact(
    composition: CompiledVehicleComposition,
    output_dir: Path,
    *,
    binding: VehicleExecutionBinding,
) -> dict[str, object]:
    """Require a fingerprint-bound command that can recreate a batch packet."""

    path = output_dir / "reproduction.txt"
    if not path.is_file():
        return {"status": "fail", "detail": "vehicle run omitted reproduction.txt"}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        return {"status": "fail", "detail": f"could not read reproduction.txt: {error}"}
    metadata = {
        "composition_id": composition.id,
        "composition_identity_sha256": composition.identity_sha256,
        "execution_factory_id": binding.factory_id,
        "execution_mode": binding.execution_mode,
    }
    for key, expected in metadata.items():
        needle = f"# {key}: {expected}"
        if needle not in lines:
            return {"status": "fail", "detail": f"reproduction.txt is missing {needle!r}"}
    commands = [line for line in lines if line and not line.startswith("#")]
    if len(commands) != 1:
        return {"status": "fail", "detail": "reproduction.txt must contain exactly one command"}
    try:
        command = shlex.split(commands[0])
    except ValueError as error:
        return {"status": "fail", "detail": f"reproduction.txt command is invalid: {error}"}
    if command[:3] != ["taoryx", "vehicle", "run"] or "--output-dir" not in command:
        return {
            "status": "fail",
            "detail": "reproduction.txt does not contain a public 'taoryx vehicle run ... --output-dir ...' command",
        }
    return {"status": "pass", "artifact": "reproduction.txt", "command": commands[0]}
    ####


def _validate_batch_release_packet(output_dir: Path) -> dict[str, object]:
    """Require every generated batch packet to be release-catalog ready.

    A one-packet release catalog is a packaging-contract check only. It proves
    that raw public-run artifacts have coherent identity and release sidecars;
    it neither evaluates numerical robustness nor promotes mission evidence.
    """

    from .composition_result_catalog import build_composition_release_catalog, validate_composition_release_catalog

    try:
        catalog = build_composition_release_catalog(output_dir.parent)
        errors = validate_composition_release_catalog(output_dir.parent, catalog)
        if errors:
            raise ValueError("; ".join(errors))
        if catalog.get("result_packet_count") != 1:
            raise ValueError("generated witness root must contain exactly one release packet")
        packets = catalog.get("packets")
        if not isinstance(packets, list) or len(packets) != 1 or not isinstance(packets[0], Mapping):
            raise ValueError("release catalog has no single packet record")
        identity = packets[0].get("execution_identity")
        if not isinstance(identity, Mapping):
            raise ValueError("release packet has no execution identity")
        reproduction = identity.get("reproduction")
        if not isinstance(reproduction, Mapping) or reproduction.get("status") != "verified":
            raise ValueError("release packet has no verified reproduction identity")
    except ValueError as error:
        return {"status": "fail", "detail": f"batch packet is not release-catalog ready: {error}"}
    return {
        "status": "pass",
        "schema": catalog["schema"],
        "result_packet_count": 1,
        "reproduction_identity": "verified",
        "claim_boundary": "Release packet conformance only; it is not a qualification result.",
    }
    ####


def _validate_batch_action_trace_artifact(
    composition: CompiledVehicleComposition,
    output_dir: Path,
    disposition: str,
) -> dict[str, object]:
    """Verify that a batch witness matches its declared action-evidence tier."""

    path = output_dir / "semantic_action_trace.json"
    if disposition == "emits_committed_interval_trace":
        if not path.is_file():
            return {
                "status": "fail",
                "detail": "binding declares committed action trace but vehicle run omitted semantic_action_trace.json",
            }
        try:
            from .composition_control_trace import validate_committed_control_trace_against_status

            payload = json.loads(path.read_text(encoding="utf-8"))
            status_path = output_dir / "status_trace.json"
            status_payload = json.loads(status_path.read_text(encoding="utf-8"))
            if not isinstance(payload, Mapping) or not isinstance(status_payload, Mapping):
                return {"status": "fail", "detail": "semantic action trace or status trace artifact is not a JSON object"}
            validate_committed_control_trace_against_status(
                composition,
                payload,
                status_trace=status_payload,
            )
        except (OSError, ValueError, json.JSONDecodeError) as error:
            return {"status": "fail", "detail": f"invalid committed action trace: {error}"}
        return {
            "status": "pass",
            "disposition": disposition,
            "artifact": "semantic_action_trace.json",
        }
    if path.exists():
        return {
            "status": "fail",
            "detail": f"binding declares {disposition!r} but vehicle run emitted semantic_action_trace.json",
        }
    return {
        "status": "pass",
        "disposition": disposition,
        "artifact": None,
    }
    ####


def _validate_batch_resource_ledger_artifact(
    composition: CompiledVehicleComposition,
    output_dir: Path,
) -> dict[str, object]:
    """Require an exact committed resource ledger for every batch endpoint."""

    ledger_path = output_dir / "resource_ledger.json"
    status_path = output_dir / "status_trace.json"
    if not ledger_path.is_file():
        return {"status": "fail", "detail": "vehicle run omitted resource_ledger.json"}
    try:
        from .composition_resource_ledger import validate_committed_resource_ledger

        ledger_payload = json.loads(ledger_path.read_text(encoding="utf-8"))
        status_payload = json.loads(status_path.read_text(encoding="utf-8"))
        if not isinstance(ledger_payload, Mapping) or not isinstance(status_payload, Mapping):
            return {"status": "fail", "detail": "resource ledger or status trace artifact is not a JSON object"}
        validate_committed_resource_ledger(composition, ledger_payload, status_trace=status_payload)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        return {"status": "fail", "detail": f"invalid committed resource ledger: {error}"}
    summaries = ledger_payload.get("summaries")
    return {
        "status": "pass",
        "artifact": "resource_ledger.json",
        "resource_count": len(summaries) if isinstance(summaries, list) else 0,
    }
    ####


def _validate_batch_result_catalog(
    output_dir: Path,
    binding: VehicleExecutionBinding,
) -> dict[str, object]:
    """Require a batch packet to satisfy the public result-catalog contract.

    This validates output packet interoperability, not mission qualification.
    A bounded translation smoke may legitimately produce a partial normalized
    mission evaluation; the catalog still has to recognize its identity,
    interface, graph-evidence, and declared action-evidence boundaries. The
    direct-wrench screen is intentionally indexed as its separate local-screen
    record kind rather than being coerced into a mission evaluation.
    """

    from .composition_result_catalog import index_composition_results

    report = index_composition_results(output_dir.parent)
    if report.get("status") != "pass":
        return {
            "status": "fail",
            "detail": f"batch packet failed normalized result-catalog validation: {report.get('errors')!r}",
        }
    records = report.get("records")
    if not isinstance(records, list) or len(records) != 1 or not isinstance(records[0], Mapping):
        return {"status": "fail", "detail": "batch packet did not produce exactly one normalized catalog record"}
    record = records[0]
    expected_kind = (
        "local_controller_screen"
        if binding.factory_id in {
            "local_direct_wrench_screen.v1",
            "local_native_coordinate_lqi_screen.v1",
        }
        else "mission_evaluation"
    )
    if record.get("record_kind") != expected_kind or record.get("status") != "valid":
        return {
            "status": "fail",
            "detail": (
                "batch packet normalized catalog record has unexpected kind/status: "
                f"{record.get('record_kind')!r}/{record.get('status')!r}; expected {expected_kind!r}/'valid'"
            ),
        }
    graph_execution = record.get("graph_execution_evidence")
    if not isinstance(graph_execution, Mapping) or graph_execution.get("status") != "verified":
        observed_status = None if not isinstance(graph_execution, Mapping) else graph_execution.get("status")
        return {
            "status": "fail",
            "detail": (
                "batch packet omitted or invalidated mission-graph execution evidence: "
                f"observed status {observed_status!r}"
            ),
        }
    controller_execution = record.get("controller_execution_evidence")
    control_execution = record.get("control_execution_evidence")
    action_trace_execution = record.get("semantic_action_trace_evidence")
    control_status = None if not isinstance(control_execution, Mapping) else control_execution.get("status")
    controller_status = None if not isinstance(controller_execution, Mapping) else controller_execution.get("status")
    if binding.execution_mode in _CONTROL_FREE_EXECUTION_MODES:
        if control_status != "not_applicable":
            return {
                "status": "fail",
                "detail": (
                    "controller-free batch packet must declare control execution evidence not_applicable: "
                    f"observed status {control_status!r}"
                ),
            }
        if controller_status != "not_applicable":
            return {
                "status": "fail",
                "detail": (
                    "controller-free batch packet must declare controller execution evidence not_applicable: "
                    f"observed status {controller_status!r}"
                ),
            }
    elif binding.execution_mode in _NATIVE_AUTONOMOUS_EXECUTION_MODES:
        if control_status != "not_applicable" or controller_status != "not_applicable":
            return {
                "status": "fail",
                "detail": (
                    "native autonomous batch packet must not claim common LQR/LQI execution evidence: "
                    f"observed control/controller statuses {control_status!r}/{controller_status!r}"
                ),
            }
        action_trace_status = (
            None if not isinstance(action_trace_execution, Mapping) else action_trace_execution.get("status")
        )
        if action_trace_status != "verified":
            return {
                "status": "fail",
                "detail": (
                    "native autonomous batch packet omitted or invalidated accepted-interval action evidence: "
                    f"observed status {action_trace_status!r}"
                ),
            }
    else:
        if control_status != "verified":
            return {
                "status": "fail",
                "detail": (
                    "batch packet omitted or invalidated control execution evidence: "
                    f"observed status {control_status!r}"
                ),
            }
        expected_controller_status = (
            "not_applicable" if binding.execution_mode == "source_surface_authority_screen" else "verified"
        )
        if controller_status != expected_controller_status:
            return {
                "status": "fail",
                "detail": (
                    "batch packet controller execution evidence disagrees with its declared execution mode: "
                    f"observed status {controller_status!r}; expected {expected_controller_status!r}"
                ),
            }
    return {
        "status": "pass",
        "record_kind": expected_kind,
        "outcome": record.get("outcome"),
        "qualification": record.get("qualification"),
        "graph_observation_status": graph_execution.get("observation_status"),
        "control_execution_evidence": dict(control_execution),
        **({"controller_execution_evidence": dict(controller_execution)} if isinstance(controller_execution, Mapping) else {}),
        **({"semantic_action_trace_evidence": dict(action_trace_execution)} if isinstance(action_trace_execution, Mapping) else {}),
        "claim_boundary": "The batch packet is consumable by the normalized result catalog; this is not qualification.",
    }
    ####


__all__ = [
    "VEHICLE_EXECUTION_WITNESSES",
    "VehicleExecutionWitness",
    "VehicleExecutionWitnessCatalog",
    "VehicleGraphExtensionWitness",
    "VehicleVariantWitness",
    "_validate_batch_result_catalog",
    "_validate_batch_resource_ledger_artifact",
    "load_vehicle_execution_witness_catalog",
    "validate_vehicle_execution_witnesses",
    "validate_vehicle_graph_extension_witnesses",
    "validate_vehicle_variant_witnesses",
]
