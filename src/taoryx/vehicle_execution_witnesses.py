"""Executable composition witnesses for every advertised runtime binding.

The execution-binding catalog declares what *could* be selected.  This module
adds the complementary onboarding proof: every currently runnable exact tuple
must have one checked-in composition request that compiles, passes translation
preflight, and resolves that exact factory without borrowing another family.
"""

from __future__ import annotations

import json
import tempfile
from collections.abc import Mapping
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .vehicle_composition import CompiledVehicleComposition, compile_vehicle_composition, load_vehicle_composition_request
from .vehicle_execution_bindings import (
    ExecutionOperation,
    VehicleExecutionBinding,
    load_vehicle_execution_binding_catalog,
    resolve_vehicle_execution_binding,
)
from .vehicle_execution_preflight import preflight_vehicle_composition
from .vehicle_registry import ROOT

VEHICLE_EXECUTION_WITNESSES = ROOT / "verification/vehicle_execution_witnesses.yaml"


class VehicleExecutionWitness(BaseModel):
    """One checked-in semantic request for an exact public endpoint."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    composition: str = Field(min_length=1)
    operation: ExecutionOperation


class VehicleExecutionWitnessCatalog(BaseModel):
    """Versioned witness mapping for the public execution-binding matrix."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_id: str = Field(alias="schema", min_length=1)
    version: int = Field(ge=1)
    description: str = Field(min_length=1)
    witnesses: tuple[VehicleExecutionWitness, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_unique_ids(self) -> VehicleExecutionWitnessCatalog:
        ids = tuple(item.id for item in self.witnesses)
        if len(ids) != len(set(ids)):
            raise ValueError("execution witness catalog has duplicate IDs")
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
) -> dict[str, object]:
    """Validate every runnable execution binding has an exact composition witness.

    The validation intentionally stops before batch integration: it is an
    onboarding and endpoint-contract gate, not a substitute for full mission
    execution or qualification. With ``execute_batch=True``, it additionally
    drives the public compose-to-run command for every batch witness. This is
    deliberately opt-in because it is an integration smoke, not a fast static
    registry check. Interactive witnesses are opened once because their
    endpoint contract includes a concrete accepted-truth episode.
    """

    witness_catalog = catalog or load_vehicle_execution_witness_catalog()
    binding_catalog = load_vehicle_execution_binding_catalog()
    runnable = tuple(item for item in binding_catalog.bindings if item.status == "runnable")
    binding_by_key: dict[tuple[str, str, str, ExecutionOperation], VehicleExecutionBinding] = {
        _binding_key(item): item for item in runnable
    }
    witness_keys: dict[tuple[str, str, str, ExecutionOperation], str] = {}
    records: list[dict[str, object]] = []
    errors: list[str] = []

    for witness in witness_catalog.witnesses:
        source = ROOT / witness.composition
        if not source.is_file():
            errors.append(f"{witness.id}: composition request is missing: {witness.composition}")
            continue
        try:
            composition = compile_vehicle_composition(load_vehicle_composition_request(source))
        except (TypeError, ValueError) as error:
            errors.append(f"{witness.id}: composition does not compile: {error}")
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
        episode_opened = False
        batch_execution: dict[str, object] | None = None
        if witness.operation == "episode":
            try:
                from .composition_episode import open_vehicle_composition_episode

                episode = open_vehicle_composition_episode(composition)
                episode.observe_frame()
                episode.close()
                episode_opened = True
            except (TypeError, ValueError) as error:
                errors.append(f"{witness.id}: declared episode did not open: {error}")
        elif execute_batch:
            batch_execution = _execute_batch_witness(
                witness.id,
                composition,
                factory_id=resolved.factory_id,
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
                "preflight_status": preflight.status,
                "episode_opened": episode_opened if witness.operation == "episode" else None,
                "batch_execution": batch_execution,
            }
        )

    for binding_key in sorted(binding_by_key):
        if binding_key not in witness_keys:
            errors.append(f"runnable execution binding has no composition witness: {_format_key(binding_key)}")
    for witness_key in sorted(witness_keys):
        if witness_key not in binding_by_key:
            errors.append(f"composition witness targets an unavailable endpoint: {_format_key(witness_key)}")
    return {
        "schema": "taoryx.vehicle-execution-witness-report/v1alpha1",
        "status": "pass" if not errors else "fail",
        "runnable_binding_count": len(runnable),
        "witness_count": len(witness_catalog.witnesses),
        "batch_execution_smoke": execute_batch,
        "records": records,
        "errors": errors,
        "claim_boundary": (
            "This verifies exact composition-to-endpoint discoverability and interactive endpoint construction. "
            "When batch_execution_smoke is enabled, it also exercises the public batch artifact seam. "
            "Neither mode establishes numerical parity or promotes qualification evidence."
        ),
    }
    ####


def _binding_key(binding: VehicleExecutionBinding) -> tuple[str, str, str, ExecutionOperation]:
    return (binding.family_id, binding.mission, binding.fidelity, binding.operation)
    ####


def _format_key(key: tuple[str, str, str, ExecutionOperation]) -> str:
    family, mission, fidelity, operation = key
    return f"{family}/{mission}/{fidelity}/{operation}"
    ####


def _execute_batch_witness(
    witness_id: str,
    composition: CompiledVehicleComposition,
    *,
    factory_id: str | None,
) -> dict[str, object]:
    """Run one exact batch witness through the public CLI without log leakage.

    The source-table fixed-wing interpreter is intentionally exercised only
    through its first eight committed rows. Full transport racetracks are
    long-running mission executions, not suitable as a generic endpoint
    smoke. Other current batch factories retain their small bounded nominal
    witnesses and run to their declared terminal state.
    """

    from .runtime.cli import main

    with tempfile.TemporaryDirectory(prefix=f"taoryx-execution-witness-{witness_id}-") as temporary:
        root = Path(temporary)
        composition_path = root / "composition.json"
        output_dir = root / "execution"
        composition.write_json(composition_path)
        stdout = StringIO()
        arguments = ["vehicle", "run", str(composition_path), "--output-dir", str(output_dir)]
        bounded_translation_smoke = factory_id == "language_backed_powered_fixed_wing.v1"
        if bounded_translation_smoke:
            arguments.extend(("--max-steps", "8"))
        with redirect_stdout(stdout):
            exit_code = main(arguments)
        execution_path = output_dir / "execution.json"
        interface_path = output_dir / "vehicle_interface.json"
        status_trace_path = output_dir / "status_trace.json"
        if not execution_path.is_file() or not interface_path.is_file() or not status_trace_path.is_file():
            return {
                "status": "fail",
                "detail": "vehicle run omitted execution, interface, or committed status-trace artifact",
            }
        try:
            from .composition_status_trace import validate_committed_status_trace

            trace_payload = json.loads(status_trace_path.read_text(encoding="utf-8"))
            if not isinstance(trace_payload, Mapping):
                return {"status": "fail", "detail": "status trace artifact is not a JSON object"}
            validate_committed_status_trace(composition, trace_payload)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            return {"status": "fail", "detail": f"invalid committed status trace: {error}"}
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
            }
        if not bool(payload.get("mission_pass")):
            return {"status": "fail", "detail": "nominal batch execution did not pass its mission contract"}
        return {
            "status": "pass",
            "detail": "public vehicle run completed and emitted execution/interface/status artifacts",
            "mode": "complete_nominal_mission",
            "mission_pass": True,
        }
    ####


__all__ = [
    "VEHICLE_EXECUTION_WITNESSES",
    "VehicleExecutionWitness",
    "VehicleExecutionWitnessCatalog",
    "load_vehicle_execution_witness_catalog",
    "validate_vehicle_execution_witnesses",
]
