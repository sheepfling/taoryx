"""Reusable callable registry for native vehicle-composition batch factories."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol, cast

from .controller_runtime_contract import ControllerRuntimeDeclaration
from .plugins import PluginCatalog, discover_plugins
from .tuning_application import TuningApplicationContext, TuningApplicationContextSet
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_artifact import VehicleExecutionArtifact, VehicleExecutionPacket
from .vehicle_execution_bindings import VehicleExecutionBinding, resolve_vehicle_execution_binding

_BATCH_FACTORY_REQUEST_CONTRACT = "taoryx.vehicle-batch-factory-request/v1alpha1"


class NativeBatchExecution(Protocol):
    """Minimum common surface returned by existing source-owned executors."""

    @property
    def output_dir(self) -> Path: ...

    def as_dict(self) -> dict[str, object]: ...


BatchFactory = Callable[[CompiledVehicleComposition, str | Path, int | None], NativeBatchExecution]


@dataclass(frozen=True, slots=True)
class VehicleBatchExecutionRequest:
    """Exact, typed invocation contract supplied to a new batch plug-in.

    This deliberately carries the resolved binding as well as the immutable
    composition, so a plug-in never needs to rediscover its family/fidelity or
    infer the selected factory from a loose argument collection.
    """

    composition: CompiledVehicleComposition
    binding: VehicleExecutionBinding
    output_dir: Path
    max_steps: int | None = None
    tuning_context: TuningApplicationContext | None = None
    tuning_context_set: TuningApplicationContextSet | None = None

    def __post_init__(self) -> None:
        if self.max_steps is not None and self.max_steps <= 0:
            raise ValueError("max_steps must be positive")
        if self.tuning_context is not None and self.tuning_context_set is not None:
            raise ValueError("batch execution request accepts either one tuning context or one context set, not both")
        identity = (self.composition.family_id, self.composition.mission, self.composition.fidelity)
        binding_identity = (self.binding.family_id, self.binding.mission, self.binding.fidelity)
        if identity != binding_identity:
            raise ValueError("batch execution request binding does not match the selected composition")
        if self.binding.operation != "batch" or self.binding.status != "runnable" or self.binding.factory_id is None:
            raise ValueError("batch execution request requires one runnable batch binding with a factory ID")
        object.__setattr__(self, "output_dir", Path(self.output_dir))
        ####

    def as_dict(self) -> dict[str, object]:
        """Return portable request identity for a plug-in-owned artifact."""

        return {
            "schema": _BATCH_FACTORY_REQUEST_CONTRACT,
            "composition_id": self.composition.id,
            "composition_identity_sha256": self.composition.identity_sha256,
            "family_id": self.composition.family_id,
            "vehicle_id": self.composition.vehicle_id,
            "mission_id": self.composition.mission,
            "fidelity": self.composition.fidelity,
            "factory_id": self.binding.factory_id,
            "execution_mode": self.binding.execution_mode,
            "output_dir": str(self.output_dir),
            "max_steps": self.max_steps,
            "tuning_application_context": (
                None
                if self.tuning_context is None
                else {
                    "campaign_id": self.tuning_context.campaign_id,
                    "node_id": self.tuning_context.node_id,
                    "candidate_profile_id": self.tuning_context.candidate_profile_id,
                    "candidate_configuration_fingerprint_sha256": (
                        self.tuning_context.candidate_configuration_fingerprint_sha256
                    ),
                    "resolved_gain_fingerprint_sha256": self.tuning_context.resolved_gain_fingerprint_sha256,
                }
            ),
            "tuning_application_context_set": (
                None
                if self.tuning_context_set is None
                else {
                    "campaign_id": self.tuning_context_set.campaign_id,
                    "controller_method": self.tuning_context_set.controller_method,
                    "node_ids": list(self.tuning_context_set.node_ids),
                    "contexts": [
                        {
                            "campaign_id": context.campaign_id,
                            "node_id": context.node_id,
                            "candidate_profile_id": context.candidate_profile_id,
                            "candidate_configuration_fingerprint_sha256": (
                                context.candidate_configuration_fingerprint_sha256
                            ),
                            "resolved_gain_fingerprint_sha256": context.resolved_gain_fingerprint_sha256,
                        }
                        for context in self.tuning_context_set.contexts
                    ],
                }
            ),
        }
        ####

    ####


StructuredBatchFactory = Callable[[VehicleBatchExecutionRequest], NativeBatchExecution]
RegisteredBatchFactory = BatchFactory | StructuredBatchFactory


def batch_factory_request_v1(factory: StructuredBatchFactory) -> StructuredBatchFactory:
    """Mark a plug-in factory as accepting :class:`VehicleBatchExecutionRequest`.

    Legacy three-argument factories remain supported while installed plug-ins
    migrate. New vehicle families should use this marker and receive the
    complete typed request rather than positional argument bags.
    """

    if not callable(factory):
        raise TypeError("typed batch execution factories must be callable")
    setattr(factory, "__taoryx_batch_factory_contract__", _BATCH_FACTORY_REQUEST_CONTRACT)
    return factory
    ####


@dataclass(frozen=True, slots=True)
class VehicleBatchExecution:
    """One exact native binding invocation through the reusable registry."""

    composition: CompiledVehicleComposition
    binding: VehicleExecutionBinding
    request: VehicleBatchExecutionRequest
    execution: NativeBatchExecution
    passed: bool
    artifact: VehicleExecutionArtifact
    packet: VehicleExecutionPacket

    @property
    def output_dir(self) -> Path:
        return self.execution.output_dir
        ####

    def as_dict(self) -> dict[str, object]:
        """Return the canonical host packet rather than a raw factory map."""

        return self.packet.as_dict()
        ####

    ####


def _batch_factories(*, plugins: PluginCatalog | None = None) -> dict[str, RegisteredBatchFactory]:
    """Build the installed model-owned factory registry fail closed."""

    catalog = plugins or discover_plugins()
    factories: dict[str, RegisteredBatchFactory] = {}
    for contribution in catalog.records("execution_factory"):
        if not callable(contribution.value):
            raise TypeError(
                f"plug-in {contribution.plugin.id!r} supplied a non-callable execution factory "
                f"for {contribution.id!r}"
            )
        factories[contribution.id] = cast(RegisteredBatchFactory, contribution.value)
    return factories
    ####


def registered_vehicle_batch_factory_ids(*, plugins: PluginCatalog | None = None) -> tuple[str, ...]:
    """Return every concrete native batch factory behind the callable seam."""

    return tuple(sorted(_batch_factories(plugins=plugins)))
    ####


def _invoke_batch_factory(
    factory: RegisteredBatchFactory,
    request: VehicleBatchExecutionRequest,
) -> NativeBatchExecution:
    """Call a new typed factory or the temporary legacy adapter explicitly."""

    if getattr(factory, "__taoryx_batch_factory_contract__", None) == _BATCH_FACTORY_REQUEST_CONTRACT:
        return cast(StructuredBatchFactory, factory)(request)
    return cast(BatchFactory, factory)(request.composition, request.output_dir, request.max_steps)
    ####


def execute_vehicle_composition_batch(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    *,
    max_steps: int | None = None,
    tuning_context: TuningApplicationContext | None = None,
    tuning_context_set: TuningApplicationContextSet | None = None,
) -> VehicleBatchExecution:
    """Resolve and invoke one exact batch binding without family fallback."""

    binding = resolve_vehicle_execution_binding(composition, "batch")
    if binding.factory_id is None:
        raise ValueError("runnable batch binding lacks a factory identifier")
    try:
        factory = _batch_factories()[binding.factory_id]
    except KeyError as error:
        raise ValueError(f"batch execution factory is declared but not implemented: {binding.factory_id!r}") from error
    request = VehicleBatchExecutionRequest(
        composition=composition,
        binding=binding,
        output_dir=Path(output_dir),
        max_steps=max_steps,
        tuning_context=tuning_context,
        tuning_context_set=tuning_context_set,
    )
    execution = _invoke_batch_factory(factory, request)
    payload = execution.as_dict()
    if not isinstance(payload, Mapping):
        raise TypeError(f"batch factory {binding.factory_id!r} returned a non-mapping artifact")
    try:
        artifact = VehicleExecutionArtifact.from_payload(payload, composition=composition)
    except (TypeError, ValueError) as error:
        raise TypeError(
            f"batch factory {binding.factory_id!r} returned an invalid execution artifact: {error}"
        ) from error
    runtime = payload.get("runtime")
    if isinstance(runtime, Mapping) and runtime.get("controller_method") is not None:
        try:
            ControllerRuntimeDeclaration.model_validate(runtime)
        except (TypeError, ValueError) as error:
            raise TypeError(
                f"batch factory {binding.factory_id!r} returned an invalid controller runtime declaration: {error}"
            ) from error
    if execution.output_dir.resolve() != request.output_dir.resolve():
        raise TypeError(f"batch factory {binding.factory_id!r} returned an execution for a different output directory")
    if artifact.output_dir is not None and Path(artifact.output_dir).resolve() != request.output_dir.resolve():
        raise TypeError(f"batch factory {binding.factory_id!r} artifact output_dir disagrees with its request")
    from .vehicle_composition import resolve_vehicle_composition_interface_contract

    interface = resolve_vehicle_composition_interface_contract(composition)
    packet = VehicleExecutionPacket.from_artifact(
        artifact,
        request=request.as_dict(),
        interface_id=interface.id,
        interface_fingerprint_sha256=interface.fingerprint,
    )
    packet.write_json(request.output_dir / "execution.json")
    return VehicleBatchExecution(
        composition=composition,
        binding=binding,
        request=request,
        execution=execution,
        passed=artifact.passed,
        artifact=artifact,
        packet=packet,
    )
    ####


__all__ = [
    "NativeBatchExecution",
    "RegisteredBatchFactory",
    "StructuredBatchFactory",
    "VehicleExecutionArtifact",
    "VehicleExecutionPacket",
    "VehicleBatchExecution",
    "VehicleBatchExecutionRequest",
    "batch_factory_request_v1",
    "execute_vehicle_composition_batch",
    "registered_vehicle_batch_factory_ids",
]
