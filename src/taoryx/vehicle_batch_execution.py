"""Reusable callable registry for native vehicle-composition batch factories."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

from .hl20_source_release_composition_execution import execute_hl20_source_booster_release_composition
from .hummingbird_composition_execution import execute_hummingbird_pseudo_composition
from .language_backed_execution import execute_powered_fixed_wing_composition
from .local_direct_wrench_composition_execution import execute_local_direct_wrench_composition
from .nesc_composition_execution import execute_nesc_source_replay_composition
from .passive_tumbling_composition_execution import execute_passive_tumbling_composition
from .reduced_fixed_wing_execution import execute_reduced_fixed_wing_composition
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_bindings import VehicleExecutionBinding, resolve_vehicle_execution_binding
from .x15_staged_composition_execution import execute_x15_staged_reachability_composition


class NativeBatchExecution(Protocol):
    """Minimum common surface returned by existing source-owned executors."""

    @property
    def output_dir(self) -> Path: ...

    def as_dict(self) -> dict[str, object]: ...


BatchFactory = Callable[[CompiledVehicleComposition, str | Path, int | None], NativeBatchExecution]


@dataclass(frozen=True, slots=True)
class VehicleBatchExecution:
    """One exact native binding invocation through the reusable registry."""

    composition: CompiledVehicleComposition
    binding: VehicleExecutionBinding
    execution: NativeBatchExecution
    passed: bool

    @property
    def output_dir(self) -> Path:
        return self.execution.output_dir
        ####

    def as_dict(self) -> dict[str, object]:
        return self.execution.as_dict()
        ####

    ####


def _language_backed(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    max_steps: int | None,
) -> NativeBatchExecution:
    return execute_powered_fixed_wing_composition(composition, output_dir, max_steps=max_steps)
    ####


def _without_max_steps(factory: Callable[[CompiledVehicleComposition, str | Path], NativeBatchExecution]) -> BatchFactory:
    def invoke(
        composition: CompiledVehicleComposition,
        output_dir: str | Path,
        max_steps: int | None,
    ) -> NativeBatchExecution:
        if max_steps is not None:
            raise ValueError(f"--max-steps is not available for batch factory {factory.__name__!r}")
        return factory(composition, output_dir)
        ####

    return invoke
    ####


_BATCH_FACTORIES: dict[str, BatchFactory] = {
    "language_backed_powered_fixed_wing.v1": _language_backed,
    "reduced_fixed_wing_openap.v1": _without_max_steps(execute_reduced_fixed_wing_composition),
    "reduced_fixed_wing_f16_source.v1": _without_max_steps(execute_reduced_fixed_wing_composition),
    "hummingbird_aggregate_thrust_pseudo_batch.v1": _without_max_steps(execute_hummingbird_pseudo_composition),
    "nesc_source_replay.v1": _without_max_steps(execute_nesc_source_replay_composition),
    "hl20_source_booster_release_replay.v1": _without_max_steps(execute_hl20_source_booster_release_composition),
    "x15_staged_reachability.v1": _without_max_steps(execute_x15_staged_reachability_composition),
    "local_direct_wrench_screen.v1": _without_max_steps(execute_local_direct_wrench_composition),
    "passive_tumbling_direct_release.v1": _without_max_steps(execute_passive_tumbling_composition),
}


def registered_vehicle_batch_factory_ids() -> tuple[str, ...]:
    """Return every concrete native batch factory behind the callable seam."""

    return tuple(sorted(_BATCH_FACTORIES))
    ####


def execute_vehicle_composition_batch(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    *,
    max_steps: int | None = None,
) -> VehicleBatchExecution:
    """Resolve and invoke one exact batch binding without family fallback."""

    if max_steps is not None and max_steps <= 0:
        raise ValueError("max_steps must be positive")
    binding = resolve_vehicle_execution_binding(composition, "batch")
    if binding.factory_id is None:
        raise ValueError("runnable batch binding lacks a factory identifier")
    try:
        factory = _BATCH_FACTORIES[binding.factory_id]
    except KeyError as error:
        raise ValueError(f"batch execution factory is declared but not implemented: {binding.factory_id!r}") from error
    execution = factory(composition, output_dir, max_steps)
    passed_value: Any = getattr(execution, "mission_pass", None)
    if passed_value is None:
        passed_value = getattr(execution, "screen_pass", None)
    if not isinstance(passed_value, bool):
        raise TypeError(f"batch factory {binding.factory_id!r} returned no boolean pass disposition")
    return VehicleBatchExecution(
        composition=composition,
        binding=binding,
        execution=execution,
        passed=passed_value,
    )
    ####


__all__ = [
    "NativeBatchExecution",
    "VehicleBatchExecution",
    "execute_vehicle_composition_batch",
    "registered_vehicle_batch_factory_ids",
]
