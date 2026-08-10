"""Reusable callable registry for native vehicle-composition batch factories."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol, cast

from .plugins import PluginCatalog, discover_plugins
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_bindings import VehicleExecutionBinding, resolve_vehicle_execution_binding


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


def _batch_factories(*, plugins: PluginCatalog | None = None) -> dict[str, BatchFactory]:
    """Build the installed model-owned factory registry fail closed."""

    catalog = plugins or discover_plugins()
    factories: dict[str, BatchFactory] = {}
    for contribution in catalog.records("execution_factory"):
        if not callable(contribution.value):
            raise TypeError(
                f"plug-in {contribution.plugin.id!r} supplied a non-callable execution factory "
                f"for {contribution.id!r}"
            )
        factories[contribution.id] = cast(BatchFactory, contribution.value)
    return factories
    ####


def registered_vehicle_batch_factory_ids(*, plugins: PluginCatalog | None = None) -> tuple[str, ...]:
    """Return every concrete native batch factory behind the callable seam."""

    return tuple(sorted(_batch_factories(plugins=plugins)))
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
        factory = _batch_factories()[binding.factory_id]
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
