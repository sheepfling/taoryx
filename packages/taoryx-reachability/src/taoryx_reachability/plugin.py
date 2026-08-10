"""Plug-in registration for the optional Taoryx reachability workbench."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from taoryx.plugins import PluginDefinition, PluginMetadata, PluginRegistrar
from taoryx.vehicle_composition import CompiledVehicleComposition

from .capabilities import reachability_mission_capability_adapters
from .preflight import reachability_semantic_handlers
from .provider import TaoryxReachabilityProvider


def _execute_hl20_source_release(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    max_steps: int | None,
) -> Any:
    if max_steps is not None:
        raise ValueError("--max-steps is not available for the HL-20 source-release reachability factory")
    from taoryx.hl20_source_release_composition_execution import execute_hl20_source_booster_release_composition

    return execute_hl20_source_booster_release_composition(composition, output_dir)
    ####


def _execute_x15_staged(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    max_steps: int | None,
) -> Any:
    if max_steps is not None:
        raise ValueError("--max-steps is not available for the X-15 staged reachability factory")
    from taoryx.x15_staged_composition_execution import execute_x15_staged_reachability_composition

    return execute_x15_staged_reachability_composition(composition, output_dir)
    ####


def _execute_passive_tumbling(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    max_steps: int | None,
) -> Any:
    if max_steps is not None:
        raise ValueError("--max-steps is not available for the passive-tumbling reachability factory")
    from taoryx.passive_tumbling_composition_execution import execute_passive_tumbling_composition

    return execute_passive_tumbling_composition(composition, output_dir)
    ####


def _register(registrar: PluginRegistrar) -> None:
    """Publish the workbench and its composed mission overlays."""

    registrar.register_reachability_provider(TaoryxReachabilityProvider())
    for adapter in reachability_mission_capability_adapters():
        registrar.register_mission_capability_adapter(adapter)
    for handler in reachability_semantic_handlers():
        registrar.register_semantic_preflight_handler(handler)
    registrar.register_execution_factory(
        "hl20_source_booster_release_replay.v1",
        _execute_hl20_source_release,
    )
    registrar.register_execution_factory(
        "x15_staged_reachability.v1",
        _execute_x15_staged,
    )
    registrar.register_execution_factory(
        "passive_tumbling_direct_release.v1",
        _execute_passive_tumbling,
    )
    ####


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="taoryx.reachability",
        package="taoryx-reachability",
        version="0.1.0a0",
        api_version="1",
        description="Optional reachability envelopes, overlays, plotting, and composed mission workflows.",
    ),
    register_callback=_register,
)

__all__ = ["PLUGIN"]
####
