"""Taoryx entry point for the parametric interceptor package."""

from __future__ import annotations

from taoryx.plugins import PluginDefinition, PluginMetadata, PluginRegistrar

from .profile import interceptor
from .provider import PACKAGE_VERSION, PROVIDER_ID, ParametricInterceptorMissionCompositionProvider
from .witnesses import runnable_prototype_profiles


def _mission_composition_provider() -> object:
    """Construct the lightweight built-in example only after selection."""

    example = interceptor(
        "generic-medium-sam",
        parameter_set_version="0.1.0",
        model_notes=(
            "Developer exemplar. Every engineering value is resolved from the versioned generic archetype and remains "
            "marked as an assumption. Applications should construct providers from their own evidence profiles."
        ),
    )
    return ParametricInterceptorMissionCompositionProvider.from_profiles(
        example,
        *runnable_prototype_profiles(),
    )
    ####


def _register(registrar: PluginRegistrar) -> None:
    registrar.register_mission_composition_provider_factory(PROVIDER_ID, _mission_composition_provider)
    ####


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="taoryx.parametric-interceptors",
        package="taoryx-parametric-interceptors",
        version=PACKAGE_VERSION,
        api_version="1",
        description="Evidence-aware parametric interceptor surrogate models.",
    ),
    register_callback=_register,
)

__all__ = ["PLUGIN"]
####
