"""Installation-boundary tests for the CADAC distribution entry point."""

from __future__ import annotations

from dataclasses import dataclass

from taoryx_cadac.plugin import PLUGIN

from taoryx.plugins import discover_plugins


@dataclass(frozen=True)
class _EntryPoint:
    """Minimal deterministic entry-point double for plug-in discovery."""

    name: str
    value: str
    target: object

    def load(self) -> object:
        return self.target
        ####

    ####


def test_cadac_entry_point_advertises_the_source_independent_catalog() -> None:
    """CADAC discovery must not require an upstream source checkout."""

    catalog = discover_plugins(
        include_builtin=False,
        entry_points=(
            _EntryPoint(
                name="taoryx.cadac",
                value="taoryx_cadac.plugin:PLUGIN",
                target=PLUGIN,
            ),
        ),
    )

    assert tuple(item.id for item in catalog.plugins) == ("taoryx.cadac",)
    contribution = catalog.contribution("mission_composition_provider", "cadac")
    assert contribution.plugin.package == "taoryx-cadac"
    provider = catalog.build_mission_composition_provider_registry().provider("cadac")
    assert provider.metadata.status == "development"
    assert len(provider.list_models()) == 17
    ####
