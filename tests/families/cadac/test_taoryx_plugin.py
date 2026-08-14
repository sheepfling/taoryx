"""Installation-boundary tests for the CADAC distribution entry point."""

from __future__ import annotations

import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

from taoryx_cadac.plugin import PLUGIN

from taoryx.plugins import DeferredMissionCompositionProvider, discover_plugins


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
    assert isinstance(contribution.value, DeferredMissionCompositionProvider)
    provider = catalog.build_mission_composition_provider_registry().provider("cadac")
    assert isinstance(provider, DeferredMissionCompositionProvider)
    assert provider.__taoryx_provider_id__ == "cadac"
    ####


def test_cadac_discovery_defers_pydantic_catalog_materialization_until_provider_use() -> None:
    """Listing CADAC must not import every actor schema before a host selects it."""

    root = Path(__file__).resolve().parents[3]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        (
            str(root / "src"),
            str(root / "packages" / "taoryx-cadac" / "src"),
        )
    )
    script = """
import sys
from dataclasses import dataclass

from taoryx.plugins import discover_plugins
from taoryx_cadac.plugin import PLUGIN


@dataclass(frozen=True)
class EntryPoint:
    name: str
    value: str
    target: object

    def load(self) -> object:
        return self.target


assert "taoryx.families.cadac.mission_composition_plugin" not in sys.modules
catalog = discover_plugins(
    include_builtin=False,
    entry_points=(EntryPoint("taoryx.cadac", "taoryx_cadac.plugin:PLUGIN", PLUGIN),),
)
assert tuple(item.id for item in catalog.plugins) == ("taoryx.cadac",)
assert "taoryx.families.cadac.mission_composition_plugin" not in sys.modules
provider = catalog.build_mission_composition_provider_registry().provider("cadac")
assert provider.__taoryx_provider_id__ == "cadac"
assert "taoryx.families.cadac.mission_composition_plugin" not in sys.modules
"""
    completed = subprocess.run(
        [sys.executable, "-c", script],
        cwd=root,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    ####
