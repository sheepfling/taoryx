from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.aim5_plugin import Aim5PluginOverrides, Aim5VehiclePlugin
from taoryx.families.cadac.plugin import CADAC_PLUGIN_CATALOG, CadacPluginRegistry
from test_aim5 import AERO, INPUT, PROP


def _write_case(tmp_path: Path) -> Path:
    path = tmp_path / "input.asc"
    path.write_text(INPUT, encoding="utf-8")
    (tmp_path / "aero.asc").write_text(AERO, encoding="utf-8")
    (tmp_path / "prop.asc").write_text(PROP, encoding="utf-8")
    return path


####


def test_runtime_registry_installs_exact_aim5_plugin(tmp_path: Path) -> None:
    plugin = Aim5VehiclePlugin(_write_case(tmp_path))
    registry = CadacPluginRegistry(CADAC_PLUGIN_CATALOG.plugins)

    registry.register_runtime(plugin)

    assert registry.runtime_ids() == ("cadac.aim5.missile",)
    assert registry.runtime("cadac.aim5.missile") is plugin
    with pytest.raises(KeyError, match="discoverable but has no installed runtime"):
        registry.runtime("cadac.falcon6.aircraft")
    ####


####


def test_aim5_plugin_applies_bounded_overrides_without_mutating_source(tmp_path: Path) -> None:
    plugin = Aim5VehiclePlugin(_write_case(tmp_path))
    original = plugin.source_definition()
    result = plugin.run_batch(
        Aim5PluginOverrides(
            navigation_gain=3.5,
            target_heading_deg=-80.0,
            end_time_s=0.2,
            sample_step_s=0.05,
        )
    )

    assert result.requested_end_time_s == pytest.approx(0.2)
    assert result.engagements[0].samples
    assert plugin.source_definition().missiles[0].config.navigation_gain == pytest.approx(original.missiles[0].config.navigation_gain)
    assert plugin.source_definition().targets[0].config.heading_deg == pytest.approx(original.targets[0].config.heading_deg)


####


def test_aim5_plugin_validation_rejects_missing_source_case(tmp_path: Path) -> None:
    plugin = Aim5VehiclePlugin(tmp_path / "missing.asc")

    blockers = plugin.validate_installation()

    assert len(blockers) == 1
    assert "does not exist" in blockers[0]


####
