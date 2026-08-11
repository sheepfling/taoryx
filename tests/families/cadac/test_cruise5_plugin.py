from __future__ import annotations

from pathlib import Path

from taoryx.families.cadac.cruise5_plugin import Cruise5PluginOverrides, Cruise5VehiclePlugin
from test_cruise5 import _case


def test_cruise5_plugin_installs_source_compatible_runtime(tmp_path: Path) -> None:
    plugin = Cruise5VehiclePlugin(_case(tmp_path))
    assert plugin.validate_installation() == ()
    assert plugin.descriptor.status == "runnable"
    assert plugin.descriptor.batch_factory_id == "cadac.cruise5.source_compatibility.batch"
    assert plugin.source_definition().taoryx_tier == "pseudo_6dof"


####


def test_cruise5_plugin_applies_overrides_without_mutating_source_definition(tmp_path: Path) -> None:
    plugin = Cruise5VehiclePlugin(_case(tmp_path))
    source = plugin.source_definition()
    run = plugin.run_batch(
        Cruise5PluginOverrides(
            altitude_m=1500.0,
            end_time_s=0.05,
            sample_step_s=0.5,
        )
    )
    assert source.initial_state.altitude_m == 1000.0
    assert source.end_time_s == 250.0
    assert run.requested_end_time_s == 0.05
    assert run.samples[0].altitude_m > 1499.0


####
