from __future__ import annotations

from pathlib import Path

from taoryx.families.cadac.ghame3_plugin import Ghame3PluginOverrides, Ghame3VehiclePlugin
from test_ghame3 import _case


def test_ghame3_plugin_installs_as_point_mass(tmp_path: Path) -> None:
    plugin = Ghame3VehiclePlugin(_case(tmp_path))
    assert plugin.validate_installation() == ()
    assert plugin.descriptor.status == "runnable"
    assert plugin.descriptor.batch_factory_id == "cadac.ghame3.source_compatibility.batch"
    assert plugin.source_definition().taoryx_tier == "point_mass_3dof"


####


def test_ghame3_plugin_overrides_do_not_mutate_source(tmp_path: Path) -> None:
    plugin = Ghame3VehiclePlugin(_case(tmp_path))
    source = plugin.source_definition()
    run = plugin.run_batch(Ghame3PluginOverrides(altitude_m=5000.0, end_time_s=0.01, sample_step_s=0.5))
    assert source.initial_state.altitude_m == 3000.0
    assert source.end_time_s == 1.0
    assert run.samples[0].altitude_m > 4999.0


####
