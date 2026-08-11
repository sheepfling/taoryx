from __future__ import annotations

from pathlib import Path

from taoryx.families.cadac.sraam6_plugin import Sraam6PluginOverrides, Sraam6VehiclePlugin
from test_sraam6 import _write_sraam6_case


def test_sraam6_plugin_descriptor_and_installation_are_exact(tmp_path: Path) -> None:
    plugin = Sraam6VehiclePlugin(_write_sraam6_case(tmp_path))
    descriptor = plugin.descriptor

    assert plugin.validate_installation() == ()
    assert descriptor.plugin_id == "cadac.sraam6.missile"
    assert descriptor.source_model == "MISSILE6"
    assert descriptor.status == "runnable"
    assert descriptor.operations == ("discover", "validate", "batch")
    assert descriptor.batch_factory_id == "cadac.sraam6.standard_fin.batch"


####


def test_sraam6_plugin_overrides_do_not_mutate_installed_source_definition(tmp_path: Path) -> None:
    plugin = Sraam6VehiclePlugin(_write_sraam6_case(tmp_path))
    installed = plugin.source_definition()

    result = plugin.run_batch(
        Sraam6PluginOverrides(
            missile_speed_mps=275.0,
            target_turn_g=-2.0,
            navigation_gain=4.0,
            end_time_s=0.04,
            sample_step_s=0.01,
        )
    )

    assert result.samples[0].speed_mps > installed.initial_state.speed_mps
    assert plugin.source_definition() is installed
    assert installed.initial_state.speed_mps == 250.0
    assert installed.target.turn_g == 3.0
    assert installed.guidance.navigation_gain == 3.75


####


def test_sraam6_plugin_reports_missing_source_case(tmp_path: Path) -> None:
    plugin = Sraam6VehiclePlugin(tmp_path / "missing.asc")
    blockers = plugin.validate_installation()

    assert len(blockers) == 1
    assert blockers[0].startswith("source case does not exist:")


####
