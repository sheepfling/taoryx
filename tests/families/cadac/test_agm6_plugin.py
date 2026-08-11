from __future__ import annotations

from pathlib import Path

from taoryx.families.cadac.agm6_plugin import Agm6PluginOverrides, Agm6VehiclePlugin
from test_agm6 import _write_agm6_case


def test_agm6_plugin_is_exact_runnable_three_actor_binding(tmp_path: Path) -> None:
    plugin = Agm6VehiclePlugin(_write_agm6_case(tmp_path))
    descriptor = plugin.descriptor

    assert plugin.validate_installation() == ()
    assert descriptor.plugin_id == "cadac.agm6.missile"
    assert descriptor.status == "runnable"
    assert descriptor.batch_factory_id == "cadac.agm6.standard_fin.batch"
    assert descriptor.default_phase_id == "fin_control"
    assert descriptor.operations == ("discover", "validate", "batch")


####


def test_agm6_plugin_applies_missile_target_aircraft_and_runtime_overrides(tmp_path: Path) -> None:
    plugin = Agm6VehiclePlugin(_write_agm6_case(tmp_path))
    result = plugin.run_batch(
        Agm6PluginOverrides(
            missile_speed_mps=320.0,
            target_speed_mps=12.0,
            aircraft_speed_mps=210.0,
            aircraft_option=1,
            aircraft_turn_g=1.5,
            navigation_gain=4.0,
            random_seed=21,
            end_time_s=0.08,
            sample_step_s=0.02,
        )
    )

    assert result.samples[0].speed_mps > 0.0
    assert result.target_samples[0].speed_mps > 0.0
    assert result.aircraft_samples[0].speed_mps > 0.0
    assert result.requested_end_time_s == 0.08
    assert result.track_samples[0].update_sequence == 0


####
