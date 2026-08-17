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


def test_agm6_plugin_materializes_source_backed_tuning_without_mutating_installation(tmp_path: Path) -> None:
    plugin = Agm6VehiclePlugin(_write_agm6_case(tmp_path))
    installed = plugin.source_definition()

    prepared = plugin.prepare_definition(
        Agm6PluginOverrides(
            fin_position_limit_deg=11.0,
            fin_rate_limit_deg_s=300.0,
            fin_natural_frequency_rad_s=210.0,
            fin_damping_ratio=0.9,
            seeker_acquisition_range_m=6_500.0,
            seeker_filter_gain_per_s=2.5,
            seeker_filter_natural_frequency_rad_s=16.0,
            seeker_filter_damping_ratio=0.75,
            structural_limit_g=15.0,
            propulsion_throttle=0.7,
        )
    )

    assert prepared.actuator.position_limit_deg == 11.0
    assert prepared.actuator.rate_limit_deg_s == 300.0
    assert prepared.actuator.natural_frequency_rad_s == 210.0
    assert prepared.actuator.damping_ratio == 0.9
    assert prepared.sensor.acquisition_range_m == 6_500.0
    assert prepared.sensor.filter_gain_per_s == 2.5
    assert prepared.sensor.filter_natural_frequency_rad_s == 16.0
    assert prepared.sensor.filter_damping_ratio == 0.75
    assert prepared.control.structural_limit_g == 15.0
    assert prepared.propulsion.throttle == 0.7
    assert plugin.source_definition() is installed
    assert installed.actuator.position_limit_deg != prepared.actuator.position_limit_deg
    assert installed.propulsion.throttle != prepared.propulsion.throttle


####
