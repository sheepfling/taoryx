from __future__ import annotations

from pathlib import Path

import pytest
from pydantic import ValidationError
from taoryx.families.cadac.ads6_aircraft import Ads6AircraftThreatTrack
from taoryx.families.cadac.ads6_aircraft_plugin import (
    Ads6AircraftPluginOverrides,
    Ads6AircraftVehiclePlugin,
)
from test_ads6_aircraft import _write_ads6_aircraft_case


def test_ads6_aircraft_plugin_binds_exact_catalog_descriptor(tmp_path: Path) -> None:
    plugin = Ads6AircraftVehiclePlugin(_write_ads6_aircraft_case(tmp_path))

    assert plugin.descriptor.plugin_id == "cadac.ads6.aircraft"
    assert plugin.descriptor.source_model == "AIRCRAFT3"
    assert plugin.descriptor.status == "runnable"
    assert plugin.descriptor.batch_factory_id == "cadac.ads6.aircraft.source_compatibility.batch"
    assert plugin.descriptor.default_phase_id == "source_model"
    assert plugin.validate_installation() == ()


####


def test_ads6_aircraft_plugin_executes_point_mass_overrides(tmp_path: Path) -> None:
    plugin = Ads6AircraftVehiclePlugin(_write_ads6_aircraft_case(tmp_path))
    result = plugin.run_batch(
        Ads6AircraftPluginOverrides(
            position_ned_m=(100.0, 20.0, -2_000.0),
            speed_mps=225.0,
            heading_deg=10.0,
            longitudinal_acceleration_g=0.05,
            end_time_s=0.05,
            sample_step_s=0.05,
        )
    )

    assert result.terminated_reason == "end_time"
    assert result.samples[0].fidelity == "point_mass_3dof"
    assert result.samples[0].control_realization == "force_model"
    assert result.samples[0].position_ned_m[0] > 100.0
    assert result.samples[-1].speed_mps > 225.0


####


def test_ads6_aircraft_plugin_revalidates_maneuver_and_threat_overrides(tmp_path: Path) -> None:
    plugin = Ads6AircraftVehiclePlugin(_write_ads6_aircraft_case(tmp_path))

    with pytest.raises(ValueError, match="maneuver_stop_s"):
        plugin.run_batch(
            Ads6AircraftPluginOverrides(
                guidance_option=1,
                maneuver_start_s=1.0,
                maneuver_stop_s=0.5,
            )
        )
    ####
    with pytest.raises(ValueError, match="requires an external threat track"):
        plugin.run_batch(
            Ads6AircraftPluginOverrides(
                guidance_option=2,
                maneuver_start_s=0.0,
                maneuver_stop_s=1.0,
            )
        )
    ####
    result = plugin.run_batch(
        Ads6AircraftPluginOverrides(
            guidance_option=2,
            guidance_gain=1.0,
            maneuver_start_s=0.0,
            maneuver_stop_s=1.0,
            threat_track=Ads6AircraftThreatTrack(
                position_ned_m=(1_000.0, 0.0, -1_000.0),
                velocity_ned_mps=(100.0, 20.0, 0.0),
            ),
            end_time_s=0.05,
        )
    )
    assert any(sample.mode == "escape" for sample in result.samples)


####


def test_ads6_aircraft_plugin_rejects_nonfinite_vectors_and_invalid_modes() -> None:
    with pytest.raises(ValidationError, match="finite"):
        Ads6AircraftPluginOverrides(position_ned_m=(0.0, float("nan"), 0.0))
    ####
    with pytest.raises(ValidationError, match="guidance_option"):
        Ads6AircraftPluginOverrides(guidance_option=3)
    ####


####
