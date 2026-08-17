from __future__ import annotations

from pathlib import Path

import pytest
from pydantic import ValidationError
from taoryx.families.cadac.ads6_srbm_plugin import Ads6SrbmPluginOverrides, Ads6SrbmVehiclePlugin
from test_ads6_srbm import _write_ads6_srbm_case


def test_ads6_srbm_plugin_binds_exact_rocket5_catalog_descriptor(tmp_path: Path) -> None:
    plugin = Ads6SrbmVehiclePlugin(_write_ads6_srbm_case(tmp_path))

    assert plugin.descriptor.plugin_id == "cadac.ads6.srbm"
    assert plugin.descriptor.source_model == "ROCKET5"
    assert plugin.descriptor.status == "runnable"
    assert plugin.descriptor.batch_factory_id == "cadac.ads6.srbm.source_compatibility.batch"
    assert plugin.descriptor.default_phase_id == "source_model"
    assert plugin.validate_installation() == ()


####


def test_ads6_srbm_plugin_executes_response_law_overrides(tmp_path: Path) -> None:
    plugin = Ads6SrbmVehiclePlugin(_write_ads6_srbm_case(tmp_path))
    result = plugin.run_batch(
        Ads6SrbmPluginOverrides(
            position_ned_m=(100.0, 20.0, -2_000.0),
            speed_mps=250.0,
            heading_deg=15.0,
            flight_path_deg=10.0,
            alpha_deg=3.0,
            beta_deg=1.0,
            end_time_s=0.05,
            sample_step_s=0.05,
        )
    )

    assert result.terminated_reason == "end_time"
    assert result.samples[0].fidelity == "pseudo_6dof"
    assert result.samples[0].control_realization == "response_law"
    assert result.samples[0].position_ned_m[0] > 100.0
    assert result.samples[-1].heading_deg != pytest.approx(0.0)


####


def test_ads6_srbm_plugin_revalidates_guidance_overrides(tmp_path: Path) -> None:
    plugin = Ads6SrbmVehiclePlugin(_write_ads6_srbm_case(tmp_path, include_sensor=True))

    with pytest.raises(ValueError, match="proportional navigation requires"):
        plugin.run_batch(Ads6SrbmPluginOverrides(seeker_mode=0, guidance_mode=1))
    ####


####


def test_ads6_srbm_plugin_rejects_nonfinite_vectors() -> None:
    with pytest.raises(ValidationError, match="finite"):
        Ads6SrbmPluginOverrides(position_ned_m=(0.0, float("nan"), 0.0))
    ####


####


def test_ads6_srbm_plugin_rejects_invalid_guidance_encoding() -> None:
    with pytest.raises(ValidationError, match="guidance_mode"):
        Ads6SrbmPluginOverrides(guidance_mode=22)
    ####


####


def test_ads6_srbm_plugin_materializes_pseudo6_response_law_tuning_without_mutating_source(tmp_path: Path) -> None:
    plugin = Ads6SrbmVehiclePlugin(_write_ads6_srbm_case(tmp_path))
    installed = plugin.source_definition()

    prepared = plugin.prepare_definition(
        Ads6SrbmPluginOverrides(
            alpha_limit_deg=22.0,
            endo_boundary_altitude_m=25_000.0,
            ascent_normal_bias_g=0.75,
        )
    )

    assert prepared.aerodynamics.alpha_limit_deg == 22.0
    assert prepared.control.endo_boundary_altitude_m == 25_000.0
    assert prepared.control.ascent_normal_bias_g == 0.75
    assert plugin.source_definition() is installed
    assert installed.aerodynamics.alpha_limit_deg == 28.0
    assert installed.control.endo_boundary_altitude_m == 30_000.0


####
