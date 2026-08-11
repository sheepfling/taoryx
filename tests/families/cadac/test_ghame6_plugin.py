from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.ghame6_plugin import Ghame6PluginOverrides, Ghame6VehiclePlugin
from test_ghame6 import _write_ghame6_case


def test_ghame6_plugin_binds_exact_runnable_catalog_descriptor(tmp_path: Path) -> None:
    plugin = Ghame6VehiclePlugin(_write_ghame6_case(tmp_path))

    assert plugin.descriptor.plugin_id == "cadac.ghame6.hypersonic_vehicle"
    assert plugin.descriptor.status == "runnable"
    assert plugin.descriptor.batch_factory_id == "cadac.ghame6.phase_aware.batch"
    assert plugin.descriptor.default_phase_id == "atmospheric_surfaces"
    assert plugin.validate_installation() == ()
    assert "TVC" in plugin.descriptor.claim_boundary


####


def test_ghame6_plugin_executes_bounded_phase_program(tmp_path: Path) -> None:
    plugin = Ghame6VehiclePlugin(_write_ghame6_case(tmp_path))
    result = plugin.run_batch(
        Ghame6PluginOverrides(
            aileron_command_deg=1.0,
            elevator_command_deg=2.0,
            boost_cutoff_time_s=0.65,
            terminal_lock_time_s=0.85,
            end_time_s=1.2,
            sample_step_s=0.05,
        )
    )

    assert result.terminated_reason == "end_time"
    assert result.events[-1].phase_after == "interceptor_terminal_rcs"
    assert result.samples[-1].runtime_fidelity == "rigid_body_6dof_direct_wrench"


####


def test_ghame6_plugin_rejects_zero_thrust_direction(tmp_path: Path) -> None:
    plugin = Ghame6VehiclePlugin(_write_ghame6_case(tmp_path))
    with pytest.raises(ValueError, match="positive magnitude"):
        plugin.run_batch(Ghame6PluginOverrides(thrust_vector_unit_body=(0.0, 0.0, 0.0)))
    ####


####
