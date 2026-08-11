from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.ads6_sam_plugin import Ads6SamPluginOverrides, Ads6SamVehiclePlugin
from test_ads6_sam import _write_ads6_sam_case


def test_ads6_sam_plugin_binds_exact_multi_realization_catalog_descriptor(tmp_path: Path) -> None:
    plugin = Ads6SamVehiclePlugin(_write_ads6_sam_case(tmp_path))

    assert plugin.descriptor.plugin_id == "cadac.ads6.sam"
    assert plugin.descriptor.status == "runnable"
    assert plugin.descriptor.batch_factory_id == "cadac.ads6.sam.multi_realization.batch"
    assert plugin.descriptor.default_phase_id == "fin_control"
    assert plugin.validate_installation() == ()


####


def test_ads6_sam_plugin_rejects_installation_that_cannot_support_all_advertised_realizations(
    tmp_path: Path,
) -> None:
    source_path = _write_ads6_sam_case(tmp_path)
    source_text = source_path.read_text(encoding="utf-8")
    source_path.write_text(
        source_text.replace(" tvc def,exec\n", "").replace(" rcs def,exec\n", ""),
        encoding="utf-8",
    )

    blockers = Ads6SamVehiclePlugin(source_path).validate_installation()

    assert "physical thrust-vector-control module" in " ".join(blockers)
    assert "aggregate reaction-control module" in " ".join(blockers)


####


@pytest.mark.parametrize(
    ("phase", "expected_fidelity"),
    (
        ("fin_control", "rigid_body_6dof_surface_allocated"),
        ("tvc_control", "rigid_body_6dof_surface_allocated"),
        ("aggregate_rcs", "rigid_body_6dof_direct_wrench"),
    ),
)
def test_ads6_sam_plugin_executes_exact_selected_realization(
    tmp_path: Path,
    phase: str,
    expected_fidelity: str,
) -> None:
    plugin = Ads6SamVehiclePlugin(_write_ads6_sam_case(tmp_path))
    result = plugin.run_batch(
        Ads6SamPluginOverrides(
            source_phase=phase,
            pitch_command_deg=1.0,
            tvc_mode=2 if phase == "tvc_control" else None,
            rcs_moment_mode=21 if phase == "aggregate_rcs" else None,
            rcs_force_mode=0 if phase == "aggregate_rcs" else None,
            pitch_attitude_command_deg=1.0,
            end_time_s=0.01,
            sample_step_s=0.005,
        )
    )

    assert result.terminated_reason == "end_time"
    assert result.samples[-1].source_phase == phase
    assert result.samples[-1].fidelity == expected_fidelity


####


def test_ads6_sam_plugin_rejects_zero_thrust_vector_direction(tmp_path: Path) -> None:
    plugin = Ads6SamVehiclePlugin(_write_ads6_sam_case(tmp_path))

    with pytest.raises(ValueError, match="finite and nonzero"):
        plugin.run_batch(
            Ads6SamPluginOverrides(
                source_phase="aggregate_rcs",
                rcs_moment_mode=12,
                thrust_vector_unit_body=(0.0, 0.0, 0.0),
            )
        )
    ####


####
