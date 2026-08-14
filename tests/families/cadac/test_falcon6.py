from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from taoryx.families.cadac.falcon6 import (
    Falcon6ActuatorConfig,
    Falcon6ActuatorState,
    Falcon6AeroState,
    Falcon6DirectPlantCommand,
    Falcon6EngineState,
    Falcon6SourceError,
    Falcon6SurfaceCommand,
    falcon6_actuator_step,
    falcon6_aerodynamic_coefficients,
    falcon6_body_wrench,
    falcon6_initial_quaternion,
    falcon6_propulsion_step,
    falcon6_rotational_derivative,
    load_falcon6_source_definition,
    run_falcon6_physical_plant,
)
from taoryx.families.cadac.falcon6_plugin import Falcon6PluginOverrides, Falcon6VehiclePlugin
from taoryx.families.cadac.plugin import CADAC_PLUGIN_CATALOG, CadacPluginRegistry


def _write_falcon_case(root: Path) -> Path:
    input_path = root / "input.asc"
    input_path.write_text(
        """TITLE synthetic FALCON6
OPTIONS n_scrn
MODULES
 environment def,exec
 kinematics def,init,exec
 aerodynamics def,init,exec
 propulsion def,exec
 forces def,exec
 guidance def,exec
 control def,exec
 actuator def,exec
 euler def,init,exec
 newton def,init,exec
END
TIMING
 plot_step 0.1
 int_step 0.001
END
VEHICLES 1
 PLANE6 SyntheticF16
  sbel1 0
  sbel2 0
  sbel3 -1000
  dvbe 180
  psiblx 0
  thtblx 1
  phiblx 0
  alpha0x 1
  beta0x 0
  AERO_DECK aero.asc
  alplimpx 16
  alplimnx -6
  xcgr 1
  xcg 1
  PROP_DECK prop.asc
  mprop 2
  vmachcom 0.6
  gmach 30
  mact 2
  dlimx 20
  ddlimx 400
  wnact 50
  zetact 0.7
  maut 35
  dalimx 20
  delimx 20
  drlimx 20
  anlimpx 9
  anlimnx 6
  philimx 70
 END
ENDTIME 2
STOP
""",
        encoding="utf-8",
    )
    aero_names_1d = (
        "cxq_vs_alpha",
        "cyr_vs_alpha",
        "cyp_vs_alpha",
        "cz_vs_alpha",
        "czq_vs_alpha",
        "clr_vs_alpha",
        "clp_vs_alpha",
        "cmq_vs_alpha",
        "cnr_vs_alpha",
        "cnp_vs_alpha",
    )
    aero_names_2d = (
        "cx_vs_elev_alpha",
        "cl_vs_beta_alpha",
        "cldr_vs_beta_alpha",
        "clda_vs_beta_alpha",
        "cm_vs_elev_alpha",
        "cn_vs_beta_alpha",
        "cnda_vs_beta_alpha",
        "cndr_vs_beta_alpha",
    )
    aero_lines = ["TITLE synthetic aero"]
    for name in aero_names_1d:
        aero_lines.extend((f"1DIM {name}", "NX1 2", "-10 0", "45 0"))
    ####
    for name in aero_names_2d:
        aero_lines.extend((f"2DIM {name}", "NX1 2 NX2 2", "-30 -10 0 0", "30 45 0 0"))
    ####
    (root / "aero.asc").write_text("\n".join(aero_lines) + "\n", encoding="utf-8")
    (root / "prop.asc").write_text(
        """TITLE synthetic prop
2DIM idle_vs_mach_alt
NX1 2 NX2 2
0 0 100 100
1 50000 100 100
2DIM mil_vs_mach_alt
NX1 2 NX2 2
0 0 1000 1000
1 50000 1000 1000
2DIM max_vs_mach_alt
NX1 2 NX2 2
0 0 2000 2000
1 50000 2000 2000
3DIM ff_vs_thrust_alt_mach
NX1 2 NX2 2 NX3 2
0 0 0 0 0 0 0
2000 50000 1 0 0 0 0
""",
        encoding="utf-8",
    )
    return input_path


####


def test_falcon6_source_lowering_and_fixed_airframe(tmp_path: Path) -> None:
    definition = load_falcon6_source_definition(_write_falcon_case(tmp_path))
    assert definition.integration_step_s == pytest.approx(0.001)
    assert definition.initial_state.position_ned_m == (0.0, 0.0, -1000.0)
    assert definition.actuator.mode == 2
    assert definition.airframe.mass_kg == pytest.approx(9496.0)
    assert definition.airframe.inertia_kg_m2[0][2] == pytest.approx(-1331.4)
    assert definition.propulsion_deck.table("ff_vs_thrust_alt_mach").dimension == 3
    assert definition.taoryx_tier == "rigid_body_6dof_surface_allocated"


####


def test_falcon6_lowering_rejects_wrong_actor(tmp_path: Path) -> None:
    path = _write_falcon_case(tmp_path)
    path.write_text(path.read_text(encoding="utf-8").replace("PLANE6", "AIM5"), encoding="utf-8")
    with pytest.raises(Falcon6SourceError, match="exactly one PLANE6"):
        load_falcon6_source_definition(path)
    ####


####


def test_falcon6_initial_quaternion_is_unit_norm(tmp_path: Path) -> None:
    definition = load_falcon6_source_definition(_write_falcon_case(tmp_path))
    quaternion = np.asarray(falcon6_initial_quaternion(definition.initial_state))
    assert np.linalg.norm(quaternion) == pytest.approx(1.0)


####


def test_falcon6_ideal_actuator_clips_physical_surfaces() -> None:
    config = Falcon6ActuatorConfig(
        mode=0,
        position_limit_deg=20.0,
        rate_limit_deg_s=400.0,
        natural_frequency_rad_s=50.0,
        damping_ratio=0.7,
    )
    result = falcon6_actuator_step(
        config,
        Falcon6ActuatorState(),
        Falcon6SurfaceCommand(aileron_deg=30.0, elevator_deg=-10.0, rudder_deg=-25.0),
        0.001,
    )
    assert result.achieved.vector() == pytest.approx((20.0, -10.0, -20.0))
    assert result.position_limited == (True, False, True)


####


def test_falcon6_second_order_actuator_exposes_requested_vs_achieved() -> None:
    config = Falcon6ActuatorConfig(
        mode=2,
        position_limit_deg=20.0,
        rate_limit_deg_s=400.0,
        natural_frequency_rad_s=50.0,
        damping_ratio=0.7,
    )
    command = Falcon6SurfaceCommand(elevator_deg=10.0)
    first = falcon6_actuator_step(config, Falcon6ActuatorState(), command, 0.001)
    second = falcon6_actuator_step(config, first.state, command, 0.001)
    assert first.requested.elevator_deg == 10.0
    assert first.achieved.elevator_deg == pytest.approx(0.0)
    assert second.achieved.elevator_deg > 0.0
    assert second.achieved.elevator_deg < command.elevator_deg


####


def test_falcon6_plant_preserves_surface_limit_feedback(tmp_path: Path) -> None:
    definition = load_falcon6_source_definition(_write_falcon_case(tmp_path))
    result = run_falcon6_physical_plant(
        definition,
        Falcon6DirectPlantCommand(
            surfaces=Falcon6SurfaceCommand(aileron_deg=30.0, elevator_deg=0.0, rudder_deg=-25.0),
        ),
        end_time_s=0.1,
        sample_step_s=0.01,
    )

    assert any(any(sample.surface_position_limited) for sample in result.samples)
    assert any(any(sample.surface_rate_limited) for sample in result.samples)
    ####


####


def test_falcon6_second_order_actuator_remains_bounded() -> None:
    config = Falcon6ActuatorConfig(
        mode=2,
        position_limit_deg=20.0,
        rate_limit_deg_s=50.0,
        natural_frequency_rad_s=50.0,
        damping_ratio=0.7,
    )
    state = Falcon6ActuatorState()
    command = Falcon6SurfaceCommand(aileron_deg=100.0, elevator_deg=-100.0, rudder_deg=100.0)
    saw_rate_limit = False
    for _ in range(5000):
        step = falcon6_actuator_step(config, state, command, 0.001)
        saw_rate_limit = saw_rate_limit or any(step.rate_limited)
        state = step.state
    ####
    assert max(abs(value) for value in step.achieved.vector()) <= 20.1
    assert saw_rate_limit
    # CADAC clamps the stored rate before integration, so the post-integration
    # internal rate may overshoot and is clamped again on the next source step.
    assert max(abs(value) for value in state.rate_deg_s) > config.rate_limit_deg_s


####


def test_falcon6_propulsion_spool_uses_source_table_units(tmp_path: Path) -> None:
    definition = load_falcon6_source_definition(_write_falcon_case(tmp_path))
    step = falcon6_propulsion_step(
        definition,
        Falcon6EngineState(),
        mach=0.5,
        altitude_m=1000.0,
        dt_s=0.001,
    )
    assert step.throttle == pytest.approx(0.77)
    assert step.military_thrust_n == pytest.approx(1000.0 * 4.448)
    assert step.maximum_thrust_n == pytest.approx(2000.0 * 4.448)
    assert step.achieved_power_percent > 0.0


####


def test_falcon6_force_moment_closure_is_physical_effector_backed(tmp_path: Path) -> None:
    definition = load_falcon6_source_definition(_write_falcon_case(tmp_path))
    coefficients = falcon6_aerodynamic_coefficients(
        definition,
        Falcon6AeroState(alpha_deg=0.0, beta_deg=0.0, speed_mps=180.0, body_rates_deg_s=(0.0, 0.0, 0.0)),
        Falcon6SurfaceCommand(),
    )
    assert coefficients.cx == pytest.approx(0.0)
    assert coefficients.cy == pytest.approx(0.0)
    assert coefficients.cz == pytest.approx(0.0)
    wrench = falcon6_body_wrench(definition, coefficients, dynamic_pressure_pa=10_000.0, thrust_n=5000.0)
    assert wrench.force_n == pytest.approx((5000.0, 0.0, 0.0))
    assert wrench.moment_nm == pytest.approx((0.0, 0.0, 0.0))


####


def test_falcon6_rotational_derivative_includes_source_inertia(tmp_path: Path) -> None:
    definition = load_falcon6_source_definition(_write_falcon_case(tmp_path))
    derivative = falcon6_rotational_derivative(
        definition,
        body_rates_rad_s=(0.0, 0.0, 0.0),
        moment_nm=(12875.0, 0.0, -1331.4),
    )
    assert derivative.angular_acceleration_rad_s2 == pytest.approx((1.0, 0.0, 0.0), abs=1.0e-12)


####


def test_falcon6_physical_plant_runs_source_ordered_rigid_body_slice(tmp_path: Path) -> None:
    definition = load_falcon6_source_definition(_write_falcon_case(tmp_path))
    result = run_falcon6_physical_plant(
        definition,
        Falcon6DirectPlantCommand(surfaces=Falcon6SurfaceCommand(elevator_deg=10.0)),
        end_time_s=0.05,
        sample_step_s=0.01,
    )
    assert result.terminated_reason == "end_time"
    assert result.executed_steps >= 50
    assert len(result.samples) >= 5
    assert all(math.isfinite(value) for sample in result.samples for value in sample.position_ned_m)
    assert result.samples[-1].mach > 0.0
    assert result.samples[-1].thrust_n > 0.0


####


def test_falcon6_physical_plant_preserves_aero_before_actuator_order(tmp_path: Path) -> None:
    definition = load_falcon6_source_definition(_write_falcon_case(tmp_path))
    result = run_falcon6_physical_plant(
        definition,
        Falcon6DirectPlantCommand(surfaces=Falcon6SurfaceCommand(elevator_deg=10.0)),
        end_time_s=0.02,
        sample_step_s=0.001,
    )
    assert result.samples[0].aero_surfaces_deg == pytest.approx((0.0, 0.0, 0.0))
    assert result.samples[0].achieved_surfaces_deg == pytest.approx((0.0, 0.0, 0.0))
    later = next(sample for sample in result.samples[1:] if abs(sample.achieved_surfaces_deg[1]) > 0.0)
    assert later.aero_surfaces_deg[1] != pytest.approx(later.achieved_surfaces_deg[1])


####


def test_falcon6_vehicle_plugin_is_exact_second_runtime(tmp_path: Path) -> None:
    path = _write_falcon_case(tmp_path)
    plugin = Falcon6VehiclePlugin(path)
    assert plugin.validate_installation() == ()
    assert plugin.descriptor.status == "runnable"
    assert plugin.descriptor.batch_factory_id == "cadac.falcon6.physical_surface.batch"
    registry = CadacPluginRegistry(CADAC_PLUGIN_CATALOG.plugins)
    registry.register_runtime(plugin)
    assert registry.runtime_ids() == ("cadac.falcon6.aircraft",)
    run = plugin.run_batch(
        Falcon6PluginOverrides(
            elevator_command_deg=5.0,
            end_time_s=0.02,
            sample_step_s=0.01,
        )
    )
    assert run.terminated_reason == "end_time"
    assert run.samples[-1].requested_surfaces_deg[1] == pytest.approx(5.0)


####


def test_falcon6_plugin_does_not_mutate_installed_source_definition(tmp_path: Path) -> None:
    plugin = Falcon6VehiclePlugin(_write_falcon_case(tmp_path))
    source = plugin.source_definition()
    run = plugin.run_batch(Falcon6PluginOverrides(speed_mps=200.0, end_time_s=0.01))
    assert run.samples[0].velocity_body_mps[0] != pytest.approx(source.initial_state.speed_mps)
    assert plugin.source_definition().initial_state.speed_mps == pytest.approx(180.0)


####
