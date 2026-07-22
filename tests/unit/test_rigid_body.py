from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from taoryx import (
    AerodynamicOutput,
    AeroQueryContext,
    MassProperties,
    PreparedAerodynamicCoefficients,
    PreparedCoefficientTable,
    RigidBody6DofModel,
    RigidBody6DofState,
    RigidBodyForceMoment,
    StageDefinition,
    StagedPropulsion,
    TableAerodynamicModel,
    ThermalLimits,
    assess_thermal_limits,
)
from taoryx.contracts import EarthModel, Frame, FrameVector3, Quantity, Unit, Vector3
from taoryx.modes import DynamicsMode, Quaternion
from taoryx.rigid_body_frames import EarthRotationAdapter
from taoryx.runtime import (
    EnvironmentSample,
    FlightPhase,
    FlightPhaseMachine,
    PhaseTransition,
    RigidBodyLoadPipeline,
    RuntimeProblem,
    StaticEnvironmentProvider,
    ThermalEntryController,
    bounded_attitude_moment,
    rigid_body_vehicle,
)
from taoryx.runtime.engine import integrate_active_vehicles
from taoryx.tables import prepare_table


def initial_state() -> RigidBody6DofState:
    return RigidBody6DofState(
        0.0,
        FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECIC),
        FrameVector3(Vector3(100.0, 0.0, 0.0), Frame.ECIC),
        Quaternion.identity(),
        Vector3(0.0, 0.0, 0.0),
        100.0,
        40.0,
    )
    ####


def test_rigid_body_state_round_trips_and_normalizes_attitude() -> None:
    state = initial_state()
    values = state.to_values()
    restored = RigidBody6DofState.from_values(2.0, values)

    assert restored.time == pytest.approx(2.0)
    assert restored.position == state.position
    assert restored.mass == pytest.approx(state.mass)
    assert restored.attitude == state.attitude
    ####


def test_rigid_body_force_and_moment_produce_independent_translation_and_rotation() -> None:
    model = RigidBody6DofModel(
        inertia=Vector3(2.0, 4.0, 8.0),
        force_moment=lambda state: RigidBodyForceMoment(Vector3(10.0, 0.0, 0.0), Vector3(0.0, 0.0, 8.0)),
    )

    rates = model.derivative(initial_state())

    assert rates[3:6] == pytest.approx((0.1, 0.0, 0.0))
    assert rates[10:13] == pytest.approx((0.0, 0.0, 1.0))
    assert rates[13:15] == pytest.approx((0.0, 0.0))
    ####


def test_rigid_body_load_pipeline_adds_force_moment_mass_flow_and_heat() -> None:
    pipeline = RigidBodyLoadPipeline(
        (
            lambda state: RigidBodyForceMoment(Vector3(2.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0), 0.5, 10.0),
            lambda state: RigidBodyForceMoment(Vector3(0.0, 3.0, 0.0), Vector3(0.0, 0.0, 4.0), 1.5, 20.0),
        )
    )

    load = pipeline.evaluate(initial_state())

    assert load.force_body == Vector3(2.0, 3.0, 0.0)
    assert load.moment_body == Vector3(0.0, 1.0, 4.0)
    assert load.propellant_mass_rate == pytest.approx(2.0)
    assert load.heat_rate == pytest.approx(30.0)
    ####


def test_staged_propulsion_resolves_force_moment_and_mass_flow_by_time() -> None:
    propulsion = StagedPropulsion(
        (
            StageDefinition("stage-1", 0.0, 2.0, Vector3(100.0, 0.0, 0.0), 2.0),
            StageDefinition("stage-2", 2.0, 4.0, Vector3(50.0, 0.0, 0.0), 1.0, Vector3(0.0, 0.0, 5.0)),
        )
    )

    first = propulsion.evaluate(1.0, 10.0)
    second = propulsion.evaluate(2.0, 10.0)
    depleted = propulsion.evaluate(2.0, 0.0)

    assert first.force_body_n == Vector3(100.0, 0.0, 0.0)
    assert first.propellant_mass_rate_kg_s == pytest.approx(2.0)
    assert first.active_stage_ids == ("stage-1",)
    assert second.moment_body_nm == Vector3(0.0, 0.0, 5.0)
    assert second.active_stage_ids == ("stage-2",)
    assert depleted.propellant_mass_rate_kg_s == 0.0
    ####


def test_mass_properties_require_dry_mass_and_fit_propellant() -> None:
    properties = MassProperties(100.0, 60.0, 40.0, Vector3(0.0, 0.0, 0.0), Vector3(2.0, 3.0, 4.0))

    assert properties.total_mass_kg == 100.0
    with pytest.raises(ValueError, match="propellant mass"):
        MassProperties(100.0, 60.0, 41.0, Vector3(0.0, 0.0, 0.0), Vector3(2.0, 3.0, 4.0))
    ####


def test_table_aerodynamic_model_resolves_air_data_and_body_loads() -> None:
    earth = EarthModel(
        Quantity(6378.137, Unit.KILOMETER),
        1.0 / 298.257223563,
        Quantity(398600.4418, Unit.METER_CUBED_PER_SECOND_SQUARED),
        Quantity(7.2921150e-5, Unit.RADIAN_PER_SECOND),
    )
    environment = StaticEnvironmentProvider(
        EnvironmentSample(
            density=1.0,
            pressure=101325.0,
            temperature=288.15,
            speed_of_sound=340.0,
            wind=FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
        )
    )
    model = TableAerodynamicModel(
        environment,
        EarthRotationAdapter(earth),
        2.0,
        4.0,
        lambda mach, alpha, beta: Vector3(-1.0, 0.1 * beta, -2.0 * alpha),
        lambda mach, alpha, beta: Vector3(0.0, 0.5 * alpha, 0.0),
    )
    position = FrameVector3(Vector3(6_378_137.0, 0.0, 0.0), Frame.ECIC)
    velocity = FrameVector3(Vector3(100.0, 7.2921150e-5 * 6_378_137.0, 0.0), Frame.ECIC)
    state = RigidBody6DofState(0.0, position, velocity, Quaternion.identity(), Vector3(0.0, 0.0, 0.0), 100.0, 0.0)

    output = model.evaluate(state)

    assert isinstance(output, AerodynamicOutput)
    assert output.airspeed_m_s == pytest.approx(100.0)
    assert output.dynamic_pressure_pa == pytest.approx(5_000.0)
    assert output.force_body_n.x == pytest.approx(-10_000.0)
    assert output.moment_body_nm == Vector3(0.0, 0.0, 0.0)
    assert output.query_values["mach"] == pytest.approx(100.0 / 340.0)
    assert output.query_values["alpha"] == pytest.approx(0.0)
    assert output.table_margins == {}
    ####


def test_prepared_aerodynamic_coefficients_interpolate_and_reject_envelope() -> None:
    def coefficient(name: str, values: tuple[float, float]) -> PreparedCoefficientTable:
        return PreparedCoefficientTable(name, ("mach",), prepare_table(((1.0, 2.0),), values))
        ####
    ####

    tables = PreparedAerodynamicCoefficients(
        {
            "cx": coefficient("cx", (-1.0, -2.0)),
            "cy": coefficient("cy", (0.0, 0.1)),
            "cz": coefficient("cz", (0.0, -0.2)),
        }
    )

    coefficients = tables.force_provider()(1.5, 0.0, 0.0)

    assert coefficients == Vector3(-1.5, 0.05, -0.1)
    assert tables.table_margins({"mach": 1.5}) == {
        "force.cx.mach": pytest.approx(0.5),
        "force.cy.mach": pytest.approx(0.5),
        "force.cz.mach": pytest.approx(0.5),
    }
    with pytest.raises(ValueError, match="outside"):
        tables.force_provider()(2.5, 0.0, 0.0)
    ####


def test_prepared_coefficient_table_accepts_machine_scale_boundary_noise() -> None:
    table = PreparedCoefficientTable(
        "control",
        ("control",),
        prepare_table(((math.radians(-14.9), math.radians(34.9)),), (1.0, 2.0)),
    )

    assert table.evaluate({"control": math.radians(-14.9) - 1.0e-14}) == pytest.approx(1.0)
    with pytest.raises(ValueError, match="outside"):
        table.evaluate({"control": math.radians(-14.9) - 1.0e-9})
    ####


def test_prepared_aerodynamic_coefficients_adapt_runtime_tables() -> None:
    prepared = prepare_table(((1.0, 2.0),), (-1.0, -2.0))
    runtime_tables = {
        name: SimpleNamespace(independent_variables=("mach",), output_variable=name, prepared=prepared)
        for name in ("cx", "cy", "cz")
    }

    coefficients = PreparedAerodynamicCoefficients.from_runtime_tables(runtime_tables).force_provider()(1.5, 0.0, 0.0)

    assert coefficients == Vector3(-1.5, -1.5, -1.5)
    ####


def test_prepared_aerodynamic_coefficients_accept_control_surface_variables() -> None:
    def coefficient(name: str, values: tuple[float, ...]) -> PreparedCoefficientTable:
        return PreparedCoefficientTable(name, ("mach", "fin_pitch"), prepare_table(((1.0, 2.0), (0.0, 1.0)), values))
        ####
    ####

    tables = PreparedAerodynamicCoefficients(
        {
            "cx": coefficient("cx", (-1.0, -1.2, -2.0, -2.2)),
            "cy": coefficient("cy", (0.0, 0.1, 0.0, 0.1)),
            "cz": coefficient("cz", (0.0, 0.0, -0.2, -0.2)),
        }
    )

    coefficients = tables.context_force_provider()(AeroQueryContext.from_air_data(1.5, 0.0, 0.0, {"fin_pitch": 0.5}))

    assert coefficients == Vector3(-1.6, 0.05, -0.1)
    ####


def test_rigid_body_model_stops_mass_flow_at_dry_mass() -> None:
    state = RigidBody6DofState(
        0.0,
        FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECIC),
        FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECIC),
        Quaternion.identity(),
        Vector3(0.0, 0.0, 0.0),
        60.0,
        0.0,
    )
    model = RigidBody6DofModel(
        Vector3(2.0, 3.0, 4.0),
        lambda current: RigidBodyForceMoment(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0), 2.0),
        dry_mass=60.0,
    )

    assert model.derivative(state)[13:15] == (0.0, 0.0)
    ####


def test_rigid_body_runtime_vehicle_integrates_stage_mass_and_heat() -> None:
    model = RigidBody6DofModel(
        inertia=Vector3(2.0, 4.0, 8.0),
        force_moment=lambda state: RigidBodyForceMoment(
            Vector3(10.0, 0.0, 0.0),
            Vector3(0.0, 0.0, 8.0),
            propellant_mass_rate=2.0,
            heat_rate=5.0,
        ),
    )
    vehicle = rigid_body_vehicle("stage-1", initial_state(), model, step_size=0.1)
    assert vehicle.dynamics_mode is DynamicsMode.RIGID_BODY_6DOF

    integrate_active_vehicles(RuntimeProblem({vehicle.name: vehicle}), 1.0)

    assert vehicle.state.time == pytest.approx(1.0)
    assert vehicle.state.named["mass"] == pytest.approx(98.0)
    assert vehicle.state.named["propellant_mass"] == pytest.approx(38.0)
    assert vehicle.state.named["heat_load"] == pytest.approx(5.0)
    assert vehicle.state.named["wz"] > 0.0
    ####


def test_rigid_body_runtime_publishes_force_moment_heat_and_orientation_observables() -> None:
    model = RigidBody6DofModel(
        inertia=Vector3(2.0, 4.0, 8.0),
        force_moment=lambda state: RigidBodyForceMoment(
            Vector3(10.0, 2.0, 1.0),
            Vector3(0.0, 0.0, 8.0),
            heat_rate=5.0,
        ),
    )
    vehicle = rigid_body_vehicle("observable", initial_state(), model, step_size=0.1)

    integrate_active_vehicles(RuntimeProblem({vehicle.name: vehicle}), 0.1)

    assert vehicle.state.named["force_body_x_n"] == pytest.approx(10.0)
    assert vehicle.state.named["moment_body_z_nm"] == pytest.approx(8.0)
    assert vehicle.state.named["total_force_ecic_n"] > 0.0
    assert vehicle.state.named["heat_rate_w_m2"] == pytest.approx(5.0)
    assert vehicle.state.named["roll_deg"] == pytest.approx(0.0)
    assert vehicle.state.named["pitch_deg"] == pytest.approx(0.0)
    assert vehicle.state.named["yaw_deg"] != pytest.approx(0.0)
    ####


def test_thermal_assessment_exposes_entry_margins() -> None:
    state = initial_state()
    assessment = assess_thermal_limits(state, ThermalLimits(10.0, 20.0), 8.0)

    assert assessment.safe
    assert assessment.heat_rate_margin == pytest.approx(2.0)
    assert assessment.heat_load_margin == pytest.approx(20.0)
    ####


def test_phase_contract_names_staged_reentry_sequence() -> None:
    transitions = (
        PhaseTransition(FlightPhase.STAGE_1_POWERED, FlightPhase.STAGE_1_SEPARATION, "stage-1-empty"),
        PhaseTransition(FlightPhase.STAGE_1_SEPARATION, FlightPhase.STAGE_2_POWERED, "stage-1-separated"),
        PhaseTransition(FlightPhase.STAGE_2_POWERED, FlightPhase.COAST_TO_APOGEE, "stage-2-burnout"),
        PhaseTransition(FlightPhase.COAST_TO_APOGEE, FlightPhase.ENTRY_THERMAL_CONTROL, "apogee"),
        PhaseTransition(FlightPhase.ENTRY_THERMAL_CONTROL, FlightPhase.TERMINAL_GUIDANCE, "terminal-envelope"),
        PhaseTransition(FlightPhase.TERMINAL_GUIDANCE, FlightPhase.IMPACT, "ground-hit"),
    )

    assert transitions[0].source is FlightPhase.STAGE_1_POWERED
    assert transitions[-1].target is FlightPhase.IMPACT
    ####


def test_phase_machine_rejects_out_of_order_events() -> None:
    machine = FlightPhaseMachine(
        FlightPhase.STAGE_1_POWERED,
        (PhaseTransition(FlightPhase.STAGE_1_POWERED, FlightPhase.STAGE_1_SEPARATION, "stage-1-empty"),),
    )

    assert machine.transition("stage-1-empty") is FlightPhase.STAGE_1_SEPARATION
    with pytest.raises(ValueError, match="invalid"):
        machine.transition("apogee")
    ####


def test_thermal_entry_controller_reduces_command_near_limit() -> None:
    controller = ThermalEntryController(ThermalLimits(100.0, 1000.0), maximum_angle_of_attack_radians=0.4)

    command = controller.command(0.3, heat_rate=95.0, heat_load=100.0)

    assert command.limited
    assert command.angle_of_attack_radians < 0.3
    ####


def test_bounded_attitude_moment_reports_saturation() -> None:
    result = bounded_attitude_moment(
        Vector3(2.0, 0.0, 0.0),
        Vector3(0.0, 0.0, 0.0),
        attitude_gain=10.0,
        rate_damping=1.0,
        maximum_moment=5.0,
    )

    assert result.saturated
    assert result.moment_body.norm() == pytest.approx(5.0)
    ####
