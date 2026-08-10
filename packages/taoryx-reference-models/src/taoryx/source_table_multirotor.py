"""Runtime-owned source-table plants for multirotor local witnesses.

The Hummingbird builder is intentionally the exact local RotorPy-derived
plant used by the physical-control validation packet.  It does not infer a
generic quadcopter from the ``multirotor`` family name and it does not make a
hover witness into a waypoint, battery, or envelope qualification.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from functools import lru_cache

from taoryx_reference_models.resources import model_resource_root

from .contracts import Vector3
from .control_allocation import EffectorLimits
from .control_automation import ControlAutomationDeclaration
from .direct_wrench import DIRECT_WRENCH_NAMES, DirectWrenchLimits, add_direct_wrench_to_local_derivative
from .language.grammar_contracts import GrammarProfile
from .local_direct_wrench import LocalDirectWrenchScreenConfig
from .physical_lqr import PhysicalWrenchLqiDesign, design_physical_wrench_lqi, project_linearization_to_wrench
from .runtime.program import LoadedProgram
from .runtime_control_adapter import RuntimeRigidBodyLocalPlant, local_rigid_body_plant_from_vehicle
from .tuning_campaign import TuningCampaign

ROOT = model_resource_root()
HUMMINGBIRD_SOURCE_PROBLEM = ROOT / "examples/generated/vehicles/hummingbird_individual_rotor_hover_6dof.prb"
TABLE_ROOT = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables"
HUMMINGBIRD_SOURCE_TABLES = tuple(
    TABLE_ROOT / name
    for name in (
        "hummingbird_cx.tbl",
        "hummingbird_cy.tbl",
        "hummingbird_cz.tbl",
        "hummingbird_cmx.tbl",
        "hummingbird_cmy.tbl",
        "hummingbird_cmz.tbl",
    )
)
SOURCE_ROOT = TABLE_ROOT.parent / "quadcopter_hummingbird"
HUMMINGBIRD_SOURCE_ASSETS = (
    SOURCE_ROOT / "VALIDITY.md",
    SOURCE_ROOT / "scripts/wrench_evaluator.py",
    SOURCE_ROOT / "controls/control_allocation_matrix.csv",
    SOURCE_ROOT / "controls/differential_speed_response.csv",
    SOURCE_ROOT / "geometry_mass/parameters.csv",
    SOURCE_ROOT / "geometry_mass/rotor_positions.csv",
    SOURCE_ROOT / "propulsion/rotor_static_map.csv",
)

HUMMINGBIRD_LOCAL_DIRECT_WRENCH_STATE_NAMES = (
    "u_m_s",
    "v_m_s",
    "w_m_s",
    "p_rad_s",
    "q_rad_s",
    "r_rad_s",
)


@lru_cache(maxsize=1)
def build_hummingbird_local_physical_wrench_lqi_design() -> PhysicalWrenchLqiDesign:
    """Build the source-hover offset-free attitude LQI through motor allocation.

    The controller operates on requested roll, pitch, and yaw moments.  It
    does not directly modify the plant: the caller must allocate every demand
    into the four bounded, lagged source motor-speed coordinates.
    """

    plant = build_hummingbird_individual_rotor_source_table_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    if not trim.success:
        raise RuntimeError(f"Hummingbird source-hover trim did not converge: {trim.as_dict()}")
    linearization = plant.linearize(
        trim,
        {
            "state_step": 1.0e-4,
            "control_step": 1.0,
            "comparison_factor": 0.5,
            "maximum_relative_difference": 0.10,
            "comparison_absolute_floor": 1.0e-7,
        },
    )
    effectiveness = plant.effectiveness(trim.state, trim.controls)
    projection = project_linearization_to_wrench(
        linearization,
        effectiveness,
        state_names=(
            "roll_error_rad",
            "pitch_error_rad",
            "yaw_error_rad",
            "p_rad_s",
            "q_rad_s",
            "r_rad_s",
        ),
        wrench_names=("moment_x_nm", "moment_y_nm", "moment_z_nm"),
        effector_names=tuple(plant.control_names),
    )
    return design_physical_wrench_lqi(
        "hummingbird.local_source_hover_wrench_lqi.v1",
        projection,
        output_names=("roll_error_rad", "pitch_error_rad", "yaw_error_rad"),
        q_diagonal=(20.0, 20.0, 10.0, 4.0, 4.0, 2.0),
        r_diagonal=(1.0, 1.0, 1.0),
        integral_q_diagonal=(40.0, 40.0, 20.0),
        state_scales=(
            math.radians(10.0),
            math.radians(10.0),
            math.radians(15.0),
            math.radians(60.0),
            math.radians(60.0),
            math.radians(60.0),
        ),
        wrench_scales=(0.10, 0.10, 0.05),
    )
    ####


@lru_cache(maxsize=1)
def build_hummingbird_local_vertical_force_lqi_design() -> PhysicalWrenchLqiDesign:
    """Build the source-hover vertical-speed LQI through physical rotors.

    The selected four controlled wrench coordinates are collective body-z
    force plus the three body moments.  The LQI outputs deliberately include
    local vertical speed alongside attitude error; a composition-owned outer
    position layer supplies the bounded speed reference.  Every force and
    moment demand still crosses the six-axis effectiveness and bounded rotor
    allocation boundary before reaching the nonlinear source plant.
    """

    plant = build_hummingbird_individual_rotor_source_table_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    if not trim.success:
        raise RuntimeError(f"Hummingbird source-hover trim did not converge: {trim.as_dict()}")
    linearization = plant.linearize(
        trim,
        {
            "state_step": 1.0e-4,
            "control_step": 1.0,
            "comparison_factor": 0.5,
            "maximum_relative_difference": 0.10,
            "comparison_absolute_floor": 1.0e-7,
        },
    )
    effectiveness = plant.force_moment_effectiveness(trim.state, trim.controls)
    projection = project_linearization_to_wrench(
        linearization,
        effectiveness,
        state_names=(
            "roll_error_rad",
            "pitch_error_rad",
            "yaw_error_rad",
            "w_m_s",
            "p_rad_s",
            "q_rad_s",
            "r_rad_s",
        ),
        wrench_names=("force_z_n", "moment_x_nm", "moment_y_nm", "moment_z_nm"),
        effector_names=tuple(plant.control_names),
    )
    return design_physical_wrench_lqi(
        "hummingbird.local_source_hover_vertical_force_lqi.v1",
        projection,
        output_names=("roll_error_rad", "pitch_error_rad", "yaw_error_rad", "w_m_s"),
        q_diagonal=(20.0, 20.0, 10.0, 8.0, 4.0, 4.0, 2.0),
        r_diagonal=(1.0, 1.0, 1.0, 1.0),
        integral_q_diagonal=(40.0, 40.0, 20.0, 16.0),
        state_scales=(
            math.radians(10.0),
            math.radians(10.0),
            math.radians(15.0),
            1.0,
            math.radians(60.0),
            math.radians(60.0),
            math.radians(60.0),
        ),
        wrench_scales=(1.0, 0.10, 0.10, 0.05),
    )
    ####


def build_hummingbird_source_rotor_lqi_tuning_campaign() -> TuningCampaign:
    """Declare the reusable source-hover rotor-coordinate LQI design screen.

    The declaration keeps all local body states so the shared tuner checks
    source-table attitude, velocity, and rate authority together.  The
    controlled outputs stay limited to attitude error, matching the local
    physical screen.  This produces a candidate motor-coordinate controller;
    the separate bounded nonlinear screen remains the only rotor-allocation
    execution proof.
    """

    return ControlAutomationDeclaration(
        id="hummingbird-source-rotor-full-local",
        campaign_id="hummingbird-source-rotor-local-lqi-v1",
        family_id="hummingbird",
        tier="rigid_body_6dof_surface_allocated",
        strategy_id="multirotor_hover_translation.v1",
        node_id="source-hover-local",
        state_scales={
            "roll_error_rad": 0.2,
            "pitch_error_rad": 0.2,
            "yaw_error_rad": 0.3,
            "u_m_s": 3.0,
            "v_m_s": 3.0,
            "w_m_s": 3.0,
            "p_rad_s": 1.0,
            "q_rad_s": 1.0,
            "r_rad_s": 1.0,
        },
        control_scales={f"rotor-{index}-speed": 500.0 for index in range(1, 5)},
        authority_state_names=(
            "roll_error_rad",
            "pitch_error_rad",
            "yaw_error_rad",
            "u_m_s",
            "v_m_s",
            "w_m_s",
            "p_rad_s",
            "q_rad_s",
            "r_rad_s",
        ),
        offset_free_outputs=("roll_error_rad", "pitch_error_rad", "yaw_error_rad"),
        profile_grid_id_prefix="hummingbird-source-rotor-local",
    ).build_campaign()
    ####


@lru_cache(maxsize=1)
def build_hummingbird_local_direct_wrench_screen_config() -> LocalDirectWrenchScreenConfig:
    """Bind the source-hover direct-wrench comparator to the shared screen runner.

    The direct wrench is injected alongside the held source-hover rotor trim;
    it deliberately bypasses allocation.  This makes it a local bridge screen,
    never evidence for motor allocation, translation, or a flight mode.
    """

    plant = build_hummingbird_individual_rotor_source_table_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    if not trim.success:
        raise RuntimeError(f"Hummingbird source-hover trim did not converge: {trim.as_dict()}")
    limits = DirectWrenchLimits(
        lower={
            "force_x_n": -0.25,
            "force_y_n": -0.25,
            "force_z_n": -0.50,
            "moment_x_nm": -0.01,
            "moment_y_nm": -0.01,
            "moment_z_nm": -0.01,
        },
        upper={
            "force_x_n": 0.25,
            "force_y_n": 0.25,
            "force_z_n": 0.50,
            "moment_x_nm": 0.01,
            "moment_y_nm": 0.01,
            "moment_z_nm": 0.01,
        },
        rate_limit_per_s={name: None for name in DIRECT_WRENCH_NAMES},
    )
    reference = {
        name: float(trim.state[name])
        for name in HUMMINGBIRD_LOCAL_DIRECT_WRENCH_STATE_NAMES
    }
    initial = dict(reference)
    initial.update(
        {
            "u_m_s": reference["u_m_s"] + 0.10,
            "v_m_s": reference["v_m_s"] - 0.06,
            "p_rad_s": reference["p_rad_s"] + 0.08,
            "r_rad_s": reference["r_rad_s"] - 0.06,
        }
    )
    source_state = dict(trim.state)
    source_effectors = dict(trim.controls)
    zero_wrench = {name: 0.0 for name in DIRECT_WRENCH_NAMES}

    def derivative(state: Mapping[str, float], wrench: Mapping[str, float]) -> dict[str, float]:
        complete_state = {
            **source_state,
            **{
                name: float(state[name])
                for name in HUMMINGBIRD_LOCAL_DIRECT_WRENCH_STATE_NAMES
            },
        }
        base = plant.state_derivative(complete_state, source_effectors, {})
        projection = limits.project(wrench, zero_wrench, 1.0)
        augmented = add_direct_wrench_to_local_derivative(
            base,
            projection,
            mass_kg=float(plant.source_state.mass),
            inertia_kg_m2=plant.inertia_kg_m2,
        )
        return {
            name: float(augmented[name])
            for name in HUMMINGBIRD_LOCAL_DIRECT_WRENCH_STATE_NAMES
        }
        ####

    return LocalDirectWrenchScreenConfig(
        id="hummingbird-source-hover-local-direct-wrench-v1",
        plant_id="hummingbird-individual-rotor-source-direct-wrench-bridge",
        state_names=HUMMINGBIRD_LOCAL_DIRECT_WRENCH_STATE_NAMES,
        reference_state=reference,
        initial_state=initial,
        source_derivative=derivative,
        balancing_wrench=lambda _state: dict(zero_wrench),
        limits=limits,
        state_scales=(2.0, 2.0, 2.0, 1.0, 1.0, 1.0),
        control_scales=(0.25, 0.25, 0.50, 0.01, 0.01, 0.01),
        state_cost_weights=(8.0, 8.0, 8.0, 2.0, 2.0, 2.0),
        # This slightly stronger control penalty keeps the common screen's
        # source-hover recovery inside the declared six-axis bounds, so a
        # pass does not rely on hidden wrench clipping.
        control_cost_weights=(0.02, 0.02, 0.02, 0.02, 0.02, 0.02),
        dt_s=0.002,
        duration_s=2.0,
        final_error_fraction_limit=0.20,
        resource_values={"mass_kg": float(plant.source_state.mass)},
    )
    ####


def build_hummingbird_individual_rotor_source_table_plant() -> RuntimeRigidBodyLocalPlant:
    """Build the four-effector Hummingbird source-hover local plant.

    The source provides a first-order motor time constant but no independent
    hard motor slew-rate bound.  The runtime therefore declares motor lag and
    speed limits while leaving the rate limit unset rather than inventing one.
    """

    program = LoadedProgram.load(
        HUMMINGBIRD_SOURCE_PROBLEM,
        HUMMINGBIRD_SOURCE_TABLES,
        profile=GrammarProfile.TAORYX,
    )
    limits = {
        f"rotor-{index}-speed": EffectorLimits(
            f"rotor-{index}-speed",
            0.0,
            1500.0,
            "rad/s",
            time_constant_s=0.005,
        )
        for index in range(1, 5)
    }
    return local_rigid_body_plant_from_vehicle(
        "hummingbird-individual-rotor-source-plant",
        "rotorpy-hover-local-v1",
        program.case().vehicles["1"],
        inertia_kg_m2=Vector3(0.00365, 0.00368, 0.00703),
        reference_length_m=0.34,
        effector_limits=limits,
        effectiveness_steps={name: 1.0 for name in limits},
        allocation_wrench_weights={
            "moment_x_nm": 1.0,
            "moment_y_nm": 1.0,
            "moment_z_nm": 1.0,
        },
        # Rotor-speed effectiveness is approximately 1e-4 N m/(rad/s).
        # The tiny regularizer only resolves the redundant rotor direction;
        # it must not turn an otherwise attainable local wrench into a false
        # infeasibility report.
        allocation_regularization=1.0e-16,
        allocation_feasibility_tolerance=1.0e-6,
    )
    ####


__all__ = [
    "HUMMINGBIRD_SOURCE_ASSETS",
    "HUMMINGBIRD_SOURCE_PROBLEM",
    "HUMMINGBIRD_SOURCE_TABLES",
    "build_hummingbird_individual_rotor_source_table_plant",
    "build_hummingbird_local_direct_wrench_screen_config",
    "build_hummingbird_local_physical_wrench_lqi_design",
    "build_hummingbird_source_rotor_lqi_tuning_campaign",
]
