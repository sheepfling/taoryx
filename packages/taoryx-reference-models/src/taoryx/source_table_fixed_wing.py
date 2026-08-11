"""Runtime-owned source-table plants for fixed-wing local-control witnesses.

These builders deliberately preserve the exact source problem, table set,
inertia, limits, and local authority settings used by the qualification
fixtures.  They do not infer a vehicle from its physical family and they do
not create a generic fixed-wing substitute.  Keeping the construction here
lets Mission Composition lower a selected X8 or B747 composition through the same
source-owned plant that developer validation uses.
"""

from __future__ import annotations

import math
from functools import lru_cache

from taoryx_reference_models.resources import model_resource_root

from .contracts import Vector3
from .control_allocation import EffectorEffectiveness, EffectorLimits
from .control_automation import ControlAutomationDeclaration
from .generic_tuning import AuthorityPreflightReport, LinearAuthorityRequirement, linear_authority_preflight
from .language.grammar_contracts import GrammarProfile
from .physical_lqr import (
    PhysicalWrenchLqiDesign,
    PhysicalWrenchLqrDesign,
    design_physical_wrench_lqi,
    design_physical_wrench_lqr,
    project_linearization_to_wrench,
)
from .runtime.program import LoadedProgram
from .runtime_control_adapter import RuntimeRigidBodyLocalPlant, local_rigid_body_plant_from_vehicle
from .tuning_campaign import TuningCampaign

ROOT = model_resource_root()
TABLE_ROOT = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables"

X8_SOURCE_PROBLEM = ROOT / "examples/generated/vehicles/skywalker_x8_table_coordinate_trim_6dof.prb"
X8_SOURCE_TABLES = tuple(
    TABLE_ROOT / name
    for name in (
        "skywalker_x8_static_6axis.tbl",
        "skywalker_x8_collective_elevon_6axis.tbl",
        "skywalker_x8_differential_elevon_6axis.tbl",
        "skywalker_x8_thrust.tbl",
    )
)

_X8_COUPLED_LATERAL_STATE_NAMES = (
    "roll_error_rad",
    "yaw_error_rad",
    "v_m_s",
    "p_rad_s",
    "r_rad_s",
)
_X8_COUPLED_LATERAL_MAXIMUM_CONDITION = 1.0e3

B747_SOURCE_PROBLEM = ROOT / "examples/generated/vehicles/b747_condition3_surface_trim_6dof.prb"
B747_SOURCE_TABLES = tuple(
    TABLE_ROOT / name
    for name in (
        "b747_nominal_static_6axis.tbl",
        "b747_nominal_elevator_6axis.tbl",
        "b747_nominal_aileron_6axis.tbl",
        "b747_nominal_rudder_6axis.tbl",
        "b747_jt9d_thrust.tbl",
    )
)


def build_x8_source_table_plant() -> RuntimeRigidBodyLocalPlant:
    """Build the pinned X8 source-table local plant.

    Controls remain the source collective/differential elevon coordinates.
    This is not an assertion about installed left/right servo wiring or a
    full-flight controller; those boundaries stay in the X8 evidence record.
    """

    program = LoadedProgram.load(X8_SOURCE_PROBLEM, X8_SOURCE_TABLES, profile=GrammarProfile.TAORYX)
    return local_rigid_body_plant_from_vehicle(
        "skywalker-x8-table-coordinate-plant",
        "source-trim-local-v1",
        program.case().vehicles["1"],
        inertia_kg_m2=Vector3(0.325, 0.140, 0.400),
        reference_length_m=0.36,
        effector_limits={
            "collective-elevon-deg": EffectorLimits(
                "collective-elevon-deg",
                -20.0,
                20.0,
                "deg",
                rate_limit_per_s=120.0,
                time_constant_s=0.05,
            ),
            "differential-elevon-deg": EffectorLimits(
                "differential-elevon-deg",
                -20.0,
                20.0,
                "deg",
                rate_limit_per_s=120.0,
                time_constant_s=0.05,
            ),
            "throttle": EffectorLimits(
                "throttle",
                0.0,
                1.0,
                "fraction",
                time_constant_s=0.2,
            ),
        },
        effectiveness_steps={
            "collective-elevon-deg": 0.1,
            "differential-elevon-deg": 0.1,
            "throttle": 0.005,
        },
        allocation_wrench_weights={
            "moment_x_nm": 1.0,
            "moment_y_nm": 1.0,
            "moment_z_nm": 0.0,
        },
        external_moment_environment_keys={
            "moment_y_nm": "external_pitch_moment_bias_nm",
        },
    )
    ####


@lru_cache(maxsize=1)
def assess_x8_source_coupled_lateral_authority() -> AuthorityPreflightReport:
    """Assess X8's real, coupled lateral source-coordinate authority.

    The X8 has no independently allocatable yaw-moment channel: differential
    elevon produces both roll and a smaller yaw moment.  This report therefore
    tests whether the *actual one-coordinate coupled subsystem* can influence
    the local roll/yaw/sideslip-rate state vector.  It is a linear authority
    diagnostic for follow-on controller work, not a nonlinear recovery or
    racetrack qualification.
    """

    plant = build_x8_source_table_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    if not trim.success:
        raise RuntimeError(f"X8 coupled-lateral authority trim did not converge: {trim.as_dict()}")
    linearization = plant.linearize(
        trim,
        {
            "state_step": 1.0e-4,
            "control_step": 1.0e-3,
            "comparison_factor": 0.5,
            "maximum_relative_difference": 0.10,
            "comparison_absolute_floor": 1.0e-8,
        },
    )
    projection = project_linearization_to_wrench(
        linearization,
        plant.effectiveness(trim.state, trim.controls),
        state_names=_X8_COUPLED_LATERAL_STATE_NAMES,
        wrench_names=("moment_x_nm",),
        effector_names=("differential-elevon-deg",),
    )
    return linear_authority_preflight(
        LinearAuthorityRequirement(
            "x8-source-coupled-lateral-authority",
            _X8_COUPLED_LATERAL_STATE_NAMES,
            maximum_controllability_condition=_X8_COUPLED_LATERAL_MAXIMUM_CONDITION,
        ),
        state_names=projection.state_names,
        a_matrix=projection.a_matrix,
        b_matrix=projection.b_matrix,
    )
    ####


def build_b747_condition3_source_table_plant() -> RuntimeRigidBodyLocalPlant:
    """Build the pinned B747 NASA CR-2144 condition-3 source-table plant.

    The public source does not provide servo dynamics, so the declared
    effector model remains ideal rather than inventing rate or lag behavior.
    """

    program = LoadedProgram.load(B747_SOURCE_PROBLEM, B747_SOURCE_TABLES, profile=GrammarProfile.TAORYX)
    limits = {
        "elevator-deg": EffectorLimits("elevator-deg", -10.0, 10.0, "deg"),
        "aileron-deg": EffectorLimits("aileron-deg", -10.0, 10.0, "deg"),
        "rudder-deg": EffectorLimits("rudder-deg", -15.0, 15.0, "deg"),
        "throttle": EffectorLimits("throttle", 0.0, 1.0, "fraction"),
    }
    return local_rigid_body_plant_from_vehicle(
        "b747-condition3-source-table-plant",
        "nasa-cr-2144-condition3-local-v1",
        program.case().vehicles["1"],
        inertia_kg_m2=Vector3(24_675_886.7, 44_877_574.1, 67_384_152.0),
        reference_length_m=8.324088,
        effector_limits=limits,
        effectiveness_steps={
            "elevator-deg": 0.1,
            "aileron-deg": 0.1,
            "rudder-deg": 0.1,
            "throttle": 0.005,
        },
        allocation_wrench_weights={
            "moment_x_nm": 1.0,
            "moment_y_nm": 1.0,
            "moment_z_nm": 1.0,
        },
        allocation_regularization=1.0e-14,
        allocation_feasibility_tolerance=1.0e-3,
        external_moment_environment_keys={
            "moment_y_nm": "external_pitch_moment_bias_nm",
        },
    )
    ####


def _b747_physical_wrench_scales(
    effectiveness: EffectorEffectiveness,
    plant: RuntimeRigidBodyLocalPlant,
) -> tuple[float, ...]:
    """Derive the retained condition-3 LQR wrench scales from source travel."""

    surface_names = ("elevator-deg", "aileron-deg", "rudder-deg")
    scales: list[float] = []
    for row in range(3):
        authority = 0.0
        for column, effector_name in enumerate(effectiveness.effector_names):
            if effector_name not in surface_names:
                continue
            limits = plant.effector_limits[effector_name]
            reference = float(effectiveness.reference_effectors[effector_name])
            travel = min(max(0.0, limits.upper - reference), max(0.0, reference - limits.lower))
            authority += abs(float(effectiveness.array[row, column])) * travel
        if authority <= 0.0:
            raise RuntimeError(f"B747 condition-3 source trim has no local moment authority on row {row}")
        scales.append(max(authority * 0.25, 1.0))
    return tuple(scales)
    ####


@lru_cache(maxsize=1)
def build_b747_condition3_source_surface_physical_lqr_design() -> PhysicalWrenchLqrDesign:
    """Build the LQR selected by the checked-in B747 condition-3 screen.

    This is the exact aggressive profile which passed the existing source-table
    nonlinear evidence packet.  It is cached only across synthesis consumers;
    every execution still creates a fresh nonlinear plant and runs its bounded
    source-table allocator.  The result is one local NASA CR-2144 condition-3
    controller, not a B747 gain schedule or transport mission controller.
    """

    plant = build_b747_condition3_source_table_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    if not trim.success:
        raise RuntimeError(f"B747 condition-3 physical trim did not converge: {trim.as_dict()}")
    linearization = plant.linearize(
        trim,
        {
            "state_step": 1.0e-4,
            "control_step": 1.0e-3,
            "comparison_factor": 0.5,
            "maximum_relative_difference": 0.10,
            "comparison_absolute_floor": 1.0e-7,
        },
    )
    effectiveness = plant.effectiveness(trim.state, trim.controls)
    state_names = (
        "roll_error_rad",
        "pitch_error_rad",
        "yaw_error_rad",
        "u_m_s",
        "v_m_s",
        "w_m_s",
        "p_rad_s",
        "q_rad_s",
        "r_rad_s",
    )
    projection = project_linearization_to_wrench(
        linearization,
        effectiveness,
        state_names=state_names,
        wrench_names=("moment_x_nm", "moment_y_nm", "moment_z_nm"),
        effector_names=("elevator-deg", "aileron-deg", "rudder-deg"),
    )
    authority = linear_authority_preflight(
        LinearAuthorityRequirement("b747-condition3-three-axis-surface-authority", state_names),
        state_names=projection.state_names,
        a_matrix=projection.a_matrix,
        b_matrix=projection.b_matrix,
    )
    if authority.status != "passed":
        raise RuntimeError(f"B747 physical-surface authority preflight blocked LQR synthesis: {authority.as_dict()}")
    return design_physical_wrench_lqr(
        "b747-condition3-aggressive-surface-wrench-lqr-v1",
        projection,
        q_diagonal=(4.0, 4.0, 4.0, 0.50, 4.0, 4.0, 1.0, 1.0, 1.0),
        r_diagonal=(0.25, 0.25, 0.25),
        state_scales=(
            math.radians(3.0),
            math.radians(3.0),
            math.radians(3.0),
            5.0,
            3.0,
            3.0,
            math.radians(9.0),
            math.radians(9.0),
            math.radians(9.0),
        ),
        wrench_scales=_b747_physical_wrench_scales(effectiveness, plant),
    )
    ####


@lru_cache(maxsize=1)
def build_b747_condition3_source_surface_physical_lqi_design() -> PhysicalWrenchLqiDesign:
    """Build the bounded-allocation B747 condition-3 LQI candidate.

    This shares the source-derived state, wrench, and engineering scales of
    the exercised LQR screen, then adds attitude-error integrators.  The
    retained integral weight passes the same deterministic local recovery and
    allocation gates plus the bounded matched pitch-moment offset screen. It
    remains a source-local result, not B747 wind or mass robustness.
    """

    lqr = build_b747_condition3_source_surface_physical_lqr_design()
    return design_physical_wrench_lqi(
        "b747-condition3-source-surface-wrench-lqi-v1",
        lqr.projection,
        output_names=("roll_error_rad", "pitch_error_rad", "yaw_error_rad"),
        q_diagonal=lqr.q_diagonal,
        r_diagonal=lqr.r_diagonal,
        integral_q_diagonal=(0.025, 0.025, 0.025),
        state_scales=lqr.state_scales,
        wrench_scales=lqr.wrench_scales,
    )
    ####


@lru_cache(maxsize=1)
def build_x8_source_surface_physical_lqr_design() -> PhysicalWrenchLqrDesign:
    """Synthesize the pinned X8 source-coordinate physical LQR once.

    The design deliberately controls only roll and pitch moment through the
    source collective/differential-elevon coordinates.  Every caller still
    runs the nonlinear plant and its bounded allocator; caching only avoids
    repeating deterministic trim and derivative synthesis while a process
    serves discovery, preflight, and execution requests.
    """

    plant = build_x8_source_table_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    if not trim.success:
        raise RuntimeError(f"source table-coordinate trim did not converge: {trim.as_dict()}")
    linearization = plant.linearize(
        trim,
        {
            "state_step": 1.0e-4,
            "control_step": 1.0e-3,
            "comparison_factor": 0.5,
            "maximum_relative_difference": 0.10,
            "comparison_absolute_floor": 1.0e-8,
        },
    )
    effectiveness = plant.effectiveness(trim.state, trim.controls)
    projection = project_linearization_to_wrench(
        linearization,
        effectiveness,
        state_names=("roll_error_rad", "pitch_error_rad", "p_rad_s", "q_rad_s"),
        wrench_names=("moment_x_nm", "moment_y_nm"),
        effector_names=("differential-elevon-deg", "collective-elevon-deg"),
    )
    authority = linear_authority_preflight(
        LinearAuthorityRequirement(
            "x8-roll-pitch-source-coordinate-authority",
            ("roll_error_rad", "pitch_error_rad", "p_rad_s", "q_rad_s"),
        ),
        state_names=projection.state_names,
        a_matrix=projection.a_matrix,
        b_matrix=projection.b_matrix,
    )
    if authority.status != "passed":
        raise RuntimeError(
            "X8 source-coordinate authority preflight blocked roll/pitch LQR synthesis: "
            f"{authority.as_dict()}"
        )
    return design_physical_wrench_lqr(
        "skywalker-x8-source-trim-roll-pitch-wrench-lqr-v1",
        projection,
        q_diagonal=(16.0, 16.0, 3.0, 3.0),
        r_diagonal=(1.0, 1.0),
        state_scales=(math.radians(10.0), math.radians(10.0), math.radians(45.0), math.radians(45.0)),
        wrench_scales=(0.20, 0.20),
    )
    ####


@lru_cache(maxsize=1)
def build_x8_source_surface_physical_lqi_design() -> PhysicalWrenchLqiDesign:
    """Build the bounded-allocation X8 local roll/pitch LQI candidate.

    The design retains the exact source-derived roll/pitch wrench projection,
    engineering scales, source-coordinate elevon limits, and actuator dynamics
    of the exercised LQR path.  It adds only roll and pitch-error integrators:
    the coupled differential-elevon lateral diagnostic does not establish an
    independently allocatable yaw-moment axis, so yaw is deliberately absent
    from this offset-free candidate.
    """

    lqr = build_x8_source_surface_physical_lqr_design()
    return design_physical_wrench_lqi(
        "skywalker-x8-source-trim-roll-pitch-wrench-lqi-v1",
        lqr.projection,
        output_names=("roll_error_rad", "pitch_error_rad"),
        q_diagonal=lqr.q_diagonal,
        r_diagonal=lqr.r_diagonal,
        integral_q_diagonal=(0.15, 0.15),
        state_scales=lqr.state_scales,
        wrench_scales=lqr.wrench_scales,
    )
    ####


def build_x8_source_surface_lqi_tuning_campaign() -> TuningCampaign:
    """Declare the X8 source-surface local LQI design screen.

    The campaign uses the *same* source-derived roll/pitch state-to-wrench
    projection as the allocator-backed nonlinear screen.  It intentionally
    does not tune the broader source-table state/elevon model, because those
    coordinates cannot be applied as an exact runtime gain by the bounded
    physical-wrench controller.  This remains a local source-table candidate,
    not a racetrack or physical-servo qualification.
    """

    design = build_x8_source_surface_physical_lqi_design()
    return ControlAutomationDeclaration(
        id="x8-source-surface-roll-pitch-wrench-local",
        campaign_id="x8-source-surface-local-lqi-v1",
        family_id="skywalker_x8",
        tier="rigid_body_6dof_surface_allocated",
        strategy_id="powered_fixed_wing.v1",
        node_id="source-trim-local",
        state_scales=dict(zip(design.projection.state_names, design.state_scales, strict=True)),
        control_scales=dict(zip(design.projection.wrench_names, design.wrench_scales, strict=True)),
        authority_state_names=design.projection.state_names,
        offset_free_outputs=design.result.output_names,
        profile_grid_id_prefix="x8-source-surface-local",
        state_weight_multipliers=(1.0,),
        control_effort_multipliers=(1.0,),
        state_base_weights=design.q_diagonal,
        control_base_weights=design.r_diagonal,
        integral_weight_multiplier=design.integral_q_diagonal[0],
        integral_weight_multipliers=(1.0,),
    ).build_campaign()
    ####


def build_b747_source_surface_lqi_tuning_campaign() -> TuningCampaign:
    """Declare the exact B747 condition-3 physical-wrench LQI runtime.

    Automatic tuning operates on the same retained local state-to-moment
    projection as the nonlinear screen.  The screen still allocates each
    request to its bounded elevator, aileron, rudder, and throttle coordinates;
    the campaign neither synthesizes raw effector gains nor replaces that
    physical execution evidence.
    """

    design = build_b747_condition3_source_surface_physical_lqi_design()
    return ControlAutomationDeclaration(
        id="b747-condition3-source-wrench-local",
        campaign_id="b747-source-surface-local-lqi-v1",
        family_id="b747",
        tier="rigid_body_6dof_surface_allocated",
        strategy_id="powered_fixed_wing.v1",
        node_id="condition3-source-wrench-local",
        state_scales=dict(zip(design.projection.state_names, design.state_scales, strict=True)),
        control_scales=dict(zip(design.projection.wrench_names, design.wrench_scales, strict=True)),
        authority_state_names=design.projection.state_names,
        offset_free_outputs=design.result.output_names,
        profile_grid_id_prefix="b747-condition3-source-wrench",
        state_weight_multipliers=(1.0,),
        control_effort_multipliers=(1.0,),
        state_base_weights=design.q_diagonal,
        control_base_weights=design.r_diagonal,
        integral_base_weights=design.integral_q_diagonal,
        integral_weight_multipliers=(0.1, 1.0, 10.0, 100.0),
    ).build_campaign()
    ####


__all__ = [
    "B747_SOURCE_PROBLEM",
    "B747_SOURCE_TABLES",
    "X8_SOURCE_PROBLEM",
    "X8_SOURCE_TABLES",
    "build_b747_condition3_source_table_plant",
    "build_b747_condition3_source_surface_physical_lqi_design",
    "build_b747_condition3_source_surface_physical_lqr_design",
    "build_b747_source_surface_lqi_tuning_campaign",
    "build_x8_source_surface_physical_lqi_design",
    "build_x8_source_surface_physical_lqr_design",
    "build_x8_source_table_plant",
    "build_x8_source_surface_lqi_tuning_campaign",
]
