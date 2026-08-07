"""Runtime-owned source-table plants for fixed-wing local-control witnesses.

These builders deliberately preserve the exact source problem, table set,
inertia, limits, and local authority settings used by the qualification
fixtures.  They do not infer a vehicle from its physical family and they do
not create a generic fixed-wing substitute.  Keeping the construction here
lets Mission Composition lower a selected X8 or B747 composition through the same
source-owned plant that developer validation uses.
"""

from __future__ import annotations

from pathlib import Path

from .contracts import Vector3
from .control_allocation import EffectorLimits
from .language.grammar_contracts import GrammarProfile
from .runtime.program import LoadedProgram
from .runtime_control_adapter import RuntimeRigidBodyLocalPlant, local_rigid_body_plant_from_vehicle

ROOT = Path(__file__).resolve().parents[2]
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
    )
    ####


__all__ = [
    "B747_SOURCE_PROBLEM",
    "B747_SOURCE_TABLES",
    "X8_SOURCE_PROBLEM",
    "X8_SOURCE_TABLES",
    "build_b747_condition3_source_table_plant",
    "build_x8_source_table_plant",
]
