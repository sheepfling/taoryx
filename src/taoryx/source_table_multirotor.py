"""Runtime-owned source-table plants for multirotor local witnesses.

The Hummingbird builder is intentionally the exact local RotorPy-derived
plant used by the physical-control validation packet.  It does not infer a
generic quadcopter from the ``multirotor`` family name and it does not make a
hover witness into a waypoint, battery, or envelope qualification.
"""

from __future__ import annotations

from pathlib import Path

from .contracts import Vector3
from .control_allocation import EffectorLimits
from .language.grammar_contracts import GrammarProfile
from .runtime.program import LoadedProgram
from .runtime_control_adapter import RuntimeRigidBodyLocalPlant, local_rigid_body_plant_from_vehicle

ROOT = Path(__file__).resolve().parents[2]
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
]
