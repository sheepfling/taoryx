"""Low-fidelity rocket-release and glide scenario for the HL-20 family."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

from .contracts import Vector3
from .hl20_showcase import render_hl20_ca_hi_showcase_composites
from .reachability_aerodynamics import HL20_SOURCE_DOCUMENT_SHA256, HL20_SOURCE_MODEL_ID
from .reachability_envelope import (
    ImpulseFrame,
    LaunchCommand,
    ReachabilityEnvelope,
    ReachabilityFidelity,
    RocketGlideVehicle,
    TerminalCriteria,
    run_reachability_envelope,
)
from .reachability_visualization import render_reachability_plot_bundle
from .vehicle import DetachedBodyDefinition

HL20_REFERENCE_AREA_M2 = 26.612075808
HL20_FIXED_MASS_KG = 8_664.1
HL20_REFERENCE_CHORD_M = 8.607552
HL20_REFERENCE_INERTIA_KG_M2 = Vector3(
    0.40 * HL20_FIXED_MASS_KG * 3.0**2,
    0.25 * HL20_FIXED_MASS_KG * HL20_REFERENCE_CHORD_M**2,
    0.25 * HL20_FIXED_MASS_KG * HL20_REFERENCE_CHORD_M**2,
)
HL20_PACKAGE_SHA256 = "443e2ed905e310cc57014dc3bad8d3b24b6b5954b8423de9110d405d66154e63"
HL20_AERODYNAMICS_SHA256 = "b2ec6260ed60d241de250599b269ad35d0b96865b50da5e7f9ef7e04de3844ec"
HL20_CAHI_TARGET_DISTANCE_M = 3_920_000.0
HL20_ENERGY_MANAGED_BANK_SCHEDULE_DEG = (
    (25.0, 0.0),
    (30.0, 0.0),
    (35.0, 30.0),
    (45.0, -30.0),
    (55.0, 0.0),
)
HL20_ENERGY_MANAGED_SEGMENTS = (
    ("stabilization", 25.0, 30.0),
    ("glide_trim", 30.0, 35.0),
    ("right_bank", 35.0, 45.0),
    ("left_bank", 45.0, 55.0),
    ("energy_descent", 55.0, 120.0),
)


@dataclass(frozen=True, slots=True)
class HL20LowFidelityBundle:
    """One reproducible point-mass release artifact and its manifest."""

    envelope: ReachabilityEnvelope
    artifact_path: Path
    manifest_path: Path
    ####


HL20_FIDELITIES = (
    ReachabilityFidelity.POINT_MASS_3DOF,
    ReachabilityFidelity.PSEUDO_6DOF,
    ReachabilityFidelity.RIGID_BODY_6DOF,
    ReachabilityFidelity.RIGID_BODY_6DOF_SURFACE_ALLOCATED,
)


def hl20_release_vehicle(
    *,
    aerodynamic_model_id: str = "surrogate_fixed_cd_ld_v1",
    attitude_control_mode: str = "open_loop",
    actuator_profile_id: str = "none",
    mission_profile_id: str = "none",
    glide_bank_schedule_deg: tuple[tuple[float, float], ...] = (),
    mission_segment_schedule: tuple[tuple[str, float, float], ...] = (),
    initial_speed_m_s: float = 250.0,
    attitude_control_gain: float = 500_000.0,
    attitude_rate_damping: float = 300_000.0,
    configuration_variant_id: str = "nominal",
    mass_property_profile_id: str = "generic_slender_body_v1",
    inertia_body_kg_m2: Vector3 | None = None,
    wind_velocity_m_s: Vector3 = Vector3(0.0, 0.0, 0.0),
    dry_mass_kg: float = HL20_FIXED_MASS_KG,
    booster_thrust_n: float = 120_000.0,
    booster_burn_time_s: float = 15.0,
    booster_release_time_s: float = 25.0,
    booster_dry_mass_kg: float = 1_500.0,
    booster_propellant_mass_kg: float = 1_500.0,
    booster_separation_impulse_n_s: Vector3 = Vector3(0.0, 0.0, 0.0),
    booster_separation_impulse_frame: ImpulseFrame = ImpulseFrame.BODY,
) -> RocketGlideVehicle:
    """Build the synthetic booster plus source-bound HL-20 glide child.

    The child mass and reference area come from the pinned HL-20 package. The
    default remains the synthetic baseline; pass ``HL20_SOURCE_MODEL_ID`` to
    run the source force/moment graph through the same reachability seam.
    """

    spent_booster = DetachedBodyDefinition.cylinder(
        "hl20-synthetic-spent-booster",
        mass_kg=booster_dry_mass_kg,
        radius_m=0.8,
        length_m=6.0,
        inertia_kg_m2=Vector3(
            5_000.0 * booster_dry_mass_kg / 1_500.0,
            5_000.0 * booster_dry_mass_kg / 1_500.0,
            500.0 * booster_dry_mass_kg / 1_500.0,
        ),
    )
    return RocketGlideVehicle(
        vehicle_id="hl20-low-fidelity-release-v1",
        dry_mass_kg=dry_mass_kg,
        propellant_mass_kg=0.0,
        thrust_n=0.0,
        burn_time_s=0.0,
        reference_area_m2=HL20_REFERENCE_AREA_M2,
        drag_coefficient=0.06,
        lift_to_drag=3.2,
        aerodynamic_model_id=aerodynamic_model_id,
        actuator_profile_id=actuator_profile_id,
        attitude_control_mode=attitude_control_mode,
        mission_profile_id=mission_profile_id,
        glide_bank_schedule_deg=glide_bank_schedule_deg,
        mission_segment_schedule=mission_segment_schedule,
        initial_speed_m_s=initial_speed_m_s,
        attitude_control_gain=attitude_control_gain,
        attitude_rate_damping=attitude_rate_damping,
        configuration_variant_id=configuration_variant_id,
        mass_property_profile_id=mass_property_profile_id,
        inertia_body_kg_m2=inertia_body_kg_m2,
        wind_velocity_m_s=wind_velocity_m_s,
        initial_altitude_m=0.0,
        booster_dry_mass_kg=booster_dry_mass_kg,
        booster_propellant_mass_kg=booster_propellant_mass_kg,
        booster_thrust_n=booster_thrust_n,
        booster_burn_time_s=booster_burn_time_s,
        booster_release_time_s=booster_release_time_s,
        booster_detached_body=spent_booster,
        booster_separation_impulse_n_s=booster_separation_impulse_n_s,
        booster_separation_impulse_frame=booster_separation_impulse_frame,
    )
    ####


def hl20_release_commands() -> tuple[LaunchCommand, ...]:
    """Return the small deterministic first-tranche launch grid."""

    return tuple(
        LaunchCommand(azimuth_rad=0.0, elevation_rad=math.radians(elevation))
        for elevation in (60.0, 70.0, 80.0)
    )
    ####


def hl20_low_fidelity_provenance(
    fidelity: ReachabilityFidelity = ReachabilityFidelity.POINT_MASS_3DOF,
    *,
    aerodynamic_model_id: str = "surrogate_fixed_cd_ld_v1",
    attitude_control_mode: str = "open_loop",
    actuator_profile_id: str = "none",
    mission_profile_id: str = "none",
    glide_bank_schedule_deg: tuple[tuple[float, float], ...] = (),
    mission_segment_schedule: tuple[tuple[str, float, float], ...] = (),
    initial_speed_m_s: float = 250.0,
    attitude_control_gain: float = 500_000.0,
    attitude_rate_damping: float = 300_000.0,
    configuration_variant_id: str = "nominal",
    mass_property_profile_id: str = "generic_slender_body_v1",
    inertia_body_kg_m2: Vector3 | None = None,
    wind_velocity_m_s: Vector3 = Vector3(0.0, 0.0, 0.0),
) -> dict[str, object]:
    """Describe the source boundary and synthetic launch-parent assumptions."""

    if fidelity not in HL20_FIDELITIES:
        raise ValueError(f"unsupported HL-20 fidelity: {fidelity}")

    return {
        "family_group": "hypersonic_lifting_body_research",
        "family_id": "reference_hl20_mod_k",
        "scenario_id": f"hl20-rocket-release-{fidelity.value}-v1",
        "qualification": "synthetic_booster_source_bound_glide_child",
        "source_package_sha256": HL20_PACKAGE_SHA256,
        "aerodynamics_sha256": HL20_AERODYNAMICS_SHA256,
        "source_family_manifest": "families/reference_hl20_mod_k/family.yaml",
        "child_model": "reference_hl20_mod_k",
        "rocket_parent": "generic-staged-booster-v1",
        "rocket_parent_claim": "synthetic scenario assumption only",
        "aerodynamic_model": aerodynamic_model_id,
        "attitude_control_mode": attitude_control_mode,
        "actuator_profile_id": actuator_profile_id,
        "mission_profile_id": mission_profile_id,
        "glide_bank_schedule_deg": [list(item) for item in glide_bank_schedule_deg],
        "mission_segment_schedule": [list(item) for item in mission_segment_schedule],
        "initial_speed_m_s": initial_speed_m_s,
        "attitude_control_gain": attitude_control_gain,
        "attitude_rate_damping": attitude_rate_damping,
        "configuration_variant_id": configuration_variant_id,
        "mass_property_profile_id": mass_property_profile_id,
        "inertia_body_kg_m2": (
            None
            if inertia_body_kg_m2 is None
            else [inertia_body_kg_m2.x, inertia_body_kg_m2.y, inertia_body_kg_m2.z]
        ),
        "wind_velocity_m_s": [wind_velocity_m_s.x, wind_velocity_m_s.y, wind_velocity_m_s.z],
        "child_aerodynamics": (
            "pinned_daveml_force_moment_graph"
            if aerodynamic_model_id == HL20_SOURCE_MODEL_ID
            else "fixed_cd_and_ld_low_fidelity_surrogate"
        ),
        "child_mass_binding": "separate_fixed_binding_v1",
        "controller_claim": False,
        "route_claim": False,
        "source_exact_trajectory": False,
        "source_aerodynamics": aerodynamic_model_id == HL20_SOURCE_MODEL_ID,
        "source_document_sha256": HL20_SOURCE_DOCUMENT_SHA256,
        "fidelity": fidelity.value,
        "release_event_id": "hl20-release-v1",
        "release_time_s": 25.0,
        "source_envelope": {"mach": [0.3, 4.0], "altitude_m": [-1000.0, 20_000.0]},
    }
    ####


def run_hl20_release(
    *,
    commands: tuple[LaunchCommand, ...] | None = None,
    fidelity: ReachabilityFidelity = ReachabilityFidelity.POINT_MASS_3DOF,
    step_size_s: float = 0.5,
    horizon_s: float = 120.0,
    spawn_children: bool = True,
    criteria: TerminalCriteria | None = None,
    aerodynamic_model_id: str = "surrogate_fixed_cd_ld_v1",
    attitude_control_mode: str = "open_loop",
    actuator_profile_id: str = "none",
    mission_profile_id: str = "none",
    glide_bank_schedule_deg: tuple[tuple[float, float], ...] = (),
    mission_segment_schedule: tuple[tuple[str, float, float], ...] = (),
    initial_speed_m_s: float = 250.0,
    attitude_control_gain: float = 500_000.0,
    attitude_rate_damping: float = 300_000.0,
    configuration_variant_id: str = "nominal",
    mass_property_profile_id: str = "generic_slender_body_v1",
    inertia_body_kg_m2: Vector3 | None = None,
    wind_velocity_m_s: Vector3 = Vector3(0.0, 0.0, 0.0),
) -> ReachabilityEnvelope:
    """Run one fidelity tier of the HL-20 rocket-release envelope."""

    if fidelity not in HL20_FIDELITIES:
        raise ValueError(f"unsupported HL-20 fidelity: {fidelity}")

    return run_reachability_envelope(
        hl20_release_vehicle(
            aerodynamic_model_id=aerodynamic_model_id,
            attitude_control_mode=attitude_control_mode,
            actuator_profile_id=actuator_profile_id,
            mission_profile_id=mission_profile_id,
            glide_bank_schedule_deg=glide_bank_schedule_deg,
            mission_segment_schedule=mission_segment_schedule,
            initial_speed_m_s=initial_speed_m_s,
            attitude_control_gain=attitude_control_gain,
            attitude_rate_damping=attitude_rate_damping,
            configuration_variant_id=configuration_variant_id,
            mass_property_profile_id=mass_property_profile_id,
            inertia_body_kg_m2=inertia_body_kg_m2,
            wind_velocity_m_s=wind_velocity_m_s,
        ),
        commands or hl20_release_commands(),
        fidelity=fidelity,
        step_size_s=step_size_s,
        horizon_s=horizon_s,
        workers=1,
        study_id=f"hl20_rocket_release_{aerodynamic_model_id}_{fidelity.value}_v1",
        provenance=hl20_low_fidelity_provenance(
            fidelity,
            aerodynamic_model_id=aerodynamic_model_id,
            attitude_control_mode=attitude_control_mode,
            actuator_profile_id=actuator_profile_id,
            mission_profile_id=mission_profile_id,
            glide_bank_schedule_deg=glide_bank_schedule_deg,
            mission_segment_schedule=mission_segment_schedule,
            initial_speed_m_s=initial_speed_m_s,
            attitude_control_gain=attitude_control_gain,
            attitude_rate_damping=attitude_rate_damping,
            configuration_variant_id=configuration_variant_id,
            mass_property_profile_id=mass_property_profile_id,
            inertia_body_kg_m2=inertia_body_kg_m2,
            wind_velocity_m_s=wind_velocity_m_s,
        ),
        spawn_children=spawn_children,
        criteria=criteria or TerminalCriteria(require_ground_contact=True),
    )
    ####


def run_hl20_low_fidelity_release(
    *,
    commands: tuple[LaunchCommand, ...] | None = None,
    step_size_s: float = 0.5,
    horizon_s: float = 120.0,
    spawn_children: bool = True,
) -> ReachabilityEnvelope:
    """Compatibility wrapper for the first point-mass HL-20 tier."""

    return run_hl20_release(
        commands=commands,
        fidelity=ReachabilityFidelity.POINT_MASS_3DOF,
        step_size_s=step_size_s,
        horizon_s=horizon_s,
        spawn_children=spawn_children,
    )
    ####


def run_hl20_fidelity_ladder(
    *,
    commands: tuple[LaunchCommand, ...] | None = None,
    step_size_s: float = 0.5,
    horizon_s: float = 120.0,
    spawn_children: bool = True,
) -> tuple[ReachabilityEnvelope, ...]:
    """Run all four HL-20 rocket-release fidelity tiers over one search grid."""

    return tuple(
        run_hl20_release(
            commands=commands,
            fidelity=fidelity,
            step_size_s=step_size_s,
            horizon_s=horizon_s,
            spawn_children=spawn_children,
        )
        for fidelity in HL20_FIDELITIES
    )
    ####


def hl20_ca_hi_terminal_criteria() -> TerminalCriteria:
    """Return the explicit local-tangent CA-HI impact contract."""

    return TerminalCriteria(
        require_ground_contact=True,
        target_position_m=(HL20_CAHI_TARGET_DISTANCE_M, 0.0, 0.0),
        max_impact_radius_m=40_000.0,
        min_impact_speed_m_s=1_000.0,
        max_impact_speed_m_s=3_000.0,
        target_id="honolulu_terminal_reference",
        target_frame="local_ecic_tangent_from_california_origin",
    )


def hl20_source_release_commands(fidelity: ReachabilityFidelity) -> tuple[LaunchCommand, ...]:
    """Return source-ladder commands over the common launch search."""

    return hl20_release_commands()


def hl20_source_release_vehicle(
    *,
    configuration_variant_id: str = "hl20_source_nominal_v1",
    wind_velocity_m_s: Vector3 = Vector3(0.0, 0.0, 0.0),
    dry_mass_kg: float = HL20_FIXED_MASS_KG,
    booster_thrust_n: float = 120_000.0,
    booster_burn_time_s: float = 15.0,
    booster_release_time_s: float = 25.0,
    inertia_body_kg_m2: Vector3 = HL20_REFERENCE_INERTIA_KG_M2,
    booster_dry_mass_kg: float = 1_500.0,
    booster_propellant_mass_kg: float = 1_500.0,
    booster_separation_impulse_n_s: Vector3 = Vector3(0.0, 0.0, 0.0),
    booster_separation_impulse_frame: ImpulseFrame = ImpulseFrame.BODY,
) -> RocketGlideVehicle:
    """Build the replayable source-bound HL-20 configuration.

    Variant inputs are explicit so robustness studies can vary launch energy,
    booster timing, wind, mass, or inertia without changing the source model
    identity or silently changing the claim boundary.
    """

    return hl20_release_vehicle(
        aerodynamic_model_id=HL20_SOURCE_MODEL_ID,
        attitude_control_mode="velocity_aligned",
        actuator_profile_id="hl20.reference_first_order.v1",
        mission_profile_id="hl20.energy_managed_entry_glide.v1",
        glide_bank_schedule_deg=HL20_ENERGY_MANAGED_BANK_SCHEDULE_DEG,
        mission_segment_schedule=HL20_ENERGY_MANAGED_SEGMENTS,
        initial_speed_m_s=350.0,
        attitude_control_gain=2_000_000.0,
        attitude_rate_damping=800_000.0,
        configuration_variant_id=configuration_variant_id,
        mass_property_profile_id="hl20_mod_k_fixed_mass_inertia_v1",
        inertia_body_kg_m2=inertia_body_kg_m2,
        wind_velocity_m_s=wind_velocity_m_s,
        dry_mass_kg=dry_mass_kg,
        booster_thrust_n=booster_thrust_n,
        booster_burn_time_s=booster_burn_time_s,
        booster_release_time_s=booster_release_time_s,
        booster_dry_mass_kg=booster_dry_mass_kg,
        booster_propellant_mass_kg=booster_propellant_mass_kg,
        booster_separation_impulse_n_s=booster_separation_impulse_n_s,
        booster_separation_impulse_frame=booster_separation_impulse_frame,
    )


def run_hl20_source_release(
    *,
    commands: tuple[LaunchCommand, ...] | None = None,
    fidelity: ReachabilityFidelity = ReachabilityFidelity.POINT_MASS_3DOF,
    step_size_s: float = 0.5,
    horizon_s: float = 120.0,
    spawn_children: bool = True,
    criteria: TerminalCriteria | None = None,
) -> ReachabilityEnvelope:
    """Run one HL-20 tier using the pinned source aerodynamic graph."""

    return run_hl20_release(
        commands=commands or hl20_source_release_commands(fidelity),
        fidelity=fidelity,
        step_size_s=step_size_s,
        horizon_s=horizon_s,
        spawn_children=spawn_children,
        criteria=criteria,
        aerodynamic_model_id=HL20_SOURCE_MODEL_ID,
        attitude_control_mode="velocity_aligned",
        actuator_profile_id="hl20.reference_first_order.v1",
        mission_profile_id="hl20.energy_managed_entry_glide.v1",
        glide_bank_schedule_deg=HL20_ENERGY_MANAGED_BANK_SCHEDULE_DEG,
        mission_segment_schedule=HL20_ENERGY_MANAGED_SEGMENTS,
        initial_speed_m_s=350.0,
        attitude_control_gain=2_000_000.0,
        attitude_rate_damping=800_000.0,
        configuration_variant_id="hl20_source_nominal_v1",
        mass_property_profile_id="hl20_mod_k_fixed_mass_inertia_v1",
        inertia_body_kg_m2=HL20_REFERENCE_INERTIA_KG_M2,
    )


def run_hl20_source_fidelity_ladder(
    *,
    commands: tuple[LaunchCommand, ...] | None = None,
    step_size_s: float = 0.5,
    horizon_s: float = 120.0,
    spawn_children: bool = True,
    criteria: TerminalCriteria | None = None,
) -> tuple[ReachabilityEnvelope, ...]:
    """Run all four fidelity tiers against the pinned source aero graph."""

    return tuple(
        run_hl20_source_release(
            commands=commands,
            fidelity=fidelity,
            step_size_s=step_size_s,
            horizon_s=horizon_s,
            spawn_children=spawn_children,
            criteria=criteria,
        )
        for fidelity in HL20_FIDELITIES
    )
    ####


def write_hl20_low_fidelity_bundle(
    directory: str | Path,
    *,
    commands: tuple[LaunchCommand, ...] | None = None,
    step_size_s: float = 0.5,
    horizon_s: float = 120.0,
    spawn_children: bool = True,
) -> HL20LowFidelityBundle:
    """Write the point-mass envelope and a source/assumption manifest."""

    destination = Path(directory)
    destination.mkdir(parents=True, exist_ok=True)
    envelope = run_hl20_low_fidelity_release(
        commands=commands,
        step_size_s=step_size_s,
        horizon_s=horizon_s,
        spawn_children=spawn_children,
    )
    artifact_path = destination / "point_mass_3dof.json"
    envelope.write_json(artifact_path)
    manifest_path = destination / "bundle-manifest.json"
    manifest_path.write_text(
        json.dumps(
            {
                "schema": "taoryx.hl20-rocket-release-bundle/v1alpha1",
                "vehicle_id": "reference_hl20_mod_k",
                "family_group": "hypersonic_lifting_body_research",
                "fidelity": "point_mass_3dof",
                "claim_boundary": "low-fidelity synthetic booster with source-bound HL-20 geometry and fixed mass",
                "artifacts": [artifact_path.name],
                "provenance": hl20_low_fidelity_provenance(),
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return HL20LowFidelityBundle(envelope, artifact_path, manifest_path)
    ####


def write_hl20_fidelity_bundle(
    directory: str | Path,
    *,
    commands: tuple[LaunchCommand, ...] | None = None,
    step_size_s: float = 0.5,
    horizon_s: float = 120.0,
    spawn_children: bool = True,
    dpi: int = 140,
    source_bound: bool = False,
) -> dict[str, object]:
    """Write all four HL-20 tiers and one comparison manifest.

    ``source_bound`` selects the pinned DAVE-ML aerodynamic graph while
    retaining the synthetic booster and fixed-mass scenario assumptions.
    """

    destination = Path(directory)
    destination.mkdir(parents=True, exist_ok=True)
    envelopes = (
        run_hl20_source_fidelity_ladder(
            commands=commands,
            step_size_s=step_size_s,
            horizon_s=horizon_s,
            spawn_children=spawn_children,
        )
        if source_bound
        else run_hl20_fidelity_ladder(
            commands=commands,
            step_size_s=step_size_s,
            horizon_s=horizon_s,
            spawn_children=spawn_children,
        )
    )
    artifacts: list[str] = []
    for envelope in envelopes:
        artifact_path = destination / f"{envelope.fidelity.value}.json"
        envelope.write_json(artifact_path)
        artifacts.append(artifact_path.name)
    plot_report = render_reachability_plot_bundle(
        envelopes[0],
        destination / "plots",
        comparison_sources=envelopes[1:],
        dpi=dpi,
    )
    showcase_report = render_hl20_ca_hi_showcase_composites(
        envelopes,
        destination / "composites",
        source_artifacts=tuple(destination / artifact for artifact in artifacts),
        dpi=dpi,
    )
    manifest_path = destination / "bundle-manifest.json"
    manifest_path.write_text(
        json.dumps(
            {
                "schema": "taoryx.hl20-rocket-release-fidelity-bundle/v1alpha1",
                "vehicle_id": "reference_hl20_mod_k",
                "family_group": "hypersonic_lifting_body_research",
                "fidelity_profiles": [fidelity.value for fidelity in HL20_FIDELITIES],
                "artifacts": artifacts,
                "claim_boundary": (
                    "four fidelity source-bound DAVE-ML booster comparison with synthetic booster and fixed mass"
                    if source_bound
                    else "four fidelity synthetic booster comparison with source-bound HL-20 geometry and fixed mass"
                ),
                "tier_contracts": {
                    "point_mass_3dof": "translation and resource baseline",
                    "pseudo_6dof": "synthesized attitude response bridge",
                    "rigid_body_6dof": "native rigid-body diagnostic with direct loads",
                    "rigid_body_6dof_surface_allocated": (
                        "source graph plus bounded seven-surface actuator commands"
                        if source_bound
                        else "bounded synthetic logical seven-surface overlay"
                    ),
                },
                "surface_allocation_claim": (
                    "source actuator position/rate/saturation telemetry with open-loop direction probe; not closed-loop controller qualification"
                    if source_bound
                    else "logical overlay only; not source-declared actuator dynamics or controller qualification"
                ),
                "plots": [path.name for path in plot_report.plot_paths],
                "plot_manifest": str(plot_report.manifest_path.relative_to(destination)),
                "showcase": {
                    "composites": [path.relative_to(destination).as_posix() for path in showcase_report.composite_paths],
                    "data_artifacts": [path.relative_to(destination).as_posix() for path in showcase_report.data_paths],
                    "summary": showcase_report.summary_path.relative_to(destination).as_posix(),
                    "manifest": showcase_report.manifest_path.relative_to(destination).as_posix(),
                },
                "provenance": hl20_low_fidelity_provenance(
                    aerodynamic_model_id=HL20_SOURCE_MODEL_ID if source_bound else "surrogate_fixed_cd_ld_v1",
                    attitude_control_mode="velocity_aligned" if source_bound else "open_loop",
                    actuator_profile_id="hl20.reference_first_order.v1" if source_bound else "none",
                    mission_profile_id="hl20.energy_managed_entry_glide.v1" if source_bound else "none",
                    glide_bank_schedule_deg=HL20_ENERGY_MANAGED_BANK_SCHEDULE_DEG if source_bound else (),
                    mission_segment_schedule=HL20_ENERGY_MANAGED_SEGMENTS if source_bound else (),
                    initial_speed_m_s=350.0 if source_bound else 250.0,
                    attitude_control_gain=2_000_000.0 if source_bound else 500_000.0,
                    attitude_rate_damping=800_000.0 if source_bound else 300_000.0,
                    configuration_variant_id="hl20_source_nominal_v1" if source_bound else "nominal",
                    mass_property_profile_id="hl20_mod_k_fixed_mass_inertia_v1" if source_bound else "generic_slender_body_v1",
                    inertia_body_kg_m2=HL20_REFERENCE_INERTIA_KG_M2 if source_bound else None,
                ),
                "provenance_by_fidelity": {
                    fidelity.value: hl20_low_fidelity_provenance(
                        fidelity,
                        aerodynamic_model_id=HL20_SOURCE_MODEL_ID if source_bound else "surrogate_fixed_cd_ld_v1",
                        attitude_control_mode="velocity_aligned" if source_bound else "open_loop",
                        actuator_profile_id="hl20.reference_first_order.v1" if source_bound else "none",
                        mission_profile_id="hl20.energy_managed_entry_glide.v1" if source_bound else "none",
                        glide_bank_schedule_deg=HL20_ENERGY_MANAGED_BANK_SCHEDULE_DEG if source_bound else (),
                        mission_segment_schedule=HL20_ENERGY_MANAGED_SEGMENTS if source_bound else (),
                        initial_speed_m_s=350.0 if source_bound else 250.0,
                        attitude_control_gain=2_000_000.0 if source_bound else 500_000.0,
                        attitude_rate_damping=800_000.0 if source_bound else 300_000.0,
                        configuration_variant_id="hl20_source_nominal_v1" if source_bound else "nominal",
                        mass_property_profile_id="hl20_mod_k_fixed_mass_inertia_v1" if source_bound else "generic_slender_body_v1",
                        inertia_body_kg_m2=HL20_REFERENCE_INERTIA_KG_M2 if source_bound else None,
                    )
                    for fidelity in HL20_FIDELITIES
                },
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return {
        "envelopes": envelopes,
        "manifest_path": manifest_path,
        "artifact_paths": tuple(destination / name for name in artifacts),
        "plot_report": plot_report,
        "showcase_report": showcase_report,
    }
    ####


__all__ = [
    "HL20LowFidelityBundle",
    "HL20_FIDELITIES",
    "HL20_REFERENCE_INERTIA_KG_M2",
    "HL20_ENERGY_MANAGED_BANK_SCHEDULE_DEG",
    "HL20_ENERGY_MANAGED_SEGMENTS",
    "HL20_CAHI_TARGET_DISTANCE_M",
    "hl20_low_fidelity_provenance",
    "hl20_release_commands",
    "hl20_ca_hi_terminal_criteria",
    "hl20_source_release_commands",
    "hl20_source_release_vehicle",
    "hl20_release_vehicle",
    "run_hl20_low_fidelity_release",
    "run_hl20_fidelity_ladder",
    "run_hl20_release",
    "run_hl20_source_fidelity_ladder",
    "run_hl20_source_release",
    "write_hl20_fidelity_bundle",
    "write_hl20_low_fidelity_bundle",
]
