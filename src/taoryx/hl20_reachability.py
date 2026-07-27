"""Low-fidelity rocket-release and glide scenario for the HL-20 family."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

from .contracts import Vector3
from .reachability_envelope import (
    LaunchCommand,
    ReachabilityEnvelope,
    ReachabilityFidelity,
    RocketGlideVehicle,
    run_reachability_envelope,
)
from .vehicle import DetachedBodyDefinition

HL20_REFERENCE_AREA_M2 = 26.612075808
HL20_FIXED_MASS_KG = 8_664.1
HL20_PACKAGE_SHA256 = "443e2ed905e310cc57014dc3bad8d3b24b6b5954b8423de9110d405d66154e63"
HL20_AERODYNAMICS_SHA256 = "b2ec6260ed60d241de250599b269ad35d0b96865b50da5e7f9ef7e04de3844ec"


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
)


def hl20_release_vehicle() -> RocketGlideVehicle:
    """Build the synthetic booster plus source-bound HL-20 glide child.

    The child mass and reference area come from the pinned HL-20 package. Its
    fixed drag and lift-to-drag values are explicitly a low-fidelity surrogate
    until the DAVE-ML graph is coupled to the reduced solver.
    """

    spent_booster = DetachedBodyDefinition.cylinder(
        "hl20-synthetic-spent-booster",
        mass_kg=1_500.0,
        radius_m=0.8,
        length_m=6.0,
        inertia_kg_m2=Vector3(5_000.0, 5_000.0, 500.0),
    )
    return RocketGlideVehicle(
        vehicle_id="hl20-low-fidelity-release-v1",
        dry_mass_kg=HL20_FIXED_MASS_KG,
        propellant_mass_kg=0.0,
        thrust_n=0.0,
        burn_time_s=0.0,
        reference_area_m2=HL20_REFERENCE_AREA_M2,
        drag_coefficient=0.06,
        lift_to_drag=3.2,
        initial_speed_m_s=250.0,
        initial_altitude_m=0.0,
        booster_dry_mass_kg=1_500.0,
        booster_propellant_mass_kg=1_500.0,
        booster_thrust_n=120_000.0,
        booster_burn_time_s=15.0,
        booster_release_time_s=25.0,
        booster_detached_body=spent_booster,
    )
    ####


def hl20_release_commands() -> tuple[LaunchCommand, ...]:
    """Return the small deterministic first-tranche launch grid."""

    return tuple(
        LaunchCommand(azimuth_rad=0.0, elevation_rad=math.radians(elevation))
        for elevation in (60.0, 70.0, 80.0)
    )
    ####


def hl20_low_fidelity_provenance() -> dict[str, object]:
    """Describe the source boundary and synthetic launch-parent assumptions."""

    return {
        "family_group": "hypersonic_lifting_body_research",
        "family_id": "reference_hl20_mod_k",
        "scenario_id": "hl20-rocket-release-3dof-v1",
        "qualification": "synthetic_booster_source_bound_glide_child",
        "source_package_sha256": HL20_PACKAGE_SHA256,
        "aerodynamics_sha256": HL20_AERODYNAMICS_SHA256,
        "source_family_manifest": "families/reference_hl20_mod_k/family.yaml",
        "child_model": "reference_hl20_mod_k",
        "rocket_parent": "generic-staged-booster-v1",
        "rocket_parent_claim": "synthetic scenario assumption only",
        "child_aerodynamics": "fixed_cd_and_ld_low_fidelity_surrogate",
        "child_mass_binding": "separate_fixed_binding_v1",
        "controller_claim": False,
        "route_claim": False,
        "source_exact_trajectory": False,
        "fidelity": "point_mass_3dof",
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
) -> ReachabilityEnvelope:
    """Run one fidelity tier of the HL-20 rocket-release envelope."""

    if fidelity not in HL20_FIDELITIES:
        raise ValueError(f"unsupported HL-20 fidelity: {fidelity}")

    return run_reachability_envelope(
        hl20_release_vehicle(),
        commands or hl20_release_commands(),
        fidelity=fidelity,
        step_size_s=step_size_s,
        horizon_s=horizon_s,
        workers=1,
        study_id="hl20_rocket_release_3dof_v1",
        provenance=hl20_low_fidelity_provenance(),
        spawn_children=spawn_children,
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
    """Run all three HL-20 rocket-release fidelity tiers over one search grid."""

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
) -> dict[str, object]:
    """Write all three HL-20 tiers and one comparison manifest."""

    destination = Path(directory)
    destination.mkdir(parents=True, exist_ok=True)
    envelopes = run_hl20_fidelity_ladder(
        commands=commands,
        step_size_s=step_size_s,
        horizon_s=horizon_s,
        spawn_children=spawn_children,
    )
    artifacts: list[str] = []
    for envelope in envelopes:
        artifact_path = destination / f"{envelope.fidelity.value}.json"
        envelope.write_json(artifact_path)
        artifacts.append(artifact_path.name)
    manifest_path = destination / "bundle-manifest.json"
    manifest_path.write_text(
        json.dumps(
            {
                "schema": "taoryx.hl20-rocket-release-fidelity-bundle/v1alpha1",
                "vehicle_id": "reference_hl20_mod_k",
                "family_group": "hypersonic_lifting_body_research",
                "fidelity_profiles": [fidelity.value for fidelity in HL20_FIDELITIES],
                "artifacts": artifacts,
                "claim_boundary": "three fidelity synthetic booster comparison with source-bound HL-20 geometry and fixed mass",
                "provenance": hl20_low_fidelity_provenance(),
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return {"envelopes": envelopes, "manifest_path": manifest_path, "artifact_paths": tuple(destination / name for name in artifacts)}
    ####


__all__ = [
    "HL20LowFidelityBundle",
    "HL20_FIDELITIES",
    "hl20_low_fidelity_provenance",
    "hl20_release_commands",
    "hl20_release_vehicle",
    "run_hl20_low_fidelity_release",
    "run_hl20_fidelity_ladder",
    "run_hl20_release",
    "write_hl20_fidelity_bundle",
    "write_hl20_low_fidelity_bundle",
]
