"""Phase-aware CADAC-to-Taoryx fidelity classification manifests."""

from __future__ import annotations

from enum import StrEnum
from typing import Literal

from pydantic import Field, model_validator

from taoryx.fidelity_contracts import FidelityTier

from .input_ast import CadacModel

CadacRuntimeFidelity = Literal["static", "point_mass_3dof", "pseudo_6dof", "rigid_body_6dof"]


class CadacPackageId(StrEnum):
    """Source package directories present in the public CADAC repository."""

    ADS6 = "ADS6"
    AGM6 = "AGM6"
    AIM5 = "AIM5"
    CRUISE5 = "CRUISE5"
    FALCON6 = "FALCON6"
    GHAME3 = "GHAME3"
    GHAME6 = "GHAME6"
    MAGSIX = "MAGSIX"
    ROCKET6G = "ROCKET6G"
    SRAAM6 = "SRAAM6"


####


class CadacActorKind(StrEnum):
    """Broad actor role used by the source package."""

    VEHICLE = "vehicle"
    TARGET = "target"
    SENSOR = "sensor"
    STATIC = "static"


####


class CadacControlRealization(StrEnum):
    """How a source phase closes its control demand into plant dynamics."""

    FORCE_MODEL = "force_model"
    RESPONSE_LAW = "response_law"
    DIRECT_WRENCH = "direct_wrench"
    EFFECTOR_ALLOCATED = "effector_allocated"
    MIXED_EFFECTOR = "mixed_effector"
    UNCONTROLLED = "uncontrolled"


####


class CadacEffectorKind(StrEnum):
    """Physical or aggregate control effectors represented by a source phase."""

    AERODYNAMIC_SURFACE = "aerodynamic_surface"
    TVC_GIMBAL = "tvc_gimbal"
    AGGREGATE_RCS = "aggregate_rcs"
    RCS_THRUSTER = "rcs_thruster"
    PROPULSION_THROTTLE = "propulsion_throttle"


####


class CadacAllocationGranularity(StrEnum):
    """Resolution at which source control is realized."""

    NONE = "none"
    FIXED_MIXING = "fixed_mixing"
    CONSTRAINED_PHYSICAL_EFFECTOR = "constrained_physical_effector"
    AXIS_AGGREGATE = "axis_aggregate"
    MIXED = "mixed"


####


class CadacActuatorDynamics(StrEnum):
    """Highest actuator-response behavior represented by a source phase."""

    NONE = "none"
    IDEAL = "ideal"
    FIRST_ORDER = "first_order"
    SECOND_ORDER = "second_order"
    ON_OFF_HYSTERETIC = "on_off_hysteretic"
    MIXED = "mixed"


####


class CadacEvidenceStatus(StrEnum):
    """Current evidence boundary for a prototype classification."""

    SOURCE_CLASSIFIED = "source_classified"
    NUMERICAL_PARITY_PENDING = "numerical_parity_pending"


####


class CadacPhaseFidelity(CadacModel):
    """One actor/phase fidelity claim and its control-realization evidence."""

    phase_id: str = Field(min_length=1)
    description: str = Field(min_length=1)
    taoryx_tier: FidelityTier | None
    runtime_fidelity: CadacRuntimeFidelity
    control_realization: CadacControlRealization
    effectors: tuple[CadacEffectorKind, ...] = ()
    allocation_granularity: CadacAllocationGranularity = CadacAllocationGranularity.NONE
    actuator_dynamics: CadacActuatorDynamics = CadacActuatorDynamics.NONE
    source_modules: tuple[str, ...] = ()
    evidence_status: CadacEvidenceStatus = CadacEvidenceStatus.NUMERICAL_PARITY_PENDING
    notes: tuple[str, ...] = ()

    @model_validator(mode="after")
    def validate_claim(self) -> "CadacPhaseFidelity":
        if self.runtime_fidelity == "static":
            if self.taoryx_tier is not None or self.control_realization is not CadacControlRealization.UNCONTROLLED:
                raise ValueError("static phases must omit a trajectory tier and be uncontrolled")
            ####
            if self.effectors or self.allocation_granularity is not CadacAllocationGranularity.NONE:
                raise ValueError("static phases cannot advertise control effectors")
            ####
            return self
        ####
        if self.taoryx_tier is None:
            raise ValueError("dynamic phases require an explicit Taoryx tier")
        ####
        runtime_by_tier: dict[FidelityTier, CadacRuntimeFidelity] = {
            "point_mass_3dof": "point_mass_3dof",
            "pseudo_6dof": "pseudo_6dof",
            "rigid_body_6dof_direct_wrench": "rigid_body_6dof",
            "rigid_body_6dof_surface_allocated": "rigid_body_6dof",
        }
        if self.runtime_fidelity != runtime_by_tier[self.taoryx_tier]:
            raise ValueError("runtime_fidelity does not match taoryx_tier")
        ####
        if self.taoryx_tier == "point_mass_3dof" and self.control_realization not in {
            CadacControlRealization.FORCE_MODEL,
            CadacControlRealization.UNCONTROLLED,
        }:
            raise ValueError("point-mass phases require force-model or uncontrolled realization")
        ####
        if self.taoryx_tier == "pseudo_6dof" and self.control_realization not in {
            CadacControlRealization.RESPONSE_LAW,
            CadacControlRealization.UNCONTROLLED,
        }:
            raise ValueError("pseudo-6-DoF phases require response-law or uncontrolled realization")
        ####
        if self.taoryx_tier == "rigid_body_6dof_direct_wrench" and self.control_realization not in {
            CadacControlRealization.DIRECT_WRENCH,
            CadacControlRealization.UNCONTROLLED,
        }:
            raise ValueError("direct-wrench phases require direct-wrench or uncontrolled realization")
        ####
        if self.taoryx_tier == "rigid_body_6dof_surface_allocated":
            if self.control_realization not in {
                CadacControlRealization.EFFECTOR_ALLOCATED,
                CadacControlRealization.MIXED_EFFECTOR,
                CadacControlRealization.UNCONTROLLED,
            }:
                raise ValueError("highest-tier phases require physical or mixed effector realization")
            ####
            if self.control_realization is not CadacControlRealization.UNCONTROLLED and not self.effectors:
                raise ValueError("controlled highest-tier phases must identify physical effectors")
            ####
        ####
        if self.control_realization is CadacControlRealization.EFFECTOR_ALLOCATED:
            if self.allocation_granularity not in {
                CadacAllocationGranularity.FIXED_MIXING,
                CadacAllocationGranularity.CONSTRAINED_PHYSICAL_EFFECTOR,
            }:
                raise ValueError("effector-allocated phases require physical allocation granularity")
            ####
            if all(effector is CadacEffectorKind.AGGREGATE_RCS for effector in self.effectors):
                raise ValueError("aggregate RCS alone is not a physical-effector claim")
            ####
        ####
        if self.control_realization is CadacControlRealization.DIRECT_WRENCH:
            if self.allocation_granularity not in {
                CadacAllocationGranularity.NONE,
                CadacAllocationGranularity.AXIS_AGGREGATE,
            }:
                raise ValueError("direct-wrench phases cannot claim physical allocation")
            ####
        ####
        if self.control_realization is CadacControlRealization.MIXED_EFFECTOR:
            if self.allocation_granularity is not CadacAllocationGranularity.MIXED:
                raise ValueError("mixed-effector phases require mixed allocation granularity")
            ####
        ####
        return self

    ####


####


class CadacActorManifest(CadacModel):
    """One independently classifiable actor in a CADAC package."""

    actor_id: str = Field(min_length=1)
    source_model: str = Field(min_length=1)
    kind: CadacActorKind
    phases: tuple[CadacPhaseFidelity, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_unique_phases(self) -> "CadacActorManifest":
        phase_ids = [phase.phase_id.casefold() for phase in self.phases]
        if len(phase_ids) != len(set(phase_ids)):
            raise ValueError(f"actor {self.actor_id!r} contains duplicate phase IDs")
        ####
        return self

    ####


####


class CadacPackageManifest(CadacModel):
    """One source package and all independently classified actors."""

    package_id: CadacPackageId
    description: str = Field(min_length=1)
    source_repository: str = "missiondesignsolutions/CADAC"
    source_path: str = Field(min_length=1)
    actors: tuple[CadacActorManifest, ...] = Field(min_length=1)
    notes: tuple[str, ...] = ()

    @model_validator(mode="after")
    def validate_unique_actors(self) -> "CadacPackageManifest":
        actor_ids = [actor.actor_id.casefold() for actor in self.actors]
        if len(actor_ids) != len(set(actor_ids)):
            raise ValueError(f"package {self.package_id.value!r} contains duplicate actor IDs")
        ####
        return self

    ####


####


class CadacManifestCatalog(CadacModel):
    """Complete source-classification catalog for the ten CADAC packages."""

    schema_id: str = "taoryx.cadac-manifest/v0alpha1"
    packages: tuple[CadacPackageManifest, ...] = Field(min_length=1)

    def package(self, package_id: str | CadacPackageId) -> CadacPackageManifest:
        """Return one package by case-insensitive source-directory name."""

        key = str(package_id).casefold()
        for package in self.packages:
            if package.package_id.value.casefold() == key:
                return package
            ####
        ####
        raise KeyError(package_id)

    ####

    @model_validator(mode="after")
    def validate_unique_packages(self) -> "CadacManifestCatalog":
        package_ids = [package.package_id.value for package in self.packages]
        if len(package_ids) != len(set(package_ids)):
            raise ValueError("catalog contains duplicate package IDs")
        ####
        return self

    ####


####


def _phase(
    phase_id: str,
    description: str,
    tier: FidelityTier | None,
    runtime: CadacRuntimeFidelity,
    realization: CadacControlRealization,
    *,
    effectors: tuple[CadacEffectorKind, ...] = (),
    allocation: CadacAllocationGranularity = CadacAllocationGranularity.NONE,
    actuator: CadacActuatorDynamics = CadacActuatorDynamics.NONE,
    modules: tuple[str, ...] = (),
    notes: tuple[str, ...] = (),
) -> CadacPhaseFidelity:
    return CadacPhaseFidelity(
        phase_id=phase_id,
        description=description,
        taoryx_tier=tier,
        runtime_fidelity=runtime,
        control_realization=realization,
        effectors=effectors,
        allocation_granularity=allocation,
        actuator_dynamics=actuator,
        source_modules=modules,
        notes=notes,
    )


####


def _actor(
    actor_id: str,
    source_model: str,
    kind: CadacActorKind,
    *phases: CadacPhaseFidelity,
) -> CadacActorManifest:
    return CadacActorManifest(actor_id=actor_id, source_model=source_model, kind=kind, phases=phases)


####


_T1: FidelityTier = "point_mass_3dof"
_T2: FidelityTier = "pseudo_6dof"
_T3: FidelityTier = "rigid_body_6dof_direct_wrench"
_T4: FidelityTier = "rigid_body_6dof_surface_allocated"

_SURFACE_MODULES = ("control", "actuator", "aerodynamics", "forces", "euler", "newton")
_TVC_MODULES = ("control", "tvc", "propulsion", "forces", "euler", "newton")
_RCS_MODULES = ("control", "rcs", "forces", "euler", "newton")


CADAC_MANIFEST_CATALOG = CadacManifestCatalog(
    packages=(
        CadacPackageManifest(
            package_id=CadacPackageId.ADS6,
            description="Air-defense engagement composition with mixed-fidelity SAM, ballistic target, aircraft, and radar actors.",
            source_path="ADS6",
            actors=(
                _actor(
                    "sam",
                    "SAM6",
                    CadacActorKind.VEHICLE,
                    _phase(
                        "fin_control",
                        "Physical fin-controlled SAM",
                        _T4,
                        "rigid_body_6dof",
                        CadacControlRealization.EFFECTOR_ALLOCATED,
                        effectors=(CadacEffectorKind.AERODYNAMIC_SURFACE,),
                        allocation=CadacAllocationGranularity.FIXED_MIXING,
                        actuator=CadacActuatorDynamics.SECOND_ORDER,
                        modules=_SURFACE_MODULES,
                    ),
                    _phase(
                        "tvc_control",
                        "Physical thrust-vector-controlled SAM",
                        _T4,
                        "rigid_body_6dof",
                        CadacControlRealization.EFFECTOR_ALLOCATED,
                        effectors=(CadacEffectorKind.TVC_GIMBAL,),
                        allocation=CadacAllocationGranularity.CONSTRAINED_PHYSICAL_EFFECTOR,
                        actuator=CadacActuatorDynamics.SECOND_ORDER,
                        modules=_TVC_MODULES,
                    ),
                    _phase(
                        "aggregate_rcs",
                        "Axis-aggregate reaction-control realization",
                        _T3,
                        "rigid_body_6dof",
                        CadacControlRealization.DIRECT_WRENCH,
                        effectors=(CadacEffectorKind.AGGREGATE_RCS,),
                        allocation=CadacAllocationGranularity.AXIS_AGGREGATE,
                        actuator=CadacActuatorDynamics.ON_OFF_HYSTERETIC,
                        modules=_RCS_MODULES,
                    ),
                ),
                _actor(
                    "srbm",
                    "ROCKET5",
                    CadacActorKind.TARGET,
                    _phase(
                        "source_model",
                        "Reduced-order short-range ballistic missile",
                        _T2,
                        "pseudo_6dof",
                        CadacControlRealization.RESPONSE_LAW,
                        modules=("environment", "kinematics", "propulsion", "aerodynamics", "sensor", "guidance", "control", "forces", "newton", "intercept"),
                    ),
                ),
                _actor(
                    "aircraft",
                    "AIRCRAFT3",
                    CadacActorKind.TARGET,
                    _phase(
                        "source_model",
                        "Three-degree-of-freedom aircraft target",
                        _T1,
                        "point_mass_3dof",
                        CadacControlRealization.FORCE_MODEL,
                        modules=("environment", "kinematics", "guidance", "control", "forces", "newton"),
                    ),
                ),
                _actor(
                    "radar", "RADAR0", CadacActorKind.SENSOR, _phase("fixed_site", "Fixed radar actor", None, "static", CadacControlRealization.UNCONTROLLED)
                ),
            ),
            notes=("Treat ADS6 as a mission composition, not a single fidelity tier.",),
        ),
        CadacPackageManifest(
            package_id=CadacPackageId.AGM6,
            description="Surface-controlled six-degree-of-freedom air-to-ground missile engagement.",
            source_path="AGM6",
            actors=(
                _actor(
                    "missile",
                    "MISSILE6",
                    CadacActorKind.VEHICLE,
                    _phase(
                        "fin_control",
                        "Physical fin-controlled missile",
                        _T4,
                        "rigid_body_6dof",
                        CadacControlRealization.EFFECTOR_ALLOCATED,
                        effectors=(CadacEffectorKind.AERODYNAMIC_SURFACE,),
                        allocation=CadacAllocationGranularity.FIXED_MIXING,
                        actuator=CadacActuatorDynamics.SECOND_ORDER,
                        modules=_SURFACE_MODULES,
                    ),
                ),
                _actor(
                    "aircraft",
                    "AIRCRAFT3",
                    CadacActorKind.VEHICLE,
                    _phase(
                        "carrier",
                        "Three-degree-of-freedom tracking aircraft",
                        _T1,
                        "point_mass_3dof",
                        CadacControlRealization.FORCE_MODEL,
                        modules=("environment", "sensor", "guidance", "control", "newton"),
                    ),
                ),
                _actor(
                    "ground_target",
                    "TARGET3",
                    CadacActorKind.TARGET,
                    _phase(
                        "moving_target",
                        "Moving three-degree-of-freedom ground target",
                        _T1,
                        "point_mass_3dof",
                        CadacControlRealization.FORCE_MODEL,
                        modules=("environment", "forces", "newton"),
                    ),
                ),
            ),
        ),
        CadacPackageManifest(
            package_id=CadacPackageId.AIM5,
            description="Pseudo-five-degree-of-freedom missile against a three-degree-of-freedom aircraft target.",
            source_path="AIM5",
            actors=(
                _actor(
                    "missile",
                    "AIM5",
                    CadacActorKind.VEHICLE,
                    _phase(
                        "source_model",
                        "Translational missile with reduced-order angle and autopilot response",
                        _T2,
                        "pseudo_6dof",
                        CadacControlRealization.RESPONSE_LAW,
                        modules=("guidance", "control", "forces", "newton"),
                    ),
                ),
                _actor(
                    "target",
                    "AIRCRAFT3",
                    CadacActorKind.TARGET,
                    _phase("source_model", "Three-degree-of-freedom aircraft target", _T1, "point_mass_3dof", CadacControlRealization.FORCE_MODEL),
                ),
            ),
        ),
        CadacPackageManifest(
            package_id=CadacPackageId.CRUISE5,
            description="Round-Earth cruise trajectory with reduced-order pitch and bank response.",
            source_path="CRUISE5",
            actors=(
                _actor(
                    "cruise_vehicle",
                    "CRUISE3",
                    CadacActorKind.VEHICLE,
                    _phase("source_model", "Source-faithful pseudo-five-degree-of-freedom response", _T2, "pseudo_6dof", CadacControlRealization.RESPONSE_LAW),
                    _phase(
                        "translation_only", "Translation-only operation over the same force model", _T1, "point_mass_3dof", CadacControlRealization.FORCE_MODEL
                    ),
                ),
            ),
        ),
        CadacPackageManifest(
            package_id=CadacPackageId.FALCON6,
            description="Full rigid-body F-16-class aircraft with physical aileron, elevator, and rudder dynamics.",
            source_path="FALCON6",
            actors=(
                _actor(
                    "aircraft",
                    "PLANE6",
                    CadacActorKind.VEHICLE,
                    _phase(
                        "surface_control",
                        "Physical surface-controlled rigid-body aircraft",
                        _T4,
                        "rigid_body_6dof",
                        CadacControlRealization.EFFECTOR_ALLOCATED,
                        effectors=(CadacEffectorKind.AERODYNAMIC_SURFACE, CadacEffectorKind.PROPULSION_THROTTLE),
                        allocation=CadacAllocationGranularity.CONSTRAINED_PHYSICAL_EFFECTOR,
                        actuator=CadacActuatorDynamics.SECOND_ORDER,
                        modules=_SURFACE_MODULES + ("propulsion",),
                    ),
                ),
            ),
        ),
        CadacPackageManifest(
            package_id=CadacPackageId.GHAME3,
            description="Round, rotating-Earth three-degree-of-freedom hypersonic cruise model with prescribed incidence/bank and propulsion control.",
            source_path="GHAME3",
            actors=(
                _actor(
                    "hypersonic_vehicle",
                    "CRUISE3",
                    CadacActorKind.VEHICLE,
                    _phase(
                        "source_model",
                        "Round-Earth 3-DoF hypersonic translation with prescribed alpha/bank and propulsion control",
                        _T1,
                        "point_mass_3dof",
                        CadacControlRealization.FORCE_MODEL,
                    ),
                ),
            ),
        ),
        CadacPackageManifest(
            package_id=CadacPackageId.GHAME6,
            description="Mixed atmospheric and exo-atmospheric hypersonic mission composition.",
            source_path="GHAME6",
            actors=(
                _actor(
                    "hypersonic_vehicle",
                    "HYPER6",
                    CadacActorKind.VEHICLE,
                    _phase(
                        "atmospheric_surfaces",
                        "Atmospheric left/right elevon and rudder control",
                        _T4,
                        "rigid_body_6dof",
                        CadacControlRealization.EFFECTOR_ALLOCATED,
                        effectors=(CadacEffectorKind.AERODYNAMIC_SURFACE,),
                        allocation=CadacAllocationGranularity.FIXED_MIXING,
                        actuator=CadacActuatorDynamics.SECOND_ORDER,
                        modules=_SURFACE_MODULES,
                    ),
                    _phase(
                        "transfer_angle_rcs",
                        "Transfer-vehicle aggregate RCS angle control",
                        _T3,
                        "rigid_body_6dof",
                        CadacControlRealization.DIRECT_WRENCH,
                        effectors=(CadacEffectorKind.AGGREGATE_RCS,),
                        allocation=CadacAllocationGranularity.AXIS_AGGREGATE,
                        actuator=CadacActuatorDynamics.ON_OFF_HYSTERETIC,
                        modules=_RCS_MODULES,
                    ),
                    _phase(
                        "transfer_vector_rcs",
                        "Transfer-vehicle aggregate RCS thrust-vector-direction control",
                        _T3,
                        "rigid_body_6dof",
                        CadacControlRealization.DIRECT_WRENCH,
                        effectors=(CadacEffectorKind.AGGREGATE_RCS,),
                        allocation=CadacAllocationGranularity.AXIS_AGGREGATE,
                        actuator=CadacActuatorDynamics.ON_OFF_HYSTERETIC,
                        modules=_RCS_MODULES,
                    ),
                    _phase(
                        "interceptor_glideslope_rcs",
                        "Interceptor aggregate RCS glideslope phase",
                        _T3,
                        "rigid_body_6dof",
                        CadacControlRealization.DIRECT_WRENCH,
                        effectors=(CadacEffectorKind.AGGREGATE_RCS,),
                        allocation=CadacAllocationGranularity.AXIS_AGGREGATE,
                        actuator=CadacActuatorDynamics.ON_OFF_HYSTERETIC,
                        modules=_RCS_MODULES,
                    ),
                    _phase(
                        "interceptor_terminal_rcs",
                        "Terminal aggregate RCS moment and side-force control",
                        _T3,
                        "rigid_body_6dof",
                        CadacControlRealization.DIRECT_WRENCH,
                        effectors=(CadacEffectorKind.AGGREGATE_RCS,),
                        allocation=CadacAllocationGranularity.AXIS_AGGREGATE,
                        actuator=CadacActuatorDynamics.MIXED,
                        modules=_RCS_MODULES,
                    ),
                ),
                _actor(
                    "satellite",
                    "SAT3",
                    CadacActorKind.TARGET,
                    _phase("trajectory", "Three-degree-of-freedom satellite trajectory actor", _T1, "point_mass_3dof", CadacControlRealization.UNCONTROLLED),
                ),
                _actor(
                    "ground_site",
                    "RADAR0",
                    CadacActorKind.STATIC,
                    _phase("fixed_site", "Fixed rotating-Earth ground radar and SAT3 track publisher", None, "static", CadacControlRealization.UNCONTROLLED),
                ),
            ),
            notes=("Control realization must be reported by phase rather than package maximum.", "The shipped GHAME6 source mission contains no TVC module."),
        ),
        CadacPackageManifest(
            package_id=CadacPackageId.MAGSIX,
            description="Restricted six-degree-of-freedom perturbation model with one-way trajectory-to-attitude coupling.",
            source_path="MAGSIX",
            actors=(
                _actor(
                    "vehicle",
                    "ROTOR",
                    CadacActorKind.VEHICLE,
                    _phase(
                        "restricted_attitude",
                        "Trajectory-driven attitude perturbation response",
                        _T2,
                        "pseudo_6dof",
                        CadacControlRealization.RESPONSE_LAW,
                        notes=("Do not promote solely because the source name contains SIX.",),
                    ),
                    _phase("trajectory_only", "Planar trajectory-only operation", _T1, "point_mass_3dof", CadacControlRealization.FORCE_MODEL),
                ),
            ),
        ),
        CadacPackageManifest(
            package_id=CadacPackageId.ROCKET6G,
            description="Staged launch vehicle with phase-dependent TVC, aggregate RCS, and ballistic coast behavior.",
            source_path="ROCKET6G",
            actors=(
                _actor(
                    "launch_vehicle",
                    "HYPER6",
                    CadacActorKind.VEHICLE,
                    _phase(
                        "aggregate_rcs",
                        "Axis-aggregate RCS-controlled phase",
                        _T3,
                        "rigid_body_6dof",
                        CadacControlRealization.DIRECT_WRENCH,
                        effectors=(CadacEffectorKind.AGGREGATE_RCS,),
                        allocation=CadacAllocationGranularity.AXIS_AGGREGATE,
                        actuator=CadacActuatorDynamics.ON_OFF_HYSTERETIC,
                        modules=_RCS_MODULES,
                    ),
                    _phase(
                        "physical_tvc",
                        "Physical gimbal-state thrust-vector-controlled phase",
                        _T4,
                        "rigid_body_6dof",
                        CadacControlRealization.EFFECTOR_ALLOCATED,
                        effectors=(CadacEffectorKind.TVC_GIMBAL,),
                        allocation=CadacAllocationGranularity.CONSTRAINED_PHYSICAL_EFFECTOR,
                        actuator=CadacActuatorDynamics.SECOND_ORDER,
                        modules=_TVC_MODULES,
                    ),
                    _phase(
                        "mixed_tvc_rcs",
                        "TVC pitch/yaw with aggregate RCS roll",
                        _T4,
                        "rigid_body_6dof",
                        CadacControlRealization.MIXED_EFFECTOR,
                        effectors=(CadacEffectorKind.TVC_GIMBAL, CadacEffectorKind.AGGREGATE_RCS),
                        allocation=CadacAllocationGranularity.MIXED,
                        actuator=CadacActuatorDynamics.MIXED,
                        modules=_TVC_MODULES + ("rcs",),
                        notes=("Tier is the highest realized path; evidence must preserve the aggregate-RCS limitation by axis.",),
                    ),
                    _phase(
                        "ballistic_coast",
                        "Uncontrolled full rigid-body coast",
                        _T3,
                        "rigid_body_6dof",
                        CadacControlRealization.UNCONTROLLED,
                        notes=("The canonical direct-wrench slot is used only as the available rigid-body tier; no control-realization claim is made.",),
                    ),
                ),
            ),
        ),
        CadacPackageManifest(
            package_id=CadacPackageId.SRAAM6,
            description="Short-range air-to-air missile with physical fins and an optional physical TVC path.",
            source_path="SRAAM6",
            actors=(
                _actor(
                    "missile",
                    "MISSILE6",
                    CadacActorKind.VEHICLE,
                    _phase(
                        "fin_control",
                        "Four-fin physical actuator realization",
                        _T4,
                        "rigid_body_6dof",
                        CadacControlRealization.EFFECTOR_ALLOCATED,
                        effectors=(CadacEffectorKind.AERODYNAMIC_SURFACE,),
                        allocation=CadacAllocationGranularity.FIXED_MIXING,
                        actuator=CadacActuatorDynamics.SECOND_ORDER,
                        modules=_SURFACE_MODULES,
                    ),
                    _phase(
                        "optional_tvc",
                        "Optional physical thrust-vector-control path",
                        _T4,
                        "rigid_body_6dof",
                        CadacControlRealization.EFFECTOR_ALLOCATED,
                        effectors=(CadacEffectorKind.TVC_GIMBAL,),
                        allocation=CadacAllocationGranularity.CONSTRAINED_PHYSICAL_EFFECTOR,
                        actuator=CadacActuatorDynamics.SECOND_ORDER,
                        modules=_TVC_MODULES,
                    ),
                ),
                _actor(
                    "target",
                    "TARGET3",
                    CadacActorKind.TARGET,
                    _phase("source_model", "Three-degree-of-freedom aircraft target", _T1, "point_mass_3dof", CadacControlRealization.FORCE_MODEL),
                ),
            ),
        ),
    )
)


def manifest_for(package_id: str | CadacPackageId) -> CadacPackageManifest:
    """Return one package manifest from the canonical prototype catalog."""

    return CADAC_MANIFEST_CATALOG.package(package_id)


####


__all__ = [
    "CADAC_MANIFEST_CATALOG",
    "CadacActorKind",
    "CadacActorManifest",
    "CadacActuatorDynamics",
    "CadacAllocationGranularity",
    "CadacControlRealization",
    "CadacEffectorKind",
    "CadacEvidenceStatus",
    "CadacManifestCatalog",
    "CadacPackageId",
    "CadacPackageManifest",
    "CadacPhaseFidelity",
    "CadacRuntimeFidelity",
    "manifest_for",
]
