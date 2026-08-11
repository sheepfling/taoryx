"""First-class CADAC actor plug-in catalog for Taoryx vehicle discovery."""

from __future__ import annotations

from collections.abc import Iterable
from enum import StrEnum
from typing import Literal, Protocol

from pydantic import Field, model_validator

from .input_ast import CadacModel
from .manifest import (
    CADAC_MANIFEST_CATALOG,
    CadacActorKind,
    CadacActorManifest,
    CadacPackageId,
    CadacPackageManifest,
    CadacPhaseFidelity,
)

CadacPluginOperation = Literal["discover", "validate", "batch", "step"]
CadacPluginStatus = Literal["runnable", "embedded", "planned", "static"]


class CadacPluginScope(StrEnum):
    """How one source actor participates in Taoryx composition."""

    PRIMARY_VEHICLE = "primary_vehicle"
    EMBEDDED_VEHICLE = "embedded_vehicle"
    TARGET = "target"
    SENSOR = "sensor"
    STATIC = "static"


####


class CadacVehiclePluginDescriptor(CadacModel):
    """Discoverable, immutable identity for one CADAC source actor plug-in."""

    plugin_id: str = Field(pattern=r"^cadac\.[a-z0-9_]+\.[a-z0-9_]+$")
    model_id: str = Field(pattern=r"^cadac\.[a-z0-9_]+\.[a-z0-9_]+$")
    family_id: str = Field(pattern=r"^cadac\.[a-z0-9_]+\.[a-z0-9_]+$")
    package_id: CadacPackageId
    actor_id: str = Field(min_length=1)
    source_model: str = Field(min_length=1)
    display_name: str = Field(min_length=1)
    scope: CadacPluginScope
    status: CadacPluginStatus
    trajectory_capable: bool
    phases: tuple[CadacPhaseFidelity, ...] = Field(min_length=1)
    default_phase_id: str = Field(min_length=1)
    operations: tuple[CadacPluginOperation, ...] = ("discover", "validate")
    batch_factory_id: str | None = None
    embedded_in_model_ids: tuple[str, ...] = ()
    source_repository: str = "missiondesignsolutions/CADAC"
    source_path: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)
    blockers: tuple[str, ...] = ()

    @model_validator(mode="after")
    def validate_plugin(self) -> "CadacVehiclePluginDescriptor":
        phase_ids = {phase.phase_id for phase in self.phases}
        if self.default_phase_id not in phase_ids:
            raise ValueError(f"CADAC plug-in {self.plugin_id!r} default phase is not declared")
        ####
        if len(self.operations) != len(set(self.operations)):
            raise ValueError(f"CADAC plug-in {self.plugin_id!r} contains duplicate operations")
        ####
        if "discover" not in self.operations or "validate" not in self.operations:
            raise ValueError("every CADAC plug-in must support discovery and validation")
        ####
        if self.status == "runnable":
            if "batch" not in self.operations or self.batch_factory_id is None:
                raise ValueError("runnable CADAC plug-ins require a batch operation and factory")
            ####
            if self.blockers:
                raise ValueError("runnable CADAC plug-ins cannot retain execution blockers")
            ####
        elif self.batch_factory_id is not None:
            raise ValueError("non-runnable CADAC plug-ins cannot advertise a batch factory")
        ####
        if self.status == "planned" and not self.blockers:
            raise ValueError("planned CADAC plug-ins require explicit blockers")
        ####
        if self.status == "static" and self.trajectory_capable:
            raise ValueError("static CADAC plug-ins cannot advertise trajectory capability")
        ####
        if not self.trajectory_capable and any(operation in {"batch", "step"} for operation in self.operations):
            raise ValueError("non-trajectory CADAC plug-ins cannot advertise execution operations")
        ####
        return self

    ####


####


class CadacVehiclePlugin(Protocol):
    """Runtime seam implemented by an executable CADAC actor plug-in."""

    @property
    def descriptor(self) -> CadacVehiclePluginDescriptor:
        """Return immutable discovery metadata."""
        ...

    ####

    def validate_installation(self) -> tuple[str, ...]:
        """Return installation blockers; an empty tuple means runnable."""
        ...

    ####


####


class CadacPluginCatalog(CadacModel):
    """Versioned actor-level CADAC plug-in discovery catalog."""

    schema_id: str = "taoryx.cadac-plugin-catalog/v0alpha1"
    plugins: tuple[CadacVehiclePluginDescriptor, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_unique_plugins(self) -> "CadacPluginCatalog":
        plugin_ids = tuple(item.plugin_id for item in self.plugins)
        model_ids = tuple(item.model_id for item in self.plugins)
        if len(plugin_ids) != len(set(plugin_ids)):
            raise ValueError("CADAC plug-in catalog contains duplicate plug-in IDs")
        ####
        if len(model_ids) != len(set(model_ids)):
            raise ValueError("CADAC plug-in catalog contains duplicate model IDs")
        ####
        return self

    ####

    def plugin(self, plugin_id: str) -> CadacVehiclePluginDescriptor:
        """Return one plug-in descriptor by stable ID."""

        for plugin in self.plugins:
            if plugin.plugin_id == plugin_id:
                return plugin
            ####
        ####
        raise KeyError(plugin_id)

    ####

    def for_package(self, package_id: str | CadacPackageId) -> tuple[CadacVehiclePluginDescriptor, ...]:
        """Return actor plug-ins for one source package."""

        key = package_id.value if isinstance(package_id, CadacPackageId) else str(package_id)
        return tuple(item for item in self.plugins if item.package_id.value.casefold() == key.casefold())

    ####


####


class CadacPluginRegistry:
    """Runtime registry that keeps metadata-only and executable plug-ins distinct."""

    def __init__(self, descriptors: Iterable[CadacVehiclePluginDescriptor] = ()) -> None:
        self._descriptors: dict[str, CadacVehiclePluginDescriptor] = {}
        self._runtime_plugins: dict[str, CadacVehiclePlugin] = {}
        for descriptor in descriptors:
            self.register_descriptor(descriptor)
        ####

    ####

    def register_descriptor(self, descriptor: CadacVehiclePluginDescriptor) -> None:
        """Register metadata without implying an executable implementation."""

        if descriptor.plugin_id in self._descriptors:
            raise ValueError(f"duplicate CADAC plug-in descriptor {descriptor.plugin_id!r}")
        ####
        self._descriptors[descriptor.plugin_id] = descriptor

    ####

    def register_runtime(self, plugin: CadacVehiclePlugin) -> None:
        """Bind an executable implementation to an existing runnable descriptor."""

        descriptor = plugin.descriptor
        existing = self._descriptors.get(descriptor.plugin_id)
        if existing is None:
            self.register_descriptor(descriptor)
            existing = descriptor
        ####
        if existing != descriptor:
            raise ValueError(f"runtime CADAC plug-in {descriptor.plugin_id!r} disagrees with catalog metadata")
        ####
        if descriptor.status != "runnable":
            raise ValueError(f"CADAC plug-in {descriptor.plugin_id!r} is not declared runnable")
        ####
        if descriptor.plugin_id in self._runtime_plugins:
            raise ValueError(f"duplicate CADAC runtime plug-in {descriptor.plugin_id!r}")
        ####
        blockers = plugin.validate_installation()
        if blockers:
            raise ValueError(f"CADAC plug-in {descriptor.plugin_id!r} installation is incomplete: {'; '.join(blockers)}")
        ####
        self._runtime_plugins[descriptor.plugin_id] = plugin

    ####

    def descriptor(self, plugin_id: str) -> CadacVehiclePluginDescriptor:
        """Return one discoverable descriptor."""

        try:
            return self._descriptors[plugin_id]
        except KeyError as error:
            raise KeyError(f"unknown CADAC plug-in {plugin_id!r}") from error
        ####

    ####

    def runtime(self, plugin_id: str) -> CadacVehiclePlugin:
        """Return one executable plug-in or fail without substituting another model."""

        try:
            return self._runtime_plugins[plugin_id]
        except KeyError as error:
            descriptor = self.descriptor(plugin_id)
            raise KeyError(f"CADAC plug-in {plugin_id!r} is discoverable but has no installed runtime; status={descriptor.status!r}") from error
        ####

    ####

    def descriptors(self) -> tuple[CadacVehiclePluginDescriptor, ...]:
        """Return deterministic plug-in discovery records."""

        return tuple(self._descriptors[key] for key in sorted(self._descriptors))

    ####

    def runtime_ids(self) -> tuple[str, ...]:
        """Return installed executable plug-in IDs."""

        return tuple(sorted(self._runtime_plugins))

    ####


####


def _slug(value: str) -> str:
    return value.casefold().replace("-", "_").replace(" ", "_")


####


def _scope_for(actor: CadacActorManifest) -> CadacPluginScope:
    if actor.kind is CadacActorKind.SENSOR:
        return CadacPluginScope.SENSOR
    ####
    if actor.kind is CadacActorKind.STATIC:
        return CadacPluginScope.STATIC
    ####
    if actor.kind is CadacActorKind.TARGET:
        return CadacPluginScope.TARGET
    ####
    return CadacPluginScope.PRIMARY_VEHICLE


####


def _display_name(package: CadacPackageManifest, actor: CadacActorManifest) -> str:
    source = actor.source_model.replace("_", " ")
    return f"CADAC {source} ({package.package_id.value})"


####


def _planned_blockers(package: CadacPackageManifest, actor: CadacActorManifest) -> tuple[str, ...]:
    if actor.kind in {CadacActorKind.SENSOR, CadacActorKind.STATIC}:
        return ()
    ####
    if package.package_id is CadacPackageId.FALCON6 and actor.actor_id == "aircraft":
        return (
            "source lowering, physical actuator dynamics, propulsion, aerodynamic wrench, and rotational derivative are implemented but not yet assembled into the source-ordered full 6-DoF batch runtime",
            "guidance/control, translational integration, event progression, and output projection are not yet bound through the common runner",
            "numerical parity evidence against the original CADAC executable is not registered",
        )
    ####
    return (
        "source actor equations have not yet been lowered into a Taoryx executable plug-in",
        "numerical parity evidence against the original CADAC executable is not registered",
    )


####


def _descriptor_for(package: CadacPackageManifest, actor: CadacActorManifest) -> CadacVehiclePluginDescriptor:
    package_slug = _slug(package.package_id.value)
    actor_slug = _slug(actor.actor_id)
    plugin_id = f"cadac.{package_slug}.{actor_slug}"
    trajectory_capable = actor.kind not in {CadacActorKind.SENSOR, CadacActorKind.STATIC}
    runnable_ads6_sam = package.package_id is CadacPackageId.ADS6 and actor.actor_id == "sam"
    runnable_ads6_srbm = package.package_id is CadacPackageId.ADS6 and actor.actor_id == "srbm"
    runnable_ads6_aircraft = package.package_id is CadacPackageId.ADS6 and actor.actor_id == "aircraft"
    runnable_agm6 = package.package_id is CadacPackageId.AGM6 and actor.actor_id == "missile"
    runnable_aim5 = package.package_id is CadacPackageId.AIM5 and actor.actor_id == "missile"
    runnable_falcon6 = package.package_id is CadacPackageId.FALCON6 and actor.actor_id == "aircraft"
    runnable_cruise5 = package.package_id is CadacPackageId.CRUISE5 and actor.actor_id == "cruise_vehicle"
    runnable_ghame3 = package.package_id is CadacPackageId.GHAME3 and actor.actor_id == "hypersonic_vehicle"
    runnable_ghame6 = package.package_id is CadacPackageId.GHAME6 and actor.actor_id == "hypersonic_vehicle"
    runnable_magsix = package.package_id is CadacPackageId.MAGSIX and actor.actor_id == "vehicle"
    runnable_rocket6g = package.package_id is CadacPackageId.ROCKET6G and actor.actor_id == "launch_vehicle"
    runnable_sraam6 = package.package_id is CadacPackageId.SRAAM6 and actor.actor_id == "missile"
    embedded_agm6_actor = package.package_id is CadacPackageId.AGM6 and actor.actor_id in {"aircraft", "ground_target"}
    embedded_aim5_target = package.package_id is CadacPackageId.AIM5 and actor.actor_id == "target"
    embedded_sraam6_target = package.package_id is CadacPackageId.SRAAM6 and actor.actor_id == "target"
    embedded_ghame6_satellite = package.package_id is CadacPackageId.GHAME6 and actor.actor_id == "satellite"
    if runnable_ads6_sam:
        status: CadacPluginStatus = "runnable"
        operations: tuple[CadacPluginOperation, ...] = ("discover", "validate", "batch")
        factory_id = "cadac.ads6.sam.multi_realization.batch"
        blockers: tuple[str, ...] = ()
    elif runnable_ads6_srbm:
        status = "runnable"
        operations = ("discover", "validate", "batch")
        factory_id = "cadac.ads6.srbm.source_compatibility.batch"
        blockers = ()
    elif runnable_ads6_aircraft:
        status = "runnable"
        operations = ("discover", "validate", "batch")
        factory_id = "cadac.ads6.aircraft.source_compatibility.batch"
        blockers = ()
    elif runnable_agm6:
        status = "runnable"
        operations = ("discover", "validate", "batch")
        factory_id = "cadac.agm6.standard_fin.batch"
        blockers = ()
    elif runnable_aim5:
        status: CadacPluginStatus = "runnable"
        operations: tuple[CadacPluginOperation, ...] = ("discover", "validate", "batch")
        factory_id = "cadac.aim5.source_compatibility.batch"
        blockers: tuple[str, ...] = ()
    elif runnable_falcon6:
        status = "runnable"
        operations = ("discover", "validate", "batch")
        factory_id = "cadac.falcon6.physical_surface.batch"
        blockers = ()
    elif runnable_cruise5:
        status = "runnable"
        operations = ("discover", "validate", "batch")
        factory_id = "cadac.cruise5.source_compatibility.batch"
        blockers = ()
    elif runnable_ghame3:
        status = "runnable"
        operations = ("discover", "validate", "batch")
        factory_id = "cadac.ghame3.source_compatibility.batch"
        blockers = ()
    elif runnable_ghame6:
        status = "runnable"
        operations = ("discover", "validate", "batch")
        factory_id = "cadac.ghame6.phase_aware.batch"
        blockers = ()
    elif runnable_magsix:
        status = "runnable"
        operations = ("discover", "validate", "batch")
        factory_id = "cadac.magsix.trajectory.batch"
        blockers = ()
    elif runnable_rocket6g:
        status = "runnable"
        operations = ("discover", "validate", "batch")
        factory_id = "cadac.rocket6g.phase_aware.batch"
        blockers = ()
    elif runnable_sraam6:
        status = "runnable"
        operations = ("discover", "validate", "batch")
        factory_id = "cadac.sraam6.standard_fin.batch"
        blockers = ()
    elif embedded_agm6_actor or embedded_aim5_target or embedded_sraam6_target or embedded_ghame6_satellite:
        status = "embedded"
        operations = ("discover", "validate")
        factory_id = None
        blockers = ()
    elif not trajectory_capable:
        status = "static"
        operations = ("discover", "validate")
        factory_id = None
        blockers = ()
    else:
        status = "planned"
        operations = ("discover", "validate")
        factory_id = None
        blockers = _planned_blockers(package, actor)
    ####
    embedded_in = (
        ("cadac.agm6.missile",)
        if embedded_agm6_actor
        else ("cadac.aim5.missile",)
        if embedded_aim5_target
        else ("cadac.sraam6.missile",)
        if embedded_sraam6_target
        else ("cadac.ghame6.hypersonic_vehicle",)
        if embedded_ghame6_satellite
        else ()
    )
    if runnable_ads6_sam:
        claim_boundary = (
            "ADS6 SAM batch execution covers the standalone source flat-Earth rigid-body plant with explicit cross-fin, "
            "physical pitch/yaw TVC, or axis-aggregate RCS realization selection. Commands enter at the controller-output "
            "seam; ADS6 seeker, guidance, INS, radar scheduling, target actors, and compiled-CADAC numerical parity remain "
            "outside this vehicle-only claim."
        )
    elif runnable_ads6_srbm:
        claim_boundary = (
            "ADS6 SRBM batch execution covers the ROCKET5 flat-Earth translation, pressure-corrected single-stage propulsion, "
            "table-driven aerodynamics, sticky exo/endo phase logic, optional fixed-target proportional navigation and spiral maneuver, "
            "and reduced-order pitch/yaw-rate plus alpha/beta response states. It remains pseudo-6DoF; full ADS6 radar/SAM engagement "
            "composition and compiled-CADAC numerical parity remain outside this actor-only claim."
        )
    elif runnable_ads6_aircraft:
        claim_boundary = (
            "ADS6 AIRCRAFT3 batch execution covers the Flat3 point-mass translation, source maneuver timing, bank-angle and normal-load "
            "response states, load limiting, and specific-force closure. Bank and load are response telemetry rather than rigid-body "
            "truth. Radar/SAM engagement scheduling, source stochastic parity, and compiled-CADAC numerical parity remain outside this "
            "standalone actor claim."
        )
    elif runnable_agm6:
        claim_boundary = (
            "AGM6 batch execution covers the source flat-Earth rigid-body missile, standard four-fin physical actuator path, "
            "continuous rocket fuel state, moving TARGET3, AIRCRAFT3 track production, source-order datalink lag, guidance, "
            "controller modes, and target-plane intercept. Real-INS error propagation, complete IIR optical geometry, C-rand "
            "parity, and compiled-CADAC numerical parity remain outside the current claim."
        )
    elif runnable_falcon6:
        claim_boundary = (
            "FALCON6 batch execution currently covers the source rigid-body physical plant with direct physical-surface commands; "
            "the CADAC waypoint guidance and autopilot modules remain outside the runnable claim."
        )
    elif runnable_cruise5:
        claim_boundary = (
            "CRUISE5 batch execution covers the source CRUISE3 round/rotating-Earth translation, turbojet, waypoint/line guidance, "
            "and lagged alpha/bank response for the supported source modes. It is a pseudo-6DoF response-law realization, not rigid-body 6-DoF."
        )
    elif runnable_ghame3:
        claim_boundary = (
            "GHAME3 batch execution covers the source CRUISE3 round/rotating-Earth 3-DoF translation, prescribed alpha/bank schedule, "
            "and fixed/Q-hold hypersonic propulsion. No pseudo-6DoF attitude response or rigid-body closure is claimed."
        )
    elif runnable_ghame6:
        claim_boundary = (
            "GHAME6 batch execution preserves HYPER6 WGS84 rigid-body truth, physical atmospheric left/right elevons and rudder, "
            "hypersonic/rocket propulsion transitions, source events, axis-aggregate RCS transfer/interceptor phases, SAT3 truth, "
            "and RADAR0 tracking. Full source GNC/estimation, stochastic C-rand parity, discarded carrier propagation, and compiled-CADAC "
            "numerical parity remain outside the current claim. The shipped GHAME6 source contains no TVC module."
        )
    elif runnable_magsix:
        claim_boundary = (
            "MAGSIX batch execution covers only the source's independently runnable planar center-of-mass/spin trajectory equations. "
            "The restricted attitude-perturbation equations remain a separate pseudo-6DoF phase and are validate-only."
        )
    elif runnable_rocket6g:
        claim_boundary = (
            "ROCKET6G batch execution preserves the source event program, staged mass properties, WGS84 rigid-body truth, "
            "physical TVC, and axis-aggregate RCS. Runtime phase fidelity is emitted per sample; LTG/autopilot/navigation "
            "and compiled-CADAC numerical parity remain outside the current claim."
        )
    elif runnable_sraam6:
        claim_boundary = (
            "SRAAM6 batch execution covers the source flat-Earth rigid-body missile, standard four-fin physical actuator path, "
            "time-deck motor and mass properties, seeker mode machine, proportional-navigation guidance, controller transition, "
            "and independently propagated TARGET3. Optional TVC, complete optical seeker error geometry, and compiled-CADAC "
            "numerical parity remain outside the current claim."
        )
    else:
        claim_boundary = (
            "Actor-level CADAC plug-in identity preserves the source package, actor, phase-specific fidelity, and execution status. "
            "Discovery does not promote planned models or collapse mixed-fidelity source scenarios into one tier."
        )
    ####
    return CadacVehiclePluginDescriptor(
        plugin_id=plugin_id,
        model_id=plugin_id,
        family_id=plugin_id,
        package_id=package.package_id,
        actor_id=actor.actor_id,
        source_model=actor.source_model,
        display_name=_display_name(package, actor),
        scope=(CadacPluginScope.EMBEDDED_VEHICLE if embedded_agm6_actor and actor.actor_id == "aircraft" else _scope_for(actor)),
        status=status,
        trajectory_capable=trajectory_capable,
        phases=actor.phases,
        default_phase_id=(
            "trajectory_only"
            if runnable_magsix
            else "atmospheric_surfaces"
            if runnable_ghame6
            else "mixed_tvc_rcs"
            if runnable_rocket6g
            else "fin_control"
            if runnable_ads6_sam or runnable_agm6 or runnable_sraam6
            else actor.phases[0].phase_id
        ),
        operations=operations,
        batch_factory_id=factory_id,
        embedded_in_model_ids=embedded_in,
        source_path=package.source_path,
        claim_boundary=claim_boundary,
        blockers=blockers,
    )


####


def build_cadac_plugin_catalog() -> CadacPluginCatalog:
    """Project phase-aware package manifests into actor-level plug-in metadata."""

    descriptors = tuple(_descriptor_for(package, actor) for package in CADAC_MANIFEST_CATALOG.packages for actor in package.actors)
    return CadacPluginCatalog(plugins=descriptors)


####


CADAC_PLUGIN_CATALOG = build_cadac_plugin_catalog()


__all__ = [
    "CADAC_PLUGIN_CATALOG",
    "CadacPluginCatalog",
    "CadacPluginOperation",
    "CadacPluginRegistry",
    "CadacPluginScope",
    "CadacPluginStatus",
    "CadacVehiclePlugin",
    "CadacVehiclePluginDescriptor",
    "build_cadac_plugin_catalog",
]
