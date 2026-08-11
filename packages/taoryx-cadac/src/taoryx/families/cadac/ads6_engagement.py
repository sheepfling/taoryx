"""Source-ordered ADS6 multi-actor engagement composition runtime."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Generator, Literal

import numpy as np
from pydantic import Field, model_validator

from .ads6_aircraft import (
    Ads6AircraftActorRuntime,
    Ads6AircraftSample,
    Ads6AircraftSourceDefinition,
    Ads6AircraftThreatTrack,
    lower_ads6_aircraft_actor,
)
from .ads6_radar import (
    Ads6RadarLaunchCommand,
    Ads6RadarMissileTruth,
    Ads6RadarRuntime,
    Ads6RadarSample,
    Ads6RadarSourceDefinition,
    Ads6RadarTargetTruth,
    Ads6RadarTrackRecord,
    lower_ads6_radar_actor,
)
from .ads6_sam import (
    Ads6SamControlCommand,
    Ads6SamDirectCommand,
    Ads6SamPhase,
    Ads6SamPlantStepper,
    Ads6SamSample,
    Ads6SamSourceDefinition,
    lower_ads6_sam_actor,
)
from .ads6_sam_controller import (
    Ads6SamControllerContext,
    Ads6SamControllerDefinition,
    Ads6SamControllerEventTrace,
    Ads6SamControllerSample,
    Ads6SamSourceController,
    lower_ads6_sam_controller_actor,
)
from .ads6_srbm import (
    Ads6SrbmActorRuntime,
    Ads6SrbmImpact,
    Ads6SrbmSample,
    Ads6SrbmSourceDefinition,
    lower_ads6_srbm_actor,
)
from .bundle import CadacSourceArtifact, CadacSourceBundle, load_cadac_source_bundle
from .input_ast import CadacModel, CadacVehicleBlock

Ads6EngagementTargetKind = Literal["aircraft", "srbm"]
Ads6EngagementActorKind = Literal["sam", "aircraft", "srbm", "radar"]
Ads6EngagementCommandLaw = Literal["hold", "line_of_sight", "source_controller"]
Ads6EngagementEventKind = Literal[
    "radar_track",
    "launch_command",
    "missile_launch",
    "target_launch",
    "target_phase",
    "target_impact",
    "sam_ground_impact",
    "sam_controller_event",
    "sam_mode_transition",
    "seeker_lock",
    "nonfinite_state",
]

_SMALL = 1.0e-9
_DEFAULT_HOLD_TIME_S = 9_999.0
_MODEL_IDS = {
    "sam": "cadac.ads6.sam",
    "aircraft": "cadac.ads6.aircraft",
    "srbm": "cadac.ads6.srbm",
    "radar": "cadac.ads6.radar",
}
_FIDELITIES = {
    "aircraft": "point_mass_3dof",
    "srbm": "pseudo_6dof",
    "radar": "static_sensor",
}


class Ads6EngagementSourceError(ValueError):
    """Source-bundle incompatibility with the ADS6 package scheduler."""


####


class Ads6EngagementActorBinding(CadacModel):
    """One source-order actor identity in an ADS6 package case."""

    actor_id: str = Field(min_length=1)
    actor_kind: Ads6EngagementActorKind
    model_id: str = Field(min_length=1)
    source_model: str = Field(min_length=1)
    source_role: str = Field(min_length=1)
    source_index: int = Field(ge=0)
    fidelity: str = Field(min_length=1)


####


class Ads6EngagementSourceDefinition(CadacModel):
    """Prepared ADS6 package case with independently typed actors."""

    source_name: str = Field(min_length=1)
    target_kind: Ads6EngagementTargetKind
    integration_step_s: float = Field(gt=0.0)
    trajectory_step_s: float | None = Field(default=None, gt=0.0)
    end_time_s: float = Field(gt=0.0)
    source_order: tuple[Ads6EngagementActorBinding, ...] = Field(min_length=3)
    sams: tuple[Ads6SamSourceDefinition, ...] = Field(min_length=1, max_length=3)
    sam_controllers: tuple[Ads6SamControllerDefinition, ...] = Field(min_length=1, max_length=3)
    aircraft_targets: tuple[Ads6AircraftSourceDefinition, ...] = ()
    srbm_targets: tuple[Ads6SrbmSourceDefinition, ...] = ()
    radar: Ads6RadarSourceDefinition
    sam_phase: Ads6SamPhase
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=1)
    claim_boundary: str = (
        "ADS6 package composition preserves source actor order, persistent actor state, immediate per-actor packet "
        "publication, RADAR0 track cadence, source-paired launch scheduling, and next-epoch missile response to radar "
        "commands. Launch scheduling follows the documented RADAR0 latched-command contract rather than reproducing "
        "the apparent unconditional launch-delay overwrite in the shipped executive. Standalone SAM/SRBM/AIRCRAFT3 "
        "physical plants participate directly. The source-controller path interleaves truth-aligned INS, deterministic "
        "RF/IR seeker states, radar-IP line guidance, terminal proportional navigation, and adaptive SAM autopilot "
        "modules with the physical plant. RF glint/thermal noise, complete IR focal-plane corruption, exact stochastic "
        "sequences, and compiled-CADAC closed-loop parity remain outside the promoted claim."
    )

    @model_validator(mode="after")
    def validate_actor_composition(self) -> "Ads6EngagementSourceDefinition":
        target_count = len(self.aircraft_targets) if self.target_kind == "aircraft" else len(self.srbm_targets)
        if target_count != len(self.sams):
            raise ValueError("ADS6 package composition requires one source-order target per SAM")
        ####
        if len(self.sam_controllers) != len(self.sams):
            raise ValueError("ADS6 package composition requires one source controller per SAM")
        ####
        if self.target_kind == "aircraft" and self.srbm_targets:
            raise ValueError("ADS6 aircraft-defense case cannot include SRBM targets")
        ####
        if self.target_kind == "srbm" and self.aircraft_targets:
            raise ValueError("ADS6 missile-defense case cannot include aircraft targets")
        ####
        if self.radar.track_mode != self.target_kind:
            raise ValueError("ADS6 RADAR0 tracking mode must match the package target kind")
        ####
        return self

    ####


####


class Ads6EngagementRunConfig(CadacModel):
    """Batch controls for the package scheduler and selected SAM controller path."""

    end_time_s: float | None = Field(default=None, gt=0.0)
    sample_step_s: float | None = Field(default=None, gt=0.0)
    command_law: Ads6EngagementCommandLaw = "source_controller"
    pointing_gain: float = Field(default=0.35, ge=0.0)
    command_limit_deg: float = Field(default=20.0, gt=0.0)
    radar_seed: int = 0


####


class Ads6EngagementPacketTrace(CadacModel):
    """One actor pass with the packet epochs it consumed and published."""

    time_s: float = Field(ge=0.0)
    actor_id: str = Field(min_length=1)
    actor_kind: Ads6EngagementActorKind
    active: bool
    observed_packet_epochs: tuple[tuple[str, float], ...] = ()
    published_epoch_s: float = Field(ge=0.0)


####


class Ads6EngagementEvent(CadacModel):
    """One package scheduling, tracking, phase, or termination event."""

    time_s: float = Field(ge=0.0)
    kind: Ads6EngagementEventKind
    actor_id: str = Field(min_length=1)
    related_actor_id: str | None = None
    details: dict[str, object] = Field(default_factory=dict)


####


class Ads6EngagementSample(CadacModel):
    """Common multi-fidelity sample emitted by one package actor."""

    time_s: float = Field(ge=0.0)
    actor_id: str = Field(min_length=1)
    actor_kind: Ads6EngagementActorKind
    model_id: str = Field(min_length=1)
    fidelity: str = Field(min_length=1)
    source_phase: str = Field(min_length=1)
    held: bool
    alive: bool
    position_ned_m: tuple[float, float, float]
    velocity_ned_mps: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float] | None = None
    body_rates_rad_s: tuple[float, float, float] | None = None
    telemetry: dict[str, object] = Field(default_factory=dict)


####


class Ads6EngagementObjectTrace(CadacModel):
    """One independently initialized root object and its accepted samples."""

    actor_id: str = Field(min_length=1)
    actor_kind: Ads6EngagementActorKind
    model_id: str = Field(min_length=1)
    source_role: str = Field(min_length=1)
    fidelity: str = Field(min_length=1)
    samples: tuple[Ads6EngagementSample, ...] = Field(min_length=1)


####


class Ads6EngagementControllerTrace(CadacModel):
    """One SAM source-controller history alongside the physical actor trace."""

    actor_id: str = Field(min_length=1)
    samples: tuple[Ads6SamControllerSample, ...] = ()
    source_events: tuple[Ads6SamControllerEventTrace, ...] = ()


####


class Ads6EngagementRunResult(CadacModel):
    """Source-ordered mixed-fidelity ADS6 package result."""

    schema_id: str = "taoryx.cadac.ads6-engagement-run/v0alpha2"
    source_name: str
    target_kind: Ads6EngagementTargetKind
    sam_phase: Ads6SamPhase
    integration_step_s: float = Field(gt=0.0)
    requested_end_time_s: float = Field(gt=0.0)
    executed_epochs: int = Field(ge=0)
    terminated_reason: str
    primary_actor_id: str
    source_order: tuple[str, ...]
    objects: tuple[Ads6EngagementObjectTrace, ...] = Field(min_length=3)
    events: tuple[Ads6EngagementEvent, ...] = ()
    packet_traces: tuple[Ads6EngagementPacketTrace, ...] = ()
    radar_tracks: tuple[Ads6RadarTrackRecord, ...] = ()
    radar_launch_commands: tuple[Ads6RadarLaunchCommand, ...] = ()
    sam_controller_traces: tuple[Ads6EngagementControllerTrace, ...] = ()
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=1)
    claim_boundary: str


####


@dataclass(slots=True)
class _Packet:
    actor_id: str
    epoch_s: float
    position_ned_m: tuple[float, float, float]
    velocity_ned_mps: tuple[float, float, float]
    alive: bool = True


####


@dataclass(frozen=True, slots=True)
class _Ads6EngagementEpoch:
    """One committed vehicle-major ADS6 package epoch for a persistent owner."""

    time_s: float
    sequence: int
    packets: tuple[_Packet, ...]
    sam_samples: tuple[Ads6SamSample, ...]
    controller_samples: tuple[Ads6SamControllerSample, ...]
    events: tuple[Ads6EngagementEvent, ...]
    terminal: bool
    terminated_reason: str | None


####


class Ads6EngagementSession:
    """Persistent ADS6 package scheduler backed by the exact source epoch loop."""

    def __init__(
        self,
        definition: Ads6EngagementSourceDefinition,
        config: Ads6EngagementRunConfig | None = None,
    ) -> None:
        self.definition = definition
        self.config = config or Ads6EngagementRunConfig()
        self.reset()
    ####

    def reset(self) -> None:
        """Reconstruct every actor, controller, radar, packet, and event state."""

        self._epochs = _run_ads6_engagement_epochs(self.definition, self.config)
        self._packets = _initial_engagement_packets(self.definition)
        self.last_epoch: _Ads6EngagementEpoch | None = None
        self.run_result: Ads6EngagementRunResult | None = None
        self.sim_time_s = 0.0
        self.executed_epochs = 0
        self.terminated_reason: str | None = None
    ####

    @property
    def completed(self) -> bool:
        """Return whether the source package reached a terminal epoch."""

        return self.run_result is not None
    ####

    @property
    def packets(self) -> tuple[_Packet, ...]:
        """Return current committed source packets in deterministic actor order."""

        return tuple(self._packets[item.actor_id] for item in self.definition.source_order)
    ####

    def packet(self, actor_id: str) -> _Packet:
        """Return one committed source packet by source actor identity."""

        return self._packets[actor_id]
    ####

    def advance(self, duration_s: float) -> tuple[_Ads6EngagementEpoch, ...]:
        """Advance exact source epochs without restarting the package scheduler."""

        steps = _ads6_engagement_session_step_count(duration_s, self.definition.integration_step_s)
        committed: list[_Ads6EngagementEpoch] = []
        for _ in range(steps):
            if self.completed:
                break
            ####
            try:
                epoch = next(self._epochs)
            except StopIteration as stop:
                result = stop.value
                if not isinstance(result, Ads6EngagementRunResult):
                    raise AssertionError("ADS6 package epoch loop completed without a run result")
                self.run_result = result
                self.terminated_reason = result.terminated_reason
                break
            ####
            self.last_epoch = epoch
            self._packets = {packet.actor_id: packet for packet in epoch.packets}
            self.executed_epochs = epoch.sequence
            self.sim_time_s += self.definition.integration_step_s
            committed.append(epoch)
            if epoch.terminal:
                try:
                    next(self._epochs)
                except StopIteration as stop:
                    result = stop.value
                    if not isinstance(result, Ads6EngagementRunResult):
                        raise AssertionError("ADS6 package terminal epoch completed without a run result")
                    self.run_result = result
                    self.terminated_reason = result.terminated_reason
                else:
                    raise AssertionError("ADS6 package emitted an epoch after marking itself terminal")
                break
            ####
        ####
        return tuple(committed)
    ####


####


def _ads6_engagement_session_step_count(duration_s: float, source_step_s: float) -> int:
    """Validate a caller hold against the exact ADS6 package timestep."""

    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("ADS6 engagement session duration_s must be positive and finite")
    ####
    steps = round(duration_s / source_step_s)
    tolerance_s = max(1.0e-12, source_step_s * 1.0e-9)
    if steps <= 0 or not math.isclose(duration_s, steps * source_step_s, rel_tol=0.0, abs_tol=tolerance_s):
        raise ValueError(f"ADS6 engagement session duration_s must be an integral multiple of source step {source_step_s:.17g} s")
    ####
    return steps


####


def _initial_engagement_packets(definition: Ads6EngagementSourceDefinition) -> dict[str, _Packet]:
    """Construct the unadvanced source packet snapshot for a new session."""

    sam_runtimes = tuple(Ads6SamPlantStepper(item) for item in definition.sams)
    aircraft_runtimes = tuple(Ads6AircraftActorRuntime(item) for item in definition.aircraft_targets)
    srbm_runtimes = tuple(Ads6SrbmActorRuntime(item) for item in definition.srbm_targets)
    radar_binding = next(item for item in definition.source_order if item.actor_kind == "radar")
    return _initial_packets(definition, sam_runtimes, aircraft_runtimes, srbm_runtimes, radar_binding)


####


def load_ads6_engagement_source_definition(path: str | Path) -> Ads6EngagementSourceDefinition:
    """Parse and lower one mixed-actor ADS6 source engagement case."""

    return lower_ads6_engagement_source_bundle(load_cadac_source_bundle(path))


####


def lower_ads6_engagement_source_bundle(bundle: CadacSourceBundle) -> Ads6EngagementSourceDefinition:
    """Lower one source-ordered ADS6 SAM/target/RADAR0 package case."""

    case = bundle.case
    missiles = case.vehicles_named("MISSILE6")
    aircraft = case.vehicles_named("AIRCRAFT3")
    rockets = case.vehicles_named("ROCKET5")
    radars = case.vehicles_named("RADAR0")
    if not 1 <= len(missiles) <= 3:
        raise Ads6EngagementSourceError(f"ADS6 package requires one to three MISSILE6 actors; found {len(missiles)}")
    ####
    if len(radars) != 1:
        raise Ads6EngagementSourceError(f"ADS6 package requires exactly one RADAR0 actor; found {len(radars)}")
    ####
    radar = lower_ads6_radar_actor(bundle, radars[0])
    if radar.track_mode == "aircraft":
        if rockets or len(aircraft) != len(missiles):
            raise Ads6EngagementSourceError("ADS6 aircraft-defense package requires matching MISSILE6/AIRCRAFT3 counts and no ROCKET5 actors")
        ####
        target_kind: Ads6EngagementTargetKind = "aircraft"
    else:
        if aircraft or len(rockets) != len(missiles):
            raise Ads6EngagementSourceError("ADS6 missile-defense package requires matching MISSILE6/ROCKET5 counts and no AIRCRAFT3 actors")
        ####
        target_kind = "srbm"
    ####
    allowed = {"missile6", "aircraft3", "rocket5", "radar0"}
    unsupported = tuple(vehicle.model_name for vehicle in case.vehicles if vehicle.model_name.casefold() not in allowed)
    if unsupported:
        raise Ads6EngagementSourceError(f"ADS6 package contains unsupported actor models: {unsupported!r}")
    ####
    sam_definitions = tuple(lower_ads6_sam_actor(bundle, vehicle) for vehicle in missiles)
    aircraft_definitions = tuple(lower_ads6_aircraft_actor(bundle, vehicle) for vehicle in aircraft)
    srbm_definitions = tuple(lower_ads6_srbm_actor(bundle, vehicle) for vehicle in rockets)
    sam_phases = tuple(_infer_sam_phase(definition) for definition in sam_definitions)
    if len(set(sam_phases)) != 1:
        raise Ads6EngagementSourceError(f"one ADS6 engagement executor requires a common SAM realization; found {sam_phases!r}")
    ####
    sam_controllers = tuple(
        lower_ads6_sam_controller_actor(
            bundle,
            vehicle,
            plant,
            selected_phase=sam_phases[0],
        )
        for vehicle, plant in zip(missiles, sam_definitions, strict=True)
    )
    source_order = _build_source_order(case.vehicles, sam_phases[0])
    radar_index = next(item.source_index for item in source_order if item.actor_kind == "radar")
    if any(item.source_index > radar_index for item in source_order if item.actor_kind == "sam"):
        raise Ads6EngagementSourceError("ADS6 source scheduler requires all MISSILE6 actors to precede RADAR0")
    ####
    try:
        integration_step_s = case.timing_values["int_step"]
    except KeyError as error:
        raise Ads6EngagementSourceError("ADS6 package source case is missing TIMING int_step") from error
    ####
    return Ads6EngagementSourceDefinition(
        source_name=case.source_name,
        target_kind=target_kind,
        integration_step_s=integration_step_s,
        trajectory_step_s=case.timing_values.get("traj_step"),
        end_time_s=case.end_time_s,
        source_order=source_order,
        sams=sam_definitions,
        sam_controllers=sam_controllers,
        aircraft_targets=aircraft_definitions,
        srbm_targets=srbm_definitions,
        radar=radar,
        sam_phase=sam_phases[0],
        source_artifacts=bundle.artifacts,
    )


####


def run_ads6_engagement(
    definition: Ads6EngagementSourceDefinition,
    config: Ads6EngagementRunConfig | None = None,
) -> Ads6EngagementRunResult:
    """Execute persistent ADS6 actors in exact source vehicle order."""

    epochs = _run_ads6_engagement_epochs(definition, config)
    while True:
        try:
            next(epochs)
        except StopIteration as stop:
            result = stop.value
            if not isinstance(result, Ads6EngagementRunResult):
                raise AssertionError("ADS6 package epoch loop completed without a run result")
            return result
        ####


####


def _run_ads6_engagement_epochs(
    definition: Ads6EngagementSourceDefinition,
    config: Ads6EngagementRunConfig | None = None,
) -> Generator[_Ads6EngagementEpoch, None, Ads6EngagementRunResult]:
    """Yield exact committed ADS6 package epochs and return the batch result."""

    options = config or Ads6EngagementRunConfig()
    dt_s = definition.integration_step_s
    requested_end = definition.end_time_s if options.end_time_s is None else options.end_time_s
    cadence = definition.trajectory_step_s or dt_s if options.sample_step_s is None else options.sample_step_s
    if requested_end is None or cadence is None:
        raise AssertionError("validated runtime controls resolved to None")
    ####
    sam_runtimes = tuple(Ads6SamPlantStepper(item) for item in definition.sams)
    sam_controllers = tuple(Ads6SamSourceController(controller, plant) for controller, plant in zip(definition.sam_controllers, definition.sams, strict=True))
    aircraft_runtimes = tuple(Ads6AircraftActorRuntime(item) for item in definition.aircraft_targets)
    srbm_runtimes = tuple(Ads6SrbmActorRuntime(item) for item in definition.srbm_targets)
    radar_runtime = Ads6RadarRuntime(definition.radar, seed=options.radar_seed)
    actor_by_id = {item.actor_id: item for item in definition.source_order}
    sam_bindings = tuple(item for item in definition.source_order if item.actor_kind == "sam")
    target_bindings = tuple(item for item in definition.source_order if item.actor_kind in {"aircraft", "srbm"})
    radar_binding = next(item for item in definition.source_order if item.actor_kind == "radar")
    packets = _initial_packets(definition, sam_runtimes, aircraft_runtimes, srbm_runtimes, radar_binding)
    traces: dict[str, list[Ads6EngagementSample]] = {item.actor_id: [] for item in definition.source_order}
    events: list[Ads6EngagementEvent] = []
    packet_traces: list[Ads6EngagementPacketTrace] = []
    radar_tracks: list[Ads6RadarTrackRecord] = []
    launch_commands: list[Ads6RadarLaunchCommand] = []
    sam_launch_times = [item.launch_delay_s for item in definition.sams]
    sam_launched = [False for _ in definition.sams]
    target_launched = [False for _ in target_bindings]
    target_previous_phase = ["prelaunch" for _ in target_bindings]
    controller_samples = [controller.sample() for controller in sam_controllers]
    controller_histories: list[list[Ads6SamControllerSample]] = [[] for _ in sam_controllers]
    controller_event_counts = [0 for _ in sam_controllers]
    alive = {item.actor_id: True for item in definition.source_order}
    sim_time = 0.0
    next_sample_time = 0.0
    epochs = 0
    terminated_reason = "end_time"
    while sim_time <= requested_end + 0.5 * dt_s:
        epochs += 1
        event_start = len(events)
        for binding in definition.source_order:
            if binding.actor_kind == "sam":
                index = int(binding.actor_id[1:]) - 1
                target_binding = target_bindings[index]
                radar_packet = packets[radar_binding.actor_id]
                target_packet = packets[target_binding.actor_id]
                observed = (
                    (target_packet.actor_id, target_packet.epoch_s),
                    (radar_packet.actor_id, radar_packet.epoch_s),
                )
                active = alive[binding.actor_id] and sim_time + 0.5 * dt_s >= sam_launch_times[index]
                if active:
                    if not sam_launched[index]:
                        sam_launched[index] = True
                        events.append(
                            Ads6EngagementEvent(
                                time_s=sim_time,
                                kind="missile_launch",
                                actor_id=binding.actor_id,
                                related_actor_id=target_binding.actor_id,
                                details={"scheduled_launch_time_s": sam_launch_times[index]},
                            )
                        )
                    ####
                    previous_controller_sample = controller_samples[index]
                    if options.command_law == "source_controller":
                        context = Ads6SamControllerContext(
                            missile_index=index + 1,
                            target_actor_id=target_packet.actor_id,
                            target_kind=definition.target_kind,
                            target_position_ned_m=target_packet.position_ned_m,
                            target_velocity_ned_mps=target_packet.velocity_ned_mps,
                            target_packet_epoch_s=target_packet.epoch_s,
                            intercept_point_ned_m=radar_runtime.intercept_points_ned_m[index],
                            radar_packet_epoch_s=radar_packet.epoch_s,
                        )
                        sam_runtimes[index].step_controlled(
                            sim_time - sam_launch_times[index],
                            sim_time,
                            sam_controllers[index],
                            context,
                        )
                        controller_samples[index] = sam_controllers[index].sample()
                        controller_histories[index].append(controller_samples[index])
                        _append_sam_controller_events(
                            events,
                            binding.actor_id,
                            target_binding.actor_id,
                            previous_controller_sample,
                            controller_samples[index],
                            sam_controllers[index].event_traces,
                            controller_event_counts[index],
                        )
                        controller_event_counts[index] = len(sam_controllers[index].event_traces)
                    else:
                        command = _sam_command(
                            definition.sam_phase,
                            options,
                            sam_runtimes[index].sample(sim_time),
                            target_packet,
                            radar_runtime.intercept_points_ned_m[index],
                        )
                        sam_runtimes[index].step(sim_time - sam_launch_times[index], command)
                    ####
                    sam_sample = sam_runtimes[index].sample(sim_time)
                    packets[binding.actor_id] = _Packet(
                        actor_id=binding.actor_id,
                        epoch_s=sim_time,
                        position_ned_m=sam_sample.position_ned_m,
                        velocity_ned_mps=sam_sample.velocity_ned_mps,
                        alive=alive[binding.actor_id],
                    )
                    if sam_runtimes[index].altitude_m < 0.0 and sim_time > sam_launch_times[index]:
                        alive[binding.actor_id] = False
                        packets[binding.actor_id].alive = False
                        events.append(
                            Ads6EngagementEvent(
                                time_s=sim_time,
                                kind="sam_ground_impact",
                                actor_id=binding.actor_id,
                                related_actor_id=target_binding.actor_id,
                            )
                        )
                    ####
                    if not sam_runtimes[index].finite:
                        alive[binding.actor_id] = False
                        packets[binding.actor_id].alive = False
                        events.append(
                            Ads6EngagementEvent(
                                time_s=sim_time,
                                kind="nonfinite_state",
                                actor_id=binding.actor_id,
                            )
                        )
                    ####
                ####
                packet_traces.append(
                    Ads6EngagementPacketTrace(
                        time_s=sim_time,
                        actor_id=binding.actor_id,
                        actor_kind="sam",
                        active=active,
                        observed_packet_epochs=observed,
                        published_epoch_s=packets[binding.actor_id].epoch_s,
                    )
                )
            elif binding.actor_kind == "aircraft":
                index = int(binding.actor_id[1:]) - 1
                runtime = aircraft_runtimes[index]
                active = alive[binding.actor_id] and sim_time + 0.5 * dt_s >= runtime.definition.launch_delay_s
                observed: tuple[tuple[str, float], ...] = ()
                threat_track: Ads6AircraftThreatTrack | None = None
                if runtime.definition.guidance.option == 2:
                    threat = packets[sam_bindings[0].actor_id]
                    observed = ((threat.actor_id, threat.epoch_s),)
                    threat_track = Ads6AircraftThreatTrack(
                        position_ned_m=threat.position_ned_m,
                        velocity_ned_mps=threat.velocity_ned_mps,
                        reference_time_s=threat.epoch_s,
                    )
                ####
                if active:
                    if not target_launched[index]:
                        target_launched[index] = True
                        events.append(
                            Ads6EngagementEvent(
                                time_s=sim_time,
                                kind="target_launch",
                                actor_id=binding.actor_id,
                            )
                        )
                    ####
                    runtime.step(sim_time, threat_track=threat_track)
                    target_sample = runtime.sample(sim_time)
                    packets[binding.actor_id] = _Packet(
                        actor_id=binding.actor_id,
                        epoch_s=sim_time,
                        position_ned_m=target_sample.position_ned_m,
                        velocity_ned_mps=target_sample.velocity_ned_mps,
                        alive=alive[binding.actor_id],
                    )
                    if not runtime.finite:
                        alive[binding.actor_id] = False
                        packets[binding.actor_id].alive = False
                        events.append(Ads6EngagementEvent(time_s=sim_time, kind="nonfinite_state", actor_id=binding.actor_id))
                    ####
                ####
                packet_traces.append(
                    Ads6EngagementPacketTrace(
                        time_s=sim_time,
                        actor_id=binding.actor_id,
                        actor_kind="aircraft",
                        active=active,
                        observed_packet_epochs=observed,
                        published_epoch_s=packets[binding.actor_id].epoch_s,
                    )
                )
            elif binding.actor_kind == "srbm":
                index = int(binding.actor_id[1:]) - 1
                runtime = srbm_runtimes[index]
                launch_delay = runtime.definition.launch_delay_s
                active = alive[binding.actor_id] and sim_time + 0.5 * dt_s >= launch_delay
                if active:
                    if not target_launched[index]:
                        target_launched[index] = True
                        events.append(
                            Ads6EngagementEvent(
                                time_s=sim_time,
                                kind="target_launch",
                                actor_id=binding.actor_id,
                            )
                        )
                    ####
                    phase = runtime.step(sim_time - launch_delay)
                    target_sample = runtime.sample(sim_time)
                    packets[binding.actor_id] = _Packet(
                        actor_id=binding.actor_id,
                        epoch_s=sim_time,
                        position_ned_m=target_sample.position_ned_m,
                        velocity_ned_mps=target_sample.velocity_ned_mps,
                        alive=alive[binding.actor_id],
                    )
                    if phase != target_previous_phase[index]:
                        events.append(
                            Ads6EngagementEvent(
                                time_s=sim_time,
                                kind="target_phase",
                                actor_id=binding.actor_id,
                                details={"previous_phase": target_previous_phase[index], "phase": phase},
                            )
                        )
                        target_previous_phase[index] = phase
                    ####
                    impact = runtime.impact(sim_time)
                    if impact is not None:
                        alive[binding.actor_id] = False
                        packets[binding.actor_id].alive = False
                        events.append(_impact_event(binding.actor_id, sim_time, impact))
                    ####
                    if not runtime.finite:
                        alive[binding.actor_id] = False
                        packets[binding.actor_id].alive = False
                        events.append(Ads6EngagementEvent(time_s=sim_time, kind="nonfinite_state", actor_id=binding.actor_id))
                    ####
                ####
                packet_traces.append(
                    Ads6EngagementPacketTrace(
                        time_s=sim_time,
                        actor_id=binding.actor_id,
                        actor_kind="srbm",
                        active=active,
                        published_epoch_s=packets[binding.actor_id].epoch_s,
                    )
                )
            elif binding.actor_kind == "radar":
                target_truth = tuple(
                    Ads6RadarTargetTruth(
                        actor_id=item.actor_id,
                        position_ned_m=packets[item.actor_id].position_ned_m,
                        velocity_ned_mps=packets[item.actor_id].velocity_ned_mps,
                    )
                    for item in target_bindings
                )
                missile_truth = tuple(
                    Ads6RadarMissileTruth(
                        actor_id=item.actor_id,
                        position_ned_m=packets[item.actor_id].position_ned_m,
                        velocity_ned_mps=packets[item.actor_id].velocity_ned_mps,
                        launched=sam_launched[index],
                    )
                    for index, item in enumerate(sam_bindings)
                )
                radar_step = radar_runtime.step(sim_time, target_truth, missiles=missile_truth)
                observed = tuple((item.actor_id, packets[item.actor_id].epoch_s) for item in (*sam_bindings, *target_bindings))
                if radar_step.tracked:
                    packets[binding.actor_id].epoch_s = sim_time
                    radar_tracks.extend(radar_step.track_records)
                    launch_commands.extend(radar_step.launch_commands)
                    events.append(
                        Ads6EngagementEvent(
                            time_s=sim_time,
                            kind="radar_track",
                            actor_id=binding.actor_id,
                            details={"track_count": len(radar_step.track_records)},
                        )
                    )
                    for command in radar_step.launch_commands:
                        sam_launch_times[command.missile_index - 1] = command.launch_time_s
                        if command.newly_latched:
                            events.append(
                                Ads6EngagementEvent(
                                    time_s=sim_time,
                                    kind="launch_command",
                                    actor_id=binding.actor_id,
                                    related_actor_id=sam_bindings[command.missile_index - 1].actor_id,
                                    details={
                                        "target_actor_id": target_bindings[command.target_index - 1].actor_id,
                                        "launch_time_s": command.launch_time_s,
                                        "intercept_point_ned_m": command.intercept_point_ned_m,
                                        "reason": command.reason,
                                    },
                                )
                            )
                        ####
                    ####
                ####
                packet_traces.append(
                    Ads6EngagementPacketTrace(
                        time_s=sim_time,
                        actor_id=binding.actor_id,
                        actor_kind="radar",
                        active=True,
                        observed_packet_epochs=observed,
                        published_epoch_s=packets[binding.actor_id].epoch_s,
                    )
                )
            else:
                raise AssertionError(f"unhandled ADS6 actor kind {binding.actor_kind!r}")
            ####
        ####
        if sim_time + 0.5 * dt_s >= next_sample_time or sim_time + 0.5 * dt_s >= requested_end:
            _append_samples(
                traces,
                definition,
                sim_time,
                alive,
                sam_launch_times,
                sam_launched,
                target_launched,
                sam_runtimes,
                tuple(controller_samples),
                options.command_law,
                aircraft_runtimes,
                srbm_runtimes,
                radar_runtime,
                actor_by_id,
            )
            while next_sample_time <= sim_time + 0.5 * dt_s:
                next_sample_time += cadence
            ####
        ####
        no_dynamic_actors_alive = not any(alive[item.actor_id] for item in (*sam_bindings, *target_bindings))
        if no_dynamic_actors_alive:
            terminated_reason = "all_dynamic_actors_terminated"
        ####
        final_epoch = no_dynamic_actors_alive or sim_time + dt_s > requested_end + 0.5 * dt_s
        yield _Ads6EngagementEpoch(
            time_s=sim_time,
            sequence=epochs,
            packets=tuple(
                _Packet(
                    actor_id=packet.actor_id,
                    epoch_s=packet.epoch_s,
                    position_ned_m=packet.position_ned_m,
                    velocity_ned_mps=packet.velocity_ned_mps,
                    alive=packet.alive,
                )
                for packet in (packets[item.actor_id] for item in definition.source_order)
            ),
            sam_samples=tuple(runtime.sample(sim_time) for runtime in sam_runtimes),
            controller_samples=tuple(controller_samples),
            events=tuple(events[event_start:]),
            terminal=final_epoch,
            terminated_reason=terminated_reason if final_epoch else None,
        )
        if no_dynamic_actors_alive:
            break
        ####
        sim_time += dt_s
    ####
    objects = tuple(
        Ads6EngagementObjectTrace(
            actor_id=binding.actor_id,
            actor_kind=binding.actor_kind,
            model_id=binding.model_id,
            source_role=binding.source_role,
            fidelity=binding.fidelity,
            samples=tuple(traces[binding.actor_id]),
        )
        for binding in definition.source_order
    )
    controller_traces = tuple(
        Ads6EngagementControllerTrace(
            actor_id=binding.actor_id,
            samples=tuple(controller_histories[index]),
            source_events=sam_controllers[index].event_traces,
        )
        for index, binding in enumerate(sam_bindings)
        if options.command_law == "source_controller"
    )
    return Ads6EngagementRunResult(
        source_name=definition.source_name,
        target_kind=definition.target_kind,
        sam_phase=definition.sam_phase,
        integration_step_s=dt_s,
        requested_end_time_s=requested_end,
        executed_epochs=epochs,
        terminated_reason=terminated_reason,
        primary_actor_id=sam_bindings[0].actor_id,
        source_order=tuple(item.actor_id for item in definition.source_order),
        objects=objects,
        events=tuple(events),
        packet_traces=tuple(packet_traces),
        radar_tracks=tuple(radar_tracks),
        radar_launch_commands=tuple(launch_commands),
        sam_controller_traces=controller_traces,
        source_artifacts=definition.source_artifacts,
        claim_boundary=definition.claim_boundary,
    )


####


def _build_source_order(
    vehicles: tuple[CadacVehicleBlock, ...],
    sam_phase: Ads6SamPhase,
) -> tuple[Ads6EngagementActorBinding, ...]:
    counters = {"missile6": 0, "aircraft3": 0, "rocket5": 0, "radar0": 0}
    result: list[Ads6EngagementActorBinding] = []
    for source_index, vehicle in enumerate(vehicles):
        key = vehicle.model_name.casefold()
        counters[key] += 1
        if key == "missile6":
            kind: Ads6EngagementActorKind = "sam"
            actor_id = f"m{counters[key]}"
            fidelity = _sam_fidelity(sam_phase)
        elif key == "aircraft3":
            kind = "aircraft"
            actor_id = f"a{counters[key]}"
            fidelity = _FIDELITIES[kind]
        elif key == "rocket5":
            kind = "srbm"
            actor_id = f"r{counters[key]}"
            fidelity = _FIDELITIES[kind]
        elif key == "radar0":
            kind = "radar"
            actor_id = f"f{counters[key]}"
            fidelity = _FIDELITIES[kind]
        else:
            raise Ads6EngagementSourceError(f"unsupported ADS6 actor model {vehicle.model_name!r}")
        ####
        result.append(
            Ads6EngagementActorBinding(
                actor_id=actor_id,
                actor_kind=kind,
                model_id=_MODEL_IDS[kind],
                source_model=vehicle.model_name,
                source_role=vehicle.role,
                source_index=source_index,
                fidelity=fidelity,
            )
        )
    ####
    return tuple(result)


####


def _infer_sam_phase(definition: Ads6SamSourceDefinition) -> Ads6SamPhase:
    if definition.tvc.mode > 0:
        return "tvc_control"
    ####
    if definition.rcs.moment_mode > 0 or definition.rcs.force_mode > 0:
        return "aggregate_rcs"
    ####
    return "fin_control"


####


def _sam_fidelity(phase: Ads6SamPhase) -> str:
    return "rigid_body_6dof_direct_wrench" if phase == "aggregate_rcs" else "rigid_body_6dof_surface_allocated"


####


def _initial_packets(
    definition: Ads6EngagementSourceDefinition,
    sams: tuple[Ads6SamPlantStepper, ...],
    aircraft: tuple[Ads6AircraftActorRuntime, ...],
    srbms: tuple[Ads6SrbmActorRuntime, ...],
    radar: Ads6EngagementActorBinding,
) -> dict[str, _Packet]:
    packets: dict[str, _Packet] = {}
    for index, runtime in enumerate(sams, start=1):
        packets[f"m{index}"] = _Packet(
            actor_id=f"m{index}",
            epoch_s=0.0,
            position_ned_m=runtime.position_ned_m,
            velocity_ned_mps=runtime.velocity_ned_mps,
        )
    ####
    for index, runtime in enumerate(aircraft, start=1):
        packets[f"a{index}"] = _Packet(
            actor_id=f"a{index}",
            epoch_s=0.0,
            position_ned_m=runtime.position_ned_m,
            velocity_ned_mps=runtime.velocity_ned_mps,
        )
    ####
    for index, runtime in enumerate(srbms, start=1):
        packets[f"r{index}"] = _Packet(
            actor_id=f"r{index}",
            epoch_s=0.0,
            position_ned_m=runtime.position_ned_m,
            velocity_ned_mps=runtime.velocity_ned_mps,
        )
    ####
    packets[radar.actor_id] = _Packet(
        actor_id=radar.actor_id,
        epoch_s=0.0,
        position_ned_m=definition.radar.position_ned_m,
        velocity_ned_mps=(0.0, 0.0, 0.0),
    )
    return packets


####


def _sam_command(
    phase: Ads6SamPhase,
    options: Ads6EngagementRunConfig,
    sam: Ads6SamSample,
    target: _Packet,
    radar_intercept_point_ned_m: tuple[float, float, float],
) -> Ads6SamDirectCommand:
    if options.command_law == "hold":
        return Ads6SamDirectCommand(phase=phase)
    ####
    aimpoint = np.asarray(radar_intercept_point_ned_m, dtype=np.float64)
    if float(np.linalg.norm(aimpoint)) <= _SMALL:
        aimpoint = np.asarray(target.position_ned_m, dtype=np.float64)
    ####
    displacement = aimpoint - np.asarray(sam.position_ned_m, dtype=np.float64)
    horizontal = math.hypot(float(displacement[0]), float(displacement[1]))
    desired_yaw = math.degrees(math.atan2(float(displacement[1]), float(displacement[0])))
    desired_pitch = math.degrees(math.atan2(-float(displacement[2]), horizontal))
    current_yaw, current_pitch, _ = sam.body_angles_deg
    yaw_error = _wrap_degrees(desired_yaw - current_yaw)
    pitch_error = desired_pitch - current_pitch
    control = Ads6SamControlCommand(
        pitch_deg=_limit(options.pointing_gain * pitch_error, options.command_limit_deg),
        yaw_deg=_limit(options.pointing_gain * yaw_error, options.command_limit_deg),
    )
    if phase == "tvc_control":
        return Ads6SamDirectCommand(phase=phase, control=control, tvc_mode=2)
    ####
    if phase == "aggregate_rcs":
        return Ads6SamDirectCommand(
            phase=phase,
            control=control,
            rcs_moment_mode=11,
            pitch_attitude_command_deg=desired_pitch,
            yaw_attitude_command_deg=desired_yaw,
        )
    ####
    return Ads6SamDirectCommand(phase=phase, control=control)


####


def _append_sam_controller_events(
    events: list[Ads6EngagementEvent],
    actor_id: str,
    target_actor_id: str,
    previous: Ads6SamControllerSample,
    current: Ads6SamControllerSample,
    traces: tuple[Ads6SamControllerEventTrace, ...],
    previous_event_count: int,
) -> None:
    """Project source event applications and controller-mode changes into package events."""

    for trace in traces[previous_event_count:]:
        events.append(
            Ads6EngagementEvent(
                time_s=trace.time_s,
                kind="sam_controller_event",
                actor_id=actor_id,
                related_actor_id=target_actor_id,
                details={
                    "missile_time_s": trace.missile_time_s,
                    "event_index": trace.event_index,
                    "source_line": trace.source_line,
                    "watch_variable": trace.watch_variable,
                    "previous_values": trace.previous_values,
                    "updated_values": trace.updated_values,
                },
            )
        )
    ####
    transitions = (
        ("control", previous.control_mode, current.control_mode),
        ("guidance", previous.guidance_mode, current.guidance_mode),
        ("sensor", previous.sensor_mode, current.sensor_mode),
    )
    for subsystem, previous_mode, current_mode in transitions:
        if previous_mode == current_mode:
            continue
        ####
        events.append(
            Ads6EngagementEvent(
                time_s=current.time_s,
                kind="sam_mode_transition",
                actor_id=actor_id,
                related_actor_id=target_actor_id,
                details={
                    "subsystem": subsystem,
                    "previous_mode": previous_mode,
                    "mode": current_mode,
                    "missile_time_s": current.missile_time_s,
                },
            )
        )
        if subsystem == "sensor" and current_mode % 10 == 4:
            events.append(
                Ads6EngagementEvent(
                    time_s=current.time_s,
                    kind="seeker_lock",
                    actor_id=actor_id,
                    related_actor_id=target_actor_id,
                    details={
                        "sensor_kind": current.sensor_kind,
                        "sensor_mode": current_mode,
                        "target_range_m": current.target_range_m,
                        "missile_time_s": current.missile_time_s,
                    },
                )
            )
        ####
    ####


####


def _append_samples(
    traces: dict[str, list[Ads6EngagementSample]],
    definition: Ads6EngagementSourceDefinition,
    sim_time_s: float,
    alive: dict[str, bool],
    sam_launch_times: list[float],
    sam_launched: list[bool],
    target_launched: list[bool],
    sams: tuple[Ads6SamPlantStepper, ...],
    sam_controllers: tuple[Ads6SamControllerSample, ...],
    command_law: Ads6EngagementCommandLaw,
    aircraft: tuple[Ads6AircraftActorRuntime, ...],
    srbms: tuple[Ads6SrbmActorRuntime, ...],
    radar: Ads6RadarRuntime,
    actor_by_id: dict[str, Ads6EngagementActorBinding],
) -> None:
    for index, runtime in enumerate(sams, start=1):
        actor_id = f"m{index}"
        traces[actor_id].append(
            _sam_engagement_sample(
                actor_by_id[actor_id],
                runtime.sample(sim_time_s),
                held=not sam_launched[index - 1],
                alive=alive[actor_id],
                controller=(sam_controllers[index - 1] if command_law == "source_controller" else None),
            )
        )
    ####
    for index, runtime in enumerate(aircraft, start=1):
        actor_id = f"a{index}"
        traces[actor_id].append(
            _aircraft_engagement_sample(
                actor_by_id[actor_id],
                runtime.sample(sim_time_s),
                held=not target_launched[index - 1],
                alive=alive[actor_id],
            )
        )
    ####
    for index, runtime in enumerate(srbms, start=1):
        actor_id = f"r{index}"
        traces[actor_id].append(
            _srbm_engagement_sample(
                actor_by_id[actor_id],
                runtime.sample(sim_time_s),
                held=not target_launched[index - 1],
                alive=alive[actor_id],
            )
        )
    ####
    radar_id = next(item.actor_id for item in definition.source_order if item.actor_kind == "radar")
    traces[radar_id].append(_radar_engagement_sample(actor_by_id[radar_id], radar.sample(sim_time_s)))


####


def _sam_engagement_sample(
    binding: Ads6EngagementActorBinding,
    sample: Ads6SamSample,
    *,
    held: bool,
    alive: bool,
    controller: Ads6SamControllerSample | None,
) -> Ads6EngagementSample:
    telemetry: dict[str, object] = {
        "altitude_m": sample.altitude_m,
        "speed_mps": sample.speed_mps,
        "mach": sample.mach,
        "dynamic_pressure_pa": sample.dynamic_pressure_pa,
        "body_angles_deg": sample.body_angles_deg,
        "requested_control_deg": sample.requested_control_deg,
        "achieved_control_deg": sample.achieved_control_deg,
        "requested_fins_deg": sample.requested_fins_deg,
        "achieved_fins_deg": sample.achieved_fins_deg,
        "achieved_lateral_acceleration_g": sample.achieved_lateral_normal_acceleration_g[0],
        "achieved_normal_acceleration_g": sample.achieved_lateral_normal_acceleration_g[1],
        "force_body_n": sample.force_body_n,
        "moment_body_nm": sample.moment_body_nm,
        "control_realization": sample.control_realization,
    }
    if controller is not None:
        telemetry.update(
            {
                "controller_mode": controller.control_mode,
                "guidance_mode": controller.guidance_mode,
                "sensor_mode": controller.sensor_mode,
                "sensor_kind": controller.sensor_kind,
                "target_actor_id": controller.target_actor_id,
                "target_packet_epoch_s": controller.target_packet_epoch_s,
                "target_range_m": controller.target_range_m,
                "closing_speed_mps": controller.closing_speed_mps,
                "pointing_pitch_yaw_rad": controller.pointing_pitch_yaw_rad,
                "los_rates_pitch_yaw_rad_s": controller.los_rates_pitch_yaw_rad_s,
                "tracking_error_pitch_yaw_rad": controller.tracking_error_pitch_yaw_rad,
                "normal_lateral_command_g": controller.normal_lateral_command_g,
                "normal_command_g": controller.normal_lateral_command_g[0],
                "lateral_command_g": controller.normal_lateral_command_g[1],
                "controller_requested_control_deg": controller.requested_control_deg,
                "radar_intercept_point_ned_m": controller.radar_intercept_point_ned_m,
                "source_controller_event_count": controller.source_event_count,
                "ins_implementation": controller.ins_implementation,
                "rf_measurement_boundary": controller.rf_measurement_boundary,
                "ir_measurement_boundary": controller.ir_measurement_boundary,
            }
        )
    ####
    return Ads6EngagementSample(
        time_s=sample.time_s,
        actor_id=binding.actor_id,
        actor_kind="sam",
        model_id=binding.model_id,
        fidelity=sample.fidelity,
        source_phase=sample.source_phase,
        held=held,
        alive=alive,
        position_ned_m=sample.position_ned_m,
        velocity_ned_mps=sample.velocity_ned_mps,
        quaternion_wxyz=sample.quaternion_wxyz,
        body_rates_rad_s=sample.body_rates_rad_s,
        telemetry=telemetry,
    )


####


def _aircraft_engagement_sample(
    binding: Ads6EngagementActorBinding,
    sample: Ads6AircraftSample,
    *,
    held: bool,
    alive: bool,
) -> Ads6EngagementSample:
    return Ads6EngagementSample(
        time_s=sample.time_s,
        actor_id=binding.actor_id,
        actor_kind="aircraft",
        model_id=binding.model_id,
        fidelity=sample.fidelity,
        source_phase=sample.mode,
        held=held,
        alive=alive,
        position_ned_m=sample.position_ned_m,
        velocity_ned_mps=sample.velocity_ned_mps,
        telemetry={
            "altitude_m": sample.altitude_m,
            "speed_mps": sample.speed_mps,
            "heading_deg": sample.heading_deg,
            "flight_path_deg": sample.flight_path_deg,
            "bank_deg": sample.bank_deg,
            "normal_load_factor_g": sample.normal_load_factor_g,
            "maneuver_active": sample.maneuver_active,
        },
    )


####


def _srbm_engagement_sample(
    binding: Ads6EngagementActorBinding,
    sample: Ads6SrbmSample,
    *,
    held: bool,
    alive: bool,
) -> Ads6EngagementSample:
    return Ads6EngagementSample(
        time_s=sample.time_s,
        actor_id=binding.actor_id,
        actor_kind="srbm",
        model_id=binding.model_id,
        fidelity=sample.fidelity,
        source_phase=sample.phase,
        held=held,
        alive=alive,
        position_ned_m=sample.position_ned_m,
        velocity_ned_mps=sample.velocity_ned_mps,
        telemetry={
            "altitude_m": sample.altitude_m,
            "speed_mps": sample.speed_mps,
            "heading_deg": sample.heading_deg,
            "flight_path_deg": sample.flight_path_deg,
            "alpha_deg": sample.alpha_deg,
            "beta_deg": sample.beta_deg,
            "mass_kg": sample.mass_kg,
            "thrust_n": sample.thrust_n,
        },
    )


####


def _radar_engagement_sample(
    binding: Ads6EngagementActorBinding,
    sample: Ads6RadarSample,
) -> Ads6EngagementSample:
    return Ads6EngagementSample(
        time_s=sample.time_s,
        actor_id=binding.actor_id,
        actor_kind="radar",
        model_id=binding.model_id,
        fidelity=binding.fidelity,
        source_phase=sample.track_mode,
        held=False,
        alive=True,
        position_ned_m=sample.position_ned_m,
        velocity_ned_mps=(0.0, 0.0, 0.0),
        telemetry={
            "next_track_time_s": sample.next_track_time_s,
            "launch_times_s": sample.launch_times_s,
            "intercept_points_ned_m": sample.intercept_points_ned_m,
            "lethal_latched": sample.lethal_latched,
            "apogee_latched": sample.apogee_latched,
            "apogee_epoch_s": sample.apogee_epoch_s,
        },
    )


####


def _impact_event(actor_id: str, time_s: float, impact: Ads6SrbmImpact) -> Ads6EngagementEvent:
    return Ads6EngagementEvent(
        time_s=time_s,
        kind="target_impact",
        actor_id=actor_id,
        details={
            "impact_kind": impact.kind,
            "distance_m": impact.distance_m,
            "position_ned_m": impact.position_ned_m,
        },
    )


####


def _wrap_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


####


def _limit(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


####


__all__ = [
    "Ads6EngagementActorBinding",
    "Ads6EngagementCommandLaw",
    "Ads6EngagementControllerTrace",
    "Ads6EngagementEvent",
    "Ads6EngagementObjectTrace",
    "Ads6EngagementPacketTrace",
    "Ads6EngagementRunConfig",
    "Ads6EngagementRunResult",
    "Ads6EngagementSession",
    "Ads6EngagementSample",
    "Ads6EngagementSourceDefinition",
    "Ads6EngagementSourceError",
    "Ads6EngagementTargetKind",
    "load_ads6_engagement_source_definition",
    "lower_ads6_engagement_source_bundle",
    "run_ads6_engagement",
]
