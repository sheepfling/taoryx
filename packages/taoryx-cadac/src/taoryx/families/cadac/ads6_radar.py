"""Source-shaped ADS6 ``RADAR0`` tracking and launch-command runtime."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Literal, TypeAlias

import numpy as np
from numpy.typing import NDArray
from pydantic import Field, model_validator

from .bundle import CadacSourceArtifact, CadacSourceBundle
from .deck import CadacDeck
from .input_ast import CadacModel, CadacVehicleBlock
from .sensor_adapter import cadac_local_ned_relative_state_track

FloatVector: TypeAlias = NDArray[np.float64]
Ads6RadarTrackMode = Literal["aircraft", "srbm"]
Ads6RadarLaunchReason = Literal["lethal_range", "apogee_prediction"]

_SMALL = 1.0e-9
_MAX_TRACKS = 3
_DEFAULT_HOLD_TIME_S = 9_999.0


class Ads6RadarSourceError(ValueError):
    """Source-bundle incompatibility with the ADS6 RADAR0 runtime."""


####


class Ads6RadarTargetTruth(CadacModel):
    """One source-order target packet consumed by RADAR0."""

    actor_id: str = Field(min_length=1)
    position_ned_m: tuple[float, float, float]
    velocity_ned_mps: tuple[float, float, float]


####


class Ads6RadarMissileTruth(CadacModel):
    """One source-order missile packet used for SRBM intercept-point refinement."""

    actor_id: str = Field(min_length=1)
    position_ned_m: tuple[float, float, float]
    velocity_ned_mps: tuple[float, float, float]
    launched: bool = False


####


class Ads6RadarSourceDefinition(CadacModel):
    """Prepared fixed-site RADAR0 source configuration."""

    source_name: str = Field(min_length=1)
    source_role: str = Field(min_length=1)
    track_mode: Ads6RadarTrackMode
    position_ned_m: tuple[float, float, float]
    integration_step_s: float = Field(gt=0.0)
    track_step_s: float = Field(ge=0.0)
    engagement_altitude_m: float = Field(default=0.0, ge=0.0)
    lethal_range_m: float = Field(default=0.0, ge=0.0)
    intercept_point_altitude_bias_m: float = 0.0
    launch_delay_bias_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
    range_sigma_m: float = Field(default=0.0, ge=0.0)
    azimuth_sigma_rad: float = Field(default=0.0, ge=0.0)
    elevation_sigma_rad: float = Field(default=0.0, ge=0.0)
    velocity_sigma_mps: float = Field(default=0.0, ge=0.0)
    missile_trajectory_deck: CadacDeck | None = None
    srbm_trajectory_deck: CadacDeck | None = None
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=1)
    claim_boundary: str = (
        "ADS6 RADAR0 is a fixed local-level site. It samples up to three source-order targets, applies source-shaped "
        "polar/velocity measurement corruption, publishes target tracks and launch times, and for SRBM defense uses "
        "the source missile/rocket trajectory decks to predict and refine intercept points. It is a mission sensor and "
        "scheduler, not a trajectory vehicle."
    )

    @model_validator(mode="after")
    def validate_mode_resources(self) -> "Ads6RadarSourceDefinition":
        if self.track_mode == "aircraft" and self.lethal_range_m <= 0.0:
            raise ValueError("ADS6 aircraft-tracking radar requires a positive lethal range")
        ####
        if self.track_mode == "srbm":
            if self.engagement_altitude_m <= 0.0:
                raise ValueError("ADS6 SRBM-tracking radar requires a positive engagement altitude")
            ####
            if self.missile_trajectory_deck is None or self.srbm_trajectory_deck is None:
                raise ValueError("ADS6 SRBM-tracking radar requires SAM_DECK and SRBM_DECK resources")
            ####
            _require_tables(
                self.missile_trajectory_deck,
                ("time_vs_ascent_altitude", "alt_vs_launch_time"),
                "SAM_DECK",
            )
            _require_tables(
                self.srbm_trajectory_deck,
                (
                    "apotime_vs_descent_altitude",
                    "x_vs_launch_time",
                    "y_vs_launch_time",
                    "z_vs_launch_time",
                ),
                "SRBM_DECK",
            )
        ####
        return self

    ####


####


class Ads6RadarLaunchCommand(CadacModel):
    """One RADAR0 launch schedule and current intercept-point uplink."""

    missile_index: int = Field(ge=1, le=_MAX_TRACKS)
    target_index: int = Field(ge=1, le=_MAX_TRACKS)
    issued_time_s: float = Field(ge=0.0)
    launch_time_s: float = Field(ge=0.0)
    intercept_point_ned_m: tuple[float, float, float]
    reason: Ads6RadarLaunchReason
    newly_latched: bool


####


class Ads6RadarTrackRecord(CadacModel):
    """One source-cadence RADAR0 target measurement."""

    time_s: float = Field(ge=0.0)
    target_index: int = Field(ge=1, le=_MAX_TRACKS)
    target_actor_id: str = Field(min_length=1)
    truth_position_ned_m: tuple[float, float, float]
    truth_velocity_ned_mps: tuple[float, float, float]
    measured_position_ned_m: tuple[float, float, float]
    measured_velocity_ned_mps: tuple[float, float, float]
    measured_range_m: float = Field(ge=0.0)
    measured_azimuth_rad: float
    measured_elevation_rad: float


####


class Ads6RadarStepResult(CadacModel):
    """Track and launch outputs from one RADAR0 module pass."""

    time_s: float = Field(ge=0.0)
    tracked: bool
    track_records: tuple[Ads6RadarTrackRecord, ...] = ()
    launch_commands: tuple[Ads6RadarLaunchCommand, ...] = ()


####


class Ads6RadarSample(CadacModel):
    """One static-site radar state sample for Mission Composition output."""

    time_s: float = Field(ge=0.0)
    position_ned_m: tuple[float, float, float]
    track_mode: Ads6RadarTrackMode
    next_track_time_s: float = Field(ge=0.0)
    launch_times_s: tuple[float, float, float]
    intercept_points_ned_m: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]
    lethal_latched: tuple[bool, bool, bool]
    apogee_latched: bool
    apogee_epoch_s: float | None = Field(default=None, ge=0.0)


####


@dataclass(slots=True)
class Ads6RadarRuntime:
    """Persistent RADAR0 packet-generation state."""

    definition: Ads6RadarSourceDefinition
    seed: int = 0
    _rng: np.random.Generator = field(init=False, repr=False)
    _next_track_time_s: float = 0.0
    _launch_times_s: FloatVector = field(
        default_factory=lambda: np.full(_MAX_TRACKS, _DEFAULT_HOLD_TIME_S, dtype=np.float64),
        repr=False,
    )
    _intercept_points_ned_m: NDArray[np.float64] = field(
        default_factory=lambda: np.zeros((_MAX_TRACKS, 3), dtype=np.float64),
        repr=False,
    )
    _lethal_latched: list[bool] = field(default_factory=lambda: [False, False, False], repr=False)
    _apogee_latched: bool = False
    _apogee_epoch_s: float | None = None
    _base_launch_time_s: float = _DEFAULT_HOLD_TIME_S
    _base_intercept_point_ned_m: FloatVector = field(
        default_factory=lambda: np.zeros(3, dtype=np.float64),
        repr=False,
    )

    def __post_init__(self) -> None:
        self._rng = np.random.default_rng(self.seed)

    ####

    def step(
        self,
        sim_time_s: float,
        targets: tuple[Ads6RadarTargetTruth, ...],
        *,
        missiles: tuple[Ads6RadarMissileTruth, ...] = (),
    ) -> Ads6RadarStepResult:
        """Execute one source-cadence radar pass against source-order packets."""

        if sim_time_s + 0.5 * self.definition.integration_step_s < self._next_track_time_s:
            return Ads6RadarStepResult(time_s=sim_time_s, tracked=False)
        ####
        self._next_track_time_s = sim_time_s + self.definition.track_step_s
        tracks: list[Ads6RadarTrackRecord] = []
        commands: list[Ads6RadarLaunchCommand] = []
        for target_index, target in enumerate(targets[:_MAX_TRACKS], start=1):
            track = self._measure(sim_time_s, target_index, target)
            tracks.append(track)
            missile = missiles[target_index - 1] if target_index <= len(missiles) else None
            if self.definition.track_mode == "aircraft":
                commands.append(self._aircraft_command(sim_time_s, target_index, track))
            else:
                command = self._srbm_command(sim_time_s, target_index, track, missile)
                if command is not None:
                    commands.append(command)
                ####
            ####
        ####
        return Ads6RadarStepResult(
            time_s=sim_time_s,
            tracked=True,
            track_records=tuple(tracks),
            launch_commands=tuple(commands),
        )

    ####

    def sample(self, time_s: float) -> Ads6RadarSample:
        """Return the current fixed-site tracking/scheduling state."""

        points = tuple(_tuple3(self._intercept_points_ned_m[index]) for index in range(_MAX_TRACKS))
        return Ads6RadarSample(
            time_s=time_s,
            position_ned_m=self.definition.position_ned_m,
            track_mode=self.definition.track_mode,
            next_track_time_s=max(0.0, self._next_track_time_s),
            launch_times_s=_tuple3(self._launch_times_s),
            intercept_points_ned_m=(points[0], points[1], points[2]),
            lethal_latched=tuple(self._lethal_latched),
            apogee_latched=self._apogee_latched,
            apogee_epoch_s=self._apogee_epoch_s,
        )

    ####

    @property
    def launch_times_s(self) -> tuple[float, float, float]:
        return _tuple3(self._launch_times_s)

    ####

    @property
    def intercept_points_ned_m(
        self,
    ) -> tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]:
        points = tuple(_tuple3(self._intercept_points_ned_m[index]) for index in range(_MAX_TRACKS))
        return points[0], points[1], points[2]

    ####

    def _measure(
        self,
        sim_time_s: float,
        target_index: int,
        target: Ads6RadarTargetTruth,
    ) -> Ads6RadarTrackRecord:
        radar = np.asarray(self.definition.position_ned_m, dtype=np.float64)
        truth_position = np.asarray(target.position_ned_m, dtype=np.float64)
        truth_velocity = np.asarray(target.velocity_ned_mps, dtype=np.float64)
        sensor = cadac_local_ned_relative_state_track(
            time_s=sim_time_s,
            host_position_ned_m=radar,
            host_velocity_ned_mps=np.zeros(3, dtype=np.float64),
            target_id=target.actor_id,
            target_position_ned_m=truth_position,
            target_velocity_ned_mps=truth_velocity,
            body_from_local=np.eye(3, dtype=np.float64),
        )
        # Preserve RADAR0's source noise sequence and polar convention above
        # the native raw relative-state projection.
        relative = truth_position - radar if isinstance(sensor, str) else np.asarray(sensor.relative_position_sensor_m, dtype=np.float64)
        distance, azimuth, elevation = _polar_from_cart(relative)
        measured_distance = max(0.0, distance + self._rng.normal(0.0, self.definition.range_sigma_m))
        measured_azimuth = azimuth + self._rng.normal(0.0, self.definition.azimuth_sigma_rad)
        measured_elevation = elevation + self._rng.normal(0.0, self.definition.elevation_sigma_rad)
        measured_relative = _cart_from_polar(measured_distance, measured_azimuth, measured_elevation)
        # Preserve the source RADAR0 expression. ADS6 supplied engagement cases
        # place the radar at the local-level origin, where this equals the usual
        # radar-plus-relative reconstruction.
        measured_position = measured_relative - radar
        measured_velocity = truth_velocity + self._rng.normal(
            0.0,
            self.definition.velocity_sigma_mps,
            size=3,
        )
        return Ads6RadarTrackRecord(
            time_s=sim_time_s,
            target_index=target_index,
            target_actor_id=target.actor_id,
            truth_position_ned_m=target.position_ned_m,
            truth_velocity_ned_mps=target.velocity_ned_mps,
            measured_position_ned_m=_tuple3(measured_position),
            measured_velocity_ned_mps=_tuple3(measured_velocity),
            measured_range_m=measured_distance,
            measured_azimuth_rad=measured_azimuth,
            measured_elevation_rad=measured_elevation,
        )

    ####

    def _aircraft_command(
        self,
        sim_time_s: float,
        target_index: int,
        track: Ads6RadarTrackRecord,
    ) -> Ads6RadarLaunchCommand:
        slot = target_index - 1
        newly_latched = False
        if track.measured_range_m < self.definition.lethal_range_m and not self._lethal_latched[slot]:
            self._lethal_latched[slot] = True
            self._launch_times_s[slot] = sim_time_s + self.definition.launch_delay_bias_s[slot]
            newly_latched = True
        ####
        self._intercept_points_ned_m[slot] = np.asarray(track.measured_position_ned_m, dtype=np.float64)
        return Ads6RadarLaunchCommand(
            missile_index=target_index,
            target_index=target_index,
            issued_time_s=sim_time_s,
            launch_time_s=float(self._launch_times_s[slot]),
            intercept_point_ned_m=_tuple3(self._intercept_points_ned_m[slot]),
            reason="lethal_range",
            newly_latched=newly_latched,
        )

    ####

    def _srbm_command(
        self,
        sim_time_s: float,
        target_index: int,
        track: Ads6RadarTrackRecord,
        missile: Ads6RadarMissileTruth | None,
    ) -> Ads6RadarLaunchCommand | None:
        definition = self.definition
        missile_deck = definition.missile_trajectory_deck
        target_deck = definition.srbm_trajectory_deck
        if missile_deck is None or target_deck is None:
            raise AssertionError("validated SRBM radar definition lost trajectory decks")
        ####
        if track.measured_velocity_ned_mps[2] > 0.0 and not self._apogee_latched:
            self._apogee_latched = True
            self._apogee_epoch_s = sim_time_s
            target_time = target_deck.table("apotime_vs_descent_altitude").interpolate((definition.engagement_altitude_m,))
            missile_time = missile_deck.table("time_vs_ascent_altitude").interpolate((definition.engagement_altitude_m,))
            self._base_launch_time_s = sim_time_s + target_time - missile_time
            target_epoch = target_time + sim_time_s
            self._base_intercept_point_ned_m = np.asarray(
                (
                    target_deck.table("x_vs_launch_time").interpolate((target_epoch,)),
                    target_deck.table("y_vs_launch_time").interpolate((target_epoch,)),
                    -definition.engagement_altitude_m,
                ),
                dtype=np.float64,
            )
        ####
        if not self._apogee_latched:
            return None
        ####
        slot = target_index - 1
        self._launch_times_s[slot] = self._base_launch_time_s + definition.launch_delay_bias_s[slot]
        point = self._base_intercept_point_ned_m.copy()
        if sim_time_s > self._base_launch_time_s and self._apogee_epoch_s is not None:
            target_altitude = -track.measured_position_ned_m[2]
            target_apo_time = target_deck.table("apotime_vs_descent_altitude").interpolate((target_altitude,))
            predicted_target_altitude = -target_deck.table("z_vs_launch_time").interpolate((target_apo_time + self._apogee_epoch_s,))
            target_altitude_error = predicted_target_altitude - target_altitude
            missile_altitude_error = 0.0
            if missile is not None and missile.launched:
                missile_altitude = -missile.position_ned_m[2]
                missile_time = missile_deck.table("time_vs_ascent_altitude").interpolate((missile_altitude,))
                predicted_missile_altitude = missile_deck.table("alt_vs_launch_time").interpolate((missile_time,))
                missile_altitude_error = predicted_missile_altitude - missile_altitude
            ####
            altitude = max(
                0.0,
                -point[2] - target_altitude_error - missile_altitude_error,
            )
            target_apo_time = target_deck.table("apotime_vs_descent_altitude").interpolate((altitude,))
            target_epoch = target_apo_time + self._apogee_epoch_s
            point = np.asarray(
                (
                    target_deck.table("x_vs_launch_time").interpolate((target_epoch,)),
                    target_deck.table("y_vs_launch_time").interpolate((target_epoch,)),
                    -altitude - definition.intercept_point_altitude_bias_m,
                ),
                dtype=np.float64,
            )
        ####
        self._intercept_points_ned_m[slot] = point
        return Ads6RadarLaunchCommand(
            missile_index=target_index,
            target_index=target_index,
            issued_time_s=sim_time_s,
            launch_time_s=float(self._launch_times_s[slot]),
            intercept_point_ned_m=_tuple3(point),
            reason="apogee_prediction",
            newly_latched=math.isclose(
                sim_time_s,
                self._apogee_epoch_s if self._apogee_epoch_s is not None else -1.0,
                rel_tol=0.0,
                abs_tol=0.5 * definition.integration_step_s,
            ),
        )

    ####


####


def lower_ads6_radar_actor(
    bundle: CadacSourceBundle,
    actor: CadacVehicleBlock,
) -> Ads6RadarSourceDefinition:
    """Lower one fixed ADS6 ``RADAR0`` block from a package case."""

    if actor.model_name.casefold() != "radar0":
        raise Ads6RadarSourceError(f"expected RADAR0 actor, received {actor.model_name!r}")
    ####
    mode_value = _integer(actor, "mtrack", 0)
    try:
        track_mode: Ads6RadarTrackMode = {1: "srbm", 2: "aircraft"}[mode_value]
    except KeyError as error:
        raise Ads6RadarSourceError("ADS6 package radar must select mtrack 1 (SRBM) or 2 (aircraft)") from error
    ####
    try:
        integration_step = bundle.case.timing_values["int_step"]
    except KeyError as error:
        raise Ads6RadarSourceError("ADS6 package source case is missing TIMING int_step") from error
    ####
    bindings = bundle.decks_for("RADAR0", vehicle_role=actor.role)
    missile_deck = _deck_by_keyword(bindings, "SAM_DECK")
    srbm_deck = _deck_by_keyword(bindings, "SRBM_DECK")
    return Ads6RadarSourceDefinition(
        source_name=bundle.case.source_name,
        source_role=actor.role,
        track_mode=track_mode,
        position_ned_m=(
            _number(actor, "srel1", 0.0),
            _number(actor, "srel2", 0.0),
            _number(actor, "srel3", 0.0),
        ),
        integration_step_s=integration_step,
        track_step_s=_number(actor, "track_step", 0.0),
        engagement_altitude_m=_number(actor, "alt_engage", 0.0),
        lethal_range_m=_number(actor, "lethal_rng", 0.0),
        intercept_point_altitude_bias_m=_number(actor, "ip_alt_bias", 0.0),
        launch_delay_bias_s=(
            _number(actor, "lnch_dly_bias1", 0.0),
            _number(actor, "lnch_dly_bias2", 0.0),
            _number(actor, "lnch_dly_bias3", 0.0),
        ),
        range_sigma_m=_number(actor, "dat_sigma", 0.0),
        azimuth_sigma_rad=_number(actor, "azat_sigma", 0.0),
        elevation_sigma_rad=_number(actor, "elat_sigma", 0.0),
        velocity_sigma_mps=_number(actor, "vel_sigma", 0.0),
        missile_trajectory_deck=missile_deck,
        srbm_trajectory_deck=srbm_deck,
        source_artifacts=bundle.artifacts,
    )


####


def _deck_by_keyword(bindings: tuple[object, ...], keyword: str) -> CadacDeck | None:
    matches = [binding.deck for binding in bindings if getattr(binding, "keyword", "").casefold() == keyword.casefold()]
    if len(matches) > 1:
        raise Ads6RadarSourceError(f"ADS6 RADAR0 has multiple {keyword} bindings")
    ####
    return matches[0] if matches else None


####


def _require_tables(deck: CadacDeck, names: tuple[str, ...], label: str) -> None:
    available = {table.name.casefold() for table in deck.tables}
    missing = tuple(name for name in names if name.casefold() not in available)
    if missing:
        raise ValueError(f"ADS6 {label} is missing required tables: {missing!r}")
    ####


####


def _number(vehicle: CadacVehicleBlock, name: str, default: float | None = None) -> float:
    try:
        value = vehicle.parameter(name)
    except KeyError:
        if default is None:
            raise Ads6RadarSourceError(f"{vehicle.model_name} {vehicle.role!r} is missing required scalar {name!r}") from None
        ####
        return float(default)
    ####
    if not isinstance(value, (int, float)):
        raise Ads6RadarSourceError(f"{vehicle.model_name} scalar {name!r} must be numeric")
    ####
    result = float(value)
    if not math.isfinite(result):
        raise Ads6RadarSourceError(f"{vehicle.model_name} scalar {name!r} must be finite")
    ####
    return result


####


def _integer(vehicle: CadacVehicleBlock, name: str, default: int | None = None) -> int:
    value = _number(vehicle, name, None if default is None else float(default))
    if not value.is_integer():
        raise Ads6RadarSourceError(f"{vehicle.model_name} scalar {name!r} must be integral")
    ####
    return int(value)


####


def _cart_from_polar(magnitude: float, azimuth_rad: float, elevation_rad: float) -> FloatVector:
    cosine = math.cos(elevation_rad)
    return np.asarray(
        (
            magnitude * cosine * math.cos(azimuth_rad),
            magnitude * cosine * math.sin(azimuth_rad),
            -magnitude * math.sin(elevation_rad),
        ),
        dtype=np.float64,
    )


####


def _polar_from_cart(vector: FloatVector) -> tuple[float, float, float]:
    distance = float(np.linalg.norm(vector))
    if distance <= _SMALL:
        return 0.0, 0.0, 0.0
    ####
    return (
        distance,
        math.atan2(float(vector[1]), float(vector[0])),
        math.atan2(-float(vector[2]), math.hypot(float(vector[0]), float(vector[1]))),
    )


####


def _tuple3(vector: NDArray[np.float64]) -> tuple[float, float, float]:
    return float(vector[0]), float(vector[1]), float(vector[2])


####


__all__ = [
    "Ads6RadarLaunchCommand",
    "Ads6RadarMissileTruth",
    "Ads6RadarRuntime",
    "Ads6RadarSample",
    "Ads6RadarSourceDefinition",
    "Ads6RadarSourceError",
    "Ads6RadarStepResult",
    "Ads6RadarTargetTruth",
    "Ads6RadarTrackMode",
    "Ads6RadarTrackRecord",
    "lower_ads6_radar_actor",
]
