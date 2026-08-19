"""Neutral point-mass kernel for resolved parametric interceptor profiles."""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field, replace
from typing import Literal

import numpy as np

from taoryx.contracts import Frame, FrameVector3, Vector3
from taoryx.equations.atmosphere import atmos_gravity_inverse_square
from taoryx.runtime.environment_runtime import EnvironmentProvider, EnvironmentSample, ExponentialAtmosphereProvider
from taoryx.sensor_api import EntityTruth, SensorContext, TruthPoint
from taoryx.sensors import AccelerationIncrement, TranslationAccelerationAdapter
from taoryx.trajectory.standard_output import StandardEcefState, project_standard_ecef_samples, standard_ecef_state_from_values

from .aerodynamics import evaluate_maneuver_drag
from .applicability import InterceptorApplicabilityEnvelope
from .control_authority import (
    ControlAllocationPolicy,
    ControlConfiguration,
    allocate_control_authority,
    evaluate_control_authority,
)
from .event_detection import (
    TranslationStepEvent,
    first_ground_contact_time_within_step,
    localize_translation_step_event,
    objective_is_captured,
)
from .guidance import (
    GuidanceArchetype,
    evaluate_direct_lateral_acceleration,
    evaluate_lateral_acceleration_tracking,
    evaluate_waypoint_guidance,
    unavailable_target_track_guidance,
)
from .profile import ResolvedInterceptorProfile
from .propulsion import PropulsionProgram
from .sensor_suite import (
    TARGET_TRACK_ENTITY_ID,
    TargetTrackSensor,
    TargetTrackTelemetry,
    TranslationAccelerationSensor,
    standard_interceptor_sensor_suite,
    target_track_telemetry,
)

_EARTH_RADIUS_M = 6_371_000.0
_STANDARD_GRAVITY_MPS2 = 9.80665
_ECI_FROM_NED = np.asarray(((0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0)))
GuidanceObjectiveKind = Literal[
    "fixed_waypoint",
    "constant_velocity_target",
    "direct_lateral_acceleration",
]


@dataclass(frozen=True, slots=True)
class PointMassMission:
    """Small batch mission accepted by the surrogate kernel."""

    launch_north_m: float = 0.0
    launch_east_m: float = 0.0
    launch_altitude_m: float = 0.0
    launch_speed_mps: float = 20.0
    launch_heading_deg: float = 0.0
    launch_flight_path_deg: float = 45.0
    waypoint_north_m: float = 10_000.0
    waypoint_east_m: float = 0.0
    waypoint_altitude_m: float = 3_000.0
    capture_radius_m: float = 100.0
    target_north_m: float = 10_000.0
    target_east_m: float = 0.0
    target_altitude_m: float = 3_000.0
    target_north_velocity_mps: float = 0.0
    target_east_velocity_mps: float = 0.0
    target_vertical_velocity_mps: float = 0.0
    target_capture_radius_m: float = 100.0
    direct_lateral_acceleration_north_mps2: float = 0.0
    direct_lateral_acceleration_east_mps2: float = 0.0
    direct_lateral_acceleration_vertical_mps2: float = 0.0
    objective_kind: GuidanceObjectiveKind = "fixed_waypoint"
    duration_s: float = 60.0
    time_step_s: float = 0.05

    def __post_init__(self) -> None:
        numeric = tuple(getattr(self, name) for name in self.__dataclass_fields__ if name != "objective_kind")
        if any(not math.isfinite(value) for value in numeric):
            raise ValueError("mission values must be finite")
        if self.objective_kind not in {
            "fixed_waypoint",
            "constant_velocity_target",
            "direct_lateral_acceleration",
        }:
            raise ValueError(f"unsupported guidance objective kind {self.objective_kind!r}")
        if self.launch_altitude_m < 0.0 or self.waypoint_altitude_m < 0.0 or self.target_altitude_m < 0.0:
            raise ValueError("mission altitudes must be nonnegative")
        if self.launch_speed_mps <= 0.0:
            raise ValueError("launch speed must be positive")
        if self.capture_radius_m <= 0.0 or self.target_capture_radius_m <= 0.0 or self.duration_s <= 0.0 or self.time_step_s <= 0.0:
            raise ValueError("capture radius, duration, and time step must be positive")
        if self.time_step_s > self.duration_s:
            raise ValueError("time step cannot exceed mission duration")
        ####

    def guidance_objective(self) -> PointMassWaypoint:
        """Lower the selected mission grammar into one shared objective."""

        if self.objective_kind == "constant_velocity_target":
            return PointMassWaypoint(
                north_m=self.target_north_m,
                east_m=self.target_east_m,
                altitude_m=self.target_altitude_m,
                capture_radius_m=self.target_capture_radius_m,
                north_velocity_mps=self.target_north_velocity_mps,
                east_velocity_mps=self.target_east_velocity_mps,
                vertical_velocity_mps=self.target_vertical_velocity_mps,
                objective_kind=self.objective_kind,
            )
        if self.objective_kind == "direct_lateral_acceleration":
            return PointMassWaypoint(
                north_m=self.launch_north_m,
                east_m=self.launch_east_m,
                altitude_m=self.launch_altitude_m,
                capture_radius_m=self.capture_radius_m,
                direct_lateral_acceleration_north_mps2=self.direct_lateral_acceleration_north_mps2,
                direct_lateral_acceleration_east_mps2=self.direct_lateral_acceleration_east_mps2,
                direct_lateral_acceleration_vertical_mps2=self.direct_lateral_acceleration_vertical_mps2,
                objective_kind=self.objective_kind,
            )
        return PointMassWaypoint(
            north_m=self.waypoint_north_m,
            east_m=self.waypoint_east_m,
            altitude_m=self.waypoint_altitude_m,
            capture_radius_m=self.capture_radius_m,
            objective_kind=self.objective_kind,
        )
        ####

    ####


@dataclass(frozen=True, slots=True)
class PointMassWaypoint:
    """One fixed waypoint, target track, or held direct-control objective."""

    north_m: float
    east_m: float
    altitude_m: float
    capture_radius_m: float
    north_velocity_mps: float = 0.0
    east_velocity_mps: float = 0.0
    vertical_velocity_mps: float = 0.0
    reference_time_s: float = 0.0
    direct_lateral_acceleration_north_mps2: float = 0.0
    direct_lateral_acceleration_east_mps2: float = 0.0
    direct_lateral_acceleration_vertical_mps2: float = 0.0
    objective_kind: GuidanceObjectiveKind = "fixed_waypoint"

    def __post_init__(self) -> None:
        if any(
            not math.isfinite(value)
            for value in (
                self.north_m,
                self.east_m,
                self.altitude_m,
                self.capture_radius_m,
                self.north_velocity_mps,
                self.east_velocity_mps,
                self.vertical_velocity_mps,
                self.reference_time_s,
                self.direct_lateral_acceleration_north_mps2,
                self.direct_lateral_acceleration_east_mps2,
                self.direct_lateral_acceleration_vertical_mps2,
            )
        ):
            raise ValueError("guidance objective values must be finite")
        if self.altitude_m < 0.0:
            raise ValueError("guidance objective reference altitude must be nonnegative")
        if self.capture_radius_m <= 0.0:
            raise ValueError("guidance objective capture radius must be positive")
        if self.reference_time_s < 0.0:
            raise ValueError("guidance objective reference time must be nonnegative")
        if self.objective_kind not in {
            "fixed_waypoint",
            "constant_velocity_target",
            "direct_lateral_acceleration",
        }:
            raise ValueError(f"unsupported guidance objective kind {self.objective_kind!r}")
        ####

    def position_at(self, time_s: float) -> tuple[float, float, float]:
        """Propagate the target reference with its constant local velocity."""

        if not math.isfinite(time_s) or time_s < self.reference_time_s:
            raise ValueError("guidance objective time must be finite and not precede its reference")
        elapsed = time_s - self.reference_time_s
        return (
            self.north_m + self.north_velocity_mps * elapsed,
            self.east_m + self.east_velocity_mps * elapsed,
            self.altitude_m + self.vertical_velocity_mps * elapsed,
        )
        ####

    @property
    def velocity_mps(self) -> tuple[float, float, float]:
        return (
            self.north_velocity_mps,
            self.east_velocity_mps,
            self.vertical_velocity_mps,
        )
        ####

    @property
    def direct_lateral_acceleration_mps2(self) -> tuple[float, float, float]:
        """Return the held local north/east/positive-up direct command."""

        return (
            self.direct_lateral_acceleration_north_mps2,
            self.direct_lateral_acceleration_east_mps2,
            self.direct_lateral_acceleration_vertical_mps2,
        )
        ####

    ####


@dataclass(frozen=True, slots=True)
class PointMassState:
    """Portable point-mass state shared by batch and stateful execution."""

    time_s: float
    north_m: float
    east_m: float
    altitude_m: float
    north_velocity_mps: float
    east_velocity_mps: float
    vertical_velocity_mps: float

    def __post_init__(self) -> None:
        if any(
            not math.isfinite(value)
            for value in (
                self.time_s,
                self.north_m,
                self.east_m,
                self.altitude_m,
                self.north_velocity_mps,
                self.east_velocity_mps,
                self.vertical_velocity_mps,
            )
        ):
            raise ValueError("point-mass state values must be finite")
        if self.time_s < 0.0:
            raise ValueError("point-mass state time must be nonnegative")
        ####

    ####


@dataclass(frozen=True, slots=True)
class PointMassSample:
    """One fully labeled point-mass sample with required standard ECEF data."""

    time_s: float
    phase_id: str
    operational: bool
    applicability_declared: bool
    applicability_status: str
    applicability_reason: str
    north_m: float
    east_m: float
    altitude_m: float
    north_velocity_mps: float
    east_velocity_mps: float
    vertical_velocity_mps: float
    speed_mps: float
    mass_kg: float
    thrust_n: float
    axial_thrust_n: float
    throttle_command: float
    throttle_achieved: float
    propellant_remaining_kg: float
    propulsion_available: bool
    propulsion_phase: str
    propulsion_pulse_index: int
    density_kg_m3: float
    pressure_pa: float
    temperature_k: float
    speed_of_sound_mps: float
    wind_velocity_x_mps: float
    wind_velocity_y_mps: float
    wind_velocity_z_mps: float
    airspeed_mps: float
    mach: float
    drag_coefficient: float
    maneuver_normal_force_coefficient: float
    maneuver_drag_factor: float
    maneuver_drag_coefficient: float
    total_drag_coefficient: float
    dynamic_pressure_pa: float
    base_drag_n: float
    maneuver_drag_n: float
    drag_n: float
    waypoint_north_accepted_m: float
    waypoint_east_accepted_m: float
    waypoint_altitude_accepted_m: float
    waypoint_capture_radius_accepted_m: float
    waypoint_range_m: float
    guidance_objective_kind: str
    target_north_m: float
    target_east_m: float
    target_altitude_m: float
    target_north_velocity_mps: float
    target_east_velocity_mps: float
    target_vertical_velocity_mps: float
    relative_north_velocity_mps: float
    relative_east_velocity_mps: float
    relative_vertical_velocity_mps: float
    time_to_closest_approach_s: float
    predicted_miss_distance_m: float
    guidance_archetype: str
    guidance_mode: str
    guidance_closing_speed_mps: float
    guidance_line_of_sight_rate_rad_s: float
    guidance_navigation_constant: float
    lateral_acceleration_command_mps2: float
    lateral_acceleration_achieved_mps2: float
    direct_lateral_acceleration_accepted_vector_mps2: tuple[float, float, float]
    lateral_acceleration_command_vector_mps2: tuple[float, float, float]
    lateral_acceleration_achieved_vector_mps2: tuple[float, float, float]
    lateral_acceleration_achievement_fraction: float
    lateral_acceleration_direction_error_valid: bool
    lateral_acceleration_direction_error_rad: float
    lateral_acceleration_limit_mps2: float
    lateral_acceleration_utilization: float
    control_configuration: str
    aerodynamic_lateral_authority_mps2: float
    thrust_vector_lateral_authority_mps2: float
    combined_lateral_authority_mps2: float
    lateral_acceleration_available_mps2: float
    lateral_acceleration_authority_utilization: float
    control_authority_structural_limit_active: bool
    control_allocation_policy: str
    aerodynamic_lateral_acceleration_achieved_mps2: float
    thrust_vector_lateral_acceleration_achieved_mps2: float
    thrust_vector_angle_achieved_rad: float
    guidance_available: bool
    control_limited: bool
    control_limit_reason: str
    target_track_applicable: bool = False
    target_track_valid: bool = False
    target_track_sampled_at_s: float = 0.0
    target_track_available_at_s: float = 0.0
    target_track_latency_s: float = 0.0
    target_track_delivery_fresh: bool = False
    target_track_sequence: int = -1
    target_track_schema_id: str = "taoryx.tracking.relative-state/v1"
    target_track_invalid_reason: str = "not-applicable"
    target_track_target_id: str = TARGET_TRACK_ENTITY_ID
    target_track_frame_id: str = "sensor"
    target_track_range_m: float = 0.0
    target_track_azimuth_rad: float = 0.0
    target_track_elevation_rad: float = 0.0
    target_track_closing_speed_mps: float = 0.0
    target_track_relative_position_sensor_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    target_track_relative_velocity_sensor_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    target_track_line_of_sight_rate_sensor_rad_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
    translation_acceleration_valid: bool = False
    translation_acceleration_interval_s: float = 0.0
    translation_delta_velocity_x_mps: float = 0.0
    translation_delta_velocity_y_mps: float = 0.0
    translation_delta_velocity_z_mps: float = 0.0
    translation_acceleration_sampled_at_s: float = 0.0
    translation_acceleration_available_at_s: float = 0.0
    translation_acceleration_latency_s: float = 0.0
    translation_acceleration_schema_id: str = "taoryx.acceleration.increment/v1"
    translation_acceleration_delivery_fresh: bool = False
    translation_acceleration_sequence: int = -1
    standard_ecef: StandardEcefState = field(init=False, repr=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "standard_ecef", standard_ecef_state_from_values(self.time_s, _point_mass_standard_values(self)))
        ####


####


def _point_mass_standard_values(sample: PointMassSample) -> dict[str, object]:
    """Return the explicit local-NED truth used by the common projector."""

    return {
        "position.local.north": sample.north_m,
        "position.local.east": sample.east_m,
        "position.geometric.altitude": sample.altitude_m,
        "velocity.local.north": sample.north_velocity_mps,
        "velocity.local.east": sample.east_velocity_mps,
        "velocity.local.vertical": sample.vertical_velocity_mps,
    }
    ####


@dataclass(frozen=True, slots=True)
class PointMassRun:
    """Kernel history and terminal disposition with history-aware ECEF data."""

    samples: tuple[PointMassSample, ...]
    termination: str

    def __post_init__(self) -> None:
        standard_states = project_standard_ecef_samples(
            tuple((sample.time_s, _point_mass_standard_values(sample)) for sample in self.samples)
        )
        for sample, standard_state in zip(self.samples, standard_states, strict=True):
            object.__setattr__(sample, "standard_ecef", standard_state)
        ####

    ####


@dataclass(frozen=True, slots=True)
class PointMassStepEvaluation:
    """Truth and acceleration evaluated at one committed state boundary."""

    sample: PointMassSample
    acceleration_mps2: tuple[float, float, float]
    truth_point: TruthPoint
    target_track: TargetTrackTelemetry
    waypoint_captured: bool
    ground_impact: bool


####


class PointMassKernel:
    """Reusable point-mass equations with an absolute propulsion clock."""

    def __init__(
        self,
        profile: ResolvedInterceptorProfile,
        *,
        environment: EnvironmentProvider | None = None,
        gravity_acceleration: Callable[[float], float] | None = None,
        translation_acceleration_adapter: TranslationAccelerationSensor | None = None,
        target_track_adapter: TargetTrackSensor | None = None,
    ) -> None:
        self._atmosphere = environment or ExponentialAtmosphereProvider(reference_radius_m=_EARTH_RADIUS_M)
        self._gravity = gravity_acceleration or _standard_gravity
        self._propulsion = PropulsionProgram.from_profile(profile)
        self._applicability = InterceptorApplicabilityEnvelope.from_profile(profile)
        self._reference_area = profile.number("reference_area_m2")
        self._drag_schedule = profile.drag_coefficient_schedule
        self._maneuver_drag_factor = profile.number("maneuver_drag_factor")
        self._acceleration_limit = profile.number("max_lateral_acceleration_mps2")
        self._control_configuration: ControlConfiguration = profile.text("control_configuration")  # type: ignore[assignment]
        self._control_allocation_policy: ControlAllocationPolicy = profile.text("control_allocation_policy")  # type: ignore[assignment]
        self._normal_force_coefficient_limit = profile.number("normal_force_coefficient_limit")
        self._max_thrust_vector_angle = profile.number("max_thrust_vector_angle_rad")
        self._guidance_time_constant = profile.number("guidance_time_constant_s")
        self._guidance_archetype: GuidanceArchetype = profile.text("guidance_archetype")  # type: ignore[assignment]
        self._navigation_constant = profile.number("navigation_constant")
        self._translation_sensor = translation_acceleration_adapter or TranslationAccelerationAdapter()
        self._target_track_sensor = target_track_adapter
        ####

    def reset_sensor(self) -> None:
        """Reset all standard sensor accepted-history state."""

        self._translation_sensor.reset()
        if self._target_track_sensor is not None:
            self._target_track_sensor.reset()
        ####

    def sensor_checkpoint(self) -> dict[str, object]:
        """Serialize navigation and target-track sensor state for replay."""

        return {
            "schema_version": 2,
            "translation": dict(self._translation_sensor.snapshot()),
            "target_track": (None if self._target_track_sensor is None else dict(self._target_track_sensor.snapshot())),
        }
        ####

    def restore_sensor(self, checkpoint: Mapping[str, object]) -> None:
        """Restore navigation and target-track history before continuation."""

        if checkpoint.get("schema_version") != 2:
            raise ValueError("unsupported point-mass sensor checkpoint schema")
        translation = checkpoint.get("translation")
        if not isinstance(translation, Mapping):
            raise ValueError("point-mass sensor checkpoint is missing translation state")
        self._translation_sensor.restore(translation)
        target_track = checkpoint.get("target_track")
        if target_track is not None:
            if not isinstance(target_track, Mapping):
                raise ValueError("point-mass target-track sensor checkpoint must be a mapping")
            self._target_sensor().restore(target_track)
        ####

    def initial_state(self, mission: PointMassMission) -> PointMassState:
        """Build the exact launch state used by batch and session routes."""

        heading = math.radians(mission.launch_heading_deg)
        flight_path = math.radians(mission.launch_flight_path_deg)
        horizontal_speed = mission.launch_speed_mps * math.cos(flight_path)
        return PointMassState(
            time_s=0.0,
            north_m=mission.launch_north_m,
            east_m=mission.launch_east_m,
            altitude_m=mission.launch_altitude_m,
            north_velocity_mps=horizontal_speed * math.cos(heading),
            east_velocity_mps=horizontal_speed * math.sin(heading),
            vertical_velocity_mps=mission.launch_speed_mps * math.sin(flight_path),
        )
        ####

    def evaluate_committed(
        self,
        state: PointMassState,
        waypoint: PointMassWaypoint,
    ) -> PointMassStepEvaluation:
        """Sample target context once, then evaluate one accepted boundary."""

        return self._evaluate_target_sensor(state, waypoint, sample=True)
        ####

    def evaluate_held(
        self,
        state: PointMassState,
        waypoint: PointMassWaypoint,
    ) -> PointMassStepEvaluation:
        """Evaluate using the packet already accepted at this boundary."""

        return self._evaluate_target_sensor(state, waypoint, sample=False)
        ####

    def _evaluate_target_sensor(
        self,
        state: PointMassState,
        waypoint: PointMassWaypoint,
        *,
        sample: bool,
    ) -> PointMassStepEvaluation:
        target_track = TargetTrackTelemetry()
        guidance_waypoint: PointMassWaypoint | None = None
        if waypoint.objective_kind == "constant_velocity_target":
            target_track, guidance_waypoint = _measure_target_track(
                self._target_sensor(),
                host=_point_mass_tracking_truth(state),
                host_position_local_m=(state.north_m, state.east_m, state.altitude_m),
                host_velocity_local_mps=(
                    state.north_velocity_mps,
                    state.east_velocity_mps,
                    state.vertical_velocity_mps,
                ),
                waypoint=waypoint,
                sample=sample,
            )
        return self.evaluate(
            state,
            waypoint,
            guidance_waypoint=guidance_waypoint,
            target_track=target_track,
        )
        ####

    def evaluate(
        self,
        state: PointMassState,
        waypoint: PointMassWaypoint,
        *,
        guidance_waypoint: PointMassWaypoint | None = None,
        target_track: TargetTrackTelemetry | None = None,
    ) -> PointMassStepEvaluation:
        """Evaluate truth and derivatives without advancing the committed state."""

        position = (state.north_m, state.east_m, state.altitude_m)
        velocity = (
            state.north_velocity_mps,
            state.east_velocity_mps,
            state.vertical_velocity_mps,
        )
        direct_control = waypoint.objective_kind == "direct_lateral_acceleration"
        target = position if direct_control else waypoint.position_at(state.time_s)
        selected_track = target_track or TargetTrackTelemetry(applicable=waypoint.objective_kind == "constant_velocity_target")
        guidance_target = waypoint if waypoint.objective_kind == "fixed_waypoint" else guidance_waypoint
        track_available = direct_control or guidance_target is not None
        propulsion_sample = self._propulsion.sample(state.time_s)
        mass = propulsion_sample.mass_kg
        displacement = _subtract(target, position)
        waypoint_range = _norm(displacement)
        speed = _norm(velocity)
        direction = _unit(velocity, fallback=_unit(displacement, fallback=(1.0, 0.0, 0.0)))
        environment_sample = self._atmosphere.sample(
            time=state.time_s,
            position=FrameVector3(
                Vector3(_EARTH_RADIUS_M + max(state.altitude_m, 0.0), 0.0, 0.0),
                Frame.ECFC,
            ),
        )
        _validate_environment_sample(environment_sample)
        air_velocity = _air_relative_velocity(velocity, environment_sample.wind)
        airspeed = _norm(air_velocity)
        air_direction = _unit(air_velocity, fallback=direction)
        dynamic_pressure = 0.5 * environment_sample.density * airspeed**2
        mach = airspeed / environment_sample.speed_of_sound
        applicability = self._applicability.evaluate(
            altitude_m=max(state.altitude_m, 0.0),
            mach=mach,
        )
        drag_coefficient = self._drag_schedule.coefficient_at(mach)
        base_drag = dynamic_pressure * self._reference_area * drag_coefficient
        thrust = propulsion_sample.thrust_n
        authority = evaluate_control_authority(
            configuration=self._control_configuration,
            dynamic_pressure_pa=dynamic_pressure,
            reference_area_m2=self._reference_area,
            normal_force_coefficient_limit=self._normal_force_coefficient_limit,
            thrust_n=thrust,
            mass_kg=mass,
            max_thrust_vector_angle_rad=self._max_thrust_vector_angle,
            structural_limit_mps2=self._acceleration_limit,
        )
        if direct_control:
            guidance = evaluate_direct_lateral_acceleration(
                waypoint.direct_lateral_acceleration_mps2,
                reference_direction=direction,
            )
        elif guidance_target is None:
            guidance = unavailable_target_track_guidance(
                archetype=self._guidance_archetype,
                navigation_constant=self._navigation_constant,
            )
        else:
            guidance = evaluate_waypoint_guidance(
                position,
                velocity,
                guidance_target.position_at(state.time_s),
                waypoint_velocity_mps=guidance_target.velocity_mps,
                archetype=self._guidance_archetype,
                pursuit_time_constant_s=self._guidance_time_constant,
                navigation_constant=self._navigation_constant,
            )
        lateral = guidance.acceleration_mps2
        commanded_lateral = guidance.commanded_acceleration_mps2
        allocation = allocate_control_authority(
            authority,
            requested_mps2=commanded_lateral,
            policy=self._control_allocation_policy,
            thrust_n=thrust,
            mass_kg=mass,
        )
        achieved_lateral = allocation.achieved_mps2
        maneuver_drag = evaluate_maneuver_drag(
            dynamic_pressure_pa=dynamic_pressure,
            reference_area_m2=self._reference_area,
            mass_kg=mass,
            aerodynamic_lateral_acceleration_mps2=allocation.aerodynamic_achieved_mps2,
            maneuver_drag_factor=self._maneuver_drag_factor,
        )
        drag = base_drag + maneuver_drag.drag_n
        limit_reasons: list[str] = []
        if commanded_lateral > self._acceleration_limit + 1.0e-9:
            limit_reasons.append("lateral_acceleration_command_saturation")
        if commanded_lateral > authority.available_mps2 + 1.0e-9 and authority.available_mps2 < self._acceleration_limit - 1.0e-9:
            limit_reasons.append(authority.saturation_reason)
        control_limited = bool(limit_reasons)
        guidance_acceleration = _scaled(lateral, achieved_lateral / commanded_lateral) if commanded_lateral > 1.0e-12 else (0.0, 0.0, 0.0)
        lateral_tracking = evaluate_lateral_acceleration_tracking(
            guidance.acceleration_mps2,
            guidance_acceleration,
        )
        acceleration = tuple(
            guidance_acceleration[index] + direction[index] * allocation.axial_thrust_n / mass - air_direction[index] * drag / mass for index in range(3)
        )
        gravity_value = self._gravity(max(state.altitude_m, 0.0))
        acceleration = (
            acceleration[0],
            acceleration[1],
            acceleration[2] - gravity_value,
        )
        captured = False if direct_control else objective_is_captured(waypoint_range, waypoint.capture_radius_m)
        ground_impact = state.altitude_m <= 1.0e-9 and state.time_s > 0.0 and state.vertical_velocity_mps < 0.0
        phase_id = (
            "ground_impact"
            if ground_impact
            else "direct_lateral_acceleration_control"
            if direct_control
            else "target_intercept"
            if captured and waypoint.objective_kind == "constant_velocity_target"
            else "waypoint_capture"
            if captured
            else "boost"
            if propulsion_sample.available
            else "target_track_unavailable"
            if waypoint.objective_kind == "constant_velocity_target" and not track_available
            else "target_track_guidance"
            if waypoint.objective_kind == "constant_velocity_target"
            else "waypoint_guidance"
        )
        return PointMassStepEvaluation(
            sample=PointMassSample(
                time_s=state.time_s,
                phase_id=phase_id,
                operational=not ground_impact,
                applicability_declared=applicability.declared,
                applicability_status=applicability.status,
                applicability_reason=applicability.reason,
                north_m=state.north_m,
                east_m=state.east_m,
                altitude_m=max(state.altitude_m, 0.0),
                north_velocity_mps=state.north_velocity_mps,
                east_velocity_mps=state.east_velocity_mps,
                vertical_velocity_mps=state.vertical_velocity_mps,
                speed_mps=speed,
                mass_kg=mass,
                thrust_n=thrust,
                axial_thrust_n=allocation.axial_thrust_n,
                throttle_command=propulsion_sample.throttle_command,
                throttle_achieved=propulsion_sample.throttle_achieved,
                propellant_remaining_kg=propulsion_sample.propellant_remaining_kg,
                propulsion_available=propulsion_sample.available,
                propulsion_phase=propulsion_sample.phase.value,
                propulsion_pulse_index=propulsion_sample.pulse_index,
                density_kg_m3=environment_sample.density,
                pressure_pa=environment_sample.pressure,
                temperature_k=environment_sample.temperature,
                speed_of_sound_mps=environment_sample.speed_of_sound,
                wind_velocity_x_mps=environment_sample.wind.vector.x,
                wind_velocity_y_mps=environment_sample.wind.vector.y,
                wind_velocity_z_mps=environment_sample.wind.vector.z,
                airspeed_mps=airspeed,
                mach=mach,
                drag_coefficient=drag_coefficient,
                maneuver_normal_force_coefficient=maneuver_drag.normal_force_coefficient,
                maneuver_drag_factor=maneuver_drag.factor,
                maneuver_drag_coefficient=maneuver_drag.drag_coefficient,
                total_drag_coefficient=drag_coefficient + maneuver_drag.drag_coefficient,
                dynamic_pressure_pa=dynamic_pressure,
                base_drag_n=base_drag,
                maneuver_drag_n=maneuver_drag.drag_n,
                drag_n=drag,
                waypoint_north_accepted_m=waypoint.north_m,
                waypoint_east_accepted_m=waypoint.east_m,
                waypoint_altitude_accepted_m=waypoint.altitude_m,
                waypoint_capture_radius_accepted_m=waypoint.capture_radius_m,
                waypoint_range_m=waypoint_range,
                guidance_objective_kind=waypoint.objective_kind,
                target_north_m=target[0],
                target_east_m=target[1],
                target_altitude_m=target[2],
                target_north_velocity_mps=waypoint.north_velocity_mps,
                target_east_velocity_mps=waypoint.east_velocity_mps,
                target_vertical_velocity_mps=waypoint.vertical_velocity_mps,
                relative_north_velocity_mps=guidance.relative_velocity_mps[0],
                relative_east_velocity_mps=guidance.relative_velocity_mps[1],
                relative_vertical_velocity_mps=guidance.relative_velocity_mps[2],
                time_to_closest_approach_s=guidance.time_to_closest_approach_s,
                predicted_miss_distance_m=guidance.predicted_miss_distance_m,
                guidance_archetype=guidance.archetype,
                guidance_mode=guidance.mode,
                guidance_closing_speed_mps=guidance.closing_speed_mps,
                guidance_line_of_sight_rate_rad_s=guidance.line_of_sight_rate_rad_s,
                guidance_navigation_constant=guidance.navigation_constant,
                lateral_acceleration_command_mps2=commanded_lateral,
                lateral_acceleration_achieved_mps2=achieved_lateral,
                direct_lateral_acceleration_accepted_vector_mps2=(waypoint.direct_lateral_acceleration_mps2 if direct_control else (0.0, 0.0, 0.0)),
                lateral_acceleration_command_vector_mps2=guidance.acceleration_mps2,
                lateral_acceleration_achieved_vector_mps2=guidance_acceleration,
                lateral_acceleration_achievement_fraction=lateral_tracking.achievement_fraction,
                lateral_acceleration_direction_error_valid=lateral_tracking.direction_error_valid,
                lateral_acceleration_direction_error_rad=lateral_tracking.direction_error_rad,
                lateral_acceleration_limit_mps2=self._acceleration_limit,
                lateral_acceleration_utilization=min(achieved_lateral / self._acceleration_limit, 1.0),
                control_configuration=authority.configuration,
                aerodynamic_lateral_authority_mps2=authority.aerodynamic_mps2,
                thrust_vector_lateral_authority_mps2=authority.thrust_vector_mps2,
                combined_lateral_authority_mps2=authority.combined_unclipped_mps2,
                lateral_acceleration_available_mps2=authority.available_mps2,
                lateral_acceleration_authority_utilization=(
                    min(achieved_lateral / authority.available_mps2, 1.0) if authority.available_mps2 > 1.0e-12 else 0.0
                ),
                control_authority_structural_limit_active=authority.structural_limit_active,
                control_allocation_policy=allocation.policy,
                aerodynamic_lateral_acceleration_achieved_mps2=allocation.aerodynamic_achieved_mps2,
                thrust_vector_lateral_acceleration_achieved_mps2=allocation.thrust_vector_achieved_mps2,
                thrust_vector_angle_achieved_rad=allocation.thrust_vector_angle_rad,
                guidance_available=track_available and not captured and not ground_impact,
                control_limited=control_limited,
                control_limit_reason="+".join(limit_reasons) if limit_reasons else "none",
            ),
            acceleration_mps2=acceleration,
            truth_point=_translation_truth_point(
                time_s=state.time_s,
                position=position,
                velocity=velocity,
                acceleration=acceleration,
                gravity_mps2=gravity_value,
            ),
            target_track=selected_track,
            waypoint_captured=captured,
            ground_impact=ground_impact,
        )
        ####

    def _target_sensor(self) -> TargetTrackSensor:
        if self._target_track_sensor is None:
            self._target_track_sensor = standard_interceptor_sensor_suite().build_target_track_sensor()
        return self._target_track_sensor
        ####

    def localize_step_event(
        self,
        state: PointMassState,
        evaluation: PointMassStepEvaluation,
        waypoint: PointMassWaypoint,
        duration_s: float,
    ) -> TranslationStepEvent | None:
        """Locate capture or ground contact inside the next held-derivative step."""

        if waypoint.objective_kind == "direct_lateral_acceleration":
            ground_time = first_ground_contact_time_within_step(
                altitude_m=state.altitude_m,
                vertical_velocity_mps=state.vertical_velocity_mps,
                vertical_acceleration_mps2=evaluation.acceleration_mps2[2],
                duration_s=duration_s,
            )
            return None if ground_time is None else TranslationStepEvent("ground_contact", ground_time)

        target = waypoint.position_at(state.time_s)
        return localize_translation_step_event(
            relative_position_m=(
                target[0] - state.north_m,
                target[1] - state.east_m,
                target[2] - state.altitude_m,
            ),
            relative_velocity_mps=(
                waypoint.north_velocity_mps - state.north_velocity_mps,
                waypoint.east_velocity_mps - state.east_velocity_mps,
                waypoint.vertical_velocity_mps - state.vertical_velocity_mps,
            ),
            interceptor_acceleration_mps2=evaluation.acceleration_mps2,
            capture_radius_m=waypoint.capture_radius_m,
            altitude_m=state.altitude_m,
            vertical_velocity_mps=state.vertical_velocity_mps,
            duration_s=duration_s,
        )
        ####

    def sample(self, evaluation: PointMassStepEvaluation) -> PointMassSample:
        """Project a committed translation boundary through the standard adapter."""

        packet = self._translation_sensor.sample(evaluation.truth_point)
        sensor_values = _translation_sensor_values(packet.payload if packet.valid else None)
        track = evaluation.target_track
        return replace(
            evaluation.sample,
            target_track_applicable=track.applicable,
            target_track_valid=track.valid,
            target_track_sampled_at_s=track.sampled_at_s,
            target_track_available_at_s=track.available_at_s,
            target_track_latency_s=track.latency_s,
            target_track_delivery_fresh=track.delivery_fresh,
            target_track_sequence=track.sequence,
            target_track_schema_id=track.schema_id,
            target_track_invalid_reason=track.invalid_reason,
            target_track_target_id=track.target_id,
            target_track_frame_id=track.frame_id,
            target_track_range_m=track.range_m,
            target_track_azimuth_rad=track.azimuth_rad,
            target_track_elevation_rad=track.elevation_rad,
            target_track_closing_speed_mps=track.closing_speed_mps,
            target_track_relative_position_sensor_m=track.relative_position_sensor_m,
            target_track_relative_velocity_sensor_mps=track.relative_velocity_sensor_mps,
            target_track_line_of_sight_rate_sensor_rad_s=track.line_of_sight_rate_sensor_rad_s,
            translation_acceleration_valid=packet.valid,
            translation_acceleration_interval_s=sensor_values[0],
            translation_delta_velocity_x_mps=sensor_values[1],
            translation_delta_velocity_y_mps=sensor_values[2],
            translation_delta_velocity_z_mps=sensor_values[3],
            translation_acceleration_sampled_at_s=packet.sampled_at_s,
            translation_acceleration_available_at_s=packet.available_at_s,
            translation_acceleration_latency_s=packet.available_at_s - packet.sampled_at_s,
            translation_acceleration_schema_id=packet.schema_id or "taoryx.acceleration.increment/v1",
            translation_acceleration_delivery_fresh=bool(getattr(self._translation_sensor, "delivery_fresh", True)),
            translation_acceleration_sequence=(-1 if packet.sequence is None else packet.sequence),
        )
        ####

    def advance(
        self,
        state: PointMassState,
        evaluation: PointMassStepEvaluation,
        duration_s: float,
    ) -> PointMassState:
        """Advance with the kernel's existing semi-implicit Euler rule."""

        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError("point-mass advance duration must be finite and positive")
        acceleration = evaluation.acceleration_mps2
        velocity = (
            state.north_velocity_mps + acceleration[0] * duration_s,
            state.east_velocity_mps + acceleration[1] * duration_s,
            state.vertical_velocity_mps + acceleration[2] * duration_s,
        )
        return PointMassState(
            time_s=round(state.time_s + duration_s, 12),
            north_m=state.north_m + velocity[0] * duration_s,
            east_m=state.east_m + velocity[1] * duration_s,
            altitude_m=state.altitude_m + velocity[2] * duration_s,
            north_velocity_mps=velocity[0],
            east_velocity_mps=velocity[1],
            vertical_velocity_mps=velocity[2],
        )
        ####

    ####


def run_point_mass_interceptor(
    profile: ResolvedInterceptorProfile,
    mission: PointMassMission,
    *,
    environment: EnvironmentProvider | None = None,
    gravity_acceleration: Callable[[float], float] | None = None,
    translation_acceleration_adapter: TranslationAccelerationSensor | None = None,
    target_track_adapter: TargetTrackSensor | None = None,
) -> PointMassRun:
    """Propagate one profile toward a fixed local-NED waypoint.

    Atmosphere comes from the standard Taoryx environment boundary. Gravity is
    injected or evaluated through the shared core inverse-square equation, not
    carried as a CADAC environment or vehicle submodel.
    """

    kernel = PointMassKernel(
        profile,
        environment=environment,
        gravity_acceleration=gravity_acceleration,
        translation_acceleration_adapter=translation_acceleration_adapter,
        target_track_adapter=target_track_adapter,
    )
    kernel.reset_sensor()
    state = kernel.initial_state(mission)
    waypoint = mission.guidance_objective()
    samples: list[PointMassSample] = []
    termination = "duration"

    while True:
        evaluation = kernel.evaluate_committed(state, waypoint)
        samples.append(kernel.sample(evaluation))
        if evaluation.ground_impact:
            termination = "ground_impact"
            break
        if evaluation.waypoint_captured:
            termination = "target_intercept" if waypoint.objective_kind == "constant_velocity_target" else "waypoint_capture"
            break
        if state.time_s >= mission.duration_s - 1.0e-12:
            break

        step = min(mission.time_step_s, mission.duration_s - state.time_s)
        event = kernel.localize_step_event(state, evaluation, waypoint, step)
        if event is not None and event.time_from_step_start_s > 1.0e-12:
            step = event.time_from_step_start_s
        state = kernel.advance(state, evaluation, step)
    return PointMassRun(samples=tuple(samples), termination=termination)
    ####


def _translation_truth_point(
    *,
    time_s: float,
    position: tuple[float, float, float],
    velocity: tuple[float, float, float],
    acceleration: tuple[float, float, float],
    gravity_mps2: float,
) -> TruthPoint:
    """Map local translation truth into the adapter's navigation-frame contract."""

    position_eci = np.asarray((_EARTH_RADIUS_M + position[2], position[1], position[0]))
    velocity_eci = np.asarray((velocity[2], velocity[1], velocity[0]))
    acceleration_eci = np.asarray((acceleration[2], acceleration[1], acceleration[0]))
    gravity_eci = np.asarray((-gravity_mps2, 0.0, 0.0))
    return TruthPoint(
        time_s=time_s,
        position_eci_m=position_eci,
        velocity_eci_mps=velocity_eci,
        velocity_without_gravity_eci_mps=velocity_eci,
        orientation_eci_from_body=None,
        gravity_eci_mps2=gravity_eci,
        angular_rate_body_radps=None,
        acceleration_eci_mps2=acceleration_eci,
    )
    ####


def _point_mass_tracking_truth(state: PointMassState) -> TruthPoint:
    """Build the advertised velocity-aligned virtual body frame for tracking."""

    horizontal_speed = math.hypot(state.north_velocity_mps, state.east_velocity_mps)
    yaw = math.atan2(state.east_velocity_mps, state.north_velocity_mps)
    pitch = math.atan2(state.vertical_velocity_mps, horizontal_speed)
    cos_pitch, sin_pitch = math.cos(pitch), math.sin(pitch)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    ned_from_body = np.asarray(
        (
            (cos_pitch * cos_yaw, -sin_yaw, sin_pitch * cos_yaw),
            (cos_pitch * sin_yaw, cos_yaw, sin_pitch * sin_yaw),
            (-sin_pitch, 0.0, cos_pitch),
        )
    )
    position = (state.north_m, state.east_m, state.altitude_m)
    velocity = (
        state.north_velocity_mps,
        state.east_velocity_mps,
        state.vertical_velocity_mps,
    )
    truth = _translation_truth_point(
        time_s=state.time_s,
        position=position,
        velocity=velocity,
        acceleration=(0.0, 0.0, 0.0),
        gravity_mps2=_standard_gravity(max(state.altitude_m, 0.0)),
    )
    return TruthPoint(
        time_s=truth.time_s,
        position_eci_m=truth.position_eci_m,
        velocity_eci_mps=truth.velocity_eci_mps,
        velocity_without_gravity_eci_mps=truth.velocity_without_gravity_eci_mps,
        orientation_eci_from_body=_ECI_FROM_NED @ ned_from_body,
        gravity_eci_mps2=truth.gravity_eci_mps2,
        angular_rate_body_radps=np.zeros(3),
        acceleration_eci_mps2=truth.acceleration_eci_mps2,
    )
    ####


def _measure_target_track(
    sensor: TargetTrackSensor,
    *,
    host: TruthPoint,
    host_position_local_m: tuple[float, float, float],
    host_velocity_local_mps: tuple[float, float, float],
    waypoint: PointMassWaypoint,
    sample: bool = True,
) -> tuple[TargetTrackTelemetry, PointMassWaypoint | None]:
    """Sample the native tracker and reconstruct its measured local reference."""

    target_position = waypoint.position_at(host.time_s)
    target_position_eci = np.asarray((_EARTH_RADIUS_M + target_position[2], target_position[1], target_position[0]))
    target_velocity_eci = np.asarray(
        (
            waypoint.vertical_velocity_mps,
            waypoint.east_velocity_mps,
            waypoint.north_velocity_mps,
        )
    )
    context = SensorContext(
        snapshot_id=f"parametric-interceptor-target@{host.time_s:.17g}",
        host=host,
        entities={
            TARGET_TRACK_ENTITY_ID: EntityTruth(
                TARGET_TRACK_ENTITY_ID,
                target_position_eci,
                target_velocity_eci,
                properties={"objective_kind": waypoint.objective_kind},
            )
        },
    )
    packet = sensor.sample_context(context) if sample else sensor.held_packet(host.time_s)
    telemetry = target_track_telemetry(
        packet,
        delivery_fresh=bool(getattr(sensor, "delivery_fresh", True)),
    )
    payload = packet.payload if packet.valid else None
    orientation = host.orientation_eci_from_body
    if payload is None or orientation is None:
        return telemetry, None
    relative_position_eci = orientation @ np.asarray(payload.relative_position_sensor_m)
    relative_velocity_eci = orientation @ np.asarray(payload.relative_velocity_sensor_mps)
    relative_position_local = _eci_vector_to_local(relative_position_eci)
    relative_velocity_local = _eci_vector_to_local(relative_velocity_eci)
    return telemetry, PointMassWaypoint(
        north_m=host_position_local_m[0] + relative_position_local[0],
        east_m=host_position_local_m[1] + relative_position_local[1],
        altitude_m=host_position_local_m[2] + relative_position_local[2],
        capture_radius_m=waypoint.capture_radius_m,
        north_velocity_mps=host_velocity_local_mps[0] + relative_velocity_local[0],
        east_velocity_mps=host_velocity_local_mps[1] + relative_velocity_local[1],
        vertical_velocity_mps=host_velocity_local_mps[2] + relative_velocity_local[2],
        reference_time_s=host.time_s,
        objective_kind=waypoint.objective_kind,
    )
    ####


def _eci_vector_to_local(value: np.ndarray) -> tuple[float, float, float]:
    return float(value[2]), float(value[1]), float(value[0])
    ####


def _validate_environment_sample(sample: EnvironmentSample) -> None:
    """Reject malformed provider output before it enters aerodynamic calculations."""

    scalars = (
        sample.density,
        sample.pressure,
        sample.temperature,
        sample.speed_of_sound,
        sample.wind.vector.x,
        sample.wind.vector.y,
        sample.wind.vector.z,
    )
    if any(not math.isfinite(value) for value in scalars):
        raise ValueError("standard environment sample values must be finite")
    if sample.density < 0.0 or sample.pressure < 0.0:
        raise ValueError("standard environment density and pressure must be nonnegative")
    if sample.temperature <= 0.0 or sample.speed_of_sound <= 0.0:
        raise ValueError("standard environment temperature and speed of sound must be positive")
    if sample.wind.frame is not Frame.ECFC:
        raise ValueError("standard environment wind must be expressed in ECFC")
    ####


def _air_relative_velocity(
    velocity_local: tuple[float, float, float],
    wind_ecfc: FrameVector3,
) -> tuple[float, float, float]:
    """Subtract ECFC radial/east/north wind from local north/east/up velocity."""

    if wind_ecfc.frame is not Frame.ECFC:
        raise ValueError("standard environment wind must be expressed in ECFC")
    return (
        velocity_local[0] - wind_ecfc.vector.z,
        velocity_local[1] - wind_ecfc.vector.y,
        velocity_local[2] - wind_ecfc.vector.x,
    )
    ####


def _translation_sensor_values(
    payload: AccelerationIncrement | None,
) -> tuple[float, float, float, float]:
    if payload is None:
        return (0.0, 0.0, 0.0, 0.0)
    return (
        payload.dt_s,
        float(payload.delta_v_eci_mps[0]),
        float(payload.delta_v_eci_mps[1]),
        float(payload.delta_v_eci_mps[2]),
    )
    ####


def _subtract(
    left: list[float] | tuple[float, float, float],
    right: list[float] | tuple[float, float, float],
) -> tuple[float, float, float]:
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])
    ####


def _dot(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
    return sum(a * b for a, b in zip(left, right, strict=True))
    ####


def _norm(value: list[float] | tuple[float, float, float]) -> float:
    return math.sqrt(sum(item * item for item in value))
    ####


def _unit(
    value: list[float] | tuple[float, float, float],
    *,
    fallback: tuple[float, float, float],
) -> tuple[float, float, float]:
    magnitude = _norm(value)
    return fallback if magnitude <= 1.0e-12 else tuple(item / magnitude for item in value)  # type: ignore[return-value]
    ####


def _scaled(value: tuple[float, float, float], factor: float) -> tuple[float, float, float]:
    return (value[0] * factor, value[1] * factor, value[2] * factor)
    ####


def _standard_gravity(altitude_m: float) -> float:
    """Use the shared core inverse-square equation with standard references."""

    return atmos_gravity_inverse_square(_STANDARD_GRAVITY_MPS2, _EARTH_RADIUS_M, altitude_m)
    ####


__all__ = [
    "PointMassKernel",
    "PointMassMission",
    "PointMassRun",
    "PointMassSample",
    "PointMassState",
    "PointMassStepEvaluation",
    "PointMassWaypoint",
    "run_point_mass_interceptor",
]
####
