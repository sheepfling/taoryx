"""Attitude-response pseudo-6DOF tier for parametric interceptors."""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Literal

import numpy as np

from taoryx.contracts import Frame, FrameVector3, Vector3
from taoryx.equations.atmosphere import atmos_gravity_inverse_square
from taoryx.runtime.environment_runtime import EnvironmentProvider, ExponentialAtmosphereProvider
from taoryx.sensor_api import TruthPoint
from taoryx.sensors import IdealImuAdapter, ImuIncrement

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
    direction_response_acceleration,
    evaluate_direct_lateral_acceleration,
    evaluate_lateral_acceleration_tracking,
    evaluate_waypoint_guidance,
    unavailable_target_track_guidance,
)
from .kernel import (
    PointMassMission,
    PointMassWaypoint,
    _air_relative_velocity,
    _measure_target_track,
    _validate_environment_sample,
)
from .profile import ResolvedInterceptorProfile
from .propulsion import PropulsionProgram
from .sensor_suite import (
    ImuSensor,
    TargetTrackSensor,
    TargetTrackTelemetry,
    standard_interceptor_sensor_suite,
)

_EARTH_RADIUS_M = 6_371_000.0
_STANDARD_GRAVITY_MPS2 = 9.80665
_ECI_FROM_NED = np.asarray(((0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0)))
_PITCH_LIMIT_RAD = math.radians(89.0)
Pseudo6ResponseAxis = Literal["roll", "pitch", "yaw"]


@dataclass(frozen=True, slots=True)
class Pseudo6Sample:
    """One pseudo-6DOF truth sample plus standard ideal-IMU readback."""

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
    flow_angles_valid: bool
    air_relative_velocity_body_mps: tuple[float, float, float]
    angle_of_attack_rad: float
    sideslip_angle_rad: float
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
    attitude_response_authority_available: bool
    attitude_response_command_support_fraction: float
    attitude_response_authority_limited: bool
    control_allocation_policy: str
    aerodynamic_lateral_acceleration_achieved_mps2: float
    thrust_vector_lateral_acceleration_achieved_mps2: float
    thrust_vector_angle_achieved_rad: float
    guidance_available: bool
    control_limited: bool
    control_limit_reason: str
    target_track_applicable: bool
    target_track_valid: bool
    target_track_sampled_at_s: float
    target_track_available_at_s: float
    target_track_latency_s: float
    target_track_delivery_fresh: bool
    target_track_sequence: int
    target_track_schema_id: str
    target_track_invalid_reason: str
    target_track_target_id: str
    target_track_frame_id: str
    target_track_range_m: float
    target_track_azimuth_rad: float
    target_track_elevation_rad: float
    target_track_closing_speed_mps: float
    target_track_relative_position_sensor_m: tuple[float, float, float]
    target_track_relative_velocity_sensor_mps: tuple[float, float, float]
    target_track_line_of_sight_rate_sensor_rad_s: tuple[float, float, float]
    roll_command_rad: float
    pitch_command_rad: float
    yaw_command_rad: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    body_rate_p_rad_s: float
    body_rate_q_rad_s: float
    body_rate_r_rad_s: float
    body_acceleration_p_rad_s2: float
    body_acceleration_q_rad_s2: float
    body_acceleration_r_rad_s2: float
    imu_valid: bool
    imu_interval_s: float
    imu_delta_velocity_x_mps: float
    imu_delta_velocity_y_mps: float
    imu_delta_velocity_z_mps: float
    imu_delta_angle_x_rad: float
    imu_delta_angle_y_rad: float
    imu_delta_angle_z_rad: float
    imu_sampled_at_s: float
    imu_available_at_s: float
    imu_latency_s: float
    imu_schema_id: str
    imu_delivery_fresh: bool
    imu_sequence: int


####


@dataclass(frozen=True, slots=True)
class Pseudo6Run:
    """Pseudo-6DOF history and terminal disposition."""

    samples: tuple[Pseudo6Sample, ...]
    termination: str

    ####


@dataclass(frozen=True, slots=True)
class Pseudo6State:
    """Portable pseudo-6DOF state shared by batch and stateful execution."""

    time_s: float
    north_m: float
    east_m: float
    altitude_m: float
    north_velocity_mps: float
    east_velocity_mps: float
    vertical_velocity_mps: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    roll_rate_rad_s: float
    pitch_rate_rad_s: float
    yaw_rate_rad_s: float

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
                self.roll_rad,
                self.pitch_rad,
                self.yaw_rad,
                self.roll_rate_rad_s,
                self.pitch_rate_rad_s,
                self.yaw_rate_rad_s,
            )
        ):
            raise ValueError("pseudo-6DOF state values must be finite")
        if self.time_s < 0.0:
            raise ValueError("pseudo-6DOF state time must be nonnegative")
        ####

    ####


@dataclass(frozen=True, slots=True)
class Pseudo6StepEvaluation:
    """Pure truth/derivative evaluation at one pseudo-6DOF state boundary."""

    state: Pseudo6State
    waypoint: PointMassWaypoint
    mass_kg: float
    thrust_n: float
    axial_thrust_n: float
    throttle_command: float
    throttle_achieved: float
    propellant_remaining_kg: float
    propulsion_available: bool
    propulsion_phase: str
    propulsion_pulse_index: int
    phase_id: str
    operational: bool
    applicability_declared: bool
    applicability_status: str
    applicability_reason: str
    density_kg_m3: float
    pressure_pa: float
    temperature_k: float
    speed_of_sound_mps: float
    wind_velocity_x_mps: float
    wind_velocity_y_mps: float
    wind_velocity_z_mps: float
    airspeed_mps: float
    flow_angles_valid: bool
    air_relative_velocity_body_mps: tuple[float, float, float]
    angle_of_attack_rad: float
    sideslip_angle_rad: float
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
    waypoint_range_m: float
    target_position_m: tuple[float, float, float]
    relative_velocity_mps: tuple[float, float, float]
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
    attitude_response_authority_available: bool
    attitude_response_command_support_fraction: float
    attitude_response_authority_limited: bool
    control_allocation_policy: str
    aerodynamic_lateral_acceleration_achieved_mps2: float
    thrust_vector_lateral_acceleration_achieved_mps2: float
    thrust_vector_angle_achieved_rad: float
    guidance_available: bool
    control_limited: bool
    control_limit_reason: str
    attitude_command_rad: tuple[float, float, float]
    body_rates_rad_s: tuple[float, float, float]
    body_accelerations_rad_s2: tuple[float, float, float]
    acceleration_mps2: tuple[float, float, float]
    euler_accelerations_rad_s2: tuple[float, float, float]
    truth_point: TruthPoint
    target_track: TargetTrackTelemetry
    waypoint_captured: bool
    ground_impact: bool


####


class Pseudo6Kernel:
    """Reusable attitude-response equations plus the standard IMU boundary."""

    def __init__(
        self,
        profile: ResolvedInterceptorProfile,
        *,
        environment: EnvironmentProvider | None = None,
        gravity_acceleration: Callable[[float], float] | None = None,
        imu_adapter: ImuSensor | None = None,
        target_track_adapter: TargetTrackSensor | None = None,
    ) -> None:
        self._atmosphere = environment or ExponentialAtmosphereProvider(reference_radius_m=_EARTH_RADIUS_M)
        self._gravity = gravity_acceleration or _standard_gravity
        self._imu = imu_adapter or IdealImuAdapter()
        self._target_track_sensor = target_track_adapter
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
        self._natural_frequency = profile.number("attitude_bandwidth_rad_s")
        self._damping_ratio = profile.number("attitude_damping_ratio")
        self._max_body_rate = profile.number("max_body_rate_rad_s")
        self._max_body_acceleration = profile.number("max_body_acceleration_rad_s2")
        self._max_bank_angle = profile.number("max_bank_angle_rad")
        self.reset_sensor()
        ####

    def reset_sensor(self) -> None:
        """Reset only standard sensor history; dynamics state remains explicit."""

        self._imu.reset()
        if self._target_track_sensor is not None:
            self._target_track_sensor.reset()
        ####

    def sensor_checkpoint(self) -> dict[str, object]:
        """Return IMU and target-track accepted-history state."""

        return {
            "schema_version": 2,
            "imu": dict(self._imu.snapshot()),
            "target_track": (None if self._target_track_sensor is None else dict(self._target_track_sensor.snapshot())),
        }
        ####

    def restore_sensor(self, checkpoint: Mapping[str, object]) -> None:
        """Restore IMU and target-track history before a continued transition."""

        if checkpoint.get("schema_version") != 2:
            raise ValueError("unsupported pseudo-6DOF sensor checkpoint schema")
        imu = checkpoint.get("imu")
        if not isinstance(imu, Mapping):
            raise ValueError("pseudo-6DOF sensor checkpoint is missing IMU state")
        self._imu.restore(imu)
        target_track = checkpoint.get("target_track")
        if target_track is not None:
            if not isinstance(target_track, Mapping):
                raise ValueError("pseudo-6DOF target-track sensor checkpoint must be a mapping")
            self._target_sensor().restore(target_track)
        ####

    def initial_state(self, mission: PointMassMission) -> Pseudo6State:
        heading = math.radians(mission.launch_heading_deg)
        flight_path = math.radians(mission.launch_flight_path_deg)
        horizontal_speed = mission.launch_speed_mps * math.cos(flight_path)
        return Pseudo6State(
            time_s=0.0,
            north_m=mission.launch_north_m,
            east_m=mission.launch_east_m,
            altitude_m=mission.launch_altitude_m,
            north_velocity_mps=horizontal_speed * math.cos(heading),
            east_velocity_mps=horizontal_speed * math.sin(heading),
            vertical_velocity_mps=mission.launch_speed_mps * math.sin(flight_path),
            roll_rad=0.0,
            pitch_rad=flight_path,
            yaw_rad=heading,
            roll_rate_rad_s=0.0,
            pitch_rate_rad_s=0.0,
            yaw_rate_rad_s=0.0,
        )
        ####

    def evaluate_committed(
        self,
        state: Pseudo6State,
        waypoint: PointMassWaypoint,
    ) -> Pseudo6StepEvaluation:
        """Sample target context once, then evaluate one accepted boundary."""

        return self._evaluate_target_sensor(state, waypoint, sample=True)
        ####

    def evaluate_held(
        self,
        state: Pseudo6State,
        waypoint: PointMassWaypoint,
    ) -> Pseudo6StepEvaluation:
        """Evaluate using the packet already accepted at this boundary."""

        return self._evaluate_target_sensor(state, waypoint, sample=False)
        ####

    def _evaluate_target_sensor(
        self,
        state: Pseudo6State,
        waypoint: PointMassWaypoint,
        *,
        sample: bool,
    ) -> Pseudo6StepEvaluation:
        target_track = TargetTrackTelemetry()
        guidance_waypoint: PointMassWaypoint | None = None
        if waypoint.objective_kind == "constant_velocity_target":
            attitude = (state.roll_rad, state.pitch_rad, state.yaw_rad)
            euler_rates = (
                state.roll_rate_rad_s,
                state.pitch_rate_rad_s,
                state.yaw_rate_rad_s,
            )
            host = _truth_point(
                time_s=state.time_s,
                position=(state.north_m, state.east_m, state.altitude_m),
                velocity=(
                    state.north_velocity_mps,
                    state.east_velocity_mps,
                    state.vertical_velocity_mps,
                ),
                acceleration=(0.0, 0.0, 0.0),
                gravity_mps2=self._gravity(max(state.altitude_m, 0.0)),
                attitude=attitude,
                body_rates=_body_rates(attitude, euler_rates),
                body_accelerations=(0.0, 0.0, 0.0),
            )
            target_track, guidance_waypoint = _measure_target_track(
                self._target_sensor(),
                host=host,
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
        state: Pseudo6State,
        waypoint: PointMassWaypoint,
        *,
        guidance_waypoint: PointMassWaypoint | None = None,
        target_track: TargetTrackTelemetry | None = None,
    ) -> Pseudo6StepEvaluation:
        """Evaluate truth and derivatives without advancing state or sensors."""

        position = (state.north_m, state.east_m, state.altitude_m)
        velocity = (
            state.north_velocity_mps,
            state.east_velocity_mps,
            state.vertical_velocity_mps,
        )
        attitude = (state.roll_rad, state.pitch_rad, state.yaw_rad)
        euler_rates = (
            state.roll_rate_rad_s,
            state.pitch_rate_rad_s,
            state.yaw_rate_rad_s,
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
        direction = _unit(velocity, fallback=_direction_from_attitude(attitude))
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
        flow_angles_valid, air_velocity_body, angle_of_attack, sideslip_angle = _body_air_relative_flow(
            air_velocity,
            attitude,
        )
        dynamic_pressure = 0.5 * environment_sample.density * airspeed**2
        mach = airspeed / environment_sample.speed_of_sound
        applicability = self._applicability.evaluate(
            altitude_m=max(state.altitude_m, 0.0),
            mach=mach,
        )
        drag_coefficient = self._drag_schedule.coefficient_at(mach)
        base_drag = dynamic_pressure * self._reference_area * drag_coefficient
        authority = evaluate_control_authority(
            configuration=self._control_configuration,
            dynamic_pressure_pa=dynamic_pressure,
            reference_area_m2=self._reference_area,
            normal_force_coefficient_limit=self._normal_force_coefficient_limit,
            thrust_n=propulsion_sample.thrust_n,
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
        if direct_control:
            response_horizon = self._guidance_time_constant / max(speed, 1.0e-6)
            target_direction = _unit(
                tuple(direction[index] + guidance.acceleration_mps2[index] * response_horizon for index in range(3)),  # type: ignore[arg-type]
                fallback=direction,
            )
        else:
            target_direction = (
                direction
                if guidance_target is None
                else _unit(
                    _subtract(guidance_target.position_at(state.time_s), position),
                    fallback=direction,
                )
            )
        commanded_lateral = guidance.commanded_acceleration_mps2
        attitude_authority_available = authority.available_mps2 > 1.0e-12
        attitude_command_support_fraction = authority.commanded_support_fraction(commanded_lateral)
        attitude_authority_limited = commanded_lateral > authority.available_mps2 + 1.0e-9
        attitude_response_scale = attitude_command_support_fraction if commanded_lateral > 1.0e-12 else float(attitude_authority_available)
        attitude_command = _attitude_command(
            velocity,
            target_direction,
            commanded_lateral,
            max_bank_angle=self._max_bank_angle,
        )
        unbounded_bank_angle = math.atan2(commanded_lateral, _STANDARD_GRAVITY_MPS2)
        bank_angle_limited = unbounded_bank_angle > self._max_bank_angle + 1.0e-12
        unbounded_pitch_angle = math.atan2(
            target_direction[2],
            max(math.hypot(target_direction[0], target_direction[1]), 1.0e-12),
        )
        pitch_angle_limited = abs(unbounded_pitch_angle) > _PITCH_LIMIT_RAD + 1.0e-12
        euler_accelerations, attitude_limited = _response_acceleration(
            attitude,
            euler_rates,
            attitude_command,
            natural_frequency=self._natural_frequency,
            damping_ratio=self._damping_ratio,
            max_acceleration=self._max_body_acceleration,
        )
        unscaled_body_accelerations = _body_rates(attitude, euler_accelerations)
        largest_body_acceleration = max(abs(item) for item in unscaled_body_accelerations)
        if largest_body_acceleration > self._max_body_acceleration:
            scale = self._max_body_acceleration / largest_body_acceleration
            euler_accelerations = (
                euler_accelerations[0] * scale,
                euler_accelerations[1] * scale,
                euler_accelerations[2] * scale,
            )
            attitude_limited = True
        euler_accelerations = (
            euler_accelerations[0] * attitude_response_scale,
            euler_accelerations[1] * attitude_response_scale,
            euler_accelerations[2] * attitude_response_scale,
        )
        body_direction = _direction_from_attitude(attitude)
        achieved_lateral_vector = direction_response_acceleration(
            direction,
            body_direction,
            speed_mps=speed,
            time_constant_s=self._guidance_time_constant,
        )
        raw_achieved_lateral = _norm(achieved_lateral_vector)
        allocation = allocate_control_authority(
            authority,
            requested_mps2=raw_achieved_lateral,
            policy=self._control_allocation_policy,
            thrust_n=propulsion_sample.thrust_n,
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
        guidance_acceleration = _scaled(achieved_lateral_vector, achieved_lateral / raw_achieved_lateral) if raw_achieved_lateral > 1.0e-12 else (0.0, 0.0, 0.0)
        lateral_tracking = evaluate_lateral_acceleration_tracking(
            guidance.acceleration_mps2,
            guidance_acceleration,
        )
        body_rates = _body_rates(attitude, euler_rates)
        limit_reasons: list[str] = []
        if commanded_lateral > self._acceleration_limit + 1.0e-9:
            limit_reasons.append("lateral_acceleration_command_saturation")
        if raw_achieved_lateral > self._acceleration_limit + 1.0e-9:
            limit_reasons.append("lateral_acceleration_achieved_saturation")
        if max(commanded_lateral, raw_achieved_lateral) > authority.available_mps2 + 1.0e-9 and authority.available_mps2 < self._acceleration_limit - 1.0e-9:
            limit_reasons.append(authority.saturation_reason)
        if bank_angle_limited:
            limit_reasons.append("bank_angle_saturation")
        if pitch_angle_limited:
            limit_reasons.append("pitch_angle_saturation")
        if attitude_limited:
            limit_reasons.append("attitude_acceleration_saturation")
        if max(abs(item) for item in body_rates) >= self._max_body_rate - 1.0e-9:
            limit_reasons.append("body_rate_saturation")
        control_limited = bool(limit_reasons)
        force_acceleration = tuple(
            guidance_acceleration[index] + body_direction[index] * allocation.axial_thrust_n / mass - air_direction[index] * drag / mass for index in range(3)
        )
        gravity_value = self._gravity(max(state.altitude_m, 0.0))
        acceleration = (
            force_acceleration[0],
            force_acceleration[1],
            force_acceleration[2] - gravity_value,
        )
        body_accelerations = _body_rates(attitude, euler_accelerations)
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
        return Pseudo6StepEvaluation(
            state=state,
            waypoint=waypoint,
            mass_kg=mass,
            thrust_n=propulsion_sample.thrust_n,
            axial_thrust_n=allocation.axial_thrust_n,
            throttle_command=propulsion_sample.throttle_command,
            throttle_achieved=propulsion_sample.throttle_achieved,
            propellant_remaining_kg=propulsion_sample.propellant_remaining_kg,
            propulsion_available=propulsion_sample.available,
            propulsion_phase=propulsion_sample.phase.value,
            propulsion_pulse_index=propulsion_sample.pulse_index,
            phase_id=phase_id,
            operational=not ground_impact,
            applicability_declared=applicability.declared,
            applicability_status=applicability.status,
            applicability_reason=applicability.reason,
            density_kg_m3=environment_sample.density,
            pressure_pa=environment_sample.pressure,
            temperature_k=environment_sample.temperature,
            speed_of_sound_mps=environment_sample.speed_of_sound,
            wind_velocity_x_mps=environment_sample.wind.vector.x,
            wind_velocity_y_mps=environment_sample.wind.vector.y,
            wind_velocity_z_mps=environment_sample.wind.vector.z,
            airspeed_mps=airspeed,
            flow_angles_valid=flow_angles_valid,
            air_relative_velocity_body_mps=air_velocity_body,
            angle_of_attack_rad=angle_of_attack,
            sideslip_angle_rad=sideslip_angle,
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
            waypoint_range_m=waypoint_range,
            target_position_m=target,
            relative_velocity_mps=guidance.relative_velocity_mps,
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
            lateral_acceleration_authority_utilization=(min(achieved_lateral / authority.available_mps2, 1.0) if authority.available_mps2 > 1.0e-12 else 0.0),
            control_authority_structural_limit_active=authority.structural_limit_active,
            attitude_response_authority_available=attitude_authority_available,
            attitude_response_command_support_fraction=attitude_command_support_fraction,
            attitude_response_authority_limited=attitude_authority_limited,
            control_allocation_policy=allocation.policy,
            aerodynamic_lateral_acceleration_achieved_mps2=allocation.aerodynamic_achieved_mps2,
            thrust_vector_lateral_acceleration_achieved_mps2=allocation.thrust_vector_achieved_mps2,
            thrust_vector_angle_achieved_rad=allocation.thrust_vector_angle_rad,
            guidance_available=track_available and not captured and not ground_impact,
            control_limited=control_limited,
            control_limit_reason="+".join(limit_reasons) if limit_reasons else "none",
            attitude_command_rad=attitude_command,
            body_rates_rad_s=body_rates,
            body_accelerations_rad_s2=body_accelerations,
            acceleration_mps2=acceleration,
            euler_accelerations_rad_s2=euler_accelerations,
            truth_point=_truth_point(
                time_s=state.time_s,
                position=position,
                velocity=velocity,
                acceleration=acceleration,
                gravity_mps2=gravity_value,
                attitude=attitude,
                body_rates=body_rates,
                body_accelerations=body_accelerations,
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
        state: Pseudo6State,
        evaluation: Pseudo6StepEvaluation,
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

    def sample(self, evaluation: Pseudo6StepEvaluation) -> Pseudo6Sample:
        """Project one newly committed truth boundary through the standard IMU."""

        state = evaluation.state
        waypoint = evaluation.waypoint
        packet = self._imu.sample(evaluation.truth_point)
        imu_values = _imu_values(packet.payload if packet.valid else None)
        track = evaluation.target_track
        return Pseudo6Sample(
            time_s=state.time_s,
            phase_id=evaluation.phase_id,
            operational=evaluation.operational,
            applicability_declared=evaluation.applicability_declared,
            applicability_status=evaluation.applicability_status,
            applicability_reason=evaluation.applicability_reason,
            north_m=state.north_m,
            east_m=state.east_m,
            altitude_m=max(state.altitude_m, 0.0),
            north_velocity_mps=state.north_velocity_mps,
            east_velocity_mps=state.east_velocity_mps,
            vertical_velocity_mps=state.vertical_velocity_mps,
            speed_mps=_norm((state.north_velocity_mps, state.east_velocity_mps, state.vertical_velocity_mps)),
            mass_kg=evaluation.mass_kg,
            thrust_n=evaluation.thrust_n,
            axial_thrust_n=evaluation.axial_thrust_n,
            throttle_command=evaluation.throttle_command,
            throttle_achieved=evaluation.throttle_achieved,
            propellant_remaining_kg=evaluation.propellant_remaining_kg,
            propulsion_available=evaluation.propulsion_available,
            propulsion_phase=evaluation.propulsion_phase,
            propulsion_pulse_index=evaluation.propulsion_pulse_index,
            density_kg_m3=evaluation.density_kg_m3,
            pressure_pa=evaluation.pressure_pa,
            temperature_k=evaluation.temperature_k,
            speed_of_sound_mps=evaluation.speed_of_sound_mps,
            wind_velocity_x_mps=evaluation.wind_velocity_x_mps,
            wind_velocity_y_mps=evaluation.wind_velocity_y_mps,
            wind_velocity_z_mps=evaluation.wind_velocity_z_mps,
            airspeed_mps=evaluation.airspeed_mps,
            flow_angles_valid=evaluation.flow_angles_valid,
            air_relative_velocity_body_mps=evaluation.air_relative_velocity_body_mps,
            angle_of_attack_rad=evaluation.angle_of_attack_rad,
            sideslip_angle_rad=evaluation.sideslip_angle_rad,
            mach=evaluation.mach,
            drag_coefficient=evaluation.drag_coefficient,
            maneuver_normal_force_coefficient=evaluation.maneuver_normal_force_coefficient,
            maneuver_drag_factor=evaluation.maneuver_drag_factor,
            maneuver_drag_coefficient=evaluation.maneuver_drag_coefficient,
            total_drag_coefficient=evaluation.total_drag_coefficient,
            dynamic_pressure_pa=evaluation.dynamic_pressure_pa,
            base_drag_n=evaluation.base_drag_n,
            maneuver_drag_n=evaluation.maneuver_drag_n,
            drag_n=evaluation.drag_n,
            waypoint_north_accepted_m=waypoint.north_m,
            waypoint_east_accepted_m=waypoint.east_m,
            waypoint_altitude_accepted_m=waypoint.altitude_m,
            waypoint_capture_radius_accepted_m=waypoint.capture_radius_m,
            waypoint_range_m=evaluation.waypoint_range_m,
            guidance_objective_kind=waypoint.objective_kind,
            target_north_m=evaluation.target_position_m[0],
            target_east_m=evaluation.target_position_m[1],
            target_altitude_m=evaluation.target_position_m[2],
            target_north_velocity_mps=waypoint.north_velocity_mps,
            target_east_velocity_mps=waypoint.east_velocity_mps,
            target_vertical_velocity_mps=waypoint.vertical_velocity_mps,
            relative_north_velocity_mps=evaluation.relative_velocity_mps[0],
            relative_east_velocity_mps=evaluation.relative_velocity_mps[1],
            relative_vertical_velocity_mps=evaluation.relative_velocity_mps[2],
            time_to_closest_approach_s=evaluation.time_to_closest_approach_s,
            predicted_miss_distance_m=evaluation.predicted_miss_distance_m,
            guidance_archetype=evaluation.guidance_archetype,
            guidance_mode=evaluation.guidance_mode,
            guidance_closing_speed_mps=evaluation.guidance_closing_speed_mps,
            guidance_line_of_sight_rate_rad_s=evaluation.guidance_line_of_sight_rate_rad_s,
            guidance_navigation_constant=evaluation.guidance_navigation_constant,
            lateral_acceleration_command_mps2=evaluation.lateral_acceleration_command_mps2,
            lateral_acceleration_achieved_mps2=evaluation.lateral_acceleration_achieved_mps2,
            direct_lateral_acceleration_accepted_vector_mps2=(evaluation.direct_lateral_acceleration_accepted_vector_mps2),
            lateral_acceleration_command_vector_mps2=evaluation.lateral_acceleration_command_vector_mps2,
            lateral_acceleration_achieved_vector_mps2=evaluation.lateral_acceleration_achieved_vector_mps2,
            lateral_acceleration_achievement_fraction=evaluation.lateral_acceleration_achievement_fraction,
            lateral_acceleration_direction_error_valid=evaluation.lateral_acceleration_direction_error_valid,
            lateral_acceleration_direction_error_rad=evaluation.lateral_acceleration_direction_error_rad,
            lateral_acceleration_limit_mps2=evaluation.lateral_acceleration_limit_mps2,
            lateral_acceleration_utilization=evaluation.lateral_acceleration_utilization,
            control_configuration=evaluation.control_configuration,
            aerodynamic_lateral_authority_mps2=evaluation.aerodynamic_lateral_authority_mps2,
            thrust_vector_lateral_authority_mps2=evaluation.thrust_vector_lateral_authority_mps2,
            combined_lateral_authority_mps2=evaluation.combined_lateral_authority_mps2,
            lateral_acceleration_available_mps2=evaluation.lateral_acceleration_available_mps2,
            lateral_acceleration_authority_utilization=evaluation.lateral_acceleration_authority_utilization,
            control_authority_structural_limit_active=evaluation.control_authority_structural_limit_active,
            attitude_response_authority_available=evaluation.attitude_response_authority_available,
            attitude_response_command_support_fraction=evaluation.attitude_response_command_support_fraction,
            attitude_response_authority_limited=evaluation.attitude_response_authority_limited,
            control_allocation_policy=evaluation.control_allocation_policy,
            aerodynamic_lateral_acceleration_achieved_mps2=evaluation.aerodynamic_lateral_acceleration_achieved_mps2,
            thrust_vector_lateral_acceleration_achieved_mps2=evaluation.thrust_vector_lateral_acceleration_achieved_mps2,
            thrust_vector_angle_achieved_rad=evaluation.thrust_vector_angle_achieved_rad,
            guidance_available=evaluation.guidance_available,
            control_limited=evaluation.control_limited,
            control_limit_reason=evaluation.control_limit_reason,
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
            roll_command_rad=evaluation.attitude_command_rad[0],
            pitch_command_rad=evaluation.attitude_command_rad[1],
            yaw_command_rad=evaluation.attitude_command_rad[2],
            roll_rad=state.roll_rad,
            pitch_rad=state.pitch_rad,
            yaw_rad=state.yaw_rad,
            body_rate_p_rad_s=evaluation.body_rates_rad_s[0],
            body_rate_q_rad_s=evaluation.body_rates_rad_s[1],
            body_rate_r_rad_s=evaluation.body_rates_rad_s[2],
            body_acceleration_p_rad_s2=evaluation.body_accelerations_rad_s2[0],
            body_acceleration_q_rad_s2=evaluation.body_accelerations_rad_s2[1],
            body_acceleration_r_rad_s2=evaluation.body_accelerations_rad_s2[2],
            imu_valid=packet.valid,
            imu_interval_s=imu_values[0],
            imu_delta_velocity_x_mps=imu_values[1],
            imu_delta_velocity_y_mps=imu_values[2],
            imu_delta_velocity_z_mps=imu_values[3],
            imu_delta_angle_x_rad=imu_values[4],
            imu_delta_angle_y_rad=imu_values[5],
            imu_delta_angle_z_rad=imu_values[6],
            imu_sampled_at_s=packet.sampled_at_s,
            imu_available_at_s=packet.available_at_s,
            imu_latency_s=packet.available_at_s - packet.sampled_at_s,
            imu_schema_id=packet.schema_id or "taoryx.imu.increment/v1",
            imu_delivery_fresh=bool(getattr(self._imu, "delivery_fresh", True)),
            imu_sequence=-1 if packet.sequence is None else packet.sequence,
        )
        ####

    def advance(
        self,
        state: Pseudo6State,
        evaluation: Pseudo6StepEvaluation,
        duration_s: float,
    ) -> Pseudo6State:
        """Advance with the mission runtime's bounded semi-implicit update."""

        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError("pseudo-6DOF advance duration must be finite and positive")
        attitude = [state.roll_rad, state.pitch_rad, state.yaw_rad]
        euler_rates = [state.roll_rate_rad_s, state.pitch_rate_rad_s, state.yaw_rate_rad_s]
        for index in range(3):
            euler_rates[index] = _clamp(
                euler_rates[index] + evaluation.euler_accelerations_rad_s2[index] * duration_s,
                -self._max_body_rate,
                self._max_body_rate,
            )
        proposed_body_rates = _body_rates(attitude, euler_rates)
        largest_body_rate = max(abs(item) for item in proposed_body_rates)
        if largest_body_rate > self._max_body_rate:
            scale = self._max_body_rate / largest_body_rate
            euler_rates = [item * scale for item in euler_rates]
        for index in range(3):
            attitude[index] += euler_rates[index] * duration_s
        attitude[0] = _clamp(attitude[0], -self._max_bank_angle, self._max_bank_angle)
        attitude[1] = _clamp(attitude[1], -_PITCH_LIMIT_RAD, _PITCH_LIMIT_RAD)
        attitude[2] = _wrap_angle(attitude[2])
        acceleration = evaluation.acceleration_mps2
        velocity = (
            state.north_velocity_mps + acceleration[0] * duration_s,
            state.east_velocity_mps + acceleration[1] * duration_s,
            state.vertical_velocity_mps + acceleration[2] * duration_s,
        )
        return Pseudo6State(
            time_s=round(state.time_s + duration_s, 12),
            north_m=state.north_m + velocity[0] * duration_s,
            east_m=state.east_m + velocity[1] * duration_s,
            altitude_m=state.altitude_m + velocity[2] * duration_s,
            north_velocity_mps=velocity[0],
            east_velocity_mps=velocity[1],
            vertical_velocity_mps=velocity[2],
            roll_rad=attitude[0],
            pitch_rad=attitude[1],
            yaw_rad=attitude[2],
            roll_rate_rad_s=euler_rates[0],
            pitch_rate_rad_s=euler_rates[1],
            yaw_rate_rad_s=euler_rates[2],
        )
        ####

    ####


def run_pseudo6_interceptor(
    profile: ResolvedInterceptorProfile,
    mission: PointMassMission,
    *,
    environment: EnvironmentProvider | None = None,
    gravity_acceleration: Callable[[float], float] | None = None,
    imu_adapter: ImuSensor | None = None,
    target_track_adapter: TargetTrackSensor | None = None,
) -> Pseudo6Run:
    """Propagate bounded attitude response and sample it with Taoryx's ideal IMU.

    The attitude loop is an explicit reduced-order response law, not a rigid-body
    moment balance or a replica of a missile controller. Environment, gravity,
    and measurement projection stay behind their standard Taoryx boundaries.
    """

    kernel = Pseudo6Kernel(
        profile,
        environment=environment,
        gravity_acceleration=gravity_acceleration,
        imu_adapter=imu_adapter,
        target_track_adapter=target_track_adapter,
    )
    state = kernel.initial_state(mission)
    waypoint = mission.guidance_objective()
    samples: list[Pseudo6Sample] = []
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
    return Pseudo6Run(samples=tuple(samples), termination=termination)
    ####


@dataclass(frozen=True, slots=True)
class Pseudo6AttitudeStepSample:
    """One sample from the bounded single-axis response-law witness."""

    time_s: float
    axis: Pseudo6ResponseAxis
    requested_command_rad: float
    effective_command_rad: float
    command_support_fraction: float
    angle_rad: float
    axis_rate_rad_s: float
    axis_acceleration_rad_s2: float
    acceleration_limited: bool
    rate_limited: bool
    angle_limited: bool


####


@dataclass(frozen=True, slots=True)
class Pseudo6AttitudeStepRun:
    """Bounded single-axis trace using the same update as mission execution."""

    samples: tuple[Pseudo6AttitudeStepSample, ...]
    axis: Pseudo6ResponseAxis
    requested_command_rad: float
    effective_command_rad: float
    command_support_fraction: float
    command_limited: bool
    acceleration_limited: bool
    rate_limited: bool
    angle_limited: bool
    settling_time_s: float | None
    termination: Literal["duration"] = "duration"
    claim_boundary: str = (
        "Bounded response-law witness under one frozen command-support fraction; the fraction may represent a local "
        "runtime force-authority condition but does not reproduce its changing environment, propulsion, or guidance "
        "state. No airframe, actuator, physical autopilot, or robust-stability claim is made."
    )


####


def run_pseudo6_attitude_step(
    profile: ResolvedInterceptorProfile,
    *,
    axis: Pseudo6ResponseAxis = "roll",
    command_step_rad: float = math.radians(10.0),
    duration_s: float = 3.0,
    time_step_s: float = 0.01,
    settling_band_fraction: float = 0.02,
    command_support_fraction: float = 1.0,
) -> Pseudo6AttitudeStepRun:
    """Run one bounded axis step through the exact pseudo-6DOF response update.

    The witness intentionally excludes translation, guidance, propulsion,
    environment, changing force authority, and sensors. It applies one frozen
    command-support fraction after the same pre-support acceleration limiting
    used by mission propagation. It shows when the corresponding local linear
    analysis remains unsaturated and when runtime rate, acceleration, or angle
    bounds take over.
    """

    numeric = (
        command_step_rad,
        duration_s,
        time_step_s,
        settling_band_fraction,
        command_support_fraction,
    )
    if any(not math.isfinite(value) for value in numeric):
        raise ValueError("pseudo-6DOF attitude-step inputs must be finite")
    if command_step_rad == 0.0 or abs(command_step_rad) > math.pi:
        raise ValueError("command_step_rad must be nonzero and no greater than pi in magnitude")
    if duration_s <= 0.0 or time_step_s <= 0.0:
        raise ValueError("duration_s and time_step_s must be positive")
    if not 0.0 < settling_band_fraction < 1.0:
        raise ValueError("settling_band_fraction must lie strictly between zero and one")
    if not 0.0 <= command_support_fraction <= 1.0:
        raise ValueError("command_support_fraction must lie in [0, 1]")

    axis_index = {"roll": 0, "pitch": 1, "yaw": 2}[axis]
    maximum_bank = profile.number("max_bank_angle_rad")
    effective_command = _effective_axis_command(
        axis,
        command_step_rad,
        maximum_bank_angle_rad=maximum_bank,
    )
    command_limited = not math.isclose(command_step_rad, effective_command, abs_tol=1.0e-12)
    attitude = [0.0, 0.0, 0.0]
    rates = [0.0, 0.0, 0.0]
    command = [0.0, 0.0, 0.0]
    command[axis_index] = effective_command
    natural_frequency = profile.number("attitude_bandwidth_rad_s")
    damping_ratio = profile.number("attitude_damping_ratio")
    maximum_acceleration = profile.number("max_body_acceleration_rad_s2")
    maximum_rate = profile.number("max_body_rate_rad_s")
    samples: list[Pseudo6AttitudeStepSample] = []
    time_s = 0.0
    prior_rate_limited = False
    prior_angle_limited = False
    any_acceleration_limited = False
    any_rate_limited = False
    any_angle_limited = False

    while True:
        accelerations, acceleration_limited = _response_acceleration(
            attitude,
            rates,
            (command[0], command[1], command[2]),
            natural_frequency=natural_frequency,
            damping_ratio=damping_ratio,
            max_acceleration=maximum_acceleration,
        )
        body_accelerations = _body_rates(attitude, accelerations)
        largest_body_acceleration = max(abs(item) for item in body_accelerations)
        if largest_body_acceleration > maximum_acceleration:
            scale = maximum_acceleration / largest_body_acceleration
            accelerations = (
                accelerations[0] * scale,
                accelerations[1] * scale,
                accelerations[2] * scale,
            )
            acceleration_limited = True
        accelerations = (
            accelerations[0] * command_support_fraction,
            accelerations[1] * command_support_fraction,
            accelerations[2] * command_support_fraction,
        )
        any_acceleration_limited = any_acceleration_limited or acceleration_limited
        samples.append(
            Pseudo6AttitudeStepSample(
                time_s=time_s,
                axis=axis,
                requested_command_rad=command_step_rad,
                effective_command_rad=effective_command,
                command_support_fraction=command_support_fraction,
                angle_rad=attitude[axis_index],
                axis_rate_rad_s=rates[axis_index],
                axis_acceleration_rad_s2=accelerations[axis_index],
                acceleration_limited=acceleration_limited,
                rate_limited=prior_rate_limited,
                angle_limited=prior_angle_limited,
            )
        )
        if time_s >= duration_s - 1.0e-12:
            break

        step = min(time_step_s, duration_s - time_s)
        rate_limited = False
        for index in range(3):
            raw_rate = rates[index] + accelerations[index] * step
            rates[index] = _clamp(raw_rate, -maximum_rate, maximum_rate)
            rate_limited = rate_limited or not math.isclose(raw_rate, rates[index], abs_tol=1.0e-12)
        proposed_body_rates = _body_rates(attitude, rates)
        largest_body_rate = max(abs(item) for item in proposed_body_rates)
        if largest_body_rate > maximum_rate:
            scale = maximum_rate / largest_body_rate
            rates = [item * scale for item in rates]
            rate_limited = True
        any_rate_limited = any_rate_limited or rate_limited

        raw_angle = attitude[axis_index] + rates[axis_index] * step
        attitude[axis_index] = _bounded_axis_angle(
            axis,
            raw_angle,
            maximum_bank_angle_rad=maximum_bank,
        )
        angle_limited = axis != "yaw" and not math.isclose(
            raw_angle,
            attitude[axis_index],
            abs_tol=1.0e-12,
        )
        any_angle_limited = any_angle_limited or angle_limited
        prior_rate_limited = rate_limited
        prior_angle_limited = angle_limited
        time_s = round(time_s + step, 12)

    return Pseudo6AttitudeStepRun(
        samples=tuple(samples),
        axis=axis,
        requested_command_rad=command_step_rad,
        effective_command_rad=effective_command,
        command_support_fraction=command_support_fraction,
        command_limited=command_limited,
        acceleration_limited=any_acceleration_limited,
        rate_limited=any_rate_limited,
        angle_limited=any_angle_limited,
        settling_time_s=_step_settling_time(
            samples,
            effective_command,
            settling_band_fraction,
        ),
    )
    ####


def _effective_axis_command(
    axis: Pseudo6ResponseAxis,
    command_rad: float,
    *,
    maximum_bank_angle_rad: float,
) -> float:
    if axis == "roll":
        return _clamp(command_rad, -maximum_bank_angle_rad, maximum_bank_angle_rad)
    if axis == "pitch":
        return _clamp(command_rad, -_PITCH_LIMIT_RAD, _PITCH_LIMIT_RAD)
    return _wrap_angle(command_rad)
    ####


def _bounded_axis_angle(
    axis: Pseudo6ResponseAxis,
    angle_rad: float,
    *,
    maximum_bank_angle_rad: float,
) -> float:
    if axis == "roll":
        return _clamp(angle_rad, -maximum_bank_angle_rad, maximum_bank_angle_rad)
    if axis == "pitch":
        return _clamp(angle_rad, -_PITCH_LIMIT_RAD, _PITCH_LIMIT_RAD)
    return _wrap_angle(angle_rad)
    ####


def _step_settling_time(
    samples: list[Pseudo6AttitudeStepSample],
    command_rad: float,
    settling_band_fraction: float,
) -> float | None:
    tolerance = max(abs(command_rad) * settling_band_fraction, 1.0e-12)
    last_outside = -1
    for index, sample in enumerate(samples):
        error = _wrap_angle(command_rad - sample.angle_rad) if sample.axis in {"roll", "yaw"} else command_rad - sample.angle_rad
        if abs(error) > tolerance:
            last_outside = index
    settled_index = last_outside + 1
    if settled_index >= len(samples):
        return None
    return samples[settled_index].time_s
    ####


def _attitude_command(
    velocity: list[float] | tuple[float, float, float],
    target_direction: tuple[float, float, float],
    lateral_acceleration: float,
    *,
    max_bank_angle: float,
) -> tuple[float, float, float]:
    horizontal = math.hypot(target_direction[0], target_direction[1])
    pitch = _clamp(
        math.atan2(target_direction[2], max(horizontal, 1.0e-12)),
        -_PITCH_LIMIT_RAD,
        _PITCH_LIMIT_RAD,
    )
    yaw = math.atan2(target_direction[1], target_direction[0])
    turn_sign = math.copysign(
        1.0,
        velocity[0] * target_direction[1] - velocity[1] * target_direction[0],
    )
    roll = turn_sign * min(math.atan2(lateral_acceleration, _STANDARD_GRAVITY_MPS2), max_bank_angle)
    if lateral_acceleration <= 1.0e-12:
        roll = 0.0
    return (roll, pitch, yaw)
    ####


def _response_acceleration(
    attitude: list[float] | tuple[float, float, float],
    rates: list[float] | tuple[float, float, float],
    command: tuple[float, float, float],
    *,
    natural_frequency: float,
    damping_ratio: float,
    max_acceleration: float,
) -> tuple[tuple[float, float, float], bool]:
    accelerations: list[float] = []
    limited = False
    for index, (actual, rate, requested) in enumerate(zip(attitude, rates, command, strict=True)):
        error = _wrap_angle(requested - actual) if index in {0, 2} else requested - actual
        raw = natural_frequency**2 * error - 2.0 * damping_ratio * natural_frequency * rate
        bounded = _clamp(raw, -max_acceleration, max_acceleration)
        accelerations.append(bounded)
        limited = limited or not math.isclose(raw, bounded, abs_tol=1.0e-12)
    return (accelerations[0], accelerations[1], accelerations[2]), limited
    ####


def _truth_point(
    *,
    time_s: float,
    position: list[float] | tuple[float, float, float],
    velocity: list[float] | tuple[float, float, float],
    acceleration: list[float] | tuple[float, float, float],
    gravity_mps2: float,
    attitude: list[float] | tuple[float, float, float],
    body_rates: tuple[float, float, float],
    body_accelerations: tuple[float, float, float],
) -> TruthPoint:
    orientation_eci_from_body = _ECI_FROM_NED @ _ned_from_body(*attitude)
    position_eci = np.asarray((_EARTH_RADIUS_M + position[2], position[1], position[0]))
    velocity_eci = np.asarray((velocity[2], velocity[1], velocity[0]))
    acceleration_eci = np.asarray((acceleration[2], acceleration[1], acceleration[0]))
    gravity_eci = np.asarray((-gravity_mps2, 0.0, 0.0))
    return TruthPoint(
        time_s=time_s,
        position_eci_m=position_eci,
        velocity_eci_mps=velocity_eci,
        velocity_without_gravity_eci_mps=velocity_eci,
        orientation_eci_from_body=orientation_eci_from_body,
        gravity_eci_mps2=gravity_eci,
        angular_rate_body_radps=np.asarray(body_rates),
        acceleration_eci_mps2=acceleration_eci,
        angular_acceleration_body_radps2=np.asarray(body_accelerations),
    )
    ####


def _ned_from_body(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cos_roll, sin_roll = math.cos(roll), math.sin(roll)
    cos_pitch, sin_pitch = math.cos(pitch), math.sin(pitch)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        (
            (
                cos_pitch * cos_yaw,
                sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw,
                cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw,
            ),
            (
                cos_pitch * sin_yaw,
                sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw,
                cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw,
            ),
            (-sin_pitch, sin_roll * cos_pitch, cos_roll * cos_pitch),
        )
    )
    ####


def _direction_from_attitude(
    attitude: list[float] | tuple[float, float, float],
) -> tuple[float, float, float]:
    body_x_ned = _ned_from_body(*attitude)[:, 0]
    return (float(body_x_ned[0]), float(body_x_ned[1]), float(-body_x_ned[2]))
    ####


def _body_air_relative_flow(
    air_velocity_local_neu_mps: tuple[float, float, float],
    attitude_rad: list[float] | tuple[float, float, float],
) -> tuple[bool, tuple[float, float, float], float, float]:
    """Project local air-relative velocity into forward/right/down body axes.

    Angle of attack is ``atan2(w, u)`` and sideslip is
    ``atan2(v, hypot(u, w))``. At zero airspeed the direction is undefined, so
    the validity flag is false and the reported components and angles are zero.
    """

    if _norm(air_velocity_local_neu_mps) <= 1.0e-12:
        return False, (0.0, 0.0, 0.0), 0.0, 0.0
    air_velocity_ned = np.asarray(
        (
            air_velocity_local_neu_mps[0],
            air_velocity_local_neu_mps[1],
            -air_velocity_local_neu_mps[2],
        )
    )
    body_velocity = _ned_from_body(*attitude_rad).T @ air_velocity_ned
    u, v, w = (float(item) for item in body_velocity)
    angle_of_attack = math.atan2(w, u)
    sideslip_angle = math.atan2(v, math.hypot(u, w))
    return True, (u, v, w), angle_of_attack, sideslip_angle
    ####


def _body_rates(
    attitude: list[float] | tuple[float, float, float],
    euler_rates: list[float] | tuple[float, float, float],
) -> tuple[float, float, float]:
    roll, pitch, _ = attitude
    roll_rate, pitch_rate, yaw_rate = euler_rates
    return (
        roll_rate - yaw_rate * math.sin(pitch),
        pitch_rate * math.cos(roll) + yaw_rate * math.sin(roll) * math.cos(pitch),
        -pitch_rate * math.sin(roll) + yaw_rate * math.cos(roll) * math.cos(pitch),
    )
    ####


def _imu_values(payload: ImuIncrement | None) -> tuple[float, float, float, float, float, float, float]:
    if payload is None:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    return (
        payload.dt_s,
        float(payload.delta_v_body_mps[0]),
        float(payload.delta_v_body_mps[1]),
        float(payload.delta_v_body_mps[2]),
        float(payload.delta_theta_body_rad[0]),
        float(payload.delta_theta_body_rad[1]),
        float(payload.delta_theta_body_rad[2]),
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


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)
    ####


def _wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi
    ####


def _standard_gravity(altitude_m: float) -> float:
    return atmos_gravity_inverse_square(_STANDARD_GRAVITY_MPS2, _EARTH_RADIUS_M, altitude_m)
    ####


__all__ = [
    "Pseudo6AttitudeStepRun",
    "Pseudo6AttitudeStepSample",
    "Pseudo6Kernel",
    "Pseudo6ResponseAxis",
    "Pseudo6Run",
    "Pseudo6Sample",
    "Pseudo6State",
    "Pseudo6StepEvaluation",
    "run_pseudo6_attitude_step",
    "run_pseudo6_interceptor",
]
####
