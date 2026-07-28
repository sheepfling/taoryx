"""Small inertial-navigation examples built on Taoryx IMU packets."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .sensors import AccelerationIncrement, GyroIncrement, ImuIncrement, MeasurementPacket


def _vector3(value: np.ndarray, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 3-vector")
    return result.copy()


def _rotation(value: np.ndarray, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (3, 3) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 3x3 matrix")
    if not np.allclose(result.T @ result, np.eye(3), atol=1.0e-8) or not np.isclose(np.linalg.det(result), 1.0, atol=1.0e-8):
        raise ValueError(f"{name} must be a proper rotation matrix")
    return result.copy()


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _exp_so3(rotation_vector: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotation_vector))
    skew = _skew(rotation_vector)
    if angle < 1.0e-8:
        return np.eye(3) + skew + 0.5 * skew @ skew
    sine_over_angle = np.sin(angle) / angle
    one_minus_cosine_over_angle_squared = (1.0 - np.cos(angle)) / (angle * angle)
    return np.eye(3) + sine_over_angle * skew + one_minus_cosine_over_angle_squared * (skew @ skew)


@dataclass(frozen=True, slots=True)
class NavigationState:
    """Nominal ECI inertial state with body-frame error biases."""

    time_s: float
    position_eci_m: np.ndarray
    velocity_eci_mps: np.ndarray
    orientation_eci_from_body: np.ndarray
    accelerometer_bias_body_mps2: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyroscope_bias_body_radps: np.ndarray = field(default_factory=lambda: np.zeros(3))
    body_rate_body_radps: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def __post_init__(self) -> None:
        if not np.isfinite(self.time_s):
            raise ValueError("navigation time must be finite")
        object.__setattr__(self, "position_eci_m", _vector3(self.position_eci_m, "position_eci_m"))
        object.__setattr__(self, "velocity_eci_mps", _vector3(self.velocity_eci_mps, "velocity_eci_mps"))
        object.__setattr__(self, "orientation_eci_from_body", _rotation(self.orientation_eci_from_body, "orientation_eci_from_body"))
        object.__setattr__(self, "accelerometer_bias_body_mps2", _vector3(self.accelerometer_bias_body_mps2, "accelerometer_bias_body_mps2"))
        object.__setattr__(self, "gyroscope_bias_body_radps", _vector3(self.gyroscope_bias_body_radps, "gyroscope_bias_body_radps"))
        object.__setattr__(self, "body_rate_body_radps", _vector3(self.body_rate_body_radps, "body_rate_body_radps"))


@dataclass(frozen=True, slots=True)
class TranslationNavigationState:
    """Translation-only ECI state with no implied attitude estimate."""

    time_s: float
    position_eci_m: np.ndarray
    velocity_eci_mps: np.ndarray

    def __post_init__(self) -> None:
        if not np.isfinite(self.time_s):
            raise ValueError("navigation time must be finite")
        object.__setattr__(self, "position_eci_m", _vector3(self.position_eci_m, "position_eci_m"))
        object.__setattr__(self, "velocity_eci_mps", _vector3(self.velocity_eci_mps, "velocity_eci_mps"))


class TranslationOnlyNavigator:
    """Propagate ECI translation from acceleration-only increments."""

    def __init__(self, initial_state: TranslationNavigationState, gravity_eci_mps2: np.ndarray) -> None:
        self.state = initial_state
        self.gravity_eci_mps2 = _vector3(gravity_eci_mps2, "gravity_eci_mps2")

    def propagate(self, packet: MeasurementPacket[AccelerationIncrement]) -> TranslationNavigationState:
        if not packet.valid:
            return self.state
        if packet.payload is None:
            raise ValueError("valid acceleration packet has no payload")
        increment = packet.payload
        if not np.isclose(self.state.time_s, increment.start_time_s, atol=1.0e-10):
            raise ValueError("acceleration increment does not continue the navigation state")
        dt = increment.dt_s
        old_velocity = self.state.velocity_eci_mps
        new_velocity = old_velocity + increment.delta_v_eci_mps + self.gravity_eci_mps2 * dt
        new_position = self.state.position_eci_m + 0.5 * (old_velocity + new_velocity) * dt
        self.state = TranslationNavigationState(increment.end_time_s, new_position, new_velocity)
        return self.state


@dataclass(frozen=True, slots=True)
class AttitudeNavigationState:
    """Attitude-only ECI state for gyro excitation without translation."""

    time_s: float
    orientation_eci_from_body: np.ndarray

    def __post_init__(self) -> None:
        if not np.isfinite(self.time_s):
            raise ValueError("navigation time must be finite")
        object.__setattr__(self, "orientation_eci_from_body", _rotation(self.orientation_eci_from_body, "orientation_eci_from_body"))


class AttitudeOnlyNavigator:
    """Propagate ECI attitude from body-frame gyro increments only."""

    def __init__(self, initial_state: AttitudeNavigationState) -> None:
        self.state = initial_state

    def propagate(self, packet: MeasurementPacket[GyroIncrement]) -> AttitudeNavigationState:
        if not packet.valid:
            return self.state
        if packet.payload is None:
            raise ValueError("valid gyro packet has no payload")
        increment = packet.payload
        if not np.isclose(self.state.time_s, increment.start_time_s, atol=1.0e-10):
            raise ValueError("gyro increment does not continue the navigation state")
        self.state = AttitudeNavigationState(
            increment.end_time_s,
            self.state.orientation_eci_from_body @ _exp_so3(increment.delta_theta_body_rad),
        )
        return self.state


class DeadReckoningNavigator:
    """First-order strapdown propagation with known gravity and no aiding."""

    def __init__(self, initial_state: NavigationState, gravity_eci_mps2: np.ndarray) -> None:
        self.state = initial_state
        self.gravity_eci_mps2 = _vector3(gravity_eci_mps2, "gravity_eci_mps2")

    def propagate(self, packet: MeasurementPacket[ImuIncrement]) -> NavigationState:
        if not packet.valid:
            return self.state
        if packet.payload is None:
            raise ValueError("valid IMU packet has no payload")
        increment = packet.payload
        self._require_contiguous(increment)
        dt = increment.dt_s
        old_velocity = self.state.velocity_eci_mps
        specific_force_eci = self.state.orientation_eci_from_body @ (increment.delta_v_body_mps / dt)
        new_velocity = old_velocity + specific_force_eci * dt + self.gravity_eci_mps2 * dt
        new_position = self.state.position_eci_m + 0.5 * (old_velocity + new_velocity) * dt
        new_orientation = self.state.orientation_eci_from_body @ _exp_so3(increment.delta_theta_body_rad)
        self.state = NavigationState(
            increment.end_time_s,
            new_position,
            new_velocity,
            new_orientation,
            body_rate_body_radps=increment.delta_theta_body_rad / dt,
        )
        return self.state

    def _require_contiguous(self, increment: ImuIncrement) -> None:
        if not np.isclose(self.state.time_s, increment.start_time_s, atol=1.0e-10):
            raise ValueError("IMU increment does not continue the navigation state")


@dataclass(frozen=True, slots=True)
class MekfNoise:
    """Continuous-time standard deviations for the 15-state MEKF model."""

    accelerometer_noise_density_mps2_sqrt_hz: float = 0.02
    gyroscope_noise_density_radps_sqrt_hz: float = 0.002
    accelerometer_bias_random_walk_mps3_sqrt_hz: float = 0.0002
    gyroscope_bias_random_walk_radps2_sqrt_hz: float = 0.00002

    def __post_init__(self) -> None:
        values = (
            self.accelerometer_noise_density_mps2_sqrt_hz,
            self.gyroscope_noise_density_radps_sqrt_hz,
            self.accelerometer_bias_random_walk_mps3_sqrt_hz,
            self.gyroscope_bias_random_walk_radps2_sqrt_hz,
        )
        if not all(np.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("MEKF noise densities must be finite and nonnegative")


class MultiplicativeEkf:
    """15-state right-error MEKF with position and velocity aiding hooks.

    The nominal state is propagated from IMU increments. The covariance uses a
    first-order continuous-to-discrete model, which is appropriate for this
    example and intentionally not a flight-qualified estimator.
    """

    ERROR_DIMENSION = 15

    def __init__(
        self,
        initial_state: NavigationState,
        initial_covariance: np.ndarray,
        gravity_eci_mps2: np.ndarray,
        *,
        noise: MekfNoise = MekfNoise(),
    ) -> None:
        covariance = np.asarray(initial_covariance, dtype=float)
        if covariance.shape != (self.ERROR_DIMENSION, self.ERROR_DIMENSION) or not np.all(np.isfinite(covariance)):
            raise ValueError("initial_covariance must be a finite 15x15 matrix")
        if not np.allclose(covariance, covariance.T, atol=1.0e-12):
            raise ValueError("initial_covariance must be symmetric")
        if np.linalg.eigvalsh(covariance).min() < -1.0e-10:
            raise ValueError("initial_covariance must be positive semidefinite")
        self.state = initial_state
        self.covariance = covariance.copy()
        self.gravity_eci_mps2 = _vector3(gravity_eci_mps2, "gravity_eci_mps2")
        self.noise = noise

    def propagate(self, packet: MeasurementPacket[ImuIncrement]) -> NavigationState:
        if not packet.valid:
            return self.state
        if packet.payload is None:
            raise ValueError("valid IMU packet has no payload")
        increment = packet.payload
        if not np.isclose(self.state.time_s, increment.start_time_s, atol=1.0e-10):
            raise ValueError("IMU increment does not continue the MEKF state")
        dt = increment.dt_s
        R = self.state.orientation_eci_from_body
        accel_bias = self.state.accelerometer_bias_body_mps2
        gyro_bias = self.state.gyroscope_bias_body_radps
        corrected_delta_v = increment.delta_v_body_mps - accel_bias * dt
        corrected_delta_theta = increment.delta_theta_body_rad - gyro_bias * dt
        specific_force_body = corrected_delta_v / dt
        old_velocity = self.state.velocity_eci_mps
        new_velocity = old_velocity + R @ corrected_delta_v + self.gravity_eci_mps2 * dt
        new_position = self.state.position_eci_m + 0.5 * (old_velocity + new_velocity) * dt
        new_orientation = R @ _exp_so3(corrected_delta_theta)
        self.state = NavigationState(
            increment.end_time_s,
            new_position,
            new_velocity,
            new_orientation,
            accel_bias,
            gyro_bias,
            corrected_delta_theta / dt,
        )
        self._propagate_covariance(R, specific_force_body, corrected_delta_theta / dt, dt)
        return self.state

    def update_position(self, position_eci_m: np.ndarray, covariance: np.ndarray) -> NavigationState:
        measurement = _vector3(position_eci_m, "position_eci_m")
        return self._measurement_update(measurement, covariance, slice(0, 3))

    def update_velocity(self, velocity_eci_mps: np.ndarray, covariance: np.ndarray) -> NavigationState:
        measurement = _vector3(velocity_eci_mps, "velocity_eci_mps")
        return self._measurement_update(measurement, covariance, slice(3, 6))

    def _propagate_covariance(self, R: np.ndarray, specific_force_body: np.ndarray, body_rate: np.ndarray, dt: float) -> None:
        F = np.zeros((self.ERROR_DIMENSION, self.ERROR_DIMENSION))
        F[0:3, 3:6] = np.eye(3)
        F[3:6, 6:9] = -R @ _skew(specific_force_body)
        F[3:6, 9:12] = -R
        F[6:9, 6:9] = -_skew(body_rate)
        F[6:9, 12:15] = -np.eye(3)
        Phi = np.eye(self.ERROR_DIMENSION) + F * dt
        G = np.zeros((self.ERROR_DIMENSION, 12))
        G[3:6, 0:3] = R
        G[6:9, 3:6] = -np.eye(3)
        G[9:12, 6:9] = np.eye(3)
        G[12:15, 9:12] = np.eye(3)
        q = np.diag(
            [
                self.noise.accelerometer_noise_density_mps2_sqrt_hz**2,
                self.noise.accelerometer_noise_density_mps2_sqrt_hz**2,
                self.noise.accelerometer_noise_density_mps2_sqrt_hz**2,
                self.noise.gyroscope_noise_density_radps_sqrt_hz**2,
                self.noise.gyroscope_noise_density_radps_sqrt_hz**2,
                self.noise.gyroscope_noise_density_radps_sqrt_hz**2,
                self.noise.accelerometer_bias_random_walk_mps3_sqrt_hz**2,
                self.noise.accelerometer_bias_random_walk_mps3_sqrt_hz**2,
                self.noise.accelerometer_bias_random_walk_mps3_sqrt_hz**2,
                self.noise.gyroscope_bias_random_walk_radps2_sqrt_hz**2,
                self.noise.gyroscope_bias_random_walk_radps2_sqrt_hz**2,
                self.noise.gyroscope_bias_random_walk_radps2_sqrt_hz**2,
            ]
        )
        self.covariance = Phi @ self.covariance @ Phi.T + G @ q @ G.T * dt
        self.covariance = 0.5 * (self.covariance + self.covariance.T)

    def _measurement_update(self, measurement: np.ndarray, covariance: np.ndarray, indices: slice) -> NavigationState:
        measurement_covariance = np.asarray(covariance, dtype=float)
        if measurement_covariance.shape != (3, 3) or not np.all(np.isfinite(measurement_covariance)):
            raise ValueError("measurement covariance must be a finite 3x3 matrix")
        if not np.allclose(measurement_covariance, measurement_covariance.T, atol=1.0e-12):
            raise ValueError("measurement covariance must be symmetric")
        H = np.zeros((3, self.ERROR_DIMENSION))
        H[:, indices] = np.eye(3)
        nominal = self.state.position_eci_m if indices.start == 0 else self.state.velocity_eci_mps
        innovation = measurement - nominal
        innovation_covariance = H @ self.covariance @ H.T + measurement_covariance
        gain = np.linalg.solve(innovation_covariance, H @ self.covariance).T
        error = gain @ innovation
        self._inject_error(error)
        identity = np.eye(self.ERROR_DIMENSION)
        residual_projection = identity - gain @ H
        self.covariance = residual_projection @ self.covariance @ residual_projection.T + gain @ measurement_covariance @ gain.T
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        return self.state

    def _inject_error(self, error: np.ndarray) -> None:
        self.state = NavigationState(
            self.state.time_s,
            self.state.position_eci_m + error[0:3],
            self.state.velocity_eci_mps + error[3:6],
            self.state.orientation_eci_from_body @ _exp_so3(error[6:9]),
            self.state.accelerometer_bias_body_mps2 + error[9:12],
            self.state.gyroscope_bias_body_radps + error[12:15],
            self.state.body_rate_body_radps,
        )
