"""Small, explicit rigid-body 6-DOF extension model for TAORYX.

This module is intentionally separate from the manual-bounded TAOS point-mass
kernel.  It provides the state and force/moment contracts needed to build
synthetic 6-DOF scenarios without claiming historical TAOS compatibility.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass

from .contracts import Frame, FrameVector3, Vector3
from .modes import FidelitySetupError, Quaternion

RIGID_BODY_STATE_NAMES = (
    "x",
    "y",
    "z",
    "vx",
    "vy",
    "vz",
    "qw",
    "qx",
    "qy",
    "qz",
    "wx",
    "wy",
    "wz",
    "mass",
    "propellant_mass",
    "heat_load",
    "peak_heat_rate",
)


@dataclass(frozen=True, slots=True)
class RigidBody6DofState:
    """ECIC translation plus body attitude, body rates, mass, and heating."""

    time: float
    position: FrameVector3
    velocity: FrameVector3
    attitude: Quaternion
    body_rate: Vector3
    mass: float
    propellant_mass: float
    heat_load: float = 0.0
    peak_heat_rate: float = 0.0

    def __post_init__(self) -> None:
        if self.position.frame is not Frame.ECIC or self.velocity.frame is not Frame.ECIC:
            raise FidelitySetupError(
                "rigid-frame-mismatch",
                "rigid-body position and velocity must use ECIC",
                "convert the inertial state to ECIC before constructing the rigid-body state",
                field="position/velocity",
            )
        if not math.isfinite(self.time):
            raise FidelitySetupError(
                "rigid-time-nonfinite",
                "rigid-body time must be finite",
                "provide a finite integration time in seconds",
                field="time",
            )
        if not math.isfinite(self.mass) or self.mass <= 0.0:
            raise FidelitySetupError(
                "invalid-mass",
                "rigid-body mass must be positive and finite",
                "declare total mass in kilograms and keep it positive at every stage boundary",
                field="mass",
            )
        if not math.isfinite(self.propellant_mass) or self.propellant_mass < 0.0 or self.propellant_mass > self.mass:
            raise FidelitySetupError(
                "invalid-propellant-mass",
                "rigid-body propellant mass must be finite and within total mass",
                "set propellant mass to a value in [0, total mass] and update it at stage or cutoff events",
                field="propellant_mass",
            )
        if not all(math.isfinite(value) and value >= 0.0 for value in (self.heat_load, self.peak_heat_rate)):
            raise FidelitySetupError(
                "invalid-thermal-state",
                "rigid-body thermal state must be finite and nonnegative",
                "initialize heat load and peak heat rate in nonnegative SI units",
                field="heat_load/peak_heat_rate",
            )
        ####

    def to_values(self) -> tuple[float, ...]:
        """Pack the state in the stable runtime integration order."""

        return (
            self.position.vector.x,
            self.position.vector.y,
            self.position.vector.z,
            self.velocity.vector.x,
            self.velocity.vector.y,
            self.velocity.vector.z,
            self.attitude.w,
            self.attitude.x,
            self.attitude.y,
            self.attitude.z,
            self.body_rate.x,
            self.body_rate.y,
            self.body_rate.z,
            self.mass,
            self.propellant_mass,
            self.heat_load,
            self.peak_heat_rate,
        )
        ####

    @classmethod
    def from_values(cls, time: float, values: Sequence[float]) -> RigidBody6DofState:
        """Unpack a runtime vector and normalize the integrated quaternion."""

        if len(values) != len(RIGID_BODY_STATE_NAMES):
            raise FidelitySetupError(
                "rigid-state-length-mismatch",
                f"rigid-body state requires {len(RIGID_BODY_STATE_NAMES)} values, received {len(values)}",
                "provide position, velocity, quaternion, body-rate, mass, propellant, and thermal channels in RIGID_BODY_STATE_NAMES order",
                field="state",
            )
        return cls(
            time,
            FrameVector3(Vector3(*values[0:3]), Frame.ECIC),
            FrameVector3(Vector3(*values[3:6]), Frame.ECIC),
            Quaternion(*values[6:10]).normalized(),
            Vector3(*values[10:13]),
            values[13],
            values[14],
            values[15],
            values[16],
        )
        ####
    ####


@dataclass(frozen=True, slots=True)
class RigidBodyForceMoment:
    """Body-frame force and moment plus mass-flow and heating observables."""

    force_body: Vector3
    moment_body: Vector3
    propellant_mass_rate: float = 0.0
    heat_rate: float = 0.0
    aero_force_body: Vector3 | None = None
    propulsion_force_body: Vector3 | None = None
    aero_moment_body: Vector3 | None = None
    propulsion_moment_body: Vector3 | None = None

    def __post_init__(self) -> None:
        vectors = {
            "force_body": self.force_body,
            "moment_body": self.moment_body,
            "aero_force_body": self.aero_force_body,
            "propulsion_force_body": self.propulsion_force_body,
            "aero_moment_body": self.aero_moment_body,
            "propulsion_moment_body": self.propulsion_moment_body,
        }
        for name, vector in vectors.items():
            if vector is not None and not all(math.isfinite(value) for value in (vector.x, vector.y, vector.z)):
                raise FidelitySetupError(
                    "nonfinite-load",
                    f"rigid-body {name} components must be finite",
                    "return finite body-frame force and moment components from the load pipeline",
                    field=name,
                )
        if self.propellant_mass_rate < 0.0 or not math.isfinite(self.propellant_mass_rate):
            raise FidelitySetupError(
                "invalid-propellant-rate",
                "propellant mass rate must be finite and nonnegative",
                "return a nonnegative kg/s consumption rate; do not encode mass ejection as a negative rate",
                field="propellant_mass_rate",
            )
        if self.heat_rate < 0.0 or not math.isfinite(self.heat_rate):
            raise FidelitySetupError(
                "invalid-heat-rate",
                "heat rate must be finite and nonnegative",
                "return a nonnegative SI heat-rate observable from the load pipeline",
                field="heat_rate",
            )
        ####
    ####


@dataclass(frozen=True, slots=True)
class ThermalLimits:
    """Hard limits used by an entry controller or event policy."""

    maximum_heat_rate: float
    maximum_heat_load: float
    maximum_dynamic_pressure: float | None = None

    def __post_init__(self) -> None:
        if self.maximum_heat_rate <= 0.0 or self.maximum_heat_load <= 0.0:
            raise ValueError("thermal limits must be positive")
        if self.maximum_dynamic_pressure is not None and self.maximum_dynamic_pressure <= 0.0:
            raise ValueError("dynamic-pressure limit must be positive")
        ####
    ####


@dataclass(frozen=True, slots=True)
class ThermalAssessment:
    """Current margins for heat-rate and integrated heat-load limits."""

    heat_rate_margin: float
    heat_load_margin: float
    safe: bool


ForceMomentProvider = Callable[[RigidBody6DofState], RigidBodyForceMoment]
GravityProvider = Callable[[RigidBody6DofState], Vector3]
InertiaProvider = Callable[[RigidBody6DofState], Vector3]
InertiaMatrix = tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
InertiaMatrixProvider = Callable[[RigidBody6DofState], InertiaMatrix]


@dataclass(frozen=True, slots=True)
class RigidBody6DofModel:
    """Evaluate a rigid-body state derivative with diagonal or full inertia.

    Forces and moments are supplied in body coordinates. Gravity is supplied
    in ECIC coordinates. This keeps the model usable for stage, coast, entry,
    and terminal-guidance phases while leaving vehicle-specific aerodynamics
    and propulsion outside the integrator.
    """

    inertia: Vector3
    force_moment: ForceMomentProvider
    gravity: GravityProvider = lambda state: Vector3(0.0, 0.0, 0.0)
    dry_mass: float | None = None
    inertia_provider: InertiaProvider | None = None
    inertia_matrix_provider: InertiaMatrixProvider | None = None

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) and value > 0.0 for value in (self.inertia.x, self.inertia.y, self.inertia.z)):
            raise FidelitySetupError(
                "invalid-inertia",
                "all principal moments of inertia must be positive",
                "supply source-backed principal inertia in kg m^2 about the declared center of gravity",
                field="inertia",
            )
        if self.dry_mass is not None and (not math.isfinite(self.dry_mass) or self.dry_mass <= 0.0):
            raise FidelitySetupError(
                "invalid-dry-mass",
                "dry mass must be positive and finite",
                "set dry mass in kilograms below the initial total mass",
                field="dry_mass",
            )
        if self.inertia_provider is not None and self.inertia_matrix_provider is not None:
            raise FidelitySetupError(
                "multiple-inertia-providers",
                "rigid-body model cannot use both diagonal and full inertia providers",
                "select one source-backed inertia representation for the runtime model",
                field="inertia_provider/inertia_matrix_provider",
            )
        ####

    def inertia_at(self, state: RigidBody6DofState) -> Vector3:
        """Return the source-backed inertia at the current mass/configuration."""

        inertia = self.inertia_provider(state) if self.inertia_provider is not None else self.inertia
        if not all(math.isfinite(value) and value > 0.0 for value in (inertia.x, inertia.y, inertia.z)):
            raise FidelitySetupError(
                "runtime-inertia-invalid",
                "runtime inertia provider returned non-positive or non-finite values",
                "validate the mass-property schedule and keep every inertia endpoint positive and finite",
                field="inertia_provider",
            )
        return inertia
        ####

    def inertia_matrix_at(self, state: RigidBody6DofState) -> InertiaMatrix:
        """Return a validated full inertia matrix at the current state."""

        if self.inertia_matrix_provider is None:
            diagonal = self.inertia_at(state)
            return (
                (diagonal.x, 0.0, 0.0),
                (0.0, diagonal.y, 0.0),
                (0.0, 0.0, diagonal.z),
            )
        matrix = self.inertia_matrix_provider(state)
        _validate_inertia_matrix(matrix)
        return matrix
        ####

    def _load(self, state: RigidBody6DofState) -> RigidBodyForceMoment:
        """Resolve a load and stop propellant flow at dry-mass depletion."""

        load = self.force_moment(state)
        if self.dry_mass is None or (
            state.propellant_mass > 1.0e-8
            and state.mass > self.dry_mass + 1.0e-8
        ):
            return load
        return RigidBodyForceMoment(
            load.force_body,
            load.moment_body,
            0.0,
            load.heat_rate,
            load.aero_force_body,
            load.propulsion_force_body,
            load.aero_moment_body,
            load.propulsion_moment_body,
        )
        ####

    def derivative(self, state: RigidBody6DofState) -> tuple[float, ...]:
        """Return the full derivative in ``RIGID_BODY_STATE_NAMES`` order."""

        load = self._load(state)
        gravity = self.gravity(state)
        force_ecic = state.attitude.rotate(load.force_body) + gravity.scaled(state.mass)
        acceleration = force_ecic.scaled(1.0 / state.mass)
        inertia = self.inertia_matrix_at(state)
        angular_momentum = _matrix_vector(inertia, state.body_rate)
        try:
            gyroscopic_term = state.body_rate.cross(angular_momentum)
        except ValueError as error:
            raise ValueError(
                "rigid-body rotational cross term became non-finite: "
                f"body_rate={state.body_rate!r}, angular_momentum={angular_momentum!r}"
            ) from error
        angular_acceleration = _solve_inertia(inertia, load.moment_body - gyroscopic_term)
        quaternion_rate = state.attitude.derivative(state.body_rate)
        derivative = (
            state.velocity.vector.x,
            state.velocity.vector.y,
            state.velocity.vector.z,
            acceleration.x,
            acceleration.y,
            acceleration.z,
            quaternion_rate.w * 0.5,
            quaternion_rate.x * 0.5,
            quaternion_rate.y * 0.5,
            quaternion_rate.z * 0.5,
            angular_acceleration.x,
            angular_acceleration.y,
            angular_acceleration.z,
            -load.propellant_mass_rate,
            -load.propellant_mass_rate,
            load.heat_rate,
            max(0.0, load.heat_rate - state.peak_heat_rate),
        )
        if not all(math.isfinite(value) for value in derivative):
            bad = tuple(
                name
                for name, value in zip(RIGID_BODY_STATE_NAMES, derivative, strict=True)
                if not math.isfinite(value)
            )
            raise ValueError("rigid-body derivative contains non-finite values: " + ", ".join(bad))
        return derivative
        ####

    def observables(self, state: RigidBody6DofState) -> dict[str, float]:
        """Return force, moment, acceleration, heating, and attitude channels.

        These values are derived from the same force/moment evaluation used by
        :meth:`derivative`, so runtime telemetry and the integrated state do
        not require a second physics implementation.
        """

        load = self._load(state)
        gravity = self.gravity(state)
        force_ecic = state.attitude.rotate(load.force_body)
        total_force_ecic = force_ecic + gravity.scaled(state.mass)
        acceleration = total_force_ecic.scaled(1.0 / state.mass)
        inertia = self.inertia_matrix_at(state)
        gravity_force_body = state.attitude.conjugate().rotate(gravity.scaled(state.mass))
        total_force_body = load.force_body + gravity_force_body
        acceleration_body = state.attitude.conjugate().rotate(acceleration)
        # ``acceleration_body`` is the inertial acceleration resolved in body
        # axes, not the time derivative of the body velocity components.  The
        # latter would require the additional omega-cross-velocity term.  The
        # Newton equation in this observable contract is therefore the direct
        # body-resolved inertial acceleration balance:
        #
        #     m R_BI a_I - (F_aero,B + F_prop,B + F_gravity,B) = 0.
        #
        # Keeping this distinct prevents rotating-frame transport terms from
        # being counted twice in closure telemetry.
        force_residual = acceleration_body.scaled(state.mass) - total_force_body
        attitude = state.attitude.normalized()
        roll = math.atan2(2.0 * (attitude.w * attitude.x + attitude.y * attitude.z), 1.0 - 2.0 * (attitude.x * attitude.x + attitude.y * attitude.y))
        pitch_argument = 2.0 * (attitude.w * attitude.y - attitude.z * attitude.x)
        pitch = math.asin(max(-1.0, min(1.0, pitch_argument)))
        yaw = math.atan2(2.0 * (attitude.w * attitude.z + attitude.x * attitude.y), 1.0 - 2.0 * (attitude.y * attitude.y + attitude.z * attitude.z))
        aero_force = load.aero_force_body or Vector3(0.0, 0.0, 0.0)
        propulsion_force = load.propulsion_force_body or (load.force_body - aero_force)
        aero_moment = load.aero_moment_body or Vector3(0.0, 0.0, 0.0)
        propulsion_moment = load.propulsion_moment_body or (load.moment_body - aero_moment)
        angular_momentum = _matrix_vector(inertia, state.body_rate)
        try:
            gyroscopic_term = state.body_rate.cross(angular_momentum)
        except ValueError as error:
            raise ValueError(
                "rigid-body observable rotational cross term became non-finite: "
                f"body_rate={state.body_rate!r}, angular_momentum={angular_momentum!r}"
            ) from error
        angular_acceleration = _solve_inertia(inertia, load.moment_body - gyroscopic_term)
        moment_residual = _matrix_vector(inertia, angular_acceleration) + gyroscopic_term - load.moment_body
        force_scale = max(state.mass * gravity.norm(), total_force_body.norm(), 1.0e-12)
        moment_scale = max(load.moment_body.norm(), 1.0)
        return {
            "force_body_x_n": load.force_body.x,
            "force_body_y_n": load.force_body.y,
            "force_body_z_n": load.force_body.z,
            "moment_body_x_nm": load.moment_body.x,
            "moment_body_y_nm": load.moment_body.y,
            "moment_body_z_nm": load.moment_body.z,
            "force_ecic_x_n": force_ecic.x,
            "force_ecic_y_n": force_ecic.y,
            "force_ecic_z_n": force_ecic.z,
            "total_force_ecic_x_n": total_force_ecic.x,
            "total_force_ecic_y_n": total_force_ecic.y,
            "total_force_ecic_z_n": total_force_ecic.z,
            "total_force_ecic_n": total_force_ecic.norm(),
            "aero_force_body_x_n": aero_force.x,
            "aero_force_body_y_n": aero_force.y,
            "aero_force_body_z_n": aero_force.z,
            "propulsion_force_body_x_n": propulsion_force.x,
            "propulsion_force_body_y_n": propulsion_force.y,
            "propulsion_force_body_z_n": propulsion_force.z,
            "gravity_force_body_x_n": gravity_force_body.x,
            "gravity_force_body_y_n": gravity_force_body.y,
            "gravity_force_body_z_n": gravity_force_body.z,
            "total_force_body_x_n": total_force_body.x,
            "total_force_body_y_n": total_force_body.y,
            "total_force_body_z_n": total_force_body.z,
            "aero_moment_body_x_nm": aero_moment.x,
            "aero_moment_body_y_nm": aero_moment.y,
            "aero_moment_body_z_nm": aero_moment.z,
            "propulsion_moment_body_x_nm": propulsion_moment.x,
            "propulsion_moment_body_y_nm": propulsion_moment.y,
            "propulsion_moment_body_z_nm": propulsion_moment.z,
            "total_moment_body_x_nm": load.moment_body.x,
            "total_moment_body_y_nm": load.moment_body.y,
            "total_moment_body_z_nm": load.moment_body.z,
            "translation_equation_residual_n": force_residual.norm(),
            "translation_equation_residual_normalized": force_residual.norm() / force_scale,
            "rotation_equation_residual_nm": moment_residual.norm(),
            "rotation_equation_residual_normalized": moment_residual.norm() / moment_scale,
            "acceleration_ecic_x_m_s2": acceleration.x,
            "acceleration_ecic_y_m_s2": acceleration.y,
            "acceleration_ecic_z_m_s2": acceleration.z,
            "angular_acceleration_body_x_rad_s2": angular_acceleration.x,
            "angular_acceleration_body_y_rad_s2": angular_acceleration.y,
            "angular_acceleration_body_z_rad_s2": angular_acceleration.z,
            "propellant_mass_rate_kg_s": load.propellant_mass_rate,
            "inertia_x_kg_m2": inertia[0][0],
            "inertia_y_kg_m2": inertia[1][1],
            "inertia_z_kg_m2": inertia[2][2],
            "inertia_xz_kg_m2": inertia[0][2],
            "heat_rate_w_m2": load.heat_rate,
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "yaw_deg": math.degrees(yaw),
        }
        ####

    def runtime_derivative(self) -> Callable[[object], tuple[float, ...]]:
        """Return a callback compatible with ``RuntimeVehicle.derivative``."""

        def evaluate(runtime_state: object) -> tuple[float, ...]:
            time = float(getattr(runtime_state, "time"))
            values = getattr(runtime_state, "values")
            return self.derivative(RigidBody6DofState.from_values(time, values))
        ####

        return evaluate
        ####
    ####


def assess_thermal_limits(state: RigidBody6DofState, limits: ThermalLimits, heat_rate: float) -> ThermalAssessment:
    """Return explicit thermal margins for guidance and event logic."""

    rate_margin = limits.maximum_heat_rate - heat_rate
    load_margin = limits.maximum_heat_load - state.heat_load
    return ThermalAssessment(rate_margin, load_margin, rate_margin >= 0.0 and load_margin >= 0.0)
####


def _validate_inertia_matrix(matrix: InertiaMatrix) -> None:
    """Validate the symmetric-positive-definite rigid-body inertia contract."""

    if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise FidelitySetupError(
            "invalid-inertia-matrix-shape",
            "rigid-body inertia matrix must be 3x3",
            "provide Ixx/Iyy/Izz and the symmetric products of inertia",
            field="inertia_matrix",
        )
    if not all(math.isfinite(value) for row in matrix for value in row):
        raise FidelitySetupError(
            "invalid-inertia-matrix-values",
            "rigid-body inertia matrix must contain only finite values",
            "remove non-finite mass-property entries before runtime construction",
            field="inertia_matrix",
        )
    if any(matrix[row][column] != matrix[column][row] for row in range(3) for column in range(3)):
        raise FidelitySetupError(
            "nonsymmetric-inertia-matrix",
            "rigid-body inertia matrix must be symmetric",
            "declare matching products of inertia on both sides of the diagonal",
            field="inertia_matrix",
        )
    leading_two = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[0][1]
    determinant = (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[1][2])
        - matrix[0][1] * (matrix[0][1] * matrix[2][2] - matrix[1][2] * matrix[0][2])
        + matrix[0][2] * (matrix[0][1] * matrix[1][2] - matrix[1][1] * matrix[0][2])
    )
    if matrix[0][0] <= 0.0 or leading_two <= 0.0 or determinant <= 0.0:
        raise FidelitySetupError(
            "nonpositive-inertia-matrix",
            "rigid-body inertia matrix must be positive definite",
            "check principal moments and products of inertia against the physical mass distribution",
            field="inertia_matrix",
        )
    ####


def _matrix_vector(matrix: InertiaMatrix, vector: Vector3) -> Vector3:
    """Multiply a 3x3 inertia matrix by a body-rate vector."""

    return Vector3(
        matrix[0][0] * vector.x + matrix[0][1] * vector.y + matrix[0][2] * vector.z,
        matrix[1][0] * vector.x + matrix[1][1] * vector.y + matrix[1][2] * vector.z,
        matrix[2][0] * vector.x + matrix[2][1] * vector.y + matrix[2][2] * vector.z,
    )
    ####


def _solve_inertia(matrix: InertiaMatrix, vector: Vector3) -> Vector3:
    """Solve ``matrix * x = vector`` for the angular acceleration."""

    determinant = (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )
    if abs(determinant) <= 1.0e-18:
        raise FidelitySetupError(
            "singular-inertia-matrix",
            "rigid-body inertia matrix is singular at runtime",
            "provide a positive-definite inertia matrix for the current mass configuration",
            field="inertia_matrix",
        )
    inverse = (
        (
            (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) / determinant,
            (matrix[0][2] * matrix[2][1] - matrix[0][1] * matrix[2][2]) / determinant,
            (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]) / determinant,
        ),
        (
            (matrix[1][2] * matrix[2][0] - matrix[1][0] * matrix[2][2]) / determinant,
            (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]) / determinant,
            (matrix[0][2] * matrix[1][0] - matrix[0][0] * matrix[1][2]) / determinant,
        ),
        (
            (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]) / determinant,
            (matrix[0][1] * matrix[2][0] - matrix[0][0] * matrix[2][1]) / determinant,
            (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]) / determinant,
        ),
    )
    return _matrix_vector(inverse, vector)
    ####
