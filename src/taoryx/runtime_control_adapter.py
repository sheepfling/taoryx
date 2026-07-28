"""Adapters that expose an executable rigid-body runtime plant to control tools.

The runtime normally owns its force/moment closure internally.  This adapter
opens a deliberately narrow, local control-analysis view without duplicating
aircraft equations: every derivative and local effectiveness probe invokes
the same lowered nonlinear vehicle used by the runtime.

It is suitable for a source-trim neighbourhood, not a declaration that the
full mission controller has been scheduled across an aircraft envelope.
"""

from __future__ import annotations

import math
from collections.abc import Iterator, Mapping, MutableMapping
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Literal, cast

from .contracts import Frame, FrameVector3, Vector3
from .control_allocation import (
    ControlPlantAdapter,
    EffectorEffectiveness,
    EffectorLimits,
    PhysicalAllocationStep,
    ProvenancedLinearization,
    allocate_and_advance_wrench,
    finite_difference_linearization_with_provenance,
)
from .modes import Quaternion
from .rigid_body import RIGID_BODY_STATE_NAMES, RigidBody6DofState
from .runtime.common import Derivative, RuntimeState, RuntimeVehicle
from .trim import TrimResult, TrimSpec, solve_trim

_LOCAL_STATE_NAMES = (
    "roll_error_rad",
    "pitch_error_rad",
    "yaw_error_rad",
    "u_m_s",
    "v_m_s",
    "w_m_s",
    "p_rad_s",
    "q_rad_s",
    "r_rad_s",
)


@dataclass(slots=True)
class RuntimeRigidBodyLocalPlant(ControlPlantAdapter):
    """Local body-velocity/rate adapter around one lowered rigid-body vehicle.

    Position is held at the supplied source operating point.  Body velocity,
    body rate, and a source-relative roll/pitch/yaw attitude-error state are
    allowed to vary.  This produces a local attitude-and-rate plant whose
    derivatives come from the same table-backed nonlinear runtime used by
    the mission simulator.  It does not inject a controller wrench.

    The attitude error uses a source-relative 3-2-1 Euler coordinate only in
    this small-neighbourhood adapter.  It is not a replacement for the
    runtime's integrated quaternion state and is never used outside the
    declared local validity region.
    """

    id: str
    revision: str
    vehicle: RuntimeVehicle
    source_state: RigidBody6DofState
    inertia_kg_m2: Vector3
    reference_length_m: float
    effector_limits: Mapping[str, EffectorLimits]
    effectiveness_steps: Mapping[str, float]
    state_bounds: Mapping[str, tuple[float, float]] = field(default_factory=dict)
    allocation_wrench_weights: Mapping[str, float] = field(default_factory=dict)
    allocation_regularization: float = 1.0e-10
    allocation_feasibility_tolerance: float = 1.0e-6
    trim_residual_mode: Literal["body_force_equilibrium", "steady_direction_glide"] = "body_force_equilibrium"
    _control_values: MutableMapping[str, float] = field(init=False, repr=False)
    _source_local_state: dict[str, float] = field(init=False, repr=False)
    _derivative: Derivative = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if not self.id.strip() or not self.revision.strip():
            raise ValueError("runtime control adapter requires a stable id and revision")
        derivative = self.vehicle.derivative
        if derivative is None:
            raise ValueError("runtime control adapter requires a vehicle derivative")
        self._derivative = derivative
        if not math.isfinite(self.reference_length_m) or self.reference_length_m <= 0.0:
            raise ValueError("runtime control adapter reference length must be finite and positive")
        if not self.effector_limits:
            raise ValueError("runtime control adapter requires at least one physical effector")
        if set(self.effectiveness_steps) != set(self.effector_limits):
            raise ValueError("effectiveness steps must exactly match declared effectors")
        if any(not math.isfinite(float(value)) or float(value) <= 0.0 for value in self.effectiveness_steps.values()):
            raise ValueError("effectiveness steps must be finite and positive")
        raw_controls = self.vehicle.control_values
        if not isinstance(raw_controls, MutableMapping):
            raise ValueError("runtime vehicle controls must be mutable for physical control probing")
        self._control_values = cast(MutableMapping[str, float], raw_controls)
        missing_controls = set(self.effector_limits) - set(self._control_values)
        if missing_controls:
            raise ValueError(f"runtime vehicle lacks declared effectors: {', '.join(sorted(missing_controls))}")
        body_velocity = self.source_state.attitude.conjugate().rotate(self.source_state.velocity.vector)
        self._source_local_state = {
            "roll_error_rad": 0.0,
            "pitch_error_rad": 0.0,
            "yaw_error_rad": 0.0,
            "u_m_s": body_velocity.x,
            "v_m_s": body_velocity.y,
            "w_m_s": body_velocity.z,
            "p_rad_s": self.source_state.body_rate.x,
            "q_rad_s": self.source_state.body_rate.y,
            "r_rad_s": self.source_state.body_rate.z,
        }
        unknown_bounds = set(self.state_bounds) - set(_LOCAL_STATE_NAMES)
        if unknown_bounds:
            raise ValueError(f"local state bounds name unknown channels: {', '.join(sorted(unknown_bounds))}")
        for name, bounds in self.state_bounds.items():
            if not all(math.isfinite(float(value)) for value in bounds) or bounds[0] > bounds[1]:
                raise ValueError(f"invalid local state bounds for {name!r}")
        expected_wrench_axes = {"moment_x_nm", "moment_y_nm", "moment_z_nm"}
        unknown_wrench_axes = set(self.allocation_wrench_weights) - expected_wrench_axes
        if unknown_wrench_axes:
            raise ValueError(
                "allocation wrench weights name unknown axes: " + ", ".join(sorted(unknown_wrench_axes))
            )
        if not math.isfinite(self.allocation_regularization) or self.allocation_regularization < 0.0:
            raise ValueError("allocation regularization must be finite and nonnegative")
        if not math.isfinite(self.allocation_feasibility_tolerance) or self.allocation_feasibility_tolerance <= 0.0:
            raise ValueError("allocation feasibility tolerance must be finite and positive")
        if self.trim_residual_mode not in {"body_force_equilibrium", "steady_direction_glide"}:
            raise ValueError(f"unsupported local trim residual mode {self.trim_residual_mode!r}")
        ####

    @property
    def state_names(self) -> tuple[str, ...]:
        """Return the stable local-state ordering for trim and LQR artifacts."""

        return _LOCAL_STATE_NAMES
        ####

    @property
    def control_names(self) -> tuple[str, ...]:
        """Return the declared physical-effector ordering."""

        return tuple(self.effector_limits)
        ####

    @property
    def source_local_state(self) -> Mapping[str, float]:
        """Return the source operating-point body velocity and rate state."""

        return dict(self._source_local_state)
        ####

    @property
    def source_effectors(self) -> Mapping[str, float]:
        """Return the actual runtime controls at the source operating point."""

        return {name: float(self._control_values[name]) for name in self.control_names}
        ####

    def state_derivative(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        """Evaluate local body dynamics through the actual lowered vehicle.

        The current runtime environment remains authoritative.  ``environment``
        is accepted to satisfy the common adapter contract but cannot be used
        to silently replace the source problem's atmosphere or wind model.
        """

        del environment
        native = self._native_state(state)
        with self._physical_controls(effectors):
            runtime = _runtime_state(native)
            derivative = self._derivative(runtime)
        # The state order is position, ECIC velocity, quaternion, body rate,
        # and resources.  Resolve velocity derivatives by their state names
        # rather than relying on a magic positional index.
        acceleration_ecic = Vector3(
            derivative[_state_index("vx")],
            derivative[_state_index("vy")],
            derivative[_state_index("vz")],
        )
        velocity_body = Vector3(float(state["u_m_s"]), float(state["v_m_s"]), float(state["w_m_s"]))
        body_rate = Vector3(float(state["p_rad_s"]), float(state["q_rad_s"]), float(state["r_rad_s"]))
        acceleration_body = native.attitude.conjugate().rotate(acceleration_ecic)
        body_velocity_derivative = acceleration_body - body_rate.cross(velocity_body)
        roll = float(state["roll_error_rad"])
        pitch = float(state["pitch_error_rad"])
        cosine_pitch = math.cos(pitch)
        if abs(cosine_pitch) <= 1.0e-8:
            raise ValueError("local attitude error reached an Euler singularity")
        sine_roll = math.sin(roll)
        cosine_roll = math.cos(roll)
        tangent_pitch = math.tan(pitch)
        return {
            "roll_error_rad": body_rate.x + body_rate.y * sine_roll * tangent_pitch + body_rate.z * cosine_roll * tangent_pitch,
            "pitch_error_rad": body_rate.y * cosine_roll - body_rate.z * sine_roll,
            "yaw_error_rad": (body_rate.y * sine_roll + body_rate.z * cosine_roll) / cosine_pitch,
            "u_m_s": body_velocity_derivative.x,
            "v_m_s": body_velocity_derivative.y,
            "w_m_s": body_velocity_derivative.z,
            "p_rad_s": derivative[_state_index("wx")],
            "q_rad_s": derivative[_state_index("wy")],
            "r_rad_s": derivative[_state_index("wz")],
        }
        ####

    def trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        """Solve the declared local trim with actual effectors.

        This adapter deliberately holds the source attitude and local velocity
        state fixed while it solves the physical effectors.  The default
        ``body_force_equilibrium`` contract controls axial and normal force
        plus all three moments; it does not pretend an X8 flying-wing deck
        with no independently controlled side-force axis has solved a lateral
        force equilibrium.  That unclosed lateral channel remains visible in
        the subsequent local dynamics and allocation evidence.

        ``steady_direction_glide`` is intentionally different.  An unpowered
        glider generally cannot hold both speed and altitude without an
        external thrust source, so this mode requires force to be aligned with
        the body velocity and all body moments to vanish.  It is a local
        direction-hold reference for trajectory tracking, not a static force
        equilibrium and must not be promoted as one.
        """

        target_state = {name: float(target.get(name, self._source_local_state[name])) for name in self.state_names}
        initial_state = {
            name: float(initial_guess.get(name, target_state[name]))
            for name in self.state_names
        }
        initial_controls = {
            name: float(initial_guess.get(name, self._control_values[name]))
            for name in self.control_names
        }
        force_scale = max(self.source_state.mass * 9.80665, 1.0)
        moment_scale = max(force_scale * self.reference_length_m, 1.0)
        velocity = Vector3(
            target_state["u_m_s"],
            target_state["v_m_s"],
            target_state["w_m_s"],
        )
        if self.trim_residual_mode == "body_force_equilibrium":
            residual_names = (
                "body_x_force_n",
                "body_z_force_n",
                "roll_moment_nm",
                "pitch_moment_nm",
                "yaw_moment_nm",
            )
            residual_scales = {
                "body_x_force_n": force_scale,
                "body_z_force_n": force_scale,
                "roll_moment_nm": moment_scale,
                "pitch_moment_nm": moment_scale,
                "yaw_moment_nm": moment_scale,
            }
        else:
            force_velocity_scale = max(force_scale * max(velocity.norm(), 1.0), 1.0)
            residual_names = (
                "force_velocity_cross_y_n_m_s",
                "force_velocity_cross_z_n_m_s",
                "roll_moment_nm",
                "pitch_moment_nm",
                "yaw_moment_nm",
            )
            residual_scales = {
                "force_velocity_cross_y_n_m_s": force_velocity_scale,
                "force_velocity_cross_z_n_m_s": force_velocity_scale,
                "roll_moment_nm": moment_scale,
                "pitch_moment_nm": moment_scale,
                "yaw_moment_nm": moment_scale,
            }
        specification = TrimSpec(
            state_names=self.state_names,
            control_names=self.control_names,
            residual_names=residual_names,
            state_initial=initial_state,
            control_initial=initial_controls,
            # The source alpha/velocity condition is an explicit operating
            # point, not an additional hidden trim variable in this local
            # adapter.  Keep it numerically fixed while solving effectors.
            state_lower={name: target_state[name] - 1.0e-10 for name in self.state_names},
            state_upper={name: target_state[name] + 1.0e-10 for name in self.state_names},
            control_lower={name: self.effector_limits[name].lower for name in self.control_names},
            control_upper={name: self.effector_limits[name].upper for name in self.control_names},
            residual_scales=residual_scales,
        )

        def residuals(state_values: Mapping[str, float], controls: Mapping[str, float]) -> Mapping[str, float]:
            force = self._physical_force(state_values, controls)
            moment = self._physical_moment(state_values, controls)
            if self.trim_residual_mode == "body_force_equilibrium":
                return {
                    "body_x_force_n": force.x,
                    "body_z_force_n": force.z,
                    "roll_moment_nm": moment.x,
                    "pitch_moment_nm": moment.y,
                    "yaw_moment_nm": moment.z,
                }
            force_velocity_cross = force.cross(
                Vector3(
                    float(state_values["u_m_s"]),
                    float(state_values["v_m_s"]),
                    float(state_values["w_m_s"]),
                )
            )
            return {
                "force_velocity_cross_y_n_m_s": force_velocity_cross.y,
                "force_velocity_cross_z_n_m_s": force_velocity_cross.z,
                "roll_moment_nm": moment.x,
                "pitch_moment_nm": moment.y,
                "yaw_moment_nm": moment.z,
            }
            ####

        return solve_trim(
            specification,
            residuals,
            residual_tolerance=1.0e-9,
            acceptance_tolerance=1.0e-3,
        )
        ####

    def linearize(self, trim: TrimResult, options: Mapping[str, float | str]) -> ProvenancedLinearization:
        """Generate a two-step derivative artifact from the nonlinear closure."""

        return finite_difference_linearization_with_provenance(
            trim.spec,
            lambda state, controls: self.state_derivative(state, controls, {}),
            trim,
            nonlinear_plant_id=self.id,
            nonlinear_plant_revision=self.revision,
            state_step=float(options.get("state_step", 1.0e-4)),
            control_step=float(options.get("control_step", 1.0e-3)),
            comparison_factor=float(options.get("comparison_factor", 0.5)),
            maximum_relative_difference=float(options.get("maximum_relative_difference", 0.10)),
            comparison_absolute_floor=float(options.get("comparison_absolute_floor", 1.0e-8)),
            state_units={
                "roll_error_rad": "rad",
                "pitch_error_rad": "rad",
                "yaw_error_rad": "rad",
                "u_m_s": "m/s",
                "v_m_s": "m/s",
                "w_m_s": "m/s",
                "p_rad_s": "rad/s",
                "q_rad_s": "rad/s",
                "r_rad_s": "rad/s",
            },
            control_units={name: self.effector_limits[name].unit for name in self.control_names},
            metadata={"adapter": self.id, "revision": self.revision},
        )
        ####

    def effectiveness(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> EffectorEffectiveness:
        """Differentiate actual runtime moments with respect to each effector."""

        baseline = self._physical_moment(state, effectors)
        columns: list[tuple[float, float, float]] = []
        for name in self.control_names:
            step = float(self.effectiveness_steps[name])
            plus = dict(effectors)
            minus = dict(effectors)
            plus[name] = self.effector_limits[name].clamp(float(effectors[name]) + step)
            minus[name] = self.effector_limits[name].clamp(float(effectors[name]) - step)
            denominator = plus[name] - minus[name]
            if abs(denominator) <= 1.0e-12:
                columns.append((0.0, 0.0, 0.0))
                continue
            positive = self._physical_moment(state, plus)
            negative = self._physical_moment(state, minus)
            columns.append(
                (
                    (positive.x - negative.x) / denominator,
                    (positive.y - negative.y) / denominator,
                    (positive.z - negative.z) / denominator,
                )
            )
        return EffectorEffectiveness(
            wrench_names=("moment_x_nm", "moment_y_nm", "moment_z_nm"),
            effector_names=self.control_names,
            matrix=tuple(
                tuple(column[axis] for column in columns)
                for axis in range(3)
            ),
            reference_wrench={
                "moment_x_nm": baseline.x,
                "moment_y_nm": baseline.y,
                "moment_z_nm": baseline.z,
            },
            reference_effectors={name: float(effectors[name]) for name in self.control_names},
            source=f"centered-runtime-moment-difference:{self.id}:{self.revision}",
        )
        ####

    def allocate(
        self,
        state: Mapping[str, float],
        desired_wrench: Mapping[str, float],
        previous_effectors: Mapping[str, float],
        dt_s: float,
    ) -> PhysicalAllocationStep:
        """Allocate through actual table effectiveness and declared actuators."""

        predicted = allocate_and_advance_wrench(
            self.effectiveness(state, previous_effectors),
            self.effector_limits,
            desired_wrench,
            previous_effectors,
            dt_s,
            preferred_effectors=self.source_effectors,
            wrench_weights=self.allocation_wrench_weights or None,
            # Keep the trim-preference term numerically present without
            # turning a feasible two-axis table allocation into a residual
            # merely because the X8 table coordinates use degrees.
            regularization=self.allocation_regularization,
            feasibility_tolerance=self.allocation_feasibility_tolerance,
        )
        # The generic allocator closes the local effectiveness problem.  The
        # adapter must then ask the nonlinear plant what those *actual*
        # effector positions produced; otherwise a table interpolation or
        # actuator-lag discrepancy could be hidden behind the local matrix.
        actual = self._physical_moment(state, predicted.actuator.actual_positions)
        achieved = {
            "moment_x_nm": actual.x,
            "moment_y_nm": actual.y,
            "moment_z_nm": actual.z,
        }
        return PhysicalAllocationStep(
            allocation=predicted.allocation,
            actuator=predicted.actuator,
            achieved_wrench=achieved,
            achieved_residual_wrench={
                name: float(desired_wrench[name]) - achieved[name]
                for name in achieved
            },
        )
        ####

    def _native_state(self, state: Mapping[str, float]) -> RigidBody6DofState:
        """Lift the local body-velocity/rate state into the runtime state."""

        missing = set(self.state_names) - set(state)
        if missing:
            raise KeyError(f"local state is missing: {', '.join(sorted(missing))}")
        values = {name: float(state[name]) for name in self.state_names}
        if any(not math.isfinite(value) for value in values.values()):
            raise ValueError("local state values must be finite")
        velocity_body = Vector3(values["u_m_s"], values["v_m_s"], values["w_m_s"])
        attitude_offset = _quaternion_from_euler(
            Vector3(values["roll_error_rad"], values["pitch_error_rad"], values["yaw_error_rad"])
        )
        attitude = self.source_state.attitude.multiply(attitude_offset).normalized()
        return RigidBody6DofState(
            self.source_state.time,
            self.source_state.position,
            FrameVector3(attitude.rotate(velocity_body), Frame.ECIC),
            attitude,
            Vector3(values["p_rad_s"], values["q_rad_s"], values["r_rad_s"]),
            self.source_state.mass,
            self.source_state.propellant_mass,
            self.source_state.heat_load,
            self.source_state.peak_heat_rate,
        )
        ####

    def _physical_moment(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> Vector3:
        """Recover the actual nonlinear total moment from Newton--Euler RHS."""

        native = self._native_state(state)
        with self._physical_controls(effectors):
            derivative = self._derivative(_runtime_state(native))
        angular_acceleration = Vector3(
            derivative[_state_index("wx")],
            derivative[_state_index("wy")],
            derivative[_state_index("wz")],
        )
        inertia = self.inertia_kg_m2
        angular_momentum = Vector3(
            inertia.x * native.body_rate.x,
            inertia.y * native.body_rate.y,
            inertia.z * native.body_rate.z,
        )
        gyroscopic = native.body_rate.cross(angular_momentum)
        return Vector3(
            inertia.x * angular_acceleration.x + gyroscopic.x,
            inertia.y * angular_acceleration.y + gyroscopic.y,
            inertia.z * angular_acceleration.z + gyroscopic.z,
        )
        ####

    def _physical_force(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> Vector3:
        """Recover the actual nonlinear total body force from the runtime RHS."""

        native = self._native_state(state)
        with self._physical_controls(effectors):
            derivative = self._derivative(_runtime_state(native))
        acceleration_ecic = Vector3(
            derivative[_state_index("vx")],
            derivative[_state_index("vy")],
            derivative[_state_index("vz")],
        )
        return native.attitude.conjugate().rotate(acceleration_ecic).scaled(native.mass)
        ####

    @contextmanager
    def _physical_controls(self, effectors: Mapping[str, float]) -> Iterator[None]:
        """Temporarily apply exact physical controls to the lowered plant."""

        missing = set(self.control_names) - set(effectors)
        if missing:
            raise KeyError(f"effector state is missing: {', '.join(sorted(missing))}")
        snapshot = dict(self._control_values)
        try:
            for name in self.control_names:
                limits = self.effector_limits[name]
                value = limits.clamp(float(effectors[name]))
                self._control_values[name] = value
                radian_name = _radian_control_name(name)
                if radian_name is not None:
                    self._control_values[radian_name] = math.radians(value)
            yield
        finally:
            self._control_values.clear()
            self._control_values.update(snapshot)
        ####

    def _state_bounds(self, name: str) -> tuple[float, float]:
        """Return a narrow source-neighbourhood bound for trim states."""

        if name in self.state_bounds:
            return self.state_bounds[name]
        value = self._source_local_state[name]
        if name in {"roll_error_rad", "pitch_error_rad", "yaw_error_rad"}:
            return (-0.3, 0.3)
        if name == "u_m_s":
            return (max(0.1, value - 2.0), value + 2.0)
        if name in {"v_m_s", "w_m_s"}:
            return (value - 2.0, value + 2.0)
        return (-0.5, 0.5)
        ####


def local_rigid_body_plant_from_vehicle(
    identifier: str,
    revision: str,
    vehicle: RuntimeVehicle,
    *,
    inertia_kg_m2: Vector3,
    reference_length_m: float,
    effector_limits: Mapping[str, EffectorLimits],
    effectiveness_steps: Mapping[str, float],
    state_bounds: Mapping[str, tuple[float, float]] | None = None,
    allocation_wrench_weights: Mapping[str, float] | None = None,
    allocation_regularization: float = 1.0e-10,
    allocation_feasibility_tolerance: float = 1.0e-6,
    trim_residual_mode: Literal["body_force_equilibrium", "steady_direction_glide"] = "body_force_equilibrium",
) -> RuntimeRigidBodyLocalPlant:
    """Build a local physical-control adapter from an accepted runtime state."""

    native = RigidBody6DofState.from_values(
        vehicle.state.time,
        tuple(float(vehicle.state.named[name]) for name in RIGID_BODY_STATE_NAMES),
    )
    return RuntimeRigidBodyLocalPlant(
        identifier,
        revision,
        vehicle,
        native,
        inertia_kg_m2,
        reference_length_m,
        dict(effector_limits),
        dict(effectiveness_steps),
        dict(state_bounds or {}),
        dict(allocation_wrench_weights or {}),
        allocation_regularization,
        allocation_feasibility_tolerance,
        trim_residual_mode,
    )
    ####


def _runtime_state(state: RigidBody6DofState) -> RuntimeState:
    """Pack a native rigid state for the runtime derivative closure."""

    values = state.to_values()
    names = dict(zip(RIGID_BODY_STATE_NAMES, values, strict=True))
    names["time"] = state.time
    return RuntimeState(state.time, values, Frame.ECIC, names, RIGID_BODY_STATE_NAMES)
    ####


def _state_index(name: str) -> int:
    """Resolve the stable rigid-body state-derivative index by name."""

    try:
        return RIGID_BODY_STATE_NAMES.index(name)
    except ValueError as error:
        raise ValueError(f"unknown rigid-body state name {name!r}") from error
    ####


def _radian_control_name(name: str) -> str | None:
    """Map grammar-facing degree controls to their table-query aliases."""

    aliases = {
        "collective-elevon-deg": "collective_elevon",
        "differential-elevon-deg": "differential_elevon",
        "elevator-deg": "elevator",
        "aileron-deg": "aileron",
        "rudder-deg": "rudder",
        "symmetric-stabilator-deg": "symmetric_stabilator",
        "differential-stabilator-deg": "differential_stabilator",
    }
    return aliases.get(name)
    ####


def _quaternion_from_euler(angles: Vector3) -> Quaternion:
    """Return a body-local 3-2-1 attitude offset quaternion."""

    half_roll = angles.x * 0.5
    half_pitch = angles.y * 0.5
    half_yaw = angles.z * 0.5
    cosine_roll, sine_roll = math.cos(half_roll), math.sin(half_roll)
    cosine_pitch, sine_pitch = math.cos(half_pitch), math.sin(half_pitch)
    cosine_yaw, sine_yaw = math.cos(half_yaw), math.sin(half_yaw)
    return Quaternion(
        cosine_roll * cosine_pitch * cosine_yaw + sine_roll * sine_pitch * sine_yaw,
        sine_roll * cosine_pitch * cosine_yaw - cosine_roll * sine_pitch * sine_yaw,
        cosine_roll * sine_pitch * cosine_yaw + sine_roll * cosine_pitch * sine_yaw,
        cosine_roll * cosine_pitch * sine_yaw - sine_roll * sine_pitch * cosine_yaw,
    ).normalized()
    ####


__all__ = ["RuntimeRigidBodyLocalPlant", "local_rigid_body_plant_from_vehicle"]
