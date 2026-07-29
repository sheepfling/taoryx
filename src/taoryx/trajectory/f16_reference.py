"""Runtime source-plant adapter for the F-16 S-119 reference family."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ..contracts import Frame, FrameVector3, Vector3
from ..control_allocation import (
    EffectorEffectiveness,
    EffectorLimits,
    PhysicalAllocationStep,
    ProvenancedLinearization,
    allocate_and_advance_wrench,
    finite_difference_linearization_with_provenance,
)
from ..modes import Quaternion
from ..rigid_body import InertiaMatrix, RigidBody6DofModel, RigidBody6DofState, RigidBodyForceMoment
from ..trim import TrimResult, TrimSpec
from .daveml_atmosphere import DAVEMLAtmosphereBinding, load_daveml_atmosphere
from .daveml_import import (
    DAVEMLTrimBinding,
    load_daveml_family_graph,
    load_daveml_trim_binding,
)
from .daveml_inertia import DAVEMLInertiaBinding


@dataclass(frozen=True, slots=True)
class F16ReferencePlant:
    """Evaluate the F-16 source loads and six-state Newton-Euler dynamics.

    The public control boundary uses SI/radian-friendly Taoryx names and a
    normalized throttle fraction. Source DAVE-ML inputs remain isolated inside
    this adapter: true airspeed is converted to ft/s, and throttle is converted
    from fraction to ``powerLeverAngle`` percent.
    """

    aerodynamics: DAVEMLTrimBinding
    propulsion: DAVEMLTrimBinding
    atmosphere: DAVEMLAtmosphereBinding
    mass_kg: float
    inertia_matrix_kg_m2: InertiaMatrix
    reference_area_m2: float = 27.870912
    mean_aerodynamic_chord_m: float = 3.450336
    span_m: float = 9.144
    cg_percent_mac: float = 0.35
    gravity_m_s2: float = 9.80665
    meters_to_feet: float = 3.280839895013123
    radians_to_degrees: float = 180.0 / math.pi

    def __post_init__(self) -> None:
        if not math.isfinite(self.mass_kg) or self.mass_kg <= 0.0:
            raise ValueError("F-16 reference mass must be positive and finite")
        matrix = np.asarray(self.inertia_matrix_kg_m2, dtype=float)
        if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
            raise ValueError("F-16 reference inertia must be a finite 3x3 matrix")
        if not np.allclose(matrix, matrix.T) or np.any(np.linalg.eigvalsh(matrix) <= 0.0):
            raise ValueError("F-16 reference inertia must be symmetric positive definite")
        ####

    def _kinematics(self, state: Mapping[str, float], altitude_m: float) -> dict[str, float]:
        u = float(state["u_m_s"])
        v = float(state["v_m_s"])
        w = float(state["w_m_s"])
        speed = math.sqrt(u * u + v * v + w * w)
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError("F-16 reference airspeed must be finite and positive")
        atmosphere = self.atmosphere.evaluate(altitude_m)
        alpha = math.atan2(w, u) * self.radians_to_degrees
        beta = math.asin(max(-1.0, min(1.0, v / speed))) * self.radians_to_degrees
        return {
            "speed_m_s": speed,
            "speed_ft_s": speed * self.meters_to_feet,
            "alpha_deg": alpha,
            "beta_deg": beta,
            "mach": speed / atmosphere["speed_of_sound_m_s"],
            "density_kg_m3": atmosphere["density_kg_m3"],
            "dynamic_pressure_pa": 0.5 * atmosphere["density_kg_m3"] * speed * speed,
            "altitude_ft": altitude_m * self.meters_to_feet,
        }
        ####

    def evaluate_loads(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        *,
        altitude_m: float = 0.0,
    ) -> dict[str, float]:
        """Return source-backed aerodynamic and propulsion loads in SI."""

        required_state = {"u_m_s", "v_m_s", "w_m_s", "p_rad_s", "q_rad_s", "r_rad_s"}
        missing = required_state - set(state)
        if missing:
            raise KeyError("F-16 reference state is missing: " + ", ".join(sorted(missing)))
        required_controls = {"elevator_deg", "aileron_deg", "rudder_deg", "throttle_fraction"}
        missing_controls = required_controls - set(controls)
        if missing_controls:
            raise KeyError("F-16 reference controls are missing: " + ", ".join(sorted(missing_controls)))
        throttle = float(controls["throttle_fraction"])
        if not math.isfinite(throttle) or not 0.0 <= throttle <= 1.0:
            raise ValueError("F-16 canonical throttle must be a finite fraction in [0, 1]")
        kinematics = self._kinematics(state, altitude_m)
        aero_state = {
            "true_airspeed_ft_s": kinematics["speed_ft_s"],
            "alpha_deg": kinematics["alpha_deg"],
            "beta_deg": kinematics["beta_deg"],
            "p_rad_s": float(state["p_rad_s"]),
            "q_rad_s": float(state["q_rad_s"]),
            "r_rad_s": float(state["r_rad_s"]),
        }
        aero_controls = {
            "elevator_deg": float(controls["elevator_deg"]),
            "aileron_deg": float(controls["aileron_deg"]),
            "rudder_deg": float(controls["rudder_deg"]),
        }
        coefficients = self.aerodynamics.evaluate(aero_state, aero_controls)
        scale = kinematics["dynamic_pressure_pa"] * self.reference_area_m2
        propulsion = self.propulsion.evaluate(
            {"altitude_ft": kinematics["altitude_ft"], "mach": kinematics["mach"]},
            {"power_pct": throttle * 100.0},
        )["thrust_lbf"]
        return {
            "aerodynamic_force_x_n": scale * coefficients["cx"],
            "aerodynamic_force_y_n": scale * coefficients["cy"],
            "aerodynamic_force_z_n": scale * coefficients["cz"],
            "propulsion_force_x_n": propulsion * 4.4482216152605,
            "total_force_x_n": scale * coefficients["cx"] + propulsion * 4.4482216152605,
            "total_force_y_n": scale * coefficients["cy"],
            "total_force_z_n": scale * coefficients["cz"],
            "total_moment_x_nm": scale * self.span_m * coefficients["cl"],
            "total_moment_y_nm": scale * self.mean_aerodynamic_chord_m * coefficients["cm"],
            "total_moment_z_nm": scale * self.span_m * coefficients["cn"],
            **kinematics,
        }
        ####

    def state_derivative(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        *,
        altitude_m: float = 0.0,
        roll_rad: float = 0.0,
        pitch_rad: float = 0.0,
    ) -> dict[str, float]:
        """Return the six body-velocity/body-rate derivatives in SI."""

        loads = self.evaluate_loads(state, controls, altitude_m=altitude_m)
        p = float(state["p_rad_s"])
        q = float(state["q_rad_s"])
        r = float(state["r_rad_s"])
        gravity = (
            -self.gravity_m_s2 * math.sin(pitch_rad),
            self.gravity_m_s2 * math.sin(roll_rad) * math.cos(pitch_rad),
            self.gravity_m_s2 * math.cos(roll_rad) * math.cos(pitch_rad),
        )
        u = float(state["u_m_s"])
        v = float(state["v_m_s"])
        w = float(state["w_m_s"])
        linear = {
            "u_m_s": loads["total_force_x_n"] / self.mass_kg + gravity[0] - q * w + r * v,
            "v_m_s": loads["total_force_y_n"] / self.mass_kg + gravity[1] - r * u + p * w,
            "w_m_s": loads["total_force_z_n"] / self.mass_kg + gravity[2] - p * v + q * u,
        }
        inertia = np.asarray(self.inertia_matrix_kg_m2, dtype=float)
        omega = np.array((p, q, r), dtype=float)
        moment = np.array(
            (loads["total_moment_x_nm"], loads["total_moment_y_nm"], loads["total_moment_z_nm"]),
            dtype=float,
        )
        angular = np.linalg.solve(inertia, moment - np.cross(omega, inertia @ omega))
        linear.update({"p_rad_s": float(angular[0]), "q_rad_s": float(angular[1]), "r_rad_s": float(angular[2])})
        return linear
        ####

    def linearize_local(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        *,
        trim_pitch_rad: float,
        altitude_m: float = 0.0,
        state_step: float = 1.0e-5,
        control_step: float = 1.0e-5,
    ) -> ProvenancedLinearization:
        """Derive a local A/B pair from the source-backed runtime derivatives.

        The controller state is the six body-velocity/body-rate channels.  The
        attitude used to resolve gravity is held at the declared trimmed pitch,
        matching the local fixed-operating-point controller contract.  This is
        intentionally a local linearization, not a full-envelope scheduled
        controller or a substitute for the quaternion runtime state.
        """

        state_names = ("u_m_s", "v_m_s", "w_m_s", "p_rad_s", "q_rad_s", "r_rad_s")
        control_names = ("elevator_deg", "aileron_deg", "rudder_deg", "throttle_fraction")
        trim_spec = TrimSpec(
            state_names=state_names,
            control_names=control_names,
            residual_names=state_names,
            state_initial={name: float(state[name]) for name in state_names},
            control_initial={name: float(controls[name]) for name in control_names},
            operating_point={"trim_pitch_rad": float(trim_pitch_rad)},
        )
        trim = TrimResult(
            spec=trim_spec,
            state={name: float(state[name]) for name in state_names},
            controls={name: float(controls[name]) for name in control_names},
            residuals={name: 0.0 for name in state_names},
            scaled_residual_norm=0.0,
            success=True,
            status=1,
            message="source-backed operating point",
            iterations=0,
            cost=0.0,
        )

        def evaluate(candidate_state: Mapping[str, float], candidate_controls: Mapping[str, float]) -> Mapping[str, float]:
            return self.state_derivative(
                candidate_state,
                candidate_controls,
                altitude_m=altitude_m,
                pitch_rad=trim_pitch_rad,
            )

        return finite_difference_linearization_with_provenance(
            trim_spec,
            evaluate,
            trim,
            nonlinear_plant_id="reference-f16-s119-source-runtime-plant",
            nonlinear_plant_revision="f16-s119-local-runtime-v1",
            state_step=state_step,
            control_step=control_step,
            state_units={
                "u_m_s": "m/s",
                "v_m_s": "m/s",
                "w_m_s": "m/s",
                "p_rad_s": "rad/s",
                "q_rad_s": "rad/s",
                "r_rad_s": "rad/s",
            },
            control_units={
                "elevator_deg": "deg",
                "aileron_deg": "deg",
                "rudder_deg": "deg",
                "throttle_fraction": "fraction",
            },
            metadata={
                "source_quality": "source_grounded",
                "linearization_scope": "fixed_altitude_local_body_dynamics",
                "trim_pitch_rad": float(trim_pitch_rad),
                "altitude_m": float(altitude_m),
            },
        )
        ####


@dataclass(frozen=True, slots=True)
class F16ReferencePhysicalPlant:
    """Expose the F-16 source plant through the generic physical-control seam.

    The adapter requests a four-axis local wrench consisting of body-axis
    propulsion/force authority and the three source aerodynamic moments.  Its
    effectiveness matrix is regenerated from centered source-load perturbations
    at the current state, while the nonlinear plant remains responsible for
    the actual body-state derivative.  This is a local allocation contract,
    not a claim that the source package contains servo, SAS, or actuator data.
    """

    source: F16ReferencePlant
    trim_result: TrimResult
    trim_pitch_rad: float
    altitude_m: float
    effectors: Mapping[str, EffectorLimits]
    wrench_names: tuple[str, ...] = (
        "total_force_x_n",
        "total_moment_x_nm",
        "total_moment_y_nm",
        "total_moment_z_nm",
    )
    effectiveness_step_surface_deg: float = 1.0e-4
    effectiveness_step_throttle: float = 1.0e-6

    @property
    def state_names(self) -> tuple[str, ...]:
        """Return the local body-velocity/body-rate state ordering."""

        return tuple(self.trim_result.spec.state_names)
        ####
    ####

    @property
    def control_names(self) -> tuple[str, ...]:
        """Return the physical effector ordering."""

        return tuple(self.trim_result.spec.control_names)
        ####
    ####

    def __post_init__(self) -> None:
        if not math.isfinite(self.trim_pitch_rad) or not math.isfinite(self.altitude_m):
            raise ValueError("F-16 physical adapter operating point must be finite")
        if set(self.effectors) != set(self.control_names):
            raise ValueError("F-16 physical adapter limits must match the trim control ordering")
        if len(self.wrench_names) != 4 or len(set(self.wrench_names)) != len(self.wrench_names):
            raise ValueError("F-16 physical adapter requires four unique local wrench axes")
        if self.effectiveness_step_surface_deg <= 0.0 or self.effectiveness_step_throttle <= 0.0:
            raise ValueError("F-16 effectiveness perturbations must be positive")
        ####
    ####

    def state_derivative(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        """Evaluate the source nonlinear derivative at actual effectors."""

        altitude = float(environment.get("altitude_m", self.altitude_m))
        pitch = float(environment.get("trim_pitch_rad", self.trim_pitch_rad))
        return self.source.state_derivative(state, effectors, altitude_m=altitude, pitch_rad=pitch)
        ####
    ####

    def trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        """Return the already-qualified physical operating-point trim."""

        del target, initial_guess
        return self.trim_result
        ####
    ####

    def linearize(self, trim: TrimResult, options: Mapping[str, float | str]) -> ProvenancedLinearization:
        """Derive local state/effectors derivatives from the source runtime."""

        if trim is not self.trim_result and tuple(trim.spec.state_names) != self.state_names:
            raise ValueError("F-16 linearization trim does not match the physical adapter")
        state_step = float(options.get("state_step", 1.0e-5))
        control_step = float(options.get("control_step", 1.0e-5))
        return self.source.linearize_local(
            trim.state,
            trim.controls,
            trim_pitch_rad=self.trim_pitch_rad,
            altitude_m=self.altitude_m,
            state_step=state_step,
            control_step=control_step,
        )
        ####
    ####

    def effectiveness(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> EffectorEffectiveness:
        """Finite-difference source loads into the declared local wrench."""

        reference = self.source.evaluate_loads(state, effectors, altitude_m=self.altitude_m)
        matrix = np.empty((len(self.wrench_names), len(self.control_names)), dtype=float)
        for column, name in enumerate(self.control_names):
            step = self.effectiveness_step_throttle if name == "throttle_fraction" else self.effectiveness_step_surface_deg
            plus = dict(effectors)
            minus = dict(effectors)
            limits = self.effectors[name]
            plus[name] = limits.clamp(float(effectors[name]) + step)
            minus[name] = limits.clamp(float(effectors[name]) - step)
            plus_loads = self.source.evaluate_loads(state, plus, altitude_m=self.altitude_m)
            minus_loads = self.source.evaluate_loads(state, minus, altitude_m=self.altitude_m)
            denominator = plus[name] - minus[name]
            for row, wrench_name in enumerate(self.wrench_names):
                matrix[row, column] = (
                    0.0
                    if denominator == 0.0
                    else (plus_loads[wrench_name] - minus_loads[wrench_name]) / denominator
                )
        return EffectorEffectiveness(
            wrench_names=self.wrench_names,
            effector_names=self.control_names,
            matrix=tuple(tuple(float(value) for value in row) for row in matrix),
            reference_wrench={name: float(reference[name]) for name in self.wrench_names},
            reference_effectors={name: float(effectors[name]) for name in self.control_names},
            source="f16-s119-source-load-centered-finite-difference-v1",
        )
        ####
    ####

    def allocate(
        self,
        state: Mapping[str, float],
        desired_wrench: Mapping[str, float],
        previous_effectors: Mapping[str, float],
        dt_s: float,
    ) -> PhysicalAllocationStep:
        """Allocate a desired local wrench through bounded F-16 effectors."""

        effectiveness = self.effectiveness(state, previous_effectors)
        return allocate_and_advance_wrench(
            effectiveness,
            self.effectors,
            desired_wrench,
            previous_effectors,
            dt_s,
            preferred_effectors=self.trim_result.controls,
            regularization=1.0e-8,
        )
        ####
    ####


@dataclass(frozen=True, slots=True)
class F16ReferenceRigidBodyPlant:
    """Bind the F-16 source adapter to Taoryx's full rigid-body state.

    The wrapper resolves the ECIC velocity into body axes before evaluating the
    source aerodynamics, rotates body loads back into the shared rigid-body
    contract, and supplies the source full inertia matrix to the Newton-Euler
    integrator.  The initial operating point is a flat, fixed-altitude local
    flight fixture: geocentric position, Earth rotation, fuel flow, and
    actuator dynamics are intentionally not implied by this wrapper.
    """

    source: F16ReferencePlant
    controls: Mapping[str, float]
    altitude_m: float = 0.0
    gravity_ecic: Vector3 = Vector3(0.0, 0.0, 9.80665)

    def __post_init__(self) -> None:
        if not math.isfinite(self.altitude_m):
            raise ValueError("F-16 rigid-body fixture altitude must be finite")
        required = {"elevator_deg", "aileron_deg", "rudder_deg", "throttle_fraction"}
        if required - set(self.controls):
            raise KeyError("F-16 rigid-body fixture controls are incomplete")
        ####

    def body_state(self, state: RigidBody6DofState) -> dict[str, float]:
        """Resolve the shared inertial velocity into source body axes."""

        velocity_body = state.attitude.conjugate().rotate(state.velocity.vector)
        return {
            "u_m_s": velocity_body.x,
            "v_m_s": velocity_body.y,
            "w_m_s": velocity_body.z,
            "p_rad_s": state.body_rate.x,
            "q_rad_s": state.body_rate.y,
            "r_rad_s": state.body_rate.z,
        }
        ####

    def force_moment(self, state: RigidBody6DofState) -> RigidBodyForceMoment:
        """Evaluate source loads using the runtime state and fixed controls."""

        loads = self.source.evaluate_loads(self.body_state(state), self.controls, altitude_m=self.altitude_m)
        aero_force = Vector3(
            loads["aerodynamic_force_x_n"],
            loads["aerodynamic_force_y_n"],
            loads["aerodynamic_force_z_n"],
        )
        propulsion_force = Vector3(loads["propulsion_force_x_n"], 0.0, 0.0)
        aero_moment = Vector3(
            loads["total_moment_x_nm"],
            loads["total_moment_y_nm"],
            loads["total_moment_z_nm"],
        )
        return RigidBodyForceMoment(
            aero_force + propulsion_force,
            aero_moment,
            aero_force_body=aero_force,
            propulsion_force_body=propulsion_force,
            aero_moment_body=aero_moment,
            propulsion_moment_body=Vector3(0.0, 0.0, 0.0),
        )
        ####

    def model(self) -> RigidBody6DofModel:
        """Return a source-load/full-inertia rigid-body runtime model."""

        diagonal = Vector3(
            self.source.inertia_matrix_kg_m2[0][0],
            self.source.inertia_matrix_kg_m2[1][1],
            self.source.inertia_matrix_kg_m2[2][2],
        )
        return RigidBody6DofModel(
            inertia=diagonal,
            force_moment=self.force_moment,
            gravity=lambda state: self.gravity_ecic,
            inertia_matrix_provider=lambda state: self.source.inertia_matrix_kg_m2,
        )
        ####

    def initial_state(self, *, true_airspeed_m_s: float, alpha_rad: float, time: float = 0.0) -> RigidBody6DofState:
        """Construct the local flat-flight state for a declared trim point."""

        if not math.isfinite(true_airspeed_m_s) or true_airspeed_m_s <= 0.0:
            raise ValueError("F-16 trim airspeed must be positive and finite")
        if not math.isfinite(alpha_rad):
            raise ValueError("F-16 trim angle of attack must be finite")
        half = 0.5 * alpha_rad
        attitude = Quaternion(math.cos(half), 0.0, math.sin(half), 0.0)
        return RigidBody6DofState(
            time=time,
            position=FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECIC),
            velocity=FrameVector3(Vector3(true_airspeed_m_s, 0.0, 0.0), Frame.ECIC),
            attitude=attitude,
            body_rate=Vector3(0.0, 0.0, 0.0),
            mass=self.source.mass_kg,
            propellant_mass=0.0,
        )
        ####


def load_f16_reference_plant(
    sidecar: str | Path,
    atmosphere_path: str | Path,
) -> F16ReferencePlant:
    """Load the hash-verified F-16 source components into one runtime plant."""

    sidecar_path = Path(sidecar)
    aerodynamics = load_daveml_trim_binding(
        sidecar_path,
        role="aerodynamics",
        state_inputs={
            "true_airspeed_ft_s": "vt",
            "alpha_deg": "alpha",
            "beta_deg": "beta",
            "p_rad_s": "p",
            "q_rad_s": "q",
            "r_rad_s": "r",
        },
        control_inputs={"elevator_deg": "el", "aileron_deg": "ail", "rudder_deg": "rdr"},
        residual_outputs={"cx": "cx", "cy": "cy", "cz": "cz", "cl": "cl", "cm": "cm", "cn": "cn"},
        fixed_inputs={"xcg": 0.35},
    )
    propulsion = load_daveml_trim_binding(
        sidecar_path,
        role="propulsion",
        state_inputs={"altitude_ft": "altitudeMSL", "mach": "mach"},
        control_inputs={"power_pct": "powerLeverAngle"},
        residual_outputs={"thrust_lbf": "thrustBodyForce_X"},
    )
    inertia_graph = load_daveml_family_graph(sidecar_path, role="mass_properties")
    inertia = DAVEMLInertiaBinding(inertia_graph)
    raw_inertia = inertia.as_inertia_matrix()
    inertia_matrix: InertiaMatrix = (
        (float(raw_inertia[0][0]), float(raw_inertia[0][1]), float(raw_inertia[0][2])),
        (float(raw_inertia[1][0]), float(raw_inertia[1][1]), float(raw_inertia[1][2])),
        (float(raw_inertia[2][0]), float(raw_inertia[2][1]), float(raw_inertia[2][2])),
    )
    return F16ReferencePlant(
        aerodynamics=aerodynamics,
        propulsion=propulsion,
        atmosphere=load_daveml_atmosphere(atmosphere_path),
        mass_kg=inertia.evaluate()["mass_kg"],
        inertia_matrix_kg_m2=inertia_matrix,
    )
    ####


__all__ = [
    "F16ReferencePhysicalPlant",
    "F16ReferencePlant",
    "F16ReferenceRigidBodyPlant",
    "load_f16_reference_plant",
]
