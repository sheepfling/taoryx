"""Source-backed X-15 direct-wrench bridge adapter.

The X-15 source deck already supplies a useful local rigid-body load model,
but its available physical effectors are not yet wired into the common
surface allocator.  This adapter therefore exposes the retained source loads
through the explicit ``rigid_body_6dof_direct_wrench`` bridge tier.  It is a
real Newton--Euler derivative witness, not a claim of stabilator, rudder,
reaction-control, or propulsion allocation.
"""

from __future__ import annotations

import math
import re
import tempfile
from collections.abc import Mapping
from dataclasses import dataclass, replace
from functools import lru_cache
from pathlib import Path
from threading import RLock
from typing import NoReturn

from taoryx_reference_models.resources import model_resource_root

from .contracts import Vector3
from .control_allocation import (
    EffectorEffectiveness,
    EffectorLimits,
    PhysicalAllocationStep,
    ProvenancedLinearization,
    allocate_and_advance_wrench,
    finite_difference_linearization_with_provenance,
)
from .control_automation import ControlAutomationDeclaration
from .direct_wrench import (
    DIRECT_WRENCH_NAMES,
    DirectWrenchLimits,
    DirectWrenchProjection,
    add_direct_wrench_to_local_derivative,
)
from .family_adapter import (
    StandardFamilyAdapter,
    descriptor_from_control_plant,
    descriptor_from_direct_wrench_state,
)
from .fidelity_contracts import FidelityTier
from .generic_tuning import LinearAuthorityRequirement, linear_authority_preflight
from .language.grammar_contracts import GrammarProfile
from .local_direct_wrench import LocalDirectWrenchScreenConfig
from .modes import Quaternion
from .physical_lqr import (
    PhysicalWrenchLqiDesign,
    PhysicalWrenchLqrDesign,
    design_physical_wrench_lqi,
    design_physical_wrench_lqr,
    project_linearization_to_wrench,
)
from .runtime.common import RuntimeState, RuntimeVehicle
from .runtime.program import LoadedProgram
from .trim import TrimResult, TrimSpec, solve_trim
from .tuning_campaign import TuningCampaign, run_tuning_campaign

ROOT = model_resource_root()
PROBLEM = ROOT / "examples/showcases/x15_rocket_to_hawaii/source_trim_hold_6dof.prb"
TABLE_ROOT = ROOT / "tests/fixtures/x15_coherent_6dof_public_research_v1/tables"
TABLES = tuple(
    TABLE_ROOT / name
    for name in (
        "x15_static_6axis.tbl",
        "x15_symmetric_stabilator_6axis.tbl",
        "x15_differential_stabilator_6axis.tbl",
        "x15_rudder_6axis.tbl",
    )
)

X15_LOCAL_STATE_NAMES: tuple[str, ...] = (
    "u_m_s",
    "v_m_s",
    "w_m_s",
    "p_rad_s",
    "q_rad_s",
    "r_rad_s",
)
X15_SURFACE_LOCAL_STATE_NAMES: tuple[str, ...] = (
    "roll_error_rad",
    "pitch_error_rad",
    "yaw_error_rad",
    "p_rad_s",
    "q_rad_s",
    "r_rad_s",
)
X15_MASS_KG = 14_641.0545
X15_INERTIA_BODY_KG_M2 = Vector3(4_948.7355, 129_114.2666, 131_825.9024)
X15_SOURCE_SURFACE_NAMES: tuple[str, ...] = (
    "symmetric_stabilator",
    "differential_stabilator",
    "rudder",
)
X15_SOURCE_SURFACE_BOUNDS_DEG: dict[str, tuple[float, float]] = {
    "symmetric_stabilator": (-14.9, 34.9),
    "differential_stabilator": (-20.05, 20.05),
    "rudder": (-29.79, 29.79),
}

# The source runtime keeps its table evaluator in a closure over the loaded
# control dictionary.  Source-surface probes temporarily bind explicit
# override coordinates there, then restore the complete dictionary.  The
# lock makes the cached evaluator a deterministic shared read-through source,
# rather than letting two caller-owned facades interleave those bindings.
_X15_SOURCE_EVALUATOR_LOCK = RLock()


def _candidate(source: str, controls: Mapping[str, float]) -> str:
    """Freeze source guidance while exposing zero physical controls."""

    updated = source
    for name, value in controls.items():
        updated, count = re.subn(
            rf"(\*runtime control {re.escape(name.replace('_', '-'))}\b[^\n]*?\bdefault=)[0-9.eE+-]+",
            rf"\g<1>{value:.16g}",
            updated,
            count=1,
        )
        if count != 1:
            raise ValueError(f"could not locate X-15 control {name!r}")
    updated = updated.replace("rudder-hold-gain-deg-per-deg=1.0", "rudder-hold-gain-deg-per-deg=0.0")
    updated = re.sub(r"\*when time>[^\n]+ stop", "*when time>0.005 stop", updated, count=1)
    return updated
    ####


def _steady_glide_seed(base: RuntimeState) -> RuntimeState:
    """Remove release angular rates for a local source-load witness."""

    values = list(base.values)
    for name in ("wx", "wy", "wz"):
        values[base.value_names.index(name)] = 0.0
    return base.with_values(values)
    ####


def x15_direct_wrench_limits() -> DirectWrenchLimits:
    """Return the deliberately local authority box for the bridge tier."""

    bounds = {
        "force_x_n": (-4.0e5, 4.0e5),
        "force_y_n": (-4.0e5, 4.0e5),
        "force_z_n": (-4.0e5, 4.0e5),
        "moment_x_nm": (-1.0e6, 1.0e6),
        "moment_y_nm": (-1.0e6, 1.0e6),
        "moment_z_nm": (-1.0e6, 1.0e6),
    }
    return DirectWrenchLimits(
        lower={name: values[0] for name, values in bounds.items()},
        upper={name: values[1] for name, values in bounds.items()},
        rate_limit_per_s={name: None for name in DIRECT_WRENCH_NAMES},
    )
    ####


def _zero_source_controls(values: Mapping[str, float]) -> dict[str, float]:
    """Return coherent degree/radian aliases for source control inputs."""

    result = {str(name): float(value) for name, value in values.items()}
    for base_name in ("symmetric-stabilator", "differential-stabilator", "rudder"):
        for alias in (base_name, base_name.replace("-", "_")):
            result[alias] = 0.0
            result[f"{alias}-deg"] = 0.0
            result[f"{alias}_deg"] = 0.0
    return result
    ####


def _runtime_state(base: RuntimeState, attitude: Quaternion, state: Mapping[str, float]) -> RuntimeState:
    """Map local body velocity/rates into the source runtime namespace."""

    body_velocity = Vector3(state["u_m_s"], state["v_m_s"], state["w_m_s"])
    inertial_velocity = attitude.rotate(body_velocity)
    values = list(base.values)
    for name, value in zip(
        ("vx", "vy", "vz"),
        (inertial_velocity.x, inertial_velocity.y, inertial_velocity.z),
        strict=True,
    ):
        values[base.value_names.index(name)] = value
    for name, value in zip(
        ("wx", "wy", "wz"),
        (state["p_rad_s"], state["q_rad_s"], state["r_rad_s"]),
        strict=True,
    ):
        values[base.value_names.index(name)] = value
    named = {
        **base.named,
        "vx": inertial_velocity.x,
        "vy": inertial_velocity.y,
        "vz": inertial_velocity.z,
        "wx": state["p_rad_s"],
        "wy": state["q_rad_s"],
        "wz": state["r_rad_s"],
        "p": state["p_rad_s"],
        "q": state["q_rad_s"],
        "r": state["r_rad_s"],
        "vel": body_velocity.norm(),
        "wt": float(base.named.get("mass", X15_MASS_KG)),
    }
    return RuntimeState(base.time, tuple(values), base.frame, named, base.value_names, base.segment_endpoints)
    ####


def _source_candidate_controls(vehicle: RuntimeVehicle) -> dict[str, float]:
    """Build the source control namespace without mutating the loaded vehicle."""

    return _zero_source_controls(vehicle.control_values)
    ####


@dataclass(slots=True)
class X15SourceDirectWrenchPlant:
    """Local X-15 source-load derivative plus a bounded direct wrench."""

    vehicle: RuntimeVehicle
    base_state: RuntimeState
    attitude: Quaternion
    source_controls: Mapping[str, float]
    limits: DirectWrenchLimits
    reference_state: Mapping[str, float]

    @property
    def state_names(self) -> tuple[str, ...]:
        return X15_LOCAL_STATE_NAMES

    @property
    def control_names(self) -> tuple[str, ...]:
        return DIRECT_WRENCH_NAMES

    def _validate_inputs(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> None:
        if set(state) != set(X15_LOCAL_STATE_NAMES):
            raise ValueError("X-15 local state channels do not match the declared schema")
        if set(effectors) != set(DIRECT_WRENCH_NAMES):
            raise ValueError("X-15 direct-wrench channels do not match the declared schema")
        if any(not math.isfinite(float(value)) for value in state.values()):
            raise ValueError("X-15 local state values must be finite")
        if any(not math.isfinite(float(value)) for value in effectors.values()):
            raise ValueError("X-15 direct-wrench values must be finite")
        ####

    def _source_loads(self, state: Mapping[str, float]) -> tuple[dict[str, float], dict[str, float]]:
        candidate = _runtime_state(self.base_state, self.attitude, state)
        observed = self.vehicle.environment_evaluator(
            {**candidate.named, **self.source_controls}
        ) if self.vehicle.environment_evaluator is not None else None
        if observed is None:
            raise RuntimeError("X-15 source candidate has no rigid-body environment evaluator")
        force = Vector3(
            float(observed["total_force_body_x_n"]),
            float(observed["total_force_body_y_n"]),
            float(observed["total_force_body_z_n"]),
        )
        moment = Vector3(
            float(observed["total_moment_body_x_nm"]),
            float(observed["total_moment_body_y_nm"]),
            float(observed["total_moment_body_z_nm"]),
        )
        rates = Vector3(state["p_rad_s"], state["q_rad_s"], state["r_rad_s"])
        velocity = Vector3(state["u_m_s"], state["v_m_s"], state["w_m_s"])
        transport = rates.cross(velocity)
        derivative = {
            "u_m_s": force.x / X15_MASS_KG - transport.x,
            "v_m_s": force.y / X15_MASS_KG - transport.y,
            "w_m_s": force.z / X15_MASS_KG - transport.z,
            "p_rad_s": moment.x / X15_INERTIA_BODY_KG_M2.x,
            "q_rad_s": moment.y / X15_INERTIA_BODY_KG_M2.y,
            "r_rad_s": moment.z / X15_INERTIA_BODY_KG_M2.z,
        }
        load = {
            "force_x_n": force.x,
            "force_y_n": force.y,
            "force_z_n": force.z,
            "moment_x_nm": moment.x,
            "moment_y_nm": moment.y,
            "moment_z_nm": moment.z,
            "alpha_deg": float(observed["aero_alpha_deg"]),
            "beta_deg": float(observed["aero_sideslip_deg"]),
            "mach": float(observed["aero_mach"]),
        }
        return derivative, load
        ####

    def source_surface_loads(
        self,
        state: Mapping[str, float],
        surface_positions_deg: Mapping[str, float],
    ) -> tuple[dict[str, float], dict[str, float]]:
        """Evaluate the source tables at explicit named physical surfaces.

        This is deliberately a fixed-state source-load operation.  It does
        not claim that these three aerodynamic surfaces trim the complete
        rocket-plane state, or that they represent its propulsion or reaction
        controls.  The returned loads nevertheless come from the same
        runtime table path used by the retained six-axis source deck.
        """

        if set(surface_positions_deg) != set(X15_SOURCE_SURFACE_NAMES):
            raise ValueError("X-15 source-surface positions must name every declared source surface exactly once")
        for name, value in surface_positions_deg.items():
            lower, upper = X15_SOURCE_SURFACE_BOUNDS_DEG[name]
            if not math.isfinite(float(value)) or not lower <= float(value) <= upper:
                raise ValueError(f"X-15 source-surface position for {name!r} lies outside its declared bounds")
        with _X15_SOURCE_EVALUATOR_LOCK:
            controls = self.vehicle.control_values
            previous = dict(controls)
            try:
                for name, value in surface_positions_deg.items():
                    degree_name = name.replace("_", "-") + "-deg"
                    controls[f"_surface-allocation-override-{degree_name}"] = float(value)
                return self._source_loads(state)
            finally:
                controls.clear()
                controls.update(previous)
        ####

    def project(self, effectors: Mapping[str, float], previous: Mapping[str, float] | None = None, dt_s: float = 1.0) -> DirectWrenchProjection:
        """Project a local direct wrench through the declared bridge authority."""

        prior = previous or {name: 0.0 for name in DIRECT_WRENCH_NAMES}
        return self.limits.project(effectors, prior, dt_s)
        ####

    def balancing_wrench(self, state: Mapping[str, float] | None = None) -> dict[str, float]:
        """Return the local direct wrench that cancels the source loads.

        This is a bridge-tier diagnostic seed.  It is deliberately derived
        from the source load evaluator at one local state and is not a
        physical X-15 trim command or a substitute for source effector trim.
        """

        candidate = dict(state or self.reference_state)
        _, loads = self._source_loads(candidate)
        return {name: -float(loads[name]) for name in DIRECT_WRENCH_NAMES}
        ####

    def _trim_spec(
        self,
        target: Mapping[str, float],
        initial_guess: Mapping[str, float],
    ) -> TrimSpec:
        """Build the fixed-state, six-axis local direct-wrench trim problem."""

        state = {name: float(target.get(name, self.reference_state[name])) for name in X15_LOCAL_STATE_NAMES}
        balance = self.balancing_wrench(state)
        controls = {
            name: float(initial_guess.get(name, balance[name]))
            for name in DIRECT_WRENCH_NAMES
        }
        state_epsilon = 1.0e-9
        return TrimSpec(
            state_names=X15_LOCAL_STATE_NAMES,
            control_names=DIRECT_WRENCH_NAMES,
            residual_names=X15_LOCAL_STATE_NAMES,
            state_initial=state,
            control_initial=controls,
            state_lower={name: value - state_epsilon for name, value in state.items()},
            state_upper={name: value + state_epsilon for name, value in state.items()},
            control_lower=self.limits.lower,
            control_upper=self.limits.upper,
            residual_scales={
                "u_m_s": 1.0,
                "v_m_s": 1.0,
                "w_m_s": 1.0,
                "p_rad_s": 1.0,
                "q_rad_s": 1.0,
                "r_rad_s": 1.0,
            },
            x_scale={
                **{name: max(1.0, abs(value)) for name, value in state.items()},
                **{
                    name: max(1.0, abs(self.limits.upper[name]), abs(self.limits.lower[name]))
                    for name in DIRECT_WRENCH_NAMES
                },
            },
            operating_point={
                "control_realization": "direct_wrench",
                "equilibrium": "source_load_cancelled_local",
                "source_trim_claim": "not_source_physical_effector_trim",
            },
        )
        ####

    def trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        """Solve a local equilibrium using only the explicit direct-wrench bridge."""

        spec = self._trim_spec(target, initial_guess)

        def evaluate(state: Mapping[str, float], controls: Mapping[str, float]) -> Mapping[str, float]:
            return self.state_derivative(state, controls, {})

        return solve_trim(
            spec,
            evaluate,
            residual_tolerance=1.0e-10,
            # The source evaluator is table-backed; this remains a very
            # small local acceleration/rate residual while allowing the
            # bounded solver's final floating-point noise.
            acceptance_tolerance=1.0e-6,
        )
        ####

    def linearize(self, trim: TrimResult, options: Mapping[str, float | str]) -> ProvenancedLinearization:
        """Differentiate the same local direct-wrench nonlinear plant used by trim."""

        if tuple(trim.spec.state_names) != X15_LOCAL_STATE_NAMES or tuple(trim.spec.control_names) != DIRECT_WRENCH_NAMES:
            raise ValueError("X-15 direct-wrench linearization trim does not match the local bridge schema")
        return finite_difference_linearization_with_provenance(
            trim.spec,
            lambda state, controls: self.state_derivative(state, controls, {}),
            trim,
            nonlinear_plant_id="x15-source-direct-wrench-local-plant",
            nonlinear_plant_revision="x15-source-direct-wrench-local-v2",
            state_step=float(options.get("state_step", 1.0e-5)),
            control_step=float(options.get("control_step", 1.0e-5)),
            state_units={
                "u_m_s": "m/s",
                "v_m_s": "m/s",
                "w_m_s": "m/s",
                "p_rad_s": "rad/s",
                "q_rad_s": "rad/s",
                "r_rad_s": "rad/s",
            },
            control_units={
                "force_x_n": "N",
                "force_y_n": "N",
                "force_z_n": "N",
                "moment_x_nm": "N*m",
                "moment_y_nm": "N*m",
                "moment_z_nm": "N*m",
            },
            metadata={
                "control_realization": "direct_wrench",
                "source_load_model": "x15-daveml-local-body-loads",
                "trim_claim": "local source-load cancellation only",
            },
        )
        ####

    def effectiveness(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
    ) -> NoReturn:
        """Reject physical-effector queries at the direct-wrench tier."""

        del state, effectors
        raise RuntimeError("X-15 direct-wrench bridge has no physical-effector effectiveness matrix")
        ####

    def allocate(
        self,
        state: Mapping[str, float],
        desired_wrench: Mapping[str, float],
        previous_effectors: Mapping[str, float],
        dt_s: float,
    ) -> NoReturn:
        """Reject physical allocation queries at the direct-wrench tier."""

        del state, desired_wrench, previous_effectors, dt_s
        raise RuntimeError("X-15 direct-wrench bridge has no physical-effectors allocator")
        ####

    def state_derivative(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        """Evaluate source loads and add only the achieved direct wrench."""

        del environment
        self._validate_inputs(state, effectors)
        source_derivative, _ = self._source_loads(state)
        projection = self.project(effectors)
        return add_direct_wrench_to_local_derivative(
            source_derivative,
            projection,
            mass_kg=X15_MASS_KG,
            inertia_kg_m2=X15_INERTIA_BODY_KG_M2,
        )
        ####


@dataclass(slots=True)
class X15SourceSurfaceLocalPlant:
    """Local X-15 attitude/rate plant with actual source-table surfaces.

    The retained X-15 source deck resolves body force and moment for the
    symmetric stabilator, differential stabilator, and rudder at a fixed
    release/glide fixture.  This adapter adds only the small-angle attitude
    kinematics required to make a local attitude/rate feedback screen
    well-defined.  It deliberately keeps the source attitude/position and
    translational reference fixture fixed throughout the screen, so a
    zero-moment solution is *not* promoted to a complete X-15 flight
    equilibrium.
    """

    source_plant: X15SourceDirectWrenchPlant
    effectiveness_step_deg: float = 0.25

    def __post_init__(self) -> None:
        if not math.isfinite(self.effectiveness_step_deg) or self.effectiveness_step_deg <= 0.0:
            raise ValueError("X-15 source-surface effectiveness step must be finite and positive")
        ####

    @property
    def state_names(self) -> tuple[str, ...]:
        """Return the closed local attitude-error and body-rate state set."""

        return X15_SURFACE_LOCAL_STATE_NAMES
        ####

    @property
    def control_names(self) -> tuple[str, ...]:
        """Return the exact named source aerodynamic controls in degrees."""

        return X15_SOURCE_SURFACE_NAMES
        ####

    @property
    def reference_state(self) -> dict[str, float]:
        """Return the fixed release fixture with zero local attitude error."""

        return {
            "roll_error_rad": 0.0,
            "pitch_error_rad": 0.0,
            "yaw_error_rad": 0.0,
            **{
                name: float(self.source_plant.reference_state[name])
                for name in ("p_rad_s", "q_rad_s", "r_rad_s")
            },
        }
        ####

    @property
    def effector_limits(self) -> dict[str, EffectorLimits]:
        """Return source-declared position bounds without invented servo data."""

        return {
            name: EffectorLimits(
                name=name,
                lower=X15_SOURCE_SURFACE_BOUNDS_DEG[name][0],
                upper=X15_SOURCE_SURFACE_BOUNDS_DEG[name][1],
                unit="deg",
            )
            for name in self.control_names
        }
        ####

    def _validate_inputs(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> None:
        if set(state) != set(self.state_names):
            raise ValueError("X-15 source-surface local state channels do not match the declared schema")
        if set(effectors) != set(self.control_names):
            raise ValueError("X-15 source-surface controls do not match the declared schema")
        if any(not math.isfinite(float(value)) for value in (*state.values(), *effectors.values())):
            raise ValueError("X-15 source-surface local state and controls must be finite")
        for name, value in effectors.items():
            limits = self.effector_limits[name]
            if not limits.lower <= float(value) <= limits.upper:
                raise ValueError(f"X-15 source-surface control {name!r} lies outside its declared bounds")
        ####

    def state_derivative(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        """Evaluate source loads plus local 3-2-1 attitude-error kinematics."""

        del environment
        self._validate_inputs(state, effectors)
        local_state = {
            **{
                name: float(self.source_plant.reference_state[name])
                for name in ("u_m_s", "v_m_s", "w_m_s")
            },
            **{name: float(state[name]) for name in ("p_rad_s", "q_rad_s", "r_rad_s")},
        }
        source_derivative, _ = self.source_plant.source_surface_loads(local_state, effectors)
        roll = float(state["roll_error_rad"])
        pitch = float(state["pitch_error_rad"])
        cosine_pitch = math.cos(pitch)
        if abs(cosine_pitch) <= 1.0e-8:
            raise ValueError("X-15 source-surface local attitude reached an Euler singularity")
        sine_roll = math.sin(roll)
        cosine_roll = math.cos(roll)
        p = float(state["p_rad_s"])
        q = float(state["q_rad_s"])
        r = float(state["r_rad_s"])
        return {
            "roll_error_rad": p + q * sine_roll * math.tan(pitch) + r * cosine_roll * math.tan(pitch),
            "pitch_error_rad": q * cosine_roll - r * sine_roll,
            "yaw_error_rad": (q * sine_roll + r * cosine_roll) / cosine_pitch,
            **{name: float(source_derivative[name]) for name in ("p_rad_s", "q_rad_s", "r_rad_s")},
        }
        ####

    def _trim_spec(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimSpec:
        """Build the fixed-fixture, three-moment source-surface trim contract."""

        state = {name: float(target.get(name, self.reference_state[name])) for name in self.state_names}
        controls = {
            name: float(initial_guess.get(name, 0.0))
            for name in self.control_names
        }
        state_epsilon = 1.0e-9
        return TrimSpec(
            state_names=self.state_names,
            control_names=self.control_names,
            residual_names=("p_rad_s", "q_rad_s", "r_rad_s"),
            state_initial=state,
            control_initial=controls,
            state_lower={name: value - state_epsilon for name, value in state.items()},
            state_upper={name: value + state_epsilon for name, value in state.items()},
            control_lower={name: self.effector_limits[name].lower for name in self.control_names},
            control_upper={name: self.effector_limits[name].upper for name in self.control_names},
            residual_scales={"p_rad_s": 1.0, "q_rad_s": 1.0, "r_rad_s": 1.0},
            x_scale={
                **{name: max(1.0, abs(value)) for name, value in state.items()},
                **{
                    name: max(1.0, abs(self.effector_limits[name].lower), abs(self.effector_limits[name].upper))
                    for name in self.control_names
                },
            },
            operating_point={
                "control_realization": "source_surface_allocated",
                "equilibrium": "source_moment_balanced_fixed_release_fixture",
                "source_trim_claim": "local_attitude_rate_only_not_full_vehicle_trim",
            },
        )
        ####

    def trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        """Solve a bounded three-moment equilibrium at the retained fixture."""

        specification = self._trim_spec(target, initial_guess)
        return solve_trim(
            specification,
            lambda state, controls: self.state_derivative(state, controls, {}),
            residual_tolerance=1.0e-10,
            acceptance_tolerance=1.0e-6,
        )
        ####

    def linearize(self, trim: TrimResult, options: Mapping[str, float | str]) -> ProvenancedLinearization:
        """Differentiate the actual source-surface nonlinear local plant."""

        if tuple(trim.spec.state_names) != self.state_names or tuple(trim.spec.control_names) != self.control_names:
            raise ValueError("X-15 source-surface linearization trim does not match the local plant schema")
        return finite_difference_linearization_with_provenance(
            trim.spec,
            lambda state, controls: self.state_derivative(state, controls, {}),
            trim,
            nonlinear_plant_id="x15-source-surface-local-attitude-rate-plant",
            nonlinear_plant_revision="x15-source-surface-local-v1",
            state_step=float(options.get("state_step", 1.0e-5)),
            control_step=float(options.get("control_step", 1.0e-4)),
            state_units={
                "roll_error_rad": "rad",
                "pitch_error_rad": "rad",
                "yaw_error_rad": "rad",
                "p_rad_s": "rad/s",
                "q_rad_s": "rad/s",
                "r_rad_s": "rad/s",
            },
            control_units={name: "deg" for name in self.control_names},
            comparison_absolute_floor=float(options.get("comparison_absolute_floor", 1.0e-8)),
            metadata={
                "control_realization": "source_surface_allocated",
                "source_load_model": "x15-daveml-local-body-loads",
                "trim_claim": "fixed-fixture moment balance only",
            },
        )
        ####

    def effectiveness(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
    ) -> EffectorEffectiveness:
        """Differentiate actual source moments in each physical surface coordinate."""

        self._validate_inputs(state, effectors)
        local_state = {
            **{
                name: float(self.source_plant.reference_state[name])
                for name in ("u_m_s", "v_m_s", "w_m_s")
            },
            **{name: float(state[name]) for name in ("p_rad_s", "q_rad_s", "r_rad_s")},
        }
        _, baseline = self.source_plant.source_surface_loads(local_state, effectors)
        axes = ("moment_x_nm", "moment_y_nm", "moment_z_nm")
        columns: list[tuple[float, float, float]] = []
        for name in self.control_names:
            limits = self.effector_limits[name]
            plus = dict(effectors)
            minus = dict(effectors)
            plus[name] = limits.clamp(float(effectors[name]) + self.effectiveness_step_deg)
            minus[name] = limits.clamp(float(effectors[name]) - self.effectiveness_step_deg)
            denominator = plus[name] - minus[name]
            if abs(denominator) <= 1.0e-12:
                columns.append((0.0, 0.0, 0.0))
                continue
            _, positive = self.source_plant.source_surface_loads(local_state, plus)
            _, negative = self.source_plant.source_surface_loads(local_state, minus)
            columns.append(tuple((float(positive[axis]) - float(negative[axis])) / denominator for axis in axes))
        return EffectorEffectiveness(
            wrench_names=axes,
            effector_names=self.control_names,
            matrix=tuple(tuple(column[axis] for column in columns) for axis in range(len(axes))),
            reference_wrench={axis: float(baseline[axis]) for axis in axes},
            reference_effectors={name: float(effectors[name]) for name in self.control_names},
            source="centered-x15-source-table-moment-difference-per-degree",
        )
        ####

    def allocate(
        self,
        state: Mapping[str, float],
        desired_wrench: Mapping[str, float],
        previous_effectors: Mapping[str, float],
        dt_s: float,
    ) -> PhysicalAllocationStep:
        """Realize moment demands through bounded named X-15 source surfaces."""

        if not math.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("X-15 source-surface allocation dt must be finite and positive")
        previous = {name: float(previous_effectors[name]) for name in self.control_names}
        predicted = allocate_and_advance_wrench(
            self.effectiveness(state, previous),
            self.effector_limits,
            desired_wrench,
            previous,
            dt_s,
            preferred_effectors=previous,
            regularization=1.0e-10,
            feasibility_tolerance=1.0e-6,
        )
        local_state = {
            **{
                name: float(self.source_plant.reference_state[name])
                for name in ("u_m_s", "v_m_s", "w_m_s")
            },
            **{name: float(state[name]) for name in ("p_rad_s", "q_rad_s", "r_rad_s")},
        }
        _, loads = self.source_plant.source_surface_loads(local_state, predicted.actuator.actual_positions)
        achieved = {name: float(loads[name]) for name in predicted.allocation.controlled_wrench_axes}
        return PhysicalAllocationStep(
            allocation=predicted.allocation,
            actuator=predicted.actuator,
            achieved_wrench=achieved,
            achieved_residual_wrench={name: float(desired_wrench[name]) - achieved[name] for name in achieved},
        )
        ####


def build_x15_source_surface_local_plant() -> X15SourceSurfaceLocalPlant:
    """Build a caller-owned physical-source local feedback plant facade."""

    return X15SourceSurfaceLocalPlant(build_x15_source_direct_wrench_plant())
    ####


def build_x15_source_surface_local_adapter(tier: FidelityTier) -> StandardFamilyAdapter:
    """Build the frozen-translation physical X-15 attitude/rate adapter.

    The adapter is deliberately narrower than the family's public
    surface-allocated fidelity label: it supplies local attitude/rate
    feedback through actual source-surface allocation at the pinned release
    fixture.  It does not assert translational trim, state propagation, or a
    high-energy flight controller.
    """

    if tier != "rigid_body_6dof_surface_allocated":
        raise ValueError(f"X-15 source-surface local adapter only implements surface allocation, not {tier!r}")
    plant = build_x15_source_surface_local_plant()
    descriptor = descriptor_from_control_plant(
        plant,
        family_id="x15",
        adapter_id="taoryx.x15_source_surface_local.v1",
        physical_family="powered_fixed_wing",
        tier=tier,
        state_units={
            "roll_error_rad": "rad",
            "pitch_error_rad": "rad",
            "yaw_error_rad": "rad",
            "p_rad_s": "rad/s",
            "q_rad_s": "rad/s",
            "r_rad_s": "rad/s",
        },
        control_units={name: "deg" for name in X15_SOURCE_SURFACE_NAMES},
        evidence_status="development",
        validity_envelope=(
            "X-15 retained source-release fixture; frozen translational state, small local attitude errors, "
            "source-table domain, and declared source-surface position bounds"
        ),
        omitted_physics=(
            "full source physical-effector trim",
            "translation and attitude propagation beyond the local frozen fixture",
            "propulsion and reaction-control allocation",
            "source actuator rate/lag data",
            "navigation, energy management, and mission guidance",
            "closed-loop flight qualification",
        ),
    )
    return StandardFamilyAdapter.from_control_plant(descriptor, plant)
    ####


@lru_cache(maxsize=1)
def build_x15_source_surface_physical_lqr_design() -> PhysicalWrenchLqrDesign:
    """Build the bounded source-surface X-15 local attitude/rate LQR design."""

    plant = build_x15_source_surface_local_plant()
    trim = plant.trim({}, {})
    if not trim.success:
        raise RuntimeError(f"X-15 source-surface moment-balance trim did not converge: {trim.as_dict()}")
    linearization = plant.linearize(trim, {})
    if not linearization.provenance.derivative_consistent:
        raise RuntimeError(
            "X-15 source-surface attitude/rate derivative did not meet the declared table-noise comparison contract: "
            f"{linearization.provenance}"
        )
    projection = project_linearization_to_wrench(
        linearization,
        plant.effectiveness(trim.state, trim.controls),
        state_names=plant.state_names,
        wrench_names=("moment_x_nm", "moment_y_nm", "moment_z_nm"),
        effector_names=plant.control_names,
    )
    authority = linear_authority_preflight(
        LinearAuthorityRequirement(
            "x15-source-surface-local-attitude-rate-authority",
            plant.state_names,
        ),
        state_names=projection.state_names,
        a_matrix=projection.a_matrix,
        b_matrix=projection.b_matrix,
    )
    if authority.status != "passed":
        raise RuntimeError(
            "X-15 source-surface attitude/rate authority preflight blocked LQR synthesis: "
            f"{authority.as_dict()}"
        )
    return design_physical_wrench_lqr(
        "x15-source-release-surface-attitude-rate-wrench-lqr-v1",
        projection,
        q_diagonal=(10.0, 10.0, 10.0, 2.0, 2.0, 2.0),
        r_diagonal=(100.0, 100.0, 100.0),
        state_scales=(0.1, 0.1, 0.1, 0.1, 0.1, 0.1),
        wrench_scales=(1.0e5, 1.0e6, 1.0e5),
    )
    ####


@lru_cache(maxsize=1)
def build_x15_source_surface_physical_lqi_design() -> PhysicalWrenchLqiDesign:
    """Build the source-surface offset-free attitude LQI design.

    Only the three local attitude-error states are integrated.  The frozen
    translational fixture has no physical force/propulsion closure in this
    retained binding and is intentionally absent from both feedback and the
    integral channels.
    """

    lqr = build_x15_source_surface_physical_lqr_design()
    return design_physical_wrench_lqi(
        "x15-source-release-surface-attitude-rate-wrench-lqi-v1",
        lqr.projection,
        output_names=("roll_error_rad", "pitch_error_rad", "yaw_error_rad"),
        q_diagonal=lqr.q_diagonal,
        r_diagonal=lqr.r_diagonal,
        integral_q_diagonal=(0.1, 0.1, 0.1),
        state_scales=lqr.state_scales,
        wrench_scales=lqr.wrench_scales,
    )
    ####


def build_x15_source_surface_lqi_tuning_campaign() -> TuningCampaign:
    """Declare the common auto-tuning path for physical X-15 surface controls."""

    return ControlAutomationDeclaration(
        id="x15-source-release-surface-attitude-rate",
        campaign_id="x15-source-surface-local-lqi-v1",
        family_id="x15",
        tier="rigid_body_6dof_surface_allocated",
        strategy_id="high_energy_glide.v1",
        node_id="source-release-frozen-translation-attitude-rate",
        state_scales={
            "roll_error_rad": 0.1,
            "pitch_error_rad": 0.1,
            "yaw_error_rad": 0.1,
            "p_rad_s": 0.1,
            "q_rad_s": 0.1,
            "r_rad_s": 0.1,
        },
        control_scales={
            "symmetric_stabilator": 10.0,
            "differential_stabilator": 10.0,
            "rudder": 10.0,
        },
        authority_state_names=X15_SURFACE_LOCAL_STATE_NAMES,
        offset_free_outputs=("roll_error_rad", "pitch_error_rad", "yaw_error_rad"),
        integral_weight_multiplier=0.1,
        profile_grid_id_prefix="x15-source-surface-local-lqi",
        linearization_options={"comparison_absolute_floor": 1.0e-8},
    ).build_campaign()
    ####


def build_x15_source_direct_wrench_plant() -> X15SourceDirectWrenchPlant:
    """Construct one caller-owned facade over the cached immutable source setup.

    The source deck, tables, and frozen source controls are invariant within a
    Python process.  Loading them repeatedly is both expensive and unrelated
    to the requested composition.  Return a fresh small facade so callers do
    not share mutable dictionaries, while the source evaluator remains cached
    beneath it.
    """

    template = _cached_x15_source_direct_wrench_plant()
    return X15SourceDirectWrenchPlant(
        vehicle=template.vehicle,
        base_state=template.base_state,
        attitude=template.attitude,
        source_controls=dict(template.source_controls),
        limits=template.limits,
        reference_state=dict(template.reference_state),
    )
    ####


@lru_cache(maxsize=1)
def _cached_x15_source_direct_wrench_plant() -> X15SourceDirectWrenchPlant:
    """Load the immutable source deck/table setup once per process.

    ``environment_evaluator`` is used only as a pure function of the supplied
    state and frozen source controls.  Runtime execution never advances this
    retained vehicle object, so sharing it cannot leak truth state between
    preflight, lowering, batch, or episode checks.
    """

    source = PROBLEM.read_text(encoding="utf-8")
    with tempfile.TemporaryDirectory(prefix="taoryx-x15-direct-wrench-adapter-") as directory:
        candidate = Path(directory) / "candidate.prb"
        candidate.write_text(
            _candidate(
                source,
                {
                    "symmetric_stabilator_deg": 0.0,
                    "differential_stabilator_deg": 0.0,
                    "rudder_deg": 0.0,
                },
            ),
            encoding="utf-8",
        )
        program = LoadedProgram.load(candidate, TABLES, profile=GrammarProfile.TAORYX)
    vehicle = program.case().vehicles["1"]
    base = _steady_glide_seed(vehicle.state)
    attitude = Quaternion(
        float(base.named["qw"]),
        float(base.named["qx"]),
        float(base.named["qy"]),
        float(base.named["qz"]),
    )
    inertial_velocity = Vector3(
        float(base.named["vx"]),
        float(base.named["vy"]),
        float(base.named["vz"]),
    )
    body_velocity = attitude.conjugate().rotate(inertial_velocity)
    reference = {
        "u_m_s": body_velocity.x,
        "v_m_s": body_velocity.y,
        "w_m_s": body_velocity.z,
        "p_rad_s": 0.0,
        "q_rad_s": 0.0,
        "r_rad_s": 0.0,
    }
    return X15SourceDirectWrenchPlant(
        vehicle=vehicle,
        base_state=base,
        attitude=attitude,
        source_controls=_source_candidate_controls(vehicle),
        limits=x15_direct_wrench_limits(),
        reference_state=reference,
    )
    ####


def build_x15_source_direct_wrench_adapter(tier: FidelityTier) -> StandardFamilyAdapter:
    """Build the executable X-15 direct-wrench bridge adapter."""

    if tier != "rigid_body_6dof_direct_wrench":
        raise ValueError(f"X-15 source bridge only implements the direct-wrench tier, not {tier!r}")
    plant = build_x15_source_direct_wrench_plant()
    descriptor = descriptor_from_direct_wrench_state(
        family_id="x15",
        adapter_id="taoryx.high_energy.fixed_wing.v1",
        physical_family="powered_fixed_wing",
        state_names=X15_LOCAL_STATE_NAMES,
        state_units={
            "u_m_s": "m/s",
            "v_m_s": "m/s",
            "w_m_s": "m/s",
            "p_rad_s": "rad/s",
            "q_rad_s": "rad/s",
            "r_rad_s": "rad/s",
        },
        evidence_status="development",
        validity_envelope="X-15 source local release/glide load witness; source table domain, Mach <= 6.7, alpha/beta <= 10 deg, and local direct-wrench authority limits",
        omitted_physics=(
            "full source physical-effector trim",
            "physical stabilator/rudder/RCS/propulsion allocation",
            "powered-to-coast scheduling",
            "navigation and mission guidance",
            "closed-loop nonlinear qualification",
        ),
    )
    return StandardFamilyAdapter.from_control_plant(descriptor, plant)
    ####


def build_x15_direct_wrench_tuning_campaign() -> TuningCampaign:
    """Return the local source-release direct-wrench LQR screen.

    This is a bounded generalized-wrench candidate over the source-backed
    release/glide witness.  It is not a physical X-15 effector controller and
    must not be read as a stabilator, rudder, RCS, or propulsion allocation.
    """

    return ControlAutomationDeclaration(
        id="x15-source-release-direct-wrench",
        campaign_id="x15-source-release-direct-wrench-v1",
        family_id="x15",
        tier="rigid_body_6dof_direct_wrench",
        strategy_id="high_energy_glide.v1",
        node_id="source-release-local",
        state_scales={
            "u_m_s": 1000.0,
            "v_m_s": 300.0,
            "w_m_s": 300.0,
            "p_rad_s": 0.5,
            "q_rad_s": 0.5,
            "r_rad_s": 0.5,
        },
        control_scales={
            "force_x_n": 100_000.0,
            "force_y_n": 100_000.0,
            "force_z_n": 100_000.0,
            "moment_x_nm": 100_000.0,
            "moment_y_nm": 100_000.0,
            "moment_z_nm": 100_000.0,
        },
        authority_state_names=X15_LOCAL_STATE_NAMES,
    ).build_campaign()
    ####


def build_x15_direct_wrench_lqi_tuning_campaign() -> TuningCampaign:
    """Return an offset-free X-15 local speed direct-wrench design screen.

    The only integrated output is the source-local body ``u`` velocity. This
    is deliberately narrower than an attitude or navigation controller: the
    bridge exposes no attitude states or physical X-15 effector allocation.
    Commands remain bounded generalized wrench coordinates and are evaluated
    only by the pinned release/glide source-load derivative.
    """

    return ControlAutomationDeclaration(
        id="x15-source-release-direct-wrench-speed",
        campaign_id="x15-source-release-direct-wrench-lqi-v1",
        family_id="x15",
        tier="rigid_body_6dof_direct_wrench",
        strategy_id="high_energy_glide.v1",
        node_id="source-release-local",
        state_scales={
            "u_m_s": 1000.0,
            "v_m_s": 300.0,
            "w_m_s": 300.0,
            "p_rad_s": 0.5,
            "q_rad_s": 0.5,
            "r_rad_s": 0.5,
        },
        control_scales={
            "force_x_n": 100_000.0,
            "force_y_n": 100_000.0,
            "force_z_n": 100_000.0,
            "moment_x_nm": 100_000.0,
            "moment_y_nm": 100_000.0,
            "moment_z_nm": 100_000.0,
        },
        authority_state_names=X15_LOCAL_STATE_NAMES,
        offset_free_outputs=("u_m_s",),
        integral_weight_multiplier=0.1,
        profile_grid_id_prefix="x15-source-release-direct-wrench-lqi",
    ).build_campaign()
    ####


@lru_cache(maxsize=1)
def build_x15_local_direct_wrench_screen_config() -> LocalDirectWrenchScreenConfig:
    """Return the source-declared X-15 local bridge-screen contract.

    This is the reusable counterpart of the legacy validation script.  The
    operating condition remains the local unpowered release/glide load point;
    the explicit load-cancellation bias is not a physical source-effector
    trim and the resulting witness is not an X-15 trajectory mission.

    The contract is frozen and source-pinned, so a process-local cache is
    safe.  Several Mission Composition checks intentionally ask for this exact screen
    during preflight, batch execution, episode opening, and parity replay;
    reparsing the same source tables for each of those checks provides no
    additional evidence and makes the catalog-wide witness audit needlessly
    slow.
    """

    plant = build_x15_source_direct_wrench_plant()
    reference = dict(plant.reference_state)
    initial = dict(reference)
    initial.update(
        {
            "u_m_s": reference["u_m_s"] + 1.0,
            "v_m_s": reference["v_m_s"] + 0.3,
            "w_m_s": reference["w_m_s"] + 0.5,
            "p_rad_s": 0.01,
            "q_rad_s": -0.015,
            "r_rad_s": 0.01,
        }
    )
    return LocalDirectWrenchScreenConfig(
        id="x15-source-release-glide-local-direct-wrench-v1",
        plant_id="x15-source-direct-wrench-local-plant",
        state_names=X15_LOCAL_STATE_NAMES,
        reference_state=reference,
        initial_state=initial,
        source_derivative=lambda state, wrench: plant.state_derivative(state, wrench, {}),
        balancing_wrench=plant.balancing_wrench,
        limits=plant.limits,
        state_scales=(20.0, 20.0, 20.0, 0.15, 0.15, 0.15),
        control_scales=(2.0e6, 2.0e6, 2.0e6, 2.0e7, 2.0e7, 2.0e7),
        state_cost_weights=(2.0, 2.0, 2.0, 2.0, 2.0, 2.0),
        control_cost_weights=(2_000.0, 2_000.0, 2_000.0, 2_000.0, 2_000.0, 2_000.0),
        dt_s=0.002,
        duration_s=2.0,
        resource_values={"mass_kg": X15_MASS_KG},
    )
    ####


@lru_cache(maxsize=1)
def build_x15_local_direct_wrench_lqi_screen_config() -> LocalDirectWrenchScreenConfig:
    """Return the exact source-release LQI screen through the common bridge.

    The selected gain is the best safe candidate produced by the advertised
    source-release LQI campaign.  This remains a pinned direct-wrench screen:
    it exercises one integrated body-speed output but does not create a
    stabilator, rudder, RCS, propulsion, or navigation controller.
    """

    report = run_tuning_campaign(
        build_x15_source_direct_wrench_adapter("rigid_body_6dof_direct_wrench"),
        build_x15_direct_wrench_lqi_tuning_campaign(),
    )
    node = report.nodes[0] if report.nodes else None
    candidate = node.lqr.best if node is not None and node.lqr is not None else None
    if candidate is None or candidate.lqi is None:
        raise RuntimeError("X-15 source-release LQI campaign produced no safe LQI candidate")
    baseline = build_x15_local_direct_wrench_screen_config()
    initial = dict(baseline.reference_state)
    initial["u_m_s"] += 0.5
    return replace(
        baseline,
        id="x15-source-release-glide-local-direct-wrench-lqi-v1",
        initial_state=initial,
        dt_s=0.01,
        duration_s=3.5,
        final_error_fraction_limit=0.05,
        controller_method="lqi",
        lqi_result=candidate.lqi,
        lqi_campaign_id="x15-source-release-direct-wrench-lqi-v1",
        integral_lower={name: -1.0 for name in candidate.lqi.output_names},
        integral_upper={name: 1.0 for name in candidate.lqi.output_names},
        assessment_state_names=("u_m_s",),
    )
    ####


__all__ = [
    "X15SourceDirectWrenchPlant",
    "X15SourceSurfaceLocalPlant",
    "X15_INERTIA_BODY_KG_M2",
    "X15_LOCAL_STATE_NAMES",
    "X15_MASS_KG",
    "X15_SURFACE_LOCAL_STATE_NAMES",
    "build_x15_local_direct_wrench_lqi_screen_config",
    "build_x15_local_direct_wrench_screen_config",
    "build_x15_direct_wrench_lqi_tuning_campaign",
    "build_x15_source_direct_wrench_adapter",
    "build_x15_source_direct_wrench_plant",
    "build_x15_direct_wrench_tuning_campaign",
    "build_x15_source_surface_local_adapter",
    "build_x15_source_surface_local_plant",
    "build_x15_source_surface_lqi_tuning_campaign",
    "build_x15_source_surface_physical_lqi_design",
    "build_x15_source_surface_physical_lqr_design",
    "x15_direct_wrench_limits",
]
####
