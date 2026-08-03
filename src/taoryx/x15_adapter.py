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
from dataclasses import dataclass
from pathlib import Path
from typing import NoReturn

from .contracts import Vector3
from .control_allocation import ProvenancedLinearization, finite_difference_linearization_with_provenance
from .direct_wrench import (
    DIRECT_WRENCH_NAMES,
    DirectWrenchLimits,
    DirectWrenchProjection,
    add_direct_wrench_to_local_derivative,
)
from .family_adapter import StandardFamilyAdapter, descriptor_from_direct_wrench_state
from .fidelity_contracts import FidelityTier
from .language.grammar_contracts import GrammarProfile
from .local_direct_wrench import LocalDirectWrenchScreenConfig
from .modes import Quaternion
from .runtime.common import RuntimeState, RuntimeVehicle
from .runtime.program import LoadedProgram
from .trim import TrimResult, TrimSpec, solve_trim

ROOT = Path(__file__).resolve().parents[2]
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
X15_MASS_KG = 14_641.0545
X15_INERTIA_BODY_KG_M2 = Vector3(4_948.7355, 129_114.2666, 131_825.9024)


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


def build_x15_source_direct_wrench_plant() -> X15SourceDirectWrenchPlant:
    """Load the source deck and construct its local direct-wrench bridge."""

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


def build_x15_local_direct_wrench_screen_config() -> LocalDirectWrenchScreenConfig:
    """Return the source-declared X-15 local bridge-screen contract.

    This is the reusable counterpart of the legacy validation script.  The
    operating condition remains the local unpowered release/glide load point;
    the explicit load-cancellation bias is not a physical source-effector
    trim and the resulting witness is not an X-15 trajectory mission.
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
    )
    ####


__all__ = [
    "X15SourceDirectWrenchPlant",
    "X15_INERTIA_BODY_KG_M2",
    "X15_LOCAL_STATE_NAMES",
    "X15_MASS_KG",
    "build_x15_local_direct_wrench_screen_config",
    "build_x15_source_direct_wrench_adapter",
    "build_x15_source_direct_wrench_plant",
    "x15_direct_wrench_limits",
]
####
