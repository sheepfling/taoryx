"""Source-backed HL-20 local rigid-body and surface-allocation adapter.

The HL-20 source package currently provides a useful local aerodynamic graph
and seven declared surface inputs, but not a complete trim and flight-control
plant.  This adapter therefore exposes only the operations that are actually
supported:

``source loads -> local six-state Newton-Euler derivative``
``source-load finite differences -> bounded surface allocation``

It intentionally does not invent navigation, gravity attitude resolution,
trim, or a source-exact guidance law.  The common family façade reports trim
and linearization as not applicable until those source bindings are available.
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
from dataclasses import dataclass, field, replace
from functools import lru_cache

from taoryx_hl20.resources import model_resource_root

from .control_allocation import (
    EffectorEffectiveness,
    EffectorLimits,
    PhysicalAllocationStep,
    ProvenancedLinearization,
    allocate_and_advance_wrench,
    finite_difference_linearization_with_provenance,
)
from .control_automation import ControlAutomationDeclaration
from .direct_wrench import DIRECT_WRENCH_NAMES, DirectWrenchLimits, add_direct_wrench_to_local_derivative
from .family_adapter import (
    AdapterChannel,
    StandardFamilyAdapter,
    TrimFragmentResult,
    descriptor_from_control_plant,
    descriptor_from_direct_wrench_state,
)
from .fidelity_contracts import FidelityTier
from .generic_tuning import LinearAuthorityRequirement, linear_authority_preflight
from .hl20_controls import HL20_SOURCE_SURFACE_BOUNDS_DEG, HL20_SURFACE_NAMES
from .hl20_source_aerodynamics import HL20DavemlAerodynamics, build_hl20_source_effectiveness
from .hl20_source_model import HL20_FIXED_MASS_KG, HL20_REFERENCE_INERTIA_KG_M2
from .local_direct_wrench import LocalDirectWrenchScreenConfig
from .physical_lqr import (
    PhysicalWrenchLqiDesign,
    PhysicalWrenchLqrDesign,
    design_physical_wrench_lqi,
    design_physical_wrench_lqr,
    project_linearization_to_wrench,
)
from .trim import TrimResult, TrimSpec, solve_trim
from .tuning_campaign import TuningCampaign, run_tuning_campaign

HL20_LOCAL_STATE_NAMES = (
    "u_m_s",
    "v_m_s",
    "w_m_s",
    "p_rad_s",
    "q_rad_s",
    "r_rad_s",
    "altitude_m",
)
HL20_TRIM_EVIDENCE = model_resource_root() / "verification/daveml_hl20_trim_evidence.json"
HL20_LOCAL_TUNING_STATE_NAMES = HL20_LOCAL_STATE_NAMES[:-1]
HL20_LOCAL_TUNING_ALTITUDE_M = 10_000.0
HL20_SURFACE_LOCAL_STATE_NAMES = (
    "roll_error_rad",
    "pitch_error_rad",
    "yaw_error_rad",
    "p_rad_s",
    "q_rad_s",
    "r_rad_s",
)
HL20_SOURCE_PITCH_TRIM_ALPHA_DEG = 6.457652069267908
HL20_SOURCE_PITCH_TRIM_ALTITUDE_M = 0.0


@dataclass(frozen=True, slots=True)
class HL20SourceSurfacePlant:
    """Evaluate the pinned source graph at one local operating point.

    The state is a local body-velocity/body-rate fixture with altitude held
    fixed.  It is suitable for derivative, effectivity, and allocation probes;
    it is not a complete atmospheric flight state because attitude, position,
    gravity resolution, and trim are intentionally outside this binding.
    """

    source: HL20DavemlAerodynamics = field(default_factory=HL20DavemlAerodynamics)
    mass_kg: float = HL20_FIXED_MASS_KG
    inertia_body_kg_m2: tuple[float, float, float] = field(
        default_factory=lambda: (
            float(HL20_REFERENCE_INERTIA_KG_M2.x),
            float(HL20_REFERENCE_INERTIA_KG_M2.y),
            float(HL20_REFERENCE_INERTIA_KG_M2.z),
        )
    )
    rate_limit_deg_s: float = 60.0
    time_constant_s: float = 0.15

    def __post_init__(self) -> None:
        if not math.isfinite(self.mass_kg) or self.mass_kg <= 0.0:
            raise ValueError("HL-20 source adapter mass must be positive and finite")
        if len(self.inertia_body_kg_m2) != 3 or any(
            not math.isfinite(value) or value <= 0.0 for value in self.inertia_body_kg_m2
        ):
            raise ValueError("HL-20 source adapter inertia must contain three positive finite values")
        if not math.isfinite(self.rate_limit_deg_s) or self.rate_limit_deg_s <= 0.0:
            raise ValueError("HL-20 source adapter rate limit must be positive and finite")
        if not math.isfinite(self.time_constant_s) or self.time_constant_s <= 0.0:
            raise ValueError("HL-20 source adapter time constant must be positive and finite")
        ####
    ####

    @property
    def state_names(self) -> tuple[str, ...]:
        return HL20_LOCAL_STATE_NAMES
        ####
    ####

    @property
    def control_names(self) -> tuple[str, ...]:
        return HL20_SURFACE_NAMES
        ####
    ####

    @property
    def effector_limits(self) -> dict[str, EffectorLimits]:
        """Return physical surface bounds used by the generic allocator."""

        return {
            name: EffectorLimits(
                name=name,
                lower=bounds[0],
                upper=bounds[1],
                unit="deg",
                rate_limit_per_s=self.rate_limit_deg_s,
                time_constant_s=self.time_constant_s,
            )
            for name, bounds in HL20_SOURCE_SURFACE_BOUNDS_DEG.items()
        }
        ####
    ####

    def state_derivative(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        """Return source-load Newton-Euler derivatives for the local fixture."""

        values = _require_state(state)
        controls = {name: float(effectors.get(name, 0.0)) for name in HL20_SURFACE_NAMES}
        altitude_m = float(environment.get("altitude_m", values["altitude_m"]))
        loads = self.source.evaluate(
            (values["u_m_s"], values["v_m_s"], values["w_m_s"]),
            altitude_m,
            (values["p_rad_s"], values["q_rad_s"], values["r_rad_s"]),
            controls=controls,
        )
        force = loads.force_body_n
        moment = loads.moment_body_nm
        velocity = (values["u_m_s"], values["v_m_s"], values["w_m_s"])
        rates = (values["p_rad_s"], values["q_rad_s"], values["r_rad_s"])
        transport = _cross(rates, velocity)
        acceleration = tuple(force[index] / self.mass_kg - transport[index] for index in range(3))
        inertia_rates: tuple[float, float, float] = (
            self.inertia_body_kg_m2[0] * rates[0],
            self.inertia_body_kg_m2[1] * rates[1],
            self.inertia_body_kg_m2[2] * rates[2],
        )
        angular_transport = _cross(rates, inertia_rates)
        angular_acceleration: tuple[float, float, float] = (
            (moment[0] - angular_transport[0]) / self.inertia_body_kg_m2[0],
            (moment[1] - angular_transport[1]) / self.inertia_body_kg_m2[1],
            (moment[2] - angular_transport[2]) / self.inertia_body_kg_m2[2],
        )
        return {
            "u_m_s": acceleration[0],
            "v_m_s": acceleration[1],
            "w_m_s": acceleration[2],
            "p_rad_s": angular_acceleration[0],
            "q_rad_s": angular_acceleration[1],
            "r_rad_s": angular_acceleration[2],
            "altitude_m": 0.0,
        }
        ####
    ####

    def effectiveness(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
    ) -> EffectorEffectiveness:
        """Return centered source-load effectiveness at the supplied state."""

        values = _require_state(state)
        controls = {name: float(effectors.get(name, 0.0)) for name in HL20_SURFACE_NAMES}
        return build_hl20_source_effectiveness(
            self.source,
            (values["u_m_s"], values["v_m_s"], values["w_m_s"]),
            values["altitude_m"],
            body_rates_rad_s=(values["p_rad_s"], values["q_rad_s"], values["r_rad_s"]),
            controls=controls,
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
        """Allocate a local six-axis source wrench through bounded surfaces."""

        if not math.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("HL-20 source allocation dt must be positive and finite")
        previous = {name: float(previous_effectors.get(name, 0.0)) for name in HL20_SURFACE_NAMES}
        effectiveness = self.effectiveness(state, previous)
        return allocate_and_advance_wrench(
            effectiveness,
            self.effector_limits,
            desired_wrench,
            previous,
            dt_s,
            preferred_effectors=previous,
            regularization=1.0e-8,
        )
        ####
    ####


def hl20_direct_wrench_limits() -> DirectWrenchLimits:
    """Return the bounded local authority box for the HL-20 bridge tier."""

    bounds = {
        "force_x_n": (-2.0e5, 2.0e5),
        "force_y_n": (-2.0e5, 2.0e5),
        "force_z_n": (-2.0e5, 2.0e5),
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


@dataclass(frozen=True, slots=True)
class HL20SourceSurfaceLocalPlant:
    """Bounded local HL-20 attitude/rate plant through actual source surfaces.

    The DAVE-ML source evaluates all seven declared surface coordinates at the
    verified Mach-1 scalar pitch-trim fragment.  This local plant retains that
    velocity/altitude fixture and adds only attitude-error kinematics, so it
    can exercise source-surface moment allocation and local feedback without
    recasting the scalar source fragment as a complete glide equilibrium.
    """

    source_plant: HL20SourceSurfacePlant = field(default_factory=HL20SourceSurfacePlant)

    @property
    def state_names(self) -> tuple[str, ...]:
        """Return the closed local attitude-error/body-rate state contract."""

        return HL20_SURFACE_LOCAL_STATE_NAMES
        ####

    @property
    def control_names(self) -> tuple[str, ...]:
        """Return every actual DAVE-ML source-surface coordinate."""

        return HL20_SURFACE_NAMES
        ####

    @property
    def reference_state(self) -> dict[str, float]:
        """Return zero local error at the pinned source pitch-trim fragment."""

        return {name: 0.0 for name in self.state_names}
        ####

    @property
    def effector_limits(self) -> dict[str, EffectorLimits]:
        """Retain the existing explicit HL-20 position/lag/rate contract."""

        return self.source_plant.effector_limits
        ####

    @property
    def reference_source_state(self) -> dict[str, float]:
        """Return the frozen source velocity, rate, and altitude fixture."""

        alpha_rad = math.radians(HL20_SOURCE_PITCH_TRIM_ALPHA_DEG)
        speed_m_s = self.source_plant.source.speed_of_sound_m_s
        return {
            "u_m_s": speed_m_s * math.cos(alpha_rad),
            "v_m_s": 0.0,
            "w_m_s": -speed_m_s * math.sin(alpha_rad),
            "p_rad_s": 0.0,
            "q_rad_s": 0.0,
            "r_rad_s": 0.0,
            "altitude_m": HL20_SOURCE_PITCH_TRIM_ALTITUDE_M,
        }
        ####

    def _interior_controls(self) -> dict[str, float]:
        """Seed trim away from unilateral flap bounds for centered derivatives."""

        return {
            "upper_left_body_flap": -2.0,
            "lower_left_body_flap": 2.0,
            "upper_right_body_flap": -2.0,
            "lower_right_body_flap": 2.0,
            "left_wing_flap": 0.5,
            "right_wing_flap": 0.5,
            "rudder": 0.5,
        }
        ####

    @property
    def reference_effectors(self) -> dict[str, float]:
        """Expose the centered-difference-safe physical surface trim seed."""

        return self._interior_controls()
        ####

    def _validate_inputs(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> None:
        if set(state) != set(self.state_names):
            raise ValueError("HL-20 local source-surface state channels do not match the declared schema")
        if set(effectors) != set(self.control_names):
            raise ValueError("HL-20 local source-surface controls do not match the declared schema")
        if any(not math.isfinite(float(value)) for value in (*state.values(), *effectors.values())):
            raise ValueError("HL-20 local source-surface state and controls must be finite")
        for name, value in effectors.items():
            limits = self.effector_limits[name]
            if not limits.lower <= float(value) <= limits.upper:
                raise ValueError(f"HL-20 source-surface control {name!r} lies outside its declared bounds")
        ####

    def _source_state(self, state: Mapping[str, float]) -> dict[str, float]:
        return {
            **{
                name: value
                for name, value in self.reference_source_state.items()
                if name in {"u_m_s", "v_m_s", "w_m_s", "altitude_m"}
            },
            **{name: float(state[name]) for name in ("p_rad_s", "q_rad_s", "r_rad_s")},
        }
        ####

    def state_derivative(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        """Evaluate source angular dynamics with local 3-2-1 error kinematics.

        ``external_pitch_moment_bias_nm`` is the explicitly named local-screen
        disturbance seam. It contributes only to physical pitch angular
        acceleration after source-surface loads have been evaluated; it never
        changes an LQI demand or an allocator result.
        """

        self._validate_inputs(state, effectors)
        source_derivative = self.source_plant.state_derivative(
            self._source_state(state),
            effectors,
            {"altitude_m": HL20_SOURCE_PITCH_TRIM_ALTITUDE_M},
        )
        roll = float(state["roll_error_rad"])
        pitch = float(state["pitch_error_rad"])
        cosine_pitch = math.cos(pitch)
        if abs(cosine_pitch) <= 1.0e-8:
            raise ValueError("HL-20 local source-surface attitude reached an Euler singularity")
        sine_roll = math.sin(roll)
        cosine_roll = math.cos(roll)
        p = float(state["p_rad_s"])
        q = float(state["q_rad_s"])
        r = float(state["r_rad_s"])
        derivative = {
            "roll_error_rad": p + q * sine_roll * math.tan(pitch) + r * cosine_roll * math.tan(pitch),
            "pitch_error_rad": q * cosine_roll - r * sine_roll,
            "yaw_error_rad": (q * sine_roll + r * cosine_roll) / cosine_pitch,
            **{name: float(source_derivative[name]) for name in ("p_rad_s", "q_rad_s", "r_rad_s")},
        }
        derivative["q_rad_s"] += _external_pitch_moment_bias_nm(environment) / self.source_plant.inertia_body_kg_m2[1]
        return derivative
        ####

    def _trim_spec(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimSpec:
        state = {name: float(target.get(name, self.reference_state[name])) for name in self.state_names}
        controls = {
            name: float(initial_guess.get(name, self._interior_controls()[name]))
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
                "equilibrium": "source_moment_balanced_mach1_pitch_fragment_fixture",
                "source_trim_claim": "local_attitude_rate_only_not_full_glide_trim",
                "source_pitch_trim_alpha_deg": HL20_SOURCE_PITCH_TRIM_ALPHA_DEG,
            },
        )
        ####

    def trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        """Balance source moments at the retained scalar pitch-trim fixture."""

        specification = self._trim_spec(target, initial_guess)
        return solve_trim(
            specification,
            lambda state, controls: self.state_derivative(state, controls, {}),
            residual_tolerance=1.0e-10,
            acceptance_tolerance=1.0e-6,
        )
        ####

    def linearize(self, trim: TrimResult, options: Mapping[str, float | str]) -> ProvenancedLinearization:
        """Differentiate the actual local DAVE-ML surface path at its moment balance."""

        if tuple(trim.spec.state_names) != self.state_names or tuple(trim.spec.control_names) != self.control_names:
            raise ValueError("HL-20 local source-surface linearization trim does not match the plant schema")
        return finite_difference_linearization_with_provenance(
            trim.spec,
            lambda state, controls: self.state_derivative(state, controls, {}),
            trim,
            nonlinear_plant_id="hl20-source-surface-local-attitude-rate-plant",
            nonlinear_plant_revision="hl20-source-surface-local-v1",
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
                "source_load_model": "hl20-daveml-local-body-loads",
                "trim_claim": "Mach-1 scalar pitch fragment plus local moment balance only",
            },
        )
        ####

    def effectiveness(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
    ) -> EffectorEffectiveness:
        """Project the actual source effectivity onto its three moment axes."""

        self._validate_inputs(state, effectors)
        source = self.source_plant.effectiveness(self._source_state(state), effectors)
        axes = ("moment_x_nm", "moment_y_nm", "moment_z_nm")
        indices = tuple(source.wrench_names.index(name) for name in axes)
        return EffectorEffectiveness(
            wrench_names=axes,
            effector_names=self.control_names,
            matrix=tuple(tuple(float(source.array[index, column]) for column in range(len(self.control_names))) for index in indices),
            reference_wrench={name: float(source.reference_wrench[name]) for name in axes},
            reference_effectors={name: float(effectors[name]) for name in self.control_names},
            source="projected-centered-hl20-daveml-source-moment-difference-per-degree",
        )
        ####

    def allocate(
        self,
        state: Mapping[str, float],
        desired_wrench: Mapping[str, float],
        previous_effectors: Mapping[str, float],
        dt_s: float,
    ) -> PhysicalAllocationStep:
        """Allocate requested body moments through bounded, lagged source surfaces."""

        if not math.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("HL-20 local source-surface allocation dt must be finite and positive")
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
        source_state = self._source_state(state)
        loads = self.source_plant.source.evaluate(
            (source_state["u_m_s"], source_state["v_m_s"], source_state["w_m_s"]),
            source_state["altitude_m"],
            (source_state["p_rad_s"], source_state["q_rad_s"], source_state["r_rad_s"]),
            controls=predicted.actuator.actual_positions,
        )
        axes = self.effectiveness(state, previous).wrench_names
        achieved = {name: float(loads.moment_body_nm[index]) for index, name in enumerate(axes)}
        return PhysicalAllocationStep(
            allocation=predicted.allocation,
            actuator=predicted.actuator,
            achieved_wrench=achieved,
            achieved_residual_wrench={name: float(desired_wrench[name]) - achieved[name] for name in achieved},
        )
        ####


def build_hl20_source_surface_local_plant() -> HL20SourceSurfaceLocalPlant:
    """Build the bounded local physical HL-20 source-surface feedback plant."""

    return HL20SourceSurfaceLocalPlant()
    ####


def _external_pitch_moment_bias_nm(environment: Mapping[str, float | str]) -> float:
    """Resolve the one declared HL-20 local external pitch-moment seam."""

    value = environment.get("external_pitch_moment_bias_nm", 0.0)
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError("external_pitch_moment_bias_nm must be finite numeric")
    return float(value)
    ####


@lru_cache(maxsize=1)
def build_hl20_source_surface_physical_lqr_design() -> PhysicalWrenchLqrDesign:
    """Design an actual-source-surface local HL-20 attitude/rate LQR."""

    plant = build_hl20_source_surface_local_plant()
    trim = plant.trim({}, {})
    if not trim.success:
        raise RuntimeError(f"HL-20 source-surface moment balance did not converge: {trim.as_dict()}")
    linearization = plant.linearize(trim, {})
    if not linearization.provenance.derivative_consistent:
        raise RuntimeError(
            "HL-20 source-surface local derivative did not meet its declared comparison contract: "
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
        LinearAuthorityRequirement("hl20-source-surface-local-attitude-rate-authority", plant.state_names),
        state_names=projection.state_names,
        a_matrix=projection.a_matrix,
        b_matrix=projection.b_matrix,
    )
    if authority.status != "passed":
        raise RuntimeError(
            "HL-20 source-surface local attitude/rate authority blocked LQR synthesis: "
            f"{authority.as_dict()}"
        )
    return design_physical_wrench_lqr(
        "hl20-source-mach1-surface-attitude-rate-wrench-lqr-v1",
        projection,
        q_diagonal=(10.0, 10.0, 10.0, 2.0, 2.0, 2.0),
        r_diagonal=(100.0, 100.0, 100.0),
        state_scales=(0.1, 0.1, 0.1, 0.1, 0.1, 0.1),
        wrench_scales=(1.0e5, 1.0e5, 1.0e5),
    )
    ####


@lru_cache(maxsize=1)
def build_hl20_source_surface_physical_lqi_design() -> PhysicalWrenchLqiDesign:
    """Return the bounded local HL-20 attitude-error LQI design."""

    lqr = build_hl20_source_surface_physical_lqr_design()
    return design_physical_wrench_lqi(
        "hl20-source-mach1-surface-attitude-rate-wrench-lqi-v1",
        lqr.projection,
        output_names=("roll_error_rad", "pitch_error_rad", "yaw_error_rad"),
        q_diagonal=lqr.q_diagonal,
        r_diagonal=lqr.r_diagonal,
        # The source fixture needs a deliberately strong integral penalty to
        # reject the declared persistent 5% pitch-moment cases within the
        # eight-second local screen.  This is a fixed fixture profile, not a
        # gain schedule or a claim about the full glide vehicle.
        integral_q_diagonal=(1000.0, 1000.0, 1000.0),
        state_scales=lqr.state_scales,
        wrench_scales=lqr.wrench_scales,
    )
    ####


def build_hl20_source_surface_lqi_tuning_campaign() -> TuningCampaign:
    """Declare the exact physical-wrench LQI runtime for HL-20 surfaces.

    The campaign uses the same frozen-fixture attitude/rate-to-moment
    projection as the nonlinear screen.  Its seven bounded source surfaces
    remain allocator-owned runtime evidence, rather than becoming a second
    raw-surface tuning coordinate system.
    """

    design = build_hl20_source_surface_physical_lqi_design()
    return ControlAutomationDeclaration(
        id="hl20-source-mach1-surface-attitude-rate-wrench",
        campaign_id="hl20-source-surface-local-lqi-v1",
        family_id="hl20_mod_k",
        tier="rigid_body_6dof_surface_allocated",
        strategy_id="lifting_body_glide.v1",
        node_id="source-mach1-frozen-translation-attitude-rate-wrench",
        state_scales=dict(zip(design.projection.state_names, design.state_scales, strict=True)),
        control_scales=dict(zip(design.projection.wrench_names, design.wrench_scales, strict=True)),
        authority_state_names=design.projection.state_names,
        offset_free_outputs=design.result.output_names,
        state_weight_multipliers=(1.0,),
        control_effort_multipliers=(1.0,),
        state_base_weights=design.q_diagonal,
        control_base_weights=design.r_diagonal,
        integral_base_weights=design.integral_q_diagonal,
        integral_weight_multipliers=(1.0,),
        profile_grid_id_prefix="hl20-source-surface-wrench-lqi",
    ).build_campaign()
    ####


@dataclass(frozen=True, slots=True)
class HL20SourceDirectWrenchPlant:
    """Evaluate source loads plus a bounded local generalized wrench.

    This is deliberately a bridge tier. The source aerodynamic load graph is
    retained, while the control contribution is an explicit generalized body
    wrench. No physical surface or actuator claim is made by this plant.
    """

    source_plant: HL20SourceSurfacePlant = field(default_factory=HL20SourceSurfacePlant)
    limits: DirectWrenchLimits = field(default_factory=hl20_direct_wrench_limits)

    @property
    def state_names(self) -> tuple[str, ...]:
        return self.source_plant.state_names
        ####

    @property
    def control_names(self) -> tuple[str, ...]:
        return DIRECT_WRENCH_NAMES
        ####

    def state_derivative(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        """Add the achieved direct wrench to the source-load derivative."""

        base = self.source_plant.state_derivative(
            state,
            {name: 0.0 for name in HL20_SURFACE_NAMES},
            environment,
        )
        projection = self.limits.project(
            effectors,
            {name: 0.0 for name in DIRECT_WRENCH_NAMES},
            float(environment.get("dt_s", 1.0)),
        )
        return add_direct_wrench_to_local_derivative(
            base,
            projection,
            mass_kg=self.source_plant.mass_kg,
            inertia_kg_m2=HL20_REFERENCE_INERTIA_KG_M2,
        )
        ####
    ####


@dataclass(frozen=True, slots=True)
class HL20SourceDirectWrenchTuningPlant:
    """Fixed-condition source-local plant for the common LQR campaign.

    The public HL-20 direct-wrench adapter retains altitude as a source
    environment channel because it does not propagate attitude or position.
    A controller candidate needs a closed state vector, so this companion
    plant fixes the documented Mach-0.5, alpha-5-degree, 10-km screen
    condition and exposes only the six velocity/rate states actually
    controlled by that screen. It remains a direct-wrench bridge, not a
    source surface allocator or high-energy mission model.
    """

    source_plant: HL20SourceDirectWrenchPlant
    reference_state: Mapping[str, float]
    altitude_m: float = HL20_LOCAL_TUNING_ALTITUDE_M

    @property
    def state_names(self) -> tuple[str, ...]:
        """Return the closed local velocity/rate state vector."""

        return HL20_LOCAL_TUNING_STATE_NAMES
        ####

    @property
    def control_names(self) -> tuple[str, ...]:
        """Return the declared generalized-wrench bridge controls."""

        return DIRECT_WRENCH_NAMES
        ####

    @property
    def limits(self) -> DirectWrenchLimits:
        """Expose the bounded direct-wrench authority box."""

        return self.source_plant.limits
        ####

    def _validate_inputs(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> None:
        if set(state) != set(self.state_names):
            raise ValueError("HL-20 local tuning state channels do not match the declared schema")
        if set(effectors) != set(self.control_names):
            raise ValueError("HL-20 direct-wrench tuning controls do not match the declared schema")
        if any(not math.isfinite(float(value)) for value in (*state.values(), *effectors.values())):
            raise ValueError("HL-20 local tuning state and controls must be finite")
        ####

    def state_derivative(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        """Evaluate the documented source condition plus a bounded wrench."""

        del environment
        self._validate_inputs(state, effectors)
        full_state = {**{name: float(state[name]) for name in self.state_names}, "altitude_m": self.altitude_m}
        derivative = self.source_plant.state_derivative(
            full_state,
            effectors,
            {"altitude_m": self.altitude_m, "dt_s": 1.0},
        )
        return {name: float(derivative[name]) for name in self.state_names}
        ####

    def balancing_wrench(self, state: Mapping[str, float] | None = None) -> dict[str, float]:
        """Cancel source loads at one local state with declared bridge authority."""

        candidate = {name: float((state or self.reference_state)[name]) for name in self.state_names}
        zero = {name: 0.0 for name in self.control_names}
        baseline = self.state_derivative(candidate, zero, {})
        mass = self.source_plant.source_plant.mass_kg
        inertia = self.source_plant.source_plant.inertia_body_kg_m2
        return {
            "force_x_n": -mass * baseline["u_m_s"],
            "force_y_n": -mass * baseline["v_m_s"],
            "force_z_n": -mass * baseline["w_m_s"],
            "moment_x_nm": -inertia[0] * baseline["p_rad_s"],
            "moment_y_nm": -inertia[1] * baseline["q_rad_s"],
            "moment_z_nm": -inertia[2] * baseline["r_rad_s"],
        }
        ####

    def _trim_spec(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimSpec:
        state = {name: float(target.get(name, self.reference_state[name])) for name in self.state_names}
        balance = self.balancing_wrench(state)
        controls = {name: float(initial_guess.get(name, balance[name])) for name in self.control_names}
        state_epsilon = 1.0e-9
        return TrimSpec(
            state_names=self.state_names,
            control_names=self.control_names,
            residual_names=self.state_names,
            state_initial=state,
            control_initial=controls,
            state_lower={name: value - state_epsilon for name, value in state.items()},
            state_upper={name: value + state_epsilon for name, value in state.items()},
            control_lower=self.limits.lower,
            control_upper=self.limits.upper,
            residual_scales={name: 1.0 for name in self.state_names},
            x_scale={
                **{name: max(1.0, abs(value)) for name, value in state.items()},
                **{
                    name: max(1.0, abs(self.limits.lower[name]), abs(self.limits.upper[name]))
                    for name in self.control_names
                },
            },
            operating_point={
                "control_realization": "direct_wrench",
                "equilibrium": "source_load_cancelled_local",
                "source_trim_claim": "not_source_physical_effector_trim",
                "altitude_m": self.altitude_m,
            },
        )
        ####

    def trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        """Solve the bounded source-load cancellation at the fixed condition."""

        spec = self._trim_spec(target, initial_guess)
        return solve_trim(
            spec,
            lambda state, controls: self.state_derivative(state, controls, {}),
            residual_tolerance=1.0e-10,
            acceptance_tolerance=1.0e-6,
        )
        ####

    def linearize(self, trim: TrimResult, options: Mapping[str, float | str]) -> ProvenancedLinearization:
        """Differentiate the same local nonlinear bridge used by trim."""

        if tuple(trim.spec.state_names) != self.state_names or tuple(trim.spec.control_names) != self.control_names:
            raise ValueError("HL-20 direct-wrench tuning linearization trim does not match the local bridge schema")
        return finite_difference_linearization_with_provenance(
            trim.spec,
            lambda state, controls: self.state_derivative(state, controls, {}),
            trim,
            nonlinear_plant_id="hl20-source-direct-wrench-local-plant",
            nonlinear_plant_revision="hl20-source-direct-wrench-local-v2",
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
                "source_load_model": "hl20-daveml-local-body-loads",
                "trim_claim": "local source-load cancellation only",
                "fixed_altitude_m": self.altitude_m,
            },
        )
        ####

    def effectiveness(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> EffectorEffectiveness:
        """Reject physical-effector effectiveness at the bridge tier."""

        del state, effectors
        raise RuntimeError("HL-20 direct-wrench bridge has no physical-effector effectiveness matrix")
        ####

    def allocate(
        self,
        state: Mapping[str, float],
        desired_wrench: Mapping[str, float],
        previous_effectors: Mapping[str, float],
        dt_s: float,
    ) -> PhysicalAllocationStep:
        """Reject physical allocation at the direct-wrench bridge tier."""

        del state, desired_wrench, previous_effectors, dt_s
        raise RuntimeError("HL-20 direct-wrench bridge has no physical-effectors allocator")
        ####


def build_hl20_source_direct_wrench_tuning_plant() -> HL20SourceDirectWrenchTuningPlant:
    """Construct the fixed-condition source-local controller plant."""

    screen = build_hl20_local_direct_wrench_screen_config()
    return HL20SourceDirectWrenchTuningPlant(
        source_plant=HL20SourceDirectWrenchPlant(),
        reference_state={name: float(screen.reference_state[name]) for name in HL20_LOCAL_TUNING_STATE_NAMES},
    )
    ####


def build_hl20_source_direct_wrench_tuning_adapter() -> StandardFamilyAdapter:
    """Build the executable local direct-wrench adapter used only for tuning."""

    plant = build_hl20_source_direct_wrench_tuning_plant()
    descriptor = descriptor_from_direct_wrench_state(
        family_id="hl20_mod_k",
        adapter_id="taoryx.lifting_body.daveml.v1",
        physical_family="lifting_body",
        state_names=plant.state_names,
        state_units={
            "u_m_s": "m/s",
            "v_m_s": "m/s",
            "w_m_s": "m/s",
            "p_rad_s": "rad/s",
            "q_rad_s": "rad/s",
            "r_rad_s": "rad/s",
        },
        evidence_status="development",
        validity_envelope="HL-20 Mach-0.5, alpha-5-degree, 10-km local DAVE-ML direct-wrench tuning screen",
        omitted_physics=(
            "physical surface allocation",
            "attitude and position propagation",
            "source-exact guidance or flight-control law",
            "closed-loop trajectory qualification",
        ),
    )
    return StandardFamilyAdapter.from_control_plant(descriptor, plant)
    ####


def build_hl20_direct_wrench_tuning_campaign() -> TuningCampaign:
    """Return the source-local HL-20 direct-wrench LQR design campaign."""

    return ControlAutomationDeclaration(
        id="hl20-source-subsonic-direct-wrench",
        campaign_id="hl20-source-subsonic-direct-wrench-v1",
        family_id="hl20_mod_k",
        tier="rigid_body_6dof_direct_wrench",
        strategy_id="lifting_body.v1",
        node_id="source-mach0p5-local",
        state_scales={
            "u_m_s": 200.0,
            "v_m_s": 100.0,
            "w_m_s": 100.0,
            "p_rad_s": 0.5,
            "q_rad_s": 0.5,
            "r_rad_s": 0.5,
        },
        control_scales={
            "force_x_n": 200_000.0,
            "force_y_n": 200_000.0,
            "force_z_n": 200_000.0,
            "moment_x_nm": 1_000_000.0,
            "moment_y_nm": 1_000_000.0,
            "moment_z_nm": 1_000_000.0,
        },
        authority_state_names=HL20_LOCAL_TUNING_STATE_NAMES,
        profile_grid_id_prefix="hl20-source-subsonic-direct-wrench",
    ).build_campaign()
    ####


def build_hl20_direct_wrench_lqi_tuning_campaign() -> TuningCampaign:
    """Return the source-local HL-20 offset-free speed bridge campaign.

    This integrates only body-forward velocity at the pinned Mach-0.5 source
    condition. The control coordinates remain the declared direct wrench
    bridge, so no result from this campaign implies an HL-20 surface allocator,
    actuator model, attitude law, or flight controller.
    """

    return ControlAutomationDeclaration(
        id="hl20-source-subsonic-direct-wrench-speed",
        campaign_id="hl20-source-subsonic-direct-wrench-lqi-v1",
        family_id="hl20_mod_k",
        tier="rigid_body_6dof_direct_wrench",
        strategy_id="lifting_body.v1",
        node_id="source-mach0p5-local",
        state_scales={
            "u_m_s": 200.0,
            "v_m_s": 100.0,
            "w_m_s": 100.0,
            "p_rad_s": 0.5,
            "q_rad_s": 0.5,
            "r_rad_s": 0.5,
        },
        control_scales={
            "force_x_n": 200_000.0,
            "force_y_n": 200_000.0,
            "force_z_n": 200_000.0,
            "moment_x_nm": 1_000_000.0,
            "moment_y_nm": 1_000_000.0,
            "moment_z_nm": 1_000_000.0,
        },
        authority_state_names=HL20_LOCAL_TUNING_STATE_NAMES,
        offset_free_outputs=("u_m_s",),
        integral_weight_multiplier=0.1,
        profile_grid_id_prefix="hl20-source-subsonic-direct-wrench-lqi",
    ).build_campaign()
    ####


@lru_cache(maxsize=1)
def build_hl20_local_direct_wrench_lqi_screen_config() -> LocalDirectWrenchScreenConfig:
    """Return the exact subsonic body-speed LQI bridge screen.

    The selected gain comes from the advertised common LQI campaign and is
    exercised only through the declared source-load direct-wrench coordinates.
    It is not a physical HL-20 surface, attitude, navigation, or glide law.
    """

    report = run_tuning_campaign(
        build_hl20_source_direct_wrench_tuning_adapter(),
        build_hl20_direct_wrench_lqi_tuning_campaign(),
    )
    node = report.nodes[0] if report.nodes else None
    candidate = node.lqr.best if node is not None and node.lqr is not None else None
    if candidate is None or candidate.lqi is None:
        raise RuntimeError("HL-20 subsonic direct-wrench LQI campaign produced no safe LQI candidate")
    baseline = build_hl20_local_direct_wrench_screen_config()
    initial = dict(baseline.reference_state)
    initial["u_m_s"] += 0.5
    return replace(
        baseline,
        id="hl20-source-subsonic-local-direct-wrench-lqi-v1",
        initial_state=initial,
        dt_s=0.01,
        duration_s=4.0,
        final_error_fraction_limit=0.05,
        controller_method="lqi",
        lqi_result=candidate.lqi,
        lqi_campaign_id="hl20-source-subsonic-direct-wrench-lqi-v1",
        integral_lower={name: -1.0 for name in candidate.lqi.output_names},
        integral_upper={name: 1.0 for name in candidate.lqi.output_names},
        assessment_state_names=("u_m_s",),
    )
    ####


def _hl20_state_channels() -> tuple[AdapterChannel, ...]:
    """Return the shared local state schema for both HL-20 rigid tiers."""

    return tuple(
        AdapterChannel(name, unit, "state", frame="body" if name != "altitude_m" else "NED")
        for name, unit in (
            ("u_m_s", "m/s"),
            ("v_m_s", "m/s"),
            ("w_m_s", "m/s"),
            ("p_rad_s", "rad/s"),
            ("q_rad_s", "rad/s"),
            ("r_rad_s", "rad/s"),
            ("altitude_m", "m"),
        )
    )
    ####


def build_hl20_source_direct_wrench_adapter() -> StandardFamilyAdapter:
    """Build the explicit HL-20 source-load direct-wrench bridge."""

    plant = HL20SourceDirectWrenchPlant()
    descriptor = descriptor_from_direct_wrench_state(
        family_id="hl20_mod_k",
        adapter_id="taoryx.lifting_body.daveml.v1",
        physical_family="lifting_body",
        state_names=tuple(item.name for item in _hl20_state_channels()),
        state_units={item.name: item.unit for item in _hl20_state_channels()},
        evidence_status="development",
        validity_envelope="HL-20 source local load witness; DAVE-ML source domain and declared bridge authority limits",
        omitted_physics=(
            "full-state source-exact trim",
            "physical surface allocation",
            "attitude and position propagation",
            "source-exact guidance or flight-control law",
            "closed-loop trajectory qualification",
        ),
    )
    return StandardFamilyAdapter.from_state_derivative(
        descriptor,
        plant.state_derivative,
        trim_fragment_provider=_hl20_trim_fragment,
    )
    ####


def build_hl20_source_adapter(tier: FidelityTier) -> StandardFamilyAdapter:
    """Build the supported HL-20 rigid-tier witness through one factory."""

    if tier == "rigid_body_6dof_direct_wrench":
        return build_hl20_source_direct_wrench_adapter()
    if tier == "rigid_body_6dof_surface_allocated":
        return build_hl20_source_surface_adapter(tier)
    raise ValueError(f"HL-20 source adapter does not implement {tier!r}")
    ####


def build_hl20_local_direct_wrench_screen_config() -> LocalDirectWrenchScreenConfig:
    """Return the declared source-feasible HL-20 direct-wrench screen.

    The selected Mach-0.5, alpha-5-degree, 10-km source point lies inside the
    retained DAVE-ML envelope and can be balanced within the declared direct
    wrench authority. Altitude is held as an environment parameter because
    this local bridge does not yet propagate attitude or position through
    gravity. This is a local bridge screen, not the high-energy glide mission.
    """

    return _build_hl20_local_direct_wrench_screen_config(
        id="hl20-source-mach0p5-local-direct-wrench-v1",
        altitude_m=10_000.0,
        mach=0.5,
        alpha_deg=5.0,
    )
    ####


def build_hl20_mach2_authority_probe_config() -> LocalDirectWrenchScreenConfig:
    """Return the deliberately blocked Mach-2 authority diagnostic fixture.

    This is retained as a source-owned negative control: the selected point
    requires balancing force beyond the currently declared bridge limits. It
    must never be bound as a runnable controller screen or a flight mission.
    """

    return _build_hl20_local_direct_wrench_screen_config(
        id="hl20-source-mach2-authority-probe-v1",
        altitude_m=10_000.0,
        mach=2.0,
        alpha_deg=5.0,
    )
    ####


def _build_hl20_local_direct_wrench_screen_config(
    *,
    id: str,
    altitude_m: float,
    mach: float,
    alpha_deg: float,
) -> LocalDirectWrenchScreenConfig:
    """Build one immutable source-domain local bridge fixture."""

    plant = HL20SourceDirectWrenchPlant()
    alpha_rad = math.radians(alpha_deg)
    speed_m_s = mach * plant.source_plant.source.speed_of_sound_m_s
    state_names = HL20_LOCAL_STATE_NAMES[:-1]
    reference = {
        "u_m_s": speed_m_s * math.cos(alpha_rad),
        "v_m_s": 0.0,
        "w_m_s": -speed_m_s * math.sin(alpha_rad),
        "p_rad_s": 0.0,
        "q_rad_s": 0.0,
        "r_rad_s": 0.0,
    }

    def augmented(state: Mapping[str, float]) -> dict[str, float]:
        return {**{name: float(state[name]) for name in state_names}, "altitude_m": altitude_m}
        ####

    def derivative(state: Mapping[str, float], wrench: Mapping[str, float]) -> dict[str, float]:
        full = plant.state_derivative(augmented(state), wrench, {"altitude_m": altitude_m, "dt_s": 1.0})
        return {name: float(full[name]) for name in state_names}
        ####

    def balance(state: Mapping[str, float]) -> dict[str, float]:
        baseline = derivative(state, {name: 0.0 for name in DIRECT_WRENCH_NAMES})
        return {
            "force_x_n": -plant.source_plant.mass_kg * baseline["u_m_s"],
            "force_y_n": -plant.source_plant.mass_kg * baseline["v_m_s"],
            "force_z_n": -plant.source_plant.mass_kg * baseline["w_m_s"],
            "moment_x_nm": -plant.source_plant.inertia_body_kg_m2[0] * baseline["p_rad_s"],
            "moment_y_nm": -plant.source_plant.inertia_body_kg_m2[1] * baseline["q_rad_s"],
            "moment_z_nm": -plant.source_plant.inertia_body_kg_m2[2] * baseline["r_rad_s"],
        }
        ####

    initial = dict(reference)
    initial.update(
        {
            "u_m_s": reference["u_m_s"] + 1.0,
            "v_m_s": 0.4,
            "w_m_s": reference["w_m_s"] + 0.8,
            "p_rad_s": 0.02,
            "q_rad_s": -0.015,
            "r_rad_s": 0.01,
        }
    )
    return LocalDirectWrenchScreenConfig(
        id=id,
        plant_id="hl20-daveml-source-direct-wrench-local-plant",
        state_names=state_names,
        reference_state=reference,
        initial_state=initial,
        source_derivative=derivative,
        balancing_wrench=balance,
        limits=plant.limits,
        state_scales=(20.0, 20.0, 20.0, 0.15, 0.15, 0.15),
        control_scales=(1.0e6, 1.0e6, 1.0e6, 5.0e6, 5.0e6, 5.0e6),
        state_cost_weights=(2.0, 2.0, 2.0, 2.0, 2.0, 2.0),
        control_cost_weights=(20.0, 20.0, 20.0, 20.0, 20.0, 20.0),
        # Two milliseconds keeps the explicit Euler screen within its
        # feasible source-local recovery envelope while avoiding a needlessly
        # dense 4,000-step test and CLI witness.
        dt_s=0.002,
        duration_s=4.0,
        resource_values={"mass_kg": plant.source_plant.mass_kg},
    )
    ####


def build_hl20_source_surface_adapter(
    tier: FidelityTier = "rigid_body_6dof_surface_allocated",
) -> StandardFamilyAdapter:
    """Build the local physical-source-surface HL-20 adapter façade."""

    if tier != "rigid_body_6dof_surface_allocated":
        raise ValueError("HL-20 source-surface adapter is only available at the surface-allocation tier")
    plant = build_hl20_source_surface_local_plant()
    descriptor = descriptor_from_control_plant(
        plant,
        family_id="hl20_mod_k",
        adapter_id="taoryx.lifting_body.daveml.v1",
        physical_family="lifting_body",
        tier=tier,
        state_units={
            "roll_error_rad": "rad",
            "pitch_error_rad": "rad",
            "yaw_error_rad": "rad",
            "p_rad_s": "rad/s",
            "q_rad_s": "rad/s",
            "r_rad_s": "rad/s",
        },
        control_units={name: "deg" for name in HL20_SURFACE_NAMES},
        evidence_status="development",
        validity_envelope=(
            "HL-20 DAVE-ML Mach-1 scalar pitch-trim fragment; frozen source translation, small local attitude "
            "errors, and declared seven-surface bounds/rate/lag realization"
        ),
        omitted_physics=(
            "full source physical-effector glide trim",
            "translation and position propagation beyond the frozen local fixture",
            "gravity resolution through attitude",
            "source-exact guidance or flight-control law",
            "closed-loop trajectory qualification",
        ),
    )
    return StandardFamilyAdapter.from_control_plant(
        descriptor,
        plant,
        trim_fragment_provider=_hl20_trim_fragment,
    )
    ####


def _hl20_trim_fragment(request: Mapping[str, float | str]) -> TrimFragmentResult:
    """Return the checked scalar HL-20 pitch-channel trim fragment.

    The request is intentionally advisory at this stage because the checked
    evidence artifact contains one pinned Mach-1 operating point.  Full-state
    trim remains a separate operation and is not inferred from this fragment.
    """

    del request
    evidence = json.loads(HL20_TRIM_EVIDENCE.read_text(encoding="utf-8"))
    trim = evidence["trim"]
    source = evidence["source"]
    return TrimFragmentResult(
        fragment_id="hl20_source_pitch_channel_trim",
        status="verified" if evidence.get("status") == "verified" and trim.get("success") else "failed",
        state={str(name): float(value) for name, value in trim.get("state", {}).items()},
        controls={str(name): float(value) for name, value in trim.get("controls", {}).items()},
        residuals={str(name): float(value) for name, value in trim.get("residuals", {}).items()},
        max_residual=float(trim.get("max_residual", float("inf"))),
        claim_boundary=str(evidence["claim_boundary"]),
        operating_point={str(name): value for name, value in evidence.get("operating_point", {}).items()},
        provenance={
            "document_sha256": str(source["document_sha256"]),
            "package_sha256": str(source["package_sha256"]),
            "evidence_artifact": str(HL20_TRIM_EVIDENCE.relative_to(HL20_TRIM_EVIDENCE.parents[1])),
        },
    )
    ####


def _require_state(state: Mapping[str, float]) -> dict[str, float]:
    missing = set(HL20_LOCAL_STATE_NAMES) - set(state)
    if missing:
        raise KeyError("HL-20 source adapter state is missing: " + ", ".join(sorted(missing)))
    values = {name: float(state[name]) for name in HL20_LOCAL_STATE_NAMES}
    if any(not math.isfinite(value) for value in values.values()):
        raise ValueError("HL-20 source adapter state must be finite")
    return values
    ####


def _cross(left: tuple[float, float, float], right: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )
    ####


__all__ = [
    "HL20_LOCAL_STATE_NAMES",
    "HL20_LOCAL_TUNING_STATE_NAMES",
    "HL20_SOURCE_PITCH_TRIM_ALPHA_DEG",
    "HL20_SOURCE_PITCH_TRIM_ALTITUDE_M",
    "HL20_SURFACE_LOCAL_STATE_NAMES",
    "HL20SourceDirectWrenchPlant",
    "HL20SourceDirectWrenchTuningPlant",
    "HL20SourceSurfacePlant",
    "HL20SourceSurfaceLocalPlant",
    "HL20_TRIM_EVIDENCE",
    "build_hl20_local_direct_wrench_lqi_screen_config",
    "build_hl20_local_direct_wrench_screen_config",
    "build_hl20_mach2_authority_probe_config",
    "build_hl20_source_adapter",
    "build_hl20_direct_wrench_lqi_tuning_campaign",
    "build_hl20_direct_wrench_tuning_campaign",
    "build_hl20_source_direct_wrench_adapter",
    "build_hl20_source_direct_wrench_tuning_adapter",
    "build_hl20_source_direct_wrench_tuning_plant",
    "build_hl20_source_surface_local_plant",
    "build_hl20_source_surface_lqi_tuning_campaign",
    "build_hl20_source_surface_physical_lqi_design",
    "build_hl20_source_surface_physical_lqr_design",
    "build_hl20_source_surface_adapter",
    "hl20_direct_wrench_limits",
]
####
