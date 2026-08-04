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
from dataclasses import dataclass, field
from pathlib import Path

from .control_allocation import (
    EffectorEffectiveness,
    EffectorLimits,
    PhysicalAllocationStep,
    allocate_and_advance_wrench,
)
from .direct_wrench import DIRECT_WRENCH_NAMES, DirectWrenchLimits, add_direct_wrench_to_local_derivative
from .family_adapter import (
    AdapterChannel,
    FamilyAdapterDescriptor,
    StandardFamilyAdapter,
    TrimFragmentResult,
    descriptor_from_direct_wrench_state,
)
from .fidelity_contracts import FidelityTier
from .hl20_controls import HL20_SOURCE_SURFACE_BOUNDS_DEG, HL20_SURFACE_NAMES
from .hl20_reachability import HL20_FIXED_MASS_KG, HL20_REFERENCE_INERTIA_KG_M2
from .local_direct_wrench import LocalDirectWrenchScreenConfig
from .reachability_aerodynamics import (
    HL20DavemlAerodynamics,
    build_hl20_source_effectiveness,
)

HL20_LOCAL_STATE_NAMES = (
    "u_m_s",
    "v_m_s",
    "w_m_s",
    "p_rad_s",
    "q_rad_s",
    "r_rad_s",
    "altitude_m",
)
HL20_TRIM_EVIDENCE = Path(__file__).resolve().parents[2] / "verification/daveml_hl20_trim_evidence.json"


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
    ####

    @property
    def control_names(self) -> tuple[str, ...]:
        return DIRECT_WRENCH_NAMES
        ####
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
        dt_s=0.001,
        duration_s=4.0,
    )
    ####


def build_hl20_source_surface_adapter(
    tier: FidelityTier = "rigid_body_6dof_surface_allocated",
) -> StandardFamilyAdapter:
    """Build the honest source-surface allocation witness façade."""

    if tier != "rigid_body_6dof_surface_allocated":
        raise ValueError("HL-20 source-surface adapter is only available at the surface-allocation tier")
    plant = HL20SourceSurfacePlant()
    descriptor = FamilyAdapterDescriptor(
        family_id="hl20_mod_k",
        adapter_id="taoryx.lifting_body.daveml.v1",
        physical_family="lifting_body",
        tier=tier,
        state_channels=_hl20_state_channels(),
        control_channels=tuple(AdapterChannel(name, "deg", "effector", frame="body") for name in HL20_SURFACE_NAMES),
        evidence_status="development",
        validity_envelope="HL-20 DAVE-ML Mach 0.3-4.0, alpha 0-15 deg, beta +/-10 deg, altitude -1000-20000 m",
        omitted_physics=(
            "attitude and position propagation",
            "gravity resolution through attitude",
            "full-state source-exact trim (only a scalar pitch-channel fragment is available)",
            "source-exact guidance or flight-control law",
            "closed-loop trajectory qualification",
        ),
    )
    return StandardFamilyAdapter.from_state_derivative(
        descriptor,
        plant.state_derivative,
        effectiveness_provider=plant.effectiveness,
        allocation_provider=plant.allocate,
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
    "HL20SourceDirectWrenchPlant",
    "HL20SourceSurfacePlant",
    "HL20_TRIM_EVIDENCE",
    "build_hl20_local_direct_wrench_screen_config",
    "build_hl20_mach2_authority_probe_config",
    "build_hl20_source_adapter",
    "build_hl20_source_direct_wrench_adapter",
    "build_hl20_source_surface_adapter",
    "hl20_direct_wrench_limits",
]
####
