"""Common metadata and capability façade for vehicle-family adapters.

The numerical plant contract lives in :mod:`taoryx.control_allocation`.  This
module adds the horizontal integration seam around it: every family exposes
the same identity, channel schemas, capability report, and delegated plant
operations.  The façade does not make an unavailable operation executable;
it makes that limitation explicit before a trim, controller, or showcase run
is attempted.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any, Literal, Protocol

from .control_allocation import (
    ControlPlantAdapter,
    EffectorEffectiveness,
    PhysicalAllocationStep,
    ProvenancedLinearization,
)
from .direct_wrench import DIRECT_WRENCH_NAMES
from .fidelity_contracts import ControlRealization, FidelityTier, control_realization_for
from .trim import TrimResult

CapabilityStatus = Literal["available", "not_applicable", "not_available", "planned"]
AdapterOperation = Literal[
    "state_derivative",
    "trim",
    "trim_fragment",
    "linearize",
    "effectiveness",
    "allocate",
    "resource_rates",
    "observe",
    "replay",
]
ADAPTER_OPERATIONS: tuple[AdapterOperation, ...] = (
    "state_derivative",
    "trim",
    "trim_fragment",
    "linearize",
    "effectiveness",
    "allocate",
    "resource_rates",
    "observe",
    "replay",
)
ConformanceSeverity = Literal["error", "warning"]
ResourceRateProvider = Callable[[Mapping[str, float], Mapping[str, float]], Mapping[str, float]]
ObservationProvider = Callable[
    [Mapping[str, float], Mapping[str, float], Mapping[str, float], Sequence[Mapping[str, Any]]],
    Mapping[str, Any],
]
ReplayProvider = Callable[[Mapping[str, Any]], Mapping[str, Any]]
StateDerivativeProvider = Callable[
    [Mapping[str, float], Mapping[str, float], Mapping[str, float | str]],
    Mapping[str, float],
]
EffectivenessProvider = Callable[
    [Mapping[str, float], Mapping[str, float]],
    EffectorEffectiveness,
]
AllocationProvider = Callable[
    [Mapping[str, float], Mapping[str, float], Mapping[str, float], float],
    PhysicalAllocationStep,
]


@dataclass(frozen=True, slots=True)
class TrimFragmentResult:
    """Auditable source-backed trim evidence that is not a full plant trim.

    A fragment may solve one channel, residual, or source operating-point
    condition.  It is intentionally separate from :class:`TrimResult` so a
    scalar aerodynamic equilibrium cannot be mistaken for a full-state,
    physical-effector trim suitable for controller synthesis.
    """

    fragment_id: str
    status: Literal["verified", "development", "failed"]
    state: Mapping[str, float]
    controls: Mapping[str, float]
    residuals: Mapping[str, float]
    max_residual: float
    claim_boundary: str
    operating_point: Mapping[str, float | str] = field(default_factory=dict)
    provenance: Mapping[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.fragment_id.strip() or not self.claim_boundary.strip():
            raise ValueError("trim fragments require an ID and claim boundary")
        if not math.isfinite(float(self.max_residual)) or self.max_residual < 0.0:
            raise ValueError("trim fragment maximum residual must be finite and nonnegative")
        for label, values in (("state", self.state), ("controls", self.controls), ("residuals", self.residuals)):
            if any(not math.isfinite(float(value)) for value in values.values()):
                raise ValueError(f"trim fragment {label} must contain finite values")
        if any(not math.isfinite(float(value)) for value in self.operating_point.values() if not isinstance(value, str)):
            raise ValueError("trim fragment operating-point numeric values must be finite")
        ####
    ####

    def as_dict(self) -> dict[str, Any]:
        """Return the fragment as a machine-readable evidence record."""

        return {
            "fragment_id": self.fragment_id,
            "status": self.status,
            "state": dict(self.state),
            "controls": dict(self.controls),
            "residuals": dict(self.residuals),
            "max_residual": self.max_residual,
            "claim_boundary": self.claim_boundary,
            "operating_point": dict(self.operating_point),
            "provenance": dict(self.provenance),
        }
        ####
    ####


TrimFragmentProvider = Callable[[Mapping[str, float | str]], TrimFragmentResult]


@dataclass(frozen=True, slots=True)
class AdapterChannel:
    """One named channel in a family adapter's public schema."""

    name: str
    unit: str
    role: str
    frame: str | None = None
    description: str = ""

    def __post_init__(self) -> None:
        if not self.name.strip() or not self.unit.strip() or not self.role.strip():
            raise ValueError("adapter channels require name, unit, and role")
        ####
    ####


@dataclass(frozen=True, slots=True)
class FamilyAdapterDescriptor:
    """Stable identity and claim boundary for one adapter realization."""

    family_id: str
    adapter_id: str
    physical_family: str
    tier: FidelityTier
    state_channels: tuple[AdapterChannel, ...]
    control_channels: tuple[AdapterChannel, ...] = ()
    resource_channels: tuple[AdapterChannel, ...] = ()
    control_realization_override: ControlRealization | None = None
    evidence_status: str = "development"
    validity_envelope: str = ""
    omitted_physics: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not self.family_id.strip() or not self.adapter_id.strip() or not self.physical_family.strip():
            raise ValueError("adapter descriptor requires family, adapter, and physical-family identity")
        if not self.state_channels:
            raise ValueError("adapter descriptor requires at least one state channel")
        _require_unique_channels(self.state_channels, "state")
        _require_unique_channels(self.control_channels, "control")
        _require_unique_channels(self.resource_channels, "resource")
        ####
    ####

    @property
    def control_realization(self) -> ControlRealization:
        """Return the realization implied by the canonical tier."""

        return self.control_realization_override or control_realization_for(self.tier)
        ####
    ####

    def as_dict(self) -> dict[str, Any]:
        """Return the reproducible descriptor used in integration artifacts."""

        return {
            "family_id": self.family_id,
            "adapter_id": self.adapter_id,
            "physical_family": self.physical_family,
            "tier": self.tier,
            "control_realization": self.control_realization,
            "state_channels": [_channel_dict(item) for item in self.state_channels],
            "control_channels": [_channel_dict(item) for item in self.control_channels],
            "resource_channels": [_channel_dict(item) for item in self.resource_channels],
            "evidence_status": self.evidence_status,
            "validity_envelope": self.validity_envelope,
            "omitted_physics": list(self.omitted_physics),
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class AdapterCapability:
    """Typed availability result for one adapter operation."""

    operation: AdapterOperation
    status: CapabilityStatus
    reason: str
    evidence: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not self.reason.strip():
            raise ValueError("adapter capability results require a reason")
        if any(not item.strip() for item in self.evidence):
            raise ValueError("adapter capability evidence identifiers must be non-empty")
        ####
    ####

    @property
    def usable(self) -> bool:
        """Return whether the operation may be invoked by the pipeline."""

        return self.status == "available"
        ####
    ####

    def as_dict(self) -> dict[str, Any]:
        """Return a stable machine-readable capability record."""

        return {
            "operation": self.operation,
            "status": self.status,
            "usable": self.usable,
            "reason": self.reason,
            "evidence": list(self.evidence),
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class FamilyCapabilityReport:
    """Complete operation-level capability report for one adapter."""

    descriptor: FamilyAdapterDescriptor
    capabilities: tuple[AdapterCapability, ...]

    def __post_init__(self) -> None:
        operations = tuple(item.operation for item in self.capabilities)
        if len(operations) != len(set(operations)):
            raise ValueError("adapter capability operations must be unique")
        ####
    ####

    def capability(self, operation: AdapterOperation) -> AdapterCapability:
        """Return one operation capability or fail with an actionable error."""

        for item in self.capabilities:
            if item.operation == operation:
                return item
        raise KeyError(f"adapter capability {operation!r} is not declared")
        ####
    ####

    @property
    def errors(self) -> tuple[AdapterCapability, ...]:
        """Return malformed capability states that block execution."""

        return tuple(item for item in self.capabilities if item.status == "not_available" and not item.reason.strip())
        ####
    ####

    def as_dict(self) -> dict[str, Any]:
        """Return descriptor and operation capabilities as one artifact."""

        return {
            "schema": "taoryx.family-adapter-capability/v1alpha1",
            "descriptor": self.descriptor.as_dict(),
            "capabilities": [item.as_dict() for item in self.capabilities],
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class AdapterConformanceFinding:
    """One tier-contract finding produced before runtime execution."""

    severity: ConformanceSeverity
    code: str
    message: str

    def as_dict(self) -> dict[str, str]:
        """Return a stable machine-readable finding."""

        return {"severity": self.severity, "code": self.code, "message": self.message}
        ####
    ####


@dataclass(frozen=True, slots=True)
class AdapterConformanceReport:
    """Result of applying the same adapter checks to one family/tier."""

    family_id: str
    adapter_id: str
    tier: FidelityTier
    status: Literal["pass", "fail"]
    findings: tuple[AdapterConformanceFinding, ...]

    @property
    def errors(self) -> tuple[AdapterConformanceFinding, ...]:
        """Return blocking adapter findings."""

        return tuple(item for item in self.findings if item.severity == "error")
        ####

    def as_dict(self) -> dict[str, Any]:
        """Return a stable conformance artifact."""

        return {
            "schema": "taoryx.family-adapter-conformance/v1alpha1",
            "family_id": self.family_id,
            "adapter_id": self.adapter_id,
            "tier": self.tier,
            "status": self.status,
            "findings": [item.as_dict() for item in self.findings],
        }
        ####
    ####


class AdapterCapabilityError(RuntimeError):
    """Raised when a caller invokes an explicitly unavailable operation."""


class FamilyAdapter(Protocol):
    """Common façade consumed by trim, control, and showcase pipelines."""

    def describe(self) -> FamilyAdapterDescriptor:
        """Return stable identity and channel metadata."""
        ...

    def state_schema(self) -> tuple[AdapterChannel, ...]:
        """Return ordered state channels."""
        ...

    def control_schema(self) -> tuple[AdapterChannel, ...]:
        """Return ordered control or effector channels."""
        ...

    def resource_schema(self) -> tuple[AdapterChannel, ...]:
        """Return ordered resource channels."""
        ...

    def capability_report(self) -> FamilyCapabilityReport:
        """Return typed operation availability."""
        ...

    def state_derivative(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        """Evaluate the nonlinear state derivative."""
        ...

    def trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        """Solve or report the declared trim operation."""
        ...

    def trim_fragment(self, request: Mapping[str, float | str]) -> TrimFragmentResult:
        """Return source-backed partial trim evidence without full-plant claims."""
        ...

    def linearize(self, trim: TrimResult, options: Mapping[str, float | str]) -> ProvenancedLinearization:
        """Produce a provenance-bearing local linearization."""
        ...

    def effectiveness(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> EffectorEffectiveness:
        """Evaluate local control effectiveness."""
        ...

    def allocate(
        self,
        state: Mapping[str, float],
        desired_wrench: Mapping[str, float],
        previous_effectors: Mapping[str, float],
        dt_s: float,
    ) -> PhysicalAllocationStep:
        """Allocate a requested wrench through declared effectors."""
        ...

    def resource_rates(self, state: Mapping[str, float], commands: Mapping[str, float]) -> Mapping[str, float]:
        """Return resource consumption rates."""
        ...

    def observe(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        resources: Mapping[str, float],
        events: Sequence[Mapping[str, Any]],
    ) -> Mapping[str, Any]:
        """Return canonical observation/diagnostic channels."""
        ...

    def replay(self, request: Mapping[str, Any]) -> Mapping[str, Any]:
        """Replay a source-backed trajectory when no participating plant exists."""
        ...


@dataclass(slots=True)
class StandardFamilyAdapter:
    """Wrap an existing plant with the common family integration façade.

    ``plant`` may be omitted for an open-loop or passive family.  Such an
    adapter can still participate in manifest, schema, lowering, and showcase
    validation, while every unavailable numerical operation remains explicit.
    """

    descriptor: FamilyAdapterDescriptor
    plant: ControlPlantAdapter | None = None
    declared_capabilities: Mapping[AdapterOperation, AdapterCapability] = field(default_factory=dict)
    resource_rate_provider: ResourceRateProvider | None = None
    observation_provider: ObservationProvider | None = None
    state_derivative_provider: StateDerivativeProvider | None = None
    effectiveness_provider: EffectivenessProvider | None = None
    allocation_provider: AllocationProvider | None = None
    replay_provider: ReplayProvider | None = None
    trim_fragment_provider: TrimFragmentProvider | None = None

    def __post_init__(self) -> None:
        if self.plant is not None:
            if tuple(self.plant.state_names) != tuple(item.name for item in self.descriptor.state_channels):
                raise ValueError("adapter state schema does not match delegated plant state_names")
            if tuple(self.plant.control_names) != tuple(item.name for item in self.descriptor.control_channels):
                raise ValueError("adapter control schema does not match delegated plant control_names")
        unknown = set(self.declared_capabilities) - set(ADAPTER_OPERATIONS)
        if unknown:
            raise ValueError("unknown adapter capabilities: " + ", ".join(sorted(unknown)))
        ####
    ####

    @classmethod
    def from_control_plant(
        cls,
        descriptor: FamilyAdapterDescriptor,
        plant: ControlPlantAdapter,
        *,
        resource_channels: Sequence[AdapterChannel] = (),
        extra_capabilities: Mapping[AdapterOperation, AdapterCapability] | None = None,
        resource_rate_provider: ResourceRateProvider | None = None,
        observation_provider: ObservationProvider | None = None,
        trim_fragment_provider: TrimFragmentProvider | None = None,
    ) -> StandardFamilyAdapter:
        """Build a façade from an existing numerical plant without rewriting it."""

        if resource_channels:
            descriptor = FamilyAdapterDescriptor(
                family_id=descriptor.family_id,
                adapter_id=descriptor.adapter_id,
                physical_family=descriptor.physical_family,
                tier=descriptor.tier,
                state_channels=descriptor.state_channels,
                control_channels=descriptor.control_channels,
                resource_channels=tuple(resource_channels),
                control_realization_override=descriptor.control_realization_override,
                evidence_status=descriptor.evidence_status,
                validity_envelope=descriptor.validity_envelope,
                omitted_physics=descriptor.omitted_physics,
            )
        base_operations: tuple[AdapterOperation, ...] = ("state_derivative", "trim", "linearize")
        capabilities: dict[AdapterOperation, AdapterCapability] = {
            operation: AdapterCapability(operation, "available", "delegated to the nonlinear family plant")
            for operation in base_operations
        }
        effector_operations: tuple[AdapterOperation, ...] = ("effectiveness", "allocate")
        capabilities.update(
            {
                operation: AdapterCapability(
                    operation,
                    "available" if descriptor.control_realization == "surface_allocated" else "not_applicable",
                    (
                        "delegated to the nonlinear family plant"
                        if descriptor.control_realization == "surface_allocated"
                        else f"{descriptor.control_realization} tier does not claim physical effector allocation"
                    ),
                )
                for operation in effector_operations
            }
        )
        capabilities.update(extra_capabilities or {})
        if trim_fragment_provider is not None:
            capabilities["trim_fragment"] = AdapterCapability(
                "trim_fragment", "available", "delegated to the source-backed trim-fragment provider"
            )
        if resource_rate_provider is not None:
            capabilities["resource_rates"] = AdapterCapability(
                "resource_rates", "available", "delegated to the family resource provider"
            )
        if observation_provider is not None:
            capabilities["observe"] = AdapterCapability(
                "observe", "available", "delegated to the family observation provider"
            )
        capabilities["replay"] = AdapterCapability(
            "replay", "not_applicable", "participating plant adapter does not expose source replay"
        )
        return cls(
            descriptor,
            plant,
            capabilities,
            resource_rate_provider,
            observation_provider,
            trim_fragment_provider=trim_fragment_provider,
        )
        ####
    ####

    @classmethod
    def passive(
        cls,
        descriptor: FamilyAdapterDescriptor,
        *,
        capabilities: Mapping[AdapterOperation, AdapterCapability] | None = None,
    ) -> StandardFamilyAdapter:
        """Build an explicit open-loop/passive façade with no fake controls."""

        defaults = {
            operation: AdapterCapability(operation, "not_applicable", "passive or open-loop family has no controlled plant")
            for operation in ADAPTER_OPERATIONS
        }
        defaults.update(capabilities or {})
        return cls(descriptor, None, defaults)
        ####
    ####

    @classmethod
    def from_state_derivative(
        cls,
        descriptor: FamilyAdapterDescriptor,
        state_derivative_provider: StateDerivativeProvider,
        *,
        resource_rate_provider: ResourceRateProvider | None = None,
        observation_provider: ObservationProvider | None = None,
        effectiveness_provider: EffectivenessProvider | None = None,
        allocation_provider: AllocationProvider | None = None,
        trim_fragment_provider: TrimFragmentProvider | None = None,
    ) -> StandardFamilyAdapter:
        """Build a façade around a provider-backed plant.

        This is the path for staged open-loop vehicles, uncontrolled bodies,
        and source-backed partial plants.  A family may provide a nonlinear
        derivative plus an independent local effectiveness/allocator pair
        before it has a defensible trim or closed-loop linearization.  Each
        operation remains independently declared; the façade never infers a
        missing operation from a neighboring one.
        """

        capabilities: dict[AdapterOperation, AdapterCapability] = {
            operation: AdapterCapability(operation, "not_applicable", "open-loop or passive adapter has no declared operation")
            for operation in ADAPTER_OPERATIONS
        }
        capabilities["state_derivative"] = AdapterCapability(
            "state_derivative", "available", "delegated to the state-only family plant"
        )
        if resource_rate_provider is not None:
            capabilities["resource_rates"] = AdapterCapability(
                "resource_rates", "available", "delegated to the family resource provider"
            )
        if observation_provider is not None:
            capabilities["observe"] = AdapterCapability(
                "observe", "available", "delegated to the family observation provider"
            )
        if effectiveness_provider is not None:
            capabilities["effectiveness"] = AdapterCapability(
                "effectiveness", "available", "delegated to the source-backed effectiveness provider"
            )
        if allocation_provider is not None:
            capabilities["allocate"] = AdapterCapability(
                "allocate", "available", "delegated to the source-backed allocation provider"
            )
        if trim_fragment_provider is not None:
            capabilities["trim_fragment"] = AdapterCapability(
                "trim_fragment", "available", "delegated to the source-backed trim-fragment provider"
            )
        capabilities["replay"] = AdapterCapability(
            "replay", "not_applicable", "state-derivative adapter does not expose source replay"
        )
        return cls(
            descriptor,
            None,
            capabilities,
            resource_rate_provider,
            observation_provider,
            state_derivative_provider,
            effectiveness_provider,
            allocation_provider,
            trim_fragment_provider=trim_fragment_provider,
        )
        ####
    ####

    def describe(self) -> FamilyAdapterDescriptor:
        """Return the stable adapter descriptor."""

        return self.descriptor
        ####
    ####

    def state_schema(self) -> tuple[AdapterChannel, ...]:
        """Return the ordered state schema."""

        return self.descriptor.state_channels
        ####
    ####

    def control_schema(self) -> tuple[AdapterChannel, ...]:
        """Return the ordered control schema."""

        return self.descriptor.control_channels
        ####
    ####

    def resource_schema(self) -> tuple[AdapterChannel, ...]:
        """Return the ordered resource schema."""

        return self.descriptor.resource_channels
        ####
    ####

    def capability_report(self) -> FamilyCapabilityReport:
        """Return all operation capabilities in stable order."""

        capabilities = tuple(
            self.declared_capabilities.get(operation, _missing_capability(operation))
            for operation in ADAPTER_OPERATIONS
        )
        return FamilyCapabilityReport(self.descriptor, capabilities)
        ####
    ####

    @property
    def state_names(self) -> tuple[str, ...]:
        """Expose the delegated plant state ordering for generic tuners."""

        return tuple(item.name for item in self.descriptor.state_channels)
        ####
    ####

    @property
    def control_names(self) -> tuple[str, ...]:
        """Expose the delegated plant control ordering for generic tuners."""

        return tuple(item.name for item in self.descriptor.control_channels)
        ####
    ####

    def state_derivative(self, state: Mapping[str, float], effectors: Mapping[str, float], environment: Mapping[str, float | str]) -> Mapping[str, float]:
        """Delegate nonlinear derivatives after checking declared capability."""

        capability = self.capability_report().capability("state_derivative")
        if not capability.usable:
            raise AdapterCapabilityError(f"{self.descriptor.family_id}.state_derivative: {capability.status}: {capability.reason}")
        if self.plant is not None:
            return self.plant.state_derivative(state, effectors, environment)
        if self.state_derivative_provider is not None:
            return self.state_derivative_provider(state, effectors, environment)
        raise AdapterCapabilityError(f"{self.descriptor.family_id}.state_derivative: no provider is bound")
        ####
    ####

    def trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        """Delegate physical-effector trim after checking declared capability."""

        plant = self._require_plant("trim")
        return plant.trim(target, initial_guess)
        ####
    ####

    def trim_fragment(self, request: Mapping[str, float | str]) -> TrimFragmentResult:
        """Return a declared source trim fragment without promoting full trim."""

        capability = self.capability_report().capability("trim_fragment")
        if not capability.usable or self.trim_fragment_provider is None:
            raise AdapterCapabilityError(
                f"{self.descriptor.family_id}.trim_fragment: {capability.status}: {capability.reason}"
            )
        return self.trim_fragment_provider(request)
        ####
    ####

    def linearize(self, trim: TrimResult, options: Mapping[str, float | str]) -> ProvenancedLinearization:
        """Delegate plant linearization after checking declared capability."""

        plant = self._require_plant("linearize")
        return plant.linearize(trim, options)
        ####
    ####

    def effectiveness(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> EffectorEffectiveness:
        """Delegate local physical-effector effectiveness."""

        capability = self.capability_report().capability("effectiveness")
        if not capability.usable:
            raise AdapterCapabilityError(f"{self.descriptor.family_id}.effectiveness: {capability.status}: {capability.reason}")
        if self.plant is not None:
            return self.plant.effectiveness(state, effectors)
        if self.effectiveness_provider is not None:
            return self.effectiveness_provider(state, effectors)
        raise AdapterCapabilityError(f"{self.descriptor.family_id}.effectiveness: no provider is bound")
        ####
    ####

    def allocate(self, state: Mapping[str, float], desired_wrench: Mapping[str, float], previous_effectors: Mapping[str, float], dt_s: float) -> PhysicalAllocationStep:
        """Delegate constrained allocation and actuator advancement."""

        capability = self.capability_report().capability("allocate")
        if not capability.usable:
            raise AdapterCapabilityError(f"{self.descriptor.family_id}.allocate: {capability.status}: {capability.reason}")
        if self.plant is not None:
            return self.plant.allocate(state, desired_wrench, previous_effectors, dt_s)
        if self.allocation_provider is not None:
            return self.allocation_provider(state, desired_wrench, previous_effectors, dt_s)
        raise AdapterCapabilityError(f"{self.descriptor.family_id}.allocate: no provider is bound")
        ####
    ####

    def resource_rates(self, state: Mapping[str, float], commands: Mapping[str, float]) -> Mapping[str, float]:
        """Return resource rates through the declared family provider."""

        capability = self.capability_report().capability("resource_rates")
        if not capability.usable or self.resource_rate_provider is None:
            raise AdapterCapabilityError(f"{self.descriptor.family_id}.resource_rates: {capability.status}: {capability.reason}")
        return self.resource_rate_provider(state, commands)
        ####
    ####

    def observe(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        resources: Mapping[str, float],
        events: Sequence[Mapping[str, Any]],
    ) -> Mapping[str, Any]:
        """Return observations through the declared family provider."""

        capability = self.capability_report().capability("observe")
        if not capability.usable or self.observation_provider is None:
            raise AdapterCapabilityError(f"{self.descriptor.family_id}.observe: {capability.status}: {capability.reason}")
        return self.observation_provider(state, controls, resources, events)
        ####

    @classmethod
    def from_replay(
        cls,
        descriptor: FamilyAdapterDescriptor,
        replay_provider: ReplayProvider,
    ) -> StandardFamilyAdapter:
        """Build a source-replay adapter without inventing a plant derivative."""

        capabilities = {
            operation: AdapterCapability(operation, "not_applicable", "source-replay adapter has no participating operation")
            for operation in ADAPTER_OPERATIONS
        }
        capabilities["replay"] = AdapterCapability(
            "replay", "available", "delegated to the source-replay provider"
        )
        return cls(descriptor, declared_capabilities=capabilities, replay_provider=replay_provider)
        ####
    ####

    def replay(self, request: Mapping[str, Any]) -> Mapping[str, Any]:
        """Return a source-backed replay artifact through the common seam."""

        capability = self.capability_report().capability("replay")
        if not capability.usable or self.replay_provider is None:
            raise AdapterCapabilityError(f"{self.descriptor.family_id}.replay: {capability.status}: {capability.reason}")
        return self.replay_provider(request)
        ####
    ####

    def _require_plant(self, operation: AdapterOperation) -> ControlPlantAdapter:
        capability = self.capability_report().capability(operation)
        if not capability.usable or self.plant is None:
            raise AdapterCapabilityError(f"{self.descriptor.family_id}.{operation}: {capability.status}: {capability.reason}")
        return self.plant
        ####
    ####


def descriptor_from_control_plant(
    plant: ControlPlantAdapter,
    *,
    family_id: str,
    adapter_id: str,
    physical_family: str,
    tier: FidelityTier,
    state_units: Mapping[str, str] | None = None,
    control_units: Mapping[str, str] | None = None,
    resource_channels: Sequence[AdapterChannel] = (),
    evidence_status: str = "development",
    validity_envelope: str = "",
    omitted_physics: Sequence[str] = (),
    control_realization_override: ControlRealization | None = None,
) -> FamilyAdapterDescriptor:
    """Construct a descriptor from an existing plant's ordered channels.

    New family adapters should declare only their stable identity and units;
    the helper derives the channel lists from the plant protocol so a second
    hand-written state or effector ordering cannot drift from runtime.
    """

    state_units = state_units or {}
    control_units = control_units or {}
    return FamilyAdapterDescriptor(
        family_id=family_id,
        adapter_id=adapter_id,
        physical_family=physical_family,
        tier=tier,
        state_channels=tuple(AdapterChannel(name, state_units.get(name, "unspecified"), "state") for name in plant.state_names),
        control_channels=tuple(AdapterChannel(name, control_units.get(name, "unspecified"), "effector") for name in plant.control_names),
        resource_channels=tuple(resource_channels),
        control_realization_override=control_realization_override,
        evidence_status=evidence_status,
        validity_envelope=validity_envelope,
        omitted_physics=tuple(omitted_physics),
    )
    ####
    ####


def descriptor_from_direct_wrench_state(
    *,
    family_id: str,
    adapter_id: str,
    physical_family: str,
    state_names: Sequence[str],
    state_units: Mapping[str, str],
    evidence_status: str = "development",
    validity_envelope: str = "",
    omitted_physics: Sequence[str] = (),
    state_frame: str = "body",
) -> FamilyAdapterDescriptor:
    """Construct the common descriptor for a local direct-wrench bridge.

    Direct-wrench adapters still own their nonlinear source loads, mass
    properties, and authority limits.  They should not, however, repeat the
    public six-axis channel vocabulary.  This helper keeps bridge metadata
    identical across families while preserving each adapter's state schema
    and claim boundary.
    """

    if not state_names:
        raise ValueError("direct-wrench descriptors require state channels")
    if any(name not in state_units or not state_units[name].strip() for name in state_names):
        raise ValueError("direct-wrench descriptors require a unit for every state channel")
    state_channels = tuple(
        AdapterChannel(name, state_units[name], "state", frame=state_frame) for name in state_names
    )
    control_channels = tuple(
        AdapterChannel(
            name,
            "N" if name.startswith("force_") else "N*m",
            "direct_wrench",
            frame="body",
            description="bounded generalized body wrench; not a physical effector",
        )
        for name in DIRECT_WRENCH_NAMES
    )
    return FamilyAdapterDescriptor(
        family_id=family_id,
        adapter_id=adapter_id,
        physical_family=physical_family,
        tier="rigid_body_6dof_direct_wrench",
        state_channels=state_channels,
        control_channels=control_channels,
        evidence_status=evidence_status,
        validity_envelope=validity_envelope,
        omitted_physics=tuple(omitted_physics),
    )
    ####
    ####


def validate_family_adapter(
    adapter: StandardFamilyAdapter,
    *,
    expected_family_id: str | None = None,
) -> AdapterConformanceReport:
    """Validate the common tier and capability contract before execution.

    This is intentionally independent of vehicle equations.  It catches the
    horizontal errors that otherwise make a direct-wrench screen look like a
    surface-controlled result, or make a passive body appear to have a
    controller merely because a generic field was omitted.
    """

    descriptor = adapter.describe()
    report = adapter.capability_report()
    findings: list[AdapterConformanceFinding] = []
    if expected_family_id is not None and descriptor.family_id != expected_family_id:
        findings.append(
            AdapterConformanceFinding(
                "error",
                "family-id-mismatch",
                f"adapter family {descriptor.family_id!r} does not match expected {expected_family_id!r}",
            )
        )
    allocation = report.capability("allocate")
    effectiveness = report.capability("effectiveness")
    realization = descriptor.control_realization
    if realization == "surface_allocated":
        if not descriptor.control_channels:
            findings.append(AdapterConformanceFinding("error", "surface-controls-missing", "surface allocation requires physical control channels"))
        if allocation.status != "available" or effectiveness.status != "available":
            findings.append(AdapterConformanceFinding("error", "surface-capability-missing", "surface allocation requires available effectiveness and allocation operations"))
    elif realization == "direct_wrench":
        if allocation.status == "available":
            findings.append(AdapterConformanceFinding("error", "direct-tier-promoted", "direct-wrench tier must not advertise physical allocation"))
    elif realization == "uncontrolled":
        if descriptor.control_channels:
            findings.append(AdapterConformanceFinding("error", "passive-controls-present", "uncontrolled family must not declare control channels"))
        if allocation.status != "not_applicable":
            findings.append(AdapterConformanceFinding("error", "passive-allocation-present", "uncontrolled family allocation must be not_applicable"))
    elif allocation.status == "available":
        findings.append(AdapterConformanceFinding("error", "lower-tier-allocation-present", f"{realization} tier must not claim physical allocation"))
    if report.capability("state_derivative").status == "available" and adapter.plant is None and adapter.state_derivative_provider is None:
        findings.append(AdapterConformanceFinding("error", "state-provider-missing", "available state_derivative capability has no provider"))
    if report.capability("resource_rates").status == "available" and adapter.resource_rate_provider is None:
        findings.append(AdapterConformanceFinding("error", "resource-provider-missing", "available resource_rates capability has no provider"))
    if report.capability("observe").status == "available" and adapter.observation_provider is None:
        findings.append(AdapterConformanceFinding("error", "observation-provider-missing", "available observe capability has no provider"))
    if report.capability("replay").status == "available" and adapter.replay_provider is None:
        findings.append(AdapterConformanceFinding("error", "replay-provider-missing", "available replay capability has no provider"))
    if report.capability("trim_fragment").status == "available" and adapter.trim_fragment_provider is None:
        findings.append(
            AdapterConformanceFinding(
                "error", "trim-fragment-provider-missing", "available trim_fragment capability has no provider"
            )
        )
    if set(item.operation for item in report.capabilities) != set(ADAPTER_OPERATIONS):
        findings.append(AdapterConformanceFinding("error", "capability-set-incomplete", "adapter capability report must declare every common operation"))
    status: Literal["pass", "fail"] = "fail" if any(item.severity == "error" for item in findings) else "pass"
    return AdapterConformanceReport(descriptor.family_id, descriptor.adapter_id, descriptor.tier, status, tuple(findings))
    ####


def _missing_capability(operation: AdapterOperation) -> AdapterCapability:
    return AdapterCapability(operation, "not_available", "operation was not declared by the family adapter")
    ####


def _require_unique_channels(channels: Sequence[AdapterChannel], role: str) -> None:
    names = [item.name for item in channels]
    if len(names) != len(set(names)):
        raise ValueError(f"{role} channel names must be unique")
    ####


def _channel_dict(channel: AdapterChannel) -> dict[str, str | None]:
    return {
        "name": channel.name,
        "unit": channel.unit,
        "role": channel.role,
        "frame": channel.frame,
        "description": channel.description,
    }
    ####


__all__ = [
    "AdapterCapability",
    "AdapterCapabilityError",
    "AdapterChannel",
    "ADAPTER_OPERATIONS",
    "AdapterConformanceFinding",
    "AdapterConformanceReport",
    "AdapterOperation",
    "CapabilityStatus",
    "FamilyAdapter",
    "FamilyAdapterDescriptor",
    "FamilyCapabilityReport",
    "AllocationProvider",
    "EffectivenessProvider",
    "StandardFamilyAdapter",
    "StateDerivativeProvider",
    "ReplayProvider",
    "TrimFragmentProvider",
    "TrimFragmentResult",
    "descriptor_from_control_plant",
    "descriptor_from_direct_wrench_state",
    "validate_family_adapter",
]
####
