"""Factory registry for the horizontal vehicle-adapter contract.

The YAML registry identifies the semantic adapter expected by a family.  This
module binds that identity to executable factories without making the
manifest import family-specific Python modules.  Registrations may be marked
development or planned; such entries remain visible as unavailable rather
than silently falling back to another family or tier.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Literal

from .family_adapter import AdapterConformanceReport, StandardFamilyAdapter, validate_family_adapter
from .family_adapter_probes import AdapterProbeCase, AdapterProbeReport, run_adapter_probe
from .fidelity_contracts import CANONICAL_FIDELITY_TIERS, FidelityTier

RegistrationStatus = Literal["available", "development", "planned", "not_applicable"]
RegistryCheckStatus = Literal["pass", "development", "blocked"]
FamilyAdapterFactory = Callable[[FidelityTier], StandardFamilyAdapter]
FamilyAdapterProbeFactory = Callable[[StandardFamilyAdapter], AdapterProbeCase]


class AdapterRegistrationError(RuntimeError):
    """Raised when a manifest adapter binding cannot produce its requested tier."""


@dataclass(frozen=True, slots=True)
class FamilyAdapterRegistration:
    """One stable family/adapter binding and optional executable factory."""

    family_id: str
    adapter_id: str
    status: RegistrationStatus
    factory: FamilyAdapterFactory | None = None
    probe_factory: FamilyAdapterProbeFactory | None = None
    supported_tiers: tuple[FidelityTier, ...] = CANONICAL_FIDELITY_TIERS
    note: str = ""

    def __post_init__(self) -> None:
        if not self.family_id.strip() or not self.adapter_id.strip():
            raise ValueError("adapter registration requires family and adapter IDs")
        if not self.supported_tiers:
            raise ValueError("adapter registration requires at least one supported tier")
        if any(tier not in CANONICAL_FIDELITY_TIERS for tier in self.supported_tiers):
            raise ValueError("adapter registration contains an unknown canonical tier")
        if len(set(self.supported_tiers)) != len(self.supported_tiers):
            raise ValueError("adapter registration tiers must be unique")
        if self.status == "available" and self.factory is None:
            raise ValueError("available adapter registration requires a factory")
        if self.factory is not None and self.status == "not_applicable":
            raise ValueError("not_applicable adapter registration cannot have a factory")
        ####
    ####

    def build(self, tier: FidelityTier) -> StandardFamilyAdapter:
        """Build one requested tier or raise an actionable registration error."""

        if tier not in self.supported_tiers:
            raise AdapterRegistrationError(f"{self.family_id}: tier {tier!r} is not registered for {self.adapter_id}")
        if self.factory is None:
            raise AdapterRegistrationError(f"{self.family_id}: {self.adapter_id} is {self.status}: {self.note or 'no factory is bound'}")
        adapter = self.factory(tier)
        descriptor = adapter.describe()
        if descriptor.family_id != self.family_id:
            raise AdapterRegistrationError(
                f"{self.family_id}: factory returned family {descriptor.family_id!r}"
            )
        if descriptor.adapter_id != self.adapter_id:
            raise AdapterRegistrationError(
                f"{self.family_id}: factory returned adapter {descriptor.adapter_id!r}, expected {self.adapter_id!r}"
            )
        if descriptor.tier != tier:
            raise AdapterRegistrationError(
                f"{self.family_id}: factory returned tier {descriptor.tier!r}, expected {tier!r}"
            )
        return adapter
        ####
    ####


@dataclass(frozen=True, slots=True)
class AdapterRegistrationCheck:
    """Machine-readable result for one registered family/tier."""

    family_id: str
    adapter_id: str
    tier: FidelityTier
    status: RegistryCheckStatus
    conformance: AdapterConformanceReport | None
    probe: AdapterProbeReport | None
    message: str

    def as_dict(self) -> dict[str, object]:
        """Return a stable registration-check artifact."""

        return {
            "family_id": self.family_id,
            "adapter_id": self.adapter_id,
            "tier": self.tier,
            "status": self.status,
            "message": self.message,
            "conformance": self.conformance.as_dict() if self.conformance is not None else None,
            "probe": self.probe.as_dict() if self.probe is not None else None,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class AdapterRegistryReport:
    """Result of checking all registered factory bindings."""

    checks: tuple[AdapterRegistrationCheck, ...]

    @property
    def status(self) -> RegistryCheckStatus:
        """Return the strictest status across all checks."""

        if any(item.status == "blocked" for item in self.checks):
            return "blocked"
        if any(item.status == "development" for item in self.checks):
            return "development"
        return "pass"
        ####

    def as_dict(self) -> dict[str, object]:
        """Return the full registry-check artifact."""

        return {
            "schema": "taoryx.family-adapter-registry/v1alpha1",
            "status": self.status,
            "checks": [item.as_dict() for item in self.checks],
        }
        ####

    def operation_status_by_family_tier(self) -> dict[str, dict[FidelityTier, dict[str, str]]]:
        """Return passing/blocked operation probes keyed for fidelity lowering.

        A capability declaration alone is not enough to authorize a tier.  The
        lowering resolver consumes these probe results so a profile with valid
        evidence cannot bypass a failed trim, linearization, or allocation
        operation.
        """

        result: dict[str, dict[FidelityTier, dict[str, str]]] = {}
        for check in self.checks:
            if check.probe is None:
                continue
            result.setdefault(check.family_id, {})[check.tier] = {
                item.operation: item.status for item in check.probe.operations
            }
        return result
        ####
    ####


class FamilyAdapterRegistry:
    """In-memory binding registry used by runtime and qualification tools."""

    def __init__(self, registrations: tuple[FamilyAdapterRegistration, ...] = ()) -> None:
        self._registrations: dict[str, FamilyAdapterRegistration] = {}
        for registration in registrations:
            self.register(registration)
        ####

    def register(self, registration: FamilyAdapterRegistration) -> None:
        """Register one family, rejecting duplicate identity."""

        if registration.family_id in self._registrations:
            raise ValueError(f"adapter registration already exists for {registration.family_id!r}")
        self._registrations[registration.family_id] = registration
        ####

    def registration(self, family_id: str) -> FamilyAdapterRegistration:
        """Return one registration or raise a diagnostic-friendly error."""

        try:
            return self._registrations[family_id]
        except KeyError as error:
            raise KeyError(f"no adapter registration exists for {family_id!r}") from error
        ####

    def build(self, family_id: str, tier: FidelityTier) -> StandardFamilyAdapter:
        """Build one family/tier through its declared factory."""

        return self.registration(family_id).build(tier)
        ####

    def check(self, family_id: str, tier: FidelityTier) -> AdapterRegistrationCheck:
        """Build and conformance-check one family/tier binding."""

        registration = self.registration(family_id)
        if registration.status in {"planned", "not_applicable"} or registration.factory is None:
            return AdapterRegistrationCheck(
                family_id,
                registration.adapter_id,
                tier,
                "development" if registration.status == "planned" else "development",
                None,
                None,
                registration.note or f"adapter is {registration.status}",
            )
        try:
            adapter = registration.build(tier)
        except Exception as error:  # noqa: BLE001 - registry must return structured diagnostics
            return AdapterRegistrationCheck(family_id, registration.adapter_id, tier, "blocked", None, None, str(error))
        conformance = validate_family_adapter(adapter, expected_family_id=family_id)
        status: RegistryCheckStatus = "pass" if conformance.status == "pass" else "blocked"
        probe: AdapterProbeReport | None = None
        if status == "pass" and registration.probe_factory is not None:
            try:
                probe = run_adapter_probe(adapter, registration.probe_factory(adapter))
            except Exception as error:  # noqa: BLE001 - registry must return structured diagnostics
                return AdapterRegistrationCheck(family_id, registration.adapter_id, tier, "blocked", conformance, None, str(error))
            if probe.status == "blocked":
                status = "blocked"
        message = "conformance and operation probes passed" if probe is not None and status == "pass" else (
            "conformance passed" if status == "pass" else "conformance or operation probe failed"
        )
        return AdapterRegistrationCheck(family_id, registration.adapter_id, tier, status, conformance, probe, message)
        ####

    def check_all(self, tiers: Mapping[str, FidelityTier] | None = None) -> AdapterRegistryReport:
        """Check each registration at one declared or default tier."""

        checks: list[AdapterRegistrationCheck] = []
        for family_id, registration in self._registrations.items():
            tier = (tiers or {}).get(family_id, registration.supported_tiers[-1])
            checks.append(self.check(family_id, tier))
        return AdapterRegistryReport(tuple(checks))
        ####

    def check_matrix(self) -> AdapterRegistryReport:
        """Check every tier explicitly claimed by available registrations.

        ``check_all`` remains the compact one-witness report used by callers
        that need one representative result per family.  The matrix is the
        stricter integration contract: a factory is invoked once for every
        tier it claims to support, so direct-wrench and surface-allocation
        evidence cannot be conflated by a representative default-tier check.
        """

        checks = [
            self.check(family_id, tier)
            for family_id, registration in self._registrations.items()
            if registration.status == "available"
            for tier in registration.supported_tiers
        ]
        return AdapterRegistryReport(tuple(checks))
        ####

    @property
    def registrations(self) -> tuple[FamilyAdapterRegistration, ...]:
        """Return registrations in insertion order."""

        return tuple(self._registrations.values())
        ####


__all__ = [
    "AdapterRegistrationCheck",
    "AdapterRegistrationError",
    "AdapterRegistryReport",
    "FamilyAdapterFactory",
    "FamilyAdapterProbeFactory",
    "FamilyAdapterRegistration",
    "FamilyAdapterRegistry",
    "RegistrationStatus",
]
####
