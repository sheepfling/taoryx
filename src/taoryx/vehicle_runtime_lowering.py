"""Fail-closed binding of compiled vehicle compositions to native adapters.

Semantic composition deliberately knows no plant equations.  This module is
the next boundary: it builds only the adapter registered for the selected
family and canonical fidelity.  A successful binding is *not* a mission run;
an adapter-specific segment translator must still lower the semantic segments
to controller, trim, and runtime requests.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from .family_adapter import AdapterCapabilityError, StandardFamilyAdapter, validate_family_adapter
from .family_adapter_registry import AdapterRegistrationError, FamilyAdapterRegistration, FamilyAdapterRegistry
from .fidelity_contracts import FidelityTier
from .hl20_adapter import build_hl20_source_adapter
from .nesc_adapter import build_nesc_replay_adapter
from .vehicle_composition import CompiledVehicleComposition
from .x15_adapter import build_x15_source_direct_wrench_adapter

RuntimeLoweringStatus = Literal["adapter_bound", "blocked"]


@dataclass(frozen=True, slots=True)
class RuntimeLoweringResult:
    """Result of binding one semantic composition to its declared adapter."""

    composition_id: str
    composition_identity_sha256: str
    family_id: str
    fidelity: FidelityTier
    status: RuntimeLoweringStatus
    adapter_descriptor: dict[str, object] | None
    adapter_capabilities: dict[str, object] | None
    required_segment_intents: tuple[str, ...]
    translator_status: Literal["pending", "not_applicable"]
    diagnostics: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        """Return a stable machine-readable lowering report."""

        return {
            "schema": "taoryx.vehicle-runtime-lowering/v1alpha1",
            "composition_id": self.composition_id,
            "composition_identity_sha256": self.composition_identity_sha256,
            "family_id": self.family_id,
            "fidelity": self.fidelity,
            "status": self.status,
            "adapter_descriptor": self.adapter_descriptor,
            "adapter_capabilities": self.adapter_capabilities,
            "required_segment_intents": list(self.required_segment_intents),
            "translator_status": self.translator_status,
            "diagnostics": list(self.diagnostics),
            "claim_boundary": (
                "adapter_bound means the selected native adapter was constructed and conformance-checked. "
                "It does not mean semantic segments were translated, a controller was run, or a vehicle was qualified."
            ),
        }
        ####
    ####


def lower_vehicle_composition(
    composition: CompiledVehicleComposition,
    *,
    adapters: FamilyAdapterRegistry | None = None,
) -> RuntimeLoweringResult:
    """Build only the adapter named by the composition's native handoff.

    There is intentionally no fallback by physical family, nominal vehicle, or
    control realization.  A missing factory is a structured blocked result so
    callers cannot accidentally replace a source table, rotor model, or
    passive body with a generic direct-force implementation.
    """

    registry = adapters or build_vehicle_runtime_adapter_registry()
    requested_adapter = str(composition.native_adapter_handoff["adapter_id"])
    required_intents = tuple(sorted({intent for segment in composition.segments for intent in segment.required_control_intents}))
    try:
        registration = registry.registration(composition.family_id)
        if registration.adapter_id != requested_adapter:
            return _blocked(
                composition,
                required_intents,
                f"registry adapter {registration.adapter_id!r} does not match composed adapter {requested_adapter!r}",
            )
        adapter = registry.build(composition.family_id, composition.fidelity)
        return _bound(composition, adapter, required_intents)
    except (AdapterRegistrationError, AdapterCapabilityError, KeyError, ValueError) as error:
        return _blocked(composition, required_intents, str(error))
    ####


def build_vehicle_runtime_adapter_registry() -> FamilyAdapterRegistry:
    """Return the central, explicitly scoped runtime adapter registry.

    The source-backed X-15, HL-20, NESC, and passive-body adapters are safe to
    construct directly from ``src``.  Other current witnesses still live in
    qualification tools or lack a mission translator; they remain explicitly
    development registrations rather than being imported through tool scripts
    or silently represented by a different family.
    """

    registrations = (
        FamilyAdapterRegistration(
            "x15",
            "taoryx.high_energy.fixed_wing.v1",
            "available",
            build_x15_source_direct_wrench_adapter,
            supported_tiers=("rigid_body_6dof_direct_wrench",),
            note="local source direct-wrench bridge only; mission translation remains pending",
        ),
        FamilyAdapterRegistration(
            "hl20_mod_k",
            "taoryx.lifting_body.daveml.v1",
            "available",
            build_hl20_source_adapter,
            supported_tiers=("rigid_body_6dof_direct_wrench", "rigid_body_6dof_surface_allocated"),
            note="local source load/surface witnesses; mission translation remains pending",
        ),
        FamilyAdapterRegistration(
            "reference_nesc_two_stage_rocket",
            "taoryx.rocket.variable_mass_nesc.v1",
            "available",
            build_nesc_replay_adapter,
            supported_tiers=("point_mass_3dof", "pseudo_6dof"),
            note="source replay adapter; active segment translation remains pending",
        ),
        FamilyAdapterRegistration(
            "tumbling_body",
            "taoryx.passive_body.rigid_aero.v1",
            "development",
            supported_tiers=("pseudo_6dof",),
            note="passive-body runtime adapter factory has not yet moved from the qualification harness",
        ),
        FamilyAdapterRegistration(
            "skywalker_x8",
            "taoryx.fixed_wing.source_table.v1",
            "development",
            note="source-table witness factory remains in qualification tooling pending runtime extraction",
        ),
        FamilyAdapterRegistration(
            "b747",
            "taoryx.fixed_wing.source_table.v1",
            "development",
            note="source-table witness factory remains in qualification tooling pending runtime extraction",
        ),
        FamilyAdapterRegistration(
            "a320_openap_3dof",
            "taoryx.fixed_wing.openap.v1",
            "development",
            note="OpenAP reduced adapter needs a semantic-segment runtime binding",
        ),
        FamilyAdapterRegistration(
            "f16_s119",
            "taoryx.fixed_wing.daveml.v1",
            "development",
            note="source/reduced witness factories remain in qualification tooling pending runtime extraction",
        ),
        FamilyAdapterRegistration(
            "hummingbird",
            "taoryx.multirotor.native_quad_x.v1",
            "development",
            note="native rotor witness factory remains in qualification tooling pending runtime extraction",
        ),
    )
    return FamilyAdapterRegistry(registrations)
    ####


def _bound(
    composition: CompiledVehicleComposition,
    adapter: StandardFamilyAdapter,
    required_intents: tuple[str, ...],
) -> RuntimeLoweringResult:
    """Return a native adapter binding after strict common conformance checks."""

    conformance = validate_family_adapter(adapter, expected_family_id=composition.family_id)
    if conformance.status != "pass":
        findings = "; ".join(f"{item.code}: {item.message}" for item in conformance.findings)
        return _blocked(composition, required_intents, f"adapter conformance failed: {findings}")
    return RuntimeLoweringResult(
        composition.id,
        composition.identity_sha256,
        composition.family_id,
        composition.fidelity,
        "adapter_bound",
        adapter.describe().as_dict(),
        adapter.capability_report().as_dict(),
        required_intents,
        "pending",
        (
            "native adapter constructed and common conformance checks passed",
            "semantic segment-to-controller translation remains an explicit next lowering stage",
        ),
    )
    ####


def _blocked(
    composition: CompiledVehicleComposition,
    required_intents: tuple[str, ...],
    diagnostic: str,
) -> RuntimeLoweringResult:
    """Return a visible failure-to-lower result without a surrogate fallback."""

    return RuntimeLoweringResult(
        composition.id,
        composition.identity_sha256,
        composition.family_id,
        composition.fidelity,
        "blocked",
        None,
        None,
        required_intents,
        "pending",
        (diagnostic,),
    )
    ####


__all__ = [
    "RuntimeLoweringResult",
    "RuntimeLoweringStatus",
    "build_vehicle_runtime_adapter_registry",
    "lower_vehicle_composition",
]
