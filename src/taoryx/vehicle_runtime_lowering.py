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

from .family_adapter import (
    AdapterCapabilityError,
    StandardFamilyAdapter,
    validate_family_adapter,
)
from .family_adapter_registry import AdapterRegistrationError, FamilyAdapterRegistry
from .fidelity_contracts import FidelityTier
from .plugins import PluginCatalog, current_plugin_catalog, discover_plugins
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_composition_registry import mission_graph_execution_contract
from .vehicle_execution_bindings import VehicleExecutionBindingError, resolve_vehicle_execution_binding
from .vehicle_execution_preflight import VehicleExecutionPreflight, preflight_vehicle_composition

RuntimeLoweringStatus = Literal["adapter_bound", "factory_bound", "blocked"]


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
    execution_binding: dict[str, object] | None
    required_segment_intents: tuple[str, ...]
    translator_status: Literal["translation_ready", "pending", "not_applicable"]
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
            "execution_binding": self.execution_binding,
            "required_segment_intents": list(self.required_segment_intents),
            "translator_status": self.translator_status,
            "diagnostics": list(self.diagnostics),
            "claim_boundary": (
                "adapter_bound means the selected native adapter was constructed and conformance-checked. "
                "factory_bound means an exact source-owned batch factory and semantic translator are both declared, "
                "but no common adapter was constructed. Neither status means a controller was run or a vehicle was qualified."
            ),
        }
        ####
    ####


def lower_vehicle_composition(
    composition: CompiledVehicleComposition,
    *,
    adapters: FamilyAdapterRegistry | None = None,
    preflight_result: VehicleExecutionPreflight | None = None,
    plugins: PluginCatalog | None = None,
) -> RuntimeLoweringResult:
    """Build only the adapter named by the composition's native handoff.

    There is intentionally no fallback by physical family, nominal vehicle, or
    control realization.  A missing factory is a structured blocked result so
    callers cannot accidentally replace a source table, rotor model, or
    passive body with a generic direct-force implementation.
    """

    registry = adapters or build_vehicle_runtime_adapter_registry(plugins=plugins)
    requested_adapter = str(composition.native_adapter_handoff["adapter_id"])
    required_intents = tuple(sorted({intent for segment in composition.segments for intent in segment.required_control_intents}))
    if (
        preflight_result is not None
        and preflight_result.composition_identity_sha256 != composition.identity_sha256
    ):
        return _blocked(
            composition,
            required_intents,
            "supplied preflight result does not belong to this immutable composition",
        )
    graph_contract = mission_graph_execution_contract(
        composition.family_id,
        composition.mission,
        composition.fidelity,
    )
    supports_family_graph_extension = graph_contract["status"] == "family_extension_declared"
    if (
        composition.mission_graph is not None
        and composition.mission_graph.status not in {"linear_sequence_only", "authored_linear_sequence_lowered"}
        and not supports_family_graph_extension
    ):
        return _blocked(
            composition,
            required_intents,
            "caller-authored mission graph is semantically valid but no selected native adapter has declared graph execution support",
        )
    try:
        registration = registry.registration(composition.family_id)
        if registration.adapter_id != requested_adapter:
            return _blocked(
                composition,
                required_intents,
                f"registry adapter {registration.adapter_id!r} does not match composed adapter {requested_adapter!r}",
            )
        adapter = registry.build(composition.family_id, composition.fidelity)
        preflight = preflight_result or preflight_vehicle_composition(composition, plugins=plugins)
        if preflight.status == "blocked":
            failed = ", ".join(check.id for check in preflight.checks if not check.passed)
            return _blocked(
                composition,
                required_intents,
                "semantic preflight failed before native adapter execution" + (f": {failed}" if failed else ""),
            )
        return _bound(composition, adapter, required_intents, preflight_result=preflight)
    except (AdapterRegistrationError, AdapterCapabilityError, KeyError, ValueError) as error:
        adapter_diagnostic = str(error)
    return _resolve_declared_batch_factory(
        composition,
        required_intents,
        adapter_diagnostic,
        preflight_result=preflight_result,
    )
    ####


def build_vehicle_runtime_adapter_registry(
    *,
    plugins: PluginCatalog | None = None,
    include_external_plugins: bool = True,
) -> FamilyAdapterRegistry:
    """Build the runtime adapter registry from compatible plug-in contributions.

    Discovery registers factories and metadata only; no plant is constructed
    here. Installed model packages preserve every current ID and can add new
    families through the same fail-closed registry contract.
    """

    catalog = plugins if plugins is not None else current_plugin_catalog()
    if catalog is None:
        catalog = discover_plugins(include_external=include_external_plugins)
    return catalog.build_family_adapter_registry()
    ####


def _bound(
    composition: CompiledVehicleComposition,
    adapter: StandardFamilyAdapter,
    required_intents: tuple[str, ...],
    *,
    preflight_result: VehicleExecutionPreflight,
) -> RuntimeLoweringResult:
    """Return a native binding and an exact factory only after preflight.

    Constructing a common adapter and lowering semantic segments are distinct
    facts.  When the same immutable composition has both a ready translator
    and a declared batch factory, report them together.  Otherwise retain a
    visible ``pending`` translator state rather than treating adapter
    construction as executable mission support.
    """

    conformance = validate_family_adapter(adapter, expected_family_id=composition.family_id)
    if conformance.status != "pass":
        findings = "; ".join(f"{item.code}: {item.message}" for item in conformance.findings)
        return _blocked(composition, required_intents, f"adapter conformance failed: {findings}")
    execution_binding: dict[str, object] | None = None
    translator_status: Literal["translation_ready", "pending", "not_applicable"] = "pending"
    diagnostics = [
        "native adapter constructed and common conformance checks passed",
    ]
    if preflight_result.status == "translation_ready":
        try:
            binding = resolve_vehicle_execution_binding(composition, "batch")
        except VehicleExecutionBindingError as error:
            diagnostics.append(
                "semantic preflight passed but no exact runnable batch factory is declared: "
                f"{error.reason}"
            )
        else:
            execution_binding = binding.model_dump(mode="json")
            translator_status = "translation_ready"
            diagnostics.extend(
                (
                    "exact semantic mission translator passed preflight",
                    f"declared source-owned batch factory {binding.factory_id!r} is available",
                )
            )
    elif preflight_result.status == "not_applicable":
        translator_status = "not_applicable"
        diagnostics.append("no semantic segment translator is declared for this composition")
    else:
        diagnostics.append("semantic segment-to-controller translation remains an explicit next lowering stage")
    return RuntimeLoweringResult(
        composition.id,
        composition.identity_sha256,
        composition.family_id,
        composition.fidelity,
        "adapter_bound",
        adapter.describe().as_dict(),
        adapter.capability_report().as_dict(),
        execution_binding,
        required_intents,
        translator_status,
        tuple(diagnostics),
    )
    ####


def _resolve_declared_batch_factory(
    composition: CompiledVehicleComposition,
    required_intents: tuple[str, ...],
    adapter_diagnostic: str,
    *,
    preflight_result: VehicleExecutionPreflight | None,
) -> RuntimeLoweringResult:
    """Bind an exact source-owned batch factory when no common adapter exists.

    Several current witnesses execute through an established source-owned
    factory rather than a :class:`StandardFamilyAdapter`.  Treating them as
    blocked after their exact semantic route has preflighted successfully
    makes discovery disagree with execution.  This branch is intentionally
    narrower than adapter binding: it checks the *exact* batch binding and
    translator declaration but does not construct a substitute plant.
    """

    preflight = preflight_result or preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        return _blocked(
            composition,
            required_intents,
            adapter_diagnostic,
            translator_status="not_applicable" if preflight.status == "not_applicable" else "pending",
        )
    try:
        binding = resolve_vehicle_execution_binding(composition, "batch")
    except VehicleExecutionBindingError as error:
        return _blocked(composition, required_intents, f"{adapter_diagnostic}; {error}")
    return RuntimeLoweringResult(
        composition.id,
        composition.identity_sha256,
        composition.family_id,
        composition.fidelity,
        "factory_bound",
        None,
        None,
        binding.model_dump(mode="json"),
        required_intents,
        "translation_ready",
        (
            "exact semantic mission translator passed preflight",
            f"declared source-owned batch factory {binding.factory_id!r} is available",
            "no StandardFamilyAdapter was constructed; this is not generic plant-adapter conformance",
        ),
    )
    ####


def _blocked(
    composition: CompiledVehicleComposition,
    required_intents: tuple[str, ...],
    diagnostic: str,
    *,
    translator_status: Literal["pending", "not_applicable"] = "pending",
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
        None,
        required_intents,
        translator_status,
        (diagnostic,),
    )
    ####


__all__ = [
    "RuntimeLoweringResult",
    "RuntimeLoweringStatus",
    "build_vehicle_runtime_adapter_registry",
    "lower_vehicle_composition",
]
