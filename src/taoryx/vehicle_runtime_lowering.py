"""Fail-closed binding of compiled vehicle compositions to native adapters.

Semantic composition deliberately knows no plant equations.  This module is
the next boundary: it builds only the adapter registered for the selected
family and canonical fidelity.  A successful binding is *not* a mission run;
an adapter-specific segment translator must still lower the semantic segments
to controller, trim, and runtime requests.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from functools import lru_cache
from typing import Literal

from .family_adapter import (
    AdapterCapabilityError,
    StandardFamilyAdapter,
    descriptor_from_control_plant,
    validate_family_adapter,
)
from .family_adapter_probes import AdapterProbeCase
from .family_adapter_registry import AdapterRegistrationError, FamilyAdapterRegistration, FamilyAdapterRegistry
from .fidelity_contracts import FidelityTier
from .hl20_adapter import build_hl20_source_adapter
from .nesc_adapter import build_nesc_replay_adapter
from .source_f16 import build_f16_source_physical_plant
from .source_table_fixed_wing import (
    build_b747_condition3_source_table_plant,
    build_x8_source_table_plant,
)
from .source_table_multirotor import build_hummingbird_individual_rotor_source_table_plant
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_composition_registry import mission_graph_execution_contract
from .vehicle_execution_bindings import VehicleExecutionBindingError, resolve_vehicle_execution_binding
from .vehicle_execution_preflight import VehicleExecutionPreflight, preflight_vehicle_composition
from .x15_adapter import build_x15_source_direct_wrench_adapter

RuntimeLoweringStatus = Literal["adapter_bound", "factory_bound", "blocked"]


def _source_local_probe(adapter: StandardFamilyAdapter) -> AdapterProbeCase:
    """Return the plant-owned source operating point for generic probes.

    This is deliberately structural: the source plant itself owns its trim
    state and effectors, while the generic adapter framework only verifies
    that every declared operation can be exercised at that exact point.
    """

    plant = adapter.plant
    if plant is None or not hasattr(plant, "source_local_state") or not hasattr(plant, "source_effectors"):
        raise ValueError(f"{adapter.describe().family_id}: plant does not expose a source local operating point")
    state = dict(getattr(plant, "source_local_state"))
    effectors = dict(getattr(plant, "source_effectors"))
    return AdapterProbeCase(
        state=state,
        effectors=effectors,
        trim_target=state,
        trim_initial_guess=effectors,
        previous_effectors=effectors,
    )
    ####


def _source_or_trim_probe(adapter: StandardFamilyAdapter) -> AdapterProbeCase:
    """Probe a source-local plant or a source-owned resolved trim point."""

    plant = adapter.plant
    if plant is None:
        raise ValueError(f"{adapter.describe().family_id}: adapter has no plant")
    if hasattr(plant, "source_local_state") and hasattr(plant, "source_effectors"):
        return _source_local_probe(adapter)
    if not hasattr(plant, "trim_result"):
        raise ValueError(f"{adapter.describe().family_id}: plant has no source operating point")
    trim = getattr(plant, "trim_result")
    state = dict(trim.state)
    effectors = dict(trim.controls)
    environment: dict[str, float | str] = {}
    for name in ("altitude_m", "trim_pitch_rad"):
        if hasattr(plant, name):
            environment[name] = float(getattr(plant, name))
    return AdapterProbeCase(
        state=state,
        effectors=effectors,
        environment=environment,
        trim_target=state,
        trim_initial_guess=effectors,
        previous_effectors=effectors,
    )
    ####


def _source_table_fixed_wing_factory(
    builder: Callable[[], object],
    *,
    family_id: str,
    adapter_id: str = "taoryx.fixed_wing.source_table.v1",
    physical_family: str = "powered_fixed_wing",
    effector_attribute: str = "effector_limits",
    omitted_physics: tuple[str, ...] = (
        "family-specific mission and resource providers",
        "gain-scheduled or envelope-wide closed-loop validation",
    ),
) -> Callable[[FidelityTier], StandardFamilyAdapter]:
    """Bind one pinned source-table plant without a family-level fallback."""

    @lru_cache(maxsize=1)
    def plant() -> object:
        return builder()
        ####

    @lru_cache(maxsize=None)
    def build(tier: FidelityTier) -> StandardFamilyAdapter:
        source_plant = plant()
        control_names = getattr(source_plant, "control_names", ())
        limits = getattr(source_plant, effector_attribute, None)
        if not control_names or not isinstance(limits, dict):
            raise ValueError(f"{family_id}: source-table plant has no declared control limits")
        descriptor = descriptor_from_control_plant(
            source_plant,  # type: ignore[arg-type]
            family_id=family_id,
            adapter_id=adapter_id,
            physical_family=physical_family,
            tier=tier,
            control_units={name: limits[name].unit for name in control_names},
            evidence_status="development",
            omitted_physics=omitted_physics,
        )
        return StandardFamilyAdapter.from_control_plant(descriptor, source_plant)  # type: ignore[arg-type]
        ####

    return build
    ####


def _source_table_multirotor_factory(
    builder: Callable[[], object],
    *,
    family_id: str,
    adapter_id: str,
) -> Callable[[FidelityTier], StandardFamilyAdapter]:
    """Bind one pinned multirotor source plant without family fallback."""

    @lru_cache(maxsize=1)
    def plant() -> object:
        return builder()
        ####

    @lru_cache(maxsize=None)
    def build(tier: FidelityTier) -> StandardFamilyAdapter:
        source_plant = plant()
        control_names = getattr(source_plant, "control_names", ())
        limits = getattr(source_plant, "effector_limits", None)
        if not control_names or not isinstance(limits, dict):
            raise ValueError(f"{family_id}: source-table plant has no declared control limits")
        descriptor = descriptor_from_control_plant(
            source_plant,  # type: ignore[arg-type]
            family_id=family_id,
            adapter_id=adapter_id,
            physical_family="multirotor",
            tier=tier,
            control_units={name: limits[name].unit for name in control_names},
            evidence_status="development",
            omitted_physics=(
                "mission and position-control providers",
                "battery and voltage resource model",
                "blade-resolved and dynamic-inflow rotor physics",
            ),
        )
        return StandardFamilyAdapter.from_control_plant(descriptor, source_plant)  # type: ignore[arg-type]
        ####

    return build
    ####


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
        preflight = preflight_result or preflight_vehicle_composition(composition)
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


def build_vehicle_runtime_adapter_registry() -> FamilyAdapterRegistry:
    """Return the central, explicitly scoped runtime adapter registry.

    The source-backed X8, B747, Hummingbird, F-16, X-15, HL-20, and NESC
    adapters are safe to construct directly from ``src``. Other current
    witnesses still live in qualification tools or lack a mission translator;
    they remain explicitly development registrations rather than being
    imported through tool scripts or silently represented by a different
    family.
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
            "available",
            _source_table_fixed_wing_factory(build_x8_source_table_plant, family_id="skywalker_x8"),
            probe_factory=_source_local_probe,
            supported_tiers=("rigid_body_6dof_direct_wrench", "rigid_body_6dof_surface_allocated"),
            note="pinned source-table local plant; semantic mission translation remains a separate gate",
        ),
        FamilyAdapterRegistration(
            "b747",
            "taoryx.fixed_wing.source_table.v1",
            "available",
            _source_table_fixed_wing_factory(build_b747_condition3_source_table_plant, family_id="b747"),
            probe_factory=_source_local_probe,
            supported_tiers=("rigid_body_6dof_direct_wrench", "rigid_body_6dof_surface_allocated"),
            note="pinned NASA condition-3 source-table plant; semantic mission translation remains a separate gate",
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
            "available",
            _source_table_fixed_wing_factory(
                build_f16_source_physical_plant,
                family_id="f16_s119",
                adapter_id="taoryx.fixed_wing.daveml.v1",
                effector_attribute="effectors",
                omitted_physics=(
                    "mission translation and gain scheduling",
                    "full-flight-envelope and release qualification",
                ),
            ),
            probe_factory=_source_or_trim_probe,
            supported_tiers=("rigid_body_6dof_direct_wrench", "rigid_body_6dof_surface_allocated"),
            note="runtime-owned source-backed first operating point; mission and schedule gates remain separate",
        ),
        FamilyAdapterRegistration(
            "hummingbird",
            "taoryx.multirotor.native_quad_x.v1",
            "available",
            _source_table_multirotor_factory(
                build_hummingbird_individual_rotor_source_table_plant,
                family_id="hummingbird",
                adapter_id="taoryx.multirotor.native_quad_x.v1",
            ),
            probe_factory=_source_local_probe,
            supported_tiers=("rigid_body_6dof_direct_wrench", "rigid_body_6dof_surface_allocated"),
            note="runtime-owned individual-rotor local plant; mission and resource providers remain separate gates",
        ),
    )
    return FamilyAdapterRegistry(registrations)
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
