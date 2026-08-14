"""Plug-in registration for the source-backed X-15 research family."""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING, Any

from taoryx.controller_tuning_registry import ControllerTuningCampaignRegistration
from taoryx.local_direct_wrench_screen_registry import LocalDirectWrenchScreenCapabilityAdapter
from taoryx.plugins import PluginDefinition, PluginMetadata, PluginRegistrar, VehicleCatalogFragment

from .resources import model_resource_root

if TYPE_CHECKING:
    from taoryx.family_adapter import StandardFamilyAdapter
    from taoryx.family_adapter_probes import AdapterProbeCase
    from taoryx.family_adapter_registry import FamilyAdapterRegistration
    from taoryx.trajectory.catalog_mission_composition import CatalogMissionCompositionProvider
    from taoryx.tuning_campaign import TuningCampaign
    from taoryx.vehicle_batch_execution import VehicleBatchExecutionRequest
    from taoryx.vehicle_composition import CompiledVehicleComposition

_FAMILY_ID = "x15"
_PROVIDER_ID = "taoryx.x15.mission-composition"
_AGGREGATE_PROVIDER_ID = "taoryx.registry.mission-composition"
_PACKAGE_VERSION = "0.1.0a0"
_DIRECT_LQR_TRANSLATOR_ID = "taoryx.x15_local_direct_wrench_screen.capability.v1"
_DIRECT_LQI_TRANSLATOR_ID = "taoryx.x15_local_direct_wrench_lqi_screen.capability.v1"
_SURFACE_AUTHORITY_TRANSLATOR_ID = "taoryx.x15_source_surface_authority_screen.capability.v1"
_SURFACE_LQI_TRANSLATOR_ID = "taoryx.x15_source_surface_attitude_rate_lqi_screen.capability.v1"
_CATALOG_FRAGMENT = VehicleCatalogFragment(
    id="taoryx.x15.vehicle-catalog",
    resource_package="taoryx_x15",
    family_ids=(_FAMILY_ID,),
)
_FOCUSED_MISSION_TEMPLATE_IDS = (
    "x15_source_surface_authority_screen_v1",
    "x15_source_surface_attitude_rate_lqi_screen_v1",
    "x15_local_direct_wrench_screen_v1",
    "x15_local_direct_wrench_lqi_screen_v1",
)


class _LazyMissionCapabilityAdapter:
    """Expose one X-15 capability identity without building its source plant."""

    def __init__(self, identifier: str, factory: Callable[[], object]) -> None:
        self.id = identifier
        self._factory = factory
        self._delegate: object | None = None
        ####

    def _resolve(self) -> Any:
        if self._delegate is None:
            candidate = self._factory()
            if getattr(candidate, "id", None) != self.id:
                raise ValueError(f"lazy X-15 capability adapter {self.id!r} resolved a mismatched identity")
            self._delegate = candidate
        return self._delegate
        ####

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return bool(self._resolve().supports(composition))
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> Any:
        return self._resolve().estimate(composition)
        ####

    ####


def _family_adapter_registration() -> FamilyAdapterRegistration:
    """Return the X-15 direct-wrench adapter registration without loading data."""

    from taoryx.x15_adapter import build_x15_source_direct_wrench_adapter

    from taoryx.family_adapter_registry import FamilyAdapterRegistration

    return FamilyAdapterRegistration(
        _FAMILY_ID,
        "taoryx.high_energy.fixed_wing.v1",
        "available",
        build_x15_source_direct_wrench_adapter,
        probe_factory=_x15_source_probe,
        supported_tiers=("rigid_body_6dof_direct_wrench",),
        note="local source direct-wrench bridge only; mission translation remains pending",
    )
    ####


def _x15_source_probe(adapter: StandardFamilyAdapter) -> AdapterProbeCase:
    """Return the pinned release/glide state for X-15 generic adapter probes."""

    from taoryx.x15_adapter import build_x15_source_direct_wrench_plant

    from taoryx.family_adapter_probes import AdapterProbeCase

    plant = build_x15_source_direct_wrench_plant()
    effectors = {name: 0.0 for name in adapter.control_names}
    return AdapterProbeCase(
        state=dict(plant.reference_state),
        effectors=effectors,
        trim_target=dict(plant.reference_state),
        trim_initial_guess=effectors,
        previous_effectors=effectors,
    )
    ####


def _direct_wrench_tuning_adapter() -> StandardFamilyAdapter:
    """Build the X-15 source direct-wrench bridge for the common tuner."""

    from taoryx.x15_adapter import build_x15_source_direct_wrench_adapter

    return build_x15_source_direct_wrench_adapter("rigid_body_6dof_direct_wrench")
    ####


def _surface_lqi_tuning_adapter() -> StandardFamilyAdapter:
    """Build the X-15 physical-wrench projection for the surface LQI campaign."""

    from taoryx.x15_adapter import build_x15_source_surface_physical_lqi_design

    from taoryx.physical_wrench_tuning import build_projected_physical_wrench_tuning_adapter

    return build_projected_physical_wrench_tuning_adapter(
        build_x15_source_surface_physical_lqi_design(),
        family_id=_FAMILY_ID,
        adapter_id="taoryx.x15.source_surface.physical_wrench_tuning.v1",
        physical_family="powered_fixed_wing",
        tier="rigid_body_6dof_surface_allocated",
        state_units={
            "roll_error_rad": "rad",
            "pitch_error_rad": "rad",
            "yaw_error_rad": "rad",
            "p_rad_s": "rad/s",
            "q_rad_s": "rad/s",
            "r_rad_s": "rad/s",
        },
        validity_envelope=(
            "X-15 frozen release-fixture local attitude/rate physical-wrench projection; nonlinear execution "
            "allocates each moment request through the bounded source surfaces"
        ),
        omitted_physics=(
            "raw source-surface campaign synthesis",
            "nonlinear surface allocation in the campaign adapter",
            "full flight trim, translation, guidance, and qualification",
        ),
    )
    ####


def _direct_wrench_lqr_campaign() -> TuningCampaign:
    from taoryx.x15_adapter import build_x15_direct_wrench_tuning_campaign

    return build_x15_direct_wrench_tuning_campaign()
    ####


def _direct_wrench_lqi_campaign() -> TuningCampaign:
    from taoryx.x15_adapter import build_x15_direct_wrench_lqi_tuning_campaign

    return build_x15_direct_wrench_lqi_tuning_campaign()
    ####


def _surface_lqi_campaign() -> TuningCampaign:
    from taoryx.x15_adapter import build_x15_source_surface_lqi_tuning_campaign

    return build_x15_source_surface_lqi_tuning_campaign()
    ####


def _campaigns() -> tuple[ControllerTuningCampaignRegistration, ...]:
    """Return X-15-only campaign metadata and deferred numerical factories."""

    return (
        ControllerTuningCampaignRegistration(
            id="x15-source-release-direct-wrench-v1",
            provider_id=_PROVIDER_ID,
            provider_aliases=(_AGGREGATE_PROVIDER_ID,),
            model_id=_FAMILY_ID,
            family_id=_FAMILY_ID,
            fidelity="rigid_body_6dof_direct_wrench",
            realization_ids=("rigid_body_6dof_direct_wrench",),
            mission_template_ids=("x15_local_direct_wrench_screen_v1",),
            description="Scaled LQR source-release local direct-wrench campaign over the declared X-15 bridge witness.",
            adapter_factory=_direct_wrench_tuning_adapter,
            campaign_factory=_direct_wrench_lqr_campaign,
            claim_boundary=(
                "This campaign screens bounded generalized direct-wrench feedback at the source release point. "
                "It does not synthesize or qualify physical X-15 effector allocation, navigation, or the staged mission."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="x15-source-release-direct-wrench-lqi-v1",
            provider_id=_PROVIDER_ID,
            provider_aliases=(_AGGREGATE_PROVIDER_ID,),
            model_id=_FAMILY_ID,
            family_id=_FAMILY_ID,
            fidelity="rigid_body_6dof_direct_wrench",
            realization_ids=("rigid_body_6dof_direct_wrench",),
            mission_template_ids=("x15_local_direct_wrench_lqi_screen_v1",),
            description="Scaled LQI source-release local speed campaign over the declared X-15 direct-wrench bridge.",
            adapter_factory=_direct_wrench_tuning_adapter,
            campaign_factory=_direct_wrench_lqi_campaign,
            claim_boundary=(
                "This campaign screens bounded generalized direct-wrench feedback at the source release point. "
                "It does not synthesize or qualify physical X-15 effector allocation, navigation, or the staged mission."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="x15-source-surface-local-lqi-v1",
            provider_id=_PROVIDER_ID,
            provider_aliases=(_AGGREGATE_PROVIDER_ID,),
            model_id=_FAMILY_ID,
            family_id=_FAMILY_ID,
            fidelity="rigid_body_6dof_surface_allocated",
            realization_ids=("rigid_body_6dof_surface_allocated",),
            mission_template_ids=("x15_source_surface_attitude_rate_lqi_screen_v1",),
            description=(
                "Exact scaled LQI candidate for the frozen-translation X-15 attitude/rate physical-wrench runtime; "
                "each nonlinear moment request remains allocated through bounded source surfaces."
            ),
            adapter_factory=_surface_lqi_tuning_adapter,
            campaign_factory=_surface_lqi_campaign,
            local_controller_screens=(
                {
                    "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
                    "id": "x15-source-release-surface-attitude-rate-lqi-screen-v1",
                    "mission_template_id": "x15_source_surface_attitude_rate_lqi_screen_v1",
                    "fidelity": "rigid_body_6dof_surface_allocated",
                    "operations": ["validate", "batch"],
                    "control_realization": "source_surface_physical_wrench_lqi_allocation",
                    "physical_effector_allocation": True,
                    "batch_action_trace": "emits_committed_interval_trace",
                    "controller": {
                        "method": "lqi",
                        "campaign_id": "x15-source-surface-local-lqi-v1",
                        "integral_output_names": ["roll_error_rad", "pitch_error_rad", "yaw_error_rad"],
                        "fixed_cadence_s": 0.01,
                        "screen_duration_s": 2.0,
                    },
                    "effector_controls": [
                        {"id": "effector.surface.symmetric_stabilator.position", "native_control_id": "symmetric_stabilator", "unit": "deg", "lower": -14.9, "upper": 34.9},
                        {"id": "effector.surface.differential_stabilator.position", "native_control_id": "differential_stabilator", "unit": "deg", "lower": -20.05, "upper": 20.05},
                        {"id": "effector.surface.rudder.position", "native_control_id": "rudder", "unit": "deg", "lower": -29.79, "upper": 29.79},
                    ],
                    "claim_boundary": (
                        "This source-surface LQI screen holds the X-15 release translational fixture fixed and controls "
                        "only local attitude error/body rate through actual bounded source-surface allocation. It does not "
                        "establish full-state trim, translation, propulsion/RCS, navigation, or flight qualification."
                    ),
                },
            ),
            claim_boundary=(
                "This campaign auto-tunes only the frozen-translation X-15 local attitude/rate source-surface model. "
                "It does not establish full vehicle trim, high-energy guidance, a gain schedule, robustness, or qualification."
            ),
        ),
    )
    ####


def _surface_authority_capability_adapter() -> object:
    from taoryx.x15_surface_authority_screen import X15SourceSurfaceAuthorityScreenCapabilityAdapter

    return X15SourceSurfaceAuthorityScreenCapabilityAdapter()
    ####


def _surface_lqi_capability_adapter() -> object:
    from taoryx.x15_local_physical_surface_lqi_screen import X15LocalPhysicalSurfaceLqiScreenCapabilityAdapter

    return X15LocalPhysicalSurfaceLqiScreenCapabilityAdapter()
    ####


def _preflight_direct_wrench(composition: CompiledVehicleComposition) -> Any:
    from taoryx.vehicle_execution_preflight import _preflight_local_direct_wrench

    return _preflight_local_direct_wrench(composition)
    ####


def _preflight_surface_authority(composition: CompiledVehicleComposition) -> Any:
    from taoryx.x15_surface_authority_screen import preflight_x15_source_surface_authority_screen

    return preflight_x15_source_surface_authority_screen(composition)
    ####


def _preflight_surface_lqi(composition: CompiledVehicleComposition) -> Any:
    from taoryx.x15_local_physical_surface_lqi_screen import preflight_x15_local_physical_surface_lqi_screen

    return preflight_x15_local_physical_surface_lqi_screen(composition)
    ####


def _open_direct_wrench_episode(
    composition: CompiledVehicleComposition,
    seed: int | None,
    integration_step_s: float,
) -> object:
    from taoryx.composition_episode import _open_local_direct_wrench_episode

    return _open_local_direct_wrench_episode(composition, seed, integration_step_s)
    ####


def _verify_direct_wrench_batch_episode_parity(
    composition: CompiledVehicleComposition,
    payload: dict[str, object],
) -> object:
    from taoryx.local_direct_wrench_batch_episode_parity import verify_serialized_local_direct_wrench_batch_episode_parity

    return verify_serialized_local_direct_wrench_batch_episode_parity(composition, payload)
    ####


def _direct_wrench_batch(request: VehicleBatchExecutionRequest) -> Any:
    from taoryx.local_direct_wrench_composition_execution import execute_local_direct_wrench_composition

    if request.max_steps is not None:
        raise ValueError("--max-steps is not available for the X-15 local direct-wrench screen")
    return execute_local_direct_wrench_composition(
        request.composition,
        request.output_dir,
        tuning_context=request.tuning_context,
    )
    ####


def _surface_authority_batch(request: VehicleBatchExecutionRequest) -> Any:
    from taoryx.x15_surface_authority_screen import execute_x15_source_surface_authority_screen

    if request.tuning_context is not None:
        raise ValueError("the X-15 source-surface authority screen does not accept a controller tuning context")
    return execute_x15_source_surface_authority_screen(
        request.composition,
        request.output_dir,
        max_steps=request.max_steps,
    )
    ####


def _surface_lqi_batch(request: VehicleBatchExecutionRequest) -> Any:
    from taoryx.x15_local_physical_surface_lqi_screen import execute_x15_local_physical_surface_lqi_screen

    return execute_x15_local_physical_surface_lqi_screen(
        request.composition,
        request.output_dir,
        max_steps=request.max_steps,
        tuning_context=request.tuning_context,
    )
    ####


def _provider() -> CatalogMissionCompositionProvider:
    """Build the portable X-15-only Mission Composition provider.

    The deferred plug-in proxy caches this provider per discovery catalog. Do
    not add a module-global cache here: a broad host must not make a later
    focused host inherit its wider execution scope.
    """

    from taoryx.family_manifest import load_unified_family_manifest_catalog
    from taoryx.horizontal_fidelity import load_horizontal_registry
    from taoryx.plugins import current_plugin_catalog
    from taoryx.trajectory.catalog_mission_composition import CatalogMissionCompositionProvider
    from taoryx.trajectory.pseudo6dof_profiles import load_pseudo6dof_catalog
    from taoryx.vehicle_composition_registry import (
        load_resolved_vehicle_composition_catalog,
        load_vehicle_composition_registry,
    )

    root = model_resource_root()
    pseudo = load_pseudo6dof_catalog(root / "verification/pseudo6dof_profiles.yaml")
    horizontal = load_horizontal_registry(root / "verification/horizontal_fidelity_registry.yaml")
    manifests = load_unified_family_manifest_catalog(
        horizontal=horizontal,
        pseudo=pseudo,
        root=root,
        validate_source_imports=False,
    )
    registry = load_vehicle_composition_registry(root / "verification/vehicle_composition_registry.yaml")
    catalog = load_resolved_vehicle_composition_catalog(registry=registry, manifests=manifests)
    return CatalogMissionCompositionProvider(
        catalog,
        provider_id=_PROVIDER_ID,
        provider_name="TAORYX X-15 Mission Composition",
        provider_short_name="X-15",
        provider_summary="Source-backed X-15 local direct-wrench and source-surface composition endpoints.",
        provider_version=_PACKAGE_VERSION,
        plugin_catalog=current_plugin_catalog(),
        include_builtin_workflows=False,
        allowed_mission_template_ids={_FAMILY_ID: _FOCUSED_MISSION_TEMPLATE_IDS},
    )
    ####


def _register(registrar: PluginRegistrar) -> None:
    """Publish only X-15-owned models, local screens, and package data seams."""

    registrar.register_vehicle_catalog_fragment(_CATALOG_FRAGMENT)
    from taoryx.x15_local_direct_wrench import x15_local_direct_wrench_screen_definitions

    registration = registrar.register_family_adapter_factory(_FAMILY_ID, _family_adapter_registration)
    registrar.register_model(_FAMILY_ID, registration)
    registrar.register_mission_composition_provider_factory(_PROVIDER_ID, _provider)
    for definition in x15_local_direct_wrench_screen_definitions():
        registrar.register_local_direct_wrench_screen_definition(definition)
        registrar.register_mission_capability_adapter(LocalDirectWrenchScreenCapabilityAdapter(definition))
    registrar.register_mission_capability_adapter(
        _LazyMissionCapabilityAdapter(_SURFACE_AUTHORITY_TRANSLATOR_ID, _surface_authority_capability_adapter)
    )
    registrar.register_mission_capability_adapter(
        _LazyMissionCapabilityAdapter(_SURFACE_LQI_TRANSLATOR_ID, _surface_lqi_capability_adapter)
    )
    for campaign in _campaigns():
        registrar.register_controller_tuning_campaign(campaign)
    for translator_id, handler in (
        (_DIRECT_LQR_TRANSLATOR_ID, _preflight_direct_wrench),
        (_DIRECT_LQI_TRANSLATOR_ID, _preflight_direct_wrench),
        (_SURFACE_AUTHORITY_TRANSLATOR_ID, _preflight_surface_authority),
        (_SURFACE_LQI_TRANSLATOR_ID, _preflight_surface_lqi),
    ):
        registrar.register_semantic_preflight_handler_callback(translator_id, handler)
    registrar.register_episode_factory("x15_local_direct_wrench_episode.v1", _open_direct_wrench_episode)
    registrar.register_batch_episode_parity_verifier(
        "taoryx.x15.local_direct_wrench_batch_episode_parity.v1",
        _verify_direct_wrench_batch_episode_parity,
    )
    registrar.register_execution_factory_request_v1("x15_local_direct_wrench_screen.v1", _direct_wrench_batch)
    registrar.register_execution_factory_request_v1("x15_source_surface_authority_screen.v1", _surface_authority_batch)
    registrar.register_execution_factory_request_v1("x15_local_physical_surface_lqi_screen.v1", _surface_lqi_batch)
    ####


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="taoryx.x15",
        package="taoryx-x15",
        version=_PACKAGE_VERSION,
        api_version="1",
        description="Source-backed X-15 local control screens, source tables, and Mission Composition provider.",
    ),
    register_callback=_register,
)

__all__ = ["PLUGIN"]
