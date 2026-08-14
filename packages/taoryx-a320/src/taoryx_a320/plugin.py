"""Plug-in registration for the OpenAP and surrogate-composite A320 family."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from typing import TYPE_CHECKING, Any

from taoryx.controller_tuning_registry import ControllerTuningCampaignRegistration
from taoryx.plugins import PluginDefinition, PluginMetadata, PluginRegistrar, VehicleCatalogFragment

from .resources import model_resource_root

if TYPE_CHECKING:
    from taoryx.family_adapter import StandardFamilyAdapter
    from taoryx.family_adapter_probes import AdapterProbeCase
    from taoryx.family_adapter_registry import FamilyAdapterRegistration
    from taoryx.fidelity_contracts import FidelityTier
    from taoryx.trajectory.catalog_mission_composition import CatalogMissionCompositionProvider
    from taoryx.tuning_campaign import TuningCampaign
    from taoryx.vehicle_batch_execution import VehicleBatchExecutionRequest
    from taoryx.vehicle_composition import CompiledVehicleComposition

_AGGREGATE_PROVIDER_ID = "taoryx.registry.mission-composition"
_PROVIDER_ID = "taoryx.a320.mission-composition"
_PACKAGE_VERSION = "0.1.0a0"
_FAMILY_ID = "a320_openap_3dof"
_RACETRACK_TRANSLATOR_ID = "taoryx.a320_openap_racetrack.capability_scaled.v1"
_LOCAL_NATIVE_LQI_TRANSLATOR_ID = "taoryx.a320.local_native_coordinate_lqi_screen.capability.v1"
_CATALOG_FRAGMENT = VehicleCatalogFragment(
    id="taoryx.a320.vehicle-catalog",
    resource_package="taoryx_a320",
    family_ids=(_FAMILY_ID,),
)


class _LazyMissionCapabilityAdapter:
    """Expose an A320 capability identity without loading its numerical plant."""

    def __init__(self, identifier: str, factory: Callable[[], object]) -> None:
        self.id = identifier
        self._factory = factory
        self._delegate: object | None = None
        ####

    def _resolve(self) -> Any:
        if self._delegate is None:
            candidate = self._factory()
            if getattr(candidate, "id", None) != self.id:
                raise ValueError(f"lazy A320 capability adapter {self.id!r} resolved a mismatched identity")
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


def _a320_racetrack_capability_adapter() -> object:
    """Load the A320 route planner only after a composition selects it."""

    from taoryx.a320_mission_capability import A320OpenAPRacetrackCapabilityAdapter

    return A320OpenAPRacetrackCapabilityAdapter()
    ####


def _a320_local_native_lqi_capability_adapter() -> object:
    """Load the A320 local named-coordinate LQI planner on demand."""

    from taoryx.a320_mission_capability import A320LocalNativeCoordinateLqiCapabilityAdapter

    return A320LocalNativeCoordinateLqiCapabilityAdapter()
    ####


def _a320_local_native_lqi_screen_definition() -> object:
    """Load A320-owned native-coordinate LQI metadata without constructing its plant."""

    from taoryx.a320_local_native_coordinate_lqi import a320_local_native_coordinate_lqi_screen_definition

    return a320_local_native_coordinate_lqi_screen_definition()
    ####


def _a320_local_native_lqi_screen_advertisement() -> object:
    """Load static A320 LQI metadata for authoring and UI consumers."""

    from taoryx.a320_local_native_coordinate_lqi import a320_local_native_coordinate_lqi_controller_screen_advertisement

    return a320_local_native_coordinate_lqi_controller_screen_advertisement()
    ####


def _a320_family_adapter(tier: FidelityTier) -> StandardFamilyAdapter:
    """Build one executable A320 reduced tier through the public adapter seam."""

    from taoryx.trajectory.a320_adapter import A320OpenAPControlPlant, A320Pseudo6DOFControlPlant
    from taoryx.trajectory.a320_openap import A320OpenAPModel
    from taoryx.trajectory.a320_pseudo6dof import A320Pseudo6DOFModel

    from taoryx.family_adapter import StandardFamilyAdapter, descriptor_from_control_plant

    plant: A320OpenAPControlPlant | A320Pseudo6DOFControlPlant
    state_units: dict[str, str]
    control_units: dict[str, str]
    if tier == "point_mass_3dof":
        plant = A320OpenAPControlPlant(A320OpenAPModel.from_repository(model_resource_root()))
        state_units = {"altitude_m": "m", "mach": "1", "mass_kg": "kg", "range_m": "m"}
        control_units = {"throttle_ratio": "1", "flight_path_angle_rad": "rad"}
    elif tier == "pseudo_6dof":
        plant = A320Pseudo6DOFControlPlant(A320Pseudo6DOFModel.from_repository(model_resource_root()))
        state_units = {
            "altitude_m": "m",
            "mach": "1",
            "mass_kg": "kg",
            "range_m": "m",
            "alpha_rad": "rad",
            "beta_rad": "rad",
            "roll_rate_rad_s": "rad/s",
            "pitch_rate_rad_s": "rad/s",
            "yaw_rate_rad_s": "rad/s",
            "bank_angle_rad": "rad",
        }
        control_units = {
            "throttle_ratio": "1",
            "flight_path_angle_rad": "rad",
            "aileron_rad": "rad",
            "elevator_rad": "rad",
            "rudder_rad": "rad",
        }
    else:
        raise ValueError(f"{_FAMILY_ID} has no executable {tier} adapter")
    descriptor = descriptor_from_control_plant(
        plant,
        family_id=_FAMILY_ID,
        adapter_id="taoryx.fixed_wing.openap.v1",
        physical_family="powered_fixed_wing",
        tier=tier,
        state_units=state_units,
        control_units=control_units,
        evidence_status="development",
        omitted_physics=("physical surface allocation", "manufacturer-authoritative flight dynamics"),
    )
    return StandardFamilyAdapter.from_control_plant(descriptor, plant)
    ####


def _a320_family_probe(adapter: StandardFamilyAdapter) -> AdapterProbeCase:
    """Provide one trim-consistent operating point for each A320 reduced tier."""

    from taoryx.trajectory.a320_adapter import A320OpenAPControlPlant, A320Pseudo6DOFControlPlant

    from taoryx.family_adapter_probes import AdapterProbeCase

    plant = adapter.plant
    if isinstance(plant, A320OpenAPControlPlant):
        point = plant.operating_point
        baseline = plant.model.evaluate(point)
        state = {
            "altitude_m": point.altitude_m,
            "mach": point.mach,
            "mass_kg": point.mass_kg,
            "range_m": 0.0,
        }
        effectors = {"throttle_ratio": baseline.required_throttle_ratio, "flight_path_angle_rad": 0.0}
    elif isinstance(plant, A320Pseudo6DOFControlPlant):
        point = plant.operating_point
        solved = plant.model.trim_pseudo6dof(point)
        state = {
            "altitude_m": point.altitude_m,
            "mach": point.mach,
            "mass_kg": point.mass_kg,
            "range_m": 0.0,
            "alpha_rad": float(solved.state.get("alpha_rad", 0.0)),
            "beta_rad": 0.0,
            "roll_rate_rad_s": 0.0,
            "pitch_rate_rad_s": 0.0,
            "yaw_rate_rad_s": 0.0,
            "bank_angle_rad": 0.0,
        }
        effectors = {name: float(value) for name, value in solved.controls.items()}
    else:
        raise ValueError("unexpected A320 family-adapter plant")
    return AdapterProbeCase(
        state=state,
        effectors=effectors,
        environment={"thrust_mode": point.thrust_mode},
        trim_target=state,
        trim_initial_guess=effectors,
        previous_effectors=effectors,
    )
    ####


def _family_adapter_registration() -> FamilyAdapterRegistration:
    """Return the A320 registration without constructing the OpenAP model."""

    from taoryx.family_adapter_registry import FamilyAdapterRegistration

    return FamilyAdapterRegistration(
        _FAMILY_ID,
        "taoryx.fixed_wing.openap.v1",
        "available",
        _a320_family_adapter,
        probe_factory=_a320_family_probe,
        supported_tiers=("point_mass_3dof", "pseudo_6dof"),
        note=(
            "OpenAP point-mass and named pseudo-6DOF response products are executable through the A320-owned "
            "Composition path; direct-wrench and physical-surface tiers remain intentionally unavailable"
        ),
    )
    ####


def _provider() -> CatalogMissionCompositionProvider:
    """Build a portable A320-only Mission Composition provider.

    The deferred plug-in proxy caches this object per discovery catalog. Do
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
    manifests = load_unified_family_manifest_catalog(horizontal=horizontal, pseudo=pseudo, root=root, validate_source_imports=False)
    registry = load_vehicle_composition_registry(root / "verification/vehicle_composition_registry.yaml")
    catalog = load_resolved_vehicle_composition_catalog(registry=registry, manifests=manifests)
    return CatalogMissionCompositionProvider(
        catalog,
        provider_id=_PROVIDER_ID,
        provider_name="TAORYX A320 Mission Composition",
        provider_short_name="A320/OpenAP",
        provider_summary="OpenAP A320 lower-fidelity controls and surrogate pseudo-6DOF composition surface.",
        provider_version=_PACKAGE_VERSION,
        plugin_catalog=current_plugin_catalog(),
        include_builtin_workflows=False,
    )
    ####


def _a320_point_tuning_adapter() -> StandardFamilyAdapter:
    """Build the point-mass A320 tier used by the common campaign runner."""

    return _a320_family_adapter("point_mass_3dof")
    ####


def _a320_pseudo_tuning_adapter() -> StandardFamilyAdapter:
    """Build the pseudo-6DOF A320 tier used by the common campaign runner."""

    from taoryx.trajectory.a320_adapter import build_a320_pseudo_control_adapter

    return build_a320_pseudo_control_adapter()
    ####


def _build_a320_point_tuning_campaign() -> TuningCampaign:
    from taoryx.trajectory.a320_adapter import build_a320_point_tuning_campaign

    return build_a320_point_tuning_campaign()
    ####


def _build_a320_pseudo_tuning_campaign() -> TuningCampaign:
    from taoryx.trajectory.a320_adapter import build_a320_pseudo_tuning_campaign

    return build_a320_pseudo_tuning_campaign()
    ####


def _campaigns() -> tuple[ControllerTuningCampaignRegistration, ...]:
    aliases = (_AGGREGATE_PROVIDER_ID,)
    return (
        ControllerTuningCampaignRegistration(
            id="a320-point-cruise-performance-lqr-v1",
            provider_id=_PROVIDER_ID,
            provider_aliases=aliases,
            model_id=_FAMILY_ID,
            family_id=_FAMILY_ID,
            fidelity="point_mass_3dof",
            realization_ids=("point_mass_3dof",),
            mission_template_ids=("powered_fixed_wing_racetrack_v1",),
            description="Scaled LQR nominal cruise-performance campaign over the declared A320 OpenAP point-mass state and control coordinates.",
            adapter_factory=_a320_point_tuning_adapter,
            campaign_factory=_build_a320_point_tuning_campaign,
            claim_boundary=(
                "This campaign is an OpenAP point-mass local performance design screen. It does not establish a route "
                "controller, wind robustness, physical surface allocation, or Airbus qualification."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="a320-pseudo-cruise-attitude-v1",
            provider_id=_PROVIDER_ID,
            provider_aliases=aliases,
            model_id=_FAMILY_ID,
            family_id=_FAMILY_ID,
            fidelity="pseudo_6dof",
            realization_ids=("pseudo_6dof", "jsbsim_surrogate_composite_pseudo6dof"),
            mission_template_ids=("powered_fixed_wing_racetrack_v1", "a320_local_native_coordinate_lqi_screen_v1"),
            description="Scaled LQI cruise-attitude inner-loop campaign over the declared A320 pseudo-6DOF response model.",
            adapter_factory=_a320_pseudo_tuning_adapter,
            campaign_factory=_build_a320_pseudo_tuning_campaign,
            claim_boundary=(
                "This campaign covers only the OpenAP/JSBSim surrogate pseudo-6DOF response at its declared local "
                "cruise condition; it does not establish physical surface allocation or transport qualification."
            ),
        ),
    )
    ####


def _preflight_a320_racetrack(composition: CompiledVehicleComposition) -> Any:
    from taoryx.vehicle_execution_preflight import preflight_powered_fixed_wing_racetrack

    return preflight_powered_fixed_wing_racetrack(composition)
    ####


def _preflight_a320_local_native_lqi(composition: CompiledVehicleComposition) -> Any:
    from taoryx.vehicle_execution_preflight import preflight_local_native_coordinate_lqi

    return preflight_local_native_coordinate_lqi(composition)
    ####


def _open_a320_reduced_episode(
    composition: CompiledVehicleComposition,
    seed: int | None,
    integration_step_s: float,
) -> object:
    from taoryx.a320_composition_episode import open_a320_reduced_composition_episode

    return open_a320_reduced_composition_episode(composition, seed, integration_step_s)
    ####


def _verify_a320_reduced_batch_episode_parity(
    composition: CompiledVehicleComposition,
    payload: Mapping[str, object],
) -> object:
    from taoryx.a320_reduced_batch_episode_parity import verify_serialized_a320_reduced_batch_episode_parity

    return verify_serialized_a320_reduced_batch_episode_parity(composition, payload)
    ####


def _a320_reduced_batch(request: VehicleBatchExecutionRequest) -> Any:
    if request.max_steps is not None:
        raise ValueError("--max-steps is not available for the A320 reduced batch")
    from taoryx.a320_reduced_execution import execute_a320_reduced_composition

    return execute_a320_reduced_composition(request.composition, request.output_dir)
    ####


def _a320_local_native_lqi_batch(request: VehicleBatchExecutionRequest) -> Any:
    if request.max_steps is not None:
        raise ValueError("--max-steps is not available for the A320 local native-coordinate LQI screen")
    from taoryx.local_native_coordinate_lqi_composition_execution import execute_local_native_coordinate_lqi_composition

    return execute_local_native_coordinate_lqi_composition(
        request.composition,
        request.output_dir,
        tuning_context=request.tuning_context,
    )
    ####


def _register(registrar: PluginRegistrar) -> None:
    """Publish only A320-owned model, control, planning, and execution seams."""

    registrar.register_vehicle_catalog_fragment(_CATALOG_FRAGMENT)
    registration = registrar.register_family_adapter_factory(_FAMILY_ID, _family_adapter_registration)
    registrar.register_model(_FAMILY_ID, registration)
    registrar.register_mission_composition_provider_factory(_PROVIDER_ID, _provider)
    registrar.register_mission_capability_adapter(
        _LazyMissionCapabilityAdapter(_RACETRACK_TRANSLATOR_ID, _a320_racetrack_capability_adapter)
    )
    registrar.register_mission_capability_adapter(
        _LazyMissionCapabilityAdapter(_LOCAL_NATIVE_LQI_TRANSLATOR_ID, _a320_local_native_lqi_capability_adapter)
    )
    registrar.register_local_native_coordinate_lqi_screen_definition(_a320_local_native_lqi_screen_definition())
    registrar.register_local_controller_screen_advertisement(_a320_local_native_lqi_screen_advertisement())
    for campaign in _campaigns():
        registrar.register_controller_tuning_campaign(campaign)
    registrar.register_semantic_preflight_handler_callback(_RACETRACK_TRANSLATOR_ID, _preflight_a320_racetrack)
    registrar.register_semantic_preflight_handler_callback(_LOCAL_NATIVE_LQI_TRANSLATOR_ID, _preflight_a320_local_native_lqi)
    registrar.register_episode_factory("reduced_fixed_wing_a320_episode.v1", _open_a320_reduced_episode)
    registrar.register_batch_episode_parity_verifier(
        "taoryx.reduced_fixed_wing.a320_action_trace_batch_episode_parity.v1",
        _verify_a320_reduced_batch_episode_parity,
    )
    registrar.register_execution_factory_request_v1("reduced_fixed_wing_openap.v1", _a320_reduced_batch)
    registrar.register_execution_factory_request_v1("local_native_coordinate_lqi_screen.v1", _a320_local_native_lqi_batch)
    ####


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="taoryx.a320",
        package="taoryx-a320",
        version=_PACKAGE_VERSION,
        api_version="1",
        description="OpenAP 3DOF and JSBSim-surrogate pseudo-6DOF A320 controls and composition runtime.",
    ),
    register_callback=_register,
)

__all__ = ["PLUGIN"]
