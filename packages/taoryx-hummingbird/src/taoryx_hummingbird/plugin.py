"""Plug-in registration for the standalone Hummingbird multirotor family."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from functools import lru_cache
from typing import TYPE_CHECKING, Any

from taoryx.controller_tuning_registry import ControllerTuningCampaignRegistration
from taoryx.local_direct_wrench_screen_registry import LocalDirectWrenchScreenCapabilityAdapter
from taoryx.plugins import PluginDefinition, PluginMetadata, PluginRegistrar, VehicleCatalogFragment

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
_PROVIDER_ID = "taoryx.hummingbird.mission-composition"
_PACKAGE_VERSION = "0.1.0a0"
_FAMILY_ID = "hummingbird"
_CATALOG_FRAGMENT = VehicleCatalogFragment(
    id="taoryx.hummingbird.vehicle-catalog",
    resource_package="taoryx_hummingbird",
    family_ids=(_FAMILY_ID,),
)


class _LazyMissionCapabilityAdapter:
    """Expose a stable capability identity without eagerly importing its plant."""

    def __init__(self, identifier: str, factory: Callable[[], object]) -> None:
        self.id = identifier
        self._factory = factory
        self._delegate: object | None = None
        ####

    def _resolve(self) -> Any:
        """Build the family adapter only when a caller evaluates a composition."""

        if self._delegate is None:
            candidate = self._factory()
            if getattr(candidate, "id", None) != self.id:
                raise ValueError(
                    f"lazy Hummingbird capability adapter {self.id!r} resolved a mismatched identity"
                )
            self._delegate = candidate
        return self._delegate
        ####

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Delegate exact family/mission/fidelity ownership when it is needed."""

        return bool(self._resolve().supports(composition))
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> Any:
        """Delegate the family-owned feasibility estimate on demand."""

        return self._resolve().estimate(composition)
        ####

    ####


def _hummingbird_hover_capability_adapter() -> object:
    """Load the aggregate-thrust capability planner only on use."""

    from taoryx.hummingbird_mission_capability import HummingbirdHoverTranslationCapabilityAdapter

    return HummingbirdHoverTranslationCapabilityAdapter()
    ####


def _reduced_interface_extension() -> object:
    """Load Hummingbird-only pseudo-6DOF controls/readbacks on selection."""

    from taoryx.hummingbird_reduced_interface import hummingbird_reduced_interface_extension

    return hummingbird_reduced_interface_extension()
    ####


def _hummingbird_local_physical_capability_adapter() -> object:
    """Load the source-local attitude/rate planner only on use."""

    from taoryx.hummingbird_local_physical_control_screen import HummingbirdLocalPhysicalControlScreenCapabilityAdapter

    return HummingbirdLocalPhysicalControlScreenCapabilityAdapter()
    ####


def _hummingbird_local_horizontal_capability_adapter() -> object:
    """Load the source-local horizontal planner only on use."""

    from taoryx.hummingbird_local_physical_control_screen import HummingbirdLocalHorizontalTranslationLqiScreenCapabilityAdapter

    return HummingbirdLocalHorizontalTranslationLqiScreenCapabilityAdapter()
    ####


def _hummingbird_local_vertical_capability_adapter() -> object:
    """Load the source-local vertical planner only on use."""

    from taoryx.hummingbird_local_physical_control_screen import HummingbirdLocalVerticalTranslationLqiScreenCapabilityAdapter

    return HummingbirdLocalVerticalTranslationLqiScreenCapabilityAdapter()
    ####


def _preflight_hummingbird_local_physical_control_screen(
    composition: CompiledVehicleComposition,
) -> Any:
    """Load the source-local semantic translator only on execution preflight."""

    from taoryx.hummingbird_local_physical_control_screen import preflight_hummingbird_local_physical_control_screen

    return preflight_hummingbird_local_physical_control_screen(composition)
    ####


def _preflight_hummingbird_hover_yaw(composition: CompiledVehicleComposition) -> Any:
    """Load the aggregate-thrust semantic translator only on execution preflight."""

    from taoryx.hummingbird_preflight import preflight_hummingbird_hover_yaw

    return preflight_hummingbird_hover_yaw(composition)
    ####


def _preflight_hummingbird_local_direct_wrench(
    composition: CompiledVehicleComposition,
) -> Any:
    """Load the shared direct-wrench translator only on execution preflight."""

    from taoryx.vehicle_execution_preflight import _preflight_local_direct_wrench

    return _preflight_local_direct_wrench(composition)
    ####


def _build_hummingbird_individual_rotor_source_table_plant() -> object:
    """Load the source rotor plant only when an adapter needs it."""

    from taoryx.source_table_multirotor import build_hummingbird_individual_rotor_source_table_plant

    return build_hummingbird_individual_rotor_source_table_plant()
    ####


def _build_hummingbird_pseudo_tuning_campaign() -> TuningCampaign:
    """Load the aggregate-thrust tuning campaign only when it is selected."""

    from taoryx.trajectory.hummingbird_adapter import build_hummingbird_pseudo_tuning_campaign

    return build_hummingbird_pseudo_tuning_campaign()
    ####


def _build_hummingbird_source_rotor_lqi_tuning_campaign() -> TuningCampaign:
    """Load the source rotor attitude campaign only when it is selected."""

    from taoryx.source_table_multirotor import build_hummingbird_source_rotor_lqi_tuning_campaign

    return build_hummingbird_source_rotor_lqi_tuning_campaign()
    ####


def _build_hummingbird_source_rotor_vertical_lqi_tuning_campaign() -> TuningCampaign:
    """Load the source rotor vertical campaign only when it is selected."""

    from taoryx.source_table_multirotor import build_hummingbird_source_rotor_vertical_lqi_tuning_campaign

    return build_hummingbird_source_rotor_vertical_lqi_tuning_campaign()
    ####


def _open_hummingbird_aggregate_thrust_episode(
    composition: CompiledVehicleComposition,
    seed: int | None,
    integration_step_s: float,
) -> object:
    """Defer the Hummingbird-owned episode implementation until selected."""

    from taoryx.hummingbird_composition_episode import open_hummingbird_pseudo_composition_episode

    return open_hummingbird_pseudo_composition_episode(composition, seed, integration_step_s)
    ####


def _verify_hummingbird_batch_episode_parity(
    composition: CompiledVehicleComposition,
    payload: Mapping[str, object],
) -> object:
    """Defer the parity host and policy runtime until verification runs."""

    from taoryx.composition_batch_episode_parity import verify_serialized_composition_batch_episode_parity

    return verify_serialized_composition_batch_episode_parity(composition, payload)
    ####


def _source_local_probe(adapter: StandardFamilyAdapter) -> AdapterProbeCase:
    """Return the source-owned hover condition for generic adapter probes."""

    from taoryx.family_adapter_probes import AdapterProbeCase

    plant = adapter.plant
    if plant is None or not hasattr(plant, "source_local_state") or not hasattr(plant, "source_effectors"):
        raise ValueError("hummingbird: plant does not expose a source local operating point")
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


def _source_table_multirotor_factory(
    builder: Callable[[], object],
) -> Callable[[FidelityTier], StandardFamilyAdapter]:
    """Bind the Hummingbird source rotor plant without a family fallback."""

    @lru_cache(maxsize=1)
    def plant() -> object:
        return builder()
        ####

    @lru_cache(maxsize=None)
    def build(tier: FidelityTier) -> StandardFamilyAdapter:
        from taoryx.family_adapter import StandardFamilyAdapter, descriptor_from_control_plant

        source_plant = plant()
        control_names = getattr(source_plant, "control_names", ())
        limits = getattr(source_plant, "effector_limits", None)
        if not control_names or not isinstance(limits, dict):
            raise ValueError("hummingbird: source-table plant has no declared control limits")
        descriptor = descriptor_from_control_plant(
            source_plant,  # type: ignore[arg-type]
            family_id="hummingbird",
            adapter_id="taoryx.multirotor.native_quad_x.v1",
            physical_family="multirotor",
            tier=tier,
            control_units={name: limits[name].unit for name in control_names},
            evidence_status="development",
            omitted_physics=(
                "battery and voltage resource model",
                "blade-resolved and dynamic-inflow rotor physics",
                "flight-qualification evidence",
            ),
        )
        return StandardFamilyAdapter.from_control_plant(descriptor, source_plant)  # type: ignore[arg-type]
        ####

    return build
    ####


def _hummingbird_family_adapter_registration() -> FamilyAdapterRegistration:
    """Build the numerical adapter registration only when a host needs it."""

    from taoryx.family_adapter_registry import FamilyAdapterRegistration

    return FamilyAdapterRegistration(
        "hummingbird",
        "taoryx.multirotor.native_quad_x.v1",
        "available",
        _source_table_multirotor_factory(_build_hummingbird_individual_rotor_source_table_plant),
        probe_factory=_source_local_probe,
        supported_tiers=("rigid_body_6dof_direct_wrench", "rigid_body_6dof_surface_allocated"),
        note="runtime-owned individual-rotor local plant with a separate aggregate-thrust pseudo-6DOF seam",
    )
    ####


def _hummingbird_pseudo_tuning_adapter() -> StandardFamilyAdapter:
    """Build the reduced Hummingbird plant used by the common campaign host."""

    from taoryx.trajectory.hummingbird_adapter import HummingbirdPseudo6DOFControlPlant

    from taoryx.family_adapter import StandardFamilyAdapter, descriptor_from_control_plant

    plant = HummingbirdPseudo6DOFControlPlant()
    descriptor = descriptor_from_control_plant(
        plant,
        family_id="hummingbird",
        adapter_id="taoryx.multirotor.aggregate_thrust.pseudo_tuning.v1",
        physical_family="multirotor",
        tier="pseudo_6dof",
    )
    return StandardFamilyAdapter.from_control_plant(descriptor, plant)
    ####


def _hummingbird_source_rotor_tuning_adapter() -> StandardFamilyAdapter:
    """Build the exact source-hover attitude/wrench runtime projection."""

    from taoryx.source_table_multirotor import build_hummingbird_local_physical_wrench_lqi_design

    from taoryx.physical_wrench_tuning import build_projected_physical_wrench_tuning_adapter

    return build_projected_physical_wrench_tuning_adapter(
        build_hummingbird_local_physical_wrench_lqi_design(),
        family_id="hummingbird",
        adapter_id="taoryx.multirotor.native_quad_x.attitude_wrench_tuning.v1",
        physical_family="multirotor",
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
            "Hummingbird source-hover attitude/rate physical-wrench projection; nonlinear execution allocates each "
            "requested moment through bounded individual rotors with motor lag."
        ),
        omitted_physics=(
            "motor-coordinate campaign synthesis",
            "wind, battery, landing, gain scheduling, and qualification",
        ),
    )
    ####


def _hummingbird_source_rotor_vertical_tuning_adapter() -> StandardFamilyAdapter:
    """Build the exact source-hover vertical-force runtime projection."""

    from taoryx.source_table_multirotor import build_hummingbird_local_vertical_force_lqi_design

    from taoryx.physical_wrench_tuning import build_projected_physical_wrench_tuning_adapter

    return build_projected_physical_wrench_tuning_adapter(
        build_hummingbird_local_vertical_force_lqi_design(),
        family_id="hummingbird",
        adapter_id="taoryx.multirotor.native_quad_x.vertical_wrench_tuning.v1",
        physical_family="multirotor",
        tier="rigid_body_6dof_surface_allocated",
        state_units={
            "roll_error_rad": "rad",
            "pitch_error_rad": "rad",
            "yaw_error_rad": "rad",
            "w_m_s": "m/s",
            "p_rad_s": "rad/s",
            "q_rad_s": "rad/s",
            "r_rad_s": "rad/s",
        },
        validity_envelope=(
            "Hummingbird source-hover vertical-speed/attitude physical-wrench projection; nonlinear execution "
            "allocates collective force and moments through bounded individual rotors with motor lag."
        ),
        omitted_physics=(
            "motor-coordinate campaign synthesis",
            "wind, battery, landing, gain scheduling, and qualification",
        ),
    )
    ####


def _rotor_effectors() -> list[dict[str, object]]:
    return [
        {
            "id": f"effector.rotor.{index}.speed.position",
            "native_control_id": f"rotor-{index}-speed",
            "unit": "rad/s",
            "lower": 0.0,
            "upper": 1500.0,
            "time_constant_s": 0.005,
        }
        for index in range(1, 5)
    ]
    ####


def _campaigns() -> tuple[ControllerTuningCampaignRegistration, ...]:
    """Declare Hummingbird campaigns for its local and aggregate providers."""

    aliases = (_AGGREGATE_PROVIDER_ID,)
    return (
        ControllerTuningCampaignRegistration(
            id="hummingbird-pseudo-hover-attitude-v1",
            provider_id=_PROVIDER_ID,
            provider_aliases=aliases,
            model_id="hummingbird",
            family_id="hummingbird",
            fidelity="pseudo_6dof",
            realization_ids=("pseudo_6dof",),
            mission_template_ids=("multirotor_pad_box_yaw_recovery_land_v1",),
            description="Scaled LQI hover-attitude inner-loop campaign over the declared aggregate-thrust response model.",
            adapter_factory=_hummingbird_pseudo_tuning_adapter,
            campaign_factory=_build_hummingbird_pseudo_tuning_campaign,
        ),
        ControllerTuningCampaignRegistration(
            id="hummingbird-source-rotor-local-lqi-v1",
            provider_id=_PROVIDER_ID,
            provider_aliases=aliases,
            model_id="hummingbird",
            family_id="hummingbird",
            fidelity="rigid_body_6dof_surface_allocated",
            realization_ids=("rigid_body_6dof_surface_allocated",),
            mission_template_ids=(
                "hummingbird_local_individual_rotor_lqi_screen_v1",
                "hummingbird_local_horizontal_translation_lqi_screen_v1",
            ),
            description=(
                "Exact scaled LQI candidate for the Hummingbird source-hover attitude/rate physical-wrench runtime; "
                "each nonlinear command remains allocated through bounded individual rotors."
            ),
            adapter_factory=_hummingbird_source_rotor_tuning_adapter,
            campaign_factory=_build_hummingbird_source_rotor_lqi_tuning_campaign,
            local_controller_screens=(
                {
                    "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
                    "id": "hummingbird-source-hover-individual-rotor-lqi-screen-v1",
                    "mission_template_id": "hummingbird_local_individual_rotor_lqi_screen_v1",
                    "fidelity": "rigid_body_6dof_surface_allocated",
                    "operations": ["batch"],
                    "control_realization": "individual_rotor_source_lqi_allocation",
                    "physical_effector_allocation": True,
                    "batch_action_trace": "emits_committed_interval_trace",
                    "controller": {
                        "method": "lqi",
                        "campaign_id": "hummingbird-source-rotor-local-lqi-v1",
                        "integral_output_names": ["roll_error_rad", "pitch_error_rad", "yaw_error_rad"],
                        "fixed_cadence_s": 0.01,
                        "screen_duration_s": 1.0,
                    },
                    "effector_controls": _rotor_effectors(),
                    "claim_boundary": (
                        "The exact batch screen allocates internal LQI moment requests to four source rotor-speed "
                        "coordinates with declared lag. It is not an externally commanded rotor session, position, wind, "
                        "battery, landing, gain-schedule, or vehicle-qualification endpoint."
                    ),
                },
                {
                    "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
                    "id": "hummingbird-source-horizontal-translation-individual-rotor-lqi-screen-v1",
                    "mission_template_id": "hummingbird_local_horizontal_translation_lqi_screen_v1",
                    "fidelity": "rigid_body_6dof_surface_allocated",
                    "operations": ["batch"],
                    "control_realization": "individual_rotor_source_lqi_allocation",
                    "physical_effector_allocation": True,
                    "batch_action_trace": "emits_committed_interval_trace",
                    "controller": {
                        "method": "lqi",
                        "campaign_id": "hummingbird-source-rotor-local-lqi-v1",
                        "integral_output_names": ["roll_error_rad", "pitch_error_rad", "yaw_error_rad"],
                        "fixed_cadence_s": 0.02,
                        "screen_duration_s": 38.0,
                        "reference_layer": "bounded_horizontal_position_error_to_tilt_yaw_reference",
                    },
                    "effector_controls": _rotor_effectors(),
                    "claim_boundary": (
                        "The exact batch screen allocates bounded local horizontal position-error/tilt/yaw LQI requests "
                        "to four source rotor-speed coordinates with declared lag. It does not establish altitude, wind, "
                        "battery, contact, landing, gain-schedule, or vehicle qualification."
                    ),
                },
            ),
            claim_boundary=(
                "This campaign binds the source-hover attitude/rate physical-wrench LQI runtime to a common candidate. "
                "It does not establish wind/battery robustness, contact or landing behavior, gain scheduling, or flight qualification."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="hummingbird-source-rotor-vertical-lqi-v1",
            provider_id=_PROVIDER_ID,
            provider_aliases=aliases,
            model_id="hummingbird",
            family_id="hummingbird",
            fidelity="rigid_body_6dof_surface_allocated",
            realization_ids=("rigid_body_6dof_surface_allocated",),
            mission_template_ids=("hummingbird_local_vertical_translation_lqi_screen_v1",),
            description=(
                "Exact scaled LQI candidate for Hummingbird's source-hover vertical-speed/attitude physical-wrench "
                "runtime; collective force and moments remain allocated through bounded individual rotors."
            ),
            adapter_factory=_hummingbird_source_rotor_vertical_tuning_adapter,
            campaign_factory=_build_hummingbird_source_rotor_vertical_lqi_tuning_campaign,
            local_controller_screens=(
                {
                    "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
                    "id": "hummingbird-source-vertical-translation-individual-rotor-lqi-screen-v1",
                    "mission_template_id": "hummingbird_local_vertical_translation_lqi_screen_v1",
                    "fidelity": "rigid_body_6dof_surface_allocated",
                    "operations": ["batch"],
                    "control_realization": "individual_rotor_source_lqi_force_moment_allocation",
                    "physical_effector_allocation": True,
                    "batch_action_trace": "emits_committed_interval_trace",
                    "controller": {
                        "method": "lqi",
                        "campaign_id": "hummingbird-source-rotor-vertical-lqi-v1",
                        "integral_output_names": ["roll_error_rad", "pitch_error_rad", "yaw_error_rad", "w_m_s"],
                        "fixed_cadence_s": 0.02,
                        "screen_duration_s": 16.0,
                        "reference_layer": "bounded_down_position_error_to_vertical_speed_reference",
                        "controlled_wrench_axes": ["force_z_n", "moment_x_nm", "moment_y_nm", "moment_z_nm"],
                    },
                    "effector_controls": _rotor_effectors(),
                    "claim_boundary": (
                        "The exact batch screen allocates bounded local down-position/vertical-speed LQI collective-force "
                        "and attitude requests to four source rotor-speed coordinates with declared lag. It does not establish "
                        "wind, battery, contact, landing, gain-schedule, or vehicle qualification."
                    ),
                },
            ),
            claim_boundary=(
                "This campaign binds only the source-hover vertical-speed/attitude physical-wrench LQI runtime to a "
                "common candidate. It does not establish wind, battery, landing, gain scheduling, or qualification."
            ),
        ),
    )
    ####


def _physical_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Run one Hummingbird source physical screen with its selected candidate."""

    from taoryx.hummingbird_local_physical_control_screen import execute_hummingbird_local_physical_control_screen

    return execute_hummingbird_local_physical_control_screen(
        request.composition,
        request.output_dir,
        max_steps=request.max_steps,
        tuning_context=request.tuning_context,
    )
    ####


def _direct_wrench_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Run the family-local direct-wrench comparator through shared host code."""

    from taoryx.local_direct_wrench_composition_execution import execute_local_direct_wrench_composition

    if request.max_steps is not None:
        raise ValueError("--max-steps is not available for the local direct-wrench screen")
    return execute_local_direct_wrench_composition(
        request.composition,
        request.output_dir,
        tuning_context=request.tuning_context,
    )
    ####


def _pseudo_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Load and run the aggregate pseudo-6DOF runtime only for batch execution."""

    from taoryx.hummingbird_composition_execution import execute_hummingbird_pseudo_composition

    if request.max_steps is not None:
        raise ValueError("--max-steps is not available for the aggregate-thrust pseudo batch")
    return execute_hummingbird_pseudo_composition(request.composition, request.output_dir)
    ####


def _provider() -> CatalogMissionCompositionProvider:
    """Build a Hummingbird-only portable composition provider.

    The deferred plug-in proxy caches this object per discovery catalog.  Do
    not add a module-global cache here: an aggregate host must not make a
    later focused host inherit its broader execution scope.
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
    from taoryx_hummingbird.resources import model_resource_root

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
        provider_name="TAORYX Hummingbird Mission Composition",
        provider_short_name="Hummingbird",
        provider_summary="Hummingbird-only model, control, and mission configuration surface.",
        provider_version=_PACKAGE_VERSION,
        plugin_catalog=current_plugin_catalog(),
        include_builtin_workflows=False,
    )
    ####


def _register(registrar: PluginRegistrar) -> None:
    """Publish only Hummingbird-owned data, controls, and runtime factories."""

    registrar.register_vehicle_catalog_fragment(_CATALOG_FRAGMENT)
    from taoryx.hummingbird_local_direct_wrench import hummingbird_local_direct_wrench_screen_definition

    registration = registrar.register_family_adapter_factory(_FAMILY_ID, _hummingbird_family_adapter_registration)
    registrar.register_model(_FAMILY_ID, registration)
    registrar.register_vehicle_interface_extension_factory(_FAMILY_ID, _reduced_interface_extension)
    registrar.register_mission_composition_provider_factory(_PROVIDER_ID, _provider)
    direct_wrench_screen = hummingbird_local_direct_wrench_screen_definition()
    registrar.register_local_direct_wrench_screen_definition(direct_wrench_screen)
    registrar.register_mission_capability_adapter(LocalDirectWrenchScreenCapabilityAdapter(direct_wrench_screen))
    for adapter in (
        _LazyMissionCapabilityAdapter(
            "taoryx.multirotor_hover_translation.capability.v1",
            _hummingbird_hover_capability_adapter,
        ),
        _LazyMissionCapabilityAdapter(
            "taoryx.hummingbird.local_individual_rotor_lqi_screen.capability.v1",
            _hummingbird_local_physical_capability_adapter,
        ),
        _LazyMissionCapabilityAdapter(
            "taoryx.hummingbird.local_horizontal_translation_lqi_screen.capability.v1",
            _hummingbird_local_horizontal_capability_adapter,
        ),
        _LazyMissionCapabilityAdapter(
            "taoryx.hummingbird.local_vertical_translation_lqi_screen.capability.v1",
            _hummingbird_local_vertical_capability_adapter,
        ),
    ):
        registrar.register_mission_capability_adapter(adapter)
    for campaign in _campaigns():
        registrar.register_controller_tuning_campaign(campaign)
    for translator_id, handler in (
        (
            "taoryx.hummingbird.local_individual_rotor_lqi_screen.capability.v1",
            _preflight_hummingbird_local_physical_control_screen,
        ),
        (
            "taoryx.hummingbird.local_horizontal_translation_lqi_screen.capability.v1",
            _preflight_hummingbird_local_physical_control_screen,
        ),
        (
            "taoryx.hummingbird.local_vertical_translation_lqi_screen.capability.v1",
            _preflight_hummingbird_local_physical_control_screen,
        ),
        (
            "taoryx.hummingbird.local_direct_wrench_screen.capability.v1",
            _preflight_hummingbird_local_direct_wrench,
        ),
        (
            "taoryx.hummingbird.hover_yaw_contact.pseudo6dof.v1",
            _preflight_hummingbird_hover_yaw,
        ),
    ):
        registrar.register_semantic_preflight_handler_callback(translator_id, handler)
    registrar.register_episode_factory("hummingbird_aggregate_thrust_episode.v1", _open_hummingbird_aggregate_thrust_episode)
    registrar.register_batch_episode_parity_verifier(
        "taoryx.hummingbird.aggregate_thrust_batch_episode_parity.v1",
        _verify_hummingbird_batch_episode_parity,
    )
    registrar.register_execution_factory_request_v1("hummingbird_local_individual_rotor_lqi_screen.v1", _physical_batch)
    registrar.register_execution_factory_request_v1("hummingbird_local_horizontal_translation_lqi_screen.v1", _physical_batch)
    registrar.register_execution_factory_request_v1("hummingbird_local_vertical_translation_lqi_screen.v1", _physical_batch)
    registrar.register_execution_factory_request_v1("hummingbird_local_direct_wrench_screen.v1", _direct_wrench_batch)
    registrar.register_execution_factory_request_v1("hummingbird_aggregate_thrust_pseudo_batch.v1", _pseudo_batch)
    ####


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="taoryx.hummingbird",
        package="taoryx-hummingbird",
        version=_PACKAGE_VERSION,
        api_version="1",
        description="AscTec Hummingbird multirotor model, controls, and composition runtime.",
    ),
    register_callback=_register,
)

__all__ = ["PLUGIN"]
