"""Plug-in registration for the source-backed HL-20 Mod K family."""

from __future__ import annotations

import math
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

_FAMILY_ID = "hl20_mod_k"
_TRIM_EVIDENCE_FAMILY_ID = "reference_hl20_mod_k"
_PROVIDER_ID = "taoryx.hl20.mission-composition"
_AGGREGATE_PROVIDER_ID = "taoryx.registry.mission-composition"
_PACKAGE_VERSION = "0.1.0a0"
_DIRECT_LQR_TRANSLATOR_ID = "taoryx.hl20_local_direct_wrench_screen.capability.v1"
_DIRECT_LQI_TRANSLATOR_ID = "taoryx.hl20_local_direct_wrench_lqi_screen.capability.v1"
_SURFACE_AUTHORITY_TRANSLATOR_ID = "taoryx.hl20_source_surface_pitch_authority_screen.capability.v1"
_SURFACE_LQI_TRANSLATOR_ID = "taoryx.hl20_source_surface_attitude_rate_lqi_screen.capability.v1"
_CATALOG_FRAGMENT = VehicleCatalogFragment(
    id="taoryx.hl20.vehicle-catalog",
    resource_package="taoryx_hl20",
    family_ids=(_FAMILY_ID,),
)
_FOCUSED_MISSION_TEMPLATE_IDS = (
    "hl20_source_surface_pitch_authority_screen_v1",
    "hl20_source_surface_attitude_rate_lqi_screen_v1",
    "hl20_local_direct_wrench_screen_v1",
    "hl20_local_direct_wrench_lqi_screen_v1",
)
_SURFACE_BOUNDS_DEG = (
    ("upper_left_body_flap", -60.0, 0.0),
    ("lower_left_body_flap", 0.0, 60.0),
    ("upper_right_body_flap", -60.0, 0.0),
    ("lower_right_body_flap", 0.0, 60.0),
    ("left_wing_flap", -30.0, 30.0),
    ("right_wing_flap", -30.0, 30.0),
    ("rudder", -30.0, 30.0),
)


class _LazyMissionCapabilityAdapter:
    """Expose one HL-20 capability identity without building its source plant."""

    def __init__(self, identifier: str, factory: Callable[[], object]) -> None:
        self.id = identifier
        self._factory = factory
        self._delegate: object | None = None
        ####

    def _resolve(self) -> Any:
        if self._delegate is None:
            candidate = self._factory()
            if getattr(candidate, "id", None) != self.id:
                raise ValueError(f"lazy HL-20 capability adapter {self.id!r} resolved a mismatched identity")
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
    """Return the HL-20 source adapter registration without loading data."""

    from taoryx.hl20_adapter import build_hl20_source_adapter

    from taoryx.family_adapter_registry import FamilyAdapterRegistration
    return FamilyAdapterRegistration(
        _FAMILY_ID,
        "taoryx.lifting_body.daveml.v1",
        "available",
        build_hl20_source_adapter,
        probe_factory=_hl20_source_probe,
        supported_tiers=("rigid_body_6dof_direct_wrench", "rigid_body_6dof_surface_allocated"),
        note="local source load/surface witnesses; mission translation remains pending",
    )
    ####


def _trim_evidence_binding() -> object:
    """Load the HL-20 source trim binding only when trim evidence is selected."""

    from taoryx.hl20_trim_evidence import trim_evidence_binding

    return trim_evidence_binding()
    ####


def _hl20_source_probe(adapter: StandardFamilyAdapter) -> AdapterProbeCase:
    """Return the documented source-local condition for HL-20 operation probes."""

    from taoryx.family_adapter_probes import AdapterProbeCase

    if adapter.describe().tier == "rigid_body_6dof_surface_allocated":
        from taoryx.hl20_adapter import build_hl20_source_surface_local_plant

        plant = build_hl20_source_surface_local_plant()
        state = dict(plant.reference_state)
        effectors = dict(plant.reference_effectors)
        return AdapterProbeCase(
            state=state,
            effectors=effectors,
            trim_target=state,
            trim_initial_guess=effectors,
            previous_effectors=effectors,
        )

    speed = 340.294
    alpha_rad = math.radians(5.0)
    state = {
        "u_m_s": speed * math.cos(alpha_rad),
        "v_m_s": 0.0,
        "w_m_s": -speed * math.sin(alpha_rad),
        "p_rad_s": 0.0,
        "q_rad_s": 0.0,
        "r_rad_s": 0.0,
        "altitude_m": 0.0,
    }
    effectors = {name: 0.0 for name in adapter.control_names}
    return AdapterProbeCase(
        state=state,
        effectors=effectors,
        trim_target=state,
        trim_initial_guess=effectors,
        previous_effectors=effectors,
    )
    ####


def _direct_wrench_tuning_adapter() -> StandardFamilyAdapter:
    """Build the fixed-condition HL-20 source direct-wrench tuning bridge."""

    from taoryx.hl20_adapter import build_hl20_source_direct_wrench_tuning_adapter

    return build_hl20_source_direct_wrench_tuning_adapter()
    ####


def _surface_lqi_tuning_adapter() -> StandardFamilyAdapter:
    """Build the exact HL-20 physical-wrench runtime projection for tuning."""

    from taoryx.hl20_adapter import build_hl20_source_surface_physical_lqi_design

    from taoryx.physical_wrench_tuning import build_projected_physical_wrench_tuning_adapter

    return build_projected_physical_wrench_tuning_adapter(
        build_hl20_source_surface_physical_lqi_design(),
        family_id=_FAMILY_ID,
        adapter_id="taoryx.hl20.source_surface.physical_wrench_tuning.v1",
        physical_family="lifting_body",
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
            "HL-20 frozen Mach-1 source-fixture local attitude/rate physical-wrench projection; nonlinear execution "
            "allocates each requested moment through seven bounded source surfaces"
        ),
        omitted_physics=(
            "raw source-surface campaign synthesis",
            "nonlinear surface allocation in the campaign adapter",
            "full glide trim, translation, guidance, and qualification",
        ),
    )
    ####


def _direct_wrench_lqr_campaign() -> TuningCampaign:
    from taoryx.hl20_adapter import build_hl20_direct_wrench_tuning_campaign

    return build_hl20_direct_wrench_tuning_campaign()
    ####


def _direct_wrench_lqi_campaign() -> TuningCampaign:
    from taoryx.hl20_adapter import build_hl20_direct_wrench_lqi_tuning_campaign

    return build_hl20_direct_wrench_lqi_tuning_campaign()
    ####


def _surface_lqi_campaign() -> TuningCampaign:
    from taoryx.hl20_adapter import build_hl20_source_surface_lqi_tuning_campaign

    return build_hl20_source_surface_lqi_tuning_campaign()
    ####


def _campaigns() -> tuple[ControllerTuningCampaignRegistration, ...]:
    """Return HL-20-only campaign metadata and deferred numerical factories."""

    return (
        ControllerTuningCampaignRegistration(
            id="hl20-source-subsonic-direct-wrench-v1",
            provider_id=_PROVIDER_ID,
            provider_aliases=(_AGGREGATE_PROVIDER_ID,),
            model_id=_FAMILY_ID,
            family_id=_FAMILY_ID,
            fidelity="rigid_body_6dof_direct_wrench",
            realization_ids=("rigid_body_6dof_direct_wrench",),
            mission_template_ids=("hl20_local_direct_wrench_screen_v1",),
            description="Scaled LQR local source-load cancellation campaign over the HL-20 declared direct-wrench bridge.",
            adapter_factory=_direct_wrench_tuning_adapter,
            campaign_factory=_direct_wrench_lqr_campaign,
            claim_boundary=(
                "This campaign screens bounded generalized direct-wrench feedback at the documented HL-20 local source condition. "
                "It does not synthesize or qualify physical HL-20 surface allocation, navigation, or a flight envelope."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="hl20-source-subsonic-direct-wrench-lqi-v1",
            provider_id=_PROVIDER_ID,
            provider_aliases=(_AGGREGATE_PROVIDER_ID,),
            model_id=_FAMILY_ID,
            family_id=_FAMILY_ID,
            fidelity="rigid_body_6dof_direct_wrench",
            realization_ids=("rigid_body_6dof_direct_wrench",),
            mission_template_ids=("hl20_local_direct_wrench_lqi_screen_v1",),
            description="Scaled LQI local body-speed campaign over the HL-20 declared direct-wrench bridge.",
            adapter_factory=_direct_wrench_tuning_adapter,
            campaign_factory=_direct_wrench_lqi_campaign,
            claim_boundary=(
                "This campaign screens bounded generalized direct-wrench feedback at the documented HL-20 local source condition. "
                "It does not synthesize or qualify physical HL-20 surface allocation, navigation, or a flight envelope."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="hl20-source-surface-local-lqi-v1",
            provider_id=_PROVIDER_ID,
            provider_aliases=(_AGGREGATE_PROVIDER_ID,),
            model_id=_FAMILY_ID,
            family_id=_FAMILY_ID,
            fidelity="rigid_body_6dof_surface_allocated",
            realization_ids=("rigid_body_6dof_surface_allocated",),
            mission_template_ids=("hl20_source_surface_attitude_rate_lqi_screen_v1",),
            description=(
                "Exact scaled LQI candidate for the frozen-translation HL-20 attitude/rate physical-wrench runtime; "
                "each nonlinear moment request remains allocated through seven bounded source surfaces."
            ),
            adapter_factory=_surface_lqi_tuning_adapter,
            campaign_factory=_surface_lqi_campaign,
            local_controller_screens=(
                {
                    "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
                    "id": "hl20-source-mach1-surface-attitude-rate-lqi-screen-v1",
                    "mission_template_id": "hl20_source_surface_attitude_rate_lqi_screen_v1",
                    "fidelity": "rigid_body_6dof_surface_allocated",
                    "operations": ["validate", "batch"],
                    "control_realization": "source_surface_physical_wrench_lqi_allocation",
                    "physical_effector_allocation": True,
                    "batch_action_trace": "emits_committed_interval_trace",
                    "controller": {
                        "method": "lqi",
                        "campaign_id": "hl20-source-surface-local-lqi-v1",
                        "integral_output_names": ["roll_error_rad", "pitch_error_rad", "yaw_error_rad"],
                        "integral_weight_multiplier": 1000.0,
                        "physical_wrench_profile": {
                            "integral_q_diagonal": [1000.0, 1000.0, 1000.0],
                            "selection_evidence": "matched external pitch-moment offset screen emitted by this batch",
                        },
                        "persistent_disturbance_screen": {
                            "id": "hl20-local-lqi-matched-pitch-wrench-offset",
                            "environment_input": "external_pitch_moment_bias_nm",
                            "body_moment_axis": "moment_y_nm",
                            "fraction_of_declared_pitch_wrench_scale": 0.05,
                            "artifact_filename": "robustness_report.json",
                        },
                        "fixed_cadence_s": 0.01,
                        "screen_duration_s": 8.0,
                    },
                    "effector_controls": [
                        {
                            "id": f"effector.surface.{name}.position",
                            "native_control_id": name,
                            "unit": "deg",
                            "lower": lower,
                            "upper": upper,
                        }
                        for name, lower, upper in _SURFACE_BOUNDS_DEG
                    ],
                    "claim_boundary": (
                        "This source-surface LQI screen holds the HL-20 Mach-1 source translational fixture fixed and controls "
                        "only local attitude error/body rate through all seven bounded named surfaces. It also emits a bounded "
                        "three-case matched external pitch-moment screen. It does not establish full-state trim, translation, "
                        "glide guidance, navigation, wind or mass robustness, or flight qualification."
                    ),
                },
            ),
            claim_boundary=(
                "This campaign auto-tunes only the frozen-translation HL-20 local attitude/rate source-surface model. "
                "The associated batch emits one bounded matched external pitch-moment screen; it does not establish a "
                "source-full-glide trim, gain schedule, wind or mass robustness, guidance, or qualification."
            ),
        ),
    )
    ####


def _surface_authority_capability_adapter() -> object:
    from taoryx.hl20_surface_authority_screen import HL20SourceSurfacePitchAuthorityScreenCapabilityAdapter

    return HL20SourceSurfacePitchAuthorityScreenCapabilityAdapter()
    ####


def _surface_lqi_capability_adapter() -> object:
    from taoryx.hl20_local_physical_surface_lqi_screen import HL20LocalPhysicalSurfaceLqiScreenCapabilityAdapter

    return HL20LocalPhysicalSurfaceLqiScreenCapabilityAdapter()
    ####


def _preflight_direct_wrench(composition: CompiledVehicleComposition) -> Any:
    from taoryx.vehicle_execution_preflight import _preflight_local_direct_wrench

    return _preflight_local_direct_wrench(composition)
    ####


def _preflight_surface_authority(composition: CompiledVehicleComposition) -> Any:
    from taoryx.hl20_surface_authority_screen import preflight_hl20_source_surface_authority_screen

    return preflight_hl20_source_surface_authority_screen(composition)
    ####


def _preflight_surface_lqi(composition: CompiledVehicleComposition) -> Any:
    from taoryx.hl20_local_physical_surface_lqi_screen import preflight_hl20_local_physical_surface_lqi_screen

    return preflight_hl20_local_physical_surface_lqi_screen(composition)
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
        raise ValueError("--max-steps is not available for the HL-20 local direct-wrench screen")
    return execute_local_direct_wrench_composition(
        request.composition,
        request.output_dir,
        tuning_context=request.tuning_context,
    )
    ####


def _surface_authority_batch(request: VehicleBatchExecutionRequest) -> Any:
    from taoryx.hl20_surface_authority_screen import execute_hl20_source_surface_authority_screen

    if request.tuning_context is not None:
        raise ValueError("the HL-20 source-surface authority screen does not accept a controller tuning context")
    return execute_hl20_source_surface_authority_screen(
        request.composition,
        request.output_dir,
        max_steps=request.max_steps,
    )
    ####


def _surface_lqi_batch(request: VehicleBatchExecutionRequest) -> Any:
    from taoryx.hl20_local_physical_surface_lqi_screen import execute_hl20_local_physical_surface_lqi_screen

    return execute_hl20_local_physical_surface_lqi_screen(
        request.composition,
        request.output_dir,
        max_steps=request.max_steps,
        tuning_context=request.tuning_context,
    )
    ####


def _provider() -> CatalogMissionCompositionProvider:
    """Build the portable HL-20-only Mission Composition provider.

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
        provider_name="TAORYX HL-20 Mission Composition",
        provider_short_name="HL-20",
        provider_summary="Source-backed HL-20 local direct-wrench and seven-surface composition endpoints.",
        provider_version=_PACKAGE_VERSION,
        plugin_catalog=current_plugin_catalog(),
        include_builtin_workflows=False,
        allowed_mission_template_ids={_FAMILY_ID: _FOCUSED_MISSION_TEMPLATE_IDS},
    )
    ####


def _register(registrar: PluginRegistrar) -> None:
    """Publish only HL-20-owned models, local screens, and package data seams."""

    registrar.register_vehicle_catalog_fragment(_CATALOG_FRAGMENT)
    from taoryx.hl20_local_direct_wrench import hl20_local_direct_wrench_screen_definitions

    registration = registrar.register_family_adapter_factory(_FAMILY_ID, _family_adapter_registration)
    registrar.register_model(_FAMILY_ID, registration)
    registrar.register_trim_evidence_binding_factory(_TRIM_EVIDENCE_FAMILY_ID, _trim_evidence_binding)
    registrar.register_mission_composition_provider_factory(_PROVIDER_ID, _provider)
    for definition in hl20_local_direct_wrench_screen_definitions():
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
    registrar.register_episode_factory("hl20_local_direct_wrench_episode.v1", _open_direct_wrench_episode)
    registrar.register_batch_episode_parity_verifier(
        "taoryx.hl20.local_direct_wrench_batch_episode_parity.v1",
        _verify_direct_wrench_batch_episode_parity,
    )
    registrar.register_execution_factory_request_v1("hl20_local_direct_wrench_screen.v1", _direct_wrench_batch)
    registrar.register_execution_factory_request_v1("hl20_source_surface_pitch_authority_screen.v1", _surface_authority_batch)
    registrar.register_execution_factory_request_v1("hl20_local_physical_surface_lqi_screen.v1", _surface_lqi_batch)
    ####


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="taoryx.hl20",
        package="taoryx-hl20",
        version=_PACKAGE_VERSION,
        api_version="1",
        description="Source-backed HL-20 local control screens, DAVE-ML source fixture, and Mission Composition provider.",
    ),
    register_callback=_register,
)

__all__ = ["PLUGIN"]
