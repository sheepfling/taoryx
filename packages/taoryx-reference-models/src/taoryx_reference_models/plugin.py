"""Plug-in registration for every shipped reference vehicle family."""

from __future__ import annotations

import math
from collections.abc import Callable
from functools import lru_cache
from pathlib import Path
from typing import Any

from taoryx.b747_local_physical_control_screen import (
    B747LocalPhysicalControlScreenCapabilityAdapter,
    B747LocalPhysicalLqiControlScreenCapabilityAdapter,
    execute_b747_condition3_local_physical_control_screen,
    preflight_b747_condition3_local_physical_control_screen,
)
from taoryx.composition_batch_episode_parity import verify_serialized_composition_batch_episode_parity
from taoryx.f16_local_physical_control_screen import (
    F16LocalPhysicalControlScreenCapabilityAdapter,
    execute_f16_local_physical_control_screen,
    preflight_f16_local_physical_control_screen,
)
from taoryx.f16_local_physical_lqi_screen import (
    F16LocalPhysicalLqiScreenCapabilityAdapter,
    execute_f16_local_physical_lqi_screen,
    preflight_f16_local_physical_lqi_screen,
)
from taoryx.f16_physical_schedule_interior_screen import (
    F16PhysicalScheduleInteriorScreenCapabilityAdapter,
    F16PhysicalScheduleLqiInteriorScreenCapabilityAdapter,
    execute_f16_physical_schedule_interior_screen,
    preflight_f16_physical_schedule_interior_screen,
)
from taoryx.f16_physical_schedule_transition_screen import (
    F16PhysicalScheduleTransitionScreenCapabilityAdapter,
    execute_f16_physical_schedule_transition_screen,
    preflight_f16_physical_schedule_transition_screen,
)
from taoryx.hl20_adapter import (
    build_hl20_direct_wrench_lqi_tuning_campaign,
    build_hl20_direct_wrench_tuning_campaign,
    build_hl20_source_adapter,
    build_hl20_source_direct_wrench_tuning_adapter,
    build_hl20_source_surface_local_plant,
    build_hl20_source_surface_lqi_tuning_campaign,
    build_hl20_source_surface_physical_lqi_design,
)
from taoryx.hl20_controls import HL20_SOURCE_SURFACE_BOUNDS_DEG
from taoryx.hl20_local_physical_surface_lqi_screen import (
    HL20LocalPhysicalSurfaceLqiScreenCapabilityAdapter,
    execute_hl20_local_physical_surface_lqi_screen,
    preflight_hl20_local_physical_surface_lqi_screen,
)
from taoryx.hl20_surface_authority_screen import (
    HL20SourceSurfacePitchAuthorityScreenCapabilityAdapter,
    execute_hl20_source_surface_authority_screen,
    preflight_hl20_source_surface_authority_screen,
)
from taoryx.hummingbird_composition_execution import execute_hummingbird_pseudo_composition
from taoryx.hummingbird_local_physical_control_screen import (
    HummingbirdLocalHorizontalTranslationLqiScreenCapabilityAdapter,
    HummingbirdLocalPhysicalControlScreenCapabilityAdapter,
    HummingbirdLocalVerticalTranslationLqiScreenCapabilityAdapter,
    execute_hummingbird_local_physical_control_screen,
    preflight_hummingbird_local_physical_control_screen,
)
from taoryx.nesc_adapter import build_nesc_replay_adapter
from taoryx.nesc_composition_execution import execute_nesc_source_replay_composition
from taoryx.reduced_fixed_wing_batch_episode_parity import verify_serialized_reduced_fixed_wing_batch_episode_parity
from taoryx.reduced_fixed_wing_execution import _f16_source_trim, execute_reduced_fixed_wing_composition
from taoryx.reference_mission_capability import reference_mission_capability_adapters
from taoryx.source_f16 import (
    build_f16_local_physical_wrench_lqi_design,
    build_f16_source_physical_plant,
    build_f16_source_physical_schedule_lqi_nodes,
    build_f16_source_physical_schedule_lqi_tuning_campaign,
    build_f16_source_physical_schedule_lqr_tuning_campaign,
    build_f16_source_physical_schedule_nodes,
    build_f16_source_surface_lqi_tuning_campaign,
    build_f16_source_surface_lqr_tuning_campaign,
    f16_source_physical_schedule_tuning_targets,
)
from taoryx.source_table_fixed_wing import (
    build_b747_condition3_source_surface_physical_lqi_design,
    build_b747_condition3_source_table_plant,
    build_b747_source_surface_lqi_tuning_campaign,
    build_x8_source_surface_lqi_tuning_campaign,
    build_x8_source_surface_physical_lqi_design,
    build_x8_source_table_plant,
)
from taoryx.source_table_multirotor import (
    build_hummingbird_individual_rotor_source_table_plant,
    build_hummingbird_local_physical_wrench_lqi_design,
    build_hummingbird_local_vertical_force_lqi_design,
    build_hummingbird_source_rotor_lqi_tuning_campaign,
    build_hummingbird_source_rotor_vertical_lqi_tuning_campaign,
)
from taoryx.trajectory.a320_adapter import (
    A320OpenAPControlPlant,
    A320OpenAPModel,
    A320Pseudo6DOFControlPlant,
    build_a320_point_tuning_campaign,
    build_a320_pseudo_control_adapter,
    build_a320_pseudo_tuning_campaign,
)
from taoryx.trajectory.a320_pseudo6dof import A320Pseudo6DOFModel
from taoryx.trajectory.f16_reduced_adapter import (
    build_f16_point_tuning_campaign,
    build_f16_pseudo_tuning_campaign,
    build_f16_reduced_control_plant,
)
from taoryx.trajectory.f16_reductions import F16AttitudeResponsePseudo6DOFModel, F16PointMass3DOFModel
from taoryx.trajectory.hummingbird_adapter import (
    HummingbirdPseudo6DOFControlPlant,
    build_hummingbird_pseudo_tuning_campaign,
)
from taoryx.trajectory.reference_mission_composition import ReferenceMissionCompositionProvider
from taoryx.trajectory.registry_mission_composition import RegistryMissionCompositionProvider
from taoryx.x8_local_physical_control_screen import (
    X8LocalPhysicalControlScreenCapabilityAdapter,
    X8LocalPhysicalLongRecoveryLqiScreenCapabilityAdapter,
    X8LocalPhysicalLqiControlScreenCapabilityAdapter,
    execute_x8_local_physical_control_screen,
    preflight_x8_local_physical_control_screen,
)
from taoryx.x15_adapter import (
    build_x15_direct_wrench_lqi_tuning_campaign,
    build_x15_direct_wrench_tuning_campaign,
    build_x15_source_direct_wrench_adapter,
    build_x15_source_direct_wrench_plant,
    build_x15_source_surface_lqi_tuning_campaign,
    build_x15_source_surface_physical_lqi_design,
)
from taoryx.x15_local_physical_surface_lqi_screen import (
    X15LocalPhysicalSurfaceLqiScreenCapabilityAdapter,
    execute_x15_local_physical_surface_lqi_screen,
    preflight_x15_local_physical_surface_lqi_screen,
)
from taoryx.x15_surface_authority_screen import (
    X15SourceSurfaceAuthorityScreenCapabilityAdapter,
    execute_x15_source_surface_authority_screen,
    preflight_x15_source_surface_authority_screen,
)

from taoryx.composition_episode import (
    _open_a320_reduced_episode,
    _open_f16_reduced_episode,
    _open_hummingbird_episode,
    _open_language_backed_episode,
    _open_local_direct_wrench_episode,
)
from taoryx.controller_tuning_registry import ControllerTuningCampaignRegistration
from taoryx.family_adapter import (
    AdapterChannel,
    FamilyAdapterDescriptor,
    StandardFamilyAdapter,
    descriptor_from_control_plant,
)
from taoryx.family_adapter_probes import AdapterProbeCase
from taoryx.family_adapter_registry import FamilyAdapterRegistration
from taoryx.family_manifest import load_unified_family_manifest_catalog
from taoryx.fidelity_contracts import FidelityTier
from taoryx.horizontal_fidelity import load_horizontal_registry
from taoryx.language_backed_batch_episode_parity import verify_serialized_language_backed_batch_episode_parity
from taoryx.language_backed_execution import execute_powered_fixed_wing_composition
from taoryx.local_controller_screen_advertisements import LocalControllerScreenAdvertisement
from taoryx.local_direct_wrench_batch_episode_parity import verify_serialized_local_direct_wrench_batch_episode_parity
from taoryx.local_direct_wrench_composition_execution import execute_local_direct_wrench_composition
from taoryx.local_native_coordinate_lqi_composition_execution import execute_local_native_coordinate_lqi_composition
from taoryx.physical_wrench_tuning import (
    build_projected_physical_wrench_tuning_adapter,
    build_scheduled_projected_physical_wrench_tuning_adapter,
)
from taoryx.plugins import PluginDefinition, PluginMetadata, PluginRegistrar
from taoryx.trajectory.contract_probe_mission_composition import ContractProbeMissionCompositionProvider
from taoryx.trajectory.language_backed_guidance import (
    LanguageBackedGuidanceControlPlant,
    LanguageBackedPseudoGuidanceControlPlant,
    build_language_backed_guidance_tuning_campaign,
    build_language_backed_pseudo_guidance_tuning_campaign,
)
from taoryx.trajectory.pseudo6dof_profiles import load_pseudo6dof_catalog
from taoryx.vehicle_batch_execution import VehicleBatchExecutionRequest, batch_factory_request_v1
from taoryx.vehicle_composition import CompiledVehicleComposition
from taoryx.vehicle_composition_registry import (
    load_resolved_vehicle_composition_catalog,
    load_vehicle_composition_registry,
)
from taoryx.vehicle_execution_preflight import (
    SemanticPreflightHandler,
    _preflight_hummingbird_hover_yaw,
    _preflight_local_direct_wrench,
    _preflight_local_native_coordinate_lqi,
    _preflight_nesc_source_replay,
    _preflight_powered_fixed_wing_racetrack,
)

from .resources import model_resource_root


def _source_local_probe(adapter: StandardFamilyAdapter) -> AdapterProbeCase:
    """Return the plant-owned source operating point for generic probes."""

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


def _hl20_source_probe(adapter: StandardFamilyAdapter) -> AdapterProbeCase:
    """Return the documented source-local condition for HL-20 operation probes."""

    if adapter.describe().tier == "rigid_body_6dof_surface_allocated":
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


def _x15_source_probe(adapter: StandardFamilyAdapter) -> AdapterProbeCase:
    """Return the pinned local source release/glide state for X-15 probes."""

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


def _nesc_source_replay_probe(adapter: StandardFamilyAdapter) -> AdapterProbeCase:
    """Exercise the exact packaged NESC history through the replay capability."""

    del adapter
    artifact = model_resource_root() / "verification" / "daveml_nesc_reduction_qualification.json"
    return AdapterProbeCase(state={}, effectors={}, replay_request={"source_artifact": str(artifact)})
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


def _passive_tumbling_adapter(tier: FidelityTier) -> StandardFamilyAdapter:
    """Expose the two executable passive-body reductions without fake controls.

    The composed direct-release executor belongs to the optional reachability
    package, but this descriptor is intentionally data-only: it tells the
    common authoring and lowering layers exactly which committed state/resource
    truth the passive family contributes.  It must not fabricate derivatives,
    trim, allocation, or a controller merely because the same interface is
    shared with controlled families.
    """

    state_channels = [
        AdapterChannel("position_m", "m", "state", frame="local_reduced"),
        AdapterChannel("velocity_m_s", "m/s", "state", frame="local_reduced"),
        AdapterChannel("projected_area_m2", "m^2", "state", frame="body"),
        AdapterChannel("drag_force_n", "N", "state", frame="local_reduced"),
    ]
    if tier == "pseudo_6dof":
        state_channels.extend(
            (
                AdapterChannel("attitude_quaternion", "dimensionless", "state", frame="body-to-local"),
                AdapterChannel("attitude_rate_rad_s", "rad/s", "state", frame="body"),
                AdapterChannel("angular_rate_norm_rad_s", "rad/s", "state", frame="body"),
            )
        )
        validity_envelope = "Declared direct release only; rigid-body passive-tumble equations are reused for committed truth telemetry."
    elif tier == "point_mass_3dof":
        state_channels.append(AdapterChannel("angular_rate_norm_rad_s", "rad/s", "state", frame="body"))
        validity_envelope = "Declared direct release only; orientation-averaged projected area is an explicit 3DOF reduction."
    else:
        raise ValueError(f"tumbling_body: unsupported passive adapter tier {tier!r}")
    descriptor = FamilyAdapterDescriptor(
        family_id="tumbling_body",
        adapter_id="taoryx.passive_body.rigid_aero.v1",
        physical_family="passive_ballistic_tumbling_body",
        tier=tier,
        state_channels=tuple(state_channels),
        resource_channels=(AdapterChannel("mass_kg", "kg", "resource"),),
        control_realization_override="uncontrolled",
        evidence_status="development",
        validity_envelope=validity_envelope,
        omitted_physics=(
            "controller and control law",
            "wrench command and actuator allocation",
            "actuator dynamics",
            "shape-specific aerodynamic-moment and damping qualification",
        ),
    )
    return StandardFamilyAdapter.passive(descriptor)
    ####


def _registrations() -> tuple[FamilyAdapterRegistration, ...]:
    """Return the legacy bundled registrations without constructing plants."""

    return (
        FamilyAdapterRegistration(
            "x15",
            "taoryx.high_energy.fixed_wing.v1",
            "available",
            build_x15_source_direct_wrench_adapter,
            probe_factory=_x15_source_probe,
            supported_tiers=("rigid_body_6dof_direct_wrench",),
            note="local source direct-wrench bridge only; mission translation remains pending",
        ),
        FamilyAdapterRegistration(
            "hl20_mod_k",
            "taoryx.lifting_body.daveml.v1",
            "available",
            build_hl20_source_adapter,
            probe_factory=_hl20_source_probe,
            supported_tiers=("rigid_body_6dof_direct_wrench", "rigid_body_6dof_surface_allocated"),
            note="local source load/surface witnesses; mission translation remains pending",
        ),
        FamilyAdapterRegistration(
            "reference_nesc_two_stage_rocket",
            "taoryx.rocket.variable_mass_nesc.v1",
            "available",
            build_nesc_replay_adapter,
            probe_factory=_nesc_source_replay_probe,
            supported_tiers=("point_mass_3dof", "pseudo_6dof"),
            note="source replay adapter; active segment translation remains pending",
        ),
        FamilyAdapterRegistration(
            "tumbling_body",
            "taoryx.passive_body.rigid_aero.v1",
            "available",
            _passive_tumbling_adapter,
            supported_tiers=("point_mass_3dof", "pseudo_6dof"),
            note=("batch-only passive direct-release reductions; no controller, derivative, trim, allocator, or physical-effector path is implied"),
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
            "available",
            _a320_family_adapter,
            probe_factory=_a320_family_probe,
            supported_tiers=("point_mass_3dof", "pseudo_6dof"),
            note=(
                "OpenAP point-mass and named pseudo-6DOF response products are executable through the shared reduced "
                "Composition path; direct-wrench and physical-surface tiers remain intentionally unavailable"
            ),
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
    ####


def _a320_family_adapter(tier: FidelityTier) -> StandardFamilyAdapter:
    """Build one executable A320 reduced tier through the public adapter seam."""

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
        raise ValueError(f"a320_openap_3dof has no executable {tier} adapter")
    descriptor = descriptor_from_control_plant(
        plant,
        family_id="a320_openap_3dof",
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


def _a320_pseudo_tuning_adapter() -> StandardFamilyAdapter:
    """Build the pseudo-6DOF A320 tier used by the common campaign runner."""

    return build_a320_pseudo_control_adapter()
    ####


def _a320_point_tuning_adapter() -> StandardFamilyAdapter:
    """Build the point-mass A320 tier used by the common campaign runner."""

    return _a320_family_adapter("point_mass_3dof")
    ####


def _hummingbird_pseudo_tuning_adapter() -> StandardFamilyAdapter:
    """Build the reduced Hummingbird plant used by the common campaign runner."""

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
    """Build the exact source-hover attitude-wrench runtime projection."""

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
            "requested moment through bounded individual rotors with motor lag"
        ),
        omitted_physics=(
            "motor-coordinate campaign synthesis",
            "nonlinear rotor allocation in the campaign adapter",
            "translation, wind, battery, landing, and qualification",
        ),
    )
    ####


def _hummingbird_source_rotor_vertical_tuning_adapter() -> StandardFamilyAdapter:
    """Build the exact source-hover vertical-force runtime projection."""

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
            "allocates collective force and moments through bounded individual rotors with motor lag"
        ),
        omitted_physics=(
            "motor-coordinate campaign synthesis",
            "nonlinear rotor allocation in the campaign adapter",
            "wind, battery, landing, gain scheduling, and qualification",
        ),
    )
    ####


def _f16_point_tuning_adapter() -> StandardFamilyAdapter:
    """Build the source-trim F-16 point-mass plant for the common runner."""

    source, trim, trim_pitch_rad = _f16_source_trim()
    plant = build_f16_reduced_control_plant(point_model=F16PointMass3DOFModel(source, trim, trim_pitch_rad))
    descriptor = descriptor_from_control_plant(
        plant,
        family_id="f16_s119",
        adapter_id="taoryx.fixed_wing.daveml.point_tuning.v1",
        physical_family="powered_fixed_wing",
        tier="point_mass_3dof",
    )
    return StandardFamilyAdapter.from_control_plant(descriptor, plant)
    ####


def _f16_pseudo_tuning_adapter() -> StandardFamilyAdapter:
    """Build the source-trim F-16 pseudo-6DOF plant for the common runner."""

    source, trim, trim_pitch_rad = _f16_source_trim()
    linearization = source.linearize_local(
        trim.state,
        trim.controls,
        trim_pitch_rad=trim_pitch_rad,
        altitude_m=0.0,
        state_step=1.0e-5,
        control_step=1.0e-5,
    )
    plant = build_f16_reduced_control_plant(
        pseudo_model=F16AttitudeResponsePseudo6DOFModel(
            source,
            trim,
            linearization,
            trim_pitch_rad,
        )
    )
    descriptor = descriptor_from_control_plant(
        plant,
        family_id="f16_s119",
        adapter_id="taoryx.fixed_wing.daveml.pseudo_tuning.v1",
        physical_family="powered_fixed_wing",
        tier="pseudo_6dof",
    )
    return StandardFamilyAdapter.from_control_plant(descriptor, plant)
    ####


def _f16_source_surface_tuning_adapter() -> StandardFamilyAdapter:
    """Build the source-trim F-16 physical effector plant for common tuning."""

    return _source_table_fixed_wing_factory(
        build_f16_source_physical_plant,
        family_id="f16_s119",
        adapter_id="taoryx.fixed_wing.daveml.v1",
        effector_attribute="effectors",
        omitted_physics=(
            "mission translation and gain scheduling",
            "full-flight-envelope and release qualification",
        ),
    )("rigid_body_6dof_surface_allocated")
    ####


def _f16_source_surface_lqi_tuning_adapter() -> StandardFamilyAdapter:
    """Build the exact source-trim F-16 state-to-wrench LQI projection."""

    return build_projected_physical_wrench_tuning_adapter(
        build_f16_local_physical_wrench_lqi_design(),
        family_id="f16_s119",
        adapter_id="taoryx.fixed_wing.daveml.f16_source_trim.physical_wrench_tuning.v1",
        physical_family="powered_fixed_wing",
        tier="rigid_body_6dof_surface_allocated",
        state_units={
            "u_m_s": "m/s",
            "v_m_s": "m/s",
            "w_m_s": "m/s",
            "p_rad_s": "rad/s",
            "q_rad_s": "rad/s",
            "r_rad_s": "rad/s",
        },
        validity_envelope=(
            "F-16 fixed-altitude source-trim velocity/rate physical-wrench projection; nonlinear execution allocates "
            "each requested force/moment through the bounded surface/throttle overlay"
        ),
        omitted_physics=(
            "raw source-surface campaign synthesis",
            "nonlinear surface/throttle allocation in the campaign adapter",
            "gain scheduling, navigation, wind/mass robustness, and qualification",
        ),
    )
    ####


def _f16_source_surface_schedule_lqi_tuning_adapter() -> StandardFamilyAdapter:
    """Build the four exact source-node projections used by the held LQI schedule."""

    nodes = build_f16_source_physical_schedule_lqi_nodes()
    return build_scheduled_projected_physical_wrench_tuning_adapter(
        {node.point_id: node.design for node in nodes},
        node_trim_targets=f16_source_physical_schedule_tuning_targets(),
        family_id="f16_s119",
        adapter_id="taoryx.fixed_wing.daveml.f16_source_schedule.physical_wrench_tuning.v1",
        physical_family="powered_fixed_wing",
        tier="rigid_body_6dof_surface_allocated",
        state_units={
            "u_m_s": "m/s",
            "v_m_s": "m/s",
            "w_m_s": "m/s",
            "p_rad_s": "rad/s",
            "q_rad_s": "rad/s",
            "r_rad_s": "rad/s",
        },
        validity_envelope=(
            "Four explicit F-16 source-retrimmed velocity/rate physical-wrench projections; nonlinear execution "
            "holds each selected candidate at its node and allocates requests through bounded surfaces/throttle"
        ),
        omitted_physics=(
            "continuous gain interpolation",
            "nonlinear surface/throttle allocation in the campaign adapter",
            "node-transition flight, navigation, wind/mass robustness, and qualification",
        ),
    )
    ####


def _f16_source_surface_schedule_lqr_tuning_adapter() -> StandardFamilyAdapter:
    """Build the four exact source-node projections used by the LQR schedule."""

    nodes = build_f16_source_physical_schedule_nodes()
    return build_scheduled_projected_physical_wrench_tuning_adapter(
        {node.point_id: node.design for node in nodes},
        node_trim_targets=f16_source_physical_schedule_tuning_targets(),
        family_id="f16_s119",
        adapter_id="taoryx.fixed_wing.daveml.f16_source_schedule.physical_wrench_lqr_tuning.v1",
        physical_family="powered_fixed_wing",
        tier="rigid_body_6dof_surface_allocated",
        state_units={
            "u_m_s": "m/s",
            "v_m_s": "m/s",
            "w_m_s": "m/s",
            "p_rad_s": "rad/s",
            "q_rad_s": "rad/s",
            "r_rad_s": "rad/s",
        },
        validity_envelope=(
            "Four explicit F-16 source-retrimmed velocity/rate physical-wrench projections; the runtime applies "
            "one candidate per node before performing its declared gain interpolation and bounded allocation"
        ),
        omitted_physics=(
            "continuous source-model interpolation during campaign synthesis",
            "nonlinear surface/throttle allocation in the campaign adapter",
            "navigation, wind/mass robustness, and qualification",
        ),
    )
    ####


def _local_controller_screen_advertisements() -> tuple[LocalControllerScreenAdvertisement, ...]:
    """Publish retained local controllers that are not tuner-owned campaigns.

    The F-16 direct-wrench comparator has a source-owned LQR profile and an
    executable Composition endpoint, but deliberately no tunable public
    direct-wrench authority.  Its static record makes those exact facts
    available to the common plan without pretending it is an allocator or a
    generic controller-tuning campaign.
    """

    return (
        LocalControllerScreenAdvertisement(
            id="f16-source-trim-direct-wrench-lqr-screen-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="f16_s119",
            family_id="f16_s119",
            fidelity="rigid_body_6dof_direct_wrench",
            realization_id="rigid_body_6dof_direct_wrench",
            mission_template_id="f16_local_physical_control_screen_v1",
            advertisement={
                "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
                "id": "f16-source-trim-direct-wrench-lqr-screen-v1",
                "mission_template_id": "f16_local_physical_control_screen_v1",
                "fidelity": "rigid_body_6dof_direct_wrench",
                "operations": ["batch"],
                "control_realization": "direct_wrench",
                "physical_effector_allocation": False,
                "batch_action_trace": "emits_committed_interval_trace",
                "controller": {
                    "method": "lqr",
                    "controller_id": "f16.local_physical_wrench_lqr.v1",
                    "campaign_id": None,
                    "fixed_cadence_s": 0.2,
                    "screen_duration_s": 1.0,
                },
                "control_authority": {
                    "availability": "internally_generated_batch_only",
                    "external_override": False,
                    "hard_limits": "not_declared",
                },
                "direct_wrench_controls": [
                    {
                        "channel_id": "wrench.force.command",
                        "component": 0,
                        "native_control_id": "total_force_x_n",
                        "unit": "N",
                        "normalization_scale": 5000.0,
                        "hard_bounds": None,
                    },
                    {
                        "channel_id": "wrench.moment.command",
                        "component": 0,
                        "native_control_id": "total_moment_x_nm",
                        "unit": "N m",
                        "normalization_scale": 10000.0,
                        "hard_bounds": None,
                    },
                    {
                        "channel_id": "wrench.moment.command",
                        "component": 1,
                        "native_control_id": "total_moment_y_nm",
                        "unit": "N m",
                        "normalization_scale": 10000.0,
                        "hard_bounds": None,
                    },
                    {
                        "channel_id": "wrench.moment.command",
                        "component": 2,
                        "native_control_id": "total_moment_z_nm",
                        "unit": "N m",
                        "normalization_scale": 10000.0,
                        "hard_bounds": None,
                    },
                ],
                "claim_boundary": (
                    "This pinned source-trim LQR internally generates only four generalized-wrench coordinates. "
                    "The source profile declares normalization scales, not hard wrench bounds or caller override; "
                    "it is neither a physical F-16 effector allocator nor a scheduled flight controller."
                ),
            },
        ),
    )
    ####


def _x15_direct_wrench_tuning_adapter() -> StandardFamilyAdapter:
    """Build the executable X-15 local direct-wrench bridge for tuning."""

    return build_x15_source_direct_wrench_adapter("rigid_body_6dof_direct_wrench")
    ####


def _x15_source_surface_tuning_adapter() -> StandardFamilyAdapter:
    """Build the exact X-15 physical-wrench runtime projection for tuning."""

    return build_projected_physical_wrench_tuning_adapter(
        build_x15_source_surface_physical_lqi_design(),
        family_id="x15",
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
            "X-15 frozen release-fixture local attitude/rate physical-wrench projection; nonlinear execution allocates "
            "each moment request through the bounded source surfaces"
        ),
        omitted_physics=(
            "raw source-surface campaign synthesis",
            "nonlinear surface allocation in the campaign adapter",
            "full flight trim, translation, guidance, and qualification",
        ),
    )
    ####


def _hl20_direct_wrench_tuning_adapter() -> StandardFamilyAdapter:
    """Build the fixed-condition HL-20 source direct-wrench tuning bridge."""

    return build_hl20_source_direct_wrench_tuning_adapter()
    ####


def _hl20_source_surface_tuning_adapter() -> StandardFamilyAdapter:
    """Build the exact HL-20 physical-wrench runtime projection for tuning."""

    return build_projected_physical_wrench_tuning_adapter(
        build_hl20_source_surface_physical_lqi_design(),
        family_id="hl20_mod_k",
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


def _x8_source_surface_tuning_adapter() -> StandardFamilyAdapter:
    """Build the exact X8 physical-wrench runtime projection for tuning."""

    return build_projected_physical_wrench_tuning_adapter(
        build_x8_source_surface_physical_lqi_design(),
        family_id="skywalker_x8",
        adapter_id="taoryx.fixed_wing.source_table.physical_wrench_tuning.v1",
        physical_family="powered_fixed_wing",
        tier="rigid_body_6dof_surface_allocated",
        state_units={
            "roll_error_rad": "rad",
            "pitch_error_rad": "rad",
            "p_rad_s": "rad/s",
            "q_rad_s": "rad/s",
        },
        validity_envelope=(
            "X8 source-trim four-state roll/pitch physical-wrench projection; each nonlinear runtime command still "
            "uses the bounded collective/differential-elevon allocator"
        ),
        omitted_physics=(
            "broader source-table state/elevon coordinate tuning",
            "nonlinear allocator execution in the campaign adapter",
            "route tracking, gain scheduling, and flight qualification",
        ),
    )
    ####


def _x8_language_guidance_tuning_adapter() -> StandardFamilyAdapter:
    """Build the exact X8 point-mass commanded-state guidance adapter."""

    plant = LanguageBackedGuidanceControlPlant("skywalker_x8")
    descriptor = descriptor_from_control_plant(
        plant,
        family_id="skywalker_x8",
        adapter_id="taoryx.fixed_wing.language_backed_guidance.v1",
        physical_family="powered_fixed_wing",
        tier="point_mass_3dof",
        state_units=plant.state_units,
        control_units=plant.control_units,
        evidence_status="development",
        omitted_physics=(
            "physical surface and thrust authority",
            "pseudo-6DOF attitude sidecar control",
            "route-level nonlinear mission tracking and wind robustness",
        ),
    )
    return StandardFamilyAdapter.from_control_plant(descriptor, plant)
    ####


def _b747_language_guidance_tuning_adapter() -> StandardFamilyAdapter:
    """Build the exact B747 point-mass commanded-state guidance adapter."""

    plant = LanguageBackedGuidanceControlPlant("b747")
    descriptor = descriptor_from_control_plant(
        plant,
        family_id="b747",
        adapter_id="taoryx.fixed_wing.language_backed_guidance.v1",
        physical_family="powered_fixed_wing",
        tier="point_mass_3dof",
        state_units=plant.state_units,
        control_units=plant.control_units,
        evidence_status="development",
        omitted_physics=(
            "physical surface and thrust authority",
            "pseudo-6DOF attitude sidecar control",
            "route-level nonlinear mission tracking and wind robustness",
        ),
    )
    return StandardFamilyAdapter.from_control_plant(descriptor, plant)
    ####


def _language_backed_pseudo_guidance_tuning_adapter(family_id: str) -> StandardFamilyAdapter:
    """Build a profile-backed pseudo-6DOF kinematic guidance adapter."""

    plant = LanguageBackedPseudoGuidanceControlPlant(family_id)
    descriptor = descriptor_from_control_plant(
        plant,
        family_id=family_id,
        adapter_id="taoryx.fixed_wing.language_backed_pseudo_guidance.v1",
        physical_family="powered_fixed_wing",
        tier="pseudo_6dof",
        state_units=plant.state_units,
        control_units=plant.control_units,
        evidence_status="development",
        omitted_physics=(
            "physical surface and thrust authority",
            "nonlinear route-level mission tracking and wind robustness",
            "source-exact pseudo-6DOF attitude or actuator dynamics",
        ),
    )
    return StandardFamilyAdapter.from_control_plant(descriptor, plant)
    ####


def _b747_source_surface_tuning_adapter() -> StandardFamilyAdapter:
    """Build the exact B747 condition-3 physical-wrench runtime projection."""

    return build_projected_physical_wrench_tuning_adapter(
        build_b747_condition3_source_surface_physical_lqi_design(),
        family_id="b747",
        adapter_id="taoryx.fixed_wing.b747_condition3.physical_wrench_tuning.v1",
        physical_family="powered_fixed_wing",
        tier="rigid_body_6dof_surface_allocated",
        state_units={
            "roll_error_rad": "rad",
            "pitch_error_rad": "rad",
            "yaw_error_rad": "rad",
            "u_m_s": "m/s",
            "v_m_s": "m/s",
            "w_m_s": "m/s",
            "p_rad_s": "rad/s",
            "q_rad_s": "rad/s",
            "r_rad_s": "rad/s",
        },
        validity_envelope=(
            "B747 NASA CR-2144 condition-3 local attitude/rate physical-wrench projection; nonlinear execution "
            "allocates each requested moment through bounded source-table surfaces"
        ),
        omitted_physics=(
            "raw source-effector campaign synthesis",
            "nonlinear surface allocation in the campaign adapter",
            "gain scheduling, transport routing, and flight qualification",
        ),
    )
    ####


def _controller_tuning_campaigns() -> tuple[ControllerTuningCampaignRegistration, ...]:
    """Advertise executable campaigns while leaving the runner in core."""

    return (
        ControllerTuningCampaignRegistration(
            id="b747-source-surface-local-lqi-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="b747",
            family_id="b747",
            fidelity="rigid_body_6dof_surface_allocated",
            realization_ids=("rigid_body_6dof_surface_allocated",),
            mission_template_ids=(
                "b747_condition3_local_physical_surface_lqr_screen_v1",
                "b747_condition3_local_physical_surface_lqi_screen_v1",
            ),
            description=(
                "Exact scaled LQI candidate for the B747 condition-3 attitude/rate physical-wrench runtime; each "
                "nonlinear moment request remains allocated through bounded source-table surfaces."
            ),
            adapter_factory=_b747_source_surface_tuning_adapter,
            campaign_factory=build_b747_source_surface_lqi_tuning_campaign,
            local_controller_screens=(
                {
                    "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
                    "id": "b747-condition3-source-surface-lqr-screen-v1",
                    "mission_template_id": "b747_condition3_local_physical_surface_lqr_screen_v1",
                    "fidelity": "rigid_body_6dof_surface_allocated",
                    "operations": ["batch"],
                    "control_realization": "source_table_surface_lqr_allocation",
                    "physical_effector_allocation": True,
                    "batch_action_trace": "emits_committed_interval_trace",
                    "controller": {"method": "lqr", "campaign_id": None, "integral_output_names": [], "fixed_cadence_s": 0.05, "screen_duration_s": 80.0},
                    "effector_controls": [
                        {"id": "effector.elevator.position", "native_control_id": "elevator-deg", "unit": "deg", "lower": -10.0, "upper": 10.0},
                        {"id": "effector.aileron.position", "native_control_id": "aileron-deg", "unit": "deg", "lower": -10.0, "upper": 10.0},
                        {"id": "effector.rudder.position", "native_control_id": "rudder-deg", "unit": "deg", "lower": -15.0, "upper": 15.0},
                        {"id": "effector.throttle.position", "native_control_id": "throttle", "unit": "1", "lower": 0.0, "upper": 1.0},
                    ],
                    "claim_boundary": "One condition-3 source-table local LQR recovery through ideal bounded coordinates; it is not a servo, schedule, racetrack, or qualification claim.",
                },
                {
                    "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
                    "id": "b747-condition3-source-surface-lqi-screen-v1",
                    "mission_template_id": "b747_condition3_local_physical_surface_lqi_screen_v1",
                    "fidelity": "rigid_body_6dof_surface_allocated",
                    "operations": ["batch"],
                    "control_realization": "source_table_surface_lqi_allocation",
                    "physical_effector_allocation": True,
                    "batch_action_trace": "emits_committed_interval_trace",
                    "controller": {
                        "method": "lqi",
                        "campaign_id": "b747-source-surface-local-lqi-v1",
                        "integral_output_names": ["roll_error_rad", "pitch_error_rad", "yaw_error_rad"],
                        "integral_weight_multiplier": 0.025,
                        "integral_weight_multipliers": [1.0],
                        "physical_wrench_profile": {
                            "integral_q_diagonal": [0.025, 0.025, 0.025],
                            "selection_evidence": "matched external pitch-moment offset screen emitted by this batch",
                        },
                        "persistent_disturbance_screen": {
                            "id": "b747-local-lqi-matched-pitch-wrench-offset",
                            "environment_input": "external_pitch_moment_bias_nm",
                            "body_moment_axis": "moment_y_nm",
                            "fraction_of_declared_pitch_wrench_scale": 0.05,
                            "artifact_filename": "robustness_report.json",
                        },
                        "fixed_cadence_s": 0.05,
                        "screen_duration_s": 80.0,
                    },
                    "effector_controls": [
                        {"id": "effector.elevator.position", "native_control_id": "elevator-deg", "unit": "deg", "lower": -10.0, "upper": 10.0},
                        {"id": "effector.aileron.position", "native_control_id": "aileron-deg", "unit": "deg", "lower": -10.0, "upper": 10.0},
                        {"id": "effector.rudder.position", "native_control_id": "rudder-deg", "unit": "deg", "lower": -15.0, "upper": 15.0},
                        {"id": "effector.throttle.position", "native_control_id": "throttle", "unit": "1", "lower": 0.0, "upper": 1.0},
                    ],
                    "claim_boundary": "One condition-3 source-table local LQI recovery through ideal bounded coordinates plus a three-case matched external pitch-moment screen; it does not establish wind rejection, a schedule, racetrack, or qualification.",
                },
            ),
            claim_boundary=(
                "This campaign is a condition-3 source-trim local surface-coordinate design screen. "
                "It does not establish a scheduled B747 racetrack, actuator hardware qualification, or transport flight envelope."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="b747-language-backed-guidance-local-lqi-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="b747",
            family_id="b747",
            fidelity="point_mass_3dof",
            realization_ids=("point_mass_3dof",),
            mission_template_ids=("powered_fixed_wing_racetrack_v1",),
            description=(
                "Scaled LQI local campaign over the explicit B747 point-mass speed, flight-path, and heading "
                "commanded-state guidance authority."
            ),
            adapter_factory=_b747_language_guidance_tuning_adapter,
            campaign_factory=lambda: build_language_backed_guidance_tuning_campaign("b747"),
            claim_boundary=(
                "This campaign covers only the native lower-tier commanded-state guidance response around the "
                "pinned level racetrack point. It does not tune source throttle/surfaces, the pseudo-6DOF attitude "
                "sidecar, wind rejection, route tracking, or physical B747 actuators."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="b747-language-backed-pseudo-guidance-local-lqi-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="b747",
            family_id="b747",
            fidelity="pseudo_6dof",
            realization_ids=("pseudo_6dof",),
            mission_template_ids=("powered_fixed_wing_racetrack_v1",),
            description=(
                "Scaled LQI local campaign over explicit B747 guidance and the declared profile-backed "
                "pseudo-6DOF Euler response sidecar."
            ),
            adapter_factory=lambda: _language_backed_pseudo_guidance_tuning_adapter("b747"),
            campaign_factory=lambda: build_language_backed_pseudo_guidance_tuning_campaign("b747"),
            claim_boundary=(
                "This campaign covers only the native lower-tier commanded-state guidance response and declared "
                "kinematic attitude sidecar around the pinned level point. It does not tune source throttle/surfaces, "
                "wind rejection, route tracking, or physical B747 actuators."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="x8-source-surface-local-lqi-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="skywalker_x8",
            family_id="skywalker_x8",
            fidelity="rigid_body_6dof_surface_allocated",
            realization_ids=("rigid_body_6dof_surface_allocated",),
            mission_template_ids=(
                "x8_local_physical_surface_lqr_screen_v1",
                "x8_local_physical_surface_lqi_screen_v1",
                "x8_local_physical_surface_lqi_long_recovery_screen_v1",
            ),
            description=(
                "Exact scaled LQI candidate for the X8 source-trim roll/pitch physical-wrench runtime, with each "
                "nonlinear command subsequently allocated through the declared elevon coordinates."
            ),
            adapter_factory=_x8_source_surface_tuning_adapter,
            campaign_factory=build_x8_source_surface_lqi_tuning_campaign,
            local_controller_screens=(
                {
                    "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
                    "id": "x8-source-table-surface-lqr-screen-v1",
                    "mission_template_id": "x8_local_physical_surface_lqr_screen_v1",
                    "fidelity": "rigid_body_6dof_surface_allocated",
                    "operations": ["batch"],
                    "control_realization": "source_table_surface_lqr_allocation",
                    "physical_effector_allocation": True,
                    "batch_action_trace": "emits_committed_interval_trace",
                    "controller": {"method": "lqr", "campaign_id": None, "integral_output_names": [], "fixed_cadence_s": 0.01, "screen_duration_s": 8.0},
                    "effector_controls": [
                        {"id": "effector.elevon.collective.position", "native_control_id": "collective-elevon-deg", "unit": "deg", "lower": -20.0, "upper": 20.0, "rate_limit_per_s": 120.0, "time_constant_s": 0.05},
                        {"id": "effector.elevon.differential.position", "native_control_id": "differential-elevon-deg", "unit": "deg", "lower": -20.0, "upper": 20.0, "rate_limit_per_s": 120.0, "time_constant_s": 0.05},
                        {"id": "effector.throttle.position", "native_control_id": "throttle", "unit": "1", "lower": 0.0, "upper": 1.0, "time_constant_s": 0.2},
                    ],
                    "claim_boundary": "One source-table local LQR recovery through collective/differential elevon coordinates; these are not individual servo telemetry or a racetrack/qualification claim.",
                },
                {
                    "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
                    "id": "x8-source-table-surface-lqi-screen-v1",
                    "mission_template_id": "x8_local_physical_surface_lqi_screen_v1",
                    "fidelity": "rigid_body_6dof_surface_allocated",
                    "operations": ["batch"],
                    "control_realization": "source_table_surface_lqi_allocation",
                    "physical_effector_allocation": True,
                    "batch_action_trace": "emits_committed_interval_trace",
                    "controller": {
                        "method": "lqi",
                        "campaign_id": "x8-source-surface-local-lqi-v1",
                        "integral_output_names": ["roll_error_rad", "pitch_error_rad"],
                        "integral_weight_multiplier": 0.15,
                        "integral_weight_multipliers": [1.0],
                        "physical_wrench_profile": {
                            "integral_q_diagonal": [0.15, 0.15],
                            "selection_evidence": "matched external pitch-moment offset screen emitted by this batch",
                        },
                        "persistent_disturbance_screen": {
                            "id": "x8-local-lqi-matched-pitch-wrench-offset",
                            "environment_input": "external_pitch_moment_bias_nm",
                            "body_moment_axis": "moment_y_nm",
                            "fraction_of_declared_pitch_wrench_scale": 0.05,
                            "artifact_filename": "robustness_report.json",
                        },
                        "fixed_cadence_s": 0.01,
                        "screen_duration_s": 8.0,
                    },
                    "effector_controls": [
                        {"id": "effector.elevon.collective.position", "native_control_id": "collective-elevon-deg", "unit": "deg", "lower": -20.0, "upper": 20.0, "rate_limit_per_s": 120.0, "time_constant_s": 0.05},
                        {"id": "effector.elevon.differential.position", "native_control_id": "differential-elevon-deg", "unit": "deg", "lower": -20.0, "upper": 20.0, "rate_limit_per_s": 120.0, "time_constant_s": 0.05},
                        {"id": "effector.throttle.position", "native_control_id": "throttle", "unit": "1", "lower": 0.0, "upper": 1.0, "time_constant_s": 0.2},
                    ],
                    "claim_boundary": "One source-table local LQI recovery through collective/differential elevon coordinates plus a three-case matched external pitch-moment screen; it does not establish yaw authority, wind rejection, a racetrack, or qualification.",
                },
                {
                    "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
                    "id": "x8-source-table-surface-lqi-long-recovery-screen-v1",
                    "mission_template_id": "x8_local_physical_surface_lqi_long_recovery_screen_v1",
                    "fidelity": "rigid_body_6dof_surface_allocated",
                    "operations": ["batch"],
                    "control_realization": "source_table_surface_lqi_allocation",
                    "physical_effector_allocation": True,
                    "source_powered_trim": "freshly_solved_for_each_estimate_and_execution",
                    "batch_action_trace": "emits_committed_interval_trace",
                    "controller": {"method": "lqi", "campaign_id": "x8-source-surface-local-lqi-v1", "integral_output_names": ["roll_error_rad", "pitch_error_rad"], "fixed_cadence_s": 0.02, "screen_duration_s": 20.0, "extended_recovery": True},
                    "effector_controls": [
                        {"id": "effector.elevon.collective.position", "native_control_id": "collective-elevon-deg", "unit": "deg", "lower": -20.0, "upper": 20.0, "rate_limit_per_s": 120.0, "time_constant_s": 0.05},
                        {"id": "effector.elevon.differential.position", "native_control_id": "differential-elevon-deg", "unit": "deg", "lower": -20.0, "upper": 20.0, "rate_limit_per_s": 120.0, "time_constant_s": 0.05},
                        {"id": "effector.throttle.position", "native_control_id": "throttle", "unit": "1", "lower": 0.0, "upper": 1.0, "time_constant_s": 0.2},
                    ],
                    "claim_boundary": "Fresh source-powered trim plus one twenty-second local LQI roll/pitch recovery through collective/differential elevon coordinates; it does not establish yaw authority, wind rejection, a racetrack, or qualification.",
                },
            ),
            claim_boundary=(
                "This campaign is a source-trim local physical-wrench design screen whose nonlinear execution remains "
                "allocator-backed. "
                "It does not establish a full X8 racetrack, actuator hardware qualification, gain schedule, or flight envelope."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="x8-language-backed-guidance-local-lqi-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="skywalker_x8",
            family_id="skywalker_x8",
            fidelity="point_mass_3dof",
            realization_ids=("point_mass_3dof",),
            mission_template_ids=("powered_fixed_wing_racetrack_v1",),
            description=(
                "Scaled LQI local campaign over the explicit X8 point-mass speed, flight-path, and heading "
                "commanded-state guidance authority."
            ),
            adapter_factory=_x8_language_guidance_tuning_adapter,
            campaign_factory=lambda: build_language_backed_guidance_tuning_campaign("skywalker_x8"),
            claim_boundary=(
                "This campaign covers only the native lower-tier commanded-state guidance response around the "
                "pinned level racetrack point. It does not tune source throttle/elevons, the pseudo-6DOF attitude "
                "sidecar, wind rejection, route tracking, or physical X8 actuators."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="x8-language-backed-pseudo-guidance-local-lqi-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="skywalker_x8",
            family_id="skywalker_x8",
            fidelity="pseudo_6dof",
            realization_ids=("pseudo_6dof",),
            mission_template_ids=("powered_fixed_wing_racetrack_v1",),
            description=(
                "Scaled LQI local campaign over explicit X8 guidance and the declared profile-backed "
                "pseudo-6DOF Euler response sidecar."
            ),
            adapter_factory=lambda: _language_backed_pseudo_guidance_tuning_adapter("skywalker_x8"),
            campaign_factory=lambda: build_language_backed_pseudo_guidance_tuning_campaign("skywalker_x8"),
            claim_boundary=(
                "This campaign covers only the native lower-tier commanded-state guidance response and declared "
                "kinematic attitude sidecar around the pinned level point. It does not tune source throttle/elevons, "
                "wind rejection, route tracking, or physical X8 actuators."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="a320-point-cruise-performance-lqr-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="a320_openap_3dof",
            family_id="a320_openap_3dof",
            fidelity="point_mass_3dof",
            realization_ids=("point_mass_3dof",),
            mission_template_ids=("powered_fixed_wing_racetrack_v1",),
            description=("Scaled LQR nominal cruise-performance campaign over the declared A320 OpenAP point-mass state and control coordinates."),
            adapter_factory=_a320_point_tuning_adapter,
            campaign_factory=build_a320_point_tuning_campaign,
            claim_boundary=(
                "This campaign is an OpenAP point-mass local performance design screen. It does not establish "
                "a route controller, wind robustness, physical surface allocation, or Airbus qualification."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="a320-pseudo-cruise-attitude-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="a320_openap_3dof",
            family_id="a320_openap_3dof",
            fidelity="pseudo_6dof",
            realization_ids=("pseudo_6dof", "jsbsim_surrogate_composite_pseudo6dof"),
            mission_template_ids=("powered_fixed_wing_racetrack_v1", "a320_local_native_coordinate_lqi_screen_v1"),
            description="Scaled LQI cruise-attitude inner-loop campaign over the declared A320 pseudo-6DOF response model.",
            adapter_factory=_a320_pseudo_tuning_adapter,
            campaign_factory=build_a320_pseudo_tuning_campaign,
        ),
        ControllerTuningCampaignRegistration(
            id="f16-point-source-trim-translation-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="f16_s119",
            family_id="f16_s119",
            fidelity="point_mass_3dof",
            realization_ids=("point_mass_3dof",),
            mission_template_ids=("powered_fixed_wing_racetrack_v1",),
            description="Scaled LQR source-trim translational campaign over the declared F-16 point-mass reduction.",
            adapter_factory=_f16_point_tuning_adapter,
            campaign_factory=build_f16_point_tuning_campaign,
        ),
        ControllerTuningCampaignRegistration(
            id="f16-pseudo-source-trim-attitude-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="f16_s119",
            family_id="f16_s119",
            fidelity="pseudo_6dof",
            realization_ids=("pseudo_6dof",),
            mission_template_ids=("powered_fixed_wing_racetrack_v1",),
            description="Scaled LQI source-trim attitude-response campaign over the declared F-16 pseudo-6DOF reduction.",
            adapter_factory=_f16_pseudo_tuning_adapter,
            campaign_factory=build_f16_pseudo_tuning_campaign,
        ),
        ControllerTuningCampaignRegistration(
            id="f16-source-surface-local-lqr-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="f16_s119",
            family_id="f16_s119",
            fidelity="rigid_body_6dof_surface_allocated",
            realization_ids=("rigid_body_6dof_surface_allocated",),
            mission_template_ids=("f16_local_physical_control_screen_v1",),
            description=("Scaled LQR source-trim local campaign over declared F-16 elevator, aileron, rudder, and throttle coordinates."),
            adapter_factory=_f16_source_surface_tuning_adapter,
            campaign_factory=build_f16_source_surface_lqr_tuning_campaign,
            local_controller_screens=(
                {
                    "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
                    "id": "f16-source-trim-surface-lqr-screen-v1",
                    "mission_template_id": "f16_local_physical_control_screen_v1",
                    "fidelity": "rigid_body_6dof_surface_allocated",
                    "operations": ["batch"],
                    "control_realization": "surface_allocated",
                    "physical_effector_allocation": True,
                    "batch_action_trace": "emits_committed_interval_trace",
                    "controller": {"method": "lqr", "campaign_id": "f16-source-surface-local-lqr-v1", "integral_output_names": [], "fixed_cadence_s": 0.2, "screen_duration_s": 1.0},
                    "effector_controls": [
                        {"id": "effector.elevator.position", "native_control_id": "elevator_deg", "unit": "deg", "lower": -25.0, "upper": 25.0, "rate_limit_per_s": 60.0, "time_constant_s": 0.08},
                        {"id": "effector.aileron.position", "native_control_id": "aileron_deg", "unit": "deg", "lower": -21.0, "upper": 21.0, "rate_limit_per_s": 80.0, "time_constant_s": 0.08},
                        {"id": "effector.rudder.position", "native_control_id": "rudder_deg", "unit": "deg", "lower": -30.0, "upper": 30.0, "rate_limit_per_s": 80.0, "time_constant_s": 0.08},
                        {"id": "effector.throttle.position", "native_control_id": "throttle_fraction", "unit": "1", "lower": 0.0, "upper": 1.0, "rate_limit_per_s": 2.5, "time_constant_s": 0.08},
                    ],
                    "claim_boundary": "One source-trim physical allocation LQR entry screen. It is not an F-16 route, schedule, wind/mass robustness, envelope, or qualification claim.",
                },
            ),
            claim_boundary=(
                "This campaign exposes one source-trim physical-effector design screen. It does not establish a gain "
                "schedule, navigation, wind/mass robustness, envelope coverage, or flight qualification."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="f16-source-surface-schedule-lqr-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="f16_s119",
            family_id="f16_s119",
            fidelity="rigid_body_6dof_surface_allocated",
            realization_ids=("rigid_body_6dof_surface_allocated",),
            mission_template_ids=(
                "f16_local_physical_surface_lqr_schedule_interior_screen_v1",
                "f16_local_physical_surface_lqr_schedule_transition_screen_v1",
            ),
            description=(
                "Four exact source-node LQR candidates in the requested-force/moment coordinates consumed by the "
                "F-16 held-node and interpolated surface/throttle schedule runtimes."
            ),
            adapter_factory=_f16_source_surface_schedule_lqr_tuning_adapter,
            campaign_factory=build_f16_source_physical_schedule_lqr_tuning_campaign,
            local_controller_screens=(
                {
                    "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
                    "id": "f16-source-node-surface-lqr-schedule-interior-screen-v1",
                    "mission_template_id": "f16_local_physical_surface_lqr_schedule_interior_screen_v1",
                    "fidelity": "rigid_body_6dof_surface_allocated",
                    "operations": ["batch"],
                    "control_realization": "surface_allocated",
                    "physical_effector_allocation": True,
                    "batch_action_trace": "emits_committed_interval_trace",
                    "controller": {
                        "method": "lqr",
                        "campaign_id": "f16-source-surface-schedule-lqr-v1",
                        "integral_output_names": [],
                        "fixed_cadence_s": 0.02,
                        "screen_duration_s": 16.0,
                        "selection": "discrete_source_node_held_for_each_recovery",
                    },
                    "effector_controls": [
                        {"id": "effector.elevator.position", "native_control_id": "elevator_deg", "unit": "deg", "lower": -25.0, "upper": 25.0, "rate_limit_per_s": 60.0, "time_constant_s": 0.08},
                        {"id": "effector.aileron.position", "native_control_id": "aileron_deg", "unit": "deg", "lower": -21.0, "upper": 21.0, "rate_limit_per_s": 80.0, "time_constant_s": 0.08},
                        {"id": "effector.rudder.position", "native_control_id": "rudder_deg", "unit": "deg", "lower": -30.0, "upper": 30.0, "rate_limit_per_s": 80.0, "time_constant_s": 0.08},
                        {"id": "effector.throttle.position", "native_control_id": "throttle_fraction", "unit": "1", "lower": 0.0, "upper": 1.0, "rate_limit_per_s": 2.5, "time_constant_s": 0.08},
                    ],
                    "claim_boundary": "Eight independent held-node source-retrimmed F-16 physical LQR recoveries through bounded surface allocation. It is not continuous gain interpolation, a node transition, a route, a full envelope, or qualification.",
                },
                {
                    "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
                    "id": "f16-source-node-surface-lqr-schedule-transition-screen-v1",
                    "mission_template_id": "f16_local_physical_surface_lqr_schedule_transition_screen_v1",
                    "fidelity": "rigid_body_6dof_surface_allocated",
                    "operations": ["batch"],
                    "control_realization": "surface_allocated",
                    "physical_effector_allocation": True,
                    "batch_action_trace": "emits_committed_interval_trace",
                    "controller": {
                        "method": "lqr",
                        "campaign_id": "f16-source-surface-schedule-lqr-v1",
                        "integral_output_names": [],
                        "fixed_cadence_s": 0.05,
                        "screen_duration_s": 240.0,
                        "selection": "linearly_interpolated_source_lqr_schedule_by_altitude_coordinate",
                    },
                    "effector_controls": [
                        {"id": "effector.elevator.position", "native_control_id": "elevator_deg", "unit": "deg", "lower": -25.0, "upper": 25.0, "rate_limit_per_s": 60.0, "time_constant_s": 0.08},
                        {"id": "effector.aileron.position", "native_control_id": "aileron_deg", "unit": "deg", "lower": -21.0, "upper": 21.0, "rate_limit_per_s": 80.0, "time_constant_s": 0.08},
                        {"id": "effector.rudder.position", "native_control_id": "rudder_deg", "unit": "deg", "lower": -30.0, "upper": 30.0, "rate_limit_per_s": 80.0, "time_constant_s": 0.08},
                        {"id": "effector.throttle.position", "native_control_id": "throttle_fraction", "unit": "1", "lower": 0.0, "upper": 1.0, "rate_limit_per_s": 2.5, "time_constant_s": 0.08},
                    ],
                    "claim_boundary": "Four retained time-marching source-node F-16 physical LQR transitions with exact candidate gains, explicit endpoint derivative/effectiveness blending, and bounded surface allocation. It is not navigation, wind/mass robustness, a full envelope, or qualification.",
                },
            ),
            claim_boundary=(
                "This campaign applies one selected candidate at each retained source node before a schedule runtime "
                "holds or interpolates those gains. It does not establish navigation, wind/mass robustness, an envelope, "
                "or flight qualification."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="f16-source-surface-local-lqi-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="f16_s119",
            family_id="f16_s119",
            fidelity="rigid_body_6dof_surface_allocated",
            realization_ids=("rigid_body_6dof_surface_allocated",),
            mission_template_ids=(
                "f16_local_physical_control_screen_v1",
                "f16_local_physical_surface_lqi_screen_v1",
            ),
            description=(
                "Scaled LQI source-trim local velocity/rate campaign in the exact requested-force/moment coordinates "
                "consumed by the bounded F-16 surface/throttle allocator."
            ),
            adapter_factory=_f16_source_surface_lqi_tuning_adapter,
            campaign_factory=build_f16_source_surface_lqi_tuning_campaign,
            local_controller_screens=(
                {
                    "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
                    "id": "f16-source-trim-surface-lqi-screen-v1",
                    "mission_template_id": "f16_local_physical_surface_lqi_screen_v1",
                    "fidelity": "rigid_body_6dof_surface_allocated",
                    "operations": ["batch"],
                    "control_realization": "surface_allocated",
                    "physical_effector_allocation": True,
                    "batch_action_trace": "emits_committed_interval_trace",
                    "controller": {"method": "lqi", "campaign_id": "f16-source-surface-local-lqi-v1", "integral_output_names": ["u_m_s", "v_m_s", "w_m_s"], "fixed_cadence_s": 0.02, "screen_duration_s": 5.0},
                    "effector_controls": [
                        {"id": "effector.elevator.position", "native_control_id": "elevator_deg", "unit": "deg", "lower": -25.0, "upper": 25.0, "rate_limit_per_s": 60.0, "time_constant_s": 0.08},
                        {"id": "effector.aileron.position", "native_control_id": "aileron_deg", "unit": "deg", "lower": -21.0, "upper": 21.0, "rate_limit_per_s": 80.0, "time_constant_s": 0.08},
                        {"id": "effector.rudder.position", "native_control_id": "rudder_deg", "unit": "deg", "lower": -30.0, "upper": 30.0, "rate_limit_per_s": 80.0, "time_constant_s": 0.08},
                        {"id": "effector.throttle.position", "native_control_id": "throttle_fraction", "unit": "1", "lower": 0.0, "upper": 1.0, "rate_limit_per_s": 2.5, "time_constant_s": 0.08},
                    ],
                    "claim_boundary": "One fixed-altitude source-trim physical allocation LQI recovery. It is not an F-16 route, wind/mass robustness, envelope, or qualification claim.",
                },
            ),
            claim_boundary=(
                "This campaign includes the fixed source-trim local physical-effector LQI design screen. It does not "
                "establish a gain schedule, envelope robustness, or flight qualification."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="f16-source-surface-schedule-lqi-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="f16_s119",
            family_id="f16_s119",
            fidelity="rigid_body_6dof_surface_allocated",
            realization_ids=("rigid_body_6dof_surface_allocated",),
            mission_template_ids=("f16_local_physical_surface_lqi_schedule_interior_screen_v1",),
            description=(
                "Four exact source-node LQI candidates in the requested-force/moment coordinates consumed by the "
                "held-node F-16 surface/throttle allocator."
            ),
            adapter_factory=_f16_source_surface_schedule_lqi_tuning_adapter,
            campaign_factory=build_f16_source_physical_schedule_lqi_tuning_campaign,
            local_controller_screens=(
                {
                    "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
                    "id": "f16-source-node-surface-lqi-schedule-interior-screen-v1",
                    "mission_template_id": "f16_local_physical_surface_lqi_schedule_interior_screen_v1",
                    "fidelity": "rigid_body_6dof_surface_allocated",
                    "operations": ["batch"],
                    "control_realization": "surface_allocated",
                    "physical_effector_allocation": True,
                    "batch_action_trace": "emits_committed_interval_trace",
                    "controller": {
                        "method": "lqi",
                        "campaign_id": "f16-source-surface-schedule-lqi-v1",
                        "integral_output_names": ["u_m_s", "v_m_s", "w_m_s"],
                        "fixed_cadence_s": 0.02,
                        "screen_duration_s": 16.0,
                        "selection": "discrete_source_node_held_for_each_recovery",
                    },
                    "effector_controls": [
                        {"id": "effector.elevator.position", "native_control_id": "elevator_deg", "unit": "deg", "lower": -25.0, "upper": 25.0, "rate_limit_per_s": 60.0, "time_constant_s": 0.08},
                        {"id": "effector.aileron.position", "native_control_id": "aileron_deg", "unit": "deg", "lower": -21.0, "upper": 21.0, "rate_limit_per_s": 80.0, "time_constant_s": 0.08},
                        {"id": "effector.rudder.position", "native_control_id": "rudder_deg", "unit": "deg", "lower": -30.0, "upper": 30.0, "rate_limit_per_s": 80.0, "time_constant_s": 0.08},
                        {"id": "effector.throttle.position", "native_control_id": "throttle_fraction", "unit": "1", "lower": 0.0, "upper": 1.0, "rate_limit_per_s": 2.5, "time_constant_s": 0.08},
                    ],
                    "claim_boundary": "Eight independent held-node source-retrimmed F-16 physical LQI recoveries through bounded surface allocation, restricted to the source-feasible plus/minus 0.25 m/s body-w interior. It is not continuous gain interpolation, a node transition, a route, a full envelope, wind/mass rejection, or qualification.",
                },
            ),
            claim_boundary=(
                "This campaign creates and applies a separately selected source-node candidate to each retained F-16 "
                "LQI recovery. It does not establish continuous gain scheduling, transition control, envelope robustness, "
                "or flight qualification."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="hummingbird-pseudo-hover-attitude-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="hummingbird",
            family_id="hummingbird",
            fidelity="pseudo_6dof",
            realization_ids=("pseudo_6dof",),
            mission_template_ids=("multirotor_pad_box_yaw_recovery_land_v1",),
            description="Scaled LQI hover-attitude inner-loop campaign over the declared aggregate-thrust response model.",
            adapter_factory=_hummingbird_pseudo_tuning_adapter,
            campaign_factory=build_hummingbird_pseudo_tuning_campaign,
        ),
        ControllerTuningCampaignRegistration(
            id="hummingbird-source-rotor-local-lqi-v1",
            provider_id="taoryx.registry.mission-composition",
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
            campaign_factory=build_hummingbird_source_rotor_lqi_tuning_campaign,
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
                    "effector_controls": [
                        {
                            "id": f"effector.rotor.{index}.speed.position",
                            "native_control_id": f"rotor-{index}-speed",
                            "unit": "rad/s",
                            "lower": 0.0,
                            "upper": 1500.0,
                            "time_constant_s": 0.005,
                        }
                        for index in range(1, 5)
                    ],
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
                    "effector_controls": [
                        {
                            "id": f"effector.rotor.{index}.speed.position",
                            "native_control_id": f"rotor-{index}-speed",
                            "unit": "rad/s",
                            "lower": 0.0,
                            "upper": 1500.0,
                            "time_constant_s": 0.005,
                        }
                        for index in range(1, 5)
                    ],
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
            provider_id="taoryx.registry.mission-composition",
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
            campaign_factory=build_hummingbird_source_rotor_vertical_lqi_tuning_campaign,
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
                    "effector_controls": [
                        {
                            "id": f"effector.rotor.{index}.speed.position",
                            "native_control_id": f"rotor-{index}-speed",
                            "unit": "rad/s",
                            "lower": 0.0,
                            "upper": 1500.0,
                            "time_constant_s": 0.005,
                        }
                        for index in range(1, 5)
                    ],
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
        ControllerTuningCampaignRegistration(
            id="x15-source-release-direct-wrench-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="x15",
            family_id="x15",
            fidelity="rigid_body_6dof_direct_wrench",
            realization_ids=("rigid_body_6dof_direct_wrench",),
            mission_template_ids=("x15_local_direct_wrench_screen_v1",),
            description="Scaled LQR source-release local direct-wrench campaign over the declared X-15 bridge witness.",
            adapter_factory=_x15_direct_wrench_tuning_adapter,
            campaign_factory=build_x15_direct_wrench_tuning_campaign,
            claim_boundary=(
                "This campaign screens bounded generalized direct-wrench feedback at the source release point. "
                "It does not synthesize or qualify physical X-15 effector allocation, navigation, or the staged mission."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="x15-source-release-direct-wrench-lqi-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="x15",
            family_id="x15",
            fidelity="rigid_body_6dof_direct_wrench",
            realization_ids=("rigid_body_6dof_direct_wrench",),
            mission_template_ids=("x15_local_direct_wrench_lqi_screen_v1",),
            description="Scaled LQI source-release local speed campaign over the declared X-15 direct-wrench bridge.",
            adapter_factory=_x15_direct_wrench_tuning_adapter,
            campaign_factory=build_x15_direct_wrench_lqi_tuning_campaign,
            claim_boundary=(
                "This campaign screens bounded generalized direct-wrench feedback at the source release point. "
                "It does not synthesize or qualify physical X-15 effector allocation, navigation, or the staged mission."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="x15-source-surface-local-lqi-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="x15",
            family_id="x15",
            fidelity="rigid_body_6dof_surface_allocated",
            realization_ids=("rigid_body_6dof_surface_allocated",),
            mission_template_ids=("x15_source_surface_attitude_rate_lqi_screen_v1",),
            description=(
                "Exact scaled LQI candidate for the frozen-translation X-15 attitude/rate physical-wrench runtime; "
                "each nonlinear moment request remains allocated through bounded source surfaces."
            ),
            adapter_factory=_x15_source_surface_tuning_adapter,
            campaign_factory=build_x15_source_surface_lqi_tuning_campaign,
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
        ControllerTuningCampaignRegistration(
            id="hl20-source-subsonic-direct-wrench-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="hl20_mod_k",
            family_id="hl20_mod_k",
            fidelity="rigid_body_6dof_direct_wrench",
            realization_ids=("rigid_body_6dof_direct_wrench",),
            mission_template_ids=("hl20_local_direct_wrench_screen_v1",),
            description="Scaled LQR local source-load cancellation campaign over the HL-20 declared direct-wrench bridge.",
            adapter_factory=_hl20_direct_wrench_tuning_adapter,
            campaign_factory=build_hl20_direct_wrench_tuning_campaign,
            claim_boundary=(
                "This campaign screens bounded generalized direct-wrench feedback at the documented HL-20 local source condition. "
                "It does not synthesize or qualify physical HL-20 surface allocation, navigation, or a flight envelope."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="hl20-source-subsonic-direct-wrench-lqi-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="hl20_mod_k",
            family_id="hl20_mod_k",
            fidelity="rigid_body_6dof_direct_wrench",
            realization_ids=("rigid_body_6dof_direct_wrench",),
            mission_template_ids=("hl20_local_direct_wrench_lqi_screen_v1",),
            description="Scaled LQI local body-speed campaign over the HL-20 declared direct-wrench bridge.",
            adapter_factory=_hl20_direct_wrench_tuning_adapter,
            campaign_factory=build_hl20_direct_wrench_lqi_tuning_campaign,
            claim_boundary=(
                "This campaign screens bounded generalized direct-wrench feedback at the documented HL-20 local source condition. "
                "It does not synthesize or qualify physical HL-20 surface allocation, navigation, or a flight envelope."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="hl20-source-surface-local-lqi-v1",
            provider_id="taoryx.registry.mission-composition",
            model_id="hl20_mod_k",
            family_id="hl20_mod_k",
            fidelity="rigid_body_6dof_surface_allocated",
            realization_ids=("rigid_body_6dof_surface_allocated",),
            mission_template_ids=("hl20_source_surface_attitude_rate_lqi_screen_v1",),
            description=(
                "Exact scaled LQI candidate for the frozen-translation HL-20 attitude/rate physical-wrench runtime; "
                "each nonlinear moment request remains allocated through seven bounded source surfaces."
            ),
            adapter_factory=_hl20_source_surface_tuning_adapter,
            campaign_factory=build_hl20_source_surface_lqi_tuning_campaign,
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
                        {"id": f"effector.surface.{name}.position", "native_control_id": name, "unit": "deg", "lower": lower, "upper": upper}
                        for name, (lower, upper) in HL20_SOURCE_SURFACE_BOUNDS_DEG.items()
                    ],
                    "claim_boundary": "This source-surface LQI screen holds the HL-20 Mach-1 source translational fixture fixed and controls only local attitude error/body rate through all seven bounded named surfaces. It also emits a bounded three-case matched external pitch-moment screen. It does not establish full-state trim, translation, glide guidance, navigation, wind or mass robustness, or flight qualification.",
                },
            ),
            claim_boundary="This campaign auto-tunes only the frozen-translation HL-20 local attitude/rate source-surface model. The associated batch emits one bounded matched external pitch-moment screen; it does not establish a source-full-glide trim, gain schedule, wind or mass robustness, guidance, or qualification.",
        ),
    )
    ####


def _without_max_steps(
    factory: Callable[[CompiledVehicleComposition, str | Path], Any],
) -> Callable[[CompiledVehicleComposition, str | Path, int | None], Any]:
    """Adapt a two-argument source executor to the common batch signature."""

    def invoke(
        composition: CompiledVehicleComposition,
        output_dir: str | Path,
        max_steps: int | None,
    ) -> Any:
        if max_steps is not None:
            raise ValueError(f"--max-steps is not available for batch factory {factory.__name__!r}")
        return factory(composition, output_dir)
        ####

    return invoke
    ####


@batch_factory_request_v1
def _language_backed_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Run the language-backed executor through the typed plug-in request."""

    return execute_powered_fixed_wing_composition(
        request.composition,
        request.output_dir,
        max_steps=request.max_steps,
    )
    ####


@batch_factory_request_v1
def _local_direct_wrench_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Apply an optional exact tuning context through the local-screen bridge."""

    if request.max_steps is not None:
        raise ValueError("--max-steps is not available for the local direct-wrench screen")
    return execute_local_direct_wrench_composition(
        request.composition,
        request.output_dir,
        tuning_context=request.tuning_context,
    )
    ####


@batch_factory_request_v1
def _local_native_coordinate_lqi_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Run a named-coordinate LQI screen with an optional exact candidate."""

    if request.max_steps is not None:
        raise ValueError("--max-steps is not available for the local native-coordinate LQI screen")
    return execute_local_native_coordinate_lqi_composition(
        request.composition,
        request.output_dir,
        tuning_context=request.tuning_context,
    )
    ####


@batch_factory_request_v1
def _x8_local_physical_surface_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Run the X8 source screen with an optional exact LQI candidate context."""

    return execute_x8_local_physical_control_screen(
        request.composition,
        request.output_dir,
        max_steps=request.max_steps,
        tuning_context=request.tuning_context,
    )
    ####


@batch_factory_request_v1
def _hummingbird_local_physical_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Run a Hummingbird source screen with its exact optional LQI candidate."""

    return execute_hummingbird_local_physical_control_screen(
        request.composition,
        request.output_dir,
        max_steps=request.max_steps,
        tuning_context=request.tuning_context,
    )
    ####


@batch_factory_request_v1
def _b747_local_physical_surface_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Run a B747 source screen with its exact optional LQI candidate."""

    return execute_b747_condition3_local_physical_control_screen(
        request.composition,
        request.output_dir,
        max_steps=request.max_steps,
        tuning_context=request.tuning_context,
    )
    ####


@batch_factory_request_v1
def _x15_local_physical_surface_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Run the X-15 source surface screen with its exact optional LQI candidate."""

    return execute_x15_local_physical_surface_lqi_screen(
        request.composition,
        request.output_dir,
        max_steps=request.max_steps,
        tuning_context=request.tuning_context,
    )
    ####


@batch_factory_request_v1
def _hl20_local_physical_surface_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Run the HL-20 source surface screen with its exact optional LQI candidate."""

    return execute_hl20_local_physical_surface_lqi_screen(
        request.composition,
        request.output_dir,
        max_steps=request.max_steps,
        tuning_context=request.tuning_context,
    )
    ####


@batch_factory_request_v1
def _f16_local_physical_lqi_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Run the F-16 physical LQI screen with its optional exact candidate."""

    return execute_f16_local_physical_lqi_screen(
        request.composition,
        request.output_dir,
        max_steps=request.max_steps,
        tuning_context=request.tuning_context,
    )
    ####


@batch_factory_request_v1
def _f16_physical_schedule_interior_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Run a held-node F-16 schedule with its complete optional candidate set."""

    if request.tuning_context is not None:
        raise ValueError("F-16 schedule-interior screens require a complete node-indexed tuning context set")
    return execute_f16_physical_schedule_interior_screen(
        request.composition,
        request.output_dir,
        max_steps=request.max_steps,
        tuning_context_set=request.tuning_context_set,
    )
    ####


@batch_factory_request_v1
def _f16_physical_schedule_transition_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Run an F-16 schedule transition with its complete optional candidate set."""

    if request.tuning_context is not None:
        raise ValueError("F-16 schedule-transition screens require a complete node-indexed tuning context set")
    return execute_f16_physical_schedule_transition_screen(
        request.composition,
        request.output_dir,
        max_steps=request.max_steps,
        tuning_context_set=request.tuning_context_set,
    )
    ####


def _semantic_handlers() -> tuple[SemanticPreflightHandler, ...]:
    """Return every reference-model translator owned by this distribution."""

    return (
        SemanticPreflightHandler(
            "taoryx.powered_fixed_wing_racetrack.capability_scaled.v1",
            _preflight_powered_fixed_wing_racetrack,
        ),
        SemanticPreflightHandler(
            "taoryx.x8_racetrack.source_route.v1",
            _preflight_powered_fixed_wing_racetrack,
        ),
        SemanticPreflightHandler(
            "taoryx.b747_racetrack.source_route.v1",
            _preflight_powered_fixed_wing_racetrack,
        ),
        SemanticPreflightHandler(
            "taoryx.f16_local_physical_control_screen.capability.v1",
            preflight_f16_local_physical_control_screen,
        ),
        SemanticPreflightHandler(
            "taoryx.f16_local_physical_surface_lqi_screen.capability.v1",
            preflight_f16_local_physical_lqi_screen,
        ),
        SemanticPreflightHandler(
            "taoryx.f16_local_physical_surface_lqr_schedule_interior_screen.capability.v1",
            preflight_f16_physical_schedule_interior_screen,
        ),
        SemanticPreflightHandler(
            "taoryx.f16_local_physical_surface_lqi_schedule_interior_screen.capability.v1",
            preflight_f16_physical_schedule_interior_screen,
        ),
        SemanticPreflightHandler(
            "taoryx.f16_local_physical_surface_lqr_schedule_transition_screen.capability.v1",
            preflight_f16_physical_schedule_transition_screen,
        ),
        SemanticPreflightHandler(
            "taoryx.b747_condition3_local_physical_surface_lqr_screen.capability.v1",
            preflight_b747_condition3_local_physical_control_screen,
        ),
        SemanticPreflightHandler(
            "taoryx.b747_condition3_local_physical_surface_lqi_screen.capability.v1",
            preflight_b747_condition3_local_physical_control_screen,
        ),
        SemanticPreflightHandler(
            "taoryx.x8_local_physical_surface_lqr_screen.capability.v1",
            preflight_x8_local_physical_control_screen,
        ),
        SemanticPreflightHandler(
            "taoryx.x8_local_physical_surface_lqi_screen.capability.v1",
            preflight_x8_local_physical_control_screen,
        ),
        SemanticPreflightHandler(
            "taoryx.x8_local_physical_surface_lqi_long_recovery_screen.capability.v1",
            preflight_x8_local_physical_control_screen,
        ),
        SemanticPreflightHandler(
            "taoryx.hummingbird.local_individual_rotor_lqi_screen.capability.v1",
            preflight_hummingbird_local_physical_control_screen,
        ),
        SemanticPreflightHandler(
            "taoryx.hummingbird.local_horizontal_translation_lqi_screen.capability.v1",
            preflight_hummingbird_local_physical_control_screen,
        ),
        SemanticPreflightHandler(
            "taoryx.hummingbird.local_vertical_translation_lqi_screen.capability.v1",
            preflight_hummingbird_local_physical_control_screen,
        ),
        SemanticPreflightHandler(
            "taoryx.hummingbird.local_direct_wrench_screen.capability.v1",
            _preflight_local_direct_wrench,
        ),
        SemanticPreflightHandler(
            "taoryx.hummingbird.hover_yaw_contact.pseudo6dof.v1",
            _preflight_hummingbird_hover_yaw,
        ),
        SemanticPreflightHandler(
            "taoryx.nesc_staged_source_replay.capability.v1",
            _preflight_nesc_source_replay,
        ),
        SemanticPreflightHandler(
            "taoryx.x15_local_direct_wrench_screen.capability.v1",
            _preflight_local_direct_wrench,
        ),
        SemanticPreflightHandler(
            "taoryx.x15_local_direct_wrench_lqi_screen.capability.v1",
            _preflight_local_direct_wrench,
        ),
        SemanticPreflightHandler(
            "taoryx.x15_source_surface_authority_screen.capability.v1",
            preflight_x15_source_surface_authority_screen,
        ),
        SemanticPreflightHandler(
            "taoryx.x15_source_surface_attitude_rate_lqi_screen.capability.v1",
            preflight_x15_local_physical_surface_lqi_screen,
        ),
        SemanticPreflightHandler(
            "taoryx.hl20_local_direct_wrench_screen.capability.v1",
            _preflight_local_direct_wrench,
        ),
        SemanticPreflightHandler(
            "taoryx.hl20_local_direct_wrench_lqi_screen.capability.v1",
            _preflight_local_direct_wrench,
        ),
        SemanticPreflightHandler(
            "taoryx.hl20_source_surface_pitch_authority_screen.capability.v1",
            preflight_hl20_source_surface_authority_screen,
        ),
        SemanticPreflightHandler(
            "taoryx.hl20_source_surface_attitude_rate_lqi_screen.capability.v1",
            preflight_hl20_local_physical_surface_lqi_screen,
        ),
        SemanticPreflightHandler(
            "taoryx.a320.local_native_coordinate_lqi_screen.capability.v1",
            _preflight_local_native_coordinate_lqi,
        ),
    )
    ####


@lru_cache(maxsize=1)
def _registry_provider() -> RegistryMissionCompositionProvider:
    """Build the common composer from this distribution's packaged registries."""

    root = model_resource_root()
    pseudo = load_pseudo6dof_catalog(root / "verification/pseudo6dof_profiles.yaml")
    horizontal = load_horizontal_registry(root / "verification/horizontal_fidelity_registry.yaml")
    manifests = load_unified_family_manifest_catalog(
        horizontal=horizontal,
        pseudo=pseudo,
        root=root,
    )
    registry = load_vehicle_composition_registry(root / "verification/vehicle_composition_registry.yaml")
    catalog = load_resolved_vehicle_composition_catalog(
        registry=registry,
        manifests=manifests,
    )
    return RegistryMissionCompositionProvider(catalog)
    ####


def _register(registrar: PluginRegistrar) -> None:
    """Publish family adapters and model ownership without constructing plants."""

    for registration in _registrations():
        registrar.register_family_adapter(registration)
        registrar.register_model(registration.family_id, registration)
    registrar.register_mission_composition_provider(_registry_provider())
    registrar.register_mission_composition_provider(ReferenceMissionCompositionProvider())
    registrar.register_mission_composition_provider(ContractProbeMissionCompositionProvider())
    for adapter in reference_mission_capability_adapters():
        registrar.register_mission_capability_adapter(adapter)
    registrar.register_mission_capability_adapter(B747LocalPhysicalControlScreenCapabilityAdapter())
    registrar.register_mission_capability_adapter(B747LocalPhysicalLqiControlScreenCapabilityAdapter())
    registrar.register_mission_capability_adapter(F16LocalPhysicalControlScreenCapabilityAdapter())
    registrar.register_mission_capability_adapter(F16LocalPhysicalLqiScreenCapabilityAdapter())
    registrar.register_mission_capability_adapter(F16PhysicalScheduleInteriorScreenCapabilityAdapter())
    registrar.register_mission_capability_adapter(F16PhysicalScheduleLqiInteriorScreenCapabilityAdapter())
    registrar.register_mission_capability_adapter(F16PhysicalScheduleTransitionScreenCapabilityAdapter())
    registrar.register_mission_capability_adapter(HummingbirdLocalHorizontalTranslationLqiScreenCapabilityAdapter())
    registrar.register_mission_capability_adapter(HummingbirdLocalPhysicalControlScreenCapabilityAdapter())
    registrar.register_mission_capability_adapter(HummingbirdLocalVerticalTranslationLqiScreenCapabilityAdapter())
    registrar.register_mission_capability_adapter(X8LocalPhysicalControlScreenCapabilityAdapter())
    registrar.register_mission_capability_adapter(X8LocalPhysicalLqiControlScreenCapabilityAdapter())
    registrar.register_mission_capability_adapter(X8LocalPhysicalLongRecoveryLqiScreenCapabilityAdapter())
    registrar.register_mission_capability_adapter(X15SourceSurfaceAuthorityScreenCapabilityAdapter())
    registrar.register_mission_capability_adapter(X15LocalPhysicalSurfaceLqiScreenCapabilityAdapter())
    registrar.register_mission_capability_adapter(HL20SourceSurfacePitchAuthorityScreenCapabilityAdapter())
    registrar.register_mission_capability_adapter(HL20LocalPhysicalSurfaceLqiScreenCapabilityAdapter())
    for campaign in _controller_tuning_campaigns():
        registrar.register_controller_tuning_campaign(campaign)
    for screen in _local_controller_screen_advertisements():
        registrar.register_local_controller_screen_advertisement(screen)
    for handler in _semantic_handlers():
        registrar.register_semantic_preflight_handler(handler)
    for identifier, factory in (
        ("language_backed_interactive.v1", _open_language_backed_episode),
        ("hummingbird_aggregate_thrust_episode.v1", _open_hummingbird_episode),
        ("reduced_fixed_wing_a320_episode.v1", _open_a320_reduced_episode),
        ("reduced_fixed_wing_f16_episode.v1", _open_f16_reduced_episode),
        ("local_direct_wrench_episode.v1", _open_local_direct_wrench_episode),
        ("x15_local_direct_wrench_episode.v1", _open_local_direct_wrench_episode),
    ):
        registrar.register_episode_factory(identifier, factory)
    registrar.register_batch_episode_parity_verifier(
        "taoryx.hummingbird.aggregate_thrust_batch_episode_parity.v1",
        verify_serialized_composition_batch_episode_parity,
    )
    registrar.register_batch_episode_parity_verifier(
        "taoryx.language_backed.action_trace_batch_episode_parity.v1",
        verify_serialized_language_backed_batch_episode_parity,
    )
    for identifier in (
        "taoryx.x15.local_direct_wrench_batch_episode_parity.v1",
        "taoryx.local_direct_wrench_batch_episode_parity.v1",
    ):
        registrar.register_batch_episode_parity_verifier(
            identifier,
            verify_serialized_local_direct_wrench_batch_episode_parity,
        )
    for identifier in (
        "taoryx.reduced_fixed_wing.a320_action_trace_batch_episode_parity.v1",
        "taoryx.reduced_fixed_wing.f16_action_trace_batch_episode_parity.v1",
    ):
        registrar.register_batch_episode_parity_verifier(
            identifier,
            verify_serialized_reduced_fixed_wing_batch_episode_parity,
        )
    registrar.register_execution_factory(
        "language_backed_powered_fixed_wing.v1",
        _language_backed_batch,
    )
    registrar.register_execution_factory(
        "local_direct_wrench_screen.v1",
        _local_direct_wrench_batch,
    )
    registrar.register_execution_factory(
        "local_native_coordinate_lqi_screen.v1",
        _local_native_coordinate_lqi_batch,
    )
    registrar.register_execution_factory(
        "reduced_fixed_wing_openap.v1",
        _without_max_steps(execute_reduced_fixed_wing_composition),
    )
    registrar.register_execution_factory(
        "reduced_fixed_wing_f16_source.v1",
        _without_max_steps(execute_reduced_fixed_wing_composition),
    )
    registrar.register_execution_factory(
        "f16_local_physical_control_screen.v1",
        execute_f16_local_physical_control_screen,
    )
    registrar.register_execution_factory(
        "f16_local_physical_surface_lqi_screen.v1",
        _f16_local_physical_lqi_batch,
    )
    registrar.register_execution_factory(
        "f16_local_physical_surface_lqr_schedule_interior_screen.v1",
        _f16_physical_schedule_interior_batch,
    )
    registrar.register_execution_factory(
        "f16_local_physical_surface_lqi_schedule_interior_screen.v1",
        _f16_physical_schedule_interior_batch,
    )
    registrar.register_execution_factory(
        "f16_local_physical_surface_lqr_schedule_transition_screen.v1",
        _f16_physical_schedule_transition_batch,
    )
    registrar.register_execution_factory(
        "b747_condition3_local_physical_surface_lqr_screen.v1",
        _b747_local_physical_surface_batch,
    )
    registrar.register_execution_factory(
        "b747_condition3_local_physical_surface_lqi_screen.v1",
        _b747_local_physical_surface_batch,
    )
    registrar.register_execution_factory(
        "x8_local_physical_surface_lqr_screen.v1",
        _x8_local_physical_surface_batch,
    )
    registrar.register_execution_factory(
        "x8_local_physical_surface_lqi_screen.v1",
        _x8_local_physical_surface_batch,
    )
    registrar.register_execution_factory(
        "x8_local_physical_surface_lqi_long_recovery_screen.v1",
        _x8_local_physical_surface_batch,
    )
    registrar.register_execution_factory(
        "hl20_source_surface_pitch_authority_screen.v1",
        execute_hl20_source_surface_authority_screen,
    )
    registrar.register_execution_factory(
        "hl20_local_physical_surface_lqi_screen.v1",
        _hl20_local_physical_surface_batch,
    )
    registrar.register_execution_factory(
        "x15_source_surface_authority_screen.v1",
        execute_x15_source_surface_authority_screen,
    )
    registrar.register_execution_factory(
        "x15_local_physical_surface_lqi_screen.v1",
        _x15_local_physical_surface_batch,
    )
    registrar.register_execution_factory(
        "hummingbird_local_individual_rotor_lqi_screen.v1",
        _hummingbird_local_physical_batch,
    )
    registrar.register_execution_factory(
        "hummingbird_local_horizontal_translation_lqi_screen.v1",
        _hummingbird_local_physical_batch,
    )
    registrar.register_execution_factory(
        "hummingbird_local_vertical_translation_lqi_screen.v1",
        _hummingbird_local_physical_batch,
    )
    registrar.register_execution_factory(
        "hummingbird_aggregate_thrust_pseudo_batch.v1",
        _without_max_steps(execute_hummingbird_pseudo_composition),
    )
    registrar.register_execution_factory(
        "nesc_source_replay.v1",
        _without_max_steps(execute_nesc_source_replay_composition),
    )
    ####


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="taoryx.reference-models",
        package="taoryx-reference-models",
        version="0.1.0a0",
        api_version="1",
        description="Source-backed and analytical reference vehicle models.",
    ),
    register_callback=_register,
)

__all__ = ["PLUGIN"]
####
