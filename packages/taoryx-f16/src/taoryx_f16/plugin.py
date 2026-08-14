"""Plug-in registration for the source-grounded F-16 S.119 family.

The package deliberately owns the F-16 catalog fragment, source-reduction
runtime, controller screens, and their narrow local-controller evidence.  It
does not depend on the reference-model aggregate to plan or execute the F-16.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from functools import lru_cache
from typing import TYPE_CHECKING, Any

from taoryx.controller_tuning_registry import ControllerTuningCampaignRegistration
from taoryx.local_controller_screen_advertisements import LocalControllerScreenAdvertisement
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
_PROVIDER_ID = "taoryx.f16.mission-composition"
_PACKAGE_VERSION = "0.1.0a0"
_FAMILY_ID = "f16_s119"
_TRIM_EVIDENCE_FAMILY_ID = "reference_f16_s119"
_RACETRACK_TRANSLATOR_ID = "taoryx.f16_racetrack.source_route.v1"
_CATALOG_FRAGMENT = VehicleCatalogFragment(
    id="taoryx.f16.vehicle-catalog",
    resource_package="taoryx_f16",
    family_ids=(_FAMILY_ID,),
)


class _LazyMissionCapabilityAdapter:
    """Expose a stable F-16 capability identity without loading a source plant."""

    def __init__(self, identifier: str, factory: Callable[[], object]) -> None:
        self.id = identifier
        self._factory = factory
        self._delegate: object | None = None
        ####

    def _resolve(self) -> Any:
        """Build the family adapter only when a composition selects it."""

        if self._delegate is None:
            candidate = self._factory()
            if getattr(candidate, "id", None) != self.id:
                raise ValueError(f"lazy F-16 capability adapter {self.id!r} resolved a mismatched identity")
            self._delegate = candidate
        return self._delegate
        ####

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Delegate exact family/mission/fidelity ownership on demand."""

        return bool(self._resolve().supports(composition))
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> Any:
        """Delegate exact F-16 feasibility planning on demand."""

        return self._resolve().estimate(composition)
        ####

    ####


def _f16_racetrack_capability_adapter() -> object:
    """Load the source-owned reduced racetrack planner only when selected."""

    from taoryx.f16_mission_capability import F16SourceRacetrackCapabilityAdapter

    return F16SourceRacetrackCapabilityAdapter()
    ####


def _trim_evidence_binding() -> object:
    """Load the F-16 source trim binding only when trim evidence is selected."""

    from taoryx.f16_trim_evidence import trim_evidence_binding

    return trim_evidence_binding()
    ####


def _reduced_interface_extension() -> object:
    """Load F-16-only reduced controls/readbacks when its interface is selected."""

    from taoryx.f16_reduced_interface import f16_reduced_interface_extension

    return f16_reduced_interface_extension()
    ####


def _f16_local_physical_capability_adapter() -> object:
    """Load the bounded direct/surface local-screen planner on demand."""

    from taoryx.f16_local_physical_control_screen import F16LocalPhysicalControlScreenCapabilityAdapter

    return F16LocalPhysicalControlScreenCapabilityAdapter()
    ####


def _f16_local_physical_lqi_capability_adapter() -> object:
    """Load the fixed source-trim physical LQI planner on demand."""

    from taoryx.f16_local_physical_lqi_screen import F16LocalPhysicalLqiScreenCapabilityAdapter

    return F16LocalPhysicalLqiScreenCapabilityAdapter()
    ####


def _f16_schedule_interior_capability_adapter() -> object:
    """Load the held-node physical LQR schedule planner on demand."""

    from taoryx.f16_physical_schedule_interior_screen import F16PhysicalScheduleInteriorScreenCapabilityAdapter

    return F16PhysicalScheduleInteriorScreenCapabilityAdapter()
    ####


def _f16_schedule_interior_lqi_capability_adapter() -> object:
    """Load the held-node physical LQI schedule planner on demand."""

    from taoryx.f16_physical_schedule_interior_screen import F16PhysicalScheduleLqiInteriorScreenCapabilityAdapter

    return F16PhysicalScheduleLqiInteriorScreenCapabilityAdapter()
    ####


def _f16_schedule_transition_capability_adapter() -> object:
    """Load the bounded physical LQR transition planner on demand."""

    from taoryx.f16_physical_schedule_transition_screen import F16PhysicalScheduleTransitionScreenCapabilityAdapter

    return F16PhysicalScheduleTransitionScreenCapabilityAdapter()
    ####


def _preflight_f16_racetrack(composition: CompiledVehicleComposition) -> Any:
    """Reuse shared geometric checks after selecting the F-16-owned planner."""

    from taoryx.vehicle_execution_preflight import preflight_powered_fixed_wing_racetrack

    return preflight_powered_fixed_wing_racetrack(composition)
    ####


def _preflight_f16_local_physical_control_screen(composition: CompiledVehicleComposition) -> Any:
    """Load F-16 local direct/surface semantic preflight only when selected."""

    from taoryx.f16_local_physical_control_screen import preflight_f16_local_physical_control_screen

    return preflight_f16_local_physical_control_screen(composition)
    ####


def _preflight_f16_local_physical_lqi_screen(composition: CompiledVehicleComposition) -> Any:
    """Load F-16 local physical LQI semantic preflight only when selected."""

    from taoryx.f16_local_physical_lqi_screen import preflight_f16_local_physical_lqi_screen

    return preflight_f16_local_physical_lqi_screen(composition)
    ####


def _preflight_f16_schedule_interior_screen(composition: CompiledVehicleComposition) -> Any:
    """Load F-16 held-node schedule semantic preflight only when selected."""

    from taoryx.f16_physical_schedule_interior_screen import preflight_f16_physical_schedule_interior_screen

    return preflight_f16_physical_schedule_interior_screen(composition)
    ####


def _preflight_f16_schedule_transition_screen(composition: CompiledVehicleComposition) -> Any:
    """Load F-16 schedule-transition semantic preflight only when selected."""

    from taoryx.f16_physical_schedule_transition_screen import preflight_f16_physical_schedule_transition_screen

    return preflight_f16_physical_schedule_transition_screen(composition)
    ####


def _build_f16_source_physical_plant() -> object:
    """Load the F-16 source plant only when a physical adapter needs it."""

    from taoryx.source_f16 import build_f16_source_physical_plant

    return build_f16_source_physical_plant()
    ####


def _source_or_trim_probe(adapter: StandardFamilyAdapter) -> AdapterProbeCase:
    """Probe the source-local F-16 plant or its resolved source trim."""

    from taoryx.family_adapter_probes import AdapterProbeCase

    plant = adapter.plant
    if plant is None:
        raise ValueError(f"{adapter.describe().family_id}: adapter has no plant")
    if hasattr(plant, "source_local_state") and hasattr(plant, "source_effectors"):
        state = dict(getattr(plant, "source_local_state"))
        effectors = dict(getattr(plant, "source_effectors"))
        return AdapterProbeCase(
            state=state,
            effectors=effectors,
            trim_target=state,
            trim_initial_guess=effectors,
            previous_effectors=effectors,
        )
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
) -> Callable[[FidelityTier], StandardFamilyAdapter]:
    """Bind the source F-16 plant without a reference-package fallback."""

    @lru_cache(maxsize=1)
    def plant() -> object:
        return builder()
        ####

    @lru_cache(maxsize=None)
    def build(tier: FidelityTier) -> StandardFamilyAdapter:
        from taoryx.family_adapter import StandardFamilyAdapter, descriptor_from_control_plant

        source_plant = plant()
        control_names = getattr(source_plant, "control_names", ())
        limits = getattr(source_plant, "effectors", None)
        if not control_names or not isinstance(limits, dict):
            raise ValueError("f16_s119: source physical plant has no declared control limits")
        descriptor = descriptor_from_control_plant(
            source_plant,  # type: ignore[arg-type]
            family_id=_FAMILY_ID,
            adapter_id="taoryx.fixed_wing.daveml.v1",
            physical_family="powered_fixed_wing",
            tier=tier,
            control_units={name: limits[name].unit for name in control_names},
            evidence_status="development",
            omitted_physics=(
                "mission translation and gain scheduling",
                "full-flight-envelope and release qualification",
            ),
        )
        return StandardFamilyAdapter.from_control_plant(descriptor, source_plant)  # type: ignore[arg-type]
        ####

    return build
    ####


def _family_adapter_registration() -> FamilyAdapterRegistration:
    """Return the F-16 source-plant registration without constructing it."""

    from taoryx.family_adapter_registry import FamilyAdapterRegistration

    return FamilyAdapterRegistration(
        _FAMILY_ID,
        "taoryx.fixed_wing.daveml.v1",
        "available",
        _source_table_fixed_wing_factory(_build_f16_source_physical_plant),
        probe_factory=_source_or_trim_probe,
        supported_tiers=("rigid_body_6dof_direct_wrench", "rigid_body_6dof_surface_allocated"),
        note="runtime-owned source-backed first operating point; mission and schedule gates remain separate",
    )
    ####


def _provider() -> CatalogMissionCompositionProvider:
    """Build a portable F-16-only Mission Composition provider.

    The deferred plug-in proxy caches this object per discovery catalog.  Do
    not add a module-global cache here: an older aggregate host must not make
    a later focused host inherit its broader execution scope.
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
        provider_name="TAORYX F-16 Mission Composition",
        provider_short_name="F-16",
        provider_summary="Source-grounded F-16 S.119 model, controls, and composition configuration surface.",
        provider_version=_PACKAGE_VERSION,
        plugin_catalog=current_plugin_catalog(),
        include_builtin_workflows=False,
    )
    ####


def _f16_point_tuning_adapter() -> StandardFamilyAdapter:
    """Build the source-trim F-16 point-mass plant for the common runner."""

    from taoryx.f16_reduced_execution import _f16_source_trim
    from taoryx.trajectory.f16_reduced_adapter import build_f16_reduced_control_plant
    from taoryx.trajectory.f16_reductions import F16PointMass3DOFModel

    from taoryx.family_adapter import StandardFamilyAdapter, descriptor_from_control_plant

    source, trim, trim_pitch_rad = _f16_source_trim()
    plant = build_f16_reduced_control_plant(point_model=F16PointMass3DOFModel(source, trim, trim_pitch_rad))
    descriptor = descriptor_from_control_plant(
        plant,
        family_id=_FAMILY_ID,
        adapter_id="taoryx.fixed_wing.daveml.point_tuning.v1",
        physical_family="powered_fixed_wing",
        tier="point_mass_3dof",
    )
    return StandardFamilyAdapter.from_control_plant(descriptor, plant)
    ####


def _f16_pseudo_tuning_adapter() -> StandardFamilyAdapter:
    """Build the source-trim F-16 pseudo-6DOF plant for the common runner."""

    from taoryx.f16_reduced_execution import _f16_source_trim
    from taoryx.trajectory.f16_reduced_adapter import build_f16_reduced_control_plant
    from taoryx.trajectory.f16_reductions import F16AttitudeResponsePseudo6DOFModel

    from taoryx.family_adapter import StandardFamilyAdapter, descriptor_from_control_plant

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
        pseudo_model=F16AttitudeResponsePseudo6DOFModel(source, trim, linearization, trim_pitch_rad)
    )
    descriptor = descriptor_from_control_plant(
        plant,
        family_id=_FAMILY_ID,
        adapter_id="taoryx.fixed_wing.daveml.pseudo_tuning.v1",
        physical_family="powered_fixed_wing",
        tier="pseudo_6dof",
    )
    return StandardFamilyAdapter.from_control_plant(descriptor, plant)
    ####


def _f16_source_surface_tuning_adapter() -> StandardFamilyAdapter:
    """Build the source-trim F-16 physical-effector plant for common tuning."""

    return _source_table_fixed_wing_factory(_build_f16_source_physical_plant)("rigid_body_6dof_surface_allocated")
    ####


def _f16_source_surface_lqi_tuning_adapter() -> StandardFamilyAdapter:
    """Build the source-trim F-16 state-to-wrench LQI projection."""

    from taoryx.source_f16 import build_f16_local_physical_wrench_lqi_design

    from taoryx.physical_wrench_tuning import build_projected_physical_wrench_tuning_adapter

    return build_projected_physical_wrench_tuning_adapter(
        build_f16_local_physical_wrench_lqi_design(),
        family_id=_FAMILY_ID,
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

    from taoryx.source_f16 import (
        build_f16_source_physical_schedule_lqi_nodes,
        f16_source_physical_schedule_tuning_targets,
    )

    from taoryx.physical_wrench_tuning import build_scheduled_projected_physical_wrench_tuning_adapter

    nodes = build_f16_source_physical_schedule_lqi_nodes()
    return build_scheduled_projected_physical_wrench_tuning_adapter(
        {node.point_id: node.design for node in nodes},
        node_trim_targets=f16_source_physical_schedule_tuning_targets(),
        family_id=_FAMILY_ID,
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

    from taoryx.source_f16 import (
        build_f16_source_physical_schedule_nodes,
        f16_source_physical_schedule_tuning_targets,
    )

    from taoryx.physical_wrench_tuning import build_scheduled_projected_physical_wrench_tuning_adapter

    nodes = build_f16_source_physical_schedule_nodes()
    return build_scheduled_projected_physical_wrench_tuning_adapter(
        {node.point_id: node.design for node in nodes},
        node_trim_targets=f16_source_physical_schedule_tuning_targets(),
        family_id=_FAMILY_ID,
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


def _build_f16_point_tuning_campaign() -> TuningCampaign:
    """Load the F-16 point-mass campaign only if a caller selects it."""

    from taoryx.trajectory.f16_reduced_adapter import build_f16_point_tuning_campaign

    return build_f16_point_tuning_campaign()
    ####


def _build_f16_pseudo_tuning_campaign() -> TuningCampaign:
    """Load the F-16 pseudo-6DOF campaign only if a caller selects it."""

    from taoryx.trajectory.f16_reduced_adapter import build_f16_pseudo_tuning_campaign

    return build_f16_pseudo_tuning_campaign()
    ####


def _build_f16_source_surface_lqr_tuning_campaign() -> TuningCampaign:
    """Load the F-16 local physical LQR campaign on demand."""

    from taoryx.source_f16 import build_f16_source_surface_lqr_tuning_campaign

    return build_f16_source_surface_lqr_tuning_campaign()
    ####


def _build_f16_source_surface_lqi_tuning_campaign() -> TuningCampaign:
    """Load the F-16 local physical LQI campaign on demand."""

    from taoryx.source_f16 import build_f16_source_surface_lqi_tuning_campaign

    return build_f16_source_surface_lqi_tuning_campaign()
    ####


def _build_f16_source_surface_schedule_lqi_tuning_campaign() -> TuningCampaign:
    """Load the F-16 held-node physical LQI campaign on demand."""

    from taoryx.source_f16 import build_f16_source_physical_schedule_lqi_tuning_campaign

    return build_f16_source_physical_schedule_lqi_tuning_campaign()
    ####


def _build_f16_source_surface_schedule_lqr_tuning_campaign() -> TuningCampaign:
    """Load the F-16 schedule LQR campaign on demand."""

    from taoryx.source_f16 import build_f16_source_physical_schedule_lqr_tuning_campaign

    return build_f16_source_physical_schedule_lqr_tuning_campaign()
    ####


def _surface_effectors() -> list[dict[str, object]]:
    """Return the declared F-16 actuator overlay without synthesized limits."""

    return [
        {
            "id": "effector.elevator.position",
            "native_control_id": "elevator_deg",
            "unit": "deg",
            "lower": -25.0,
            "upper": 25.0,
            "rate_limit_per_s": 60.0,
            "time_constant_s": 0.08,
        },
        {
            "id": "effector.aileron.position",
            "native_control_id": "aileron_deg",
            "unit": "deg",
            "lower": -21.0,
            "upper": 21.0,
            "rate_limit_per_s": 80.0,
            "time_constant_s": 0.08,
        },
        {
            "id": "effector.rudder.position",
            "native_control_id": "rudder_deg",
            "unit": "deg",
            "lower": -30.0,
            "upper": 30.0,
            "rate_limit_per_s": 80.0,
            "time_constant_s": 0.08,
        },
        {
            "id": "effector.throttle.position",
            "native_control_id": "throttle_fraction",
            "unit": "1",
            "lower": 0.0,
            "upper": 1.0,
            "rate_limit_per_s": 2.5,
            "time_constant_s": 0.08,
        },
    ]
    ####


def _surface_screen(
    *,
    identifier: str,
    mission_template_id: str,
    method: str,
    campaign_id: str,
    integral_output_names: list[str],
    fixed_cadence_s: float,
    screen_duration_s: float,
    claim_boundary: str,
    selection: str | None = None,
) -> dict[str, object]:
    """Build one complete static physical-control advertisement."""

    controller: dict[str, object] = {
        "method": method,
        "campaign_id": campaign_id,
        "integral_output_names": integral_output_names,
        "fixed_cadence_s": fixed_cadence_s,
        "screen_duration_s": screen_duration_s,
    }
    if selection is not None:
        controller["selection"] = selection
    return {
        "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
        "id": identifier,
        "mission_template_id": mission_template_id,
        "fidelity": "rigid_body_6dof_surface_allocated",
        "operations": ["batch"],
        "control_realization": "surface_allocated",
        "physical_effector_allocation": True,
        "batch_action_trace": "emits_committed_interval_trace",
        "controller": controller,
        "effector_controls": _surface_effectors(),
        "claim_boundary": claim_boundary,
    }
    ####


def _campaigns() -> tuple[ControllerTuningCampaignRegistration, ...]:
    """Declare F-16-local reduced and physical controller campaigns."""

    aliases = (_AGGREGATE_PROVIDER_ID,)
    return (
        ControllerTuningCampaignRegistration(
            id="f16-point-source-trim-translation-v1",
            provider_id=_PROVIDER_ID,
            provider_aliases=aliases,
            model_id=_FAMILY_ID,
            family_id=_FAMILY_ID,
            fidelity="point_mass_3dof",
            realization_ids=("point_mass_3dof",),
            mission_template_ids=("powered_fixed_wing_racetrack_v1",),
            description="Scaled LQR source-trim translational campaign over the declared F-16 point-mass reduction.",
            adapter_factory=_f16_point_tuning_adapter,
            campaign_factory=_build_f16_point_tuning_campaign,
            claim_boundary=(
                "This campaign covers the local source-force point-mass reduction around its pinned trim. It does not "
                "establish route guidance, physical actuator allocation, an envelope, or flight qualification."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="f16-pseudo-source-trim-attitude-v1",
            provider_id=_PROVIDER_ID,
            provider_aliases=aliases,
            model_id=_FAMILY_ID,
            family_id=_FAMILY_ID,
            fidelity="pseudo_6dof",
            realization_ids=("pseudo_6dof",),
            mission_template_ids=("powered_fixed_wing_racetrack_v1",),
            description="Scaled LQI source-trim attitude-response campaign over the declared F-16 pseudo-6DOF reduction.",
            adapter_factory=_f16_pseudo_tuning_adapter,
            campaign_factory=_build_f16_pseudo_tuning_campaign,
            claim_boundary=(
                "This campaign covers the named source-local attitude/rate response reduction. It does not establish "
                "physical F-16 effector allocation, route tracking, an envelope, or flight qualification."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="f16-source-surface-local-lqr-v1",
            provider_id=_PROVIDER_ID,
            provider_aliases=aliases,
            model_id=_FAMILY_ID,
            family_id=_FAMILY_ID,
            fidelity="rigid_body_6dof_surface_allocated",
            realization_ids=("rigid_body_6dof_surface_allocated",),
            mission_template_ids=("f16_local_physical_control_screen_v1",),
            description="Scaled LQR source-trim local campaign over declared F-16 elevator, aileron, rudder, and throttle coordinates.",
            adapter_factory=_f16_source_surface_tuning_adapter,
            campaign_factory=_build_f16_source_surface_lqr_tuning_campaign,
            local_controller_screens=(
                _surface_screen(
                    identifier="f16-source-trim-surface-lqr-screen-v1",
                    mission_template_id="f16_local_physical_control_screen_v1",
                    method="lqr",
                    campaign_id="f16-source-surface-local-lqr-v1",
                    integral_output_names=[],
                    fixed_cadence_s=0.2,
                    screen_duration_s=1.0,
                    claim_boundary=(
                        "One source-trim physical allocation LQR entry screen. It is not an F-16 route, schedule, "
                        "wind/mass robustness, envelope, or qualification claim."
                    ),
                ),
            ),
            claim_boundary=(
                "This campaign exposes one source-trim physical-effector design screen. It does not establish a gain "
                "schedule, navigation, wind/mass robustness, envelope coverage, or flight qualification."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="f16-source-surface-schedule-lqr-v1",
            provider_id=_PROVIDER_ID,
            provider_aliases=aliases,
            model_id=_FAMILY_ID,
            family_id=_FAMILY_ID,
            fidelity="rigid_body_6dof_surface_allocated",
            realization_ids=("rigid_body_6dof_surface_allocated",),
            mission_template_ids=(
                "f16_local_physical_surface_lqr_schedule_interior_screen_v1",
                "f16_local_physical_surface_lqr_schedule_transition_screen_v1",
            ),
            description=(
                "Four exact source-node LQR candidates in the requested-force/moment coordinates consumed by the F-16 "
                "held-node and interpolated surface/throttle schedule runtimes."
            ),
            adapter_factory=_f16_source_surface_schedule_lqr_tuning_adapter,
            campaign_factory=_build_f16_source_surface_schedule_lqr_tuning_campaign,
            local_controller_screens=(
                _surface_screen(
                    identifier="f16-source-node-surface-lqr-schedule-interior-screen-v1",
                    mission_template_id="f16_local_physical_surface_lqr_schedule_interior_screen_v1",
                    method="lqr",
                    campaign_id="f16-source-surface-schedule-lqr-v1",
                    integral_output_names=[],
                    fixed_cadence_s=0.02,
                    screen_duration_s=16.0,
                    selection="discrete_source_node_held_for_each_recovery",
                    claim_boundary=(
                        "Eight independent held-node source-retrimmed F-16 physical LQR recoveries through bounded surface "
                        "allocation. It is not continuous gain interpolation, a node transition, a route, a full envelope, "
                        "or qualification."
                    ),
                ),
                _surface_screen(
                    identifier="f16-source-node-surface-lqr-schedule-transition-screen-v1",
                    mission_template_id="f16_local_physical_surface_lqr_schedule_transition_screen_v1",
                    method="lqr",
                    campaign_id="f16-source-surface-schedule-lqr-v1",
                    integral_output_names=[],
                    fixed_cadence_s=0.05,
                    screen_duration_s=240.0,
                    selection="linearly_interpolated_source_lqr_schedule_by_altitude_coordinate",
                    claim_boundary=(
                        "Four retained time-marching source-node F-16 physical LQR transitions with exact candidate gains, "
                        "explicit endpoint derivative/effectiveness blending, and bounded surface allocation. It is not "
                        "navigation, wind/mass robustness, a full envelope, or qualification."
                    ),
                ),
            ),
            claim_boundary=(
                "This campaign applies one selected candidate at each retained source node before a schedule runtime "
                "holds or interpolates those gains. It does not establish navigation, wind/mass robustness, an envelope, "
                "or flight qualification."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="f16-source-surface-local-lqi-v1",
            provider_id=_PROVIDER_ID,
            provider_aliases=aliases,
            model_id=_FAMILY_ID,
            family_id=_FAMILY_ID,
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
            campaign_factory=_build_f16_source_surface_lqi_tuning_campaign,
            local_controller_screens=(
                _surface_screen(
                    identifier="f16-source-trim-surface-lqi-screen-v1",
                    mission_template_id="f16_local_physical_surface_lqi_screen_v1",
                    method="lqi",
                    campaign_id="f16-source-surface-local-lqi-v1",
                    integral_output_names=["u_m_s", "v_m_s", "w_m_s"],
                    fixed_cadence_s=0.02,
                    screen_duration_s=5.0,
                    claim_boundary=(
                        "One fixed-altitude source-trim physical allocation LQI recovery. It is not an F-16 route, "
                        "wind/mass robustness, envelope, or qualification claim."
                    ),
                ),
            ),
            claim_boundary=(
                "This campaign includes the fixed source-trim local physical-effector LQI design screen. It does not "
                "establish a gain schedule, envelope robustness, or flight qualification."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="f16-source-surface-schedule-lqi-v1",
            provider_id=_PROVIDER_ID,
            provider_aliases=aliases,
            model_id=_FAMILY_ID,
            family_id=_FAMILY_ID,
            fidelity="rigid_body_6dof_surface_allocated",
            realization_ids=("rigid_body_6dof_surface_allocated",),
            mission_template_ids=("f16_local_physical_surface_lqi_schedule_interior_screen_v1",),
            description=(
                "Four exact source-node LQI candidates in the requested-force/moment coordinates consumed by the held-node "
                "F-16 surface/throttle allocator."
            ),
            adapter_factory=_f16_source_surface_schedule_lqi_tuning_adapter,
            campaign_factory=_build_f16_source_surface_schedule_lqi_tuning_campaign,
            local_controller_screens=(
                _surface_screen(
                    identifier="f16-source-node-surface-lqi-schedule-interior-screen-v1",
                    mission_template_id="f16_local_physical_surface_lqi_schedule_interior_screen_v1",
                    method="lqi",
                    campaign_id="f16-source-surface-schedule-lqi-v1",
                    integral_output_names=["u_m_s", "v_m_s", "w_m_s"],
                    fixed_cadence_s=0.02,
                    screen_duration_s=16.0,
                    selection="discrete_source_node_held_for_each_recovery",
                    claim_boundary=(
                        "Eight independent held-node source-retrimmed F-16 physical LQI recoveries through bounded surface "
                        "allocation, restricted to the source-feasible plus/minus 0.25 m/s body-w interior. It is not "
                        "continuous gain interpolation, a node transition, a route, a full envelope, wind/mass rejection, "
                        "or qualification."
                    ),
                ),
            ),
            claim_boundary=(
                "This campaign creates and applies a separately selected source-node candidate to each retained F-16 "
                "LQI recovery. It does not establish continuous gain scheduling, transition control, envelope robustness, "
                "or flight qualification."
            ),
        ),
    )
    ####


def _static_direct_wrench_screen() -> LocalControllerScreenAdvertisement:
    """Advertise the retained F-16 direct-wrench comparator without a tuner claim."""

    return LocalControllerScreenAdvertisement(
        id="f16-source-trim-direct-wrench-lqr-screen-v1",
        provider_id=_PROVIDER_ID,
        provider_aliases=(_AGGREGATE_PROVIDER_ID,),
        model_id=_FAMILY_ID,
        family_id=_FAMILY_ID,
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
                "This pinned source-trim LQR internally generates only four generalized-wrench coordinates. The source "
                "profile declares normalization scales, not hard wrench bounds or caller override; it is neither a "
                "physical F-16 effector allocator nor a scheduled flight controller."
            ),
        },
    )
    ####


def _open_f16_reduced_episode(
    composition: CompiledVehicleComposition,
    seed: int | None,
    integration_step_s: float,
) -> object:
    """Open the F-16-owned reduced interactive runtime only when selected."""

    from taoryx.f16_composition_episode import open_f16_reduced_composition_episode

    return open_f16_reduced_composition_episode(composition, seed, integration_step_s)
    ####


def _verify_f16_reduced_batch_episode_parity(
    composition: CompiledVehicleComposition,
    payload: Mapping[str, object],
) -> object:
    """Defer exact F-16 batch/episode parity to its family implementation."""

    from taoryx.f16_reduced_batch_episode_parity import verify_serialized_f16_reduced_batch_episode_parity

    return verify_serialized_f16_reduced_batch_episode_parity(composition, payload)
    ####


def _f16_reduced_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Run an F-16 lower-fidelity source reduction through the typed host request."""

    if request.max_steps is not None:
        raise ValueError("--max-steps is not available for the F-16 reduced source batch")
    from taoryx.f16_reduced_execution import execute_f16_reduced_composition

    return execute_f16_reduced_composition(request.composition, request.output_dir)
    ####


def _f16_local_physical_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Run the F-16 direct/surface local controller screen on demand."""

    from taoryx.f16_local_physical_control_screen import execute_f16_local_physical_control_screen

    return execute_f16_local_physical_control_screen(request.composition, request.output_dir, request.max_steps)
    ####


def _f16_local_physical_lqi_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Run the F-16 local physical LQI screen with an optional exact candidate."""

    from taoryx.f16_local_physical_lqi_screen import execute_f16_local_physical_lqi_screen

    return execute_f16_local_physical_lqi_screen(
        request.composition,
        request.output_dir,
        request.max_steps,
        tuning_context=request.tuning_context,
    )
    ####


def _f16_schedule_interior_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Run an F-16 held-node schedule only with a complete node context set."""

    if request.tuning_context is not None:
        raise ValueError("F-16 schedule-interior screens require a complete node-indexed tuning context set")
    from taoryx.f16_physical_schedule_interior_screen import execute_f16_physical_schedule_interior_screen

    return execute_f16_physical_schedule_interior_screen(
        request.composition,
        request.output_dir,
        request.max_steps,
        tuning_context_set=request.tuning_context_set,
    )
    ####


def _f16_schedule_transition_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Run an F-16 schedule transition only with a complete node context set."""

    if request.tuning_context is not None:
        raise ValueError("F-16 schedule-transition screens require a complete node-indexed tuning context set")
    from taoryx.f16_physical_schedule_transition_screen import execute_f16_physical_schedule_transition_screen

    return execute_f16_physical_schedule_transition_screen(
        request.composition,
        request.output_dir,
        request.max_steps,
        tuning_context_set=request.tuning_context_set,
    )
    ####


def _register(registrar: PluginRegistrar) -> None:
    """Publish only F-16-owned model, controls, translation, and execution seams."""

    registrar.register_vehicle_catalog_fragment(_CATALOG_FRAGMENT)
    registration = registrar.register_family_adapter_factory(_FAMILY_ID, _family_adapter_registration)
    registrar.register_model(_FAMILY_ID, registration)
    registrar.register_vehicle_interface_extension_factory(_FAMILY_ID, _reduced_interface_extension)
    registrar.register_trim_evidence_binding_factory(_TRIM_EVIDENCE_FAMILY_ID, _trim_evidence_binding)
    registrar.register_mission_composition_provider_factory(_PROVIDER_ID, _provider)
    for adapter in (
        _LazyMissionCapabilityAdapter(_RACETRACK_TRANSLATOR_ID, _f16_racetrack_capability_adapter),
        _LazyMissionCapabilityAdapter(
            "taoryx.f16_local_physical_control_screen.capability.v1",
            _f16_local_physical_capability_adapter,
        ),
        _LazyMissionCapabilityAdapter(
            "taoryx.f16_local_physical_surface_lqi_screen.capability.v1",
            _f16_local_physical_lqi_capability_adapter,
        ),
        _LazyMissionCapabilityAdapter(
            "taoryx.f16_local_physical_surface_lqr_schedule_interior_screen.capability.v1",
            _f16_schedule_interior_capability_adapter,
        ),
        _LazyMissionCapabilityAdapter(
            "taoryx.f16_local_physical_surface_lqi_schedule_interior_screen.capability.v1",
            _f16_schedule_interior_lqi_capability_adapter,
        ),
        _LazyMissionCapabilityAdapter(
            "taoryx.f16_local_physical_surface_lqr_schedule_transition_screen.capability.v1",
            _f16_schedule_transition_capability_adapter,
        ),
    ):
        registrar.register_mission_capability_adapter(adapter)
    for campaign in _campaigns():
        registrar.register_controller_tuning_campaign(campaign)
    registrar.register_local_controller_screen_advertisement(_static_direct_wrench_screen())
    for translator_id, handler in (
        (_RACETRACK_TRANSLATOR_ID, _preflight_f16_racetrack),
        ("taoryx.f16_local_physical_control_screen.capability.v1", _preflight_f16_local_physical_control_screen),
        ("taoryx.f16_local_physical_surface_lqi_screen.capability.v1", _preflight_f16_local_physical_lqi_screen),
        (
            "taoryx.f16_local_physical_surface_lqr_schedule_interior_screen.capability.v1",
            _preflight_f16_schedule_interior_screen,
        ),
        (
            "taoryx.f16_local_physical_surface_lqi_schedule_interior_screen.capability.v1",
            _preflight_f16_schedule_interior_screen,
        ),
        (
            "taoryx.f16_local_physical_surface_lqr_schedule_transition_screen.capability.v1",
            _preflight_f16_schedule_transition_screen,
        ),
    ):
        registrar.register_semantic_preflight_handler_callback(translator_id, handler)
    registrar.register_episode_factory("reduced_fixed_wing_f16_episode.v1", _open_f16_reduced_episode)
    registrar.register_batch_episode_parity_verifier(
        "taoryx.reduced_fixed_wing.f16_action_trace_batch_episode_parity.v1",
        _verify_f16_reduced_batch_episode_parity,
    )
    registrar.register_execution_factory_request_v1("reduced_fixed_wing_f16_source.v1", _f16_reduced_batch)
    registrar.register_execution_factory_request_v1("f16_local_physical_control_screen.v1", _f16_local_physical_batch)
    registrar.register_execution_factory_request_v1("f16_local_physical_surface_lqi_screen.v1", _f16_local_physical_lqi_batch)
    registrar.register_execution_factory_request_v1(
        "f16_local_physical_surface_lqr_schedule_interior_screen.v1",
        _f16_schedule_interior_batch,
    )
    registrar.register_execution_factory_request_v1(
        "f16_local_physical_surface_lqi_schedule_interior_screen.v1",
        _f16_schedule_interior_batch,
    )
    registrar.register_execution_factory_request_v1(
        "f16_local_physical_surface_lqr_schedule_transition_screen.v1",
        _f16_schedule_transition_batch,
    )
    ####


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="taoryx.f16",
        package="taoryx-f16",
        version=_PACKAGE_VERSION,
        api_version="1",
        description="Source-grounded F-16 S.119 model, controls, and composition runtime.",
    ),
    register_callback=_register,
)

__all__ = ["PLUGIN"]
