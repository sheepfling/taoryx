"""Plug-in registration for the source-table X8 and B747 families.

The registration surface intentionally advertises stable identities without
constructing a source plant, parsing table data, or building a Pydantic model
schema. Those operations occur only when a host selects a family adapter,
focused provider, capability estimate, or execution endpoint.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from functools import lru_cache
from typing import TYPE_CHECKING, Any

from taoryx.controller_tuning_registry import ControllerTuningCampaignRegistration
from taoryx.plugins import PluginDefinition, PluginMetadata, PluginRegistrar, VehicleCatalogFragment, current_plugin_catalog

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

_X8_FAMILY_ID = "skywalker_x8"
_B747_FAMILY_ID = "b747"
_X8_PROVIDER_ID = "taoryx.x8.mission-composition"
_B747_PROVIDER_ID = "taoryx.b747.mission-composition"
_AGGREGATE_PROVIDER_ID = "taoryx.registry.mission-composition"
_PACKAGE_VERSION = "0.1.0a0"
_CATALOG_FRAGMENT = VehicleCatalogFragment(
    id="taoryx.source-table-fixed-wing.vehicle-catalog",
    resource_package="taoryx_source_table_fixed_wing",
    family_ids=(_X8_FAMILY_ID, _B747_FAMILY_ID),
)


class _LazyMissionCapabilityAdapter:
    """Advertise an exact capability identity without eagerly loading a plant."""

    def __init__(self, identifier: str, factory: Callable[[], object]) -> None:
        self.id = identifier
        self._factory = factory
        self._delegate: object | None = None
        ####

    def _resolve(self) -> Any:
        if self._delegate is None:
            candidate = self._factory()
            if getattr(candidate, "id", None) != self.id:
                raise ValueError(f"lazy source-table fixed-wing capability {self.id!r} resolved a mismatched identity")
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


def _source_local_probe(adapter: StandardFamilyAdapter) -> AdapterProbeCase:
    """Return the plant-owned source operating point for generic probes."""

    from taoryx.family_adapter_probes import AdapterProbeCase

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
        from taoryx.family_adapter import StandardFamilyAdapter, descriptor_from_control_plant

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


def _build_x8_source_table_plant() -> object:
    """Load the X8 source plant only when its adapter is selected."""

    from taoryx.source_table_fixed_wing import build_x8_source_table_plant

    return build_x8_source_table_plant()
    ####


def _build_b747_source_table_plant() -> object:
    """Load the B747 source plant only when its adapter is selected."""

    from taoryx.source_table_fixed_wing import build_b747_condition3_source_table_plant

    return build_b747_condition3_source_table_plant()
    ####


def _family_adapter_registration(family_id: str) -> FamilyAdapterRegistration:
    """Build one source-table adapter registration only when the host needs it."""

    from taoryx.family_adapter_registry import FamilyAdapterRegistration

    if family_id == _X8_FAMILY_ID:
        return FamilyAdapterRegistration(
            _X8_FAMILY_ID,
            "taoryx.fixed_wing.source_table.v1",
            "available",
            _source_table_fixed_wing_factory(_build_x8_source_table_plant, family_id=_X8_FAMILY_ID),
            probe_factory=_source_local_probe,
            supported_tiers=("rigid_body_6dof_direct_wrench", "rigid_body_6dof_surface_allocated"),
            note="pinned source-table local plant; semantic mission translation remains a separate gate",
        )
    if family_id == _B747_FAMILY_ID:
        return FamilyAdapterRegistration(
            _B747_FAMILY_ID,
            "taoryx.fixed_wing.source_table.v1",
            "available",
            _source_table_fixed_wing_factory(_build_b747_source_table_plant, family_id=_B747_FAMILY_ID),
            probe_factory=_source_local_probe,
            supported_tiers=("rigid_body_6dof_direct_wrench", "rigid_body_6dof_surface_allocated"),
            note="pinned NASA condition-3 source-table plant; semantic mission translation remains a separate gate",
        )
    raise ValueError(f"unsupported source-table fixed-wing family {family_id!r}")
    ####


def _family_adapter_registration_factory(family_id: str) -> Callable[[], FamilyAdapterRegistration]:
    """Bind one lazy adapter factory to its advertised source-table family."""

    def build() -> FamilyAdapterRegistration:
        return _family_adapter_registration(family_id)
        ####

    return build
    ####


def _x8_source_surface_tuning_adapter() -> StandardFamilyAdapter:
    """Build the exact X8 physical-wrench runtime projection for tuning."""

    from taoryx.source_table_fixed_wing import build_x8_source_surface_physical_lqi_design

    from taoryx.physical_wrench_tuning import build_projected_physical_wrench_tuning_adapter

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

    from taoryx.family_adapter import StandardFamilyAdapter, descriptor_from_control_plant
    from taoryx.trajectory.language_backed_guidance import LanguageBackedGuidanceControlPlant

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

    from taoryx.family_adapter import StandardFamilyAdapter, descriptor_from_control_plant
    from taoryx.trajectory.language_backed_guidance import LanguageBackedGuidanceControlPlant

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

    from taoryx.family_adapter import StandardFamilyAdapter, descriptor_from_control_plant
    from taoryx.trajectory.language_backed_guidance import LanguageBackedPseudoGuidanceControlPlant

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

    from taoryx.source_table_fixed_wing import build_b747_condition3_source_surface_physical_lqi_design

    from taoryx.physical_wrench_tuning import build_projected_physical_wrench_tuning_adapter

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


def _build_b747_source_surface_lqi_tuning_campaign() -> TuningCampaign:
    """Load the B747 source-local campaign only when tuning selects it."""

    from taoryx.source_table_fixed_wing import build_b747_source_surface_lqi_tuning_campaign

    return build_b747_source_surface_lqi_tuning_campaign()
    ####


def _build_x8_source_surface_lqi_tuning_campaign() -> TuningCampaign:
    """Load the X8 source-local campaign only when tuning selects it."""

    from taoryx.source_table_fixed_wing import build_x8_source_surface_lqi_tuning_campaign

    return build_x8_source_surface_lqi_tuning_campaign()
    ####


def _build_language_backed_guidance_tuning_campaign(family_id: str) -> TuningCampaign:
    """Load one reduced fixed-wing guidance campaign on demand."""

    from taoryx.trajectory.language_backed_guidance import build_language_backed_guidance_tuning_campaign

    return build_language_backed_guidance_tuning_campaign(family_id)
    ####


def _build_language_backed_pseudo_guidance_tuning_campaign(family_id: str) -> TuningCampaign:
    """Load one pseudo-6DOF guidance campaign on demand."""

    from taoryx.trajectory.language_backed_guidance import build_language_backed_pseudo_guidance_tuning_campaign

    return build_language_backed_pseudo_guidance_tuning_campaign(family_id)
    ####


def _controller_tuning_campaigns() -> tuple[ControllerTuningCampaignRegistration, ...]:
    """Advertise executable campaigns while leaving the runner in core."""

    return (
        ControllerTuningCampaignRegistration(
            id="b747-source-surface-local-lqi-v1",
            provider_id=_B747_PROVIDER_ID,
            provider_aliases=(_AGGREGATE_PROVIDER_ID,),
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
            campaign_factory=_build_b747_source_surface_lqi_tuning_campaign,
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
            provider_id=_B747_PROVIDER_ID,
            provider_aliases=(_AGGREGATE_PROVIDER_ID,),
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
            campaign_factory=lambda: _build_language_backed_guidance_tuning_campaign(_B747_FAMILY_ID),
            claim_boundary=(
                "This campaign covers only the native lower-tier commanded-state guidance response around the "
                "pinned level racetrack point. It does not tune source throttle/surfaces, the pseudo-6DOF attitude "
                "sidecar, wind rejection, route tracking, or physical B747 actuators."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="b747-language-backed-pseudo-guidance-local-lqi-v1",
            provider_id=_B747_PROVIDER_ID,
            provider_aliases=(_AGGREGATE_PROVIDER_ID,),
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
            campaign_factory=lambda: _build_language_backed_pseudo_guidance_tuning_campaign(_B747_FAMILY_ID),
            claim_boundary=(
                "This campaign covers only the native lower-tier commanded-state guidance response and declared "
                "kinematic attitude sidecar around the pinned level point. It does not tune source throttle/surfaces, "
                "wind rejection, route tracking, or physical B747 actuators."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="x8-source-surface-local-lqi-v1",
            provider_id=_X8_PROVIDER_ID,
            provider_aliases=(_AGGREGATE_PROVIDER_ID,),
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
            campaign_factory=_build_x8_source_surface_lqi_tuning_campaign,
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
            provider_id=_X8_PROVIDER_ID,
            provider_aliases=(_AGGREGATE_PROVIDER_ID,),
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
            campaign_factory=lambda: _build_language_backed_guidance_tuning_campaign(_X8_FAMILY_ID),
            claim_boundary=(
                "This campaign covers only the native lower-tier commanded-state guidance response around the "
                "pinned level racetrack point. It does not tune source throttle/elevons, the pseudo-6DOF attitude "
                "sidecar, wind rejection, route tracking, or physical X8 actuators."
            ),
        ),
        ControllerTuningCampaignRegistration(
            id="x8-language-backed-pseudo-guidance-local-lqi-v1",
            provider_id=_X8_PROVIDER_ID,
            provider_aliases=(_AGGREGATE_PROVIDER_ID,),
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
            campaign_factory=lambda: _build_language_backed_pseudo_guidance_tuning_campaign(_X8_FAMILY_ID),
            claim_boundary=(
                "This campaign covers only the native lower-tier commanded-state guidance response and declared "
                "kinematic attitude sidecar around the pinned level point. It does not tune source throttle/elevons, "
                "wind rejection, route tracking, or physical X8 actuators."
            ),
        ),
    )
    ####


def _b747_racetrack_capability_adapter() -> object:
    from taoryx.source_table_fixed_wing_mission_capability import B747SourceRacetrackCapabilityAdapter

    return B747SourceRacetrackCapabilityAdapter()
    ####


def _x8_racetrack_capability_adapter() -> object:
    from taoryx.source_table_fixed_wing_mission_capability import X8SourceRacetrackCapabilityAdapter

    return X8SourceRacetrackCapabilityAdapter()
    ####


def _b747_lqr_capability_adapter() -> object:
    from taoryx.b747_local_physical_control_screen import B747LocalPhysicalControlScreenCapabilityAdapter

    return B747LocalPhysicalControlScreenCapabilityAdapter()
    ####


def _b747_lqi_capability_adapter() -> object:
    from taoryx.b747_local_physical_control_screen import B747LocalPhysicalLqiControlScreenCapabilityAdapter

    return B747LocalPhysicalLqiControlScreenCapabilityAdapter()
    ####


def _x8_lqr_capability_adapter() -> object:
    from taoryx.x8_local_physical_control_screen import X8LocalPhysicalControlScreenCapabilityAdapter

    return X8LocalPhysicalControlScreenCapabilityAdapter()
    ####


def _x8_lqi_capability_adapter() -> object:
    from taoryx.x8_local_physical_control_screen import X8LocalPhysicalLqiControlScreenCapabilityAdapter

    return X8LocalPhysicalLqiControlScreenCapabilityAdapter()
    ####


def _x8_long_recovery_lqi_capability_adapter() -> object:
    from taoryx.x8_local_physical_control_screen import X8LocalPhysicalLongRecoveryLqiScreenCapabilityAdapter

    return X8LocalPhysicalLongRecoveryLqiScreenCapabilityAdapter()
    ####


def _preflight_powered_fixed_wing_racetrack(composition: CompiledVehicleComposition) -> Any:
    from taoryx.vehicle_execution_preflight import preflight_powered_fixed_wing_racetrack

    return preflight_powered_fixed_wing_racetrack(composition)
    ####


def _preflight_b747_local_screen(composition: CompiledVehicleComposition) -> Any:
    from taoryx.b747_local_physical_control_screen import preflight_b747_condition3_local_physical_control_screen

    return preflight_b747_condition3_local_physical_control_screen(composition)
    ####


def _preflight_x8_local_screen(composition: CompiledVehicleComposition) -> Any:
    from taoryx.x8_local_physical_control_screen import preflight_x8_local_physical_control_screen

    return preflight_x8_local_physical_control_screen(composition)
    ####


def _open_language_backed_episode(
    composition: CompiledVehicleComposition,
    seed: int | None,
    integration_step_s: float,
) -> object:
    """Load the shared interactive route episode only when selected."""

    from .episode import SourceTableFixedWingCompositionEpisode

    del integration_step_s
    return SourceTableFixedWingCompositionEpisode(composition, seed=seed)
    ####


def _verify_language_backed_batch_episode_parity(
    composition: CompiledVehicleComposition,
    payload: Mapping[str, object],
) -> object:
    """Load the common parity verifier only when a route witness requests it."""

    from taoryx.language_backed_batch_episode_parity import verify_serialized_language_backed_batch_episode_parity
    from taoryx.language_backed_racetrack import LanguageBackedRacetrackAssets

    return verify_serialized_language_backed_batch_episode_parity(
        composition,
        payload,
        assets=LanguageBackedRacetrackAssets.from_root(model_resource_root()),
        plugins=current_plugin_catalog(),
    )
    ####


def _language_backed_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Run language-backed X8/B747 execution through the typed request contract."""

    from taoryx.language_backed_execution import execute_powered_fixed_wing_composition
    from taoryx.language_backed_racetrack import LanguageBackedRacetrackAssets

    return execute_powered_fixed_wing_composition(
        request.composition,
        request.output_dir,
        max_steps=request.max_steps,
        assets=LanguageBackedRacetrackAssets.from_root(model_resource_root()),
        plugins=current_plugin_catalog(),
    )
    ####


def _x8_local_physical_surface_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Run the X8 source screen with an optional exact LQI candidate context."""

    from taoryx.x8_local_physical_control_screen import execute_x8_local_physical_control_screen

    return execute_x8_local_physical_control_screen(
        request.composition,
        request.output_dir,
        max_steps=request.max_steps,
        tuning_context=request.tuning_context,
    )
    ####


def _b747_local_physical_surface_batch(request: VehicleBatchExecutionRequest) -> Any:
    """Run a B747 source screen with its exact optional LQI candidate."""

    from taoryx.b747_local_physical_control_screen import execute_b747_condition3_local_physical_control_screen

    return execute_b747_condition3_local_physical_control_screen(
        request.composition,
        request.output_dir,
        max_steps=request.max_steps,
        tuning_context=request.tuning_context,
    )
    ####


def _focused_provider(family_id: str) -> CatalogMissionCompositionProvider:
    """Build one family-only provider from this package's two-fragment catalog.

    The deferred plug-in proxy caches each provider per discovery catalog. Do
    not cache this factory globally: a broad host must not make a later
    focused X8 or B747 host inherit its wider execution scope.
    """

    from taoryx.family_manifest import load_unified_family_manifest_catalog
    from taoryx.horizontal_fidelity import load_horizontal_registry
    from taoryx.trajectory.catalog_mission_composition import CatalogMissionCompositionProvider
    from taoryx.trajectory.pseudo6dof_profiles import load_pseudo6dof_catalog
    from taoryx.vehicle_composition_registry import (
        ResolvedVehicleCompositionCatalog,
        load_resolved_vehicle_composition_catalog,
        load_vehicle_composition_registry,
    )

    metadata = {
        _X8_FAMILY_ID: (
            _X8_PROVIDER_ID,
            "TAORYX Skywalker X8 Mission Composition",
            "Skywalker X8",
            "Skywalker X8-only source-table and reduced guidance composition surface.",
        ),
        _B747_FAMILY_ID: (
            _B747_PROVIDER_ID,
            "TAORYX B747 Mission Composition",
            "B747",
            "B747-only source-table and reduced guidance composition surface.",
        ),
    }
    try:
        provider_id, provider_name, provider_short_name, provider_summary = metadata[family_id]
    except KeyError as error:
        raise ValueError(f"unsupported source-table fixed-wing provider family {family_id!r}") from error
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
    selected = tuple(item for item in catalog.vehicles if item.family.family_id == family_id)
    if len(selected) != 1:
        raise ValueError(f"source-table fixed-wing provider {family_id!r} resolved {len(selected)} family records")
    return CatalogMissionCompositionProvider(
        ResolvedVehicleCompositionCatalog(selected),
        provider_id=provider_id,
        provider_name=provider_name,
        provider_short_name=provider_short_name,
        provider_summary=provider_summary,
        provider_version=_PACKAGE_VERSION,
        plugin_catalog=current_plugin_catalog(),
        include_builtin_workflows=False,
    )
    ####


def _x8_provider() -> CatalogMissionCompositionProvider:
    return _focused_provider(_X8_FAMILY_ID)
    ####


def _b747_provider() -> CatalogMissionCompositionProvider:
    return _focused_provider(_B747_FAMILY_ID)
    ####


def _register(registrar: PluginRegistrar) -> None:
    """Publish X8/B747 ownership without constructing numerical source plants."""

    registrar.register_vehicle_catalog_fragment(_CATALOG_FRAGMENT)
    for family_id in (_X8_FAMILY_ID, _B747_FAMILY_ID):
        registration = registrar.register_family_adapter_factory(
            family_id,
            _family_adapter_registration_factory(family_id),
        )
        registrar.register_model(family_id, registration)
    registrar.register_mission_composition_provider_factory(_X8_PROVIDER_ID, _x8_provider)
    registrar.register_mission_composition_provider_factory(_B747_PROVIDER_ID, _b747_provider)
    for adapter in (
        _LazyMissionCapabilityAdapter("taoryx.x8_racetrack.source_route.v1", _x8_racetrack_capability_adapter),
        _LazyMissionCapabilityAdapter("taoryx.b747_racetrack.source_route.v1", _b747_racetrack_capability_adapter),
        _LazyMissionCapabilityAdapter(
            "taoryx.b747_condition3_local_physical_surface_lqr_screen.capability.v1",
            _b747_lqr_capability_adapter,
        ),
        _LazyMissionCapabilityAdapter(
            "taoryx.b747_condition3_local_physical_surface_lqi_screen.capability.v1",
            _b747_lqi_capability_adapter,
        ),
        _LazyMissionCapabilityAdapter(
            "taoryx.x8_local_physical_surface_lqr_screen.capability.v1",
            _x8_lqr_capability_adapter,
        ),
        _LazyMissionCapabilityAdapter(
            "taoryx.x8_local_physical_surface_lqi_screen.capability.v1",
            _x8_lqi_capability_adapter,
        ),
        _LazyMissionCapabilityAdapter(
            "taoryx.x8_local_physical_surface_lqi_long_recovery_screen.capability.v1",
            _x8_long_recovery_lqi_capability_adapter,
        ),
    ):
        registrar.register_mission_capability_adapter(adapter)
    for campaign in _controller_tuning_campaigns():
        registrar.register_controller_tuning_campaign(campaign)
    for translator_id, handler in (
        ("taoryx.x8_racetrack.source_route.v1", _preflight_powered_fixed_wing_racetrack),
        ("taoryx.b747_racetrack.source_route.v1", _preflight_powered_fixed_wing_racetrack),
        ("taoryx.b747_condition3_local_physical_surface_lqr_screen.capability.v1", _preflight_b747_local_screen),
        ("taoryx.b747_condition3_local_physical_surface_lqi_screen.capability.v1", _preflight_b747_local_screen),
        ("taoryx.x8_local_physical_surface_lqr_screen.capability.v1", _preflight_x8_local_screen),
        ("taoryx.x8_local_physical_surface_lqi_screen.capability.v1", _preflight_x8_local_screen),
        ("taoryx.x8_local_physical_surface_lqi_long_recovery_screen.capability.v1", _preflight_x8_local_screen),
    ):
        registrar.register_semantic_preflight_handler_callback(translator_id, handler)
    registrar.register_episode_factory("language_backed_interactive.v1", _open_language_backed_episode)
    registrar.register_batch_episode_parity_verifier(
        "taoryx.language_backed.action_trace_batch_episode_parity.v1",
        _verify_language_backed_batch_episode_parity,
    )
    registrar.register_execution_factory_request_v1("language_backed_powered_fixed_wing.v1", _language_backed_batch)
    registrar.register_execution_factory_request_v1(
        "b747_condition3_local_physical_surface_lqr_screen.v1",
        _b747_local_physical_surface_batch,
    )
    registrar.register_execution_factory_request_v1(
        "b747_condition3_local_physical_surface_lqi_screen.v1",
        _b747_local_physical_surface_batch,
    )
    registrar.register_execution_factory_request_v1(
        "x8_local_physical_surface_lqr_screen.v1",
        _x8_local_physical_surface_batch,
    )
    registrar.register_execution_factory_request_v1(
        "x8_local_physical_surface_lqi_screen.v1",
        _x8_local_physical_surface_batch,
    )
    registrar.register_execution_factory_request_v1(
        "x8_local_physical_surface_lqi_long_recovery_screen.v1",
        _x8_local_physical_surface_batch,
    )
    ####


PLUGIN = PluginDefinition(
    metadata=PluginMetadata(
        id="taoryx.source-table-fixed-wing",
        package="taoryx-source-table-fixed-wing",
        version=_PACKAGE_VERSION,
        api_version="1",
        description="Skywalker X8 and B747 source-table vehicles, controls, and focused Mission Composition providers.",
    ),
    register_callback=_register,
)

__all__ = ["PLUGIN"]
