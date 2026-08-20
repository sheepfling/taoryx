"""Taoryx Mission Composition bridge for the source-compatible CRUISE5 plug-in."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from taoryx.trajectory.configuration_contract import (
    ConfigurationGroupSchema,
    ConfigurationGroupValue,
    ConfigurationParameterSchema,
    ConfigurationParameterValue,
    PreparedTrajectoryConfiguration,
    TrajectoryConfigurationInstance,
    TrajectoryConfigurationSchema,
    TrajectoryEntityOutputMetadata,
    TrajectoryFidelityMetadata,
    TrajectoryMissionOperationMetadata,
    TrajectoryMissionTemplateMetadata,
    TrajectoryModelCapabilities,
    TrajectoryModelMetadata,
    TrajectoryModelPresentationMetadata,
    TrajectoryModelPropertyMetadata,
    TrajectoryOutputChannelMetadata,
    TrajectoryOutputSchema,
    TrajectoryRealizationMetadata,
    TrajectoryReferenceFrameMetadata,
    TrajectoryTelemetryGroupMetadata,
    validate_configuration_instance,
)
from taoryx.trajectory.execution_contract import (
    MissionCompositionDiagnostic,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
    MissionCompositionTrajectoryResult,
    TrajectoryChannelMetadata,
    TrajectoryEvent,
    TrajectoryObject,
    TrajectorySample,
    resolve_output_selection,
)

from .aim5_mission_composition import CADAC_PROVIDER_ID
from .configuration_defaults import materialize_configuration_defaults
from .control_metadata import blocked_control_advertisement, source_managed_control_advertisement
from .cruise5 import Cruise5RunResult, Cruise5Sample
from .cruise5_plugin import Cruise5PluginOverrides, Cruise5VehiclePlugin
from .output_metadata import cadac_output_quantity

CRUISE5_MODEL_ID = "cadac.cruise5.cruise_vehicle"
CRUISE5_MODEL_VERSION = "0.5.0"
CRUISE5_FIDELITY_ID = "pseudo_6dof"
CRUISE5_TRANSLATION_FIDELITY_ID = "point_mass_3dof"
CRUISE5_REALIZATION_ID = "cadac-source-compatibility"
CRUISE5_TRANSLATION_REALIZATION_ID = "cadac-source-phase.translation_only"
CRUISE5_MISSION_ID = "waypoint_line_guidance"
CRUISE5_EXECUTOR_ID = "cadac.cruise5.source_compatibility.batch"
CRUISE5_SOURCE_PHASE_ID = "source_model"
CRUISE5_TRANSLATION_PHASE_ID = "translation_only"
_GEODETIC_FRAME_ID = "cadac.round3.geodetic"
_GEOGRAPHIC_FRAME_ID = "cadac.round3.geographic"


class CadacCruise5MissionCompositionProvider:
    """Self-describing batch provider for one installed CRUISE5 source case."""

    def __init__(self, plugin: Cruise5VehiclePlugin) -> None:
        blockers = plugin.validate_installation()
        if blockers:
            raise ValueError("cannot publish CRUISE5 provider with an incomplete installation: " + "; ".join(blockers))
        ####
        self._plugin = plugin
        self._schema = _build_configuration_schema(plugin)
        self._output_schema = _build_output_schema()
        self._model = _build_model_metadata(self._schema, self._output_schema)

    ####

    def list_models(self) -> tuple[TrajectoryModelMetadata, ...]:
        return (self._model,)

    ####

    def get_model_schema(self, model_id: str) -> TrajectoryConfigurationSchema:
        if model_id != CRUISE5_MODEL_ID:
            raise KeyError(f"unknown CRUISE5 model {model_id!r}")
        ####
        return self._schema

    ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        if model_id != CRUISE5_MODEL_ID:
            raise KeyError(f"unknown CRUISE5 model {model_id!r}")
        ####
        return self._output_schema

    ####

    def validate_configuration(self, configuration: TrajectoryConfigurationInstance) -> PreparedTrajectoryConfiguration:
        configuration = materialize_configuration_defaults(self._schema, configuration)
        prepared = validate_configuration_instance(self._schema, configuration)
        resolved = prepared.resolved
        if not isinstance(resolved, Mapping):
            raise ValueError("CRUISE5 resolved configuration root must be a mapping")
        ####
        phase = resolved.get("source_phase")
        if phase == CRUISE5_SOURCE_PHASE_ID:
            if configuration.fidelity != CRUISE5_FIDELITY_ID:
                raise ValueError(f"CRUISE5 source-model phase requires fidelity {CRUISE5_FIDELITY_ID!r}")
            ####
            if configuration.realization_id not in {None, CRUISE5_REALIZATION_ID}:
                raise ValueError(f"CRUISE5 source-model phase supports only realization {CRUISE5_REALIZATION_ID!r}")
            ####
            if configuration.mission_template_id not in {None, CRUISE5_MISSION_ID}:
                raise ValueError(f"CRUISE5 source-model phase supports only mission template {CRUISE5_MISSION_ID!r}")
            ####
        elif phase == CRUISE5_TRANSLATION_PHASE_ID:
            if configuration.fidelity != CRUISE5_TRANSLATION_FIDELITY_ID:
                raise ValueError(f"CRUISE5 translation-only phase requires fidelity {CRUISE5_TRANSLATION_FIDELITY_ID!r}")
            ####
            if configuration.realization_id not in {None, CRUISE5_TRANSLATION_REALIZATION_ID}:
                raise ValueError(f"CRUISE5 translation-only phase requires realization {CRUISE5_TRANSLATION_REALIZATION_ID!r}")
            ####
            if configuration.mission_template_id is not None:
                raise ValueError("CRUISE5 translation-only validation does not advertise the waypoint mission template")
            ####
        else:
            raise ValueError(f"unsupported CRUISE5 source phase {phase!r}")
        ####
        return prepared

    ####

    def execute_batch(self, request: MissionCompositionRunRequest) -> MissionCompositionTrajectoryResult:
        if request.provider_id != CADAC_PROVIDER_ID or request.model_id != CRUISE5_MODEL_ID:
            raise ValueError("CRUISE5 executor received a request for another provider/model")
        ####
        prepared = self.validate_configuration(request.prepared_configuration.configuration)
        if prepared.fingerprint != request.prepared_configuration.fingerprint:
            raise ValueError("CRUISE5 prepared configuration does not match provider validation")
        ####
        if prepared.configuration.fidelity != CRUISE5_FIDELITY_ID:
            raise ValueError("CRUISE5 translation-only phase is validation-only until an independent T1 lowering is installed")
        ####
        run = self._plugin.run_batch(_overrides_from_resolved(prepared.resolved))
        return _project_run_result(request, run, self._output_schema)

    ####


####


def build_default_cruise5_configuration(
    provider: CadacCruise5MissionCompositionProvider,
    *,
    configuration_id: str = "cruise5-default",
    phase_id: str = CRUISE5_SOURCE_PHASE_ID,
    overrides: Mapping[str, object] | None = None,
) -> TrajectoryConfigurationInstance:
    """Create a source-model or validate-only translation configuration from source defaults."""

    if phase_id not in {CRUISE5_SOURCE_PHASE_ID, CRUISE5_TRANSLATION_PHASE_ID}:
        raise ValueError(f"unknown CRUISE5 phase {phase_id!r}")
    ####
    supplied = dict(overrides or {})
    routes = {
        "longitude_deg": ("initialization", "longitude_deg"),
        "latitude_deg": ("initialization", "latitude_deg"),
        "altitude_m": ("initialization", "altitude_m"),
        "speed_mps": ("initialization", "speed_mps"),
        "heading_deg": ("initialization", "heading_deg"),
        "flight_path_deg": ("initialization", "flight_path_deg"),
        "alpha_deg": ("initialization", "alpha_deg"),
        "bank_deg": ("initialization", "bank_deg"),
        "end_time_s": ("runtime", "end_time_s"),
        "sample_step_s": ("runtime", "sample_step_s"),
    }
    unknown = sorted(set(supplied) - set(routes))
    if unknown:
        raise ValueError(f"unknown CRUISE5 configuration overrides: {unknown!r}")
    ####
    grouped: dict[str, dict[str, object]] = {}
    for source_name, value in supplied.items():
        group_id, parameter_id = routes[source_name]
        grouped.setdefault(group_id, {})[parameter_id] = value
    ####
    values: dict[str, Any] = {
        "source_phase": ConfigurationParameterValue(value=phase_id),
    }
    values.update(
        {
            group_id: ConfigurationGroupValue(
                values={parameter_id: ConfigurationParameterValue(value=value, unit=_configuration_unit(parameter_id)) for parameter_id, value in items.items()}
            )
            for group_id, items in grouped.items()
        }
    )
    schema = provider.get_model_schema(CRUISE5_MODEL_ID)
    source_model = phase_id == CRUISE5_SOURCE_PHASE_ID
    return TrajectoryConfigurationInstance(
        configuration_id=configuration_id,
        model_id=CRUISE5_MODEL_ID,
        model_version=CRUISE5_MODEL_VERSION,
        schema_fingerprint=schema.fingerprint,
        fidelity=CRUISE5_FIDELITY_ID if source_model else CRUISE5_TRANSLATION_FIDELITY_ID,
        realization_id=CRUISE5_REALIZATION_ID if source_model else CRUISE5_TRANSLATION_REALIZATION_ID,
        mission_template_id=CRUISE5_MISSION_ID if source_model else None,
        root=ConfigurationGroupValue(values=values),
    )


####


def register_cruise5_mission_composition(
    provider: CadacCruise5MissionCompositionProvider,
    registry: MissionCompositionRunnerRegistry,
) -> None:
    """Register the exact provider/model CRUISE5 source-compatible batch executor."""

    registry.register(CADAC_PROVIDER_ID, CRUISE5_MODEL_ID, provider.execute_batch)


####


def _build_configuration_schema(plugin: Cruise5VehiclePlugin) -> TrajectoryConfigurationSchema:
    source = plugin.source_definition()
    initial = source.initial_state
    compatible = (CRUISE5_FIDELITY_ID, CRUISE5_TRANSLATION_FIDELITY_ID)
    return TrajectoryConfigurationSchema(
        model_id=CRUISE5_MODEL_ID,
        model_version=CRUISE5_MODEL_VERSION,
        supported_fidelities=compatible,
        root=ConfigurationGroupSchema(
            id="cruise5",
            label="CRUISE5 source mission",
            description="Source CRUISE3 waypoint/line mission with an explicit fidelity phase selector.",
            children=(
                ConfigurationParameterSchema(
                    id="source_phase",
                    label="Source phase",
                    description="Select source-faithful pseudo-6DoF or the validate-only translation projection.",
                    value_type="enum",
                    required=False,
                    default=CRUISE5_SOURCE_PHASE_ID,
                    default_declared=True,
                    choices=(CRUISE5_SOURCE_PHASE_ID, CRUISE5_TRANSLATION_PHASE_ID),
                    role="variant",
                    compatible_fidelities=compatible,
                    provenance="missiondesignsolutions/CADAC/CRUISE5",
                ),
                ConfigurationGroupSchema(
                    id="initialization",
                    label="Initial truth",
                    children=(
                        _parameter(
                            "longitude_deg", "Longitude", "Initial east-positive longitude.", default=initial.longitude_deg, unit="deg", compatible=compatible
                        ),
                        _parameter("latitude_deg", "Latitude", "Initial geocentric latitude.", default=initial.latitude_deg, unit="deg", compatible=compatible),
                        _parameter("altitude_m", "Altitude", "Initial spherical-Earth altitude.", default=initial.altitude_m, unit="m", compatible=compatible),
                        _parameter("speed_mps", "Speed", "Initial Earth-relative speed.", default=initial.speed_mps, unit="m/s", compatible=compatible),
                        _parameter("heading_deg", "Heading", "Initial geographic heading.", default=initial.heading_deg, unit="deg", compatible=compatible),
                        _parameter(
                            "flight_path_deg",
                            "Flight path",
                            "Initial geographic flight-path angle.",
                            default=initial.flight_path_deg,
                            unit="deg",
                            compatible=compatible,
                        ),
                        _parameter(
                            "alpha_deg",
                            "Angle of attack",
                            "Initial reduced-order angle of attack.",
                            default=source.initial_alpha_deg,
                            unit="deg",
                            compatible=compatible,
                        ),
                        _parameter("bank_deg", "Bank", "Initial reduced-order bank angle.", default=source.initial_bank_deg, unit="deg", compatible=compatible),
                    ),
                ),
                ConfigurationGroupSchema(
                    id="runtime",
                    label="Runtime",
                    children=(
                        _parameter(
                            "end_time_s",
                            "End time",
                            "Requested source-compatible propagation horizon.",
                            default=source.end_time_s,
                            unit="s",
                            role="constraint",
                            compatible=compatible,
                        ),
                        _parameter(
                            "sample_step_s",
                            "Sample cadence",
                            "Returned trajectory cadence.",
                            default=source.plot_step_s or source.integration_step_s,
                            unit="s",
                            role="constraint",
                            compatible=compatible,
                        ),
                    ),
                ),
            ),
        ),
        claim_boundary=(
            "The installed executable phase preserves the source waypoint/line mission and response laws. "
            "The translation-only phase is classified and validate-only until an independent point-mass execution lowering is installed."
        ),
    )


####


def _parameter(
    parameter_id: str,
    label: str,
    description: str,
    *,
    default: object,
    unit: str | None = None,
    role: str = "initialization",
    compatible: tuple[str, ...],
) -> ConfigurationParameterSchema:
    return ConfigurationParameterSchema(
        id=parameter_id,
        label=label,
        description=description,
        canonical_unit=unit,
        display_unit=unit,
        required=False,
        default=default,
        default_declared=True,
        role=role,
        compatible_fidelities=compatible,
        provenance="missiondesignsolutions/CADAC/CRUISE5/input.asc",
    )


####


def _output(
    channel_id: str,
    label: str,
    description: str,
    *,
    unit: str | None = None,
    shape: tuple[int, ...] = (),
    frame: str | None = None,
) -> TrajectoryOutputChannelMetadata:
    return TrajectoryOutputChannelMetadata(
        id=channel_id,
        label=label,
        description=description,
        quantity=cadac_output_quantity(channel_id, unit),
        canonical_unit=unit,
        display_unit=unit,
        shape=shape,
        frame=frame,
        availability="guaranteed",
        compatible_fidelities=(CRUISE5_FIDELITY_ID,),
        compatible_realizations=(CRUISE5_REALIZATION_ID,),
        compatible_mission_templates=(CRUISE5_MISSION_ID,),
        operations=("batch",),
        provenance="CADAC CRUISE5 source-grounded waypoint/line reconstruction",
        claim_boundary="Available from the Python source-compatibility runtime with selected shipped-plot regression checkpoints.",
    )


####


def _build_output_schema() -> TrajectoryOutputSchema:
    core = (
        _output("longitude_deg", "Longitude", "East-positive source longitude.", unit="deg", frame=_GEODETIC_FRAME_ID),
        _output("latitude_deg", "Latitude", "Source latitude.", unit="deg", frame=_GEODETIC_FRAME_ID),
        _output("altitude_m", "Altitude", "Spherical-Earth source altitude.", unit="m", frame=_GEODETIC_FRAME_ID),
        _output("velocity_geographic_mps", "Geographic velocity", "North/east/down geographic velocity.", unit="m/s", shape=(3,), frame=_GEOGRAPHIC_FRAME_ID),
    )
    flight = (
        _output("speed_mps", "Speed", "Earth-relative speed.", unit="m/s"),
        _output("heading_deg", "Heading", "Geographic heading angle.", unit="deg"),
        _output("flight_path_deg", "Flight path", "Geographic flight-path angle.", unit="deg"),
        _output("mach", "Mach", "Source atmosphere-relative Mach number."),
        _output("dynamic_pressure_pa", "Dynamic pressure", "Source dynamic pressure.", unit="Pa"),
        _output("specific_force_velocity_mps2", "Specific force", "Non-gravitational specific force in velocity axes.", unit="m/s^2", shape=(3,)),
    )
    propulsion = (
        _output("propulsion_mode", "Propulsion mode", "Source propulsion mode/diagnostic branch."),
        _output("thrust_n", "Thrust", "Source turbojet thrust.", unit="N"),
        _output("mass_kg", "Mass", "Source current vehicle mass.", unit="kg"),
        _output("fuel_mass_kg", "Fuel mass", "Source remaining fuel mass.", unit="kg"),
        _output("center_of_gravity_in", "Center of gravity", "Source body-station center-of-gravity coordinate.", unit="in"),
    )
    response = (
        _output("lift_to_drag", "Lift-to-drag", "Source aerodynamic lift-to-drag ratio."),
        _output("alpha_deg", "Angle of attack", "Reduced-order source angle-of-attack response.", unit="deg"),
        _output("bank_deg", "Bank", "Reduced-order source bank response.", unit="deg"),
        _output("bank_command_deg", "Bank command", "Source bank command before response lag/limit.", unit="deg"),
        _output("load_command_g", "Load command", "Source normal load-factor command.", unit="g"),
        _output("lateral_command_g", "Lateral command", "Source lateral acceleration command.", unit="g"),
    )
    guidance = (
        _output("waypoint_ground_range_m", "Waypoint range", "Horizontal geographic range to the active source waypoint.", unit="m"),
        _output("waypoint_flag", "Waypoint flag", "Source closing/fleeting/outside waypoint flag."),
    )
    telemetry = (*flight, *propulsion, *response, *guidance)
    return TrajectoryOutputSchema(
        model_id=CRUISE5_MODEL_ID,
        model_version=CRUISE5_MODEL_VERSION,
        core_channels=core,
        telemetry_channels=telemetry,
        telemetry_groups=(
            TrajectoryTelemetryGroupMetadata(
                id="flight_condition",
                label="Flight condition",
                description="Kinematic and atmosphere diagnostics.",
                channel_ids=tuple(item.id for item in flight),
            ),
            TrajectoryTelemetryGroupMetadata(
                id="propulsion", label="Propulsion", description="Turbojet and resource diagnostics.", channel_ids=tuple(item.id for item in propulsion)
            ),
            TrajectoryTelemetryGroupMetadata(
                id="response_law",
                label="Response law",
                description="Reduced-order bank/alpha control response.",
                channel_ids=tuple(item.id for item in response),
            ),
            TrajectoryTelemetryGroupMetadata(
                id="guidance", label="Guidance", description="Source waypoint guidance diagnostics.", channel_ids=tuple(item.id for item in guidance)
            ),
        ),
        entity_output=TrajectoryEntityOutputMetadata(),
        claim_boundary="Round-Earth CRUISE5 source-model truth and telemetry for the executable pseudo-6DoF waypoint/line realization.",
    )


####


def _build_model_metadata(
    schema: TrajectoryConfigurationSchema,
    output_schema: TrajectoryOutputSchema,
) -> TrajectoryModelMetadata:
    source_fidelity = TrajectoryFidelityMetadata(
        id=CRUISE5_FIDELITY_ID,
        label="CADAC CRUISE5 pseudo-6DoF",
        rank=1,
        declared=True,
        dynamics_fidelity="pseudo_6dof",
        input_realization="guidance_command",
        runtime_fidelity="pseudo_6dof",
        control_realization="response_law",
        promotion_status="development",
        operations=("validate", "batch"),
        profile_id="cadac.cruise5.source-model",
        blockers=("full shipped-plot parity through waypoint event transitions remains a promotion gate",),
        required_operations=("round-Earth translation", "turbojet propulsion", "line guidance", "lagged bank/alpha response"),
        claim_boundary="Source translation, forces, guidance, propulsion, and reduced-order response participate; no rigid-body rotational moment closure is claimed.",
    )
    translation_fidelity = TrajectoryFidelityMetadata(
        id=CRUISE5_TRANSLATION_FIDELITY_ID,
        label="CADAC CRUISE5 translation projection",
        rank=0,
        declared=True,
        dynamics_fidelity="point_mass_3dof",
        input_realization="provider_defined",
        runtime_fidelity="point_mass_3dof",
        control_realization="force_model",
        promotion_status="planned",
        operations=("validate",),
        profile_id="cadac.cruise5.translation-only",
        blockers=("translation-only force-model execution has not been independently lowered from the source response-law mission",),
        required_operations=("independent point-mass force-model lowering",),
        claim_boundary="Classified lower tier only; validation does not execute the pseudo-6DoF response-law runtime and relabel it as 3-DoF.",
    )
    source_realization = TrajectoryRealizationMetadata(
        id=CRUISE5_REALIZATION_ID,
        label="CADAC source-compatible waypoint mission",
        description="Source-ordered round-Earth CRUISE3 mission with turbojet, line guidance, and response laws.",
        status="available",
        dynamics_fidelities=("pseudo_6dof",),
        input_realization="guidance_command",
        controls=source_managed_control_advertisement(
            mission_ids=(CRUISE5_MISSION_ID,),
            source_refs=("missiondesignsolutions/CADAC/CRUISE5",),
            claim_boundary="CRUISE5 source guidance and response-law control are internal to the batch runtime.",
        ),
        fidelity_aliases=(CRUISE5_FIDELITY_ID,),
        mission_template_ids=(CRUISE5_MISSION_ID,),
        operations=("validate", "batch"),
        native_factory_ids=(CRUISE5_EXECUTOR_ID,),
        source_refs=("missiondesignsolutions/CADAC/CRUISE5",),
        blockers=("point/arc/pro-nav source modes are not yet included in this executable realization",),
        claim_boundary="Executable for supported waypoint/line source modes with source event sequencing and stored-derivative integration.",
    )
    translation_realization = TrajectoryRealizationMetadata(
        id=CRUISE5_TRANSLATION_REALIZATION_ID,
        label="CADAC translation-only projection",
        description="Planned point-mass projection of the CRUISE5 force model without reduced-order bank/alpha response.",
        status="blocked",
        dynamics_fidelities=("point_mass_3dof",),
        input_realization="provider_defined",
        controls=blocked_control_advertisement(
            claim_boundary="The translation-only projection has no installed CADAC control boundary.",
        ),
        fidelity_aliases=(CRUISE5_TRANSLATION_FIDELITY_ID,),
        operations=("validate",),
        blockers=("standalone translation-only execution binding is not installed",),
        source_refs=("missiondesignsolutions/CADAC/CRUISE5",),
        claim_boundary="Discovery/validation only; no automatic lowering from pseudo-6DoF is performed.",
    )
    mission = TrajectoryMissionTemplateMetadata(
        id=CRUISE5_MISSION_ID,
        name="CRUISE5 waypoint/line guidance",
        description="Execute the installed source waypoint mission with source events, line guidance, turbojet, and pseudo-6DoF response laws.",
        status="development",
        initialization_variants=("source_case_override",),
        segment_sequence=("source_waypoint_line_mission",),
        compatible_fidelities=(CRUISE5_FIDELITY_ID,),
        operations=(
            TrajectoryMissionOperationMetadata(
                fidelity=CRUISE5_FIDELITY_ID,
                realization_id=CRUISE5_REALIZATION_ID,
                operation="validate",
                status="available",
                execution_mode="source_grounded_validation",
                common_runner_status="not_available",
                claim_boundary="Validation preserves source mission/event semantics and supported mode boundaries.",
            ),
            TrajectoryMissionOperationMetadata(
                fidelity=CRUISE5_FIDELITY_ID,
                realization_id=CRUISE5_REALIZATION_ID,
                operation="batch",
                status="available",
                execution_mode="source_compatibility_runtime",
                availability_scope="native_runtime_binding",
                common_runner_status="registered",
                executor_id=CRUISE5_EXECUTOR_ID,
                claim_boundary="Batch dispatch reaches the exact installed CRUISE5 source-compatible plug-in.",
            ),
        ),
        provenance="CADAC CRUISE5 environment, aerodynamics, propulsion, forces, Round3 Newton, line guidance, control, and events",
        claim_boundary="Waypoint/line source realization only; point/arc/pro-nav expansion is separate work.",
    )
    return TrajectoryModelMetadata(
        id=CRUISE5_MODEL_ID,
        name="CADAC CRUISE5 cruise vehicle",
        version=CRUISE5_MODEL_VERSION,
        description="CADAC CRUISE3/CRUISE5 round-Earth cruise vehicle exposed as a pseudo-6DoF Taoryx plug-in.",
        presentation=TrajectoryModelPresentationMetadata(
            display_name="CADAC CRUISE5",
            short_name="CRUISE5",
            summary="Round/rotating-Earth cruise missile with source turbojet, waypoint line guidance, and lagged bank/alpha response.",
            category="missile",
            subcategory="cruise",
            sort_key="cadac-cruise5",
            badges=("CADAC", "pseudo-6DoF", "round-Earth", "source-golden-checkpoints"),
            default_fidelity_id=CRUISE5_FIDELITY_ID,
            default_mission_template_id=CRUISE5_MISSION_ID,
            default_output_channel_ids=("longitude_deg", "latitude_deg", "altitude_m", "velocity_geographic_mps"),
            properties=(
                TrajectoryModelPropertyMetadata(
                    id="source_model",
                    label="Source model",
                    description="Upstream CADAC actor identifier.",
                    semantic_role="identity",
                    value_type="string",
                    value_kind="declared",
                    value="CRUISE3",
                    value_declared=True,
                    source_refs=("missiondesignsolutions/CADAC/CRUISE5",),
                    provenance="CADAC CRUISE5 source package",
                    claim_boundary="Identity metadata only.",
                ),
            ),
        ),
        family_id=CRUISE5_MODEL_ID,
        physical_family="cruise_missile",
        model_kind="vehicle_plugin",
        status="development",
        tags=("cadac", "cruise_missile", "pseudo_6dof", "round_earth"),
        execution_capability_profile="taoryx_universal",
        operations=("discover", "validate", "batch"),
        common_runner_operations=("batch",),
        capabilities=TrajectoryModelCapabilities(
            initialization_modes=("source_case_override",),
            segment_types=("source_waypoint_line_mission",),
            termination_modes=("end_time", "terminal_waypoint_altitude", "nonfinite_state"),
            operations=("discover", "validate", "batch"),
            supports_custom_segments=False,
            supports_deployment=False,
            supports_staging=False,
            supports_dynamic_child_generation=False,
            supports_multiple_stages=False,
            supports_submodels=False,
        ),
        realizations=(source_realization, translation_realization),
        mission_templates=(mission,),
        deployments=(),
        reference_frames=(
            TrajectoryReferenceFrameMetadata(
                id=_GEODETIC_FRAME_ID,
                name="CADAC Round3 spherical geodetic coordinates",
                description="Source spherical-Earth longitude, latitude, and altitude coordinates.",
                frame_kind="geodetic",
                axes=("longitude", "latitude", "altitude"),
                handedness="not_applicable",
                origin="Earth center / mean spherical Earth",
                orientation="east-positive longitude, source geocentric latitude, outward altitude",
                source_refs=("missiondesignsolutions/CADAC/CRUISE5/round3_modules.cpp",),
                provenance="CADAC Round3 spherical-Earth convention",
            ),
            TrajectoryReferenceFrameMetadata(
                id=_GEOGRAPHIC_FRAME_ID,
                name="CADAC Round3 geographic axes",
                description="Local north/east/down geographic frame used by the source Round3 equations.",
                frame_kind="local_tangent",
                axes=("north", "east", "down"),
                handedness="right",
                origin="instantaneous vehicle geographic location",
                orientation="north-east-down",
                source_refs=("missiondesignsolutions/CADAC/CRUISE5/round3_modules.cpp",),
                provenance="CADAC Round3 geographic convention",
            ),
        ),
        output_schema=output_schema,
        output_schema_id=output_schema.schema_id,
        output_schema_fingerprint=output_schema.fingerprint,
        configuration_schema_id=schema.schema_id,
        configuration_schema_fingerprint=schema.fingerprint,
        fidelities=(source_fidelity, translation_fidelity),
        fidelity_transitions=(),
        source_refs=("missiondesignsolutions/CADAC/CRUISE5",),
        provenance="CADAC CRUISE5 source-grounded Python waypoint/line reconstruction",
        claim_boundary="Executable pseudo-6DoF waypoint/line realization plus an explicitly blocked translation-only lower tier; no rigid-body claim.",
    )


####


def _overrides_from_resolved(resolved: Any) -> Cruise5PluginOverrides:
    if not isinstance(resolved, Mapping):
        raise ValueError("CRUISE5 resolved configuration root must be a mapping")
    ####
    initial = _group(resolved, "initialization")
    runtime = _group(resolved, "runtime")
    return Cruise5PluginOverrides(
        longitude_deg=float(initial["longitude_deg"]),
        latitude_deg=float(initial["latitude_deg"]),
        altitude_m=float(initial["altitude_m"]),
        speed_mps=float(initial["speed_mps"]),
        heading_deg=float(initial["heading_deg"]),
        flight_path_deg=float(initial["flight_path_deg"]),
        alpha_deg=float(initial["alpha_deg"]),
        bank_deg=float(initial["bank_deg"]),
        end_time_s=float(runtime["end_time_s"]),
        sample_step_s=float(runtime["sample_step_s"]),
    )


####


def _group(resolved: Mapping[str, Any], group_id: str) -> Mapping[str, Any]:
    group = resolved.get(group_id)
    if not isinstance(group, Mapping):
        raise ValueError(f"CRUISE5 resolved configuration is missing group {group_id!r}")
    ####
    return group


####


def _configuration_unit(parameter_id: str) -> str | None:
    if parameter_id in {"longitude_deg", "latitude_deg", "heading_deg", "flight_path_deg", "alpha_deg", "bank_deg"}:
        return "deg"
    ####
    if parameter_id == "altitude_m":
        return "m"
    ####
    if parameter_id == "speed_mps":
        return "m/s"
    ####
    if parameter_id in {"end_time_s", "sample_step_s"}:
        return "s"
    ####
    return None


####


def _project_run_result(
    request: MissionCompositionRunRequest,
    run: Cruise5RunResult,
    output_schema: TrajectoryOutputSchema,
) -> MissionCompositionTrajectoryResult:
    selected = resolve_output_selection(
        output_schema,
        request.output,
        fidelity=CRUISE5_FIDELITY_ID,
        operation="batch",
        realization_id=CRUISE5_REALIZATION_ID,
        mission_template_id=CRUISE5_MISSION_ID,
    )
    channels = tuple(_runtime_channel(item, output_schema) for item in selected)
    samples = tuple(
        TrajectorySample(
            time_s=sample.time_s,
            values={channel.id: _sample_value(channel.id, sample) for channel in selected},
        )
        for sample in run.samples
    )
    failed = run.terminated_reason == "nonfinite_state"
    object_result = TrajectoryObject(
        object_id="cruise1",
        model_id=CRUISE5_MODEL_ID,
        realization_id=CRUISE5_REALIZATION_ID,
        name="CADAC CRUISE5 cruise vehicle",
        role="vehicle",
        fidelity=CRUISE5_FIDELITY_ID,
        status="failed" if failed else "completed",
        active_from_s=samples[0].time_s,
        active_to_s=samples[-1].time_s,
        terminal_disposition=run.terminated_reason,
        channels=channels,
        samples=samples,
        provenance="CADAC CRUISE5 source-compatible round-Earth waypoint/line runner",
        claim_boundary="Pseudo-6DoF source response-law truth; rotational rigid-body moments are not part of this object.",
    )
    events = (
        tuple(
            TrajectoryEvent(
                id=f"source-event-{event.event_index + 1}",
                time_s=event.time_s,
                category="custom",
                kind="cadac-source-event",
                object_id="cruise1",
                detail=f"{event.watch_variable} source event from line {event.source_line}",
                data={"source_line": event.source_line, "criterion": event.criterion, "updates": dict(event.updates)},
            )
            for event in run.events
        )
        if request.output.include_events
        else ()
    )
    diagnostics = (
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-source-golden-checkpoints",
            message="CRUISE5 reconstruction has regression checkpoints against the repository-shipped plot1.asc at 0.0, 0.5, and 1.0 seconds for the default waypoint case.",
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=CRUISE5_MODEL_ID,
            object_id="cruise1",
        ),
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-pseudo-sixdof-realization",
            message="CRUISE5 bank and angle-of-attack are source response-law states coupled to translation; this result does not claim rigid-body 6-DoF moment closure.",
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=CRUISE5_MODEL_ID,
            object_id="cruise1",
        ),
    )
    return MissionCompositionTrajectoryResult(
        provider_id=CADAC_PROVIDER_ID,
        provider_version=CRUISE5_MODEL_VERSION,
        request_id=request.request_id,
        configuration_fingerprint=request.prepared_configuration.fingerprint,
        primary_model_id=CRUISE5_MODEL_ID,
        primary_object_id="cruise1",
        status="failed" if failed else "completed",
        objects=(object_result,),
        events=events,
        relationships=(),
        diagnostics=diagnostics,
        claim_boundary="One exact CRUISE5 source-model pseudo-6DoF vehicle returned through the common runner; the point-mass translation tier remains validate-only.",
    )


####


def _runtime_channel(
    channel: TrajectoryOutputChannelMetadata,
    output_schema: TrajectoryOutputSchema,
) -> TrajectoryChannelMetadata:
    group_by_channel = {channel_id: group.id for group in output_schema.telemetry_groups for channel_id in group.channel_ids}
    return TrajectoryChannelMetadata(
        id=channel.id,
        channel_class="core_state" if channel in output_schema.core_channels else "telemetry",
        telemetry_group=group_by_channel.get(channel.id),
        quantity=channel.quantity,
        unit=channel.canonical_unit,
        data_type=channel.data_type,
        shape=channel.shape,
        frame=channel.frame,
        sampling_semantics=channel.sampling_semantics,
        interpolation=channel.interpolation,
        description=channel.description,
    )


####


def _sample_value(channel_id: str, sample: Cruise5Sample) -> object:
    values: dict[str, object] = {
        "longitude_deg": sample.longitude_deg,
        "latitude_deg": sample.latitude_deg,
        "altitude_m": sample.altitude_m,
        "velocity_geographic_mps": sample.velocity_geographic_mps,
        "speed_mps": sample.speed_mps,
        "heading_deg": sample.heading_deg,
        "flight_path_deg": sample.flight_path_deg,
        "mach": sample.mach,
        "dynamic_pressure_pa": sample.dynamic_pressure_pa,
        "specific_force_velocity_mps2": sample.specific_force_velocity_mps2,
        "propulsion_mode": sample.propulsion_mode,
        "thrust_n": sample.thrust_n,
        "mass_kg": sample.mass_kg,
        "fuel_mass_kg": sample.fuel_mass_kg,
        "center_of_gravity_in": sample.center_of_gravity_in,
        "lift_to_drag": sample.lift_to_drag,
        "alpha_deg": sample.alpha_deg,
        "bank_deg": sample.bank_deg,
        "bank_command_deg": sample.bank_command_deg,
        "load_command_g": sample.load_command_g,
        "lateral_command_g": sample.lateral_command_g,
        "waypoint_ground_range_m": sample.waypoint_ground_range_m,
        "waypoint_flag": sample.waypoint_flag,
    }
    return values[channel_id]


####


__all__ = [
    "CRUISE5_EXECUTOR_ID",
    "CRUISE5_FIDELITY_ID",
    "CRUISE5_MISSION_ID",
    "CRUISE5_MODEL_ID",
    "CRUISE5_MODEL_VERSION",
    "CRUISE5_REALIZATION_ID",
    "CRUISE5_SOURCE_PHASE_ID",
    "CRUISE5_TRANSLATION_FIDELITY_ID",
    "CRUISE5_TRANSLATION_PHASE_ID",
    "CRUISE5_TRANSLATION_REALIZATION_ID",
    "CadacCruise5MissionCompositionProvider",
    "build_default_cruise5_configuration",
    "register_cruise5_mission_composition",
]
