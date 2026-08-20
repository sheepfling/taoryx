"""Taoryx Mission Composition bridge for the source-compatible GHAME3 plug-in."""

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
from .control_metadata import source_managed_control_advertisement
from .ghame3 import Ghame3RunResult, Ghame3Sample
from .ghame3_plugin import Ghame3PluginOverrides, Ghame3VehiclePlugin
from .output_metadata import cadac_output_quantity

GHAME3_MODEL_ID = "cadac.ghame3.hypersonic_vehicle"
GHAME3_MODEL_VERSION = "0.5.0"
GHAME3_FIDELITY_ID = "point_mass_3dof"
GHAME3_REALIZATION_ID = "cadac-source-compatibility"
GHAME3_MISSION_ID = "climb_cruise_schedule"
GHAME3_EXECUTOR_ID = "cadac.ghame3.source_compatibility.batch"
_GEODETIC_FRAME_ID = "cadac.round3.geodetic"
_GEOGRAPHIC_FRAME_ID = "cadac.round3.geographic"


class CadacGhame3MissionCompositionProvider:
    """Self-describing batch provider for one installed GHAME3 point-mass source case."""

    def __init__(self, plugin: Ghame3VehiclePlugin) -> None:
        blockers = plugin.validate_installation()
        if blockers:
            raise ValueError("cannot publish GHAME3 provider with an incomplete installation: " + "; ".join(blockers))
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
        if model_id != GHAME3_MODEL_ID:
            raise KeyError(f"unknown GHAME3 model {model_id!r}")
        ####
        return self._schema

    ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        if model_id != GHAME3_MODEL_ID:
            raise KeyError(f"unknown GHAME3 model {model_id!r}")
        ####
        return self._output_schema

    ####

    def validate_configuration(self, configuration: TrajectoryConfigurationInstance) -> PreparedTrajectoryConfiguration:
        if configuration.fidelity != GHAME3_FIDELITY_ID:
            raise ValueError(f"GHAME3 source model requires fidelity {GHAME3_FIDELITY_ID!r}")
        ####
        if configuration.realization_id not in {None, GHAME3_REALIZATION_ID}:
            raise ValueError(f"GHAME3 supports only realization {GHAME3_REALIZATION_ID!r}")
        ####
        if configuration.mission_template_id not in {None, GHAME3_MISSION_ID}:
            raise ValueError(f"GHAME3 supports only mission template {GHAME3_MISSION_ID!r}")
        ####
        configuration = materialize_configuration_defaults(self._schema, configuration)
        return validate_configuration_instance(self._schema, configuration)

    ####

    def execute_batch(self, request: MissionCompositionRunRequest) -> MissionCompositionTrajectoryResult:
        if request.provider_id != CADAC_PROVIDER_ID or request.model_id != GHAME3_MODEL_ID:
            raise ValueError("GHAME3 executor received a request for another provider/model")
        ####
        prepared = self.validate_configuration(request.prepared_configuration.configuration)
        if prepared.fingerprint != request.prepared_configuration.fingerprint:
            raise ValueError("GHAME3 prepared configuration does not match provider validation")
        ####
        run = self._plugin.run_batch(_overrides_from_resolved(prepared.resolved))
        return _project_run_result(request, run, self._output_schema)

    ####


####


def build_default_ghame3_configuration(
    provider: CadacGhame3MissionCompositionProvider,
    *,
    configuration_id: str = "ghame3-default",
    overrides: Mapping[str, object] | None = None,
) -> TrajectoryConfigurationInstance:
    """Create one source-schedule GHAME3 configuration from installed source defaults."""

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
        raise ValueError(f"unknown GHAME3 configuration overrides: {unknown!r}")
    ####
    grouped: dict[str, dict[str, object]] = {}
    for source_name, value in supplied.items():
        group_id, parameter_id = routes[source_name]
        grouped.setdefault(group_id, {})[parameter_id] = value
    ####
    root = ConfigurationGroupValue(
        values={
            group_id: ConfigurationGroupValue(
                values={
                    parameter_id: ConfigurationParameterValue(value=value, unit=_configuration_unit(parameter_id)) for parameter_id, value in values.items()
                }
            )
            for group_id, values in grouped.items()
        }
    )
    schema = provider.get_model_schema(GHAME3_MODEL_ID)
    return TrajectoryConfigurationInstance(
        configuration_id=configuration_id,
        model_id=GHAME3_MODEL_ID,
        model_version=GHAME3_MODEL_VERSION,
        schema_fingerprint=schema.fingerprint,
        fidelity=GHAME3_FIDELITY_ID,
        realization_id=GHAME3_REALIZATION_ID,
        mission_template_id=GHAME3_MISSION_ID,
        root=root,
    )


####


def register_ghame3_mission_composition(
    provider: CadacGhame3MissionCompositionProvider,
    registry: MissionCompositionRunnerRegistry,
) -> None:
    registry.register(CADAC_PROVIDER_ID, GHAME3_MODEL_ID, provider.execute_batch)


####


def _build_configuration_schema(plugin: Ghame3VehiclePlugin) -> TrajectoryConfigurationSchema:
    source = plugin.source_definition()
    initial = source.initial_state
    return TrajectoryConfigurationSchema(
        model_id=GHAME3_MODEL_ID,
        model_version=GHAME3_MODEL_VERSION,
        supported_fidelities=(GHAME3_FIDELITY_ID,),
        root=ConfigurationGroupSchema(
            id="ghame3",
            label="GHAME3 point-mass mission",
            description="Round-Earth 3-DoF source mission with prescribed alpha/bank and hypersonic propulsion events.",
            children=(
                ConfigurationGroupSchema(
                    id="initialization",
                    label="Initial truth",
                    children=(
                        _parameter("longitude_deg", "Longitude", "Initial east-positive longitude.", default=initial.longitude_deg, unit="deg"),
                        _parameter("latitude_deg", "Latitude", "Initial source latitude.", default=initial.latitude_deg, unit="deg"),
                        _parameter("altitude_m", "Altitude", "Initial spherical-Earth altitude.", default=initial.altitude_m, unit="m"),
                        _parameter("speed_mps", "Speed", "Initial Earth-relative speed.", default=initial.speed_mps, unit="m/s"),
                        _parameter("heading_deg", "Heading", "Initial geographic heading.", default=initial.heading_deg, unit="deg"),
                        _parameter("flight_path_deg", "Flight path", "Initial geographic flight-path angle.", default=initial.flight_path_deg, unit="deg"),
                        _parameter("alpha_deg", "Angle of attack", "Initial prescribed source angle of attack.", default=source.initial_alpha_deg, unit="deg"),
                        _parameter("bank_deg", "Bank", "Initial prescribed source bank angle.", default=source.initial_bank_deg, unit="deg"),
                    ),
                ),
                ConfigurationGroupSchema(
                    id="runtime",
                    label="Runtime",
                    children=(
                        _parameter(
                            "end_time_s", "End time", "Requested source-compatible propagation horizon.", default=source.end_time_s, unit="s", role="constraint"
                        ),
                        _parameter(
                            "sample_step_s",
                            "Sample cadence",
                            "Returned trajectory cadence.",
                            default=source.plot_step_s or source.integration_step_s,
                            unit="s",
                            role="constraint",
                        ),
                    ),
                ),
            ),
        ),
        claim_boundary="Configuration preserves the installed GHAME3 source event schedule and point-mass force model; no attitude-response state is introduced.",
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
        compatible_fidelities=(GHAME3_FIDELITY_ID,),
        provenance="missiondesignsolutions/CADAC/GHAME3/Inputs/input.asc",
    )


####


def _output(
    channel_id: str, label: str, description: str, *, unit: str | None = None, shape: tuple[int, ...] = (), frame: str | None = None
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
        compatible_fidelities=(GHAME3_FIDELITY_ID,),
        compatible_realizations=(GHAME3_REALIZATION_ID,),
        compatible_mission_templates=(GHAME3_MISSION_ID,),
        operations=("batch",),
        provenance="CADAC GHAME3 source-grounded point-mass reconstruction",
        claim_boundary="Available from the Python source-compatibility point-mass runtime; compiled golden parity remains a separate promotion gate.",
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
        _output("mach", "Mach", "Source Mach number."),
        _output("dynamic_pressure_pa", "Dynamic pressure", "Source dynamic pressure.", unit="Pa"),
        _output("specific_force_velocity_mps2", "Specific force", "Non-gravitational specific force in velocity axes.", unit="m/s^2", shape=(3,)),
    )
    aero = (
        _output("alpha_deg", "Angle of attack", "Prescribed source angle of attack.", unit="deg"),
        _output("bank_deg", "Bank", "Prescribed source bank angle.", unit="deg"),
        _output("lift_to_drag", "Lift-to-drag", "Source aerodynamic lift-to-drag ratio."),
    )
    propulsion = (
        _output("propulsion_mode", "Propulsion mode", "Source fixed/Q-hold propulsion mode."),
        _output("throttle", "Throttle", "Source hypersonic engine throttle."),
        _output("thrust_n", "Thrust", "Source hypersonic engine thrust.", unit="N"),
        _output("specific_impulse_s", "Specific impulse", "Source propulsion-deck specific impulse.", unit="s"),
        _output("capture_area_factor", "Capture area factor", "Source inlet capture-area factor."),
        _output("mass_kg", "Mass", "Current vehicle mass.", unit="kg"),
        _output("fuel_mass_kg", "Fuel mass", "Remaining source fuel mass.", unit="kg"),
    )
    telemetry = (*flight, *aero, *propulsion)
    return TrajectoryOutputSchema(
        model_id=GHAME3_MODEL_ID,
        model_version=GHAME3_MODEL_VERSION,
        core_channels=core,
        telemetry_channels=telemetry,
        telemetry_groups=(
            TrajectoryTelemetryGroupMetadata(
                id="flight_condition",
                label="Flight condition",
                description="Round3 kinematic and atmosphere diagnostics.",
                channel_ids=tuple(item.id for item in flight),
            ),
            TrajectoryTelemetryGroupMetadata(
                id="aerodynamics",
                label="Aerodynamics",
                description="Prescribed incidence/bank and drag-polar diagnostics.",
                channel_ids=tuple(item.id for item in aero),
            ),
            TrajectoryTelemetryGroupMetadata(
                id="propulsion", label="Propulsion", description="Hypersonic engine and fuel diagnostics.", channel_ids=tuple(item.id for item in propulsion)
            ),
        ),
        entity_output=TrajectoryEntityOutputMetadata(),
        claim_boundary="Round-Earth GHAME3 point-mass truth with aerodynamic and hypersonic propulsion telemetry.",
    )


####


def _build_model_metadata(schema: TrajectoryConfigurationSchema, output_schema: TrajectoryOutputSchema) -> TrajectoryModelMetadata:
    fidelity = TrajectoryFidelityMetadata(
        id=GHAME3_FIDELITY_ID,
        label="CADAC GHAME3 3-DoF",
        rank=0,
        declared=True,
        dynamics_fidelity="point_mass_3dof",
        input_realization="provider_defined",
        runtime_fidelity="point_mass_3dof",
        control_realization="force_model",
        promotion_status="development",
        operations=("validate", "batch"),
        profile_id="cadac.ghame3.source-model",
        blockers=("compiled/source golden trajectory parity is not yet registered",),
        required_operations=("Round3 translation", "hypersonic aerodynamic force model", "fixed/Q-hold propulsion"),
        claim_boundary="Translational state is integrated; source alpha/bank are prescribed data/event values rather than rotational response states.",
    )
    realization = TrajectoryRealizationMetadata(
        id=GHAME3_REALIZATION_ID,
        label="CADAC GHAME3 source-compatible 3-DoF",
        description="Source-ordered Round3 hypersonic climb/cruise mission with prescribed alpha/bank and propulsion events.",
        status="available",
        dynamics_fidelities=("point_mass_3dof",),
        input_realization="provider_defined",
        controls=source_managed_control_advertisement(
            mission_ids=(GHAME3_MISSION_ID,),
            source_refs=("missiondesignsolutions/CADAC/GHAME3",),
            claim_boundary="GHAME3 source event control is internal to the batch compatibility runtime.",
        ),
        fidelity_aliases=(GHAME3_FIDELITY_ID,),
        mission_template_ids=(GHAME3_MISSION_ID,),
        operations=("validate", "batch"),
        native_factory_ids=(GHAME3_EXECUTOR_ID,),
        source_refs=("missiondesignsolutions/CADAC/GHAME3",),
        blockers=(),
        claim_boundary="Executable source point-mass mission; no pseudo-6DoF promotion is inferred from prescribed alpha or bank.",
    )
    mission = TrajectoryMissionTemplateMetadata(
        id=GHAME3_MISSION_ID,
        name="GHAME3 climb/cruise event schedule",
        description="Execute the installed source event schedule over the Round3 point-mass hypersonic vehicle.",
        status="development",
        initialization_variants=("source_case_override",),
        segment_sequence=("source_event_schedule",),
        compatible_fidelities=(GHAME3_FIDELITY_ID,),
        operations=(
            TrajectoryMissionOperationMetadata(
                fidelity=GHAME3_FIDELITY_ID,
                realization_id=GHAME3_REALIZATION_ID,
                operation="validate",
                status="available",
                execution_mode="source_grounded_validation",
                common_runner_status="not_available",
                claim_boundary="Validation preserves supported source event semantics.",
            ),
            TrajectoryMissionOperationMetadata(
                fidelity=GHAME3_FIDELITY_ID,
                realization_id=GHAME3_REALIZATION_ID,
                operation="batch",
                status="available",
                execution_mode="source_compatibility_runtime",
                availability_scope="native_runtime_binding",
                common_runner_status="registered",
                executor_id=GHAME3_EXECUTOR_ID,
                claim_boundary="Batch dispatch reaches the exact installed GHAME3 point-mass plug-in.",
            ),
        ),
        provenance="CADAC GHAME3 Round3 environment/Newton, aerodynamic drag polar, hypersonic propulsion, forces, and source events",
        claim_boundary="Installed source climb/cruise event schedule only.",
    )
    return TrajectoryModelMetadata(
        id=GHAME3_MODEL_ID,
        name="CADAC GHAME3 hypersonic vehicle",
        version=GHAME3_MODEL_VERSION,
        description="CADAC GHAME3 round-Earth hypersonic 3-DoF vehicle exposed as a Taoryx plug-in.",
        presentation=TrajectoryModelPresentationMetadata(
            display_name="CADAC GHAME3",
            short_name="GHAME3",
            summary="Round/rotating-Earth hypersonic point-mass vehicle with prescribed incidence/bank and Q-hold propulsion.",
            category="hypersonic_vehicle",
            subcategory="cruise",
            sort_key="cadac-ghame3",
            badges=("CADAC", "3-DoF", "round-Earth", "hypersonic"),
            default_fidelity_id=GHAME3_FIDELITY_ID,
            default_mission_template_id=GHAME3_MISSION_ID,
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
                    source_refs=("missiondesignsolutions/CADAC/GHAME3",),
                    provenance="CADAC GHAME3 source package",
                    claim_boundary="Identity metadata only.",
                ),
            ),
        ),
        family_id=GHAME3_MODEL_ID,
        physical_family="hypersonic_cruise_vehicle",
        model_kind="vehicle_plugin",
        status="development",
        tags=("cadac", "hypersonic", "point_mass_3dof", "round_earth"),
        execution_capability_profile="taoryx_universal",
        operations=("discover", "validate", "batch"),
        common_runner_operations=("batch",),
        capabilities=TrajectoryModelCapabilities(
            initialization_modes=("source_case_override",),
            segment_types=("source_event_schedule",),
            termination_modes=("end_time", "nonfinite_state"),
            operations=("discover", "validate", "batch"),
            supports_custom_segments=False,
            supports_deployment=False,
            supports_staging=False,
            supports_dynamic_child_generation=False,
            supports_multiple_stages=False,
            supports_submodels=False,
        ),
        realizations=(realization,),
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
                orientation="east-positive longitude, source latitude, outward altitude",
                source_refs=("missiondesignsolutions/CADAC/GHAME3/round3_modules.cpp",),
                provenance="CADAC Round3 convention",
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
                source_refs=("missiondesignsolutions/CADAC/GHAME3/round3_modules.cpp",),
                provenance="CADAC Round3 convention",
            ),
        ),
        output_schema=output_schema,
        output_schema_id=output_schema.schema_id,
        output_schema_fingerprint=output_schema.fingerprint,
        configuration_schema_id=schema.schema_id,
        configuration_schema_fingerprint=schema.fingerprint,
        fidelities=(fidelity,),
        fidelity_transitions=(),
        source_refs=("missiondesignsolutions/CADAC/GHAME3",),
        provenance="CADAC GHAME3 source-grounded Python point-mass reconstruction",
        claim_boundary="Executable 3-DoF source mission; prescribed alpha/bank are not used to promote the model to pseudo-6DoF.",
    )


####


def _overrides_from_resolved(resolved: Any) -> Ghame3PluginOverrides:
    if not isinstance(resolved, Mapping):
        raise ValueError("GHAME3 resolved configuration root must be a mapping")
    ####
    initial = _group(resolved, "initialization")
    runtime = _group(resolved, "runtime")
    return Ghame3PluginOverrides(
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
        raise ValueError(f"GHAME3 resolved configuration is missing group {group_id!r}")
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
    request: MissionCompositionRunRequest, run: Ghame3RunResult, output_schema: TrajectoryOutputSchema
) -> MissionCompositionTrajectoryResult:
    selected = resolve_output_selection(
        output_schema,
        request.output,
        fidelity=GHAME3_FIDELITY_ID,
        operation="batch",
        realization_id=GHAME3_REALIZATION_ID,
        mission_template_id=GHAME3_MISSION_ID,
    )
    channels = tuple(_runtime_channel(item, output_schema) for item in selected)
    samples = tuple(
        TrajectorySample(time_s=sample.time_s, values={channel.id: _sample_value(channel.id, sample) for channel in selected}) for sample in run.samples
    )
    failed = run.terminated_reason == "nonfinite_state"
    object_result = TrajectoryObject(
        object_id="ghame1",
        model_id=GHAME3_MODEL_ID,
        realization_id=GHAME3_REALIZATION_ID,
        name="CADAC GHAME3 hypersonic vehicle",
        role="vehicle",
        fidelity=GHAME3_FIDELITY_ID,
        status="failed" if failed else "completed",
        active_from_s=samples[0].time_s,
        active_to_s=samples[-1].time_s,
        terminal_disposition=run.terminated_reason,
        channels=channels,
        samples=samples,
        provenance="CADAC GHAME3 source-compatible point-mass runner",
        claim_boundary="Round3 translational truth only; prescribed alpha/bank do not constitute attitude-state integration.",
    )
    events = (
        tuple(
            TrajectoryEvent(
                id=f"source-event-{event.event_index + 1}",
                time_s=event.time_s,
                category="custom",
                kind="cadac-source-event",
                object_id="ghame1",
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
            code="cadac-point-mass-classification",
            message="GHAME3 executes as source 3-DoF Round3 translation; alpha and bank are prescribed source data/event values, not rotational response states.",
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=GHAME3_MODEL_ID,
            object_id="ghame1",
        ),
    )
    return MissionCompositionTrajectoryResult(
        provider_id=CADAC_PROVIDER_ID,
        provider_version=GHAME3_MODEL_VERSION,
        request_id=request.request_id,
        configuration_fingerprint=request.prepared_configuration.fingerprint,
        primary_model_id=GHAME3_MODEL_ID,
        primary_object_id="ghame1",
        status="failed" if failed else "completed",
        objects=(object_result,),
        events=events,
        relationships=(),
        diagnostics=diagnostics,
        claim_boundary="One exact GHAME3 3-DoF source vehicle returned through the common runner without pseudo-6DoF promotion.",
    )


####


def _runtime_channel(channel: TrajectoryOutputChannelMetadata, output_schema: TrajectoryOutputSchema) -> TrajectoryChannelMetadata:
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


def _sample_value(channel_id: str, sample: Ghame3Sample) -> object:
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
        "alpha_deg": sample.alpha_deg,
        "bank_deg": sample.bank_deg,
        "lift_to_drag": sample.lift_to_drag,
        "propulsion_mode": sample.propulsion_mode,
        "throttle": sample.throttle,
        "thrust_n": sample.thrust_n,
        "specific_impulse_s": sample.specific_impulse_s,
        "capture_area_factor": sample.capture_area_factor,
        "mass_kg": sample.mass_kg,
        "fuel_mass_kg": sample.fuel_mass_kg,
    }
    return values[channel_id]


####


__all__ = [
    "GHAME3_EXECUTOR_ID",
    "GHAME3_FIDELITY_ID",
    "GHAME3_MISSION_ID",
    "GHAME3_MODEL_ID",
    "GHAME3_MODEL_VERSION",
    "GHAME3_REALIZATION_ID",
    "CadacGhame3MissionCompositionProvider",
    "build_default_ghame3_configuration",
    "register_ghame3_mission_composition",
]
