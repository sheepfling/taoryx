"""Taoryx Mission Composition bridge for the ADS6 ``AIRCRAFT3`` plug-in."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any

from taoryx.trajectory.configuration_contract import (
    ConfigurationGroupSchema,
    ConfigurationGroupValue,
    ConfigurationParameterSchema,
    ConfigurationParameterValue,
    ConfigurationValueType,
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

from .ads6_aircraft import Ads6AircraftRunResult, Ads6AircraftSample, Ads6AircraftThreatTrack
from .ads6_aircraft_plugin import Ads6AircraftPluginOverrides, Ads6AircraftVehiclePlugin
from .aim5_mission_composition import CADAC_PROVIDER_ID
from .configuration_defaults import materialize_configuration_defaults
from .control_metadata import source_managed_control_advertisement
from .output_metadata import cadac_output_quantity

ADS6_AIRCRAFT_MODEL_ID = "cadac.ads6.aircraft"
ADS6_AIRCRAFT_MODEL_VERSION = "0.10.0"
ADS6_AIRCRAFT_FIDELITY_ID = "point_mass_3dof"
ADS6_AIRCRAFT_REALIZATION_ID = "cadac-ads6-aircraft-force-model"
ADS6_AIRCRAFT_PHASE_ID = "source_model"
ADS6_AIRCRAFT_MISSION_ID = "ads6_aircraft_flight"
ADS6_AIRCRAFT_EXECUTOR_ID = "cadac.ads6.aircraft.source_compatibility.batch"
_LOCAL_FRAME_ID = "cadac.ads6.local_ned"


class CadacAds6AircraftMissionCompositionProvider:
    """Self-describing batch provider for one installed ADS6 AIRCRAFT3 source case."""

    def __init__(self, plugin: Ads6AircraftVehiclePlugin) -> None:
        blockers = plugin.validate_installation()
        if blockers:
            raise ValueError("cannot publish ADS6 AIRCRAFT3 provider with an incomplete installation: " + "; ".join(blockers))
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
        if model_id != ADS6_AIRCRAFT_MODEL_ID:
            raise KeyError(f"unknown ADS6 AIRCRAFT3 model {model_id!r}")
        ####
        return self._schema

    ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        if model_id != ADS6_AIRCRAFT_MODEL_ID:
            raise KeyError(f"unknown ADS6 AIRCRAFT3 model {model_id!r}")
        ####
        return self._output_schema

    ####

    def validate_configuration(
        self,
        configuration: TrajectoryConfigurationInstance,
    ) -> PreparedTrajectoryConfiguration:
        if configuration.fidelity != ADS6_AIRCRAFT_FIDELITY_ID:
            raise ValueError(f"ADS6 AIRCRAFT3 source model requires fidelity {ADS6_AIRCRAFT_FIDELITY_ID!r}")
        ####
        if configuration.realization_id not in {None, ADS6_AIRCRAFT_REALIZATION_ID}:
            raise ValueError(f"ADS6 AIRCRAFT3 supports only realization {ADS6_AIRCRAFT_REALIZATION_ID!r}")
        ####
        if configuration.mission_template_id not in {None, ADS6_AIRCRAFT_MISSION_ID}:
            raise ValueError(f"ADS6 AIRCRAFT3 supports only mission template {ADS6_AIRCRAFT_MISSION_ID!r}")
        ####
        configuration = materialize_configuration_defaults(self._schema, configuration)
        prepared = validate_configuration_instance(self._schema, configuration)
        resolved = prepared.resolved
        if not isinstance(resolved, Mapping) or resolved.get("source_phase") != ADS6_AIRCRAFT_PHASE_ID:
            raise ValueError(f"ADS6 AIRCRAFT3 requires source phase {ADS6_AIRCRAFT_PHASE_ID!r}")
        ####
        overrides = _overrides_from_resolved(resolved)
        if overrides.guidance_option == 2 and overrides.threat_track is None:
            raise ValueError("ADS6 AIRCRAFT3 escape mode requires threat_track_enabled and a nonzero threat velocity")
        ####
        return prepared

    ####

    def execute_batch(self, request: MissionCompositionRunRequest) -> MissionCompositionTrajectoryResult:
        if request.provider_id != CADAC_PROVIDER_ID or request.model_id != ADS6_AIRCRAFT_MODEL_ID:
            raise ValueError("ADS6 AIRCRAFT3 executor received a request for another provider/model")
        ####
        prepared = self.validate_configuration(request.prepared_configuration.configuration)
        if prepared.fingerprint != request.prepared_configuration.fingerprint:
            raise ValueError("ADS6 AIRCRAFT3 prepared configuration does not match provider validation")
        ####
        run = self._plugin.run_batch(_overrides_from_resolved(prepared.resolved))
        return _project_run_result(request, run, self._output_schema)

    ####


####


def build_default_ads6_aircraft_configuration(
    provider: CadacAds6AircraftMissionCompositionProvider,
    *,
    configuration_id: str = "ads6-aircraft-default",
    overrides: Mapping[str, object] | None = None,
) -> TrajectoryConfigurationInstance:
    """Create one exact source AIRCRAFT3 configuration from installed defaults."""

    supplied = dict(overrides or {})
    routes = {
        "north_m": ("initialization", "north_m"),
        "east_m": ("initialization", "east_m"),
        "down_m": ("initialization", "down_m"),
        "speed_mps": ("initialization", "speed_mps"),
        "heading_deg": ("initialization", "heading_deg"),
        "flight_path_deg": ("initialization", "flight_path_deg"),
        "guidance_option": ("maneuver", "guidance_option"),
        "guidance_gain": ("maneuver", "guidance_gain"),
        "turn_load_g": ("maneuver", "turn_load_g"),
        "maneuver_start_s": ("maneuver", "maneuver_start_s"),
        "maneuver_stop_s": ("maneuver", "maneuver_stop_s"),
        "bank_time_constant_s": ("response", "bank_time_constant_s"),
        "bank_limit_deg": ("response", "bank_limit_deg"),
        "load_factor_time_constant_s": ("response", "load_factor_time_constant_s"),
        "alpha_limit_deg": ("response", "alpha_limit_deg"),
        "lift_slope_per_deg": ("response", "lift_slope_per_deg"),
        "wing_loading_n_m2": ("response", "wing_loading_n_m2"),
        "longitudinal_acceleration_g": ("response", "longitudinal_acceleration_g"),
        "threat_track_enabled": ("threat", "threat_track_enabled"),
        "threat_north_m": ("threat", "threat_north_m"),
        "threat_east_m": ("threat", "threat_east_m"),
        "threat_down_m": ("threat", "threat_down_m"),
        "threat_velocity_north_mps": ("threat", "threat_velocity_north_mps"),
        "threat_velocity_east_mps": ("threat", "threat_velocity_east_mps"),
        "threat_velocity_down_mps": ("threat", "threat_velocity_down_mps"),
        "threat_reference_time_s": ("threat", "threat_reference_time_s"),
        "end_time_s": ("runtime", "end_time_s"),
        "sample_step_s": ("runtime", "sample_step_s"),
    }
    unknown = sorted(set(supplied) - set(routes))
    if unknown:
        raise ValueError(f"unknown ADS6 AIRCRAFT3 configuration overrides: {unknown!r}")
    ####
    grouped: dict[str, dict[str, object]] = {}
    for source_name, value in supplied.items():
        group_id, parameter_id = routes[source_name]
        grouped.setdefault(group_id, {})[parameter_id] = value
    ####
    values: dict[str, Any] = {"source_phase": ConfigurationParameterValue(value=ADS6_AIRCRAFT_PHASE_ID)}
    values.update(
        {
            group_id: ConfigurationGroupValue(
                values={
                    parameter_id: ConfigurationParameterValue(
                        value=value,
                        unit=_configuration_unit(parameter_id),
                    )
                    for parameter_id, value in items.items()
                }
            )
            for group_id, items in grouped.items()
        }
    )
    schema = provider.get_model_schema(ADS6_AIRCRAFT_MODEL_ID)
    return TrajectoryConfigurationInstance(
        configuration_id=configuration_id,
        model_id=ADS6_AIRCRAFT_MODEL_ID,
        model_version=ADS6_AIRCRAFT_MODEL_VERSION,
        schema_fingerprint=schema.fingerprint,
        fidelity=ADS6_AIRCRAFT_FIDELITY_ID,
        realization_id=ADS6_AIRCRAFT_REALIZATION_ID,
        mission_template_id=ADS6_AIRCRAFT_MISSION_ID,
        root=ConfigurationGroupValue(values=values),
    )


####


def register_ads6_aircraft_mission_composition(
    provider: CadacAds6AircraftMissionCompositionProvider,
    registry: MissionCompositionRunnerRegistry,
) -> None:
    registry.register(CADAC_PROVIDER_ID, ADS6_AIRCRAFT_MODEL_ID, provider.execute_batch)


####


def _build_configuration_schema(
    plugin: Ads6AircraftVehiclePlugin,
) -> TrajectoryConfigurationSchema:
    source = plugin.source_definition()
    initial = source.initial_state
    guidance = source.guidance
    control = source.control
    return TrajectoryConfigurationSchema(
        model_id=ADS6_AIRCRAFT_MODEL_ID,
        model_version=ADS6_AIRCRAFT_MODEL_VERSION,
        supported_fidelities=(ADS6_AIRCRAFT_FIDELITY_ID,),
        root=ConfigurationGroupSchema(
            id="ads6_aircraft",
            label="ADS6 AIRCRAFT3 point-mass target",
            description="Flat3 translation with source bank/load-factor response and force closure.",
            children=(
                _parameter(
                    "source_phase",
                    "Source phase",
                    "Exact installed source-model phase selector.",
                    default=ADS6_AIRCRAFT_PHASE_ID,
                    choices=(ADS6_AIRCRAFT_PHASE_ID,),
                    role="variant",
                    value_type="enum",
                ),
                ConfigurationGroupSchema(
                    id="initialization",
                    label="Initial truth",
                    children=(
                        _parameter("north_m", "North", "Initial local north position.", default=initial.position_ned_m[0], unit="m"),
                        _parameter("east_m", "East", "Initial local east position.", default=initial.position_ned_m[1], unit="m"),
                        _parameter("down_m", "Down", "Initial local down position.", default=initial.position_ned_m[2], unit="m"),
                        _parameter("speed_mps", "Speed", "Initial Earth-relative speed.", default=initial.speed_mps, unit="m/s"),
                        _parameter("heading_deg", "Heading", "Initial heading angle.", default=initial.heading_deg, unit="deg"),
                        _parameter("flight_path_deg", "Flight path", "Initial flight-path angle.", default=initial.flight_path_deg, unit="deg"),
                    ),
                ),
                ConfigurationGroupSchema(
                    id="maneuver",
                    label="Source maneuver",
                    children=(
                        _parameter(
                            "guidance_option",
                            "Guidance option",
                            "0 steady; 1 horizontal g-turn; 2 threat escape.",
                            default=guidance.option,
                            role="variant",
                            value_type="integer",
                        ),
                        _parameter("guidance_gain", "Escape gain", "Cross-product escape guidance gain.", default=guidance.guidance_gain),
                        _parameter("turn_load_g", "Turn load", "Horizontal turn command; positive is right.", default=guidance.turn_load_g, unit="g"),
                        _parameter("maneuver_start_s", "Maneuver start", "Strict source maneuver-window start.", default=guidance.maneuver_start_s, unit="s"),
                        _parameter("maneuver_stop_s", "Maneuver stop", "Strict source maneuver-window stop.", default=guidance.maneuver_stop_s, unit="s"),
                    ),
                ),
                ConfigurationGroupSchema(
                    id="response",
                    label="Bank and load response",
                    children=(
                        _parameter(
                            "bank_time_constant_s",
                            "Bank lag",
                            "Source first-order bank response time constant; zero is ideal.",
                            default=control.bank_time_constant_s,
                            unit="s",
                        ),
                        _parameter("bank_limit_deg", "Bank limit", "Achieved bank-angle limit.", default=control.bank_limit_deg, unit="deg"),
                        _parameter(
                            "load_factor_time_constant_s",
                            "Load lag",
                            "Source first-order normal-load response time constant; zero is ideal.",
                            default=control.load_factor_time_constant_s,
                            unit="s",
                        ),
                        _parameter("alpha_limit_deg", "Alpha limit", "Load-factor limiter angle-of-attack bound.", default=control.alpha_limit_deg, unit="deg"),
                        _parameter(
                            "lift_slope_per_deg",
                            "Lift slope",
                            "Aircraft lift slope used by the load limiter.",
                            default=control.lift_slope_per_deg,
                            unit="1/deg",
                        ),
                        _parameter(
                            "wing_loading_n_m2",
                            "Wing loading",
                            "Aircraft wing loading used by the load limiter.",
                            default=control.wing_loading_n_m2,
                            unit="N/m^2",
                        ),
                        _parameter(
                            "longitudinal_acceleration_g",
                            "Longitudinal acceleration",
                            "Source body-axis longitudinal specific force.",
                            default=control.longitudinal_acceleration_g,
                            unit="g",
                        ),
                    ),
                ),
                ConfigurationGroupSchema(
                    id="threat",
                    label="Standalone escape observation",
                    description="Constant-velocity external threat seam used only when guidance option 2 is selected.",
                    children=(
                        _parameter(
                            "threat_track_enabled",
                            "Threat enabled",
                            "Enable the standalone external threat track.",
                            default=False,
                            value_type="boolean",
                            role="variant",
                        ),
                        _parameter("threat_north_m", "Threat north", "Threat position at the reference epoch.", default=0.0, unit="m"),
                        _parameter("threat_east_m", "Threat east", "Threat position at the reference epoch.", default=0.0, unit="m"),
                        _parameter("threat_down_m", "Threat down", "Threat position at the reference epoch.", default=0.0, unit="m"),
                        _parameter("threat_velocity_north_mps", "Threat north velocity", "Constant threat north velocity.", default=0.0, unit="m/s"),
                        _parameter("threat_velocity_east_mps", "Threat east velocity", "Constant threat east velocity.", default=0.0, unit="m/s"),
                        _parameter("threat_velocity_down_mps", "Threat down velocity", "Constant threat down velocity.", default=0.0, unit="m/s"),
                        _parameter(
                            "threat_reference_time_s", "Threat reference time", "Epoch associated with the supplied threat position.", default=0.0, unit="s"
                        ),
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
                            default=source.trajectory_step_s or source.integration_step_s,
                            unit="s",
                            role="constraint",
                        ),
                    ),
                ),
            ),
        ),
        claim_boundary=source.claim_boundary,
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
    choices: tuple[str, ...] = (),
    value_type: ConfigurationValueType = "number",
) -> ConfigurationParameterSchema:
    return ConfigurationParameterSchema(
        id=parameter_id,
        label=label,
        description=description,
        value_type=value_type,
        canonical_unit=unit,
        display_unit=unit,
        required=False,
        default=default,
        default_declared=True,
        choices=choices,
        role=role,
        compatible_fidelities=(ADS6_AIRCRAFT_FIDELITY_ID,),
        provenance="missiondesignsolutions/CADAC/ADS6 AIRCRAFT3 source case",
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
    data_type: str = "float64",
    availability: str = "guaranteed",
) -> TrajectoryOutputChannelMetadata:
    return TrajectoryOutputChannelMetadata(
        id=channel_id,
        label=label,
        description=description,
        quantity=cadac_output_quantity(channel_id, unit, data_type),
        canonical_unit=unit,
        display_unit=unit,
        data_type=data_type,
        shape=shape,
        frame=frame,
        interpolation="step" if data_type in {"boolean", "string", "json"} else "linear",
        availability=availability,
        compatible_fidelities=(ADS6_AIRCRAFT_FIDELITY_ID,),
        compatible_realizations=(ADS6_AIRCRAFT_REALIZATION_ID,),
        compatible_mission_templates=(ADS6_AIRCRAFT_MISSION_ID,),
        operations=("batch",),
        provenance="CADAC ADS6 AIRCRAFT3 source-grounded point-mass reconstruction",
        claim_boundary="Available from the Python source-compatibility runtime; compiled golden parity remains a separate promotion gate.",
    )


####


def _build_output_schema() -> TrajectoryOutputSchema:
    core = (
        _output("position_ned_m", "Position", "Flat-Earth local north/east/down position.", unit="m", shape=(3,), frame=_LOCAL_FRAME_ID),
        _output("velocity_ned_mps", "Velocity", "Flat-Earth local north/east/down velocity.", unit="m/s", shape=(3,), frame=_LOCAL_FRAME_ID),
    )
    flight = (
        _output("altitude_m", "Altitude", "Source flat-Earth altitude.", unit="m"),
        _output("speed_mps", "Speed", "Earth-relative speed.", unit="m/s"),
        _output("heading_deg", "Heading", "Source heading angle.", unit="deg"),
        _output("flight_path_deg", "Flight path", "Source flight-path angle.", unit="deg"),
        _output("mode", "Guidance mode", "Steady, g-turn, or escape source behavior.", data_type="string"),
        _output("maneuver_active", "Maneuver active", "Whether the strict source maneuver window is active.", data_type="boolean"),
    )
    response = (
        _output(
            "commanded_acceleration_ned_mps2",
            "Commanded acceleration",
            "Guidance acceleration in local NED coordinates.",
            unit="m/s^2",
            shape=(3,),
            frame=_LOCAL_FRAME_ID,
        ),
        _output(
            "commanded_acceleration_velocity_mps2", "Velocity-frame command", "Guidance acceleration in source velocity coordinates.", unit="m/s^2", shape=(3,)
        ),
        _output("commanded_bank_deg", "Commanded bank", "Bank command derived from lateral/normal acceleration.", unit="deg"),
        _output("bank_state_deg", "Bank response state", "Unclamped source bank-response state.", unit="deg"),
        _output("bank_deg", "Achieved bank", "Bank angle after source output limiting.", unit="deg"),
        _output("bank_limited", "Bank limited", "Whether the bank output reached its source limit.", data_type="boolean"),
        _output("commanded_load_factor_g", "Commanded load", "Source total normal-load command.", unit="g"),
        _output("normal_load_factor_g", "Achieved load", "Source normal-load response state after limiting.", unit="g"),
        _output("load_factor_limit_g", "Load limit", "Dynamic-pressure/alpha-derived source load limit.", unit="g"),
        _output("load_factor_limited", "Load limited", "Whether the normal-load response reached its source limit.", data_type="boolean"),
        _output("longitudinal_acceleration_g", "Longitudinal acceleration", "Configured body-axis longitudinal acceleration.", unit="g"),
        _output("specific_force_body_mps2", "Specific force", "Non-gravitational specific force in aircraft axes.", unit="m/s^2", shape=(3,)),
    )
    environment = (
        _output("density_kg_m3", "Density", "NASA-Marshall US76 density.", unit="kg/m^3"),
        _output("pressure_pa", "Pressure", "NASA-Marshall US76 pressure.", unit="Pa"),
        _output("dynamic_pressure_pa", "Dynamic pressure", "Source dynamic pressure.", unit="Pa"),
        _output("mach", "Mach", "Source Mach number."),
        _output("gravity_mps2", "Gravity", "Inverse-square local gravitational acceleration.", unit="m/s^2"),
    )
    threat = (_output("threat_range_m", "Threat range", "Range to the external standalone threat track.", unit="m", availability="conditional"),)
    telemetry = (*flight, *response, *environment, *threat)
    return TrajectoryOutputSchema(
        model_id=ADS6_AIRCRAFT_MODEL_ID,
        model_version=ADS6_AIRCRAFT_MODEL_VERSION,
        core_channels=core,
        telemetry_channels=telemetry,
        telemetry_groups=(
            TrajectoryTelemetryGroupMetadata(
                id="flight_condition", label="Flight condition", description="Flat3 kinematics and source mode.", channel_ids=tuple(item.id for item in flight)
            ),
            TrajectoryTelemetryGroupMetadata(
                id="response",
                label="Bank/load response",
                description="Guidance commands, lagged bank/load states, limits, and force closure.",
                channel_ids=tuple(item.id for item in response),
            ),
            TrajectoryTelemetryGroupMetadata(
                id="environment", label="Environment", description="NASA-Marshall atmosphere diagnostics.", channel_ids=tuple(item.id for item in environment)
            ),
            TrajectoryTelemetryGroupMetadata(
                id="threat",
                label="Threat observation",
                description="Optional standalone escape-mode observation.",
                channel_ids=tuple(item.id for item in threat),
            ),
        ),
        entity_output=TrajectoryEntityOutputMetadata(),
        claim_boundary="ADS6 AIRCRAFT3 translational truth with bank/load control telemetry; no attitude quaternion or body-rate channels are emitted.",
    )


####


def _build_model_metadata(
    schema: TrajectoryConfigurationSchema,
    output_schema: TrajectoryOutputSchema,
) -> TrajectoryModelMetadata:
    fidelity = TrajectoryFidelityMetadata(
        id=ADS6_AIRCRAFT_FIDELITY_ID,
        label="CADAC ADS6 AIRCRAFT3 point mass",
        rank=0,
        declared=True,
        dynamics_fidelity="point_mass_3dof",
        input_realization="provider_defined",
        runtime_fidelity="point_mass_3dof",
        control_realization="force_model",
        promotion_status="development",
        operations=("validate", "batch"),
        profile_id="cadac.ads6.aircraft.source-model",
        blockers=("compiled/source golden trajectory parity is not yet registered",),
        required_operations=("Flat3 translation", "bank/load response", "specific-force closure"),
        claim_boundary="Only position and velocity are trajectory truth; bank and load factor are source force-response telemetry, not rigid-body attitude state.",
    )
    realization = TrajectoryRealizationMetadata(
        id=ADS6_AIRCRAFT_REALIZATION_ID,
        label="CADAC ADS6 AIRCRAFT3 force model",
        description="Source-ordered steady, horizontal-turn, and optional external-threat escape target model.",
        status="available",
        dynamics_fidelities=("point_mass_3dof",),
        input_realization="provider_defined",
        controls=source_managed_control_advertisement(
            mission_ids=(ADS6_AIRCRAFT_MISSION_ID,),
            source_refs=("missiondesignsolutions/CADAC/ADS6/aircraft_modules.cpp",),
            claim_boundary="Source-scheduled AIRCRAFT3 control is internal to the batch compatibility runtime.",
        ),
        fidelity_aliases=(ADS6_AIRCRAFT_FIDELITY_ID,),
        mission_template_ids=(ADS6_AIRCRAFT_MISSION_ID,),
        operations=("validate", "batch"),
        native_factory_ids=(ADS6_AIRCRAFT_EXECUTOR_ID,),
        source_refs=("missiondesignsolutions/CADAC/ADS6/aircraft_modules.cpp",),
        blockers=(),
        claim_boundary="Executable AIRCRAFT3 point-mass force model; no rigid-body rotational or physical-effector claim is made.",
    )
    mission = TrajectoryMissionTemplateMetadata(
        id=ADS6_AIRCRAFT_MISSION_ID,
        name="ADS6 AIRCRAFT3 source flight",
        description="Execute the installed AIRCRAFT3 steady/g-turn/escape source behavior.",
        status="development",
        initialization_variants=("source_case_override", "constant_velocity_threat_seam"),
        segment_sequence=("steady", "maneuver_window", "steady"),
        compatible_fidelities=(ADS6_AIRCRAFT_FIDELITY_ID,),
        operations=(
            TrajectoryMissionOperationMetadata(
                fidelity=ADS6_AIRCRAFT_FIDELITY_ID,
                realization_id=ADS6_AIRCRAFT_REALIZATION_ID,
                operation="validate",
                status="available",
                execution_mode="source_grounded_validation",
                common_runner_status="not_available",
                claim_boundary="Validation preserves the point-mass fidelity boundary.",
            ),
            TrajectoryMissionOperationMetadata(
                fidelity=ADS6_AIRCRAFT_FIDELITY_ID,
                realization_id=ADS6_AIRCRAFT_REALIZATION_ID,
                operation="batch",
                status="available",
                execution_mode="source_compatibility_runtime",
                availability_scope="native_runtime_binding",
                common_runner_status="registered",
                executor_id=ADS6_AIRCRAFT_EXECUTOR_ID,
                claim_boundary="Batch dispatch reaches the exact installed ADS6 AIRCRAFT3 plug-in.",
            ),
        ),
        provenance="CADAC ADS6 Flat3 and AIRCRAFT3 source modules",
        claim_boundary="Installed source target flight only; full radar/SAM communication scheduling is a later composition boundary.",
    )
    return TrajectoryModelMetadata(
        id=ADS6_AIRCRAFT_MODEL_ID,
        name="CADAC ADS6 aircraft target",
        version=ADS6_AIRCRAFT_MODEL_VERSION,
        description="CADAC ADS6 AIRCRAFT3 point-mass target exposed as a Taoryx plug-in.",
        presentation=TrajectoryModelPresentationMetadata(
            display_name="CADAC ADS6 Aircraft",
            short_name="ADS6 Aircraft",
            summary="Flat-Earth aircraft target with steady, g-turn, and escape force-model behaviors.",
            category="aircraft",
            subcategory="target",
            sort_key="cadac-ads6-aircraft",
            badges=("CADAC", "3-DoF", "target", "force model"),
            default_fidelity_id=ADS6_AIRCRAFT_FIDELITY_ID,
            default_mission_template_id=ADS6_AIRCRAFT_MISSION_ID,
            default_output_channel_ids=("position_ned_m", "velocity_ned_mps", "bank_deg", "normal_load_factor_g"),
            properties=(
                TrajectoryModelPropertyMetadata(
                    id="source_model",
                    label="Source model",
                    description="Upstream CADAC actor identifier.",
                    semantic_role="identity",
                    value_type="string",
                    value_kind="declared",
                    value="AIRCRAFT3",
                    value_declared=True,
                    source_refs=("missiondesignsolutions/CADAC/ADS6",),
                    provenance="CADAC ADS6 source package",
                    claim_boundary="Identity metadata only.",
                ),
            ),
        ),
        family_id=ADS6_AIRCRAFT_MODEL_ID,
        physical_family="aircraft_target",
        model_kind="vehicle_plugin",
        status="development",
        tags=("cadac", "ads6", "aircraft", "point_mass_3dof", "target"),
        execution_capability_profile="taoryx_universal",
        operations=("discover", "validate", "batch"),
        common_runner_operations=("batch",),
        capabilities=TrajectoryModelCapabilities(
            initialization_modes=("source_case_override", "constant_velocity_threat_seam"),
            segment_types=("steady", "maneuver_window", "g_turn", "escape"),
            termination_modes=("end_time", "ground_impact", "nonfinite_state"),
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
                id=_LOCAL_FRAME_ID,
                name="CADAC ADS6 local level",
                description="Flat-Earth local north/east/down frame used by Flat3.",
                frame_kind="local_tangent",
                axes=("north", "east", "down"),
                handedness="right",
                origin="source local-level reference point",
                orientation="north-east-down",
                source_refs=("missiondesignsolutions/CADAC/ADS6/flat3_modules.cpp",),
                provenance="CADAC ADS6 Flat3 convention",
            ),
        ),
        output_schema=output_schema,
        output_schema_id=output_schema.schema_id,
        output_schema_fingerprint=output_schema.fingerprint,
        configuration_schema_id=schema.schema_id,
        configuration_schema_fingerprint=schema.fingerprint,
        fidelities=(fidelity,),
        fidelity_transitions=(),
        source_refs=("missiondesignsolutions/CADAC/ADS6",),
        provenance="CADAC ADS6 AIRCRAFT3 source-grounded Python reconstruction",
        claim_boundary=(
            "Executable point-mass AIRCRAFT3 source vehicle. Bank and load-factor response affect force orientation but are not promoted to pseudo-6DoF or rigid-body attitude truth."
        ),
    )


####


def _overrides_from_resolved(resolved: Any) -> Ads6AircraftPluginOverrides:
    if not isinstance(resolved, Mapping):
        raise ValueError("ADS6 AIRCRAFT3 resolved configuration root must be a mapping")
    ####
    initial = _group(resolved, "initialization")
    maneuver = _group(resolved, "maneuver")
    response = _group(resolved, "response")
    threat = _group(resolved, "threat")
    runtime = _group(resolved, "runtime")
    threat_track = None
    if bool(threat["threat_track_enabled"]):
        threat_track = Ads6AircraftThreatTrack(
            position_ned_m=(
                float(threat["threat_north_m"]),
                float(threat["threat_east_m"]),
                float(threat["threat_down_m"]),
            ),
            velocity_ned_mps=(
                float(threat["threat_velocity_north_mps"]),
                float(threat["threat_velocity_east_mps"]),
                float(threat["threat_velocity_down_mps"]),
            ),
            reference_time_s=float(threat["threat_reference_time_s"]),
        )
    ####
    return Ads6AircraftPluginOverrides(
        position_ned_m=(
            float(initial["north_m"]),
            float(initial["east_m"]),
            float(initial["down_m"]),
        ),
        speed_mps=float(initial["speed_mps"]),
        heading_deg=float(initial["heading_deg"]),
        flight_path_deg=float(initial["flight_path_deg"]),
        guidance_option=int(maneuver["guidance_option"]),
        guidance_gain=float(maneuver["guidance_gain"]),
        turn_load_g=float(maneuver["turn_load_g"]),
        maneuver_start_s=float(maneuver["maneuver_start_s"]),
        maneuver_stop_s=float(maneuver["maneuver_stop_s"]),
        bank_time_constant_s=float(response["bank_time_constant_s"]),
        bank_limit_deg=float(response["bank_limit_deg"]),
        load_factor_time_constant_s=float(response["load_factor_time_constant_s"]),
        alpha_limit_deg=float(response["alpha_limit_deg"]),
        lift_slope_per_deg=float(response["lift_slope_per_deg"]),
        wing_loading_n_m2=float(response["wing_loading_n_m2"]),
        longitudinal_acceleration_g=float(response["longitudinal_acceleration_g"]),
        threat_track=threat_track,
        end_time_s=float(runtime["end_time_s"]),
        sample_step_s=float(runtime["sample_step_s"]),
    )


####


def _group(resolved: Mapping[str, Any], group_id: str) -> Mapping[str, Any]:
    group = resolved.get(group_id)
    if not isinstance(group, Mapping):
        raise ValueError(f"ADS6 AIRCRAFT3 resolved configuration is missing group {group_id!r}")
    ####
    return group


####


def _configuration_unit(parameter_id: str) -> str | None:
    if parameter_id.endswith("_m"):
        return "m"
    ####
    if parameter_id.endswith("_mps"):
        return "m/s"
    ####
    if parameter_id in {"speed_mps"}:
        return "m/s"
    ####
    if parameter_id.endswith("_deg"):
        return "deg"
    ####
    if parameter_id.endswith("_s"):
        return "s"
    ####
    if parameter_id in {"turn_load_g", "longitudinal_acceleration_g"}:
        return "g"
    ####
    if parameter_id == "lift_slope_per_deg":
        return "1/deg"
    ####
    if parameter_id == "wing_loading_n_m2":
        return "N/m^2"
    ####
    return None


####


def _project_run_result(
    request: MissionCompositionRunRequest,
    run: Ads6AircraftRunResult,
    output_schema: TrajectoryOutputSchema,
) -> MissionCompositionTrajectoryResult:
    selected = resolve_output_selection(
        output_schema,
        request.output,
        fidelity=ADS6_AIRCRAFT_FIDELITY_ID,
        operation="batch",
        realization_id=ADS6_AIRCRAFT_REALIZATION_ID,
        mission_template_id=ADS6_AIRCRAFT_MISSION_ID,
    )
    # The source uses an infinite range sentinel whenever no standalone threat
    # track is installed.  A finite scalar channel cannot carry that sentinel
    # through the common result contract, so omit this conditional channel for
    # this particular run rather than turning it into a fictitious value.
    if not all(isinstance(sample.threat_range_m, int | float) and math.isfinite(sample.threat_range_m) for sample in run.samples):
        selected = tuple(item for item in selected if item.id != "threat_range_m")
    ####
    channels = tuple(_runtime_channel(item, output_schema) for item in selected)
    samples = tuple(
        TrajectorySample(
            time_s=sample.time_s,
            values={channel.id: _sample_value(channel.id, sample) for channel in selected},
        )
        for sample in run.samples
    )
    failed = run.terminated_reason == "nonfinite_state"
    terminated = run.terminated_reason not in {"end_time", "nonfinite_state"}
    object_result = TrajectoryObject(
        object_id="ads6-aircraft-1",
        model_id=ADS6_AIRCRAFT_MODEL_ID,
        realization_id=ADS6_AIRCRAFT_REALIZATION_ID,
        name="CADAC ADS6 aircraft target",
        role="target",
        fidelity=ADS6_AIRCRAFT_FIDELITY_ID,
        status="failed" if failed else "terminated" if terminated else "completed",
        active_from_s=samples[0].time_s,
        active_to_s=samples[-1].time_s,
        terminal_disposition=run.terminated_reason,
        channels=channels,
        samples=samples,
        provenance="CADAC ADS6 AIRCRAFT3 source-compatible point-mass runner",
        claim_boundary=run.claim_boundary,
    )
    events: tuple[TrajectoryEvent, ...] = ()
    if request.output.include_events:
        events = tuple(
            TrajectoryEvent(
                id=f"maneuver-{index + 1}",
                time_s=transition.time_s,
                category="custom",
                kind="cadac-aircraft-maneuver-window",
                object_id="ads6-aircraft-1",
                detail=f"{transition.mode} active={transition.active}",
                data={"active": transition.active, "mode": transition.mode},
            )
            for index, transition in enumerate(run.maneuver_transitions)
        )
    ####
    diagnostics = (
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-point-mass-classification",
            message="ADS6 AIRCRAFT3 bank and load-factor response states orient point-mass force; they are telemetry, not attitude truth.",
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=ADS6_AIRCRAFT_MODEL_ID,
            object_id="ads6-aircraft-1",
        ),
    )
    return MissionCompositionTrajectoryResult(
        provider_id=CADAC_PROVIDER_ID,
        provider_version=ADS6_AIRCRAFT_MODEL_VERSION,
        request_id=request.request_id,
        configuration_fingerprint=request.prepared_configuration.fingerprint,
        primary_model_id=ADS6_AIRCRAFT_MODEL_ID,
        primary_object_id="ads6-aircraft-1",
        status="failed" if failed else "completed",
        objects=(object_result,),
        events=events,
        relationships=(),
        diagnostics=diagnostics,
        claim_boundary="One exact ADS6 AIRCRAFT3 point-mass target returned through the common runner without pseudo-6DoF promotion.",
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


def _sample_value(channel_id: str, sample: Ads6AircraftSample) -> object:
    values: dict[str, object] = {
        "position_ned_m": sample.position_ned_m,
        "velocity_ned_mps": sample.velocity_ned_mps,
        "altitude_m": sample.altitude_m,
        "speed_mps": sample.speed_mps,
        "heading_deg": sample.heading_deg,
        "flight_path_deg": sample.flight_path_deg,
        "mode": sample.mode,
        "maneuver_active": sample.maneuver_active,
        "density_kg_m3": sample.density_kg_m3,
        "pressure_pa": sample.pressure_pa,
        "dynamic_pressure_pa": sample.dynamic_pressure_pa,
        "mach": sample.mach,
        "gravity_mps2": sample.gravity_mps2,
        "commanded_acceleration_ned_mps2": sample.commanded_acceleration_ned_mps2,
        "commanded_acceleration_velocity_mps2": sample.commanded_acceleration_velocity_mps2,
        "commanded_bank_deg": sample.commanded_bank_deg,
        "bank_state_deg": sample.bank_state_deg,
        "bank_deg": sample.bank_deg,
        "bank_limited": sample.bank_limited,
        "commanded_load_factor_g": sample.commanded_load_factor_g,
        "normal_load_factor_g": sample.normal_load_factor_g,
        "load_factor_limit_g": sample.load_factor_limit_g,
        "load_factor_limited": sample.load_factor_limited,
        "longitudinal_acceleration_g": sample.longitudinal_acceleration_g,
        "specific_force_body_mps2": sample.specific_force_body_mps2,
        "threat_range_m": sample.threat_range_m,
    }
    return values[channel_id]


####


__all__ = [
    "ADS6_AIRCRAFT_EXECUTOR_ID",
    "ADS6_AIRCRAFT_FIDELITY_ID",
    "ADS6_AIRCRAFT_MISSION_ID",
    "ADS6_AIRCRAFT_MODEL_ID",
    "ADS6_AIRCRAFT_MODEL_VERSION",
    "ADS6_AIRCRAFT_PHASE_ID",
    "ADS6_AIRCRAFT_REALIZATION_ID",
    "CadacAds6AircraftMissionCompositionProvider",
    "build_default_ads6_aircraft_configuration",
    "register_ads6_aircraft_mission_composition",
]
