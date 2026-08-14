"""Taoryx Mission Composition bridge for the ROCKET6G phase-aware plug-in."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Literal

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

from .aim5_mission_composition import AIM5_MODEL_VERSION, CADAC_PROVIDER_ID
from .configuration_defaults import materialize_configuration_defaults
from .control_metadata import CadacControlOutputEvidence, batch_configuration_control_advertisement
from .output_metadata import cadac_output_quantity
from .rocket6g import Rocket6gPlantRunResult, Rocket6gPlantSample
from .rocket6g_plugin import Rocket6gPluginOverrides, Rocket6gVehiclePlugin

Rocket6gConfigurationValueType = Literal["number", "integer", "boolean", "string", "enum", "vector3", "vector4"]
Rocket6gConfigurationRole = Literal["initialization", "segment", "constraint", "variant", "output"]
Rocket6gOutputDataType = Literal["float64", "int64", "boolean", "string", "json"]

ROCKET6G_MODEL_ID = "cadac.rocket6g.launch_vehicle"
ROCKET6G_MODEL_VERSION = AIM5_MODEL_VERSION
ROCKET6G_FIDELITY_ID = "rigid_body_6dof_surface_allocated"
ROCKET6G_REALIZATION_ID = "cadac-source-phase-program"
ROCKET6G_MISSION_ID = "three_stage_source_phase_program"
ROCKET6G_EXECUTOR_ID = "cadac.rocket6g.phase_aware.batch"
_ECI_FRAME_ID = "cadac.earth_centered_inertial"
_BODY_FRAME_ID = "cadac.body_xyz"


class CadacRocket6gMissionCompositionProvider:
    """Self-describing batch provider for one installed ROCKET6G source program."""

    def __init__(self, plugin: Rocket6gVehiclePlugin) -> None:
        blockers = plugin.validate_installation()
        if blockers:
            raise ValueError("cannot publish ROCKET6G provider with an incomplete installation: " + "; ".join(blockers))
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
        if model_id != ROCKET6G_MODEL_ID:
            raise KeyError(f"unknown ROCKET6G model {model_id!r}")
        ####
        return self._schema

    ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        if model_id != ROCKET6G_MODEL_ID:
            raise KeyError(f"unknown ROCKET6G model {model_id!r}")
        ####
        return self._output_schema

    ####

    def validate_configuration(self, configuration: TrajectoryConfigurationInstance) -> PreparedTrajectoryConfiguration:
        if configuration.fidelity != ROCKET6G_FIDELITY_ID:
            raise ValueError(
                "the ROCKET6G source phase program uses the physical-effector envelope fidelity; "
                "each accepted sample separately reports T3 aggregate-RCS or T4 TVC participation"
            )
        ####
        if configuration.realization_id not in {None, ROCKET6G_REALIZATION_ID}:
            raise ValueError(f"ROCKET6G supports only realization {ROCKET6G_REALIZATION_ID!r}")
        ####
        if configuration.mission_template_id not in {None, ROCKET6G_MISSION_ID}:
            raise ValueError(f"ROCKET6G supports only mission template {ROCKET6G_MISSION_ID!r}")
        ####
        configuration = materialize_configuration_defaults(self._schema, configuration)
        return validate_configuration_instance(self._schema, configuration)

    ####

    def execute_batch(self, request: MissionCompositionRunRequest) -> MissionCompositionTrajectoryResult:
        if request.provider_id != CADAC_PROVIDER_ID or request.model_id != ROCKET6G_MODEL_ID:
            raise ValueError("ROCKET6G executor received a request for another provider/model")
        ####
        prepared = self.validate_configuration(request.prepared_configuration.configuration)
        if prepared.fingerprint != request.prepared_configuration.fingerprint:
            raise ValueError("ROCKET6G prepared configuration does not match provider validation")
        ####
        run = self._plugin.run_batch(_overrides_from_resolved(prepared.resolved))
        return _project_run_result(request, run, self._output_schema)

    ####


####


def build_default_rocket6g_configuration(
    provider: CadacRocket6gMissionCompositionProvider,
    *,
    configuration_id: str = "rocket6g-default",
    overrides: Mapping[str, object] | None = None,
) -> TrajectoryConfigurationInstance:
    """Create one source phase-program configuration from defaults plus bounded overrides."""

    supplied = dict(overrides or {})
    routes = {
        "longitude_deg": ("initialization", "longitude_deg"),
        "latitude_deg": ("initialization", "latitude_deg"),
        "altitude_m": ("initialization", "altitude_m"),
        "geographic_speed_mps": ("initialization", "geographic_speed_mps"),
        "roll_deg": ("initialization", "roll_deg"),
        "pitch_deg": ("initialization", "pitch_deg"),
        "yaw_deg": ("initialization", "yaw_deg"),
        "alpha_deg": ("initialization", "alpha_deg"),
        "beta_deg": ("initialization", "beta_deg"),
        "body_rates_deg_s": ("initialization", "body_rates_deg_s"),
        "tvc_pitch_command_deg": ("commands", "tvc_pitch_command_deg"),
        "tvc_yaw_command_deg": ("commands", "tvc_yaw_command_deg"),
        "thrust_vector_unit_body": ("commands", "thrust_vector_unit_body"),
        "roll_command_deg": ("commands", "roll_command_deg"),
        "pitch_command_deg": ("commands", "pitch_command_deg"),
        "yaw_command_deg": ("commands", "yaw_command_deg"),
        "enable_boost_cutoff": ("runtime", "enable_boost_cutoff"),
        "boost_cutoff_time_s": ("runtime", "boost_cutoff_time_s"),
        "end_time_s": ("runtime", "end_time_s"),
        "sample_step_s": ("runtime", "sample_step_s"),
    }
    unknown = sorted(set(supplied) - set(routes))
    if unknown:
        raise ValueError(f"unknown ROCKET6G configuration overrides: {unknown!r}")
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
    schema = provider.get_model_schema(ROCKET6G_MODEL_ID)
    return TrajectoryConfigurationInstance(
        configuration_id=configuration_id,
        model_id=ROCKET6G_MODEL_ID,
        model_version=ROCKET6G_MODEL_VERSION,
        schema_fingerprint=schema.fingerprint,
        fidelity=ROCKET6G_FIDELITY_ID,
        realization_id=ROCKET6G_REALIZATION_ID,
        mission_template_id=ROCKET6G_MISSION_ID,
        root=root,
    )


####


def register_rocket6g_mission_composition(
    provider: CadacRocket6gMissionCompositionProvider,
    registry: MissionCompositionRunnerRegistry,
) -> None:
    """Register the exact provider/model ROCKET6G batch executor."""

    registry.register(CADAC_PROVIDER_ID, ROCKET6G_MODEL_ID, provider.execute_batch)


####


def _build_configuration_schema(plugin: Rocket6gVehiclePlugin) -> TrajectoryConfigurationSchema:
    source = plugin.source_definition
    initial = source.initial_state
    source_values = source.initial_parameters
    return TrajectoryConfigurationSchema(
        model_id=ROCKET6G_MODEL_ID,
        model_version=ROCKET6G_MODEL_VERSION,
        supported_fidelities=(ROCKET6G_FIDELITY_ID,),
        root=ConfigurationGroupSchema(
            id="rocket6g",
            label="ROCKET6G source phase program",
            description="Three-stage source event program with direct TVC/thrust-vector command boundary.",
            children=(
                ConfigurationGroupSchema(
                    id="initialization",
                    label="Initial truth",
                    children=(
                        _parameter("longitude_deg", "Longitude", "Initial geodetic longitude.", default=initial.longitude_deg, unit="deg"),
                        _parameter("latitude_deg", "Latitude", "Initial geodetic latitude.", default=initial.latitude_deg, unit="deg"),
                        _parameter("altitude_m", "Altitude", "Initial altitude above the source WGS84 ellipsoid.", default=initial.altitude_m, unit="m"),
                        _parameter(
                            "geographic_speed_mps", "Speed", "Initial Earth-relative geographic speed.", default=initial.geographic_speed_mps, unit="m/s"
                        ),
                        _parameter("roll_deg", "Roll", "Initial body roll angle in geodetic axes.", default=initial.roll_deg, unit="deg"),
                        _parameter("pitch_deg", "Pitch", "Initial body pitch angle in geodetic axes.", default=initial.pitch_deg, unit="deg"),
                        _parameter("yaw_deg", "Yaw", "Initial body yaw angle in geodetic axes.", default=initial.yaw_deg, unit="deg"),
                        _parameter("alpha_deg", "Angle of attack", "Initial source angle of attack.", default=initial.alpha_deg, unit="deg"),
                        _parameter("beta_deg", "Sideslip", "Initial source sideslip angle.", default=initial.beta_deg, unit="deg"),
                        _parameter(
                            "body_rates_deg_s",
                            "Body rates",
                            "Initial Earth-relative roll/pitch/yaw rates.",
                            default=initial.body_rates_deg_s,
                            value_type="vector3",
                            unit="deg/s",
                            frame=_BODY_FRAME_ID,
                        ),
                    ),
                ),
                ConfigurationGroupSchema(
                    id="commands",
                    label="Direct command boundary",
                    description="Commands supplied where omitted source guidance/control would feed physical TVC or RCS direction control.",
                    children=(
                        _parameter(
                            "tvc_pitch_command_deg",
                            "TVC pitch command",
                            "Direct source delecx-equivalent command entering the physical nozzle actuator.",
                            default=0.0,
                            unit="deg",
                            role="segment",
                        ),
                        _parameter(
                            "tvc_yaw_command_deg",
                            "TVC yaw command",
                            "Direct source delrcx-equivalent command entering the physical nozzle actuator.",
                            default=0.0,
                            unit="deg",
                            role="segment",
                        ),
                        _parameter(
                            "thrust_vector_unit_body",
                            "Desired thrust-vector direction",
                            "Body-axis direction used by source RCS vector moment modes and, when its force mode is nonzero, aggregate RCS force modes.",
                            default=(1.0, 0.0, 0.0),
                            value_type="vector3",
                            frame=_BODY_FRAME_ID,
                            role="segment",
                        ),
                        _parameter(
                            "roll_command_deg",
                            "RCS roll command",
                            "Geodetic roll command consumed by source RCS modes.",
                            default=float(source_values.get("phibdcomx", 0.0)),
                            unit="deg",
                            role="segment",
                        ),
                        _parameter(
                            "pitch_command_deg",
                            "RCS pitch command",
                            "Geodetic pitch command consumed by source RCS angle-control mode.",
                            default=float(source_values.get("thtbdcomx", 0.0)),
                            unit="deg",
                            role="segment",
                        ),
                        _parameter(
                            "yaw_command_deg",
                            "RCS yaw command",
                            "Geodetic yaw command consumed by source RCS angle-control mode.",
                            default=float(source_values.get("psibdcomx", 0.0)),
                            unit="deg",
                            role="segment",
                        ),
                    ),
                ),
                ConfigurationGroupSchema(
                    id="runtime",
                    label="Runtime",
                    children=(
                        _parameter(
                            "enable_boost_cutoff",
                            "Enable direct boost cutoff",
                            "When true, set the source beco_flag at the requested time after earlier source events have completed.",
                            default=False,
                            value_type="boolean",
                            role="constraint",
                        ),
                        _parameter(
                            "boost_cutoff_time_s",
                            "Boost cutoff time",
                            "Direct source beco_flag time used only when enabled.",
                            default=max(1.0, min(source.end_time_s, 180.0)),
                            unit="s",
                            role="constraint",
                        ),
                        _parameter(
                            "end_time_s", "End time", "Requested propagation horizon.", default=min(source.end_time_s, 15.0), unit="s", role="constraint"
                        ),
                        _parameter(
                            "sample_step_s",
                            "Sample cadence",
                            "Returned trajectory cadence.",
                            default=max(source.plot_step_s or 0.1, source.integration_step_s),
                            unit="s",
                            role="constraint",
                        ),
                    ),
                ),
            ),
        ),
        claim_boundary=(
            "Configuration selects one source phase program under the T4 envelope because physical TVC participates. "
            "Per-sample telemetry preserves T3 aggregate-RCS phases and mixed-axis limitations."
        ),
    )


####


def _parameter(
    parameter_id: str,
    label: str,
    description: str,
    *,
    default: object,
    value_type: Rocket6gConfigurationValueType = "number",
    unit: str | None = None,
    frame: str | None = None,
    role: Rocket6gConfigurationRole = "initialization",
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
        role=role,
        compatible_fidelities=(ROCKET6G_FIDELITY_ID,),
        frame=frame,
        provenance="missiondesignsolutions/CADAC/ROCKET6G/input.asc",
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
    data_type: Rocket6gOutputDataType = "float64",
    interpolation: Literal["linear", "step", "periodic", "slerp", "event"] = "linear",
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
        interpolation=interpolation,
        availability="guaranteed",
        compatible_fidelities=(ROCKET6G_FIDELITY_ID,),
        compatible_realizations=(ROCKET6G_REALIZATION_ID,),
        compatible_mission_templates=(ROCKET6G_MISSION_ID,),
        operations=("batch",),
        provenance="CADAC ROCKET6G source-grounded phase-aware plant",
        claim_boundary="Available from the Python source reconstruction; compiled-CADAC numerical parity remains pending.",
    )


####


def _build_output_schema() -> TrajectoryOutputSchema:
    core = (
        _output("position_inertial_m", "Inertial position", "Earth-centered inertial position.", unit="m", shape=(3,), frame=_ECI_FRAME_ID),
        _output("velocity_inertial_mps", "Inertial velocity", "Earth-centered inertial velocity.", unit="m/s", shape=(3,), frame=_ECI_FRAME_ID),
        _output("quaternion_wxyz", "Attitude quaternion", "Scalar-first inertial-to-body quaternion.", shape=(4,)),
        _output(
            "body_rates_inertial_rad_s",
            "Inertial body rates",
            "Body angular velocity relative to inertial axes.",
            unit="rad/s",
            shape=(3,),
            frame=_BODY_FRAME_ID,
        ),
    )
    telemetry = (
        _output(
            "body_rates_earth_rad_s", "Earth-relative body rates", "Body angular velocity relative to Earth.", unit="rad/s", shape=(3,), frame=_BODY_FRAME_ID
        ),
        _output("longitude_deg", "Longitude", "Geodetic longitude.", unit="deg"),
        _output("latitude_deg", "Latitude", "Geodetic latitude.", unit="deg"),
        _output("altitude_m", "Altitude", "Altitude above the source WGS84 ellipsoid.", unit="m"),
        _output("geographic_speed_mps", "Geographic speed", "Earth-relative geographic speed.", unit="m/s"),
        _output("heading_deg", "Heading", "Geodetic heading angle.", unit="deg"),
        _output("flight_path_deg", "Flight-path angle", "Geodetic flight-path angle.", unit="deg"),
        _output("roll_deg", "Roll", "Geodetic body roll angle.", unit="deg"),
        _output("pitch_deg", "Pitch", "Geodetic body pitch angle.", unit="deg"),
        _output("yaw_deg", "Yaw", "Geodetic body yaw angle.", unit="deg"),
        _output("alpha_deg", "Angle of attack", "Source aerodynamic angle of attack.", unit="deg"),
        _output("beta_deg", "Sideslip", "Source aerodynamic sideslip.", unit="deg"),
        _output("total_alpha_deg", "Total incidence", "Aeroballistic total angle of attack.", unit="deg"),
        _output("aerodynamic_roll_deg", "Aerodynamic roll", "Aeroballistic roll angle.", unit="deg"),
        _output("mach", "Mach", "Source atmosphere-relative Mach number."),
        _output("dynamic_pressure_pa", "Dynamic pressure", "Source dynamic pressure.", unit="Pa"),
        _output("active_stage", "Active stage", "Source stage number selected by maero.", data_type="int64", interpolation="step"),
        _output("source_phase", "Source phase", "Runtime control phase derived from active source effectors.", data_type="string", interpolation="step"),
        _output("runtime_fidelity", "Runtime fidelity", "Per-sample Taoryx fidelity boundary.", data_type="string", interpolation="step"),
        _output(
            "control_realization", "Control realization", "Per-sample active physical/direct-wrench realization.", data_type="string", interpolation="step"
        ),
        _output("propulsion_mode", "Propulsion mode", "Source mprop value.", data_type="int64", interpolation="step"),
        _output("rcs_moment_mode", "RCS moment mode", "Source mrcs_moment value.", data_type="int64", interpolation="step"),
        _output(
            "rcs_force_mode",
            "RCS force mode",
            "Source mrcs_force value; zero means the requested thrust-vector direction produces no aggregate RCS force in that sample.",
            data_type="int64",
            interpolation="step",
        ),
        _output("tvc_mode", "TVC mode", "Source mtvc value.", data_type="int64", interpolation="step"),
        _output("mass_kg", "Mass", "Source stage gross mass after fuel expenditure.", unit="kg"),
        _output("remaining_fuel_kg", "Remaining fuel", "Current source stage fuel remaining.", unit="kg"),
        _output("center_of_gravity_m", "Center of gravity", "Source center-of-gravity location from the nose.", unit="m"),
        _output("inertia_diagonal_kgm2", "Inertia diagonal", "Source roll/pitch/yaw principal inertias.", unit="kg*m^2", shape=(3,), frame=_BODY_FRAME_ID),
        _output("thrust_n", "Thrust", "Current source rocket thrust.", unit="N"),
        _output(
            "requested_tvc_control_deg",
            "Requested TVC control",
            "Direct pitch/yaw control request entering the source TVC module.",
            unit="deg",
            shape=(2,),
            frame=_BODY_FRAME_ID,
        ),
        _output(
            "requested_nozzle_deg", "Requested nozzle", "Requested pitch/yaw nozzle deflection after source gain.", unit="deg", shape=(2,), frame=_BODY_FRAME_ID
        ),
        _output(
            "achieved_nozzle_deg", "Achieved nozzle", "Physical actuator-achieved pitch/yaw nozzle deflection.", unit="deg", shape=(2,), frame=_BODY_FRAME_ID
        ),
        _output(
            "requested_rcs_attitude_deg",
            "Requested RCS attitude",
            "Requested roll/pitch/yaw attitude coordinates entering aggregate RCS.",
            unit="deg",
            shape=(3,),
            frame=_BODY_FRAME_ID,
        ),
        _output(
            "requested_thrust_vector_unit_body",
            "Requested thrust-vector direction",
            "Requested unit thrust direction entering aggregate RCS.",
            shape=(3,),
            frame=_BODY_FRAME_ID,
        ),
        _output("rcs_force_body_n", "RCS force", "Axis-aggregate RCS body force.", unit="N", shape=(3,), frame=_BODY_FRAME_ID),
        _output("rcs_moment_body_nm", "RCS moment", "Axis-aggregate RCS body moment.", unit="N*m", shape=(3,), frame=_BODY_FRAME_ID),
        _output("force_body_n", "Total body force", "Total non-gravitational body force.", unit="N", shape=(3,), frame=_BODY_FRAME_ID),
        _output("moment_body_nm", "Total body moment", "Total body moment.", unit="N*m", shape=(3,), frame=_BODY_FRAME_ID),
    )
    return TrajectoryOutputSchema(
        model_id=ROCKET6G_MODEL_ID,
        model_version=ROCKET6G_MODEL_VERSION,
        core_channels=core,
        telemetry_channels=telemetry,
        telemetry_groups=(
            TrajectoryTelemetryGroupMetadata(
                id="phase_aware_plant",
                label="Phase-aware plant",
                description="WGS84 truth, source phases, stage resources, TVC, RCS, and total wrench.",
                channel_ids=tuple(item.id for item in telemetry),
            ),
        ),
        entity_output=TrajectoryEntityOutputMetadata(),
        claim_boundary="Single launch-vehicle object with internal stage/phase transitions; stages are not projected as spawned child entities.",
    )


####


def _build_model_metadata(
    schema: TrajectoryConfigurationSchema,
    output_schema: TrajectoryOutputSchema,
) -> TrajectoryModelMetadata:
    fidelity = TrajectoryFidelityMetadata(
        id=ROCKET6G_FIDELITY_ID,
        label="CADAC ROCKET6G mixed-effector 6-DoF envelope",
        rank=3,
        declared=True,
        dynamics_fidelity="rigid_body_6dof",
        input_realization="actuator_allocated",
        actuator_types=("mixed",),
        compatibility_aliases=("rigid_body_6dof_effector_allocated",),
        runtime_fidelity="phase_reported_rigid_body_6dof",
        control_realization="mixed_tvc_rcs",
        promotion_status="development",
        operations=("validate", "batch"),
        profile_id="cadac.rocket6g.source-phase-program",
        blockers=(
            "compiled CADAC numerical golden parity is not yet registered",
            "LTG/autopilot and GPS/INS/star-tracker behavior are not part of the direct-command plant realization",
        ),
        required_operations=("WGS84 rigid-body propagation", "stage events", "physical TVC", "aggregate RCS"),
        claim_boundary=(
            "T4 is the run envelope because physical TVC participates. Aggregate RCS remains T3 by phase/axis and is never promoted "
            "to physical thruster allocation; runtime telemetry preserves that distinction."
        ),
    )
    realization = TrajectoryRealizationMetadata(
        id=ROCKET6G_REALIZATION_ID,
        label="CADAC source phase program",
        description="Three-stage source event program with physical TVC, axis-aggregate RCS, and direct omitted-controller command inputs.",
        status="available",
        dynamics_fidelities=("rigid_body_6dof",),
        input_realization="actuator_allocated",
        actuator_types=("mixed",),
        controls=batch_configuration_control_advertisement(
            schema,
            channels={
                "tvc.pitch.deflection": ("commands", "tvc_pitch_command_deg"),
                "tvc.yaw.deflection": ("commands", "tvc_yaw_command_deg"),
                "rcs.thrust_vector.direction": ("commands", "thrust_vector_unit_body"),
                "rcs.roll.attitude_command": ("commands", "roll_command_deg"),
                "rcs.pitch.attitude_command": ("commands", "pitch_command_deg"),
                "rcs.yaw.attitude_command": ("commands", "yaw_command_deg"),
            },
            output_evidence={
                "tvc.pitch.deflection": CadacControlOutputEvidence("requested_tvc_control_deg", ("achieved_nozzle_deg",), 0, (0,)),
                "tvc.yaw.deflection": CadacControlOutputEvidence("requested_tvc_control_deg", ("achieved_nozzle_deg",), 1, (1,)),
                "rcs.thrust_vector.direction": CadacControlOutputEvidence(
                    "requested_thrust_vector_unit_body",
                    ("rcs_force_body_n", "rcs_moment_body_nm"),
                ),
                "rcs.roll.attitude_command": CadacControlOutputEvidence("requested_rcs_attitude_deg", ("rcs_moment_body_nm",), 0, (0,)),
                "rcs.pitch.attitude_command": CadacControlOutputEvidence("requested_rcs_attitude_deg", ("rcs_moment_body_nm",), 1, (1,)),
                "rcs.yaw.attitude_command": CadacControlOutputEvidence("requested_rcs_attitude_deg", ("rcs_moment_body_nm",), 2, (2,)),
            },
            authority_id="direct_tvc_and_rcs_commands",
            authority="native_bridge",
            authority_description="Caller-owned physical-TVC and aggregate-RCS command coordinates fixed for one ROCKET6G batch.",
            intent_id="direct_launch_vehicle_control",
            intent_label="Direct Launch-Vehicle Control",
            intent_description="Set reconstructed ROCKET6G TVC/RCS command coordinates at the omitted source guidance/control boundary.",
            mission_ids=(ROCKET6G_MISSION_ID,),
            source_refs=("missiondesignsolutions/CADAC/ROCKET6G",),
            claim_boundary="The caller supplies fixed direct command coordinates through the batch configuration. LTG/autopilot and common-runner step control remain unavailable.",
        ),
        fidelity_aliases=(ROCKET6G_FIDELITY_ID, "rigid_body_6dof_effector_allocated"),
        mission_template_ids=(ROCKET6G_MISSION_ID,),
        operations=("validate", "batch"),
        native_factory_ids=(ROCKET6G_EXECUTOR_ID,),
        source_refs=("missiondesignsolutions/CADAC/ROCKET6G",),
        blockers=("source command generation and navigation estimators are excluded",),
        claim_boundary="Available as a phase-aware physical plant, not as a complete source insertion-guidance/navigation replay.",
    )
    mission = TrajectoryMissionTemplateMetadata(
        id=ROCKET6G_MISSION_ID,
        name="ROCKET6G three-stage source phase program",
        description="Propagate the source event/stage schedule while injecting bounded direct commands at omitted controller boundaries.",
        status="development",
        initialization_variants=("source_launch_override",),
        segment_sequence=("aggregate_rcs", "mixed_tvc_rcs", "aggregate_rcs"),
        compatible_fidelities=(ROCKET6G_FIDELITY_ID,),
        operations=(
            TrajectoryMissionOperationMetadata(
                fidelity=ROCKET6G_FIDELITY_ID,
                realization_id=ROCKET6G_REALIZATION_ID,
                operation="validate",
                status="available",
                execution_mode="source_grounded_validation",
                common_runner_status="not_available",
                claim_boundary="Validation prepares one exact source phase program.",
            ),
            TrajectoryMissionOperationMetadata(
                fidelity=ROCKET6G_FIDELITY_ID,
                realization_id=ROCKET6G_REALIZATION_ID,
                operation="batch",
                status="available",
                execution_mode="source_ordered_phase_aware_plant",
                availability_scope="native_runtime_binding",
                common_runner_status="registered",
                executor_id=ROCKET6G_EXECUTOR_ID,
                claim_boundary="Batch dispatch reaches the exact installed ROCKET6G source-program plug-in.",
            ),
        ),
        provenance="CADAC ROCKET6G input events, WGS84 rigid body, propulsion, aerodynamics, RCS, TVC, forces, Newton, and Euler modules",
        claim_boundary="The source stage/event schedule participates; omitted guidance/navigation remains explicit.",
    )
    return TrajectoryModelMetadata(
        id=ROCKET6G_MODEL_ID,
        name="CADAC ROCKET6G launch vehicle",
        version=ROCKET6G_MODEL_VERSION,
        description="CADAC ROCKET6G three-stage launch vehicle exposed as a phase-aware Taoryx plug-in.",
        presentation=TrajectoryModelPresentationMetadata(
            display_name="CADAC ROCKET6G",
            short_name="ROCKET6G",
            summary="Three-stage WGS84 rigid-body launch vehicle with physical TVC and axis-aggregate RCS.",
            category="launch_vehicle",
            subcategory="three_stage_solid_booster",
            sort_key="cadac-rocket6g",
            badges=("CADAC", "6-DoF", "three-stage", "TVC", "RCS", "development"),
            default_fidelity_id=ROCKET6G_FIDELITY_ID,
            default_mission_template_id=ROCKET6G_MISSION_ID,
            default_output_channel_ids=(
                "position_inertial_m",
                "velocity_inertial_mps",
                "quaternion_wxyz",
                "body_rates_inertial_rad_s",
            ),
            properties=(
                TrajectoryModelPropertyMetadata(
                    id="source_stage_count",
                    label="Source stage count",
                    description="Number of serial source booster stages in the installed phase program.",
                    semantic_role="capability",
                    value_type="integer",
                    value_kind="exact",
                    value=3,
                    value_declared=True,
                    source_refs=("missiondesignsolutions/CADAC/ROCKET6G/input.asc",),
                    provenance="source event/stage declarations",
                    claim_boundary="Describes internal stage transitions, not dynamically spawned Taoryx child vehicles.",
                ),
            ),
        ),
        family_id=ROCKET6G_MODEL_ID,
        physical_family="three_stage_launch_vehicle",
        model_kind="phase_aware_rigid_body_launch_vehicle",
        status="development",
        tags=("cadac", "rocket6g", "launch-vehicle", "wgs84", "tvc", "rcs"),
        operations=("discover", "validate", "batch"),
        common_runner_operations=("batch",),
        capabilities=TrajectoryModelCapabilities(
            initialization_modes=("source_launch_override",),
            segment_types=("aggregate_rcs", "physical_tvc", "mixed_tvc_rcs", "ballistic_coast"),
            termination_modes=("end_time", "nonfinite_state"),
            operations=("discover", "validate", "batch"),
            supports_custom_segments=False,
            supports_deployment=False,
            supports_staging=True,
            supports_dynamic_child_generation=False,
            supports_multiple_stages=True,
            supports_submodels=True,
        ),
        realizations=(realization,),
        mission_templates=(mission,),
        deployments=(),
        reference_frames=(
            TrajectoryReferenceFrameMetadata(
                id=_ECI_FRAME_ID,
                name="CADAC Earth-centered inertial",
                description="Earth-centered inertial frame used by ROCKET6G Newton propagation.",
                frame_kind="inertial",
                axes=("x", "y", "z"),
                handedness="right",
                origin="Earth center",
                orientation="source inertial axes with Greenwich celestial longitude zero at simulation start",
                source_refs=("missiondesignsolutions/CADAC/ROCKET6G/newton.cpp",),
                provenance="CADAC ROCKET6G WGS84 convention",
            ),
            TrajectoryReferenceFrameMetadata(
                id=_BODY_FRAME_ID,
                name="CADAC launch-vehicle body axes",
                description="Body-fixed axes used by source forces, moments, rates, TVC, and RCS.",
                frame_kind="body",
                axes=("x", "y", "z"),
                handedness="right",
                origin="instantaneous vehicle reference point",
                orientation="source ROCKET6G body convention",
                source_refs=("missiondesignsolutions/CADAC/ROCKET6G",),
                provenance="CADAC ROCKET6G body convention",
            ),
        ),
        output_schema=output_schema,
        output_schema_id=output_schema.schema_id,
        output_schema_fingerprint=output_schema.fingerprint,
        configuration_schema_id=schema.schema_id,
        configuration_schema_fingerprint=schema.fingerprint,
        fidelities=(fidelity,),
        fidelity_transitions=(),
        source_refs=("missiondesignsolutions/CADAC/ROCKET6G",),
        provenance="CADAC ROCKET6G source-grounded Python phase-aware plant reconstruction",
        claim_boundary="Executable at development status; command-generation/navigation and compiled parity are separate promotion gates.",
    )


####


def _overrides_from_resolved(resolved: Any) -> Rocket6gPluginOverrides:
    if not isinstance(resolved, Mapping):
        raise ValueError("ROCKET6G resolved configuration root must be a mapping")
    ####
    initial = _group(resolved, "initialization")
    commands = _group(resolved, "commands")
    runtime = _group(resolved, "runtime")
    enable_cutoff = bool(runtime["enable_boost_cutoff"])
    return Rocket6gPluginOverrides(
        longitude_deg=float(initial["longitude_deg"]),
        latitude_deg=float(initial["latitude_deg"]),
        altitude_m=float(initial["altitude_m"]),
        geographic_speed_mps=float(initial["geographic_speed_mps"]),
        roll_deg=float(initial["roll_deg"]),
        pitch_deg=float(initial["pitch_deg"]),
        yaw_deg=float(initial["yaw_deg"]),
        alpha_deg=float(initial["alpha_deg"]),
        beta_deg=float(initial["beta_deg"]),
        body_rates_deg_s=_vector3(initial["body_rates_deg_s"]),
        tvc_pitch_command_deg=float(commands["tvc_pitch_command_deg"]),
        tvc_yaw_command_deg=float(commands["tvc_yaw_command_deg"]),
        thrust_vector_unit_body=_vector3(commands["thrust_vector_unit_body"]),
        roll_command_deg=float(commands["roll_command_deg"]),
        pitch_command_deg=float(commands["pitch_command_deg"]),
        yaw_command_deg=float(commands["yaw_command_deg"]),
        boost_cutoff_time_s=float(runtime["boost_cutoff_time_s"]) if enable_cutoff else None,
        end_time_s=float(runtime["end_time_s"]),
        sample_step_s=float(runtime["sample_step_s"]),
    )


####


def _group(root: Mapping[str, object], name: str) -> Mapping[str, object]:
    value = root.get(name)
    if not isinstance(value, Mapping):
        raise ValueError(f"ROCKET6G resolved configuration requires group {name!r}")
    ####
    return value


####


def _vector3(value: object) -> tuple[float, float, float]:
    if not isinstance(value, (tuple, list)) or len(value) != 3:
        raise ValueError("ROCKET6G vector configuration requires exactly three components")
    ####
    return (float(value[0]), float(value[1]), float(value[2]))


####


def _configuration_unit(parameter_id: str) -> str | None:
    if parameter_id in {
        "longitude_deg",
        "latitude_deg",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "alpha_deg",
        "beta_deg",
        "tvc_pitch_command_deg",
        "tvc_yaw_command_deg",
        "roll_command_deg",
        "pitch_command_deg",
        "yaw_command_deg",
    }:
        return "deg"
    ####
    if parameter_id == "altitude_m":
        return "m"
    ####
    if parameter_id == "geographic_speed_mps":
        return "m/s"
    ####
    if parameter_id == "body_rates_deg_s":
        return "deg/s"
    ####
    if parameter_id in {"boost_cutoff_time_s", "end_time_s", "sample_step_s"}:
        return "s"
    ####
    return None


####


def _project_run_result(
    request: MissionCompositionRunRequest,
    run: Rocket6gPlantRunResult,
    output_schema: TrajectoryOutputSchema,
) -> MissionCompositionTrajectoryResult:
    selected = resolve_output_selection(
        output_schema,
        request.output,
        fidelity=ROCKET6G_FIDELITY_ID,
        operation="batch",
        realization_id=ROCKET6G_REALIZATION_ID,
        mission_template_id=ROCKET6G_MISSION_ID,
    )
    channels = tuple(_runtime_channel(item, output_schema) for item in selected)
    samples = tuple(
        TrajectorySample(
            time_s=sample.time_s,
            values={channel.id: _sample_value(channel.id, sample) for channel in selected},
        )
        for sample in run.samples
    )
    failed = run.terminated_reason != "end_time"
    object_result = TrajectoryObject(
        object_id="launch_vehicle1",
        model_id=ROCKET6G_MODEL_ID,
        realization_id=ROCKET6G_REALIZATION_ID,
        name="CADAC ROCKET6G launch vehicle",
        role="launch_vehicle",
        fidelity=ROCKET6G_FIDELITY_ID,
        status="failed" if failed else "completed",
        active_from_s=samples[0].time_s,
        active_to_s=samples[-1].time_s,
        terminal_disposition=run.terminated_reason,
        channels=channels,
        samples=samples,
        provenance="CADAC ROCKET6G source-grounded phase-aware mixed RCS/TVC plant",
        claim_boundary="Run-envelope T4 does not promote axis-aggregate RCS; each sample reports its actual phase fidelity and realization.",
    )
    projected_events = tuple(
        TrajectoryEvent(
            id=f"rocket6g-source-event-{event.event_index}",
            time_s=event.time_s,
            category="burnout" if event.watch_variable.casefold() == "thrust" else "segment",
            kind=f"source_event_{event.event_index}",
            object_id="launch_vehicle1",
            detail=f"{event.phase_before} -> {event.phase_after}; stage {event.stage_before} -> {event.stage_after}",
            data=event.model_dump(mode="json"),
        )
        for event in run.events
    )
    diagnostics: list[MissionCompositionDiagnostic] = [
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-parity-pending",
            message="ROCKET6G executes through the source-grounded Python phase-aware plant; compiled-CADAC numerical parity is not yet promoted.",
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=ROCKET6G_MODEL_ID,
            object_id="launch_vehicle1",
        ),
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-mixed-fidelity-phase-reporting",
            message="The run uses a T4 envelope because physical TVC participates; aggregate-RCS phases and axes remain explicitly T3 in returned telemetry.",
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=ROCKET6G_MODEL_ID,
            object_id="launch_vehicle1",
        ),
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-direct-command-boundary",
            message="LTG/autopilot and GPS/INS/star-tracker modules are not executed; direct commands enter at their declared TVC/RCS boundary.",
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=ROCKET6G_MODEL_ID,
            object_id="launch_vehicle1",
        ),
    ]
    if failed:
        diagnostics.append(
            MissionCompositionDiagnostic(
                severity="error",
                code="cadac-nonfinite-state",
                message="ROCKET6G propagation terminated after a non-finite state was detected.",
                phase="execution",
                recoverability="fatal",
                provider_id=CADAC_PROVIDER_ID,
                model_id=ROCKET6G_MODEL_ID,
                object_id="launch_vehicle1",
            )
        )
    ####
    return MissionCompositionTrajectoryResult(
        provider_id=CADAC_PROVIDER_ID,
        provider_version=ROCKET6G_MODEL_VERSION,
        request_id=request.request_id,
        configuration_fingerprint=request.prepared_configuration.fingerprint,
        primary_model_id=ROCKET6G_MODEL_ID,
        primary_object_id="launch_vehicle1",
        status="failed" if failed else "completed",
        objects=(object_result,),
        events=projected_events,
        relationships=(),
        diagnostics=tuple(diagnostics),
        claim_boundary="One exact phase-aware ROCKET6G launch-vehicle object returned through the common runner with phase-specific fidelity telemetry.",
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


def _sample_value(channel_id: str, sample: Rocket6gPlantSample) -> object:
    return getattr(sample, channel_id)


####


__all__ = [
    "ROCKET6G_EXECUTOR_ID",
    "ROCKET6G_FIDELITY_ID",
    "ROCKET6G_MISSION_ID",
    "ROCKET6G_MODEL_ID",
    "ROCKET6G_MODEL_VERSION",
    "ROCKET6G_REALIZATION_ID",
    "CadacRocket6gMissionCompositionProvider",
    "build_default_rocket6g_configuration",
    "register_rocket6g_mission_composition",
]
