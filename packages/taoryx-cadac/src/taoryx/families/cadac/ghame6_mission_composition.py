"""Taoryx Mission Composition bridge for the GHAME6 phase-aware plug-in."""

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

from .aim5_mission_composition import CADAC_PROVIDER_ID
from .configuration_defaults import materialize_configuration_defaults
from .control_metadata import CadacControlOutputEvidence, batch_configuration_control_advertisement
from .ghame6 import Ghame6HyperSample, Ghame6RunResult
from .ghame6_plugin import Ghame6PluginOverrides, Ghame6VehiclePlugin
from .output_metadata import cadac_output_quantity

Ghame6ConfigurationValueType = Literal["number", "integer", "boolean", "string", "enum", "vector3", "vector4"]
Ghame6ConfigurationRole = Literal["initialization", "segment", "constraint", "variant", "output"]
Ghame6OutputDataType = Literal["float64", "int64", "boolean", "string", "json"]

GHAME6_MODEL_ID = "cadac.ghame6.hypersonic_vehicle"
GHAME6_SATELLITE_MODEL_ID = "cadac.ghame6.satellite"
GHAME6_RADAR_MODEL_ID = "cadac.ghame6.ground_site"
GHAME6_MODEL_VERSION = "0.8.0"
GHAME6_FIDELITY_ID = "rigid_body_6dof_surface_allocated"
GHAME6_REALIZATION_ID = "cadac-source-atmospheric-surfaces-to-aggregate-rcs"
GHAME6_MISSION_ID = "atmospheric_to_exo_intercept"
GHAME6_EXECUTOR_ID = "cadac.ghame6.phase_aware.batch"
_ECI_FRAME_ID = "cadac.earth_centered_inertial"
_BODY_FRAME_ID = "cadac.body_xyz"


class CadacGhame6MissionCompositionProvider:
    """Self-describing batch provider for one installed GHAME6 source mission."""

    def __init__(self, plugin: Ghame6VehiclePlugin) -> None:
        blockers = plugin.validate_installation()
        if blockers:
            raise ValueError("cannot publish GHAME6 provider with an incomplete installation: " + "; ".join(blockers))
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
        if model_id != GHAME6_MODEL_ID:
            raise KeyError(f"unknown GHAME6 model {model_id!r}")
        ####
        return self._schema

    ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        if model_id != GHAME6_MODEL_ID:
            raise KeyError(f"unknown GHAME6 model {model_id!r}")
        ####
        return self._output_schema

    ####

    def validate_configuration(self, configuration: TrajectoryConfigurationInstance) -> PreparedTrajectoryConfiguration:
        if configuration.fidelity != GHAME6_FIDELITY_ID:
            raise ValueError(
                "GHAME6 uses the T4 run envelope because physical atmospheric surfaces participate; returned samples preserve later T3 aggregate-RCS phases"
            )
        ####
        if configuration.realization_id not in {None, GHAME6_REALIZATION_ID}:
            raise ValueError(f"GHAME6 supports only realization {GHAME6_REALIZATION_ID!r}")
        ####
        if configuration.mission_template_id not in {None, GHAME6_MISSION_ID}:
            raise ValueError(f"GHAME6 supports only mission template {GHAME6_MISSION_ID!r}")
        ####
        configuration = materialize_configuration_defaults(self._schema, configuration)
        return validate_configuration_instance(self._schema, configuration)

    ####

    def execute_batch(self, request: MissionCompositionRunRequest) -> MissionCompositionTrajectoryResult:
        if request.provider_id != CADAC_PROVIDER_ID or request.model_id != GHAME6_MODEL_ID:
            raise ValueError("GHAME6 executor received a request for another provider/model")
        ####
        prepared = self.validate_configuration(request.prepared_configuration.configuration)
        if prepared.fingerprint != request.prepared_configuration.fingerprint:
            raise ValueError("GHAME6 prepared configuration does not match provider validation")
        ####
        run = self._plugin.run_batch(_overrides_from_resolved(prepared.resolved))
        return _project_run_result(request, run, self._output_schema)

    ####


####


def build_default_ghame6_configuration(
    provider: CadacGhame6MissionCompositionProvider,
    *,
    configuration_id: str = "ghame6-default",
    overrides: Mapping[str, object] | None = None,
) -> TrajectoryConfigurationInstance:
    """Create one source mission configuration from defaults plus bounded overrides."""

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
        "aileron_command_deg": ("commands", "aileron_command_deg"),
        "elevator_command_deg": ("commands", "elevator_command_deg"),
        "rudder_command_deg": ("commands", "rudder_command_deg"),
        "thrust_vector_unit_body": ("commands", "thrust_vector_unit_body"),
        "roll_command_deg": ("commands", "roll_command_deg"),
        "pitch_command_deg": ("commands", "pitch_command_deg"),
        "yaw_command_deg": ("commands", "yaw_command_deg"),
        "alpha_command_deg": ("commands", "alpha_command_deg"),
        "beta_command_deg": ("commands", "beta_command_deg"),
        "lateral_acceleration_command_g": ("commands", "lateral_acceleration_command_g"),
        "normal_acceleration_command_g": ("commands", "normal_acceleration_command_g"),
        "enable_boost_cutoff": ("runtime", "enable_boost_cutoff"),
        "boost_cutoff_time_s": ("runtime", "boost_cutoff_time_s"),
        "enable_terminal_lock": ("runtime", "enable_terminal_lock"),
        "terminal_lock_time_s": ("runtime", "terminal_lock_time_s"),
        "end_time_s": ("runtime", "end_time_s"),
        "sample_step_s": ("runtime", "sample_step_s"),
        "random_seed": ("runtime", "random_seed"),
    }
    unknown = sorted(set(supplied) - set(routes))
    if unknown:
        raise ValueError(f"unknown GHAME6 configuration overrides: {unknown!r}")
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
    schema = provider.get_model_schema(GHAME6_MODEL_ID)
    return TrajectoryConfigurationInstance(
        configuration_id=configuration_id,
        model_id=GHAME6_MODEL_ID,
        model_version=GHAME6_MODEL_VERSION,
        schema_fingerprint=schema.fingerprint,
        fidelity=GHAME6_FIDELITY_ID,
        realization_id=GHAME6_REALIZATION_ID,
        mission_template_id=GHAME6_MISSION_ID,
        root=root,
    )


####


def register_ghame6_mission_composition(
    provider: CadacGhame6MissionCompositionProvider,
    registry: MissionCompositionRunnerRegistry,
) -> None:
    """Register the exact provider/model GHAME6 batch executor."""

    registry.register(CADAC_PROVIDER_ID, GHAME6_MODEL_ID, provider.execute_batch)


####


def _build_configuration_schema(plugin: Ghame6VehiclePlugin) -> TrajectoryConfigurationSchema:
    source = plugin.source_definition
    initial = source.initial_state
    source_values = source.initial_parameters
    return TrajectoryConfigurationSchema(
        model_id=GHAME6_MODEL_ID,
        model_version=GHAME6_MODEL_VERSION,
        supported_fidelities=(GHAME6_FIDELITY_ID,),
        root=ConfigurationGroupSchema(
            id="ghame6",
            label="GHAME6 source mission",
            description="Atmospheric physical-surface vehicle transitioning to aggregate-RCS transfer/interceptor phases with SAT3 and RADAR0.",
            children=(
                ConfigurationGroupSchema(
                    id="initialization",
                    label="Initial truth",
                    children=(
                        _parameter(
                            "longitude_deg", "Longitude", "Initial geodetic longitude after source mission planning.", default=initial.longitude_deg, unit="deg"
                        ),
                        _parameter(
                            "latitude_deg", "Latitude", "Initial geodetic latitude after source mission planning.", default=initial.latitude_deg, unit="deg"
                        ),
                        _parameter("altitude_m", "Altitude", "Initial altitude above the source ellipsoid.", default=initial.altitude_m, unit="m"),
                        _parameter("geographic_speed_mps", "Speed", "Initial Earth-relative speed.", default=initial.geographic_speed_mps, unit="m/s"),
                        _parameter("roll_deg", "Roll", "Initial geodetic roll.", default=initial.roll_deg, unit="deg"),
                        _parameter("pitch_deg", "Pitch", "Initial geodetic pitch.", default=initial.pitch_deg, unit="deg"),
                        _parameter("yaw_deg", "Yaw", "Initial geodetic yaw.", default=initial.yaw_deg, unit="deg"),
                        _parameter("alpha_deg", "Angle of attack", "Initial angle of attack.", default=initial.alpha_deg, unit="deg"),
                        _parameter("beta_deg", "Sideslip", "Initial sideslip.", default=initial.beta_deg, unit="deg"),
                        _parameter(
                            "body_rates_deg_s",
                            "Body rates",
                            "Initial Earth-relative body rates.",
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
                    description="Inputs supplied where omitted source arc/LTG/glideslope guidance and flight controller modules would command surfaces or RCS.",
                    children=(
                        _parameter(
                            "aileron_command_deg",
                            "Aileron command",
                            "Direct aileron command entering physical left/right elevon mixing.",
                            default=0.0,
                            unit="deg",
                            role="segment",
                        ),
                        _parameter(
                            "elevator_command_deg",
                            "Elevator command",
                            "Direct elevator command entering physical left/right elevon mixing.",
                            default=0.0,
                            unit="deg",
                            role="segment",
                        ),
                        _parameter("rudder_command_deg", "Rudder command", "Direct physical rudder command.", default=0.0, unit="deg", role="segment"),
                        _parameter(
                            "thrust_vector_unit_body",
                            "Desired thrust direction",
                            "Body-axis unit direction consumed by aggregate RCS vector-control modes.",
                            default=(1.0, 0.0, 0.0),
                            value_type="vector3",
                            frame=_BODY_FRAME_ID,
                            role="segment",
                        ),
                        _parameter(
                            "roll_command_deg",
                            "RCS roll command",
                            "Aggregate-RCS roll-angle command.",
                            default=float(source_values.get("phibdcomx", 0.0)),
                            unit="deg",
                            role="segment",
                        ),
                        _parameter(
                            "pitch_command_deg",
                            "RCS pitch command",
                            "Aggregate-RCS pitch-angle command.",
                            default=float(source_values.get("thtbdcomx", 0.0)),
                            unit="deg",
                            role="segment",
                        ),
                        _parameter(
                            "yaw_command_deg",
                            "RCS yaw command",
                            "Aggregate-RCS yaw-angle command.",
                            default=float(source_values.get("psibdcomx", 0.0)),
                            unit="deg",
                            role="segment",
                        ),
                        _parameter(
                            "alpha_command_deg",
                            "Incidence command",
                            "Aggregate-RCS incidence command for source mode 3.",
                            default=0.0,
                            unit="deg",
                            role="segment",
                        ),
                        _parameter(
                            "beta_command_deg", "Sideslip command", "Aggregate-RCS sideslip command for source mode 3.", default=0.0, unit="deg", role="segment"
                        ),
                        _parameter(
                            "lateral_acceleration_command_g",
                            "Lateral acceleration command",
                            "Aggregate side-thruster lateral acceleration command.",
                            default=0.0,
                            unit="g",
                            role="segment",
                        ),
                        _parameter(
                            "normal_acceleration_command_g",
                            "Normal acceleration command",
                            "Aggregate side-thruster normal acceleration command.",
                            default=0.0,
                            unit="g",
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
                            "Enable direct insertion cutoff",
                            "Allow the bounded test/runtime seam to assert source beco_flag.",
                            default=False,
                            value_type="boolean",
                            role="constraint",
                        ),
                        _parameter(
                            "boost_cutoff_time_s",
                            "Insertion cutoff time",
                            "Time used to assert source beco_flag when enabled.",
                            default=max(1.0, min(source.end_time_s, 1150.0)),
                            unit="s",
                            role="constraint",
                        ),
                        _parameter(
                            "enable_terminal_lock",
                            "Enable direct terminal lock",
                            "Allow the bounded test/runtime seam to assert source mseek=4.",
                            default=False,
                            value_type="boolean",
                            role="constraint",
                        ),
                        _parameter(
                            "terminal_lock_time_s",
                            "Terminal lock time",
                            "Time used to assert source mseek=4 when enabled.",
                            default=max(1.0, min(source.end_time_s, 1782.0)),
                            unit="s",
                            role="constraint",
                        ),
                        _parameter(
                            "end_time_s", "End time", "Requested propagation horizon.", default=min(source.end_time_s, 20.0), unit="s", role="constraint"
                        ),
                        _parameter(
                            "sample_step_s",
                            "Sample cadence",
                            "Returned trajectory cadence.",
                            default=max(source.plot_step_s or 0.1, source.integration_step_s),
                            unit="s",
                            role="constraint",
                        ),
                        _parameter(
                            "random_seed",
                            "Radar random seed",
                            "Deterministic seed for source-shaped RADAR0 measurement corruption.",
                            default=12345,
                            value_type="integer",
                            role="variant",
                        ),
                    ),
                ),
            ),
        ),
        claim_boundary=(
            "The configuration selects one persistent HYPER6 truth object under a T4 run envelope. Atmospheric physical surfaces are T4; "
            "later aggregate-RCS phases are T3 and are reported per sample. GHAME6 has no TVC module."
        ),
    )


####


def _parameter(
    parameter_id: str,
    label: str,
    description: str,
    *,
    default: object,
    value_type: Ghame6ConfigurationValueType = "number",
    unit: str | None = None,
    frame: str | None = None,
    role: Ghame6ConfigurationRole = "initialization",
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
        compatible_fidelities=(GHAME6_FIDELITY_ID,),
        frame=frame,
        provenance="missiondesignsolutions/CADAC/GHAME6/input.asc",
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
    data_type: Ghame6OutputDataType = "float64",
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
        compatible_fidelities=(GHAME6_FIDELITY_ID,),
        compatible_realizations=(GHAME6_REALIZATION_ID,),
        compatible_mission_templates=(GHAME6_MISSION_ID,),
        operations=("batch",),
        provenance="CADAC GHAME6 source-grounded phase-aware mission",
        claim_boundary="Available from the Python reconstruction; compiled-CADAC numerical parity remains pending.",
    )


####


def _build_output_schema() -> TrajectoryOutputSchema:
    core = (
        _output("position_inertial_m", "Inertial position", "Earth-centered inertial HYPER6 position.", unit="m", shape=(3,), frame=_ECI_FRAME_ID),
        _output("velocity_inertial_mps", "Inertial velocity", "Earth-centered inertial HYPER6 velocity.", unit="m/s", shape=(3,), frame=_ECI_FRAME_ID),
        _output("quaternion_wxyz", "Attitude quaternion", "Scalar-first inertial-to-body quaternion.", shape=(4,), interpolation="slerp"),
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
        _output("altitude_m", "Altitude", "Altitude above the source ellipsoid.", unit="m"),
        _output("geographic_speed_mps", "Geographic speed", "Earth-relative speed.", unit="m/s"),
        _output("heading_deg", "Heading", "Geodetic heading.", unit="deg"),
        _output("flight_path_deg", "Flight-path angle", "Geodetic flight-path angle.", unit="deg"),
        _output("roll_deg", "Roll", "Geodetic roll.", unit="deg"),
        _output("pitch_deg", "Pitch", "Geodetic pitch.", unit="deg"),
        _output("yaw_deg", "Yaw", "Geodetic yaw.", unit="deg"),
        _output("alpha_deg", "Angle of attack", "Source angle of attack.", unit="deg"),
        _output("beta_deg", "Sideslip", "Source sideslip.", unit="deg"),
        _output("mach", "Mach", "Source atmosphere-relative Mach number."),
        _output("dynamic_pressure_pa", "Dynamic pressure", "Source dynamic pressure.", unit="Pa"),
        _output("source_phase", "Source phase", "Active source mission/control phase.", data_type="string", interpolation="step"),
        _output("runtime_fidelity", "Runtime fidelity", "Per-sample Taoryx fidelity boundary.", data_type="string", interpolation="step"),
        _output("control_realization", "Control realization", "Physical surfaces, aggregate RCS, or uncontrolled.", data_type="string", interpolation="step"),
        _output("aerodynamic_mode", "Aerodynamic mode", "Source maero value.", data_type="int64", interpolation="step"),
        _output("propulsion_mode", "Propulsion mode", "Source mprop value.", data_type="int64", interpolation="step"),
        _output("seeker_mode", "Seeker mode", "Source mseek value.", data_type="int64", interpolation="step"),
        _output("rcs_moment_mode", "RCS moment mode", "Source mrcs_moment value.", data_type="int64", interpolation="step"),
        _output("rcs_force_mode", "RCS force mode", "Source mrcs_force value.", data_type="int64", interpolation="step"),
        _output("mass_kg", "Mass", "Current source mass.", unit="kg"),
        _output("remaining_fuel_kg", "Remaining fuel", "Current source phase fuel remaining.", unit="kg"),
        _output("inertia_diagonal_kgm2", "Inertia diagonal", "Body inertia diagonal.", unit="kg*m^2", shape=(3,), frame=_BODY_FRAME_ID),
        _output("inertia_xz_kgm2", "Inertia x-z product", "Atmospheric GHAME source product of inertia.", unit="kg*m^2"),
        _output("throttle", "Throttle", "Current hypersonic propulsion throttle."),
        _output("thrust_n", "Thrust", "Current axial propulsion thrust.", unit="N"),
        _output("requested_control_deg", "Requested controls", "Aileron/elevator/rudder requests.", unit="deg", shape=(3,), frame=_BODY_FRAME_ID),
        _output(
            "requested_surfaces_deg", "Requested surfaces", "Requested left elevon/right elevon/rudder positions.", unit="deg", shape=(3,), frame=_BODY_FRAME_ID
        ),
        _output(
            "achieved_surfaces_deg",
            "Achieved surfaces",
            "Physical achieved left elevon/right elevon/rudder positions.",
            unit="deg",
            shape=(3,),
            frame=_BODY_FRAME_ID,
        ),
        _output(
            "achieved_control_deg", "Achieved controls", "Unmixed achieved aileron/elevator/rudder coordinates.", unit="deg", shape=(3,), frame=_BODY_FRAME_ID
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
            "requested_rcs_incidence_deg",
            "Requested RCS incidence",
            "Requested alpha/beta coordinates entering aggregate RCS.",
            unit="deg",
            shape=(2,),
            frame=_BODY_FRAME_ID,
        ),
        _output(
            "requested_rcs_acceleration_g",
            "Requested RCS acceleration",
            "Requested lateral/normal acceleration coordinates entering aggregate RCS.",
            unit="g",
            shape=(2,),
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
        model_id=GHAME6_MODEL_ID,
        model_version=GHAME6_MODEL_VERSION,
        core_channels=core,
        telemetry_channels=telemetry,
        telemetry_groups=(
            TrajectoryTelemetryGroupMetadata(
                id="phase_aware_mission",
                label="Phase-aware mission",
                description="WGS84 truth, physical atmospheric surfaces, aggregate RCS, propulsion, seeker mode, and total wrench.",
                channel_ids=tuple(item.id for item in telemetry),
            ),
        ),
        entity_output=TrajectoryEntityOutputMetadata(supports_multiple_entities=True),
        claim_boundary="HYPER6 is primary; SAT3 and RADAR0 are separate source-root participants returned by the batch result.",
    )


####


def _build_model_metadata(
    schema: TrajectoryConfigurationSchema,
    output_schema: TrajectoryOutputSchema,
) -> TrajectoryModelMetadata:
    fidelity = TrajectoryFidelityMetadata(
        id=GHAME6_FIDELITY_ID,
        label="CADAC GHAME6 physical-surface-to-RCS 6-DoF envelope",
        rank=3,
        declared=True,
        dynamics_fidelity="rigid_body_6dof",
        input_realization="actuator_allocated",
        actuator_types=("mixed",),
        compatibility_aliases=("rigid_body_6dof_effector_allocated",),
        runtime_fidelity="phase_reported_rigid_body_6dof",
        control_realization="atmospheric_surfaces_to_axis_aggregate_rcs",
        promotion_status="development",
        operations=("validate", "batch"),
        profile_id="cadac.ghame6.source-phase-program",
        blockers=(
            "compiled CADAC numerical golden parity is not yet registered",
            "full arc/LTG/glideslope command generation and GPS/INS/star-tracker/RF-EKF estimation are excluded",
        ),
        required_operations=("WGS84 rigid-body propagation", "physical atmospheric surfaces", "aggregate RCS", "SAT3 truth", "RADAR0 measurements"),
        claim_boundary=(
            "T4 is the run envelope because physical surfaces participate before release. Transfer/interceptor RCS remains axis-aggregate T3; "
            "GHAME6 has no TVC source module and is never advertised as TVC-controlled."
        ),
    )
    realization = TrajectoryRealizationMetadata(
        id=GHAME6_REALIZATION_ID,
        label="CADAC GHAME6 source phase program",
        description="Atmospheric three-surface HYPER6 followed by aggregate-RCS transfer/interceptor phases with SAT3 and RADAR0.",
        status="available",
        dynamics_fidelities=("rigid_body_6dof",),
        input_realization="actuator_allocated",
        actuator_types=("mixed",),
        controls=batch_configuration_control_advertisement(
            schema,
            channels={
                "actuator.aileron.deflection": ("commands", "aileron_command_deg"),
                "actuator.elevator.deflection": ("commands", "elevator_command_deg"),
                "actuator.rudder.deflection": ("commands", "rudder_command_deg"),
                "rcs.thrust_vector.direction": ("commands", "thrust_vector_unit_body"),
                "rcs.roll.attitude_command": ("commands", "roll_command_deg"),
                "rcs.pitch.attitude_command": ("commands", "pitch_command_deg"),
                "rcs.yaw.attitude_command": ("commands", "yaw_command_deg"),
                "rcs.alpha.command": ("commands", "alpha_command_deg"),
                "rcs.beta.command": ("commands", "beta_command_deg"),
                "rcs.lateral_acceleration.command": ("commands", "lateral_acceleration_command_g"),
                "rcs.normal_acceleration.command": ("commands", "normal_acceleration_command_g"),
            },
            output_evidence={
                "actuator.aileron.deflection": CadacControlOutputEvidence("requested_control_deg", ("achieved_control_deg",), 0, (0,)),
                "actuator.elevator.deflection": CadacControlOutputEvidence("requested_control_deg", ("achieved_control_deg",), 1, (1,)),
                "actuator.rudder.deflection": CadacControlOutputEvidence("requested_control_deg", ("achieved_control_deg",), 2, (2,)),
                "rcs.thrust_vector.direction": CadacControlOutputEvidence(
                    "requested_thrust_vector_unit_body",
                    ("rcs_force_body_n", "rcs_moment_body_nm"),
                ),
                "rcs.roll.attitude_command": CadacControlOutputEvidence("requested_rcs_attitude_deg", ("rcs_moment_body_nm",), 0, (0,)),
                "rcs.pitch.attitude_command": CadacControlOutputEvidence("requested_rcs_attitude_deg", ("rcs_moment_body_nm",), 1, (1,)),
                "rcs.yaw.attitude_command": CadacControlOutputEvidence("requested_rcs_attitude_deg", ("rcs_moment_body_nm",), 2, (2,)),
                "rcs.alpha.command": CadacControlOutputEvidence("requested_rcs_incidence_deg", ("alpha_deg",), 0),
                "rcs.beta.command": CadacControlOutputEvidence("requested_rcs_incidence_deg", ("beta_deg",), 1),
                "rcs.lateral_acceleration.command": CadacControlOutputEvidence("requested_rcs_acceleration_g", ("rcs_force_body_n",), 0, (1,)),
                "rcs.normal_acceleration.command": CadacControlOutputEvidence("requested_rcs_acceleration_g", ("rcs_force_body_n",), 1, (2,)),
            },
            authority_id="direct_surface_and_rcs_commands",
            authority="native_bridge",
            authority_description="Caller-owned atmospheric-surface and aggregate-RCS command coordinates fixed for one GHAME6 batch.",
            intent_id="direct_phase_control",
            intent_label="Direct Phase Control",
            intent_description="Set the reconstructed GHAME6 direct controls at the omitted source GNC boundary before batch execution.",
            mission_ids=(GHAME6_MISSION_ID,),
            source_refs=("missiondesignsolutions/CADAC/GHAME6",),
            claim_boundary="The caller supplies fixed direct control coordinates at the reconstructed GHAME6 GNC seam. Source guidance/estimation and common-runner step control remain unavailable.",
        ),
        fidelity_aliases=(GHAME6_FIDELITY_ID, "rigid_body_6dof_effector_allocated"),
        mission_template_ids=(GHAME6_MISSION_ID,),
        operations=("validate", "batch"),
        native_factory_ids=(GHAME6_EXECUTOR_ID,),
        source_refs=("missiondesignsolutions/CADAC/GHAME6",),
        blockers=("omitted source guidance/estimation modules enter through explicit direct command/truth boundaries",),
        claim_boundary="Available as a phase-aware physical plant and multi-actor truth composition, not as a complete source GNC/estimator replay.",
    )
    mission = TrajectoryMissionTemplateMetadata(
        id=GHAME6_MISSION_ID,
        name="GHAME6 atmospheric-to-exo satellite intercept",
        description="Execute the source event sequence across atmospheric flight, transfer-vehicle insertion, interceptor glideslope, and terminal RCS phases.",
        status="development",
        initialization_variants=("source_automated_or_geodetic",),
        segment_sequence=(
            "atmospheric_surfaces",
            "transfer_angle_rcs",
            "transfer_vector_rcs",
            "interceptor_glideslope_rcs",
            "interceptor_terminal_rcs",
        ),
        compatible_fidelities=(GHAME6_FIDELITY_ID,),
        operations=(
            TrajectoryMissionOperationMetadata(
                fidelity=GHAME6_FIDELITY_ID,
                realization_id=GHAME6_REALIZATION_ID,
                operation="validate",
                status="available",
                execution_mode="source_grounded_validation",
                common_runner_status="not_available",
                claim_boundary="Validation prepares one exact GHAME6 actor/event/deck composition.",
            ),
            TrajectoryMissionOperationMetadata(
                fidelity=GHAME6_FIDELITY_ID,
                realization_id=GHAME6_REALIZATION_ID,
                operation="batch",
                status="available",
                execution_mode="source_ordered_phase_aware_multi_actor",
                availability_scope="native_runtime_binding",
                common_runner_status="registered",
                executor_id=GHAME6_EXECUTOR_ID,
                claim_boundary="Batch execution returns HYPER6, SAT3, RADAR0 state/measurement evidence with per-sample phase fidelity.",
            ),
        ),
        provenance="missiondesignsolutions/CADAC/GHAME6/input.asc",
        claim_boundary="The source replaces the persistent HYPER6 mass/control realization by event; discarded carriers are not propagated as invented child entities.",
    )
    return TrajectoryModelMetadata(
        id=GHAME6_MODEL_ID,
        name="CADAC GHAME6 hypersonic vehicle",
        version=GHAME6_MODEL_VERSION,
        description="Phase-aware WGS84 hypersonic atmospheric vehicle, transfer vehicle, and exo interceptor against SAT3 with RADAR0 tracking.",
        presentation=TrajectoryModelPresentationMetadata(
            display_name="CADAC GHAME6",
            short_name="GHAME6",
            summary="Atmospheric physical surfaces transition to aggregate-RCS exo phases; SAT3 and RADAR0 remain separate actors.",
            category="aerospace",
            subcategory="hypersonic_space_interceptor",
            sort_key="cadac-ghame6",
            badges=("T4→T3", "multi-actor", "native-sensor", "source-events", "no-TVC"),
            default_fidelity_id=GHAME6_FIDELITY_ID,
            default_mission_template_id=GHAME6_MISSION_ID,
            default_output_channel_ids=("position_inertial_m", "velocity_inertial_mps", "source_phase", "runtime_fidelity"),
            properties=(
                TrajectoryModelPropertyMetadata(
                    id="source_model",
                    label="Source model",
                    description="Upstream CADAC primary actor identifier.",
                    semantic_role="identity",
                    value_type="string",
                    value_kind="declared",
                    value="HYPER6",
                    value_declared=True,
                    source_refs=("missiondesignsolutions/CADAC/GHAME6",),
                    provenance="CADAC GHAME6 source package",
                    claim_boundary="Identity metadata only.",
                ),
            ),
        ),
        family_id=GHAME6_MODEL_ID,
        physical_family="hypersonic_atmospheric_to_exo_interceptor",
        model_kind="mission_composition",
        status="development",
        tags=(
            "cadac",
            "hypersonic",
            "rigid_body_6dof",
            "physical_surfaces",
            "aggregate_rcs",
            "satellite",
            "radar",
            "native_relative_state",
        ),
        execution_capability_profile="taoryx_universal",
        operations=("discover", "validate", "batch"),
        common_runner_operations=("batch",),
        capabilities=TrajectoryModelCapabilities(
            initialization_modes=("source_automated_or_geodetic",),
            segment_types=(
                "atmospheric_surfaces",
                "transfer_angle_rcs",
                "transfer_vector_rcs",
                "interceptor_glideslope_rcs",
                "interceptor_terminal_rcs",
            ),
            termination_modes=("end_time", "nonfinite_state"),
            operations=("discover", "validate", "batch"),
            supports_custom_segments=False,
            supports_deployment=True,
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
                description="Earth-centered inertial frame used by HYPER6, SAT3, and RADAR0 truth.",
                frame_kind="inertial",
                axes=("x", "y", "z"),
                handedness="right",
                origin="Earth center",
                orientation="source inertial axes with Greenwich celestial longitude zero at simulation start",
                source_refs=("missiondesignsolutions/CADAC/GHAME6",),
                provenance="CADAC GHAME6 convention",
            ),
            TrajectoryReferenceFrameMetadata(
                id=_BODY_FRAME_ID,
                name="CADAC GHAME6 body axes",
                description="Body-fixed axes used by forces, moments, surfaces, rates, and RCS.",
                frame_kind="body",
                axes=("x", "y", "z"),
                handedness="right",
                origin="instantaneous vehicle reference point",
                orientation="source GHAME6 body convention",
                source_refs=("missiondesignsolutions/CADAC/GHAME6",),
                provenance="CADAC GHAME6 body convention",
            ),
        ),
        output_schema=output_schema,
        output_schema_id=output_schema.schema_id,
        output_schema_fingerprint=output_schema.fingerprint,
        configuration_schema_id=schema.schema_id,
        configuration_schema_fingerprint=schema.fingerprint,
        fidelities=(fidelity,),
        fidelity_transitions=(),
        source_refs=("missiondesignsolutions/CADAC/GHAME6",),
        provenance="CADAC GHAME6 source-grounded Python phase-aware multi-actor reconstruction",
        claim_boundary="Executable at development status; full source GNC/estimation and compiled parity remain separate promotion gates.",
    )


####


def _overrides_from_resolved(resolved: Any) -> Ghame6PluginOverrides:
    if not isinstance(resolved, Mapping):
        raise ValueError("GHAME6 resolved configuration root must be a mapping")
    ####
    initial = _group(resolved, "initialization")
    commands = _group(resolved, "commands")
    runtime = _group(resolved, "runtime")
    return Ghame6PluginOverrides(
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
        aileron_command_deg=float(commands["aileron_command_deg"]),
        elevator_command_deg=float(commands["elevator_command_deg"]),
        rudder_command_deg=float(commands["rudder_command_deg"]),
        thrust_vector_unit_body=_vector3(commands["thrust_vector_unit_body"]),
        roll_command_deg=float(commands["roll_command_deg"]),
        pitch_command_deg=float(commands["pitch_command_deg"]),
        yaw_command_deg=float(commands["yaw_command_deg"]),
        alpha_command_deg=float(commands["alpha_command_deg"]),
        beta_command_deg=float(commands["beta_command_deg"]),
        lateral_acceleration_command_g=float(commands["lateral_acceleration_command_g"]),
        normal_acceleration_command_g=float(commands["normal_acceleration_command_g"]),
        boost_cutoff_time_s=float(runtime["boost_cutoff_time_s"]) if bool(runtime["enable_boost_cutoff"]) else None,
        terminal_lock_time_s=float(runtime["terminal_lock_time_s"]) if bool(runtime["enable_terminal_lock"]) else None,
        end_time_s=float(runtime["end_time_s"]),
        sample_step_s=float(runtime["sample_step_s"]),
        random_seed=int(runtime["random_seed"]),
    )


####


def _group(root: Mapping[str, object], name: str) -> Mapping[str, object]:
    value = root.get(name)
    if not isinstance(value, Mapping):
        raise ValueError(f"GHAME6 resolved configuration requires group {name!r}")
    ####
    return value


####


def _vector3(value: object) -> tuple[float, float, float]:
    if not isinstance(value, (tuple, list)) or len(value) != 3:
        raise ValueError("GHAME6 vector configuration requires exactly three components")
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
        "aileron_command_deg",
        "elevator_command_deg",
        "rudder_command_deg",
        "roll_command_deg",
        "pitch_command_deg",
        "yaw_command_deg",
        "alpha_command_deg",
        "beta_command_deg",
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
    if parameter_id in {"lateral_acceleration_command_g", "normal_acceleration_command_g"}:
        return "g"
    ####
    if parameter_id in {"boost_cutoff_time_s", "terminal_lock_time_s", "end_time_s", "sample_step_s"}:
        return "s"
    ####
    return None


####


def _project_run_result(
    request: MissionCompositionRunRequest,
    run: Ghame6RunResult,
    output_schema: TrajectoryOutputSchema,
) -> MissionCompositionTrajectoryResult:
    selected = resolve_output_selection(
        output_schema,
        request.output,
        fidelity=GHAME6_FIDELITY_ID,
        operation="batch",
        realization_id=GHAME6_REALIZATION_ID,
        mission_template_id=GHAME6_MISSION_ID,
    )
    hyper_channels = tuple(_runtime_channel(item, output_schema) for item in selected)
    hyper_samples = tuple(
        TrajectorySample(time_s=sample.time_s, values={channel.id: _hyper_value(channel.id, sample) for channel in selected}) for sample in run.samples
    )
    satellite_channels = _satellite_channels(request.output.mode)
    satellite_samples = tuple(
        TrajectorySample(
            time_s=sample.time_s,
            values={
                "position_inertial_m": sample.position_inertial_m,
                "velocity_inertial_mps": sample.velocity_inertial_mps,
                **(
                    {
                        "longitude_deg": sample.longitude_deg,
                        "latitude_deg": sample.latitude_deg,
                        "altitude_m": sample.altitude_m,
                        "speed_mps": sample.speed_mps,
                    }
                    if request.output.mode != "core"
                    else {}
                ),
            },
        )
        for sample in run.satellite_samples
    )
    radar_channels = _radar_channels(request.output.mode)
    radar_samples = tuple(
        TrajectorySample(
            time_s=sample.time_s,
            values={
                "position_inertial_m": sample.position_inertial_m,
                "velocity_inertial_mps": sample.velocity_inertial_mps,
                **(
                    {
                        "longitude_deg": sample.longitude_deg,
                        "latitude_deg": sample.latitude_deg,
                        "altitude_m": sample.altitude_m,
                    }
                    if request.output.mode != "core"
                    else {}
                ),
            },
        )
        for sample in run.radar_samples
    )
    failed = run.terminated_reason != "end_time"
    hyper_object = TrajectoryObject(
        object_id="ghame6-hyper-1",
        model_id=GHAME6_MODEL_ID,
        realization_id=GHAME6_REALIZATION_ID,
        name="CADAC GHAME6 HYPER6",
        role="hypersonic_vehicle_transfer_interceptor",
        fidelity=GHAME6_FIDELITY_ID,
        status="failed" if failed else "completed",
        active_from_s=hyper_samples[0].time_s,
        active_to_s=hyper_samples[-1].time_s,
        terminal_disposition=run.terminated_reason,
        channels=hyper_channels,
        samples=hyper_samples,
        provenance="CADAC GHAME6 source-grounded phase-aware HYPER6 runtime",
        claim_boundary="T4 is the run envelope; every sample reports whether physical surfaces or aggregate-RCS T3 actually participates.",
    )
    satellite_object = TrajectoryObject(
        object_id="ghame6-satellite-1",
        model_id=GHAME6_SATELLITE_MODEL_ID,
        realization_id=GHAME6_REALIZATION_ID,
        name="CADAC GHAME6 SAT3",
        role="target_satellite",
        fidelity="point_mass_3dof",
        status="completed",
        active_from_s=satellite_samples[0].time_s,
        active_to_s=satellite_samples[-1].time_s,
        terminal_disposition="source_run_end",
        channels=satellite_channels,
        samples=satellite_samples,
        provenance="CADAC GHAME6 SAT3 orbital-element truth propagation",
        claim_boundary="SAT3 is an independent root object and is not inferred as a deployed child of HYPER6.",
    )
    radar_object = TrajectoryObject(
        object_id="ghame6-radar-1",
        model_id=GHAME6_RADAR_MODEL_ID,
        realization_id=GHAME6_REALIZATION_ID,
        name="CADAC GHAME6 RADAR0",
        role="fixed_ground_radar",
        fidelity="static",
        status="completed",
        active_from_s=radar_samples[0].time_s,
        active_to_s=radar_samples[-1].time_s,
        terminal_disposition="source_run_end",
        channels=radar_channels,
        samples=radar_samples,
        provenance="CADAC GHAME6 RADAR0 rotating-Earth fixed-site kinematics",
        claim_boundary="RADAR0 is a static source actor with inertial state induced only by Earth rotation; it is not a standalone trajectory plug-in.",
    )
    events: list[TrajectoryEvent] = []
    if request.output.include_events:
        events.extend(
            TrajectoryEvent(
                id=f"ghame6-source-event-{event.event_index}",
                time_s=event.time_s,
                category="deployment" if event.event_index in {1, 3} else "segment",
                kind=f"source_event_{event.event_index}",
                object_id="ghame6-hyper-1",
                detail=f"{event.phase_before} -> {event.phase_after}",
                data=event.model_dump(mode="json"),
            )
            for event in run.events
        )
        for track in run.radar_tracks:
            source_track = track.model_dump(mode="json")
            native_packet = source_track.pop("native_relative_state_packet")
            events.append(
                TrajectoryEvent(
                    id=f"ghame6-radar-track-{track.update_sequence}",
                    time_s=track.time_s,
                    category="custom",
                    kind="radar_track_update",
                    object_id="ghame6-radar-1",
                    detail="RADAR0 measured SAT3 and refreshed its source track file.",
                    data=source_track,
                )
            )
            events.append(
                TrajectoryEvent(
                    id=f"ghame6-native-relative-state-{track.update_sequence}",
                    time_s=track.time_s,
                    category="custom",
                    kind="native_relative_state_track",
                    object_id="ghame6-radar-1",
                    detail="Standard raw relative-state packet projected from the committed RADAR0/SAT3 geometry.",
                    data=native_packet,
                )
            )
        ####
    ####
    diagnostics = (
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-parity-pending",
            message="GHAME6 executes through the source-grounded Python reconstruction; compiled-CADAC numerical parity is not yet promoted.",
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=GHAME6_MODEL_ID,
            object_id="ghame6-hyper-1",
        ),
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-ghame6-no-tvc",
            message="The shipped GHAME6 source mission has no TVC module. Atmospheric control is physical left/right elevons plus rudder; exo control is axis-aggregate RCS.",
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=GHAME6_MODEL_ID,
            object_id="ghame6-hyper-1",
        ),
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-direct-command-estimation-boundary",
            message="Full arc/LTG/glideslope command generation and GPS/INS/star-tracker/RF-EKF estimation are excluded; direct commands and truth geometry enter at declared boundaries.",
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=GHAME6_MODEL_ID,
            object_id="ghame6-hyper-1",
        ),
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-ghame6-native-radar-track",
            message=(
                "Every RADAR0 update also emits a Taoryx relative-state-track packet from committed RADAR0/SAT3 geometry. "
                "The source-shaped noisy RADAR0 track remains a separate event, and this batch-only package does not claim SensorBus delivery."
            ),
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=GHAME6_MODEL_ID,
            object_id="ghame6-radar-1",
        ),
    )
    return MissionCompositionTrajectoryResult(
        provider_id=CADAC_PROVIDER_ID,
        provider_version=GHAME6_MODEL_VERSION,
        request_id=request.request_id,
        configuration_fingerprint=request.prepared_configuration.fingerprint,
        primary_model_id=GHAME6_MODEL_ID,
        primary_object_id="ghame6-hyper-1",
        status="failed" if failed else "completed",
        objects=(hyper_object, satellite_object, radar_object),
        events=tuple(sorted(events, key=lambda item: (item.time_s, item.id))),
        relationships=(),
        diagnostics=diagnostics,
        claim_boundary="HYPER6, SAT3, and RADAR0 remain independent source-root actors; source release events change the persistent HYPER6 realization without inventing lineage.",
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


def _satellite_channels(mode: str) -> tuple[TrajectoryChannelMetadata, ...]:
    core = (
        TrajectoryChannelMetadata(
            id="position_inertial_m", channel_class="core_state", unit="m", shape=(3,), frame=_ECI_FRAME_ID, description="SAT3 inertial position."
        ),
        TrajectoryChannelMetadata(
            id="velocity_inertial_mps", channel_class="core_state", unit="m/s", shape=(3,), frame=_ECI_FRAME_ID, description="SAT3 inertial velocity."
        ),
    )
    if mode == "core":
        return core
    ####
    return (
        *core,
        TrajectoryChannelMetadata(
            id="longitude_deg", channel_class="telemetry", telemetry_group="satellite_truth", unit="deg", description="SAT3 geodetic longitude."
        ),
        TrajectoryChannelMetadata(
            id="latitude_deg", channel_class="telemetry", telemetry_group="satellite_truth", unit="deg", description="SAT3 geodetic latitude."
        ),
        TrajectoryChannelMetadata(id="altitude_m", channel_class="telemetry", telemetry_group="satellite_truth", unit="m", description="SAT3 altitude."),
        TrajectoryChannelMetadata(id="speed_mps", channel_class="telemetry", telemetry_group="satellite_truth", unit="m/s", description="SAT3 inertial speed."),
    )


####


def _radar_channels(mode: str) -> tuple[TrajectoryChannelMetadata, ...]:
    core = (
        TrajectoryChannelMetadata(
            id="position_inertial_m",
            channel_class="core_state",
            unit="m",
            shape=(3,),
            frame=_ECI_FRAME_ID,
            description="RADAR0 inertial position from fixed-Earth kinematics.",
        ),
        TrajectoryChannelMetadata(
            id="velocity_inertial_mps",
            channel_class="core_state",
            unit="m/s",
            shape=(3,),
            frame=_ECI_FRAME_ID,
            description="RADAR0 inertial velocity from Earth rotation.",
        ),
    )
    if mode == "core":
        return core
    ####
    return (
        *core,
        TrajectoryChannelMetadata(
            id="longitude_deg", channel_class="telemetry", telemetry_group="radar_site", unit="deg", description="Fixed radar longitude."
        ),
        TrajectoryChannelMetadata(id="latitude_deg", channel_class="telemetry", telemetry_group="radar_site", unit="deg", description="Fixed radar latitude."),
        TrajectoryChannelMetadata(id="altitude_m", channel_class="telemetry", telemetry_group="radar_site", unit="m", description="Fixed radar altitude."),
    )


####


def _hyper_value(channel_id: str, sample: Ghame6HyperSample) -> object:
    values: dict[str, object] = {
        "position_inertial_m": sample.position_inertial_m,
        "velocity_inertial_mps": sample.velocity_inertial_mps,
        "quaternion_wxyz": sample.quaternion_wxyz,
        "body_rates_inertial_rad_s": sample.body_rates_inertial_rad_s,
        "body_rates_earth_rad_s": sample.body_rates_earth_rad_s,
        "longitude_deg": sample.longitude_deg,
        "latitude_deg": sample.latitude_deg,
        "altitude_m": sample.altitude_m,
        "geographic_speed_mps": sample.geographic_speed_mps,
        "heading_deg": sample.heading_deg,
        "flight_path_deg": sample.flight_path_deg,
        "roll_deg": sample.roll_deg,
        "pitch_deg": sample.pitch_deg,
        "yaw_deg": sample.yaw_deg,
        "alpha_deg": sample.alpha_deg,
        "beta_deg": sample.beta_deg,
        "mach": sample.mach,
        "dynamic_pressure_pa": sample.dynamic_pressure_pa,
        "source_phase": sample.source_phase,
        "runtime_fidelity": sample.runtime_fidelity,
        "control_realization": sample.control_realization,
        "aerodynamic_mode": sample.aerodynamic_mode,
        "propulsion_mode": sample.propulsion_mode,
        "seeker_mode": sample.seeker_mode,
        "rcs_moment_mode": sample.rcs_moment_mode,
        "rcs_force_mode": sample.rcs_force_mode,
        "mass_kg": sample.mass_kg,
        "remaining_fuel_kg": sample.remaining_fuel_kg,
        "inertia_diagonal_kgm2": sample.inertia_diagonal_kgm2,
        "inertia_xz_kgm2": sample.inertia_xz_kgm2,
        "throttle": sample.throttle,
        "thrust_n": sample.thrust_n,
        "requested_control_deg": sample.requested_control_deg,
        "requested_surfaces_deg": sample.requested_surfaces_deg,
        "achieved_surfaces_deg": sample.achieved_surfaces_deg,
        "achieved_control_deg": sample.achieved_control_deg,
        "requested_rcs_attitude_deg": sample.requested_rcs_attitude_deg,
        "requested_rcs_incidence_deg": sample.requested_rcs_incidence_deg,
        "requested_rcs_acceleration_g": sample.requested_rcs_acceleration_g,
        "requested_thrust_vector_unit_body": sample.requested_thrust_vector_unit_body,
        "rcs_force_body_n": sample.rcs_force_body_n,
        "rcs_moment_body_nm": sample.rcs_moment_body_nm,
        "force_body_n": sample.force_body_n,
        "moment_body_nm": sample.moment_body_nm,
    }
    return values[channel_id]


####


__all__ = [
    "GHAME6_EXECUTOR_ID",
    "GHAME6_FIDELITY_ID",
    "GHAME6_MISSION_ID",
    "GHAME6_MODEL_ID",
    "GHAME6_MODEL_VERSION",
    "GHAME6_RADAR_MODEL_ID",
    "GHAME6_REALIZATION_ID",
    "GHAME6_SATELLITE_MODEL_ID",
    "CadacGhame6MissionCompositionProvider",
    "build_default_ghame6_configuration",
    "register_ghame6_mission_composition",
]
