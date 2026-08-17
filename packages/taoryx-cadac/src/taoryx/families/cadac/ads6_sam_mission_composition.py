"""Taoryx Mission Composition bridge for the standalone ADS6 SAM plug-in."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Literal, TypeAlias

from taoryx.trajectory.configuration_contract import (
    ConfigurationBound,
    ConfigurationGroupSchema,
    ConfigurationGroupValue,
    ConfigurationInterval,
    ConfigurationParameterSchema,
    ConfigurationParameterValue,
    PreparedTrajectoryConfiguration,
    TrajectoryConfigurationInstance,
    TrajectoryConfigurationSchema,
    TrajectoryControlAdvertisement,
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
    TrajectoryProviderMetadata,
    TrajectoryProviderPresentationMetadata,
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
    TrajectoryObject,
    TrajectorySample,
    resolve_output_selection,
)

from .ads6_sam import Ads6SamRunResult, Ads6SamSample
from .ads6_sam_plugin import Ads6SamPluginOverrides, Ads6SamVehiclePlugin
from .aim5_mission_composition import CADAC_PROVIDER_ID
from .configuration_defaults import materialize_configuration_defaults
from .control_metadata import CadacControlOutputEvidence, batch_configuration_control_advertisement
from .output_metadata import cadac_output_quantity

Ads6SamConfigurationValueType: TypeAlias = Literal["number", "integer", "boolean", "string", "enum", "vector3", "vector4"]
Ads6SamConfigurationRole: TypeAlias = Literal["initialization", "segment", "constraint", "variant", "output"]
Ads6SamOutputDataType: TypeAlias = Literal["float64", "int64", "boolean", "string", "json"]
Ads6SamInterpolation: TypeAlias = Literal["linear", "step", "periodic", "slerp", "event"]
Ads6SamInputRealization: TypeAlias = Literal["uncontrolled", "guidance_command", "direct_wrench", "actuator_allocated", "source_replay", "provider_defined"]
Ads6SamActuatorType: TypeAlias = Literal[
    "not_applicable",
    "aerodynamic_surfaces",
    "rotors",
    "thrust_vectoring",
    "gimbals",
    "rcs",
    "mixed",
    "provider_defined",
]

ADS6_SAM_MODEL_ID = "cadac.ads6.sam"
ADS6_SAM_MODEL_VERSION = "0.9.0"
ADS6_SAM_FIN_PHASE_ID = "fin_control"
ADS6_SAM_TVC_PHASE_ID = "tvc_control"
ADS6_SAM_RCS_PHASE_ID = "aggregate_rcs"
ADS6_SAM_T4_FIDELITY_ID = "rigid_body_6dof_surface_allocated"
ADS6_SAM_T3_FIDELITY_ID = "rigid_body_6dof_direct_wrench"
ADS6_SAM_FIN_REALIZATION_ID = "cadac-ads6-cross-fin"
ADS6_SAM_TVC_REALIZATION_ID = "cadac-ads6-physical-tvc"
ADS6_SAM_RCS_REALIZATION_ID = "cadac-ads6-axis-aggregate-rcs"
ADS6_SAM_FIN_MISSION_ID = "ads6_sam_direct_fin_plant"
ADS6_SAM_TVC_MISSION_ID = "ads6_sam_direct_tvc_plant"
ADS6_SAM_RCS_MISSION_ID = "ads6_sam_direct_rcs_plant"
ADS6_SAM_EXECUTOR_ID = "cadac.ads6.sam.multi_realization.batch"
_LOCAL_NED_FRAME_ID = "cadac.local_ned"
_BODY_FRAME_ID = "cadac.body_xyz"

_PHASE_IDS = (ADS6_SAM_FIN_PHASE_ID, ADS6_SAM_TVC_PHASE_ID, ADS6_SAM_RCS_PHASE_ID)
_REALIZATION_IDS = (
    ADS6_SAM_FIN_REALIZATION_ID,
    ADS6_SAM_TVC_REALIZATION_ID,
    ADS6_SAM_RCS_REALIZATION_ID,
)
_MISSION_IDS = (ADS6_SAM_FIN_MISSION_ID, ADS6_SAM_TVC_MISSION_ID, ADS6_SAM_RCS_MISSION_ID)
_FIDELITY_IDS = (ADS6_SAM_T4_FIDELITY_ID, ADS6_SAM_T3_FIDELITY_ID)


class CadacAds6SamMissionCompositionProvider:
    """Self-describing provider for one installed standalone ADS6 SAM plant."""

    def __init__(self, plugin: Ads6SamVehiclePlugin) -> None:
        blockers = plugin.validate_installation()
        if blockers:
            raise ValueError("cannot publish ADS6 SAM provider with an incomplete installation: " + "; ".join(blockers))
        ####
        self._plugin = plugin
        self._schema = _build_configuration_schema(plugin)
        self._output_schema = _build_output_schema()
        self._model = _build_model_metadata(self._schema, self._output_schema)
        self._metadata = TrajectoryProviderMetadata(
            id=CADAC_PROVIDER_ID,
            name="CADAC vehicle plug-ins",
            version=ADS6_SAM_MODEL_VERSION,
            description="Standalone ADS6 SAM physical-control realizations exposed through Taoryx Mission Composition.",
            presentation=TrajectoryProviderPresentationMetadata(
                display_name="CADAC Vehicle Plug-ins",
                short_name="CADAC",
                summary="CADAC source actors exposed through typed Taoryx vehicle plug-in contracts.",
                organization="Taoryx integration layer",
                categories=("aerospace", "reference-models", "vehicle-plugins"),
            ),
            status="development",
            tags=("cadac", "ads6", "sam", "multi-realization"),
            execution_contract=ADS6_SAM_EXECUTOR_ID,
            model_count=1,
            provenance="missiondesignsolutions/CADAC ADS6 SAM source model plus Taoryx physical-plant reconstruction",
            claim_boundary=(
                "Cross-fin, physical-TVC, and aggregate-RCS realizations are exact selectable vehicle boundaries. "
                "ADS6 engagement actors, source GNC, and compiled-CADAC numerical parity remain separate gates."
            ),
        )

    ####

    @property
    def metadata(self) -> TrajectoryProviderMetadata:
        """Return provider identity and ADS6 SAM execution claim boundary."""

        return self._metadata

    ####

    def list_models(self) -> tuple[TrajectoryModelMetadata, ...]:
        return (self._model,)

    ####

    def get_model_schema(self, model_id: str) -> TrajectoryConfigurationSchema:
        if model_id != ADS6_SAM_MODEL_ID:
            raise KeyError(f"unknown ADS6 SAM model {model_id!r}")
        ####
        return self._schema

    ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        if model_id != ADS6_SAM_MODEL_ID:
            raise KeyError(f"unknown ADS6 SAM model {model_id!r}")
        ####
        return self._output_schema

    ####

    def validate_configuration(self, configuration: TrajectoryConfigurationInstance) -> PreparedTrajectoryConfiguration:
        configuration = materialize_configuration_defaults(self._schema, configuration)
        prepared = validate_configuration_instance(self._schema, configuration)
        if not isinstance(prepared.resolved, Mapping):
            raise ValueError("ADS6 SAM resolved configuration root must be a mapping")
        ####
        phase = prepared.resolved.get("source_phase")
        if not isinstance(phase, str) or phase not in _PHASE_IDS:
            raise ValueError(f"unsupported ADS6 SAM source phase {phase!r}")
        ####
        expected_fidelity, expected_realization, expected_mission = _phase_contract(phase)
        if configuration.fidelity != expected_fidelity:
            raise ValueError(f"ADS6 SAM phase {phase!r} requires fidelity {expected_fidelity!r}, not {configuration.fidelity!r}")
        ####
        if configuration.realization_id not in {None, expected_realization}:
            raise ValueError(f"ADS6 SAM phase {phase!r} requires realization {expected_realization!r}, not {configuration.realization_id!r}")
        ####
        if configuration.mission_template_id not in {None, expected_mission}:
            raise ValueError(f"ADS6 SAM phase {phase!r} requires mission {expected_mission!r}, not {configuration.mission_template_id!r}")
        ####
        overrides = _overrides_from_resolved(prepared.resolved)
        blockers = self._plugin.validate_realization(overrides.source_phase, overrides)
        if blockers:
            raise ValueError("ADS6 SAM realization is unavailable: " + "; ".join(blockers))
        ####
        return prepared

    ####

    def execute_batch(self, request: MissionCompositionRunRequest) -> MissionCompositionTrajectoryResult:
        if request.provider_id != CADAC_PROVIDER_ID or request.model_id != ADS6_SAM_MODEL_ID:
            raise ValueError("ADS6 SAM executor received a request for another provider/model")
        ####
        prepared = self.validate_configuration(request.prepared_configuration.configuration)
        if prepared.fingerprint != request.prepared_configuration.fingerprint:
            raise ValueError("ADS6 SAM prepared configuration does not match provider validation")
        ####
        if not isinstance(prepared.resolved, Mapping):
            raise ValueError("ADS6 SAM resolved configuration root must be a mapping")
        ####
        run = self._plugin.run_batch(_overrides_from_resolved(prepared.resolved))
        return _project_run_result(request, run, self._output_schema)

    ####


####


def build_default_ads6_sam_configuration(
    provider: CadacAds6SamMissionCompositionProvider,
    *,
    configuration_id: str = "ads6-sam-default",
    phase_id: str = ADS6_SAM_FIN_PHASE_ID,
    overrides: Mapping[str, object] | None = None,
) -> TrajectoryConfigurationInstance:
    """Create one exact ADS6 SAM source-phase configuration."""

    if phase_id not in _PHASE_IDS:
        raise ValueError(f"unsupported ADS6 SAM source phase {phase_id!r}")
    ####
    supplied = dict(overrides or {})
    routes = {
        "position_ned_m": ("initialization", "position_ned_m"),
        "speed_mps": ("initialization", "speed_mps"),
        "yaw_deg": ("initialization", "yaw_deg"),
        "pitch_deg": ("initialization", "pitch_deg"),
        "roll_deg": ("initialization", "roll_deg"),
        "alpha_deg": ("initialization", "alpha_deg"),
        "beta_deg": ("initialization", "beta_deg"),
        "body_rates_deg_s": ("initialization", "body_rates_deg_s"),
        "roll_command_deg": ("commands", "roll_command_deg"),
        "pitch_command_deg": ("commands", "pitch_command_deg"),
        "yaw_command_deg": ("commands", "yaw_command_deg"),
        "tvc_mode": ("commands", "tvc_mode"),
        "rcs_moment_mode": ("commands", "rcs_moment_mode"),
        "rcs_force_mode": ("commands", "rcs_force_mode"),
        "roll_attitude_command_deg": ("commands", "roll_attitude_command_deg"),
        "pitch_attitude_command_deg": ("commands", "pitch_attitude_command_deg"),
        "yaw_attitude_command_deg": ("commands", "yaw_attitude_command_deg"),
        "alpha_command_deg": ("commands", "alpha_command_deg"),
        "beta_command_deg": ("commands", "beta_command_deg"),
        "lateral_acceleration_command_g": ("commands", "lateral_acceleration_command_g"),
        "normal_acceleration_command_g": ("commands", "normal_acceleration_command_g"),
        "thrust_vector_unit_body": ("commands", "thrust_vector_unit_body"),
        "fin_position_limit_deg": ("actuation", "fin_position_limit_deg"),
        "fin_rate_limit_deg_s": ("actuation", "fin_rate_limit_deg_s"),
        "fin_natural_frequency_rad_s": ("actuation", "fin_natural_frequency_rad_s"),
        "fin_damping_ratio": ("actuation", "fin_damping_ratio"),
        "tvc_position_limit_deg": ("actuation", "tvc_position_limit_deg"),
        "tvc_rate_limit_deg_s": ("actuation", "tvc_rate_limit_deg_s"),
        "tvc_natural_frequency_rad_s": ("actuation", "tvc_natural_frequency_rad_s"),
        "tvc_damping_ratio": ("actuation", "tvc_damping_ratio"),
        "tvc_initial_gain": ("actuation", "tvc_initial_gain"),
        "end_time_s": ("runtime", "end_time_s"),
        "sample_step_s": ("runtime", "sample_step_s"),
    }
    unknown = sorted(set(supplied) - set(routes))
    if unknown:
        raise ValueError(f"unknown ADS6 SAM configuration overrides: {unknown!r}")
    ####
    grouped: dict[str, dict[str, object]] = {}
    for source_name, value in supplied.items():
        group_id, parameter_id = routes[source_name]
        grouped.setdefault(group_id, {})[parameter_id] = value
    ####
    root_values: dict[str, object] = {
        "source_phase": ConfigurationParameterValue(value=phase_id),
    }
    for group_id, values in grouped.items():
        root_values[group_id] = ConfigurationGroupValue(
            values={parameter_id: ConfigurationParameterValue(value=value, unit=_configuration_unit(parameter_id)) for parameter_id, value in values.items()}
        )
    ####
    fidelity, realization, mission = _phase_contract(phase_id)
    schema = provider.get_model_schema(ADS6_SAM_MODEL_ID)
    return TrajectoryConfigurationInstance(
        configuration_id=configuration_id,
        model_id=ADS6_SAM_MODEL_ID,
        model_version=ADS6_SAM_MODEL_VERSION,
        schema_fingerprint=schema.fingerprint,
        fidelity=fidelity,
        realization_id=realization,
        mission_template_id=mission,
        root=ConfigurationGroupValue(values=root_values),
    )


####


def register_ads6_sam_mission_composition(
    provider: CadacAds6SamMissionCompositionProvider,
    registry: MissionCompositionRunnerRegistry,
) -> None:
    """Register the exact provider/model ADS6 SAM batch executor."""

    registry.register(CADAC_PROVIDER_ID, ADS6_SAM_MODEL_ID, provider.execute_batch)


####


def _build_configuration_schema(plugin: Ads6SamVehiclePlugin) -> TrajectoryConfigurationSchema:
    source = plugin.source_definition()
    initial = source.initial_state
    tvc_default = source.tvc.mode if source.tvc.mode else 2
    rcs_moment_default = source.rcs.moment_mode if source.rcs.moment_mode else 21
    return TrajectoryConfigurationSchema(
        model_id=ADS6_SAM_MODEL_ID,
        model_version=ADS6_SAM_MODEL_VERSION,
        supported_fidelities=_FIDELITY_IDS,
        root=ConfigurationGroupSchema(
            id="ads6_sam",
            label="ADS6 SAM physical plant",
            description="Standalone SAM rigid-body plant with exact fin, TVC, or aggregate-RCS realization selection.",
            children=(
                ConfigurationParameterSchema(
                    id="source_phase",
                    label="Source realization",
                    description="Select the source physical-control realization.",
                    value_type="enum",
                    required=False,
                    default=ADS6_SAM_FIN_PHASE_ID,
                    default_declared=True,
                    choices=_PHASE_IDS,
                    role="variant",
                    compatible_fidelities=_FIDELITY_IDS,
                    provenance="missiondesignsolutions/CADAC/ADS6",
                ),
                ConfigurationGroupSchema(
                    id="initialization",
                    label="Initial truth",
                    children=(
                        _parameter(
                            "position_ned_m",
                            "Position",
                            "Initial local-NED position.",
                            default=initial.position_ned_m,
                            value_type="vector3",
                            unit="m",
                            frame=_LOCAL_NED_FRAME_ID,
                        ),
                        _parameter("speed_mps", "Speed", "Initial SAM speed.", default=initial.speed_mps, unit="m/s"),
                        _parameter("yaw_deg", "Yaw", "Initial body yaw angle.", default=initial.yaw_deg, unit="deg"),
                        _parameter("pitch_deg", "Pitch", "Initial body pitch angle.", default=initial.pitch_deg, unit="deg"),
                        _parameter("roll_deg", "Roll", "Initial body roll angle.", default=initial.roll_deg, unit="deg"),
                        _parameter("alpha_deg", "Angle of attack", "Initial angle of attack.", default=initial.alpha_deg, unit="deg"),
                        _parameter("beta_deg", "Sideslip", "Initial sideslip angle.", default=initial.beta_deg, unit="deg"),
                        _parameter(
                            "body_rates_deg_s",
                            "Body rates",
                            "Initial roll/pitch/yaw body rates.",
                            default=initial.body_rates_deg_s,
                            value_type="vector3",
                            unit="deg/s",
                            frame=_BODY_FRAME_ID,
                        ),
                    ),
                ),
                ConfigurationGroupSchema(
                    id="commands",
                    label="Controller-output commands",
                    description="Direct commands entering at the source controller-to-effector seam.",
                    children=(
                        _parameter("roll_command_deg", "Roll control", "Requested roll control coordinate.", default=0.0, unit="deg", role="segment"),
                        _parameter("pitch_command_deg", "Pitch control", "Requested pitch control coordinate.", default=0.0, unit="deg", role="segment"),
                        _parameter("yaw_command_deg", "Yaw control", "Requested yaw control coordinate.", default=0.0, unit="deg", role="segment"),
                        _parameter(
                            "tvc_mode",
                            "TVC mode",
                            "Source TVC mode: 1 ideal-source behavior, 2 second-order fixed gain, 3 second-order dynamic-pressure gain.",
                            default=tvc_default,
                            value_type="integer",
                            role="variant",
                        ),
                        _parameter(
                            "rcs_moment_mode",
                            "RCS moment mode",
                            "Source encoded aggregate-RCS moment type/mode.",
                            default=rcs_moment_default,
                            value_type="integer",
                            role="variant",
                        ),
                        _parameter(
                            "rcs_force_mode",
                            "RCS force mode",
                            "Source aggregate side-force mode.",
                            default=source.rcs.force_mode,
                            value_type="integer",
                            role="variant",
                        ),
                        _parameter(
                            "roll_attitude_command_deg",
                            "RCS roll attitude",
                            "Aggregate-RCS roll attitude command.",
                            default=source.rcs.roll_command_deg,
                            unit="deg",
                            role="segment",
                        ),
                        _parameter(
                            "pitch_attitude_command_deg",
                            "RCS pitch attitude",
                            "Aggregate-RCS pitch attitude command.",
                            default=source.rcs.pitch_command_deg,
                            unit="deg",
                            role="segment",
                        ),
                        _parameter(
                            "yaw_attitude_command_deg",
                            "RCS yaw attitude",
                            "Aggregate-RCS yaw attitude command.",
                            default=source.rcs.yaw_command_deg,
                            unit="deg",
                            role="segment",
                        ),
                        _parameter("alpha_command_deg", "RCS alpha", "Aggregate-RCS incidence command.", default=0.0, unit="deg", role="segment"),
                        _parameter("beta_command_deg", "RCS beta", "Aggregate-RCS sideslip command.", default=0.0, unit="deg", role="segment"),
                        _parameter(
                            "lateral_acceleration_command_g",
                            "RCS lateral acceleration",
                            "Aggregate side-force lateral acceleration command.",
                            default=0.0,
                            unit="g",
                            role="segment",
                        ),
                        _parameter(
                            "normal_acceleration_command_g",
                            "RCS normal acceleration",
                            "Aggregate side-force normal acceleration command.",
                            default=0.0,
                            unit="g",
                            role="segment",
                        ),
                        _parameter(
                            "thrust_vector_unit_body",
                            "Thrust-vector direction",
                            "Unit direction used by vector-direction RCS modes.",
                            default=(1.0, 0.0, 0.0),
                            value_type="vector3",
                            frame=_BODY_FRAME_ID,
                            role="segment",
                        ),
                    ),
                ),
                ConfigurationGroupSchema(
                    id="actuation",
                    label="Physical-effector tuning",
                    description=(
                        "Source-backed actuator limits and second-order dynamics. These values define a reproducible "
                        "vehicle variant before propagation; they are not live command channels."
                    ),
                    children=(
                        _parameter(
                            "fin_position_limit_deg",
                            "Fin position limit",
                            "Maximum physical cross-fin deflection used by the source actuator.",
                            default=source.fin_actuator.position_limit_deg,
                            unit="deg",
                            role="variant",
                            interval=_positive_interval(),
                        ),
                        _parameter(
                            "fin_rate_limit_deg_s",
                            "Fin rate limit",
                            "Maximum physical cross-fin slew rate used by the source actuator.",
                            default=source.fin_actuator.rate_limit_deg_s,
                            unit="deg/s",
                            role="variant",
                            interval=_positive_interval(),
                        ),
                        _parameter(
                            "fin_natural_frequency_rad_s",
                            "Fin natural frequency",
                            "Second-order cross-fin actuator natural frequency.",
                            default=source.fin_actuator.natural_frequency_rad_s,
                            unit="rad/s",
                            role="variant",
                            interval=_positive_interval(),
                        ),
                        _parameter(
                            "fin_damping_ratio",
                            "Fin damping ratio",
                            "Second-order cross-fin actuator damping ratio.",
                            default=source.fin_actuator.damping_ratio,
                            role="variant",
                            interval=_nonnegative_interval(),
                        ),
                        _parameter(
                            "tvc_position_limit_deg",
                            "TVC position limit",
                            "Maximum physical pitch/yaw nozzle deflection used by the source TVC actuator.",
                            default=source.tvc.position_limit_deg,
                            unit="deg",
                            role="variant",
                            interval=_positive_interval(),
                        ),
                        _parameter(
                            "tvc_rate_limit_deg_s",
                            "TVC rate limit",
                            "Maximum physical pitch/yaw nozzle slew rate used by the source TVC actuator.",
                            default=source.tvc.rate_limit_deg_s,
                            unit="deg/s",
                            role="variant",
                            interval=_positive_interval(),
                        ),
                        _parameter(
                            "tvc_natural_frequency_rad_s",
                            "TVC natural frequency",
                            "Second-order pitch/yaw TVC actuator natural frequency.",
                            default=source.tvc.natural_frequency_rad_s,
                            unit="rad/s",
                            role="variant",
                            interval=_positive_interval(),
                        ),
                        _parameter(
                            "tvc_damping_ratio",
                            "TVC damping ratio",
                            "Second-order pitch/yaw TVC actuator damping ratio.",
                            default=source.tvc.damping_ratio,
                            role="variant",
                            interval=_nonnegative_interval(),
                        ),
                        _parameter(
                            "tvc_initial_gain",
                            "TVC initial gain",
                            "Source TVC command gain before any selected dynamic-pressure scheduling.",
                            default=source.tvc.initial_gain,
                            role="variant",
                            interval=_nonnegative_interval(),
                        ),
                    ),
                ),
                ConfigurationGroupSchema(
                    id="runtime",
                    label="Runtime",
                    children=(
                        _parameter(
                            "end_time_s", "End time", "Requested plant propagation horizon.", default=min(source.end_time_s, 30.0), unit="s", role="constraint"
                        ),
                        _parameter(
                            "sample_step_s",
                            "Sample cadence",
                            "Returned trajectory cadence.",
                            default=max(source.integration_step_s, 0.02),
                            unit="s",
                            role="constraint",
                        ),
                    ),
                ),
            ),
        ),
        claim_boundary=(
            "Configuration selects one exact ADS6 SAM control realization. Commands enter at the source controller-output seam; "
            "source INS, sensor, guidance, and radar scheduling are not executed by this vehicle-only provider."
        ),
    )


####


def _parameter(
    parameter_id: str,
    label: str,
    description: str,
    *,
    default: object,
    value_type: Ads6SamConfigurationValueType = "number",
    unit: str | None = None,
    frame: str | None = None,
    role: Ads6SamConfigurationRole = "initialization",
    interval: ConfigurationInterval | None = None,
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
        interval=interval,
        role=role,
        compatible_fidelities=_FIDELITY_IDS,
        frame=frame,
        provenance="missiondesignsolutions/CADAC/ADS6",
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
    data_type: Ads6SamOutputDataType = "float64",
    interpolation: Ads6SamInterpolation = "linear",
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
        compatible_fidelities=_FIDELITY_IDS,
        compatible_realizations=_REALIZATION_IDS,
        compatible_mission_templates=_MISSION_IDS,
        operations=("batch",),
        provenance="CADAC ADS6 SAM source-grounded physical plant",
        claim_boundary="Available from the Python vehicle reconstruction; compiled-CADAC parity remains pending.",
    )


####


def _build_output_schema() -> TrajectoryOutputSchema:
    core = (
        _output("position_ned_m", "Position", "Local-NED position.", unit="m", shape=(3,), frame=_LOCAL_NED_FRAME_ID),
        _output("velocity_ned_mps", "Velocity", "Local-NED velocity.", unit="m/s", shape=(3,), frame=_LOCAL_NED_FRAME_ID),
        _output("quaternion_wxyz", "Attitude quaternion", "Scalar-first body-from-local quaternion.", shape=(4,)),
        _output("body_rates_rad_s", "Body rates", "Roll/pitch/yaw angular velocity.", unit="rad/s", shape=(3,), frame=_BODY_FRAME_ID),
    )
    telemetry = (
        _output("source_phase", "Source phase", "Selected physical-control realization.", data_type="string", interpolation="step"),
        _output("fidelity", "Sample fidelity", "Actual Taoryx fidelity for the selected realization.", data_type="string", interpolation="step"),
        _output("control_realization", "Control realization", "Physical-effector or direct-wrench closure.", data_type="string", interpolation="step"),
        _output("body_angles_deg", "Body angles", "Roll, pitch, and yaw angles.", unit="deg", shape=(3,), frame=_BODY_FRAME_ID),
        _output("incidence_deg", "Incidence", "Angle of attack and sideslip.", unit="deg", shape=(2,), frame=_BODY_FRAME_ID),
        _output("altitude_m", "Altitude", "Flat-Earth altitude.", unit="m"),
        _output("speed_mps", "Speed", "SAM speed.", unit="m/s"),
        _output("mach", "Mach", "Atmosphere-relative Mach number."),
        _output("dynamic_pressure_pa", "Dynamic pressure", "Source dynamic pressure.", unit="Pa"),
        _output("mass_kg", "Mass", "Source time-table mass.", unit="kg"),
        _output("center_of_gravity_m", "Center of gravity", "Source center-of-gravity location from the nose.", unit="m"),
        _output("inertia_diagonal_kgm2", "Inertia", "Roll, pitch, and yaw principal inertias.", unit="kg*m^2", shape=(3,), frame=_BODY_FRAME_ID),
        _output("thrust_n", "Thrust", "Source pressure-corrected rocket thrust.", unit="N"),
        _output("requested_control_deg", "Requested controls", "Requested roll/pitch/yaw control coordinates.", unit="deg", shape=(3,), frame=_BODY_FRAME_ID),
        _output("requested_fins_deg", "Requested fins", "Requested cross-fin positions.", unit="deg", shape=(4,), frame=_BODY_FRAME_ID),
        _output("achieved_fins_deg", "Achieved fins", "Actuator-achieved cross-fin positions.", unit="deg", shape=(4,), frame=_BODY_FRAME_ID),
        _output(
            "achieved_control_deg", "Achieved controls", "Control coordinates reconstructed from achieved fins.", unit="deg", shape=(3,), frame=_BODY_FRAME_ID
        ),
        _output(
            "requested_tvc_pitch_yaw_deg", "Requested TVC", "Requested source pitch/yaw control coordinates.", unit="deg", shape=(2,), frame=_BODY_FRAME_ID
        ),
        _output("achieved_tvc_pitch_yaw_deg", "Achieved TVC", "Physical achieved pitch/yaw nozzle positions.", unit="deg", shape=(2,), frame=_BODY_FRAME_ID),
        _output("tvc_effective_gain", "TVC gain", "Effective source TVC command reduction gain."),
        _output(
            "requested_rcs_attitude_deg",
            "Requested RCS attitude",
            "Requested roll/pitch/yaw attitude coordinates entering the aggregate-RCS law.",
            unit="deg",
            shape=(3,),
            frame=_BODY_FRAME_ID,
        ),
        _output(
            "requested_rcs_incidence_deg",
            "Requested RCS incidence",
            "Requested alpha/beta coordinates entering the aggregate-RCS law.",
            unit="deg",
            shape=(2,),
            frame=_BODY_FRAME_ID,
        ),
        _output(
            "requested_rcs_acceleration_g",
            "Requested RCS acceleration",
            "Requested lateral/normal acceleration coordinates entering the aggregate-RCS law.",
            unit="g",
            shape=(2,),
            frame=_BODY_FRAME_ID,
        ),
        _output(
            "requested_thrust_vector_unit_body",
            "Requested thrust-vector direction",
            "Requested unit thrust direction used by the aggregate-RCS force law.",
            shape=(3,),
            frame=_BODY_FRAME_ID,
        ),
        _output(
            "achieved_lateral_normal_acceleration_g",
            "Achieved lateral/normal acceleration",
            "Achieved body lateral/normal specific force at the accepted plant boundary.",
            unit="g",
            shape=(2,),
            frame=_BODY_FRAME_ID,
        ),
        _output("rcs_force_body_n", "RCS force", "Axis-aggregate RCS body force.", unit="N", shape=(3,), frame=_BODY_FRAME_ID),
        _output("rcs_moment_body_nm", "RCS moment", "Axis-aggregate RCS body moment.", unit="N*m", shape=(3,), frame=_BODY_FRAME_ID),
        _output("force_body_n", "Body force", "Total non-gravitational body force.", unit="N", shape=(3,), frame=_BODY_FRAME_ID),
        _output("moment_body_nm", "Body moment", "Total body moment.", unit="N*m", shape=(3,), frame=_BODY_FRAME_ID),
        _output("fin_position_limited", "Fin position limit", "Per-fin source travel-limit flags.", shape=(4,), data_type="boolean", interpolation="step"),
        _output("fin_rate_limited", "Fin rate limit", "Per-fin source rate-limit flags.", shape=(4,), data_type="boolean", interpolation="step"),
        _output(
            "tvc_position_limited", "TVC position limit", "Per-axis source nozzle travel-limit flags.", shape=(2,), data_type="boolean", interpolation="step"
        ),
        _output("tvc_rate_limited", "TVC rate limit", "Per-axis source nozzle rate-limit flags.", shape=(2,), data_type="boolean", interpolation="step"),
    )
    return TrajectoryOutputSchema(
        model_id=ADS6_SAM_MODEL_ID,
        model_version=ADS6_SAM_MODEL_VERSION,
        core_channels=core,
        telemetry_channels=telemetry,
        telemetry_groups=(
            TrajectoryTelemetryGroupMetadata(
                id="realization_evidence",
                label="Control-realization evidence",
                description="Phase, effectors, saturation, propulsion, and body-wrench evidence.",
                channel_ids=tuple(item.id for item in telemetry),
            ),
        ),
        entity_output=TrajectoryEntityOutputMetadata(),
        claim_boundary="Rigid-body truth plus phase-specific fin, TVC, or aggregate-RCS evidence from one standalone SAM.",
    )


####


def _build_model_metadata(
    schema: TrajectoryConfigurationSchema,
    output_schema: TrajectoryOutputSchema,
) -> TrajectoryModelMetadata:
    t4 = TrajectoryFidelityMetadata(
        id=ADS6_SAM_T4_FIDELITY_ID,
        label="ADS6 SAM physical-effector 6-DoF",
        rank=3,
        declared=True,
        dynamics_fidelity="rigid_body_6dof",
        input_realization="actuator_allocated",
        actuator_types=("mixed",),
        compatibility_aliases=("rigid_body_6dof_effector_allocated",),
        runtime_fidelity="rigid_body_6dof",
        control_realization="effector_allocated",
        promotion_status="development",
        operations=("validate", "batch"),
        profile_id="cadac.ads6.sam.physical-effectors",
        blockers=("compiled CADAC numerical golden parity is not yet registered",),
        required_operations=("rigid-body propagation", "physical effector realization", "force/moment closure"),
        claim_boundary="Physical cross fins or physical TVC participate; these realizations do not imply source sensor/guidance execution.",
    )
    t3 = TrajectoryFidelityMetadata(
        id=ADS6_SAM_T3_FIDELITY_ID,
        label="ADS6 SAM aggregate-RCS 6-DoF",
        rank=2,
        declared=True,
        dynamics_fidelity="rigid_body_6dof",
        input_realization="direct_wrench",
        actuator_types=("not_applicable",),
        runtime_fidelity="rigid_body_6dof",
        control_realization="direct_wrench",
        promotion_status="development",
        operations=("validate", "batch"),
        profile_id="cadac.ads6.sam.aggregate-rcs",
        blockers=("individual RCS jet geometry/allocation and compiled CADAC numerical parity are not registered",),
        required_operations=("rigid-body propagation", "axis-aggregate RCS wrench", "force/moment closure"),
        claim_boundary="RCS is resolved only as axis-aggregate force/moment and is not promoted to physical-thruster allocation.",
    )
    realizations = (
        _realization(
            ADS6_SAM_FIN_REALIZATION_ID,
            "CADAC cross-fin realization",
            "Four independently dynamic cross-configured aerodynamic fins.",
            fidelity=ADS6_SAM_T4_FIDELITY_ID,
            input_realization="actuator_allocated",
            actuator_types=("aerodynamic_surfaces",),
            mission_id=ADS6_SAM_FIN_MISSION_ID,
            controls=batch_configuration_control_advertisement(
                schema,
                channels={
                    "actuator.roll.command": ("commands", "roll_command_deg"),
                    "actuator.pitch.command": ("commands", "pitch_command_deg"),
                    "actuator.yaw.command": ("commands", "yaw_command_deg"),
                },
                output_evidence={
                    "actuator.roll.command": CadacControlOutputEvidence("requested_control_deg", ("achieved_control_deg",), 0, (0,)),
                    "actuator.pitch.command": CadacControlOutputEvidence("requested_control_deg", ("achieved_control_deg",), 1, (1,)),
                    "actuator.yaw.command": CadacControlOutputEvidence("requested_control_deg", ("achieved_control_deg",), 2, (2,)),
                },
                authority_id="cross_fin_commands",
                authority="effector",
                authority_description="Caller-owned cross-fin control coordinates fixed for one direct-fin batch run.",
                intent_id="direct_fin_control",
                intent_label="Direct Fin Control",
                intent_description="Command the ADS6 SAM cross-fin plant at its direct roll/pitch/yaw control-coordinate boundary.",
                mission_ids=(ADS6_SAM_FIN_MISSION_ID,),
                source_refs=("missiondesignsolutions/CADAC/ADS6",),
                claim_boundary="The caller supplies fixed direct cross-fin controls. ADS6 engagement guidance and common-runner step control remain outside this standalone plant.",
            ),
        ),
        _realization(
            ADS6_SAM_TVC_REALIZATION_ID,
            "CADAC physical TVC realization",
            "Physical pitch/yaw nozzle actuator with source gain scheduling and thrust wrench.",
            fidelity=ADS6_SAM_T4_FIDELITY_ID,
            input_realization="actuator_allocated",
            actuator_types=("thrust_vectoring",),
            mission_id=ADS6_SAM_TVC_MISSION_ID,
            controls=batch_configuration_control_advertisement(
                schema,
                channels={
                    "tvc.pitch.deflection": ("commands", "pitch_command_deg"),
                    "tvc.yaw.deflection": ("commands", "yaw_command_deg"),
                },
                output_evidence={
                    "tvc.pitch.deflection": CadacControlOutputEvidence("requested_tvc_pitch_yaw_deg", ("achieved_tvc_pitch_yaw_deg",), 0, (0,)),
                    "tvc.yaw.deflection": CadacControlOutputEvidence("requested_tvc_pitch_yaw_deg", ("achieved_tvc_pitch_yaw_deg",), 1, (1,)),
                },
                authority_id="physical_tvc_commands",
                authority="effector",
                authority_description="Caller-owned pitch/yaw control coordinates fixed for one direct-TVC batch run.",
                intent_id="direct_tvc_control",
                intent_label="Direct TVC Control",
                intent_description="Command the ADS6 SAM pitch/yaw nozzle boundary through the reconstructed direct-TVC plant.",
                mission_ids=(ADS6_SAM_TVC_MISSION_ID,),
                source_refs=("missiondesignsolutions/CADAC/ADS6",),
                claim_boundary="The caller supplies fixed direct pitch/yaw controls. ADS6 engagement guidance and common-runner step control remain outside this standalone plant.",
            ),
        ),
        _realization(
            ADS6_SAM_RCS_REALIZATION_ID,
            "CADAC aggregate RCS realization",
            "Axis-aggregate proportional or Schmitt RCS force/moment closure.",
            fidelity=ADS6_SAM_T3_FIDELITY_ID,
            input_realization="direct_wrench",
            actuator_types=("not_applicable",),
            mission_id=ADS6_SAM_RCS_MISSION_ID,
            controls=batch_configuration_control_advertisement(
                schema,
                channels={
                    "rcs.roll.attitude_command": ("commands", "roll_attitude_command_deg"),
                    "rcs.pitch.attitude_command": ("commands", "pitch_attitude_command_deg"),
                    "rcs.yaw.attitude_command": ("commands", "yaw_attitude_command_deg"),
                    "rcs.alpha.command": ("commands", "alpha_command_deg"),
                    "rcs.beta.command": ("commands", "beta_command_deg"),
                    "rcs.lateral_acceleration.command": ("commands", "lateral_acceleration_command_g"),
                    "rcs.normal_acceleration.command": ("commands", "normal_acceleration_command_g"),
                    "rcs.thrust_vector.direction": ("commands", "thrust_vector_unit_body"),
                },
                output_evidence={
                    "rcs.roll.attitude_command": CadacControlOutputEvidence("requested_rcs_attitude_deg", ("rcs_moment_body_nm",), 0, (0,)),
                    "rcs.pitch.attitude_command": CadacControlOutputEvidence("requested_rcs_attitude_deg", ("rcs_moment_body_nm",), 1, (1,)),
                    "rcs.yaw.attitude_command": CadacControlOutputEvidence("requested_rcs_attitude_deg", ("rcs_moment_body_nm",), 2, (2,)),
                    "rcs.alpha.command": CadacControlOutputEvidence("requested_rcs_incidence_deg", ("incidence_deg",), 0, (0,)),
                    "rcs.beta.command": CadacControlOutputEvidence("requested_rcs_incidence_deg", ("incidence_deg",), 1, (1,)),
                    "rcs.lateral_acceleration.command": CadacControlOutputEvidence(
                        "requested_rcs_acceleration_g", ("achieved_lateral_normal_acceleration_g",), 0, (0,)
                    ),
                    "rcs.normal_acceleration.command": CadacControlOutputEvidence(
                        "requested_rcs_acceleration_g", ("achieved_lateral_normal_acceleration_g",), 1, (1,)
                    ),
                    "rcs.thrust_vector.direction": CadacControlOutputEvidence("requested_thrust_vector_unit_body", ("rcs_force_body_n",)),
                },
                authority_id="aggregate_rcs_commands",
                authority="wrench",
                authority_description="Caller-owned aggregate-RCS command coordinates fixed for one direct-RCS batch run.",
                intent_id="direct_rcs_control",
                intent_label="Direct Aggregate-RCS Control",
                intent_description="Command the ADS6 SAM aggregate-RCS force/moment boundary without claiming individual thruster allocation.",
                mission_ids=(ADS6_SAM_RCS_MISSION_ID,),
                source_refs=("missiondesignsolutions/CADAC/ADS6",),
                claim_boundary="The caller supplies fixed aggregate-RCS command coordinates. Individual jet allocation, engagement guidance, and common-runner step control remain unavailable.",
            ),
        ),
    )
    missions = (
        _mission(ADS6_SAM_FIN_MISSION_ID, "ADS6 SAM direct-fin plant", ADS6_SAM_T4_FIDELITY_ID, ADS6_SAM_FIN_REALIZATION_ID, "direct_fin"),
        _mission(ADS6_SAM_TVC_MISSION_ID, "ADS6 SAM direct-TVC plant", ADS6_SAM_T4_FIDELITY_ID, ADS6_SAM_TVC_REALIZATION_ID, "direct_tvc"),
        _mission(ADS6_SAM_RCS_MISSION_ID, "ADS6 SAM direct-RCS plant", ADS6_SAM_T3_FIDELITY_ID, ADS6_SAM_RCS_REALIZATION_ID, "direct_rcs"),
    )
    return TrajectoryModelMetadata(
        id=ADS6_SAM_MODEL_ID,
        name="CADAC ADS6 SAM",
        version=ADS6_SAM_MODEL_VERSION,
        description="ADS6 surface-to-air missile exposed as a multi-realization Taoryx rigid-body vehicle plug-in.",
        presentation=TrajectoryModelPresentationMetadata(
            display_name="CADAC ADS6 SAM",
            short_name="ADS6 SAM",
            summary="Source-grounded rigid-body SAM with exact cross-fin, physical-TVC, and aggregate-RCS realization choices.",
            category="missile",
            subcategory="surface-to-air",
            sort_key="cadac-ads6-sam",
            badges=("CADAC", "6-DoF", "multi-realization", "development"),
            default_fidelity_id=ADS6_SAM_T4_FIDELITY_ID,
            default_mission_template_id=ADS6_SAM_FIN_MISSION_ID,
            default_output_channel_ids=("position_ned_m", "velocity_ned_mps", "quaternion_wxyz", "body_rates_rad_s"),
            properties=(
                TrajectoryModelPropertyMetadata(
                    id="source_model",
                    label="Source model",
                    description="Upstream CADAC actor identifier.",
                    semantic_role="identity",
                    value_type="string",
                    value_kind="declared",
                    value="MISSILE6/SAM",
                    value_declared=True,
                    source_refs=("missiondesignsolutions/CADAC/ADS6",),
                    provenance="CADAC ADS6 source package",
                    claim_boundary="Vehicle identity only; full ADS6 engagement composition is a later trench.",
                ),
            ),
        ),
        family_id=ADS6_SAM_MODEL_ID,
        physical_family="cadac_ads6_sam",
        model_kind="vehicle_plugin",
        status="development",
        tags=("cadac", "ads6", "sam", "physical-effectors", "aggregate-rcs"),
        operations=("discover", "validate", "batch"),
        common_runner_operations=("batch",),
        capabilities=TrajectoryModelCapabilities(
            initialization_modes=("source_case_override",),
            segment_types=("direct_fin", "direct_tvc", "direct_rcs"),
            termination_modes=("end_time", "ground_impact", "nonfinite_state"),
            operations=("discover", "validate", "batch"),
            supports_custom_segments=False,
            supports_deployment=False,
            supports_staging=False,
            supports_dynamic_child_generation=False,
            supports_multiple_stages=False,
            supports_submodels=False,
        ),
        realizations=realizations,
        mission_templates=missions,
        deployments=(),
        reference_frames=(
            TrajectoryReferenceFrameMetadata(
                id=_LOCAL_NED_FRAME_ID,
                name="CADAC flat-Earth local NED",
                description="Local North-East-Down frame used by ADS6 Flat6.",
                frame_kind="local_tangent",
                axes=("north", "east", "down"),
                handedness="right",
                origin="source-case local reference point E",
                orientation="north-east-down",
                source_refs=("missiondesignsolutions/CADAC/ADS6/newton.cpp",),
                provenance="CADAC ADS6 Flat6 convention",
            ),
            TrajectoryReferenceFrameMetadata(
                id=_BODY_FRAME_ID,
                name="CADAC SAM body axes",
                description="Body-fixed axes used by source forces, moments, rates, fins, TVC, and RCS.",
                frame_kind="body",
                axes=("x", "y", "z"),
                handedness="right",
                origin="instantaneous SAM reference point",
                orientation="source ADS6 body convention",
                source_refs=("missiondesignsolutions/CADAC/ADS6",),
                provenance="CADAC ADS6 body convention",
            ),
        ),
        output_schema=output_schema,
        output_schema_id=output_schema.schema_id,
        output_schema_fingerprint=output_schema.fingerprint,
        configuration_schema_id=schema.schema_id,
        configuration_schema_fingerprint=schema.fingerprint,
        fidelities=(t4, t3),
        fidelity_transitions=(),
        source_refs=("missiondesignsolutions/CADAC/ADS6",),
        provenance="CADAC ADS6 source-grounded Python SAM physical-plant reconstruction",
        claim_boundary=(
            "Executable vehicle-only plant at development status. ADS6 radar, target actors, seeker/guidance/INS command generation, "
            "and compiled-CADAC numerical parity remain separate integration and promotion gates."
        ),
    )


####


def _realization(
    realization_id: str,
    label: str,
    description: str,
    *,
    fidelity: str,
    input_realization: Ads6SamInputRealization,
    actuator_types: tuple[Ads6SamActuatorType, ...],
    mission_id: str,
    controls: TrajectoryControlAdvertisement,
) -> TrajectoryRealizationMetadata:
    return TrajectoryRealizationMetadata(
        id=realization_id,
        label=label,
        description=description,
        status="available",
        dynamics_fidelities=("rigid_body_6dof",),
        input_realization=input_realization,
        actuator_types=actuator_types,
        controls=controls,
        fidelity_aliases=(fidelity,),
        mission_template_ids=(mission_id,),
        operations=("validate", "batch"),
        native_factory_ids=(ADS6_SAM_EXECUTOR_ID,),
        source_refs=("missiondesignsolutions/CADAC/ADS6",),
        blockers=("compiled CADAC numerical golden parity is not yet registered",),
        claim_boundary="Exact source realization is executable through the standalone SAM plant boundary.",
    )


####


def _mission(
    mission_id: str,
    name: str,
    fidelity: str,
    realization_id: str,
    segment: str,
) -> TrajectoryMissionTemplateMetadata:
    return TrajectoryMissionTemplateMetadata(
        id=mission_id,
        name=name,
        description=f"Propagate the standalone ADS6 SAM using its {segment.replace('_', ' ')} realization.",
        status="development",
        initialization_variants=("source_case_override",),
        segment_sequence=(segment,),
        compatible_fidelities=(fidelity,),
        operations=(
            TrajectoryMissionOperationMetadata(
                fidelity=fidelity,
                realization_id=realization_id,
                operation="validate",
                status="available",
                execution_mode="source_grounded_validation",
                common_runner_status="not_available",
                claim_boundary="Validation checks exact phase, fidelity, realization, and installed source resources.",
            ),
            TrajectoryMissionOperationMetadata(
                fidelity=fidelity,
                realization_id=realization_id,
                operation="batch",
                status="available",
                execution_mode="source_grounded_physical_plant",
                availability_scope="native_runtime_binding",
                common_runner_status="registered",
                executor_id=ADS6_SAM_EXECUTOR_ID,
                claim_boundary="Batch dispatch reaches the exact installed ADS6 SAM plug-in.",
            ),
        ),
        provenance="CADAC ADS6 SAM rigid-body, propulsion, aerodynamics, effector, Euler, and Newton modules",
        claim_boundary="Direct vehicle-plant mission; ADS6 engagement-level command generation is outside this mission.",
    )


####


def _phase_contract(phase_id: str) -> tuple[str, str, str]:
    if phase_id == ADS6_SAM_FIN_PHASE_ID:
        return ADS6_SAM_T4_FIDELITY_ID, ADS6_SAM_FIN_REALIZATION_ID, ADS6_SAM_FIN_MISSION_ID
    ####
    if phase_id == ADS6_SAM_TVC_PHASE_ID:
        return ADS6_SAM_T4_FIDELITY_ID, ADS6_SAM_TVC_REALIZATION_ID, ADS6_SAM_TVC_MISSION_ID
    ####
    if phase_id == ADS6_SAM_RCS_PHASE_ID:
        return ADS6_SAM_T3_FIDELITY_ID, ADS6_SAM_RCS_REALIZATION_ID, ADS6_SAM_RCS_MISSION_ID
    ####
    raise ValueError(f"unsupported ADS6 SAM source phase {phase_id!r}")


####


def _overrides_from_resolved(resolved: Mapping[str, object]) -> Ads6SamPluginOverrides:
    phase = resolved.get("source_phase")
    if not isinstance(phase, str):
        raise ValueError("ADS6 SAM resolved configuration requires source_phase")
    ####
    initial = _group(resolved, "initialization")
    commands = _group(resolved, "commands")
    actuation = _group(resolved, "actuation")
    runtime = _group(resolved, "runtime")
    return Ads6SamPluginOverrides(
        source_phase=phase,
        position_ned_m=_vector3(initial["position_ned_m"]),
        speed_mps=float(initial["speed_mps"]),
        yaw_deg=float(initial["yaw_deg"]),
        pitch_deg=float(initial["pitch_deg"]),
        roll_deg=float(initial["roll_deg"]),
        alpha_deg=float(initial["alpha_deg"]),
        beta_deg=float(initial["beta_deg"]),
        body_rates_deg_s=_vector3(initial["body_rates_deg_s"]),
        roll_command_deg=float(commands["roll_command_deg"]),
        pitch_command_deg=float(commands["pitch_command_deg"]),
        yaw_command_deg=float(commands["yaw_command_deg"]),
        tvc_mode=int(commands["tvc_mode"]),
        rcs_moment_mode=int(commands["rcs_moment_mode"]),
        rcs_force_mode=int(commands["rcs_force_mode"]),
        roll_attitude_command_deg=float(commands["roll_attitude_command_deg"]),
        pitch_attitude_command_deg=float(commands["pitch_attitude_command_deg"]),
        yaw_attitude_command_deg=float(commands["yaw_attitude_command_deg"]),
        alpha_command_deg=float(commands["alpha_command_deg"]),
        beta_command_deg=float(commands["beta_command_deg"]),
        lateral_acceleration_command_g=float(commands["lateral_acceleration_command_g"]),
        normal_acceleration_command_g=float(commands["normal_acceleration_command_g"]),
        thrust_vector_unit_body=_vector3(commands["thrust_vector_unit_body"]),
        fin_position_limit_deg=float(actuation["fin_position_limit_deg"]),
        fin_rate_limit_deg_s=float(actuation["fin_rate_limit_deg_s"]),
        fin_natural_frequency_rad_s=float(actuation["fin_natural_frequency_rad_s"]),
        fin_damping_ratio=float(actuation["fin_damping_ratio"]),
        tvc_position_limit_deg=float(actuation["tvc_position_limit_deg"]),
        tvc_rate_limit_deg_s=float(actuation["tvc_rate_limit_deg_s"]),
        tvc_natural_frequency_rad_s=float(actuation["tvc_natural_frequency_rad_s"]),
        tvc_damping_ratio=float(actuation["tvc_damping_ratio"]),
        tvc_initial_gain=float(actuation["tvc_initial_gain"]),
        end_time_s=float(runtime["end_time_s"]),
        sample_step_s=float(runtime["sample_step_s"]),
    )


####


def _group(root: Mapping[str, object], name: str) -> Mapping[str, object]:
    value = root.get(name)
    if not isinstance(value, Mapping):
        raise ValueError(f"ADS6 SAM resolved configuration requires group {name!r}")
    ####
    return value


####


def _vector3(value: object) -> tuple[float, float, float]:
    if not isinstance(value, (tuple, list)) or len(value) != 3:
        raise ValueError("ADS6 SAM vector configuration requires exactly three components")
    ####
    return (float(value[0]), float(value[1]), float(value[2]))


####


def _configuration_unit(parameter_id: str) -> str | None:
    if parameter_id == "position_ned_m":
        return "m"
    ####
    if parameter_id == "speed_mps":
        return "m/s"
    ####
    if parameter_id in {
        "yaw_deg",
        "pitch_deg",
        "roll_deg",
        "alpha_deg",
        "beta_deg",
        "roll_command_deg",
        "pitch_command_deg",
        "yaw_command_deg",
        "roll_attitude_command_deg",
        "pitch_attitude_command_deg",
        "yaw_attitude_command_deg",
        "alpha_command_deg",
        "beta_command_deg",
        "fin_position_limit_deg",
        "tvc_position_limit_deg",
    }:
        return "deg"
    ####
    if parameter_id == "body_rates_deg_s":
        return "deg/s"
    ####
    if parameter_id in {"fin_rate_limit_deg_s", "tvc_rate_limit_deg_s"}:
        return "deg/s"
    ####
    if parameter_id in {"fin_natural_frequency_rad_s", "tvc_natural_frequency_rad_s"}:
        return "rad/s"
    ####
    if parameter_id in {"lateral_acceleration_command_g", "normal_acceleration_command_g"}:
        return "g"
    ####
    if parameter_id in {"end_time_s", "sample_step_s"}:
        return "s"
    ####
    return None


####


def _positive_interval() -> ConfigurationInterval:
    """Return the source-model domain for a strictly positive scalar."""

    return ConfigurationInterval(minimum=ConfigurationBound(value=0.0, inclusive=False))


####


def _nonnegative_interval() -> ConfigurationInterval:
    """Return the source-model domain for a nonnegative scalar."""

    return ConfigurationInterval(minimum=ConfigurationBound(value=0.0))


####


def _project_run_result(
    request: MissionCompositionRunRequest,
    run: Ads6SamRunResult,
    output_schema: TrajectoryOutputSchema,
) -> MissionCompositionTrajectoryResult:
    fidelity, realization, mission = _phase_contract(run.source_phase)
    selected = resolve_output_selection(
        output_schema,
        request.output,
        fidelity=fidelity,
        operation="batch",
        realization_id=realization,
        mission_template_id=mission,
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
    terminated = run.terminated_reason == "ground_impact"
    object_result = TrajectoryObject(
        object_id="ads6-sam-1",
        model_id=ADS6_SAM_MODEL_ID,
        realization_id=realization,
        name="CADAC ADS6 SAM",
        role="interceptor",
        fidelity=fidelity,
        status="failed" if failed else "terminated" if terminated else "completed",
        active_from_s=samples[0].time_s,
        active_to_s=samples[-1].time_s,
        terminal_disposition=run.terminated_reason,
        channels=channels,
        samples=samples,
        provenance="CADAC ADS6 standalone SAM source-grounded physical plant",
        claim_boundary="One exact vehicle realization participates; ADS6 radar and target composition are not inferred.",
    )
    diagnostics = (
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-parity-pending",
            message="ADS6 SAM executes through the source-grounded Python plant; compiled-CADAC numerical parity is not yet promoted.",
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=ADS6_SAM_MODEL_ID,
            object_id="ads6-sam-1",
        ),
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-direct-effector-boundary",
            message="Commands enter at the source controller-output seam; INS, seeker, guidance, and radar-generated commands are outside this vehicle-only realization.",
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=ADS6_SAM_MODEL_ID,
            object_id="ads6-sam-1",
        ),
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-phase-fidelity-explicit",
            message=("Cross fins and physical TVC are T4 physical-effector realizations; axis-aggregate RCS remains T3 direct wrench."),
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=ADS6_SAM_MODEL_ID,
            object_id="ads6-sam-1",
        ),
    )
    return MissionCompositionTrajectoryResult(
        provider_id=CADAC_PROVIDER_ID,
        provider_version=ADS6_SAM_MODEL_VERSION,
        request_id=request.request_id,
        configuration_fingerprint=request.prepared_configuration.fingerprint,
        primary_model_id=ADS6_SAM_MODEL_ID,
        primary_object_id="ads6-sam-1",
        status="failed" if failed else "terminated" if terminated else "completed",
        objects=(object_result,),
        events=(),
        relationships=(),
        diagnostics=diagnostics,
        claim_boundary="One standalone ADS6 SAM root object returned through exact provider/model dispatch.",
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


def _sample_value(channel_id: str, sample: Ads6SamSample) -> object:
    return getattr(sample, channel_id)


####


__all__ = [
    "ADS6_SAM_EXECUTOR_ID",
    "ADS6_SAM_FIN_MISSION_ID",
    "ADS6_SAM_FIN_PHASE_ID",
    "ADS6_SAM_FIN_REALIZATION_ID",
    "ADS6_SAM_MODEL_ID",
    "ADS6_SAM_MODEL_VERSION",
    "ADS6_SAM_RCS_MISSION_ID",
    "ADS6_SAM_RCS_PHASE_ID",
    "ADS6_SAM_RCS_REALIZATION_ID",
    "ADS6_SAM_T3_FIDELITY_ID",
    "ADS6_SAM_T4_FIDELITY_ID",
    "ADS6_SAM_TVC_MISSION_ID",
    "ADS6_SAM_TVC_PHASE_ID",
    "ADS6_SAM_TVC_REALIZATION_ID",
    "CadacAds6SamMissionCompositionProvider",
    "build_default_ads6_sam_configuration",
    "register_ads6_sam_mission_composition",
]
