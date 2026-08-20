"""Mission Composition advertisement and execution for parametric interceptors."""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Callable, Literal, TypeVar

from taoryx.composition_episode import EpisodeStatus
from taoryx.fixture_composition_episode import FixtureCompositionEpisode, FixtureTransition
from taoryx.runtime.environment_runtime import EnvironmentProvider
from taoryx.trajectory.configuration_contract import (
    ConfigurationBound,
    ConfigurationContractError,
    ConfigurationGroupSchema,
    ConfigurationGroupValue,
    ConfigurationInterval,
    ConfigurationParameterSchema,
    ConfigurationParameterValue,
    ConfigurationValueSpace,
    ControlCommandSemantics,
    PreparedTrajectoryConfiguration,
    PresentationLinkMetadata,
    TrajectoryConfigurationInstance,
    TrajectoryConfigurationSchema,
    TrajectoryControlAdvertisement,
    TrajectoryControlAuthorityMetadata,
    TrajectoryControlChannelMetadata,
    TrajectoryControlIntentMetadata,
    TrajectoryControlNativeBindingMetadata,
    TrajectoryFidelityMetadata,
    TrajectoryFidelityTransition,
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
    ValuePresentationMetadata,
    build_trajectory_composition_advertisement,
    validate_configuration_instance,
)
from taoryx.trajectory.execution_contract import (
    MissionCompositionDiagnostic,
    MissionCompositionExecutionError,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
    MissionCompositionTrajectoryResult,
    TrajectoryChannelMetadata,
    TrajectoryEvent,
    TrajectoryObject,
    TrajectorySample,
    TrajectorySegmentResult,
    resolve_output_selection,
)
from taoryx.trajectory.session_interface import build_session_interface_contract

from .applicability import (
    APPLICABILITY_CONTRACT,
    APPLICABILITY_ENFORCEMENT,
    InterceptorApplicabilityEnvelope,
)
from .calibration import (
    DEFAULT_ENVIRONMENT_MODEL_ID,
    DEFAULT_GRAVITY_MODEL_ID,
    CalibrationFidelity,
    InterceptorCalibrationResult,
    InterceptorCalibrationScenario,
    calibration_scenario_from_reported_profile,
    evaluate_interceptor_calibration,
)
from .catalogue import load_catalogue_interceptor_record
from .control_authority import CONTROL_ALLOCATION_POLICIES
from .kernel import (
    PointMassKernel,
    PointMassMission,
    PointMassRun,
    PointMassSample,
    PointMassState,
    PointMassWaypoint,
    run_point_mass_interceptor,
)
from .profile import (
    PARAMETER_USAGE_CONTRACT,
    AssumptionCase,
    EvidenceInterval,
    InterceptorEvidenceProfile,
    ResolvedInterceptorProfile,
    interceptor_parameter_usage,
    load_interceptor_profile,
)
from .pseudo6 import (
    Pseudo6AttitudeStepRun,
    Pseudo6Kernel,
    Pseudo6ResponseAxis,
    Pseudo6Run,
    Pseudo6Sample,
    Pseudo6State,
    run_pseudo6_attitude_step,
    run_pseudo6_interceptor,
)
from .resolver import resolve_interceptor
from .response_analysis import (
    Pseudo6OperatingPointAnalysisRequest,
    Pseudo6OperatingPointResponseAnalysis,
    Pseudo6OperatingPointResponseComparison,
    Pseudo6ResponseAnalysis,
    Pseudo6ResponseAnalysisRequest,
    Pseudo6ResponseComparison,
    Pseudo6ResponseOperatingPoint,
    analyze_pseudo6_response,
    analyze_pseudo6_response_at_operating_point,
    compare_pseudo6_responses,
    compare_pseudo6_responses_at_operating_point,
)
from .sensor_suite import (
    DEFAULT_SENSOR_SUITE_ID,
    InterceptorSensorSuite,
    standard_interceptor_sensor_suite,
)
from .thrust_curve import AbsoluteThrustCurve

PROVIDER_ID = "taoryx.parametric-interceptors.mission-composition"
PACKAGE_VERSION = "0.1.0a32"
POINT_MASS_FIDELITY_ID = "point_mass_3dof"
PSEUDO6_FIDELITY_ID = "attitude_response_pseudo_6dof"
POINT_MASS_REALIZATION_ID = "parametric_guided_point_mass"
PSEUDO6_REALIZATION_ID = "parametric_attitude_response_pseudo6"
# Backward-compatible names for the default and original tier.
FIDELITY_ID = POINT_MASS_FIDELITY_ID
REALIZATION_ID = POINT_MASS_REALIZATION_ID
MISSION_TEMPLATE_ID = "fixed_waypoint_intercept"
TARGET_TRACK_MISSION_TEMPLATE_ID = "constant_velocity_target_intercept"
DIRECT_ACCELERATION_MISSION_TEMPLATE_ID = "direct_lateral_acceleration_control"
MISSION_CONTROL_SCOPE_CONTRACT = "taoryx.parametric-interceptors.exclusive-mission-controls/v1"
_MISSION_TEMPLATE_IDS = (
    MISSION_TEMPLATE_ID,
    TARGET_TRACK_MISSION_TEMPLATE_ID,
    DIRECT_ACCELERATION_MISSION_TEMPLATE_ID,
)

_DEFAULTS: dict[str, float] = {
    "launch.north_m": 0.0,
    "launch.east_m": 0.0,
    "launch.altitude_m": 0.0,
    "launch.speed_mps": 20.0,
    "launch.heading_deg": 0.0,
    "launch.flight_path_deg": 45.0,
    "navigation.waypoint.north.command": 10_000.0,
    "navigation.waypoint.east.command": 0.0,
    "navigation.waypoint.altitude.command": 3_000.0,
    "navigation.waypoint.capture_radius.command": 100.0,
    "navigation.target.position.north.command": 10_000.0,
    "navigation.target.position.east.command": 0.0,
    "navigation.target.position.altitude.command": 3_000.0,
    "navigation.target.velocity.north.command": 0.0,
    "navigation.target.velocity.east.command": 0.0,
    "navigation.target.velocity.vertical.command": 0.0,
    "navigation.target.capture_radius.command": 100.0,
    "control.lateral_acceleration.local.north.command": 0.0,
    "control.lateral_acceleration.local.east.command": 0.0,
    "control.lateral_acceleration.local.vertical.command": 0.0,
    "runtime.duration_s": 60.0,
    "runtime.time_step_s": 0.05,
}
_WAYPOINT_ACTION_IDS = (
    "navigation.waypoint.north.command",
    "navigation.waypoint.east.command",
    "navigation.waypoint.altitude.command",
    "navigation.waypoint.capture_radius.command",
)
_TARGET_TRACK_ACTION_IDS = (
    "navigation.target.position.north.command",
    "navigation.target.position.east.command",
    "navigation.target.position.altitude.command",
    "navigation.target.velocity.north.command",
    "navigation.target.velocity.east.command",
    "navigation.target.velocity.vertical.command",
    "navigation.target.capture_radius.command",
)
_DIRECT_ACCELERATION_ACTION_IDS = (
    "control.lateral_acceleration.local.north.command",
    "control.lateral_acceleration.local.east.command",
    "control.lateral_acceleration.local.vertical.command",
)
_MISSION_ACTION_IDS = {
    MISSION_TEMPLATE_ID: _WAYPOINT_ACTION_IDS,
    TARGET_TRACK_MISSION_TEMPLATE_ID: _TARGET_TRACK_ACTION_IDS,
    DIRECT_ACCELERATION_MISSION_TEMPLATE_ID: _DIRECT_ACCELERATION_ACTION_IDS,
}
_ACTION_MISSION_TEMPLATE_IDS = {action_id: mission_template_id for mission_template_id, action_ids in _MISSION_ACTION_IDS.items() for action_id in action_ids}
_RuntimeModel = TypeVar("_RuntimeModel")


def _runtime_model_registry(
    built_in_model_id: str,
    models: Mapping[str, _RuntimeModel] | None,
    kind: str,
) -> dict[str, _RuntimeModel | None]:
    """Build one explicit-ID runtime dependency registry around the built-in default."""

    registry: dict[str, _RuntimeModel | None] = {built_in_model_id: None}
    for identifier, model in (models or {}).items():
        if not identifier or identifier != identifier.strip():
            raise ValueError(f"{kind} model IDs must be non-empty and must not have surrounding whitespace")
        if identifier == built_in_model_id:
            raise ValueError(f"{kind} model ID {identifier!r} is reserved for the built-in standard model")
        registry[identifier] = model
    return registry
    ####


def _sensor_suite_registry(
    suites: Mapping[str, InterceptorSensorSuite] | None,
) -> dict[str, InterceptorSensorSuite]:
    """Build a deterministic registry without sharing sensor-instance state."""

    standard = standard_interceptor_sensor_suite()
    registry = {standard.id: standard}
    for identifier, suite in (suites or {}).items():
        if not identifier or identifier != identifier.strip():
            raise ValueError("sensor suite IDs must be non-empty and must not have surrounding whitespace")
        if identifier != suite.id:
            raise ValueError(f"sensor suite registry key {identifier!r} must match suite ID {suite.id!r}")
        if identifier == DEFAULT_SENSOR_SUITE_ID:
            raise ValueError(f"sensor suite ID {identifier!r} is reserved for the built-in standard suite")
        registry[identifier] = suite
    return registry
    ####


class ParametricInterceptorMissionCompositionProvider:
    """Provider built from one or more sparse or resolved profiles."""

    def __init__(
        self,
        profiles: Sequence[InterceptorEvidenceProfile | ResolvedInterceptorProfile],
        *,
        provider_version: str = PACKAGE_VERSION,
        assumption_case: AssumptionCase | str | None = None,
        environment_models: Mapping[str, EnvironmentProvider] | None = None,
        gravity_models: Mapping[str, Callable[[float], float]] | None = None,
        sensor_suites: Mapping[str, InterceptorSensorSuite] | None = None,
        default_environment_model_id: str = DEFAULT_ENVIRONMENT_MODEL_ID,
        default_gravity_model_id: str = DEFAULT_GRAVITY_MODEL_ID,
        default_sensor_suite_id: str = DEFAULT_SENSOR_SUITE_ID,
    ) -> None:
        if not profiles:
            raise ValueError("at least one interceptor profile is required")
        resolved = tuple(
            item if isinstance(item, ResolvedInterceptorProfile) else resolve_interceptor(item, assumption_case=assumption_case) for item in profiles
        )
        model_ids = tuple(item.model_id for item in resolved)
        if len(model_ids) != len(set(model_ids)):
            raise ValueError("parametric interceptor model IDs must be unique")
        self._environment_models = _runtime_model_registry(
            DEFAULT_ENVIRONMENT_MODEL_ID,
            environment_models,
            "environment",
        )
        self._gravity_models = _runtime_model_registry(
            DEFAULT_GRAVITY_MODEL_ID,
            gravity_models,
            "gravity",
        )
        self._sensor_suites = _sensor_suite_registry(sensor_suites)
        if default_environment_model_id not in self._environment_models:
            raise ValueError(f"unknown default environment model {default_environment_model_id!r}")
        if default_gravity_model_id not in self._gravity_models:
            raise ValueError(f"unknown default gravity model {default_gravity_model_id!r}")
        if default_sensor_suite_id not in self._sensor_suites:
            raise ValueError(f"unknown default sensor suite {default_sensor_suite_id!r}")
        self._default_environment_model_id = default_environment_model_id
        self._default_gravity_model_id = default_gravity_model_id
        self._default_sensor_suite_id = default_sensor_suite_id
        self._profiles = {item.model_id: item for item in resolved}
        self._schemas = {
            item.model_id: _configuration_schema(
                item,
                environment_model_ids=tuple(self._environment_models),
                gravity_model_ids=tuple(self._gravity_models),
                sensor_suites=self._sensor_suites,
                default_environment_model_id=default_environment_model_id,
                default_gravity_model_id=default_gravity_model_id,
                default_sensor_suite_id=default_sensor_suite_id,
            )
            for item in resolved
        }
        self._models = {
            item.model_id: _model_metadata(
                item,
                self._schemas[item.model_id],
                environment_model_ids=tuple(self._environment_models),
                gravity_model_ids=tuple(self._gravity_models),
                sensor_suites=self._sensor_suites,
                default_environment_model_id=default_environment_model_id,
                default_gravity_model_id=default_gravity_model_id,
                default_sensor_suite_id=default_sensor_suite_id,
            )
            for item in resolved
        }
        self._metadata = TrajectoryProviderMetadata(
            id=PROVIDER_ID,
            name="TAORYX Parametric Interceptors",
            version=provider_version,
            description=(
                "Developer-facing, evidence-aware SAM/interceptor surrogates for compact public or synthetic profiles. "
                "Create a profile in Python, flat YAML, or catalogue YAML; the provider resolves it into selectable "
                "point-mass or attitude-response pseudo-6DOF Mission Composition models with standard controls, "
                "environment, gravity, and sensors."
            ),
            presentation=TrajectoryProviderPresentationMetadata(
                display_name="Parametric Interceptors",
                short_name="Interceptor Surrogates",
                summary=(
                    "Create and integrate provenance-preserving, low-fidelity SAM/interceptor surrogates without "
                    "writing a vehicle-specific runtime."
                ),
                organization="TAORYX",
                categories=("Parametric Models", "Interceptors", "Low Fidelity", "Developer Tools"),
                links=(
                    PresentationLinkMetadata(
                        relation="documentation",
                        label="Parametric interceptor developer guide",
                        uri="packages/taoryx-parametric-interceptors/docs/model-architecture.md",
                        media_type="text/markdown",
                    ),
                    PresentationLinkMetadata(
                        relation="documentation",
                        label="Profile quickstart and examples",
                        uri="packages/taoryx-parametric-interceptors/README.md",
                        media_type="text/markdown",
                    ),
                ),
            ),
            status="runnable_workflow",
            tags=(
                "parametric",
                "interceptor",
                "sam",
                "point-mass",
                "pseudo-6dof",
                "evidence-aware",
                "scenario-calibration-screen",
                "bounded-parameter-fit",
                "response-analysis",
                "live-waypoint-session",
            ),
            execution_contract="taoryx.parametric-interceptors.batch/v1",
            model_count=len(resolved),
            provenance="taoryx-parametric-interceptors resolver, neutral kernels, and registered Taoryx sensor suites",
            claim_boundary=(
                "Resolved profiles are surrogate simulation cases. Archetype defaults and calibrated or derived values "
                "remain distinguishable from observations; no model is a weapon-performance or controller-qualification claim."
            ),
        )
        ####

    @classmethod
    def from_profiles(
        cls,
        *profiles: InterceptorEvidenceProfile | ResolvedInterceptorProfile,
        assumption_case: AssumptionCase | str | None = None,
        environment_models: Mapping[str, EnvironmentProvider] | None = None,
        gravity_models: Mapping[str, Callable[[float], float]] | None = None,
        sensor_suites: Mapping[str, InterceptorSensorSuite] | None = None,
        default_environment_model_id: str = DEFAULT_ENVIRONMENT_MODEL_ID,
        default_gravity_model_id: str = DEFAULT_GRAVITY_MODEL_ID,
        default_sensor_suite_id: str = DEFAULT_SENSOR_SUITE_ID,
    ) -> ParametricInterceptorMissionCompositionProvider:
        """Ergonomic constructor for application and notebook code."""

        return cls(
            profiles,
            assumption_case=assumption_case,
            environment_models=environment_models,
            gravity_models=gravity_models,
            sensor_suites=sensor_suites,
            default_environment_model_id=default_environment_model_id,
            default_gravity_model_id=default_gravity_model_id,
            default_sensor_suite_id=default_sensor_suite_id,
        )
        ####

    @classmethod
    def from_yaml(
        cls,
        *paths: str | Path,
        assumption_case: AssumptionCase | str | None = None,
        environment_models: Mapping[str, EnvironmentProvider] | None = None,
        gravity_models: Mapping[str, Callable[[float], float]] | None = None,
        sensor_suites: Mapping[str, InterceptorSensorSuite] | None = None,
        default_environment_model_id: str = DEFAULT_ENVIRONMENT_MODEL_ID,
        default_gravity_model_id: str = DEFAULT_GRAVITY_MODEL_ID,
        default_sensor_suite_id: str = DEFAULT_SENSOR_SUITE_ID,
    ) -> ParametricInterceptorMissionCompositionProvider:
        """Load one or more copy-ready YAML profiles into a provider."""

        if not paths:
            raise ValueError("at least one interceptor profile path is required")
        return cls.from_profiles(
            *(load_interceptor_profile(path) for path in paths),
            assumption_case=assumption_case,
            environment_models=environment_models,
            gravity_models=gravity_models,
            sensor_suites=sensor_suites,
            default_environment_model_id=default_environment_model_id,
            default_gravity_model_id=default_gravity_model_id,
            default_sensor_suite_id=default_sensor_suite_id,
        )
        ####

    @classmethod
    def from_catalogue_yaml(
        cls,
        *paths: str | Path,
        assumption_case: AssumptionCase | str | None = None,
        environment_models: Mapping[str, EnvironmentProvider] | None = None,
        gravity_models: Mapping[str, Callable[[float], float]] | None = None,
        sensor_suites: Mapping[str, InterceptorSensorSuite] | None = None,
        default_environment_model_id: str = DEFAULT_ENVIRONMENT_MODEL_ID,
        default_gravity_model_id: str = DEFAULT_GRAVITY_MODEL_ID,
        default_sensor_suite_id: str = DEFAULT_SENSOR_SUITE_ID,
    ) -> ParametricInterceptorMissionCompositionProvider:
        """Load nested ontology evidence records directly into a provider."""

        if not paths:
            raise ValueError("at least one catalogue interceptor record path is required")
        return cls.from_profiles(
            *(load_catalogue_interceptor_record(path) for path in paths),
            assumption_case=assumption_case,
            environment_models=environment_models,
            gravity_models=gravity_models,
            sensor_suites=sensor_suites,
            default_environment_model_id=default_environment_model_id,
            default_gravity_model_id=default_gravity_model_id,
            default_sensor_suite_id=default_sensor_suite_id,
        )
        ####

    @property
    def metadata(self) -> TrajectoryProviderMetadata:
        return self._metadata
        ####

    def list_models(self) -> tuple[TrajectoryModelMetadata, ...]:
        return tuple(self._models.values())
        ####

    def model(self, model_id: str) -> TrajectoryModelMetadata:
        try:
            return self._models[model_id]
        except KeyError as error:
            raise KeyError(f"unknown parametric interceptor model {model_id!r}") from error
        ####

    def resolved_profile(self, model_id: str) -> ResolvedInterceptorProfile:
        """Return the complete parameter/evidence record advertised by a model."""

        self.model(model_id)
        return self._profiles[model_id]
        ####

    def reported_calibration_scenario(
        self,
        model_id: str,
        scenario_id: str,
        *,
        scenario_basis: str,
        mission: PointMassMission | None = None,
        fidelity: CalibrationFidelity = "point_mass_3dof",
        relative_tolerance: float = 0.20,
        include: Sequence[str] = (
            "reported_max_speed_mps",
            "reported_max_altitude_m",
            "reported_max_range_m",
        ),
        severity: Literal["required", "advisory"] = "advisory",
    ) -> InterceptorCalibrationScenario:
        """Build a scenario from this provider's available reported evidence."""

        sensor_suite = self._sensor_suites[self._default_sensor_suite_id]
        return calibration_scenario_from_reported_profile(
            self.resolved_profile(model_id),
            scenario_id,
            scenario_basis=scenario_basis,
            mission=mission,
            fidelity=fidelity,
            relative_tolerance=relative_tolerance,
            include=include,
            severity=severity,
            environment_model_id=self._default_environment_model_id,
            gravity_model_id=self._default_gravity_model_id,
            sensor_suite_id=sensor_suite.id,
            sensor_suite_version=sensor_suite.version,
            sensor_suite_fingerprint=sensor_suite.fingerprint,
        )
        ####

    def evaluate_calibration(
        self,
        model_id: str,
        scenario: InterceptorCalibrationScenario,
    ) -> InterceptorCalibrationResult:
        """Evaluate one provider model through the typed calibration boundary."""

        try:
            environment = self._environment_models[scenario.environment_model_id]
        except KeyError as error:
            raise KeyError(f"unregistered calibration environment model {scenario.environment_model_id!r}") from error
        try:
            gravity_acceleration = self._gravity_models[scenario.gravity_model_id]
        except KeyError as error:
            raise KeyError(f"unregistered calibration gravity model {scenario.gravity_model_id!r}") from error
        try:
            sensor_suite = self._sensor_suites[scenario.sensor_suite_id]
        except KeyError as error:
            raise KeyError(f"unregistered calibration sensor suite {scenario.sensor_suite_id!r}") from error
        return evaluate_interceptor_calibration(
            self.resolved_profile(model_id),
            scenario,
            environment=environment,
            gravity_acceleration=gravity_acceleration,
            sensor_suite=sensor_suite,
        )
        ####

    def analyze_pseudo6_response(
        self,
        model_id: str,
        request: Pseudo6ResponseAnalysisRequest | None = None,
    ) -> Pseudo6ResponseAnalysis:
        """Inspect frozen-support continuous and implemented-discrete response poles."""

        return analyze_pseudo6_response(self.resolved_profile(model_id), request)
        ####

    def analyze_pseudo6_response_at_operating_point(
        self,
        model_id: str,
        operating_point: Pseudo6ResponseOperatingPoint,
        request: Pseudo6OperatingPointAnalysisRequest | None = None,
    ) -> Pseudo6OperatingPointResponseAnalysis:
        """Resolve runtime force support and inspect the local response law."""

        return analyze_pseudo6_response_at_operating_point(
            self.resolved_profile(model_id),
            operating_point,
            request,
        )
        ####

    def compare_pseudo6_responses(
        self,
        baseline_model_id: str,
        candidate_model_id: str,
        request: Pseudo6ResponseAnalysisRequest | None = None,
    ) -> Pseudo6ResponseComparison:
        """Compare two models under one identical response-analysis request."""

        return compare_pseudo6_responses(
            self.resolved_profile(baseline_model_id),
            self.resolved_profile(candidate_model_id),
            request,
        )
        ####

    def compare_pseudo6_responses_at_operating_point(
        self,
        baseline_model_id: str,
        candidate_model_id: str,
        operating_point: Pseudo6ResponseOperatingPoint,
        request: Pseudo6OperatingPointAnalysisRequest | None = None,
    ) -> Pseudo6OperatingPointResponseComparison:
        """Compare local response tuning and independently resolved force support."""

        return compare_pseudo6_responses_at_operating_point(
            self.resolved_profile(baseline_model_id),
            self.resolved_profile(candidate_model_id),
            operating_point,
            request,
        )
        ####

    def run_pseudo6_attitude_step(
        self,
        model_id: str,
        *,
        axis: Pseudo6ResponseAxis = "roll",
        command_step_rad: float = 0.17453292519943295,
        duration_s: float = 3.0,
        time_step_s: float = 0.01,
        settling_band_fraction: float = 0.02,
        command_support_fraction: float = 1.0,
    ) -> Pseudo6AttitudeStepRun:
        """Execute the bounded response-law witness for one provider model."""

        return run_pseudo6_attitude_step(
            self.resolved_profile(model_id),
            axis=axis,
            command_step_rad=command_step_rad,
            duration_s=duration_s,
            time_step_s=time_step_s,
            settling_band_fraction=settling_band_fraction,
            command_support_fraction=command_support_fraction,
        )
        ####

    def get_model_schema(self, model_id: str) -> TrajectoryConfigurationSchema:
        self.model(model_id)
        return self._schemas[model_id]
        ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        return self.model(model_id).output_schema
        ####

    def configuration(
        self,
        model_id: str,
        *,
        configuration_id: str = "parametric-interceptor-run",
        fidelity: str = POINT_MASS_FIDELITY_ID,
        mission_template_id: str = MISSION_TEMPLATE_ID,
        startup_authority_profile_id: str | None = None,
        **overrides: float | str,
    ) -> TrajectoryConfigurationInstance:
        """Build a common configuration without manually constructing its AST.

        Override keys are the exact advertised parameter IDs. For ordinary
        Python call sites, underscores may replace dots.
        """

        model = self.model(model_id)
        schema = self.get_model_schema(model_id)
        realizations = {
            POINT_MASS_FIDELITY_ID: POINT_MASS_REALIZATION_ID,
            PSEUDO6_FIDELITY_ID: PSEUDO6_REALIZATION_ID,
        }
        if fidelity not in realizations:
            raise KeyError(f"unknown fidelity {fidelity!r}; expected one of {tuple(realizations)!r}")
        if mission_template_id not in _MISSION_TEMPLATE_IDS:
            raise KeyError(f"unknown mission template {mission_template_id!r}; expected one of {_MISSION_TEMPLATE_IDS!r}")
        values: dict[str, float | str] = dict(_DEFAULTS)
        values["runtime.environment_model_id"] = self._default_environment_model_id
        values["runtime.gravity_model_id"] = self._default_gravity_model_id
        values["runtime.sensor_suite_id"] = self._default_sensor_suite_id
        aliases = {name.replace(".", "_"): name for name in values}
        for supplied, value in overrides.items():
            key = supplied if supplied in values else aliases.get(supplied)
            if key is None:
                raise KeyError(f"unknown mission parameter {supplied!r}; expected one of {tuple(values)!r}")
            values[key] = value
        return TrajectoryConfigurationInstance(
            configuration_id=configuration_id,
            model_id=model.id,
            model_version=model.version,
            schema_fingerprint=schema.fingerprint,
            fidelity=fidelity,
            realization_id=realizations[fidelity],
            mission_template_id=mission_template_id,
            startup_authority_profile_id=startup_authority_profile_id,
            root=ConfigurationGroupValue(values={name: ConfigurationParameterValue(value=value, unit=_parameter_unit(name)) for name, value in values.items()}),
        )
        ####

    def build_model_default_configuration(
        self,
        model_id: str,
        *,
        configuration_id: str,
    ) -> TrajectoryConfigurationInstance:
        """Return the resolved profile's deterministic waypoint-intercept example."""

        return self.configuration(model_id, configuration_id=configuration_id)
        ####

    def configuration_from_mapping(
        self,
        model_id: str,
        values: Mapping[str, float | str],
        *,
        configuration_id: str = "parametric-interceptor-run",
        fidelity: str = POINT_MASS_FIDELITY_ID,
        mission_template_id: str = MISSION_TEMPLATE_ID,
        startup_authority_profile_id: str | None = None,
    ) -> TrajectoryConfigurationInstance:
        """Build a configuration from agent/CLI-friendly parameter mappings."""

        return self.configuration(
            model_id,
            configuration_id=configuration_id,
            fidelity=fidelity,
            mission_template_id=mission_template_id,
            startup_authority_profile_id=startup_authority_profile_id,
            **dict(values),
        )
        ####

    def validate_configuration(
        self,
        configuration: TrajectoryConfigurationInstance,
    ) -> PreparedTrajectoryConfiguration:
        schema = self.get_model_schema(configuration.model_id)
        realization_by_fidelity = {
            POINT_MASS_FIDELITY_ID: POINT_MASS_REALIZATION_ID,
            PSEUDO6_FIDELITY_ID: PSEUDO6_REALIZATION_ID,
        }
        expected_realization = realization_by_fidelity.get(configuration.fidelity)
        if expected_realization is None:
            raise ConfigurationContractError(
                "unknown-fidelity",
                f"expected one of {tuple(realization_by_fidelity)!r}",
                path="configuration.fidelity",
            )
        if configuration.realization_id not in {None, expected_realization}:
            raise ConfigurationContractError(
                "unknown-realization",
                f"fidelity {configuration.fidelity!r} requires {expected_realization!r}",
                path="configuration.realization_id",
            )
        if configuration.mission_template_id not in {
            None,
            *_MISSION_TEMPLATE_IDS,
        }:
            raise ConfigurationContractError(
                "unknown-mission-template",
                f"expected one of {_MISSION_TEMPLATE_IDS!r}",
                path="configuration.mission_template_id",
            )
        mission_template_id = configuration.mission_template_id or MISSION_TEMPLATE_ID
        if mission_template_id == TARGET_TRACK_MISSION_TEMPLATE_ID:
            supported_authorities = {None, "target_track_guidance", "live_target_track_guidance"}
        elif mission_template_id == DIRECT_ACCELERATION_MISSION_TEMPLATE_ID:
            supported_authorities = {None, "direct_lateral_acceleration", "live_direct_lateral_acceleration"}
        else:
            supported_authorities = {None, "waypoint_guidance", "live_waypoint_guidance"}
        if configuration.startup_authority_profile_id not in supported_authorities:
            raise ConfigurationContractError(
                "unknown-control-authority",
                (
                    f"fidelity {configuration.fidelity!r} supports startup authorities "
                    f"{tuple(sorted(item for item in supported_authorities if item is not None))!r}"
                ),
                path="configuration.startup_authority_profile_id",
            )
        prepared = validate_configuration_instance(schema, configuration)
        _validate_mission_control_scope(
            prepared.resolved,
            mission_template_id=mission_template_id,
        )
        if configuration.fidelity == PSEUDO6_FIDELITY_ID:
            mission = _mission_from_resolved(
                prepared.resolved,
                mission_template_id=mission_template_id,
            )
            response = self.analyze_pseudo6_response(
                configuration.model_id,
                Pseudo6ResponseAnalysisRequest(
                    analysis_id="configuration-discretization-preflight",
                    sample_time_s=mission.time_step_s,
                ),
            )
            if response.discrete_status == "unstable":
                raise ConfigurationContractError(
                    "unstable-response-discretization",
                    (
                        f"time step {mission.time_step_s:g} s gives response-law spectral radius "
                        f"{response.discrete_spectral_radius:.6g}; choose a value below the local stability "
                        f"limit {response.maximum_stable_sample_time_s:.6g} s"
                    ),
                    path="configuration.root.runtime.time_step_s",
                )
        return prepared
        ####

    def build_runner(self) -> MissionCompositionRunnerRegistry:
        return MissionCompositionRunnerRegistry({(PROVIDER_ID, model_id): self._execute_batch for model_id in self._models})
        ####

    def open_session_episode(
        self,
        prepared: PreparedTrajectoryConfiguration,
        *,
        seed: int | None = None,
        integration_step_s: float = 0.02,
    ) -> FixtureCompositionEpisode:
        """Open the selected live waypoint, target-track, or direct-control episode."""

        configuration = prepared.configuration
        model = self.model(configuration.model_id)
        sensor_suite = self._selected_sensor_suite(prepared)
        if configuration.fidelity == PSEUDO6_FIDELITY_ID:
            return _open_pseudo6_session_episode(
                model,
                self._profiles[model.id],
                prepared,
                seed=seed,
                integration_step_s=integration_step_s,
                environment=self._selected_environment(prepared),
                gravity_acceleration=self._selected_gravity(prepared),
                sensor_suite=sensor_suite,
                environment_model_id=_resolved_text(prepared.resolved, "runtime.environment_model_id"),
                gravity_model_id=_resolved_text(prepared.resolved, "runtime.gravity_model_id"),
            )
        if configuration.fidelity != POINT_MASS_FIDELITY_ID:
            raise ValueError(f"unsupported stateful fidelity {configuration.fidelity!r}")
        return _open_point_mass_session_episode(
            model,
            self._profiles[model.id],
            prepared,
            seed=seed,
            integration_step_s=integration_step_s,
            environment=self._selected_environment(prepared),
            gravity_acceleration=self._selected_gravity(prepared),
            sensor_suite=sensor_suite,
            environment_model_id=_resolved_text(prepared.resolved, "runtime.environment_model_id"),
            gravity_model_id=_resolved_text(prepared.resolved, "runtime.gravity_model_id"),
        )
        ####

    def _execute_batch(self, request: MissionCompositionRunRequest) -> MissionCompositionTrajectoryResult:
        if request.provider_id != self.metadata.id or request.provider_version != self.metadata.version:
            raise _execution_error("provider-version-mismatch", "request provider identity does not match this provider", request.model_id)
        model = self.model(request.model_id)
        prepared = self.validate_configuration(request.prepared_configuration.configuration)
        if prepared.fingerprint != request.prepared_configuration.fingerprint:
            raise _execution_error("prepared-configuration-stale", "prepared configuration fingerprint is stale", model.id)
        mission = _mission_from_resolved(
            prepared.resolved,
            mission_template_id=prepared.configuration.mission_template_id or MISSION_TEMPLATE_ID,
        )
        environment = self._selected_environment(prepared)
        gravity_acceleration = self._selected_gravity(prepared)
        sensor_suite = self._selected_sensor_suite(prepared)
        if prepared.configuration.fidelity == PSEUDO6_FIDELITY_ID:
            run: PointMassRun | Pseudo6Run = run_pseudo6_interceptor(
                self._profiles[model.id],
                mission,
                environment=environment,
                gravity_acceleration=gravity_acceleration,
                imu_adapter=sensor_suite.build_pseudo6_sensor(seed=0),
                target_track_adapter=sensor_suite.build_target_track_sensor(seed=0),
            )
        else:
            run = run_point_mass_interceptor(
                self._profiles[model.id],
                mission,
                environment=environment,
                gravity_acceleration=gravity_acceleration,
                translation_acceleration_adapter=sensor_suite.build_point_mass_sensor(seed=0),
                target_track_adapter=sensor_suite.build_target_track_sensor(seed=0),
            )
        return _trajectory_result(
            request,
            model,
            run,
            fidelity=prepared.configuration.fidelity,
            sensor_suite=sensor_suite,
        )
        ####

    def _selected_environment(self, prepared: PreparedTrajectoryConfiguration) -> EnvironmentProvider | None:
        return self._environment_models[_resolved_text(prepared.resolved, "runtime.environment_model_id")]
        ####

    def _selected_gravity(self, prepared: PreparedTrajectoryConfiguration) -> Callable[[float], float] | None:
        return self._gravity_models[_resolved_text(prepared.resolved, "runtime.gravity_model_id")]
        ####

    def _selected_sensor_suite(self, prepared: PreparedTrajectoryConfiguration) -> InterceptorSensorSuite:
        return self._sensor_suites[_resolved_text(prepared.resolved, "runtime.sensor_suite_id")]
        ####

    ####


def _open_pseudo6_session_episode(
    model: TrajectoryModelMetadata,
    profile: ResolvedInterceptorProfile,
    prepared: PreparedTrajectoryConfiguration,
    *,
    seed: int | None,
    integration_step_s: float,
    environment: EnvironmentProvider | None,
    gravity_acceleration: Callable[[float], float] | None,
    sensor_suite: InterceptorSensorSuite,
    environment_model_id: str,
    gravity_model_id: str,
) -> FixtureCompositionEpisode:
    """Build a checkpoint-safe pseudo-6DOF session around shared equations."""

    response = analyze_pseudo6_response(
        profile,
        Pseudo6ResponseAnalysisRequest(
            analysis_id="session-discretization-preflight",
            sample_time_s=integration_step_s,
        ),
    )
    if response.discrete_status == "unstable":
        raise ValueError(
            f"session integration step {integration_step_s:g} s gives pseudo-6DOF response-law "
            f"spectral radius {response.discrete_spectral_radius:.6g}; choose a value below "
            f"{response.maximum_stable_sample_time_s:.6g} s"
        )
    mission_template_id = prepared.configuration.mission_template_id or MISSION_TEMPLATE_ID
    mission = _mission_from_resolved(
        prepared.resolved,
        mission_template_id=mission_template_id,
    )
    kernel: Pseudo6Kernel | None = None
    contract, observation_schema = build_session_interface_contract(
        model,
        realization_id=PSEUDO6_REALIZATION_ID,
        fidelity=PSEUDO6_FIDELITY_ID,
        family_id="parametric_interceptor",
        physical_family="interceptor",
        claim_boundary=(
            "Stateful attitude-response pseudo-6DOF surrogate using the same bounded response law, standard "
            "environment boundary, propulsion program, and selected registered Taoryx sensor suite as batch execution. "
            "A target mission guides from registered direct-geometry relative-state packets and evaluates capture from "
            "target truth. A direct-acceleration mission accepts a local-NEU request, projects it transverse to velocity, "
            "and applies the existing bounded attitude and force-authority response. It does not claim rigid-body "
            "dynamics, propagation, signatures, gimbals, a track manager, physical seeker, datalink, actuator, "
            "autopilot, or weapon performance."
        ),
        default_authority_profile_id=_session_authority_id(mission_template_id),
    )

    def initial_state_factory(reset_seed: int | None) -> Mapping[str, object]:
        nonlocal kernel
        kernel = Pseudo6Kernel(
            profile,
            environment=environment,
            gravity_acceleration=gravity_acceleration,
            imu_adapter=sensor_suite.build_pseudo6_sensor(seed=reset_seed or 0),
            target_track_adapter=sensor_suite.build_target_track_sensor(seed=reset_seed or 0),
        )
        state = kernel.initial_state(mission)
        objective = mission.guidance_objective()
        initial_evaluation = kernel.evaluate_committed(state, objective)
        sample = kernel.sample(initial_evaluation)
        observation = {channel.name: _sample_value(sample, channel.name) for channel in observation_schema}
        observation["guidance.objective.capture_occurred"] = initial_evaluation.waypoint_captured
        payload = _pseudo6_state_mapping(state)
        payload.update(
            {
                "waypoint_capture_active": initial_evaluation.waypoint_captured,
                "guidance_objective_capture_latched": initial_evaluation.waypoint_captured,
                "imu_checkpoint": kernel.sensor_checkpoint(),
                "session_observation": observation,
            }
        )
        payload.update(_session_objective_state(objective))
        return payload
        ####

    def observation_factory(
        state: Mapping[str, object],
        _: float,
        __: EpisodeStatus,
    ) -> Mapping[str, object]:
        values = state.get("session_observation")
        if not isinstance(values, Mapping):
            raise ValueError("pseudo-6DOF session state is missing its committed observation")
        return dict(values)
        ####

    def transition(
        state: Mapping[str, object],
        authority_profile_id: str,
        action: Mapping[str, object],
        duration_s: float,
        _: float,
    ) -> FixtureTransition:
        expected_authority_id = _session_authority_id(mission_template_id)
        if authority_profile_id != expected_authority_id:
            raise ValueError(f"unsupported pseudo-6DOF session authority {authority_profile_id!r}")
        next_state = dict(state)
        if kernel is None:
            raise RuntimeError("pseudo-6DOF session sensor suite has not been initialized")
        sensor_checkpoint = next_state.get("imu_checkpoint")
        if not isinstance(sensor_checkpoint, Mapping):
            raise ValueError("pseudo-6DOF session state is missing its IMU checkpoint")
        kernel.restore_sensor(sensor_checkpoint)
        pseudo_state = _pseudo6_state_from_mapping(next_state)
        objective, accepted, retargeted = _accept_session_objective(
            next_state,
            action,
            mission_template_id=mission_template_id,
            time_s=pseudo_state.time_s,
        )
        events: list[str] = []
        if retargeted:
            events.append(_session_update_event(mission_template_id))
        capture_event = "target_intercept" if mission_template_id == TARGET_TRACK_MISSION_TEMPLATE_ID else "waypoint_captured"
        capture_active = bool(next_state.get("waypoint_capture_active", False))
        capture_latched = bool(next_state.get("guidance_objective_capture_latched", capture_active))
        if retargeted:
            capture_active = False
            capture_latched = False
        capture_event_times_s: list[float] = []
        ground_impact = False
        remaining_s = duration_s
        final = kernel.evaluate_committed(pseudo_state, objective) if retargeted else kernel.evaluate_held(pseudo_state, objective)
        final_sample: Pseudo6Sample | None = None
        while remaining_s > 1.0e-12:
            evaluation = final
            if evaluation.waypoint_captured and not capture_latched:
                events.append(capture_event)
                capture_event_times_s.append(pseudo_state.time_s)
                capture_latched = True
            capture_active = evaluation.waypoint_captured
            ground_impact = ground_impact or evaluation.ground_impact
            step_s = min(integration_step_s, remaining_s)
            localized_event = kernel.localize_step_event(pseudo_state, evaluation, objective, step_s)
            if localized_event is not None and localized_event.kind == "objective_capture" and not capture_latched:
                events.append(capture_event)
                capture_event_times_s.append(pseudo_state.time_s + localized_event.time_from_step_start_s)
                capture_latched = True
            pseudo_state = kernel.advance(pseudo_state, evaluation, step_s)
            final = kernel.evaluate_committed(pseudo_state, objective)
            final_sample = kernel.sample(final)
            remaining_s = max(0.0, remaining_s - step_s)
        if final_sample is None:
            raise RuntimeError("pseudo-6DOF session transition produced no committed sample")
        if final.waypoint_captured and not capture_latched:
            events.append(capture_event)
            capture_event_times_s.append(pseudo_state.time_s)
            capture_latched = True
        capture_active = final.waypoint_captured
        ground_impact = ground_impact or final.ground_impact
        next_state.update(_pseudo6_state_mapping(pseudo_state))
        next_state["waypoint_capture_active"] = capture_active
        next_state["guidance_objective_capture_latched"] = capture_latched
        next_state["imu_checkpoint"] = kernel.sensor_checkpoint()
        observation = {channel.name: _sample_value(final_sample, channel.name) for channel in observation_schema}
        observation["guidance.objective.capture_occurred"] = capture_latched
        next_state["session_observation"] = observation
        return FixtureTransition(
            state=next_state,
            applied_action=_applied_objective_mapping(objective),
            applied_semantic_action=accepted,
            lowering_evidence={
                "authority_profile_id": authority_profile_id,
                "lowering_chain": [
                    *_session_lowering_prefix(mission_template_id),
                    "bounded_attitude_response",
                    "pseudo6_state_advance",
                    "taoryx_registered_imu_projection",
                ],
                "accepted_guidance_objective": _objective_lowering_mapping(objective),
                "target_position_m": {
                    "north": final.target_position_m[0],
                    "east": final.target_position_m[1],
                    "altitude": final.target_position_m[2],
                },
                "relative_velocity_mps": {
                    "north": final.relative_velocity_mps[0],
                    "east": final.relative_velocity_mps[1],
                    "vertical": final.relative_velocity_mps[2],
                },
                "time_to_closest_approach_s": final.time_to_closest_approach_s,
                "predicted_miss_distance_m": final.predicted_miss_distance_m,
                "objective_capture_occurred": capture_latched,
                "objective_capture_event_times_s": capture_event_times_s,
                "waypoint_range_m": final.waypoint_range_m,
                "applicability_declared": final.applicability_declared,
                "applicability_status": final.applicability_status,
                "applicability_reason": final.applicability_reason,
                "applicability_enforcement": APPLICABILITY_ENFORCEMENT,
                "guidance_available": final.guidance_available,
                "guidance_archetype": final.guidance_archetype,
                "guidance_mode": final.guidance_mode,
                "guidance_closing_speed_mps": final.guidance_closing_speed_mps,
                "guidance_line_of_sight_rate_rad_s": final.guidance_line_of_sight_rate_rad_s,
                "guidance_navigation_constant": final.guidance_navigation_constant,
                "lateral_acceleration_command_mps2": final.lateral_acceleration_command_mps2,
                "lateral_acceleration_achieved_mps2": final.lateral_acceleration_achieved_mps2,
                "lateral_acceleration_command_vector_mps2": {
                    "north": final.lateral_acceleration_command_vector_mps2[0],
                    "east": final.lateral_acceleration_command_vector_mps2[1],
                    "vertical": final.lateral_acceleration_command_vector_mps2[2],
                },
                "lateral_acceleration_achieved_vector_mps2": {
                    "north": final.lateral_acceleration_achieved_vector_mps2[0],
                    "east": final.lateral_acceleration_achieved_vector_mps2[1],
                    "vertical": final.lateral_acceleration_achieved_vector_mps2[2],
                },
                "lateral_acceleration_achievement_fraction": final.lateral_acceleration_achievement_fraction,
                "lateral_acceleration_direction_error_valid": final.lateral_acceleration_direction_error_valid,
                "lateral_acceleration_direction_error_rad": final.lateral_acceleration_direction_error_rad,
                "lateral_acceleration_limit_mps2": final.lateral_acceleration_limit_mps2,
                "lateral_acceleration_utilization": final.lateral_acceleration_utilization,
                "control_configuration": final.control_configuration,
                "aerodynamic_lateral_authority_mps2": final.aerodynamic_lateral_authority_mps2,
                "thrust_vector_lateral_authority_mps2": final.thrust_vector_lateral_authority_mps2,
                "combined_lateral_authority_mps2": final.combined_lateral_authority_mps2,
                "lateral_acceleration_available_mps2": final.lateral_acceleration_available_mps2,
                "lateral_acceleration_authority_utilization": final.lateral_acceleration_authority_utilization,
                "control_authority_structural_limit_active": final.control_authority_structural_limit_active,
                "attitude_response_authority_available": final.attitude_response_authority_available,
                "attitude_response_command_support_fraction": final.attitude_response_command_support_fraction,
                "attitude_response_authority_limited": final.attitude_response_authority_limited,
                "control_allocation_policy": final.control_allocation_policy,
                "aerodynamic_lateral_acceleration_achieved_mps2": final.aerodynamic_lateral_acceleration_achieved_mps2,
                "thrust_vector_lateral_acceleration_achieved_mps2": final.thrust_vector_lateral_acceleration_achieved_mps2,
                "thrust_vector_angle_achieved_rad": final.thrust_vector_angle_achieved_rad,
                "axial_thrust_n": final.axial_thrust_n,
                "base_drag_n": final.base_drag_n,
                "maneuver_drag_n": final.maneuver_drag_n,
                "total_drag_n": final.drag_n,
                "flow_angles_valid": final.flow_angles_valid,
                "air_relative_velocity_body_mps": {
                    "x": final.air_relative_velocity_body_mps[0],
                    "y": final.air_relative_velocity_body_mps[1],
                    "z": final.air_relative_velocity_body_mps[2],
                },
                "angle_of_attack_rad": final.angle_of_attack_rad,
                "sideslip_angle_rad": final.sideslip_angle_rad,
                "control_limited": final.control_limited,
                "control_limit_reason": final.control_limit_reason,
                "imu_valid": final_sample.imu_valid,
                "imu_interval_s": final_sample.imu_interval_s,
                "native_transition": "Pseudo6Kernel.evaluate+advance+sample",
                "sensor_suite_id": sensor_suite.id,
                "sensor_suite_version": sensor_suite.version,
                "sensor_suite_fingerprint": sensor_suite.fingerprint,
                "sensor_provider_kind": sensor_suite.pseudo6_provider.kind,
                "target_track_sensor_provider_kind": sensor_suite.target_track_provider.kind,
                "target_track_sensor_schema_id": final_sample.target_track_schema_id,
                "target_track_valid": final_sample.target_track_valid,
                "target_track_invalid_reason": final_sample.target_track_invalid_reason,
                "environment_model_id": environment_model_id,
                "gravity_model_id": gravity_model_id,
            },
            events=tuple(events),
            diagnostics=(("pseudo-6DOF surrogate crossed the local altitude floor",) if ground_impact else ()),
            status="completed" if ground_impact else "active",
        )
        ####

    return FixtureCompositionEpisode(
        interface_contract=contract,
        observation_schema=observation_schema,
        initial_state_factory=initial_state_factory,
        observation_factory=observation_factory,
        transition=transition,
        claim_boundary=contract.claim_boundary,
        seed=seed,
    )
    ####


def _open_point_mass_session_episode(
    model: TrajectoryModelMetadata,
    profile: ResolvedInterceptorProfile,
    prepared: PreparedTrajectoryConfiguration,
    *,
    seed: int | None,
    integration_step_s: float,
    environment: EnvironmentProvider | None,
    gravity_acceleration: Callable[[float], float] | None,
    sensor_suite: InterceptorSensorSuite,
    environment_model_id: str,
    gravity_model_id: str,
) -> FixtureCompositionEpisode:
    """Build one standard session around the exact point-mass batch equations."""

    mission_template_id = prepared.configuration.mission_template_id or MISSION_TEMPLATE_ID
    mission = _mission_from_resolved(
        prepared.resolved,
        mission_template_id=mission_template_id,
    )
    kernel: PointMassKernel | None = None
    contract, observation_schema = build_session_interface_contract(
        model,
        realization_id=POINT_MASS_REALIZATION_ID,
        fidelity=POINT_MASS_FIDELITY_ID,
        family_id="parametric_interceptor",
        physical_family="interceptor",
        claim_boundary=(
            "Stateful point-mass surrogate using the same resolved profile, standard environment boundary, "
            "propulsion schedule, semi-implicit transition, and selected registered Taoryx sensor suite as batch "
            "execution. A target mission guides from registered direct-geometry relative-state packets and evaluates "
            "capture from target truth. A direct-acceleration mission accepts a local-NEU request, projects it transverse "
            "to velocity, and applies the existing force-authority allocation. It does not claim attitude, propagation, "
            "signatures, gimbals, a track manager, a physical seeker, datalink, actuator, autopilot, or weapon-performance "
            "implementation."
        ),
        default_authority_profile_id=_session_authority_id(mission_template_id),
    )

    def initial_state_factory(reset_seed: int | None) -> Mapping[str, object]:
        nonlocal kernel
        kernel = PointMassKernel(
            profile,
            environment=environment,
            gravity_acceleration=gravity_acceleration,
            translation_acceleration_adapter=sensor_suite.build_point_mass_sensor(seed=reset_seed or 0),
            target_track_adapter=sensor_suite.build_target_track_sensor(seed=reset_seed or 0),
        )
        point_state = kernel.initial_state(mission)
        objective = mission.guidance_objective()
        initial_evaluation = kernel.evaluate_committed(point_state, objective)
        sample = kernel.sample(initial_evaluation)
        observation = {channel.name: _sample_value(sample, channel.name) for channel in observation_schema}
        observation["guidance.objective.capture_occurred"] = initial_evaluation.waypoint_captured
        payload = _point_mass_state_mapping(point_state)
        payload.update(
            {
                "waypoint_capture_active": initial_evaluation.waypoint_captured,
                "guidance_objective_capture_latched": initial_evaluation.waypoint_captured,
                "translation_sensor_checkpoint": kernel.sensor_checkpoint(),
                "session_observation": observation,
            }
        )
        payload.update(_session_objective_state(objective))
        return payload
        ####

    def observation_factory(
        state: Mapping[str, object],
        _: float,
        __: EpisodeStatus,
    ) -> Mapping[str, object]:
        values = state.get("session_observation")
        if not isinstance(values, Mapping):
            raise ValueError("point-mass session state is missing its committed observation")
        return dict(values)
        ####

    def transition(
        state: Mapping[str, object],
        authority_profile_id: str,
        action: Mapping[str, object],
        duration_s: float,
        _: float,
    ) -> FixtureTransition:
        expected_authority_id = _session_authority_id(mission_template_id)
        if authority_profile_id != expected_authority_id:
            raise ValueError(f"unsupported point-mass session authority {authority_profile_id!r}")
        next_state = dict(state)
        if kernel is None:
            raise RuntimeError("point-mass session sensor suite has not been initialized")
        sensor_checkpoint = next_state.get("translation_sensor_checkpoint")
        if not isinstance(sensor_checkpoint, Mapping):
            raise ValueError("point-mass session state is missing its translation-sensor checkpoint")
        kernel.restore_sensor(sensor_checkpoint)
        point_state = _point_mass_state_from_mapping(next_state)
        objective, accepted, retargeted = _accept_session_objective(
            next_state,
            action,
            mission_template_id=mission_template_id,
            time_s=point_state.time_s,
        )
        events: list[str] = []
        if retargeted:
            events.append(_session_update_event(mission_template_id))
        capture_event = "target_intercept" if mission_template_id == TARGET_TRACK_MISSION_TEMPLATE_ID else "waypoint_captured"
        capture_active = bool(next_state.get("waypoint_capture_active", False))
        capture_latched = bool(next_state.get("guidance_objective_capture_latched", capture_active))
        if retargeted:
            capture_active = False
            capture_latched = False
        capture_event_times_s: list[float] = []
        ground_impact = False
        remaining_s = duration_s
        final = kernel.evaluate_committed(point_state, objective) if retargeted else kernel.evaluate_held(point_state, objective)
        final_sample: PointMassSample | None = None
        while remaining_s > 1.0e-12:
            evaluation = final
            if evaluation.waypoint_captured and not capture_latched:
                events.append(capture_event)
                capture_event_times_s.append(point_state.time_s)
                capture_latched = True
            capture_active = evaluation.waypoint_captured
            ground_impact = ground_impact or evaluation.ground_impact
            step_s = min(integration_step_s, remaining_s)
            localized_event = kernel.localize_step_event(point_state, evaluation, objective, step_s)
            if localized_event is not None and localized_event.kind == "objective_capture" and not capture_latched:
                events.append(capture_event)
                capture_event_times_s.append(point_state.time_s + localized_event.time_from_step_start_s)
                capture_latched = True
            point_state = kernel.advance(point_state, evaluation, step_s)
            final = kernel.evaluate_committed(point_state, objective)
            final_sample = kernel.sample(final)
            remaining_s = max(0.0, remaining_s - step_s)
        if final_sample is None:
            raise RuntimeError("point-mass session transition produced no committed sample")
        if final.waypoint_captured and not capture_latched:
            events.append(capture_event)
            capture_event_times_s.append(point_state.time_s)
            capture_latched = True
        capture_active = final.waypoint_captured
        ground_impact = ground_impact or final.ground_impact
        next_state.update(_point_mass_state_mapping(point_state))
        next_state["waypoint_capture_active"] = capture_active
        next_state["guidance_objective_capture_latched"] = capture_latched
        next_state["translation_sensor_checkpoint"] = kernel.sensor_checkpoint()
        observation = {channel.name: _sample_value(final_sample, channel.name) for channel in observation_schema}
        observation["guidance.objective.capture_occurred"] = capture_latched
        next_state["session_observation"] = observation
        return FixtureTransition(
            state=next_state,
            applied_action=_applied_objective_mapping(objective),
            applied_semantic_action=accepted,
            lowering_evidence={
                "authority_profile_id": authority_profile_id,
                "lowering_chain": [
                    *_session_lowering_prefix(mission_template_id),
                    "point_mass_state_advance",
                    "taoryx_registered_translation_acceleration_projection",
                ],
                "accepted_guidance_objective": _objective_lowering_mapping(objective),
                "target_position_m": {
                    "north": final.sample.target_north_m,
                    "east": final.sample.target_east_m,
                    "altitude": final.sample.target_altitude_m,
                },
                "relative_velocity_mps": {
                    "north": final.sample.relative_north_velocity_mps,
                    "east": final.sample.relative_east_velocity_mps,
                    "vertical": final.sample.relative_vertical_velocity_mps,
                },
                "time_to_closest_approach_s": final.sample.time_to_closest_approach_s,
                "predicted_miss_distance_m": final.sample.predicted_miss_distance_m,
                "objective_capture_occurred": capture_latched,
                "objective_capture_event_times_s": capture_event_times_s,
                "waypoint_range_m": final.sample.waypoint_range_m,
                "applicability_declared": final.sample.applicability_declared,
                "applicability_status": final.sample.applicability_status,
                "applicability_reason": final.sample.applicability_reason,
                "applicability_enforcement": APPLICABILITY_ENFORCEMENT,
                "guidance_available": final.sample.guidance_available,
                "guidance_archetype": final.sample.guidance_archetype,
                "guidance_mode": final.sample.guidance_mode,
                "guidance_closing_speed_mps": final.sample.guidance_closing_speed_mps,
                "guidance_line_of_sight_rate_rad_s": final.sample.guidance_line_of_sight_rate_rad_s,
                "guidance_navigation_constant": final.sample.guidance_navigation_constant,
                "lateral_acceleration_command_mps2": final.sample.lateral_acceleration_command_mps2,
                "lateral_acceleration_achieved_mps2": final.sample.lateral_acceleration_achieved_mps2,
                "lateral_acceleration_command_vector_mps2": {
                    "north": final.sample.lateral_acceleration_command_vector_mps2[0],
                    "east": final.sample.lateral_acceleration_command_vector_mps2[1],
                    "vertical": final.sample.lateral_acceleration_command_vector_mps2[2],
                },
                "lateral_acceleration_achieved_vector_mps2": {
                    "north": final.sample.lateral_acceleration_achieved_vector_mps2[0],
                    "east": final.sample.lateral_acceleration_achieved_vector_mps2[1],
                    "vertical": final.sample.lateral_acceleration_achieved_vector_mps2[2],
                },
                "lateral_acceleration_achievement_fraction": final.sample.lateral_acceleration_achievement_fraction,
                "lateral_acceleration_direction_error_valid": final.sample.lateral_acceleration_direction_error_valid,
                "lateral_acceleration_direction_error_rad": final.sample.lateral_acceleration_direction_error_rad,
                "lateral_acceleration_limit_mps2": final.sample.lateral_acceleration_limit_mps2,
                "lateral_acceleration_utilization": final.sample.lateral_acceleration_utilization,
                "control_configuration": final.sample.control_configuration,
                "aerodynamic_lateral_authority_mps2": final.sample.aerodynamic_lateral_authority_mps2,
                "thrust_vector_lateral_authority_mps2": final.sample.thrust_vector_lateral_authority_mps2,
                "combined_lateral_authority_mps2": final.sample.combined_lateral_authority_mps2,
                "lateral_acceleration_available_mps2": final.sample.lateral_acceleration_available_mps2,
                "lateral_acceleration_authority_utilization": final.sample.lateral_acceleration_authority_utilization,
                "control_authority_structural_limit_active": final.sample.control_authority_structural_limit_active,
                "control_allocation_policy": final.sample.control_allocation_policy,
                "aerodynamic_lateral_acceleration_achieved_mps2": final.sample.aerodynamic_lateral_acceleration_achieved_mps2,
                "thrust_vector_lateral_acceleration_achieved_mps2": final.sample.thrust_vector_lateral_acceleration_achieved_mps2,
                "thrust_vector_angle_achieved_rad": final.sample.thrust_vector_angle_achieved_rad,
                "axial_thrust_n": final.sample.axial_thrust_n,
                "base_drag_n": final.sample.base_drag_n,
                "maneuver_drag_n": final.sample.maneuver_drag_n,
                "total_drag_n": final.sample.drag_n,
                "control_limited": final.sample.control_limited,
                "control_limit_reason": final.sample.control_limit_reason,
                "translation_acceleration_valid": final_sample.translation_acceleration_valid,
                "translation_acceleration_interval_s": final_sample.translation_acceleration_interval_s,
                "native_transition": "PointMassKernel.evaluate+advance+sample",
                "sensor_suite_id": sensor_suite.id,
                "sensor_suite_version": sensor_suite.version,
                "sensor_suite_fingerprint": sensor_suite.fingerprint,
                "sensor_provider_kind": sensor_suite.point_mass_provider.kind,
                "target_track_sensor_provider_kind": sensor_suite.target_track_provider.kind,
                "target_track_sensor_schema_id": final_sample.target_track_schema_id,
                "target_track_valid": final_sample.target_track_valid,
                "target_track_invalid_reason": final_sample.target_track_invalid_reason,
                "environment_model_id": environment_model_id,
                "gravity_model_id": gravity_model_id,
            },
            events=tuple(events),
            diagnostics=(("point-mass surrogate crossed the local altitude floor",) if ground_impact else ()),
            status="completed" if ground_impact else "active",
        )
        ####

    return FixtureCompositionEpisode(
        interface_contract=contract,
        observation_schema=observation_schema,
        initial_state_factory=initial_state_factory,
        observation_factory=observation_factory,
        transition=transition,
        claim_boundary=contract.claim_boundary,
        seed=seed,
    )
    ####


def _pseudo6_state_mapping(state: Pseudo6State) -> dict[str, object]:
    return {
        "time_s": state.time_s,
        "north_m": state.north_m,
        "east_m": state.east_m,
        "altitude_m": state.altitude_m,
        "north_velocity_mps": state.north_velocity_mps,
        "east_velocity_mps": state.east_velocity_mps,
        "vertical_velocity_mps": state.vertical_velocity_mps,
        "roll_rad": state.roll_rad,
        "pitch_rad": state.pitch_rad,
        "yaw_rad": state.yaw_rad,
        "roll_rate_rad_s": state.roll_rate_rad_s,
        "pitch_rate_rad_s": state.pitch_rate_rad_s,
        "yaw_rate_rad_s": state.yaw_rate_rad_s,
    }
    ####


def _pseudo6_state_from_mapping(state: Mapping[str, object]) -> Pseudo6State:
    return Pseudo6State(
        time_s=_state_float(state, "time_s"),
        north_m=_state_float(state, "north_m"),
        east_m=_state_float(state, "east_m"),
        altitude_m=_state_float(state, "altitude_m"),
        north_velocity_mps=_state_float(state, "north_velocity_mps"),
        east_velocity_mps=_state_float(state, "east_velocity_mps"),
        vertical_velocity_mps=_state_float(state, "vertical_velocity_mps"),
        roll_rad=_state_float(state, "roll_rad"),
        pitch_rad=_state_float(state, "pitch_rad"),
        yaw_rad=_state_float(state, "yaw_rad"),
        roll_rate_rad_s=_state_float(state, "roll_rate_rad_s"),
        pitch_rate_rad_s=_state_float(state, "pitch_rate_rad_s"),
        yaw_rate_rad_s=_state_float(state, "yaw_rate_rad_s"),
    )
    ####


def _point_mass_state_mapping(state: PointMassState) -> dict[str, object]:
    return {
        "time_s": state.time_s,
        "north_m": state.north_m,
        "east_m": state.east_m,
        "altitude_m": state.altitude_m,
        "north_velocity_mps": state.north_velocity_mps,
        "east_velocity_mps": state.east_velocity_mps,
        "vertical_velocity_mps": state.vertical_velocity_mps,
    }
    ####


def _point_mass_state_from_mapping(state: Mapping[str, object]) -> PointMassState:
    return PointMassState(
        time_s=_state_float(state, "time_s"),
        north_m=_state_float(state, "north_m"),
        east_m=_state_float(state, "east_m"),
        altitude_m=_state_float(state, "altitude_m"),
        north_velocity_mps=_state_float(state, "north_velocity_mps"),
        east_velocity_mps=_state_float(state, "east_velocity_mps"),
        vertical_velocity_mps=_state_float(state, "vertical_velocity_mps"),
    )
    ####


def _waypoint_from_session_state(state: Mapping[str, object]) -> PointMassWaypoint:
    return PointMassWaypoint(
        north_m=_state_float(state, "navigation.waypoint.north.command"),
        east_m=_state_float(state, "navigation.waypoint.east.command"),
        altitude_m=_state_float(state, "navigation.waypoint.altitude.command"),
        capture_radius_m=_state_float(state, "navigation.waypoint.capture_radius.command"),
    )
    ####


def _session_authority_id(mission_template_id: str) -> str:
    if mission_template_id == TARGET_TRACK_MISSION_TEMPLATE_ID:
        return "live_target_track_guidance"
    if mission_template_id == DIRECT_ACCELERATION_MISSION_TEMPLATE_ID:
        return "live_direct_lateral_acceleration"
    return "live_waypoint_guidance"
    ####


def _session_action_ids(mission_template_id: str) -> tuple[str, ...]:
    if mission_template_id == TARGET_TRACK_MISSION_TEMPLATE_ID:
        return _TARGET_TRACK_ACTION_IDS
    if mission_template_id == DIRECT_ACCELERATION_MISSION_TEMPLATE_ID:
        return _DIRECT_ACCELERATION_ACTION_IDS
    return _WAYPOINT_ACTION_IDS
    ####


def _session_update_event(mission_template_id: str) -> str:
    if mission_template_id == TARGET_TRACK_MISSION_TEMPLATE_ID:
        return "target_track_updated"
    if mission_template_id == DIRECT_ACCELERATION_MISSION_TEMPLATE_ID:
        return "direct_lateral_acceleration_updated"
    return "waypoint_retargeted"
    ####


def _session_lowering_prefix(mission_template_id: str) -> tuple[str, ...]:
    if mission_template_id == TARGET_TRACK_MISSION_TEMPLATE_ID:
        return (
            "live_target_track_hold",
            "constant_velocity_target_propagation",
            "taoryx_registered_relative_state_track",
            "target_relative_guidance",
        )
    if mission_template_id == DIRECT_ACCELERATION_MISSION_TEMPLATE_ID:
        return (
            "live_local_neu_acceleration_hold",
            "velocity_transverse_projection",
            "shared_force_authority_allocation",
        )
    return (
        "live_waypoint_hold",
        "fixed_waypoint_projection",
        "fixed_waypoint_reference",
        "target_relative_guidance",
    )
    ####


def _session_objective_state(objective: PointMassWaypoint) -> dict[str, object]:
    if objective.objective_kind == "constant_velocity_target":
        return {
            "navigation.target.position.north.command": objective.north_m,
            "navigation.target.position.east.command": objective.east_m,
            "navigation.target.position.altitude.command": objective.altitude_m,
            "navigation.target.velocity.north.command": objective.north_velocity_mps,
            "navigation.target.velocity.east.command": objective.east_velocity_mps,
            "navigation.target.velocity.vertical.command": objective.vertical_velocity_mps,
            "navigation.target.capture_radius.command": objective.capture_radius_m,
            "guidance_objective_reference_time_s": objective.reference_time_s,
        }
    if objective.objective_kind == "direct_lateral_acceleration":
        return {
            "control.lateral_acceleration.local.north.command": objective.direct_lateral_acceleration_north_mps2,
            "control.lateral_acceleration.local.east.command": objective.direct_lateral_acceleration_east_mps2,
            "control.lateral_acceleration.local.vertical.command": objective.direct_lateral_acceleration_vertical_mps2,
        }
    return {
        "navigation.waypoint.north.command": objective.north_m,
        "navigation.waypoint.east.command": objective.east_m,
        "navigation.waypoint.altitude.command": objective.altitude_m,
        "navigation.waypoint.capture_radius.command": objective.capture_radius_m,
    }
    ####


def _session_objective_from_state(
    state: Mapping[str, object],
    mission_template_id: str,
) -> PointMassWaypoint:
    if mission_template_id == DIRECT_ACCELERATION_MISSION_TEMPLATE_ID:
        return PointMassWaypoint(
            north_m=_state_float(state, "north_m"),
            east_m=_state_float(state, "east_m"),
            altitude_m=max(_state_float(state, "altitude_m"), 0.0),
            capture_radius_m=1.0,
            direct_lateral_acceleration_north_mps2=_state_float(
                state,
                "control.lateral_acceleration.local.north.command",
            ),
            direct_lateral_acceleration_east_mps2=_state_float(
                state,
                "control.lateral_acceleration.local.east.command",
            ),
            direct_lateral_acceleration_vertical_mps2=_state_float(
                state,
                "control.lateral_acceleration.local.vertical.command",
            ),
            objective_kind="direct_lateral_acceleration",
        )
    if mission_template_id != TARGET_TRACK_MISSION_TEMPLATE_ID:
        return _waypoint_from_session_state(state)
    return PointMassWaypoint(
        north_m=_state_float(state, "navigation.target.position.north.command"),
        east_m=_state_float(state, "navigation.target.position.east.command"),
        altitude_m=_state_float(state, "navigation.target.position.altitude.command"),
        capture_radius_m=_state_float(state, "navigation.target.capture_radius.command"),
        north_velocity_mps=_state_float(state, "navigation.target.velocity.north.command"),
        east_velocity_mps=_state_float(state, "navigation.target.velocity.east.command"),
        vertical_velocity_mps=_state_float(state, "navigation.target.velocity.vertical.command"),
        reference_time_s=_state_float(state, "guidance_objective_reference_time_s"),
        objective_kind="constant_velocity_target",
    )
    ####


def _accept_session_objective(
    state: dict[str, object],
    action: Mapping[str, object],
    *,
    mission_template_id: str,
    time_s: float,
) -> tuple[PointMassWaypoint, dict[str, float], bool]:
    """Accept a partial update for the selected held objective grammar."""

    action_ids = _session_action_ids(mission_template_id)
    current = _session_objective_from_state(state, mission_template_id)
    if mission_template_id == TARGET_TRACK_MISSION_TEMPLATE_ID:
        current_position = current.position_at(time_s)
        previous = {
            "navigation.target.position.north.command": current_position[0],
            "navigation.target.position.east.command": current_position[1],
            "navigation.target.position.altitude.command": current_position[2],
            "navigation.target.velocity.north.command": current.north_velocity_mps,
            "navigation.target.velocity.east.command": current.east_velocity_mps,
            "navigation.target.velocity.vertical.command": current.vertical_velocity_mps,
            "navigation.target.capture_radius.command": current.capture_radius_m,
        }
    else:
        previous = {identifier: _state_float(state, identifier) for identifier in action_ids}
    accepted = dict(previous)
    accepted.update({identifier: _state_float(action, identifier) for identifier in action})
    changed = any(accepted[identifier] != previous[identifier] for identifier in action)
    if action:
        for identifier, value in accepted.items():
            state[identifier] = value
        if mission_template_id == TARGET_TRACK_MISSION_TEMPLATE_ID:
            state["guidance_objective_reference_time_s"] = time_s
    objective = _session_objective_from_state(state, mission_template_id)
    return objective, accepted, changed
    ####


def _objective_lowering_mapping(objective: PointMassWaypoint) -> dict[str, object]:
    if objective.objective_kind == "direct_lateral_acceleration":
        return {
            "objective_kind": objective.objective_kind,
            "lateral_acceleration_local_neu_mps2": {
                "north": objective.direct_lateral_acceleration_north_mps2,
                "east": objective.direct_lateral_acceleration_east_mps2,
                "vertical": objective.direct_lateral_acceleration_vertical_mps2,
            },
        }
    target_position = objective.position_at(objective.reference_time_s)
    return {
        "objective_kind": objective.objective_kind,
        "reference_time_s": objective.reference_time_s,
        "reference_position_m": {
            "north": target_position[0],
            "east": target_position[1],
            "altitude": target_position[2],
        },
        "velocity_mps": {
            "north": objective.north_velocity_mps,
            "east": objective.east_velocity_mps,
            "vertical": objective.vertical_velocity_mps,
        },
        "capture_radius_m": objective.capture_radius_m,
    }
    ####


def _applied_objective_mapping(objective: PointMassWaypoint) -> dict[str, object]:
    if objective.objective_kind == "direct_lateral_acceleration":
        return {
            "objective_kind": objective.objective_kind,
            "north_acceleration_mps2": objective.direct_lateral_acceleration_north_mps2,
            "east_acceleration_mps2": objective.direct_lateral_acceleration_east_mps2,
            "vertical_acceleration_mps2": objective.direct_lateral_acceleration_vertical_mps2,
        }
    return {
        "objective_kind": objective.objective_kind,
        "north_m": objective.north_m,
        "east_m": objective.east_m,
        "altitude_m": objective.altitude_m,
        "north_velocity_mps": objective.north_velocity_mps,
        "east_velocity_mps": objective.east_velocity_mps,
        "vertical_velocity_mps": objective.vertical_velocity_mps,
        "capture_radius_m": objective.capture_radius_m,
    }
    ####


def _state_float(state: Mapping[str, object], identifier: str) -> float:
    value = state.get(identifier)
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"point-mass session state {identifier!r} must be numeric")
    return float(value)
    ####


def _configuration_schema(
    profile: ResolvedInterceptorProfile,
    *,
    environment_model_ids: tuple[str, ...],
    gravity_model_ids: tuple[str, ...],
    sensor_suites: Mapping[str, InterceptorSensorSuite],
    default_environment_model_id: str,
    default_gravity_model_id: str,
    default_sensor_suite_id: str,
) -> TrajectoryConfigurationSchema:
    direct_acceleration_limit_mps2 = profile.number("max_lateral_acceleration_mps2")
    parameters = (
        _number("launch.north_m", "Launch North", "Initial north coordinate in the local NED frame.", "m", None, None, "initialization"),
        _number("launch.east_m", "Launch East", "Initial east coordinate in the local NED frame.", "m", None, None, "initialization"),
        _number("launch.altitude_m", "Launch Altitude", "Initial geometric altitude.", "m", 0.0, None, "initialization"),
        _number("launch.speed_mps", "Launch Speed", "Initial scalar speed.", "m/s", 0.1, None, "initialization"),
        _number("launch.heading_deg", "Launch Heading", "Initial local heading clockwise from north.", "deg", -360.0, 360.0, "initialization"),
        _number("launch.flight_path_deg", "Launch Flight-Path Angle", "Initial flight-path angle, positive upward.", "deg", -89.0, 89.0, "initialization"),
        _number("navigation.waypoint.north.command", "Waypoint North", "Initial or batch waypoint north coordinate.", "m", None, None, "segment"),
        _number("navigation.waypoint.east.command", "Waypoint East", "Initial or batch waypoint east coordinate.", "m", None, None, "segment"),
        _number("navigation.waypoint.altitude.command", "Waypoint Altitude", "Initial or batch waypoint geometric altitude.", "m", 0.0, None, "segment"),
        _number("navigation.waypoint.capture_radius.command", "Capture Radius", "Three-dimensional waypoint capture radius.", "m", 0.1, None, "constraint"),
        _number(
            "navigation.target.position.north.command",
            "Target North",
            "Constant-velocity target reference north coordinate at mission epoch.",
            "m",
            None,
            None,
            "segment",
        ),
        _number(
            "navigation.target.position.east.command",
            "Target East",
            "Constant-velocity target reference east coordinate at mission epoch.",
            "m",
            None,
            None,
            "segment",
        ),
        _number(
            "navigation.target.position.altitude.command",
            "Target Altitude",
            "Constant-velocity target reference geometric altitude at mission epoch.",
            "m",
            0.0,
            None,
            "segment",
        ),
        _number(
            "navigation.target.velocity.north.command",
            "Target North Velocity",
            "Held target north velocity for the constant-velocity track surrogate.",
            "m/s",
            None,
            None,
            "segment",
        ),
        _number(
            "navigation.target.velocity.east.command",
            "Target East Velocity",
            "Held target east velocity for the constant-velocity track surrogate.",
            "m/s",
            None,
            None,
            "segment",
        ),
        _number(
            "navigation.target.velocity.vertical.command",
            "Target Vertical Velocity",
            "Held target vertical velocity, positive upward, for the constant-velocity track surrogate.",
            "m/s",
            None,
            None,
            "segment",
        ),
        _number(
            "navigation.target.capture_radius.command",
            "Target Intercept Radius",
            "Three-dimensional target-intercept radius for the constant-velocity track surrogate.",
            "m",
            0.1,
            None,
            "constraint",
        ),
        _number(
            "control.lateral_acceleration.local.north.command",
            "Direct Lateral Acceleration North",
            "Held external reduced-order lateral-acceleration command in local north; projected transverse to current velocity before allocation.",
            "m/s^2",
            -direct_acceleration_limit_mps2,
            direct_acceleration_limit_mps2,
            "segment",
        ),
        _number(
            "control.lateral_acceleration.local.east.command",
            "Direct Lateral Acceleration East",
            "Held external reduced-order lateral-acceleration command in local east; projected transverse to current velocity before allocation.",
            "m/s^2",
            -direct_acceleration_limit_mps2,
            direct_acceleration_limit_mps2,
            "segment",
        ),
        _number(
            "control.lateral_acceleration.local.vertical.command",
            "Direct Lateral Acceleration Vertical",
            "Held external reduced-order lateral-acceleration command, positive upward; projected transverse to current velocity before allocation.",
            "m/s^2",
            -direct_acceleration_limit_mps2,
            direct_acceleration_limit_mps2,
            "segment",
        ),
        _number("runtime.duration_s", "Duration", "Maximum batch propagation duration.", "s", 0.01, 3_600.0, "constraint"),
        _number(
            "runtime.time_step_s",
            "Time Step",
            "Fixed integration step; pseudo-6DOF validation rejects a locally unstable response-law discretization.",
            "s",
            0.001,
            5.0,
            "constraint",
        ),
        _enum_parameter(
            "runtime.environment_model_id",
            "Environment Model",
            "Exact registered standard environment provider used for atmosphere and wind sampling.",
            environment_model_ids,
            default_environment_model_id,
        ),
        _enum_parameter(
            "runtime.gravity_model_id",
            "Gravity Model",
            "Exact registered gravity acceleration model used by the translational kernels.",
            gravity_model_ids,
            default_gravity_model_id,
        ),
        _enum_parameter(
            "runtime.sensor_suite_id",
            "Sensor Suite",
            ("Versioned registered Taoryx sensor-provider pair: translation acceleration for point mass and IMU for pseudo-6DOF."),
            tuple(sensor_suites),
            default_sensor_suite_id,
            enum_labels={identifier: f"{identifier} @ {suite.version} ({suite.fingerprint[:12]})" for identifier, suite in sensor_suites.items()},
            provenance=(
                "registered Taoryx interceptor sensor suites: "
                + ", ".join(f"{identifier}@{suite.version}#{suite.fingerprint}" for identifier, suite in sensor_suites.items())
            ),
        ),
    )
    return TrajectoryConfigurationSchema(
        model_id=profile.model_id,
        model_version=profile.parameter_set_version,
        supported_fidelities=(POINT_MASS_FIDELITY_ID, PSEUDO6_FIDELITY_ID),
        root=ConfigurationGroupSchema(
            id="intercept",
            label="Interceptor Mission",
            description="Compact launch, waypoint, target-track, direct lateral-acceleration control, and runtime configuration.",
            children=parameters,
        ),
        claim_boundary="Validation covers schema shape, units, and local bounds; it does not establish engagement feasibility.",
    )
    ####


def _enum_parameter(
    identifier: str,
    label: str,
    description: str,
    choices: tuple[str, ...],
    default: str,
    *,
    enum_labels: Mapping[str, str] | None = None,
    provenance: str = "registered standard Taoryx runtime dependency",
) -> ConfigurationParameterSchema:
    return ConfigurationParameterSchema(
        id=identifier,
        label=label,
        description=description,
        value_type="enum",
        choices=choices,
        default=default,
        default_declared=True,
        role="variant",
        compatible_fidelities=(POINT_MASS_FIDELITY_ID, PSEUDO6_FIDELITY_ID),
        transform="categorical",
        value_space=ConfigurationValueSpace(
            topology="discrete",
            representation="category_id",
            error_rule="equality",
            interpolation_rule="none",
        ),
        presentation=ValuePresentationMetadata(
            group="runtime",
            control="select",
            enum_labels=dict(enum_labels or {}),
        ),
        provenance=provenance,
    )
    ####


def _number(
    identifier: str,
    label: str,
    description: str,
    unit: str,
    lower: float | None,
    upper: float | None,
    role: str,
) -> ConfigurationParameterSchema:
    value_space = ConfigurationValueSpace(
        topology="bounded_interval" if lower is not None or upper is not None else "euclidean",
        representation="scalar",
        error_rule="subtraction",
        interpolation_rule="linear",
    )
    return ConfigurationParameterSchema(
        id=identifier,
        label=label,
        description=description,
        value_type="number",
        quantity=_quantity(unit),
        canonical_unit=unit,
        display_unit=unit,
        default=_DEFAULTS[identifier],
        default_declared=True,
        interval=ConfigurationInterval(
            minimum=ConfigurationBound(value=lower) if lower is not None else None,
            maximum=ConfigurationBound(value=upper) if upper is not None else None,
        ),
        role=role,  # type: ignore[arg-type]
        compatible_fidelities=(POINT_MASS_FIDELITY_ID, PSEUDO6_FIDELITY_ID),
        value_space=value_space,
        presentation=ValuePresentationMetadata(group=identifier.split(".", 1)[0], control="number_input"),
        provenance="taoryx-parametric-interceptors portable mission grammar",
    )
    ####


def _model_metadata(
    profile: ResolvedInterceptorProfile,
    schema: TrajectoryConfigurationSchema,
    *,
    environment_model_ids: tuple[str, ...],
    gravity_model_ids: tuple[str, ...],
    sensor_suites: Mapping[str, InterceptorSensorSuite],
    default_environment_model_id: str,
    default_gravity_model_id: str,
    default_sensor_suite_id: str,
) -> TrajectoryModelMetadata:
    output = _output_schema(profile)
    controls = _control_advertisement(profile, step_capable=True)
    default_response = analyze_pseudo6_response(profile)
    applicability = InterceptorApplicabilityEnvelope.from_profile(profile)
    applicability_bounds = ", ".join(f"{name}={getattr(applicability, name)}" for name in applicability.declared_bounds) or "none"
    mission = TrajectoryMissionTemplateMetadata(
        id=MISSION_TEMPLATE_ID,
        name="Waypoint Intercept",
        description="Launch one resolved surrogate toward a local-NED waypoint; either tier may retarget the held waypoint in a session.",
        status="reduced_order_batch_and_session_ready",
        initialization_variants=("local_ned_launch",),
        segment_sequence=("boost", "waypoint_guidance"),
        compatible_fidelities=(POINT_MASS_FIDELITY_ID, PSEUDO6_FIDELITY_ID),
        operations=(
            TrajectoryMissionOperationMetadata(
                fidelity=POINT_MASS_FIDELITY_ID,
                realization_id=POINT_MASS_REALIZATION_ID,
                operation="validate",
                status="available",
                execution_mode="portable_schema_validation",
                claim_boundary="Structural and local-domain validation only.",
            ),
            TrajectoryMissionOperationMetadata(
                fidelity=POINT_MASS_FIDELITY_ID,
                realization_id=POINT_MASS_REALIZATION_ID,
                operation="batch",
                status="available",
                execution_mode="resolved_profile_to_neutral_point_mass_kernel",
                common_runner_status="registered",
                executor_id="taoryx.parametric-interceptors.point-mass-batch.v1",
                claim_boundary=(
                    "Surrogate batch execution with selected registered translation-acceleration projection only; "
                    "no attitude, sensor-error, weapon, or control-system performance claim."
                ),
            ),
            TrajectoryMissionOperationMetadata(
                fidelity=POINT_MASS_FIDELITY_ID,
                realization_id=POINT_MASS_REALIZATION_ID,
                operation="step",
                status="available",
                execution_mode="held_live_waypoint_to_shared_point_mass_kernel_and_standard_translation_sensor",
                common_runner_status="registered",
                executor_id="taoryx.parametric-interceptors.point-mass-session.v1",
                claim_boundary=(
                    "Stateful surrogate waypoint retargeting with checkpointed registered translation-sensor history only; "
                    "no attitude, seeker, datalink, autopilot, or weapon-performance claim."
                ),
            ),
            TrajectoryMissionOperationMetadata(
                fidelity=PSEUDO6_FIDELITY_ID,
                realization_id=PSEUDO6_REALIZATION_ID,
                operation="validate",
                status="available",
                execution_mode="portable_schema_validation",
                claim_boundary="Structural and local-domain validation only.",
            ),
            TrajectoryMissionOperationMetadata(
                fidelity=PSEUDO6_FIDELITY_ID,
                realization_id=PSEUDO6_REALIZATION_ID,
                operation="batch",
                status="available",
                execution_mode="resolved_profile_to_attitude_response_kernel_and_standard_imu",
                common_runner_status="registered",
                executor_id="taoryx.parametric-interceptors.pseudo6-batch.v1",
                claim_boundary=(
                    "Reduced-order attitude response with a selected compatible registered Taoryx IMU; no rigid-body, actuator, seeker, or controller-qualification claim."
                ),
            ),
            TrajectoryMissionOperationMetadata(
                fidelity=PSEUDO6_FIDELITY_ID,
                realization_id=PSEUDO6_REALIZATION_ID,
                operation="step",
                status="available",
                execution_mode="held_live_waypoint_to_shared_attitude_response_kernel_and_standard_imu",
                common_runner_status="registered",
                executor_id="taoryx.parametric-interceptors.pseudo6-session.v1",
                claim_boundary=(
                    "Stateful reduced-order attitude response with checkpointed registered Taoryx IMU history; "
                    "no rigid-body, actuator, seeker, datalink, or controller-qualification claim."
                ),
            ),
        ),
        provenance="resolved interceptor profile and parametric waypoint kernel",
        claim_boundary="The fixed or live waypoint is a reduced-order guidance objective, not a seeker, datalink, or fire-control system.",
    )
    target_track_mission = mission.model_copy(
        update={
            "id": TARGET_TRACK_MISSION_TEMPLATE_ID,
            "name": "Constant-Velocity Target Intercept",
            "description": (
                "Launch one resolved surrogate toward an externally supplied constant-velocity local target track; "
                "either tier may update the held track in a session."
            ),
            "initialization_variants": ("local_ned_launch_and_target_track",),
            "segment_sequence": ("boost", "target_track_guidance"),
            "operations": tuple(
                item.model_copy(
                    update={
                        "execution_mode": (
                            "portable_schema_validation"
                            if item.operation == "validate"
                            else (
                                "constant_velocity_target_track_through_standard_relative_state_sensor_to_shared_point_mass_kernel"
                                if item.fidelity == POINT_MASS_FIDELITY_ID and item.operation == "batch"
                                else (
                                    "held_live_target_track_through_standard_relative_state_sensor_to_shared_point_mass_kernel"
                                    if item.fidelity == POINT_MASS_FIDELITY_ID
                                    else (
                                        "constant_velocity_target_track_through_standard_relative_state_sensor_to_shared_attitude_response_kernel_and_standard_imu"
                                        if item.operation == "batch"
                                        else "held_live_target_track_through_standard_relative_state_sensor_to_shared_attitude_response_kernel_and_standard_imu"
                                    )
                                )
                            )
                        ),
                        "claim_boundary": (
                            "Structural and local-domain validation only."
                            if item.operation == "validate"
                            else (
                                "Caller-supplied constant-velocity target truth is projected through Taoryx's registered "
                                "direct-geometry relative-state sensor before the shared reduced-order guidance kernel; no "
                                "propagation, signature, gimbal, track-manager, datalink, target-dynamics, controller, or "
                                "weapon-performance claim."
                            )
                        ),
                    }
                )
                for item in mission.operations
            ),
            "provenance": ("resolved interceptor profile, external target-truth grammar, registered Taoryx relative-state sensor, and shared guidance kernel"),
            "claim_boundary": (
                "The target truth is caller-supplied and constant velocity. Guidance consumes Taoryx's registered "
                "direct-geometry relative-state measurement; capture evaluation remains truth based. This does not "
                "implement or qualify propagation, signatures, gimbals, a track manager, datalink, target dynamics, "
                "a physical seeker, or a fire-control system."
            ),
        }
    )
    direct_acceleration_mission = mission.model_copy(
        update={
            "id": DIRECT_ACCELERATION_MISSION_TEMPLATE_ID,
            "name": "Direct Lateral Acceleration Control",
            "description": (
                "Launch one resolved surrogate with a caller-held local-NEU lateral-acceleration demand; "
                "either tier may update the demand through a standard session."
            ),
            "initialization_variants": ("local_ned_launch_and_direct_acceleration",),
            "segment_sequence": ("direct_lateral_acceleration_control",),
            "operations": tuple(
                item.model_copy(
                    update={
                        "execution_mode": (
                            "portable_schema_validation"
                            if item.operation == "validate"
                            else (
                                "held_local_neu_acceleration_through_shared_point_mass_force_authority"
                                if item.fidelity == POINT_MASS_FIDELITY_ID
                                else "held_local_neu_acceleration_through_shared_attitude_response_and_force_authority"
                            )
                        ),
                        "claim_boundary": (
                            "Structural and local-domain validation only."
                            if item.operation == "validate"
                            else (
                                "Caller-supplied local-NEU acceleration is projected transverse to current velocity "
                                "and lowered through the existing reduced-order authority path; no actuator, control-surface, "
                                "autopilot, rigid-body moment, seeker, or weapon-performance claim."
                            )
                        ),
                    }
                )
                for item in mission.operations
            ),
            "provenance": "resolved interceptor profile and shared reduced-order force-authority kernels",
            "claim_boundary": (
                "The external command is a reduced-order lateral acceleration request, not a physical control surface, "
                "actuator, autopilot, or rigid-body moment command."
            ),
        }
    )
    point_mass_realization = TrajectoryRealizationMetadata(
        id=POINT_MASS_REALIZATION_ID,
        label="Parametric Guided Point Mass",
        description="Translation-only point mass with bounded surrogate waypoint steering and standard acceleration projection.",
        status="available",
        dynamics_fidelities=("point_mass_3dof",),
        input_realization="guidance_command",
        controls=controls,
        fidelity_aliases=(POINT_MASS_FIDELITY_ID,),
        mission_template_ids=_MISSION_TEMPLATE_IDS,
        operations=("validate", "batch", "step"),
        native_factory_ids=("parametric_interceptor_point_mass.v1", "taoryx.sensor-plugin-registry.v1"),
        source_refs=("taoryx_parametric_interceptors/kernel.py", "taoryx.sensor_api.SensorPluginRegistry"),
        claim_boundary=(
            "No attitude, actuator, seeker, or rigid-body states are represented. Translation sensor values come from "
            "the selected compatible registered provider; inspect its suite before making any error-model claim."
        ),
    )
    pseudo6_realization = TrajectoryRealizationMetadata(
        id=PSEUDO6_REALIZATION_ID,
        label="Parametric Attitude-Response Pseudo-6DOF",
        description=("Bounded roll, pitch, yaw, and body-rate response driven by the same waypoint guidance authority, with registered Taoryx IMU projection."),
        status="available",
        dynamics_fidelities=("pseudo_6dof",),
        input_realization="guidance_command",
        controls=controls,
        fidelity_aliases=(PSEUDO6_FIDELITY_ID,),
        mission_template_ids=_MISSION_TEMPLATE_IDS,
        operations=("validate", "batch", "step"),
        native_factory_ids=("parametric_interceptor_pseudo6.v1", "taoryx.sensor-plugin-registry.v1"),
        source_refs=("taoryx_parametric_interceptors/pseudo6.py", "taoryx.sensor_api.SensorPluginRegistry"),
        claim_boundary=(
            "Attitude is a bounded response law scaled by current lateral command support rather than a rigid-body "
            "moment balance. Body-axis relative wind and flow angles are geometric projections, not coefficient-deck "
            "inputs or aerodynamic moment states. IMU values come from the "
            "selected compatible registered provider; inspect its suite before making any error-model claim."
        ),
    )
    evidence_count = sum(item.origin.value in {"observed", "reported"} for item in profile.parameters.values())
    evidence_count += int(profile.drag_coefficient_schedule.origin in {"observed", "reported"})
    evidence_count += int(profile.thrust_profile_schedule.origin in {"observed", "reported"})
    if profile.second_pulse_thrust_profile_schedule is not None:
        evidence_count += int(profile.second_pulse_thrust_profile_schedule.origin in {"observed", "reported"})
    if profile.thrust_time_curve is not None:
        evidence_count += int(profile.thrust_time_curve.origin in {"observed", "reported"})
    if profile.dual_pulse_thrust_program is not None:
        evidence_count += sum(
            origin in {"observed", "reported"}
            for origin in (
                profile.dual_pulse_thrust_program.origin,
                profile.dual_pulse_thrust_program.first_pulse.origin,
                profile.dual_pulse_thrust_program.second_pulse.origin,
            )
        )
    parameter_usage = interceptor_parameter_usage(profile)
    parameter_usage_manifest = json.dumps(
        {identifier: usage.model_dump(mode="json", exclude={"contract", "parameter_id"}) for identifier, usage in parameter_usage.items()},
        sort_keys=True,
        separators=(",", ":"),
    )
    display_identity = profile.variant_basis or f"{profile.interceptor_id} {profile.variant}"
    capabilities = TrajectoryModelCapabilities(
        initialization_modes=(
            "local_ned_launch",
            "local_ned_launch_and_target_track",
            "local_ned_launch_and_direct_acceleration",
        ),
        segment_types=(
            "boost",
            "waypoint_guidance",
            "target_track_guidance",
            "target_track_unavailable",
            "direct_lateral_acceleration_control",
        ),
        termination_modes=("duration", "waypoint_capture", "target_intercept", "ground_impact"),
        operations=("discover", "validate", "batch", "step"),
    )
    fidelity_transitions = (
        TrajectoryFidelityTransition(
            from_fidelity=POINT_MASS_FIDELITY_ID,
            to_fidelity=PSEUDO6_FIDELITY_ID,
            direction="step_up",
            status="available",
            automatic=False,
            selection_policy="explicit_upgrade_only",
            requirements=("prepare a new batch configuration with the pseudo-6DOF fidelity",),
            state_transfer="shared launch and waypoint configuration; a new run initializes attitude from launch velocity",
            claim_boundary="No live in-run promotion or rigid-body state reconstruction is claimed.",
        ),
        TrajectoryFidelityTransition(
            from_fidelity=PSEUDO6_FIDELITY_ID,
            to_fidelity=POINT_MASS_FIDELITY_ID,
            direction="step_down",
            status="available",
            automatic=False,
            selection_policy="exact_only",
            requirements=("prepare a new batch configuration with the point-mass fidelity",),
            state_transfer=(
                "shared launch and waypoint configuration; attitude and IMU state are dropped, and a new "
                "translation-sensor history starts with the point-mass run"
            ),
            claim_boundary="No live in-run lowering or cross-fidelity estimator-state transfer is claimed.",
        ),
    )
    return TrajectoryModelMetadata(
        id=profile.model_id,
        name=f"{display_identity} Parametric Interceptor",
        version=profile.parameter_set_version,
        description=(
            "Evidence-aware, data-defined interceptor surrogate with explicit archetype resolution. Add a new model "
            "from Python, flat YAML, or a provenance-bearing catalogue record; no model-specific runtime is required."
        ),
        presentation=TrajectoryModelPresentationMetadata(
            display_name=display_identity,
            short_name=profile.variant_basis or profile.interceptor_id,
            summary=("Resolved low-fidelity interceptor surrogate with batch/live waypoint, target-track, and direct lateral-acceleration control."),
            category="Parametric Interceptors",
            subcategory="Surface-to-Air / Generic Interceptor",
            sort_key=f"interceptor:{profile.model_id}",
            badges=("Point-Mass", "Pseudo-6DOF", "Evidence-Aware", profile.assumption_case.value.title()),
            default_fidelity_id=POINT_MASS_FIDELITY_ID,
            default_mission_template_id=MISSION_TEMPLATE_ID,
            default_output_channel_ids=("position.local.north", "position.local.east", "position.geometric.altitude", "velocity.speed"),
            properties=(
                _property(profile, "profile_fingerprint", "Resolved Profile Fingerprint", profile.fingerprint, "string", "identity", None, 10),
                _property(profile, "assumption_case", "Assumption Case", profile.assumption_case.value, "string", "evidence", None, 20),
                _property(profile, "evidence_value_count", "Observed / Reported Values", evidence_count, "integer", "evidence", None, 30),
                _property(
                    profile,
                    "calibration_screen_status",
                    "Calibration Screen Status",
                    "available_via_provider_api",
                    "string",
                    "capability",
                    None,
                    31,
                ),
                _property(
                    profile,
                    "calibration_observables",
                    "Calibration Observables",
                    (
                        "peak_speed_mps, maximum_altitude_m, horizontal_distance_m, elapsed_time_s, "
                        "terminal_waypoint_range_m, propellant_remaining_kg, waypoint_captured"
                    ),
                    "string",
                    "capability",
                    None,
                    32,
                ),
                _property(
                    profile,
                    "calibration_qualification",
                    "Calibration Qualification",
                    "unqualified_surrogate_screen",
                    "string",
                    "evidence",
                    None,
                    33,
                ),
                _property(
                    profile,
                    "parameter_fit_status",
                    "Parameter Fit Status",
                    "available_via_authoring_api",
                    "string",
                    "capability",
                    None,
                    34,
                ),
                _property(
                    profile,
                    "parameter_fit_variables",
                    "Parameter Fit Variables",
                    ("thrust_scale, drag_scale, maneuverability_scale, guidance_time_constant_scale"),
                    "string",
                    "capability",
                    None,
                    35,
                ),
                _property(
                    profile,
                    "parameter_fit_receipt_contract",
                    "Parameter Fit Receipt Contract",
                    "taoryx.parametric-interceptors.fit-receipt/v1",
                    "string",
                    "implementation",
                    None,
                    36,
                ),
                _property(
                    profile,
                    "response_analysis_status",
                    "Response Analysis Status",
                    "available_local_unsaturated_frozen_command_support",
                    "string",
                    "capability",
                    None,
                    37,
                ),
                _property(
                    profile,
                    "response_analysis_contract",
                    "Response Analysis Contract",
                    default_response.contract,
                    "string",
                    "implementation",
                    None,
                    38,
                ),
                _property(
                    profile,
                    "response_comparison_status",
                    "Response Comparison Status",
                    "available_like_for_like",
                    "string",
                    "capability",
                    None,
                    39,
                ),
                _property(
                    profile,
                    "default_response_discrete_status",
                    "Default Response Discrete Status",
                    default_response.discrete_status,
                    "string",
                    "capability",
                    None,
                    40,
                ),
                _property(
                    profile,
                    "default_response_spectral_radius",
                    "Default Response Spectral Radius",
                    default_response.discrete_spectral_radius,
                    "number",
                    "capability",
                    "1",
                    41,
                ),
                _property(
                    profile,
                    "default_response_sampling_quality",
                    "Default Response Sampling Quality",
                    default_response.sampling_quality,
                    "string",
                    "capability",
                    None,
                    42,
                ),
                _property(
                    profile,
                    "response_analysis_axes",
                    "Response Analysis Axes",
                    "roll, pitch, yaw",
                    "string",
                    "capability",
                    None,
                    43,
                ),
                _property(
                    profile,
                    "response_angle_topology",
                    "Response Angle Topology",
                    (f"roll=bounded(+/-{profile.number('max_bank_angle_rad'):.6g} rad); pitch=bounded(+/-1.55334 rad); yaw=periodic[-pi,pi)"),
                    "string",
                    "capability",
                    None,
                    44,
                ),
                _property(
                    profile,
                    "response_step_witness_status",
                    "Response Step Witness Status",
                    "available_via_provider_api",
                    "string",
                    "capability",
                    None,
                    45,
                ),
                _property(
                    profile,
                    "response_operating_point_analysis_status",
                    "Response Operating-Point Analysis Status",
                    "available_shared_force_authority_binding_v1",
                    "string",
                    "capability",
                    None,
                    46,
                ),
                _property(
                    profile,
                    "response_operating_point_comparison_status",
                    "Response Operating-Point Comparison Status",
                    "available_matched_force_point_authority_and_tuning_deltas_v1",
                    "string",
                    "capability",
                    None,
                    47,
                ),
                _property(
                    profile,
                    "live_waypoint_session_status",
                    "Live Waypoint Session Status",
                    "available_point_mass_and_pseudo6",
                    "string",
                    "capability",
                    None,
                    48,
                ),
                _property(
                    profile,
                    "live_waypoint_feedback",
                    "Live Waypoint Feedback",
                    (
                        "requested, accepted, held, waypoint_range, guidance_available, "
                        "guidance_archetype, guidance_mode, closing_speed, line_of_sight_rate, "
                        "navigation_constant, "
                        "lateral_acceleration_commanded, lateral_acceleration_achieved, "
                        "local_neu_command_vector, local_neu_achieved_vector, "
                        "command_achievement_fraction, direction_error_valid, direction_error, "
                        "lateral_acceleration_limit, lateral_acceleration_utilization, "
                        "control_configuration, aerodynamic_authority, thrust_vector_authority, "
                        "combined_authority, instantaneous_authority, authority_utilization, "
                        "structural_limit_active, allocation_policy, aerodynamic_achieved, "
                        "thrust_vector_achieved, thrust_vector_angle, axial_thrust, "
                        "base_drag, maneuver_drag, total_drag, "
                        "pseudo6_attitude_authority_available, pseudo6_command_support_fraction, "
                        "pseudo6_attitude_authority_limited, pseudo6_body_relative_wind, "
                        "pseudo6_angle_of_attack, pseudo6_sideslip, "
                        "control_limited, control_limit_reason, "
                        "applicability_declared, applicability_status, applicability_reason, "
                        "phase_id, vehicle_operational, propulsion_available"
                    ),
                    "string",
                    "capability",
                    None,
                    47,
                ),
                _property(
                    profile,
                    "default_environment_model_id",
                    "Default Environment Model",
                    default_environment_model_id,
                    "string",
                    "implementation",
                    None,
                    48,
                ),
                _property(
                    profile,
                    "available_environment_model_ids",
                    "Available Environment Models",
                    ", ".join(environment_model_ids),
                    "string",
                    "implementation",
                    None,
                    49,
                ),
                _property(
                    profile,
                    "default_gravity_model_id",
                    "Default Gravity Model",
                    default_gravity_model_id,
                    "string",
                    "implementation",
                    None,
                    50,
                ),
                _property(
                    profile,
                    "available_gravity_model_ids",
                    "Available Gravity Models",
                    ", ".join(gravity_model_ids),
                    "string",
                    "implementation",
                    None,
                    51,
                ),
                _property(
                    profile,
                    "default_sensor_suite_id",
                    "Default Sensor Suite",
                    default_sensor_suite_id,
                    "string",
                    "implementation",
                    None,
                    52,
                ),
                _property(
                    profile,
                    "available_sensor_suites",
                    "Available Sensor Suites",
                    ", ".join(f"{identifier}@{suite.version}" for identifier, suite in sensor_suites.items()),
                    "string",
                    "implementation",
                    None,
                    53,
                ),
                _property(
                    profile,
                    "default_point_mass_sensor_provider",
                    "Default Point-Mass Sensor Provider",
                    sensor_suites[default_sensor_suite_id].point_mass_provider.kind,
                    "string",
                    "implementation",
                    None,
                    54,
                ),
                _property(
                    profile,
                    "default_pseudo6_sensor_provider",
                    "Default Pseudo-6DOF Sensor Provider",
                    sensor_suites[default_sensor_suite_id].pseudo6_provider.kind,
                    "string",
                    "implementation",
                    None,
                    55,
                ),
                _property(
                    profile,
                    "default_target_track_sensor_provider",
                    "Default Target-Track Sensor Provider",
                    sensor_suites[default_sensor_suite_id].target_track_provider.kind,
                    "string",
                    "implementation",
                    None,
                    56,
                ),
                _property(
                    profile,
                    "live_target_track_session_status",
                    "Live Target-Track Session Status",
                    "available_point_mass_and_pseudo6",
                    "string",
                    "capability",
                    None,
                    57,
                ),
                _property(
                    profile,
                    "capture_event_detection_contract",
                    "Capture Event Detection Contract",
                    "taoryx.parametric-interceptors.semi-implicit-step-events/v1",
                    "string",
                    "implementation",
                    None,
                    58,
                ),
                _property(
                    profile,
                    "applicability.contract",
                    "Applicability Contract",
                    APPLICABILITY_CONTRACT,
                    "string",
                    "implementation",
                    None,
                    59,
                ),
                _property(
                    profile,
                    "applicability.enforcement",
                    "Applicability Enforcement",
                    APPLICABILITY_ENFORCEMENT,
                    "string",
                    "capability",
                    None,
                    60,
                ),
                _property(
                    profile,
                    "applicability.declared",
                    "Applicability Envelope Declared",
                    applicability.declared,
                    "boolean",
                    "evidence",
                    None,
                    61,
                ),
                _property(
                    profile,
                    "applicability.bounds",
                    "Declared Applicability Bounds",
                    applicability_bounds,
                    "string",
                    "evidence",
                    None,
                    62,
                ),
                _property(
                    profile,
                    "pseudo6.response_authority_coupling",
                    "Pseudo-6DOF Response Authority Coupling",
                    "current_lateral_command_support_fraction",
                    "string",
                    "implementation",
                    None,
                    63,
                ),
                _property(
                    profile,
                    "target_track.truth_fallback",
                    "Target-Track Truth Fallback",
                    "disabled_for_guidance_and_attitude_commands",
                    "string",
                    "implementation",
                    None,
                    64,
                ),
                _property(
                    profile,
                    "pseudo6.flow_angle_contract",
                    "Pseudo-6DOF Flow-Angle Contract",
                    "air_relative_velocity_projected_into_forward_right_down_response_body_frame",
                    "string",
                    "implementation",
                    None,
                    65,
                ),
                _property(
                    profile,
                    "parameter_usage.contract",
                    "Parameter Usage Contract",
                    PARAMETER_USAGE_CONTRACT,
                    "string",
                    "implementation",
                    None,
                    66,
                ),
                _property(
                    profile,
                    "parameter_usage.manifest",
                    "Resolved Parameter Usage Manifest",
                    parameter_usage_manifest,
                    "string",
                    "implementation",
                    None,
                    67,
                ),
                _property(
                    profile,
                    "control_allocation_policy.supported",
                    "Supported Control Allocation Policies",
                    ", ".join(CONTROL_ALLOCATION_POLICIES),
                    "string",
                    "capability",
                    None,
                    68,
                ),
                _property(
                    profile,
                    "response_analysis.authority_coupling",
                    "Response Analysis Authority Coupling",
                    "request_frozen_command_support_fraction_inclusive_0_to_1",
                    "string",
                    "capability",
                    None,
                    69,
                ),
                _property(
                    profile,
                    "guidance.lateral_tracking.contract",
                    "Guidance Lateral Tracking Contract",
                    "normalized_achievement_and_validated_direction_error",
                    "string",
                    "implementation",
                    None,
                    70,
                ),
                _property(
                    profile,
                    "mission_control_scope.contract",
                    "Mission Control Scope Contract",
                    MISSION_CONTROL_SCOPE_CONTRACT,
                    "string",
                    "implementation",
                    None,
                    71,
                ),
                *_profile_context_properties(profile),
                *_resolved_properties(profile),
            ),
        ),
        family_id="parametric_interceptor",
        physical_family="interceptor",
        model_kind="parametric_surrogate",
        status="common_runner_ready",
        tags=(
            "interceptor",
            "sam",
            "parametric",
            "point-mass",
            "pseudo-6dof",
            "scenario-calibration-screen",
            "bounded-parameter-fit",
            "response-analysis",
            "live-waypoint-session",
            "constant-velocity-target-intercept",
            "direct-lateral-acceleration-control",
            "native-relative-state-target-guidance",
            "advisory-applicability-envelope",
            "within-step-capture-localization",
            profile.assumption_case.value,
            profile.resolution_status.value,
        ),
        execution_capability_profile="taoryx_universal",
        operations=("discover", "validate", "batch", "step"),
        common_runner_operations=("batch", "step"),
        capabilities=capabilities,
        composition_advertisement=build_trajectory_composition_advertisement(
            capabilities=capabilities,
            realizations=(point_mass_realization, pseudo6_realization),
            mission_templates=(mission, target_track_mission, direct_acceleration_mission),
            deployments=(),
            output_schema=output,
            fidelity_transitions=fidelity_transitions,
        ),
        realizations=(point_mass_realization, pseudo6_realization),
        mission_templates=(mission, target_track_mission, direct_acceleration_mission),
        reference_frames=(
            TrajectoryReferenceFrameMetadata(
                id="local_ned",
                name="Local North-East-Down",
                description="Local tangent frame with altitude reported as positive up.",
                frame_kind="local_tangent",
                axes=("north", "east", "down"),
                handedness="right",
                origin="configured launch datum",
                orientation="north-east-down local tangent axes",
                provenance="parametric interceptor mission configuration",
            ),
            TrajectoryReferenceFrameMetadata(
                id="local_neu",
                name="Local North-East-Up",
                description=("Local tangent vector frame used where the model's vertical component is explicitly positive up."),
                frame_kind="local_tangent",
                axes=("north", "east", "up"),
                handedness="left",
                origin="configured launch datum",
                orientation="north-east-up local tangent axes",
                provenance="parametric interceptor guidance and state convention",
            ),
            TrajectoryReferenceFrameMetadata(
                id="body",
                name="Body Forward-Right-Down",
                description="Pseudo-6DOF response-law body frame used for attitude truth and registered IMU increments.",
                frame_kind="body",
                axes=("forward", "right", "down"),
                handedness="right",
                origin="surrogate vehicle reference point",
                orientation="roll-pitch-yaw response relative to local north-east-down",
                provenance="taoryx_parametric_interceptors.pseudo6",
            ),
            TrajectoryReferenceFrameMetadata(
                id="sensor",
                name="Target-Tracker Sensor Frame",
                description=(
                    "Forward-right-down measurement frame of the selected relative-state tracker; identity-mounted "
                    "to the pseudo-6DOF body or the point-mass velocity-aligned virtual body."
                ),
                frame_kind="provider_defined",
                axes=("x_forward", "y_right", "z_down"),
                handedness="right",
                origin="surrogate vehicle reference point",
                orientation="selected target-track sensor mounting relative to the active surrogate body frame",
                provenance="taoryx.sensor_plugins.relative_state",
            ),
            TrajectoryReferenceFrameMetadata(
                id="eci",
                name="Surrogate Earth-Centered Inertial Embedding",
                description=("Static local-to-ECI embedding used only at the standard translation and IMU sensor boundaries."),
                frame_kind="inertial",
                axes=("x", "y", "z"),
                handedness="right",
                origin="Earth center",
                orientation="launch-local radial/east/north embedding without Earth-rotation propagation",
                provenance="parametric interceptor local-surrogate sensor projection",
            ),
            TrajectoryReferenceFrameMetadata(
                id="ecfc",
                name="Earth-Centered Earth-Fixed",
                description="Standard environment-provider frame used for the advertised wind vector.",
                frame_kind="earth_fixed",
                axes=("x", "y", "z"),
                handedness="right",
                origin="Earth center",
                orientation="TAORYX ECFC axes supplied by the selected standard environment provider",
                provenance="taoryx.runtime.environment_runtime.EnvironmentProvider",
            ),
        ),
        output_schema=output,
        output_schema_id=output.schema_id,
        output_schema_fingerprint=output.fingerprint,
        configuration_schema_id=schema.schema_id,
        configuration_schema_fingerprint=schema.fingerprint,
        fidelities=(
            TrajectoryFidelityMetadata(
                id=POINT_MASS_FIDELITY_ID,
                label="Point-Mass 3-DOF",
                rank=0,
                declared=True,
                dynamics_fidelity="point_mass_3dof",
                input_realization="guidance_command",
                compatibility_aliases=(POINT_MASS_FIDELITY_ID,),
                runtime_fidelity="parametric_guided_point_mass",
                control_realization="fixed_or_live_waypoint_guidance",
                promotion_status="runnable_surrogate",
                operations=("validate", "batch", "step"),
                profile_id=profile.fingerprint,
                claim_boundary=(
                    "Translation-only surrogate with selected registered acceleration increments; no attitude, sensor-error, actuator, or rigid-body state."
                ),
            ),
            TrajectoryFidelityMetadata(
                id=PSEUDO6_FIDELITY_ID,
                label="Attitude-Response Pseudo-6DOF",
                rank=1,
                declared=True,
                dynamics_fidelity="pseudo_6dof",
                input_realization="guidance_command",
                compatibility_aliases=(PSEUDO6_FIDELITY_ID,),
                runtime_fidelity="parametric_attitude_response_pseudo6",
                control_realization="fixed_or_live_waypoint_to_bounded_attitude_response",
                promotion_status="runnable_surrogate",
                operations=("validate", "batch", "step"),
                profile_id=profile.fingerprint,
                claim_boundary=("Reduced-order attitude and body-rate response with selected registered sensor projection; not rigid-body 6-DOF."),
            ),
        ),
        fidelity_transitions=fidelity_transitions,
        source_refs=profile.source_record_ids,
        provenance=f"resolved profile {profile.fingerprint}",
        claim_boundary=(
            "All numerical behavior is a low-fidelity surrogate. Per-parameter origins are available through "
            "provider.resolved_profile(model_id); archetype assumptions are not factual ontology assertions."
        ),
    )
    ####


def _property(
    profile: ResolvedInterceptorProfile,
    identifier: str,
    label: str,
    value: object,
    value_type: str,
    semantic_role: str,
    unit: str | None,
    order: int,
) -> TrajectoryModelPropertyMetadata:
    if identifier in profile.parameters:
        source_refs = profile.parameters[identifier].source_record_ids
        origin = profile.parameters[identifier].origin.value
    elif identifier.startswith("drag_schedule."):
        source_refs = profile.drag_coefficient_schedule.source_record_ids
        origin = profile.drag_coefficient_schedule.origin
    elif identifier.startswith("thrust_profile_schedule."):
        source_refs = profile.thrust_profile_schedule.source_record_ids
        origin = profile.thrust_profile_schedule.origin
    elif identifier.startswith("second_pulse_thrust_profile_schedule.") and profile.second_pulse_thrust_profile_schedule is not None:
        source_refs = profile.second_pulse_thrust_profile_schedule.source_record_ids
        origin = profile.second_pulse_thrust_profile_schedule.origin
    elif identifier.startswith("thrust_time_curve.") and profile.thrust_time_curve is not None:
        source_refs = profile.thrust_time_curve.source_record_ids
        origin = profile.thrust_time_curve.origin
    elif identifier.startswith("dual_pulse_thrust_program.first_pulse.") and profile.dual_pulse_thrust_program is not None:
        source_refs = profile.dual_pulse_thrust_program.first_pulse.source_record_ids
        origin = profile.dual_pulse_thrust_program.first_pulse.origin
    elif identifier.startswith("dual_pulse_thrust_program.second_pulse.") and profile.dual_pulse_thrust_program is not None:
        source_refs = profile.dual_pulse_thrust_program.second_pulse.source_record_ids
        origin = profile.dual_pulse_thrust_program.second_pulse.origin
    elif identifier.startswith("dual_pulse_thrust_program.") and profile.dual_pulse_thrust_program is not None:
        source_refs = profile.dual_pulse_thrust_program.source_record_ids
        origin = profile.dual_pulse_thrust_program.origin
    else:
        source_refs = profile.source_record_ids
        origin = "resolved profile"
    return TrajectoryModelPropertyMetadata(
        id=identifier,
        label=label,
        description=f"Resolved {label.lower()} ({origin}).",
        semantic_role=semantic_role,  # type: ignore[arg-type]
        value_type=value_type,  # type: ignore[arg-type]
        value_kind="nominal" if value_type == "number" else "declared",
        value=value,
        value_declared=True,
        quantity=_quantity(unit) if unit else None,
        canonical_unit=unit,
        display_unit=unit,
        presentation=ValuePresentationMetadata(group=semantic_role, order=order),
        source_refs=source_refs,
        provenance=f"resolved profile {profile.fingerprint}",
        claim_boundary="Surrogate metadata; inspect the resolved profile before treating this value as sourced evidence.",
    )
    ####


def _resolved_properties(profile: ResolvedInterceptorProfile) -> tuple[TrajectoryModelPropertyMetadata, ...]:
    """Project every resolved model value onto generic Composition metadata."""

    properties: list[TrajectoryModelPropertyMetadata] = []
    parameter_usage = interceptor_parameter_usage(profile)
    for index, (identifier, parameter) in enumerate(profile.parameters.items(), start=4):
        raw_value = parameter.value
        if isinstance(raw_value, bool):
            value_type = "boolean"
            value: object = raw_value
        elif isinstance(raw_value, int):
            value_type = "integer"
            value = raw_value
        elif isinstance(raw_value, float):
            value_type = "number"
            value = raw_value
        elif isinstance(raw_value, tuple):
            value_type = "string"
            value = ", ".join(raw_value)
        else:
            value_type = "string"
            value = raw_value
        role = _parameter_role(identifier)
        provenance_parts = [
            f"origin={parameter.origin.value}",
            f"confidence={parameter.confidence.value}",
            f"method={parameter.method}",
            f"usage_class={parameter_usage[identifier].usage_class}",
            f"affects_dynamics={str(parameter_usage[identifier].affects_dynamics).lower()}",
        ]
        if parameter_usage[identifier].consumer_ids:
            provenance_parts.append(f"consumers={','.join(parameter_usage[identifier].consumer_ids)}")
        if parameter_usage[identifier].resolved_sink_ids:
            provenance_parts.append(f"resolved_sinks={','.join(parameter_usage[identifier].resolved_sink_ids)}")
        if parameter.archetype_id is not None:
            provenance_parts.append(f"archetype_id={parameter.archetype_id}")
        if parameter.assumption_case is not None:
            provenance_parts.append(f"assumption_case={parameter.assumption_case.value}")
        if parameter.depends_on:
            provenance_parts.append(f"depends_on={','.join(parameter.depends_on)}")
        if parameter.source_value is not None:
            provenance_parts.append(f"source_value={parameter.source_value.value} {parameter.source_value.unit or ''}".rstrip())
        label = identifier.replace("_", " ").title()
        properties.append(
            TrajectoryModelPropertyMetadata(
                id=identifier,
                label=label,
                description=(f"Resolved {label.lower()}; {parameter.method}. Usage: {parameter_usage[identifier].description}"),
                semantic_role=role,  # type: ignore[arg-type]
                value_type=value_type,  # type: ignore[arg-type]
                value_kind="nominal" if value_type in {"number", "integer"} else "declared",
                value=value,
                value_declared=True,
                quantity=_quantity(parameter.unit) if value_type in {"number", "integer"} else None,
                canonical_unit=parameter.unit if value_type in {"number", "integer"} else None,
                display_unit=parameter.unit if value_type in {"number", "integer"} else None,
                presentation=ValuePresentationMetadata(group=role, order=index * 10),
                source_refs=parameter.source_record_ids,
                provenance="; ".join(provenance_parts),
                claim_boundary=(
                    "This property preserves the resolved simulation value and its evidence status. "
                    "Archetype, derived, inferred, calibrated, and simulation-assumption origins are not source facts. "
                    + (
                        "The usage manifest identifies how it reaches executable dynamics."
                        if parameter_usage[identifier].affects_dynamics
                        else "This value does not affect current low-fidelity dynamics."
                    )
                ),
            )
        )
    schedule = profile.drag_coefficient_schedule
    schedule_rows: tuple[tuple[str, str, object, str], ...] = (
        ("drag_schedule.contract", "Drag Schedule Contract", schedule.contract, "string"),
        ("drag_schedule.id", "Drag Schedule ID", schedule.schedule_id, "string"),
        ("drag_schedule.fingerprint", "Drag Schedule Fingerprint", schedule.fingerprint, "string"),
        ("drag_schedule.origin", "Drag Schedule Origin", schedule.origin, "string"),
        ("drag_schedule.confidence", "Drag Schedule Confidence", schedule.confidence, "string"),
        ("drag_schedule.method", "Drag Schedule Method", schedule.method, "string"),
        (
            "drag_schedule.source_record_ids",
            "Drag Schedule Source Records",
            ", ".join(schedule.source_record_ids),
            "string",
        ),
        ("drag_schedule.point_count", "Drag Schedule Point Count", len(schedule.points), "integer"),
        (
            "drag_schedule.mach_domain",
            "Drag Schedule Mach Domain",
            f"{schedule.points[0].mach:g}..{schedule.points[-1].mach:g}",
            "string",
        ),
        (
            "drag_schedule.points",
            "Drag Schedule Points",
            ", ".join(f"M{item.mach:g}:{item.coefficient:.9g}" for item in schedule.points),
            "string",
        ),
        (
            "drag_schedule.interpolation",
            "Drag Schedule Interpolation",
            schedule.interpolation,
            "string",
        ),
        (
            "drag_schedule.extrapolation",
            "Drag Schedule Extrapolation",
            schedule.extrapolation,
            "string",
        ),
    )
    base_order = (len(properties) + 4) * 10
    properties.extend(
        _property(
            profile,
            identifier,
            label,
            value,
            value_type,
            "implementation",
            None,
            base_order + index * 10,
        )
        for index, (identifier, label, value, value_type) in enumerate(schedule_rows, start=1)
    )
    thrust_schedule = profile.thrust_profile_schedule
    thrust_schedule_rows: tuple[tuple[str, str, object, str], ...] = (
        (
            "thrust_profile_schedule.contract",
            "Thrust Profile Schedule Contract",
            thrust_schedule.contract,
            "string",
        ),
        ("thrust_profile_schedule.id", "Thrust Profile Schedule ID", thrust_schedule.schedule_id, "string"),
        (
            "thrust_profile_schedule.fingerprint",
            "Thrust Profile Schedule Fingerprint",
            thrust_schedule.fingerprint,
            "string",
        ),
        ("thrust_profile_schedule.origin", "Thrust Profile Schedule Origin", thrust_schedule.origin, "string"),
        (
            "thrust_profile_schedule.confidence",
            "Thrust Profile Schedule Confidence",
            thrust_schedule.confidence,
            "string",
        ),
        ("thrust_profile_schedule.method", "Thrust Profile Schedule Method", thrust_schedule.method, "string"),
        (
            "thrust_profile_schedule.source_record_ids",
            "Thrust Profile Schedule Source Records",
            ", ".join(thrust_schedule.source_record_ids),
            "string",
        ),
        (
            "thrust_profile_schedule.point_count",
            "Thrust Profile Schedule Point Count",
            len(thrust_schedule.points),
            "integer",
        ),
        (
            "thrust_profile_schedule.points",
            "Thrust Profile Schedule Points",
            ", ".join(f"f{item.burn_fraction:g}:{item.multiplier:.9g}" for item in thrust_schedule.points),
            "string",
        ),
        (
            "thrust_profile_schedule.interpolation",
            "Thrust Profile Schedule Interpolation",
            thrust_schedule.interpolation,
            "string",
        ),
        (
            "thrust_profile_schedule.normalization",
            "Thrust Profile Schedule Normalization",
            thrust_schedule.normalization,
            "string",
        ),
        (
            "thrust_profile_schedule.depends_on",
            "Thrust Profile Schedule Resolver Dependencies",
            ", ".join(profile.thrust_profile_schedule_depends_on) or "none",
            "string",
        ),
        (
            "thrust_profile_schedule.raw_area",
            "Thrust Profile Schedule Raw Area",
            thrust_schedule.raw_area,
            "number",
        ),
        (
            "thrust_profile_schedule.normalized_peak",
            "Thrust Profile Schedule Normalized Peak",
            thrust_schedule.normalized_peak,
            "number",
        ),
    )
    thrust_base_order = base_order + (len(schedule_rows) + 1) * 10
    properties.extend(
        _property(
            profile,
            identifier,
            label,
            value,
            value_type,
            "implementation",
            "1" if value_type == "number" else None,
            thrust_base_order + index * 10,
        )
        for index, (identifier, label, value, value_type) in enumerate(thrust_schedule_rows, start=1)
    )
    next_structured_order = thrust_base_order + (len(thrust_schedule_rows) + 1) * 10
    second_thrust_schedule = profile.second_pulse_thrust_profile_schedule
    if second_thrust_schedule is not None:
        second_schedule_rows: tuple[tuple[str, str, object, str], ...] = (
            (
                "second_pulse_thrust_profile_schedule.contract",
                "Second-Pulse Thrust Schedule Contract",
                second_thrust_schedule.contract,
                "string",
            ),
            (
                "second_pulse_thrust_profile_schedule.id",
                "Second-Pulse Thrust Schedule ID",
                second_thrust_schedule.schedule_id,
                "string",
            ),
            (
                "second_pulse_thrust_profile_schedule.fingerprint",
                "Second-Pulse Thrust Schedule Fingerprint",
                second_thrust_schedule.fingerprint,
                "string",
            ),
            (
                "second_pulse_thrust_profile_schedule.origin",
                "Second-Pulse Thrust Schedule Origin",
                second_thrust_schedule.origin,
                "string",
            ),
            (
                "second_pulse_thrust_profile_schedule.confidence",
                "Second-Pulse Thrust Schedule Confidence",
                second_thrust_schedule.confidence,
                "string",
            ),
            (
                "second_pulse_thrust_profile_schedule.method",
                "Second-Pulse Thrust Schedule Method",
                second_thrust_schedule.method,
                "string",
            ),
            (
                "second_pulse_thrust_profile_schedule.source_record_ids",
                "Second-Pulse Thrust Schedule Source Records",
                ", ".join(second_thrust_schedule.source_record_ids),
                "string",
            ),
            (
                "second_pulse_thrust_profile_schedule.points",
                "Second-Pulse Thrust Schedule Points",
                ", ".join(f"f{item.burn_fraction:g}:{item.multiplier:.9g}" for item in second_thrust_schedule.points),
                "string",
            ),
            (
                "second_pulse_thrust_profile_schedule.interpolation",
                "Second-Pulse Thrust Schedule Interpolation",
                second_thrust_schedule.interpolation,
                "string",
            ),
            (
                "second_pulse_thrust_profile_schedule.normalization",
                "Second-Pulse Thrust Schedule Normalization",
                second_thrust_schedule.normalization,
                "string",
            ),
            (
                "second_pulse_thrust_profile_schedule.raw_area",
                "Second-Pulse Thrust Schedule Raw Area",
                second_thrust_schedule.raw_area,
                "number",
            ),
            (
                "second_pulse_thrust_profile_schedule.normalized_peak",
                "Second-Pulse Thrust Schedule Normalized Peak",
                second_thrust_schedule.normalized_peak,
                "number",
            ),
        )
        properties.extend(
            _property(
                profile,
                identifier,
                label,
                value,
                value_type,
                "implementation",
                "1" if value_type == "number" else None,
                next_structured_order + index * 10,
            )
            for index, (identifier, label, value, value_type) in enumerate(
                second_schedule_rows,
                start=1,
            )
        )
        next_structured_order += (len(second_schedule_rows) + 1) * 10
    thrust_curve = profile.thrust_time_curve
    if thrust_curve is not None:
        curve_rows: tuple[tuple[str, str, object, str, str | None], ...] = (
            ("thrust_time_curve.contract", "Absolute Thrust Curve Contract", thrust_curve.contract, "string", None),
            ("thrust_time_curve.id", "Absolute Thrust Curve ID", thrust_curve.curve_id, "string", None),
            (
                "thrust_time_curve.fingerprint",
                "Absolute Thrust Curve Fingerprint",
                thrust_curve.fingerprint,
                "string",
                None,
            ),
            ("thrust_time_curve.origin", "Absolute Thrust Curve Origin", thrust_curve.origin, "string", None),
            (
                "thrust_time_curve.confidence",
                "Absolute Thrust Curve Confidence",
                thrust_curve.confidence,
                "string",
                None,
            ),
            ("thrust_time_curve.method", "Absolute Thrust Curve Method", thrust_curve.method, "string", None),
            (
                "thrust_time_curve.source_record_ids",
                "Absolute Thrust Curve Source Records",
                ", ".join(thrust_curve.source_record_ids),
                "string",
                None,
            ),
            (
                "thrust_time_curve.interpolation",
                "Absolute Thrust Curve Interpolation",
                thrust_curve.interpolation,
                "string",
                None,
            ),
            ("thrust_time_curve.time_unit", "Absolute Thrust Curve Time Unit", thrust_curve.time_unit, "string", None),
            (
                "thrust_time_curve.thrust_unit",
                "Absolute Thrust Curve Force Unit",
                thrust_curve.thrust_unit,
                "string",
                None,
            ),
            (
                "thrust_time_curve.points",
                "Absolute Thrust Curve Source Points",
                ", ".join(f"t{item.time:g}:{item.thrust:.9g}" for item in thrust_curve.points),
                "string",
                None,
            ),
            (
                "thrust_time_curve.point_count",
                "Absolute Thrust Curve Point Count",
                len(thrust_curve.points),
                "integer",
                None,
            ),
            (
                "thrust_time_curve.duration",
                "Absolute Thrust Curve Duration",
                thrust_curve.duration_s,
                "number",
                "s",
            ),
            (
                "thrust_time_curve.mean_thrust",
                "Absolute Thrust Curve Mean Thrust",
                thrust_curve.mean_thrust_n,
                "number",
                "N",
            ),
            (
                "thrust_time_curve.total_impulse",
                "Absolute Thrust Curve Total Impulse",
                thrust_curve.total_impulse_n_s,
                "number",
                "N*s",
            ),
        )
        properties.extend(
            _property(
                profile,
                identifier,
                label,
                value,
                value_type,
                "evidence",
                unit,
                next_structured_order + index * 10,
            )
            for index, (identifier, label, value, value_type, unit) in enumerate(curve_rows, start=1)
        )
        next_structured_order += (len(curve_rows) + 1) * 10
    dual_program = profile.dual_pulse_thrust_program
    if dual_program is not None:
        dual_rows: tuple[tuple[str, str, object, str, str | None], ...] = (
            (
                "dual_pulse_thrust_program.contract",
                "Dual-Pulse Thrust Program Contract",
                dual_program.contract,
                "string",
                None,
            ),
            (
                "dual_pulse_thrust_program.id",
                "Dual-Pulse Thrust Program ID",
                dual_program.program_id,
                "string",
                None,
            ),
            (
                "dual_pulse_thrust_program.fingerprint",
                "Dual-Pulse Thrust Program Fingerprint",
                dual_program.fingerprint,
                "string",
                None,
            ),
            (
                "dual_pulse_thrust_program.origin",
                "Dual-Pulse Thrust Program Origin",
                dual_program.origin,
                "string",
                None,
            ),
            (
                "dual_pulse_thrust_program.confidence",
                "Dual-Pulse Thrust Program Confidence",
                dual_program.confidence,
                "string",
                None,
            ),
            (
                "dual_pulse_thrust_program.method",
                "Dual-Pulse Thrust Program Method",
                dual_program.method,
                "string",
                None,
            ),
            (
                "dual_pulse_thrust_program.source_record_ids",
                "Dual-Pulse Thrust Program Source Records",
                ", ".join(dual_program.source_record_ids),
                "string",
                None,
            ),
            (
                "dual_pulse_thrust_program.coast_time.source",
                "Dual-Pulse Source Coast Time",
                dual_program.inter_pulse_coast_time,
                "number",
                dual_program.coast_time_unit,
            ),
            (
                "dual_pulse_thrust_program.coast_time.canonical",
                "Dual-Pulse Canonical Coast Time",
                dual_program.inter_pulse_coast_time_s,
                "number",
                "s",
            ),
            (
                "dual_pulse_thrust_program.active_burn_time",
                "Dual-Pulse Active Burn Time",
                dual_program.active_burn_time_s,
                "number",
                "s",
            ),
            (
                "dual_pulse_thrust_program.duration",
                "Dual-Pulse Program Duration",
                dual_program.duration_s,
                "number",
                "s",
            ),
            (
                "dual_pulse_thrust_program.mean_thrust",
                "Dual-Pulse Active-Burn Mean Thrust",
                dual_program.mean_active_burn_thrust_n,
                "number",
                "N",
            ),
            (
                "dual_pulse_thrust_program.total_impulse",
                "Dual-Pulse Total Impulse",
                dual_program.total_impulse_n_s,
                "number",
                "N*s",
            ),
            (
                "dual_pulse_thrust_program.second_to_first_mean_thrust_ratio",
                "Dual-Pulse Second/First Mean Thrust Ratio",
                dual_program.second_to_first_mean_thrust_ratio,
                "number",
                "1",
            ),
            *_absolute_curve_property_rows(
                dual_program.first_pulse,
                prefix="dual_pulse_thrust_program.first_pulse",
                label="First-Pulse",
            ),
            *_absolute_curve_property_rows(
                dual_program.second_pulse,
                prefix="dual_pulse_thrust_program.second_pulse",
                label="Second-Pulse",
            ),
        )
        properties.extend(
            _property(
                profile,
                identifier,
                label,
                value,
                value_type,
                "evidence",
                unit,
                next_structured_order + index * 10,
            )
            for index, (identifier, label, value, value_type, unit) in enumerate(
                dual_rows,
                start=1,
            )
        )
    return tuple(properties)
    ####


def _absolute_curve_property_rows(
    curve: AbsoluteThrustCurve,
    *,
    prefix: str,
    label: str,
) -> tuple[tuple[str, str, object, str, str | None], ...]:
    return (
        (f"{prefix}.id", f"{label} Absolute Curve ID", curve.curve_id, "string", None),
        (
            f"{prefix}.fingerprint",
            f"{label} Absolute Curve Fingerprint",
            curve.fingerprint,
            "string",
            None,
        ),
        (f"{prefix}.origin", f"{label} Absolute Curve Origin", curve.origin, "string", None),
        (
            f"{prefix}.confidence",
            f"{label} Absolute Curve Confidence",
            curve.confidence,
            "string",
            None,
        ),
        (f"{prefix}.method", f"{label} Absolute Curve Method", curve.method, "string", None),
        (
            f"{prefix}.source_record_ids",
            f"{label} Absolute Curve Source Records",
            ", ".join(curve.source_record_ids),
            "string",
            None,
        ),
        (f"{prefix}.time_unit", f"{label} Absolute Curve Time Unit", curve.time_unit, "string", None),
        (
            f"{prefix}.thrust_unit",
            f"{label} Absolute Curve Force Unit",
            curve.thrust_unit,
            "string",
            None,
        ),
        (
            f"{prefix}.points",
            f"{label} Absolute Curve Source Points",
            ", ".join(f"t{item.time:g}:{item.thrust:.9g}" for item in curve.points),
            "string",
            None,
        ),
        (f"{prefix}.duration", f"{label} Burn Duration", curve.duration_s, "number", "s"),
        (f"{prefix}.mean_thrust", f"{label} Mean Thrust", curve.mean_thrust_n, "number", "N"),
        (
            f"{prefix}.total_impulse",
            f"{label} Total Impulse",
            curve.total_impulse_n_s,
            "number",
            "N*s",
        ),
    )
    ####


def _profile_context_properties(
    profile: ResolvedInterceptorProfile,
) -> tuple[TrajectoryModelPropertyMetadata, ...]:
    """Advertise catalogue identity, gaps, intervals, and resolver status."""

    rows: list[tuple[str, str, object, str]] = [
        ("resolution_status", "Resolution Status", profile.resolution_status.value, "evidence"),
        ("evidence_gap_count", "Evidence Gap Count", len(profile.evidence_gaps), "evidence"),
    ]
    if profile.catalogue_interceptor_id is not None:
        rows.append(("catalogue_interceptor_id", "Catalogue Interceptor ID", profile.catalogue_interceptor_id, "identity"))
    if profile.variant_basis is not None:
        rows.append(("variant_basis", "Variant Basis", profile.variant_basis, "identity"))
    if profile.application_context is not None:
        rows.append(("application_context", "Application Context", profile.application_context, "identity"))
    if profile.required_diagnostics:
        rows.append(("required_diagnostics", "Required Diagnostics", ", ".join(profile.required_diagnostics), "evidence"))
    if profile.evidence_gaps:
        rows.append(
            (
                "evidence_gaps",
                "Unavailable Evidence",
                "; ".join(f"{item.parameter_id}:{item.status}" for item in profile.evidence_gaps),
                "evidence",
            )
        )
    if profile.evidence_intervals:
        rows.append(
            (
                "evidence_intervals",
                "Variant Evidence Intervals",
                "; ".join(_interval_summary(item) for item in profile.evidence_intervals),
                "evidence",
            )
        )
    return tuple(
        _property(
            profile,
            identifier,
            label,
            value,
            "integer" if isinstance(value, int) else "string",
            role,
            None,
            40 + index * 10,
        )
        for index, (identifier, label, value, role) in enumerate(rows)
    )
    ####


def _interval_summary(interval: EvidenceInterval) -> str:
    summary = f"{interval.parameter_id}=[{interval.minimum_value:g}, {interval.maximum_value:g}] {interval.unit}"
    if interval.source_minimum_value is not None and interval.source_maximum_value is not None and interval.source_unit is not None:
        summary += f" (source=[{interval.source_minimum_value:g}, {interval.source_maximum_value:g}] {interval.source_unit})"
    return summary
    ####


def _parameter_role(identifier: str) -> str:
    if identifier in {
        "length_m",
        "body_diameter_m",
        "wingspan_m",
        "reference_area_m2",
        "reference_length_m",
        "slenderness_ratio",
    }:
        return "geometry"
    if identifier in {"launch_mass_kg", "burnout_mass_kg", "propellant_fraction", "mass_flow_kg_s"}:
        return "mass"
    if identifier.startswith("reported_") or identifier in {
        "effective_specific_impulse_s",
        "nominal_thrust_n",
        "thrust_n",
        "active_burn_total_impulse_n_s",
    }:
        return "performance"
    if any(token in identifier for token in ("guidance", "maneuver", "control", "attitude", "body_rate", "body_acceleration", "bank_angle", "target_classes")):
        return "capability"
    return "implementation"
    ####


def _control_advertisement(
    profile: ResolvedInterceptorProfile,
    *,
    step_capable: bool,
) -> TrajectoryControlAdvertisement:
    value_space = ConfigurationValueSpace(
        topology="euclidean",
        representation="scalar",
        error_rule="subtraction",
        interpolation_rule="held constant",
    )
    direct_acceleration_limit_mps2 = profile.number("max_lateral_acceleration_mps2")
    direct_interval = ConfigurationInterval(
        minimum=ConfigurationBound(value=-direct_acceleration_limit_mps2),
        maximum=ConfigurationBound(value=direct_acceleration_limit_mps2),
    )
    direct_value_space = ConfigurationValueSpace(
        topology="bounded_interval",
        representation="scalar",
        error_rule="subtraction",
        interpolation_rule="held constant",
        normalization_rule="affine to [-1, 1]",
        coordinate_chart=(f"[-{direct_acceleration_limit_mps2:.9g}, {direct_acceleration_limit_mps2:.9g}] m/s^2"),
    )
    waypoint_specs = (
        (
            "navigation.waypoint.north.command",
            "Waypoint North",
            "m",
            None,
            "guidance.waypoint.north.accepted",
        ),
        (
            "navigation.waypoint.east.command",
            "Waypoint East",
            "m",
            None,
            "guidance.waypoint.east.accepted",
        ),
        (
            "navigation.waypoint.altitude.command",
            "Waypoint Altitude",
            "m",
            0.0,
            "guidance.waypoint.altitude.accepted",
        ),
        (
            "navigation.waypoint.capture_radius.command",
            "Waypoint Capture Radius",
            "m",
            0.1,
            "guidance.waypoint.capture_radius.accepted",
        ),
    )
    operations: tuple[Literal["batch", "step"], ...] = ("batch", "step") if step_capable else ("batch",)
    waypoint_channels = tuple(
        TrajectoryControlChannelMetadata(
            id=identifier,
            label=label,
            description=(
                f"Held {label.lower()} used by batch and live surrogate guidance."
                if step_capable
                else f"Batch-profile {label.lower()} used by the surrogate guidance law."
            ),
            channel_kind="action",
            quantity="length",
            canonical_unit=unit,
            display_unit=unit,
            interval=ConfigurationInterval(minimum=ConfigurationBound(value=lower)) if lower is not None else None,
            sampling_semantics="held_action" if step_capable else "batch_profile",
            value_space=value_space,
            semantics=ControlCommandSemantics(
                value_domain="continuous",
                command_mode="absolute",
                temporal_semantics="held" if step_capable else "profile",
                release_behavior="hold",
                agent_normalization="auto",
            ),
            availability="available" if step_capable else "available_in_batch",
            operations=operations,
            native_channel_id=identifier,
            native_binding=TrajectoryControlNativeBindingMetadata(
                id=identifier,
                quantity="length",
                canonical_unit=unit,
                interval=ConfigurationInterval(minimum=ConfigurationBound(value=lower)) if lower is not None else None,
                value_space=value_space,
                provider_binding={
                    "configuration_path": identifier,
                    "session_state_path": identifier,
                },
            ),
            provider_binding={
                "configuration_path": identifier,
                "session_state_path": identifier,
                "feedback_channel_id": feedback_channel_id,
                "feedback_group": "guidance",
            },
            presentation=ValuePresentationMetadata(group="waypoint guidance", order=index * 10, control="number_input"),
            provenance="parametric interceptor batch configuration",
            claim_boundary="Reduced-order waypoint coordinate; no seeker, datalink, or physical controller claim.",
        )
        for index, (identifier, label, unit, lower, feedback_channel_id) in enumerate(
            waypoint_specs,
            start=1,
        )
    )
    target_specs = (
        (
            "navigation.target.position.north.command",
            "Target Reference North",
            "length",
            "m",
            None,
            "local_ned",
            "guidance.target.reference_position.north.accepted",
        ),
        (
            "navigation.target.position.east.command",
            "Target Reference East",
            "length",
            "m",
            None,
            "local_ned",
            "guidance.target.reference_position.east.accepted",
        ),
        (
            "navigation.target.position.altitude.command",
            "Target Reference Altitude",
            "length",
            "m",
            0.0,
            "local_ned",
            "guidance.target.reference_position.altitude.accepted",
        ),
        (
            "navigation.target.velocity.north.command",
            "Target North Velocity",
            "speed",
            "m/s",
            None,
            "local_ned",
            "guidance.target.velocity.north.accepted",
        ),
        (
            "navigation.target.velocity.east.command",
            "Target East Velocity",
            "speed",
            "m/s",
            None,
            "local_ned",
            "guidance.target.velocity.east.accepted",
        ),
        (
            "navigation.target.velocity.vertical.command",
            "Target Vertical Velocity",
            "speed",
            "m/s",
            None,
            "local_ned",
            "guidance.target.velocity.vertical.accepted",
        ),
        (
            "navigation.target.capture_radius.command",
            "Target Intercept Radius",
            "length",
            "m",
            0.1,
            None,
            "guidance.target.capture_radius.accepted",
        ),
    )
    target_channels = tuple(
        TrajectoryControlChannelMetadata(
            id=identifier,
            label=label,
            description=(
                f"Held {label.lower()} used by batch and live constant-velocity target guidance."
                if step_capable
                else f"Batch-profile {label.lower()} used by constant-velocity target guidance."
            ),
            channel_kind="action",
            quantity=quantity,
            canonical_unit=unit,
            display_unit=unit,
            frame=frame,
            interval=ConfigurationInterval(minimum=ConfigurationBound(value=lower)) if lower is not None else None,
            sampling_semantics="held_action" if step_capable else "batch_profile",
            value_space=value_space,
            semantics=ControlCommandSemantics(
                value_domain="continuous",
                command_mode="absolute",
                temporal_semantics="held" if step_capable else "profile",
                release_behavior="hold",
                agent_normalization="auto",
            ),
            availability="available" if step_capable else "available_in_batch",
            operations=operations,
            native_channel_id=identifier,
            native_binding=TrajectoryControlNativeBindingMetadata(
                id=identifier,
                quantity=quantity,
                canonical_unit=unit,
                interval=ConfigurationInterval(minimum=ConfigurationBound(value=lower)) if lower is not None else None,
                value_space=value_space,
                provider_binding={
                    "configuration_path": identifier,
                    "session_state_path": identifier,
                },
            ),
            provider_binding={
                "configuration_path": identifier,
                "session_state_path": identifier,
                "feedback_channel_id": feedback_channel_id,
                "feedback_group": "guidance",
            },
            presentation=ValuePresentationMetadata(group="target-track guidance", order=index * 10, control="number_input"),
            provenance="parametric interceptor target-track grammar",
            claim_boundary=(
                "Caller-supplied constant-velocity target truth is measured by the selected registered direct-geometry "
                "relative-state provider; no propagation, signature, gimbal, track-manager, physical-seeker, datalink, "
                "target-dynamics, or physical-controller claim. Vertical velocity is positive upward."
            ),
        )
        for index, (identifier, label, quantity, unit, lower, frame, feedback_channel_id) in enumerate(
            target_specs,
            start=1,
        )
    )
    direct_specs = (
        (
            "control.lateral_acceleration.local.north.command",
            "Direct Lateral Acceleration North",
            "control.lateral_acceleration.accepted.local.north",
        ),
        (
            "control.lateral_acceleration.local.east.command",
            "Direct Lateral Acceleration East",
            "control.lateral_acceleration.accepted.local.east",
        ),
        (
            "control.lateral_acceleration.local.vertical.command",
            "Direct Lateral Acceleration Vertical",
            "control.lateral_acceleration.accepted.local.vertical",
        ),
    )
    direct_channels = tuple(
        TrajectoryControlChannelMetadata(
            id=identifier,
            label=label,
            description=(
                f"Held {label.lower()} for the shared reduced-order force-authority path. "
                "The runtime projects the local-NEU vector transverse to current velocity before allocation. Each "
                f"component is bounded to +/-{direct_acceleration_limit_mps2:.9g} m/s^2 for portable normalization; "
                "the vector norm and instantaneous force authority can still cause achieved-command saturation."
            ),
            channel_kind="action",
            quantity="acceleration",
            canonical_unit="m/s^2",
            display_unit="m/s^2",
            frame="local_neu",
            interval=direct_interval,
            sampling_semantics="held_action" if step_capable else "batch_profile",
            value_space=direct_value_space,
            semantics=ControlCommandSemantics(
                value_domain="continuous",
                command_mode="absolute",
                temporal_semantics="held" if step_capable else "profile",
                release_behavior="hold",
                agent_normalization="affine",
                agent_clip=True,
            ),
            availability="available" if step_capable else "available_in_batch",
            operations=operations,
            native_channel_id=identifier,
            native_binding=TrajectoryControlNativeBindingMetadata(
                id=identifier,
                quantity="acceleration",
                canonical_unit="m/s^2",
                interval=direct_interval,
                value_space=direct_value_space,
                provider_binding={
                    "configuration_path": identifier,
                    "session_state_path": identifier,
                },
            ),
            provider_binding={
                "configuration_path": identifier,
                "session_state_path": identifier,
                "feedback_channel_id": feedback_channel_id,
                "feedback_group": "direct lateral acceleration",
            },
            presentation=ValuePresentationMetadata(
                group="direct lateral acceleration",
                order=index * 10,
                control="number_input",
            ),
            provenance="parametric interceptor shared reduced-order force-authority input",
            claim_boundary=(
                "External reduced-order acceleration demand only; not a control-surface, actuator, autopilot, "
                "seeker, or rigid-body moment command. The symmetric interval is a per-axis structural envelope, not "
                "a promise that every combined vector is achievable. Feedback confirms the raw accepted hold; "
                "projected and achieved response remain separate outputs. Vertical is positive upward."
            ),
        )
        for index, (identifier, label, feedback_channel_id) in enumerate(direct_specs, start=1)
    )
    channels = waypoint_channels + target_channels + direct_channels
    authorities: list[TrajectoryControlAuthorityMetadata] = [
        TrajectoryControlAuthorityMetadata(
            id="waypoint_guidance",
            authority="mission",
            availability="available_in_batch",
            channel_ids=tuple(item.id for item in waypoint_channels),
            operations=("batch",),
            description="Caller-supplied fixed waypoint lowered into the selected reduced-order response tier.",
            command_owner="caller",
            selection_scope="batch",
            switching_policy="locked",
            scheme_id="mission.waypoint",
            applicable_phase_ids=("boost", "waypoint_guidance", "waypoint_capture"),
            lowering_chain=("waypoint_profile", "bounded_direction_response", "selected_fidelity_response"),
            provenance="parametric interceptor reduced-order kernels",
            claim_boundary="Fixed batch waypoint authority; no seeker, datalink, or controller claim.",
        ),
        TrajectoryControlAuthorityMetadata(
            id="target_track_guidance",
            authority="mission",
            availability="available_in_batch",
            channel_ids=tuple(item.id for item in target_channels),
            operations=("batch",),
            description="Caller-supplied constant-velocity target track lowered into the selected response tier.",
            command_owner="caller",
            selection_scope="batch",
            switching_policy="locked",
            scheme_id="mission.target_track",
            scheme_layer="mission",
            consumer_roles=("autonomy", "remote_operator"),
            streaming_preference="alternative",
            ui_order=20,
            applicable_phase_ids=("boost", "target_track_guidance", "target_intercept"),
            lowering_chain=(
                "constant_velocity_target_propagation",
                "taoryx_registered_relative_state_track",
                "target_relative_guidance",
                "selected_fidelity_response",
            ),
            provenance="parametric interceptor target-track kernels",
            claim_boundary=(
                "External target truth measured through the selected registered direct-geometry relative-state provider; "
                "no physical-seeker, propagation, track-manager, datalink, or controller claim."
            ),
        ),
        TrajectoryControlAuthorityMetadata(
            id="direct_lateral_acceleration",
            authority="kinematic",
            availability="available_in_batch",
            channel_ids=tuple(item.id for item in direct_channels),
            operations=("batch",),
            description="Caller-supplied local-NEU lateral acceleration lowered through the shared force-authority evaluator.",
            command_owner="caller",
            selection_scope="batch",
            switching_policy="locked",
            scheme_id="control.lateral_acceleration",
            scheme_layer="kinematic",
            consumer_roles=("autonomy", "remote_operator", "test_engineer"),
            streaming_preference="alternative",
            ui_order=30,
            applicable_phase_ids=("direct_lateral_acceleration_control",),
            lowering_chain=(
                "held_local_neu_acceleration",
                "velocity_transverse_projection",
                "shared_force_authority_allocation",
                "selected_fidelity_response",
            ),
            provenance="parametric interceptor shared point-mass/pseudo-6DOF control-authority path",
            claim_boundary=(
                "Reduced-order force demand for controller integration; no actuator, control-surface, autopilot, or rigid-body moment implementation."
            ),
        ),
    ]
    if step_capable:
        authorities.append(
            TrajectoryControlAuthorityMetadata(
                id="live_waypoint_guidance",
                authority="mission",
                availability="available",
                channel_ids=tuple(item.id for item in waypoint_channels),
                operations=("step",),
                description="Caller-held waypoint that may be retargeted at any accepted session step boundary.",
                command_owner="caller",
                selection_scope="session",
                switching_policy="locked",
                scheme_id="mission.waypoint.live",
                scheme_layer="mission",
                consumer_roles=("autonomy", "remote_operator"),
                streaming_preference="primary",
                ui_order=10,
                applicable_phase_ids=("boost", "waypoint_guidance", "waypoint_capture"),
                lowering_chain=("live_waypoint_hold", "bounded_direction_response", "selected_fidelity_state_advance"),
                provenance="parametric interceptor point-mass and pseudo-6DOF session kernels",
                claim_boundary=(
                    "Live reduced-order waypoint authority with held commands and explicit feedback; "
                    "no seeker, datalink, autopilot, or controller qualification claim."
                ),
            )
        )
        authorities.append(
            TrajectoryControlAuthorityMetadata(
                id="live_target_track_guidance",
                authority="mission",
                availability="available",
                channel_ids=tuple(item.id for item in target_channels),
                operations=("step",),
                description="Caller-held constant-velocity target track that may be updated at a session step boundary.",
                command_owner="caller",
                selection_scope="session",
                switching_policy="locked",
                scheme_id="mission.target_track.live",
                scheme_layer="mission",
                consumer_roles=("autonomy", "remote_operator"),
                streaming_preference="primary",
                ui_order=20,
                applicable_phase_ids=("boost", "target_track_guidance", "target_intercept"),
                lowering_chain=(
                    "live_target_track_hold",
                    "constant_velocity_target_propagation",
                    "taoryx_registered_relative_state_track",
                    "target_relative_guidance",
                    "selected_fidelity_state_advance",
                ),
                provenance="parametric interceptor point-mass and pseudo-6DOF session kernels",
                claim_boundary=(
                    "Live external constant-velocity target truth is measured by the selected registered direct-geometry "
                    "relative-state provider with held commands and explicit feedback; no physical-seeker, propagation, "
                    "track-manager, datalink, target-dynamics, or controller qualification claim."
                ),
            )
        )
        authorities.append(
            TrajectoryControlAuthorityMetadata(
                id="live_direct_lateral_acceleration",
                authority="kinematic",
                availability="available",
                channel_ids=tuple(item.id for item in direct_channels),
                operations=("step",),
                description="Caller-held local-NEU lateral acceleration updated through the standard session API.",
                command_owner="caller",
                selection_scope="session",
                switching_policy="locked",
                scheme_id="control.lateral_acceleration.live",
                scheme_layer="kinematic",
                consumer_roles=("autonomy", "remote_operator", "test_engineer"),
                streaming_preference="primary",
                ui_order=30,
                applicable_phase_ids=("direct_lateral_acceleration_control",),
                lowering_chain=(
                    "live_local_neu_acceleration_hold",
                    "velocity_transverse_projection",
                    "shared_force_authority_allocation",
                    "selected_fidelity_state_advance",
                ),
                provenance="parametric interceptor point-mass and pseudo-6DOF session kernels",
                claim_boundary=(
                    "Live reduced-order force demand with held commands and explicit feedback; no actuator, "
                    "control-surface, autopilot, or rigid-body qualification."
                ),
            )
        )
    intents = [
        TrajectoryControlIntentMetadata(
            id="fixed_waypoint_intercept",
            label="Fixed Waypoint Intercept",
            description="Guide the selected surrogate toward one caller-selected local waypoint.",
            resolution="external_channel",
            segment_ids=("boost", "waypoint_guidance"),
            mission_template_ids=(MISSION_TEMPLATE_ID,),
            channel_ids=tuple(item.id for item in waypoint_channels),
            operations=("batch",),
            provenance="parametric interceptor mission grammar",
            claim_boundary="A reduced mission intent, not terminal-homing qualification.",
        ),
        TrajectoryControlIntentMetadata(
            id="constant_velocity_target_intercept",
            label="Constant-Velocity Target Intercept",
            description="Guide the selected surrogate toward one caller-supplied constant-velocity local target track.",
            resolution="external_channel",
            segment_ids=("boost", "target_track_guidance"),
            mission_template_ids=(TARGET_TRACK_MISSION_TEMPLATE_ID,),
            channel_ids=tuple(item.id for item in target_channels),
            operations=("batch",),
            provenance="parametric interceptor target-track mission grammar",
            claim_boundary="A reduced kinematic-track intent, not terminal-homing or tracker qualification.",
        ),
        TrajectoryControlIntentMetadata(
            id="direct_lateral_acceleration_control",
            label="Direct Lateral Acceleration Control",
            description="Drive the shared reduced-order lateral-force authority from an external controller.",
            resolution="external_channel",
            segment_ids=("direct_lateral_acceleration_control",),
            mission_template_ids=(DIRECT_ACCELERATION_MISSION_TEMPLATE_ID,),
            channel_ids=tuple(item.id for item in direct_channels),
            operations=("batch",),
            provenance="parametric interceptor direct-control mission grammar",
            claim_boundary="Reduced-order acceleration-control intent; not actuator or autopilot qualification.",
        ),
    ]
    if step_capable:
        intents.append(
            TrajectoryControlIntentMetadata(
                id="live_waypoint_retargeting",
                label="Live Waypoint Retargeting",
                description="Hold or replace a local waypoint through the standard Composition session API.",
                resolution="external_channel",
                segment_ids=("boost", "waypoint_guidance"),
                mission_template_ids=(MISSION_TEMPLATE_ID,),
                channel_ids=tuple(item.id for item in waypoint_channels),
                operations=("step",),
                provenance="parametric interceptor session grammar",
                claim_boundary="Reduced-order streaming intent; not seeker, datalink, or engagement qualification.",
            )
        )
        intents.append(
            TrajectoryControlIntentMetadata(
                id="live_target_track_updates",
                label="Live Target-Track Updates",
                description="Hold or replace a constant-velocity target track through the Composition session API.",
                resolution="external_channel",
                segment_ids=("boost", "target_track_guidance"),
                mission_template_ids=(TARGET_TRACK_MISSION_TEMPLATE_ID,),
                channel_ids=tuple(item.id for item in target_channels),
                operations=("step",),
                provenance="parametric interceptor target-track session grammar",
                claim_boundary="Reduced-order streaming target state; not seeker, tracker, or datalink qualification.",
            )
        )
        intents.append(
            TrajectoryControlIntentMetadata(
                id="live_direct_lateral_acceleration_control",
                label="Live Direct Lateral Acceleration Control",
                description="Hold or replace a local-NEU lateral-acceleration command through the Composition session API.",
                resolution="external_channel",
                segment_ids=("direct_lateral_acceleration_control",),
                mission_template_ids=(DIRECT_ACCELERATION_MISSION_TEMPLATE_ID,),
                channel_ids=tuple(item.id for item in direct_channels),
                operations=("step",),
                provenance="parametric interceptor direct-control session grammar",
                claim_boundary="Reduced-order streaming force demand; not actuator, autopilot, or controller qualification.",
            )
        )
    return TrajectoryControlAdvertisement(
        status="available",
        channels=channels,
        authorities=tuple(authorities),
        intents=tuple(intents),
        default_authority_id=("live_waypoint_guidance" if step_capable else "waypoint_guidance"),
        claim_boundary=(
            "Controls are typed batch/live waypoint, constant-velocity target-track, or direct local-NEU lateral-acceleration "
            "inputs with accepted commands, achieved steering, availability, and saturation readback at both runtime tiers. "
            "Non-default controls from an inactive mission grammar fail configuration validation."
            if step_capable
            else (
                "Typed batch waypoint, target-track, or direct-acceleration inputs with accepted/projected demand, "
                "relative-state, achieved steering, availability, range, and saturation readback; non-default inactive "
                "mission controls fail configuration validation."
            )
        ),
    )
    ####


def _output_schema(profile: ResolvedInterceptorProfile) -> TrajectoryOutputSchema:
    core_specs = (
        ("position.local.north", "North Position", "length", "m", "local_ned", "float64", "linear"),
        ("position.local.east", "East Position", "length", "m", "local_ned", "float64", "linear"),
        ("position.geometric.altitude", "Altitude", "length", "m", "local_ned", "float64", "linear"),
        ("velocity.local.north", "North Velocity", "speed", "m/s", "local_ned", "float64", "linear"),
        ("velocity.local.east", "East Velocity", "speed", "m/s", "local_ned", "float64", "linear"),
        ("velocity.local.vertical", "Vertical Velocity", "speed", "m/s", "local_ned", "float64", "linear"),
        ("velocity.speed", "Speed", "speed", "m/s", None, "float64", "linear"),
    )
    telemetry_specs = (
        ("phase.id", "Flight Phase", None, None, None, "string", "step", "mission"),
        ("vehicle.operational", "Vehicle Operational", None, None, None, "boolean", "step", "mission"),
        ("model.applicability.declared", "Applicability Envelope Declared", None, None, None, "boolean", "step", "applicability"),
        ("model.applicability.status", "Applicability Status", None, None, None, "string", "step", "applicability"),
        ("model.applicability.reason", "Applicability Reason", None, None, None, "string", "step", "applicability"),
        ("mass.total", "Mass", "mass", "kg", None, "float64", "linear", "resources"),
        ("propulsion.thrust", "Thrust", "force", "N", None, "float64", "linear", "propulsion"),
        ("propulsion.thrust.axial", "Axial Thrust", "force", "N", None, "float64", "linear", "propulsion"),
        (
            "propulsion.throttle.commanded",
            "Motor Schedule Command",
            "dimensionless",
            "1",
            None,
            "float64",
            "step",
            "propulsion",
        ),
        ("propulsion.throttle.achieved", "Achieved Motor Level", "dimensionless", "1", None, "float64", "linear", "propulsion"),
        ("propulsion.propellant.remaining", "Propellant Remaining", "mass", "kg", None, "float64", "linear", "resources"),
        ("propulsion.available", "Propulsive Thrust Available", None, None, None, "boolean", "step", "propulsion"),
        ("propulsion.phase", "Propulsion Phase", None, None, None, "string", "step", "propulsion"),
        ("propulsion.pulse.index", "Active Pulse Index", None, None, None, "int64", "step", "propulsion"),
        ("environment.density", "Air Density", "density", "kg/m^3", None, "float64", "linear", "environment"),
        ("environment.pressure", "Air Pressure", "pressure", "Pa", None, "float64", "linear", "environment"),
        ("environment.temperature", "Air Temperature", "temperature", "K", None, "float64", "linear", "environment"),
        ("environment.speed_of_sound", "Speed of Sound", "speed", "m/s", None, "float64", "linear", "environment"),
        ("environment.wind.ecfc.x", "ECFC Wind X", "speed", "m/s", "ecfc", "float64", "linear", "environment"),
        ("environment.wind.ecfc.y", "ECFC Wind Y", "speed", "m/s", "ecfc", "float64", "linear", "environment"),
        ("environment.wind.ecfc.z", "ECFC Wind Z", "speed", "m/s", "ecfc", "float64", "linear", "environment"),
        ("aerodynamics.airspeed", "Air-Relative Speed", "speed", "m/s", None, "float64", "linear", "aerodynamics"),
        ("aerodynamics.mach", "Mach Number", "dimensionless", "1", None, "float64", "linear", "aerodynamics"),
        ("aerodynamics.drag_coefficient", "Base Mach Drag Coefficient", "dimensionless", "1", None, "float64", "linear", "aerodynamics"),
        (
            "aerodynamics.maneuver.normal_force_coefficient",
            "Achieved Maneuver Normal-Force Coefficient",
            "dimensionless",
            "1",
            None,
            "float64",
            "linear",
            "aerodynamics",
        ),
        ("aerodynamics.maneuver.drag_factor", "Maneuver Drag Factor", "dimensionless", "1", None, "float64", "step", "aerodynamics"),
        ("aerodynamics.maneuver.drag_coefficient", "Maneuver Drag Coefficient", "dimensionless", "1", None, "float64", "linear", "aerodynamics"),
        ("aerodynamics.drag_coefficient.total", "Total Drag Coefficient", "dimensionless", "1", None, "float64", "linear", "aerodynamics"),
        ("aerodynamics.dynamic_pressure", "Dynamic Pressure", "pressure", "Pa", None, "float64", "linear", "aerodynamics"),
        ("aerodynamics.drag.base", "Base Mach Drag", "force", "N", None, "float64", "linear", "aerodynamics"),
        ("aerodynamics.drag.maneuver", "Maneuver Drag", "force", "N", None, "float64", "linear", "aerodynamics"),
        ("aerodynamics.drag", "Total Drag", "force", "N", None, "float64", "linear", "aerodynamics"),
        ("guidance.waypoint.north.accepted", "Accepted Waypoint North", "length", "m", "local_ned", "float64", "step", "guidance"),
        ("guidance.waypoint.east.accepted", "Accepted Waypoint East", "length", "m", "local_ned", "float64", "step", "guidance"),
        ("guidance.waypoint.altitude.accepted", "Accepted Waypoint Altitude", "length", "m", "local_ned", "float64", "step", "guidance"),
        (
            "guidance.waypoint.capture_radius.accepted",
            "Accepted Waypoint Capture Radius",
            "length",
            "m",
            None,
            "float64",
            "step",
            "guidance",
        ),
        ("guidance.waypoint.range", "Waypoint Range", "length", "m", "local_ned", "float64", "linear", "guidance"),
        ("guidance.objective.kind", "Guidance Objective Kind", None, None, None, "string", "step", "guidance"),
        ("guidance.objective.range", "Guidance Objective Range", "length", "m", None, "float64", "linear", "guidance"),
        ("guidance.objective.captured", "Guidance Objective Captured", None, None, None, "boolean", "step", "guidance"),
        (
            "guidance.objective.capture_occurred",
            "Guidance Objective Capture Occurred",
            None,
            None,
            None,
            "boolean",
            "step",
            "guidance",
        ),
        (
            "guidance.target.reference_position.north.accepted",
            "Accepted Target Reference North",
            "length",
            "m",
            "local_ned",
            "float64",
            "step",
            "guidance",
        ),
        (
            "guidance.target.reference_position.east.accepted",
            "Accepted Target Reference East",
            "length",
            "m",
            "local_ned",
            "float64",
            "step",
            "guidance",
        ),
        (
            "guidance.target.reference_position.altitude.accepted",
            "Accepted Target Reference Altitude",
            "length",
            "m",
            "local_ned",
            "float64",
            "step",
            "guidance",
        ),
        (
            "guidance.target.velocity.north.accepted",
            "Accepted Target North Velocity",
            "speed",
            "m/s",
            "local_ned",
            "float64",
            "step",
            "guidance",
        ),
        (
            "guidance.target.velocity.east.accepted",
            "Accepted Target East Velocity",
            "speed",
            "m/s",
            "local_ned",
            "float64",
            "step",
            "guidance",
        ),
        (
            "guidance.target.velocity.vertical.accepted",
            "Accepted Target Vertical Velocity",
            "speed",
            "m/s",
            "local_ned",
            "float64",
            "step",
            "guidance",
        ),
        (
            "guidance.target.capture_radius.accepted",
            "Accepted Target Intercept Radius",
            "length",
            "m",
            None,
            "float64",
            "step",
            "guidance",
        ),
        ("guidance.target.position.north", "Target North", "length", "m", "local_ned", "float64", "linear", "guidance"),
        ("guidance.target.position.east", "Target East", "length", "m", "local_ned", "float64", "linear", "guidance"),
        ("guidance.target.position.altitude", "Target Altitude", "length", "m", "local_ned", "float64", "linear", "guidance"),
        ("guidance.relative.velocity.north", "Relative North Velocity", "speed", "m/s", "local_ned", "float64", "linear", "guidance"),
        ("guidance.relative.velocity.east", "Relative East Velocity", "speed", "m/s", "local_ned", "float64", "linear", "guidance"),
        ("guidance.relative.velocity.vertical", "Relative Vertical Velocity", "speed", "m/s", "local_ned", "float64", "linear", "guidance"),
        (
            "guidance.relative.time_to_closest_approach",
            "Linear Time to Closest Approach",
            "time",
            "s",
            None,
            "float64",
            "linear",
            "guidance",
        ),
        (
            "guidance.relative.predicted_miss_distance",
            "Linear Predicted Miss Distance",
            "length",
            "m",
            None,
            "float64",
            "linear",
            "guidance",
        ),
        ("guidance.law.id", "Guidance Law", None, None, None, "string", "step", "guidance"),
        ("guidance.law.mode", "Guidance Law Runtime Mode", None, None, None, "string", "step", "guidance"),
        ("guidance.closing_speed", "Waypoint Closing Speed", "speed", "m/s", None, "float64", "linear", "guidance"),
        ("guidance.line_of_sight_rate", "Line-of-Sight Angular Rate", "angular_rate", "rad/s", None, "float64", "linear", "guidance"),
        ("guidance.navigation_constant", "Navigation Constant", "dimensionless", "1", None, "float64", "step", "guidance"),
        (
            "control.lateral_acceleration.accepted.local.north",
            "Accepted Direct Lateral Acceleration North",
            "acceleration",
            "m/s^2",
            "local_neu",
            "float64",
            "step",
            "controls",
        ),
        (
            "control.lateral_acceleration.accepted.local.east",
            "Accepted Direct Lateral Acceleration East",
            "acceleration",
            "m/s^2",
            "local_neu",
            "float64",
            "step",
            "controls",
        ),
        (
            "control.lateral_acceleration.accepted.local.vertical",
            "Accepted Direct Lateral Acceleration Vertical",
            "acceleration",
            "m/s^2",
            "local_neu",
            "float64",
            "step",
            "controls",
        ),
        ("guidance.lateral_acceleration.commanded", "Commanded Lateral Acceleration", "acceleration", "m/s^2", None, "float64", "linear", "guidance"),
        ("guidance.lateral_acceleration.achieved", "Achieved Lateral Acceleration", "acceleration", "m/s^2", None, "float64", "linear", "guidance"),
        (
            "guidance.lateral_acceleration.commanded.local.north",
            "Commanded Lateral Acceleration North",
            "acceleration",
            "m/s^2",
            "local_neu",
            "float64",
            "linear",
            "guidance",
        ),
        (
            "guidance.lateral_acceleration.commanded.local.east",
            "Commanded Lateral Acceleration East",
            "acceleration",
            "m/s^2",
            "local_neu",
            "float64",
            "linear",
            "guidance",
        ),
        (
            "guidance.lateral_acceleration.commanded.local.vertical",
            "Commanded Lateral Acceleration Vertical",
            "acceleration",
            "m/s^2",
            "local_neu",
            "float64",
            "linear",
            "guidance",
        ),
        (
            "guidance.lateral_acceleration.achieved.local.north",
            "Achieved Lateral Acceleration North",
            "acceleration",
            "m/s^2",
            "local_neu",
            "float64",
            "linear",
            "guidance",
        ),
        (
            "guidance.lateral_acceleration.achieved.local.east",
            "Achieved Lateral Acceleration East",
            "acceleration",
            "m/s^2",
            "local_neu",
            "float64",
            "linear",
            "guidance",
        ),
        (
            "guidance.lateral_acceleration.achieved.local.vertical",
            "Achieved Lateral Acceleration Vertical",
            "acceleration",
            "m/s^2",
            "local_neu",
            "float64",
            "linear",
            "guidance",
        ),
        (
            "guidance.lateral_acceleration.achievement_fraction",
            "Lateral Acceleration Command Achievement Fraction",
            "dimensionless",
            "1",
            None,
            "float64",
            "linear",
            "guidance",
        ),
        (
            "guidance.lateral_acceleration.direction_error.valid",
            "Lateral Acceleration Direction Error Valid",
            None,
            None,
            None,
            "boolean",
            "step",
            "guidance",
        ),
        (
            "guidance.lateral_acceleration.direction_error",
            "Lateral Acceleration Direction Error",
            "angle",
            "rad",
            None,
            "float64",
            "linear",
            "guidance",
        ),
        ("guidance.lateral_acceleration.limit", "Structural Lateral Acceleration Limit", "acceleration", "m/s^2", None, "float64", "step", "guidance"),
        (
            "guidance.lateral_acceleration.utilization",
            "Structural Lateral Acceleration Utilization",
            "dimensionless",
            "1",
            None,
            "float64",
            "linear",
            "guidance",
        ),
        ("guidance.available", "Guidance Available", None, None, None, "boolean", "step", "guidance"),
        ("control.authority.configuration", "Control Authority Configuration", None, None, None, "string", "step", "controls"),
        ("control.authority.aerodynamic", "Aerodynamic Lateral Authority", "acceleration", "m/s^2", None, "float64", "linear", "controls"),
        ("control.authority.thrust_vector", "Thrust-Vector Lateral Authority", "acceleration", "m/s^2", None, "float64", "linear", "controls"),
        ("control.authority.combined_unclipped", "Combined Unclipped Lateral Authority", "acceleration", "m/s^2", None, "float64", "linear", "controls"),
        ("control.authority.available", "Instantaneous Lateral Authority", "acceleration", "m/s^2", None, "float64", "linear", "controls"),
        ("control.authority.utilization", "Instantaneous Authority Utilization", "dimensionless", "1", None, "float64", "linear", "controls"),
        ("control.authority.structural_limit_active", "Authority Structurally Limited", None, None, None, "boolean", "step", "controls"),
        ("control.allocation.policy", "Control Allocation Policy", None, None, None, "string", "step", "controls"),
        (
            "control.allocation.aerodynamic_achieved",
            "Aerodynamic Lateral Acceleration Achieved",
            "acceleration",
            "m/s^2",
            None,
            "float64",
            "linear",
            "controls",
        ),
        (
            "control.allocation.thrust_vector_achieved",
            "Thrust-Vector Lateral Acceleration Achieved",
            "acceleration",
            "m/s^2",
            None,
            "float64",
            "linear",
            "controls",
        ),
        ("control.allocation.thrust_vector_angle", "Achieved Thrust-Vector Angle", "angle", "rad", None, "float64", "linear", "controls"),
        ("control.limited", "Control Limited", None, None, None, "boolean", "step", "controls"),
        ("control.limit.reason", "Control Limit Reason", None, None, None, "string", "step", "controls"),
    )
    pseudo6_specs = (
        ("control.attitude_response.authority_available", "Attitude-Response Authority Available", None, None, None, "boolean", "step", "controls"),
        (
            "control.attitude_response.command_support_fraction",
            "Attitude Command Support Fraction",
            "dimensionless",
            "1",
            None,
            "float64",
            "linear",
            "controls",
        ),
        ("control.attitude_response.authority_limited", "Attitude Response Authority Limited", None, None, None, "boolean", "step", "controls"),
        ("aerodynamics.flow_angles.valid", "Body Flow Angles Valid", None, None, None, "boolean", "step", "aerodynamics"),
        ("aerodynamics.relative_velocity.body.x", "Body-X Air-Relative Velocity", "speed", "m/s", "body", "float64", "linear", "aerodynamics"),
        ("aerodynamics.relative_velocity.body.y", "Body-Y Air-Relative Velocity", "speed", "m/s", "body", "float64", "linear", "aerodynamics"),
        ("aerodynamics.relative_velocity.body.z", "Body-Z Air-Relative Velocity", "speed", "m/s", "body", "float64", "linear", "aerodynamics"),
        ("aerodynamics.angle_of_attack", "Air-Relative Angle of Attack", "angle", "rad", "body", "float64", "linear", "aerodynamics"),
        ("aerodynamics.sideslip_angle", "Air-Relative Sideslip Angle", "angle", "rad", "body", "float64", "linear", "aerodynamics"),
        ("attitude.roll.commanded", "Commanded Roll", "angle", "rad", "local_ned", "float64", "linear", "attitude_response"),
        ("attitude.pitch.commanded", "Commanded Pitch", "angle", "rad", "local_ned", "float64", "linear", "attitude_response"),
        ("attitude.yaw.commanded", "Commanded Yaw", "angle", "rad", "local_ned", "float64", "linear", "attitude_response"),
        ("attitude.roll", "Roll", "angle", "rad", "local_ned", "float64", "linear", "attitude_response"),
        ("attitude.pitch", "Pitch", "angle", "rad", "local_ned", "float64", "linear", "attitude_response"),
        ("attitude.yaw", "Yaw", "angle", "rad", "local_ned", "float64", "linear", "attitude_response"),
        ("angular_velocity.body.p", "Body Roll Rate", "angular_rate", "rad/s", "body", "float64", "linear", "attitude_response"),
        ("angular_velocity.body.q", "Body Pitch Rate", "angular_rate", "rad/s", "body", "float64", "linear", "attitude_response"),
        ("angular_velocity.body.r", "Body Yaw Rate", "angular_rate", "rad/s", "body", "float64", "linear", "attitude_response"),
        ("angular_acceleration.body.p", "Body Roll Acceleration", "angular_acceleration", "rad/s^2", "body", "float64", "linear", "attitude_response"),
        ("angular_acceleration.body.q", "Body Pitch Acceleration", "angular_acceleration", "rad/s^2", "body", "float64", "linear", "attitude_response"),
        ("angular_acceleration.body.r", "Body Yaw Acceleration", "angular_acceleration", "rad/s^2", "body", "float64", "linear", "attitude_response"),
        ("sensor.imu.valid", "IMU Sample Valid", None, None, None, "boolean", "step", "sensors"),
        ("sensor.imu.interval", "IMU Sample Interval", "time", "s", None, "float64", "step", "sensors"),
        ("sensor.imu.sampled_at", "IMU Sample Time", "time", "s", None, "float64", "step", "sensors"),
        ("sensor.imu.available_at", "IMU Availability Time", "time", "s", None, "float64", "step", "sensors"),
        ("sensor.imu.latency", "IMU Delivery Latency", "time", "s", None, "float64", "step", "sensors"),
        ("sensor.imu.delivery_fresh", "Fresh IMU Delivery", None, None, None, "boolean", "step", "sensors"),
        ("sensor.imu.sequence", "IMU Packet Sequence", None, None, None, "int64", "step", "sensors"),
        ("sensor.imu.schema_id", "IMU Payload Schema", None, None, None, "string", "step", "sensors"),
        ("sensor.imu.delta_velocity.body.x", "IMU Body-X Delta Velocity", "speed", "m/s", "body", "float64", "step", "sensors"),
        ("sensor.imu.delta_velocity.body.y", "IMU Body-Y Delta Velocity", "speed", "m/s", "body", "float64", "step", "sensors"),
        ("sensor.imu.delta_velocity.body.z", "IMU Body-Z Delta Velocity", "speed", "m/s", "body", "float64", "step", "sensors"),
        ("sensor.imu.delta_angle.body.x", "IMU Body-X Delta Angle", "angle", "rad", "body", "float64", "step", "sensors"),
        ("sensor.imu.delta_angle.body.y", "IMU Body-Y Delta Angle", "angle", "rad", "body", "float64", "step", "sensors"),
        ("sensor.imu.delta_angle.body.z", "IMU Body-Z Delta Angle", "angle", "rad", "body", "float64", "step", "sensors"),
    )
    point_sensor_specs = (
        ("sensor.translation_acceleration.valid", "Translation Acceleration Sample Valid", None, None, None, "boolean", "step", "sensors"),
        ("sensor.translation_acceleration.interval", "Translation Acceleration Interval", "time", "s", None, "float64", "step", "sensors"),
        ("sensor.translation_acceleration.sampled_at", "Translation Sensor Sample Time", "time", "s", None, "float64", "step", "sensors"),
        ("sensor.translation_acceleration.available_at", "Translation Sensor Availability Time", "time", "s", None, "float64", "step", "sensors"),
        ("sensor.translation_acceleration.latency", "Translation Sensor Delivery Latency", "time", "s", None, "float64", "step", "sensors"),
        ("sensor.translation_acceleration.delivery_fresh", "Fresh Translation Sensor Delivery", None, None, None, "boolean", "step", "sensors"),
        ("sensor.translation_acceleration.sequence", "Translation Sensor Packet Sequence", None, None, None, "int64", "step", "sensors"),
        ("sensor.translation_acceleration.schema_id", "Translation Sensor Payload Schema", None, None, None, "string", "step", "sensors"),
        (
            "sensor.translation_acceleration.delta_velocity.eci.x",
            "Translation Sensor ECI-X Delta Velocity",
            "speed",
            "m/s",
            "eci",
            "float64",
            "step",
            "sensors",
        ),
        (
            "sensor.translation_acceleration.delta_velocity.eci.y",
            "Translation Sensor ECI-Y Delta Velocity",
            "speed",
            "m/s",
            "eci",
            "float64",
            "step",
            "sensors",
        ),
        (
            "sensor.translation_acceleration.delta_velocity.eci.z",
            "Translation Sensor ECI-Z Delta Velocity",
            "speed",
            "m/s",
            "eci",
            "float64",
            "step",
            "sensors",
        ),
    )
    target_sensor_specs = (
        ("sensor.target_track.applicable", "Target Track Sensor Applicable", None, None, None, "boolean", "step", "sensors"),
        ("sensor.target_track.valid", "Target Track Valid", None, None, None, "boolean", "step", "sensors"),
        ("sensor.target_track.sampled_at", "Target Track Sample Time", "time", "s", None, "float64", "step", "sensors"),
        ("sensor.target_track.available_at", "Target Track Availability Time", "time", "s", None, "float64", "step", "sensors"),
        ("sensor.target_track.latency", "Target Track Delivery Latency", "time", "s", None, "float64", "step", "sensors"),
        ("sensor.target_track.delivery_fresh", "Fresh Target Track Delivery", None, None, None, "boolean", "step", "sensors"),
        ("sensor.target_track.sequence", "Target Track Packet Sequence", None, None, None, "int64", "step", "sensors"),
        ("sensor.target_track.schema_id", "Target Track Payload Schema", None, None, None, "string", "step", "sensors"),
        ("sensor.target_track.invalid_reason", "Target Track Invalid Reason", None, None, None, "string", "step", "sensors"),
        ("sensor.target_track.target_id", "Target Track Entity ID", None, None, None, "string", "step", "sensors"),
        ("sensor.target_track.frame_id", "Target Track Frame ID", None, None, None, "string", "step", "sensors"),
        ("sensor.target_track.range", "Measured Target Range", "length", "m", "sensor", "float64", "linear", "sensors"),
        ("sensor.target_track.azimuth", "Measured Target Azimuth", "angle", "rad", "sensor", "float64", "linear", "sensors"),
        ("sensor.target_track.elevation", "Measured Target Elevation", "angle", "rad", "sensor", "float64", "linear", "sensors"),
        ("sensor.target_track.closing_speed", "Measured Target Closing Speed", "speed", "m/s", "sensor", "float64", "linear", "sensors"),
        ("sensor.target_track.relative_position.sensor.x", "Target Relative Sensor-X Position", "length", "m", "sensor", "float64", "linear", "sensors"),
        ("sensor.target_track.relative_position.sensor.y", "Target Relative Sensor-Y Position", "length", "m", "sensor", "float64", "linear", "sensors"),
        ("sensor.target_track.relative_position.sensor.z", "Target Relative Sensor-Z Position", "length", "m", "sensor", "float64", "linear", "sensors"),
        ("sensor.target_track.relative_velocity.sensor.x", "Target Relative Sensor-X Velocity", "speed", "m/s", "sensor", "float64", "linear", "sensors"),
        ("sensor.target_track.relative_velocity.sensor.y", "Target Relative Sensor-Y Velocity", "speed", "m/s", "sensor", "float64", "linear", "sensors"),
        ("sensor.target_track.relative_velocity.sensor.z", "Target Relative Sensor-Z Velocity", "speed", "m/s", "sensor", "float64", "linear", "sensors"),
        ("sensor.target_track.line_of_sight_rate.sensor.x", "Target LOS-Rate Sensor X", "angular_rate", "rad/s", "sensor", "float64", "linear", "sensors"),
        ("sensor.target_track.line_of_sight_rate.sensor.y", "Target LOS-Rate Sensor Y", "angular_rate", "rad/s", "sensor", "float64", "linear", "sensors"),
        ("sensor.target_track.line_of_sight_rate.sensor.z", "Target LOS-Rate Sensor Z", "angular_rate", "rad/s", "sensor", "float64", "linear", "sensors"),
    )
    all_fidelities = (POINT_MASS_FIDELITY_ID, PSEUDO6_FIDELITY_ID)
    all_realizations = (POINT_MASS_REALIZATION_ID, PSEUDO6_REALIZATION_ID)
    core = tuple(
        _output_channel(
            *spec,
            availability="guaranteed",
            group="core_state",
            fidelities=all_fidelities,
            realizations=all_realizations,
        )
        for spec in core_specs
    )
    telemetry = (
        tuple(
            _output_channel(
                *spec[:-1],
                availability="guaranteed",
                group=spec[-1],
                fidelities=all_fidelities,
                realizations=all_realizations,
            )
            for spec in telemetry_specs
        )
        + tuple(
            _output_channel(
                *spec[:-1],
                availability="guaranteed",
                group=spec[-1],
                fidelities=all_fidelities,
                realizations=all_realizations,
            )
            for spec in target_sensor_specs
        )
        + tuple(
            _output_channel(
                *spec[:-1],
                availability="guaranteed",
                group=spec[-1],
                fidelities=(POINT_MASS_FIDELITY_ID,),
                realizations=(POINT_MASS_REALIZATION_ID,),
                sampling_semantics="interval",
            )
            for spec in point_sensor_specs
        )
        + tuple(
            _output_channel(
                *spec[:-1],
                availability="guaranteed",
                group=spec[-1],
                fidelities=(PSEUDO6_FIDELITY_ID,),
                realizations=(PSEUDO6_REALIZATION_ID,),
                sampling_semantics="interval" if spec[-1] == "sensors" else "continuous_sample",
            )
            for spec in pseudo6_specs
        )
    )
    groups = tuple(
        TrajectoryTelemetryGroupMetadata(
            id=group,
            label=group.replace("_", " ").title(),
            description=f"{group.replace('_', ' ').title()} outputs from the resolved surrogate.",
            channel_ids=tuple(item.id for item in telemetry if item.presentation.group == group),
            default_selected=group in {"resources", "guidance", "controls", "applicability"},
            presentation=ValuePresentationMetadata(group="telemetry", order=index * 10),
        )
        for index, group in enumerate(
            (
                "mission",
                "applicability",
                "resources",
                "propulsion",
                "controls",
                "environment",
                "aerodynamics",
                "guidance",
                "attitude_response",
                "sensors",
            ),
            start=1,
        )
    )
    return TrajectoryOutputSchema(
        model_id=profile.model_id,
        model_version=profile.parameter_set_version,
        core_channels=core,
        telemetry_channels=telemetry,
        telemetry_groups=groups,
        claim_boundary=(
            "Core state, guidance feedback, and standard environment/air-relative aerodynamic readback are available "
            "at both tiers. The standard relative-state target track is exposed at both tiers for target missions; "
            "registered translation-acceleration intervals are point-mass-only, while attitude truth and registered-IMU "
            "intervals are pseudo-6DOF-only."
        ),
    )
    ####


def _output_channel(
    identifier: str,
    label: str,
    quantity: str | None,
    unit: str | None,
    frame: str | None,
    data_type: str,
    interpolation: str,
    *,
    availability: str,
    group: str,
    fidelities: tuple[str, ...],
    realizations: tuple[str, ...],
    sampling_semantics: str = "continuous_sample",
) -> TrajectoryOutputChannelMetadata:
    descriptions = {
        "phase.id": "Runtime mission phase used by the standard Composition authority-availability contract.",
        "vehicle.operational": "False after a terminal ground-impact condition; completed episodes reject further control actions.",
        "model.applicability.declared": "True when at least one optional altitude or Mach applicability bound was authored; no archetype bounds are fabricated.",
        "model.applicability.status": "Advisory not_declared, within_declared_envelope, or outside_declared_envelope status; it does not silently terminate or gate the surrogate.",
        "model.applicability.reason": "Stable '+'-delimited altitude/Mach envelope reasons, none when inside, or not_declared when no bound exists.",
        "propulsion.throttle.commanded": "Binary ignition schedule from the resolved solid-motor program; this is not a caller-controlled throttle.",
        "propulsion.thrust.axial": "Current thrust projected onto the vehicle forward direction after the achieved thrust-vector allocation; never greater than total thrust.",
        "propulsion.throttle.achieved": "Instantaneous thrust normalized by the resolved program's peak scheduled thrust.",
        "propulsion.propellant.remaining": "Resolved surrogate propellant mass remaining after the current motor-schedule sample.",
        "propulsion.available": "True only while the current motor phase is producing propulsive thrust.",
        "propulsion.phase": "Current single-pulse, first-pulse, inter-pulse-coast, second-pulse, or burnout phase.",
        "propulsion.pulse.index": "One-based active pulse index, or zero while coasting or after burnout.",
        "environment.density": "Density sampled from the selected standard Taoryx environment provider.",
        "environment.pressure": "Pressure sampled from the selected standard Taoryx environment provider.",
        "environment.temperature": "Temperature sampled from the selected standard Taoryx environment provider.",
        "environment.speed_of_sound": "Speed of sound sampled from the selected standard Taoryx environment provider.",
        "environment.wind.ecfc.x": "Earth-fixed wind X component supplied by the standard environment provider.",
        "environment.wind.ecfc.y": "Earth-fixed wind Y component supplied by the standard environment provider.",
        "environment.wind.ecfc.z": "Earth-fixed wind Z component supplied by the standard environment provider.",
        "aerodynamics.airspeed": "Magnitude of vehicle velocity relative to the standard environment-provider wind.",
        "aerodynamics.mach": "Air-relative speed divided by the environment provider's speed of sound.",
        "aerodynamics.drag_coefficient": "Base coefficient interpolated from the resolved Mach-drag schedule at the current air-relative Mach number; maneuver drag is separate.",
        "aerodynamics.maneuver.normal_force_coefficient": "Normal-force coefficient reconstructed from the achieved aerodynamic lateral acceleration, current mass, dynamic pressure, and reference area.",
        "aerodynamics.maneuver.drag_factor": "Resolved nonnegative factor k in the reduced-order Cdm = k*Cn^2 maneuver-drag relation.",
        "aerodynamics.maneuver.drag_coefficient": "Load-dependent drag increment computed from the achieved aerodynamic normal-force coefficient.",
        "aerodynamics.drag_coefficient.total": "Sum of the base Mach-schedule coefficient and achieved maneuver-drag coefficient.",
        "aerodynamics.dynamic_pressure": "Dynamic pressure computed from environment density and air-relative speed.",
        "aerodynamics.drag.base": "Base drag force from dynamic pressure, reference area, and the resolved Mach-schedule coefficient.",
        "aerodynamics.drag.maneuver": "Additional drag force associated only with achieved aerodynamic lateral control force; thrust-vector authority does not create this term.",
        "aerodynamics.drag": "Total surrogate drag magnitude equal to base Mach drag plus achieved maneuver drag.",
        "guidance.law.id": (
            "Resolved simulation guidance archetype, or external_lateral_acceleration for the direct-control mission; "
            "distinct from the sourced seeker or homing-family evidence field."
        ),
        "guidance.law.mode": (
            "Active runtime law mode, including pursuit fallback for non-closing proportional-navigation geometry and "
            "direct_lateral_acceleration for an external local-NEU demand."
        ),
        "guidance.objective.kind": ("Selected fixed-waypoint, constant-velocity-target, or direct-lateral-acceleration objective grammar."),
        "guidance.objective.range": "Current three-dimensional range from the interceptor to the propagated guidance objective.",
        "guidance.objective.captured": "True when current objective range is within the accepted capture radius.",
        "guidance.objective.capture_occurred": "Batch terminal capture status or live-session latch showing that the current objective capture volume was entered since initialization or the last retarget.",
        "guidance.target.reference_position.north.accepted": "Accepted target-track north coordinate at its advertised reference epoch.",
        "guidance.target.reference_position.east.accepted": "Accepted target-track east coordinate at its advertised reference epoch.",
        "guidance.target.reference_position.altitude.accepted": "Accepted target-track geometric altitude at its advertised reference epoch.",
        "guidance.target.velocity.north.accepted": "Accepted constant target north velocity.",
        "guidance.target.velocity.east.accepted": "Accepted constant target east velocity.",
        "guidance.target.velocity.vertical.accepted": "Accepted constant target vertical velocity, positive upward.",
        "guidance.target.capture_radius.accepted": "Accepted three-dimensional target-intercept radius.",
        "guidance.target.position.north": "Propagated current target-track north coordinate.",
        "guidance.target.position.east": "Propagated current target-track east coordinate.",
        "guidance.target.position.altitude": "Propagated current target-track geometric altitude.",
        "guidance.relative.velocity.north": "Target north velocity minus interceptor north velocity.",
        "guidance.relative.velocity.east": "Target east velocity minus interceptor east velocity.",
        "guidance.relative.velocity.vertical": "Target vertical velocity minus interceptor vertical velocity, positive upward.",
        "guidance.relative.time_to_closest_approach": "Nonnegative closest-approach time from a constant relative-velocity linear projection.",
        "guidance.relative.predicted_miss_distance": "Closest-approach distance from a constant relative-velocity linear projection, not a terminal-hit prediction.",
        "guidance.closing_speed": "Target-relative closing speed along the current line of sight; negative means opening.",
        "guidance.line_of_sight_rate": "Magnitude of the inertial target-relative line-of-sight angular-rate vector.",
        "guidance.navigation_constant": "Resolved dimensionless proportional-navigation gain; advertised even when pursuit is selected.",
        "control.lateral_acceleration.accepted.local.north": "Raw held north component accepted by the direct-control grammar before projection transverse to current velocity; zero outside the direct-control mission.",
        "control.lateral_acceleration.accepted.local.east": "Raw held east component accepted by the direct-control grammar before projection transverse to current velocity; zero outside the direct-control mission.",
        "control.lateral_acceleration.accepted.local.vertical": "Raw held positive-up vertical component accepted by the direct-control grammar before projection transverse to current velocity; zero outside the direct-control mission.",
        "guidance.lateral_acceleration.commanded.local.north": "North component of the unbounded guidance-law lateral-acceleration request before current force or attitude realization.",
        "guidance.lateral_acceleration.commanded.local.east": "East component of the unbounded guidance-law lateral-acceleration request before current force or attitude realization.",
        "guidance.lateral_acceleration.commanded.local.vertical": "Positive-up vertical component of the unbounded guidance-law lateral-acceleration request before current force or attitude realization.",
        "guidance.lateral_acceleration.achieved.local.north": "North component actually realized by the force-limited point-mass response or the force- and attitude-limited pseudo-6DOF response.",
        "guidance.lateral_acceleration.achieved.local.east": "East component actually realized by the force-limited point-mass response or the force- and attitude-limited pseudo-6DOF response.",
        "guidance.lateral_acceleration.achieved.local.vertical": "Positive-up vertical component actually realized by the force-limited point-mass response or the force- and attitude-limited pseudo-6DOF response.",
        "guidance.lateral_acceleration.achievement_fraction": "Achieved lateral-acceleration magnitude divided by nonzero commanded magnitude and bounded to [0, 1]; zero command with zero achievement reports 1 by convention.",
        "guidance.lateral_acceleration.direction_error.valid": "True only when both commanded and achieved lateral-acceleration vectors have nonzero magnitude and therefore define a directional comparison.",
        "guidance.lateral_acceleration.direction_error": "Angle between nonzero commanded and achieved lateral-acceleration vectors; zero when invalid and meaningful only when the adjacent validity channel is true.",
        "guidance.lateral_acceleration.limit": "Resolved structural maneuver envelope; instantaneous force authority may be lower.",
        "guidance.lateral_acceleration.utilization": "Achieved lateral acceleration divided by the structural maneuver envelope and bounded to [0, 1].",
        "control.authority.configuration": "Resolved aerodynamic, thrust-assisted, or mixed reduced-order force-authority configuration.",
        "control.authority.aerodynamic": "Instantaneous q*S*Cn/m aerodynamic lateral-acceleration authority before structural clipping.",
        "control.authority.thrust_vector": "Instantaneous thrust*sin(vector-angle)/mass lateral-acceleration authority before structural clipping; zero when thrust is unavailable.",
        "control.authority.combined_unclipped": "Sum of the enabled aerodynamic and thrust-vector authority components before structural clipping.",
        "control.authority.available": "Current lateral-acceleration authority after clipping the enabled force components to the structural maneuver envelope.",
        "control.authority.utilization": "Achieved lateral acceleration divided by current available authority and bounded to [0, 1].",
        "control.authority.structural_limit_active": "True when the combined force-authority surrogate exceeds and is clipped by the structural maneuver envelope.",
        "control.allocation.policy": "Resolved reduced-order rule used to divide achieved lateral demand between enabled authority components.",
        "control.allocation.aerodynamic_achieved": "Achieved lateral acceleration assigned to aerodynamic authority by the resolved allocation policy.",
        "control.allocation.thrust_vector_achieved": "Achieved lateral acceleration assigned to thrust-vector authority by the resolved allocation policy.",
        "control.allocation.thrust_vector_angle": "Thrust-vector angle needed for the allocated lateral thrust; zero when no thrust-vector contribution is achieved.",
        "control.attitude_response.authority_available": "True when the current aerodynamic/TVC force model provides nonzero lateral authority to support pseudo-6DOF attitude response.",
        "control.attitude_response.command_support_fraction": "Current available lateral authority divided by nonzero commanded lateral demand and bounded to [0, 1]; zero demand reports 1 by convention.",
        "control.attitude_response.authority_limited": "True when current lateral authority is below the nonzero guidance demand used to drive the pseudo-6DOF response law.",
        "aerodynamics.flow_angles.valid": "True when nonzero air-relative speed defines body-axis relative-wind direction; zero airspeed reports false and zero-valued angles.",
        "aerodynamics.relative_velocity.body.x": "Air-relative velocity projected onto the pseudo-6DOF response body's forward axis.",
        "aerodynamics.relative_velocity.body.y": "Air-relative velocity projected onto the pseudo-6DOF response body's right axis.",
        "aerodynamics.relative_velocity.body.z": "Air-relative velocity projected onto the pseudo-6DOF response body's down axis.",
        "aerodynamics.angle_of_attack": "Geometric air-relative atan2(body-down velocity, body-forward velocity); response-state telemetry, not an aerodynamic coefficient-deck input.",
        "aerodynamics.sideslip_angle": "Geometric air-relative atan2(body-right velocity, forward/down-plane speed); response-state telemetry, not an aerodynamic coefficient-deck input.",
        "control.limited": "True when requested or achieved control demand is clipped by an advertised force-authority, structural, attitude-response, or body-rate bound.",
        "control.limit.reason": "Stable '+'-delimited active limit codes in deterministic order, including aerodynamic_authority_saturation, thrust_vector_authority_saturation, or combined_authority_saturation; 'none' means no control bound is active.",
        "sensor.target_track.applicable": (
            "True only for the constant-velocity target mission; fixed-waypoint and direct-acceleration modes do not fabricate a target sensor sample."
        ),
        "sensor.target_track.valid": "True when the registered relative-state provider delivered a usable target packet at this accepted boundary.",
        "sensor.target_track.sampled_at": "Committed scene time at which the registered relative-state provider sampled the target.",
        "sensor.target_track.available_at": "Runtime time at which the target-track measurement became causally deliverable.",
        "sensor.target_track.latency": "Delivery delay between target-track sampling and availability.",
        "sensor.target_track.delivery_fresh": "True only when this boundary generated or released a new target-track packet; false for a held packet.",
        "sensor.target_track.sequence": "Monotonic generated target-track packet sequence, or -1 before any delivery.",
        "sensor.target_track.schema_id": "Versioned native Taoryx relative-state payload contract used by guidance.",
        "sensor.target_track.invalid_reason": "Stable reason for an unavailable target measurement, or 'none' for a valid track.",
        "sensor.target_track.range": "Direct sensor-frame geometric range after the selected provider's declared measurement transform.",
        "sensor.target_track.azimuth": "Measured target azimuth in the provider's sensor frame.",
        "sensor.target_track.elevation": "Measured target elevation in the provider's sensor frame.",
        "sensor.target_track.closing_speed": "Measured target-relative closing speed along the sensor line of sight.",
        "sensor.translation_acceleration.valid": "True after the standard translation adapter has two committed truth boundaries.",
        "sensor.translation_acceleration.interval": "Accepted truth interval integrated by the standard translation adapter.",
        "sensor.translation_acceleration.sampled_at": "Committed-truth time at which the registered translation sensor sampled.",
        "sensor.translation_acceleration.available_at": "Runtime time at which the registered translation measurement becomes deliverable.",
        "sensor.translation_acceleration.latency": "Advertised delivery delay between translation measurement sampling and availability.",
        "sensor.translation_acceleration.delivery_fresh": "True only at a committed boundary that released a new available translation packet; false while holding the prior delivery.",
        "sensor.translation_acceleration.sequence": "Monotonic generated-packet sequence, or -1 before any packet is delivered.",
        "sensor.translation_acceleration.schema_id": "Versioned Taoryx payload contract emitted by the selected translation sensor provider.",
        "sensor.imu.sampled_at": "Committed-truth time at which the registered IMU sampled.",
        "sensor.imu.available_at": "Runtime time at which the registered IMU measurement becomes deliverable.",
        "sensor.imu.latency": "Advertised delivery delay between IMU sampling and availability.",
        "sensor.imu.delivery_fresh": "True only at a committed boundary that released a new available IMU packet; false while holding the prior delivery.",
        "sensor.imu.sequence": "Monotonic generated-packet sequence, or -1 before any packet is delivered.",
        "sensor.imu.schema_id": "Versioned Taoryx payload contract emitted by the selected IMU provider.",
    }
    sensor_provenance = "selected registered Taoryx sensor plug-in over accepted surrogate truth"
    return TrajectoryOutputChannelMetadata(
        id=identifier,
        label=label,
        description=descriptions.get(identifier, f"{label} from the neutral parametric interceptor kernel."),
        quantity=quantity,
        canonical_unit=unit,
        display_unit=unit,
        data_type=data_type,  # type: ignore[arg-type]
        frame=frame,
        sampling_semantics=sampling_semantics,  # type: ignore[arg-type]
        interpolation=interpolation,  # type: ignore[arg-type]
        availability=availability,  # type: ignore[arg-type]
        compatible_fidelities=fidelities,
        compatible_realizations=realizations,
        compatible_mission_templates=_MISSION_TEMPLATE_IDS,
        operations=("batch", "step"),
        presentation=ValuePresentationMetadata(group=group),
        provenance=(sensor_provenance if group == "sensors" else "taoryx_parametric_interceptors reduced-order kernels"),
        claim_boundary=(
            "Registered sensor projection of accepted surrogate truth; inspect the selected suite before making any "
            "error-model claim, and never interpret it as seeker or hardware qualification."
            if group == "sensors"
            else "Surrogate output only; not source-model or rigid-body truth."
        ),
    )
    ####


def _mission_from_resolved(
    resolved: object,
    *,
    mission_template_id: str,
) -> PointMassMission:
    if not isinstance(resolved, Mapping):
        raise ValueError("resolved mission configuration must be a mapping")
    values = {name: _resolved_float(resolved, name) for name in _DEFAULTS}
    return PointMassMission(
        launch_north_m=values["launch.north_m"],
        launch_east_m=values["launch.east_m"],
        launch_altitude_m=values["launch.altitude_m"],
        launch_speed_mps=values["launch.speed_mps"],
        launch_heading_deg=values["launch.heading_deg"],
        launch_flight_path_deg=values["launch.flight_path_deg"],
        waypoint_north_m=values["navigation.waypoint.north.command"],
        waypoint_east_m=values["navigation.waypoint.east.command"],
        waypoint_altitude_m=values["navigation.waypoint.altitude.command"],
        capture_radius_m=values["navigation.waypoint.capture_radius.command"],
        target_north_m=values["navigation.target.position.north.command"],
        target_east_m=values["navigation.target.position.east.command"],
        target_altitude_m=values["navigation.target.position.altitude.command"],
        target_north_velocity_mps=values["navigation.target.velocity.north.command"],
        target_east_velocity_mps=values["navigation.target.velocity.east.command"],
        target_vertical_velocity_mps=values["navigation.target.velocity.vertical.command"],
        target_capture_radius_m=values["navigation.target.capture_radius.command"],
        direct_lateral_acceleration_north_mps2=values["control.lateral_acceleration.local.north.command"],
        direct_lateral_acceleration_east_mps2=values["control.lateral_acceleration.local.east.command"],
        direct_lateral_acceleration_vertical_mps2=values["control.lateral_acceleration.local.vertical.command"],
        objective_kind=(
            "constant_velocity_target"
            if mission_template_id == TARGET_TRACK_MISSION_TEMPLATE_ID
            else "direct_lateral_acceleration"
            if mission_template_id == DIRECT_ACCELERATION_MISSION_TEMPLATE_ID
            else "fixed_waypoint"
        ),
        duration_s=values["runtime.duration_s"],
        time_step_s=values["runtime.time_step_s"],
    )
    ####


def _validate_mission_control_scope(
    resolved: Mapping[str, object],
    *,
    mission_template_id: str,
) -> None:
    """Reject command values that the selected mission grammar cannot consume."""

    active_ids = set(_MISSION_ACTION_IDS[mission_template_id])
    for identifier, owner_mission_template_id in _ACTION_MISSION_TEMPLATE_IDS.items():
        if identifier in active_ids:
            continue
        value = _resolved_float(resolved, identifier)
        default = _DEFAULTS[identifier]
        if value != default:
            raise ConfigurationContractError(
                "inactive-mission-control",
                (
                    f"mission template {mission_template_id!r} does not consume {identifier!r}; "
                    f"select {owner_mission_template_id!r} or reset the inactive value to its default {default!r}"
                ),
                path=f"configuration.root.{identifier}",
            )
    ####


def _resolved_float(values: Mapping[str, object], name: str) -> float:
    value = values.get(name)
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"resolved parameter {name!r} must be numeric")
    return float(value)
    ####


def _resolved_text(values: Mapping[str, object], name: str) -> str:
    value = values.get(name)
    if not isinstance(value, str) or not value:
        raise ValueError(f"resolved parameter {name!r} must be a non-empty string")
    return value
    ####


def _trajectory_result(
    request: MissionCompositionRunRequest,
    model: TrajectoryModelMetadata,
    run: PointMassRun | Pseudo6Run,
    *,
    fidelity: str,
    sensor_suite: InterceptorSensorSuite,
) -> MissionCompositionTrajectoryResult:
    realization_id = PSEUDO6_REALIZATION_ID if fidelity == PSEUDO6_FIDELITY_ID else POINT_MASS_REALIZATION_ID
    selected = resolve_output_selection(
        model.output_schema,
        request.output,
        fidelity=fidelity,
        operation="batch",
        realization_id=realization_id,
        mission_template_id=request.prepared_configuration.configuration.mission_template_id or MISSION_TEMPLATE_ID,
    )
    group_by_channel = {channel_id: group.id for group in model.output_schema.telemetry_groups for channel_id in group.channel_ids}
    channels = tuple(
        TrajectoryChannelMetadata(
            id=item.id,
            channel_class="core_state" if item in model.output_schema.core_channels else "telemetry",
            telemetry_group=group_by_channel.get(item.id),
            quantity=item.quantity,
            unit=item.canonical_unit,
            data_type=item.data_type,
            shape=item.shape,
            frame=item.frame,
            sampling_semantics=item.sampling_semantics,
            interpolation=item.interpolation,
            description=item.description,
        )
        for item in selected
    )
    indexes = _sample_indices(run.samples, request.output.cadence_s, request.output.maximum_samples_per_object)
    selected_ids = tuple(item.id for item in selected)
    samples = tuple(
        TrajectorySample(
            time_s=run.samples[index].time_s,
            values={channel_id: _sample_value(run.samples[index], channel_id) for channel_id in selected_ids},
            segment_instance_id="intercept-01",
            standard_ecef=run.samples[index].standard_ecef,
        )
        for index in indexes
    )
    object_id = f"{request.request_id}:primary"
    terminal_status: Literal["completed", "terminated"] = "terminated" if run.termination == "ground_impact" else "completed"
    events = (
        (
            TrajectoryEvent(
                id=f"{object_id}:{run.termination}",
                time_s=samples[-1].time_s,
                category="impact" if run.termination == "ground_impact" else "termination",
                kind=run.termination,
                object_id=object_id,
                segment_instance_id="intercept-01",
                detail=f"Parametric interceptor batch ended by {run.termination}.",
            ),
        )
        if request.output.include_events
        else ()
    )
    segments = (
        (
            TrajectorySegmentResult(
                id="waypoint-intercept",
                instance_id="intercept-01",
                object_id=object_id,
                start_time_s=samples[0].time_s,
                end_time_s=samples[-1].time_s,
                status=terminal_status,
                event_ids=tuple(item.id for item in events),
            ),
        )
        if request.output.include_segments
        else ()
    )
    diagnostics = (
        MissionCompositionDiagnostic(
            severity="info",
            code="surrogate-assumption-case",
            message="Execution used a resolved parametric surrogate; inspect the profile fingerprint and evidence trace.",
            phase="projection",
            recoverability="degraded",
            provider_id=request.provider_id,
            model_id=model.id,
            object_id=object_id,
            details={
                "profile_id": next(item.profile_id for item in model.fidelities if item.id == fidelity),
                "fidelity": fidelity,
                "environment_model_id": _resolved_text(
                    request.prepared_configuration.resolved,
                    "runtime.environment_model_id",
                ),
                "gravity_model_id": _resolved_text(
                    request.prepared_configuration.resolved,
                    "runtime.gravity_model_id",
                ),
                "sensor_suite_id": sensor_suite.id,
                "sensor_suite_version": sensor_suite.version,
                "sensor_suite_fingerprint": sensor_suite.fingerprint,
                "sensor_provider_kind": sensor_suite.provider_kind_for_fidelity(fidelity),
                "target_track_sensor_provider_kind": sensor_suite.target_track_provider.kind,
                "target_track_sensor_schema_id": "taoryx.tracking.relative-state/v1",
            },
        ),
    )
    return MissionCompositionTrajectoryResult(
        provider_id=request.provider_id,
        provider_version=request.provider_version,
        request_id=request.request_id,
        configuration_fingerprint=request.prepared_configuration.fingerprint,
        primary_model_id=model.id,
        primary_object_id=object_id,
        status=terminal_status,
        objects=(
            TrajectoryObject(
                object_id=object_id,
                model_id=model.id,
                realization_id=realization_id,
                name=model.name,
                role="primary_vehicle",
                fidelity=fidelity,
                status=terminal_status,
                active_from_s=samples[0].time_s,
                active_to_s=samples[-1].time_s,
                terminal_disposition=run.termination,
                channels=channels,
                samples=samples,
                segments=segments,
                provenance=model.provenance,
                claim_boundary=model.claim_boundary,
            ),
        ),
        events=events,
        diagnostics=diagnostics,
        claim_boundary=model.claim_boundary,
    )
    ####


def _sample_value(sample: PointMassSample | Pseudo6Sample, channel_id: str) -> float | int | bool | str:
    values: dict[str, float | int | bool | str] = {
        "phase.id": sample.phase_id,
        "vehicle.operational": sample.operational,
        "model.applicability.declared": sample.applicability_declared,
        "model.applicability.status": sample.applicability_status,
        "model.applicability.reason": sample.applicability_reason,
        "position.local.north": sample.north_m,
        "position.local.east": sample.east_m,
        "position.geometric.altitude": sample.altitude_m,
        "velocity.local.north": sample.north_velocity_mps,
        "velocity.local.east": sample.east_velocity_mps,
        "velocity.local.vertical": sample.vertical_velocity_mps,
        "velocity.speed": sample.speed_mps,
        "mass.total": sample.mass_kg,
        "propulsion.thrust": sample.thrust_n,
        "propulsion.thrust.axial": sample.axial_thrust_n,
        "propulsion.throttle.commanded": sample.throttle_command,
        "propulsion.throttle.achieved": sample.throttle_achieved,
        "propulsion.propellant.remaining": sample.propellant_remaining_kg,
        "propulsion.available": sample.propulsion_available,
        "propulsion.phase": sample.propulsion_phase,
        "propulsion.pulse.index": sample.propulsion_pulse_index,
        "environment.density": sample.density_kg_m3,
        "environment.pressure": sample.pressure_pa,
        "environment.temperature": sample.temperature_k,
        "environment.speed_of_sound": sample.speed_of_sound_mps,
        "environment.wind.ecfc.x": sample.wind_velocity_x_mps,
        "environment.wind.ecfc.y": sample.wind_velocity_y_mps,
        "environment.wind.ecfc.z": sample.wind_velocity_z_mps,
        "aerodynamics.airspeed": sample.airspeed_mps,
        "aerodynamics.mach": sample.mach,
        "aerodynamics.drag_coefficient": sample.drag_coefficient,
        "aerodynamics.maneuver.normal_force_coefficient": sample.maneuver_normal_force_coefficient,
        "aerodynamics.maneuver.drag_factor": sample.maneuver_drag_factor,
        "aerodynamics.maneuver.drag_coefficient": sample.maneuver_drag_coefficient,
        "aerodynamics.drag_coefficient.total": sample.total_drag_coefficient,
        "aerodynamics.dynamic_pressure": sample.dynamic_pressure_pa,
        "aerodynamics.drag.base": sample.base_drag_n,
        "aerodynamics.drag.maneuver": sample.maneuver_drag_n,
        "aerodynamics.drag": sample.drag_n,
        "guidance.waypoint.north.accepted": sample.waypoint_north_accepted_m,
        "guidance.waypoint.east.accepted": sample.waypoint_east_accepted_m,
        "guidance.waypoint.altitude.accepted": sample.waypoint_altitude_accepted_m,
        "guidance.waypoint.capture_radius.accepted": sample.waypoint_capture_radius_accepted_m,
        "guidance.waypoint.range": sample.waypoint_range_m,
        "guidance.objective.kind": sample.guidance_objective_kind,
        "guidance.objective.range": sample.waypoint_range_m,
        "guidance.objective.captured": sample.phase_id in {"waypoint_capture", "target_intercept"},
        "guidance.objective.capture_occurred": sample.phase_id in {"waypoint_capture", "target_intercept"},
        "guidance.target.reference_position.north.accepted": sample.waypoint_north_accepted_m,
        "guidance.target.reference_position.east.accepted": sample.waypoint_east_accepted_m,
        "guidance.target.reference_position.altitude.accepted": sample.waypoint_altitude_accepted_m,
        "guidance.target.velocity.north.accepted": sample.target_north_velocity_mps,
        "guidance.target.velocity.east.accepted": sample.target_east_velocity_mps,
        "guidance.target.velocity.vertical.accepted": sample.target_vertical_velocity_mps,
        "guidance.target.capture_radius.accepted": sample.waypoint_capture_radius_accepted_m,
        "guidance.target.position.north": sample.target_north_m,
        "guidance.target.position.east": sample.target_east_m,
        "guidance.target.position.altitude": sample.target_altitude_m,
        "guidance.relative.velocity.north": sample.relative_north_velocity_mps,
        "guidance.relative.velocity.east": sample.relative_east_velocity_mps,
        "guidance.relative.velocity.vertical": sample.relative_vertical_velocity_mps,
        "guidance.relative.time_to_closest_approach": sample.time_to_closest_approach_s,
        "guidance.relative.predicted_miss_distance": sample.predicted_miss_distance_m,
        "guidance.law.id": sample.guidance_archetype,
        "guidance.law.mode": sample.guidance_mode,
        "guidance.closing_speed": sample.guidance_closing_speed_mps,
        "guidance.line_of_sight_rate": sample.guidance_line_of_sight_rate_rad_s,
        "guidance.navigation_constant": sample.guidance_navigation_constant,
        "control.lateral_acceleration.accepted.local.north": sample.direct_lateral_acceleration_accepted_vector_mps2[0],
        "control.lateral_acceleration.accepted.local.east": sample.direct_lateral_acceleration_accepted_vector_mps2[1],
        "control.lateral_acceleration.accepted.local.vertical": sample.direct_lateral_acceleration_accepted_vector_mps2[2],
        "guidance.lateral_acceleration.commanded": sample.lateral_acceleration_command_mps2,
        "guidance.lateral_acceleration.achieved": sample.lateral_acceleration_achieved_mps2,
        "guidance.lateral_acceleration.commanded.local.north": sample.lateral_acceleration_command_vector_mps2[0],
        "guidance.lateral_acceleration.commanded.local.east": sample.lateral_acceleration_command_vector_mps2[1],
        "guidance.lateral_acceleration.commanded.local.vertical": sample.lateral_acceleration_command_vector_mps2[2],
        "guidance.lateral_acceleration.achieved.local.north": sample.lateral_acceleration_achieved_vector_mps2[0],
        "guidance.lateral_acceleration.achieved.local.east": sample.lateral_acceleration_achieved_vector_mps2[1],
        "guidance.lateral_acceleration.achieved.local.vertical": sample.lateral_acceleration_achieved_vector_mps2[2],
        "guidance.lateral_acceleration.achievement_fraction": sample.lateral_acceleration_achievement_fraction,
        "guidance.lateral_acceleration.direction_error.valid": sample.lateral_acceleration_direction_error_valid,
        "guidance.lateral_acceleration.direction_error": sample.lateral_acceleration_direction_error_rad,
        "guidance.lateral_acceleration.limit": sample.lateral_acceleration_limit_mps2,
        "guidance.lateral_acceleration.utilization": sample.lateral_acceleration_utilization,
        "guidance.available": sample.guidance_available,
        "control.authority.configuration": sample.control_configuration,
        "control.authority.aerodynamic": sample.aerodynamic_lateral_authority_mps2,
        "control.authority.thrust_vector": sample.thrust_vector_lateral_authority_mps2,
        "control.authority.combined_unclipped": sample.combined_lateral_authority_mps2,
        "control.authority.available": sample.lateral_acceleration_available_mps2,
        "control.authority.utilization": sample.lateral_acceleration_authority_utilization,
        "control.authority.structural_limit_active": sample.control_authority_structural_limit_active,
        "control.allocation.policy": sample.control_allocation_policy,
        "control.allocation.aerodynamic_achieved": sample.aerodynamic_lateral_acceleration_achieved_mps2,
        "control.allocation.thrust_vector_achieved": sample.thrust_vector_lateral_acceleration_achieved_mps2,
        "control.allocation.thrust_vector_angle": sample.thrust_vector_angle_achieved_rad,
        "control.limited": sample.control_limited,
        "control.limit.reason": sample.control_limit_reason,
        "sensor.target_track.applicable": sample.target_track_applicable,
        "sensor.target_track.valid": sample.target_track_valid,
        "sensor.target_track.sampled_at": sample.target_track_sampled_at_s,
        "sensor.target_track.available_at": sample.target_track_available_at_s,
        "sensor.target_track.latency": sample.target_track_latency_s,
        "sensor.target_track.delivery_fresh": sample.target_track_delivery_fresh,
        "sensor.target_track.sequence": sample.target_track_sequence,
        "sensor.target_track.schema_id": sample.target_track_schema_id,
        "sensor.target_track.invalid_reason": sample.target_track_invalid_reason,
        "sensor.target_track.target_id": sample.target_track_target_id,
        "sensor.target_track.frame_id": sample.target_track_frame_id,
        "sensor.target_track.range": sample.target_track_range_m,
        "sensor.target_track.azimuth": sample.target_track_azimuth_rad,
        "sensor.target_track.elevation": sample.target_track_elevation_rad,
        "sensor.target_track.closing_speed": sample.target_track_closing_speed_mps,
        "sensor.target_track.relative_position.sensor.x": sample.target_track_relative_position_sensor_m[0],
        "sensor.target_track.relative_position.sensor.y": sample.target_track_relative_position_sensor_m[1],
        "sensor.target_track.relative_position.sensor.z": sample.target_track_relative_position_sensor_m[2],
        "sensor.target_track.relative_velocity.sensor.x": sample.target_track_relative_velocity_sensor_mps[0],
        "sensor.target_track.relative_velocity.sensor.y": sample.target_track_relative_velocity_sensor_mps[1],
        "sensor.target_track.relative_velocity.sensor.z": sample.target_track_relative_velocity_sensor_mps[2],
        "sensor.target_track.line_of_sight_rate.sensor.x": sample.target_track_line_of_sight_rate_sensor_rad_s[0],
        "sensor.target_track.line_of_sight_rate.sensor.y": sample.target_track_line_of_sight_rate_sensor_rad_s[1],
        "sensor.target_track.line_of_sight_rate.sensor.z": sample.target_track_line_of_sight_rate_sensor_rad_s[2],
    }
    if isinstance(sample, PointMassSample):
        values.update(
            {
                "sensor.translation_acceleration.valid": sample.translation_acceleration_valid,
                "sensor.translation_acceleration.interval": sample.translation_acceleration_interval_s,
                "sensor.translation_acceleration.delta_velocity.eci.x": sample.translation_delta_velocity_x_mps,
                "sensor.translation_acceleration.delta_velocity.eci.y": sample.translation_delta_velocity_y_mps,
                "sensor.translation_acceleration.delta_velocity.eci.z": sample.translation_delta_velocity_z_mps,
                "sensor.translation_acceleration.sampled_at": sample.translation_acceleration_sampled_at_s,
                "sensor.translation_acceleration.available_at": sample.translation_acceleration_available_at_s,
                "sensor.translation_acceleration.latency": sample.translation_acceleration_latency_s,
                "sensor.translation_acceleration.schema_id": sample.translation_acceleration_schema_id,
                "sensor.translation_acceleration.delivery_fresh": sample.translation_acceleration_delivery_fresh,
                "sensor.translation_acceleration.sequence": sample.translation_acceleration_sequence,
            }
        )
    else:
        values.update(
            {
                "attitude.roll.commanded": sample.roll_command_rad,
                "control.attitude_response.authority_available": sample.attitude_response_authority_available,
                "control.attitude_response.command_support_fraction": sample.attitude_response_command_support_fraction,
                "control.attitude_response.authority_limited": sample.attitude_response_authority_limited,
                "aerodynamics.flow_angles.valid": sample.flow_angles_valid,
                "aerodynamics.relative_velocity.body.x": sample.air_relative_velocity_body_mps[0],
                "aerodynamics.relative_velocity.body.y": sample.air_relative_velocity_body_mps[1],
                "aerodynamics.relative_velocity.body.z": sample.air_relative_velocity_body_mps[2],
                "aerodynamics.angle_of_attack": sample.angle_of_attack_rad,
                "aerodynamics.sideslip_angle": sample.sideslip_angle_rad,
                "attitude.pitch.commanded": sample.pitch_command_rad,
                "attitude.yaw.commanded": sample.yaw_command_rad,
                "attitude.roll": sample.roll_rad,
                "attitude.pitch": sample.pitch_rad,
                "attitude.yaw": sample.yaw_rad,
                "angular_velocity.body.p": sample.body_rate_p_rad_s,
                "angular_velocity.body.q": sample.body_rate_q_rad_s,
                "angular_velocity.body.r": sample.body_rate_r_rad_s,
                "angular_acceleration.body.p": sample.body_acceleration_p_rad_s2,
                "angular_acceleration.body.q": sample.body_acceleration_q_rad_s2,
                "angular_acceleration.body.r": sample.body_acceleration_r_rad_s2,
                "sensor.imu.valid": sample.imu_valid,
                "sensor.imu.interval": sample.imu_interval_s,
                "sensor.imu.delta_velocity.body.x": sample.imu_delta_velocity_x_mps,
                "sensor.imu.delta_velocity.body.y": sample.imu_delta_velocity_y_mps,
                "sensor.imu.delta_velocity.body.z": sample.imu_delta_velocity_z_mps,
                "sensor.imu.delta_angle.body.x": sample.imu_delta_angle_x_rad,
                "sensor.imu.delta_angle.body.y": sample.imu_delta_angle_y_rad,
                "sensor.imu.delta_angle.body.z": sample.imu_delta_angle_z_rad,
                "sensor.imu.sampled_at": sample.imu_sampled_at_s,
                "sensor.imu.available_at": sample.imu_available_at_s,
                "sensor.imu.latency": sample.imu_latency_s,
                "sensor.imu.schema_id": sample.imu_schema_id,
                "sensor.imu.delivery_fresh": sample.imu_delivery_fresh,
                "sensor.imu.sequence": sample.imu_sequence,
            }
        )
    return values[channel_id]
    ####


def _sample_indices(
    samples: Sequence[PointMassSample | Pseudo6Sample],
    cadence_s: float | None,
    maximum_samples: int | None,
) -> tuple[int, ...]:
    if cadence_s is None:
        indexes = list(range(len(samples)))
    else:
        indexes = [0]
        next_time = samples[0].time_s + cadence_s
        for index, sample in enumerate(samples[1:-1], start=1):
            if sample.time_s + 1.0e-12 >= next_time:
                indexes.append(index)
                next_time = sample.time_s + cadence_s
        if len(samples) > 1:
            indexes.append(len(samples) - 1)
    if maximum_samples is not None and len(indexes) > maximum_samples:
        if maximum_samples == 1:
            indexes = [0]
        else:
            indexes = [indexes[round(i * (len(indexes) - 1) / (maximum_samples - 1))] for i in range(maximum_samples)]
    return tuple(dict.fromkeys(indexes))
    ####


def _parameter_unit(identifier: str) -> str | None:
    if identifier.endswith("_id"):
        return None
    if "acceleration." in identifier:
        return "m/s^2"
    if ".velocity." in identifier:
        return "m/s"
    if identifier.endswith("_deg"):
        return "deg"
    if identifier.endswith("_mps"):
        return "m/s"
    if identifier.endswith("_s"):
        return "s"
    return "m"
    ####


def _quantity(unit: str | None) -> str | None:
    return {
        "1": "dimensionless",
        "m": "length",
        "m^2": "area",
        "m/s": "speed",
        "m/s^2": "acceleration",
        "s": "time",
        "deg": "angle",
        "rad": "angle",
        "rad/s": "angular_rate",
        "rad/s^2": "angular_acceleration",
        "kg": "mass",
        "kg/s": "mass_flow_rate",
        "N": "force",
        "N*s": "impulse",
    }.get(unit or "")
    ####


def _execution_error(code: str, message: str, model_id: str) -> MissionCompositionExecutionError:
    return MissionCompositionExecutionError(
        MissionCompositionDiagnostic(
            severity="error",
            code=code,
            message=message,
            phase="preflight",
            recoverability="correctable",
            provider_id=PROVIDER_ID,
            model_id=model_id,
        ),
        category="invalid_request",
    )
    ####


__all__ = [
    "DIRECT_ACCELERATION_MISSION_TEMPLATE_ID",
    "FIDELITY_ID",
    "MISSION_CONTROL_SCOPE_CONTRACT",
    "MISSION_TEMPLATE_ID",
    "PACKAGE_VERSION",
    "PROVIDER_ID",
    "REALIZATION_ID",
    "TARGET_TRACK_MISSION_TEMPLATE_ID",
    "ParametricInterceptorMissionCompositionProvider",
]
####
