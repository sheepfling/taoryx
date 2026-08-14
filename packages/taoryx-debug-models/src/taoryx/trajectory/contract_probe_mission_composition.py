"""Synthetic provider that exercises the complete Mission Composition surface.

The contract probe is intentionally not a physical vehicle.  It gives plug-in
hosts, generic editors, transports, and result consumers one deterministic
advertisement that contains every configuration-node family, value type,
presentation hint, fidelity transition, operation state, deployment state,
reference-frame shape, output interpolation kind, multi-object lineage path,
and semantic control/agent-action mode in the public contract.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, Literal

from ..fixture_composition_episode import FixtureCompositionEpisode, FixtureTransition
from .configuration_contract import (
    ConfigurationBound,
    ConfigurationChoiceSchema,
    ConfigurationChoiceValue,
    ConfigurationChoiceVariant,
    ConfigurationContractError,
    ConfigurationGroupSchema,
    ConfigurationGroupValue,
    ConfigurationInterval,
    ConfigurationNodeValue,
    ConfigurationOptionalSchema,
    ConfigurationOptionalValue,
    ConfigurationParameterSchema,
    ConfigurationParameterValue,
    ConfigurationPeriodicity,
    ConfigurationSequenceSchema,
    ConfigurationSequenceTemplate,
    ConfigurationSequenceValue,
    ConfigurationValueSpace,
    ControlCommandSemantics,
    ControlQuantizationMetadata,
    NumericPresentationMetadata,
    PreparedTrajectoryConfiguration,
    PresentationLinkMetadata,
    TrajectoryConfigurationInstance,
    TrajectoryConfigurationSchema,
    TrajectoryControlAdvertisement,
    TrajectoryControlAuthorityMetadata,
    TrajectoryControlAvailability,
    TrajectoryControlChannelMetadata,
    TrajectoryControlIntentMetadata,
    TrajectoryControlNativeBindingMetadata,
    TrajectoryDeploymentMetadata,
    TrajectoryEntityOutputMetadata,
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
    validate_configuration_instance,
)
from .execution_contract import (
    MissionCompositionDiagnostic,
    MissionCompositionExecutionError,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
    MissionCompositionTrajectoryResult,
    TrajectoryChannelMetadata,
    TrajectoryEntityRelationship,
    TrajectoryEvent,
    TrajectoryObject,
    TrajectorySample,
    TrajectorySegmentResult,
    TrajectoryStateSnapshot,
    resolve_output_selection,
)
from .session_interface import build_session_interface_contract

if TYPE_CHECKING:
    from taoryx.plugins.discovery import PluginCatalog

CONTRACT_PROBE_PROVIDER_ID = "taoryx.debug.mission-composition-contract-probe"
CONTRACT_PROBE_MODEL_ID = "contract_probe_vehicle"
CONTRACT_PROBE_MODEL_VERSION = "1.0.0"
_DEFAULT_PROVIDER_VERSION = "0.1.0a0"
_FIDELITIES: tuple[Literal["coarse", "medium", "high"], ...] = ("coarse", "medium", "high")


class ContractProbeMissionCompositionProvider:
    """One-model provider for end-to-end contract and UI development."""

    def __init__(
        self,
        *,
        provider_version: str = _DEFAULT_PROVIDER_VERSION,
        plugin_catalog: PluginCatalog | None = None,
    ) -> None:
        if not provider_version.strip():
            raise ValueError("contract-probe provider version must not be empty")
        self._schema = contract_probe_configuration_schema()
        self._model = contract_probe_model_metadata(self._schema)
        self._plugin_catalog = plugin_catalog
        self._metadata = TrajectoryProviderMetadata(
            id=CONTRACT_PROBE_PROVIDER_ID,
            name="TAORYX Mission Composition Contract Probe",
            version=provider_version,
            description="Synthetic, deterministic provider that exercises every public advertisement and result surface.",
            presentation=TrajectoryProviderPresentationMetadata(
                display_name="Mission Composition Contract Probe",
                short_name="Contract Probe",
                summary="Development-only vehicle for generic editors, plug-in hosts, transports, and result viewers.",
                organization="TAORYX",
                categories=("Development", "Contract Conformance"),
                links=(
                    PresentationLinkMetadata(
                        relation="documentation",
                        label="Mission Composition front door",
                        uri="docs/MISSION_COMPOSITION.md",
                        media_type="text/markdown",
                    ),
                    PresentationLinkMetadata(
                        relation="source",
                        label="Contract probe implementation",
                        uri="src/taoryx/trajectory/contract_probe_mission_composition.py",
                        media_type="text/x-python",
                    ),
                    PresentationLinkMetadata(
                        relation="support",
                        label="Structured failure contract",
                        uri="docs/architecture/mission-composition-provider-api.md",
                        media_type="text/markdown",
                    ),
                ),
            ),
            status="runnable_debug_contract",
            tags=("mission-composition", "debug", "contract-probe", "synthetic"),
            execution_contract="taoryx.contract-probe-execution/v1",
            model_count=1,
            provenance="synthetic successor-side interface witness",
            claim_boundary="Development contract witness only; it contains no physical-vehicle or qualification claims.",
        )
        ####

    @property
    def metadata(self) -> TrajectoryProviderMetadata:
        return self._metadata
        ####

    @property
    def plugin_catalog(self) -> PluginCatalog | None:
        """Expose the selected plug-in scope retained by the focused provider."""

        return self._plugin_catalog
        ####

    def list_models(self) -> tuple[TrajectoryModelMetadata, ...]:
        return (self._model,)
        ####

    def get_model_schema(self, model_id: str) -> TrajectoryConfigurationSchema:
        if model_id != CONTRACT_PROBE_MODEL_ID:
            raise KeyError(f"unknown contract-probe model {model_id!r}")
        return self._schema
        ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        """Return the probe's exhaustive output-capability schema."""

        if model_id != CONTRACT_PROBE_MODEL_ID:
            raise KeyError(f"unknown contract-probe model {model_id!r}")
        return self._model.output_schema
        ####

    def validate_configuration(
        self,
        configuration: TrajectoryConfigurationInstance,
    ) -> PreparedTrajectoryConfiguration:
        prepared = validate_configuration_instance(self._schema, configuration)
        self._validate_advertised_selection(configuration, operation="validate")
        return prepared
        ####

    def open_session_episode(
        self,
        prepared: PreparedTrajectoryConfiguration,
        *,
        seed: int | None = None,
        integration_step_s: float = 0.02,
    ) -> FixtureCompositionEpisode:
        """Open the development-only interactive contract stressor."""

        del integration_step_s
        self._validate_advertised_selection(prepared.configuration, operation="step")
        return _open_contract_probe_session_episode(self._model, prepared, seed=seed)
        ####

    def _validate_advertised_selection(
        self,
        configuration: TrajectoryConfigurationInstance,
        *,
        operation: Literal["validate", "batch", "step"],
    ) -> None:
        """Reject configurations outside this provider's exact public matrix."""

        realization = next((item for item in self._model.realizations if item.id == configuration.realization_id), None)
        if realization is None:
            raise ConfigurationContractError(
                "unknown-realization",
                "contract-probe execution requires one advertised realization_id",
                path="configuration.realization_id",
            )
        if realization.status != "available":
            raise ConfigurationContractError(
                "realization-unavailable",
                f"realization {realization.id!r} is {realization.status!r}: {list(realization.blockers)!r}",
                path="configuration.realization_id",
            )
        if configuration.fidelity not in realization.fidelity_aliases:
            raise ConfigurationContractError(
                "realization-fidelity-incompatible",
                f"realization {realization.id!r} does not support fidelity {configuration.fidelity!r}",
                path="configuration.realization_id",
            )
        mission = next((item for item in self._model.mission_templates if item.id == configuration.mission_template_id), None)
        if mission is None:
            raise ConfigurationContractError(
                "unknown-mission-template",
                "contract-probe execution requires one advertised mission_template_id",
                path="configuration.mission_template_id",
            )
        if configuration.fidelity not in mission.compatible_fidelities:
            raise ConfigurationContractError(
                "mission-fidelity-incompatible",
                f"mission {mission.id!r} does not support fidelity {configuration.fidelity!r}",
                path="configuration.mission_template_id",
            )
        if mission.id not in realization.mission_template_ids:
            raise ConfigurationContractError(
                "realization-mission-incompatible",
                f"realization {realization.id!r} does not support mission {mission.id!r}",
                path="configuration.realization_id",
            )
        if not isinstance(configuration.root, ConfigurationGroupValue):
            raise ConfigurationContractError(
                "invalid-configuration-root",
                "contract-probe configuration root must be a group",
                path="configuration.root",
            )
        segments = configuration.root.values.get("segments")
        if not isinstance(segments, ConfigurationSequenceValue):
            raise ConfigurationContractError(
                "missing-mission-sequence",
                "contract-probe configuration requires a mission segment sequence",
                path="configuration.root.segments",
            )
        selected_segments = tuple(item.selected for item in segments.items if isinstance(item, ConfigurationChoiceValue))
        if selected_segments != mission.segment_sequence:
            raise ConfigurationContractError(
                "mission-sequence-mismatch",
                f"mission {mission.id!r} requires segment sequence {list(mission.segment_sequence)!r}",
                path="configuration.root.segments",
            )
        exact = next(
            (
                item
                for item in mission.operations
                if item.fidelity == configuration.fidelity and item.realization_id == realization.id and item.operation == operation
            ),
            None,
        )
        if exact is None or exact.status != "available":
            raise ConfigurationContractError(
                "operation-unavailable",
                f"mission {mission.id!r} has no available {operation!r} operation for {configuration.fidelity!r}/{realization.id!r}",
                path="configuration.mission_template_id",
            )
        ####

    def build_runner(self) -> MissionCompositionRunnerRegistry:
        return MissionCompositionRunnerRegistry({(self.metadata.id, CONTRACT_PROBE_MODEL_ID): self._execute})
        ####

    def _execute(self, request: MissionCompositionRunRequest) -> MissionCompositionTrajectoryResult:
        if request.provider_id != self.metadata.id or request.provider_version != self.metadata.version:
            raise _probe_error(
                "provider-identity-mismatch",
                "Run request does not match the contract-probe provider identity.",
                request,
                path="/provider_id",
            )
        if request.model_id != CONTRACT_PROBE_MODEL_ID:
            raise _probe_error(
                "model-identity-mismatch",
                "Run request does not select the contract-probe vehicle.",
                request,
                path="/prepared_configuration/configuration/model_id",
            )
        if request.operation != "batch":
            raise _probe_error(
                "stateful-session-required",
                "Interactive execution is not advertised until the debug probe implements the common stateful session lifecycle.",
                request,
                path="/operation",
            )
        try:
            self._validate_advertised_selection(
                request.prepared_configuration.configuration,
                operation="batch",
            )
        except ConfigurationContractError as error:
            raise _probe_error(
                "operation-not-advertised",
                str(error),
                request,
                path="/prepared_configuration/configuration",
            ) from error
        try:
            selected_channels = resolve_output_selection(
                self._model.output_schema,
                request.output,
                fidelity=request.prepared_configuration.configuration.fidelity,
                operation=request.operation,
            )
        except ValueError as error:
            raise _probe_error(
                "invalid-output-selection",
                str(error),
                request,
                path="/output",
            ) from error
        if request.output.maximum_samples_per_object is not None and request.output.maximum_samples_per_object < 3:
            raise _probe_error(
                "sample-limit-not-supported",
                "The contract probe requires three samples per returned object to exercise interpolation metadata.",
                request,
                path="/output/maximum_samples_per_object",
            )
        include_child = request.output.include_spawned_objects and (request.output.maximum_objects is None or request.output.maximum_objects >= 2)
        include_descendant = include_child and (request.output.maximum_objects is None or request.output.maximum_objects >= 3)
        return _probe_result(
            request,
            selected_channels,
            include_child=include_child,
            include_descendant=include_descendant,
        )
        ####

    ####


def contract_probe_configuration_schema() -> TrajectoryConfigurationSchema:
    """Build a grammar containing every public configuration node and value type."""

    initialization = ConfigurationGroupSchema(
        id="initialization",
        label="Initial State",
        description="All scalar and vector parameter representations in one group.",
        children=(
            _number_parameter(
                "launch_altitude_m",
                "Launch Altitude",
                "Positive scalar with hard, qualified, and safe-extended intervals.",
                quantity="length",
                unit="m",
                default=1000.0,
                interval=_interval(0.0, 100_000.0),
                qualified_interval=_interval(100.0, 20_000.0),
                safe_extended_interval=_interval(0.0, 50_000.0),
                transform="log",
                control="slider",
                group="initial state",
                order=10,
            ),
            ConfigurationParameterSchema(
                id="integration_order",
                label="Integration Order",
                description="Integer-valued configuration leaf.",
                value_type="integer",
                default=4,
                default_declared=True,
                interval=_interval(1.0, 8.0),
                role="initialization",
                presentation=ValuePresentationMetadata(group="numerics", order=20, control="number_input"),
                provenance="contract-probe grammar",
            ),
            ConfigurationParameterSchema(
                id="emit_diagnostics",
                label="Emit Diagnostics",
                description="Boolean configuration leaf.",
                value_type="boolean",
                default=True,
                default_declared=True,
                role="initialization",
                presentation=ValuePresentationMetadata(group="debug", order=30, visibility="debug", control="toggle"),
                provenance="contract-probe grammar",
            ),
            ConfigurationParameterSchema(
                id="callsign",
                label="Callsign",
                description="Free-form string configuration leaf.",
                value_type="string",
                default="PROBE-01",
                default_declared=True,
                role="initialization",
                presentation=ValuePresentationMetadata(group="identity", order=40, control="text", placeholder="PROBE-01"),
                provenance="contract-probe grammar",
            ),
            ConfigurationParameterSchema(
                id="coordinate_mode",
                label="Coordinate Mode",
                description="Enumerated configuration leaf with human-readable labels.",
                value_type="enum",
                choices=("local_ned", "earth_fixed"),
                default="local_ned",
                default_declared=True,
                role="initialization",
                transform="categorical",
                presentation=ValuePresentationMetadata(
                    group="coordinates",
                    order=50,
                    control="select",
                    enum_labels={"local_ned": "Local NED", "earth_fixed": "Earth Fixed"},
                ),
                provenance="contract-probe grammar",
            ),
            ConfigurationParameterSchema(
                id="initial_position_m",
                label="Initial Position",
                description="Three-component vector configuration leaf.",
                value_type="vector3",
                quantity="length",
                canonical_unit="m",
                display_unit="m",
                default=(0.0, 0.0, -1000.0),
                default_declared=True,
                role="initialization",
                frame="local_ned",
                value_space=_vector_space(3),
                presentation=ValuePresentationMetadata(group="coordinates", order=60, control="coordinate_picker"),
                provenance="contract-probe grammar",
            ),
            ConfigurationParameterSchema(
                id="initial_attitude_quaternion",
                label="Initial Attitude Quaternion",
                description="Four-component normalized quaternion configuration leaf.",
                value_type="vector4",
                default=(1.0, 0.0, 0.0, 0.0),
                default_declared=True,
                role="initialization",
                frame="body",
                value_space=ConfigurationValueSpace(
                    topology="unit_quaternion",
                    representation="quaternion_wxyz",
                    error_rule="shortest_rotation_angle",
                    interpolation_rule="slerp",
                    normalization_rule="unit_norm_and_canonical_sign",
                    equivalence="q and -q represent the same orientation",
                ),
                presentation=ValuePresentationMetadata(group="coordinates", order=70, control="vector_editor", visibility="advanced"),
                provenance="contract-probe grammar",
            ),
            _number_parameter(
                "heading_deg",
                "Heading",
                "Periodic scalar with a circular editor contract.",
                quantity="angle",
                unit="deg",
                default=45.0,
                interval=_interval(0.0, 360.0, maximum_inclusive=False),
                periodicity=ConfigurationPeriodicity(period=360.0, canonical_minimum=0.0),
                control="slider",
                group="coordinates",
                order=80,
            ),
            _number_parameter(
                "success_probability",
                "Success Probability",
                "Bounded scalar exercising logit transform metadata.",
                quantity="probability",
                unit=None,
                default=0.95,
                interval=_interval(0.0, 1.0),
                transform="logit",
                control="slider",
                group="debug",
                order=90,
            ),
        ),
    )
    motion = ConfigurationChoiceSchema(
        id="motion_mode",
        label="Motion Mode",
        description="Mutually exclusive configuration shapes represented structurally.",
        variants=(
            ConfigurationChoiceVariant(
                id="constant_speed",
                label="Constant Speed",
                description="One speed parameter.",
                compatible_fidelities=_FIDELITIES,
                node=ConfigurationGroupSchema(
                    id="constant_speed_parameters",
                    label="Constant Speed Parameters",
                    children=(
                        _number_parameter(
                            "speed_m_s",
                            "Speed",
                            "Commanded constant speed.",
                            quantity="speed",
                            unit="m/s",
                            default=100.0,
                            interval=_interval(0.0, 1000.0),
                            control="slider",
                            group="motion",
                            order=10,
                            role="variant",
                        ),
                    ),
                ),
            ),
            ConfigurationChoiceVariant(
                id="constant_acceleration",
                label="Constant Acceleration",
                description="Initial speed and signed acceleration parameters.",
                compatible_fidelities=("medium", "high"),
                node=ConfigurationGroupSchema(
                    id="constant_acceleration_parameters",
                    label="Constant Acceleration Parameters",
                    children=(
                        _number_parameter(
                            "initial_speed_m_s",
                            "Initial Speed",
                            "Speed at mode entry.",
                            quantity="speed",
                            unit="m/s",
                            default=100.0,
                            interval=_interval(0.0, 1000.0),
                            control="number_input",
                            group="motion",
                            order=10,
                            role="variant",
                        ),
                        _number_parameter(
                            "acceleration_m_s2",
                            "Acceleration",
                            "Signed constant acceleration.",
                            quantity="acceleration",
                            unit="m/s^2",
                            default=2.0,
                            interval=_interval(-50.0, 50.0),
                            control="number_input",
                            group="motion",
                            order=20,
                            role="variant",
                        ),
                    ),
                ),
            ),
        ),
    )
    segment_choice = ConfigurationChoiceSchema(
        id="segment",
        label="Segment Type",
        description="Choice item reused by the variable-length sequence.",
        variants=(
            _segment_variant("hold", "Hold", load_factor=False),
            _segment_variant("maneuver", "Maneuver", load_factor=True),
            _segment_variant("deploy", "Deploy Child", load_factor=False),
        ),
    )
    segments = ConfigurationSequenceSchema(
        id="segments",
        label="Mission Segments",
        description="Open sequence with reviewed templates and custom authoring enabled.",
        item=segment_choice,
        minimum_items=1,
        maximum_items=8,
        templates=(
            ConfigurationSequenceTemplate(
                id="contract_walkthrough",
                label="Contract Walkthrough",
                description="Exercises two segment variants without a deployment.",
                item_variants=("hold", "maneuver"),
                compatible_fidelities=("coarse", "medium"),
            ),
            ConfigurationSequenceTemplate(
                id="deployment_walkthrough",
                label="Deployment Walkthrough",
                description="Exercises parent-child lineage through a deployment segment.",
                item_variants=("hold", "deploy"),
                compatible_fidelities=("coarse", "medium"),
            ),
        ),
        allow_custom=True,
    )
    payload = ConfigurationOptionalSchema(
        id="payload",
        label="Optional Payload",
        description="Explicitly enabled or disabled subtree.",
        item=ConfigurationGroupSchema(
            id="payload_parameters",
            label="Payload Parameters",
            children=(
                _number_parameter(
                    "mass_kg",
                    "Payload Mass",
                    "Optional payload mass.",
                    quantity="mass",
                    unit="kg",
                    default=25.0,
                    interval=_interval(0.0, 500.0),
                    control="number_input",
                    group="payload",
                    order=10,
                    role="variant",
                ),
            ),
        ),
    )
    return TrajectoryConfigurationSchema(
        model_id=CONTRACT_PROBE_MODEL_ID,
        model_version=CONTRACT_PROBE_MODEL_VERSION,
        supported_fidelities=_FIDELITIES,
        root=ConfigurationGroupSchema(
            id="mission",
            label="Contract Probe Mission",
            description="Complete portable configuration-language witness.",
            children=(initialization, motion, segments, payload),
        ),
        claim_boundary="Validates contract shape and metadata semantics only; no physical feasibility claim is possible.",
    )
    ####


def contract_probe_model_metadata(schema: TrajectoryConfigurationSchema | None = None) -> TrajectoryModelMetadata:
    """Return the complete synthetic model advertisement."""

    resolved_schema = schema or contract_probe_configuration_schema()
    fidelities = _probe_fidelities()
    channels = _probe_output_channels()
    output_schema = _probe_output_schema(channels)
    return TrajectoryModelMetadata(
        id=CONTRACT_PROBE_MODEL_ID,
        name="Mission Composition Contract Probe Vehicle",
        version=CONTRACT_PROBE_MODEL_VERSION,
        description="Synthetic debug vehicle covering the full provider advertisement and execution contract.",
        presentation=TrajectoryModelPresentationMetadata(
            display_name="Mission Composition Contract Probe Vehicle",
            short_name="Contract Probe",
            summary="Use this non-physical vehicle to build and inspect a generic Mission Composition integration.",
            category="Development Vehicles",
            subcategory="Contract Probes",
            sort_key="debug:contract-probe",
            badges=("Debug Only", "Full Surface", "Multi-Object"),
            default_fidelity_id="medium",
            default_mission_template_id="deployment_walkthrough",
            default_output_channel_ids=tuple(item.id for item in channels),
            properties=_probe_properties(),
        ),
        family_id="debug.contract-probe",
        physical_family="synthetic_debug",
        model_kind="contract_probe",
        status="runnable_debug_contract",
        tags=("debug", "synthetic", "contract-probe", "do-not-use-for-analysis"),
        operations=("discover", "validate", "batch", "step"),
        common_runner_operations=("batch", "step"),
        capabilities=TrajectoryModelCapabilities(
            initialization_modes=("geodetic", "local_cartesian"),
            segment_types=("hold", "maneuver", "deploy"),
            termination_modes=("duration", "event", "manual_close"),
            operations=("discover", "validate", "batch", "step"),
            supports_custom_segments=True,
            supports_deployment=True,
            supports_dynamic_child_generation=True,
            supports_submodels=True,
        ),
        realizations=tuple(
            TrajectoryRealizationMetadata(
                id=item.id,
                label=item.label,
                description=f"Synthetic {item.label.casefold()} realization.",
                status="available" if "batch" in item.operations or "step" in item.operations else "blocked",
                dynamics_fidelities=(item.dynamics_fidelity,),
                input_realization=item.input_realization,
                actuator_types=item.actuator_types,
                controls=_probe_control_advertisement(item),
                fidelity_aliases=(item.id,),
                mission_template_ids=("deployment_walkthrough",),
                operations=item.operations,
                native_factory_ids=("contract_probe.v1",) if "batch" in item.operations or "step" in item.operations else (),
                blockers=item.blockers,
                source_refs=("src/taoryx/trajectory/contract_probe_mission_composition.py",),
                claim_boundary="Synthetic contract fixture only.",
            )
            for item in fidelities
        ),
        mission_templates=_probe_mission_templates(),
        deployments=_probe_deployments(),
        reference_frames=_probe_reference_frames(),
        output_schema=output_schema,
        output_schema_id=output_schema.schema_id,
        output_schema_fingerprint=output_schema.fingerprint,
        configuration_schema_id=resolved_schema.schema_id,
        configuration_schema_fingerprint=resolved_schema.fingerprint,
        fidelities=fidelities,
        fidelity_transitions=_probe_fidelity_transitions(),
        source_refs=(
            "src/taoryx/trajectory/configuration_contract.py",
            "src/taoryx/trajectory/execution_contract.py",
            "docs/architecture/mission-composition-provider-api.md",
        ),
        provenance="synthetic successor-side full-contract witness",
        claim_boundary="Must never be presented as a physical vehicle, qualified model, or mission-analysis result.",
    )
    ####


def _open_contract_probe_session_episode(
    model: TrajectoryModelMetadata,
    prepared: PreparedTrajectoryConfiguration,
    *,
    seed: int | None,
) -> FixtureCompositionEpisode:
    """Create the synthetic multi-type action and output stress episode."""

    resolved = prepared.resolved
    if not isinstance(resolved, Mapping):
        raise ValueError("contract-probe prepared root must be an object")
    initialization = resolved.get("initialization")
    motion = resolved.get("motion_mode")
    if not isinstance(initialization, Mapping) or not isinstance(motion, Mapping):
        raise ValueError("contract-probe prepared initialization and motion must be objects")
    selected_motion = motion.get("selected")
    motion_values = motion.get("value")
    if not isinstance(selected_motion, str) or not isinstance(motion_values, Mapping):
        raise ValueError("contract-probe prepared motion choice is malformed")
    position = initialization.get("initial_position_m", (0.0, 0.0, -1000.0))
    quaternion = initialization.get("initial_attitude_quaternion", (1.0, 0.0, 0.0, 0.0))
    if not isinstance(position, (list, tuple)) or len(position) != 3:
        raise ValueError("contract-probe initial_position_m must contain three values")
    if not isinstance(quaternion, (list, tuple)) or len(quaternion) != 4:
        raise ValueError("contract-probe quaternion must contain four values")
    initial_speed = float(
        motion_values.get("speed_m_s", motion_values.get("initial_speed_m_s", 0.0))
    )
    acceleration = float(motion_values.get("acceleration_m_s2", 0.0))
    realization_id = prepared.configuration.realization_id or prepared.configuration.fidelity
    contract, observation_schema = build_session_interface_contract(
        model,
        realization_id=realization_id,
        fidelity=prepared.configuration.fidelity,
        family_id="debug.contract-probe",
        physical_family="synthetic_debug",
        claim_boundary=(
            "Synthetic transport, renderer, switching, and action-schema stress fixture only; "
            "it must not be used for physical analysis."
        ),
    )

    def initial_state_factory(_: int | None) -> Mapping[str, object]:
        return {
            "north_m": float(position[0]),
            "east_m": float(position[1]),
            "altitude_m": max(0.0, -float(position[2])),
            "speed_m_s": initial_speed,
            "base_acceleration_m_s2": acceleration,
            "heading_deg": float(initialization.get("heading_deg", 0.0)) % 360.0,
            "quaternion": [float(item) for item in quaternion],
            "held_guidance": {},
            "flap_detent_deg": 0.0,
            "spoiler_position": 0.0,
            "autopilot_mode": "manual",
            "deadman": False,
            "event_marker": 0,
            "fired_events": [],
            "active_profile_id": contract.default_authority_profile_id,
        }
        ####

    return FixtureCompositionEpisode(
        interface_contract=contract,
        observation_schema=observation_schema,
        initial_state_factory=initial_state_factory,
        observation_factory=_contract_probe_session_observation,
        transition=_contract_probe_session_transition,
        authority_selection_hook=_contract_probe_authority_handoff,
        claim_boundary=contract.claim_boundary,
        seed=seed,
    )
    ####


def _contract_probe_session_transition(
    state: Mapping[str, Any],
    profile_id: str,
    action: Mapping[str, Any],
    duration_s: float,
    _: float,
) -> FixtureTransition:
    next_state = dict(state)
    events: list[str] = []
    diagnostics: list[str] = []
    applied_semantic: dict[str, object] = dict(action)
    native: dict[str, object] = {}
    heading_deg = float(next_state["heading_deg"])
    speed_m_s = float(next_state["speed_m_s"])
    acceleration = float(next_state["base_acceleration_m_s2"])

    if profile_id == "debug_guidance_control":
        held = dict(next_state.get("held_guidance", {}))
        held.update(action)
        if "attitude.quaternion.command" in held:
            next_state["quaternion"] = [float(item) for item in held["attitude.quaternion.command"]]
        if "guidance.heading.command" in action:
            heading_deg = float(action["guidance.heading.command"]) % 360.0
        elif "guidance.heading_rate.command" in action:
            heading_deg = (
                heading_deg + float(action["guidance.heading_rate.command"]) * duration_s
            ) % 360.0
        acceleration += float(action.get("guidance.acceleration.increment", 0.0))
        next_state["held_guidance"] = held
        native = {
            "synthetic.heading_deg": heading_deg,
            "synthetic.acceleration_m_s2": acceleration,
            "synthetic.quaternion": next_state["quaternion"],
        }
        lowering = ("debug_guidance_hold", "synthetic_kinematic_transition")
    elif profile_id == "debug_discrete_control":
        if "aerodynamics.flap.detent" in action:
            next_state["flap_detent_deg"] = float(action["aerodynamics.flap.detent"])
        if "aerodynamics.spoiler.increment" in action:
            next_state["spoiler_position"] = min(
                1.0,
                max(-1.0, float(next_state["spoiler_position"]) + float(action["aerodynamics.spoiler.increment"])),
            )
        if "autopilot.mode.select" in action:
            next_state["autopilot_mode"] = str(action["autopilot.mode.select"])
        next_state["deadman"] = bool(action.get("safety.deadman", False))
        native = {
            "synthetic.flap_detent_deg": next_state["flap_detent_deg"],
            "synthetic.spoiler_position": next_state["spoiler_position"],
            "synthetic.autopilot_mode": next_state["autopilot_mode"],
            "synthetic.deadman": next_state["deadman"],
        }
        lowering = ("debug_discrete_hold", "synthetic_mode_transition")
    elif profile_id == "debug_event_control":
        fired = set(str(item) for item in next_state.get("fired_events", []))
        for channel_id, expected, event_name in (
            ("payload.arm.command", "arm", "payload_armed"),
            ("payload.release.command", "release", "payload_released"),
        ):
            if action.get(channel_id) != expected:
                continue
            if channel_id in fired:
                diagnostics.append(f"{channel_id} repeat suppressed by once-until-reset policy")
                applied_semantic.pop(channel_id, None)
                continue
            fired.add(channel_id)
            events.append(event_name)
            next_state["event_marker"] = int(next_state["event_marker"]) + 1
        next_state["fired_events"] = sorted(fired)
        native = {"synthetic.events": list(events)}
        lowering = ("debug_event_gate", "synthetic_event_record")
    else:
        raise ValueError(f"unsupported contract-probe session authority {profile_id!r}")

    speed_m_s = max(0.0, speed_m_s + acceleration * duration_s)
    heading_rad = math.radians(heading_deg)
    next_state["north_m"] = float(next_state["north_m"]) + speed_m_s * math.cos(heading_rad) * duration_s
    next_state["east_m"] = float(next_state["east_m"]) + speed_m_s * math.sin(heading_rad) * duration_s
    next_state["speed_m_s"] = speed_m_s
    next_state["heading_deg"] = heading_deg
    next_state["active_profile_id"] = profile_id
    return FixtureTransition(
        state=next_state,
        applied_action=native,
        applied_semantic_action=applied_semantic,
        lowering_evidence={
            "authority_profile_id": profile_id,
            "lowering_chain": list(lowering),
            "native_action": native,
            "synthetic_only": True,
        },
        events=tuple(events),
        diagnostics=tuple(diagnostics),
        status="active",
    )
    ####


def _contract_probe_authority_handoff(
    state: Mapping[str, Any],
    _: str,
    selected: str,
) -> Mapping[str, object]:
    next_state = dict(state)
    next_state["active_profile_id"] = selected
    if selected == "debug_guidance_control":
        next_state["held_guidance"] = {
            "guidance.heading.command": float(state["heading_deg"]),
            "guidance.heading_rate.command": 0.0,
            "guidance.acceleration.increment": 0.0,
            "attitude.quaternion.command": list(state["quaternion"]),
        }
    return next_state
    ####


def _contract_probe_session_observation(
    state: Mapping[str, Any],
    _: float,
    __: str,
) -> Mapping[str, object]:
    mode = str(state["autopilot_mode"])
    return {
        "position.north_m": float(state["north_m"]),
        "velocity.speed_m_s": float(state["speed_m_s"]),
        "attitude.heading_deg": float(state["heading_deg"]) % 360.0,
        "mode.index": {"manual": 0, "hold": 1, "track": 2}.get(mode, -1),
        "attitude.quaternion": list(state["quaternion"]),
        "health.valid": True,
        "status.mode": mode,
        "diagnostics.payload": {
            "synthetic": True,
            "active_profile_id": state.get("active_profile_id"),
            "flap_detent_deg": state.get("flap_detent_deg"),
            "spoiler_position": state.get("spoiler_position"),
            "deadman": state.get("deadman"),
            "fired_events": list(state.get("fired_events", [])),
        },
        "event.marker": int(state["event_marker"]),
    }
    ####


def build_contract_probe_configuration(
    provider: ContractProbeMissionCompositionProvider,
    *,
    fidelity: Literal["coarse", "medium", "high"] = "medium",
) -> TrajectoryConfigurationInstance:
    """Build a valid, nontrivial configuration for the contract probe."""

    schema = provider.get_model_schema(CONTRACT_PROBE_MODEL_ID)
    initialization = ConfigurationGroupValue(
        values={
            "launch_altitude_m": ConfigurationParameterValue(value=1000.0, unit="m"),
            "integration_order": ConfigurationParameterValue(value=4),
            "emit_diagnostics": ConfigurationParameterValue(value=True),
            "callsign": ConfigurationParameterValue(value="PROBE-01"),
            "coordinate_mode": ConfigurationParameterValue(value="local_ned"),
            "initial_position_m": ConfigurationParameterValue(value=(0.0, 0.0, -1000.0), unit="m"),
            "initial_attitude_quaternion": ConfigurationParameterValue(value=(1.0, 0.0, 0.0, 0.0)),
            "heading_deg": ConfigurationParameterValue(value=45.0, unit="deg"),
            "success_probability": ConfigurationParameterValue(value=0.95),
        }
    )
    segments = ConfigurationSequenceValue(
        items=(
            ConfigurationChoiceValue(
                selected="hold",
                value=ConfigurationGroupValue(values={"duration_s": ConfigurationParameterValue(value=1.0, unit="s")}),
            ),
            ConfigurationChoiceValue(
                selected="deploy",
                value=ConfigurationGroupValue(values={"duration_s": ConfigurationParameterValue(value=1.0, unit="s")}),
            ),
        )
    )
    motion_selected = "constant_speed" if fidelity == "coarse" else "constant_acceleration"
    motion_values: dict[str, ConfigurationNodeValue] = (
        {"speed_m_s": ConfigurationParameterValue(value=100.0, unit="m/s")}
        if motion_selected == "constant_speed"
        else {
            "initial_speed_m_s": ConfigurationParameterValue(value=100.0, unit="m/s"),
            "acceleration_m_s2": ConfigurationParameterValue(value=2.0, unit="m/s^2"),
        }
    )
    return TrajectoryConfigurationInstance(
        configuration_id=f"contract-probe-{fidelity}",
        model_id=CONTRACT_PROBE_MODEL_ID,
        model_version=CONTRACT_PROBE_MODEL_VERSION,
        schema_fingerprint=schema.fingerprint,
        fidelity=fidelity,
        realization_id=fidelity,
        mission_template_id="deployment_walkthrough",
        root=ConfigurationGroupValue(
            values={
                "initialization": initialization,
                "motion_mode": ConfigurationChoiceValue(
                    selected=motion_selected,
                    value=ConfigurationGroupValue(values=motion_values),
                ),
                "segments": segments,
                "payload": ConfigurationOptionalValue(enabled=False),
            }
        ),
    )
    ####


def _probe_properties() -> tuple[TrajectoryModelPropertyMetadata, ...]:
    scalar_format = NumericPresentationMetadata(decimal_places=2, use_grouping=True)
    return (
        _property("debug_identity", "Debug Identity", "identity", "string", "declared", "CONTRACT-PROBE", group="identity", order=10),
        _property("physical_model", "Physical Model", "identity", "boolean", "exact", False, group="identity", order=20),
        _property(
            "reference_mass_kg",
            "Reference Mass",
            "mass",
            "number",
            "nominal",
            1250.0,
            quantity="mass",
            unit="kg",
            group="physical",
            order=30,
            format=scalar_format,
        ),
        _property("axis_count", "Axis Count", "geometry", "integer", "exact", 3, group="physical", order=40),
        _property(
            "debug_mode",
            "Debug Mode",
            "implementation",
            "enum",
            "declared",
            "full_surface",
            choices=("minimal", "full_surface"),
            group="implementation",
            order=50,
        ),
        _property(
            "reference_vector",
            "Reference Vector",
            "geometry",
            "vector3",
            "representative",
            (1.0, 2.0, 3.0),
            quantity="length",
            unit="m",
            group="physical",
            order=60,
        ),
        _property("reference_quaternion", "Reference Quaternion", "geometry", "vector4", "representative", (1.0, 0.0, 0.0, 0.0), group="physical", order=70),
        TrajectoryModelPropertyMetadata(
            id="demonstrated_speed_range",
            label="Demonstrated Speed Range",
            description="Range-shaped property for renderer and transport conformance.",
            semantic_role="performance",
            value_type="number",
            value_kind="range",
            interval=_interval(0.0, 500.0),
            quantity="speed",
            canonical_unit="m/s",
            display_unit="kt",
            presentation=ValuePresentationMetadata(group="performance", order=80, format=scalar_format),
            provenance="synthetic contract probe",
            claim_boundary="Synthetic range used only to exercise range presentation.",
        ),
        TrajectoryModelPropertyMetadata(
            id="qualification_limit",
            label="Qualification Limit",
            description="Unknown-valued property proving that missing evidence can be advertised explicitly.",
            semantic_role="evidence",
            value_type="number",
            value_kind="unknown",
            quantity="load_factor",
            canonical_unit="g0",
            display_unit="g0",
            presentation=ValuePresentationMetadata(group="evidence", order=90, visibility="expert"),
            provenance="synthetic contract probe",
            claim_boundary="Unknown by design; consumers must not infer a numeric value.",
        ),
    )
    ####


def _property(
    property_id: str,
    label: str,
    semantic_role: Literal["identity", "geometry", "mass", "performance", "capability", "evidence", "implementation"],
    value_type: Literal["number", "integer", "boolean", "string", "enum", "vector3", "vector4"],
    value_kind: Literal["exact", "nominal", "representative", "limit", "range", "declared", "unknown"],
    value: object,
    *,
    quantity: str | None = None,
    unit: str | None = None,
    choices: tuple[str, ...] = (),
    group: str,
    order: int,
    format: NumericPresentationMetadata | None = None,
) -> TrajectoryModelPropertyMetadata:
    return TrajectoryModelPropertyMetadata(
        id=property_id,
        label=label,
        description=f"Synthetic {label.casefold()} property for contract coverage.",
        semantic_role=semantic_role,
        value_type=value_type,
        value_kind=value_kind,
        value=value,
        value_declared=True,
        choices=choices,
        quantity=quantity,
        canonical_unit=unit,
        display_unit=unit,
        presentation=ValuePresentationMetadata(group=group, order=order, format=format),
        provenance="synthetic contract probe",
        claim_boundary="Value exists only to exercise the metadata contract.",
    )
    ####


def _probe_fidelities() -> tuple[TrajectoryFidelityMetadata, ...]:
    return (
        TrajectoryFidelityMetadata(
            id="coarse",
            label="Coarse Debug Fidelity",
            rank=0,
            declared=True,
            dynamics_fidelity="point_mass_3dof",
            input_realization="guidance_command",
            compatibility_aliases=("coarse",),
            runtime_fidelity="synthetic_coarse",
            control_realization="kinematic",
            promotion_status="contract_witness",
            operations=("validate", "batch", "step"),
            profile_id="contract-probe-coarse",
            claim_boundary="Synthetic contract tier only.",
        ),
        TrajectoryFidelityMetadata(
            id="medium",
            label="Medium Debug Fidelity",
            rank=1,
            declared=True,
            dynamics_fidelity="pseudo_6dof",
            input_realization="guidance_command",
            compatibility_aliases=("medium",),
            runtime_fidelity="synthetic_medium",
            control_realization="point_mass_surrogate",
            promotion_status="contract_witness",
            operations=("validate", "batch", "step"),
            profile_id="contract-probe-medium",
            claim_boundary="Synthetic contract tier only.",
        ),
        TrajectoryFidelityMetadata(
            id="high",
            label="High Debug Fidelity",
            rank=2,
            declared=True,
            dynamics_fidelity="rigid_body_6dof",
            input_realization="provider_defined",
            compatibility_aliases=("high",),
            runtime_fidelity="synthetic_high",
            control_realization="declared_only",
            promotion_status="blocked",
            operations=("validate",),
            blockers=("No high-fidelity physical plant exists for the synthetic probe.",),
            required_operations=("batch", "step"),
            claim_boundary="Declared blocked tier used to prove fidelity feedback.",
        ),
    )
    ####


def _probe_control_advertisement(
    fidelity: TrajectoryFidelityMetadata,
) -> TrajectoryControlAdvertisement:
    """Publish the exhaustive control and agent-action conformance surface.

    This is deliberately an advertisement witness, not a claim that every
    synthetic command changes the batch result.  A generic consumer must be
    able to render, serialize, normalize, mask, and replay every channel
    shape below without deriving semantics from a label or a UI widget.
    """

    batch_available = "batch" in fidelity.operations
    step_available = "step" in fidelity.operations
    availability: TrajectoryControlAvailability = "available_in_batch" if batch_available else "planned"
    operations: tuple[Literal["batch", "step"], ...] = ("batch",) if batch_available else ()
    live_channel_ids = {
        "attitude.quaternion.command",
        "guidance.heading.command",
        "guidance.heading_rate.command",
        "guidance.acceleration.increment",
        "aerodynamics.flap.detent",
        "aerodynamics.spoiler.increment",
        "autopilot.mode.select",
        "safety.deadman",
        "payload.release.command",
        "payload.arm.command",
    }
    speed_interval = _interval(0.0, 500.0)
    heading_interval = _interval(0.0, 360.0, maximum_inclusive=False)
    flap_interval = _interval(-10.0, 20.0)
    spoiler_interval = _interval(-1.0, 1.0)
    speed_space = ConfigurationValueSpace(
        topology="bounded_interval",
        representation="scalar",
        error_rule="componentwise subtraction",
        interpolation_rule="linear",
        coordinate_chart="[0, 500]",
    )
    quaternion_space = ConfigurationValueSpace(
        topology="unit_quaternion",
        representation="vector4",
        error_rule="shortest rotation",
        interpolation_rule="slerp",
        normalization_rule="unit norm with canonical sign",
        equivalence="q and -q",
        coordinate_chart="S^3 / {q ~ -q}",
    )
    periodic_heading_space = ConfigurationValueSpace(
        topology="periodic_circle",
        representation="scalar",
        error_rule="wrapped signed angular difference",
        interpolation_rule="shortest_arc",
        period=360.0,
        coordinate_chart="[0, 360)",
    )
    unbounded_scalar_space = ConfigurationValueSpace(
        topology="euclidean",
        representation="scalar",
        error_rule="subtraction",
        interpolation_rule="linear",
        coordinate_chart="R",
    )
    boolean_space = ConfigurationValueSpace(
        topology="boolean",
        representation="boolean",
        error_rule="exact equality",
        interpolation_rule="not interpolable",
    )
    enum_space = ConfigurationValueSpace(
        topology="finite_set",
        representation="string",
        error_rule="exact equality",
        interpolation_rule="not interpolable",
    )
    event_space = ConfigurationValueSpace(
        topology="event",
        representation="named_event",
        error_rule="event identity",
        interpolation_rule="not interpolable",
    )
    detent_space = ConfigurationValueSpace(
        topology="bounded_interval",
        representation="scalar",
        error_rule="difference after detent quantization",
        interpolation_rule="step",
    )
    composite_effector_space = ConfigurationValueSpace(
        topology="product",
        representation="provider_defined_composite",
        error_rule="provider-defined by component",
        interpolation_rule="provider-defined by component",
        components=(speed_space, enum_space, boolean_space),
    )

    def channel(
        channel_id: str,
        label: str,
        description: str,
        value_space: ConfigurationValueSpace,
        semantics: ControlCommandSemantics,
        *,
        order: int,
        channel_kind: Literal["action", "effector"] = "action",
        quantity: str | None = None,
        canonical_unit: str | None = None,
        display_unit: str | None = None,
        data_type: Literal["float64", "int64", "boolean", "string", "json"] = "float64",
        shape: tuple[int | Literal["variable"], ...] = (),
        interval: ConfigurationInterval | None = None,
        choices: tuple[str, ...] = (),
        frame: str | None = None,
        sampling_semantics: Literal[
            "held_action",
            "batch_profile",
            "segment_generated",
            "not_sampled",
            "provider_reported",
            "event",
        ] = "held_action",
        control: Literal[
            "automatic",
            "number_input",
            "slider",
            "toggle",
            "select",
            "button",
            "stepper",
            "dial",
            "text",
            "vector_editor",
            "coordinate_picker",
        ] = "automatic",
    ) -> TrajectoryControlChannelMetadata:
        """Build a semantic/native pair with deliberately matching metadata."""

        native_id = f"synthetic.{channel_id}"
        channel_operations: tuple[Literal["batch", "step"], ...] = operations
        channel_availability = availability
        if step_available and channel_id in live_channel_ids:
            channel_operations = ("batch", "step") if batch_available else ("step",)
            channel_availability = "available"
        return TrajectoryControlChannelMetadata(
            id=channel_id,
            label=label,
            description=description,
            channel_kind=channel_kind,
            quantity=quantity,
            canonical_unit=canonical_unit,
            display_unit=display_unit,
            data_type=data_type,
            shape=shape,
            interval=interval,
            choices=choices,
            frame=frame,
            sampling_semantics=sampling_semantics,
            value_space=value_space,
            semantics=semantics,
            availability=channel_availability,
            operations=channel_operations,
            native_channel_id=native_id,
            native_binding=TrajectoryControlNativeBindingMetadata(
                id=native_id,
                quantity=quantity,
                canonical_unit=canonical_unit,
                data_type=data_type,
                shape=shape,
                interval=interval,
                value_space=value_space,
                semantics=semantics,
                provider_binding={"debug_field": channel_id},
            ),
            provider_binding={"debug_field": channel_id},
            presentation=ValuePresentationMetadata(group="controls", order=order, control=control),
            source_refs=("src/taoryx/trajectory/contract_probe_mission_composition.py",),
            provenance="synthetic contract probe",
            claim_boundary="Synthetic transport, renderer, and agent-action witness only.",
        )
        ####

    channels = (
        channel(
            "guidance.speed.command",
            "Speed Command",
            "Bounded continuous profile command with an affine agent projection.",
            speed_space,
            ControlCommandSemantics(
                value_domain="continuous",
                command_mode="absolute",
                temporal_semantics="profile",
                release_behavior="hold",
                agent_normalization="auto",
                agent_clip=True,
            ),
            order=10,
            quantity="speed",
            canonical_unit="m/s",
            display_unit="m/s",
            interval=speed_interval,
            sampling_semantics="batch_profile",
            control="slider",
        ),
        channel(
            "attitude.quaternion.command",
            "Attitude Quaternion Command",
            "Fixed-size vector command with an explicit release value and identity agent projection.",
            quaternion_space,
            ControlCommandSemantics(
                value_domain="vector",
                command_mode="absolute",
                temporal_semantics="held",
                release_behavior="release_value",
                release_value=(1.0, 0.0, 0.0, 0.0),
                agent_normalization="identity",
            ),
            order=20,
            quantity="orientation",
            shape=(4,),
            frame="body",
            sampling_semantics="held_action",
            control="vector_editor",
        ),
        channel(
            "mission.enable",
            "Mission Enable",
            "Latched boolean switch with an explicit fail-safe release policy.",
            boolean_space,
            ControlCommandSemantics(
                value_domain="boolean",
                command_mode="absolute",
                temporal_semantics="latched",
                release_behavior="failsafe",
            ),
            order=30,
            data_type="boolean",
            sampling_semantics="segment_generated",
            control="toggle",
        ),
        channel(
            "guidance.heading.command",
            "Heading Command",
            "Periodic absolute heading command; agent values wrap deterministically at the principal boundary.",
            periodic_heading_space,
            ControlCommandSemantics(
                value_domain="periodic",
                command_mode="absolute",
                temporal_semantics="held",
                release_behavior="hold",
                agent_normalization="periodic_wrap",
            ),
            order=40,
            quantity="angle",
            canonical_unit="deg",
            display_unit="deg",
            interval=heading_interval,
            sampling_semantics="held_action",
            control="dial",
        ),
        channel(
            "guidance.heading_rate.command",
            "Heading-Rate Command",
            "Unbounded rate command with provider-declared normalization statistics.",
            unbounded_scalar_space,
            ControlCommandSemantics(
                value_domain="continuous",
                command_mode="rate",
                temporal_semantics="sampled",
                release_behavior="default",
                rate_unit="deg/s",
                agent_normalization="standardize",
                agent_center=0.0,
                agent_scale=45.0,
                agent_clip=True,
            ),
            order=50,
            quantity="angular_rate",
            canonical_unit="deg/s",
            display_unit="deg/s",
            sampling_semantics="held_action",
            control="number_input",
        ),
        channel(
            "guidance.acceleration.increment",
            "Acceleration Increment",
            "Unbounded incremental command that intentionally requires external training statistics.",
            unbounded_scalar_space,
            ControlCommandSemantics(
                value_domain="continuous",
                command_mode="increment",
                temporal_semantics="sampled",
                release_behavior="release_value",
                release_value=0.0,
                agent_normalization="auto",
            ),
            order=60,
            quantity="acceleration",
            canonical_unit="m/s^2",
            display_unit="m/s^2",
            sampling_semantics="held_action",
            control="number_input",
        ),
        channel(
            "aerodynamics.flap.detent",
            "Flap Detent",
            "Latched finite set of physically named flap detents.",
            detent_space,
            ControlCommandSemantics(
                value_domain="discrete_levels",
                command_mode="absolute",
                temporal_semantics="latched",
                release_behavior="hold",
                quantization=ControlQuantizationMetadata(
                    mode="levels",
                    levels=(-10.0, 0.0, 10.0, 20.0),
                    rounding="reject",
                ),
            ),
            order=70,
            quantity="angle",
            canonical_unit="deg",
            display_unit="deg",
            interval=flap_interval,
            sampling_semantics="held_action",
            control="stepper",
        ),
        channel(
            "aerodynamics.spoiler.increment",
            "Spoiler Detent Increment",
            "Sampled discrete increment with a fixed quantization grid and nearest-detent policy.",
            detent_space,
            ControlCommandSemantics(
                value_domain="discrete_levels",
                command_mode="increment",
                temporal_semantics="sampled",
                release_behavior="release_value",
                release_value=0.0,
                quantization=ControlQuantizationMetadata(
                    mode="step",
                    step=0.5,
                    origin=0.0,
                    rounding="nearest",
                ),
            ),
            order=80,
            quantity="normalized_control",
            interval=spoiler_interval,
            sampling_semantics="held_action",
            control="stepper",
        ),
        channel(
            "autopilot.mode.select",
            "Autopilot Mode",
            "Latched categorical selector with stable advertised ordering.",
            enum_space,
            ControlCommandSemantics(
                value_domain="enum",
                command_mode="absolute",
                temporal_semantics="latched",
                release_behavior="hold",
            ),
            order=90,
            data_type="string",
            choices=("manual", "hold", "track"),
            sampling_semantics="held_action",
            control="select",
        ),
        channel(
            "safety.deadman",
            "Deadman Switch",
            "Momentary boolean control that returns to its provider default when released.",
            boolean_space,
            ControlCommandSemantics(
                value_domain="boolean",
                command_mode="absolute",
                temporal_semantics="momentary",
                release_behavior="default",
            ),
            order=100,
            data_type="boolean",
            sampling_semantics="held_action",
            control="toggle",
        ),
        channel(
            "payload.release.command",
            "Release Payload",
            "One-shot pulse event requiring an action mask after it fires in an episode.",
            event_space,
            ControlCommandSemantics(
                value_domain="event",
                command_mode="event",
                temporal_semantics="pulse",
                release_behavior="auto_reset",
                pulse_duration_s=0.1,
                repeat_policy="once_per_episode",
            ),
            order=110,
            data_type="string",
            choices=("release",),
            sampling_semantics="event",
            control="button",
        ),
        channel(
            "payload.arm.command",
            "Arm Payload",
            "Pulse event that remains unavailable until the session is reset after it fires.",
            event_space,
            ControlCommandSemantics(
                value_domain="event",
                command_mode="event",
                temporal_semantics="pulse",
                release_behavior="auto_reset",
                pulse_duration_s=0.1,
                repeat_policy="once_until_reset",
            ),
            order=120,
            data_type="string",
            choices=("arm",),
            sampling_semantics="event",
            control="button",
        ),
        channel(
            "debug.composite.effector",
            "Provider-Defined Composite Effector",
            "Heterogeneous recursive value-space tree retained for provider-specific consumers.",
            composite_effector_space,
            ControlCommandSemantics(
                value_domain="provider_defined",
                command_mode="absolute",
                temporal_semantics="sampled",
                release_behavior="failsafe",
            ),
            order=130,
            channel_kind="effector",
            data_type="json",
            shape=("variable",),
            sampling_semantics="provider_reported",
            control="automatic",
        ),
        channel(
            "debug.unsampled.effector",
            "Unsampled Effector Declaration",
            "Declared effect without an emitted sample, retained to exercise the explicit not-sampled state.",
            composite_effector_space,
            ControlCommandSemantics(
                value_domain="provider_defined",
                command_mode="absolute",
                temporal_semantics="sampled",
                release_behavior="failsafe",
            ),
            order=140,
            channel_kind="effector",
            data_type="json",
            shape=("variable",),
            sampling_semantics="not_sampled",
            control="automatic",
        ),
    )
    resolution: Literal["provider_internal", "blocked"] = "provider_internal" if batch_available else "blocked"
    authorities: list[TrajectoryControlAuthorityMetadata] = [
        TrajectoryControlAuthorityMetadata(
            id="synthetic_mission_authority",
            authority="provider_defined",
            availability=availability,
            channel_ids=tuple(item.id for item in channels),
            operations=operations,
            description="Synthetic mutually exclusive batch authority over every probe command.",
            command_owner="provider_controller",
            selection_scope="provider",
            switching_policy="provider_managed",
            scheme_id="provider.program",
            lowering_chain=("synthetic_mission_program", "synthetic_probe_commands"),
            source_refs=("src/taoryx/trajectory/contract_probe_mission_composition.py",),
            provenance="synthetic contract probe",
            claim_boundary="Debug contract coverage only.",
        )
    ]
    if step_available:
        authorities.extend(
            (
                TrajectoryControlAuthorityMetadata(
                    id="debug_guidance_control",
                    authority="kinematic",
                    availability="available",
                    channel_ids=(
                        "attitude.quaternion.command",
                        "guidance.heading.command",
                        "guidance.heading_rate.command",
                        "guidance.acceleration.increment",
                    ),
                    operations=("step",),
                    description="Caller-owned vector, periodic, rate, and increment stress surface.",
                    command_owner="caller",
                    selection_scope="session",
                    switching_policy="explicit_bumpless",
                    scheme_id="debug.mixed",
                    lowering_chain=("debug_guidance_hold", "synthetic_kinematic_transition"),
                    provenance="synthetic contract probe",
                    claim_boundary="Transport and action-schema stress only; no physical guidance claim.",
                ),
                TrajectoryControlAuthorityMetadata(
                    id="debug_discrete_control",
                    authority="native_bridge",
                    availability="available",
                    channel_ids=(
                        "aerodynamics.flap.detent",
                        "aerodynamics.spoiler.increment",
                        "autopilot.mode.select",
                        "safety.deadman",
                    ),
                    operations=("step",),
                    description="Caller-owned detent, increment, enum, and boolean stress surface.",
                    command_owner="caller",
                    selection_scope="session",
                    switching_policy="explicit_bumpless",
                    scheme_id="debug.discrete",
                    lowering_chain=("debug_discrete_hold", "synthetic_mode_transition"),
                    provenance="synthetic contract probe",
                    claim_boundary="Transport and action-schema stress only; controls are deliberately synthetic.",
                ),
                TrajectoryControlAuthorityMetadata(
                    id="debug_event_control",
                    authority="mission",
                    availability="available",
                    channel_ids=("payload.release.command", "payload.arm.command"),
                    operations=("step",),
                    description="Caller-owned once-per-episode event stress surface.",
                    command_owner="caller",
                    selection_scope="session",
                    switching_policy="explicit_bumpless",
                    scheme_id="debug.event",
                    lowering_chain=("debug_event_gate", "synthetic_event_record"),
                    provenance="synthetic contract probe",
                    claim_boundary="Event repeat-policy witness only; no payload is physically modeled.",
                ),
            )
        )
    return TrajectoryControlAdvertisement(
        status="available" if step_available else "internally_generated" if batch_available else "blocked",
        channels=channels,
        authorities=tuple(authorities),
        intents=(
            TrajectoryControlIntentMetadata(
                id="deployment_guidance",
                label="Deployment Guidance",
                description="Synthetic intent resolved during the deploy segment.",
                resolution=resolution,
                segment_ids=("deploy",),
                mission_template_ids=("deployment_walkthrough",),
                channel_ids=tuple(item.id for item in channels),
                operations=operations,
                source_refs=("src/taoryx/trajectory/contract_probe_mission_composition.py",),
                provenance="synthetic contract probe",
                claim_boundary="Debug contract coverage only.",
            ),
        ),
        default_authority_id=(
            "debug_guidance_control"
            if step_available
            else "synthetic_mission_authority"
            if batch_available
            else None
        ),
        claim_boundary="Synthetic full-surface control advertisement for generic composer development.",
    )
    ####


def _probe_fidelity_transitions() -> tuple[TrajectoryFidelityTransition, ...]:
    return (
        _transition("coarse", "medium", "step_up", "available", automatic=False, policy="explicit_upgrade_only"),
        _transition("medium", "coarse", "step_down", "available", automatic=True, policy="validated_lower_only"),
        _transition("medium", "high", "step_up", "conditional", automatic=False, policy="explicit_upgrade_only"),
        _transition("high", "medium", "step_down", "available", automatic=False, policy="exact_only"),
    )
    ####


def _transition(
    source: str,
    target: str,
    direction: Literal["step_up", "step_down"],
    status: Literal["available", "conditional", "blocked", "not_available"],
    *,
    automatic: bool,
    policy: Literal["exact_only", "validated_lower_only", "explicit_upgrade_only"],
) -> TrajectoryFidelityTransition:
    return TrajectoryFidelityTransition(
        from_fidelity=source,
        to_fidelity=target,
        direction=direction,
        status=status,
        automatic=automatic,
        selection_policy=policy,
        requirements=("explicit caller selection",) if direction == "step_up" else ("state projection accepted",),
        state_transfer="Synthetic identity projection with an explicit transition record.",
        claim_boundary="Exercises fidelity-selection metadata; no physical equivalence is claimed.",
    )
    ####


def _probe_mission_templates() -> tuple[TrajectoryMissionTemplateMetadata, ...]:
    return (
        TrajectoryMissionTemplateMetadata(
            id="deployment_walkthrough",
            name="Deployment Walkthrough",
            description="Two-segment path that emits a child object through the standardized lineage contract.",
            status="runnable_debug_contract",
            initialization_variants=("debug_initial_state",),
            segment_sequence=("hold", "deploy"),
            compatible_fidelities=("coarse", "medium"),
            operations=tuple(item for item in _probe_operations() if item.fidelity in {"coarse", "medium"}),
            provenance="synthetic contract probe",
            claim_boundary="Contract coverage only.",
        ),
    )
    ####


def _probe_operations() -> tuple[TrajectoryMissionOperationMetadata, ...]:
    records: list[TrajectoryMissionOperationMetadata] = []
    operations: tuple[Literal["validate", "batch", "step"], ...] = ("validate", "batch", "step")
    for fidelity in ("coarse", "medium"):
        for operation in operations:
            available = operation in {"validate", "batch", "step"} and fidelity in {"coarse", "medium"}
            common_status: Literal["registered", "adapter_required", "not_available"] = (
                "registered" if available and operation in {"batch", "step"} else "not_available"
            )
            records.append(
                TrajectoryMissionOperationMetadata(
                    fidelity=fidelity,
                    realization_id=fidelity,
                    operation=operation,
                    status="available" if available else "blocked",
                    execution_mode=(
                        "synthetic_provider_session"
                        if available and operation == "step"
                        else "synthetic_deterministic"
                        if available
                        else None
                    ),
                    availability_scope="provider_interface",
                    common_runner_status=common_status,
                    executor_id=(
                        "contract-probe-session.v1"
                        if common_status == "registered" and operation == "step"
                        else "contract-probe-executor"
                        if common_status == "registered"
                        else None
                    ),
                    blockers=() if available else (f"{operation} is intentionally blocked at {fidelity} fidelity.",),
                    claim_boundary="Synthetic operation-state witness only.",
                )
            )
    return tuple(records)
    ####


def _probe_deployments() -> tuple[TrajectoryDeploymentMetadata, ...]:
    return (
        TrajectoryDeploymentMetadata(
            id="debug_child_release",
            name="Debug Child Release",
            description="Available independently propagated child-object path.",
            status="available",
            trigger_segment_ids=("deploy",),
            trigger_event_kinds=("debug_child_released",),
            child_role="debug_payload",
            child_model_id="contract_probe_child",
            child_model_scope="provider_generated",
            child_model_kind="synthetic_child",
            minimum_children=1,
            maximum_children=1,
            compatible_fidelities=("coarse", "medium"),
            operations=("batch",),
            common_runner_status="registered",
            executor_id="contract-probe-executor",
            state_initialization="inherited_at_accepted_boundary",
            fidelity_policy="provider_mapped",
            lifecycle="independently_propagated",
            provenance="synthetic contract probe",
            claim_boundary="Lineage and result-shape witness only.",
        ),
        TrajectoryDeploymentMetadata(
            id="debug_descendant_release",
            name="Debug Descendant Release",
            description="Available child-to-grandchild propagation path proving recursive entity lineage.",
            status="available",
            trigger_segment_ids=("deploy",),
            trigger_event_kinds=("debug_descendant_released",),
            child_role="debug_subpayload",
            child_model_id="contract_probe_grandchild",
            child_model_scope="provider_generated",
            child_model_kind="synthetic_descendant",
            minimum_children=1,
            maximum_children=1,
            compatible_fidelities=("coarse", "medium"),
            operations=("batch",),
            common_runner_status="registered",
            executor_id="contract-probe-executor",
            state_initialization="inherited_at_accepted_boundary",
            fidelity_policy="provider_mapped",
            lifecycle="independently_propagated",
            provenance="synthetic contract probe",
            claim_boundary="Recursive lineage and result-shape witness only.",
        ),
        TrajectoryDeploymentMetadata(
            id="declared_event_only_release",
            name="Declared Event-Only Release",
            description="Declared event without child propagation.",
            status="declared",
            trigger_segment_ids=("deploy",),
            trigger_event_kinds=("declared_release",),
            child_role="event_marker",
            child_model_scope="provider_generated",
            child_model_kind="event_only",
            compatible_fidelities=_FIDELITIES,
            state_initialization="not_advertised",
            fidelity_policy="provider_mapped",
            lifecycle="event_only",
            claim_boundary="Exercises declared event-only advertisement state.",
        ),
        TrajectoryDeploymentMetadata(
            id="blocked_external_release",
            name="Blocked External Release",
            description="External child path with an explicit blocker.",
            status="blocked",
            trigger_segment_ids=("deploy",),
            trigger_event_kinds=("external_release",),
            child_role="external_booster",
            child_model_id="external.booster",
            child_model_scope="external",
            child_model_kind="external_model",
            compatible_fidelities=("high",),
            state_initialization="external",
            fidelity_policy="external",
            lifecycle="external",
            blockers=("External provider is intentionally absent.",),
            claim_boundary="Exercises blocked external-dependency feedback.",
        ),
        TrajectoryDeploymentMetadata(
            id="unavailable_release",
            name="Unavailable Release",
            description="Explicitly unavailable deployment state.",
            status="not_available",
            trigger_segment_ids=("deploy",),
            trigger_event_kinds=("unavailable_release",),
            child_role="none",
            child_model_scope="provider_generated",
            child_model_kind="none",
            compatible_fidelities=_FIDELITIES,
            state_initialization="not_advertised",
            fidelity_policy="provider_mapped",
            lifecycle="event_only",
            claim_boundary="Exercises explicit not-available advertisement state.",
        ),
    )
    ####


def _probe_reference_frames() -> tuple[TrajectoryReferenceFrameMetadata, ...]:
    return (
        TrajectoryReferenceFrameMetadata(
            id="local_ned",
            name="Local North-East-Down",
            description="Right-handed local tangent frame.",
            frame_kind="local_tangent",
            axes=("north", "east", "down"),
            handedness="right",
            origin="synthetic launch point",
            orientation="north-east-down",
            provenance="contract probe",
        ),
        TrajectoryReferenceFrameMetadata(
            id="body",
            name="Body Frame",
            description="Right-handed body-fixed forward-right-down frame.",
            frame_kind="body",
            axes=("forward", "right", "down"),
            handedness="right",
            origin="synthetic center of mass",
            orientation="forward-right-down",
            provenance="contract probe",
        ),
        TrajectoryReferenceFrameMetadata(
            id="earth_fixed",
            name="Earth Fixed",
            description="Synthetic Earth-fixed Cartesian frame.",
            frame_kind="earth_fixed",
            axes=("x", "y", "z"),
            handedness="right",
            origin="synthetic Earth center",
            orientation="provider-defined Earth-fixed axes",
            provenance="contract probe",
        ),
    )
    ####


def _probe_output_channels() -> tuple[TrajectoryOutputChannelMetadata, ...]:
    specifications: tuple[
        tuple[
            str,
            str,
            str | None,
            str | None,
            str | None,
            Literal["linear", "step", "periodic", "slerp", "event"],
            Literal["float64", "int64", "boolean", "string", "json"],
            tuple[int | Literal["variable"], ...],
            Literal["continuous_sample", "discrete_sample", "event", "provider_reported"],
        ],
        ...,
    ] = (
        ("position.north_m", "North Position", "length", "m", "local_ned", "linear", "float64", (), "continuous_sample"),
        ("velocity.speed_m_s", "Speed", "speed", "m/s", None, "linear", "float64", (), "continuous_sample"),
        ("attitude.heading_deg", "Heading", "angle", "deg", "local_ned", "periodic", "float64", (), "continuous_sample"),
        ("mode.index", "Mode Index", "discrete", None, None, "step", "int64", (), "discrete_sample"),
        ("attitude.quaternion", "Attitude Quaternion", "dimensionless", None, "body", "slerp", "float64", (4,), "continuous_sample"),
        ("health.valid", "Health Valid", "boolean", None, None, "step", "boolean", (), "discrete_sample"),
        ("status.mode", "Mode Name", "categorical", None, None, "step", "string", (), "discrete_sample"),
        ("diagnostics.payload", "Diagnostic Payload", None, None, None, "step", "json", (), "provider_reported"),
        ("event.marker", "Event Marker", "discrete", None, None, "event", "int64", (), "event"),
    )
    result: list[TrajectoryOutputChannelMetadata] = []
    for order, (channel_id, label, quantity, unit, frame, interpolation, data_type, shape, sampling) in enumerate(
        specifications,
        start=1,
    ):
        periodicity = ConfigurationPeriodicity(period=360.0, canonical_minimum=0.0) if interpolation == "periodic" else None
        result.append(
            TrajectoryOutputChannelMetadata(
                id=channel_id,
                label=label,
                description=f"Synthetic {label.casefold()} channel exercising {interpolation} interpolation metadata.",
                quantity=quantity,
                canonical_unit=unit,
                display_unit=unit,
                data_type=data_type,
                shape=shape,
                frame=frame,
                sampling_semantics=sampling,
                interpolation=interpolation,
                periodicity=periodicity,
                availability="guaranteed",
                compatible_fidelities=_FIDELITIES,
                operations=("batch", "step"),
                presentation=ValuePresentationMetadata(
                    group=channel_id.split(".", maxsplit=1)[0],
                    order=order * 10,
                    visibility="debug" if order > 4 else "primary",
                    format=NumericPresentationMetadata(decimal_places=3),
                ),
                provenance="synthetic contract probe",
                claim_boundary="Synthetic channel for interface coverage only.",
            )
        )
    return tuple(result)
    ####


def _probe_output_schema(
    channels: tuple[TrajectoryOutputChannelMetadata, ...],
) -> TrajectoryOutputSchema:
    channel_by_id = {item.id: item for item in channels}
    core_ids = ("position.north_m", "velocity.speed_m_s", "attitude.heading_deg")
    telemetry_ids = (
        "mode.index",
        "attitude.quaternion",
        "health.valid",
        "status.mode",
        "diagnostics.payload",
        "event.marker",
    )
    return TrajectoryOutputSchema(
        model_id=CONTRACT_PROBE_MODEL_ID,
        model_version=CONTRACT_PROBE_MODEL_VERSION,
        core_channels=tuple(channel_by_id[item] for item in core_ids),
        telemetry_channels=tuple(channel_by_id[item] for item in telemetry_ids),
        telemetry_groups=(
            TrajectoryTelemetryGroupMetadata(
                id="control",
                label="Control and Mode",
                description="Synthetic discrete controller and mode telemetry.",
                channel_ids=("mode.index", "status.mode"),
                default_selected=True,
                presentation=ValuePresentationMetadata(group="telemetry", order=10),
            ),
            TrajectoryTelemetryGroupMetadata(
                id="attitude_detail",
                label="Attitude Detail",
                description="Optional higher-detail attitude representation.",
                channel_ids=("attitude.quaternion",),
                presentation=ValuePresentationMetadata(group="telemetry", order=20),
            ),
            TrajectoryTelemetryGroupMetadata(
                id="diagnostics",
                label="Typed Diagnostics",
                description="Boolean and structured JSON telemetry values.",
                channel_ids=("health.valid", "diagnostics.payload"),
                presentation=ValuePresentationMetadata(group="telemetry", order=30, visibility="debug"),
            ),
            TrajectoryTelemetryGroupMetadata(
                id="debug_events",
                label="Debug Event Channels",
                description="Sample-aligned debug marker distinct from structural lifecycle events.",
                channel_ids=("event.marker",),
                presentation=ValuePresentationMetadata(group="telemetry", order=40, visibility="debug"),
            ),
        ),
        entity_output=TrajectoryEntityOutputMetadata(
            supports_multiple_entities=True,
            supports_dynamic_spawning=True,
            supports_recursive_spawning=True,
            maximum_descendant_depth=2,
            relationship_kinds=("deployment",),
            child_output_schema_policy="same_as_parent",
            includes_spawn_initial_state=True,
            includes_lifecycle_events=True,
        ),
        claim_boundary="Synthetic full-surface output and entity-lineage witness only.",
    )
    ####


def _probe_result(
    request: MissionCompositionRunRequest,
    advertised_channels: tuple[TrajectoryOutputChannelMetadata, ...],
    *,
    include_child: bool,
    include_descendant: bool,
) -> MissionCompositionTrajectoryResult:
    primary_id = f"{request.request_id}:primary"
    child_id = f"{request.request_id}:child-1"
    descendant_id = f"{request.request_id}:grandchild-1"
    deployment_event_id = f"{request.request_id}:debug-child-released"
    descendant_event_id = f"{request.request_id}:debug-descendant-released"
    primary_channels = tuple(_result_channel(item) for item in advertised_channels)
    primary_segments = (
        (
            TrajectorySegmentResult(
                id="hold",
                instance_id="01-hold",
                object_id=primary_id,
                start_time_s=0.0,
                end_time_s=1.0,
                status="completed",
            ),
            TrajectorySegmentResult(
                id="deploy",
                instance_id="02-deploy",
                object_id=primary_id,
                start_time_s=1.0,
                end_time_s=2.0,
                status="completed",
                event_ids=(deployment_event_id,) if include_child else (),
            ),
        )
        if request.output.include_segments
        else ()
    )
    primary = TrajectoryObject(
        object_id=primary_id,
        model_id=CONTRACT_PROBE_MODEL_ID,
        realization_id=request.prepared_configuration.configuration.realization_id or request.prepared_configuration.configuration.fidelity,
        name="Contract Probe Primary",
        role="primary_vehicle",
        fidelity=request.prepared_configuration.configuration.fidelity,
        status="completed",
        active_from_s=0.0,
        active_to_s=2.0,
        terminal_disposition="debug_sequence_complete",
        channels=primary_channels,
        samples=tuple(_probe_sample(time_s, advertised_channels, child=False) for time_s in (0.0, 1.0, 2.0)),
        segments=primary_segments,
        provenance="synthetic contract probe",
        claim_boundary="Deterministic interface witness only.",
    )
    objects: list[TrajectoryObject] = [primary]
    events: list[TrajectoryEvent] = []
    relationships: list[TrajectoryEntityRelationship] = []
    if include_child:
        events.append(
            TrajectoryEvent(
                id=deployment_event_id,
                time_s=1.0,
                category="deployment",
                kind="debug_child_released",
                object_id=child_id,
                parent_object_id=primary_id,
                deployment_id="debug_child_release",
                segment_instance_id="02-deploy",
                detail="Synthetic child emitted to exercise lineage.",
                data={"state_transfer": "inherited_at_accepted_boundary"},
            )
        )
        child_samples = tuple(_probe_sample(time_s, advertised_channels, child=True) for time_s in (1.0, 1.5, 2.0))
        objects.append(
            TrajectoryObject(
                object_id=child_id,
                model_id="contract_probe_child",
                realization_id="synthetic_child_v1",
                name="Contract Probe Child",
                role="debug_payload",
                fidelity="coarse",
                status="completed",
                parent_object_id=primary_id,
                deployment_id="debug_child_release",
                spawn_event_id=deployment_event_id,
                active_from_s=1.0,
                active_to_s=2.0,
                terminal_disposition="debug_child_complete",
                channels=primary_channels,
                samples=child_samples,
                segments=(
                    TrajectorySegmentResult(
                        id="deploy",
                        instance_id="child-01-deploy",
                        object_id=child_id,
                        start_time_s=1.0,
                        end_time_s=2.0,
                        status="completed",
                        event_ids=(descendant_event_id,) if include_descendant else (),
                    ),
                )
                if request.output.include_segments
                else (),
                provenance="synthetic child generated by the contract probe",
                claim_boundary="Lineage witness only.",
            )
        )
        relationships.append(
            TrajectoryEntityRelationship(
                id=f"{request.request_id}:relationship:debug-child-release",
                kind="deployment",
                parent_object_id=primary_id,
                child_object_id=child_id,
                deployment_id="debug_child_release",
                event_id=deployment_event_id,
                created_at_s=1.0,
                state_transfer="inherited_at_accepted_boundary",
                initial_state=TrajectoryStateSnapshot(time_s=1.0, values=child_samples[0].values),
                provenance="synthetic contract probe",
            )
        )
        if include_descendant:
            events.append(
                TrajectoryEvent(
                    id=descendant_event_id,
                    time_s=1.5,
                    category="deployment",
                    kind="debug_descendant_released",
                    object_id=descendant_id,
                    parent_object_id=child_id,
                    deployment_id="debug_descendant_release",
                    segment_instance_id="child-01-deploy" if request.output.include_segments else None,
                    detail="Synthetic grandchild emitted to exercise recursive lineage.",
                    data={"state_transfer": "inherited_at_accepted_boundary"},
                )
            )
            descendant_samples = tuple(_probe_sample(time_s, advertised_channels, child=True) for time_s in (1.5, 1.75, 2.0))
            objects.append(
                TrajectoryObject(
                    object_id=descendant_id,
                    model_id="contract_probe_grandchild",
                    realization_id="synthetic_descendant_v1",
                    name="Contract Probe Grandchild",
                    role="debug_subpayload",
                    fidelity="coarse",
                    status="completed",
                    parent_object_id=child_id,
                    deployment_id="debug_descendant_release",
                    spawn_event_id=descendant_event_id,
                    active_from_s=1.5,
                    active_to_s=2.0,
                    terminal_disposition="debug_descendant_complete",
                    channels=primary_channels,
                    samples=descendant_samples,
                    provenance="synthetic descendant generated by the contract probe",
                    claim_boundary="Recursive lineage witness only.",
                )
            )
            relationships.append(
                TrajectoryEntityRelationship(
                    id=f"{request.request_id}:relationship:debug-descendant-release",
                    kind="deployment",
                    parent_object_id=child_id,
                    child_object_id=descendant_id,
                    deployment_id="debug_descendant_release",
                    event_id=descendant_event_id,
                    created_at_s=1.5,
                    state_transfer="inherited_at_accepted_boundary",
                    initial_state=TrajectoryStateSnapshot(
                        time_s=1.5,
                        values=descendant_samples[0].values,
                    ),
                    provenance="synthetic contract probe",
                )
            )
    diagnostics = (
        MissionCompositionDiagnostic(
            severity="info",
            code="contract-probe-result",
            message="Synthetic contract-probe trajectory completed.",
            phase="projection",
            recoverability="degraded",
            provider_id=request.provider_id,
            model_id=request.model_id,
            object_id=primary_id,
            details={
                "operation": request.operation,
                "child_included": include_child,
                "descendant_included": include_descendant,
            },
        ),
        MissionCompositionDiagnostic(
            severity="warning",
            code="debug-result-not-physical",
            message="Do not use contract-probe values for physical analysis.",
            phase="projection",
            recoverability="degraded",
            provider_id=request.provider_id,
            model_id=request.model_id,
            object_id=primary_id,
        ),
    )
    return MissionCompositionTrajectoryResult(
        provider_id=request.provider_id,
        provider_version=request.provider_version,
        request_id=request.request_id,
        configuration_fingerprint=request.prepared_configuration.fingerprint,
        primary_model_id=CONTRACT_PROBE_MODEL_ID,
        primary_object_id=primary_id,
        status="completed",
        objects=tuple(objects),
        events=tuple(events),
        relationships=tuple(relationships),
        diagnostics=diagnostics,
        claim_boundary="Synthetic contract witness; no physical or qualification claim.",
    )
    ####


def _result_channel(channel: TrajectoryOutputChannelMetadata) -> TrajectoryChannelMetadata:
    telemetry_group_by_id = {
        "mode.index": "control",
        "status.mode": "control",
        "attitude.quaternion": "attitude_detail",
        "health.valid": "diagnostics",
        "diagnostics.payload": "diagnostics",
        "event.marker": "debug_events",
    }
    telemetry_group = telemetry_group_by_id.get(channel.id)
    return TrajectoryChannelMetadata(
        id=channel.id,
        channel_class="telemetry" if telemetry_group is not None else "core_state",
        telemetry_group=telemetry_group,
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


def _probe_sample(
    time_s: float,
    channels: tuple[TrajectoryOutputChannelMetadata, ...],
    *,
    child: bool,
) -> TrajectorySample:
    offset = 100.0 if child else 0.0
    available = {
        "position.north_m": offset + 100.0 * time_s,
        "velocity.speed_m_s": 50.0 + 2.0 * time_s,
        "attitude.heading_deg": (45.0 + 30.0 * time_s) % 360.0,
        "mode.index": 0 if time_s < 1.0 else 1,
        "attitude.quaternion": [math.cos(0.1 * time_s), 0.0, 0.0, math.sin(0.1 * time_s)],
        "health.valid": True,
        "status.mode": "hold" if time_s < 1.0 else "maneuver",
        "diagnostics.payload": {"child": child, "time_s": time_s},
        "event.marker": 1 if abs(time_s - 1.0) < 1.0e-12 else 0,
    }
    return TrajectorySample(time_s=time_s, values={item.id: available[item.id] for item in channels})
    ####


def _probe_error(
    code: str,
    message: str,
    request: MissionCompositionRunRequest,
    *,
    path: str,
) -> MissionCompositionExecutionError:
    return MissionCompositionExecutionError(
        MissionCompositionDiagnostic(
            severity="error",
            code=code,
            message=message,
            phase="preflight",
            recoverability="correctable",
            path=path,
            hint="Refresh the provider advertisement and submit a supported value.",
            provider_id=request.provider_id,
            model_id=request.model_id,
        ),
        category="unsupported",
    )
    ####


def _number_parameter(
    parameter_id: str,
    label: str,
    description: str,
    *,
    quantity: str,
    unit: str | None,
    default: float,
    interval: ConfigurationInterval,
    control: Literal["number_input", "slider"],
    group: str,
    order: int,
    role: Literal["initialization", "segment", "constraint", "variant", "output"] = "initialization",
    qualified_interval: ConfigurationInterval | None = None,
    safe_extended_interval: ConfigurationInterval | None = None,
    periodicity: ConfigurationPeriodicity | None = None,
    transform: Literal["identity", "log", "logit", "categorical"] = "identity",
) -> ConfigurationParameterSchema:
    value_space = (
        ConfigurationValueSpace(
            topology="circle",
            representation="canonical_scalar_angle",
            error_rule="shortest_arc_difference",
            interpolation_rule="shortest_arc",
            normalization_rule="wrap_to_principal_interval",
            period=periodicity.period,
            equivalence="values separated by integer multiples of the period are equivalent",
        )
        if periodicity is not None
        else ConfigurationValueSpace(
            topology="bounded_interval" if interval.minimum is not None or interval.maximum is not None else "real_line",
            representation="scalar",
            error_rule="absolute_difference",
            interpolation_rule="linear",
        )
    )
    return ConfigurationParameterSchema(
        id=parameter_id,
        label=label,
        description=description,
        value_type="number",
        quantity=quantity,
        canonical_unit=unit,
        display_unit=unit,
        default=default,
        default_declared=True,
        interval=interval,
        qualified_interval=qualified_interval,
        safe_extended_interval=safe_extended_interval,
        periodicity=periodicity,
        role=role,
        compatible_fidelities=_FIDELITIES,
        transform=transform,
        projection_policy="reject_invalid",
        value_space=value_space,
        presentation=ValuePresentationMetadata(
            group=group,
            order=order,
            control=control,
            format=NumericPresentationMetadata(decimal_places=2, use_grouping=True),
        ),
        provenance="contract-probe grammar",
    )
    ####


def _segment_variant(segment_id: str, label: str, *, load_factor: bool) -> ConfigurationChoiceVariant:
    children: list[ConfigurationParameterSchema] = [
        _number_parameter(
            "duration_s",
            "Duration",
            "Duration of this segment occurrence.",
            quantity="time",
            unit="s",
            default=1.0,
            interval=_interval(0.01, 60.0),
            control="number_input",
            group="segment",
            order=10,
            role="segment",
        )
    ]
    if load_factor:
        children.append(
            _number_parameter(
                "maximum_load_factor_g",
                "Maximum Load Factor",
                "Per-segment maneuver bound.",
                quantity="load_factor",
                unit="g0",
                default=3.0,
                interval=_interval(0.0, 12.0),
                qualified_interval=_interval(0.0, 6.0),
                control="slider",
                group="constraints",
                order=20,
                role="constraint",
            )
        )
    return ConfigurationChoiceVariant(
        id=segment_id,
        label=label,
        description=f"Synthetic {label.casefold()} segment.",
        compatible_fidelities=_FIDELITIES,
        node=ConfigurationGroupSchema(
            id=f"{segment_id}_parameters",
            label=f"{label} Parameters",
            children=tuple(children),
        ),
    )
    ####


def _interval(
    minimum: float | None,
    maximum: float | None,
    *,
    minimum_inclusive: bool = True,
    maximum_inclusive: bool = True,
) -> ConfigurationInterval:
    return ConfigurationInterval(
        minimum=ConfigurationBound(value=minimum, inclusive=minimum_inclusive) if minimum is not None else None,
        maximum=ConfigurationBound(value=maximum, inclusive=maximum_inclusive) if maximum is not None else None,
    )
    ####


def _vector_space(length: Literal[3, 4]) -> ConfigurationValueSpace:
    component = ConfigurationValueSpace(
        topology="real_line",
        representation="scalar",
        error_rule="absolute_difference",
        interpolation_rule="linear",
    )
    return ConfigurationValueSpace(
        topology="euclidean_vector",
        representation=f"vector{length}",
        error_rule="euclidean_norm",
        interpolation_rule="componentwise_linear",
        components=tuple(component for _ in range(length)),
    )
    ####


__all__ = [
    "CONTRACT_PROBE_MODEL_ID",
    "CONTRACT_PROBE_PROVIDER_ID",
    "ContractProbeMissionCompositionProvider",
    "build_contract_probe_configuration",
    "contract_probe_configuration_schema",
    "contract_probe_model_metadata",
]
####
