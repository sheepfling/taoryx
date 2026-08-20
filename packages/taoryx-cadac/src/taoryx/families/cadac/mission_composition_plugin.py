"""Catalog-wide Taoryx Mission Composition discovery for CADAC actor plug-ins."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path

from taoryx.fidelity_contracts import FidelityTier
from taoryx.sensor_api import MeasurementPacket
from taoryx.trajectory.configuration_contract import (
    ConfigurationGroupSchema,
    ConfigurationGroupValue,
    ConfigurationParameterSchema,
    ConfigurationParameterValue,
    PreparedTrajectoryConfiguration,
    TrajectoryActuatorType,
    TrajectoryConfigurationInstance,
    TrajectoryConfigurationSchema,
    TrajectoryDynamicsFidelity,
    TrajectoryEntityOutputMetadata,
    TrajectoryFidelityMetadata,
    TrajectoryInputRealization,
    TrajectoryModelCapabilities,
    TrajectoryModelMetadata,
    TrajectoryModelPresentationMetadata,
    TrajectoryModelPropertyMetadata,
    TrajectoryOutputChannelMetadata,
    TrajectoryOutputSchema,
    TrajectoryProviderMetadata,
    TrajectoryProviderPresentationMetadata,
    TrajectoryRealizationMetadata,
    validate_configuration_instance,
)
from taoryx.trajectory.execution_contract import (
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
    MissionCompositionTrajectoryResult,
)
from taoryx.trajectory.mission_composition import (
    MissionCompositionClosedSession,
    MissionCompositionCloseSessionRequest,
    MissionCompositionInspectSessionRequest,
    MissionCompositionOpenSessionRequest,
    MissionCompositionResetSessionRequest,
    MissionCompositionSessionDescriptor,
    MissionCompositionSessionObservation,
    MissionCompositionSessionStepRequest,
    MissionCompositionSessionStepResult,
)

from .ads6_aircraft_mission_composition import (
    ADS6_AIRCRAFT_MODEL_ID,
    ADS6_AIRCRAFT_PHASE_ID,
    CadacAds6AircraftMissionCompositionProvider,
    build_default_ads6_aircraft_configuration,
)
from .ads6_aircraft_plugin import Ads6AircraftVehiclePlugin
from .ads6_engagement_mission_composition import (
    ADS6_ENGAGEMENT_MODEL_ID,
    ADS6_ENGAGEMENT_MODEL_VERSION,
    ADS6_ENGAGEMENT_PHASE_ID,
    CadacAds6EngagementMissionCompositionProvider,
    build_default_ads6_engagement_configuration,
)
from .ads6_engagement_plugin import Ads6EngagementPlugin
from .ads6_sam_mission_composition import (
    ADS6_SAM_FIN_PHASE_ID,
    ADS6_SAM_MODEL_ID,
    CadacAds6SamMissionCompositionProvider,
    build_default_ads6_sam_configuration,
)
from .ads6_sam_plugin import Ads6SamVehiclePlugin
from .ads6_srbm_mission_composition import (
    ADS6_SRBM_MODEL_ID,
    ADS6_SRBM_PHASE_ID,
    CadacAds6SrbmMissionCompositionProvider,
    build_default_ads6_srbm_configuration,
)
from .ads6_srbm_plugin import Ads6SrbmVehiclePlugin
from .agm6_mission_composition import (
    AGM6_MODEL_ID,
    AGM6_PHASE_ID,
    CadacAgm6MissionCompositionProvider,
    build_default_agm6_configuration,
)
from .agm6_plugin import Agm6VehiclePlugin
from .aim5_mission_composition import (
    AIM5_MODEL_ID,
    CADAC_PROVIDER_ID,
    CadacAim5MissionCompositionProvider,
    build_default_aim5_configuration,
)
from .aim5_plugin import Aim5VehiclePlugin
from .control_metadata import blocked_control_advertisement
from .cruise5_mission_composition import (
    CRUISE5_MODEL_ID,
    CadacCruise5MissionCompositionProvider,
    build_default_cruise5_configuration,
)
from .cruise5_plugin import Cruise5VehiclePlugin
from .falcon6_mission_composition import (
    FALCON6_MODEL_ID,
    CadacFalcon6MissionCompositionProvider,
    build_default_falcon6_configuration,
)
from .falcon6_plugin import Falcon6VehiclePlugin
from .ghame3_mission_composition import (
    GHAME3_MODEL_ID,
    CadacGhame3MissionCompositionProvider,
    build_default_ghame3_configuration,
)
from .ghame3_plugin import Ghame3VehiclePlugin
from .ghame6_mission_composition import (
    GHAME6_MODEL_ID,
    CadacGhame6MissionCompositionProvider,
    build_default_ghame6_configuration,
)
from .ghame6_plugin import Ghame6VehiclePlugin
from .integration_contract import CadacModelIntegrationContract, build_cadac_model_integration_contract
from .magsix_mission_composition import (
    MAGSIX_MODEL_ID,
    CadacMagsixMissionCompositionProvider,
    build_default_magsix_configuration,
)
from .magsix_plugin import MagsixVehiclePlugin
from .manifest import (
    CadacControlRealization,
    CadacEffectorKind,
    CadacPhaseFidelity,
)
from .output_metadata import validate_cadac_model_io_contract, validate_cadac_output_schema
from .plugin import CADAC_PLUGIN_CATALOG, CadacVehiclePluginDescriptor
from .rocket6g_mission_composition import (
    ROCKET6G_MODEL_ID,
    CadacRocket6gMissionCompositionProvider,
    build_default_rocket6g_configuration,
)
from .rocket6g_plugin import Rocket6gVehiclePlugin
from .sensor_integration import CadacSensorIntegrationContract
from .sraam6_mission_composition import (
    SRAAM6_FIN_PHASE_ID,
    SRAAM6_MODEL_ID,
    CadacSraam6MissionCompositionProvider,
    build_default_sraam6_configuration,
)
from .sraam6_plugin import Sraam6VehiclePlugin

CADAC_CATALOG_PROVIDER_VERSION = ADS6_ENGAGEMENT_MODEL_VERSION
_PLANNED_REALIZATION_PREFIX = "cadac-source-phase"
_CADAC_PACKAGE_MODEL_IDS = frozenset((ADS6_ENGAGEMENT_MODEL_ID,))
_CADAC_SELECTABLE_MODEL_IDS = (
    frozenset(descriptor.model_id for descriptor in CADAC_PLUGIN_CATALOG.plugins if descriptor.trajectory_capable) | _CADAC_PACKAGE_MODEL_IDS
)


class CadacMissionCompositionProvider:
    """Discovery surface for all or a selected set of CADAC actor/package models.

    A source-bound host may select one exact model to avoid constructing
    unrelated configuration and output schemas. The default retains the full
    metadata catalog for source-authoring tools. A host-facing provider can
    instead set ``publish_unbound_models=False`` so only exact installed
    runtimes enter a trajectory list. Package models that own multiple source
    actors, such as ADS6 engagement, are selectable even though they do not
    correspond to a single actor manifest descriptor.
    """

    def __init__(
        self,
        *,
        ads6_aircraft_plugin: Ads6AircraftVehiclePlugin | None = None,
        ads6_engagement_plugin: Ads6EngagementPlugin | None = None,
        ads6_sam_plugin: Ads6SamVehiclePlugin | None = None,
        ads6_srbm_plugin: Ads6SrbmVehiclePlugin | None = None,
        agm6_plugin: Agm6VehiclePlugin | None = None,
        aim5_plugin: Aim5VehiclePlugin | None = None,
        cruise5_plugin: Cruise5VehiclePlugin | None = None,
        falcon6_plugin: Falcon6VehiclePlugin | None = None,
        ghame3_plugin: Ghame3VehiclePlugin | None = None,
        ghame6_plugin: Ghame6VehiclePlugin | None = None,
        magsix_plugin: MagsixVehiclePlugin | None = None,
        rocket6g_plugin: Rocket6gVehiclePlugin | None = None,
        sraam6_plugin: Sraam6VehiclePlugin | None = None,
        selected_model_ids: tuple[str, ...] | None = None,
        publish_unbound_models: bool = True,
    ) -> None:
        selected = _resolve_selected_model_ids(selected_model_ids)
        bound_model_ids = frozenset(
            model_id
            for model_id, plugin in (
                (ADS6_AIRCRAFT_MODEL_ID, ads6_aircraft_plugin),
                (ADS6_ENGAGEMENT_MODEL_ID, ads6_engagement_plugin),
                (ADS6_SAM_MODEL_ID, ads6_sam_plugin),
                (ADS6_SRBM_MODEL_ID, ads6_srbm_plugin),
                (AGM6_MODEL_ID, agm6_plugin),
                (AIM5_MODEL_ID, aim5_plugin),
                (CRUISE5_MODEL_ID, cruise5_plugin),
                (FALCON6_MODEL_ID, falcon6_plugin),
                (GHAME3_MODEL_ID, ghame3_plugin),
                (GHAME6_MODEL_ID, ghame6_plugin),
                (MAGSIX_MODEL_ID, magsix_plugin),
                (ROCKET6G_MODEL_ID, rocket6g_plugin),
                (SRAAM6_MODEL_ID, sraam6_plugin),
            )
            if plugin is not None
        )
        if selected is not None:
            unselected_bindings = tuple(sorted(bound_model_ids - selected))
            if unselected_bindings:
                raise ValueError("CADAC selected model scope omits explicit source bindings: " + ", ".join(unselected_bindings))
        ####

        def is_selected(model_id: str) -> bool:
            return selected is None or model_id in selected
            ####

        self._ads6_aircraft = (
            CadacAds6AircraftMissionCompositionProvider(ads6_aircraft_plugin)
            if ads6_aircraft_plugin is not None and is_selected(ADS6_AIRCRAFT_MODEL_ID)
            else None
        )
        self._ads6_engagement = (
            CadacAds6EngagementMissionCompositionProvider(ads6_engagement_plugin)
            if ads6_engagement_plugin is not None and is_selected(ADS6_ENGAGEMENT_MODEL_ID)
            else None
        )
        self._ads6_sam = CadacAds6SamMissionCompositionProvider(ads6_sam_plugin) if ads6_sam_plugin is not None and is_selected(ADS6_SAM_MODEL_ID) else None
        self._ads6_srbm = (
            CadacAds6SrbmMissionCompositionProvider(ads6_srbm_plugin) if ads6_srbm_plugin is not None and is_selected(ADS6_SRBM_MODEL_ID) else None
        )
        self._agm6 = CadacAgm6MissionCompositionProvider(agm6_plugin) if agm6_plugin is not None and is_selected(AGM6_MODEL_ID) else None
        self._aim5 = CadacAim5MissionCompositionProvider(aim5_plugin) if aim5_plugin is not None and is_selected(AIM5_MODEL_ID) else None
        self._cruise5 = CadacCruise5MissionCompositionProvider(cruise5_plugin) if cruise5_plugin is not None and is_selected(CRUISE5_MODEL_ID) else None
        self._falcon6 = CadacFalcon6MissionCompositionProvider(falcon6_plugin) if falcon6_plugin is not None and is_selected(FALCON6_MODEL_ID) else None
        self._ghame3 = CadacGhame3MissionCompositionProvider(ghame3_plugin) if ghame3_plugin is not None and is_selected(GHAME3_MODEL_ID) else None
        self._ghame6 = CadacGhame6MissionCompositionProvider(ghame6_plugin) if ghame6_plugin is not None and is_selected(GHAME6_MODEL_ID) else None
        self._magsix = CadacMagsixMissionCompositionProvider(magsix_plugin) if magsix_plugin is not None and is_selected(MAGSIX_MODEL_ID) else None
        self._rocket6g = CadacRocket6gMissionCompositionProvider(rocket6g_plugin) if rocket6g_plugin is not None and is_selected(ROCKET6G_MODEL_ID) else None
        self._sraam6 = CadacSraam6MissionCompositionProvider(sraam6_plugin) if sraam6_plugin is not None and is_selected(SRAAM6_MODEL_ID) else None
        self._batch_executors: dict[
            str,
            Callable[[MissionCompositionRunRequest], MissionCompositionTrajectoryResult],
        ] = {
            model_id: executor
            for model_id, executor in (
                (ADS6_AIRCRAFT_MODEL_ID, self._ads6_aircraft.execute_batch if self._ads6_aircraft is not None else None),
                (ADS6_ENGAGEMENT_MODEL_ID, self._ads6_engagement.execute_batch if self._ads6_engagement is not None else None),
                (ADS6_SAM_MODEL_ID, self._ads6_sam.execute_batch if self._ads6_sam is not None else None),
                (ADS6_SRBM_MODEL_ID, self._ads6_srbm.execute_batch if self._ads6_srbm is not None else None),
                (AGM6_MODEL_ID, self._agm6.execute_batch if self._agm6 is not None else None),
                (AIM5_MODEL_ID, self._aim5.execute_batch if self._aim5 is not None else None),
                (CRUISE5_MODEL_ID, self._cruise5.execute_batch if self._cruise5 is not None else None),
                (FALCON6_MODEL_ID, self._falcon6.execute_batch if self._falcon6 is not None else None),
                (GHAME3_MODEL_ID, self._ghame3.execute_batch if self._ghame3 is not None else None),
                (GHAME6_MODEL_ID, self._ghame6.execute_batch if self._ghame6 is not None else None),
                (MAGSIX_MODEL_ID, self._magsix.execute_batch if self._magsix is not None else None),
                (ROCKET6G_MODEL_ID, self._rocket6g.execute_batch if self._rocket6g is not None else None),
                (SRAAM6_MODEL_ID, self._sraam6.execute_batch if self._sraam6 is not None else None),
            )
            if executor is not None
        }
        self._schemas: dict[str, TrajectoryConfigurationSchema] = {}
        self._output_schemas: dict[str, TrajectoryOutputSchema] = {}
        self._models: dict[str, TrajectoryModelMetadata] = {}
        self._integration_contracts: dict[str, CadacModelIntegrationContract] = {}
        for descriptor in CADAC_PLUGIN_CATALOG.plugins:
            if not descriptor.trajectory_capable or not is_selected(descriptor.model_id):
                continue
            ####
            if descriptor.model_id == ADS6_AIRCRAFT_MODEL_ID and self._ads6_aircraft is not None:
                model = self._ads6_aircraft.list_models()[0]
                schema = self._ads6_aircraft.get_model_schema(ADS6_AIRCRAFT_MODEL_ID)
                output = self._ads6_aircraft.get_model_output_schema(ADS6_AIRCRAFT_MODEL_ID)
            elif descriptor.model_id == ADS6_SAM_MODEL_ID and self._ads6_sam is not None:
                model = self._ads6_sam.list_models()[0]
                schema = self._ads6_sam.get_model_schema(ADS6_SAM_MODEL_ID)
                output = self._ads6_sam.get_model_output_schema(ADS6_SAM_MODEL_ID)
            elif descriptor.model_id == ADS6_SRBM_MODEL_ID and self._ads6_srbm is not None:
                model = self._ads6_srbm.list_models()[0]
                schema = self._ads6_srbm.get_model_schema(ADS6_SRBM_MODEL_ID)
                output = self._ads6_srbm.get_model_output_schema(ADS6_SRBM_MODEL_ID)
            elif descriptor.model_id == AGM6_MODEL_ID and self._agm6 is not None:
                model = self._agm6.list_models()[0]
                schema = self._agm6.get_model_schema(AGM6_MODEL_ID)
                output = self._agm6.get_model_output_schema(AGM6_MODEL_ID)
            elif descriptor.model_id == AIM5_MODEL_ID and self._aim5 is not None:
                model = self._aim5.list_models()[0]
                schema = self._aim5.get_model_schema(AIM5_MODEL_ID)
                output = self._aim5.get_model_output_schema(AIM5_MODEL_ID)
            elif descriptor.model_id == CRUISE5_MODEL_ID and self._cruise5 is not None:
                model = self._cruise5.list_models()[0]
                schema = self._cruise5.get_model_schema(CRUISE5_MODEL_ID)
                output = self._cruise5.get_model_output_schema(CRUISE5_MODEL_ID)
            elif descriptor.model_id == FALCON6_MODEL_ID and self._falcon6 is not None:
                model = self._falcon6.list_models()[0]
                schema = self._falcon6.get_model_schema(FALCON6_MODEL_ID)
                output = self._falcon6.get_model_output_schema(FALCON6_MODEL_ID)
            elif descriptor.model_id == GHAME3_MODEL_ID and self._ghame3 is not None:
                model = self._ghame3.list_models()[0]
                schema = self._ghame3.get_model_schema(GHAME3_MODEL_ID)
                output = self._ghame3.get_model_output_schema(GHAME3_MODEL_ID)
            elif descriptor.model_id == GHAME6_MODEL_ID and self._ghame6 is not None:
                model = self._ghame6.list_models()[0]
                schema = self._ghame6.get_model_schema(GHAME6_MODEL_ID)
                output = self._ghame6.get_model_output_schema(GHAME6_MODEL_ID)
            elif descriptor.model_id == MAGSIX_MODEL_ID and self._magsix is not None:
                model = self._magsix.list_models()[0]
                schema = self._magsix.get_model_schema(MAGSIX_MODEL_ID)
                output = self._magsix.get_model_output_schema(MAGSIX_MODEL_ID)
            elif descriptor.model_id == ROCKET6G_MODEL_ID and self._rocket6g is not None:
                model = self._rocket6g.list_models()[0]
                schema = self._rocket6g.get_model_schema(ROCKET6G_MODEL_ID)
                output = self._rocket6g.get_model_output_schema(ROCKET6G_MODEL_ID)
            elif descriptor.model_id == SRAAM6_MODEL_ID and self._sraam6 is not None:
                model = self._sraam6.list_models()[0]
                schema = self._sraam6.get_model_schema(SRAAM6_MODEL_ID)
                output = self._sraam6.get_model_output_schema(SRAAM6_MODEL_ID)
            else:
                if not publish_unbound_models:
                    continue
                schema = _build_planned_schema(descriptor)
                output = _build_planned_output_schema(descriptor)
                model = _build_planned_model_metadata(descriptor, schema, output)
            ####
            validate_cadac_output_schema(output)
            validate_cadac_model_io_contract(model)
            self._schemas[descriptor.model_id] = schema
            self._output_schemas[descriptor.model_id] = output
            self._models[descriptor.model_id] = model
            self._integration_contracts[descriptor.model_id] = build_cadac_model_integration_contract(model)
        ####
        if self._ads6_engagement is not None and is_selected(ADS6_ENGAGEMENT_MODEL_ID):
            engagement_model = self._ads6_engagement.list_models()[0]
            engagement_schema = self._ads6_engagement.get_model_schema(ADS6_ENGAGEMENT_MODEL_ID)
            engagement_output = self._ads6_engagement.get_model_output_schema(ADS6_ENGAGEMENT_MODEL_ID)
            validate_cadac_output_schema(engagement_output)
            validate_cadac_model_io_contract(engagement_model)
            self._models[ADS6_ENGAGEMENT_MODEL_ID] = engagement_model
            self._schemas[ADS6_ENGAGEMENT_MODEL_ID] = engagement_schema
            self._output_schemas[ADS6_ENGAGEMENT_MODEL_ID] = engagement_output
            self._integration_contracts[ADS6_ENGAGEMENT_MODEL_ID] = build_cadac_model_integration_contract(engagement_model)
        ####
        self._metadata = TrajectoryProviderMetadata(
            id=CADAC_PROVIDER_ID,
            name="CADAC vehicle plug-ins",
            version=CADAC_CATALOG_PROVIDER_VERSION,
            description="CADAC actor plug-ins and installed source-ordered package compositions projected into Taoryx.",
            presentation=TrajectoryProviderPresentationMetadata(
                display_name="CADAC Vehicle Plug-ins",
                short_name="CADAC",
                summary="Phase-aware CADAC actors plus exact installed multi-root package compositions.",
                organization="Taoryx integration layer",
                categories=("aerospace", "vehicle-plugins", "reference-models"),
            ),
            status="development" if publish_unbound_models or self._models else "catalog_only",
            tags=("cadac", "plugin-catalog", "phase-aware-fidelity")
            if publish_unbound_models
            else ("cadac", "runtime-only", "source-bindings-required"),
            execution_contract="exact provider/model common-runner registration; no family fallback",
            model_count=len(self._models),
            provenance="missiondesignsolutions/CADAC actor classification and Taoryx plug-in projection",
            claim_boundary=(
                "All dynamic source actors are discoverable. Only exact installed runtime bindings advertise batch execution; "
                "planned actors remain non-executable and cannot fall back to neighboring models. Installed package "
                "compositions are additional exact models rather than replacements for their actor plug-ins."
                if publish_unbound_models
                else "This host-facing provider publishes only exact installed runtime bindings. Source actor catalog "
                "metadata remains available through CadacPluginCatalog, and no unbound actor is exposed as a trajectory model."
            ),
        )

    ####

    @property
    def metadata(self) -> TrajectoryProviderMetadata:
        """Return provider-wide CADAC catalog identity."""

        return self._metadata

    ####

    def list_models(self) -> tuple[TrajectoryModelMetadata, ...]:
        """Return every trajectory-capable CADAC actor in deterministic model-ID order."""

        return tuple(self._models[key] for key in sorted(self._models))

    ####

    def get_model_schema(self, model_id: str) -> TrajectoryConfigurationSchema:
        """Return one exact actor plug-in configuration schema."""

        try:
            return self._schemas[model_id]
        except KeyError as error:
            raise KeyError(f"unknown CADAC trajectory plug-in {model_id!r}") from error
        ####

    ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        """Return one exact actor plug-in output schema."""

        try:
            return self._output_schemas[model_id]
        except KeyError as error:
            raise KeyError(f"unknown CADAC trajectory plug-in {model_id!r}") from error
        ####

    ####

    def get_model_integration_contract(self, model_id: str) -> CadacModelIntegrationContract:
        """Return strict step, environment, controller, and sensor readiness."""

        try:
            return self._integration_contracts[model_id]
        except KeyError as error:
            raise KeyError(f"unknown CADAC trajectory plug-in {model_id!r}") from error
        ####

    ####

    def list_model_integration_contracts(self) -> tuple[CadacModelIntegrationContract, ...]:
        """Return every model integration contract in deterministic model order."""

        return tuple(self._integration_contracts[key] for key in sorted(self._integration_contracts))
        ####

    def get_model_sensor_integration_contract(self, model_id: str) -> CadacSensorIntegrationContract:
        """Return the native-sensor boundary advertised by one CADAC model."""

        return self.get_model_integration_contract(model_id).sensor_integration
        ####

    def list_model_sensor_integration_contracts(self) -> tuple[CadacSensorIntegrationContract, ...]:
        """Return CADAC sensor boundaries in deterministic model order."""

        return tuple(item.sensor_integration for item in self.list_model_integration_contracts())
        ####

    def validate_configuration(
        self,
        configuration: TrajectoryConfigurationInstance,
    ) -> PreparedTrajectoryConfiguration:
        """Validate source phase/fidelity selection without implying runtime availability."""

        if configuration.model_id == ADS6_ENGAGEMENT_MODEL_ID and self._ads6_engagement is not None:
            return self._ads6_engagement.validate_configuration(configuration)
        ####
        if configuration.model_id == ADS6_AIRCRAFT_MODEL_ID and self._ads6_aircraft is not None:
            return self._ads6_aircraft.validate_configuration(configuration)
        ####
        if configuration.model_id == ADS6_SAM_MODEL_ID and self._ads6_sam is not None:
            return self._ads6_sam.validate_configuration(configuration)
        ####
        if configuration.model_id == ADS6_SRBM_MODEL_ID and self._ads6_srbm is not None:
            return self._ads6_srbm.validate_configuration(configuration)
        ####
        if configuration.model_id == AGM6_MODEL_ID and self._agm6 is not None:
            return self._agm6.validate_configuration(configuration)
        ####
        if configuration.model_id == AIM5_MODEL_ID and self._aim5 is not None:
            return self._aim5.validate_configuration(configuration)
        ####
        if configuration.model_id == CRUISE5_MODEL_ID and self._cruise5 is not None:
            return self._cruise5.validate_configuration(configuration)
        ####
        if configuration.model_id == FALCON6_MODEL_ID and self._falcon6 is not None:
            return self._falcon6.validate_configuration(configuration)
        ####
        if configuration.model_id == GHAME3_MODEL_ID and self._ghame3 is not None:
            return self._ghame3.validate_configuration(configuration)
        ####
        if configuration.model_id == GHAME6_MODEL_ID and self._ghame6 is not None:
            return self._ghame6.validate_configuration(configuration)
        ####
        if configuration.model_id == MAGSIX_MODEL_ID and self._magsix is not None:
            return self._magsix.validate_configuration(configuration)
        ####
        if configuration.model_id == ROCKET6G_MODEL_ID and self._rocket6g is not None:
            return self._rocket6g.validate_configuration(configuration)
        ####
        if configuration.model_id == SRAAM6_MODEL_ID and self._sraam6 is not None:
            return self._sraam6.validate_configuration(configuration)
        ####
        descriptor = CADAC_PLUGIN_CATALOG.plugin(configuration.model_id)
        if not descriptor.trajectory_capable:
            raise ValueError(f"CADAC actor {configuration.model_id!r} is not a trajectory vehicle")
        ####
        schema = self.get_model_schema(configuration.model_id)
        prepared = validate_configuration_instance(schema, configuration)
        if not isinstance(prepared.resolved, Mapping):
            raise ValueError("CADAC planned plug-in resolved configuration must be a mapping")
        ####
        phase_id = prepared.resolved.get("source_phase")
        if not isinstance(phase_id, str):
            raise ValueError("CADAC planned plug-in configuration must resolve source_phase")
        ####
        phase = _phase(descriptor, phase_id)
        expected_realization = _phase_realization_id(phase_id)
        if configuration.realization_id not in {None, expected_realization}:
            raise ValueError(f"CADAC phase {phase_id!r} requires realization {expected_realization!r}, not {configuration.realization_id!r}")
        ####
        if phase.taoryx_tier != configuration.fidelity:
            raise ValueError(f"CADAC phase {phase_id!r} is classified as {phase.taoryx_tier!r}, not requested fidelity {configuration.fidelity!r}")
        ####
        return prepared

    ####

    def execute_batch(self, request: MissionCompositionRunRequest) -> MissionCompositionTrajectoryResult:
        """Dispatch one bound actor through the catalog's exact provider revision.

        Each actor bridge has its own implementation revision, while callers of
        this catalog selected the catalog provider revision.  The wrapper
        preserves the selected provider identity in the common result instead
        of leaking an inner bridge version through the catalog boundary.
        """

        if request.provider_id != self.metadata.id or request.provider_version != self.metadata.version:
            raise ValueError("CADAC batch request names another catalog provider version")
        ####
        try:
            executor = self._batch_executors[request.model_id]
        except KeyError as error:
            raise ValueError(f"CADAC model {request.model_id!r} has no installed exact batch binding") from error
        ####
        result = executor(request)
        return result.model_copy(update={"provider_version": self.metadata.version})
        ####

    ####

    def open_session(self, request: MissionCompositionOpenSessionRequest) -> MissionCompositionSessionDescriptor:
        """Open an exact installed persistent CADAC model session.

        This provider-native path is reserved for installed live CADAC episode
        bindings. A common host uses :class:`MissionCompositionSessionManager`
        for the universal ``step`` contract; it can derive an explicitly
        read-only replay session from any exact registered CADAC batch result.
        """

        if request.model_id == AIM5_MODEL_ID and self._aim5 is not None:
            if request.provider_id != self.metadata.id or request.provider_version != self.metadata.version:
                raise ValueError("CADAC session request names another catalog provider version")
            return self._aim5.open_session(request, advertised_provider_version=self.metadata.version)
        if request.model_id == ADS6_SRBM_MODEL_ID and self._ads6_srbm is not None:
            if request.provider_id != self.metadata.id or request.provider_version != self.metadata.version:
                raise ValueError("CADAC session request names another catalog provider version")
            return self._ads6_srbm.open_session(request, advertised_provider_version=self.metadata.version)
        if request.model_id == ADS6_ENGAGEMENT_MODEL_ID and self._ads6_engagement is not None:
            if request.provider_id != self.metadata.id or request.provider_version != self.metadata.version:
                raise ValueError("CADAC session request names another catalog provider version")
            return self._ads6_engagement.open_session(request, advertised_provider_version=self.metadata.version)
        if request.model_id == AGM6_MODEL_ID and self._agm6 is not None:
            if request.provider_id != self.metadata.id or request.provider_version != self.metadata.version:
                raise ValueError("CADAC session request names another catalog provider version")
            return self._agm6.open_session(request, advertised_provider_version=self.metadata.version)
        if request.model_id == SRAAM6_MODEL_ID and self._sraam6 is not None:
            if request.provider_id != self.metadata.id or request.provider_version != self.metadata.version:
                raise ValueError("CADAC session request names another catalog provider version")
            return self._sraam6.open_session(request, advertised_provider_version=self.metadata.version)
        raise ValueError(
            f"CADAC model {request.model_id!r} has no installed native persistent session binding; "
            "use MissionCompositionSessionManager for a registered core replay session"
        )
        ####

    create_session = open_session

    def build_runner(self) -> MissionCompositionRunnerRegistry:
        """Build the common runner for exactly the CADAC runtimes installed here.

        CADAC has always exposed :meth:`register_runnable_models`, but the
        Mission Composition plug-in contribution is also required to expose
        the provider-owned runner directly.  Building a fresh registry keeps
        the selected catalog's provider/version boundary intact while leaving
        planned or uninstalled CADAC actors unregistered.
        """

        registry = MissionCompositionRunnerRegistry()
        self.register_runnable_models(registry)
        return registry
        ####

    def inspect_session(
        self,
        request: MissionCompositionInspectSessionRequest | str,
    ) -> MissionCompositionSessionObservation:
        """Inspect a currently open installed CADAC session."""

        return self._require_session_owner(request.session_id if isinstance(request, MissionCompositionInspectSessionRequest) else request).inspect_session(
            request
        )
        ####

    def step_session(self, request: MissionCompositionSessionStepRequest) -> MissionCompositionSessionStepResult:
        """Advance a currently open installed CADAC session."""

        return self._require_session_owner(request.session_id).step_session(request)
        ####

    def reset_session(self, request: MissionCompositionResetSessionRequest) -> MissionCompositionSessionObservation:
        """Reset a currently open installed CADAC session."""

        return self._require_session_owner(request.session_id).reset_session(request)
        ####

    def close_session(
        self,
        request: MissionCompositionCloseSessionRequest | str,
    ) -> MissionCompositionClosedSession:
        """Close a currently open installed CADAC session."""

        session_id = request.session_id if isinstance(request, MissionCompositionCloseSessionRequest) else request
        return self._require_session_owner(session_id).close_session(request)
        ####

    def session_sensor_packets(
        self,
        session_id: str,
        *,
        delivered: bool = True,
    ) -> tuple[MeasurementPacket[object], ...]:
        """Return the full native SensorBus packet history for an installed session."""

        return self._require_session_owner(session_id).session_sensor_packets(session_id, delivered=delivered)
        ####

    def _require_session_owner(
        self,
        session_id: str,
    ) -> (
        CadacAim5MissionCompositionProvider
        | CadacAds6SrbmMissionCompositionProvider
        | CadacAds6EngagementMissionCompositionProvider
        | CadacAgm6MissionCompositionProvider
        | CadacSraam6MissionCompositionProvider
    ):
        for provider in (self._aim5, self._ads6_srbm, self._ads6_engagement, self._agm6, self._sraam6):
            if provider is not None and provider.has_session(session_id):
                return provider
        raise ValueError(f"CADAC session {session_id!r} is not owned by an installed persistent session provider")
        ####

    def has_installed_ads6_engagement_runtime(self) -> bool:
        """Return whether one source-ordered ADS6 package composition is installed."""

        return self._ads6_engagement is not None

    ####

    def has_installed_ads6_aircraft_runtime(self) -> bool:
        """Return whether the ADS6 AIRCRAFT3 point-mass plug-in is installed."""

        return self._ads6_aircraft is not None

    ####

    def has_installed_ads6_sam_runtime(self) -> bool:
        """Return whether the standalone ADS6 SAM multi-realization plug-in is installed."""

        return self._ads6_sam is not None

    ####

    def has_installed_ads6_srbm_runtime(self) -> bool:
        """Return whether the ADS6 ROCKET5 pseudo-6DoF plug-in is installed."""

        return self._ads6_srbm is not None

    ####

    def has_installed_agm6_runtime(self) -> bool:
        """Return whether the AGM6 three-actor standard-fin plug-in is installed."""

        return self._agm6 is not None

    ####

    def has_installed_aim5_runtime(self) -> bool:
        """Return whether the AIM5 source-compatible plug-in is installed."""

        return self._aim5 is not None

    ####

    def has_installed_cruise5_runtime(self) -> bool:
        """Return whether the CRUISE5 source-compatible plug-in is installed."""

        return self._cruise5 is not None

    ####

    def has_installed_falcon6_runtime(self) -> bool:
        """Return whether the FALCON6 physical-surface plant plug-in is installed."""

        return self._falcon6 is not None

    ####

    def has_installed_ghame3_runtime(self) -> bool:
        """Return whether the GHAME3 point-mass source-compatible plug-in is installed."""

        return self._ghame3 is not None

    ####

    def has_installed_ghame6_runtime(self) -> bool:
        """Return whether the GHAME6 phase-aware multi-actor plug-in is installed."""

        return self._ghame6 is not None

    ####

    def has_installed_magsix_runtime(self) -> bool:
        """Return whether the MAGSIX trajectory-only source-compatible plug-in is installed."""

        return self._magsix is not None

    ####

    def has_installed_rocket6g_runtime(self) -> bool:
        """Return whether the ROCKET6G phase-aware rigid-body plug-in is installed."""

        return self._rocket6g is not None

    ####

    def has_installed_sraam6_runtime(self) -> bool:
        """Return whether the SRAAM6 standard-fin engagement plug-in is installed."""

        return self._sraam6 is not None

    ####

    def register_runnable_models(self, registry: MissionCompositionRunnerRegistry) -> None:
        """Register only exact bound actors through this catalog wrapper.

        The registry receives one catalog-level executor so its result retains
        the version that callers selected.  It never substitutes an unbound
        actor or routes through an inner provider's independent revision.
        """

        for model_id in sorted(self._batch_executors):
            registry.register(self.metadata.id, model_id, self.execute_batch)
        ####

    ####


####


def _resolve_selected_model_ids(selected_model_ids: tuple[str, ...] | None) -> frozenset[str] | None:
    """Validate an optional exact CADAC provider-model scope before building schemas."""

    if selected_model_ids is None:
        return None
    if not selected_model_ids:
        raise ValueError("CADAC selected model scope must contain at least one model ID")
    if len(selected_model_ids) != len(set(selected_model_ids)):
        raise ValueError("CADAC selected model scope contains duplicate model IDs")
    if any(not model_id.strip() for model_id in selected_model_ids):
        raise ValueError("CADAC selected model scope contains an empty model ID")
    selected = frozenset(selected_model_ids)
    unknown = tuple(sorted(selected - _CADAC_SELECTABLE_MODEL_IDS))
    if unknown:
        raise ValueError("CADAC selected model scope contains unknown model IDs: " + ", ".join(unknown))
    return selected
    ####


@dataclass(frozen=True, slots=True)
class CadacSourceCaseBindings:
    """Explicit local source-case bindings for the CADAC Mission Composition catalog.

    Entry-point discovery deliberately does not search a workstation for an
    upstream CADAC checkout.  Callers that own a checkout can use
    :meth:`from_standard_checkout` for the upstream layout, or provide a
    partial set of paths directly.  Unbound catalog models remain discoverable
    but validate-only, preserving the no-fallback execution boundary.
    """

    ads6_aircraft_case_path: Path | None = None
    ads6_engagement_case_path: Path | None = None
    ads6_sam_case_path: Path | None = None
    ads6_sam_missile_actor_index: int | None = None
    ads6_srbm_case_path: Path | None = None
    agm6_case_path: Path | None = None
    aim5_case_path: Path | None = None
    cruise5_case_path: Path | None = None
    falcon6_case_path: Path | None = None
    ghame3_case_path: Path | None = None
    ghame6_case_path: Path | None = None
    magsix_case_path: Path | None = None
    rocket6g_case_path: Path | None = None
    sraam6_case_path: Path | None = None

    @classmethod
    def from_standard_checkout(cls, root: str | Path) -> CadacSourceCaseBindings:
        """Bind every runnable catalog model to the standard upstream checkout layout.

        The checkout is inspected only at this explicit call site.  Source
        definitions are still parsed and validated by their exact vehicle
        plug-ins while building the provider.
        """

        root_path = Path(root).expanduser().resolve()
        paths: dict[str, Path] = {
            "ads6_aircraft_case_path": root_path / "ADS6/input_AC_Straight and level.asc",
            "ads6_engagement_case_path": root_path / "ADS6/input_SAM_RF_AC_Radar_#3.asc",
            "ads6_sam_case_path": root_path / "ADS6/input_SAM_RF_AC_Radar_#3.asc",
            "ads6_srbm_case_path": root_path / "ADS6/input_SRBM_Ballistic.asc",
            "agm6_case_path": root_path / "AGM6/input.asc",
            "aim5_case_path": root_path / "AIM5/input.asc",
            "cruise5_case_path": root_path / "CRUISE5/input.asc",
            "falcon6_case_path": root_path / "FALCON6/input.asc",
            "ghame3_case_path": root_path / "GHAME3/Inputs/input.asc",
            "ghame6_case_path": root_path / "GHAME6/input.asc",
            "magsix_case_path": root_path / "MAGSIX/input.asc",
            "rocket6g_case_path": root_path / "ROCKET6G/input.asc",
            "sraam6_case_path": root_path / "SRAAM6/input.asc",
        }
        missing = [path.relative_to(root_path) for path in paths.values() if not path.is_file()]
        if missing:
            expected = ", ".join(str(path) for path in missing)
            raise FileNotFoundError(f"CADAC standard checkout is missing required source cases: {expected}")
        ####
        return cls(**paths, ads6_sam_missile_actor_index=0)

    ####

    def build_provider(
        self,
        *,
        selected_model_ids: tuple[str, ...] | None = None,
    ) -> CadacMissionCompositionProvider:
        """Build a full or selected provider with exactly the bound source cases.

        A selected scope must include every explicitly bound source case. This
        prevents a source case from being parsed or registered as a hidden
        fallback outside the caller's requested model boundary.
        """

        selected = _resolve_selected_model_ids(selected_model_ids)
        bound_model_ids = frozenset(
            model_id
            for model_id, source_case_path in (
                (ADS6_AIRCRAFT_MODEL_ID, self.ads6_aircraft_case_path),
                (ADS6_ENGAGEMENT_MODEL_ID, self.ads6_engagement_case_path),
                (ADS6_SAM_MODEL_ID, self.ads6_sam_case_path),
                (ADS6_SRBM_MODEL_ID, self.ads6_srbm_case_path),
                (AGM6_MODEL_ID, self.agm6_case_path),
                (AIM5_MODEL_ID, self.aim5_case_path),
                (CRUISE5_MODEL_ID, self.cruise5_case_path),
                (FALCON6_MODEL_ID, self.falcon6_case_path),
                (GHAME3_MODEL_ID, self.ghame3_case_path),
                (GHAME6_MODEL_ID, self.ghame6_case_path),
                (MAGSIX_MODEL_ID, self.magsix_case_path),
                (ROCKET6G_MODEL_ID, self.rocket6g_case_path),
                (SRAAM6_MODEL_ID, self.sraam6_case_path),
            )
            if source_case_path is not None
        )
        if selected is not None:
            unselected_bindings = tuple(sorted(bound_model_ids - selected))
            if unselected_bindings:
                raise ValueError("CADAC selected model scope omits explicit source bindings: " + ", ".join(unselected_bindings))
            ####
        ####

        return CadacMissionCompositionProvider(
            ads6_aircraft_plugin=(Ads6AircraftVehiclePlugin(self.ads6_aircraft_case_path) if self.ads6_aircraft_case_path is not None else None),
            ads6_engagement_plugin=(Ads6EngagementPlugin(self.ads6_engagement_case_path) if self.ads6_engagement_case_path is not None else None),
            ads6_sam_plugin=(
                Ads6SamVehiclePlugin(
                    self.ads6_sam_case_path,
                    missile_actor_index=self.ads6_sam_missile_actor_index,
                )
                if self.ads6_sam_case_path is not None
                else None
            ),
            ads6_srbm_plugin=(Ads6SrbmVehiclePlugin(self.ads6_srbm_case_path) if self.ads6_srbm_case_path is not None else None),
            agm6_plugin=Agm6VehiclePlugin(self.agm6_case_path) if self.agm6_case_path is not None else None,
            aim5_plugin=Aim5VehiclePlugin(self.aim5_case_path) if self.aim5_case_path is not None else None,
            cruise5_plugin=(Cruise5VehiclePlugin(self.cruise5_case_path) if self.cruise5_case_path is not None else None),
            falcon6_plugin=(Falcon6VehiclePlugin(self.falcon6_case_path) if self.falcon6_case_path is not None else None),
            ghame3_plugin=(Ghame3VehiclePlugin(self.ghame3_case_path) if self.ghame3_case_path is not None else None),
            ghame6_plugin=(Ghame6VehiclePlugin(self.ghame6_case_path) if self.ghame6_case_path is not None else None),
            magsix_plugin=(MagsixVehiclePlugin(self.magsix_case_path) if self.magsix_case_path is not None else None),
            rocket6g_plugin=(Rocket6gVehiclePlugin(self.rocket6g_case_path) if self.rocket6g_case_path is not None else None),
            sraam6_plugin=(Sraam6VehiclePlugin(self.sraam6_case_path) if self.sraam6_case_path is not None else None),
            selected_model_ids=selected_model_ids,
        )

    ####


####


def build_default_cadac_configuration(
    provider: CadacMissionCompositionProvider,
    model_id: str,
    *,
    phase_id: str | None = None,
    configuration_id: str = "cadac-default",
) -> TrajectoryConfigurationInstance:
    """Create a default validation configuration for one discovered planned/embedded actor."""

    if model_id == ADS6_ENGAGEMENT_MODEL_ID and provider.has_installed_ads6_engagement_runtime():
        if phase_id not in {None, ADS6_ENGAGEMENT_PHASE_ID}:
            raise ValueError(f"installed ADS6 package runtime supports only source phase {ADS6_ENGAGEMENT_PHASE_ID!r}")
        ####
        if provider._ads6_engagement is None:
            raise RuntimeError("installed ADS6 package provider disappeared")
        ####
        return build_default_ads6_engagement_configuration(
            provider._ads6_engagement,
            configuration_id=configuration_id,
        )
    ####
    if model_id == ADS6_AIRCRAFT_MODEL_ID and provider.has_installed_ads6_aircraft_runtime():
        if phase_id not in {None, ADS6_AIRCRAFT_PHASE_ID}:
            raise ValueError(f"installed ADS6 AIRCRAFT3 runtime supports only source phase {ADS6_AIRCRAFT_PHASE_ID!r}")
        ####
        if provider._ads6_aircraft is None:
            raise RuntimeError("installed ADS6 AIRCRAFT3 provider disappeared")
        ####
        return build_default_ads6_aircraft_configuration(
            provider._ads6_aircraft,
            configuration_id=configuration_id,
        )
    ####
    if model_id == ADS6_SAM_MODEL_ID and provider.has_installed_ads6_sam_runtime():
        if provider._ads6_sam is None:
            raise RuntimeError("installed ADS6 SAM provider disappeared")
        ####
        return build_default_ads6_sam_configuration(
            provider._ads6_sam,
            configuration_id=configuration_id,
            phase_id=phase_id or ADS6_SAM_FIN_PHASE_ID,
        )
    ####
    if model_id == ADS6_SRBM_MODEL_ID and provider.has_installed_ads6_srbm_runtime():
        if phase_id not in {None, ADS6_SRBM_PHASE_ID}:
            raise ValueError(f"installed ADS6 SRBM runtime supports only source phase {ADS6_SRBM_PHASE_ID!r}")
        ####
        if provider._ads6_srbm is None:
            raise RuntimeError("installed ADS6 SRBM provider disappeared")
        ####
        return build_default_ads6_srbm_configuration(
            provider._ads6_srbm,
            configuration_id=configuration_id,
        )
    ####
    if model_id == AGM6_MODEL_ID and provider.has_installed_agm6_runtime():
        if phase_id not in {None, AGM6_PHASE_ID}:
            raise ValueError(f"installed AGM6 runtime supports only source phase {AGM6_PHASE_ID!r}")
        ####
        if provider._agm6 is None:
            raise RuntimeError("installed AGM6 provider disappeared")
        ####
        return build_default_agm6_configuration(
            provider._agm6,
            configuration_id=configuration_id,
        )
    ####
    if model_id == AIM5_MODEL_ID and provider.has_installed_aim5_runtime():
        if phase_id is not None:
            raise ValueError("installed AIM5 runtime does not select source phases independently")
        if provider._aim5 is None:
            raise RuntimeError("installed AIM5 provider disappeared")
        return build_default_aim5_configuration(provider._aim5, configuration_id=configuration_id)
    ####
    if model_id == CRUISE5_MODEL_ID and provider.has_installed_cruise5_runtime():
        if provider._cruise5 is None:
            raise RuntimeError("installed CRUISE5 provider disappeared")
        ####
        return build_default_cruise5_configuration(
            provider._cruise5,
            configuration_id=configuration_id,
            phase_id=phase_id or "source_model",
        )
    ####
    if model_id == GHAME3_MODEL_ID and provider.has_installed_ghame3_runtime():
        if phase_id not in {None, "source_model"}:
            raise ValueError("installed GHAME3 point-mass runtime supports only source phase 'source_model'")
        ####
        if provider._ghame3 is None:
            raise RuntimeError("installed GHAME3 provider disappeared")
        ####
        return build_default_ghame3_configuration(provider._ghame3, configuration_id=configuration_id)
    ####
    if model_id == GHAME6_MODEL_ID and provider.has_installed_ghame6_runtime():
        if phase_id not in {None, "atmospheric_surfaces"}:
            raise ValueError(
                "installed GHAME6 runtime executes the complete source phase program; "
                "individual phases remain runtime-reported rather than separately selectable"
            )
        ####
        if provider._ghame6 is None:
            raise RuntimeError("installed GHAME6 provider disappeared")
        ####
        return build_default_ghame6_configuration(provider._ghame6, configuration_id=configuration_id)
    ####
    if model_id == MAGSIX_MODEL_ID and provider.has_installed_magsix_runtime():
        if provider._magsix is None:
            raise RuntimeError("installed MAGSIX provider disappeared")
        ####
        return build_default_magsix_configuration(
            provider._magsix,
            configuration_id=configuration_id,
            phase_id=phase_id or "trajectory_only",
        )
    ####
    if model_id == FALCON6_MODEL_ID and provider.has_installed_falcon6_runtime():
        if phase_id not in {None, "surface_control"}:
            raise ValueError("installed FALCON6 direct plant supports only source phase 'surface_control'")
        ####
        if provider._falcon6 is None:
            raise RuntimeError("installed FALCON6 provider disappeared")
        ####
        return build_default_falcon6_configuration(provider._falcon6, configuration_id=configuration_id)
    ####
    if model_id == ROCKET6G_MODEL_ID and provider.has_installed_rocket6g_runtime():
        if phase_id not in {None, "mixed_tvc_rcs"}:
            raise ValueError(
                "installed ROCKET6G runtime executes the complete source phase program; "
                "individual phases remain runtime-reported rather than separately selectable"
            )
        ####
        if provider._rocket6g is None:
            raise RuntimeError("installed ROCKET6G provider disappeared")
        ####
        return build_default_rocket6g_configuration(provider._rocket6g, configuration_id=configuration_id)
    ####
    if model_id == SRAAM6_MODEL_ID and provider.has_installed_sraam6_runtime():
        if provider._sraam6 is None:
            raise RuntimeError("installed SRAAM6 provider disappeared")
        ####
        return build_default_sraam6_configuration(
            provider._sraam6,
            configuration_id=configuration_id,
            phase_id=phase_id or SRAAM6_FIN_PHASE_ID,
        )
    ####
    descriptor = CADAC_PLUGIN_CATALOG.plugin(model_id)
    selected_phase = phase_id or descriptor.default_phase_id
    phase = _phase(descriptor, selected_phase)
    if phase.taoryx_tier is None:
        raise ValueError(f"CADAC actor phase {model_id}/{selected_phase} is static")
    ####
    schema = provider.get_model_schema(model_id)
    return TrajectoryConfigurationInstance(
        configuration_id=configuration_id,
        model_id=model_id,
        model_version=CADAC_CATALOG_PROVIDER_VERSION,
        schema_fingerprint=schema.fingerprint,
        fidelity=phase.taoryx_tier,
        realization_id=_phase_realization_id(selected_phase),
        root=ConfigurationGroupValue(
            values={
                "source_phase": ConfigurationParameterValue(value=selected_phase),
            }
        ),
    )


####


def _build_planned_schema(descriptor: CadacVehiclePluginDescriptor) -> TrajectoryConfigurationSchema:
    fidelities = _dynamic_tiers(descriptor)
    return TrajectoryConfigurationSchema(
        model_id=descriptor.model_id,
        model_version=CADAC_CATALOG_PROVIDER_VERSION,
        supported_fidelities=fidelities,
        root=ConfigurationGroupSchema(
            id="cadac_actor",
            label=descriptor.display_name,
            description="Source phase selection for a CADAC actor plug-in whose executable lowering is not yet installed.",
            children=(
                ConfigurationParameterSchema(
                    id="source_phase",
                    label="Source phase",
                    description="Explicit CADAC actor/phase fidelity realization.",
                    value_type="enum",
                    required=False,
                    default=descriptor.default_phase_id,
                    default_declared=True,
                    choices=tuple(phase.phase_id for phase in descriptor.phases),
                    role="variant",
                    compatible_fidelities=fidelities,
                    provenance=f"{descriptor.source_repository}/{descriptor.source_path}",
                ),
            ),
        ),
        claim_boundary="Validation selects an already-classified source phase; it does not make a planned actor executable.",
    )


####


def _build_planned_output_schema(descriptor: CadacVehiclePluginDescriptor) -> TrajectoryOutputSchema:
    fidelities = _dynamic_tiers(descriptor)
    realization_ids = tuple(_phase_realization_id(phase.phase_id) for phase in descriptor.phases)
    core = (
        TrajectoryOutputChannelMetadata(
            id="position",
            label="Position",
            description="Planned canonical position truth channel; source frame binding is established during executable lowering.",
            quantity="length",
            canonical_unit="m",
            display_unit="m",
            shape=(3,),
            availability="guaranteed",
            compatible_fidelities=fidelities,
            compatible_realizations=realization_ids,
            operations=(),
            provenance=f"planned interface for {descriptor.plugin_id}",
            claim_boundary="Interface reservation only; no output exists until an exact runtime is installed.",
        ),
        TrajectoryOutputChannelMetadata(
            id="velocity",
            label="Velocity",
            description="Planned canonical velocity truth channel; source frame binding is established during executable lowering.",
            quantity="velocity",
            canonical_unit="m/s",
            display_unit="m/s",
            shape=(3,),
            availability="guaranteed",
            compatible_fidelities=fidelities,
            compatible_realizations=realization_ids,
            operations=(),
            provenance=f"planned interface for {descriptor.plugin_id}",
            claim_boundary="Interface reservation only; no output exists until an exact runtime is installed.",
        ),
    )
    return TrajectoryOutputSchema(
        model_id=descriptor.model_id,
        model_version=CADAC_CATALOG_PROVIDER_VERSION,
        core_channels=core,
        telemetry_channels=(),
        telemetry_groups=(),
        entity_output=TrajectoryEntityOutputMetadata(),
        claim_boundary="Planned core-state interface only; source coordinate/frame semantics must be bound before execution promotion.",
    )


####


def _build_planned_model_metadata(
    descriptor: CadacVehiclePluginDescriptor,
    schema: TrajectoryConfigurationSchema,
    output_schema: TrajectoryOutputSchema,
) -> TrajectoryModelMetadata:
    fidelities = tuple(_fidelity_metadata(descriptor, tier) for tier in schema.supported_fidelities)
    realizations = tuple(_realization_metadata(descriptor, phase) for phase in descriptor.phases)
    status = "embedded" if descriptor.status == "embedded" else "planned"
    return TrajectoryModelMetadata(
        id=descriptor.model_id,
        name=descriptor.display_name,
        version=CADAC_CATALOG_PROVIDER_VERSION,
        description=f"CADAC {descriptor.source_model} actor plug-in projected from {descriptor.package_id.value}.",
        presentation=TrajectoryModelPresentationMetadata(
            display_name=descriptor.display_name,
            short_name=descriptor.source_model,
            summary="Phase-aware CADAC actor plug-in; executable lowering is not independently installed.",
            category="target" if descriptor.scope.value == "target" else "vehicle",
            subcategory=descriptor.package_id.value.casefold(),
            sort_key=descriptor.model_id,
            badges=("CADAC", status),
            default_fidelity_id=_phase(descriptor, descriptor.default_phase_id).taoryx_tier,
            default_output_channel_ids=("position", "velocity"),
            properties=(
                TrajectoryModelPropertyMetadata(
                    id="source_model",
                    label="Source model",
                    description="Upstream CADAC actor/model identifier.",
                    semantic_role="identity",
                    value_type="string",
                    value_kind="declared",
                    value=descriptor.source_model,
                    value_declared=True,
                    source_refs=(f"{descriptor.source_repository}/{descriptor.source_path}",),
                    provenance="CADAC actor manifest",
                    claim_boundary="Identity/classification metadata only.",
                ),
            ),
        ),
        family_id=descriptor.family_id,
        physical_family=f"cadac_{descriptor.package_id.value.casefold()}",
        model_kind="vehicle_plugin",
        status=status,
        tags=("cadac", descriptor.package_id.value.casefold(), status),
        operations=("discover", "validate"),
        common_runner_operations=(),
        capabilities=TrajectoryModelCapabilities(
            initialization_modes=("source_phase_selection",),
            segment_types=("source_defined",),
            termination_modes=("source_defined",),
            operations=("discover", "validate"),
            supports_custom_segments=False,
            supports_deployment=False,
            supports_staging=descriptor.package_id.value in {"ROCKET6G", "GHAME6"},
            supports_dynamic_child_generation=False,
            supports_multiple_stages=descriptor.package_id.value == "ROCKET6G",
            supports_submodels=descriptor.status == "embedded",
        ),
        realizations=realizations,
        mission_templates=(),
        deployments=(),
        reference_frames=(),
        output_schema=output_schema,
        output_schema_id=output_schema.schema_id,
        output_schema_fingerprint=output_schema.fingerprint,
        configuration_schema_id=schema.schema_id,
        configuration_schema_fingerprint=schema.fingerprint,
        fidelities=fidelities,
        fidelity_transitions=(),
        source_refs=(f"{descriptor.source_repository}/{descriptor.source_path}",),
        provenance="CADAC source classification projected into Taoryx Mission Composition discovery",
        claim_boundary=descriptor.claim_boundary,
    )


####


def _fidelity_metadata(
    descriptor: CadacVehiclePluginDescriptor,
    tier: FidelityTier,
) -> TrajectoryFidelityMetadata:
    matching = tuple(phase for phase in descriptor.phases if phase.taoryx_tier == tier)
    runtime_fidelity = _runtime_fidelity(tier)
    control_realization = matching[0].control_realization.value if len({item.control_realization for item in matching}) == 1 else "phase_defined"
    return TrajectoryFidelityMetadata(
        id=tier,
        label=f"{descriptor.source_model} {tier}",
        rank=_tier_rank(tier),
        declared=True,
        dynamics_fidelity=runtime_fidelity,
        input_realization="provider_defined",
        runtime_fidelity=runtime_fidelity,
        control_realization=control_realization,
        promotion_status="planned" if descriptor.status == "planned" else "development",
        operations=("validate",),
        blockers=descriptor.blockers or ("standalone executable runtime is not registered",),
        required_operations=tuple(dict.fromkeys(module for phase in matching for module in phase.source_modules)),
        claim_boundary="Source-level fidelity classification only; no execution claim is implied.",
    )


####


def _realization_metadata(
    descriptor: CadacVehiclePluginDescriptor,
    phase: CadacPhaseFidelity,
) -> TrajectoryRealizationMetadata:
    if phase.taoryx_tier is None:
        raise ValueError("static CADAC phases cannot become trajectory realizations")
    ####
    input_realization = _input_realization(phase)
    actuator_types = _actuator_types(phase) if input_realization == "actuator_allocated" else ("not_applicable",)
    blockers = descriptor.blockers or ("actor is available only as an embedded submodel of another CADAC runtime",)
    return TrajectoryRealizationMetadata(
        id=_phase_realization_id(phase.phase_id),
        label=phase.phase_id.replace("_", " ").title(),
        description=phase.description,
        status="blocked",
        dynamics_fidelities=(_runtime_fidelity(phase.taoryx_tier),),
        input_realization=input_realization,
        actuator_types=actuator_types,
        controls=blocked_control_advertisement(
            claim_boundary="This CADAC phase is classified but has no installed source-bound control runtime.",
        ),
        fidelity_aliases=(phase.taoryx_tier,),
        operations=("validate",),
        blockers=blockers,
        source_refs=(f"{descriptor.source_repository}/{descriptor.source_path}",),
        claim_boundary="Phase realization is classified and selectable for validation but has no standalone executor yet.",
    )


####


def _input_realization(phase: CadacPhaseFidelity) -> TrajectoryInputRealization:
    if phase.control_realization is CadacControlRealization.DIRECT_WRENCH:
        return "direct_wrench"
    ####
    if phase.control_realization in {CadacControlRealization.EFFECTOR_ALLOCATED, CadacControlRealization.MIXED_EFFECTOR}:
        return "actuator_allocated"
    ####
    if phase.control_realization is CadacControlRealization.UNCONTROLLED:
        return "uncontrolled"
    ####
    return "provider_defined"


####


def _actuator_types(phase: CadacPhaseFidelity) -> tuple[TrajectoryActuatorType, ...]:
    kinds = {kind for kind in phase.effectors if kind is not CadacEffectorKind.PROPULSION_THROTTLE}
    if len(kinds) > 1:
        return ("mixed",)
    ####
    if CadacEffectorKind.AERODYNAMIC_SURFACE in kinds:
        return ("aerodynamic_surfaces",)
    ####
    if CadacEffectorKind.TVC_GIMBAL in kinds:
        return ("thrust_vectoring",)
    ####
    if CadacEffectorKind.RCS_THRUSTER in kinds:
        return ("rcs",)
    ####
    return ("provider_defined",)


####


def _phase_realization_id(phase_id: str) -> str:
    return f"{_PLANNED_REALIZATION_PREFIX}.{phase_id}"


####


def _phase(descriptor: CadacVehiclePluginDescriptor, phase_id: str) -> CadacPhaseFidelity:
    for phase in descriptor.phases:
        if phase.phase_id == phase_id:
            return phase
        ####
    ####
    raise ValueError(f"CADAC actor {descriptor.plugin_id!r} has no source phase {phase_id!r}")


####


def _dynamic_tiers(descriptor: CadacVehiclePluginDescriptor) -> tuple[FidelityTier, ...]:
    tiers: list[FidelityTier] = []
    for phase in descriptor.phases:
        if phase.taoryx_tier is not None and phase.taoryx_tier not in tiers:
            tiers.append(phase.taoryx_tier)
        ####
    ####
    return tuple(tiers)


####


def _runtime_fidelity(tier: FidelityTier) -> TrajectoryDynamicsFidelity:
    if tier == "point_mass_3dof":
        return "point_mass_3dof"
    ####
    if tier == "pseudo_6dof":
        return "pseudo_6dof"
    ####
    return "rigid_body_6dof"


####


def _tier_rank(tier: FidelityTier) -> int:
    ranks = {
        "point_mass_3dof": 0,
        "pseudo_6dof": 1,
        "rigid_body_6dof_direct_wrench": 2,
        "rigid_body_6dof_surface_allocated": 3,
    }
    return ranks[tier]


####


__all__ = [
    "CADAC_CATALOG_PROVIDER_VERSION",
    "CadacMissionCompositionProvider",
    "CadacSourceCaseBindings",
    "build_default_cadac_configuration",
]
