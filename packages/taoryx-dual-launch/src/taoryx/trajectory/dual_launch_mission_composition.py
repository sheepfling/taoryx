"""Portable, fail-closed advertisement for the Alpha 2 dual-launch glider."""

from __future__ import annotations

from functools import lru_cache
from typing import Any, Literal

from taoryx_dual_launch.resources import model_resource_root

from .catalog import load_family_catalog
from .configuration_contract import (
    ConfigurationBound,
    ConfigurationChoiceSchema,
    ConfigurationChoiceValue,
    ConfigurationChoiceVariant,
    ConfigurationContractError,
    ConfigurationGroupSchema,
    ConfigurationGroupValue,
    ConfigurationInterval,
    ConfigurationParameterSchema,
    ConfigurationPeriodicity,
    ConfigurationSequenceSchema,
    ConfigurationSequenceTemplate,
    ConfigurationSequenceValue,
    ConfigurationValueSpace,
    PreparedTrajectoryConfiguration,
    TrajectoryConfigurationInstance,
    TrajectoryConfigurationSchema,
    TrajectoryControlAdvertisement,
    TrajectoryControlAuthorityMetadata,
    TrajectoryControlChannelMetadata,
    TrajectoryControlIntentMetadata,
    TrajectoryControlNativeBindingMetadata,
    TrajectoryDeploymentMetadata,
    TrajectoryDynamicsFidelity,
    TrajectoryEntityOutputMetadata,
    TrajectoryFidelityMetadata,
    TrajectoryInputRealization,
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
    ValuePresentationMetadata,
)
from .contracts import CaseIntent, CaseValue, FamilyCatalog, ResolvedCase
from .resolution import resolve_case

DUAL_LAUNCH_MODEL_ID = "dual_launch_glider"
DUAL_LAUNCH_MODEL_VERSION = "1.0.0-mission-composition-v1"
_FIDELITIES = (
    "point_mass_3dof",
    "pseudo_6dof",
    "rigid_body_6dof_direct_wrench",
)
_POINT_MASS: Literal["point_mass_3dof"] = "point_mass_3dof"
_REALIZATION_ID = "generated_native_problem"
_FAMILY_CATALOG = model_resource_root() / "verification" / "dual_launch_family_catalog.yaml"


def dual_launch_configuration_schema() -> TrajectoryConfigurationSchema:
    """Advertise both launch forms and their shared post-release mission."""

    launch_mode = ConfigurationChoiceSchema(
        id="launch_mode",
        label="Launch Mode",
        description="Select air release or an attached-booster launch; the post-release glider mission is shared.",
        variants=(
            ConfigurationChoiceVariant(
                id="air_release",
                label="Air Release",
                description="Initialize directly at the common glider release boundary.",
                compatible_fidelities=_FIDELITIES,
                node=ConfigurationGroupSchema(
                    id="air_release_state",
                    label="Air-Release State",
                    children=(
                        _number("release_altitude_m", "Release Altitude", "m", 3200.0, 0.0),
                        _number("release_speed_m_s", "Release Speed", "m/s", 155.0, 0.0),
                        _angle("initial_heading_deg", "Initial Heading", 90.0),
                        _number("flight_path_angle_deg", "Flight-Path Angle", "deg", 5.0, -89.0, 89.0),
                    ),
                ),
            ),
            ConfigurationChoiceVariant(
                id="attached_booster",
                label="Attached Booster",
                description="Begin with the booster attached and separate at the configured accepted boundary.",
                compatible_fidelities=_FIDELITIES,
                node=ConfigurationGroupSchema(
                    id="attached_booster_state",
                    label="Attached-Booster State",
                    children=(
                        _number("initial_altitude_m", "Initial Altitude", "m", 3000.0, 0.0),
                        _number("initial_speed_m_s", "Initial Speed", "m/s", 120.0, 0.0),
                        _number("boost_duration_s", "Boost Duration", "s", 2.0, 0.0001),
                        _number("booster_thrust_n", "Booster Thrust", "N", 15000.0, 0.0),
                        _number("booster_mass_flow_kg_s", "Booster Mass Flow", "kg/s", 50.0, 0.0),
                    ),
                ),
            ),
        ),
    )
    vehicle = ConfigurationGroupSchema(
        id="vehicle",
        label="Vehicle",
        description="Shared glider and attached-stack surrogate parameters.",
        children=(
            _number("stack_mass_kg", "Attached Stack Mass", "kg", 1000.0, 1.0),
            _number("glider_mass_kg", "Released Glider Mass", "kg", 900.0, 1.0),
            _number("drag_coefficient", "Drag Coefficient", "dimensionless", 0.02, 0.0),
            _number("lift_to_drag", "Lift-to-Drag Ratio", "dimensionless", 4.0, 0.0),
        ),
    )
    endpoint = ConfigurationGroupSchema(
        id="endpoint",
        label="Waypoint Endpoint",
        description="Shared target used by both launch forms.",
        children=(
            _number("target_range_m", "Target Range", "m", 2000.0, 0.0),
            _angle("target_bearing_deg", "Target Bearing", 90.0),
            _number("target_altitude_m", "Target Altitude", "m", 3200.0, 0.0),
            _number("target_speed_m_s", "Target Speed", "m/s", 120.0, 0.0),
        ),
    )
    segment = ConfigurationChoiceSchema(
        id="segment",
        label="Segment",
        description="One ordered dual-launch mission segment.",
        variants=tuple(
            ConfigurationChoiceVariant(
                id=identifier,
                label=label,
                description=description,
                compatible_fidelities=_FIDELITIES,
                node=ConfigurationGroupSchema(id=f"{identifier}_parameters", label=f"{label} Parameters", children=()),
            )
            for identifier, label, description in (
                ("attached_booster", "Attached Booster", "Powered attached-stack segment ending at separation."),
                ("release_glide", "Release Glide", "Shared unpowered post-release glider segment."),
                ("terminal_guidance", "Terminal Guidance", "Shared waypoint-capture guidance segment."),
            )
        ),
    )
    segments = ConfigurationSequenceSchema(
        id="segments",
        label="Mission Segments",
        description="Choose one of the two exact launch-form templates.",
        item=segment,
        minimum_items=2,
        maximum_items=3,
        templates=(
            ConfigurationSequenceTemplate(
                id="air_release_waypoint",
                label="Air Release to Waypoint",
                description="Release glide followed by terminal waypoint guidance.",
                item_variants=("release_glide", "terminal_guidance"),
                compatible_fidelities=_FIDELITIES,
            ),
            ConfigurationSequenceTemplate(
                id="attached_booster_waypoint",
                label="Attached Booster to Waypoint",
                description="Attached boost, separation, release glide, and terminal guidance.",
                item_variants=("attached_booster", "release_glide", "terminal_guidance"),
                compatible_fidelities=_FIDELITIES,
            ),
        ),
        allow_custom=False,
    )
    return TrajectoryConfigurationSchema(
        model_id=DUAL_LAUNCH_MODEL_ID,
        model_version=DUAL_LAUNCH_MODEL_VERSION,
        supported_fidelities=_FIDELITIES,
        root=ConfigurationGroupSchema(
            id="mission",
            label="Dual-Launch Glider Mission",
            description="Launch form, shared vehicle, endpoint, and ordered mission segments.",
            children=(launch_mode, vehicle, endpoint, segments),
        ),
        claim_boundary=(
            "The exact selected launch form is available through the source-generated point-mass common batch route. "
            "It does not advertise a stateful executor, higher-fidelity dynamics, or independent post-separation children."
        ),
    )
    ####


def build_dual_launch_example_configuration(
    launch_mode: Literal["air_release", "attached_booster"] = "air_release",
    schema: TrajectoryConfigurationSchema | None = None,
) -> TrajectoryConfigurationInstance:
    """Return the checked portable witness for one exact point-mass launch form."""

    resolved_schema = schema or dual_launch_configuration_schema()
    mission_id, segments = {
        "air_release": ("air_release_waypoint", ("release_glide", "terminal_guidance")),
        "attached_booster": (
            "attached_booster_waypoint",
            ("attached_booster", "release_glide", "terminal_guidance"),
        ),
    }[launch_mode]
    empty = ConfigurationGroupValue(values={})
    return TrajectoryConfigurationInstance(
        configuration_id=f"dual-launch-{launch_mode}-common-witness",
        model_id=DUAL_LAUNCH_MODEL_ID,
        model_version=resolved_schema.model_version,
        schema_fingerprint=resolved_schema.fingerprint,
        fidelity=_POINT_MASS,
        realization_id=_REALIZATION_ID,
        mission_template_id=mission_id,
        root=ConfigurationGroupValue(
            values={
                "launch_mode": ConfigurationChoiceValue(selected=launch_mode, value=empty),
                "vehicle": empty,
                "endpoint": empty,
                "segments": ConfigurationSequenceValue(items=tuple(ConfigurationChoiceValue(selected=item, value=empty) for item in segments)),
            }
        ),
    )
    ####


def build_dual_launch_prepared_case(prepared: PreparedTrajectoryConfiguration) -> ResolvedCase:
    """Lower one validated point-mass request into the immutable Alpha 2 case authority.

    The native dual-launch generator already accepts only ``ResolvedCase``.
    Keeping this lowering in the workflow package makes the normal provider
    validation result—not a parallel hand-authored fixture—the sole input to
    that generator.
    """

    configuration = prepared.configuration
    if configuration.model_id != DUAL_LAUNCH_MODEL_ID:
        raise ConfigurationContractError(
            "model-mismatch",
            f"expected model {DUAL_LAUNCH_MODEL_ID!r}",
            path="prepared.configuration.model_id",
        )
    if configuration.fidelity != _POINT_MASS:
        raise ConfigurationContractError(
            "execution-fidelity-unavailable",
            "Dual-launch native execution is registered only for point_mass_3dof.",
            path="prepared.configuration.fidelity",
        )
    if configuration.realization_id not in {None, _REALIZATION_ID}:
        raise ConfigurationContractError(
            "execution-realization-unavailable",
            f"expected realization {_REALIZATION_ID!r}",
            path="prepared.configuration.realization_id",
        )
    root = _resolved_mapping(prepared.resolved, "prepared configuration root")
    launch_mode, launch = _resolved_choice(root, "launch_mode")
    vehicle = _resolved_mapping(root.get("vehicle"), "prepared.resolved.vehicle")
    endpoint = _resolved_mapping(root.get("endpoint"), "prepared.resolved.endpoint")
    segment_ids = _resolved_segments(root)
    expected_mission, expected_segments = {
        "air_release": ("air_release_waypoint", ("release_glide", "terminal_guidance")),
        "attached_booster": (
            "attached_booster_waypoint",
            ("attached_booster", "release_glide", "terminal_guidance"),
        ),
    }.get(launch_mode, (None, ()))
    if expected_mission is None:
        raise ConfigurationContractError(
            "unknown-launch-mode",
            f"unsupported launch mode {launch_mode!r}",
            path="prepared.resolved.launch_mode.selected",
        )
    if configuration.mission_template_id != expected_mission or segment_ids != expected_segments:
        raise ConfigurationContractError(
            "launch-mission-mismatch",
            f"launch mode {launch_mode!r} requires {expected_mission!r} with {expected_segments!r}",
            path="prepared.configuration.mission_template_id",
        )
    values: dict[str, float] = {
        "vehicle.mass.initial": _prepared_number(vehicle, "stack_mass_kg"),
        "vehicle.mass.glider": _prepared_number(vehicle, "glider_mass_kg"),
        "vehicle.aero.drag_coefficient": _prepared_number(vehicle, "drag_coefficient"),
        "vehicle.aero.lift_to_drag": _prepared_number(vehicle, "lift_to_drag"),
        "mission.target_range": _prepared_number(endpoint, "target_range_m"),
        "mission.target_bearing": _prepared_number(endpoint, "target_bearing_deg"),
        "mission.target_altitude": _prepared_number(endpoint, "target_altitude_m"),
        "mission.target_speed": _prepared_number(endpoint, "target_speed_m_s"),
    }
    if launch_mode == "air_release":
        values.update(
            {
                "mission.release_altitude": _prepared_number(launch, "release_altitude_m"),
                "mission.release_speed": _prepared_number(launch, "release_speed_m_s"),
                "mission.initial_heading": _prepared_number(launch, "initial_heading_deg"),
                "mission.initial_flight_path_angle": _prepared_number(launch, "flight_path_angle_deg"),
            }
        )
    else:
        values.update(
            {
                "mission.initial_altitude": _prepared_number(launch, "initial_altitude_m"),
                "mission.initial_speed": _prepared_number(launch, "initial_speed_m_s"),
                "mission.boost_duration": _prepared_number(launch, "boost_duration_s"),
                "vehicle.booster.thrust": _prepared_number(launch, "booster_thrust_n"),
                "vehicle.booster.mass_flow": _prepared_number(launch, "booster_mass_flow_kg_s"),
            }
        )
    intent = CaseIntent(
        case_id=configuration.configuration_id,
        family=DUAL_LAUNCH_MODEL_ID,
        fidelity=_POINT_MASS,
        mission="waypoint_release",
        segment_plan="dual_launch",
        controller="waypoint-autopilot",
        overrides={item: CaseValue(value=value) for item, value in values.items()},
        extensions={
            "launch_mode": launch_mode,
            "mission_composition_mission_template_id": expected_mission,
            "mission_composition_fingerprint": prepared.fingerprint,
        },
    )
    return resolve_case(intent, _dual_launch_catalog())
    ####


@lru_cache(maxsize=1)
def _dual_launch_catalog() -> FamilyCatalog:
    return load_family_catalog(_FAMILY_CATALOG)
    ####


def _resolved_mapping(value: object, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ConfigurationContractError("invalid-prepared-configuration", f"{path} must be a mapping", path=path)
    return value
    ####


def _resolved_choice(root: dict[str, Any], key: str) -> tuple[str, dict[str, Any]]:
    value = _resolved_mapping(root.get(key), f"prepared.resolved.{key}")
    selected = value.get("selected")
    nested = value.get("value")
    if not isinstance(selected, str) or not isinstance(nested, dict):
        raise ConfigurationContractError(
            "invalid-prepared-configuration",
            f"prepared choice {key!r} is malformed",
            path=f"prepared.resolved.{key}",
        )
    return selected, nested
    ####


def _resolved_segments(root: dict[str, Any]) -> tuple[str, ...]:
    values = root.get("segments")
    if not isinstance(values, list):
        raise ConfigurationContractError(
            "invalid-prepared-configuration",
            "prepared segments must be a list",
            path="prepared.resolved.segments",
        )
    selected: list[str] = []
    for index, value in enumerate(values):
        if not isinstance(value, dict) or not isinstance(value.get("selected"), str):
            raise ConfigurationContractError(
                "invalid-prepared-configuration",
                "prepared segment selection is malformed",
                path=f"prepared.resolved.segments[{index}]",
            )
        selected.append(value["selected"])
    return tuple(selected)
    ####


def _prepared_number(values: dict[str, Any], key: str) -> float:
    value = values.get(key)
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ConfigurationContractError(
            "invalid-prepared-configuration",
            f"prepared value {key!r} must be numeric",
            path=f"prepared.resolved.{key}",
        )
    return float(value)
    ####


def dual_launch_model_metadata(schema: TrajectoryConfigurationSchema | None = None) -> TrajectoryModelMetadata:
    """Publish dual-launch discovery without overstating native executability."""

    resolved = schema or dual_launch_configuration_schema()
    fidelities = tuple(_fidelity(item, index) for index, item in enumerate(_FIDELITIES))
    operations = tuple(_mission_operation(_POINT_MASS, operation) for operation in ("validate", "batch", "step"))
    missions = tuple(
        TrajectoryMissionTemplateMetadata(
            id=identifier,
            name=label,
            description=description,
            status="point_mass_batch_ready"
            if _POINT_MASS in {item.fidelity for item in operations if item.operation == "batch" and item.status == "available"}
            else "configuration_ready_execution_blocked",
            initialization_variants=(launch_mode,),
            segment_sequence=segments,
            compatible_fidelities=(_POINT_MASS,),
            operations=operations,
            provenance="packages/taoryx-dual-launch/verification/dual_launch_family_catalog.yaml; src/taoryx/trajectory/dual_launch.py",
            claim_boundary=(
                "The point-mass batch path runs this exact launch form through the Alpha 2 native source generator. "
                "It does not create an independently propagated booster or released-glider entity."
            ),
        )
        for identifier, label, description, launch_mode, segments in (
            (
                "air_release_waypoint",
                "Air Release to Waypoint",
                "Air release into shared glide and terminal guidance.",
                "air_release",
                ("release_glide", "terminal_guidance"),
            ),
            (
                "attached_booster_waypoint",
                "Attached Booster to Waypoint",
                "Attached boost and separation into the shared glider mission.",
                "attached_booster",
                ("attached_booster", "release_glide", "terminal_guidance"),
            ),
        )
    )
    output_schema = _output_schema()
    deployment = TrajectoryDeploymentMetadata(
        id="booster_glider_separation",
        name="Booster/Glider Separation",
        description="Accepted-boundary handoff from attached stack to released glider.",
        status="declared",
        trigger_segment_ids=("attached_booster",),
        trigger_event_kinds=("separation",),
        child_role="released_glider",
        child_model_id=DUAL_LAUNCH_MODEL_ID,
        child_model_scope="provider_catalog",
        child_model_kind="glider",
        compatible_fidelities=(_POINT_MASS,),
        operations=(),
        state_initialization="inherited_at_accepted_boundary",
        fidelity_policy="inherit_parent",
        lifecycle="event_only",
        blockers=("independent_parent_and_child_trajectory_binding",),
        source_refs=("packages/taoryx-dual-launch/verification/dual_launch_family_catalog.yaml", "src/taoryx/trajectory/dual_launch.py"),
        claim_boundary="The point-mass batch result reports the source-native separation boundary, but not an independently propagated child trajectory.",
    )
    return TrajectoryModelMetadata(
        id=DUAL_LAUNCH_MODEL_ID,
        name="Dual-Launch Glider",
        version=resolved.model_version,
        description="One glider mission configurable for either air release or attached-booster launch.",
        presentation=TrajectoryModelPresentationMetadata(
            display_name="Dual-Launch Glider",
            short_name="Dual Launch",
            summary="Shared glider mission with air-release and attached-booster forms; both have a source-generated point-mass batch path.",
            category="Composition Proof Families",
            subcategory="Glider",
            sort_key="glider:dual-launch",
            badges=("Point-Mass Batch Ready", "Higher Fidelities Blocked", "Event-Only Separation"),
            default_fidelity_id="point_mass_3dof",
            default_mission_template_id="air_release_waypoint",
            properties=(
                TrajectoryModelPropertyMetadata(
                    id="launch_form_count",
                    label="Launch Forms",
                    description="Number of structurally distinct launch configurations.",
                    semantic_role="capability",
                    value_type="integer",
                    value_kind="exact",
                    value=2,
                    value_declared=True,
                    presentation=ValuePresentationMetadata(group="capabilities", order=10),
                    provenance="packages/taoryx-dual-launch/verification/dual_launch_family_catalog.yaml",
                    claim_boundary="Configuration topology only.",
                ),
            ),
        ),
        family_id=DUAL_LAUNCH_MODEL_ID,
        physical_family="glider",
        model_kind="composition_proof_family",
        status="common_runner_ready",
        tags=("glider", "dual-launch", "deployment"),
        execution_capability_profile="taoryx_universal",
        operations=("discover", "validate", "batch"),
        common_runner_operations=("batch",),
        capabilities=TrajectoryModelCapabilities(
            initialization_modes=("air_release", "attached_booster"),
            segment_types=("attached_booster", "release_glide", "terminal_guidance"),
            termination_modes=("separation", "time", "waypoint_capture"),
            operations=("discover", "validate", "batch"),
            supports_deployment=True,
            supports_staging=True,
            supports_submodels=True,
        ),
        realizations=(
            TrajectoryRealizationMetadata(
                id="generated_native_problem",
                label="Generated Native Problem",
                description="Alpha 2 resolved-case source generator shared by both launch forms.",
                status="available",
                dynamics_fidelities=("point_mass_3dof",),
                input_realization="guidance_command",
                controls=_dual_launch_control_advertisement(missions),
                fidelity_aliases=(_POINT_MASS,),
                mission_template_ids=tuple(item.id for item in missions),
                operations=("validate", "batch"),
                native_factory_ids=("dual_launch_generated_native_problem.v1",),
                blockers=(),
                source_refs=("src/taoryx/trajectory/dual_launch.py",),
                claim_boundary="Available only for the synthetic point-mass source generator; it is not pseudo-6DOF, rigid-body, or physical-glider qualification.",
            ),
        ),
        mission_templates=missions,
        deployments=(deployment,),
        reference_frames=(
            TrajectoryReferenceFrameMetadata(
                id="local_ned",
                name="Local North-East-Down",
                description="Local tangent frame used by the generated native launch problems.",
                frame_kind="local_tangent",
                axes=("north", "east", "down"),
                handedness="right",
                origin="launch datum",
                orientation="north-east-down",
                provenance="src/taoryx/trajectory/dual_launch.py",
            ),
        ),
        output_schema=output_schema,
        output_schema_id=output_schema.schema_id,
        output_schema_fingerprint=output_schema.fingerprint,
        configuration_schema_id=resolved.schema_id,
        configuration_schema_fingerprint=resolved.fingerprint,
        fidelities=fidelities,
        source_refs=("packages/taoryx-dual-launch/verification/dual_launch_family_catalog.yaml", "src/taoryx/trajectory/dual_launch.py"),
        provenance="Synthetic successor-side Alpha 2 dual-launch composition proof.",
        claim_boundary="The source-generated point-mass batch route is available for both launch forms. Higher-fidelity, interactive, and independently propagated child-trajectory claims remain blocked.",
    )
    ####


def _dual_launch_control_advertisement(
    missions: tuple[TrajectoryMissionTemplateMetadata, ...],
) -> TrajectoryControlAdvertisement:
    """Publish source-generated batch commands without implying caller control."""

    value_space = ConfigurationValueSpace(
        topology="euclidean",
        representation="scalar",
        error_rule="componentwise subtraction",
        interpolation_rule="linear",
        coordinate_chart="R^1",
    )
    channels = tuple(
        TrajectoryControlChannelMetadata(
            id=channel_id,
            label=label,
            description=description,
            channel_kind="action",
            quantity=quantity,
            canonical_unit=unit,
            display_unit=unit,
            interval=ConfigurationInterval(
                minimum=ConfigurationBound(value=minimum),
                maximum=ConfigurationBound(value=maximum),
            ),
            sampling_semantics="held_action",
            value_space=value_space,
            availability="available_in_batch",
            operations=("batch",),
            native_channel_id=channel_id,
            native_binding=TrajectoryControlNativeBindingMetadata(
                id=channel_id,
                quantity=quantity,
                canonical_unit=unit,
                interval=ConfigurationInterval(
                    minimum=ConfigurationBound(value=minimum),
                    maximum=ConfigurationBound(value=maximum),
                ),
                value_space=value_space,
                provider_binding={"generated_problem_variable": channel_id},
            ),
            provider_binding={"generated_problem_variable": channel_id},
            presentation=ValuePresentationMetadata(group="controls", order=index * 10, control="slider"),
            source_refs=("src/taoryx/trajectory/dual_launch.py",),
            provenance="Alpha 2 generated native problem declaration",
            claim_boundary="Generated native batch command only; it is not an externally supplied actuator or interactive action.",
        )
        for index, (channel_id, label, description, quantity, unit, minimum, maximum) in enumerate(
            (
                ("command.bank", "Bank Command", "Generated glider bank command.", "angle", "deg", -25.0, 25.0),
                (
                    "command.throttle",
                    "Throttle Command",
                    "Generated attached-booster throttle command.",
                    "dimensionless",
                    "dimensionless",
                    0.0,
                    1.0,
                ),
            ),
            start=1,
        )
    )
    mission_ids = tuple(item.id for item in missions)
    return TrajectoryControlAdvertisement(
        status="internally_generated",
        channels=channels,
        authorities=(
            TrajectoryControlAuthorityMetadata(
                id="generated_mission_commands",
                authority="mission",
                availability="available_in_batch",
                channel_ids=tuple(item.id for item in channels),
                operations=("batch",),
                description="Generated bank and throttle coordinates in both launch-form source problems.",
                command_owner="provider_controller",
                selection_scope="provider",
                switching_policy="provider_managed",
                scheme_id="provider.program",
                lowering_chain=("dual_launch_mission_generator", "generated_bank_throttle_profile"),
                source_refs=("src/taoryx/trajectory/dual_launch.py",),
                provenance="Alpha 2 source generator",
                claim_boundary="The generator owns these batch command profiles; callers cannot change them through a session action.",
            ),
        ),
        intents=(
            TrajectoryControlIntentMetadata(
                id="bank",
                label="Bank",
                description="Post-release glider bank intent.",
                resolution="provider_internal",
                segment_ids=("release_glide", "terminal_guidance"),
                mission_template_ids=mission_ids,
                channel_ids=("command.bank",),
                operations=("batch",),
                source_refs=("src/taoryx/trajectory/dual_launch.py",),
                provenance="generated source problem",
                claim_boundary="The generator resolves this intent internally for its point-mass batch source problem.",
            ),
            TrajectoryControlIntentMetadata(
                id="throttle",
                label="Throttle",
                description="Attached-booster propulsion intent.",
                resolution="provider_internal",
                segment_ids=("attached_booster",),
                mission_template_ids=tuple(item.id for item in missions if "attached_booster" in item.segment_sequence),
                channel_ids=("command.throttle",),
                operations=("batch",),
                source_refs=("src/taoryx/trajectory/dual_launch.py",),
                provenance="generated source problem",
                claim_boundary="The generator resolves this attached-booster intent internally in the point-mass batch source problem.",
            ),
        ),
        claim_boundary="Dual-launch controls are source-generated batch profiles, not caller-owned feedback or actuator controls.",
    )
    ####


def _mission_operation(
    fidelity: str,
    operation: Literal["validate", "batch", "step"],
) -> TrajectoryMissionOperationMetadata:
    """Describe the exact source-backed operation rather than a family-wide guess."""

    if operation == "validate":
        return TrajectoryMissionOperationMetadata(
            fidelity=fidelity,
            realization_id=_REALIZATION_ID,
            operation=operation,
            status="available",
            execution_mode="portable_schema_validation",
            claim_boundary="Structural and local-domain validation only.",
        )
    if fidelity == _POINT_MASS and operation == "batch":
        return TrajectoryMissionOperationMetadata(
            fidelity=fidelity,
            realization_id=_REALIZATION_ID,
            operation=operation,
            status="available",
            execution_mode="validated_configuration_to_resolved_case_to_native_point_mass_batch",
            availability_scope="provider_interface",
            common_runner_status="registered",
            executor_id="taoryx.dual-launch.generated-native-problem.batch.v1",
            claim_boundary="The exact selected launch form is generated and executed as a synthetic native point-mass batch problem.",
        )
    blockers = (
        ("no interactive dual-launch Mission Composition session binding is registered",)
        if operation == "step"
        else ("no native pseudo_or_rigid_body_dual_launch_executor",)
    )
    return TrajectoryMissionOperationMetadata(
        fidelity=fidelity,
        realization_id=_REALIZATION_ID,
        operation=operation,
        status="blocked",
        blockers=blockers,
        claim_boundary="This exact operation has no source-backed native binding.",
    )
    ####


def _fidelity(identifier: str, rank: int) -> TrajectoryFidelityMetadata:
    dynamics: TrajectoryDynamicsFidelity = (
        "point_mass_3dof" if identifier == "point_mass_3dof" else "pseudo_6dof" if identifier == "pseudo_6dof" else "rigid_body_6dof"
    )
    input_realization: TrajectoryInputRealization = "direct_wrench" if dynamics == "rigid_body_6dof" else "guidance_command"
    available = identifier == _POINT_MASS
    return TrajectoryFidelityMetadata(
        id=identifier,
        label=identifier.replace("_", " ").title(),
        rank=rank,
        declared=True,
        dynamics_fidelity=dynamics,
        input_realization=input_realization,
        compatibility_aliases=(identifier, "rigid_body_6dof") if dynamics == "rigid_body_6dof" else (identifier,),
        runtime_fidelity="generated_native_point_mass" if available else "not_available",
        control_realization="generated_batch_commands" if available else "not_available",
        promotion_status="synthetic_point_mass_batch" if available else "configuration_ready_execution_blocked",
        operations=("validate", "batch") if available else ("validate",),
        blockers=() if available else ("no native pseudo_or_rigid_body_dual_launch_executor",),
        claim_boundary=(
            "Point-mass readiness applies only to the synthetic source-generated launch workflow."
            if available
            else "This fidelity has no native dual-launch executor and cannot be inferred from the source point-mass problem."
        ),
    )
    ####


def _output_schema() -> TrajectoryOutputSchema:
    core = tuple(
        _output_channel(
            identifier,
            label,
            description,
            quantity,
            unit,
            frame="local_ned",
        )
        for identifier, label, description, quantity, unit in (
            ("position.local.north", "North Position", "North displacement from the native launch datum.", "length", "m"),
            ("position.local.east", "East Position", "East displacement from the native launch datum.", "length", "m"),
            ("position.local.down", "Down Position", "Down displacement from the native launch datum.", "length", "m"),
            ("velocity.local.north", "North Velocity", "Finite-difference north velocity from native local position samples.", "speed", "m/s"),
            ("velocity.local.east", "East Velocity", "Finite-difference east velocity from native local position samples.", "speed", "m/s"),
            ("velocity.local.down", "Down Velocity", "Finite-difference down velocity from native local position samples.", "speed", "m/s"),
        )
    )
    telemetry = (
        _output_channel("mass.total", "Mass", "Native attached-stack or released-glider mass.", "mass", "kg"),
        _output_channel("propulsion.thrust", "Thrust", "Native source-generated propulsion thrust.", "force", "N"),
        _output_channel("propulsion.mass_flow", "Mass Flow", "Native source-generated propellant mass flow.", "mass_flow_rate", "kg/s"),
        _output_channel("control.bank.commanded", "Bank Command", "Source-generated bank command.", "angle", "deg"),
        _output_channel("control.throttle.commanded", "Throttle Command", "Source-generated throttle command.", "dimensionless", "dimensionless"),
        _output_channel("aerodynamics.dynamic_pressure", "Dynamic Pressure", "Native point-mass atmospheric dynamic pressure.", "pressure", "Pa"),
        _output_channel(
            "diagnostics.segment_index",
            "Segment Index",
            "Native segment index emitted with each sample.",
            "dimensionless",
            "dimensionless",
            interpolation="step",
        ),
    )
    return TrajectoryOutputSchema(
        model_id=DUAL_LAUNCH_MODEL_ID,
        model_version=DUAL_LAUNCH_MODEL_VERSION,
        core_channels=core,
        telemetry_channels=telemetry,
        telemetry_groups=(
            TrajectoryTelemetryGroupMetadata(
                id="resources",
                label="Resources",
                description="Native mass and propellant-consumption telemetry.",
                channel_ids=("mass.total", "propulsion.mass_flow"),
                default_selected=True,
                presentation=ValuePresentationMetadata(group="telemetry", order=10),
            ),
            TrajectoryTelemetryGroupMetadata(
                id="propulsion",
                label="Propulsion",
                description="Source-generated booster propulsion telemetry.",
                channel_ids=("propulsion.thrust",),
                presentation=ValuePresentationMetadata(group="telemetry", order=20),
            ),
            TrajectoryTelemetryGroupMetadata(
                id="controls",
                label="Generated Controls",
                description="Internal native batch command profiles.",
                channel_ids=("control.bank.commanded", "control.throttle.commanded"),
                presentation=ValuePresentationMetadata(group="telemetry", order=30),
            ),
            TrajectoryTelemetryGroupMetadata(
                id="aerodynamics",
                label="Aerodynamics",
                description="Native point-mass aerodynamic telemetry.",
                channel_ids=("aerodynamics.dynamic_pressure",),
                presentation=ValuePresentationMetadata(group="telemetry", order=40),
            ),
            TrajectoryTelemetryGroupMetadata(
                id="diagnostics",
                label="Diagnostics",
                description="Native source-segment execution state.",
                channel_ids=("diagnostics.segment_index",),
                presentation=ValuePresentationMetadata(group="telemetry", order=50),
            ),
        ),
        entity_output=TrajectoryEntityOutputMetadata(includes_lifecycle_events=True),
        claim_boundary="The point-mass batch route emits normalized state, resource, source-generated-control, and source-segment telemetry. It does not emit independently propagated post-separation children.",
    )
    ####


def _output_channel(
    identifier: str,
    label: str,
    description: str,
    quantity: str,
    unit: str,
    *,
    frame: str | None = None,
    interpolation: Literal["linear", "step"] = "linear",
) -> TrajectoryOutputChannelMetadata:
    return TrajectoryOutputChannelMetadata(
        id=identifier,
        label=label,
        description=description,
        quantity=quantity,
        canonical_unit=unit,
        display_unit=unit,
        frame=frame,
        availability="guaranteed",
        compatible_fidelities=(_POINT_MASS,),
        compatible_realizations=(_REALIZATION_ID,),
        operations=("batch",),
        interpolation=interpolation,
        presentation=ValuePresentationMetadata(group="telemetry", order=0),
        source_refs=("src/taoryx/trajectory/dual_launch.py",),
        provenance="Alpha 2 dual-launch native point-mass projection",
        claim_boundary="Guaranteed only for the registered synthetic point-mass batch path.",
    )
    ####


def _number(
    identifier: str,
    label: str,
    unit: str,
    default: float,
    minimum: float | None = None,
    maximum: float | None = None,
) -> ConfigurationParameterSchema:
    return ConfigurationParameterSchema(
        id=identifier,
        label=label,
        description=label,
        value_type="number",
        quantity={
            "m": "length",
            "m/s": "speed",
            "s": "time",
            "kg": "mass",
            "N": "force",
            "kg/s": "mass_rate",
            "deg": "angle",
            "dimensionless": "dimensionless",
        }[unit],
        canonical_unit=unit,
        display_unit=unit,
        required=False,
        default=default,
        default_declared=True,
        interval=ConfigurationInterval(
            minimum=ConfigurationBound(value=minimum) if minimum is not None else None,
            maximum=ConfigurationBound(value=maximum) if maximum is not None else None,
        ),
        role="initialization",
        availability="declared",
        provenance="packages/taoryx-dual-launch/verification/dual_launch_family_catalog.yaml",
    )
    ####


def _angle(identifier: str, label: str, default: float) -> ConfigurationParameterSchema:
    return _number(identifier, label, "deg", default, 0.0, 360.0).model_copy(
        update={"periodicity": ConfigurationPeriodicity(period=360.0, canonical_minimum=0.0)}
    )
    ####


__all__ = [
    "DUAL_LAUNCH_MODEL_ID",
    "DUAL_LAUNCH_MODEL_VERSION",
    "build_dual_launch_example_configuration",
    "build_dual_launch_prepared_case",
    "dual_launch_configuration_schema",
    "dual_launch_model_metadata",
]
