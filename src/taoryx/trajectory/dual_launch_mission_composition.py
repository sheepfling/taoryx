"""Portable, fail-closed advertisement for the Alpha 2 dual-launch glider."""

from __future__ import annotations

from .configuration_contract import (
    ConfigurationBound,
    ConfigurationChoiceSchema,
    ConfigurationChoiceVariant,
    ConfigurationGroupSchema,
    ConfigurationInterval,
    ConfigurationParameterSchema,
    ConfigurationPeriodicity,
    ConfigurationSequenceSchema,
    ConfigurationSequenceTemplate,
    TrajectoryConfigurationSchema,
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
    ValuePresentationMetadata,
)

DUAL_LAUNCH_MODEL_ID = "dual_launch_glider"
DUAL_LAUNCH_MODEL_VERSION = "1.0.0-mission-composition-v1"
_FIDELITIES = (
    "point_mass_3dof",
    "pseudo_6dof",
    "rigid_body_6dof_direct_wrench",
)


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
            "The schema proves portable configuration and source-generation coverage. It does not advertise a common "
            "batch or stateful executor until an exact immutable composition binding is registered."
        ),
    )
    ####


def dual_launch_model_metadata(schema: TrajectoryConfigurationSchema | None = None) -> TrajectoryModelMetadata:
    """Publish dual-launch discovery without overstating native executability."""

    resolved = schema or dual_launch_configuration_schema()
    fidelities = tuple(_fidelity(item, index) for index, item in enumerate(_FIDELITIES))
    operations = tuple(
        TrajectoryMissionOperationMetadata(
            fidelity=fidelity,
            realization_id="generated_native_problem",
            operation=operation,
            status="available" if operation == "validate" else "blocked",
            execution_mode="portable_schema_validation" if operation == "validate" else None,
            blockers=() if operation == "validate" else ("immutable_vehicle_composition_binding", "common_executor_adapter"),
            claim_boundary="Native source generation is not an executable common-runner binding.",
        )
        for fidelity in _FIDELITIES
        for operation in ("validate", "batch", "step")
    )
    missions = tuple(
        TrajectoryMissionTemplateMetadata(
            id=identifier,
            name=label,
            description=description,
            status="configuration_ready_execution_blocked",
            initialization_variants=(launch_mode,),
            segment_sequence=segments,
            compatible_fidelities=_FIDELITIES,
            operations=operations,
            provenance="verification/alpha2_family_catalog.yaml; src/taoryx/trajectory/dual_launch.py",
            claim_boundary="Configuration-ready launch form; execution remains explicitly blocked.",
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
        compatible_fidelities=_FIDELITIES,
        operations=(),
        state_initialization="inherited_at_accepted_boundary",
        fidelity_policy="inherit_parent",
        lifecycle="event_only",
        blockers=("independent_parent_and_child_trajectory_binding",),
        source_refs=("verification/alpha2_family_catalog.yaml", "src/taoryx/trajectory/dual_launch.py"),
        claim_boundary="The native generator records separation continuity but does not yet return two independent trajectories.",
    )
    return TrajectoryModelMetadata(
        id=DUAL_LAUNCH_MODEL_ID,
        name="Dual-Launch Glider",
        version=resolved.model_version,
        description="One glider mission configurable for either air release or attached-booster launch.",
        presentation=TrajectoryModelPresentationMetadata(
            display_name="Dual-Launch Glider",
            short_name="Dual Launch",
            summary="Shared glider mission with air-release and attached-booster initialization forms.",
            category="Composition Proof Families",
            subcategory="Glider",
            sort_key="glider:dual-launch",
            badges=("Configuration Ready", "Execution Blocked"),
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
                    provenance="verification/alpha2_family_catalog.yaml",
                    claim_boundary="Configuration topology only.",
                ),
            ),
        ),
        family_id=DUAL_LAUNCH_MODEL_ID,
        physical_family="glider",
        model_kind="composition_proof_family",
        status="configuration_ready_execution_blocked",
        tags=("glider", "dual-launch", "deployment"),
        operations=("discover", "validate"),
        common_runner_operations=(),
        capabilities=TrajectoryModelCapabilities(
            initialization_modes=("air_release", "attached_booster"),
            segment_types=("attached_booster", "release_glide", "terminal_guidance"),
            termination_modes=("separation", "time", "waypoint_capture"),
            operations=("discover", "validate"),
            supports_deployment=True,
            supports_staging=True,
            supports_submodels=True,
        ),
        realizations=(
            TrajectoryRealizationMetadata(
                id="generated_native_problem",
                label="Generated Native Problem",
                description="Alpha 2 resolved-case source generator shared by both launch forms.",
                status="blocked",
                dynamics_fidelities=("point_mass_3dof", "pseudo_6dof", "rigid_body_6dof"),
                input_realization="guidance_command",
                fidelity_aliases=(*_FIDELITIES, "rigid_body_6dof"),
                mission_template_ids=tuple(item.id for item in missions),
                operations=("validate",),
                blockers=("immutable_vehicle_composition_binding", "common_executor_adapter"),
                source_refs=("src/taoryx/trajectory/dual_launch.py",),
                claim_boundary="Source generation alone is not common-runner execution availability.",
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
        source_refs=("verification/alpha2_family_catalog.yaml", "src/taoryx/trajectory/dual_launch.py"),
        provenance="Synthetic successor-side Alpha 2 dual-launch composition proof.",
        claim_boundary="Published for generic discovery and configuration; every execution operation remains blocked.",
    )
    ####


def _fidelity(identifier: str, rank: int) -> TrajectoryFidelityMetadata:
    dynamics: TrajectoryDynamicsFidelity = (
        "point_mass_3dof"
        if identifier == "point_mass_3dof"
        else "pseudo_6dof"
        if identifier == "pseudo_6dof"
        else "rigid_body_6dof"
    )
    input_realization: TrajectoryInputRealization = (
        "direct_wrench" if dynamics == "rigid_body_6dof" else "guidance_command"
    )
    return TrajectoryFidelityMetadata(
        id=identifier,
        label=identifier.replace("_", " ").title(),
        rank=rank,
        declared=True,
        dynamics_fidelity=dynamics,
        input_realization=input_realization,
        compatibility_aliases=(identifier, "rigid_body_6dof") if dynamics == "rigid_body_6dof" else (identifier,),
        runtime_fidelity="generated_native_problem",
        control_realization=input_realization,
        promotion_status="configuration_ready_execution_blocked",
        operations=("validate",),
        blockers=("immutable_vehicle_composition_binding", "common_executor_adapter"),
        claim_boundary="Fidelity is declared by the Alpha 2 proof family but has no common execution binding.",
    )
    ####


def _output_schema() -> TrajectoryOutputSchema:
    core = tuple(
        TrajectoryOutputChannelMetadata(
            id=identifier,
            label=label,
            description=f"Expected {label.casefold()} when an execution adapter is registered.",
            quantity=quantity,
            canonical_unit=unit,
            display_unit=unit,
            frame="local_ned",
            availability="guaranteed",
            compatible_fidelities=_FIDELITIES,
            operations=(),
            claim_boundary="Declared output normalization target; no executor currently emits it.",
        )
        for identifier, label, quantity, unit in (
            ("position.local.north", "North Position", "length", "m"),
            ("position.local.east", "East Position", "length", "m"),
            ("position.local.down", "Down Position", "length", "m"),
            ("velocity.local.north", "North Velocity", "speed", "m/s"),
            ("velocity.local.east", "East Velocity", "speed", "m/s"),
            ("velocity.local.down", "Down Velocity", "speed", "m/s"),
        )
    )
    return TrajectoryOutputSchema(
        model_id=DUAL_LAUNCH_MODEL_ID,
        model_version=DUAL_LAUNCH_MODEL_VERSION,
        core_channels=core,
        entity_output=TrajectoryEntityOutputMetadata(includes_lifecycle_events=True),
        claim_boundary="Output target is published for contract completeness; execution remains blocked.",
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
        quantity={"m": "length", "m/s": "speed", "s": "time", "kg": "mass", "N": "force", "kg/s": "mass_rate", "deg": "angle", "dimensionless": "dimensionless"}[unit],
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
        provenance="verification/alpha2_family_catalog.yaml",
    )
    ####


def _angle(identifier: str, label: str, default: float) -> ConfigurationParameterSchema:
    return _number(identifier, label, "deg", default, 0.0, 360.0).model_copy(
        update={"periodicity": ConfigurationPeriodicity(period=360.0, canonical_minimum=0.0)}
    )
    ####


__all__ = [
    "DUAL_LAUNCH_MODEL_ID",
    "dual_launch_configuration_schema",
    "dual_launch_model_metadata",
]
