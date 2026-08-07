"""Mission Composition advertisement for the Simple Aero workflow.

Simple Aero is a configurable trajectory workflow rather than a physical
vehicle family.  This module projects its checked-in Alpha 2 parameter family,
specialized segment contracts, and source-shaped fixture vocabulary into the
portable Mission Composition configuration grammar.  It deliberately keeps
fixture readiness separate from vehicle promotion and runtime availability.
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Any, Literal

from ..fidelity_contracts import CANONICAL_FIDELITY_TIERS
from ..simple_aero_builder import FixedLD3DOFParameters, SimpleAeroTrajectoryBuild, SimpleAeroTrajectoryBuilder
from ..specialized_segments import SIMPLE_AERO_FAMILY_SEGMENTS, specialized_segment_contract
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
    ConfigurationOptionalSchema,
    ConfigurationParameterSchema,
    ConfigurationParameterValue,
    ConfigurationPeriodicity,
    ConfigurationSequenceSchema,
    ConfigurationSequenceTemplate,
    ConfigurationSequenceValue,
    ConfigurationValueSpace,
    PreparedTrajectoryConfiguration,
    TrajectoryConfigurationInstance,
    TrajectoryConfigurationSchema,
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
    TrajectoryRealizationMetadata,
    TrajectoryReferenceFrameMetadata,
    TrajectoryTelemetryGroupMetadata,
    ValuePresentationMetadata,
)
from .contracts import FamilyPackage, ParameterSchema

SIMPLE_AERO_MODEL_ID = "simple_aero"
_ROOT = Path(__file__).resolve().parents[3]
_FAMILY_CATALOG = _ROOT / "verification" / "alpha2_family_catalog.yaml"
_MODEL_VERSION_SUFFIX = "mission-composition-v1"
_POINT_MASS = "point_mass_3dof"

_QUANTITY_BY_UNIT = {
    "dimensionless": "dimensionless",
    "m": "length",
    "m/s": "speed",
    "m/s^2": "acceleration",
    "s": "time",
    "deg": "angle",
    "Hz": "frequency",
    "kg": "mass",
    "N": "force",
    "kg/s": "mass_flow_rate",
}

_PARAMETER_DESCRIPTIONS = {
    "vehicle.id": "Stable identifier written into the generated point-mass trajectory.",
    "vehicle.mass.initial": "Vehicle mass at the launch state.",
    "vehicle.booster.thrust": "Constant surrogate thrust used during powered ascent.",
    "vehicle.booster.mass_flow": "Constant surrogate propellant mass-flow rate during powered ascent.",
    "vehicle.aero.drag_coefficient": "Constant axial drag coefficient for the fixed-coefficient surrogate.",
    "vehicle.aero.lift_to_drag": "Fixed lift-to-drag ratio used to derive the surrogate normal-force coefficient.",
    "mission.initial_altitude": "Geodetic altitude at the launch point.",
    "mission.initial_speed": "Speed at the launch point.",
    "mission.burnout_speed": "Requested burnout-speed checkpoint; not a solved vehicle-design constraint.",
    "mission.apogee_altitude": "Requested apogee checkpoint used to derive a default coast duration.",
    "mission.pitch_over_angle": "Initial flight-path or pitch-over angle for the reduced-order launch state.",
    "mission.target_range": "Spherical-Earth surface range from launch point to aimpoint.",
    "mission.target_bearing": "Initial bearing from launch point to aimpoint.",
    "mission.initial_heading_offset": "Signed heading offset added to the launch-to-aimpoint bearing.",
    "mission.target_altitude": "Altitude assigned to the endpoint target trajectory.",
    "mission.target_speed": "Speed assigned to the endpoint target trajectory.",
    "mission.target_heading": "Heading assigned to the endpoint target trajectory.",
    "runtime.time_step": "Integrator time step for the generated point-mass problem.",
    "runtime.output_interval": "Requested output sampling interval for the generated problem.",
}


def simple_aero_configuration_schema() -> TrajectoryConfigurationSchema:
    """Return the portable configuration tree for the Simple Aero workflow."""

    family = _simple_aero_family()
    launch = ConfigurationGroupSchema(
        id="launch_state",
        label="Launch State",
        description="Start point and reduced-order initial state.",
        children=(
            _number(
                "launch.latitude_deg",
                "Launch Latitude",
                "Geodetic latitude of the launch point.",
                unit="deg",
                default=0.0,
                lower=-90.0,
                upper=90.0,
                role="initialization",
                provenance="tests/fixtures/simple_aero_v1/*.yaml",
            ),
            _number(
                "launch.longitude_deg",
                "Launch Longitude",
                "Canonical geodetic longitude of the launch point.",
                unit="deg",
                default=0.0,
                periodicity=ConfigurationPeriodicity(period=360.0, canonical_minimum=-180.0),
                role="initialization",
                provenance="tests/fixtures/simple_aero_v1/*.yaml",
            ),
            _family_parameter(family, "mission.initial_altitude", role="initialization"),
            _family_parameter(family, "mission.initial_speed", role="initialization"),
            _family_parameter(family, "mission.pitch_over_angle", role="initialization"),
            _family_parameter(family, "mission.initial_heading_offset", role="initialization"),
        ),
    )
    endpoint = ConfigurationChoiceSchema(
        id="endpoint",
        label="Endpoint Definition",
        description="Choose a range/bearing aimpoint or supply an explicit geodetic aimpoint.",
        variants=(
            ConfigurationChoiceVariant(
                id="range_bearing",
                label="Range and Bearing",
                description="Derive the endpoint from spherical-Earth surface range and initial bearing.",
                compatible_fidelities=(_POINT_MASS,),
                node=ConfigurationGroupSchema(
                    id="range_bearing_parameters",
                    label="Range and Bearing Parameters",
                    children=(
                        _family_parameter(family, "mission.target_range", role="initialization"),
                        _family_parameter(
                            family,
                            "mission.target_bearing",
                            role="initialization",
                            periodicity=ConfigurationPeriodicity(period=360.0, canonical_minimum=0.0),
                            interval=None,
                        ),
                    ),
                ),
            ),
            ConfigurationChoiceVariant(
                id="geodetic_aimpoint",
                label="Geodetic Aimpoint",
                description="Use the explicit latitude and longitude shape present in the Simple Aero source fixtures.",
                compatible_fidelities=(_POINT_MASS,),
                node=ConfigurationGroupSchema(
                    id="geodetic_aimpoint_parameters",
                    label="Geodetic Aimpoint Parameters",
                    children=(
                        _number(
                            "aimpoint.latitude_deg",
                            "Aimpoint Latitude",
                            "Geodetic latitude of the endpoint aimpoint.",
                            unit="deg",
                            default=0.0,
                            lower=-90.0,
                            upper=90.0,
                            role="initialization",
                            provenance="tests/fixtures/simple_aero_v1/*.yaml",
                        ),
                        _number(
                            "aimpoint.longitude_deg",
                            "Aimpoint Longitude",
                            "Canonical geodetic longitude of the endpoint aimpoint.",
                            unit="deg",
                            default=0.0,
                            periodicity=ConfigurationPeriodicity(period=360.0, canonical_minimum=-180.0),
                            role="initialization",
                            provenance="tests/fixtures/simple_aero_v1/*.yaml",
                        ),
                    ),
                ),
            ),
        ),
    )
    endpoint_state = ConfigurationGroupSchema(
        id="endpoint_state",
        label="Endpoint State",
        description="Optional motion assigned to the endpoint target trajectory.",
        children=(
            _family_parameter(family, "mission.target_altitude", role="initialization"),
            _family_parameter(family, "mission.target_speed", role="initialization"),
            _family_parameter(
                family,
                "mission.target_heading",
                role="initialization",
                periodicity=ConfigurationPeriodicity(period=360.0, canonical_minimum=-180.0),
                interval=None,
            ),
        ),
    )
    vehicle = ConfigurationGroupSchema(
        id="vehicle_surrogate",
        label="Vehicle and Aero Surrogate",
        description="Small fixed-coefficient point-mass parameter surface; not a promoted vehicle model.",
        children=tuple(
            _family_parameter(family, parameter_id, role="initialization")
            for parameter_id in (
                "vehicle.id",
                "vehicle.mass.initial",
                "vehicle.booster.thrust",
                "vehicle.booster.mass_flow",
                "vehicle.aero.drag_coefficient",
                "vehicle.aero.lift_to_drag",
            )
        ),
    )
    constraints = ConfigurationGroupSchema(
        id="trajectory_checkpoints",
        label="Trajectory Checkpoints",
        description="Convenience targets used by the reduced-order builder; they are not solved design constraints.",
        children=(
            _family_parameter(family, "mission.burnout_speed", role="constraint"),
            _family_parameter(family, "mission.apogee_altitude", role="constraint"),
        ),
    )
    segments = _segment_sequence(family)
    runtime = ConfigurationGroupSchema(
        id="runtime",
        label="Runtime Settings",
        description="Numerical and environment settings for generated point-mass problems.",
        children=(
            _family_parameter(family, "runtime.time_step", role="constraint"),
            _family_parameter(family, "runtime.output_interval", role="output"),
            _enum(
                "earth_model",
                "Earth and Atmosphere",
                "Environment preset understood by the reduced-order builder.",
                choices=("standard_wgs84", "vacuum_spherical"),
                default="standard_wgs84",
                role="constraint",
                provenance="src/taoryx/simple_aero_builder.py",
            ),
        ),
    )
    return TrajectoryConfigurationSchema(
        model_id=SIMPLE_AERO_MODEL_ID,
        model_version=_model_version(family),
        supported_fidelities=(_POINT_MASS,),
        root=ConfigurationGroupSchema(
            id="mission",
            label="Simple Aero Mission",
            description="Launch, endpoint, surrogate vehicle, ordered maneuver segments, and runtime settings.",
            children=(launch, endpoint, endpoint_state, vehicle, constraints, segments, runtime),
        ),
        claim_boundary=(
            "Validation proves the portable tree shape, local value domains, units, point-mass fidelity, and segment-template "
            "compatibility. Coupled feasibility such as burnout speed above launch speed, apogee above launch altitude, "
            "target capture, and vehicle-specific maneuver response remains an execution or promotion gate."
        ),
    )
    ####


def simple_aero_model_metadata(schema: TrajectoryConfigurationSchema | None = None) -> TrajectoryModelMetadata:
    """Return provider-independent discovery metadata for Simple Aero."""

    family = _simple_aero_family()
    resolved_schema = schema or simple_aero_configuration_schema()
    sequence = _segments(resolved_schema)
    missions = tuple(_mission_metadata(template) for template in sequence.templates)
    fidelities = tuple(_simple_aero_fidelity(tier) for tier in CANONICAL_FIDELITY_TIERS)
    transitions = tuple(
        transition
        for lower, upper in zip(CANONICAL_FIDELITY_TIERS[:-1], CANONICAL_FIDELITY_TIERS[1:], strict=True)
        for transition in (
            _unavailable_transition(lower, upper, direction="step_up"),
            _unavailable_transition(upper, lower, direction="step_down"),
        )
    )
    presentation = TrajectoryModelPresentationMetadata(
        display_name="Simple Aero Fixed-L/D 3-DOF Workflow",
        short_name="Simple Aero",
        summary="Launch-to-aimpoint workflow with reusable point-mass maneuver segments.",
        category="Trajectory Workflows",
        subcategory="Simple Aero",
        sort_key="workflow:simple-aero",
        badges=("Workflow", "Runnable Baseline", "Custom Segments"),
        default_fidelity_id=_POINT_MASS,
        default_mission_template_id="fixed_ld_baseline",
        default_output_channel_ids=(
            "position.geodetic.altitude",
            "velocity.speed",
            "mass.total",
        ),
        properties=(
            _simple_aero_property(
                "workflow_kind",
                "Workflow Kind",
                "Provider classification for this composition-first model.",
                value_type="string",
                value="trajectory_workflow",
                group="identity",
                order=10,
            ),
            _simple_aero_property(
                "segment_type_count",
                "Segment Types",
                "Number of reusable segment variants available to a custom mission sequence.",
                value_type="integer",
                value=len(_segment_names()),
                group="capabilities",
                order=20,
            ),
            _simple_aero_property(
                "mission_template_count",
                "Mission Templates",
                "Number of reviewed starting sequences published by the workflow.",
                value_type="integer",
                value=len(sequence.templates),
                group="capabilities",
                order=30,
            ),
            TrajectoryModelPropertyMetadata(
                id="custom_segment_count",
                label="Custom Segment Count",
                description="Supported cardinality for caller-authored segment sequences.",
                semantic_role="capability",
                value_type="integer",
                value_kind="range",
                interval=ConfigurationInterval(
                    minimum=ConfigurationBound(value=float(sequence.minimum_items)),
                    maximum=ConfigurationBound(value=float(sequence.maximum_items)) if sequence.maximum_items is not None else None,
                ),
                presentation=ValuePresentationMetadata(group="capabilities", order=40),
                provenance="Simple Aero sequence grammar",
                claim_boundary="Structural cardinality only; individual sequences still require semantic and execution validation.",
            ),
        ),
    )
    output_schema = _simple_aero_output_schema(resolved_schema)
    return TrajectoryModelMetadata(
        id=SIMPLE_AERO_MODEL_ID,
        name="Simple Aero Fixed-L/D 3-DOF Workflow",
        version=resolved_schema.model_version,
        description=(
            "Configurable launch-to-aimpoint point-mass workflow with source-shaped ballistic, bank, alpha-profile, "
            "skip, weave, and terminal-guidance segment templates."
        ),
        presentation=presentation,
        family_id=family.family_id,
        physical_family=None,
        model_kind="trajectory_workflow",
        status="common_runner_ready",
        tags=("simple-aero", "workflow", "point-mass", "fixed-ld", "synthetic"),
        operations=("discover", "validate", "batch"),
        common_runner_operations=("batch",),
        capabilities=TrajectoryModelCapabilities(
            initialization_modes=("launch_state", "range_bearing", "geodetic_aimpoint"),
            segment_types=_segment_names(),
            termination_modes=("time", "physical_burnout", "commanded_heading", "commanded_range"),
            operations=("discover", "validate", "batch"),
            supports_custom_segments=True,
        ),
        realizations=(
            TrajectoryRealizationMetadata(
                id="fixed_ld_point_mass",
                label="Fixed-L/D Point-Mass",
                description="Generated native point-mass realization for the executable fixed-L/D baseline templates.",
                status="available",
                dynamics_fidelities=("point_mass_3dof",),
                input_realization="guidance_command",
                fidelity_aliases=(_POINT_MASS,),
                mission_template_ids=tuple(item.id for item in missions),
                operations=("validate", "batch"),
                native_factory_ids=("simple_aero_generated_problem.v1",),
                source_refs=("src/taoryx/simple_aero_builder.py",),
                claim_boundary="Synthetic fixed-coefficient workflow only; this is not vehicle-family qualification.",
            ),
        ),
        mission_templates=missions,
        reference_frames=(
            TrajectoryReferenceFrameMetadata(
                id="geodetic",
                name="Geodetic Coordinates",
                description="Latitude, longitude, and altitude relative to the configured Earth model.",
                frame_kind="geodetic",
                axes=("latitude", "longitude", "altitude"),
                handedness="not_applicable",
                origin="configured Earth ellipsoid",
                orientation="geodetic latitude/longitude with positive-up altitude",
                provenance="generated Simple Aero runtime problem",
            ),
        ),
        output_schema=output_schema,
        output_schema_id=output_schema.schema_id,
        output_schema_fingerprint=output_schema.fingerprint,
        configuration_schema_id=resolved_schema.schema_id,
        configuration_schema_fingerprint=resolved_schema.fingerprint,
        fidelities=fidelities,
        fidelity_transitions=transitions,
        source_refs=(
            "verification/alpha2_family_catalog.yaml",
            "verification/simple_aero_segment_catalog.yaml",
            "tests/fixtures/simple_aero_v1",
            "src/taoryx/simple_aero_builder.py",
        ),
        provenance="synthetic successor-side workflow plus traced Simple Aero fixture vocabulary",
        claim_boundary=(
            "This is a configurable workflow and contract fixture, not a physical vehicle. The fixed-L/D baseline has a "
            "builder/runtime path; named maneuver templates advertise fixture-level semantics until a configuration-to-runtime "
            "adapter and vehicle-specific evidence are registered."
        ),
    )
    ####


def _simple_aero_property(
    property_id: str,
    label: str,
    description: str,
    *,
    value_type: Literal["integer", "string"],
    value: int | str,
    group: str,
    order: int,
) -> TrajectoryModelPropertyMetadata:
    return TrajectoryModelPropertyMetadata(
        id=property_id,
        label=label,
        description=description,
        semantic_role="capability" if group == "capabilities" else "implementation",
        value_type=value_type,
        value_kind="declared",
        value=value,
        value_declared=True,
        presentation=ValuePresentationMetadata(group=group, order=order),
        provenance="Simple Aero Mission Composition advertisement",
        claim_boundary="Workflow metadata only; not physical-vehicle qualification evidence.",
    )
    ####


def _simple_aero_output_channels() -> tuple[TrajectoryOutputChannelMetadata, ...]:
    return tuple(
        TrajectoryOutputChannelMetadata(
            id=channel_id,
            label=label,
            description=description,
            quantity=quantity,
            canonical_unit=unit,
            display_unit=unit,
            availability="guaranteed",
            compatible_fidelities=(_POINT_MASS,),
            operations=("batch",),
            frame=frame,
            interpolation="periodic" if channel_id == "position.geodetic.longitude" else "linear",
            periodicity=(
                ConfigurationPeriodicity(period=360.0, canonical_minimum=-180.0)
                if channel_id == "position.geodetic.longitude"
                else None
            ),
            presentation=ValuePresentationMetadata(group=group, order=order),
            provenance="src/taoryx/simple_aero_builder.py and Simulation Runtime output projection",
            claim_boundary="Guaranteed for the primary Simple Aero vehicle in the registered fixed-L/D batch path.",
        )
        for order, (channel_id, label, description, quantity, unit, frame, group) in enumerate(
            (
                ("position.geodetic.altitude", "Altitude", "Geodetic altitude of the primary point-mass vehicle.", "length", "m", "geodetic", "core_state"),
                ("position.geodetic.latitude", "Latitude", "Geodetic latitude of the primary point-mass vehicle.", "angle", "deg", "geodetic", "core_state"),
                ("position.geodetic.longitude", "Longitude", "Geodetic longitude of the primary point-mass vehicle.", "angle", "deg", "geodetic", "core_state"),
                ("velocity.speed", "Speed", "Scalar speed of the primary point-mass vehicle.", "speed", "m/s", None, "core_state"),
                ("mass.total", "Mass", "Current mass of the primary point-mass vehicle.", "mass", "kg", None, "resources"),
                ("propulsion.mass_flow", "Mass Flow", "Propellant mass-flow rate of the surrogate booster.", "mass_rate", "kg/s", None, "resources"),
                ("propulsion.thrust", "Thrust", "Realized surrogate propulsion thrust.", "force", "N", None, "propulsion"),
                ("propulsion.throttle_command", "Throttle Command", "Generated normalized propulsion command.", "dimensionless", "1", None, "controls"),
                ("aerodynamics.dynamic_pressure", "Dynamic Pressure", "Atmospheric dynamic pressure.", "pressure", "Pa", None, "aerodynamics"),
                ("aerodynamics.angle_of_attack", "Angle of Attack", "Generated point-mass angle of attack.", "angle", "deg", None, "aerodynamics"),
            ),
            start=10,
        )
    )
    ####


def _simple_aero_output_schema(schema: TrajectoryConfigurationSchema) -> TrajectoryOutputSchema:
    channels = _simple_aero_output_channels()
    return TrajectoryOutputSchema(
        model_id=schema.model_id,
        model_version=schema.model_version,
        core_channels=tuple(item for item in channels if item.presentation.group == "core_state"),
        telemetry_channels=tuple(item for item in channels if item.presentation.group != "core_state"),
        telemetry_groups=(
            TrajectoryTelemetryGroupMetadata(
                id="resources",
                label="Resources",
                description="Mass and consumable-resource telemetry from the Simple Aero trajectory.",
                channel_ids=("mass.total", "propulsion.mass_flow"),
                default_selected=True,
                presentation=ValuePresentationMetadata(group="telemetry", order=10),
            ),
            TrajectoryTelemetryGroupMetadata(
                id="propulsion",
                label="Propulsion",
                description="Surrogate booster propulsion output.",
                channel_ids=("propulsion.thrust",),
                presentation=ValuePresentationMetadata(group="telemetry", order=20),
            ),
            TrajectoryTelemetryGroupMetadata(
                id="controls",
                label="Controls",
                description="Generated control commands applied by the workflow.",
                channel_ids=("propulsion.throttle_command",),
                presentation=ValuePresentationMetadata(group="telemetry", order=30),
            ),
            TrajectoryTelemetryGroupMetadata(
                id="aerodynamics",
                label="Aerodynamics",
                description="Atmospheric and aerodynamic state from the point-mass runtime.",
                channel_ids=("aerodynamics.dynamic_pressure", "aerodynamics.angle_of_attack"),
                presentation=ValuePresentationMetadata(group="telemetry", order=40),
            ),
        ),
        claim_boundary=(
            "The fixed-L/D baseline guarantees the advertised core state and mass telemetry. "
            "Other Simple Aero mission templates remain subject to their exact execution advertisement."
        ),
    )
    ####


def build_simple_aero_prepared_configuration(
    prepared: PreparedTrajectoryConfiguration,
) -> SimpleAeroTrajectoryBuild:
    """Compile one validated fixed-L/D baseline configuration.

    Family-specific Simple Aero maneuver templates are intentionally rejected
    here until their source-shaped parameters have registered runtime adapters.
    """

    configuration = prepared.configuration
    if configuration.model_id != SIMPLE_AERO_MODEL_ID:
        raise ConfigurationContractError(
            "model-mismatch",
            f"expected model {SIMPLE_AERO_MODEL_ID!r}",
            path="prepared.configuration.model_id",
        )
    root = prepared.resolved
    if not isinstance(root, dict):
        raise ConfigurationContractError("invalid-prepared-configuration", "resolved root must be a mapping")
    segment_values = root["segments"]
    if not isinstance(segment_values, list):
        raise ConfigurationContractError("invalid-prepared-configuration", "resolved segments must be a list")
    selected = tuple(str(item["selected"]) for item in segment_values)
    baseline = ("powered_ascent", "ballistic_coast", "bank_maneuver", "terminal_pronav")
    if selected != baseline:
        raise ConfigurationContractError(
            "execution-template-unavailable",
            f"the registered builder accepts only {baseline!r}; received {selected!r}",
            path="prepared.resolved.segments",
        )

    launch = _resolved_mapping(root, "launch_state")
    endpoint = _resolved_mapping(root, "endpoint")
    endpoint_state = _resolved_mapping(root, "endpoint_state")
    vehicle = _resolved_mapping(root, "vehicle_surrogate")
    checkpoints = _resolved_mapping(root, "trajectory_checkpoints")
    runtime = _resolved_mapping(root, "runtime")
    powered = _resolved_segment(segment_values[0], "powered_ascent")
    coast = _resolved_segment(segment_values[1], "ballistic_coast")
    bank = _resolved_segment(segment_values[2], "bank_maneuver")
    terminal = _resolved_segment(segment_values[3], "terminal_pronav")

    endpoint_kind = str(endpoint["selected"])
    endpoint_values = endpoint.get("value")
    if not isinstance(endpoint_values, dict):
        raise ConfigurationContractError("invalid-prepared-configuration", "resolved endpoint value must be a mapping")
    target_arguments: dict[str, float]
    if endpoint_kind == "range_bearing":
        target_arguments = {
            "target_range_m": float(endpoint_values["mission.target_range"]),
            "target_bearing_deg": float(endpoint_values["mission.target_bearing"]),
        }
    elif endpoint_kind == "geodetic_aimpoint":
        target_arguments = {
            "target_latitude_deg": float(endpoint_values["aimpoint.latitude_deg"]),
            "target_longitude_deg": float(endpoint_values["aimpoint.longitude_deg"]),
        }
    else:
        raise ConfigurationContractError("unknown-choice", f"unsupported endpoint {endpoint_kind!r}")

    cutoff = str(powered["cutoff_condition"])
    cutoff_mode = {
        "physical_burnout": "physical",
        "commanded_burnout_speed": "commanded",
    }[cutoff]
    parameters = FixedLD3DOFParameters(
        scenario_id=configuration.configuration_id,
        vehicle_id=str(vehicle["vehicle.id"]),
        family="fixed_ld_baseline",
        initial_altitude_m=float(launch["mission.initial_altitude"]),
        initial_speed_m_s=float(launch["mission.initial_speed"]),
        vbo_m_s=float(checkpoints["mission.burnout_speed"]),
        apogee_altitude_m=float(checkpoints["mission.apogee_altitude"]),
        pitch_over_angle_deg=float(launch["mission.pitch_over_angle"]),
        initial_heading_offset_deg=float(launch["mission.initial_heading_offset"]),
        target_altitude_m=float(endpoint_state["mission.target_altitude"]),
        target_speed_m_s=float(endpoint_state["mission.target_speed"]),
        target_heading_deg=float(endpoint_state["mission.target_heading"]),
        launch_latitude_deg=float(launch["launch.latitude_deg"]),
        launch_longitude_deg=float(launch["launch.longitude_deg"]),
        mass_kg=float(vehicle["vehicle.mass.initial"]),
        thrust_n=float(vehicle["vehicle.booster.thrust"]),
        mass_flow_kg_s=float(vehicle["vehicle.booster.mass_flow"]),
        drag_coefficient=float(vehicle["vehicle.aero.drag_coefficient"]),
        lift_to_drag=float(vehicle["vehicle.aero.lift_to_drag"]),
        alpha_deg=float(coast["alpha_deg"]),
        bank_deg=float(bank["bank_deg"]),
        boost_duration_s=float(powered["duration_s"]),
        coast_duration_s=float(coast["duration_s"]),
        bank_duration_s=float(bank["duration_s"]),
        terminal_duration_s=float(terminal["duration_s"]),
        terminal_capture_range_m=float(terminal["capture_range_m"]),
        time_step_s=float(runtime["runtime.time_step"]),
        output_interval_s=float(runtime["runtime.output_interval"]),
        earth="standard" if runtime["earth_model"] == "standard_wgs84" else "none",
        **target_arguments,
    )
    segment_graph = {
        "cutoff_mode": cutoff_mode,
        "segments": [
            {"id": "powered-ascent", "kind": "powered_ascent", "duration_s": powered["duration_s"]},
            {"id": "ballistic-coast", "kind": "ballistic_coast", "duration_s": coast["duration_s"]},
            {"id": "bank-maneuver", "kind": "bank_maneuver", "duration_s": bank["duration_s"]},
            {"id": "terminal-pronav", "kind": "terminal_pronav", "duration_s": terminal["duration_s"]},
        ],
    }
    return SimpleAeroTrajectoryBuilder(parameters, segment_graph=segment_graph).build()
    ####


def build_simple_aero_example_configuration(
    schema: TrajectoryConfigurationSchema | None = None,
) -> TrajectoryConfigurationInstance:
    """Return the small checked contract witness for the runnable baseline."""

    resolved_schema = schema or simple_aero_configuration_schema()
    empty = ConfigurationGroupValue(values={})
    return TrajectoryConfigurationInstance(
        configuration_id="simple-aero-fixed-ld-common-witness",
        model_id=SIMPLE_AERO_MODEL_ID,
        model_version=resolved_schema.model_version,
        schema_fingerprint=resolved_schema.fingerprint,
        fidelity=_POINT_MASS,
        realization_id="fixed_ld_point_mass",
        mission_template_id="fixed_ld_baseline",
        root=ConfigurationGroupValue(
            values={
                "launch_state": ConfigurationGroupValue(
                    values={
                        "launch.latitude_deg": ConfigurationParameterValue(value=35.8766, unit="deg"),
                        "launch.longitude_deg": ConfigurationParameterValue(value=14.4425, unit="deg"),
                        "mission.initial_speed": ConfigurationParameterValue(value=10.0, unit="m/s"),
                    }
                ),
                "endpoint": ConfigurationChoiceValue(
                    selected="geodetic_aimpoint",
                    value=ConfigurationGroupValue(
                        values={
                            "aimpoint.latitude_deg": ConfigurationParameterValue(value=36.000975, unit="deg"),
                            "aimpoint.longitude_deg": ConfigurationParameterValue(value=-5.60999, unit="deg"),
                        }
                    ),
                ),
                "endpoint_state": empty,
                "vehicle_surrogate": ConfigurationGroupValue(
                    values={"vehicle.mass.initial": ConfigurationParameterValue(value=1000.0, unit="kg")}
                ),
                "trajectory_checkpoints": ConfigurationGroupValue(
                    values={"mission.burnout_speed": ConfigurationParameterValue(value=1200.0, unit="m/s")}
                ),
                "segments": ConfigurationSequenceValue(
                    items=tuple(
                        ConfigurationChoiceValue(selected=identifier, value=empty)
                        for identifier in (
                            "powered_ascent",
                            "ballistic_coast",
                            "bank_maneuver",
                            "terminal_pronav",
                        )
                    )
                ),
                "runtime": empty,
            }
        ),
    )
    ####


@lru_cache(maxsize=1)
def _simple_aero_family() -> FamilyPackage:
    return load_family_catalog(_FAMILY_CATALOG).family(SIMPLE_AERO_MODEL_ID)
    ####


def _resolved_mapping(root: dict[str, Any], key: str) -> dict[str, Any]:
    value = root.get(key)
    if not isinstance(value, dict):
        raise ConfigurationContractError(
            "invalid-prepared-configuration",
            f"resolved field {key!r} must be a mapping",
            path=f"prepared.resolved.{key}",
        )
    return value
    ####


def _resolved_segment(value: Any, selected: str) -> dict[str, Any]:
    if not isinstance(value, dict) or value.get("selected") != selected or not isinstance(value.get("value"), dict):
        raise ConfigurationContractError(
            "invalid-prepared-configuration",
            f"expected resolved {selected!r} segment",
            path="prepared.resolved.segments",
        )
    return value["value"]
    ####


def _model_version(family: FamilyPackage) -> str:
    return f"{family.version}+{_MODEL_VERSION_SUFFIX}"
    ####


def _segment_sequence(family: FamilyPackage) -> ConfigurationSequenceSchema:
    variants = tuple(_segment_variant(name, family) for name in _segment_names())
    templates = (
        ConfigurationSequenceTemplate(
            id="fixed_ld_baseline",
            label="Fixed-L/D Baseline",
            description="The four-phase sequence implemented by the reduced-order Simple Aero builder.",
            item_variants=("powered_ascent", "ballistic_coast", "bank_maneuver", "terminal_pronav"),
            compatible_fidelities=(_POINT_MASS,),
        ),
        *(
            ConfigurationSequenceTemplate(
                id=family_id,
                label=_label(family_id),
                description=f"Source-shaped {family_id.replace('_', ' ')} fixture template projected onto reusable segment contracts.",
                item_variants=_family_segment_sequence(family_id),
                compatible_fidelities=(_POINT_MASS,),
            )
            for family_id in SIMPLE_AERO_FAMILY_SEGMENTS
            if family_id != "propnav"
        ),
    )
    return ConfigurationSequenceSchema(
        id="segments",
        label="Mission Segments",
        description=(
            "Ordered, per-occurrence segment configurations. Reviewed templates provide easy starting points; custom "
            "sequences remain allowed for composition experiments."
        ),
        item=ConfigurationChoiceSchema(
            id="segment",
            label="Simple Aero Segment",
            description="Select one reusable or source-shaped maneuver segment.",
            variants=variants,
        ),
        minimum_items=1,
        maximum_items=32,
        templates=templates,
        allow_custom=True,
    )
    ####


def _segment_names() -> tuple[str, ...]:
    return (
        "powered_ascent",
        "ballistic_coast",
        "bank_maneuver",
        "cbcr",
        "crossrange",
        "marv",
        "phugoid",
        "range_extension",
        "skip",
        "slalom",
        "weave",
        "terminal_pronav",
    )
    ####


def _family_segment_sequence(family_id: str) -> tuple[str, ...]:
    sequence: list[str] = []
    for segment in SIMPLE_AERO_FAMILY_SEGMENTS[family_id]:
        if segment == "bank_maneuver":
            sequence.append(family_id)
        elif segment == "alpha_profile":
            sequence.append(family_id)
        elif segment == "skip_maneuver":
            sequence.append("skip")
        else:
            sequence.append(segment)
    return tuple(sequence)
    ####


def _segment_variant(name: str, family: FamilyPackage) -> ConfigurationChoiceVariant:
    children = _segment_parameters(name, family)
    generic_contract = _generic_contract_name(name)
    contract = specialized_segment_contract(generic_contract)
    return ConfigurationChoiceVariant(
        id=name,
        label=_label(name),
        description=f"{contract.description} {contract.claim_boundary}.",
        compatible_fidelities=(_POINT_MASS,),
        node=ConfigurationGroupSchema(
            id=f"{name}_parameters",
            label=f"{_label(name)} Parameters",
            description="Values apply to this segment occurrence only.",
            children=children,
        ),
    )
    ####


def _generic_contract_name(name: str) -> Literal[
    "powered_ascent",
    "ballistic_coast",
    "bank_maneuver",
    "alpha_profile",
    "skip_maneuver",
    "terminal_pronav",
]:
    if name in {"bank_maneuver", "cbcr", "crossrange", "marv", "slalom", "weave"}:
        return "bank_maneuver"
    if name in {"phugoid", "range_extension"}:
        return "alpha_profile"
    if name == "skip":
        return "skip_maneuver"
    return name  # type: ignore[return-value]
    ####


def _segment_parameters(name: str, family: FamilyPackage) -> tuple[ConfigurationParameterSchema | ConfigurationOptionalSchema, ...]:
    source = "tests/fixtures/simple_aero_v1/*.yaml"
    if name == "powered_ascent":
        return (
            _family_duration(family, "mission.boost_duration", "duration_s", "Powered Duration"),
            _enum(
                "cutoff_condition",
                "Cutoff Condition",
                "Condition that ends powered ascent.",
                choices=("physical_burnout", "commanded_burnout_speed"),
                default="physical_burnout",
                role="segment",
                provenance="verification/alpha2_family_catalog.yaml",
            ),
        )
    if name == "ballistic_coast":
        return (
            _family_duration(family, "mission.coast_duration", "duration_s", "Coast Duration"),
            _number("alpha_deg", "Angle of Attack", "Constant coast angle of attack.", unit="deg", default=0.0, lower=-45.0, upper=45.0, role="segment", provenance="src/taoryx/simple_aero_builder.py"),
        )
    if name == "bank_maneuver":
        return (
            _family_duration(family, "mission.bank_duration", "duration_s", "Maneuver Duration"),
            _number("bank_deg", "Bank Angle", "Signed constant bank command.", unit="deg", default=0.0, lower=-180.0, upper=180.0, role="segment", provenance="src/taoryx/simple_aero_builder.py"),
        )
    if name == "cbcr":
        return (
            _boolean("go_left", "Turn Left", "Select the source-shaped left or right CBCR maneuver.", True, source),
            _number("maneuver_altitude_start_m", "Maneuver Start Altitude", "Altitude checkpoint for maneuver entry.", unit="m", default=40_000.0, lower=0.0, role="segment", provenance=source),
            _number("duration_s", "Maneuver Duration", "Requested CBCR maneuver duration.", unit="s", default=35.0, lower=0.0, lower_inclusive=False, role="segment", provenance=source),
            _number("minimum_time_to_go_s", "Minimum Time to Go", "Terminal time-to-go floor.", unit="s", default=60.0, lower=0.0, role="segment", provenance=source),
        )
    if name == "crossrange":
        return (
            _number("initial_heading_error_deg", "Initial Heading Error", "Signed launch heading error used to seed crossrange.", unit="deg", default=15.0, lower=-180.0, upper=180.0, role="segment", provenance=source),
            _number("minimum_time_to_go_s", "Minimum Time to Go", "Terminal time-to-go floor.", unit="s", default=60.0, lower=0.0, role="segment", provenance=source),
        )
    if name == "marv":
        return (
            _number("maneuver_begin_time_to_go_s", "Maneuver Begin Time to Go", "Time-to-go checkpoint for maneuver entry.", unit="s", default=60.0, lower=0.0, role="segment", provenance=source),
            _number("minimum_time_to_go_s", "Minimum Time to Go", "Terminal time-to-go floor.", unit="s", default=30.0, lower=0.0, role="segment", provenance=source),
        )
    if name == "phugoid":
        return (
            _optional_number("start_range_to_go_m", "Start Range to Go", "Optional range-to-go checkpoint for profile entry.", unit="m", lower=0.0, provenance=source),
            _number("amplitude_deg", "Phugoid Amplitude", "Source-shaped alpha-profile amplitude.", unit="deg", default=2.0, lower=0.0, role="segment", provenance=source),
            _number("frequency_hz", "Phugoid Frequency", "Source-shaped alpha-profile frequency.", unit="Hz", default=0.015, lower=0.0, lower_inclusive=False, role="segment", provenance=source),
            _number("maneuver_roll_deg", "Maneuver Roll", "Signed roll command associated with the profile.", unit="deg", default=45.0, lower=-180.0, upper=180.0, role="segment", provenance=source),
        )
    if name == "range_extension":
        return (_number("minimum_time_to_go_s", "Minimum Time to Go", "Terminal time-to-go floor.", unit="s", default=60.0, lower=0.0, role="segment", provenance=source),)
    if name == "skip":
        return (_number("maneuver_begin_time_to_go_s", "Maneuver Begin Time to Go", "Time-to-go checkpoint for skip entry.", unit="s", default=400.0, lower=0.0, role="segment", provenance=source),)
    if name == "slalom":
        return (
            _optional_number("start_range_to_go_m", "Start Range to Go", "Optional range-to-go checkpoint for slalom entry.", unit="m", lower=0.0, provenance=source),
            _number("end_range_to_go_m", "End Range to Go", "Range-to-go checkpoint for slalom exit.", unit="m", default=200_000.0, lower=0.0, role="segment", provenance=source),
            _number("minimum_time_to_go_s", "Minimum Time to Go", "Terminal time-to-go floor.", unit="s", default=60.0, lower=0.0, role="segment", provenance=source),
        )
    if name == "weave":
        return (
            _optional_number("end_range_to_go_m", "Weave End Range to Go", "Optional range-to-go checkpoint for weave exit.", unit="m", lower=0.0, provenance=source),
            _number("minimum_time_to_go_s", "Minimum Time to Go", "Terminal time-to-go floor.", unit="s", default=60.0, lower=0.0, role="segment", provenance=source),
        )
    if name == "terminal_pronav":
        return (
            _family_duration(family, "mission.terminal_duration", "duration_s", "Terminal Duration"),
            _renamed_family_parameter(family, "mission.terminal_capture_range", "capture_range_m", "Capture Range", role="segment"),
        )
    raise KeyError(f"unknown Simple Aero segment {name!r}")
    ####


def _family_duration(family: FamilyPackage, parameter_id: str, node_id: str, label: str) -> ConfigurationParameterSchema:
    return _renamed_family_parameter(family, parameter_id, node_id, label, role="segment")
    ####


def _renamed_family_parameter(
    family: FamilyPackage,
    parameter_id: str,
    node_id: str,
    label: str,
    *,
    role: Literal["initialization", "segment", "constraint", "variant", "output"],
) -> ConfigurationParameterSchema:
    parameter = family.parameter_map()[parameter_id]
    return _parameter_from_family(parameter, role=role).model_copy(update={"id": node_id, "label": label})
    ####


def _family_parameter(
    family: FamilyPackage,
    parameter_id: str,
    *,
    role: Literal["initialization", "segment", "constraint", "variant", "output"],
    periodicity: ConfigurationPeriodicity | None = None,
    interval: ConfigurationInterval | None | Literal["from_family"] = "from_family",
) -> ConfigurationParameterSchema:
    parameter = family.parameter_map()[parameter_id]
    resolved = _parameter_from_family(parameter, role=role)
    updates: dict[str, object] = {}
    if periodicity is not None:
        updates["periodicity"] = periodicity
        updates["value_space"] = _value_space(resolved.value_type, periodic=True)
    if interval != "from_family":
        updates["interval"] = interval
    return resolved.model_copy(update=updates)
    ####


def _parameter_from_family(
    parameter: ParameterSchema,
    *,
    role: Literal["initialization", "segment", "constraint", "variant", "output"],
) -> ConfigurationParameterSchema:
    interval = _interval(parameter.minimum, parameter.maximum)
    qualified = _interval(parameter.qualified_minimum, parameter.qualified_maximum)
    value_type = parameter.kind
    return ConfigurationParameterSchema(
        id=parameter.id,
        label=_label(parameter.id),
        description=parameter.description or _PARAMETER_DESCRIPTIONS.get(parameter.id, f"Simple Aero parameter {parameter.id}."),
        value_type=value_type,
        quantity=_QUANTITY_BY_UNIT.get(parameter.canonical_unit or ""),
        canonical_unit=parameter.canonical_unit,
        display_unit=parameter.canonical_unit,
        required=parameter.required,
        default=parameter.default,
        default_declared=not parameter.required,
        interval=interval,
        qualified_interval=qualified,
        role=role,
        compatible_fidelities=(_POINT_MASS,),
        transform="identity",
        projection_policy="reject_invalid",
        coupling_group=parameter.coupling_group,
        invalidations=tuple(
            item
            for item, active in (
                ("requires_retrim", parameter.requires_retrim),
                ("requires_requalification", parameter.requires_requalification),
            )
            if active
        ),
        value_space=_value_space(value_type, periodic=False),
        provenance=parameter.provenance or "verification/alpha2_family_catalog.yaml",
    )
    ####


def _number(
    node_id: str,
    label: str,
    description: str,
    *,
    unit: str,
    default: float,
    lower: float | None = None,
    upper: float | None = None,
    lower_inclusive: bool = True,
    upper_inclusive: bool = True,
    periodicity: ConfigurationPeriodicity | None = None,
    role: Literal["initialization", "segment", "constraint", "variant", "output"],
    provenance: str,
) -> ConfigurationParameterSchema:
    return ConfigurationParameterSchema(
        id=node_id,
        label=label,
        description=description,
        value_type="number",
        quantity=_QUANTITY_BY_UNIT.get(unit),
        canonical_unit=unit,
        display_unit=unit,
        default=default,
        default_declared=True,
        interval=_interval(lower, upper, lower_inclusive=lower_inclusive, upper_inclusive=upper_inclusive),
        periodicity=periodicity,
        role=role,
        compatible_fidelities=(_POINT_MASS,),
        value_space=_value_space("number", periodic=periodicity is not None),
        provenance=provenance,
    )
    ####


def _optional_number(
    node_id: str,
    label: str,
    description: str,
    *,
    unit: str,
    lower: float | None,
    provenance: str,
) -> ConfigurationOptionalSchema:
    return ConfigurationOptionalSchema(
        id=node_id,
        label=label,
        description=description,
        item=ConfigurationParameterSchema(
            id="value",
            label=label,
            description=description,
            value_type="number",
            quantity=_QUANTITY_BY_UNIT.get(unit),
            canonical_unit=unit,
            display_unit=unit,
            required=True,
            interval=_interval(lower, None),
            role="segment",
            compatible_fidelities=(_POINT_MASS,),
            value_space=_value_space("number", periodic=False),
            provenance=provenance,
        ),
    )
    ####


def _boolean(node_id: str, label: str, description: str, default: bool, provenance: str) -> ConfigurationParameterSchema:
    return ConfigurationParameterSchema(
        id=node_id,
        label=label,
        description=description,
        value_type="boolean",
        default=default,
        default_declared=True,
        role="segment",
        compatible_fidelities=(_POINT_MASS,),
        value_space=_value_space("boolean", periodic=False),
        provenance=provenance,
    )
    ####


def _enum(
    node_id: str,
    label: str,
    description: str,
    *,
    choices: tuple[str, ...],
    default: str,
    role: Literal["initialization", "segment", "constraint", "variant", "output"],
    provenance: str,
) -> ConfigurationParameterSchema:
    return ConfigurationParameterSchema(
        id=node_id,
        label=label,
        description=description,
        value_type="enum",
        choices=choices,
        default=default,
        default_declared=True,
        role=role,
        compatible_fidelities=(_POINT_MASS,),
        transform="categorical",
        value_space=_value_space("enum", periodic=False),
        provenance=provenance,
    )
    ####


def _interval(
    lower: float | None,
    upper: float | None,
    *,
    lower_inclusive: bool = True,
    upper_inclusive: bool = True,
) -> ConfigurationInterval | None:
    if lower is None and upper is None:
        return None
    return ConfigurationInterval(
        minimum=ConfigurationBound(value=lower, inclusive=lower_inclusive) if lower is not None else None,
        maximum=ConfigurationBound(value=upper, inclusive=upper_inclusive) if upper is not None else None,
    )
    ####


def _value_space(value_type: str, *, periodic: bool) -> ConfigurationValueSpace:
    if periodic:
        return ConfigurationValueSpace(
            topology="circle",
            representation="canonical_scalar_angle",
            error_rule="shortest_arc_difference",
            interpolation_rule="shortest_arc",
            normalization_rule="wrap_to_principal_interval",
            period=360.0,
            equivalence="values separated by integer multiples of 360 degrees are equivalent",
        )
    if value_type in {"enum", "boolean", "string"}:
        return ConfigurationValueSpace(
            topology="finite_set" if value_type != "string" else "discrete_labels",
            representation="scalar_label",
            error_rule="exact_equality",
            interpolation_rule="not_applicable",
        )
    return ConfigurationValueSpace(
        topology="interval",
        representation="scalar",
        error_rule="absolute_difference",
        interpolation_rule="linear",
    )
    ####


def _segments(schema: TrajectoryConfigurationSchema) -> ConfigurationSequenceSchema:
    if not isinstance(schema.root, ConfigurationGroupSchema):
        raise ValueError("Simple Aero schema root must be a group")
    segments = next(item for item in schema.root.children if item.id == "segments")
    if not isinstance(segments, ConfigurationSequenceSchema):
        raise ValueError("Simple Aero segments node must be a sequence")
    return segments
    ####


def _mission_metadata(template: ConfigurationSequenceTemplate) -> TrajectoryMissionTemplateMetadata:
    baseline = template.id == "fixed_ld_baseline"
    batch = TrajectoryMissionOperationMetadata(
        fidelity=_POINT_MASS,
        realization_id="fixed_ld_point_mass",
        operation="batch",
        status="available" if baseline else "blocked",
        execution_mode="build_simple_aero_prepared_configuration_then_taoryx_batch" if baseline else None,
        availability_scope="provider_interface" if baseline else "native_runtime_binding",
        common_runner_status="registered" if baseline else "not_available",
        executor_id="taoryx.simple-aero.common-batch.v1" if baseline else None,
        blockers=()
        if baseline
        else ("no family-specific Mission Composition configuration-to-runtime adapter is registered",),
        claim_boundary=(
            "Generated fixed-coefficient point-mass execution only; no vehicle-specific performance claim."
            if baseline
            else "The maneuver is advertised at fixture-contract level; schema validation is not execution permission."
        ),
    )
    return TrajectoryMissionTemplateMetadata(
        id=template.id,
        name=template.label,
        description=template.description,
        status="runnable" if baseline else "fixture_ready",
        initialization_variants=("launch_to_endpoint",),
        segment_sequence=template.item_variants,
        compatible_fidelities=(_POINT_MASS,),
        operations=(
            TrajectoryMissionOperationMetadata(
                fidelity=_POINT_MASS,
                realization_id="fixed_ld_point_mass",
                operation="validate",
                status="available",
                execution_mode="portable_schema_validation",
                claim_boundary="Structural and local-domain validation only.",
            ),
            batch,
            TrajectoryMissionOperationMetadata(
                fidelity=_POINT_MASS,
                realization_id="fixed_ld_point_mass",
                operation="step",
                status="blocked",
                blockers=("no interactive Simple Aero Mission Composition session binding is registered",),
                claim_boundary="Batch or fixture readiness does not imply an interactive stepping contract.",
            ),
        ),
        provenance="verification/simple_aero_segment_catalog.yaml; tests/fixtures/simple_aero_v1",
        claim_boundary=(
            "Template order and parameter vocabulary are advertised. Vehicle-specific aerodynamics, control response, "
            "terminal accuracy, and promotion evidence remain outside this fixture-level contract."
        ),
    )
    ####


def _simple_aero_fidelity(tier: str) -> TrajectoryFidelityMetadata:
    declared = tier == _POINT_MASS
    labels = {
        "point_mass_3dof": "Point-Mass 3-DOF",
        "pseudo_6dof": "Pseudo 6-DOF",
        "rigid_body_6dof_direct_wrench": "Rigid-Body 6-DOF, Direct Wrench",
        "rigid_body_6dof_surface_allocated": "Rigid-Body 6-DOF, Surface Allocated",
    }
    return TrajectoryFidelityMetadata(
        id=tier,
        label=labels[tier],
        rank=CANONICAL_FIDELITY_TIERS.index(tier),
        declared=declared,
        dynamics_fidelity=(
            "point_mass_3dof"
            if tier == "point_mass_3dof"
            else "pseudo_6dof"
            if tier == "pseudo_6dof"
            else "rigid_body_6dof"
        ),
        input_realization=(
            "guidance_command"
            if tier in {"point_mass_3dof", "pseudo_6dof"}
            else "direct_wrench"
            if tier == "rigid_body_6dof_direct_wrench"
            else "actuator_allocated"
        ),
        actuator_types=("aerodynamic_surfaces",) if tier == "rigid_body_6dof_surface_allocated" else ("not_applicable",),
        compatibility_aliases=(tier,),
        runtime_fidelity="point_mass_3dof" if declared else "not_available",
        control_realization="generated segment commands" if declared else "not_available",
        promotion_status="synthetic_contract_fixture" if declared else "not_declared",
        operations=("validate", "batch") if declared else (),
        profile_id="simple_aero.fixed_ld_3dof" if declared else None,
        blockers=() if declared else ("Simple Aero workflow does not declare this realization tier",),
        required_operations=("configuration_validation", "generated_problem_batch") if declared else (),
        claim_boundary=(
            "Point-mass readiness applies to the synthetic fixed-coefficient workflow only."
            if declared
            else "An undeclared tier cannot be inferred from the reusable segment vocabulary."
        ),
    )
    ####


def _unavailable_transition(
    source: str,
    target: str,
    *,
    direction: Literal["step_up", "step_down"],
) -> TrajectoryFidelityTransition:
    return TrajectoryFidelityTransition(
        from_fidelity=source,
        to_fidelity=target,
        direction=direction,
        status="not_available",
        automatic=False,
        selection_policy="explicit_upgrade_only" if direction == "step_up" else "exact_only",
        requirements=("both adjacent Simple Aero fidelity realizations must be declared and evidenced",),
        state_transfer="not_advertised; start a newly prepared run after a target fidelity is implemented",
        claim_boundary="Reusable segment names do not establish a fidelity projection, lowering path, or live state transfer.",
    )
    ####


def _label(identifier: str) -> str:
    return identifier.replace(".", " ").replace("_", " ").title()
    ####


__all__ = [
    "SIMPLE_AERO_MODEL_ID",
    "build_simple_aero_example_configuration",
    "build_simple_aero_prepared_configuration",
    "simple_aero_configuration_schema",
    "simple_aero_model_metadata",
]
