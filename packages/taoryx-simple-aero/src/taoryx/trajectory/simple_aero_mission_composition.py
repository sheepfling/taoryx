"""Mission Composition advertisement for the Simple Aero workflow.

Simple Aero is a configurable trajectory workflow rather than a physical
vehicle family.  This module projects its checked-in Alpha 2 parameter family,
specialized segment contracts, and source-shaped fixture vocabulary into the
portable Mission Composition configuration grammar.  It deliberately keeps
fixture readiness separate from vehicle promotion and runtime availability.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from functools import lru_cache
from typing import Any, Literal, cast

from pydantic import BaseModel, ConfigDict, Field, model_validator
from taoryx_simple_aero.provider import ReferencePointMassSession
from taoryx_simple_aero.resources import simple_aero_resource_root

from ..fidelity_contracts import CANONICAL_FIDELITY_TIERS
from ..fixture_composition_episode import FixtureCompositionEpisode, FixtureTransition
from ..simple_aero_builder import FixedLD3DOFParameters, SimpleAeroTrajectoryBuild, SimpleAeroTrajectoryBuilder
from ..specialized_segments import SIMPLE_AERO_FAMILY_SEGMENTS, specialized_segment_contract
from ..value_space import default_value_space_for_value_type
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
    PreparedTrajectoryConfiguration,
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
    TrajectoryOpenSegmentSequenceMetadata,
    TrajectoryOutputChannelMetadata,
    TrajectoryOutputSchema,
    TrajectoryRealizationMetadata,
    TrajectoryReferenceFrameMetadata,
    TrajectoryTelemetryGroupMetadata,
    ValuePresentationMetadata,
    validate_configuration_instance,
)
from .contracts import ControlFrame, FamilyPackage, ParameterSchema
from .session_interface import build_session_interface_contract

SIMPLE_AERO_MODEL_ID = "simple_aero"
SIMPLE_AERO_CUSTOM_COMPOSITION_ID = "custom_composition"
_ROOT = simple_aero_resource_root()
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

SimpleAeroSegmentKind = Literal[
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
]
SimpleAeroCutoffCondition = Literal["physical_burnout", "commanded_burnout_speed"]
SimpleAeroEarthModel = Literal["standard_wgs84", "vacuum_spherical"]

_SEGMENT_PARAMETER_UNITS: dict[SimpleAeroSegmentKind, dict[str, str | None]] = {
    "powered_ascent": {"duration_s": "s", "cutoff_condition": None},
    "ballistic_coast": {"duration_s": "s", "alpha_deg": "deg"},
    "bank_maneuver": {"duration_s": "s", "bank_deg": "deg"},
    "cbcr": {
        "go_left": None,
        "maneuver_altitude_start_m": "m",
        "duration_s": "s",
        "minimum_time_to_go_s": "s",
    },
    "crossrange": {"initial_heading_error_deg": "deg", "minimum_time_to_go_s": "s"},
    "marv": {"maneuver_begin_time_to_go_s": "s", "minimum_time_to_go_s": "s"},
    "phugoid": {
        "start_range_to_go_m": "m",
        "amplitude_deg": "deg",
        "frequency_hz": "Hz",
        "maneuver_roll_deg": "deg",
    },
    "range_extension": {"minimum_time_to_go_s": "s"},
    "skip": {"maneuver_begin_time_to_go_s": "s"},
    "slalom": {
        "start_range_to_go_m": "m",
        "end_range_to_go_m": "m",
        "minimum_time_to_go_s": "s",
    },
    "weave": {"end_range_to_go_m": "m", "minimum_time_to_go_s": "s"},
    "terminal_pronav": {"duration_s": "s", "capture_range_m": "m"},
}
_OPTIONAL_SEGMENT_PARAMETERS = frozenset(
    {
        ("phugoid", "start_range_to_go_m"),
        ("slalom", "start_range_to_go_m"),
        ("weave", "end_range_to_go_m"),
    }
)


def _require_finite(value: float, name: str) -> None:
    """Reject non-finite convenience inputs before configuration assembly."""

    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    ####


def _canonical_longitude_deg(value: float) -> float:
    """Return a longitude in the portable grammar's ``[-180, 180)`` interval."""

    return (value + 180.0) % 360.0 - 180.0
    ####


def _canonical_signed_heading_deg(value: float) -> float:
    """Return a signed heading in the portable grammar's ``[-180, 180)`` interval."""

    return (value + 180.0) % 360.0 - 180.0
    ####


class SimpleAeroLaunch(BaseModel):
    """Friendly SI launch state for one Simple Aero mission."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    latitude_deg: float = Field(default=0.0, ge=-90.0, le=90.0)
    longitude_deg: float = 0.0
    altitude_m: float = Field(default=0.0, ge=0.0)
    speed_m_s: float = Field(default=50.0, ge=0.0)
    pitch_over_angle_deg: float = Field(default=80.0, ge=-89.0, le=89.0)
    initial_heading_offset_deg: float = Field(default=0.0, ge=-180.0, le=180.0)

    @model_validator(mode="after")
    def validate_launch(self) -> SimpleAeroLaunch:
        for name, value in self.__dict__.items():
            _require_finite(float(value), name)
        object.__setattr__(self, "longitude_deg", _canonical_longitude_deg(self.longitude_deg))
        return self
        ####

    ####


class SimpleAeroRangeBearingEndpoint(BaseModel):
    """Endpoint expressed as spherical-Earth surface range and initial bearing."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    target_range_m: float = Field(default=100_000.0, ge=0.0)
    target_bearing_deg: float = 90.0

    @model_validator(mode="after")
    def validate_endpoint(self) -> SimpleAeroRangeBearingEndpoint:
        _require_finite(self.target_range_m, "target_range_m")
        _require_finite(self.target_bearing_deg, "target_bearing_deg")
        object.__setattr__(self, "target_bearing_deg", self.target_bearing_deg % 360.0)
        return self
        ####

    ####


class SimpleAeroGeodeticAimpoint(BaseModel):
    """Endpoint expressed directly as a geodetic aimpoint."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    latitude_deg: float = Field(default=0.0, ge=-90.0, le=90.0)
    longitude_deg: float = 0.0

    @model_validator(mode="after")
    def validate_aimpoint(self) -> SimpleAeroGeodeticAimpoint:
        _require_finite(self.latitude_deg, "latitude_deg")
        _require_finite(self.longitude_deg, "longitude_deg")
        object.__setattr__(self, "longitude_deg", _canonical_longitude_deg(self.longitude_deg))
        return self
        ####

    ####


SimpleAeroEndpoint = SimpleAeroRangeBearingEndpoint | SimpleAeroGeodeticAimpoint


class SimpleAeroEndpointState(BaseModel):
    """Optional terminal target motion assigned by the fixed-L/D builder."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    altitude_m: float = Field(default=0.0, ge=0.0)
    speed_m_s: float = Field(default=0.0, ge=0.0)
    heading_deg: float = 0.0

    @model_validator(mode="after")
    def validate_endpoint_state(self) -> SimpleAeroEndpointState:
        _require_finite(self.altitude_m, "altitude_m")
        _require_finite(self.speed_m_s, "speed_m_s")
        _require_finite(self.heading_deg, "heading_deg")
        object.__setattr__(self, "heading_deg", _canonical_signed_heading_deg(self.heading_deg))
        return self
        ####

    ####


class SimpleAeroSurrogate(BaseModel):
    """Fixed-coefficient point-mass surrogate inputs, all in canonical SI units."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    vehicle_id: str = Field(default="generic-3dof", min_length=1)
    initial_mass_kg: float = Field(default=1000.0, ge=1.0)
    thrust_n: float = Field(default=100_000.0, ge=0.0)
    mass_flow_kg_s: float = Field(default=20.0, ge=0.0)
    drag_coefficient: float = Field(default=0.02, ge=0.0)
    lift_to_drag: float = Field(default=4.0, ge=0.0)

    @model_validator(mode="after")
    def validate_surrogate(self) -> SimpleAeroSurrogate:
        if not self.vehicle_id.strip():
            raise ValueError("vehicle_id must contain non-whitespace text")
        for name, value in self.__dict__.items():
            if name != "vehicle_id":
                _require_finite(float(value), name)
        return self
        ####

    ####


class SimpleAeroCheckpoints(BaseModel):
    """Convenience checkpoint targets for the reduced-order builder."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    burnout_speed_m_s: float = Field(default=1200.0, gt=0.0)
    apogee_altitude_m: float = Field(default=80_000.0, gt=0.0)

    @model_validator(mode="after")
    def validate_checkpoints(self) -> SimpleAeroCheckpoints:
        _require_finite(self.burnout_speed_m_s, "burnout_speed_m_s")
        _require_finite(self.apogee_altitude_m, "apogee_altitude_m")
        return self
        ####

    ####


class SimpleAeroRuntime(BaseModel):
    """Numerical and environment inputs for the generated batch problem."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    time_step_s: float = Field(default=0.05, gt=0.0)
    output_interval_s: float = Field(default=0.5, gt=0.0)
    earth_model: SimpleAeroEarthModel = "standard_wgs84"

    @model_validator(mode="after")
    def validate_runtime(self) -> SimpleAeroRuntime:
        _require_finite(self.time_step_s, "time_step_s")
        _require_finite(self.output_interval_s, "output_interval_s")
        return self
        ####

    ####


class SimpleAeroSegment(BaseModel):
    """One source-shaped Simple Aero segment occurrence.

    Use the named constructors instead of assembling ``parameters`` manually.
    They expose every published source vocabulary parameter while this type
    retains segment identity for an arbitrary ordered mission sequence.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: SimpleAeroSegmentKind
    instance_id: str | None = Field(default=None, min_length=1)
    parameters: dict[str, float | str | bool] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_segment(self) -> SimpleAeroSegment:
        if self.instance_id is not None and not self.instance_id.strip():
            raise ValueError("instance_id must contain non-whitespace text")
        units = _SEGMENT_PARAMETER_UNITS[self.kind]
        unknown = set(self.parameters).difference(units)
        if unknown:
            raise ValueError(f"{self.kind} does not accept parameters {sorted(unknown)!r}")
        for parameter_id, value in self.parameters.items():
            if parameter_id == "go_left":
                if not isinstance(value, bool):
                    raise ValueError("go_left must be boolean")
            elif parameter_id == "cutoff_condition":
                if value not in {"physical_burnout", "commanded_burnout_speed"}:
                    raise ValueError("cutoff_condition must select a published cutoff")
            else:
                if isinstance(value, bool) or not isinstance(value, int | float):
                    raise ValueError(f"{parameter_id} must be numeric")
                _require_finite(float(value), parameter_id)
        return self
        ####

    ####

    @classmethod
    def powered_ascent(
        cls,
        *,
        duration_s: float = 5.0,
        cutoff_condition: SimpleAeroCutoffCondition = "physical_burnout",
        instance_id: str | None = None,
    ) -> SimpleAeroSegment:
        """Create the thrust-driven segment and its declared cutoff mode."""

        return cls(
            kind="powered_ascent",
            instance_id=instance_id,
            parameters={"duration_s": duration_s, "cutoff_condition": cutoff_condition},
        )
        ####

    @classmethod
    def ballistic_coast(cls, *, duration_s: float = 10.0, alpha_deg: float = 0.0, instance_id: str | None = None) -> SimpleAeroSegment:
        """Create a coast with an explicit constant alpha input."""

        return cls(kind="ballistic_coast", instance_id=instance_id, parameters={"duration_s": duration_s, "alpha_deg": alpha_deg})
        ####

    @classmethod
    def bank_maneuver(cls, *, duration_s: float = 5.0, bank_deg: float = 0.0, instance_id: str | None = None) -> SimpleAeroSegment:
        """Create the generic bounded bank segment."""

        return cls(kind="bank_maneuver", instance_id=instance_id, parameters={"duration_s": duration_s, "bank_deg": bank_deg})
        ####

    @classmethod
    def cbcr(
        cls,
        *,
        go_left: bool = True,
        maneuver_altitude_start_m: float = 40_000.0,
        duration_s: float = 35.0,
        minimum_time_to_go_s: float = 60.0,
        instance_id: str | None = None,
    ) -> SimpleAeroSegment:
        """Create the source-shaped CBCR vocabulary occurrence."""

        return cls(
            kind="cbcr",
            instance_id=instance_id,
            parameters={
                "go_left": go_left,
                "maneuver_altitude_start_m": maneuver_altitude_start_m,
                "duration_s": duration_s,
                "minimum_time_to_go_s": minimum_time_to_go_s,
            },
        )
        ####

    @classmethod
    def crossrange(
        cls,
        *,
        initial_heading_error_deg: float = 15.0,
        minimum_time_to_go_s: float = 60.0,
        instance_id: str | None = None,
    ) -> SimpleAeroSegment:
        """Create the source-shaped crossrange vocabulary occurrence."""

        return cls(
            kind="crossrange",
            instance_id=instance_id,
            parameters={"initial_heading_error_deg": initial_heading_error_deg, "minimum_time_to_go_s": minimum_time_to_go_s},
        )
        ####

    @classmethod
    def marv(
        cls,
        *,
        maneuver_begin_time_to_go_s: float = 60.0,
        minimum_time_to_go_s: float = 30.0,
        instance_id: str | None = None,
    ) -> SimpleAeroSegment:
        """Create the source-shaped MARV vocabulary occurrence."""

        return cls(
            kind="marv",
            instance_id=instance_id,
            parameters={
                "maneuver_begin_time_to_go_s": maneuver_begin_time_to_go_s,
                "minimum_time_to_go_s": minimum_time_to_go_s,
            },
        )
        ####

    @classmethod
    def phugoid(
        cls,
        *,
        start_range_to_go_m: float | None = None,
        amplitude_deg: float = 2.0,
        frequency_hz: float = 0.015,
        maneuver_roll_deg: float = 45.0,
        instance_id: str | None = None,
    ) -> SimpleAeroSegment:
        """Create the source-shaped alpha-profile phugoid occurrence."""

        return cls(
            kind="phugoid",
            instance_id=instance_id,
            parameters={
                **({"start_range_to_go_m": start_range_to_go_m} if start_range_to_go_m is not None else {}),
                "amplitude_deg": amplitude_deg,
                "frequency_hz": frequency_hz,
                "maneuver_roll_deg": maneuver_roll_deg,
            },
        )
        ####

    @classmethod
    def range_extension(cls, *, minimum_time_to_go_s: float = 60.0, instance_id: str | None = None) -> SimpleAeroSegment:
        """Create the source-shaped range-extension occurrence."""

        return cls(kind="range_extension", instance_id=instance_id, parameters={"minimum_time_to_go_s": minimum_time_to_go_s})
        ####

    @classmethod
    def skip(cls, *, maneuver_begin_time_to_go_s: float = 400.0, instance_id: str | None = None) -> SimpleAeroSegment:
        """Create the source-shaped skip occurrence."""

        return cls(kind="skip", instance_id=instance_id, parameters={"maneuver_begin_time_to_go_s": maneuver_begin_time_to_go_s})
        ####

    @classmethod
    def slalom(
        cls,
        *,
        start_range_to_go_m: float | None = None,
        end_range_to_go_m: float = 200_000.0,
        minimum_time_to_go_s: float = 60.0,
        instance_id: str | None = None,
    ) -> SimpleAeroSegment:
        """Create the source-shaped slalom occurrence."""

        return cls(
            kind="slalom",
            instance_id=instance_id,
            parameters={
                **({"start_range_to_go_m": start_range_to_go_m} if start_range_to_go_m is not None else {}),
                "end_range_to_go_m": end_range_to_go_m,
                "minimum_time_to_go_s": minimum_time_to_go_s,
            },
        )
        ####

    @classmethod
    def weave(
        cls,
        *,
        end_range_to_go_m: float | None = None,
        minimum_time_to_go_s: float = 60.0,
        instance_id: str | None = None,
    ) -> SimpleAeroSegment:
        """Create the source-shaped weave occurrence."""

        return cls(
            kind="weave",
            instance_id=instance_id,
            parameters={
                **({"end_range_to_go_m": end_range_to_go_m} if end_range_to_go_m is not None else {}),
                "minimum_time_to_go_s": minimum_time_to_go_s,
            },
        )
        ####

    @classmethod
    def terminal_pronav(
        cls,
        *,
        duration_s: float = 5.0,
        capture_range_m: float = 25.0,
        instance_id: str | None = None,
    ) -> SimpleAeroSegment:
        """Create the terminal proportional-navigation vocabulary occurrence."""

        return cls(
            kind="terminal_pronav",
            instance_id=instance_id,
            parameters={"duration_s": duration_s, "capture_range_m": capture_range_m},
        )
        ####


class SimpleAeroMission(BaseModel):
    """A complete ergonomic Simple Aero composition over the advertised tree.

    ``operation_template_id`` selects the published batch-operation binding.
    It does not rewrite or restrict ``segments``: the caller-authored sequence
    remains the exact source sequence given to the fixed-L/D lowering.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    configuration_id: str = Field(default="simple-aero-mission", min_length=1)
    launch: SimpleAeroLaunch = Field(default_factory=SimpleAeroLaunch)
    endpoint: SimpleAeroEndpoint = Field(default_factory=SimpleAeroRangeBearingEndpoint)
    endpoint_state: SimpleAeroEndpointState = Field(default_factory=SimpleAeroEndpointState)
    surrogate: SimpleAeroSurrogate = Field(default_factory=SimpleAeroSurrogate)
    checkpoints: SimpleAeroCheckpoints = Field(default_factory=SimpleAeroCheckpoints)
    runtime: SimpleAeroRuntime = Field(default_factory=SimpleAeroRuntime)
    segments: tuple[SimpleAeroSegment, ...]
    operation_template_id: str = Field(default=SIMPLE_AERO_CUSTOM_COMPOSITION_ID, min_length=1)

    @model_validator(mode="after")
    def validate_mission(self) -> SimpleAeroMission:
        if not self.configuration_id.strip():
            raise ValueError("configuration_id must contain non-whitespace text")
        if not 1 <= len(self.segments) <= 32:
            raise ValueError("segments must contain between 1 and 32 occurrences")
        instance_ids = tuple(item.instance_id for item in self.segments if item.instance_id is not None)
        if len(instance_ids) != len(set(instance_ids)):
            raise ValueError("segment instance_id values must be unique")
        if not self.operation_template_id.strip():
            raise ValueError("operation_template_id must contain non-whitespace text")
        return self
        ####

    ####

    @classmethod
    def template(
        cls,
        mission_template_id: str,
        *,
        configuration_id: str | None = None,
        launch: SimpleAeroLaunch | None = None,
        endpoint: SimpleAeroEndpoint | None = None,
        endpoint_state: SimpleAeroEndpointState | None = None,
        surrogate: SimpleAeroSurrogate | None = None,
        checkpoints: SimpleAeroCheckpoints | None = None,
        runtime: SimpleAeroRuntime | None = None,
    ) -> SimpleAeroMission:
        """Materialize a published template as named segment occurrences.

        The materialized occurrences intentionally leave their parameter maps
        empty, so the portable schema remains the authority for defaults.
        Replace an occurrence with a named ``SimpleAeroSegment`` constructor
        when a template parameter needs to change.
        """

        return cls(
            configuration_id=configuration_id or f"simple-aero-{mission_template_id}",
            launch=launch or SimpleAeroLaunch(),
            endpoint=endpoint or SimpleAeroRangeBearingEndpoint(),
            endpoint_state=endpoint_state or SimpleAeroEndpointState(),
            surrogate=surrogate or SimpleAeroSurrogate(),
            checkpoints=checkpoints or SimpleAeroCheckpoints(),
            runtime=runtime or SimpleAeroRuntime(),
            segments=tuple(SimpleAeroSegment(kind=kind) for kind in _template_segment_ids(mission_template_id)),
            operation_template_id=mission_template_id,
        )
        ####


def build_simple_aero_configuration(
    mission: SimpleAeroMission,
    *,
    schema: TrajectoryConfigurationSchema | None = None,
) -> TrajectoryConfigurationInstance:
    """Build the exact portable configuration tree for an ergonomic mission.

    This is the inverse of an advertisement renderer, not a parallel mission
    format. Every typed convenience field below maps to one published schema
    node, preserving choice, optional, sequence, unit, and instance identity
    semantics for generic API consumers.
    """

    if not isinstance(mission, SimpleAeroMission):
        raise TypeError("mission must be a SimpleAeroMission")
    resolved_schema = schema or simple_aero_configuration_schema()
    if resolved_schema.model_id != SIMPLE_AERO_MODEL_ID:
        raise ValueError(f"expected Simple Aero schema, received {resolved_schema.model_id!r}")
    template_ids = {item.id for item in _segments(resolved_schema).templates} | {SIMPLE_AERO_CUSTOM_COMPOSITION_ID}
    if mission.operation_template_id not in template_ids:
        raise KeyError(f"unknown Simple Aero operation template {mission.operation_template_id!r}")
    if isinstance(mission.endpoint, SimpleAeroRangeBearingEndpoint):
        endpoint = ConfigurationChoiceValue(
            selected="range_bearing",
            value=ConfigurationGroupValue(
                values={
                    "mission.target_range": _simple_aero_parameter(mission.endpoint.target_range_m, "m"),
                    "mission.target_bearing": _simple_aero_parameter(mission.endpoint.target_bearing_deg, "deg"),
                }
            ),
        )
    else:
        endpoint = ConfigurationChoiceValue(
            selected="geodetic_aimpoint",
            value=ConfigurationGroupValue(
                values={
                    "aimpoint.latitude_deg": _simple_aero_parameter(mission.endpoint.latitude_deg, "deg"),
                    "aimpoint.longitude_deg": _simple_aero_parameter(mission.endpoint.longitude_deg, "deg"),
                }
            ),
        )
    return TrajectoryConfigurationInstance(
        configuration_id=mission.configuration_id,
        model_id=SIMPLE_AERO_MODEL_ID,
        model_version=resolved_schema.model_version,
        schema_fingerprint=resolved_schema.fingerprint,
        fidelity=_POINT_MASS,
        realization_id="fixed_ld_point_mass",
        mission_template_id=mission.operation_template_id,
        root=ConfigurationGroupValue(
            values={
                "launch_state": ConfigurationGroupValue(
                    values={
                        "launch.latitude_deg": _simple_aero_parameter(mission.launch.latitude_deg, "deg"),
                        "launch.longitude_deg": _simple_aero_parameter(mission.launch.longitude_deg, "deg"),
                        "mission.initial_altitude": _simple_aero_parameter(mission.launch.altitude_m, "m"),
                        "mission.initial_speed": _simple_aero_parameter(mission.launch.speed_m_s, "m/s"),
                        "mission.pitch_over_angle": _simple_aero_parameter(mission.launch.pitch_over_angle_deg, "deg"),
                        "mission.initial_heading_offset": _simple_aero_parameter(mission.launch.initial_heading_offset_deg, "deg"),
                    }
                ),
                "endpoint": endpoint,
                "endpoint_state": ConfigurationGroupValue(
                    values={
                        "mission.target_altitude": _simple_aero_parameter(mission.endpoint_state.altitude_m, "m"),
                        "mission.target_speed": _simple_aero_parameter(mission.endpoint_state.speed_m_s, "m/s"),
                        "mission.target_heading": _simple_aero_parameter(mission.endpoint_state.heading_deg, "deg"),
                    }
                ),
                "vehicle_surrogate": ConfigurationGroupValue(
                    values={
                        "vehicle.id": _simple_aero_parameter(mission.surrogate.vehicle_id, None),
                        "vehicle.mass.initial": _simple_aero_parameter(mission.surrogate.initial_mass_kg, "kg"),
                        "vehicle.booster.thrust": _simple_aero_parameter(mission.surrogate.thrust_n, "N"),
                        "vehicle.booster.mass_flow": _simple_aero_parameter(mission.surrogate.mass_flow_kg_s, "kg/s"),
                        "vehicle.aero.drag_coefficient": _simple_aero_parameter(mission.surrogate.drag_coefficient, "dimensionless"),
                        "vehicle.aero.lift_to_drag": _simple_aero_parameter(mission.surrogate.lift_to_drag, "dimensionless"),
                    }
                ),
                "trajectory_checkpoints": ConfigurationGroupValue(
                    values={
                        "mission.burnout_speed": _simple_aero_parameter(mission.checkpoints.burnout_speed_m_s, "m/s"),
                        "mission.apogee_altitude": _simple_aero_parameter(mission.checkpoints.apogee_altitude_m, "m"),
                    }
                ),
                "segments": ConfigurationSequenceValue(
                    items=tuple(_simple_aero_segment_value(segment, index) for index, segment in enumerate(mission.segments, start=1))
                ),
                "runtime": ConfigurationGroupValue(
                    values={
                        "runtime.time_step": _simple_aero_parameter(mission.runtime.time_step_s, "s"),
                        "runtime.output_interval": _simple_aero_parameter(mission.runtime.output_interval_s, "s"),
                        "earth_model": _simple_aero_parameter(mission.runtime.earth_model, None),
                    }
                ),
            }
        ),
    )
    ####


def prepare_simple_aero_mission(
    mission: SimpleAeroMission,
    *,
    schema: TrajectoryConfigurationSchema | None = None,
) -> PreparedTrajectoryConfiguration:
    """Build and validate an ergonomic mission without a provider wrapper."""

    resolved_schema = schema or simple_aero_configuration_schema()
    return validate_configuration_instance(
        resolved_schema,
        build_simple_aero_configuration(mission, schema=resolved_schema),
    )
    ####


def _simple_aero_parameter(value: object, unit: str | None) -> ConfigurationParameterValue:
    """Return one explicit canonical parameter leaf."""

    return ConfigurationParameterValue(value=value, unit=unit)
    ####


def _simple_aero_segment_value(segment: SimpleAeroSegment, index: int) -> ConfigurationChoiceValue:
    """Project one named convenience segment onto its exact choice subtree."""

    values: dict[str, ConfigurationNodeValue] = {}
    for parameter_id, value in segment.parameters.items():
        unit = _SEGMENT_PARAMETER_UNITS[segment.kind][parameter_id]
        parameter = _simple_aero_parameter(value, unit)
        if (segment.kind, parameter_id) in _OPTIONAL_SEGMENT_PARAMETERS:
            values[parameter_id] = ConfigurationOptionalValue(enabled=True, value=parameter)
        else:
            values[parameter_id] = parameter
    return ConfigurationChoiceValue(
        selected=segment.kind,
        instance_id=segment.instance_id or f"{segment.kind}-{index:02d}",
        value=ConfigurationGroupValue(values=values),
    )
    ####


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
    missions = (*(_mission_metadata(template) for template in sequence.templates), _custom_composition_metadata())
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
        operations=("discover", "validate", "batch", "step"),
        common_runner_operations=("batch", "step"),
        capabilities=TrajectoryModelCapabilities(
            initialization_modes=("launch_state", "range_bearing", "geodetic_aimpoint"),
            segment_types=_segment_names(),
            termination_modes=("time", "physical_burnout", "commanded_heading", "commanded_range"),
            operations=("discover", "validate", "batch", "step"),
            supports_custom_segments=True,
        ),
        realizations=(
            TrajectoryRealizationMetadata(
                id="fixed_ld_point_mass",
                label="Fixed-L/D Point-Mass",
                description=("Generated native point-mass realization for fixed-L/D baseline and source-shaped parameter-mapped maneuver templates."),
                status="available",
                dynamics_fidelities=("point_mass_3dof",),
                input_realization="guidance_command",
                controls=_simple_aero_control_advertisement(family, missions),
                fidelity_aliases=(_POINT_MASS,),
                mission_template_ids=tuple(item.id for item in missions),
                operations=("validate", "batch", "step"),
                native_factory_ids=("simple_aero_generated_problem.v1", "simple_aero_point_mass_session.v1"),
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
            "This is a configurable workflow and contract fixture, not a physical vehicle. Every declared point-mass "
            "template lowers through the fixed-L/D builder, but source-shaped maneuver parameters become bounded synthetic "
            "alpha/bank/time profiles. That execution does not validate a historical Simple Aero runtime, vehicle-specific "
            "aerodynamics, optimization, terminal accuracy, or a physical control system."
        ),
    )
    ####


def _simple_aero_control_advertisement(
    family: FamilyPackage,
    missions: tuple[TrajectoryMissionTemplateMetadata, ...],
) -> TrajectoryControlAdvertisement:
    """Publish generated control coordinates separately from caller inputs."""

    value_space = ConfigurationValueSpace.from_mapping(default_value_space_for_value_type("scalar").as_dict())
    generated_channels = tuple(
        TrajectoryControlChannelMetadata(
            id=item.id,
            label=item.id.replace(".", " ").replace("_", " ").title(),
            description=item.description or f"Generated {item.id} command used by the fixed-L/D workflow.",
            channel_kind="action",
            quantity=_QUANTITY_BY_UNIT.get(item.unit or ""),
            canonical_unit=item.unit,
            display_unit=item.unit,
            interval=(
                ConfigurationInterval(
                    minimum=ConfigurationBound(value=item.minimum) if item.minimum is not None else None,
                    maximum=ConfigurationBound(value=item.maximum) if item.maximum is not None else None,
                )
                if item.minimum is not None or item.maximum is not None
                else None
            ),
            sampling_semantics="batch_profile",
            value_space=value_space,
            availability="available_in_batch",
            operations=("batch",),
            native_channel_id=item.id,
            native_binding=TrajectoryControlNativeBindingMetadata(
                id=item.id,
                quantity=_QUANTITY_BY_UNIT.get(item.unit or ""),
                canonical_unit=item.unit,
                interval=(
                    ConfigurationInterval(
                        minimum=ConfigurationBound(value=item.minimum) if item.minimum is not None else None,
                        maximum=ConfigurationBound(value=item.maximum) if item.maximum is not None else None,
                    )
                    if item.minimum is not None or item.maximum is not None
                    else None
                ),
                value_space=value_space,
                provider_binding={"generated_problem_variable": item.id},
            ),
            provider_binding={"generated_problem_variable": item.id},
            presentation=ValuePresentationMetadata(group="controls", order=index * 10, control="slider"),
            source_refs=("verification/alpha2_family_catalog.yaml", "src/taoryx/simple_aero_builder.py"),
            provenance="Simple Aero generated native problem",
            claim_boundary="Provider-generated command only; it is not an externally allocated physical actuator.",
        )
        for index, item in enumerate(family.controls, start=1)
    )
    direct_space = ConfigurationValueSpace(
        topology="bounded_interval",
        representation="scalar",
        error_rule="subtraction",
        interpolation_rule="held constant",
        normalization_rule="bounded to [0, 1]",
        coordinate_chart="[0, 1]",
    )
    direct_throttle = TrajectoryControlChannelMetadata(
        id="propulsion.command.fraction",
        label="Direct Throttle Fraction",
        description="Caller-held normalized throttle for the native interactive point-mass provider.",
        channel_kind="action",
        quantity="dimensionless",
        canonical_unit="dimensionless",
        display_unit="dimensionless",
        interval=ConfigurationInterval(
            minimum=ConfigurationBound(value=0.0),
            maximum=ConfigurationBound(value=1.0),
        ),
        sampling_semantics="held_action",
        value_space=direct_space,
        semantics=ControlCommandSemantics(
            value_domain="continuous",
            command_mode="absolute",
            temporal_semantics="held",
            release_behavior="hold",
            agent_normalization="affine",
            agent_clip=True,
        ),
        availability="available",
        operations=("step",),
        native_channel_id="command.throttle",
        native_binding=TrajectoryControlNativeBindingMetadata(
            id="command.throttle",
            quantity="dimensionless",
            canonical_unit="dimensionless",
            interval=ConfigurationInterval(
                minimum=ConfigurationBound(value=0.0),
                maximum=ConfigurationBound(value=1.0),
            ),
            value_space=direct_space,
            provider_binding={
                "provider": "ReferencePointMassSession",
                "field": "command.throttle",
                "feedback_channel_id": "propulsion.throttle_command",
            },
        ),
        provider_binding={
            "provider": "ReferencePointMassSession",
            "field": "command.throttle",
            "feedback_channel_id": "propulsion.throttle_command",
        },
        presentation=ValuePresentationMetadata(group="interactive controls", order=10, control="slider"),
        source_refs=("packages/taoryx-simple-aero/src/taoryx_simple_aero/provider.py",),
        provenance="existing Simple Aero reference point-mass interactive provider",
        claim_boundary=(
            "Direct normalized thrust scaling only. The interactive proof has no bank steering, "
            "aerodynamic response, atmosphere, or vehicle-specific engine dynamics."
        ),
    )
    channels = (*generated_channels, direct_throttle)
    mission_ids = tuple(item.id for item in missions)
    segment_ids = {segment for mission in missions for segment in mission.advertised_segment_ids}
    intents = (
        TrajectoryControlIntentMetadata(
            id="bank",
            label="Bank",
            description="Generated bank-profile intent used by maneuver and fixed-L/D segments.",
            resolution="provider_internal",
            segment_ids=tuple(item for item in _segment_names() if item in segment_ids and item not in {"powered_ascent", "ballistic_coast"}),
            mission_template_ids=mission_ids,
            channel_ids=tuple(item.id for item in channels if item.id == "command.bank"),
            operations=("batch",),
            source_refs=("verification/simple_aero_segment_catalog.yaml",),
            provenance="Simple Aero segment grammar",
            claim_boundary="Intent is resolved by generated workflow commands, not by a caller-owned feedback controller.",
        ),
        TrajectoryControlIntentMetadata(
            id="throttle",
            label="Throttle",
            description="Generated powered-ascent throttle intent.",
            resolution="provider_internal",
            segment_ids=("powered_ascent",),
            mission_template_ids=tuple(item.id for item in missions if "powered_ascent" in item.advertised_segment_ids),
            channel_ids=tuple(item.id for item in channels if item.id == "command.throttle"),
            operations=("batch",),
            source_refs=("verification/simple_aero_segment_catalog.yaml",),
            provenance="Simple Aero segment grammar",
            claim_boundary="Synthetic point-mass propulsion command only; no physical throttle or engine qualification claim.",
        ),
        TrajectoryControlIntentMetadata(
            id="direct_throttle",
            label="Direct Throttle",
            description="Hold a normalized throttle fraction in the stateful point-mass session.",
            resolution="external_channel",
            segment_ids=_segment_names(),
            mission_template_ids=mission_ids,
            channel_ids=("propulsion.command.fraction",),
            operations=("step",),
            source_refs=("packages/taoryx-simple-aero/src/taoryx_simple_aero/provider.py",),
            provenance="existing Simple Aero interactive provider",
            claim_boundary="Normalized point-mass thrust scaling only; no engine or flight-control qualification.",
        ),
    )
    return TrajectoryControlAdvertisement(
        status="available",
        channels=channels,
        authorities=(
            TrajectoryControlAuthorityMetadata(
                id="generated_mission_commands",
                authority="mission",
                availability="available",
                channel_ids=tuple(item.id for item in generated_channels),
                operations=("batch", "step"),
                description="Provider-generated bank and throttle profiles for batch and zero-action session stepping.",
                command_owner="provider_controller",
                selection_scope="session",
                switching_policy="explicit_bumpless",
                scheme_id="provider.program",
                lowering_chain=("simple_aero_segment_program", "generated_bank_throttle_profile"),
                source_refs=("src/taoryx/simple_aero_builder.py",),
                provenance="Simple Aero builder",
                claim_boundary=(
                    "Provider-owned schedule. Session bank is advertised and recorded but deliberately has no steering "
                    "effect in the existing point-mass interactive proof."
                ),
            ),
            TrajectoryControlAuthorityMetadata(
                id="direct_throttle_command",
                authority="native_bridge",
                availability="available",
                channel_ids=("propulsion.command.fraction",),
                operations=("step",),
                description="Caller-owned normalized throttle mapped to the native command.throttle channel.",
                command_owner="caller",
                selection_scope="session",
                switching_policy="explicit_bumpless",
                scheme_id="kinematic.energy",
                lowering_chain=("propulsion.command.fraction", "command.throttle", "point_mass_thrust_scaling"),
                source_refs=("packages/taoryx-simple-aero/src/taoryx_simple_aero/provider.py",),
                provenance="existing Simple Aero interactive provider",
                claim_boundary="Direct point-mass thrust scaling only; bank remains provider-generated and non-steering.",
            ),
        ),
        intents=intents,
        default_authority_id="generated_mission_commands",
        claim_boundary=(
            "Generated mission commands and direct throttle share one stateful session. Only direct throttle is caller-owned; "
            "generated bank remains visible but is not promoted to a steering control."
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
            operations=(
                ("batch", "step")
                if channel_id
                in {
                    "position.geodetic.altitude",
                    "velocity.speed",
                    "mass.total",
                    "propulsion.thrust",
                    "propulsion.throttle_command",
                }
                else ("batch",)
            ),
            frame=frame,
            interpolation="periodic" if channel_id == "position.geodetic.longitude" else "linear",
            periodicity=(ConfigurationPeriodicity(period=360.0, canonical_minimum=-180.0) if channel_id == "position.geodetic.longitude" else None),
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
            "Every declared Simple Aero template guarantees the advertised core state and mass telemetry through its "
            "exact fixed-L/D point-mass lowering. Maneuver-specific telemetry and all physical/vehicle claims remain "
            "outside this synthetic workflow contract."
        ),
    )
    ####


def open_simple_aero_session_episode(
    model: TrajectoryModelMetadata,
    prepared: PreparedTrajectoryConfiguration,
    *,
    seed: int | None = None,
    integration_step_s: float = 0.02,
) -> FixtureCompositionEpisode:
    """Open the existing interactive point-mass kernel through Mission Composition."""

    del integration_step_s
    build = build_simple_aero_prepared_configuration(prepared)
    parameters = build.parameters
    family = _simple_aero_family()
    kernel = ReferencePointMassSession(
        initial_speed_m_s=parameters.initial_speed_m_s,
        initial_altitude_m=parameters.initial_altitude_m,
        mass_kg=parameters.mass_kg,
        thrust_n=parameters.thrust_n,
        controls=family.controls,
        provider_id="taoryx.simple-aero.point-mass-session",
        case_id=prepared.configuration.configuration_id,
        duration_s=build.derived.total_duration_s,
        time_step_s=parameters.time_step_s,
    )
    contract, observation_schema = build_session_interface_contract(
        model,
        realization_id=prepared.configuration.realization_id or "fixed_ld_point_mass",
        fidelity=prepared.configuration.fidelity,
        family_id=family.family_id,
        physical_family="synthetic_fixed_ld_point_mass",
        claim_boundary=(
            "Interactive Simple Aero proof reusing ReferencePointMassSession. It implements constant-acceleration "
            "downrange motion and normalized thrust scaling only; scheduled bank is observable but non-steering, "
            "and no atmosphere or extra gravity model is carried into the session."
        ),
    )

    def initial_state_factory(_: int | None) -> Mapping[str, object]:
        native = kernel.reset()
        return {
            "downrange_m": float(native.values["position.downrange_m"]),
            "altitude_m": float(native.values["position.altitude_m"]),
            "speed_m_s": float(native.values["velocity.m_s"]),
            "mass_kg": parameters.mass_kg,
            "thrust_available_n": parameters.thrust_n,
            "throttle_realized": 0.0,
            "bank_command_deg": 0.0,
            "held_direct_throttle": 0.0,
            "generated_phase": "powered_ascent",
        }
        ####

    def transition(
        state: Mapping[str, Any],
        profile_id: str,
        action: Mapping[str, Any],
        duration_s: float,
        start_s: float,
    ) -> FixtureTransition:
        current = kernel.observe()
        native_values = cast(Mapping[str, Any], current.values)
        if (
            abs(current.time_s - start_s) > 1.0e-9
            or abs(float(native_values["position.downrange_m"]) - float(state["downrange_m"])) > 1.0e-9
            or abs(float(native_values["velocity.m_s"]) - float(state["speed_m_s"])) > 1.0e-9
        ):
            kernel.restore_state(
                time_s=start_s,
                downrange_m=float(state["downrange_m"]),
                altitude_m=float(state["altitude_m"]),
                speed_m_s=float(state["speed_m_s"]),
            )

        if profile_id == "generated_mission_commands":
            if action:
                raise ValueError("generated Simple Aero mission commands accept no caller action")
            throttle_request, bank_request, phase = _simple_aero_scheduled_commands(build, start_s)
            applied_semantic: dict[str, object] = {}
        elif profile_id == "direct_throttle_command":
            held = float(action.get("propulsion.command.fraction", state["held_direct_throttle"]))
            throttle_request = held
            bank_request = 0.0
            phase = "direct_throttle"
            applied_semantic = {"propulsion.command.fraction": held}
        else:
            raise ValueError(f"unsupported Simple Aero authority profile {profile_id!r}")

        result = kernel.step(
            duration_s,
            ControlFrame(
                values={"command.throttle": throttle_request, "command.bank": bank_request},
                authority={"command.throttle": "commanded", "command.bank": "commanded"},
            ),
        )
        throttle_realized = float(result.applied_controls.get("command.throttle", 0.0))
        bank_realized = float(result.applied_controls.get("command.bank", 0.0))
        next_state = {
            **dict(state),
            "downrange_m": float(result.state.values["position.downrange_m"]),
            "altitude_m": float(result.state.values["position.altitude_m"]),
            "speed_m_s": float(result.state.values["velocity.m_s"]),
            "throttle_realized": throttle_realized,
            "bank_command_deg": bank_realized,
            "generated_phase": phase,
        }
        if profile_id == "direct_throttle_command":
            next_state["held_direct_throttle"] = throttle_request
        return FixtureTransition(
            state=next_state,
            applied_action={
                "command.throttle": throttle_realized,
                "command.bank": bank_realized,
            },
            applied_semantic_action=applied_semantic,
            lowering_evidence={
                "authority_profile_id": profile_id,
                "lowering_chain": (
                    ["simple_aero_segment_program", "generated_bank_throttle_profile"]
                    if profile_id == "generated_mission_commands"
                    else ["propulsion.command.fraction", "command.throttle", "point_mass_thrust_scaling"]
                ),
                "requested_native_action": {
                    "command.throttle": throttle_request,
                    "command.bank": bank_request,
                },
                "achieved_native_action": dict(result.applied_controls),
                "control_decisions": list(result.control_decisions),
                "bank_effect": "telemetry_only_non_steering",
                "kernel": "ReferencePointMassSession",
            },
            diagnostics=tuple(result.diagnostics),
            status="active",
        )
        ####

    return FixtureCompositionEpisode(
        interface_contract=contract,
        observation_schema=observation_schema,
        initial_state_factory=initial_state_factory,
        observation_factory=_simple_aero_session_observation,
        transition=transition,
        claim_boundary=contract.claim_boundary,
        seed=seed,
    )
    ####


def _simple_aero_scheduled_commands(
    build: SimpleAeroTrajectoryBuild,
    time_s: float,
) -> tuple[float, float, str]:
    boost_end = build.derived.boost_duration_s
    coast_end = boost_end + build.derived.coast_duration_s
    bank_end = coast_end + build.parameters.bank_duration_s
    if time_s < boost_end:
        return 1.0, 0.0, "powered_ascent"
    if time_s < coast_end:
        return 0.0, 0.0, "ballistic_coast"
    if time_s < bank_end:
        return 0.0, build.parameters.bank_deg, "bank_maneuver"
    return 0.0, 0.0, "terminal"
    ####


def _simple_aero_session_observation(
    state: Mapping[str, Any],
    _: float,
    __: str,
) -> Mapping[str, object]:
    throttle = float(state["throttle_realized"])
    return {
        "position.geodetic.altitude": float(state["altitude_m"]),
        "velocity.speed": float(state["speed_m_s"]),
        "mass.total": float(state["mass_kg"]),
        "propulsion.thrust": throttle * float(state["thrust_available_n"]),
        "propulsion.throttle_command": throttle,
    }
    ####


def build_simple_aero_prepared_configuration(
    prepared: PreparedTrajectoryConfiguration,
) -> SimpleAeroTrajectoryBuild:
    """Compile one validated Simple Aero sequence through the fixed-L/D runner.

    Every declared source-shaped maneuver is lowered into a bounded synthetic
    point-mass profile.  The mapping is explicit and retained in generated
    problem comments; it gives authors a real compose-to-run path without
    misrepresenting fixture vocabulary as vehicle-specific maneuver physics.
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

    launch = _resolved_mapping(root, "launch_state")
    endpoint = _resolved_mapping(root, "endpoint")
    endpoint_state = _resolved_mapping(root, "endpoint_state")
    vehicle = _resolved_mapping(root, "vehicle_surrogate")
    checkpoints = _resolved_mapping(root, "trajectory_checkpoints")
    runtime = _resolved_mapping(root, "runtime")
    lowered_graph = _simple_aero_execution_segment_graph(segment_values)
    lowered_segments = lowered_graph["segments"]
    if not isinstance(lowered_segments, list):
        raise AssertionError("Simple Aero execution graph must contain a segment list")
    defaults = _simple_aero_fixed_ld_defaults(lowered_segments)

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
        alpha_deg=defaults["alpha_deg"],
        bank_deg=defaults["bank_deg"],
        boost_duration_s=defaults["boost_duration_s"],
        coast_duration_s=defaults["coast_duration_s"],
        bank_duration_s=defaults["bank_duration_s"],
        terminal_duration_s=defaults["terminal_duration_s"],
        terminal_capture_range_m=defaults["terminal_capture_range_m"],
        time_step_s=float(runtime["runtime.time_step"]),
        output_interval_s=float(runtime["runtime.output_interval"]),
        earth="standard" if runtime["earth_model"] == "standard_wgs84" else "none",
        **target_arguments,
    )
    return SimpleAeroTrajectoryBuilder(parameters, segment_graph=lowered_graph).build()
    ####


def _simple_aero_execution_segment_graph(raw_segments: list[Any]) -> dict[str, object]:
    """Lower declared segment vocabulary into bounded fixed-L/D primitives.

    The source fixtures describe maneuver intent, not a recoverable dynamics
    implementation.  Each lowering therefore preserves the named, authored
    input that has a direct bounded point-mass counterpart (duration, sign,
    bank, or alpha profile) and records the source ID beside the generated
    primitive.  Values with no fixed-L/D analogue remain provenance only.
    """

    if not raw_segments:
        raise ConfigurationContractError(
            "empty-segment-sequence",
            "Simple Aero execution requires at least one declared segment",
            path="prepared.resolved.segments",
        )
    source_ids = tuple(_resolved_segment_id(item) for item in raw_segments)
    occurrences = {identifier: source_ids.count(identifier) for identifier in source_ids}
    seen: dict[str, int] = {}
    lowered: list[dict[str, object]] = []
    cutoff_mode = "physical"
    for index, raw in enumerate(raw_segments, start=1):
        source_id = _resolved_segment_id(raw)
        values = _resolved_segment_values(raw, source_id)
        seen[source_id] = seen.get(source_id, 0) + 1
        identifier = source_id.replace("_", "-")
        if occurrences[source_id] > 1:
            identifier = f"{identifier}-{seen[source_id]}"
        segment = _lower_simple_aero_segment(source_id, values, identifier)
        lowered.append(segment)
        if source_id == "powered_ascent":
            cutoff = str(values.get("cutoff_condition", "physical_burnout"))
            try:
                cutoff_mode = {
                    "physical_burnout": "physical",
                    "commanded_burnout_speed": "commanded",
                }[cutoff]
            except KeyError as error:
                raise ConfigurationContractError(
                    "unknown-cutoff-condition",
                    f"unsupported powered-ascent cutoff {cutoff!r}",
                    path=f"prepared.resolved.segments[{index - 1}].value.cutoff_condition",
                ) from error
    return {
        "cutoff_mode": cutoff_mode,
        "source_segment_sequence": list(source_ids),
        "lowering": "source_parameter_mapped_fixed_ld_point_mass_v1",
        "segments": lowered,
    }
    ####


def _lower_simple_aero_segment(
    source_id: str,
    values: Mapping[str, Any],
    identifier: str,
) -> dict[str, object]:
    """Return one explicit source-parameter-to-fixed-L/D segment mapping."""

    def duration(parameter_id: str, default: float) -> float:
        return _positive_segment_value(values, parameter_id, default, source_id)
        ####

    base: dict[str, object] = {"id": identifier, "source_segment_id": source_id}
    if source_id == "powered_ascent":
        return {**base, "kind": "powered_ascent", "duration_s": duration("duration_s", 5.0)}
    if source_id == "ballistic_coast":
        return {
            **base,
            "kind": "ballistic_coast",
            "duration_s": duration("duration_s", 10.0),
            "alpha_deg": _finite_segment_value(values, "alpha_deg", 0.0, source_id),
        }
    if source_id == "bank_maneuver":
        return {
            **base,
            "kind": "bank_maneuver",
            "duration_s": duration("duration_s", 5.0),
            "bank_deg": _finite_segment_value(values, "bank_deg", 0.0, source_id),
        }
    if source_id == "cbcr":
        bank_deg = 15.0 if bool(values.get("go_left", True)) else -15.0
        return {**base, "kind": "bank_maneuver", "duration_s": duration("duration_s", 35.0), "bank_deg": bank_deg}
    if source_id == "crossrange":
        return {
            **base,
            "kind": "bank_maneuver",
            "duration_s": duration("minimum_time_to_go_s", 60.0),
            "bank_deg": _finite_segment_value(values, "initial_heading_error_deg", 15.0, source_id),
        }
    if source_id == "marv":
        return {
            **base,
            "kind": "bank_maneuver",
            "duration_s": duration("maneuver_begin_time_to_go_s", 60.0),
            "bank_deg": 0.0,
        }
    if source_id == "phugoid":
        frequency_hz = _positive_segment_value(values, "frequency_hz", 0.015, source_id)
        profile_duration_s = 1.0 / frequency_hz
        amplitude_deg = _finite_segment_value(values, "amplitude_deg", 2.0, source_id)
        return {
            **base,
            "kind": "alpha_profile",
            "duration_s": profile_duration_s,
            "bank_deg": _finite_segment_value(values, "maneuver_roll_deg", 45.0, source_id),
            "alpha_profile_deg": [
                [0.0, 0.0],
                [profile_duration_s * 0.25, amplitude_deg],
                [profile_duration_s * 0.75, -amplitude_deg],
                [profile_duration_s, 0.0],
            ],
        }
    if source_id == "range_extension":
        return {**base, "kind": "alpha_profile", "duration_s": duration("minimum_time_to_go_s", 60.0), "alpha_deg": 0.0}
    if source_id == "skip":
        return {**base, "kind": "skip_maneuver", "duration_s": duration("maneuver_begin_time_to_go_s", 400.0), "alpha_deg": 0.0, "bank_deg": 0.0}
    if source_id == "slalom":
        return {**base, "kind": "alpha_profile", "duration_s": duration("minimum_time_to_go_s", 60.0), "alpha_deg": 0.0}
    if source_id == "weave":
        return {**base, "kind": "bank_maneuver", "duration_s": duration("minimum_time_to_go_s", 60.0), "bank_deg": 0.0}
    if source_id == "terminal_pronav":
        return {
            **base,
            "kind": "terminal_pronav",
            "duration_s": duration("duration_s", 5.0),
            "capture_range_m": _positive_segment_value(values, "capture_range_m", 25.0, source_id),
        }
    raise ConfigurationContractError(
        "unsupported-execution-segment",
        f"Simple Aero does not have a fixed-L/D lowering for {source_id!r}",
        path="prepared.resolved.segments",
    )
    ####


def _simple_aero_fixed_ld_defaults(segments: list[object]) -> dict[str, float]:
    """Return positive builder fallbacks from the first matching lowered segments."""

    mapping = [item for item in segments if isinstance(item, Mapping)]

    def find(kind: str, key: str, default: float) -> float:
        for item in mapping:
            if item.get("kind") == kind and key in item:
                return float(item[key])
        return default
        ####

    return {
        "alpha_deg": find("ballistic_coast", "alpha_deg", 0.0),
        "bank_deg": find("bank_maneuver", "bank_deg", 0.0),
        "boost_duration_s": find("powered_ascent", "duration_s", 5.0),
        "coast_duration_s": find("ballistic_coast", "duration_s", 10.0),
        "bank_duration_s": find("bank_maneuver", "duration_s", 5.0),
        "terminal_duration_s": find("terminal_pronav", "duration_s", 5.0),
        "terminal_capture_range_m": find("terminal_pronav", "capture_range_m", 25.0),
    }
    ####


def _resolved_segment_id(value: Any) -> str:
    """Return one resolved segment choice identifier with a useful path error."""

    if not isinstance(value, Mapping) or not isinstance(value.get("selected"), str):
        raise ConfigurationContractError(
            "invalid-prepared-configuration",
            "each resolved segment must identify its selected variant",
            path="prepared.resolved.segments",
        )
    return str(value["selected"])
    ####


def _resolved_segment_values(value: Any, source_id: str) -> Mapping[str, Any]:
    """Return one resolved segment parameter mapping."""

    if not isinstance(value, Mapping) or not isinstance(value.get("value"), Mapping):
        raise ConfigurationContractError(
            "invalid-prepared-configuration",
            f"resolved {source_id!r} segment requires a parameter mapping",
            path="prepared.resolved.segments",
        )
    return value["value"]
    ####


def _finite_segment_value(values: Mapping[str, Any], parameter_id: str, default: float, source_id: str) -> float:
    """Read a finite source parameter or surface an execution-specific error."""

    try:
        value = float(values.get(parameter_id, default))
    except (TypeError, ValueError) as error:
        raise ConfigurationContractError(
            "invalid-execution-parameter",
            f"{source_id!r} parameter {parameter_id!r} must be numeric",
            path=f"prepared.resolved.segments.{source_id}.{parameter_id}",
        ) from error
    if not math.isfinite(value):
        raise ConfigurationContractError(
            "invalid-execution-parameter",
            f"{source_id!r} parameter {parameter_id!r} must be finite",
            path=f"prepared.resolved.segments.{source_id}.{parameter_id}",
        )
    return value
    ####


def _positive_segment_value(values: Mapping[str, Any], parameter_id: str, default: float, source_id: str) -> float:
    """Read a strictly positive duration or range required by native lowering."""

    value = _finite_segment_value(values, parameter_id, default, source_id)
    if value <= 0.0:
        raise ConfigurationContractError(
            "invalid-execution-duration",
            f"{source_id!r} parameter {parameter_id!r} must be positive for fixed-L/D execution",
            path=f"prepared.resolved.segments.{source_id}.{parameter_id}",
        )
    return value
    ####


def build_simple_aero_example_configuration(
    schema: TrajectoryConfigurationSchema | None = None,
) -> TrajectoryConfigurationInstance:
    """Return the small checked contract witness for the runnable baseline."""

    return build_simple_aero_template_configuration("fixed_ld_baseline", schema=schema)
    ####


def build_simple_aero_template_configuration(
    mission_template_id: str,
    *,
    schema: TrajectoryConfigurationSchema | None = None,
) -> TrajectoryConfigurationInstance:
    """Return a minimal valid request for one published Simple Aero template.

    This is intentionally useful to plug-in and model authors: the selected
    template supplies its declared sequence while ordinary defaults populate
    source-shaped maneuver parameters.  Callers can replace any value in the
    returned typed tree before validation; the runtime will retain the exact
    selected segment IDs in its fixed-L/D lowering comments.
    """

    resolved_schema = schema or simple_aero_configuration_schema()
    sequence = _segments(resolved_schema)
    try:
        template = next(item for item in sequence.templates if item.id == mission_template_id)
    except StopIteration as error:
        raise KeyError(f"unknown Simple Aero mission template {mission_template_id!r}") from error
    empty = ConfigurationGroupValue(values={})
    return TrajectoryConfigurationInstance(
        configuration_id=f"simple-aero-{mission_template_id}-common-witness",
        model_id=SIMPLE_AERO_MODEL_ID,
        model_version=resolved_schema.model_version,
        schema_fingerprint=resolved_schema.fingerprint,
        fidelity=_POINT_MASS,
        realization_id="fixed_ld_point_mass",
        mission_template_id=mission_template_id,
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
                "vehicle_surrogate": ConfigurationGroupValue(values={"vehicle.mass.initial": ConfigurationParameterValue(value=1000.0, unit="kg")}),
                "trajectory_checkpoints": ConfigurationGroupValue(values={"mission.burnout_speed": ConfigurationParameterValue(value=1200.0, unit="m/s")}),
                "segments": ConfigurationSequenceValue(
                    items=tuple(ConfigurationChoiceValue(selected=identifier, value=empty) for identifier in template.item_variants)
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


def _generic_contract_name(
    name: str,
) -> Literal[
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
            _number(
                "alpha_deg",
                "Angle of Attack",
                "Constant coast angle of attack.",
                unit="deg",
                default=0.0,
                lower=-45.0,
                upper=45.0,
                role="segment",
                provenance="src/taoryx/simple_aero_builder.py",
            ),
        )
    if name == "bank_maneuver":
        return (
            _family_duration(family, "mission.bank_duration", "duration_s", "Maneuver Duration"),
            _number(
                "bank_deg",
                "Bank Angle",
                "Signed constant bank command.",
                unit="deg",
                default=0.0,
                lower=-180.0,
                upper=180.0,
                role="segment",
                provenance="src/taoryx/simple_aero_builder.py",
            ),
        )
    if name == "cbcr":
        return (
            _boolean("go_left", "Turn Left", "Select the source-shaped left or right CBCR maneuver.", True, source),
            _number(
                "maneuver_altitude_start_m",
                "Maneuver Start Altitude",
                "Altitude checkpoint for maneuver entry.",
                unit="m",
                default=40_000.0,
                lower=0.0,
                role="segment",
                provenance=source,
            ),
            _number(
                "duration_s",
                "Maneuver Duration",
                "Requested CBCR maneuver duration.",
                unit="s",
                default=35.0,
                lower=0.0,
                lower_inclusive=False,
                role="segment",
                provenance=source,
            ),
            _number(
                "minimum_time_to_go_s", "Minimum Time to Go", "Terminal time-to-go floor.", unit="s", default=60.0, lower=0.0, role="segment", provenance=source
            ),
        )
    if name == "crossrange":
        return (
            _number(
                "initial_heading_error_deg",
                "Initial Heading Error",
                "Signed launch heading error used to seed crossrange.",
                unit="deg",
                default=15.0,
                lower=-180.0,
                upper=180.0,
                role="segment",
                provenance=source,
            ),
            _number(
                "minimum_time_to_go_s", "Minimum Time to Go", "Terminal time-to-go floor.", unit="s", default=60.0, lower=0.0, role="segment", provenance=source
            ),
        )
    if name == "marv":
        return (
            _number(
                "maneuver_begin_time_to_go_s",
                "Maneuver Begin Time to Go",
                "Time-to-go checkpoint for maneuver entry.",
                unit="s",
                default=60.0,
                lower=0.0,
                role="segment",
                provenance=source,
            ),
            _number(
                "minimum_time_to_go_s", "Minimum Time to Go", "Terminal time-to-go floor.", unit="s", default=30.0, lower=0.0, role="segment", provenance=source
            ),
        )
    if name == "phugoid":
        return (
            _optional_number(
                "start_range_to_go_m", "Start Range to Go", "Optional range-to-go checkpoint for profile entry.", unit="m", lower=0.0, provenance=source
            ),
            _number(
                "amplitude_deg",
                "Phugoid Amplitude",
                "Source-shaped alpha-profile amplitude.",
                unit="deg",
                default=2.0,
                lower=0.0,
                role="segment",
                provenance=source,
            ),
            _number(
                "frequency_hz",
                "Phugoid Frequency",
                "Source-shaped alpha-profile frequency.",
                unit="Hz",
                default=0.015,
                lower=0.0,
                lower_inclusive=False,
                role="segment",
                provenance=source,
            ),
            _number(
                "maneuver_roll_deg",
                "Maneuver Roll",
                "Signed roll command associated with the profile.",
                unit="deg",
                default=45.0,
                lower=-180.0,
                upper=180.0,
                role="segment",
                provenance=source,
            ),
        )
    if name == "range_extension":
        return (
            _number(
                "minimum_time_to_go_s", "Minimum Time to Go", "Terminal time-to-go floor.", unit="s", default=60.0, lower=0.0, role="segment", provenance=source
            ),
        )
    if name == "skip":
        return (
            _number(
                "maneuver_begin_time_to_go_s",
                "Maneuver Begin Time to Go",
                "Time-to-go checkpoint for skip entry.",
                unit="s",
                default=400.0,
                lower=0.0,
                role="segment",
                provenance=source,
            ),
        )
    if name == "slalom":
        return (
            _optional_number(
                "start_range_to_go_m", "Start Range to Go", "Optional range-to-go checkpoint for slalom entry.", unit="m", lower=0.0, provenance=source
            ),
            _number(
                "end_range_to_go_m",
                "End Range to Go",
                "Range-to-go checkpoint for slalom exit.",
                unit="m",
                default=200_000.0,
                lower=0.0,
                role="segment",
                provenance=source,
            ),
            _number(
                "minimum_time_to_go_s", "Minimum Time to Go", "Terminal time-to-go floor.", unit="s", default=60.0, lower=0.0, role="segment", provenance=source
            ),
        )
    if name == "weave":
        return (
            _optional_number(
                "end_range_to_go_m", "Weave End Range to Go", "Optional range-to-go checkpoint for weave exit.", unit="m", lower=0.0, provenance=source
            ),
            _number(
                "minimum_time_to_go_s", "Minimum Time to Go", "Terminal time-to-go floor.", unit="s", default=60.0, lower=0.0, role="segment", provenance=source
            ),
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


def _template_segment_ids(mission_template_id: str) -> tuple[SimpleAeroSegmentKind, ...]:
    """Return the declared source sequence for one published mission template."""

    try:
        template = next(item for item in _segments(simple_aero_configuration_schema()).templates if item.id == mission_template_id)
    except StopIteration as error:
        raise KeyError(f"unknown Simple Aero mission template {mission_template_id!r}") from error
    return cast(tuple[SimpleAeroSegmentKind, ...], template.item_variants)
    ####


def _mission_metadata(template: ConfigurationSequenceTemplate) -> TrajectoryMissionTemplateMetadata:
    batch = TrajectoryMissionOperationMetadata(
        fidelity=_POINT_MASS,
        realization_id="fixed_ld_point_mass",
        operation="batch",
        status="available",
        execution_mode="build_simple_aero_prepared_configuration_then_taoryx_batch",
        availability_scope="provider_interface",
        common_runner_status="registered",
        executor_id="taoryx.simple-aero.common-batch.v1",
        blockers=(),
        claim_boundary=(
            "Generated fixed-coefficient point-mass execution. Source-shaped maneuver parameters lower only to "
            "the declared bounded fixed-L/D alpha, bank, and time profile; this is not vehicle-specific performance "
            "or control validation."
        ),
    )
    return TrajectoryMissionTemplateMetadata(
        id=template.id,
        name=template.label,
        description=template.description,
        status="runnable",
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
                status="available",
                execution_mode="simple_aero_point_mass_session",
                availability_scope="provider_interface",
                common_runner_status="registered",
                executor_id="taoryx.simple-aero.point-mass-session.v1",
                claim_boundary=(
                    "Reuses the existing constant-acceleration ReferencePointMassSession. Direct throttle is active; "
                    "bank is schedule telemetry only and does not steer the interactive proof."
                ),
            ),
        ),
        provenance="verification/simple_aero_segment_catalog.yaml; tests/fixtures/simple_aero_v1",
        claim_boundary=(
            "Template order and parameter vocabulary execute through a bounded synthetic fixed-L/D point-mass lowering. "
            "Vehicle-specific aerodynamics, control response, terminal accuracy, optimization, and promotion evidence "
            "remain outside this workflow-level contract."
        ),
    )
    ####


def _custom_composition_metadata() -> TrajectoryMissionTemplateMetadata:
    """Return the explicit batch binding for arbitrary schema-valid segment orders."""

    baseline = _mission_metadata(
        ConfigurationSequenceTemplate(
            id="fixed_ld_baseline",
            label="Fixed-L/D Baseline",
            description="The four-phase sequence implemented by the reduced-order Simple Aero builder.",
            item_variants=("powered_ascent", "ballistic_coast", "bank_maneuver", "terminal_pronav"),
            compatible_fidelities=(_POINT_MASS,),
        )
    )
    return TrajectoryMissionTemplateMetadata(
        id=SIMPLE_AERO_CUSTOM_COMPOSITION_ID,
        name="Custom Composition (Open Sequence)",
        description=(
            "Caller-authored sequence of one through 32 published Simple Aero segment occurrences. "
            "This is an execution binding, not a reviewed source fixture template."
        ),
        status=baseline.status,
        initialization_variants=baseline.initialization_variants,
        open_segment_sequence=TrajectoryOpenSegmentSequenceMetadata(
            configuration_node_id="segments",
            allowed_segment_ids=_segment_names(),
            minimum_items=1,
            maximum_items=32,
        ),
        compatible_fidelities=baseline.compatible_fidelities,
        operations=baseline.operations,
        provenance=baseline.provenance,
        claim_boundary=(
            "Any schema-valid ordered source segment vocabulary lowers through the bounded synthetic fixed-L/D "
            "point-mass mapping. The order is caller-authored rather than a reviewed source fixture, and vehicle "
            "performance, control response, terminal accuracy, optimization, and promotion evidence remain outside "
            "this workflow-level contract."
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
        dynamics_fidelity=("point_mass_3dof" if tier == "point_mass_3dof" else "pseudo_6dof" if tier == "pseudo_6dof" else "rigid_body_6dof"),
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
        operations=("validate", "batch", "step") if declared else (),
        profile_id="simple_aero.fixed_ld_3dof" if declared else None,
        blockers=() if declared else ("Simple Aero workflow does not declare this realization tier",),
        required_operations=("configuration_validation", "generated_problem_batch", "point_mass_session") if declared else (),
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
    "SimpleAeroCheckpoints",
    "SimpleAeroCutoffCondition",
    "SimpleAeroEarthModel",
    "SimpleAeroEndpoint",
    "SimpleAeroEndpointState",
    "SimpleAeroGeodeticAimpoint",
    "SimpleAeroLaunch",
    "SimpleAeroMission",
    "SimpleAeroRangeBearingEndpoint",
    "SimpleAeroRuntime",
    "SimpleAeroSegment",
    "SimpleAeroSegmentKind",
    "SimpleAeroSurrogate",
    "build_simple_aero_configuration",
    "build_simple_aero_example_configuration",
    "build_simple_aero_prepared_configuration",
    "open_simple_aero_session_episode",
    "build_simple_aero_template_configuration",
    "prepare_simple_aero_mission",
    "simple_aero_configuration_schema",
    "simple_aero_model_metadata",
]
