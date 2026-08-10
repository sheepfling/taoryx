"""Reusable local LQI witness through a model's named native controls.

The common controller host can synthesize an offset-free candidate for any
``ControlPlantAdapter``.  This module is the intentionally narrow execution
companion for models that accept named controls directly but do not own a
physical wrench allocator.  It preserves that distinction: a successful
screen establishes one bounded nonlinear recovery through the adapter's own
coordinates, never an actuator, surface-allocation, navigation, or vehicle
qualification result.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field

from .control_allocation import ControlPlantAdapter
from .generic_tuning import (
    GenericLqrCandidate,
    NativeCoordinateLqiSample,
    NativeCoordinateLqiValidation,
    validate_nonlinear_native_coordinate_lqi,
)
from .trim import TrimResult

NativeStatusSampleMapper = Callable[[NativeCoordinateLqiSample], Mapping[str, object]]


def _finite_named_values(
    values: Mapping[str, float],
    *,
    names: tuple[str, ...],
    label: str,
) -> dict[str, float]:
    """Validate one finite named numeric mapping without changing its order."""

    identifiers = set(values)
    allowed = set(names)
    if identifiers != allowed:
        raise ValueError(f"{label} names must exactly match {list(names)!r}")
    result = {name: float(value) for name, value in values.items()}
    if any(not math.isfinite(value) for value in result.values()):
        raise ValueError(f"{label} values must be finite")
    return result
    ####


@dataclass(frozen=True, slots=True)
class LocalNativeCoordinateLqiScreenConfig:
    """A plug-in supplied pinned native-coordinate LQI recovery case.

    Plug-ins provide their existing plant, retained common-host candidate,
    initial perturbation, limits, and a truthful status projection.  The host
    owns the nonlinear integration, integral state accounting, acceptance
    gates, and portable artifact shape.
    """

    id: str
    plant_id: str
    fidelity: str
    plant: ControlPlantAdapter
    trim: TrimResult
    candidate: GenericLqrCandidate
    campaign_id: str
    initial_state: Mapping[str, float]
    duration_s: float
    dt_s: float
    assessment_state_names: tuple[str, ...]
    control_lower: Mapping[str, float]
    control_upper: Mapping[str, float]
    integral_lower: Mapping[str, float]
    integral_upper: Mapping[str, float]
    environment: Mapping[str, float | str]
    status_sample_mapper: NativeStatusSampleMapper
    resource_values: Mapping[str, float] = field(default_factory=dict)
    final_error_fraction_limit: float = 0.01
    maximum_control_saturation_fraction: float = 0.0

    def __post_init__(self) -> None:
        if not self.id.strip() or not self.plant_id.strip() or not self.campaign_id.strip():
            raise ValueError("local native-coordinate LQI screen requires nonempty identities")
        if not self.fidelity.strip():
            raise ValueError("local native-coordinate LQI screen requires a fidelity")
        if self.candidate.method != "lqi" or self.candidate.lqi is None or not self.candidate.safe:
            raise ValueError("local native-coordinate LQI screen requires a retained safe LQI candidate")
        if tuple(self.trim.spec.state_names) != tuple(self.plant.state_names):
            raise ValueError("local native-coordinate LQI trim state names must match the plant")
        if tuple(self.trim.spec.control_names) != tuple(self.plant.control_names):
            raise ValueError("local native-coordinate LQI trim control names must match the plant")
        _finite_named_values(self.initial_state, names=tuple(self.plant.state_names), label="initial state")
        candidate_controls = tuple(self.candidate.control_names)
        _finite_named_values(self.control_lower, names=candidate_controls, label="control lower")
        _finite_named_values(self.control_upper, names=candidate_controls, label="control upper")
        if any(float(self.control_lower[name]) > float(self.control_upper[name]) for name in self.control_lower):
            raise ValueError("local native-coordinate LQI control bounds must be ordered")
        outputs = tuple(self.candidate.lqi.output_names)
        _finite_named_values(self.integral_lower, names=outputs, label="integral lower")
        _finite_named_values(self.integral_upper, names=outputs, label="integral upper")
        if any(float(self.integral_lower[name]) > float(self.integral_upper[name]) for name in self.integral_lower):
            raise ValueError("local native-coordinate LQI integral bounds must be ordered")
        if not self.assessment_state_names or set(self.assessment_state_names) - set(self.candidate.state_names):
            raise ValueError("local native-coordinate LQI assessment states must be candidate states")
        if len(set(self.assessment_state_names)) != len(self.assessment_state_names):
            raise ValueError("local native-coordinate LQI assessment states must be unique")
        for label, value in (
            ("duration_s", self.duration_s),
            ("dt_s", self.dt_s),
            ("final_error_fraction_limit", self.final_error_fraction_limit),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{label} must be finite and positive")
        if not math.isfinite(self.maximum_control_saturation_fraction) or not 0.0 <= self.maximum_control_saturation_fraction <= 1.0:
            raise ValueError("maximum_control_saturation_fraction must lie within [0, 1]")
        if not callable(self.status_sample_mapper):
            raise TypeError("local native-coordinate LQI screen requires a callable status sample mapper")
        if any(not name.strip() or not math.isfinite(float(value)) for name, value in self.resource_values.items()):
            raise ValueError("resource_values must use nonempty names and finite values")
        ####
    ####


@dataclass(frozen=True, slots=True)
class LocalNativeCoordinateLqiScreenExecution:
    """Auditable outcome of one generic native-coordinate LQI screen."""

    config: LocalNativeCoordinateLqiScreenConfig
    validation: NativeCoordinateLqiValidation
    screen_pass: bool

    @property
    def initial_error_norm(self) -> float:
        return self.validation.initial_normalized_feedback_error_norm
        ####

    @property
    def final_error_norm(self) -> float:
        return self.validation.final_normalized_feedback_error_norm
        ####

    @property
    def final_error_fraction(self) -> float:
        return self.final_error_norm / max(self.initial_error_norm, 1.0e-12)
        ####

    def as_dict(self) -> dict[str, object]:
        """Serialize one local screen without relabeling controls as effectors."""

        return {
            "schema": "taoryx.local-native-coordinate-lqi-screen/v1alpha1",
            "id": self.config.id,
            "plant_id": self.config.plant_id,
            "fidelity": self.config.fidelity,
            "control_realization": "native_named_coordinates",
            "physical_effector_allocation": False,
            "controller": {
                "method": "lqi",
                "tuning_campaign_id": self.config.campaign_id,
                "candidate": self.config.candidate.as_dict(),
                "integral_output_names": list(self.config.candidate.lqi.output_names if self.config.candidate.lqi else ()),
                "integrators_exercised": self.validation.integrators_exercised,
            },
            "state_names": list(self.config.plant.state_names),
            "native_control_names": list(self.config.candidate.control_names),
            "assessment_state_names": list(self.config.assessment_state_names),
            "reference_state": dict(self.validation.state_reference),
            "initial_state": dict(self.validation.initial_state),
            "resource_values": dict(self.config.resource_values),
            "control_limits": {
                "lower": dict(self.config.control_lower),
                "upper": dict(self.config.control_upper),
                "saturation_fraction_limit": self.config.maximum_control_saturation_fraction,
            },
            "integral_limits": {
                "lower": dict(self.config.integral_lower),
                "upper": dict(self.config.integral_upper),
            },
            "environment": dict(self.config.environment),
            "evaluation": {
                "mission_pass": self.screen_pass,
                "initial_error_norm": self.initial_error_norm,
                "final_error_norm": self.final_error_norm,
                "final_error_fraction": self.final_error_fraction,
                "final_error_fraction_limit": self.config.final_error_fraction_limit,
                "control_saturation_fraction": self.validation.control_saturation_fraction,
                "maximum_control_saturation_fraction": self.config.maximum_control_saturation_fraction,
                "saturated_controls": list(self.validation.saturated_controls),
                "integrators_exercised": self.validation.integrators_exercised,
                "sample_count": len(self.validation.samples),
                "dt_s": self.validation.dt_s,
                "duration_s": self.validation.duration_s,
            },
            "telemetry": [sample.as_dict() for sample in self.validation.samples],
            "claim_boundary": (
                "This is one pinned local LQI recovery through the model's declared named native controls. "
                "It does not establish physical effector allocation, actuator dynamics, navigation, route tracking, "
                "persistent-disturbance rejection outside the declared local derivative environment, or vehicle qualification."
            ),
        }
        ####
    ####


def run_local_native_coordinate_lqi_screen(
    config: LocalNativeCoordinateLqiScreenConfig,
) -> LocalNativeCoordinateLqiScreenExecution:
    """Execute one plug-in supplied LQI case through the shared nonlinear validator."""

    validation = validate_nonlinear_native_coordinate_lqi(
        config.plant,
        config.trim,
        config.candidate,
        initial_state=config.initial_state,
        duration_s=config.duration_s,
        dt_s=config.dt_s,
        assessment_state_names=config.assessment_state_names,
        control_lower=config.control_lower,
        control_upper=config.control_upper,
        integral_lower=config.integral_lower,
        integral_upper=config.integral_upper,
        environment=config.environment,
    )
    final_error_fraction = validation.final_normalized_feedback_error_norm / max(
        validation.initial_normalized_feedback_error_norm,
        1.0e-12,
    )
    screen_pass = (
        validation.integrators_exercised
        and final_error_fraction < config.final_error_fraction_limit
        and validation.control_saturation_fraction <= config.maximum_control_saturation_fraction
    )
    return LocalNativeCoordinateLqiScreenExecution(config, validation, screen_pass)
    ####


__all__ = [
    "LocalNativeCoordinateLqiScreenConfig",
    "LocalNativeCoordinateLqiScreenExecution",
    "NativeStatusSampleMapper",
    "run_local_native_coordinate_lqi_screen",
]
