"""Claim-bounded local response analysis for the pseudo-6DOF surrogate tier."""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping
from pathlib import Path
from typing import TYPE_CHECKING, Literal, Self, cast

import numpy as np
import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .control_authority import (
    ControlConfiguration,
    evaluate_control_authority,
)
from .profile import (
    InterceptorEvidenceProfile,
    ResolvedInterceptorProfile,
    ValueOrigin,
)
from .resolver import resolve_interceptor

if TYPE_CHECKING:
    from .pseudo6 import Pseudo6Sample

ResponseStabilityStatus = Literal["stable", "marginal", "unstable"]
ResponseSamplingQuality = Literal["fine", "adequate", "coarse", "very_coarse", "unstable", "not_applicable"]
ResponseAxis = Literal["roll", "pitch", "yaw"]
ResponseAngleTopology = Literal["bounded", "periodic"]


class Pseudo6ResponseAnalysisRequest(BaseModel):
    """One reproducible local response-law analysis request."""

    model_config = ConfigDict(frozen=True, extra="forbid", str_strip_whitespace=True)

    analysis_id: str = Field(
        default="default-pseudo6-response-analysis",
        pattern=r"^[a-z0-9]+(?:[._-][a-z0-9]+)*$",
    )
    sample_time_s: float = Field(default=0.05, gt=0.0)
    command_step_rad: float = Field(default=math.radians(10.0), gt=0.0, le=math.pi / 2.0)
    command_support_fraction: float = Field(default=1.0, ge=0.0, le=1.0)
    axis: ResponseAxis = "roll"
    settling_band_fraction: float = Field(default=0.02, gt=0.0, lt=1.0)
    stability_tolerance: float = Field(default=1.0e-9, ge=0.0)
    operating_point_id: str = Field(
        default="zero-error-unsaturated-local-response",
        pattern=r"^[a-z0-9]+(?:[._-][a-z0-9]+)*$",
    )
    claim_boundary: str = (
        "Analysis covers one selected axis of the symmetric, unsaturated second-order attitude-response law and its "
        "implemented semi-implicit discrete update at one sample time under the request's frozen command-support "
        "fraction. A fraction of one is nominal full support; a smaller value is a local frozen approximation of "
        "runtime environment/propulsion authority coupling, not a gain schedule. Axis-specific angle bounds are "
        "reported separately. It is not a missile airframe, actuator, autopilot, nonlinear, or robust-stability "
        "qualification."
    )

    @model_validator(mode="after")
    def validate_finite_values(self) -> Self:
        values = (
            self.sample_time_s,
            self.command_step_rad,
            self.command_support_fraction,
            self.settling_band_fraction,
            self.stability_tolerance,
        )
        if any(not math.isfinite(value) for value in values):
            raise ValueError("pseudo-6DOF response-analysis values must be finite")
        return self
        ####

    @property
    def fingerprint(self) -> str:
        """Return the stable identity of this analysis request."""

        return _fingerprint(self.model_dump(mode="json"))
        ####

    ####


class Pseudo6ResponsePole(BaseModel):
    """One JSON-safe continuous or discrete response-law pole."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    real: float
    imaginary: float
    magnitude: float = Field(ge=0.0)

    @model_validator(mode="after")
    def validate_finite_values(self) -> Self:
        if any(not math.isfinite(value) for value in (self.real, self.imaginary, self.magnitude)):
            raise ValueError("pseudo-6DOF response poles must be finite")
        return self
        ####

    ####


class Pseudo6ResponseParameterTrace(BaseModel):
    """Resolved parameter and provenance used by one response analysis."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    parameter_id: str
    value: float
    unit: str
    origin: ValueOrigin
    method: str
    source_record_ids: tuple[str, ...] = ()

    ####


class Pseudo6ResponseAnalysis(BaseModel):
    """Exact local analysis of the pseudo-6DOF response law and update rule."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    contract: Literal["taoryx.parametric-interceptors.pseudo6-response-analysis/v2"] = "taoryx.parametric-interceptors.pseudo6-response-analysis/v2"
    analysis_id: str
    request: Pseudo6ResponseAnalysisRequest
    request_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    model_id: str
    profile_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    operating_point_id: str
    analysis_axis: ResponseAxis
    linear_response_axes: tuple[ResponseAxis, ...] = ("roll", "pitch", "yaw")
    linear_response_axes_symmetric: Literal[True] = True
    angle_topology_axis_specific: Literal[True] = True
    angle_topology: ResponseAngleTopology
    angle_limit_rad: float | None = Field(default=None, gt=0.0)
    angle_limit_provenance: str
    parameter_trace: tuple[Pseudo6ResponseParameterTrace, ...]
    authority_coupling_assumption: Literal["frozen_command_support_fraction"] = "frozen_command_support_fraction"
    command_support_fraction: float = Field(ge=0.0, le=1.0)
    response_authority_available: bool
    natural_frequency_rad_s: float = Field(gt=0.0)
    damping_ratio: float = Field(gt=0.0)
    effective_natural_frequency_rad_s: float = Field(ge=0.0)
    effective_damping_ratio: float = Field(ge=0.0)
    damped_natural_frequency_rad_s: float = Field(ge=0.0)
    continuous_poles: tuple[Pseudo6ResponsePole, Pseudo6ResponsePole]
    continuous_status: ResponseStabilityStatus
    continuous_spectral_abscissa_rad_s: float
    continuous_stability_margin_rad_s: float
    discrete_poles: tuple[Pseudo6ResponsePole, Pseudo6ResponsePole]
    discrete_status: ResponseStabilityStatus
    discrete_spectral_radius: float = Field(ge=0.0)
    discrete_stability_margin: float
    normalized_sample_time: float = Field(ge=0.0)
    maximum_stable_sample_time_s: float | None = Field(default=None, gt=0.0)
    sample_time_fraction_of_stability_limit: float | None = Field(default=None, gt=0.0)
    sampling_quality: ResponseSamplingQuality
    continuous_settling_time_estimate_s: float | None = Field(default=None, ge=0.0)
    discrete_settling_time_estimate_s: float | None = Field(default=None, ge=0.0)
    step_overshoot_fraction: float = Field(ge=0.0)
    command_step_rad: float = Field(gt=0.0)
    effective_command_step_rad: float = Field(gt=0.0)
    command_step_limited: bool
    pre_support_initial_angular_acceleration_demand_rad_s2: float = Field(ge=0.0)
    initial_angular_acceleration_demand_rad_s2: float = Field(ge=0.0)
    characteristic_body_rate_demand_rad_s: float = Field(ge=0.0)
    linear_peak_body_rate_demand_rad_s: float = Field(ge=0.0)
    maximum_body_acceleration_rad_s2: float = Field(gt=0.0)
    authority_scaled_maximum_body_acceleration_rad_s2: float = Field(ge=0.0)
    maximum_body_rate_rad_s: float = Field(gt=0.0)
    acceleration_headroom_ratio: float = Field(ge=0.0)
    body_rate_headroom_ratio: float | None = Field(default=None, ge=0.0)
    angle_headroom_ratio: float | None = Field(default=None, ge=0.0)
    angle_headroom_rad: float | None = None
    predicted_acceleration_saturation: bool
    predicted_body_rate_saturation: bool
    predicted_angle_saturation: bool
    local_unsaturated_continuous_linear_claim: Literal[True] = True
    local_unsaturated_discrete_linear_claim: Literal[True] = True
    physical_controller_qualification_claim: Literal[False] = False
    airframe_stability_claim: Literal[False] = False
    nonlinear_or_robust_stability_claim: Literal[False] = False
    claim_boundary: str

    @model_validator(mode="after")
    def validate_identity(self) -> Self:
        if self.analysis_id != self.request.analysis_id:
            raise ValueError("response analysis ID must match its request")
        if self.request_fingerprint != self.request.fingerprint:
            raise ValueError("response analysis request fingerprint does not match its request")
        if self.operating_point_id != self.request.operating_point_id:
            raise ValueError("response analysis operating point must match its request")
        if self.analysis_axis != self.request.axis:
            raise ValueError("response analysis axis must match its request")
        if self.command_support_fraction != self.request.command_support_fraction:
            raise ValueError("response analysis support fraction must match its request")
        if self.response_authority_available != (self.command_support_fraction > 0.0):
            raise ValueError("response-authority availability must match the frozen support fraction")
        if self.response_authority_available != (self.maximum_stable_sample_time_s is not None):
            raise ValueError("only a nonzero-support response has a finite sampled stability limit")
        if (self.sample_time_fraction_of_stability_limit is None) == self.response_authority_available:
            raise ValueError("sample-time stability-limit utilization must follow response availability")
        if self.angle_topology == "bounded":
            if self.angle_limit_rad is None or self.angle_headroom_ratio is None or self.angle_headroom_rad is None:
                raise ValueError("bounded response axes require an angle limit and headroom")
        elif self.angle_limit_rad is not None or self.angle_headroom_ratio is not None or self.angle_headroom_rad is not None:
            raise ValueError("periodic response axes must not advertise a finite angle limit or headroom")
        return self
        ####

    ####


class Pseudo6ResponseComparison(BaseModel):
    """Like-for-like comparison with separate speed, overshoot, and authority results."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    contract: Literal["taoryx.parametric-interceptors.pseudo6-response-comparison/v2"] = "taoryx.parametric-interceptors.pseudo6-response-comparison/v2"
    request: Pseudo6ResponseAnalysisRequest
    request_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    baseline: Pseudo6ResponseAnalysis
    candidate: Pseudo6ResponseAnalysis
    continuous_settling_time_delta_s: float | None = None
    step_overshoot_fraction_delta: float
    discrete_spectral_radius_delta: float
    acceleration_headroom_ratio_delta: float
    body_rate_headroom_ratio_delta: float | None = None
    angle_headroom_ratio_delta: float | None = None
    faster_settling_model_id: str | None = None
    lower_overshoot_model_id: str | None = None
    greater_acceleration_headroom_model_id: str | None = None
    greater_body_rate_headroom_model_id: str | None = None
    greater_angle_headroom_model_id: str | None = None
    tradeoffs: tuple[str, ...]
    overall_winner_model_id: Literal[None] = None
    physical_controller_qualification_claim: Literal[False] = False
    airframe_stability_claim: Literal[False] = False
    nonlinear_or_robust_stability_claim: Literal[False] = False
    claim_boundary: str = (
        "Candidate-minus-baseline deltas compare the same local reduced-order response request. The report deliberately "
        "does not select an overall winner or qualify a physical controller, airframe, nonlinear response, or robustness."
    )

    @model_validator(mode="after")
    def validate_like_for_like_request(self) -> Self:
        if self.request_fingerprint != self.request.fingerprint:
            raise ValueError("response comparison request fingerprint does not match its request")
        if self.baseline.request_fingerprint != self.request_fingerprint:
            raise ValueError("baseline response analysis did not use the comparison request")
        if self.candidate.request_fingerprint != self.request_fingerprint:
            raise ValueError("candidate response analysis did not use the comparison request")
        return self
        ####

    ####


class Pseudo6ResponseOperatingPoint(BaseModel):
    """Concrete force-authority inputs for one local response analysis."""

    model_config = ConfigDict(frozen=True, extra="forbid", str_strip_whitespace=True)

    contract: Literal["taoryx.parametric-interceptors.pseudo6-response-operating-point/v1"] = (
        "taoryx.parametric-interceptors.pseudo6-response-operating-point/v1"
    )
    operating_point_id: str = Field(
        default="local-force-authority-point",
        pattern=r"^[a-z0-9]+(?:[._-][a-z0-9]+)*$",
    )
    dynamic_pressure_pa: float = Field(ge=0.0)
    mass_kg: float = Field(gt=0.0)
    thrust_n: float = Field(ge=0.0)
    commanded_lateral_acceleration_mps2: float = Field(ge=0.0)
    source_sample_time_s: float | None = Field(default=None, ge=0.0)
    source_phase_id: str | None = None
    source_basis: str = "Caller-supplied local force-authority operating point."

    @model_validator(mode="after")
    def validate_finite_values(self) -> Self:
        values = (
            self.dynamic_pressure_pa,
            self.mass_kg,
            self.thrust_n,
            self.commanded_lateral_acceleration_mps2,
        )
        if self.source_sample_time_s is not None:
            values += (self.source_sample_time_s,)
        if any(not math.isfinite(value) for value in values):
            raise ValueError("pseudo-6DOF response operating-point values must be finite")
        return self
        ####

    @classmethod
    def from_sample(
        cls,
        sample: Pseudo6Sample,
        *,
        operating_point_id: str = "runtime-sample-force-authority-point",
        source_basis: str = "Pseudo-6DOF runtime sample supplied by the caller.",
    ) -> Pseudo6ResponseOperatingPoint:
        """Capture only the runtime values needed to reproduce authority support."""

        return cls(
            operating_point_id=operating_point_id,
            dynamic_pressure_pa=sample.dynamic_pressure_pa,
            mass_kg=sample.mass_kg,
            thrust_n=sample.thrust_n,
            commanded_lateral_acceleration_mps2=sample.lateral_acceleration_command_mps2,
            source_sample_time_s=sample.time_s,
            source_phase_id=sample.phase_id,
            source_basis=source_basis,
        )
        ####

    @property
    def fingerprint(self) -> str:
        """Return the stable identity of the operating values and their basis."""

        return _fingerprint(self.model_dump(mode="json"))
        ####

    ####


class Pseudo6OperatingPointAnalysisRequest(BaseModel):
    """Response-analysis settings whose support comes from an operating point."""

    model_config = ConfigDict(frozen=True, extra="forbid", str_strip_whitespace=True)

    analysis_id: str = Field(
        default="operating-point-pseudo6-response-analysis",
        pattern=r"^[a-z0-9]+(?:[._-][a-z0-9]+)*$",
    )
    sample_time_s: float = Field(default=0.05, gt=0.0)
    command_step_rad: float = Field(default=math.radians(10.0), gt=0.0, le=math.pi / 2.0)
    axis: ResponseAxis = "roll"
    settling_band_fraction: float = Field(default=0.02, gt=0.0, lt=1.0)
    stability_tolerance: float = Field(default=1.0e-9, ge=0.0)
    claim_boundary: str = (
        "Analysis freezes the shared reduced-order force-authority support resolved at one explicit operating point. "
        "The operating point is not a gain schedule, flight-envelope proof, physical controller, airframe, actuator, "
        "nonlinear, or robust-stability qualification."
    )

    @model_validator(mode="after")
    def validate_finite_values(self) -> Self:
        if any(
            not math.isfinite(value)
            for value in (
                self.sample_time_s,
                self.command_step_rad,
                self.settling_band_fraction,
                self.stability_tolerance,
            )
        ):
            raise ValueError("pseudo-6DOF operating-point analysis values must be finite")
        return self
        ####

    @property
    def fingerprint(self) -> str:
        """Return the stable identity of the analysis settings."""

        return _fingerprint(self.model_dump(mode="json"))
        ####

    ####


class Pseudo6OperatingPointAnalysisCase(BaseModel):
    """One copy-ready operating point plus its local analysis settings."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    contract: Literal["taoryx.parametric-interceptors.pseudo6-operating-point-analysis-case/v1"] = (
        "taoryx.parametric-interceptors.pseudo6-operating-point-analysis-case/v1"
    )
    operating_point: Pseudo6ResponseOperatingPoint
    analysis: Pseudo6OperatingPointAnalysisRequest = Field(default_factory=Pseudo6OperatingPointAnalysisRequest)

    @property
    def fingerprint(self) -> str:
        """Return the stable identity of the complete file-first analysis case."""

        return _fingerprint(self.model_dump(mode="json"))
        ####

    ####


class Pseudo6ResponseOperatingPointResolution(BaseModel):
    """Profile-specific force authority and runtime-equivalent response support."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    contract: Literal["taoryx.parametric-interceptors.pseudo6-response-operating-point-resolution/v1"] = (
        "taoryx.parametric-interceptors.pseudo6-response-operating-point-resolution/v1"
    )
    operating_point: Pseudo6ResponseOperatingPoint
    operating_point_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    model_id: str
    profile_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    control_configuration: ControlConfiguration
    control_configuration_origin: ValueOrigin
    control_configuration_method: str
    authority_parameter_trace: tuple[Pseudo6ResponseParameterTrace, ...]
    aerodynamic_authority_mps2: float = Field(ge=0.0)
    thrust_vector_authority_mps2: float = Field(ge=0.0)
    combined_unclipped_authority_mps2: float = Field(ge=0.0)
    available_authority_mps2: float = Field(ge=0.0)
    structural_limit_mps2: float = Field(gt=0.0)
    structural_limit_active: bool
    force_authority_available: bool
    command_support_fraction: float = Field(ge=0.0, le=1.0)
    response_support_fraction: float = Field(ge=0.0, le=1.0)
    zero_command_availability_convention_applied: bool
    claim_boundary: str = (
        "Authority uses the same q*S*Cn/m and thrust*sin(vector-angle)/m evaluator as both runtime tiers, clipped by "
        "the resolved structural limit. Response support matches the pseudo-6DOF runtime's zero-command availability "
        "convention. This is a reduced-order operating-point calculation, not hardware authority or controller evidence."
    )

    @model_validator(mode="after")
    def validate_resolution(self) -> Self:
        if self.operating_point_fingerprint != self.operating_point.fingerprint:
            raise ValueError("response operating-point fingerprint does not match its request")
        force_available = self.available_authority_mps2 > 1.0e-12
        if self.force_authority_available != force_available:
            raise ValueError("force-authority availability does not match available acceleration")
        zero_command = self.operating_point.commanded_lateral_acceleration_mps2 <= 1.0e-12
        expected_command_support = (
            1.0
            if zero_command
            else min(
                self.available_authority_mps2 / self.operating_point.commanded_lateral_acceleration_mps2,
                1.0,
            )
        )
        expected_response_support = float(force_available) if zero_command else expected_command_support
        if not math.isclose(self.command_support_fraction, expected_command_support, abs_tol=1.0e-12):
            raise ValueError("command-support fraction does not match the resolved authority")
        if not math.isclose(self.response_support_fraction, expected_response_support, abs_tol=1.0e-12):
            raise ValueError("response-support fraction does not match the runtime convention")
        if self.zero_command_availability_convention_applied != zero_command:
            raise ValueError("zero-command convention status does not match the operating point")
        return self
        ####

    ####


class Pseudo6OperatingPointResponseAnalysis(BaseModel):
    """Local response analysis bound to one reproducible force operating point."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    contract: Literal["taoryx.parametric-interceptors.pseudo6-operating-point-response-analysis/v1"] = (
        "taoryx.parametric-interceptors.pseudo6-operating-point-response-analysis/v1"
    )
    request: Pseudo6OperatingPointAnalysisRequest
    request_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    operating_point_resolution: Pseudo6ResponseOperatingPointResolution
    response_analysis: Pseudo6ResponseAnalysis
    claim_boundary: str

    @model_validator(mode="after")
    def validate_binding(self) -> Self:
        if self.request_fingerprint != self.request.fingerprint:
            raise ValueError("operating-point response request fingerprint does not match its request")
        resolution = self.operating_point_resolution
        analysis = self.response_analysis
        if analysis.model_id != resolution.model_id or analysis.profile_fingerprint != resolution.profile_fingerprint:
            raise ValueError("operating-point response analysis does not match the resolved profile")
        if analysis.operating_point_id != resolution.operating_point.operating_point_id:
            raise ValueError("response analysis does not identify the resolved operating point")
        if not math.isclose(
            analysis.command_support_fraction,
            resolution.response_support_fraction,
            abs_tol=1.0e-12,
        ):
            raise ValueError("response analysis did not use the resolved operating-point support")
        return self
        ####

    ####


class Pseudo6OperatingPointResponseComparison(BaseModel):
    """Matched-force-point comparison with separate authority and tuning deltas."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    contract: Literal["taoryx.parametric-interceptors.pseudo6-operating-point-response-comparison/v1"] = (
        "taoryx.parametric-interceptors.pseudo6-operating-point-response-comparison/v1"
    )
    operating_point: Pseudo6ResponseOperatingPoint
    operating_point_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    request: Pseudo6OperatingPointAnalysisRequest
    request_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    baseline: Pseudo6OperatingPointResponseAnalysis
    candidate: Pseudo6OperatingPointResponseAnalysis
    aerodynamic_authority_delta_mps2: float
    thrust_vector_authority_delta_mps2: float
    available_authority_delta_mps2: float
    response_support_fraction_delta: float
    continuous_settling_time_delta_s: float | None = None
    step_overshoot_fraction_delta: float
    discrete_spectral_radius_delta: float
    acceleration_headroom_ratio_delta: float
    body_rate_headroom_ratio_delta: float | None = None
    angle_headroom_ratio_delta: float | None = None
    greater_available_authority_model_id: str | None = None
    greater_response_support_model_id: str | None = None
    faster_settling_model_id: str | None = None
    lower_overshoot_model_id: str | None = None
    greater_acceleration_headroom_model_id: str | None = None
    greater_body_rate_headroom_model_id: str | None = None
    greater_angle_headroom_model_id: str | None = None
    tradeoffs: tuple[str, ...]
    overall_winner_model_id: Literal[None] = None
    physical_controller_qualification_claim: Literal[False] = False
    airframe_stability_claim: Literal[False] = False
    nonlinear_or_robust_stability_claim: Literal[False] = False
    claim_boundary: str = (
        "Candidate-minus-baseline deltas use one shared reduced-order force operating point, but each profile resolves "
        "its own available authority and response support. The report separates those effects from response tuning, "
        "selects no overall winner, and does not qualify a physical controller, airframe, nonlinear response, or robustness."
    )

    @model_validator(mode="after")
    def validate_matched_operating_point(self) -> Self:
        if self.operating_point_fingerprint != self.operating_point.fingerprint:
            raise ValueError("operating-point comparison fingerprint does not match its operating point")
        if self.request_fingerprint != self.request.fingerprint:
            raise ValueError("operating-point comparison request fingerprint does not match its request")
        if self.baseline.response_analysis.model_id == self.candidate.response_analysis.model_id:
            raise ValueError("operating-point response comparison requires distinct model IDs")
        for label, result in (("baseline", self.baseline), ("candidate", self.candidate)):
            resolution = result.operating_point_resolution
            if resolution.operating_point != self.operating_point:
                raise ValueError(f"{label} operating-point resolution does not match the shared operating point")
            if result.request != self.request:
                raise ValueError(f"{label} operating-point analysis settings do not match the comparison request")
            if result.response_analysis.operating_point_id != self.operating_point.operating_point_id:
                raise ValueError(f"{label} response analysis does not identify the shared operating point")
        return self
        ####

    ####


def analyze_pseudo6_response(
    profile: InterceptorEvidenceProfile | ResolvedInterceptorProfile,
    request: Pseudo6ResponseAnalysisRequest | None = None,
) -> Pseudo6ResponseAnalysis:
    """Analyze one resolved symmetric response law without running a trajectory."""

    resolved = profile if isinstance(profile, ResolvedInterceptorProfile) else resolve_interceptor(profile)
    analysis_request = request or Pseudo6ResponseAnalysisRequest()
    natural_frequency = resolved.number("attitude_bandwidth_rad_s")
    damping_ratio = resolved.number("attitude_damping_ratio")
    support_fraction = analysis_request.command_support_fraction
    response_available = support_fraction > 0.0
    sample_time = analysis_request.sample_time_s
    tolerance = analysis_request.stability_tolerance

    stiffness = support_fraction * natural_frequency**2
    damping_coefficient = 2.0 * support_fraction * damping_ratio * natural_frequency
    effective_natural_frequency = math.sqrt(stiffness)
    effective_damping_ratio = damping_coefficient / (2.0 * effective_natural_frequency) if response_available else 0.0
    continuous_values = np.roots((1.0, damping_coefficient, stiffness))
    continuous_poles = _poles(continuous_values)
    spectral_abscissa = max(item.real for item in continuous_poles)
    continuous_status = _continuous_status(spectral_abscissa, tolerance)

    discrete_matrix: np.ndarray = np.asarray(
        (
            (
                1.0 - stiffness * sample_time**2,
                sample_time * (1.0 - damping_coefficient * sample_time),
            ),
            (
                -stiffness * sample_time,
                1.0 - damping_coefficient * sample_time,
            ),
        ),
        dtype=float,
    )
    discrete_poles = _poles(np.linalg.eigvals(discrete_matrix))
    spectral_radius = max(item.magnitude for item in discrete_poles)
    discrete_status = _discrete_status(spectral_radius, tolerance)
    normalized_sample_time = effective_natural_frequency * sample_time
    maximum_stable_sample_time = (
        2.0 * (math.sqrt(effective_damping_ratio**2 + 1.0) - effective_damping_ratio) / effective_natural_frequency if response_available else None
    )
    sampling_quality: ResponseSamplingQuality = _sampling_quality(normalized_sample_time, discrete_status) if response_available else "not_applicable"

    settling_log = -math.log(analysis_request.settling_band_fraction)
    continuous_settling = settling_log / -spectral_abscissa if spectral_abscissa < 0.0 else None
    if spectral_radius == 0.0:
        discrete_settling = 0.0
    elif spectral_radius < 1.0:
        discrete_settling = math.ceil(math.log(analysis_request.settling_band_fraction) / math.log(spectral_radius)) * sample_time
    else:
        discrete_settling = None

    if response_available and effective_damping_ratio < 1.0:
        overshoot = math.exp(-math.pi * effective_damping_ratio / math.sqrt(1.0 - effective_damping_ratio**2))
        damped_frequency = effective_natural_frequency * math.sqrt(1.0 - effective_damping_ratio**2)
    else:
        overshoot = 0.0
        damped_frequency = 0.0

    command_step = analysis_request.command_step_rad
    max_acceleration = resolved.number("max_body_acceleration_rad_s2")
    max_rate = resolved.number("max_body_rate_rad_s")
    angle_topology, angle_limit, angle_limit_provenance = _angle_limit(resolved, analysis_request.axis)
    effective_command_step = min(command_step, angle_limit) if angle_limit is not None else command_step
    command_step_limited = not math.isclose(command_step, effective_command_step, abs_tol=1.0e-12)
    pre_support_acceleration_demand = effective_command_step * natural_frequency**2
    acceleration_demand = support_fraction * pre_support_acceleration_demand
    characteristic_rate_demand = effective_command_step * effective_natural_frequency
    peak_rate_demand = (
        _linear_peak_rate_demand(
            effective_command_step,
            effective_natural_frequency,
            effective_damping_ratio,
            continuous_poles,
        )
        if response_available
        else 0.0
    )
    trace_ids: tuple[str, ...] = (
        "attitude_bandwidth_rad_s",
        "attitude_damping_ratio",
        "max_body_acceleration_rad_s2",
        "max_body_rate_rad_s",
    )
    if analysis_request.axis == "roll":
        trace_ids += ("max_bank_angle_rad",)
    angle_headroom_ratio = angle_limit / command_step if angle_limit is not None else None
    angle_headroom_rad = angle_limit - command_step if angle_limit is not None else None
    return Pseudo6ResponseAnalysis(
        analysis_id=analysis_request.analysis_id,
        request=analysis_request,
        request_fingerprint=analysis_request.fingerprint,
        model_id=resolved.model_id,
        profile_fingerprint=resolved.fingerprint,
        operating_point_id=analysis_request.operating_point_id,
        analysis_axis=analysis_request.axis,
        angle_topology=angle_topology,
        angle_limit_rad=angle_limit,
        angle_limit_provenance=angle_limit_provenance,
        parameter_trace=tuple(_parameter_trace(resolved, name) for name in trace_ids),
        command_support_fraction=support_fraction,
        response_authority_available=response_available,
        natural_frequency_rad_s=natural_frequency,
        damping_ratio=damping_ratio,
        effective_natural_frequency_rad_s=effective_natural_frequency,
        effective_damping_ratio=effective_damping_ratio,
        damped_natural_frequency_rad_s=damped_frequency,
        continuous_poles=continuous_poles,
        continuous_status=continuous_status,
        continuous_spectral_abscissa_rad_s=spectral_abscissa,
        continuous_stability_margin_rad_s=-spectral_abscissa,
        discrete_poles=discrete_poles,
        discrete_status=discrete_status,
        discrete_spectral_radius=spectral_radius,
        discrete_stability_margin=1.0 - spectral_radius,
        normalized_sample_time=normalized_sample_time,
        maximum_stable_sample_time_s=maximum_stable_sample_time,
        sample_time_fraction_of_stability_limit=(sample_time / maximum_stable_sample_time if maximum_stable_sample_time is not None else None),
        sampling_quality=sampling_quality,
        continuous_settling_time_estimate_s=continuous_settling,
        discrete_settling_time_estimate_s=discrete_settling,
        step_overshoot_fraction=overshoot,
        command_step_rad=command_step,
        effective_command_step_rad=effective_command_step,
        command_step_limited=command_step_limited,
        pre_support_initial_angular_acceleration_demand_rad_s2=pre_support_acceleration_demand,
        initial_angular_acceleration_demand_rad_s2=acceleration_demand,
        characteristic_body_rate_demand_rad_s=characteristic_rate_demand,
        linear_peak_body_rate_demand_rad_s=peak_rate_demand,
        maximum_body_acceleration_rad_s2=max_acceleration,
        authority_scaled_maximum_body_acceleration_rad_s2=support_fraction * max_acceleration,
        maximum_body_rate_rad_s=max_rate,
        acceleration_headroom_ratio=max_acceleration / pre_support_acceleration_demand,
        body_rate_headroom_ratio=(max_rate / peak_rate_demand if peak_rate_demand > 1.0e-15 else None),
        angle_headroom_ratio=angle_headroom_ratio,
        angle_headroom_rad=angle_headroom_rad,
        predicted_acceleration_saturation=pre_support_acceleration_demand > max_acceleration + tolerance,
        predicted_body_rate_saturation=peak_rate_demand > max_rate + tolerance,
        predicted_angle_saturation=angle_limit is not None and command_step > angle_limit + tolerance,
        claim_boundary=analysis_request.claim_boundary,
    )
    ####


def resolve_pseudo6_response_operating_point(
    profile: InterceptorEvidenceProfile | ResolvedInterceptorProfile,
    operating_point: Pseudo6ResponseOperatingPoint,
) -> Pseudo6ResponseOperatingPointResolution:
    """Resolve one operating point through the shared runtime authority equation."""

    resolved = profile if isinstance(profile, ResolvedInterceptorProfile) else resolve_interceptor(profile)
    configuration_parameter = resolved.parameters["control_configuration"]
    configuration = cast(ControlConfiguration, resolved.text("control_configuration"))
    authority = evaluate_control_authority(
        configuration=configuration,
        dynamic_pressure_pa=operating_point.dynamic_pressure_pa,
        reference_area_m2=resolved.number("reference_area_m2"),
        normal_force_coefficient_limit=resolved.number("normal_force_coefficient_limit"),
        thrust_n=operating_point.thrust_n,
        mass_kg=operating_point.mass_kg,
        max_thrust_vector_angle_rad=resolved.number("max_thrust_vector_angle_rad"),
        structural_limit_mps2=resolved.number("max_lateral_acceleration_mps2"),
    )
    command = operating_point.commanded_lateral_acceleration_mps2
    command_support = authority.commanded_support_fraction(command)
    zero_command = command <= 1.0e-12
    force_available = authority.available_mps2 > 1.0e-12
    response_support = float(force_available) if zero_command else command_support
    return Pseudo6ResponseOperatingPointResolution(
        operating_point=operating_point,
        operating_point_fingerprint=operating_point.fingerprint,
        model_id=resolved.model_id,
        profile_fingerprint=resolved.fingerprint,
        control_configuration=configuration,
        control_configuration_origin=configuration_parameter.origin,
        control_configuration_method=configuration_parameter.method,
        authority_parameter_trace=tuple(
            _parameter_trace(resolved, parameter_id)
            for parameter_id in (
                "reference_area_m2",
                "normal_force_coefficient_limit",
                "max_thrust_vector_angle_rad",
                "max_lateral_acceleration_mps2",
            )
        ),
        aerodynamic_authority_mps2=authority.aerodynamic_mps2,
        thrust_vector_authority_mps2=authority.thrust_vector_mps2,
        combined_unclipped_authority_mps2=authority.combined_unclipped_mps2,
        available_authority_mps2=authority.available_mps2,
        structural_limit_mps2=authority.structural_limit_mps2,
        structural_limit_active=authority.structural_limit_active,
        force_authority_available=force_available,
        command_support_fraction=command_support,
        response_support_fraction=response_support,
        zero_command_availability_convention_applied=zero_command,
    )
    ####


def analyze_pseudo6_response_at_operating_point(
    profile: InterceptorEvidenceProfile | ResolvedInterceptorProfile,
    operating_point: Pseudo6ResponseOperatingPoint,
    request: Pseudo6OperatingPointAnalysisRequest | None = None,
) -> Pseudo6OperatingPointResponseAnalysis:
    """Analyze response tuning with support derived from one explicit force point."""

    resolved = profile if isinstance(profile, ResolvedInterceptorProfile) else resolve_interceptor(profile)
    analysis_request = request or Pseudo6OperatingPointAnalysisRequest()
    point_resolution = resolve_pseudo6_response_operating_point(resolved, operating_point)
    frozen_request = Pseudo6ResponseAnalysisRequest(
        analysis_id=analysis_request.analysis_id,
        sample_time_s=analysis_request.sample_time_s,
        command_step_rad=analysis_request.command_step_rad,
        command_support_fraction=point_resolution.response_support_fraction,
        axis=analysis_request.axis,
        settling_band_fraction=analysis_request.settling_band_fraction,
        stability_tolerance=analysis_request.stability_tolerance,
        operating_point_id=operating_point.operating_point_id,
        claim_boundary=analysis_request.claim_boundary,
    )
    analysis = analyze_pseudo6_response(resolved, frozen_request)
    return Pseudo6OperatingPointResponseAnalysis(
        request=analysis_request,
        request_fingerprint=analysis_request.fingerprint,
        operating_point_resolution=point_resolution,
        response_analysis=analysis,
        claim_boundary=(f"{analysis_request.claim_boundary} {point_resolution.claim_boundary}"),
    )
    ####


def compare_pseudo6_responses_at_operating_point(
    baseline: InterceptorEvidenceProfile | ResolvedInterceptorProfile,
    candidate: InterceptorEvidenceProfile | ResolvedInterceptorProfile,
    operating_point: Pseudo6ResponseOperatingPoint,
    request: Pseudo6OperatingPointAnalysisRequest | None = None,
) -> Pseudo6OperatingPointResponseComparison:
    """Compare two profiles at one shared force point without flattening support."""

    analysis_request = request or Pseudo6OperatingPointAnalysisRequest(analysis_id="operating-point-pseudo6-response-comparison")
    baseline_result = analyze_pseudo6_response_at_operating_point(
        baseline,
        operating_point,
        analysis_request,
    )
    candidate_result = analyze_pseudo6_response_at_operating_point(
        candidate,
        operating_point,
        analysis_request,
    )
    baseline_resolution = baseline_result.operating_point_resolution
    candidate_resolution = candidate_result.operating_point_resolution
    baseline_analysis = baseline_result.response_analysis
    candidate_analysis = candidate_result.response_analysis
    return Pseudo6OperatingPointResponseComparison(
        operating_point=operating_point,
        operating_point_fingerprint=operating_point.fingerprint,
        request=analysis_request,
        request_fingerprint=analysis_request.fingerprint,
        baseline=baseline_result,
        candidate=candidate_result,
        aerodynamic_authority_delta_mps2=(candidate_resolution.aerodynamic_authority_mps2 - baseline_resolution.aerodynamic_authority_mps2),
        thrust_vector_authority_delta_mps2=(candidate_resolution.thrust_vector_authority_mps2 - baseline_resolution.thrust_vector_authority_mps2),
        available_authority_delta_mps2=(candidate_resolution.available_authority_mps2 - baseline_resolution.available_authority_mps2),
        response_support_fraction_delta=(candidate_resolution.response_support_fraction - baseline_resolution.response_support_fraction),
        continuous_settling_time_delta_s=_optional_delta(
            candidate_analysis.continuous_settling_time_estimate_s,
            baseline_analysis.continuous_settling_time_estimate_s,
        ),
        step_overshoot_fraction_delta=(candidate_analysis.step_overshoot_fraction - baseline_analysis.step_overshoot_fraction),
        discrete_spectral_radius_delta=(candidate_analysis.discrete_spectral_radius - baseline_analysis.discrete_spectral_radius),
        acceleration_headroom_ratio_delta=(candidate_analysis.acceleration_headroom_ratio - baseline_analysis.acceleration_headroom_ratio),
        body_rate_headroom_ratio_delta=_optional_delta(
            candidate_analysis.body_rate_headroom_ratio,
            baseline_analysis.body_rate_headroom_ratio,
        ),
        angle_headroom_ratio_delta=_optional_delta(
            candidate_analysis.angle_headroom_ratio,
            baseline_analysis.angle_headroom_ratio,
        ),
        greater_available_authority_model_id=_higher_metric_model(
            baseline_analysis.model_id,
            baseline_resolution.available_authority_mps2,
            candidate_analysis.model_id,
            candidate_resolution.available_authority_mps2,
        ),
        greater_response_support_model_id=_higher_metric_model(
            baseline_analysis.model_id,
            baseline_resolution.response_support_fraction,
            candidate_analysis.model_id,
            candidate_resolution.response_support_fraction,
        ),
        faster_settling_model_id=_lower_metric_model(
            baseline_analysis.model_id,
            baseline_analysis.continuous_settling_time_estimate_s,
            candidate_analysis.model_id,
            candidate_analysis.continuous_settling_time_estimate_s,
        ),
        lower_overshoot_model_id=_lower_metric_model(
            baseline_analysis.model_id,
            baseline_analysis.step_overshoot_fraction,
            candidate_analysis.model_id,
            candidate_analysis.step_overshoot_fraction,
        ),
        greater_acceleration_headroom_model_id=_higher_metric_model(
            baseline_analysis.model_id,
            baseline_analysis.acceleration_headroom_ratio,
            candidate_analysis.model_id,
            candidate_analysis.acceleration_headroom_ratio,
        ),
        greater_body_rate_headroom_model_id=_optional_higher_metric_model(
            baseline_analysis.model_id,
            baseline_analysis.body_rate_headroom_ratio,
            candidate_analysis.model_id,
            candidate_analysis.body_rate_headroom_ratio,
        ),
        greater_angle_headroom_model_id=_optional_higher_metric_model(
            baseline_analysis.model_id,
            baseline_analysis.angle_headroom_ratio,
            candidate_analysis.model_id,
            candidate_analysis.angle_headroom_ratio,
        ),
        tradeoffs=_operating_point_comparison_tradeoffs(
            baseline_result,
            candidate_result,
        ),
    )
    ####


def compare_pseudo6_responses(
    baseline: InterceptorEvidenceProfile | ResolvedInterceptorProfile,
    candidate: InterceptorEvidenceProfile | ResolvedInterceptorProfile,
    request: Pseudo6ResponseAnalysisRequest | None = None,
) -> Pseudo6ResponseComparison:
    """Compare two response profiles at the exact same operating request."""

    analysis_request = request or Pseudo6ResponseAnalysisRequest(analysis_id="pseudo6-response-comparison")
    baseline_report = analyze_pseudo6_response(baseline, analysis_request)
    candidate_report = analyze_pseudo6_response(candidate, analysis_request)
    settling_delta = _optional_delta(
        candidate_report.continuous_settling_time_estimate_s,
        baseline_report.continuous_settling_time_estimate_s,
    )
    tradeoffs = _comparison_tradeoffs(baseline_report, candidate_report)
    return Pseudo6ResponseComparison(
        request=analysis_request,
        request_fingerprint=analysis_request.fingerprint,
        baseline=baseline_report,
        candidate=candidate_report,
        continuous_settling_time_delta_s=settling_delta,
        step_overshoot_fraction_delta=(candidate_report.step_overshoot_fraction - baseline_report.step_overshoot_fraction),
        discrete_spectral_radius_delta=(candidate_report.discrete_spectral_radius - baseline_report.discrete_spectral_radius),
        acceleration_headroom_ratio_delta=(candidate_report.acceleration_headroom_ratio - baseline_report.acceleration_headroom_ratio),
        body_rate_headroom_ratio_delta=_optional_delta(
            candidate_report.body_rate_headroom_ratio,
            baseline_report.body_rate_headroom_ratio,
        ),
        angle_headroom_ratio_delta=_optional_delta(
            candidate_report.angle_headroom_ratio,
            baseline_report.angle_headroom_ratio,
        ),
        faster_settling_model_id=_lower_metric_model(
            baseline_report.model_id,
            baseline_report.continuous_settling_time_estimate_s,
            candidate_report.model_id,
            candidate_report.continuous_settling_time_estimate_s,
        ),
        lower_overshoot_model_id=_lower_metric_model(
            baseline_report.model_id,
            baseline_report.step_overshoot_fraction,
            candidate_report.model_id,
            candidate_report.step_overshoot_fraction,
        ),
        greater_acceleration_headroom_model_id=_higher_metric_model(
            baseline_report.model_id,
            baseline_report.acceleration_headroom_ratio,
            candidate_report.model_id,
            candidate_report.acceleration_headroom_ratio,
        ),
        greater_body_rate_headroom_model_id=_optional_higher_metric_model(
            baseline_report.model_id,
            baseline_report.body_rate_headroom_ratio,
            candidate_report.model_id,
            candidate_report.body_rate_headroom_ratio,
        ),
        greater_angle_headroom_model_id=_optional_higher_metric_model(
            baseline_report.model_id,
            baseline_report.angle_headroom_ratio,
            candidate_report.model_id,
            candidate_report.angle_headroom_ratio,
        ),
        tradeoffs=tradeoffs,
    )
    ####


def load_pseudo6_response_analysis_request(
    path: str | Path,
) -> Pseudo6ResponseAnalysisRequest:
    """Load one copy-ready response-analysis request from YAML."""

    source = Path(path)
    try:
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"could not load pseudo-6DOF response analysis {source}: {error}") from error
    if not isinstance(payload, Mapping):
        raise ValueError(f"pseudo-6DOF response analysis {source} must contain one mapping")
    return Pseudo6ResponseAnalysisRequest.model_validate(payload)
    ####


def load_pseudo6_operating_point_analysis_case(
    path: str | Path,
) -> Pseudo6OperatingPointAnalysisCase:
    """Load one operating-point response-analysis case from YAML."""

    source = Path(path)
    try:
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"could not load pseudo-6DOF operating-point analysis {source}: {error}") from error
    if not isinstance(payload, Mapping):
        raise ValueError(f"pseudo-6DOF operating-point analysis {source} must contain one mapping")
    return Pseudo6OperatingPointAnalysisCase.model_validate(payload)
    ####


def _parameter_trace(
    profile: ResolvedInterceptorProfile,
    parameter_id: str,
) -> Pseudo6ResponseParameterTrace:
    parameter = profile.parameters[parameter_id]
    value = parameter.value
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise TypeError(f"response parameter {parameter_id!r} must be numeric")
    return Pseudo6ResponseParameterTrace(
        parameter_id=parameter_id,
        value=float(value),
        unit=parameter.unit or "1",
        origin=parameter.origin,
        method=parameter.method,
        source_record_ids=parameter.source_record_ids,
    )
    ####


def _angle_limit(
    profile: ResolvedInterceptorProfile,
    axis: ResponseAxis,
) -> tuple[ResponseAngleTopology, float | None, str]:
    if axis == "roll":
        parameter = profile.parameters["max_bank_angle_rad"]
        return "bounded", profile.number("max_bank_angle_rad"), (f"resolved max_bank_angle_rad; origin={parameter.origin.value}; method={parameter.method}")
    if axis == "pitch":
        return "bounded", math.radians(89.0), "fixed pseudo-6DOF pitch-coordinate guard at +/-89 deg"
    return "periodic", None, "yaw is wrapped onto the periodic [-pi, pi) coordinate"
    ####


def _linear_peak_rate_demand(
    command_step_rad: float,
    natural_frequency_rad_s: float,
    damping_ratio: float,
    poles: tuple[Pseudo6ResponsePole, Pseudo6ResponsePole],
) -> float:
    """Return the exact peak rate of the unsaturated continuous step response."""

    if damping_ratio < 1.0 - 1.0e-12:
        root = math.sqrt(1.0 - damping_ratio**2)
        return command_step_rad * natural_frequency_rad_s * math.exp(-damping_ratio * math.acos(damping_ratio) / root)
    if math.isclose(damping_ratio, 1.0, abs_tol=1.0e-12):
        return command_step_rad * natural_frequency_rad_s / math.e
    dominant = max(item.real for item in poles)
    fast = min(item.real for item in poles)
    peak_time = math.log(fast / dominant) / (dominant - fast)
    return abs(command_step_rad * natural_frequency_rad_s**2 / (dominant - fast) * (math.exp(dominant * peak_time) - math.exp(fast * peak_time)))
    ####


def _poles(values: np.ndarray) -> tuple[Pseudo6ResponsePole, Pseudo6ResponsePole]:
    poles = tuple(
        sorted(
            (
                Pseudo6ResponsePole(
                    real=float(value.real),
                    imaginary=float(value.imag),
                    magnitude=float(abs(value)),
                )
                for value in values
            ),
            key=lambda item: (item.real, item.imaginary),
            reverse=True,
        )
    )
    if len(poles) != 2:
        raise ValueError("pseudo-6DOF response analysis requires exactly two poles")
    return poles[0], poles[1]
    ####


def _continuous_status(value: float, tolerance: float) -> ResponseStabilityStatus:
    if value < -tolerance:
        return "stable"
    if value > tolerance:
        return "unstable"
    return "marginal"
    ####


def _discrete_status(value: float, tolerance: float) -> ResponseStabilityStatus:
    if value < 1.0 - tolerance:
        return "stable"
    if value > 1.0 + tolerance:
        return "unstable"
    return "marginal"
    ####


def _sampling_quality(
    normalized_sample_time: float,
    stability: ResponseStabilityStatus,
) -> ResponseSamplingQuality:
    if stability == "unstable":
        return "unstable"
    if normalized_sample_time <= 0.1:
        return "fine"
    if normalized_sample_time <= 0.25:
        return "adequate"
    if normalized_sample_time <= 0.5:
        return "coarse"
    return "very_coarse"
    ####


def _optional_delta(candidate: float | None, baseline: float | None) -> float | None:
    if candidate is None or baseline is None:
        return None
    return candidate - baseline
    ####


def _lower_metric_model(
    baseline_id: str,
    baseline: float | None,
    candidate_id: str,
    candidate: float | None,
) -> str | None:
    if baseline is None or candidate is None or math.isclose(baseline, candidate):
        return None
    return baseline_id if baseline < candidate else candidate_id
    ####


def _higher_metric_model(
    baseline_id: str,
    baseline: float,
    candidate_id: str,
    candidate: float,
) -> str | None:
    if math.isclose(baseline, candidate):
        return None
    return baseline_id if baseline > candidate else candidate_id
    ####


def _optional_higher_metric_model(
    baseline_id: str,
    baseline: float | None,
    candidate_id: str,
    candidate: float | None,
) -> str | None:
    if baseline is None or candidate is None:
        return None
    return _higher_metric_model(baseline_id, baseline, candidate_id, candidate)
    ####


def _operating_point_comparison_tradeoffs(
    baseline: Pseudo6OperatingPointResponseAnalysis,
    candidate: Pseudo6OperatingPointResponseAnalysis,
) -> tuple[str, ...]:
    """Describe authority and local-response differences without ranking models."""

    notes: list[str] = []
    baseline_resolution = baseline.operating_point_resolution
    candidate_resolution = candidate.operating_point_resolution
    baseline_analysis = baseline.response_analysis
    candidate_analysis = candidate.response_analysis
    if not math.isclose(
        candidate_resolution.available_authority_mps2,
        baseline_resolution.available_authority_mps2,
    ):
        notes.append(
            "candidate has greater available lateral authority at the shared operating point"
            if candidate_resolution.available_authority_mps2 > baseline_resolution.available_authority_mps2
            else "baseline has greater available lateral authority at the shared operating point"
        )
    if not math.isclose(
        candidate_resolution.response_support_fraction,
        baseline_resolution.response_support_fraction,
    ):
        notes.append(
            "candidate has greater pseudo-6DOF response support at the shared operating point"
            if candidate_resolution.response_support_fraction > baseline_resolution.response_support_fraction
            else "baseline has greater pseudo-6DOF response support at the shared operating point"
        )
    notes.extend(_comparison_tradeoffs(baseline_analysis, candidate_analysis))
    return tuple(dict.fromkeys(notes))
    ####


def _comparison_tradeoffs(
    baseline: Pseudo6ResponseAnalysis,
    candidate: Pseudo6ResponseAnalysis,
) -> tuple[str, ...]:
    notes: list[str] = []
    settling_delta = _optional_delta(
        candidate.continuous_settling_time_estimate_s,
        baseline.continuous_settling_time_estimate_s,
    )
    if settling_delta is not None and not math.isclose(settling_delta, 0.0):
        notes.append("candidate response settles faster" if settling_delta < 0.0 else "baseline response settles faster")
    if not math.isclose(candidate.step_overshoot_fraction, baseline.step_overshoot_fraction):
        notes.append(
            "candidate response has lower analytic step overshoot"
            if candidate.step_overshoot_fraction < baseline.step_overshoot_fraction
            else "baseline response has lower analytic step overshoot"
        )
    if not math.isclose(candidate.acceleration_headroom_ratio, baseline.acceleration_headroom_ratio):
        notes.append(
            "candidate response has greater initial-acceleration headroom"
            if candidate.acceleration_headroom_ratio > baseline.acceleration_headroom_ratio
            else "baseline response has greater initial-acceleration headroom"
        )
    if (
        candidate.body_rate_headroom_ratio is not None
        and baseline.body_rate_headroom_ratio is not None
        and not math.isclose(candidate.body_rate_headroom_ratio, baseline.body_rate_headroom_ratio)
    ):
        notes.append(
            "candidate response has greater characteristic-rate headroom"
            if candidate.body_rate_headroom_ratio > baseline.body_rate_headroom_ratio
            else "baseline response has greater characteristic-rate headroom"
        )
    if (
        baseline.angle_headroom_ratio is not None
        and candidate.angle_headroom_ratio is not None
        and not math.isclose(candidate.angle_headroom_ratio, baseline.angle_headroom_ratio)
    ):
        notes.append(
            "candidate response has greater axis-angle headroom"
            if candidate.angle_headroom_ratio > baseline.angle_headroom_ratio
            else "baseline response has greater axis-angle headroom"
        )
    if baseline.discrete_status != candidate.discrete_status:
        notes.append(f"implemented discrete stability differs: baseline={baseline.discrete_status}, candidate={candidate.discrete_status}")
    if not notes:
        notes.append("no material difference in the advertised response metrics")
    return tuple(notes)
    ####


def _fingerprint(payload: object) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return hashlib.sha256(encoded).hexdigest()
    ####
