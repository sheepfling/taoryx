"""Controller-neutral analysis for CADAC command/response evidence.

These routines consume explicit traces or an explicitly supplied closed-loop
state matrix. They never infer a controller from a model name and never turn
a finite successful trajectory into a formal stability claim.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Literal

import numpy as np
from pydantic import Field, model_validator

from .input_ast import CadacModel


class CadacControllerTrace(CadacModel):
    """One scalar command/response trace at accepted truth boundaries."""

    controller_id: str = Field(min_length=1)
    model_id: str = Field(min_length=1)
    channel_id: str = Field(min_length=1)
    unit: str | None = None
    time_s: tuple[float, ...] = Field(min_length=2)
    reference: tuple[float, ...] = Field(min_length=2)
    response: tuple[float, ...] = Field(min_length=2)
    requested_control: tuple[float, ...] = ()
    realized_control: tuple[float, ...] = ()
    saturated: tuple[bool, ...] = ()
    provenance: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_trace(self) -> "CadacControllerTrace":
        expected = len(self.time_s)
        named = {
            "reference": self.reference,
            "response": self.response,
            "requested_control": self.requested_control,
            "realized_control": self.realized_control,
            "saturated": self.saturated,
        }
        for name, values in named.items():
            if values and len(values) != expected:
                raise ValueError(f"CADAC controller trace {name} must align with time_s")
        if any(later <= earlier for earlier, later in zip(self.time_s, self.time_s[1:], strict=False)):
            raise ValueError("CADAC controller trace times must be strictly increasing")
        numeric = (*self.time_s, *self.reference, *self.response, *self.requested_control, *self.realized_control)
        if any(not math.isfinite(float(value)) for value in numeric):
            raise ValueError("CADAC controller trace values must be finite")
        return self
        ####

    ####


class CadacTimeDomainControllerReport(CadacModel):
    """Finite-run tracking and effort metrics with no formal stability claim."""

    schema_id: str = "taoryx.cadac-controller-time-domain-analysis/v0alpha1"
    controller_id: str
    model_id: str
    channel_id: str
    unit: str | None = None
    sample_count: int = Field(ge=2)
    duration_s: float = Field(gt=0.0)
    rmse: float = Field(ge=0.0)
    peak_absolute_error: float = Field(ge=0.0)
    final_absolute_error: float = Field(ge=0.0)
    overshoot_fraction: float = Field(ge=0.0)
    settling_time_s: float | None = Field(default=None, ge=0.0)
    requested_control_rms: float | None = Field(default=None, ge=0.0)
    realized_control_rms: float | None = Field(default=None, ge=0.0)
    control_tracking_rmse: float | None = Field(default=None, ge=0.0)
    saturation_fraction: float | None = Field(default=None, ge=0.0, le=1.0)
    tracking_pass: bool
    formal_stability_claim: Literal[False] = False
    claim_boundary: str = Field(min_length=1)


class CadacControllerComparisonReport(CadacModel):
    """Like-for-like comparison over one identical time/reference trace."""

    schema_id: str = "taoryx.cadac-controller-comparison/v0alpha1"
    model_id: str
    channel_id: str
    baseline_controller_id: str
    candidate_controller_id: str
    rmse_delta: float
    peak_absolute_error_delta: float
    final_absolute_error_delta: float
    settling_time_delta_s: float | None = None
    saturation_fraction_delta: float | None = None
    lower_rmse_controller_id: str | None = None
    formal_stability_claim: Literal[False] = False
    claim_boundary: str = Field(min_length=1)


class CadacClosedLoopMode(CadacModel):
    """One eigenvalue of an explicitly supplied closed-loop state matrix."""

    real: float
    imaginary: float
    magnitude: float = Field(ge=0.0)


class CadacLocalStabilityReport(CadacModel):
    """Local linear stability result at one declared operating point."""

    schema_id: str = "taoryx.cadac-controller-local-stability/v0alpha1"
    controller_id: str = Field(min_length=1)
    model_id: str = Field(min_length=1)
    operating_point_id: str = Field(min_length=1)
    domain: Literal["continuous", "discrete"]
    sample_time_s: float | None = Field(default=None, gt=0.0)
    state_names: tuple[str, ...] = Field(min_length=1)
    status: Literal["stable", "marginal", "unstable"]
    spectral_abscissa: float | None = None
    spectral_radius: float | None = Field(default=None, ge=0.0)
    modes: tuple[CadacClosedLoopMode, ...] = Field(min_length=1)
    local_linear_claim: Literal[True] = True
    nonlinear_or_robust_stability_claim: Literal[False] = False
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_domain(self) -> "CadacLocalStabilityReport":
        if self.domain == "continuous":
            if self.sample_time_s is not None or self.spectral_abscissa is None or self.spectral_radius is not None:
                raise ValueError("continuous CADAC stability reports require only spectral_abscissa")
        elif self.sample_time_s is None or self.spectral_radius is None or self.spectral_abscissa is not None:
            raise ValueError("discrete CADAC stability reports require sample_time_s and spectral_radius")
        return self
        ####

    ####


def cadac_controller_trace_from_samples(
    samples: Sequence[object],
    *,
    controller_id: str,
    model_id: str,
    reference_channel_id: str,
    response_channel_id: str,
    unit: str | None = None,
    requested_control_channel_id: str | None = None,
    realized_control_channel_id: str | None = None,
    saturation_channel_id: str | None = None,
    provenance: str | None = None,
) -> CadacControllerTrace:
    """Build one finite-run trace from scalar standard trajectory outputs.

    ``samples`` may contain Mission Composition ``TrajectorySample`` objects or
    JSON-shaped mappings with ``time_s`` and ``values`` entries. The extractor
    accepts only scalar finite channels, so callers must choose an explicit
    vector component before analysis rather than silently reducing a multi-axis
    CADAC command or response.
    """

    if len(samples) < 2:
        raise ValueError("CADAC controller trace extraction requires at least two samples")
    ####
    times: list[float] = []
    reference: list[float] = []
    response: list[float] = []
    requested_control: list[float] = []
    realized_control: list[float] = []
    saturated: list[bool] = []
    for index, sample in enumerate(samples):
        time_s, values = _controller_sample_fields(sample, index)
        times.append(time_s)
        reference.append(_controller_scalar(values, reference_channel_id, index))
        response.append(_controller_scalar(values, response_channel_id, index))
        if requested_control_channel_id is not None:
            requested_control.append(_controller_scalar(values, requested_control_channel_id, index))
        ####
        if realized_control_channel_id is not None:
            realized_control.append(_controller_scalar(values, realized_control_channel_id, index))
        ####
        if saturation_channel_id is not None:
            value = values.get(saturation_channel_id)
            if not isinstance(value, bool):
                raise ValueError(f"CADAC controller trace sample {index} channel {saturation_channel_id!r} must be boolean")
            saturated.append(value)
        ####
    ####
    return CadacControllerTrace(
        controller_id=controller_id,
        model_id=model_id,
        channel_id=response_channel_id,
        unit=unit,
        time_s=tuple(times),
        reference=tuple(reference),
        response=tuple(response),
        requested_control=tuple(requested_control),
        realized_control=tuple(realized_control),
        saturated=tuple(saturated),
        provenance=provenance
        or (
            "standard trajectory output extraction: "
            f"{reference_channel_id!r} reference to {response_channel_id!r} response"
        ),
    )
    ####


def _controller_sample_fields(sample: object, index: int) -> tuple[float, Mapping[str, object]]:
    """Read one typed or JSON-shaped standard trajectory sample."""

    if isinstance(sample, Mapping):
        time_value = sample.get("time_s")
        values = sample.get("values")
    else:
        time_value = getattr(sample, "time_s", None)
        values = getattr(sample, "values", None)
    ####
    if isinstance(time_value, bool):
        raise ValueError(f"CADAC controller trace sample {index} has a nonnumeric time_s")
    try:
        time_s = float(time_value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"CADAC controller trace sample {index} has a nonnumeric time_s") from error
    if not math.isfinite(time_s):
        raise ValueError(f"CADAC controller trace sample {index} has a nonfinite time_s")
    if not isinstance(values, Mapping):
        raise ValueError(f"CADAC controller trace sample {index} has no values mapping")
    return time_s, values
    ####


def _controller_scalar(values: Mapping[str, object], channel_id: str, index: int) -> float:
    """Read one finite scalar standard-output value with no implicit vector reduction."""

    value = values.get(channel_id)
    if isinstance(value, bool) or isinstance(value, (Mapping, Sequence)):
        raise ValueError(f"CADAC controller trace sample {index} channel {channel_id!r} must be a scalar")
    try:
        scalar = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"CADAC controller trace sample {index} channel {channel_id!r} must be a scalar") from error
    if not math.isfinite(scalar):
        raise ValueError(f"CADAC controller trace sample {index} channel {channel_id!r} must be finite")
    return scalar
    ####


def analyze_controller_trace(
    trace: CadacControllerTrace,
    *,
    settling_tolerance_fraction: float = 0.02,
    settling_absolute_tolerance: float = 0.0,
) -> CadacTimeDomainControllerReport:
    """Measure finite-horizon tracking, effort, realization, and saturation."""

    if not math.isfinite(settling_tolerance_fraction) or settling_tolerance_fraction <= 0.0:
        raise ValueError("settling_tolerance_fraction must be positive and finite")
    if not math.isfinite(settling_absolute_tolerance) or settling_absolute_tolerance < 0.0:
        raise ValueError("settling_absolute_tolerance must be nonnegative and finite")
    times = np.asarray(trace.time_s, dtype=float)
    reference = np.asarray(trace.reference, dtype=float)
    response = np.asarray(trace.response, dtype=float)
    error = response - reference
    final_reference = float(reference[-1])
    tolerance = max(abs(final_reference) * settling_tolerance_fraction, settling_absolute_tolerance)
    reference_changes = np.flatnonzero(np.abs(np.diff(reference)) > max(1.0e-12, tolerance * 1.0e-6))
    search_start = int(reference_changes[-1] + 1) if len(reference_changes) else 0
    settled_index: int | None = None
    within = np.abs(error) <= tolerance
    for index in range(search_start, len(times)):
        if bool(np.all(within[index:])):
            settled_index = index
            break
        ####
    ####
    transition = final_reference - float(response[search_start])
    if abs(transition) <= 1.0e-12:
        overshoot = 0.0
    elif transition > 0.0:
        overshoot = max(0.0, float(np.max(response[search_start:])) - final_reference) / abs(transition)
    else:
        overshoot = max(0.0, final_reference - float(np.min(response[search_start:]))) / abs(transition)
    ####
    requested_rms = _rms(trace.requested_control) if trace.requested_control else None
    realized_rms = _rms(trace.realized_control) if trace.realized_control else None
    realization_rmse = None
    if trace.requested_control and trace.realized_control:
        realization_rmse = _rms(tuple(realized - requested for requested, realized in zip(trace.requested_control, trace.realized_control, strict=True)))
    ####
    saturation_fraction = sum(trace.saturated) / len(trace.saturated) if trace.saturated else None
    final_error = abs(float(error[-1]))
    return CadacTimeDomainControllerReport(
        controller_id=trace.controller_id,
        model_id=trace.model_id,
        channel_id=trace.channel_id,
        unit=trace.unit,
        sample_count=len(trace.time_s),
        duration_s=float(times[-1] - times[0]),
        rmse=float(math.sqrt(float(np.mean(error * error)))),
        peak_absolute_error=float(np.max(np.abs(error))),
        final_absolute_error=final_error,
        overshoot_fraction=overshoot,
        settling_time_s=None if settled_index is None else float(times[settled_index] - times[search_start]),
        requested_control_rms=requested_rms,
        realized_control_rms=realized_rms,
        control_tracking_rmse=realization_rmse,
        saturation_fraction=saturation_fraction,
        tracking_pass=settled_index is not None and final_error <= tolerance,
        claim_boundary=(
            "Finite-horizon accepted-sample metrics only. Passing tracking does not establish local, nonlinear, robust, gain-scheduled, or envelope stability."
        ),
    )
    ####


def compare_controller_traces(
    baseline: CadacControllerTrace,
    candidate: CadacControllerTrace,
    *,
    settling_tolerance_fraction: float = 0.02,
    settling_absolute_tolerance: float = 0.0,
) -> CadacControllerComparisonReport:
    """Compare controllers only over identical model, channel, time, and reference data."""

    if (baseline.model_id, baseline.channel_id, baseline.unit) != (
        candidate.model_id,
        candidate.channel_id,
        candidate.unit,
    ):
        raise ValueError("CADAC controller comparison requires identical model/channel/unit identity")
    if baseline.time_s != candidate.time_s or baseline.reference != candidate.reference:
        raise ValueError("CADAC controller comparison requires identical time and reference traces")
    baseline_report = analyze_controller_trace(
        baseline,
        settling_tolerance_fraction=settling_tolerance_fraction,
        settling_absolute_tolerance=settling_absolute_tolerance,
    )
    candidate_report = analyze_controller_trace(
        candidate,
        settling_tolerance_fraction=settling_tolerance_fraction,
        settling_absolute_tolerance=settling_absolute_tolerance,
    )
    lower_rmse: str | None
    if abs(candidate_report.rmse - baseline_report.rmse) <= 1.0e-12:
        lower_rmse = None
    elif candidate_report.rmse < baseline_report.rmse:
        lower_rmse = candidate.controller_id
    else:
        lower_rmse = baseline.controller_id
    ####
    return CadacControllerComparisonReport(
        model_id=baseline.model_id,
        channel_id=baseline.channel_id,
        baseline_controller_id=baseline.controller_id,
        candidate_controller_id=candidate.controller_id,
        rmse_delta=candidate_report.rmse - baseline_report.rmse,
        peak_absolute_error_delta=(candidate_report.peak_absolute_error - baseline_report.peak_absolute_error),
        final_absolute_error_delta=(candidate_report.final_absolute_error - baseline_report.final_absolute_error),
        settling_time_delta_s=_optional_delta(
            candidate_report.settling_time_s,
            baseline_report.settling_time_s,
        ),
        saturation_fraction_delta=_optional_delta(
            candidate_report.saturation_fraction,
            baseline_report.saturation_fraction,
        ),
        lower_rmse_controller_id=lower_rmse,
        claim_boundary=("Like-for-like finite-horizon comparison only. A lower error metric is not a formal stability, robustness, or qualification result."),
    )
    ####


def analyze_closed_loop_state_matrix(
    *,
    controller_id: str,
    model_id: str,
    operating_point_id: str,
    state_names: tuple[str, ...],
    closed_loop_state_matrix: tuple[tuple[float, ...], ...],
    domain: Literal["continuous", "discrete"] = "continuous",
    sample_time_s: float | None = None,
    tolerance: float = 1.0e-9,
) -> CadacLocalStabilityReport:
    """Analyze local poles from an explicitly declared complete closed-loop matrix."""

    if not state_names or len(set(state_names)) != len(state_names):
        raise ValueError("CADAC local stability analysis requires unique state names")
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("CADAC local stability tolerance must be finite and nonnegative")
    matrix = np.asarray(closed_loop_state_matrix, dtype=float)
    size = len(state_names)
    if matrix.shape != (size, size) or not np.isfinite(matrix).all():
        raise ValueError("CADAC closed-loop state matrix must be finite and square in declared state order")
    if domain == "continuous" and sample_time_s is not None:
        raise ValueError("continuous CADAC stability analysis does not accept sample_time_s")
    if domain == "discrete" and (sample_time_s is None or not math.isfinite(sample_time_s) or sample_time_s <= 0.0):
        raise ValueError("discrete CADAC stability analysis requires positive finite sample_time_s")
    eigenvalues = np.linalg.eigvals(matrix)
    modes = tuple(
        CadacClosedLoopMode(
            real=float(value.real),
            imaginary=float(value.imag),
            magnitude=float(abs(value)),
        )
        for value in eigenvalues
    )
    if domain == "continuous":
        spectral_abscissa = max(mode.real for mode in modes)
        if spectral_abscissa < -tolerance:
            status: Literal["stable", "marginal", "unstable"] = "stable"
        elif spectral_abscissa > tolerance:
            status = "unstable"
        else:
            status = "marginal"
        return CadacLocalStabilityReport(
            controller_id=controller_id,
            model_id=model_id,
            operating_point_id=operating_point_id,
            domain=domain,
            state_names=state_names,
            status=status,
            spectral_abscissa=spectral_abscissa,
            modes=modes,
            claim_boundary=(
                "Eigenvalue result for the supplied continuous local closed-loop matrix at one declared operating point. "
                "It does not establish nonlinear, robust, scheduled, saturated-actuator, or envelope stability."
            ),
        )
    ####
    spectral_radius = max(mode.magnitude for mode in modes)
    if spectral_radius < 1.0 - tolerance:
        status = "stable"
    elif spectral_radius > 1.0 + tolerance:
        status = "unstable"
    else:
        status = "marginal"
    return CadacLocalStabilityReport(
        controller_id=controller_id,
        model_id=model_id,
        operating_point_id=operating_point_id,
        domain=domain,
        sample_time_s=sample_time_s,
        state_names=state_names,
        status=status,
        spectral_radius=spectral_radius,
        modes=modes,
        claim_boundary=(
            "Eigenvalue result for the supplied discrete local closed-loop matrix at one declared operating point. "
            "It does not establish nonlinear, robust, scheduled, saturated-actuator, or envelope stability."
        ),
    )
    ####


def _rms(values: tuple[float, ...]) -> float:
    array = np.asarray(values, dtype=float)
    return float(math.sqrt(float(np.mean(array * array))))
    ####


def _optional_delta(left: float | None, right: float | None) -> float | None:
    if left is None or right is None:
        return None
    return left - right
    ####


__all__ = [
    "CadacClosedLoopMode",
    "CadacControllerComparisonReport",
    "CadacControllerTrace",
    "CadacLocalStabilityReport",
    "CadacTimeDomainControllerReport",
    "analyze_closed_loop_state_matrix",
    "analyze_controller_trace",
    "cadac_controller_trace_from_samples",
    "compare_controller_traces",
]
