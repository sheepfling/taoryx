"""Bounded multi-scenario fitting for interceptor surrogate scale parameters."""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any, Literal, Self

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from taoryx.optimization import OptimizationStatus
from taoryx.runtime.environment_runtime import EnvironmentProvider
from taoryx.runtime.optimization_runtime import OptimizerBackend, OptimizeRuntime, select_optimizer

from .calibration import (
    DEFAULT_ENVIRONMENT_MODEL_ID,
    DEFAULT_GRAVITY_MODEL_ID,
    InterceptorCalibrationResult,
    InterceptorCalibrationScenario,
    evaluate_interceptor_calibration,
)
from .profile import (
    AssumptionCase,
    InterceptorEvidenceProfile,
    ResolvedInterceptorProfile,
    ValueOrigin,
    calibrated,
)
from .resolver import resolve_interceptor
from .sensor_suite import DEFAULT_SENSOR_SUITE_ID, InterceptorSensorSuite, standard_interceptor_sensor_suite

FitParameterId = Literal[
    "thrust_scale",
    "drag_scale",
    "maneuverability_scale",
    "guidance_time_constant_scale",
]

_FIT_PARAMETER_IDS = {
    "thrust_scale",
    "drag_scale",
    "maneuverability_scale",
    "guidance_time_constant_scale",
}


class InterceptorFitVariable(BaseModel):
    """One bounded simulation-only scale exposed to the fit campaign."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    parameter_id: FitParameterId
    lower_bound: float = Field(gt=0.0)
    upper_bound: float = Field(gt=0.0)
    initial_value: float | None = Field(default=None, gt=0.0)
    unit: Literal["1"] = "1"

    @model_validator(mode="after")
    def validate_interval(self) -> Self:
        values = (self.lower_bound, self.upper_bound)
        if any(not math.isfinite(value) for value in values):
            raise ValueError("interceptor fit-variable bounds must be finite")
        if self.lower_bound >= self.upper_bound:
            raise ValueError("interceptor fit-variable lower bound must be below upper bound")
        if self.initial_value is not None:
            if not math.isfinite(self.initial_value):
                raise ValueError("interceptor fit-variable initial value must be finite")
            if not self.lower_bound <= self.initial_value <= self.upper_bound:
                raise ValueError("interceptor fit-variable initial value must lie within its bounds")
        return self
        ####

    ####


class InterceptorFitCampaign(BaseModel):
    """Immutable scenarios, variables, and numerical settings for one fit."""

    model_config = ConfigDict(frozen=True, extra="forbid", str_strip_whitespace=True)

    campaign_id: str = Field(pattern=r"^[a-z0-9]+(?:[._-][a-z0-9]+)*$")
    scenarios: tuple[InterceptorCalibrationScenario, ...] = Field(min_length=1)
    variables: tuple[InterceptorFitVariable, ...] = Field(min_length=1)
    assumption_case: AssumptionCase = AssumptionCase.NOMINAL
    optimizer_backend: OptimizerBackend = OptimizerBackend.BUILTIN_RQP
    max_iterations: int = Field(default=60, ge=1, le=10_000)
    tolerance: float = Field(default=1.0e-6, gt=0.0)
    derivative_step: float = Field(default=1.0e-3, gt=0.0)
    acceptance_normalized_error: float = Field(default=1.0, ge=0.0)
    objective_contract: Literal["mean_squared_normalized_target_error_v1"] = "mean_squared_normalized_target_error_v1"
    claim_boundary: str = (
        "The campaign fits bounded surrogate scale parameters to declared scenarios. It does not alter source evidence or validate physical weapon performance."
    )

    @model_validator(mode="after")
    def validate_campaign(self) -> Self:
        scenario_ids = tuple(item.scenario_id for item in self.scenarios)
        if len(scenario_ids) != len(set(scenario_ids)):
            raise ValueError("interceptor fit campaign scenario IDs must be unique")
        variable_ids = tuple(item.parameter_id for item in self.variables)
        if len(variable_ids) != len(set(variable_ids)):
            raise ValueError("interceptor fit campaign variable IDs must be unique")
        numeric_target_count = sum(not isinstance(target.target, bool) for scenario in self.scenarios for target in scenario.targets)
        if numeric_target_count == 0:
            raise ValueError("interceptor fit campaign requires at least one numeric calibration target")
        for value, label in (
            (self.tolerance, "tolerance"),
            (self.derivative_step, "derivative_step"),
            (self.acceptance_normalized_error, "acceptance_normalized_error"),
        ):
            if not math.isfinite(value):
                raise ValueError(f"interceptor fit campaign {label} must be finite")
        return self
        ####

    @property
    def fingerprint(self) -> str:
        return _fingerprint(self.model_dump(mode="json"))
        ####

    ####


class FittedInterceptorParameter(BaseModel):
    """One bounded parameter value retained in an immutable fit receipt."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    parameter_id: FitParameterId
    initial_value: float
    fitted_value: float
    lower_bound: float
    upper_bound: float
    unit: Literal["1"] = "1"
    origin: Literal[ValueOrigin.CALIBRATED] = ValueOrigin.CALIBRATED

    @model_validator(mode="after")
    def validate_fitted_value(self) -> Self:
        values = (
            self.initial_value,
            self.fitted_value,
            self.lower_bound,
            self.upper_bound,
        )
        if any(not math.isfinite(value) for value in values):
            raise ValueError("fitted interceptor parameter values must be finite")
        if not self.lower_bound <= self.initial_value <= self.upper_bound:
            raise ValueError("fit receipt initial value lies outside its bounds")
        if not self.lower_bound <= self.fitted_value <= self.upper_bound:
            raise ValueError("fit receipt fitted value lies outside its bounds")
        return self
        ####

    ####


class InterceptorFitReceipt(BaseModel):
    """Portable evidence boundary for one completed numerical fit attempt."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    contract: Literal["taoryx.parametric-interceptors.fit-receipt/v1"] = "taoryx.parametric-interceptors.fit-receipt/v1"
    schema_version: int = 1
    campaign_id: str
    campaign_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    base_model_id: str
    base_profile_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    assumption_case: AssumptionCase
    optimizer_backend: OptimizerBackend
    optimizer_status: OptimizationStatus
    converged: bool
    accepted: bool
    acceptance_normalized_error: float = Field(ge=0.0)
    objective_initial: float = Field(ge=0.0)
    objective_final: float = Field(ge=0.0)
    maximum_normalized_error_initial: float = Field(ge=0.0)
    maximum_normalized_error_final: float = Field(ge=0.0)
    iterations: int = Field(ge=0)
    unique_objective_evaluations: int = Field(ge=1)
    fitted_parameters: tuple[FittedInterceptorParameter, ...] = Field(min_length=1)
    baseline_results: tuple[InterceptorCalibrationResult, ...] = Field(min_length=1)
    fitted_results: tuple[InterceptorCalibrationResult, ...] = Field(min_length=1)
    fitted_profile_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    fitted_resolved_profile_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    claim_boundary: str = (
        "Fitted values are calibrated simulation assumptions. This receipt does not relabel them "
        "as observations, validate the archetype, or establish weapon performance."
    )

    @model_validator(mode="after")
    def validate_receipt(self) -> Self:
        if self.converged != (self.optimizer_status is OptimizationStatus.CONVERGED):
            raise ValueError("fit receipt converged flag must match optimizer status")
        if self.accepted != (self.maximum_normalized_error_final <= self.acceptance_normalized_error):
            raise ValueError("fit receipt accepted flag must match its normalized-error threshold")
        if self.objective_final > self.objective_initial + 1.0e-12:
            raise ValueError("fit receipt final objective must not exceed its initial objective")
        if len(self.baseline_results) != len(self.fitted_results):
            raise ValueError("fit receipt baseline and fitted scenario counts must match")
        baseline_scenarios = tuple(item.scenario_fingerprint for item in self.baseline_results)
        fitted_scenarios = tuple(item.scenario_fingerprint for item in self.fitted_results)
        if baseline_scenarios != fitted_scenarios:
            raise ValueError("fit receipt baseline and fitted scenarios must match in order")
        return self
        ####

    @property
    def fingerprint(self) -> str:
        return _fingerprint(self.model_dump(mode="json"))
        ####

    ####


def fit_interceptor_profile(
    profile: InterceptorEvidenceProfile,
    campaign: InterceptorFitCampaign,
    *,
    environment_models: Mapping[str, EnvironmentProvider] | None = None,
    gravity_models: Mapping[str, Callable[[float], float]] | None = None,
    sensor_suites: Mapping[str, InterceptorSensorSuite] | None = None,
) -> InterceptorFitReceipt:
    """Fit bounded scale parameters while leaving the input profile untouched."""

    _require_fit_eligible_profile_fields(profile, campaign.variables)
    initial = tuple(_initial_value(profile, item) for item in campaign.variables)
    bounds = tuple((item.lower_bound, item.upper_bound) for item in campaign.variables)
    cache: dict[
        tuple[float, ...],
        tuple[float, tuple[InterceptorCalibrationResult, ...]],
    ] = {}

    def assess(point: tuple[float, ...]) -> tuple[float, tuple[InterceptorCalibrationResult, ...]]:
        key = tuple(float(value) for value in point)
        cached = cache.get(key)
        if cached is not None:
            return cached
        candidate = _profile_with_fit_values(profile, campaign, key)
        resolved = resolve_interceptor(candidate, assumption_case=campaign.assumption_case)
        results = tuple(
            evaluate_interceptor_calibration(
                resolved,
                scenario,
                environment=_fit_dependency(
                    scenario.environment_model_id,
                    DEFAULT_ENVIRONMENT_MODEL_ID,
                    environment_models,
                    "environment model",
                ),
                gravity_acceleration=_fit_dependency(
                    scenario.gravity_model_id,
                    DEFAULT_GRAVITY_MODEL_ID,
                    gravity_models,
                    "gravity model",
                ),
                sensor_suite=_fit_sensor_suite(scenario.sensor_suite_id, sensor_suites),
            )
            for scenario in campaign.scenarios
        )
        normalized_errors = tuple(metric.normalized_error for result in results for metric in result.evaluation.metrics if metric.normalized_error is not None)
        if not normalized_errors:
            raise ValueError("interceptor fit campaign emitted no scoreable normalized errors")
        objective = sum(value * value for value in normalized_errors) / len(normalized_errors)
        cache[key] = (objective, results)
        return cache[key]
        ####

    initial_objective, baseline_results = assess(initial)
    maximum_initial_error = _maximum_normalized_error(baseline_results)
    selected_backend = select_optimizer(campaign.optimizer_backend, has_constraints=False)
    optimizer = OptimizeRuntime(
        objective=lambda point: assess(point)[0],
        bounds=bounds,
        backend=selected_backend,
        tolerance=campaign.tolerance,
        derivative_step=campaign.derivative_step,
    )
    optimized = optimizer.run(initial, max_iterations=campaign.max_iterations)
    fitted_objective, fitted_results = assess(optimized.parameters)
    if fitted_objective > initial_objective:
        final_point = initial
        fitted_objective = initial_objective
        fitted_results = baseline_results
    else:
        final_point = optimized.parameters
    maximum_final_error = _maximum_normalized_error(fitted_results)
    fitted_profile = _profile_with_fit_values(profile, campaign, final_point)
    fitted_resolved = resolve_interceptor(
        fitted_profile,
        assumption_case=campaign.assumption_case,
    )
    parameters = tuple(
        FittedInterceptorParameter(
            parameter_id=variable.parameter_id,
            initial_value=initial[index],
            fitted_value=final_point[index],
            lower_bound=variable.lower_bound,
            upper_bound=variable.upper_bound,
        )
        for index, variable in enumerate(campaign.variables)
    )
    return InterceptorFitReceipt(
        campaign_id=campaign.campaign_id,
        campaign_fingerprint=campaign.fingerprint,
        base_model_id=profile.model_id,
        base_profile_fingerprint=profile.fingerprint,
        assumption_case=campaign.assumption_case,
        optimizer_backend=selected_backend,
        optimizer_status=optimized.status,
        converged=optimized.converged,
        accepted=maximum_final_error <= campaign.acceptance_normalized_error,
        acceptance_normalized_error=campaign.acceptance_normalized_error,
        objective_initial=initial_objective,
        objective_final=fitted_objective,
        maximum_normalized_error_initial=maximum_initial_error,
        maximum_normalized_error_final=maximum_final_error,
        iterations=optimized.iterations,
        unique_objective_evaluations=len(cache),
        fitted_parameters=parameters,
        baseline_results=baseline_results,
        fitted_results=fitted_results,
        fitted_profile_fingerprint=fitted_profile.fingerprint,
        fitted_resolved_profile_fingerprint=fitted_resolved.fingerprint,
    )
    ####


def apply_interceptor_fit_receipt(
    profile: InterceptorEvidenceProfile,
    receipt: InterceptorFitReceipt,
    *,
    allow_unaccepted: bool = False,
) -> InterceptorEvidenceProfile:
    """Apply one exact receipt to the profile it was fitted from."""

    if profile.model_id != receipt.base_model_id:
        raise ValueError("fit receipt model ID does not match the supplied profile")
    if profile.fingerprint != receipt.base_profile_fingerprint:
        raise ValueError("fit receipt base profile fingerprint does not match the supplied profile")
    if not receipt.accepted and not allow_unaccepted:
        raise ValueError("fit receipt did not meet its acceptance threshold; pass allow_unaccepted=True to inspect its candidate")
    variables = tuple(
        InterceptorFitVariable(
            parameter_id=item.parameter_id,
            lower_bound=item.lower_bound,
            upper_bound=item.upper_bound,
            initial_value=item.initial_value,
        )
        for item in receipt.fitted_parameters
    )
    _require_fit_eligible_profile_fields(profile, variables)
    fitted = _profile_with_values_from_receipt(profile, receipt)
    if fitted.fingerprint != receipt.fitted_profile_fingerprint:
        raise ValueError("applied fit profile does not match the receipt fingerprint")
    resolved = resolve_interceptor(fitted, assumption_case=receipt.assumption_case)
    if resolved.fingerprint != receipt.fitted_resolved_profile_fingerprint:
        raise ValueError("applied resolved profile does not match the receipt fingerprint")
    return fitted
    ####


def resolve_interceptor_fit_receipt(
    profile: InterceptorEvidenceProfile,
    receipt: InterceptorFitReceipt,
    *,
    allow_unaccepted: bool = False,
) -> ResolvedInterceptorProfile:
    """Apply and resolve a fit receipt in one developer-facing call."""

    fitted = apply_interceptor_fit_receipt(
        profile,
        receipt,
        allow_unaccepted=allow_unaccepted,
    )
    return resolve_interceptor(fitted, assumption_case=receipt.assumption_case)
    ####


def load_interceptor_fit_campaign(path: str | Path) -> InterceptorFitCampaign:
    """Load one YAML fit campaign containing explicit nested scenarios."""

    return _load_model(path, InterceptorFitCampaign, "interceptor fit campaign")
    ####


def load_interceptor_fit_receipt(path: str | Path) -> InterceptorFitReceipt:
    """Load one JSON or YAML fit receipt."""

    return _load_model(path, InterceptorFitReceipt, "interceptor fit receipt")
    ####


def write_interceptor_fit_receipt(
    receipt: InterceptorFitReceipt,
    path: str | Path,
) -> Path:
    """Write one deterministic, JSON-serializable fit receipt."""

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        receipt.model_dump_json(indent=2) + "\n",
        encoding="utf-8",
    )
    return destination
    ####


def _initial_value(
    profile: InterceptorEvidenceProfile,
    variable: InterceptorFitVariable,
) -> float:
    if variable.initial_value is not None:
        return variable.initial_value
    evidence = getattr(profile, variable.parameter_id)
    if evidence is None:
        if not variable.lower_bound <= 1.0 <= variable.upper_bound:
            raise ValueError(f"fit parameter {variable.parameter_id!r} defaults to 1.0, which lies outside campaign bounds; declare initial_value explicitly")
        return 1.0
    value = evidence.value
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise TypeError(f"fit parameter {variable.parameter_id!r} must be numeric")
    numeric = float(value)
    if not variable.lower_bound <= numeric <= variable.upper_bound:
        raise ValueError(f"profile value for fit parameter {variable.parameter_id!r} lies outside campaign bounds")
    return numeric
    ####


def _fit_dependency[DependencyT](
    identifier: str,
    built_in_identifier: str,
    dependencies: Mapping[str, DependencyT] | None,
    label: str,
) -> DependencyT | None:
    if identifier == built_in_identifier:
        return None
    if dependencies is None or identifier not in dependencies:
        raise KeyError(f"fit campaign requires registered {label} {identifier!r}")
    return dependencies[identifier]
    ####


def _fit_sensor_suite(
    identifier: str,
    sensor_suites: Mapping[str, InterceptorSensorSuite] | None,
) -> InterceptorSensorSuite:
    if identifier == DEFAULT_SENSOR_SUITE_ID:
        return standard_interceptor_sensor_suite()
    if sensor_suites is None or identifier not in sensor_suites:
        raise KeyError(f"fit campaign requires registered sensor suite {identifier!r}")
    return sensor_suites[identifier]
    ####


def _require_fit_eligible_profile_fields(
    profile: InterceptorEvidenceProfile,
    variables: tuple[InterceptorFitVariable, ...],
) -> None:
    for variable in variables:
        if variable.parameter_id not in _FIT_PARAMETER_IDS:
            raise ValueError(f"unsupported interceptor fit parameter {variable.parameter_id!r}")
        evidence = getattr(profile, variable.parameter_id)
        if evidence is None:
            continue
        if evidence.origin not in {
            ValueOrigin.SIMULATION_ASSUMPTION,
            ValueOrigin.CALIBRATED,
        }:
            raise ValueError(
                f"fit parameter {variable.parameter_id!r} has protected origin "
                f"{evidence.origin.value!r}; only simulation assumptions or prior calibrated values may be refit"
            )
    ####


def _profile_with_fit_values(
    profile: InterceptorEvidenceProfile,
    campaign: InterceptorFitCampaign,
    point: tuple[float, ...],
) -> InterceptorEvidenceProfile:
    method = f"bounded multi-scenario fit campaign {campaign.campaign_id}; campaign_fingerprint={campaign.fingerprint}"
    updates: dict[str, Any] = {
        variable.parameter_id: calibrated(
            point[index],
            unit="1",
            method=method,
        )
        for index, variable in enumerate(campaign.variables)
    }
    return profile.model_copy(update=updates)
    ####


def _profile_with_values_from_receipt(
    profile: InterceptorEvidenceProfile,
    receipt: InterceptorFitReceipt,
) -> InterceptorEvidenceProfile:
    method = f"bounded multi-scenario fit campaign {receipt.campaign_id}; campaign_fingerprint={receipt.campaign_fingerprint}"
    return profile.model_copy(
        update={
            item.parameter_id: calibrated(
                item.fitted_value,
                unit=item.unit,
                method=method,
            )
            for item in receipt.fitted_parameters
        }
    )
    ####


def _maximum_normalized_error(
    results: tuple[InterceptorCalibrationResult, ...],
) -> float:
    errors = tuple(metric.normalized_error for result in results for metric in result.evaluation.metrics if metric.normalized_error is not None)
    if not errors:
        raise ValueError("interceptor fit results contain no normalized target errors")
    return max(errors)
    ####


def _load_model[ModelT: BaseModel](
    path: str | Path,
    model: type[ModelT],
    label: str,
) -> ModelT:
    source = Path(path)
    try:
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"could not load {label} {source}: {error}") from error
    if not isinstance(payload, Mapping):
        raise ValueError(f"{label} {source} must contain one mapping")
    return model.model_validate(payload)
    ####


def _fingerprint(payload: object) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return hashlib.sha256(encoded).hexdigest()
    ####


__all__ = [
    "FitParameterId",
    "FittedInterceptorParameter",
    "InterceptorFitCampaign",
    "InterceptorFitReceipt",
    "InterceptorFitVariable",
    "apply_interceptor_fit_receipt",
    "fit_interceptor_profile",
    "load_interceptor_fit_campaign",
    "load_interceptor_fit_receipt",
    "resolve_interceptor_fit_receipt",
    "write_interceptor_fit_receipt",
]
####
