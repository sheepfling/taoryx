"""CADAC-specific readiness facts above the common composition contracts.

The common model schema describes what a host can call today. This companion
contract records the migration boundaries that must not be inferred from a
batch-capable source reconstruction: persistent stepping, environment
ownership, and controller-analysis readiness.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Literal

from pydantic import Field, model_validator

from taoryx.trajectory.configuration_contract import TrajectoryModelMetadata

from .input_ast import CadacModel
from .sensor_integration import CadacSensorIntegrationContract, build_cadac_sensor_integration_contract

CadacReadiness = Literal["available", "development", "blocked", "not_applicable"]
CadacControllerOwnership = Literal[
    "external_at_source_boundary",
    "source_owned",
    "fixed_source_program",
    "none",
]


class CadacStepIntegrationContract(CadacModel):
    """Persistent execution readiness for one advertised CADAC model."""

    status: CadacReadiness
    session_contract: str = "taoryx.mission-composition-session/v1"
    state_semantics: Literal["persistent_native_state", "batch_only", "not_executable"]
    action_semantics: Literal["held_step_action", "configuration_fixed", "provider_internal", "none"]
    blockers: tuple[str, ...] = ()
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_status(self) -> "CadacStepIntegrationContract":
        if self.status == "available" and self.state_semantics != "persistent_native_state":
            raise ValueError("available CADAC stepping requires persistent native state")
        if self.status == "blocked" and not self.blockers:
            raise ValueError("blocked CADAC stepping requires explicit blockers")
        return self
        ####

    ####


class CadacEnvironmentIntegrationContract(CadacModel):
    """Ownership of atmosphere, gravity, wind, and frame transformations."""

    execution_profile: Literal["cadac_compat", "taoryx_native", "not_bound"]
    atmosphere_owner: Literal["cadac_compatibility_runtime", "taoryx_shared_runtime", "not_bound"]
    gravity_owner: Literal["cadac_compatibility_runtime", "taoryx_shared_runtime", "not_bound"]
    host_environment_status: CadacReadiness
    source_environment_retained_for_parity: bool
    blockers: tuple[str, ...] = ()
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_ownership(self) -> "CadacEnvironmentIntegrationContract":
        if self.execution_profile == "taoryx_native":
            if self.atmosphere_owner != "taoryx_shared_runtime" or self.gravity_owner != "taoryx_shared_runtime":
                raise ValueError("taoryx_native CADAC execution must use shared atmosphere and gravity ownership")
            if self.source_environment_retained_for_parity:
                raise ValueError("taoryx_native CADAC execution cannot execute source compatibility environment math")
        if self.host_environment_status == "blocked" and not self.blockers:
            raise ValueError("blocked CADAC environment integration requires explicit blockers")
        return self
        ####

    ####


class CadacControllerAnalysisContract(CadacModel):
    """Analysis methods supported by the published command/response surface."""

    ownership: CadacControllerOwnership
    command_output_channel_ids: tuple[str, ...] = ()
    response_output_channel_ids: tuple[str, ...] = ()
    time_domain_analysis: CadacReadiness
    controller_comparison: CadacReadiness
    local_linear_stability: CadacReadiness
    frequency_domain_margins: CadacReadiness
    blockers: tuple[str, ...] = ()
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_analysis(self) -> "CadacControllerAnalysisContract":
        if self.time_domain_analysis == "available" and (not self.command_output_channel_ids or not self.response_output_channel_ids):
            raise ValueError("available CADAC time-domain analysis requires command and response outputs")
        if self.controller_comparison == "available" and self.time_domain_analysis != "available":
            raise ValueError("CADAC controller comparison requires time-domain analysis")
        if any(status == "blocked" for status in (self.time_domain_analysis, self.local_linear_stability, self.frequency_domain_margins)) and not self.blockers:
            raise ValueError("blocked CADAC controller analysis requires explicit blockers")
        return self
        ####

    ####


class CadacModelIntegrationContract(CadacModel):
    """One discoverable CADAC model's strict host-integration boundary."""

    schema_id: str = "taoryx.cadac-model-integration/v0alpha1"
    model_id: str = Field(min_length=1)
    model_version: str = Field(min_length=1)
    step: CadacStepIntegrationContract
    environment: CadacEnvironmentIntegrationContract
    controller_analysis: CadacControllerAnalysisContract
    sensor_integration: CadacSensorIntegrationContract
    claim_boundary: str = Field(min_length=1)


def build_cadac_model_integration_contract(
    model: TrajectoryModelMetadata,
) -> CadacModelIntegrationContract:
    """Build readiness facts from the exact installed/discovered model record."""

    has_batch = "batch" in model.common_runner_operations
    has_step = "step" in model.common_runner_operations
    external_controls = tuple(channel for realization in model.realizations for channel in realization.controls.channels if channel.operations)
    internal_control = any(realization.controls.status == "internally_generated" for realization in model.realizations)
    requested_ids, realized_ids = _external_control_output_ids(external_controls)
    if not requested_ids and not realized_ids:
        requested_ids, realized_ids = _source_control_output_ids(model)
    ####

    if has_step:
        step = CadacStepIntegrationContract(
            status="available",
            state_semantics="persistent_native_state",
            action_semantics="held_step_action" if external_controls else "provider_internal",
            claim_boundary="The model advertises an exact registered persistent session boundary.",
        )
    elif has_batch:
        step = CadacStepIntegrationContract(
            status="blocked",
            state_semantics="batch_only",
            action_semantics="configuration_fixed" if external_controls else "provider_internal",
            blockers=(
                "no registered persistent Mission Composition session owns the model's hidden integrator, actuator, controller, and event state",
                "batch execution must not be relabeled as step or reconstructed by resetting hidden state between calls",
            ),
            claim_boundary=(
                "The installed source reconstruction is batch-capable only. Controls remain fixed in the prepared "
                "configuration unless a true native session is added."
            ),
        )
    else:
        step = CadacStepIntegrationContract(
            status="not_applicable",
            state_semantics="not_executable",
            action_semantics="none",
            claim_boundary="This discovered or embedded actor has no standalone executable session binding.",
        )
    ####

    if has_batch:
        environment = CadacEnvironmentIntegrationContract(
            execution_profile="cadac_compat",
            atmosphere_owner="cadac_compatibility_runtime",
            gravity_owner="cadac_compatibility_runtime",
            host_environment_status="blocked",
            source_environment_retained_for_parity=True,
            blockers=(
                "the installed compatibility runtime still evaluates the CADAC source atmosphere/gravity path internally",
                "a taoryx_native session must inject the shared environment and gravity providers before duplicate source environment execution can be removed",
            ),
            claim_boundary=(
                "Legacy environment equations are retained only inside cadac_compat for source parity. They are not "
                "advertised as a second host environment service or as the future taoryx_native path."
            ),
        )
    else:
        environment = CadacEnvironmentIntegrationContract(
            execution_profile="not_bound",
            atmosphere_owner="not_bound",
            gravity_owner="not_bound",
            host_environment_status="not_applicable",
            source_environment_retained_for_parity=False,
            claim_boundary="No standalone runtime is bound, so no environment implementation executes.",
        )
    ####

    if external_controls:
        ownership: CadacControllerOwnership = "external_at_source_boundary"
    elif internal_control and requested_ids:
        ownership = "source_owned"
    elif internal_control:
        ownership = "fixed_source_program"
    else:
        ownership = "none"
    ####
    time_ready = bool(has_batch and requested_ids and realized_ids)
    analysis_blockers: list[str] = []
    if has_batch and not time_ready and ownership in {"source_owned", "external_at_source_boundary"}:
        analysis_blockers.append("command and realized-response channels are not both published in the standard output schema")
    if has_batch and ownership != "none":
        analysis_blockers.append(
            "no CADAC StandardFamilyAdapter yet publishes trim, controller state, actuator dynamics, and closed-loop linearization coordinates"
        )
    controller = CadacControllerAnalysisContract(
        ownership=ownership,
        command_output_channel_ids=requested_ids,
        response_output_channel_ids=realized_ids,
        time_domain_analysis="available" if time_ready else ("blocked" if has_batch and ownership == "source_owned" else "not_applicable"),
        controller_comparison="available" if time_ready else "not_applicable",
        local_linear_stability="blocked" if has_batch and ownership != "none" else "not_applicable",
        frequency_domain_margins="blocked" if has_batch and ownership != "none" else "not_applicable",
        blockers=tuple(analysis_blockers),
        claim_boundary=(
            "Finite-run command/response analysis and like-for-like controller comparison are allowed when their channels are "
            "published. Formal local stability or frequency margins require a declared operating point and complete closed-loop "
            "state model; controller presence alone is not stability evidence."
        ),
    )
    return CadacModelIntegrationContract(
        model_id=model.id,
        model_version=model.version,
        step=step,
        environment=environment,
        controller_analysis=controller,
        sensor_integration=build_cadac_sensor_integration_contract(model.id, persistent_session=has_step),
        claim_boundary=(
            "This contract records integration readiness only and does not promote numerical parity, stability, robustness, or source equivalence."
        ),
    )
    ####


def _external_control_output_ids(channels: Sequence[object]) -> tuple[tuple[str, ...], tuple[str, ...]]:
    requested: list[str] = []
    realized: list[str] = []
    for channel in channels:
        provider_binding = getattr(channel, "provider_binding", None)
        if not isinstance(provider_binding, Mapping):
            continue
        ####
        evidence = provider_binding.get("output_evidence")
        if not isinstance(evidence, Mapping):
            continue
        ####
        requested_endpoint = evidence.get("requested")
        if isinstance(requested_endpoint, Mapping):
            channel_id = requested_endpoint.get("channel_id")
            if isinstance(channel_id, str):
                requested.append(channel_id)
            ####
        ####
        realized_endpoints = evidence.get("realized")
        if isinstance(realized_endpoints, Sequence):
            for endpoint in realized_endpoints:
                if not isinstance(endpoint, Mapping):
                    continue
                ####
                channel_id = endpoint.get("channel_id")
                if isinstance(channel_id, str):
                    realized.append(channel_id)
                ####
            ####
        ####
    ####
    return tuple(dict.fromkeys(requested)), tuple(dict.fromkeys(realized))
    ####


def _source_control_output_ids(model: TrajectoryModelMetadata) -> tuple[tuple[str, ...], tuple[str, ...]]:
    output_ids = tuple(channel.id for channel in model.output_schema.channels)
    commands = tuple(
        channel_id
        for channel_id in output_ids
        if any(token in channel_id for token in ("commanded_", "_command_", "_command_g", "_command_deg")) and not channel_id.startswith("requested_")
    )
    responses = tuple(
        channel_id
        for channel_id in output_ids
        if any(
            token in channel_id
            for token in (
                "achieved_",
                "response_",
                "_response_",
                "normal_load_factor_g",
                "bank_state_deg",
                "lateral_acceleration_g",
                "normal_acceleration_g",
            )
        )
        and channel_id not in commands
    )
    return commands, responses
    ####


__all__ = [
    "CadacControllerAnalysisContract",
    "CadacEnvironmentIntegrationContract",
    "CadacModelIntegrationContract",
    "CadacSensorIntegrationContract",
    "CadacReadiness",
    "CadacStepIntegrationContract",
    "build_cadac_model_integration_contract",
]
