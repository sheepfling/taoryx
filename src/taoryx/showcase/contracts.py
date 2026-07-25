"""Common, backend-neutral contracts for family flagship showcases.

The showcase layer describes what a run must prove and how it is evidenced.
It does not own vehicle state, integration, or controller implementation.  A
provider turns a resolved showcase into a run artifact; the renderer consumes
that artifact without reopening a problem file or guessing units.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from taoryx.trajectory.contracts import FidelityProfile

EvidenceGrade = Literal[
    "source",
    "identified",
    "derived",
    "estimated",
    "synthetic",
    "mixed",
    "unavailable",
]
StartContractType = Literal[
    "grounded",
    "trimmed_airborne",
    "hover",
    "air_release",
    "separation",
    "orbit_state",
    "impact_state",
]


class FailureCode(StrEnum):
    """Stable failure vocabulary shared by showcase evaluators."""

    START_CONTRACT_FAILED = "START_CONTRACT_FAILED"
    DATA_DOMAIN_VIOLATION = "DATA_DOMAIN_VIOLATION"
    FRAME_OR_TIME_INVALID = "FRAME_OR_TIME_INVALID"
    NUMERICAL_DIVERGENCE = "NUMERICAL_DIVERGENCE"
    GUIDANCE_INFEASIBLE = "GUIDANCE_INFEASIBLE"
    CONTROL_AUTHORITY_EXCEEDED = "CONTROL_AUTHORITY_EXCEEDED"
    ACTUATOR_LIMIT_EXCEEDED = "ACTUATOR_LIMIT_EXCEEDED"
    RESOURCE_DEPLETED = "RESOURCE_DEPLETED"
    CONTACT_OR_COLLISION_FAILURE = "CONTACT_OR_COLLISION_FAILURE"
    SEGMENT_TRANSITION_FAILED = "SEGMENT_TRANSITION_FAILED"
    TERMINAL_CORRIDOR_MISSED = "TERMINAL_CORRIDOR_MISSED"
    TIMEOUT = "TIMEOUT"
    ARTIFACT_OR_REPLAY_MISMATCH = "ARTIFACT_OR_REPLAY_MISMATCH"
    UNSUPPORTED_CLAIM = "UNSUPPORTED_CLAIM"
####


class StartContract(BaseModel):
    """Family-appropriate initial condition and acceptance contract."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    type: StartContractType
    acceptance: dict[str, Any] = Field(default_factory=dict)
    settling_duration_s: float = Field(default=0.0, ge=0.0)
    hidden_settling_allowed: bool = False
####


class MissionSegmentSpec(BaseModel):
    """One named objective-bearing mission segment."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    objective: str = Field(min_length=1)
    entry_requirements: dict[str, Any] = Field(default_factory=dict)
    success_event: str = Field(min_length=1)
    failure_events: tuple[FailureCode, ...] = ()
    required_controls: tuple[str, ...] = ()
    required_observations: tuple[str, ...] = ()
####


class TerminalContract(BaseModel):
    """Explicit terminal set; timeout is never implicitly success."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    success_event: str = Field(min_length=1)
    corridor: dict[str, Any] = Field(default_factory=dict)
    dwell_s: float = Field(default=0.0, ge=0.0)
    timeout_is_success: Literal[False] = False
    failure_events: tuple[FailureCode, ...] = ()
####


class FamilyShowcaseTemplate(BaseModel):
    """Semantic mission template shared by vehicles in one physical family."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    version: str = Field(min_length=1)
    family: str = Field(min_length=1)
    display_name: str = Field(min_length=1)
    claim: str = Field(min_length=1)
    nonclaims: tuple[str, ...] = ()
    start_contract: StartContract
    segments: tuple[MissionSegmentSpec, ...] = Field(min_length=1)
    required_semantic_controls: tuple[str, ...] = ()
    required_events: tuple[str, ...] = ()
    terminal_contract: TerminalContract
    plot_modules: tuple[str, ...] = ()
    robustness_profile: str | None = None

    @model_validator(mode="after")
    def validate_segments(self) -> FamilyShowcaseTemplate:
        ids = [segment.id for segment in self.segments]
        if len(ids) != len(set(ids)):
            raise ValueError("showcase segment IDs must be unique")
        events = {segment.success_event for segment in self.segments}
        events.add(self.terminal_contract.success_event)
        missing = set(self.required_events).difference(events)
        if missing:
            raise ValueError(f"showcase required events are not produced: {sorted(missing)}")
        return self
        ####
    ####


class VehicleShowcaseBinding(BaseModel):
    """Vehicle/data-package binding for a family showcase."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    family: str = Field(min_length=1)
    version: str = Field(min_length=1)
    vehicle_package: str = Field(min_length=1)
    source_hashes: tuple[str, ...] = ()
    start_state_factory: str = Field(min_length=1)
    envelope: dict[str, Any] = Field(default_factory=dict)
    terminal_tolerances: dict[str, Any] = Field(default_factory=dict)
    supported_segments: tuple[str, ...] = ()
    evidence_grade: EvidenceGrade
    unsupported_behaviors: tuple[str, ...] = ()
####


class FidelityShowcaseRealization(BaseModel):
    """Named realization of a semantic showcase at one dynamics fidelity."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    fidelity: FidelityProfile
    realization_id: str = Field(min_length=1)
    state_schema: tuple[str, ...] = Field(min_length=1)
    semantic_command_mapping: dict[str, str] = Field(default_factory=dict)
    physical_effectors: tuple[str, ...] = ()
    available_physics: tuple[str, ...] = ()
    claim: str = Field(min_length=1)
    nonclaims: tuple[str, ...] = ()
    evidence_grade: EvidenceGrade
####


class ArtifactFile(BaseModel):
    """One self-contained artifact entry with content identity."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    path: str = Field(min_length=1)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    media_type: str = Field(min_length=1)
    required: bool = True
####


class EvidenceBoardSpec(BaseModel):
    """Renderer declaration for the standard card and 4K evidence board."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    profile: str = Field(min_length=1)
    modules: tuple[str, ...] = Field(min_length=1)
    event_marker_policy: Literal["family_local"] = "family_local"
    missing_channel_policy: Literal["unavailable"] = "unavailable"
    units_required: bool = True
####


class ShowcaseRunArtifact(BaseModel):
    """Manifest for a replayable, evaluator-backed showcase run."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: str = "taoryx.showcase/v1alpha1"
    run_id: str = Field(min_length=1)
    showcase_id: str = Field(min_length=1)
    vehicle_binding_id: str = Field(min_length=1)
    fidelity: FidelityProfile
    scenario_contract_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    outcome: Literal[
        "completed",
        "completed_degraded",
        "partial",
        "resource_limited",
        "envelope_limited",
        "time_limited",
        "aborted",
        "numerical_failure",
    ]
    claim: str = Field(min_length=1)
    nonclaims: tuple[str, ...] = ()
    files: tuple[ArtifactFile, ...] = Field(min_length=1)
    board: EvidenceBoardSpec

    @model_validator(mode="after")
    def validate_artifacts(self) -> ShowcaseRunArtifact:
        paths = [item.path for item in self.files]
        if len(paths) != len(set(paths)):
            raise ValueError("showcase artifact paths must be unique")
        required = {item.path for item in self.files if item.required}
        if "manifest.json" not in required:
            raise ValueError("showcase artifact must include a required manifest.json")
        return self
        ####
    ####


__all__ = [
    "ArtifactFile",
    "EvidenceBoardSpec",
    "FailureCode",
    "FidelityShowcaseRealization",
    "FamilyShowcaseTemplate",
    "MissionSegmentSpec",
    "ShowcaseRunArtifact",
    "StartContract",
    "TerminalContract",
    "VehicleShowcaseBinding",
]
####
