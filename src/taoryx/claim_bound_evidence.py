"""Versioned identity binding for release-evidence sidecars.

Numerical reports remain owned by the vehicle family that produced them.  The
small contract here adds the facts a release consumer must be able to rely on:
what kind of evidence the file contains, which compiled composition it covers,
and how the reported outcome relates to the claim boundary.  It deliberately
leaves provider-specific metrics and plots additive.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Final, Literal

from pydantic import AliasChoices, BaseModel, ConfigDict, Field, field_validator, model_validator

RELEASE_EVIDENCE_SCHEMA: Final = "taoryx.claim-bound-release-evidence/v1alpha1"
ReleaseEvidenceKind = Literal["convergence", "robustness", "fidelity_mapping", "disagreement"]
ReleaseEvidenceStatus = Literal["pass", "fail", "partial", "not_applicable"]


class ReleaseEvidenceSubject(BaseModel):
    """Immutable composition identity covered by a release-evidence file."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    composition_id: str = Field(min_length=1)
    composition_identity_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    vehicle_id: str = Field(min_length=1)
    family_id: str = Field(min_length=1)
    mission_id: str = Field(min_length=1)
    fidelity: str = Field(min_length=1)
    control_realization: str = Field(min_length=1)

    @field_validator(
        "composition_id",
        "composition_identity_sha256",
        "vehicle_id",
        "family_id",
        "mission_id",
        "fidelity",
        "control_realization",
    )
    @classmethod
    def nonblank_identity_text(cls, value: str) -> str:
        """Make every subject component usable as an identity key."""

        if not value.strip():
            raise ValueError("release evidence subject fields must not be blank")
        return value
        ####

    ####


class ReleaseEvidenceOutcome(BaseModel):
    """Common machine-readable disposition, independent of family metrics."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    status: ReleaseEvidenceStatus
    passed: bool | None = None

    @model_validator(mode="after")
    def consistent_pass_flag(self) -> ReleaseEvidenceOutcome:
        """Require the Boolean projection when the disposition is decisive."""

        if self.status == "pass" and self.passed is not True:
            raise ValueError("a passing evidence outcome requires passed=True")
        if self.status == "fail" and self.passed is not False:
            raise ValueError("a failing evidence outcome requires passed=False")
        if self.status in {"partial", "not_applicable"} and self.passed is not None:
            raise ValueError(f"a {self.status} evidence outcome requires passed=None")
        return self
        ####

    ####


class ClaimBoundEvidenceArtifact(BaseModel):
    """Strict additive contract carried by current JSON release sidecars.

    ``schema`` stays family-owned so a robust physical-control report can keep
    its own domain schema.  The ``release_evidence_*`` fields are the common
    release contract and are intentionally independent of that provider schema.
    """

    model_config = ConfigDict(frozen=True, extra="allow")

    schema_id: str = Field(alias="schema", min_length=1)
    status: ReleaseEvidenceStatus
    claim_boundary: str = Field(min_length=1)
    release_evidence_schema: Literal["taoryx.claim-bound-release-evidence/v1alpha1"]
    release_evidence_kind: ReleaseEvidenceKind
    release_evidence_subject: ReleaseEvidenceSubject
    release_evidence_outcome: ReleaseEvidenceOutcome

    @field_validator("schema_id", "claim_boundary")
    @classmethod
    def nonblank_envelope_text(cls, value: str) -> str:
        """Reject whitespace-only fields before evidence reaches a catalog."""

        if not value.strip():
            raise ValueError("release evidence envelope fields must not be blank")
        return value
        ####

    @model_validator(mode="after")
    def outcome_agrees_with_top_level_status(self) -> ClaimBoundEvidenceArtifact:
        """Prevent reports whose legacy and common dispositions diverge."""

        if self.release_evidence_outcome.status != self.status:
            raise ValueError("release evidence outcome status must equal top-level status")
        return self
        ####

    def as_dict(self) -> dict[str, object]:
        """Return the JSON-safe payload while retaining all provider details."""

        return self.model_dump(mode="json", by_alias=True)
        ####

    @classmethod
    def from_payload(cls, payload: Mapping[str, object]) -> ClaimBoundEvidenceArtifact:
        """Validate one current release-evidence sidecar."""

        return cls.model_validate(payload)
        ####

    ####


class LegacyClaimBoundEvidenceArtifact(BaseModel):
    """Compatibility envelope for sidecars emitted before the typed contract."""

    model_config = ConfigDict(frozen=True, extra="allow")

    status: str = Field(min_length=1)
    claim_boundary: str = Field(validation_alias=AliasChoices("claim_boundary", "claim"), min_length=1)

    @field_validator("status", "claim_boundary")
    @classmethod
    def nonblank_legacy_text(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("legacy release evidence fields must not be blank")
        return value
        ####

    ####


def bind_release_evidence(
    payload: Mapping[str, object],
    *,
    kind: ReleaseEvidenceKind,
    composition: object,
) -> dict[str, object]:
    """Add the common release binding without changing family report fields.

    Plug-in authors keep producing their domain report; this helper derives the
    common composition identity directly from their compiled composition and
    validates the completed sidecar before it is written.
    """

    def field(name: str) -> str:
        value = getattr(composition, name, None)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"cannot bind release evidence: composition {name!r} is missing")
        return value
        ####

    status = payload.get("status")
    claim_boundary = payload.get("claim_boundary")
    if status not in {"pass", "fail", "partial", "not_applicable"}:
        raise ValueError("cannot bind release evidence: payload status must be pass, fail, partial, or not_applicable")
    if not isinstance(claim_boundary, str) or not claim_boundary.strip():
        raise ValueError("cannot bind release evidence: payload has no nonblank claim boundary")
    passed = payload.get("pass")
    if passed is not None and not isinstance(passed, bool):
        raise ValueError("cannot bind release evidence: payload pass field must be boolean when present")
    bound = {
        **dict(payload),
        "release_evidence_schema": RELEASE_EVIDENCE_SCHEMA,
        "release_evidence_kind": kind,
        "release_evidence_subject": {
            "composition_id": field("id"),
            "composition_identity_sha256": field("identity_sha256"),
            "vehicle_id": field("vehicle_id"),
            "family_id": field("family_id"),
            "mission_id": field("mission"),
            "fidelity": field("fidelity"),
            "control_realization": field("control_realization"),
        },
        "release_evidence_outcome": {"status": status, "passed": passed},
    }
    return ClaimBoundEvidenceArtifact.from_payload(bound).as_dict()
    ####


__all__ = [
    "ClaimBoundEvidenceArtifact",
    "LegacyClaimBoundEvidenceArtifact",
    "RELEASE_EVIDENCE_SCHEMA",
    "ReleaseEvidenceKind",
    "ReleaseEvidenceOutcome",
    "ReleaseEvidenceStatus",
    "ReleaseEvidenceSubject",
    "bind_release_evidence",
]
