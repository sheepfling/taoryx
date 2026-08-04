"""Machine-readable pseudo-6DOF profile contracts.

The profile catalog is deliberately declarative. It describes what a family
is allowed to claim at the pseudo-6DOF boundary; it does not replace a family
runner or manufacture physical moments/effectors. A profile with
``rigid_body_reuse`` is the explicit exception used for tumbling bodies. A
``DirectWrenchProfile`` is the explicit bridge between a response-law
surrogate and physical-effector allocation: it integrates the rigid-body
plant with a bounded generalized force/moment command without claiming that
real effectors produced that command.
"""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Literal, cast

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from ..fidelity_contracts import (
    CANONICAL_FIDELITY_TIERS,
    QUALIFIED_FIDELITY_STATUSES,
    FidelityTier,
    LegacyFidelityTier,
)
from ..fidelity_lowering import LoweringCandidate, select_canonical_lowering
from ..fidelity_lowering import LoweringStatus as SharedLoweringStatus
from ..vehicle_registry import ROOT

Pseudo6DOFModelKind = Literal[
    "attitude_response_surrogate",
    "source_derived_reduced_model",
    "control_surface_surrogate",
    "thrust_vector_surrogate",
    "rigid_body_reuse",
]
ProfileStatus = Literal["planned", "development", "nominal_case_pass", "promoted"]
AreaPolicy = Literal["not_applicable", "average_projected_area", "steady_stage_cross_section", "geometry_schedule"]
ProfileControlRealization = Literal["response_law", "surface_allocated", "uncontrolled"]
DirectWrenchControlRealization = Literal["direct_wrench"]
FidelityName = FidelityTier
QUALIFIED_EVIDENCE_STATUSES = QUALIFIED_FIDELITY_STATUSES


class AxisResponseProfile(BaseModel):
    """Bounded first-order response parameters for one attitude axis."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    time_constant_s: float = Field(gt=0.0)
    damping_ratio: float = Field(gt=0.0, le=2.0)
    maximum_rate_rad_s: float = Field(gt=0.0)
    maximum_acceleration_rad_s2: float = Field(gt=0.0)


class Pseudo6DOFProfile(BaseModel):
    """One family-specific pseudo-6DOF realization contract."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    family_id: str = Field(min_length=1)
    parent_3dof_profile_id: str = Field(min_length=1)
    model_kind: Pseudo6DOFModelKind
    control_realization: ProfileControlRealization
    status: ProfileStatus
    evidence_grade: str = Field(min_length=1)
    response: dict[str, AxisResponseProfile] = Field(default_factory=dict)
    phase_response: dict[str, dict[str, AxisResponseProfile]] = Field(default_factory=dict)
    resource_channels: tuple[str, ...] = ()
    required_channels: tuple[str, ...] = ()
    unsupported_claims: tuple[str, ...] = Field(min_length=1)
    area_policy: AreaPolicy = "not_applicable"
    area_source: str | None = None

    @model_validator(mode="after")
    def validate_realization(self) -> Pseudo6DOFProfile:
        axes = set(self.response)
        if self.model_kind == "rigid_body_reuse":
            if axes:
                raise ValueError("rigid_body_reuse profiles must not declare surrogate response axes")
            if self.control_realization != "uncontrolled":
                raise ValueError("rigid_body_reuse profiles must declare uncontrolled realization")
        elif axes != {"roll", "pitch", "yaw"}:
            raise ValueError("pseudo-6DOF response profiles must declare roll, pitch, and yaw")
        for phase, phase_axes in self.phase_response.items():
            if not phase.strip():
                raise ValueError("pseudo-6DOF response schedule phase names must not be empty")
            if set(phase_axes) != {"roll", "pitch", "yaw"}:
                raise ValueError(f"pseudo-6DOF response schedule for {phase!r} must declare roll, pitch, and yaw")
        if self.model_kind != "rigid_body_reuse" and self.control_realization == "uncontrolled":
            raise ValueError("only rigid_body_reuse profiles may declare uncontrolled realization")
        if self.area_policy == "not_applicable" and self.area_source is not None:
            raise ValueError("area_source requires an applicable area_policy")
        if self.area_policy != "not_applicable" and not self.area_source:
            raise ValueError("an applicable area_policy requires area_source")
        return self
        ####

    def response_for_phase(self, phase: str) -> tuple[str, dict[str, AxisResponseProfile]]:
        """Return the active response axes and their provenance label.

        A phase-specific schedule is an explicit reduced-model assumption. If
        no phase entry exists, the profile-wide response is used and labeled
        ``default`` so telemetry never makes an unscheduled law look like a
        physically derived operating-point transition.
        """

        scheduled = self.phase_response.get(phase)
        if scheduled is not None:
            return phase, scheduled
        return "default", self.response
        ####

    ####


class DirectWrenchProfile(BaseModel):
    """A bounded rigid-body generalized-wrench bridge profile."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    family_id: str = Field(min_length=1)
    parent_3dof_profile_id: str = Field(min_length=1)
    control_realization: DirectWrenchControlRealization = "direct_wrench"
    status: ProfileStatus
    evidence_grade: str = Field(min_length=1)
    force_axes: tuple[str, ...] = Field(min_length=1)
    moment_axes: tuple[str, ...] = Field(min_length=1)
    trim_contract: str = Field(min_length=1)
    envelope_contract: str = Field(min_length=1)
    resource_channels: tuple[str, ...] = ()
    required_channels: tuple[str, ...] = ()
    unsupported_claims: tuple[str, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_axes(self) -> DirectWrenchProfile:
        if len(set(self.force_axes)) != len(self.force_axes):
            raise ValueError("direct-wrench force axes must be unique")
        if len(set(self.moment_axes)) != len(self.moment_axes):
            raise ValueError("direct-wrench moment axes must be unique")
        if set(self.force_axes) & set(self.moment_axes):
            raise ValueError("direct-wrench force and moment axes must be disjoint")
        return self
        ####

    ####


class SurfaceAllocationProfile(BaseModel):
    """A declared physical-effector realization, qualified independently."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    family_id: str = Field(min_length=1)
    parent_direct_wrench_profile_id: str = Field(min_length=1)
    control_realization: Literal["surface_allocated"] = "surface_allocated"
    status: Literal["planned", "development", "nominal_case_pass", "promoted", "not_applicable"]
    evidence_grade: str = Field(min_length=1)
    allocator_id: str = Field(min_length=1)
    effector_channels: tuple[str, ...] = ()
    required_channels: tuple[str, ...] = ()
    unsupported_claims: tuple[str, ...] = Field(min_length=1)


class FidelityBinding(BaseModel):
    """Pair a family point-mass identity with its pseudo realization."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    family_id: str = Field(min_length=1)
    point_mass_profile_id: str = Field(min_length=1)
    pseudo_profile_id: str = Field(min_length=1)
    direct_wrench_profile_id: str | None = None
    surface_allocation_profile_id: str | None = None
    automatic_lowering: bool
    lowering_note: str = Field(min_length=1)


@dataclass(frozen=True, slots=True)
class AutomaticLoweringStep:
    """One auditable candidate considered by automatic fidelity lowering."""

    fidelity: LegacyFidelityTier
    profile_id: str | None
    status: SharedLoweringStatus
    prerequisite: str
    reason: str
    required_operations: tuple[str, ...] = ()
    missing_operations: tuple[str, ...] = ()

    def as_dict(self) -> dict[str, object]:
        """Return a stable machine-readable step record."""

        return {
            "fidelity": self.fidelity,
            "profile_id": self.profile_id,
            "status": self.status,
            "prerequisite": self.prerequisite,
            "reason": self.reason,
            "required_operations": list(self.required_operations),
            "missing_operations": list(self.missing_operations),
        }
        ####

    ####


class FidelityEvidenceRecord(BaseModel):
    """One checked artifact that can authorize automatic tier selection."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    profile_id: str = Field(min_length=1)
    fidelity: FidelityName
    status: str = Field(min_length=1)
    artifact: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)


def _validated_evidence_record(record: FidelityEvidenceRecord) -> dict[str, object] | None:
    """Return evidence only when its referenced artifact independently passes."""

    artifact_path = ROOT / record.artifact
    try:
        payload = json.loads(artifact_path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return None
    if not isinstance(payload, Mapping):
        return None
    claim = payload.get("claim")
    claim_status = claim.get("status") if isinstance(claim, Mapping) else None
    payload_status = str(payload.get("status", claim_status or ""))
    status_matches = payload_status == record.status or payload_status.startswith(f"{record.status}_")
    evaluation = payload.get("evaluation")
    if not isinstance(evaluation, Mapping):
        evaluation = payload.get("metrics")
    runtime = payload.get("runtime")
    truth_evaluation = payload.get("truth_evaluation")
    run = payload.get("run")
    envelope_report = payload.get("envelope_report")
    if not status_matches:
        return None
    if isinstance(evaluation, Mapping):
        if evaluation.get("mission_pass") is not True:
            return None
        if isinstance(runtime, Mapping) and runtime.get("hard_gates_passed") is not True:
            return None
    elif isinstance(truth_evaluation, Mapping) and isinstance(run, Mapping):
        if truth_evaluation.get("mission_pass") is not True:
            return None
        completed = run.get("completed")
        if not isinstance(completed, list) or not all(item is True for item in completed):
            return None
        if isinstance(envelope_report, Mapping) and envelope_report.get("pass") is not True:
            return None
        if run.get("exit_code") != 0:
            return None
    else:
        return None
    return {
        "status": record.status,
        "artifact": record.artifact,
        "fidelity": record.fidelity,
        "claim_boundary": record.claim_boundary,
    }
    ####


@lru_cache(maxsize=8)
def load_qualified_fidelity_evidence(path: str | Path | None = None) -> dict[str, Mapping[str, object]]:
    """Load only evidence records whose referenced artifact passes its gates.

    Missing or stale artifacts are discarded, which keeps automatic lowering
    fail-closed while allowing a release manifest to be regenerated without
    changing controller code.
    """

    evidence_path = Path(path) if path is not None else ROOT / "verification/alpha3_fidelity_evidence.yaml"
    try:
        payload = yaml.safe_load(evidence_path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, yaml.YAMLError):
        return {}
    if not isinstance(payload, Mapping) or not isinstance(payload.get("records"), list):
        return {}
    result: dict[str, Mapping[str, object]] = {}
    for raw_record in payload["records"]:
        try:
            record = FidelityEvidenceRecord.model_validate(raw_record)
        except (TypeError, ValueError):
            continue
        checked = _validated_evidence_record(record)
        if checked is not None:
            result[record.profile_id] = checked
    return result
    ####


@dataclass(frozen=True, slots=True)
class AutomaticLoweringReport:
    """Result of resolving the highest evidenced tier without silent fallback."""

    family_id: str
    requested: LegacyFidelityTier
    selected: LegacyFidelityTier | None
    first_blocker: str | None
    steps: tuple[AutomaticLoweringStep, ...]

    @property
    def accepted(self) -> bool:
        """Return whether a qualified tier was selected."""

        return self.selected is not None
        ####

    ####

    def as_dict(self) -> dict[str, object]:
        """Return the complete lowering decision and evidence trail."""

        return {
            "family_id": self.family_id,
            "requested": self.requested,
            "selected": self.selected,
            "accepted": self.accepted,
            "first_blocker": self.first_blocker,
            "steps": [step.as_dict() for step in self.steps],
        }
        ####

    ####


class Pseudo6DOFCatalog(BaseModel):
    """Versioned catalog of family pseudo-6DOF contracts."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_id: str = Field(alias="schema", min_length=1)
    profiles: tuple[Pseudo6DOFProfile, ...] = Field(min_length=1)
    direct_wrench_profiles: tuple[DirectWrenchProfile, ...] = ()
    surface_allocation_profiles: tuple[SurfaceAllocationProfile, ...] = ()
    bindings: tuple[FidelityBinding, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_references(self) -> Pseudo6DOFCatalog:
        profile_ids = {profile.id for profile in self.profiles}
        direct_ids = {profile.id for profile in self.direct_wrench_profiles}
        surface_ids = {profile.id for profile in self.surface_allocation_profiles}
        if (
            len(profile_ids) != len(self.profiles)
            or len(direct_ids) != len(self.direct_wrench_profiles)
            or len(surface_ids) != len(self.surface_allocation_profiles)
        ):
            raise ValueError("fidelity profile IDs must be unique within each profile class")
        if (profile_ids & direct_ids) or (profile_ids & surface_ids) or (direct_ids & surface_ids):
            raise ValueError("fidelity profile IDs must be globally unique")
        direct_by_id = {profile.id: profile for profile in self.direct_wrench_profiles}
        surface_by_id = {profile.id: profile for profile in self.surface_allocation_profiles}
        families = {binding.family_id for binding in self.bindings}
        if len(families) != len(self.bindings):
            raise ValueError("pseudo-6DOF bindings must contain one entry per family")
        for binding in self.bindings:
            if binding.pseudo_profile_id not in profile_ids:
                raise ValueError(f"unknown pseudo profile: {binding.pseudo_profile_id}")
            profile = next(profile for profile in self.profiles if profile.id == binding.pseudo_profile_id)
            if profile.family_id != binding.family_id:
                raise ValueError(f"profile family mismatch for {binding.family_id}")
            if binding.direct_wrench_profile_id is not None:
                direct = next((item for item in self.direct_wrench_profiles if item.id == binding.direct_wrench_profile_id), None)
                if direct is None:
                    raise ValueError(f"unknown direct-wrench profile: {binding.direct_wrench_profile_id}")
                if direct.family_id != binding.family_id:
                    raise ValueError(f"direct-wrench profile family mismatch for {binding.family_id}")
            if binding.surface_allocation_profile_id is not None:
                surface = surface_by_id.get(binding.surface_allocation_profile_id)
                if surface is None:
                    raise ValueError(f"unknown surface-allocation profile: {binding.surface_allocation_profile_id}")
                if surface.family_id != binding.family_id:
                    raise ValueError(f"surface-allocation profile family mismatch for {binding.family_id}")
                if surface.parent_direct_wrench_profile_id not in direct_by_id:
                    raise ValueError(f"surface-allocation profile {surface.id} must reference a direct-wrench profile")
                if direct_by_id[surface.parent_direct_wrench_profile_id].family_id != binding.family_id:
                    raise ValueError(f"surface-allocation parent family mismatch for {binding.family_id}")
        return self
        ####

    def for_family(self, family_id: str) -> tuple[FidelityBinding, Pseudo6DOFProfile]:
        """Resolve one family binding and profile, or fail closed."""

        binding = next((item for item in self.bindings if item.family_id == family_id), None)
        if binding is None:
            raise KeyError(f"no pseudo-6DOF binding for family {family_id}")
        profile = next(item for item in self.profiles if item.id == binding.pseudo_profile_id)
        return binding, profile
        ####

    def for_family_direct_wrench(self, family_id: str) -> tuple[FidelityBinding, DirectWrenchProfile]:
        """Resolve the explicit direct-wrench bridge for one family."""

        binding = next((item for item in self.bindings if item.family_id == family_id), None)
        if binding is None:
            raise KeyError(f"no fidelity binding for family {family_id}")
        if binding.direct_wrench_profile_id is None:
            raise KeyError(f"no direct-wrench profile for family {family_id}")
        profile = next(item for item in self.direct_wrench_profiles if item.id == binding.direct_wrench_profile_id)
        return binding, profile
        ####

    def for_family_surface_allocated(self, family_id: str) -> tuple[FidelityBinding, SurfaceAllocationProfile]:
        """Resolve the declared physical-effector path for one family."""

        binding = next((item for item in self.bindings if item.family_id == family_id), None)
        if binding is None:
            raise KeyError(f"no fidelity binding for family {family_id}")
        if binding.surface_allocation_profile_id is None:
            raise KeyError(f"no surface-allocation profile for family {family_id}")
        profile = next(item for item in self.surface_allocation_profiles if item.id == binding.surface_allocation_profile_id)
        return binding, profile
        ####

    ####


@lru_cache(maxsize=8)
def load_pseudo6dof_catalog(path: str | Path | None = None) -> Pseudo6DOFCatalog:
    """Load and validate the canonical pseudo-6DOF catalog."""

    catalog_path = Path(path) if path is not None else ROOT / "verification/pseudo6dof_profiles.yaml"
    payload = yaml.safe_load(catalog_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{catalog_path} must contain a mapping")
    return Pseudo6DOFCatalog.model_validate(payload)
    ####


def _automatic_lowering_step_from_record(record: Mapping[str, object]) -> AutomaticLoweringStep:
    """Convert one shared-lowerer record while preserving operation diagnostics."""

    required = record.get("required_operations", ())
    missing = record.get("missing_operations", ())
    required_operations = tuple(str(item) for item in required) if isinstance(required, (list, tuple)) else ()
    missing_operations = tuple(str(item) for item in missing) if isinstance(missing, (list, tuple)) else ()
    return AutomaticLoweringStep(
        cast(LegacyFidelityTier, record["fidelity"]),
        record["profile_id"] if isinstance(record["profile_id"], str) else None,
        cast(SharedLoweringStatus, record["status"]),
        str(record["prerequisite"]),
        str(record["reason"]),
        required_operations,
        missing_operations,
    )
    ####


def build_automatic_lowering_report(
    family_id: str,
    requested: LegacyFidelityTier,
    evidence: Mapping[str, Mapping[str, object]] | None = None,
    *,
    catalog: Pseudo6DOFCatalog | None = None,
    required_operations: Mapping[FidelityTier, Sequence[str]] | None = None,
    operation_status: Mapping[FidelityTier, Mapping[str, str]] | None = None,
) -> AutomaticLoweringReport:
    """Resolve a family tier using only explicit qualification evidence.

    The pseudo-profile catalog describes possible reductions, not proof that a
    reduction is safe to select.  ``evidence`` is keyed by profile ID and
    must contain one of :data:`QUALIFIED_EVIDENCE_STATUSES`.  Missing evidence,
    a development-only profile, a disabled binding, or a missing native rigid
    profile all produce a visible blocked step.  No fallback is implicit.  When
    operation requirements and statuses are supplied, a profile is also
    ineligible unless its adapter operations are explicitly available.  This
    keeps profile evidence and executable adapter capability in one lowering
    decision.
    """

    resolved_catalog = catalog or load_pseudo6dof_catalog()
    binding, pseudo_profile = resolved_catalog.for_family(family_id)
    records = evidence if evidence is not None else load_qualified_fidelity_evidence()
    if requested in CANONICAL_FIDELITY_TIERS:
        canonical_requested = requested
        declared_operations = required_operations or {}
        candidates: dict[FidelityTier, LoweringCandidate] = {
            "point_mass_3dof": LoweringCandidate(
                "point_mass_3dof",
                binding.point_mass_profile_id,
                prerequisite="qualified point-mass parent evidence",
                required_operations=tuple(declared_operations.get("point_mass_3dof", ())),
            ),
            "pseudo_6dof": LoweringCandidate(
                "pseudo_6dof",
                pseudo_profile.id,
                (binding.point_mass_profile_id,),
                "qualified pseudo-6DOF profile and parent 3-DOF evidence",
                tuple(declared_operations.get("pseudo_6dof", ())),
            ),
            "rigid_body_6dof_direct_wrench": LoweringCandidate(
                "rigid_body_6dof_direct_wrench",
                binding.direct_wrench_profile_id,
                tuple(profile.parent_3dof_profile_id for profile in resolved_catalog.direct_wrench_profiles if profile.id == binding.direct_wrench_profile_id),
                "qualified direct-wrench bridge and parent 3DOF evidence",
                tuple(declared_operations.get("rigid_body_6dof_direct_wrench", ())),
            ),
            "rigid_body_6dof_surface_allocated": LoweringCandidate(
                "rigid_body_6dof_surface_allocated",
                binding.surface_allocation_profile_id,
                tuple(
                    profile.parent_direct_wrench_profile_id
                    for profile in resolved_catalog.surface_allocation_profiles
                    if profile.id == binding.surface_allocation_profile_id
                ),
                "qualified surface-allocation profile and direct-wrench parent evidence",
                tuple(declared_operations.get("rigid_body_6dof_surface_allocated", ())),
            ),
        }
        decision = select_canonical_lowering(
            candidates,
            canonical_requested,
            records,
            allow_lowering=binding.automatic_lowering,
            operation_status=operation_status,
        )
        return AutomaticLoweringReport(
            family_id,
            requested,
            decision.selected,
            decision.first_blocker,
            tuple(_automatic_lowering_step_from_record(item) for item in decision.considered),
        )
    steps: list[AutomaticLoweringStep] = []
    first_blocker: str | None = None

    def add_step(
        fidelity: LegacyFidelityTier,
        profile_id: str | None,
        status: SharedLoweringStatus,
        prerequisite: str,
        reason: str,
    ) -> bool:
        nonlocal first_blocker
        steps.append(AutomaticLoweringStep(fidelity, profile_id, status, prerequisite, reason))
        if status != "eligible" and first_blocker is None:
            first_blocker = reason
        return status == "eligible"
        ####

    def qualified(profile_id: str) -> tuple[bool, str]:
        record = records.get(profile_id, {})
        evidence_status = str(record.get("status", "missing")) if isinstance(record, Mapping) else "invalid"
        if evidence_status in QUALIFIED_EVIDENCE_STATUSES:
            return True, f"evidence status {evidence_status!r} is qualified"
        return False, f"no qualified evidence for {profile_id!r} (evidence status {evidence_status!r})"
        ####

    def qualified_pair(profile_id: str, declared_status: str | None = None) -> tuple[bool, str]:
        """Require a pseudo profile and its 3-DOF parent to be qualified.

        A response law is not an independently selectable vehicle model.  It
        inherits the translational/resource contract of its parent, so a
        pseudo tier must never be selected when only the attitude reduction
        has a nominal result.
        """

        pseudo_ok, pseudo_reason = qualified(profile_id)
        parent_id = pseudo_profile.parent_3dof_profile_id
        parent_ok, parent_reason = qualified(parent_id)
        if pseudo_ok and parent_ok:
            return True, f"{pseudo_reason}; parent: {parent_reason}"
        if not pseudo_ok:
            return False, pseudo_reason
        return False, f"qualified pseudo profile but parent 3-DOF evidence is missing: {parent_reason}"
        ####

    legacy_requested = requested == "rigid_body_6dof"
    if requested in {"rigid_body_6dof", "rigid_body_6dof_surface_allocated"}:
        surface_id = binding.surface_allocation_profile_id
        if requested == "rigid_body_6dof_surface_allocated" and surface_id is not None:
            surface_profile = next(profile for profile in resolved_catalog.surface_allocation_profiles if profile.id == surface_id)
            surface_ok, surface_reason = qualified(surface_profile.id)
            direct_ok, direct_reason = qualified(surface_profile.parent_direct_wrench_profile_id)
            if surface_ok and not direct_ok:
                surface_ok = False
                surface_reason = f"surface profile is checked but its direct-wrench parent is not: {direct_reason}"
            add_step(
                "rigid_body_6dof_surface_allocated",
                surface_id,
                "eligible" if surface_ok else "blocked",
                "qualified surface-allocation profile and direct-wrench parent evidence",
                surface_reason,
            )
            if surface_ok:
                return AutomaticLoweringReport(family_id, requested, "rigid_body_6dof_surface_allocated", None, tuple(steps))
        elif requested == "rigid_body_6dof_surface_allocated":
            add_step(
                "rigid_body_6dof_surface_allocated",
                None,
                "unavailable",
                "declared surface-allocation profile",
                "no surface-allocation profile was supplied for this family",
            )
        if requested == "rigid_body_6dof_surface_allocated" and not binding.automatic_lowering:
            return AutomaticLoweringReport(family_id, requested, None, first_blocker, tuple(steps))
        if legacy_requested:
            rigid_id = f"{family_id}.rigid_body_6dof"
            rigid_record = records.get(rigid_id, {})
            rigid_status = str(rigid_record.get("status", "missing")) if isinstance(rigid_record, Mapping) else "invalid"
            rigid_ok = rigid_status in QUALIFIED_EVIDENCE_STATUSES
            if add_step(
                "rigid_body_6dof",
                rigid_id,
                "eligible" if rigid_ok else "unavailable",
                "legacy native rigid-body qualification artifact",
                f"evidence status {rigid_status!r} for {rigid_id!r}"
                if rigid_ok
                else "no native rigid-body qualification profile was supplied (legacy compatibility path)",
            ):
                return AutomaticLoweringReport(family_id, requested, "rigid_body_6dof", first_blocker, tuple(steps))

    direct_id = binding.direct_wrench_profile_id
    if direct_id is not None and requested in {
        "rigid_body_6dof",
        "rigid_body_6dof_direct_wrench",
        "rigid_body_6dof_surface_allocated",
    }:
        direct_profile = next(profile for profile in resolved_catalog.direct_wrench_profiles if profile.id == direct_id)
        # Unlike a response profile, a direct bridge is executable only when
        # its referenced artifact was checked.  Catalog status alone must not
        # authorize an injected wrench.
        direct_ok, direct_reason = qualified(direct_profile.id)
        parent_ok, parent_reason = qualified(direct_profile.parent_3dof_profile_id)
        if direct_ok and parent_ok:
            direct_reason = f"{direct_reason}; parent: {parent_reason}"
        elif direct_ok:
            direct_ok = False
            direct_reason = f"qualified direct-wrench profile but parent 3-DOF evidence is missing: {parent_reason}"
        add_step(
            "rigid_body_6dof_direct_wrench",
            direct_id,
            "eligible" if direct_ok else "blocked",
            "qualified direct-wrench bridge and parent 3DOF evidence",
            direct_reason,
        )
        if direct_ok:
            # A missing native surface-allocated tier is informative, but it
            # is not a blocker once the explicitly requested bridge tier has
            # passed.
            return AutomaticLoweringReport(family_id, requested, "rigid_body_6dof_direct_wrench", None, tuple(steps))
        if requested == "rigid_body_6dof_direct_wrench" or not binding.automatic_lowering:
            return AutomaticLoweringReport(family_id, requested, None, first_blocker, tuple(steps))
    elif requested == "rigid_body_6dof" and not binding.automatic_lowering:
        return AutomaticLoweringReport(family_id, requested, None, first_blocker, tuple(steps))

    if requested == "rigid_body_6dof_direct_wrench":
        add_step(
            "rigid_body_6dof_direct_wrench",
            None,
            "unavailable",
            "explicit direct-wrench profile binding",
            "no direct-wrench profile was supplied for this family",
        )
        return AutomaticLoweringReport(family_id, requested, None, first_blocker, tuple(steps))

    if requested == "point_mass_3dof":
        point_ok, point_reason = qualified(binding.point_mass_profile_id)
        add_step(
            "point_mass_3dof",
            binding.point_mass_profile_id,
            "eligible" if point_ok else "blocked",
            "qualified point-mass parent evidence",
            point_reason,
        )
        return AutomaticLoweringReport(
            family_id,
            requested,
            "point_mass_3dof" if point_ok else None,
            first_blocker,
            tuple(steps),
        )

    pseudo_ok, pseudo_reason = qualified_pair(pseudo_profile.id, pseudo_profile.status)
    if add_step(
        "pseudo_6dof",
        pseudo_profile.id,
        "eligible" if pseudo_ok else "blocked",
        "qualified pseudo-6DOF profile and parent 3DOF evidence",
        pseudo_reason,
    ):
        return AutomaticLoweringReport(family_id, requested, "pseudo_6dof", first_blocker, tuple(steps))
    if not binding.automatic_lowering:
        return AutomaticLoweringReport(family_id, requested, None, first_blocker, tuple(steps))

    point_ok, point_reason = qualified(binding.point_mass_profile_id)
    add_step(
        "point_mass_3dof",
        binding.point_mass_profile_id,
        "eligible" if point_ok else "blocked",
        "qualified point-mass parent evidence",
        point_reason,
    )
    selected: FidelityName | None = "point_mass_3dof" if point_ok else None
    return AutomaticLoweringReport(family_id, requested, selected, first_blocker, tuple(steps))
    ####


__all__ = [
    "AutomaticLoweringReport",
    "AutomaticLoweringStep",
    "AxisResponseProfile",
    "DirectWrenchControlRealization",
    "DirectWrenchProfile",
    "FidelityBinding",
    "FidelityEvidenceRecord",
    "ProfileStatus",
    "Pseudo6DOFCatalog",
    "Pseudo6DOFModelKind",
    "Pseudo6DOFProfile",
    "SurfaceAllocationProfile",
    "ProfileControlRealization",
    "QUALIFIED_EVIDENCE_STATUSES",
    "build_automatic_lowering_report",
    "load_pseudo6dof_catalog",
    "load_qualified_fidelity_evidence",
]
