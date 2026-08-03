"""Construct replayable showcase artifacts from resolved realizations."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import asdict, dataclass
from typing import TYPE_CHECKING, Literal, Mapping, cast

from taoryx.fidelity_contracts import ControlRealization, FidelityTier, control_realization_for

from .contracts import (
    ArtifactFile,
    EvidenceBoardSpec,
    FidelityShowcaseRealization,
    ObjectLineage,
    ShowcaseArchetype,
    ShowcaseOutcome,
    ShowcaseRunArtifact,
    VehicleInterfaceEvidence,
)

if TYPE_CHECKING:
    from taoryx.vehicle_composition import CompiledVehicleComposition
    from taoryx.vehicle_composition_registry import ResolvedVehicleCompositionCatalog
    from taoryx.vehicle_interface import VehicleInterfaceContract


@dataclass(frozen=True, slots=True)
class ShowcaseArtifactBoundaryFinding:
    """One machine-readable violation of the common showcase boundary."""

    code: str
    message: str
    severity: Literal["error", "warning"] = "error"

    def as_dict(self) -> dict[str, str]:
        """Return a JSON-ready finding."""

        return asdict(self)
        ####
    ####


def inspect_showcase_run_artifact(
    artifact: ShowcaseRunArtifact,
) -> tuple[ShowcaseArtifactBoundaryFinding, ...]:
    """Inspect a parsed artifact without changing its claim metadata.

    The Pydantic contract protects construction-time invariants. This second
    inspection is intentionally stricter for release packets: a legacy
    artifact may still be read for compatibility, but it cannot pass the
    horizontal artifact boundary unless it retains the resolved realization.
    """

    findings: list[ShowcaseArtifactBoundaryFinding] = []
    realization = artifact.realization
    if realization is None:
        findings.append(
            ShowcaseArtifactBoundaryFinding(
                "realization_missing",
                "run artifact does not retain the resolved fidelity/control realization",
            )
        )
        return tuple(findings)

    expected_control = control_realization_for(cast(FidelityTier, realization.fidelity))
    if realization.control_realization not in {expected_control, "uncontrolled"}:
        findings.append(
            ShowcaseArtifactBoundaryFinding(
                "control_realization_mismatch",
                f"tier {realization.fidelity!r} expects {expected_control!r}, "
                f"got {realization.control_realization!r}",
            )
        )
    if realization.control_realization != "surface_allocated" and realization.physical_effectors:
        findings.append(
            ShowcaseArtifactBoundaryFinding(
                "hidden_physical_effectors",
                "non-surface realization declares physical effectors: "
                + ", ".join(realization.physical_effectors),
            )
        )
    if realization.fidelity != artifact.fidelity:
        findings.append(
            ShowcaseArtifactBoundaryFinding(
                "fidelity_drift",
                "artifact fidelity differs from its retained realization",
            )
        )
    if realization.control_realization != artifact.control_realization:
        findings.append(
            ShowcaseArtifactBoundaryFinding(
                "control_path_drift",
                "artifact control realization differs from its retained realization",
            )
        )
    if realization.claim != artifact.claim:
        findings.append(
            ShowcaseArtifactBoundaryFinding(
                "claim_drift",
                "artifact claim differs from its retained realization",
            )
        )
    if realization.nonclaims != artifact.nonclaims:
        findings.append(
            ShowcaseArtifactBoundaryFinding(
                "nonclaim_drift",
                "artifact nonclaims differ from its retained realization",
            )
        )
    return tuple(findings)
    ####


def validate_showcase_run_artifact_boundary(
    payload: Mapping[str, object] | ShowcaseRunArtifact,
) -> ShowcaseRunArtifact:
    """Parse and strictly validate one horizontal showcase artifact.

    This is the release-packet check. It deliberately rejects legacy artifact
    records that can still be loaded by :class:`ShowcaseRunArtifact` for
    backwards-compatible inspection.
    """

    artifact = payload if isinstance(payload, ShowcaseRunArtifact) else ShowcaseRunArtifact.model_validate(payload)
    findings = inspect_showcase_run_artifact(artifact)
    if findings:
        detail = "; ".join(f"{item.code}: {item.message}" for item in findings)
        raise ValueError(f"showcase artifact boundary failed: {detail}")
    return artifact
    ####


def build_showcase_run_artifact(
    *,
    realization: FidelityShowcaseRealization,
    run_id: str,
    showcase_id: str,
    vehicle_binding_id: str,
    scenario_contract_sha256: str,
    outcome: ShowcaseOutcome,
    files: Sequence[ArtifactFile],
    board: EvidenceBoardSpec,
    archetypes: Sequence[ShowcaseArchetype] = (),
    object_lineage: ObjectLineage | None = None,
    vehicle_interface_contract: VehicleInterfaceContract | None = None,
    selected_observation_profile: str | None = None,
) -> ShowcaseRunArtifact:
    """Build one run artifact from the canonical fidelity realization.

    Fidelity, control realization, claim, and nonclaims are intentionally
    taken from ``realization`` rather than supplied a second time. This keeps
    renderers from drifting into a different claim boundary than the adapter
    or resolved family manifest. Legacy callers may still construct an
    artifact without retaining the realization object.
    """

    return ShowcaseRunArtifact(
        run_id=run_id,
        showcase_id=showcase_id,
        vehicle_binding_id=vehicle_binding_id,
        fidelity=realization.fidelity,
        control_realization=realization.control_realization,
        realization=realization,
        scenario_contract_sha256=scenario_contract_sha256,
        outcome=outcome,
        claim=realization.claim,
        nonclaims=realization.nonclaims,
        files=tuple(files),
        board=board,
        archetypes=tuple(archetypes),
        object_lineage=object_lineage,
        vehicle_interface=(
            None
            if vehicle_interface_contract is None
            else interface_evidence_for_showcase(
                vehicle_interface_contract,
                selected_observation_profile=selected_observation_profile,
            )
        ),
    )
    ####


def build_showcase_run_artifact_for_composition(
    *,
    composition: CompiledVehicleComposition,
    realization: FidelityShowcaseRealization,
    run_id: str,
    showcase_id: str,
    vehicle_binding_id: str,
    scenario_contract_sha256: str,
    outcome: ShowcaseOutcome,
    files: Sequence[ArtifactFile],
    board: EvidenceBoardSpec,
    archetypes: Sequence[ShowcaseArchetype] = (),
    object_lineage: ObjectLineage | None = None,
    catalog: ResolvedVehicleCompositionCatalog | None = None,
) -> ShowcaseRunArtifact:
    """Build board evidence from the exact compiled composition contract.

    This is the composition-facing entry point.  It resolves the declared
    observation selection before constructing the generic run artifact, so a
    board cannot retain a family/fidelity truth-debug contract when the run
    actually used a declared sensor profile.
    """

    from taoryx.vehicle_composition import resolve_vehicle_composition_interface_contract

    artifact_paths = {item.path for item in files}
    if "status_trace.json" not in artifact_paths:
        raise ValueError(
            "a composition-backed showcase requires status_trace.json in its artifact files"
        )
    if composition.observation.declared_sensor is not None and "sensor_observations.json" not in artifact_paths:
        raise ValueError(
            "a composition-backed showcase with a declared sensor profile requires "
            "sensor_observations.json in its artifact files"
        )
    contract = resolve_vehicle_composition_interface_contract(composition, catalog=catalog)
    return build_showcase_run_artifact(
        realization=realization,
        run_id=run_id,
        showcase_id=showcase_id,
        vehicle_binding_id=vehicle_binding_id,
        scenario_contract_sha256=scenario_contract_sha256,
        outcome=outcome,
        files=files,
        board=board,
        archetypes=archetypes,
        object_lineage=object_lineage,
        vehicle_interface_contract=contract,
        selected_observation_profile=composition.observation.profile_id,
    )
    ####


def interface_evidence_for_showcase(
    contract: VehicleInterfaceContract,
    *,
    selected_observation_profile: str | None = None,
) -> VehicleInterfaceEvidence:
    """Project one resolved execution interface into immutable board evidence."""

    available_observations = tuple(item.id for item in contract.observation_profiles if item.availability == "available")
    if selected_observation_profile is not None and selected_observation_profile not in available_observations:
        raise ValueError(
            f"selected showcase observation profile {selected_observation_profile!r} is unavailable from "
            f"interface {contract.id!r}"
        )

    return VehicleInterfaceEvidence(
        interface_id=contract.id,
        fingerprint_sha256=contract.fingerprint,
        fidelity=contract.fidelity,
        control_realization=cast(ControlRealization, contract.control_realization),
        evidence_status=contract.evidence_status,
        available_authority_profiles=tuple(item.id for item in contract.authority_profiles if item.availability == "available"),
        available_observation_profiles=available_observations,
        selected_observation_profile=selected_observation_profile,
        claim_boundary=contract.claim_boundary,
    )
    ####


__all__ = [
    "ShowcaseArtifactBoundaryFinding",
    "build_showcase_run_artifact",
    "build_showcase_run_artifact_for_composition",
    "interface_evidence_for_showcase",
    "inspect_showcase_run_artifact",
    "validate_showcase_run_artifact_boundary",
]
