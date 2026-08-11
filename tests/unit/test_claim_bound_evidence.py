"""Focused tests for typed, composition-bound release sidecars."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from taoryx.claim_bound_evidence import ClaimBoundEvidenceArtifact, bind_release_evidence


def _composition() -> SimpleNamespace:
    """Return the minimum immutable identity expected by the binding helper."""

    return SimpleNamespace(
        id="fixture-composition",
        identity_sha256="a" * 64,
        vehicle_id="fixture-vehicle",
        family_id="fixture-family",
        mission="fixture-mission",
        fidelity="point_mass_3dof",
        control_realization="fixture-controls",
    )
    ####


def _payload(*, status: str = "pass", passed: bool | None = True) -> dict[str, object]:
    """Return a minimal provider-owned evidence report."""

    payload: dict[str, object] = {
        "schema": "taoryx.fixture-release-evidence/v1alpha1",
        "status": status,
        "claim_boundary": "Fixture evidence only; no qualification claim.",
    }
    if passed is not None:
        payload["pass"] = passed
    return payload
    ####


def test_release_evidence_requires_a_canonical_status_and_matching_boolean() -> None:
    """Typos and ambiguous outcomes stop before a sidecar reaches release assembly."""

    with pytest.raises(ValueError, match="payload status must be"):
        bind_release_evidence(_payload(status="accepted"), kind="robustness", composition=_composition())
    with pytest.raises(ValueError, match="requires passed=True"):
        bind_release_evidence(_payload(passed=None), kind="robustness", composition=_composition())
    with pytest.raises(ValueError, match="requires passed=False"):
        bind_release_evidence(_payload(status="fail", passed=True), kind="robustness", composition=_composition())
    ####


def test_release_evidence_allows_only_explicitly_indeterminate_outcomes() -> None:
    """Partial and not-applicable evidence cannot accidentally carry a Boolean verdict."""

    bound = bind_release_evidence(
        _payload(status="partial", passed=None),
        kind="fidelity_mapping",
        composition=_composition(),
    )

    evidence = ClaimBoundEvidenceArtifact.from_payload(bound)
    assert evidence.status == "partial"
    assert evidence.release_evidence_outcome.passed is None

    payload = dict(bound)
    payload["release_evidence_outcome"] = {"status": "partial", "passed": True}
    with pytest.raises(ValueError, match="requires passed=None"):
        ClaimBoundEvidenceArtifact.from_payload(payload)
    ####
