"""Regression tests for the F-16 ladder's canonical showcase boundary."""

from __future__ import annotations

import hashlib

import pytest

from taoryx.showcase import ArtifactFile, EvidenceBoardSpec, build_showcase_run_artifact, validate_showcase_run_artifact_boundary
from tools.build_f16_racetrack_fidelity_packet import MODE_FIDELITIES, _showcase_realization


def _evidence() -> dict[str, object]:
    return {
        "claim_boundary": "development witness only",
        "runtime": {"numerical_valid": True, "envelope_violations": []},
        "evaluation": {"mission_pass": True},
    }


@pytest.mark.parametrize(
    ("runtime_mode", "expected_control", "expected_effectors"),
    [
        ("point_mass_3dof", "force_model", ()),
        ("pseudo_6dof_kinematic_bridge", "response_law", ()),
        ("direct_wrench", "direct_wrench", ()),
        (
            "surface_allocated",
            "surface_allocated",
            ("elevator_deg", "aileron_deg", "rudder_deg", "throttle_fraction"),
        ),
    ],
)
def test_f16_modes_emit_unambiguous_canonical_realizations(
    runtime_mode: str,
    expected_control: str,
    expected_effectors: tuple[str, ...],
) -> None:
    realization = _showcase_realization(runtime_mode, _evidence())

    assert realization.fidelity == MODE_FIDELITIES[runtime_mode]
    assert realization.control_realization == expected_control
    assert realization.physical_effectors == expected_effectors

    digest = hashlib.sha256(b"f16-scenario").hexdigest()
    artifact = build_showcase_run_artifact(
        realization=realization,
        run_id=f"f16-{runtime_mode}",
        showcase_id="org.taoryx.showcase.reference_f16_s119.racetrack",
        vehicle_binding_id="f16-s119",
        scenario_contract_sha256=digest,
        outcome="completed",
        files=(ArtifactFile(path="manifest.json", sha256=digest, media_type="application/json"),),
        board=EvidenceBoardSpec(profile="family-evidence-board-v1", modules=("mission_geometry",)),
    )

    assert validate_showcase_run_artifact_boundary(artifact) == artifact
    ####
