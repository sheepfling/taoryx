from pathlib import Path

import yaml

from taoryx.hl20_reachability import (
    HL20_FIDELITIES,
    hl20_release_vehicle,
    run_hl20_fidelity_ladder,
    run_hl20_low_fidelity_release,
    write_hl20_low_fidelity_bundle,
)

ROOT = Path(__file__).resolve().parents[2]


def test_x15_and_hl20_share_a_comparison_family_without_shared_claims() -> None:
    family = yaml.safe_load(
        (ROOT / "verification/hypersonic_lifting_body_family.yaml").read_text(encoding="utf-8")
    )
    assert family["family_group_id"] == "hypersonic_lifting_body_research"
    assert {member["vehicle_id"] for member in family["members"]} == {"x15", "reference_hl20_mod_k"}
    assert (ROOT / family["members"][0]["source_fixture"]).is_dir()
    assert (ROOT / family["members"][1]["source_family_manifest"]).is_file()
    assert family["claim_boundary"]["shared_dynamics"] is False
    assert family["comparison_contract"]["source_evidence"] is False
    ladder = family["rocket_composition"]["fidelity_ladder"]
    assert [item["runtime_fidelity"] for item in ladder] == ["point_mass_3dof", "pseudo_6dof", "rigid_body_6dof"]
    ####


def test_hl20_ca_hi_contract_has_no_controller_or_arrival_claim() -> None:
    contract = yaml.safe_load(
        (ROOT / "examples/showcases/hl20_california_to_hawaii/showcase.yaml").read_text(encoding="utf-8")
    )
    assert contract["status"] == "low_fidelity_executable"
    assert contract["claim_boundary"]["controller_claims"] is False
    assert contract["claim_boundary"]["source_exact_route"] is False
    assert "landing" in " ".join(contract["nonclaims"])
    assert contract["launch_composition"]["release_event"]["lineage_required"] is True
    assert [item["id"] for item in contract["launch_composition"]["fidelity_ladder"]] == [
        "boost_release_3dof",
        "boost_release_pseudo6dof",
        "boost_release_rigid6dof",
    ]
    ####


def test_hl20_low_fidelity_release_runs_with_event_lineage() -> None:
    result = run_hl20_low_fidelity_release()
    assert result.fidelity.value == "point_mass_3dof"
    assert {state.phase for sample in result.samples for state in sample.trajectory.states} >= {"boost", "coast", "glide"}
    assert all(sample.trajectory.deployment_events for sample in result.samples)
    assert all(event["event_id"] == "booster-release" for sample in result.samples for event in sample.trajectory.deployment_events)
    assert all(sample.trajectory.states[-1].mass_kg == hl20_release_vehicle().release_mass_kg for sample in result.samples)
    ####


def test_hl20_low_fidelity_bundle_is_self_describing(tmp_path: Path) -> None:
    bundle = write_hl20_low_fidelity_bundle(tmp_path, horizon_s=30.0)
    assert bundle.artifact_path.is_file()
    assert bundle.manifest_path.is_file()
    payload = yaml.safe_load(bundle.manifest_path.read_text(encoding="utf-8"))
    assert payload["fidelity"] == "point_mass_3dof"
    assert payload["provenance"]["source_exact_trajectory"] is False
    ####


def test_hl20_fidelity_ladder_runs_all_three_tiers() -> None:
    envelopes = run_hl20_fidelity_ladder(horizon_s=30.0, step_size_s=0.5)
    assert tuple(envelope.fidelity for envelope in envelopes) == HL20_FIDELITIES
    assert all(envelope.samples for envelope in envelopes)
    assert all(sample.trajectory.deployment_events for envelope in envelopes for sample in envelope.samples)
    assert all(
        sample.trajectory.states[0].__class__.__name__ == ("PointMass3DofState" if envelope.fidelity.value == "point_mass_3dof" else "Pseudo6DofState" if envelope.fidelity.value == "pseudo_6dof" else "RigidBody6DofReachabilityState")
        for envelope in envelopes
        for sample in envelope.samples
    )
    rigid = envelopes[2]
    assert rigid.samples[0].trajectory.states[0].__class__.__name__ == "RigidBody6DofReachabilityState"
    assert all("attitude_quaternion" in state for sample in rigid.samples for state in sample.trajectory.telemetry)
    ####
