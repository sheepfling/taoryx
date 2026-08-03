from __future__ import annotations

import hashlib
from pathlib import Path

import pytest
import yaml

from taoryx.showcase import (
    ArtifactFile,
    EvidenceBoardSpec,
    FamilyShowcaseTemplate,
    FidelityShowcaseRealization,
    MissionSegmentSpec,
    ObjectLineage,
    ObjectLineageEvent,
    ObjectLineageNode,
    ShowcaseArchetypeCatalog,
    ShowcaseArchetypeSpec,
    ShowcaseRecipe,
    ShowcaseRunArtifact,
    StartContract,
    TerminalContract,
    VehicleShowcaseBinding,
    build_showcase_run_artifact,
    build_showcase_run_artifact_for_composition,
    validate_showcase_run_artifact_boundary,
)
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request
from taoryx.vehicle_interface import resolve_vehicle_interface_contract

ROOT = Path(__file__).resolve().parents[2]


def _template() -> FamilyShowcaseTemplate:
    return FamilyShowcaseTemplate(
        id="org.taoryx.showcase.synthetic.route",
        version="0.1.0",
        family="powered_fixed_wing",
        display_name="Synthetic route flagship",
        claim="The declared synthetic vehicle completes its route contract.",
        start_contract=StartContract(type="trimmed_airborne"),
        segments=(
            MissionSegmentSpec(id="outbound", objective="capture-outbound", success_event="outbound-captured"),
            MissionSegmentSpec(id="return", objective="capture-return", success_event="return-captured"),
        ),
        required_events=("outbound-captured", "return-captured"),
        terminal_contract=TerminalContract(success_event="return-captured", corridor={"position_m": 2.0}),
        required_semantic_controls=("command.bank", "command.throttle"),
        plot_modules=("trajectory_3d", "attitude_and_rates", "terminal_corridor"),
    )


def test_showcase_template_has_family_local_segments_and_explicit_terminal() -> None:
    template = _template()

    assert [segment.id for segment in template.segments] == ["outbound", "return"]
    assert template.terminal_contract.timeout_is_success is False
    assert "command.bank" in template.required_semantic_controls


def test_showcase_template_rejects_duplicate_segment_ids() -> None:
    with pytest.raises(ValueError, match="segment IDs must be unique"):
        FamilyShowcaseTemplate(
            **_template().model_dump(mode="python") | {
                "segments": (_template().segments[0], _template().segments[0])
            }
        )


def test_showcase_run_artifact_requires_hashed_manifest_and_declares_fidelity() -> None:
    digest = hashlib.sha256(b"manifest").hexdigest()
    artifact = ShowcaseRunArtifact(
        run_id="run-001",
        showcase_id="org.taoryx.showcase.synthetic.route",
        vehicle_binding_id="synthetic-v1",
        fidelity="pseudo_6dof",
        scenario_contract_sha256=digest,
        outcome="completed",
        claim="Declared synthetic route completed.",
        files=(ArtifactFile(path="manifest.json", sha256=digest, media_type="application/json"),),
        board=EvidenceBoardSpec(profile="evidence-board-v1", modules=("trajectory_3d", "terminal_corridor")),
    )

    assert artifact.board.event_marker_policy == "family_local"
    assert artifact.files[0].path == "manifest.json"


def test_showcase_artifact_factory_retains_resolved_realization() -> None:
    digest = hashlib.sha256(b"manifest").hexdigest()
    realization = FidelityShowcaseRealization(
        fidelity="rigid_body_6dof_direct_wrench",
        control_realization="direct_wrench",
        realization_id="synthetic-direct-v1",
        state_schema=("position_m",),
        claim="Direct-wrench evidence only.",
        nonclaims=("physical effectors",),
        evidence_grade="derived",
    )

    artifact = build_showcase_run_artifact(
        realization=realization,
        run_id="run-factory-001",
        showcase_id="synthetic-showcase",
        vehicle_binding_id="synthetic-v1",
        scenario_contract_sha256=digest,
        outcome="completed",
        files=(ArtifactFile(path="manifest.json", sha256=digest, media_type="application/json"),),
        board=EvidenceBoardSpec(profile="evidence-board-v1", modules=("mission_geometry",)),
    )

    assert artifact.realization == realization
    assert artifact.fidelity == "rigid_body_6dof_direct_wrench"
    assert artifact.control_realization == "direct_wrench"
    assert artifact.nonclaims == ("physical effectors",)
    assert validate_showcase_run_artifact_boundary(artifact) == artifact


def test_showcase_artifact_can_retain_the_exact_vehicle_interface_contract() -> None:
    digest = hashlib.sha256(b"manifest").hexdigest()
    interface = resolve_vehicle_interface_contract("skywalker_x8", "pseudo_6dof")
    realization = FidelityShowcaseRealization(
        fidelity="pseudo_6dof",
        control_realization="response_law",
        realization_id="x8-route-lag-v1",
        state_schema=("position_m", "velocity_m_s", "attitude_response"),
        claim="Pseudo-6DOF route-lag response evidence only.",
        nonclaims=("physical elevon allocation",),
        evidence_grade="derived",
    )

    artifact = build_showcase_run_artifact(
        realization=realization,
        run_id="x8-interface-board",
        showcase_id="org.taoryx.showcase.x8",
        vehicle_binding_id="skywalker-x8-v1",
        scenario_contract_sha256=digest,
        outcome="completed",
        files=(ArtifactFile(path="manifest.json", sha256=digest, media_type="application/json"),),
        board=EvidenceBoardSpec(profile="evidence-board-v1", modules=("trajectory_3d",)),
        vehicle_interface_contract=interface,
    )

    assert artifact.vehicle_interface is not None
    assert artifact.vehicle_interface.interface_id == interface.id
    assert artifact.vehicle_interface.fingerprint_sha256 == interface.fingerprint
    assert artifact.vehicle_interface.available_authority_profiles == ("native_control_bridge",)
    assert validate_showcase_run_artifact_boundary(artifact) == artifact


def test_composition_showcase_artifact_retains_the_selected_sensor_profile() -> None:
    digest = hashlib.sha256(b"sensor-manifest").hexdigest()
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(
            ROOT / "examples/vehicle_composition/x8_racetrack_sensor_episode_3dof_compose.yaml"
        )
    )
    realization = FidelityShowcaseRealization(
        fidelity="point_mass_3dof",
        control_realization="force_model",
        realization_id="x8-point-mass-sensor-v1",
        state_schema=("position_velocity_mass",),
        claim="Point-mass route evidence with declared delayed observations.",
        nonclaims=("physical elevon allocation", "sensor noise or estimator validation"),
        evidence_grade="derived",
    )

    common = {
        "composition": composition,
        "realization": realization,
        "run_id": "x8-sensor-board",
        "showcase_id": "org.taoryx.showcase.x8.sensor",
        "vehicle_binding_id": "skywalker-x8-point-mass-v1",
        "scenario_contract_sha256": digest,
        "outcome": "completed",
        "board": EvidenceBoardSpec(profile="evidence-board-v1", modules=("trajectory_3d",)),
    }
    with pytest.raises(ValueError, match="status_trace.json"):
        build_showcase_run_artifact_for_composition(
            **common,
            files=(ArtifactFile(path="manifest.json", sha256=digest, media_type="application/json"),),
        )

    with pytest.raises(ValueError, match="sensor_observations.json"):
        build_showcase_run_artifact_for_composition(
            **common,
            files=(
                ArtifactFile(path="manifest.json", sha256=digest, media_type="application/json"),
                ArtifactFile(path="status_trace.json", sha256=digest, media_type="application/json"),
            ),
        )

    artifact = build_showcase_run_artifact_for_composition(
        **common,
        files=(
            ArtifactFile(path="manifest.json", sha256=digest, media_type="application/json"),
            ArtifactFile(path="status_trace.json", sha256=digest, media_type="application/json"),
            ArtifactFile(path="sensor_observations.json", sha256=digest, media_type="application/json"),
        ),
    )

    assert artifact.vehicle_interface is not None
    assert artifact.vehicle_interface.selected_observation_profile == "declared_sensor"
    assert "declared_sensor" in artifact.vehicle_interface.available_observation_profiles
    assert validate_showcase_run_artifact_boundary(artifact) == artifact


def test_showcase_boundary_rejects_legacy_artifact_without_realization() -> None:
    digest = hashlib.sha256(b"manifest").hexdigest()
    artifact = ShowcaseRunArtifact(
        run_id="run-legacy-boundary",
        showcase_id="synthetic-showcase",
        vehicle_binding_id="synthetic-v1",
        fidelity="pseudo_6dof",
        scenario_contract_sha256=digest,
        outcome="completed",
        claim="legacy artifact",
        files=(ArtifactFile(path="manifest.json", sha256=digest, media_type="application/json"),),
        board=EvidenceBoardSpec(profile="evidence-board-v1", modules=("mission_geometry",)),
    )

    with pytest.raises(ValueError, match="realization_missing"):
        validate_showcase_run_artifact_boundary(artifact)


def test_showcase_binding_and_realization_preserve_claim_boundary() -> None:
    binding = VehicleShowcaseBinding(
        id="synthetic-v1",
        family="powered_fixed_wing",
        version="1.0.0",
        vehicle_package="synthetic-research-surrogate",
        start_state_factory="synthetic.trim",
        evidence_grade="synthetic",
        unsupported_behaviors=("stall",),
    )
    realization = FidelityShowcaseRealization(
        fidelity="rigid_body_6dof",
        control_realization="surface_allocated",
        realization_id="synthetic-rigid-v1",
        state_schema=("position_ecef_m", "quaternion_xyzw", "body_rates_rad_s"),
        semantic_command_mapping={"command.bank": "aileron_deg"},
        physical_effectors=("aileron_deg",),
        available_physics=("forces", "moments", "actuator_rate_limits"),
        claim="Rigid-body response inside the synthetic declared envelope.",
        nonclaims=("flight qualification",),
        evidence_grade="synthetic",
    )

    assert binding.evidence_grade == "synthetic"
    assert "flight qualification" in realization.nonclaims
    assert isinstance(realization.control_realization, str)
    assert realization.fidelity == "rigid_body_6dof_surface_allocated"


def test_showcase_control_realization_rejects_hidden_effector_claims() -> None:
    with pytest.raises(ValueError, match="direct_wrench showcase realizations"):
        FidelityShowcaseRealization(
            fidelity="rigid_body_6dof",
            control_realization="direct_wrench",
            realization_id="direct-wrench-v1",
            state_schema=("position_m",),
            physical_effectors=("elevator",),
            claim="direct wrench screen",
            evidence_grade="derived",
        )

    with pytest.raises(ValueError, match="surface_allocated showcase realizations"):
        FidelityShowcaseRealization(
            fidelity="rigid_body_6dof",
            control_realization="surface_allocated",
            realization_id="surface-v1",
            state_schema=("position_m",),
            claim="surface allocation",
            evidence_grade="derived",
        )


def test_showcase_rejects_ambiguous_legacy_rigid_body_fidelity() -> None:
    with pytest.raises(ValueError, match="legacy 'rigid_body_6dof' is ambiguous"):
        FidelityShowcaseRealization(
            fidelity="rigid_body_6dof",
            realization_id="ambiguous-rigid-v1",
            state_schema=("position_m",),
            claim="ambiguous rigid-body record",
            evidence_grade="derived",
        )

    digest = hashlib.sha256(b"manifest").hexdigest()
    with pytest.raises(ValueError, match="legacy 'rigid_body_6dof' is ambiguous"):
        ShowcaseRunArtifact(
            run_id="run-ambiguous-rigid",
            showcase_id="org.taoryx.showcase.synthetic.route",
            vehicle_binding_id="synthetic-v1",
            fidelity="rigid_body_6dof",
            scenario_contract_sha256=digest,
            outcome="completed",
            claim="ambiguous rigid-body record",
            files=(ArtifactFile(path="manifest.json", sha256=digest, media_type="application/json"),),
            board=EvidenceBoardSpec(profile="evidence-board-v1", modules=("trajectory_3d",)),
        )


def test_showcase_artifact_normalizes_legacy_surface_fidelity() -> None:
    digest = hashlib.sha256(b"manifest").hexdigest()
    artifact = ShowcaseRunArtifact(
        run_id="run-surface-legacy",
        showcase_id="org.taoryx.showcase.synthetic.route",
        vehicle_binding_id="synthetic-v1",
        fidelity="rigid_body_6dof",
        control_realization="surface_allocated",
        scenario_contract_sha256=digest,
        outcome="completed",
        claim="surface-allocated rigid-body record",
        files=(ArtifactFile(path="manifest.json", sha256=digest, media_type="application/json"),),
        board=EvidenceBoardSpec(profile="evidence-board-v1", modules=("trajectory_3d",)),
    )

    assert artifact.fidelity == "rigid_body_6dof_surface_allocated"


def test_showcase_archetype_catalog_requires_common_proof_products() -> None:
    archetype_ids = (
        "mission_geometry",
        "mission_timeline",
        "dynamics_and_resources",
        "envelope_and_qualification",
        "object_lineage",
    )
    catalog = ShowcaseArchetypeCatalog(
        id="taoryx-showcase-archetypes-v1",
        claim_boundary="synthetic showcase contract only",
        archetypes=tuple(
            ShowcaseArchetypeSpec(
                id=archetype_id,
                display_name=archetype_id.replace("_", " ").title(),
                purpose=f"Proof surface for {archetype_id}.",
                required_modules=(archetype_id,),
            )
            for archetype_id in archetype_ids
        ),
        recipes=(
            ShowcaseRecipe(
                id="b747-v1",
                family="heavy_transport",
                display_name="B747 transport energy management",
                priority=1,
                archetypes=archetype_ids[:4],
                emphasis=("long-horizon energy management", "arrival gate"),
            ),
        ),
    )

    assert len(catalog.archetypes) == 5
    assert catalog.recipes[0].lineage_required is False


def test_lineage_validates_parent_child_events_and_intervals() -> None:
    lineage = ObjectLineage(
        nodes=(
            ObjectLineageNode(id="carrier", role="carrier_aircraft", active_from_s=0.0),
            ObjectLineageNode(id="x15", role="air_launched_vehicle", parent_id="carrier", spawn_event_id="release", active_from_s=3.0),
            ObjectLineageNode(id="booster", role="spent_booster", parent_id="x15", spawn_event_id="separation", active_from_s=92.0, active_to_s=140.0, death_event_id="booster-impact", terminal_disposition="ballistic_impact"),
        ),
        events=(
            ObjectLineageEvent(id="release", event_type="release", object_id="x15", parent_object_id="carrier", time_s=3.0, reason="carrier release"),
            ObjectLineageEvent(id="separation", event_type="separation", object_id="booster", parent_object_id="x15", time_s=92.0, reason="burnout separation"),
            ObjectLineageEvent(id="booster-impact", event_type="terminal", object_id="booster", time_s=140.0, reason="ballistic impact"),
        ),
    )

    assert lineage.nodes[2].parent_id == "x15"


def test_lineage_rejects_unknown_parent() -> None:
    with pytest.raises(ValueError, match="lineage parent is unknown"):
        ObjectLineage(
            nodes=(ObjectLineageNode(id="child", role="debris", parent_id="missing", active_from_s=1.0),),
        )


def test_lineage_archetype_requires_artifact_in_run_manifest() -> None:
    digest = hashlib.sha256(b"manifest").hexdigest()
    with pytest.raises(ValueError, match="object_lineage archetype"):
        ShowcaseRunArtifact(
            run_id="run-lineage-missing",
            showcase_id="org.taoryx.showcase.x15.storyboard",
            vehicle_binding_id="x15-v1",
            fidelity="point_mass_3dof",
            scenario_contract_sha256=digest,
            outcome="completed",
            claim="Declared mission storyboard completed.",
            files=(ArtifactFile(path="manifest.json", sha256=digest, media_type="application/json"),),
            board=EvidenceBoardSpec(profile="evidence-board-v1", modules=("mission_timeline",)),
            archetypes=("mission_timeline", "object_lineage"),
        )


def test_checked_in_b747_and_x15_catalog_recipes_validate() -> None:
    payload = yaml.safe_load((ROOT / "verification/showcase_archetype_catalog.yaml").read_text(encoding="utf-8"))
    catalog = ShowcaseArchetypeCatalog(**payload)
    recipes = {recipe.id: recipe for recipe in catalog.recipes}

    assert recipes["b747-transport-energy-arrival-v1"].lineage_required is False
    assert recipes["x15-boost-glide-storyboard-v1"].lineage_required is True
    assert "arrival_gate" in recipes["b747-transport-energy-arrival-v1"].required_events
    assert "separation" in recipes["x15-boost-glide-storyboard-v1"].required_events
    assert "x8-fixed-wing-racetrack-response-v1" in recipes
    assert "hummingbird-multirotor-hover-yaw-contact-v1" in recipes
    assert "tumbling-body-passive-deployment-v1" in recipes
    assert "shutdown" in recipes["hummingbird-multirotor-hover-yaw-contact-v1"].required_events
    assert "impact" in recipes["tumbling-body-passive-deployment-v1"].required_events
