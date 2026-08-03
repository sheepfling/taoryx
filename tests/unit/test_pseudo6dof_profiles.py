"""Contract tests for the Alpha 3 pseudo-6DOF profile catalog."""

from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.trajectory.pseudo6dof_profiles import (
    Pseudo6DOFProfile,
    build_automatic_lowering_report,
    load_pseudo6dof_catalog,
    load_qualified_fidelity_evidence,
)

ROOT = Path(__file__).resolve().parents[2]


def test_alpha3_catalog_covers_all_target_families() -> None:
    catalog = load_pseudo6dof_catalog(ROOT / "verification/pseudo6dof_profiles.yaml")

    assert len(catalog.bindings) == 9
    assert len(catalog.direct_wrench_profiles) == 8
    assert len(catalog.surface_allocation_profiles) == 8
    assert {item.family_id for item in catalog.bindings} == {
        "skywalker_x8",
        "b747",
        "a320_openap_3dof",
        "f16_s119",
        "x15",
        "hummingbird",
        "hl20_mod_k",
        "reference_nesc_two_stage_rocket",
        "tumbling_body",
    }
    assert all(profile.unsupported_claims for profile in catalog.profiles)
    assert {profile.control_realization for profile in catalog.profiles} == {"response_law", "uncontrolled"}
    assert {profile.family_id for profile in catalog.direct_wrench_profiles} >= {"hummingbird", "hl20_mod_k", "x15"}
    assert {profile.family_id for profile in catalog.surface_allocation_profiles} >= {"skywalker_x8", "b747", "hummingbird", "hl20_mod_k"}
    ####


def test_tumbling_body_uses_rigid_body_reuse_not_prescribed_attitude() -> None:
    catalog = load_pseudo6dof_catalog(ROOT / "verification/pseudo6dof_profiles.yaml")
    binding, profile = catalog.for_family("tumbling_body")

    assert binding.automatic_lowering is False
    assert profile.model_kind == "rigid_body_reuse"
    assert profile.response == {}
    assert profile.area_policy == "average_projected_area"
    ####


def test_x15_declares_auditable_powered_coast_glide_response_schedule() -> None:
    """The staged pseudo bridge exposes its active regime explicitly."""

    catalog = load_pseudo6dof_catalog(ROOT / "verification/pseudo6dof_profiles.yaml")
    _, profile = catalog.for_family("x15")

    assert set(profile.phase_response) == {"boost", "coast", "glide"}
    for phase in ("boost", "coast", "glide"):
        label, response = profile.response_for_phase(phase)
        assert label == phase
        assert set(response) == {"roll", "pitch", "yaw"}
    label, response = profile.response_for_phase("unprofiled")
    assert label == "default"
    assert response == profile.response
    ####


def test_surrogate_profiles_require_all_attitude_axes() -> None:
    with pytest.raises(ValueError, match="roll, pitch, and yaw"):
        Pseudo6DOFProfile(
            id="invalid",
            family_id="test",
            parent_3dof_profile_id="test.3dof",
            model_kind="attitude_response_surrogate",
            control_realization="response_law",
            status="planned",
            evidence_grade="synthetic",
            response={},
            unsupported_claims=("everything physical",),
        )
    ####


def test_rigid_body_reuse_rejects_surrogate_response_axes() -> None:
    with pytest.raises(ValueError, match="must not declare surrogate response axes"):
        Pseudo6DOFProfile(
            id="invalid",
            family_id="tumbling_body",
            parent_3dof_profile_id="tumbling_body.3dof",
            model_kind="rigid_body_reuse",
            control_realization="response_law",
            status="development",
            evidence_grade="derived",
            response={
                "roll": {
                    "time_constant_s": 1.0,
                    "damping_ratio": 0.7,
                    "maximum_rate_rad_s": 1.0,
                    "maximum_acceleration_rad_s2": 1.0,
                }
            },
            unsupported_claims=("prescribed tumble",),
        )
    ####


def test_automatic_lowering_stops_at_passive_rigid_reuse_boundary() -> None:
    """Passive rigid-body reuse does not invent an automatic controller tier."""

    catalog = load_pseudo6dof_catalog(ROOT / "verification/pseudo6dof_profiles.yaml")
    report = build_automatic_lowering_report(
        "tumbling_body",
        "rigid_body_6dof",
        {
            "tumbling_body.average_area_3dof.v1": {"status": "equivalence_passed"},
        },
        catalog=catalog,
    )

    assert not report.accepted
    assert report.selected is None
    assert [step.fidelity for step in report.steps] == ["rigid_body_6dof"]
    assert "no native rigid-body qualification profile" in report.steps[0].reason
    ####


def test_passive_rigid_reuse_is_nominal_but_not_a_controller_fallback() -> None:
    """Passive pseudo evidence is solid without authorizing invented control."""

    catalog = load_pseudo6dof_catalog(ROOT / "verification/pseudo6dof_profiles.yaml")
    binding, profile = catalog.for_family("tumbling_body")

    assert profile.status == "nominal_case_pass"
    assert profile.model_kind == "rigid_body_reuse"
    assert profile.control_realization == "uncontrolled"
    assert binding.automatic_lowering is False
    ####


def test_canonical_lowering_never_emits_ambiguous_legacy_rigid_step() -> None:
    """Canonical four-tier requests must remain canonical at passive boundaries."""

    catalog = load_pseudo6dof_catalog(ROOT / "verification/pseudo6dof_profiles.yaml")
    report = build_automatic_lowering_report(
        "tumbling_body",
        "rigid_body_6dof_surface_allocated",
        {"tumbling_body.average_area_3dof.v1": {"status": "equivalence_passed"}},
        catalog=catalog,
    )

    assert report.selected is None
    assert all(step.fidelity != "rigid_body_6dof" for step in report.steps)
    assert report.steps[0].fidelity == "rigid_body_6dof_surface_allocated"
    ####


def test_automatic_lowering_accepts_scoped_x15_event_energy_policy() -> None:
    """The X-15 staged witness may lower only under its declared scope."""

    report = build_automatic_lowering_report(
        "x15",
        "rigid_body_6dof",
        {
            "x15.point_mass_3dof.v1": {"status": "equivalence_passed"},
            "x15.attitude_response_p6dof.v1": {"status": "nominal_case_pass"},
        },
    )

    assert report.accepted
    assert report.selected == "pseudo_6dof"
    assert next(step for step in report.steps if step.fidelity == "pseudo_6dof").status == "eligible"
    ####


def test_automatic_lowering_accepts_scoped_hummingbird_nominal_policy() -> None:
    """The scoped Hummingbird nominal packet may lower to its pseudo tier."""

    report = build_automatic_lowering_report(
        "hummingbird",
        "rigid_body_6dof",
        {
            "hummingbird.point_mass_3dof.v1": {"status": "equivalence_passed"},
            "hummingbird.attitude_response_p6dof.v1": {"status": "nominal_case_pass"},
        },
    )

    assert report.accepted
    assert report.selected == "pseudo_6dof"
    assert len(report.steps) == 3
    assert report.steps[0].status == "unavailable"
    assert report.steps[1].status == "blocked"
    assert report.steps[2].status == "eligible"
    ####


def test_direct_wrench_is_an_explicit_bridge_before_pseudo_lowering() -> None:
    """A checked direct bridge is selectable without claiming physical effectors."""

    catalog = load_pseudo6dof_catalog(ROOT / "verification/pseudo6dof_profiles.yaml")
    evidence = load_qualified_fidelity_evidence(ROOT / "verification/alpha3_fidelity_evidence.yaml")
    for family_id in ("hummingbird", "hl20_mod_k", "x15"):
        binding, profile = catalog.for_family_direct_wrench(family_id)
        report = build_automatic_lowering_report(family_id, "rigid_body_6dof", evidence, catalog=catalog)
        assert binding.direct_wrench_profile_id == profile.id
        assert report.accepted
        assert report.selected == "rigid_body_6dof_direct_wrench"
        assert report.steps[-1].profile_id == profile.id
    ####


def test_surface_allocation_is_selectable_only_with_explicit_surface_evidence() -> None:
    """The highest tier requires its own checked evidence and its direct parent."""

    catalog = load_pseudo6dof_catalog(ROOT / "verification/pseudo6dof_profiles.yaml")
    binding, surface = catalog.for_family_surface_allocated("skywalker_x8")
    evidence = {
        surface.id: {"status": "nominal_case_pass"},
        "skywalker_x8.direct_wrench_6dof.v1": {"status": "nominal_case_pass"},
        "skywalker_x8.point_mass_3dof.v1": {"status": "nominal_case_pass"},
    }
    report = build_automatic_lowering_report(
        "skywalker_x8",
        "rigid_body_6dof_surface_allocated",
        evidence,
        catalog=catalog,
    )
    assert binding.surface_allocation_profile_id == surface.id
    assert report.selected == "rigid_body_6dof_surface_allocated"
    assert report.steps[0].status == "eligible"
    ####


def test_lowering_combines_manifest_operations_with_profile_evidence() -> None:
    """A qualified profile still lowers when its adapter operation is absent."""

    catalog = load_pseudo6dof_catalog(ROOT / "verification/pseudo6dof_profiles.yaml")
    evidence = {
        "skywalker_x8.surface_allocated_6dof.v1": {"status": "nominal_case_pass"},
        "skywalker_x8.direct_wrench_6dof.v1": {"status": "nominal_case_pass"},
        "skywalker_x8.attitude_response_p6dof.v1": {"status": "nominal_case_pass"},
        "skywalker_x8.point_mass_3dof.v1": {"status": "nominal_case_pass"},
    }
    report = build_automatic_lowering_report(
        "skywalker_x8",
        "rigid_body_6dof_surface_allocated",
        evidence,
        catalog=catalog,
        required_operations={
            "rigid_body_6dof_surface_allocated": ("trim", "effectiveness", "allocate"),
        },
        operation_status={
            "rigid_body_6dof_surface_allocated": {
                "trim": "not_available",
                "effectiveness": "available",
                "allocate": "available",
            },
        },
    )

    assert report.selected == "rigid_body_6dof_direct_wrench"
    surface_step = report.steps[0]
    assert surface_step.required_operations == ("trim", "effectiveness", "allocate")
    assert surface_step.missing_operations == ("trim",)
    ####


def test_all_controlled_families_share_the_direct_wrench_bridge_contract() -> None:
    """Every controlled family selects the same bridge tier; passive bodies do not."""

    catalog = load_pseudo6dof_catalog(ROOT / "verification/pseudo6dof_profiles.yaml")
    evidence = load_qualified_fidelity_evidence(ROOT / "verification/alpha3_fidelity_evidence.yaml")
    controlled = (
        "skywalker_x8",
        "b747",
        "a320_openap_3dof",
        "f16_s119",
        "x15",
        "hummingbird",
        "hl20_mod_k",
        "reference_nesc_two_stage_rocket",
    )
    for family_id in controlled:
        report = build_automatic_lowering_report(family_id, "rigid_body_6dof", evidence, catalog=catalog)
        assert report.selected == "rigid_body_6dof_direct_wrench"
    passive = build_automatic_lowering_report("tumbling_body", "rigid_body_6dof", evidence, catalog=catalog)
    assert passive.selected is None
    ####


def test_automatic_lowering_selects_qualified_pseudo_profile() -> None:
    """A qualified pseudo profile is preferred over its point-mass parent."""

    report = build_automatic_lowering_report(
        "f16_s119",
        "pseudo_6dof",
        {
            "f16_s119.attitude_response_p6dof.v1": {"status": "nominal_case_pass"},
            "f16_s119.point_mass_3dof.v1": {"status": "equivalence_passed"},
        },
    )

    assert report.accepted
    assert report.selected == "pseudo_6dof"
    assert len(report.steps) == 1
    assert report.steps[0].status == "eligible"
    ####


def test_automatic_lowering_does_not_select_pseudo_without_parent_evidence() -> None:
    """A pseudo response result cannot hide an unqualified translational plant."""

    report = build_automatic_lowering_report(
        "f16_s119",
        "pseudo_6dof",
        {"f16_s119.attitude_response_p6dof.v1": {"status": "nominal_case_pass"}},
    )

    assert not report.accepted
    assert report.selected is None
    assert report.steps[0].status == "blocked"
    assert "parent 3-DOF evidence is missing" in report.steps[0].reason
    ####


def test_checked_nominal_evidence_authorizes_all_current_nominal_lowering() -> None:
    """Canonical passing artifacts enable only the tiers they actually prove."""

    evidence = load_qualified_fidelity_evidence(ROOT / "verification/alpha3_fidelity_evidence.yaml")
    assert set(evidence) == {
        "skywalker_x8.point_mass_3dof.v1",
        "skywalker_x8.attitude_response_p6dof.v1",
        "b747.point_mass_3dof.v1",
        "b747.attitude_response_p6dof.v1",
        "a320_openap_3dof.point_mass_3dof.v1",
        "a320.attitude_response_p6dof.v1",
        "a320.direct_wrench_6dof.v1",
        "f16_s119.point_mass_3dof.v1",
        "f16_s119.attitude_response_p6dof.v1",
        "f16_s119.direct_wrench_6dof.v1",
        "x15.point_mass_3dof.v1",
        "x15.attitude_response_p6dof.v1",
        "hummingbird.point_mass_3dof.v1",
        "hummingbird.attitude_response_p6dof.v1",
        "hl20_mod_k.point_mass_3dof.v1",
        "hl20.attitude_response_p6dof.v1",
            "nesc_rocket.performance_3dof.v1",
            "nesc_rocket.attitude_response_p6dof.v1",
        "skywalker_x8.direct_wrench_6dof.v1",
        "b747.direct_wrench_6dof.v1",
        "x15.direct_wrench_6dof.v1",
        "hummingbird.direct_wrench_6dof.v1",
        "hl20.direct_wrench_6dof.v1",
        "nesc_rocket.direct_wrench_6dof.v1",
            "tumbling_body.average_area_3dof.v1",
            "tumbling_body.rigid_body_reuse_p6dof.v1",
        }
    for family_id in ("skywalker_x8", "b747", "a320_openap_3dof", "f16_s119", "x15", "hummingbird", "hl20_mod_k", "reference_nesc_two_stage_rocket"):
        report = build_automatic_lowering_report(family_id, "pseudo_6dof", evidence)
        assert report.accepted
        assert report.selected == "pseudo_6dof"
    ####


def test_rigid_body_reuse_requires_explicit_rigid_realization() -> None:
    """Native reuse cannot be mislabeled as a kinematic response law."""

    with pytest.raises(ValueError, match="uncontrolled realization"):
        Pseudo6DOFProfile(
            id="invalid",
            family_id="tumbling_body",
            parent_3dof_profile_id="tumbling_body.3dof",
            model_kind="rigid_body_reuse",
            control_realization="response_law",
            status="development",
            evidence_grade="derived",
            unsupported_claims=("prescribed tumble",),
        )
    ####
