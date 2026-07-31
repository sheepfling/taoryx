from __future__ import annotations

import json
from pathlib import Path

import pytest

from tools.validate_passive_deployment import run_qualification


def test_passive_deployment_qualification_is_deterministic_and_covers_all_shapes(tmp_path: Path) -> None:
    first = run_qualification(
        tmp_path / "first-report.json",
        tmp_path / "first-artifacts",
        samples_per_shape=2,
        horizon_s=90.0,
        step_size_s=0.5,
        dpi=80,
    )
    second = run_qualification(
        tmp_path / "second-report.json",
        tmp_path / "second-artifacts",
        samples_per_shape=2,
        horizon_s=90.0,
        step_size_s=0.5,
        dpi=80,
    )

    assert first["status"] == "qualified_witness_with_physical_footprint_uncertainty"
    assert first["execution"] == second["execution"]
    assert [shape["shape"] for shape in first["shapes"]] == ["sphere", "cylinder", "cone", "triaxial_ellipsoid"]
    for shape in first["shapes"]:
        assert shape["uncertainty"]["classification_counts"] == {"impact": 2}
        assert shape["uncertainty"]["impact_distribution"]["finite_count"] == 2
        assert set(shape["nominal"]) == {"point_mass_3dof", "pseudo_6dof", "rigid_body_6dof"}
        assert {tier["classification"] for tier in shape["nominal"].values()} == {"impact"}
        point_area = shape["nominal"]["point_mass_3dof"]["area_policy_witness"]
        pseudo_area = shape["nominal"]["pseudo_6dof"]["area_policy_witness"]
        rigid_area = shape["nominal"]["rigid_body_6dof"]["area_policy_witness"]
        assert point_area["policies"] == ["orientation_averaged_projected_area"]
        assert point_area["ratio_to_reference_average_min"] == pytest.approx(1.0)
        assert point_area["ratio_to_reference_average_max"] == pytest.approx(1.0)
        assert pseudo_area["policies"] == ["native_rigid_body_reuse_instantaneous_projected_area"]
        assert rigid_area["policies"] == ["instantaneous_geometry_projected_area"]
        assert pseudo_area["finite"] is True
        assert rigid_area["finite"] is True
        assert pseudo_area["angular_rate_max_rad_s"] > 0.0
        assert rigid_area["angular_rate_max_rad_s"] > 0.0
        rotation = shape["rotation_policy_witness"]
        assert rotation["comparison"] == "passive_aerodynamic_moment_vs_zero_moment_spin_baseline"
        assert set(rotation["records"]) == {
            "passive_aerodynamic_moment",
            "prescribed_spin_zero_aerodynamic_moment",
        }
        assert all(record["finite"] is True for record in rotation["records"].values())
        assert rotation["records"]["prescribed_spin_zero_aerodynamic_moment"]["aerodynamic_moment_norm_max_nm"] == pytest.approx(0.0)

    assert first["contract"]["release_state"] == {
        "initial_altitude_m": 1000.0,
        "initial_speed_m_s": 250.0,
        "azimuth_rad": 0.0,
        "elevation_rad": 0.0,
        "bank_rad": 0.0,
        "separation_impulse_n_s": [0.0, 0.0, 0.0],
    }

    manifest = tmp_path / "first-artifacts" / "deployment-manifest.json"
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    assert payload["claim_boundary"].startswith("passive child terminal footprints")
    assert any(item["path"].endswith("terminal-footprint-samples.csv") for item in payload["files"])
    assert any(item["path"].endswith("area-policy-rotation.png") for item in payload["files"])
