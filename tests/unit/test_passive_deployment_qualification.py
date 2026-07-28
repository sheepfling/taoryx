from __future__ import annotations

import json
from pathlib import Path

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
