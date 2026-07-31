from __future__ import annotations

import json
from pathlib import Path

from tools.validate_hl20_fidelity_ladder import write_hl20_fidelity_ladder


def test_hl20_fidelity_ladder_preserves_synthetic_source_boundary(tmp_path: Path) -> None:
    manifest = write_hl20_fidelity_ladder(tmp_path, step_size_s=2.0, horizon_s=180.0)

    assert manifest["status"] == "nominal_case_pass"
    assert [record["fidelity"] for record in manifest["records"]] == ["point_mass_3dof", "pseudo_6dof"]
    assert json.loads((tmp_path / "comparison.json").read_text(encoding="utf-8"))["deltas"]
    for record in manifest["records"]:
        evidence = json.loads((tmp_path / Path(str(record["artifact"])).name).read_text(encoding="utf-8"))
        assert evidence["evaluation"]["mission_pass"] is True
        assert evidence["status"] == "nominal_case_pass"
        assert evidence["nominal_case_scope"] == "hl20.synthetic_release_glide_impact_v1"
        assert evidence["physical_promotion_boundary"]
        assert evidence["source_boundary"]["booster"] == "synthetic"
        assert evidence["claim"]["nonclaims"]
