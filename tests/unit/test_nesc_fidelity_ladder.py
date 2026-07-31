from __future__ import annotations

import json
from pathlib import Path

from tools.validate_nesc_fidelity_ladder import write_nesc_fidelity_ladder


def test_nesc_fidelity_ladder_preserves_source_translation_and_surrogate_boundary(tmp_path: Path) -> None:
    manifest = write_nesc_fidelity_ladder(tmp_path)

    assert manifest["status"] == "nominal_case_pass"
    assert [record["fidelity"] for record in manifest["records"]] == ["point_mass_3dof", "pseudo_6dof"]
    point = json.loads((tmp_path / "point_mass_3dof_evidence.json").read_text(encoding="utf-8"))
    pseudo = json.loads((tmp_path / "pseudo_6dof_evidence.json").read_text(encoding="utf-8"))
    assert point["evaluation"]["mission_pass"] is True
    assert pseudo["evaluation"]["mission_pass"] is True
    assert point["status"] == "nominal_case_pass"
    assert pseudo["status"] == "nominal_case_pass"
    assert point["nominal_case_scope"] == "nesc.source_translation_staged_orbit_coast_v1"
    assert pseudo["physical_promotion_boundary"]
    assert pseudo["control_path"]["gimbal_allocation"] is False
    assert json.loads((tmp_path / "comparison.json").read_text(encoding="utf-8"))["translation_rows_equal"] is True
