from __future__ import annotations

import json
from pathlib import Path

from tools.validate_x15_fidelity_ladder import write_x15_fidelity_ladder


def test_x15_fidelity_ladder_writes_scoped_nominal_evidence_for_both_reduced_tiers(tmp_path: Path) -> None:
    manifest = write_x15_fidelity_ladder(tmp_path, step_size_s=1.0, horizon_s=600.0)

    assert manifest["status"] == "nominal_case_pass"
    assert len(manifest["records"]) == 2
    for record in manifest["records"]:
        artifact = json.loads((tmp_path / Path(str(record["artifact"])).name).read_text(encoding="utf-8"))
        assert artifact["status"] == "nominal_case_pass"
        assert artifact["nominal_case_scope"] == "x15.staged_event_energy_corridor_impact_v1"
        assert artifact["evaluation"]["mission_pass"] is True
        assert artifact["mission"]["independent_truth_evaluation"] is True
        assert artifact["control_path"]["physical_effectors"] == []
        assert artifact["claim"]["nonclaims"]
        assert artifact["physical_promotion_boundary"]
        corridor = next(item for item in artifact["mission"]["required_objectives"] if item["id"] == "high_energy_terminal_corridor")
        assert corridor["truth_result"] == "PASS"
        assert corridor["corridor"]["phase"] == "glide"
        assert corridor["actual"]["altitude_m"] > corridor["corridor"]["minimum_altitude_m"]
        handoff = next(item for item in artifact["mission"]["required_objectives"] if item["id"] == "atmospheric_terminal_handoff")
        assert handoff["truth_result"] == "PASS"
        assert handoff["corridor"]["control_mode"] == "open_loop_handoff_witness"
        assert artifact["mission"]["handoff_contract"] == "open_loop_atmospheric_terminal_handoff_gate"
