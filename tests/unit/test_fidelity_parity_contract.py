from __future__ import annotations

import json
from pathlib import Path

from tools.generate_fidelity_parity_contracts import build_contract

ROOT = Path(__file__).resolve().parents[2]


def test_parity_catalog_covers_all_four_families_and_preserves_blockers() -> None:
    contract = build_contract()

    assert {family["id"] for family in contract["families"]} == {"b747", "skywalker_x8", "hummingbird", "x15"}
    statuses = {family["id"]: family["status"] for family in contract["families"]}
    assert statuses["b747"] == "candidate"
    assert statuses["skywalker_x8"] == "candidate"
    assert statuses["hummingbird"] == "candidate"
    assert statuses["x15"] == "candidate"
    for family in contract["families"]:
        assert family["scenario_id"]
        assert family["inputs"]["point_mass_problem"]["sha256"]
        assert family["source_tables"]
        assert set(family["unit_profiles"]) == {"point_mass", "rigid_body"}
        assert family["reduction_kind"] in {"point-mass-3dof", "kinematic-3-plus-3-dof"}
        assert set(family["parity_tolerances"]) == {"altitude_m", "speed_m_s", "mass_kg"}
        assert set(family["continuity_tolerances"]) == {"altitude_m", "speed_m_s", "mass_kg"}


def test_generated_contract_is_current() -> None:
    expected = json.dumps(build_contract(), indent=2, sort_keys=True) + "\n"
    generated = ROOT / "verification/generated/fidelity_parity_contracts.json"
    assert generated.read_text(encoding="utf-8") == expected
