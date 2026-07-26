from __future__ import annotations

import json
from pathlib import Path

from tools.validate_daveml_family_readiness import validate_readiness

ROOT = Path(__file__).resolve().parents[2]


def test_family_readiness_passes_source_and_graph_gates_without_promoting_derived_layers() -> None:
    report = validate_readiness(ROOT / "verification/daveml_family_readiness.yaml")

    assert report["status"] == "verified"
    families = {item["family_id"]: item for item in report["families"]}
    assert families["reference_f16_s119"]["checkdata"]["passed"] == 84
    assert families["reference_hl20_mod_k"]["checkdata"]["passed"] == 240
    assert families["reference_nesc_two_stage_rocket"]["checkdata"]["computed"] == "no_embedded_cases"
    assert families["reference_f16_s119"]["derived_layers"]["tuning"] == "source_channel_reduced_lqr_verified"
    assert families["reference_f16_s119"]["derived_layers"]["trim"] == "source_equilibrium_trim_verified"


def test_checked_in_readiness_report_is_json_and_matches_registry() -> None:
    report = json.loads((ROOT / "verification/daveml_family_readiness.json").read_text(encoding="utf-8"))

    assert report["status"] == "verified"
    assert len(report["families"]) == 3
