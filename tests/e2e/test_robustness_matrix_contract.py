from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from tools.run_robustness_matrix import _variants

ROOT = Path(__file__).resolve().parents[2]
MATRIX = yaml.safe_load((ROOT / "verification/acceptance/robustness_matrix_v1.yaml").read_text(encoding="utf-8"))

pytestmark = pytest.mark.slow


def test_robustness_matrix_declares_three_by_three_by_three_axes() -> None:
    families = [family for family in MATRIX["families"] if family.get("status") != "evidence_only"]
    assert {family["id"] for family in families} == {"SV01", "SV03", "SV05"}
    for family in families:
        variants = _variants(family)
        assert len(variants) == 27
        assert int(MATRIX["gates"]["bounded_robustness"]["minimum_cases_per_family"]) == 9
    ####


def test_cahi_is_not_in_the_engineering_pass_set() -> None:
    cahi = next(family for family in MATRIX["families"] if family["id"] == "CAHI")
    assert cahi["status"] == "evidence_only"
    assert MATRIX["claim_boundary"]["engineering_validity"] == "unproven"
    ####
