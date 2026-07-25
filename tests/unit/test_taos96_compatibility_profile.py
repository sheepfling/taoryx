"""Keep the TAOS 96.0 evidence boundary machine-checkable."""

from __future__ import annotations

from pathlib import Path

import yaml

ROOT = Path(__file__).parents[2]


def _profile() -> dict[str, object]:
    with (ROOT / "verification/taos96_compatibility_profile.yaml").open(encoding="utf-8") as stream:
        profile = yaml.safe_load(stream)
    assert isinstance(profile, dict)
    return profile
####


def test_taos96_profile_supports_bounded_claims_and_excludes_runtime_equivalence() -> None:
    profile = _profile()
    claims = profile["claims"]
    assert isinstance(claims, dict)
    assert claims["documentary_fidelity"]["status"] == "supported_bounded"
    assert claims["parser_conformance"]["status"] == "supported_bounded"
    assert claims["semantic_traceability"]["status"] == "supported_bounded"
    assert claims["numerical_kernel_correctness"]["status"] == "supported_by_kernel"
    historical = claims["historical_runtime_equivalence"]
    assert historical["status"] == "excluded_unverifiable"
    assert historical["required_oracle"]
    assert profile["historical_oracle_available"] is False
####


def test_taos96_profile_separates_successor_extensions() -> None:
    profile = _profile()
    extensions = profile["successor_extensions"]
    assert extensions["status"] == "explicitly_outside_historical_profile"
    assert "pseudo_6dof" in extensions["examples"]
    assert "rigid_body_6dof" in extensions["examples"]
    wording = profile["release_wording"]
    assert "evidence-bounded" in wording["allowed"].lower()
    assert "TAOS 96.0 runtime-compatible" in wording["forbidden"]
####
