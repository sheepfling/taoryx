from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from taoryx.fixtures.spectre_dumps import (
    group_families_by_phase,
    load_catalog,
    load_spec,
    render_workspace,
    validate_catalog_against_workspace,
)

ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "problem_file_dumps" / "spectre_segments"
SPEC = ROOT / "spec.yaml"
CATALOG = ROOT / "catalog.yaml"
pytestmark = pytest.mark.spectre


def test_spectre_segment_spec_renders_the_checked_in_problem_files() -> None:
    spec = load_spec(SPEC)
    rendered = render_workspace(spec)

    assert set(rendered) == {path.name for path in ROOT.glob("*.prb")}
    for name, text in rendered.items():
        assert (ROOT / name).read_text(encoding="utf-8") == text
    ####


def test_spectre_segment_manifest_matches_problem_files() -> None:
    manifest = yaml.safe_load((ROOT / "manifest.yaml").read_text(encoding="utf-8"))
    listed = manifest["files"]
    actual = sorted(path.name for path in ROOT.glob("*.prb"))

    assert listed == actual
    assert manifest["workspace"] == "problem_file_dumps"
    assert manifest["leaf"] == "spectre_segments"
    assert manifest["source_bundle"] == "../../spectre_simple_aero_v1"
    assert manifest["generated_catalog"] == "catalog.yaml"
####


def test_spectre_segment_catalog_groups_families_by_phase() -> None:
    catalog = load_catalog(CATALOG)
    workspace = load_spec(SPEC)
    grouped = group_families_by_phase(catalog)

    validate_catalog_against_workspace(catalog, workspace)

    assert [phase.name for phase in catalog.phases] == ["boost", "coast", "maneuver", "final_pronav"]
    assert [family.family for family in grouped["boost"]] == ["ballistic"]
    assert [family.family for family in grouped["maneuver"]] == [
        "cbcr",
        "crossrange",
        "marv",
        "phugoid",
        "range_extension",
        "skip",
        "slalom",
        "weave",
    ]
####


def test_spectre_segment_problem_files_are_minimally_structured() -> None:
    for path in sorted(ROOT.glob("*.prb")):
        text = path.read_text(encoding="utf-8").splitlines()
        nonblank = [line for line in text if line.strip()]
        assert nonblank[0].startswith("(")
        assert nonblank[-1] == "*end"
        assert any(line.startswith("# Synthetic TAOS translation") for line in nonblank)
    ####
