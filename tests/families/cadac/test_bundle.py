from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.bundle import load_cadac_source_bundle
from taoryx.families.cadac.input_ast import CadacDeckKind

INPUT = """\
TITLE bundle.asc Bundle fixture
OPTIONS y_plot
MODULES
    environment def,exec
END
TIMING
    int_step 0.01
END
VEHICLES 1
    AIM5 Missile
        AERO_DECK aero.asc
        PROP_DECK prop.asc
    END
ENDTIME 1
STOP
"""

DECK = """\
TITLE fixture
1DIM value_vs_time
NX1 2
0 1
1 2
"""


def _write_bundle(tmp_path: Path) -> Path:
    input_path = tmp_path / "input.asc"
    input_path.write_text(INPUT, encoding="utf-8")
    (tmp_path / "aero.asc").write_text(DECK, encoding="utf-8")
    (tmp_path / "prop.asc").write_text(DECK.replace("value_vs_time", "mass_vs_time"), encoding="utf-8")
    return input_path


####


def test_load_bundle_resolves_vehicle_decks(tmp_path: Path) -> None:
    bundle = load_cadac_source_bundle(_write_bundle(tmp_path))

    assert len(bundle.deck_bindings) == 2
    assert len(bundle.artifacts) == 3
    assert len(bundle.input_artifact.sha256) == 64
    assert all(len(binding.artifact.sha256) == 64 for binding in bundle.deck_bindings)
    assert [binding.keyword for binding in bundle.decks_for("aim5")] == ["AERO_DECK", "PROP_DECK"]
    assert bundle.deck_for("AIM5", CadacDeckKind.AERODYNAMIC).table("value_vs_time").values == (1.0, 2.0)
    assert bundle.deck_for("AIM5", CadacDeckKind.PROPULSION).table("mass_vs_time").values == (1.0, 2.0)


####


def test_load_bundle_reports_missing_deck_at_source_line(tmp_path: Path) -> None:
    input_path = _write_bundle(tmp_path)
    (tmp_path / "prop.asc").unlink()

    with pytest.raises(FileNotFoundError, match="referenced deck does not exist: prop.asc"):
        load_cadac_source_bundle(input_path)
    ####


####


def test_bundle_allows_multiple_same_model_vehicles_to_share_decks(tmp_path: Path) -> None:
    input_path = _write_bundle(tmp_path)
    text = input_path.read_text(encoding="utf-8")
    first = """    AIM5 Missile\n        AERO_DECK aero.asc\n        PROP_DECK prop.asc\n    END\n"""
    replacement = first + """    AIM5 Missile_2\n        AERO_DECK aero.asc\n        PROP_DECK prop.asc\n    END\n"""
    input_path.write_text(text.replace("VEHICLES 1", "VEHICLES 2").replace(first, replacement), encoding="utf-8")

    bundle = load_cadac_source_bundle(input_path)
    assert len(bundle.case.vehicles_named("AIM5")) == 2
    assert len(bundle.decks_for("AIM5")) == 4
    assert len(bundle.decks_for("AIM5", vehicle_role="Missile_2")) == 2
    assert bundle.deck_for("AIM5", CadacDeckKind.AERODYNAMIC, vehicle_role="Missile_2").tables
    assert len(bundle.artifacts) == 3


####


def test_bundle_resolves_project_level_decks_for_inputs_subdirectory(tmp_path: Path) -> None:
    project = tmp_path / "GHAME3"
    inputs = project / "Inputs"
    inputs.mkdir(parents=True)
    (project / "aero.asc").write_text("TITLE aero\n1DIM a\nNX1 2\n0 0\n1 1\n", encoding="utf-8")
    case = inputs / "input.asc"
    case.write_text(
        """TITLE nested input\nOPTIONS y_plot\nMODULES\naerodynamics def,exec\nEND\nTIMING\nint_step 0.1\nEND\nVEHICLES 1\nCRUISE3 Vehicle\nAERO_DECK aero.asc\nEND\nEND\nENDTIME 1\nSTOP\n""",
        encoding="utf-8",
    )
    bundle = load_cadac_source_bundle(case)
    binding = bundle.deck_binding_for("CRUISE3", CadacDeckKind.AERODYNAMIC)
    assert Path(binding.artifact.resolved_path).name == "aero.asc"
    assert binding.deck.table("a").interpolate((0.5,)) == pytest.approx(0.5)


####
