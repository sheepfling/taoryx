from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.language.analytical_tables import lower_analytical_table, parse_analytical_table_text
from taoryx.language.table_parser import parse_table_text

pytestmark = pytest.mark.grammar
FIXTURES = Path(__file__).parents[1] / "fixtures" / "analytical_tables"


def test_analytical_syntax_is_opt_in_and_historical_table_parser_remains_separate() -> None:
    source = "(demo)\ngenerate cd(alphat) from shape=sphere\n"

    disabled = parse_analytical_table_text(source)
    assert disabled.declaration is None
    assert disabled.diagnostics[0].code == "analytical-extension-disabled"

    historical = parse_table_text(source)
    assert historical.tables == []
    assert historical.diagnostics
####


def test_cone_declaration_lowers_to_an_ordinary_validated_table() -> None:
    source = """(demo-cone)
generate cd(alphat) from shape=cone
  radius=0.5 height=2.0 reference-area=0.7853981634
  model=front-broadside-rear cd-front=0.30 cd-broadside=1.10 cd-rear=1.25
  alphat start=0 stop=180 step=45
  no-extrap
"""

    parsed = parse_analytical_table_text(source, "demo.tbl", enabled=True)
    assert not parsed.diagnostics
    assert parsed.declaration is not None

    lowered = lower_analytical_table(parsed.declaration)
    ordinary = parse_table_text(lowered.table_text, "generated.tbl")
    assert ordinary.executable_complete
    assert not ordinary.diagnostics
    assert ordinary.tables[0].name == "demo-cone"
    assert ordinary.tables[0].assignments[-1].name == "cd"
    assert lowered.provenance_fingerprint == lowered.provenance["source_fingerprint"]
    assert "radius" in str(lowered.provenance["parameters_json"])
    assert "45.0" in str(lowered.provenance["axis_grids_json"])
    assert lower_analytical_table(parsed.declaration).table_text == lowered.table_text
####


def test_triaxial_declaration_preserves_two_axis_lowering() -> None:
    source = """(demo-ellipsoid)
generate cd(alphat,phi) from shape=triaxial-ellipsoid
  axes=1.0,0.6,0.4 reference-area=0.7539822369
  alphat start=0 stop=180 step=90
  phi start=0 stop=360 step=180
  no-extrap
"""

    parsed = parse_analytical_table_text(source, enabled=True)
    assert not parsed.diagnostics
    assert parsed.declaration is not None
    lowered = lower_analytical_table(parsed.declaration)
    ordinary = parse_table_text(lowered.table_text)
    assert ordinary.executable_complete
    assert ordinary.tables[0].independent_variables == ["alphat", "phi"]
    assert len(ordinary.tables[0].assignments[-1].values) == 9
####


@pytest.mark.parametrize(
    ("shape", "parameters"),
    [
        ("sphere", "radius=0.5"),
        ("spheroid", "axial-semi-axis=1.0 transverse-semi-axis=0.5"),
        ("cylinder", "radius=0.5 length=2.0"),
        ("cone", "radius=0.5 height=2.0"),
    ],
)
def test_each_axisymmetric_shape_family_lowers_to_a_validated_cd_table(shape: str, parameters: str) -> None:
    source = f"""({shape})
generate cd(alphat) from shape={shape}
  {parameters} reference-area=0.7853981634
  alphat start=0 stop=90 step=45
  no-extrap
"""
    parsed = parse_analytical_table_text(source, enabled=True)
    assert not parsed.diagnostics
    assert parsed.declaration is not None
    ordinary = parse_table_text(lower_analytical_table(parsed.declaration).table_text)
    assert ordinary.executable_complete
    assert not ordinary.diagnostics
####


@pytest.mark.parametrize(
    ("source", "code"),
    [
        ("(x)\ngenerate cd(alphat) from shape=unknown\n", "analytical-unsupported-shape"),
        ("(x)\ngenerate cd(alphat) from shape=cone\n  alphat start=0 stop=180 step=0\n", "analytical-missing-reference-area"),
        ("(x)\ngenerate cd(alphat) from shape=cone\n  reference-area=0\n  alphat start=0 stop=180 step=5\n", "analytical-invalid-reference-area"),
    ],
)
def test_analytical_declarations_report_source_located_diagnostics(source: str, code: str) -> None:
    result = parse_analytical_table_text(source, "broken.tbl", enabled=True)
    assert any(item.code == code and item.location is not None for item in result.diagnostics)
####


def test_checked_in_analytical_fixtures_round_trip_through_lowering() -> None:
    cone = parse_analytical_table_text((FIXTURES / "cone.tbl").read_text(), enabled=True)
    ellipsoid = parse_analytical_table_text((FIXTURES / "triaxial-ellipsoid.tbl").read_text(), enabled=True)
    invalid = parse_analytical_table_text((FIXTURES / "missing-dimension.tbl").read_text(), enabled=True)

    assert cone.declaration is not None
    assert ellipsoid.declaration is not None
    assert any(item.code == "analytical-missing-parameter" for item in invalid.diagnostics)
    assert parse_table_text(lower_analytical_table(cone.declaration).table_text).executable_complete
    assert parse_table_text(lower_analytical_table(ellipsoid.declaration).table_text).executable_complete
####
