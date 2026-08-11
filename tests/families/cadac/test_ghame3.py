from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.ghame3 import load_ghame3_source_definition, run_ghame3_source_compatibility

_INPUT = """TITLE GHAME3 fixture
OPTIONS y_plot
MODULES
environment def,init,exec
aerodynamics def,exec
propulsion def,init,exec
forces def,exec
newton def,init,exec
END
TIMING
plot_step 0.5
int_step 0.01
END
VEHICLES 1
CRUISE3 GHAME3
lonx -80.55
latx 28.43
alt 3000
psivgx 90
thtvgx 0
dvbe 250
alphax 7
phimvx 0
AERO_DECK ghame3_aero_deck.asc
area 557.42
PROP_DECK ghame3_prop_deck.asc
mprop 1
mass0 136077
fmass0 81646
acowl 27.87
throttle 0.2
thrtl_idle 0.05
thrtl_max 2
IF time > 0.01
mprop 2
qhold 50000
tq 1
alphax 2.5
ENDIF
END
END
ENDTIME 1
STOP
"""

_AERO = """TITLE aero
1DIM cd0_vs_mach
NX1 2
0 0.04
24 0.04
1DIM cl0_vs_mach
NX1 2
0 0.02
24 0.02
1DIM ckk_vs_mach
NX1 2
0 0.4
24 0.4
1DIM cla_vs_mach
NX1 2
0 0.03
24 0.03
1DIM cla0_vs_mach
NX1 2
0 -0.04
24 -0.04
"""

_PROP = """TITLE prop
2DIM ca_vs_alpha_mach
NX1 2 NX2 2
-3 0 1 1
21 24 1 1
2DIM spi_vs_throttle_mach
NX1 2 NX2 2
0 0 0 0
2 24 3000 3000
"""


def _case(tmp_path: Path) -> Path:
    project = tmp_path / "GHAME3"
    inputs = project / "Inputs"
    inputs.mkdir(parents=True)
    (project / "ghame3_aero_deck.asc").write_text(_AERO, encoding="utf-8")
    (project / "ghame3_prop_deck.asc").write_text(_PROP, encoding="utf-8")
    path = inputs / "input.asc"
    path.write_text(_INPUT, encoding="utf-8")
    return path


####


def test_ghame3_lowers_as_point_mass_round3(tmp_path: Path) -> None:
    definition = load_ghame3_source_definition(_case(tmp_path))
    assert definition.taoryx_tier == "point_mass_3dof"
    assert definition.control_realization == "force_model"
    assert definition.module_order == ("environment", "aerodynamics", "propulsion", "forces", "newton")
    assert {Path(item.resolved_path).name for item in definition.source_artifacts} == {
        "input.asc",
        "ghame3_aero_deck.asc",
        "ghame3_prop_deck.asc",
    }


####


def test_ghame3_fixed_throttle_source_epoch_is_finite(tmp_path: Path) -> None:
    definition = load_ghame3_source_definition(_case(tmp_path))
    run = run_ghame3_source_compatibility(definition, end_time_s=0.01, sample_step_s=0.5)
    sample = run.samples[0]
    assert sample.time_s == 0.0
    assert sample.propulsion_mode == 1
    assert sample.thrust_n > 0.0
    assert sample.mass_kg < definition.propulsion.initial_mass_kg
    assert sample.mach > 0.0
    assert sample.alpha_deg == 7.0
    assert sample.bank_deg == 0.0
    assert all(abs(value) < 100.0 for value in sample.specific_force_velocity_mps2)


####


def test_ghame3_time_event_preserves_pre_environment_source_lag(tmp_path: Path) -> None:
    definition = load_ghame3_source_definition(_case(tmp_path))
    run = run_ghame3_source_compatibility(definition, end_time_s=0.05, sample_step_s=0.01)
    assert len(run.events) == 1
    event = run.events[0]
    # event() runs before environment() refreshes the source 'time' variable.
    assert event.time_s == pytest.approx(0.03)
    assert dict(event.updates)["mprop"] == 2
    sample_after = next(sample for sample in run.samples if sample.time_s == pytest.approx(0.03))
    assert sample_after.propulsion_mode == 2
    assert sample_after.alpha_deg == 2.5


####
