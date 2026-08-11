from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.cruise5 import load_cruise5_source_definition, run_cruise5_source_compatibility
from taoryx.families.cadac.input_parser import parse_cadac_input

_INPUT = """TITLE inguid1: Waypoint/line guidance against GPS target coordinates
OPTIONS y_plot
MODULES
environment def,init,exec
aerodynamics def,exec
propulsion def,init,exec
forces def,exec
newton def,init,exec
guidance def,exec
control def,exec
intercept def,exec
END
TIMING
plot_step .5
int_step 0.05
END
VEHICLES 1
CRUISE3 Missile
lonx 45
latx 35
alt 1000
psivgx 90
thtvgx 1
dvbe 260
alphax 1.0
phimvx 0.0
AERO_DECK cruise3_aero_deck.asc
mprop 4
mach_com 0.7
mass_init 833
fuel_init 50
gfthm 893620
tfth 1
PROP_DECK cruise3_prop_deck.asc
mguidance 30
line_gain 1
nl_gain_fact .6
decrement 1000
mcontrol 46
anposlimx 3
anneglimx -1
gacp 10
ta .8
alpposlimx 15
alpneglimx -10
gcp 2
allimx 1
philimx 70
tphi .5
altcom 2000
altdlim 50
gh .3
gv 1.0
wp_lonx 45.15
wp_latx 35.15
psifgx 45
IF wp_grdrange < 100
wp_lonx 45.25
wp_latx 35.25
psifgx 90
ENDIF
IF wp_grdrange < 100
mguidance 33
mcontrol 44
psifgx 80
thtfgx -60
wp_lonx 45.38
wp_latx 35.25
wp_alt 100
ENDIF
END
END
ENDTIME 250
STOP
"""

_AERO = """TITLE Cruise3 aero deck for BSTA 134 in
1DIM cd0_vs_mach
NX1 6
0.4000 0.0506
0.5500 0.0462
0.6500 0.0435
0.7700 0.0410
0.8500 0.0506
0.9500 0.1078
1DIM cl0_vs_mach
NX1 6
0.4000 0.2670
0.5500 0.2279
0.6500 0.2540
0.7700 0.2527
0.8500 0.2139
0.9500 0.0613
1DIM ckk_vs_mach
NX1 6
0.4000 0.0567
0.5500 0.0840
0.6500 0.1212
0.7700 0.1488
0.8500 0.1513
0.9500 0.1931
1DIM cla_vs_mach
NX1 6
0.4000 0.1106
0.5500 0.1121
0.6500 0.1121
0.7700 0.1159
0.8500 0.1230
0.9500 0.1071
1DIM cla0_vs_mach
NX1 6
0.4000 0.2482
0.5500 0.2180
0.6500 0.1647
0.7700 0.1106
0.8500 0.0771
0.9500 -0.0541
"""

_PROP = """TITLE Cruise3 Propulsion Deck
1DIM cg_vs_mass
NX1 5
827 135
863 138
1116 130
1352 136
1418 133
1DIM iff_vs_alt
NX1 5
0 0.0182
3048 0.0107
6096 0.0084
9144 0.0084
12192 0.0084
2DIM tav_vs_alt_mach
NX1 3 NX2 4
0 0.4 2168.0 2088.8 2043.9 1987.8
1524 0.55 1942.9 1891.3 1863.3 1846.8
3048 0.7 1711.6 1677.3 1676.0 1683.6
0.85
2DIM fidle_vs_alt_mach
NX1 5 NX2 4
0 0.4 386.08 305.13 229.07 177.92
3048 0.55 304.68 256.64 214.39 178.36
6096 0.7 298.46 249.53 203.71 157.45
9144 0.85 366.51 324.25 286.45 254.42
12192 386.53 358.06 332.71 306.02
3DIM ff_vs_thrust_alt_mach
NX1 5 NX2 3 NX3 4
0 0 0.4 0.0050 0.0072 0.0093 0.0113 0.0045 0.0060 0.0076 0.0091 0.0040 0.0049 0.0058 0.0067
445 1524 0.55 0.0149 0.0170 0.0193 0.0217 0.0137 0.0155 0.0174 0.0194 0.0129 0.0142 0.0158 0.0174
890 3048 0.7 0.0242 0.0270 0.0297 0.0323 0.0229 0.0252 0.0275 0.0300 0.0217 0.0239 0.0260 0.0281
1446 0.85 0.0369 0.0401 0.0434 0.0471 0.0352 0.0385 0.0418 0.0450 0.0348 0.0377 0.0401 0.0430
2179 0.0549 0.0593 0.0630 0.0679 0.0556 0.0593 0.0617 0.0661 0.0568 0.0605 0.0636 0.0661
"""


def _case(tmp_path: Path) -> Path:
    (tmp_path / "input.asc").write_text(_INPUT, encoding="utf-8")
    (tmp_path / "cruise3_aero_deck.asc").write_text(_AERO, encoding="utf-8")
    (tmp_path / "cruise3_prop_deck.asc").write_text(_PROP, encoding="utf-8")
    return tmp_path / "input.asc"


####


def test_parser_accepts_optional_outer_vehicle_end() -> None:
    parsed = parse_cadac_input(_INPUT)
    assert parsed.vehicles[0].model_name == "CRUISE3"
    assert len(parsed.vehicles[0].events) == 2


####


def test_cruise5_source_lowering(tmp_path: Path) -> None:
    definition = load_cruise5_source_definition(_case(tmp_path))
    assert definition.taoryx_tier == "pseudo_6dof"
    assert definition.module_order == (
        "environment",
        "aerodynamics",
        "propulsion",
        "forces",
        "newton",
        "guidance",
        "control",
        "intercept",
    )
    assert definition.propulsion_deck.table("ff_vs_thrust_alt_mach").dimension == 3


####


def test_cruise5_multi_epoch_source_parity_matches_shipped_plot(tmp_path: Path) -> None:
    definition = load_cruise5_source_definition(_case(tmp_path))
    result = run_cruise5_source_compatibility(definition, end_time_s=1.0, sample_step_s=0.5)
    assert [sample.time_s for sample in result.samples] == pytest.approx((0.0, 0.5, 1.0))

    expected = (
        {
            "specific_force_velocity_mps2": (-1.49407, 0.0, -9.45654),
            "dynamic_pressure_pa": 37573.5,
            "mach": 0.772811,
            "longitude_deg": 45.0004,
            "latitude_deg": 35.0,
            "altitude_m": 1000.23,
            "speed_mps": 259.958,
            "heading_deg": 90.0005,
            "flight_path_deg": 0.998734,
            "propulsion_mode": 5,
            "center_of_gravity_in": 135.5,
            "thrust_n": 201.833,
            "lift_to_drag": 5.44385,
            "alpha_deg": 0.107658,
            "bank_deg": -3.5,
            "bank_command_deg": -118.524,
            "load_command_g": 3.0,
            "lateral_command_g": -1.0,
            "waypoint_ground_range_m": 21532.6,
        },
        {
            "specific_force_velocity_mps2": (-1.53704, -10.6912, -10.4595),
            "dynamic_pressure_pa": 37338.5,
            "mach": 0.770486,
            "longitude_deg": 45.0018,
            "latitude_deg": 35.0,
            "altitude_m": 1002.23,
            "speed_mps": 259.129,
            "heading_deg": 89.4780,
            "flight_path_deg": 0.863171,
            "propulsion_mode": 5,
            "center_of_gravity_in": 135.499,
            "thrust_n": 202.544,
            "lift_to_drag": 8.3974,
            "alpha_deg": 2.31986,
            "bank_deg": -48.2093,
            "bank_command_deg": -74.9261,
            "load_command_g": 3.0,
            "lateral_command_g": -1.0,
            "waypoint_ground_range_m": 21450.6,
        },
        {
            "specific_force_velocity_mps2": (-1.89753, -17.8502, -12.1478),
            "dynamic_pressure_pa": 37074.3,
            "mach": 0.76785,
            "longitude_deg": 45.0032,
            "latitude_deg": 35.0,
            "altitude_m": 1004.3,
            "speed_mps": 258.22,
            "heading_deg": 87.8250,
            "flight_path_deg": 0.999175,
            "propulsion_mode": 5,
            "center_of_gravity_in": 135.499,
            "thrust_n": 203.35,
            "lift_to_drag": 10.0769,
            "alpha_deg": 3.64413,
            "bank_deg": -55.4273,
            "bank_command_deg": -51.8929,
            "load_command_g": 3.0,
            "lateral_command_g": -1.0,
            "waypoint_ground_range_m": 21367.3,
        },
    )
    for sample, source in zip(result.samples, expected, strict=True):
        assert sample.specific_force_velocity_mps2 == pytest.approx(source["specific_force_velocity_mps2"], rel=3.0e-5, abs=3.0e-5)
        assert sample.dynamic_pressure_pa == pytest.approx(source["dynamic_pressure_pa"], rel=3.0e-5)
        assert sample.mach == pytest.approx(source["mach"], rel=3.0e-5)
        assert sample.longitude_deg == pytest.approx(source["longitude_deg"], abs=5.0e-5)
        assert sample.latitude_deg == pytest.approx(source["latitude_deg"], abs=5.0e-5)
        assert sample.altitude_m == pytest.approx(source["altitude_m"], abs=0.03)
        assert sample.speed_mps == pytest.approx(source["speed_mps"], abs=0.03)
        assert sample.heading_deg == pytest.approx(source["heading_deg"], abs=0.003)
        assert sample.flight_path_deg == pytest.approx(source["flight_path_deg"], abs=0.003)
        assert sample.propulsion_mode == source["propulsion_mode"]
        assert sample.center_of_gravity_in == pytest.approx(source["center_of_gravity_in"], abs=0.01)
        assert sample.thrust_n == pytest.approx(source["thrust_n"], abs=0.2)
        assert sample.lift_to_drag == pytest.approx(source["lift_to_drag"], rel=5.0e-5)
        assert sample.alpha_deg == pytest.approx(source["alpha_deg"], abs=2.0e-4)
        assert sample.bank_deg == pytest.approx(source["bank_deg"], abs=2.0e-4)
        assert sample.bank_command_deg == pytest.approx(source["bank_command_deg"], abs=0.03)
        assert sample.load_command_g == pytest.approx(source["load_command_g"], abs=2.0e-4)
        assert sample.lateral_command_g == pytest.approx(source["lateral_command_g"], abs=2.0e-4)
        assert sample.waypoint_ground_range_m == pytest.approx(source["waypoint_ground_range_m"], abs=1.0)
    ####


####
