"""Structural checks for the verification baseline registries."""

from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from taoryx.contracts import Angle, Latitude, Longitude
from taoryx.coordinates import geocentric_unit_vectors, geodetic_unit_vectors, tangent_plane_unit_vectors
from taoryx.equations import (
    CartesianVector3,
    ecfc_to_ecic_position,
    geodetic_body_axes_from_euler_angles,
    wind_unit_vectors_from_velocity,
)

ROOT = Path(__file__).parents[2]
VERIFICATION = ROOT / "verification"


def _load(name: str) -> dict[str, object]:
    with (VERIFICATION / name).open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    assert isinstance(document, dict)
    return document
####


def test_baseline_registries_have_unique_stable_ids() -> None:
    requirements = _load("spec/requirements.yaml")["requirements"]
    ambiguities = _load("spec/ambiguities.yaml")["ambiguities"]
    assert isinstance(requirements, list)
    assert isinstance(ambiguities, list)
    requirement_ids = [item["id"] for item in requirements]
    ambiguity_ids = [item["id"] for item in ambiguities]
    assert len(requirement_ids) == len(set(requirement_ids))
    assert len(ambiguity_ids) == len(set(ambiguity_ids))
    assert all(identifier.startswith("TAOS-REQ-") for identifier in requirement_ids)
    assert all(identifier.startswith("TAOS-AMB-") for identifier in ambiguity_ids)
    for ambiguity in ambiguities:
        assert ambiguity["category"] != "inventory_seed"
        assert ambiguity["source_form"]
        assert ambiguity["candidate_interpretations"]
        assert ambiguity["tests"]
####


def test_constant_registry_has_exact_decimal_seed_entries() -> None:
    constants = _load("spec/constants.yaml")["constants"]
    assert isinstance(constants, list)
    assert constants
    constant_ids = [item["id"] for item in constants]
    assert len(constant_ids) == len(set(constant_ids))
    assert any(identifier.startswith("earth.") for identifier in constant_ids)
    assert any(identifier.startswith("atmosphere.") for identifier in constant_ids)
    expected_exact_decimals = {
        "earth.wgs84.equatorial_radius": "20925646.3255",
        "earth.wgs84.flattening": "1/298.257223563",
        "earth.wgs84.gravitational_parameter": "1.40764438125e16",
        "earth.wgs84.rotation_rate": "7.292115e-5",
        "earth.wgs84.j2": "1.08262998905e-3",
        "earth.wgs72.equatorial_radius": "20925639.7638",
        "earth.wgs72.polar_radius": "20855480.7087",
        "earth.wgs72.flattening": "1/298.26",
        "earth.wgs72.rotation_rate": "7.292115147e-5",
        "earth.wgs72.gravitational_parameter": "1.40764544069e16",
        "earth.wgs72.j2": "1.08261579002e-3",
        "earth.wgs84_full.c20": "-1.082629e-3",
        "earth.wgs84_full.c22": "1.572805e-6",
        "earth.wgs84_full.c30": "2.532153e-6",
        "earth.wgs84_full.c31": "2.194673e-6",
        "earth.wgs84_full.c32": "3.096837e-7",
        "earth.wgs84_full.c33": "1.000789e-7",
        "earth.wgs84_full.c40": "1.610987e-6",
        "earth.wgs84_full.c41": "-5.080013e-7",
        "earth.wgs84_full.c42": "7.780961e-8",
        "earth.wgs84_full.c43": "5.926679e-8",
        "earth.wgs84_full.c44": "-3.948164e-9",
        "earth.wgs84_full.s22": "-9.023759e-7",
        "earth.wgs84_full.s31": "2.709571e-7",
        "earth.wgs84_full.s32": "-2.121201e-7",
        "earth.wgs84_full.s33": "1.973456e-7",
        "earth.wgs84_full.s41": "-4.498693e-7",
        "earth.wgs84_full.s42": "1.466394e-7",
        "earth.wgs84_full.s43": "-1.189998e-8",
        "earth.wgs84_full.s44": "6.540039e-9",
        "earth.gem_t1_full.c20": "-1.082625e-3",
        "earth.gem_t1_full.c22": "1.574322e-6",
        "earth.gem_t1_full.c30": "2.532618e-6",
        "earth.gem_t1_full.c31": "2.192402e-6",
        "earth.gem_t1_full.c32": "3.086210e-7",
        "earth.gem_t1_full.c33": "1.005372e-7",
        "earth.gem_t1_full.c40": "1.616190e-6",
        "earth.gem_t1_full.c41": "-5.060561e-7",
        "earth.gem_t1_full.c42": "7.759155e-8",
        "earth.gem_t1_full.c43": "5.922238e-8",
        "earth.gem_t1_full.c44": "-4.015116e-9",
        "earth.gem_t1_full.s22": "-9.035928e-7",
        "earth.gem_t1_full.s31": "2.695880e-7",
        "earth.gem_t1_full.s32": "-2.119137e-7",
        "earth.gem_t1_full.s33": "1.970571e-7",
        "earth.gem_t1_full.s41": "-4.507384e-7",
        "earth.gem_t1_full.s42": "1.484816e-7",
        "earth.gem_t1_full.s43": "-1.198933e-8",
        "earth.gem_t1_full.s44": "6.517407e-9",
        "earth.tsap84.j3": "-2.532153068e-6",
        "earth.tsap84.j4": "-1.610987610e-6",
        "earth.tsap72.j3": "-2.538810043e-6",
        "earth.tsap72.j4": "-1.655970000e-6",
        "atmosphere.us_standard_1976.reference_gravity": "9.80665",
        "atmosphere.us_standard_1976.molecular_weight": "28.9644",
        "atmosphere.standard.universal_gas_constant": "8314.32",
    }
    expected_ambiguity_form = "The WGS-84 gravity table prints J2 as 1.08262998905e-3, while the runtime gravity tests use a rounded 0.00108262668 value."
    for constant in constants:
        assert constant["exact_decimal"] == expected_exact_decimals[constant["id"]]
        assert constant["parsed_value"] is not None
        assert constant["model"]
        assert constant["unit"]
        assert constant["source"]["manual_section"]
        assert constant["source"]["manual_page"]
        if constant["id"].startswith("earth."):
            assert constant["source"]["manual_table"]
        assert constant["source"]["tex_file"]
        assert constant["source"]["tex_line_start"] <= constant["source"]["tex_line_end"]
        assert constant["source"]["tests"]

        tex_file = ROOT / constant["source"]["tex_file"]
        lines = tex_file.read_text(encoding="utf-8").splitlines()
        start = constant["source"]["tex_line_start"] - 1
        end = constant["source"]["tex_line_end"]
        snippet = "\n".join(lines[start:end])
        assert constant["symbol"] or constant["id"]
        rendered_scientific = _render_scientific_token(constant["exact_decimal"])
        assert any(token in snippet for token in (constant["symbol"], constant["exact_decimal"], rendered_scientific))

    ambiguities = _load("spec/ambiguities.yaml")["ambiguities"]
    ambiguity = next(item for item in ambiguities if item["id"] == "TAOS-AMB-0003")
    assert ambiguity["location"]["source_page"] == "4-55"
    assert ambiguity["location"]["table"] == "4-16"
    assert ambiguity["location"]["symbol"] == "j2"
    assert ambiguity["source_form"] == expected_ambiguity_form
    assert {item["id"] for item in ambiguity["candidate_interpretations"]} == {"source_table", "runtime_default"}
####


def test_ambiguity_records_are_source_located_and_explicit() -> None:
    ambiguities = _load("spec/ambiguities.yaml")["ambiguities"]
    assert isinstance(ambiguities, list)
    ambiguity_map = {item["id"]: item for item in ambiguities}

    for ambiguity in ambiguities:
        location = ambiguity["location"]
        assert location["source_page"]
        assert ambiguity["source_form"]
        assert ambiguity["candidate_interpretations"]
        assert ambiguity["decision"]["documentary_edition"]
        assert ambiguity["decision"]["status"]
        assert ambiguity["tests"]

    body_text = (ROOT / "manual/chapters/chapter02/01_07_body_fixed.tex").read_text(encoding="utf-8")
    body_text = " ".join(body_text.split())
    assert "This method is singular when the body points straight up or down" in body_text
    assert "yaw is undefined" in body_text
    assert "TAOS-AMB-0002" in ambiguity_map
    assert "documented_pole_convention" in {item["id"] for item in ambiguity_map["TAOS-AMB-0002"]["candidate_interpretations"]}

    gravity_text = (ROOT / "manual/chapters/chapter04/04_03_earth.tex").read_text(encoding="utf-8")
    gravity_text = " ".join(gravity_text.split())
    assert "J_2" in gravity_text
    assert "1.08262998905\\times10^{-3}" in gravity_text
    assert "TAOS-AMB-0003" in ambiguity_map
    assert ambiguity_map["TAOS-AMB-0003"]["location"]["source_page"] == "4-55"

    atmos_text = (ROOT / "manual/chapters/chapter02/03_01_atmosphere.tex").read_text(encoding="utf-8")
    atmos_text = " ".join(atmos_text.split())
    assert "This convention is inconsistent with the gravity model used for trajectory propagation" in atmos_text
    assert "TAOS-AMB-0004" in ambiguity_map
    assert ambiguity_map["TAOS-AMB-0004"]["candidate_interpretations"][0]["id"] == "atmosphere_model_latitude"
####


def test_requirement_links_point_to_repository_evidence() -> None:
    requirements = _load("spec/requirements.yaml")["requirements"]
    assert isinstance(requirements, list)
    for requirement in requirements:
        for test_path in requirement["tests"]:
            assert (ROOT / test_path).exists(), f"missing evidence path: {test_path}"
####


def test_traceability_references_declared_requirements() -> None:
    requirements = _load("spec/requirements.yaml")["requirements"]
    traceability = _load("traceability.yaml")["requirements"]
    assert isinstance(requirements, list)
    assert isinstance(traceability, dict)
    declared = {item["id"] for item in requirements}
    assert set(traceability) <= declared
####


def test_every_documented_coordinate_frame_has_a_verification_card() -> None:
    source = _load_from(ROOT / "metadata/coordinate_systems_chapter2.yaml")
    cards = _load("spec/frames.yaml")["frames"]
    assert isinstance(source["coordinate_systems"], list)
    assert isinstance(cards, list)
    source_ids = {item["id"] for item in source["coordinate_systems"]}
    card_ids = {item["source_registry_id"] for item in cards}
    assert card_ids == source_ids
    for card in cards:
        assert card["origin"]
        assert card["axes"]
        assert card["transform_convention"]
        assert card["anchor_cases"]
####


def test_frame_cards_are_supported_by_manual_source_text() -> None:
    frame_sources: dict[str, tuple[Path, tuple[str, ...]]] = {
        "ecfc": (
            ROOT / "manual/chapters/chapter02/01_01_ecfc.tex",
            (
                "$z$ axis points through the north pole",
            ),
        ),
        "ecic": (
            ROOT / "manual/chapters/chapter02/01_02_ecic.tex",
            (
                "rotation about the common $z$ axis",
                "earth's rotation rate",
            ),
        ),
        "local_geocentric_horizon": (
            ROOT / "manual/chapters/chapter02/01_03_local_geocentric.tex",
            (
                "The $x$ axis points toward the north pole",
                "the $y$ axis points east",
            ),
        ),
        "local_geodetic_horizon": (
            ROOT / "manual/chapters/chapter02/01_05_local_geodetic.tex",
            (
                "referenced to an ellipsoid rather than to a sphere",
                "The $x$ axis is in the plane formed by $\\vec{r}$ and $\\hat{z}_{\\oplus}$ and points north",
                "The $y$ axis is perpendicular to the $x$ axis and points east",
            ),
        ),
        "body_fixed": (
            ROOT / "manual/chapters/chapter02/01_07_body_fixed.tex",
            (
                "The first rotation is a yaw about $\\hat{z}_{gd}$",
                "The second rotation is a pitch about $\\hat{y}_1$",
                "The final rotation is a roll about $\\hat{x}_2$",
                "This method is singular when the body points straight up or down",
                "yaw is undefined",
            ),
        ),
        "wind": (
            ROOT / "manual/chapters/chapter02/01_09_wind.tex",
            (
                "The wind and body systems are related through one of three equivalent pairs of aerodynamic angles",
                "bank angle",
                "angle of attack",
                "sideslip",
                "When $\\hat x_w\\mathbin{\\cdot}\\hat x_b=0$",
                "TAOS uses the special convention",
                "Because the basic projection definitions are singular in this case",
                "windward meridian",
            ),
        ),
        "tangent_plane": (
            ROOT / "manual/chapters/chapter02/01_11_tangent_plane.tex",
            (
                "The $x$ direction is set by an azimuth",
                "measured clockwise from north",
                "The $z$ axis points upward",
                "right-handed system",
            ),
        ),
    }

    cards = _load("spec/frames.yaml")["frames"]
    assert isinstance(cards, list)
    card_map = {card["id"]: card for card in cards}
    for frame_id, (path, required_snippets) in frame_sources.items():
        assert frame_id in card_map
        text = path.read_text(encoding="utf-8")
        normalized_text = " ".join(text.split())
        for snippet in required_snippets:
            assert snippet in normalized_text, f"missing manual support for {frame_id}: {snippet}"

        singularities = card_map[frame_id]["singularities"]
        if frame_id == "body_fixed":
            assert "euler_branch_boundaries" in singularities
        elif frame_id == "wind":
            assert "zero_air_relative_speed" in singularities
            assert "total_angle_zero" in singularities
            assert "total_angle_pi" in singularities
        elif frame_id == "tangent_plane":
            assert "reference_point_pole" in singularities
        elif frame_id in {"local_geocentric_horizon", "local_geodetic_horizon", "geocentric", "geodetic"}:
            assert singularities
        elif frame_id == "ecic":
            assert singularities == []
####


def test_wind_branch_special_cases_are_explicit_in_the_manual() -> None:
    text = (ROOT / "manual/chapters/chapter02/01_09_wind.tex").read_text(encoding="utf-8")
    frame_tests = (ROOT / "tests/unit/test_frames_equations.py").read_text(encoding="utf-8")
    force_tests = (ROOT / "tests/unit/test_forces_equations.py").read_text(encoding="utf-8")
    normalized_text = " ".join(text.split())
    normalized_frame_tests = " ".join(frame_tests.split())
    normalized_force_tests = " ".join(force_tests.split())
    assert "For the special case $\\alpha=\\pm90^\\circ$, TAOS uses $\\beta=-\\phi_w$." in normalized_text
    assert "For the special case $\\alpha=\\pm90^\\circ$, TAOS uses $\\phi_w=-\\beta$." in normalized_text
    assert "For the special case $\\alpha_T=90^\\circ$, TAOS uses $\\beta=-\\phi_w$." in normalized_text
    assert "test_aerodynamic_angle_round_trip_handles_forward_and_90_degree_alpha_cases" in normalized_frame_tests
    assert "test_aerodynamic_angle_total_alpha_ninety_branch_respects_windward_meridian_convention" in normalized_frame_tests
    assert "ninety_recovered.windward_meridian_radians == pytest.approx(-math.radians(30.0))" in normalized_frame_tests
    assert "ninety_degree_body_axes" in normalized_frame_tests
    assert "test_body_windward_meridian_vector_and_axial_normal_force_follow_the_manual_formula" in normalized_force_tests
####


def test_frame_equations_are_anchored_by_manual_equation_labels() -> None:
    ecfc_text = (ROOT / "manual/chapters/chapter02/01_01_ecfc.tex").read_text(encoding="utf-8")
    ecic_text = (ROOT / "manual/chapters/chapter02/01_02_ecic.tex").read_text(encoding="utf-8")
    geocentric_text = (ROOT / "manual/chapters/chapter02/01_03_local_geocentric.tex").read_text(encoding="utf-8")
    body_text = (ROOT / "manual/chapters/chapter02/01_07_body_fixed.tex").read_text(encoding="utf-8")
    wind_text = (ROOT / "manual/chapters/chapter02/01_09_wind.tex").read_text(encoding="utf-8")
    tangent_text = (ROOT / "manual/chapters/chapter02/01_11_tangent_plane.tex").read_text(encoding="utf-8")
    normalized_ecfc = " ".join(ecfc_text.split())
    normalized_ecic = " ".join(ecic_text.split())
    normalized_geocentric = " ".join(geocentric_text.split())
    normalized_body = " ".join(body_text.split())
    normalized_wind = " ".join(wind_text.split())
    normalized_tangent = " ".join(tangent_text.split())

    assert "\\label{sec:ecfc}" in normalized_ecfc
    assert "\\label{sec:ecic}" in normalized_ecic
    assert "\\label{eq:ecic-rotation-angle}" in normalized_ecic
    assert "\\label{eq:ecic-position-components}" in normalized_ecic
    assert "\\label{eq:ecic-velocity-components}" in normalized_ecic
    assert "\\label{eq:ecic-acceleration-components}" in normalized_ecic
    assert "\\label{eq:local-geocentric-unit-vectors}" in normalized_geocentric
    assert "\\label{eq:body-euler-transformation}" in normalized_body
    assert "\\label{eq:body-x-from-geodetic-euler}" in normalized_body
    assert "\\label{eq:body-y-from-geodetic-euler}" in normalized_body
    assert "\\label{eq:body-z-from-geodetic-euler}" in normalized_body
    assert "\\label{eq:euler-angle-pole-convention}" in normalized_body
    assert "\\label{eq:wind-unit-vectors}" in normalized_wind
    assert "\\label{eq:wind-to-body-aero-transformation}" in normalized_wind
    assert "\\label{eq:body-x-from-aero-angles}" in normalized_wind
    assert "\\label{eq:body-y-from-aero-angles}" in normalized_wind
    assert "\\label{eq:body-z-from-aero-angles}" in normalized_wind
    assert "\\label{eq:projected-angle-of-attack}" in normalized_wind
    assert "\\label{eq:projected-sideslip}" in normalized_wind
    assert "\\label{eq:total-angle-of-attack}" in normalized_wind
    assert "\\label{eq:windward-meridian}" in normalized_wind
    assert "\\label{eq:projected-sideslip-at-90-alpha}" in normalized_wind
    assert "\\label{eq:tangent-plane-x-unit-vector}" in normalized_tangent
    assert "\\label{eq:tangent-plane-y-unit-vector}" in normalized_tangent
    assert "\\label{eq:tangent-plane-z-unit-vector}" in normalized_tangent
####


def test_atmosphere_equations_are_anchored_by_manual_equation_labels() -> None:
    atmos_text = (ROOT / "manual/chapters/chapter02/03_01_atmosphere.tex").read_text(encoding="utf-8")
    normalized_atmos = " ".join(atmos_text.split())

    assert "\\label{eq:atmos-aerostatic}" in normalized_atmos
    assert "\\label{eq:atmos-perfect-gas}" in normalized_atmos
    assert "\\label{eq:atmos-gravity-inverse-square}" in normalized_atmos
    assert "\\label{eq:lambert-sea-level-gravity}" in normalized_atmos
    assert "\\label{eq:effective-geopotential-earth-radius}" in normalized_atmos
    assert "\\label{eq:pressure-gradient-layer}" in normalized_atmos
    assert "\\label{eq:atmos-speed-of-sound}" in normalized_atmos
    assert "\\label{eq:atmos-kinematic-viscosity}" in normalized_atmos
####


def test_geodesy_equations_are_anchored_by_manual_equation_labels() -> None:
    geocentric_text = (ROOT / "manual/chapters/chapter02/01_04_geocentric.tex").read_text(encoding="utf-8")
    geodetic_text = (ROOT / "manual/chapters/chapter02/01_05_local_geodetic.tex").read_text(encoding="utf-8")
    geodetic_position_text = (ROOT / "manual/chapters/chapter02/01_06_geodetic.tex").read_text(encoding="utf-8")
    normalized_geocentric = " ".join(geocentric_text.split())
    normalized_geodetic = " ".join(geodetic_text.split())
    normalized_geodetic_position = " ".join(geodetic_position_text.split())

    assert "\\label{eq:geocentric-position-to-ecfc}" in normalized_geocentric
    assert "\\label{eq:longitude-from-ecfc}" in normalized_geocentric
    assert "\\label{eq:geocentric-latitude-from-ecfc}" in normalized_geocentric
    assert "\\label{eq:geocentric-velocity-x}" in normalized_geocentric
    assert "\\label{eq:geocentric-velocity-y}" in normalized_geocentric
    assert "\\label{eq:geocentric-velocity-z}" in normalized_geocentric
    assert "\\label{eq:local-geodetic-x-unit-vector}" in normalized_geodetic
    assert "\\label{eq:local-geodetic-y-unit-vector}" in normalized_geodetic
    assert "\\label{eq:local-geodetic-z-unit-vector}" in normalized_geodetic
    assert "\\label{eq:ellipsoid-parameter-relation}" in normalized_geodetic
    assert "\\label{eq:ellipsoid-surface}" in normalized_geodetic_position
    assert "\\label{eq:ellipsoid-plane-equation}" in normalized_geodetic_position
    assert "\\label{eq:ellipsoid-xs}" in normalized_geodetic_position
    assert "\\label{eq:geodetic-position-x}" in normalized_geodetic_position
    assert "\\label{eq:geodetic-position-y}" in normalized_geodetic_position
    assert "\\label{eq:geodetic-position-z}" in normalized_geodetic_position
    assert "\\label{eq:geodetic-velocity-x}" in normalized_geodetic_position
    assert "\\label{eq:geodetic-velocity-y}" in normalized_geodetic_position
    assert "\\label{eq:geodetic-velocity-z}" in normalized_geodetic_position
####


def test_gravity_equations_are_anchored_by_manual_equation_labels() -> None:
    gravity_text = (ROOT / "manual/chapters/chapter02/03_04_gravity_model.tex").read_text(encoding="utf-8")
    normalized_gravity = " ".join(gravity_text.split())

    assert "\\label{eq:gravity-potential-gradient}" in normalized_gravity
    assert "\\label{eq:geopotential-spherical-harmonics}" in normalized_gravity
    assert "\\label{eq:point-mass-geopotential}" in normalized_gravity
    assert "\\label{eq:zonal-geopotential-c-coefficients}" in normalized_gravity
    assert "\\label{eq:zonal-geopotential-j-coefficients}" in normalized_gravity
    assert "\\label{eq:associated-legendre-definition}" in normalized_gravity
    assert "\\label{eq:first-four-legendre-functions}" in normalized_gravity
    assert "\\label{eq:normalized-legendre-function}" in normalized_gravity
    assert "\\label{eq:normalized-gravity-coefficient}" in normalized_gravity
    assert "\\label{eq:gravity-geocentric-x-gradient}" in normalized_gravity
    assert "\\label{eq:gravity-geocentric-y-gradient}" in normalized_gravity
    assert "\\label{eq:gravity-geocentric-z-gradient}" in normalized_gravity
    assert "\\label{eq:gravity-full-geocentric-x}" in normalized_gravity
    assert "\\label{eq:gravity-full-geocentric-y}" in normalized_gravity
    assert "\\label{eq:gravity-full-geocentric-z}" in normalized_gravity
    assert "\\label{eq:gravity-j2-geocentric-x}" in normalized_gravity
    assert "\\label{eq:gravity-j2-geocentric-y}" in normalized_gravity
    assert "\\label{eq:gravity-j2-geocentric-z}" in normalized_gravity
####


def test_optimization_equations_are_anchored_by_manual_equation_labels() -> None:
    search_text = (ROOT / "manual/chapters/chapter02/06_02_search_methods.tex").read_text(encoding="utf-8")
    optimization_text = (ROOT / "manual/chapters/chapter02/06_03_optimization.tex").read_text(encoding="utf-8")
    control_text = (ROOT / "manual/chapters/chapter02/05_01_control_variable_solution.tex").read_text(encoding="utf-8")
    normalized_search = " ".join(search_text.split())
    normalized_optimization = " ".join(optimization_text.split())
    normalized_control = " ".join(control_text.split())

    assert "\\label{eq:newton-linearization}" in normalized_search
    assert "\\label{eq:newton-update-search}" in normalized_search
    assert "\\label{eq:secant-root-estimate}" in normalized_search
    assert "\\label{eq:parabolic-search-polynomial}" in normalized_search
    assert "\\label{eq:golden-section-x1}" in normalized_search
    assert "\\label{eq:golden-section-x2}" in normalized_search
    assert "\\label{eq:parabolic-minimum}" in normalized_search
    assert "\\label{eq:general-nonlinear-program}" in normalized_optimization
    assert "\\label{eq:active-set-equality-problem}" in normalized_optimization
    assert "\\label{eq:optimization-quadratic-approximation}" in normalized_optimization
    assert "\\label{eq:optimization-constraint-linearization}" in normalized_optimization
    assert "\\label{eq:optimization-quadratic-subproblem}" in normalized_optimization
    assert "\\label{eq:maximum-altitude-integral}" in normalized_optimization
    assert "\\label{eq:maximum-altitude-constraint}" in normalized_optimization
    assert "\\label{eq:parabolic-guidance-state}" in normalized_control
    assert "\\label{eq:parabolic-guidance-rate}" in normalized_control
    assert "\\label{eq:parabolic-guidance-a}" in normalized_control
    assert "\\label{eq:parabolic-guidance-b}" in normalized_control
    assert "\\label{eq:cubic-guidance-state}" in normalized_control
    assert "\\label{eq:cubic-guidance-acceleration}" in normalized_control
    assert "\\label{eq:guidance-newton-update}" in normalized_control
    assert "\\label{eq:guidance-jacobian}" in normalized_control
    assert "\\label{eq:guidance-gaussian-system}" in normalized_control
####


def test_optimization_and_guidance_contract_is_source_located() -> None:
    search_text = (ROOT / "manual/chapters/chapter02/06_02_search_methods.tex").read_text(encoding="utf-8")
    optimization_text = (ROOT / "manual/chapters/chapter02/06_03_optimization.tex").read_text(encoding="utf-8")
    control_text = (ROOT / "manual/chapters/chapter02/05_01_control_variable_solution.tex").read_text(encoding="utf-8")
    search_tests = (ROOT / "tests/unit/test_search_optimization_equations.py").read_text(encoding="utf-8")
    optimization_tests = (ROOT / "tests/unit/test_optimization.py").read_text(encoding="utf-8")
    lowering_tests = (ROOT / "tests/unit/test_runtime_lowering.py").read_text(encoding="utf-8")
    normalized_search = " ".join(search_text.split())
    normalized_optimization = " ".join(optimization_text.split())
    normalized_control = " ".join(control_text.split())
    normalized_search_tests = " ".join(search_tests.split())
    normalized_optimization_tests = " ".join(optimization_tests.split())
    normalized_lowering_tests = " ".join(lowering_tests.split())

    assert "The Newton--Raphson method starts with an estimate" in normalized_search
    assert "The secant method also assumes that $f(x)$ is linear over the current interval." in normalized_search
    assert "The parabolic method approximates the function by" in normalized_search
    assert "The golden-section method minimizes a function without derivatives." in normalized_search
    assert "Parabolic minimization uses the same startup and coefficient formulas as the parabolic root search" in normalized_search
    assert "The general nonlinear programming problem varies $n$ parameters" in normalized_optimization
    assert "TAOS uses the Han--Powell method" in normalized_optimization
    assert "Initial estimates for the parameters are required." in normalized_optimization
    assert "The user supplies the objective function and constraints through the problem file." in normalized_optimization
    assert "TAOS therefore permits normalization by reference values" in normalized_optimization
    assert "TAOS separates guidance rules that directly specify control variables from those that do not." in normalized_control
    assert "The indirect solution is iterative." in normalized_control
    assert "The interval $\\Delta t_{\\mathrm{guid}}$ acts as a time constant." in normalized_control
    assert "The function \\nolinkurl{parab_guidance_correction} in the \\texttt{derivs} module" in normalized_control
    assert "curve with \\nolinkurl{cubic_guidance_correction}, as illustrated in" in normalized_control
    assert "The multidimensional Newton--Raphson iteration is" in normalized_control
    assert "test_newton_and_secant_search_helpers_follow_the_manual_formulas" in normalized_search_tests
    assert "test_parabolic_search_helpers_follow_the_manual_formulas" in normalized_search_tests
    assert "test_golden_section_helpers_follow_the_manual_formulas" in normalized_search_tests
    assert "test_optimization_helpers_follow_the_manual_formulas" in normalized_search_tests
    assert "test_build_optimization_problem_preserves_constraint_kinds" in normalized_optimization_tests
    assert "test_projected_rqp_converges_on_scalar_problem" in normalized_optimization_tests
    assert "test_path_violation_integral_and_control_redistribution" in normalized_optimization_tests
    assert "test_lowering_expands_surveys_and_prepares_tables" in normalized_lowering_tests
####


def test_search_and_optimize_diagnostics_are_source_located() -> None:
    search_text = (ROOT / "manual/chapters/chapter04/04_09_search.tex").read_text(encoding="utf-8")
    optimize_text = (ROOT / "manual/chapters/chapter04/04_06_optimize.tex").read_text(encoding="utf-8")
    parser_text = (ROOT / "tests/parser/test_problem_parser.py").read_text(encoding="utf-8")
    framing_text = (ROOT / "tests/parser/test_problem_framing.py").read_text(encoding="utf-8")
    normalized_search = " ".join(search_text.split())
    normalized_optimize = " ".join(optimize_text.split())
    normalized_parser = " ".join(parser_text.split())
    normalized_framing = " ".join(framing_text.split())

    assert "Every \\texttt{srch-n} placeholder requires a corresponding" in normalized_search
    assert "A segment and trajectory number are required for the first output variable" in normalized_search
    assert "The method requires an initial estimate" in normalized_search
    assert "The loop letter follows the \\problemblock{*optimize} keyword" in normalized_optimize
    assert "The segment and trajectory numbers are required" in normalized_optimize
    assert "The first items in the block are the search number" in normalized_search

    for code in (
        "unknown-search-reference",
        "missing-search-control",
        "unsupported-search-control",
        "invalid-search-endpoint",
        "duplicate-search-control",
        "duplicate-search-id",
        "unknown-optimize-loop",
        "unknown-optimize-parameter",
        "missing-optimize-parameters",
        "orphan-optimize-parameter-bound",
        "nonsequential-optimize-parameters",
        "duplicate-optimize-loop",
        "duplicate-optimize-control",
        "invalid-optimize-header",
        "conflicting-endpoint-trajectory",
    ):
        assert code in normalized_parser or code in normalized_framing

    assert "diagnostic.location.line == 6" in normalized_parser or "location.line == 6" in normalized_parser
    assert "all(item.location.line == 2 for item in result.diagnostics" in normalized_parser
    assert "any(record.code == \"invalid-search-endpoint\"" in normalized_framing
    assert "invalid-optimize-header" in normalized_framing
    assert "conflicting-endpoint-trajectory" in normalized_framing
####


def test_velocity_frame_singularities_are_source_located() -> None:
    velocity_text = (ROOT / "manual/chapters/chapter02/01_08_velocity.tex").read_text(encoding="utf-8")
    rates_text = (ROOT / "manual/chapters/chapter02/02_05_flight_path_rates.tex").read_text(encoding="utf-8")
    coordinates_text = (ROOT / "tests/unit/test_coordinates.py").read_text(encoding="utf-8")
    normalized_velocity = " ".join(velocity_text.split())
    normalized_rates = " ".join(rates_text.split())
    normalized_coordinates = " ".join(coordinates_text.split())

    assert "The velocity-system $y$ axis is perpendicular to the plane formed by its $x$ axis" in normalized_velocity
    assert "The function \\texttt{wind\\_unit\\_vectors} computes the air-relative velocity-system unit vectors" in normalized_velocity
    assert "These relationships are used in the derivation of the flight-path-angle rates" in normalized_velocity
    assert "The rates are derived from the angular velocity between the velocity coordinate system" in normalized_rates
    assert "test_velocity_frames_reject_zero_or_vertical_relative_velocity" in normalized_coordinates
    assert "zero velocity" in normalized_coordinates
    assert "vertical velocity" in normalized_coordinates
####


def test_coordinate_diagnostics_are_source_located() -> None:
    geocentric_text = (ROOT / "manual/chapters/chapter02/01_04_geocentric.tex").read_text(encoding="utf-8")
    geodetic_text = (ROOT / "manual/chapters/chapter02/01_05_local_geodetic.tex").read_text(encoding="utf-8")
    velocity_text = (ROOT / "manual/chapters/chapter02/01_08_velocity.tex").read_text(encoding="utf-8")
    coordinates_text = (ROOT / "tests/unit/test_coordinates.py").read_text(encoding="utf-8")
    normalized_geocentric = " ".join(geocentric_text.split())
    normalized_geodetic = " ".join(geodetic_text.split())
    normalized_velocity = " ".join(velocity_text.split())
    normalized_coordinates = " ".join(coordinates_text.split())

    assert "The reverse relationships are evaluated by \\texttt{p\\_ecfc\\_to\\_geocentric}" in normalized_geocentric
    assert "These quantities are calculated in \\texttt{v\\_ecfc\\_to\\_geocentric}." in normalized_geocentric
    assert "The latitude required by these equations is obtained with the iterative method" in normalized_geodetic
    assert "The function \\texttt{wind\\_unit\\_vectors} computes the air-relative velocity-system unit vectors" in normalized_velocity
    assert "test_ecfc_position_inverse_rejects_wrong_frame_or_dimension" in normalized_coordinates
    assert "test_geocentric_velocity_conversion_rejects_invalid_inputs" in normalized_coordinates
    assert "test_geodetic_position_inverse_rejects_origin_and_nonconvergence" in normalized_coordinates
    assert "test_velocity_frames_reject_zero_or_vertical_relative_velocity" in normalized_coordinates
    assert "position must be expressed" in normalized_coordinates
    assert "velocity must be expressed" in normalized_coordinates
    assert "origin" in normalized_coordinates
    assert "zero velocity" in normalized_coordinates
    assert "vertical velocity" in normalized_coordinates
####


def test_coordinate_basis_conventions_are_source_located() -> None:
    geocentric_text = (ROOT / "manual/chapters/chapter02/01_03_local_geocentric.tex").read_text(encoding="utf-8")
    geodetic_text = (ROOT / "manual/chapters/chapter02/01_05_local_geodetic.tex").read_text(encoding="utf-8")
    coordinates_text = (ROOT / "tests/unit/test_coordinates.py").read_text(encoding="utf-8")
    normalized_geocentric = " ".join(geocentric_text.split())
    normalized_geodetic = " ".join(geodetic_text.split())
    normalized_coordinates = " ".join(coordinates_text.split())

    assert "The unit vectors, expressed in ECFC coordinates, are computed by \\texttt{geoc\\_unit\\_vectors}" in normalized_geocentric
    assert "\\label{eq:local-geocentric-unit-vectors}" in normalized_geocentric
    assert "The geodetic unit vectors are" in normalized_geodetic
    assert "\\label{eq:local-geodetic-x-unit-vector}" in normalized_geodetic
    assert "\\label{eq:local-geodetic-y-unit-vector}" in normalized_geodetic
    assert "\\label{eq:local-geodetic-z-unit-vector}" in normalized_geodetic
    assert "test_geocentric_unit_vectors_return_framed_right_handed_basis" in normalized_coordinates
    assert "test_geocentric_unit_vectors_handle_equator_and_pole_conventions" in normalized_coordinates
    assert "test_geodetic_unit_vectors_are_framed_and_orthonormal" in normalized_coordinates
    assert "test_geocentric_position_conversion_preserves_order_units_and_round_trips" in normalized_coordinates
    assert "test_geocentric_velocity_conversion_handles_cardinal_and_vertical_conventions" in normalized_coordinates
####


def test_geodetic_position_and_velocity_contracts_are_source_located() -> None:
    geodetic_text = (ROOT / "manual/chapters/chapter02/01_06_geodetic.tex").read_text(encoding="utf-8")
    coordinates_text = (ROOT / "tests/unit/test_coordinates.py").read_text(encoding="utf-8")
    normalized_geodetic = " ".join(geodetic_text.split())
    normalized_coordinates = " ".join(coordinates_text.split())

    assert "\\label{eq:ellipsoid-surface}" in normalized_geodetic
    assert "\\label{eq:ellipsoid-plane-equation}" in normalized_geodetic
    assert "\\label{eq:ellipsoid-xs}" in normalized_geodetic
    assert "\\label{eq:geodetic-position-x}" in normalized_geodetic
    assert "\\label{eq:geodetic-position-y}" in normalized_geodetic
    assert "\\label{eq:geodetic-position-z}" in normalized_geodetic
    assert "\\label{eq:geodetic-iteration-zd}" in normalized_geodetic
    assert "\\label{eq:geodetic-iteration-latitude}" in normalized_geodetic
    assert "\\label{eq:geodetic-iteration-normal}" in normalized_geodetic
    assert "\\label{eq:geodetic-iteration-intercept}" in normalized_geodetic
    assert "\\label{eq:geodetic-velocity-x}" in normalized_geodetic
    assert "\\label{eq:geodetic-velocity-y}" in normalized_geodetic
    assert "\\label{eq:geodetic-velocity-z}" in normalized_geodetic
    assert "test_geodetic_position_conversion_round_trips_with_explicit_units" in normalized_coordinates
    assert "test_geodetic_velocity_conversion_round_trips_and_uses_geodetic_frame" in normalized_coordinates
    assert "test_geodetic_position_inverse_rejects_origin_and_nonconvergence" in normalized_coordinates
####


def test_gravity_wrapper_contract_is_source_located() -> None:
    gravity_text = (ROOT / "manual/chapters/chapter02/03_04_gravity_model.tex").read_text(encoding="utf-8")
    gravity_equation_test = (ROOT / "tests/unit/test_gravity_equations.py").read_text(encoding="utf-8")
    gravity_test = (ROOT / "tests/unit/test_gravity.py").read_text(encoding="utf-8")
    normalized_gravity = " ".join(gravity_text.split())
    normalized_equation_test = " ".join(gravity_equation_test.split())
    normalized_test = " ".join(gravity_test.split())

    assert "\\label{eq:gravity-potential-gradient}" in normalized_gravity
    assert "\\label{eq:geopotential-spherical-harmonics}" in normalized_gravity
    assert "\\label{eq:gravity-j2-geocentric-x}" in normalized_gravity
    assert "\\label{eq:gravity-j2-geocentric-y}" in normalized_gravity
    assert "\\label{eq:gravity-j2-geocentric-z}" in normalized_gravity
    assert "test_legendre_and_normalization_helpers_follow_the_manual_formulas" in normalized_equation_test
    assert "test_geopotential_and_acceleration_helpers_follow_the_manual_formulas" in normalized_equation_test
    assert "test_gravity_wrappers_preserve_equation_registry_values_and_frames" in normalized_test
    assert "gravity_acceleration_full" in normalized_test
    assert "gravity_acceleration_j2" in normalized_test
    assert "associated_legendre" in normalized_test
####


def test_gravity_helper_contract_is_source_located() -> None:
    gravity_text = (ROOT / "manual/chapters/chapter02/03_04_gravity_model.tex").read_text(encoding="utf-8")
    gravity_equation_test = (ROOT / "tests/unit/test_gravity_equations.py").read_text(encoding="utf-8")
    normalized_gravity = " ".join(gravity_text.split())
    normalized_equation_test = " ".join(gravity_equation_test.split())

    assert "\\label{eq:associated-legendre-definition}" in normalized_gravity
    assert "\\label{eq:first-four-legendre-functions}" in normalized_gravity
    assert "\\label{eq:normalized-legendre-function}" in normalized_gravity
    assert "\\label{eq:normalized-gravity-coefficient}" in normalized_gravity
    assert "\\label{eq:gravity-geocentric-x-gradient}" in normalized_gravity
    assert "\\label{eq:gravity-geocentric-y-gradient}" in normalized_gravity
    assert "\\label{eq:gravity-geocentric-z-gradient}" in normalized_gravity
    assert "\\label{eq:gravity-full-geocentric-x}" in normalized_gravity
    assert "\\label{eq:gravity-full-geocentric-y}" in normalized_gravity
    assert "\\label{eq:gravity-full-geocentric-z}" in normalized_gravity
    assert "test_legendre_and_normalization_helpers_follow_the_manual_formulas" in normalized_equation_test
    assert "test_geopotential_and_acceleration_helpers_follow_the_manual_formulas" in normalized_equation_test
####


def test_gravity_model_registry_preserves_family_provenance() -> None:
    constants = _load("spec/constants.yaml")
    models = constants["models"]
    assert isinstance(models, list)
    model_map = {item["id"]: item for item in models}
    assert set(model_map) == {
        "earth.spherical",
        "earth.wgs72_j2",
        "earth.wgs84_j2",
        "earth.tsap72_degree4_zonal",
        "earth.tsap84_degree4_zonal",
        "earth.wgs84_degree4_full",
        "earth.gem_t1_degree4_full",
    }
    assert model_map["earth.spherical"]["source_name"] == "spherical"
    assert model_map["earth.spherical"]["normalization"] == "not_applicable"
    assert model_map["earth.wgs72_j2"]["source_name"] == "wgs-72"
    assert model_map["earth.wgs84_j2"]["source_name"] == "wgs-84"
    assert model_map["earth.tsap72_degree4_zonal"]["source_name"] == "tsap-72"
    assert model_map["earth.tsap84_degree4_zonal"]["source_name"] == "tsap-84"
    assert model_map["earth.wgs84_degree4_full"]["source_name"] == "wgs-84-full"
    assert model_map["earth.gem_t1_degree4_full"]["source_name"] == "gem-t1-full"
    assert all(item["normalization"] == "source_specific" for key, item in model_map.items() if key != "earth.spherical")
    assert all(item["status"] for item in models)
####


def test_atmosphere_constant_registry_preserves_exact_decimals_and_units() -> None:
    constants = _load("spec/constants.yaml")["constants"]
    constant_map = {item["id"]: item for item in constants if item["id"].startswith("atmosphere.")}
    assert constant_map["atmosphere.us_standard_1976.reference_gravity"]["exact_decimal"] == "9.80665"
    assert constant_map["atmosphere.us_standard_1976.reference_gravity"]["unit"] == "meter/second^2"
    assert constant_map["atmosphere.us_standard_1976.reference_gravity"]["source"]["manual_page"] == "2-52"
    assert constant_map["atmosphere.us_standard_1976.molecular_weight"]["exact_decimal"] == "28.9644"
    assert constant_map["atmosphere.us_standard_1976.molecular_weight"]["unit"] == "kilogram/kilomole"
    assert constant_map["atmosphere.standard.universal_gas_constant"]["exact_decimal"] == "8314.32"
    assert constant_map["atmosphere.standard.universal_gas_constant"]["unit"] == "joule/kilomole/kelvin"
    assert constant_map["atmosphere.standard.universal_gas_constant"]["source"]["manual_page"] == "2-52"
####


def test_wind_block_contract_is_source_located() -> None:
    wind_text = (ROOT / "manual/chapters/chapter04/04_14_wind.tex").read_text(encoding="utf-8")
    output_tests = (ROOT / "tests/parser/test_output_blocks.py").read_text(encoding="utf-8")
    normalized_wind = " ".join(wind_text.split())
    normalized_tests = " ".join(output_tests.split())

    assert "If no block is supplied, all wind velocities are zero." in normalized_wind
    assert "Wind heading is the direction \\emph{from which} the wind blows" in normalized_wind
    assert "The speed/heading set and the east/north component set cannot be mixed." in normalized_wind
    assert "test_wind_rejects_mixed_component_sets" in normalized_tests
    assert "mixed-wind-components" in normalized_tests
    assert "test_wind_component_form_is_typed_without_guessing_mixed_input" in normalized_tests
####


def test_print_block_contract_is_source_located() -> None:
    print_text = (ROOT / "manual/chapters/chapter04/04_07_print.tex").read_text(encoding="utf-8")
    output_tests = (ROOT / "tests/parser/test_output_blocks.py").read_text(encoding="utf-8")
    normalized_print = " ".join(print_text.split())
    normalized_tests = " ".join(output_tests.split())

    assert "The \\problemblock{*print} data block can appear either within a trajectory or within a problem" in normalized_print
    assert "trajectory numbers enclosed in brackets are required" in normalized_print
    assert "A table of trajectory data is written, with a maximum of 10 output variables" in normalized_print
    assert "test_problem_output_requires_two_subscripts_for_related_variables" in normalized_tests
    assert "test_problem_file_after_completed_trajectory_is_problem_scoped" in normalized_tests
    assert "test_dual_scope_define_and_print_after_segments_report_ambiguity" in normalized_tests
    assert "test_related_output_variables_reject_extra_subscripts" in normalized_tests
    assert "test_trajectory_output_requires_one_subscript_for_related_variables" in normalized_tests
    assert "test_print_block_enforces_manual_ten_variable_limit_across_continuations" in normalized_tests
####


def test_units_fmt_block_contract_is_source_located() -> None:
    units_text = (ROOT / "manual/chapters/chapter04/04_13_units_fmt.tex").read_text(encoding="utf-8")
    framing_tests = (ROOT / "tests/parser/test_problem_framing.py").read_text(encoding="utf-8")
    normalized_units = " ".join(units_text.split())
    normalized_tests = " ".join(framing_tests.split())

    assert "\\label{tab:allowable-units}" in normalized_units
    assert "Units must be dimensionally consistent" in normalized_units
    assert "When both are changed, the unit must appear first and the format second." in normalized_units
    assert "test_units_format_block_parses_inline_and_continuation_settings" in normalized_tests
    assert "test_units_format_block_recovers_malformed_setting" in normalized_tests
    assert "test_units_format_rejects_units_outside_the_manual_table" in normalized_tests
    assert "test_units_format_rejects_dimensionally_incompatible_units" in normalized_tests
    assert "test_units_format_rejects_malformed_variable_names_with_recovery" in normalized_tests
####


def test_survey_block_contract_is_source_located() -> None:
    survey_text = (ROOT / "manual/chapters/chapter04/04_11_survey.tex").read_text(encoding="utf-8")
    framing_tests = (ROOT / "tests/parser/test_problem_framing.py").read_text(encoding="utf-8")
    parser_tests = (ROOT / "tests/parser/test_problem_parser.py").read_text(encoding="utf-8")
    normalized_survey = " ".join(survey_text.split())
    normalized_framing = " ".join(framing_tests.split())
    normalized_parser = " ".join(parser_tests.split())

    assert "The purpose of a survey is to change one or more input parameters" in normalized_survey
    assert "Survey numbers relate placeholders and blocks" in normalized_survey
    assert "A block begins with the \\problemblock{*survey} keyword and the survey number." in normalized_survey
    assert "Survey values can be assigned in two ways" in normalized_survey
    assert "The second method enters the desired values directly" in normalized_survey
    assert "test_survey_block_parses_incremental_and_explicit_values" in normalized_framing
    assert "test_survey_requires_complete_incremental_or_explicit_values" in normalized_framing
    assert "test_duplicate_incremental_survey_setting_is_ambiguous_but_preserved" in normalized_framing
    assert "test_duplicate_explicit_survey_values_are_ambiguous_but_preserved" in normalized_framing
    assert "test_duplicate_survey_ids_do_not_resolve_placeholders" in normalized_framing
    assert "test_survey_and_search_numbers_must_begin_at_one" in normalized_framing
    assert "test_survey_name_cannot_duplicate_output_variable_name" in normalized_framing
    assert "test_survey_placeholder_must_match_declared_survey" in normalized_parser
    assert "test_declared_survey_placeholder_is_valid" in normalized_parser
####


def test_title_block_contract_is_source_located() -> None:
    title_text = (ROOT / "manual/chapters/chapter04/04_12_title.tex").read_text(encoding="utf-8")
    framing_tests = (ROOT / "tests/parser/test_problem_framing.py").read_text(encoding="utf-8")
    lexical_tests = (ROOT / "tests/parser/test_lexical.py").read_text(encoding="utf-8")
    normalized_title = " ".join(title_text.split())
    normalized_framing = " ".join(framing_tests.split())
    normalized_lexical = " ".join(lexical_tests.split())

    assert "A problem title consisting of one or more lines of text is optional." in normalized_title
    assert "The title begins with the first nonblank character after the \\problemblock{*title} keyword" in normalized_title
    assert "A title can therefore contain arbitrary text except a valid data-block keyword." in normalized_title
    assert "test_title_continuation_lines_remain_title_text" in normalized_framing
    assert "test_title_preserves_delimiters_but_treats_hash_as_comment_boundary" in normalized_framing
    assert "test_title_tail_preserves_case_and_delimiters" in normalized_lexical
####


def test_summarize_block_contract_is_source_located() -> None:
    summarize_text = (ROOT / "manual/chapters/chapter04/04_10_summarize.tex").read_text(encoding="utf-8")
    summarize_tests = (ROOT / "tests/parser/test_summarize_parser.py").read_text(encoding="utf-8")
    normalized_summarize = " ".join(summarize_text.split())
    normalized_tests = " ".join(summarize_tests.split())

    assert "Summary variables are used with surveys to perform tradeoff and sensitivity studies." in normalized_summarize
    assert "The summary starts at zero." in normalized_summarize
    assert "The operations from \\texttt{add} through \\texttt{iexp} require an additional value" in normalized_summarize
    assert "The operations from \\texttt{abs} through \\texttt{atan} act only on the current summary" in normalized_summarize
    assert "test_summarize_accepts_every_documented_math_operation" in normalized_tests
    assert "test_summarize_operations_preserve_documented_operands" in normalized_tests
    assert "test_summarize_reports_multiple_bad_operations_and_recovers" in normalized_tests
    assert "test_summarize_rejects_missing_name" in normalized_tests
    assert "test_summarize_rejects_undocumented_nested_function_operands" in normalized_tests
    assert "test_summarize_special_functions_require_variable_operands_and_one_trajectory_form" in normalized_tests
####


def test_summarize_special_function_contract_is_source_located() -> None:
    summarize_text = (ROOT / "manual/chapters/chapter04/04_10_summarize.tex").read_text(encoding="utf-8")
    summarize_tests = (ROOT / "tests/parser/test_summarize_parser.py").read_text(encoding="utf-8")
    normalized_summarize = " ".join(summarize_text.split())
    normalized_tests = " ".join(summarize_tests.split())

    assert "Summary variables are used with surveys to perform tradeoff and sensitivity studies." in normalized_summarize
    assert "The summary starts at zero." in normalized_summarize
    assert "The \\texttt{max} function searches the complete trajectory for the maximum value of a variable." in normalized_summarize
    assert "The corresponding \\texttt{min} and \\texttt{first} functions are also available." in normalized_summarize
    assert "Segment numbers are required because none of the special functions is used." in normalized_summarize
    assert "test_summarize_accepts_every_documented_math_operation" in normalized_tests
    assert "test_summarize_operations_preserve_documented_operands" in normalized_tests
    assert "test_summarize_reports_multiple_bad_operations_and_recovers" in normalized_tests
    assert "test_summarize_rejects_missing_name" in normalized_tests
    assert "test_summarize_rejects_undocumented_nested_function_operands" in normalized_tests
    assert "test_summarize_special_functions_require_variable_operands_and_one_trajectory_form" in normalized_tests
####


def test_file_block_contract_is_source_located() -> None:
    file_text = (ROOT / "manual/chapters/chapter04/04_05_file.tex").read_text(encoding="utf-8")
    output_tests = (ROOT / "tests/parser/test_output_blocks.py").read_text(encoding="utf-8")
    normalized_file = " ".join(file_text.split())
    normalized_tests = " ".join(output_tests.split())

    assert "The \\problemblock{*file} data block is used to create files containing columns of trajectory information." in normalized_file
    assert "the variables listed for \\problemblock{*file} data blocks outside of the trajectory definitions must contain trajectory numbers enclosed in brackets." in normalized_file
    assert "Any standard output variable or user-defined output variable can be written to the file." in normalized_file
    assert "test_output_blocks_collect_continuation_variables" in normalized_tests
    assert "test_problem_output_requires_two_subscripts_for_related_variables" in normalized_tests
    assert "test_problem_file_after_completed_trajectory_is_problem_scoped" in normalized_tests
    assert "test_related_output_variables_reject_extra_subscripts" in normalized_tests
    assert "test_trajectory_output_requires_one_subscript_for_related_variables" in normalized_tests
    assert "test_missing_output_filename_and_variables_are_file_diagnostics" in normalized_tests
####


def test_output_block_diagnostics_are_source_located() -> None:
    output_tests = (ROOT / "tests/parser/test_output_blocks.py").read_text(encoding="utf-8")
    normalized_tests = " ".join(output_tests.split())

    assert "invalid-related-output-subscript" in normalized_tests
    assert "missing-output-filename" in normalized_tests
    assert "missing-output-variables" in normalized_tests
    assert "too-many-print-variables" in normalized_tests
    assert "mixed-wind-components" in normalized_tests
    assert "invalid-egs-summary" in normalized_tests
    assert "test_problem_output_requires_two_subscripts_for_related_variables" in normalized_tests
    assert "test_problem_file_after_completed_trajectory_is_problem_scoped" in normalized_tests
    assert "test_related_output_variables_reject_extra_subscripts" in normalized_tests
    assert "test_trajectory_output_requires_one_subscript_for_related_variables" in normalized_tests
    assert "test_missing_output_filename_and_variables_are_file_diagnostics" in normalized_tests
    assert "test_print_block_enforces_manual_ten_variable_limit_across_continuations" in normalized_tests
####


def test_aerodynamic_force_contract_is_source_located() -> None:
    aero_text = (ROOT / "manual/chapters/chapter02/03_02_aerodynamic_forces.tex").read_text(encoding="utf-8")
    force_tests = (ROOT / "tests/unit/test_forces_equations.py").read_text(encoding="utf-8")
    frame_tests = (ROOT / "tests/unit/test_frames_equations.py").read_text(encoding="utf-8")
    normalized_aero = " ".join(aero_text.split())
    normalized_force_tests = " ".join(force_tests.split())
    normalized_frame_tests = " ".join(frame_tests.split())

    assert "TAOS accepts three coefficient systems: axial and normal coefficients" in normalized_aero
    assert "The axial and normal coefficients then produce" in normalized_aero
    assert "Lift, drag, and side-force coefficients are commonly used for aircraft." in normalized_aero
    assert "Coefficients $C_X$, $C_Y$, and $C_Z$" in normalized_aero
    assert "The lift, drag, and side-force outputs are always calculated." in normalized_aero
    assert "test_body_windward_meridian_vector_and_axial_normal_force_follow_the_manual_formula" in normalized_force_tests
    assert "test_lift_drag_side_body_axis_and_propulsive_force_formulas_follow_the_manual" in normalized_force_tests
    assert "test_specific_load_helpers_follow_the_manual_definitions" in normalized_force_tests
    assert "test_wind_and_meridian_singularity_diagnostics_are_explicit" in normalized_force_tests
    assert "test_aerodynamic_angle_round_trip_handles_forward_and_90_degree_alpha_cases" in normalized_frame_tests
####


def test_radar_block_contract_is_source_located() -> None:
    radar_text = (ROOT / "manual/chapters/chapter04/04_08_radar.tex").read_text(encoding="utf-8")
    framing_tests = (ROOT / "tests/parser/test_problem_framing.py").read_text(encoding="utf-8")
    snippet_tests = (ROOT / "tests/parser/test_manual_snippet_corpus.py").read_text(encoding="utf-8")
    normalized_radar = " ".join(radar_text.split())
    normalized_framing = " ".join(framing_tests.split())
    normalized_snippet = " ".join(snippet_tests.split())

    assert "Radar observations of trajectories can be simulated from one or more fixed radar stations." in normalized_radar
    assert "Station coordinates normally use the earth shape selected by \\problemblock{*earth}." in normalized_radar
    assert "A radar block may override that shape for radar geometry only." in normalized_radar
    assert "The station number and name must immediately follow the \\problemblock{*radar} keyword" in normalized_radar
    assert "test_radar_block_parses_station_identity_shape_and_parameters" in normalized_framing
    assert "test_radar_continuation_preserves_shape_and_typed_parameters" in normalized_framing
    assert "test_radar_direct_earth_shape_requires_radius_and_one_polar_parameter" in normalized_framing
    assert "test_radar_named_earth_shape_cannot_be_mixed_with_direct_geometry" in normalized_framing
    assert "test_radar_block_recovers_missing_identity_and_unknown_parameter" in normalized_framing
    assert "test_radar_station_numbers_are_positive_and_unique" in normalized_framing
    assert "test_duplicate_radar_parameters_are_preserved_and_diagnosed" in normalized_framing
    assert "test_manual_radar_continuation_keeps_earth_shape_and_parameters" in normalized_snippet
####


def test_gravity_constants_are_anchored_by_manual_table_labels() -> None:
    earth_text = (ROOT / "manual/chapters/chapter04/04_03_earth.tex").read_text(encoding="utf-8")
    normalized_earth = " ".join(earth_text.split())
    assert "\\label{tab:wgs-earth-model-values}" in normalized_earth
    assert "\\label{tab:tsap-compatible-earth-values}" in normalized_earth
    assert "\\label{tab:full-earth-model-values}" in normalized_earth
    assert "The source manual intentionally skips Table 4-18." in normalized_earth
    assert "WGS-72 and WGS-84 Earth Models" in normalized_earth
    assert "J_2" in normalized_earth
    assert "J_3" in normalized_earth
    assert "J_4" in normalized_earth


def test_gravity_constants_have_explicit_source_page_anchors() -> None:
    constants = _load("spec/constants.yaml")["constants"]
    gravity_constants = [item for item in constants if item["id"].startswith("earth.")]
    assert gravity_constants

    expected_pages = {
        "earth.wgs84.equatorial_radius": "4-55",
        "earth.wgs84.flattening": "4-55",
        "earth.wgs84.gravitational_parameter": "4-55",
        "earth.wgs84.rotation_rate": "4-55",
        "earth.wgs84.j2": "4-55",
        "earth.wgs72.equatorial_radius": "4-55",
        "earth.wgs72.polar_radius": "4-55",
        "earth.wgs72.flattening": "4-55",
        "earth.wgs72.rotation_rate": "4-55",
        "earth.wgs72.gravitational_parameter": "4-55",
        "earth.wgs72.j2": "4-55",
        "earth.tsap84.j3": "4-55",
        "earth.tsap84.j4": "4-55",
        "earth.tsap72.j3": "4-55",
        "earth.tsap72.j4": "4-55",
        "earth.wgs84_full.c20": "4-56",
        "earth.wgs84_full.c22": "4-56",
        "earth.wgs84_full.c30": "4-56",
        "earth.wgs84_full.c31": "4-56",
        "earth.wgs84_full.c32": "4-56",
        "earth.wgs84_full.c33": "4-56",
        "earth.wgs84_full.c40": "4-56",
        "earth.wgs84_full.c41": "4-56",
        "earth.wgs84_full.c42": "4-56",
        "earth.wgs84_full.c43": "4-56",
        "earth.wgs84_full.c44": "4-56",
        "earth.wgs84_full.s22": "4-56",
        "earth.wgs84_full.s31": "4-56",
        "earth.wgs84_full.s32": "4-56",
        "earth.wgs84_full.s33": "4-56",
        "earth.wgs84_full.s41": "4-56",
        "earth.wgs84_full.s42": "4-56",
        "earth.wgs84_full.s43": "4-56",
        "earth.wgs84_full.s44": "4-56",
        "earth.gem_t1_full.c20": "4-56",
        "earth.gem_t1_full.c22": "4-56",
        "earth.gem_t1_full.c30": "4-56",
        "earth.gem_t1_full.c31": "4-56",
        "earth.gem_t1_full.c32": "4-56",
        "earth.gem_t1_full.c33": "4-56",
        "earth.gem_t1_full.c40": "4-56",
        "earth.gem_t1_full.c41": "4-56",
        "earth.gem_t1_full.c42": "4-56",
        "earth.gem_t1_full.c43": "4-56",
        "earth.gem_t1_full.c44": "4-56",
        "earth.gem_t1_full.s22": "4-56",
        "earth.gem_t1_full.s31": "4-56",
        "earth.gem_t1_full.s32": "4-56",
        "earth.gem_t1_full.s33": "4-56",
        "earth.gem_t1_full.s41": "4-56",
        "earth.gem_t1_full.s42": "4-56",
        "earth.gem_t1_full.s43": "4-56",
        "earth.gem_t1_full.s44": "4-56",
    }
    expected_tables = {
        "earth.wgs84.equatorial_radius": "4-16",
        "earth.wgs84.flattening": "4-16",
        "earth.wgs84.gravitational_parameter": "4-16",
        "earth.wgs84.rotation_rate": "4-16",
        "earth.wgs84.j2": "4-16",
        "earth.wgs72.equatorial_radius": "4-16",
        "earth.wgs72.polar_radius": "4-16",
        "earth.wgs72.flattening": "4-16",
        "earth.wgs72.rotation_rate": "4-16",
        "earth.wgs72.gravitational_parameter": "4-16",
        "earth.wgs72.j2": "4-16",
        "earth.tsap84.j3": "4-17",
        "earth.tsap84.j4": "4-17",
        "earth.tsap72.j3": "4-17",
        "earth.tsap72.j4": "4-17",
        "earth.wgs84_full.c20": "4-19",
        "earth.wgs84_full.c22": "4-19",
        "earth.wgs84_full.c30": "4-19",
        "earth.wgs84_full.c31": "4-19",
        "earth.wgs84_full.c32": "4-19",
        "earth.wgs84_full.c33": "4-19",
        "earth.wgs84_full.c40": "4-19",
        "earth.wgs84_full.c41": "4-19",
        "earth.wgs84_full.c42": "4-19",
        "earth.wgs84_full.c43": "4-19",
        "earth.wgs84_full.c44": "4-19",
        "earth.wgs84_full.s22": "4-19",
        "earth.wgs84_full.s31": "4-19",
        "earth.wgs84_full.s32": "4-19",
        "earth.wgs84_full.s33": "4-19",
        "earth.wgs84_full.s41": "4-19",
        "earth.wgs84_full.s42": "4-19",
        "earth.wgs84_full.s43": "4-19",
        "earth.wgs84_full.s44": "4-19",
        "earth.gem_t1_full.c20": "4-19",
        "earth.gem_t1_full.c22": "4-19",
        "earth.gem_t1_full.c30": "4-19",
        "earth.gem_t1_full.c31": "4-19",
        "earth.gem_t1_full.c32": "4-19",
        "earth.gem_t1_full.c33": "4-19",
        "earth.gem_t1_full.c40": "4-19",
        "earth.gem_t1_full.c41": "4-19",
        "earth.gem_t1_full.c42": "4-19",
        "earth.gem_t1_full.c43": "4-19",
        "earth.gem_t1_full.c44": "4-19",
        "earth.gem_t1_full.s22": "4-19",
        "earth.gem_t1_full.s31": "4-19",
        "earth.gem_t1_full.s32": "4-19",
        "earth.gem_t1_full.s33": "4-19",
        "earth.gem_t1_full.s41": "4-19",
        "earth.gem_t1_full.s42": "4-19",
        "earth.gem_t1_full.s43": "4-19",
        "earth.gem_t1_full.s44": "4-19",
    }

    for constant in gravity_constants:
        source = constant["source"]
        assert source["manual_page"] == expected_pages[constant["id"]]
        assert source["manual_table"] == expected_tables[constant["id"]]
        assert source["tex_file"] == "manual/chapters/chapter04/04_03_earth.tex"
        assert source["tex_line_start"] <= source["tex_line_end"]
####


def test_atmosphere_constants_are_anchored_by_manual_table_labels() -> None:
    atmos_text = (ROOT / "manual/chapters/chapter04/04_01_atmos.tex").read_text(encoding="utf-8")
    normalized_text = " ".join(atmos_text.split())

    assert "\\label{tab:atmosphere-types}" in normalized_text
    assert "The default is the 1976 U.S. Standard Atmosphere." in normalized_text
    assert "One selected atmosphere applies to all trajectories in the problem" in normalized_text
    assert "type 1 is common" in normalized_text
    assert "standard" in normalized_text
    assert "none" in normalized_text
####


def test_atmosphere_equation_contract_is_source_located() -> None:
    atmos_text = (ROOT / "manual/chapters/chapter02/03_01_atmosphere.tex").read_text(encoding="utf-8")
    atmos_tests = (ROOT / "tests/unit/test_atmosphere_equations.py").read_text(encoding="utf-8")
    normalized_atmos = " ".join(atmos_text.split())
    normalized_tests = " ".join(atmos_tests.split())

    assert "\\label{eq:atmos-aerostatic}" in normalized_atmos
    assert "\\label{eq:atmos-perfect-gas}" in normalized_atmos
    assert "\\label{eq:molecular-temperature-gradient}" in normalized_atmos
    assert "\\label{eq:molecular-temperature-definition}" in normalized_atmos
    assert "\\label{eq:geopotential-altitude-integral}" in normalized_atmos
    assert "\\label{eq:atmos-gravity-inverse-square}" in normalized_atmos
    assert "\\label{eq:geopotential-altitude-closed-form}" in normalized_atmos
    assert "\\label{eq:lambert-sea-level-gravity}" in normalized_atmos
    assert "\\label{eq:effective-geopotential-earth-radius}" in normalized_atmos
    assert "\\label{eq:layer-molecular-temperature}" in normalized_atmos
    assert "\\label{eq:layer-molecular-weight}" in normalized_atmos
    assert "\\label{eq:atmos-temperature}" in normalized_atmos
    assert "\\label{eq:pressure-geopotential-differential}" in normalized_atmos
    assert "\\label{eq:pressure-molecular-temperature}" in normalized_atmos
    assert "\\label{eq:pressure-separable-differential}" in normalized_atmos
    assert "\\label{eq:pressure-isothermal-layer}" in normalized_atmos
    assert "\\label{eq:pressure-gradient-layer}" in normalized_atmos
    assert "\\label{eq:atmos-density}" in normalized_atmos
    assert "\\label{eq:atmos-speed-of-sound}" in normalized_atmos
    assert "\\label{eq:atmos-kinematic-viscosity}" in normalized_atmos
    assert "test_atmosphere_base_relations_follow_the_manual_formulas" in normalized_tests
    assert "test_atmosphere_layer_pressure_and_properties_follow_the_manual_formulas" in normalized_tests
####


def test_geodesy_equation_contract_is_source_located() -> None:
    geodesy_text = (ROOT / "manual/chapters/chapter02/01_04_geocentric.tex").read_text(encoding="utf-8")
    geodetic_text = (ROOT / "manual/chapters/chapter02/01_05_local_geodetic.tex").read_text(encoding="utf-8")
    geodesy_tests = (ROOT / "tests/unit/test_geodesy_equations.py").read_text(encoding="utf-8")
    normalized_geodesy = " ".join(geodesy_text.split())
    normalized_geodetic = " ".join(geodetic_text.split())
    normalized_tests = " ".join(geodesy_tests.split())

    assert "\\label{eq:geocentric-position-to-ecfc}" in normalized_geodesy
    assert "\\label{eq:longitude-from-ecfc}" in normalized_geodesy
    assert "\\label{eq:geocentric-latitude-from-ecfc}" in normalized_geodesy
    assert "\\label{eq:geocentric-velocity-x}" in normalized_geodesy
    assert "\\label{eq:geocentric-velocity-y}" in normalized_geodesy
    assert "\\label{eq:geocentric-velocity-z}" in normalized_geodesy
    assert "\\label{eq:geocentric-gamma-from-ecfc}" in normalized_geodesy
    assert "\\label{eq:geocentric-psi-from-ecfc}" in normalized_geodesy
    assert "\\label{eq:geocentric-to-ecfc-x}" in normalized_geodesy
    assert "\\label{eq:geocentric-to-ecfc-y}" in normalized_geodesy
    assert "\\label{eq:geocentric-to-ecfc-z}" in normalized_geodesy
    assert "\\label{eq:ellipsoid-parameter-relation}" in normalized_geodetic
    assert "\\label{eq:local-geodetic-x-unit-vector}" in normalized_geodetic
    assert "\\label{eq:local-geodetic-y-unit-vector}" in normalized_geodetic
    assert "\\label{eq:local-geodetic-z-unit-vector}" in normalized_geodetic
    assert "test_polar_radius_matches_ellipsoid_parameter_relation" in normalized_tests
    assert "test_geocentric_position_round_trips_through_ecfc" in normalized_tests
    assert "test_geodetic_surface_geometry_matches_manual_formulas" in normalized_tests
    assert "test_geodetic_position_round_trips_through_ecfc" in normalized_tests
    assert "test_geodetic_unit_vectors_are_orthonormal" in normalized_tests
    assert "test_geocentric_velocity_angles_and_transform_round_trip" in normalized_tests
    assert "test_geocentric_velocity_zero_and_vertical_edge_cases" in normalized_tests
    assert "test_geodetic_velocity_angles_and_transform_round_trip" in normalized_tests
    assert "test_geocentric_unit_vectors_are_orthonormal" in normalized_tests
    assert "test_tangent_plane_unit_vectors_follow_manual_formulas" in normalized_tests
####


def test_branch_singularity_matrices_are_source_located() -> None:
    branches = _load("spec/branch_matrices.yaml")["branches"]
    assert isinstance(branches, list)
    assert branches
    branch_ids = {item["id"] for item in branches}
    assert branch_ids == {
        "TAOS-BRANCH-0001",
        "TAOS-BRANCH-0002",
        "TAOS-BRANCH-0003",
        "TAOS-BRANCH-0004",
        "TAOS-BRANCH-0005",
        "TAOS-BRANCH-0006",
    }

    for branch in branches:
        source = branch["source"]
        assert source["manual_section"]
        assert source["manual_pages"]
        assert source["manual_text"]
        assert branch["branch"]
        assert branch["expected_behavior"]
        assert branch["tests"]

    branch_map = {item["id"]: item for item in branches}
    assert branch_map["TAOS-BRANCH-0001"]["expected_diagnostic"] == "windward meridian unit vector is undefined when total angle of attack is zero"
    assert branch_map["TAOS-BRANCH-0002"]["expected_behavior"] == "preserve_source_special_case_convention"
    assert branch_map["TAOS-BRANCH-0002"]["expected_diagnostic"] == "none"
    assert branch_map["TAOS-BRANCH-0003"]["expected_diagnostic"] == "wind axes are undefined at zero speed"
    assert branch_map["TAOS-BRANCH-0004"]["expected_diagnostic"] == "wind axes are undefined when velocity is parallel to the geodetic up axis"
    assert branch_map["TAOS-BRANCH-0005"]["expected_diagnostic"] == "yaw is undefined"
    assert branch_map["TAOS-BRANCH-0006"]["expected_diagnostic"] == "flight path angle"


def test_branch_requirement_is_traced_to_the_matrix_and_diagnostics() -> None:
    requirements = _load("spec/requirements.yaml")["requirements"]
    traceability = _load("traceability.yaml")["requirements"]
    requirement = next(item for item in requirements if item["id"] == "TAOS-REQ-BRANCH-0001")
    assert requirement["source"]["metadata"] == "verification/spec/branch_matrices.yaml"
    assert "tests/unit/test_frames_equations.py" in requirement["tests"]
    assert "tests/unit/test_forces_equations.py" in requirement["tests"]

    trace_entry = traceability["TAOS-REQ-BRANCH-0001"]
    assert trace_entry["evidence_type"] == "branch_and_singularity_matrix"
    assert "verification/spec/branch_matrices.yaml" in trace_entry["evidence"]
    assert "manual/chapters/chapter02/01_09_wind.tex" in trace_entry["evidence"]


def test_constant_provenance_matrix_is_traced_to_the_registry_and_manual_tables() -> None:
    constants = _load("spec/constant_matrices.yaml")["constants"]
    requirements = _load("spec/requirements.yaml")["requirements"]
    traceability = _load("traceability.yaml")["requirements"]
    requirement = next(item for item in requirements if item["id"] == "TAOS-REQ-GRAV-0007")
    assert requirement["source"]["metadata"] == "verification/spec/constant_matrices.yaml"
    assert "tests/unit/test_gravity.py" in requirement["tests"]
    assert "tests/unit/test_atmosphere_equations.py" in requirement["tests"]

    constant_map = {item["id"]: item for item in constants}
    assert constant_map["TAOS-CONST-0001"]["source"]["manual_table"] == "4-16"
    assert constant_map["TAOS-CONST-0002"]["family"] == "wgs72_j2"
    assert constant_map["TAOS-CONST-0003"]["source"]["manual_table"] == "4-17"
    assert constant_map["TAOS-CONST-0004"]["source"]["manual_table"] == "4-19"
    assert constant_map["TAOS-CONST-0005"]["family"] == "gem_t1_degree4_full"
    assert constant_map["TAOS-CONST-0006"]["source"]["manual_pages"] == ["2-52"]

    trace_entry = traceability["TAOS-REQ-GRAV-0007"]
    assert trace_entry["evidence_type"] == "family_constant_provenance_matrix"
    assert "verification/spec/constant_matrices.yaml" in trace_entry["evidence"]
    assert "manual/chapters/chapter04/04_03_earth.tex" in trace_entry["evidence"]


def test_equation_derivation_matrix_is_traced_to_the_registry_and_manual_sections() -> None:
    equation_matrix = _load("spec/equation_matrices.yaml")["equations"]
    requirements = _load("spec/requirements.yaml")["requirements"]
    traceability = _load("traceability.yaml")["requirements"]
    requirement = next(item for item in requirements if item["id"] == "TAOS-REQ-EQMAT-0001")
    assert requirement["source"]["metadata"] == "verification/spec/equation_matrices.yaml"
    assert "tests/unit/test_frames_equations.py" in requirement["tests"]
    assert "tests/unit/test_gravity_equations.py" in requirement["tests"]

    equation_map = {item["id"]: item for item in equation_matrix}
    assert equation_map["TAOS-EQMAT-0001"]["topic"] == "coordinate_frames"
    assert equation_map["TAOS-EQMAT-0004"]["topic"] == "geodesy"
    assert equation_map["TAOS-EQMAT-0006"]["topic"] == "optimization"

    trace_entry = traceability["TAOS-REQ-EQMAT-0001"]
    assert trace_entry["evidence_type"] == "topic_equation_derivation_matrix"
    assert "verification/spec/equation_matrices.yaml" in trace_entry["evidence"]
    assert "manual/chapters/chapter02/03_04_gravity_model.tex" in trace_entry["evidence"]


def test_regression_matrix_is_traced_to_the_registry_and_source_corpora() -> None:
    regression_matrix = _load("spec/regression_matrices.yaml")["regressions"]
    requirements = _load("spec/requirements.yaml")["requirements"]
    traceability = _load("traceability.yaml")["requirements"]
    requirement = next(item for item in requirements if item["id"] == "TAOS-REQ-REG-0001")
    assert requirement["source"]["metadata"] == "verification/spec/regression_matrices.yaml"
    assert "tests/unit/test_runtime_lowering.py" in requirement["tests"]
    assert "tests/unit/test_outputs.py" in requirement["tests"]
    assert "tests/parser/test_manual_snippet_corpus.py" in requirement["tests"]

    regression_map = {item["id"]: item for item in regression_matrix}
    assert regression_map["TAOS-REG-0001"]["category"] == "canonical_ecfc_zero_force"
    assert regression_map["TAOS-REG-0002"]["evidence"]["oracles"] == "monotonic"
    assert regression_map["TAOS-REG-0004"]["source"]["manifest"] == "tests/fixtures/taos_manual_corpus_v22/manifest.yaml"
    assert regression_map["TAOS-REG-0004"]["source"]["manual_examples"][0] == "manual/chapters/chapter04/05_examples_intro.tex"
    assert "manual/chapters/chapter04/05_04_ground_intercept.tex" in regression_map["TAOS-REG-0004"]["source"]["manual_examples"]
    assert "examples/chapter04/ballistic-reentry-summary.txt" in regression_map["TAOS-REG-0004"]["source"]["manual_output_listings"]
    assert "examples/chapter04/problem-output-example.txt" in regression_map["TAOS-REG-0004"]["source"]["manual_output_listings"]

    trace_entry = traceability["TAOS-REQ-REG-0001"]
    assert trace_entry["evidence_type"] == "source_linked_regression_matrix"
    assert "verification/spec/regression_matrices.yaml" in trace_entry["evidence"]
    assert "tests/fixtures/taos_manual_corpus_v22/manifest.yaml" in trace_entry["evidence"]
    assert "manual/chapters/chapter04/05_02_ballistic_rocket.tex" in trace_entry["evidence"]
    assert "examples/chapter04/ballistic-reentry-summary.txt" in trace_entry["evidence"]
    assert "examples/chapter04/problem-output-example.txt" in trace_entry["evidence"]


def test_diagnostic_matrix_is_traced_to_the_registry_and_recovery_families() -> None:
    diagnostic_matrix = _load("spec/diagnostic_matrices.yaml")["diagnostics"]
    requirements = _load("spec/requirements.yaml")["requirements"]
    traceability = _load("traceability.yaml")["requirements"]
    requirement = next(item for item in requirements if item["id"] == "TAOS-REQ-DIAG-0001")
    assert requirement["source"]["metadata"] == "verification/spec/diagnostic_matrices.yaml"
    assert "tests/parser/test_problem_framing.py" in requirement["tests"]
    assert "tests/parser/test_manual_snippet_corpus.py" in requirement["tests"]

    diag_map = {item["id"]: item for item in diagnostic_matrix}
    assert diag_map["TAOS-DIAG-0001"]["category"] == "frame_singularity"
    assert diag_map["TAOS-DIAG-0002"]["category"] == "velocity_singularity"
    assert diag_map["TAOS-DIAG-0003"]["category"] == "atmosphere_malformed_input"
    assert diag_map["TAOS-DIAG-0004"]["category"] == "problem_framing"
    assert diag_map["TAOS-DIAG-0005"]["category"] == "output_block_malformed_input"
    assert diag_map["TAOS-DIAG-0006"]["category"] == "ambiguous_manual_corpus"
    assert diag_map["TAOS-DIAG-0007"]["category"] == "duplicate_independent_values"

    duplicate_requirement = next(item for item in requirements if item["id"] == "TAOS-REQ-DIAG-0002")
    assert duplicate_requirement["source"]["metadata"] == "verification/spec/diagnostic_matrices.yaml"
    assert "tests/parser/test_manual_snippet_corpus.py" in duplicate_requirement["tests"]
    assert "tests/unit/test_verification_baseline.py" in duplicate_requirement["tests"]

    trace_entry = traceability["TAOS-REQ-DIAG-0001"]
    assert trace_entry["evidence_type"] == "source_linked_diagnostic_matrix"
    assert "verification/spec/diagnostic_matrices.yaml" in trace_entry["evidence"]
    assert "tests/parser/test_output_blocks.py" in trace_entry["evidence"]

    duplicate_trace_entry = traceability["TAOS-REQ-DIAG-0002"]
    assert duplicate_trace_entry["evidence_type"] == "source_linked_diagnostic_matrix"
    assert "verification/spec/diagnostic_matrices.yaml" in duplicate_trace_entry["evidence"]
    assert "tests/parser/test_manual_snippet_corpus.py" in duplicate_trace_entry["evidence"]
    assert "tests/fixtures/taos_manual_corpus_v22/wrappers/chapter03/ch3-035__wrapped.tbl" in duplicate_trace_entry["evidence"]


def test_ambiguity_matrix_is_traced_to_the_registry_and_family_entries() -> None:
    ambiguity_matrix = _load("spec/ambiguity_matrices.yaml")["ambiguities"]
    ambiguities = _load("spec/ambiguities.yaml")["ambiguities"]
    requirements = _load("spec/requirements.yaml")["requirements"]
    traceability = _load("traceability.yaml")["requirements"]
    requirement = next(item for item in requirements if item["id"] == "TAOS-REQ-AMBMAT-0001")
    assert requirement["source"]["metadata"] == "verification/spec/ambiguity_matrices.yaml"
    assert "tests/unit/test_verification_baseline.py" in requirement["tests"]

    ambiguity_map = {item["id"]: item for item in ambiguities}
    ambiguity_matrix_map = {item["id"]: item for item in ambiguity_matrix}
    assert ambiguity_matrix_map["TAOS-AMBMAT-0001"]["ambiguity_ids"] == ["TAOS-AMB-0001"]
    assert ambiguity_matrix_map["TAOS-AMBMAT-0002"]["ambiguity_ids"] == ["TAOS-AMB-0002", "TAOS-AMB-0005"]
    assert ambiguity_matrix_map["TAOS-AMBMAT-0003"]["ambiguity_ids"] == ["TAOS-AMB-0003", "TAOS-AMB-0004"]
    assert ambiguity_map["TAOS-AMB-0003"]["decision"]["status"] == "unresolved"

    trace_entry = traceability["TAOS-REQ-AMBMAT-0001"]
    assert trace_entry["evidence_type"] == "source_linked_ambiguity_matrix"
    assert "verification/spec/ambiguity_matrices.yaml" in trace_entry["evidence"]
    assert "verification/spec/ambiguities.yaml" in trace_entry["evidence"]


def test_sign_matrix_is_traced_to_the_registry_and_orientation_families() -> None:
    sign_matrix = _load("spec/sign_matrices.yaml")["signs"]
    requirements = _load("spec/requirements.yaml")["requirements"]
    traceability = _load("traceability.yaml")["requirements"]
    requirement = next(item for item in requirements if item["id"] == "TAOS-REQ-SIGN-0001")
    assert requirement["source"]["metadata"] == "verification/spec/sign_matrices.yaml"
    assert "tests/unit/test_frames_equations.py" in requirement["tests"]
    assert "tests/unit/test_gravity_equations.py" in requirement["tests"]

    sign_map = {item["id"]: item for item in sign_matrix}
    assert sign_map["TAOS-SIGN-0001"]["category"] == "ecfc_orientation"
    assert sign_map["TAOS-SIGN-0002"]["category"] == "local_horizon_orientation"
    assert sign_map["TAOS-SIGN-0003"]["category"] == "body_orientation"
    assert sign_map["TAOS-SIGN-0004"]["category"] == "wind_orientation"
    assert sign_map["TAOS-SIGN-0005"]["category"] == "gravity_and_coefficients"

    trace_entry = traceability["TAOS-REQ-SIGN-0001"]
    assert trace_entry["evidence_type"] == "source_linked_sign_matrix"
    assert "verification/spec/sign_matrices.yaml" in trace_entry["evidence"]
    assert "manual/chapters/chapter02/01_01_ecfc.tex" in trace_entry["evidence"]


def test_frame_matrix_is_traced_to_the_registry_and_frame_families() -> None:
    frame_matrix = _load("spec/frame_matrices.yaml")["frames"]
    requirements = _load("spec/requirements.yaml")["requirements"]
    traceability = _load("traceability.yaml")["requirements"]
    requirement = next(item for item in requirements if item["id"] == "TAOS-REQ-FRAME-0009")
    assert requirement["source"]["metadata"] == "verification/spec/frame_matrices.yaml"
    assert "tests/unit/test_coordinates.py" in requirement["tests"]
    assert "tests/unit/test_geodesy_equations.py" in requirement["tests"]

    frame_map = {item["id"]: item for item in frame_matrix}
    assert frame_map["TAOS-FRAME-0001"]["category"] == "earth_fixed_and_inertial"
    assert frame_map["TAOS-FRAME-0002"]["category"] == "local_horizon_and_geodesy"
    assert frame_map["TAOS-FRAME-0003"]["category"] == "body_and_wind"
    assert frame_map["TAOS-FRAME-0004"]["category"] == "tangent_plane"

    trace_entry = traceability["TAOS-REQ-FRAME-0009"]
    assert trace_entry["evidence_type"] == "source_linked_frame_matrix"
    assert "verification/spec/frame_matrices.yaml" in trace_entry["evidence"]
    assert "verification/spec/anchors.yaml" in trace_entry["evidence"]


def test_matrix_inventory_is_traced_to_the_registry_and_all_matrix_files() -> None:
    inventory = _load("spec/matrix_inventory.yaml")["matrices"]
    requirements = _load("spec/requirements.yaml")["requirements"]
    traceability = _load("traceability.yaml")["requirements"]
    requirement = next(item for item in requirements if item["id"] == "TAOS-REQ-MATRIX-0001")
    assert requirement["source"]["metadata"] == "verification/spec/matrix_inventory.yaml"
    assert "tests/unit/test_verification_baseline.py" in requirement["tests"]

    inventory_map = {item["id"]: item for item in inventory}
    assert inventory_map["TAOS-MATRIX-0001"]["path"] == "verification/spec/frame_matrices.yaml"
    assert inventory_map["TAOS-MATRIX-0004"]["path"] == "verification/spec/constant_matrices.yaml"
    assert inventory_map["TAOS-MATRIX-0011"]["path"] == "verification/traceability.yaml"
    assert inventory_map["TAOS-MATRIX-0009"]["requirement_id"] == "TAOS-REQ-FRAME-0003"

    trace_entry = traceability["TAOS-REQ-MATRIX-0001"]
    assert trace_entry["evidence_type"] == "matrix_inventory"
    assert "verification/spec/matrix_inventory.yaml" in trace_entry["evidence"]
    assert "verification/spec/branch_matrices.yaml" in trace_entry["evidence"]


def test_chapter4_normative_inventory_is_traced_to_the_registry_and_section_groups() -> None:
    chapter4_inventory = _load("spec/chapter4_normative_inventory.yaml")["chapters"]
    requirements = _load("spec/requirements.yaml")["requirements"]
    traceability = _load("traceability.yaml")["requirements"]
    requirement = next(item for item in requirements if item["id"] == "TAOS-REQ-CH4-0001")
    assert requirement["source"]["metadata"] == "verification/spec/chapter4_normative_inventory.yaml"
    assert "tests/parser/test_problem_framing.py" in requirement["tests"]
    assert "tests/unit/test_runtime_lowering.py" in requirement["tests"]

    chapter_map = {item["id"]: item for item in chapter4_inventory}
    assert chapter_map["TAOS-CH4-0001"]["source"]["manual_text"] == [
        "manual/chapters/chapter04/00_introduction.tex",
        "manual/chapters/chapter04/01_file_format.tex",
    ]
    assert chapter_map["TAOS-CH4-0001"]["requirement_ids"] == [
        "TAOS-REQ-PRB-0001",
        "TAOS-REQ-GRAMMAR-0001",
        "TAOS-REQ-GRAMMAR-0003",
    ]
    assert chapter_map["TAOS-CH4-0002"]["source"]["manual_text"][0] == "manual/chapters/chapter04/02_segment_overview.tex"
    assert "TAOS-REQ-CH4-SEG-0001" in chapter_map["TAOS-CH4-0002"]["requirement_ids"]
    assert "TAOS-REQ-INIT-0002" in chapter_map["TAOS-CH4-0002"]["requirement_ids"]
    assert chapter_map["TAOS-CH4-0003"]["source"]["manual_text"][0] == "manual/chapters/chapter04/03_trajectory_overview.tex"
    assert "TAOS-REQ-CH4-TRAJ-0001" in chapter_map["TAOS-CH4-0003"]["requirement_ids"]
    assert "TAOS-REQ-UFMT-0001" in chapter_map["TAOS-CH4-0003"]["requirement_ids"]
    assert chapter_map["TAOS-CH4-0004"]["source"]["manual_text"][0] == "manual/chapters/chapter04/04_problem_overview.tex"
    assert "TAOS-REQ-CH4-PROB-0001" in chapter_map["TAOS-CH4-0004"]["requirement_ids"]
    assert "TAOS-REQ-GRAV-0004" in chapter_map["TAOS-CH4-0004"]["requirement_ids"]
    assert chapter_map["TAOS-CH4-0005"]["source"]["manual_text"][0] == "manual/chapters/chapter04/04_05_file.tex"
    assert "TAOS-REQ-FILE-0001" in chapter_map["TAOS-CH4-0005"]["requirement_ids"]
    assert "TAOS-REQ-OPT-0002" in chapter_map["TAOS-CH4-0005"]["requirement_ids"]
    assert "TAOS-REQ-WIND-0001" in chapter_map["TAOS-CH4-0006"]["requirement_ids"]
    assert "TAOS-REQ-SCEN-0004" in chapter_map["TAOS-CH4-0007"]["requirement_ids"]

    trace_entry = traceability["TAOS-REQ-CH4-0001"]
    assert trace_entry["evidence_type"] == "chapter4_normative_inventory"
    assert "verification/spec/chapter4_normative_inventory.yaml" in trace_entry["evidence"]
    assert "manual/chapters/chapter04/04_14_wind.tex" in trace_entry["evidence"]

    requirement_ids = {item["id"] for item in requirements}
    for identifier in (
        "TAOS-REQ-CH4-SEG-0001",
        "TAOS-REQ-CH4-TRAJ-0001",
        "TAOS-REQ-CH4-PROB-0001",
        "TAOS-REQ-CH4-EX-0001",
    ):
        assert identifier in requirement_ids
        assert identifier in traceability

    assert "manual/chapters/chapter04/02_segment_overview.tex" in traceability["TAOS-REQ-CH4-SEG-0001"]["evidence"]
    assert "manual/chapters/chapter04/03_trajectory_overview.tex" in traceability["TAOS-REQ-CH4-TRAJ-0001"]["evidence"]
    assert "manual/chapters/chapter04/04_problem_overview.tex" in traceability["TAOS-REQ-CH4-PROB-0001"]["evidence"]
    assert "manual/chapters/chapter04/05_04_ground_intercept.tex" in traceability["TAOS-REQ-CH4-EX-0001"]["evidence"]


def test_reviewer_inventory_is_traced_to_the_registry_and_required_roles() -> None:
    review_inventory = _load("spec/review_inventory.yaml")["reviews"]
    requirements = _load("spec/requirements.yaml")["requirements"]
    traceability = _load("traceability.yaml")["requirements"]
    requirement = next(item for item in requirements if item["id"] == "TAOS-REQ-REVIEW-0001")
    assert requirement["source"]["metadata"] == "verification/spec/review_inventory.yaml"
    assert "tests/unit/test_verification_baseline.py" in requirement["tests"]

    review_map = {item["id"]: item for item in review_inventory}
    assert review_map["TAOS-REVIEW-0001"]["required_roles"] == [
        "documentary_reviewer",
        "flight_mechanics_reviewer",
    ]
    assert review_map["TAOS-REVIEW-0002"]["required_roles"] == [
        "flight_mechanics_reviewer",
        "geodesy_gravity_reviewer",
    ]
    assert review_map["TAOS-REVIEW-0003"]["required_roles"] == [
        "flight_mechanics_reviewer",
        "mathematical_reviewer",
    ]
    assert review_map["TAOS-REVIEW-0004"]["required_roles"] == [
        "geodesy_gravity_reviewer",
        "documentary_reviewer",
    ]
    assert review_map["TAOS-REVIEW-0005"]["required_roles"] == [
        "mathematical_reviewer",
        "flight_mechanics_reviewer",
    ]
    assert review_map["TAOS-REVIEW-0006"]["required_roles"] == [
        "numerical_reviewer",
        "historical_reviewer",
    ]
    assert review_map["TAOS-REVIEW-0007"]["required_roles"] == [
        "language_reviewer",
        "documentary_reviewer",
    ]
    assert review_map["TAOS-REVIEW-0008"]["required_roles"] == [
        "mathematical_reviewer",
        "documentary_reviewer",
        "historical_reviewer",
    ]
    assert review_map["TAOS-REVIEW-0009"]["required_roles"] == [
        "documentary_reviewer",
        "numerical_reviewer",
    ]
    assert review_map["TAOS-REVIEW-0010"]["target"] == "verification/spec/chapter4_normative_inventory.yaml"
    assert review_map["TAOS-REVIEW-0011"]["target"] == "TAOS-REQ-CH4-SEG-0001"
    assert review_map["TAOS-REVIEW-0012"]["required_roles"] == [
        "documentary_reviewer",
        "numerical_reviewer",
    ]
    assert review_map["TAOS-REVIEW-0013"]["target"] == "TAOS-REQ-CH4-PROB-0001"
    assert review_map["TAOS-REVIEW-0014"]["required_roles"] == [
        "numerical_reviewer",
        "historical_reviewer",
    ]

    trace_entry = traceability["TAOS-REQ-REVIEW-0001"]
    assert trace_entry["evidence_type"] == "reviewer_role_inventory"
    assert "verification/spec/review_inventory.yaml" in trace_entry["evidence"]
    assert "verification/review_policy.md" in trace_entry["evidence"]


def test_mutation_matrix_is_traced_to_the_registry_and_defect_families() -> None:
    mutation_matrix = _load("spec/mutation_matrices.yaml")["mutations"]
    requirements = _load("spec/requirements.yaml")["requirements"]
    traceability = _load("traceability.yaml")["requirements"]
    requirement = next(item for item in requirements if item["id"] == "TAOS-REQ-MUT-0001")
    assert requirement["source"]["metadata"] == "verification/spec/mutation_matrices.yaml"
    assert "tests/unit/test_coordinates.py" in requirement["tests"]
    assert "tests/parser/test_manual_snippet_corpus.py" in requirement["tests"]

    mutation_map = {item["id"]: item for item in mutation_matrix}
    assert mutation_map["TAOS-MUT-0001"]["category"] == "anchor_sign_flip"
    assert mutation_map["TAOS-MUT-0002"]["category"] == "body_and_wind_branch_flip"
    assert mutation_map["TAOS-MUT-0003"]["category"] == "coefficient_sign_flip"
    assert mutation_map["TAOS-MUT-0004"]["category"] == "diagnostic_survival"
    assert mutation_map["TAOS-MUT-0003"]["mutation_family"] == [
        "flip_j2_sign",
        "flip_c20_sign",
        "swap_normalized_and_unnormalized_coefficients",
    ]

    trace_entry = traceability["TAOS-REQ-MUT-0001"]
    assert trace_entry["evidence_type"] == "source_linked_mutation_matrix"
    assert "verification/spec/mutation_matrices.yaml" in trace_entry["evidence"]
    assert "verification/spec/diagnostic_matrices.yaml" in trace_entry["evidence"]


def test_malformed_atmosphere_diagnostics_are_source_located() -> None:
    text = (ROOT / "tests/parser/test_problem_framing.py").read_text(encoding="utf-8")
    normalized_text = " ".join(text.split())

    assert "duplicate-atmos-altitude" in normalized_text
    assert "unordered-atmos-altitude" in normalized_text
    assert "conflicting-earth-shape-parameters" in normalized_text
    assert "location.line == 5" in normalized_text
    assert "recovered_records" in normalized_text
####


def test_malformed_problem_framing_diagnostics_are_source_located() -> None:
    problem_overview = (ROOT / "manual/chapters/chapter04/04_problem_overview.tex").read_text(encoding="utf-8")
    problem_text = (ROOT / "tests/parser/test_problem_framing.py").read_text(encoding="utf-8")
    normalized_overview = " ".join(problem_overview.split())
    normalized_problem = " ".join(problem_text.split())

    assert "A problem file contains one or more problems or cases" in normalized_overview
    assert "The trajectories contain one or more segments" in normalized_overview
    assert "The blocks at the end usually reference trajectory or segment information" in normalized_overview
    assert "invalid-trajectory-header" in normalized_problem
    assert "invalid-segment-header" in normalized_problem
    assert "orphan_lines == {6, 10}" in normalized_problem
####


def test_canonical_e2e_regression_anchor_is_source_linked() -> None:
    manifest = _load_from(ROOT / "tests/fixtures/taos_e2e_v23/manifest.yaml")
    assert isinstance(manifest["cases"], list)
    case = next(item for item in manifest["cases"] if item["id"] == "p001_linear_ecfc_zero_force")
    assert case["kind"] == "positive"
    assert case["source_sections"] == ["2.2.1", "4.3.3", "4.3.5"]
    assert case["features"]
    assert case["oracles"]
    assert case["problem_file"] == "input/p001_linear_ecfc_zero_force.prb"
####


def test_gravity_e2e_regression_anchor_is_source_linked() -> None:
    manifest = _load_from(ROOT / "tests/fixtures/taos_e2e_v23/manifest.yaml")
    assert isinstance(manifest["cases"], list)
    case = next(item for item in manifest["cases"] if item["id"] == "p043_wgs84_gravity_drop")
    assert case["kind"] == "positive"
    assert case["source_sections"] == ["2.3.4", "4.4.3"]
    assert case["features"]
    assert case["oracles"]
    assert case["problem_file"] == "input/p043_wgs84_gravity_drop.prb"
####


def test_optimization_e2e_regression_anchor_is_source_linked() -> None:
    manifest = _load_from(ROOT / "tests/fixtures/taos_e2e_v23/manifest.yaml")
    assert isinstance(manifest["cases"], list)
    case = next(item for item in manifest["cases"] if item["id"] == "p022_optimize_linear_boundary")
    assert case["kind"] == "positive"
    assert case["source_sections"] == ["2.6.3", "4.4.6"]
    assert case["features"]
    assert case["oracles"]
    assert case["problem_file"] == "input/p022_optimize_linear_boundary.prb"
####


def test_documented_coverage_matrix_is_complete_and_preserves_ambiguity_contract() -> None:
    from tools.build_e2e_documented_coverage import collect

    payload = collect()

    assert payload["summary"] == {
        "block_scopes_covered": 33,
        "block_scopes_total": 33,
        "table_types_covered": 19,
        "table_types_total": 19,
        "table_operations_covered": 28,
        "table_operations_total": 28,
    }
    assert payload["manual_basis"] == "TAOS User's Manual, Chapters 3 and 4"
    assert payload["ambiguous_scope_contract"]["covered_by_manifest_contract"] == [
        "problem_define_block",
        "problem_file_block",
        "problem_print_block",
    ]
    assert not [item for category in ("block_scopes", "table_types", "table_operations") for item in payload[category] if not item["covered"]]
####


def test_canonical_regression_anchors_have_explicit_scenario_thresholds() -> None:
    manifest = _load_from(ROOT / "tests/fixtures/taos_e2e_v23/manifest.json")
    assert isinstance(manifest["cases"], list)
    case_map = {item["id"]: item for item in manifest["cases"]}

    p001 = case_map["p001_linear_ecfc_zero_force"]
    p043 = case_map["p043_wgs84_gravity_drop"]
    p022 = case_map["p022_optimize_linear_boundary"]

    assert [oracle["type"] for oracle in p001["oracles"][:4]] == ["final_approx", "final_approx", "final_approx", "final_approx"]
    assert all("atol" in oracle for oracle in p001["oracles"] if oracle["type"] == "final_approx")
    assert any(oracle["type"] == "series_constant" and "atol" in oracle for oracle in p001["oracles"])

    assert [oracle["type"] for oracle in p043["oracles"]] == ["series_monotonic", "series_monotonic", "no_nan"]
    assert all("atol" in oracle for oracle in p043["oracles"] if oracle["type"] == "series_monotonic")

    assert [oracle["type"] for oracle in p022["oracles"]] == ["final_approx", "final_approx"]
    assert all("atol" in oracle for oracle in p022["oracles"] if oracle["type"] == "final_approx")
    assert float(p022["oracles"][0]["atol"]) == pytest.approx(0.1)
    assert float(p001["oracles"][0]["atol"]) == pytest.approx(1e-9)
####


def test_metamorphic_groups_are_source_linked_and_comparison_bounded() -> None:
    groups = _load_from(ROOT / "tests/fixtures/taos_e2e_v23/metamorphic/groups.yaml")["groups"]
    manifest = _load_from(ROOT / "tests/fixtures/taos_e2e_v23/manifest.json")
    assert isinstance(groups, list)
    assert isinstance(manifest["cases"], list)
    manifest_map = {item["id"]: item for item in manifest["cases"]}

    expected_group_ids = {
        "m001_dt_convergence",
        "m002_simple_full_table_equivalence",
        "m003_wind_effect",
        "m004_coordinate_initialization_equivalence",
        "m005_prop_aggregation_equivalence",
        "m006_guidance_intercept_methods",
        "m007_wind_representation_equivalence",
        "m008_ecic_coordinate_initialization_equivalence",
        "m009_aerodynamic_aggregation_equivalence",
    }
    group_map = {item["id"]: item for item in groups}
    assert set(group_map) == expected_group_ids

    for group in groups:
        assert group["comparison"] in {
            "allclose",
            "convergence",
            "crosswind_increases_airspeed",
            "both_reduce_relative_range",
        }
        assert group["cases"]
        assert group["columns"]
        for case_id in group["cases"]:
            case = manifest_map[case_id]
            assert case["kind"] == "positive"
            assert case["source_sections"]
            assert case["problem_file"]
####


def test_parser_recovery_contract_is_explicit_in_the_repository_baseline() -> None:
    framing_text = (ROOT / "tests/parser/test_problem_framing.py").read_text(encoding="utf-8")
    lossless_text = (ROOT / "tests/parser/test_lossless_baseline.py").read_text(encoding="utf-8")
    normalized_framing = " ".join(framing_text.split())
    normalized_lossless = " ".join(lossless_text.split())

    assert "duplicate-atmos-altitude" in normalized_framing
    assert "unordered-atmos-altitude" in normalized_framing
    assert "conflicting-earth-shape-parameters" in normalized_framing
    assert "invalid-trajectory-header" in normalized_framing
    assert "invalid-segment-header" in normalized_framing
    assert "ambiguous-dual-scope-block" in normalized_framing
    assert "sourceescape" in normalized_lossless or "surrogateescape" in normalized_lossless
    assert "record.location is not None" in normalized_lossless
    assert "render_bytes() == source" in normalized_lossless
####


def test_inherited_initialization_runtime_activation_is_explicit() -> None:
    runtime_text = (ROOT / "src/taoryx/runtime/initialization.py").read_text(encoding="utf-8")
    model_text = (ROOT / "src/taoryx/runtime/runtime_model.py").read_text(encoding="utf-8")
    lowering_test = (ROOT / "tests/unit/test_runtime_lowering.py").read_text(encoding="utf-8")
    algorithms_test = (ROOT / "tests/unit/test_runtime_algorithms.py").read_text(encoding="utf-8")
    problem_overview = (ROOT / "manual/chapters/chapter04/04_problem_overview.tex").read_text(encoding="utf-8")
    normalized_runtime = " ".join(runtime_text.split())
    normalized_model = " ".join(model_text.split())
    normalized_lowering = " ".join(lowering_test.split())
    normalized_algorithms = " ".join(algorithms_test.split())
    normalized_overview = " ".join(problem_overview.split())

    assert "initial state cannot be both direct and inherited" in normalized_runtime
    assert "unknown inherited initial state" in normalized_runtime
    assert "vehicle dependency cycle" in normalized_model
    assert "test_inherited_trajectory_activates_at_source_segment" in normalized_lowering
    assert "test_runtime_graph_rejects_cycles_and_steps_at_boundaries" in normalized_algorithms
    assert "These initial blocks are not associated with a particular trajectory or segment" in normalized_overview
    assert "The blocks at the end usually reference trajectory or segment information" in normalized_overview
####


def test_segment_discontinuity_order_is_explicit_in_the_repository_baseline() -> None:
    increment_text = (ROOT / "manual/chapters/chapter04/02_05_increment.tex").read_text(encoding="utf-8")
    reset_text = (ROOT / "manual/chapters/chapter04/02_11_reset.tex").read_text(encoding="utf-8")
    normalized_increment = " ".join(increment_text.split())
    normalized_reset = " ".join(reset_text.split())
    runtime_lowering = (ROOT / "tests/unit/test_runtime_lowering.py").read_text(encoding="utf-8")
    normalized_lowering = " ".join(runtime_lowering.split())

    assert "If both \\problemblock{*reset} and \\problemblock{*increment} blocks are present in a segment, values are reset first and then incremented." in normalized_increment
    assert "Multiple reset and increment blocks can be given; they are executed in the order in which they appear." in normalized_increment
    assert "the \\problemblock{*reset} block sets them equal to input values." in normalized_reset
    assert "sets the weight to 1100~lb regardless of what has happened previously" in normalized_reset
    assert "test_goto_applies_segment_reset_and_increment" in normalized_lowering
####


def test_ambiguity_registry_is_source_located_and_explicit() -> None:
    ambiguities = _load("spec/ambiguities.yaml")["ambiguities"]
    assert isinstance(ambiguities, list)
    assert ambiguities
    ambiguity_map = {item["id"]: item for item in ambiguities}
    assert len(ambiguity_map) == len(ambiguities)
    assert all(identifier.startswith("TAOS-AMB-") for identifier in ambiguity_map)

    for ambiguity in ambiguities:
        location = ambiguity["location"]
        assert location["source_page"]
        assert ambiguity["source_form"]
        assert ambiguity["candidate_interpretations"]
        assert ambiguity["decision"]["documentary_edition"]
        assert ambiguity["decision"]["status"]
        assert ambiguity["tests"]

    assert "TAOS-AMB-0002" in ambiguity_map
    assert "documented_pole_convention" in {item["id"] for item in ambiguity_map["TAOS-AMB-0002"]["candidate_interpretations"]}
    assert "TAOS-AMB-0003" in ambiguity_map
    assert "TAOS-AMB-0004" in ambiguity_map
    assert "TAOS-AMB-0005" in ambiguity_map
    assert {item["id"] for item in ambiguity_map["TAOS-AMB-0005"]["candidate_interpretations"]} == {
        "direct_projection_branch",
        "normalized_branch_family",
    }
    wind_text = (ROOT / "manual/chapters/chapter02/01_09_wind.tex").read_text(encoding="utf-8")
    normalized_wind = " ".join(wind_text.split())
    assert "For the special case $\\alpha=\\pm90^\\circ$, TAOS uses $\\beta=-\\phi_w$." in normalized_wind
    assert "For the special case $\\alpha=\\pm90^\\circ$, TAOS uses $\\phi_w=-\\beta$." in normalized_wind
    assert "For the special case $\\alpha_T=90^\\circ$, TAOS uses $\\beta=-\\phi_w$." in normalized_wind
####


def test_numerical_verification_gates_are_explicit() -> None:
    text = (ROOT / "verification/gates.md").read_text(encoding="utf-8")
    normalized_text = " ".join(text.split())

    for gate in ["N0", "N1", "N2", "N3", "N4", "N5", "N6", "N7", "N8"]:
        assert gate in normalized_text

    assert "source-located diagnostic" in normalized_text
    assert "manual table labels" in normalized_text
    assert "ambiguity record" in normalized_text
    assert "Historical-compatibility language" in normalized_text
####


def test_fixed_frame_anchors_match_expected_vectors() -> None:
    anchors = _load("spec/anchors.yaml")["anchors"]
    assert isinstance(anchors, list)
    anchor_map = {anchor["id"]: anchor for anchor in anchors}
    anchor_ids = set(anchor_map)
    assert len(anchor_ids) == len(anchors)

    assert anchor_map["ecfc_equator_greenwich"]["expected_position"] == [1.0, 0.0, 0.0]
    assert anchor_map["ecfc_equator_ninety_east"]["expected_position"] == [0.0, 1.0, 0.0]
    assert anchor_map["ecfc_north_pole"]["expected_position"] == [0.0, 0.0, 1.0]

    geocentric = geocentric_unit_vectors(Longitude(0.0), Latitude(0.0))
    assert _vector_components(geocentric.first) == pytest.approx((0.0, 0.0, 1.0))
    assert _vector_components(geocentric.second) == pytest.approx((0.0, 1.0, 0.0))
    assert _vector_components(geocentric.third) == pytest.approx((-1.0, 0.0, 0.0))

    geodetic = geodetic_unit_vectors(Longitude(0.0), Latitude(0.0))
    assert _vector_components(geodetic.first) == pytest.approx((0.0, 0.0, 1.0))
    assert _vector_components(geodetic.second) == pytest.approx((0.0, 1.0, 0.0))
    assert _vector_components(geodetic.third) == pytest.approx((-1.0, 0.0, 0.0))

    ecic = ecfc_to_ecic_position(CartesianVector3(1.0, 0.0, 0.0), math.pi / 2.0)
    assert _vector_components(ecic) == pytest.approx((0.0, 1.0, 0.0))

    wind = wind_unit_vectors_from_velocity(
        CartesianVector3(1.0, 0.0, 0.0),
        geodetic_up=CartesianVector3(0.0, 0.0, 1.0),
    )
    assert _vector_components(wind.x) == pytest.approx((1.0, 0.0, 0.0))
    assert _vector_components(wind.y) == pytest.approx((0.0, 1.0, 0.0))
    assert _vector_components(wind.z) == pytest.approx((0.0, 0.0, 1.0))

    body_axes = geodetic_body_axes_from_euler_angles(0.0, 0.0, 0.0, 0.0, 0.0)
    assert _vector_components(body_axes.x) == pytest.approx((0.0, 0.0, 1.0))
    assert _vector_components(body_axes.y) == pytest.approx((0.0, 1.0, 0.0))
    assert _vector_components(body_axes.z) == pytest.approx((-1.0, 0.0, 0.0))
    assert anchor_map["body_zero_euler_geodetic_equator_greenwich"]["expected_basis"] == {
        "x": [0.0, 0.0, 1.0],
        "y": [0.0, 1.0, 0.0],
        "z": [-1.0, 0.0, 0.0],
    }

    tangent_basis = tangent_plane_unit_vectors(Longitude(0.0), Latitude(0.0), Angle(0.0))
    assert _vector_components(tangent_basis.first) == pytest.approx((0.0, 0.0, 1.0))
    assert _vector_components(tangent_basis.second) == pytest.approx((0.0, -1.0, 0.0))
    assert _vector_components(tangent_basis.third) == pytest.approx((1.0, 0.0, 0.0))
    assert anchor_map["tangent_plane_zero_azimuth_equator_greenwich"]["expected_basis"] == {
        "x": [0.0, 0.0, 1.0],
        "y": [0.0, -1.0, 0.0],
        "z": [1.0, 0.0, 0.0],
    }


def test_anchor_comparison_rejects_obvious_sign_and_axis_mutations() -> None:
    expected_body = {
        "x": (0.0, 0.0, 1.0),
        "y": (0.0, 1.0, 0.0),
        "z": (-1.0, 0.0, 0.0),
    }
    expected_tangent = {
        "x": (0.0, 0.0, 1.0),
        "y": (0.0, -1.0, 0.0),
        "z": (1.0, 0.0, 0.0),
    }

    body_mutations = (
        SimpleNamespace(
            x=CartesianVector3(0.0, 0.0, -1.0),
            y=CartesianVector3(0.0, 1.0, 0.0),
            z=CartesianVector3(-1.0, 0.0, 0.0),
        ),
        SimpleNamespace(
            x=CartesianVector3(0.0, 0.0, 1.0),
            y=CartesianVector3(0.0, -1.0, 0.0),
            z=CartesianVector3(-1.0, 0.0, 0.0),
        ),
        SimpleNamespace(
            x=CartesianVector3(0.0, 0.0, 1.0),
            y=CartesianVector3(0.0, 1.0, 0.0),
            z=CartesianVector3(1.0, 0.0, 0.0),
        ),
        SimpleNamespace(
            x=CartesianVector3(0.0, 1.0, 0.0),
            y=CartesianVector3(0.0, 0.0, 1.0),
            z=CartesianVector3(-1.0, 0.0, 0.0),
        ),
    )
    tangent_mutations = (
        SimpleNamespace(
            x=CartesianVector3(0.0, 0.0, -1.0),
            y=CartesianVector3(0.0, -1.0, 0.0),
            z=CartesianVector3(1.0, 0.0, 0.0),
        ),
        SimpleNamespace(
            x=CartesianVector3(0.0, 0.0, 1.0),
            y=CartesianVector3(0.0, 1.0, 0.0),
            z=CartesianVector3(1.0, 0.0, 0.0),
        ),
        SimpleNamespace(
            x=CartesianVector3(0.0, 0.0, 1.0),
            y=CartesianVector3(0.0, -1.0, 0.0),
            z=CartesianVector3(-1.0, 0.0, 0.0),
        ),
        SimpleNamespace(
            x=CartesianVector3(0.0, -1.0, 0.0),
            y=CartesianVector3(0.0, 0.0, 1.0),
            z=CartesianVector3(1.0, 0.0, 0.0),
        ),
    )

    for mutation in body_mutations:
        assert not _basis_matches(mutation, expected_body)
    for mutation in tangent_mutations:
        assert not _basis_matches(mutation, expected_tangent)
####


def test_anchor_mutation_gate_is_explicit() -> None:
    text = (ROOT / "tests/unit/test_verification_baseline.py").read_text(encoding="utf-8")
    normalized_text = " ".join(text.split())
    assert "test_anchor_comparison_rejects_obvious_sign_and_axis_mutations" in normalized_text
    assert "assert not _basis_matches(mutation, expected_body)" in normalized_text
    assert "assert not _basis_matches(mutation, expected_tangent)" in normalized_text
####


def test_inherited_initialization_dependency_semantics_are_explicit() -> None:
    runtime_text = (ROOT / "src/taoryx/runtime/initialization.py").read_text(encoding="utf-8")
    lowering_source = (ROOT / "src/taoryx/runtime/lowering.py").read_text(encoding="utf-8")
    engine_text = (ROOT / "src/taoryx/runtime/engine.py").read_text(encoding="utf-8")
    lowering_test = (ROOT / "tests/unit/test_runtime_lowering.py").read_text(encoding="utf-8")
    problem_overview = (ROOT / "manual/chapters/chapter04/04_problem_overview.tex").read_text(encoding="utf-8")
    normalized_runtime = " ".join(runtime_text.split())
    normalized_lowering_source = " ".join(lowering_source.split())
    normalized_engine_source = " ".join(engine_text.split())
    normalized_lowering = " ".join(lowering_test.split())
    normalized_overview = " ".join(problem_overview.split())

    assert "initial state cannot be both direct and inherited" in normalized_runtime
    assert "unknown inherited initial state" in normalized_runtime
    assert "dependency_segments" in normalized_lowering_source
    assert "activate_dependent_vehicles" in normalized_engine_source
    assert "test_inherited_trajectory_activates_at_source_segment" in normalized_lowering
    assert "A problem begins with an identifier in parentheses" in normalized_overview
    assert "These initial blocks are not associated with a particular trajectory or segment" in normalized_overview
####


def test_initialization_case_coverage_is_source_located() -> None:
    lowering_test = (ROOT / "tests/unit/test_runtime_lowering.py").read_text(encoding="utf-8")
    algorithms_test = (ROOT / "tests/unit/test_runtime_algorithms.py").read_text(encoding="utf-8")
    runtime_text = (ROOT / "src/taoryx/runtime/initialization.py").read_text(encoding="utf-8")
    normalized_lowering = " ".join(lowering_test.split())
    normalized_algorithms = " ".join(algorithms_test.split())
    normalized_runtime = " ".join(runtime_text.split())

    assert "test_inherited_trajectory_activates_at_source_segment" in normalized_lowering
    assert "test_initial_coordinate_models_populate_finite_frame_outputs" in normalized_lowering
    assert "p018_geodetic_initialization" in normalized_lowering
    assert "p019_ecfc_initialization" in normalized_lowering
    assert "test_runtime_graph_rejects_cycles_and_steps_at_boundaries" in normalized_algorithms
    assert "initial state cannot be both direct and inherited" in normalized_runtime
    assert "a direct state or inherited reference is required" in normalized_runtime
    assert "unknown inherited initial state" in normalized_runtime
####


def _vector_components(vector: object) -> tuple[float, float, float]:
    return (vector.x, vector.y, vector.z)
####


def _basis_matches(basis: object, expected: dict[str, tuple[float, float, float]]) -> bool:
    return (
        _vector_components(basis.x) == expected["x"]
        and _vector_components(basis.y) == expected["y"]
        and _vector_components(basis.z) == expected["z"]
    )
####


def _load_from(path: Path) -> dict[str, object]:
    with path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    assert isinstance(document, dict)
    return document
####


def _render_scientific_token(value: str) -> str:
    if "e" not in value and "E" not in value:
        return value
    mantissa, exponent = value.lower().split("e", maxsplit=1)
    return f"{mantissa}\\times10^{{{int(exponent)}}}"
####
