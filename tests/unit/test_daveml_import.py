from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

from taoryx.trajectory import (
    DAVEMLCompositeTrimBinding,
    DAVEMLFixedWingDynamicsBinding,
    DAVEMLFixedWingLoadBinding,
    DAVEMLFunctionChannel,
    DAVEMLInertiaBinding,
    DAVEMLTrimBinding,
    load_daveml_atmosphere,
    load_daveml_family_graph,
    load_daveml_family_import,
    load_daveml_function_channel,
    load_daveml_trim_binding,
    load_reference_family_catalog,
)
from taoryx.trim import TrimSpec, solve_trim

ROOT = Path(__file__).resolve().parents[2]
CATALOG_ROOT = Path(
    os.environ.get(
        "TAORYX_DAVEML_CATALOG_ROOT",
        ROOT / "INBOX/taoryx-daveml-nesc-model-catalog-v1.0",
    )
)


def test_qualified_family_sidecars_are_bound_to_the_reference_catalog() -> None:
    """Each promoted family exposes a validated DAVE-ML import boundary."""

    manifests = load_reference_family_catalog(ROOT / "verification/reference_family_catalog.yaml")
    for manifest in manifests:
        assert manifest.daveml_import == "plant/daveml-import.json"
        record = load_daveml_family_import(ROOT / "families" / manifest.family_id / manifest.daveml_import)
        assert record.family_id == manifest.family_id
        assert record.package.sha256 == manifest.source.package_sha256
        assert record.roundtrip.status == "verified"
        assert record.roundtrip.checkdata_status in {"verified", "not_present"}
        assert record.replay.status == "runtime_replay_qualification_passed"
        assert all(document.structural_diff_count == 0 for document in record.package.source_documents)
        assert all(document.numeric_diff_count == 0 for document in record.package.source_documents)


def test_family_graph_binding_verifies_package_and_source_member_before_execution() -> None:
    binding = load_daveml_family_graph(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        role="propulsion",
    )
    values = binding.evaluate(
        {"powerLeverAngle": 0.0, "altitudeMSL": 0.0, "mach": 0.0},
        ("thrustBodyForce_X",),
    )
    assert binding.family_id == "reference_f16_s119"
    assert binding.package_member == "models/propulsion.dml"
    assert binding.unit_for("thrustBodyForce_X") == "lbf"
    assert values["thrustBodyForce_X"] == 1060.0

    trim_binding = DAVEMLTrimBinding(
        graph=binding,
        state_inputs={"altitude_ft": "altitudeMSL"},
        control_inputs={"power_pct": "powerLeverAngle"},
        residual_outputs={"thrust_lbf": "thrustBodyForce_X"},
        fixed_inputs={"mach": 0.0},
    )
    assert trim_binding.evaluate({"altitude_ft": 0.0}, {"power_pct": 0.0}) == {"thrust_lbf": 1060.0}


def test_function_channel_declares_source_inputs_units_and_output() -> None:
    channel = load_daveml_function_channel(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        role="aerodynamics",
        function_id="cxt",
        input_channels={"elevator_deg": "el", "alpha_deg": "alpha"},
    )
    assert isinstance(channel, DAVEMLFunctionChannel)
    assert channel.input_unit("elevator_deg") == "deg"
    assert channel.input_unit("alpha_deg") == "deg"
    assert channel.output_unit() == "nd"
    assert channel.evaluate({"elevator_deg": 0.0, "alpha_deg": 0.0}) == -0.021


def test_hl20_function_channel_preserves_hash_verified_graph_boundary() -> None:
    channel = load_daveml_function_channel(
        ROOT / "families/reference_hl20_mod_k/plant/daveml-import.json",
        role="aerodynamics",
        function_id="CL0A0",
        input_channels={"mach": "XMACH"},
    )
    assert channel.graph.document_sha256 == "b2ec6260ed60d241de250599b269ad35d0b96865b50da5e7f9ef7e04de3844ec"
    assert channel.evaluate({"mach": 1.0}) == pytest.approx(-0.07936)


def test_function_channel_rejects_missing_source_function() -> None:
    with pytest.raises(ValueError, match="not present"):
        load_daveml_function_channel(
            ROOT / "families/reference_f16_s119/plant/daveml-import.json",
            role="aerodynamics",
            function_id="not_a_source_function",
            input_channels={"alpha_deg": "alpha"},
        )


def test_f16_source_pitch_trim_solves_through_daveml_binding() -> None:
    binding = load_daveml_trim_binding(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        role="aerodynamics",
        state_inputs={},
        control_inputs={"elevator_deg": "el"},
        residual_outputs={"pitch_cm": "cm"},
        fixed_inputs={
            "vt": 500.0,
            "alpha": 2.0,
            "beta": 0.0,
            "p": 0.0,
            "q": 0.0,
            "r": 0.0,
            "ail": 0.0,
            "rdr": 0.0,
            "xcg": 0.35,
        },
    )
    spec = TrimSpec(
        state_names=(),
        control_names=("elevator_deg",),
        residual_names=("pitch_cm",),
        state_initial={},
        control_initial={"elevator_deg": 0.0},
        control_lower={"elevator_deg": -24.0},
        control_upper={"elevator_deg": 24.0},
    )
    result = solve_trim(spec, binding.as_evaluator(), max_nfev=100, residual_tolerance=1.0e-10)
    assert result.success
    assert result.controls["elevator_deg"] == pytest.approx(-0.7681660899, abs=1.0e-8)
    assert abs(result.residuals["pitch_cm"]) < 1.0e-9


def test_f16_aero_and_propulsion_bindings_compose_without_losing_provenance() -> None:
    aero = load_daveml_trim_binding(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        role="aerodynamics",
        state_inputs={},
        control_inputs={"elevator_deg": "el"},
        residual_outputs={"pitch_cm": "cm"},
        fixed_inputs={
            "vt": 500.0,
            "alpha": 2.0,
            "beta": 0.0,
            "p": 0.0,
            "q": 0.0,
            "r": 0.0,
            "ail": 0.0,
            "rdr": 0.0,
            "xcg": 0.35,
        },
    )
    propulsion = load_daveml_trim_binding(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        role="propulsion",
        state_inputs={"altitude_ft": "altitudeMSL", "mach": "mach"},
        control_inputs={"power_pct": "powerLeverAngle"},
        residual_outputs={"thrust_lbf": "thrustBodyForce_X"},
    )
    composite = DAVEMLCompositeTrimBinding((aero, propulsion))
    values = composite.evaluate(
        {"altitude_ft": 0.0, "mach": 0.0},
        {"elevator_deg": 0.0, "power_pct": 0.0},
    )
    assert values["pitch_cm"] == pytest.approx(-0.0074)
    assert values["thrust_lbf"] == pytest.approx(1060.0)
    assert aero.graph.document_sha256 != propulsion.graph.document_sha256


def test_f16_fixed_wing_load_binding_reports_explicit_si_channels() -> None:
    aero = load_daveml_trim_binding(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        role="aerodynamics",
        state_inputs={},
        control_inputs={"elevator_deg": "el"},
        residual_outputs={
            "cx": "cx",
            "cy": "cy",
            "cz": "cz",
            "cl": "cl",
            "cm": "cm",
            "cn": "cn",
        },
        fixed_inputs={
            "vt": 500.0,
            "alpha": 2.0,
            "beta": 0.0,
            "p": 0.0,
            "q": 0.0,
            "r": 0.0,
            "ail": 0.0,
            "rdr": 0.0,
            "xcg": 0.35,
        },
    )
    propulsion = load_daveml_trim_binding(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        role="propulsion",
        state_inputs={"altitude_ft": "altitudeMSL", "mach": "mach"},
        control_inputs={"power_pct": "powerLeverAngle"},
        residual_outputs={"thrust_lbf": "thrustBodyForce_X"},
    )
    binding = DAVEMLFixedWingLoadBinding(
        aerodynamics=aero,
        propulsion=propulsion,
        reference_area_m2=27.870912,
        mean_aerodynamic_chord_m=3.450336,
        span_m=9.144,
        dynamic_pressure_pa=1000.0,
    )
    loads = binding.evaluate(
        {"altitude_ft": 0.0, "mach": 0.0},
        {"elevator_deg": 0.0, "power_pct": 0.0},
    )
    scale = 1000.0 * 27.870912
    assert loads["aerodynamic_force_x_n"] == pytest.approx(scale * -0.0142)
    assert loads["aerodynamic_force_z_n"] == pytest.approx(scale * -0.2264)
    assert loads["aerodynamic_moment_y_nm"] == pytest.approx(
        scale * 3.450336 * -0.0074
    )
    assert loads["propulsion_force_x_n"] == pytest.approx(1060.0 * 4.4482216152605)
    assert loads["total_force_x_n"] == pytest.approx(
        loads["aerodynamic_force_x_n"] + loads["propulsion_force_x_n"]
    )


def test_f16_load_binding_can_derive_dynamic_pressure_from_daveml_atmosphere() -> None:
    aero = load_daveml_trim_binding(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        role="aerodynamics",
        state_inputs={},
        control_inputs={"elevator_deg": "el"},
        residual_outputs={"cx": "cx", "cy": "cy", "cz": "cz", "cl": "cl", "cm": "cm", "cn": "cn"},
        fixed_inputs={"vt": 500.0, "alpha": 2.0, "beta": 0.0, "p": 0.0, "q": 0.0, "r": 0.0, "ail": 0.0, "rdr": 0.0, "xcg": 0.35},
    )
    binding = DAVEMLFixedWingLoadBinding(
        aerodynamics=aero,
        reference_area_m2=27.870912,
        mean_aerodynamic_chord_m=3.450336,
        span_m=9.144,
        dynamic_pressure_pa=1.0,
    )
    atmosphere = load_daveml_atmosphere(ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml")
    loads = binding.evaluate_with_atmosphere(
        {},
        {"elevator_deg": 0.0},
        atmosphere,
        geometric_altitude_m=0.0,
        true_airspeed_m_s=152.4,
    )
    expected_q = 0.5 * 1.225 * 152.4**2
    assert loads["aerodynamic_force_x_n"] == pytest.approx(expected_q * 27.870912 * -0.0142)


def test_f16_inertia_binding_converts_source_mass_properties_to_si() -> None:
    graph = load_daveml_family_graph(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        role="mass_properties",
    )
    binding = DAVEMLInertiaBinding(graph)
    values = binding.evaluate()
    assert values["mass_kg"] == pytest.approx(637.1595 * 14.59390294)
    assert values["cg_percent_mac"] == pytest.approx(35.0)
    assert values["inertia_yy_kg_m2"] == pytest.approx(55814.0 * 1.3558179483314004)
    matrix = binding.as_inertia_matrix()
    assert matrix[0][2] == pytest.approx(-982.0 * 1.3558179483314004)


def test_fixed_wing_dynamics_binding_returns_true_newton_euler_derivatives() -> None:
    aero = load_daveml_trim_binding(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        role="aerodynamics",
        state_inputs={},
        control_inputs={"elevator_deg": "el"},
        residual_outputs={"cx": "cx", "cy": "cy", "cz": "cz", "cl": "cl", "cm": "cm", "cn": "cn"},
        fixed_inputs={"vt": 500.0, "alpha": 2.0, "beta": 0.0, "p": 0.0, "q": 0.0, "r": 0.0, "ail": 0.0, "rdr": 0.0, "xcg": 0.35},
    )
    loads = DAVEMLFixedWingLoadBinding(
        aerodynamics=aero,
        reference_area_m2=27.870912,
        mean_aerodynamic_chord_m=3.450336,
        span_m=9.144,
        dynamic_pressure_pa=1000.0,
    )
    dynamics = DAVEMLFixedWingDynamicsBinding(
        loads=loads,
        mass_kg=1000.0,
        inertia_matrix_kg_m2=((100.0, 0.0, 0.0), (0.0, 200.0, 0.0), (0.0, 0.0, 300.0)),
    )
    derivatives = dynamics.evaluate(
        {"u_m_s": 0.0, "v_m_s": 0.0, "w_m_s": 0.0, "p_rad_s": 0.0, "q_rad_s": 0.0, "r_rad_s": 0.0},
        {"elevator_deg": 0.0},
    )
    assert derivatives["u_m_s"] == pytest.approx(1000.0 * 27.870912 * -0.0142 / 1000.0)
    assert derivatives["q_rad_s"] == pytest.approx(1000.0 * 27.870912 * 3.450336 * -0.0074 / 200.0)


@pytest.mark.skipif(not CATALOG_ROOT.is_dir(), reason="local DAVE-ML catalog is external to the repository")
def test_full_catalog_import_report_covers_sources_packages_and_library_targets() -> None:
    """The importer keeps source-ready records distinct from runtime families."""

    report = json.loads((ROOT / "verification/daveml_catalog_import.json").read_text(encoding="utf-8"))
    assert report["status"] == "verified"
    assert report["source_document_count"] == 22
    assert report["qualified_package_count"] == 3
    assert report["model_family_count"] == 9
    assert len(report["sources"]) == 22
    assert len(report["qualified_packages"]) == 3
    assert sum(item["library_status"] == "qualified_reference_family" for item in report["family_library"]) == 3
    assert all(item["roundtrip"]["status"] == "verified" for item in report["qualified_packages"])
    assert all(item["replay"]["status"] == "runtime_replay_qualification_passed" for item in report["qualified_packages"])
