"""Build the Taoryx-authored A320 surrogate-composite DAVE-ML collection."""

from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from dataclasses import asdict
from pathlib import Path
from typing import Any

from taoryx.trajectory import A320OpenAPOperatingPoint, A320Pseudo6DOFModel, A320Pseudo6DOFOperatingPoint
from taoryx.trajectory.daveml_semantic import build_daveml_ir, export_daveml_ir

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "families/a320_openap_jsbsim_pseudo6dof/daveml"


def _variable(parent: ET.Element, identifier: str, units: str, *, input_variable: bool = False, initial: float | None = None) -> None:
    element = ET.SubElement(parent, "variableDef", {"varID": identifier, "units": units})
    if input_variable:
        ET.SubElement(element, "isInput")
    if initial is not None:
        element.set("initialValue", f"{initial:.17g}")
    ####


def _function(parent: ET.Element, input_id: str, output_id: str, x_values: list[float], y_values: list[float]) -> None:
    function = ET.SubElement(parent, "function")
    ET.SubElement(function, "independentVarPts", {"varID": input_id}).text = " ".join(f"{value:.17g}" for value in x_values)
    ET.SubElement(function, "dependentVarPts", {"varID": output_id}).text = " ".join(f"{value:.17g}" for value in y_values)
    ####


def _check(parent: ET.Element, case_id: str, input_id: str, input_value: float, output_id: str, expected: float) -> None:
    shot = ET.SubElement(parent, "staticShot", {"name": case_id})
    inputs = ET.SubElement(shot, "checkInputs")
    if input_id:
        signal = ET.SubElement(inputs, "signal")
        ET.SubElement(signal, "signalID").text = input_id
        ET.SubElement(signal, "signalValue").text = f"{input_value:.17g}"
    outputs = ET.SubElement(shot, "checkOutputs")
    signal = ET.SubElement(outputs, "signal")
    ET.SubElement(signal, "signalID").text = output_id
    ET.SubElement(signal, "signalValue").text = f"{expected:.17g}"
    ET.SubElement(signal, "tol").text = "1e-12"
    ####


def _document(name: str, variables: list[tuple[str, str, bool, float | None]], functions: list[tuple[str, str, list[float], list[float]]], checks: list[tuple[str, str, float, str, float]]) -> bytes:
    root = ET.Element("DAVEfunc", {"name": name})
    for identifier, units, input_variable, initial in variables:
        _variable(root, identifier, units, input_variable=input_variable, initial=initial)
    for input_id, output_id, x_values, y_values in functions:
        _function(root, input_id, output_id, x_values, y_values)
    if checks:
        check_data = ET.SubElement(root, "checkData")
        for check in checks:
            _check(check_data, *check)
    source = ET.tostring(root, encoding="utf-8")
    ir = build_daveml_ir(source, document_id=name)
    return export_daveml_ir(ir)
    ####


def build(output: Path) -> dict[str, Any]:
    """Write deterministic DAVE-ML documents and provenance sidecars."""

    model = A320Pseudo6DOFModel.from_repository(ROOT)
    base = A320OpenAPOperatingPoint(11000.0, 0.78, 60000.0)
    base_result = model.openap.evaluate(base)

    performance_mach = [0.7, 0.78, 0.82]
    performance_drag = [model.openap.evaluate(A320OpenAPOperatingPoint(11000.0, mach, 60000.0)).drag_n for mach in performance_mach]
    performance = _document(
        "a320-pseudo6dof.aerodynamics-performance-openap",
        [("mach", "nd", True, None), ("openap_drag_n", "N", False, None)],
        [("mach", "openap_drag_n", performance_mach, performance_drag)],
        [("reference-cruise-drag", "mach", 0.78, "openap_drag_n", base_result.drag_n)],
    )

    beta_values = [-0.35, 0.0, 0.35]
    beta_coefficients = [model.evaluate(A320Pseudo6DOFOperatingPoint(base, beta_rad=value)).side_force_coefficient for value in beta_values]
    aileron_values = [-0.02, 0.0, 0.02]
    roll_coefficients = [model.evaluate(A320Pseudo6DOFOperatingPoint(base, aileron_rad=value)).roll_moment_coefficient for value in aileron_values]
    elevator_values = [-0.02, 0.0, 0.02]
    pitch_coefficients = [model.evaluate(A320Pseudo6DOFOperatingPoint(base, elevator_rad=value)).pitch_moment_coefficient for value in elevator_values]
    rudder_values = [-0.02, 0.0, 0.02]
    yaw_coefficients = [model.evaluate(A320Pseudo6DOFOperatingPoint(base, rudder_rad=value)).yaw_moment_coefficient for value in rudder_values]
    rotational = _document(
        "a320-pseudo6dof.aerodynamics-rotational-jsbsim",
        [
            ("beta_rad", "rad", True, None),
            ("aileron_rad", "rad", True, None),
            ("elevator_rad", "rad", True, None),
            ("rudder_rad", "rad", True, None),
            ("side_force_coefficient", "nd", False, None),
            ("roll_moment_coefficient", "nd", False, None),
            ("pitch_moment_coefficient", "nd", False, None),
            ("yaw_moment_coefficient", "nd", False, None),
        ],
        [
            ("beta_rad", "side_force_coefficient", beta_values, beta_coefficients),
            ("aileron_rad", "roll_moment_coefficient", aileron_values, roll_coefficients),
            ("elevator_rad", "pitch_moment_coefficient", elevator_values, pitch_coefficients),
            ("rudder_rad", "yaw_moment_coefficient", rudder_values, yaw_coefficients),
        ],
        [
            ("reference-side-force", "beta_rad", 0.0, "side_force_coefficient", beta_coefficients[1]),
            ("reference-roll-pulse", "aileron_rad", 0.01, "roll_moment_coefficient", model.evaluate(A320Pseudo6DOFOperatingPoint(base, aileron_rad=0.01)).roll_moment_coefficient),
            ("reference-pitch-pulse", "elevator_rad", -0.01, "pitch_moment_coefficient", model.evaluate(A320Pseudo6DOFOperatingPoint(base, elevator_rad=-0.01)).pitch_moment_coefficient),
            ("reference-yaw-pulse", "rudder_rad", 0.01, "yaw_moment_coefficient", model.evaluate(A320Pseudo6DOFOperatingPoint(base, rudder_rad=0.01)).yaw_moment_coefficient),
        ],
    )

    throttle_values = [0.0, base_result.required_throttle_ratio, 1.0]
    thrust_values = [model.openap.evaluate(A320OpenAPOperatingPoint(**{**asdict(base), "throttle_ratio": value})).thrust_n for value in throttle_values]
    fuel_values = [model.openap.evaluate(A320OpenAPOperatingPoint(**{**asdict(base), "throttle_ratio": value})).fuel_flow_at_throttle_kg_s for value in throttle_values]
    propulsion = _document(
        "a320-pseudo6dof.propulsion-openap",
        [("throttle_ratio", "nd", True, None), ("openap_thrust_n", "N", False, None), ("openap_fuel_flow_kg_s", "kg/s", False, None)],
        [("throttle_ratio", "openap_thrust_n", throttle_values, thrust_values), ("throttle_ratio", "openap_fuel_flow_kg_s", throttle_values, fuel_values)],
        [("reference-cruise-thrust", "throttle_ratio", base_result.required_throttle_ratio, "openap_thrust_n", base_result.thrust_n)],
    )

    inertia = model.inertia_for_mass(base.mass_kg)
    mass = _document(
        "a320-pseudo6dof.mass-properties-estimated",
        [
            ("mass_kg", "kg", False, base.mass_kg),
            ("inertia_xx_kg_m2", "kg*m^2", False, inertia[0]),
            ("inertia_yy_kg_m2", "kg*m^2", False, inertia[1]),
            ("inertia_zz_kg_m2", "kg*m^2", False, inertia[2]),
        ],
        [],
        [
            ("mass", "", 0.0, "mass_kg", base.mass_kg),
            ("inertia-xx", "", 0.0, "inertia_xx_kg_m2", inertia[0]),
            ("inertia-yy", "", 0.0, "inertia_yy_kg_m2", inertia[1]),
            ("inertia-zz", "", 0.0, "inertia_zz_kg_m2", inertia[2]),
        ],
    )
    control = _document(
        "a320-pseudo6dof.control-mapping",
        [("taoryx_control", "nd", True, None), ("source_control", "nd", False, None)],
        [("taoryx_control", "source_control", [0.0, 1.0], [0.0, 1.0])],
        [("identity-control", "taoryx_control", 0.5, "source_control", 0.5)],
    )
    documents = {
        "aerodynamics-performance-openap.dml": performance,
        "aerodynamics-rotational-jsbsim.dml": rotational,
        "propulsion-openap.dml": propulsion,
        "mass-properties-estimated.dml": mass,
        "control-mapping.dml": control,
    }
    output.mkdir(parents=True, exist_ok=True)
    hashes: dict[str, str] = {}
    for name, payload in documents.items():
        path = output / name
        path.write_bytes(payload)
        hashes[name] = hashlib.sha256(payload).hexdigest()
    (output / "authority-map.json").write_text(json.dumps(model.provenance["authorities"], indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "assumptions.json").write_text(
        json.dumps(
            {
                "qualification_class": "surrogate_composite",
                "source_preserving_export_available": False,
                "canonical_regenerated_export_available": True,
                "source_exact": False,
                "manufacturer_validated": False,
                "dml_export_scope": "representative source-derived channel projections; complete composite runtime remains in the Taoryx binding",
                "disabled_contributions": model.provenance["disabled_contributions"],
                "inertia_policy": model.provenance["inertia_policy"],
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    (output / "provenance.json").write_text(json.dumps({"source": model.provenance, "documents": hashes}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return {"output": str(output), "documents": hashes, "provenance": model.provenance}
    ####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    print(json.dumps(build(args.output_dir.resolve()), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
