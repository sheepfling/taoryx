"""Compare the executable A320 surrogate channels with authored DAVE-ML."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from taoryx.trajectory import A320OpenAPOperatingPoint, A320Pseudo6DOFModel, A320Pseudo6DOFOperatingPoint, load_daveml_graph

ROOT = Path(__file__).resolve().parents[1]
COLLECTION = ROOT / "families/a320_openap_jsbsim_pseudo6dof/daveml"
DEFAULT_OUTPUT = ROOT / "families/a320_openap_jsbsim_pseudo6dof/validation/runtime-daveml-replay.json"


def _comparison(channel: str, model_value: float, daveml_value: float) -> dict[str, object]:
    difference = abs(model_value - daveml_value)
    return {
        "channel": channel,
        "model_value": model_value,
        "daveml_value": daveml_value,
        "absolute_difference": difference,
        "pass": math.isfinite(model_value) and math.isfinite(daveml_value) and difference <= 1.0e-12,
    }


def replay(collection: Path, output: Path) -> dict[str, object]:
    model = A320Pseudo6DOFModel.from_repository(ROOT)
    openap_point = A320OpenAPOperatingPoint(11000.0, 0.78, 60000.0, throttle_ratio=0.7174806937690654)
    side_point = A320Pseudo6DOFOperatingPoint(openap_point)
    roll_point = A320Pseudo6DOFOperatingPoint(openap_point, aileron_rad=0.01)
    pitch_point = A320Pseudo6DOFOperatingPoint(openap_point, elevator_rad=-0.01)
    yaw_point = A320Pseudo6DOFOperatingPoint(openap_point, rudder_rad=0.01)
    side_result = model.evaluate(side_point)
    roll_result = model.evaluate(roll_point)
    pitch_result = model.evaluate(pitch_point)
    yaw_result = model.evaluate(yaw_point)

    performance_graph = load_daveml_graph((collection / "aerodynamics-performance-openap.dml").read_bytes(), document_id="performance")
    rotational_graph = load_daveml_graph((collection / "aerodynamics-rotational-jsbsim.dml").read_bytes(), document_id="rotational")
    propulsion_graph = load_daveml_graph((collection / "propulsion-openap.dml").read_bytes(), document_id="propulsion")
    mass_graph = load_daveml_graph((collection / "mass-properties-estimated.dml").read_bytes(), document_id="mass")

    performance = performance_graph.evaluate({"mach": openap_point.mach}, ("openap_drag_n",))["openap_drag_n"]
    rotational_inputs = {"beta_rad": 0.0, "aileron_rad": 0.01, "elevator_rad": -0.01, "rudder_rad": 0.01}
    rotational = rotational_graph.evaluate(
        rotational_inputs,
        ("side_force_coefficient", "roll_moment_coefficient", "pitch_moment_coefficient", "yaw_moment_coefficient"),
    )
    propulsion = propulsion_graph.evaluate({"throttle_ratio": openap_point.throttle_ratio or 0.0}, ("openap_thrust_n", "openap_fuel_flow_kg_s"))
    mass = mass_graph.evaluate({}, ("mass_kg", "inertia_xx_kg_m2", "inertia_yy_kg_m2", "inertia_zz_kg_m2"))
    comparisons = [
        _comparison("performance.drag_n", side_result.performance.drag_n, performance),
        _comparison("rotational.side_force_coefficient", side_result.side_force_coefficient, rotational["side_force_coefficient"]),
        _comparison("rotational.roll_moment_coefficient", roll_result.roll_moment_coefficient, rotational["roll_moment_coefficient"]),
        _comparison("rotational.pitch_moment_coefficient", pitch_result.pitch_moment_coefficient, rotational["pitch_moment_coefficient"]),
        _comparison("rotational.yaw_moment_coefficient", yaw_result.yaw_moment_coefficient, rotational["yaw_moment_coefficient"]),
        _comparison("propulsion.thrust_n", side_result.performance.thrust_n, propulsion["openap_thrust_n"]),
        _comparison("propulsion.fuel_flow_kg_s", side_result.performance.fuel_flow_at_throttle_kg_s, propulsion["openap_fuel_flow_kg_s"]),
        _comparison("mass.scalar_schedule", openap_point.mass_kg, mass["mass_kg"]),
        _comparison("mass.cg_and_inertia.ixx", model.inertia_for_mass(openap_point.mass_kg)[0], mass["inertia_xx_kg_m2"]),
        _comparison("mass.cg_and_inertia.iyy", model.inertia_for_mass(openap_point.mass_kg)[1], mass["inertia_yy_kg_m2"]),
        _comparison("mass.cg_and_inertia.izz", model.inertia_for_mass(openap_point.mass_kg)[2], mass["inertia_zz_kg_m2"]),
    ]
    report = {
        "schema_version": "taoryx.a320-pseudo6dof-daveml-runtime-replay/v1",
        "status": "verified" if all(item["pass"] for item in comparisons) else "failed",
        "claim_boundary": "fresh-process numerical replay of authored surrogate channels; not full authoritative flight-dynamics validation",
        "collection": "a320-openap-jsbsim-pseudo6dof",
        "comparisons": comparisons,
        "comparison_count": len(comparisons),
        "provenance": model.provenance,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collection-dir", type=Path, default=COLLECTION)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    report = replay(args.collection_dir.resolve(), args.output.resolve())
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "verified" else 1


if __name__ == "__main__":
    raise SystemExit(main())
