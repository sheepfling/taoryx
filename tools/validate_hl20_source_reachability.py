"""Qualify the HL-20 source aerodynamic reachability integration seam."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

from taoryx.hl20_reachability import run_hl20_source_fidelity_ladder
from taoryx.reachability_aerodynamics import (
    HL20_SOURCE_MODEL_ID,
    HL20DavemlAerodynamics,
    probe_hl20_control_directions,
)
from taoryx.trajectory import load_daveml_trim_binding
from taoryx.trim import TrimSpec, solve_trim

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "artifacts/verification/hl20-source-reachability.json"


def _anchor_report(model: HL20DavemlAerodynamics) -> dict[str, object]:
    alpha_rad = math.radians(5.0)
    speed = model.speed_of_sound_m_s
    loads = model.evaluate((speed * math.cos(alpha_rad), 0.0, -speed * math.sin(alpha_rad)), 0.0)
    coefficients = dict(loads.coefficients)
    expected = {"cl": 0.17524959583333333, "cd": 0.12083715666666667, "cm": 0.007435285000000005}
    residuals = {name: abs(coefficients[name] - value) for name, value in expected.items()}
    return {
        "operating_point": dict(loads.operating_point),
        "expected_coefficients": expected,
        "actual_coefficients": coefficients,
        "absolute_residuals": residuals,
        "maximum_absolute_residual": max(residuals.values()),
        "status": "pass" if max(residuals.values()) <= 1.0e-12 else "fail",
    }


def _tier_report(envelope: Any) -> dict[str, object]:
    # The concrete type is deliberately not imported into this report helper;
    # the public artifact shape is the contract under qualification.
    payload = envelope.as_dict(include_trajectories=True)
    samples = payload["samples"]
    assert isinstance(samples, list)
    source_query_count = 0
    validity_diagnostic_count = 0
    for sample in samples:
        assert isinstance(sample, dict)
        telemetry = sample.get("telemetry", [])
        assert isinstance(telemetry, list)
        for row in telemetry:
            assert isinstance(row, dict)
            source_query_count += int("source_aerodynamics" in row)
            validity_diagnostic_count += int("source_aerodynamic_validity_error" in row)
    return {
        "fidelity": payload["fidelity"],
        "aerodynamic_model_id": payload["study"]["vehicle_parameters"]["aerodynamic_model_id"],
        "source_query_count": source_query_count,
        "validity_diagnostic_count": validity_diagnostic_count,
        "classification_counts": payload["summary"]["classification_counts"],
        "failure_reason_counts": payload["summary"]["failure_reason_counts"],
        "timed_out_query_ids": payload["summary"]["timed_out_query_ids"],
        "source_provenance": {
            "package_sha256": payload["provenance"]["source_package_sha256"],
            "document_sha256": payload["provenance"]["source_document_sha256"],
        },
    }


def _trim_operating_points() -> list[dict[str, object]]:
    """Solve the source pitch-channel trim at every declared Mach anchor."""

    reports: list[dict[str, object]] = []
    for mach in (0.3, 0.5, 1.0, 2.0, 4.0):
        binding = load_daveml_trim_binding(
            ROOT / "families/reference_hl20_mod_k/plant/daveml-import.json",
            role="aerodynamics",
            state_inputs={"alpha_deg": "ALP_UNLIM"},
            control_inputs={},
            residual_outputs={"pitch_cm": "CM"},
            fixed_inputs={
                "BETA": 0.0,
                "XMACH": mach,
                "PB": 0.0,
                "QB": 0.0,
                "RB": 0.0,
                "VRW": mach * 340.294 * 3.280839895013123,
                "H_rwy": 0.0,
                "DBFUL": 0.0,
                "DBFUR": 0.0,
                "DBFLL": 0.0,
                "DBFLR": 0.0,
                "DWFL": 0.0,
                "DWFR": 0.0,
                "DRUD": 0.0,
                "DLG": 0.0,
            },
        )
        spec = TrimSpec(
            state_names=("alpha_deg",),
            control_names=(),
            residual_names=("pitch_cm",),
            state_initial={"alpha_deg": 5.0},
            control_initial={},
            state_lower={"alpha_deg": 0.0},
            state_upper={"alpha_deg": 15.0},
        )
        trim = solve_trim(spec, binding.as_evaluator(), max_nfev=100, residual_tolerance=1.0e-10)
        reports.append(
            {
                "mach": mach,
                "state": dict(trim.state),
                "residuals": dict(trim.residuals),
                "max_residual": trim.max_residual,
                "success": trim.success,
                "iterations": trim.iterations,
                "source_document_sha256": binding.graph.document_sha256,
                "source_package_sha256": binding.graph.package_sha256,
            }
        )
    return reports


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--horizon-s", type=float, default=30.0)
    parser.add_argument("--step-size-s", type=float, default=0.5)
    args = parser.parse_args()
    if args.horizon_s <= 0.0 or args.step_size_s <= 0.0:
        parser.error("--horizon-s and --step-size-s must be positive")
    if args.horizon_s <= 25.0:
        parser.error("--horizon-s must extend beyond the HL-20 release at 25 s so source glide queries are exercised")

    model = HL20DavemlAerodynamics()
    envelopes = run_hl20_source_fidelity_ladder(
        horizon_s=args.horizon_s,
        step_size_s=args.step_size_s,
        spawn_children=False,
    )
    anchor = _anchor_report(model)
    trim_points = _trim_operating_points()
    trim_success_count = sum(int(bool(point["success"])) for point in trim_points)
    report = {
        "schema": "taoryx.hl20-source-reachability-qualification/v1alpha1",
        "status": "source_coupled_with_fail_closed_diagnostics",
        "claim_boundary": "source DAVE-ML coupling and validity diagnostics; not source-exact trajectory or mission capability",
        "source_model_id": HL20_SOURCE_MODEL_ID,
        "source": model.provenance,
        "replay_anchor": anchor,
        "trim_operating_points": trim_points,
        "control_direction_probe": probe_hl20_control_directions(),
        "trim_evidence": {
            "artifact": "verification/daveml_hl20_trim_evidence.json",
            "claim_boundary": "source-bounded pitch-channel trim; not full 6-DOF equilibrium",
            "status": "all_declared_anchors_verified" if trim_success_count == len(trim_points) else "bounded_anchor_matrix_with_failures",
            "successful_anchor_count": trim_success_count,
            "declared_anchor_count": len(trim_points),
        },
        "tiers": [_tier_report(envelope) for envelope in envelopes],
        "open_gap": "Longer-horizon native candidates and surface-excitation variants still require stabilization and trim ownership of source alpha/Mach transitions before native reachability is promoted.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if anchor["status"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
