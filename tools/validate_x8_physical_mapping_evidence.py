#!/usr/bin/env python3
"""Record source-resolved evidence for the X8 elevon sign mapping.

The checked-in source package supplies collective/differential elevon
coordinates and signed aerodynamic authority.  Its original package note
kept the physical sign fail-closed because a TensorAeroSpace convention uses
the opposite aileron sign.  The primary X8 source paper now supplies the
missing equation, so this witness records both conventions and the selected
source conversion explicitly.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

from taoryx.airbreathing_control_mapping import x8_mapping_hypotheses, x8_source_mapping

ROOT = Path(__file__).resolve().parents[1]
STATUS = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/cruise_class_uav_skywalker_x8/controls/control_mapping_status.csv"
SOURCE_LQR = ROOT / "verification/generated/x8_table_coordinate_physical_lqr.json"
DEFAULT_OUTPUT = ROOT / "verification/alpha3_x8_physical_mapping/manifest.json"


def _mapping_status() -> dict[str, str]:
    """Read the source package's declared physical-mapping status."""

    with STATUS.open(newline="", encoding="utf-8") as handle:
        rows = {row["item"]: row for row in csv.DictReader(handle)}
    row = rows.get("physical_left_right_sign_mapping")
    if row is None:
        raise RuntimeError("X8 control package has no physical left/right mapping status row")
    return {key: str(value) for key, value in row.items()}
    ####


def _round_trip_error(mapping: Any, collective_deg: float, differential_deg: float) -> float:
    """Return the virtual-coordinate reconstruction error for one hypothesis."""

    left_deg, right_deg = mapping.virtual_to_physical(collective_deg, differential_deg)
    recovered_collective, recovered_differential = mapping.physical_to_virtual(left_deg, right_deg)
    return max(
        abs(recovered_collective - collective_deg),
        abs(recovered_differential - differential_deg),
    )
    ####


def build_artifact() -> dict[str, object]:
    """Build the deterministic mapping witness without choosing a sign."""

    source_status = _mapping_status()
    if source_status["status"] != "verify_before_use":
        raise RuntimeError(f"unexpected original X8 mapping status: {source_status['status']!r}")
    lqr = json.loads(SOURCE_LQR.read_text(encoding="utf-8"))
    effectiveness = lqr["effectiveness"]
    wrench_names = tuple(str(name) for name in effectiveness["wrench_names"])
    effector_names = tuple(str(name) for name in effectiveness["effector_names"])
    matrix = effectiveness["matrix"]
    differential_column = effector_names.index("differential-elevon-deg")
    roll_row = wrench_names.index("moment_x_nm")
    yaw_row = wrench_names.index("moment_z_nm")
    virtual_authority = {
        "positive_differential_command": {
            "moment_x_nm_per_deg": float(matrix[roll_row][differential_column]),
            "moment_z_nm_per_deg": float(matrix[yaw_row][differential_column]),
        },
        "source_coordinate": "differential-elevon-deg",
        "source_plant": "verification/generated/x8_table_coordinate_physical_lqr.json",
    }
    probe_commands = ((0.0, 5.0), (0.0, -5.0), (3.0, 5.0))
    hypotheses: list[dict[str, object]] = []
    for mapping in x8_mapping_hypotheses():
        probes = []
        for collective_deg, differential_deg in probe_commands:
            left_deg, right_deg = mapping.virtual_to_physical(collective_deg, differential_deg)
            probes.append(
                {
                    "collective_deg": collective_deg,
                    "differential_deg": differential_deg,
                    "left_deg": left_deg,
                    "right_deg": right_deg,
                    "round_trip_error_deg": _round_trip_error(mapping, collective_deg, differential_deg),
                }
            )
        hypotheses.append(
            {
                **mapping.as_dict(),
                "probes": probes,
                "invertible": all(probe["round_trip_error_deg"] == 0.0 for probe in probes),
            }
        )
    selected = x8_source_mapping()
    selected_left, selected_right = selected.virtual_to_physical(3.0, 5.0)
    return {
        "schema": "taoryx.x8-physical-elevon-mapping-evidence/v1alpha1",
        "status": "X8_physical_left_right_mapping_resolved_by_source_equation",
        "vehicle": "skywalker_x8",
        "source_status": source_status,
        "resolution": {
            "status": "selected",
            "selected_mapping": selected.as_dict(),
            "equation": "[delta_e, delta_a]^T = 1/2 [[1, 1], [-1, 1]] [delta_er, delta_el]^T",
            "inverse": "delta_el = delta_e + delta_a; delta_er = delta_e - delta_a",
            "source_equation_reference": "Eq. (14)",
            "source_url": "https://doi.org/10.1007/s13272-025-00816-3",
            "source_coordinate_interpretation": "differential_elevon-deg is delta_a from the source X8 deck",
            "selected_probe": {
                "collective_deg": 3.0,
                "differential_deg": 5.0,
                "left_deg": selected_left,
                "right_deg": selected_right,
            },
            "alternate_convention": {
                "meaning": "TensorAeroSpace-style delta_a = (delta_er - delta_el)/2",
                "conversion_required": "negate the source differential coordinate before applying that convention",
            },
        },
        "virtual_coordinate_authority": virtual_authority,
        "hypotheses": hypotheses,
        "discrimination": {
            "source_selects_one_hypothesis": True,
            "selected_mapping_id": selected.mapping_id,
            "reason": "The primary X8 source paper supplies Eq. (14); the alternate convention remains valid only after an explicit differential-coordinate sign conversion.",
            "required_next_evidence": [
                "individual actuator travel/rate and servo-neutral evidence before hardware-level promotion",
                "nonlinear racetrack witness using mapped left/right telemetry",
            ],
        },
        "claim": {
            "proves": "The two possible left/right reconstructions are explicit and invertible, and the source-paper equation selects the left-plus/right-minus mapping for the checked-in X8 source coordinate.",
            "nonclaims": [
                "This resolves the source coordinate mapping, but does not prove servo wiring, hinge-sign calibration, or hardware-level actuator dynamics.",
                "No sign is inferred from trajectory appearance or controller success.",
                "The source-table plant remains a source-coordinate realization until mapped left/right actuator telemetry is exercised end to end.",
            ],
            "promotion_authorized": True,
            "direct_body_moment_injection": False,
            "promotion_blocker": "individual_left_right_actuator_and_end_to_end_racetrack_evidence_pending",
        },
        "reproduction": "PYTHONPATH=src python3 tools/validate_x8_physical_mapping_evidence.py",
    }
    ####


def main() -> int:
    """Write the mapping witness."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(build_artifact(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(json.loads(arguments.output.read_text(encoding="utf-8")), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
