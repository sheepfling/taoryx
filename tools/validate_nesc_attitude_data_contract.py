#!/usr/bin/env python3
"""Audit the NESC rocket source package for attitude-control evidence.

This is an onboarding diagnostic, not a controller. It inventories declared
DAVE-ML variables in the pinned aerodynamics, propulsion, and inertia source
members and classifies whether a parent attitude/effectivity path is actually
available. A missing path is a valid fail-closed result: the pseudo tier may
retain a named response law, but it must not be promoted as source-derived
gimbal control.
"""

from __future__ import annotations

import csv
import json
import re
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_nesc_attitude_data_contract"
SOURCE_ROOT = ROOT / "resources/aerospace/daveml/nesc-model-catalog-v1.0"
SOURCE_MEMBERS = {
    "aerodynamics": SOURCE_ROOT / "normalized/nesc-all-models-two-stage-package-twostage-aero/variables.csv",
    "propulsion": SOURCE_ROOT / "normalized/nesc-all-models-two-stage-package-twostage-prop/variables.csv",
    "mass_properties": SOURCE_ROOT / "normalized/nesc-all-models-two-stage-package-twostage-inertia/variables.csv",
}
CONTROL_TERMS = ("control", "actuator", "gimbal", "elevon", "elevator", "rudder", "aileron", "fin")
ATTITUDE_TERMS = ("attitude", "body_rate", "roll", "pitch", "yaw", "moment", "torque")


def _read_variables(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))
    ####


def _matches(row: dict[str, str], terms: tuple[str, ...]) -> bool:
    text = " ".join(row.values()).lower()
    return any(re.search(rf"\b{re.escape(term)}\b", text) is not None for term in terms)
    ####


def _classify(rows: list[dict[str, str]]) -> dict[str, Any]:
    control_rows = [row for row in rows if _matches(row, CONTROL_TERMS)]
    attitude_rows = [row for row in rows if _matches(row, ATTITUDE_TERMS)]
    input_rows = [row for row in rows if "isInput" in row.get("roles", "")]
    output_rows = [row for row in rows if "isOutput" in row.get("roles", "")]
    control_inputs = [row for row in control_rows if "isInput" in row.get("roles", "")]
    gimbal_rows = [row for row in rows if "gimbal" in " ".join(row.values()).lower()]
    return {
        "variable_count": len(rows),
        "input_count": len(input_rows),
        "output_count": len(output_rows),
        "control_candidate_count": len(control_rows),
        "control_input_count": len(control_inputs),
        "gimbal_candidate_count": len(gimbal_rows),
        "attitude_candidate_count": len(attitude_rows),
        "control_inputs": [row.get("variable_id", "") for row in control_inputs],
        "control_candidates": [row.get("variable_id", "") for row in control_rows],
        "attitude_candidates": [row.get("variable_id", "") for row in attitude_rows],
    }
    ####


def _plot(report: dict[str, Any], output: Path) -> None:
    names = list(report["source_members"])
    counts = report["source_members"]
    categories = ("inputs", "control\ncandidates", "gimbal\ncandidates", "attitude\ncandidates")
    values = [[counts[name][key] for name in names] for key in ("input_count", "control_candidate_count", "gimbal_candidate_count", "attitude_candidate_count")]
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), constrained_layout=True)
    axes[0].bar(names, [counts[name]["variable_count"] for name in names], color="#4c78a8")
    axes[0].set_title("Pinned source variable inventory")
    axes[0].set_ylabel("declared variables")
    axes[0].tick_params(axis="x", rotation=20)
    axes[0].grid(axis="y", alpha=0.25)
    width = 0.18
    x = list(range(len(names)))
    colors = ("#59a14f", "#f28e2b", "#e15759", "#b279a2")
    for index, (label, row, color) in enumerate(zip(categories, values, colors, strict=True)):
        offsets = [value + (index - 1.5) * width for value in x]
        axes[1].bar(offsets, row, width=width, label=label, color=color)
    axes[1].set_xticks(x, names)
    axes[1].set_title("Control/effectivity evidence candidates")
    axes[1].set_ylabel("count")
    axes[1].legend(fontsize=8)
    axes[1].grid(axis="y", alpha=0.25)
    fig.suptitle("NESC two-stage rocket attitude-control data contract", fontsize=14)
    fig.savefig(output / "attitude_data_contract_board.png", dpi=160)
    plt.close(fig)
    ####


def build_contract(output: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    """Write the source inventory and fail-closed attitude contract."""

    output.mkdir(parents=True, exist_ok=True)
    members: dict[str, dict[str, Any]] = {}
    all_rows: list[dict[str, str]] = []
    for role, path in SOURCE_MEMBERS.items():
        rows = _read_variables(path)
        summary = _classify(rows)
        summary["relative_path"] = str(path.relative_to(ROOT))
        members[role] = summary
        all_rows.extend(rows)

    all_control_inputs = [row for row in all_rows if _matches(row, CONTROL_TERMS) and "isInput" in row.get("roles", "")]
    all_gimbal = [row for row in all_rows if "gimbal" in " ".join(row.values()).lower()]
    all_attitude_inputs = [row for row in all_rows if _matches(row, ATTITUDE_TERMS) and "isInput" in row.get("roles", "")]
    status = "blocked_parent_attitude_data_unavailable" if not all_control_inputs else "parent_attitude_data_available_for_followup"
    report: dict[str, Any] = {
        "schema": "taoryx.nesc-attitude-data-contract/v1alpha1",
        "status": status,
        "family_id": "reference_nesc_two_stage_rocket",
        "source_package": "nasa-nesc-two-stage-rocket-scenario17",
        "source_members": members,
        "aggregate": {
            "control_input_count": len(all_control_inputs),
            "gimbal_candidate_count": len(all_gimbal),
            "attitude_input_count": len(all_attitude_inputs),
            "declared_control_input_ids": [row.get("variable_id", "") for row in all_control_inputs],
            "declared_gimbal_ids": [row.get("variable_id", "") for row in all_gimbal],
            "declared_attitude_input_ids": [row.get("variable_id", "") for row in all_attitude_inputs],
        },
        "findings": [
            "The aerodynamics source declares alpha and beta as inputs, but they are state/environment inputs rather than control effectors.",
            "The aerodynamics source declares static roll, pitch, and yaw moment coefficient outputs; these are not an effector-effectiveness matrix.",
            "The propulsion source declares axial thrust and mass-flow outputs plus firing flags, but no gimbal angle, thrust-vector, or control-moment input.",
            "The inertia source supplies scheduled mass properties and center-of-mass data, not actuator authority.",
        ],
        "claims": [
            "The pinned source members were inspected through their normalized variable inventories.",
            "The parent package has no declared control-input or gimbal channel from which Taoryx can derive a physical allocator.",
            "The absence is recorded as a promotion blocker rather than repaired with invented effectivity.",
        ],
        "nonclaims": [
            "source-exact attitude response",
            "source-exact gimbal or thrust-vector allocation",
            "closed-loop trim or controlled-flight qualification",
            "that static aerodynamic moment outputs can be treated as control derivatives",
        ],
        "required_input_to_unblock": [
            "declared thrust-vector or gimbal input channels and sign conventions",
            "control-dependent force and moment outputs or an authoritative effectiveness deck",
            "actuator ranges, rates, neutral positions, and stage-dependent availability",
            "attitude history, body-rate history, and event-aligned parent telemetry for comparison",
            "a trim or guidance definition identifying the intended attitude command during powered phases",
        ],
        "next_gate": "Provide authoritative gimbal/effectivity and event-aligned attitude data, then run source-direction, trim, linearization, and bounded allocation probes before promoting the NESC pseudo tier.",
        "reproduction": "PYTHONPATH=src python3 tools/validate_nesc_attitude_data_contract.py",
    }
    (output / "attitude_data_contract.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _plot(report, output)
    manifest = {
        "schema": "taoryx.nesc-attitude-data-contract-manifest/v1alpha1",
        "status": status,
        "artifact": "attitude_data_contract.json",
        "plot": "attitude_data_contract_board.png",
        "reproduction": report["reproduction"],
        "claim_boundary": "Source-data availability diagnostic only; no parent attitude or gimbal qualification.",
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    """Run the NESC attitude-data contract audit."""

    print(json.dumps(build_contract(), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
