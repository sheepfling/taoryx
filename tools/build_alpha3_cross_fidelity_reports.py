#!/usr/bin/env python3
"""Build common cross-fidelity comparison reports for the Alpha 3 catalog."""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INDEX = ROOT / "verification/alpha3_fidelity_ladder/manifest.json"
DEFAULT_OUTPUT = ROOT / "verification/alpha3_cross_fidelity"


def _load(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert isinstance(payload, dict)
    return payload
    ####


def _artifact_reference(path: Path) -> str:
    try:
        return path.relative_to(ROOT).as_posix()
    except ValueError:
        return path.name
    ####


def _evaluation(payload: dict[str, Any]) -> dict[str, Any]:
    evaluation = payload.get("evaluation")
    if isinstance(evaluation, dict):
        return evaluation
    truth = payload.get("truth_evaluation")
    if isinstance(truth, dict):
        return truth
    return {}
    ####


def _objectives(payload: dict[str, Any]) -> list[dict[str, Any]]:
    evaluation = _evaluation(payload)
    results = evaluation.get("results")
    if isinstance(results, list):
        return [item for item in results if isinstance(item, dict)]
    mission = payload.get("mission")
    if isinstance(mission, dict) and isinstance(mission.get("required_objectives"), list):
        return [item for item in mission["required_objectives"] if isinstance(item, dict)]
    return []
    ####


def _objective_delta(left: dict[str, Any], right: dict[str, Any]) -> dict[str, Any]:
    left_time = left.get("truth_time_s")
    right_time = right.get("truth_time_s")
    left_margin = left.get("margin")
    right_margin = right.get("margin")
    delta: dict[str, Any] = {"id": left.get("id"), "left_result": left.get("status", left.get("truth_result")), "right_result": right.get("status", right.get("truth_result"))}
    if isinstance(left_time, (int, float)) and isinstance(right_time, (int, float)):
        delta["truth_time_delta_s"] = float(right_time) - float(left_time)
    if isinstance(left_margin, (int, float)) and isinstance(right_margin, (int, float)):
        delta["margin_delta"] = float(right_margin) - float(left_margin)
    if "sample_id" in left or "sample_id" in right:
        delta["sample_id_pair"] = [left.get("sample_id"), right.get("sample_id")]
    return delta
    ####


def _realization(payload: dict[str, Any]) -> str:
    control_path = payload.get("control_path")
    if isinstance(control_path, dict):
        return str(control_path.get("realization", "unspecified"))
    return str(payload.get("realization", "unspecified"))
    ####


def _comparison(family_id: str, records: list[dict[str, Any]]) -> dict[str, Any]:
    assert len(records) == 2
    left, right = records
    left_payload = _load(ROOT / str(left["artifact"]))
    right_payload = _load(ROOT / str(right["artifact"]))
    left_objectives = _objectives(left_payload)
    right_objectives = _objectives(right_payload)
    left_by_id = {str(item.get("id")): item for item in left_objectives}
    right_by_id = {str(item.get("id")): item for item in right_objectives}
    common_ids = sorted(set(left_by_id) & set(right_by_id))
    unmatched_ids = sorted((set(left_by_id) ^ set(right_by_id)) - {"None"})
    left_eval = _evaluation(left_payload)
    right_eval = _evaluation(right_payload)
    report = {
        "schema": "taoryx.alpha3-cross-fidelity-comparison/v1alpha1",
        "family_id": family_id,
        "reference_fidelity": left["fidelity"],
        "comparison_fidelity": right["fidelity"],
        "reference_artifact": left["artifact"],
        "comparison_artifact": right["artifact"],
        "mission_semantics": {
            "common_objective_ids": common_ids,
            "unmatched_objective_ids": unmatched_ids,
            "objective_sequence_preserved": not unmatched_ids and [str(item.get("id")) for item in left_objectives] == [str(item.get("id")) for item in right_objectives],
        },
        "mission_result": {
            "reference_pass": bool(left_eval.get("mission_pass")),
            "comparison_pass": bool(right_eval.get("mission_pass")),
            "agreement": bool(left_eval.get("mission_pass")) == bool(right_eval.get("mission_pass")),
        },
        "objective_deltas": [_objective_delta(left_by_id[item_id], right_by_id[item_id]) for item_id in common_ids],
        "control_realization": {
            "reference": _realization(left_payload),
            "comparison": _realization(right_payload),
        },
        "interpretation": "The report measures semantic agreement, event-time deltas, and available objective margins. It does not rank fidelities or imply that absent physical effectors exist.",
        "comparison_status": "pass" if not unmatched_ids and bool(left_eval.get("mission_pass")) and bool(right_eval.get("mission_pass")) else "development_review",
    }
    if family_id == "tumbling_body":
        passive_loss_path = ROOT / "verification/alpha3_tumbling_body/fidelity_ladder/comparison.json"
        if passive_loss_path.exists():
            report["passive_loss_report"] = {
                "artifact": _artifact_reference(passive_loss_path),
                "claim": "Measured terminal and projected-area differences between averaged-area translation and native rigid-body reuse; this is not a scalar quality score.",
            }
    return report
    ####


def build_reports(index_path: Path = DEFAULT_INDEX, output: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    """Build one comparison artifact for every paired family in the index."""

    index = _load(index_path)
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for record in index.get("records", []):
        assert isinstance(record, dict)
        grouped[str(record["family_id"])].append(record)
    output.mkdir(parents=True, exist_ok=True)
    reports: list[dict[str, Any]] = []
    for family_id in sorted(grouped):
        records = grouped[family_id]
        if len(records) != 2:
            raise ValueError(f"expected exactly two fidelity records for {family_id}, found {len(records)}")
        report = _comparison(family_id, records)
        path = output / f"{family_id}.json"
        path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        reports.append({"family_id": family_id, "artifact": _artifact_reference(path), "status": report["comparison_status"]})
    manifest: dict[str, Any] = {
        "schema": "taoryx.alpha3-cross-fidelity-manifest/v1alpha1",
        "status": "pass" if all(item["status"] == "pass" for item in reports) else "development_review",
        "family_count": len(reports),
        "reports": reports,
        "interpretation": "Cross-fidelity reports are semantic comparisons. They do not promote development records or claim physical-effector fidelity.",
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest
    ####


def main() -> int:
    """Build and summarize Alpha 3 cross-fidelity reports."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--index", type=Path, default=DEFAULT_INDEX)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    manifest = build_reports(arguments.index, arguments.output)
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0 if manifest["status"] == "pass" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
