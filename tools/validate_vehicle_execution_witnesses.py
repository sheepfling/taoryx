#!/usr/bin/env python3
"""Validate checked-in composition witnesses for every runnable endpoint."""

from __future__ import annotations

import json
import sys
from collections.abc import Mapping
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.vehicle_execution_parity_witnesses import validate_vehicle_execution_parity_witnesses

from taoryx.plugins import discover_plugins
from taoryx.vehicle_execution_witnesses import validate_vehicle_execution_witnesses, validate_vehicle_variant_witnesses


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--execute-batch",
        action="store_true",
        help="run selected batch witnesses through the public compose-to-run entry point",
    )
    parser.add_argument(
        "--results-dir",
        type=Path,
        help="retain generated batch packets in an empty directory and write an aggregate release catalog",
    )
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument(
        "--family",
        action="append",
        metavar="FAMILY_ID",
        help="limit validation to one vehicle family; repeat to select several families",
    )
    selection.add_argument(
        "--witness",
        action="append",
        metavar="WITNESS_ID",
        help="limit validation to one exact endpoint witness; repeat to select several",
    )
    parser.add_argument(
        "--plugin",
        action="append",
        metavar="PLUGIN_ID",
        help=(
            "use only the named source-tree plug-in(s) for executable validation; "
            "repeat to select an intentional dependency set"
        ),
    )
    parser.add_argument(
        "--variants-only",
        action="store_true",
        help="validate only runtime-bound variant composition witnesses without opening endpoints",
    )
    parser.add_argument(
        "--execute-parity",
        action="store_true",
        help="replay every checked-in trace for pairs with registered batch/episode parity evidence",
    )
    parser.add_argument(
        "--parity-only",
        action="store_true",
        help="replay selected family parity witnesses without first validating every family endpoint witness",
    )
    parser.add_argument(
        "--summary",
        action="store_true",
        help="print only scoped counts and status instead of the complete witness evidence payload",
    )
    arguments = parser.parse_args()
    if arguments.variants_only and (
        arguments.execute_batch
        or arguments.execute_parity
        or arguments.parity_only
        or arguments.witness
        or arguments.results_dir
    ):
        parser.error("--variants-only cannot be combined with --execute-batch, --execute-parity, --results-dir, or --witness")
    if arguments.results_dir is not None and not arguments.execute_batch:
        parser.error("--results-dir requires --execute-batch")
    if arguments.execute_parity and arguments.witness:
        parser.error("--execute-parity cannot be combined with --witness; use --family for a scoped parity run")
    if arguments.parity_only:
        if arguments.witness or arguments.variants_only or arguments.execute_batch or arguments.results_dir:
            parser.error("--parity-only cannot be combined with --witness, --variants-only, --execute-batch, or --results-dir")
        if not arguments.family:
            parser.error("--parity-only requires at least one --family")
    plugins = None
    if arguments.plugin:
        plugins = discover_plugins(include_external=False, selected=tuple(arguments.plugin))
    if arguments.parity_only:
        parity = validate_vehicle_execution_parity_witnesses(family_ids=arguments.family, plugins=plugins)
        parity_errors = parity.get("errors")
        if not isinstance(parity_errors, list) or not all(isinstance(item, str) for item in parity_errors):
            raise ValueError("parity witness validation returned invalid errors")
        report: dict[str, object] = {
            "schema": "taoryx.vehicle-execution-witness-report/v1alpha1",
            "status": parity["status"],
            "family_filter": parity["family_filter"],
            "witness_count": 0,
            "variant_witness_count": 0,
            "graph_extension_witness_count": 0,
            "errors": parity_errors,
            "batch_episode_parity_witnesses": parity,
            "claim_boundary": (
                "This replays only registered batch/episode parity evidence for the selected families. "
                "It does not validate every endpoint witness or qualify the vehicle."
            ),
        }
    elif arguments.variants_only:
        report = validate_vehicle_variant_witnesses(family_ids=arguments.family, plugins=plugins)
    else:
        report = validate_vehicle_execution_witnesses(
            execute_batch=arguments.execute_batch,
            family_ids=arguments.family,
            witness_ids=arguments.witness,
            retained_results_directory=arguments.results_dir,
            plugins=plugins,
        )
        if arguments.execute_parity:
            parity = validate_vehicle_execution_parity_witnesses(family_ids=arguments.family, plugins=plugins)
            report["batch_episode_parity_witnesses"] = parity
            if parity["status"] != "pass":
                report["status"] = "fail"
    if plugins is not None:
        report["plugin_scope"] = {
            "plugin_ids": [item.id for item in plugins.plugins],
            "plugin_revisions": [item.public_dict() for item in plugins.plugin_revisions],
        }
    if arguments.summary:
        parity_payload = report.get("batch_episode_parity_witnesses")
        parity_summary = None
        if isinstance(parity_payload, Mapping):
            parity_summary = {
                "status": parity_payload.get("status"),
                "registered_binding_count": parity_payload.get("registered_binding_count"),
                "witness_count": parity_payload.get("witness_count"),
            }
        errors = report.get("errors")
        payload = {
            "schema": report.get("schema"),
            "status": report.get("status"),
            "family_filter": report.get("family_filter"),
            "witness_count": report.get("witness_count"),
            "variant_witness_count": report.get("variant_witness_count"),
            "graph_extension_witness_count": report.get("graph_extension_witness_count"),
            "error_count": len(errors) if isinstance(errors, list) else 0,
            "batch_episode_parity": parity_summary,
            "plugin_scope": report.get("plugin_scope"),
        }
    else:
        payload = report
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if report["status"] == "pass" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
