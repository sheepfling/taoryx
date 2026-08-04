"""Index normalized Product 3 result artifacts without re-evaluating a run.

Family executors own truth evaluation, numerical checks, and qualification.
This module only validates their common ``evaluation.json`` envelope and
publishes a compact discovery projection for completed result directories.
"""

from __future__ import annotations

import hashlib
import json
import math
import shlex
from collections.abc import Mapping
from pathlib import Path
from typing import Literal, cast

from .composition_control_trace import validate_committed_control_trace_against_status
from .composition_graph_evidence import GraphExecutionDispatch, observed_mission_graph_execution
from .composition_resource_ledger import validate_committed_resource_ledger
from .fidelity_contracts import FidelityTier
from .trajectory.evaluation import TrajectoryEvaluation
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_bindings import batch_episode_parity_record, resolve_vehicle_execution_binding

_KNOWN_ARTIFACTS = (
    "execution.json",
    "composition.json",
    "preflight.json",
    "local_screen.json",
    "semantic_action_trace.json",
    "control_provenance.json",
    "mission_graph_execution.json",
    "variant_runtime_evidence.json",
    "objective_report.json",
    "status_trace.json",
    "resource_ledger.json",
    "vehicle_interface.json",
    "convergence_report.json",
    "robustness_report.json",
    "batch_step_parity.json",
    "equation_closure_report.json",
    "fidelity_mapping.json",
    "disagreement_report.json",
    "reproduction.txt",
    "truth_telemetry.csv",
    "telemetry.csv",
)
_LOCAL_DIRECT_WRENCH_SCREEN_SCHEMA = "taoryx.local-direct-wrench-screen/v1alpha1"
_RELEASE_EVIDENCE_ARTIFACTS = (
    "convergence_report.json",
    "robustness_report.json",
    "reproduction.txt",
    "fidelity_mapping.json",
    "disagreement_report.json",
)


def index_composition_results(directory: str | Path) -> dict[str, object]:
    """Return a validated discovery index for evaluation artifacts below ``directory``.

    Invalid JSON or an invalid evaluation schema remains visible as an error
    record and makes the catalog fail. An empty directory is valid discovery
    evidence with ``status: empty`` rather than a fabricated success record.
    """

    root = Path(directory)
    if not root.is_dir():
        raise ValueError(f"result directory does not exist: {root}")
    records: list[dict[str, object]] = []
    errors: list[str] = []
    evaluation_paths = tuple(sorted(root.rglob("evaluation.json")))
    evaluation_directories = {path.parent for path in evaluation_paths}
    for evaluation_path in evaluation_paths:
        relative_path = evaluation_path.relative_to(root)
        try:
            payload = json.loads(evaluation_path.read_text(encoding="utf-8"))
            if not isinstance(payload, Mapping):
                raise ValueError("evaluation artifact is not a JSON object")
            evaluation = TrajectoryEvaluation.model_validate(payload)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            errors.append(f"{relative_path}: invalid evaluation artifact: {error}")
            records.append(
                {
                    "evaluation_path": str(relative_path),
                    "status": "invalid",
                    "error": str(error),
                }
            )
            continue
        output_directory = evaluation_path.parent
        provenance = _composition_provenance(output_directory, evaluation)
        if provenance["status"] == "invalid":
            detail = str(provenance["error"])
            errors.append(f"{relative_path}: invalid composition provenance: {detail}")
            records.append(
                {
                    "evaluation_path": str(relative_path),
                    "output_directory": str(output_directory.relative_to(root)),
                    "status": "invalid",
                    "error": detail,
                    "composition_provenance": provenance,
                }
            )
            continue
        capability_preflight_evidence = _capability_preflight_evidence(output_directory, provenance)
        if capability_preflight_evidence["status"] == "invalid":
            detail = str(capability_preflight_evidence["error"])
            errors.append(f"{relative_path}: invalid concrete capability preflight: {detail}")
            records.append(
                {
                    "evaluation_path": str(relative_path),
                    "output_directory": str(output_directory.relative_to(root)),
                    "status": "invalid",
                    "error": detail,
                    "composition_provenance": provenance,
                    "capability_preflight_evidence": capability_preflight_evidence,
                }
            )
            continue
        interface_provenance = _interface_provenance(output_directory, provenance)
        if interface_provenance["status"] == "invalid":
            detail = str(interface_provenance["error"])
            errors.append(f"{relative_path}: invalid vehicle-interface provenance: {detail}")
            records.append(
                {
                    "evaluation_path": str(relative_path),
                    "output_directory": str(output_directory.relative_to(root)),
                    "status": "invalid",
                    "error": detail,
                    "composition_provenance": provenance,
                    "capability_preflight_evidence": capability_preflight_evidence,
                    "interface_provenance": interface_provenance,
                }
            )
            continue
        action_trace_evidence = _semantic_action_trace_evidence(output_directory, provenance)
        if action_trace_evidence["status"] == "invalid":
            detail = str(action_trace_evidence["error"])
            errors.append(f"{relative_path}: invalid semantic action trace: {detail}")
            records.append(
                {
                    "evaluation_path": str(relative_path),
                    "output_directory": str(output_directory.relative_to(root)),
                    "status": "invalid",
                    "error": detail,
                    "composition_provenance": provenance,
                    "capability_preflight_evidence": capability_preflight_evidence,
                    "interface_provenance": interface_provenance,
                    "semantic_action_trace_evidence": action_trace_evidence,
                }
            )
            continue
        graph_execution_evidence = _graph_execution_evidence(output_directory, provenance)
        if graph_execution_evidence["status"] == "invalid":
            detail = str(graph_execution_evidence["error"])
            errors.append(f"{relative_path}: invalid mission-graph execution evidence: {detail}")
            records.append(
                {
                    "evaluation_path": str(relative_path),
                    "output_directory": str(output_directory.relative_to(root)),
                    "status": "invalid",
                    "error": detail,
                    "composition_provenance": provenance,
                    "capability_preflight_evidence": capability_preflight_evidence,
                    "interface_provenance": interface_provenance,
                    "graph_execution_evidence": graph_execution_evidence,
                }
            )
            continue
        variant_runtime_evidence = _variant_runtime_evidence(output_directory, provenance)
        if variant_runtime_evidence["status"] == "invalid":
            detail = str(variant_runtime_evidence["error"])
            errors.append(f"{relative_path}: invalid variant runtime evidence: {detail}")
            records.append(
                {
                    "evaluation_path": str(relative_path),
                    "output_directory": str(output_directory.relative_to(root)),
                    "status": "invalid",
                    "error": detail,
                    "composition_provenance": provenance,
                    "capability_preflight_evidence": capability_preflight_evidence,
                    "interface_provenance": interface_provenance,
                    "graph_execution_evidence": graph_execution_evidence,
                    "variant_runtime_evidence": variant_runtime_evidence,
                }
            )
            continue
        resource_ledger_evidence = _resource_ledger_evidence(output_directory, provenance)
        if resource_ledger_evidence["status"] == "invalid":
            detail = str(resource_ledger_evidence["error"])
            errors.append(f"{relative_path}: invalid resource ledger: {detail}")
            records.append(
                {
                    "evaluation_path": str(relative_path),
                    "output_directory": str(output_directory.relative_to(root)),
                    "status": "invalid",
                    "error": detail,
                    "composition_provenance": provenance,
                    "capability_preflight_evidence": capability_preflight_evidence,
                    "interface_provenance": interface_provenance,
                    "graph_execution_evidence": graph_execution_evidence,
                    "variant_runtime_evidence": variant_runtime_evidence,
                    "resource_ledger_evidence": resource_ledger_evidence,
                }
            )
            continue
        reproduction_evidence = _reproduction_evidence(output_directory, provenance)
        if reproduction_evidence["status"] == "invalid":
            detail = str(reproduction_evidence["error"])
            errors.append(f"{relative_path}: invalid reproduction record: {detail}")
            records.append(
                {
                    "evaluation_path": str(relative_path),
                    "output_directory": str(output_directory.relative_to(root)),
                    "status": "invalid",
                    "error": detail,
                    "composition_provenance": provenance,
                    "capability_preflight_evidence": capability_preflight_evidence,
                    "interface_provenance": interface_provenance,
                    "graph_execution_evidence": graph_execution_evidence,
                    "variant_runtime_evidence": variant_runtime_evidence,
                    "resource_ledger_evidence": resource_ledger_evidence,
                    "reproduction_evidence": reproduction_evidence,
                }
            )
            continue
        records.append(
            _mission_record(
                root,
                evaluation_path,
                evaluation,
                provenance,
                capability_preflight_evidence,
                interface_provenance,
                action_trace_evidence,
                graph_execution_evidence,
                variant_runtime_evidence,
                resource_ledger_evidence,
                reproduction_evidence,
            )
        )
    local_screen_paths = tuple(sorted(root.rglob("local_screen.json")))
    for screen_path in local_screen_paths:
        if screen_path.parent in evaluation_directories:
            relative_path = screen_path.relative_to(root)
            errors.append(f"{relative_path}: local controller screen conflicts with normalized evaluation in one directory")
            records.append(
                {
                    "screen_path": str(relative_path),
                    "output_directory": str(screen_path.parent.relative_to(root)),
                    "record_kind": "local_controller_screen",
                    "status": "invalid",
                    "error": "local controller screen conflicts with normalized evaluation in one directory",
                }
            )
            continue
        record, screen_error = _local_controller_screen_record(root, screen_path)
        records.append(record)
        if screen_error is not None:
            errors.append(f"{screen_path.relative_to(root)}: {screen_error}")
    status = "fail" if errors else "empty" if not records else "pass"
    return {
        "schema": "taoryx.vehicle-composition-result-catalog/v1alpha1",
        "root_directory": str(root),
        "status": status,
        "result_count": len(records),
        "mission_result_count": len(evaluation_paths),
        "local_controller_screen_count": len(local_screen_paths),
        "valid_result_count": sum(record.get("status") == "valid" for record in records),
        "records": records,
        "errors": errors,
        "claim_boundary": (
            "This catalog validates and indexes existing normalized mission evaluations and explicitly scoped local "
            "controller screens. It does not execute a vehicle, recompute objectives, compare families, or promote qualification."
        ),
    }
    ####


def build_composition_release_catalog(directory: str | Path) -> dict[str, object]:
    """Build a hash-bound release inventory from validated existing packets.

    This is deliberately a packaging index rather than another evaluator.  It
    rejects invalid or empty result trees, hashes the evaluation/screen records
    and their discoverable sidecars, and reports optional release evidence as
    present or missing.  A missing robustness or cross-fidelity artifact is a
    visible coverage gap, never a reason to fabricate a release badge.
    """

    root = Path(directory)
    catalog = index_composition_results(root)
    if catalog["status"] != "pass":
        raise ValueError(f"cannot build release catalog from result index status {catalog['status']!r}")
    raw_records = catalog.get("records")
    if not isinstance(raw_records, list) or not raw_records:
        raise ValueError("cannot build release catalog without at least one valid result packet")

    artifact_paths: set[str] = set()
    packet_records: list[dict[str, object]] = []
    evidence_counts = {name: 0 for name in _RELEASE_EVIDENCE_ARTIFACTS}
    for index, record in enumerate(raw_records):
        if not isinstance(record, Mapping) or record.get("status") != "valid":
            raise ValueError(f"result catalog record {index} is not a valid release packet")
        primary_path = record.get("evaluation_path", record.get("screen_path"))
        if not isinstance(primary_path, str):
            raise ValueError(f"result catalog record {index} has no primary artifact path")
        artifact_paths.add(primary_path)
        sidecars = record.get("artifact_paths")
        if not isinstance(sidecars, Mapping):
            raise ValueError(f"result catalog record {index} has invalid artifact path mapping")
        record_artifacts = {primary_path}
        for name, path in sidecars.items():
            if not isinstance(name, str) or not isinstance(path, str):
                raise ValueError(f"result catalog record {index} has malformed artifact path")
            artifact_paths.add(path)
            record_artifacts.add(path)
        release_evidence: dict[str, dict[str, object]] = {}
        for name in _RELEASE_EVIDENCE_ARTIFACTS:
            if name not in sidecars:
                release_evidence[name] = {"status": "missing"}
                continue
            path = sidecars[name]
            if not isinstance(path, str):
                raise ValueError(f"result catalog record {index} has malformed {name} path")
            evidence_counts[name] += 1
            release_evidence[name] = _validate_release_evidence_artifact(root / path, name)
        packet_records.append(
            {
                "record_kind": record.get("record_kind"),
                "scenario_id": record.get("scenario_id"),
                "screen_id": record.get("screen_id"),
                "outcome": record.get("outcome"),
                "qualification": record.get("qualification"),
                "execution_identity": _release_execution_identity(record),
                "artifact_paths": sorted(record_artifacts),
                "release_evidence": release_evidence,
            }
        )

    digests: dict[str, str] = {}
    for relative_path in sorted(artifact_paths):
        source = root / relative_path
        if not source.is_file():
            raise ValueError(f"release artifact is missing: {relative_path}")
        digests[relative_path] = hashlib.sha256(source.read_bytes()).hexdigest()
    return {
        "schema": "taoryx.vehicle-composition-release-catalog/v1alpha1",
        "status": "ready",
        "result_catalog_schema": catalog["schema"],
        "result_packet_count": len(packet_records),
        "packets": packet_records,
        "artifact_sha256": digests,
        "release_evidence_coverage": {
            name: {"present_packet_count": count, "missing_packet_count": len(packet_records) - count} for name, count in evidence_counts.items()
        },
        "claim_boundary": (
            "This is a hash-bound inventory of already validated result packets. It does not rerun vehicles, "
            "recompute objectives, validate the content of optional release evidence, or promote any result to qualification."
        ),
    }
    ####


def _release_execution_identity(record: Mapping[str, object]) -> dict[str, object]:
    """Project verified composition/interface identity into a release packet.

    The result index already validates these sidecars.  The release catalog
    should retain their public identity so a client can select the exact
    vehicle/fidelity/control path without reopening a family-specific result
    directory.  Missing external provenance remains explicit; this function
    never substitutes a factory name, execution mode, or vehicle identity.
    """

    return {
        "composition": _release_identity_evidence(
            record.get("composition_provenance"),
            (
                "composition_id",
                "composition_identity_sha256",
                "vehicle_id",
                "family_id",
                "physical_family",
                "mission_id",
                "fidelity",
                "runtime_fidelity",
                "control_realization",
                "variant",
            ),
        ),
        "interface": _release_identity_evidence(
            record.get("interface_provenance"),
            (
                "interface_id",
                "fingerprint_sha256",
                "family_id",
                "fidelity",
                "control_realization",
                "execution_bindings",
            ),
        ),
        "capability_preflight": _release_identity_evidence(
            record.get("capability_preflight_evidence"),
            (
                "preflight_status",
                "semantic_translator_id",
                "adapter_id",
                "feasibility",
                "derived_mission_sha256",
            ),
        ),
        "reproduction": _release_identity_evidence(
            record.get("reproduction_evidence"),
            (
                "composition_id",
                "composition_identity_sha256",
                "execution_factory_id",
                "execution_mode",
                "command",
            ),
        ),
    }
    ####


def _release_identity_evidence(raw: object, fields: tuple[str, ...]) -> dict[str, object]:
    """Copy an indexed evidence status and only its already verified fields."""

    if not isinstance(raw, Mapping):
        return {
            "status": "missing",
            "claim_boundary": "The indexed result has no corresponding identity evidence.",
        }
    status = raw.get("status")
    if status not in {"verified", "missing", "unbound"}:
        raise ValueError("release packet identity projection requires validated evidence status")
    result: dict[str, object] = {"status": status}
    if status in {"missing", "unbound"}:
        result["claim_boundary"] = raw.get(
            "claim_boundary",
            "The selected result does not supply this optional identity evidence.",
        )
        return result
    for field in fields:
        if field in raw:
            result[field] = raw[field]
    return result
    ####


def _validate_release_evidence_artifact(path: Path, name: str) -> dict[str, object]:
    """Validate only the shared structural envelope of one optional sidecar.

    Families retain ownership of the numerical content.  Release assembly
    requires just enough common structure to ensure a present sidecar is not a
    blank or malformed placeholder: JSON evidence must identify a status and
    claim boundary, while a reproduction recipe must be nonempty text.
    """

    try:
        if name == "reproduction.txt":
            text = path.read_text(encoding="utf-8").strip()
            if not text:
                raise ValueError("reproduction recipe is empty")
            return {"status": "verified", "format": "text", "line_count": len(text.splitlines())}
        payload = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError("JSON sidecar is not an object")
        status = payload.get("status")
        if not isinstance(status, str) or not status.strip():
            raise ValueError("JSON sidecar has no nonempty status")
        claim = payload.get("claim_boundary", payload.get("claim"))
        if not isinstance(claim, str) or not claim.strip():
            raise ValueError("JSON sidecar has no nonempty claim or claim_boundary")
        return {"status": "verified", "format": "json", "reported_status": status}
    except (OSError, ValueError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid optional release evidence {name!r}: {error}") from error
    ####


def validate_composition_release_catalog(
    directory: str | Path,
    payload: Mapping[str, object],
) -> tuple[str, ...]:
    """Return deterministic release-catalog integrity failures."""

    errors: list[str] = []
    if payload.get("schema") != "taoryx.vehicle-composition-release-catalog/v1alpha1":
        errors.append("unsupported release catalog schema")
        return tuple(errors)
    if payload.get("status") != "ready":
        errors.append("release catalog status must be ready")
    expected = payload.get("artifact_sha256")
    if not isinstance(expected, Mapping):
        errors.append("release catalog artifact_sha256 must be a mapping")
        return tuple(errors)
    try:
        rebuilt = build_composition_release_catalog(directory)
    except ValueError as error:
        return (f"cannot rebuild release catalog: {error}",)
    actual = rebuilt["artifact_sha256"]
    if expected != actual:
        errors.append("release catalog artifact SHA-256 ledger disagrees with current validated packets")
    for field in (
        "result_catalog_schema",
        "result_packet_count",
        "packets",
        "release_evidence_coverage",
        "claim_boundary",
    ):
        if payload.get(field) != rebuilt[field]:
            errors.append(f"release catalog {field} disagrees with current validated packets")
    return tuple(errors)
    ####


def write_composition_release_catalog(
    directory: str | Path,
    output: str | Path,
) -> dict[str, object]:
    """Write one deterministic release catalog after validating source packets."""

    manifest = build_composition_release_catalog(directory)
    destination = Path(output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest
    ####


def _mission_record(
    root: Path,
    evaluation_path: Path,
    evaluation: TrajectoryEvaluation,
    provenance: dict[str, object],
    capability_preflight_evidence: dict[str, object],
    interface_provenance: dict[str, object],
    action_trace_evidence: dict[str, object],
    graph_execution_evidence: dict[str, object],
    variant_runtime_evidence: dict[str, object],
    resource_ledger_evidence: dict[str, object],
    reproduction_evidence: dict[str, object],
) -> dict[str, object]:
    """Return the compact catalog record for one normalized mission result."""

    output_directory = evaluation_path.parent
    return {
        "evaluation_path": str(evaluation_path.relative_to(root)),
        "output_directory": str(output_directory.relative_to(root)),
        "record_kind": "mission_evaluation",
        "status": "valid",
        "scenario_id": evaluation.scenario_id,
        "scenario_contract_sha256": evaluation.scenario_contract_sha256,
        "validity": evaluation.validity,
        "qualification": evaluation.qualification,
        "feasibility": evaluation.feasibility,
        "outcome": evaluation.outcome,
        "gate_statuses": {gate.id: gate.status for gate in evaluation.gates},
        "evaluation_summary": _evaluation_summary(evaluation),
        "composition_provenance": provenance,
        "capability_preflight_evidence": capability_preflight_evidence,
        "interface_provenance": interface_provenance,
        "semantic_action_trace_evidence": action_trace_evidence,
        "graph_execution_evidence": graph_execution_evidence,
        "variant_runtime_evidence": variant_runtime_evidence,
        "resource_ledger_evidence": resource_ledger_evidence,
        "reproduction_evidence": reproduction_evidence,
        "batch_episode_parity": _batch_episode_parity_disposition(provenance),
        "artifact_paths": _artifact_paths(root, output_directory),
        "claim_boundary": evaluation.claim_boundary,
    }
    ####


def _evaluation_summary(evaluation: TrajectoryEvaluation) -> dict[str, object]:
    """Project typed evaluation evidence into a comparison-safe summary.

    The catalog must not recompute terminal geometry, resource use, or a
    scalar score from family telemetry. It can, however, expose the already
    typed required/advisory objective and gate disposition so a client does
    not need to parse a plot or family-owned JSON to distinguish a complete
    result from a partial one.
    """

    required = tuple(metric for metric in evaluation.metrics if metric.severity == "required")
    advisory = tuple(metric for metric in evaluation.metrics if metric.severity == "advisory")
    normalized_errors = tuple(metric.normalized_error for metric in required if metric.normalized_error is not None and math.isfinite(metric.normalized_error))
    gates = tuple(evaluation.gates)
    channels = {
        "requested_controls": evaluation.requested_controls,
        "achieved_controls": evaluation.achieved_controls,
        "resources": evaluation.resources,
        "events": evaluation.events,
    }
    return {
        "required_objectives": {
            "count": len(required),
            "passed": sum(metric.status == "pass" for metric in required),
            "failed": sum(metric.status == "fail" for metric in required),
            "blocked": sum(metric.status == "blocked" for metric in required),
            "worst_normalized_error": max(normalized_errors) if normalized_errors else None,
        },
        "advisory_objective_count": len(advisory),
        "gates": {
            "count": len(gates),
            "passed": sum(gate.status == "pass" for gate in gates),
            "failed": sum(gate.status == "fail" for gate in gates),
            "blocked": sum(gate.status == "blocked" for gate in gates),
            "advisory": sum(gate.status == "advisory" for gate in gates),
        },
        "evidence_channels": {
            kind: {
                "count": len(items),
                "available": sum(item.status == "available" for item in items),
                "unavailable": sum(item.status == "unavailable" for item in items),
                "invalid": sum(item.status == "invalid" for item in items),
            }
            for kind, items in channels.items()
        },
        "claim_boundary": (
            "Summary values are projections of the typed normalized evaluation only. Missing metrics or channels "
            "remain missing; this summary does not infer a terminal state, resource amount, or control result."
        ),
    }
    ####


def _local_controller_screen_record(
    root: Path,
    screen_path: Path,
) -> tuple[dict[str, object], str | None]:
    """Index a direct-wrench recovery screen without turning it into a mission.

    A local screen establishes a bounded controller witness, not route or
    terminal success. It has its own explicit record kind so callers cannot
    accidentally rank it alongside normalized mission outcomes.
    """

    output_directory = screen_path.parent
    relative_path = screen_path.relative_to(root)
    try:
        payload = json.loads(screen_path.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError("local controller screen is not a JSON object")
        if payload.get("schema") != _LOCAL_DIRECT_WRENCH_SCREEN_SCHEMA:
            raise ValueError("unsupported local controller screen schema")
        screen_id = _nonempty_string(payload, "id")
        plant_id = _nonempty_string(payload, "plant_id")
        if payload.get("fidelity") != "rigid_body_6dof_direct_wrench":
            raise ValueError("local controller screen must declare rigid_body_6dof_direct_wrench")
        if payload.get("control_realization") != "direct_wrench":
            raise ValueError("local controller screen must declare direct_wrench control realization")
        if payload.get("physical_effector_allocation") is not False:
            raise ValueError("local controller screen must not claim physical effector allocation")
        evaluation = payload.get("evaluation")
        if not isinstance(evaluation, Mapping) or not isinstance(evaluation.get("mission_pass"), bool):
            raise ValueError("local controller screen has no boolean local recovery result")
        provenance = _local_screen_composition_provenance(output_directory)
        if provenance["status"] != "verified":
            raise ValueError(f"invalid local controller screen composition provenance: {provenance.get('error')}")
        capability_preflight_evidence = _capability_preflight_evidence(output_directory, provenance)
        if capability_preflight_evidence["status"] == "invalid":
            raise ValueError(
                "invalid local controller screen concrete capability preflight: "
                f"{capability_preflight_evidence.get('error')}"
            )
        graph_execution_evidence = _graph_execution_evidence(output_directory, provenance)
        if graph_execution_evidence["status"] == "invalid":
            raise ValueError(f"invalid local controller screen graph evidence: {graph_execution_evidence.get('error')}")
        resource_ledger_evidence = _resource_ledger_evidence(output_directory, provenance)
        if resource_ledger_evidence["status"] == "invalid":
            raise ValueError(f"invalid local controller screen resource ledger: {resource_ledger_evidence.get('error')}")
        reproduction_evidence = _reproduction_evidence(output_directory, provenance)
        if reproduction_evidence["status"] == "invalid":
            raise ValueError(f"invalid local controller screen reproduction record: {reproduction_evidence.get('error')}")
    except (OSError, ValueError, json.JSONDecodeError) as error:
        return (
            {
                "screen_path": str(relative_path),
                "output_directory": str(output_directory.relative_to(root)),
                "record_kind": "local_controller_screen",
                "status": "invalid",
                "error": str(error),
            },
            f"invalid local controller screen: {error}",
        )
    return (
        {
            "screen_path": str(relative_path),
            "output_directory": str(output_directory.relative_to(root)),
            "record_kind": "local_controller_screen",
            "status": "valid",
            "screen_id": screen_id,
            "plant_id": plant_id,
            "fidelity": "rigid_body_6dof_direct_wrench",
            "control_realization": "direct_wrench",
            "physical_effector_allocation": False,
            "outcome": "local_screen_pass" if evaluation["mission_pass"] else "local_screen_failed",
            "composition_provenance": provenance,
            "capability_preflight_evidence": capability_preflight_evidence,
            "graph_execution_evidence": graph_execution_evidence,
            "resource_ledger_evidence": resource_ledger_evidence,
            "reproduction_evidence": reproduction_evidence,
            "batch_episode_parity": _batch_episode_parity_disposition(provenance),
            "artifact_paths": _artifact_paths(root, output_directory),
            "claim_boundary": (
                "Local controller screen only. This outcome is not a mission completion, terminal result, "
                "physical-effector allocation result, or family qualification."
            ),
        },
        None,
    )
    ####


def _nonempty_string(payload: Mapping[str, object], field: str) -> str:
    """Read one mandatory nonempty string from a typed result artifact."""

    value = payload.get(field)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"local controller screen field {field!r} must be a nonempty string")
    return value
    ####


def _reproduction_evidence(
    output_directory: Path,
    composition_provenance: Mapping[str, object],
) -> dict[str, object]:
    """Validate a public batch reproduction record when it can be bound.

    External result packets may preserve an arbitrary reproduction note without
    a Taoryx composition sidecar. That is retained as ``unbound`` rather than
    rejected. A packet with verified compiled-composition provenance is held
    to the stronger public CLI format: its identity and selected batch binding
    must agree exactly with the record it claims to reproduce.
    """

    source = output_directory / "reproduction.txt"
    if not source.is_file():
        return {
            "status": "missing",
            "path": None,
            "claim_boundary": "No reproduction record was supplied with this result packet.",
        }
    try:
        lines = source.read_text(encoding="utf-8").splitlines()
        if not any(line.strip() for line in lines):
            raise ValueError("reproduction record is empty")
        commands = [line for line in lines if line.strip() and not line.startswith("#")]
        if composition_provenance.get("status") != "verified":
            return {
                "status": "unbound",
                "path": str(source),
                "line_count": len(lines),
                "claim_boundary": (
                    "The reproduction record is retained, but no compiled Taoryx composition sidecar is available "
                    "to bind its command or identity."
                ),
            }
        if len(commands) != 1:
            raise ValueError("bound reproduction record must contain exactly one public command")
        command = shlex.split(commands[0])
        if command[:3] != ["taoryx", "vehicle", "run"] or "--output-dir" not in command:
            raise ValueError("bound reproduction record must contain 'taoryx vehicle run ... --output-dir ...'")
        composition_source = output_directory / "composition.json"
        payload = json.loads(composition_source.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError("composition sidecar is not a JSON object")
        composition = CompiledVehicleComposition.model_validate(payload)
        binding = resolve_vehicle_execution_binding(composition, "batch")
        expected_metadata = {
            "composition_id": composition.id,
            "composition_identity_sha256": composition.identity_sha256,
            "execution_factory_id": binding.factory_id,
            "execution_mode": binding.execution_mode,
        }
        for key, expected in expected_metadata.items():
            if f"# {key}: {expected}" not in lines:
                raise ValueError(f"reproduction record is missing matching {key}")
    except (OSError, ValueError, json.JSONDecodeError) as error:
        return {"status": "invalid", "path": str(source), "error": str(error)}
    return {
        "status": "verified",
        "path": str(source),
        "composition_id": composition.id,
        "composition_identity_sha256": composition.identity_sha256,
        "execution_factory_id": binding.factory_id,
        "execution_mode": binding.execution_mode,
        "command": commands[0],
    }
    ####


def _artifact_paths(root: Path, output_directory: Path) -> dict[str, str]:
    """Return known sidecar artifacts relative to one catalog root."""

    return {name: str((output_directory / name).relative_to(root)) for name in _KNOWN_ARTIFACTS if (output_directory / name).is_file()}
    ####


def _composition_provenance(
    output_directory: Path,
    evaluation: TrajectoryEvaluation,
) -> dict[str, object]:
    """Verify a colocated compiled composition when the run provides one.

    External/provider results may legitimately have no Taoryx composition
    sidecar, so absence is reported as ``missing`` rather than converted into
    an invented identity. A present sidecar is stricter: it must parse and
    agree exactly with the normalized evaluation or the result catalog fails.
    """

    source = output_directory / "composition.json"
    if not source.is_file():
        return {"status": "missing", "path": None}
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError("composition sidecar is not a JSON object")
        composition = CompiledVehicleComposition.model_validate(payload)
        if composition.id != evaluation.scenario_id:
            raise ValueError(f"scenario ID mismatch: composition={composition.id!r}, evaluation={evaluation.scenario_id!r}")
        if evaluation.scenario_contract_sha256 != composition.identity_sha256:
            raise ValueError(
                f"composition fingerprint mismatch: composition={composition.identity_sha256!r}, evaluation={evaluation.scenario_contract_sha256!r}"
            )
    except (OSError, ValueError, json.JSONDecodeError) as error:
        return {"status": "invalid", "path": str(source), "error": str(error)}
    return {
        "status": "verified",
        "path": str(source),
        "composition_id": composition.id,
        "composition_identity_sha256": composition.identity_sha256,
        "vehicle_id": composition.vehicle_id,
        "family_id": composition.family_id,
        "physical_family": composition.physical_family,
        "mission_id": composition.mission,
        "fidelity": composition.fidelity,
        "runtime_fidelity": composition.runtime_fidelity,
        "control_realization": composition.control_realization,
        "variant": composition.variant.model_dump(mode="json"),
    }
    ####


def _capability_preflight_evidence(
    output_directory: Path,
    composition_provenance: Mapping[str, object],
) -> dict[str, object]:
    """Validate optional concrete capability evidence without rerunning planning.

    This is a discovery-time integrity check over the preflight sidecar that
    every current Product 3 batch executor emits.  It deliberately does not
    recompute a family capability estimate: a result packet must remain a
    provenance record for the exact planning decision that preceded that run,
    rather than silently being rewritten by current planner code.
    """

    source = output_directory / "preflight.json"
    if not source.is_file():
        return {
            "status": "missing",
            "path": None,
            "claim_boundary": (
                "No preflight artifact was supplied. Capability-planning provenance remains unavailable; "
                "the result catalog does not infer it from a trajectory or evaluation."
            ),
        }
    if composition_provenance.get("status") != "verified":
        return {
            "status": "invalid",
            "path": str(source),
            "error": "concrete capability preflight cannot be bound without verified composition provenance",
        }
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError("preflight artifact is not a JSON object")
        if payload.get("schema") != "taoryx.vehicle-execution-preflight/v1alpha1":
            raise ValueError("unsupported preflight artifact schema")
        expected_identity = {
            "composition_id": composition_provenance.get("composition_id"),
            "composition_identity_sha256": composition_provenance.get("composition_identity_sha256"),
            "vehicle_id": composition_provenance.get("vehicle_id"),
            "family_id": composition_provenance.get("family_id"),
            "fidelity": composition_provenance.get("fidelity"),
        }
        for field, expected in expected_identity.items():
            if payload.get(field) != expected:
                raise ValueError(
                    f"preflight {field} does not match compiled composition: {payload.get(field)!r} != {expected!r}"
                )
        status = payload.get("status")
        if status not in {"translation_ready", "blocked", "not_applicable"}:
            raise ValueError("preflight status is invalid")
        translator_id = payload.get("translator_id")
        if status == "translation_ready" and (not isinstance(translator_id, str) or not translator_id.strip()):
            raise ValueError("translation-ready preflight has no semantic translator ID")
        capability = payload.get("capability_estimate")
        derived_mission = payload.get("derived_mission")
        if status != "translation_ready":
            if capability is not None:
                raise ValueError("non-ready preflight must not expose a concrete capability estimate")
            return {
                "status": "verified",
                "path": str(source),
                "preflight_status": status,
                "semantic_translator_id": translator_id,
                "capability_available": False,
                "claim_boundary": payload.get("claim_boundary"),
            }
        if not isinstance(capability, Mapping):
            raise ValueError("translation-ready preflight omits concrete capability evidence")
        if not isinstance(derived_mission, Mapping):
            raise ValueError("translation-ready preflight has no derived mission object")
        expected_capability = {
            "schema": "taoryx.concrete-capability-preflight/v1alpha1",
            "composition_id": composition_provenance.get("composition_id"),
            "composition_identity_sha256": composition_provenance.get("composition_identity_sha256"),
            "family_id": composition_provenance.get("family_id"),
            "mission_id": composition_provenance.get("mission_id"),
            "fidelity": composition_provenance.get("fidelity"),
            "semantic_translator_id": translator_id,
        }
        for field, expected in expected_capability.items():
            if capability.get(field) != expected:
                raise ValueError(
                    f"concrete capability {field} does not match preflight/composition: "
                    f"{capability.get(field)!r} != {expected!r}"
                )
        adapter_id = capability.get("adapter_id")
        if not isinstance(adapter_id, str) or not adapter_id.strip():
            raise ValueError("concrete capability estimate has no adapter ID")
        feasibility = capability.get("feasibility")
        if feasibility not in {
            "feasible",
            "likely_feasible",
            "unknown",
            "likely_infeasible",
            "certainly_infeasible",
        }:
            raise ValueError("concrete capability estimate has invalid feasibility")
        encoded = json.dumps(
            derived_mission,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
        ).encode("utf-8")
        expected_sha256 = hashlib.sha256(encoded).hexdigest()
        if capability.get("derived_mission_sha256") != expected_sha256:
            raise ValueError("concrete capability derived-mission fingerprint does not match preflight payload")
    except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
        return {"status": "invalid", "path": str(source), "error": str(error)}
    return {
        "status": "verified",
        "path": str(source),
        "preflight_status": status,
        "semantic_translator_id": translator_id,
        "capability_available": True,
        "adapter_id": adapter_id,
        "feasibility": feasibility,
        "derived_mission_sha256": expected_sha256,
        "claim_boundary": capability.get("claim_boundary"),
    }
    ####


def _interface_provenance(
    output_directory: Path,
    composition_provenance: Mapping[str, object],
) -> dict[str, object]:
    """Validate the optional exact interface artifact against a composition.

    Result indexing may include external/provider artifacts which legitimately
    lack both a composition and a Taoryx interface record. A present interface
    sidecar is stricter: it must agree with the verified composition rather
    than allowing a run to advertise the controls/status schema of a different
    family, fidelity, or realization.
    """

    source = output_directory / "vehicle_interface.json"
    if not source.is_file():
        return {"status": "missing", "path": None}
    if composition_provenance.get("status") != "verified":
        return {
            "status": "invalid",
            "path": str(source),
            "error": "vehicle interface cannot be bound without verified composition provenance",
        }
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError("vehicle interface sidecar is not a JSON object")
        if payload.get("validation") != {"status": "pass", "findings": []}:
            raise ValueError("vehicle interface sidecar does not contain a passing validation record")
        interface_id = _nonempty_string(payload, "interface_id")
        fingerprint = _nonempty_string(payload, "fingerprint_sha256")
        family_id = _nonempty_string(payload, "family_id")
        fidelity = _nonempty_string(payload, "fidelity")
        control_realization = _nonempty_string(payload, "control_realization")
        if family_id != composition_provenance.get("family_id"):
            raise ValueError(f"family mismatch: interface={family_id!r}, composition={composition_provenance.get('family_id')!r}")
        if fidelity != composition_provenance.get("fidelity"):
            raise ValueError(f"fidelity mismatch: interface={fidelity!r}, composition={composition_provenance.get('fidelity')!r}")
        if control_realization != composition_provenance.get("control_realization"):
            raise ValueError(
                f"control-realization mismatch: interface={control_realization!r}, composition={composition_provenance.get('control_realization')!r}"
            )
        execution_records = payload.get("execution_records")
        if not isinstance(execution_records, list):
            raise ValueError("vehicle interface sidecar has invalid execution records")
        binding_records = [
            {
                "operation": item.get("operation"),
                "factory_id": item.get("factory_id"),
                "status": item.get("status"),
                "execution_mode": item.get("execution_mode"),
            }
            for item in execution_records
            if isinstance(item, Mapping)
        ]
    except (OSError, ValueError, json.JSONDecodeError) as error:
        return {"status": "invalid", "path": str(source), "error": str(error)}
    return {
        "status": "verified",
        "path": str(source),
        "interface_id": interface_id,
        "fingerprint_sha256": fingerprint,
        "family_id": family_id,
        "fidelity": fidelity,
        "control_realization": control_realization,
        "execution_bindings": binding_records,
    }
    ####


def _semantic_action_trace_evidence(
    output_directory: Path,
    composition_provenance: Mapping[str, object],
) -> dict[str, object]:
    """Validate an optional semantic command trace against its composition.

    A trace makes requested-action or achieved-effector values visible in a
    normalized evaluation. It must therefore be identity-bound like graph and
    variant artifacts; an unrelated trace must not supply a clean run's
    control evidence.
    """

    source = output_directory / "semantic_action_trace.json"
    if not source.is_file() and composition_provenance.get("status") != "verified":
        return {
            "status": "missing",
            "path": None,
            "claim_boundary": "No committed semantic action trace was supplied; command/effect evidence may remain unavailable.",
        }
    if composition_provenance.get("status") != "verified":
        return {
            "status": "invalid",
            "path": str(source),
            "error": "semantic action trace cannot be bound without verified composition provenance",
        }
    composition_path = output_directory / "composition.json"
    try:
        composition_payload = json.loads(composition_path.read_text(encoding="utf-8"))
        if not isinstance(composition_payload, Mapping):
            raise ValueError("composition sidecar is not a JSON object")
        composition = CompiledVehicleComposition.model_validate(composition_payload)
        try:
            disposition = resolve_vehicle_execution_binding(composition, "batch").batch_action_trace
        except ValueError:
            disposition = "not_applicable"
        if not source.is_file():
            return {
                "status": "missing",
                "path": None,
                "declared_disposition": disposition,
                "claim_boundary": "No committed semantic action trace was supplied; command/effect evidence may remain unavailable.",
            }
        if disposition != "emits_committed_interval_trace":
            raise ValueError(f"semantic action trace is present although the selected batch binding declares {disposition!r}")
        payload = json.loads(source.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError("semantic action trace artifact is not a JSON object")
        status_source = output_directory / "status_trace.json"
        if not status_source.is_file():
            raise ValueError("semantic action trace requires colocated committed status trace")
        status_payload = json.loads(status_source.read_text(encoding="utf-8"))
        if not isinstance(status_payload, Mapping):
            raise ValueError("committed status trace artifact is not a JSON object")
        validate_committed_control_trace_against_status(
            composition,
            payload,
            status_trace=status_payload,
        )
        samples = payload.get("samples")
        actions = payload.get("requested_action_channels")
        effectors = payload.get("achieved_effector_channels")
        if not isinstance(samples, list) or not isinstance(actions, list) or not isinstance(effectors, list):
            raise ValueError("validated semantic action trace has malformed summary fields")
    except (OSError, ValueError, json.JSONDecodeError) as error:
        return {"status": "invalid", "path": str(source), "error": str(error)}
    return {
        "status": "verified",
        "path": str(source),
        "sample_count": len(samples),
        "requested_action_channel_count": len(actions),
        "achieved_effector_channel_count": len(effectors),
        "declared_disposition": disposition,
        "claim_boundary": payload.get("claim_boundary"),
    }
    ####


def _graph_execution_evidence(
    output_directory: Path,
    composition_provenance: Mapping[str, object],
) -> dict[str, object]:
    """Validate an optional graph-dispatch artifact against its composition.

    A graph record is execution evidence, not a mission evaluator.  Missing
    evidence remains visible as missing so historic/external results do not
    receive an invented controller claim.  A present record must be internally
    valid and match the colocated compiled composition.
    """

    source = output_directory / "mission_graph_execution.json"
    if not source.is_file():
        return {
            "status": "missing",
            "path": None,
            "claim_boundary": "No graph-dispatch evidence was supplied; controller sequencing is not asserted.",
        }
    if composition_provenance.get("status") != "verified":
        return {
            "status": "invalid",
            "path": str(source),
            "error": "mission-graph execution evidence cannot be bound without verified composition provenance",
        }
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError("mission-graph execution artifact is not a JSON object")
        if payload.get("schema") != "taoryx.mission-graph-execution/v1alpha1":
            raise ValueError("unsupported mission-graph execution schema")
        if payload.get("composition_id") != composition_provenance.get("composition_id"):
            raise ValueError("mission-graph execution composition ID does not match the compiled composition")
        if payload.get("composition_identity_sha256") != composition_provenance.get("composition_identity_sha256"):
            raise ValueError("mission-graph execution composition fingerprint does not match the compiled composition")
        composition = _compiled_composition_for_graph_evidence(output_directory, composition_provenance)
        graph = composition.mission_graph
        if graph is None:
            raise ValueError("compiled composition has no mission graph")
        if payload.get("graph_status") != graph.status:
            raise ValueError("mission-graph execution graph status does not match the compiled composition")
        if payload.get("entry_instance_id") != graph.entry_instance_id:
            raise ValueError("mission-graph execution entry instance does not match the compiled composition")
        expected_instance_ids = payload.get("expected_instance_ids")
        declared_instance_ids = [node.instance_id for node in graph.nodes]
        if expected_instance_ids != declared_instance_ids:
            raise ValueError("mission-graph execution expected instance order does not match the compiled composition")
        observation_status = payload.get("observation_status")
        if observation_status not in {"observed", "unobserved"}:
            raise ValueError("mission-graph execution observation_status is invalid")
        dispatches = payload.get("dispatches")
        if not isinstance(dispatches, list):
            raise ValueError("mission-graph execution dispatches must be a list")
        completed = payload.get("completed_nominal_success_path")
        if completed is not None and not isinstance(completed, bool):
            raise ValueError("mission-graph execution completion must be boolean or null")
        unobserved_reason = payload.get("unobserved_reason")
        if observation_status == "unobserved":
            if dispatches or completed is not None:
                raise ValueError("unobserved mission-graph evidence must not claim dispatches or nominal completion")
            if not isinstance(unobserved_reason, str) or not unobserved_reason.strip():
                raise ValueError("unobserved mission-graph evidence requires a nonempty reason")
        else:
            if not dispatches:
                raise ValueError("observed mission-graph evidence requires at least one dispatch")
            if unobserved_reason is not None:
                raise ValueError("observed mission-graph evidence must not include an unobserved reason")
            reconstructed_dispatches = _reconstruct_graph_dispatches(dispatches)
            expected_evidence = observed_mission_graph_execution(composition, reconstructed_dispatches)
            expected_payload = expected_evidence.as_dict()
            if payload.get("executed_instance_ids") != expected_payload["executed_instance_ids"]:
                raise ValueError("mission-graph execution dispatch order does not match the declared graph")
            if completed != expected_payload["completed_nominal_success_path"]:
                raise ValueError("mission-graph execution completion does not match its declared dispatches")
    except (OSError, ValueError, json.JSONDecodeError) as error:
        return {"status": "invalid", "path": str(source), "error": str(error)}
    return {
        "status": "verified",
        "path": str(source),
        "observation_status": observation_status,
        "graph_status": payload.get("graph_status"),
        "entry_instance_id": payload.get("entry_instance_id"),
        "dispatch_count": len(dispatches),
        "completed_nominal_success_path": completed,
        "claim_boundary": payload.get("claim_boundary"),
    }
    ####


def _compiled_composition_for_graph_evidence(
    output_directory: Path,
    composition_provenance: Mapping[str, object],
) -> CompiledVehicleComposition:
    """Reload the verified composition for graph-edge integrity validation."""

    source = output_directory / "composition.json"
    payload = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError("composition sidecar is not a JSON object")
    composition = CompiledVehicleComposition.model_validate(payload)
    if composition.id != composition_provenance.get("composition_id"):
        raise ValueError("reloaded composition ID does not match verified composition provenance")
    if composition.identity_sha256 != composition_provenance.get("composition_identity_sha256"):
        raise ValueError("reloaded composition fingerprint does not match verified composition provenance")
    return composition
    ####


def _reconstruct_graph_dispatches(
    payload: list[object],
) -> tuple[GraphExecutionDispatch, ...]:
    """Parse graph records before comparing them with declared graph edges."""

    outcomes = {"success", "abort", "resource_limit", "envelope_limit", "timeout"}
    transition_statuses = {
        "transitioned",
        "declared_terminal",
        "no_declared_transition",
        "unhandled_outcome",
    }
    dispatches: list[GraphExecutionDispatch] = []
    for index, item in enumerate(payload):
        if not isinstance(item, Mapping):
            raise ValueError(f"mission-graph dispatch {index} is not a mapping")
        instance_id = item.get("instance_id")
        segment_id = item.get("segment_id")
        outcome = item.get("outcome")
        committed_time_s = item.get("committed_time_s")
        transition_status = item.get("transition_status")
        next_instance_id = item.get("next_instance_id")
        state_transfer = item.get("state_transfer")
        if not isinstance(instance_id, str) or not instance_id:
            raise ValueError(f"mission-graph dispatch {index} has invalid instance ID")
        if not isinstance(segment_id, str) or not segment_id:
            raise ValueError(f"mission-graph dispatch {index} has invalid segment ID")
        if outcome not in outcomes:
            raise ValueError(f"mission-graph dispatch {index} has invalid outcome")
        if isinstance(committed_time_s, bool) or not isinstance(committed_time_s, int | float) or not math.isfinite(float(committed_time_s)) or float(committed_time_s) < 0.0:
            raise ValueError(f"mission-graph dispatch {index} has invalid committed time")
        if transition_status not in transition_statuses:
            raise ValueError(f"mission-graph dispatch {index} has invalid transition status")
        if next_instance_id is not None and (not isinstance(next_instance_id, str) or not next_instance_id):
            raise ValueError(f"mission-graph dispatch {index} has invalid next instance ID")
        if state_transfer is not None and (not isinstance(state_transfer, str) or not state_transfer):
            raise ValueError(f"mission-graph dispatch {index} has invalid state-transfer declaration")
        dispatches.append(
            GraphExecutionDispatch(
                instance_id=instance_id,
                segment_id=segment_id,
                outcome=cast(Literal["success", "abort", "resource_limit", "envelope_limit", "timeout"], outcome),
                committed_time_s=float(committed_time_s),
                transition_status=cast(
                    Literal["transitioned", "declared_terminal", "no_declared_transition", "unhandled_outcome"],
                    transition_status,
                ),
                next_instance_id=next_instance_id,
                state_transfer=state_transfer,
            )
        )
    return tuple(dispatches)
    ####


def _variant_runtime_evidence(
    output_directory: Path,
    composition_provenance: Mapping[str, object],
) -> dict[str, object]:
    """Validate optional selected-variant consumption evidence.

    This validates artifact identity and explicit evidence status. It does not
    recalculate native adapter inputs or resource evolution from telemetry.
    """

    source = output_directory / "variant_runtime_evidence.json"
    if not source.is_file():
        return {
            "status": "missing",
            "path": None,
            "claim_boundary": "No selected-variant runtime-consumption evidence was supplied.",
        }
    if composition_provenance.get("status") != "verified":
        return {
            "status": "invalid",
            "path": str(source),
            "error": "variant runtime evidence cannot be bound without verified composition provenance",
        }
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError("variant runtime evidence is not a JSON object")
        if payload.get("schema") != "taoryx.variant-runtime-evidence/v1alpha1":
            raise ValueError("unsupported variant runtime evidence schema")
        if payload.get("composition_id") != composition_provenance.get("composition_id"):
            raise ValueError("variant runtime evidence composition ID does not match the compiled composition")
        if payload.get("composition_identity_sha256") != composition_provenance.get("composition_identity_sha256"):
            raise ValueError("variant runtime evidence composition fingerprint does not match the compiled composition")
        status = payload.get("status")
        if status not in {"pass", "fail", "not_applicable"}:
            raise ValueError("variant runtime evidence status is invalid")
        bindings = payload.get("bindings")
        if not isinstance(bindings, list):
            raise ValueError("variant runtime evidence bindings must be a list")
        if status == "not_applicable" and bindings:
            raise ValueError("not-applicable variant runtime evidence must not contain bindings")
        if status in {"pass", "fail"} and not bindings:
            raise ValueError("selected-variant runtime evidence requires at least one binding")
        binding_statuses: list[str] = []
        for index, binding in enumerate(bindings):
            if not isinstance(binding, Mapping):
                raise ValueError(f"variant runtime evidence binding {index} is not a mapping")
            binding_status = binding.get("status")
            if binding_status not in {"pass", "fail"}:
                raise ValueError(f"variant runtime evidence binding {index} has invalid status")
            if not isinstance(binding.get("consumed_native_input_match"), bool):
                raise ValueError(f"variant runtime evidence binding {index} omits native-input match status")
            channels = binding.get("status_channels")
            if not isinstance(channels, list):
                raise ValueError(f"variant runtime evidence binding {index} has invalid status channels")
            binding_statuses.append(binding_status)
        if status == "pass" and any(item != "pass" for item in binding_statuses):
            raise ValueError("passing variant runtime evidence contains a failed binding")
        if status == "fail" and all(item == "pass" for item in binding_statuses):
            raise ValueError("failing variant runtime evidence contains no failed binding")
    except (OSError, ValueError, json.JSONDecodeError) as error:
        return {"status": "invalid", "path": str(source), "error": str(error)}
    return {
        "status": "verified",
        "path": str(source),
        "evidence_status": status,
        "binding_count": len(bindings),
        "claim_boundary": payload.get("claim_boundary"),
    }
    ####


def _resource_ledger_evidence(
    output_directory: Path,
    composition_provenance: Mapping[str, object],
) -> dict[str, object]:
    """Validate an optional committed resource ledger against source status truth."""

    source = output_directory / "resource_ledger.json"
    if not source.is_file():
        return {
            "status": "missing",
            "path": None,
            "claim_boundary": "No committed resource ledger was supplied; resource history remains unavailable.",
        }
    if composition_provenance.get("status") != "verified":
        return {
            "status": "invalid",
            "path": str(source),
            "error": "resource ledger cannot be bound without verified composition provenance",
        }
    try:
        composition_source = output_directory / "composition.json"
        status_source = output_directory / "status_trace.json"
        if not status_source.is_file():
            raise ValueError("resource ledger requires colocated status_trace.json")
        composition_payload = json.loads(composition_source.read_text(encoding="utf-8"))
        status_payload = json.loads(status_source.read_text(encoding="utf-8"))
        ledger_payload = json.loads(source.read_text(encoding="utf-8"))
        if not isinstance(composition_payload, Mapping):
            raise ValueError("composition sidecar is not a JSON object")
        if not isinstance(status_payload, Mapping) or not isinstance(ledger_payload, Mapping):
            raise ValueError("resource ledger or status trace is not a JSON object")
        composition = CompiledVehicleComposition.model_validate(composition_payload)
        validate_committed_resource_ledger(composition, ledger_payload, status_trace=status_payload)
        summaries = ledger_payload.get("summaries")
        samples = ledger_payload.get("samples")
        if not isinstance(summaries, list) or not isinstance(samples, list):
            raise ValueError("validated resource ledger has malformed summary fields")
    except (OSError, ValueError, json.JSONDecodeError) as error:
        return {"status": "invalid", "path": str(source), "error": str(error)}
    return {
        "status": "verified",
        "path": str(source),
        "resource_count": len(summaries),
        "sample_count": len(samples),
        "claim_boundary": ledger_payload.get("claim_boundary"),
    }
    ####


def _batch_episode_parity_disposition(composition_provenance: Mapping[str, object]) -> dict[str, object]:
    """Expose registered parity evidence without inferring it from one result."""

    if composition_provenance.get("status") != "verified":
        return {
            "availability": "not_available",
            "reason": "batch/episode parity cannot be resolved without verified composition provenance",
            "claim_boundary": "No parity is claimed from an unbound result artifact.",
        }
    family_id = composition_provenance.get("family_id")
    mission_id = composition_provenance.get("mission_id")
    fidelity = composition_provenance.get("fidelity")
    if not isinstance(family_id, str) or not isinstance(mission_id, str) or not isinstance(fidelity, str):
        return {
            "availability": "not_available",
            "reason": "verified composition provenance omitted one parity lookup identity",
            "claim_boundary": "No parity is claimed from incomplete composition provenance.",
        }
    return batch_episode_parity_record(family_id, mission_id, cast(FidelityTier, fidelity))
    ####


def _local_screen_composition_provenance(output_directory: Path) -> dict[str, object]:
    """Verify that a local screen remains bound to its direct-wrench composition."""

    source = output_directory / "composition.json"
    if not source.is_file():
        return {
            "status": "invalid",
            "path": None,
            "error": "local controller screens require a colocated compiled composition",
        }
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError("composition sidecar is not a JSON object")
        composition = CompiledVehicleComposition.model_validate(payload)
        if composition.fidelity != "rigid_body_6dof_direct_wrench":
            raise ValueError("local controller screen composition has the wrong fidelity")
        binding = resolve_vehicle_execution_binding(composition, "batch")
        if binding is None or binding.factory_id != "local_direct_wrench_screen.v1":
            raise ValueError("local controller screen composition has no matching batch screen binding")
    except (OSError, ValueError, json.JSONDecodeError) as error:
        return {"status": "invalid", "path": str(source), "error": str(error)}
    return {
        "status": "verified",
        "path": str(source),
        "composition_id": composition.id,
        "composition_identity_sha256": composition.identity_sha256,
        "vehicle_id": composition.vehicle_id,
        "family_id": composition.family_id,
        "mission_id": composition.mission,
        "fidelity": composition.fidelity,
    }
    ####


__all__ = [
    "build_composition_release_catalog",
    "index_composition_results",
    "validate_composition_release_catalog",
    "write_composition_release_catalog",
]
