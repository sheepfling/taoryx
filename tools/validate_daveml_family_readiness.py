"""Validate DAVE-ML source, graph, and family-library readiness gates."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import yaml

from taoryx.trajectory import load_daveml_family_graph, load_daveml_family_import, load_reference_family_manifest


def main() -> int:
    """Validate promoted family sidecars against the readiness registry."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--readiness", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    report = validate_readiness(arguments.readiness)
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "verified" else 1


def validate_readiness(readiness_path: str | Path) -> dict[str, Any]:
    """Return deterministic source/runtime and derived-layer readiness evidence."""

    source = Path(readiness_path)
    payload = _read_yaml(source)
    raw_families = payload.get("families")
    if not isinstance(raw_families, list) or not raw_families:
        raise ValueError(f"readiness registry must contain a non-empty families list: {source}")
    family_reports = [_validate_family(source, item) for item in raw_families]
    applicable_pass = all(report["applicable_gates_status"] == "passed" for report in family_reports)
    return {
        "schema_version": "taoryx.daveml-family-readiness-report/v1",
        "status": "verified" if applicable_pass else "failed",
        "readiness_registry": source.as_posix(),
        "claim_boundary": str(payload.get("claim_boundary", "")),
        "families": family_reports,
        "pending_derived_layers": [
            {
                "family_id": report["family_id"],
                "layers": report["pending_derived_layers"],
            }
            for report in family_reports
            if report["pending_derived_layers"]
        ],
    }


def _validate_family(readiness_path: Path, entry: object) -> dict[str, Any]:
    if not isinstance(entry, dict):
        raise ValueError(f"readiness family entry must be a mapping: {readiness_path}")
    family_id = str(entry.get("id", ""))
    manifest_label = str(entry.get("manifest", ""))
    if not family_id or not manifest_label:
        raise ValueError(f"readiness family entry needs id and manifest: {readiness_path}")
    manifest_path = _resolve_repository_path(readiness_path, manifest_label)
    manifest = load_reference_family_manifest(manifest_path)
    if manifest.family_id != family_id:
        raise ValueError(f"readiness family {family_id!r} points to {manifest.family_id!r}")
    if manifest.daveml_import is None:
        raise ValueError(f"family {family_id!r} does not declare a DAVE-ML import sidecar")
    sidecar_path = manifest_path.parent / manifest.daveml_import
    imported = load_daveml_family_import(sidecar_path)
    if imported.family_id != family_id or imported.package.sha256 != manifest.source.package_sha256:
        raise ValueError(f"family {family_id!r} has a stale or mismatched DAVE-ML import sidecar")

    roles = sorted({document.role for document in imported.package.source_documents})
    graph_reports: list[dict[str, object]] = []
    graph_failures: list[str] = []
    for role in roles:
        try:
            binding = load_daveml_family_graph(sidecar_path, role=role)
            graph_reports.append(
                {
                    "role": role,
                    "package_member": binding.package_member,
                    "document_sha256": binding.document_sha256,
                    "variable_count": len(binding.graph.variables),
                    "function_count": len(binding.graph.functions),
                    "status": "loaded_hash_verified",
                }
            )
        except (OSError, ValueError, KeyError) as error:
            graph_failures.append(f"{role}: {error}")

    check_count = sum(document.checkdata_count for document in imported.package.source_documents)
    check_passed = sum(document.checkdata_passed for document in imported.package.source_documents)
    check_failed = sum(document.checkdata_failed for document in imported.package.source_documents)
    check_unsupported = sum(document.checkdata_unsupported for document in imported.package.source_documents)
    check_status = _checkdata_status(check_count, check_failed, check_unsupported)
    declared_check_status = str(entry.get("checkdata", ""))
    check_declaration_match = declared_check_status == check_status
    applicable_gates = {
        "source_hashes": imported.package.sha256 == manifest.source.package_sha256,
        "roundtrip": imported.roundtrip.status == "verified",
        "runtime_replay": imported.replay.status == "runtime_replay_qualification_passed",
        "graph_execution": not graph_failures,
        "checkdata_numeric_evaluation": check_status in {"verified_all_embedded_cases", "no_embedded_cases"},
        "checkdata_declaration": check_declaration_match,
        "layer_declarations": _layer_declarations_match(manifest.layers, entry),
    }
    pending_layers = {
        name: str(entry[name])
        for name in ("trim", "linearization", "tuning", "objectives", "scenarios")
        if str(entry.get(name, "")).startswith(("pending", "source_package", "scenario_", "glide_", "trim_", "trajectory_"))
    }
    return {
        "family_id": family_id,
        "model_id": imported.model_id,
        "source_import_declared": str(entry.get("source_import", "")),
        "checkdata": {
            "declared": declared_check_status,
            "computed": check_status,
            "count": check_count,
            "passed": check_passed,
            "failed": check_failed,
            "unsupported": check_unsupported,
        },
        "graph_bindings": graph_reports,
        "graph_failures": graph_failures,
        "applicable_gates": applicable_gates,
        "applicable_gates_status": "passed" if all(applicable_gates.values()) else "failed",
        "derived_layers": {
            name: str(entry.get(name, ""))
            for name in ("trim", "linearization", "tuning", "objectives", "scenarios")
        },
        "pending_derived_layers": pending_layers,
    }


def _checkdata_status(count: int, failed: int, unsupported: int) -> str:
    if count == 0:
        return "no_embedded_cases"
    if failed == 0 and unsupported == 0:
        return "verified_all_embedded_cases"
    return "failed_or_quarantined"


def _layer_declarations_match(layers: object, entry: dict[str, object]) -> bool:
    """Ensure readiness statuses agree with family manifest layer evidence."""

    if not isinstance(layers, dict):
        return False
    for name in ("trim", "linearization", "tuning", "objectives", "scenarios"):
        layer = layers.get(name)
        if layer is None or str(getattr(layer, "evidence", "")) != str(entry.get(name, "")):
            return False
    return True


def _read_yaml(path: Path) -> dict[str, Any]:
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"readiness registry cannot be read: {path}: {error}") from error
    if not isinstance(payload, dict):
        raise ValueError(f"readiness registry must be a mapping: {path}")
    return payload


def _resolve_repository_path(registry_path: Path, label: str) -> Path:
    """Resolve either a repository-root or registry-relative artifact label."""

    candidates = (
        registry_path.parent / label,
        registry_path.parent.parent / label,
        Path.cwd() / label,
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    return candidates[0].resolve()


if __name__ == "__main__":
    raise SystemExit(main())
