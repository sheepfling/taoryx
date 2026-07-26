"""Build canonical DAVE-ML IR and compare a fresh canonical re-import."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile

from taoryx.trajectory import (
    build_daveml_ir,
    compare_daveml_ir,
    compare_daveml_numeric,
    evaluate_daveml_checkdata,
    evaluate_daveml_vector_checkdata,
    export_daveml_ir,
    read_collection_archive,
)


def main() -> int:
    """Run canonical IR/export/re-import for every DAVE-ML source member."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifact", type=Path, nargs="?", help="verified .txcollection or .txair archive")
    parser.add_argument("--catalog-root", type=Path, help="INBOX DAVE-ML catalog root to cycle")
    parser.add_argument("--output-dir", type=Path, required=True, help="derived report directory")
    parser.add_argument("--summary-output", type=Path, help="optional compact copy of the catalog report")
    arguments = parser.parse_args()
    if (arguments.artifact is None) == (arguments.catalog_root is None):
        parser.error("provide exactly one of ARTIFACT or --catalog-root")
    if arguments.catalog_root is not None:
        return _run_catalog(arguments.catalog_root, arguments.output_dir, arguments.summary_output)
    if arguments.artifact.suffix.casefold() == ".txair":
        return _run_package(arguments.artifact, arguments.output_dir)
    contents = read_collection_archive(arguments.artifact)
    output = arguments.output_dir
    (output / "canonical").mkdir(parents=True, exist_ok=True)
    (output / "exported").mkdir(parents=True, exist_ok=True)
    documents: list[dict[str, object]] = []
    all_diffs: list[dict[str, object]] = []
    all_numeric_diffs: list[dict[str, object]] = []
    for document in contents.manifest.source_documents:
        source_member = _resolve_source_member(document.package_member, document.source_path, contents.files)
        payload = contents.files[source_member]
        if not source_member.casefold().endswith((".dml", ".xml")):
            continue
        ir = build_daveml_ir(payload, document_id=document.document_id)
        exported = export_daveml_ir(ir)
        fresh = build_daveml_ir(exported, document_id=document.document_id)
        diffs = compare_daveml_ir(ir, fresh)
        numeric_diffs = compare_daveml_numeric(ir, fresh)
        check_results = evaluate_daveml_checkdata(payload)
        vector_check_results = evaluate_daveml_vector_checkdata(payload)
        stem = Path(source_member).stem
        (output / "canonical" / f"{stem}.ir.json").write_bytes(ir.canonical_json())
        (output / "exported" / f"{stem}.dml").write_bytes(exported)
        for diff in diffs:
            all_diffs.append({"document_id": document.document_id, **diff.to_dict()})
        for diff in numeric_diffs:
            all_numeric_diffs.append({"document_id": document.document_id, **diff.to_dict()})
        documents.append(
            {
                "document_id": document.document_id,
                "source_member": source_member,
                "source_sha256": ir.source_sha256,
                "exported_sha256": _sha256(exported),
                "opaque_paths": list(ir.opaque_paths),
                "reference_summary": _reference_summary(ir),
                "structural_diff_count": len(diffs),
                "numeric_diff_count": len(numeric_diffs),
                "checkdata": _checkdata_summary(check_results, vector_check_results),
            }
        )
    _write_export_manifest(output, documents)
    report = {
        "status": "verified" if not all_diffs and not all_numeric_diffs else "failed",
        "checkdata_status": _documents_checkdata_status(documents),
        "collection": contents.manifest.collection_id,
        "schema_version": "taoryx.daveml-roundtrip/v1",
        "documents": documents,
        "structural_diff": all_diffs,
        "numeric_diff": all_numeric_diffs,
        "limitations": [
            "Canonical export is generated from the Taoryx semantic tree IR.",
            "Source bytes remain the immutable authority for L0/L1 identity.",
            "Numeric graph evaluation and runtime equivalence remain separate gates.",
        ],
    }
    (output / "roundtrip-report.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if not all_diffs and not all_numeric_diffs and _documents_checkdata_status(documents) != "failed" else 1
    ####


def _write_export_manifest(output: Path, documents: list[dict[str, object]]) -> None:
    """Write the canonical export provenance sidecar required by the plan."""

    entries = []
    for document in documents:
        entries.append(
            {
                "document_id": document.get("document_id", document.get("source")),
                "source_member": document.get("source_member", document.get("source")),
                "source_sha256": document.get("source_sha256"),
                "ir_sha256": document.get("ir_sha256"),
                "exported_sha256": document.get("exported_sha256"),
                "opaque_paths": document.get("opaque_paths", []),
            }
        )
    manifest = {
        "schema_version": "taoryx.daveml-export-manifest/v1",
        "exporter": "taoryx.trajectory.daveml_semantic.export_daveml_ir",
        "documents": sorted(entries, key=lambda item: str(item["document_id"])),
    }
    target = output / "canonical" / "export-manifest.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _resolve_source_member(package_member: str, source_path: str, files: dict[str, bytes]) -> str:
    """Resolve a source document's collection member."""

    candidates = (
        f"source/{package_member.removeprefix('models/')}",
        f"source/{Path(package_member).name}",
        f"source/{Path(source_path).name}",
    )
    for candidate in candidates:
        if candidate in files:
            return candidate
    raise ValueError(f"source payload is absent from collection: {package_member}")
    ####


def _sha256(payload: bytes) -> str:
    """Return the SHA-256 digest of one exported payload."""

    return hashlib.sha256(payload).hexdigest()
    ####


def _checkdata_summary(results: object, vector_results: object = ()) -> dict[str, object]:
    """Summarize evaluator-backed checkData evidence without hiding quarantine."""

    values = list(results) if isinstance(results, tuple) else []
    vector_values = list(vector_results) if isinstance(vector_results, tuple) else []
    statuses = [str(getattr(result, "status", "unknown")) for result in values + vector_values]
    status = (
        "not_present"
        if not statuses
        else "failed"
        if "failed" in statuses
        else "verified_with_quarantine"
        if "unsupported" in statuses
        else "verified"
    )
    return {
        "count": len(values),
        "passed": statuses.count("passed"),
        "failed": statuses.count("failed"),
        "unsupported": statuses.count("unsupported"),
        "status": status,
        "results": [result.to_dict() for result in values],
        "vector_count": len(vector_values),
        "vector_passed": sum(getattr(result, "status", "") == "passed" for result in vector_values),
        "vector_failed": sum(getattr(result, "status", "") == "failed" for result in vector_values),
        "vector_unsupported": sum(getattr(result, "status", "") == "unsupported" for result in vector_values),
        "vector_results": [result.to_dict() for result in vector_values],
    }
    ####


def _documents_checkdata_status(documents: list[dict[str, object]]) -> str:
    """Aggregate checkData status without weakening structural round-trip status."""

    summaries = [item.get("checkdata") for item in documents]
    statuses = [str(item.get("status", "unknown")) for item in summaries if isinstance(item, dict)]
    if "failed" in statuses:
        return "failed"
    if "verified_with_quarantine" in statuses:
        return "verified_with_quarantine"
    if "verified" in statuses:
        return "verified"
    return "not_present"
    ####


def _reference_summary(ir: object) -> dict[str, int]:
    """Summarize typed function-reference dispositions from one IR."""

    semantic = getattr(ir, "semantic", {})
    functions = semantic.get("functions", ()) if isinstance(semantic, dict) else ()
    statuses: list[str] = []
    for function in functions if isinstance(functions, list) else ():
        if not isinstance(function, dict):
            continue
        references = function.get("references", ())
        for reference in references if isinstance(references, list) else ():
            if isinstance(reference, dict):
                statuses.append(str(reference.get("status", "unknown")))
    return {status: statuses.count(status) for status in sorted(set(statuses))}
    ####


def _run_package(package: Path, output: Path) -> int:
    """Run the same gate over DAVE-ML source members in a .txair package."""

    with ZipFile(package) as archive:
        files = {name: archive.read(name) for name in archive.namelist() if not name.endswith("/")}
    sources = {name: payload for name, payload in files.items() if name.casefold().endswith((".dml", ".xml"))}
    if not sources:
        raise ValueError(f"package has no DAVE-ML source members: {package}")
    output.joinpath("canonical").mkdir(parents=True, exist_ok=True)
    output.joinpath("exported").mkdir(parents=True, exist_ok=True)
    documents: list[dict[str, object]] = []
    structural: list[dict[str, object]] = []
    numeric: list[dict[str, object]] = []
    for source_member, payload in sorted(sources.items()):
        document_id = source_member
        ir = build_daveml_ir(payload, document_id=document_id)
        exported = export_daveml_ir(ir)
        fresh = build_daveml_ir(exported, document_id=document_id)
        structural_diffs = compare_daveml_ir(ir, fresh)
        numeric_diffs = compare_daveml_numeric(ir, fresh)
        check_results = evaluate_daveml_checkdata(payload)
        vector_check_results = evaluate_daveml_vector_checkdata(payload)
        stem = Path(source_member).stem
        (output / "canonical" / f"{stem}.ir.json").write_bytes(ir.canonical_json())
        (output / "exported" / f"{stem}.dml").write_bytes(exported)
        structural.extend({"document_id": document_id, **diff.to_dict()} for diff in structural_diffs)
        numeric.extend({"document_id": document_id, **diff.to_dict()} for diff in numeric_diffs)
        documents.append({
            "document_id": document_id,
            "source_member": source_member,
            "source_sha256": ir.source_sha256,
            "ir_sha256": _sha256(ir.canonical_json()),
            "exported_sha256": _sha256(exported),
            "opaque_paths": list(ir.opaque_paths),
            "reference_summary": _reference_summary(ir),
            "structural_diff_count": len(structural_diffs),
            "numeric_diff_count": len(numeric_diffs),
            "checkdata": _checkdata_summary(check_results, vector_check_results),
        })
    manifest = _json_member(files, "manifest.json")
    report = {
        "status": "verified" if not structural and not numeric else "failed",
        "checkdata_status": _documents_checkdata_status(documents),
        "package": str(package),
        "package_sha256": _sha256(package.read_bytes()),
        "package_id": manifest.get("package_id", manifest.get("model_id", manifest.get("id"))),
        "schema_version": "taoryx.daveml-roundtrip/v1",
        "documents": documents,
        "structural_diff": structural,
        "numeric_diff": numeric,
        "limitations": [
            "Canonical export is generated from the Taoryx semantic tree IR.",
            "Source bytes remain the immutable authority for package identity.",
            "Numeric graph evaluation and runtime equivalence remain separate gates.",
        ],
    }
    output.joinpath("roundtrip-report.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if not structural and not numeric and report["checkdata_status"] != "failed" else 1
    ####


def _run_catalog(catalog_root: Path, output: Path, summary_output: Path | None = None) -> int:
    """Cycle every normalized source and qualified package in an INBOX catalog."""

    normalized = sorted(catalog_root.glob("normalized/*/source.dml"))
    packages = sorted(catalog_root.glob("qualified/**/*.txair"))
    if not normalized and not packages:
        raise ValueError(f"catalog has no normalized DAVE-ML sources or qualified packages: {catalog_root}")
    output.mkdir(parents=True, exist_ok=True)
    source_reports: list[dict[str, object]] = []
    for source in normalized:
        relative = source.relative_to(catalog_root).with_suffix("")
        target = output / "sources" / relative.parent / relative.name
        target.parent.mkdir(parents=True, exist_ok=True)
        payload = source.read_bytes()
        ir = build_daveml_ir(payload, document_id=source.relative_to(catalog_root).as_posix())
        exported = export_daveml_ir(ir)
        fresh = build_daveml_ir(exported, document_id=ir.document_id)
        structural = compare_daveml_ir(ir, fresh)
        numeric = compare_daveml_numeric(ir, fresh)
        check_results = evaluate_daveml_checkdata(payload)
        vector_check_results = evaluate_daveml_vector_checkdata(payload)
        checkdata = _checkdata_summary(check_results, vector_check_results)
        (target.parent / f"{target.name}.ir.json").write_bytes(ir.canonical_json())
        (target.parent / f"{target.name}.dml").write_bytes(exported)
        source_reports.append(
            {
                "source": ir.document_id,
                "source_sha256": ir.source_sha256,
                "exported_sha256": _sha256(exported),
                "opaque_paths": list(ir.opaque_paths),
                "reference_summary": _reference_summary(ir),
                "structural_diff_count": len(structural),
                "numeric_diff_count": len(numeric),
                "checkdata": checkdata,
                "checkdata_status": checkdata["status"],
                "checkdata_failed": checkdata["failed"],
                "ir_sha256": _sha256(ir.canonical_json()),
            }
        )
    package_reports: list[dict[str, object]] = []
    for package in packages:
        package_output = output / "packages" / package.stem
        status = _run_package(package, package_output)
        report_path = package_output / "roundtrip-report.json"
        report = json.loads(report_path.read_text(encoding="utf-8"))
        package_reports.append(
            {
                "package": str(package.relative_to(catalog_root)),
                "status": report.get("status"),
                "checkdata_status": report.get("checkdata_status", "not_present"),
                "checkdata_failed": sum(
                    int(item.get("failed", 0))
                    for item in report.get("documents", [])
                    if isinstance(item, dict)
                    and isinstance(item.get("checkdata"), dict)
                ),
                "document_count": len(report.get("documents", [])),
                "structural_diff_count": len(report.get("structural_diff", [])),
                "numeric_diff_count": len(report.get("numeric_diff", [])),
            }
        )
        if status != 0:
            raise ValueError(f"canonical package round trip failed: {package}")
    report = {
        "status": "verified"
        if all(
            item["structural_diff_count"] == 0
            and item["numeric_diff_count"] == 0
            and item.get("checkdata_status", "not_present") != "failed"
            and item.get("checkdata_failed", 0) == 0
            for item in source_reports + package_reports
        )
        else "failed",
        "schema_version": "taoryx.daveml-roundtrip/v1",
        "catalog_root": str(catalog_root),
        "source_count": len(source_reports),
        "package_count": len(package_reports),
        "sources": source_reports,
        "packages": package_reports,
        "limitations": [
            "The normalized source layer is the input authority for catalog-wide cycling.",
            "Canonical XML export is deterministic but does not claim byte identity with raw source XML.",
            "Runtime replay and semantic graph equivalence remain separate evidence layers.",
        ],
    }
    serialized = json.dumps(report, indent=2, sort_keys=True) + "\n"
    (output / "catalog-roundtrip-report.json").write_text(serialized, encoding="utf-8")
    _write_export_manifest(output, source_reports)
    if summary_output is not None:
        summary_output.parent.mkdir(parents=True, exist_ok=True)
        summary_output.write_text(serialized, encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


def _json_member(files: dict[str, bytes], basename: str) -> dict[str, object]:
    """Read a JSON member by basename from a package."""

    for name, payload in files.items():
        if Path(name).name.casefold() == basename.casefold():
            value = json.loads(payload)
            return value if isinstance(value, dict) else {}
    return {}
    ####


if __name__ == "__main__":
    raise SystemExit(main())
