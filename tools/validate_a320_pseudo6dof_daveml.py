"""Fresh-process round-trip gate for the A320 authored DAVE-ML collection."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

from taoryx.trajectory import build_daveml_ir, compare_daveml_ir, compare_daveml_numeric, evaluate_daveml_checkdata, export_daveml_ir

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_COLLECTION = ROOT / "families/a320_openap_jsbsim_pseudo6dof/daveml"
DEFAULT_REPORT = ROOT / "families/a320_openap_jsbsim_pseudo6dof/validation/roundtrip-report.json"


def validate(collection: Path, report_path: Path) -> dict[str, object]:
    documents: list[dict[str, object]] = []
    structural: list[dict[str, object]] = []
    numeric: list[dict[str, object]] = []
    with tempfile.TemporaryDirectory(prefix="a320-pseudo-daveml-", dir=ROOT / "build") as directory:
        temporary = Path(directory)
        for source in sorted(collection.glob("*.dml")):
            payload = source.read_bytes()
            document_id = source.name
            ir = build_daveml_ir(payload, document_id=document_id)
            exported = export_daveml_ir(ir)
            exported_path = temporary / source.name
            fresh_path = temporary / f"fresh-{source.name}"
            exported_path.write_bytes(exported)
            environment = dict(os.environ)
            environment["PYTHONPATH"] = str(ROOT / "src")
            subprocess.run(
                [sys.executable, str(ROOT / "tools/reimport_daveml_ir.py"), str(exported_path), "--document-id", document_id, "--output", str(fresh_path)],
                cwd=ROOT,
                env=environment,
                check=True,
                capture_output=True,
                text=True,
            )
            fresh = build_daveml_ir(fresh_path.read_bytes(), document_id=document_id)
            structural_diffs = compare_daveml_ir(ir, fresh)
            numeric_diffs = compare_daveml_numeric(ir, fresh)
            check_results = evaluate_daveml_checkdata(payload)
            structural.extend({"document_id": document_id, **diff.to_dict()} for diff in structural_diffs)
            numeric.extend({"document_id": document_id, **diff.to_dict()} for diff in numeric_diffs)
            documents.append(
                {
                    "document_id": document_id,
                    "source_sha256": hashlib.sha256(payload).hexdigest(),
                    "canonical_export_sha256": hashlib.sha256(exported).hexdigest(),
                    "fresh_process_reimport": "verified",
                    "structural_diff_count": len(structural_diffs),
                    "numeric_diff_count": len(numeric_diffs),
                    "checkdata": {
                        "status": "verified" if all(item.status == "passed" for item in check_results) else "failed",
                        "count": len(check_results),
                        "passed": sum(item.status == "passed" for item in check_results),
                        "results": [item.to_dict() for item in check_results],
                    },
                }
            )
    report = {
        "schema_version": "taoryx.a320-pseudo6dof-daveml-roundtrip/v1",
        "status": "verified" if documents and not structural and not numeric and all(item["checkdata"]["status"] == "verified" for item in documents) else "failed",
        "claim_boundary": "Taoryx-authored surrogate-composite DAVE-ML; not upstream source-preserving or manufacturer-authoritative",
        "collection": "a320-openap-jsbsim-pseudo6dof",
        "documents": documents,
        "structural_diff": structural,
        "numeric_diff": numeric,
        "provenance": json.loads((collection / "provenance.json").read_text(encoding="utf-8")),
    }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collection-dir", type=Path, default=DEFAULT_COLLECTION)
    parser.add_argument("--output", type=Path, default=DEFAULT_REPORT)
    args = parser.parse_args()
    report = validate(args.collection_dir.resolve(), args.output.resolve())
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "verified" else 1


if __name__ == "__main__":
    raise SystemExit(main())
