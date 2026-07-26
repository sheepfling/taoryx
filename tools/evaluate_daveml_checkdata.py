"""Evaluate supported DAVE-ML static checkData cases with typed reports."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from taoryx.trajectory.daveml_evaluator import evaluate_daveml_checkdata, evaluate_daveml_vector_checkdata


def main() -> int:
    """Evaluate every DAVE-ML document in a directory."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    documents: list[dict[str, object]] = []
    for source in sorted(arguments.source_dir.glob("*.dml")):
        payload = source.read_bytes()
        results = evaluate_daveml_checkdata(payload)
        vector_results = evaluate_daveml_vector_checkdata(payload)
        documents.append(
            {
                "document": source.name,
                "check_count": len(results),
                "passed": sum(result.status == "passed" for result in results),
                "failed": sum(result.status == "failed" for result in results),
                "unsupported": sum(result.status == "unsupported" for result in results),
                "checks": [result.to_dict() for result in results],
                "vector_check_count": len(vector_results),
                "vector_passed": sum(result.status == "passed" for result in vector_results),
                "vector_failed": sum(result.status == "failed" for result in vector_results),
                "vector_unsupported": sum(result.status == "unsupported" for result in vector_results),
                "vector_checks": [result.to_dict() for result in vector_results],
            }
        )
    has_unsupported = any(item["unsupported"] > 0 or item["vector_unsupported"] > 0 for item in documents)
    report = {
        "schema_version": "taoryx.daveml-checkdata/v1",
        "status": "failed" if any(item["failed"] > 0 or item["vector_failed"] > 0 for item in documents) else "verified_with_quarantine" if has_unsupported else "verified",
        "documents": documents,
        "limitations": [
            "Direct point functions, nested MathML calculations, and regular gridded tables are evaluated.",
            "Regular-grid queries follow NASA DAVEtools endpoint-clamp behavior outside breakpoint bounds.",
            "Unsupported graph forms remain explicit quarantined results.",
            "Documents without checkData are represented with check_count zero.",
        ],
    }
    arguments.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] != "failed" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
