"""Validate the seven authoritative DAVE-ML example documents."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path

from taoryx.trajectory import build_daveml_ir, compare_daveml_ir, compare_daveml_numeric, export_daveml_ir


def main() -> int:
    """Run deterministic export and fresh-import checks for all fixtures."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fixture_dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    manifest = json.loads((arguments.fixture_dir / "manifest.json").read_text(encoding="utf-8"))
    expected = {item["file"]: item["sha256"].casefold() for item in manifest["fixtures"]}
    documents: list[dict[str, object]] = []
    for source in sorted(arguments.fixture_dir.glob("*.dml")):
        payload = source.read_bytes()
        digest = hashlib.sha256(payload).hexdigest()
        ir = build_daveml_ir(payload, document_id=source.name)
        exported = export_daveml_ir(ir)
        fresh = build_daveml_ir(exported, document_id=source.name)
        structural = compare_daveml_ir(ir, fresh)
        numeric = compare_daveml_numeric(ir, fresh)
        source_checks = _checkdata_signature(payload)
        exported_checks = _checkdata_signature(exported)
        documents.append(
            {
                "fixture": source.name,
                "source_sha256": digest,
                "manifest_hash_match": expected.get(source.name) == digest,
                "exported_sha256": hashlib.sha256(exported).hexdigest(),
                "structural_diff_count": len(structural),
                "numeric_diff_count": len(numeric),
                "checkdata_count": source_checks[0],
                "checkdata_numeric_literal_count": source_checks[1],
                "checkdata_roundtrip_match": source_checks == exported_checks,
                "opaque_paths": list(ir.opaque_paths),
            }
        )
    report = {
        "schema_version": "taoryx.daveml-official-conformance/v1",
        "fixture_count": len(documents),
        "status": "verified" if len(documents) == 7 and len(expected) == 7 and all(
            item["manifest_hash_match"] and item["structural_diff_count"] == 0 and item["numeric_diff_count"] == 0 and item["checkdata_roundtrip_match"] for item in documents
        ) else "failed",
        "documents": documents,
        "limitations": [
            "This gate verifies source-preserving semantic export and fresh import.",
            "DAVE-ML checkData numerical evaluation remains a separate runtime gate.",
        ],
    }
    arguments.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "verified" else 1
    ####


def _checkdata_signature(payload: bytes) -> tuple[int, int]:
    """Return checkData element and numeric-literal counts for provenance."""

    root = ET.fromstring(payload)
    elements = [element for element in root.iter() if _local_name(element.tag) == "checkData"]
    numeric_count = sum(len(re.findall(r"(?<![A-Za-z])[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?(?![A-Za-z])", " ".join(element.itertext()))) for element in elements)
    return len(elements), numeric_count


def _local_name(tag: str) -> str:
    """Return an XML tag's local name."""

    return tag.rsplit("}", 1)[-1]


if __name__ == "__main__":
    raise SystemExit(main())
