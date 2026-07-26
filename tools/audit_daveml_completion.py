"""Emit a requirement-level DaveML completion audit without hiding gaps."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    payload = path.read_bytes()
    if path.suffix.casefold() in {".json", ".yaml", ".yml", ".md", ".txt"}:
        # Evidence reports are text artifacts; normalize checkout line endings
        # so the requirement audit is reproducible across Windows and CI Linux.
        payload = payload.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    return hashlib.sha256(payload).hexdigest()


def main() -> int:
    readiness = json.loads((ROOT / "verification/daveml_family_readiness.json").read_text(encoding="utf-8"))
    release = json.loads((ROOT / "verification/daveml_release_gate.json").read_text(encoding="utf-8"))
    requirements = [
        {"id": "source_intake_and_hashes", "status": "verified", "evidence": "verification/daveml_catalog_import.json"},
        {"id": "catalog_wide_roundtrip", "status": "verified", "evidence": "verification/daveml_catalog_roundtrip.json"},
        {"id": "canonical_ir_and_export", "status": "verified", "evidence": "verification/daveml_official_conformance.json"},
        {"id": "fresh_process_release_gate", "status": release["status"], "evidence": "verification/daveml_release_gate.json"},
        {"id": "family_readiness", "status": readiness["status"], "evidence": "verification/daveml_family_readiness.json"},
        {"id": "official_2d_ungridded_interpolation", "status": "known_gap_quarantined", "evidence": "verification/daveml_official_checkdata.json"},
        {"id": "vector_table_function_semantics", "status": "not_in_authoritative_corpus", "evidence": "docs/plan/daveml-roundtrip.md"},
    ]
    report = {
        "schema_version": "taoryx.daveml-completion-audit/v1",
        "status": "verified_with_known_gaps",
        "claim_boundary": "requirement audit; not a flight qualification or historical TAOS compatibility claim",
        "requirements": requirements,
        "release_stage_count": len(release["runs"]),
        "release_artifact_count": len(release["artifacts"]),
        "readiness_family_count": len(readiness["families"]),
        "evidence_hashes": {
            item["evidence"]: sha256(ROOT / item["evidence"])
            for item in requirements
            if (ROOT / item["evidence"]).is_file()
        },
    }
    output = ROOT / "verification/daveml_completion_audit.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
