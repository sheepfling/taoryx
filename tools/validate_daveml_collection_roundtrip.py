"""Run a fresh-process canonical round trip for a deterministic collection fixture."""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.trajectory import CollectionManifest, SourceDocument, write_deterministic_collection


def main() -> int:
    source_path = ROOT / "resources/aerospace/daveml/official-conformance-v1/F16_aero.dml"
    source = source_path.read_bytes()
    source_hash = hashlib.sha256(source).hexdigest()
    manifest = CollectionManifest(
        collection_type="taoryx.txcollection/v1alpha1",
        collection_id="daveml-f16-canonical-smoke",
        collection_version="1.0.0",
        family_id="reference_f16_s119",
        canonical_authority="taoryx_canonical",
        source_documents=(
            SourceDocument(
                document_id="f16.aerodynamics",
                role="aerodynamics",
                source_path="f16/aerodynamics.dml",
                package_member="models/aerodynamics.dml",
                sha256=source_hash,
                byte_identical_to_upstream=True,
            ),
        ),
        component_bindings=(),
        contribution_authority=(),
        transforms=(),
        stateful_components=(),
        runtime_artifact="runtime/not-declared.txair",
        validation_artifact="validation/not-declared.json",
        source_payload_external=False,
    )
    with tempfile.TemporaryDirectory(prefix="daveml-collection-") as temporary:
        root = Path(temporary)
        collection = write_deterministic_collection(
            {"manifest.json": manifest.canonical_json(), "source/aerodynamics.dml": source},
            root / "fixture.txcollection",
        )
        output = root / "output"
        environment = dict(os.environ)
        environment["PYTHONPATH"] = str(ROOT / "src")
        process = subprocess.run(
            [sys.executable, "tools/roundtrip_daveml_collection.py", "--collection", str(collection.path), "--output-dir", str(output)],
            cwd=ROOT,
            env=environment,
            capture_output=True,
            text=True,
        )
        report = json.loads((output / "roundtrip-report.json").read_text(encoding="utf-8")) if (output / "roundtrip-report.json").is_file() else {}
    result = {
        "schema_version": "taoryx.daveml-collection-roundtrip-evidence/v1",
        "status": "verified" if process.returncode == 0 and report.get("levels", {}).get("L3", "").startswith("verified") else "failed",
        "claim_boundary": "fresh-process collection L0-L3 canonical round trip; L4 runtime replay remains separate",
        "collection_id": manifest.collection_id,
        "source_sha256": source_hash,
        "subprocess_returncode": process.returncode,
        "levels": report.get("levels", {}),
        "canonical_documents": report.get("canonical_documents", []),
        "stderr_tail": process.stderr[-1000:],
    }
    output_path = ROOT / "verification/daveml_collection_roundtrip_evidence.json"
    output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "verified" else 1


if __name__ == "__main__":
    raise SystemExit(main())
