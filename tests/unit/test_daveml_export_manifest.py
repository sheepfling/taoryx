from __future__ import annotations

import json

from tools.canonical_daveml_roundtrip import _write_export_manifest


def test_export_manifest_is_sorted_and_hash_linked(tmp_path) -> None:
    """The canonical export sidecar is deterministic and provenance-bearing."""

    _write_export_manifest(
        tmp_path,
        [
            {
                "document_id": "z-document",
                "source_member": "source/z.dml",
                "source_sha256": "a" * 64,
                "ir_sha256": "b" * 64,
                "exported_sha256": "c" * 64,
                "opaque_paths": ["/0/2"],
            },
            {
                "document_id": "a-document",
                "source_member": "source/a.dml",
                "source_sha256": "d" * 64,
                "ir_sha256": "e" * 64,
                "exported_sha256": "f" * 64,
            },
        ],
    )

    manifest = json.loads((tmp_path / "canonical" / "export-manifest.json").read_text())
    assert manifest["schema_version"] == "taoryx.daveml-export-manifest/v1"
    assert [item["document_id"] for item in manifest["documents"]] == ["a-document", "z-document"]
    assert manifest["documents"][1]["opaque_paths"] == ["/0/2"]
    assert manifest["documents"][0]["ir_sha256"] == "e" * 64
