from __future__ import annotations

import hashlib
import importlib.util
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TOOL_PATH = ROOT / "tools/verify_daveml_reference_inputs.py"
SPEC = importlib.util.spec_from_file_location("verify_daveml_reference_inputs", TOOL_PATH)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_reference_manifest_reports_current_external_inputs_as_blocked() -> None:
    report = MODULE.verify_manifest(
        ROOT / "verification/daveml_reference_integration.yaml",
        ROOT / "source-inputs-that-are-not-in-the-checkout",
    )

    assert report["overall_status"] == "blocked"
    assert {item["status"] for item in report["families"]} == {"blocked_missing_input", "intentionally_blocked"}
####


def test_input_verifier_accepts_an_exact_hash(tmp_path: Path) -> None:
    payload = b"small deterministic DAVE-ML intake fixture\n"
    expected = hashlib.sha256(payload).hexdigest()
    source_root = tmp_path / "inputs"
    source_root.mkdir()
    relative = Path("fixture/model.dml")
    path = source_root / relative
    path.parent.mkdir()
    path.write_bytes(payload)

    family = {
        "id": "fixture_family",
        "expected_input": {"kind": "source_file", "relative_path": relative.as_posix(), "sha256": expected},
    }
    result = MODULE._input_result(family, source_root)

    assert result["status"] == "verified_input"
    assert result["observed_sha256"] == expected
####


def test_input_verifier_rejects_a_hash_mismatch(tmp_path: Path) -> None:
    source_root = tmp_path / "inputs"
    source_root.mkdir()
    path = source_root / "model.dml"
    path.write_bytes(b"wrong bytes\n")

    result = MODULE._input_result(
        {"id": "fixture_family", "expected_input": {"relative_path": "model.dml", "sha256": "0" * 64}},
        source_root,
    )

    assert result["status"] == "blocked_hash_mismatch"
    assert result["reason"] == "payload_hash_does_not_match_record"
####


def test_input_verifier_checks_an_embedded_txair_member_ledger(tmp_path: Path) -> None:
    member = b"deterministic model member\n"
    member_hash = hashlib.sha256(member).hexdigest()
    package = tmp_path / "model.txair"
    with zipfile.ZipFile(package, "w") as archive:
        archive.writestr("manifest.json", b"{}\n")
        archive.writestr("model.dml", member)
        archive.writestr("checksums.sha256", f"{member_hash}  model.dml\n")

    result = MODULE._input_result(
        {
            "id": "fixture_family",
            "expected_input": {
                "kind": "package",
                "relative_path": "model.txair",
                "sha256": hashlib.sha256(package.read_bytes()).hexdigest(),
            },
        },
        tmp_path,
    )

    assert result["status"] == "verified_input"
    assert result["package_integrity"] == {"member_count": 3, "checksum_entries": 1}
####
