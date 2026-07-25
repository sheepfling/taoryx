"""Verify exact external source inputs for the DAVE-ML reference families.

The repository intentionally keeps source-grounded model bytes in an external
input store until their licensing and distribution path is settled. This tool
turns that absence into a machine-readable intake status instead of allowing a
recorded hash to be mistaken for an available executable model.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import zipfile
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = ROOT / "verification/daveml_reference_integration.yaml"


def sha256_file(path: Path) -> str:
    """Return the SHA-256 digest of *path* without loading it all at once."""

    with path.open("rb") as stream:
        return _sha256_stream(stream)
####


def _sha256_stream(stream: Any) -> str:
    digest = hashlib.sha256()
    for block in iter(lambda: stream.read(1024 * 1024), b""):
        digest.update(block)
    return digest.hexdigest()
####


def _verify_txair_package(path: Path) -> dict[str, Any]:
    """Verify the package CRC and its embedded member checksum ledger."""

    with zipfile.ZipFile(path) as package:
        bad_member = package.testzip()
        if bad_member:
            raise ValueError(f"package CRC failure: {bad_member}")
        names = set(package.namelist())
        if "manifest.json" not in names or "checksums.sha256" not in names:
            raise ValueError("package requires manifest.json and checksums.sha256")
        ledger = package.read("checksums.sha256").decode("utf-8")
        checked = 0
        for line in ledger.splitlines():
            if not line.strip():
                continue
            expected, relative = line.split("  ", 1)
            member = Path(relative)
            if member.is_absolute() or ".." in member.parts or member.as_posix() not in names:
                raise ValueError(f"unsafe or missing package member: {relative}")
            with package.open(member.as_posix()) as stream:
                observed = _sha256_stream(stream)
            if observed != expected:
                raise ValueError(f"package member hash mismatch: {relative}")
            checked += 1
    return {"member_count": len(names), "checksum_entries": checked}
####


def _input_result(
    family: dict[str, Any],
    source_root: Path,
    expected: dict[str, Any] | None = None,
) -> dict[str, Any]:
    expected = expected if expected is not None else family.get("expected_input")
    if not expected:
        if family.get("status") == "blocked_missing_source":
            return {
                "family": family["id"],
                "status": "intentionally_blocked",
                "reason": "source_acquisition_not_started",
            }
        return {
            "family": family["id"],
            "status": "record_incomplete",
            "reason": "missing_expected_input_declaration",
        }

    relative_path = Path(str(expected["relative_path"]))
    if relative_path.is_absolute() or ".." in relative_path.parts:
        return {
            "family": family["id"],
            "status": "record_invalid",
            "reason": "expected_input_path_must_be_relative",
            "relative_path": str(relative_path),
        }

    candidate = source_root / relative_path
    result: dict[str, Any] = {
        "family": family["id"],
        "role": expected.get("role"),
        "kind": expected.get("kind"),
        "relative_path": relative_path.as_posix(),
        "expected_sha256": str(expected["sha256"]),
        "absolute_path": str(candidate),
    }
    if not candidate.is_file():
        result.update(status="blocked_missing_input", reason="expected_payload_not_present")
        return result

    observed = sha256_file(candidate)
    result["observed_sha256"] = observed
    if observed != result["expected_sha256"]:
        result.update(status="blocked_hash_mismatch", reason="payload_hash_does_not_match_record")
    else:
        try:
            if expected.get("kind") == "package":
                result["package_integrity"] = _verify_txair_package(candidate)
        except (ValueError, zipfile.BadZipFile) as exc:
            result.update(status="blocked_package_integrity", reason=str(exc))
            return result
        result.update(status="verified_input", reason="payload_present_and_hash_verified")
    return result
####


def verify_manifest(manifest_path: Path, source_root: Path) -> dict[str, Any]:
    """Verify all declared inputs and return a stable report payload."""

    document = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    families = document.get("families", [])
    results: list[dict[str, Any]] = []
    for family in families:
        expected_inputs = family.get("expected_inputs")
        if expected_inputs is None:
            expected_inputs = [family.get("expected_input")]
        if not expected_inputs or expected_inputs == [None]:
            results.append(_input_result(family, source_root))
            continue
        results.extend(_input_result(family, source_root, expected) for expected in expected_inputs)
    blocking = {
        "blocked_missing_input",
        "blocked_hash_mismatch",
        "blocked_package_integrity",
        "record_invalid",
        "record_incomplete",
    }
    required = [item for item in results if item["status"] != "intentionally_blocked"]
    overall = "ready_for_source_replay" if required and all(item["status"] == "verified_input" for item in required) else "blocked"
    return {
        "schema_version": 1,
        "manifest": str(manifest_path),
        "source_root": str(source_root),
        "overall_status": overall,
        "blocking_statuses": sorted({item["status"] for item in results if item["status"] in blocking}),
        "families": results,
    }
####


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument(
        "--source-root",
        type=Path,
        required=True,
        help="External input-store root containing the paths declared by the manifest.",
    )
    parser.add_argument("--json", type=Path, help="Write the report to this path as well as stdout.")
    parser.add_argument("--strict", action="store_true", help="Exit nonzero unless every declared input verifies.")
    return parser
####


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    report = verify_manifest(args.manifest.resolve(), args.source_root.resolve())
    encoded = json.dumps(report, indent=2, sort_keys=True)
    print(encoded)
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(encoded + "\n", encoding="utf-8")
    return 0 if not args.strict or report["overall_status"] == "ready_for_source_replay" else 2
####


if __name__ == "__main__":
    sys.exit(main())
