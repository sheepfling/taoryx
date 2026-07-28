from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
from zipfile import BadZipFile, ZipFile


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
    ####


def verify_txair(path: Path) -> list[str]:
    errors: list[str] = []
    try:
        with ZipFile(path) as archive:
            bad = archive.testzip()
            if bad is not None:
                errors.append(f"{path}: bad ZIP member {bad}")
            names = set(archive.namelist())
            for name in names:
                pure = PurePosixPath(name)
                if pure.is_absolute() or ".." in pure.parts:
                    errors.append(f"{path}: unsafe member {name}")
            if "checksums.sha256" in names:
                ledger = archive.read("checksums.sha256").decode("utf-8")
                for line in ledger.splitlines():
                    if not line.strip():
                        continue
                    expected, member = line.split("  ", 1)
                    if member not in names:
                        errors.append(f"{path}: missing internal member {member}")
                        continue
                    actual = hashlib.sha256(archive.read(member)).hexdigest()
                    if actual != expected:
                        errors.append(f"{path}: internal hash mismatch {member}")
    except BadZipFile:
        errors.append(f"{path}: invalid ZIP")
    return errors
    ####


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    errors: list[str] = []
    ledger = root / "checksums.sha256"
    for line in ledger.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        expected, relative = line.split("  ", 1)
        path = root / relative
        if not path.is_file():
            errors.append(f"missing: {relative}")
            continue
        actual = sha256_file(path)
        if actual != expected:
            errors.append(f"hash mismatch: {relative}")
    summary = json.loads((root / "catalog/catalog-summary.json").read_text(encoding="utf-8"))
    documents = json.loads((root / "catalog/source-documents.json").read_text(encoding="utf-8"))
    models = json.loads((root / "catalog/models.json").read_text(encoding="utf-8"))
    scenarios = json.loads((root / "catalog/scenarios.json").read_text(encoding="utf-8"))
    if len(documents) != summary["source_document_count"]:
        errors.append("source document count mismatch")
    if len(models) != summary["model_family_count"]:
        errors.append("model count mismatch")
    if len(scenarios) != summary["scenario_count"]:
        errors.append("scenario count mismatch")
    for row in documents:
        if row["parse_status"] == "passed":
            normalized = root / row["normalized_path"]
            if not (normalized / "model.json").is_file():
                errors.append(f"missing normalized IR: {row['document_id']}")
    for txair in sorted((root / "qualified").rglob("*.txair")):
        errors.extend(verify_txair(txair))
    report = {
        "passed": not errors,
        "error_count": len(errors),
        "errors": errors,
        "catalog_id": summary["catalog_id"],
        "models": len(models),
        "documents": len(documents),
        "scenarios": len(scenarios),
    }
    if args.json:
        args.json.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if not errors else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
