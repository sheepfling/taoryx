"""Package the current native maneuver matrix as a redacted evidence packet."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import uuid
import zipfile
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
MATRIX_REPORT = ROOT / "artifacts/vehicle-maneuver-matrix/matrix-report.json"
MATRIX_CONFIG = ROOT / "verification/vehicle_maneuver_matrix.yaml"
BINDING_CONFIG = ROOT / "verification/maneuver_evidence.yaml"
####


def _redact(value: Any) -> Any:
    """Replace machine-local paths in JSON evidence with repository paths."""

    if isinstance(value, str):
        root = str(ROOT)
        if value.startswith(root):
            return value[len(root) + 1 :]
        return value
    if isinstance(value, list):
        return [_redact(item) for item in value]
    if isinstance(value, dict):
        return {key: _redact(item) for key, item in value.items()}
    return value
####


def _hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
####


def build(output: Path, *, family: str | None = None) -> Path:
    """Build a UUID-scoped directory and ZIP for the current matrix run."""

    report = json.loads(MATRIX_REPORT.read_text(encoding="utf-8"))
    selected = [
        case
        for case in report["cases"]
        if family is None or str(case.get("id", "")).split("/", 1)[0] == family
    ]
    if not selected:
        raise ValueError(f"no maneuver evidence found for family {family!r}")
    run_id = str(uuid.uuid4())
    packet = output / run_id
    packet.mkdir(parents=True, exist_ok=False)
    evidence = packet / "evidence"
    evidence.mkdir()
    copied: dict[str, str] = {}

    def copy_file(source: Path, destination: Path) -> None:
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        copied[str(destination.relative_to(packet))] = _hash(destination)
    ####

    for source in (MATRIX_CONFIG, BINDING_CONFIG):
        copy_file(source, evidence / source.name)
    report_path = evidence / "matrix-report.json"
    report_path.write_text(json.dumps(_redact(report), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    copied[str(report_path.relative_to(packet))] = _hash(report_path)
    golden_dir = packet / "golden-plants"
    for source in sorted((ROOT / "artifacts/golden_plants").glob("*.json")):
        copy_file(source, golden_dir / source.name)
    parity_report = ROOT / "artifacts/verification/fidelity_parity-current/report.json"
    if parity_report.is_file():
        copy_file(parity_report, evidence / "fidelity-parity-report.json")
    # Recompute the differential summary from the current source/table inputs
    # so the packet never relies on a stale generated report.
    import sys

    sys.path.insert(0, str(ROOT))
    from tests.e2e.test_golden_source_differential import source_differential_reports

    differential_path = evidence / "source-differential-report.json"
    differential_path.write_text(
        json.dumps(_redact(source_differential_reports()), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    copied[str(differential_path.relative_to(packet))] = _hash(differential_path)

    for case in selected:
        case_id = str(case["id"])
        source_dir = ROOT / "artifacts/vehicle-maneuver-matrix" / case_id.replace("/", "-")
        if not source_dir.is_dir():
            continue
        destination_dir = packet / "cases" / case_id.replace("/", "-")
        for source in source_dir.rglob("*"):
            if not source.is_file():
                continue
            relative = source.relative_to(source_dir)
            destination = destination_dir / relative
            if source.suffix.lower() == ".json":
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_text(json.dumps(_redact(json.loads(source.read_text(encoding="utf-8"))), indent=2, sort_keys=True) + "\n", encoding="utf-8")
                copied[str(destination.relative_to(packet))] = _hash(destination)
            else:
                copy_file(source, destination)
    manifest = {
        "schema_version": 1,
        "run_id": run_id,
        "claim": "native maneuver matrix evidence",
        "claim_boundary": "research-surrogate bounded maneuvers; not flight qualification or historical TAOS compatibility",
        "families": sorted({str(case["id"]).split("/", 1)[0] for case in selected}),
        "case_count": len(selected),
        "status_counts": {
            status: sum(case.get("status") == status for case in selected)
            for status in sorted({str(case.get("status")) for case in selected})
        },
        "files": copied,
    }
    manifest_path = packet / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    copied["manifest.json"] = _hash(manifest_path)
    archive = output / f"maneuver-evidence-{run_id}.zip"
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as handle:
        for source in sorted(packet.rglob("*")):
            if source.is_file():
                handle.write(source, source.relative_to(packet))
    return archive
####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/verification/maneuver_matrix")
    parser.add_argument("--family")
    arguments = parser.parse_args()
    arguments.output.mkdir(parents=True, exist_ok=True)
    print(build(arguments.output, family=arguments.family))
    return 0
####


if __name__ == "__main__":
    raise SystemExit(main())
####
