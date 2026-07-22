"""Package the latest family-scoped fidelity packets into one review bundle."""

from __future__ import annotations

import argparse
import hashlib
import json
import uuid
import zipfile
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
FAMILY_ORDER = ("b747", "skywalker_x8", "hummingbird", "x15")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
    ####


def _latest_family_packets(source: Path) -> dict[str, Path]:
    """Find the newest valid packet for each required family."""

    candidates: dict[str, tuple[float, Path]] = {}
    for archive in source.glob("fidelity-ladder-evidence-*.zip"):
        try:
            with zipfile.ZipFile(archive) as handle:
                manifest = json.loads(handle.read("manifest.json"))
            families = manifest.get("families", [])
            if len(families) != 1:
                continue
            family_id = str(families[0]["id"])
        except (OSError, KeyError, ValueError, zipfile.BadZipFile):
            continue
        current = candidates.get(family_id)
        candidate = (archive.stat().st_mtime, archive)
        if current is None or candidate[0] > current[0]:
            candidates[family_id] = candidate
    missing = [family for family in FAMILY_ORDER if family not in candidates]
    if missing:
        raise FileNotFoundError(
            "missing family-scoped fidelity packets: " + ", ".join(missing)
        )
    return {family: candidates[family][1] for family in FAMILY_ORDER}
    ####


def build(source: Path, output: Path) -> Path:
    """Build a portable rollup containing the four family packets."""

    packets = _latest_family_packets(source)
    run_id = str(uuid.uuid4())
    packet_dir = output / run_id
    packet_dir.mkdir(parents=True, exist_ok=False)
    packet_records: list[dict[str, Any]] = []
    for family, archive in packets.items():
        destination = packet_dir / "families" / f"{family}.zip"
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(archive.read_bytes())
        with zipfile.ZipFile(archive) as handle:
            manifest = json.loads(handle.read("manifest.json"))
        family_manifest = manifest["families"][0]
        packet_records.append(
            {
                "family": family,
                "archive": str(destination.relative_to(packet_dir)),
                "archive_sha256": _sha256(destination),
                "source_run_id": manifest["run_id"],
                "tier_ids": sorted(family_manifest.get("tiers", {})),
                "controller_ids": [item["id"] for item in manifest.get("controller_cases", ())],
                "claim_boundary": manifest["claim_boundary"],
            }
        )
    rollup = {
        "schema_version": 1,
        "run_id": run_id,
        "families": list(FAMILY_ORDER),
        "claim_boundary": "source-bounded research-surrogate evidence; not flight qualification",
        "source_catalogs": [
            "verification/fidelity_ladder.yaml",
            "verification/fidelity_parity.yaml",
            "verification/long_validation_trajectories.yaml",
            "verification/controller_scenarios.yaml",
        ],
        "packets": packet_records,
    }
    manifest = packet_dir / "rollup.json"
    manifest.write_text(json.dumps(rollup, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    archive = output / f"fidelity-ladder-rollup-{run_id}.zip"
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as handle:
        for path in sorted(packet_dir.rglob("*")):
            if path.is_file():
                handle.write(path, path.relative_to(packet_dir))
    return archive
    ####


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        type=Path,
        default=ROOT / "artifacts/verification/fidelity_ladder_scoped",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "artifacts/verification/fidelity_ladder_rollup",
    )
    arguments = parser.parse_args()
    arguments.output.mkdir(parents=True, exist_ok=True)
    print(build(arguments.source, arguments.output))
    ####


if __name__ == "__main__":
    main()
