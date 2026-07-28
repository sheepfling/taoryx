"""Assemble the four Alpha 2 showcase packs and one reviewer composite.

This is a catalog assembler, not a second simulator.  It copies the already
evaluated, packet-scoped artifacts and records the exact status and claim
boundary of each family.  A nominal integration result is never promoted to
family qualification by this tool.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import zipfile
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DEFAULT = ROOT / "artifacts/showcases/alpha2/final-catalog-v1"
####


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()
####


def _latest_b747_packet() -> Path | None:
    root = ROOT / "artifacts/showcases/alpha2-b747-x8"
    candidates = sorted((path for path in root.iterdir() if path.is_dir()), key=lambda path: path.stat().st_mtime) if root.exists() else []
    return candidates[-1] if candidates else None
####


def _b747_record() -> tuple[dict[str, Any], Path | None]:
    packet = _latest_b747_packet()
    if packet is None:
        return ({
            "id": "b747-transport",
            "vehicle": "B747-100 research surrogate",
            "status": "integration_evidence_pending_packet_rebuild",
            "claim": "Existing B747 source-bounded trim, descent, and route-controller evidence remains available in the fidelity-ladder artifacts.",
            "nonclaims": ["complete transport lifecycle", "physical actuator qualification", "runway operations", "fuel-range qualification"],
            "source": "artifacts/verification/fidelity_ladder_goal_plots",
        }, None)
    return ({
        "id": "b747-transport",
        "vehicle": "B747-100 research surrogate",
        "status": "nominal_integration_evidence_family_qualification_pending",
        "claim": "The source-bounded B747 packet contains reproducible trim, descent/recovery, approach, route-controller, and source-differential evidence.",
        "nonclaims": ["manufacturer flight-control fidelity", "runway takeoff or landing", "global transport envelope", "fuel-range qualification"],
        "source": str(packet.relative_to(ROOT)),
    }, packet)
####


def _b747_racetrack_packet() -> Path | None:
    """Return the reusable-template B747 witness when it has been built."""

    packet = ROOT / "artifacts/showcases/alpha2/b747-racetrack-altitude-turns-3dof-v1"
    return packet if packet.exists() else None
####


def _load_status(path: Path, key: str = "status") -> str:
    data = json.loads(path.read_text(encoding="utf-8"))
    return str(data.get(key, "unknown"))
####


def _render_composite(catalog: Path, sources: list[tuple[str, Path]]) -> None:
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt
    from PIL import Image

    figure, axes = plt.subplots(2, 2, figsize=(18, 12), layout="constrained")
    for axis, (label, path) in zip(axes.flat, sources):
        axis.axis("off")
        if path.exists():
            axis.imshow(Image.open(path).convert("RGB"))
        else:
            axis.text(0.5, 0.5, f"{label}\nartifact not available", ha="center", va="center", color="#991b1b", fontsize=14)
        axis.set_title(label, loc="left", fontsize=13, fontweight="bold")
    figure.suptitle("TAORYX Alpha 2 final showcase catalog — honest nominal evidence", fontsize=20, fontweight="bold")
    figure.text(0.01, 0.005, "Green does not mean family-qualified: each panel preserves its exact fidelity, realization, and nonclaim boundary.", fontsize=9, color="#334155")
    figure.savefig(catalog / "alpha2-showcase-composite.png", dpi=180, bbox_inches="tight")
    plt.close(figure)
####


def build(output: Path) -> Path:
    output.mkdir(parents=True, exist_ok=True)
    packs = output / "packs"
    packs.mkdir(exist_ok=True)
    records: list[dict[str, Any]] = []
    source_images: list[tuple[str, Path]] = []

    x8_source = ROOT / "artifacts/showcases/x8_working/x8-racetrack-altitude-turns-v1"
    x8_target = packs / "x8-racetrack-altitude-turns-v1"
    if x8_target.exists():
        shutil.rmtree(x8_target)
    shutil.copytree(x8_source, x8_target)
    x8_summary = json.loads((x8_source / "summary.json").read_text(encoding="utf-8"))
    records.append({"id": "x8-racetrack", "vehicle": "Skywalker X8", "status": x8_summary.get("status"), "mission_pass": x8_summary.get("mission_pass"), "claim": "Physical elevon-allocation racetrack nominal case with truth-based climb, level, bank, descent, and terminal crossing objectives.", "nonclaims": ["source-validated controller law", "wind robustness", "manufacturer fidelity", "landing/recovery dynamics"], "packet": str(x8_target.relative_to(output))})
    source_images.append(("X8 physical elevon allocation — nominal case pass / qualification pending", x8_source / "qualification_board.png"))

    hb_source = ROOT / "artifacts/showcases/alpha2/hummingbird-pad-to-pad-altitude-yaw-v2"
    hb_target = packs / hb_source.name
    if hb_target.exists():
        shutil.rmtree(hb_target)
    shutil.copytree(hb_source, hb_target)
    hb_summary = json.loads((hb_source / "summary.json").read_text(encoding="utf-8"))
    records.append({"id": "hummingbird-pad-to-pad", "vehicle": "AscTec Hummingbird", "status": hb_summary.get("status"), "mission_pass": hb_summary.get("mission_pass"), "claim": hb_summary.get("claim"), "nonclaims": hb_summary.get("nonclaims"), "packet": str(hb_target.relative_to(output))})
    source_images.append(("Hummingbird altitude-gated box, yaw, contact, and settle", hb_source / "mission_sequence.png"))

    cahi_source = ROOT / "artifacts/showcases/alpha2/cahi-x8-plus-boosters-v1"
    cahi_target = packs / cahi_source.name
    if cahi_target.exists():
        shutil.rmtree(cahi_target)
    shutil.copytree(cahi_source, cahi_target)
    cahi_summary = json.loads((cahi_source / "objective_report.json").read_text(encoding="utf-8"))
    records.append({"id": "cahi-x8-plus-boosters", "vehicle": "Synthetic X8-plus-booster CA-HI case", "status": "evidence_only_nominal_endpoint_failure", "mission_pass": cahi_summary.get("mission_pass"), "claim": "Staged powered, coast, glide, terminal-guidance, mass-flow, and equation-closure evidence.", "nonclaims": ["successful endpoint", "real X8 or booster fidelity", "validated thermal/aeroballistic design", "historical TAOS compatibility"], "packet": str(cahi_target.relative_to(output))})
    source_images.append(("CA-HI staged deployment — endpoint failure retained", cahi_source / "qualification_board.png"))

    b747_record, b747_source = _b747_record()
    if b747_source is not None:
        b747_target = packs / "b747-evidence"
        if b747_target.exists():
            shutil.rmtree(b747_target)
        shutil.copytree(b747_source, b747_target)
        b747_record["packet"] = str(b747_target.relative_to(output))
        candidate = next(iter(sorted(b747_source.rglob("*.png"))), None)
        source_images.append(("B747 transport evidence — qualification pending", candidate or Path("/missing")))
    else:
        source_images.append(("B747 transport evidence — packet rebuild pending", ROOT / "artifacts/showcases/canonical_fidelity_composites_v4/b747_composite.png"))
    b747_racetrack_source = _b747_racetrack_packet()
    if b747_racetrack_source is not None:
        b747_racetrack_target = packs / "b747-transport-scaled-racetrack"
        if b747_racetrack_target.exists():
            shutil.rmtree(b747_racetrack_target)
        shutil.copytree(b747_racetrack_source, b747_racetrack_target)
        b747_record["supporting_template_packet"] = str(b747_racetrack_target.relative_to(output))
        b747_record["supporting_template_claim"] = (
            "The B747 point-mass realization completes the same reusable racetrack phase and gate contract at transport scale; it does not add a rigid-body or surface-control claim."
        )
    records.append(b747_record)

    catalog = {"schema_version": 1, "catalog_id": "taoryx-alpha2-showcase-catalog-v1", "status": "honest_nominal_evidence_complete_family_qualification_pending", "goal_boundary": "All four requested packs are present and independently classified; only a pack whose required objectives and terminal contract pass may receive a qualified badge.", "families": records}
    (output / "catalog.json").write_text(json.dumps(catalog, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text("PYTHONPATH=.:src python3 tools/build_cahi_showcase_packet.py --output artifacts/showcases/alpha2\nPYTHONPATH=.:src python3 tools/build_alpha2_showcase_catalog.py --output artifacts/showcases/alpha2/final-catalog-v1\n", encoding="utf-8")
    _render_composite(output, source_images)
    manifest = {"schema_version": 1, "catalog": "catalog.json", "files": {}}
    manifest_path = output / "manifest.json"
    manifest["files"] = {str(path.relative_to(output)): _sha256(path) for path in sorted(output.rglob("*")) if path.is_file() and path != manifest_path}
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    archive = output.parent / "taoryx-alpha2-showcase-catalog-v1.zip"
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as handle:
        for path in sorted(output.rglob("*")):
            if path.is_file():
                handle.write(path, path.relative_to(output))
    return archive
####


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=OUTPUT_DEFAULT)
    args = parser.parse_args()
    print(build(args.output))
    ####


if __name__ == "__main__":
    main()
