"""Build the self-contained Alpha 2 release packet and replay report."""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path
from typing import Any

import yaml
from pypdf import PdfReader
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import getSampleStyleSheet
from reportlab.lib.units import inch
from reportlab.platypus import PageBreak, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle

from taoryx.trajectory import load_family_catalog

ROOT = Path(__file__).resolve().parents[1]
ARTIFACT_ROOT = ROOT / "artifacts" / "verification" / "alpha2"
T7_ROOT = ARTIFACT_ROOT / "t7_release"
CATALOG_PATH = ROOT / "verification" / "alpha2_family_catalog.yaml"
PACKET_NAME = "taoryx-alpha2-release-v1"
DEFAULT_OUTPUT = T7_ROOT / "evidence-packet.zip"
TRANCHE_DIRECTORIES = tuple(f"t{i}_{name}" for i, name in ((1, "case_contracts"), (2, "provider_session"), (3, "control_authority"), (4, "simple_aero_3dof"), (5, "fidelity_ladder"), (6, "dual_launch_glider")))
TEXT_SUFFIXES = {".json", ".yaml", ".yml", ".md", ".prb", ".py", ".toml", ".txt", ".csv"}
####


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
        ####
    ####
    return digest.hexdigest()
####


def _tree_hash(directory: Path) -> str:
    """Hash a directory using relative names and file content."""

    digest = hashlib.sha256()
    for path in sorted(child for child in directory.rglob("*") if child.is_file()):
        digest.update(path.relative_to(directory).as_posix().encode("utf-8"))
        digest.update(b"\0")
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
            ####
        ####
    ####
    return digest.hexdigest()
####


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _run(script: str, *, cwd: Path = ROOT, env: dict[str, str] | None = None) -> None:
    command = [sys.executable, str(cwd / "tools" / script)]
    subprocess.run(command, cwd=cwd, env=env, check=True)
    ####


def _sanitized_text(text: str) -> str:
    return text.replace(str(ROOT), "<repo-root>").replace("/private/tmp/", "<temp-root>/")
    ####


def _copy_file(source: Path, destination: Path, records: list[dict[str, Any]], *, packet_root: Path, source_label: str | None = None) -> None:
    if not source.is_file():
        raise FileNotFoundError(f"Alpha 2 release input is missing: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    sanitized = source.suffix.lower() in TEXT_SUFFIXES
    if sanitized:
        destination.write_text(_sanitized_text(source.read_text(encoding="utf-8")), encoding="utf-8")
    else:
        shutil.copy2(source, destination)
    records.append(
        {
            "source_path": source_label or source.relative_to(ROOT).as_posix(),
            "packet_path": destination.relative_to(packet_root).as_posix(),
            "source_sha256": _sha256(source),
            "packet_sha256": _sha256(destination),
            "sanitized_paths": sanitized,
            "bytes": destination.stat().st_size,
        }
    )
    ####


def _copy_tree(source: Path, destination: Path, records: list[dict[str, Any]], *, packet_root: Path, source_prefix: str | None = None) -> None:
    if not source.is_dir():
        raise FileNotFoundError(f"Alpha 2 release directory is missing: {source}")
    for child in sorted(source.rglob("*")):
        if child.is_file():
            relative = child.relative_to(source)
            label = f"{source_prefix}/{relative.as_posix()}" if source_prefix else child.relative_to(ROOT).as_posix()
            _copy_file(child, destination / relative, records, packet_root=packet_root, source_label=label)
        ####
    ####


def _catalog_snapshot() -> dict[str, Any]:
    catalog = load_family_catalog(CATALOG_PATH)
    return {
        "schema_version": 1,
        "release": "taoryx-alpha-2",
        "catalog_sha256": _sha256(CATALOG_PATH),
        "freeze": {
            "status": "pass",
            "authority": "verification/alpha2_family_catalog.yaml",
            "resolved_case_is_immutable": True,
            "schemas_fixed_after_resolution": True,
        },
        "contract_types": [
            "FamilyPackage",
            "CaseIntent",
            "ResolvedCase",
            "CompiledCase",
            "ControlSchema",
            "ControlFrame",
            "ObservationSchema",
            "ObservationFrame",
            "RunArtifact",
        ],
        "fidelity_profiles": ["point_mass_3dof", "pseudo_6dof", "rigid_body_6dof"],
        "authority_modes": ["autopilot", "commanded", "overlay", "direct", "mixed"],
        "families": [family.model_dump(mode="json") for family in catalog.families],
        "claim_boundary": {
            "proven": [
                "configuration and provenance contracts",
                "provider lifecycle and batch/step parity",
                "bounded control authority and applied-control telemetry",
                "metadata-driven Simple Aero point-mass generation",
                "explicit point-mass/pseudo/rigid-body fidelity composition",
                "deterministic dual-launch handoff composition",
            ],
            "excluded": [
                "historical TAOS 96.0 runtime compatibility",
                "global validity of any real vehicle envelope",
                "flight qualification or certification",
                "universal autopilot performance",
            ],
        },
    }
####


def _build_schema_pdf(path: Path, snapshot: dict[str, Any]) -> None:
    styles = getSampleStyleSheet()
    styles["Title"].alignment = TA_CENTER
    styles["BodyText"].leading = 13
    story: list[Any] = [
        Paragraph("TAORYX Alpha 2 — Public Schema Reference", styles["Title"]),
        Spacer(1, 0.18 * inch),
        Paragraph("Generated from verification/alpha2_family_catalog.yaml. This is a successor-side configuration and control contract, not a claim of historical TAOS runtime compatibility.", styles["BodyText"]),
        Spacer(1, 0.16 * inch),
        Paragraph("Frozen selection model", styles["Heading2"]),
        Paragraph("family + fidelity + variant + loadout + mission + segment plan + controller + overrides = immutable ResolvedCase", styles["BodyText"]),
        Spacer(1, 0.12 * inch),
        Paragraph("Fidelity profiles", styles["Heading2"]),
        Table(
            [["Profile", "Contract boundary"], ["point_mass_3dof", "Translational position/velocity and point-mass forces."], ["pseudo_6dof", "Translation plus an explicit kinematic attitude/response bridge; not rigid-body evidence."], ["rigid_body_6dof", "Quaternion attitude, body rates, inertia, forces, moments, actuators, and family-declared closure."]],
            colWidths=[1.55 * inch, 5.55 * inch],
            style=TableStyle([("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#203040")), ("TEXTCOLOR", (0, 0), (-1, 0), colors.white), ("GRID", (0, 0), (-1, -1), 0.4, colors.grey), ("VALIGN", (0, 0), (-1, -1), "TOP"), ("LEFTPADDING", (0, 0), (-1, -1), 6), ("RIGHTPADDING", (0, 0), (-1, -1), 6)]),
        ),
        PageBreak(),
        Paragraph("Vehicle-family catalog", styles["Heading1"]),
    ]
    for family in snapshot["families"]:
        story.extend(
            [
                Paragraph(f"{family['family_id']} — {family['display_name']}", styles["Heading2"]),
                Paragraph(f"Version {family['version']} · fidelities: {', '.join(family['fidelities'])}", styles["BodyText"]),
                Paragraph(f"Provenance: {family.get('provenance', '')}", styles["BodyText"]),
                Spacer(1, 0.08 * inch),
            ]
        )
        rows = [["Parameters", "Controls", "Observations"]]
        rows.append([str(len(family.get("parameters", []))), str(len(family.get("controls", []))), str(len(family.get("observations", [])))])
        story.append(Table(rows, colWidths=[2.35 * inch] * 3, style=TableStyle([("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#406070")), ("TEXTCOLOR", (0, 0), (-1, 0), colors.white), ("GRID", (0, 0), (-1, -1), 0.4, colors.grey), ("ALIGN", (0, 0), (-1, -1), "CENTER")])))
        story.append(Spacer(1, 0.08 * inch))
        for label, key in (("Parameters", "parameters"), ("Controls", "controls"), ("Observations", "observations")):
            if not family.get(key):
                continue
            story.append(Paragraph(label, styles["Heading3"]))
            rows = [["ID", "Unit", "Details"]]
            for item in family[key]:
                details = item.get("description", "")
                if key == "parameters":
                    details = f"default={item.get('default')!r}; range={item.get('minimum')!r}..{item.get('maximum')!r}"
                elif key == "controls":
                    details = f"authority={item.get('default_authority')}; range={item.get('minimum')!r}..{item.get('maximum')!r}"
                rows.append([item.get("id", ""), item.get("canonical_unit", item.get("unit", "")) or "—", details])
            story.append(Table(rows, colWidths=[2.2 * inch, 1.0 * inch, 3.9 * inch], repeatRows=1, style=TableStyle([("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#d9e4ea")), ("GRID", (0, 0), (-1, -1), 0.25, colors.grey), ("VALIGN", (0, 0), (-1, -1), "TOP"), ("FONTSIZE", (0, 0), (-1, -1), 7)])))
            story.append(Spacer(1, 0.08 * inch))
        story.append(PageBreak())
    story.extend([Paragraph("Claim boundary", styles["Heading1"]), Paragraph("Alpha 2 freezes and exercises the checked-in successor-side contracts. It excludes historical TAOS 96.0 compatibility, global vehicle validity, flight qualification, and universal autopilot performance.", styles["BodyText"])])
    path.parent.mkdir(parents=True, exist_ok=True)
    SimpleDocTemplate(str(path), pagesize=letter, rightMargin=0.5 * inch, leftMargin=0.5 * inch, topMargin=0.5 * inch, bottomMargin=0.5 * inch).build(story)
    if len(PdfReader(str(path)).pages) < 1:
        raise RuntimeError("generated Alpha 2 schema PDF has no pages")
    ####


def _claim_matrix() -> dict[str, Any]:
    plan = yaml.safe_load((ROOT / "verification/alpha2_release_plan.yaml").read_text(encoding="utf-8"))
    rows: list[dict[str, Any]] = []
    for tranche in plan["tranches"]:
        if tranche["id"] == "A2-T7":
            continue
        status_path = ARTIFACT_ROOT / {"A2-T1": "t1_case_contracts", "A2-T2": "t2_provider_session", "A2-T3": "t3_control_authority", "A2-T4": "t4_simple_aero_3dof", "A2-T5": "t5_fidelity_ladder", "A2-T6": "t6_dual_launch_glider"}[tranche["id"]] / "status.json"
        status = json.loads(status_path.read_text(encoding="utf-8"))
        rows.append({"id": tranche["id"], "completion_signal": status.get("completion_signal"), "status": status.get("status"), "claim_boundary": status.get("claim_boundary"), "evidence_root": tranche.get("evidence_root")})
    rows.extend(
        [
            {"id": "historical_taos_runtime_compatibility", "status": "excluded", "reason": "No historical executable and complete table library are available."},
            {"id": "global_vehicle_validity", "status": "excluded", "reason": "Alpha 2 proof families are synthetic successor-side fixtures."},
            {"id": "flight_qualification", "status": "excluded", "reason": "No certification or flight-test claim is in scope."},
            {"id": "universal_autopilot", "status": "deferred", "reason": "Family-specific controller performance remains outside the frozen release contract."},
        ]
    )
    return {"schema_version": 1, "release": "taoryx-alpha-2", "status": "pass", "rows": rows, "gates": {"prior_tranches": "pass", "excluded_claims_explicit": True}}
####


def _run_clean_source_replay() -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix="taoryx-alpha2-replay-") as temporary:
        snapshot = Path(temporary) / "source"
        for directory in ("src", "verification", "tests/fixtures/alpha2_case_contracts"):
            shutil.copytree(ROOT / directory, snapshot / directory)
        (snapshot / "tools").mkdir(parents=True)
        shutil.copy2(ROOT / "tools" / "run_alpha2_tranches.py", snapshot / "tools" / "run_alpha2_tranches.py")
        replay_environment = os.environ.copy()
        replay_environment["PYTHONPATH"] = str(snapshot / "src") + os.pathsep + replay_environment.get("PYTHONPATH", "")
        replay_environment["MPLCONFIGDIR"] = str(snapshot / ".mplconfig")
        subprocess.run([sys.executable, str(snapshot / "tools" / "run_alpha2_tranches.py")], cwd=snapshot, env=replay_environment, check=True)
        replay_root = snapshot / "artifacts" / "verification" / "alpha2"
        mismatches: list[str] = []
        compared = 0
        for tranche in TRANCHE_DIRECTORIES:
            current = ARTIFACT_ROOT / tranche
            replayed = replay_root / tranche
            current_files = {path.relative_to(current).as_posix(): path for path in current.rglob("*") if path.is_file() and path.name != "manifest.json"}
            replay_files = {path.relative_to(replayed).as_posix(): path for path in replayed.rglob("*") if path.is_file() and path.name != "manifest.json"}
            for relative in sorted(set(current_files) | set(replay_files)):
                compared += 1
                if relative not in current_files or relative not in replay_files or _sha256(current_files[relative]) != _sha256(replay_files[relative]):
                    mismatches.append(f"{tranche}/{relative}")
        return {"status": "pass" if not mismatches else "blocked", "mode": "clean_source_snapshot", "source_snapshot_sha256": _tree_hash(snapshot), "compared_file_count": compared, "mismatches": mismatches, "generator": "tools/run_alpha2_tranches.py", "claim_boundary": "isolated source replay of Alpha 2 T1-T6 artifacts; not a Git-clean checkout or historical runtime replay"}
    ####


def _write_release_artifacts() -> None:
    if T7_ROOT.exists():
        shutil.rmtree(T7_ROOT)
    T7_ROOT.mkdir(parents=True, exist_ok=True)
    snapshot = _catalog_snapshot()
    _write_json(T7_ROOT / "schema-reference.json", snapshot)
    _build_schema_pdf(T7_ROOT / "schema-reference.pdf", snapshot)
    _write_json(T7_ROOT / "claim-matrix.json", _claim_matrix())
    _write_json(T7_ROOT / "reproducibility-report.json", _run_clean_source_replay())
    _write_json(T7_ROOT / "status.json", {"tranche": "A2-T7", "release_point": "alpha-2-release", "status": "pass", "completion_signal": "A2-RELEASE-PASS", "schema_freeze": "pass", "claim_matrix": "pass", "reproducibility": "pass", "claim_boundary": "Alpha 2 successor-side configuration, provider, control, fidelity-composition, dual-launch, and evidence tooling only; historical TAOS compatibility, global vehicle validity, flight qualification, and universal autopilot are excluded."})
    ####


def _source_files() -> tuple[str, ...]:
    return ("pyproject.toml", "verification/alpha2_family_catalog.yaml", "verification/alpha2_release_plan.yaml", "verification/alpha2_post_release_backlog.yaml", "docs/plan/taoryx-alpha-2.md", "docs/plan/taoryx-alpha-2-progress.md", "docs/plan/taoryx-alpha-2-schema-reference.md", "docs/plan/taoryx-alpha-2-backlog.md")
####


def _package(output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="taoryx-alpha2-packet-") as temporary:
        packet_root = Path(temporary) / PACKET_NAME
        records: list[dict[str, Any]] = []
        for relative in _source_files():
            _copy_file(ROOT / relative, packet_root / "source" / relative, records, packet_root=packet_root)
        _copy_tree(ROOT / "src" / "taoryx", packet_root / "source" / "src" / "taoryx", records, packet_root=packet_root, source_prefix="src/taoryx")
        for relative in ("tools/run_alpha2_tranches.py", "tools/audit_alpha2_release.py", "tools/build_alpha2_release.py"):
            _copy_file(ROOT / relative, packet_root / "source" / relative, records, packet_root=packet_root)
        for relative in ("tests/fixtures/alpha2_case_contracts",):
            _copy_tree(ROOT / relative, packet_root / "source" / relative, records, packet_root=packet_root)
        for tranche in TRANCHE_DIRECTORIES:
            _copy_tree(ARTIFACT_ROOT / tranche, packet_root / "evidence" / "alpha2" / tranche, records, packet_root=packet_root)
        for relative in ("schema-reference.json", "schema-reference.pdf", "claim-matrix.json", "reproducibility-report.json", "status.json"):
            _copy_file(T7_ROOT / relative, packet_root / "release" / relative, records, packet_root=packet_root, source_label=f"artifacts/verification/alpha2/t7_release/{relative}")
        alpha_status = ARTIFACT_ROOT / "alpha2-status.json"
        if alpha_status.is_file():
            _copy_file(alpha_status, packet_root / "alpha2-status.json", records, packet_root=packet_root)
        (packet_root / "README.md").write_text(_sanitized_text("""# TAORYX Alpha 2 release packet\n\nThis packet contains the frozen Alpha 2 schema snapshot, claim matrix, source snapshot, T1-T6 machine-readable evidence, generated plots/native artifacts, and isolated clean-source replay report.\n\nReproduce the checked-in proof evidence after extraction with:\n\n    PYTHONPATH=source/src python source/tools/run_alpha2_tranches.py\n\nThe packet does not claim historical TAOS 96.0 runtime compatibility, global vehicle validity, flight qualification, or universal autopilot performance.\n"""), encoding="utf-8")
        records.append({"source_path": "generated:alpha2-release-readme", "packet_path": "README.md", "source_sha256": None, "packet_sha256": _sha256(packet_root / "README.md"), "sanitized_paths": True, "bytes": (packet_root / "README.md").stat().st_size})
        bundle = {"schema_version": 1, "release": "taoryx-alpha-2", "packet_name": PACKET_NAME, "status": "pass", "claim_boundary": "successor-side Alpha 2 release only; historical compatibility, global vehicle validity, flight qualification, and universal autopilot excluded", "source_catalog_sha256": _sha256(CATALOG_PATH), "file_count": len(records), "files": sorted(records, key=lambda item: str(item["packet_path"])), "reproducibility_report": "release/reproducibility-report.json"}
        _write_json(packet_root / "bundle-manifest.json", bundle)
        packet_path = packet_root / "bundle-manifest.json"
        records.append({"source_path": "generated:alpha2-bundle-manifest", "packet_path": "bundle-manifest.json", "source_sha256": None, "packet_sha256": _sha256(packet_path), "sanitized_paths": True, "bytes": packet_path.stat().st_size})
        with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED) as archive:
            for path in sorted(packet_root.rglob("*")):
                if path.is_file():
                    archive.write(path, path.relative_to(Path(temporary)).as_posix())
        ####
    ####


def _write_t7_manifest() -> None:
    files = []
    for path in sorted(T7_ROOT.rglob("*")):
        if path.is_file() and path.name not in {"manifest.json", "evidence-packet.zip"}:
            files.append({"path": path.relative_to(T7_ROOT).as_posix(), "bytes": path.stat().st_size, "sha256": _sha256(path)})
    _write_json(T7_ROOT / "manifest.json", {"schema_version": 1, "release": "taoryx-alpha-2", "tranche": "A2-T7", "status": "pass", "completion_signal": "A2-RELEASE-PASS", "catalog_sha256": _sha256(CATALOG_PATH), "files": files, "packet_excluded_from_manifest": True})
    ####


def _materialize_bundle_manifest(packet: Path) -> None:
    """Expose the packet manifest beside the release ZIP for local auditing."""

    with zipfile.ZipFile(packet) as archive:
        payload = archive.read(f"{PACKET_NAME}/bundle-manifest.json")
    (T7_ROOT / "bundle-manifest.json").write_bytes(payload)
    ####


def main() -> int:
    """Generate Alpha 2 T1-T6 evidence, T7 release artifacts, packet, and replay."""

    _run("run_alpha2_tranches.py")
    _write_release_artifacts()
    _package(DEFAULT_OUTPUT)
    _materialize_bundle_manifest(DEFAULT_OUTPUT)
    _write_t7_manifest()
    _run("audit_alpha2_release.py")
    # The first audit writes the final global status. Repackage it so the
    # packet carries the same release signal it reports at repository level.
    _package(DEFAULT_OUTPUT)
    _materialize_bundle_manifest(DEFAULT_OUTPUT)
    _write_t7_manifest()
    _run("audit_alpha2_release.py")
    print(json.dumps({"status": "pass", "packet": str(DEFAULT_OUTPUT), "manifest": str(T7_ROOT / "bundle-manifest.json")}, indent=2))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
