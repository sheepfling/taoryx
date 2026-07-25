"""Audit the self-contained evidence for the completed Alpha 2 tranches."""

from __future__ import annotations

import csv
import hashlib
import json
import re
import zipfile
from pathlib import Path
from typing import Any

from taoryx.trajectory import ResolvedCase

ROOT = Path(__file__).resolve().parents[1]
ARTIFACT_ROOT = ROOT / "artifacts" / "verification" / "alpha2"
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


def _read(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))
####


def _audit_manifest(directory: Path) -> list[str]:
    errors: list[str] = []
    manifest_path = directory / "manifest.json"
    if not manifest_path.is_file():
        return [f"missing manifest: {manifest_path}"]
    manifest = _read(manifest_path)
    if manifest.get("status") != "pass":
        errors.append(f"manifest status is not pass: {manifest.get('status')!r}")
    for item in manifest.get("files", []):
        relative = item.get("path")
        if not isinstance(relative, str):
            errors.append("manifest contains a non-string path")
            continue
        path = directory / relative
        if not path.is_file():
            errors.append(f"manifest file is missing: {relative}")
            continue
        if path.stat().st_size != item.get("bytes"):
            errors.append(f"byte count mismatch: {relative}")
        if _sha256(path) != item.get("sha256"):
            errors.append(f"sha256 mismatch: {relative}")
    ####
    return errors
####


def _audit_t1(directory: Path) -> list[str]:
    errors = _audit_manifest(directory)
    status = _read(directory / "status.json")
    if status.get("completion_signal") != "A2-T1-PASS":
        errors.append("A2-T1 completion signal is missing")
    if not status.get("identity_replay"):
        errors.append("A2-T1 identity replay did not pass")
    if not status.get("provenance_complete"):
        errors.append("A2-T1 provenance completeness did not pass")
    diagnostics = _read(directory / "diagnostics.json").get("negative_cases", [])
    codes = {item.get("code") for item in diagnostics}
    if codes != {"unknown-override", "unit-mismatch", "unsupported-fidelity"}:
        errors.append(f"A2-T1 negative diagnostic set is incomplete: {sorted(codes)}")
    for path in sorted((directory / "cases").glob("*/resolved-case.json")):
        resolved = ResolvedCase.model_validate_json(path.read_text(encoding="utf-8"))
        if resolved.recompute_identity() != resolved.identity_sha256:
            errors.append(f"resolved case identity mismatch: {path.relative_to(directory)}")
        provenance = _read(path.parent / "provenance.json")
        if len(provenance) != len(resolved.parameters):
            errors.append(f"provenance count mismatch: {path.relative_to(directory)}")
    return errors
####


def _audit_t2(directory: Path) -> list[str]:
    errors = _audit_manifest(directory)
    status = _read(directory / "status.json")
    if status.get("completion_signal") != "A2-T2-PASS":
        errors.append("A2-T2 completion signal is missing")
    parity = _read(directory / "parity-report.json")
    if parity.get("status") != "pass":
        errors.append("A2-T2 parity report is not pass")
    providers = parity.get("providers", {})
    if set(providers) != {"reference.point_mass", "taoryx.native"}:
        errors.append(f"A2-T2 provider set is incomplete: {sorted(providers)}")
    for provider_id, result in providers.items():
        if not result.get("batch_equals_repeated_step"):
            errors.append(f"A2-T2 batch/step parity failed: {provider_id}")
    translations = _read(directory / "translation-report.json")
    for provider_id in providers:
        if provider_id not in translations:
            errors.append(f"missing translation report: {provider_id}")
    return errors
####


def _audit_t3(directory: Path) -> list[str]:
    """Audit control-authority configuration, replay, and telemetry."""

    errors = _audit_manifest(directory)
    status = _read(directory / "status.json")
    if status.get("completion_signal") != "A2-T3-PASS":
        errors.append("A2-T3 completion signal is missing")
    replay = _read(directory / "control-replay-report.json")
    if replay.get("status") != "pass" or replay.get("provider_parity") is not True:
        errors.append("A2-T3 provider replay parity is not pass")
    required_behaviors = set(replay.get("required_behaviors", []))
    expected_behaviors = {"autopilot", "commanded", "overlay", "direct", "cadence_hold", "failsafe", "rate_limit", "activity_mask"}
    if required_behaviors != expected_behaviors:
        errors.append(f"A2-T3 required behavior set is incomplete: {sorted(required_behaviors)}")
    with (directory / "control-arbitration.csv").open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    expected_rows = int(status.get("provider_count", 0)) * int(status.get("frame_count", 0)) * 2
    if len(rows) != expected_rows:
        errors.append(f"A2-T3 arbitration row count {len(rows)} does not equal {expected_rows}")
    if not any(row.get("source") == "hold" for row in rows):
        errors.append("A2-T3 replay did not exercise hold behavior")
    if not any(row.get("source") == "failsafe" for row in rows):
        errors.append("A2-T3 replay did not exercise failsafe behavior")
    if not any(row.get("authority") == "overlay" for row in rows):
        errors.append("A2-T3 replay did not exercise overlay authority")
    if not any(row.get("authority") == "direct" for row in rows):
        errors.append("A2-T3 replay did not exercise direct authority")
    if not any(row.get("rate_limited") == "True" for row in rows):
        errors.append("A2-T3 replay did not exercise rate limiting")
    if not any(row.get("active") == "False" for row in rows):
        errors.append("A2-T3 replay did not exercise activity masking")
    return errors
    ####


def _audit_t4(directory: Path) -> list[str]:
    """Audit metadata-driven Simple Aero generation and native execution."""

    errors = _audit_manifest(directory)
    status = _read(directory / "status.json")
    if status.get("completion_signal") != "A2-T4-PASS":
        errors.append("A2-T4 completion signal is missing")
    if status.get("stable_sweeps") is not True:
        errors.append("A2-T4 sweep reproducibility did not pass")
    negative = _read(directory / "negative-case.json")
    if negative.get("status") != "pass" or negative.get("code") != "out-of-range":
        errors.append(f"A2-T4 negative case did not fail closed: {negative}")
    for case_dir in sorted((directory / "cases").iterdir()):
        if not case_dir.is_dir():
            continue
        required = ("resolved-case.json", "segment-graph.json", "events.csv", "trajectory.csv", "mission-metrics.json", "plot-manifest.json")
        for filename in required:
            if not (case_dir / filename).is_file():
                errors.append(f"A2-T4 case {case_dir.name} is missing {filename}")
        metrics = _read(case_dir / "mission-metrics.json")
        if metrics.get("status") != "pass":
            errors.append(f"A2-T4 case {case_dir.name} metrics are not pass")
        graph = _read(case_dir / "segment-graph.json")
        segments = graph.get("segments", [])
        expected = ["powered_ascent", "ballistic_coast", "bank_maneuver", "terminal_pronav"]
        if [item.get("id") for item in segments] != expected:
            errors.append(f"A2-T4 segment graph is not the declared reusable plan: {case_dir.name}")
        with (case_dir / "events.csv").open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        event_ids = {row.get("event_id") for row in rows}
        if not {"physical_burnout", "commanded_cutoff", "aim_point_configured"}.issubset(event_ids):
            errors.append(f"A2-T4 event annotations are incomplete: {case_dir.name}")
        active_cutoffs = [row for row in rows if row.get("event_id") in {"physical_burnout", "commanded_cutoff"} and row.get("observed") == "True"]
        if len(active_cutoffs) != 1:
            errors.append(f"A2-T4 expected one active cutoff event: {case_dir.name}")
    sweep = _read(directory / "sweep-reproducibility.json")
    if sweep.get("status") != "pass" or not all(row.get("stable") is True for row in sweep.get("rows", [])):
        errors.append("A2-T4 sweep replay rows are not all stable")
    return errors
    ####


def _audit_t5(directory: Path) -> list[str]:
    """Audit the Alpha2 T5 fidelity ladder and its explicit claim split."""

    errors = _audit_manifest(directory)
    status = _read(directory / "status.json")
    if status.get("completion_signal") != "A2-T5-PASS":
        errors.append("A2-T5 completion signal is missing")
    expected = {"point_mass_3dof", "pseudo_6dof", "rigid_body_6dof"}
    if set(status.get("fidelities", [])) != expected:
        errors.append(f"A2-T5 fidelity set is incomplete: {status.get('fidelities')!r}")
    required = (
        "resolved-case.json",
        "fidelity-contract.json",
        "initial-state-adapter.json",
        "command-adapter.json",
        "cross-fidelity-comparison.json",
        "convergence-report.json",
        "rigid-body-evidence.json",
        "fidelity-ladder.png",
    )
    for filename in required:
        if not (directory / filename).is_file():
            errors.append(f"A2-T5 missing required artifact: {filename}")
    contract = _read(directory / "fidelity-contract.json")
    if contract.get("status") != "pass" or contract.get("expected_comparison", {}).get("rigid_body_6dof") != "free-body divergence is expected and measured":
        errors.append("A2-T5 fidelity contract does not declare the parity/divergence split")
    comparison = _read(directory / "cross-fidelity-comparison.json")
    parity = comparison.get("reduction_parity", {})
    if parity.get("status") != "pass" or float(parity.get("max_absolute_error", 1.0)) > float(parity.get("tolerance", 0.0)):
        errors.append("A2-T5 point/pseudo reduction parity did not pass")
    rigid = _read(directory / "rigid-body-evidence.json")
    for key in ("moments_nm", "body_rate_max_rad_s", "envelope_margin_min_deg", "translation_closure_max_normalized", "rotation_closure_max_normalized"):
        if key not in rigid:
            errors.append(f"A2-T5 rigid-body evidence is missing {key}")
    if rigid.get("status") != "pass" or rigid.get("pseudo_is_not_rigid_evidence") is not True:
        errors.append("A2-T5 rigid-body evidence status or pseudo-boundary flag is invalid")
    convergence = _read(directory / "convergence-report.json")
    if convergence.get("status") != "pass" or set(convergence.get("step_pairs", {})) != expected:
        errors.append("A2-T5 convergence report is incomplete")
    for fidelity in sorted(expected):
        trajectory = directory / "runs" / fidelity / "trajectory.csv"
        if not trajectory.is_file():
            errors.append(f"A2-T5 trajectory telemetry is missing: {fidelity}")
    return errors
    ####


def _audit_t6(directory: Path) -> list[str]:
    """Audit the Alpha2 dual-launch glider family and handoff evidence."""

    errors = _audit_manifest(directory)
    status = _read(directory / "status.json")
    if status.get("completion_signal") != "A2-T6-PASS":
        errors.append("A2-T6 completion signal is missing")
    required = ("air-release-case.json", "booster-launch-case.json", "separation-event.json", "shared-comparison.json", "dual-launch-glider.png")
    for filename in required:
        if not (directory / filename).is_file():
            errors.append(f"A2-T6 missing required artifact: {filename}")
    shared = _read(directory / "shared-comparison.json")
    if shared.get("status") != "pass" or shared.get("common_contract_match") is not True:
        errors.append("A2-T6 launch forms do not share a resolved mission contract")
    if shared.get("launch_modes") != ["air_release", "attached_booster"]:
        errors.append(f"A2-T6 launch mode set is incomplete: {shared.get('launch_modes')!r}")
    for filename in ("air-release-case.json", "booster-launch-case.json"):
        case = _read(directory / filename)
        if not case.get("completed") or case.get("diagnostics"):
            errors.append(f"A2-T6 case did not complete cleanly: {filename}")
        trajectory = directory / "runs" / ("air_release" if filename.startswith("air") else "attached_booster") / "trajectory.csv"
        if not trajectory.is_file():
            errors.append(f"A2-T6 trajectory telemetry is missing: {trajectory.relative_to(directory)}")
    separation = _read(directory / "separation-event.json")
    if separation.get("status") != "pass" or separation.get("event_id") != "separation":
        errors.append("A2-T6 separation event is missing or failed")
    if separation.get("state_reset_or_increment_present") is not False:
        errors.append("A2-T6 handoff silently uses a reset or increment")
    policy = separation.get("policy", {})
    if any(policy.get(channel) != "continuous" for channel in ("position", "velocity", "attitude", "rates", "mass")):
        errors.append("A2-T6 separation continuity policy is incomplete")
    attached_problem = directory / "problems" / "attached_booster.prb"
    if attached_problem.is_file():
        source = attached_problem.read_text(encoding="utf-8")
        if "*reset" in source or "*increment" in source:
            errors.append("A2-T6 attached-booster problem contains an undeclared state reset/increment")
        if "*when tseg=" not in source or "attached-booster" not in source:
            errors.append("A2-T6 attached-booster problem does not show a native segment handoff")
    return errors
    ####


def _audit_t7(directory: Path) -> list[str]:
    """Audit the frozen Alpha 2 release packet and isolated replay."""

    errors: list[str] = []
    required = (
        "status.json",
        "schema-reference.json",
        "schema-reference.pdf",
        "claim-matrix.json",
        "reproducibility-report.json",
        "bundle-manifest.json",
        "manifest.json",
        "evidence-packet.zip",
    )
    for filename in required:
        if not (directory / filename).is_file():
            errors.append(f"A2-T7 missing required artifact: {filename}")
    if errors:
        return errors
    status = _read(directory / "status.json")
    if status.get("status") != "pass" or status.get("completion_signal") != "A2-RELEASE-PASS":
        errors.append("A2-T7 completion signal or status is not pass")
    schema = _read(directory / "schema-reference.json")
    if schema.get("freeze", {}).get("status") != "pass":
        errors.append("A2-T7 schema freeze is not pass")
    if schema.get("catalog_sha256") != _sha256(ROOT / "verification/alpha2_family_catalog.yaml"):
        errors.append("A2-T7 schema snapshot catalog hash does not match the source catalog")
    if not schema.get("families"):
        errors.append("A2-T7 schema snapshot has no families")
    if len(directory.joinpath("schema-reference.pdf").read_bytes()) == 0:
        errors.append("A2-T7 schema PDF is empty")
    claim_matrix = _read(directory / "claim-matrix.json")
    if claim_matrix.get("status") != "pass" or claim_matrix.get("gates", {}).get("prior_tranches") != "pass":
        errors.append("A2-T7 claim matrix is not pass")
    for row in claim_matrix.get("rows", []):
        if row.get("id") == "historical_taos_runtime_compatibility" and row.get("status") == "pass":
            errors.append("A2-T7 claim matrix silently claims historical TAOS compatibility")
    replay = _read(directory / "reproducibility-report.json")
    if replay.get("status") != "pass" or replay.get("mode") != "clean_source_snapshot" or replay.get("mismatches"):
        errors.append("A2-T7 clean-source replay did not pass")
    manifest = _read(directory / "manifest.json")
    if manifest.get("status") != "pass" or manifest.get("completion_signal") != "A2-RELEASE-PASS":
        errors.append("A2-T7 release manifest is not pass")
    for item in manifest.get("files", []):
        relative = item.get("path")
        path = directory / relative if isinstance(relative, str) else directory / "__invalid__"
        if not path.is_file():
            errors.append(f"A2-T7 manifest file is missing: {relative}")
        elif path.stat().st_size != item.get("bytes") or _sha256(path) != item.get("sha256"):
            errors.append(f"A2-T7 manifest hash mismatch: {relative}")
    bundle = _read(directory / "bundle-manifest.json")
    if bundle.get("status") != "pass" or bundle.get("source_catalog_sha256") != schema.get("catalog_sha256"):
        errors.append("A2-T7 bundle manifest is not pass or has a catalog hash mismatch")
    packet_path = directory / "evidence-packet.zip"
    try:
        with zipfile.ZipFile(packet_path) as archive:
            names = set(archive.namelist())
            prefix = "taoryx-alpha2-release-v1/"
            required_packet = {prefix + name for name in ("bundle-manifest.json", "README.md", "release/schema-reference.json", "release/schema-reference.pdf", "release/claim-matrix.json", "release/reproducibility-report.json", "alpha2-status.json")}
            missing = sorted(required_packet - names)
            if missing:
                errors.append(f"A2-T7 packet is missing required entries: {missing}")
            for name in names:
                if name.endswith((".json", ".yaml", ".yml", ".md", ".prb", ".py", ".toml", ".txt")):
                    text = archive.read(name).decode("utf-8")
                    if re.search(r"/Users/[A-Za-z0-9_.-]+/", text) or re.search(r"/private/(?:tmp|var)/[A-Za-z0-9_.-]+", text):
                        errors.append(f"A2-T7 packet contains an absolute workstation path: {name}")
    except (OSError, zipfile.BadZipFile) as error:
        errors.append(f"A2-T7 evidence packet cannot be read: {error}")
    return errors
    ####


def main() -> int:
    """Validate both tranche directories and emit a compact report."""

    tranche_errors = {
        "A2-T1": _audit_t1(ARTIFACT_ROOT / "t1_case_contracts"),
        "A2-T2": _audit_t2(ARTIFACT_ROOT / "t2_provider_session"),
        "A2-T3": _audit_t3(ARTIFACT_ROOT / "t3_control_authority"),
        "A2-T4": _audit_t4(ARTIFACT_ROOT / "t4_simple_aero_3dof"),
        "A2-T5": _audit_t5(ARTIFACT_ROOT / "t5_fidelity_ladder"),
        "A2-T6": _audit_t6(ARTIFACT_ROOT / "t6_dual_launch_glider"),
    }
    t7_directory = ARTIFACT_ROOT / "t7_release"
    if t7_directory.is_dir():
        tranche_errors["A2-T7"] = _audit_t7(t7_directory)
    status = "pass" if not any(tranche_errors.values()) else "blocked"
    completion_signal = "A2-RELEASE-PASS" if "A2-T7" in tranche_errors and status == "pass" else "A2-T6-PASS" if status == "pass" else None
    report = {
        "release": "taoryx-alpha-2",
        "status": status,
        "completion_signal": completion_signal,
        "claim_boundary": "successor-side Alpha 2 contracts and proof families only; historical TAOS compatibility, global vehicle validity, flight qualification, and universal autopilot excluded",
        "tranches": tranche_errors,
    }
    output = ARTIFACT_ROOT / "alpha2-status.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if status == "pass" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
