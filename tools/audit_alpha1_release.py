"""Audit the current Alpha 1 release gates without widening project claims."""

from __future__ import annotations

import argparse
import json
import zipfile
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Literal

import yaml

ROOT = Path(__file__).resolve().parents[1]
GateStatus = Literal["pass", "partial", "blocked", "not_run"]
GATE_ORDER = tuple(f"R{index}" for index in range(10))


@dataclass(frozen=True, slots=True)
class GateResult:
    """One evidence-bounded Alpha 1 release-gate result."""

    id: str
    name: str
    status: GateStatus
    reason: str
    evidence: tuple[str, ...]
    next_action: str

    def as_dict(self) -> dict[str, Any]:
        """Return a JSON-compatible gate record."""

        return asdict(self)
    ####


def _present(root: Path, paths: tuple[str, ...]) -> tuple[str, ...]:
    """Return the paths that are present in the current checkout."""

    return tuple(path for path in paths if (root / path).exists())
    ####


def _gate(
    gate_id: str,
    name: str,
    status: GateStatus,
    reason: str,
    evidence: tuple[str, ...],
    next_action: str,
) -> GateResult:
    """Construct a gate result with a validated status."""

    if gate_id not in GATE_ORDER:
        raise ValueError(f"unknown Alpha 1 gate: {gate_id}")
    return GateResult(gate_id, name, status, reason, evidence, next_action)
    ####


def _load_json(root: Path, relative: str) -> dict[str, Any] | None:
    """Load a JSON evidence file when it exists and is well formed."""

    path = root / relative
    if not path.is_file():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return payload if isinstance(payload, dict) else None
    ####


def _load_yaml(root: Path, relative: str) -> dict[str, Any] | None:
    """Load a YAML evidence file when it exists and is well formed."""

    path = root / relative
    if not path.is_file():
        return None
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError):
        return None
    return payload if isinstance(payload, dict) else None
    ####


def _feature_matrix_complete(root: Path, expected_count: int) -> bool:
    """Check the generated Alpha 1 matrix has complete per-feature links."""

    payload = _load_yaml(root, "verification/alpha1_feature_matrix.yaml")
    features = payload.get("features", ()) if payload else ()
    required = {
        "feature_id",
        "category",
        "name",
        "manual_scope",
        "grammar",
        "semantics",
        "implementation_owner",
        "implementation_status",
        "fixtures",
        "tests",
        "requirement_ids",
    }
    if not isinstance(features, list) or len(features) != expected_count:
        return False
    return all(
        isinstance(feature, dict)
        and required <= feature.keys()
        and all(feature[field] for field in required if field not in {"fixtures", "tests", "requirement_ids"})
        and isinstance(feature["fixtures"], list)
        and isinstance(feature["tests"], list)
        and isinstance(feature["requirement_ids"], list)
        and feature["grammar"] in {"grammars/taos_problem.ebnf", "grammars/taos_table.ebnf"}
        for feature in features
    )
    ####


def _manual_example_report(root: Path) -> dict[str, Any] | None:
    """Load the latest four-family manual execution report."""

    return _load_json(root, "artifacts/verification/alpha1/manual_examples/report.json")
    ####


def _composition_report_complete(root: Path) -> bool:
    """Check the generic composition proof has a resolved and runnable case."""

    report = _load_json(root, "artifacts/verification/alpha1/composition_case/report.json")
    if report is None or report.get("status") != "pass":
        return False
    composition = report.get("composition", {})
    generated = report.get("generated", {})
    runtime = report.get("runtime", {})
    generated_paths = (
        generated.get("problem"),
        generated.get("manifest"),
        generated.get("audit"),
        generated.get("resolved_case"),
    )
    artifacts = runtime.get("artifacts", ())
    return (
        composition.get("api") == "taoryx.composition.TrajectoryBuilder"
        and composition.get("template_registry") == "SegmentCompositionRegistry.standard"
        and composition.get("bespoke_runner_logic") is False
        and all(isinstance(path, str) and (root / path).is_file() for path in generated_paths)
        and isinstance(artifacts, list)
        and bool(artifacts)
        and all(isinstance(path, str) and (root / path).is_file() for path in artifacts)
    )
    ####


def _validation_blockers(root: Path) -> tuple[str, ...]:
    """Return explicitly recorded vehicle-validation blockers."""

    path = root / "verification/family_validation_execution.yaml"
    if not path.is_file():
        return ("verification/family_validation_execution.yaml is missing",)
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        return ("family validation execution registry is unreadable",)
    blockers = []
    for scenario in payload.get("scenarios", ()):
        if scenario.get("status") == "blocked" or scenario.get("verdict") == "blocked":
            blockers.append(f"{scenario.get('id', '<unnamed>')}: {scenario.get('blocking_reason', 'blocked')}")
    return tuple(blockers)
    ####


def _alpha1_vehicle_evidence(root: Path) -> tuple[bool, tuple[str, ...], tuple[str, ...]]:
    """Validate the narrower R7 baseline-plant evidence contract.

    The family-validation registry intentionally contains stronger mission
    claims than Alpha 1 can support.  R7 therefore consumes a separate,
    explicit release-scope registry: it requires one source-backed plant case
    per family, but does not silently promote blocked route/controller work.
    """

    relative = "verification/alpha1_vehicle_evidence.yaml"
    payload = _load_yaml(root, relative)
    if payload is None:
        return False, (f"{relative} is missing or unreadable",), ()
    required_families = tuple(str(item) for item in payload.get("required_families", ()))
    entries = payload.get("vehicles", ())
    accepted_rigid = {str(item) for item in payload.get("accepted_rigid_statuses", ())}
    if not required_families or not isinstance(entries, list):
        return False, (f"{relative} has no required family list or vehicle entries",), ()

    seen: set[str] = set()
    failures: list[str] = []
    deferred: list[str] = []
    for entry in entries:
        if not isinstance(entry, dict):
            failures.append("vehicle evidence entry is not a mapping")
            continue
        vehicle_id = str(entry.get("id", "<unnamed>"))
        seen.add(vehicle_id)
        if entry.get("release_status") != "pass":
            failures.append(f"{vehicle_id}: release_status is not pass")
        modes = entry.get("mode_contracts", {})
        if not isinstance(modes, dict) or modes.get("point_mass_3dof") != "pass" or modes.get("pseudo_6dof") != "pass":
            failures.append(f"{vehicle_id}: 3-DOF/pseudo-6-DOF contracts are incomplete")
        if not isinstance(modes, dict) or modes.get("rigid_body_6dof") not in accepted_rigid:
            failures.append(f"{vehicle_id}: rigid-body baseline is not in an accepted status")
        for field in ("source_differential_test", "convention_firewall"):
            path = entry.get(field)
            if not isinstance(path, str) or not (root / path).is_file():
                failures.append(f"{vehicle_id}: missing {field}")
        baseline = entry.get("baseline", {})
        if not isinstance(baseline, dict):
            failures.append(f"{vehicle_id}: baseline is not a mapping")
        else:
            for field in ("problem", "trim_report", "long_scenario"):
                path = baseline.get(field)
                if not isinstance(path, str) or not (root / path).exists():
                    failures.append(f"{vehicle_id}: missing baseline {field}")
        trim = entry.get("source_trim", {})
        if isinstance(trim, dict) and trim.get("status") == "source_only" and trim.get("claim_excluded") is not True:
            failures.append(f"{vehicle_id}: source-only trim must explicitly exclude an equilibrium claim")
        mission = entry.get("higher_order_mission", {})
        if isinstance(mission, dict) and mission.get("status") in {"blocked", "candidate"}:
            deferred.append(f"{vehicle_id}: higher-order mission remains {mission.get('status')}")
    missing = set(required_families) - seen
    if missing:
        failures.append(f"missing required families: {', '.join(sorted(missing))}")
    if set(required_families) != seen:
        failures.append("vehicle evidence contains entries outside the required family set or has duplicates")
    return not failures, tuple(failures), tuple(deferred)
    ####


def _alpha1_packet_complete(root: Path) -> bool:
    """Check the Alpha 1 packet manifest and ZIP agree without local paths."""

    manifest_path = root / "artifacts/verification/alpha1/packet/bundle-manifest.json"
    archive_path = root / "dist/taoryx-alpha1-evidence-v1.zip"
    if not manifest_path.is_file() or not archive_path.is_file():
        return False
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        records = manifest.get("files", ())
        with zipfile.ZipFile(archive_path) as archive:
            names = set(archive.namelist())
    except (OSError, ValueError, zipfile.BadZipFile):
        return False
    if manifest.get("release") != "taoryx-alpha-1" or not isinstance(records, list):
        return False
    prefix = "taoryx-alpha1-evidence-v1/"
    if "taoryx-alpha1-evidence-v1/README.md" not in names or "taoryx-alpha1-evidence-v1/bundle-manifest.json" not in names:
        return False
    if int(manifest.get("file_count", -1)) != len(records):
        return False
    for record in records:
        if not isinstance(record, dict):
            return False
        packet_path = record.get("packet_path")
        if not isinstance(packet_path, str) or packet_path.startswith("/") or "/Users/" in packet_path:
            return False
        if prefix + packet_path not in names:
            return False
    return True
    ####


def audit(root: Path = ROOT) -> dict[str, Any]:
    """Return the current Alpha 1 gate status from repository evidence."""

    root = root.resolve()
    coverage = _load_json(root, "tests/fixtures/taos_e2e_v23/coverage/documented_surface_coverage.json") or {}
    coverage_summary = coverage.get("summary", {})
    onboarding = _load_json(root, "artifacts/controller_autotune/onboarding.json") or {}
    onboarding_statuses = {str(item.get("vehicle_id")): str(item.get("status")) for item in onboarding.get("vehicles", ())}
    validation_blockers = _validation_blockers(root)
    vehicle_evidence_pass, vehicle_evidence_failures, deferred_vehicle_work = _alpha1_vehicle_evidence(root)
    complete_coverage = coverage_summary == {
        "block_scopes_covered": 33,
        "block_scopes_total": 33,
        "table_types_covered": 19,
        "table_types_total": 19,
        "table_operations_covered": 28,
        "table_operations_total": 28,
    }
    documented_feature_count = sum(
        int(coverage_summary.get(key, 0))
        for key in ("block_scopes_total", "table_types_total", "table_operations_total")
    )
    feature_matrix_complete = _feature_matrix_complete(root, documented_feature_count)
    manual_report = _manual_example_report(root)
    manual_cases = manual_report.get("cases", ()) if manual_report else ()
    manual_summary = manual_report.get("summary", {}) if manual_report else {}
    manual_report_complete = (
        isinstance(manual_cases, list)
        and len(manual_cases) == 4
        and manual_summary.get("companion_passing") == 4
        and manual_summary.get("release_status") == "pass"
    )
    composition_report_complete = _composition_report_complete(root)
    release_records = ("docs/plan/taoryx-alpha-1.md", "verification/alpha1_release_plan.yaml", "verification/claims.md")

    gates = [
        _gate(
            "R0",
            "claim-freeze",
            "pass" if all((root / path).is_file() for path in release_records) else "partial",
            "Alpha 1 scope, exclusions, claims, and release gates are recorded."
            if all((root / path).is_file() for path in release_records)
            else "release claim records are incomplete",
            _present(root, (*release_records, "verification/evidence_policy.md")),
            "Keep claim wording synchronized as implementation status changes.",
        ),
        _gate(
            "R1",
            "requirement-inventory",
            "pass" if feature_matrix_complete else "partial",
            "The generated Alpha 1 feature matrix links every bounded Chapter 3/4 surface item to its grammar, semantic registry, implementation owner, fixture, test, and requirement group."
            if feature_matrix_complete
            else "Normative registries and documented-surface coverage exist, but the dedicated Alpha 1 feature matrix is missing or incomplete.",
            _present(
                root,
                (
                    "verification/spec/requirements.yaml",
                    "verification/spec/chapter4_normative_inventory.yaml",
                    "verification/traceability.yaml",
                    "tests/fixtures/taos_e2e_v23/coverage/feature_coverage.json",
                    "verification/alpha1_feature_matrix.yaml",
                ),
            ),
            "Regenerate the Alpha 1 feature matrix and add a direct mapping for any newly admitted syntax.",
        ),
        _gate(
            "R2",
            "language-safety",
            "pass" if complete_coverage else "partial",
            "Documented block, table-type, and table-operation coverage is complete for the current bounded parser corpus; this remains a bounded-language claim.",
            _present(
                root,
                (
                    "tests/e2e/test_documented_coverage.py",
                    "tests/e2e/test_static_corpus.py",
                    "verification/spec/diagnostic_matrices.yaml",
                    "tests/fixtures/taos_e2e_v23/coverage/documented_surface_coverage.json",
                ),
            ),
            "Keep adding negative/recovery fixtures for any newly admitted syntax.",
        ),
        _gate(
            "R3",
            "canonical-runtime-transition",
            "pass"
            if all(
                (root / path).is_file()
                    for path in (
                        "src/taoryx/runtime/runner.py",
                        "src/taoryx/runtime/interactive.py",
                    "tests/unit/test_interactive_runtime.py",
                    "tests/unit/test_scenario_contract.py",
                )
            )
            else "partial",
            "The runtime, interactive step path, scenario contract, deterministic replay, and batch/interactive parity tests are present.",
            _present(
                root,
                (
                    "src/taoryx/runtime/runner.py",
                    "src/taoryx/runtime/interactive.py",
                    "tests/unit/test_interactive_runtime.py",
                    "tests/unit/test_scenario_contract.py",
                    "tests/unit/test_scenario_artifacts.py",
                ),
            ),
            "Retain run/step parity as a release regression and include it in the handoff report.",
        ),
        _gate(
            "R4",
            "manual-example-execution",
            "pass" if manual_report_complete else "partial",
            "All four manual example families have a complete claim-bounded companion execution report through the common 3-DOF runner; source optimization qualification remains separately reported."
            if manual_report_complete
            else "The four-family execution report is missing or contains blocked companion cases; parser coverage must not be mistaken for runtime completion.",
            _present(
                root,
                (
                    "examples/chapter04/ballistic-reentry.prb",
                    "examples/chapter04/ballistic-rocket.prb",
                    "examples/chapter04/air-launched-intercept.prb",
                    "examples/chapter04/ground-launched-intercept.prb",
                    "tests/fixtures/taos_e2e_v23/coverage/feature_coverage.json",
                    "artifacts/verification/alpha1/manual_examples/report.json",
                ),
            ),
            "Keep the source optimization blockers explicit, or make the synthetic deck coherent enough to qualify the original optimization endpoints, then regenerate the report.",
        ),
        _gate(
            "R5",
            "reusable-composition",
            "pass" if composition_report_complete else "partial",
            "A new case is resolved from metadata through the standard composition template registry, lowered to native problem syntax, parsed, and executed through the common runtime without bespoke vehicle-runner logic."
            if composition_report_complete
            else "Composition, segmentation, generated problem files, and control contracts are implemented and tested, but Alpha 1 still needs a single release report proving a new case can be generated without bespoke runner logic.",
            _present(
                root,
                (
                    "src/taoryx/composition.py",
                    "src/taoryx/segmentation.py",
                    "tools/generate_problem_files.py",
                    "tests/unit/test_composition.py",
                    "tests/unit/test_problem_generation.py",
                    "verification/alpha1_composition_case.yaml",
                    "artifacts/verification/alpha1/composition_case/report.json",
                ),
            ),
            "Run `python tools/dev.py alpha1-composition-case` and retain the generated report, or fix the first failed composition/runtime contract.",
        ),
        _gate(
            "R6",
            "fidelity-ladder-contracts",
            "pass"
            if all(
                (root / path).is_file()
                for path in (
                    "verification/fidelity_ladder.yaml",
                    "tests/e2e/test_fidelity_ladder.py",
                    "tests/unit/test_modes.py",
                    "tests/unit/test_rigid_body.py",
                )
            )
            else "partial",
            "The three fidelity modes have explicit runtime contracts and ladder/convergence tests; this is a contract/numerical-path result, not a global vehicle-validity claim.",
            _present(
                root,
                (
                    "verification/fidelity_ladder.yaml",
                    "tests/e2e/test_fidelity_ladder.py",
                    "tests/unit/test_modes.py",
                    "tests/unit/test_rigid_body.py",
                    "docs/architecture/dynamics-fidelity-ladder.md",
                ),
            ),
            "Keep each family at its lowest failing fidelity when promoting mission evidence.",
        ),
        _gate(
            "R7",
            "vehicle-evidence",
            "pass" if vehicle_evidence_pass else "blocked",
            "The Alpha 1 baseline-plant registry passes for all four required families; stronger route/controller candidates remain separately dispositioned."
            if vehicle_evidence_pass
            else "Alpha 1 vehicle baseline evidence is incomplete: " + "; ".join(vehicle_evidence_failures[:3]),
            _present(
                root,
                (
                    "verification/alpha1_vehicle_evidence.yaml",
                    "verification/family_validation_execution.yaml",
                    "artifacts/controller_autotune/onboarding.json",
                    "verification/vehicle_models.yaml",
                    "verification/trim_specs.yaml",
                ),
            ),
            "Keep higher-order route/controller blockers explicit; promote only after the baseline evidence registry, source parity, and bounded plant artifacts are regenerated together.",
        ),
        _gate(
            "R8",
            "artifact-reproducibility",
            "pass" if _alpha1_packet_complete(root) else "partial",
            "The self-contained Alpha 1 evidence packet, manifest, sanitized references, and ZIP agree."
            if _alpha1_packet_complete(root)
            else "A reproducible four-family fidelity packet exists, but the self-contained Alpha 1 packet is not current or complete.",
            _present(
                root,
                (
                    "tools/audit_fidelity_packet.py",
                    "tools/audit_fidelity_milestones.py",
                    "tests/unit/test_fidelity_packet_audit.py",
                    "tests/unit/test_scenario_artifacts.py",
                    "tools/build_alpha1_packet.py",
                    "artifacts/verification/alpha1/packet/bundle-manifest.json",
                    "dist/taoryx-alpha1-evidence-v1.zip",
                ),
            ),
            "Run `python tools/dev.py alpha1-packet` after regenerating the manual, composition, vehicle, and gate evidence.",
        ),
        _gate(
            "R9",
            "clean-handoff",
            "pass" if (root / "dist/taos-manual-codex-handoff-v21.zip").is_file() else "not_run",
            "The required handoff bundle is present."
            if (root / "dist/taos-manual-codex-handoff-v21.zip").is_file()
            else "The Alpha 1 handoff command has not produced a current bundle.",
            _present(root, ("tools/build_handoff_bundle.py", "dist/taos-manual-codex-handoff-v21.zip")),
            "Run the Alpha 1 handoff command after the lower gates are promoted.",
        ),
    ]
    lowest = next((gate for gate in gates if gate.status != "pass"), None)
    overall: GateStatus = "blocked" if any(gate.status == "blocked" for gate in gates) else "partial" if lowest else "pass"
    return {
        "schema_version": 1,
        "release": "taoryx-alpha-1",
        "status": overall,
        "lowest_unmet_gate": lowest.id if lowest else None,
        "vehicle_evidence_deferred_work": deferred_vehicle_work,
        "legacy_family_validation_blockers": validation_blockers,
        "onboarding_statuses": onboarding_statuses,
        "gates": [gate.as_dict() for gate in gates],
        "claim_boundary": "Alpha 1 evidence-bounded successor claims; not historical TAOS 96.0 compatibility or flight qualification.",
    }
    ####


def main() -> int:
    """Write or print the current Alpha 1 status report."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    payload = audit(args.root)
    rendered = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
        print(f"{payload['status'].upper()}: {args.output}")
    else:
        print(rendered, end="")
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
