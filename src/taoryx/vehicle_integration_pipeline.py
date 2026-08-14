"""Provider-neutral staged integration reports for reference vehicle families.

The readiness validator answers whether a family declares the metadata needed
to begin runtime work.  This module composes that result with the existing
source import, operational-contract, integration-record, and evidence files.
It deliberately does not run or synthesize a vehicle model.  A missing source
layer remains a blocker, while a development artifact remains development
evidence.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import shutil
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, cast

import yaml

from .trajectory.daveml_import import DAVEMLFamilyImport, load_daveml_family_import
from .trajectory.reference_families import ReferenceFamilyManifest, load_reference_family_manifest
from .vehicle_controller_mission_preflight import validate_vehicle_controller_mission_preflight
from .vehicle_effectivity_preflight import validate_vehicle_effectivity_preflight
from .vehicle_integration_readiness import (
    VehicleIntegrationReadinessReport,
    validate_all_vehicle_integration_readiness,
    validate_vehicle_integration_readiness,
)
from .vehicle_registry import ROOT
from .vehicle_trim_adapters import solve_vehicle_trim_evidence
from .vehicle_trim_orchestration import orchestrate_trim_recipe

StageStatus = Literal["passed", "development", "blocked", "planned", "not_applicable"]
STAGE_ORDER: tuple[str, ...] = (
    "intake",
    "conventions",
    "plant",
    "effectivity",
    "trim",
    "operating_points",
    "fidelity",
    "controller",
    "mission",
)
VERIFIED_WORDS = frozenset({"verified", "passed", "qualification_passed", "runtime_replay_qualification_passed"})
DEVELOPMENT_WORDS = frozenset({"development", "screen", "candidate", "contract_ready", "nominal_case_pass"})
BLOCKED_WORDS = frozenset({"blocked", "missing", "pending", "not_declared", "not_available", "failed"})


@dataclass(frozen=True, slots=True)
class IntegrationPipelineFinding:
    """One staged integration finding with an actionable remedy."""

    severity: Literal["error", "warning", "info"]
    code: str
    path: str
    message: str
    hint: str

    def as_dict(self) -> dict[str, str]:
        """Return a stable JSON representation."""

        return {
            "severity": self.severity,
            "code": self.code,
            "path": self.path,
            "message": self.message,
            "hint": self.hint,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class IntegrationStageReport:
    """Result of one provider-neutral integration stage."""

    stage_id: str
    status: StageStatus
    claim: str
    evidence: tuple[str, ...]
    findings: tuple[IntegrationPipelineFinding, ...]
    metrics: Mapping[str, object]

    @property
    def blockers(self) -> tuple[IntegrationPipelineFinding, ...]:
        """Return stage findings that prevent promotion."""

        return tuple(item for item in self.findings if item.severity == "error")
        ####

    def as_dict(self) -> dict[str, object]:
        """Return a stable JSON representation."""

        return {
            "stage_id": self.stage_id,
            "status": self.status,
            "claim": self.claim,
            "evidence": list(self.evidence),
            "metrics": dict(self.metrics),
            "error_count": len(self.blockers),
            "findings": [item.as_dict() for item in self.findings],
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class VehicleIntegrationPipelineReport:
    """Complete staged integration and promotion report for one family."""

    family_id: str
    display_name: str
    physical_family: str
    status: str
    current_tier: str
    next_gate: str
    promotion_eligible: bool
    claim_boundary: str
    stages: tuple[IntegrationStageReport, ...]
    blockers: tuple[IntegrationPipelineFinding, ...]
    caveats: tuple[IntegrationPipelineFinding, ...]

    def as_dict(self) -> dict[str, object]:
        """Return the self-contained machine-readable report."""

        return {
            "schema_version": "taoryx.vehicle-integration-pipeline/v1",
            "family_id": self.family_id,
            "display_name": self.display_name,
            "physical_family": self.physical_family,
            "status": self.status,
            "current_tier": self.current_tier,
            "next_gate": self.next_gate,
            "promotion_eligible": self.promotion_eligible,
            "claim_boundary": self.claim_boundary,
            "stages": [stage.as_dict() for stage in self.stages],
            "blockers": [item.as_dict() for item in self.blockers],
            "caveats": [item.as_dict() for item in self.caveats],
        }
        ####
    ####


def _finding(
    severity: Literal["error", "warning", "info"],
    code: str,
    path: str,
    message: str,
    hint: str,
) -> IntegrationPipelineFinding:
    return IntegrationPipelineFinding(severity, code, path, message, hint)
    ####


def _read_yaml(path: Path) -> Mapping[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{path} must contain a mapping")
    return payload
    ####


def _read_json(path: Path) -> Mapping[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{path} must contain a mapping")
    return payload
    ####


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)
    ####


def _evidence_path(value: object) -> Path | None:
    label = str(value).split("#", 1)[0].strip()
    if not label or label in {"not_applicable", "not_available", "none"}:
        return None
    if not any(token in label for token in ("/", ".json", ".yaml", ".yml", ".csv", ".tbl", ".prb", ".zip")):
        return None
    path = Path(label)
    return path if path.is_absolute() else ROOT / path
    ####


def _status_word(value: object) -> str:
    return str(value).strip().lower().replace("-", "_").replace(" ", "_")
    ####


def _classify_declared_status(value: object) -> StageStatus:
    status = _status_word(value)
    if "not_applicable" in status or status.startswith("not_applicable"):
        return "not_applicable"
    if any(word in status for word in BLOCKED_WORDS):
        return "blocked"
    if any(word in status for word in VERIFIED_WORDS) and "development" not in status:
        return "passed"
    if any(word in status for word in DEVELOPMENT_WORDS):
        return "development"
    if not status or status in {"planned", "not_attempted"}:
        return "planned"
    return "development"
    ####


def _compare_numeric(
    findings: list[IntegrationPipelineFinding],
    path: str,
    left: object,
    right: object,
    label: str,
    tolerance: float = 1.0e-9,
) -> None:
    try:
        if not isinstance(left, (int, float, str)) or not isinstance(right, (int, float, str)):
            raise TypeError("numeric comparison requires scalar values")
        left_value = float(left)
        right_value = float(right)
    except (TypeError, ValueError):
        findings.append(_finding("error", "convention-nonnumeric", path, f"{label} is not numeric in both records", "Keep canonical geometry and envelope values in SI numeric form."))
        return
    if not math.isclose(left_value, right_value, rel_tol=tolerance, abs_tol=tolerance):
        findings.append(_finding("error", "convention-mismatch", path, f"{label} differs: {left_value} != {right_value}", "Regenerate the derived record from the same pinned source manifest; do not silently choose one value."))
    ####


def _canonical_convention(value: object) -> str:
    """Normalize equivalent prose/list convention encodings for comparison."""

    if isinstance(value, (list, tuple)):
        value = "_".join(str(item) for item in value)
    return re.sub(r"[^a-z0-9]", "", str(value).lower())
    ####


def _convention_compatible(key: str, source_value: object, package_value: object) -> bool:
    """Compare source and imported convention prose without hiding semantics."""

    source = _canonical_convention(source_value)
    package = _canonical_convention(package_value)
    if key == "source_force_frame":
        return source in package or package in source
    if key == "force_channels":
        known_channels = ("cx", "cy", "cz", "cl", "cd", "cmx", "cmy", "cmz")
        source_channels = {item for item in known_channels if item in source}
        package_channels = {item for item in known_channels if item in package}
        return package_channels <= source_channels
    if key == "drag_representation":
        semantic_tokens = ("bodyaxiscx", "windaxiscd", "windaxiscl", "directwindaxiscd")
        return {item for item in semantic_tokens if item in source} == {item for item in semantic_tokens if item in package}
    return source == package
    ####


def _stage(
    stage_id: str,
    status: StageStatus,
    claim: str,
    evidence: list[str],
    findings: list[IntegrationPipelineFinding],
    metrics: Mapping[str, object] | None = None,
) -> IntegrationStageReport:
    return IntegrationStageReport(stage_id, status, claim, tuple(evidence), tuple(findings), metrics or {})
    ####


def _load_integration_record(manifest_path: Path) -> tuple[Path, Mapping[str, Any]]:
    path = manifest_path.parent / "qualification/integration-record.yaml"
    return path, _read_yaml(path)
    ####


def _intake_stage(
    readiness: VehicleIntegrationReadinessReport,
    manifest_path: Path,
    import_path: Path,
    record_path: Path,
) -> IntegrationStageReport:
    findings = [
        _finding("error", finding.code, finding.path, finding.message, finding.hint)
        for finding in readiness.errors
    ]
    evidence = [_relative(manifest_path), _relative(import_path), _relative(record_path)]
    status: StageStatus = "blocked" if findings else "passed"
    claim = "family manifest, source import record, and integration record are discoverable and readiness-valid"
    return _stage("intake", status, claim, evidence, findings, {"readiness_status": readiness.status})
    ####


def _convention_stage(
    manifest: ReferenceFamilyManifest,
    manifest_path: Path,
    import_record: DAVEMLFamilyImport,
    import_path: Path,
    contract: Mapping[str, Any] | None,
    contract_path: Path,
) -> IntegrationStageReport:
    findings: list[IntegrationPipelineFinding] = []
    package = import_record.package
    plant = manifest.plant
    if package.frames.get("body") != plant.body_frame:
        findings.append(_finding("error", "body-frame-mismatch", _relative(import_path), "source import and family plant disagree on body frame", "Repair the binding or regenerate the import record; never auto-rotate a source plant."))
    if package.frames.get("navigation") != plant.navigation_frame:
        findings.append(_finding("error", "navigation-frame-mismatch", _relative(import_path), "source import and family plant disagree on navigation frame", "Declare one tested navigation frame in both records."))
    if package.frames.get("quaternion_order") != plant.quaternion_order:
        findings.append(_finding("error", "quaternion-order-mismatch", _relative(import_path), "source import and family plant disagree on quaternion order", "Add an explicit adapter or correct the metadata before integration."))
    for key, value in (("area_m2", plant.reference_geometry.area_m2), ("mean_aerodynamic_chord_m", plant.reference_geometry.mean_aerodynamic_chord_m), ("span_m", plant.reference_geometry.span_m)):
        _compare_numeric(findings, _relative(manifest_path), package.reference_geometry.get(key), value, f"reference geometry {key}")
    for key, value in plant.validity_envelope.model_dump().items():
        _compare_numeric(findings, _relative(manifest_path), package.validity_envelope.get(key), value, f"validity envelope {key}")
    source_convention = manifest.source.aerodynamic_convention.model_dump()
    package_convention = package.aerodynamic_convention.model_dump()
    for key, value in source_convention.items():
        if key == "provenance_note" or key not in package_convention:
            continue
        package_value = package_convention.get(key)
        if not _convention_compatible(key, value, package_value):
            findings.append(_finding("error", "aero-convention-mismatch", _relative(import_path), f"aerodynamic convention {key!r} differs between family and import records", "Preserve the source force channels and composition rule exactly."))
    contract_metrics: dict[str, object] = {"contract_checked": contract is not None}
    if contract is None:
        findings.append(_finding("error", "operational-contract-missing", _relative(contract_path), "no operational contract is declared for this family", "Add the family to the provider-neutral operational contract registry."))
    else:
        frames = contract.get("frames")
        if isinstance(frames, Mapping):
            for key, expected_frame in (("body", plant.body_frame), ("navigation", plant.navigation_frame), ("quaternion_order", plant.quaternion_order)):
                if frames.get(key) != expected_frame:
                    findings.append(_finding("error", "contract-frame-mismatch", _relative(contract_path), f"operational contract {key!r} disagrees with family plant", "Regenerate the contract from the canonical family manifest."))
        geometry = contract.get("reference_geometry")
        if isinstance(geometry, Mapping):
            for key, value in (("area_m2", plant.reference_geometry.area_m2), ("mean_aerodynamic_chord_m", plant.reference_geometry.mean_aerodynamic_chord_m), ("span_m", plant.reference_geometry.span_m)):
                _compare_numeric(findings, _relative(contract_path), geometry.get(key), value, f"operational geometry {key}")
        contract_metrics["binding_count"] = len(contract.get("bindings", ())) if isinstance(contract.get("bindings"), list) else 0
    control_ids = [control.id for control in manifest.controls]
    observation_ids = [observation.id for observation in manifest.observations]
    if len(control_ids) != len(set(control_ids)):
        findings.append(_finding("error", "duplicate-control-id", _relative(manifest_path), "family manifest contains duplicate control IDs", "Give each semantic control one stable identifier."))
    if len(observation_ids) != len(set(observation_ids)):
        findings.append(_finding("error", "duplicate-observation-id", _relative(manifest_path), "family manifest contains duplicate observation IDs", "Give each observation one stable identifier."))
    status: StageStatus = "blocked" if any(item.severity == "error" for item in findings) else "passed"
    return _stage("conventions", status, "frames, geometry, envelopes, aerodynamic conventions, and semantic IDs agree across source records", [_relative(manifest_path), _relative(import_path), _relative(contract_path)], findings, contract_metrics)
    ####


def _plant_stage(manifest: ReferenceFamilyManifest, import_record: DAVEMLFamilyImport, import_path: Path) -> IntegrationStageReport:
    findings: list[IntegrationPipelineFinding] = []
    replay = import_record.replay
    roundtrip = import_record.roundtrip
    if replay.status != "runtime_replay_qualification_passed":
        findings.append(_finding("error", "plant-replay-not-qualified", _relative(import_path), f"source replay status is {replay.status!r}", "Run the family-specific source replay and retain its evidence before plant promotion."))
    if roundtrip.status != "verified" or roundtrip.structural_diff_count or roundtrip.numeric_diff_count or roundtrip.checkdata_failed:
        findings.append(_finding("error", "source-roundtrip-not-verified", _relative(import_path), "canonical source roundtrip or checkdata evidence is not verified", "Complete the source import/roundtrip gate; do not lower fidelity around a broken parent plant."))
    if not math.isfinite(replay.force_moment_residual) or replay.force_moment_residual < 0.0:
        findings.append(_finding("error", "invalid-force-moment-residual", _relative(import_path), "plant replay residual is not a finite nonnegative number", "Record an independent closure residual from the runtime replay."))
    status: StageStatus = "blocked" if findings else "passed"
    return _stage("plant", status, "the pinned source plant loads, round-trips, and reproduces its declared runtime load contract", [_relative(import_path)], findings, {"source_document_count": len(import_record.package.source_documents), "roundtrip_status": roundtrip.status, "replay_status": replay.status, "force_moment_residual": replay.force_moment_residual})
    ####


def _layer_stage(
    stage_id: str,
    claim: str,
    record: Mapping[str, Any],
    manifest: ReferenceFamilyManifest,
    layer_names: tuple[str, ...],
) -> IntegrationStageReport:
    findings: list[IntegrationPipelineFinding] = []
    evidence: list[str] = []
    statuses: list[StageStatus] = []
    layers: dict[str, Any] = {}
    raw_layers = record.get("layers")
    if isinstance(raw_layers, Mapping):
        layers.update(raw_layers)
    layers.update({key: value.model_dump() for key, value in ((name, manifest.layers.get(name)) for name in layer_names) if value is not None})
    selected_layers = tuple((name, layers.get(name)) for name in layer_names if layers.get(name) is not None)
    if not selected_layers:
        return _stage(stage_id, "planned", claim, [], [_finding("error", "layer-not-declared", f"{manifest.family_id}:layers", f"no layer declaration exists for {stage_id}", "Declare the layer and its evidence boundary before automation can evaluate it.")])
    for name, raw in selected_layers:
        if not isinstance(raw, Mapping):
            findings.append(_finding("error", "layer-malformed", f"{manifest.family_id}:layers.{name}", "layer declaration is not a mapping", "Use version/evidence fields and keep the layer status explicit."))
            continue
        value = str(raw.get("evidence", raw.get("status", "")))
        evidence_path = _evidence_path(value)
        if evidence_path is not None:
            evidence.append(_relative(evidence_path))
            if not evidence_path.is_file():
                findings.append(_finding("error", "layer-evidence-missing", _relative(evidence_path), f"evidence for layer {name!r} does not exist", "Generate or correct the evidence artifact; do not classify a missing file as passed."))
        declared_status = raw.get("status", raw.get("evidence", ""))
        stage_status = _classify_declared_status(declared_status)
        statuses.append(stage_status)
        if stage_status == "blocked":
            findings.append(_finding("error", "layer-not-promoted", f"{manifest.family_id}:layers.{name}", f"layer {name!r} is declared {declared_status!r}", "Complete the layer-specific evidence or keep the downstream profile blocked."))
        elif stage_status in {"development", "planned"}:
            findings.append(_finding("warning", "layer-development-only", f"{manifest.family_id}:layers.{name}", f"layer {name!r} remains {declared_status!r}", "Keep the promotion report at development evidence until the declared gate passes."))
    if any(item.severity == "error" for item in findings):
        status: StageStatus = "blocked"
    elif any(item == "development" for item in statuses):
        status = "development"
    elif any(item == "planned" for item in statuses):
        status = "planned"
    elif statuses and all(item == "not_applicable" for item in statuses):
        status = "not_applicable"
    else:
        status = "passed"
    return _stage(stage_id, status, claim, evidence, findings, {"layers_checked": list(layer_names), "declared_statuses": statuses})
    ####


def _fidelity_stage(readiness: VehicleIntegrationReadinessReport) -> IntegrationStageReport:
    findings: list[IntegrationPipelineFinding] = []
    evidence: list[str] = [readiness.manifest]
    for profile in readiness.profiles:
        for item in profile.findings:
            severity: Literal["error", "warning", "info"] = "error" if item.severity == "error" else "warning"
            findings.append(_finding(severity, item.code, item.path, item.message, item.hint))
    status: StageStatus = "blocked" if any(item.severity == "error" for item in findings) else "development"
    if not findings and any(profile.automatically_lowerable for profile in readiness.profiles):
        status = "passed"
    return _stage("fidelity", status, "each declared fidelity is explicit, and automatic lowering is permitted only by qualified evidence", evidence, findings, {"profile_count": len(readiness.profiles), "automatic_lowering_eligible_profiles": [profile.profile_id for profile in readiness.profiles if profile.automatically_lowerable]})
    ####


def _effectivity_stage(family_id: str) -> IntegrationStageReport:
    """Map numeric effectivity and overlay diagnostics into the pipeline."""

    report = validate_vehicle_effectivity_preflight(family_id)
    findings = [
        _finding(
            "error" if finding.severity == "error" else "warning" if finding.severity == "warning" else "info",
            finding.code,
            finding.path,
            finding.message,
            finding.hint,
        )
        for finding in report.findings
    ]
    return _stage(
        "effectivity",
        cast(StageStatus, report.status),
        "numeric local effectivity or an explicitly bounded downstream allocation overlay is declared and diagnosed",
        list(report.evidence),
        findings,
        report.metrics,
    )
    ####


def _trim_orchestration_stages(family_id: str) -> tuple[IntegrationStageReport, IntegrationStageReport]:
    """Map the declarative recipe/worklist report into pipeline stages."""

    recipe_path = ROOT / "families" / family_id / "qualification/trim-recipe.yaml"
    report = orchestrate_trim_recipe(family_id)
    solve_report = solve_vehicle_trim_evidence(family_id) if report.status == "ready_for_adapter" else None
    recipe_findings = tuple(
        _finding(
            "error" if finding.severity == "error" else "warning",
            finding.code,
            finding.path,
            finding.message,
            finding.hint,
        )
        for finding in report.findings
    )
    evidence = [_relative(recipe_path)]
    if report.catalog_path:
        evidence.append(report.catalog_path)
    trim_status: StageStatus = "blocked" if report.status == "blocked" else "development"
    trim_findings = list(recipe_findings)
    if solve_report is not None and solve_report.status == "verified":
        trim_status = "passed"
        trim_findings.append(
            _finding(
                "info",
                "trim-adapter-solve-verified",
                f"{family_id}:trim",
                f"family adapter solved {len(solve_report.points)} operating point(s) against the pinned evaluator",
                "Retain the generated solve evidence and do not extend the claim beyond the declared recipe boundary.",
            )
        )
    elif report.status == "ready_for_adapter":
        trim_findings.append(
            _finding(
                "warning",
                "trim-adapter-not-invoked",
                _relative(recipe_path),
                "generic orchestration produced work items but did not evaluate the family plant",
                "Bind the worklist to the family adapter and retain solve residuals before promoting trim evidence.",
            )
        )
    trim_stage = _stage(
        "trim",
        trim_status,
        "declarative trim unknowns, residuals, bounds, and solver policy are validated and bound to the declared family evaluator",
        evidence,
        trim_findings,
        {"recipe_status": report.status, "work_item_count": len(report.work_items), "adapter_solve_status": solve_report.status if solve_report else None},
    )
    operating_findings = [item for item in recipe_findings if item.code.startswith("operating-point-")]
    if report.status == "blocked":
        operating_findings.append(
            _finding(
                "error",
                "trim-worklist-blocked",
                _relative(recipe_path),
                "no adapter-ready operating-point worklist was produced",
                "Repair the trim recipe or operating-point catalog before family evaluation.",
            )
        )
    elif solve_report is not None and solve_report.status == "verified":
        operating_findings.append(
            _finding(
                "info",
                "operating-point-solve-verified",
                f"{family_id}:operating_points",
                f"family adapter solved {len(solve_report.points)} catalog point(s)",
                "Persist the solve report in the evidence packet before promotion.",
            )
        )
    elif not operating_findings:
        operating_findings.append(
            _finding(
                "warning",
                "operating-point-adapter-pending",
                report.catalog_path or _relative(recipe_path),
                "operating-point work items are ready for a family adapter but have not been solved here",
                "Run the adapter evaluator and write residual/validity evidence for each point.",
            )
        )
    operating_stage = _stage(
        "operating_points",
        "blocked" if report.status == "blocked" else "passed" if solve_report is not None and solve_report.status == "verified" else "development",
        "operating-point catalog is structurally validated and ordered into adapter-ready trim work items",
        evidence,
        operating_findings,
        {"work_item_count": len(report.work_items), "recipe_id": report.recipe_id, "adapter_solve_status": solve_report.status if solve_report else None},
    )
    return trim_stage, operating_stage
    ####


def _controller_mission_preflight_stages(family_id: str) -> tuple[IntegrationStageReport, IntegrationStageReport]:
    """Map controller/mission preflight into the common pipeline stages."""

    report = validate_vehicle_controller_mission_preflight(family_id)

    def convert(findings: tuple[object, ...]) -> list[IntegrationPipelineFinding]:
        converted: list[IntegrationPipelineFinding] = []
        for finding in findings:
            severity = getattr(finding, "severity")
            converted.append(
                _finding(
                    "error" if severity == "error" else "warning" if severity == "warning" else "info",
                    str(getattr(finding, "code")),
                    str(getattr(finding, "path")),
                    str(getattr(finding, "message")),
                    str(getattr(finding, "hint")),
                )
            )
        return converted

    def stage_status(status: str) -> StageStatus:
        return cast(StageStatus, status) if status in {"passed", "development", "planned", "blocked", "not_applicable"} else "development"

    controller_stage = _stage(
        "controller",
        stage_status(report.controller.status),
        "declared controller profiles, plant/effectiveness references, and physical control-path claims pass structural preflight",
        [_relative(ROOT / "families" / family_id / "controllers")],
        convert(report.controller.findings),
        report.controller.metrics,
    )
    mission_stage = _stage(
        "mission",
        stage_status(report.mission.status),
        "mission binding, realization IDs, route geometry, phase order, and capability-based time estimates pass preflight",
        list(report.mission.evidence),
        convert(report.mission.findings),
        report.mission.metrics,
    )
    return controller_stage, mission_stage
    ####


def _contract_for_family(family_id: str) -> tuple[Path, Mapping[str, Any] | None]:
    path = ROOT / "verification/daveml_operational_contracts.yaml"
    if not path.is_file():
        return path, None
    payload = _read_yaml(path)
    families = payload.get("families")
    if not isinstance(families, list):
        return path, None
    return path, next((entry for entry in families if isinstance(entry, Mapping) and str(entry.get("id")) == family_id), None)
    ####


def _load_records(manifest_path: Path) -> tuple[Path, Mapping[str, Any], Path, DAVEMLFamilyImport]:
    record_path, record = _load_integration_record(manifest_path)
    import_path = manifest_path.parent / "plant/daveml-import.json"
    return record_path, record, import_path, load_daveml_family_import(import_path)
    ####


def _build_report(
    readiness: VehicleIntegrationReadinessReport,
    manifest_path: Path,
    manifest: ReferenceFamilyManifest,
    record_path: Path,
    record: Mapping[str, Any],
    import_path: Path,
    import_record: DAVEMLFamilyImport,
    contract_path: Path,
    contract: Mapping[str, Any] | None,
) -> VehicleIntegrationPipelineReport:
    trim_stage, operating_points_stage = _trim_orchestration_stages(manifest.family_id)
    controller_stage, mission_stage = _controller_mission_preflight_stages(manifest.family_id)
    stages = (
        _intake_stage(readiness, manifest_path, import_path, record_path),
        _convention_stage(manifest, manifest_path, import_record, import_path, contract, contract_path),
        _plant_stage(manifest, import_record, import_path),
        _effectivity_stage(manifest.family_id),
        trim_stage,
        operating_points_stage,
        _fidelity_stage(readiness),
        controller_stage,
        mission_stage,
    )
    blockers = tuple(item for stage in stages for item in stage.blockers)
    caveats = tuple(item for stage in stages for item in stage.findings if item.severity != "error")
    if any(stage.status == "blocked" for stage in stages):
        status = "blocked"
    elif any(stage.status in {"development", "planned"} for stage in stages):
        status = "development_ready_with_gates"
    else:
        status = "promotion_ready"
    completed = {stage.stage_id for stage in stages if stage.status == "passed"}
    if "mission" in completed and "controller" in completed:
        current_tier = "T6_family_integration_ready"
    elif "plant" in completed:
        current_tier = "T2_source_plant_ready"
    elif "conventions" in completed:
        current_tier = "T1_conventions_ready"
    elif "intake" in completed:
        current_tier = "T0_metadata_ready"
    else:
        current_tier = "T0_metadata_blocked"
    next_gate = manifest.next_gate
    return VehicleIntegrationPipelineReport(
        family_id=manifest.family_id,
        display_name=manifest.display_name,
        physical_family=readiness.physical_family,
        status=status,
        current_tier=current_tier,
        next_gate=next_gate,
        promotion_eligible=not blockers and all(stage.status == "passed" for stage in stages),
        claim_boundary="provider-neutral integration evidence; not flight, operational, or family qualification",
        stages=stages,
        blockers=blockers,
        caveats=caveats,
    )
    ####


def validate_vehicle_integration_pipeline(family_id: str) -> VehicleIntegrationPipelineReport:
    """Run all non-inventive integration stages for one reference family."""

    readiness = validate_vehicle_integration_readiness(family_id)
    manifest_path = ROOT / readiness.manifest
    try:
        manifest = load_reference_family_manifest(manifest_path)
        record_path, record, import_path, import_record = _load_records(manifest_path)
        contract_path, contract = _contract_for_family(family_id)
    except (OSError, ValueError, KeyError, TypeError) as error:
        finding = _finding("error", "integration-record-load-failed", _relative(manifest_path), str(error), "Repair the family manifest/import record before running downstream integration stages.")
        stage = _stage("intake", "blocked", "family integration records are loadable", [_relative(manifest_path)], [finding])
        return VehicleIntegrationPipelineReport(family_id, readiness.display_name, readiness.physical_family, "blocked", "T0_metadata", "repair_intake_records", False, "provider-neutral integration evidence; not flight, operational, or family qualification", (stage,), (finding,), ())
    return _build_report(readiness, manifest_path, manifest, record_path, record, import_path, import_record, contract_path, contract)
    ####


def validate_all_vehicle_integration_pipelines() -> tuple[VehicleIntegrationPipelineReport, ...]:
    """Run the staged integration report for every supported reference family."""

    return tuple(validate_vehicle_integration_pipeline(report.family_id) for report in validate_all_vehicle_integration_readiness())
    ####


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def _packet_candidates(manifest_path: Path, manifest: ReferenceFamilyManifest, record: Mapping[str, Any], import_record: DAVEMLFamilyImport, contract_path: Path) -> tuple[Path, ...]:
    mission_binding_path = manifest_path.parent / "qualification/mission-binding.yaml"
    if not mission_binding_path.is_file():
        mission_binding_path = manifest_path.parent / "qualification/racetrack-binding.yaml"
    candidates: list[Path] = [
        manifest_path,
        manifest_path.parent / "plant/daveml-import.json",
        manifest_path.parent / "qualification/integration-record.yaml",
        manifest_path.parent / "qualification/trim-recipe.yaml",
        manifest_path.parent / "qualification/operating-points.yaml",
        mission_binding_path,
        contract_path,
    ]
    controller_paths = tuple(manifest_path.parent.glob("controllers/*.yaml"))
    candidates.extend(controller_paths)
    for controller_path in controller_paths:
        try:
            controller = _read_yaml(controller_path)
        except (OSError, ValueError, yaml.YAMLError):
            controller = {}
        if isinstance(controller, Mapping):
            for key in ("linearization_artifact", "effectiveness_artifact", "evidence"):
                artifact_path = _evidence_path(controller.get(key))
                if artifact_path is not None:
                    candidates.append(artifact_path)
    for binding in manifest.bindings:
        candidates.append(manifest_path.parent / binding)
    for layer in manifest.layers.values():
        evidence_path = _evidence_path(layer.evidence)
        if evidence_path is not None:
            candidates.append(evidence_path)
    raw_layers = record.get("layers")
    if isinstance(raw_layers, Mapping):
        for layer in raw_layers.values():
            if isinstance(layer, Mapping):
                evidence_path = _evidence_path(layer.get("evidence"))
                if evidence_path is not None:
                    candidates.append(evidence_path)
    trim_recipe_path = manifest_path.parent / "qualification/trim-recipe.yaml"
    if trim_recipe_path.is_file():
        try:
            recipe = _read_yaml(trim_recipe_path)
        except (OSError, ValueError, yaml.YAMLError):
            recipe = {}
        source_evidence = _evidence_path(recipe.get("source_evidence")) if isinstance(recipe, Mapping) else None
        if source_evidence is not None:
            candidates.append(source_evidence)
    if mission_binding_path.is_file():
        try:
            mission_binding = _read_yaml(mission_binding_path)
        except (OSError, ValueError, yaml.YAMLError):
            mission_binding = {}
        if isinstance(mission_binding, Mapping):
            source_catalog = _evidence_path(mission_binding.get("source_catalog"))
            if source_catalog is not None:
                candidates.append(source_catalog)
            realizations = mission_binding.get("realizations")
            if isinstance(realizations, list):
                for realization in realizations:
                    if isinstance(realization, Mapping):
                        composition_path = _evidence_path(realization.get("composition"))
                        if composition_path is not None:
                            candidates.append(composition_path)
    source_package = ROOT / import_record.package.path
    if source_package.is_file():
        candidates.append(source_package)
    effectivity_report = validate_vehicle_effectivity_preflight(manifest.family_id)
    candidates.extend(ROOT / evidence for evidence in effectivity_report.evidence)
    return tuple(dict.fromkeys(path for path in candidates if path.is_file()))
    ####


def write_vehicle_integration_packet(
    family_id: str,
    output: str | Path,
    *,
    include_source_package: bool = True,
) -> VehicleIntegrationPipelineReport:
    """Write a reproducible integration evidence packet for one family.

    The packet copies canonical metadata, bindings, referenced evidence, and
    the pinned DAVE-ML package when available.  The packet manifest records
    every included hash and every referenced-but-unavailable artifact; it does
    not claim that an external common corpus was copied merely because its
    hash is recorded.
    """

    report = validate_vehicle_integration_pipeline(family_id)
    manifest_path = ROOT / report.stages[0].evidence[0]
    manifest = load_reference_family_manifest(manifest_path)
    record_path, record, import_path, import_record = _load_records(manifest_path)
    contract_path, _ = _contract_for_family(family_id)
    packet = Path(output)
    packet.mkdir(parents=True, exist_ok=True)
    files: list[dict[str, str]] = []
    candidates = _packet_candidates(manifest_path, manifest, record, import_record, contract_path)
    if not include_source_package:
        source_package = ROOT / import_record.package.path
        candidates = tuple(path for path in candidates if path != source_package)
    for source in candidates:
        relative = _relative(source)
        target = packet / "inputs" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        files.append({"source": relative, "packet_path": str(target.relative_to(packet)), "sha256": _sha256(source)})
    (packet / "integration_report.json").write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    effectivity = validate_vehicle_effectivity_preflight(family_id)
    (packet / "effectivity_preflight.json").write_text(json.dumps(effectivity.as_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    preflight = validate_vehicle_controller_mission_preflight(family_id)
    (packet / "controller_mission_preflight.json").write_text(json.dumps(preflight.as_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    trim_solve = solve_vehicle_trim_evidence(family_id)
    (packet / "vehicle_trim_solve_evidence.json").write_text(json.dumps(trim_solve.as_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    packet_manifest = {
        "schema_version": "taoryx.vehicle-integration-packet/v1",
        "family_id": family_id,
        "claim_boundary": report.claim_boundary,
        "source_package_included": include_source_package and any(item["source"] == import_record.package.path for item in files),
        "files": files,
        "referenced_external_hashes": {
            "source_package_sha256": import_record.package.sha256,
        },
        "report": "integration_report.json",
        "effectivity_preflight": "effectivity_preflight.json",
        "controller_mission_preflight": "controller_mission_preflight.json",
        "vehicle_trim_solve_evidence": "vehicle_trim_solve_evidence.json",
    }
    (packet / "manifest.json").write_text(json.dumps(packet_manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report
    ####


__all__ = [
    "IntegrationPipelineFinding",
    "IntegrationStageReport",
    "VehicleIntegrationPipelineReport",
    "validate_all_vehicle_integration_pipelines",
    "validate_vehicle_integration_pipeline",
    "write_vehicle_integration_packet",
]
####
