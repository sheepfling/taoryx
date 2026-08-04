"""Provider-neutral controller and mission preflight diagnostics.

This module checks whether a family has declared enough controller and mission
metadata to begin execution.  It does not solve gains, run a vehicle, or infer
missing effectors.  A direct-wrench profile is reported as development/screen
evidence; a missing controller or mission binding remains explicit.
"""

from __future__ import annotations

import json
import re
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import yaml

from .generic_tuning import LinearAuthorityRequirement, linear_authority_preflight
from .racetrack_template import resolve_racetrack_binding
from .vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request
from .vehicle_execution_preflight import preflight_vehicle_composition
from .vehicle_registry import ROOT

PreflightStatus = Literal["passed", "development", "planned", "blocked", "not_applicable"]


@dataclass(frozen=True, slots=True)
class PreflightFinding:
    """One controller or mission preflight finding."""

    severity: Literal["error", "warning", "info"]
    code: str
    path: str
    message: str
    hint: str

    def as_dict(self) -> dict[str, str]:
        """Return a stable machine-readable finding."""

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
class ControllerPreflightResult:
    """Structural preflight of a family's declared controller profiles."""

    family_id: str
    status: PreflightStatus
    profiles_checked: tuple[str, ...]
    findings: tuple[PreflightFinding, ...]
    metrics: Mapping[str, object]

    @property
    def blockers(self) -> tuple[PreflightFinding, ...]:
        """Return controller blockers."""

        return tuple(item for item in self.findings if item.severity == "error")
        ####

    def as_dict(self) -> dict[str, object]:
        """Return a stable JSON representation."""

        return {
            "family_id": self.family_id,
            "status": self.status,
            "profiles_checked": list(self.profiles_checked),
            "metrics": dict(self.metrics),
            "error_count": len(self.blockers),
            "findings": [item.as_dict() for item in self.findings],
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class MissionPreflightResult:
    """Structural and capability-based preflight of a family mission binding."""

    family_id: str
    status: PreflightStatus
    mission_id: str | None
    evidence: tuple[str, ...]
    findings: tuple[PreflightFinding, ...]
    metrics: Mapping[str, object]

    @property
    def blockers(self) -> tuple[PreflightFinding, ...]:
        """Return mission blockers."""

        return tuple(item for item in self.findings if item.severity == "error")
        ####

    def as_dict(self) -> dict[str, object]:
        """Return a stable JSON representation."""

        return {
            "family_id": self.family_id,
            "status": self.status,
            "mission_id": self.mission_id,
            "evidence": list(self.evidence),
            "metrics": dict(self.metrics),
            "error_count": len(self.blockers),
            "findings": [item.as_dict() for item in self.findings],
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class VehicleControllerMissionPreflightReport:
    """Combined controller/mission preflight for one family."""

    family_id: str
    status: PreflightStatus
    controller: ControllerPreflightResult
    mission: MissionPreflightResult

    @property
    def blockers(self) -> tuple[PreflightFinding, ...]:
        """Return all preflight blockers."""

        return self.controller.blockers + self.mission.blockers
        ####

    @property
    def caveats(self) -> tuple[PreflightFinding, ...]:
        """Return warnings and informational findings."""

        return tuple(
            item
            for result in (self.controller, self.mission)
            for item in result.findings
            if item.severity != "error"
        )
        ####

    def as_dict(self) -> dict[str, object]:
        """Return a stable machine-readable report."""

        return {
            "schema_version": "taoryx.vehicle-controller-mission-preflight/v1",
            "family_id": self.family_id,
            "status": self.status,
            "controller": self.controller.as_dict(),
            "mission": self.mission.as_dict(),
            "blockers": [item.as_dict() for item in self.blockers],
            "caveats": [item.as_dict() for item in self.caveats],
        }
        ####
    ####


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)
    ####


def _read_yaml(path: Path) -> Mapping[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{path} must contain a mapping")
    return payload
    ####


def _read_json(path: Path) -> Mapping[str, Any]:
    """Load one declared controller evidence artifact as a mapping."""

    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{path} must contain a JSON object")
    return payload
    ####


def _finding(
    findings: list[PreflightFinding],
    severity: Literal["error", "warning", "info"],
    code: str,
    path: str,
    message: str,
    hint: str,
) -> None:
    findings.append(PreflightFinding(severity, code, path, message, hint))
    ####


def _path_from_value(value: object) -> Path | None:
    label = str(value).split("#", 1)[0].strip()
    if not label or label in {"none", "not_applicable", "not_available"}:
        return None
    if not any(token in label for token in ("/", ".json", ".yaml", ".yml", ".csv", ".tbl", ".prb")):
        return None
    path = Path(label)
    return path if path.is_absolute() else ROOT / path
    ####


def _normalize_channel(value: object) -> str:
    return re.sub(r"[^a-z0-9]", "", str(value).lower()).replace("control", "").replace("fraction", "").replace("deg", "")
    ####


def _status_from_findings(findings: list[PreflightFinding], *, empty: PreflightStatus = "planned") -> PreflightStatus:
    if any(item.severity == "error" for item in findings):
        return "blocked"
    if any(item.severity == "warning" for item in findings):
        return "development"
    return empty
    ####


def _load_control_names(family_dir: Path) -> set[str]:
    path = family_dir / "bindings/canonical-controls.yaml"
    if not path.is_file():
        return set()
    try:
        payload = _read_yaml(path)
    except (OSError, ValueError, yaml.YAMLError):
        return set()
    controls = payload.get("controls")
    if not isinstance(controls, list):
        return set()
    names: set[str] = set()
    for item in controls:
        if isinstance(item, Mapping):
            names.update(_normalize_channel(value) for value in (item.get("id"), item.get("source_name")) if value)
            if _normalize_channel(item.get("id", "")) == "throttle":
                names.add("throttle")
    return names
    ####


def _authority_preflight_for_controller(
    path: Path,
    payload: Mapping[str, Any],
    linearization_path: Path | None,
    findings: list[PreflightFinding],
) -> Mapping[str, object] | None:
    """Check a declared LQR objective against its plant-derived A/B artifact.

    This is the onboarding use of the generic authority gate.  The controller
    profile declares *which states must be controlled*; the artifact declares
    the actual local state derivative.  Neither side gets to silently infer
    the other, and an unavailable artifact remains a prior preflight error.
    """

    requirement_payload = payload.get("authority_requirement")
    if not isinstance(requirement_payload, Mapping):
        _finding(
            findings,
            "error",
            "controller-authority-requirement-missing",
            _relative(path),
            "controller profile does not declare the state authority required before tuning",
            "Declare authority_requirement.id and authority_requirement.required_state_names so rank loss is caught before gain search.",
        )
        return None
    names = requirement_payload.get("required_state_names")
    if not isinstance(names, list) or not names or not all(isinstance(name, str) and name for name in names):
        _finding(
            findings,
            "error",
            "controller-authority-requirement-invalid",
            f"{_relative(path)}:authority_requirement.required_state_names",
            "authority requirement must contain one or more non-empty state names",
            "Use the exact state ordering/names emitted by the plant-derived linearization artifact.",
        )
        return None
    if linearization_path is None or not linearization_path.is_file():
        return None
    try:
        artifact = _read_json(linearization_path)
        state_names = artifact.get("state_names")
        a_matrix = artifact.get("a_matrix")
        b_matrix = artifact.get("b_matrix")
        if not isinstance(state_names, list) or not isinstance(a_matrix, list) or not isinstance(b_matrix, list):
            raise ValueError("linearization artifact lacks state_names, a_matrix, or b_matrix")
        requirement = LinearAuthorityRequirement(
            str(requirement_payload.get("id", "")),
            tuple(names),
            minimum_controllability_rank=(
                int(requirement_payload["minimum_controllability_rank"])
                if requirement_payload.get("minimum_controllability_rank") is not None
                else None
            ),
            maximum_uncontrolled_fraction=float(requirement_payload.get("maximum_uncontrolled_fraction", 1.0e-8)),
            maximum_controllability_condition=(
                float(requirement_payload["maximum_controllability_condition"])
                if requirement_payload.get("maximum_controllability_condition") is not None
                else None
            ),
        )
        report = linear_authority_preflight(
            requirement,
            state_names=tuple(str(name) for name in state_names),
            a_matrix=a_matrix,
            b_matrix=b_matrix,
        )
    except (KeyError, TypeError, ValueError, OSError, json.JSONDecodeError) as error:
        _finding(
            findings,
            "error",
            "controller-authority-preflight-invalid",
            _relative(linearization_path),
            f"authority preflight could not consume the declared linearization: {error}",
            "Regenerate the A/B artifact and align the controller authority requirement with its exact state schema.",
        )
        return None
    if report.status != "passed":
        _finding(
            findings,
            "error",
            "controller-authority-blocked",
            _relative(linearization_path),
            report.reason,
            "Add/control the missing physical axis, select a supported reduced objective, or use a different scheduled operating point before tuning.",
        )
    return report.as_dict()
    ####


def _controller_preflight(family_id: str) -> ControllerPreflightResult:
    family_dir = ROOT / "families" / family_id
    controller_dir = family_dir / "controllers"
    findings: list[PreflightFinding] = []
    profile_ids: list[str] = []
    authority_reports: dict[str, Mapping[str, object]] = {}
    if not controller_dir.is_dir():
        _finding(findings, "info", "controller-directory-absent", _relative(controller_dir), "no controller profiles are declared", "Use not_applicable only when the family contract explicitly declares no controller.")
        return ControllerPreflightResult(family_id, "not_applicable", (), tuple(findings), {"profiles_checked": 0})
    control_names = _load_control_names(family_dir)
    for path in sorted(controller_dir.glob("*.yaml")):
        try:
            payload = _read_yaml(path)
        except (OSError, ValueError, yaml.YAMLError) as error:
            _finding(findings, "error", "controller-profile-load-failed", _relative(path), str(error), "Repair the controller profile YAML before execution.")
            continue
        profile_id = str(payload.get("controller_id", path.stem))
        profile_ids.append(profile_id)
        for required in ("controller_id", "method", "fidelity", "state_names"):
            if required not in payload:
                _finding(findings, "error", "controller-field-missing", f"{_relative(path)}:{required}", f"controller profile lacks {required!r}", "Declare the controller identity and ordered state contract.")
        implementation = str(payload.get("method", ""))
        linearization_path: Path | None = None
        if implementation in {"continuous_lqr", "lqr", "lqi", "gain_scheduled_lqr"} and not payload.get("linearization_artifact"):
            _finding(findings, "error", "controller-linearization-missing", _relative(path), "LQR profile does not name a linearization artifact", "Bind the controller to a plant-derived A/B artifact.")
        if implementation in {"continuous_lqr", "lqr", "lqi", "gain_scheduled_lqr"}:
            artifact_value = payload.get("linearization_artifact")
            linearization_path = _path_from_value(artifact_value)
            if linearization_path is not None and not linearization_path.is_file():
                _finding(findings, "error", "controller-linearization-missing", _relative(linearization_path), "controller linearization artifact does not exist", "Generate the referenced linearization evidence before controller preflight.")
            authority = _authority_preflight_for_controller(path, payload, linearization_path, findings)
            if authority is not None:
                authority_reports[profile_id] = authority
        effectiveness_path = _path_from_value(payload.get("effectiveness_artifact"))
        if effectiveness_path is not None and not effectiveness_path.is_file():
            _finding(findings, "error", "controller-effectiveness-missing", _relative(effectiveness_path), "controller effectiveness artifact does not exist", "Generate or correct the effectivity evidence.")
        effectors = payload.get("effector_names", payload.get("control_names", ()))
        if not payload.get("effector_names"):
            _finding(findings, "warning", "controller-effectors-unresolved", _relative(path), "controller profile does not declare a separate physical-effector realization", "Keep the profile at screen/development evidence until the allocator and actuator path are explicit.")
        if isinstance(effectors, list) and control_names:
            missing = [item for item in effectors if _normalize_channel(item) not in control_names]
            if missing:
                _finding(findings, "error", "controller-effector-unbound", _relative(path), f"controller effectors are not in the canonical control binding: {missing}", "Add explicit control binding or correct the profile channel names.")
        if "direct_wrench" in str(payload.get("fidelity", "")).lower() or "wrench" in str(payload.get("control_space", "")).lower() and not payload.get("effector_names"):
            _finding(findings, "warning", "direct-wrench-screen", _relative(path), "controller profile uses a direct-wrench or non-effector control path", "Keep this result at development/screen evidence until physical allocation reaches the nonlinear plant.")
        if not payload.get("evidence") and not payload.get("linearization_artifact"):
            _finding(findings, "warning", "controller-evidence-missing", _relative(path), "controller profile names no evidence artifact or source", "Attach the design, trim, and nonlinear validation evidence.")
    status = _status_from_findings(findings, empty="passed" if profile_ids else "not_applicable")
    return ControllerPreflightResult(
        family_id,
        status,
        tuple(profile_ids),
        tuple(findings),
        {
            "profiles_checked": len(profile_ids),
            "canonical_control_count": len(control_names),
            "authority_preflights": authority_reports,
        },
    )
    ####


def _estimate_racetrack(template_id: str, binding_id: str, binding: Mapping[str, Any]) -> float | None:
    """Resolve timing through the canonical mission compiler.

    The vertical phases occur on the straight legs, so adding their durations
    to a closed-course estimate double-counts route time.  Reuse the canonical
    resolver rather than maintaining a second approximation here.
    """

    try:
        return resolve_racetrack_binding(template_id, binding_id, dict(binding)).horizon_s
    except (KeyError, TypeError, ValueError):
        return None
    ####


def _semantic_composition_mission_preflight(
    family_id: str,
    binding_path: Path,
    binding: Mapping[str, Any],
) -> MissionPreflightResult:
    """Validate a reusable mission through checked-in semantic witnesses.

    A family mission need not be a fixed-wing racetrack.  This binding form
    verifies that each retained composition witness compiles to the declared
    family, mission, fidelity, phase order, and semantic-preflight status. It
    deliberately stops before runtime execution: a translated plan is not a
    plant, controller, truth-objective result, or qualification claim.
    """

    findings: list[PreflightFinding] = []
    evidence = [_relative(binding_path)]
    mission_id = str(binding.get("mission_id", "")) or None
    expected_family = str(binding.get("family", ""))
    composition_family_id = str(binding.get("composition_family_id", ""))
    if expected_family != family_id:
        _finding(
            findings,
            "error",
            "mission-binding-family-mismatch",
            _relative(binding_path),
            f"semantic mission binding belongs to {expected_family!r}, not {family_id!r}",
            "Bind each reusable mission record to exactly one source family.",
        )
    if not composition_family_id:
        _finding(
            findings,
            "error",
            "mission-binding-composition-family-missing",
            _relative(binding_path),
            "semantic mission binding does not declare its public composition family ID",
            "Declare composition_family_id explicitly; source-family and public composition identities may differ.",
        )
    if mission_id is None:
        _finding(
            findings,
            "error",
            "mission-binding-id-missing",
            _relative(binding_path),
            "semantic mission binding has no mission_id",
            "Declare the exact public mission template ID.",
        )
    phase_order = binding.get("phase_order")
    if not isinstance(phase_order, list) or not phase_order or not all(isinstance(item, str) and item for item in phase_order):
        _finding(
            findings,
            "error",
            "mission-phase-order-missing",
            _relative(binding_path),
            "semantic mission binding has no ordered phase contract",
            "Declare the characteristic mission lifecycle in order.",
        )
        phase_order = []
    realizations = binding.get("realizations")
    if not isinstance(realizations, list) or not realizations:
        _finding(
            findings,
            "error",
            "mission-realizations-missing",
            _relative(binding_path),
            "semantic mission binding declares no checked-in composition witnesses",
            "Bind one or more exact composition requests and expected semantic-preflight dispositions.",
        )
        realizations = []

    preflight_status_by_fidelity: dict[str, str] = {}
    composition_ids: dict[str, str] = {}
    for index, realization in enumerate(realizations):
        path = f"{_relative(binding_path)}:realizations[{index}]"
        if not isinstance(realization, Mapping):
            _finding(findings, "error", "mission-realization-invalid", path, "mission realization must be a mapping", "Declare fidelity, composition, and expected_preflight_status.")
            continue
        fidelity = str(realization.get("fidelity", ""))
        expected_status = str(realization.get("expected_preflight_status", ""))
        composition_value = realization.get("composition")
        composition_path = _path_from_value(composition_value)
        if not fidelity or expected_status not in {"translation_ready", "blocked", "not_applicable"}:
            _finding(
                findings,
                "error",
                "mission-realization-contract-invalid",
                path,
                "mission realization must declare a fidelity and canonical expected_preflight_status",
                "Use one explicit fidelity and one of translation_ready, blocked, or not_applicable.",
            )
            continue
        if composition_path is None or not composition_path.is_file():
            _finding(
                findings,
                "error",
                "mission-composition-witness-missing",
                path,
                "mission realization does not resolve to a checked-in composition witness",
                "Add the composition request or correct its repository-relative path.",
            )
            continue
        evidence.append(_relative(composition_path))
        try:
            composition = compile_vehicle_composition(load_vehicle_composition_request(composition_path))
            preflight = preflight_vehicle_composition(composition)
        except (OSError, TypeError, ValueError) as error:
            _finding(
                findings,
                "error",
                "mission-composition-witness-invalid",
                _relative(composition_path),
                f"mission composition witness could not compile/preflight: {error}",
                "Repair the request or its declared family capability/translator binding.",
            )
            continue
        if composition.family_id != composition_family_id or composition.mission != mission_id or composition.fidelity != fidelity:
            _finding(
                findings,
                "error",
                "mission-composition-witness-mismatch",
                _relative(composition_path),
                "compiled witness identity does not match the binding family, mission, and fidelity",
                "Keep the witness and binding composition_family_id, mission_id, and fidelity aligned; do not reuse a nearby family request.",
            )
        actual_phase_order = [segment.id for segment in composition.segments]
        if actual_phase_order != phase_order:
            _finding(
                findings,
                "error",
                "mission-composition-phase-order-mismatch",
                _relative(composition_path),
                f"compiled witness phases {actual_phase_order!r} do not match binding phase order {phase_order!r}",
                "Update the binding only when the semantic mission contract intentionally changes.",
            )
        if preflight.status != expected_status:
            _finding(
                findings,
                "error",
                "mission-preflight-status-mismatch",
                _relative(composition_path),
                f"semantic preflight is {preflight.status!r}, expected {expected_status!r}",
                "Repair the capability/translator declaration or update an intentional expectation with review.",
            )
        preflight_status_by_fidelity[fidelity] = preflight.status
        composition_ids[fidelity] = composition.id

    declared_status = str(binding.get("status", ""))
    if "planned" in declared_status.lower() or "development" in declared_status.lower():
        _finding(
            findings,
            "warning",
            "mission-runtime-planned",
            _relative(binding_path),
            f"mission binding status is {declared_status!r}",
            "Keep translation evidence separate until a family-owned runtime, truth objectives, and terminal evaluator are bound.",
        )
    status = _status_from_findings(findings, empty="passed")
    return MissionPreflightResult(
        family_id,
        status,
        mission_id,
        tuple(dict.fromkeys(evidence)),
        tuple(findings),
        {
            "realization_count": len(preflight_status_by_fidelity),
            "preflight_status_by_fidelity": preflight_status_by_fidelity,
            "composition_ids_by_fidelity": composition_ids,
            "estimated_duration_s": None,
        },
    )
    ####


def _mission_preflight(family_id: str) -> MissionPreflightResult:
    family_dir = ROOT / "families" / family_id
    semantic_binding_path = family_dir / "qualification/mission-binding.yaml"
    binding_path = semantic_binding_path if semantic_binding_path.is_file() else family_dir / "qualification/racetrack-binding.yaml"
    findings: list[PreflightFinding] = []
    if not binding_path.is_file():
        _finding(findings, "error", "mission-binding-missing", _relative(binding_path), "no reusable mission binding is declared for this family", "Declare a family mission binding before running or estimating a flagship mission.")
        return MissionPreflightResult(family_id, "blocked", None, (), tuple(findings), {"estimated_duration_s": None})
    try:
        binding = _read_yaml(binding_path)
    except (OSError, ValueError, yaml.YAMLError) as error:
        _finding(findings, "error", "mission-binding-load-failed", _relative(binding_path), str(error), "Repair the mission binding YAML before preflight.")
        return MissionPreflightResult(family_id, "blocked", None, (_relative(binding_path),), tuple(findings), {"estimated_duration_s": None})
    if str(binding.get("kind", "")) == "semantic_composition":
        return _semantic_composition_mission_preflight(family_id, binding_path, binding)
    template_path = _path_from_value(binding.get("source_catalog"))
    evidence = [_relative(binding_path)]
    if template_path is None or not template_path.is_file():
        _finding(findings, "error", "mission-template-missing", _relative(template_path or (family_dir / "qualification")), "mission binding does not resolve to a source catalog", "Pin the reusable mission template before estimating route timing.")
        return MissionPreflightResult(family_id, "blocked", binding.get("mission_profile") and str(binding["mission_profile"]), tuple(evidence), tuple(findings), {"estimated_duration_s": None})
    evidence.append(_relative(template_path))
    try:
        catalog = _read_yaml(template_path)
    except (OSError, ValueError, yaml.YAMLError) as error:
        _finding(findings, "error", "mission-template-load-failed", _relative(template_path), str(error), "Repair the mission template before preflight.")
        return MissionPreflightResult(family_id, "blocked", str(binding.get("mission_profile", "")) or None, tuple(evidence), tuple(findings), {"estimated_duration_s": None})
    binding_ids = binding.get("binding_ids")
    catalog_bindings = catalog.get("bindings")
    template = catalog.get("template")
    template_id = str(template.get("id", "")) if isinstance(template, Mapping) else ""
    if not template_id:
        _finding(findings, "error", "mission-template-id-missing", _relative(template_path), "mission template has no semantic identifier", "Declare template.id before resolving route timing.")
    if not isinstance(binding_ids, list) or not binding_ids:
        _finding(findings, "error", "mission-binding-ids-missing", _relative(binding_path), "mission binding declares no realization IDs", "List the 3DOF, pseudo-6DOF, and/or 6DOF realizations explicitly.")
    if not isinstance(catalog_bindings, Mapping):
        _finding(findings, "error", "mission-catalog-bindings-missing", _relative(template_path), "mission catalog has no binding map", "Declare vehicle-specific mission realization values in the shared template.")
        catalog_bindings = {}
    estimates: dict[str, float] = {}
    for binding_id in binding_ids if isinstance(binding_ids, list) else ():
        candidate = catalog_bindings.get(binding_id)
        if not isinstance(candidate, Mapping):
            _finding(findings, "error", "mission-realization-missing", f"{_relative(template_path)}:bindings.{binding_id}", f"mission realization {binding_id!r} is not in the shared catalog", "Add the realization or remove it from the family binding.")
            continue
        if str(candidate.get("vehicle_id")) != family_id:
            _finding(findings, "error", "mission-realization-family-mismatch", f"{_relative(template_path)}:bindings.{binding_id}", "mission realization belongs to a different vehicle family", "Use only bindings whose vehicle_id matches the selected family.")
        estimate = _estimate_racetrack(template_id, str(binding_id), candidate) if template_id else None
        if estimate is None:
            _finding(findings, "error", "mission-time-estimate-unavailable", f"{_relative(template_path)}:bindings.{binding_id}", "route timing cannot be estimated from the declared geometry and rates", "Declare positive speed, turn radius, leg length, and climb/descent rates.")
        else:
            estimates[str(binding_id)] = estimate
        realization = str(candidate.get("source_realization", ""))
        if "direct" in realization.lower() or "wrench" in realization.lower() or "moment_injection" in realization.lower():
            _finding(findings, "warning", "mission-direct-wrench-realization", f"{_relative(template_path)}:bindings.{binding_id}", "mission realization declares direct-wrench or moment-injection behavior", "Keep this mission as an integration baseline until physical effector realization is selected.")
        if "pending" in str(candidate.get("status", "")).lower():
            _finding(findings, "warning", "mission-realization-pending", f"{_relative(template_path)}:bindings.{binding_id}", f"mission realization is declared {candidate.get('status')!r}", "Do not promote the mission until its truth objectives and controls pass.")
    phase_order = binding.get("phase_order")
    if not isinstance(phase_order, list) or not phase_order:
        _finding(findings, "error", "mission-phase-order-missing", _relative(binding_path), "mission binding has no ordered phase contract", "Declare the characteristic mission lifecycle in order.")
    mission_id = str(binding.get("mission_profile", "")) or None
    status = _status_from_findings(findings, empty="passed")
    metrics: dict[str, object] = {"realization_count": len(estimates), "estimated_duration_s_by_realization": estimates}
    if estimates:
        metrics["estimated_duration_s_min"] = min(estimates.values())
        metrics["estimated_duration_s_max"] = max(estimates.values())
    return MissionPreflightResult(family_id, status, mission_id, tuple(evidence), tuple(findings), metrics)
    ####


def validate_vehicle_controller_mission_preflight(family_id: str) -> VehicleControllerMissionPreflightReport:
    """Run controller and mission preflight without executing a vehicle."""

    controller = _controller_preflight(family_id)
    mission = _mission_preflight(family_id)
    if controller.status == "blocked" or mission.status == "blocked":
        status: PreflightStatus = "blocked"
    elif controller.status == "not_applicable" and mission.status == "not_applicable":
        status = "not_applicable"
    elif controller.status in {"development", "planned"} or mission.status in {"development", "planned"}:
        status = "development"
    else:
        status = "passed"
    return VehicleControllerMissionPreflightReport(family_id, status, controller, mission)
    ####


def validate_all_vehicle_controller_mission_preflight() -> tuple[VehicleControllerMissionPreflightReport, ...]:
    """Run preflight for the two registered source-family pilots."""

    return tuple(validate_vehicle_controller_mission_preflight(family_id) for family_id in ("reference_f16_s119", "reference_hl20_mod_k"))
    ####


__all__ = [
    "ControllerPreflightResult",
    "MissionPreflightResult",
    "PreflightFinding",
    "VehicleControllerMissionPreflightReport",
    "validate_all_vehicle_controller_mission_preflight",
    "validate_vehicle_controller_mission_preflight",
]
####
