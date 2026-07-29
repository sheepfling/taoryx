"""Provider-neutral controller and mission preflight diagnostics.

This module checks whether a family has declared enough controller and mission
metadata to begin execution.  It does not solve gains, run a vehicle, or infer
missing effectors.  A direct-wrench profile is reported as development/screen
evidence; a missing controller or mission binding remains explicit.
"""

from __future__ import annotations

import math
import re
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import yaml

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


def _controller_preflight(family_id: str) -> ControllerPreflightResult:
    family_dir = ROOT / "families" / family_id
    controller_dir = family_dir / "controllers"
    findings: list[PreflightFinding] = []
    profile_ids: list[str] = []
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
        if implementation in {"continuous_lqr", "lqr", "lqi", "gain_scheduled_lqr"} and not payload.get("linearization_artifact"):
            _finding(findings, "error", "controller-linearization-missing", _relative(path), "LQR profile does not name a linearization artifact", "Bind the controller to a plant-derived A/B artifact.")
        if implementation in {"continuous_lqr", "lqr", "lqi", "gain_scheduled_lqr"}:
            artifact_value = payload.get("linearization_artifact")
            artifact_path = _path_from_value(artifact_value)
            if artifact_path is not None and not artifact_path.is_file():
                _finding(findings, "error", "controller-linearization-missing", _relative(artifact_path), "controller linearization artifact does not exist", "Generate the referenced linearization evidence before controller preflight.")
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
    return ControllerPreflightResult(family_id, status, tuple(profile_ids), tuple(findings), {"profiles_checked": len(profile_ids), "canonical_control_count": len(control_names)})
    ####


def _estimate_racetrack(binding: Mapping[str, Any]) -> float | None:
    try:
        speed = float(binding["speed_m_s"])
        straight = float(binding["straight_length_m"])
        radius = float(binding["turn_radius_m"])
        climb = abs(float(binding["climb_rate_m_s"]))
        descent = abs(float(binding["descent_rate_m_s"]))
        altitude_delta = abs(float(binding["high_altitude_m"]) - float(binding["low_altitude_m"]))
        margin = max(0.0, float(binding.get("simulation_margin_s", 0.0)))
    except (KeyError, TypeError, ValueError):
        return None
    if speed <= 0.0 or straight <= 0.0 or radius <= 0.0 or climb <= 0.0 or descent <= 0.0:
        return None
    estimate = (2.0 * straight + 2.0 * math.pi * radius) / speed
    return estimate + altitude_delta / climb + altitude_delta / descent + margin
    ####


def _mission_preflight(family_id: str) -> MissionPreflightResult:
    family_dir = ROOT / "families" / family_id
    binding_path = family_dir / "qualification/racetrack-binding.yaml"
    findings: list[PreflightFinding] = []
    if not binding_path.is_file():
        _finding(findings, "error", "mission-binding-missing", _relative(binding_path), "no reusable mission binding is declared for this family", "Declare a family mission binding before running or estimating a flagship mission.")
        return MissionPreflightResult(family_id, "blocked", None, (), tuple(findings), {"estimated_duration_s": None})
    try:
        binding = _read_yaml(binding_path)
    except (OSError, ValueError, yaml.YAMLError) as error:
        _finding(findings, "error", "mission-binding-load-failed", _relative(binding_path), str(error), "Repair the mission binding YAML before preflight.")
        return MissionPreflightResult(family_id, "blocked", None, (_relative(binding_path),), tuple(findings), {"estimated_duration_s": None})
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
        estimate = _estimate_racetrack(candidate)
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
