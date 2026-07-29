"""Structural effectivity and actuator preflight diagnostics."""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import numpy as np
import yaml

from .control_allocation import EffectorEffectiveness, EffectorLimits, allocate_and_advance_wrench
from .vehicle_registry import ROOT

EffectivityStatus = Literal["passed", "development", "blocked", "not_applicable"]


@dataclass(frozen=True, slots=True)
class EffectivityFinding:
    """One effectivity or actuator diagnostic."""

    severity: Literal["error", "warning", "info"]
    code: str
    path: str
    message: str
    hint: str

    def as_dict(self) -> dict[str, str]:
        """Return a stable finding record."""

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
class VehicleEffectivityPreflightReport:
    """Effectivity/actuator preflight for one family."""

    family_id: str
    status: EffectivityStatus
    evidence: tuple[str, ...]
    findings: tuple[EffectivityFinding, ...]
    metrics: Mapping[str, object]

    @property
    def blockers(self) -> tuple[EffectivityFinding, ...]:
        """Return blocking findings."""

        return tuple(item for item in self.findings if item.severity == "error")
        ####

    def as_dict(self) -> dict[str, object]:
        """Return a stable JSON report."""

        return {
            "schema_version": "taoryx.vehicle-effectivity-preflight/v1",
            "family_id": self.family_id,
            "status": self.status,
            "evidence": list(self.evidence),
            "metrics": dict(self.metrics),
            "error_count": len(self.blockers),
            "findings": [item.as_dict() for item in self.findings],
        }
        ####
    ####


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)
    ####


def _read(path: Path) -> Mapping[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8")) if path.suffix == ".json" else yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, Mapping):
        raise ValueError(f"{path} must contain a mapping")
    return value
    ####


def _finding(findings: list[EffectivityFinding], severity: Literal["error", "warning", "info"], code: str, path: str, message: str, hint: str) -> None:
    findings.append(EffectivityFinding(severity, code, path, message, hint))
    ####


def _path(value: object) -> Path | None:
    label = str(value).split("#", 1)[0].strip()
    if not label or label in {"none", "not_applicable", "not_available"} or not any(token in label for token in ("/", ".json", ".yaml", ".yml")):
        return None
    candidate = Path(label)
    return candidate if candidate.is_absolute() else ROOT / candidate
    ####


def _matrix_candidate(payload: Mapping[str, Any]) -> Mapping[str, Any] | None:
    candidates: list[Mapping[str, Any]] = []
    for key in ("effectiveness", "design", "metrics"):
        candidate = payload.get(key)
        if isinstance(candidate, Mapping):
            candidates.append(candidate)
    samples = payload.get("samples")
    if isinstance(samples, list):
        candidates.extend(item for item in samples if isinstance(item, Mapping))
    return next(
        (
            candidate
            for candidate in candidates
            if "matrix" in candidate or "effectiveness_matrix" in candidate
        ),
        None,
    )
    ####


def _inspect_matrix(path: Path, payload: Mapping[str, Any], findings: list[EffectivityFinding]) -> dict[str, object]:
    effectiveness = _matrix_candidate(payload)
    if effectiveness is None:
        return {}
    samples = payload.get("samples")
    matrix_value = effectiveness.get("matrix", effectiveness.get("effectiveness_matrix"))
    try:
        matrix = np.asarray(matrix_value, dtype=float)
    except (TypeError, ValueError):
        matrix = np.asarray(())
    if matrix.ndim != 2 or not matrix.size or not np.isfinite(matrix).all():
        _finding(findings, "error", "effectivity-matrix-invalid", _relative(path), "effectiveness matrix is not a finite rectangular numeric matrix", "Emit a numeric local effector-to-wrench matrix from the source evaluator.")
        return {}
    declared_rank = effectiveness.get("rank", effectiveness.get("effectiveness_rank"))
    actual_rank = int(np.linalg.matrix_rank(matrix))
    singular_values = np.linalg.svd(matrix, compute_uv=False)
    nonzero_singular_values = singular_values[singular_values > 1.0e-12]
    condition_number = float(singular_values[0] / nonzero_singular_values[-1]) if nonzero_singular_values.size else float("inf")
    if declared_rank is not None and int(declared_rank) != actual_rank:
        _finding(findings, "error", "effectivity-rank-mismatch", _relative(path), f"declared rank {declared_rank} differs from computed rank {actual_rank}", "Regenerate the evidence from the same matrix and ordering.")
    effector_names = payload.get("effector_names")
    wrench_names = payload.get("wrench_names")
    if isinstance(effector_names, list) and len(effector_names) != matrix.shape[1]:
        _finding(findings, "error", "effectivity-effector-order-mismatch", _relative(path), "effector name count does not match matrix columns", "Keep matrix columns and physical effector ordering synchronized.")
    if isinstance(wrench_names, list) and len(wrench_names) != matrix.shape[0]:
        _finding(findings, "error", "effectivity-wrench-order-mismatch", _relative(path), "wrench name count does not match matrix rows", "Keep matrix rows and wrench ordering synchronized.")
    _finding(findings, "warning", "effectivity-development-screen", _relative(path), "numeric effectivity evidence is a local development/allocation screen", "Do not promote it to controller qualification without actuator and nonlinear response evidence.")
    matrix_source = "sample" if isinstance(samples, list) and any(effectiveness is item for item in samples) else "declared"
    axis_names = payload.get("wrench_names")
    if not isinstance(axis_names, list):
        axis_names = payload.get("controlled_wrench_axes")
    if not isinstance(axis_names, list):
        axis_names = effectiveness.get("controlled_wrench_axes")
    axes = tuple(str(name) for name in axis_names) if isinstance(axis_names, list) and len(axis_names) == matrix.shape[0] else tuple(f"axis_{index}" for index in range(matrix.shape[0]))
    effector_labels = payload.get("effector_names")
    if not isinstance(effector_labels, list) and isinstance(effectiveness.get("actual_effectors"), Mapping):
        effector_labels = list(effectiveness["actual_effectors"])
    effectors = tuple(str(name) for name in effector_labels) if isinstance(effector_labels, list) and len(effector_labels) == matrix.shape[1] else tuple(f"effector_{index}" for index in range(matrix.shape[1]))
    sign_probe = {
        effector: {
            "positive_axes": [axis for axis, value in zip(axes, matrix[:, index], strict=True) if value > 1.0e-12],
            "negative_axes": [axis for axis, value in zip(axes, matrix[:, index], strict=True) if value < -1.0e-12],
            "zero_axes": [axis for axis, value in zip(axes, matrix[:, index], strict=True) if abs(value) <= 1.0e-12],
        }
        for index, effector in enumerate(effectors)
    }
    return {
        "matrix_rows": int(matrix.shape[0]),
        "matrix_columns": int(matrix.shape[1]),
        "computed_rank": actual_rank,
        "matrix_source": matrix_source,
        "singular_values": [float(value) for value in singular_values],
        "condition_number": condition_number,
        "nonzero_column_count": int(np.count_nonzero(np.linalg.norm(matrix, axis=0) > 1.0e-12)),
        "sign_probe": sign_probe,
    }
    ####


def _normal_token(value: object) -> str:
    return "".join(character for character in str(value).lower() if character.isalnum())
    ####


def _load_effector_limits(family_dir: Path, effectors: Sequence[str]) -> dict[str, EffectorLimits] | None:
    actuator_dir = family_dir / "actuators"
    profiles = tuple(sorted(actuator_dir.glob("*.yaml"))) if actuator_dir.is_dir() else ()
    profile_path = next((path for path in profiles if "reference-first-order" in path.stem), profiles[0] if profiles else None)
    if profile_path is None:
        return None
    try:
        payload = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError):
        return None
    raw_limits = payload.get("limits") if isinstance(payload, Mapping) else None
    if not isinstance(raw_limits, Mapping):
        return None
    limits: dict[str, EffectorLimits] = {}
    normalized_limits = {_normal_token(name): value for name, value in raw_limits.items() if isinstance(value, Mapping)}
    for name in effectors:
        normalized = _normal_token(name)
        limit = normalized_limits.get(normalized)
        if limit is None:
            limit = next(
                (value for key, value in normalized_limits.items() if key in normalized or normalized in key),
                None,
            )
        if not isinstance(limit, Mapping):
            return None
        position = limit.get("position")
        if isinstance(position, Sequence) and not isinstance(position, (str, bytes)) and len(position) == 2:
            lower = position[0]
            upper = position[1]
        else:
            lower = limit.get("lower")
            upper = limit.get("upper")
        if lower is None or upper is None:
            return None
        rate_limit = limit.get("rate_limit_per_s", limit.get("rate_per_s"))
        limits[name] = EffectorLimits(
            name=name,
            lower=float(lower),
            upper=float(upper),
            unit=str(limit.get("unit", "unspecified")),
            rate_limit_per_s=float(rate_limit) if rate_limit is not None else None,
            time_constant_s=float(limit.get("time_constant_s", 0.0)),
        )
    return limits
    ####


def _bounded_replay(
    family_id: str,
    payload: Mapping[str, Any],
    matrix_payload: Mapping[str, Any],
    findings: list[EffectivityFinding],
) -> dict[str, object]:
    effectors_value = payload.get("effector_names")
    wrench_value = payload.get("wrench_names")
    effectors = tuple(str(item) for item in effectors_value) if isinstance(effectors_value, list) else ()
    wrench_names = tuple(str(item) for item in wrench_value) if isinstance(wrench_value, list) else ()
    if not effectors:
        sample_effectors = matrix_payload.get("actual_effectors")
        effectors = tuple(str(name) for name in sample_effectors) if isinstance(sample_effectors, Mapping) else ()
    if not wrench_names:
        sample_wrench = matrix_payload.get("achieved_wrench")
        if isinstance(sample_wrench, Mapping):
            wrench_names = tuple(str(name) for name in sample_wrench)
        else:
            controlled_axes = matrix_payload.get("controlled_wrench_axes")
            wrench_names = tuple(str(name) for name in controlled_axes) if isinstance(controlled_axes, list) else ()
    matrix_value = matrix_payload.get("matrix", matrix_payload.get("effectiveness_matrix"))
    reference_effectors_value = matrix_payload.get("reference_effectors", matrix_payload.get("actual_effectors"))
    reference_wrench_value = matrix_payload.get("reference_wrench", matrix_payload.get("achieved_wrench"))
    if not effectors or not wrench_names or not isinstance(reference_effectors_value, Mapping) or not isinstance(reference_wrench_value, Mapping):
        return {"status": "not_run", "reason": "effectivity evidence lacks ordered names and a reference state"}
    limits = _load_effector_limits(ROOT / "families" / family_id, effectors)
    if limits is None:
        return {"status": "not_run", "reason": "no compatible actuator-limit profile"}
    try:
        matrix = tuple(tuple(float(value) for value in row) for row in matrix_value)
        effectiveness = EffectorEffectiveness(
            wrench_names=wrench_names,
            effector_names=effectors,
            matrix=matrix,
            reference_wrench={name: float(reference_wrench_value[name]) for name in wrench_names},
            reference_effectors={name: float(reference_effectors_value[name]) for name in effectors},
            source=f"{family_id}:preflight-evidence",
        )
        result = allocate_and_advance_wrench(
            effectiveness,
            limits,
            effectiveness.reference_wrench,
            effectiveness.reference_effectors,
            float(payload.get("dt_s", 0.01)),
        )
    except (KeyError, TypeError, ValueError, np.linalg.LinAlgError) as error:
        _finding(findings, "warning", "bounded-replay-failed", family_id, f"bounded allocation replay could not be evaluated: {error}", "Provide compatible effector names, limits, reference effectors, and reference wrench values.")
        return {"status": "failed", "reason": str(error)}
    if result.allocation.status not in {"feasible", "feasible_near_limit"}:
        _finding(findings, "warning", "bounded-replay-infeasible", family_id, f"reference allocation replay returned {result.allocation.status!r}", "Keep the physical allocation path at development status until the reference demand is feasible under declared limits.")
    _finding(findings, "warning", "bounded-replay-development", family_id, "bounded allocation replay is a generic numerical screen, not nonlinear actuator qualification", "Retain command/actual, rate, saturation, and nonlinear response evidence before promotion.")
    return {
        "status": result.allocation.status,
        "residual_norm": result.achieved_controlled_residual_norm,
        "effectivity_rank": result.allocation.effectiveness_rank,
        "saturated_effectors": sorted(set(result.allocation.position_saturated) | set(result.allocation.rate_limited)),
    }


def _inspect_overlay(path: Path, payload: Mapping[str, Any], findings: list[EffectivityFinding]) -> dict[str, object]:
    if str(payload.get("qualification_class", "")).lower() == "synthetic" or str(payload.get("authority", "")).lower().endswith("overlay_only"):
        _finding(findings, "warning", "synthetic-effectivity-overlay", _relative(path), "effectivity is a downstream or synthetic overlay rather than source-declared authority", "Keep source and overlay claims separate and require sign, saturation, and nonlinear regression evidence.")
    physical = set(str(item) for item in payload.get("physical_outputs", ()) if isinstance(item, str))
    mapping = payload.get("mapping")
    if isinstance(mapping, Mapping):
        missing = sorted({str(output) for outputs in mapping.values() if isinstance(outputs, list) for output in outputs} - physical)
        if missing:
            _finding(findings, "error", "overlay-output-unbound", _relative(path), f"logical mapping references undeclared physical outputs: {missing}", "Declare every mapped effector in physical_outputs.")
    return {"logical_command_count": len(mapping) if isinstance(mapping, Mapping) else 0, "physical_output_count": len(physical)}
    ####


def validate_vehicle_effectivity_preflight(family_id: str) -> VehicleEffectivityPreflightReport:
    """Validate declared numeric effectivity, overlays, and actuator profiles."""

    family_dir = ROOT / "families" / family_id
    record_path = family_dir / "qualification/integration-record.yaml"
    findings: list[EffectivityFinding] = []
    evidence: list[str] = []
    metrics: dict[str, object] = {}
    if not record_path.is_file():
        _finding(findings, "error", "integration-record-missing", _relative(record_path), "family integration record is missing", "Declare effectivity and actuator layers in the family integration record.")
        return VehicleEffectivityPreflightReport(family_id, "blocked", (), tuple(findings), metrics)
    try:
        record = _read(record_path)
    except (OSError, ValueError, json.JSONDecodeError, yaml.YAMLError) as error:
        _finding(findings, "error", "integration-record-load-failed", _relative(record_path), str(error), "Repair the family integration record before effectivity preflight.")
        return VehicleEffectivityPreflightReport(family_id, "blocked", (), tuple(findings), metrics)
    layers = record.get("layers")
    if not isinstance(layers, Mapping):
        _finding(findings, "error", "effectivity-layers-missing", _relative(record_path), "integration record has no layers map", "Declare allocation and actuator evidence separately.")
        return VehicleEffectivityPreflightReport(family_id, "blocked", (), tuple(findings), metrics)
    allocation_path = _path(layers.get("allocation", {}).get("evidence") if isinstance(layers.get("allocation"), Mapping) else None)
    if allocation_path is None:
        family_manifest_path = family_dir / "family.yaml"
        if family_manifest_path.is_file():
            try:
                family_manifest = _read(family_manifest_path)
            except (OSError, ValueError, json.JSONDecodeError, yaml.YAMLError):
                family_manifest = {}
            manifest_layers = family_manifest.get("layers") if isinstance(family_manifest, Mapping) else None
            allocation_path = _path(manifest_layers.get("allocation", {}).get("evidence") if isinstance(manifest_layers, Mapping) and isinstance(manifest_layers.get("allocation"), Mapping) else None)
    if allocation_path is not None:
        evidence.append(_relative(allocation_path))
        if not allocation_path.is_file():
            _finding(findings, "error", "allocation-evidence-missing", _relative(allocation_path), "allocation evidence does not exist", "Generate or correct the declared effectivity artifact.")
        else:
            try:
                allocation = _read(allocation_path)
                matrix_metrics = _inspect_matrix(allocation_path, allocation, findings)
                metrics.update(matrix_metrics)
                if not matrix_metrics:
                    _finding(findings, "warning", "allocation-evidence-not-numeric", _relative(allocation_path), "allocation evidence does not contain a numeric local effectivity matrix", "Keep the overlay contract explicit and add a numeric sign, saturation, and residual screen before promotion.")
                else:
                    matrix_payload = _matrix_candidate(allocation)
                    if matrix_payload is not None:
                        metrics["bounded_replay"] = _bounded_replay(family_id, allocation, matrix_payload, findings)
                if "effectiveness" not in allocation:
                    overlays = allocation.get("overlays")
                    overlay = next(
                        (
                            item
                            for item in overlays
                            if isinstance(item, Mapping)
                            and str(item.get("family_id")) == family_id
                            and str(item.get("kind", "allocation")) == "allocation"
                        ),
                        None,
                    ) if isinstance(overlays, list) else None
                    overlay_artifact = _path(overlay.get("artifact") if isinstance(overlay, Mapping) else None)
                    if overlay_artifact is not None and overlay_artifact.is_file():
                        evidence.append(_relative(overlay_artifact))
                        metrics.update(_inspect_overlay(overlay_artifact, _read(overlay_artifact), findings))
            except (OSError, ValueError, TypeError, json.JSONDecodeError, yaml.YAMLError) as error:
                _finding(findings, "error", "allocation-evidence-load-failed", _relative(allocation_path), str(error), "Repair the declared allocation artifact.")
    else:
        _finding(findings, "warning", "allocation-evidence-not-numeric", _relative(record_path), "no numeric allocation matrix is declared; any overlay remains contract-level evidence", "Add a source- or adapter-derived effectivity matrix when the family supports one.")
    actuator_layer = layers.get("actuators")
    actuator_path = _path(actuator_layer.get("evidence") if isinstance(actuator_layer, Mapping) else None)
    if actuator_path is not None and actuator_path.is_file():
        evidence.append(_relative(actuator_path))
    elif actuator_layer and str(actuator_layer).startswith("no_source"):
        _finding(findings, "warning", "actuator-source-missing", _relative(record_path), "source actuator dynamics are not available", "Use an explicitly labeled overlay and report its limits, lag, and provenance.")
    elif actuator_layer:
        _finding(findings, "warning", "actuator-evidence-not-resolved", _relative(record_path), "actuator layer is declared without a resolvable evidence artifact", "Declare the actuator overlay path or explicitly state that only a contract-level screen exists.")
    status: EffectivityStatus = "blocked" if any(item.severity == "error" for item in findings) else "development" if findings else "passed"
    return VehicleEffectivityPreflightReport(family_id, status, tuple(dict.fromkeys(evidence)), tuple(findings), metrics)
    ####


def validate_all_vehicle_effectivity_preflight() -> tuple[VehicleEffectivityPreflightReport, ...]:
    """Run effectivity preflight for the conformance pilots."""

    return tuple(validate_vehicle_effectivity_preflight(family_id) for family_id in ("reference_f16_s119", "reference_hl20_mod_k"))
    ####


__all__ = [
    "EffectivityFinding",
    "VehicleEffectivityPreflightReport",
    "validate_all_vehicle_effectivity_preflight",
    "validate_vehicle_effectivity_preflight",
]
####
