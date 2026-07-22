"""Audit vehicle problem files against the canonical model registry."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
REGISTRY = ROOT / "verification/vehicle_models.yaml"
CATALOG = ROOT / "verification/vehicle_catalog.yaml"
NUMBER = r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?"
PAIR = re.compile(rf"(?P<name>[A-Za-z0-9_-]+)=(?P<value>{NUMBER}|[^\s]+)")
FIELDS = {
    "reference_area_m2": "reference-area",
    "reference_length_m": "reference-length",
    "dry_mass_kg": "dry-mass-kg",
    "aero_alpha_reference_deg": "aero-alpha-reference-deg",
}
INERTIA_FIELDS = {"x": "inertia-x", "y": "inertia-y", "z": "inertia-z"}
ENVELOPE_FIELDS = {
    "min_forward_speed_m_s": "envelope-min-forward-speed",
    "max_speed_m_s": "envelope-max-speed",
    "max_mach": "envelope-max-mach",
    "max_alpha_deg": "envelope-max-alpha-deg",
    "max_beta_deg": "envelope-max-beta-deg",
}


def _problem_paths(vehicle: dict[str, Any]) -> tuple[Path, ...]:
    paths: set[Path] = set()
    for pattern in vehicle["audit_problem_globs"]:
        paths.update(ROOT.glob(pattern))
    return tuple(sorted(path for path in paths if path.is_file()))
####


def _runtime_vehicle_values(path: Path) -> dict[str, str] | None:
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("*runtime status vehicle "):
            return {match.group("name"): match.group("value") for match in PAIR.finditer(line)}
    return None
####


def _compare_number(findings: list[dict[str, str]], path: Path, key: str, actual: dict[str, str], expected: float) -> None:
    if key not in actual:
        findings.append({"path": str(path.relative_to(ROOT)), "field": key, "kind": "missing", "expected": str(expected)})
    elif float(actual[key]) != expected:
        findings.append({"path": str(path.relative_to(ROOT)), "field": key, "kind": "mismatch", "expected": str(expected), "actual": actual[key]})
####


def audit() -> dict[str, Any]:
    """Return a machine-readable vehicle provenance audit."""

    registry = yaml.safe_load(REGISTRY.read_text(encoding="utf-8"))
    catalog = yaml.safe_load(CATALOG.read_text(encoding="utf-8"))
    catalog_by_id = {str(entry["model_id"]): entry for entry in catalog["vehicles"]}
    findings: list[dict[str, str]] = []
    files_checked = 0
    vehicle_summary: dict[str, int] = {}
    for vehicle_id, vehicle in registry["vehicles"].items():
        source = vehicle.get("source", {})
        bundle = ROOT / str(source.get("bundle", ""))
        if not bundle.is_dir():
            findings.append({"vehicle": vehicle_id, "field": "source.bundle", "kind": "missing", "actual": str(bundle)})
        if not source.get("model_path"):
            findings.append({"vehicle": vehicle_id, "field": "source.model_path", "kind": "missing"})
        catalog_entry = catalog_by_id.get(vehicle_id)
        if catalog_entry is None:
            findings.append({"vehicle": vehicle_id, "field": "vehicle_catalog", "kind": "missing"})
        else:
            source_tables = set(vehicle.get("table_bindings", []))
            case_tables = set(catalog_entry.get("tables", []))
            unknown_tables = case_tables - source_tables
            if not case_tables or unknown_tables:
                findings.append({
                    "vehicle": vehicle_id,
                    "field": "table_bindings",
                    "kind": "catalog-unknown-table" if unknown_tables else "catalog-empty",
                    "expected": ";".join(sorted(source_tables)),
                    "actual": ";".join(sorted(case_tables)),
                })
        paths = _problem_paths(vehicle)
        vehicle_summary[vehicle_id] = len(paths)
        for path in paths:
            actual = _runtime_vehicle_values(path)
            if actual is None:
                continue
            files_checked += 1
            for model_key, problem_key in FIELDS.items():
                if model_key in vehicle:
                    _compare_number(findings, path, problem_key, actual, float(vehicle[model_key]))
            for inertia_key, problem_key in INERTIA_FIELDS.items():
                _compare_number(findings, path, problem_key, actual, float(vehicle["inertia_kg_m2"][inertia_key]))
            for envelope_key, problem_key in ENVELOPE_FIELDS.items():
                if envelope_key in vehicle.get("envelope", {}):
                    _compare_number(findings, path, problem_key, actual, float(vehicle["envelope"][envelope_key]))
            for model_key, problem_key in (("aero_load_mode", "aero-load-mode"), ("aero_wrench_frame", "aero-wrench-frame")):
                if model_key in vehicle and actual.get(problem_key) != str(vehicle[model_key]):
                    findings.append({"path": str(path.relative_to(ROOT)), "field": problem_key, "kind": "mismatch", "expected": str(vehicle[model_key]), "actual": actual.get(problem_key, "")})
    return {
        "registry": str(REGISTRY.relative_to(ROOT)),
        "vehicles": vehicle_summary,
        "files_checked": files_checked,
        "findings": findings,
        "status": "pass" if not findings else "fail",
    }
####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", type=Path, help="write the audit report to this path")
    args = parser.parse_args()
    report = audit()
    if args.json:
        args.json.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "pass" else 1
####


if __name__ == "__main__":
    raise SystemExit(main())
####
