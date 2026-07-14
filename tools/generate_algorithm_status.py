"""Generate the tracked algorithm implementation-status ledger."""

from __future__ import annotations

import csv
import importlib
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "metadata/algorithm_catalog/algorithms.json"
BINDINGS = ROOT / "docs/equations/implementation-bindings.md"
ROADMAP = ROOT / "metadata/algorithm_catalog/IMPLEMENTATION_ROADMAP.md"
OUTPUT = ROOT / "metadata/algorithm_catalog/algorithm_status.csv"
ID_RE = re.compile(r"`(TAOS-ALG-[A-Z0-9-]+)`")
BINDING_RE = re.compile(r"^\| `(?P<id>TAOS-ALG-[A-Z0-9-]+)` .*? \| (?P<binding>[^|]+) \| (?P<verification>[^|]+) \|$")
ROADMAP_RE = re.compile(r"^- \[(?P<mark>[ xX])\].*?\*\*(?P<id>TAOS-ALG-[A-Z0-9-]+)\s+[—-]")


def _binding_records(text: str) -> dict[str, tuple[str, str]]:
    """Extract documented executable targets and their verification paths."""

    records: dict[str, tuple[str, str]] = {}
    for line in text.splitlines():
        match = BINDING_RE.match(line)
        if match is None:
            continue
        targets = re.findall(r"`(taoryx\.[A-Za-z0-9_.]+)", match.group("binding"))
        tests = tuple(re.findall(r"`(tests/[^`]+)`", match.group("verification")))
        if targets:
            records[match.group("id")] = (targets[0], ";".join(tests))
    return records
####


def _target_importable(target: str) -> bool:
    """Check the documented target without treating a name as implementation."""

    module_name, _, symbol = target.rpartition(".")
    if not module_name or not symbol:
        return False
    try:
        module = importlib.import_module(module_name)
    except ImportError:
        return False
    return hasattr(module, symbol)
####


def _roadmap_status(text: str) -> dict[str, str]:
    """Extract completion marks from the human-readable implementation roadmap."""

    statuses: dict[str, str] = {}
    for line in text.splitlines():
        match = ROADMAP_RE.match(line)
        if match is not None:
            if match.group("id") in statuses:
                raise ValueError(f"algorithm appears more than once in roadmap: {match.group('id')}")
            statuses[match.group("id")] = "complete" if match.group("mark").lower() == "x" else "planned"
    return statuses
####


def generate() -> None:
    payload = json.loads(CATALOG.read_text(encoding="utf-8"))
    binding_text = BINDINGS.read_text(encoding="utf-8")
    roadmap_text = ROADMAP.read_text(encoding="utf-8")
    bound = set(ID_RE.findall(binding_text))
    records = _binding_records(binding_text)
    roadmap = _roadmap_status(roadmap_text)
    catalog_ids = {algorithm["id"] for algorithm in payload["algorithms"]}
    missing_roadmap = sorted(catalog_ids - roadmap.keys())
    extra_roadmap = sorted(roadmap.keys() - catalog_ids)
    if missing_roadmap or extra_roadmap:
        raise ValueError(f"catalog/roadmap IDs differ; missing={missing_roadmap}, extra={extra_roadmap}")
    rows = []
    for algorithm in payload["algorithms"]:
        algorithm_id = algorithm["id"]
        target, verification = records.get(algorithm_id, ("", ""))
        importable = bool(target) and _target_importable(target)
        verification_paths = tuple(path for path in verification.split(";") if path)
        verification_exists = bool(verification_paths) and all((ROOT / path).is_file() for path in verification_paths)
        rows.append(
            {
                "id": algorithm_id,
                "catalog_status": algorithm["status"],
                "roadmap_status": roadmap.get(algorithm_id, "missing"),
                "binding_documented": "true" if algorithm_id in bound else "false",
                "target_importable": "true" if importable else "false",
                "verification_paths_exist": "true" if verification_exists else "false",
                "implementation_stage": "unit_verified" if importable and verification_exists else "importable" if importable else "typed_binding" if algorithm_id in bound else "unbound",
                "historical_equivalence": "unverified",
                "target": f"{algorithm['implementation_target']['package']}.{algorithm['implementation_target']['module']}.{algorithm['implementation_target']['symbol']}",
                "documented_binding": target,
                "verification_tests": verification,
                "source_pages": ";".join(algorithm["source"]["manual_pages"]),
                "test_plan_units": str(len(algorithm["test_plan"]["unit"])),
            }
        )
    with OUTPUT.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    generate()
####
