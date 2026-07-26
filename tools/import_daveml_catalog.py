"""Cycle the DAVE-ML catalog and emit Taoryx family-import evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

from taoryx.trajectory import build_daveml_family_import, build_daveml_ir, compare_daveml_ir, compare_daveml_numeric, export_daveml_ir

PACKAGE_FAMILIES = {
    "f16-s119": "reference_f16_s119",
    "hl20-mod-k": "reference_hl20_mod_k",
    "nesc-two-stage-rocket": "reference_nesc_two_stage_rocket",
}
FAMILY_BOUNDARIES = {
    "reference_f16_s119": {
        "claims": (
            "qualified F-16 source package is bound to the Taoryx rigid-body load contract",
            "package DAVE-ML source members complete the canonical structural and numeric cycle",
        ),
        "nonclaims": (
            "production F-16 training or operational fidelity",
            "actuator, SAS, autopilot, fuel-transient, or runway behavior from the source plant alone",
        ),
    },
    "reference_hl20_mod_k": {
        "claims": (
            "qualified HL-20 source package is bound to the Taoryx rigid-body load contract",
            "package DAVE-ML aerodynamics complete the canonical structural and numeric cycle",
        ),
        "nonclaims": (
            "landing gear, ground contact, flare, tire friction, touchdown, or rollout behavior",
            "generic hypersonic-glider or operational spacecraft performance",
        ),
    },
    "reference_nesc_two_stage_rocket": {
        "claims": (
            "qualified NESC Scenario 17 package is bound to the Taoryx rigid-body load contract",
            "DAVE-ML aerodynamics, propulsion, and scheduled inertia members are imported as separate components",
            "staging and variable-mass evidence remains visible through the package acceptance schedule",
        ),
        "nonclaims": (
            "independent full participating-simulation trajectory equivalence",
            "perigee performance beyond the package's retained acceptance boundary",
        ),
    },
}


def main() -> int:
    """Run the complete catalog source and qualified-package cycle."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--write-family-imports", type=Path, help="also write one JSON sidecar below this families root")
    arguments = parser.parse_args()
    report = cycle_catalog(arguments.catalog_root, family_output_root=arguments.write_family_imports)
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "verified" else 1


def cycle_catalog(catalog_root: str | Path, *, family_output_root: str | Path | None = None) -> dict[str, Any]:
    """Return deterministic source and qualified-package import evidence."""

    root = Path(catalog_root)
    release = _json(root / "release-manifest.json")
    source_rows = _json(root / "catalog/source-documents.json")
    model_rows = _json(root / "catalog/models.json")
    index_rows = _json(root / "qualified/qualified-index.json")
    source_reports = [_cycle_source(root, source, source_rows) for source in sorted((root / "normalized").glob("*/source.dml"))]
    package_reports = []
    for row in sorted(index_rows, key=lambda item: str(item.get("model_id", ""))):
        package = root / str(row["txair_path"]).replace("/", "\\")
        family_id = PACKAGE_FAMILIES.get(str(row.get("model_id")))
        if family_id is None:
            raise ValueError(f"qualified package has no family mapping: {row.get('model_id')}")
        imported = build_daveml_family_import(
            package,
            family_id=family_id,
            catalog_root=root.as_posix(),
            claims=tuple(FAMILY_BOUNDARIES[family_id]["claims"]),
            nonclaims=tuple(FAMILY_BOUNDARIES[family_id]["nonclaims"]),
            package_path_label=(root / str(row["txair_path"])).as_posix(),
        )
        package_report = imported.model_dump(mode="json")
        package_reports.append(package_report)
        if family_output_root is not None:
            destination = Path(family_output_root) / family_id / "plant/daveml-import.json"
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text(json.dumps(package_report, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    if not source_reports:
        raise ValueError(f"catalog has no normalized source documents: {root}")
    if not package_reports:
        raise ValueError(f"catalog has no qualified packages: {root}")
    source_failed = sum(int(item["structural_diff_count"]) + int(item["numeric_diff_count"]) for item in source_reports)
    package_failed = sum(
        int(item["roundtrip"]["structural_diff_count"])
        + int(item["roundtrip"]["numeric_diff_count"])
        + (0 if item["replay"]["status"] == "runtime_replay_qualification_passed" else 1)
        for item in package_reports
    )
    family_library = []
    for model in sorted(model_rows, key=lambda item: str(item.get("model_id", ""))):
        model_id = str(model["model_id"])
        family_library.append(
            {
                "model_id": model_id,
                "display_name": model.get("display_name", ""),
                "qualification_status": model.get("qualification_status", ""),
                "family_id": PACKAGE_FAMILIES.get(model_id),
                "library_status": "qualified_reference_family" if model_id in PACKAGE_FAMILIES else "catalog_source_record_only",
            }
        )
    return {
        "schema_version": "taoryx.daveml-catalog-import/v1",
        "status": "verified" if source_failed == 0 and package_failed == 0 else "failed",
        "catalog_id": release.get("catalog_id", release.get("artifact", "")),
        "catalog_root": root.as_posix(),
        "source_document_count": len(source_reports),
        "qualified_package_count": len(package_reports),
        "model_family_count": len(family_library),
        "sources": source_reports,
        "qualified_packages": package_reports,
        "family_library": family_library,
        "claim_boundary": [
            "Every normalized catalog source completed the canonical structural and numeric cycle.",
            "Only packages with a qualified txair artifact are promoted to executable reference families.",
            "Runtime replay evidence remains bounded by each package's declared host contract.",
        ],
    }


def _cycle_source(root: Path, source: Path, rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Cycle one normalized source and attach its catalog row."""

    relative = source.relative_to(root).as_posix()
    row = next((item for item in rows if item.get("normalized_path") == str(Path(relative).parent).replace("\\", "/")), {})
    payload = source.read_bytes()
    ir = build_daveml_ir(payload, document_id=relative)
    exported = export_daveml_ir(ir)
    fresh = build_daveml_ir(exported, document_id=relative)
    structural = compare_daveml_ir(ir, fresh)
    numeric = compare_daveml_numeric(ir, fresh)
    return {
        "document_id": row.get("document_id", relative),
        "model_id": row.get("model_id", ""),
        "component_role": row.get("component_role", ""),
        "normalized_path": relative,
        "source_sha256": _sha256(payload),
        "canonical_ir_sha256": _sha256(ir.canonical_json()),
        "canonical_export_sha256": _sha256(exported),
        "structural_diff_count": len(structural),
        "numeric_diff_count": len(numeric),
    }


def _json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid DAVE-ML catalog JSON: {path}: {error}") from error


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


if __name__ == "__main__":
    raise SystemExit(main())
