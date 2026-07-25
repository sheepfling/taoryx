"""Build the self-contained, path-sanitized TAORYX Alpha 1 evidence packet."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import tempfile
import zipfile
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
PACKET_NAME = "taoryx-alpha1-evidence-v1"
DEFAULT_OUTPUT = ROOT / "dist" / f"{PACKET_NAME}.zip"

STATIC_FILES = (
    "docs/plan/taoryx-alpha-1.md",
    "docs/plan/taoryx-alpha-1-progress.md",
    "verification/claims.md",
    "verification/evidence_policy.md",
    "verification/alpha1_release_plan.yaml",
    "verification/alpha1_feature_matrix.yaml",
    "verification/alpha1_manual_execution.yaml",
    "verification/alpha1_composition_case.yaml",
    "verification/alpha1_vehicle_evidence.yaml",
    "verification/family_validation_execution.yaml",
    "verification/staged_completion_matrix.yaml",
    "verification/trim_specs.yaml",
    "verification/vehicle_models.yaml",
    "verification/x15_trim_adjudication.yaml",
    "artifacts/verification/alpha1/alpha1-status.json",
    "artifacts/verification/alpha1/manual_examples/report.json",
    "artifacts/verification/alpha1/composition_case/report.json",
    "artifacts/golden_plants/b747_condition3_trim_report.json",
    "artifacts/golden_plants/x8_powered_trim_report.json",
    "artifacts/golden_plants/hummingbird_hover_trim_report.json",
    "artifacts/golden_plants/x15_release_glide_trim_report.json",
    "artifacts/tests_e2e_test_golden_source_differential.py_test_source_differential_report_writes_review_artifact/source-differential/golden_static_grid_report.json",
    "examples/chapter04/ballistic-reentry.prb",
    "examples/chapter04/ballistic-rocket.prb",
    "examples/chapter04/air-launched-intercept.prb",
    "examples/chapter04/ground-launched-intercept.prb",
    "examples/generated/vehicles/b747_canonical_source_anchor_6dof.prb",
    "examples/generated/vehicles/skywalker_x8_canonical_6dof.prb",
    "examples/generated/vehicles/hummingbird_canonical_hover_6dof.prb",
    "examples/showcases/x15_rocket_to_hawaii/long_unpowered_glide_6dof.prb",
    "examples/mission_families/slower_b747/SV01_long_trim_hold_120_6dof.prb",
    "examples/mission_families/slower_x8/SV03_long_rectangle_route_6dof.prb",
    "examples/mission_families/slower_hummingbird/SV05_rectangle_course_6dof.prb",
)

TABLE_FILES = (
    "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/b747_nominal_static_6axis.tbl",
    "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/b747_nominal_elevator_6axis.tbl",
    "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/b747_jt9d_thrust.tbl",
    "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/skywalker_x8_static_6axis.tbl",
    "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/skywalker_x8_collective_elevon_6axis.tbl",
    "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/skywalker_x8_differential_elevon_6axis.tbl",
    "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/skywalker_x8_thrust.tbl",
    "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/hummingbird_rotor_static.tbl",
    "tests/fixtures/x15_coherent_6dof_public_research_v1/tables/x15_static_6axis.tbl",
    "tests/fixtures/x15_coherent_6dof_public_research_v1/tables/x15_symmetric_stabilator_6axis.tbl",
    "tests/fixtures/x15_coherent_6dof_public_research_v1/tables/x15_differential_stabilator_6axis.tbl",
    "tests/fixtures/x15_coherent_6dof_public_research_v1/tables/x15_rudder_6axis.tbl",
)

ARTIFACT_DIRECTORIES = (
    (
        "artifacts/tests_e2e_test_vehicle_family_validation.py_test_b747_long_6dof_trim_hold_writes_contract_and_convergence_artifacts/vehicle-family-validation/b747-trim-hold-contract",
        "evidence/family/b747",
    ),
    (
        "artifacts/tests_e2e_test_x8_family_validation.py_test_x8_long_powered_candidate_writes_family_artifacts/vehicle-family-validation/x8-powered-validation-6dof",
        "evidence/family/x8",
    ),
    (
        "artifacts/tests_e2e_test_hummingbird_family_validation.py_test_hummingbird_return_home_landing_writes_phase_artifacts/vehicle-family-validation/hummingbird-return-home-land",
        "evidence/family/hummingbird",
    ),
    (
        "artifacts/tests_e2e_test_glider_family_validation.py_test_glider_6dof_writes_family_artifacts/vehicle-family-validation/glider-unpowered-6dof",
        "evidence/family/x15",
    ),
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
    ####


def _sanitized_text(text: str) -> str:
    """Remove workstation-specific paths while retaining source references."""

    return text.replace(str(ROOT), "<repo-root>").replace("/private/tmp/", "<temp-root>/")
    ####


def _copy_file(source: Path, destination: Path, records: list[dict[str, Any]]) -> None:
    if not source.is_file():
        raise FileNotFoundError(f"Alpha 1 packet input is missing: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    source_hash = _sha256(source)
    sanitized = source.suffix.lower() in {".json", ".yaml", ".yml", ".md", ".prb", ".txt"}
    if sanitized:
        destination.write_text(_sanitized_text(source.read_text(encoding="utf-8")), encoding="utf-8")
    else:
        shutil.copy2(source, destination)
    records.append(
        {
            "source_path": source.relative_to(ROOT).as_posix(),
            "packet_path": destination.relative_to(next(parent for parent in destination.parents if parent.name == PACKET_NAME)).as_posix(),
            "source_sha256": source_hash,
            "packet_sha256": _sha256(destination),
            "sanitized_paths": sanitized,
            "bytes": destination.stat().st_size,
        }
    )
    ####


def _copy_tree(source: Path, destination: Path, records: list[dict[str, Any]]) -> None:
    if not source.is_dir():
        raise FileNotFoundError(f"Alpha 1 packet artifact directory is missing: {source}")
    for child in sorted(source.rglob("*")):
        if child.is_file():
            _copy_file(child, destination / child.relative_to(source), records)
    ####


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()
    ####


def main() -> None:
    args = _parse_args()
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    # Keep the deeply nested packet staging path beside the ZIP on Windows;
    # the system temp prefix can otherwise exceed MAX_PATH before archiving.
    with tempfile.TemporaryDirectory(prefix="a1-", dir=ROOT.parent) as directory:
        stage = Path(directory) / PACKET_NAME
        stage.mkdir()
        records: list[dict[str, Any]] = []
        for relative in (*STATIC_FILES, *TABLE_FILES):
            source = ROOT / relative
            _copy_file(source, stage / relative, records)
        for source_relative, packet_relative in ARTIFACT_DIRECTORIES:
            _copy_tree(ROOT / source_relative, stage / packet_relative, records)
        manifest = {
            "schema_version": 1,
            "release": "taoryx-alpha-1",
            "packet_name": PACKET_NAME,
            "claim_boundary": "Evidence-bounded TAORYX Alpha 1 claims; not historical TAOS compatibility or flight qualification.",
            "file_count": len(records),
            "files": sorted(records, key=lambda item: item["packet_path"]),
        }
        (stage / "bundle-manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        readme = """# TAORYX Alpha 1 evidence packet

This packet is a self-contained snapshot of the Alpha 1 release evidence.
Paths inside textual artifacts are sanitized to `<repo-root>` or `<temp-root>`.
The packet preserves the distinction between source-backed baseline plant
evidence and deferred route/controller candidates.

Rebuild from the repository with:

```text
python tools/dev.py alpha1-manual-examples
python tools/dev.py alpha1-composition-case
python tools/dev.py audit-alpha1
python tools/dev.py alpha1-packet
```

`bundle-manifest.json` records source and packet SHA-256 values for every
included file. The X-15 source-only trim diagnostic is intentional and does not
claim an equilibrium that the source release data do not establish.
"""
        (stage / "README.md").write_text(readme, encoding="utf-8")
        temporary = output.with_suffix(output.suffix + ".tmp")
        if temporary.exists():
            temporary.unlink()
        with zipfile.ZipFile(temporary, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
            for path in sorted(stage.rglob("*")):
                if path.is_file():
                    archive.write(path, arcname=f"{PACKET_NAME}/{path.relative_to(stage).as_posix()}")
        temporary.replace(output)
        packet_manifest = ROOT / "artifacts/verification/alpha1/packet/bundle-manifest.json"
        packet_manifest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(stage / "bundle-manifest.json", packet_manifest)
    print(f"Created {output} ({output.stat().st_size} bytes, SHA-256 {_sha256(output)})")
    ####


if __name__ == "__main__":
    main()
####
