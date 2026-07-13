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


FINAL_PRODUCTS: dict[str, Path] = {
    "TAOS_manual_reconstruction_v21_codex_handoff.pdf": ROOT / "build" / "manual.pdf",
    "TAOS_equation_provenance_audit_v21.pdf": ROOT
    / "qa"
    / "TAOS_equation_provenance_audit_v21.pdf",
    "TAOS_manual_visual_comparison_v19.pdf": ROOT
    / "qa"
    / "build"
    / "figure_comparison.pdf",
    "TAOS_targeted_figure_review_v19.pdf": ROOT
    / "qa"
    / "TAOS_targeted_figure_review_v19.pdf",
    "TAOS_source_text_fidelity_audit_v19.pdf": ROOT
    / "qa"
    / "TAOS_source_text_fidelity_audit_v19.pdf",
    "TAOS_fixture_validation_v19.pdf": ROOT
    / "qa"
    / "TAOS_fixture_validation_v19.pdf",
    "TAOS_chapter04_residual_audit_v19.pdf": ROOT
    / "qa"
    / "TAOS_chapter04_residual_audit_v19.pdf",
    "taoryx.language-0.1.0-py3-none-any.whl": ROOT
    / "dist"
    / "taoryx.language-0.1.0-py3-none-any.whl",
}


TRANSIENT_NAMES = {
    ".git",
    ".pytest_cache",
    "__pycache__",
    "taoryx.language.egg-info",
    "equation-audit-build",
    "targeted-build",
    "build-v19-targeted",
    "make_check_v21.log",
    "make_handoff_v21.log",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--version", default="21")
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "dist" / "taos-manual-codex-handoff-v21.zip",
    )
    return parser.parse_args()
####


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
        ####
    ####
    return digest.hexdigest()
####


def prepare_final_products(version: str) -> Path:
    output_dir = ROOT / "dist" / "final"
    output_dir.mkdir(parents=True, exist_ok=True)
    for path in output_dir.iterdir():
        if path.is_file():
            path.unlink()
        elif path.is_dir():
            shutil.rmtree(path)
        ####
    ####

    for destination_name, source_path in FINAL_PRODUCTS.items():
        if not source_path.exists() or source_path.stat().st_size == 0:
            raise FileNotFoundError(f"Required final product is missing: {source_path}")
        ####
        shutil.copy2(source_path, output_dir / destination_name)
    ####

    readme = f"""# TAOS Codex handoff final products - Version {version}

This directory contains the principal generated deliverables. The repository root contains the canonical sources, original 1995 PDF, metadata, scripts, parser package, tests, and reproducible build instructions.

- `TAOS_manual_reconstruction_v21_codex_handoff.pdf`: normalized reconstructed manual.
- `TAOS_equation_provenance_audit_v21.pdf`: complete 326-equation provenance audit.
- `TAOS_manual_visual_comparison_v19.pdf`: complete 72-figure comparison report; still current because later releases did not alter numbered figures.
- `TAOS_targeted_figure_review_v19.pdf`: focused review of Figures 1-5, 1-6, 2-1, and 4-2.
- `TAOS_source_text_fidelity_audit_v19.pdf`: latest complete manual-wide source-text audit included in the codebase.
- `TAOS_fixture_validation_v19.pdf`: parser-backed fixture validation report.
- `TAOS_chapter04_residual_audit_v19.pdf`: classified residual Chapter 4 audit.
- `taoryx.language-0.1.0-py3-none-any.whl`: installable Phase 1 parser/validator package.

The original `TAOS_manual_1995.pdf` is included at the bundle root.
"""
    (output_dir / "README.md").write_text(readme, encoding="utf-8")

    checksum_lines: list[str] = []
    for path in sorted(output_dir.iterdir()):
        if path.is_file() and path.name != "SHA256SUMS.txt":
            checksum_lines.append(f"{sha256_file(path)}  {path.name}")
        ####
    ####
    (output_dir / "SHA256SUMS.txt").write_text(
        "\n".join(checksum_lines) + "\n",
        encoding="utf-8",
    )
    return output_dir
####


def remove_transients(staged_root: Path, output_name: str) -> None:
    for path in sorted(staged_root.rglob("*"), reverse=True):
        if path.name in TRANSIENT_NAMES:
            if path.is_dir():
                shutil.rmtree(path)
            elif path.exists():
                path.unlink()
            ####
            continue
        ####
        if path.is_file() and (
            path.suffix in {".pyc", ".pyo"}
            or path.name == output_name
            or path.name.endswith(".zip") and path.parent.name == "dist"
        ):
            path.unlink()
        ####
    ####

    for relative in (
        Path("build") / "bdist.linux-x86_64",
        Path("build") / "lib",
        Path("qa") / "build",
    ):
        path = staged_root / relative
        if path.exists():
            shutil.rmtree(path)
        ####
    ####
####


def file_manifest(staged_root: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for path in sorted(staged_root.rglob("*")):
        if not path.is_file() or path.name == "BUNDLE_MANIFEST.json":
            continue
        ####
        records.append(
            {
                "path": path.relative_to(staged_root).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": sha256_file(path),
            }
        )
    ####
    return records
####


def write_bundle_metadata(staged_root: Path, version: str) -> None:
    records = file_manifest(staged_root)
    payload = {
        "schema_version": 1,
        "release": f"v{version}",
        "bundle_root": staged_root.name,
        "file_count": len(records),
        "total_bytes": sum(record["bytes"] for record in records),
        "source_pdf": "TAOS_manual_1995.pdf",
        "manual_pdf": "dist/final/TAOS_manual_reconstruction_v21_codex_handoff.pdf",
        "equation_provenance": "metadata/equations_provenance.json",
        "files": records,
    }
    (staged_root / "BUNDLE_MANIFEST.json").write_text(
        json.dumps(payload, indent=2) + "\n",
        encoding="utf-8",
    )

    contents = f"""# Bundle contents

Release: **Version {version}**

- Files: **{payload['file_count']}**
- Uncompressed bytes: **{payload['total_bytes']}**
- Original source: `TAOS_manual_1995.pdf`
- Reconstructed manual: `dist/final/TAOS_manual_reconstruction_v21_codex_handoff.pdf`
- Equation provenance: `metadata/equations_provenance.json`
- Codex instructions: `AGENTS.md`
- Handoff status and next work: `CODEX_HANDOFF.md`
- Build instructions: `BUILDING.md`

`BUNDLE_MANIFEST.json` records the size and SHA-256 digest of every bundled file.
"""
    (staged_root / "BUNDLE_CONTENTS.md").write_text(contents, encoding="utf-8")
####


def create_zip(staged_root: Path, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_suffix(output_path.suffix + ".tmp")
    if temporary_path.exists():
        temporary_path.unlink()
    ####
    try:
        with zipfile.ZipFile(
            temporary_path,
            mode="w",
            compression=zipfile.ZIP_DEFLATED,
            compresslevel=3,
            allowZip64=True,
        ) as archive:
            for path in sorted(staged_root.rglob("*")):
                if path.is_file():
                    archive.write(
                        path,
                        arcname=(
                            Path(staged_root.name) / path.relative_to(staged_root)
                        ).as_posix(),
                    )
                ####
            ####
        ####
        temporary_path.replace(output_path)
    finally:
        if temporary_path.exists():
            temporary_path.unlink()
        ####
    ####
####


def main() -> None:
    args = parse_args()
    prepare_final_products(args.version)
    with tempfile.TemporaryDirectory(prefix="taos-codex-handoff-") as directory:
        staged_root = Path(directory) / f"taos-manual-codex-handoff-v{args.version}"
        shutil.copytree(ROOT, staged_root)
        remove_transients(staged_root, args.output.name)
        write_bundle_metadata(staged_root, args.version)
        create_zip(staged_root, args.output)
    ####
    print(
        f"Created {args.output} ({args.output.stat().st_size} bytes, "
        f"SHA-256 {sha256_file(args.output)})."
    )
####


if __name__ == "__main__":
    main()
####
