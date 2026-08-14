"""Report whether the taoryx development environment is ready."""

from __future__ import annotations

import argparse
import importlib.metadata
import importlib.util
import platform
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

DOCUMENTATION_REQUIRED_TOOLS = frozenset(
    {"latexmk", "pdflatex", "xelatex", "pandoc", "pdfinfo", "pdftotext", "pdftoppm"}
)


@dataclass(frozen=True, slots=True)
class Check:
    label: str
    ok: bool
    detail: str
    required: bool = True


def package_check(
    name: str,
    import_name: str | None = None,
    distribution_name: str | None = None,
    *,
    required: bool = True,
) -> Check:
    import_name = import_name or name
    distribution_name = distribution_name or name
    import_present = importlib.util.find_spec(import_name) is not None
    try:
        installed_version = importlib.metadata.version(distribution_name)
    except importlib.metadata.PackageNotFoundError:
        installed_version = None
    present = import_present and installed_version is not None
    if present:
        detail = f"{installed_version}"
    elif not import_present:
        detail = "missing import"
    else:
        detail = f"distribution {distribution_name!r} is not installed"
    return Check(f"Python package: {name}", present, detail, required=required)


def command_check(name: str, *, required: bool = False) -> Check:
    path = shutil.which(name)
    return Check(
        f"External tool: {name}",
        path is not None,
        path or "missing from PATH",
        required=required,
    )


def run_check(*, documentation: bool = False) -> list[Check]:
    checks = [
        Check(
            "Python runtime",
            sys.version_info >= (3, 12),
            platform.python_version(),
        ),
        Check(
            "Virtual environment",
            sys.prefix != sys.base_prefix,
            str(Path(sys.prefix)),
            required=False,
        ),
    ]
    checks.extend(
        package_check(
            name,
            import_name,
            distribution_name,
            required=distribution_name != "taoryx-reference-models",
        )
        for name, import_name, distribution_name in (
            ("Taoryx core", "taoryx", "taoryx"),
            ("Taoryx DAVE-ML plug-in", "taoryx_daveml", "taoryx-daveml"),
            ("Taoryx debug-model plug-in", "taoryx_debug_models", "taoryx-debug-models"),
            ("Taoryx A320 plug-in", "taoryx_a320", "taoryx-a320"),
            ("Taoryx F-16 plug-in", "taoryx_f16", "taoryx-f16"),
            ("Taoryx Hummingbird plug-in", "taoryx_hummingbird", "taoryx-hummingbird"),
            ("Taoryx NESC plug-in", "taoryx_nesc", "taoryx-nesc"),
            ("Taoryx passive-bodies plug-in", "taoryx_passive_bodies", "taoryx-passive-bodies"),
            ("Taoryx Simple Aero plug-in", "taoryx_simple_aero", "taoryx-simple-aero"),
            ("Taoryx Dual Launch plug-in", "taoryx_dual_launch", "taoryx-dual-launch"),
            ("Taoryx X-15 plug-in", "taoryx_x15", "taoryx-x15"),
            ("Taoryx HL-20 plug-in", "taoryx_hl20", "taoryx-hl20"),
            (
                "Taoryx source-table fixed-wing plug-in",
                "taoryx_source_table_fixed_wing",
                "taoryx-source-table-fixed-wing",
            ),
            (
                "Taoryx reference-model compatibility aggregate",
                "taoryx_reference_models",
                "taoryx-reference-models",
            ),
            (
                "Taoryx reachability plug-in",
                "taoryx_reachability",
                "taoryx-reachability",
            ),
            ("pydantic", "pydantic", "pydantic"),
            ("PyYAML", "yaml", "PyYAML"),
            ("pypdf", "pypdf", "pypdf"),
            ("pytest", "pytest", "pytest"),
            ("build", "build", "build"),
            ("ruff", "ruff", "ruff"),
            ("mypy", "mypy", "mypy"),
            ("PyMuPDF", "fitz", "PyMuPDF"),
            ("Pillow", "PIL", "Pillow"),
            ("NumPy", "numpy", "numpy"),
            ("SciPy", "scipy", "scipy"),
            ("RapidFuzz", "rapidfuzz", "RapidFuzz"),
            ("ReportLab", "reportlab", "reportlab"),
        )
    )
    checks.extend(
        command_check(name, required=(documentation and name in DOCUMENTATION_REQUIRED_TOOLS) or required)
        for name, required in (
            ("latexmk", True),
            ("pdflatex", True),
            ("xelatex", False),
            ("pdfinfo", False),
            ("pdftotext", False),
            ("pdftoppm", False),
            ("qpdf", False),
            ("pandoc", False),
        )
    )
    repository_paths = [
        "pyproject.toml",
        "manual/manual.tex",
        "grammars/taos_problem.ebnf",
    ]
    if documentation:
        repository_paths.extend(
            (
                "docs/latex/taoryx_extensions_and_verification.tex",
                "docs/latex/taoryx_language_reference.tex",
            )
        )
    checks.extend(
        Check(
            f"Repository path: {path}",
            (ROOT / path).exists(),
            "present" if (ROOT / path).exists() else "missing",
        )
        for path in repository_paths
    )
    return checks


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--strict",
        action="store_true",
        help="return non-zero for any missing required package or tool",
    )
    parser.add_argument(
        "--docs",
        action="store_true",
        help="check every dependency required to build and render the documentation PDFs",
    )
    args = parser.parse_args()

    checks = run_check(documentation=args.docs)
    scope = "documentation PDF workflow" if args.docs else "development workflow"
    print(f"taoryx doctor — {scope} — Python {platform.python_version()} on {platform.system()}")
    for check in checks:
        marker = "OK" if check.ok else ("FAIL" if check.required else "WARN")
        print(f"[{marker:4}] {check.label}: {check.detail}")

    if args.strict and any(not check.ok and check.required for check in checks):
        print("\nEnvironment is not ready for the required workflow.")
        return 1
    print("\nEnvironment checks completed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
