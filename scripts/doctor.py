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


@dataclass(frozen=True)
class Check:
    label: str
    ok: bool
    detail: str
    required: bool = True


def package_check(
    name: str,
    import_name: str | None = None,
    distribution_name: str | None = None,
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
    return Check(f"Python package: {name}", present, detail)


def command_check(name: str, *, required: bool = False) -> Check:
    path = shutil.which(name)
    return Check(
        f"External tool: {name}",
        path is not None,
        path or "missing from PATH",
        required=required,
    )


def run_check() -> list[Check]:
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
        package_check(name, import_name, distribution_name)
        for name, import_name, distribution_name in (
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
            ("RapidFuzz", "rapidfuzz", "RapidFuzz"),
            ("ReportLab", "reportlab", "reportlab"),
        )
    )
    checks.extend(
        command_check(name, required=required)
        for name, required in (
            ("latexmk", True),
            ("pdflatex", True),
            ("pdfinfo", False),
            ("pdftotext", False),
            ("pdftoppm", False),
            ("qpdf", False),
            ("pandoc", False),
        )
    )
    checks.extend(
        Check(
            f"Repository path: {path}",
            (ROOT / path).exists(),
            "present" if (ROOT / path).exists() else "missing",
        )
        for path in ("pyproject.toml", "manual/manual.tex", "grammars/taos_problem.ebnf")
    )
    return checks


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--strict",
        action="store_true",
        help="return non-zero for any missing required package or tool",
    )
    args = parser.parse_args()

    checks = run_check()
    print(f"taoryx doctor — Python {platform.python_version()} on {platform.system()}")
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
