"""Portable development task runner for the taoryx repository."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Callable

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
TOOLS = ROOT / "tools"
VENV_PYTHON = (
    ROOT / ".venv" / ("Scripts/python.exe" if sys.platform == "win32" else "bin/python")
)


def project_python() -> str:
    if VENV_PYTHON.exists():
        return str(VENV_PYTHON)
    ####
    return sys.executable
####


def run(command: list[str]) -> None:
    print("+", " ".join(command))
    subprocess.run(command, cwd=ROOT, check=True)
    ####
####


def python_tool(script: str, *args: str) -> list[str]:
    return [project_python(), str(SCRIPTS / script), *args]
####


def tool_script(script: str, *args: str) -> list[str]:
    return [project_python(), str(TOOLS / script), *args]
####


def bootstrap() -> None:
    run(python_tool("bootstrap.py"))
####


def source_pdf() -> None:
    run(python_tool("fetch_source_pdf.py", "--link-root"))
####


def doctor() -> None:
    run(python_tool("doctor.py"))
####


def lint() -> None:
    run([project_python(), "-m", "ruff", "check", "src", "tests", "tools", "scripts"])
####


def typecheck() -> None:
    run([project_python(), "-m", "mypy"])
####


def test() -> None:
    run([project_python(), "-m", "pytest"])
####


def grammar() -> None:
    run([project_python(), "-m", "pytest", "tests/parser"])
    run([project_python(), str(TOOLS / "check_taos_fixtures.py")])
####


def manual() -> None:
    run(["latexmk", "manual/manual.tex"])
    run(tool_script("normalize_pdf.py", "build/manual.pdf"))
####


def equation_audit() -> None:
    source_pdf = os.environ.get("SOURCE_PDF", "TAOS_manual_1995.pdf")
    run(
        [
            project_python(),
            str(TOOLS / "audit_equation_provenance.py"),
            "--source-pdf",
            source_pdf,
            "--render-source-pages",
            "--version",
            "21",
        ]
    )
    audit_build = ROOT / "qa" / "equation-audit-build"
    shutil.rmtree(audit_build, ignore_errors=True)
    audit_build.mkdir(parents=True, exist_ok=True)
    run(
        [
            "latexmk",
            "-pdf",
            "-interaction=nonstopmode",
            "-halt-on-error",
            f"-output-directory={audit_build}",
            "qa/equation_provenance_audit.tex",
        ]
    )
    shutil.copy2(audit_build / "equation_provenance_audit.pdf", ROOT / "qa/TAOS_equation_provenance_audit_v21.pdf")
    run(tool_script("check_equation_provenance.py"))
####


def handoff() -> None:
    equation_audit()
    check()
    run([project_python(), "-m", "build", "--wheel", "--outdir", "dist"])
    run(
        tool_script(
            "build_handoff_bundle.py",
            "--version",
            "21",
            "--output",
            "dist/taos-manual-codex-handoff-v21.zip",
        )
    )
####


def check() -> None:
    lint()
    typecheck()
    test()
    manual()
####


TASKS: dict[str, Callable[[], None]] = {
    "bootstrap": bootstrap,
    "doctor": doctor,
    "source-pdf": source_pdf,
    "lint": lint,
    "typecheck": typecheck,
    "grammar": grammar,
    "test": test,
    "manual": manual,
    "equation-audit": equation_audit,
    "handoff": handoff,
    "check": check,
    "all": check,
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("task", choices=sorted(TASKS))
    args = parser.parse_args()
    TASKS[args.task]()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
