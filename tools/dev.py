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
    run([project_python(), "-m", "pytest", "-m", "not slow and not artifact and not spectre"])
    ####


def test_all() -> None:
    """Run every pytest category, including opt-in and artifact tests."""
    run([project_python(), "-m", "pytest", "-m", ""])
    ####


def test_category(marker: str) -> None:
    """Run one explicitly selected pytest marker."""
    run([project_python(), "-m", "pytest", "-m", marker])
    ####


def test_views() -> None:
    """Print the supported pytest views and cost-category selections."""
    print("Overlapping test views:")
    print("  grammar      parser, lexer, EBNF, corpus, and language validation")
    print("  equations    equation catalog, implementations, provenance, verification")
    print("  algorithms   algorithm catalog, runtime bindings, verification")
    print("Cost/output categories:")
    print("  slow         long-running or historical/stress tests")
    print("  artifact     human-readable outputs written below artifacts/")
    print("  spectre      Spectre problem/segment/trajectory corpus")
    print("  dof-matrix   3-DOF-first/6-DOF-second robustness evidence summary")
    print("  robustness-matrix   bounded paired vehicle verification with convergence and failure reports")
    print("  verification-artifacts   render the full Matplotlib verification and CA-HI bundle")
    print("  showcase-composites   render nominal paired family and all-family overview figures")
    print("  maneuver-matrix   run bound native vehicle maneuvers and write classified evidence")
    print("  slower-tables regenerate B747, Skywalker X8, and Hummingbird research decks")
    print("Commands: test-grammar, test-equations, test-algorithms, test-slow, test-artifacts, test-spectre")
    ####


def showcase_california_hawaii() -> None:
    """Regenerate the California-to-Hawaii JSON, SQLite, text, and PNG artifact."""
    run(
        [
            project_python(),
            str(ROOT / "examples/showcases/california_to_hawaii/run_showcase.py"),
            "--output-dir",
            "artifacts/showcases/california_to_hawaii",
        ]
    )
    ####


def dof_matrix() -> None:
    """Generate machine-readable 3-DOF/6-DOF robustness evidence."""
    run([project_python(), str(TOOLS / "run_dof_matrix.py")])
    ####


def robustness_matrix() -> None:
    """Run the bounded paired vehicle verification matrix and write reports."""
    run([project_python(), str(TOOLS / "run_robustness_matrix.py")])
    ####


def verification_artifacts() -> None:
    """Render the full verification and CA-HI Matplotlib artifact bundle."""
    mpl_config = ROOT / "artifacts" / ".mplconfig"
    mpl_config.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(mpl_config))
    # The matrix report may deliberately contain envelope rejections.  Keep
    # producing the remaining artifacts so those classified smoke-test
    # outcomes do not prevent the overview bundle from being inspected.
    print("+", project_python(), TOOLS / "run_robustness_matrix.py", "--plots")
    matrix_result = subprocess.run(
        [project_python(), str(TOOLS / "run_robustness_matrix.py"), "--plots"],
        cwd=ROOT,
        check=False,
    )
    if matrix_result.returncode:
        print(f"robustness matrix retained classified failures (exit {matrix_result.returncode}); continuing artifact generation")
    print("+", project_python(), TOOLS / "run_dof_matrix.py", "--plots")
    dof_result = subprocess.run(
        [project_python(), str(TOOLS / "run_dof_matrix.py"), "--plots"],
        cwd=ROOT,
        check=False,
    )
    if dof_result.returncode:
        print(f"DOF matrix retained classified failures (exit {dof_result.returncode}); continuing artifact generation")
    showcase_result = subprocess.run(
        [
            project_python(),
            str(ROOT / "examples/showcases/california_to_hawaii/run_showcase.py"),
            "--output-dir",
            "artifacts/showcases/california_to_hawaii",
        ],
        cwd=ROOT,
        check=False,
    )
    if showcase_result.returncode:
        print(f"CA-HI showcase retained its reported status (exit {showcase_result.returncode}); continuing artifact generation")
    run([project_python(), str(TOOLS / "render_showcase_composites.py")])
    ####


def showcase_composites() -> None:
    """Render nominal paired trajectory composites for the slower vehicles."""
    run([project_python(), str(TOOLS / "render_showcase_composites.py")])
    ####


def maneuver_matrix() -> None:
    """Run bound native vehicle maneuvers and write classified evidence."""
    run([project_python(), str(TOOLS / "run_maneuver_matrix.py"), "--plots"])
    ####


def import_slower_tables() -> None:
    """Regenerate slower-vehicle research decks from source CSV files."""
    run([project_python(), str(TOOLS / "import_slower_6dof_tables.py")])
    ####


def grammar() -> None:
    run([project_python(), "-m", "pytest", "tests/parser"])
    run([project_python(), str(TOOLS / "check_taos_fixtures.py")])
    manual_corpus()
####


def manual_corpus() -> None:
    """Verify the tracked manual corpus and its parser evidence."""
    run([project_python(), str(TOOLS / "check_manual_snippet_corpus.py")])
####


def e2e() -> None:
    """Validate the checked-in application-level corpus without a TAOS executable."""
    run([project_python(), "-m", "pytest", "tests/e2e", "-m", "not runtime"])
    run([project_python(), "-m", "tools.build_e2e_documented_coverage"])
####


def legacy_audit() -> None:
    run(tool_script("audit_legacy_inbox.py", "--verify-fixtures"))
####


def legacy_close_check() -> None:
    run(tool_script("check_legacy_closure.py"))
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
    e2e()
    manual_corpus()
    manual()
####


TASKS: dict[str, Callable[[], None]] = {
    "bootstrap": bootstrap,
    "doctor": doctor,
    "source-pdf": source_pdf,
    "lint": lint,
    "typecheck": typecheck,
    "grammar": grammar,
    "legacy-audit": legacy_audit,
    "legacy-close-check": legacy_close_check,
    "test": test,
    "test-all": test_all,
    "test-artifacts": lambda: test_category("artifact"),
    "test-algorithms": lambda: test_category("algorithms"),
    "test-equations": lambda: test_category("equations"),
    "test-grammar": lambda: test_category("grammar"),
    "test-slow": lambda: test_category("slow"),
    "test-spectre": lambda: test_category("spectre"),
    "test-views": test_views,
    "showcase-california-hawaii": showcase_california_hawaii,
    "dof-matrix": dof_matrix,
    "robustness-matrix": robustness_matrix,
    "verification-artifacts": verification_artifacts,
    "showcase-composites": showcase_composites,
    "maneuver-matrix": maneuver_matrix,
    "slower-tables": import_slower_tables,
    "e2e": e2e,
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
