"""Portable development task runner for the taoryx repository."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import zipfile
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


def docs_doctor() -> None:
    """Check all tools needed to build and render every documentation PDF."""
    run(python_tool("doctor.py", "--docs", "--strict"))
    ####


def lint() -> None:
    run([project_python(), "-m", "ruff", "check", "src", "tests", "tools", "scripts"])
####


def typecheck() -> None:
    run([project_python(), "-m", "mypy"])
####


def test() -> None:
    run([project_python(), "-m", "pytest", "-m", "not slow and not artifact and not simple_aero"])
    ####


def test_all() -> None:
    """Run every pytest category, including opt-in and artifact tests."""
    run([project_python(), "-m", "pytest", "-m", ""])
    ####


def test_category(marker: str) -> None:
    """Run one explicitly selected pytest marker."""
    run([project_python(), "-m", "pytest", "-m", marker])
    ####


def test_slice(paths: tuple[str, ...], expression: str) -> None:
    """Run a named test slice without inheriting the repository fast default."""
    run([project_python(), "-m", "pytest", *paths, "-m", expression, "-o", "addopts="])
    ####


def test_x15_segments() -> None:
    """Run only catalog and isolated X-15 segment-gate tests."""
    test_slice(
        ("tests/unit/test_x15_maneuvers.py", "tests/e2e/test_glider_family_validation.py"),
        "segment",
    )
    ####


def test_x15_catalog() -> None:
    """Run the fast X-15 catalog and source/table traceability checks."""
    test_slice(("tests/unit/test_x15_maneuvers.py",), "segment")
    ####


def test_simple_aero_segments() -> None:
    """Run the isolated SimpleAero segment fixture-quality ladder."""
    test_slice(("tests/e2e/test_simple_aero_segment_validation.py",), "simple_aero and segment")
    ####


def test_vehicle_family(family: str) -> None:
    """Run only one vehicle family, including its opt-in slow tests."""

    test_category(family)
    ####


def test_views() -> None:
    """Print the supported pytest views and cost-category selections."""
    print("Overlapping test views:")
    print("  grammar      parser, lexer, EBNF, corpus, and language validation")
    print("  equations    equation catalog, implementations, provenance, verification")
    print("  algorithms   algorithm catalog, runtime bindings, verification")
    print("  segment      isolated segment contracts and maneuver gates")
    print("  plot         plotting and visualization-only tests")
    print("Cost/output categories:")
    print("  slow         long-running or historical/stress tests")
    print("  artifact     human-readable outputs written below artifacts/")
    print("  simple_aero      SimpleAero problem/segment/trajectory corpus")
    print("  dof-matrix   3-DOF-first/6-DOF-second robustness evidence summary")
    print("  robustness-matrix   bounded paired vehicle verification with convergence and failure reports")
    print("  verification-artifacts   render the full Matplotlib verification and CA-HI bundle")
    print("  showcase-composites   render nominal paired family and all-family overview figures")
    print("  b747-x8-evidence      build a UUID-scoped B747/X8 evidence packet")
    print("  fidelity-packet       build a UUID-scoped four-family fidelity ladder packet")
    print("  audit-fidelity         audit the newest fidelity packet and milestone gates")
    print("  maneuver-matrix   run bound native vehicle maneuvers and write classified evidence")
    print("  slower-tables regenerate B747, Skywalker X8, and Hummingbird research decks")
    print("  generate-problems render metadata-driven native .prb products")
    print("  alpha1-composition-case prove metadata-driven new-case composition")
    print("  alpha1-packet   build the self-contained Alpha 1 evidence packet")
    print("  alpha2-tranches build A2-T1 through A2-T6 evidence")
    print("  alpha2-release  build the self-contained Alpha 2 T7 release packet")
    print("  audit-alpha2    audit the A2-T1 through A2-T7 machine-readable exit signals")
    print("  check-problems verify generated .prb products are current")
    print("  check-vehicles verify vehicle-family contracts and table bindings")
    print("  onboard-vehicles diagnose the complete new-vehicle metadata path")
    print("  compile-segments compile external segment catalogs into native .prb products")
    print("  audit-vehicles verify scoped problem files match registry provenance")
    print("  vehicles   run the complete vehicle registry, generation, and provenance check")
    print("  segment-lint/build/run validate, compile, or execute the declarative segment catalog")
    print("  trim-vehicles solve B747, X8, and Hummingbird trims through the common trim solver")
    print("  control-directions run the signed control-direction convention harness")
    print("  taoryx-extension-pdf build the composite LaTeX and Markdown extension PDF")
    print("  language-reference build the manual-parallel TAORYX language reference PDF")
    print("  docs-doctor diagnose tools needed for every documentation PDF")
    print("  all-pdfs rebuild the historical and successor documentation PDFs")
    print("Commands: test-grammar, test-equations, test-algorithms, test-slow, test-artifacts, test-simple_aero")
    print("Vehicle families: test-b747, test-x8, test-hummingbird, test-x15")
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


def b747_x8_evidence() -> None:
    """Build the clean B747/X8 source-bound evidence packet."""
    run([project_python(), str(TOOLS / "build_b747_x8_evidence_packet.py")])
    ####


def fidelity_packet() -> None:
    """Build the all-family 3-DOF/kinematic/6-DOF evidence packet."""
    run([project_python(), str(TOOLS / "build_fidelity_ladder_packet.py")])
    ####


def audit_fidelity() -> None:
    """Audit the newest all-family fidelity packet and its milestone gates."""
    packets: list[Path] = []
    for candidate in (ROOT / "artifacts" / "verification" / "fidelity_ladder").glob("*.zip"):
        try:
            with zipfile.ZipFile(candidate) as archive:
                manifest = json.loads(archive.read("manifest.json"))
            if len(manifest.get("families", [])) == 4:
                packets.append(candidate)
        except (OSError, KeyError, ValueError, zipfile.BadZipFile):
            continue
    packets.sort(key=lambda path: path.stat().st_mtime)
    if not packets:
        raise FileNotFoundError("no all-family fidelity packet found; run `python tools/dev.py fidelity-packet` first")
    packet = packets[-1]
    run([project_python(), str(TOOLS / "audit_fidelity_packet.py"), str(packet)])
    reproduction_dir = ROOT / "artifacts" / "verification" / "fidelity_reproducibility"
    reproductions = sorted(reproduction_dir.glob("clean-snapshot-*.zip"), key=lambda path: path.stat().st_mtime)
    command = [project_python(), str(TOOLS / "audit_fidelity_milestones.py"), str(packet)]
    if reproductions:
        command.extend(["--reproduced-from", str(reproductions[-1])])
    command.append("--json")
    run(command)
    ####


def audit_alpha1() -> None:
    """Write the current Alpha 1 release-gate status report."""
    run(
        [
            project_python(),
            str(TOOLS / "audit_alpha1_release.py"),
            "--output",
            "artifacts/verification/alpha1/alpha1-status.json",
        ]
    )
    ####


def alpha1_feature_matrix() -> None:
    """Generate the bounded Alpha 1 language feature matrix."""
    run([project_python(), str(TOOLS / "build_alpha1_feature_matrix.py")])
    ####


def alpha1_manual_examples() -> None:
    """Execute and classify the four Alpha 1 manual-example families."""
    run(
        [
            project_python(),
            str(TOOLS / "run_alpha1_manual_examples.py"),
            "--output",
            "artifacts/verification/alpha1/manual_examples/report.json",
        ],
    )
    ####


def alpha1_composition_case() -> None:
    """Resolve and execute the generic Alpha 1 composition proof case."""
    run(
        [
            project_python(),
            str(TOOLS / "run_alpha1_composition_case.py"),
            "--case",
            "verification/alpha1_composition_case.yaml",
            "--report",
            "artifacts/verification/alpha1/composition_case/report.json",
        ],
    )
    ####


def alpha1_packet() -> None:
    """Build the self-contained, path-sanitized Alpha 1 evidence packet."""
    run([project_python(), str(TOOLS / "build_alpha1_packet.py")])
    ####


def alpha2_tranches() -> None:
    """Build the completed Alpha 2 tranche evidence directories."""
    run(tool_script("run_alpha2_tranches.py"))
    ####


def audit_alpha2() -> None:
    """Audit the completed Alpha 2 tranche evidence directories."""
    run(tool_script("audit_alpha2_release.py"))
    ####


def alpha2_release() -> None:
    """Build and audit the self-contained Alpha 2 T7 release packet."""
    run(tool_script("build_alpha2_release.py"))
    ####


def maneuver_matrix() -> None:
    """Run bound native vehicle maneuvers and write classified evidence."""
    run([project_python(), str(TOOLS / "run_maneuver_matrix.py"), "--plots"])
    ####


def import_slower_tables() -> None:
    """Regenerate slower-vehicle research decks from source CSV files."""
    run([project_python(), str(TOOLS / "import_slower_6dof_tables.py")])
    ####


def generate_problem_files() -> None:
    """Generate native problem files from the tracked scenario catalog."""
    run([project_python(), str(TOOLS / "generate_problem_files.py")])
    ####


def check_problem_files() -> None:
    """Verify metadata-driven problem files are current."""
    run([project_python(), str(TOOLS / "generate_problem_files.py"), "--check"])
    ####


def check_vehicle_models() -> None:
    """Verify vehicle-family membership, fields, and table bindings."""
    run([project_python(), str(TOOLS / "generate_problem_files.py"), "--check-models"])
    ####


def onboard_vehicles() -> None:
    """Diagnose every registered vehicle's metadata and source-contract hooks."""
    run([project_python(), str(TOOLS / "validate_vehicle_onboarding.py"), "--vehicle", "all"])
    ####


def check_fidelity_parity_contracts() -> None:
    """Verify shared four-family reduction-parity metadata and hashes."""
    run([project_python(), str(TOOLS / "generate_fidelity_parity_contracts.py"), "--check"])
    ####


def compile_segments() -> None:
    """Compile the external segmentation catalog into native problem files."""
    run([project_python(), str(TOOLS / "compile_segments.py")])
    ####


def lint_segments() -> None:
    """Lint the external segment catalog without writing products."""
    run([project_python(), str(TOOLS / "compile_segments.py"), "lint"])
    ####


def run_segments() -> None:
    """Build and dispatch catalog scenarios through the standard runtime."""
    run([project_python(), str(TOOLS / "compile_segments.py"), "run"])
    ####


def trim_vehicles() -> None:
    """Solve the current source-backed family trims through one utility."""
    for script in ("solve_b747_trim.py", "solve_x8_trim.py", "solve_hummingbird_trim.py"):
        run([project_python(), str(TOOLS / script)])
    ####


def control_directions() -> None:
    """Run the configured signed control-direction probes."""
    run([project_python(), str(TOOLS / "audit_control_directions.py")])
    ####


def run_fidelity_parity() -> None:
    """Execute candidate shared parity windows and classify blockers."""
    run([project_python(), str(TOOLS / "run_fidelity_parity.py")])
    ####


def audit_vehicle_provenance() -> None:
    """Audit vehicle problem headers against the canonical model registry."""
    run([project_python(), str(TOOLS / "audit_vehicle_provenance.py")])
    ####


def vehicle_check() -> None:
    """Run the one-command vehicle authoring and provenance workflow."""
    check_vehicle_models()
    onboard_vehicles()
    audit_vehicle_provenance()
    check_problem_files()
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
    """Validate the bounded application-level corpus without a TAOS executable."""
    run([project_python(), "-m", "pytest", "tests/e2e", "-m", "not runtime and not slow and not artifact and not simple_aero"])
    run([project_python(), "-m", "tools.build_e2e_documented_coverage"])
    ####


def e2e_all() -> None:
    """Validate every application-level case, including opt-in categories."""
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


def successor_guide() -> None:
    """Build the successor-side TAORYX extensions and verification guide."""
    output = ROOT / "output" / "pdf"
    output.mkdir(parents=True, exist_ok=True)
    run(
        [
            "latexmk",
            "-pdf",
            "-interaction=nonstopmode",
            "-halt-on-error",
            f"-output-directory={output}",
            "docs/latex/taoryx_extensions_and_verification.tex",
        ]
    )
    run(tool_script("normalize_pdf.py", str(output / "taoryx_extensions_and_verification.pdf")))
####


def language_reference() -> None:
    """Build the manual-parallel TAORYX language and mathematics reference."""
    output = ROOT / "output" / "pdf"
    output.mkdir(parents=True, exist_ok=True)
    run(
        [
            "latexmk",
            "-pdf",
            "-interaction=nonstopmode",
            "-halt-on-error",
            f"-output-directory={output}",
            "docs/latex/taoryx_language_reference.tex",
        ]
    )
    run(tool_script("normalize_pdf.py", str(output / "taoryx_language_reference.pdf")))
    ####


def _composite_extension_pdf() -> None:
    """Build and normalize the composite successor documentation PDF."""
    run(tool_script("build_taoryx_extension_pdf.py"))
    run(tool_script("normalize_pdf.py", "output/pdf/taoryx_extensions_composite.pdf"))
    ####


def taoryx_extension_pdf() -> None:
    """Build and normalize the composite TAORYX extension reference PDF."""

    successor_guide()
    language_reference()
    _composite_extension_pdf()
    ####


def all_pdfs() -> None:
    """Rebuild the frozen-manual PDF and every successor documentation PDF."""
    manual()
    successor_guide()
    language_reference()
    _composite_extension_pdf()
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


def daveml_readiness() -> None:
    """Validate promoted DAVE-ML family source and graph gates."""

    run(
        tool_script(
            "validate_daveml_family_readiness.py",
            "--readiness",
            "verification/daveml_family_readiness.yaml",
            "--output",
            "verification/daveml_family_readiness.json",
        )
    )
    ####


def daveml_trim() -> None:
    """Generate reproducible source-backed DaveML trim evidence."""

    run(tool_script("validate_daveml_trim.py"))
    ####


def daveml_atmosphere() -> None:
    """Generate reproducible DAVE-ML atmosphere evidence."""

    run(tool_script("validate_daveml_atmosphere.py"))
    ####


def daveml_linearization() -> None:
    """Generate reproducible DAVE-ML dynamics linearization evidence."""

    run(tool_script("validate_daveml_linearization.py"))
    ####


def handoff() -> None:
    equation_audit()
    check()
    # The repository bootstrap may be intentionally offline.  Dependencies
    # are provisioned by `bootstrap`; avoid making the release handoff reach
    # out to PyPI for an isolated build environment.
    run([project_python(), "-m", "build", "--wheel", "--no-isolation", "--outdir", "dist"])
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
    check_vehicle_models()
    onboard_vehicles()
    check_fidelity_parity_contracts()
    compile_segments()
    audit_vehicle_provenance()
    check_problem_files()
    lint()
    typecheck()
    test()
    e2e()
    manual_corpus()
    manual()
    taoryx_extension_pdf()
####


TASKS: dict[str, Callable[[], None]] = {
    "bootstrap": bootstrap,
    "doctor": doctor,
    "docs-doctor": docs_doctor,
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
    "test-segments": lambda: test_category("segment"),
    "test-plots": lambda: test_category("plot"),
    "test-slow": lambda: test_category("slow"),
    "test-simple_aero": lambda: test_category("simple_aero"),
    "test-simple_aero-segments": test_simple_aero_segments,
    "test-b747": lambda: test_vehicle_family("b747"),
    "test-x8": lambda: test_vehicle_family("x8"),
    "test-hummingbird": lambda: test_vehicle_family("hummingbird"),
    "test-x15": lambda: test_vehicle_family("x15"),
    "test-x15-segments": test_x15_segments,
    "test-x15-catalog": test_x15_catalog,
    "test-views": test_views,
    "showcase-california-hawaii": showcase_california_hawaii,
    "dof-matrix": dof_matrix,
    "robustness-matrix": robustness_matrix,
    "verification-artifacts": verification_artifacts,
    "showcase-composites": showcase_composites,
    "b747-x8-evidence": b747_x8_evidence,
    "fidelity-packet": fidelity_packet,
    "audit-fidelity": audit_fidelity,
    "alpha1-feature-matrix": alpha1_feature_matrix,
    "alpha1-manual-examples": alpha1_manual_examples,
    "alpha1-composition-case": alpha1_composition_case,
    "alpha1-packet": alpha1_packet,
    "audit-alpha1": audit_alpha1,
    "alpha2-tranches": alpha2_tranches,
    "alpha2-release": alpha2_release,
    "audit-alpha2": audit_alpha2,
    "maneuver-matrix": maneuver_matrix,
    "slower-tables": import_slower_tables,
    "generate-problems": generate_problem_files,
    "check-problems": check_problem_files,
    "check-vehicles": check_vehicle_models,
    "onboard-vehicles": onboard_vehicles,
    "check-parity": check_fidelity_parity_contracts,
    "run-parity": run_fidelity_parity,
    "compile-segments": compile_segments,
    "segment-lint": lint_segments,
    "segment-build": compile_segments,
    "segment-run": run_segments,
    "trim-vehicles": trim_vehicles,
    "control-directions": control_directions,
    "audit-vehicles": audit_vehicle_provenance,
    "vehicles": vehicle_check,
    "e2e": e2e,
    "e2e-all": e2e_all,
    "manual": manual,
    "successor-guide": successor_guide,
    "language-reference": language_reference,
    "taoryx-extension-pdf": taoryx_extension_pdf,
    "all-pdfs": all_pdfs,
    "equation-audit": equation_audit,
    "daveml-readiness": daveml_readiness,
    "daveml-trim": daveml_trim,
    "daveml-atmosphere": daveml_atmosphere,
    "daveml-linearization": daveml_linearization,
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
