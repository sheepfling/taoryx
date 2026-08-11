"""Portable development task runner for the taoryx repository."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import zipfile
from collections.abc import Iterable
from pathlib import Path
from typing import Callable

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
TOOLS = ROOT / "tools"
VENV_PYTHON = ROOT / ".venv" / ("Scripts/python.exe" if sys.platform == "win32" else "bin/python")
SOURCE_ROOTS = (
    ROOT / "src",
    ROOT / "packages" / "taoryx-daveml" / "src",
    ROOT / "packages" / "taoryx-simple-aero" / "src",
    ROOT / "packages" / "taoryx-reference-models" / "src",
    ROOT / "packages" / "taoryx-reachability" / "src",
)
QUICK_TEST_PATHS = (
    "tests/parser/test_lexical.py",
    "tests/parser/test_expressions.py",
    "tests/unit/test_simulation_runtime_contracts.py",
    "tests/unit/test_simulation_runtime_quality.py",
    "tests/unit/test_runtime_algorithms.py",
    "tests/unit/test_runtime_cli.py",
    "tests/unit/test_interactive_runtime.py",
    "tests/unit/test_composition_policy.py",
    "tests/unit/test_mission_composition_completion.py",
    "tests/unit/test_model_authoring.py",
    "tests/unit/test_tooling_entrypoints.py",
)
VEHICLE_VERTICAL_TEST_PATHS: dict[str, tuple[str, ...]] = {
    # A vertical slice is deliberately one family rather than every test with
    # that family marker.  It proves the public Composition path from the
    # plug-in advertisement through execution and controller automation.
    "f16_s119": ("tests/unit/test_f16_vehicle_vertical.py",),
    "a320_openap_3dof": ("tests/unit/test_a320_vehicle_vertical.py",),
    "hummingbird": ("tests/unit/test_hummingbird_vehicle_vertical.py",),
    "x15": ("tests/unit/test_x15_vehicle_vertical.py",),
    "hl20_mod_k": ("tests/unit/test_hl20_vehicle_vertical.py",),
    "reference_nesc_two_stage_rocket": ("tests/unit/test_nesc_vehicle_vertical.py",),
    "tumbling_body": ("tests/unit/test_tumbling_body_vehicle_vertical.py",),
    "skywalker_x8": ("tests/unit/test_x8_vehicle_vertical.py",),
    "b747": ("tests/unit/test_b747_vehicle_vertical.py",),
    "simple_aero": (
        "tests/unit/test_simple_aero_vehicle_vertical.py",
        "tests/unit/test_mission_workflow_endpoint.py::test_workflow_endpoints_compile_and_preflight_through_the_installed_provider[simple-aero-fixed-ld-batch]",
        "tests/unit/test_mission_workflow_endpoint.py::test_workflow_endpoints_emit_their_declared_normalized_result_surface[simple-aero-fixed-ld-batch]",
    ),
    "dual_launch_glider": (
        "tests/unit/test_dual_launch_vehicle_vertical.py",
        "tests/unit/test_mission_workflow_endpoint.py::test_workflow_endpoints_compile_and_preflight_through_the_installed_provider[dual-launch-attached-booster-batch]",
        "tests/unit/test_mission_workflow_endpoint.py::test_workflow_endpoints_emit_their_declared_normalized_result_surface[dual-launch-attached-booster-batch]",
    ),
}


def project_python() -> str:
    if VENV_PYTHON.exists():
        return str(VENV_PYTHON)
    ####
    return sys.executable


####


def run(command: list[str]) -> None:
    print("+", " ".join(command))
    environment = os.environ.copy()
    existing_pythonpath = environment.get("PYTHONPATH")
    environment["PYTHONPATH"] = os.pathsep.join(path for path in (*(str(item) for item in SOURCE_ROOTS), existing_pythonpath) if path)
    subprocess.run(command, cwd=ROOT, check=True, env=environment)
    ####


####


def python_tool(script: str, *args: str) -> list[str]:
    return [project_python(), str(SCRIPTS / script), *args]


####


def tool_script(script: str, *args: str) -> list[str]:
    """Build a module invocation for a tool in the repository package."""

    module = Path(script).stem
    return [project_python(), "-m", f"tools.{module}", *args]


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


def installation_check() -> None:
    """Require all official editable distributions and installed entry points."""

    run(
        [
            project_python(),
            "-m",
            "taoryx.runtime.cli",
            "plugins",
            "check",
            "--profile",
            "full",
        ]
    )
    ####


def docs_doctor() -> None:
    """Check all tools needed to build and render every documentation PDF."""
    run(python_tool("doctor.py", "--docs", "--strict"))
    ####


def lint() -> None:
    run([project_python(), "-m", "ruff", "check", "src", "packages", "tests", "tools", "scripts"])


####


def typecheck() -> None:
    run([project_python(), "-m", "mypy"])


####


def test() -> None:
    run([project_python(), "-m", "pytest", "-m", "not slow and not artifact and not simple_aero", "--basetemp", ".pytest-fast"])
    ####


def test_quick() -> None:
    """Run the curated inner-loop smoke and contract tests."""
    run(
        [
            project_python(),
            "-m",
            "pytest",
            *QUICK_TEST_PATHS,
            "-q",
            "-x",
            "--basetemp",
            ".pytest-quick",
        ]
    )
    ####


def _git_changed_paths() -> tuple[str, ...]:
    """Return staged, unstaged, and untracked repository paths."""
    commands = (
        ("git", "diff", "--name-only", "--diff-filter=ACMR", "HEAD"),
        ("git", "ls-files", "--others", "--exclude-standard"),
    )
    paths: set[str] = set()
    for command in commands:
        try:
            output = subprocess.check_output(command, cwd=ROOT, text=True)
        except (OSError, subprocess.CalledProcessError):
            return ()
        paths.update(line.strip() for line in output.splitlines() if line.strip())
    return tuple(sorted(paths))
    ####


def _changed_test_paths(changed_paths: Iterable[str]) -> tuple[str, ...]:
    """Map changed source/test files to a small, conservative pytest selection."""
    changed = tuple(Path(path).as_posix() for path in changed_paths)
    test_files = {path.relative_to(ROOT).as_posix() for path in (ROOT / "tests").rglob("test_*.py") if path.is_file()}
    selected: set[str] = set()
    source_paths = tuple(path for path in changed if path.startswith("src/taoryx/") and path.endswith(".py"))
    for normalized in changed:
        if normalized in test_files:
            selected.add(normalized)
        if normalized == "tests/conftest.py" or normalized == "pyproject.toml":
            return QUICK_TEST_PATHS
        if normalized == "tools/dev.py":
            selected.add("tests/unit/test_tooling_entrypoints.py")
        if normalized.startswith("src/taoryx/") and normalized.endswith(".py"):
            stem = Path(normalized).stem
            direct = f"tests/unit/test_{stem}.py"
            if direct in test_files:
                selected.add(direct)
    for source in source_paths:
        module = f"taoryx.{source.removeprefix('src/taoryx/').removesuffix('.py').replace('/', '.')}"
        for test_path in test_files:
            try:
                contents = (ROOT / test_path).read_text(encoding="utf-8")
            except OSError:
                continue
            if module in contents:
                selected.add(test_path)
    return tuple(sorted(selected)) or QUICK_TEST_PATHS
    ####


def test_changed() -> None:
    """Run tests associated with current Git changes, falling back to smoke tests."""
    changed_paths = _git_changed_paths()
    selected = _changed_test_paths(changed_paths)
    print("Changed-test selection:")
    for path in selected:
        print(f"  {path}")
    if not changed_paths:
        print("  (no Git changes detected; using the quick suite)")
    run(
        [
            project_python(),
            "-m",
            "pytest",
            *selected,
            "-q",
            "-x",
            "-o",
            "addopts=",
            "--strict-markers",
            "-m",
            "not slow and not artifact and not simple_aero",
            "--basetemp",
            ".pytest-changed",
        ]
    )
    ####


def _python_module_available(module: str) -> bool:
    """Return whether the project interpreter can import an optional module."""
    result = subprocess.run(
        [project_python(), "-c", f"import {module}"],
        cwd=ROOT,
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return result.returncode == 0
    ####


def test_parallel() -> None:
    """Run the default fast suite across workers when pytest-xdist is installed."""
    if not _python_module_available("xdist"):
        raise SystemExit("pytest-xdist is not installed; run `python -m pip install -e '.[dev]'` first")
    run(
        [
            project_python(),
            "-m",
            "pytest",
            "-n",
            "auto",
            "--dist",
            "loadfile",
            "-m",
            "not slow and not artifact and not simple_aero",
            "--basetemp",
            ".pytest-parallel",
        ]
    )
    ####


def test_all() -> None:
    """Run every pytest category, including opt-in and artifact tests."""
    run([project_python(), "-m", "pytest", "-m", "", "--basetemp", ".pytest-all"])
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


def test_vehicle_vertical(family: str) -> None:
    """Run the fast end-to-end Composition slice for one runnable vehicle."""

    try:
        paths = VEHICLE_VERTICAL_TEST_PATHS[family]
    except KeyError as error:
        available = ", ".join(sorted(VEHICLE_VERTICAL_TEST_PATHS))
        raise SystemExit(f"no focused vehicle slice is defined for {family!r}; available: {available}") from error
    marker_expression = "not slow and not artifact"
    if family != "simple_aero":
        marker_expression += " and not simple_aero"
    run(
        [
            project_python(),
            "-m",
            "pytest",
            *paths,
            "-q",
            "-x",
            "-o",
            "addopts=",
            "--strict-markers",
            "-m",
            marker_expression,
            "--basetemp",
            f".pytest-vehicle-{family}",
        ]
    )
    ####


def test_vehicle_catalogue() -> None:
    """Check that catalogue-owned controller campaigns have vertical coverage."""

    run(
        [
            project_python(),
            "-m",
            "pytest",
            "tests/unit/test_vehicle_catalogue_contract.py",
            "-q",
            "-x",
            "-o",
            "addopts=",
            "--strict-markers",
            "--basetemp",
            ".pytest-vehicle-catalogue",
        ]
    )
    ####


def vehicle_catalogue() -> None:
    """Validate runnable catalogue contracts without promoting planned tiers."""

    test_vehicle_catalogue()
    check_vehicle_models()
    check_vehicle_interfaces()
    onboard_vehicles()
    ####


def test_views() -> None:
    """Print the supported pytest views and cost-category selections."""
    print("Inner-loop commands:")
    print("  test-quick   curated smoke and contract suite")
    print("  test-changed tests associated with current Git changes")
    print("  test-parallel default fast suite with pytest-xdist when installed")
    print(f"  test-vehicle <family> runnable one-family Composition vertical slice ({', '.join(sorted(VEHICLE_VERTICAL_TEST_PATHS))})")
    print("  test-vehicle-catalogue controller-campaign to vertical-slice catalogue contract")
    print("  test-f16     F-16 S.119 vertical-slice convenience alias")
    print("  test-a320    A320 pseudo-6DOF vertical-slice convenience alias")
    print("  test-hummingbird-vertical Hummingbird pseudo-6DOF vertical-slice convenience alias")
    print("  test-x15-vertical X-15 direct-wrench vertical-slice convenience alias")
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
    print("  check-vehicle-interfaces verify every declared semantic interface and episode binding")
    print("  check-vehicle-maturity  verify maturity records retain public batch witnesses")
    print("  onboard-vehicles diagnose the complete new-vehicle metadata path")
    print("  fidelity-readiness check declared data for all four fidelity tiers")
    print("  pseudo6dof-profiles validate the Alpha 3 pseudo-6DOF family profile catalog")
    print("  horizontal-fidelity validate the canonical four-tier family registry")
    print("  family-adapter-registry run executable family-adapter witnesses and planned bindings")
    print("  alpha3-readiness regenerate paired-fidelity, cross-fidelity, and R1 readiness indexes")
    print("  showcase-artifact-boundary validate resolved fidelity/control metadata in generated showcase runs")
    print("  showcase-catalog-references validate aggregate catalog child manifests")
    print("  alpha3-showcase-catalog verify all nine family recipes, evidence pairs, and canonical boards")
    print("  hummingbird-directional validate the Hummingbird altitude/yaw/body-direction mission witness")
    print("  hummingbird-native-horizontal validate the Hummingbird native source-plant horizontal witness")
    print("  hummingbird-native-vertical validate the Hummingbird native source-plant vertical-force witness")
    print("  airbreathing-r1 run the fixed initial-state R1 matrix for X8 and B747")
    print("  a320-r1 run the fixed calibrated operating-point R1 matrix for A320 reductions")
    print("  f16-r1 run the fixed reduced-tier R1 matrix for the reference F-16")
    print("  hl20-r1 run the fixed passive/open-loop R1 matrix for HL-20 reductions")
    print("  hummingbird-r1 run the fixed aggregate-thrust R1 matrix for Hummingbird")
    print("  x15-r1 run the fixed staged-surrogate R1 matrix for X-15 reductions")
    print("  nesc-r1 run the fixed source-replay/response-law R1 matrix for NESC")
    print("  b747-physical-r1 run the fixed local B747 surface-allocation R1 matrix")
    print("  f16-physical-r1 record the fixed local F-16 physical-effector R1 boundary matrix")
    print("  f16-physical-schedule validate source-effector F-16 nodes across the local schedule")
    print("  f16-physical-schedule-envelope record F-16 source-effector authority boundaries")
    print("  f16-physical-schedule-interior qualify the conservative schedule-wide F-16 interior")
    print("  x8-physical-mapping record fail-closed X8 left/right elevon mapping evidence")
    print("  b747-physical-schedule validate source-effector B747 nodes across CR-2144 conditions")
    print("  hummingbird-physical-r1 run the fixed local Hummingbird rotor-allocation R1 matrix")
    print("  tumbling-r1 index the fixed passive tumbling-body matrix")
    print("  reference-tuning verify the four-family generic controller-design report")
    print("  compile-segments compile external segment catalogs into native .prb products")
    print("  powered-fixed-wing-mission-proposals verify capability-scaled first-mission planning profiles")
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


def showcase_hl20_low_fidelity() -> None:
    """Run the HL-20 four-tier release witness and its showcase composites."""
    run(
        [
            project_python(),
            str(ROOT / "examples/showcases/hl20_california_to_hawaii/run_low_fidelity.py"),
            "--output-dir",
            "artifacts/showcases/hl20_california_to_hawaii/low_fidelity",
        ]
    )
    ####


def showcase_hl20_composites() -> None:
    """Render the reproducible HL-20 boost/glide and spent-stage boards."""

    showcase_hl20_low_fidelity()
    ####


def showcase_hl20_source_composites() -> None:
    """Render HL-20 composites using the pinned source graph and actuators."""
    run(
        [
            project_python(),
            str(ROOT / "examples/showcases/hl20_california_to_hawaii/run_low_fidelity.py"),
            "--source-bound",
            "--output-dir",
            "artifacts/showcases/hl20_california_to_hawaii/source_bound",
        ]
    )
    ####


def qualify_hl20() -> None:
    """Regenerate the HL-20 bundle and qualify it through HL20-G6."""

    run(tool_script("validate_hl20_qualification.py"))
    ####


def qualify_hl20_source_reachability() -> None:
    """Qualify source-aero coupling and report fail-closed tier gaps."""

    run(tool_script("validate_hl20_source_reachability.py"))
    ####


def qualify_hl20_terminal_contract() -> None:
    """Execute the explicit CA-HI target, radius, speed, and timeout contract."""
    run(tool_script("validate_hl20_terminal_contract.py"))
    ####


def qualify_hl20_source_robustness() -> None:
    """Generate HL-20 timestep, source-boundary, and timeout-rerun evidence."""
    run(tool_script("validate_hl20_source_robustness.py"))
    ####


def qualify_passive_deployment() -> None:
    """Qualify passive sphere, cylinder, cone, and ellipsoid deployments."""
    run(tool_script("validate_passive_deployment.py"))
    ####


def dof_matrix() -> None:
    """Generate machine-readable 3-DOF/6-DOF robustness evidence."""
    run(tool_script("run_dof_matrix.py"))
    ####


def robustness_matrix() -> None:
    """Run the bounded paired vehicle verification matrix and write reports."""
    run(tool_script("run_robustness_matrix.py"))
    ####


def verification_artifacts() -> None:
    """Render the full verification and CA-HI Matplotlib artifact bundle."""
    mpl_config = ROOT / "artifacts" / ".mplconfig"
    mpl_config.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(mpl_config))
    # The matrix report may deliberately contain envelope rejections.  Keep
    # producing the remaining artifacts so those classified smoke-test
    # outcomes do not prevent the overview bundle from being inspected.
    matrix_command = tool_script("run_robustness_matrix.py", "--plots")
    print("+", " ".join(matrix_command))
    matrix_result = subprocess.run(matrix_command, cwd=ROOT, check=False)
    if matrix_result.returncode:
        print(f"robustness matrix retained classified failures (exit {matrix_result.returncode}); continuing artifact generation")
    dof_command = tool_script("run_dof_matrix.py", "--plots")
    print("+", " ".join(dof_command))
    dof_result = subprocess.run(dof_command, cwd=ROOT, check=False)
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
    run(tool_script("render_showcase_composites.py"))
    ####


def showcase_composites() -> None:
    """Render nominal paired trajectory composites for the slower vehicles."""
    run(tool_script("render_showcase_composites.py"))


def b747_x8_evidence() -> None:
    """Build the clean B747/X8 source-bound evidence packet."""
    run(tool_script("build_b747_x8_evidence_packet.py"))
    ####


def fidelity_packet() -> None:
    """Build the all-family 3-DOF/kinematic/6-DOF evidence packet."""
    run(tool_script("build_fidelity_ladder_packet.py"))
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
    run(tool_script("audit_fidelity_packet.py", str(packet)))
    reproduction_dir = ROOT / "artifacts" / "verification" / "fidelity_reproducibility"
    reproductions = sorted(reproduction_dir.glob("clean-snapshot-*.zip"), key=lambda path: path.stat().st_mtime)
    command = tool_script("audit_fidelity_milestones.py", str(packet))
    if reproductions:
        command.extend(["--reproduced-from", str(reproductions[-1])])
    command.append("--json")
    run(command)
    ####


def audit_alpha1() -> None:
    """Write the current Alpha 1 release-gate status report."""
    run(
        [
            *tool_script("audit_alpha1_release.py"),
            "--output",
            "artifacts/verification/alpha1/alpha1-status.json",
        ]
    )
    ####


def alpha1_feature_matrix() -> None:
    """Generate the bounded Alpha 1 language feature matrix."""
    run(tool_script("build_alpha1_feature_matrix.py"))
    ####


def alpha1_manual_examples() -> None:
    """Execute and classify the four Alpha 1 manual-example families."""
    run(
        [
            *tool_script("run_alpha1_manual_examples.py"),
            "--output",
            "artifacts/verification/alpha1/manual_examples/report.json",
        ],
    )
    ####


def alpha1_composition_case() -> None:
    """Resolve and execute the generic Alpha 1 composition proof case."""
    run(
        [
            *tool_script("run_alpha1_composition_case.py"),
            "--case",
            "verification/alpha1_composition_case.yaml",
            "--report",
            "artifacts/verification/alpha1/composition_case/report.json",
        ],
    )
    ####


def alpha1_packet() -> None:
    """Build the self-contained, path-sanitized Alpha 1 evidence packet."""
    run(tool_script("build_alpha1_packet.py"))
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
    run(tool_script("run_maneuver_matrix.py", "--plots"))
    ####


def import_slower_tables() -> None:
    """Regenerate slower-vehicle research decks from source CSV files."""
    run(tool_script("import_slower_6dof_tables.py"))
    ####


def generate_problem_files() -> None:
    """Generate native problem files from the tracked scenario catalog."""
    run(tool_script("generate_problem_files.py"))
    ####


def check_problem_files() -> None:
    """Verify metadata-driven problem files are current."""
    run(tool_script("generate_problem_files.py", "--check"))
    ####


def check_vehicle_models() -> None:
    """Verify vehicle-family membership, fields, and table bindings."""
    run(tool_script("generate_problem_files.py", "--check-models"))
    ####


def check_supported_reference_families() -> None:
    """Verify source-grounded family bindings and their claim boundaries."""
    run(tool_script("validate_supported_reference_families.py"))
    ####


def check_vehicle_maturity_registry() -> None:
    """Verify composition-managed maturity claims retain public batch witnesses."""

    run(tool_script("validate_vehicle_maturity_registry.py"))
    ####


def check_vehicle_interfaces() -> None:
    """Validate all resolved semantic interfaces against declared runtime bindings."""

    run(tool_script("validate_vehicle_interface_catalog.py", "--check"))
    ####


def check_vehicle_execution_witnesses() -> None:
    """Validate endpoint requests and execute registered M3 parity replays."""

    run(tool_script("validate_vehicle_execution_witnesses.py", "--execute-parity"))
    ####


def check_mission_composition_completion() -> None:
    """Verify the production registry, common executors, and generated coverage matrix."""

    run(tool_script("build_mission_composition_completion.py", "--check"))
    ####


def check_simulation_runtime_quality() -> None:
    """Build the Simulation Runtime M4 report and execute the selected fixed-step cases."""

    run(
        tool_script(
            "validate_simulation_runtime_quality.py",
            "--execute",
            "--scenario",
            "two-stage-ballistic",
            "--scenario",
            "two-stage-demo",
            "--scenario",
            "interactive-california-hawaii",
            "--output",
            "artifacts/verification/simulation_runtime_quality/report.json",
        )
    )
    ####


def onboard_vehicles() -> None:
    """Diagnose every registered vehicle's metadata and source-contract hooks."""
    run(tool_script("validate_vehicle_onboarding.py", "--vehicle", "all"))
    ####


def fidelity_readiness() -> None:
    """Check declared data readiness for every registered fidelity tier."""
    run(tool_script("validate_fidelity_readiness.py", "--vehicle", "all"))
    ####


def pseudo6dof_profiles() -> None:
    """Validate the Alpha 3 pseudo-6DOF family profile catalog."""
    run(tool_script("validate_pseudo6dof_profiles.py", "--smoke"))
    ####


def horizontal_fidelity() -> None:
    """Validate the shared four-tier family integration registry."""
    run(tool_script("validate_horizontal_fidelity.py", "--check"))
    ####


def unified_family_manifest() -> None:
    """Validate the joined horizontal, profile, source, and vehicle manifests."""
    run(tool_script("validate_unified_family_manifest.py", "--check"))
    ####


def horizontal_readiness() -> None:
    """Regenerate the joined all-family, all-tier readiness matrix."""
    run(tool_script("validate_horizontal_readiness.py", "--check"))
    ####


def family_strategy_worklists() -> None:
    """Compile generic topology-specific integration and tuning worklists."""

    run(tool_script("compile_family_strategy_worklists.py", "--check"))
    ####


def family_adapter_registry() -> None:
    """Run executable family-adapter witnesses and planned-binding checks."""
    run(tool_script("validate_family_adapter_registry.py", "--check"))
    ####


def alpha3_readiness() -> None:
    """Regenerate the Alpha 3 paired-fidelity and readiness indexes."""
    run(tool_script("validate_alpha3_fidelity_ladder.py"))
    run(tool_script("build_alpha3_cross_fidelity_reports.py"))
    run(tool_script("qualify_f16_physical_schedule_interior.py"))
    run(tool_script("validate_hummingbird_directional_mission.py"))
    run(tool_script("validate_hummingbird_native_horizontal_mission.py"))
    run(tool_script("validate_hummingbird_native_vertical_mission.py"))
    run(tool_script("build_alpha3_readiness_report.py"))
    ####


def alpha3_showcase_catalog() -> None:
    """Verify the complete Alpha 3 family showcase artifact catalog."""
    run(tool_script("build_alpha3_direct_wrench_contract.py"))
    run(tool_script("validate_alpha3_showcase_catalog.py"))
    ####


def hummingbird_directional() -> None:
    """Validate the Hummingbird directional-translation mission witness."""
    run(tool_script("validate_hummingbird_directional_mission.py"))
    ####


def hummingbird_native_horizontal() -> None:
    """Validate the native Hummingbird source-plant horizontal witness."""
    run(tool_script("validate_hummingbird_native_horizontal_mission.py"))
    ####


def hummingbird_native_vertical() -> None:
    """Validate the native Hummingbird source-plant vertical-force witness."""
    run(tool_script("validate_hummingbird_native_vertical_mission.py"))
    ####


def f16_physical_schedule_interior() -> None:
    """Qualify the conservative schedule-wide F-16 physical interior."""
    run(tool_script("qualify_f16_physical_schedule_interior.py"))
    ####


def airbreathing_r1_matrix() -> None:
    """Run the fixed initial-state R1 matrix for X8 and B747."""
    run(tool_script("validate_airbreathing_r1_matrix.py"))
    ####


def a320_r1_matrix() -> None:
    """Run the fixed calibrated operating-point R1 matrix for A320 reductions."""
    run(tool_script("validate_a320_r1_matrix.py"))
    ####


def f16_r1_matrix() -> None:
    """Run the fixed reduced-tier R1 matrix for the reference F-16."""
    run(tool_script("validate_f16_r1_matrix.py"))
    ####


def hl20_r1_matrix() -> None:
    """Run the fixed passive/open-loop R1 matrix for HL-20 reductions."""
    run(tool_script("validate_hl20_r1_matrix.py"))
    ####


def hummingbird_r1_matrix() -> None:
    """Run the fixed aggregate-thrust R1 matrix for Hummingbird."""
    run(tool_script("validate_hummingbird_r1_matrix.py"))
    ####


def x15_r1_matrix() -> None:
    """Run the fixed staged-surrogate R1 matrix for X-15 reductions."""
    run(tool_script("validate_x15_r1_matrix.py"))
    ####


def nesc_r1_matrix() -> None:
    """Run the fixed source-replay/response-law R1 matrix for NESC."""
    run(tool_script("validate_nesc_r1_matrix.py"))
    ####


def b747_physical_r1_matrix() -> None:
    """Run the fixed local B747 physical-surface R1 matrix."""
    run(tool_script("validate_b747_physical_r1_matrix.py"))
    ####


def hummingbird_physical_r1_matrix() -> None:
    """Run the fixed local Hummingbird individual-rotor R1 matrix."""
    run(tool_script("validate_hummingbird_physical_r1_matrix.py"))
    ####


def tumbling_r1_matrix() -> None:
    """Index the fixed passive tumbling-body matrix."""
    run(tool_script("validate_tumbling_body_r1_matrix.py"))
    ####


def integration_readiness() -> None:
    """Check source-family intake and fidelity readiness before runtime probes."""
    run(tool_script("validate_vehicle_integration_readiness.py", "--family", "all"))
    ####


def integration_pipeline() -> None:
    """Run staged provider-neutral source-family integration diagnostics."""
    run(tool_script("validate_vehicle_integration_pipeline.py", "--family", "all", "--allow-blocked"))
    ####


def integration_pilots() -> None:
    """Run the A320 collection and NESC reference integration pilots."""
    run(
        tool_script(
            "validate_a320_racetrack.py",
            "--mode",
            "all",
            "--output-dir",
            "verification/a320_racetrack",
        )
    )
    run(
        tool_script(
            "validate_vehicle_integration_pilots.py",
            "--family",
            "all",
            "--allow-blocked",
            "--json",
            "verification/vehicle_integration_pilots.json",
            "--packet-dir",
            "verification/integration-packets",
        )
    )
    ####


def effectivity_preflight() -> None:
    """Run numeric effectivity and downstream allocation diagnostics."""
    run(
        tool_script(
            "validate_vehicle_effectivity_preflight.py",
            "--family",
            "all",
            "--allow-blocked",
            "--json",
            "verification/vehicle_effectivity_preflight.json",
        )
    )
    ####


def trim_orchestration() -> None:
    """Validate pilot trim recipes and emit adapter-ready worklists."""
    run(
        tool_script(
            "validate_vehicle_trim_orchestration.py",
            "--family",
            "all",
            "--json",
            "verification/vehicle_trim_orchestration.json",
        )
    )
    ####


def controller_mission_preflight() -> None:
    """Run controller and mission preflight for the conformance pilots."""
    run(
        tool_script(
            "validate_vehicle_controller_mission_preflight.py",
            "--family",
            "all",
            "--allow-blocked",
            "--json",
            "verification/vehicle_controller_mission_preflight.json",
        )
    )
    ####


def solve_trim_evidence() -> None:
    """Solve the conformance-pilot trim worklists through family adapters."""
    run(tool_script("solve_vehicle_trim_evidence.py"))
    ####


def check_fidelity_parity_contracts() -> None:
    """Verify shared four-family reduction-parity metadata and hashes."""
    run(tool_script("generate_fidelity_parity_contracts.py", "--check"))
    ####


def check_reference_tuning() -> None:
    """Verify the four-family generic tuning report is reproducible."""
    run([project_python(), str(TOOLS / "tune_reference_aircraft.py"), "--check"])
    ####


def reduced_tuning_campaigns() -> None:
    """Verify cross-topology reduced-order tuning campaigns without manual gains."""
    run(tool_script("validate_reduced_tuning_campaigns.py", "--check"))
    ####


def powered_fixed_wing_mission_proposals() -> None:
    """Verify capability-scaled powered-fixed-wing mission planning evidence."""
    run(tool_script("validate_powered_fixed_wing_mission_proposals.py", "--check"))
    ####


def compile_segments() -> None:
    """Compile the external segmentation catalog into native problem files."""
    run(tool_script("compile_segments.py"))
    ####


def lint_segments() -> None:
    """Lint the external segment catalog without writing products."""
    run(tool_script("compile_segments.py", "lint"))
    ####


def run_segments() -> None:
    """Build and dispatch catalog scenarios through the standard runtime."""
    run(tool_script("compile_segments.py", "run"))
    ####


def trim_vehicles() -> None:
    """Solve the current source-backed family trims through one utility."""
    for script in ("solve_b747_trim.py", "solve_x8_trim.py", "solve_hummingbird_trim.py"):
        run(tool_script(script))
    ####


def control_directions() -> None:
    """Run the configured signed control-direction probes."""
    run(tool_script("audit_control_directions.py"))
    ####


def run_fidelity_parity() -> None:
    """Execute candidate shared parity windows and classify blockers."""
    run(tool_script("run_fidelity_parity.py"))
    ####


def audit_vehicle_provenance() -> None:
    """Audit vehicle problem headers against the canonical model registry."""
    run(tool_script("audit_vehicle_provenance.py"))
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
    run(tool_script("check_taos_fixtures.py"))
    manual_corpus()


####


def manual_corpus() -> None:
    """Verify the tracked manual corpus and its parser evidence."""
    run(tool_script("check_manual_snippet_corpus.py"))


####


def e2e() -> None:
    """Validate the bounded application-level corpus without a TAOS executable."""
    run([project_python(), "-m", "pytest", "tests/e2e", "-m", "not runtime and not slow and not artifact and not simple_aero", "--basetemp", ".pytest-e2e"])
    run([project_python(), "-m", "tools.build_e2e_documented_coverage"])
    ####


def e2e_all() -> None:
    """Validate every application-level case, including opt-in categories."""
    run([project_python(), "-m", "pytest", "tests/e2e", "-m", "not runtime", "--basetemp", ".pytest-e2e-all"])
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
            *tool_script("audit_equation_provenance.py"),
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


def daveml_layer_dispositions() -> None:
    """Validate explicit DAVE-ML family-library layer dispositions."""

    run(
        tool_script(
            "validate_daveml_layer_dispositions.py",
            "--registry",
            "verification/daveml_family_layer_dispositions.yaml",
            "--output",
            "verification/daveml_family_layer_dispositions.json",
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


def daveml_tuning() -> None:
    """Generate reproducible DAVE-ML tuning evidence."""

    run(tool_script("validate_daveml_tuning.py"))
    ####


def daveml_scenario() -> None:
    """Generate reproducible DAVE-ML smoke scenario evidence."""

    run(tool_script("validate_daveml_scenario.py"))
    ####


def daveml_release() -> None:
    """Run the fresh-process DAVE-ML promotion gate."""

    run(tool_script("validate_daveml_release.py"))
    ####


def daveml_equilibrium_trim() -> None:
    """Generate reproducible F-16 DAVE-ML equilibrium trim evidence."""

    run(tool_script("validate_daveml_equilibrium_trim.py"))
    ####


def daveml_f16_runtime_linearization() -> None:
    """Generate the plant-derived F-16 runtime A/B evidence."""

    run(tool_script("validate_f16_runtime_linearization.py"))
    ####


def daveml_f16_lqr_trim_hold() -> None:
    """Generate bounded local F-16 LQR trim-hold evidence."""

    run(tool_script("validate_f16_lqr_trim_hold.py"))
    ####


def daveml_f16_physical_allocation() -> None:
    """Generate F-16 source-effectiveness and bounded-allocation evidence."""

    run(tool_script("validate_f16_physical_allocation.py"))
    ####


def daveml_f16_physical_wrench_lqr() -> None:
    """Generate the local F-16 wrench-LQR physical recovery evidence."""

    run(tool_script("validate_f16_physical_wrench_lqr.py"))
    ####


def daveml_f16_physical_wrench_perturbations() -> None:
    """Generate the F-16 local physical-wrench perturbation matrix."""

    run(tool_script("validate_f16_physical_wrench_perturbations.py"))
    ####


def alpha3_f16_physical_r1() -> None:
    """Build the fail-closed F-16 physical-effector R1 boundary artifact."""

    run(tool_script("validate_f16_physical_r1_matrix.py"))
    ####


def alpha3_f16_physical_schedule() -> None:
    """Validate source-effector F-16 physical controller nodes across altitude."""

    run(tool_script("validate_f16_physical_schedule.py"))
    ####


def alpha3_f16_physical_schedule_transition() -> None:
    """Replay the F-16 source-effector schedule through a local transition."""

    run(tool_script("validate_f16_physical_schedule_transition.py"))
    ####


def alpha3_f16_physical_schedule_envelope() -> None:
    """Record F-16 alpha/beta/rate authority boundaries around the schedule."""

    run(tool_script("validate_f16_physical_schedule_envelope.py"))
    ####


def alpha3_x8_physical_mapping() -> None:
    """Record both unresolved X8 physical left/right elevon sign hypotheses."""

    run(tool_script("validate_x8_physical_mapping_evidence.py"))
    ####


def alpha3_b747_physical_schedule() -> None:
    """Validate source-effector B747 nodes across CR-2144 conditions."""

    run(tool_script("validate_b747_physical_schedule.py"))
    ####


def daveml_f16_local_maneuvers() -> None:
    """Generate local F-16 bank/pitch reversal maneuver evidence."""

    run(tool_script("validate_f16_local_maneuvers.py"))
    ####


def daveml_f16_reductions() -> None:
    """Generate local F-16 point-mass and pseudo-6DOF reduction evidence."""

    run(tool_script("validate_f16_reductions.py"))
    ####


def daveml_hl20_load() -> None:
    """Generate reproducible HL-20 DAVE-ML load evidence."""

    run(tool_script("validate_daveml_hl20_load.py"))
    ####


def daveml_hl20_linearization() -> None:
    """Generate reproducible HL-20 DAVE-ML linearization evidence."""

    run(tool_script("validate_daveml_hl20_linearization.py"))
    ####


def daveml_hl20_scenario() -> None:
    """Generate reproducible HL-20 DAVE-ML scenario evidence."""

    run(tool_script("validate_daveml_hl20_scenario.py"))
    ####


def daveml_collection_roundtrip() -> None:
    """Run the fresh-process DAVE-ML collection round trip."""

    run(tool_script("validate_daveml_collection_roundtrip.py"))
    ####


def daveml_nesc_replay() -> None:
    """Run the fresh-process NESC DAVE-ML package replay."""

    run(tool_script("validate_daveml_nesc_replay.py"))
    ####


def daveml_completion_audit() -> None:
    """Emit the requirement-level DaveML completion audit."""

    run(tool_script("audit_daveml_completion.py"))
    ####


def daveml_showcase() -> None:
    """Build the promoted DaveML family evidence-board packs."""

    run(tool_script("build_daveml_showcase_composites.py"))
    ####


def showcase_artifact_boundary() -> None:
    """Build and validate the common showcase artifact boundary."""

    run(tool_script("validate_showcase_artifact_boundary.py", "--build-daveml"))
    ####


def showcase_catalog_references() -> None:
    """Validate aggregate showcase catalogs without reinterpreting children."""

    run(tool_script("validate_showcase_catalog_references.py"))
    ####


def daveml_operational_contracts() -> None:
    """Validate the shared DaveML family operational contracts."""

    run(tool_script("validate_daveml_operational_contracts.py"))
    ####


def daveml_operating_points() -> None:
    """Validate the DaveML family operating-point catalog."""

    run(tool_script("validate_daveml_operating_points.py"))
    ####


def daveml_alpha3_completion() -> None:
    """Build and validate the Alpha 3 DAVE-ML completion evidence."""

    run(tool_script("build_daveml_alpha3_evidence.py"))
    run(tool_script("validate_daveml_alpha3_qualification.py"))
    run(tool_script("validate_daveml_alpha3_completion.py"))
    ####


def daveml_alpha3_qualification() -> None:
    """Build the requested F-16, NESC, and A320 Alpha 3 qualification evidence."""

    run(tool_script("validate_daveml_alpha3_qualification.py"))
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
    check_supported_reference_families()
    check_vehicle_maturity_registry()
    check_vehicle_interfaces()
    check_vehicle_execution_witnesses()
    check_mission_composition_completion()
    check_simulation_runtime_quality()
    onboard_vehicles()
    pseudo6dof_profiles()
    horizontal_fidelity()
    unified_family_manifest()
    family_adapter_registry()
    horizontal_readiness()
    family_strategy_worklists()
    showcase_artifact_boundary()
    run(tool_script("build_alpha3_direct_wrench_contract.py"))
    check_fidelity_parity_contracts()
    check_reference_tuning()
    reduced_tuning_campaigns()
    powered_fixed_wing_mission_proposals()
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
    "install-check": installation_check,
    "docs-doctor": docs_doctor,
    "source-pdf": source_pdf,
    "lint": lint,
    "typecheck": typecheck,
    "grammar": grammar,
    "legacy-audit": legacy_audit,
    "legacy-close-check": legacy_close_check,
    "test": test,
    "test-quick": test_quick,
    "test-changed": test_changed,
    "test-parallel": test_parallel,
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
    "test-f16": lambda: test_vehicle_vertical("f16_s119"),
    "test-a320": lambda: test_vehicle_vertical("a320_openap_3dof"),
    "test-hummingbird-vertical": lambda: test_vehicle_vertical("hummingbird"),
    "test-x15-vertical": lambda: test_vehicle_vertical("x15"),
    "test-vehicle-catalogue": test_vehicle_catalogue,
    "test-x15-segments": test_x15_segments,
    "test-x15-catalog": test_x15_catalog,
    "test-views": test_views,
    "showcase-california-hawaii": showcase_california_hawaii,
    "showcase-hl20-low-fidelity": showcase_hl20_low_fidelity,
    "showcase-hl20-composites": showcase_hl20_composites,
    "showcase-hl20-source-composites": showcase_hl20_source_composites,
    "qualify-hl20": qualify_hl20,
    "qualify-hl20-source-reachability": qualify_hl20_source_reachability,
    "qualify-hl20-terminal-contract": qualify_hl20_terminal_contract,
    "qualify-hl20-source-robustness": qualify_hl20_source_robustness,
    "qualify-passive-deployment": qualify_passive_deployment,
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
    "vehicle-catalogue": vehicle_catalogue,
    "check-reference-families": check_supported_reference_families,
    "check-vehicle-maturity": check_vehicle_maturity_registry,
    "check-vehicle-interfaces": check_vehicle_interfaces,
    "mission-composition-completion": check_mission_composition_completion,
    "check-simulation-runtime-quality": check_simulation_runtime_quality,
    "onboard-vehicles": onboard_vehicles,
    "fidelity-readiness": fidelity_readiness,
    "pseudo6dof-profiles": pseudo6dof_profiles,
    "horizontal-fidelity": horizontal_fidelity,
    "family-adapter-registry": family_adapter_registry,
    "horizontal-readiness": horizontal_readiness,
    "family-strategy-worklists": family_strategy_worklists,
    "showcase-artifact-boundary": showcase_artifact_boundary,
    "showcase-catalog-references": showcase_catalog_references,
    "alpha3-readiness": alpha3_readiness,
    "alpha3-showcase-catalog": alpha3_showcase_catalog,
    "hummingbird-directional": hummingbird_directional,
    "hummingbird-native-horizontal": hummingbird_native_horizontal,
    "hummingbird-native-vertical": hummingbird_native_vertical,
    "f16-physical-schedule-interior": f16_physical_schedule_interior,
    "airbreathing-r1": airbreathing_r1_matrix,
    "a320-r1": a320_r1_matrix,
    "f16-r1": f16_r1_matrix,
    "hl20-r1": hl20_r1_matrix,
    "hummingbird-r1": hummingbird_r1_matrix,
    "x15-r1": x15_r1_matrix,
    "nesc-r1": nesc_r1_matrix,
    "b747-physical-r1": b747_physical_r1_matrix,
    "hummingbird-physical-r1": hummingbird_physical_r1_matrix,
    "tumbling-r1": tumbling_r1_matrix,
    "integration-readiness": integration_readiness,
    "integration-pipeline": integration_pipeline,
    "integration-pilots": integration_pilots,
    "effectivity-preflight": effectivity_preflight,
    "trim-orchestration": trim_orchestration,
    "controller-mission-preflight": controller_mission_preflight,
    "solve-trim-evidence": solve_trim_evidence,
    "reference-tuning": check_reference_tuning,
    "reduced-tuning-campaigns": reduced_tuning_campaigns,
    "powered-fixed-wing-mission-proposals": powered_fixed_wing_mission_proposals,
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
    "daveml-layer-dispositions": daveml_layer_dispositions,
    "daveml-trim": daveml_trim,
    "daveml-atmosphere": daveml_atmosphere,
    "daveml-linearization": daveml_linearization,
    "daveml-tuning": daveml_tuning,
    "daveml-scenario": daveml_scenario,
    "daveml-release": daveml_release,
    "daveml-equilibrium-trim": daveml_equilibrium_trim,
    "daveml-f16-runtime-linearization": daveml_f16_runtime_linearization,
    "daveml-f16-lqr-trim-hold": daveml_f16_lqr_trim_hold,
    "daveml-f16-physical-allocation": daveml_f16_physical_allocation,
    "daveml-f16-physical-wrench-lqr": daveml_f16_physical_wrench_lqr,
    "daveml-f16-physical-wrench-perturbations": daveml_f16_physical_wrench_perturbations,
    "f16-physical-r1": alpha3_f16_physical_r1,
    "f16-physical-schedule": alpha3_f16_physical_schedule,
    "f16-physical-schedule-transition": alpha3_f16_physical_schedule_transition,
    "f16-physical-schedule-envelope": alpha3_f16_physical_schedule_envelope,
    "x8-physical-mapping": alpha3_x8_physical_mapping,
    "b747-physical-schedule": alpha3_b747_physical_schedule,
    "daveml-f16-local-maneuvers": daveml_f16_local_maneuvers,
    "daveml-f16-reductions": daveml_f16_reductions,
    "daveml-hl20-load": daveml_hl20_load,
    "daveml-hl20-linearization": daveml_hl20_linearization,
    "daveml-hl20-scenario": daveml_hl20_scenario,
    "daveml-collection-roundtrip": daveml_collection_roundtrip,
    "daveml-nesc-replay": daveml_nesc_replay,
    "daveml-completion-audit": daveml_completion_audit,
    "daveml-showcase": daveml_showcase,
    "daveml-operational-contracts": daveml_operational_contracts,
    "daveml-operating-points": daveml_operating_points,
    "daveml-alpha3-completion": daveml_alpha3_completion,
    "daveml-alpha3-qualification": daveml_alpha3_qualification,
    "handoff": handoff,
    "check": check,
    "all": check,
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("task", choices=sorted((*TASKS, "test-vehicle")))
    parser.add_argument("vehicle", nargs="?")
    args = parser.parse_args()
    if args.task == "test-vehicle":
        if args.vehicle is None:
            parser.error("test-vehicle requires a vehicle family, for example: f16_s119")
        test_vehicle_vertical(args.vehicle)
        return 0
    if args.vehicle is not None:
        parser.error(f"{args.task} does not accept a vehicle family")
    TASKS[args.task]()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
