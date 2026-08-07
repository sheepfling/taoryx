"""Capture the Simulation Runtime M0 runtime baseline."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import subprocess
import sys
import tempfile
import time
from collections.abc import Callable, Sequence
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from taoryx.runtime.interactive import ControlSpec, InteractiveSession
from taoryx.runtime.program import LoadedProgram
from taoryx.simulation_runtime_contracts import SimulationRuntimeStatus

ROOT = Path(__file__).resolve().parents[1]
PYTHON = Path(sys.executable)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
    ####


def _file_inventory(root: Path) -> tuple[dict[str, object], ...]:
    records: list[dict[str, object]] = []
    for path in sorted(item for item in root.rglob("*") if item.is_file()):
        records.append(
            {
                "path": str(path.relative_to(root)),
                "bytes": path.stat().st_size,
                "sha256": _sha256(path),
            }
        )
    return tuple(records)
    ####


def _artifact_hashes(inventory: Sequence[dict[str, object]], names: Sequence[str]) -> dict[str, str | None]:
    by_path = {str(item["path"]): str(item["sha256"]) for item in inventory}
    return {name: by_path.get(name) for name in names}
    ####


def _portable_token(token: str, output_dir: Path) -> str:
    candidate = Path(token)
    candidate_resolved = candidate.resolve()
    try:
        return str(candidate_resolved.relative_to(ROOT))
    except ValueError:
        pass
    for selected in (output_dir, output_dir.parent / f"{output_dir.name}-repeat"):
        selected_resolved = selected.resolve()
        try:
            return "<output-dir>" + str(candidate_resolved.relative_to(selected_resolved))
        except ValueError:
            continue
    return token
    ####


def _portable_command(
    command: Sequence[str] | Sequence[Sequence[str]], output_dir: Path
) -> list[str] | list[list[str]]:
    if command and isinstance(command[0], (list, tuple)):
        return [[_portable_token(str(token), output_dir) for token in nested] for nested in command]  # type: ignore[arg-type]
    return [_portable_token(str(token), output_dir) for token in command]  # type: ignore[arg-type]
    ####


def _status_from_return_code(return_code: int) -> SimulationRuntimeStatus:
    if return_code == 0:
        return SimulationRuntimeStatus.PASSED
    if return_code == 1:
        return SimulationRuntimeStatus.INCOMPLETE
    return SimulationRuntimeStatus.FAILED
    ####


def _run_subprocess(command: Sequence[str], output_dir: Path) -> tuple[int, str | None]:
    output_dir.mkdir(parents=True, exist_ok=True)
    try:
        completed = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, check=False, timeout=180)
    except OSError as error:
        return 2, str(error)
    except subprocess.TimeoutExpired:
        return 1, "command exceeded the 180-second M0 baseline timeout"
    if completed.returncode == 0:
        return 0, None
    detail = (completed.stderr or completed.stdout).strip().splitlines()
    return completed.returncode, detail[-1] if detail else "command failed without diagnostic output"
    ####


def _source_command(output_dir: Path, problem: Path, tables: Sequence[Path], profile: str) -> tuple[str, ...]:
    artifact = output_dir / "run-artifact.json"
    return (
        str(PYTHON),
        "-m",
        "taoryx.runtime.cli",
        "run",
        str(problem),
        *(str(table) for table in tables),
        "--profile",
        profile,
        "--integrator",
        "rk4",
        "--max-steps",
        "20000",
        "--output-dir",
        str(output_dir),
        "--report",
        str(output_dir / "run-report.json"),
        "--artifact",
        str(artifact),
    )
    ####


def _composition_commands(output_dir: Path, composition: Path) -> tuple[tuple[str, ...], ...]:
    compiled = output_dir / "composition.json"
    run_dir = output_dir / "run"
    return (
        (
            str(PYTHON),
            "-m",
            "taoryx.runtime.cli",
            "vehicle",
            "compose",
            str(composition),
            "--output",
            str(compiled),
        ),
        (
            str(PYTHON),
            "-m",
            "taoryx.runtime.cli",
            "vehicle",
            "run",
            str(compiled),
            "--output-dir",
            str(run_dir),
        ),
    )
    ####


def _run_sequence(commands: Sequence[Sequence[str]], output_dir: Path) -> tuple[int, str | None]:
    for command in commands:
        return_code, detail = _run_subprocess(command, output_dir)
        if return_code != 0:
            return return_code, detail
    return 0, None
    ####


def _run_interactive(output_dir: Path) -> tuple[int, str | None]:
    try:
        program = LoadedProgram.load(
            ROOT / "examples/showcases/california_to_hawaii/mission.prb",
            (ROOT / "examples/showcases/california_to_hawaii/aero.tbl",),
            profile="taoryx",
        )
        session = InteractiveSession(
            program.case(),
            controls=(
                ControlSpec("throttle", unit="fraction", lower=0.0, upper=1.0),
                ControlSpec("alpha-deg", unit="deg", lower=-20.0, upper=20.0),
                ControlSpec("bank-deg", unit="deg", lower=-180.0, upper=180.0),
            ),
        )
        session.step(10.0, {"throttle": 1.0, "alpha-deg": 0.0, "bank-deg": 0.0})
        session.step(10.0, {"throttle": 0.8, "alpha-deg": 2.0, "bank-deg": 5.0})
        session.to_run_artifact().write_json(output_dir / "interactive-artifact.json")
    except (OSError, RuntimeError, TypeError, ValueError) as error:
        return 2, str(error)
    return 0, None
    ####


def _capture_case(
    case_id: str,
    description: str,
    operation: str,
    expected_disposition: SimulationRuntimeStatus,
    command: Sequence[str] | Sequence[Sequence[str]],
    runner: Callable[[Path], tuple[int, str | None]],
    output_dir: Path,
    repeat_names: Sequence[str],
) -> dict[str, object]:
    started = time.perf_counter()
    return_code, error = runner(output_dir)
    inventory = _file_inventory(output_dir)
    total_bytes = sum(int(item["bytes"]) for item in inventory)
    record: dict[str, object] = {
        "id": case_id,
        "description": description,
        "operation": operation,
        "expected_disposition": expected_disposition.value,
        "observed_status": _status_from_return_code(return_code).value,
        "return_code": return_code,
        "elapsed_s": round(time.perf_counter() - started, 6),
        "command": _portable_command(command, output_dir),
        "error": error,
        "file_count": len(inventory),
        "total_bytes": total_bytes,
        "files": list(inventory),
        "repeatability_basis": _artifact_hashes(inventory, repeat_names),
    }
    return record
    ####


def _case_specs() -> tuple[dict[str, object], ...]:
    return (
        {
            "id": "two-stage-ballistic",
            "description": "Small staged-rocket source batch smoke test.",
            "operation": "batch",
            "expected": SimulationRuntimeStatus.PASSED,
            "command": lambda output: _source_command(
                output,
                ROOT / "examples/mission_families/two_stage_ballistic_rocket/mission.prb",
                (),
                "taos96",
            ),
            "repeat_names": ("run-artifact.json",),
        },
        {
            "id": "two-stage-demo",
            "description": "Changing-mass source tables and separation example.",
            "operation": "batch",
            "expected": SimulationRuntimeStatus.PASSED,
            "command": lambda output: _source_command(
                output,
                ROOT / "examples/vehicle_families/staged_rocket/two_stage_demo/mission.prb",
                (
                    ROOT / "examples/vehicle_families/staged_rocket/two_stage_demo/aero.tbl",
                    ROOT / "examples/vehicle_families/staged_rocket/two_stage_demo/propulsion.tbl",
                ),
                "taoryx",
            ),
            "repeat_names": ("run-artifact.json",),
        },
        {
            "id": "nesc-pseudo6dof",
            "description": "NESC staged source-history pseudo-6DOF witness.",
            "operation": "composition_batch",
            "expected": SimulationRuntimeStatus.DEVELOPMENT,
            "composition": ROOT / "examples/vehicle_composition/nesc_staged_source_replay_pseudo6dof_compose.yaml",
            "repeat_names": ("run/truth_telemetry.csv", "run/status_trace.json"),
        },
        {
            "id": "x15-staged-pseudo6dof",
            "description": "X-15-scaled staged booster reachability witness.",
            "operation": "composition_batch",
            "expected": SimulationRuntimeStatus.DEVELOPMENT,
            "composition": ROOT / "examples/vehicle_composition/x15_staged_booster_reachability_pseudo6dof_compose.yaml",
            "repeat_names": ("run/truth_telemetry.csv", "run/status_trace.json"),
        },
        {
            "id": "hl20-source-release-pseudo6dof",
            "description": "HL-20 source-booster release pseudo-6DOF witness.",
            "operation": "composition_batch",
            "expected": SimulationRuntimeStatus.DEVELOPMENT,
            "composition": ROOT / "examples/vehicle_composition/hl20_source_booster_release_replay_pseudo6dof_compose.yaml",
            "repeat_names": ("run/truth_telemetry.csv", "run/status_trace.json"),
        },
        {
            "id": "hl20-four-fidelity",
            "description": "HL-20 California-Hawaii four-tier release/glide bundle.",
            "operation": "batch",
            "expected": SimulationRuntimeStatus.DEVELOPMENT,
            "command": lambda output: (
                str(PYTHON),
                str(ROOT / "examples/showcases/hl20_california_to_hawaii/run_low_fidelity.py"),
                "--output-dir",
                str(output),
            ),
            "repeat_names": ("bundle-manifest.json", "composites/showcase-summary.json"),
        },
        {
            "id": "synthetic-california-hawaii",
            "description": "Synthetic native rigid-body California-Hawaii route.",
            "operation": "batch",
            "expected": SimulationRuntimeStatus.DEVELOPMENT,
            "command": lambda output: (
                str(PYTHON),
                str(ROOT / "examples/showcases/california_to_hawaii/run_showcase.py"),
                "--output-dir",
                str(output),
            ),
            "repeat_names": ("realized_fidelity.json",),
        },
        {
            "id": "interactive-california-hawaii",
            "description": "Two accepted external interactive steps over the synthetic route.",
            "operation": "interactive",
            "expected": SimulationRuntimeStatus.PASSED,
            "interactive": True,
            "repeat_names": ("interactive-artifact.json",),
        },
    )
    ####


def _resolve_case_runner(
    spec: dict[str, object], output_dir: Path
) -> tuple[Sequence[str] | Sequence[Sequence[str]], Callable[[Path], tuple[int, str | None]]]:
    command_value = spec.get("command")
    if spec.get("interactive"):
        runner = _run_interactive
        command: Sequence[str] | Sequence[Sequence[str]] = ("python", "interactive-session-baseline")
    elif isinstance(spec.get("composition"), Path):
        composition = spec["composition"]
        assert isinstance(composition, Path)
        commands = _composition_commands(output_dir, composition)
        runner = lambda destination: _run_sequence(commands, destination)
        command = commands
    elif callable(command_value):
        command = command_value(output_dir)
        runner = lambda destination: _run_subprocess(command, destination)
        source_bound = spec.get("command_source_bound")
        if callable(source_bound):
            base_command = command
            source_bound_command = source_bound(output_dir)
            runner = lambda destination: _run_sequence((base_command, source_bound_command), destination)
            command = (base_command, source_bound_command)
    else:
        raise ValueError(f"baseline case {spec['id']!r} has no executable command")
    return command, runner
    ####


def _run_case(spec: dict[str, object], output_dir: Path) -> dict[str, object]:
    command, runner = _resolve_case_runner(spec, output_dir)
    first = _capture_case(
        str(spec["id"]),
        str(spec["description"]),
        str(spec["operation"]),
        spec["expected"],  # type: ignore[arg-type]
        command,
        runner,
        output_dir,
        tuple(str(name) for name in spec["repeat_names"]),  # type: ignore[arg-type]
    )
    repeat_dir = output_dir.parent / f"{output_dir.name}-repeat"
    repeat_command, repeat_runner = _resolve_case_runner(spec, repeat_dir)
    second = _capture_case(
        str(spec["id"]),
        str(spec["description"]),
        str(spec["operation"]),
        spec["expected"],  # type: ignore[arg-type]
        repeat_command,
        repeat_runner,
        repeat_dir,
        tuple(str(name) for name in spec["repeat_names"]),  # type: ignore[arg-type]
    )
    first_hashes = first["repeatability_basis"]
    second_hashes = second["repeatability_basis"]
    repeatable = (
        bool(first_hashes)
        and all(value is not None for value in first_hashes.values())
        and first_hashes == second_hashes
        and second["return_code"] == first["return_code"]
    )
    first["repeatability"] = {
        "repeat_run_status": second["observed_status"],
        "repeat_run_elapsed_s": second["elapsed_s"],
        "repeatable": repeatable,
        "comparison": {"first": first_hashes, "second": second_hashes},
    }
    return first
    ####


def build_baseline(output: Path) -> dict[str, Any]:
    """Execute and record the canonical Simulation Runtime baseline set."""

    with tempfile.TemporaryDirectory(prefix="taoryx-simulation-runtime-baseline-") as temporary:
        temporary_root = Path(temporary)
        cases = [_run_case(spec, temporary_root / str(spec["id"])) for spec in _case_specs()]
    return {
        "schema": "taoryx.simulation-runtime-baseline/v1alpha1",
        "captured_at_utc": datetime.now(UTC).isoformat(),
        "git_revision": subprocess.check_output(("git", "rev-parse", "HEAD"), cwd=ROOT, text=True).strip(),
        "working_tree_dirty": bool(subprocess.check_output(("git", "status", "--porcelain"), cwd=ROOT, text=True).strip()),
        "runtime": {
            "python": platform.python_version(),
            "platform": platform.platform(),
            "executable": str(PYTHON),
        },
        "status_vocabulary": [status.value for status in SimulationRuntimeStatus],
        "cases": cases,
        "claim_boundary": (
            "This is a Simulation Runtime execution, artifact-size, and repeatability baseline. "
            "It does not establish numerical qualification, physical-effector behavior, "
            "historical fidelity, or mission capability."
        ),
    }
    ####


def main(argv: list[str] | None = None) -> int:
    """Write the Simulation Runtime baseline report."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "verification/simulation_runtime_baseline.json")
    args = parser.parse_args(argv)
    report = build_baseline(args.output)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
