"""Reproducible Simulation Runtime scenario bundles."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import shutil
import subprocess
import sys
from collections.abc import Callable, Sequence
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path

from taoryx.outputs import DynamicsKind, RunArtifact, TelemetryChannel, VehicleKind, VehicleTelemetry
from taoryx.runtime.runner import run_files
from taoryx.simulation_runtime_catalog import ROOT, SimulationRuntimeScenario
from taoryx.simulation_runtime_contracts import SimulationRuntimeStatus
from taoryx.simulation_runtime_manifest import (
    SimulationRuntimeManifestArtifact,
    SimulationRuntimeManifestSourceInput,
    artifact_inventory,
    build_simulation_runtime_run_manifest,
    read_simulation_runtime_run_manifest,
)
from taoryx.vehicle_composition import load_compiled_vehicle_composition
from taoryx.visualization import render_run_artifact_plots


def build_simulation_runtime_bundle(
    scenario: SimulationRuntimeScenario,
    output_dir: str | Path,
    *,
    root: Path = ROOT,
    run: bool = True,
    plots: bool = True,
) -> dict[str, object]:
    """Collect one catalog scenario into a portable evidence directory."""

    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    input_dir = output / "inputs"
    input_dir.mkdir(parents=True, exist_ok=True)
    copied_inputs = _copy_inputs(scenario, input_dir, root)
    (output / "catalog-entry.json").write_text(json.dumps(scenario.as_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")

    return_code = 0
    status = SimulationRuntimeStatus.PASSED if run else SimulationRuntimeStatus.INCOMPLETE
    termination: dict[str, object] = {"completed": run, "reason": "not_run" if not run else "pending"}
    artifact_path: Path | None = None
    if run:
        return_code, artifact_path, termination = _run_scenario(scenario, output, root)
        status = _status_from_return_code(return_code)
        if plots and artifact_path is not None and artifact_path.is_file():
            plot_dir = output / "plots"
            try:
                artifact = RunArtifact.model_validate_json(artifact_path.read_text(encoding="utf-8"))
                render_run_artifact_plots(artifact, plot_dir)
            except (OSError, TypeError, ValueError):
                termination["plots"] = "not_available_for_artifact"
            else:
                termination["plots"] = "rendered"

    reproduction = _write_reproduction(output, scenario, root)
    source_inputs = tuple(_copied_input_record(item, copied_inputs[item.path], output) for item in scenario.inputs)
    artifacts = _bundle_artifacts(output)
    manifest = build_simulation_runtime_run_manifest(
        scenario_id=scenario.id,
        status=status,
        expected_disposition=scenario.expected_disposition,
        operation=scenario.operation,
        fidelity=scenario.fidelity,
        realization=scenario.realization,
        source_inputs=source_inputs,
        integration={"profile": scenario.profile, "integrator": scenario.integrator, "seed": scenario.seed, "command": list(scenario.run_command)},
        time={"requested_duration_s": scenario.requested_duration_s, "accepted_start_s": _accepted_time(artifact_path)[0], "accepted_end_s": _accepted_time(artifact_path)[1]},
        termination={**termination, "return_code": return_code},
        artifacts=artifacts,
        claim_boundary=scenario.claim_boundary,
        reproduction_command=reproduction,
    )
    manifest_path = manifest.write_json(output / "run-manifest.json")
    read_simulation_runtime_run_manifest(manifest_path)
    index: dict[str, object] = {
        "schema": "taoryx.simulation-runtime-bundle/v1alpha1",
        "scenario_id": scenario.id,
        "status": status.value,
        "expected_disposition": scenario.expected_disposition.value,
        "output_dir": str(output),
        "manifest": str(manifest_path),
        "reproduction": str(output / "reproduction.txt"),
        "artifact_count": len(artifacts),
        "return_code": return_code,
        "claim_boundary": scenario.claim_boundary,
    }
    (output / "bundle-index.json").write_text(json.dumps(index, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if return_code not in {0, 1}:
        raise RuntimeError(f"scenario bundle run failed with return code {return_code}")
    return index
    ####


def _run_scenario(scenario: SimulationRuntimeScenario, output: Path, root: Path) -> tuple[int, Path | None, dict[str, object]]:
    run_dir = output / "run"
    run_dir.mkdir(parents=True, exist_ok=True)
    if scenario.entrypoint == "source":
        problem = _input_path(scenario, "problem", root)
        tables = tuple(scenario.input_path(item, root=root) for item in scenario.inputs if item.kind == "table")
        report = run_files(problem, tables, output_dir=run_dir, max_steps=20000, integrator=scenario.integrator, profile=scenario.profile)
        artifact_path = run_dir / "run-artifact.json"
        if report.artifacts:
            report.artifacts[0].write_json(artifact_path)
        (run_dir / "run-report.json").write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return report.exit_code, artifact_path if artifact_path.is_file() else None, {
            "completed": report.status is SimulationRuntimeStatus.PASSED,
            "reason": report.results[0].stop_reason if report.results else "no_execution_result",
        }
    if scenario.entrypoint == "interactive":
        command = [sys.executable, str(_input_path(scenario, "stepping_runner", root)), "--output-dir", str(run_dir)]
        completed = subprocess.run(command, cwd=root, capture_output=True, text=True, check=False, timeout=180)
        (run_dir / "command-output.txt").write_text(completed.stdout + completed.stderr, encoding="utf-8")
        return completed.returncode, run_dir / "run-artifact.json", {"completed": completed.returncode == 0, "reason": "interactive_witness"}
    if scenario.entrypoint == "composition":
        from taoryx.runtime.cli import main

        request = _input_path(scenario, "composition", root)
        composition_path = output / "composition.json"
        compose_code = _invoke_cli(main, ["vehicle", "compose", str(request), "--output", str(composition_path)])
        if compose_code != 0:
            return compose_code, None, {"completed": False, "reason": "composition_compile_failed"}
        run_code = _invoke_cli(main, ["vehicle", "run", str(composition_path), "--output-dir", str(run_dir)])
        composition = load_compiled_vehicle_composition(composition_path)
        composition_artifact_path = write_composition_run_artifact(run_dir, composition)
        return run_code, composition_artifact_path, {"completed": run_code == 0, "reason": "composition_batch" if run_code == 0 else "composition_runtime"}
    script = _input_path(scenario, "showcase_runner", root)
    completed = subprocess.run(
        [sys.executable, str(script), "--output-dir", str(run_dir)],
        cwd=root,
        capture_output=True,
        text=True,
        check=False,
        timeout=180,
    )
    (run_dir / "command-output.txt").write_text(completed.stdout + completed.stderr, encoding="utf-8")
    return completed.returncode, None, {"completed": completed.returncode == 0, "reason": "showcase_runner" if completed.returncode == 0 else "showcase_failed"}
    ####


def _invoke_cli(main: Callable[[list[str]], int], arguments: list[str]) -> int:
    buffer = StringIO()
    with redirect_stdout(buffer):
        return int(main(arguments))
    ####


def _copy_inputs(scenario: SimulationRuntimeScenario, input_dir: Path, root: Path) -> dict[str, Path]:
    copied: dict[str, Path] = {}
    for item in scenario.inputs:
        source = scenario.input_path(item, root=root)
        relative = Path(item.path) if not Path(item.path).is_absolute() else Path(source.name)
        destination = input_dir / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        copied[item.path] = destination
    return copied
    ####


def _copied_input_record(item: object, path: Path, output: Path) -> SimulationRuntimeManifestSourceInput:
    catalog_input = item
    role = str(getattr(catalog_input, "role"))
    data = path.read_bytes()
    return SimulationRuntimeManifestSourceInput(
        path=str(path.relative_to(output)),
        role=role,
        sha256=hashlib.sha256(data).hexdigest(),
        bytes=len(data),
    )
    ####


def _bundle_artifacts(output: Path) -> tuple[SimulationRuntimeManifestArtifact, ...]:
    return tuple(item for item in artifact_inventory(output, exclude=("run-manifest.json",)) if not item.path.startswith("inputs/"))
    ####


def _write_reproduction(output: Path, scenario: SimulationRuntimeScenario, root: Path) -> tuple[str, ...]:
    output.mkdir(parents=True, exist_ok=True)
    commands = [_replace_command(command, output, root) for command in scenario.setup_commands]
    commands.append(_replace_command(scenario.run_command, output, root))
    lines = [
        "# TAORYX Simulation Runtime reproducible bundle",
        f"# scenario_id: {scenario.id}",
        "# Run from this bundle directory with the repository environment active.",
    ]
    for command in commands:
        lines.append(" ".join(command))
    (output / "reproduction.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return commands[-1]
    ####


def _replace_command(command: Sequence[str], output: Path, root: Path) -> tuple[str, ...]:
    replaced: list[str] = []
    for token in command:
        value = token.replace("{output_dir}", ".")
        for candidate in sorted((path for path in root.rglob("*") if path.is_file()), key=lambda item: len(str(item)), reverse=True):
            if str(candidate.relative_to(root)) in value:
                value = value.replace(str(candidate.relative_to(root)), f"inputs/{candidate.relative_to(root)}")
        replaced.append(value)
    return tuple(replaced)
    ####


def _input_path(scenario: SimulationRuntimeScenario, role: str, root: Path) -> Path:
    for item in scenario.inputs:
        if item.role == role or (role == "composition" and item.kind == "composition"):
            return scenario.input_path(item, root=root)
    raise ValueError(f"scenario {scenario.id!r} has no {role!r} input")
    ####


def _accepted_time(path: Path | None) -> tuple[float | None, float | None]:
    if path is None or not path.is_file():
        return None, None
    try:
        artifact = RunArtifact.model_validate_json(path.read_text(encoding="utf-8"))
    except (OSError, TypeError, ValueError):
        return None, None
    times = [time for vehicle in artifact.vehicles.values() for time in vehicle.times]
    return (min(times), max(times)) if times else (None, None)
    ####


def _status_from_return_code(return_code: int) -> SimulationRuntimeStatus:
    if return_code == 0:
        return SimulationRuntimeStatus.PASSED
    if return_code == 1:
        return SimulationRuntimeStatus.INCOMPLETE
    return SimulationRuntimeStatus.FAILED
    ####


def write_composition_run_artifact(output_dir: str | Path, composition: object) -> Path | None:
    """Project scalar truth telemetry into the common normalized RunArtifact shape."""

    destination = Path(output_dir)
    telemetry_path = destination / "truth_telemetry.csv"
    if not telemetry_path.is_file():
        return None
    with telemetry_path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        return None
    time_name = "time_s" if "time_s" in rows[0] else "time" if "time" in rows[0] else None
    if time_name is None:
        return None
    times = [float(row[time_name]) for row in rows]
    channels: dict[str, TelemetryChannel] = {}
    for name in rows[0]:
        if name == time_name:
            continue
        values: list[float | None] = []
        numeric = True
        for row in rows:
            raw = row.get(name, "")
            if raw in {None, "", "nan", "NaN"}:
                values.append(None)
                continue
            try:
                value = float(raw)
            except (TypeError, ValueError):
                numeric = False
                break
            values.append(value if math.isfinite(value) else None)
        if numeric:
            channels[name] = TelemetryChannel(source_name=name, semantic_name=name, values=values)
    fidelity = str(getattr(composition, "fidelity", ""))
    dynamics = {
        "point_mass_3dof": DynamicsKind.POINT_MASS_3DOF,
        "pseudo_6dof": DynamicsKind.KINEMATIC_3_PLUS_3_DOF,
        "rigid_body_6dof_direct_wrench": DynamicsKind.RIGID_BODY_6DOF,
        "rigid_body_6dof_surface_allocated": DynamicsKind.RIGID_BODY_6DOF,
    }.get(fidelity, DynamicsKind.KINEMATIC_3_PLUS_3_DOF)
    artifact = RunArtifact(
        problem=str(getattr(composition, "id", "composition-run")),
        vehicles={
            str(getattr(composition, "vehicle_id", "vehicle")): VehicleTelemetry(
                vehicle_id=str(getattr(composition, "vehicle_id", "vehicle")),
                name=str(getattr(composition, "family_id", "composition")),
                kind=VehicleKind.GENERIC,
                dynamics=dynamics,
                times=times,
                channels=channels,
            )
        },
        scenario_identity=str(getattr(composition, "identity_sha256", "")) or None,
        visualization={
            "source": "SimulationRuntimeCompositionTruthTelemetry",
            "schema_version": 1,
            "claim_boundary": "Generic scalar projection of family-owned truth telemetry; family-native artifacts remain authoritative.",
        },
    )
    return artifact.write_json(destination / "run-artifact.json")
    ####


__all__ = ["build_simulation_runtime_bundle"]
####
