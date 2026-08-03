"""Synthetic executable inputs for the extended family showcases."""

from __future__ import annotations

import hashlib
import json
import math
import mimetypes
from pathlib import Path
from typing import Callable

from taoryx.family_debug_rendering import FamilyDebugRenderReport, render_family_debug_artifacts
from taoryx.outputs import DynamicsKind, EventRecord, RunArtifact, TelemetryChannel, VehicleKind, VehicleTelemetry
from taoryx.showcase import (
    ArtifactFile,
    EvidenceBoardSpec,
    FidelityShowcaseRealization,
    build_showcase_run_artifact,
)


def generate_showcase(showcase_id: str, family: str, output_dir: str | Path) -> tuple[FamilyDebugRenderReport, ...]:
    """Build and persist a synthetic showcase artifact and its debug panels."""

    builders: dict[str, Callable[[], RunArtifact]] = {
        "orbital_insertion_coast_reentry": _orbital_artifact,
        "suborbital_ballistic_return": _suborbital_artifact,
        "quadcopter_drone_racetrack": _quadcopter_artifact,
    }
    artifact = builders[showcase_id]()
    root = Path(output_dir)
    root.mkdir(parents=True, exist_ok=True)
    artifact.write_json(root / "telemetry.json")
    artifact.write_sqlite(root / "telemetry.sqlite")
    (root / "telemetry.txt").write_text(artifact.format_text(max_rows=5), encoding="utf-8")
    reports = render_family_debug_artifacts(artifact, root / "plots", family)
    realization = _realization(showcase_id, family, artifact)
    (root / "realized_fidelity.json").write_text(
        json.dumps(realization.model_dump(mode="json"), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (root / "claim.json").write_text(
        json.dumps(
            {
                "claim": realization.claim,
                "nonclaims": list(realization.nonclaims),
                "fidelity": realization.fidelity,
                "control_realization": realization.control_realization,
                "outcome": "completed",
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    scenario_hash = _payload_sha256(
        {
            "showcase_id": showcase_id,
            "family": family,
            "dynamics": artifact.vehicles[next(iter(artifact.vehicles))].dynamics.value,
            "problem": artifact.problem,
        }
    )
    showcase_run = build_showcase_run_artifact(
        realization=realization,
        run_id=f"{showcase_id}-{realization.fidelity}",
        showcase_id=f"org.taoryx.showcase.synthetic.{showcase_id}",
        vehicle_binding_id=f"synthetic.{family}.fixture-v1",
        scenario_contract_sha256=scenario_hash,
        outcome="completed",
        files=_artifact_files(root, scenario_hash),
        board=EvidenceBoardSpec(
            profile="family-debug-board-v1",
            modules=("trajectory_3d", "mission_timeline", "dynamics_and_resources", "envelope_margins"),
        ),
        archetypes=("mission_geometry", "mission_timeline", "dynamics_and_resources", "envelope_and_qualification"),
    )
    manifest = {
        "schema_version": 1,
        "showcase_id": showcase_id,
        "family": family,
        "fidelity": realization.fidelity,
        "control_realization": realization.control_realization,
        "realized_fidelity": "realized_fidelity.json",
        "run_artifacts": [showcase_run.model_dump(mode="json")],
        "files": {},
    }
    manifest_path = root / "manifest.json"
    manifest["files"] = {
        path.relative_to(root).as_posix(): _sha256(path)
        for path in sorted(root.rglob("*"))
        if path.is_file() and path != manifest_path
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return reports
####


def _payload_sha256(value: object) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
####


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()
####


def _artifact_files(root: Path, scenario_hash: str) -> tuple[ArtifactFile, ...]:
    names = ["manifest.json"]
    names.extend(
        path.relative_to(root).as_posix()
        for path in sorted(root.rglob("*"))
        if path.is_file() and path.name != "manifest.json"
    )
    return tuple(
        ArtifactFile(
            path=name,
            sha256=scenario_hash if name == "manifest.json" else _sha256(root / name),
            media_type=mimetypes.guess_type(name)[0] or "application/octet-stream",
        )
        for name in names
    )
####


def _realization(showcase_id: str, family: str, artifact: RunArtifact) -> FidelityShowcaseRealization:
    dynamics = artifact.vehicles[next(iter(artifact.vehicles))].dynamics
    if dynamics is DynamicsKind.POINT_MASS_3DOF:
        fidelity = "point_mass_3dof"
        control_realization = "force_model"
        state_schema = ("position_velocity_mass_resource",)
        effectors: tuple[str, ...] = ()
        available = ("synthetic point-mass translational equations", "declared resource channels")
    else:
        fidelity = "pseudo_6dof"
        control_realization = "response_law"
        state_schema = ("position_velocity_attitude_response_rates",)
        effectors = ()
        available = ("synthetic translational telemetry", "named kinematic attitude/rate response")
    return FidelityShowcaseRealization(
        fidelity=fidelity,
        control_realization=control_realization,
        realization_id=f"synthetic.{showcase_id}.{fidelity}.v1",
        state_schema=state_schema,
        semantic_command_mapping={"mission": "synthetic showcase fixture", "controls": "semantic telemetry channels"},
        physical_effectors=effectors,
        available_physics=available,
        claim=f"Synthetic {family} fixture emits a reproducible {fidelity} evidence artifact and family debug panels.",
        nonclaims=(
            "source-grounded vehicle fidelity",
            "physical actuator, moment, rotor, wheel, or thruster qualification",
            "family-wide mission qualification",
        ),
        evidence_grade="synthetic",
    )
####


def _orbital_artifact() -> RunArtifact:
    times = _times(0.0, 1800.0, 61)
    altitude = [150_000.0 + 80_000.0 * math.sin(math.pi * time / 1800.0) for time in times]
    speed = [7_500.0 - 900.0 * math.sin(math.pi * time / 1800.0) for time in times]
    channels = _channels(
        times,
        {
            "position.altitude.geodetic": (altitude, "m"),
            "velocity.ecfc.x": ([value for value in speed], "m/s"),
            "velocity.ecfc.y": ([0.15 * value for value in speed], "m/s"),
            "velocity.ecfc.z": ([20.0 * math.cos(math.pi * time / 1800.0) for time in times], "m/s"),
            "orbital.specific_energy": ([-30.0e6 + 1.0e4 * math.sin(time / 180.0) for time in times], "m2/s2"),
            "orbital.angular_momentum": ([5.2e10 + 2.0e8 * math.cos(time / 300.0) for time in times], "m2/s"),
            "position.ecfc.x": ([value * math.cos(time / 1800.0) for value, time in zip(altitude, times, strict=True)], "m"),
            "position.ecfc.y": ([value * math.sin(time / 1800.0) for value, time in zip(altitude, times, strict=True)], "m"),
            "aerodynamics.angle_of_attack": ([2.0 + 1.0 * math.sin(time / 120.0) for time in times], "deg"),
            "aerodynamics.dynamic_pressure": ([100.0 + 400.0 * math.exp(-alt / 80_000.0) for alt in altitude], "Pa"),
        },
    )
    events = _events("orbiter", [(0.0, "insertion-burn", "segment_transition"), (600.0, "apoapsis", "apoapsis"), (1200.0, "periapsis", "periapsis"), (1700.0, "reentry", "reentry")])
    return _artifact("orbital_insertion_coast_reentry", "orbiter", VehicleKind.GENERIC, DynamicsKind.POINT_MASS_3DOF, times, channels, events)
####


def _suborbital_artifact() -> RunArtifact:
    times = _times(0.0, 900.0, 61)
    altitude = [max(0.0, 90_000.0 * math.sin(math.pi * time / 900.0)) for time in times]
    speed = [1_400.0 + 900.0 * math.sin(math.pi * time / 900.0) for time in times]
    channels = _channels(
        times,
        {
            "position.altitude.geodetic": (altitude, "m"),
            "velocity.ecfc.x": (speed, "m/s"),
            "aerodynamics.mach": ([value / 340.0 for value in speed], "-"),
            "aerodynamics.dynamic_pressure": ([101_325.0 * math.exp(-alt / 8500.0) * (value / 340.0) ** 2 for alt, value in zip(altitude, speed, strict=True)], "Pa"),
            "aerodynamics.normal_load": ([1.0 + 2.0 * math.sin(math.pi * time / 900.0) for time in times], "g"),
            "mass.total": ([500.0 - 40.0 * min(1.0, time / 120.0) for time in times], "kg"),
            "propulsion.thrust": ([20_000.0 if time < 120.0 else 0.0 for time in times], "N"),
            "mass.flow": ([0.33 if time < 120.0 else 0.0 for time in times], "kg/s"),
        },
    )
    events = _events("vehicle", [(0.0, "launch", "segment_transition"), (450.0, "apogee", "apogee"), (650.0, "atmospheric-return", "atmospheric-return"), (900.0, "ground", "ground")])
    return _artifact("suborbital_ballistic_return", "vehicle", VehicleKind.ROCKET, DynamicsKind.POINT_MASS_3DOF, times, channels, events)
####


def _quadcopter_artifact() -> RunArtifact:
    times = _times(0.0, 240.0, 81)
    channels = _channels(
        times,
        {
            "position.ecfc.x": ([50.0 * math.sin(time / 20.0) for time in times], "m"),
            "position.ecfc.y": ([50.0 * math.sin(time / 40.0) for time in times], "m"),
            "position.altitude.geodetic": ([40.0 + 3.0 * math.sin(time / 15.0) for time in times], "m"),
            "guidance.altitude_command": ([40.0 for _ in times], "m"),
            "velocity.ecfc.z": ([0.2 * math.cos(time / 15.0) for time in times], "m/s"),
            "velocity.airspeed": ([15.0 + 2.0 * math.sin(time / 18.0) for time in times], "m/s"),
            "environment.wind.east": ([4.0 + 1.0 * math.sin(time / 30.0) for time in times], "m/s"),
            "environment.wind.north": ([2.0 for _ in times], "m/s"),
            "environment.wind.down": ([0.0 for _ in times], "m/s"),
            "attitude.roll": ([5.0 * math.sin(time / 12.0) for time in times], "deg"),
            "attitude.pitch": ([3.0 * math.cos(time / 15.0) for time in times], "deg"),
            "attitude.yaw": ([90.0 + 10.0 * math.sin(time / 25.0) for time in times], "deg"),
            "rates.body": ([0.1 * math.cos(time / 12.0) for time in times], "rad/s"),
            "control.throttle": ([0.55 + 0.05 * math.sin(time / 20.0) for time in times], "fraction"),
            "control.roll": ([0.2 * math.sin(time / 12.0) for time in times], "fraction"),
            "control.pitch": ([0.1 * math.cos(time / 15.0) for time in times], "fraction"),
            "control.yaw": ([0.1 * math.sin(time / 25.0) for time in times], "fraction"),
        },
    )
    events = _events("drone", [(0.0, "launch", "segment_transition"), (60.0, "lap-complete", "lap-complete"), (120.0, "wind-profile-change", "wind-profile-change"), (180.0, "control-saturation", "control-saturation"), (240.0, "lap-complete", "lap-complete")])
    return _artifact("quadcopter_drone_racetrack", "drone", VehicleKind.GENERIC, DynamicsKind.KINEMATIC_3_PLUS_3_DOF, times, channels, events)
####


def _artifact(problem: str, vehicle_id: str, kind: VehicleKind, dynamics: DynamicsKind, times: list[float], channels: dict[str, TelemetryChannel], events: list[EventRecord]) -> RunArtifact:
    vehicle = VehicleTelemetry(vehicle_id=vehicle_id, name=vehicle_id, kind=kind, dynamics=dynamics, times=times, channels=channels, events=events)
    return RunArtifact(problem=problem, vehicles={vehicle_id: vehicle}, parameters={"provenance": "synthetic_showcase_fixture", "engineering_validity": False})
####


def _channels(times: list[float], definitions: dict[str, tuple[list[float], str]]) -> dict[str, TelemetryChannel]:
    return {name: TelemetryChannel(source_name=name, semantic_name=name, unit=unit, values=values) for name, (values, unit) in definitions.items() if len(values) == len(times)}
####


def _events(vehicle: str, definitions: list[tuple[float, str, str]]) -> list[EventRecord]:
    return [EventRecord(time=time, vehicle=vehicle, name=name, kind=kind) for time, name, kind in definitions]
####


def _times(start: float, end: float, count: int) -> list[float]:
    return [start + (end - start) * index / (count - 1) for index in range(count)]
####
