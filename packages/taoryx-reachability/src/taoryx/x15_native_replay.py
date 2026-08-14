"""Selective native X-15 rigid-body replay for reachability evidence."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

from taoryx_x15.resources import model_resource_root

from .reachability_visualization import load_reachability_artifact
from .runtime.runner import run_files

_SOURCE_PROBLEM = "examples/showcases/x15_rocket_to_hawaii/release_glide_parity_6dof.prb"
_SOURCE_TABLE = "tests/fixtures/x15_coherent_6dof_public_research_v1/tables/x15_static_6axis.tbl"
_EARTH_RADIUS_M = 6_378_137.0
_NATIVE_TABLE_MAX_ALTITUDE_M = 24_384.0
# Keep a small margin below the declared 24,384 m table ceiling while still
# allowing the reduced staged witness to provide a native replay checkpoint.
# This is a table-domain bridge, not permission to extrapolate above the
# source envelope.
_NATIVE_REPLAY_SAFE_ALTITUDE_M = 24_000.0


@dataclass(frozen=True, slots=True)
class X15NativeReplayRecord:
    """One native replay selected from a reduced-order envelope sample."""

    query_id: str
    reduced_classification: str
    reduced_fidelity: str
    generated_problem: str
    native_artifact: str | None
    completed: bool
    diagnostics: tuple[dict[str, object], ...]
    bridge: dict[str, object]
    comparison: dict[str, float | None]

    def as_dict(self) -> dict[str, object]:
        return {
            "query_id": self.query_id,
            "reduced_classification": self.reduced_classification,
            "reduced_fidelity": self.reduced_fidelity,
            "generated_problem": self.generated_problem,
            "native_artifact": self.native_artifact,
            "completed": self.completed,
            "diagnostics": list(self.diagnostics),
            "bridge": self.bridge,
            "comparison": self.comparison,
        }


@dataclass(frozen=True, slots=True)
class X15NativeReplayBundle:
    """Manifest and per-candidate native replay records."""

    records: tuple[X15NativeReplayRecord, ...]
    manifest_path: Path


def write_x15_native_boundary_replay(
    envelope_path: str | Path,
    directory: str | Path,
    *,
    max_points: int = 4,
    duration_s: float = 0.01,
    max_steps: int = 50,
) -> X15NativeReplayBundle:
    """Replay selected pseudo-6DOF boundary samples through the native runner.

    The generated native case starts from a reduced-order glide checkpoint in a
    documented local-to-ECIC bridge. This is selective rigid-body evidence, not
    a native batch envelope provider.
    """

    if max_points <= 0 or duration_s <= 0.0 or max_steps <= 0:
        raise ValueError("max_points, duration_s, and max_steps must be positive")
    payload = load_reachability_artifact(envelope_path)
    fidelity = str(payload.get("fidelity", payload.get("study", {}).get("fidelity", "")))
    if fidelity != "pseudo_6dof":
        raise ValueError("native X-15 replay requires a pseudo_6dof reachability artifact")
    samples = _select_samples(payload, max_points)
    destination = Path(directory)
    destination.mkdir(parents=True, exist_ok=True)
    repo_root = model_resource_root()
    source_table = repo_root / _SOURCE_TABLE
    records: list[X15NativeReplayRecord] = []
    for sample in samples:
        records.append(
            _replay_sample(
                sample,
                payload,
                destination,
                source_table=source_table,
                duration_s=duration_s,
                max_steps=max_steps,
            )
        )
    manifest = {
        "schema": "taoryx.x15-native-reachability-replay/v1alpha1",
        "source_envelope": str(Path(envelope_path)),
        "source_problem": _SOURCE_PROBLEM,
        "source_tables": [_SOURCE_TABLE],
        "selection": {
            "max_points": max_points,
            "selected_query_ids": [record.query_id for record in records],
            "policy": "one nearest-feasible-boundary and one nearest-infeasible-boundary sample, then input order",
        },
        "claim_boundary": "Selective native rigid-body replay from a reduced-order checkpoint; not a native batch envelope",
        "records": [record.as_dict() for record in records],
    }
    manifest_path = destination / "replay-manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return X15NativeReplayBundle(tuple(records), manifest_path)


def _select_samples(payload: dict[str, Any], max_points: int) -> list[dict[str, Any]]:
    samples = [item for item in payload.get("samples", []) if isinstance(item, dict)]
    if not samples:
        raise ValueError("reachability artifact has no samples")
    feasible = [item for item in samples if item.get("classification") == "feasible"]
    infeasible = [item for item in samples if item.get("classification") != "feasible"]
    selected: list[dict[str, Any]] = []
    for group in (feasible, infeasible):
        replayable = [item for item in group if _can_enter_native_domain(item)]
        candidates = replayable or group
        if candidates and len(selected) < max_points:
            selected.append(min(candidates, key=_boundary_distance))
    for sample in samples:
        if len(selected) >= max_points:
            break
        if sample not in selected:
            selected.append(sample)
    return selected


def _boundary_distance(sample: dict[str, Any]) -> float:
    margins = sample.get("terminal_margins", {})
    if not isinstance(margins, dict):
        return math.inf
    values = [abs(float(value)) for name, value in margins.items() if name.endswith("margin_m_s") and value is not None]
    return min(values, default=math.inf)


def _replay_sample(
    sample: dict[str, Any],
    payload: dict[str, Any],
    destination: Path,
    *,
    source_table: Path,
    duration_s: float,
    max_steps: int,
) -> X15NativeReplayRecord:
    query_id = str(sample.get("query_id", "query-unknown"))
    try:
        checkpoint = _glide_checkpoint(sample)
    except ValueError as error:
        return X15NativeReplayRecord(
            query_id=query_id,
            reduced_classification=str(sample.get("classification", "unknown")),
            reduced_fidelity=str(payload.get("fidelity", payload.get("study", {}).get("fidelity", "unknown"))),
            generated_problem="",
            native_artifact=None,
            completed=False,
            diagnostics=(
                {
                    "code": "native-table-envelope-unreachable",
                    "message": str(error),
                },
            ),
            bridge={"native_table_max_altitude_m": _NATIVE_TABLE_MAX_ALTITUDE_M},
            comparison={
                "reduced_release_speed_m_s": None,
                "native_replay_terminal_speed_m_s": None,
                "native_replay_terminal_mass_kg": None,
            },
        )
    bridge = _checkpoint_bridge(checkpoint)
    problem_path = destination / f"{query_id}-native.prb"
    problem_path.write_text(_native_problem_text(bridge, duration_s), encoding="utf-8")
    run_directory = destination / f"{query_id}-run"
    report = run_files(
        problem_path,
        (source_table,),
        output_dir=run_directory,
        max_steps=max_steps,
        profile="taoryx",
    )
    artifact_path: Path | None = None
    comparison: dict[str, float | None] = {
        "reduced_release_speed_m_s": _checkpoint_speed(checkpoint),
        "native_replay_terminal_speed_m_s": None,
        "native_replay_terminal_mass_kg": None,
    }
    if report.artifacts:
        artifact_path = run_directory / "native-run-artifact.json"
        report.artifacts[0].write_json(artifact_path)
        telemetry = report.artifacts[0].vehicles.get("1")
        if telemetry is not None:
            comparison["native_replay_terminal_speed_m_s"] = _telemetry_speed(telemetry)
            comparison["native_replay_terminal_mass_kg"] = _telemetry_last(telemetry, "taos.mass_kg") or _telemetry_last(telemetry, "mass.total")
    diagnostics: tuple[dict[str, object], ...] = tuple(
        {"code": item.code, "message": item.message} for item in report.diagnostics
    )
    return X15NativeReplayRecord(
        query_id=query_id,
        reduced_classification=str(sample.get("classification", "unknown")),
        reduced_fidelity=str(payload.get("fidelity", payload.get("study", {}).get("fidelity", "unknown"))),
        generated_problem=str(problem_path),
        native_artifact=str(artifact_path) if artifact_path is not None else None,
        completed=report.exit_code == 0,
        diagnostics=diagnostics,
        bridge=bridge,
        comparison=comparison,
    )


def _glide_checkpoint(sample: dict[str, Any]) -> dict[str, Any]:
    trajectory = sample.get("trajectory")
    if not isinstance(trajectory, dict):
        raise ValueError("native replay requires trajectory tables in the envelope artifact")
    fields = trajectory.get("fields")
    rows = trajectory.get("rows")
    if not isinstance(fields, list) or not isinstance(rows, list):
        raise ValueError("native replay trajectory table is malformed")
    glide_rows: list[dict[str, Any]] = []
    for row in rows:
        values = dict(zip((str(field) for field in fields), row, strict=False))
        if values.get("phase") == "glide":
            glide_rows.append(values)
    if not glide_rows:
        raise ValueError("native replay requires a glide checkpoint")
    for checkpoint in glide_rows:
        if 0.0 <= float(checkpoint["z_m"]) <= _NATIVE_REPLAY_SAFE_ALTITUDE_M:
            return checkpoint
    raise ValueError(
        "glide trajectory never entered the native aerodynamic table envelope "
        f"[0, {_NATIVE_REPLAY_SAFE_ALTITUDE_M}] m"
    )


def _can_enter_native_domain(sample: dict[str, Any]) -> bool:
    try:
        _glide_checkpoint(sample)
    except (KeyError, TypeError, ValueError):
        return False
    return True


def _checkpoint_bridge(checkpoint: dict[str, Any]) -> dict[str, object]:
    position = tuple(float(checkpoint[name]) for name in ("x_m", "y_m", "z_m"))
    velocity = tuple(float(checkpoint[name]) for name in ("vx_m_s", "vy_m_s", "vz_m_s"))
    reduced_attitude = tuple(float(checkpoint.get(name, 0.0)) for name in ("roll_rad", "pitch_rad", "yaw_rad"))
    reduced_velocity_ecic = (velocity[2], velocity[0], velocity[1])
    speed = _speed(reduced_velocity_ecic) or 0.0
    native_velocity_ecic = (0.0, speed, 0.0)
    quaternion = (0.5, -0.5, -0.5, 0.5)
    return {
        "local_frame": "x-downrange/y-crossrange/z-up tangent frame",
        "ecic_origin_m": [_EARTH_RADIUS_M, 0.0, 0.0],
        "position_ecic_m": [_EARTH_RADIUS_M + position[2], position[0], position[1]],
        "velocity_ecic_m_s": list(native_velocity_ecic),
        "reduced_velocity_ecic_m_s": list(reduced_velocity_ecic),
        "attitude_quaternion_wxyz": list(quaternion),
        "reduced_attitude_euler_rad": list(reduced_attitude),
        "mass_kg": float(checkpoint["mass_kg"]),
        "propellant_mass_kg": 0.0,
        "checkpoint_time_s": float(checkpoint["time_s"]),
        "bridge_claim": "local reduced-order checkpoint embedded in an ECIC tangent-frame surrogate; native replay uses the source-aligned release attitude and tangential velocity policy, not native attitude reconstruction",
    }


def _native_problem_text(bridge: dict[str, object], duration_s: float) -> str:
    position = cast(list[float], bridge["position_ecic_m"])
    velocity = cast(list[float], bridge["velocity_ecic_m_s"])
    quaternion = cast(list[float], bridge["attitude_quaternion_wxyz"])
    return f"""(x15-native-reachability-replay)
*title Selective X-15 native rigid-body replay from a reduced-order checkpoint
*mode rigid-body-6dof
*atmos standard
*earth wgs-84 omega=7.2921151467e-5 gm=3.986004418e14
*runtime status vehicle reference-area=18.580608 reference-length=3.130296 dry-mass-kg=9000.0 nominal-mass-kg=14641.0545 inertia-x=4948.7355 inertia-y=129114.2666 inertia-z=131825.9024
*runtime control bank-deg vehicle=1 default=0.0 lower=-25.0 upper=25.0
*runtime status actuator maximum-moment=250000 maximum-body-rate-deg-s=3600
*trajectory 1 x15 start on 1
  *initial ecic x={position[0]} y={position[1]} z={position[2]} xdt={velocity[0]} ydt={velocity[1]} zdt={velocity[2]} qw={quaternion[0]} qx={quaternion[1]} qy={quaternion[2]} qz={quaternion[3]} wx=0.0 wy=0.0 wz=0.0 time=0.0 mass={bridge["mass_kg"]} propellant_mass=0.0
  *segment 1 release-glide-replay
    *integ dtprnt=0.01 dt=0.001
    *prop thrust=0 mdot=0
    *aero cx=(cx) cy=(cy) cz=(cz) cmx=(cmx) cmy=(cmy) cmz=(cmz)
    *fly bankgc=0
    *when time>{duration_s} stop
*end
"""


def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _align_body_x_to_velocity(velocity: tuple[float, float, float]) -> tuple[float, float, float, float]:
    length = math.sqrt(sum(value * value for value in velocity))
    if length <= 1.0e-12:
        return (1.0, 0.0, 0.0, 0.0)
    target = tuple(value / length for value in velocity)
    dot = max(-1.0, min(1.0, target[0]))
    if dot > 1.0 - 1.0e-12:
        return (1.0, 0.0, 0.0, 0.0)
    if dot < -1.0 + 1.0e-12:
        return (0.0, 0.0, 1.0, 0.0)
    axis = (0.0, -target[2], target[1])
    scale = math.sqrt(2.0 * (1.0 + dot))
    return (0.5 * scale, axis[0] / scale, axis[1] / scale, axis[2] / scale)


def _speed(values: object) -> float | None:
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        return None
    return math.sqrt(sum(float(value) ** 2 for value in values))


def _checkpoint_speed(checkpoint: dict[str, Any]) -> float | None:
    values = tuple(checkpoint.get(name) for name in ("vx_m_s", "vy_m_s", "vz_m_s"))
    return _speed(values) if all(value is not None for value in values) else None


def _telemetry_speed(telemetry: Any) -> float | None:
    components = [_telemetry_last(telemetry, name) for name in ("taos.vx", "taos.vy", "taos.vz")]
    return _speed(components) if all(value is not None for value in components) else None


def _telemetry_last(telemetry: Any, name: str) -> float | None:
    channel = telemetry.channels.get(name)
    if channel is None or not channel.values or channel.values[-1] is None:
        return None
    return float(channel.values[-1])


__all__ = ["X15NativeReplayBundle", "X15NativeReplayRecord", "write_x15_native_boundary_replay"]
