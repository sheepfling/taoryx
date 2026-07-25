from __future__ import annotations

import hashlib
from pathlib import Path

import pytest
import yaml

from taoryx.contracts import Frame, FrameVector3, Vector3
from taoryx.modes import DynamicsMode, Kinematic6DofState
from taoryx.runtime import (
    ControlSpec,
    EnvironmentKeyframe,
    EnvironmentSample,
    EventCondition,
    HypersonicGlideTerminal,
    InteractiveSession,
    RacetrackController,
    RacetrackSpec,
    RuntimeProblem,
    RuntimeState,
    RuntimeVehicle,
    ScheduledEnvironmentProvider,
    WindCompensatedPursuit,
)
from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[1].parent / "examples" / "mission_families"

pytestmark = pytest.mark.simple_aero


def test_hypersonic_glide_guidance_runs_with_two_vehicle_target(tmp_path: Path) -> None:
    family = ROOT / "hypersonic_glide_terminal_guidance"
    report = run_files(
        family / "mission.prb",
        (family / "aero.tbl",),
        output_dir=tmp_path,
        max_steps=10_000,
    )

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    assert len(report.results) == 1
    result = report.results[0]
    assert result.completed
    assert set(result.states) == {"1", "2"}
    glider = result.states["1"]
    target = result.states["2"]
    assert glider[0].named["mach"] > 5.0
    assert min(state.named["relrng[2]"] for state in glider) <= 1_000.0
    assert glider[-1].named["relrng[2]"] <= 1_000.0
    assert int(glider[-1].named["_segment"]) == 2
    assert all(state.named["alt"] == state.named["alt"] for state in glider)
    assert target[-1].time >= glider[-1].time
    ####


def test_tumbling_ballistic_launch_runs_from_generated_analysis_table(tmp_path: Path) -> None:
    generated = Path(__file__).resolve().parents[1].parent / "analysis" / "tumbling" / "generated"
    family = ROOT / "suborbital_ballistic_tumble_return"
    report = run_files(
        family / "mission.prb",
        (generated / "cone_cd.tbl",),
        output_dir=tmp_path,
        max_steps=20_000,
    )

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    result = report.results[0]
    assert result.completed
    history = result.states["1"]
    assert history[0].named["alt"] == pytest.approx(1.0)
    assert max(state.named["alt"] for state in history) > 50_000.0
    assert history[-1].named["alt"] < 1.0
    assert max(state.named["dynprs"] for state in history) > 0.0
    ####


def test_suborbital_tumble_provenance_hashes_match_inputs() -> None:
    manifest_path = ROOT / "suborbital_ballistic_tumble_return" / "provenance.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    root = Path(__file__).resolve().parents[1].parent
    for record in (*manifest["sources"], *manifest["artifacts"]):
        path = root / record["path"]
        assert path.exists(), path
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        assert digest == record["sha256"], path
    ####


def test_wind_compensated_missile_tracks_a_moving_target(tmp_path: Path) -> None:
    family = ROOT / "wind_compensated_cruise_missile"
    report = run_files(
        family / "mission.prb",
        (family / "tables.tbl",),
        output_dir=tmp_path,
        max_steps=10_000,
    )

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    result = report.results[0]
    assert result.completed
    missile = result.states["1"]
    assert missile[0].named["vair"] != pytest.approx(missile[0].named["vel"])
    assert min(state.named["relrng[2]"] for state in missile) < missile[0].named["relrng[2]"]
    assert max(state.named["thrust"] for state in missile) > 0.0
    ####


def test_wind_missile_has_a_zero_wind_metamorphic_baseline(tmp_path: Path) -> None:
    family = ROOT / "wind_compensated_cruise_missile"
    zero_wind_problem = tmp_path / "zero-wind.prb"
    zero_wind_problem.write_text(
        (family / "mission.prb").read_text(encoding="utf-8").replace("*wind geodetic windv=30 windh=90 windd=0\n", ""),
        encoding="utf-8",
    )
    report = run_files(zero_wind_problem, (family / "tables.tbl",), output_dir=tmp_path / "zero", max_steps=10_000)

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    missile = report.results[0].states["1"]
    assert missile[0].named["vair"] == pytest.approx(missile[0].named["vel"])
    ####


def test_extension_pursuit_controller_hits_a_slowly_moving_target_with_wind() -> None:
    wind = Vector3(30.0, 0.0, 0.0)
    controller = WindCompensatedPursuit(airspeed=200.0, prediction_time=1.0, capture_gain=4.0)
    target_start = Vector3(1000.0, 200.0, 0.0)
    target_velocity = Vector3(20.0, 0.0, 0.0)

    def target_position(time: float) -> Vector3:
        return target_start + target_velocity.scaled(time)
    ####

    def derivative(state: RuntimeState) -> tuple[float, float, float, float]:
        return (
            state.named.get("vx", 0.0),
            state.named.get("vy", 0.0),
            (state.named.get("air_x", 0.0) + wind.x - state.named.get("vx", 0.0)) / 0.5,
            (state.named.get("air_y", 0.0) + wind.y - state.named.get("vy", 0.0)) / 0.5,
        )
    ####

    def in_range(state: RuntimeState) -> bool:
        return (target_position(state.time) - Vector3(state.values[0], state.values[1], 0.0)).norm() <= 10.0
    ####

    vehicle = RuntimeVehicle(
        "missile-extension",
        RuntimeState(0.0, (0.0, 0.0, 0.0, 0.0), value_names=("x", "y", "vx", "vy")),
        derivative=derivative,
        step_size=0.1,
        events=(EventCondition("target-hit", lambda state: 10.0 if in_range(state) else -1.0, predicate=in_range),),
    )
    session = InteractiveSession(
        RuntimeProblem({vehicle.name: vehicle}),
        controls=(
            ControlSpec("air_x", unit="m/s", lower=-400.0, upper=400.0),
            ControlSpec("air_y", unit="m/s", lower=-400.0, upper=400.0),
            ControlSpec("throttle", unit="fraction", lower=0.0, upper=3.0, default=1.0),
        ),
    )

    for _ in range(400):
        state = vehicle.state
        command = controller.command(Vector3(state.values[0], state.values[1], 0.0), target_position(state.time), target_velocity, wind)
        session.step(0.1, {"air_x": command.air_velocity.x, "air_y": command.air_velocity.y, "throttle": command.throttle})
        if session.status.value == "completed":
            break

    final_state = vehicle.state
    assert session.status.value == "completed"
    assert in_range(final_state)
    assert final_state.time < 40.0
    ####


def test_extension_hypersonic_glide_hits_a_ground_target_with_bounded_descent() -> None:
    wind = Vector3(40.0, -20.0, 0.0)
    controller = HypersonicGlideTerminal(airspeed=7000.0, maximum_flight_path_angle_radians=0.5)
    target_start = Vector3(300_000.0, 10_000.0, 0.0)
    target_velocity = Vector3(100.0, 0.0, 0.0)

    def target_position(time: float) -> Vector3:
        return target_start + target_velocity.scaled(time)
    ####

    def derivative(state: RuntimeState) -> tuple[float, float, float, float, float, float]:
        return (
            state.named.get("vx", 0.0),
            state.named.get("vy", 0.0),
            state.named.get("vz", 0.0),
            (state.named.get("air_x", 0.0) + wind.x - state.named.get("vx", 0.0)) / 1.0,
            (state.named.get("air_y", 0.0) + wind.y - state.named.get("vy", 0.0)) / 1.0,
            (state.named.get("air_z", 0.0) + wind.z - state.named.get("vz", 0.0)) / 1.0,
        )
    ####

    def in_range(state: RuntimeState) -> bool:
        return (target_position(state.time) - Vector3(state.values[0], state.values[1], state.values[2])).norm() <= 100.0
    ####

    vehicle = RuntimeVehicle(
        "glide-extension",
        RuntimeState(0.0, (0.0, 0.0, 80_000.0, 0.0, 0.0, 0.0), value_names=("x", "y", "z", "vx", "vy", "vz")),
        derivative=derivative,
        step_size=0.1,
        events=(EventCondition("ground-target-hit", lambda state: 100.0 if in_range(state) else -1.0, predicate=in_range),),
    )
    session = InteractiveSession(
        RuntimeProblem({vehicle.name: vehicle}),
        controls=(
            ControlSpec("air_x", unit="m/s", lower=-8000.0, upper=8000.0),
            ControlSpec("air_y", unit="m/s", lower=-8000.0, upper=8000.0),
            ControlSpec("air_z", unit="m/s", lower=-8000.0, upper=8000.0),
        ),
    )

    commanded_angles: list[float] = []
    for _ in range(1_000):
        state = vehicle.state
        command = controller.command(Vector3(state.values[0], state.values[1], state.values[2]), target_position(state.time), target_velocity, wind)
        commanded_angles.append(command.flight_path_angle_radians)
        session.step(0.1, {"air_x": command.air_velocity.x, "air_y": command.air_velocity.y, "air_z": command.air_velocity.z})
        if session.status.value == "completed":
            break

    assert session.status.value == "completed"
    assert in_range(vehicle.state)
    assert max(abs(angle) for angle in commanded_angles) <= 0.5
    assert vehicle.state.time < 100.0
    ####


def test_drone_racetrack_tracks_through_a_changing_environment() -> None:
    spec = RacetrackSpec(straight_length=200.0, turn_radius=50.0, speed=20.0)
    controller = RacetrackController(spec, position_gain=0.8)
    def sample(wind_x: float, wind_y: float) -> EnvironmentSample:
        return EnvironmentSample(1.2, 101325.0, 288.15, 340.0, FrameVector3(Vector3(wind_x, wind_y, 0.0), Frame.ECFC), humidity=0.4)
    provider = ScheduledEnvironmentProvider((EnvironmentKeyframe(0.0, sample(0.0, 0.0)), EnvironmentKeyframe(40.0, sample(8.0, -5.0))))
    initial = controller.reference(0.0).position

    def derivative(state: RuntimeState) -> tuple[float, float]:
        position = Vector3(state.values[0], state.values[1], 0.0)
        environment = provider.sample(time=state.time, position=FrameVector3(position, Frame.ECFC))
        return (
            state.named.get("air_x", 0.0) + environment.wind.vector.x,
            state.named.get("air_y", 0.0) + environment.wind.vector.y,
        )
    ####

    vehicle = RuntimeVehicle(
        "drone",
        RuntimeState(0.0, (initial.x, initial.y), value_names=("x", "y")),
        derivative=derivative,
        step_size=0.1,
        dynamics_mode=DynamicsMode.KINEMATIC_6DOF,
        kinematic_state=Kinematic6DofState(
            0.0,
            FrameVector3(initial, Frame.ECFC),
            FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
        ),
    )
    session = InteractiveSession(
        RuntimeProblem({"drone": vehicle}),
        controls=(
            ControlSpec("air_x", unit="m/s", lower=-100.0, upper=100.0),
            ControlSpec("air_y", unit="m/s", lower=-100.0, upper=100.0),
        ),
    )
    dt = 0.1
    for index in range(400):
        time = index * dt
        state = vehicle.state
        position = Vector3(state.values[0], state.values[1], 0.0)
        environment = provider.sample(time=time, position=FrameVector3(position, Frame.ECFC))
        command = controller.command(time, position, environment.wind.vector)
        session.step(dt, {"air_x": command.air_velocity.x, "air_y": command.air_velocity.y})
    final = controller.command(40.0, position, provider.sample(time=40.0, position=FrameVector3(position, Frame.ECFC)).wind.vector)

    assert session.time == pytest.approx(40.0)
    assert session.status.value == "running"
    assert final.lap >= 1
    assert final.cross_track_error < 20.0
    assert provider.sample(time=20.0, position=FrameVector3(position, Frame.ECFC)).wind.vector == Vector3(4.0, -2.5, 0.0)
    ####
