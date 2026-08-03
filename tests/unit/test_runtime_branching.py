"""Source-to-runtime execution and branch/snapshot coverage."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from taoryx.contracts import Frame, FrameVector3, Vector3
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language.ingest import ingest_file
from taoryx.language.models import ProblemDocument
from taoryx.modes import DynamicsMode, Kinematic6DofState, Quaternion
from taoryx.runtime.common import RuntimeProblem, RuntimeState, RuntimeVehicle
from taoryx.runtime.engine import compute_trajectories
from taoryx.runtime.interactive import ControlSpec, InteractiveSession
from taoryx.runtime.lowering import lower_problem_document
from taoryx.runtime.program import LoadedProgram
from taoryx.runtime.runner import run_files
from taoryx.runtime.sensor_scenario import SensorScenarioSpec


def test_runtime_problem_clone_interpolates_and_branches_independently() -> None:
    vehicle = RuntimeVehicle(
        "test",
        RuntimeState(0.0, (0.0,), named={"x": 0.0, "stash": 0.0}, value_names=("x",)),
        derivative=lambda state: (state.named["rate"],),
        step_size=1.0,
        control_values={"rate": 2.0},
    )
    problem = RuntimeProblem({"test": vehicle}, final_time=3.0)
    compute_trajectories(problem, max_steps=4)

    branch = problem.clone_at(1.5)
    assert branch is not problem
    assert branch.vehicles["test"] is not vehicle
    assert branch.vehicles["test"].state.time == pytest.approx(1.5)
    assert branch.vehicles["test"].state.values == pytest.approx((3.0,))
    assert len(branch.vehicles["test"].history) == 3

    branch.vehicles["test"].control_values = {"rate": 4.0}
    compute_trajectories(branch, max_steps=2)

    assert branch.vehicles["test"].state.values == pytest.approx((9.0,))
    assert vehicle.state.values == pytest.approx((6.0,))
    assert branch.metadata["cloned_at_time"] == pytest.approx(1.5)
####


def test_runtime_problem_clone_never_carries_a_future_control_timestamp() -> None:
    vehicle = RuntimeVehicle(
        "test",
        RuntimeState(0.0, (0.0,), value_names=("x",)),
        derivative=lambda state: (state.named["rate"],),
        step_size=1.0,
        control_values={"rate": 1.0},
    )
    problem = RuntimeProblem({"test": vehicle}, final_time=3.0)
    compute_trajectories(problem, max_steps=4)
    vehicle.control_values = {"rate": 2.0}
    vehicle.control_values_time_s = vehicle.state.time

    branch = problem.clone_at(1.0)

    assert branch.vehicles["test"].control_values_time_s == pytest.approx(1.0)
    compute_trajectories(branch, max_steps=1)
    assert branch.vehicles["test"].state.time == pytest.approx(2.0)
####


def test_problem_file_lowers_custom_stash_and_time_varying_control(tmp_path: Path) -> None:
    source = tmp_path / "varying-controls.prb"
    source.write_text(
        "(varying-controls)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *define scheduled_throttle\n"
        "    if (time < 1) {\n"
        "      scheduled_throttle = 0.25;\n"
        "    } else {\n"
        "      scheduled_throttle = 0.75;\n"
        "    }\n"
        "  *define integral fuel_used=0\n"
        "    fuel_used = scheduled_throttle;\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=10 gama=0 psi=0 time=0 wt=10\n"
        "  *file varying-controls.dat time scheduled_throttle fuel_used\n"
        "  *segment 1 flight\n"
        "    *integ dt=0.5\n"
        "    *prop thrust=10*scheduled_throttle mdot=0\n"
        "    *when time>2 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    ingested = ingest_file(source, profile=GrammarProfile.TAORYX)
    assert isinstance(ingested.document, ProblemDocument)
    assert not [item for item in ingested.diagnostics if item.severity.value == "error"]
    lowered = lower_problem_document(ingested.document)
    vehicle = lowered.cases[0].problem.vehicles["1"]
    result = compute_trajectories(lowered.cases[0].problem, max_steps=20)

    assert result.completed
    history = result.states["1"]
    throttles = [state.named["scheduled_throttle"] for state in history]
    assert min(throttles) == pytest.approx(0.25)
    assert max(throttles) == pytest.approx(0.75)
    assert history[-1].named["fuel_used"] > 0.75
    assert vehicle.state.time == pytest.approx(2.0)
####


def test_source_file_runner_emits_declared_custom_stash_output(tmp_path: Path) -> None:
    source = tmp_path / "source-run.prb"
    source.write_text(
        "(source-run)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *define accumulated=0\n"
        "    if (time < 0.5) {\n"
        "      accumulated = 1;\n"
        "    } else {\n"
        "      accumulated = 2;\n"
        "    }\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=10 gama=0 psi=0 time=0 wt=1\n"
        "  *file source-run.dat time accumulated\n"
        "  *segment 1 flight\n"
        "    *integ dt=0.25\n"
        "    *when time>0.75 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(source, output_dir=tmp_path / "out", max_steps=20)

    assert report.exit_code == 0
    output = (tmp_path / "out" / "source-run.dat").read_text(encoding="utf-8")
    assert "accumulated" in output
    assert "2.0" in output
####


def test_lowered_source_can_be_flown_by_an_external_controller(tmp_path: Path) -> None:
    source = tmp_path / "player-flight.prb"
    source.write_text(
        "(player-flight)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*runtime control throttle default=0 lower=0 upper=1\n"
        "*trajectory 1 player start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=10 gama=0 psi=0 time=0 wt=1\n"
        "  *segment 1 flight\n"
        "    *integ dt=0.1\n"
        "    *prop thrust=10 mdot=0\n"
        "    *when time>0.2 stop\n"
        "*end\n",
        encoding="utf-8",
    )
    ingested = ingest_file(source, profile=GrammarProfile.TAORYX)
    assert isinstance(ingested.document, ProblemDocument)
    lowered = lower_problem_document(ingested.document)
    session = InteractiveSession(
        lowered.cases[0].problem,
        controls=(ControlSpec("throttle", unit="fraction", lower=0.0, upper=1.0),),
    )

    first = session.step(0.1, {"throttle": 0.2})
    second = session.step(0.1, {"throttle": 0.9})

    assert first.commands[0].applied == pytest.approx(0.2)
    assert second.commands[0].applied == pytest.approx(0.9)
    assert second.states["1"].time == pytest.approx(0.2)
    assert second.states["1"].named["vel"] > first.states["1"].named["vel"]
    assert session.to_run_artifact().commands == [
        {"duration": 0.1, "commands": {"throttle": 0.2}},
        {"duration": 0.1, "commands": {"throttle": 0.9}},
    ]
####


def test_loaded_program_inspects_source_and_emulator_graph(tmp_path: Path) -> None:
    source = tmp_path / "inspectable.prb"
    source.write_text(
        "(inspectable)\n"
        "*title Inspectable Flight Program\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*runtime control throttle default=0.5 lower=0 upper=1\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=10 gama=0 psi=0 time=0 wt=1\n"
        "  *segment 1 flight\n"
        "    *integ dt=0.1\n"
        "    *prop thrust=10 mdot=0\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    program = LoadedProgram.load(source, profile=GrammarProfile.TAORYX)
    summary = program.inspect()

    assert summary["grammar_profile"] == "taoryx"
    assert summary["title"] == "Inspectable Flight Program"
    assert summary["metadata"]["problem_name"] == "inspectable"
    assert summary["problems"][0]["trajectories"][0]["segments"][0]["number"] == 1
    assert summary["vehicles"][0]["controls"] == ["throttle"]
    assert program.inspect_controls()[0]["name"] == "throttle"
    assert program.inspect_controls()[0]["lower"] == pytest.approx(0.0)
    assert program.inspect_controls()[0]["current_values"] == {"1": pytest.approx(0.5)}
    assert program.copy_case().vehicles["1"].state.time == pytest.approx(0.0)
    assert program.clone_case_at(0.0).vehicles["1"].state.values == pytest.approx(program.case().vehicles["1"].state.values)
    assert program.inspect_case()["vehicles"]["1"]["source_segment"]["title"] == "flight"
####


def test_loaded_program_can_introspect_and_modify_live_controls(tmp_path: Path) -> None:
    source = tmp_path / "mutable.prb"
    source.write_text(
        "(mutable)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*runtime control throttle default=0.1 lower=0 upper=1\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=10 gama=0 psi=0 time=0 wt=1\n"
        "  *segment 1 flight\n"
        "    *integ dt=0.1\n"
        "    *prop thrust=10 mdot=0\n"
        "    *when time>0.2 stop\n"
        "*end\n",
        encoding="utf-8",
    )
    program = LoadedProgram.load(source, profile=GrammarProfile.TAORYX)

    assert program.set_control("throttle", 2.0) == pytest.approx(1.0)
    before = program.inspect_case()
    assert before["vehicles"]["1"]["controls"]["throttle"] == pytest.approx(1.0)
    program.set_parameter("test-gain", 3.0)
    assert program.inspect_case()["parameters"]["test-gain"] == pytest.approx(3.0)
    with pytest.raises(KeyError, match="unknown runtime control"):
        program.set_control("rudder", 0.2)
####


def test_loaded_program_checkpoint_restarts_from_disk(tmp_path: Path) -> None:
    source = tmp_path / "checkpoint.prb"
    source.write_text(
        "(checkpoint)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=10 gama=0 psi=0 time=0 wt=1\n"
        "  *segment 1 flight\n"
        "    *integ dt=0.5\n"
        "    *prop thrust=10 mdot=0\n"
        "    *when time>2 stop\n"
        "*end\n",
        encoding="utf-8",
    )
    program = LoadedProgram.load(source, profile=GrammarProfile.TAORYX)
    compute_trajectories(program.case(), max_steps=20, stop_when=lambda problem: problem.vehicles["1"].state.time >= 1.0)
    checkpoint = program.save_checkpoint(tmp_path / "flight.checkpoint.json")

    resumed = LoadedProgram.load_checkpoint(checkpoint)
    assert resumed.inspect_case()["vehicles"]["1"]["time"] == pytest.approx(1.0)
    resumed_result = compute_trajectories(resumed.case(), max_steps=20)

    assert resumed_result.completed
    assert resumed_result.states["1"][-1].time == pytest.approx(2.0)
    assert resumed.inspect()["title"] == "checkpoint"
####


def test_loaded_program_checkpoint_reconstructs_declared_sensor_scenario(tmp_path: Path) -> None:
    source = tmp_path / "checkpoint-sensor.prb"
    source.write_text(
        "(checkpoint-sensor)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=10 gama=0 psi=0 time=0 wt=1\n"
        "  *segment 1 flight\n"
        "    *integ dt=0.1\n"
        "    *prop thrust=10 mdot=0\n"
        "    *when time>0.2 stop\n"
        "*end\n",
        encoding="utf-8",
    )
    program = LoadedProgram.load(source, profile=GrammarProfile.TAORYX)
    runtime = program.attach_sensor_scenario(
        SensorScenarioSpec(
            "accel",
            "1",
            provider="translation-acceleration",
            truth_mode="translation-only",
            cadence_s=0.1,
            delivery_s=0.1,
            estimator_modes=("translation-dead-reckoning",),
        )
    )
    compute_trajectories(program.case(), max_steps=1)
    checkpoint = program.save_checkpoint(tmp_path / "sensor.checkpoint.json")

    restored = LoadedProgram.load_checkpoint(checkpoint)

    assert restored.case().sensor_bus is not None
    assert restored.case().metadata["sensor_rebind"] == {
        "required": False,
        "scenario_id": "sensor-scenario-v1",
        "status": "restored-from-checkpoint",
    }
    compute_trajectories(program.case(), max_steps=10)
    compute_trajectories(restored.case(), max_steps=10)
    original_packets = runtime.bus.packets("accel")
    restored_packets = restored.case().sensor_bus.packets("accel")
    assert [packet.sampled_at_s for packet in restored_packets] == pytest.approx(
        [packet.sampled_at_s for packet in original_packets]
    )
####


def test_checkpoint_rejects_source_tampering(tmp_path: Path) -> None:
    source = tmp_path / "integrity.prb"
    source.write_text("(integrity)\n*atmos none\n*trajectory 1 vehicle start on 1\n  *initial geodetic alt=0 long=0 lat=0 vel=1 gama=0 psi=0 wt=1\n  *segment 1 flight\n    *integ dt=0.1\n    *when time>0.1 stop\n*end\n", encoding="utf-8")
    program = LoadedProgram.load(source, profile=GrammarProfile.TAORYX)
    checkpoint = program.save_checkpoint(tmp_path / "integrity.json")
    payload = json.loads(checkpoint.read_text(encoding="utf-8"))
    payload["source"]["problems"][0]["name"] = "tampered"
    checkpoint.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match="source integrity"):
        LoadedProgram.load_checkpoint(checkpoint)
####


def test_kinematic_checkpoint_restores_attitude_sidecar_and_segment_endpoints(tmp_path: Path) -> None:
    source = tmp_path / "kinematic-checkpoint.prb"
    source.write_text(
        "(kinematic-checkpoint)\n"
        "*mode kinematic-6dof\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=10 ydt=0 zdt=0 mass=1 time=0\n"
        "  *segment 1 flight\n"
        "    *integ dt=0.5\n"
        "    *when time>1 stop\n"
        "*end\n",
        encoding="utf-8",
    )
    program = LoadedProgram.load(source, profile=GrammarProfile.TAORYX)
    vehicle = program.case().vehicles["1"]
    assert vehicle.dynamics_mode is DynamicsMode.KINEMATIC_6DOF
    vehicle.kinematic_state = Kinematic6DofState(
        0.0,
        FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
        FrameVector3(Vector3(10.0, 0.0, 0.0), Frame.ECFC),
        Quaternion(0.5, 0.5, 0.5, 0.5).normalized(),
    )
    compute_trajectories(program.case(), max_steps=2)
    checkpoint = program.save_checkpoint(tmp_path / "kinematic.json")

    resumed = LoadedProgram.load_checkpoint(checkpoint)
    restored = resumed.case().vehicles["1"]
    assert restored.kinematic_state is not None
    assert restored.kinematic_state.time == pytest.approx(vehicle.kinematic_state.time)
    assert restored.kinematic_state.position.vector == vehicle.kinematic_state.position.vector
    assert restored.kinematic_state.velocity.vector == vehicle.kinematic_state.velocity.vector
    assert restored.kinematic_state.attitude == vehicle.kinematic_state.attitude
    assert restored.state.segment_endpoints == vehicle.state.segment_endpoints
####


def test_rigid_body_checkpoint_restores_state_and_runtime_settings(tmp_path: Path) -> None:
    source = Path("tests/fixtures/alpha2_case_contracts/t5-rigid-probe.prb")
    program = LoadedProgram.load(source, profile=GrammarProfile.TAORYX)
    compute_trajectories(program.case(), max_steps=2)
    before = program.case().vehicles["1"]
    checkpoint = program.save_checkpoint(tmp_path / "rigid.json")

    resumed = LoadedProgram.load_checkpoint(checkpoint)
    after = resumed.case().vehicles["1"]
    assert after.dynamics_mode is DynamicsMode.RIGID_BODY_6DOF
    assert after.state.time == pytest.approx(before.state.time)
    assert after.state.values == pytest.approx(before.state.values)
    assert after.integrator == before.integrator
    assert after.step_size == pytest.approx(before.step_size)
####


def test_loaded_program_exposes_tiered_standard_status_and_deep_observations(tmp_path: Path) -> None:
    source = tmp_path / "observations.prb"
    source.write_text(
        "(observations)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*runtime status speed source=vel unit=mps\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=1 y=2 z=3 xdt=4 ydt=5 zdt=6 mass=7 time=0\n"
        "  *segment 1 flight\n"
        "    *integ dt=0.1\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )
    program = LoadedProgram.load(source, profile=GrammarProfile.TAORYX)

    observation = program.observe(vehicle="1")
    assert observation.standard.position_ecfc.x == pytest.approx(1.0)
    assert observation.standard.velocity_ecfc.z == pytest.approx(6.0)
    assert observation.status["vel"] == pytest.approx((4.0**2 + 5.0**2 + 6.0**2) ** 0.5)
    assert observation.deep is None
    assert program.observe(vehicle="1", include_deep=True).deep is not None


def test_problem_file_separates_setup_parameters_controls_and_mutable_inputs(tmp_path: Path) -> None:
    source = tmp_path / "runtime_inputs.prb"
    source.write_text(
        "(runtime-inputs)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*runtime parameter initial-mass value=100 unit=kg\n"
        "*runtime parameter guidance-gain value=1.25 unit=1\n"
        "*runtime parameter moving-target-x value=1000 unit=m mutable=true\n"
        "*runtime control throttle vehicle=1 default=0 lower=0 upper=1\n"
        "*runtime status throttle source=throttle unit=fraction\n"
        "*runtime status target-x source=moving-target-x unit=m\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=1 ydt=0 zdt=0 mass=100 time=0\n"
        "  *segment 1 flight\n"
        "    *integ dt=0.1\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )
    program = LoadedProgram.load(source, profile=GrammarProfile.TAORYX, parameter_overrides={"guidance-gain": 2.5})

    parameters = {item["name"]: item for item in program.inspect_parameters()}
    assert parameters["initial-mass"]["value"] == pytest.approx(100.0)
    assert parameters["guidance-gain"]["value"] == pytest.approx(2.5)
    assert parameters["initial-mass"]["mutable"] is False
    assert parameters["moving-target-x"]["mutable"] is True

    assert program.set_control("throttle", 0.75, vehicle="1") == pytest.approx(0.75)
    program.set_parameter("moving-target-x", 1200.0)
    with pytest.raises(ValueError, match="setup-only"):
        program.set_parameter("initial-mass", 80.0)

    observation = program.observe(vehicle="1")
    assert observation.status["throttle"] == pytest.approx(0.75)
    assert observation.status["moving-target-x"] == pytest.approx(1200.0)


def test_problem_file_declares_lqr_contract(tmp_path: Path) -> None:
    source = tmp_path / "lqr.prb"
    source.write_text(
        "(lqr)\n"
        "*atmos none\n"
        "*runtime lqr attitude states=alpha,q controls=elevator linearization=trim q-table=Q r-table=R update=segment\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=1 ydt=0 zdt=0 mass=1 time=0\n"
        "  *segment 1 flight\n"
        "    *integ dt=0.1\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )
    program = LoadedProgram.load(source, profile=GrammarProfile.TAORYX)
    assert program.inspect_lqr() == [{
        "name": "attitude",
        "states": ("alpha", "q"),
        "controls": ("elevator",),
        "q_source": "Q",
        "r_source": "R",
        "linearization_source": "trim",
        "method": "continuous",
        "update": "segment",
    }]
