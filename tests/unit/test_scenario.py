from __future__ import annotations

import json
from pathlib import Path

import pytest

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.common import RuntimeProblem, RuntimeState, RuntimeVehicle
from taoryx.scenario import (
    ControlContract,
    InheritanceOverride,
    InitialFrameState,
    InitialValueOverride,
    MassAdjustment,
    OutputContract,
    RandomSeed,
    ResolvedScenario,
    ScenarioCompileError,
    ScenarioCompiler,
    ScenarioRuntimeContract,
    VelocityImpulse,
    _apply_runtime_patches,
)

ROOT = Path(__file__).resolve().parents[2]
PROBLEM = ROOT / "examples/chapter04/ballistic-reentry.prb"
TABLE = ROOT / "examples/chapter04/ballistic-reentry.tbl"

pytestmark = pytest.mark.algorithms


def test_compiler_records_profile_sources_and_deterministic_identity(tmp_path: Path) -> None:
    compiler = ScenarioCompiler()
    first = compiler.compile(PROBLEM, table_paths=(TABLE,), profile=GrammarProfile.TAOS96, seed=1729, integrator="rk4")
    second = compiler.compile(PROBLEM, table_paths=(TABLE,), profile=GrammarProfile.TAOS96, seed=1729, integrator="rk4")

    assert first.identity == second.identity
    assert first.sources[0].sha256
    assert first.table_names == ("ca-ex-1",)
    assert first.request.profile is GrammarProfile.TAOS96

    path = first.write_json(tmp_path / "scenario.json")
    assert ResolvedScenario.read_json(path) == first
    assert json.loads(path.read_text(encoding="utf-8"))["schema_version"] == 1
    ####


def test_random_seed_patch_is_part_of_resolved_identity() -> None:
    first = ScenarioCompiler().compile(PROBLEM, table_paths=(TABLE,), seed=1729, patches=(RandomSeed(seed=7),))
    second = ScenarioCompiler().compile(PROBLEM, table_paths=(TABLE,), seed=1729, patches=(RandomSeed(seed=8),))

    assert first.request.seed == 7
    assert first.identity != second.identity
    records = first.lower().cases[0].problem.metadata["composition_records"]
    assert records[0]["target"] == "seed"
    ####


def test_runtime_contract_projects_into_interactive_and_batch_artifacts(tmp_path: Path) -> None:
    runtime = ScenarioRuntimeContract(
        controls=(ControlContract(name="throttle", unit="fraction", lower=0.0, upper=1.0),),
        outputs=(OutputContract(channels=("position.altitude.geodetic",), sample_interval=0.5),),
    )
    scenario = ScenarioCompiler().compile(PROBLEM, table_paths=(TABLE,), runtime=runtime)

    session = scenario.interactive_session()
    assert session.controls[0].name == "throttle"
    assert session.output_subscriptions[0].sample_interval == pytest.approx(0.5)
    artifact = scenario.run(output_dir=tmp_path / "run", max_steps=20_000)[0]
    assert artifact.visualization["runtime"] == runtime.model_dump(mode="json")
    sampling = artifact.visualization["output_sampling"]
    assert sampling["sample_interval"] == pytest.approx(0.5)
    assert sampling["channels"] == ["position.altitude.geodetic"]
    assert set(artifact.vehicles["1"].channels) == {"position.altitude.geodetic"}
    ####


def test_runtime_declarations_in_problem_file_project_into_contract(tmp_path: Path) -> None:
    source = tmp_path / "runtime.prb"
    source.write_text(
        PROBLEM.read_text(encoding="utf-8").replace(
            "*end",
            """*runtime control throttle unit=fraction default=0.5 lower=0 upper=1
*runtime status altitude source=alt unit=m
*runtime event ground condition=alt<0 action=stop
*runtime output channels=alt,vel interval=0.5 events=true
*end""",
            1,
        ),
        encoding="utf-8",
    )

    scenario = ScenarioCompiler().compile(source, table_paths=(TABLE,), profile=GrammarProfile.TAORYX)

    assert scenario.request.runtime.controls[0].name == "throttle"
    assert scenario.request.runtime.statuses[0].source == "alt"
    assert scenario.request.runtime.events[0].action == "stop"
    assert scenario.request.runtime.outputs[0].channels == ("alt", "vel")
    assert scenario.lower().cases[0].problem.vehicles["1"].state.named["throttle"] == pytest.approx(0.5)
    session = scenario.interactive_session()
    assert session.event_specs[0].name == "ground"
    assert session.output_subscriptions[0].sample_interval == pytest.approx(0.5)
    ####


def test_runtime_stop_event_is_applied_by_batch_execution(tmp_path: Path) -> None:
    source = tmp_path / "runtime-stop.prb"
    source.write_text(
        PROBLEM.read_text(encoding="utf-8").replace(
            "*end",
            "*runtime event immediate condition=time>=0 action=stop\n*end",
            1,
        ),
        encoding="utf-8",
    )

    artifact = ScenarioCompiler().compile(source, table_paths=(TABLE,), profile=GrammarProfile.TAORYX).run(output_dir=tmp_path / "run", max_steps=20_000)[0]

    assert artifact.events
    assert artifact.events[0]["name"] == "runtime-immediate"
    assert artifact.events[0]["action"] == "stop"
    assert str(source) in str(artifact.events[0]["source"])
    ####


def test_runtime_throttle_scales_point_mass_propulsion(tmp_path: Path) -> None:
    source = tmp_path / "throttle.prb"
    source.write_text(
        """(throttle)
*atmos none
*runtime control throttle default=0.5 lower=0 upper=1
*trajectory 1 T start on 1
  *initial geodetic
    long=0 lat=0 alt=1000 vel=100 gama=0 psi=0 wt=100
  *segment 1 S
    *integ dt=0.1
    *prop thrust=100 mdot=2
    *when alt<0 stop
*end
""",
        encoding="utf-8",
    )

    scenario = ScenarioCompiler().compile(source, profile=GrammarProfile.TAORYX)
    vehicle = scenario.lower().cases[0].problem.vehicles["1"]
    rates = dict(zip(vehicle.state.value_names, vehicle.derivative(vehicle.state), strict=True))

    assert vehicle.state.named["throttle"] == pytest.approx(0.5)
    assert rates["wt"] == pytest.approx(-1.0)
    ####


def test_cache_reuses_preparsed_documents_without_reingestion(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    cache = tmp_path / "scenario.json"
    compiler = ScenarioCompiler()
    first = compiler.load_or_compile(cache, problem_path=PROBLEM, table_paths=(TABLE,))
    assert first.parsed_problem is not None
    assert first.parsed_tables

    def fail_ingest(*args: object, **kwargs: object) -> object:
        raise AssertionError("cache hit unexpectedly re-ingested source")

    monkeypatch.setattr("taoryx.scenario.ingest_file", fail_ingest)
    cached = compiler.load_or_compile(cache, problem_path=PROBLEM, table_paths=(TABLE,))
    assert cached.identity == first.identity
    assert cached.lower().cases
    ####


def test_cache_rebuilds_when_source_bytes_change(tmp_path: Path) -> None:
    problem = tmp_path / "problem.prb"
    problem.write_text(PROBLEM.read_text(encoding="utf-8"), encoding="utf-8")
    cache = tmp_path / "scenario.json"
    compiler = ScenarioCompiler()

    first = compiler.load_or_compile(cache, problem_path=problem, table_paths=(TABLE,))
    cached = compiler.load_or_compile(cache, problem_path=problem, table_paths=(TABLE,))
    assert cached == first

    problem.write_text(problem.read_text(encoding="utf-8") + "\n# source revision\n", encoding="utf-8")
    rebuilt = compiler.load_or_compile(cache, problem_path=problem, table_paths=(TABLE,))
    assert rebuilt.identity != first.identity
    ####


def test_compiler_preserves_validation_diagnostics(tmp_path: Path) -> None:
    problem = tmp_path / "invalid.prb"
    problem.write_text("(invalid)\n*unsupported\n*end\n", encoding="utf-8")

    with pytest.raises(ScenarioCompileError) as raised:
        ScenarioCompiler().compile(problem)

    assert raised.value.diagnostics
    assert all(item.location is not None for item in raised.value.diagnostics)
    ####


def test_resolved_scenario_applies_geodetic_initial_state_and_impulse() -> None:
    scenario = ScenarioCompiler().compile(
        PROBLEM,
        table_paths=(TABLE,),
        patches=(
            InitialFrameState(
                vehicle="1",
                frame="geodetic",
                position=(-80.6, 28.5, 30_000.0),
                velocity=(100.0, 5.0, 90.0),
            ),
            VelocityImpulse(vehicle="1", frame="geodetic", delta_velocity=(0.0, 0.0, 12.0)),
            MassAdjustment(vehicle="1", delta_mass=-5.0),
        ),
    )

    lowered = scenario.lower()
    state = lowered.cases[0].problem.vehicles["1"].state
    assert state.named["long"] == pytest.approx(-80.6)
    assert state.named["lat"] == pytest.approx(28.5)
    assert state.named["alt"] == pytest.approx(30_000.0)
    assert state.named["vel"] > 100.0
    assert state.named["mass"] == pytest.approx(545.0)
    assert [record["kind"] for record in lowered.cases[0].problem.metadata["composition_records"]] == [
        "initial-frame",
        "velocity-impulse",
        "mass",
    ]
    ####


def test_ecfc_initial_state_round_trips_to_geodetic_aliases() -> None:
    scenario = ScenarioCompiler().compile(
        PROBLEM,
        table_paths=(TABLE,),
        patches=(
            InitialFrameState(
                vehicle="1",
                frame="ecfc",
                position=(20_925_646.3255, 0.0, 0.0),
                velocity=(0.0, 100.0, 0.0),
            ),
        ),
    )

    state = scenario.lower().cases[0].problem.vehicles["1"].state
    assert state.named["long"] == pytest.approx(0.0)
    assert state.named["lat"] == pytest.approx(0.0)
    assert state.named["alt"] == pytest.approx(0.0)
    assert state.named["vel"] == pytest.approx(100.0)
    ####


def test_scalar_composition_units_convert_to_runtime_canonical_units() -> None:
    scenario = ScenarioCompiler().compile(
        PROBLEM,
        table_paths=(TABLE,),
        patches=(InitialValueOverride(vehicle="1", field="mass", value=1.0, unit="kg"),),
    )

    state = scenario.lower().cases[0].problem.vehicles["1"].state
    assert state.named["mass"] == pytest.approx(2.20462262185)
    ####


def test_body_impulse_requires_attitude_provider() -> None:
    scenario = ScenarioCompiler().compile(
        PROBLEM,
        table_paths=(TABLE,),
        patches=(VelocityImpulse(vehicle="1", frame="body", delta_velocity=(1.0, 0.0, 0.0)),),
    )

    with pytest.raises(ValueError, match="attitude provider"):
        scenario.lower()
    ####


def test_inheritance_patch_sets_dependency_and_activation_contract() -> None:
    source = RuntimeVehicle("source", RuntimeState(0.0, (10.0,)))
    target = RuntimeVehicle("target", RuntimeState(0.0, (0.0,)))
    problem = RuntimeProblem({source.name: source, target.name: target})

    _apply_runtime_patches(
        problem,
        (InheritanceOverride(vehicle="target", source_vehicle="source", source_segment=2),),
    )

    assert target.dependencies == ("source",)
    assert target.dependency_segments == {"source": 2}
    assert not target.active
    assert target.activation_pending
    ####
