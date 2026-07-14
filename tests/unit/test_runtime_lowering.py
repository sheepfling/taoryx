from __future__ import annotations

import importlib
import json
import math
from pathlib import Path

import pytest

import taoryx.language._legacy_table as legacy_table
import taoryx.language.grammar_contracts as grammar_contracts
import taoryx.runtime.lowering as lowering_module
from taoryx.language.diagnostics import SourceLocation
from taoryx.language.expressions import CallExpression, IndexedExpression, NameExpression, NumberExpression, ParameterExpression
from taoryx.language.models import (
    Assignment,
    DownrangeCrossrangeBlock,
    FlyBlock,
    FlyPoint,
    OptimizeEndpoint,
    Problem,
    ProblemDocument,
    SearchBlock,
    SearchObjective,
    Segment,
    Trajectory,
)
from taoryx.language.problem_parser import parse_problem_file, parse_problem_text
from taoryx.language.table_parser import parse_table_file, parse_table_text
from taoryx.optimization import OptimizationResult, OptimizationStatus
from taoryx.runtime.cli import main
from taoryx.runtime.common import RuntimeProblem, RuntimeState, RuntimeVehicle
from taoryx.runtime.engine import ExecutionResult, run_taos
from taoryx.runtime.lowering import (
    _adjust_guidance_history_parameters,
    _assemble_ecfc_rates,
    _closure_velocity,
    _coordinate_search,
    _endpoint_value,
    _geodetic_force_rates,
    _optimization_endpoint_requirements,
    _optimization_endpoint_stop_when,
    _standard_atmosphere_properties,
    _table_evaluators,
    _trajectory_observables,
    _unsupported_features,
    lower_problem_document,
    lower_tables,
)
from taoryx.runtime.runner import RunReport, run_files

ROOT = Path(__file__).resolve().parents[2]
PROBLEM = ROOT / "examples/chapter04/ballistic-reentry.prb"
TABLE = ROOT / "examples/chapter04/ballistic-reentry.tbl"


@pytest.fixture(autouse=True)
def _reset_runtime_parser_state() -> None:
    importlib.reload(grammar_contracts)
    importlib.reload(legacy_table)


def test_lowering_expands_surveys_and_prepares_tables() -> None:
    problem = parse_problem_file(PROBLEM)
    table = parse_table_file(TABLE)
    tables = lower_tables(table)
    lowered = lower_problem_document(problem, tables)
    assert len(lowered.cases) == 16
    assert lowered.tables["ca-ex-1"].evaluate({"mach": 8.0}) == 0.0742
    assert lowered.cases[0].problem.vehicles["1"].state.named["vel"] == 15000.0
####


def test_survey_optimization_can_carry_forward_the_previous_optimum(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    problem = tmp_path / "survey-optimize.prb"
    problem.write_text(
        "(survey-optimize)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*survey 1 speed vals=10,20\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=opta-1 ydt=opta-2 zdt=opta-3 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time=1 stop\n"
        "*optimize a for xecfc=max on segment 1, trajectory 1\n"
        "  par-1=3 lo-1=7 hi-1=7 par-2=4 lo-2=8 hi-2=8 par-3=5 lo-3=9 hi-3=9 maxitr=1 surveys=1\n"
        "*end\n",
        encoding="utf-8",
    )
    starts: list[tuple[float, ...]] = []

    class FakeOptimizer:
        def run(self, initial: tuple[float, ...], *, max_iterations: int) -> OptimizationResult:
            starts.append(initial)
            return OptimizationResult((7.0, 8.0, 9.0), 0.0, (), (), 0, OptimizationStatus.CONVERGED)
        ####
    ####

    monkeypatch.setattr(lowering_module, "resolve_optimize_block", lambda *args, **kwargs: FakeOptimizer())
    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    assert report.cases == 2
    assert starts == [(3.0, 4.0, 5.0), (7.0, 8.0, 9.0)]
####


def test_lowered_full_tables_can_reference_other_runtime_tables() -> None:
    document = parse_table_text(
        "(inner)\n"
        "table output\n"
        "start\n"
        "set 4\n"
        "end\n"
        "(outer)\n"
        "table output\n"
        "start\n"
        "add inner()\n"
        "mult 3\n"
        "end\n"
    )
    tables = lower_tables(document)

    assert tables["outer"].evaluate({}, tables) == pytest.approx(12.0)
####


def test_table_evaluators_are_reused_for_one_runtime_table_registry() -> None:
    document = parse_table_text("(gain)\ntable ca(vel)\nvel=0,100\nca=2,4\n")
    tables = lower_tables(document)

    assert _table_evaluators(tables) is _table_evaluators(tables)
####


def test_trajectory_observables_resolve_table_backed_reference_assignments() -> None:
    tables = lower_tables(parse_table_text("(reference)\ntable output(alt)\nalt=0,100\noutput=0,10\n"))
    location = SourceLocation(path="<test>", line=1)
    block = DownrangeCrossrangeBlock(keyword="dwn/crs", scope="trajectory", location=location)
    block.assignments = [
        Assignment(
            name="latgd",
            value=CallExpression(
                function="table",
                arguments=[NameExpression(name="reference"), NameExpression(name="alt")],
            ),
            location=location,
        ),
        Assignment(name="long", value=NumberExpression(value=0.0), location=location),
        Assignment(name="azm", value=NumberExpression(value=0.0), location=location),
    ]

    result = _trajectory_observables(
        (block,),
        {"alt": 50.0, "lat": 0.0, "long": 0.0, "vel": 1.0, "gama": 0.0, "psi": 0.0},
        {},
        0.0,
        tables,
    )

    assert result["dwnrng"] == pytest.approx(-math.radians(5.0) * 20925646.3255)
    ####


def test_high_dimensional_coordinate_search_is_bounded_and_deterministic() -> None:
    calls: list[tuple[float, ...]] = []

    def objective(point: tuple[float, ...]) -> float:
        calls.append(point)
        return sum((value - 1.0) ** 2 for value in point)
    ####

    result = _coordinate_search((0.0,) * 5, ((-2.0, 2.0),) * 5, objective, max_sweeps=1)

    assert result == _coordinate_search((0.0,) * 5, ((-2.0, 2.0),) * 5, lambda point: sum((value - 1.0) ** 2 for value in point), max_sweeps=1)
    assert len(calls) <= 1 + 4 * 5
    assert sum(value * value for value in result) > 0.0
####


def test_optimization_endpoint_sealing_waits_for_segment_exit() -> None:
    endpoint = OptimizeEndpoint(text="x", segment=2, trajectory=1)
    requirements = _optimization_endpoint_requirements(endpoint)
    stop_when = _optimization_endpoint_stop_when(requirements)
    assert stop_when is not None
    state = RuntimeState(1.0, (1.0,), named={"x": 1.0, "_segment": 2.0}, value_names=("x",))
    vehicle = RuntimeVehicle("1", state, segment_number=2, active=True)
    problem = RuntimeProblem({"1": vehicle})
    assert not stop_when(problem)
    vehicle.segment_number = 3
    state = RuntimeState(2.0, (2.0,), named={"x": 2.0, "_segment": 3.0}, value_names=("x",))
    vehicle.state = state
    vehicle.history.append(state)
    assert stop_when(problem)
    assert not vehicle.active
####


def test_lowered_full_table_preserves_argumented_nested_table_lookup() -> None:
    document = parse_table_text(
        "(lookup)\n"
        "table output(x)\n"
        "x=0,10\n"
        "output=1,3\n"
        "(outer)\n"
        "table output\n"
        "start\n"
        "add lookup(5)\n"
        "mult 2\n"
        "end\n"
    )

    tables = lower_tables(document)

    assert tables["outer"].evaluate({}, tables) == pytest.approx(4.0)
####


def test_runtime_definition_supports_argumented_table_lookup(tmp_path: Path) -> None:
    problem = tmp_path / "argumented-table.prb"
    problem.write_text(
        "(argumented-table)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=100 gama=0 psi=0 time=0 mass=1\n"
        "  *define sampled\n"
        "    sampled = table(drag, vel);\n"
        "  *file argumented.dat time sampled\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )
    table = tmp_path / "drag.tbl"
    table.write_text("(drag)\ntable ca(vel)\nvel=0,100\nca=0.1,0.3\n", encoding="utf-8")

    report = run_files(problem, (table,), output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["sampled"] == pytest.approx(0.3)
####


def test_runtime_definition_supports_nested_full_table_references(tmp_path: Path) -> None:
    problem = tmp_path / "nested-full-table.prb"
    problem.write_text(
        "(nested-full-table)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=0 ydt=0 zdt=0 time=0 mass=1\n"
        "  *define sampled\n"
        "    sampled = table(wrapper);\n"
        "  *file nested.dat time sampled\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )
    table = tmp_path / "nested.tbl"
    table.write_text(
        "(seed)\n"
        "table output\n"
        "start\n"
        "set 4\n"
        "end\n"
        "(wrapper)\n"
        "table output\n"
        "start\n"
        "add seed\n"
        "mult 3\n"
        "end\n",
        encoding="utf-8",
    )

    report = run_files(problem, (table,), output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["sampled"] == pytest.approx(12.0)
    assert "12.0" in (tmp_path / "out" / "nested.dat").read_text(encoding="utf-8")
####


def test_runtime_definition_executes_conditional_controls_each_stage(tmp_path: Path) -> None:
    problem = tmp_path / "conditional-define.prb"
    problem.write_text(
        "(conditional-define)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*define late\n"
        "if (time > 0.15) then late = 2;\n"
        "else early = 1;\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *file conditional.dat time late early\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>0.2 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    history = report.results[0].states["1"]
    assert history[1].named["early"] == pytest.approx(1.0)
    assert history[-1].named["late"] == pytest.approx(2.0)
    assert "0.2 2.0 1.0" in (tmp_path / "out" / "conditional.dat").read_text(encoding="utf-8")
####


def test_runtime_definition_implements_documented_math_functions(tmp_path: Path) -> None:
    problem = tmp_path / "define-math.prb"
    problem.write_text(
        "(define-math)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=0 ydt=0 zdt=0 time=0 mass=1\n"
        "  *define sine\n"
        "    sine = sin(90); cosine = cos(180); tangent = tan(45);\n"
        "    inverse = atan2(1, 1); logarithm = log10(100);\n"
        "    hyperbolic = sinh(0); rounded = ceil(1.2) + floor(1.8);\n"
        "  *file math.dat time sine cosine tangent inverse logarithm hyperbolic rounded\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time=0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=20)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1].named
    assert final["sine"] == pytest.approx(1.0)
    assert final["cosine"] == pytest.approx(-1.0)
    assert final["tangent"] == pytest.approx(1.0)
    assert final["inverse"] == pytest.approx(math.pi / 4.0)
    assert final["logarithm"] == pytest.approx(2.0)
    assert final["hyperbolic"] == pytest.approx(0.0)
    assert final["rounded"] == pytest.approx(3.0)
####


def test_ecfc_gravity_assembly_applies_j2_for_wgs_models() -> None:
    radius = 20_902_646.3255
    named = {"x": radius, "y": 0.0, "z": 0.0, "xdt": 0.0, "ydt": 0.0, "zdt": 0.0}
    point_mass = {name: 0.0 for name in ("x", "y", "z", "xdt", "ydt", "zdt")}
    j2 = dict(point_mass)

    _assemble_ecfc_rates(point_mass, named, 1.407646463e16, 0.0, 0.0)
    _assemble_ecfc_rates(j2, named, 1.407646463e16, 0.0, 1.08262668e-3)

    assert j2["xdt"] < point_mass["xdt"]
    assert j2["xdt"] == pytest.approx(point_mass["xdt"] * (1.0 + 1.5 * 1.08262668e-3))
####


def test_ecfc_gravity_assembly_consumes_full_earth_harmonic_coefficients() -> None:
    radius = 20_902_646.3255
    named = {"x": radius, "y": 0.0, "z": 0.0, "xdt": 0.0, "ydt": 0.0, "zdt": 0.0}
    point_mass = {name: 0.0 for name in ("x", "y", "z", "xdt", "ydt", "zdt")}
    full = dict(point_mass)

    _assemble_ecfc_rates(point_mass, named, 1.407646463e16, 0.0, 0.0)
    _assemble_ecfc_rates(full, named, 1.407646463e16, 0.0, 0.0, {(2, 0): (0.01, 0.0)})

    assert full["xdt"] == pytest.approx(point_mass["xdt"] * 0.985)
    assert full["xdt"] != pytest.approx(point_mass["xdt"])
####


def test_geodetic_force_assembly_uses_the_same_wgs_gravity_model() -> None:
    segment = Segment(number=1, title="coast", location=SourceLocation(path="test", line=1), blocks=())
    named = {
        "_geodetic_state": 1.0,
        "long": 0.0,
        "lat": 0.0,
        "alt": 0.0,
        "vel": 1000.0,
        "gama": 0.0,
        "psi": 45.0,
        "x": 20_902_646.3255,
        "y": 0.0,
        "z": 0.0,
        "xdt": 0.0,
        "ydt": 1000.0,
        "zdt": 0.0,
    }
    point_mass = _geodetic_force_rates(segment, named, {}, {}, 0.0, 1.0, 1.407646463e16, 0.0, 0.0)
    wgs = _geodetic_force_rates(segment, named, {}, {}, 0.0, 1.0, 1.407646463e16, 7.2921151467e-5, 1.08262668e-3)

    assert point_mass is not None
    assert wgs is not None
    assert wgs[0] == pytest.approx(point_mass[0], abs=1e-12)
    assert wgs[1] != pytest.approx(point_mass[1])
    assert math.isfinite(wgs[2])
####


def test_geodetic_force_assembly_includes_guidance_acceleration() -> None:
    segment = Segment(number=1, title="guidance", location=SourceLocation(path="test", line=1), blocks=())
    named = {
        "_geodetic_state": 1.0,
        "long": 0.0,
        "lat": 0.0,
        "alt": 0.0,
        "vel": 1000.0,
        "gama": 0.0,
        "psi": 0.0,
        "x": 20_902_646.3255,
        "y": 0.0,
        "z": 0.0,
        "xdt": 0.0,
        "ydt": 1000.0,
        "zdt": 0.0,
    }

    baseline = _geodetic_force_rates(segment, named, {}, {}, 0.0, 1.0, 0.0)
    guided = _geodetic_force_rates(segment, named, {}, {}, 0.0, 1.0, 0.0, guidance_acceleration=(0.0, 100.0, 0.0))

    assert baseline is not None
    assert guided is not None
    assert guided[2] != pytest.approx(baseline[2])
####


def test_standard_atmosphere_uses_layered_high_altitude_properties() -> None:
    sea_level = _standard_atmosphere_properties(0.0)
    high_altitude = _standard_atmosphere_properties(100_000.0)

    assert sea_level[0] == pytest.approx(518.67, abs=0.05)
    assert sea_level[1] == pytest.approx(2116.22, abs=0.1)
    assert high_altitude[0] > 350.0
    assert high_altitude[1] < sea_level[1]
    assert high_altitude[2] < sea_level[2]
    assert all(math.isfinite(value) and value > 0.0 for value in high_altitude)
####


def test_file_runner_emits_case_outputs_and_report(tmp_path: Path) -> None:
    report = run_files(PROBLEM, (TABLE,), output_dir=tmp_path)
    assert report.exit_code == 0
    assert report.cases == 16
    assert (tmp_path / "1.print").exists()
    assert (tmp_path / "ex1.dbf").exists()
    summary = next(tmp_path.glob("case-1-summaries.json"))
    assert "max-q" in json.loads(summary.read_text(encoding="utf-8"))
####


def test_problem_scope_print_renders_indexed_history(tmp_path: Path) -> None:
    problem = tmp_path / "problem-print.prb"
    problem.write_text(
        "(problem-print)\n"
        "*print time[1] xecfc[1]\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=2 y=0 z=0 xdt=0 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=20)

    assert report.exit_code == 0
    output = (tmp_path / "out" / "problem.print").read_text(encoding="utf-8")
    assert output.splitlines()[:2] == ["time xecfc[1]", "0.0 2.0"]
    assert "nan" not in output.lower()
####


def test_run_report_excludes_preexisting_output_files(tmp_path: Path) -> None:
    sentinel = tmp_path / "stale-artifact.txt"
    sentinel.write_text("do not report\n", encoding="utf-8")

    report = run_files(PROBLEM, (TABLE,), output_dir=tmp_path)

    assert report.exit_code == 0
    assert str(sentinel) not in report.outputs
    assert list(report.outputs) == sorted(report.outputs)
####


def test_run_report_returns_nonzero_for_incomplete_execution() -> None:
    report = RunReport("problem.prb", (), 1, (ExecutionResult({}, False, "boundary_stall"),), (), ())

    assert report.exit_code == 1
####


def test_constants_and_cg_blocks_feed_tabled_aerodynamics(tmp_path: Path) -> None:
    root = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p036_constants_cg_user_variables/input"
    report = run_files(
        root / "p036_constants_cg_user_variables.prb",
        (root / "weight-cg.tbl", root / "cg-sensitive-ca.tbl"),
        output_dir=tmp_path,
        max_steps=300,
    )

    assert report.exit_code == 0
    first_row = (tmp_path / "cg.dat").read_text(encoding="utf-8").splitlines()[1].split()
    assert float(first_row[2]) == pytest.approx(0.725)
    assert float(first_row[4]) > 0.0
####


def test_tangent_downrange_and_iip_outputs_are_finite(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p028_tangent_iip_downrange/input/p028_tangent_iip_downrange.prb"
    report = run_files(problem, output_dir=tmp_path, max_steps=300)

    assert report.exit_code == 0
    lines = (tmp_path / "iip.dat").read_text(encoding="utf-8").splitlines()
    assert all("nan" not in line.lower() for line in lines[1:])
####


def test_relative_and_radar_outputs_are_indexed_and_finite(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p017_radar_relative/input/p017_radar_relative.prb"
    report = run_files(problem, output_dir=tmp_path, max_steps=200)

    assert report.exit_code == 0
    lines = (tmp_path / "compare.dat").read_text(encoding="utf-8").splitlines()
    assert lines[1].split()[1:] == ["1000.0", "20.0", "0.0"]
    assert all("nan" not in line.lower() for line in lines[1:])
####


def test_relative_velocity_uses_signed_closure_convention() -> None:
    from taoryx.equations.geodesy import CartesianVector3

    source_position = CartesianVector3(0.0, 0.0, 0.0)
    target_position = CartesianVector3(100.0, 0.0, 0.0)
    source_velocity = CartesianVector3(0.0, 0.0, 0.0)
    assert _closure_velocity(
        target_position,
        source_position,
        CartesianVector3(-10.0, 0.0, 0.0),
        source_velocity,
    ) == pytest.approx(10.0)
    assert _closure_velocity(
        target_position,
        source_position,
        CartesianVector3(10.0, 0.0, 0.0),
        source_velocity,
    ) == pytest.approx(-10.0)
####


def test_ground_intercept_file_drives_predictive_guidance_and_limits(tmp_path: Path) -> None:
    root = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p035_manual_ground_intercept_synthetic/input"
    report = run_files(
        root / "p035_manual_ground_intercept_synthetic.prb",
        tuple(root.glob("*.tbl")),
        output_dir=tmp_path,
        max_steps=5000,
    )

    assert report.exit_code == 0
    assert not report.diagnostics
    result = report.results[0]
    assert result.completed
    assert result.states["1"][-1].named["_segment"] == pytest.approx(4.0)
    assert result.states["1"][-1].named["alphat"] <= 15.0 + 1e-6
    assert result.states["2"][-1].named["_segment"] == pytest.approx(10.0)
####


def test_multiple_problems_execute_sequentially_without_output_cross_talk(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p031_multi_problem_document/input/p031_multi_problem_document.prb"
    report = run_files(problem, output_dir=tmp_path, max_steps=100)

    assert report.exit_code == 0
    assert report.cases == 2
    one = (tmp_path / "one.dat").read_text(encoding="utf-8").splitlines()[2].split()[1]
    two = (tmp_path / "two.dat").read_text(encoding="utf-8").splitlines()[2].split()[1]
    assert float(two) - float(one) == pytest.approx(1.0)
####


def test_less_than_when_condition_transitions_and_stops(tmp_path: Path) -> None:
    problem = tmp_path / "less-than-event.prb"
    problem.write_text(
        "(less-than-event)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=1 y=0 z=0 xdt=-1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when x<0.25 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=20)

    assert report.exit_code == 0
    assert report.results[0].states["1"][-1].time == pytest.approx(0.75)
####


@pytest.mark.parametrize(
    ("operator", "initial", "velocity", "expected_time"),
    (("<=", 1.0, -1.0, 0.75), (">=", 0.0, 1.0, 0.25)),
)
def test_non_strict_when_conditions_trigger_at_boundary(
    tmp_path: Path,
    operator: str,
    initial: float,
    velocity: float,
    expected_time: float,
) -> None:
    problem = tmp_path / f"non-strict-{operator}.prb"
    problem.write_text(
        "(non-strict-event)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        f"  *initial ecfc x={initial} y=0 z=0 xdt={velocity} ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        f"    *when x{operator}0.25 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=20)

    assert report.exit_code == 0
    assert report.results[0].states["1"][-1].time == pytest.approx(expected_time)
####


def test_multi_problem_outputs_do_not_collide(tmp_path: Path) -> None:
    problem = tmp_path / "shared-output.prb"
    problem.write_text(
        "(first)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *file shared.dat time xecfc\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time=0.1 stop\n"
        "*end\n"
        "(second)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=10 y=0 z=0 xdt=2 ydt=0 zdt=0 time=0 mass=1\n"
        "  *file shared.dat time xecfc\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time=0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    first = tmp_path / "out" / "shared.dat"
    second = tmp_path / "out" / "shared-problem-2-case-2.dat"
    assert first.exists()
    assert second.exists()
    assert first.read_text(encoding="utf-8") != second.read_text(encoding="utf-8")
####


def test_piecewise_full_table_if_goto_recomputes_forcing(tmp_path: Path) -> None:
    root = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p009_piecewise_full_thrust/input"
    report = run_files(root / "p009_piecewise_full_thrust.prb", (root / "piecewise.tbl",), output_dir=tmp_path, max_steps=300)

    assert report.exit_code == 0
    rows = (tmp_path / "piecewise.dat").read_text(encoding="utf-8").splitlines()
    header = rows[0].split()
    thrust_index = header.index("thrust")
    before = next(row for row in rows[1:] if float(row.split()[0]) < 2.0)
    after = next(row for row in rows[1:] if float(row.split()[0]) > 2.0)
    assert float(before.split()[thrust_index]) == pytest.approx(100.0)
    assert float(after.split()[thrust_index]) == pytest.approx(20.0)
####


@pytest.mark.parametrize("case_id", ["p018_geodetic_initialization", "p019_ecfc_initialization"])
def test_initial_coordinate_models_populate_finite_frame_outputs(tmp_path: Path, case_id: str) -> None:
    case_root = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive" / case_id / "input"
    problem = next(case_root.glob("*.prb"))
    report = run_files(problem, output_dir=tmp_path / case_id, max_steps=200)

    assert report.exit_code == 0
    output = next((tmp_path / case_id).glob("*.dat")).read_text(encoding="utf-8")
    assert "nan" not in output.lower()
####


def test_site_atmosphere_and_crosswind_refresh_air_relative_outputs(tmp_path: Path) -> None:
    site_root = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p013_site_atmosphere/input"
    wind_root = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p015_crosswind/input"
    site = run_files(next(site_root.glob("*.prb")), output_dir=tmp_path / "site", max_steps=100)
    wind = run_files(next(wind_root.glob("*.prb")), tuple(wind_root.glob("*.tbl")), output_dir=tmp_path / "wind", max_steps=100)

    assert site.exit_code == 0
    assert wind.exit_code == 0
    assert "nan" not in (tmp_path / "site/atmosphere.dat").read_text(encoding="utf-8").lower()
    first_wind = (tmp_path / "wind/wind.dat").read_text(encoding="utf-8").splitlines()[1].split()
    assert float(first_wind[2]) > float(first_wind[1])
####


def test_weight_based_rail_launch_clears_static_friction(tmp_path: Path) -> None:
    root = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p016_rail_stage_branch/input"
    report = run_files(next(root.glob("*.prb")), tuple(root.glob("*.tbl")), output_dir=tmp_path, max_steps=1000)

    assert report.exit_code == 0
    assert report.results[0].completed
    assert report.results[0].states["1"][-1].named["wt"] == pytest.approx(500.0)
####


def test_case_insensitive_table_axes_link_to_normalized_runtime_state(tmp_path: Path) -> None:
    root = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p046_case_insensitive_free_field/input"
    report = run_files(next(root.glob("*.prb")), tuple(root.glob("*.tbl")), output_dir=tmp_path, max_steps=100)

    assert report.exit_code == 0
    assert "nan" not in (tmp_path / "lexical.dat").read_text(encoding="utf-8").lower()
####


def test_run_taos_accepts_problem_and_table_files(tmp_path: Path) -> None:
    report = run_taos(PROBLEM, (TABLE,), output_dir=tmp_path)

    assert report.exit_code == 0
    assert report.cases == 16
####


def test_full_table_define_fixture_writes_trajectory_file(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p010_output_define_full_table/input/p010_output_define_full_table.prb"
    table = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p010_output_define_full_table/input/output.tbl"
    report = run_files(problem, (table,), output_dir=tmp_path)
    assert report.exit_code == 0
    assert (tmp_path / "heat.dat").exists()
    assert "heat-proxy-value" in (tmp_path / "heat.dat").read_text(encoding="utf-8")
####


def test_propulsion_fixture_reaches_time_event(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p002_constant_thrust_table/input/p002_constant_thrust_table.prb"
    tables = tuple((problem.parent / name) for name in ("constant-thrust.tbl", "zero-mdot.tbl"))
    report = run_files(problem, tables, output_dir=tmp_path, max_steps=500)
    assert report.exit_code == 0
    assert report.results[0].completed
    assert report.results[0].states["1"][-1].time == pytest.approx(5.0)
####


def test_search_fixture_recomputes_trajectory_until_target(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p021_search_linear_target/input/p021_search_linear_target.prb"
    report = run_files(problem, output_dir=tmp_path, max_steps=500)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["xdt"] == pytest.approx(100.0, abs=1e-3)
    assert final.named["x"] == pytest.approx(20926646.3255, abs=1e-3)
####


def test_search_maxitr_zero_runs_only_the_seeded_trajectory(tmp_path: Path) -> None:
    problem = tmp_path / "search-diagnostic.prb"
    problem.write_text(
        "(search-diagnostic)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=srch-1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time=1 stop\n"
        "*search 1 vary x-velocity until xecfc=999 on segment 1, trajectory 1\n"
        "  xlo=0 xhi=20 xest=3 dx=1 tol=0.001 maxitr=0\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["xdt"] == pytest.approx(3.0)
    assert final.named["x"] == pytest.approx(3.0)
####


def test_search_minimum_recomputes_trajectory_for_keyword_objective(tmp_path: Path) -> None:
    problem = tmp_path / "search-minimum.prb"
    problem.write_text(
        "(search-minimum)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=srch-1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time=1 stop\n"
        "*search 1 vary x-velocity until xecfc on segment 1, trajectory 1 = min\n"
        "  xlo=0 xhi=20 xest=5 dx=1 tol=0.001 maxitr=40 fref=10\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["xdt"] == pytest.approx(0.0, abs=1e-3)
    assert final.named["x"] == pytest.approx(0.0, abs=1e-3)
####


def test_search_uses_initial_estimate_and_increment_for_parabolic_startup(tmp_path: Path) -> None:
    problem = tmp_path / "search-parabolic-startup.prb"
    problem.write_text(
        "(search-parabolic-startup)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=srch-1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *define q=xdt*xdt\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time=0 stop\n"
        "*search 1 vary x-velocity until q=4 on segment 1, trajectory 1\n"
        "  xlo=0 xhi=10 xest=2 dx=0.5 tol=0.001 maxitr=1 print=1 integ=0\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=10)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["xdt"] == pytest.approx(2.0, abs=1e-6)
    trial_files = sorted((tmp_path / "out").glob("search-1-case-1-trial-*.print"))
    assert len(trial_files) == 3
    assert all("xdt" in path.read_text(encoding="utf-8") for path in trial_files)
####


def test_multiple_search_blocks_resolve_in_source_order(tmp_path: Path) -> None:
    problem = tmp_path / "two-searches.prb"
    problem.write_text(
        "(two-searches)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=20925646.3255 y=0 z=0 xdt=srch-1 ydt=srch-2 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time=1 stop\n"
        "*search 1 vary x-velocity until xecfc=20925656.3255 on segment 1, trajectory 1\n"
        "  xlo=0 xhi=20 xest=5 dx=1 tol=0.001 xref=10 fref=1 maxitr=30 print=0 integ=0\n"
        "*search 2 vary y-velocity until yecfc=20 on segment 1, trajectory 1\n"
        "  xlo=0 xhi=40 xest=10 dx=1 tol=0.001 xref=10 fref=1 maxitr=30 print=0 integ=0\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["xdt"] == pytest.approx(10.0, abs=1e-3)
    assert final.named["ydt"] == pytest.approx(20.0, abs=1e-3)
####


def test_optimize_fixture_recomputes_scalar_boundary(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p022_optimize_linear_boundary/input/p022_optimize_linear_boundary.prb"
    report = run_files(problem, output_dir=tmp_path, max_steps=500)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["xdt"] == pytest.approx(100.0, abs=1e-2)
    assert final.named["x"] == pytest.approx(20926646.3255, abs=0.1)
####


def test_search_uses_qualified_segment_endpoint(tmp_path: Path) -> None:
    problem = tmp_path / "search-segment.prb"
    problem.write_text(
        "(search-segment)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=srch-1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 first\n"
        "    *integ dt=0.1\n"
        "    *when time>1 goto 2\n"
        "  *segment 2 second\n"
        "    *when time>2 stop\n"
        "*search 1 vary x-velocity until xecfc=9 on segment 1, trajectory 1\n"
        "  xlo=0 xhi=20 xest=5 dx=1 tol=0.001 xref=10 fref=1 maxitr=30 print=0 integ=0\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["xdt"] == pytest.approx(10.0, abs=1e-3)
    assert final.named["x"] == pytest.approx(20.0, abs=0.05)
####


def test_indexed_optimization_endpoint_reads_selected_trajectory_state() -> None:
    endpoint = OptimizeEndpoint(
        text="alt[2]",
        expression=IndexedExpression(name="alt", index=2),
        segment=3,
        trajectory_subscript=2,
    )
    state = RuntimeState(
        4.0,
        (450.0,),
        named={"alt": 450.0, "_segment": 3.0},
        value_names=("alt",),
    )
    result = ExecutionResult(states={"2": (state,)}, completed=True)

    assert _endpoint_value(endpoint, result, {}) == pytest.approx(450.0)
####


def test_unsupported_search_shape_is_reported_before_execution() -> None:
    location = SourceLocation(path="search.prb", line=1)
    controls = [
        Assignment(name=name, value=NumberExpression(value=value), location=location)
        for name, value in (("xlo", 0.0), ("xhi", 20.0), ("tol", 0.001), ("maxitr", 30.0))
    ]
    search = SearchBlock(
        search_id=1,
        objective=SearchObjective(
            left=OptimizeEndpoint(text="x", expression=NameExpression(name="x")),
            operator=None,
            right=OptimizeEndpoint(text="9", expression=NumberExpression(value=9.0)),
        ),
        controls=controls,
        location=location,
        scope="problem",
    )
    problem = Problem(name="search", location=location, blocks=[search])

    assert _unsupported_features(problem) == ("search",)
    assert lower_problem_document(ProblemDocument(problems=[problem])).unsupported_features == ("search",)
####


def test_inequality_search_recomputes_trajectory_to_the_boundary(tmp_path: Path) -> None:
    problem = tmp_path / "inequality-search.prb"
    problem.write_text(
        "(inequality-search)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=srch-1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>1 stop\n"
        "*search 1 vary x-velocity until xecfc>9 on segment 1, trajectory 1\n"
        "  xlo=0 xhi=20 xest=5 dx=1 tol=0.001 xref=10 fref=1 maxitr=30 print=0 integ=0\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["xdt"] == pytest.approx(9.0, abs=1e-3)
    assert final.named["x"] == pytest.approx(9.0, abs=1e-3)
####


def test_optimization_uses_qualified_segment_endpoint(tmp_path: Path) -> None:
    problem = tmp_path / "optimize-segment.prb"
    problem.write_text(
        "(optimize-segment)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=opta-1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 first\n"
        "    *integ dt=0.1\n"
        "    *when time>1 goto 2\n"
        "  *segment 2 second\n"
        "    *when time>2 stop\n"
        "*optimize a for xecfc=max on segment 1, trajectory 1\n"
        "  par-1=5 lo-1=0 hi-1=20 maxitr=30 tol=0.001\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["xdt"] == pytest.approx(20.0, abs=0.2)
    assert final.named["x"] == pytest.approx(40.0, abs=0.5)
####


def test_optimization_maxitr_zero_runs_only_seeded_trajectory(tmp_path: Path) -> None:
    problem = tmp_path / "optimize-diagnostic.prb"
    problem.write_text(
        "(optimize-diagnostic)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=opta-1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time=1 stop\n"
        "*optimize a for xecfc=max on segment 1, trajectory 1\n"
        "  par-1=3 lo-1=0 hi-1=20 maxitr=0 restarts=2\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["xdt"] == pytest.approx(3.0)
    assert final.named["x"] == pytest.approx(3.0)
####


def test_optimization_restarts_retry_and_double_final_budget(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    problem = tmp_path / "optimize-restarts.prb"
    problem.write_text(
        "(optimize-restarts)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=opta-1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time=1 stop\n"
        "*optimize a for xecfc=max on segment 1, trajectory 1\n"
        "  par-1=3 lo-1=0 hi-1=20 maxitr=4 restarts=2\n"
        "*end\n",
        encoding="utf-8",
    )
    budgets: list[int] = []

    class FakeOptimizer:
        def run(self, initial: tuple[float, ...], *, max_iterations: int) -> OptimizationResult:
            budgets.append(max_iterations)
            return OptimizationResult(initial, 0.0, (), (), max_iterations, OptimizationStatus.MAX_ITERATIONS)
        ####
    ####

    monkeypatch.setattr(lowering_module, "resolve_optimize_block", lambda *args, **kwargs: FakeOptimizer())
    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    assert budgets == [4, 4, 8]
####


def test_adjust_redistributes_parameterized_tseg_guidance_history() -> None:
    location = SourceLocation(path="adjust.prb", line=1)
    point = lambda index, value: FlyPoint(
        independent=ParameterExpression(family="optimize", loop="a", index=index),
        value=ParameterExpression(family="optimize", loop="a", index=value),
        location=location,
    )
    fly = FlyBlock(guidance_variable="alpha", reference="tseg", points=[point(1, 4), point(2, 5), point(3, 6)], location=location, scope="segment")
    segment = Segment(number=1, title="guidance", location=location, blocks=[fly])
    trajectory = Trajectory(number=1, name="vehicle", start_segment=1, location=location, segments=[segment])
    problem = Problem(name="adjust", location=location, trajectories=[trajectory])
    parameters = {
        "optimize-a-1": 0.0,
        "optimize-a-2": 100.0,
        "optimize-a-3": 260.0,
        "optimize-a-4": 0.0,
        "optimize-a-5": 20.0,
        "optimize-a-6": 0.0,
    }

    adjusted = _adjust_guidance_history_parameters(problem, parameters)

    assert adjusted["optimize-a-1"] == pytest.approx(0.0)
    assert adjusted["optimize-a-2"] == pytest.approx(130.0)
    assert adjusted["optimize-a-3"] == pytest.approx(260.0)
    assert adjusted["optimize-a-4"] == pytest.approx(0.0)
    assert adjusted["optimize-a-5"] == pytest.approx(16.25)
    assert adjusted["optimize-a-6"] == pytest.approx(0.0)
####


def test_user_atmosphere_refreshes_environment_outputs(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p012_user_atmosphere/input/p012_user_atmosphere.prb"
    report = run_files(problem, output_dir=tmp_path, max_steps=100)

    assert report.exit_code == 0
    lines = (tmp_path / "atmosphere.dat").read_text(encoding="utf-8").splitlines()
    assert all("483.03" in line and "1455.3" in line and "0.001756" in line for line in lines[1:])
####


def test_aero_table_uses_environment_pressure_and_reference_area(tmp_path: Path) -> None:
    problem = tmp_path / "aero.prb"
    problem.write_text(
        "(aero)\n"
        "*atmos user\n"
        "alt temp pres rho sndspd nu\n"
        "0 300 100 1 100 1\n"
        "100 300 100 1 100 1\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=100 gama=0 psi=0 time=0 mass=1\n"
        "  *file aero.dat time vel dynprs mach ca\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *aero ca=(drag)\n"
        "    *when time=0.2 stop\n"
        "*end\n",
        encoding="utf-8",
    )
    table = tmp_path / "drag.tbl"
    table.write_text("(drag)\ntable ca(mach) sref=2\nmach=0,2\nca=0.01,0.01\n", encoding="utf-8")

    report = run_files(problem, (table,), output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    lines = (tmp_path / "out" / "aero.dat").read_text(encoding="utf-8").splitlines()
    assert "5000.0" in lines[1]
    assert "1.0" in lines[1]
    assert float(lines[-1].split()[1]) < 100.0
    assert "0.01" in lines[-1]
####


def test_aero_block_reference_area_scales_runtime_force(tmp_path: Path) -> None:
    problem = tmp_path / "aero-area.prb"
    problem.write_text(
        "(aero-area)\n"
        "*atmos user\n"
        "alt temp pres rho sndspd nu\n"
        "0 300 100 1 100 1\n"
        "100 300 100 1 100 1\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=10 ydt=0 zdt=0 time=0 mass=1\n"
        "  *file area.dat time xecfcdt\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *aero ca=0.1 sref=2\n"
        "    *when time=0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["xdt"] == pytest.approx(10.0 / 1.1, abs=1e-5)
####


def test_initial_state_resolves_table_valued_expression(tmp_path: Path) -> None:
    problem = tmp_path / "initial-table.prb"
    problem.write_text(
        "(initial-table)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=(seed) y=0 z=0 xdt=0 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time=0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )
    table = tmp_path / "seed.tbl"
    table.write_text("(seed)\ntable thrust(time)\ntime=0,1\nthrust=5,5\n", encoding="utf-8")

    report = run_files(problem, (table,), output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    assert report.results[0].states["1"][0].named["x"] == pytest.approx(5.0)
####


def test_summary_selects_trajectory_history_and_alias(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p020_survey_summarize/input/p020_survey_summarize.prb"
    report = run_files(problem, output_dir=tmp_path, max_steps=500)

    assert report.exit_code == 0
    summary = json.loads((tmp_path / "case-1-summaries.json").read_text(encoding="utf-8"))
    assert summary["final-x"] == pytest.approx(20925746.3255, abs=1e-3)
####


def test_summary_runtime_evaluates_typed_arithmetic_chain(tmp_path: Path) -> None:
    problem = tmp_path / "summary-math.prb"
    problem.write_text(
        "(summary-math)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=2 y=0 z=0 xdt=0 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>0.2 stop\n"
        "*summarize arithmetic\n"
        "  add last(xecfc[1])\n"
        "  mult 3\n"
        "  sub 1\n"
        "  div 5\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    summary = json.loads((tmp_path / "out" / "case-1-summaries.json").read_text(encoding="utf-8"))
    assert summary["arithmetic"] == pytest.approx(1.0)
####


def test_summary_history_functions_use_the_retained_trajectory_samples(tmp_path: Path) -> None:
    problem = tmp_path / "summary-history.prb"
    problem.write_text(
        "(summary-history)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=2 y=0 z=0 xdt=1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *file history.dat time xecfc\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>0.3 stop\n"
        "*summarize first-x\n"
        "  add first(xecfc[1])\n"
        "*summarize last-x\n"
        "  add last(xecfc[1])\n"
        "*summarize min-x\n"
        "  add min(xecfc[1])\n"
        "*summarize max-x\n"
        "  add max(xecfc[1])\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    summary = json.loads((tmp_path / "out" / "case-1-summaries.json").read_text(encoding="utf-8"))
    assert summary == {
        "first-x": pytest.approx(2.0),
        "last-x": pytest.approx(2.3),
        "min-x": pytest.approx(2.0),
        "max-x": pytest.approx(2.3),
    }
    rows = (tmp_path / "out" / "history.dat").read_text(encoding="utf-8").splitlines()
    assert rows[1].split() == ["0.0", "2.0"]
    assert rows[-1].split() == ["0.3", "2.3"]
####


def test_summary_segment_uses_exact_goto_boundary_state(tmp_path: Path) -> None:
    problem = tmp_path / "summary-segment-boundary.prb"
    problem.write_text(
        "(summary-segment-boundary)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 handoff\n"
        "    *integ dt=0.3\n"
        "    *when time=1 goto 2\n"
        "  *segment 2 coast\n"
        "    *integ dt=0.2\n"
        "    *when time>1.4 stop\n"
        "*summarize segment-one-end\n"
        "  add xecfc on segment 1\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    summary = json.loads((tmp_path / "out" / "case-1-summaries.json").read_text(encoding="utf-8"))
    assert summary["segment-one-end"] == pytest.approx(1.0)
    assert report.results[0].states["1"][-1].named["_segment"] == pytest.approx(2.0)
####


def test_summary_inverse_operations_and_trigonometry_follow_manual_units(tmp_path: Path) -> None:
    problem = tmp_path / "summary-operators.prb"
    problem.write_text(
        "(summary-operators)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=0 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time=0.1 stop\n"
        "*summarize inverse-division\n"
        "  add 2\n"
        "  idiv 8\n"
        "*summarize inverse-exponent\n"
        "  add 2\n"
        "  iexp 3\n"
        "*summarize sine-degrees\n"
        "  add 90\n"
        "  sin\n"
        "*summarize inverse-sine-degrees\n"
        "  add 0.5\n"
        "  asin\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=20)

    assert report.exit_code == 0
    summary = json.loads((tmp_path / "out/case-1-summaries.json").read_text(encoding="utf-8"))
    assert summary["inverse-division"] == pytest.approx(4.0)
    assert summary["inverse-exponent"] == pytest.approx(9.0)
    assert summary["sine-degrees"] == pytest.approx(1.0)
    assert summary["inverse-sine-degrees"] == pytest.approx(30.0)
####


def test_egs_summary_file_aggregates_survey_cases(tmp_path: Path) -> None:
    problem = tmp_path / "egs-summary.prb"
    problem.write_text(
        "(egs-summary)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=surv-1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>0.2 stop\n"
        "*survey 1 speed vals=1,2\n"
        "*summarize final-speed\n"
        "  add last(xecfc[1])\n"
        "*egs summary summary.dbf\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=20)

    assert report.exit_code == 0
    output = (tmp_path / "out" / "summary.dbf").read_text(encoding="utf-8")
    assert "LEVEL 1 speed" in output
    assert "LEVEL 2 final-speed" in output
    assert "1.0" in output and "2.0" in output
####


def test_direct_fly_control_refreshes_runtime_state(tmp_path: Path) -> None:
    problem = tmp_path / "fly.prb"
    problem.write_text(
        "(fly)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 aircraft start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=10 gama=0 psi=0 time=0 mass=1\n"
        "  *file fly.dat time mach\n"
        "  *segment 1 cruise\n"
        "    *integ dt=0.1\n"
        "    *fly mach=0.55\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    assert "0.55" in (tmp_path / "out" / "fly.dat").read_text(encoding="utf-8")
####


def test_wildcard_fly_reuses_previous_segment_value(tmp_path: Path) -> None:
    problem = tmp_path / "wildcard.prb"
    problem.write_text(
        "(wildcard)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=10 gama=0 psi=0 time=0 wt=1\n"
        "  *file wildcard.dat time yawgd\n"
        "  *segment 1 set-heading\n"
        "    *integ dt=0.1\n"
        "    *fly yawgd=25\n"
        "    *when time>0.1 goto 2\n"
        "  *segment 2 hold-heading\n"
        "    *integ dt=0.1\n"
        "    *fly yawgd=*\n"
        "    *when time>0.2 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    assert report.results[0].states["1"][-1].named["yawgd"] == pytest.approx(25.0)
####


def test_ld_max_guidance_solves_active_aerodynamic_coefficients(tmp_path: Path) -> None:
    problem = tmp_path / "ld-max.prb"
    problem.write_text(
        "(ld-max)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=10 gama=0 psi=0 time=0 wt=1\n"
        "  *file ld-max.dat time alpha\n"
        "  *segment 1 glide\n"
        "    *integ dt=0.1\n"
        "    *aero cl=4-alpha^2 cd=1\n"
        "    *fly l/d-max\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["alpha_l/d"] == pytest.approx(0.0, abs=1e-4)
    assert final.named["l/d"] == pytest.approx(4.0, abs=1e-4)
####


def test_standard_atmosphere_and_geodetic_kinematics(tmp_path: Path) -> None:
    problem = tmp_path / "standard.prb"
    problem.write_text(
        "(standard)\n"
        "*atmos standard\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=100 gama=0 psi=90 time=0 mass=1\n"
        "  *file standard.dat time alt long lat temp rho mach dynprs\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["temp"] == pytest.approx(518.67)
    assert final.named["rho"] == pytest.approx(0.0023769)
    assert final.named["dynprs"] == pytest.approx(11.8845, rel=1e-5)
    assert final.named["long"] > 0.0
####


def test_geodetic_vector_thrust_changes_speed_and_flight_path(tmp_path: Path) -> None:
    problem = tmp_path / "vector-thrust.prb"
    problem.write_text(
        "(vector-thrust)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=10 gama=0 psi=0 time=0 mass=1\n"
        "  *file vector-thrust.dat time vel gama psi\n"
        "  *segment 1 launch\n"
        "    *integ dt=0.1\n"
        "    *prop thrust=1 mdot=0\n"
        "    *fly pitchi=45\n"
        "    *fly yawi=0\n"
        "    *fly rolli=0\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["vel"] > 10.05
    assert final.named["gama"] > 0.2
    assert abs(final.named["psi"]) < 1e-10
####


def test_geodetic_vector_thrust_projects_ep_controls_into_heading(tmp_path: Path) -> None:
    problem = tmp_path / "geodetic-vector-heading.prb"
    problem.write_text(
        "(geodetic-vector-heading)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=100 gama=0 psi=0 time=0 mass=1\n"
        "  *file geodetic-vector-heading.dat time vel gama psi\n"
        "  *segment 1 launch\n"
        "    *integ dt=0.1\n"
        "    *prop thrust=10 mdot=0 ep1=90 ep2=0\n"
        "    *fly yawi=0\n"
        "    *fly pitchi=0\n"
        "    *fly rolli=0\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["vel"] > 100.0
    assert abs(final.named["psi"]) > 0.05
    assert abs(final.named["gama"]) < 1e-8
####


def test_integral_define_control_program_drives_runtime_state(tmp_path: Path) -> None:
    problem = tmp_path / "integral.prb"
    problem.write_text(
        "(integral)\n"
        "*atmos none\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *define integral accumulated=0\n"
        "    if (vel > 1000) {\n"
        "      accumulated = vel;\n"
        "    } else {\n"
        "      accumulated = 0.0;\n"
        "    }\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=2000 gama=0 psi=0 time=0 wt=1\n"
        "  *file integral.dat time accumulated\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>0.2 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["accumulated"] == pytest.approx(400.0, abs=1e-6)
####


def test_guidance_table_refreshes_live_control_value(tmp_path: Path) -> None:
    problem = tmp_path / "guidance-table.prb"
    problem.write_text(
        "(guidance-table)\n"
        "*atmos none\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=100 gama=0 psi=0 time=0 wt=1\n"
        "  *file guidance-table.dat time tseg alpha\n"
        "  *segment 1 guidance\n"
        "    *integ dt=0.1\n"
        "    *fly alpha vrs tseg interp-2\n"
        "      0 0\n"
        "      1 10\n"
        "    *when time>0.5 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["alpha"] == pytest.approx(0.05, abs=1e-6)
####


def test_manual_guidance_table_uses_value_then_independent_columns(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p024_guidance_table/input/p024_guidance_table.prb"
    report = run_files(problem, output_dir=tmp_path, max_steps=100)

    assert report.exit_code == 0
    history = report.results[0].states["1"]
    sample = next(state for state in history if state.time == pytest.approx(0.3))
    assert sample.named["alpha"] == pytest.approx(1.5, abs=1e-6)
####


@pytest.mark.parametrize(
    ("interpolation", "points", "expected"),
    (("interp-2", "350 0\n10 1", 360.0), ("interp-3", "0 0\n1 1\n4 2", 0.25)),
)
def test_guidance_interpolation_modes(
    tmp_path: Path,
    interpolation: str,
    points: str,
    expected: float,
) -> None:
    problem = tmp_path / f"{interpolation}.prb"
    problem.write_text(
        "(guidance-interpolation)\n"
        "*atmos none\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=0 gama=0 psi=0 time=0 wt=1\n"
        "  *segment 1 guidance\n"
        "    *integ dt=0.5\n"
        f"    *fly alpha vrs tseg {interpolation}\n"
        + "\n".join(f"      {line}" for line in points.splitlines())
        + "\n    *when tseg=0.5 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=20)

    assert report.exit_code == 0
    assert report.results[0].states["1"][-1].named["alpha"] == pytest.approx(expected, abs=1e-6)
####


def test_rate_guidance_drives_integrated_heading_and_power(tmp_path: Path) -> None:
    problem = tmp_path / "rate-guidance.prb"
    problem.write_text(
        "(rate-guidance)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=0 gama=0 psi=0 time=0 wt=1\n"
        "  *file rate-guidance.dat time psi power\n"
        "  *segment 1 guidance\n"
        "    *integ dt=0.1\n"
        "    *fly psigddt=5\n"
        "    *fly powerdt=2\n"
        "    *when time>1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["psi"] == pytest.approx(5.0, abs=1e-6)
    assert final.named["power"] == pytest.approx(2.0, abs=1e-6)
####


def test_ordinary_define_control_program_refreshes_output(tmp_path: Path) -> None:
    problem = tmp_path / "derived-control.prb"
    problem.write_text(
        "(derived-control)\n"
        "*atmos none\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *define regime\n"
        "    if (vel > 100) {\n"
        "      regime = 2;\n"
        "    } else {\n"
        "      regime = 1;\n"
        "    }\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=200 gama=0 psi=0 time=0 wt=1\n"
        "  *file derived-control.dat time regime\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    assert report.results[0].states["1"][-1].named["regime"] == pytest.approx(2.0)
####


def test_geodetic_gravity_projects_into_flight_path_rate(tmp_path: Path) -> None:
    problem = tmp_path / "geodetic-gravity.prb"
    problem.write_text(
        "(geodetic-gravity)\n"
        "*atmos none\n"
        "*earth spherical gm=14070000000000000 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=100 gama=0 psi=0 time=0 wt=1\n"
        "  *file geodetic-gravity.dat time vel gama\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["gama"] < -0.1
####


def test_geodetic_wind_coefficient_family_projects_lift_drag_and_side(tmp_path: Path) -> None:
    problem = tmp_path / "wind-forces.prb"
    problem.write_text(
        "(wind-forces)\n"
        "*atmos user\n"
        "alt temp pres rho sndspd nu\n"
        "0 300 100 1 100 1\n"
        "100 300 100 1 100 1\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=100 gama=0 psi=0 time=0 wt=1\n"
        "  *file wind-forces.dat time vel gama psi\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *aero cl=0.01 cd=0.01 cs=0.01\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["vel"] < 100.0
    assert final.named["gama"] > 1.0
    assert final.named["psi"] > 1.0
####


def test_limits_bound_guidance_controls_without_stopping_trajectory(tmp_path: Path) -> None:
    problem = tmp_path / "limits.prb"
    problem.write_text(
        "(limits)\n"
        "*atmos none\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=0 gama=0 psi=0 time=0 wt=1\n"
        "  *file limits.dat time alpha\n"
        "  *segment 1 launch\n"
        "    *integ dt=1\n"
        "    *fly alpha=10\n"
        "    *limits alpha<2\n"
        "    *when time>1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["alpha"] == pytest.approx(2.0, abs=1e-8)
    assert final.time == pytest.approx(1.0, abs=1e-8)
####


def test_ecfc_propulsive_vector_uses_vector_angles_and_body_attitude(tmp_path: Path) -> None:
    problem = tmp_path / "ecfc-thrust-vector.prb"
    problem.write_text(
        "(ecfc-thrust-vector)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=0 ydt=0 zdt=0 time=0 mass=10\n"
        "  *file ecfc-thrust-vector.dat time xecfcdt yecfcdt zecfcdt\n"
        "  *segment 1 burn\n"
        "    *integ dt=0.1\n"
        "    *prop thrust=10 mdot=0 ep1=90 ep2=0\n"
        "    *fly yawi=90\n"
        "    *fly pitchi=0\n"
        "    *fly rolli=0\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["xdt"] == pytest.approx(0.1, abs=1e-8)
    assert final.named["ydt"] == pytest.approx(0.0, abs=1e-8)
    assert final.named["zdt"] == pytest.approx(0.0, abs=1e-8)
####


def test_ecfc_specific_load_observables_project_body_axis_thrust(tmp_path: Path) -> None:
    problem = tmp_path / "specific-load.prb"
    problem.write_text(
        "(specific-load)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=0 ydt=0 zdt=0 time=0 mass=10\n"
        "  *file specific-load.dat time nx ny nz ntotal\n"
        "  *segment 1 burn\n"
        "    *integ dt=0.1\n"
        "    *prop thrust=10 mdot=0\n"
        "    *fly yawi=0\n"
        "    *fly pitchi=0\n"
        "    *fly rolli=0\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1].named
    assert final["nx"] == pytest.approx(1.0, abs=1e-8)
    assert final["ny"] == pytest.approx(0.0, abs=1e-8)
    assert final["nz"] == pytest.approx(0.0, abs=1e-8)
    assert final["ntotal"] == pytest.approx(1.0, abs=1e-8)
####


def test_ecfc_axial_load_guidance_solves_free_angle(tmp_path: Path) -> None:
    problem = tmp_path / "axial-load-guidance.prb"
    problem.write_text(
        "(axial-load-guidance)\n"
        "*atmos user\n"
        "alt temp pres rho sndspd nu\n"
        "0 300 100 1 100 1\n"
        "100 300 100 1 100 1\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=100 ydt=0 zdt=0 time=0 mass=1\n"
        "  *file axial-load-guidance.dat time alpha nx\n"
        "  *segment 1 trim\n"
        "    *integ dt=0.1\n"
        "    *aero ca=0.01*alpha\n"
        "    *fly nx=-50\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1].named
    assert 1.0 < final["alpha"] < 1.2
    assert final["nx"] == pytest.approx(-50.0, abs=1e-5)
####


def test_ecfc_lateral_load_guidance_solves_free_sideslip(tmp_path: Path) -> None:
    problem = tmp_path / "lateral-load-guidance.prb"
    problem.write_text(
        "(lateral-load-guidance)\n"
        "*atmos user\n"
        "alt temp pres rho sndspd nu\n"
        "0 300 100 1 100 1\n"
        "100 300 100 1 100 1\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=100 ydt=0 zdt=0 time=0 mass=1\n"
        "  *file lateral-load-guidance.dat time betae ny\n"
        "  *segment 1 trim\n"
        "    *integ dt=0.1\n"
        "    *aero cs=0.01*betae\n"
        "    *fly ny=50\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1].named
    assert 1.0 < final["betae"] < 1.2
    assert final["ny"] == pytest.approx(50.0, abs=1e-5)
####


def test_ecfc_wind_aero_family_contributes_in_all_velocity_axes(tmp_path: Path) -> None:
    problem = tmp_path / "ecfc-wind-forces.prb"
    problem.write_text(
        "(ecfc-wind-forces)\n"
        "*atmos user\n"
        "alt temp pres rho sndspd nu\n"
        "0 300 100 1 100 1\n"
        "100 300 100 1 100 1\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=100 ydt=0 zdt=0 time=0 mass=1\n"
        "  *file ecfc-wind-forces.dat time xecfcdt yecfcdt zecfcdt\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *aero cl=0.01 cd=0.01 cs=0.01\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert 94.0 < final.named["xdt"] < 100.0
    assert final.named["ydt"] > 4.0
    assert final.named["zdt"] < -4.0
####


def test_wgs84_earth_defaults_supply_gravity_without_explicit_gm(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p043_wgs84_gravity_drop/input/p043_wgs84_gravity_drop.prb"
    report = run_files(problem, output_dir=tmp_path, max_steps=10000)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["alt"] < 100000.0
    assert final.named["vel"] > 300.0
####


def test_level_mach_guidance_connects_command_values_to_rates(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p025_level_mach_guidance/input/p025_level_mach_guidance.prb"
    tables = tuple(problem.parent.glob("*.tbl"))
    report = run_files(problem, tables, output_dir=tmp_path, max_steps=20000)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["gama"] == pytest.approx(0.0, abs=1e-8)
    assert final.named["mach"] == pytest.approx(0.55, abs=1e-12)
    assert final.named["power"] > 0.0
    assert final.named["vel"] < 600.0
    assert "nan" not in (tmp_path / "level.dat").read_text(encoding="utf-8").casefold()
####


def test_heading_guidance_connects_psigd_command_to_live_heading_rate(tmp_path: Path) -> None:
    problem = tmp_path / "heading-guidance.prb"
    problem.write_text(
        "(heading-guidance)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=100 gama=0 psi=0 time=0 mass=1\n"
        "  *file heading-guidance.dat time psi\n"
        "  *segment 1 turn\n"
        "    *integ dt=0.1 dtguid=0.5\n"
        "    *fly psigd=90\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert 15.0 < final.named["psi"] < 18.0
####


def test_indirect_coefficient_guidance_solves_free_angle(tmp_path: Path) -> None:
    problem = tmp_path / "coefficient-guidance.prb"
    problem.write_text(
        "(coefficient-guidance)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=100 gama=0 psi=0 time=0 mass=1\n"
        "  *file coefficient-guidance.dat time alpha cl\n"
        "  *segment 1 trim\n"
        "    *integ dt=0.1\n"
        "    *aero cl=0.1*alpha cd=1\n"
        "    *fly cl=0.5\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["alpha"] == pytest.approx(5.0, abs=1e-5)
    assert final.named["cl"] == pytest.approx(0.5, abs=1e-6)
####


def test_relative_guidance_connects_target_state_to_live_commands(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p027_proportional_navigation/input/p027_proportional_navigation.prb"
    tables = tuple(problem.parent.glob("*.tbl"))
    report = run_files(problem, tables, output_dir=tmp_path, max_steps=20000)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert "yawi" in final.named
    assert "pitchi" in final.named
    assert final.named["relrng[2]"] > 0.0
    assert math.isfinite(final.named["relvel[2]"])
####


@pytest.mark.parametrize(
    ("rule", "expected_sign"),
    (("downria", -1.0), ("upria", 1.0)),
)
def test_range_insensitive_guidance_binds_iip_sensitivity_direction(
    tmp_path: Path,
    rule: str,
    expected_sign: float,
) -> None:
    problem = tmp_path / f"{rule}.prb"
    problem.write_text(
        "(range-insensitive-guidance)\n"
        "*atmos none\n"
        "*earth wgs-72\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=100000 long=0 lat=0 vel=2500 gama=0 psi=45 time=0 mass=1\n"
        f"  *segment 1 {rule}\n"
        "    *integ dt=0.1\n"
        f"    *fly {rule}=0\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1].named
    assert math.isfinite(final["yawi"])
    assert math.isfinite(final["pitchi"])
    assert final["_ria_position_sensitivity"] >= 0.0
    assert final["_ria_time_sensitivity"] * expected_sign > 0.0
####


def test_rail_static_and_sliding_resistance_affect_motion(tmp_path: Path) -> None:
    problem = tmp_path / "rail.prb"
    problem.write_text(
        "(rail)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=0 gama=0 psi=0 time=0 mass=1\n"
        "  *file rail.dat time vel\n"
        "  *segment 1 launch\n"
        "    *integ dt=0.1\n"
        "    *prop thrust=1 mdot=0\n"
        "    *rail launch cfstat=0.5 cfslid=0.1\n"
        "    *when time>0.2 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["vel"] == pytest.approx(0.0, abs=1e-10)
####


def test_multi_parameter_optimization_recomputes_constraint(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    programs = []
    real_builder = lowering_module.build_optimization_problem

    def capture_builder(*args, **kwargs):
        program = real_builder(*args, **kwargs)
        programs.append(program)
        return program
    ####

    monkeypatch.setattr(lowering_module, "build_optimization_problem", capture_builder)
    problem = tmp_path / "optimize-two.prb"
    problem.write_text(
        "(optimize-two)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=opta-1 ydt=opta-2 zdt=0 time=0 mass=1\n"
        "  *file optimize-two.dat time xecfc yecfc xecfcdt yecfcdt\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>1 stop\n"
        "*optimize a for xecfc=max on segment 1, trajectory 1\n"
        "  constrain yecfc=10 on segment 1, trajectory 1\n"
        "  par-1=5 lo-1=0 hi-1=20 par-2=0 lo-2=-20 hi-2=20 maxitr=30 tol=0.001\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=200)

    assert report.exit_code == 0
    assert programs
    assert programs[0].parameters == ("par-1", "par-2")
    assert programs[0].bounds == ((0.0, 20.0), (-20.0, 20.0))
    assert programs[0].equality_constraints == ("yecfc=10",)
    final = report.results[0].states["1"][-1]
    assert final.named["x"] == pytest.approx(20.0, abs=0.2)
    assert final.named["y"] == pytest.approx(10.0, abs=0.2)
####


def test_optimization_parameter_indices_are_numeric() -> None:
    indices = ("10", "2", "1", "alpha")

    assert tuple(sorted(indices, key=lowering_module._optimization_parameter_index)) == ("1", "2", "10", "alpha")
####


def test_optimization_constraint_qualifier_applies_to_unqualified_left_endpoint(tmp_path: Path) -> None:
    problem = tmp_path / "optimize-qualified-constraint.prb"
    problem.write_text(
        "(optimize-qualified-constraint)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=opta-1 ydt=opta-2 zdt=0 time=0 mass=1\n"
        "  *segment 1 first\n"
        "    *integ dt=0.1\n"
        "    *when time=1 goto 2\n"
        "  *segment 2 second\n"
        "    *integ dt=0.1\n"
        "    *when time=2 stop\n"
        "*optimize a for xecfc=max on segment 2, trajectory 1\n"
        "  constrain yecfc=10 on segment 1, trajectory 1\n"
        "  par-1=5 lo-1=0 hi-1=20 par-2=0 lo-2=-20 hi-2=20 maxitr=20 tol=0.001\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=200)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["y"] == pytest.approx(20.0, abs=0.2)
    assert final.named["x"] == pytest.approx(40.0, abs=0.2)
####


def test_optimization_rejects_an_unsatisfied_endpoint_constraint(tmp_path: Path) -> None:
    problem = tmp_path / "optimize-infeasible.prb"
    problem.write_text(
        "(optimize-infeasible)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=opta-1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>1 stop\n"
        "*optimize a for xecfc=max on segment 1, trajectory 1\n"
        "  constrain xecfc=10 on segment 1, trajectory 1\n"
        "  par-1=0 lo-1=0 hi-1=1 maxitr=10 tol=0.001\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 2
    assert any(item.code == "runtime-execution-failed" and "constraints" in item.message for item in report.diagnostics)
####


def test_multiple_optimization_loops_keep_parameter_scopes_separate(tmp_path: Path) -> None:
    problem = tmp_path / "optimize-two-loops.prb"
    problem.write_text(
        "(optimize-two-loops)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=opta-1 ydt=optb-1 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>1 stop\n"
        "*optimize a for xecfc=max on segment 1, trajectory 1\n"
        "  par-1=1 lo-1=0 hi-1=10 maxitr=10 tol=0.001\n"
        "*optimize b for yecfc=max on segment 1, trajectory 1\n"
        "  par-1=2 lo-1=0 hi-1=20 maxitr=10 tol=0.001\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["x"] == pytest.approx(10.0, abs=0.2)
    assert final.named["y"] == pytest.approx(20.0, abs=0.2)
####


def test_optimization_recomputes_indexed_cross_trajectory_constraint(tmp_path: Path) -> None:
    problem = tmp_path / "cross-trajectory-optimization.prb"
    problem.write_text(
        "(cross-trajectory-optimization)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 parent start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=opta-1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>1 stop\n"
        "*trajectory 2 target start on 1\n"
        "  *initial ecfc x=5 y=0 z=0 xdt=0 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>1 stop\n"
        "*optimize a for xecfc=max on segment 1, trajectory 1\n"
        "  constrain x[1] on segment 1 = x[2] on segment 1\n"
        "  par-1=1 lo-1=0 hi-1=10 maxitr=10 tol=0.01\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    assert report.results[0].states["1"][-1].named["xdt"] == pytest.approx(5.0, abs=0.1)
####


def test_ecfc_fixture_integrates_cartesian_state_and_alias_outputs(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p001_linear_ecfc_zero_force/input/p001_linear_ecfc_zero_force.prb"
    report = run_files(problem, output_dir=tmp_path)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["x"] == pytest.approx(20925746.3255)
    assert final.named["y"] == pytest.approx(80.0)
    assert "10.0 20925746.3255 80.0 -40.0 10.0 -2.0 1.0 10.0" in (tmp_path / "linear.dat").read_text(encoding="utf-8")
####


def test_goto_applies_segment_reset_and_increment(tmp_path: Path) -> None:
    problem = tmp_path / "segments.prb"
    problem.write_text(
        "(segments)\n"
        "*trajectory 1 rocket start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=0 ydt=0 zdt=0 time=0 wt=1\n"
        "  *file segments.dat time wt vel\n"
        "  *segment 1 first\n"
        "    *integ dt=0.1\n"
        "    *when time>1 goto 2\n"
        "  *segment 2 second\n"
        "    *reset wt=2\n"
        "    *increment vel=3\n"
        "    *when time>2 stop\n"
        "*end\n",
        encoding="utf-8",
    )
    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)
    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.time == 2.0
    assert final.named["wt"] == 2.0
    assert final.named["vel"] == 3.0
    history_lines = (tmp_path / "out" / "segments.dat").read_text(encoding="utf-8").splitlines()
    assert any(line.startswith("1.1 ") and " 2.0 3.0" in line for line in history_lines)
####


def test_file_output_retains_exact_pre_transition_state(tmp_path: Path) -> None:
    problem = tmp_path / "segment-output-boundary.prb"
    problem.write_text(
        "(segment-output-boundary)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=1 ydt=0 zdt=0 time=0 wt=1\n"
        "  *file transitions.dat time wt\n"
        "  *segment 1 handoff\n"
        "    *integ dt=0.2\n"
        "    *when time=1 goto 2\n"
        "  *segment 2 coast\n"
        "    *reset wt=10\n"
        "    *when time>1.2 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    rows = (tmp_path / "out" / "transitions.dat").read_text(encoding="utf-8").splitlines()
    assert [row.split() for row in rows if row.startswith("1.0 ")] == [["1.0", "1.0"], ["1.0", "10.0"]]
####


def test_segment_resets_precede_increments_even_when_source_order_is_mixed(tmp_path: Path) -> None:
    problem = tmp_path / "mixed-discontinuities.prb"
    problem.write_text(
        "(mixed-discontinuities)\n"
        "*trajectory 1 rocket start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=0 ydt=0 zdt=0 time=0 wt=1\n"
        "  *segment 1 first\n"
        "    *integ dt=1\n"
        "    *when time>0 goto 2\n"
        "  *segment 2 second\n"
        "    *increment wt=1\n"
        "    *reset wt=10\n"
        "    *increment wt=2\n"
        "    *when time>1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=20)

    assert report.exit_code == 0
    assert report.results[0].states["1"][1].named["wt"] == pytest.approx(13.0)
####


def test_goto_resets_segment_clock_in_integrated_state(tmp_path: Path) -> None:
    problem = tmp_path / "segment-clock.prb"
    problem.write_text(
        "(segment-clock)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=0 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 first\n"
        "    *integ dt=0.1\n"
        "    *when time=1 goto 2\n"
        "  *segment 2 second\n"
        "    *when tseg>0.5 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    assert report.results[0].states["1"][-1].time == pytest.approx(1.5)
####


def test_goto_switches_to_target_segment_integration_step(tmp_path: Path) -> None:
    problem = tmp_path / "segment-steps.prb"
    problem.write_text(
        "(segment-steps)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 rocket start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 coarse\n"
        "    *integ dt=0.2\n"
        "    *when time>0.4 goto 2\n"
        "  *segment 2 fine\n"
        "    *integ dt=0.05\n"
        "    *when time>0.6 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    history = report.results[0].states["1"]
    segment_two_times = [state.time for state in history if state.named.get("_segment") == 2.0]
    assert segment_two_times[-1] == pytest.approx(0.6)
    assert any((right - left) == pytest.approx(0.05) for left, right in zip(segment_two_times, segment_two_times[1:], strict=False))
####


def test_goto_does_not_reapply_source_segment_stop_condition(tmp_path: Path) -> None:
    problem = tmp_path / "segment-event-scope.prb"
    problem.write_text(
        "(segment-event-scope)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 handoff\n"
        "    *integ dt=0.1\n"
        "    *when time>0.2 goto 2\n"
        "  *segment 2 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>0.8 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    assert report.results[0].states["1"][-1].time == pytest.approx(0.8)
    assert report.results[0].states["1"][-1].named["_segment"] == pytest.approx(2.0)
####


def test_inertial_platform_alignment_exposes_fixed_platform_observables(tmp_path: Path) -> None:
    problem = tmp_path / "inertial-platform.prb"
    problem.write_text(
        "(inertial-platform)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=100 y=0 z=0 xdt=1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *file platform.dat time xip yip zip xipdt yipdt zipdt\n"
        "  *segment 1 coast\n"
        "    *inertial ecfc\n"
        "    *integ dt=0.5 dtprnt=0.5\n"
        "    *when time=0.5 goto 2\n"
        "  *segment 2 coast\n"
        "    *inertial ecfc\n"
        "    *integ dt=0.5 dtprnt=0.5\n"
        "    *when time=1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=20)

    assert report.exit_code == 0
    rows = (tmp_path / "out/platform.dat").read_text(encoding="utf-8").splitlines()
    assert rows[0] == "time xip yip zip xipdt yipdt zipdt"
    assert rows[1].split() == ["0.0", "0.0", "0.0", "0.0", "1.0", "0.0", "0.0"]
    assert rows[2].split() == ["0.5", "0.0", "0.0", "0.0", "1.0", "0.0", "0.0"]
    assert rows[-1].split() == ["1.0", "0.5", "0.0", "0.0", "1.0", "0.0", "0.0"]
    ####


def test_inertial_platform_explicit_geodetic_origin_is_used(tmp_path: Path) -> None:
    problem = tmp_path / "inertial-origin.prb"
    problem.write_text(
        "(inertial-origin)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=100 y=0 z=0 xdt=0 ydt=0 zdt=0 time=0 mass=1\n"
        "  *file platform.dat time xip yip zip\n"
        "  *segment 1 coast\n"
        "    *inertial geodetic\n"
        "      long=0 lat=0 alt=-20925546.3255\n"
        "    *integ dt=0.1\n"
        "    *when time=0 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=10)

    assert report.exit_code == 0
    row = (tmp_path / "out/platform.dat").read_text(encoding="utf-8").splitlines()[1].split()
    assert row == ["0.0", "0.0", "0.0", "0.0"]
    ####


def test_initially_satisfied_when_stops_before_first_integration_step(tmp_path: Path) -> None:
    problem = tmp_path / "initial-event.prb"
    problem.write_text(
        "(initial-event)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=1\n"
        "    *when time=0 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=10)

    assert report.exit_code == 0
    history = report.results[0].states["1"]
    assert len(history) == 1
    assert history[0].time == pytest.approx(0.0)
####


def test_stop_when_does_not_apply_another_when_block_goto_target(tmp_path: Path) -> None:
    problem = tmp_path / "mixed-events.prb"
    problem.write_text(
        "(mixed-events)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 rocket start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 first\n"
        "    *integ dt=0.1\n"
        "    *when time>0.1 stop\n"
        "    *when time>0.2 goto 2\n"
        "  *segment 2 second\n"
        "    *when time>1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.time == pytest.approx(0.1)
    assert final.named["tseg"] == pytest.approx(0.1)
####


def test_inherited_trajectory_activates_at_source_segment(tmp_path: Path) -> None:
    problem = tmp_path / "branch.prb"
    problem.write_text(
        "(branch)\n"
        "*trajectory 1 parent start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>1 goto 2\n"
        "  *segment 2 handoff\n"
        "    *when tseg>0 stop\n"
        "*trajectory 2 child start on 1\n"
        "  *initial from trajectory 1, segment 2\n"
        "  *segment 1 child\n"
        "    *integ dt=0.1\n"
        "    *when time>2 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    child = report.results[0].states["2"]
    assert child[0].time == pytest.approx(1.0)
    assert child[0].named["x"] == pytest.approx(1.0)
    assert child[-1].time == pytest.approx(2.0)
####


def test_run_cli_returns_input_error_and_writes_json_report(tmp_path: Path, capsys) -> None:
    report = tmp_path / "run.json"
    exit_code = main(["run", str(tmp_path / "missing.prb"), "--report", str(report), "--json"])
    assert exit_code == 2
    assert json.loads(report.read_text(encoding="utf-8"))["diagnostics"][0]["code"] == "problem-ingest-failed"
    assert "problem-ingest-failed" in capsys.readouterr().out
####


def test_run_files_reports_output_directory_errors(tmp_path: Path) -> None:
    output_path = tmp_path / "not-a-directory"
    output_path.write_text("occupied", encoding="utf-8")

    report = run_files(PROBLEM, output_dir=output_path)

    assert report.exit_code == 2
    assert [item.code for item in report.diagnostics] == ["output-dir-failed"]
    assert report.outputs == ()
####


def test_file_runtime_executes_bounded_optimization_by_recomputing_trajectory(tmp_path: Path) -> None:
    problem = tmp_path / "optimization.prb"
    problem.write_text(
        "(optimization)\n"
        "*earth spherical gm=0 omega=0\n"
        "*atmos none\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=opta-1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *file result.dat time xecfc\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time=1 stop\n"
        "*optimize a for xecfc=max on segment 1, trajectory 1\n"
        "  fref=1 maxitr=20 tol=1e-6\n"
        "  par-1=0 lo-1=-1 hi-1=1\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    assert report.cases == 1
    assert report.results[0].completed
    assert report.results[0].states["1"][-1].named["x"] == pytest.approx(1.0, abs=1e-3)
    assert (tmp_path / "out/result.dat").exists()
    assert not any(item.code == "unsupported-runtime-feature" for item in report.diagnostics)
####


def test_lowering_marks_optimization_as_unsupported() -> None:
    document = parse_problem_text(
        "(optimization)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial ecfc x=0 y=0 z=0 xdt=0 ydt=0 zdt=0 mass=1\n"
        "*segment 1 coast\n"
        "*when time=1 stop\n"
        "*optimize xecfc=max on segment 1, trajectory 1\n"
        "*end\n"
    )

    lowered = lower_problem_document(document)

    assert lowered.unsupported_features == ("optimize",)
####


def test_file_output_respects_dtprnt_without_losing_terminal_state(tmp_path: Path) -> None:
    problem = tmp_path / "print-interval.prb"
    problem.write_text(
        "(print-interval)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *file cadence.dat time xecfc\n"
        "  *segment 1 coast\n"
        "    *integ dtprnt=0.5 dt=0.1\n"
        "    *when time>1.2 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    times = [float(line.split()[0]) for line in (tmp_path / "out" / "cadence.dat").read_text(encoding="utf-8").splitlines()[1:]]
    assert times == pytest.approx([0.0, 0.5, 1.0, 1.2])
    ####


def test_file_runtime_applies_units_and_formats_at_input_and_output_boundaries(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p023_units_format/input/p023_units_format.prb"

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=300)

    assert report.exit_code == 0
    rows = (tmp_path / "out" / "units.dat").read_text(encoding="utf-8").splitlines()
    assert rows[0] == "time xecfc xecfcdt"
    assert rows[1].split() == ["0.00", "6378.137000", "10.000"]
    assert rows[-1].split() == ["10.00", "6378.237000", "10.000"]
    ####
