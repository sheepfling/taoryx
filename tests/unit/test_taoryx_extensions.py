from pathlib import Path

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language.ingest import FileKind, ingest_file
from taoryx.language.models import DefineBlock, DofDirectiveBlock, ExtensionBlock, TableDocument, WindBlock
from taoryx.runtime.runner import run_files

CORPUS = Path(__file__).parents[2] / "examples" / "taoryx" / "syntax_fragments"


def _errors(path: Path, *, profile: GrammarProfile = GrammarProfile.TAORYX) -> list[str]:
    document = ingest_file(path, profile=profile)
    return [diagnostic.code for diagnostic in document.diagnostics if diagnostic.severity.value == "error"]


def test_taoryx_focused_problem_corpus_is_grammar_complete() -> None:
    fixtures = sorted(CORPUS.glob("*.prb"))
    assert len(fixtures) == 11
    assert all(_errors(path) == [] for path in fixtures)


def test_successor_blocks_and_aliases_are_typed() -> None:
    document = ingest_file(CORPUS / "05_propulsion_rail.prb", profile=GrammarProfile.TAORYX).document
    problem = document.problems[0]
    dof = next(block for block in problem.blocks if isinstance(block, DofDirectiveBlock))
    method = next(block for block in problem.blocks if isinstance(block, ExtensionBlock) and block.keyword == "method")
    segment = problem.trajectories[0].segments[0]
    mass = next(block for block in segment.blocks if isinstance(block, ExtensionBlock) and block.keyword == "mass")
    assert dof.keyword == "sixdof"
    assert dof.mode == "rigid-body-6dof"
    assert method.method == "rk4-fixed"
    assert "variable-mass" in method.options
    assert [assignment.name for assignment in mass.assignments] == ["xcg", "ixx", "iyy", "izz"]


def test_deployment_cases_helpers_and_file_wind_reach_the_ast() -> None:
    deployment = ingest_file(CORPUS / "06_deployment.prb", profile=GrammarProfile.TAORYX).document
    deployed = deployment.problems[0].trajectories[1].blocks[0]
    assert isinstance(deployed, ExtensionBlock)
    assert deployed.keyword == "deployed"
    assert (deployed.source_trajectory, deployed.source_segment) == (1, 1)

    helpers = ingest_file(CORPUS / "07_surface_helpers.prb", profile=GrammarProfile.TAORYX).document
    define = helpers.problems[0].blocks[0]
    assert isinstance(define, DefineBlock)
    assert define.declaration == "initial variables"
    assert define.helper_calls == ["surface_ref(launch_lat,launch_long);"]

    wind = ingest_file(CORPUS / "09_environment.prb", profile=GrammarProfile.TAORYX).document
    wind_block = next(block for block in wind.problems[0].blocks if isinstance(block, WindBlock))
    assert (wind_block.filename, wind_block.units) == ("environment_wind.dat", "ft/sec")


def test_taoryx_extensions_remain_outside_taos96_profile() -> None:
    document = ingest_file(CORPUS / "01_dynamics_method.prb", profile=GrammarProfile.TAOS96)
    assert any(diagnostic.code == "taoryx-extension-requires-profile" for diagnostic in document.diagnostics)


def test_successor_table_families_are_accepted() -> None:
    document = ingest_file(CORPUS / "04_aero_tables.tbl", profile=GrammarProfile.TAORYX)
    assert document.kind is FileKind.TABLE
    assert isinstance(document.document, TableDocument)
    assert [table.table_type for table in document.document.tables] == ["aero_force", "aero_moment", "inertia"]
    assert not [diagnostic for diagnostic in document.diagnostics if diagnostic.severity.value == "error"]


def test_successor_table_families_require_taoryx_profile() -> None:
    document = ingest_file(CORPUS / "04_aero_tables.tbl", profile=GrammarProfile.TAOS96)
    assert any(diagnostic.code == "taoryx-extension-requires-profile" for diagnostic in document.diagnostics)


def test_full_taoryx_examples_lower_and_execute_without_runtime_errors(tmp_path: Path) -> None:
    examples = sorted((CORPUS.parent / "full_examples").glob("*.prb"))
    assert len(examples) == 8
    for problem in examples:
        report = run_files(problem, output_dir=tmp_path / problem.stem, max_steps=20, profile=GrammarProfile.TAORYX)
        assert report.cases >= 1
        assert not [item for item in report.diagnostics if item.code == "runtime-execution-failed"]
        assert report.artifacts
