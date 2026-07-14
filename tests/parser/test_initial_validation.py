from taoryx.language.models import InitialBlock
from taoryx.language.problem_parser import parse_problem_text
from taoryx.language.semantic_validation import validate_problem


def test_initial_accepts_authoritative_segment_trajectory_copy_form() -> None:
    document = parse_problem_text(
        "(demo)\n"
            "*trajectory 2 vehicle start on 1\n"
            "*initial from segment 6, trajectory 1\n"
            "*segment 1\n"
            "*when time>1 stop\n"
            "*end\n"
    )

    block = document.problems[0].trajectories[0].blocks[0]
    assert isinstance(block, InitialBlock)
    assert block.mode == "from"
    assert block.source_segment == 6
    assert block.source_trajectory == 1
    assert not document.diagnostics


def test_initial_rejects_coordinate_inconsistent_and_conflicting_values() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial ecfc\n"
        "alt=100 x=1 wt=2 mass=3 vel=4 mach=5\n"
        "*end\n"
    )

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "unsupported-initial-parameter" in codes
    assert "conflicting-initial-mass" in codes
    assert "conflicting-initial-velocity" in codes


def test_initial_requires_weight_or_mass_for_direct_state() -> None:
    document = parse_problem_text("(demo)\n*trajectory 1 vehicle start on 1\n*initial geodetic\nalt=100\n*end\n")

    assert any(diagnostic.code == "missing-initial-mass" for diagnostic in document.diagnostics)


def test_initial_assignment_form_defaults_to_geodetic() -> None:
    document = parse_problem_text(
        "(demo)\n"
            "*trajectory 1 vehicle start on 1\n"
            "*initial alt=100 lat=0 long=0 wt=1\n"
            "*segment 1\n"
            "*when time>1 stop\n"
            "*end\n"
    )

    block = document.problems[0].trajectories[0].blocks[0]
    assert isinstance(block, InitialBlock)
    assert block.coordinate_system == "geodetic"
    assert not document.diagnostics


def test_semantic_validation_rejects_unknown_initial_source_segment() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial ecfc\n"
        "x=0 y=0 z=0 wt=1\n"
        "*segment 1\n"
        "*when time>1 stop\n"
        "*trajectory 2 vehicle start on 1\n"
        "*initial from segment 9, trajectory 1\n"
        "*segment 1\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    diagnostics = validate_problem(document)
    assert any(item.code == "unknown-initial-segment" for item in diagnostics)
