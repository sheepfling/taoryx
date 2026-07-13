from taoryx.language.models import EgsBlock, FileBlock, PrintBlock, WindBlock
from taoryx.language.problem_parser import parse_problem_text


def test_output_blocks_collect_continuation_variables() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*egs traj.dbf\n"
        "time alt range\n"
        "*file compare.dat\n"
        "time[1] alt[1] alt[2]\n"
        "*print\n"
        "time[1] radrng[2][1]\n"
        "*wind geodetic\n"
        "windv=10 windh=90\n"
        "windd=0\n"
        "*end\n"
    )

    problem = document.problems[0]
    egs, file_block, print_block, wind = problem.blocks
    assert isinstance(egs, EgsBlock)
    assert egs.variables == ["time", "alt", "range"]
    assert isinstance(file_block, FileBlock)
    assert file_block.variables == ["time[1]", "alt[1]", "alt[2]"]
    assert isinstance(print_block, PrintBlock)
    assert print_block.variables == ["time[1]", "radrng[2][1]"]
    assert isinstance(wind, WindBlock)
    assert [assignment.name for assignment in wind.assignments] == ["windv", "windh", "windd"]
    assert wind.wind_form == "speed-heading"
    assert not document.diagnostics


def test_egs_summary_form_rejects_variable_body_but_recovers() -> None:
    document = parse_problem_text("(demo)\n*egs summary file.dbf\nalt\n*end\n")

    assert any(diagnostic.code == "invalid-egs-summary" for diagnostic in document.diagnostics)
    assert any(record.code == "invalid-egs-summary" for record in document.recovered_records)
    assert isinstance(document.problems[0].blocks[0], EgsBlock)
    assert document.problems[0].blocks[0].summary is True


def test_problem_output_requires_two_subscripts_for_related_variables() -> None:
    document = parse_problem_text("(demo)\n*print\nradrng[2] relvel[4][1]\n*end\n")

    diagnostic = next(item for item in document.diagnostics if item.code == "invalid-related-output-subscript")
    assert diagnostic.location.line == 3
    assert any(record.code == "invalid-related-output-subscript" for record in document.recovered_records)
    ####


def test_related_output_variables_reject_extra_subscripts() -> None:
    document = parse_problem_text("(demo)\n*print\nradrng[2][1][3] relvel[4][1][2]\n*end\n")

    diagnostics = [item for item in document.diagnostics if item.code == "invalid-related-output-subscript"]
    assert len(diagnostics) == 2
    assert all(record.code == "invalid-related-output-subscript" for record in document.recovered_records)
    ####


def test_trajectory_output_requires_one_subscript_for_related_variables() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*print relvel[2]\n"
        "*segment 1\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    assert not [item for item in document.diagnostics if item.code == "invalid-related-output-subscript"]
    ####


def test_egs_summary_requires_survey_and_summary_variable() -> None:
    document = parse_problem_text("(demo)\n*egs summary file.dbf\n*end\n")

    codes = {diagnostic.code for diagnostic in document.diagnostics}
    assert {"missing-egs-summary-survey", "missing-egs-summary-variable"} <= codes
    assert {record.code for record in document.recovered_records} >= codes
    ####


def test_egs_summary_accepts_documented_survey_and_summary_inputs() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*survey 1 alt vals=1,2\n"
        "*summarize max-alt\n"
        "add max(alt)\n"
        "*egs summary file.dbf\n"
        "*end\n"
    )

    assert not [item for item in document.diagnostics if item.code.startswith("missing-egs-summary-")]
    ####


def test_wind_rejects_mixed_component_sets() -> None:
    document = parse_problem_text("(demo)\n*wind geodetic windv=10 winde=2 windd=0\n*end\n")

    assert any(diagnostic.code == "mixed-wind-components" for diagnostic in document.diagnostics)


def test_wind_component_form_is_typed_without_guessing_mixed_input() -> None:
    valid = parse_problem_text("(demo)\n*wind geocentric winde=1 windn=2 windd=3\n*end\n")
    assert valid.problems[0].blocks[0].wind_form == "east-north"

    invalid = parse_problem_text("(demo)\n*wind geocentric windv=1 winde=2 windd=3\n*end\n")
    assert invalid.problems[0].blocks[0].wind_form is None


def test_missing_output_filename_and_variables_are_file_diagnostics() -> None:
    document = parse_problem_text("(demo)\n*file\n*egs output.dbf\n*end\n")

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "missing-output-filename" in codes
    assert "missing-output-variables" in codes


def test_print_block_enforces_manual_ten_variable_limit_across_continuations() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1\n"
        "*print time alt long latgd range vel gama psi mach wt\n"
        "  mass fuel east north dynprs\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    block = document.problems[0].trajectories[0].segments[0].blocks[0]
    assert len(block.variables) == 15
    diagnostic = next(item for item in document.diagnostics if item.code == "too-many-print-variables")
    assert diagnostic.location.line == 4
    assert any(record.code == "too-many-print-variables" for record in document.recovered_records)
