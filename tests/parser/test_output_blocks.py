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
    assert not document.diagnostics


def test_egs_summary_form_rejects_variable_body_but_recovers() -> None:
    document = parse_problem_text("(demo)\n*egs summary file.dbf\nalt\n*end\n")

    assert any(diagnostic.code == "invalid-egs-summary" for diagnostic in document.diagnostics)
    assert any(record.code == "invalid-egs-summary" for record in document.recovered_records)
    assert isinstance(document.problems[0].blocks[0], EgsBlock)
    assert document.problems[0].blocks[0].summary is True


def test_wind_rejects_mixed_component_sets() -> None:
    document = parse_problem_text("(demo)\n*wind geodetic windv=10 winde=2 windd=0\n*end\n")

    assert any(diagnostic.code == "mixed-wind-components" for diagnostic in document.diagnostics)


def test_missing_output_filename_and_variables_are_file_diagnostics() -> None:
    document = parse_problem_text("(demo)\n*file\n*egs output.dbf\n*end\n")

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "missing-output-filename" in codes
    assert "missing-output-variables" in codes
