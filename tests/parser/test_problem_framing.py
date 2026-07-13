from taoryx.language.problem_parser import parse_problem_text


def test_end_closes_one_problem_and_allows_the_next_problem() -> None:
    document = parse_problem_text("(first)\n*title first\n*end\n(second)\n*title second\n*end\n")

    assert [problem.name for problem in document.problems] == ["first", "second"]
    assert all(problem.ended for problem in document.problems)
    assert not [diagnostic for diagnostic in document.diagnostics if diagnostic.severity == "error"]


def test_block_after_end_is_not_attached_to_the_closed_problem() -> None:
    document = parse_problem_text("(demo)\n*end\n*title invalid\n")

    assert [block.keyword for block in document.problems[0].blocks] == []
    assert any(diagnostic.code == "block-after-end" for diagnostic in document.diagnostics)


def test_end_before_a_problem_is_an_error() -> None:
    document = parse_problem_text("*end\n")

    assert {diagnostic.code for diagnostic in document.diagnostics} >= {"end-before-problem", "missing-problem"}


def test_define_assignment_can_span_lines_until_semicolon() -> None:
    document = parse_problem_text("(demo)\n*define x\nvalue = alt +\n  3.0D+02;\n*end\n")

    assignment = document.problems[0].blocks[0].assignments[0]
    assert assignment.name == "value"


def test_define_control_statement_is_typed_and_preserved() -> None:
    document = parse_problem_text("(demo)\n*define x\nif (alt > 100) then value = 1;\nelse value = 0;\n*end\n")

    block = document.problems[0].blocks[0]
    assert [control.kind for control in block.control_statements] == ["if", "else"]
    assert block.control_statements[0].assignment is not None
    assert not [diagnostic for diagnostic in document.diagnostics if diagnostic.severity == "error"]


def test_define_statements_preserve_source_order() -> None:
    document = parse_problem_text("(demo)\n*define x\na = 1; if (alt > 100) then b = 2; else b = 0;\n*end\n")

    block = document.problems[0].blocks[0]
    assert [statement.kind for statement in block.typed_statements] == ["assignment", "if", "else"]
    assert [assignment.name for assignment in block.assignments] == ["a"]


def test_nested_define_control_is_retained() -> None:
    document = parse_problem_text("(demo)\n*define x\nif (alt > 100) then if (vel < 2) then value = 1;\n*end\n")

    control = document.problems[0].blocks[0].control_statements[0]
    assert control.nested is not None
    assert control.nested.assignment is not None


def test_malformed_define_control_gets_a_diagnostic() -> None:
    document = parse_problem_text("(demo)\n*define x\nif alt > ;\n*end\n")

    assert any(diagnostic.code == "invalid-define-control" for diagnostic in document.diagnostics)


def test_recovery_reports_multiple_errors_and_retains_source_records() -> None:
    document = parse_problem_text("(first)\n*unknown\n*trajectory malformed\n*end\n(second)\n*segment 1\n*end\n")

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert codes.count("unknown-block") == 1
    assert "invalid-trajectory-header" in codes
    assert "segment-outside-trajectory" in codes
    assert len(document.recovered_records) >= 3
    assert [problem.name for problem in document.problems] == ["first", "second"]


def test_recovery_continues_after_multiple_bad_lines_in_one_file() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory malformed\n"
        "*when altitude > goto 2\n"
        "*initial polar\n"
        "not-an-assignment\n"
        "*unknown\n"
        "*end\n"
    )

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert codes.count("invalid-trajectory-header") == 1
    assert "invalid-when-condition" in codes
    assert "invalid-initial-header" in codes
    assert "orphan-line" in codes
    assert "unknown-block" in codes
    assert len(document.recovered_records) >= 5


def test_when_block_has_typed_condition_and_action() -> None:
    document = parse_problem_text("(demo)\n*trajectory 1 vehicle start on 1\n*segment 1\n*when time > 10 goto 2\n*end\n")

    block = document.problems[0].trajectories[0].segments[0].blocks[0]
    assert block.keyword == "when"
    assert block.action == "goto"
    assert block.target_segment == 2
    assert block.condition is not None


def test_malformed_when_block_is_diagnosed_and_preserved() -> None:
    document = parse_problem_text("(demo)\n*trajectory 1 vehicle start on 1\n*segment 1\n*when time > 10 continue\n*end\n")

    assert any(diagnostic.code == "invalid-when-statement" for diagnostic in document.diagnostics)
    assert any(record.code == "invalid-when-statement" and record.text.startswith("*when") for record in document.recovered_records)


def test_initial_block_parses_coordinate_and_copied_forms() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial geodetic\n"
        "alt = 1000\n"
        "wt = 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*end\n"
    )

    first, second = document.problems[0].trajectories[0].blocks[0], document.problems[0].trajectories[0].blocks[1]
    assert first.coordinate_system == "geodetic"
    assert first.assignments[0].name == "alt"
    assert second.source_trajectory == 1
    assert second.source_segment == 1


def test_assignment_only_trajectory_blocks_validate_documented_parameters() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*dwn/crs latgd=10 long=95 azm=45\n"
        "*iip iip_beta=550 iip_alt=1000\n"
        "*tangent latgd=21.982 long=-159.759 alt=87.2 azm=140\n"
        "*segment 1\n"
        "*end\n"
    )

    blocks = document.problems[0].trajectories[0].blocks
    assert [assignment.name for assignment in blocks[0].assignments] == ["latgd", "long", "azm"]
    assert [assignment.name for assignment in blocks[1].assignments] == ["iip_beta", "iip_alt"]
    assert [assignment.name for assignment in blocks[2].assignments] == ["latgd", "long", "alt", "azm"]
    assert not document.diagnostics


def test_assignment_only_block_preserves_malformed_header_and_recovers() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*dwn/crs latgd= long=95 unknown=4 trailing\n"
        "*segment 1\n"
        "*end\n"
    )

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "invalid-block-header" in codes
    assert "unsupported-block-parameter" in codes
    assert any(record.code == "invalid-block-header" for record in document.recovered_records)
    assert document.problems[0].trajectories[0].blocks[0].header == "latgd= long=95 unknown=4 trailing"


def test_segment_assignment_blocks_reject_unparsed_header_text() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1\n"
        "*aero ca=(drag) stray\n"
        "*constants gravity=32.2\n"
        "*cg cg=0.5\n"
        "*integ dt=0.1 dtprnt=1.0\n"
        "*prop thrust=(thrust) mdot=(mdot)\n"
        "*reset wt=100\n"
        "*increment wt=-1\n"
        "*end\n"
    )

    assert sum(diagnostic.code == "invalid-block-header" for diagnostic in document.diagnostics) == 1
    assert any(record.code == "invalid-block-header" for record in document.recovered_records)
    assert len(document.problems[0].trajectories[0].segments[0].blocks) == 7


def test_limits_block_parses_multiple_relationships() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1\n"
        "*limits bankgd>-60 bankgd<60 alpha>-15 alpha<15.0\n"
        "*end\n"
    )

    limits = document.problems[0].trajectories[0].segments[0].blocks[0].limits
    assert [(item.variable, item.operator) for item in limits] == [("bankgd", ">"), ("bankgd", "<"), ("alpha", ">"), ("alpha", "<")]
    assert not document.diagnostics


def test_limits_block_recovers_malformed_relationships() -> None:
    document = parse_problem_text("(demo)\n*trajectory 1 vehicle start on 1\n*segment 1\n*limits alpha<\n*end\n")

    assert any(diagnostic.code == "invalid-limits-header" for diagnostic in document.diagnostics)
    assert any(record.code == "invalid-limits-header" for record in document.recovered_records)


def test_fly_block_parses_documented_guidance_forms() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1\n"
        "*fly alpha=5.0\n"
        "*fly alpha vrs tseg interp-2\n"
        "*fly l/d-max\n"
        "*end\n"
    )

    blocks = document.problems[0].trajectories[0].segments[0].blocks
    assert blocks[0].value is not None
    assert blocks[1].reference == "tseg"
    assert blocks[1].interpolation == "interp-2"
    assert blocks[2].guidance_variable == "l/d-max"
    assert not document.diagnostics


def test_fly_and_rail_malformed_forms_are_recovered() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1\n"
        "*fly alpha vrs\n"
        "*rail catapult cfstat=0.1\n"
        "*end\n"
    )

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "invalid-fly-statement" in codes
    assert "invalid-rail-header" in codes
    assert len(document.recovered_records) >= 2


def test_optimize_constraints_and_controls_are_typed() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*optimize a for vel=max on segment 6, trajectory 1\n"
        "constrain vel=3000\n"
        "constrain alt on segment 3, trajectory 2 = 3000\n"
        "constrain east[2] on segment 4 = east[1] on segment 7, ref=1000\n"
        "constrain gamgd on segment 6, trajectory 1\n"
        " = gamgd on segment 4, trajectory 3\n"
        "fref=100 maxitr=40 tol=1.0e-7\n"
        "par-1=9.9 lo-1=-15 hi-1=15\n"
        "*end\n"
    )

    block = document.problems[0].blocks[0]
    assert len(block.constraints) == 4
    assert block.constraints[0].left.text == "vel"
    assert block.constraints[1].left.segment == 3
    assert block.constraints[2].right.trajectory_subscript == 1
    assert block.constraints[2].reference is not None
    assert [assignment.name for assignment in block.controls] == ["fref", "maxitr", "tol", "par-1", "lo-1", "hi-1"]
    assert not document.diagnostics


def test_optimize_constraint_recovery_continues_to_controls() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*optimize a for vel=max on segment 1, trajectory 1\n"
        "constrain missing endpoint\n"
        "fref=100 maxitr=2\n"
        "*end\n"
    )

    block = document.problems[0].blocks[0]
    assert any(diagnostic.code == "invalid-optimize-constraint" for diagnostic in document.diagnostics)
    assert [assignment.name for assignment in block.controls] == ["fref", "maxitr"]
    assert any(record.code == "invalid-optimize-constraint" for record in document.recovered_records)


def test_optimize_header_and_control_diagnostics_preserve_source() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*optimize malformed\n"
        "foo=1\n"
        "*end\n"
    )

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "invalid-optimize-header" in codes
    assert "unsupported-optimize-control" in codes
    assert document.problems[0].blocks[0].header == "malformed"
    assert document.problems[0].blocks[0].controls[0].name == "foo"


def test_inertial_block_parses_documented_alignment_forms() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*inertial platform alignment geocentric\n"
        "*inertial platform lat=28.8 long=-81.5 alt=550 time=100\n"
        "*inertial ecfc eastx=0.7071 easty=0.7071 eastz=0.0 downz=-1\n"
        "*segment 1\n"
        "*end\n"
    )

    blocks = document.problems[0].trajectories[0].blocks
    assert blocks[0].alignment == "geocentric"
    assert blocks[0].coordinate_system == "geocentric"
    assert blocks[1].alignment == "geodetic"
    assert blocks[2].alignment == "ecfc"
    assert not document.diagnostics


def test_inertial_block_rejects_ambiguous_alignment_text() -> None:
    document = parse_problem_text("(demo)\n*trajectory 1 vehicle start on 1\n*inertial polar\n*segment 1\n*end\n")

    assert any(diagnostic.code == "invalid-inertial-header" for diagnostic in document.diagnostics)
    assert any(record.code == "invalid-inertial-header" for record in document.recovered_records)


def test_survey_block_parses_incremental_and_explicit_values() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*survey 1 alt0\n"
        "lo=40000 hi=80000 inc=20000\n"
        "vals=35000,45000\n"
        "*end\n"
    )

    block = document.problems[0].blocks[0]
    assert block.survey_id == 1
    assert block.name == "alt0"
    assert [(setting.name, setting.values) for setting in block.settings] == [
        ("lo", ["40000"]),
        ("hi", ["80000"]),
        ("inc", ["20000"]),
        ("vals", ["35000", "45000"]),
    ]
    assert not document.diagnostics


def test_survey_block_recovers_bad_header_and_setting() -> None:
    document = parse_problem_text("(demo)\n*survey missing\nlo=one unknown=2\n*end\n")

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "invalid-survey-header" in codes
    assert "invalid-survey-value" in codes
    assert "invalid-survey-setting" in codes
    assert len(document.recovered_records) >= 2


def test_search_block_parses_objective_continuation_and_controls() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*search 1 vary pitch-rate until pitchi on segment 2\n"
        " = pitchi on segment 3, trajectory 2\n"
        "xlo=0.5 xhi=10.0 xest=5.0 dx=1.0 tol=0.001\n"
        "xref=1.0 fref=10000 maxitr=20 print=1 integ=0\n"
        "*end\n"
    )

    block = document.problems[0].blocks[0]
    assert block.search_id == 1
    assert block.variable == "pitch-rate"
    assert block.objective is not None
    assert block.objective.operator == "="
    assert block.objective.left.segment == 2
    assert block.objective.right.segment == 3
    assert block.objective.right.trajectory == 2
    assert [assignment.name for assignment in block.controls] == [
        "xlo", "xhi", "xest", "dx", "tol", "xref", "fref", "maxitr", "print", "integ"
    ]
    assert not document.diagnostics


def test_search_block_recovers_incomplete_objective_and_unknown_control() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*search 1 vary alpha until alt on segment 3\n"
        "unknown=1\n"
        "*end\n"
    )

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "invalid-search-objective" in codes
    assert "unsupported-search-control" in codes
    assert any(record.code == "invalid-search-objective" for record in document.recovered_records)


def test_radar_block_parses_station_identity_shape_and_parameters() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*radar 2 station_y alt=112.4 long=65.345 latgd=12.435 wgs-72 distn=25.5 diste=15.2 distd=-10.3\n"
        "*end\n"
    )

    block = document.problems[0].blocks[0]
    assert block.radar_id == 2
    assert block.station_name == "station_y"
    assert block.earth_shape == "wgs-72"
    assert [assignment.name for assignment in block.assignments] == ["alt", "long", "latgd", "distn", "diste", "distd"]
    assert not document.diagnostics


def test_radar_block_recovers_missing_identity_and_unknown_parameter() -> None:
    document = parse_problem_text("(demo)\n*radar missing alt=1 unknown=2\n*end\n")

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "invalid-radar-header" in codes
    assert any(record.code == "invalid-radar-header" for record in document.recovered_records)


def test_units_format_block_parses_inline_and_continuation_settings() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*units/fmt sref in\n"
        "alt km f.3\n"
        "vel m/sec f.2\n"
        "range km\n"
        "mach e.5\n"
        "*end\n"
    )

    settings = document.problems[0].blocks[0].settings
    assert [(setting.variable, setting.unit, setting.format) for setting in settings] == [
        ("sref", "in", None),
        ("alt", "km", "f.3"),
        ("vel", "m/sec", "f.2"),
        ("range", "km", None),
        ("mach", "e.5", None),
    ]
    assert not document.diagnostics


def test_units_format_block_recovers_malformed_setting() -> None:
    document = parse_problem_text("(demo)\n*units/fmt alt km f.x\nlonely\n*end\n")

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "invalid-units-format" in codes
    assert "invalid-units-format-setting" in codes
    assert len(document.recovered_records) >= 2


def test_atmosphere_and_earth_headers_and_user_rows_are_typed() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*atmos user\n"
        "alt temp pres rho sndspd visc\n"
        "0 548.2 2111.0 0.00224 1147 3.895e-7\n"
        "*earth wgs-84 omega=0\n"
        "*end\n"
    )

    atmos, earth = document.problems[0].blocks
    assert atmos.columns == ["alt", "temp", "pres", "rho", "sndspd", "visc"]
    assert atmos.rows[0][0] == 0.0
    assert earth.model == "wgs-84"
    assert earth.assignments[0].name == "omega"
    assert not document.diagnostics


def test_atmosphere_and_earth_recover_bad_forms() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*atmos 21\n"
        "*atmos site\n"
        "alt pressure rho\n"
        "*atmos site\n"
        "alt pres rho\n"
        "0 bad 0.002\n"
        "*earth unknown omega=0\n"
        "*end\n"
    )

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "invalid-atmos-header" in codes
    assert "invalid-atmos-columns" in codes
    assert "invalid-atmos-row" in codes
    assert "invalid-earth-header" in codes
    assert len(document.recovered_records) >= 4


def test_invalid_initial_form_is_diagnosed_and_recovered() -> None:
    document = parse_problem_text("(demo)\n*trajectory 1 vehicle start on 1\n*initial polar\n*segment 1\n*end\n")

    assert any(diagnostic.code == "invalid-initial-header" for diagnostic in document.diagnostics)
    assert any(record.code == "invalid-initial-header" for record in document.recovered_records)


def test_else_if_chain_is_nested_as_typed_control() -> None:
    document = parse_problem_text("(demo)\n*define x\nif (alt > 100) then value = 1;\nelse if (alt > 50) then value = 2;\nelse value = 0;\n*end\n")

    controls = document.problems[0].blocks[0].control_statements
    assert controls[1].kind == "else"
    assert controls[1].nested is not None
    assert controls[1].nested.kind == "if"
