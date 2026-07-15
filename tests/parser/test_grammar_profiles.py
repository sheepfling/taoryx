from __future__ import annotations

from taoryx.language import GrammarProfile, parse_problem_text


def test_default_problem_profile_is_taos96() -> None:
    document = parse_problem_text("(legacy)\n*end\n")

    assert document.grammar_profile is GrammarProfile.TAOS96
    assert document.diagnostics == []
    ####


def test_taoryx_profile_is_explicit_for_successor_extensions() -> None:
    document = parse_problem_text(
        "(successor)\n*mode rigid-body-6dof\n*end\n",
        profile=GrammarProfile.TAORYX,
    )

    assert document.grammar_profile is GrammarProfile.TAORYX
    assert document.problems[0].blocks[0].keyword == "mode"
    assert document.problems[0].blocks[0].mode == "rigid-body-6dof"
    assert not [item for item in document.diagnostics if item.severity.value == "error"]
    ####


def test_profile_string_alias_is_supported_for_cli_and_configuration_callers() -> None:
    document = parse_problem_text("(successor)\n*mode point-mass\n*end\n", profile="taoryx")

    assert document.grammar_profile == GrammarProfile.TAORYX
    ####


def test_dof_directives_are_successor_only_and_map_to_runtime_modes() -> None:
    legacy = parse_problem_text("(legacy)\n*6dof\n*end\n")
    successor = parse_problem_text("(successor)\n*6dof\n*end\n", profile="taoryx")

    assert any(item.code == "taoryx-extension-requires-profile" for item in legacy.diagnostics)
    assert successor.diagnostics == []
    assert successor.problems[0].blocks[0].mode == "rigid-body-6dof"
    ####


def test_runtime_declarations_are_successor_only_and_typed() -> None:
    legacy = parse_problem_text("(legacy)\n*runtime control throttle\n*end\n")
    successor = parse_problem_text(
        """(successor)
*runtime control throttle unit=fraction default=0.5 lower=0 upper=1 slew=2
*runtime status altitude source=alt unit=m
*runtime event ground condition=alt<0 action=stop
*runtime output channels=alt,vel interval=0.5 events=true
*end
""",
        profile=GrammarProfile.TAORYX,
    )

    assert any(item.code == "taoryx-extension-requires-profile" for item in legacy.diagnostics)
    assert successor.diagnostics == []
    declarations = successor.problems[0].blocks
    assert [block.declaration for block in declarations] == ["control", "status", "event", "output"]
    assert declarations[0].attributes["upper"] == "1"
    assert declarations[2].attributes["condition"] == "alt<0"
    ####
