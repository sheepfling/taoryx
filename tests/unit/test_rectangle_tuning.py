from __future__ import annotations

from tools.tune_rectangle_controller import inject_controls


def test_inject_controls_replaces_existing_and_adds_missing_runtime_attributes() -> None:
    source = (
        "*runtime status route mode=rectangle rectangle-bank-deg=15\n"
        "*runtime status guidance rectangle-coordinated-turn=false\n"
    )

    tuned = inject_controls(
        source,
        {
            "rectangle-bank-deg": 22.5,
            "rectangle-heading-gain-nm-per-rad": 125000.0,
        },
    )

    assert "rectangle-bank-deg=22.5" in tuned
    assert "rectangle-heading-gain-nm-per-rad=125000" in tuned
    assert "rectangle-coordinated-turn=true" in tuned
    ####


def test_inject_controls_requires_a_guidance_status_line_for_missing_attributes() -> None:
    source = "*runtime status route mode=rectangle\n"

    try:
        inject_controls(source, {"rectangle-bank-gain-nm-per-rad": 10.0})
    except ValueError as error:
        assert "guidance line" in str(error)
    else:
        raise AssertionError("missing guidance status line should fail closed")
    ####
