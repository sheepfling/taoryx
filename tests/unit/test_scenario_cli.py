from __future__ import annotations

import json
from pathlib import Path

from taoryx.runtime.cli import main

ROOT = Path(__file__).resolve().parents[2]


def test_scenario_compile_cli_writes_cache(tmp_path: Path, capsys) -> None:
    output = tmp_path / "scenario.json"
    exit_code = main(
        [
            "scenario",
            "compile",
            str(ROOT / "examples/chapter04/ballistic-reentry.prb"),
            str(ROOT / "examples/chapter04/ballistic-reentry.tbl"),
            "--output",
            str(output),
            "--seed",
            "1729",
            "--json",
        ]
    )

    assert exit_code == 0
    assert json.loads(output.read_text(encoding="utf-8"))["identity"]
    assert json.loads(capsys.readouterr().out)["request"]["seed"] == 1729
    ####
