import json
from pathlib import Path

from taoryx.language.cli import main


def test_cli_aggregates_files_and_writes_report(tmp_path: Path, monkeypatch, capsys) -> None:
    problem = tmp_path / "broken.prb"
    problem.write_text("(demo)\n*trajectory malformed\n*unknown\n*end\n", encoding="utf-8")
    table = tmp_path / "broken.tbl"
    table.write_text("not a table\n(good)\n", encoding="utf-8")
    report_path = tmp_path / "diagnostics.json"

    monkeypatch.setattr(
        "sys.argv",
        ["taoryx-validate", "--report", str(report_path), str(problem), str(table)],
    )

    assert main() == 1
    output = capsys.readouterr().out
    assert str(problem) in output
    assert str(table) in output
    assert "invalid-trajectory-header" in output
    assert "missing-table-name" in output

    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert [item["path"] for item in report] == [str(problem), str(table)]
    assert all(item["diagnostics"] for item in report)
