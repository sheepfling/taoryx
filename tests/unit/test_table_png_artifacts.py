from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.language.table_parser import parse_table_file
from taoryx.runtime.lowering import lower_tables
from taoryx.visualization import render_table_png

ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "table_examples_v1"


@pytest.mark.artifact
def test_table_example_workspace_renders_png_artifacts(artifact_dir: Path) -> None:
    output_root = artifact_dir / "table-plots"
    generated: list[Path] = []

    for table_path in sorted(ROOT.rglob("*.tbl")):
        tables = lower_tables(parse_table_file(table_path))
        for table_name, table in sorted(tables.items()):
            assert table.prepared is not None, table_path
            output_path = output_root / table_path.relative_to(ROOT).with_suffix(".png")
            output_path.parent.mkdir(parents=True, exist_ok=True)
            output_path.write_bytes(
                render_table_png(
                    table.prepared,
                    axis_labels=table.independent_variables,
                    title=table_name,
                )
            )
            generated.append(output_path)

    index = output_root / "manifest.txt"
    index.write_text("\n".join(str(path.relative_to(output_root)) for path in sorted(generated)), encoding="utf-8")

    assert generated
    assert all(path.exists() for path in generated)
    assert index.exists()
    ####
