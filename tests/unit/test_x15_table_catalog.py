from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from taoryx.table_explorer import TableInspectionStatus, inspect_table_file
from taoryx.visualization import render_table_png

ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "x15_coherent_6dof_public_research_v1"
TABLE_ROOT = ROOT / "tables"

pytestmark = pytest.mark.table


def test_x15_catalog_lists_every_generated_table() -> None:
    catalog = yaml.safe_load((TABLE_ROOT / "catalog.yaml").read_text(encoding="utf-8"))
    listed = sorted(entry["path"] for entry in catalog["tables"])
    actual = sorted(path.name for path in TABLE_ROOT.glob("*.tbl"))

    assert listed == actual
    assert catalog["provenance"]["source_status"] == "beta"
    assert catalog["kind"] == "vehicle-specific-public-6dof-research-surrogate"
####


@pytest.mark.parametrize("table_path", sorted(TABLE_ROOT.glob("*.tbl")), ids=lambda path: path.stem)
def test_x15_tables_are_prepared_and_error_free(table_path: Path) -> None:
    inspection = inspect_table_file(table_path)

    assert inspection.valid, inspection.format_catalog()
    assert inspection.executable_complete, inspection.format_catalog()
    assert inspection.tables, table_path
    assert all(table.status is TableInspectionStatus.PREPARED for table in inspection.tables), inspection.format_catalog()
    assert all(table.prepared is not None for table in inspection.tables), inspection.format_catalog()
####


@pytest.mark.artifact
def test_x15_tables_render_catalog_plots(artifact_dir: Path) -> None:
    output = artifact_dir / "x15-table-catalog"
    generated: list[Path] = []

    for table_path in sorted(TABLE_ROOT.glob("*.tbl")):
        inspection = inspect_table_file(table_path)
        for table in inspection.tables:
            if table.prepared is None:
                raise AssertionError(f"table was not prepared: {table_path}:{table.name}")
            fixed_axes = {name: value for name, value in (("mach", 4.0), ("altitude_m", 12192.0)) if name in table.independent_variables}
            plot_path = output / f"{table_path.stem}__{table.name}.png"
            plot_path.parent.mkdir(parents=True, exist_ok=True)
            plot_path.write_bytes(
                render_table_png(
                    table.prepared,
                    axis_labels=table.independent_variables,
                    fixed_axes=fixed_axes,
                    value_label=table.name,
                    title=f"X-15 {table.name} — {table_path.stem}",
                )
            )
            generated.append(plot_path)
        ####
    ####

    assert len(generated) == 26
    assert all(path.stat().st_size > 1000 for path in generated)
    (output / "manifest.txt").write_text("\n".join(path.name for path in generated), encoding="utf-8")
####
