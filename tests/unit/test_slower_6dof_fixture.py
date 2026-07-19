from __future__ import annotations

import csv
import json
from pathlib import Path

import pytest
import yaml

from taoryx.table_explorer import TableInspectionStatus, inspect_table_file
from taoryx.visualization import render_table_png

ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "slower_airbreathing_and_multirotor_6dof_bundle_v1"
TABLE_ROOT = ROOT / "tables"

pytestmark = pytest.mark.table


def test_slower_bundle_source_validation_and_catalog_are_present() -> None:
    report = json.loads((ROOT / "checks/validation_report.json").read_text(encoding="utf-8"))
    catalog = yaml.safe_load((TABLE_ROOT / "catalog.yaml").read_text(encoding="utf-8"))

    assert report["status"] == "PASS"
    assert report["failures"] == []
    assert catalog["provenance"]["source_bundle_sha256"] == "17ee0ca897e3ee44d4dd68deb6eb6c4922fef9ef87de46fb032124596b39b911"
    assert catalog["provenance"]["source_status"] == "mixed-public-research"
    assert {entry["vehicle"] for entry in catalog["tables"] if "vehicle" in entry} == {"b747", "skywalker_x8", "hummingbird"}
    ####


def test_slower_bundle_source_row_counts_are_stable() -> None:
    expected = {
        "jet_b747/aero/static_six_axis_grid.csv": 650,
        "jet_b747/propulsion/jt9d_installed_thrust_map.csv": 150,
        "cruise_class_uav_skywalker_x8/aero/static_airframe_grid.csv": 840,
        "cruise_class_uav_skywalker_x8/aero/rate_effects_grid.csv": 360,
        "cruise_class_uav_skywalker_x8/propulsion/simplified_thrust_map.csv": 252,
        "quadcopter_hummingbird/aero/common_speed_wrench_grid.csv": 270,
        "quadcopter_hummingbird/propulsion/rotor_static_map.csv": 61,
    }
    for relative, row_count in expected.items():
        with (ROOT / relative).open(encoding="utf-8", newline="") as handle:
            assert sum(1 for _ in csv.DictReader(handle)) == row_count
        ####
    ####


def test_slower_generated_tables_are_prepared_and_error_free() -> None:
    catalog = yaml.safe_load((TABLE_ROOT / "catalog.yaml").read_text(encoding="utf-8"))
    listed = sorted(entry["path"] for entry in catalog["tables"] if entry.get("status") != "source-csv-only")
    actual = sorted(path.name for path in TABLE_ROOT.glob("*.tbl"))
    assert listed == actual
    for table_path in sorted(TABLE_ROOT.glob("*.tbl")):
        inspection = inspect_table_file(table_path)
        assert inspection.valid, inspection.format_catalog()
        assert inspection.executable_complete, inspection.format_catalog()
        assert all(table.status is TableInspectionStatus.PREPARED for table in inspection.tables), inspection.format_catalog()
        assert all(table.prepared is not None for table in inspection.tables), inspection.format_catalog()
    ####


@pytest.mark.artifact
def test_slower_generated_tables_render_catalog_plots(artifact_dir: Path) -> None:
    output = artifact_dir / "slower-vehicle-table-catalog"
    generated: list[Path] = []
    for table_path in sorted(TABLE_ROOT.glob("*.tbl")):
        inspection = inspect_table_file(table_path)
        for table in inspection.tables:
            assert table.prepared is not None
            fixed_axes = {
                axis: values[len(values) // 2]
                for axis, values in zip(table.independent_variables, table.prepared.axes, strict=True)
                if axis not in table.independent_variables[:2]
            }
            plot_path = output / f"{table_path.stem}__{table.name}.png"
            plot_path.parent.mkdir(parents=True, exist_ok=True)
            plot_path.write_bytes(
                render_table_png(
                    table.prepared,
                    axis_labels=table.independent_variables,
                    fixed_axes=fixed_axes,
                    value_label=table.name,
                    title=f"Slower vehicle {table.name} — {table_path.stem}",
                )
            )
            generated.append(plot_path)
        ####
    ####
    assert len(generated) == 60
    assert all(path.stat().st_size > 1000 for path in generated)
    (output / "manifest.txt").write_text("\n".join(path.name for path in generated), encoding="utf-8")
    ####
