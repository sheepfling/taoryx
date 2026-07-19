"""Render overview composites for the slower-vehicle trajectory families."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Any

from taoryx.language import GrammarProfile
from taoryx.outputs import RunArtifact, VehicleTelemetry
from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[1]
FIXTURE_TABLES = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables"
FAMILY_ROOT = ROOT / "examples/mission_families"
####


FAMILIES: tuple[dict[str, Any], ...] = (
    {
        "id": "SV01",
        "name": "B747",
        "directory": "slower_b747",
        "problem_3dof": "SV01_3dof.prb",
        "problem_6dof": "SV01_6dof.prb",
        "tables": ("b747_nominal_static_6axis.tbl",),
    },
    {
        "id": "SV03",
        "name": "Skywalker X8",
        "directory": "slower_x8",
        "problem_3dof": "SV03_3dof.prb",
        "problem_6dof": "SV03_6dof.prb",
        "tables": ("skywalker_x8_static_6axis.tbl",),
    },
    {
        "id": "SV05",
        "name": "AscTec Hummingbird",
        "directory": "slower_hummingbird",
        "problem_3dof": "SV05_3dof.prb",
        "problem_6dof": "SV05_6dof.prb",
        "tables": ("hummingbird_cx.tbl", "hummingbird_cy.tbl", "hummingbird_cz.tbl", "hummingbird_cmx.tbl", "hummingbird_cmy.tbl", "hummingbird_cmz.tbl"),
    },
)
####


def _run_family(family: dict[str, Any], output_dir: Path) -> tuple[RunArtifact | None, RunArtifact | None, dict[str, str]]:
    directory = FAMILY_ROOT / family["directory"]
    tables = tuple(FIXTURE_TABLES / name for name in family["tables"])
    baseline = run_files(directory / family["problem_3dof"], output_dir=output_dir / family["id"] / "3dof", max_steps=200, profile=GrammarProfile.TAORYX)
    high_fidelity = run_files(directory / family["problem_6dof"], tables, output_dir=output_dir / family["id"] / "6dof", max_steps=200, integrator="euler", profile=GrammarProfile.TAORYX)
    statuses: dict[str, str] = {}
    artifacts: list[RunArtifact | None] = []
    for label, report in (("3dof", baseline), ("6dof", high_fidelity)):
        if report.exit_code != 0 or not report.artifacts:
            statuses[label] = "; ".join(f"{item.code}: {item.message}" for item in report.diagnostics) or "no artifact"
            artifacts.append(None)
        else:
            statuses[label] = "pass"
            artifacts.append(report.artifacts[0])
    return artifacts[0], artifacts[1], statuses
####


def _series(vehicle: VehicleTelemetry, source_names: tuple[str, ...]) -> tuple[list[float], list[float]] | None:
    channel = next((item for item in vehicle.channels.values() if item.source_name in source_names), None)
    if channel is None:
        return None
    points = [(time, value) for time, value in zip(vehicle.times, channel.values, strict=True) if value is not None]
    if not points:
        return None
    return [point[0] for point in points], [float(point[1]) for point in points]
####


def _plot_family(name: str, baseline: RunArtifact, high_fidelity: RunArtifact, path: Path, plt: Any) -> None:
    panels = (
        ("Altitude", ("alt", "altitude_m"), "m"),
        ("Speed", ("vel", "speed_m_s"), "m/s"),
        ("Mass", ("mass",), "source units"),
        ("Dynamic pressure", ("dynprs", "aero_dynamic_pressure_pa"), "Pa"),
        ("Total body forces", ("total_force_body_x_n", "total_force_body_y_n", "total_force_body_z_n"), "N"),
        ("Total body moments", ("total_moment_body_x_nm", "total_moment_body_y_nm", "total_moment_body_z_nm"), "N m"),
        ("Attitude", ("roll_deg", "pitch_deg", "yaw_deg"), "deg"),
        ("Thermal proxy — not certified", ("heat_rate_w_m2", "peak_heat_rate"), "W/m² or source"),
        ("Mach number", ("aero_mach",), "dimensionless"),
    )
    figure, axes = plt.subplots(3, 3, figsize=(15, 11), layout="constrained")
    for axis, (title, source_names, unit) in zip(axes.flat, panels, strict=True):
        plotted = False
        for label, artifact, color, style in (("3-DOF baseline", baseline, "#2563eb", "-"), ("6-DOF", high_fidelity, "#dc2626", "--")):
            if artifact is None:
                continue
            vehicle = next(iter(artifact.vehicles.values()))
            values = _series(vehicle, source_names)
            if values is None:
                continue
            times, samples = values
            axis.plot(times, samples, label=label, color=color, linestyle=style, linewidth=1.8)
            plotted = True
        axis.set_title(title, loc="left", fontweight="bold")
        axis.set_xlabel("time (s)")
        axis.set_ylabel(unit)
        axis.grid(True, color="#cbd5e1", linewidth=0.7)
        if plotted:
            axis.legend(fontsize="small")
        else:
            axis.text(0.5, 0.5, "channel unavailable", ha="center", va="center", transform=axis.transAxes)
    figure.suptitle(f"{name}: 3-DOF baseline / native 6-DOF evaluation overview", fontsize=16, fontweight="bold")
    figure.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(figure)
####


def _plot_overview(records: list[dict[str, Any]], path: Path, plt: Any) -> None:
    figure, axes = plt.subplots(len(records), 3, figsize=(15, 4.2 * len(records)), squeeze=False, layout="constrained")
    panels = (("Altitude", ("alt", "altitude_m")), ("Speed", ("vel", "speed_m_s")), ("Mass", ("mass",)))
    for row, record in enumerate(records):
        for column, (title, source_names) in enumerate(panels):
            axis = axes[row][column]
            for label, artifact, color, style in (("3-DOF baseline", record["baseline"], "#2563eb", "-"), ("6-DOF", record["six_dof"], "#dc2626", "--")):
                if artifact is None:
                    continue
                vehicle = next(iter(artifact.vehicles.values()))
                values = _series(vehicle, source_names)
                if values is not None:
                    axis.plot(*values, label=label, color=color, linestyle=style, linewidth=1.8)
            axis.set_title(f"{record['name']} — {title}", loc="left", fontweight="bold")
            axis.set_xlabel("time (s)")
            axis.grid(True, color="#cbd5e1", linewidth=0.7)
            if column == 0:
                axis.set_ylabel("value")
            axis.legend(fontsize="small")
    figure.suptitle("TAORYX slower-vehicle trajectory overview", fontsize=17, fontweight="bold")
    figure.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(figure)
####


def generate(output_dir: Path) -> Path:
    """Run nominal paired examples and write family/all-family composites."""
    output_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(output_dir / ".mplconfig"))
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    records: list[dict[str, Any]] = []
    for family in FAMILIES:
        baseline, high_fidelity, statuses = _run_family(family, output_dir / "runs")
        _plot_family(family["name"], baseline, high_fidelity, output_dir / f"{family['id'].lower()}_composite.png", plt)
        records.append({"id": family["id"], "name": family["name"], "baseline": baseline, "six_dof": high_fidelity, "statuses": statuses})
    _plot_overview(records, output_dir / "all_families_overview.png", plt)
    manifest = {
        "families": [{"id": record["id"], "name": record["name"], "composite": f"{record['id'].lower()}_composite.png", "statuses": record["statuses"]} for record in records],
        "overview": "all_families_overview.png",
        "claim_boundary": {"engineering_validity": False, "historical_taos_96_compatibility": False, "source_data": "public_research_surrogates"},
    }
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest_path
####


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "artifacts/showcases/slower_vehicle_composites")
    arguments = parser.parse_args()
    print(generate(arguments.output_dir))
    ####


if __name__ == "__main__":
    main()
