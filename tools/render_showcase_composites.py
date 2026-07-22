"""Render overview composites for the slower-vehicle trajectory families."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Any

import yaml

from taoryx.language import GrammarProfile
from taoryx.outputs import RunArtifact, VehicleTelemetry
from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[1]
LADDER = ROOT / "verification/fidelity_ladder.yaml"
####


def _families() -> tuple[dict[str, Any], ...]:
    """Load the same family metadata used by the staged verification ladder."""

    payload = yaml.safe_load(LADDER.read_text(encoding="utf-8"))
    return tuple(payload["families"])
####


def _kinematic_problem(source: Path, destination: Path) -> Path:
    """Derive the bridge case exactly as the staged ladder tests do."""

    lines = source.read_text(encoding="utf-8").splitlines()
    title_index = next(index for index, line in enumerate(lines) if line.startswith("*title"))
    lines[title_index + 1:title_index + 1] = [
        "*mode kinematic-6dof",
        "*runtime status attitude mode=lag roll-deg=0 pitch-deg=0 yaw-deg=0 lag-s=0.25 max-rate-deg-s=360",
    ]
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return destination
####


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
####


def _scenario_contract(problem: Path, tables: tuple[Path, ...], tier: str, unit_system: str, duration_s: float | None) -> dict[str, Any]:
    return {
        "problem_sha256": _sha256(problem),
        "table_sha256": [_sha256(path) for path in tables],
        "dynamics_tier": tier,
        "unit_system": unit_system,
        "duration_s": duration_s,
    }
####


def _contract_comparison(left: dict[str, Any], right: dict[str, Any]) -> dict[str, Any]:
    fields = ("problem_sha256", "table_sha256", "unit_system", "duration_s")
    mismatches = [field for field in fields if left[field] != right[field]]
    return {"comparable": not mismatches, "mismatches": mismatches}
####


def _run_family(family: dict[str, Any], output_dir: Path) -> tuple[RunArtifact | None, RunArtifact | None, RunArtifact | None, dict[str, str], dict[str, Any]]:
    point_problem = ROOT / str(family.get("showcase_point_mass_problem", family["point_mass_problem"]))
    rigid_problem = ROOT / str(family.get("showcase_rigid_body_problem", family["rigid_body_problem"]))
    tables = tuple(ROOT / str(path) for path in family["tables"])
    rigid_tables = tuple(ROOT / str(path) for path in family["rigid_tables"])
    max_steps = int(family.get("showcase_max_steps", family["max_steps"]))
    baseline = run_files(point_problem, tables, output_dir=output_dir / family["id"] / "3dof", max_steps=max_steps, profile=GrammarProfile.TAORYX)
    bridge_problem = _kinematic_problem(point_problem, output_dir / family["id"] / "kinematic.prb")
    bridge = run_files(bridge_problem, tables, output_dir=output_dir / family["id"] / "bridge", max_steps=max_steps, profile=GrammarProfile.TAORYX)
    high_fidelity = run_files(rigid_problem, rigid_tables, output_dir=output_dir / family["id"] / "6dof", max_steps=max_steps, integrator="rk4", profile=GrammarProfile.TAORYX)
    statuses: dict[str, str] = {}
    artifacts: list[RunArtifact | None] = []
    for label, report in (("3dof", baseline), ("bridge", bridge), ("6dof", high_fidelity)):
        if report.exit_code != 0 or not report.artifacts:
            statuses[label] = "; ".join(f"{item.code}: {item.message}" for item in report.diagnostics) or "no artifact"
            artifacts.append(None)
        else:
            statuses[label] = "pass"
            artifacts.append(report.artifacts[0])
    unit_point = str(family["point_mass_unit_system"])
    unit_rigid = str(family["rigid_body_unit_system"])
    contracts = {
        "3dof": _scenario_contract(point_problem, tables, "point-mass-3dof", unit_point, _duration(artifacts[0])),
        "bridge": _scenario_contract(point_problem, tables, "kinematic-3-plus-3-dof", unit_point, _duration(artifacts[1])),
        "6dof": _scenario_contract(rigid_problem, rigid_tables, "rigid-body-6dof", unit_rigid, _duration(artifacts[2])),
    }
    contracts["comparisons"] = {
        "3dof_vs_bridge": _contract_comparison(contracts["3dof"], contracts["bridge"]),
        "3dof_vs_6dof": _contract_comparison(contracts["3dof"], contracts["6dof"]),
    }
    return artifacts[0], artifacts[1], artifacts[2], statuses, contracts
####


def _scale_for(source_name: str, unit_system: str) -> float:
    """Return the conversion from declared native output units to SI."""

    if unit_system == "si":
        return 1.0
    if unit_system != "fps":
        raise ValueError(f"unsupported showcase unit system: {unit_system}")
    if source_name in {"alt", "vel", "xdt", "ydt", "zdt"}:
        return 0.3048
    if source_name in {"mass", "wt"}:
        return 0.45359237
    if source_name == "dynprs":
        return 47.88025898
    if source_name.startswith("total_force_body_"):
        return 4.4482216152605
    if source_name.startswith("total_moment_body_"):
        return 1.3558179483314004
    return 1.0
####


def _series(vehicle: VehicleTelemetry, source_names: tuple[str, ...], unit_system: str) -> tuple[list[float], list[float]] | None:
    channel = next((item for item in vehicle.channels.values() if item.source_name in source_names), None)
    if channel is None:
        return None
    points = [(time, value) for time, value in zip(vehicle.times, channel.values, strict=True) if value is not None]
    if not points:
        return None
    scale = _scale_for(channel.source_name, unit_system)
    return [point[0] for point in points], [float(point[1]) * scale for point in points]
####


def _plot_family(name: str, baseline: RunArtifact | None, bridge: RunArtifact | None, high_fidelity: RunArtifact | None, baseline_units: str, bridge_units: str, high_fidelity_units: str, path: Path, plt: Any, contracts: dict[str, Any]) -> None:
    panels = (
        ("Altitude", ("alt", "altitude_m"), "m"),
        ("Speed", ("vel", "speed_m_s"), "m/s"),
        ("Mass", ("mass",), "kg"),
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
        for label, artifact, units, color, style in (("3-DOF", baseline, baseline_units, "#2563eb", "-"), ("3+3 bridge", bridge, bridge_units, "#d97706", "-."), ("6-DOF", high_fidelity, high_fidelity_units, "#dc2626", "--")):
            if artifact is None:
                continue
            vehicle = next(iter(artifact.vehicles.values()))
            values = _series(vehicle, source_names, units)
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
    mismatch = contracts["comparisons"]["3dof_vs_6dof"]["mismatches"]
    title = f"{name}: 3-DOF / kinematic 3+3 bridge / canonical 6-DOF overview"
    if mismatch:
        title += "\nNOT COMPARABLE: scenario-contract mismatch — " + ", ".join(mismatch)
    figure.suptitle(title, fontsize=16, fontweight="bold", color="#991b1b" if mismatch else "#111827")
    figure.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(figure)
####


def _plot_overview(records: list[dict[str, Any]], path: Path, plt: Any) -> None:
    figure, axes = plt.subplots(len(records), 3, figsize=(15, 4.2 * len(records)), squeeze=False, layout="constrained")
    panels = (("Altitude", ("alt", "altitude_m")), ("Speed", ("vel", "speed_m_s")), ("Mass", ("mass",)))
    for row, record in enumerate(records):
        for column, (title, source_names) in enumerate(panels):
            axis = axes[row][column]
            for label, artifact, units, color, style in (("3-DOF", record["baseline"], record["baseline_units"], "#2563eb", "-"), ("3+3 bridge", record["bridge"], record["bridge_units"], "#d97706", "-."), ("6-DOF", record["six_dof"], record["six_dof_units"], "#dc2626", "--")):
                if artifact is None:
                    continue
                vehicle = next(iter(artifact.vehicles.values()))
                values = _series(vehicle, source_names, units)
                if values is not None:
                    axis.plot(*values, label=label, color=color, linestyle=style, linewidth=1.8)
            axis.set_title(f"{record['name']} — {title}", loc="left", fontweight="bold")
            axis.set_xlabel("time (s)")
            axis.grid(True, color="#cbd5e1", linewidth=0.7)
            if column == 0:
                axis.set_ylabel({"Altitude": "m", "Speed": "m/s", "Mass": "kg"}[title])
            axis.legend(fontsize="small")
        mismatch = record["contracts"]["comparisons"]["3dof_vs_6dof"]["mismatches"]
        if mismatch:
            axes[row][0].text(
                0.01,
                0.98,
                "NOT COMPARABLE: " + ", ".join(mismatch),
                transform=axes[row][0].transAxes,
                va="top",
                fontsize="x-small",
                color="#991b1b",
                bbox={"facecolor": "#fef2f2", "edgecolor": "#fecaca", "alpha": 0.9},
            )
    figure.suptitle("TAORYX four-family dynamics-fidelity ladder overview", fontsize=17, fontweight="bold")
    figure.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(figure)
####


def _duration(artifact: RunArtifact | None) -> float | None:
    if artifact is None or not artifact.vehicles:
        return None
    vehicle = next(iter(artifact.vehicles.values()))
    return float(vehicle.times[-1]) if vehicle.times else None


def _comparison_warning(left: RunArtifact | None, right: RunArtifact | None) -> list[str]:
    """Return a conservative warning for the visual overlay.

    The full manifest remains the authoritative contract report.  This small
    helper keeps the figure honest even when it is opened without the JSON.
    """

    if left is None or right is None:
        return ["missing tier"]
    return ["scenario contract not established"]
####


def generate(output_dir: Path) -> Path:
    """Run nominal paired examples and write family/all-family composites."""
    output_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(output_dir / ".mplconfig"))
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    records: list[dict[str, Any]] = []
    for family in _families():
        baseline, bridge, high_fidelity, statuses, contracts = _run_family(family, output_dir / "runs")
        baseline_units = str(family["point_mass_unit_system"])
        bridge_units = baseline_units
        high_fidelity_units = str(family["rigid_body_unit_system"])
        _plot_family(family["display_name"], baseline, bridge, high_fidelity, baseline_units, bridge_units, high_fidelity_units, output_dir / f"{family['id'].lower()}_composite.png", plt, contracts)
        records.append({"id": family["id"], "name": family["display_name"], "baseline": baseline, "bridge": bridge, "six_dof": high_fidelity, "baseline_units": baseline_units, "bridge_units": bridge_units, "six_dof_units": high_fidelity_units, "durations_s": {"3dof": _duration(baseline), "bridge": _duration(bridge), "6dof": _duration(high_fidelity)}, "statuses": statuses, "contracts": contracts})
    _plot_overview(records, output_dir / "all_families_overview.png", plt)
    manifest = {
        "families": [{"id": record["id"], "name": record["name"], "composite": f"{record['id'].lower()}_composite.png", "statuses": record["statuses"], "unit_systems": {"3dof": record["baseline_units"], "bridge": record["bridge_units"], "6dof": record["six_dof_units"]}, "durations_s": record["durations_s"], "scenario_contract": record["contracts"]} for record in records],
        "overview": "all_families_overview.png",
        "tiers": ["point-mass-3dof", "kinematic-3-plus-3-dof", "rigid-body-6dof"],
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
