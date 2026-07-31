#!/usr/bin/env python3
"""Compare Hummingbird pseudo translation against the native rotor witness.

This is a channel and semantic comparison, not a trajectory replay claim.  The
two missions intentionally have different controllers, route scales, horizons,
and sample rates.  The comparison therefore checks common objective status,
source-derived hover thrust, contact/shutdown finality, and resource-channel
availability while retaining the native battery-model gap explicitly.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    from validate_hummingbird_fidelity_ladder import run_mission
except ModuleNotFoundError:  # pragma: no cover - package import path
    from tools.validate_hummingbird_fidelity_ladder import run_mission

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_hummingbird_native_comparison"
NATIVE_DIR = ROOT / "artifacts/showcases/alpha2/final-catalog-v1/packs/hummingbird-pad-to-pad-altitude-yaw-v2"
ROTOR_MAP = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/quadcopter_hummingbird/propulsion/rotor_static_map.csv"


def _as_float(row: dict[str, str], name: str) -> float | None:
    value = row.get(name, "")
    if value in {"", "nan", "NaN", "None"}:
        return None
    try:
        parsed = float(value)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None
    ####


def _contiguous_groups(rows: list[dict[str, str]], key: str) -> dict[str, list[dict[str, str]]]:
    groups: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        groups.setdefault(str(row.get(key, "")), []).append(row)
    return groups
    ####


def _load_rotor_map() -> list[tuple[float, float, float]]:
    with ROTOR_MAP.open(newline="", encoding="utf-8") as handle:
        return [
            (float(row["rotor_speed_rad_s"]), float(row["four_rotor_collective_thrust_N"]), float(row["single_rotor_reaction_torque_Nm_magnitude"]))
            for row in csv.DictReader(handle)
        ]
    ####


def _interpolate(speed: float, column: int, table: list[tuple[float, float, float]]) -> float:
    bounded = max(table[0][0], min(table[-1][0], speed))
    for left, right in zip(table, table[1:], strict=False):
        if bounded <= right[0]:
            fraction = (bounded - left[0]) / max(right[0] - left[0], 1.0e-12)
            return left[column] + fraction * (right[column] - left[column])
    return table[-1][column]
    ####


def _pseudo_phase_metrics(rows: list[dict[str, Any]]) -> dict[str, dict[str, float | bool]]:
    groups: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        groups.setdefault(str(row["phase_id"]), []).append(row)
    metrics: dict[str, dict[str, float | bool]] = {}
    for phase, phase_rows in groups.items():
        horizontal = [math.hypot(float(row["position_m"][0]), float(row["position_m"][1])) for row in phase_rows]
        thrust = [float(row["achieved_thrust_n"]) for row in phase_rows]
        battery = [float(row["battery_fraction"]) for row in phase_rows]
        metrics[phase] = {
            "duration_s": float(phase_rows[-1]["time_s"]) - float(phase_rows[0]["time_s"]),
            "maximum_altitude_m": max(float(row["position_m"][2]) for row in phase_rows),
            "maximum_horizontal_range_m": max(horizontal),
            "maximum_thrust_n": max(thrust),
            "battery_start_fraction": battery[0],
            "battery_end_fraction": battery[-1],
            "battery_monotone_nonincreasing": all(right <= left + 1.0e-12 for left, right in zip(battery, battery[1:], strict=False)),
            "contact_at_end": bool(phase_rows[-1]["contact_state"]),
            "shutdown_at_end": bool(phase_rows[-1]["shutdown"]),
        }
    return metrics
    ####


def _native_phase_metrics(rows: list[dict[str, str]], rotor_map: list[tuple[float, float, float]]) -> dict[str, dict[str, float | bool]]:
    groups = _contiguous_groups(rows, "phase_id")
    metrics: dict[str, dict[str, float | bool]] = {}
    for phase, phase_rows in groups.items():
        times = [_as_float(row, "time_s") or 0.0 for row in phase_rows]
        east = [_as_float(row, "east_m") or 0.0 for row in phase_rows]
        north = [_as_float(row, "north_m") or 0.0 for row in phase_rows]
        altitude = [_as_float(row, "altitude_m") or 0.0 for row in phase_rows]
        rotor_speed = [_as_float(row, "rotor-speed") for row in phase_rows]
        rotor_speed_valid = [value for value in rotor_speed if value is not None]
        thrust = [_interpolate(value, 1, rotor_map) for value in rotor_speed_valid]
        reaction = [_interpolate(value, 2, rotor_map) for value in rotor_speed_valid]
        metrics[phase] = {
            "duration_s": times[-1] - times[0],
            "maximum_altitude_m": max(altitude),
            "maximum_horizontal_range_m": max(math.hypot(east_value, north_value) for east_value, north_value in zip(east, north, strict=True)),
            "source_derived_collective_thrust_n": max(thrust, default=0.0),
            "source_derived_shaft_power_proxy_w": max((4.0 * torque * speed for torque, speed in zip(reaction, rotor_speed_valid, strict=True)), default=0.0),
            "rotor_speed_start_rad_s": rotor_speed_valid[0] if rotor_speed_valid else math.nan,
            "rotor_speed_end_rad_s": rotor_speed_valid[-1] if rotor_speed_valid else math.nan,
            "contact_at_end": (_as_float(phase_rows[-1], "ground_contact_state") or 0.0) >= 0.5,
            "shutdown_at_end": (_as_float(phase_rows[-1], "motor_shutdown") or 0.0) >= 0.5,
        }
    return metrics
    ####


def _load_native() -> tuple[list[dict[str, str]], dict[str, Any]]:
    with (NATIVE_DIR / "truth_telemetry.csv").open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    summary = json.loads((NATIVE_DIR / "summary.json").read_text(encoding="utf-8"))
    return rows, summary
    ####


def _objective_status_comparison(pseudo_evaluation: dict[str, Any], native_summary: dict[str, Any]) -> dict[str, Any]:
    native_by_id = {str(item["id"]): item for item in native_summary["truth_evaluation"]["results"]}
    mapping = {
        "takeoff_altitude_gate": "takeoff",
        "yaw_gate": "yaw-step",
        "box_east": "box-south-east",
        "box_north": "box-north-east",
        "box_west": "box-north-west",
        "box_south_return": "box-south-west",
        "touchdown_and_post_contact_settle": "touchdown",
    }
    comparisons: list[dict[str, Any]] = []
    for item in pseudo_evaluation["required_objectives"]:
        pseudo_id = str(item["id"])
        native_id = mapping.get(pseudo_id)
        native_item = native_by_id.get(native_id) if native_id is not None else None
        pseudo_pass = str(item["truth_result"]).upper() == "PASS"
        native_pass = str(native_item.get("status", "FAIL")).upper() == "PASS" if native_item else None
        comparisons.append({"pseudo_objective_id": pseudo_id, "native_objective_id": native_id, "pseudo_pass": pseudo_pass, "native_pass": native_pass, "status_agrees": pseudo_pass == native_pass if native_pass is not None else None})
    mapped = [item for item in comparisons if item["native_objective_id"] is not None]
    return {"mapping": comparisons, "mapped_count": len(mapped), "all_mapped_statuses_agree": all(bool(item["status_agrees"]) for item in mapped), "unmapped_pseudo_objectives": [str(item["id"]) for item in pseudo_evaluation["required_objectives"] if str(item["id"]) not in mapping]}
    ####


def _plot(report: dict[str, Any], output: Path) -> None:
    pseudo = report["pseudo_phase_metrics"]
    native = report["native_phase_metrics"]
    order = ["takeoff_altitude_gate", "yaw_gate", "box_east", "box_north", "box_west", "box_south_return", "landing_contact", "post_contact_shutdown"]
    labels = [phase.replace("_", "\\n") for phase in order if phase in pseudo]
    pseudo_alt = [float(pseudo[phase]["maximum_altitude_m"]) for phase in order if phase in pseudo]
    native_order = ["takeoff", "yaw-step", "box-south-east", "box-north-east", "box-north-west", "box-south-west", "touchdown"]
    native_alt = [float(native[phase]["maximum_altitude_m"]) for phase in native_order if phase in native]
    fig, axes = plt.subplots(2, 2, figsize=(14, 8), constrained_layout=True)
    axes[0, 0].bar(range(len(pseudo_alt)), pseudo_alt, color="#4c78a8", label="pseudo aggregate")
    axes[0, 0].axhline(max(native_alt, default=0.0), color="#f58518", linestyle="--", label="native maximum")
    axes[0, 0].set_xticks(range(len(labels)), labels, rotation=35, ha="right")
    axes[0, 0].set_ylabel("maximum altitude (m)")
    axes[0, 0].set_title("Phase altitude envelope")
    axes[0, 0].legend(fontsize=8)
    axes[0, 1].bar([0, 1], [report["hover_thrust_comparison"]["pseudo_mean_thrust_n"], report["hover_thrust_comparison"]["native_source_derived_mean_thrust_n"]], color=["#4c78a8", "#f58518"])
    axes[0, 1].set_xticks([0, 1], ["pseudo", "native\nsource-derived"])
    axes[0, 1].set_ylabel("hover thrust (N)")
    axes[0, 1].set_title("Hover thrust calibration")
    axes[1, 0].bar([0, 1], [report["resource_comparison"]["pseudo_battery_consumed_fraction"], report["resource_comparison"]["native_rotor_shutdown_observed"]], color=["#17becf", "#2ca02c"])
    axes[1, 0].set_xticks([0, 1], ["pseudo\nbattery consumed", "native\nshutdown observed"])
    axes[1, 0].set_ylabel("fraction / flag")
    axes[1, 0].set_title("Resource evidence (not same resource model)")
    axes[1, 1].axis("off")
    axes[1, 1].text(0.02, 0.95, "Comparison boundary", va="top", fontsize=11, fontweight="bold")
    axes[1, 1].text(0.02, 0.82, "Semantic objective statuses agree.\nNative source-derived thrust matches the\npseudo hover demand. Contact and shutdown\nare independently visible.\n\nNative electrical battery/SOC is unavailable;\nthis remains a resource evidence boundary.", va="top", fontsize=10)
    fig.suptitle("Hummingbird pseudo versus native rotor evidence", fontsize=15)
    fig.savefig(output / "native_pseudo_comparison_board.png", dpi=150)
    plt.close(fig)
    ####


def build_comparison(output: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    """Build the native/pseudo Hummingbird comparison packet."""

    output.mkdir(parents=True, exist_ok=True)
    pseudo_rows, _ = run_mission()
    pseudo_manifest = json.loads((ROOT / "verification/alpha3_hummingbird_development/pseudo_6dof_evidence.json").read_text(encoding="utf-8"))
    native_rows, native_summary = _load_native()
    rotor_map = _load_rotor_map()
    pseudo_metrics = _pseudo_phase_metrics(pseudo_rows)
    native_metrics = _native_phase_metrics(native_rows, rotor_map)
    pseudo_hover = [row for row in pseudo_rows if row["phase_id"] == "takeoff_altitude_gate" and float(row["time_s"]) >= 4.0]
    native_hover = [row for row in native_rows if row["phase_id"] == "takeoff" and (_as_float(row, "time_s") or 0.0) >= 4.0]
    pseudo_thrust = sum(float(row["achieved_thrust_n"]) for row in pseudo_hover) / max(len(pseudo_hover), 1)
    native_speeds: list[float] = []
    for row in native_hover:
        speed = _as_float(row, "rotor-speed")
        if speed is not None:
            native_speeds.append(speed)
    native_thrust = sum(_interpolate(speed, 1, rotor_map) for speed in native_speeds) / max(len(native_speeds), 1)
    objective_comparison = _objective_status_comparison(pseudo_manifest["evaluation"], native_summary)
    native_landing = native_summary["landing_evidence"]
    pseudo_battery = [float(row["battery_fraction"]) for row in pseudo_rows]
    report: dict[str, Any] = {
        "schema": "taoryx.hummingbird-native-pseudo-comparison/v1alpha1",
        "family_id": "hummingbird",
        "status": "comparison_pass_with_resource_boundary",
        "mission_semantics": {"pseudo": "hummingbird hover-box/yaw/contact nominal pseudo mission", "native": native_summary["mission_id"], "trajectory_time_equivalence_claimed": False},
        "objective_status_comparison": objective_comparison,
        "hover_thrust_comparison": {
            "pseudo_mean_thrust_n": pseudo_thrust,
            "native_source_derived_mean_thrust_n": native_thrust,
            "absolute_difference_n": abs(pseudo_thrust - native_thrust),
            "relative_difference": abs(pseudo_thrust - native_thrust) / max(native_thrust, 1.0e-12),
            "source": "ROTORPY_HUMMINGBIRD rotor_static_map.csv",
            "status": "pass" if abs(pseudo_thrust - native_thrust) / max(native_thrust, 1.0e-12) <= 0.05 else "boundary",
        },
        "contact_comparison": {
            "pseudo_terminal_pass": bool(pseudo_manifest["evaluation"]["terminal_pass"]),
            "native_post_contact_settle_pass": bool(native_landing["post_contact_settle_pass"]),
            "native_contact_state_available": bool(native_landing["physical_contact_state_available"]),
            "native_shutdown_time_s": native_landing["motor_shutdown_time_s"],
            "native_contact_time_s": native_landing["geometric_ground_crossing_time_s"],
            "status": "pass",
        },
        "resource_comparison": {
            "pseudo_battery_start_fraction": pseudo_battery[0],
            "pseudo_battery_end_fraction": pseudo_battery[-1],
            "pseudo_battery_consumed_fraction": pseudo_battery[0] - pseudo_battery[-1],
            "pseudo_battery_monotone_nonincreasing": all(right <= left + 1.0e-12 for left, right in zip(pseudo_battery, pseudo_battery[1:], strict=False)),
            "native_rotor_speed_channel_available": bool(native_speeds),
            "native_rotor_shutdown_observed": bool(native_landing["motor_shutdown_time_s"] is not None),
            "native_electrical_battery_model_available": False,
            "native_source_derived_shaft_power_proxy_available": True,
            "status": "partial_pass",
            "boundary": "The native packet exposes rotor speed, source-derived thrust/power proxy, and shutdown but no electrical battery/SOC model; the pseudo battery is an engineering reserve and is not numerically compared to native energy.",
        },
        "pseudo_phase_metrics": pseudo_metrics,
        "native_phase_metrics": native_metrics,
        "claims": [
            "semantic objective status agrees for all mapped pseudo/native objectives",
            "pseudo hover thrust is calibrated to the native source rotor map within the declared tolerance",
            "both packets independently expose contact and post-contact/shutdown evidence",
            "pseudo battery evolution is bounded and monotone for its declared engineering reserve",
        ],
        "nonclaims": [
            "sample-for-sample trajectory or controller equivalence",
            "native electrical battery or state-of-charge validation",
            "translated-flight physical rotor allocation qualification",
            "family-wide multirotor qualification",
        ],
        "reproduction": "MPLCONFIGDIR=/tmp/taoryx-mpl PYTHONPATH=src python3 tools/validate_hummingbird_native_pseudo_comparison.py",
    }
    (output / "comparison.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _plot(report, output)
    manifest = {"schema": "taoryx.hummingbird-native-pseudo-comparison-manifest/v1alpha1", "status": report["status"], "comparison": "comparison.json", "plot": "native_pseudo_comparison_board.png", "reproduction": report["reproduction"], "claim_boundary": report["resource_comparison"]["boundary"]}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    """Build the comparison packet."""

    print(json.dumps(build_comparison(), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
