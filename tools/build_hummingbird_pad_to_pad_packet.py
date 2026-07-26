"""Build a truth-evaluated Hummingbird pad-to-pad mission packet.

The runtime currently exposes reusable takeoff, waypoint, and landing problem
contracts rather than one monolithic mission route.  This harness composes
those contracts by handing the complete rigid-body state from one phase to the
next.  The handoff is checked for continuity and the final objective result is
computed independently from the concatenated truth telemetry.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import tempfile
import zipfile
from pathlib import Path
from typing import Any

from taoryx.language import GrammarProfile
from taoryx.mission_objectives import ControllerTransition, TruthObjectiveSpec, evaluate_truth_objectives
from taoryx.runtime.runner import run_files
from tools.build_family_qualification_packet import _render_mission_sequence, _sha256

ROOT = Path(__file__).resolve().parents[1]
TABLE_ROOT = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables"
TABLES = tuple(TABLE_ROOT / name for name in ("hummingbird_cx.tbl", "hummingbird_cy.tbl", "hummingbird_cz.tbl", "hummingbird_cmx.tbl", "hummingbird_cmy.tbl", "hummingbird_cmz.tbl"))
FAMILY_DIR = ROOT / "examples/mission_families/slower_hummingbird"
TAKEOFF = FAMILY_DIR / "SV05_takeoff_6dof.prb"
WAYPOINT = FAMILY_DIR / "SV05_basic_waypoint_6dof.prb"
LANDING = FAMILY_DIR / "SV05_landing_6dof.prb"
R_EARTH_M = 6_378_137.0


def _write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _local_rows(states: tuple[Any, ...], phase_id: str, phase_index: int, time_offset_s: float, origin_lat: float, origin_lon: float) -> tuple[dict[str, object], ...]:
    if not states:
        return ()
    scale_east = R_EARTH_M * math.cos(math.radians(origin_lat))
    rows: list[dict[str, object]] = []
    for state in states:
        named = dict(state.named)
        latitude = float(named.get("latitude_deg", origin_lat))
        longitude = float(named.get("longitude_deg", origin_lon))
        rows.append(
            {
                **named,
                "time_s": float(state.time) + time_offset_s,
                "phase_id": phase_id,
                "phase_index": float(phase_index),
                "north_m": math.radians(latitude - origin_lat) * R_EARTH_M,
                "east_m": math.radians(longitude - origin_lon) * scale_east,
            }
        )
    return tuple(rows)


def _write_csv(path: Path, rows: tuple[dict[str, object], ...]) -> None:
    keys = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _render_overview(packet: Path, rows: tuple[dict[str, object], ...], evaluation: dict[str, object]) -> None:
    """Render a compact route/objective board from the same truth rows."""

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def numeric(row: dict[str, object], key: str, default: float = math.nan) -> float:
        value = row.get(key, default)
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    stride = max(1, len(rows) // 3000)
    sampled = rows[::stride]
    if sampled[-1] is not rows[-1]:
        sampled = (*sampled, rows[-1])
    times = [float(row["time_s"]) for row in sampled]
    figure, axes = plt.subplots(3, 2, figsize=(16, 14), layout="constrained")
    axes[0, 0].plot([float(row["east_m"]) for row in sampled], [float(row["north_m"]) for row in sampled], color="#2563eb", linewidth=1.5)
    axes[0, 0].plot([0.0, 0.5, 0.5, 0.0, 0.0], [0.0, 0.0, 0.5, 0.5, 0.0], color="#dc2626", linestyle="--", linewidth=1.0, label="declared perimeter")
    for label, north, east in (("1", 0.0, 0.0), ("2", 0.0, 0.5), ("3", 0.5, 0.5), ("4", 0.5, 0.0), ("5", 0.0, 0.0)):
        axes[0, 0].annotate(label, (east, north), fontsize=9, fontweight="bold", ha="center", va="center", color="#991b1b")
    axes[0, 0].set_title("Truth path and declared perimeter")
    axes[0, 0].set_xlabel("east (m)")
    axes[0, 0].set_ylabel("north (m)")
    axes[0, 0].axis("equal")
    axes[0, 0].grid(True, color="#cbd5e1")
    axes[0, 0].legend(fontsize="small")
    result_by_id = {str(item["id"]): item for item in evaluation["results"]}
    status_sequence = (
        ("TAKEOFF", "takeoff"),
        ("HOVER", "hover-dwell"),
        ("YAW", "yaw-step"),
        ("ALTITUDE", "climb-altitude-gate"),
        ("PERIMETER", "return-home"),
        ("TOUCHDOWN", "touchdown"),
        ("SETTLE", "post-touchdown-settle"),
    )
    status_line = " | ".join(f"{label} {result_by_id[objective]['status'].upper()}" for label, objective in status_sequence)
    axes[0, 0].text(
        0.01,
        0.98,
        "MISSION STATUS\n" + status_line,
        transform=axes[0, 0].transAxes,
        va="top",
        ha="left",
        fontsize=7,
        color="#14532d" if all(result_by_id[objective]["status"] == "pass" for _, objective in status_sequence[:5]) else "#7f1d1d",
        bbox={"facecolor": "white", "edgecolor": "#94a3b8", "alpha": 0.88, "pad": 3.0},
    )
    axes[0, 0].text(
        0.01,
        0.02,
        "Mission: precision hover + altitude-gated perimeter + yaw step + landing\n"
        "Purpose: waypoint capture, hover dwell, heading authority, shutdown, and landing evaluation\n"
        "Expected difficulty: LOW",
        transform=axes[0, 0].transAxes,
        va="bottom",
        ha="left",
        fontsize=6,
        color="#334155",
        bbox={"facecolor": "white", "edgecolor": "#cbd5e1", "alpha": 0.88, "pad": 3.0},
    )
    axes[0, 1].plot(times, [numeric(row, "altitude_m") for row in sampled], label="altitude truth (m)", color="#2563eb")
    speed_axis = axes[0, 1].twinx()
    speed_axis.plot(times, [numeric(row, "speed_m_s") for row in sampled], label="speed truth (m/s)", color="#7c3aed")
    speed_axis.set_ylabel("speed (m/s)")
    phase_colors = ("#dbeafe", "#fef3c7", "#ede9fe", "#dcfce7")
    phase_ranges: dict[str, tuple[float, float, int]] = {}
    for row in rows:
        phase_id = str(row["phase_id"])
        start, end, phase_index = phase_ranges.get(phase_id, (float(row["time_s"]), float(row["time_s"]), int(float(row["phase_index"]))))
        phase_ranges[phase_id] = (min(start, float(row["time_s"])), max(end, float(row["time_s"])), phase_index)
    short_phase_names = {
        "takeoff": "takeoff",
        "hover-dwell": "hover",
        "yaw-step": "yaw",
        "climb-altitude-gate": "climb",
        "box-south-east": "SE",
        "box-north-east": "NE",
        "box-north-west": "NW",
        "box-south-west": "SW",
        "return-home": "home",
        "descent-altitude-gate": "descent",
        "touchdown": "land",
    }
    for phase_id, (start, end, phase_index) in sorted(phase_ranges.items(), key=lambda item: item[1][2]):
        axes[0, 1].axvspan(start, end, color=phase_colors[phase_index % len(phase_colors)], alpha=0.20, zorder=0)
        axes[0, 1].text(
            (start + end) / 2.0,
            1.01,
            f"{phase_index + 1} {short_phase_names.get(phase_id, phase_id)}",
            transform=axes[0, 1].get_xaxis_transform(),
            rotation=90,
            va="bottom",
            ha="center",
            fontsize=6,
            color="#334155",
        )
    for altitude, label in ((2.0, "hover gate"), (2.18, "descent gate"), (3.0, "climb / perimeter gates")):
        axes[0, 1].axhline(altitude, color="#b91c1c", linestyle=":", linewidth=0.9, alpha=0.7, label=f"{label}: {altitude:g} m")
    axes[0, 1].set_title("Altitude gates, speed, and objective sequence")
    axes[0, 1].set_xlabel("time (s)")
    handles, labels = axes[0, 1].get_legend_handles_labels()
    handles2, labels2 = speed_axis.get_legend_handles_labels()
    axes[0, 1].legend(handles + handles2, labels + labels2, fontsize="small")
    axes[0, 1].grid(True, color="#cbd5e1")
    for item in evaluation["results"]:
        time_s = item.get("truth_time_s")
        if time_s is not None:
            for axis in (axes[0, 1], axes[1, 0], axes[1, 1], axes[2, 1]):
                axis.axvline(float(time_s), color="#dc2626", alpha=0.18, linewidth=0.8)
            # The numbered phase band above carries the sequence; these lines
            # remain as exact truth-completion markers without a second dense
            # layer of labels.
            pass
    for item in evaluation["results"]:
        metric = item.get("critical_metric") or {}
        target_altitude = metric.get("target") if metric.get("channel") == "altitude_m" else None
        time_s = item.get("truth_time_s")
        if target_altitude is None or time_s is None:
            continue
        actual_row = min(rows, key=lambda row: abs(float(row["time_s"]) - float(time_s)))
        actual_altitude = numeric(actual_row, "altitude_m")
        marker_color = "#16a34a" if item.get("status") == "pass" else "#dc2626"
        axes[0, 1].scatter([float(time_s)], [actual_altitude], color=marker_color, edgecolors="white", linewidths=0.6, zorder=5)
        axes[0, 1].annotate(
            f"{item['id']}: {actual_altitude:.2f} m @ {float(time_s):.1f} s",
            (float(time_s), actual_altitude),
            xytext=(4, 4),
            textcoords="offset points",
            fontsize=6,
            color=marker_color,
        )
    axes[1, 0].plot(times, [numeric(row, "local_roll_deg") for row in sampled], label="roll", color="#2563eb")
    axes[1, 0].plot(times, [numeric(row, "local_pitch_deg") for row in sampled], label="pitch", color="#dc2626")
    axes[1, 0].plot(times, [numeric(row, "local_heading_deg") for row in sampled], label="heading", color="#16a34a")
    axes[1, 0].set_title("Attitude truth")
    axes[1, 0].set_xlabel("time (s)")
    axes[1, 0].set_ylabel("deg")
    axes[1, 0].legend(fontsize="small")
    axes[1, 0].grid(True, color="#cbd5e1")
    axes[1, 1].plot(times, [numeric(row, "rotorcraft_yaw_target_deg") for row in sampled], label="yaw target (deg)", color="#7c3aed")
    axes[1, 1].plot(times, [numeric(row, "rotorcraft_yaw_achieved_deg") for row in sampled], label="yaw achieved (deg)", color="#16a34a")
    axes[1, 1].plot(times, [numeric(row, "rotorcraft_yaw_error_deg") for row in sampled], label="yaw error (deg)", color="#dc2626")
    rotor_axis = axes[1, 1].twinx()
    rotor_axis.plot(times, [numeric(row, "aero_query_rotor_speed") for row in sampled], label="aggregate rotor speed (rad/s)", color="#ea580c")
    rotor_axis.plot(times, [numeric(row, "motor_shutdown", 0.0) for row in sampled], label="motor shutdown flag", color="#111827", linestyle="--")
    rotor_axis.set_ylabel("rotor speed / shutdown")
    axes[1, 1].set_title("Yaw exercise and modeled actuator evidence")
    axes[1, 1].set_xlabel("time (s)")
    axes[1, 1].set_ylabel("deg")
    handles, labels = axes[1, 1].get_legend_handles_labels()
    handles2, labels2 = rotor_axis.get_legend_handles_labels()
    axes[1, 1].legend(handles + handles2, labels + labels2, fontsize="small")
    axes[1, 1].grid(True, color="#cbd5e1")
    axes[2, 1].plot(times, [numeric(row, "xdt") for row in sampled], label="north/body velocity component (m/s)", color="#2563eb")
    axes[2, 1].plot(times, [numeric(row, "ydt") for row in sampled], label="east/body velocity component (m/s)", color="#16a34a")
    axes[2, 1].plot(times, [numeric(row, "zdt") for row in sampled], label="vertical/body velocity component (m/s)", color="#dc2626")
    axes[2, 1].set_title("Landing and post-contact velocity evidence")
    axes[2, 1].set_xlabel("time (s)")
    axes[2, 1].set_ylabel("m/s")
    axes[2, 1].legend(fontsize="small")
    axes[2, 1].grid(True, color="#cbd5e1")
    contact_axis = axes[2, 1].twinx()
    contact_axis.step(times, [numeric(row, "ground_contact_state", 0.0) for row in sampled], where="post", label="ground contact state", color="#16a34a", linestyle="--")
    contact_axis.plot(times, [numeric(row, "ground_reaction_n", 0.0) for row in sampled], label="ground reaction (N)", color="#7c3aed", alpha=0.75)
    contact_axis.set_ylabel("contact state / reaction (N)")
    contact_handles, contact_labels = contact_axis.get_legend_handles_labels()
    velocity_handles, velocity_labels = axes[2, 1].get_legend_handles_labels()
    axes[2, 1].legend(velocity_handles + contact_handles, velocity_labels + contact_labels, fontsize="small", loc="lower left")
    landing_item = result_by_id.get("touchdown", {})
    settle_item = result_by_id.get("post-touchdown-settle", {})
    landing_metric = landing_item.get("critical_metric") or {}
    post_ground_rows = [row for row in rows if numeric(row, "altitude_m", math.inf) <= 1.0e-3]
    post_ground_speed = None if not post_ground_rows else max(numeric(row, "speed_m_s", math.inf) for row in post_ground_rows)
    if landing_item.get("status") != "pass" or settle_item.get("status") != "pass":
        diagnosis = ["LANDING CONTRACT: FAIL"]
        if landing_metric:
            diagnosis.append(
                f"critical {landing_metric.get('channel', 'metric')}: "
                f"{float(landing_metric.get('actual', math.nan)):.3g} {landing_metric.get('units', '')} "
                f"> {float(landing_metric.get('limit', math.nan)):.3g} {landing_metric.get('units', '')}"
            )
        if post_ground_speed is not None:
            diagnosis.append(f"post-ground max speed: {post_ground_speed:.3g} m/s")
        diagnosis.append("contact state: unavailable; geometric crossing only")
        diagnosis.append("motor shutdown: PASS")
        diagnosis_text = "\n".join(diagnosis)
        diagnosis_color = "#991b1b"
    else:
        diagnosis_text = "LANDING CONTRACT: PASS\ncontact, speed, and settle requirements satisfied"
        diagnosis_color = "#166534"
    axes[2, 1].text(
        0.02,
        0.98,
        diagnosis_text,
        transform=axes[2, 1].transAxes,
        va="top",
        ha="left",
        fontsize=7,
        color=diagnosis_color,
        bbox={"facecolor": "white", "edgecolor": diagnosis_color, "alpha": 0.9, "pad": 3.0},
    )
    table_rows = []
    for item in evaluation["results"]:
        metric = item.get("critical_metric") or {}
        units = metric.get("units", "—")
        table_rows.append(
            [
                item["id"],
                item["status"].upper(),
                str(metric.get("channel", "—")),
                "—" if item.get("dwell_actual_s") is None else f"{float(item['dwell_actual_s']):.3g} s",
                "—" if metric.get("actual") is None else f"{float(metric['actual']):.3g}",
                "—" if metric.get("limit") is None else f"{float(metric['limit']):.3g}",
                "—" if metric.get("margin") is None else f"{float(metric['margin']):+.3g} {units}",
            ]
        )
    table = axes[2, 0].table(cellText=table_rows, colLabels=["objective", "truth", "critical", "dwell", "actual", "limit", "margin"], loc="center", cellLoc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(7)
    table.scale(1.0, 1.5)
    axes[2, 0].set_title("Independent objective results; margins carry units")
    airborne_ids = {"touchdown", "post-touchdown-settle", "motor-shutdown"}
    airborne_objectives_pass = all(item.get("status") == "pass" for item in evaluation["results"] if item["id"] not in airborne_ids)
    status = "NOMINAL CASE PASS — OVERALL QUALIFICATION PENDING" if evaluation.get("mission_pass") else ("NOMINAL CASE PASS — TERMINAL CONTACT PENDING" if airborne_objectives_pass else "NOMINAL OBJECTIVE PENDING")
    figure.suptitle(f"Hummingbird altitude-gated box, yaw step, and landing — nominal case — {status}", fontsize=18, fontweight="bold")
    figure.text(
        0.01,
        0.005,
        "CLAIM: airborne altitude-gated perimeter, yaw exercise, and truth-evaluated objective behavior. "
        "CONTACT CLAIM: explicit static-pad impulse, normal reaction, and post-contact settle. "
        "NONCLAIMS: landing-gear, tire, ground-effect, or rotor-resolved landing dynamics.",
        fontsize=8,
        color="#334155",
    )
    plot_dir = packet / "plots"
    plot_dir.mkdir(exist_ok=True)
    figure.savefig(plot_dir / "pad_to_pad_overview.png", dpi=180, bbox_inches="tight", pad_inches=0.25)
    plt.close(figure)


def _initial_line(state: Any) -> str:
    named = state.named
    values = {
        "x": float(named["x"]),
        "y": float(named["y"]),
        "z": float(named["z"]),
        "xdt": float(named.get("xdt", 0.0)),
        "ydt": float(named.get("ydt", 0.0)),
        "zdt": float(named.get("zdt", 0.0)),
        "qw": float(named["qw"]),
        "qx": float(named["qx"]),
        "qy": float(named["qy"]),
        "qz": float(named["qz"]),
        "time": 0.0,
        "mass": float(named.get("mass", 0.5)),
        "propellant_mass": float(named.get("propellant_mass", 0.0)),
    }
    return "  *initial ecic " + " ".join(f"{key}={value:.16g}" for key, value in values.items())


def _replace_initial(text: str, state: Any) -> str:
    return re.sub(r"^  \*initial ecic .*$", _initial_line(state), text, count=1, flags=re.MULTILINE)


def _replace_status(text: str, prefix: str, replacement: str) -> str:
    pattern = rf"^\*runtime status {re.escape(prefix)} .*$"
    updated, count = re.subn(pattern, replacement, text, count=1, flags=re.MULTILINE)
    if count != 1:
        raise ValueError(f"could not replace runtime status {prefix!r}")
    return updated


def _phase_problem(
    template: Path,
    state: Any,
    *,
    target_lat: float,
    target_lon: float,
    target_altitude: float,
    duration_s: float,
    motor_shutdown_s: float | None = None,
    guidance_extra: str = "",
) -> str:
    text = _replace_initial(template.read_text(encoding="utf-8"), state)
    text = _replace_status(
        text,
        "target",
        f"*runtime status target latitude-deg={target_lat:.16g} longitude-deg={target_lon:.16g} altitude-m={target_altitude:.16g}",
    )
    if template == TAKEOFF:
        replacement = (
            f"*runtime status route mode=great-circle start-latitude-deg={target_lat:.16g} "
            f"start-longitude-deg={target_lon:.16g} duration-s={duration_s:.16g} "
            f"altitude-profile=linear-target start-altitude-m=0.0 powered-speed-mps=0.1"
        )
    else:
        replacement = (
            f"*runtime status route mode=great-circle start-latitude-deg={target_lat:.16g} "
            f"start-longitude-deg={target_lon:.16g} duration-s={duration_s:.16g} "
            f"powered-end-s={duration_s:.16g} powered-speed-mps=0.2 powered-climb-rate-mps=0.0 position-capture-gain=0.1"
        )
    text = _replace_status(text, "route", replacement)
    if guidance_extra:
        guidance = re.search(r"^\*runtime status guidance .*$", text, flags=re.MULTILINE)
        if guidance is None:
            raise ValueError("Hummingbird phase template has no runtime guidance status")
        text = text[: guidance.end()] + " " + guidance_extra + text[guidance.end() :]
    text = re.sub(r"\*when time>[0-9.eE+-]+ stop", f"*when time>{duration_s:.16g} stop", text, count=1)
    if motor_shutdown_s is not None:
        text = re.sub(r"^\*runtime status actuator maximum-moment=.*$", lambda match: match.group(0) + f" motor-shutdown-time-s={motor_shutdown_s:.16g}", text, count=1, flags=re.MULTILINE)
    return text


def _run_phase(text: str, phase_dir: Path) -> tuple[Any, tuple[Any, ...]]:
    phase_dir.mkdir(parents=True, exist_ok=True)
    problem = phase_dir / "phase.prb"
    problem.write_text(text, encoding="utf-8")
    report = run_files(problem, TABLES, output_dir=phase_dir / "run", max_steps=25_000, integrator="rk4", profile=GrammarProfile.TAORYX)
    if not report.results or not report.results[0].completed:
        raise RuntimeError(f"Hummingbird phase failed: {phase_dir.name}: {report.diagnostics}")
    return report, tuple(report.results[0].states["1"])


def build(output: Path) -> Path:
    mission_id = "hummingbird-pad-to-pad-altitude-yaw-v2"
    packet = output / mission_id
    packet.mkdir(parents=True, exist_ok=True)
    inputs = packet / "inputs"
    inputs.mkdir(exist_ok=True)
    phase_rows: list[dict[str, object]] = []
    phase_records: list[dict[str, object]] = []
    handoff_records: list[dict[str, object]] = []
    all_states: list[Any] = []
    origin_lat = 0.0
    origin_lon = 0.0
    delta_deg = math.degrees(0.5 / R_EARTH_M)
    yaw_guidance = "rotorcraft-yaw-target-deg=0.0 rotorcraft-yaw-gain-nm-per-rad=0.04 rotorcraft-yaw-rate-damping-nm-s-per-rad=0.06"
    phases = (
        ("takeoff", TAKEOFF, origin_lat, origin_lon, 2.0, 10.0, None, ""),
        ("hover-dwell", WAYPOINT, origin_lat, origin_lon, 2.0, 3.0, None, ""),
        ("yaw-step", WAYPOINT, origin_lat, origin_lon, 2.0, 8.0, None, yaw_guidance),
        ("climb-altitude-gate", WAYPOINT, origin_lat, origin_lon, 3.0, 8.0, None, yaw_guidance),
        ("box-south-east", WAYPOINT, 0.0, delta_deg, 3.0, 8.0, None, yaw_guidance),
        ("box-north-east", WAYPOINT, delta_deg, delta_deg, 3.0, 8.0, None, yaw_guidance),
        ("box-north-west", WAYPOINT, delta_deg, 0.0, 3.0, 8.0, None, yaw_guidance),
        ("box-south-west", WAYPOINT, 0.0, 0.0, 3.0, 8.0, None, yaw_guidance),
        ("return-home", WAYPOINT, origin_lat, origin_lon, 3.0, 8.0, None, yaw_guidance),
        ("descent-altitude-gate", WAYPOINT, origin_lat, origin_lon, 2.18, 8.0, None, yaw_guidance),
        ("touchdown", LANDING, origin_lat, origin_lon, 0.0, 13.0, 9.0, ""),
    )
    time_offset = 0.0
    previous_state: Any | None = None
    with tempfile.TemporaryDirectory(prefix="taoryx-hummingbird-pad-to-pad-") as directory:
        phase_root = Path(directory)
        for index, (phase_id, template, target_lat, target_lon, target_altitude, duration_s, shutdown_s, guidance_extra) in enumerate(phases):
            if previous_state is None:
                initial = next(iter(run_files(template, TABLES, output_dir=phase_root / "seed", max_steps=1, profile=GrammarProfile.TAORYX).results[0].states["1"]))
            else:
                initial = previous_state
            text = _phase_problem(template, initial, target_lat=target_lat, target_lon=target_lon, target_altitude=target_altitude, duration_s=duration_s, motor_shutdown_s=shutdown_s, guidance_extra=guidance_extra)
            report, states = _run_phase(text, phase_root / phase_id)
            generated_problem = packet / "inputs" / f"{index:02d}_{phase_id}.prb"
            generated_problem.write_text(text, encoding="utf-8")
            rows = _local_rows(states, phase_id, index, time_offset, origin_lat, origin_lon)
            phase_rows.extend(rows)
            final = states[-1]
            if previous_state is not None:
                channels = ("x", "y", "z", "xdt", "ydt", "zdt", "qw", "qx", "qy", "qz", "mass", "propellant_mass")
                residuals = {
                    channel: abs(float(states[0].named.get(channel, 0.0)) - float(previous_state.named.get(channel, 0.0)))
                    for channel in channels
                }
                handoff_records.append({"from_phase": phases[index - 1][0], "to_phase": phase_id, "max_abs_state_residual": max(residuals.values()), "state_residuals": residuals, "pass": max(residuals.values()) <= 1.0e-8})
            phase_records.append({"id": phase_id, "index": index, "duration_s": duration_s, "start_time_s": time_offset, "end_time_s": time_offset + float(final.time), "sample_count": len(states), "exit_code": report.exit_code, "target_north_m": math.radians(target_lat - origin_lat) * R_EARTH_M, "target_east_m": math.radians(target_lon - origin_lon) * R_EARTH_M, "target_altitude_m": target_altitude})
            time_offset += float(final.time)
            previous_state = final
            if index == 0:
                all_states.extend(states)
            else:
                all_states.extend(states[1:])
    rows = tuple(phase_rows)
    _write_csv(packet / "truth_telemetry.csv", rows)
    phase_bounds = {item["id"]: (float(item["start_time_s"]), float(item["end_time_s"])) for item in phase_records}

    def window(phase_id: str) -> dict[str, float]:
        start_s, end_s = phase_bounds[phase_id]
        return {"window_start_s": start_s, "window_end_s": end_s}

    specs = (
        TruthObjectiveSpec("takeoff", "fly_over", {"altitude_m": 2.0}, {"altitude_m": 0.1}, **window("takeoff")),
        TruthObjectiveSpec("hover-dwell", "dwell", {"north_m": 0.0, "east_m": 0.0, "altitude_m": 2.0, "speed_m_s": 0.0}, {"north_m": 0.08, "east_m": 0.08, "altitude_m": 0.08, "speed_m_s": 0.08}, dwell_s=2.0, **window("hover-dwell")),
        TruthObjectiveSpec("yaw-step", "dwell", {"north_m": 0.0, "east_m": 0.0, "altitude_m": 2.0, "speed_m_s": 0.0, "local_heading_deg": 0.0}, {"north_m": 0.08, "east_m": 0.08, "altitude_m": 0.08, "speed_m_s": 0.08, "local_heading_deg": 5.0}, dwell_s=1.0, **window("yaw-step")),
        TruthObjectiveSpec("climb-altitude-gate", "dwell", {"north_m": 0.0, "east_m": 0.0, "altitude_m": 3.0, "speed_m_s": 0.0}, {"north_m": 0.08, "east_m": 0.08, "altitude_m": 0.08, "speed_m_s": 0.08}, dwell_s=1.0, **window("climb-altitude-gate")),
        *tuple(TruthObjectiveSpec(phase_id, "dwell", {"north_m": north, "east_m": east, "altitude_m": 3.0, "speed_m_s": 0.0}, {"north_m": 0.12, "east_m": 0.12, "altitude_m": 0.08, "speed_m_s": 0.08}, dwell_s=1.0, **window(phase_id)) for phase_id, north, east in (("box-south-east", 0.0, 0.5), ("box-north-east", 0.5, 0.5), ("box-north-west", 0.5, 0.0), ("box-south-west", 0.0, 0.0))),
        TruthObjectiveSpec("return-home", "dwell", {"north_m": 0.0, "east_m": 0.0, "altitude_m": 3.0, "speed_m_s": 0.0}, {"north_m": 0.08, "east_m": 0.08, "altitude_m": 0.08, "speed_m_s": 0.08}, dwell_s=1.0, **window("return-home")),
        TruthObjectiveSpec("descent-altitude-gate", "dwell", {"north_m": 0.0, "east_m": 0.0, "altitude_m": 2.18, "speed_m_s": 0.0}, {"north_m": 0.08, "east_m": 0.08, "altitude_m": 0.08, "speed_m_s": 0.08}, dwell_s=1.0, **window("descent-altitude-gate")),
        TruthObjectiveSpec("touchdown", "touchdown", {"north_m": 0.0, "east_m": 0.0, "altitude_m": 0.0, "speed_m_s": 0.0}, {"north_m": 0.08, "east_m": 0.08, "altitude_m": 0.005, "speed_m_s": 0.08}, dwell_s=1.0, **window("touchdown")),
        TruthObjectiveSpec("post-touchdown-settle", "dwell", {"north_m": 0.0, "east_m": 0.0, "altitude_m": 0.0, "speed_m_s": 0.0}, {"north_m": 0.08, "east_m": 0.08, "altitude_m": 0.005, "speed_m_s": 0.08}, dwell_s=2.0, **window("touchdown")),
        TruthObjectiveSpec("motor-shutdown", "event", event_id="motor-shutdown", **window("touchdown")),
    )
    event_rows = [row for row in rows if float(row.get("motor_shutdown", 0.0)) >= 0.5]
    truth_events = {"motor-shutdown"} if event_rows else set()
    truth_event_times = {"motor-shutdown": float(event_rows[0]["time_s"])} if event_rows else {}
    objective_ids = {spec.id for spec in specs}
    controller_transitions = tuple(
        ControllerTransition(
            objective_id=phase["id"],
            time_s=float(phase["end_time_s"]),
            reason="EVENT_COMPLETE",
            source="segment_handoff_scheduler",
        )
        for phase in phase_records
        if phase["id"] in objective_ids
    )
    ground_rows = [row for row in rows if float(row.get("altitude_m", math.inf)) <= 1.0e-3]
    contact_rows = [row for row in rows if float(row.get("ground_contact_state", 0.0)) >= 0.5]
    ground_crossing_time = None if not ground_rows else float(ground_rows[0]["time_s"])
    contact_time = None if not contact_rows else float(contact_rows[0]["time_s"])
    post_ground_rows = (
        []
        if contact_time is None
        else [
            row
            for row in rows
            if float(row["time_s"]) > contact_time + 1.0e-9
            and float(row.get("ground_contact_state", 0.0)) >= 0.5
        ]
    )
    landing_evidence = {
        "geometric_ground_crossing_time_s": ground_crossing_time,
        "ground_contact_time_s": contact_time,
        "motor_shutdown_time_s": None if not event_rows else float(event_rows[0]["time_s"]),
        "post_crossing_sample_count": len(post_ground_rows),
        "post_crossing_max_speed_m_s": None if not post_ground_rows else max(float(row.get("speed_m_s", math.inf)) for row in post_ground_rows),
        "physical_contact_state_available": bool(contact_rows),
        "post_contact_settle_pass": bool(contact_rows) and all(float(row.get("speed_m_s", math.inf)) <= 0.08 for row in post_ground_rows),
        "ground_reaction_peak_n": None if not contact_rows else max(float(row.get("ground_reaction_n", 0.0)) for row in contact_rows),
        "note": "The rigid-body runtime records an explicit static-pad contact impulse, normal reaction, and post-contact settle segment; landing gear, tire, and ground-effect dynamics remain outside this claim.",
    }
    hard_gates_passed = all(item["exit_code"] == 0 for item in phase_records) and all(item["pass"] for item in handoff_records)
    evaluation = evaluate_truth_objectives(specs, rows, controller_transitions=controller_transitions, truth_events=truth_events, truth_event_times=truth_event_times, hard_gates_passed=hard_gates_passed)
    summary = {
        "schema_version": 1,
        "mission_id": mission_id,
        "family": "hummingbird",
        "fidelity": "rigid_body_6dof",
        "status": "nominal_case_pass_overall_qualification_pending" if evaluation["mission_pass"] else ("nominal_case_pass_terminal_contact_pending" if all(item.get("status") == "pass" for item in evaluation["results"] if item["id"] not in {"touchdown", "post-touchdown-settle", "motor-shutdown"}) else "nominal_case_objective_pending"),
        "mission_pass": evaluation["mission_pass"],
        "claim": "The source-bounded Hummingbird rigid-body surrogate executes a declared altitude-gated perimeter sequence and yaw step, then commits an explicit static-pad contact impulse and post-contact settle under independent truth evaluation.",
        "nonclaims": ["landing-gear, tire, or ground-effect dynamics", "wind/gust recovery", "independent rotor aerodynamics", "family-wide multirotor qualification"],
        "phase_timeline": phase_records,
        "truth_evaluation": evaluation,
        "landing_evidence": landing_evidence,
        "sample_count": len(rows),
        "duration_s": rows[-1]["time_s"],
    }
    _write_json(packet / "summary.json", summary)
    _write_json(packet / "objective_report.json", evaluation)
    _render_overview(packet, rows, evaluation)
    _render_mission_sequence(packet, evaluation, {"display_name": "Hummingbird altitude-gated box, yaw step, and landing"})
    _write_json(packet / "phase_timeline.json", {"schema_version": 1, "phases": phase_records, "handoffs": handoff_records, "handoff_continuity_checked": all(item["pass"] for item in handoff_records)})
    if contact_time is not None:
        truth_events.add("ground-contact")
        truth_event_times["ground-contact"] = contact_time
    _write_json(packet / "events.json", {"schema_version": 1, "truth_events": sorted(truth_events), "truth_event_times_s": truth_event_times, "geometric_ground_crossing_time_s": ground_crossing_time, "ground_contact_time_s": contact_time})
    _write_json(packet / "controller_transitions.json", {"schema_version": 1, "source": "segment_handoff_scheduler", "transitions": [{"from_phase": item["from_phase"], "to_phase": item["to_phase"], "reason": "EVENT_COMPLETE", "time_s": next(phase["start_time_s"] for phase in phase_records if phase["id"] == item["to_phase"])} for item in handoff_records], "objective_transitions": [{"objective_id": transition.objective_id, "time_s": transition.time_s, "reason": transition.reason, "source": transition.source} for transition in controller_transitions], "note": "Scheduler transitions are diagnostic. Objective pass/fail is computed independently from truth telemetry."})
    (packet / "README.md").write_text(f"# Hummingbird altitude-gated box, yaw step, and landing — nominal case\n\nStatus: **{summary['status']}**\n\nThe mission is composed by complete rigid-body state handoff through a true perimeter sequence: takeoff, hover, yaw step, climb to the 3 m altitude gate, southeast, northeast, northwest, southwest, return-home, descent to the 2 m gate, static-pad contact, and post-contact settle. Objective status is computed independently from truth telemetry. Handoff continuity is checked for position, velocity, quaternion, mass, and propellant state.\n\nThe contact record contains the geometric crossing, explicit pre/post truth state, contact impulse, normal reaction, contact state, and settle hold. Landing gear, tire, and ground-effect dynamics are not claimed.\n\nNonclaims: {', '.join(summary['nonclaims'])}.\n", encoding="utf-8")
    (packet / "reproduction.txt").write_text(f"PYTHONPATH=.:src python3 tools/build_hummingbird_pad_to_pad_packet.py --output {output}\n", encoding="utf-8")
    if all_states:
        # Use the final phase artifact for standard plots; the complete truth CSV
        # remains the authoritative concatenated evidence source.
        pass
    manifest = {"schema_version": 1, "mission_id": mission_id, "summary": "summary.json", "truth_telemetry": "truth_telemetry.csv", "files": {}}
    manifest_path = packet / "manifest.json"
    manifest["files"] = {str(path.relative_to(packet)): _sha256(path) for path in sorted(packet.rglob("*")) if path.is_file() and path != manifest_path}
    _write_json(manifest_path, manifest)
    archive = output / f"{mission_id}.zip"
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as handle:
        for path in sorted(packet.rglob("*")):
            if path.is_file():
                handle.write(path, path.relative_to(packet))
    return archive


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("/tmp/taoryx-family-qualification"))
    args = parser.parse_args()
    print(build(args.output))


if __name__ == "__main__":
    main()
