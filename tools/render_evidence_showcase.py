"""Render reviewer-focused composites from a fidelity-ladder evidence packet.

This tool deliberately composes the already-rendered, packet-scoped plots.  It
does not rerun a simulation or silently mix runs from different packets.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import shutil
import zipfile
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PACKET = ROOT / "artifacts/verification/fidelity_ladder"
DEFAULT_OUTPUT = ROOT / "artifacts/verification/showcase_composites"

FAMILY_ORDER = ("b747", "skywalker-x8", "hummingbird", "x15")
FAMILY_LABELS = {
    "b747": "Boeing 747-100",
    "skywalker-x8": "Skywalker X8",
    "hummingbird": "AscTec Hummingbird",
    "x15": "X-15",
}
MISSION_PANELS = {
    "b747": (
        "1_taos-altitude-m.png",
        "1_taos-speed-m-s.png",
        "__route-error__",
        "__aero-angles__",
        "__velocity-angles__",
        "1_taos-translation-equation-residual-normalized.png",
    ),
    "skywalker-x8": (
        "1_taos-altitude-m.png",
        "1_taos-speed-m-s.png",
        "__route-error__",
        "__aero-angles__",
        "__velocity-angles__",
        "1_taos-translation-equation-residual-normalized.png",
    ),
    "hummingbird": (
        "1_taos-altitude-m.png",
        "1_taos-speed-m-s.png",
        "__route-error__",
        "__aero-angles__",
        "__velocity-angles__",
        "1_taos-translation-equation-residual-normalized.png",
    ),
    "x15": (
        "1_taos-altitude-m.png",
        "1_taos-speed-m-s.png",
        "__route-error__",
        "__aero-angles__",
        "__velocity-angles__",
        "1_taos-translation-equation-residual-normalized.png",
    ),
}
PANEL_TITLES = {
    "b747": ("Altitude", "Speed", "Route error", "AoA / bank / sideslip", "FPA / heading", "Independent translation closure"),
    "skywalker-x8": ("Altitude", "Speed", "Route error", "AoA / bank / sideslip", "FPA / heading", "Independent translation closure"),
    "hummingbird": ("Altitude", "Speed", "Route error", "AoA / bank / sideslip", "FPA / heading", "Independent translation closure"),
    "x15": ("Altitude", "Speed", "Route error", "AoA / bank / sideslip", "FPA / heading", "Independent translation closure"),
}

ANGLE_SERIES = {
    "aero": (
        ("AoA", ("aero_alpha_deg",), "deg", "#2563eb"),
        # The aero-query bank is a frame-relative lookup coordinate, not the
        # vehicle's physical local bank.  Never prefer it for this plot.
        ("Bank", ("route_bank_achieved_deg", "bank_achieved_deg", "local_roll_deg"), "deg", "#dc2626"),
        ("Sideslip", ("aero_sideslip_deg",), "deg", "#d97706"),
    ),
    "velocity": (
        ("FPA", ("flight_path_angle_deg", "aero_query_flight_path_angle_deg"), "deg", "#2563eb"),
        ("Heading", ("local_heading_deg",), "deg", "#7c3aed"),
    ),
}
####


def _route_objective(objectives: dict[str, dict[str, Any]]) -> dict[str, Any]:
    """Select the family-specific terminal route objective."""

    for objective_id in ("route-capture", "final-corner-capture", "return-capture", "route-transition"):
        if objective_id in objectives:
            return objectives[objective_id]
    return {}
####


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()
####


def _latest_packet(root: Path) -> Path:
    candidates = sorted((item for item in root.iterdir() if item.is_dir()), key=lambda item: item.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(f"no packet directories found under {root}")
    return candidates[-1]
####


def _mission_records(packet: Path) -> dict[str, tuple[Path, dict[str, Any]]]:
    records: dict[str, tuple[Path, dict[str, Any]]] = {}
    directory = packet / "controller-missions"
    for summary_path in directory.glob("*/summary.json"):
        payload = json.loads(summary_path.read_text(encoding="utf-8"))
        mission_id = summary_path.parent.name
        family = mission_id.split("-", 1)[0]
        if mission_id.startswith("skywalker-x8"):
            family = "skywalker-x8"
        records[family] = (summary_path.parent, payload)
    missing = [family for family in FAMILY_ORDER if family not in records]
    if missing:
        raise ValueError(f"packet is missing controller mission summaries: {', '.join(missing)}")
    return records
####


def _mission_metrics(payload: dict[str, Any]) -> dict[str, Any]:
    objectives = payload.get("objective_evaluation", {}).get("objectives", [])
    by_id = {str(item.get("id")): item for item in objectives}
    closure = payload.get("closure_evaluation", {}).get("checks", {})
    route = _route_objective(by_id)
    envelope = next((item for item in objectives if item.get("kind") == "envelope"), {})
    return {
        "score": payload.get("objective_evaluation", {}).get("score"),
        "duration_s": by_id.get("duration", {}).get("actual"),
        "route_m": route.get("actual"),
        "route_target_m": route.get("target"),
        "envelope_actual": envelope.get("actual"),
        "envelope_target": envelope.get("target"),
        "envelope_unit": envelope.get("unit", "1"),
        "closure_translation": closure.get("translation_p99", {}).get("actual"),
        "closure_rotation": closure.get("rotation_p99", {}).get("actual"),
        "status": payload.get("objective_evaluation", {}).get("status", "unknown"),
        "continuity": payload.get("event_continuity_audit", {}).get("status", "unavailable"),
        "pro_nav": bool(payload.get("telemetry_metrics", {}).get("final", {}).get("pro_nav_active", 0.0)),
    }
####


def _render_scoreboard(records: dict[str, tuple[Path, dict[str, Any]]], output: Path, plt: Any) -> None:
    rows: list[list[str]] = []
    for family in FAMILY_ORDER:
        metrics = _mission_metrics(records[family][1])
        route = "—" if metrics["route_m"] is None else f"{metrics['route_m']:.1f} / {metrics['route_target_m']:.1f} m"
        closure = "—" if metrics["closure_translation"] is None else f"{metrics['closure_translation']:.1e}"
        envelope = "—" if metrics["envelope_actual"] is None else f"{metrics['envelope_actual']:.2f} / {metrics['envelope_target']:.2f} {metrics['envelope_unit']}"
        rows.append(
            [
                FAMILY_LABELS[family],
                str(metrics["status"]).upper(),
                f"{metrics['score']:.2f}" if metrics["score"] is not None else "—",
                f"{metrics['duration_s']:.1f} s" if metrics["duration_s"] is not None else "—",
                route,
                envelope,
                "see family panel",
                closure,
                str(metrics["continuity"]).upper(),
            ]
        )
    figure, axis = plt.subplots(figsize=(16, 5.8), layout="constrained")
    axis.axis("off")
    axis.set_title("TAORYX controller-mission evidence — four-family scoreboard", loc="left", fontsize=17, fontweight="bold", pad=18)
    axis.text(0.0, 1.02, "Native problem-file runs • canonical SI telemetry • objective and closure gates", transform=axis.transAxes, fontsize=10)
    table = axis.table(
        cellText=rows,
        colLabels=["Family", "Status", "Score / 100", "Duration", "Final / limit", "Envelope actual / limit", "Other envelope", "Force p99", "Continuity"],
        cellLoc="center",
        colLoc="center",
        bbox=(0.0, 0.28, 1.0, 0.58),
    )
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 1.8)
    for (row, column), cell in table.get_celld().items():
        cell.set_edgecolor("#cbd5e1")
        if row == 0:
            cell.set_facecolor("#e2e8f0")
            cell.set_text_props(weight="bold")
        elif column == 1:
            cell.set_facecolor("#dcfce7")
            cell.set_text_props(weight="bold", color="#166534")
    axis.text(
        0.0,
        0.08,
        "Interpretation: this is source-bounded research-surrogate evidence, not flight qualification, historical TAOS compatibility, or certification.",
        transform=axis.transAxes,
        fontsize=10,
        color="#7f1d1d",
    )
    figure.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(figure)
####


def _render_plant_scoreboard(packet: Path, output: Path, plt: Any) -> None:
    """Show the longer plant-validation gates separately from controller missions."""

    rows: list[list[str]] = []
    for family in ("b747", "skywalker_x8", "hummingbird", "x15"):
        summary = json.loads((packet / "families" / family / "long-validation" / "summary.json").read_text(encoding="utf-8"))
        objective = summary.get("objective_evaluation", {})
        closure = summary.get("follow_on", {}).get("closure_evaluation", {}).get("checks", {})
        convergence = summary.get("convergence", {})
        rows.append(
            [
                FAMILY_LABELS["skywalker-x8" if family == "skywalker_x8" else family],
                str(objective.get("status", "unknown")).upper(),
                f"{objective.get('score', 0.0):.2f}",
                f"{summary.get('catalog', {}).get('duration_s', 0.0):.0f} s",
                f"{closure.get('translation_p99', {}).get('actual', float('nan')):.1e}",
                f"{closure.get('rotation_p99', {}).get('actual', float('nan')):.1e}",
                str(convergence.get("status", "unknown")).upper(),
            ]
        )
    figure, axis = plt.subplots(figsize=(14, 5.2), layout="constrained")
    axis.axis("off")
    axis.set_title("TAORYX plant-validation evidence — sustained family-appropriate cases", loc="left", fontsize=17, fontweight="bold", pad=18)
    axis.text(0.0, 1.02, "Long-validation catalog gates • independent closure p99 • fixed-step convergence", transform=axis.transAxes, fontsize=10)
    table = axis.table(
        cellText=rows,
        colLabels=["Family", "Status", "Score / 100", "Catalog duration", "Force p99", "Moment p99", "Convergence"],
        cellLoc="center",
        colLoc="center",
        bbox=(0.0, 0.30, 1.0, 0.55),
    )
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 1.8)
    for (row, column), cell in table.get_celld().items():
        cell.set_edgecolor("#cbd5e1")
        if row == 0:
            cell.set_facecolor("#e2e8f0")
            cell.set_text_props(weight="bold")
        elif column in (1, 6):
            cell.set_facecolor("#dcfce7")
            cell.set_text_props(weight="bold", color="#166534")
    axis.text(0.0, 0.08, "These are research-surrogate plant gates, not global envelope validation or engineering certification.", transform=axis.transAxes, fontsize=10, color="#7f1d1d")
    figure.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(figure)
####


def _telemetry_rows(mission_dir: Path) -> list[dict[str, float]]:
    """Read canonical-SI mission telemetry for custom composite panels."""

    path = mission_dir / "run" / "telemetry.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        rows: list[dict[str, float]] = []
        for raw in csv.DictReader(stream):
            row: dict[str, float] = {}
            for key, value in raw.items():
                if value in (None, ""):
                    continue
                try:
                    number = float(value)
                except ValueError:
                    continue
                if math.isfinite(number):
                    row[key] = number
            rows.append(row)
    return rows
    ####


def _series_from_rows(rows: list[dict[str, float]], names: tuple[str, ...]) -> tuple[str, list[float], list[float]] | None:
    """Select the first available numeric telemetry channel."""

    for name in names:
        points = [(row["time_s"], row[name]) for row in rows if "time_s" in row and name in row]
        if points:
            return name, [point[0] for point in points], [point[1] for point in points]
    return None
    ####


def _objective_markers(payload: dict[str, Any], rows: list[dict[str, float]]) -> list[tuple[float, str]]:
    """Return family-specific objective and execution events.

    Objective scoring is usually evaluated at the end of a mission, so its
    records do not necessarily contain execution times.  When the telemetry
    exposes them, route-leg changes and ProNav activation provide the actual
    family-specific event markers.  We do not invent timestamps for objectives
    that were only scored as final-state or envelope checks.
    """

    if not rows:
        return []
    start = rows[0].get("time_s", 0.0)
    finish = rows[-1].get("time_s", start)
    markers: list[tuple[float, str]] = [(start, "objective evaluation start"), (finish, "objective evaluation finish")]
    for objective in payload.get("objective_evaluation", {}).get("objectives", []):
        value = objective.get("time_s")
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            markers.append((float(value), str(objective.get("id", "objective"))))
        elif objective.get("source") == "final" and objective.get("kind") in {"waypoint", "route_transition"}:
            # Preserve the distinction between an observed transition and a
            # final-state score: this is a completion evaluation marker, not an
            # invented waypoint-arrival timestamp.
            markers.append((finish, f"final: {objective.get('id', 'objective')}"))
    previous_leg: float | None = None
    for row in rows:
        time_s = row.get("time_s")
        if time_s is None:
            continue
        leg = row.get("route_leg_index")
        if leg is not None and leg != previous_leg:
            if previous_leg is not None:
                markers.append((time_s, f"waypoint leg {int(leg)}"))
            previous_leg = leg
    previous_phase: float | None = None
    for row in rows:
        time_s = row.get("time_s")
        if time_s is None:
            continue
        phase = row.get("route_phase_index")
        if phase is not None and phase != previous_phase:
            if previous_phase is not None:
                markers.append((time_s, f"figure-eight phase {int(phase)}"))
            previous_phase = phase
    previous_segment: float | None = None
    for row in rows:
        time_s = row.get("time_s")
        segment = row.get("_segment")
        if time_s is None or segment is None:
            continue
        if previous_segment is not None and segment != previous_segment:
            markers.append((time_s, f"segment {int(segment)}"))
        previous_segment = segment
    previous_pro_nav = 0.0
    for row in rows:
        time_s = row.get("time_s")
        active = row.get("pro_nav_active")
        if time_s is None or active is None:
            continue
        if active > 0.5 and previous_pro_nav <= 0.5:
            markers.append((time_s, "ProNav activation"))
        previous_pro_nav = active
    unique: dict[float, str] = {}
    for time_s, label in markers:
        key = round(time_s, 9)
        if key in unique and label not in unique[key].split(" / "):
            unique[key] = f"{unique[key]} / {label}"
        else:
            unique.setdefault(key, label)
    return sorted((time_s, label) for time_s, label in unique.items())
    ####


def _compact_marker_label(label: str) -> str:
    """Keep per-family timeline annotations readable without changing meaning."""

    if label == "objective evaluation start":
        return "start"
    if label == "objective evaluation finish":
        return "finish"
    if label == "ProNav activation":
        return "PN"
    if label.startswith("waypoint leg "):
        return "W" + label.removeprefix("waypoint leg ")
    if label.startswith("figure-eight phase "):
        return "P" + label.removeprefix("figure-eight phase ")
    if label.startswith("segment "):
        return "S" + label.removeprefix("segment ")
    if label.startswith("final: "):
        return "final"
    if " / " in label:
        return " / ".join(_compact_marker_label(part) for part in label.split(" / "))
    return label
    ####


def _draw_angle_panel(axis: Any, rows: list[dict[str, float]], kind: str, markers: list[tuple[float, str]]) -> None:
    """Draw a semantic angle panel directly from packet telemetry."""

    plotted = False
    for label, names, unit, color in ANGLE_SERIES[kind]:
        selected = _series_from_rows(rows, names)
        if selected is None:
            continue
        selected_name, times, values = selected
        display_label = label
        if label == "Bank":
            if selected_name == "local_roll_deg":
                display_label = "Bank (local roll)"
            elif selected_name in {"route_bank_achieved_deg", "bank_achieved_deg"}:
                display_label = "Bank (achieved)"
        axis.plot(times, values, label=display_label, color=color, linewidth=1.8)
        plotted = True
    for time_s, label in markers:
        axis.axvline(time_s, color="#dc2626", linestyle="--", linewidth=0.9, alpha=0.55)
    axis.set_xlabel("time (s)")
    axis.set_ylabel("deg")
    axis.grid(True, color="#cbd5e1", linewidth=0.7)
    if plotted:
        axis.legend(fontsize="x-small", loc="best")
    else:
        axis.text(0.5, 0.5, "angle channels unavailable", ha="center", va="center", transform=axis.transAxes)
    ####


def _draw_route_panel(axis: Any, rows: list[dict[str, float]], markers: list[tuple[float, str]]) -> None:
    """Draw route error and annotate family-local steering/mode events."""

    selected = _series_from_rows(rows, ("route_target_error_m", "route_cross_track_error_m"))
    if selected is None:
        axis.text(0.5, 0.5, "route metrics unavailable", ha="center", va="center", transform=axis.transAxes)
    else:
        name, times, values = selected
        label = "Distance to active target (m)" if name == "route_target_error_m" else "Cross-track error (m)"
        axis.plot(times, values, label=label, color="#2563eb", linewidth=1.8)
        axis.legend(fontsize="x-small", loc="best")
    for time_s, label in markers:
        axis.axvline(time_s, color="#dc2626", linestyle="--", linewidth=0.9, alpha=0.55)
        axis.text(
            time_s,
            0.98,
            _compact_marker_label(label),
            color="#b91c1c",
            fontsize=8,
            ha="left",
            va="top",
            rotation=90,
            transform=axis.get_xaxis_transform(),
            clip_on=True,
        )
    axis.set_xlabel("time (s)")
    axis.set_ylabel("m")
    axis.grid(True, color="#cbd5e1", linewidth=0.7)
    ####


def _render_objective_score_timeline(records: dict[str, tuple[Path, dict[str, Any]]], output: Path, plt: Any) -> None:
    """Show mission scores against their evaluation windows and objective events."""

    figure, axis = plt.subplots(figsize=(15, 6.5), layout="constrained")
    durations: list[float] = []
    for row, family in enumerate(FAMILY_ORDER):
        mission_dir, payload = records[family]
        telemetry = _telemetry_rows(mission_dir)
        markers = _objective_markers(payload, telemetry)
        duration = telemetry[-1].get("time_s", 0.0) if telemetry else 0.0
        durations.append(duration)
        score = payload.get("objective_evaluation", {}).get("score")
        axis.hlines(row, 0.0, duration, color="#94a3b8", linewidth=3.0)
        axis.scatter([duration], [row], color="#2563eb", s=55, zorder=3)
        if score is not None:
            axis.text(duration, row + 0.14, f"score {float(score):.1f}", ha="right", va="bottom", fontsize=9)
        for time_s, label in markers:
            # Keep event lines on the owning family's lane.  A full-height
            # line would falsely suggest that another vehicle shared the
            # same waypoint or guidance event.
            axis.vlines(time_s, row - 0.22, row + 0.22, color="#dc2626", linestyle="--", linewidth=0.9, alpha=0.55)
            short_label = _compact_marker_label(label)
            axis.text(time_s, row + 0.16, short_label, color="#b91c1c", fontsize=8, ha="left", va="bottom", rotation=90)
    axis.set_yticks(range(len(FAMILY_ORDER)), [FAMILY_LABELS[family] for family in FAMILY_ORDER])
    axis.set_xlabel("mission time (s)")
    axis.set_xlim(left=0.0, right=max(durations, default=1.0) * 1.04)
    axis.set_title("Controller-mission scores and objective evaluation windows", loc="left", fontsize=16, fontweight="bold", pad=18)
    figure.text(0.01, 0.01, "Each family has its own event lane: route-leg changes, ProNav activation, declared objective times, and evaluation bounds. Final-only objectives are not assigned invented times.", fontsize=9, color="#475569")
    axis.grid(True, axis="x", color="#cbd5e1", linewidth=0.7)
    figure.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(figure)
    ####


def _load_image(path: Path, plt: Any) -> Any:
    if not path.exists():
        return None
    return plt.imread(path)
####


def _render_mission_grid(family: str, mission_dir: Path, output: Path, plt: Any) -> None:
    figure, axes = plt.subplots(2, 3, figsize=(18, 9.5), layout="constrained")
    telemetry = _telemetry_rows(mission_dir)
    payload = json.loads((mission_dir / "summary.json").read_text(encoding="utf-8"))
    markers = _objective_markers(payload, telemetry)
    for axis, title, filename in zip(axes.flat, PANEL_TITLES[family], MISSION_PANELS[family], strict=True):
        axis.axis("off")
        axis.set_title(title, loc="left", fontsize=11, fontweight="bold")
        if filename == "__route-error__":
            axis.axis("on")
            _draw_route_panel(axis, telemetry, markers)
        elif filename == "__aero-angles__":
            axis.axis("on")
            _draw_angle_panel(axis, telemetry, "aero", markers)
        elif filename == "__velocity-angles__":
            axis.axis("on")
            _draw_angle_panel(axis, telemetry, "velocity", markers)
        else:
            image = _load_image(mission_dir / "plots" / filename, plt)
            if image is not None:
                axis.imshow(image)
            else:
                axis.text(0.5, 0.5, f"missing: {filename}", ha="center", va="center")
    figure.suptitle(f"{FAMILY_LABELS[family]} — strongest native controller-mission evidence", fontsize=17, fontweight="bold")
    figure.text(0.01, 0.005, "All panels are packet-rendered telemetry. Closure panels are independent recomputations; event boundaries are excluded where declared.", fontsize=9, color="#475569")
    figure.savefig(output, dpi=150, bbox_inches="tight")
    plt.close(figure)
####


def _render_mission_overview(records: dict[str, tuple[Path, dict[str, Any]]], output: Path, plt: Any) -> None:
    figure, axes = plt.subplots(4, 4, figsize=(20, 18), layout="constrained")
    for row, family in enumerate(FAMILY_ORDER):
        mission_dir, _ = records[family]
        overview_panels = (
            ("Altitude", "1_taos-altitude-m.png"),
            ("Speed", "1_taos-speed-m-s.png"),
            PANEL_TITLES[family][2],
            ("Independent force closure", "1_taos-translation-equation-residual-normalized.png"),
        )
        for column, panel in enumerate(overview_panels):
            title, filename = panel if isinstance(panel, tuple) else (panel, MISSION_PANELS[family][2])
            axis = axes[row, column]
            image = _load_image(mission_dir / "plots" / filename, plt)
            axis.axis("off")
            axis.set_title(title, loc="left", fontsize=11, fontweight="bold")
            if image is not None:
                axis.imshow(image)
            else:
                axis.text(0.5, 0.5, "unavailable", ha="center", va="center")
            if column == 0:
                axis.text(-0.02, 0.5, FAMILY_LABELS[family], transform=axis.transAxes, rotation=90, va="center", ha="right", fontsize=11, fontweight="bold")
    figure.suptitle("TAORYX controller missions — compact cross-family evidence overview", fontsize=18, fontweight="bold")
    figure.text(0.01, 0.005, "Rows are distinct native problem-file missions; this is not a cross-fidelity overlay. Units are embedded in each source plot.", fontsize=10, color="#475569")
    figure.savefig(output, dpi=140, bbox_inches="tight")
    plt.close(figure)
####


def _render_manifest(records: dict[str, tuple[Path, dict[str, Any]]], packet: Path, output: Path) -> None:
    packet_reference = str(packet.relative_to(ROOT)) if packet.is_relative_to(ROOT) else packet.name
    entries: list[dict[str, Any]] = []
    for family in FAMILY_ORDER:
        mission_dir, payload = records[family]
        entries.append(
            {
                "family": family,
                "mission_id": mission_dir.name,
                "summary_sha256": _sha256(mission_dir / "summary.json"),
                "metrics": _mission_metrics(payload),
                "source_packet": packet_reference,
            }
        )
    manifest = {
        "schema_version": "showcase-composite-v1",
        "source_packet": packet_reference,
        "source_packet_manifest_sha256": _sha256(packet / "manifest.json"),
        "claim_boundary": "source-bounded research-surrogate evidence; not flight qualification or historical TAOS compatibility",
        "composites": ["plant-validation-scoreboard.png", "controller-mission-scoreboard.png", "objective-score-timeline.png", "controller-missions-overview.png"] + [f"{family}-controller-evidence.png" for family in FAMILY_ORDER],
        "missions": entries,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "README.md").write_text(
        "# TAORYX evidence showcase\n\n"
        "These composites are derived from the clean, hashed fidelity-ladder packet named in `manifest.json`. "
        "They emphasize native controller missions, objective scores, route errors, combined AoA/bank/sideslip and FPA/heading panels, envelope signals, and independent closure. "
        "The objective timeline and each family route-error panel keep event markers on the owning family: route-leg changes (W1/W2/W3), segment transitions, ProNav activation, declared objective times, and final-state completion checks. It does not assign invented times to final-only objectives. Hummingbird uses local roll as the declared bank fallback when no aerodynamic bank channel is available.\n\n"
        "The evidence is source-bounded research-surrogate evidence. It is not flight qualification, historical TAOS 96.0 compatibility, or certification.\n",
        encoding="utf-8",
    )
####


def _zip_output(output: Path) -> Path:
    archive = output.with_suffix(".zip")
    if archive.exists():
        archive.unlink()
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as bundle:
        for path in sorted(output.rglob("*")):
            if path.is_file() and ".mplconfig" not in path.parts:
                bundle.write(path, path.relative_to(output.parent))
    return archive
####


def generate(packet: Path, output: Path) -> Path:
    """Generate composites and a manifest from one immutable packet directory."""
    if not (packet / "manifest.json").exists():
        raise FileNotFoundError(f"not a fidelity-ladder packet: {packet}")
    if output.exists():
        shutil.rmtree(output)
    output.mkdir(parents=True)
    os_mpl = output / ".mplconfig"
    os_mpl.mkdir()
    os.environ.setdefault("MPLCONFIGDIR", str(os_mpl))
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    records = _mission_records(packet)
    _render_plant_scoreboard(packet, output / "plant-validation-scoreboard.png", plt)
    _render_scoreboard(records, output / "controller-mission-scoreboard.png", plt)
    _render_objective_score_timeline(records, output / "objective-score-timeline.png", plt)
    _render_mission_overview(records, output / "controller-missions-overview.png", plt)
    for family in FAMILY_ORDER:
        _render_mission_grid(family, records[family][0], output / f"{family}-controller-evidence.png", plt)
    _render_manifest(records, packet, output)
    return _zip_output(output)
####


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--packet", type=Path, help="packet directory; defaults to the newest fidelity-ladder packet")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT / "latest")
    arguments = parser.parse_args()
    packet = arguments.packet or _latest_packet(DEFAULT_PACKET)
    print(generate(packet.resolve(), arguments.output.resolve()))
    ####


if __name__ == "__main__":
    main()
