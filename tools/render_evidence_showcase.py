"""Render reviewer-focused composites from a fidelity-ladder evidence packet.

This tool deliberately composes the already-rendered, packet-scoped plots.  It
does not rerun a simulation or silently mix runs from different packets.
"""

from __future__ import annotations

import argparse
import hashlib
import json
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
        "1_taos-local-heading-deg.png",
        "1_taos-aero-alpha-deg.png",
        "1_taos-aero-sideslip-deg.png",
        "1_taos-translation-equation-residual-normalized.png",
    ),
    "skywalker-x8": (
        "1_taos-altitude-m.png",
        "1_taos-speed-m-s.png",
        "1_taos-route-target-error-m.png",
        "1_taos-aero-alpha-deg.png",
        "1_taos-aero-sideslip-deg.png",
        "1_taos-translation-equation-residual-normalized.png",
    ),
    "hummingbird": (
        "1_taos-altitude-m.png",
        "1_taos-speed-m-s.png",
        "1_taos-route-target-error-m.png",
        "1_taos-local-heading-deg.png",
        "1_taos-local-roll-deg.png",
        "1_taos-translation-equation-residual-normalized.png",
    ),
    "x15": (
        "1_taos-altitude-m.png",
        "1_taos-speed-m-s.png",
        "1_taos-local-heading-deg.png",
        "1_taos-aero-alpha-deg.png",
        "1_taos-aero-sideslip-deg.png",
        "1_taos-translation-equation-residual-normalized.png",
    ),
}
PANEL_TITLES = {
    "b747": ("Altitude", "Speed", "Local heading", "Aerodynamic state", "Lateral state", "Independent translation closure"),
    "skywalker-x8": ("Altitude", "Speed", "Route error", "Aerodynamic state", "Sideslip", "Independent translation closure"),
    "hummingbird": ("Altitude", "Speed", "Route error", "Heading", "Roll", "Independent translation closure"),
    "x15": ("Altitude", "Speed", "Local heading", "Aerodynamic state", "Sideslip", "Independent translation closure"),
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


def _load_image(path: Path, plt: Any) -> Any:
    if not path.exists():
        return None
    return plt.imread(path)
####


def _render_mission_grid(family: str, mission_dir: Path, output: Path, plt: Any) -> None:
    figure, axes = plt.subplots(2, 3, figsize=(18, 9.5), layout="constrained")
    for axis, title, filename in zip(axes.flat, PANEL_TITLES[family], MISSION_PANELS[family], strict=True):
        image = _load_image(mission_dir / "plots" / filename, plt)
        axis.axis("off")
        axis.set_title(title, loc="left", fontsize=11, fontweight="bold")
        if image is None:
            axis.text(0.5, 0.5, f"missing: {filename}", ha="center", va="center")
        else:
            axis.imshow(image)
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
        "composites": ["plant-validation-scoreboard.png", "controller-mission-scoreboard.png", "controller-missions-overview.png"] + [f"{family}-controller-evidence.png" for family in FAMILY_ORDER],
        "missions": entries,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "README.md").write_text(
        "# TAORYX evidence showcase\n\n"
        "These composites are derived from the clean, hashed fidelity-ladder packet named in `manifest.json`. "
        "They emphasize native controller missions, objective scores, route errors, envelope signals, and independent closure.\n\n"
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
