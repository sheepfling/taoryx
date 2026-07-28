"""Compose the X8 qualification boards into one reviewer-facing composite.

The composite intentionally places the direct-moment baseline beside the
physical elevon-allocation candidate.  A successful baseline is not allowed to
silently stand in for surface-control qualification.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROOT = ROOT / "artifacts/showcases/x8_working"
####


def _read_summary(root: Path, mission_id: str) -> dict[str, Any]:
    return json.loads((root / mission_id / "summary.json").read_text(encoding="utf-8"))
####


def _status_card(axis: Any, summary: dict[str, Any], title: str, note: str, realization: dict[str, Any]) -> None:
    axis.set_facecolor("#fff7ed")
    axis.set_xlim(0.0, 1.0)
    axis.set_ylim(0.0, 1.0)
    axis.axis("off")
    axis.text(0.04, 0.90, title, fontsize=18, fontweight="bold", color="#991b1b", va="top")
    axis.text(
        0.04,
        0.76,
        f"STATUS: {str(summary.get('status', 'unknown')).replace('_', ' ').upper()}",
        fontsize=13,
        fontweight="bold",
        color="#b91c1c",
        va="top",
    )
    realization = summary.get("realization", {}) or realization
    lines = [
        f"actuator: {realization.get('actuator_realization', 'undeclared')}",
        f"moment source: {realization.get('moment_source', 'undeclared')}",
        f"runtime exit: {summary.get('run', {}).get('exit_code', '—')}",
        f"truth objectives: {summary.get('truth_evaluation', {}).get('required_passed', 0)} / {summary.get('truth_evaluation', {}).get('required_objectives', 0)}",
        "",
        note,
        "",
        "This is a promotion blocker, not a hidden fallback.",
    ]
    axis.text(0.04, 0.64, "\n".join(lines), fontsize=11, color="#334155", va="top", wrap=True)
####


def build(root: Path) -> Path:
    from textwrap import fill

    from PIL import Image, ImageDraw, ImageFont

    figure_eight_id = "x8-figure-eight-altitude-bank-pitch-v2"
    direct_id = "x8-racetrack-altitude-turns-direct-moment-baseline-v1"
    physical_id = "x8-racetrack-altitude-turns-v1"
    figure_eight = _read_summary(root, figure_eight_id)
    direct = _read_summary(root, direct_id)
    physical = _read_summary(root, physical_id)

    output = root / "x8-showcase-composite.png"
    canvas = Image.new("RGB", (3840, 2160), "white")
    draw = ImageDraw.Draw(canvas)
    try:
        font_path = "/System/Library/Fonts/Supplemental/Arial.ttf"
        title_font = ImageFont.truetype(font_path, 48)
        panel_font = ImageFont.truetype(font_path, 34)
        body_font = ImageFont.truetype(font_path, 28)
    except OSError:
        title_font = panel_font = body_font = ImageFont.load_default()

    draw.text((60, 30), "TAORYX Skywalker X8 showcase composite — nominal behavior and control-realization boundary", fill="#111827", font=title_font)
    panel_specs = (
        (figure_eight_id, "Nominal airborne objective witness", (60, 135, 1860, 1000)),
        (direct_id, "Racetrack direct-moment baseline", (1980, 135, 3780, 1000)),
    )
    for mission_id, label, box in panel_specs:
        left, top, right, bottom = box
        draw.text((left, 95), label, fill="#111827", font=panel_font)
        image = Image.open(root / mission_id / "qualification_board.png").convert("RGB")
        image.thumbnail((right - left, bottom - top), Image.Resampling.LANCZOS)
        paste_x = left + ((right - left) - image.width) // 2
        paste_y = top + ((bottom - top) - image.height) // 2
        canvas.paste(image, (paste_x, paste_y))

    x0, y0, x1, y1 = 60, 1080, 1860, 2100
    physical_pass = bool(physical.get("mission_pass"))
    physical_edge = "#16a34a" if physical_pass else "#dc2626"
    physical_fill = "#f0fdf4" if physical_pass else "#fff7ed"
    draw.rectangle((x0, y0, x1, y1), fill=physical_fill, outline=physical_edge, width=4)
    draw.text((x0 + 30, y0 + 30), "Physical elevon-allocation qualification candidate", fill="#166534" if physical_pass else "#991b1b", font=panel_font)
    board = Image.open(root / physical_id / "qualification_board.png").convert("RGB")
    board.thumbnail((x1 - x0 - 60, y1 - y0 - 120), Image.Resampling.LANCZOS)
    canvas.paste(board, (x0 + 30 + ((x1 - x0 - 60) - board.width) // 2, y0 + 95))

    x0, y0, x1, y1 = 1980, 1080, 3780, 2100
    draw.rectangle((x0, y0, x1, y1), fill="#f8fafc", outline="#64748b", width=3)
    draw.text((x0 + 30, y0 + 30), "X8 control-realization map", fill="#0f172a", font=panel_font)
    map_text = (
        "NOMINAL WITNESS\n"
        "guidance → attitude LQR → direct canonical body moment → source aero tables → plant\n\n"
        "RACETRACK BASELINE\n"
        "guidance → attitude LQR → direct canonical body moment → source aero tables → plant\n\n"
        "PHYSICAL RACETRACK RESULT\n"
        "guidance → bank/pitch response law → bounded local elevon inversion → source aero tables → plant\n"
        f"truth objectives: {physical.get('truth_evaluation', {}).get('required_passed', 0)} / {physical.get('truth_evaluation', {}).get('required_objectives', 0)}\n"
        f"terminal: {'PASS' if physical.get('mission_pass') else 'PENDING'}; envelope: {'PASS' if physical.get('envelope_report', {}).get('pass') else 'FAIL'}\n\n"
        "QUALIFICATION BOUNDARY\n"
        "• truth objectives are independent of controller transitions\n"
        "• all controlled roll/pitch moments are allocated through the two elevons\n"
        "• no independent yaw effector is claimed; aero yaw residual remains visible\n"
        "• no manufacturer or flight-test controller fidelity is claimed"
    )
    draw.multiline_text((x0 + 30, y0 + 105), fill(map_text, 70), fill="#334155", font=body_font, spacing=10)
    canvas.save(output, quality=95)

    manifest = {
        "schema_version": 1,
        "vehicle": "skywalker_x8",
        "composite": output.name,
        "packets": {
            "nominal_airborne_objectives": figure_eight_id,
            "direct_moment_racetrack_baseline": direct_id,
            "physical_elevon_candidate": physical_id,
        },
        "claims": {
            "nominal_airborne_objectives": figure_eight.get("claim"),
            "direct_moment_racetrack_baseline": direct.get("claim"),
            "physical_elevon_candidate": (
                physical.get("claim")
                if physical.get("mission_pass")
                else "blocked at the declared aerodynamic table boundary; no qualification claim"
            ),
        },
        "realizations": {
            "nominal_airborne_objectives": figure_eight.get("realization", {}),
            "direct_moment_racetrack_baseline": direct.get("realization", {}),
            "physical_elevon_candidate": physical.get("realization", {}),
        },
        "note": "A direct-moment baseline is retained for comparison and cannot certify the physical elevon-allocation candidate.",
    }
    (root / "x8-showcase-composite.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return output
####


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    arguments = parser.parse_args()
    print(build(arguments.root.resolve()))
    ####


if __name__ == "__main__":
    main()
