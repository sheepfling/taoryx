"""Evidence-board composites for the HL-20 booster-release showcase.

The renderer consumes the four replayable reachability envelopes directly.  It
does not turn the synthetic launch parent or the CA-HI comparison contract into
a range, controller, or source-trajectory claim.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .reachability_envelope import (
    DetachedBodyTrajectory,
    EnvelopeSample,
    EnvelopeTermination,
    Pseudo6DofState,
    ReachabilityEnvelope,
    ReachabilityFidelity,
)

_BACKGROUND = "#0b1220"
_PANEL = "#142235"
_TEXT = "#f1f5f9"
_MUTED = "#a7b8ca"
_GRID = "#3c526a"
_BOOST = "#f59e0b"
_COAST = "#94a3b8"
_GLIDE = "#2dd4bf"
_CHILD = "#fb7185"
_SUCCESS = "#4ade80"
_WARNING = "#fbbf24"
_FAILURE = "#fb7185"
_TIER_COLORS = ("#60a5fa", "#c084fc", "#f97316", "#2dd4bf")

_EXPECTED_FIDELITIES = (
    ReachabilityFidelity.POINT_MASS_3DOF,
    ReachabilityFidelity.PSEUDO_6DOF,
    ReachabilityFidelity.RIGID_BODY_6DOF,
    ReachabilityFidelity.RIGID_BODY_6DOF_SURFACE_ALLOCATED,
)

_TIER_LABELS = {
    ReachabilityFidelity.POINT_MASS_3DOF: "Tier 0 | 3DOF translation",
    ReachabilityFidelity.PSEUDO_6DOF: "Tier 1 | pseudo-6DOF bridge",
    ReachabilityFidelity.RIGID_BODY_6DOF: "Tier 2 | native rigid 6DOF",
    ReachabilityFidelity.RIGID_BODY_6DOF_SURFACE_ALLOCATED: "Tier 3 | rigid 6DOF + logical surfaces",
}

_TIER_CONTRACTS = {
    ReachabilityFidelity.POINT_MASS_3DOF: "Translation, mass, and phase baseline. Rotation and effectors are unavailable.",
    ReachabilityFidelity.PSEUDO_6DOF: "Translation plus filtered attitude/rate response. Not native rigid-body rotation.",
    ReachabilityFidelity.RIGID_BODY_6DOF: "Native rigid state with reduced direct-load observables. Open-loop only.",
    ReachabilityFidelity.RIGID_BODY_6DOF_SURFACE_ALLOCATED: "Native rigid state plus bounded logical seven-surface telemetry overlay.",
}


@dataclass(frozen=True, slots=True)
class HL20ShowcaseCompositeReport:
    """Replayable files emitted for one four-tier HL-20 showcase."""

    composite_paths: tuple[Path, ...]
    data_paths: tuple[Path, ...]
    summary_path: Path
    manifest_path: Path
    ####


def render_hl20_ca_hi_showcase_composites(
    envelopes: Sequence[ReachabilityEnvelope],
    directory: str | Path,
    *,
    source_artifacts: Sequence[str | Path] = (),
    dpi: int = 160,
) -> HL20ShowcaseCompositeReport:
    """Render the all-tier flight, envelope, and spent-stage evidence boards.

    ``source_artifacts`` should point to the serialized envelope inputs when
    the caller has written them.  The output manifest records their hashes so
    reviewers can identify exactly which witness set produced the composites.
    """

    if dpi <= 0:
        raise ValueError("showcase composite dpi must be positive")
    ordered = tuple(envelopes)
    if tuple(envelope.fidelity for envelope in ordered) != _EXPECTED_FIDELITIES:
        values = ", ".join(envelope.fidelity.value for envelope in ordered)
        raise ValueError(f"HL-20 showcase requires the ordered four-tier ladder; got {values}")
    if any(not envelope.samples for envelope in ordered):
        raise ValueError("HL-20 showcase requires at least one sample in every fidelity tier")

    destination = Path(directory)
    destination.mkdir(parents=True, exist_ok=True)
    summaries = tuple(_tier_summary(envelope) for envelope in ordered)
    candidate_rows = tuple(
        row
        for envelope in ordered
        for row in _candidate_rows(envelope)
    )
    parent_rows = tuple(
        row
        for envelope in ordered
        for row in _parent_rows(envelope)
    )
    child_rows = tuple(
        row
        for envelope in ordered
        for row in _child_rows(envelope)
    )
    surface_rows = tuple(
        row
        for envelope in ordered
        for row in _surface_rows(envelope)
    )

    data_paths = (
        _write_csv(destination / "candidate-summary.csv", candidate_rows),
        _write_csv(destination / "parent-telemetry-all-tiers.csv", parent_rows),
        _write_csv(destination / "spent-booster-telemetry.csv", child_rows),
        _write_csv(destination / "surface-allocation.csv", surface_rows),
    )

    plots = (
        _render_flight_composite(ordered, summaries, destination / "hl20-ca-hi-flight-composite.png", dpi=dpi),
        _render_envelope_comparison(ordered, summaries, destination / "hl20-ca-hi-envelope-comparison.png", dpi=dpi),
        _render_spent_booster_composite(ordered, summaries, destination / "hl20-ca-hi-spent-booster-composite.png", dpi=dpi),
    )

    summary = {
        "schema": "taoryx.hl20-ca-hi-showcase-summary/v1alpha1",
        "title": "HL-20 CA-HI four-tier boost, glide, and passive-cylinder witness",
        "scene": {
            "id": "hl20_california_to_hawaii_comparison",
            "route_contract": "synthetic California-to-Hawaii comparison geometry",
            "runtime_boundary": "local synthetic booster-release witness; not a route, arrival, controller, or source-trajectory claim",
        },
        "phase_schedule": _phase_schedule(ordered[0]),
        "search_space": ordered[0].search_space.as_dict() if ordered[0].search_space is not None else {},
        "execution": {
            "step_size_s": ordered[0].step_size_s,
            "horizon_s": ordered[0].horizon_s,
            "candidate_count_per_tier": len(ordered[0].samples),
            "all_tiers_share_search_space": all(
                _search_signature(envelope) == _search_signature(ordered[0])
                for envelope in ordered[1:]
            ),
        },
        "tiers": list(summaries),
        "cross_fidelity": _cross_fidelity_summary(ordered),
        "claim_boundary": {
            "synthetic_booster": True,
            "source_bound_hl20_geometry_and_fixed_mass": True,
            "source_exact_route": False,
            "controller_claim": False,
            "arrival_or_landing_claim": False,
            "source_aerodynamics": all(
                dict(envelope.provenance).get("aerodynamic_model") == "hl20_mod_k_daveml_source_v1"
                for envelope in ordered
            ),
            "actuator_profiles": sorted(
                {
                    str(dict(envelope.provenance).get("actuator_profile_id", "none"))
                    for envelope in ordered
                }
            ),
            "surface_controls": "Requested and achieved source surface channels are recorded when the source model is selected; this is not a closed-loop authority claim.",
            "spent_stage": "Synthetic passive aero-ballistic cylinder. Native tumble is only represented in native rigid tiers.",
        },
        "data_artifacts": [path.name for path in data_paths],
        "composites": [path.name for path in plots],
    }
    summary_path = destination / "showcase-summary.json"
    _write_json(summary_path, summary)

    manifest = {
        "schema": "taoryx.hl20-ca-hi-showcase-composite-manifest/v1alpha1",
        "summary": _file_record(summary_path),
        "source_artifacts": [_file_record(Path(source), relative_to=destination) for source in source_artifacts],
        "composites": [_file_record(path) for path in plots],
        "data_artifacts": [_file_record(path) for path in data_paths],
        "input_configuration_hashes": {
            envelope.fidelity.value: _envelope_configuration_hash(envelope)
            for envelope in ordered
        },
        "rendering": {"dpi": dpi, "renderer": "matplotlib", "deterministic_layout": True},
    }
    manifest_path = destination / "showcase-manifest.json"
    _write_json(manifest_path, manifest)
    return HL20ShowcaseCompositeReport(plots, data_paths, summary_path, manifest_path)
    ####


def _render_flight_composite(
    envelopes: Sequence[ReachabilityEnvelope],
    summaries: Sequence[dict[str, object]],
    path: Path,
    *,
    dpi: int,
) -> Path:
    """Render one row per tier for geometry, timeline, controls, and objectives."""

    plt = _get_matplotlib()
    figure = plt.figure(figsize=(24.0, 18.5), facecolor=_BACKGROUND, layout="constrained")
    grid = figure.add_gridspec(5, 4, height_ratios=(0.34, 1.0, 1.0, 1.0, 1.0), width_ratios=(1.15, 1.22, 1.16, 1.04))
    header = figure.add_subplot(grid[0, :])
    _draw_header(
        header,
        "HL-20 CA-HI | Four-tier boost / coast / glide evidence board",
        _flight_subtitle(envelopes[0]),
    )
    try:
        for index, (envelope, summary) in enumerate(zip(envelopes, summaries, strict=True)):
            geometry = figure.add_subplot(grid[index + 1, 0])
            timeline = figure.add_subplot(grid[index + 1, 1])
            controls = figure.add_subplot(grid[index + 1, 2])
            objective = figure.add_subplot(grid[index + 1, 3])
            _plot_parent_geometry(geometry, envelope, show_legend=index == 0)
            _plot_flight_timeline(timeline, envelope, show_legend=index == 0)
            _plot_control_panel(controls, envelope)
            _draw_tier_card(objective, envelope, summary)
            _tier_row_label(geometry, envelope, index)
        figure.text(
            0.012,
            0.012,
            "Claim boundary: synthetic booster; source-bound HL-20 geometry/fixed mass; no CA-HI arrival, controller, landing, or source-trajectory claim.",
            color=_MUTED,
            fontsize=9.5,
            ha="left",
        )
        return _save_figure(figure, path, dpi)
    finally:
        plt.close(figure)
    ####


def _render_envelope_comparison(
    envelopes: Sequence[ReachabilityEnvelope],
    summaries: Sequence[dict[str, object]],
    path: Path,
    *,
    dpi: int,
) -> Path:
    """Render cross-fidelity search coverage and terminal/capability comparison."""

    plt = _get_matplotlib()
    figure = plt.figure(figsize=(21.0, 12.0), facecolor=_BACKGROUND, layout="constrained")
    grid = figure.add_gridspec(3, 3, height_ratios=(0.42, 1.0, 1.0))
    header = figure.add_subplot(grid[0, :])
    _draw_header(
        header,
        "HL-20 CA-HI | Search-space and capability comparison",
        "Same launch grid in every tier. Terminal disposition remains explicit; a horizon-limited witness is not promoted to arrival success.",
    )
    axes = (
        figure.add_subplot(grid[1, 0]),
        figure.add_subplot(grid[1, 1]),
        figure.add_subplot(grid[1, 2]),
        figure.add_subplot(grid[2, 0]),
        figure.add_subplot(grid[2, 1]),
        figure.add_subplot(grid[2, 2]),
    )
    try:
        _plot_terminal_cloud(axes[0], envelopes)
        _plot_max_altitude(axes[1], envelopes)
        _plot_terminal_speed(axes[2], envelopes)
        _plot_search_matrix(axes[3], envelopes)
        _plot_stage_objective_matrix(axes[4], summaries)
        _plot_fidelity_delta(axes[5], envelopes)
        return _save_figure(figure, path, dpi)
    finally:
        plt.close(figure)
    ####


def _render_spent_booster_composite(
    envelopes: Sequence[ReachabilityEnvelope],
    summaries: Sequence[dict[str, object]],
    path: Path,
    *,
    dpi: int,
) -> Path:
    """Render the passive-cylinder deployment and tumble evidence board."""

    plt = _get_matplotlib()
    figure = plt.figure(figsize=(24.0, 18.5), facecolor=_BACKGROUND, layout="constrained")
    grid = figure.add_gridspec(5, 4, height_ratios=(0.34, 1.0, 1.0, 1.0, 1.0), width_ratios=(1.16, 1.22, 1.16, 1.03))
    header = figure.add_subplot(grid[0, :])
    _draw_header(
        header,
        "HL-20 CA-HI | Spent booster passive aero-ballistic cylinder",
        "One cylinder per accepted release. The board separates center-of-mass reductions from pseudo attitude and native rigid-body tumble evidence.",
    )
    try:
        for index, (envelope, summary) in enumerate(zip(envelopes, summaries, strict=True)):
            geometry = figure.add_subplot(grid[index + 1, 0])
            loads = figure.add_subplot(grid[index + 1, 1])
            attitude = figure.add_subplot(grid[index + 1, 2])
            lineage = figure.add_subplot(grid[index + 1, 3])
            _plot_stage_geometry(geometry, envelope, show_legend=index == 0)
            _plot_stage_loads(loads, envelope, show_legend=index == 0)
            _plot_tumble_panel(attitude, envelope)
            _draw_stage_card(lineage, envelope, summary)
            _tier_row_label(geometry, envelope, index)
        figure.text(
            0.012,
            0.012,
            "Cylinder contract: 1,500 kg, radius 0.8 m, length 6.0 m, passive-tumble policy. It is a synthetic detached body, not a source booster reconstruction.",
            color=_MUTED,
            fontsize=9.5,
            ha="left",
        )
        return _save_figure(figure, path, dpi)
    finally:
        plt.close(figure)
    ####


def _draw_header(axis: Any, title: str, subtitle: str) -> None:
    axis.set_facecolor(_BACKGROUND)
    axis.axis("off")
    axis.text(0.0, 0.77, title, color=_TEXT, fontsize=22, fontweight="bold", ha="left", va="center", transform=axis.transAxes)
    axis.text(0.0, 0.24, subtitle, color=_MUTED, fontsize=10.5, ha="left", va="center", transform=axis.transAxes)
    ####


def _flight_subtitle(envelope: ReachabilityEnvelope) -> str:
    schedule = _phase_schedule(envelope)
    search = _search_description(envelope)
    return (
        f"{search}. Synthetic booster: boost 0-{schedule['burnout_time_s']:.1f}s, coast to {schedule['release_time_s']:.1f}s, "
        "then unpowered HL-20 glide. Each row uses the same retained candidate set."
    )
    ####


def _plot_parent_geometry(axis: Any, envelope: ReachabilityEnvelope, *, show_legend: bool) -> None:
    _style_axis(axis)
    nominal = _nominal_sample(envelope)
    for sample in envelope.samples:
        _plot_phase_path(
            axis,
            sample.trajectory.states,
            horizontal_index=0,
            vertical_index=2,
            linewidth=2.4 if sample is nominal else 0.85,
            alpha=1.0 if sample is nominal else 0.36,
        )
    for label, time_s, color in _stage_markers(envelope):
        state = _state_at_time(nominal, time_s)
        if state is not None:
            axis.scatter(state.position_m[0] / 1_000.0, state.position_m[2] / 1_000.0, marker="o", s=28, color=color, edgecolor=_BACKGROUND, linewidth=0.8, zorder=6)
            axis.annotate(label, (state.position_m[0] / 1_000.0, state.position_m[2] / 1_000.0), xytext=(4, 6), textcoords="offset points", fontsize=7.5, color=_MUTED)
    terminal = nominal.terminal
    axis.scatter(terminal.position_m[0] / 1_000.0, terminal.position_m[2] / 1_000.0, marker="x", s=42, color=_TEXT, zorder=7)
    axis.set_title("Parent path: downrange / altitude", loc="left", color=_TEXT, fontsize=10.5, fontweight="bold")
    axis.set_xlabel("downrange (km)", color=_MUTED, fontsize=8.5)
    axis.set_ylabel("altitude (km)", color=_MUTED, fontsize=8.5)
    if show_legend:
        _phase_legend(axis, child=False)
    ####


def _plot_flight_timeline(axis: Any, envelope: ReachabilityEnvelope, *, show_legend: bool) -> None:
    _style_axis(axis)
    nominal = _nominal_sample(envelope)
    for sample in envelope.samples:
        alpha = 0.34 if sample is not nominal else 1.0
        width = 0.9 if sample is not nominal else 2.1
        axis.plot(_times(sample), [state.position_m[2] / 1_000.0 for state in sample.trajectory.states], color="#60a5fa", linewidth=width, alpha=alpha)
    speed_axis = axis.twinx()
    _style_twin_axis(speed_axis)
    for sample in envelope.samples:
        alpha = 0.25 if sample is not nominal else 0.95
        width = 0.8 if sample is not nominal else 1.8
        speed_axis.plot(_times(sample), [state.speed_m_s for state in sample.trajectory.states], color="#f472b6", linewidth=width, linestyle="--", alpha=alpha)
    _phase_bands(axis, envelope)
    _stage_lines(axis, envelope, labels=True)
    axis.set_title("Boost / glide timeline", loc="left", color=_TEXT, fontsize=10.5, fontweight="bold")
    axis.set_xlabel("time (s)", color=_MUTED, fontsize=8.5)
    axis.set_ylabel("altitude (km)", color="#93c5fd", fontsize=8.5)
    speed_axis.set_ylabel("speed (m/s)", color="#f9a8d4", fontsize=8.5)
    if show_legend:
        from matplotlib.lines import Line2D

        axis.legend(
            handles=(
                Line2D([0], [0], color="#60a5fa", linewidth=2, label="altitude"),
                Line2D([0], [0], color="#f472b6", linewidth=2, linestyle="--", label="speed"),
            ),
            loc="upper left",
            fontsize=7.5,
            facecolor=_PANEL,
            edgecolor=_GRID,
            labelcolor=_TEXT,
        )
    ####


def _plot_control_panel(axis: Any, envelope: ReachabilityEnvelope) -> None:
    fidelity = envelope.fidelity
    nominal = _nominal_sample(envelope)
    if fidelity is ReachabilityFidelity.POINT_MASS_3DOF:
        _style_axis(axis)
        axis.axis("off")
        command = nominal.command
        _card_text(
            axis,
            "Control / attitude contract",
            (
                ("Launch elevation", f"{math.degrees(command.elevation_rad):.1f} deg"),
                ("Launch azimuth", f"{math.degrees(command.azimuth_rad):.1f} deg"),
                ("Glide bank", f"{math.degrees(command.bank_rad):.1f} deg"),
                ("Attitude / rates", "unavailable by 3DOF contract"),
                ("Effectors", "unavailable by 3DOF contract"),
                ("Command mode", "open-loop launch direction + lift-vector bank"),
            ),
        )
        return
    if fidelity is ReachabilityFidelity.PSEUDO_6DOF:
        _plot_pseudo_attitude(axis, nominal, envelope)
        return
    if fidelity is ReachabilityFidelity.RIGID_BODY_6DOF:
        _plot_native_loads(axis, nominal, envelope)
        return
    _plot_surface_allocation(axis, nominal, envelope)
    ####


def _plot_pseudo_attitude(axis: Any, sample: EnvelopeSample, envelope: ReachabilityEnvelope) -> None:
    _style_axis(axis)
    states = [state for state in sample.trajectory.states if isinstance(state, Pseudo6DofState)]
    if not states:
        _unavailable(axis, "Pseudo-6DOF attitude rows are unavailable")
        return
    times = [state.time_s for state in states]
    labels = ("roll", "pitch", "yaw")
    colors = ("#60a5fa", "#f59e0b", "#c084fc")
    for component, (label, color) in enumerate(zip(labels, colors, strict=True)):
        axis.plot(times, [math.degrees(state.attitude_rad[component]) for state in states], label=label, color=color, linewidth=1.8)
    rate_axis = axis.twinx()
    _style_twin_axis(rate_axis)
    rate_axis.plot(times, [_rate_magnitude(state.attitude_rate_rad_s) for state in states], color="#f472b6", linestyle="--", linewidth=1.5, label="rate magnitude")
    _phase_bands(axis, envelope)
    _stage_lines(axis, envelope, labels=False)
    axis.set_title("Pseudo attitude / rate bridge", loc="left", color=_TEXT, fontsize=10.5, fontweight="bold")
    axis.set_xlabel("time (s)", color=_MUTED, fontsize=8.5)
    axis.set_ylabel("attitude (deg)", color=_MUTED, fontsize=8.5)
    rate_axis.set_ylabel("rate magnitude (rad/s)", color="#f9a8d4", fontsize=8.5)
    axis.legend(loc="upper left", fontsize=7.5, facecolor=_PANEL, edgecolor=_GRID, labelcolor=_TEXT)
    ####


def _plot_native_loads(axis: Any, sample: EnvelopeSample, envelope: ReachabilityEnvelope) -> None:
    _style_axis(axis)
    rows = tuple(sample.trajectory.telemetry)
    if not rows or "total_force_body_x_n" not in rows[0]:
        _unavailable(axis, "Native rigid-body load observables are unavailable")
        return
    times = _telemetry_times(rows)
    axis.plot(times, [_number(row, "propulsion_force_body_x_n") / 1_000.0 for row in rows], color=_BOOST, label="propulsion Fx", linewidth=1.9)
    axis.plot(times, [_number(row, "aero_force_body_x_n") / 1_000.0 for row in rows], color="#60a5fa", label="aero Fx", linewidth=1.55)
    axis.plot(times, [_number(row, "aero_force_body_z_n") / 1_000.0 for row in rows], color=_GLIDE, label="aero Fz", linewidth=1.55)
    rate_axis = axis.twinx()
    _style_twin_axis(rate_axis)
    rate_axis.plot(times, [_row_rate_magnitude(row) for row in rows], color="#f472b6", linestyle="--", linewidth=1.45, label="body rate")
    _phase_bands(axis, envelope)
    _stage_lines(axis, envelope, labels=False)
    axis.set_title("Native loads / body-rate observables", loc="left", color=_TEXT, fontsize=10.5, fontweight="bold")
    axis.set_xlabel("time (s)", color=_MUTED, fontsize=8.5)
    axis.set_ylabel("force (kN)", color=_MUTED, fontsize=8.5)
    rate_axis.set_ylabel("body rate (rad/s)", color="#f9a8d4", fontsize=8.5)
    axis.legend(loc="upper left", fontsize=7.0, facecolor=_PANEL, edgecolor=_GRID, labelcolor=_TEXT)
    ####


def _plot_surface_allocation(axis: Any, sample: EnvelopeSample, envelope: ReachabilityEnvelope) -> None:
    _style_axis(axis)
    rows = tuple(sample.trajectory.telemetry)
    allocation = next((row for row in rows if "surface_allocation_active" in row), None)
    if allocation is None:
        _unavailable(axis, "Logical surface-allocation telemetry is unavailable")
        return
    channels = (
        ("UL body", "upper_left_body_flap_deg"),
        ("LL body", "lower_left_body_flap_deg"),
        ("UR body", "upper_right_body_flap_deg"),
        ("LR body", "lower_right_body_flap_deg"),
        ("L wing", "left_wing_flap_deg"),
        ("R wing", "right_wing_flap_deg"),
        ("rudder", "rudder_deg"),
    )
    values = [_number(allocation, field) for _, field in channels]
    colors = ("#60a5fa", "#60a5fa", "#2dd4bf", "#2dd4bf", "#f59e0b", "#f59e0b", "#c084fc")
    y_positions = list(range(len(channels)))
    axis.barh(y_positions, values, color=colors, alpha=0.9)
    axis.axvline(0.0, color=_MUTED, linewidth=0.8)
    axis.set_yticks(y_positions, [label for label, _ in channels], color=_MUTED, fontsize=7.5)
    axis.invert_yaxis()
    axis.set_xlabel("logical deflection (deg)", color=_MUTED, fontsize=8.5)
    axis.set_title("Tier 3 logical surface allocation", loc="left", color=_TEXT, fontsize=10.5, fontweight="bold")
    request = (
        _number(allocation, "surface_allocation_requested_bank_deg"),
        _number(allocation, "surface_allocation_requested_pitch_deg"),
        _number(allocation, "surface_allocation_requested_yaw_deg"),
    )
    achieved = (
        _number(allocation, "surface_allocation_achieved_bank_deg"),
        _number(allocation, "surface_allocation_achieved_pitch_deg"),
        _number(allocation, "surface_allocation_achieved_yaw_deg"),
    )
    axis.text(
        0.02,
        0.03,
        "request b/p/y: " + "/".join(f"{value:.1f}" for value in request) + " deg\n"
        + "achieved b/p/y: " + "/".join(f"{value:.1f}" for value in achieved) + " deg\n"
        + f"residual: {_number(allocation, 'surface_allocation_residual_deg'):.3g} deg | saturated: {'yes' if _number(allocation, 'surface_allocation_saturated') else 'no'}\n"
        + "Overlay only: not source actuator dynamics or controller qualification.",
        color=_MUTED,
        fontsize=7.2,
        va="bottom",
        ha="left",
        transform=axis.transAxes,
        bbox={"facecolor": _BACKGROUND, "edgecolor": _GRID, "alpha": 0.9, "pad": 3.0},
    )
    ####


def _draw_tier_card(axis: Any, envelope: ReachabilityEnvelope, summary: Mapping[str, object]) -> None:
    _style_axis(axis)
    axis.axis("off")
    candidates = summary["candidates"]
    objectives = summary["phase_objectives"]
    assert isinstance(candidates, Mapping)
    assert isinstance(objectives, list)
    lines: list[tuple[str, str]] = [
        ("Fidelity contract", _TIER_CONTRACTS[envelope.fidelity]),
        ("Search", f"{len(envelope.samples)} candidates; " + _classification_description(candidates)),
        ("Terminal", _termination_description(candidates)),
        ("Mass schedule", _mass_schedule_description(envelope)),
    ]
    for objective in objectives:
        if not isinstance(objective, Mapping):
            continue
        lines.append((str(objective["id"]), f"{objective['status']}: {objective['detail']}"))
    _card_text(axis, "Stage objectives / terminal disposition", tuple(lines))
    ####


def _plot_terminal_cloud(axis: Any, envelopes: Sequence[ReachabilityEnvelope]) -> None:
    _style_axis(axis)
    for index, envelope in enumerate(envelopes):
        color = _TIER_COLORS[index]
        x_values = [sample.terminal.position_m[0] / 1_000.0 for sample in envelope.samples]
        z_values = [sample.terminal.position_m[2] / 1_000.0 for sample in envelope.samples]
        axis.scatter(x_values, z_values, color=color, s=54, edgecolor=_BACKGROUND, linewidth=0.7, alpha=0.92, label=_short_fidelity_label(envelope.fidelity))
    axis.set_title("Terminal altitude capability", loc="left", color=_TEXT, fontsize=10.5, fontweight="bold")
    axis.set_xlabel("terminal downrange (km)", color=_MUTED, fontsize=8.5)
    axis.set_ylabel("terminal altitude (km)", color=_MUTED, fontsize=8.5)
    axis.legend(loc="best", fontsize=7.2, facecolor=_PANEL, edgecolor=_GRID, labelcolor=_TEXT)
    ####


def _plot_max_altitude(axis: Any, envelopes: Sequence[ReachabilityEnvelope]) -> None:
    _style_axis(axis)
    for index, envelope in enumerate(envelopes):
        points = sorted(
            (
                math.degrees(sample.command.elevation_rad),
                max(state.position_m[2] for state in sample.trajectory.states) / 1_000.0,
            )
            for sample in envelope.samples
        )
        axis.plot([point[0] for point in points], [point[1] for point in points], marker="o", markersize=4.5, color=_TIER_COLORS[index], linewidth=1.7, label=_short_fidelity_label(envelope.fidelity))
    axis.set_title("Maximum altitude over launch elevation", loc="left", color=_TEXT, fontsize=10.5, fontweight="bold")
    axis.set_xlabel("launch elevation (deg)", color=_MUTED, fontsize=8.5)
    axis.set_ylabel("max altitude (km)", color=_MUTED, fontsize=8.5)
    axis.legend(loc="best", fontsize=7.2, facecolor=_PANEL, edgecolor=_GRID, labelcolor=_TEXT)
    ####


def _plot_terminal_speed(axis: Any, envelopes: Sequence[ReachabilityEnvelope]) -> None:
    _style_axis(axis)
    for index, envelope in enumerate(envelopes):
        points = sorted(
            (math.degrees(sample.command.elevation_rad), sample.terminal.speed_m_s)
            for sample in envelope.samples
        )
        axis.plot([point[0] for point in points], [point[1] for point in points], marker="o", markersize=4.5, color=_TIER_COLORS[index], linewidth=1.7, label=_short_fidelity_label(envelope.fidelity))
    axis.set_title("Terminal speed over launch elevation", loc="left", color=_TEXT, fontsize=10.5, fontweight="bold")
    axis.set_xlabel("launch elevation (deg)", color=_MUTED, fontsize=8.5)
    axis.set_ylabel("terminal speed (m/s)", color=_MUTED, fontsize=8.5)
    axis.legend(loc="best", fontsize=7.2, facecolor=_PANEL, edgecolor=_GRID, labelcolor=_TEXT)
    ####


def _plot_search_matrix(axis: Any, envelopes: Sequence[ReachabilityEnvelope]) -> None:
    _style_axis(axis)
    elevations = sorted({math.degrees(sample.command.elevation_rad) for envelope in envelopes for sample in envelope.samples})
    status_values = {"feasible": 1.0, "infeasible": 0.45, "invalid": 0.0}
    for row, envelope in enumerate(envelopes):
        by_elevation = {math.degrees(sample.command.elevation_rad): sample for sample in envelope.samples}
        for column, elevation in enumerate(elevations):
            sample = by_elevation.get(elevation)
            value = status_values.get(sample.classification if sample is not None else "invalid", 0.0)
            color = _SUCCESS if value == 1.0 else _WARNING if value > 0.0 else _FAILURE
            axis.scatter(column, row, s=390, marker="s", color=color, edgecolor=_BACKGROUND, linewidth=1.0)
            if sample is not None:
                symbol = "F" if sample.classification == "feasible" else "I" if sample.classification == "infeasible" else "X"
                axis.text(column, row, symbol, color=_BACKGROUND, fontsize=9, fontweight="bold", ha="center", va="center")
    axis.set_title("Realized search / classification", loc="left", color=_TEXT, fontsize=10.5, fontweight="bold")
    axis.set_xticks(range(len(elevations)), [f"{value:.0f}" for value in elevations], color=_MUTED, fontsize=8)
    axis.set_yticks(range(len(envelopes)), [_short_fidelity_label(envelope.fidelity) for envelope in envelopes], color=_MUTED, fontsize=7.5)
    axis.set_xlabel("launch elevation (deg)", color=_MUTED, fontsize=8.5)
    axis.set_xlim(-0.6, len(elevations) - 0.4)
    axis.set_ylim(len(envelopes) - 0.5, -0.5)
    axis.text(0.02, 0.02, "F feasible | I infeasible | X invalid", color=_MUTED, fontsize=7.5, transform=axis.transAxes)
    ####


def _plot_stage_objective_matrix(axis: Any, summaries: Sequence[dict[str, object]]) -> None:
    _style_axis(axis)
    objective_ids = ("boost", "burnout", "release", "glide", "spent_cylinder", "terminal")
    color_by_status = {"achieved": _SUCCESS, "partial": _WARNING, "not_observed": _FAILURE, "horizon_limited": _WARNING, "impact_observed": _SUCCESS, "invalid": _FAILURE}
    for row, summary in enumerate(summaries):
        raw_objectives = summary.get("phase_objectives", ())
        objectives = raw_objectives if isinstance(raw_objectives, (list, tuple)) else ()
        values = {str(item["id"]): str(item["status"]) for item in objectives if isinstance(item, Mapping)}
        for column, objective_id in enumerate(objective_ids):
            status = values.get(objective_id, "not_observed")
            axis.scatter(column, row, s=390, marker="s", color=color_by_status.get(status, _WARNING), edgecolor=_BACKGROUND, linewidth=1.0)
            axis.text(column, row, "OK" if status in {"achieved", "impact_observed"} else "~" if status in {"partial", "horizon_limited"} else "-", color=_BACKGROUND, fontsize=8, fontweight="bold", ha="center", va="center")
    axis.set_title("Stage objectives by fidelity", loc="left", color=_TEXT, fontsize=10.5, fontweight="bold")
    axis.set_xticks(range(len(objective_ids)), [label.replace("_", "\n") for label in objective_ids], color=_MUTED, fontsize=7.5)
    axis.set_yticks(range(len(summaries)), [_short_fidelity_label(ReachabilityFidelity(str(summary["fidelity"]))) for summary in summaries], color=_MUTED, fontsize=7.5)
    axis.set_xlim(-0.6, len(objective_ids) - 0.4)
    axis.set_ylim(len(summaries) - 0.5, -0.5)
    axis.text(0.02, 0.02, "OK achieved/impact | ~ partial/horizon-limited | - not observed", color=_MUTED, fontsize=7.2, transform=axis.transAxes)
    ####


def _plot_fidelity_delta(axis: Any, envelopes: Sequence[ReachabilityEnvelope]) -> None:
    _style_axis(axis)
    reference = envelopes[0]
    elevations = sorted({math.degrees(sample.command.elevation_rad) for sample in reference.samples})
    reference_by_elevation = {math.degrees(sample.command.elevation_rad): sample for sample in reference.samples}
    for index, envelope in enumerate(envelopes[1:], start=1):
        values: list[float] = []
        for elevation in elevations:
            sample = next(sample for sample in envelope.samples if math.isclose(math.degrees(sample.command.elevation_rad), elevation, abs_tol=1.0e-9))
            baseline = reference_by_elevation[elevation]
            values.append((sample.terminal.position_m[0] - baseline.terminal.position_m[0]) / 1_000.0)
        axis.plot(elevations, values, marker="o", markersize=4.5, color=_TIER_COLORS[index], linewidth=1.7, label=_short_fidelity_label(envelope.fidelity))
    axis.axhline(0.0, color=_MUTED, linewidth=0.9)
    axis.set_title("Terminal downrange delta vs 3DOF", loc="left", color=_TEXT, fontsize=10.5, fontweight="bold")
    axis.set_xlabel("launch elevation (deg)", color=_MUTED, fontsize=8.5)
    axis.set_ylabel("delta downrange (km)", color=_MUTED, fontsize=8.5)
    axis.legend(loc="best", fontsize=7.2, facecolor=_PANEL, edgecolor=_GRID, labelcolor=_TEXT)
    ####


def _plot_stage_geometry(axis: Any, envelope: ReachabilityEnvelope, *, show_legend: bool) -> None:
    _style_axis(axis)
    sample = _nominal_sample(envelope)
    _plot_phase_path(axis, sample.trajectory.states, horizontal_index=0, vertical_index=2, linewidth=2.1, alpha=0.95)
    child = _nominal_child(sample)
    if child is not None:
        states = child.states
        axis.plot(
            [state.position_m[0] / 1_000.0 for state in states],
            [state.position_m[2] / 1_000.0 for state in states],
            color=_CHILD,
            linewidth=1.8,
            linestyle="--",
            label="spent cylinder",
        )
        axis.scatter(states[0].position_m[0] / 1_000.0, states[0].position_m[2] / 1_000.0, color=_CHILD, marker="o", s=32, zorder=5)
        axis.scatter(states[-1].position_m[0] / 1_000.0, states[-1].position_m[2] / 1_000.0, color=_CHILD, marker="x", s=42, zorder=5)
    else:
        _unavailable(axis, "No accepted deployment before the configured horizon")
        return
    axis.set_title("Parent / cylinder separation geometry", loc="left", color=_TEXT, fontsize=10.5, fontweight="bold")
    axis.set_xlabel("downrange (km)", color=_MUTED, fontsize=8.5)
    axis.set_ylabel("altitude (km)", color=_MUTED, fontsize=8.5)
    if show_legend:
        _phase_legend(axis, child=True)
    ####


def _plot_stage_loads(axis: Any, envelope: ReachabilityEnvelope, *, show_legend: bool) -> None:
    _style_axis(axis)
    child = _nominal_child(_nominal_sample(envelope))
    if child is None:
        _unavailable(axis, "No child telemetry before the configured horizon")
        return
    rows = tuple(child.telemetry)
    if not rows:
        _unavailable(axis, "Detached-body aero telemetry is unavailable")
        return
    times = _telemetry_times(rows)
    axis.plot(times, [_number(row, "projected_area_m2") for row in rows], color="#60a5fa", linewidth=1.8, label="projected area")
    drag_axis = axis.twinx()
    _style_twin_axis(drag_axis)
    drag_axis.plot(times, [_number(row, "drag_force_n") / 1_000.0 for row in rows], color=_CHILD, linewidth=1.7, linestyle="--", label="drag")
    axis.axvline(child.deployment_time_s, color=_WARNING, linewidth=1.0, linestyle=":")
    axis.set_title("Cylinder aero-ballistic history", loc="left", color=_TEXT, fontsize=10.5, fontweight="bold")
    axis.set_xlabel("time (s)", color=_MUTED, fontsize=8.5)
    axis.set_ylabel("projected area (m2)", color="#93c5fd", fontsize=8.5)
    drag_axis.set_ylabel("drag (kN)", color="#fda4af", fontsize=8.5)
    if show_legend:
        from matplotlib.lines import Line2D

        axis.legend(
            handles=(
                Line2D([0], [0], color="#60a5fa", linewidth=2, label="projected area"),
                Line2D([0], [0], color=_CHILD, linewidth=2, linestyle="--", label="drag"),
            ),
            loc="upper left",
            fontsize=7.5,
            facecolor=_PANEL,
            edgecolor=_GRID,
            labelcolor=_TEXT,
        )
    ####


def _plot_tumble_panel(axis: Any, envelope: ReachabilityEnvelope) -> None:
    _style_axis(axis)
    child = _nominal_child(_nominal_sample(envelope))
    if child is None:
        _unavailable(axis, "No child deployment before the configured horizon")
        return
    if envelope.fidelity is ReachabilityFidelity.POINT_MASS_3DOF:
        axis.axis("off")
        _card_text(
            axis,
            "Tumble representation",
            (
                ("State model", "center of mass only"),
                ("Passive cylinder policy", "retained in configuration"),
                ("Rotation evidence", "unavailable by 3DOF contract"),
                ("Projected area", "orientation-independent reduced treatment"),
            ),
        )
        return
    rows = tuple(child.telemetry)
    if not rows or not isinstance(rows[0].get("attitude_rate_rad_s"), list):
        _unavailable(axis, "Detached-body attitude telemetry is unavailable")
        return
    times = _telemetry_times(rows)
    rate_vectors = [row["attitude_rate_rad_s"] for row in rows]
    for component, (label, color) in enumerate(zip(("p", "q", "r"), ("#60a5fa", "#f59e0b", "#c084fc"), strict=True)):
        axis.plot(times, [float(vector[component]) for vector in rate_vectors if isinstance(vector, list) and len(vector) == 3], color=color, linewidth=1.55, label=label)
    magnitude_axis = axis.twinx()
    _style_twin_axis(magnitude_axis)
    magnitude_axis.plot(times, [_row_rate_magnitude(row) for row in rows], color=_CHILD, linewidth=1.45, linestyle="--", label="magnitude")
    title = "Reduced tumble attitude / rates" if envelope.fidelity is ReachabilityFidelity.PSEUDO_6DOF else "Native rigid-body tumble / rates"
    axis.set_title(title, loc="left", color=_TEXT, fontsize=10.5, fontweight="bold")
    axis.set_xlabel("time (s)", color=_MUTED, fontsize=8.5)
    axis.set_ylabel("body rate (rad/s)", color=_MUTED, fontsize=8.5)
    magnitude_axis.set_ylabel("rate magnitude (rad/s)", color="#fda4af", fontsize=8.5)
    axis.legend(loc="upper left", fontsize=7.5, facecolor=_PANEL, edgecolor=_GRID, labelcolor=_TEXT)
    ####


def _draw_stage_card(axis: Any, envelope: ReachabilityEnvelope, summary: Mapping[str, object]) -> None:
    _style_axis(axis)
    axis.axis("off")
    parameters = dict(envelope.vehicle_parameters)
    body = parameters.get("booster_detached_body")
    body_data = body if isinstance(body, Mapping) else {}
    sample = _nominal_sample(envelope)
    child = _nominal_child(sample)
    event = sample.trajectory.deployment_events[0] if sample.trajectory.deployment_events else {}
    raw_objectives = summary.get("phase_objectives", ())
    objectives = raw_objectives if isinstance(raw_objectives, (list, tuple)) else ()
    cylinder = next((item for item in objectives if isinstance(item, Mapping) and item.get("id") == "spent_cylinder"), {})
    terminal = "not deployed before horizon" if child is None else f"{child.classification} at {child.terminal.time_s:.1f} s"
    _card_text(
        axis,
        "Deployment / object-lineage contract",
        (
            ("Parent -> child", f"{event.get('parent_model_id', envelope.vehicle_id)} -> {body_data.get('body_id', 'unavailable')}"),
            ("Accepted release", f"{_finite_number(event.get('accepted_time_s')):.1f} s" if event else "not observed"),
            ("Child geometry", f"{body_data.get('shape', 'unavailable')}; {body_data.get('mass_kg', 'n/a')} kg; {body_data.get('dimensions_m', 'n/a')} m"),
            ("Attitude policy", str(body_data.get("tumbling_policy", "unavailable"))),
            ("Retained impulse", str(event.get("retained_impulse_n_s", "unavailable"))),
            ("Cylinder objective", f"{cylinder.get('status', 'not_observed')}: {cylinder.get('detail', '')}"),
            ("Child terminal", terminal),
        ),
    )
    ####


def _tier_summary(envelope: ReachabilityEnvelope) -> dict[str, object]:
    parameters = dict(envelope.vehicle_parameters)
    candidates = {
        "count": len(envelope.samples),
        "classification_counts": envelope.classification_counts,
        "termination_counts": _termination_counts(envelope),
        "timed_out_query_ids": list(envelope.timed_out_query_ids),
        "spawned_child_count": sum(len(sample.trajectory.spawned_bodies) for sample in envelope.samples),
        "child_classification_counts": envelope.child_classification_counts,
    }
    return {
        "fidelity": envelope.fidelity.value,
        "label": _TIER_LABELS[envelope.fidelity],
        "contract": _TIER_CONTRACTS[envelope.fidelity],
        "aerodynamic_model_id": parameters.get("aerodynamic_model_id"),
        "actuator_profile_id": parameters.get("actuator_profile_id"),
        "mission_profile_id": parameters.get("mission_profile_id"),
        "mission_segment_schedule": parameters.get("mission_segment_schedule", []),
        "state_fields": list(envelope.state_fields),
        "candidates": candidates,
        "phase_objectives": _phase_objectives(envelope),
        "mass_schedule": {
            "initial_mass_kg": _finite_number(parameters.get("dry_mass_kg"))
            + _finite_number(parameters.get("propellant_mass_kg"))
            + _finite_number(parameters.get("booster_dry_mass_kg"))
            + _finite_number(parameters.get("booster_propellant_mass_kg")),
            "burnout_mass_kg": _finite_number(parameters.get("dry_mass_kg"))
            + _finite_number(parameters.get("propellant_mass_kg"))
            + _finite_number(parameters.get("booster_dry_mass_kg")),
            "release_mass_kg": _finite_number(parameters.get("dry_mass_kg")) + _finite_number(parameters.get("propellant_mass_kg")),
        },
        "terminal_criteria": envelope.terminal_criteria.as_dict(),
    }
    ####


def _phase_objectives(envelope: ReachabilityEnvelope) -> list[dict[str, object]]:
    phases = [set(state.phase for state in sample.trajectory.states) for sample in envelope.samples]
    all_have = lambda phase: all(phase in values for values in phases)
    any_have = lambda phase: any(phase in values for values in phases)
    deployed = [sample for sample in envelope.samples if sample.trajectory.deployment_events and sample.trajectory.spawned_bodies]
    all_deployed = len(deployed) == len(envelope.samples)
    terminal_counts = _termination_counts(envelope)
    cylinder_detail = _cylinder_tumble_detail(envelope)
    terminal_status = (
        "invalid"
        if terminal_counts.get(EnvelopeTermination.INVALID.value, 0)
        else "impact_observed"
        if terminal_counts.get(EnvelopeTermination.GROUND_CONTACT.value, 0) == len(envelope.samples)
        else "horizon_limited"
        if terminal_counts.get(EnvelopeTermination.HORIZON.value, 0)
        else "partial"
    )
    return [
        _objective("boost", "achieved" if all_have("boost") else "partial" if any_have("boost") else "not_observed", "boost phase retained for every candidate"),
        _objective("burnout", "achieved" if all_have("coast") else "partial" if any_have("coast") else "not_observed", "coast phase begins at scheduled booster burnout"),
        _objective("release", "achieved" if all_deployed else "partial" if deployed else "not_observed", f"accepted booster-release events: {len(deployed)}/{len(envelope.samples)}"),
        _objective("glide", "achieved" if all_have("glide") else "partial" if any_have("glide") else "not_observed", "unpowered HL-20 glide phase after release"),
        _objective("spent_cylinder", cylinder_detail[0], cylinder_detail[1]),
        _objective("terminal", terminal_status, _terminal_objective_detail(terminal_counts)),
    ]
    ####


def _objective(identifier: str, status: str, detail: str) -> dict[str, object]:
    return {"id": identifier, "status": status, "detail": detail}
    ####


def _cylinder_tumble_detail(envelope: ReachabilityEnvelope) -> tuple[str, str]:
    children = [child for sample in envelope.samples for child in sample.trajectory.spawned_bodies]
    if not children:
        return "not_observed", "no accepted cylinder deployment before the configured horizon"
    if envelope.fidelity is ReachabilityFidelity.POINT_MASS_3DOF:
        return "achieved", "cylinder propagated as a center-of-mass passive aero-ballistic reduction; rotation unavailable"
    peak_rate = max((_peak_child_rate(child) for child in children), default=0.0)
    representation = "native rigid-body passive tumble" if envelope.fidelity in {ReachabilityFidelity.RIGID_BODY_6DOF, ReachabilityFidelity.RIGID_BODY_6DOF_SURFACE_ALLOCATED} else "pseudo-6DOF reduced tumble"
    return "achieved", f"{representation}; peak body-rate magnitude {peak_rate:.4g} rad/s"
    ####


def _terminal_objective_detail(counts: Mapping[str, int]) -> str:
    parts = [f"{count} {name}" for name, count in sorted(counts.items())]
    if EnvelopeTermination.HORIZON.value in counts:
        return "; ".join(parts) + "; horizon is retained as a classified outcome, not target-arrival success"
    return "; ".join(parts)
    ####


def _candidate_rows(envelope: ReachabilityEnvelope) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for sample in envelope.samples:
        metrics = dict(sample.path_metrics)
        child = _nominal_child(sample)
        rows.append(
            {
                "fidelity": envelope.fidelity.value,
                "query_id": sample.query_id,
                "candidate_index": sample.index,
                "azimuth_deg": math.degrees(sample.command.azimuth_rad),
                "elevation_deg": math.degrees(sample.command.elevation_rad),
                "bank_deg": math.degrees(sample.command.bank_rad),
                "classification": sample.classification,
                "success": sample.feasible,
                "termination": sample.trajectory.termination.value,
                "timed_out": sample.timed_out,
                "failure_reasons": list(sample.failure_reasons),
                "limiting_factor": sample.limiting_factor,
                "terminal_time_s": sample.terminal.time_s,
                "terminal_downrange_m": sample.terminal.position_m[0],
                "terminal_crossrange_m": sample.terminal.position_m[1],
                "terminal_altitude_m": sample.terminal.position_m[2],
                "terminal_speed_m_s": sample.terminal.speed_m_s,
                "terminal_impact_radius_m": metrics.get("terminal_impact_radius_m"),
                "terminal_impact_speed_m_s": metrics.get("terminal_impact_speed_m_s"),
                "target_id": envelope.terminal_criteria.target_id,
                "target_frame": envelope.terminal_criteria.target_frame,
                "maximum_altitude_m": metrics.get("maximum_altitude_m"),
                "maximum_speed_m_s": metrics.get("maximum_speed_m_s"),
                "minimum_mass_kg": metrics.get("minimum_mass_kg"),
                "phases": sorted({state.phase for state in sample.trajectory.states}),
                "deployment_event_count": len(sample.trajectory.deployment_events),
                "mission_event_count": len(sample.trajectory.mission_events),
                "mission_segments": sorted(
                    {
                        str(row.get("mission_segment"))
                        for row in sample.trajectory.telemetry
                        if row.get("mission_segment") not in {None, "unprofiled"}
                    }
                ),
                "child_body_id": None if child is None else child.body_id,
                "child_shape": None if child is None else child.shape,
                "child_termination": None if child is None else child.termination.value,
                "child_peak_rate_rad_s": None if child is None else _peak_child_rate(child),
            }
        )
    return rows
    ####


def _parent_rows(envelope: ReachabilityEnvelope) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for sample in envelope.samples:
        telemetry = tuple(sample.trajectory.telemetry)
        for index, state in enumerate(sample.trajectory.states):
            row: dict[str, object] = {
                "fidelity": envelope.fidelity.value,
                "query_id": sample.query_id,
                "candidate_index": sample.index,
                "time_s": state.time_s,
                "phase": state.phase,
                "position_m": list(state.position_m),
                "velocity_m_s": list(state.velocity_m_s),
                "speed_m_s": state.speed_m_s,
                "mass_kg": state.mass_kg,
            }
            if index < len(telemetry):
                row.update(telemetry[index])
            _flatten_source_telemetry(row)
            rows.append(row)
    return rows
    ####


def _flatten_source_telemetry(row: dict[str, object]) -> None:
    """Expose source and actuator channels as analysis-friendly CSV columns."""

    source = row.get("source_aerodynamics")
    if not isinstance(source, Mapping):
        return
    coefficients = source.get("coefficients")
    if isinstance(coefficients, Mapping):
        for name, value in coefficients.items():
            row[f"source_coefficient_{name}"] = value
    operating_point = source.get("operating_point")
    if isinstance(operating_point, Mapping):
        for name, value in operating_point.items():
            row[f"source_operating_{name}"] = value
    actuator = source.get("actuator")
    if not isinstance(actuator, Mapping):
        return
    for channel, values in (
        ("requested", actuator.get("requested_deg")),
        ("achieved", actuator.get("achieved_deg")),
        ("rate", actuator.get("rate_deg_s")),
    ):
        if isinstance(values, Mapping):
            for name, value in values.items():
                row[f"actuator_{channel}_{name}"] = value
    for field in ("saturated", "rate_limited"):
        values = actuator.get(field)
        if isinstance(values, list):
            row[f"actuator_{field}"] = list(values)


def _child_rows(envelope: ReachabilityEnvelope) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for sample in envelope.samples:
        for child in sample.trajectory.spawned_bodies:
            telemetry = tuple(child.telemetry)
            for index, state in enumerate(child.states):
                row: dict[str, object] = {
                    "fidelity": envelope.fidelity.value,
                    "query_id": sample.query_id,
                    "candidate_index": sample.index,
                    "body_id": child.body_id,
                    "shape": child.shape,
                    "parent_event_id": child.parent_event_id,
                    "deployment_time_s": child.deployment_time_s,
                    "time_s": state.time_s,
                    "phase": state.phase,
                    "position_m": list(state.position_m),
                    "velocity_m_s": list(state.velocity_m_s),
                    "speed_m_s": state.speed_m_s,
                    "mass_kg": state.mass_kg,
                }
                if index < len(telemetry):
                    row.update(telemetry[index])
                rows.append(row)
    return rows
    ####


def _surface_rows(envelope: ReachabilityEnvelope) -> list[dict[str, object]]:
    if envelope.fidelity is not ReachabilityFidelity.RIGID_BODY_6DOF_SURFACE_ALLOCATED:
        return []
    rows: list[dict[str, object]] = []
    for sample in envelope.samples:
        for telemetry in sample.trajectory.telemetry:
            if "surface_allocation_active" not in telemetry:
                continue
            rows.append({"fidelity": envelope.fidelity.value, "query_id": sample.query_id, "candidate_index": sample.index, **telemetry})
    return rows
    ####


def _cross_fidelity_summary(envelopes: Sequence[ReachabilityEnvelope]) -> dict[str, object]:
    reference = envelopes[0]
    rows: list[dict[str, object]] = []
    for envelope in envelopes[1:]:
        for baseline, comparison in zip(reference.samples, envelope.samples, strict=True):
            rows.append(
                {
                    "query_id": comparison.query_id,
                    "fidelity": envelope.fidelity.value,
                    "terminal_downrange_delta_m_vs_3dof": comparison.terminal.position_m[0] - baseline.terminal.position_m[0],
                    "terminal_altitude_delta_m_vs_3dof": comparison.terminal.position_m[2] - baseline.terminal.position_m[2],
                    "terminal_speed_delta_m_s_vs_3dof": comparison.terminal.speed_m_s - baseline.terminal.speed_m_s,
                    "termination_changed": comparison.trajectory.termination.value != baseline.trajectory.termination.value,
                }
            )
    return {
        "reference_fidelity": reference.fidelity.value,
        "candidate_deltas": rows,
        "comparison_note": "Differences expose model-form changes. They are not an accuracy ranking without independently qualified source dynamics.",
    }
    ####


def _phase_schedule(envelope: ReachabilityEnvelope) -> dict[str, float]:
    parameters = dict(envelope.vehicle_parameters)
    return {
        "boost_start_s": 0.0,
        "burnout_time_s": _finite_number(parameters.get("booster_burn_time_s")),
        "release_time_s": _finite_number(parameters.get("booster_release_time_s")),
        "horizon_s": envelope.horizon_s,
    }
    ####


def _stage_markers(envelope: ReachabilityEnvelope) -> tuple[tuple[str, float, str], ...]:
    schedule = _phase_schedule(envelope)
    markers: list[tuple[str, float, str]] = []
    if schedule["burnout_time_s"] > 0.0:
        markers.append(("burnout", schedule["burnout_time_s"], _COAST))
    if schedule["release_time_s"] > 0.0:
        markers.append(("release", schedule["release_time_s"], _CHILD))
    return tuple(markers)
    ####


def _phase_bands(axis: Any, envelope: ReachabilityEnvelope) -> None:
    schedule = _phase_schedule(envelope)
    terminal_time = max(sample.terminal.time_s for sample in envelope.samples)
    boundaries = (
        (0.0, min(schedule["burnout_time_s"], terminal_time), _BOOST),
        (schedule["burnout_time_s"], min(schedule["release_time_s"], terminal_time), _COAST),
        (schedule["release_time_s"], terminal_time, _GLIDE),
    )
    for start, stop, color in boundaries:
        if stop > start:
            axis.axvspan(start, stop, color=color, alpha=0.08, zorder=0)
    ####


def _stage_lines(axis: Any, envelope: ReachabilityEnvelope, *, labels: bool) -> None:
    for label, time_s, color in _stage_markers(envelope):
        if time_s > max(sample.terminal.time_s for sample in envelope.samples):
            continue
        axis.axvline(time_s, color=color, linewidth=0.9, linestyle=":", zorder=1)
        if labels:
            axis.text(time_s, 0.98, label, rotation=90, color=color, fontsize=6.8, ha="right", va="top", transform=axis.get_xaxis_transform())
    ####


def _plot_phase_path(
    axis: Any,
    states: Sequence[Any],
    *,
    horizontal_index: int,
    vertical_index: int,
    linewidth: float,
    alpha: float,
) -> None:
    colors = {"boost": _BOOST, "coast": _COAST, "glide": _GLIDE}
    for left, right in zip(states[:-1], states[1:], strict=True):
        axis.plot(
            [left.position_m[horizontal_index] / 1_000.0, right.position_m[horizontal_index] / 1_000.0],
            [left.position_m[vertical_index] / 1_000.0, right.position_m[vertical_index] / 1_000.0],
            color=colors.get(left.phase, _MUTED),
            linewidth=linewidth,
            alpha=alpha,
        )
    ####


def _phase_legend(axis: Any, *, child: bool) -> None:
    from matplotlib.lines import Line2D

    handles = [
        Line2D([0], [0], color=_BOOST, linewidth=2.5, label="boost"),
        Line2D([0], [0], color=_COAST, linewidth=2.5, label="coast"),
        Line2D([0], [0], color=_GLIDE, linewidth=2.5, label="HL-20 glide"),
    ]
    if child:
        handles.append(Line2D([0], [0], color=_CHILD, linewidth=1.8, linestyle="--", label="spent cylinder"))
    axis.legend(handles=handles, loc="best", fontsize=7.3, facecolor=_PANEL, edgecolor=_GRID, labelcolor=_TEXT)
    ####


def _card_text(axis: Any, title: str, lines: Sequence[tuple[str, str]]) -> None:
    axis.text(0.0, 1.0, title, color=_TEXT, fontsize=10.5, fontweight="bold", ha="left", va="top", transform=axis.transAxes)
    top = 0.87
    step = 0.115 if len(lines) <= 6 else 0.097
    for index, (label, value) in enumerate(lines):
        axis.text(0.0, top - index * step, label, color="#93c5fd", fontsize=7.35, fontweight="bold", ha="left", va="top", transform=axis.transAxes)
        axis.text(0.0, top - index * step - 0.035, value, color=_MUTED, fontsize=7.1, ha="left", va="top", wrap=True, transform=axis.transAxes)
    ####


def _unavailable(axis: Any, text: str) -> None:
    axis.text(0.5, 0.5, text, color=_MUTED, fontsize=9.0, ha="center", va="center", wrap=True, transform=axis.transAxes)
    ####


def _tier_row_label(axis: Any, envelope: ReachabilityEnvelope, index: int) -> None:
    axis.text(-0.31, 0.5, _TIER_LABELS[envelope.fidelity], rotation=90, color=_TIER_COLORS[index], fontsize=9.5, fontweight="bold", ha="center", va="center", transform=axis.transAxes)
    ####


def _style_axis(axis: Any) -> None:
    axis.set_facecolor(_PANEL)
    axis.tick_params(colors=_MUTED, labelsize=8)
    for spine in axis.spines.values():
        spine.set_color(_GRID)
    axis.grid(True, color=_GRID, linewidth=0.55, alpha=0.56)
    axis.set_axisbelow(True)
    ####


def _style_twin_axis(axis: Any) -> None:
    axis.set_facecolor("none")
    axis.tick_params(colors=_MUTED, labelsize=8)
    for spine in axis.spines.values():
        spine.set_color(_GRID)
    ####


def _nominal_sample(envelope: ReachabilityEnvelope) -> EnvelopeSample:
    ranked = sorted(envelope.samples, key=lambda sample: (sample.command.elevation_rad, sample.command.azimuth_rad, sample.index))
    return ranked[len(ranked) // 2]
    ####


def _nominal_child(sample: EnvelopeSample) -> DetachedBodyTrajectory | None:
    return sample.trajectory.spawned_bodies[0] if sample.trajectory.spawned_bodies else None
    ####


def _state_at_time(sample: EnvelopeSample, time_s: float) -> Any | None:
    candidates = [state for state in sample.trajectory.states if state.time_s <= time_s + 1.0e-9]
    return candidates[-1] if candidates else None
    ####


def _times(sample: EnvelopeSample) -> list[float]:
    return [state.time_s for state in sample.trajectory.states]
    ####


def _telemetry_times(rows: Sequence[Mapping[str, object]]) -> list[float]:
    return [_number(row, "time_s") for row in rows]
    ####


def _rate_magnitude(vector: Sequence[float]) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in vector))
    ####


def _row_rate_magnitude(row: Mapping[str, object]) -> float:
    values = row.get("attitude_rate_rad_s")
    if isinstance(values, list) and len(values) == 3:
        return _rate_magnitude([float(value) for value in values])
    return 0.0
    ####


def _peak_child_rate(child: DetachedBodyTrajectory) -> float:
    return max((_row_rate_magnitude(row) for row in child.telemetry), default=0.0)
    ####


def _number(row: Mapping[str, object], key: str) -> float:
    value = row.get(key, 0.0)
    if isinstance(value, bool):
        return float(value)
    if isinstance(value, (float, int)):
        return float(value)
    return 0.0
    ####


def _finite_number(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        return 0.0
    return float(value)
    ####


def _termination_counts(envelope: ReachabilityEnvelope) -> dict[str, int]:
    counts: dict[str, int] = {}
    for sample in envelope.samples:
        termination = sample.trajectory.termination.value
        counts[termination] = counts.get(termination, 0) + 1
    return dict(sorted(counts.items()))
    ####


def _classification_description(candidates: Mapping[str, object]) -> str:
    counts = candidates.get("classification_counts", {})
    if not isinstance(counts, Mapping):
        return "classification unavailable"
    return ", ".join(f"{value} {key}" for key, value in sorted(counts.items()))
    ####


def _termination_description(candidates: Mapping[str, object]) -> str:
    counts = candidates.get("termination_counts", {})
    if not isinstance(counts, Mapping):
        return "terminal disposition unavailable"
    return ", ".join(f"{value} {key}" for key, value in sorted(counts.items()))
    ####


def _mass_schedule_description(envelope: ReachabilityEnvelope) -> str:
    parameters = dict(envelope.vehicle_parameters)
    initial = _finite_number(parameters.get("dry_mass_kg")) + _finite_number(parameters.get("propellant_mass_kg")) + _finite_number(parameters.get("booster_dry_mass_kg")) + _finite_number(parameters.get("booster_propellant_mass_kg"))
    burnout = _finite_number(parameters.get("dry_mass_kg")) + _finite_number(parameters.get("propellant_mass_kg")) + _finite_number(parameters.get("booster_dry_mass_kg"))
    release = _finite_number(parameters.get("dry_mass_kg")) + _finite_number(parameters.get("propellant_mass_kg"))
    return f"{initial / 1_000.0:.3f} t start -> {burnout / 1_000.0:.3f} t burnout -> {release / 1_000.0:.3f} t HL-20"
    ####


def _short_fidelity_label(fidelity: ReachabilityFidelity) -> str:
    return {
        ReachabilityFidelity.POINT_MASS_3DOF: "3DOF",
        ReachabilityFidelity.PSEUDO_6DOF: "pseudo-6DOF",
        ReachabilityFidelity.RIGID_BODY_6DOF: "rigid 6DOF",
        ReachabilityFidelity.RIGID_BODY_6DOF_SURFACE_ALLOCATED: "rigid + surfaces",
    }[fidelity]
    ####


def _search_description(envelope: ReachabilityEnvelope) -> str:
    if envelope.search_space is None:
        return f"{len(envelope.samples)} retained candidates"
    axes = {axis.name: axis for axis in envelope.search_space.axes}
    elevation = axes.get("launch.elevation_rad")
    azimuth = axes.get("launch.azimuth_rad")
    bank = axes.get("glide.bank_rad")
    elevation_text = "/".join(f"{math.degrees(value):.0f}" for value in elevation.values) if elevation is not None else "n/a"
    azimuth_text = "/".join(f"{math.degrees(value):.0f}" for value in azimuth.values) if azimuth is not None else "n/a"
    bank_text = "/".join(f"{math.degrees(value):.0f}" for value in bank.values) if bank is not None else "n/a"
    return f"Search grid: elevation {elevation_text} deg; azimuth {azimuth_text} deg; bank {bank_text} deg; {len(envelope.samples)} candidates"
    ####


def _search_signature(envelope: ReachabilityEnvelope) -> str:
    payload = envelope.search_space.as_dict() if envelope.search_space is not None else {"samples": len(envelope.samples)}
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))
    ####


def _envelope_configuration_hash(envelope: ReachabilityEnvelope) -> str:
    payload = {
        "study_id": envelope.study_id,
        "vehicle_id": envelope.vehicle_id,
        "fidelity": envelope.fidelity.value,
        "search_space": envelope.search_space.as_dict() if envelope.search_space is not None else {},
        "step_size_s": envelope.step_size_s,
        "horizon_s": envelope.horizon_s,
        "vehicle_parameters": dict(envelope.vehicle_parameters),
        "provenance": dict(envelope.provenance),
    }
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":"), default=str).encode("utf-8")).hexdigest()
    ####


def _write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> Path:
    keys = sorted({str(key) for row in rows for key in row})
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    key: json.dumps(row[key], sort_keys=True) if isinstance(row.get(key), (dict, list, tuple)) else row.get(key, "")
                    for key in keys
                }
            )
    return path
    ####


def _write_json(path: Path, payload: Mapping[str, object]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _file_record(path: Path, *, relative_to: Path | None = None) -> dict[str, object]:
    display_path = path.name if relative_to is None else Path(os.path.relpath(path, relative_to)).as_posix()
    return {
        "path": display_path,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "bytes": path.stat().st_size,
    }
    ####


def _save_figure(figure: Any, path: Path, dpi: int) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=dpi, facecolor=figure.get_facecolor())
    return path
    ####


def _get_matplotlib() -> Any:
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    return plt
    ####


__all__ = ["HL20ShowcaseCompositeReport", "render_hl20_ca_hi_showcase_composites"]
