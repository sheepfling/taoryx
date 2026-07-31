"""Qualify the Alpha 3 passive aero-ballistic deployment profiles."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import random
from dataclasses import replace
from pathlib import Path

from taoryx.contracts import Vector3
from taoryx.reachability_envelope import (
    LaunchCommand,
    ReachabilityEnvelope,
    ReachabilityFidelity,
    RocketGlideVehicle,
    TrajectoryResult,
    run_reachability_envelope,
    simulate_rocket_glide,
)
from taoryx.reachability_visualization import render_reachability_plot_bundle
from taoryx.vehicle import DetachedBodyDefinition, DetachedBodyShape, TumblingPolicy

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "artifacts/verification/alpha3-passive-deployment.json"
DEFAULT_ARTIFACT_DIR = ROOT / "artifacts/deployment/passive_ballistic"
FIDELITIES = (
    ReachabilityFidelity.POINT_MASS_3DOF,
    ReachabilityFidelity.PSEUDO_6DOF,
    ReachabilityFidelity.RIGID_BODY_6DOF,
)
SHAPES = (
    DetachedBodyShape.SPHERE,
    DetachedBodyShape.CYLINDER,
    DetachedBodyShape.CONE,
    DetachedBodyShape.TRIAXIAL_ELLIPSOID,
)


def _inertia_for_shape(shape: DetachedBodyShape, mass_kg: float) -> Vector3:
    if shape is DetachedBodyShape.SPHERE:
        value = 0.4 * mass_kg * 0.45**2
        return Vector3(value, value, value)
    if shape is DetachedBodyShape.CYLINDER:
        return Vector3(0.5 * mass_kg * 0.35**2, mass_kg * (3.0 * 0.35**2 + 2.0**2) / 12.0, mass_kg * (3.0 * 0.35**2 + 2.0**2) / 12.0)
    if shape is DetachedBodyShape.CONE:
        return Vector3(0.3 * mass_kg * 0.45**2, mass_kg * (0.15 * 0.45**2 + 0.6 * 1.8**2), mass_kg * (0.15 * 0.45**2 + 0.6 * 1.8**2))
    return Vector3(
        mass_kg * (0.65**2 + 0.45**2) / 5.0,
        mass_kg * (1.4**2 + 0.45**2) / 5.0,
        mass_kg * (1.4**2 + 0.65**2) / 5.0,
    )


def _body_for_shape(
    shape: DetachedBodyShape,
    *,
    angular_rate: Vector3 = Vector3(0.25, 0.4, 0.6),
    tumbling_policy: TumblingPolicy = TumblingPolicy.PASSIVE_TUMBLE,
) -> DetachedBodyDefinition:
    """Return one canonical passive body profile with explicit inertia."""

    mass_kg = 12.0
    inertia = _inertia_for_shape(shape, mass_kg)
    if shape is DetachedBodyShape.SPHERE:
        return DetachedBodyDefinition.sphere(
            "passive-sphere",
            mass_kg=mass_kg,
            radius_m=0.45,
            tumbling_policy=tumbling_policy,
            inertia_kg_m2=inertia,
            initial_angular_rate_body_rad_s=angular_rate,
        )
    if shape is DetachedBodyShape.CYLINDER:
        return DetachedBodyDefinition.cylinder(
            "passive-cylinder",
            mass_kg=mass_kg,
            radius_m=0.35,
            length_m=2.0,
            tumbling_policy=tumbling_policy,
            inertia_kg_m2=inertia,
            initial_angular_rate_body_rad_s=angular_rate,
        )
    if shape is DetachedBodyShape.CONE:
        return DetachedBodyDefinition.cone(
            "passive-cone",
            mass_kg=mass_kg,
            base_radius_m=0.45,
            height_m=1.8,
            tumbling_policy=tumbling_policy,
            inertia_kg_m2=inertia,
            initial_angular_rate_body_rad_s=angular_rate,
        )
    return DetachedBodyDefinition.triaxial_ellipsoid(
        "passive-triaxial-ellipsoid",
        mass_kg=mass_kg,
        semi_axis_x_m=1.4,
        semi_axis_y_m=0.65,
        semi_axis_z_m=0.45,
        tumbling_policy=tumbling_policy,
        inertia_kg_m2=inertia,
        initial_angular_rate_body_rad_s=angular_rate,
    )


def _vehicle(
    body: DetachedBodyDefinition,
    *,
    initial_speed_m_s: float,
    bank_rad: float,
    drag_scale: float,
    density_scale: float,
    scale_height_scale: float,
    sample_id: str,
) -> RocketGlideVehicle:
    """Build the common synthetic release parent for deployment qualification."""

    return RocketGlideVehicle(
        vehicle_id=f"alpha3-passive-parent-{body.shape.value}-v1",
        dry_mass_kg=1_000.0,
        propellant_mass_kg=0.0,
        thrust_n=0.0,
        burn_time_s=0.0,
        reference_area_m2=0.1,
        drag_coefficient=0.25 * drag_scale,
        lift_to_drag=0.1,
        initial_speed_m_s=initial_speed_m_s,
        initial_altitude_m=1_000.0,
        sea_level_density_kg_m3=1.225 * density_scale,
        density_scale_height_m=8_500.0 * scale_height_scale,
        configuration_variant_id=sample_id,
        mass_property_profile_id="alpha3-passive-deployment-parent-v1",
        booster_dry_mass_kg=body.mass_kg,
        booster_propellant_mass_kg=1.0,
        booster_thrust_n=10.0,
        booster_burn_time_s=0.5,
        booster_release_time_s=0.5,
        booster_detached_body=body,
    )


def _command(bank_rad: float) -> LaunchCommand:
    return LaunchCommand(azimuth_rad=0.0, elevation_rad=0.0, bank_rad=bank_rad)


def _terminal_record(
    trajectory: TrajectoryResult,
    *,
    shape: DetachedBodyShape,
    fidelity: ReachabilityFidelity,
    sample_id: str,
    parameters: dict[str, float],
    parent_model_id: str | None = None,
) -> dict[str, object]:
    event = trajectory.deployment_events[0] if trajectory.deployment_events else None
    child = trajectory.spawned_bodies[0] if trajectory.spawned_bodies else None
    if child is None:
        return {
            "shape": shape.value,
            "fidelity": fidelity.value,
            "sample_id": sample_id,
            "classification": "deployment_failure",
            "termination": trajectory.termination.value,
            "parameters": parameters,
            "lineage": {"parent_model_id": parent_model_id, "child_model_id": None, "event_id": None},
            "failure": "no_child_trajectory_after_requested_deployment",
        }
    initial = child.states[0]
    terminal = child.terminal
    delta = tuple(terminal.position_m[index] - initial.position_m[index] for index in range(3))
    return {
        "shape": shape.value,
        "fidelity": fidelity.value,
        "sample_id": sample_id,
        "classification": child.classification,
        "termination": child.termination.value,
        "parameters": parameters,
        "lineage": {
            "parent_model_id": None if event is None else event.get("parent_model_id"),
            "child_model_id": child.body_id,
            "event_id": child.parent_event_id,
            "accepted_time_s": child.deployment_time_s,
        },
        "initial_state": {
            "position_m": list(initial.position_m),
            "velocity_m_s": list(initial.velocity_m_s),
            "mass_kg": initial.mass_kg,
        },
        "terminal_state": {
            "position_m": list(terminal.position_m),
            "velocity_m_s": list(terminal.velocity_m_s),
            "mass_kg": terminal.mass_kg,
            "speed_m_s": terminal.speed_m_s,
            "altitude_m": terminal.position_m[2],
        },
        "footprint_m": {"downrange": delta[0], "crossrange": delta[1], "radius": math.hypot(delta[0], delta[1])},
        "terminal_finite": all(math.isfinite(value) for value in (*terminal.position_m, *terminal.velocity_m_s)),
    }


def _run_sample(
    body: DetachedBodyDefinition,
    *,
    fidelity: ReachabilityFidelity,
    sample_id: str,
    mass_scale: float,
    bank_rad: float,
    initial_speed_m_s: float,
    drag_scale: float,
    density_scale: float,
    scale_height_scale: float,
    angular_rate: Vector3,
    horizon_s: float,
    step_size_s: float,
) -> dict[str, object]:
    sample_body = replace(body, initial_angular_rate_body_rad_s=angular_rate)
    vehicle = _vehicle(
        sample_body,
        initial_speed_m_s=initial_speed_m_s,
        bank_rad=bank_rad,
        drag_scale=drag_scale,
        density_scale=density_scale,
        scale_height_scale=scale_height_scale,
        sample_id=sample_id,
    )
    parameters = {
        "mass_scale": mass_scale,
        "drag_scale": drag_scale,
        "density_scale": density_scale,
        "scale_height_scale": scale_height_scale,
        "initial_speed_m_s": initial_speed_m_s,
        "bank_rad": bank_rad,
        "angular_rate_x_rad_s": angular_rate.x,
        "angular_rate_y_rad_s": angular_rate.y,
        "angular_rate_z_rad_s": angular_rate.z,
    }
    try:
        trajectory = simulate_rocket_glide(
            vehicle,
            _command(bank_rad),
            fidelity=fidelity,
            step_size_s=step_size_s,
            horizon_s=horizon_s,
            spawn_children=True,
        )
    except (OverflowError, ValueError) as error:
        return {
            "shape": body.shape.value,
            "fidelity": fidelity.value,
            "sample_id": sample_id,
            "classification": "invalid",
            "parameters": parameters,
            "lineage": {"parent_model_id": vehicle.vehicle_id, "child_model_id": body.body_id, "event_id": None},
            "failure": str(error),
        }
    return _terminal_record(
        trajectory,
        shape=body.shape,
        fidelity=fidelity,
        sample_id=sample_id,
        parameters=parameters,
        parent_model_id=vehicle.vehicle_id,
    )


def _quantile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def _distribution(records: list[dict[str, object]], *, classification: str | None = None) -> dict[str, object]:
    selected = [record for record in records if classification is None or record.get("classification") == classification]
    values: dict[str, list[float]] = {"downrange_m": [], "crossrange_m": [], "radius_m": [], "speed_m_s": [], "altitude_m": []}
    for record in selected:
        footprint = record.get("footprint_m")
        terminal = record.get("terminal_state")
        if not isinstance(footprint, dict) or not isinstance(terminal, dict):
            continue
        values["downrange_m"].append(float(footprint["downrange"]))
        values["crossrange_m"].append(float(footprint["crossrange"]))
        values["radius_m"].append(float(footprint["radius"]))
        values["speed_m_s"].append(float(terminal["speed_m_s"]))
        values["altitude_m"].append(float(terminal["altitude_m"]))
    return {
        "classification": classification or "all_finite_terminals",
        "sample_count": len(selected),
        "finite_count": len(values["radius_m"]),
        "quantiles": {
            name: {str(percent): _quantile(series, percent / 100.0) for percent in (0, 5, 50, 95, 100)}
            for name, series in values.items()
        },
    }


def _area_policy_witness(telemetry: tuple[dict[str, object], ...]) -> dict[str, object]:
    """Summarize the declared area reduction and native area evolution."""

    policies = sorted({str(row["projected_area_policy"]) for row in telemetry})
    areas = [float(row["projected_area_m2"]) for row in telemetry]
    ratios = [float(row["projected_area_ratio_to_reference_average"]) for row in telemetry]
    rates = [float(row["angular_rate_norm_rad_s"]) for row in telemetry]
    references = [float(row["projected_area_reference_average_m2"]) for row in telemetry]
    finite = all(math.isfinite(value) for value in (*areas, *ratios, *rates, *references))
    return {
        "policies": policies,
        "finite": finite,
        "reference_average_area_m2": references[0] if references else None,
        "projected_area_min_m2": min(areas) if areas else None,
        "projected_area_max_m2": max(areas) if areas else None,
        "projected_area_span_m2": max(areas) - min(areas) if areas else None,
        "ratio_to_reference_average_min": min(ratios) if ratios else None,
        "ratio_to_reference_average_max": max(ratios) if ratios else None,
        "angular_rate_max_rad_s": max(rates) if rates else None,
        "sample_count": len(telemetry),
    }
    ####


def _rotation_policy_witness(
    shape: DetachedBodyShape,
    *,
    horizon_s: float,
    step_size_s: float,
) -> dict[str, object]:
    """Compare passive aerodynamic moments with a zero-moment spin baseline."""

    records: dict[str, dict[str, object]] = {}
    for policy, label in (
        (TumblingPolicy.PASSIVE_TUMBLE, "passive_aerodynamic_moment"),
        (TumblingPolicy.PRESCRIBED_SPIN, "prescribed_spin_zero_aerodynamic_moment"),
    ):
        body = _body_for_shape(shape, tumbling_policy=policy)
        vehicle = _vehicle(
            body,
            initial_speed_m_s=250.0,
            bank_rad=0.0,
            drag_scale=1.0,
            density_scale=1.0,
            scale_height_scale=1.0,
            sample_id=f"rotation-policy-{shape.value}-{label}",
        )
        trajectory = simulate_rocket_glide(
            vehicle,
            _command(0.0),
            fidelity=ReachabilityFidelity.RIGID_BODY_6DOF,
            step_size_s=step_size_s,
            horizon_s=horizon_s,
            spawn_children=True,
        )
        child = trajectory.spawned_bodies[0]
        telemetry = child.telemetry
        moments = [
            math.sqrt(
                sum(float(row[f"aero_moment_body_{axis}_nm"]) ** 2 for axis in ("x", "y", "z"))
            )
            for row in telemetry
        ]
        rates = [float(row["angular_rate_norm_rad_s"]) for row in telemetry]
        records[label] = {
            "tumbling_policy": policy.value,
            "classification": child.classification,
            "termination": child.termination.value,
            "aerodynamic_moment_norm_max_nm": max(moments) if moments else None,
            "aerodynamic_moment_norm_min_nm": min(moments) if moments else None,
            "angular_rate_norm_max_rad_s": max(rates) if rates else None,
            "telemetry_sample_count": len(telemetry),
            "finite": all(math.isfinite(value) for value in (*moments, *rates)),
        }
    return {
        "comparison": "passive_aerodynamic_moment_vs_zero_moment_spin_baseline",
        "records": records,
        "claim_boundary": "The zero-moment baseline is a diagnostic passive reference, not a physical controller or a prescribed-tumble qualification.",
    }
    ####


def _write_footprint_plots(records: list[dict[str, object]], destination: Path, dpi: int) -> list[Path]:
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    destination.mkdir(parents=True, exist_ok=True)
    paths: list[Path] = []
    figure, axes = plt.subplots(2, 2, figsize=(12.0, 9.0), constrained_layout=True)
    for axis, shape in zip(axes.flat, SHAPES, strict=True):
        selected = [record for record in records if record.get("shape") == shape.value]
        by_classification: dict[str, list[dict[str, object]]] = {}
        for record in selected:
            by_classification.setdefault(str(record.get("classification")), []).append(record)
        for classification, group in sorted(by_classification.items()):
            points: list[dict[str, float]] = []
            for record in group:
                point = record.get("footprint_m")
                if isinstance(point, dict) and all(isinstance(point.get(name), (int, float)) for name in ("downrange", "crossrange")):
                    points.append({"downrange": float(point["downrange"]), "crossrange": float(point["crossrange"])})
            axis.scatter(
                [point["downrange"] / 1_000.0 for point in points],
                [point["crossrange"] / 1_000.0 for point in points],
                s=28,
                alpha=0.75,
                label=classification,
            )
        axis.set_title(shape.value)
        axis.set_xlabel("Downrange from release (km)")
        axis.set_ylabel("Crossrange from release (km)")
        axis.grid(alpha=0.25)
        axis.legend(fontsize=8)
    figure.suptitle("Passive aero-ballistic terminal footprint uncertainty")
    paths.append(destination / "terminal-footprints.png")
    figure.savefig(paths[-1], dpi=dpi, bbox_inches="tight")
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(11.0, 6.0), constrained_layout=True)
    for shape in SHAPES:
        selected = [record for record in records if record.get("shape") == shape.value and record.get("footprint_m")]
        radii: list[float] = []
        for record in selected:
            footprint = record.get("footprint_m")
            if isinstance(footprint, dict) and isinstance(footprint.get("radius"), (int, float)):
                radii.append(float(footprint["radius"]) / 1_000.0)
        if radii:
            axis.boxplot(radii, positions=[SHAPES.index(shape) + 1], widths=0.45)
    axis.set_ylabel("Terminal footprint radius (km)")
    axis.set_title("Physical terminal footprint distributions by geometry")
    axis.set_xticks(range(1, len(SHAPES) + 1), [shape.value for shape in SHAPES])
    axis.grid(axis="y", alpha=0.25)
    paths.append(destination / "terminal-footprint-distributions.png")
    figure.savefig(paths[-1], dpi=dpi, bbox_inches="tight")
    plt.close(figure)
    return paths


def _write_area_policy_plot(
    envelopes: dict[ReachabilityFidelity, ReachabilityEnvelope],
    destination: Path,
    shape: DetachedBodyShape,
    dpi: int,
) -> Path:
    """Render the passive area-policy and rotational-state diagnostic board."""

    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    destination.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(2, 2, figsize=(12.0, 8.0), constrained_layout=True)
    colors = {
        ReachabilityFidelity.POINT_MASS_3DOF: "#4c78a8",
        ReachabilityFidelity.PSEUDO_6DOF: "#f58518",
        ReachabilityFidelity.RIGID_BODY_6DOF: "#54a24b",
    }
    for fidelity in FIDELITIES:
        child = envelopes[fidelity].samples[0].trajectory.spawned_bodies[0]
        telemetry = child.telemetry
        time = [float(row["time_s"]) for row in telemetry]
        label = fidelity.value.replace("_", " ")
        color = colors[fidelity]
        axes[0, 0].plot(time, [float(row["projected_area_m2"]) for row in telemetry], color=color, label=label)
        axes[0, 1].plot(
            time,
            [float(row["projected_area_ratio_to_reference_average"]) for row in telemetry],
            color=color,
            label=label,
        )
        axes[1, 0].plot(time, [float(row["angular_rate_norm_rad_s"]) for row in telemetry], color=color, label=label)
        if fidelity is not ReachabilityFidelity.POINT_MASS_3DOF:
            moment = [
                math.sqrt(
                    sum(float(row[f"aero_moment_body_{axis}_nm"]) ** 2 for axis in ("x", "y", "z"))
                )
                for row in telemetry
            ]
            axes[1, 1].plot(time, moment, color=color, label=label)
    axes[0, 0].set_ylabel("projected area (m²)")
    axes[0, 1].axhline(1.0, color="#777777", linestyle="--", linewidth=1.0)
    axes[0, 1].set_ylabel("area / orientation average")
    axes[1, 0].set_ylabel("angular rate norm (rad/s)")
    axes[1, 1].set_ylabel("aerodynamic moment norm (N·m)")
    for axis in axes.flat:
        axis.set_xlabel("time after release (s)")
        axis.grid(alpha=0.25)
        axis.legend(fontsize=7)
    figure.suptitle(f"Passive {shape.value}: area policy and rotational evidence")
    path = destination / "area-policy-rotation.png"
    figure.savefig(path, dpi=dpi, bbox_inches="tight")
    plt.close(figure)
    return path


def _write_csv(records: list[dict[str, object]], path: Path) -> None:
    fields = (
        "shape",
        "fidelity",
        "sample_id",
        "classification",
        "termination",
        "downrange_m",
        "crossrange_m",
        "radius_m",
        "speed_m_s",
        "altitude_m",
    )
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for record in records:
            footprint = record.get("footprint_m", {})
            terminal = record.get("terminal_state", {})
            writer.writerow(
                {
                    "shape": record.get("shape"),
                    "fidelity": record.get("fidelity"),
                    "sample_id": record.get("sample_id"),
                    "classification": record.get("classification"),
                    "termination": record.get("termination"),
                    "downrange_m": footprint.get("downrange") if isinstance(footprint, dict) else None,
                    "crossrange_m": footprint.get("crossrange") if isinstance(footprint, dict) else None,
                    "radius_m": footprint.get("radius") if isinstance(footprint, dict) else None,
                    "speed_m_s": terminal.get("speed_m_s") if isinstance(terminal, dict) else None,
                    "altitude_m": terminal.get("altitude_m") if isinstance(terminal, dict) else None,
                }
            )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _root_relative(path: Path) -> str:
    """Return a repository-relative path when an artifact is inside the repo."""

    try:
        return path.relative_to(ROOT).as_posix()
    except ValueError:
        return str(path)


def run_qualification(
    output: Path = DEFAULT_OUTPUT,
    artifact_dir: Path = DEFAULT_ARTIFACT_DIR,
    *,
    seed: int = 31_415,
    samples_per_shape: int = 16,
    horizon_s: float = 90.0,
    step_size_s: float = 0.25,
    dpi: int = 120,
) -> dict[str, object]:
    """Run nominal tier qualification and a deterministic footprint ensemble."""

    if samples_per_shape <= 0 or horizon_s <= 0.0 or step_size_s <= 0.0 or dpi <= 0:
        raise ValueError("samples_per_shape, horizon_s, step_size_s, and dpi must be positive")
    output = output.resolve()
    artifact_dir = artifact_dir.resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    uncertainty_records: list[dict[str, object]] = []
    shape_reports: list[dict[str, object]] = []
    for shape_index, shape in enumerate(SHAPES):
        body = _body_for_shape(shape)
        shape_nominal: dict[str, dict[str, object]] = {}
        nominal_envelopes: dict[ReachabilityFidelity, ReachabilityEnvelope] = {}
        for fidelity in FIDELITIES:
            vehicle = _vehicle(
                body,
                initial_speed_m_s=250.0,
                bank_rad=0.0,
                drag_scale=1.0,
                density_scale=1.0,
                scale_height_scale=1.0,
                sample_id=f"nominal-{shape.value}-{fidelity.value}",
            )
            envelope = run_reachability_envelope(
                vehicle,
                (_command(0.0),),
                fidelity=fidelity,
                step_size_s=step_size_s,
                horizon_s=horizon_s,
                spawn_children=True,
                study_id=f"alpha3_passive_deployment_{shape.value}_{fidelity.value}_v1",
                provenance={
                    "qualification": "passive_aero_ballistic_deployment",
                    "parent_child_claim_boundary": "child_terminal_footprint_only",
                    "random_seed": seed,
                },
            )
            nominal_envelopes[fidelity] = envelope
            artifact_path = artifact_dir / f"nominal-{shape.value}-{fidelity.value}.json"
            envelope.write_json(artifact_path, include_trajectories=True)
            sample = envelope.samples[0]
            record = _terminal_record(
                sample.trajectory,
                shape=shape,
                fidelity=fidelity,
                sample_id=f"nominal-{shape.value}-{fidelity.value}",
                parameters={"mass_scale": 1.0, "drag_scale": 1.0, "density_scale": 1.0, "scale_height_scale": 1.0, "initial_speed_m_s": 250.0, "bank_rad": 0.0, "angular_rate_x_rad_s": 0.25, "angular_rate_y_rad_s": 0.4, "angular_rate_z_rad_s": 0.6},
                parent_model_id=vehicle.vehicle_id,
            )
            child = sample.trajectory.spawned_bodies[0]
            shape_nominal[fidelity.value] = {
                "artifact": artifact_path.relative_to(artifact_dir).as_posix(),
                "classification": record["classification"],
                "parent_classification_counts": envelope.classification_counts,
                "child_classification_counts": envelope.child_classification_counts,
                "lineage": record["lineage"],
                "terminal_state": record.get("terminal_state"),
                "footprint_m": record.get("footprint_m"),
                "area_policy_witness": _area_policy_witness(child.telemetry),
            }
        plot_dir = artifact_dir / "plots" / shape.value
        plot_report = render_reachability_plot_bundle(
            nominal_envelopes[ReachabilityFidelity.PSEUDO_6DOF],
            plot_dir,
            comparison_sources=(nominal_envelopes[FIDELITIES[0]], nominal_envelopes[FIDELITIES[2]]),
            dpi=dpi,
        )
        rotation_policy = _rotation_policy_witness(
            shape,
            horizon_s=horizon_s,
            step_size_s=step_size_s,
        )
        area_policy_plot = _write_area_policy_plot(
            nominal_envelopes,
            plot_dir,
            shape,
            dpi,
        )
        rng = random.Random(seed + shape_index)
        shape_records: list[dict[str, object]] = []
        for sample_index in range(samples_per_shape):
            mass_scale = rng.uniform(0.9, 1.1)
            angular_rate = Vector3(rng.uniform(-0.6, 0.6), rng.uniform(-0.6, 0.6), rng.uniform(-0.6, 0.6))
            sampled_body = replace(
                body,
                mass_kg=body.mass_kg * mass_scale,
                inertia_kg_m2=body.inertia_kg_m2.scaled(mass_scale) if body.inertia_kg_m2 is not None else None,
            )
            record = _run_sample(
                sampled_body,
                fidelity=ReachabilityFidelity.PSEUDO_6DOF,
                sample_id=f"sweep-{shape.value}-{sample_index:04d}",
                mass_scale=mass_scale,
                bank_rad=rng.uniform(-math.radians(20.0), math.radians(20.0)),
                initial_speed_m_s=rng.uniform(237.5, 262.5),
                drag_scale=rng.uniform(0.9, 1.1),
                density_scale=rng.uniform(0.9, 1.1),
                scale_height_scale=rng.uniform(0.95, 1.05),
                angular_rate=angular_rate,
                horizon_s=horizon_s,
                step_size_s=step_size_s,
            )
            shape_records.append(record)
            uncertainty_records.append(record)
        counts: dict[str, int] = {}
        for record in shape_records:
            classification = str(record["classification"])
            counts[classification] = counts.get(classification, 0) + 1
        shape_reports.append(
            {
                "shape": shape.value,
                "dimensions_contract": list(body.dimensions_m),
                "nominal": shape_nominal,
                "uncertainty": {
                    "fidelity": ReachabilityFidelity.PSEUDO_6DOF.value,
                    "sample_count": len(shape_records),
                    "classification_counts": dict(sorted(counts.items())),
                    "terminal_distribution": _distribution(shape_records),
                    "impact_distribution": _distribution(shape_records, classification="impact"),
                },
                "plot_manifest": plot_report.manifest_path.relative_to(artifact_dir).as_posix(),
                "area_policy_plot": area_policy_plot.relative_to(artifact_dir).as_posix(),
                "rotation_policy_witness": rotation_policy,
            }
        )
    csv_path = artifact_dir / "terminal-footprint-samples.csv"
    _write_csv(uncertainty_records, csv_path)
    plot_paths = _write_footprint_plots(uncertainty_records, artifact_dir / "ensemble-plots", dpi)
    report: dict[str, object] = {
        "schema": "taoryx.alpha3-passive-deployment-qualification/v1alpha1",
        "status": "qualified_witness_with_physical_footprint_uncertainty",
        "claim_boundary": "passive child terminal footprints and deployment lineage; not controller-effective reachability, source-exact child aerodynamics, or parent capability",
        "parent_capability_claim": "not_promoted",
        "passive_physics_contract": {
            "point_mass_3dof": {
                "projected_area_policy": "orientation_averaged_projected_area",
                "claim": "center-of-mass translation using a deterministic geometry-specific orientation average",
                "nonclaim": "instantaneous attitude-dependent drag or moment-driven tumble",
            },
            "pseudo_6dof": {
                "projected_area_policy": "native_rigid_body_reuse_instantaneous_projected_area",
                "claim": "native rigid-body rotational and attitude state exposed through the pseudo interface",
                "nonclaim": "prescribed attitude response or control authority",
            },
            "rigid_body_6dof": {
                "projected_area_policy": "instantaneous_geometry_projected_area",
                "claim": "passive attitude-dependent projected area, drag, aerodynamic moment, and body-rate evolution",
                "nonclaim": "active control, actuator, or guidance authority",
            },
        },
        "contract": {
            "shapes": [shape.value for shape in SHAPES],
            "fidelities": [fidelity.value for fidelity in FIDELITIES],
            "child_policy": TumblingPolicy.PASSIVE_TUMBLE.value,
            "parent_model": "synthetic_fixed_mass_release_parent_v1",
            "event_contract": "accepted_booster_release_with_parent_child_lineage",
            "release_state": {
                "initial_altitude_m": 1_000.0,
                "initial_speed_m_s": 250.0,
                "azimuth_rad": 0.0,
                "elevation_rad": 0.0,
                "bank_rad": 0.0,
                "separation_impulse_n_s": [0.0, 0.0, 0.0],
            },
        },
        "execution": {
            "seed": seed,
            "samples_per_shape": samples_per_shape,
            "horizon_s": horizon_s,
            "step_size_s": step_size_s,
            "deterministic": True,
            "random_stream_policy": "seed_plus_shape_index",
        },
        "uncertainty_axes": {
            "mass_scale": [0.9, 1.1],
            "drag_scale": [0.9, 1.1],
            "density_scale": [0.9, 1.1],
            "scale_height_scale": [0.95, 1.05],
            "release_speed_m_s": [237.5, 262.5],
            "release_bank_rad": [-math.radians(20.0), math.radians(20.0)],
            "initial_angular_rate_rad_s": [-0.6, 0.6],
        },
        "lineage_policy": {
            "parent_and_child_ids_required": True,
            "accepted_event_id_required": True,
            "child_result_separate_from_parent_envelope": True,
            "physical_footprint_not_controller_reachability": True,
        },
        "shapes": shape_reports,
        "ensemble_artifacts": {
            "terminal_samples_csv": csv_path.relative_to(artifact_dir).as_posix(),
            "plots": [path.relative_to(artifact_dir).as_posix() for path in plot_paths],
        },
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest_files = [output, csv_path, *plot_paths]
    for shape_report in shape_reports:
        nominal_payload = shape_report.get("nominal")
        if isinstance(nominal_payload, dict):
            for fidelity_report in nominal_payload.values():
                if isinstance(fidelity_report, dict) and "artifact" in fidelity_report:
                    manifest_files.append(artifact_dir / str(fidelity_report["artifact"]))
        if "plot_manifest" in shape_report:
            manifest_files.append(artifact_dir / str(shape_report["plot_manifest"]))
        if "area_policy_plot" in shape_report:
            manifest_files.append(artifact_dir / str(shape_report["area_policy_plot"]))
    manifest = {
        "schema": "taoryx.alpha3-passive-deployment-artifact-manifest/v1alpha1",
        "qualification_report": _root_relative(output),
        "files": [
            {"path": _root_relative(path), "sha256": _sha256(path), "size_bytes": path.stat().st_size}
            for path in manifest_files
        ],
        "claim_boundary": report["claim_boundary"],
    }
    manifest_path = artifact_dir / "deployment-manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"report": str(output), "manifest": str(manifest_path), "shape_count": len(shape_reports)}, indent=2))
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--artifact-dir", type=Path, default=DEFAULT_ARTIFACT_DIR)
    parser.add_argument("--seed", type=int, default=31_415)
    parser.add_argument("--samples-per-shape", type=int, default=16)
    parser.add_argument("--horizon-s", type=float, default=90.0)
    parser.add_argument("--step-size-s", type=float, default=0.25)
    parser.add_argument("--dpi", type=int, default=120)
    args = parser.parse_args()
    run_qualification(
        args.output,
        args.artifact_dir,
        seed=args.seed,
        samples_per_shape=args.samples_per_shape,
        horizon_s=args.horizon_s,
        step_size_s=args.step_size_s,
        dpi=args.dpi,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
