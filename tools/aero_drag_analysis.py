#!/usr/bin/env python3
"""Generate full-angle drag-area data, plots, and TAOS coefficient tables."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg", force=True)
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ANALYSIS = ROOT / "analysis" / "tumbling"
SRC = ROOT / "src"
if str(ANALYSIS) not in sys.path:
    sys.path.insert(0, str(ANALYSIS))
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
####

from tbl_writer import write_multi_axis_table, write_one_dimensional_table

from taoryx.aero_drag_tables import (
    DragModel,
    generate_sweep,
    generate_triaxial_sweep,
    write_sweep_csv,
)

DEFAULT_MODELS: dict[str, DragModel] = {
    "sphere": DragModel("sphere", "sphere", np.pi * 0.5**2, {"radius_m": 0.5, "cd": 0.47}),
    "prolate-spheroid": DragModel("prolate-spheroid", "spheroid", np.pi * 0.5**2, {"axial_semi_axis_m": 1.0, "transverse_semi_axis_m": 0.5}),
    "cylinder": DragModel("cylinder", "cylinder", np.pi * 0.5**2, {"radius_m": 0.5, "length_m": 2.0, "cd_front": 1.0, "cd_broadside": 1.1, "cd_rear": 1.0}),
    "cone": DragModel("cone", "cone", np.pi * 0.5**2, {"radius_m": 0.5, "height_m": 2.0, "cd_front": 0.3, "cd_broadside": 1.1, "cd_rear": 1.25}),
    "triaxial-ellipsoid": DragModel("triaxial-ellipsoid", "triaxial", np.pi * 0.6 * 0.4, {"a_m": 1.0, "b_m": 0.6, "c_m": 0.4}),
}


def load_model(path: Path | None, shape: str) -> DragModel:
    if path is None:
        return DEFAULT_MODELS[shape]
    data = json.loads(path.read_text(encoding="utf-8"))
    name = str(data.get("name", shape))
    model_shape = str(data["shape"])
    reference_area = float(data["reference_area_m2"])
    parameters = {str(key): float(value) for key, value in data.get("parameters", {}).items()}
    return DragModel(name, model_shape, reference_area, parameters)
####


def plot_sweep(model: DragModel, output_dir: Path) -> None:
    sweep = generate_sweep(model)
    figure, axes = plt.subplots(1, 2, figsize=(12.0, 4.8), layout="constrained")
    axes[0].plot(sweep.angle_deg, sweep.projected_area_m2, color="#0f766e", linewidth=2.0)
    axes[0].set(xlabel="Orientation angle (deg)", ylabel="Projected area (m2)")
    axes[1].plot(sweep.angle_deg, sweep.drag_area_m2, color="#b45309", linewidth=2.0, label="K_D")
    axes[1].plot(sweep.angle_deg, sweep.cd, color="#2563eb", linewidth=1.8, label="C_D")
    axes[1].set(xlabel="Orientation angle (deg)", ylabel="Drag area / coefficient")
    axes[1].legend()
    for axis in axes:
        axis.grid(True, alpha=0.25)
    output_dir.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_dir / f"{model.name}-full-angle.png", dpi=180)
    plt.close(figure)
####


def write_axisymmetric_outputs(model: DragModel, output_dir: Path) -> None:
    sweep = generate_sweep(model)
    csv_path = output_dir / f"{model.name}.csv"
    write_sweep_csv(csv_path, sweep)
    write_one_dimensional_table(
        output_dir / f"{model.name}.tbl",
        title=f"{model.name}-cd",
        table_type="cd",
        axis_name="alphat",
        value_name="cd",
        axis_values=sweep.angle_deg[:37],
        value_values=sweep.cd[:37],
        sref=model.reference_area_m2,
        precision=10,
    )
    plot_sweep(model, output_dir)
####


def write_triaxial_outputs(model: DragModel, output_dir: Path) -> None:
    alpha, phi, area, cd = generate_triaxial_sweep(model)
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / f"{model.name}.csv").open("w", encoding="utf-8") as handle:
        handle.write("alphat_deg,phi_deg,projected_area_m2,cd\n")
        for i, alpha_value in enumerate(alpha):
            for j, phi_value in enumerate(phi):
                handle.write(f"{alpha_value},{phi_value},{area[i, j]},{cd[i, j]}\n")
    ####
    write_multi_axis_table(
        output_dir / f"{model.name}.tbl",
        title=f"{model.name}-cd",
        table_type="cd",
        axis_names=("alphat", "phi"),
        axis_values=(alpha, phi),
        value_name="cd",
        value_values=cd.ravel(),
        sref=model.reference_area_m2,
        precision=8,
    )
    figure, axis = plt.subplots(figsize=(8.0, 5.5), layout="constrained")
    image = axis.pcolormesh(phi, alpha, cd, shading="auto", cmap="viridis")
    axis.set(xlabel="Windward meridian phi (deg)", ylabel="Total angle alphat (deg)")
    figure.colorbar(image, ax=axis, label="C_D")
    figure.savefig(output_dir / f"{model.name}-orientation-map.png", dpi=180)
    plt.close(figure)
####


def write_comparison_outputs(models: list[DragModel], output_dir: Path) -> None:
    """Write the cross-shape comparison figures from the analysis dump."""

    planar_models = [model for model in models if model.shape != "triaxial"]
    sweeps = [(model, generate_sweep(model)) for model in planar_models]
    output_dir.mkdir(parents=True, exist_ok=True)

    figure, axis = plt.subplots(figsize=(9.0, 5.5), layout="constrained")
    for model, sweep in sweeps:
        axis.plot(sweep.angle_deg, sweep.cd, linewidth=1.8, label=model.name)
    axis.set(
        xlabel="Orientation angle (deg)",
        ylabel="Normalized drag coefficient / projected-area surrogate",
        title="Full-angle normalized drag comparison",
    )
    axis.grid(True, alpha=0.25)
    axis.legend()
    figure.savefig(output_dir / "normalized-full-angle-comparison.png", dpi=180)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(7.0, 7.0), subplot_kw={"projection": "polar"}, layout="constrained")
    for model, sweep in sweeps:
        axis.plot(np.deg2rad(sweep.angle_deg), sweep.cd, linewidth=1.8, label=model.name)
    axis.set_title("Planar tumble drag signatures", pad=20.0)
    axis.legend(loc="upper right", bbox_to_anchor=(1.25, 1.15))
    figure.savefig(output_dir / "normalized-full-angle-polar.png", dpi=180)
    plt.close(figure)

    summary = []
    for model, sweep in sweeps:
        summary.append(
            {
                "name": model.name,
                "shape": model.shape,
                "max_cd": float(np.max(sweep.cd)),
                "mean_cd": float(np.mean(sweep.cd)),
                "reference_area_m2": model.reference_area_m2,
            }
        )
    (output_dir / "normalized-comparison-summary.json").write_text(
        json.dumps(summary, indent=2) + "\n",
        encoding="utf-8",
    )
####


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config_positional", nargs="?", type=Path, help="optional JSON model configuration")
    parser.add_argument("--config", dest="config_option", type=Path, help="JSON model configuration")
    parser.add_argument("--shape", choices=sorted(DEFAULT_MODELS), default="cone")
    parser.add_argument("--all", action="store_true", help="generate all default models and comparison/gallery plots")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "build" / "aero-drag")
    args = parser.parse_args()
    config = args.config_option if args.config_option is not None else args.config_positional
    if args.all and config is not None:
        parser.error("--all cannot be combined with a model configuration")
    if args.all:
        models = list(DEFAULT_MODELS.values())
        for model in models:
            model_output = args.output_dir / model.name
            if model.shape == "triaxial":
                write_triaxial_outputs(model, model_output)
            else:
                write_axisymmetric_outputs(model, model_output)
        write_comparison_outputs(models, args.output_dir)
        print(args.output_dir)
        return
    ####
    model = load_model(config, args.shape)
    if model.shape == "triaxial":
        write_triaxial_outputs(model, args.output_dir)
    else:
        write_axisymmetric_outputs(model, args.output_dir)
    (args.output_dir / f"{model.name}-summary.json").write_text(
        json.dumps({"name": model.name, "shape": model.shape, "reference_area_m2": model.reference_area_m2, "parameters": model.parameters}, indent=2) + "\n",
        encoding="utf-8",
    )
    print(args.output_dir)
####


if __name__ == "__main__":
    main()
