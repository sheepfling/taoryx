from __future__ import annotations

import os
import tempfile
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "matplotlib-cache"))

import matplotlib

matplotlib.use("Agg", force=True)
import matplotlib.pyplot as plt
import numpy as np

from aero_models import (
    cone_isotropic_mean_projected_area,
    cylinder_isotropic_mean_projected_area,
    ellipsoid_isotropic_mean_projected_area,
    sphere_isotropic_mean_projected_area,
)


def knud_thomsen_surface_area(a: float, b: float, c: float) -> float:
    p = 1.6075
    return float(4.0 * np.pi * ((a**p * b**p + a**p * c**p + b**p * c**p) / 3.0) ** (1.0 / p))


def main() -> None:
    radius = 0.5
    length = 2.0
    height = 2.0
    ellipsoid_axes = (1.2, 0.8, 0.5)

    labels = ["Sphere", "Cylinder", "Cone", "Ellipsoid"]
    values = [
        sphere_isotropic_mean_projected_area(radius),
        cylinder_isotropic_mean_projected_area(radius, length),
        cone_isotropic_mean_projected_area(radius, height),
        ellipsoid_isotropic_mean_projected_area(knud_thomsen_surface_area(*ellipsoid_axes)),
    ]

    reference_area = np.pi * radius**2
    normalized = np.array(values) / reference_area

    figure, axis = plt.subplots(figsize=(9.0, 4.8), layout="constrained")
    bars = axis.bar(labels, normalized, color=["#1d4ed8", "#0f766e", "#b45309", "#7c3aed"])
    axis.set_title("Isotropic mean projected area, normalized by a sphere reference area")
    axis.set_ylabel("Mean projected area / sphere reference area")
    axis.grid(True, axis="y", alpha=0.25)
    axis.set_axisbelow(True)

    for bar, value in zip(bars, normalized, strict=True):
        axis.text(bar.get_x() + bar.get_width() / 2.0, value + 0.03, f"{value:.2f}", ha="center", va="bottom")

    output_dir = Path(__file__).resolve().parent / "artifacts"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "isotropic_mean_projected_area.png"
    figure.savefig(output_path, dpi=180)
    print(output_path)


if __name__ == "__main__":
    main()
