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
    cone_projected_area,
    cylinder_projected_area,
    sphere_projected_area,
    spheroid_projected_area,
    triaxial_ellipsoid_projected_area,
)


def main() -> None:
    alpha_deg = np.linspace(0.0, 180.0, 721)
    alpha_rad = np.deg2rad(alpha_deg)

    sphere_area = sphere_projected_area(0.5)
    spheroid_oblate = spheroid_projected_area(1.2, 0.6, alpha_rad)
    spheroid_prolate = spheroid_projected_area(0.6, 1.2, alpha_rad)

    triaxial_alpha = np.deg2rad(alpha_deg)
    triaxial_direction = np.stack(
        [np.sin(triaxial_alpha), np.zeros_like(triaxial_alpha), np.cos(triaxial_alpha)],
        axis=1,
    )
    triaxial_area = np.array(
        [triaxial_ellipsoid_projected_area((1.2, 0.8, 0.5), direction) for direction in triaxial_direction],
        dtype=np.float64,
    )

    cylinder_area = cylinder_projected_area(0.5, 2.0, alpha_rad)
    cone_area = cone_projected_area(0.5, 2.0, alpha_rad)

    figure, axes = plt.subplots(2, 1, figsize=(9.0, 8.0), sharex=True, layout="constrained")

    axes[0].plot(alpha_deg, np.full_like(alpha_deg, sphere_area), label="Sphere", linewidth=2.2)
    axes[0].plot(alpha_deg, spheroid_oblate, label="Oblate spheroid (a=1.2, b=0.6)", linewidth=2.2)
    axes[0].plot(alpha_deg, spheroid_prolate, label="Prolate spheroid (a=0.6, b=1.2)", linewidth=2.2)
    axes[0].set_title("Sphere and spheroid projected-area sweeps")
    axes[0].set_ylabel("Projected area")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend()

    axes[1].plot(alpha_deg, triaxial_area, label="Triaxial ellipsoid", linewidth=2.2)
    axes[1].plot(alpha_deg, cylinder_area, label="Cylinder reference", linewidth=1.8, alpha=0.85)
    axes[1].plot(alpha_deg, cone_area, label="Cone reference", linewidth=1.8, alpha=0.85)
    axes[1].set_title("Triaxial ellipsoid sweep along a body-plane great circle")
    axes[1].set_xlabel("Angle from body axis (deg)")
    axes[1].set_ylabel("Projected area")
    axes[1].grid(True, alpha=0.25)
    axes[1].legend()

    output_dir = Path(__file__).resolve().parent / "artifacts"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "sphere_ellipsoid_sweeps.png"
    figure.savefig(output_path, dpi=180)
    print(output_path)


if __name__ == "__main__":
    main()
