from __future__ import annotations

import os
import tempfile
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "matplotlib-cache"))

import matplotlib

matplotlib.use("Agg", force=True)
import matplotlib.pyplot as plt
import numpy as np

from aero_models import cone_projected_area, cylinder_projected_area


def cylinder_drag_area(alpha_rad: np.ndarray, k0: float, k90: float) -> np.ndarray:
    c = np.abs(np.cos(alpha_rad))
    s = np.abs(np.sin(alpha_rad))
    return k0 * c**3 + k90 * s**3


def cone_drag_area(alpha_rad: np.ndarray, k_nose: float, k_base: float, k90: float) -> np.ndarray:
    c = np.cos(alpha_rad)
    s = np.abs(np.sin(alpha_rad))
    return k_nose * np.maximum(c, 0.0) ** 3 + k_base * np.maximum(-c, 0.0) ** 3 + k90 * s**3


def main() -> None:
    alpha_deg = np.linspace(0.0, 180.0, 721)
    alpha_rad = np.deg2rad(alpha_deg)

    cylinder_proj = cylinder_projected_area(0.5, 2.0, alpha_rad)
    cylinder_drag = cylinder_drag_area(alpha_rad, 1.2, 0.7)

    cone_proj = cone_projected_area(0.5, 2.0, alpha_rad)
    cone_drag = cone_drag_area(alpha_rad, 1.5, 2.3, 0.8)

    figure, axes = plt.subplots(2, 1, figsize=(9.0, 7.2), sharex=True, layout="constrained")

    axes[0].plot(alpha_deg, cylinder_proj, label="Projected area", linewidth=2.2)
    axes[0].plot(alpha_deg, cylinder_drag, label="Drag-area model", linewidth=2.2)
    axes[0].set_title("Cylinder: projected area vs. drag-area model")
    axes[0].set_ylabel("Area / drag area")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend()

    axes[1].plot(alpha_deg, cone_proj, label="Projected area", linewidth=2.2)
    axes[1].plot(alpha_deg, cone_drag, label="Drag-area model", linewidth=2.2)
    axes[1].set_title("Cone: projected area vs. drag-area model")
    axes[1].set_xlabel("Angle from body axis (deg)")
    axes[1].set_ylabel("Area / drag area")
    axes[1].grid(True, alpha=0.25)
    axes[1].legend()

    output_dir = Path(__file__).resolve().parent / "artifacts"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "drag_area_models.png"
    figure.savefig(output_path, dpi=180)
    print(output_path)


if __name__ == "__main__":
    main()
