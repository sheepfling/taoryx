from __future__ import annotations

from pathlib import Path

import numpy as np

from aero_models import (
    sphere_projected_area,
    spheroid_projected_area,
    triaxial_ellipsoid_projected_area,
)
from tbl_writer import write_manifest, write_one_dimensional_table


def cylinder_drag_area(alpha_rad: np.ndarray, k0: float, k90: float) -> np.ndarray:
    c = np.abs(np.cos(alpha_rad))
    s = np.abs(np.sin(alpha_rad))
    return k0 * c**3 + k90 * s**3
####


def cone_drag_area(alpha_rad: np.ndarray, k_nose: float, k_base: float, k90: float) -> np.ndarray:
    c = np.cos(alpha_rad)
    s = np.abs(np.sin(alpha_rad))
    return k_nose * np.maximum(c, 0.0) ** 3 + k_base * np.maximum(-c, 0.0) ** 3 + k90 * s**3
####


def shape_ratio(projected_area: np.ndarray, sref: float) -> np.ndarray:
    if sref <= 0.0:
        raise ValueError("sref must be positive")
    ####
    return projected_area / sref
####


def main() -> None:
    root = Path(__file__).resolve().parent
    generated_root = root / "generated"
    alpha_deg = np.linspace(0.0, 180.0, 37)
    alpha_rad = np.deg2rad(alpha_deg)

    r = 0.5
    cylinder_length = 2.0
    cone_height = 2.0
    spheroid_oblate_axes = (1.2, 0.6)
    spheroid_prolate_axes = (0.6, 1.2)
    triaxial_axes = (1.2, 0.8, 0.5)

    sphere_sref = sphere_projected_area(r)
    cylinder_sref = sphere_sref
    cone_sref = sphere_sref
    spheroid_oblate_sref = sphere_sref
    spheroid_prolate_sref = sphere_sref
    triaxial_sref = np.pi * triaxial_axes[0] * triaxial_axes[1]

    spheroid_oblate_proj = spheroid_projected_area(spheroid_oblate_axes[0], spheroid_oblate_axes[1], alpha_rad)
    spheroid_prolate_proj = spheroid_projected_area(spheroid_prolate_axes[0], spheroid_prolate_axes[1], alpha_rad)
    triaxial_direction = np.stack([np.sin(alpha_rad), np.zeros_like(alpha_rad), np.cos(alpha_rad)], axis=1)
    triaxial_proj = np.array(
        [triaxial_ellipsoid_projected_area(triaxial_axes, direction) for direction in triaxial_direction],
        dtype=np.float64,
    )

    cylinder_cd = cylinder_drag_area(alpha_rad, 1.2, 0.7)
    cone_cd = cone_drag_area(alpha_rad, 1.5, 2.3, 0.8)
    sphere_cd = np.full_like(alpha_rad, 0.47)

    write_one_dimensional_table(
        generated_root / "sphere_cd.tbl",
        title="sphere-cd",
        table_type="cd",
        axis_name="alpha_deg",
        value_name="cd",
        axis_values=alpha_deg,
        value_values=sphere_cd,
        sref=sphere_sref,
    )
    write_one_dimensional_table(
        generated_root / "cylinder_cd.tbl",
        title="cylinder-cd",
        table_type="cd",
        axis_name="alpha_deg",
        value_name="cd",
        axis_values=alpha_deg,
        value_values=cylinder_cd,
        sref=cylinder_sref,
    )
    write_one_dimensional_table(
        generated_root / "cone_cd.tbl",
        title="cone-cd",
        table_type="cd",
        axis_name="alpha_deg",
        value_name="cd",
        axis_values=alpha_deg,
        value_values=cone_cd,
        sref=cone_sref,
    )
    write_one_dimensional_table(
        generated_root / "spheroid_oblate_cd.tbl",
        title="spheroid-oblate-cd",
        table_type="cd",
        axis_name="alpha_deg",
        value_name="cd",
        axis_values=alpha_deg,
        value_values=shape_ratio(spheroid_oblate_proj, spheroid_oblate_sref),
        sref=spheroid_oblate_sref,
    )
    write_one_dimensional_table(
        generated_root / "spheroid_prolate_cd.tbl",
        title="spheroid-prolate-cd",
        table_type="cd",
        axis_name="alpha_deg",
        value_name="cd",
        axis_values=alpha_deg,
        value_values=shape_ratio(spheroid_prolate_proj, spheroid_prolate_sref),
        sref=spheroid_prolate_sref,
    )
    write_one_dimensional_table(
        generated_root / "triaxial_ellipsoid_cd.tbl",
        title="triaxial-ellipsoid-cd",
        table_type="cd",
        axis_name="alpha_deg",
        value_name="cd",
        axis_values=alpha_deg,
        value_values=shape_ratio(triaxial_proj, triaxial_sref),
        sref=triaxial_sref,
    )

    write_manifest(
        generated_root / "README.md",
        [
            "# Generated Tumbling Tables",
            "",
            "These `.tbl` files are derived from the analysis models in `aero_models.py`.",
            "They use explicit `sref=` normalization so the coefficient curves stay traceable to the chosen reference areas.",
            "",
            "- `sphere_cd.tbl`",
            "- `cylinder_cd.tbl`",
            "- `cone_cd.tbl`",
            "- `spheroid_oblate_cd.tbl`",
            "- `spheroid_prolate_cd.tbl`",
            "- `triaxial_ellipsoid_cd.tbl`",
        ],
    )
    print(generated_root)
####


if __name__ == "__main__":
    main()
####
