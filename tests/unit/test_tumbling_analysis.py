from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
ANALYSIS = ROOT / "analysis" / "tumbling"
if str(ANALYSIS) not in sys.path:
    sys.path.insert(0, str(ANALYSIS))

pytestmark = pytest.mark.table

from aero_models import (
    cone_projected_area,
    cylinder_projected_area,
    sphere_projected_area,
    triaxial_ellipsoid_projected_area,
)
from generate_tbl_data import cone_drag_area, cylinder_drag_area, shape_ratio
from tbl_writer import write_multi_axis_table, write_one_dimensional_table

from taoryx.aero_drag_tables import (
    DragModel,
    drag_coefficient,
    generate_sweep,
    generate_triaxial_sweep,
    periodic_linear_interpolate,
    spherical_direction,
    triaxial_projected_area,
)


def test_projected_area_helpers_match_expected_anchor_values() -> None:
    alpha = np.array([0.0, np.pi / 2.0], dtype=np.float64)

    assert sphere_projected_area(0.5) == pytest.approx(np.pi * 0.25)
    assert cylinder_projected_area(0.5, 2.0, alpha) == pytest.approx((np.pi * 0.25, 2.0))
    assert cone_projected_area(0.5, 2.0, alpha) == pytest.approx((np.pi * 0.25, 1.0))
    assert cylinder_drag_area(alpha, 1.2, 0.7) == pytest.approx((1.2, 0.7))
    assert cone_drag_area(alpha, 1.5, 2.3, 0.8) == pytest.approx((1.5, 0.8))


def test_triaxial_projection_and_ratio_helpers_are_positive_and_normalized() -> None:
    area = triaxial_ellipsoid_projected_area((1.2, 0.8, 0.5), np.array([0.0, 0.0, 1.0], dtype=np.float64))

    assert area == pytest.approx(np.pi * 0.96)
    assert shape_ratio(np.array([2.0, 4.0], dtype=np.float64), 2.0) == pytest.approx((1.0, 2.0))
    with pytest.raises(ValueError, match="positive"):
        shape_ratio(np.array([1.0], dtype=np.float64), 0.0)


def test_table_writers_emit_a_traceable_taos_deck(tmp_path: Path) -> None:
    one_dimensional = tmp_path / "sphere_cd.tbl"
    write_one_dimensional_table(
        one_dimensional,
        title="sphere-cd",
        table_type="cd",
        axis_name="alpha_deg",
        value_name="cd",
        axis_values=(0.0, 90.0),
        value_values=(0.47, 0.47),
        sref=0.7853981633974483,
    )

    assert one_dimensional.read_text(encoding="utf-8") == (
        "(sphere-cd)\n"
        "  table cd(alpha_deg) no-extrap sref=0.785398\n"
        "  alpha_deg = 0, 90\n"
        "  cd = 0.47, 0.47\n"
    )

    multi_axis = tmp_path / "history.tbl"
    write_multi_axis_table(
        multi_axis,
        title="history",
        table_type="output",
        axis_names=("time_s", "channel_id"),
        axis_values=((0.0, 1.0), (0.0, 1.0)),
        value_name="output",
        value_values=(10.0, 11.0, 20.0, 21.0),
    )

    assert multi_axis.read_text(encoding="utf-8") == (
        "(history)\n"
        "  table output(time_s,channel_id) no-extrap\n"
        "  time_s = 0, 1\n"
        "  channel_id = 0, 1\n"
        "  output = 10, 11, 20, 21\n"
    )
####


def test_full_angle_models_preserve_geometry_and_front_rear_distinction() -> None:
    angles = np.deg2rad(np.array([0.0, 90.0, 180.0]))

    model = DragModel(
        "cone",
        "cone",
        1.0,
        {
            "radius_m": 0.5,
            "height_m": 2.0,
            "cd_front": 0.3,
            "cd_broadside": 1.1,
            "cd_rear": 1.25,
        },
    )
    assert drag_coefficient(model, np.rad2deg(angles), np.ones(3)) == pytest.approx((0.3, 1.1, 1.25))
    assert generate_sweep(model).drag_area_m2.shape == generate_sweep(model).cd.shape
####


def test_ellipsoid_angle_pair_is_normalized_and_periodic_table_wraps() -> None:
    directions = spherical_direction(
        np.array([0.0, 90.0]),
        np.array([0.0, 90.0]),
    )
    assert np.linalg.norm(np.stack(directions, axis=-1), axis=-1) == pytest.approx((1.0, 1.0))
    assert triaxial_projected_area(
        np.array([0.0]),
        np.array([0.0]),
        (1.0, 0.6, 0.4),
    )[0] == pytest.approx(np.pi * 0.6 * 0.4)

    values = (
        periodic_linear_interpolate(-10.0, np.array([0.0, 90.0, 180.0, 270.0]), np.array([1.0, 2.0, 3.0, 4.0])),
        periodic_linear_interpolate(370.0, np.array([0.0, 90.0, 180.0, 270.0]), np.array([1.0, 2.0, 3.0, 4.0])),
    )
    assert values == pytest.approx((4.0 - 80.0 * 3.0 / 90.0, 1.0 + 10.0 / 90.0))


def test_triaxial_sweep_uses_a_full_two_angle_grid_and_wraps_phi() -> None:
    model = DragModel(
        "triaxial-demo",
        "triaxial",
        1.0,
        {"a_m": 1.2, "b_m": 0.8, "c_m": 0.5},
    )

    alpha, phi, area, cd = generate_triaxial_sweep(model, step_deg=90.0)

    assert alpha.tolist() == [0.0, 90.0, 180.0]
    assert phi.tolist() == [0.0, 90.0, 180.0, 270.0, 360.0]
    assert area.shape == (3, 5)
    assert cd.shape == (3, 5)
    assert np.allclose(area[:, 0], area[:, -1])
    assert area[0, 0] == pytest.approx(np.pi * 0.8 * 0.5)
    assert area[1, 0] == pytest.approx(np.pi * 1.2 * 0.8)
    assert area[1, 1] == pytest.approx(np.pi * 1.2 * 0.5)
####
