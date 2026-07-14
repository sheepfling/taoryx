from __future__ import annotations

import math
import os
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from collections.abc import Sequence

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
####

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "matplotlib-cache"))

import matplotlib

matplotlib.use("Agg", force=True)
import matplotlib.pyplot as plt
import numpy as np

from ballistic_simulation import gravity, load_cd_table, standard_density, tumble_angle
from tbl_writer import write_multi_axis_table


HISTORY_CHANNELS: tuple[tuple[int, str], ...] = (
    (0, "downrange_m"),
    (1, "altitude_m"),
    (2, "speed_mps"),
    (3, "flight_path_angle_deg"),
    (4, "dynamic_pressure_pa"),
    (5, "alpha_deg"),
    (6, "cd"),
)


@dataclass(frozen=True, slots=True)
class ShapeCase:
    name: str
    table_name: str
    reference_area_m2: float
    color: str
####


@dataclass(frozen=True, slots=True)
class ShapeSample:
    time_s: float
    downrange_m: float
    altitude_m: float
    speed_mps: float
    flight_path_angle_deg: float
    dynamic_pressure_pa: float
    alpha_deg: float
    cd: float
####


@dataclass(frozen=True, slots=True)
class BatchLaunch:
    initial_speed_mps: float = 600.0
    flight_path_angle_deg: float = 20.0
    altitude_m: float = 10_000.0
    mass_kg: float = 100.0
    reference_radius_m: float = 0.5
    tumble_mean_deg: float = 45.0
    tumble_amplitude_deg: float = 35.0
    tumble_period_s: float = 18.0
####


def _dynamics(
    launch: BatchLaunch,
    shape: ShapeCase,
    cd_table,
    time_s: float,
    state: np.ndarray,
) -> np.ndarray:
    downrange_m, altitude_m, speed_mps, flight_path_angle_rad = state
    rho = standard_density(float(altitude_m))
    q_inf = 0.5 * rho * speed_mps**2
    alpha_deg = tumble_angle(launch, time_s)
    cd = float(cd_table.evaluate({"alpha_deg": alpha_deg}))
    drag = q_inf * cd * shape.reference_area_m2
    g = gravity(float(altitude_m))
    return np.array(
        [
            speed_mps * math.cos(flight_path_angle_rad),
            speed_mps * math.sin(flight_path_angle_rad),
            -(drag / launch.mass_kg) - g * math.sin(flight_path_angle_rad),
            -(g * math.cos(flight_path_angle_rad)) / max(speed_mps, 1.0),
        ],
        dtype=np.float64,
    )
####


def simulate_shape(
    launch: BatchLaunch,
    shape: ShapeCase,
    cd_table,
    *,
    dt: float = 0.1,
    max_time_s: float = 240.0,
) -> list[ShapeSample]:
    if dt <= 0.0 or max_time_s <= 0.0:
        raise ValueError("dt and max_time_s must be positive")
    ####
    state = np.array(
        [0.0, launch.altitude_m, launch.initial_speed_mps, math.radians(launch.flight_path_angle_deg)],
        dtype=np.float64,
    )
    samples: list[ShapeSample] = []
    time_s = 0.0
    while time_s <= max_time_s and state[1] >= 0.0 and state[2] > 0.0:
        alpha_deg = tumble_angle(launch, time_s)
        rho = standard_density(float(state[1]))
        q_inf = 0.5 * rho * float(state[2]) ** 2
        cd = float(cd_table.evaluate({"alpha_deg": alpha_deg}))
        samples.append(
            ShapeSample(
                time_s=float(time_s),
                downrange_m=float(state[0]),
                altitude_m=float(state[1]),
                speed_mps=float(state[2]),
                flight_path_angle_deg=math.degrees(float(state[3])),
                dynamic_pressure_pa=float(q_inf),
                alpha_deg=float(alpha_deg),
                cd=cd,
            ),
        )
        ####
        k1 = _dynamics(launch, shape, cd_table, time_s, state)
        k2 = _dynamics(launch, shape, cd_table, time_s + 0.5 * dt, state + 0.5 * dt * k1)
        k3 = _dynamics(launch, shape, cd_table, time_s + 0.5 * dt, state + 0.5 * dt * k2)
        k4 = _dynamics(launch, shape, cd_table, time_s + dt, state + dt * k3)
        next_state = state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        time_s += dt
        if next_state[1] < 0.0:
            next_state[1] = 0.0
            terminal_alpha_deg = tumble_angle(launch, time_s)
            terminal_rho = standard_density(float(next_state[1]))
            terminal_q_inf = 0.5 * terminal_rho * float(next_state[2]) ** 2
            terminal_cd = float(cd_table.evaluate({"alpha_deg": terminal_alpha_deg}))
            samples.append(
                ShapeSample(
                    time_s=float(time_s),
                    downrange_m=float(next_state[0]),
                    altitude_m=0.0,
                    speed_mps=float(next_state[2]),
                    flight_path_angle_deg=math.degrees(float(next_state[3])),
                    dynamic_pressure_pa=float(terminal_q_inf),
                    alpha_deg=float(terminal_alpha_deg),
                    cd=terminal_cd,
                ),
            )
            break
        ####
        state = next_state
    ####
    return samples
####


def write_shape_history(path: Path, samples: Sequence[ShapeSample], shape: ShapeCase) -> None:
    values: list[float] = []
    for sample in samples:
        values.extend(
            [
                sample.downrange_m,
                sample.altitude_m,
                sample.speed_mps,
                sample.flight_path_angle_deg,
                sample.dynamic_pressure_pa,
                sample.alpha_deg,
                sample.cd,
            ],
        )
    ####
    write_multi_axis_table(
        path,
        title=f"{shape.name.replace(' ', '-')}-ballistic-history",
        table_type="output",
        axis_names=("time_s", "channel_id"),
        axis_values=(tuple(sample.time_s for sample in samples), tuple(float(i) for i, _ in HISTORY_CHANNELS)),
        value_name="output",
        value_values=values,
        precision=6,
    )
####


def plot_comparison(
    launch: BatchLaunch,
    shapes: Sequence[ShapeCase],
    histories: dict[str, Sequence[ShapeSample]],
    output_path: Path,
) -> None:
    figure, axes = plt.subplots(3, 2, figsize=(12.0, 12.0), layout="constrained")
    plot_specs = (
        (axes[0, 0], "downrange_m", "Downrange (km)", lambda s: s.downrange_m / 1000.0),
        (axes[0, 1], "altitude_m", "Altitude (km)", lambda s: s.altitude_m / 1000.0),
        (axes[1, 0], "flight_path_angle_deg", "Flight-path angle (deg)", lambda s: s.flight_path_angle_deg),
        (axes[1, 1], "alpha_deg", "Body orientation, alpha (deg)", lambda s: s.alpha_deg),
        (axes[2, 0], "speed_mps", "Speed (m/s)", lambda s: s.speed_mps),
        (axes[2, 1], "cd", "Drag coefficient, C_D", lambda s: s.cd),
    )
    for axis, _, ylabel, transform in plot_specs:
        for shape in shapes:
            samples = histories[shape.name]
            times = [sample.time_s for sample in samples]
            axis.plot(times, [transform(sample) for sample in samples], color=shape.color, label=shape.name, linewidth=2.0)
        axis.set_xlabel("Time (s)")
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.25)
    axes[0, 0].set_title(
        f"Four tumbling ballistic bodies | V0={launch.initial_speed_mps:.0f} m/s, "
        f"gamma0={launch.flight_path_angle_deg:.0f} deg",
    )
    axes[0, 1].legend(loc="best")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)
####


def main() -> None:
    root = Path(__file__).resolve().parent
    generated = root / "generated" / "four_shape_histories"
    artifacts = root / "artifacts"
    launch = BatchLaunch()
    shapes = (
        ShapeCase("sphere", "sphere_cd.tbl", math.pi * 0.5**2, "#0f766e"),
        ShapeCase("cylinder", "cylinder_cd.tbl", math.pi * 0.5**2, "#2563eb"),
        ShapeCase("cone", "cone_cd.tbl", math.pi * 0.5**2, "#b45309"),
        ShapeCase("triaxial ellipsoid", "triaxial_ellipsoid_cd.tbl", math.pi * 1.2 * 0.8, "#7c3aed"),
    )
    histories: dict[str, list[ShapeSample]] = {}
    for shape in shapes:
        table = load_cd_table(root / "generated" / shape.table_name)
        samples = simulate_shape(launch, shape, table)
        histories[shape.name] = samples
        write_shape_history(generated / f"{shape.name.replace(' ', '_')}.tbl", samples, shape)
    ####
    plot_comparison(launch, shapes, histories, artifacts / "four_shape_ballistic_comparison.png")
    (generated / "README.md").write_text(
        "# Four-Shape Ballistic Histories\n\n"
        "All histories use the same 10 km, 600 m/s, 20 deg launch and 100 kg mass.\n\n"
        "Channels: 0 downrange_m, 1 altitude_m, 2 speed_mps, 3 flight_path_angle_deg, "
        "4 dynamic_pressure_pa, 5 alpha_deg, 6 cd.\n",
        encoding="utf-8",
    )
    print(artifacts / "four_shape_ballistic_comparison.png")
####


if __name__ == "__main__":
    main()
