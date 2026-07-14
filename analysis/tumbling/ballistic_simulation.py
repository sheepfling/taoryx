from __future__ import annotations

import os
import math
import tempfile
from dataclasses import dataclass
from collections.abc import Sequence
from pathlib import Path
import sys

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

from taoryx.language.table_parser import parse_table_file
from taoryx.runtime.lowering import lower_tables

from aero_models import sphere_projected_area
from tbl_writer import write_multi_axis_table


HISTORY_CHANNELS: tuple[tuple[int, str], ...] = (
    (0, "altitude_m"),
    (1, "speed_mps"),
    (2, "flight_path_angle_deg"),
    (3, "dynamic_pressure_pa"),
    (4, "alpha_deg"),
    (5, "cd"),
)


@dataclass(frozen=True, slots=True)
class LaunchCase:
    name: str
    initial_speed_mps: float
    flight_path_angle_deg: float
    altitude_m: float = 10_000.0
    mass_kg: float = 100.0
    reference_radius_m: float = 0.5
    tumble_mean_deg: float = 45.0
    tumble_amplitude_deg: float = 35.0
    tumble_period_s: float = 18.0
####


@dataclass(frozen=True, slots=True)
class TrajectorySample:
    time_s: float
    altitude_m: float
    speed_mps: float
    flight_path_angle_deg: float
    dynamic_pressure_pa: float
    alpha_deg: float
    cd: float
####


def load_cd_table(table_path: Path):
    document = parse_table_file(table_path)
    runtime_tables = lower_tables(document)
    if not runtime_tables:
        raise ValueError(f"no tables found in {table_path}")
    ####
    return next(iter(runtime_tables.values()))
####


def standard_density(altitude_m: float) -> float:
    sea_level_density = 1.225
    scale_height = 7_500.0
    return float(sea_level_density * math.exp(-max(altitude_m, 0.0) / scale_height))
####


def gravity(altitude_m: float) -> float:
    earth_radius = 6_371_000.0
    g0 = 9.80665
    return float(g0 * (earth_radius / (earth_radius + max(altitude_m, 0.0))) ** 2)
####


def tumble_angle(case: LaunchCase, time_s: float) -> float:
    raw = case.tumble_mean_deg + case.tumble_amplitude_deg * math.sin(2.0 * math.pi * time_s / case.tumble_period_s)
    return float(min(max(raw, 0.0), 180.0))
####


def dynamics(case: LaunchCase, cd_table, time_s: float, state: np.ndarray) -> np.ndarray:
    altitude_m, speed_mps, flight_path_angle_rad = state
    rho = standard_density(altitude_m)
    q_inf = 0.5 * rho * speed_mps**2
    alpha_deg = tumble_angle(case, time_s)
    cd = cd_table.evaluate({"alpha_deg": alpha_deg})
    drag = q_inf * cd * sphere_projected_area(case.reference_radius_m)
    g = gravity(altitude_m)
    h_dot = speed_mps * math.sin(flight_path_angle_rad)
    v_dot = -(drag / case.mass_kg) - g * math.sin(flight_path_angle_rad)
    gamma_dot = -(g * math.cos(flight_path_angle_rad)) / max(speed_mps, 1.0)
    return np.array([h_dot, v_dot, gamma_dot], dtype=np.float64)
####


def simulate_case(
    case: LaunchCase,
    cd_table,
    *,
    dt: float = 0.1,
    max_time_s: float = 240.0,
) -> list[TrajectorySample]:
    if dt <= 0.0 or max_time_s <= 0.0:
        raise ValueError("dt and max_time_s must be positive")
    ####
    state = np.array([case.altitude_m, case.initial_speed_mps, math.radians(case.flight_path_angle_deg)], dtype=np.float64)
    time_s = 0.0
    samples: list[TrajectorySample] = []
    while time_s <= max_time_s and state[0] >= 0.0 and state[1] > 0.0:
        alpha_deg = tumble_angle(case, time_s)
        rho = standard_density(float(state[0]))
        q_inf = 0.5 * rho * float(state[1]) ** 2
        cd = cd_table.evaluate({"alpha_deg": alpha_deg})
        samples.append(
            TrajectorySample(
                time_s=float(time_s),
                altitude_m=float(state[0]),
                speed_mps=float(state[1]),
                flight_path_angle_deg=math.degrees(float(state[2])),
                dynamic_pressure_pa=float(q_inf),
                alpha_deg=float(alpha_deg),
                cd=float(cd),
            )
        )
        ####
        k1 = dynamics(case, cd_table, time_s, state)
        k2 = dynamics(case, cd_table, time_s + 0.5 * dt, state + 0.5 * dt * k1)
        k3 = dynamics(case, cd_table, time_s + 0.5 * dt, state + 0.5 * dt * k2)
        k4 = dynamics(case, cd_table, time_s + dt, state + dt * k3)
        next_state = state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        time_s += dt
        if next_state[0] < 0.0:
            next_state[0] = 0.0
            rho = standard_density(float(next_state[0]))
            q_inf = 0.5 * rho * float(next_state[1]) ** 2
            alpha_deg = tumble_angle(case, time_s)
            cd = cd_table.evaluate({"alpha_deg": alpha_deg})
            samples.append(
                TrajectorySample(
                    time_s=float(time_s),
                    altitude_m=float(next_state[0]),
                    speed_mps=float(next_state[1]),
                    flight_path_angle_deg=math.degrees(float(next_state[2])),
                    dynamic_pressure_pa=float(q_inf),
                    alpha_deg=float(alpha_deg),
                    cd=float(cd),
                )
            )
            break
        ####
        state = next_state
    ####
    return samples
####


def write_history_table(path: Path, samples: Sequence[TrajectorySample]) -> None:
    time_axis = [sample.time_s for sample in samples]
    channel_axis = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    values: list[float] = []
    for sample in samples:
        values.extend(
            [
                sample.altitude_m,
                sample.speed_mps,
                sample.flight_path_angle_deg,
                sample.dynamic_pressure_pa,
                sample.alpha_deg,
                sample.cd,
            ]
        )
    ####
    write_multi_axis_table(
        path,
        title="ballistic-cone-history",
        table_type="output",
        axis_names=("time_s", "channel_id"),
        axis_values=(time_axis, channel_axis),
        value_name="output",
        value_values=values,
        precision=6,
    )
####


def write_history_readme(path: Path, case: LaunchCase) -> None:
    lines = [
        "# Ballistic Cone History",
        "",
        f"Reference case: `{case.name}`",
        "",
        "Channel mapping:",
    ]
    lines.extend(f"- `{index}` = `{name}`" for index, name in HISTORY_CHANNELS)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
####


def plot_history(case: LaunchCase, samples: Sequence[TrajectorySample], output_path: Path) -> None:
    times = np.array([sample.time_s for sample in samples], dtype=np.float64)
    altitude = np.array([sample.altitude_m for sample in samples], dtype=np.float64)
    speed = np.array([sample.speed_mps for sample in samples], dtype=np.float64)
    dynamic_pressure = np.array([sample.dynamic_pressure_pa for sample in samples], dtype=np.float64)
    angle = np.array([sample.alpha_deg for sample in samples], dtype=np.float64)
    cd = np.array([sample.cd for sample in samples], dtype=np.float64)

    figure, axes = plt.subplots(4, 1, figsize=(9.0, 10.0), sharex=True, layout="constrained")
    axes[0].plot(times, altitude / 1000.0, color="#0f766e", linewidth=2.2)
    axes[0].set_ylabel("Altitude (km)")
    axes[0].set_title(f"Ballistic cone launch: {case.name}")
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(times, speed, color="#1d4ed8", linewidth=2.2)
    axes[1].set_ylabel("Speed (m/s)")
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(times, dynamic_pressure / 1000.0, color="#b45309", linewidth=2.2)
    axes[2].set_ylabel("Dyn. press. (kPa)")
    axes[2].grid(True, alpha=0.25)

    axes[3].plot(times, angle, label="tumble angle", color="#7c3aed", linewidth=2.0)
    axes[3].plot(times, cd, label="C_D", color="#475569", linewidth=1.8)
    axes[3].set_xlabel("Time (s)")
    axes[3].set_ylabel("Angle / C_D")
    axes[3].grid(True, alpha=0.25)
    axes[3].legend()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180)
####


def simulate_and_render(case: LaunchCase, cd_table, *, history_path: Path, plot_path: Path) -> list[TrajectorySample]:
    samples = simulate_case(case, cd_table)
    write_history_table(history_path, samples)
    write_history_readme(history_path.with_suffix(".md"), case)
    plot_history(case, samples, plot_path)
    return samples
####
