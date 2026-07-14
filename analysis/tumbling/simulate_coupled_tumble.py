from __future__ import annotations

import argparse
import math
import os
import sys
import tempfile
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))
####

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "matplotlib-cache"))

import matplotlib

matplotlib.use("Agg", force=True)
import matplotlib.pyplot as plt
import numpy as np
from aero_drag_analysis import DEFAULT_MODELS
from ballistic_simulation import gravity, standard_density
from matplotlib import animation
from tbl_writer import write_multi_axis_table

from taoryx.aero_drag_tables import DragModel, generate_sweep, periodic_linear_interpolate, triaxial_projected_area


@dataclass(frozen=True, slots=True)
class RigidBodyCase:
    model: DragModel
    mass_kg: float
    length_m: float
    cp_offset_m: float
    cm_q: float
    cm_static: float = 0.0
####


@dataclass(frozen=True, slots=True)
class CoupledSample:
    time_s: float
    x_m: float
    z_m: float
    vx_mps: float
    vz_mps: float
    theta_deg: float
    alpha_deg: float
    omega_deg_s: float
    dynamic_pressure_pa: float
    drag_n: float
    torque_cp_nm: float
    torque_damping_nm: float
    torque_static_nm: float
    torque_total_nm: float
####


def _cd_function(model: DragModel) -> tuple[np.ndarray, np.ndarray]:
    angles = np.arange(0.0, 361.0, 5.0, dtype=np.float64)
    if model.shape == "triaxial":
        alpha = angles
        area = triaxial_projected_area(alpha, np.zeros_like(alpha), (model.parameters["a_m"], model.parameters["b_m"], model.parameters["c_m"]))
        return angles, area / model.reference_area_m2
    sweep = generate_sweep(model, angles)
    return sweep.angle_deg, sweep.cd
####


def _aero_terms(case: RigidBodyCase, altitude_m: float, theta_rad: float, vx: float, vz: float, omega: float, tables: tuple[np.ndarray, np.ndarray]) -> tuple[float, float, float, float, float, float]:
    speed = max(math.hypot(vx, vz), 1e-6)
    gamma = math.atan2(vz, vx)
    alpha_deg = math.degrees((theta_rad - gamma) % (2.0 * math.pi))
    axis, cd_values = tables
    cd = periodic_linear_interpolate(alpha_deg, axis, cd_values)
    rho = standard_density(altitude_m)
    q_inf = 0.5 * rho * speed**2
    drag = q_inf * case.model.reference_area_m2 * cd
    fx = -drag * vx / speed
    fz = -drag * vz / speed
    body_x = math.cos(theta_rad)
    body_z = math.sin(theta_rad)
    torque_cp = (case.cp_offset_m * body_x) * fz - (case.cp_offset_m * body_z) * fx
    torque_damping = -q_inf * case.model.reference_area_m2 * case.length_m * case.cm_q * omega * case.length_m / (2.0 * speed)
    torque_static = q_inf * case.model.reference_area_m2 * case.length_m * case.cm_static * math.sin(math.radians(alpha_deg))
    return q_inf, drag, alpha_deg, torque_cp, torque_damping, torque_static
####


def simulate(case: RigidBodyCase, *, dt: float = 0.01, max_time_s: float = 30.0) -> list[CoupledSample]:
    tables = _cd_function(case.model)
    inertia = case.mass_kg * case.length_m**2 / 12.0
    state = np.array([0.0, 250.0, 70.0, -2.0, math.radians(25.0), math.radians(120.0)], dtype=np.float64)
    samples: list[CoupledSample] = []
    time_s = 0.0
    while time_s <= max_time_s and state[1] >= 0.0:
        x_m, z_m, vx, vz, theta, omega = state
        q_inf, drag, alpha_deg, torque_cp, torque_damping, torque_static = _aero_terms(case, z_m, theta, vx, vz, omega, tables)
        samples.append(CoupledSample(time_s, x_m, z_m, vx, vz, math.degrees(theta), alpha_deg, math.degrees(omega), q_inf, drag, torque_cp, torque_damping, torque_static, torque_cp + torque_damping + torque_static))
        ####
        def derivative(value: np.ndarray) -> np.ndarray:
            _, altitude, velocity_x, velocity_z, attitude, rate = value
            q_value, drag_value, _, cp_value, damping_value, static_value = _aero_terms(case, altitude, attitude, velocity_x, velocity_z, rate, tables)
            speed = max(math.hypot(velocity_x, velocity_z), 1e-6)
            return np.array(
                [velocity_x, velocity_z, -drag_value * velocity_x / (case.mass_kg * speed), -drag_value * velocity_z / (case.mass_kg * speed) - gravity(altitude), rate, (cp_value + damping_value + static_value) / inertia],
                dtype=np.float64,
            )
        ####
        k1 = derivative(state)
        k2 = derivative(state + 0.5 * dt * k1)
        k3 = derivative(state + 0.5 * dt * k2)
        k4 = derivative(state + dt * k3)
        state = state + dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
        time_s += dt
        if state[1] < 0.0:
            state[1] = 0.0
            break
    ####
    return samples
####


def write_history(path: Path, samples: Sequence[CoupledSample], model: DragModel) -> None:
    channels = ("x_m", "z_m", "vx_mps", "vz_mps", "theta_deg", "alpha_deg", "omega_deg_s", "dynamic_pressure_pa", "drag_n", "torque_cp_nm", "torque_damping_nm", "torque_static_nm", "torque_total_nm")
    values: list[float] = []
    for sample in samples:
        values.extend((sample.x_m, sample.z_m, sample.vx_mps, sample.vz_mps, sample.theta_deg, sample.alpha_deg, sample.omega_deg_s, sample.dynamic_pressure_pa, sample.drag_n, sample.torque_cp_nm, sample.torque_damping_nm, sample.torque_static_nm, sample.torque_total_nm))
    ####
    write_multi_axis_table(path, title=f"{model.name}-coupled-history", table_type="output", axis_names=("time_s", "channel_id"), axis_values=(tuple(sample.time_s for sample in samples), tuple(float(i) for i in range(len(channels)))), value_name="output", value_values=values, precision=6)
    path.with_suffix(".md").write_text("# Coupled Tumble History\n\n" + "\n".join(f"- `{i}` = `{name}`" for i, name in enumerate(channels)) + "\n", encoding="utf-8")
####


def plot_history(samples: Sequence[CoupledSample], case: RigidBodyCase, output: Path) -> None:
    time = [s.time_s for s in samples]
    figure, axes = plt.subplots(3, 2, figsize=(12.0, 10.0), layout="constrained")
    axes[0, 0].plot([s.x_m for s in samples], [s.z_m for s in samples], color="#0f766e")
    axes[0, 0].set(xlabel="x (m)", ylabel="z (m)", title=f"{case.model.name}: coupled flight path")
    axes[0, 1].plot(time, [s.theta_deg for s in samples], label="theta", color="#7c3aed")
    axes[0, 1].plot(time, [s.alpha_deg for s in samples], label="alpha", color="#2563eb")
    axes[0, 1].set_ylabel("Angle (deg)")
    axes[0, 1].legend()
    axes[1, 0].plot(time, [s.omega_deg_s for s in samples], color="#b45309")
    axes[1, 0].set_ylabel("Angular rate (deg/s)")
    axes[1, 1].plot(time, [s.drag_n for s in samples], color="#0f766e")
    axes[1, 1].set_ylabel("Drag force (N)")
    axes[2, 0].plot(time, [s.torque_cp_nm for s in samples], label="CP offset")
    axes[2, 0].plot(time, [s.torque_damping_nm for s in samples], label="rate damping")
    axes[2, 0].plot(time, [s.torque_total_nm for s in samples], label="total", linewidth=2.0)
    axes[2, 0].set_ylabel("Torque (N m)")
    axes[2, 0].legend()
    axes[2, 1].plot(time, [s.dynamic_pressure_pa for s in samples], color="#b45309")
    axes[2, 1].set_ylabel("Dynamic pressure (Pa)")
    for axis in axes.flat:
        axis.set_xlabel("Time (s)")
        axis.grid(True, alpha=0.25)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    plt.close(figure)
####


def render_animation(samples: Sequence[CoupledSample], case: RigidBodyCase, output: Path) -> None:
    if not samples:
        raise ValueError("animation requires at least one sample")
    ####
    frame_indices = np.linspace(0, len(samples) - 1, min(60, len(samples)), dtype=int)
    frame_samples = [samples[index] for index in frame_indices]
    figure, axis = plt.subplots(figsize=(7.0, 5.0), layout="constrained")
    axis.set_aspect("equal", adjustable="box")
    span = max(case.length_m, 5.0)
    x_values = [sample.x_m for sample in samples]
    z_values = [sample.z_m for sample in samples]
    axis.set_xlim(min(x_values) - span, max(x_values) + span)
    axis.set_ylim(min(z_values) - span, max(z_values) + span)
    axis.set(xlabel="x (m)", ylabel="z (m)", title=f"{case.model.name}: coupled tumble")
    path_line, = axis.plot([], [], color="#94a3b8", linewidth=1.2)
    body_line, = axis.plot([], [], color="#7c3aed", linewidth=4.0)
    cp_point, = axis.plot([], [], "o", color="#b45309")
    force_line, = axis.plot([], [], color="#dc2626", linewidth=2.0)
    time_label = axis.text(0.03, 0.95, "", transform=axis.transAxes)
    ####
    def update(frame_data: tuple[int, CoupledSample]):
        index, frame = frame_data
        path_line.set_data(x_values[: index + 1], z_values[: index + 1])
        ux = math.cos(math.radians(frame.theta_deg))
        uz = math.sin(math.radians(frame.theta_deg))
        half = 0.5 * case.length_m
        body_line.set_data((frame.x_m - half * ux, frame.x_m + half * ux), (frame.z_m - half * uz, frame.z_m + half * uz))
        cp_x = frame.x_m + case.cp_offset_m * ux
        cp_z = frame.z_m + case.cp_offset_m * uz
        cp_point.set_data((cp_x,), (cp_z,))
        scale = min(0.25, 0.01 * frame.drag_n)
        speed = max(math.hypot(frame.vx_mps, frame.vz_mps), 1e-6)
        force_line.set_data((cp_x, cp_x - scale * frame.vx_mps / speed), (cp_z, cp_z - scale * frame.vz_mps / speed))
        time_label.set_text(f"t = {frame.time_s:.2f} s")
        return path_line, body_line, cp_point, force_line, time_label
    ####
    movie = animation.FuncAnimation(figure, update, frames=zip(frame_indices, frame_samples, strict=True), interval=50, blit=True, cache_frame_data=False)
    output.parent.mkdir(parents=True, exist_ok=True)
    movie.save(output, writer="pillow", fps=12, dpi=80)
    plt.close(figure)
####


def main() -> None:
    parser = argparse.ArgumentParser(description="Integrate planar translational and rotational tumble dynamics.")
    parser.add_argument("--shape", choices=sorted(DEFAULT_MODELS), default="cone")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "analysis" / "tumbling" / "generated" / "coupled")
    args = parser.parse_args()
    model = DEFAULT_MODELS[args.shape]
    case = RigidBodyCase(model=model, mass_kg=1.0, length_m=2.0, cp_offset_m=0.08, cm_q=0.8)
    samples = simulate(case)
    write_history(args.output_dir / f"{model.name}.tbl", samples, model)
    plot_history(samples, case, args.output_dir / f"{model.name}.png")
    if model.name == "cone":
        render_animation(samples, case, args.output_dir / "tumbling_cone.gif")
    print(args.output_dir)
####


if __name__ == "__main__":
    main()
