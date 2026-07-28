"""Generate TAORYX ``.tbl`` decks from the slower-vehicle research bundle.

The CSV files remain the source-preserving records.  This importer creates only
regular numeric grids that the existing TAOS table parser can validate and
inspect; categorical metadata and unsupported actuator semantics stay in the
source bundle.
"""

from __future__ import annotations

import csv
import itertools
import json
import math
from pathlib import Path
from typing import Callable

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1"
TABLES = FIXTURE / "tables"
COEFFICIENTS = ("cx", "cy", "cz", "cmx", "cmy", "cmz")


def _number(value: float) -> str:
    return f"{value:.12g}"
    ####


def _read(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))
    ####


def _axis_value(column: str, value: str) -> float:
    if column == "altitude_ft":
        return float(value) * 0.3048
    if column.endswith("_deg"):
        return math.radians(float(value))
    return float(value)
    ####


def _write_grid(
    source: Path,
    destination: Path,
    *,
    axes: tuple[tuple[str, str], ...],
    filters: tuple[Callable[[dict[str, str]], bool], ...] = (),
    value_columns: tuple[tuple[str, str], ...] = tuple((name, name.upper()) for name in COEFFICIENTS),
    comment: str,
    extra_comments: tuple[str, ...] = (),
    trailing_blank_line: bool = True,
) -> None:
    rows = [row for row in _read(source) if all(predicate(row) for predicate in filters)]
    if not rows:
        raise ValueError(f"no rows remain for {destination}")
    axis_values = tuple(
        tuple(sorted({_axis_value(column, row[column]) for row in rows}))
        for _, column in axes
    )
    indexed = {
        tuple(_axis_value(column, row[column]) for _, column in axes): row
        for row in rows
    }
    points = list(itertools.product(*axis_values))
    if len(indexed) != len(points):
        raise ValueError(f"{source}: filtered rows do not form a complete regular grid")
    axis_names = tuple(name for name, _ in axes)
    text = f"# Derived from {source.relative_to(FIXTURE)}; {comment}\n"
    text += "".join(f"# {line}\n" for line in extra_comments)
    for table_name, column in value_columns:
        values = [float(indexed[point][column]) for point in points]
        text += f"({table_name})\ntable {table_name}({','.join(axis_names)}) no-extrap\n"
        for name, values_for_axis in zip(axis_names, axis_values, strict=True):
            text += f"{name}={','.join(_number(value) for value in values_for_axis)}\n"
        text += f"{table_name}={','.join(_number(value) for value in values)}\n\n"
    destination.parent.mkdir(parents=True, exist_ok=True)
    terminator = "\n\n" if trailing_blank_line else "\n"
    destination.write_text(text.rstrip("\n") + terminator, encoding="utf-8")
    ####


def _write_one_dimensional(
    source: Path,
    destination: Path,
    *,
    axis: tuple[str, str],
    values: tuple[tuple[str, str], ...],
    filters: tuple[Callable[[dict[str, str]], bool], ...] = (),
    comment: str,
) -> None:
    _write_grid(source, destination, axes=(axis,), filters=filters, value_columns=values, comment=comment)
    ####


def main() -> None:
    TABLES.mkdir(parents=True, exist_ok=True)
    for stale in TABLES.glob("hummingbird_*.tbl"):
        stale.unlink()
    ####
    b747 = FIXTURE / "jet_b747"
    x8 = FIXTURE / "cruise_class_uav_skywalker_x8"
    hummingbird = FIXTURE / "quadcopter_hummingbird"

    _write_grid(
        b747 / "aero/static_six_axis_grid.csv",
        TABLES / "b747_nominal_static_6axis.tbl",
        axes=(("alpha", "alpha_offset_from_reference_deg"), ("beta", "beta_deg")),
        filters=(
            lambda row: row["configuration"] == "nominal",
            lambda row: row["reference_fc_id"] == "3",
            lambda row: row["mach"] == "0.45",
        ),
        comment="nominal configuration at reference flight condition 3; alpha is offset from trim and angular axes are radians",
        extra_comments=(
            "Convention: cx/cy/cz are absolute body-axis force coefficients (x-forward, y-right, z-down); cx/cz are not wind-axis CD/CL.",
        ),
        trailing_blank_line=False,
    )
    _write_grid(
        b747 / "propulsion/jt9d_installed_thrust_map.csv",
        TABLES / "b747_jt9d_thrust.tbl",
        axes=(("altitude_m", "altitude_m"), ("mach", "mach"), ("throttle", "throttle")),
        value_columns=(("thrust", "total_cluster_thrust_N"),),
        comment="installed four-engine thrust map; fuel flow is not modeled",
    )
    _write_grid(
        b747 / "aero/elevator_grid.csv",
        TABLES / "b747_nominal_elevator_6axis.tbl",
        axes=(
            ("alpha", "alpha_offset_from_reference_deg"),
            ("beta", "beta_deg"),
            ("elevator", "elevator_deg"),
        ),
        filters=(
            lambda row: row["configuration"] == "nominal",
            lambda row: row["reference_fc_id"] == "3",
            lambda row: row["mach"] == "0.45",
        ),
        comment="nominal condition-3 elevator control derivative deck; alpha is offset from trim and angular axes are radians",
        extra_comments=(
            "Convention: values are absolute control-conditioned coefficients, including the zero-elevator baseline; use directly or form (elevator - static) deltas, never add both absolute decks.",
        ),
        trailing_blank_line=False,
    )
    for control_name in ("aileron", "rudder"):
        _write_grid(
            b747 / "aero" / f"{control_name}_grid.csv",
            TABLES / f"b747_nominal_{control_name}_6axis.tbl",
            axes=(
                ("alpha", "alpha_offset_from_reference_deg"),
                ("beta", "beta_deg"),
                (control_name, f"{control_name}_deg"),
            ),
            filters=(
                lambda row: row["configuration"] == "nominal",
                lambda row: row["reference_fc_id"] == "3",
                lambda row: row["mach"] == "0.45",
            ),
            comment=f"nominal condition-3 {control_name} control derivative deck; alpha is offset from trim and angular axes are radians",
            trailing_blank_line=False,
        )
    _write_grid(
        x8 / "aero/static_airframe_grid.csv",
        TABLES / "skywalker_x8_static_6axis.tbl",
        axes=(("velocity_m_s", "velocity_m_s"), ("altitude_m", "altitude_m"), ("alpha", "alpha_deg"), ("beta", "beta_deg")),
        value_columns=tuple((name, name.upper()) for name in COEFFICIENTS),
        comment="zero-control, zero-throttle flight-test identified grid; angular axes are radians",
        extra_comments=(
            "Composition contract: this is the zero-throttle baseline. X8 control decks are absolute total coefficients at their separate throttle=0.44 reference condition, so their zero-control values are not expected to equal this table.",
        ),
        trailing_blank_line=False,
    )
    for control_name, source_name, axis_name, axis_column in (
        ("collective_elevon", "collective_elevon_grid.csv", "collective_elevon", "collective_elevon_deg"),
        ("differential_elevon", "differential_elevon_grid.csv", "differential_elevon", "differential_elevon_deg"),
    ):
        composition_axis_name = "collective" if control_name == "collective_elevon" else "differential"
        composition_comment = (
            "Composition contract: values are absolute total coefficients at the control-grid throttle=0.44 reference condition, "
            f"not deltas; runtime uses ({composition_axis_name} table - static table) within the documented composition."
        )
        _write_grid(
            x8 / "aero" / source_name,
            TABLES / f"skywalker_x8_{control_name}_6axis.tbl",
            axes=(
                ("velocity_m_s", "velocity_m_s"),
                ("altitude_m", "altitude_m"),
                ("alpha", "alpha_deg"),
                ("beta", "beta_deg"),
                (control_name, axis_column),
            ),
            value_columns=tuple((name, name.upper()) for name in COEFFICIENTS),
            comment=f"{control_name} source grid; angular axes are radians",
            extra_comments=(composition_comment,),
            trailing_blank_line=False,
        )
    for rate_name, rate_column in (("p", "p_hat"), ("q", "q_hat"), ("r", "r_hat")):
        _write_grid(
            x8 / "aero/rate_effects_grid.csv",
            TABLES / f"skywalker_x8_rate_{rate_name}_6axis.tbl",
            axes=(
                ("velocity_m_s", "velocity_m_s"),
                ("altitude_m", "altitude_m"),
                (f"{rate_name}_hat", rate_column),
            ),
            filters=(lambda row, channel=rate_name: row["active_rate_channel"] == f"{channel}_hat",),
            value_columns=tuple((name, name.upper()) for name in COEFFICIENTS),
            comment=f"{rate_name}-rate source slice at the identified X8 reference alpha, beta, and controls",
        )
    _write_grid(
        x8 / "propulsion/simplified_thrust_map.csv",
        TABLES / "skywalker_x8_thrust.tbl",
        axes=(("altitude_m", "altitude_m"), ("velocity_m_s", "velocity_m_s"), ("throttle", "throttle")),
        value_columns=(("thrust", "thrust_N"),),
        comment="simplified calibrated electric-propeller model",
    )
    _write_one_dimensional(
        x8 / "propulsion/propeller_coefficients.csv",
        TABLES / "skywalker_x8_propeller_ct.tbl",
        axis=("advance_ratio", "advance_ratio_J"),
        values=(("output", "CT"),),
        comment="published propeller polynomial sample points",
    )
    _write_one_dimensional(
        x8 / "propulsion/propeller_coefficients.csv",
        TABLES / "skywalker_x8_propeller_cq.tbl",
        axis=("advance_ratio", "advance_ratio_J"),
        values=(("output", "CQ"),),
        comment="published propeller polynomial sample points",
    )
    wrench_axes = (
        ("velocity_x", "body_velocity_x_m_s"),
        ("velocity_y", "body_velocity_y_m_s"),
        ("velocity_z", "body_velocity_z_m_s"),
        ("rotor_speed", "common_rotor_speed_rad_s"),
    )
    for output_name, source_column in (("cx", "FX_body_N"), ("cy", "FY_body_N"), ("cz", "FZ_body_N"), ("cmx", "MX_body_Nm"), ("cmy", "MY_body_Nm"), ("cmz", "MZ_body_Nm")):
        _write_grid(
            hummingbird / "aero/common_speed_wrench_grid.csv",
            TABLES / f"hummingbird_{output_name}.tbl",
            axes=wrench_axes,
            value_columns=((output_name, source_column),),
            comment=f"common-speed wrench component {output_name}; body rates are fixed at zero",
            extra_comments=(
                {
                    "cx": "Convention: cx is an adapter channel for source body-wrench FX, not an aerodynamic or wind-axis drag coefficient.",
                    "cy": "Convention: cy is an adapter channel for source body-wrench FY, not an aerodynamic side-force coefficient convention.",
                    "cz": "Convention: cz is an adapter channel for source body-wrench FZ, not an aerodynamic normal-force or wind-axis lift/drag coefficient.",
                }.get(output_name, ""),
            ) if output_name in {"cx", "cy", "cz"} else (),
            trailing_blank_line=output_name not in {"cx", "cy", "cz"},
        )
    _write_one_dimensional(
        hummingbird / "propulsion/rotor_static_map.csv",
        TABLES / "hummingbird_rotor_static.tbl",
        axis=("rotor_speed", "rotor_speed_rad_s"),
        values=(("thrust", "four_rotor_collective_thrust_N"),),
        comment="four-rotor quadratic static map",
    )
    _write_one_dimensional(
        hummingbird / "propulsion/rotor_static_map.csv",
        TABLES / "hummingbird_rotor_reaction_torque.tbl",
        axis=("rotor_speed", "rotor_speed_rad_s"),
        values=(("output", "single_rotor_reaction_torque_Nm_magnitude"),),
        comment="single-rotor reaction torque magnitude",
    )
    notional_first_cut = ("b747_notional_fuel_flow.tbl",)
    metadata = {
        "dataset_name": "slower_airbreathing_and_multirotor_6dof_bundle_v1",
        "source_bundle_sha256": "17ee0ca897e3ee44d4dd68deb6eb6c4922fef9ef87de46fb032124596b39b911",
        "fidelity": "vehicle-specific public research surrogates; not flight-qualified",
        "source_preserved": True,
        "generated": sorted(path.name for path in TABLES.glob("*.tbl") if path.name not in notional_first_cut),
        "taoryx_notional_first_cut": list(notional_first_cut),
        "models": {
            "b747": {"family": "subsonic-four-engine-jet", "source": "jet_b747"},
            "skywalker_x8": {"family": "fixed-wing-uav", "source": "cruise_class_uav_skywalker_x8"},
            "hummingbird": {"family": "multirotor", "source": "quadcopter_hummingbird"},
        },
        "omissions": [
            "The bounded direct-wrench B747 table is a Taoryx-derived local beta extension and is not source-envelope qualification.",
            "GTM remains a pointer-only alternate because its binary aero database was not supplied.",
        ],
    }
    (TABLES / "manifest.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    ####


if __name__ == "__main__":
    main()
    ####
