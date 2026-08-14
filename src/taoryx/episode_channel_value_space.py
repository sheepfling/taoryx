"""Topology declarations for legacy native episode channels.

This narrow module intentionally has no dependency on the interactive episode
host.  Composition-provider discovery needs the declarations while it builds
its public configuration schema; importing the full set of episode factories
at that point would construct unrelated physics and controller machinery.
"""

from __future__ import annotations

import math

from .value_space import (
    ValueSpaceSpec,
    boolean,
    bounded_interval,
    euclidean,
    finite_set,
    periodic_circle,
    positive_half_line,
    product,
    unit_interval,
)


def episode_channel_value_space(
    name: str,
    unit: str | None,
    lower: float | None,
    upper: float | None,
) -> ValueSpaceSpec:
    """Return a reviewed topology for a legacy native episode channel.

    The semantic interface remains the authority for AI/RL-facing channels.
    This adapter makes the older native-name episode schema equally explicit
    instead of asking a caller to guess from a unit or identifier. It covers
    the current source-owned episode bindings only; a new native channel must
    extend this table or provide its own ``ValueSpaceSpec`` at construction.
    """

    normalized = name.casefold()
    source_name = normalized.split(".", maxsplit=1)[1] if normalized.partition(".")[0].isdigit() else normalized
    if unit == "boolean" or normalized in {
        "guidance-override-enabled",
        "motors_enabled",
        "contact",
        "wrench_saturated",
        "physical_motor_allocation",
        "physical_effector_allocation",
    }:
        return boolean()
    if normalized in {"response_profile_id", "control_realization", "wrench_status"}:
        return finite_set(representation="scalar declared identifier")
    if normalized in {"yaw_rad", "attitude.yaw.command"} or normalized.endswith(".heading_rad"):
        return periodic_circle(2.0 * math.pi)
    if source_name in {"psi", "psigd", "yawgd", "_command_psi", "long"}:
        return periodic_circle(360.0, representation="scalar source-degree")
    if normalized == "attitude_rad":
        return product(
            bounded_interval(),
            bounded_interval(),
            periodic_circle(2.0 * math.pi),
            representation="vector3 [roll, pitch, yaw]",
        )
    vector_channels = {
        "position_ned_m",
        "velocity_ned_m_s",
        "body_rate_rad_s",
        "force_body_n",
        "moment_body_nm",
        "body_velocity_m_s",
        "requested_force_body_n",
        "requested_moment_body_nm",
        "achieved_force_body_n",
        "achieved_moment_body_nm",
        "residual_force_body_n",
        "residual_moment_body_nm",
    }
    if normalized in vector_channels:
        return euclidean(3)
    if normalized in {"battery_fraction", "thrust_ratio"}:
        return unit_interval()
    if source_name in {"throttle"}:
        return unit_interval()
    if source_name in {
        "collective-elevon-deg",
        "differential-elevon-deg",
        "gama",
        "gamgd",
        "_command_gamgd",
        "pitchgd",
        "alpha",
        "alphat",
        "_aero_alpha_reference_deg",
        "lat",
        "latgd",
    }:
        return bounded_interval(representation="scalar source-angle")
    if source_name in {"_geodetic_state", "_point_mass_si_contract", "_dtprnt", "_segment"}:
        return finite_set(representation="scalar source state code")
    if source_name in {
        "aggregate_thrust_n",
        "time_s",
        "duration_s",
        "speed_m_s",
        "mass_kg",
        "alt",
        "vel",
        "vair",
        "mass",
        "rho",
        "mach",
        "dynprs",
        "ntotal",
        "plength",
        "tseg",
        "tmark",
        "time",
        "temp",
        "pres",
        "sndspd",
        "nu",
        "rotor_speed",
        "_command_vel",
        "power",
        "wt",
        "fuel",
        "rcm",
        "thrust",
        "mdot",
        "range",
    }:
        return positive_half_line()
    if lower is not None or upper is not None:
        return bounded_interval()
    return euclidean()
    ####


__all__ = ["episode_channel_value_space"]
