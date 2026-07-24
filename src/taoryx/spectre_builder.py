"""Low-code builders for reduced-order Spectre-style 3-DOF trajectories.

The builder is deliberately small: a user supplies launch, burnout, apogee,
heading, and fixed-L/D parameters and receives ordinary TAOS problem text plus
a manifest of the derived timing and geometry.  The generated problem is a
reduced-order, numeric-coefficient surrogate.  It is useful for composition,
grammar checks, and segment-level tests; it is not a recovered Spectre model
or a substitute for vehicle-specific aerodynamic tables.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

EarthMode = Literal["standard", "none"]


class FixedLD3DOFParameters(BaseModel):
    """Minimal parameter surface for a fixed-L/D 3-DOF trajectory.

    ``vbo_m_s`` is the requested burnout speed and ``apogee_altitude_m`` is a
    checkpoint target used to derive the default coast duration.  The
    generated trajectory does not solve a launch-vehicle design problem; the
    manifest records the surrogate assumptions so they can be replaced by a
    vehicle adapter when promoted.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    scenario_id: str = Field(default="spectre-fixed-ld", min_length=1)
    vehicle_id: str = Field(default="generic-3dof", min_length=1)
    family: str = Field(default="ballistic", min_length=1)

    initial_altitude_m: float = Field(default=0.0, ge=0.0)
    initial_speed_m_s: float = Field(default=50.0, ge=0.0)
    vbo_m_s: float = Field(default=1200.0, gt=0.0)
    apogee_altitude_m: float = Field(default=80_000.0, gt=0.0)
    pitch_over_angle_deg: float = Field(default=80.0, gt=-89.0, lt=89.0)

    target_range_m: float = Field(default=100_000.0, gt=0.0)
    target_bearing_deg: float = Field(default=90.0, ge=-360.0, le=360.0)
    initial_heading_offset_deg: float = Field(default=0.0, ge=-180.0, le=180.0)
    target_altitude_m: float = Field(default=0.0, ge=0.0)
    target_speed_m_s: float = Field(default=0.0, ge=0.0)
    target_heading_deg: float = Field(default=0.0, ge=-360.0, le=360.0)

    mass_kg: float = Field(default=1000.0, gt=0.0)
    thrust_n: float = Field(default=100_000.0, ge=0.0)
    mass_flow_kg_s: float = Field(default=20.0, ge=0.0)
    boost_acceleration_m_s2: float = Field(default=100.0, gt=0.0)

    drag_coefficient: float = Field(default=0.02, gt=0.0)
    lift_to_drag: float = Field(default=4.0, ge=0.0)
    alpha_deg: float = Field(default=0.0, ge=-45.0, le=45.0)
    bank_deg: float = Field(default=0.0, ge=-180.0, le=180.0)

    boost_duration_s: float | None = Field(default=None, gt=0.0)
    coast_duration_s: float | None = Field(default=None, gt=0.0)
    bank_duration_s: float = Field(default=20.0, gt=0.0)
    terminal_duration_s: float = Field(default=30.0, gt=0.0)
    terminal_capture_range_m: float = Field(default=25.0, gt=0.0)
    time_step_s: float = Field(default=0.05, gt=0.0)
    output_interval_s: float = Field(default=0.5, gt=0.0)
    earth: EarthMode = "standard"

    @model_validator(mode="after")
    def validate_profile(self) -> FixedLD3DOFParameters:
        if self.vbo_m_s <= self.initial_speed_m_s:
            raise ValueError("vbo_m_s must exceed initial_speed_m_s")
        if self.apogee_altitude_m <= self.initial_altitude_m:
            raise ValueError("apogee_altitude_m must exceed initial_altitude_m")
        if self.mass_flow_kg_s > 0.0 and self.thrust_n <= 0.0:
            raise ValueError("positive mass_flow_kg_s requires positive thrust_n")
        return self
        ####


class DerivedFixedLDProfile(BaseModel):
    """Auditable values calculated from :class:`FixedLD3DOFParameters`."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    initial_heading_deg: float
    lift_coefficient: float
    boost_duration_s: float
    coast_duration_s: float
    total_duration_s: float
    target_latitude_deg: float
    target_longitude_deg: float
    assumptions: tuple[str, ...]
    ####


class SpectreTrajectoryBuild(BaseModel):
    """Rendered problem and provenance manifest returned by the builder."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    parameters: FixedLD3DOFParameters
    derived: DerivedFixedLDProfile
    problem_text: str = Field(min_length=1)

    def write(self, problem_path: str | Path, manifest_path: str | Path | None = None) -> None:
        """Write the generated `.prb` and optional JSON manifest."""

        problem_target = Path(problem_path)
        problem_target.parent.mkdir(parents=True, exist_ok=True)
        problem_target.write_text(self.problem_text, encoding="utf-8")
        if manifest_path is not None:
            manifest_target = Path(manifest_path)
            manifest_target.parent.mkdir(parents=True, exist_ok=True)
            manifest_target.write_text(
                json.dumps(
                    {
                        "parameters": self.parameters.model_dump(mode="json"),
                        "derived": self.derived.model_dump(mode="json"),
                    },
                    indent=2,
                    sort_keys=True,
                )
                + "\n",
                encoding="utf-8",
            )
        ####


def _wrap_heading(angle_deg: float) -> float:
    """Wrap a heading to the conventional ``[-180, 180)`` interval."""

    return (angle_deg + 180.0) % 360.0 - 180.0
    ####


def _target_geodetic(
    *,
    range_m: float,
    bearing_deg: float,
    latitude_deg: float = 0.0,
    longitude_deg: float = 0.0,
) -> tuple[float, float]:
    """Return a spherical-Earth target location from local launch geometry."""

    radius_m = 6_378_137.0
    lat = math.radians(latitude_deg)
    lon = math.radians(longitude_deg)
    bearing = math.radians(bearing_deg)
    angular_range = range_m / radius_m
    target_lat = math.asin(
        math.sin(lat) * math.cos(angular_range)
        + math.cos(lat) * math.sin(angular_range) * math.cos(bearing)
    )
    target_lon = lon + math.atan2(
        math.sin(bearing) * math.sin(angular_range) * math.cos(lat),
        math.cos(angular_range) - math.sin(lat) * math.sin(target_lat),
    )
    return math.degrees(target_lat), _wrap_heading(math.degrees(target_lon))
    ####


class SpectreTrajectoryBuilder:
    """Build a reduced-order 3-DOF Spectre-style trajectory with few inputs."""

    def __init__(self, parameters: FixedLD3DOFParameters | None = None, **overrides: Any) -> None:
        if parameters is not None and overrides:
            raise ValueError("pass either parameters or keyword overrides, not both")
        self.parameters = parameters or FixedLD3DOFParameters(**overrides)
        ####

    def derive(self) -> DerivedFixedLDProfile:
        """Calculate headings, fixed-L/D coefficients, and phase durations."""

        p = self.parameters
        boost_duration = p.boost_duration_s or (p.vbo_m_s - p.initial_speed_m_s) / p.boost_acceleration_m_s2
        coast_duration = p.coast_duration_s or math.sqrt(
            2.0 * (p.apogee_altitude_m - p.initial_altitude_m) / 9.80665
        )
        initial_heading = _wrap_heading(p.target_bearing_deg + p.initial_heading_offset_deg)
        target_latitude, target_longitude = _target_geodetic(
            range_m=p.target_range_m,
            bearing_deg=p.target_bearing_deg,
        )
        return DerivedFixedLDProfile(
            initial_heading_deg=initial_heading,
            lift_coefficient=p.drag_coefficient * p.lift_to_drag,
            boost_duration_s=boost_duration,
            coast_duration_s=coast_duration,
            total_duration_s=boost_duration + coast_duration + p.bank_duration_s + p.terminal_duration_s,
            target_latitude_deg=target_latitude,
            target_longitude_deg=target_longitude,
            assumptions=(
                "constant numeric aerodynamic coefficients",
                "fixed lift-to-drag surrogate: CN = CA magnitude * L/D",
                "apogee derives a time checkpoint; it is not a solved trajectory constraint",
                "target geometry uses a spherical local launch-to-target construction",
                "family is provenance metadata; the builder emits a common four-phase skeleton",
            ),
        )
        ####

    def build(self) -> SpectreTrajectoryBuild:
        """Render a parser-compatible point-mass problem and its manifest."""

        p = self.parameters
        d = self.derive()
        ca = -p.drag_coefficient
        cn = d.lift_coefficient
        total = d.total_duration_s
        target_heading = _wrap_heading(p.target_heading_deg)
        earth_lines = (
            "*atmos standard\n*earth wgs-84 omega=0"
            if p.earth == "standard"
            else "*atmos none\n*earth spherical gm=0 omega=0"
        )
        text = f"""({p.scenario_id})
*title Reduced-order Spectre-style fixed-L/D 3-DOF trajectory
*mode point-mass
{earth_lines}

# Generated from taoryx.spectre_builder.  Replace numeric CA/CN and timing
# assumptions with vehicle-specific tables before promotion.
*runtime status vehicle={p.vehicle_id} family={p.family} vbo-mps={p.vbo_m_s:g} apogee-altitude-m={p.apogee_altitude_m:g} pitch-over-deg={p.pitch_over_angle_deg:g} initial-heading-offset-deg={p.initial_heading_offset_deg:g} fixed-lift-to-drag={p.lift_to_drag:g}
*runtime status target range-m={p.target_range_m:g} bearing-deg={p.target_bearing_deg:g} latitude-deg={d.target_latitude_deg:g} longitude-deg={d.target_longitude_deg:g} altitude-m={p.target_altitude_m:g} velocity-mps={p.target_speed_m_s:g}

*trajectory 1 {p.vehicle_id} start on 1
  *initial geodetic alt={p.initial_altitude_m:g} long=0 lat=0 vel={p.initial_speed_m_s:g} gama={p.pitch_over_angle_deg:g} psi={d.initial_heading_deg:g} time=0 mass={p.mass_kg:g}
  *file {p.vehicle_id}.dat time alt vel mass
  *segment 1 powered-ascent
    *integ dtprnt={p.output_interval_s:g} dt={p.time_step_s:g}
    *prop thrust={p.thrust_n:g} mdot={p.mass_flow_kg_s:g}
    *aero ca={ca:g} cn={cn:g}
    *when tseg={d.boost_duration_s:g} goto 2
  *segment 2 ballistic-coast
    *integ dtprnt={p.output_interval_s:g} dt={p.time_step_s:g}
    *aero ca={ca:g} cn={cn:g}
    *fly alpha={p.alpha_deg:g}
    *when tseg={d.coast_duration_s:g} goto 3
  *segment 3 bank-maneuver
    *integ dtprnt={p.output_interval_s:g} dt={p.time_step_s:g}
    *aero ca={ca:g} cn={cn:g}
    *fly bankgd={p.bank_deg:g}
    *when tseg={p.bank_duration_s:g} goto 4
  *segment 4 terminal-pronav
    *integ dtprnt={p.output_interval_s:g} dt={p.time_step_s:g}
    *aero ca={ca:g} cn={cn:g}
    *fly propnav=2
    *when relrng[2] < {p.terminal_capture_range_m:g} stop
    *when tseg={p.terminal_duration_s:g} stop

*trajectory 2 target start on 1
  *initial geodetic alt={p.target_altitude_m:g} long={d.target_longitude_deg:g} lat={d.target_latitude_deg:g} vel={p.target_speed_m_s:g} gama=0 psi={target_heading:g} time=0 mass=1
  *file target.dat time alt vel
  *segment 1 target-track
    *integ dtprnt={p.output_interval_s:g} dt={p.time_step_s:g}
    *when tseg={total:g} stop
*end
"""
        return SpectreTrajectoryBuild(parameters=p, derived=d, problem_text=text)
        ####


def build_fixed_ld_3dof(**parameters: Any) -> SpectreTrajectoryBuild:
    """Convenience function for one-call, minimal-code trajectory generation."""

    return SpectreTrajectoryBuilder(**parameters).build()
    ####


__all__ = [
    "DerivedFixedLDProfile",
    "FixedLD3DOFParameters",
    "SpectreTrajectoryBuild",
    "SpectreTrajectoryBuilder",
    "build_fixed_ld_3dof",
]
