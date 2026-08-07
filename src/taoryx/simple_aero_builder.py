"""Low-code builders for reduced-order SimpleAero-style 3-DOF trajectories.

The builder is deliberately small: a user supplies launch, burnout, apogee,
heading, and fixed-L/D parameters and receives ordinary TAOS problem text plus
a manifest of the derived timing and geometry.  The generated problem is a
reduced-order, numeric-coefficient surrogate.  It is useful for composition,
grammar checks, and segment-level tests; it is not a recovered SimpleAero model
or a substitute for vehicle-specific aerodynamic tables.
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
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

    scenario_id: str = Field(default="simple_aero-fixed-ld", min_length=1)
    vehicle_id: str = Field(default="generic-3dof", min_length=1)
    family: str = Field(default="ballistic", min_length=1)

    initial_altitude_m: float = Field(default=0.0, ge=0.0)
    initial_speed_m_s: float = Field(default=50.0, ge=0.0)
    vbo_m_s: float = Field(default=1200.0, gt=0.0)
    apogee_altitude_m: float = Field(default=80_000.0, gt=0.0)
    pitch_over_angle_deg: float = Field(default=80.0, gt=-89.0, lt=89.0)

    target_range_m: float = Field(default=100_000.0, gt=0.0)
    target_bearing_deg: float = Field(default=90.0, ge=-360.0, le=360.0)
    target_latitude_deg: float | None = Field(default=None, ge=-90.0, le=90.0)
    target_longitude_deg: float | None = Field(default=None, ge=-180.0, lt=180.0)
    initial_heading_offset_deg: float = Field(default=0.0, ge=-180.0, le=180.0)
    target_altitude_m: float = Field(default=0.0, ge=0.0)
    target_speed_m_s: float = Field(default=0.0, ge=0.0)
    target_heading_deg: float = Field(default=0.0, ge=-360.0, le=360.0)
    launch_latitude_deg: float = Field(default=0.0, ge=-90.0, le=90.0)
    launch_longitude_deg: float = Field(default=0.0, ge=-360.0, le=360.0)

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
        if (self.target_latitude_deg is None) != (self.target_longitude_deg is None):
            raise ValueError("target_latitude_deg and target_longitude_deg must be supplied together")
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
    target_range_m: float
    target_bearing_deg: float
    target_latitude_deg: float
    target_longitude_deg: float
    assumptions: tuple[str, ...]
    ####


class SimpleAeroTrajectoryBuild(BaseModel):
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


def _target_range_bearing(
    *,
    target_latitude_deg: float,
    target_longitude_deg: float,
    launch_latitude_deg: float,
    launch_longitude_deg: float,
) -> tuple[float, float]:
    """Return spherical-Earth range and initial bearing to an aimpoint."""

    radius_m = 6_378_137.0
    launch_lat = math.radians(launch_latitude_deg)
    target_lat = math.radians(target_latitude_deg)
    longitude_delta = math.radians(target_longitude_deg - launch_longitude_deg)
    central_angle = math.acos(
        max(
            -1.0,
            min(
                1.0,
                math.sin(launch_lat) * math.sin(target_lat)
                + math.cos(launch_lat) * math.cos(target_lat) * math.cos(longitude_delta),
            ),
        )
    )
    bearing = math.atan2(
        math.sin(longitude_delta) * math.cos(target_lat),
        math.cos(launch_lat) * math.sin(target_lat)
        - math.sin(launch_lat) * math.cos(target_lat) * math.cos(longitude_delta),
    )
    return radius_m * central_angle, _wrap_heading(math.degrees(bearing))
    ####


class SimpleAeroTrajectoryBuilder:
    """Build a reduced-order 3-DOF SimpleAero-style trajectory with few inputs."""

    def __init__(
        self,
        parameters: FixedLD3DOFParameters | None = None,
        *,
        segment_graph: Mapping[str, Any] | None = None,
        **overrides: Any,
    ) -> None:
        if parameters is not None and overrides:
            raise ValueError("pass either parameters or keyword overrides, not both")
        self.parameters = parameters or FixedLD3DOFParameters(**overrides)
        self.segment_graph = dict(segment_graph or {})
        ####

    def derive(self) -> DerivedFixedLDProfile:
        """Calculate headings, fixed-L/D coefficients, and phase durations."""

        p = self.parameters
        boost_duration = p.boost_duration_s or (p.vbo_m_s - p.initial_speed_m_s) / p.boost_acceleration_m_s2
        coast_duration = p.coast_duration_s or math.sqrt(
            2.0 * (p.apogee_altitude_m - p.initial_altitude_m) / 9.80665
        )
        if p.target_latitude_deg is not None and p.target_longitude_deg is not None:
            target_latitude = p.target_latitude_deg
            target_longitude = p.target_longitude_deg
            target_range, target_bearing = _target_range_bearing(
                target_latitude_deg=target_latitude,
                target_longitude_deg=target_longitude,
                launch_latitude_deg=p.launch_latitude_deg,
                launch_longitude_deg=p.launch_longitude_deg,
            )
        else:
            target_range = p.target_range_m
            target_bearing = p.target_bearing_deg
            target_latitude, target_longitude = _target_geodetic(
                range_m=target_range,
                bearing_deg=target_bearing,
                latitude_deg=p.launch_latitude_deg,
                longitude_deg=p.launch_longitude_deg,
            )
        initial_heading = _wrap_heading(target_bearing + p.initial_heading_offset_deg)
        return DerivedFixedLDProfile(
            initial_heading_deg=initial_heading,
            lift_coefficient=p.drag_coefficient * p.lift_to_drag,
            boost_duration_s=boost_duration,
            coast_duration_s=coast_duration,
            total_duration_s=boost_duration + coast_duration + p.bank_duration_s + p.terminal_duration_s,
            target_range_m=target_range,
            target_bearing_deg=target_bearing,
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

    def _segment_lines(self, derived: DerivedFixedLDProfile) -> tuple[str, ...]:
        """Render a data-selected phase graph into ordinary native syntax."""

        p = self.parameters
        graph = self.segment_graph
        raw_segments = graph.get("segments")
        if not isinstance(raw_segments, list) or not raw_segments:
            raw_segments = [
                {"id": "powered-ascent", "kind": "powered_ascent", "duration_s": derived.boost_duration_s},
                {"id": "ballistic-coast", "kind": "ballistic_coast", "duration_s": derived.coast_duration_s},
                {"id": "bank-maneuver", "kind": "bank_maneuver", "duration_s": p.bank_duration_s},
                {"id": "terminal-pronav", "kind": "terminal_pronav", "duration_s": p.terminal_duration_s},
            ]
        parameter_values: dict[str, float] = {
            "mission.boost_duration": derived.boost_duration_s,
            "mission.coast_duration": derived.coast_duration_s,
            "mission.bank_duration": p.bank_duration_s,
            "mission.terminal_duration": p.terminal_duration_s,
        }
        ids = [str(item["id"]) for item in raw_segments]
        if len(set(ids)) != len(ids):
            raise ValueError("segment graph contains duplicate IDs")
        lines: list[str] = []
        for index, item in enumerate(raw_segments, start=1):
            if not isinstance(item, Mapping):
                raise ValueError("segment graph entries must be mappings")
            segment_id = str(item["id"])
            kind = str(item.get("kind", "ballistic_coast"))
            duration_key = item.get("duration_parameter")
            duration = float(item.get("duration_s", parameter_values.get(str(duration_key), 0.0)))
            if duration <= 0.0:
                raise ValueError(f"segment {segment_id!r} duration must be positive")
            target = "stop" if index == len(raw_segments) else f"goto {index + 1}"
            lines.append(f"  *segment {index} {segment_id}")
            lines.append(f"    *integ dtprnt={p.output_interval_s:g} dt={p.time_step_s:g}")
            lines.append(f"    *aero ca={-p.drag_coefficient:g} cn={derived.lift_coefficient:g}")
            if kind == "powered_ascent":
                lines.append(f"    *prop thrust={p.thrust_n:g} mdot={p.mass_flow_kg_s:g}")
                cutoff_mode = str(graph.get("cutoff_mode", "physical"))
                if cutoff_mode == "physical":
                    lines.append(f"    # event physical_burnout: tseg={duration:g}; mass-flow depletion is the active source condition")
                    lines.append(f"    # event commanded_cutoff: vel>{p.vbo_m_s:g}; available as a separate requested condition")
                    lines.append(f"    *when tseg={duration:g} {target}")
                elif cutoff_mode == "commanded":
                    lines.append(f"    # event physical_burnout: tseg={duration:g}; available as a separate source condition")
                    lines.append(f"    # event commanded_cutoff: vel>{p.vbo_m_s:g}; requested speed is the active condition")
                    lines.append(f"    *when vel>{p.vbo_m_s:g} {target}")
                else:
                    raise ValueError(f"unsupported cutoff mode {cutoff_mode!r}")
            elif kind == "ballistic_coast":
                lines.append(f"    *fly alpha={p.alpha_deg:g}")
                lines.append(f"    *when tseg={duration:g} {target}")
            elif kind == "bank_maneuver":
                lines.append(f"    *fly bankgd={p.bank_deg:g}")
                lines.append(f"    *when tseg={duration:g} {target}")
            elif kind == "terminal_pronav":
                lines.append("    *fly propnav=2")
                lines.append(f"    *when relrng[2] < {p.terminal_capture_range_m:g} stop")
                lines.append(f"    *when tseg={duration:g} stop")
            else:
                raise ValueError(f"unsupported Simple Aero segment kind {kind!r}")
        return tuple(lines)
        ####

    def build(self) -> SimpleAeroTrajectoryBuild:
        """Render a parser-compatible point-mass problem and its manifest."""

        p = self.parameters
        d = self.derive()
        total = sum(
            float(item.get("duration_s", 0.0))
            if "duration_s" in item
            else float(
                {
                    "mission.boost_duration": d.boost_duration_s,
                    "mission.coast_duration": d.coast_duration_s,
                    "mission.bank_duration": p.bank_duration_s,
                    "mission.terminal_duration": p.terminal_duration_s,
                }.get(str(item.get("duration_parameter")), 0.0)
            )
            for item in self.segment_graph.get("segments", ())
        ) or d.total_duration_s
        target_heading = _wrap_heading(p.target_heading_deg)
        earth_lines = (
            "*atmos standard\n*earth wgs-84 omega=0"
            if p.earth == "standard"
            else "*atmos none\n*earth spherical gm=0 omega=0"
        )
        text = f"""({p.scenario_id})
*title Reduced-order SimpleAero-style fixed-L/D 3-DOF trajectory
*mode point-mass
{earth_lines}

# Generated from taoryx.simple_aero_builder.  Replace numeric CA/CN and timing
# assumptions with vehicle-specific tables before promotion.
*runtime status vehicle={p.vehicle_id} family={p.family} vbo-mps={p.vbo_m_s:g} apogee-altitude-m={p.apogee_altitude_m:g} pitch-over-deg={p.pitch_over_angle_deg:g} initial-heading-offset-deg={p.initial_heading_offset_deg:g} fixed-lift-to-drag={p.lift_to_drag:g}
*runtime status target range-m={d.target_range_m:g} bearing-deg={d.target_bearing_deg:g} latitude-deg={d.target_latitude_deg:g} longitude-deg={d.target_longitude_deg:g} altitude-m={p.target_altitude_m:g} velocity-mps={p.target_speed_m_s:g}

*trajectory 1 {p.vehicle_id} start on 1
  *initial geodetic alt={p.initial_altitude_m:g} long={p.launch_longitude_deg:g} lat={p.launch_latitude_deg:g} vel={p.initial_speed_m_s:g} gama={p.pitch_over_angle_deg:g} psi={d.initial_heading_deg:g} time=0 mass={p.mass_kg:g}
  *file {p.vehicle_id}.dat time alt vel mass
{chr(10).join(self._segment_lines(d))}

*trajectory 2 target start on 1
  *initial geodetic alt={p.target_altitude_m:g} long={d.target_longitude_deg:g} lat={d.target_latitude_deg:g} vel={p.target_speed_m_s:g} gama=0 psi={target_heading:g} time=0 mass=1
  *file target.dat time alt vel
  *segment 1 target-track
    *integ dtprnt={p.output_interval_s:g} dt={p.time_step_s:g}
    *when tseg={total:g} stop
*end
"""
        return SimpleAeroTrajectoryBuild(parameters=p, derived=d, problem_text=text)
        ####


def build_fixed_ld_3dof(**parameters: Any) -> SimpleAeroTrajectoryBuild:
    """Convenience function for one-call, minimal-code trajectory generation."""

    return SimpleAeroTrajectoryBuilder(**parameters).build()
    ####


def build_fixed_ld_from_resolved_case(case: Any) -> SimpleAeroTrajectoryBuild:
    """Generate a native problem from one resolved Alpha 2 case.

    The mapping is intentionally explicit at this boundary: the family catalog
    owns semantic parameter IDs, while the legacy-shaped builder owns the
    native problem spelling.  No mission or loadout values are duplicated in
    this adapter.
    """

    def value(parameter_id: str, default: Any) -> Any:
        resolved = case.parameters.get(parameter_id)
        return default if resolved is None else resolved.value
        ####

    graph = dict(case.segment_graph)
    graph["cutoff_mode"] = case.extensions.get("cutoff_mode", "physical")
    launch = graph.get("launch", {})
    aim_point = graph.get("aim_point", {})
    parameters = FixedLD3DOFParameters(
        scenario_id=case.case_id,
        vehicle_id=str(value("vehicle.id", case.family)),
        family=case.family,
        initial_altitude_m=float(value("mission.initial_altitude", 0.0)),
        initial_speed_m_s=float(value("mission.initial_speed", 50.0)),
        vbo_m_s=float(value("mission.burnout_speed", 1200.0)),
        apogee_altitude_m=float(value("mission.apogee_altitude", 80_000.0)),
        pitch_over_angle_deg=float(value("mission.pitch_over_angle", 80.0)),
        target_range_m=float(value("mission.target_range", 100_000.0)),
        target_bearing_deg=float(value("mission.target_bearing", 90.0)),
        initial_heading_offset_deg=float(value("mission.initial_heading_offset", 0.0)),
        target_altitude_m=float(aim_point.get("altitude_m", value("mission.target_altitude", 0.0))),
        target_speed_m_s=float(value("mission.target_speed", 0.0)),
        target_heading_deg=float(value("mission.target_heading", 0.0)),
        launch_latitude_deg=float(launch.get("latitude_deg", 0.0)),
        launch_longitude_deg=float(launch.get("longitude_deg", 0.0)),
        mass_kg=float(value("vehicle.mass.initial", 1000.0)),
        thrust_n=float(value("vehicle.booster.thrust", 100_000.0)),
        mass_flow_kg_s=float(value("vehicle.booster.mass_flow", 20.0)),
        drag_coefficient=float(value("vehicle.aero.drag_coefficient", 0.02)),
        lift_to_drag=float(value("vehicle.aero.lift_to_drag", 4.0)),
        boost_duration_s=float(value("mission.boost_duration", 5.0)),
        coast_duration_s=float(value("mission.coast_duration", 10.0)),
        bank_duration_s=float(value("mission.bank_duration", 5.0)),
        terminal_duration_s=float(value("mission.terminal_duration", 5.0)),
        terminal_capture_range_m=float(value("mission.terminal_capture_range", 25.0)),
        time_step_s=float(value("runtime.time_step", 0.05)),
        output_interval_s=float(value("runtime.output_interval", 0.5)),
    )
    return SimpleAeroTrajectoryBuilder(parameters, segment_graph=graph).build()
    ####


__all__ = [
    "DerivedFixedLDProfile",
    "FixedLD3DOFParameters",
    "SimpleAeroTrajectoryBuild",
    "SimpleAeroTrajectoryBuilder",
    "build_fixed_ld_from_resolved_case",
    "build_fixed_ld_3dof",
]
