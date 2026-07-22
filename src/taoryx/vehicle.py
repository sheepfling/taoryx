"""Native TAORYX vehicle contracts for mass properties and propulsion."""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import Protocol

from .contracts import Frame, Vector3
from .rigid_body import RigidBody6DofState, RigidBodyForceMoment
from .rigid_body_frames import EarthRotationAdapter
from .rotorcraft import QuadRotorAllocation, RotorCommandSet
from .runtime.environment_runtime import EnvironmentProvider
from .tables import PreparedTable, interpolate_nd


@dataclass(frozen=True, slots=True)
class MassProperties:
    """Mass and rigid-body properties at one resolved simulation state."""

    total_mass_kg: float
    dry_mass_kg: float
    propellant_mass_kg: float
    center_of_mass_m: Vector3
    inertia_kg_m2: Vector3

    def __post_init__(self) -> None:
        if self.dry_mass_kg <= 0.0 or self.total_mass_kg < self.dry_mass_kg:
            raise ValueError("total mass must be at least positive dry mass")
        if self.propellant_mass_kg < 0.0 or self.propellant_mass_kg > self.total_mass_kg - self.dry_mass_kg + 1.0e-12:
            raise ValueError("propellant mass must fit between total and dry mass")
        if min(self.inertia_kg_m2.x, self.inertia_kg_m2.y, self.inertia_kg_m2.z) <= 0.0:
            raise ValueError("all principal inertias must be positive")
        ####
    ####

    @classmethod
    def from_state(cls, state: RigidBody6DofState, dry_mass_kg: float, inertia_kg_m2: Vector3, center_of_mass_m: Vector3 | None = None) -> MassProperties:
        """Build a typed snapshot from the integrated rigid-body state."""

        return cls(state.mass, dry_mass_kg, state.propellant_mass, center_of_mass_m or Vector3(0.0, 0.0, 0.0), inertia_kg_m2)
        ####
    ####


@dataclass(frozen=True, slots=True)
class StageDefinition:
    """One time-bounded propulsion stage in the native successor model."""

    identifier: str
    start_time_s: float
    end_time_s: float
    thrust_body_n: Vector3
    propellant_mass_rate_kg_s: float
    moment_body_nm: Vector3 = Vector3(0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if not self.identifier or self.start_time_s < 0.0 or self.end_time_s <= self.start_time_s:
            raise ValueError("stage requires a nonempty identifier and positive time interval")
        if self.propellant_mass_rate_kg_s < 0.0:
            raise ValueError("stage propellant mass rate must be nonnegative")
        ####
    ####

    def active(self, time_s: float) -> bool:
        """Return whether this stage is active at a simulation time."""

        return self.start_time_s <= time_s < self.end_time_s
        ####
    ####


@dataclass(frozen=True, slots=True)
class PropulsionOutput:
    """Resolved body propulsion load and positive propellant flow."""

    force_body_n: Vector3
    moment_body_nm: Vector3
    propellant_mass_rate_kg_s: float
    active_stage_ids: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if self.propellant_mass_rate_kg_s < 0.0:
            raise ValueError("propellant mass rate must be nonnegative")
        ####
    ####


@dataclass(frozen=True, slots=True)
class StagedPropulsion:
    """Resolve non-overlapping or clustered stages into one propulsion load."""

    stages: tuple[StageDefinition, ...]

    def __post_init__(self) -> None:
        if len({stage.identifier for stage in self.stages}) != len(self.stages):
            raise ValueError("stage identifiers must be unique")
        ####
    ####

    def evaluate(self, time_s: float, propellant_mass_kg: float) -> PropulsionOutput:
        """Return the active stage load without mutating vehicle state."""

        if propellant_mass_kg <= 0.0:
            return PropulsionOutput(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0), 0.0)
        active = tuple(stage for stage in self.stages if stage.active(time_s))
        return PropulsionOutput(
            Vector3(sum(stage.thrust_body_n.x for stage in active), sum(stage.thrust_body_n.y for stage in active), sum(stage.thrust_body_n.z for stage in active)),
            Vector3(sum(stage.moment_body_nm.x for stage in active), sum(stage.moment_body_nm.y for stage in active), sum(stage.moment_body_nm.z for stage in active)),
            sum(stage.propellant_mass_rate_kg_s for stage in active),
            tuple(stage.identifier for stage in active),
        )
        ####
    ####

    def force_moment(self, state: RigidBody6DofState) -> RigidBodyForceMoment:
        """Adapt the native propulsion contract to the rigid-body model."""

        output = self.evaluate(state.time, state.propellant_mass)
        return RigidBodyForceMoment(output.force_body_n, output.moment_body_nm, output.propellant_mass_rate_kg_s)
        ####
    ####


@dataclass(frozen=True, slots=True)
class AerodynamicOutput:
    """Resolved aerodynamic load and air-data observables."""

    force_body_n: Vector3
    moment_body_nm: Vector3
    density_kg_m3: float
    dynamic_pressure_pa: float
    speed_of_sound_m_s: float
    airspeed_m_s: float
    mach: float
    angle_of_attack_rad: float
    sideslip_rad: float
    query_values: Mapping[str, float] = field(default_factory=dict)
    table_margins: Mapping[str, float] = field(default_factory=dict)


AerodynamicCoefficientProvider = Callable[[float, float, float], Vector3]


@dataclass(frozen=True, slots=True)
class AeroQueryContext:
    """Named aerodynamic query values supplied to coefficient tables."""

    values: Mapping[str, float]

    @classmethod
    def from_air_data(cls, mach: float, alpha: float, beta: float, extra: Mapping[str, float] | None = None) -> AeroQueryContext:
        """Build a query with canonical air-data names and extensions."""

        values = {"mach": mach, "alpha": alpha, "beta": beta}
        if extra is not None:
            values.update({name.casefold(): float(value) for name, value in extra.items()})
        return cls(values)
        ####
    ####


ContextCoefficientProvider = Callable[[AeroQueryContext], Vector3]


@dataclass(frozen=True, slots=True)
class PreparedCoefficientTable:
    """One prepared coefficient table with named independent variables."""

    name: str
    independent_variables: tuple[str, ...]
    table: PreparedTable

    def evaluate(self, values: Mapping[str, float]) -> float:
        """Evaluate strictly inside the declared coefficient envelope."""

        query_values = dict(values)
        if "velocity_m_s" in self.independent_variables and "velocity_m_s" not in query_values:
            query_values["velocity_m_s"] = query_values["airspeed_m_s"]
        if "airspeed_m_s" in self.independent_variables and "airspeed_m_s" not in query_values:
            query_values["airspeed_m_s"] = query_values["velocity_m_s"]
        raw_query = tuple(query_values[name] for name in self.independent_variables)
        query_values_at_boundary: list[float] = []
        for axis_name, axis, value in zip(self.independent_variables, self.table.axes, raw_query, strict=True):
            lower, upper = min(axis), max(axis)
            boundary_tolerance = 1.0e-12 * max(1.0, abs(lower), abs(upper))
            if value < lower - boundary_tolerance or value > upper + boundary_tolerance:
                distance = min(abs(value - lower), abs(value - upper))
                raise ValueError(
                    f"coefficient table {self.name!r} query {raw_query!r} is outside its declared envelope "
                    f"on axis {axis_name!r}: value={value}, range=[{lower}, {upper}], "
                    f"distance_to_boundary={distance}"
                )
            query_values_at_boundary.append(min(upper, max(lower, value)))
        query = tuple(query_values_at_boundary)
        return interpolate_nd(self.table, query)
        ####

    def margins(self, values: Mapping[str, float]) -> Mapping[str, float]:
        """Return distance to every declared interpolation-axis boundary."""

        query_values = dict(values)
        if "velocity_m_s" in self.independent_variables and "velocity_m_s" not in query_values:
            query_values["velocity_m_s"] = query_values["airspeed_m_s"]
        if "airspeed_m_s" in self.independent_variables and "airspeed_m_s" not in query_values:
            query_values["airspeed_m_s"] = query_values["velocity_m_s"]
        return {
            axis_name: min(abs(float(query_values[axis_name]) - min(axis)), abs(max(axis) - float(query_values[axis_name])))
            for axis_name, axis in zip(self.independent_variables, self.table.axes, strict=True)
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class PreparedAerodynamicCoefficients:
    """Force and moment coefficient tables resolved from `.tbl` data."""

    force_tables: Mapping[str, PreparedCoefficientTable]
    moment_tables: Mapping[str, PreparedCoefficientTable] = field(default_factory=dict)
    control_force_tables: Mapping[str, Mapping[str, PreparedCoefficientTable]] = field(default_factory=dict)
    control_moment_tables: Mapping[str, Mapping[str, PreparedCoefficientTable]] = field(default_factory=dict)

    @classmethod
    def from_runtime_tables(cls, tables: Mapping[str, RuntimeTableLike]) -> PreparedAerodynamicCoefficients:
        """Adapt prepared runtime tables without reparsing their source."""

        force: dict[str, PreparedCoefficientTable] = {}
        moments: dict[str, PreparedCoefficientTable] = {}
        control_force: dict[str, dict[str, PreparedCoefficientTable]] = {}
        control_moment: dict[str, dict[str, PreparedCoefficientTable]] = {}
        for key, runtime_table in tables.items():
            if runtime_table.prepared is None:
                raise ValueError(f"aerodynamic table {key!r} has no prepared interpolation data")
            key_name = key.casefold()
            name = (runtime_table.output_variable or key).casefold()
            family = key_name.removeprefix(f"{name}-") if key_name.startswith(f"{name}-") else ""
            prepared = PreparedCoefficientTable(name, tuple(runtime_table.independent_variables), runtime_table.prepared)
            if name in {"cx", "cy", "cz"}:
                if family and family != "static":
                    control_force.setdefault(family, {})[name] = prepared
                else:
                    force.setdefault(name, prepared)
            elif name in {"cmx", "cmy", "cmz"}:
                if family and family != "static":
                    control_moment.setdefault(family, {})[name] = prepared
                else:
                    moments.setdefault(name, prepared)
        return cls(force, moments, control_force, control_moment)
        ####

    def _provider(
        self,
        tables: Mapping[str, PreparedCoefficientTable],
        values: Mapping[str, float],
        required: tuple[str, str, str] = ("cx", "cy", "cz"),
    ) -> Vector3:
        normalized = {name.casefold(): table for name, table in tables.items()}
        missing = tuple(name for name in required if name not in normalized)
        if missing:
            raise ValueError(f"aerodynamic coefficient set is missing: {', '.join(missing)}")
        return Vector3(*(normalized[name].evaluate(values) for name in required))
        ####

    def force_provider(self) -> AerodynamicCoefficientProvider:
        """Return the callback consumed by :class:`TableAerodynamicModel`."""

        return lambda mach, alpha, beta: self._provider(self.force_tables, {"mach": mach, "alpha": alpha, "beta": beta})
        ####

    def context_force_provider(self) -> ContextCoefficientProvider:
        """Return an extensible provider accepting arbitrary named variables."""

        return lambda context: self._compose_provider(self.force_tables, self.control_force_tables, context.values)
        ####

    def moment_provider(self) -> AerodynamicCoefficientProvider | None:
        """Return a moment callback when all three moment tables are present."""

        normalized = {name.casefold() for name in self.moment_tables}
        if not normalized:
            return None
        if not {"cmx", "cmy", "cmz"}.issubset(normalized):
            raise ValueError("moment coefficient set must contain cmx, cmy, and cmz")
        tables = {name[2:]: table for name, table in self.moment_tables.items()}
        return lambda mach, alpha, beta: self._provider(tables, {"mach": mach, "alpha": alpha, "beta": beta}, ("x", "y", "z"))
        ####

    def context_moment_provider(self) -> ContextCoefficientProvider | None:
        """Return an extensible moment provider when moment tables exist."""

        normalized = {name.casefold() for name in self.moment_tables}
        if not normalized:
            return None
        if not {"cmx", "cmy", "cmz"}.issubset(normalized):
            raise ValueError("moment coefficient set must contain cmx, cmy, and cmz")
        tables = {name[2:]: table for name, table in self.moment_tables.items()}
        control_tables = {
            family: {name[2:]: table for name, table in members.items()}
            for family, members in self.control_moment_tables.items()
        }
        return lambda context: self._compose_provider(tables, control_tables, context.values, ("x", "y", "z"))
        ####

    def _compose_provider(
        self,
        base_tables: Mapping[str, PreparedCoefficientTable],
        control_tables: Mapping[str, Mapping[str, PreparedCoefficientTable]],
        values: Mapping[str, float],
        required: tuple[str, str, str] = ("cx", "cy", "cz"),
    ) -> Vector3:
        """Compose a static coefficient set with qualified control increments."""

        result = self._provider(base_tables, values, required)
        for members in control_tables.values():
            if not all(name in members for name in required):
                continue
            axes = tuple(
                axis
                for axis in members[required[0]].independent_variables
                if axis not in {"mach", "altitude_m", "alpha", "beta", "airspeed_m_s", "velocity_m_s"}
            )
            if len(axes) != 1:
                continue
            control_axis = axes[0]
            zero_values = dict(values)
            zero_values[control_axis] = 0.0
            current_vector = self._provider(members, values, required)
            zero_vector = self._provider(members, zero_values, required)
            result = result + (current_vector - zero_vector)
        return result
        ####

    def table_margins(self, values: Mapping[str, float]) -> Mapping[str, float]:
        """Return named distances to every force and moment table boundary."""

        result: dict[str, float] = {}
        for family, tables in (("force", self.force_tables), ("moment", self.moment_tables)):
            for name, table in tables.items():
                for axis, margin in table.margins(values).items():
                    result[f"{family}.{name}.{axis}"] = margin
        for family, control_tables in (
            ("force", self.control_force_tables),
            ("moment", self.control_moment_tables),
        ):
            for control_family, members in control_tables.items():
                for name, table in members.items():
                    for axis, margin in table.margins(values).items():
                        result[f"{family}.{control_family}.{name}.{axis}"] = margin
        return result
        ####
    ####


class RuntimeTableLike(Protocol):
    """Minimal structural contract accepted from runtime table lowering."""

    independent_variables: tuple[str, ...]
    output_variable: str
    prepared: PreparedTable | None
    ####


@dataclass(frozen=True, slots=True)
class TableAerodynamicModel:
    """Evaluate body-axis coefficient data against the native environment.

    The coefficient callback is deliberately table-shaped: inputs are Mach,
    angle of attack, and sideslip; outputs are body ``CX/CY/CZ`` coefficients.
    A future `.tbl` resolver can supply the callback without changing the
    rigid-body load contract.
    """

    environment: EnvironmentProvider
    earth_rotation: EarthRotationAdapter
    reference_area_m2: float
    reference_length_m: float
    coefficients: AerodynamicCoefficientProvider
    moment_coefficients: AerodynamicCoefficientProvider | None = None
    context_coefficients: ContextCoefficientProvider | None = None
    context_moment_coefficients: ContextCoefficientProvider | None = None
    control_provider: Callable[[RigidBody6DofState], Mapping[str, float]] | None = None
    alpha_reference_rad: float = 0.0
    table_margin_provider: Callable[[Mapping[str, float]], Mapping[str, float]] | None = None

    def __post_init__(self) -> None:
        if self.reference_area_m2 <= 0.0 or self.reference_length_m <= 0.0:
            raise ValueError("aerodynamic reference geometry must be positive")
        ####
    ####

    def evaluate(self, state: RigidBody6DofState) -> AerodynamicOutput:
        """Resolve environment, air data, and body loads for one state."""

        air_velocity_body = self.air_velocity_body(state)
        airspeed = air_velocity_body.norm()
        position_ecfc, earth_relative_velocity = self.earth_rotation.ecic_to_ecfc(
            state.position,
            state.velocity,
            time_seconds=state.time,
        )
        sample = self.environment.sample(time=state.time, position=position_ecfc)
        if sample.wind.frame is not Frame.ECFC:
            raise ValueError("environment wind must be expressed in ECFC")
        del earth_relative_velocity
        dynamic_pressure = 0.5 * sample.density * airspeed * airspeed
        speed_of_sound = sample.speed_of_sound
        mach = airspeed / speed_of_sound if speed_of_sound > 0.0 else 0.0
        # Canonical fixed-wing body axes are +X forward, +Y right, +Z down.
        # Positive alpha is the velocity component toward +Z (the vehicle is
        # pitched nose-up relative to the airflow).  Source-native rotor
        # frames use the separate DirectWrenchTableModel adapter below.
        angle_of_attack = math.atan2(air_velocity_body.z, max(abs(air_velocity_body.x), 1.0e-12))
        sideslip = math.atan2(air_velocity_body.y, max(math.hypot(air_velocity_body.x, air_velocity_body.z), 1.0e-12))
        context = AeroQueryContext.from_air_data(
            mach,
            angle_of_attack,
            sideslip,
            {
                # Numerical integration can place a state a few ulps below
                # the reference surface.  Geometric altitude for atmosphere
                # and table lookup has a physical floor at zero.
                "altitude_m": max(0.0, position_ecfc.vector.norm() - self.earth_rotation.earth.equatorial_radius.si_value),
                "airspeed_m_s": airspeed,
                "speed_of_sound_m_s": speed_of_sound,
                # Some historical local aerodynamic decks use alpha as an
                # offset from the published trim condition.  Preserve the
                # physical air-data observable while applying that convention
                # only to the table query.
                "alpha": angle_of_attack - self.alpha_reference_rad,
                **(self.control_provider(state) if self.control_provider is not None else {}),
            },
        )
        nonfinite_query = tuple(
            name for name, value in context.values.items() if not math.isfinite(float(value))
        )
        if nonfinite_query:
            raise ValueError(
                "aerodynamic query contains non-finite values: "
                + ", ".join(sorted(nonfinite_query))
            )
        force_coefficients = self.context_coefficients(context) if self.context_coefficients is not None else self.coefficients(mach, angle_of_attack, sideslip)
        if not all(math.isfinite(value) for value in (force_coefficients.x, force_coefficients.y, force_coefficients.z)):
            raise ValueError("aerodynamic force coefficients are non-finite")
        force_body = force_coefficients.scaled(dynamic_pressure * self.reference_area_m2)
        moment_coefficients = (
            self.context_moment_coefficients(context)
            if self.context_moment_coefficients is not None
            else self.moment_coefficients(mach, angle_of_attack, sideslip)
            if self.moment_coefficients is not None
            else Vector3(0.0, 0.0, 0.0)
        )
        moment_body = moment_coefficients.scaled(dynamic_pressure * self.reference_area_m2 * self.reference_length_m)
        if not all(math.isfinite(value) for value in (moment_coefficients.x, moment_coefficients.y, moment_coefficients.z)):
            raise ValueError("aerodynamic moment coefficients are non-finite")
        return AerodynamicOutput(
            force_body,
            moment_body,
            sample.density,
            dynamic_pressure,
            speed_of_sound,
            airspeed,
            mach,
            angle_of_attack,
            sideslip,
            dict(context.values),
            dict(self.table_margin_provider(context.values)) if self.table_margin_provider is not None else {},
        )
        ####
    ####

    def air_velocity_body(self, state: RigidBody6DofState) -> Vector3:
        """Return the current air-relative velocity resolved in body axes."""

        position_ecfc, _ = self.earth_rotation.ecic_to_ecfc(
            state.position,
            state.velocity,
            time_seconds=state.time,
        )
        sample = self.environment.sample(time=state.time, position=position_ecfc)
        if sample.wind.frame is not Frame.ECFC:
            raise ValueError("environment wind must be expressed in ECFC")
        air_velocity_ecic = self.earth_rotation.air_relative_velocity_ecic(
            state.position,
            state.velocity,
            sample.wind,
            time_seconds=state.time,
        ).vector
        return state.attitude.conjugate().rotate(air_velocity_ecic)
        ####

    def force_moment(self, state: RigidBody6DofState) -> RigidBodyForceMoment:
        """Adapt aerodynamic output to the rigid-body force/moment contract."""

        output = self.evaluate(state)
        return RigidBodyForceMoment(output.force_body_n, output.moment_body_nm)
        ####
    ####
####


@dataclass(frozen=True, slots=True)
class DirectWrenchTableModel:
    """Evaluate body-force/body-moment tables without coefficient scaling.

    This generic bridge is useful for rotorcraft and other models whose source
    deck reports dimensional loads directly.  The table names still use the
    existing six-component ``*aero`` assignment contract; ``aero-load-mode``
    selects whether those values are coefficients or SI loads.
    """

    environment: EnvironmentProvider
    earth_rotation: EarthRotationAdapter
    loads: PreparedAerodynamicCoefficients
    control_provider: Callable[[RigidBody6DofState], Mapping[str, float]] | None = None
    source_z_up: bool = False
    rotor_allocation: QuadRotorAllocation | None = None

    def evaluate(self, state: RigidBody6DofState) -> AerodynamicOutput:
        """Resolve direct wrench tables against body-relative velocity."""

        position_ecfc, _ = self.earth_rotation.ecic_to_ecfc(state.position, state.velocity, time_seconds=state.time)
        sample = self.environment.sample(time=state.time, position=position_ecfc)
        air_velocity_body = self.air_velocity_body(state)
        airspeed = air_velocity_body.norm()
        speed_of_sound = sample.speed_of_sound
        mach = airspeed / speed_of_sound if speed_of_sound > 0.0 else 0.0
        alpha = math.atan2(-air_velocity_body.z, max(abs(air_velocity_body.x), 1.0e-12))
        beta = math.atan2(air_velocity_body.y, max(math.hypot(air_velocity_body.x, air_velocity_body.z), 1.0e-12))
        source_velocity_z = -air_velocity_body.z if self.source_z_up else air_velocity_body.z
        controls = self.control_provider(state) if self.control_provider else {}
        rotor_commands = _rotor_commands_from_controls(controls, self.rotor_allocation)
        rotor_speed = rotor_commands.rms_speed_rad_s if rotor_commands is not None else controls.get("rotor_speed", 469.124102661955)
        query = AeroQueryContext.from_air_data(
            mach,
            alpha,
            beta,
            {
                "velocity_x": air_velocity_body.x,
                "velocity_y": air_velocity_body.y,
                "velocity_z": source_velocity_z,
                "airspeed_m_s": airspeed,
                "rotor_speed": rotor_speed,
                **{name: value for name, value in controls.items() if name.startswith("rotor-")},
            },
        )
        force = self.loads.context_force_provider()(query)
        moments = self.loads.context_moment_provider()
        moment = moments(query) if moments is not None else Vector3(0.0, 0.0, 0.0)
        allocation = self.rotor_allocation
        if rotor_commands is not None and allocation is not None:
            moment = moment + allocation.differential_moment_source(rotor_commands)
        if self.source_z_up:
            # RotorPy uses +Z for thrust/up; TAORYX rigid-body body Z is down.
            force = Vector3(force.x, force.y, -force.z)
            moment = Vector3(-moment.x, -moment.y, moment.z)
        return AerodynamicOutput(
            force,
            moment,
            sample.density,
            0.5 * sample.density * airspeed * airspeed,
            speed_of_sound,
            airspeed,
            mach,
            alpha,
            beta,
            dict(query.values),
            dict(self.loads.table_margins(query.values)),
        )
        ####

    def air_velocity_body(self, state: RigidBody6DofState) -> Vector3:
        """Return body-relative air velocity for direct-load queries."""

        position_ecfc, _ = self.earth_rotation.ecic_to_ecfc(state.position, state.velocity, time_seconds=state.time)
        sample = self.environment.sample(time=state.time, position=position_ecfc)
        air_velocity_ecic = self.earth_rotation.air_relative_velocity_ecic(state.position, state.velocity, sample.wind, time_seconds=state.time).vector
        return state.attitude.conjugate().rotate(air_velocity_ecic)
        ####
    ####
####


def _rotor_commands_from_controls(
    controls: Mapping[str, float],
    allocation: QuadRotorAllocation | None,
) -> RotorCommandSet | None:
    if allocation is None:
        return None
    names = ("rotor-1-speed", "rotor-2-speed", "rotor-3-speed", "rotor-4-speed")
    if not all(name in controls for name in names):
        return RotorCommandSet(*(controls.get(name, controls.get("rotor_speed", 469.124102661955)) for name in names))
    return RotorCommandSet(*(controls[name] for name in names))
    ####
