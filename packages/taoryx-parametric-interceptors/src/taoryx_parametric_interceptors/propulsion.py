"""Shared resolved propulsion schedule for both interceptor fidelity tiers."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum

from .profile import ResolvedInterceptorProfile
from .thrust_schedule import ThrustProfileSchedule


class PropulsionPhase(StrEnum):
    """Stable phase labels emitted through the Composition output contract."""

    SINGLE_PULSE = "single_pulse"
    FIRST_PULSE = "first_pulse"
    INTER_PULSE_COAST = "inter_pulse_coast"
    SECOND_PULSE = "second_pulse"
    BURNOUT = "burnout"

    ####


@dataclass(frozen=True, slots=True)
class PropulsionSample:
    """One evaluated motor-schedule sample."""

    mass_kg: float
    propellant_remaining_kg: float
    thrust_n: float
    throttle_command: float
    throttle_achieved: float
    available: bool
    phase: PropulsionPhase
    pulse_index: int


####


@dataclass(frozen=True, slots=True)
class PropulsionProgram:
    """Executable single- or dual-pulse program from a resolved profile.

    ``nominal_thrust_n`` is the active-burn mean. Profile shapes and the
    second-pulse ratio are normalized to preserve the corresponding nominal
    total impulse. Propellant allocation remains independently visible.
    """

    architecture: str
    thrust_profile_schedule: ThrustProfileSchedule
    second_pulse_thrust_profile_schedule: ThrustProfileSchedule
    launch_mass_kg: float
    burnout_mass_kg: float
    nominal_thrust_n: float
    first_pulse_burn_time_s: float
    inter_pulse_coast_time_s: float
    second_pulse_burn_time_s: float
    second_pulse_thrust_ratio: float
    second_pulse_propellant_fraction: float

    @classmethod
    def from_profile(cls, profile: ResolvedInterceptorProfile) -> PropulsionProgram:
        """Build the common runtime program from advertised resolved values."""

        program = cls(
            architecture=profile.text("propulsion_architecture"),
            thrust_profile_schedule=profile.thrust_profile_schedule,
            second_pulse_thrust_profile_schedule=(profile.second_pulse_thrust_profile_schedule or profile.thrust_profile_schedule),
            launch_mass_kg=profile.number("launch_mass_kg"),
            burnout_mass_kg=profile.number("burnout_mass_kg"),
            nominal_thrust_n=profile.number("thrust_n"),
            first_pulse_burn_time_s=profile.number("first_pulse_burn_time_s"),
            inter_pulse_coast_time_s=profile.number("inter_pulse_coast_time_s"),
            second_pulse_burn_time_s=profile.number("second_pulse_burn_time_s"),
            second_pulse_thrust_ratio=profile.number("second_pulse_thrust_ratio"),
            second_pulse_propellant_fraction=profile.number("second_pulse_propellant_fraction"),
        )
        program._validate()
        return program
        ####

    @property
    def active_burn_time_s(self) -> float:
        return self.first_pulse_burn_time_s + self.second_pulse_burn_time_s
        ####

    @property
    def duration_s(self) -> float:
        return self.active_burn_time_s + self.inter_pulse_coast_time_s
        ####

    @property
    def propellant_mass_kg(self) -> float:
        return self.launch_mass_kg - self.burnout_mass_kg
        ####

    @property
    def nominal_total_impulse_ns(self) -> float:
        return self.nominal_thrust_n * self.active_burn_time_s
        ####

    def sample(self, time_s: float) -> PropulsionSample:
        """Evaluate mass, thrust, and schedule feedback at nonnegative time."""

        if not math.isfinite(time_s) or time_s < 0.0:
            raise ValueError("propulsion sample time must be finite and nonnegative")
        pulse_one_end = self.first_pulse_burn_time_s
        pulse_two_start = pulse_one_end + self.inter_pulse_coast_time_s
        program_end = pulse_two_start + self.second_pulse_burn_time_s
        pulse_one_propellant = self.propellant_mass_kg * (1.0 - self.second_pulse_propellant_fraction)
        pulse_two_propellant = self.propellant_mass_kg - pulse_one_propellant

        if _strictly_before(time_s, pulse_one_end):
            fraction = time_s / self.first_pulse_burn_time_s
            consumed = pulse_one_propellant * self.thrust_profile_schedule.cumulative_fraction_at(fraction)
            phase = PropulsionPhase.FIRST_PULSE if self.second_pulse_burn_time_s > 0.0 else PropulsionPhase.SINGLE_PULSE
            return self._active_sample(
                mass_kg=self.launch_mass_kg - consumed,
                phase=phase,
                pulse_index=1,
                fraction=fraction,
                pulse_thrust_ratio=1.0,
                schedule=self.thrust_profile_schedule,
            )
        if self.second_pulse_burn_time_s > 0.0 and _strictly_before(time_s, pulse_two_start):
            return self._inactive_sample(
                mass_kg=self.launch_mass_kg - pulse_one_propellant,
                phase=PropulsionPhase.INTER_PULSE_COAST,
            )
        if self.second_pulse_burn_time_s > 0.0 and _strictly_before(time_s, program_end):
            fraction = (time_s - pulse_two_start) / self.second_pulse_burn_time_s
            consumed = pulse_one_propellant + pulse_two_propellant * self.second_pulse_thrust_profile_schedule.cumulative_fraction_at(fraction)
            return self._active_sample(
                mass_kg=self.launch_mass_kg - consumed,
                phase=PropulsionPhase.SECOND_PULSE,
                pulse_index=2,
                fraction=fraction,
                pulse_thrust_ratio=self.second_pulse_thrust_ratio,
                schedule=self.second_pulse_thrust_profile_schedule,
            )
        return self._inactive_sample(mass_kg=self.burnout_mass_kg, phase=PropulsionPhase.BURNOUT)
        ####

    def _active_sample(
        self,
        *,
        mass_kg: float,
        phase: PropulsionPhase,
        pulse_index: int,
        fraction: float,
        pulse_thrust_ratio: float,
        schedule: ThrustProfileSchedule,
    ) -> PropulsionSample:
        raw_factor = schedule.multiplier_at(fraction) * pulse_thrust_ratio
        normalization = (self.first_pulse_burn_time_s + self.second_pulse_burn_time_s * self.second_pulse_thrust_ratio) / self.active_burn_time_s
        thrust = self.nominal_thrust_n * raw_factor / normalization
        peak_raw_factor = max(
            self.thrust_profile_schedule.normalized_peak,
            self.second_pulse_thrust_profile_schedule.normalized_peak * self.second_pulse_thrust_ratio,
        )
        return PropulsionSample(
            mass_kg=max(self.burnout_mass_kg, mass_kg),
            propellant_remaining_kg=max(0.0, mass_kg - self.burnout_mass_kg),
            thrust_n=thrust,
            throttle_command=1.0,
            throttle_achieved=raw_factor / peak_raw_factor,
            available=True,
            phase=phase,
            pulse_index=pulse_index,
        )
        ####

    def _inactive_sample(self, *, mass_kg: float, phase: PropulsionPhase) -> PropulsionSample:
        return PropulsionSample(
            mass_kg=mass_kg,
            propellant_remaining_kg=max(0.0, mass_kg - self.burnout_mass_kg),
            thrust_n=0.0,
            throttle_command=0.0,
            throttle_achieved=0.0,
            available=False,
            phase=phase,
            pulse_index=0,
        )
        ####

    def _validate(self) -> None:
        numeric = (
            self.launch_mass_kg,
            self.burnout_mass_kg,
            self.nominal_thrust_n,
            self.first_pulse_burn_time_s,
            self.inter_pulse_coast_time_s,
            self.second_pulse_burn_time_s,
            self.second_pulse_thrust_ratio,
            self.second_pulse_propellant_fraction,
        )
        if any(not math.isfinite(value) for value in numeric):
            raise ValueError("resolved propulsion values must be finite")
        if self.launch_mass_kg <= self.burnout_mass_kg or self.burnout_mass_kg <= 0.0:
            raise ValueError("resolved propulsion requires launch mass above positive burnout mass")
        if self.nominal_thrust_n <= 0.0 or self.first_pulse_burn_time_s <= 0.0:
            raise ValueError("resolved propulsion requires positive thrust and first-pulse duration")
        if self.inter_pulse_coast_time_s < 0.0 or self.second_pulse_burn_time_s < 0.0:
            raise ValueError("resolved propulsion times must be nonnegative")
        if self.second_pulse_burn_time_s > 0.0:
            if self.architecture != "dual_pulse_solid":
                raise ValueError("only dual_pulse_solid may execute a second pulse")
            if self.second_pulse_thrust_ratio <= 0.0:
                raise ValueError("dual-pulse thrust ratio must be positive")
            if not 0.0 < self.second_pulse_propellant_fraction < 1.0:
                raise ValueError("dual-pulse propellant fraction must lie strictly between zero and one")
        elif any(
            value != 0.0
            for value in (
                self.inter_pulse_coast_time_s,
                self.second_pulse_thrust_ratio,
                self.second_pulse_propellant_fraction,
            )
        ):
            raise ValueError("single-pulse programs require zero-valued second-pulse settings")
        if not math.isclose(
            self.thrust_profile_schedule.cumulative_fraction_at(1.0),
            1.0,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ):
            raise ValueError("resolved thrust profile schedule must preserve unit normalized impulse")
        if not math.isclose(
            self.second_pulse_thrust_profile_schedule.cumulative_fraction_at(1.0),
            1.0,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ):
            raise ValueError("resolved second-pulse thrust profile schedule must preserve unit normalized impulse")
        ####

    ####


def _strictly_before(value: float, boundary: float) -> bool:
    return value < boundary and not math.isclose(value, boundary, rel_tol=0.0, abs_tol=1.0e-12)
    ####


__all__ = ["PropulsionPhase", "PropulsionProgram", "PropulsionSample"]
####
