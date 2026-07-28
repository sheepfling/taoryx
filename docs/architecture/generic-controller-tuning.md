# Generic controller tuning pipeline

Taoryx tuning is an adapter-driven workflow, not a collection of vehicle
specific gain edits. A family adapter supplies the equations, frames, units,
operating-point contract, and actuator semantics. The shared tuning utilities
perform the numerical work:

```text
family plant adapter
        |
        v
bounded TrimSpec + residual evaluator
        |
        v
TrimResult
        |
        v
true state-derivative evaluator
        |
        v
finite-difference A/B linearization
        |
        v
dimension-matched Q/R profile sweep
        |
        v
LQR candidates -> stability/uncertainty/mission gates
        |
        v
controller realization -> allocator -> actuator -> plant
```

The reusable entry point is
`taoryx.generic_tuning.trim_linearize_and_tune`. It requires:

- a bounded `TrimSpec`;
- a trim residual evaluator;
- a dynamics evaluator returning derivatives named exactly like the trim
  states;
- positive state and control scaling vectors;
- one or more `GenericLqrProfile` objects;
- optional maneuver metrics and `AutoTuneLimits`.

The utility accepts arbitrary named state and control dimensions. It is not
limited to the six-state/three-moment attitude bridge used by the legacy
registered-vehicle convenience function. This allows the same pipeline to
serve fixed-wing surfaces, rotorcraft rates, spacecraft attitude actuators,
rocket gimbals, and reduced-order response models.

## What remains family-specific

The shared tuner must not invent these items:

- aerodynamic, rotor, propulsion, or spacecraft equations;
- state and control frames or units;
- trim residual meaning;
- source-table interpolation and validity domains;
- actuator limits and rate laws;
- surface, rotor, thruster, or gimbal allocation;
- mission objectives and terminal contracts.

For example, a B747 adapter may expose elevator/aileron/rudder controls and a
source-backed aerodynamic derivative evaluator. A Hummingbird adapter may
expose body rates and motor-speed controls through a quadrotor allocator. The
same `trim_linearize_and_tune` call structure applies, but the resulting
matrices and claims remain family-specific.

## Tuning stages and claim boundaries

1. **Trim** proves only that the declared operating point satisfies the
   adapter's residual contract.
2. **Linearization** is plant-backed only when the dynamics evaluator returns
   true state derivatives. A force or moment residual Jacobian is not silently
   promoted to an `A/B` model.
3. **LQR synthesis** screens profiles for poles, conditioning, derivative
   uncertainty, and optional operational metrics.
4. **Allocation and actuator realization** determine whether the requested
   generalized control can be produced physically.
5. **Mission tuning** evaluates route, altitude, energy, saturation, and
   terminal objectives outside the local LQR solve.

The generic report therefore identifies failed prerequisites instead of
falling back to a tuned initial guess. A stable LQR candidate is not a vehicle
or mission qualification result.

## Four reference-aircraft status

The tracked binding catalog and generated report apply this contract to the
current four reference aircraft:

```text
PYTHONPATH=src python3 tools/tune_reference_aircraft.py
PYTHONPATH=src python3 tools/tune_reference_aircraft.py --check
```

Each entry is deliberately marked `tuning_stage: generic_lqr_screen` and
`qualification_status: screen_only`. B747, Skywalker X8, and X-15 currently
use a runtime-inertia attitude bridge with direct generalized moments. The
Hummingbird entry uses the declared quad-X rotor allocation around hover.
Those results prove that the shared Q/R/profile machinery is dimensionally
valid and locally stable for the declared bridge; they do not prove source
aircraft `A/B` derivatives, surface/rotor actuator dynamics, or mission-level
flight-control qualification.

## Recommended family adapter contract

Each vehicle family should eventually provide:

```text
resolve_operating_point(parameters) -> TrimSpec
evaluate_trim(state, controls) -> residual mapping
evaluate_derivatives(state, controls) -> state-derivative mapping
allocate(control demand, state) -> achieved effectors + residual
evaluate_maneuver(controller) -> metrics and envelope margins
```

The last two functions are intentionally outside the LQR solver. This keeps
direct-wrench debugging available while making physical surface, rotor, wheel,
thruster, or gimbal realization explicit when the family supports it.

## B747 consequence

The B747 condition-3 source-surface adapter now uses this generic pipeline.
It solves trim through elevator, aileron, rudder, and installed-engine
throttle; derives a plant-backed nine-state local `A/B`; evaluates normalized
gentle, standard, and aggressive profiles; and retains every candidate in
`verification/generated/b747_condition3_physical_surface_lqr.json`. The
selected local controller reaches T5 only because its moment requests are
allocated back through actual source-table surface coordinates and checked in
the nonlinear plant.

That does not turn the existing hand-tuned direct-moment racetrack into a
physical controller. It remains a T0 integration witness. The next B747 work
is to repeat the same generic adapter pipeline at adjacent cruise and descent
nodes, then test the transitions before any scheduling claim.

## Four-vehicle physical-realization ledger

The generic screen is intentionally retained for debugging and scaling, but it
is not the strongest evidence for the four reference vehicles. Their current
promotion state is tracked separately so a clean direct-wrench screen cannot
be mistaken for actuator-realizable control:

| Vehicle | Current highest evidence | Actual control path | Boundary |
| --- | --- | --- | --- |
| Skywalker X8 | T3 local table-coordinate result | collective/differential elevon source-table coordinates with declared travel, lag, and rate bounds | Public left/right elevon sign/gearing is unresolved, so it is not a hardware-surface claim. |
| Hummingbird | T5 local hover result | four individual motor-speed commands, source rotor thrust/reaction torque, and 5 ms motor lag | Attitude/rate recovery only; waypoint, contact, battery, and envelope cases are still separate work. |
| B747 | T5 local condition-3 result | elevator, aileron, rudder, and installed-engine throttle trim; moments allocated through the three aerodynamic surfaces | One clean condition, ideal-declared servo response, and no schedule/fuel/high-lift claim. |
| X-15 | T0 structural readiness | source stabilator/rudder coordinates are inventoried, but no physical LQR is synthesized | Both source-bounded trim attempts fail; XLR99 is a time history rather than a throttle map, and RCS/gearing/rate data are incomplete. |

Regenerate the vehicle-specific evidence before consuming the ledger:

```text
PYTHONPATH=src python3 tools/validate_x8_physical_lqr.py
PYTHONPATH=src python3 tools/validate_hummingbird_physical_lqr.py
PYTHONPATH=src python3 tools/validate_b747_physical_lqr.py
PYTHONPATH=src python3 tools/validate_x15_physical_lqr.py
PYTHONPATH=src python3 tools/validate_four_vehicle_control_evidence.py
```

The last command is deliberately successful when it writes
`blocked_before_T1_trim`; that is a verified missing prerequisite, not a
controller failure that should trigger direct-wrench substitution. The final
command writes `verification/generated/four_vehicle_control_evidence.json`, a
compact ledger of the highest earned tier and exact control path for all four
vehicles.
