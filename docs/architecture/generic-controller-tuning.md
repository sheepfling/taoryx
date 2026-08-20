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
normalized Q/R profile grid
        |
        v
LQR candidates -> linear, nonlinear-response, and mission gates
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
- either explicit `GenericLqrProfile` candidates or a bounded
  `NormalizedLqrProfileGrid`;
- optional maneuver metrics and `AutoTuneLimits`.

The utility accepts arbitrary named state and control dimensions. It is not
limited to the six-state/three-moment attitude bridge used by the legacy
registered-vehicle convenience function. This allows the same pipeline to
serve fixed-wing surfaces, rotorcraft rates, spacecraft attitude actuators,
rocket gimbals, and reduced-order response models.

## Candidate generation policy

The first controller-design action for a newly integrated controlled family is
not a multi-day manual gain search. `TuningCampaign` performs a fixed sequence
at each declared operating point:

```text
trim
  -> two-step derivative-consistency check
  -> declared-axis authority check
  -> closed nested-loop projection check, where applicable
  -> normalized LQR candidate grid
  -> declared linear and nonlinear-response gates
```

`NormalizedLqrProfileGrid` generates a small Cartesian lattice of relative
tracking and control-effort priorities *after* the family has declared state
and control scales. The default grid is nine candidates: tracking multipliers
`[0.25, 1, 4]` crossed with effort multipliers `[2, 1, 0.25]`. This removes
unexplained absolute Q/R constants from ordinary integrations while keeping
the search bounded, reproducible, and auditable. A family may supply base
weights only to express an intentional axis priority, such as pitch versus
yaw; it must not use the grid as a substitute for an unavailable effector or
an uncontrolled axis.

The winner is only a `candidate_ready` design-screen result. It is promoted
only after the family-specific local nonlinear-response evaluator, allocator,
actuator, and mission gates pass. This prevents a numerically stable gain from
being presented as a flight-control solution.

The checked lower-tier witnesses demonstrate that a multirotor and a
powered-fixed-wing family now use this identical sequence without manual gain
selection:

```text
PYTHONPATH=src .venv/bin/python tools/validate_reduced_tuning_campaigns.py
PYTHONPATH=src .venv/bin/python tools/validate_reduced_tuning_campaigns.py --check
```

The same inputs now cross the installable plug-in boundary as typed campaign
registrations and can be discovered and executed without importing a
vehicle-specific runner:

```text
taoryx model plan taoryx.a320.mission-composition a320_openap_3dof \
  --fidelity pseudo_6dof
taoryx model tune taoryx.a320.mission-composition a320_openap_3dof

taoryx model plan taoryx.hummingbird.mission-composition hummingbird \
  --fidelity pseudo_6dof
taoryx model tune taoryx.hummingbird.mission-composition hummingbird

taoryx model plan taoryx.f16.mission-composition f16_s119 \
  --fidelity pseudo_6dof
taoryx model tune taoryx.f16.mission-composition f16_s119 \
  --fidelity pseudo_6dof \
  --campaign f16-pseudo-source-trim-attitude-v1

taoryx model tune taoryx.x15.mission-composition x15 \
  --campaign x15-source-release-direct-wrench-v1
```

The core owns `run_tuning_campaign`; the reference-model plug-in owns the
adapter and campaign factories. See
[Model-to-mission authoring and automation](../developer/model-authoring-automation.md) for
the complete authoring and registration path.

Hummingbird's pseudo-6DOF hover-attitude inner loop, the A320 pseudo-6DOF
cruise-attitude inner loop, both F-16 source-trim reductions, and the X-15
source-release direct-wrench bridge each generate the nine normalized
candidates. Their claim boundary remains local candidate design only: the
F-16 reductions do not prove physical surface allocation, and the X-15 bridge
does not represent a physical stabilator/rudder/RCS/propulsion controller.

For an example of a registered candidate becoming a reusable scheduled-control
execution path, the F-16 surface LQR campaign also serves the bounded
`f16_local_physical_surface_lqr_schedule_transition_screen_v1` endpoint. The
core schedule runner interpolates the controller demand; the F-16 plug-in
supplies the explicit blend of validated source derivative/effectiveness nodes
and the bounded physical allocator. That division keeps a plug-in author from
having to reimplement scheduling mechanics while preserving the family-owned
physics and claim boundary.

## Persistent disturbance and mass-variation policy

A fixed local LQR regulates to its nominal trim; it generally leaves a steady
tracking error when the plant sees a constant matched disturbance, such as a
steady wind load, thrust bias, or a small center-of-gravity/weight mismatch.
The common first response is **scheduled LQI**, not unconstrained adaptation:

```text
declared mass / inertia / flight-condition point
        -> family-provided local A/B and trim
        -> scheduled LQI gain
        -> integrate declared output error (y - reference)
        -> allocator / actuator / plant
```

`solve_continuous_lqi` augments the supplied plant only with explicitly named
output-error integrators. `LqiController` requires a reference and accepted
sample interval, freezes integration when its new command saturates, exposes
the integral state for telemetry, and resets only on an explicit mode or
segment transition. The full augmented `Q` is input data: no integral weight
or tracked output is inferred from a vehicle name.

`GainScheduledLqiController` combines that offset-free loop with the existing
family-owned mass/inertia schedule builder. The builder must use a declared
mass-property source; core never infers inertia from mass. A schedule point is
therefore evidence of the actual local plant, rather than a gain-scaling rule
hidden in a controller. Each plug-in must test its declared wind/disturbance,
mass, inertia, saturation, and schedule-transition envelope before claiming
T6.

The A320 pseudo-cruise and Hummingbird pseudo-hover campaigns now use scaled
LQI candidate grids. Each explicitly declares the inner-loop outputs whose
errors are integrated and their augmented-Q weights: A320 integrates bank;
Hummingbird integrates roll, pitch, and yaw. These are local attitude-loop
screens, not an assertion that wind rejection is solved at the route or
position loop.

The Hummingbird physical vertical endpoint adds a separate retained local
mass-mismatch proof: it independently re-trims the source plant at 85%, 100%,
and 115% of the declared source mass while holding one nominal-mass LQI design
fixed. This tests the actual collective/moment rotor allocation and motor lag
under those discrete cases; it is deliberately not described as a gain
schedule, a payload envelope, or an in-flight mass transition.

RSLQR remains a future promotion path rather than a synonym for LQI. An
adaptive augmentation likewise remains deferred until it declares an update
law, parameter projection bounds, excitation conditions, freeze/fallback
behavior, and comparison against the scheduled LQI baseline. The currently
implemented LQI API rejects constant *matched* disturbances; it does not
claim robustness to arbitrary time-varying or unmatched dynamics.

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
| Skywalker X8 | T4 source-coordinate allocation result | collective/differential elevon source-table coordinates with declared travel, lag, and rate bounds plus the source-paper left/right conversion | Servo wiring, hinge sign, mapped actuator telemetry, and end-to-end physical racetrack evidence remain separate gates. |
| Hummingbird | T5 local hover result | four individual motor-speed commands, source rotor thrust/reaction torque, and 5 ms motor lag | Attitude/rate recovery only; waypoint, contact, battery, and envelope cases are still separate work. |
| B747 | T5 local condition-3 result | elevator, aileron, rudder, and installed-engine throttle trim; moments allocated through the three aerodynamic surfaces | One clean condition, ideal-declared servo response, and no schedule/fuel/high-lift claim. |
| X-15 | local direct-wrench candidate | source release/glide load witness with bounded generalized wrench feedback | This is not a physical LQR: physical stabilator/rudder/RCS/propulsion allocation remains blocked because source actuator/gearing/rate data are incomplete. |

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
