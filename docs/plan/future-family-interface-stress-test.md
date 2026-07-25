# Future-family interface stress test

This is a deliberately small Alpha 2 closeout tranche. It investigates the
future family backlog so the neutral contracts are strong enough for Alpha 3;
it does not pull every future vehicle implementation into Alpha 2.

## Families used as interface probes

| Probe | Requirements that must be representable |
| --- | --- |
| C172P-class | Propeller/engine, flap and gear configuration, fuel mass/CG/inertia, ground contact, stall envelope, absolute and rate controls. |
| R44, UH-1H, UH-60A | Cyclic/collective/pedal controls, scheduled derivative decks, power authority, rotorcraft modes, achieved control displacement, local trim conditions. |
| XV-15 and V-22-class | Nacelle state/rate, rotor/wing lift sharing, scheduled control allocation, forward/reverse conversion, abort/reversion, hysteresis, resource continuity. |
| Learjet 24-class | Source-linear versus bounded-proxy model modes, local validity envelopes, jet spool, source/estimated parameter separation. |
| Spacecraft | Epochs, orbital and body frames, reaction wheels/thrusters, propellant, battery, eclipse/resource modes, attitude/orbit fidelity separation. |
| Public surrogates | Battery/fuel/series-hybrid backends, payload/loadout composition, mission mode state machines, uncertainty bands, evidence grades. |

## Current contract coverage

The current neutral contract already provides a good base:

- three explicit fidelity profiles;
- immutable `CaseIntent` and `ResolvedCase`;
- canonical parameter values and provenance;
- absolute and requested-rate control inputs;
- authority modes and achieved actuator observations;
- resource and event-prediction observation kinds;
- one batch/step transition boundary.

The stress test identifies six concepts that should become first-class before
the Alpha 2 interface is frozen:

1. **Capabilities.** A family must declare whether it supports resources,
   modes, transitions, allocation, checkpointing, each fidelity, and each
   control/observation abstraction. Unsupported requests must fail at resolve.
2. **Components.** Engines, rotors, surfaces, actuators, payloads, tanks,
   batteries, and effectors need typed slots and provenance rather than being
   encoded as unrelated parameter names.
3. **Channel semantics.** Controls and observations need frame, semantic level
   (guidance, attitude/rate, or effector), neutral value, validity/availability,
   and achieved-state metadata.
4. **Allocation and actuation.** A requested cyclic, nacelle angle, wheel
   torque, or elevon command may map to several effectors. The contract must
   preserve requested, allocated, limited, and achieved values.
5. **Mode transitions.** Hybrid vehicles need typed entry/exit/abort guards,
   continuous schedules, controller handoff, hysteresis, and resource/state
   continuity. A free-form segment dictionary is not sufficient as the long-
   term validation surface.
6. **Evidence-aware parameters.** Source, identified, derived, estimated,
   synthetic, and unavailable values need typed status and uncertainty policy
   in the resolved case, not only in a separate human document.

## Alpha 2 versus Alpha 3 decision

### Must close in Alpha 2

- Define and test the neutral schemas for the six concepts above.
- Add capability-aware resolution diagnostics.
- Ensure metadata survives `ResolvedCase` identity and provenance.
- Prove that a future-family fixture can be rejected or resolved without a
  family-specific runner.
- Keep the canonical transition function unchanged by future-family intake.

### Implement in Alpha 3

- C172, helicopter, tiltrotor, spacecraft, Learjet, and surrogate providers.
- Nonlinear rotor, inflow, engine, governor, and ground-effect models.
- Full source correlation and flight-test replay.
- Fleet, corpus, weather, sensor, and large search infrastructure.

## Exit evidence

The stress tranche is complete when a machine-readable compatibility report
shows each probe family mapped to typed contract fields, declared extension
points, or an explicit unsupported diagnostic. The report should include both
positive and negative cases and must not claim that a family is implemented just
because its interface can be described.
