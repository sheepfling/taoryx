# Hypersonic Lifting-Body Rocket Scenarios

## Decision

The X-15 and HL-20 belong to one comparison family, but they remain distinct
vehicle models. The X-15 is a rocket-powered research aircraft with an
unpowered post-burn glide. The HL-20 is an unpowered lifting-body entry/glider
concept. A rocket is therefore a launch-parent composition for both models,
not a propulsion model added to either child vehicle.

## Composition contract

Each scenario has two lineage-linked models:

```text
rocket parent
    -> accepted release event
        -> X-15 or HL-20 child
            -> unpowered entry/glide
```

The release event must retain the parent and child IDs, accepted time,
pre-release parent state, child initial state, and provenance for both source
data and scenario assumptions. The child must not inherit an implied engine,
controller, or route-success claim from the parent.

## Fidelity ladder

### R1: boost and release point-mass: verified witness

Use a point-mass parent and child to establish release altitude, velocity,
flight-path angle, energy, mass, and event lineage. This is the first shared
case for X-15 and HL-20. The first HL-20 witness is implemented by
`taoryx.hl20_reachability` with a synthetic booster parent and the pinned
HL-20 fixed mass/reference area. Its drag and lift-to-drag values remain
explicit low-fidelity assumptions.

### R2: pseudo-6-DOF release: executable witness

Add attitude and rate response surrogates. Verify release alignment, bounded
bank response, and continuity across the event. This remains a surrogate and
does not claim source-native actuator dynamics.

### R3: rigid-body 6-DOF release: executable witness

The HL-20 scenario now uses the native rigid-body state and integrator for its
retained parent and its typed detached-body event. Its aerodynamic loads remain
the explicit fixed-CD/fixed-L/D low-fidelity surrogate; coupling the full
byte-pinned DAVE-ML force/moment graph is the next fidelity promotion, not an
implicit claim of source-exact trajectory equivalence. The case remains
direct/open loop until a separately qualified controller is added.

### R4: native rigid-body plus logical surface allocation: executable witness

The fourth ladder tier retains the native rigid-body state and adds a bounded
seven-channel logical surface allocator. It records requested and achieved
bank/pitch/yaw channels, residual, saturation, and the resolved logical
deflections. The overlay is deliberately not coupled to source-declared
actuator dynamics and does not make a controller claim.

### R5: energy-managed entry/glide

Only after R4 is stable, add explicit energy-management segments and compare
normalized altitude/Mach, specific energy, range, crossrange, and termination
disposition. Route success is not a default acceptance criterion.

R1-R4 are now executable synthetic witnesses and are qualified by
`HL20-G0` through `HL20-G6`. The remaining implementation is tracked in
[`hl20-post-g6-program.md`](hl20-post-g6-program.md): P1 couples the source
DAVE-ML force/moment graph, P2 qualifies controls and trim, P3 adds
energy-managed segments, P4 makes terminal route goals executable, and P5
closes robustness and evidence.

## Initial cases

| Case | Parent | Child | Initial evidence |
|---|---|---|---|
| X-15 rocket release | synthetic or existing booster | X-15 public research surrogate | existing native showcase |
| HL-20 rocket release | generic staged booster | HL-20 Mod K source-grounded plant | four-tier synthetic witness qualified through HL20-G6 |

The HL-20 CA-HI route contract remains separate from this executable release
witness. The current runtime implementation is an open-loop release/glide
trajectory across all four fidelity tiers, not a target-hit or landing
simulation. Its reproducible showcase emits three evidence boards: the
boost/coast/glide flight board, a cross-fidelity capability board, and a
spent-cylinder deployment/tumble board. The associated JSON and CSV artifacts
retain the exact search grid, per-candidate disposition, controls, and child
telemetry used to render those boards.

## Nonclaims

- No historical X-15 or HL-20 rocket trajectory reconstruction is implied.
- A synthetic rocket parent does not make the child source-exact as a mission.
- HL-20 controller, thermal, landing, and terminal guidance claims remain out
  of scope until independently sourced and qualified.
