# Airbreathing racetrack fidelity ladder

The powered-fixed-wing racetrack is one reusable mission contract.  X8 and
B747 bindings use the same ordered phases and truth gates while scaling the
straight length, turn radius, speed, and vertical rates to the vehicle.

```text
outbound climb → outbound level → left semicircle
→ inbound descent → inbound level → right semicircle → start/finish gate
```

The advertised dynamics tiers are explicit:

| Tier | Runtime mode | What the packet proves | What it does not prove |
|---|---|---|---|
| 3DOF | `point-mass` | Translational route, altitude, speed, and terminal gate | Attitude, moments, or surfaces |
| pseudo-6DOF | `kinematic-6dof` | The same translation plus a named prescribed-attitude lag/rate sidecar | Moment-derived attitude or physical surface allocation |
| rigid 6DOF — direct/induced wrench | `rigid-body-6dof` | Coupled translation/rotation, source loads, and a declared direct body force/moment controller | Physical surface/rotor allocation |
| rigid 6DOF — surface allocated | `rigid-body-6dof` | Coupled translation/rotation with declared effectors producing the control moment | More actuator fidelity than the binding declares |

The current X8 surface-allocated binding uses bounded local elevon allocation.
The X8 direct-wrench packet is retained as a diagnostic comparison.  The B747
rigid witness uses source-table aerodynamic loads, source-moment/side-force
cancellation, a direct local-bank/local-pitch PD moment request, and an
explicit beta-proportional plus lateral-velocity damping force.  Its
surface-allocated tier is explicitly unavailable until coherent aileron,
elevator, rudder, and actuator data are bound.

## Run the complete four-tier catalog

From the repository root:

```bash
PYTHONPATH=src python3 tools/build_airbreathing_racetrack_ladder.py
```

This runs every available packet and records unavailable tiers in the ladder
manifest instead of silently omitting them. The output is written to
`artifacts/showcases/airbreathing-racetrack-fidelity-ladder/`.  Its
`summary.json` records one status and one reproduction command per packet.

## Run one vehicle at every tier

```bash
PYTHONPATH=src python3 tools/build_airbreathing_racetrack_ladder.py --vehicle x8
PYTHONPATH=src python3 tools/build_airbreathing_racetrack_ladder.py --vehicle b747
```

## Run one exact tier

```bash
# X8
PYTHONPATH=src python3 tools/build_airbreathing_racetrack_ladder.py --vehicle x8 --fidelity 3dof
PYTHONPATH=src python3 tools/build_airbreathing_racetrack_ladder.py --vehicle x8 --fidelity pseudo-6dof
PYTHONPATH=src python3 tools/build_airbreathing_racetrack_ladder.py --vehicle x8 --fidelity 6dof-direct-wrench
PYTHONPATH=src python3 tools/build_airbreathing_racetrack_ladder.py --vehicle x8 --fidelity 6dof-surfaces

# B747
PYTHONPATH=src python3 tools/build_airbreathing_racetrack_ladder.py --vehicle b747 --fidelity 3dof
PYTHONPATH=src python3 tools/build_airbreathing_racetrack_ladder.py --vehicle b747 --fidelity pseudo-6dof
PYTHONPATH=src python3 tools/build_airbreathing_racetrack_ladder.py --vehicle b747 --fidelity 6dof-direct-wrench
PYTHONPATH=src python3 tools/build_airbreathing_racetrack_ladder.py --vehicle b747 --fidelity 6dof-surfaces
```

Each packet contains the resolved binding, preflight timing estimate, truth
telemetry, objective report, mission sequence, qualification board, plots, and
`reproduction.txt`.  The packet builder independently evaluates the truth
gates; the controller does not certify its own route transitions.

## Current nominal packet status

The checked-in ladder is intentionally honest about capability maturity:

| Vehicle | 3DOF | pseudo-6DOF | rigid direct/induced wrench | rigid surface allocated |
| --- | --- | --- | --- | --- |
| X8 | pass | pass | pass | pass |
| B747 | pass | pass | nominal case pass; qualification pending | unavailable — no coherent surface bundle |

The B747 direct-wrench packet now completes all four truth-evaluated gates and
the source-bounded +/-5 degree accepted beta envelope with the verified Euler
cadence.  Its table has an explicitly estimated +/-0.12-radian extension only
for intermediate integrator queries.  It remains a nominal-case result rather
than a source-qualified transport controller, and the surface tier is not
represented by a direct-moment substitute.

## Inspect a packet

```bash
cat artifacts/showcases/airbreathing-racetrack-fidelity-ladder/summary.json
cat artifacts/showcases/airbreathing-racetrack-fidelity-ladder/x8-racetrack-altitude-turns-pseudo-6dof-v1/summary.json
open artifacts/showcases/airbreathing-racetrack-fidelity-ladder/x8-racetrack-altitude-turns-pseudo-6dof-v1/qualification_board.png
```

A `mission_pass` value means that the nominal truth objectives and numerical
run passed.  It is not, by itself, a family-level robustness or source-
validation badge.
