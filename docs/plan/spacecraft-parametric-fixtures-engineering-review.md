# Spacecraft parametric fixtures engineering review

This review updates the spacecraft milestone with corrected notional baselines.
The values below are source-anchored engineering proxies, not exact subsystem
specifications. Public facts, component-class anchors, formula-derived values,
geometry estimates, and Taoryx choices remain separately labeled.

## Revised fixture identities

```text
spacecraft.6u_observer_rw.standard.v1
spacecraft.6u_observer_rw.resilient.v1
spacecraft.agile_imager_rw.v1
spacecraft.spheres_like_rcs.v1
spacecraft.marco_like_hybrid.v1
```

The earlier `spacecraft.6u_hybrid_deepspace.v1` name remains a compatibility
alias for the MarCO-like fixture until manifests and examples migrate.

## Revised baselines

| Fixture | Revised nominal | Qualification emphasis |
| --- | --- | --- |
| 6U standard observer | 10.2 kg; approximately 0.37 × 0.24 × 0.11 m; inertia `[0.060, 0.127, 0.165] kg·m²`; 3 × 0.004 N·m / 0.015 N·m·s wheels; 3 × 0.6 A·m² torquers | Magnetic detumble, wheel handoff, pointing, saturation, unloading |
| 6U resilient observer | Same bus; 4 skewed 0.007 N·m / 0.050 N·m·s wheels | Fault-tolerant authority and wheel-out behavior with real mass/power cost |
| Agile imager | 45 kg; 0.60 × 0.45 × 0.40 m; inertia `[1.36, 1.95, 2.11] kg·m²`; 4 × 0.025 N·m / 0.5 N·m·s wheels | 20–25 second large slews, torque versus momentum envelope |
| SPHERES-like free flyer | 4.635 kg; inertia `[0.0258, 0.0225, 0.0230] kg·m²`; 12 × 0.13 N pulsed thrusters; 172 g CO₂ | Six-axis impulse control, pulse quantization, allocation, relative motion |
| MarCO-like hybrid | 14 kg; inertia `[0.085, 0.173, 0.225] kg·m²`; 3-wheel XACT-15-class ADCS; 8 × 25 mN thrusters; 755 N·s total impulse | Wheel pointing, long-duration low-thrust correction, misalignment rejection, unloading |

## Lifecycle corrections

The standard 6U observer must not hand worst-case deployment rates directly to
small reaction wheels:

```text
deployment at 0–5 deg/s
→ magnetic B-dot detumble
→ wheel handoff at or below 1 deg/s
→ wheel-controlled Sun acquisition and pointing
```

The qualification target for magnetic detumble is 20–60 minutes, not the ideal
instantaneous-torque lower bound. A 30-degree standard 6U slew should target
12–25 seconds; a 90-degree safe-mode slew should target 30–90 seconds.

The resilient 6U configuration must account for the mass and peak-power cost of
four higher-capacity wheels. Skewed-wheel torque and momentum capability must be
computed from the wheel-axis matrix, never estimated as four times one wheel.

SPHERES is classified as a `microgravity_lab_freeflyer`, not automatically as a
generic orbital spacecraft. Its public geometry and pulsed six-axis controller
are valuable references, but ISS-interior pressure, thermal, plume, and
propellant-life assumptions must not silently become orbital claims.

The MarCO-like hybrid is a long-duration, low-thrust spacecraft:

```text
RCS captures initial rates
→ reaction wheels provide ordinary pointing
→ RCS unloads wheel momentum
→ four 25 mN thrusters execute multi-minute corrections
→ wheels reject residual thrust misalignment
→ wheels reacquire precise pointing
```

It is not a fast proximity free flyer. The approximately 1.92 kg propellant
value implied by 755 N·s and 40 s specific impulse is formula-derived, not a
published tank-load claim.

## Actuator qualification rules

For reaction-wheel families, generated cases must calculate capture momentum,
slew torque, and slew momentum from the actual inertia and wheel-axis geometry.
At minimum require:

- capture momentum margin ≥ 1.5;
- slew torque margin ≥ 1.25;
- slew momentum margin ≥ 1.5;
- ADCS mass and peak power within bus allocations;
- unloading completion within the mission interval;
- one-wheel-out authority when advertised.

For thruster families, calculate force, moment, minimum impulse bit, and
propellant from physical positions and directions. At minimum require:

- positive and negative authority on every advertised axis;
- nonnegative attainable-wrench allocation;
- pulse resolution adequate for station keeping;
- propellant and total impulse reserve;
- valid allocation under CG migration;
- power and duty-cycle compliance;
- failed-thruster authority when advertised.

## Milestone fixture tests

The revised milestone adds these tests to the common spacecraft qualification
packet:

1. 6U deployment tipoff sweep and magnetic detumble-to-wheel handoff.
2. Standard versus resilient 6U mass, power, momentum, and wheel-out comparison.
3. Agile imager slew-time and wheel-polytope envelope sweep.
4. SPHERES 10 ms effective pulse, approximately 6 ms valve-delay, and
   six-axis allocation regression.
5. SPHERES microgravity station-keeping and minimum-impulse characterization.
6. MarCO-like 25 mN burn-duration, total-impulse, and wheel-reacquisition test.
7. MarCO-like thrust-misalignment and RCS unloading test.
8. Formula-derived sizing checks and rejection reports for invalid generated
   actuator systems.

The milestone is not complete until each fixture reports the source/evidence
label for every revised value, preserves rejected samples, and distinguishes
orbital, microgravity-lab, and deep-space claims.
