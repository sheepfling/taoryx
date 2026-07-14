# Vehicle Data Model

This note captures the higher-level data families that sit above the current
TAOS `.tbl` and `.prb` grammar. The historical TAOS table system already
supports interpolation-based aero, propulsion, and center-of-gravity inputs;
this document groups the additional data needed for richer successor models so
they can be researched, digested, and backlog-tracked in one place.

## Scope

The current TAOS-compatible surface is still a segment-scoped point-mass
system. That means the canonical runtime inputs are still:

- aerodynamic coefficients;
- thrust magnitude;
- mass flow;
- center of gravity;
- user-defined tabulated variables.

The successor-side model extends that with data that is useful for
air-breathing propulsion and control effectors, but it does not claim that the
historical TAOS executable accepted these structures directly.

## Core Data Families

### 1. Reference geometry

Geometry provides the static shape and reference locations that coefficient and
mass-property data need to be interpreted consistently.

Typical fields:

- vehicle length;
- body diameter or other reference dimensions;
- aerodynamic reference area;
- reference length;
- aerodynamic moment reference point;
- station map for nose, stages, engines, fins, wings, and control surfaces;
- axis and sign conventions.

### 2. Mass properties

Mass properties should cover both the point-mass minimum and the richer rigid
body case.

Minimum point-mass fields:

- total mass versus time or mission state;
- stage separation discontinuities;
- propellant depletion.

Richer rigid-body fields:

- center of gravity vector;
- inertia tensor;
- products of inertia;
- mass-property changes across staging and fuel burn.

### 3. Propulsion

Propulsion should be split into families rather than forced into a single
thrust-versus-time abstraction.

Rocket-family inputs:

- thrust versus time or mission state;
- mass-flow rate;
- thrust-vector direction;
- stage ignition, burnout, and separation timing.

Air-breathing-family inputs:

- thrust deck versus Mach, altitude, throttle, and engine state;
- fuel flow deck versus the same operating conditions;
- installation losses;
- inlet recovery or operability limits;
- throttle lag or engine-state dynamics.

### 4. Aerodynamic effectors

Effectors should be modeled as incremental changes to aerodynamic coefficients
rather than as direct force generators.

Representative effectors:

- elevator;
- aileron;
- rudder;
- flap;
- slat;
- spoiler;
- speed brake;
- canard;
- missile fin;
- landing gear;
- drag chute.

Effectors may be represented as:

- linear control derivatives near trim;
- nonlinear increment tables versus Mach, angle of attack, sideslip, and
  deflection;
- full configuration-specific coefficient decks for large deployable devices.

## Recommended Digestion Split

Use these buckets when importing or curating data:

- `source-faithful-tbl`: historical TAOS-compatible tables with no semantic
  reinterpretation.
- `synthetic-reference`: software-development tables that exercise a feature
  family but are not flight-validated.
- `generated-engine-deck`: thrust or effector data derived from an external
  model and documented with its generator inputs.
- `measured-public-data`: released wind-tunnel, engine, or flight-test data
  with explicit provenance.
- `successor-schema`: manifest-level data that may not map one-to-one to a
  legacy `.tbl` file.

## Suggested File Layout

```text
vehicle/
  geometry.yaml
  mass_properties.csv
  propulsion/
    rocket_stage_1.csv
    turbojet_deck.csv
    thrust_vector_control.yaml
  aerodynamics/
    base_coefficients.csv
    elevator_effect.csv
    aileron_effect.csv
    rudder_effect.csv
    flap_effect.csv
    spoiler_effect.csv
```

## Current Interpretation Rules

- Rocket and air-breathing propulsion should share a common force-and-mass-flow
  output interface, but not a common table shape.
- Control surfaces should change coefficient increments, not bypass the
  aerodynamic model.
- A 6-DOF implementation should treat moments, inertia, and application points
  as first-class data; a point-mass implementation does not need them.
- Synthetic tables should be labeled synthetic until a documented historical or
  measured source exists.

## Backlog Use

This note is the landing zone for future parser, catalog, and fixture work on:

- air-breathing engine decks;
- control-surface increment decks;
- configuration-dependent aero polars;
- mass-property histories that include inertia;
- vehicle manifests that aggregate the above into one importable bundle.
