# Public value-space contract

Every public TAORYX parameter, action, status channel, observation, resource,
and objective value has both a unit and a **mathematical value space**. A unit
alone is insufficient: heading and heading rate may both use degrees, but
heading is periodic while heading rate is an ordinary signed scalar.

This contract is part of Mission Composition discovery and Simulation Runtime stepping. It tells
authoring tools how to validate an input, controllers how to form an error,
and AI/RL clients whether a value can be safely normalized, interpolated, or
compared componentwise.

## Public rule

Every exposed value must declare:

- canonical unit and representation;
- topology;
- error rule;
- interpolation rule;
- normalization/equivalence rule when it applies; and
- bounds or finite options where the interface owns them.

`topology_pending` is not a publishable contract. The catalog and topology
audit fail closed if a resolved registry parameter, interface channel,
observation binding, or truth-objective target lacks a reviewed value-space
entry.

## Common spaces

| Value kind | Topology | Error / interpolation rule | Examples |
| --- | --- | --- | --- |
| Signed scalar or vector | `euclidean` | ordinary subtraction / linear interpolation | north, east, altitude error, body-rate, angular acceleration, AoA rate |
| Bounded nonperiodic coordinate | `bounded_interval` | linear difference with explicit limit handling | bank command, elevator deflection, nacelle angle |
| Circular angle | `periodic_circle` | shortest wrapped signed difference / shortest-arc interpolation | heading, yaw, course |
| Nonnegative magnitude | `positive_half_line` | linear or an explicitly selected log-space comparison / no negative values | speed, mass, fuel quantity, dynamic pressure, thrust magnitude |
| Fraction | `unit_interval` | linear difference, explicit clamp or reject policy | throttle fraction, propellant fraction, battery state of charge |
| Attitude quaternion | `rotation_group_so3` | geodesic/log-map error / shortest-arc SLERP | body-to-navigation attitude |
| Unit direction | `unit_sphere` | angular/geodesic error / normalized spherical interpolation | gate normal, thrust direction |
| Enumerated state or event | `finite_set` or `event` | exact equality / not interpolable | flight mode, stage state, event code |
| Boolean | `boolean` | exact equality / not interpolable | contact, engine enabled, actuator health |
| Mixed tuple | `product` | component-specific rule / component-specific interpolation | Euler roll-pitch-yaw representation |

The important edge case is angular derivatives: heading, yaw, and azimuth live
on a circle; their rates and accelerations live on a line. A controller must
wrap `heading_error`, but it must not wrap `heading_rate_error`. Likewise, a
quaternion is not a four-dimensional Euclidean attitude error: `q` and `-q`
are the same physical attitude.

## Where the declarations live

- `verification/parameter_value_space_catalog.yaml` owns initialization,
  segment, and variant input contracts.
- `verification/interface_channel_value_space_catalog.yaml` owns public
  action, effector, status, observation, resource, and diagnostic channels.
- `verification/truth_objective_channel_value_space_catalog.yaml` owns
  family-independent objective target and tolerance dimensions.
- `src/taoryx/value_space.py` is the semantic implementation of validation,
  declared error rules, and interpolation rules.

The three catalogs are versioned source data rather than inferred from an ID
or a unit suffix. For example, metres do not imply a positive value: north
and east are signed coordinates, whereas altitude-related clearance and speed
are nonnegative magnitudes only when their declared meaning says so.

## How a caller discovers the contract

```bash
# Initialization, segment, and variant inputs include value_space metadata.
taoryx vehicle parameters skywalker_x8 --scope segment
taoryx vehicle parameters a320_openap_3dof --scope variant_configuration

# Per-fidelity action, effector, status, resource, and observation channels.
taoryx vehicle describe skywalker_x8
taoryx vehicle interface-report

# Catalog-wide completeness gate; it reports zero topology findings only when
# every published Mission Composition surface has an explicit reviewed contract.
taoryx vehicle topology-report
```

The emitted JSON is the authority. A UI should read `value_space`, not guess
from a label: use periodic widgets for headings, bounded sliders only where
bounds are declared, geodesic attitude error for quaternions, and no
interpolation for events, modes, or booleans.

## Timing and control implications

The topology contract does not authorize interpolation. Simulation Runtime publishes
truth at committed integration boundaries. A sensor, observer, or UI may
sample only its declared committed status rows; a command is held across an
explicit interval. `semantic_action_trace.json` is additionally checked
against `status_trace.json`, so both the interval start and end must be real
committed truth timestamps.

Consequently, a client receives two independent guarantees:

1. It knows the mathematical operation appropriate for each value.
2. It knows exactly when that value existed as committed truth.

Neither guarantee implies a physical-effector model or a qualified vehicle.
Those remain fidelity and evidence-tier claims.

## Addition checklist

Before exposing a new field, the family author must answer:

1. What physical/semantic quantity is this, in what canonical unit and frame?
2. Does it live on a line, circle, sphere, rotation group, finite set, or a
   product of those spaces?
3. What error is meaningful to guidance/control/evaluation?
4. Is interpolation meaningful at all? If yes, what is the declared rule?
5. Are bounds hard-valid, qualified, or merely UI guidance?
6. Is it committed truth, a held requested action, an achieved effector, a
   sensor observation, or a derived diagnostic?
7. Which registry/catalog entry makes the decision machine-readable?

A new field that cannot answer these questions stays private to its model
until its Runtime / Composition contract is ready.
