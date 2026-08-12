# Control semantics and agent action spaces

The Mission Composition control advertisement has two separate concerns:

1. **Value semantics** describe what values are legal and how they behave
   mathematically.
2. **Command semantics** describe how a caller's value enters time-stepped
   execution.

Presentation metadata such as `slider`, `toggle`, `dial`, `stepper`, `button`,
and `select` is only a UI hint. A slider may represent a continuous throttle,
a periodic bearing, or a detented discrete knob; consumers must use the
semantic contract instead.

## Scheme, authority, and action are separate

A control scheme is the small cross-family concept a UI, remote client, or
autonomy service can compare. An authority profile is the exact selectable
surface supplied by one realization. An action channel is one typed coordinate
inside that profile. Consumers must not infer a scheme from an action name.

| Layer | Stable scheme examples | Meaning |
| --- | --- | --- |
| Open loop/provider | `open_loop.coast`, `provider.program` | No caller action, with ownership still explicit |
| Mission | `mission.destination`, `mission.waypoint`, `mission.route` | A high-level objective lowered by registered guidance |
| Kinematic | `kinematic.position`, `kinematic.velocity`, `kinematic.flight_path`, `kinematic.energy` | Reduced translational or energy response without physical-effector claims |
| Pilot | `pilot.normalized_axes`, `pilot.rotorcraft`, `pilot.ground_vehicle`, `pilot.marine` | Family-specific human/controller-oriented axes |
| Body motion/event | `body_motion.attitude`, `body_motion.body_rate`, `event.mission` | Rotational references or explicitly typed mission events |
| Physical bridge | `wrench.direct`, `effector.direct` | Expert/diagnostic coordinates with separate qualification requirements |
| Provider/debug | `provider.native_bridge`, `debug.*` | Explicit nonportable or nonphysical surfaces |

The catalog reserves `mission.orbit_target`, `mission.relative_pose`, and
`kinematic.relative_motion` for future spacecraft work. Reservation is not an
availability claim. Likewise, the reserved rotorcraft, ground-vehicle, and
marine pilot schemes do not pretend their collective/cyclic/pedals,
steering/braking, or rudder/propulsion axes are fixed-wing pilot axes; an
eventual profile must publish its family-specific channels and lowering chain.

Every support record includes a scheme ID and layer, intended consumer roles,
streaming preference, UI order, exact realization and fidelity IDs, authority
profile ID, operations, action IDs, ownership, switching policy, and claim
boundary. `wrench.direct` and `effector.direct` are deliberately marked
`diagnostic`; generic clients should prefer mission, kinematic, pilot, or body-
motion schemes when those are available.

## Value domains

`ControlCommandSemantics.value_domain` covers the public control families:

| Domain | Typical example | Required contract |
| --- | --- | --- |
| `continuous` | unbounded derivative or bounded throttle | numeric type, optional interval |
| `periodic` | heading or bearing | periodic value space and period |
| `boolean` | enable/disable switch | boolean type and exact-value semantics |
| `enum` | propulsion mode | named choices and exact-value semantics |
| `discrete_levels` | three-position gimbal detent | numeric quantization levels or finite step grid |
| `event` | stage separation or fire command | named event choices and event command mode |
| `vector` | attitude or multi-axis command | vector shape and component value space |
| `provider_defined` | heterogeneous adapter-only effector | explicit provider-owned topology and claim boundary |

Bounds are independent of topology. An omitted lower or upper endpoint means
the corresponding side is unbounded. A `rate` command additionally declares
`rate_unit`; it is not inferred from the display unit.

## Command lifecycle

`ControlCommandSemantics` also declares:

- `command_mode`: `absolute`, `rate`, `increment`, or `event`;
- `temporal_semantics`: `held`, `sampled`, `profile`, `momentary`, `latched`,
  or `pulse`;
- `release_behavior`: `hold`, `default`, `failsafe`, `release_value`, or
  `auto_reset`;
- `pulse_duration_s` and `repeat_policy` for one-shot/pulse actions;
- `quantization` for numeric steps or explicit detents; and
- the agent normalization policy, including declared standardization center
  and scale when required.

This keeps a rocket separation event distinct from a boolean “separation
enabled” level, and keeps a latched switch distinct from a momentary control.
Event repetition and action masks are part of the agent projection so replay
and training cannot silently fire a once-only event twice.

## RL/Torch projection

The dependency-free projection is built from the authoritative advertisement:

```python
from taoryx.trajectory import build_rl_action_space

action_space = build_rl_action_space(realization.controls, operation="step")
torch_metadata = action_space.torch_spec()
```

The projection publishes stable advertised channel order, encoding, tensor
dtype, shape, native bounds, agent bounds, discrete values, normalization
policy, mask requirements, and whether external training statistics are still
required. The mapping is:

| Control | Agent encoding |
| --- | --- |
| finite bounded numeric | `box`, affine normalized to `[-1, 1]` |
| unbounded numeric | `box`, identity unless standardization is declared |
| periodic scalar | `box`, canonical wrapping through the declared period |
| boolean | binary action |
| enum or detented numeric | discrete action index |
| event | discrete event index with repeat/mask metadata |
| mixed realization | dictionary of per-channel spaces |

`encode_agent_action` and `decode_agent_action` provide deterministic
boundary conversion without importing Torch or Gymnasium. A Torch adapter can
construct its policy heads from `torch_spec()` and should preserve the
advertised action mask and channel IDs in replay records.

This metadata describes an action interface and normalization contract. It
does not qualify a policy, choose training statistics, or promote the physical
fidelity of the underlying vehicle.

## Contract-probe vehicle

`taoryx.debug.mission-composition-contract-probe` is the required consumer
conformance fixture. Its `contract_probe_vehicle` model is available through
the same Mission Composition provider registry as production vehicles, but is
explicitly synthetic. Its batch surface advertises every value domain, command
and temporal mode, release behavior, quantization mode, sampling state, and
agent normalization form. Its coarse and medium realizations also expose
persistent step profiles for continuous/vector commands, discrete/enum/boolean
commands, and one-shot events. The recursive provider-defined composite
effector remains batch-only because no truthful scalar/vector live lowering
exists for it.

Use `realization.controls.rl_action_space(operation="batch")` against that
model before assuming a consumer understands a physical vehicle's narrower
surface. The generated action space is intentionally mixed and includes
one-shot event masks plus an unbounded channel that requires external training
statistics. For streaming conformance, select exactly one advertised authority
profile through the Mission Composition session API; do not union the profiles
into one action vector. The probe declares `physical_model=false`, so generic
controller automation treats these transport stress surfaces as
non-applicable to tuning and qualification.
