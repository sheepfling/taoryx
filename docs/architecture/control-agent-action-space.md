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
explicitly synthetic and batch-only. Its medium realization advertises every
value domain, command and temporal mode, release behavior, quantization mode,
sampling state, and agent normalization form. It also contains a recursive
provider-defined composite effector whose value space has typed component
spaces.

Use `realization.controls.rl_action_space(operation="batch")` against that
model before assuming a consumer understands a physical vehicle's narrower
surface. The generated action space is intentionally mixed and includes
one-shot event masks plus an unbounded channel that requires external training
statistics.
