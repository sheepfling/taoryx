# Generic control and segment contracts

TAORYX uses one Python control lifecycle across the vehicle families while
keeping the historical `.prb` language unchanged.

```text
observation -> controller -> demand -> allocator -> command -> plant -> loads
      ^                                                        |
      +---------------- telemetry / segment event ------------+
```

The contracts live in `taoryx.control`:

- `VehicleObservation` is the canonical controller input;
- `Controller` produces named `ControlDemand` values;
- `ControlAllocator` maps demands into actuator `ControlCommand` values;
- `VehiclePlant` returns a `PlantEvaluation` containing force, moment, mass
  flow, and diagnostics;
- `SegmentSchedule` and `SegmentController` own lifecycle resets and segment
  transitions.

## Family adapters

`taoryx.control_adapters` supplies the first adapters:

| Family | Plant adapter | Allocator |
| --- | --- | --- |
| Point-mass 3-DOF | `FunctionalPlantAdapter` | `DirectControlAllocator` |
| Kinematic 3+3 | `FunctionalPlantAdapter` | `DirectControlAllocator` |
| B747 / X8 rigid body | `RigidBodyPlantAdapter` | direct or LQR-backed controls |
| Hummingbird | `RigidBodyPlantAdapter` | `QuadRotorControlAllocator` |
| X-15 | `RigidBodyPlantAdapter` | direct stabilator/throttle controls |

The adapters receive already-resolved table, frame, unit, mass, and
propulsion evaluators. They do not reinterpret source data and do not add
vehicle-specific syntax. That keeps convention checking at the plant boundary
and keeps controller code reusable.

## Controller design methods

Controller synthesis is separate from the plant and allocator contracts. A
`ControllerDesignSpec` names the design method, the trim artifact, the state
channels, the control channels, and the allocator. The first implemented
factory is `build_lqr_controller`; `pid`, `mpc`, `pole_placement`,
`dynamic_inversion`, and `rule_based` are valid catalog methods reserved for
additional factories.

```text
source plant -> TrimResult -> local A/B -> design method -> allocator -> plant
```

The factory rejects a design whose state/control names do not exactly match the
trim artifact. This prevents a gain or controller configuration from being
silently reused across vehicles. LQR is therefore the first controller, not a
permanent architectural dependency.

When mass properties vary, use `GainScheduledLqrController` with an explicit
operating-point builder. The builder receives the current mass and inertia;
the controller never infers inertia from mass. The catalog's controller scale
contract records `mass_scale_kg`, `inertia_scale_kg_m2`, `force_scale_n`,
`weight_moment_scale_nm`, and `inertia_moment_scale_nm`. A profile may opt
into nominal-ratio mass conditioning; the current inertia still comes from the
plant's source-backed mass-property provider. For estimated aerodynamic
derivatives, attach an `LqrUncertaintySpec` and screen the fixed gain across
the declared `A/B` envelope before promoting a maneuver result.

## Directional convention probes

`audit_control_directions` in `taoryx.control_directions` probes a named control
at `+delta` and `-delta` around the same plant condition. It reports the signed
central response, antisymmetry error, and declared expected sign. Vehicle
adapters should bind it to dimensional body-frame force/moment outputs before
accepting a trim or controller result. A positive or negative response is not
interpreted globally: the expected sign belongs to the vehicle's source
convention contract.

The standard entry points are:

```bash
./.venv/bin/python tools/dev.py trim-vehicles
./.venv/bin/python tools/dev.py control-directions
```

The trim command currently covers the source-backed B747, X8, and Hummingbird
solvers. X-15 remains source-trimmed until its independent plant residual is
formalized. Direction contracts intentionally fail when a source sign is not
yet declared; they do not infer a sign from controller success.

## Runtime integration

`InteractiveSession` accepts a `segment_controllers` mapping keyed by runtime
vehicle name. At every accepted time-step it publishes a
`VehicleObservation`, invokes `SegmentController.step`, applies the resulting
commands through the existing `ControlSpec` bounds and slew limits, and keeps
the effective request in the replay stream. Controller saturation is visible
in the snapshot diagnostics and remains available to the normal interactive
and `RunArtifact` output paths.

Existing `.prb` scenarios remain unchanged when no segment controller is
provided. This is an opt-in Python/runtime bridge, not a new problem-language
feature.

## LQR-first provenance retrofit

The primary regulator is selected through the controller realization contract;
it is not selected by hidden scenario gains. The active path is recorded as:

```text
guidance/reference
  -> LQR-family regulator
  -> generalized demand
  -> allocation/mixer
  -> actuator limits/dynamics
  -> plant
```

`ControllerProvenance`, `GuidanceReference`, `GeneralizedControlRequest`,
`AllocationResult`, and `ControllerRuntimeState` provide the machine-readable
boundary for this path. A runtime record identifies the design and matrix
hashes, active trim and schedule, requested demand, allocated command,
achieved actuator state, saturation/rate limiting, and transition events.

Existing rule-based or PID-like paths are retained as explicit baseline
controllers. A scenario containing gain overrides can be reproduced and
compared, but cannot receive `controller_qualified`. Use
`tools/audit_controller_inventory.py` to verify that every active family path
is classified before migrating it to LQR/RSLQR/LQI.

Backend selection is likewise explicit. `ControllerBackendRegistry` resolves a
named implementation during case construction; it does not branch on vehicle
or route names. The default registry exposes the implemented fixed-point LQR,
declared gain-scheduled/RSLQR/LQI promotion points, and the regression-only
`legacy_pid_baseline`. Declared-but-unimplemented backends fail closed when a
factory is requested, and legacy baselines are never qualification eligible.

### Rate-loop realization and rotor allocation

Rigid-body cases may declare a separate `*runtime lqr rate` block with the
ordered body-rate state `(wx, wy, wz)` and moment inputs
`(moment-x, moment-y, moment-z)`. This is a rate regulator, not an attitude
regulator with its angle states removed by convention. Its default plant
linearization is the dimensional body-rate relation

```text
dot([wx, wy, wz]) = diag(1/Ix, 1/Iy, 1/Iz) [moment-x, moment-y, moment-z]
```

When a rotor allocator is declared, the LQR request is converted from the
canonical rigid-body moment frame into the source wrench frame before rotor
allocation. For the Hummingbird source `+Z`-up convention, source roll and
pitch moments reverse sign while source yaw retains sign; the direct-wrench
model then applies the inverse load-frame conversion exactly once. The
requested canonical moment, individual rotor commands, saturation state, and
realization metadata are retained as runtime evidence.

If no rate LQR is declared, the historical rotor-rate-damping path remains
available as an explicit regression baseline. Declaring a rate LQR disables
that duplicate damping term so the active rate authority has one owner.
