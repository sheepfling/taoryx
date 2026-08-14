# TAORYX F-16 S.119 plug-in

`taoryx-f16` owns the source-grounded F-16 S.119 family, including its
3DOF and pseudo-6DOF reductions, local physical controller screens, source
assets, and focused Composition provider.  It depends on the Taoryx core and
the DAVE-ML plug-in, but not on `taoryx-reference-models`.

The published interface distinguishes reduced guidance control from the
bounded local physical-control screens.  The latter are local development
evidence, not route-level flight qualification.

## Versioned discovery

The package/provider release is `0.1.0a0`; the runnable F-16 model advertises
model version `1.0.0+composition-v1`.  A focused host can compare
`catalog.plugin_revision("taoryx.f16")` to detect a package update without
invalidating its view because another installed vehicle changed.  Provider,
model, configuration-schema, and output-schema fingerprints remain available
for UI, agent, and cache invalidation.

Discovery is metadata-only: it publishes the F-16 family-adapter, trim,
capability, preflight, execution, controller-screen, and interface identities
without importing the source plant, reduced runtime, interface implementation,
or physical-screen implementations. Those modules resolve only when their
specific model contract is selected.

It also publishes `taoryx.f16.vehicle-catalog`, a typed root for the exact
F-16 package-data fragment. When a caller supplies the selected F-16 plug-in
catalog to the generic composition compiler, catalog records, profiles,
bindings, and source assets are resolved only from this wheel; a missing asset
fails closed instead of falling back to the compatibility aggregate.
Its two registered batch/episode parity declarations and their two replay
inputs are scoped the same way, so the focused parity gate cannot borrow a
trace from another vehicle package.

## Composition and controls

Use the focused provider when authoring or streaming an F-16 composition:

```bash
taoryx model list --provider taoryx.f16.mission-composition
taoryx model plan taoryx.f16.mission-composition f16_s119 \
  --fidelity pseudo_6dof --realization pseudo_6dof \
  --mission powered_fixed_wing_racetrack_v1
```

Both reduced racetrack endpoints advertise the following inspectable authority
families:

- `kinematic_guidance` for speed, flight-path angle, heading, and bank;
- `reduced_pilot_command` for a more direct reduced control surface; and
- `live_waypoint_guidance` when the route mode and current flight phase make
  waypoint updates available.

The pseudo-6DOF endpoint additionally advertises `body_rate_command`. The
F-16 wheel owns that addition through its lazy
`vehicle_interface_extension`; the core only joins and validates the generic
contract. This keeps F-16-specific rate-reference semantics out of the common
fixed-wing interface while preserving the same advertised contract for UI and
agent clients.

The authoring plan and runtime status frames identify the selected authority,
available channels, native units, and any availability/readback limits. A
consumer should select an advertised authority rather than infer one from a
physical-effector name.

For the interactive `body_rate_command` profile, the three accepted commands
read back on the corresponding `body_rate.roll`, `body_rate.pitch`, and
`body_rate.yaw` status channels.  Those scalars are streaming response-law
readbacks, not an F-16 flight-control-system or effector-allocation claim. The
status binding accepts the exact scalar episode value or the indexed committed
batch rate vector; a route never fabricates the other representation. The
batch result retains the same reduced motion through standard
`angular_rate.body.x`, `.y`, and `.z` output channels.

The direct-wrench and surface-allocated endpoints retain source-local LQR/LQI
screens. They expose bounded elevator, aileron, rudder, and throttle overlays
with local controller metadata, but do not claim a full-envelope controller,
route-following qualification, or an aircraft-level flight-control system.

## Focused verification

Run the package gate while changing this family:

```bash
python tools/dev.py check-vehicle f16_s119
python tools/verify_plugin_wheels.py --plugin f16
```

The first command also verifies that the package-data fragment is an exact
fresh extract of its canonical F-16 source, evidence, and witness inputs:

```bash
python tools/extract_f16_plugin_assets.py --check
```

`check-vehicle f16_s119` intentionally covers only the point-mass 3DOF and
pseudo-6DOF interfaces, their four reduced endpoint witnesses, and their two
registered parity traces. It discovers only `taoryx.f16`, and the selected
catalog remains attached when the ordinary common batch/session APIs resolve
the provider. The source-local direct-wrench, surface LQR/LQI, and schedule
campaigns remain in `tests/unit/test_f16_vehicle_integration.py`; run that
separate evidence suite when changing the physical plant, controller, or
screen contract.

The wheel smoke installs only the core, DAVE-ML, and F-16 wheels in a
temporary target and proves that F-16 does not recover its runtime or package
data from the compatibility aggregate.
