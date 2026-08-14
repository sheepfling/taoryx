# Taoryx Hummingbird plug-in

`taoryx-hummingbird` owns the AscTec Hummingbird multirotor family: its
source-table data, reduced pseudo-6DOF model, physical rotor screens,
composition metadata, control campaigns, witnesses, and execution factories.
It depends only on the Taoryx host contracts and can be installed without the
legacy reference-model aggregate.

## Versioned discovery

The package/provider release is `0.1.0a0`; its runnable model advertises
model version `1.0.0+composition-v1`. A focused client can compare
`catalog.plugin_revision("taoryx.hummingbird")` to detect an update without
invalidating its model view because another vehicle changed. Provider, model,
configuration-schema, and output-schema fingerprints remain available for UI,
agent, and cache invalidation.

Its packaged data includes `package_data_provenance.json`, which identifies
the family-owned registry, source-table, witness, and local-screen fragment.
The repository-root registries remain canonical; the package manifest does not
turn its local physical-controller evidence into a flight-qualification claim.

`taoryx.hummingbird.vehicle-catalog` is the typed root for that data. Under a
selected Hummingbird catalog, generic composition, endpoint/variant/graph
witness, and parity-replay loaders can resolve only its package rows; a
missing normal resource fails closed instead of borrowing a sibling or the
compatibility aggregate.

Discovery keeps the local direct-wrench definition static: it advertises its
identity, bounded wrench controls, and claim boundary without importing the
source-hover configuration, numerical rotor tables, or runtime. Those are
resolved only if that physical screen is selected; the reduced interface,
capabilities, preflights, and batch/session implementations follow the same
deferred boundary.

## Composition and controls

Use the focused provider for authoring or streaming the reduced Hummingbird:

```bash
taoryx model list --provider taoryx.hummingbird.mission-composition
taoryx model plan taoryx.hummingbird.mission-composition hummingbird \
  --fidelity pseudo_6dof --realization pseudo_6dof \
  --mission multirotor_pad_box_yaw_recovery_land_v1
```

The pseudo-6DOF endpoint advertises three selectable, `explicit_bumpless`
authority profiles through the same session API:

- `body_motion_response` for held roll, pitch, yaw, aggregate-thrust fraction,
  and motor enable;
- `velocity_yaw_command` for north/east/positive-up velocity, yaw, and motor
  enable; and
- `live_waypoint_guidance` for an updatable local waypoint, capture radius,
  speed limits, yaw, and motor enable.

The plug-in owns these controls, profiles, committed state readbacks,
aggregate-thrust feedback, battery/mass resources, and realization diagnostics
through a lazy `vehicle_interface_extension`. Core joins and validates that
additive contract; it does not hard-code a Hummingbird model name. The public
contract therefore exposes action units, bounds, lowering chains, availability,
and feedback bindings for UI and agent consumers without presenting individual
rotor speed, motor current, thrust-vector, or physical allocation authority.

Each committed status frame includes position, velocity, attitude, body rate,
contact, aggregate-thrust, waypoint state, mass, battery reserve, control
realization, and physical-allocation status. These are pseudo-plant truth or
derived response-law values, not flight qualification or blade-resolved rotor
evidence. A client should use the published control feedback and authority
availability to identify saturation, depletion, or unavailable commands rather
than assume each requested value was achieved.

The source-local individual-rotor LQI, horizontal/vertical translation, and
direct-wrench screens remain separate physical development evidence. They are
not folded into the reduced streaming contract and do not claim a general
flight controller, route qualification, battery model, or full multirotor
envelope.

## Focused verification

Run the package gate while changing the reduced public interface:

```bash
python tools/dev.py check-vehicle hummingbird
python tools/verify_plugin_wheels.py --plugin hummingbird
```

The focused check begins by verifying that the package-data fragment is an
exact fresh extract of its canonical Hummingbird source tables, witnesses, and
registry rows:

```bash
python tools/extract_hummingbird_plugin_assets.py --check
```

`check-vehicle hummingbird` intentionally discovers only
`taoryx.hummingbird`, checks the pseudo-6DOF interface, executes the two
pseudo endpoint witnesses, replays its one parity trace, and runs the short
advertisement/batch/session vertical test. The selected catalog remains
attached through the common provider, batch/session APIs, witness preflight,
lowering, and parity replay. The physical rotor/direct-wrench screens stay in
`tests/unit/test_hummingbird_vehicle_integration.py`; run that separate evidence
suite only when their plant, controller, or screen contract changes.
