# TAORYX A320/OpenAP plug-in

`taoryx-a320` owns the executable A320 family products: the OpenAP
point-mass 3DOF performance model, the explicitly surrogate OpenAP/JSBSim
pseudo-6DOF response model, their reduced guidance runtime, local
named-coordinate LQI screen, and A320-only package data.

The plug-in is not a manufacturer A320 model and does not claim physical
surface allocation, actuator dynamics, or aircraft qualification. Its
point-mass and pseudo-6DOF products expose only the guidance/control authority
and readback that their declared lower-fidelity contracts support.

Use the focused provider when composing an A320 run:

```bash
taoryx model plan taoryx.a320.mission-composition a320_openap_3dof \
  --fidelity pseudo_6dof --realization jsbsim_surrogate_composite_pseudo6dof \
  --mission powered_fixed_wing_racetrack_v1
```

The batch-only native-coordinate LQI screen is a separate endpoint. Its plan
advertises the retained campaign, exact aileron/elevator/rudder coordinate
bounds, cadence, output/readback boundary, and the fact that it is neither a
physical A320 surface allocator nor caller-owned action channel:

```bash
taoryx model plan taoryx.a320.mission-composition a320_openap_3dof \
  --fidelity pseudo_6dof --realization jsbsim_surrogate_composite_pseudo6dof \
  --mission a320_local_native_coordinate_lqi_screen_v1
taoryx plugins inspect taoryx.a320 --no-builtin
```

The inspection result includes the package release/API versions and its scoped
`taoryx.plugin-revision/v1` fingerprint. Model plans carry the corresponding
model metadata revision, so a UI or agent can refresh this family without
invalidating unrelated model plug-ins.

## Versioned focused runtime

The package/provider release is `0.1.0a0`; the runnable A320 model advertises
model version `1.0.0+composition-v1`. A focused client can compare
`catalog.plugin_revision("taoryx.a320")` with its cached revision, then use
the provider/model/configuration/output-schema fingerprints to refresh only
this family.

The provider retains the exact catalog that discovered it. That scope follows
the ordinary batch and session APIs, execution preflight/lowering, witnesses,
and batch/episode parity replay; opening A320 does not rediscover sibling
vehicle plug-ins. The two public low-fidelity tiers advertise the same
semantic control families where supported: `kinematic_guidance`,
`reduced_pilot_command`, and `live_waypoint_guidance`. Each advertises units,
bounds, availability, lowering chain, and committed feedback so a UI or agent
can select a profile rather than infer controls from unavailable surfaces.

`taoryx.a320.vehicle-catalog` is the typed root for that package-data scope.
When callers select `taoryx.a320`, generic composition, execution-witness,
and parity-replay loaders resolve only this wheel's catalog rows and replay
inputs; a missing normal resource fails closed rather than falling back to a
sibling or compatibility aggregate.

Focused discovery also retains the native-coordinate LQI screen's static
identity, bounds, cadence, and claim boundary without importing NumPy, SciPy,
the OpenAP/JSBSim surrogate plant, or the numerical LQI runtime. The screen's
configuration factory resolves that implementation only after the exact
batch-only endpoint is selected.

The focused gate intentionally covers only the point-mass and pseudo-6DOF
reduced interfaces, their four normal batch/episode witnesses, and two parity
traces. The batch-only native-coordinate LQI screen remains separate physical
development evidence; it is not rerun when changing the streaming API.

For the family inner loop, run:

```bash
python tools/dev.py check-vehicle a320_openap_3dof
python tools/verify_plugin_wheels.py --plugin a320
```

The first command also verifies that package data is an exact fresh extract of
the canonical A320 inputs:

```bash
python tools/extract_a320_plugin_assets.py --check
```

The package retains the immutable, hash-verified corpus archive required by
the OpenAP and JSBSim surrogate readers plus its attribution notice. It does
not duplicate the archive's exploded F-16, HL-20, NESC, or unrelated JSBSim
assets; A320 member-level provenance remains in its family manifests.

The wheel check installs only core and the A320 wheel, then proves that
discovery, planning, batch execution, and the reduced episode do not recover
their data or runtime from `taoryx-reference-models`.
