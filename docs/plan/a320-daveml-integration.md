# A320 DAVE-ML integration record

The source-exact A320 lane remains unavailable: no authoritative Airbus
DAVE-ML package is claimed. The executable family library has separate
compatible lanes backed by the pinned aerospace corpus.

`a320-openap-3dof` is `derived_exact`: it reproduces the pinned OpenAP 2.6.0
public model and is qualified for performance, trim, tuning, and objectives.
`a320-openap-jsbsim-pseudo6dof` is `surrogate_composite`: OpenAP owns
translational performance while JSBSim supplies explicitly approximate
rotational and control structure. Its authority map disables duplicate
JSBSim drag, thrust, and fuel contributions.

## Required intake package

```text
exact DAVE-ML source bytes
source repository/revision or archive hash
license and notices
embedded or companion check cases
geometry and reference dimensions
mass, CG, and inertia record
propulsion and fuel-flow model
control-surface and actuator definitions
declared frames, units, signs, bounds, and extrapolation policy
```

The source-exact acquisition record remains at
`families/reference_a320/qualification/integration-record.yaml` and remains
`blocked_missing_source` until those inputs exist. The compatible products
are recorded separately under `families/a320_openap_3dof/` and
`families/a320_openap_jsbsim_pseudo6dof/`; neither is silently promoted to
manufacturer DAVE-ML source evidence.

## Planned fidelity path

```text
source package and check cases
    ↓
loss-preserving DAVE-ML graph
    ↓
canonical SI / FRD / NED adapter
    ↓
fixed or configuration-dependent mass properties
    ↓
propulsion and control overlays
    ↓
rigid_body_6dof source regression
    ↓
named pseudo-6DOF reduction
    ↓
force-complete 3DOF reduction
```

## Compatible collection work order

The surrogate lane is executed independently of source-exact acquisition:

1. Build the OpenAP `derived_exact` collection and preserve its verified
   performance, thrust, fuel, scalar-mass, and cruise-trim evidence.
2. Normalize the pinned JSBSim A320 aerodynamic and control tables into a
   rotational component without importing its drag, thrust, or fuel outputs.
3. Bind estimated CG and inertia, actuator policy, and stability augmentation
   as Taoryx-owned layers with explicit provenance and nonclaims.
4. Replay cruise, climb, and approach trim; coordinated-turn; and control-pulse
   cases with bounded residual and authority reports.
5. Export the Taoryx-authored DAVE-ML collection as separate performance,
   rotational, propulsion, mass, control, authority, assumptions, and
   provenance documents.
6. Fresh-process re-import the regenerated documents and compare structure,
   seeded values, check cases, and runtime behavior.

The generated collection is canonical regenerated output. It is not an
upstream Airbus, OpenAP, or JSBSim DAVE-ML package, and it must never be
promoted to `reference_exact`.

## Parallel transport reference

The generic NASA Transport Class Model is a separate acquisition target for a
public transport-class nonlinear 6-DOF reference. It is not substituted for
the A320-specific surrogate and is not required to unblock this implementation
lane. Once acquired, it receives its own source hash, collection manifest,
round-trip evidence, and family-library integration record.

The first A320 mission should be airborne trim-to-arrival, matching the
library’s claim-bounded approach for the B747. Runway takeoff, ground effect,
gear, braking, steering, rotation, flare, touchdown, and rollout require their
own data and must not be inferred from an airborne source plant.

## First contributor action

Do not create a guessed A320 DAVE-ML file. Add the source archive to the
controlled input store, record exact bytes and notices, run the common source
firewall, and append every parser or convention failure to the integration
notebook before adding overlays.
