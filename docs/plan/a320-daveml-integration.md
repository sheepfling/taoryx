# A320 DAVE-ML integration record

The A320 is added as a planned source-grounded reference family, but its
integration cannot begin at the parser or runtime layer until an exact DAVE-ML
source package is supplied and hash-pinned. The repository currently has no
A320 `.dml`/`.txair` payload, source revision, mass-property binding, or
check-case record.

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

The source record is maintained at
`families/reference_a320/qualification/integration-record.yaml` and remains
`blocked_missing_source` until these inputs exist. Generic A320 data from a
flight simulator, OpenAP, or a textbook is not silently promoted to DAVE-ML
source evidence.

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

The first A320 mission should be airborne trim-to-arrival, matching the
library’s claim-bounded approach for the B747. Runway takeoff, ground effect,
gear, braking, steering, rotation, flare, touchdown, and rollout require their
own data and must not be inferred from an airborne source plant.

## First contributor action

Do not create a guessed A320 DAVE-ML file. Add the source archive to the
controlled input store, record exact bytes and notices, run the common source
firewall, and append every parser or convention failure to the integration
notebook before adding overlays.
