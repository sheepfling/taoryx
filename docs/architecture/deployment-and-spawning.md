# Deployment and Spawning

Deployment is a generic runtime capability. It is not synonymous with
discarding a stage, creating an aero-ballistic body, or running a reachability
study.

Alpha 3 intentionally promotes only the passive aero-ballistic deployment
path. The generic runtime seam is retained so Alpha 4 can design an authoring
API for active children without replacing the event transaction model.

## Generic Contract

An accepted-boundary deployment produces a `SpawnRequest` containing:

- an event identity;
- an optional parent model identity;
- a complete child `RuntimeVehicle`; and
- optional source and metadata for provenance and analysis.

The child can be any runtime vehicle supported by the model layer: a
propulsive payload, guided interceptor, drone, sensor vehicle, spacecraft,
glider, or passive ballistic object. Its own state, derivative, controls,
events, dependencies, and integrator settings remain explicit on the child.
Deployment validation is performed before the batch is committed, so a failed
request does not partially add children to the active model collection.

## Optional Specializations

`DetachedBodyDefinition` is an aero-ballistic payload description. It adds
shape, dimensions, inertia, projected-area behavior, tumbling, and passive
force/moment assumptions to a child; it is not the generic deployment type.
Likewise, `StageSeparationEvent` is a staging/ejection specialization with
mass, propellant, cutoff, impulse, and separation-mechanism semantics.

These specializations should lower into the same accepted-boundary runtime
transaction as any other deployment provider. They must not force an active
payload or an unrelated spawned vehicle through a ballistic schema.

## Reachability Composition

Reachability owns search-space definition, propagation, terminal criteria,
candidate classification, timeout refinement, and envelope artifacts. It may
run without deployment for an aircraft, drone, glider, or spacecraft. When a
study needs parent/child outcomes, it can consume deployment events and
include the resulting histories, but deployment remains an independent
runtime capability.
