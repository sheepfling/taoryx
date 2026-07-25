# Alpha 2 vehicle-family authoring guide

Alpha 2 adds a vehicle family through a versioned `FamilyPackage`, not through
a new mission runner or a bespoke problem-file interpreter. The package is a
provider-neutral contract. A runtime provider may implement only the declared
fidelities and capabilities; unsupported requests must fail during resolution.

## Required package shape

Start with these fields:

```python
FamilyPackage(
    family_id="example.family",
    version="1.0.0",
    display_name="Example family",
    fidelities=("point_mass_3dof", "pseudo_6dof", "rigid_body_6dof"),
    parameters=(...),
    controls=(...),
    observations=(...),
    capabilities=CapabilitySchema(...),
    component_slots=(...),
    resources=(...),
    allocations=(...),
    mode_transitions=(...),
    provenance="source bundle and model path",
)
```

Keep the model asset immutable and content-hashed. Use `ParameterSchema` for
independent, derived, and developer-only values; use `VariantSpace` for
bounded semantic modifiers. Do not expose raw table-cell mutation, arbitrary
inertia entries, or topology changes as ordinary case overrides.

## Controls and observations

Every control declares its canonical unit, frame, semantic level, authority
modes, absolute/rate command modes, limits, and achieved observation channel.
Use `ControlInputMode("absolute")` for a requested position or setpoint and
`"rate"` for a requested actuator rate. The authority arbiter records the
requested, allocated, limited, and achieved values.

Every observation declares its unit, frame, semantic level, availability,
kind, evidence grade, and uncertainty. A channel that is not available for a
fidelity is declared `availability="unavailable"`; requesting it produces a
stable resolution diagnostic instead of a fabricated zero.

## Components, resources, and transitions

Use `ComponentSlot` for engines, rotors, surfaces, actuators, payloads, tanks,
batteries, and effectors. Use `ResourceSchema` for fuel, propellant, battery
energy, power, reaction-wheel momentum, or thermal budget. Resource depletion
must remove capability through the runtime resource ledger; it must not create
energy or permit last-step borrowing.

Use `AllocationSchema` when one semantic input maps to multiple effectors. Use
`ModeTransitionSchema` for conversion, staging, gear, rotor/wing, or resource
mode changes. Each transition should name entry and exit guards, abort target,
hysteresis, schedules, controller handoff, and state/resource continuity.

## Resolution and evidence workflow

1. Add the family package and source/provenance record.
2. Declare every supported fidelity and capability.
3. Add convention-firewall and source-table fixtures.
4. Resolve a baseline `CaseIntent` for each advertised fidelity.
5. Add positive, projected, and rejected variant cases.
6. Add control-direction, achieved-control, resource, and transition tests.
7. Run the interface stress matrix and record the compatibility report.
8. Only then add provider-specific trims, controllers, and missions.

The result is a resolved case with canonical units, fixed schemas, immutable
provenance, a stable identity hash, qualification metadata, and explicit claim
boundaries. A contract probe does not promote an unimplemented vehicle to
flight or source-validation maturity.
