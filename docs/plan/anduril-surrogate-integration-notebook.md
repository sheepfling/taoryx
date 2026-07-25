# Anduril surrogate integration notebook

This is the contributor-facing record for bringing the public-data aircraft
proxies into Taoryx. It deliberately records friction and rejected assumptions,
not just successful commands. The first deliverable was a cataloged parameter
pack. The shared acceleration and attitude-response adapter now provides a
smoke-supported execution seam; family-specific force, propulsion, and
actuator overlays remain later gates.

## Current status

| Layer | Status | Evidence |
| --- | --- | --- |
| Public-source roster | cataloged | `docs/plan/fleet-space-public-surrogate-expansion.md` |
| Corrected parameter pack | data-contract expanded | `verification/anduril_surrogate_parameters_v1.yaml` |
| Shared 3DOF equations | planned | four equation families plus resource backends |
| Named pseudo-6DOF overlays | planned | common state, lag, limits, allocation, and transitions |
| Energy/resource scheduler | planned | battery, fuel, selectable, and series-hybrid contracts |
| Executable vehicle adapters | shared kinematic adapter smoke-supported | family-specific overlays remain gated |
| Mission and reachability fixtures | planned | after parameter, resource, and mode gates |

## The repeatable path

```text
source register
    ↓
evidence and uncertainty classification
    ↓
coherent parameter pack
    ↓
3DOF force and resource checks
    ↓
pseudo-6DOF response overlay
    ↓
loading-state and transition checks
    ↓
mission fixtures
    ↓
reachability and corpus generation
```

For each configuration, keep the immutable data pack separate from executable
runtime overlays. A controller, actuator lag, or transition blend is a Taoryx
engineering layer and does not inherit the public-source evidence grade.

## The v0.2 modeling rule

The compact vehicle fields are convenient compatibility views. The promoted
review record is the parameter ledger, where every numeric input has:

```yaml
path: propulsion.installed_peak_power_kw
value: [2.5, 3.5]
unit: kW
evidence_grade: E
source_ref: comparable_small_rotorcraft_class
uncertainty: full_declared_band
validity: [hover, launch, low_speed_transit]
```

Do not put a guessed value in a field that looks published. Use `P` for a
direct public anchor, `D` for a standard-physics derivation, `E` for a
constrained engineering estimate, and `S` for a scenario placeholder that
must be swept. Unknown is a valid result; it is preferable to a false exact
number.

The resolved vehicle also selects an energy backend:

```yaml
energy_backend: selectable
backend_options: [battery_electric, fuel_burning]
resource_policy:
  reserve_fraction_at_commit: 0.20
  low_soc_derate: true
  fuel_changes_mass_properties: true
```

Battery cases retain mass and inertia while energy falls. Fuel cases update
mass, CG, and inertia continuously. Series-hybrid cases expose generator,
battery, bus-power, fuel-flow, and landing-reserve channels. Never implement
these as a post-processing endurance timer.

## Shared state and mode record

Use the common state layout before adding a vehicle-specific adapter:

```text
position_N, velocity_B, quaternion_BN, body_rates_B,
fuel_remaining, battery_energy, actuator_states,
hover_to_wingborne_blend, nacelle_angle
```

Not every fidelity realizes every field. The fidelity profile must name the
omissions and the reduction law. Modes are data records with entry, blend,
exit, abort, and resource policies. Roadrunner, Omen, and Thunder require
continuous force/moment and attitude blending through transition; a mode
switch that teleports the thrust axis is invalid.

## Configuration selection recipe

Use the following order for an agent or contributor:

1. Select the generic family (`electric_multirotor`,
   `deployable_fixed_wing`, `fuel_turbojet_fixed_wing`, `fuel_jet_vtol`,
   `selectable_energy_tailsitter`, or `series_hybrid_tiltrotor`).
2. Select the named configuration and loading state (`light`, `nominal`,
   `heavy`, or `dash`).
3. Select the energy backend. For Omen and ALTIUS, run every declared backend
   as a separate configuration; never collapse them into one nominal.
4. Select fidelity: `point_mass_3dof` or a named `pseudo_6dof` profile.
5. Select a role-specific mission state machine and reserve policy.
6. Resolve the parameter ledger and reject missing units, grades, bands, or
   source references before runtime execution.
7. Run the data firewall, resource balance, trim/equilibrium, transition,
   actuator, and paired-fidelity gates in order.

The output should state the exact vehicle, loading state, backend, fidelity,
mission, environment, and evidence boundary. “Anduril drone” is not a
reproducible model identity.

## First integration slice

The recommended first executable slice is `bolt` because it has the clearest
public mass/endurance anchor and a compact multirotor force model. It should be
implemented in this order:

1. Register the `bolt` manifest without a runtime claim.
2. Derive hover power, battery reserve, maximum acceleration, and drag checks.
3. Add a point-mass 3DOF session with acceleration-vector control.
4. Add `p6dof.multirotor_rate_response.v1` with quaternion, rate, tilt, and lag.
5. Compare 3DOF and pseudo-6DOF on hover, acceleration, braking, and return.
6. Add light/nominal loading states only after the baseline is reproducible.

Ghost-X should follow Bolt, but its single-main-rotor architecture must be
implemented as a helicopter/rotorcraft family. It must not inherit a quadrotor
allocator just because both are hover-capable.

ALTIUS-600 is the first energy-backend comparison target because its public
mass, speed, range, and endurance anchors support both a light-payload battery
case and a small-engine fuel case. Roadrunner follows for jet-VTOL transition
and recovery reserve. Barracuda-250 follows for altitude/Mach thrust lapse,
spool, high-speed drag, and fuel-range coupling. Omen and Thunder follow only
after the generic backend and mode contracts are stable.

## Failure and friction ledger

These findings came from the intake review and are preserved as onboarding
lessons. They are not claims that the executable adapter has already failed.

| Finding | Why it matters | Required guard |
| --- | --- | --- |
| Ghost-X was initially treated as a multirotor | Architecture changes rotor force, drag, control, and failure semantics | Separate helicopter/rotorcraft family from quadrotor family |
| Public maxima were combined into one configuration | Payload, endurance, range, and speed maxima are not simultaneous | Use light, nominal, heavy, and dash loading states |
| Anvil was assigned an unsupported 15 kg nominal | Sparse public data cannot justify precise mass | Use 5.5 kg with 4.5–7 kg E/S band and sensitivity |
| ALTIUS-600 used excessive induced drag | High aspect ratio materially changes energy/range behavior | Derive `k` from aspect ratio and expose the uncertainty band |
| Barracuda operating minimum was called stall speed | Operational speed and aerodynamic stall are different constraints | Derive stall from `W/(q S CLmax)` and keep operating minimum separate |
| Fury used a generic low induced-drag factor | Compact fighter-like geometry needs higher induced drag or explicit body lift | Use `k=0.16–0.23` unless body lift is modeled |
| Omen hover power was too high | The proxy became energetically inconsistent with its size | Recompute induced power and distinguish practical from installed power |
| Thunder was given precise performance values | Concept-level public data does not support exact numbers | Keep `thunder_concept_proxy_v0` S-grade and catalog-only |
| Maximum speed was treated as a hard clamp | Clamping hides inadequate propulsion or unrealistic drag | Require a thrust/drag intersection at declared altitude |
| Hybrid transition was a mode switch | Discontinuities corrupt trajectories and reachability boundaries | Use continuous force, thrust-axis, and attitude blending |
| First parameter-pack parse failed on an unquoted colon in a YAML rule | YAML treats the colon as a mapping separator in plain scalar text | Quote rule strings containing `:` and run the pack parser before registration |
| Thunder hover power was declared in MW while the generic check expected kW | Mixed units made valid concept data look absent | Normalize declared units at the adapter boundary and retain the source unit in the manifest |
| Ghost-X loading-state data had no top-level nominal mass or power | The validator could not select a default state | Expose a nominal state and teach the validator to inspect state-specific values |
| Roadrunner had static thrust but no public power model | A power gate would have invented a value or silently passed | Allow a smoke check from static thrust, but keep the power qualification gate open |
| Fury had area and `CLmax` but no explicit derived stall field | The fixed-wing check could not compare a declared value | Add the derived 47.7 m/s field and retain its D/E provenance |

## What a junior should record during implementation

For every failed check, add:

```text
date
configuration and fidelity
command or test fixture
observed diagnostic
first incorrect layer
root cause
fix or intentional deferral
new regression test
claim/status change
```

Do not “fix” a failing run by widening the envelope, clamping the state, or
changing a source value without recording the reason. If the failure is caused
by missing public evidence, keep the field as a range or mark it unknown.

## Promotion gates

The parameter pack may be promoted from intake to executable only when the
configuration has:

- a validated manifest and evidence ledger;
- finite, unit-bearing mass, geometry, resource, and performance fields;
- passing hover or cruise equilibrium checks;
- a reproducible 3DOF transition and resource test;
- a named pseudo-6DOF response law, if advertised;
- no hidden hard speed clamp or discontinuous transition;
- a documented limitation and reproduction command.

The shared adapter smoke run is not yet `M1 — Executable` for a vehicle
realization; it proves only the common transition seam. A vehicle realization
reaches `M1 — Executable` after its family-specific force/resource checks pass.
It is not `M5 — Qualified / Pickup-Ready` until the vehicle readiness gates,
loading states, controller behavior, evaluation artifacts, and failure ledger
are complete.

## Promotion checklist

Before changing a configuration status, record:

- parameter-pack hash and source register hash;
- selected loading state and energy backend;
- 3-DOF and pseudo-6-DOF parent identity;
- initial mode and all mode transition events;
- resource balance, reserve, and mass-property telemetry;
- actuator requested, limited, and achieved values;
- trim, closure, convergence, and table/envelope margins;
- E-grade and S-grade sweep results;
- unsupported claims and the next blocked rung.

If a check fails because the public evidence is insufficient, mark the rung
`blocked` or `catalog-only`. Do not widen the model silently and do not change
the evidence grade to make the promotion appear complete.
