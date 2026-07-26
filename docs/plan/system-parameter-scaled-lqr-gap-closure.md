# System-parameter-scaled LQR gap closure

Status: active implementation plan
Scope: controller realization, plant-informed scaling, and four existing
controller-mission families
Claim boundary: controller infrastructure and research-surrogate evidence;
not flight-control certification or historical TAOS executable compatibility

## Decision

Taoryx will not replace every PID or rewrite the controller stack. The current
controller missions remain frozen baselines while the active control path is
made auditable and the existing system-parameter-scaled LQR core is promoted
through explicit qualification gates.

The implementation order is:

```text
freeze baselines
  -> audit the active path
  -> declare controller realizations
  -> verify scaled-LQR synthesis
  -> wire telemetry and preflight
  -> migrate Hummingbird, X8, B747, X-15
  -> compare against retained PID/baseline cases
  -> publish controller and mission claims separately
```

The first tranche now exists in code as
`taoryx.controller_realization.ControllerRealization` and
`preflight_controller_realization`. A resolved Alpha 2 case may carry this
record in `ResolvedCase.controller_realization`. The existing LQR solver also
records hashes for `A`, `B`, `Q`, `R`, and the physical `K` matrix.

## Why this is needed

The repository already has useful pieces:

* `runtime.lqr` solves named continuous and scaled LQR problems and screens
  closed-loop poles.
* `controller_autotune` derives mass/inertia-aware scale contracts and sweeps
  gentle, standard, and aggressive profiles.
* `controller_designs.yaml` binds a trim, state order, control order, and
  allocator for B747, X8, Hummingbird, and X-15.
* Native runtime lowering can build an attitude LQR and schedule it with mass
  and inertia.

The gap is that a gain can be numerically stable without proving that it was
connected to the intended plant, frames, controls, allocator, or actuator.
The gain also needs to be reproducible from a resolved operating point and
scaling contract. A stable pole set is controller evidence; it is not by
itself a mission or vehicle-family qualification.

## Controller realization contract

Every active controller must resolve to a versioned record containing:

* role and implementation/version;
* fidelity and design identifier;
* ordered state and input channels, units, frames, scales, and bounds;
* reference bounds;
* plant and linearization provenance;
* operating point and schedule coordinates/domains/interpolation;
* state/control scale identifiers and `Q`/`R` identifiers;
* SHA-256 hashes of `A`, `B`, `Q`, `R`, and `K` for LQR/LQI;
* integral states, closed-loop poles, and maximum real pole;
* allocator, actuator bindings, and the declared path;
* declared fallback, scenario-override policy, claim status, and provenance.

The contract is immutable. It belongs in `ResolvedCase`, and the runtime
command telemetry must reference its digest. The state and input tuples are
ordered contracts; changing order without changing the realization hash is a
hard error.

## Fail-closed preflight

Activation must reject, with stable diagnostic codes:

* missing role, implementation version, plant source, or LQR linearization;
* state/input order or channel mismatch;
* missing units or frames;
* a controllability rank below state dimension;
* schedule coordinates outside their declared domains;
* disconnected regulator, allocator, actuator, or plant outputs;
* an active fallback not declared in the contract;
* scenario gain or matrix overrides unless explicitly allowed.

Diagnostic Riccati regularization may be used in a development tool only when
it is recorded as such. It is never an implicit production fallback.

## Active-path audit

For each mission, record the actual chain, not the intended architecture:

```text
case declaration
  -> resolver
  -> controller factory
  -> guidance/reference
  -> regulator
  -> allocator
  -> actuator limits/dynamics
  -> plant
```

At every boundary, log requested, unsaturated, limited, achieved, and fallback
values where the channel exists. The required command/observation contract is
the source of truth for what a controller is allowed to write and what the
plant reports back.

## Scaled-LQR verification contract

For each promoted realization:

1. Resolve the physical vehicle, mass properties, operating point, and trim.
2. Generate or import a named `A/B` linearization from that same point.
3. Select a reduced plant only when its state/control projection is declared.
4. Normalize states and controls with physical scales or limits.
5. Solve CARE/DARE with dimensionless `Q/R` and map `K` back to physical units.
6. Check controllability/stabilizability, finite matrices, and closed-loop poles.
7. Record matrix hashes, scales, profile IDs, and pole diagnostics.
8. Schedule or interpolate only inside declared operating-point domains.
9. Re-run the affected trim and qualification probes when a coupled parameter
   changes.

The existing default attitude bridge remains a conditioning baseline. It must
be labeled `runtime-inertia-attitude-bridge` and cannot support a source-model
controller claim without a vehicle-specific trim Jacobian.

## LQR-first retrofit policy

This is a controlled retrofit of the existing control seam, not a dynamics or
mission rewrite. Existing PID, rule-based, and mixed paths remain available
under explicit baseline identities for regression comparison. They are not
eligible for `controller_qualified`.

The checked-in
[`verification/controller_inventory.yaml`](../../verification/controller_inventory.yaml)
is the first runtime-path inventory. It records the current four-family
problem-file paths, their runtime-status gains, corresponding LQR design
candidates, owners, and qualification eligibility. Audit it with
`tools/audit_controller_inventory.py` before changing any gains.

A reproducible baseline may still contain scenario gain overrides; that makes
it regression evidence, not qualified-controller evidence. A qualified path
must have `scenario_gain_overrides: false`, an immutable design hash, and
runtime provenance for the active trim, schedule, requested generalized
control, allocation, achieved actuator state, saturation, and rate limiting.

The neutral runtime records are `ControllerProvenance`, `GuidanceReference`,
`GeneralizedControlRequest`, `AllocationResult`, and `ControllerRuntimeState`.
They complement—not replace—the existing `ControlDemand`, `ControlCommand`,
allocator, actuator, and plant contracts.

`ControllerBackendRegistry` is the selection boundary. It resolves an
implementation by declared backend ID and makes the claim boundary visible:
the fixed-point LQR factory is executable, while gain-scheduled LQR, RSLQR,
and LQI remain explicit promotion points until their factories are supplied.
The registry also exposes `legacy_pid_baseline` as regression-only. A missing
factory or an attempt to qualify a legacy backend fails closed.

## Migration order

### M0 — Freeze and audit

Exit when each active loop is classified as `KEEP`, `LQI`, `MIGRATE`,
`BASELINE`, `REMOVE`, or `REFORMULATE`; the current showcase runs and controller
configs are hashed; `verification/controller_inventory.yaml` covers every
active family path; and hidden gain overrides are either rejected or marked
baseline.

### M1 — Contract and solver evidence

Exit when all four design catalog entries are role-declared, matrix hashes are
present for generated LQR realizations, runtime provenance channels are
schema-validated, an invalid ordering/frame/schedule/fallback or scenario gain
override is rejected before activation, and a synthetic double-integrator
contract passes end-to-end preflight.

### M2 — Hummingbird

Migrate the rate loop first, then position/altitude guidance. Preserve the
existing controller mission as a baseline. The promoted case must show
requested moment, allocated rotor command, achieved rotor/moment state, rate
limits, saturation, and the independent truth-objective report.

Exit: `controller_wiring_verified` and `controller_local_stability_verified`
for the declared hover operating point; nominal mission status remains a
separate gate.

Implementation note: the checked-in
`SV05_rate_lqr_hover_6dof.prb` is the M2 Hummingbird witness. It declares a
three-state body-rate LQR, resolves through the common realization contract,
maps canonical body moments into the source +Z rotor frame, and allocates
individual rotor speeds. `SV05_rate_damped_hover_6dof.prb` remains the
legacy rate-damping regression baseline. The focused witness asserts pole
stability, rate settling, matrix/provenance metadata, rotor bounds, and
nonzero allocated moment response. This is controller evidence only; it does
not qualify the Hummingbird mission or family envelope.

The X8 figure-eight altitude-reversal case is also registered as a wiring
witness for the same contract. Its attitude LQR realization exposes ordered
channels, matrix hashes, an explicit allocator path, and no scenario-gain
override permission. Its route-specific surface and sideslip gains remain
outside the LQR contract, so full X8 controller and route qualification stays
in M3.

### M3 — X8

Migrate longitudinal and lateral local regulators using the published local
trim neighborhood and source-envelope limits. Explicitly exercise right and
left turns, bank reversal, pitch reversal, and altitude gates. Do not promote
the route until the surface/attitude response is physically connected or the
pseudo-6DOF nonclaim is explicit.

Exit: local controller gates pass at the powered trim and all route objectives
are independently truth-evaluated.

### M4 — B747

Use the source-anchor trim and transport-class response schedule. Validate
slow-response pole behavior, opposite-direction turns, energy management, and
the multidimensional arrival gate. Keep runway claims out of scope until
high-lift, gear, ground, and brake models are separately qualified.

The B747 is the first production-scale LQR target. Complete open-loop
authority probes before tuning; then reuse one transport design across
heavy/nominal/light mass, cruise/descent operating points, and both turn
directions. Route geometry may be made physically attainable, but gains may
not be changed per route.

### M5 — X-15

Use the reachability/release work as the launch and aeroballistic baseline.
First qualify the source-relevant local glide or powered segment, then connect
the scaled bank/attitude regulator to the event-driven release, burn, cutoff,
coast, re-entry, and terminal handoff. Preserve event order and resource
continuity; a reachability result does not replace controller evidence.

Keep the canonical X-15 air-launch showcase separate from the
California-to-Hawaii staged boost-glide surrogate. The latter is a useful
reachability and deployment witness, but it must not inherit an authentic
historical X-15 claim. After separation, spent boosters become independently
propagated child objects under the aero-ballistic deployment and object-lineage
contracts.

### M6 — robustness and release packet

Run fixed perturbations and then a declared development set for each migrated
vehicle. Keep controller qualification, mission qualification, and family
envelope qualification as separate statuses.

## Evidence packet pages

Each vehicle packet must contain:

1. Mission Summary — objectives, truth results, terminal state, and exact claim.
2. Controller Realization — contract digest, active path, references,
   controls/effectors, saturation, and achieved signals.
3. Numerical/Robustness — convergence, batch/step parity, fixed perturbations,
   poles, and failure classifications.
4. Controller Comparison — migrated LQR against the retained PID/baseline,
   with the same plant, mission, and acceptance file.
5. Cross Fidelity — semantic mission mapping and measured disagreement.
6. Evidence/Claims — source, identified, derived, estimated, and unsupported
   claims by subsystem.

Every displayed value must be recomputable from the packet telemetry and
metric dictionary. A scalar mission score may rank quality, but it cannot
override a failed required objective, unstable pole, envelope violation, or
invalid controller wiring.

## Status vocabulary

Use these independently:

* `controller_wiring_verified`
* `controller_local_stability_verified`
* `nominal_mission_pass`
* `controller_schedule_qualified`
* `fixed_matrix_robustness_pass`
* `cross_fidelity_compared`
* `family_envelope_qualified`
* `pickup_ready`

The current four-family showcase baselines are integration evidence until the
controller realization, independent truth objectives, and robustness pages
support a stronger status.

## Definition of done

The scaled-LQR gap is closed when every active loop is role-declared; runtime
telemetry identifies the realization that produced each command; gains are
derived from the resolved plant/operating point, physical scaling, and
versioned `Q/R`; reduced plants pass controllability and pole checks; PID loops
are explicitly subordinate, retained as baselines, or migrated; hidden
fallbacks and scenario gain mutation are impossible; and B747, X8,
Hummingbird, and X-15 each have separate controller, mission, robustness, and
claim packets. A configured consumer can select an approved controller profile
and run it without editing implementation code.

This definition does not claim that all four vehicles are source-validated,
globally robust, or historically TAOS-compatible. Those remain explicit
evidence gates.
