# TAORYX Alpha 2 progress log

**Release:** `v0.2.0-alpha`  
**Plan:** [TAORYX Alpha 2](taoryx-alpha-2.md)  
**Current exit:** `A2-RELEASE-PASS`
**Overall release:** core contract complete; remaining Alpha 2 closeout is the
platform and four-family proof tranche. Source-grounded reference anchors are
deferred to Alpha 3.

This is the running implementation log for Alpha 2. A tranche is complete only
when its checked-in tests pass, its evidence directory is generated from the
canonical catalog and fixtures, and its audit verifies the machine-readable
completion signal and file hashes.

## Completed tranches

### A2-T1 — `case-contracts`

Status: **PASS** (`A2-T1-PASS`)

Delivered:

- neutral Pydantic contracts for families, cases, parameters, controls, and observations;
- canonical unit conversion and fail-closed range/type/fidelity checks;
- preset precedence from family defaults through explicit case overrides;
- immutable `ResolvedCase` values with stable SHA-256 identity;
- per-value provenance with original and canonical values;
- validation for requested control and observation channels;
- CLI catalog/family/case/schema inspection;
- deterministic case fixtures and negative diagnostics.

Evidence: `artifacts/verification/alpha2/t1_case_contracts/`

Claim boundary: case resolution and provenance only. This tranche does not
claim trajectory physics, controller behavior, or TAOS 96.0 compatibility.

### A2-T2 — `provider-session`

Status: **PASS** (`A2-T2-PASS`)

Delivered:

- provider capability discovery through an explicit registry;
- a non-Taoryx analytical point-mass reference provider;
- a Taoryx-native adapter over the public `InteractiveSession` transition;
- common compile, reset, step, run, and normalized result contracts;
- explicit native/approximated/unsupported translation entries;
- deterministic batch-versus-repeated-step parity;
- self-contained capability, translation, result, parity, status, and hash manifests.

Evidence: `artifacts/verification/alpha2/t2_provider_session/`

Claim boundary: provider lifecycle and deterministic parity for the synthetic
`simple_aero` point-mass fixture. This tranche does not claim broad vehicle
migration, controllers, pseudo-6DOF, rigid-body 6DOF, or source fidelity.

### A2-T3 — `control-authority`

Status: **PASS** (`A2-T3-PASS`)

Delivered:

- neutral control-frame inputs for autopilot, commanded, overlay, direct, and mixed authority;
- per-channel supported modes and default authority metadata;
- cadence windows with deterministic hold behavior;
- default and explicit failsafe behavior for invalid frames;
- global bounds, overlay bounds, and per-second rate limits;
- stable error codes for unknown channels and unsupported/ambiguous authority;
- per-step applied-control and per-channel arbitration telemetry;
- parity replay through both the analytical reference provider and Taoryx adapter.

Evidence: `artifacts/verification/alpha2/t3_control_authority/`

Claim boundary: deterministic control arbitration for the synthetic
`simple_aero` point-mass provider bindings. This tranche does not claim vehicle
mission performance, policy transfer, unbounded mixing, direct-control vehicle
missions, or 6-DOF control fidelity.

## Reproduce and audit

```bash
python tools/dev.py alpha2-tranches
python tools/dev.py audit-alpha2
```

The generator clears only the Alpha 2 tranche directories. It does not
remove unrelated artifacts. The audit checks completion signals, manifest byte
counts and SHA-256 values, resolved-case identity replay, provenance coverage,
negative diagnostic coverage, provider set, translation presence, and
batch/step parity, reduction parity, rigid-body evidence, and convergence.

## A2-T4 release notes

### A2-T4 — `simple_aero_3dof`

Status: **PASS** (`A2-T4-PASS`)

Delivered:

- metadata-driven Simple Aero parameter schemas, launch/aim-point metadata, and
  reusable segment graphs;
- baseline and light/high-thrust loadout cases generated from resolved inputs;
- native `.prb` output with ordinary TAOS/TAORYX syntax and distinct physical
  burnout versus commanded-cutoff annotations;
- bounded command overlay metadata and deterministic repeated sweep hashes;
- native runtime execution with segment transitions, events, metrics, CSV
  telemetry, and plot manifests;
- an invalid pitch-envelope case that fails during case resolution.

Evidence: `artifacts/verification/alpha2/t4_simple_aero_3dof/`

Claim boundary: metadata-driven synthetic Simple Aero-style point-mass
composition and generation only. This is not a recovered historical TAOS
runtime, a source-validated vehicle model, or a pseudo-/rigid-body 6-DOF claim.

The T4 telemetry artifact is CSV because the base project intentionally does
not require a binary dataframe engine. A later evidence packaging tranche may
add Parquet as an optional export without changing the canonical case or
runtime contract.

## A2-T5 release notes

### A2-T5 — `fidelity-ladder`

Status: **PASS** (`A2-T5-PASS`)

Delivered:

- a versioned `simple_aero_ladder` family package that advertises all three
  Alpha2 fidelity profiles from one resolved case;
- a canonical ECIC initial-state adapter and common segment/command contract;
- native point-mass and pseudo-6DOF problem generation from the same resolved
  case, with exact constrained translation parity;
- a native rigid-body 6DOF projection with explicit attitude, inertia, force,
  moment, body-rate, envelope, actuator-rate, and equation-closure telemetry;
- separate machine-readable parity/divergence, initial-state, command,
  convergence, rigid-body, status, and hash manifests;
- a canonical-SI fidelity-ladder plot generated from the machine-readable
  trajectory CSVs.

Evidence: `artifacts/verification/alpha2/t5_fidelity_ladder/`

Claim boundary: one synthetic successor-side family demonstrates the Alpha2
fidelity-ladder mechanics. The pseudo tier is a kinematic attitude bridge and
is explicitly not rigid-body evidence. The rigid tier is a free-body research
projection, not historical TAOS compatibility or a global vehicle-validity
claim.

The T5 convergence gate compares each native tier with a deterministic
half-step replay. The rigid-body report separately records nonzero moments and
body rates, positive synthetic alpha/beta margins, and independent normalized
force/moment closure. Rigid-versus-point divergence is expected and reported;
only point-versus-pseudo is a reduction-parity gate.

## A2-T6 release notes

### A2-T6 — `dual-launch-glider`

Status: **PASS** (`A2-T6-PASS`)

Delivered:

- a versioned `dual_launch_glider` family package advertising the Alpha 2
  fidelity ladder and common control/observation schemas;
- two checked-in case intents using the same family, mission, loadout,
  segment plan, controls, and post-release guidance contract;
- ordinary TAORYX problems for air release and attached-booster launch, emitted
  by one reusable resolved-case adapter;
- a native powered segment followed by a `*when` segment transition with no
  hidden `*reset` or `*increment` state change;
- explicit separation policy, pre/post samples, launch-form comparison,
  authority-mode evidence, common-SI trajectory CSVs, and a handoff plot;
- focused parser/runtime tests and an Alpha 2 release audit that checks all
  required files and hashes.

Evidence: `artifacts/verification/alpha2/t6_dual_launch_glider/`

Claim boundary: deterministic successor-side composition and handoff only.
The fixture is not a globally valid glider, a historical flight reconstruction,
or evidence of TAOS 96.0 compatibility. The T6 evidence also does not claim
that the synthetic waypoint objective substitutes for a source-validated
vehicle mission.

## A2-T7 — `alpha-2-release`

Status: **PASS** (`A2-RELEASE-PASS`)

Delivered:

- a catalog-derived public schema snapshot covering family packages, case
  resolution, fidelity profiles, controls, observations, and authority modes;
- a generated `schema-reference.pdf` and machine-readable claim matrix;
- a self-contained, path-sanitized packet containing the T1–T6 source inputs,
  evidence, plots, native problem artifacts, hashes, and release metadata;
- an isolated clean-source replay of the T1–T6 generator with per-file hash
  comparison and no workstation-path dependency;
- an auditable release gate that verifies the packet, schema freeze, exclusions,
  hashes, and replay status.

Evidence: `artifacts/verification/alpha2/t7_release/`

Reproduce with:

```bash
python tools/dev.py alpha2-release
python tools/dev.py audit-alpha2
```

Claim boundary: Alpha 2 proves the checked-in successor-side configuration,
provider lifecycle, control authority, native generation, fidelity composition,
dual-launch composition, and release tooling. It does not claim historical
TAOS 96.0 runtime compatibility, global vehicle validity, flight qualification,
or a universal autopilot.

## A3-T1 deferred — source-grounded reference anchors

The F-16 S-119 and HL-20 Mod K source-anchor tranche is an Alpha 3 breadth
dependency. It is complete only when both pinned collections replay their
source DAVE-ML graphs through a fresh Taoryx process, pass source check cases
and trim/hold regressions, and produce reproducible direct-control plant
artifacts. This tranche does not include actuator/controller overlays,
flagship missions, RL tasks, or derived 3DOF/pseudo-6DOF reductions.

## Deferred after Alpha 2

Remote model marketplaces, untrusted hot-loading, universal autopilot behavior,
historical TAOS runtime compatibility, and global vehicle-validity claims remain
outside the frozen release contract.

The expanded work is ranked in
[`taoryx-alpha-2-backlog.md`](taoryx-alpha-2-backlog.md). The immediate P0 path
is common scenario/evaluation, interactive Lab/session closure, generic vehicle
onboarding, and checkpoint/replay; flagship missions and additional families
remain P1 work until those seams are complete.
