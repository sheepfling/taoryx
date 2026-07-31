# Alpha 3 direct-wrench family matrix

Status: active execution ledger

This matrix separates a usable direct-wrench evidence tier from a physical
effector claim. A direct-wrench result may use a full Newton–Euler plant and
source aerodynamic/propulsive loads, but the controller contribution is an
explicit bounded body force/moment injection. It must therefore report
`physical_effector_allocation: false`.

## Current evidence

| Family | Current direct-wrench evidence | Honest status | Next promotion gate |
|---|---|---|---|
| X8 | Local source-trim nonlinear six-axis recovery | T3 nominal local pass | Add the same contract to the full racetrack and retain the physical elevon path separately |
| B747 | Source-table Newton–Euler transport racetrack | T3 nominal mission pass | Close source trim and schedule nodes; do not inherit this as surface allocation |
| A320 | OpenAP/JSBSim source-calibrated local six-axis recovery | T3 nominal local pass | Add a true rigid-body mission wrapper and then physical surface/throttle realization |
| F-16 | Source-plant local six-axis recovery; racetrack retained as a failed-route witness | T3 nominal local pass, route pending | Reconcile the direct route’s turn/terminal geometry before promotion |
| X-15 | Local release/glide source-load direct-wrench recovery bridge; source trim remains blocked | T3 direct-wrench bridge; physical path pending | Source-backed trim, phase-aware wrench authority, and a terminal handoff gate |
| Hummingbird | Native individual-rotor allocation is the primary 6DOF path; a bounded direct-wrench bridge comparator also exists | T3 direct-wrench bridge plus T5 native rotor path | Keep native rotor allocation as the physical path; use the bridge for controller/plant comparison |
| HL-20 | Source-surface replay, bounded source allocation, and a local source-load direct-wrench bridge | T3 direct-wrench bridge; physical surface path preferred | Source-bound trim, bank/alpha/energy control, then compare direct and surface paths |
| NESC | Translation-derived local direct-wrench bridge with declared engineering inertia; source gimbal/effectivity is absent | T3 engineering direct-wrench bridge; physical path blocked | Obtain authoritative gimbal/effectivity and event-aligned attitude data |
| Tumbling body | No controller; native passive rigid body is the truth path | Direct-wrench tier not applicable | Preserve native uncontrolled 6DOF and averaged-area 3DOF reduction |

## Promotion contract

Each family that advertises direct-wrench 6DOF must provide:

1. A declared state ordering and SI force/moment ordering.
2. A valid trim, release, or passive initial condition.
3. Source loads separated from injected control loads.
4. Bounded requested, achieved, and residual wrench telemetry.
5. Nonlinear replay through the same plant used to derive the local model.
6. Data-domain, mass/resource, quaternion/rate, and numerical checks.
7. An independent mission evaluator when a mission is claimed.
8. An exact nonclaim for physical effectors unless an allocator is actually in
   the loop. The direct-wrench bridge remains a valid integrated rigid-body
   tier even when physical allocation is unavailable.

A local T3 pass does not promote a family mission. A mission pass does not
promote physical elevator, elevon, rotor, stabilator, gimbal, wheel, or
thruster allocation. Conversely, an uncontrolled passive body is not a failed
controller case; it is a different contract and remains native rigid-body
evidence.

## Remaining execution sequence

The direct-wrench work is complete only when each controlled family has the
following vertical slice at a declared operating point:

```text
valid trim/release contract
    -> source-load decomposition
    -> plant-derived A/B matrices
    -> scaled LQR or declared controller
    -> bounded wrench projection
    -> nonlinear rigid-body replay
    -> requested/achieved/residual telemetry
    -> independent perturbation and timestep checks
```

The current artifacts close the common bounded-wrench bridge slice for all
eight controlled families: X8, B747, A320, F-16, X-15, HL-20, Hummingbird,
and NESC. Physical-effector and source-authority promotion remains
intentionally
family-specific:

1. X8: replace the direct-wrench command with the source-mapped elevon path
   for the reusable fixed-wing template.
2. B747 and A320: reuse that allocator with transport-scale trim nodes and
   explicit throttle/elevator/surface limits.
3. F-16: add scheduled effectiveness and alpha/beta/load-factor authority
   witnesses before claiming high-rate physical control.
4. Hummingbird: retain the native motor/rotor allocator as the physical path;
   the direct-wrench bridge remains a comparison realization.
5. HL-20: close source-bound trim and nonlinear surface-routed bank/alpha/
   energy response.
6. X-15: close powered/coast/glide source trim and phase-dependent effector
   authority before any terminal-handoff claim.
7. NESC: obtain source gimbal/effectivity and attitude history; until then the
   translation-derived direct-wrench bridge is the honest ceiling.
8. Tumbling body: do not invent a controller. Its native uncontrolled rigid
   body is the 6DOF truth path, with an averaged-area 3DOF reduction.

## Reproduction anchors

```text
PYTHONPATH=src python3 tools/validate_x8_direct_wrench.py
PYTHONPATH=src python3 tools/build_family_qualification_packet.py \
  --output artifacts/showcases/airbreathing-racetrack-fidelity-ladder \
  --mission b747-racetrack-altitude-turns-6dof-v1
PYTHONPATH=src python3 tools/validate_a320_direct_wrench.py
PYTHONPATH=src python3 tools/validate_f16_direct_wrench_local.py
PYTHONPATH=src MPLCONFIGDIR=/tmp/taoryx-mpl python3 tools/validate_hl20_direct_wrench_local.py
PYTHONPATH=src MPLCONFIGDIR=/tmp/taoryx-mpl python3 tools/validate_x15_direct_wrench_local.py
PYTHONPATH=src MPLCONFIGDIR=/tmp/taoryx-mpl python3 tools/validate_nesc_direct_wrench_local.py
```

The machine-readable readiness index records these local witnesses under
`direct_wrench`, independently of the reduced R1 matrix and physical-effector
fields.

The normalized four-dimension contract ledger is generated at
`verification/alpha3_direct_wrench_contract/manifest.json`. Every family
record explicitly declares trim or release status, source and control
force/moment composition, resource model, envelope, promotion status, and
nonclaims. The tumbling body is included as a native uncontrolled exception;
it is not forced through a controller or direct-wrench schema.

Reproduce it with:

```text
PYTHONPATH=src python3 tools/build_alpha3_direct_wrench_contract.py
```

The regime-specific readiness record is generated at
`verification/alpha3_regime_direct_wrench_readiness/manifest.json`. It is
fail-closed: X-15 remains blocked at source-bounded T1 trim, NESC remains
blocked at the missing source attitude/gimbal contract, HL-20 retains the
source-surface path as the preferred realization, and Hummingbird retains
native individual-rotor allocation as the preferred realization. These
preferences do not invalidate the direct-wrench bridge; they identify the
next physical-effector promotion step. Direct-wrench evidence is never
counted as physical-effector evidence.

Reproduce it with:

```text
PYTHONPATH=src python3 tools/build_alpha3_regime_direct_wrench_readiness.py
```
