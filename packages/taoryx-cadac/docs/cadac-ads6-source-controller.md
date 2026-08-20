# CADAC ADS6 source-controller package path

The `cadac.ads6.engagement` package defaults to a persistent source-shaped SAM controller rather than a synthetic pointing law. The controller is interleaved with `Ads6SamPlantStepper` at the installed CADAC module boundaries.

```text
source event cursor
  → truth-aligned INS boundary
  → deterministic RF or IR seeker state
  → RADAR0 intercept-point line guidance
  → terminal RF/IR proportional navigation
  → adaptive rate/acceleration autopilot
  → physical fin, TVC, or aggregate-RCS plant
```

## Ordering

Events are evaluated before the SAM module pass. Controller modules execute at `ins`, `sensor`, `guidance`, and `control`. A command emitted by `control` is consumed by later actuator/TVC/RCS/force modules in the same SAM pass. Target and radar packets retain source actor-publication epochs, so the earlier SAM observes the previous packet while the later radar observes current-epoch target truth.

## Participating behavior

- source `maut`, `mguide`, and `mseek` modes;
- sequential event mutations and event-relative time;
- deterministic RF gimbal and IR filter states;
- acquisition, lock, blind-range, and supported break-lock transitions;
- nonlinear line guidance to the radar intercept point;
- terminal RF PN and compensated IR PN;
- adaptive roll, rate, and acceleration control;
- requested control, achieved physical effectors, and final body wrench;
- actor-qualified controller events and mode transitions.

The aerodynamic module owns one derivative ledger. The controller consumes that exact ledger at the following `ins` boundary rather than repeating deck queries against a different state snapshot.

## Source parser correction

Repeated initial assignments are legal source behavior. The AST keeps every declaration in order and resolves the final value for execution. This is required by the shipped ADS6 aircraft-defense input, which declares `gain_rf` twice.

## Evidence boundary

The controller uses truth-aligned navigation for all requested INS modes. RF glint, full radar-equation/SNR and thermal-noise corruption, complete IR focal-plane/aimpoint behavior, exact stochastic sequences, complete mode-4 TVC command routing, and compiled-CADAC numerical parity remain outside the promoted claim. `hold` and `line_of_sight` remain explicit comparison laws rather than fallback implementations.
