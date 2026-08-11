# CADAC ADS6 SRBM plug-in

`cadac.ads6.srbm` is the exact runnable ADS6 short-range ballistic-missile actor. The catalog identity is `SRBM5`; the standalone CADAC input object is `ROCKET5`.

## Fidelity

The model is T2 `pseudo_6dof` with a `response_law` realization. It integrates Flat3 translation and reduced-order alpha/beta response states. It does not claim quaternion attitude, body-rate truth, or moment closure.

## Source phases

- `prelaunch`: launch-delay hold.
- `endo_ascent`: endo acceleration response with the source ascent normal-bias command.
- `exo_ballistic`: exo coast with response states reset and no active incidence controller.
- `endo_reentry`: endo response, optionally driven by the fixed-target PN/spiral guidance path.

The exo flag remains latched after first crossing the configured endo altitude, allowing ascent and reentry to be distinguished without changing the T2 fidelity claim.

## Source ownership

The provider owns source lowering, ordered module execution, source events, extended US76 atmosphere, pressure-corrected propulsion, mass depletion, lift/drag lookup, reduced-order response, Flat3 translation, kinematic fixed-target seeker, PN/spiral commands, and source termination behavior. Input alpha/beta initialize the published aerodynamic variables only; the internal response-law states begin at zero, matching CADAC. Low-Mach table extrapolation is preserved even when the resulting source drag coefficient is negative.

## Result composition

Mission Composition returns one independent root object:

```text
ads6-srbm-1  cadac.ads6.srbm
```

Core state contains position and velocity only. Alpha, beta, heading, flight path, atmosphere, propulsion, guidance, and seeker quantities are telemetry rather than fabricated rigid-body channels.

## Composition, controls, and sensors

The installed provider supports both `batch` and persistent `step` execution.
Each session retains the ROCKET5 translation, propulsion, phase, response-law,
and termination state and accepts only holds that are exact multiples of the
source `int_step`. Its controller is source-owned, so there are no caller
action channels. Instead, the standard outputs distinguish
`normal_command_g` / `lateral_command_g` from the realized
`normal_acceleration_g` / `lateral_acceleration_g` specific-force responses.
Those outputs support finite-run control-trace analysis and controlled
comparisons; no trim, local linear stability, or frequency-margin claim is
made.

At each accepted source boundary the session also publishes an
`ads6-srbm-native-relative-state` packet through the Taoryx `SensorBus`. It is
a typed, geometry-only `relative-state-track` for the configured fixed target.
This packet does not replace the ROCKET5 source seeker's enable, endo/exo, and
guidance behavior, which remain separately visible in the source sensor
telemetry. The local-NED adapter carries no CADAC gravity or atmosphere model
into the shared Taoryx sensor interface.

## Source-data smoke

The verification artifact separates exact-step and structural evidence. Both pinned upstream SRBM cases execute through `70 s` at the source `0.001 s` step and cross from endo ascent into exo ballistic coast near `63 s`. Full-course structural runs use `0.01 s`: the ballistic case reaches the configured `470 s` end time after reentry, while the PN-plus-spiral case reaches target closest approach near `477.670 s` with a reconstructed `1.864 m` miss. The accelerated results are not parity evidence. No CADAC source or deck is bundled, full-course exact-step completion remains open, and no compiled-executable parity claim is made.

## Evidence boundary

The standalone runtime does not claim the ADS6 SAM, `AIRCRAFT3`, `RADAR0`, radar scheduling, package actor-order communication, rigid-body attitude, or compiled-CADAC numerical parity. Exact dispatch cannot fall back to another ADS6 model or fidelity tier.
