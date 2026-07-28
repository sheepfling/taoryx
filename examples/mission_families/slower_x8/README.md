# Skywalker X8 powered-trim pair

`SV03_3dof.prb` is the point-mass baseline. `SV03_6dof.prb` uses the native
rigid-body integrator and the zero-control X8 identified coefficient deck near
its published reference region. Wind, elevon control, and the glide follow-up
remain separate cases so this local trim pair does not overclaim the source
model envelope.

`SV03_long_anchor_propagation_6dof.prb` is the bounded open-loop propagation
case. `SV03_bounded_route_controller_6dof.prb` exercises the native route-
attitude controller for a bounded 0.1-second response.
`SV03_long_controller_recovery_6dof.prb` is the retained 0.5-second collective-
elevon alpha-hold diagnostic. The generated
`examples/generated/vehicles/skywalker_x8_controller_recovery_10_6dof.prb`
is the current metadata-driven 10-second powered recovery case. It composes
the static, collective-elevon, differential-elevon, and thrust tables and uses
the declared source-neighborhood controls. Its enlarged moment allowance is a
notional research-controller setting, recorded so that this case exercises the
plant and envelope for a useful duration without claiming a published X8
flight-control law.

`SV03_combined_controller_recovery_6dof.prb` is the harmonized five-second
longitudinal/lateral recovery gate. It uses the same composed coefficient deck
as the powered trim and lateral response, with collective alpha hold and
differential sideslip/rate feedback active together. This remains a bounded
research-controller demonstration, not a source-validated flight-control
claim.

`SV03_lateral_rate_response_6dof.prb` is the independent lateral gate. It uses
the composed static, collective-elevon, and differential-elevon decks plus
generic restoring sideslip and body-rate feedback, and runs for five seconds
without leaving the local alpha/beta envelope or saturating the actuator. The
composition is the ordinary source-table expression
`static + (collective - static) + (differential - static)`: it is an explicit
additive research assumption, not a claim that the source provides a coupled
nonlinear superposition. The three source rate-effect slices are also retained
as verified catalog fixtures; they are evidence for source transcription and
are not silently substituted for the static/control deck in the runtime. The
combined recovery fixture is the first re-verification of that coupled path;
longer route and mission claims remain separate.

When several `.tbl` files define the same TAOS output name, the runtime binds
them by deterministic source-qualified aliases such as `cx-static`,
`cx-collective`, and `cx-differential`. This keeps the `.prb` language surface
ordinary while making table-family provenance visible and preventing silent
last-table-wins behavior.

## Racetrack control-realization comparison

`SV03_racetrack_altitude_turns_direct_moment_6dof.prb` is the explicit legacy
baseline:

```text
racetrack guidance → attitude LQR → direct canonical body moment → plant
```

It is useful as a trajectory/controller integration witness, but it is not an
elevon-allocation qualification.

`SV03_racetrack_altitude_turns_6dof.prb` is the physical-realization candidate:

```text
racetrack guidance → bank/pitch response law → bounded local elevon inversion → aero tables → plant
```

The candidate uses only the controlled roll/pitch subspace of the collective and
differential elevon pair, bounded travel, and a bounded deflection offset from
the source trim. It reports requested versus local-linearized achieved moment,
actual aero moment, actual allocation residual, and saturation state. The X8
has no independently declared yaw effector in this case; coupled yaw/sideslip
response remains in the plant and is an explicit residual rather than a
direct-moment bypass. The local linearization is allocator evidence, not a
full manufacturer actuator model.

The direct baseline is plotted with a different signal contract. Its raw
controller request, limited command, final injected moment, aerodynamic load,
and total moment are separate telemetry channels. The route bank and pitch
channels are desired attitude references, not body-moment commands. A
near-zero total moment in the direct baseline means that the injected
generalized moment is cancelling the aerodynamic load; it does not mean that
the elevons achieved that load. The default direct LQR design currently uses
the rigid-body inertia bridge rather than a source-aerodynamic
stiffness/damping linearization, so a large bank-reference tracking error can
be a real plant/controller limitation. The direct case remains an integration
baseline until that plant-aware design is closed.
The current nominal packet passes all six independent truth objectives and the
declared source envelope, including the oriented terminal crossing at 169.58 s.
This is a fixed-configuration nominal result, not a family, robustness,
flight-test, or manufacturer-controller qualification. The closed racetrack
uses the same signed physical turn curvature for both semicircular return
lobes; an explicit opposite-bank-direction exercise remains separate.
The tables must not be widened and a direct yaw moment must not be reintroduced
to make this candidate pass.
