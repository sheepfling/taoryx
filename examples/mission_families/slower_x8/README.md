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
