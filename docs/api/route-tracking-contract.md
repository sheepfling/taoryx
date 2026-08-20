# Route-tracking telemetry contract

Every vehicle route run uses the same signed, SI-labelled navigation channels.
The runtime may omit a channel when its route geometry cannot define it; it
must not substitute a different quantity under the same name.

| Channel | Meaning | Units |
| --- | --- | --- |
| `route_target_error_m` | Distance to the active reference point | m |
| `route_cross_track_error_m` | Signed lateral distance to the route tangent | m |
| `route_along_track_error_m` | Signed error along the route tangent | m |
| `route_heading_error_deg` | Velocity heading relative to route tangent | deg |
| `route_bank_command_deg` | Commanded fixed-wing bank/roll | deg |
| `route_bank_achieved_deg` | Achieved local roll compatibility channel | deg |
| `route_bank_tracking_error_deg` | Command minus achieved local roll | deg |

Square routes additionally expose `route_leg_index`. Smooth figure-eight
routes expose `route_phase_index`; these are not interchangeable. A smooth
route's distance to its moving reference point can grow because of phase lag,
so cross-track error is the primary path-following metric.

The contract is machine-readable in
`verification/spec/route_tracking.yaml` and is used by evidence builders and
controller tests.

## Controller authority

Route guidance has one active attitude authority at a time. A declared
continuous LQR owns the attitude error-to-moment command; the coordinated-turn
controller is used for routes without an LQR declaration. The runtime does not
stack both controllers. LQR designs are rejected unless every closed-loop pole
has a strictly negative real part. Vehicle-specific weights remain part of the
problem-file controller contract because inertia and available moment authority
are not portable between vehicle families.
####
