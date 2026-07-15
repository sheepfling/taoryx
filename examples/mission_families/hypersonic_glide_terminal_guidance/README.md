# Hypersonic glide with terminal guidance

Status: scaffolded.

This scenario combines a lifting point-mass vehicle, a descending atmospheric
trajectory, a ground target, and terminal guidance. The `.prb` source
exercises the manual-shaped path; the extension controller has an independent
bounded-descent and terminal-hit oracle.

The source pack contains:

- Mach/angle-of-attack lift and drag tables;
- atmosphere and Earth model declaration;
- initial suborbital state;
- guidance/control limits;
- moving or fixed ground target;
- output file containing altitude, range, and guidance variables.

This is a TAORYX 3-DOF extension scenario. It does not require rigid-body
moments or actuator dynamics. The current `.prb` case exercises the
manual-shaped atmospheric/glide path; the independently verified
`HypersonicGlideTerminal` controller supplies the extension-level terminal-hit
oracle while the legacy `*fly propnav` behavior is still being adjudicated.
Full completion requires connecting that terminal oracle to the source mission
and resolving the guidance contract.
