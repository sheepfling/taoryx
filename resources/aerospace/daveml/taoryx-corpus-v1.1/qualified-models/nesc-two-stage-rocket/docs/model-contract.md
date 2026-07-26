# NASA/NESC scenario 17 two-stage rocket

This package binds three semantically normalized DAVE-ML source components:

- axisymmetric aerodynamics as functions of angle of attack and sideslip;
- two constant-thrust, constant-specific-impulse propulsion stages; and
- fuel-scheduled mass, center of gravity, moment reference center, and inertia.

## Mission logic

Stage 1 ignites at liftoff and burns to source-defined propellant depletion. The
full stack then coasts for 96.79 seconds. At the next event the dry first-stage
hardware is jettisoned and Stage 2 ignites. Stage 2 burns to depletion, followed
by an unpowered coast to the 200-second endpoint.

## Runtime dynamics

The validation propagator uses ECI translation, a scalar-first body-to-ECI
quaternion, body angular rates, WGS-84 position conversion, Earth rotation, J2
gravity, atmosphere-relative velocity, MRC-to-CG moment transfer, scheduled
inertia, and a fixed-step event-split RK4 integrator.

## Atmospheric boundary

The lower atmosphere follows the 1976 geopotential layer equations through
84.852 km. Above that altitude the package uses a documented exponential
continuation. Aerodynamic forces are already extremely small in that region.

## Provenance boundary

The included DAVE-ML documents are deterministic semantic reconstructions, not
byte-identical copies of upstream NASA files. Exact upstream repository paths,
commit, and Git blob SHAs are recorded. The trajectory checkpoint table is a
selected comparison subset; full reference-file paths and hashes are retained.

## Acceptance boundary

The package passes source-equivalence, event/mass, numerical-convergence, and
NASA checkpoint-envelope tests. Its 200-second osculating perigee does not meet
the source README's qualitative greater-than-125-km expectation, and that fact
is retained explicitly rather than hidden or tuned away.
