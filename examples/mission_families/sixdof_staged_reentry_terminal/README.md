# Rudimentary 6-DOF staged reentry and terminal guidance

Status: scaffolded TAORYX extension.

This family is intentionally outside the historical TAOS claim boundary. It
uses the new rigid-body 6-DOF state contract for a future two-stage launch,
coast-to-apogee, thermally constrained entry, and terminal ProNav handoff.

The current implementation provides:

- ECIC position and inertial velocity;
- body-to-ECIC quaternion attitude;
- body angular rates and diagonal inertia dynamics;
- propellant mass-flow and heat-load state;
- explicit stage, coast, entry, terminal, and impact phase labels;
- event-validated phase transitions;
- a thermal entry controller that bounds angle-of-attack commands.

The rigid-body equations are integrated in ECIC/ECI. Earth-fixed atmosphere
and wind providers must use `EarthRotationAdapter` to obtain ECFC position and
Earth-relative velocity, including the transport correction for Earth
rotation. This is separate from the historical TAOS point-mass ECFC path.

The remaining scenario-level work is to connect stage-specific force and
inertia models, atmospheric drag/heating, and the ProNav controller into one
independent launch-to-impact oracle. Until then, this directory documents the
extension boundary and the unit tests are the executable baseline.
