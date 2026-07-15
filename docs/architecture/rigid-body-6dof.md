# Rudimentary rigid-body 6-DOF extension

TAOS manual compatibility remains a three-degree-of-freedom point-mass
claim. TAORYX may provide a separate rigid-body extension without changing
that boundary.

The extension state is integrated in ECIC/ECI and packed in this order:

```text
position_ecic[3]
inertial_velocity_ecic[3]
attitude_quaternion[w, x, y, z]
body_rate[3]
mass
propellant_mass
heat_load
peak_heat_rate
```

`RigidBody6DofModel` accepts body-frame force and moment providers, rotates
force into ECIC using the attitude quaternion, adds ECIC gravity, and applies
the diagonal-inertia Euler equations. Because the translational equations are
inertial, they do not add ECFC Coriolis or centrifugal terms. Propellant flow
and heat rate are explicit state rates rather than hidden side effects.

The runtime adapter publishes additional channels from that same model
evaluation: body and ECIC forces, body moments, total ECIC force, ECIC
acceleration, propellant flow, heat rate, and roll/pitch/yaw orientation.
These channels are available to `RunArtifact`, SQLite, text, and plot sinks
without duplicating the dynamics calculation.

Environment models may still be Earth-fixed. `EarthRotationAdapter` provides
the explicit boundary:

```text
ECIC position + inertial velocity
    → ECFC position + Earth-relative velocity
    → atmosphere / wind / aerodynamic model
    → force and moment in body axes
    → ECIC force and moment for integration
```

The velocity conversion includes the Earth-rotation transport term. Therefore
a point fixed to the rotating Earth has zero ECFC air-relative velocity when
wind is zero. The historical point-mass kernel remains ECFC and continues to
use its documented rotating-frame terms; this extension does not change it.

The `.prb` lowering path currently remains historical point-mass/kinematic
syntax. A rigid-body `.prb` extension must be added to the language contract
and lowering tests before a new directive is introduced. The California–Hawaii
case is therefore a migration fixture, not evidence that undocumented 6-DOF
syntax is already accepted.

The runtime phase vocabulary is:

```text
stage-1-powered → stage-1-separation → stage-2-powered
                 → coast-to-apogee → entry-thermal-control
                 → terminal-guidance → impact
```

`FlightPhaseMachine` validates event ordering. `ThermalEntryController`
reduces its requested angle-of-attack command as heat-rate or integrated
heat-load margins close. ProNav remains a downstream guidance input: it must
produce a desired acceleration or attitude command, which then passes through
the attitude controller and the force/moment model.

This is a TAORYX extension, not a historical TAOS `.prb` compatibility claim.
The California–Hawaii scenario is a source-driven launch-to-impact verification
fixture executed through the normal file runner and retaining its existing
artifact/plot acceptance oracle.

The source-first runtime path now accepts the native policy declarations used
by the California–Hawaii `.prb` fixture:

```text
*runtime status vehicle dry-mass-kg=8000.0 ...
*runtime status actuator maximum-moment=250000.0 maximum-body-rate-deg-s=3600.0
*runtime status thermal policy=stop maximum-heat-rate-w-m2=... maximum-heat-load-j-m2=...
```

These declarations are lowered into the rigid-body kernel. Dry mass controls
propellant depletion, actuator limits bound the attitude moment and publish
saturation, and `policy=stop` installs thermal limit events. Python showcase
code only invokes the normal file runner and renders the resulting artifact.
