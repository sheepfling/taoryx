# Model fidelity and public-data ceiling

## Selected model

The X-15 was selected because one public executable model supplies all six force/moment channels, controls, rate derivatives, mass properties, tanks, an engine, and a nozzle in a single coordinate framework.

## Why this still falls short of an engineering-qualified deck

1. The source file is marked `BETA`.
2. Its own note says it contains guesses and is intended for educational and entertainment use.
3. Static aerodynamics are compositional rather than a released dense wind-tunnel table.
4. Most lateral/control derivatives are linear and separable.
5. No Reynolds correction, control-interaction grid, covariance, or quantified uncertainty is supplied.
6. No hard roll/yaw actuator-rate limit is supplied.
7. The RCS tank is represented but its thruster geometry and impulse model are absent.
8. Tank inertia and slosh are absent.
9. No certified thermal model is included.

## Better documented but unavailable as raw data

NASA's Shuttle Real-Time Flight Dynamics System documentation describes a database of more than 46,000 points including all coefficients, controls, viscous effects, dynamic derivatives, landing gear, ground effect, nonlinear sideslip, and uncertainties. The underlying coefficient book was identified, but no machine-readable public coefficient release was found in this research pass.

The AFRL Road Runner Generic Hypersonic Vehicle is also described as a public-releasable Mach-6 model with four independent controls and HEAT-TK/RJPA lookup tables. Publicly accessible descriptions were found, but not the actual aerodynamic and propulsion table files.

## Proper use

Use this package for:

- simulator plumbing;
- six-axis sign and frame validation;
- integrator convergence;
- controller architecture prototyping;
- uncertainty and sensitivity framework development;
- comparison of commanded and achievable acceleration.

Do not use it as the sole basis for:

- flight release;
- structural or thermal certification;
- control-authority guarantees;
- route-specific safety claims;
- vehicle performance claims about a modern hypersonic glider.
