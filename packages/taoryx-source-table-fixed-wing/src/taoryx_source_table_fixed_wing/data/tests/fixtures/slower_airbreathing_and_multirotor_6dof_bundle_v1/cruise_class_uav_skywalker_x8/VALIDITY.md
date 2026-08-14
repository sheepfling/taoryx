# Skywalker X8 validity and interpretation

This is the non-weapon cruise-class fixed-wing surrogate in the bundle.

- Aerodynamic coefficients were identified from flight data near 18 m/s.
- The source paper identifies a nonlinear 6-DOF model and validates it against separate maneuvers.
- Recommended alpha use is approximately 0 to 12 degrees. Post-stall behavior is absent.
- Negative-alpha and far-from-trim predictions have lower confidence.
- The generated grids cover 12 to 27 m/s, 0 to 3000 m, beta ±5 degrees and virtual elevon controls ±20 degrees.
- The implementation's simple thrust model is a calibrated approximation; the published CT(J)/CQ(J) polynomial is supplied separately.
- Battery voltage is represented as a nominal 16 V parameter. Battery discharge, mass change and thermal limits are not modeled.
- The physical left/right elevon differential sign must be reconciled against the source paper before connecting hardware.
- No weapon payload, targeting, terrain following, terminal guidance or mission-performance data are included.
