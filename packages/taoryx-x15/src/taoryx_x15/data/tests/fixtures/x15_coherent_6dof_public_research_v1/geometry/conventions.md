# Coordinate and coefficient conventions

## Body axes used by the generated tables

- `+X_body`: forward through the nose.
- `+Y_body`: vehicle right.
- `+Z_body`: down.
- `CMX`, `CMY`, and `CMZ` are rolling, pitching, and yawing moment coefficients in the body frame.
- Roll and yaw moments use `qbar * Sref * bref`.
- Pitch moment uses `qbar * Sref * cref`.

The original JSBSim `<location>` values use a structural coordinate system whose X station increases aft. The archive preserves those station values and separately supplies body-relative offsets. Do not insert structural X values directly into a body-axis cross product.

## Wind-to-body conversion

The source model supplies drag, side-force, and lift-axis contributions. The archive converts `[-CD, CY, -CL]` into body coefficients using alpha and beta:

```text
CX = -CD*cos(alpha)*cos(beta) - CY*cos(alpha)*sin(beta) + CL*sin(alpha)
CY = -CD*sin(beta) + CY*cos(beta)
CZ = -CD*sin(alpha)*cos(beta) - CY*sin(alpha)*sin(beta) - CL*cos(alpha)
```

Angles and control derivatives are evaluated in radians. Rate derivatives use:

```text
p_hat = p*b/(2V)
q_hat = q*c/(2V)
r_hat = r*b/(2V)
```
