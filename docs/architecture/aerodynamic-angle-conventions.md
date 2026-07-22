# Aerodynamic-angle and attitude conventions

This is the semantic firewall for angle telemetry, table queries, and
controller commands. A channel may only be compared with another channel when
its frame, reference vector, branch, and sign convention match.

## Canonical axes

Fixed-wing body axes are:

- `+X_B`: forward;
- `+Y_B`: vehicle right;
- `+Z_B`: down.

The rigid-body state stores attitude as a quaternion mapping body vectors into
ECIC. Human-facing attitude telemetry is resolved into the local geocentric
NED frame (`north, east, down`), not read from ECIC Euler angles.

## Distinct angle families

| Quantity | Meaning | Frame/reference | Runtime channel |
| --- | --- | --- | --- |
| `alpha` / AoA | Angle between body longitudinal direction and the air-relative velocity projection in the body symmetry plane | body and wind | `aero_alpha_deg` |
| `beta` / projected sideslip | Projection-based sideslip angle | body and wind; singular near `alpha = +/-90°` | `aero_sideslip_deg` |
| `betae` / Euler sideslip | Euler wind-axis sideslip angle | body-to-wind Euler sequence | not substituted by `aero_sideslip_deg` |
| `alphat` / total AoA | Unsigned angle between body `+X` and air-relative velocity | body and wind; `[0°, 180°]` | explicit source/query channel only |
| `phi` / windward meridian | Azimuth of the air-relative velocity in the body transverse plane | body and wind; paired with `alphat` | explicit source/query channel only |
| `bankgc` / geocentric bank | Rotation of the wind/vehicle transverse plane about the velocity direction, referenced to the geocentric horizon | velocity + local geocentric horizon | command/query only unless explicitly derived |
| `bankgd` / geodetic bank | Same bank concept referenced to the geodetic horizon | velocity + local geodetic horizon | command/query only unless explicitly derived |
| `rollgc` / `rollgd` | Body Euler roll angle relative to the corresponding local horizon | body + local horizon | `local_roll_deg` is the successor's geocentric-NED diagnostic |
| FPA | Direction of the velocity vector relative to the local horizontal | velocity + local horizon | `flight_path_angle_deg` |
| heading | Azimuth of the velocity vector, clockwise from local north | velocity + local horizon | `local_heading_deg` |

The manual explicitly distinguishes geocentric/geodetic bank from geocentric/
geodetic body roll. They are related but are not the same named quantity.
Likewise, FPA and heading describe velocity direction; they do not describe
body attitude.

## Sign and branch rules

For the canonical fixed-wing body convention, with air-relative velocity
components `(u, v, w)` in body axes:

```text
alpha = atan2(w, |u|)
beta  = atan2(v, sqrt(u^2 + w^2))
```

Thus positive AoA is velocity toward body `+Z_B` (down), and positive
sideslip is velocity toward body `+Y_B` (right). These formulas are the
ordinary forward-flight branch used by the rigid-body table adapter. Near zero
air-relative speed the angles are unavailable; they must not be interpreted as
physical values.

The manual's special branches for `alpha = +/-90°`, total AoA, and windward
meridian remain separate semantic cases. No renderer or controller may replace
them with a generic wrapped Euler angle.

## Bank versus roll in TAORYX telemetry

`local_roll_deg` is the measured body attitude about the local forward axis.
`bank_achieved_deg` and `route_bank_achieved_deg` are compatibility/controller
channels currently populated from that local-roll diagnostic when a bank
command or route controller is active. They should be read as **achieved local
roll**, not as proof that the aerodynamic bank variable was independently
computed.

`aero_query_*` channels are the exact values sent to a coefficient table. If a
table happens to have an axis named `bank`, `aero_query_bank-deg` is a
frame-relative lookup coordinate. It is not a physical attitude channel and
must never be the default source for a human-facing bank plot. The B747 value
of `180°` came from this distinction; its local physical roll is approximately
`0°`.

## Evidence requirements

Every vehicle evidence packet must identify:

1. body-axis convention and source frame;
2. local horizon convention (geocentric or geodetic);
3. whether the plotted bank is commanded, achieved local roll, or independently
   derived aerodynamic bank;
4. the exact alpha/beta family used by the table;
5. the zero-speed and branch handling policy.

The showcase therefore labels a local-roll fallback explicitly and excludes
raw aerodynamic query-bank coordinates from the physical bank series.

Sources: reconstructed Manual sections 2.1.7--2.1.9 and 2.2.1, especially
`manual/chapters/chapter02/01_07_body_fixed.tex`,
`manual/chapters/chapter02/01_09_wind.tex`, and
`manual/chapters/chapter02/05_00_guidance_rules.tex`.
