# Fixed-wing racetrack course contract

`mode=racetrack` is the reusable fixed-wing breathing-course template. It is
defined in the local tangent plane at the declared start point and closes at
the start/finish point:

```text
outbound climb → outbound level → left semicircle
→ inbound descent → inbound level → right semicircle → start/finish
```

The outbound and inbound straights are intentionally long enough to isolate
the vertical changes from the bank reversals. The turns are semicircles, so the
reference tangent is continuous and the turn directions are opposite.

Required route attributes:

```text
mode=racetrack
racetrack-length-m=...
racetrack-turn-radius-m=...
racetrack-speed-mps=...
racetrack-low-altitude-m=...
racetrack-high-altitude-m=...
racetrack-climb-rate-mps=...
racetrack-descent-rate-mps=...
duration-s=...                 # normally the estimate below, with margin
```

For a vehicle whose attitude response lags the nominal flight-path command,
`racetrack-altitude-capture-gain-per-s` and
`racetrack-altitude-capture-max-mps` provide an explicit bounded vertical
capture term. It is still a guidance velocity request; it does not inject
vertical force or bypass the vehicle controller. The truth altitude gates
remain the acceptance authority.

The first-pass timing estimate is:

```text
straight_time = length / speed
turn_time = pi * turn_radius / speed
climb_time = (high_altitude - low_altitude) / climb_rate
descent_time = (high_altitude - low_altitude) / descent_rate
total_time = 2 * straight_time + 2 * turn_time
```

The estimator rejects a course when either vertical phase would consume the
whole straight. The result is a horizon and objective-window estimate, not a
claim that the vehicle can meet the climb, descent, turn radius, or terminal
gate. Those are independently checked from truth telemetry.

The route phase index is:

| Index | Phase | Expected behavior |
|---:|---|---|
| 0 | outbound climb | positive flight-path command, zero bank |
| 1 | outbound level | level, straight, high altitude |
| 2 | left turn | high-altitude semicircle, positive route curvature |
| 3 | inbound descent | negative flight-path command, zero bank |
| 4 | inbound level | level, straight, low altitude |
| 5 | right turn | low-altitude semicircle, opposite curvature |

The X8 fixture uses the existing attitude/LQR and surface-control seam. The
racetrack mode supplies only the geometric velocity and scheduled bank
reference; it does not inject forces or bypass the rigid-body equations.
