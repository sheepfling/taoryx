# Closed rectangular course goal

The next maneuvering flag is a closed rectangle, not a one-way fly-by.
Each vehicle starts at the southwest corner, travels east, turns north at
the first corner, reaches an elevated northeast corner, turns west, descends
through the northwest corner, and returns south to the start corridor.

The route shape is selected through the generic runtime route attributes:

```text
*runtime status route mode=rectangle \
  start-latitude-deg=... start-longitude-deg=... \
  start-altitude-m=... rectangle-length-m=... rectangle-width-m=... \
  elevated-corner-altitude-m=... duration-s=...
```

The course is vehicle-scaled: dimensions and leg time must remain inside the
declared coefficient, speed, altitude, and actuator envelopes. A corner
transition window may be declared for fixed-wing vehicles so the controller
has time to coordinate the turn. The current development scales provide
approximately 80 seconds per B747 leg, 30 seconds per X8 leg, and 10 seconds
per Hummingbird leg. The B747 and X8 corner windows are 75 and 30 seconds
respectively, chosen to keep the turn radius compatible with the declared
speed and bank. These are comfortable research-course timings, not
mission-performance claims.

For a route that must correct position error, `position-capture-gain` may be
paired with `position-capture-max-correction-mps`. The latter bounds the
additional commanded velocity before attitude guidance sees it, preserving
the vehicle's aerodynamic beta authority instead of converting a large corner
miss into an impossible lateral demand. The cap is a generic route-controller
primitive; it does not relax table boundaries or create a waypoint-capture
claim by itself.

## Success gates

The course is evidence-ready only when all of these pass:

1. The run completes all four legs and the return event.
2. Final horizontal distance from the start is within the declared corridor.
3. Final altitude is within the start-altitude corridor.
4. The elevated-corner altitude is reached within tolerance.
5. Every active table query remains in-envelope.
6. Alpha, beta, speed, body rates, actuator commands, and mass remain within
   declared vehicle limits.
7. Force/moment closure remains below the research threshold away from route
   corner events.
8. The result is convergent at `dt`, `dt/2`, and `dt/4`.
9. The packet includes a plan-view route plot, altitude profile, speed,
   attitude, aerodynamic angles, forces, moments, and table margins.

The current development fixtures are:

- `slower_b747/SV01_rectangle_course_6dof.prb`
- `slower_x8/SV03_rectangle_course_6dof.prb`
- `slower_hummingbird/SV05_rectangle_course_6dof.prb`

They are not yet promoted into the passing evidence manifest. The current
one-way maneuver manifest remains the passing baseline until each closed
course satisfies the gates above.

The B747 and X8 development fixtures now declare the existing TAORYX attitude
LQR contract with provisional diagonal weights. These declarations exercise the
generic inner-loop path; they are not claims that the supplied default
linearization reproduces a source vehicle mode. Source-anchored A/B matrices,
trim residuals, closed-loop poles, and small-disturbance comparisons remain
required before an LQR configuration becomes vehicle evidence.

## Bounded gain tuning

The controller search is deliberately an external development tool. It copies
one of the native `.prb` fixtures into a temporary candidate directory,
changes only ordinary `*runtime status` attributes, runs the same TAORYX
parser and integrator, and writes a ranking JSON file:

```text
python tools/tune_rectangle_controller.py --case b747 --evaluations 8
python tools/tune_rectangle_controller.py --case x8 --evaluations 8
python tools/tune_rectangle_controller.py --case x8 --method differential-evolution --evaluations 48
python tools/tune_rectangle_controller.py --case b747 --phase turn-prefix --evaluations 8
```

The random search is deterministic for a fixed `--seed`. The differential
evolution backend uses the same bounded native evaluator with a fixed seed;
its population means the exact number of evaluations may be slightly above
the requested budget. `turn-prefix` is a deliberately incomplete first-corner
search; its ranking identifies stable turn behavior, but its candidates must
still pass a subsequent `--phase full` run. A candidate is `safe` only when it
completes the full native course, has no configured angle, speed, altitude, or
corner-capture violation, and reports all four corner errors.
`completed_unsafe` means it reached the time horizon but is not evidence-ready;
`incomplete` means it hit the runtime step limit; `failed` means ingestion or
execution failed. The X8 search is bound to the current checked-in long
rectangle fixture rather than the older short development fixture. Its
current baseline remains `completed_unsafe`: the best non-origin corner
captures are approximately 28 m, 217 m, and 89 m against the 25 m research
gate. Rankings are exploratory artifacts and never replace the fixed baseline
fixtures or the closed-course acceptance tests.
