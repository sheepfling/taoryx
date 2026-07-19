# Vehicle maneuver matrix

The standard vehicle catalog needs more evidence than a single rectangular
course. The machine-readable source is
`verification/vehicle_maneuver_matrix.yaml`.

Runnable problem-file bindings live in
`verification/maneuver_evidence.yaml` and are executed with:

```text
python tools/dev.py maneuver-matrix
python tools/run_maneuver_matrix.py --only b747/trim-hold --plots
```

The runner writes a development bundle below
`artifacts/vehicle-maneuver-matrix/`, including the native run report, plot
manifest, and classified matrix summary. A bound case may still be incomplete
or unsafe; that status is preserved in the report. The summary also reports
dimension coverage, so a `[3dof, 6dof]` matrix row is not considered complete
until it has one binding for each dimension.

Use `python tools/run_maneuver_matrix.py --aggregate-only` to rebuild the
summary from current fingerprinted binding reports without rerunning the
simulations. The report's `passing_matrix_rows` field counts rows with at least
one completed, limit-respecting declared dimension; unsafe sibling dimensions
remain visible.

The runner distinguishes four execution outcomes:

- `completed`: the native problem reached its stop condition and declared
  limits were respected;
- `completed_unsafe`: the stop condition was reached, but an evidence limit
  such as alpha, sideslip, speed, or altitude was violated;
- `incomplete`: the runtime step limit was reached first;
- `failed`: the runtime produced no usable result.

These are evidence classifications, not engineering-validity claims. A
completed native run may still be only a local research-surrogate result.

Every fixed-wing maneuver is developed in two stages:

1. a 3-DOF reduction using the same atmosphere, tables, propulsion, mass, and
   control laws;
2. a 6-DOF run using the same plant with attitude, rates, actuators, and LQR.

The Hummingbird 3-DOF reduction is a translational thrust-vector model, not a
fixed-wing lift/drag model. X-15 cases remain separate from slower-vehicle
evidence because their atmospheric and control envelopes differ.

## Required maneuver families

- trim or hover equilibrium;
- takeoff, launch, or powered climb;
- altitude step and recovery;
- heading or yaw step;
- rounded racetrack or waypoint square;
- descent corridor;
- wind and actuator disturbances;
- return, go-around, or benign terminal waypoint capture.

Landing rows are currently non-contact approach/go-around checkpoints. They
exercise native 6-DOF propagation and envelope checks before the contact
boundary. A physical landing claim remains excluded until ground effect, stall,
landing gear, contact, and thermal assumptions are explicitly modeled and
verified.

## Evidence gates

Each available or in-progress case must record:

- convention-firewall and initial-condition report;
- table-query margins and source hashes;
- trim or hover residual;
- controller saturation and rate history;
- route, altitude, and speed tracking metrics;
- force and moment closure;
- `dt`, `dt/2`, and `dt/4` comparison;
- standard Matplotlib artifacts;
- a UUID-scoped manifest.

The matrix is a staged work plan. `available` means an existing bounded case
can run; it does not mean the maneuver is a validated engineering result.
