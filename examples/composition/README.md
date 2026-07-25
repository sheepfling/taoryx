# Composition examples

These examples demonstrate successor-side composition over validated TAOS
source files. They are synthetic TAORYX workflows and do not claim historical
TAOS 96.0 numerical compatibility.

Run the launch-variant example from the repository root:

```bash
python3 examples/composition/launch_variants.py --output-dir build/composition
```

The example compiles the source mission, applies a geodetic launch state and a
local-frame velocity impulse, runs the resolved scenario, and writes the first
case as JSON, CSV, SQLite, and artifact-only HTML. All generated products are
under `build/`.

The composition API is intentionally semantic:

```python
scenario = ScenarioCompiler().compile(
    problem,
    table_paths=(table,),
    patches=(
        InitialFrameState(
            vehicle="1",
            frame="geodetic",
            position=(-80.6, 28.5, 30_000.0),
            velocity=(100.0, 5.0, 90.0),
        ),
        VelocityImpulse(vehicle="1", frame="geodetic", delta_velocity=(0.0, 0.0, 12.0)),
    ),
)
```

The source remains unchanged; the resolved scenario records the patch payload
and derives a different deterministic identity.

Scalar and frame-component patches may carry explicit units. They are converted
through the runtime unit boundary before state application, with the applied
before/after values retained as resolution records in the run artifact.

Run the explicit initial-kick example, including its no-attitude negative case:

```bash
python3 examples/composition/initial_kick.py
```

Run the interactive and staged event examples as well:

```bash
python3 examples/composition/interactive_replay.py
python3 examples/composition/staged_signals.py
python3 examples/composition/artifact_round_trip.py
python3 examples/composition/mode_validation.py
```

The first writes command history, status/event metadata, CSV, JSON, and HTML.
The second records a deterministic stage transition and terminal stop event in
JSON, SQLite, and HTML. Its console output also shows each typed pre/post truth
pair, including the event time, segment handoff, physical discontinuity flag,
and state values. Both examples use synthetic runtime models and are not
historical TAOS output claims.

For the source-language version of the same contract, inspect and run
`examples/taoryx/syntax_fragments/12_transition_truth.prb` with the `taoryx`
profile. The source uses ordinary `*when`, `goto`, `*increment`, and `stop`
syntax; the runtime supplies the transition truth audit automatically.

The round-trip example reloads JSON and writes the same run through all output
sinks. The mode example accepts a control only for its declared dynamics mode
and verifies that an invalid mode request is rejected.

`source_runtime_branch.py` is the source-to-runtime branching example. It
ingests `source_runtime_branch.prb`, lowers its staged flight sequence, executes
the time-varying `scheduled_throttle` command and cumulative `fuel_used` stash,
then clones the executable graph at `t=1.0` for an independent continuation.
The fixture is a TAORYX extension demonstration, not a historical TAOS runtime
compatibility claim.
