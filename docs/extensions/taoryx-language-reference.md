# Taoryx language extension reference

This is the focused reference for successor `.prb` and `.tbl` syntax. It is
separate from the reconstructed TAOS 96 language: select the `taoryx` grammar
profile when using any construct listed here.

```python
from taoryx.language import GrammarProfile, parse_problem_file

document = parse_problem_file("mission.prb", profile=GrammarProfile.TAORYX)
```

The parser produces a lossless document, typed semantic AST nodes, and
source-located diagnostics. Grammar acceptance is not by itself a claim that a
feature is implemented by every runtime adapter.

## Extension surface

| Surface and scope | AST representation | Runtime status |
| --- | --- | --- |
| `*mode ...` (problem) | `ModeBlock` | point-mass and rigid-body modes supported; kinematic mode is adapter-dependent |
| `*3dof`, `*6dof`, `*sixdof` (problem) | `DofDirectiveBlock` | aliases for point-mass or rigid-body 6-DOF |
| `*method ...` (problem) | `ExtensionBlock` | method/options are runtime-dependent |
| `*mass ...`, `*ptmass` (segment) | `ExtensionBlock` | variable-mass and point-mass transitions |
| `*deployed ...` (trajectory) | `ExtensionBlock` | deployment initialization is supported |
| `*cases` (problem) | `ExtensionBlock` | rows retained for batch adapters |
| `surface_*` helpers (define/expressions) | typed helper or expression AST | supported helper path |
| `maxfit`, `minfit` (summaries) | summary AST function | supported summary path |
| `*atmos`, `*wind` (problem) | `AtmosBlock`, `WindBlock` | adapter, table-data, or file dependent |
| `aero_force`, `aero_moment`, `inertia` (tables) | typed table document | successor table families |
| `*runtime ...` (problem) | `RuntimeBlock` | lowers to shared runtime contract where supported |
| `*runtime sensor ...` (problem) | `RuntimeBlock` + `SensorClockSpec` | registers a provider-neutral accepted-truth clock; a compatible plug-in remains explicit |

The explicit aliases are intentionally normalized in the AST: `*3dof` maps to
`point-mass`, while `*6dof` and `*sixdof` map to `rigid-body-6dof`. Original
source spelling remains available through the lossless source records.

### Sensor timing contract

Sensor clocks are successor syntax. The runtime scheduler takes the minimum of
the vehicle/model cadence, output and event boundaries, explicit truth
timestamps, and the next sensor clock boundary. Solver stages are never
sensor-visible. Instantaneous sensors require a committed boundary; interval
sensors consume the accepted segment. No post-hoc state interpolation is used
to manufacture a measurement. A loaded program exposes normalized clock
declarations as `sensor_clocks` metadata and requires a separate provider to
produce physical measurements, noise, or estimator delivery.

The language `kind` is a broad sensor family such as `imu`, `accelerometer`,
`gyroscope`, `infrared`, `camera`, `gnss`/`gps`, `radar`, or `custom`. Concrete
identifiers such as `imu-error-model`, `ir-bearing`, `ir-point-source`, and
`gnss-fix` are plug-in-sidecar values, not language keywords. A plug-in
manifest declares its compatible language families, and runtime attachment
fails when the clock family and selected provider disagree. Bias, noise,
outage, target selection, focal-plane dimensions, and other family physics
belong to the plug-in configuration rather than `.prb` syntax.

### Segment-transition truth contract

TAORYX does not add a new transition keyword for truth capture. Existing
`*when ... goto`, `*when ... stop`, `*reset`, and `*increment` constructs retain
their documented source meaning. When one of those events is applied by the
runtime, the implementation automatically records a `TransitionTruthPair`:

| Snapshot | Captured when | Required contents |
| --- | --- | --- |
| `pre_truth` | At the accepted event boundary, before the event handler | State, time, frame, named channels, source segment, active status, achieved controls |
| `post_truth` | At the same event time, after reset/increment/segment action | The same fields, including the destination segment and resulting achieved controls |

The pair is available through `RuntimeProblem.transition_history`, loaded
program inspection, checkpoints, and run-artifact event records. The legacy
`event_history` remains compatible and mirrors each pair under `pre_truth` and
`post_truth`, with `transition_index` linking to the typed record.

The two snapshots are immutable copies. Solver stages and rejected trial steps
are never transition truth. A continuous `goto` normally has identical
physical values on both sides and reports `state_discontinuity=false`. A reset,
increment, impulse, staging, or other physical jump must be explicit and is
reported with `state_discontinuity=true`. Sensors and controllers must use the
committed boundary truth; they must not sample between the pair or interpolate
across it.

This is an implementation contract rather than an opt-in language feature:
source syntax declares the transition, while the runtime always supplies the
audit record.

## Representative forms

```text
(successor-example)
*sixdof
*method rk4-fixed variable-mass
*atmos rcc ktf-annual
*wind geodetic file=wind.dat units=ft/sec
*runtime sensor imu kind=imu cadence-s=0.01 sample=instantaneous delivery-s=0 truth=boundary rate-policy=split
*runtime sensor nose_ir kind=infrared cadence-s=0.02 sample=instantaneous delivery-s=0.01 truth=boundary rate-policy=split
*runtime sensor gnss kind=gnss cadence-s=1 sample=instantaneous delivery-s=0.15 truth=boundary rate-policy=split

*trajectory 1 vehicle start on 1
  *initial ecic x=20925646 y=0 z=0 xdt=0 ydt=300 zdt=0 time=0 mass=100
  *segment 1 powered
    *mass xcg=0 ixx=10 iyy=11 izz=12
    *when time>1 stop

*trajectory 2 payload start on 1
  *deployed from trajectory 1, segment 1, wt=25
  *segment 1 coast
    *ptmass
    *when alt<0 stop
*end
```

```text
*define initial variables
  surface_ref(launch_lat,launch_long);
  launch_azimuth = surface_azm(target_lat,target_long);
  target_distance = surface_dist(target_lat,target_long);
*end

*cases
  case launch_lat launch_long target_lat target_long
  1 22.0 -159.8 39.5 -175.5
```

## AST and validation contract

The extension block models are defined in
[`src/taoryx/language/models.py`](../../src/taoryx/language/models.py),
dispatch and source validation are in
[`problem_parser.py`](../../src/taoryx/language/problem_parser.py), and the
reviewed keyword/table contract is in
[`grammar_contracts.py`](../../src/taoryx/language/grammar_contracts.py).
The documentary EBNF is
[`grammars/taoryx_extensions.ebnf`](../../grammars/taoryx_extensions.ebnf).

For focused syntax and negative-diagnostic coverage, run:

```bash
taoryx-validate --profile taoryx \
  examples/taoryx/syntax_fragments/*.prb \
  examples/taoryx/syntax_fragments/*.tbl
python tools/dev.py test-grammar
```

The focused fixtures are listed in
[`examples/taoryx/syntax_fragments/README.md`](../../examples/taoryx/syntax_fragments/README.md).
Larger combined cases are in
[`examples/taoryx/full_examples/`](../../examples/taoryx/full_examples/).

## End-to-end artifacts

To regenerate parser reports, normalized artifacts, telemetry, and plots for
the extension corpus:

```bash
PYTHONPATH=src .venv/bin/python examples/run_corpus.py \
  --family taoryx --execute --output artifacts/examples/taoryx
```

The shared output contract is documented in
[`examples/taoryx/output_contract.yaml`](../../examples/taoryx/output_contract.yaml).
A short run can return exit code `1` when its step budget is exhausted after
writing artifacts; parser errors and runtime exceptions are reported separately.

## Claim boundary

`taos96` remains the default historical-language profile. It is an
evidence-bounded language and specification profile, not a claim of behavioral
equivalence with the unavailable historical executable. `taoryx` is the
successor profile. The local runtime can execute the supported subset, while
successor extensions must remain explicitly labeled as Taoryx behavior.
See [`taos96-evidence-bounded-profile.md`](../verification/taos96-evidence-bounded-profile.md)
for the release boundary and approved wording.
