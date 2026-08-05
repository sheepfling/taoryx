# Product 2: find, set up, step, and diagnose a scenario

This is the shortest route for a new engineer or a consumer who wants to run
an existing TAORYX model. Product 2 is the simulation and stepping platform:

```text
source or composed model
        -> validate / inspect
        -> resolve and lower
        -> batch run or external step loop
        -> telemetry, events, diagnostics, and artifacts
```

The repository has several kinds of “model.” Keeping them separate prevents
the most common setup mistake:

| Name | What it is | Typical location |
| --- | --- | --- |
| Source scenario | A `.prb` problem and its `.tbl` inputs. | `examples/mission_families/`, `examples/showcases/`, `examples/vehicle_families/` |
| Family/model package | Source identity, bindings, assumptions, and qualification metadata. | `families/` |
| Composition request | A typed YAML/JSON selection of vehicle, fidelity, initialization, and ordered mission segments. | `examples/vehicle_composition/` |
| Compiled composition | The immutable, fingerprinted handoff produced from a request. | `artifacts/` or a caller-selected generated directory |
| Run artifact | Normalized Product 2 telemetry and event data from a source run or step session. | caller-selected `artifacts/` directory |

This page uses `pseudo-6DOF` for the repository’s exact `pseudo_6dof`
identifier. It is not “full six degrees of freedom with some fields hidden.”
The current contract is translational integration plus a named attitude/rate
response law. It does not, by itself, prove integrated moments, a participating
control system, gimbal/surface allocation, or physical-effector behavior.

If the `taoryx` command is not on `PATH`, use `.venv/bin/taoryx`. If the
`python` alias is unavailable, use `.venv/bin/python` for the commands below.
Run all commands from the repository root.

## The five-minute first run

Start with a small source-first staged rocket. It exercises parsing, table
loading, changing mass, stage transitions, and ordinary batch artifacts without
requiring a vehicle composition request.

```bash
taoryx-validate --profile taos96 \
  examples/mission_families/two_stage_ballistic_rocket/mission.prb

taoryx run \
  examples/mission_families/two_stage_ballistic_rocket/mission.prb \
  --profile taos96 \
  --integrator rk4 \
  --max-steps 20000 \
  --output-dir artifacts/product2/two-stage-ballistic \
  --report artifacts/product2/two-stage-ballistic/run-report.json \
  --artifact artifacts/product2/two-stage-ballistic/run-artifact.json

taoryx artifact inspect \
  artifacts/product2/two-stage-ballistic/run-artifact.json

taoryx artifact plot \
  artifacts/product2/two-stage-ballistic/run-artifact.json \
  --output-dir artifacts/product2/two-stage-ballistic/plots
```

The source run and the normalized artifact answer different questions. The
source output files show what the historical-style problem requested; the
`RunArtifact` is the stable Product 2 telemetry surface used by plots and
downstream consumers. A successful run is not automatically a vehicle or
mission qualification result.

## Find the right scenario

Use this map before opening files at random. The three similarly named Hawaii
paths are deliberately different witnesses.

| Interest | Start here | Fidelity / operation | What it proves | Important nonclaim |
| --- | --- | --- | --- | --- |
| Small staged-rocket runtime smoke test | [`two_stage_ballistic_rocket`](../examples/mission_families/two_stage_ballistic_rocket/README.md) | Point-mass 3DOF, source batch | Powered stages, coast, and ballistic return execute through the ordinary runner. | Synthetic fixture; no rigid-body attitude or engineering performance claim. |
| Explicit changing-mass/separation example | [`two_stage_demo`](../examples/vehicle_families/staged_rocket/two_stage_demo/README.md) | Point-mass 3DOF, source batch with `aero.tbl` and `propulsion.tbl` | Reusable source tables, stage transitions, mass change, and child-state inheritance. | Synthetic family example; not historical TAOS numerical equivalence. |
| Staged rocket pseudo-6DOF | [`nesc_staged_source_replay_pseudo6dof_compose.yaml`](../examples/vehicle_composition/nesc_staged_source_replay_pseudo6dof_compose.yaml) | Pseudo-6DOF, source-history batch replay | The retained NESC two-stage event/mass history plus a named attitude-response law. | No participating rocket plant, active guidance, gimbal, or physical separation dynamics. |
| HL-20 with a staged launch parent | [`hl20_source_booster_release_replay_pseudo6dof_compose.yaml`](../examples/vehicle_composition/hl20_source_booster_release_replay_pseudo6dof_compose.yaml) | Pseudo-6DOF, source-scheduled batch witness | Source-bound HL-20 aero/release/bank/contact sequence with a synthetic booster assumption. | Not a source-exact NASA trajectory, controller, landing, or California-to-Hawaii arrival claim. |
| X-15-scaled staged booster | [`x15_staged_booster_reachability_pseudo6dof_compose.yaml`](../examples/vehicle_composition/x15_staged_booster_reachability_pseudo6dof_compose.yaml) | Pseudo-6DOF, reduced open-loop batch witness | Source-pinned boost, booster release, high-energy glide handoff, and impact witness. | Local-frame X-15-scaled reduction; not the synthetic CA–HI vehicle or a native X-15 controlled mission. |
| Synthetic California–Hawaii route | [`california_to_hawaii`](../examples/showcases/california_to_hawaii/README.md) | Native rigid-body 6DOF, source batch runner | A synthetic staged boost/coast/glide/terminal route and artifact pipeline. | Invented vehicle and assumptions; not HL-20, X-15, historical TAOS, or validated flight design. |
| HL-20 California–Hawaii comparison | [`hl20_california_to_hawaii`](../examples/showcases/hl20_california_to_hawaii/README.md) | Four-fidelity release/glide evidence ladder | Comparable boost, release, glide, terminal, and passive-child telemetry across 3DOF, pseudo-6DOF, rigid-body, and surface-overlay tiers. | Synthetic route and booster assumptions; no target-hit, guidance, thermal, landing, or source-exact route claim. |
| X-15 source-language Hawaii attempt | [`x15_rocket_to_hawaii`](../examples/showcases/x15_rocket_to_hawaii/README.md) | Native rigid-body 6DOF `.prb` migration | Source-language staging seam and bounded native table execution. | The current regression records an out-of-envelope attempt rather than a Hawaii intercept. |

The `families/reference_hl20_mod_k/` and
`families/reference_nesc_two_stage_rocket/` directories are model-family
packages. They are not themselves runnable `.prb` files. Use their metadata to
understand provenance and use the composition or showcase entry points above
to execute a declared witness.

## Load and inspect a source scenario

For a source scenario, inspect inputs before running them:

```bash
taoryx-validate --profile taoryx \
  examples/vehicle_families/staged_rocket/two_stage_demo/mission.prb \
  examples/vehicle_families/staged_rocket/two_stage_demo/aero.tbl \
  examples/vehicle_families/staged_rocket/two_stage_demo/propulsion.tbl

taoryx table inspect \
  examples/vehicle_families/staged_rocket/two_stage_demo/aero.tbl

taoryx table inspect \
  examples/vehicle_families/staged_rocket/two_stage_demo/propulsion.tbl \
  --json

taoryx scenario compile \
  examples/vehicle_families/staged_rocket/two_stage_demo/mission.prb \
  examples/vehicle_families/staged_rocket/two_stage_demo/aero.tbl \
  examples/vehicle_families/staged_rocket/two_stage_demo/propulsion.tbl \
  --profile taoryx \
  --output artifacts/product2/two-stage-demo/scenario.json \
  --json
```

`taoryx-validate` answers “is the source structurally valid?”; `scenario
compile` answers “can this source and table set form a resolved scenario?”;
neither one runs the model. The `table inspect` command is useful for checking
table names, axes, bounds, and interpolation probes before a trajectory is
attempted.

For a composed model, the setup sequence is explicit:

```bash
taoryx vehicle endpoints reference_nesc_two_stage_rocket
taoryx vehicle endpoints hl20_mod_k

taoryx vehicle compose \
  examples/vehicle_composition/nesc_staged_source_replay_pseudo6dof_compose.yaml \
  --output artifacts/product2/nesc-pseudo6dof/composition.json

taoryx vehicle preflight \
  artifacts/product2/nesc-pseudo6dof/composition.json

taoryx vehicle lower \
  artifacts/product2/nesc-pseudo6dof/composition.json
```

`compose` validates and fingerprints the semantic request. `preflight` checks
the mission and required event/segment contract. `lower` selects the exact
family adapter or source-owned factory. A `translation_ready`, `adapter_bound`,
or `factory_bound` result is not yet a run; it is the handoff needed before a
run.

## Run the pseudo-6DOF witnesses

The following is the common batch recipe. Keep the compiled composition and
the run artifacts together so a consumer can reproduce the exact request.

### NESC staged two-stage replay

```bash
taoryx vehicle compose \
  examples/vehicle_composition/nesc_staged_source_replay_pseudo6dof_compose.yaml \
  --output artifacts/product2/nesc-pseudo6dof/composition.json

taoryx vehicle run \
  artifacts/product2/nesc-pseudo6dof/composition.json \
  --output-dir artifacts/product2/nesc-pseudo6dof/run

taoryx vehicle result \
  artifacts/product2/nesc-pseudo6dof/run
```

This is the most direct staged-rocket pseudo-6DOF example. Inspect
`execution.json`, `truth_telemetry.csv`, `status_trace.json`,
`objective_report.json`, `evaluation.json`, `source_provenance.json`, and
`reproduction.txt` together. The attitude and rate channels are a named
response-law projection of the retained source history.

### X-15-scaled staged booster witness

```bash
taoryx vehicle compose \
  examples/vehicle_composition/x15_staged_booster_reachability_pseudo6dof_compose.yaml \
  --output artifacts/product2/x15-staged-pseudo6dof/composition.json

taoryx vehicle run \
  artifacts/product2/x15-staged-pseudo6dof/composition.json \
  --output-dir artifacts/product2/x15-staged-pseudo6dof/run

taoryx vehicle result \
  artifacts/product2/x15-staged-pseudo6dof/run
```

This is a local-frame, source-pinned reduction. Its staged sequence is
booster burn → coast/release → reduced glide handoff → passive impact witness.
It is intentionally separate from both the synthetic CA–HI route and the
native X-15 `.prb` migration.

### HL-20 source-booster release witness

```bash
taoryx vehicle compose \
  examples/vehicle_composition/hl20_source_booster_release_replay_pseudo6dof_compose.yaml \
  --output artifacts/product2/hl20-source-release-pseudo6dof/composition.json

taoryx vehicle run \
  artifacts/product2/hl20-source-release-pseudo6dof/composition.json \
  --output-dir artifacts/product2/hl20-source-release-pseudo6dof/run

taoryx vehicle result \
  artifacts/product2/hl20-source-release-pseudo6dof/run
```

For the complete four-fidelity HL-20 evidence packet, use the dedicated
showcase runner instead of assuming that one pseudo-6DOF run represents the
whole ladder:

```bash
python -m tools.dev showcase-hl20-composites
python -m tools.dev showcase-hl20-source-composites
python -m tools.dev qualify-hl20
python tools/validate_hl20_terminal_contract.py \
  --output artifacts/product2/hl20-ca-hi/terminal-contract.json
```

The first command uses the low-fidelity fixed-`CD`/`L/D` witness. The second
uses the pinned HL-20 DAVE-ML graph and actuator profile. Both retain a
synthetic booster and synthetic California/Hawaii route assumption. The
qualification gates verify the evidence contract; they do not turn the
scenario into a source-exact or arrival-capable flight model.

## Run the California–Hawaii cases without mixing them up

There are three nearby names because they answer different questions:

1. **Synthetic CA–HI route** — run
   `python examples/showcases/california_to_hawaii/run_showcase.py
   --output-dir artifacts/product2/california-to-hawaii`. This is a native
   rigid-body 6DOF synthetic vehicle and route pipeline.
2. **HL-20 CA–HI comparison** — run the HL-20 commands above. This is a
   source-grounded HL-20 model with a synthetic staged launch parent and
   comparison geometry. It is not the synthetic CA–HI vehicle.
3. **X-15 rocket-to-Hawaii** — run the source `.prb` with its pinned X-15 table
   set if you need the native source-language staging seam. It is not the HL-20
   model and is currently expected to demonstrate bounded/out-of-envelope
   behavior rather than a successful intercept.

Do not select the synthetic CA–HI family when the question is “what does the
HL-20 source model do?” Use `reference_hl20_mod_k` and the HL-20 release/glide
entry point. Do not select the X-15-scaled pseudo-6DOF composition when the
question is “does the native CA–HI showcase hit its target?” They have
different models, frames, source assumptions, and claim boundaries.

## External time stepping

Product 2 has one runtime behind both batch and interactive execution. A
consumer supplies a bounded named command at an accepted numerical boundary;
the consumer does not edit raw state or bypass the plant.

```python
from taoryx.runtime.interactive import ControlSpec, InteractiveSession
from taoryx.runtime.program import LoadedProgram

program = LoadedProgram.load(
    "examples/showcases/california_to_hawaii/mission.prb",
    ("examples/showcases/california_to_hawaii/aero.tbl",),
    profile="taoryx",
)

print(program.inspect()["vehicles"])

session = InteractiveSession(
    program.case(),
    controls=(
        ControlSpec("throttle", unit="fraction", lower=0.0, upper=1.0),
        ControlSpec("alpha-deg", unit="deg", lower=-20.0, upper=20.0),
        ControlSpec("bank-deg", unit="deg", lower=-180.0, upper=180.0),
    ),
)

for command in (
    {"throttle": 1.0, "alpha-deg": 0.0, "bank-deg": 0.0},
    {"throttle": 0.8, "alpha-deg": 2.0, "bank-deg": 5.0},
):
    snapshot = session.step(10.0, command)
    print(snapshot.time_start, snapshot.time_end, snapshot.status.value)
    print([item.as_dict() for item in snapshot.commands])

session.to_run_artifact().write_json(
    "artifacts/product2/california-to-hawaii/interactive-artifact.json"
)
```

For a source `.prb`, `*integ dt=...` is the integration cadence and
`dtprnt=...` is an output/print cadence. They are not interchangeable. For
the external API, `InteractiveSession.step(duration, commands)` requests an
external accepted interval; the runtime may use smaller internal steps to
respect event, sensor, or solver boundaries and returns the accepted truth
interval in the snapshot.

The useful interactive operations are:

- `step(duration, commands)` — advance with bounded named controls;
- `pause()`, `resume()`, and `interrupt()` — control the session lifecycle;
- `replay(frames)` — replay a recorded command stream;
- `to_run_artifact()` — project the session into the common telemetry format;
- `save_checkpoint(...)` / `load_checkpoint(...)` — preserve a reproducible
  stepping boundary; and
- `clone_case_at(time)` on `LoadedProgram` — create an independent branch from
  a recorded runtime case.

Composition episodes expose a higher-level equivalent when the selected
family has an episode factory:

```bash
taoryx vehicle episode-info <compiled-composition.json> --seed 7
taoryx vehicle replay-policy <compiled-composition.json> <policy-trace.json>
```

Use `taoryx vehicle endpoints <family>` first. A batch-only model is not
broken because it has no episode endpoint; do not invent an interactive path
for a source-history replay.

## Diagnostic ladder

Run the narrowest diagnostic that answers the current question.

| Question | Command | Interpretation |
| --- | --- | --- |
| Is the source parseable? | `taoryx-validate --profile ... problem.prb tables...` | Source and table diagnostics with file/line locations. |
| Are table names, axes, and bounds usable? | `taoryx table inspect model.tbl --json` | Table inventory and optional interpolation probes. |
| Can the source form a resolved cache? | `taoryx scenario compile ... --output scenario.json --json` | Semantic resolution, not execution. |
| Did the source runtime finish? | `taoryx run ... --report run-report.json --artifact run-artifact.json` | Exit `0` is complete; `1` is incomplete/step-limited; `2` is input or runtime failure. |
| What telemetry was actually emitted? | `taoryx artifact inspect run-artifact.json --json` | Sample counts, time range, channels, events, and artifact metadata. |
| Can the artifact be plotted? | `taoryx artifact plot run-artifact.json --output-dir plots` | Renderer output only; it does not validate the model. |
| Does a composition match its mission contract? | `taoryx vehicle preflight composition.json` | Required inputs, ordered segments, event/terminal checks, and blockers. |
| Which exact adapter/factory is selected? | `taoryx vehicle lower composition.json` | `adapter_bound` or `factory_bound` is a binding result, not qualification. |
| Did a composition run and pass its declared result envelope? | `taoryx vehicle result artifacts/.../run` | Normalized evaluation/artifact validation; inspect native reports too. |
| Did the HL-20 evidence gates pass? | `python -m tools.dev qualify-hl20` | HL20-G0 through G6 evidence contract only. |

When a run stops early, inspect `run-report.json` or `runtime_report.json` for
the diagnostic code and `max_steps`. A short run may be incomplete rather than
failed. Increase the step budget only after checking the integration cadence,
event boundaries, source envelope, and terminal policy.

## What to inspect after a composition run

At minimum, retain and review:

- `composition.json` — the request identity and resolved values;
- `preflight.json` — semantic and capability checks;
- `execution.json` — exact source-owned operation and claim boundary;
- `truth_telemetry.csv` — committed truth samples;
- `status_trace.json` — declared status/resource/diagnostic channels;
- `objective_report.json` — native gate/objective result;
- `evaluation.json` — normalized Product 3 projection;
- `source_provenance.json` — source package and hash lineage; and
- `reproduction.txt` — the command and inputs used to regenerate the run.

For pseudo-6DOF, also look for the response-law identifier and omitted-physics
list. A clean altitude or attitude plot is not evidence of a physical
effector. The realization and the committed truth boundary must be visible in
the artifact.

## Product 2 maturity boundary

The current Product 2 baseline is strong enough for deterministic source batch
runs, named external stepping, replay/checkpoints, normalized telemetry, and
the declared four-tier fidelity vocabulary. The remaining maturity work is
primarily discoverability and uniform evidence packaging:

- make the scenario index and claim boundaries easy to find;
- keep batch-only versus episode-capable endpoints explicit;
- expose the same inspectable artifact shape for source and composed runs;
- compare identical action streams only where a registered batch/episode pair
  exists; and
- keep pseudo-6DOF response laws visibly separate from participating rigid-body
  plants and physical effectors.

The ongoing release criteria are tracked in the
[Product 2 maturity plan](plan/product-two-maturity.md). The broader
architecture and timing contracts remain the authoritative references for
[interactive stepping](architecture/interactive-engine.md),
[accepted truth](architecture/eom-timing-contract.md), and
[vehicle realizations](architecture/vehicle-realizations.md).
