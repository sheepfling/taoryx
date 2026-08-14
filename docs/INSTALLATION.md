# Installing Taoryx and its model packages

Taoryx is distributed as a small core host plus thirteen direct official
plug-in distributions and one compatibility aggregate. Choose the smallest
profile that contains the work you need; contributors and automation agents
should use the direct `developer` profile by default.

`taoryx-cadac` is a separately installable, source-bound CADAC integration.
It is deliberately outside the default model and full profiles because it does
not redistribute the upstream CADAC data. Use the dedicated `cadac` profile
after obtaining an authorized local CADAC checkout.

Python 3.12 or newer is required. Use `python -m pip` so the installer and the
Python interpreter always refer to the same environment.

`developer` installs every direct family/model-format/overlay plug-in but does
not install `taoryx-reference-models`. `compatibility` installs that aggregate
and its declared transitive plug-in dependencies for callers that still require
`taoryx.registry.mission-composition`; it intentionally excludes the debug,
passive-body, and reachability plug-ins. `full` contains both broad routes for
migration and release integration.

## Package map

```text
taoryx (language, compiler, engine, registries, common CLI)
├── taoryx-daveml
├── taoryx-debug-models
├── taoryx-a320
├── taoryx-f16 requires taoryx-daveml
├── taoryx-hummingbird
├── taoryx-nesc requires taoryx-daveml
├── taoryx-passive-bodies
├── taoryx-simple-aero
├── taoryx-dual-launch
├── taoryx-x15
├── taoryx-hl20 requires taoryx-daveml
├── taoryx-source-table-fixed-wing
├── taoryx-reference-models
│   ├── compatibility aggregate requires taoryx-daveml
│   ├── compatibility aggregate requires taoryx-a320
│   ├── compatibility aggregate requires taoryx-f16
│   ├── compatibility aggregate requires taoryx-hummingbird
│   ├── compatibility aggregate requires taoryx-nesc
│   ├── compatibility aggregate requires taoryx-simple-aero
│   ├── compatibility aggregate requires taoryx-dual-launch
│   ├── compatibility aggregate requires taoryx-x15
│   ├── compatibility aggregate requires taoryx-hl20
│   └── compatibility aggregate requires taoryx-source-table-fixed-wing
└── taoryx-reachability requires taoryx-x15 and taoryx-hl20

taoryx-cadac (optional source-bound CADAC catalog and converter)
```

| Distribution | Install it when you need | Direct Taoryx dependencies |
| --- | --- | --- |
| `taoryx` | `.tbl` / `.prb` language tools, the simulation host, common contracts, registries, and CLI | none |
| `taoryx-daveml` | DAVE-ML import, semantic evaluation, replay, and model-format registration | `taoryx` |
| `taoryx-debug-models` | development-only analytical and contract-probe Mission Composition providers | `taoryx` |
| `taoryx-a320` | OpenAP point-mass and explicitly surrogate pseudo-6DOF A320 products, focused composition provider, local named-coordinate LQI screen, and A320-only source/evidence resources | `taoryx` |
| `taoryx-f16` | F-16 S.119 source assets, reduced guidance controls, bounded local physical-control screens, focused Composition provider, and source-owned evidence | `taoryx`, `taoryx-daveml` |
| `taoryx-hummingbird` | AscTec Hummingbird source assets, fidelity/control metadata, composition provider, rotor screens, and tuning campaigns | `taoryx` |
| `taoryx-nesc` | NASA/NESC Scenario 17 source-replay assets, adapters, focused composition provider, and stage-separation parent contract | `taoryx`, `taoryx-daveml` |
| `taoryx-passive-bodies` | reusable tumbling/released bodies, direct-release witnesses, and composition-selected child propagation | `taoryx` |
| `taoryx-simple-aero` | analytical Simple Aero models, fixtures, and the point-mass provider | `taoryx` |
| `taoryx-dual-launch` | synthetic dual-launch glider workflow, source-problem lowering, focused batch provider, and package-owned endpoint witness | `taoryx` |
| `taoryx-x15` | X-15 source tables, local direct-wrench and source-surface controller screens, focused Composition provider, campaigns, and family-owned evidence | `taoryx` |
| `taoryx-hl20` | HL-20 Mod K source fixture, local direct-wrench and seven-surface controller screens, focused Composition provider, campaigns, and family-owned evidence | `taoryx`, `taoryx-daveml` |
| `taoryx-source-table-fixed-wing` | Skywalker X8 and B747 source-table plants, focused Composition providers, route assets, local controller screens, campaigns, and witnesses | `taoryx` |
| `taoryx-reference-models` | compatibility aggregate Mission Composition provider | `taoryx`, `taoryx-daveml`, `taoryx-a320`, `taoryx-f16`, `taoryx-hummingbird`, `taoryx-nesc`, `taoryx-simple-aero`, `taoryx-dual-launch`, `taoryx-x15`, `taoryx-hl20`, `taoryx-source-table-fixed-wing` |
| `taoryx-reachability` | reachability envelopes, continuation, plots, and reachability mission overlays | `taoryx`, `taoryx-x15`, `taoryx-hl20` |
| `taoryx-cadac` | CADAC actor catalog, compatibility runtimes, and canonical CADAC table conversion | `taoryx` |

Installing `taoryx-reference-models` from a release index or wheelhouse pulls
the source-table fixed-wing, DAVE-ML, A320, F-16, Hummingbird, NESC, Simple
Aero, Dual Launch, X-15, and HL-20 distributions needed by its compatibility aggregate.
Passive bodies remain independently optional: a
composition selects that child plug-in only when it needs a released-body
trajectory. The independent debug-provider distribution is part of the
`models`, `developer`, and `full` profiles. The `compatibility` profile mirrors
only the aggregate's declared dependency closure, so it excludes both optional
packages. Installing `taoryx-reachability` pulls only its
direct X-15 and HL-20 overlay dependencies (and HL-20's DAVE-ML dependency); it
does not install the compatibility aggregate. The corresponding local source
commands name every path explicitly so pip never tries to find an unpublished
sibling distribution on a package index.

The source-table fixed-wing wheel declares NumPy and SciPy because its plants
and local controller campaigns use the common numerical host. Those
dependencies remain outside the small core-only profile.

## Fast path for contributors and agents

From the repository root:

```bash
python -m tools.dev bootstrap
source .venv/bin/activate
python -m tools.dev install-check
python -m tools.dev doctor
```

On Windows PowerShell, activate with `.venv\Scripts\Activate.ps1` instead.

`bootstrap` creates or reuses `.venv`, installs `taoryx[dev]`, installs the
thirteen direct local plug-in distributions in editable mode, and runs the
strict `developer` installation check. The explicit second check is useful in
agent logs and after changing package metadata. Use `--profile full` only when
you are intentionally validating compatibility consumers.

Do not treat a passing source-tree test as proof that the distributions are
installed. The repository test configuration adds sibling `src` directories
to `PYTHONPATH`; `install-check` deliberately ignores that source fallback and
requires real distribution metadata plus all thirteen direct `taoryx.plugins`
entry points.

## Install from a source checkout

Create and activate a virtual environment first:

```bash
python -m venv .venv
source .venv/bin/activate
```

Core language and engine only:

```bash
python -m pip install -e .
taoryx plugins check --profile core
```

All model and model-format packages, without reachability:

```bash
python -m pip install \
  -e . \
  -e packages/taoryx-daveml \
  -e packages/taoryx-debug-models \
  -e packages/taoryx-a320 \
  -e packages/taoryx-f16 \
  -e packages/taoryx-hummingbird \
  -e packages/taoryx-nesc \
  -e packages/taoryx-passive-bodies \
  -e packages/taoryx-simple-aero \
  -e packages/taoryx-dual-launch \
  -e packages/taoryx-x15 \
  -e packages/taoryx-hl20 \
  -e packages/taoryx-source-table-fixed-wing
taoryx plugins check --profile models
```

The direct developer profile adds the reachability overlay but still excludes
the aggregate:

```bash
python scripts/bootstrap.py --profile developer
taoryx plugins check --profile developer
python tools/dev.py check-developer-plugins
```

The minimal aggregate compatibility route installs only the packages it
declares as dependencies; it does not install debug models, passive bodies, or
reachability:

```bash
python scripts/bootstrap.py --profile compatibility
taoryx plugins check --profile compatibility
```

The complete official suite:

```bash
python -m pip install \
  -e . \
  -e packages/taoryx-daveml \
  -e packages/taoryx-debug-models \
  -e packages/taoryx-a320 \
  -e packages/taoryx-f16 \
  -e packages/taoryx-hummingbird \
  -e packages/taoryx-nesc \
  -e packages/taoryx-passive-bodies \
  -e packages/taoryx-simple-aero \
  -e packages/taoryx-dual-launch \
  -e packages/taoryx-x15 \
  -e packages/taoryx-hl20 \
  -e packages/taoryx-source-table-fixed-wing \
  -e packages/taoryx-reference-models \
  -e packages/taoryx-reachability
taoryx plugins check --profile full
```

Add `-e '.[dev]'` instead of `-e .` to either multi-package command when the
environment must run repository lint, type, test, manual, or release gates.

## Install release wheels

Use this form only when your configured package index or release wheelhouse
contains the matching Taoryx wheels. A wheelhouse path may be local or an
internal release location:

```bash
# Core only
python -m pip install --find-links /path/to/taoryx-wheels taoryx==0.1.0a0

# Compatibility aggregate and its declared plug-in dependencies
python -m pip install --find-links /path/to/taoryx-wheels taoryx-reference-models==0.1.0a0

# Reachability overlay and its direct vehicle dependencies
python -m pip install --find-links /path/to/taoryx-wheels taoryx-reachability==0.1.0a0

# Complete official suite
python -m pip install --find-links /path/to/taoryx-wheels \
  taoryx-reference-models==0.1.0a0 \
  taoryx-debug-models==0.1.0a0 \
  taoryx-passive-bodies==0.1.0a0 \
  taoryx-reachability==0.1.0a0
```

Keep the fifteen Taoryx distributions on compatible versions. The current alpha
plug-ins require `taoryx>=0.1.0a0,<0.2`; pip should resolve the dependency
graph rather than installing plug-in wheels with `--no-deps`.

## Optional third-party feature dependencies

These are root-package extras, not additional Taoryx model distributions:

| Extra | Command | Adds |
| --- | --- | --- |
| Development | `python -m pip install -e '.[dev]'` | build, plotting, numerical, PDF, test, lint, and type-check tools |
| Sensors | `python -m pip install -e '.[sensors]'` | `imu-error-model==0.1.3` and NumPy |
| SciPy integration | `python -m pip install -e '.[scipy]'` | SciPy for custom model plug-ins or direct use of trim/LQR APIs; `taoryx-source-table-fixed-wing` already requires it |

For a fresh direct developer environment that also exercises the optional
sensor dependency, use:

```bash
python scripts/bootstrap.py --with-sensors
```

## Verify what is actually installed

The profile check loads installed entry points only and does not construct a
vehicle plant. It verifies the installed distribution, the declared entry
point's ID/distribution/version, the loaded plug-in metadata, and staged
registration as separate checks:

```bash
taoryx plugins check --profile developer
taoryx plugins check --profile developer --json

# Compatibility/release integration only.
taoryx plugins check --profile full
```

To inspect declarations without loading a plug-in target, use:

```bash
# Installed distribution metadata only.
taoryx plugins entry-points --no-builtin --json

# Source-checkout declarations only.
taoryx plugins entry-points --no-external --json
```

Inspect the complete typed advertisement catalog when diagnosing a model,
control, metadata, or execution contribution:

```bash
taoryx plugins list --no-builtin
taoryx plugins inspect taoryx.source-table-fixed-wing --no-builtin
taoryx plugins inspect taoryx.reference-models --no-builtin
taoryx plugins inspect taoryx.a320 --no-builtin
taoryx plugins inspect taoryx.f16 --no-builtin
taoryx plugins inspect taoryx.hl20 --no-builtin
taoryx plugins inspect taoryx.hummingbird --no-builtin
taoryx plugins inspect taoryx.nesc --no-builtin
taoryx model list --provider taoryx.hummingbird.mission-composition
taoryx model list --provider taoryx.nesc.mission-composition
taoryx model list --provider taoryx.f16.mission-composition
taoryx model list --provider taoryx.a320.mission-composition
taoryx model list --provider taoryx.hl20.mission-composition
taoryx model list --provider taoryx.x8.mission-composition
taoryx model list --provider taoryx.b747.mission-composition

# Only after installing the compatibility or full profile.
taoryx model list --provider taoryx.registry.mission-composition
taoryx model plan taoryx.x8.mission-composition skywalker_x8 \
  --fidelity point_mass_3dof --realization point_mass_3dof \
  --mission powered_fixed_wing_racetrack_v1
```

After the model profile passes, verify the model-to-mission authoring surface:

```bash
taoryx model list
taoryx model plan \
  taoryx.hummingbird.mission-composition \
  hummingbird \
  --fidelity pseudo_6dof
```

`model list` exposes focused A320, F-16, HL-20, Hummingbird, NESC
source-replay, X8, B747, and `tumbling_body` passive-body providers in the
direct developer profile. The compatibility aggregate appears only after its
explicit profile is installed. The A320 pseudo-6DOF entry, Hummingbird
pseudo-6DOF entry, X8/B747 lower-tier guidance, and Hummingbird source-hover
rotor screen also advertise registered tuning campaign IDs.
See [Model-to-mission authoring and automation](architecture/model-authoring-automation.md)
for scaffold, compile, and tune commands.

`--no-builtin` is important in a checkout: it disables the convenience source
fallback and shows only installed Python entry points. A full profile is ready
only when the check reports all fifteen distributions and these fourteen plug-in IDs:

```text
taoryx.daveml
taoryx.debug-models
taoryx.a320
taoryx.f16
taoryx.hl20
taoryx.hummingbird
taoryx.nesc
taoryx.passive-bodies
taoryx.simple-aero
taoryx.dual-launch
taoryx.x15
taoryx.source-table-fixed-wing
taoryx.reference-models
taoryx.reachability
```

The CADAC plug-in remains an explicit source-bound installation:

```bash
python -m pip install -e . -e packages/taoryx-cadac
taoryx plugins check --profile cadac
taoryx-cadac convert-tree /path/to/CADAC --output build/cadac-tables
```

The converter retains source hashes but must not be used to commit or publish
upstream coefficient data without a separate redistribution review.

Profiles express minimum required packages; they do not uninstall additional
plug-ins already present in an environment. Use a fresh virtual environment
when proving the strict core-only or no-reachability boundary.

If a distribution is present but its plug-in fails to load, inspect the
diagnostic from `taoryx plugins list --no-builtin --json`. If pip cannot resolve
a local sibling package, rerun the matching source-checkout command above with
all prerequisite paths in the same invocation. An offline bootstrap may fall
back to `--no-deps`; `install-check` and `doctor` will then expose any dependency
that still needs to be supplied.
