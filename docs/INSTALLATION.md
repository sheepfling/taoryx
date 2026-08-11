# Installing Taoryx and its model packages

Taoryx is distributed as a small core host plus four official plug-in
distributions. Choose the smallest profile that contains the work you need;
contributors and automation agents should use the full profile.

`taoryx-cadac` is a separately installable, source-bound CADAC integration.
It is deliberately outside the default model and full profiles because it does
not redistribute the upstream CADAC data. Use the dedicated `cadac` profile
after obtaining an authorized local CADAC checkout.

Python 3.12 or newer is required. Use `python -m pip` so the installer and the
Python interpreter always refer to the same environment.

## Package map

```text
taoryx (language, compiler, engine, registries, common CLI)
├── taoryx-daveml
├── taoryx-simple-aero
└── taoryx-reference-models
    ├── requires taoryx-daveml
    ├── requires taoryx-simple-aero
    └── taoryx-reachability requires taoryx-reference-models

taoryx-cadac (optional source-bound CADAC catalog and converter)
```

| Distribution | Install it when you need | Direct Taoryx dependencies |
| --- | --- | --- |
| `taoryx` | `.tbl` / `.prb` language tools, the simulation host, common contracts, registries, and CLI | none |
| `taoryx-daveml` | DAVE-ML import, semantic evaluation, replay, and model-format registration | `taoryx` |
| `taoryx-simple-aero` | analytical Simple Aero models, fixtures, and the point-mass provider | `taoryx` |
| `taoryx-reference-models` | X-15, HL-20, NESC, passive body, X8, B747, A320, F-16, and Hummingbird models plus registered tuning inputs | `taoryx`, `taoryx-daveml`, `taoryx-simple-aero` |
| `taoryx-reachability` | reachability envelopes, continuation, plots, and reachability mission overlays | `taoryx`, `taoryx-reference-models` |
| `taoryx-cadac` | CADAC actor catalog, compatibility runtimes, and canonical CADAC table conversion | `taoryx` |

Installing `taoryx-reference-models` from a release index or wheelhouse pulls
the DAVE-ML and Simple Aero distributions. Installing `taoryx-reachability`
pulls the complete five-distribution suite. The corresponding local source
commands name every path explicitly so pip never tries to find an unpublished
sibling distribution on a package index.

The reference-model wheel also declares NumPy and SciPy because its executable
family adapters and registered controller-tuning campaigns use the common
numerical host. Those dependencies remain outside the small core-only profile.

## Fast path for contributors and agents

From the repository root:

```bash
python -m tools.dev bootstrap
source .venv/bin/activate
python -m tools.dev install-check
python -m tools.dev doctor
```

On Windows PowerShell, activate with `.venv\Scripts\Activate.ps1` instead.

`bootstrap` creates or reuses `.venv`, installs `taoryx[dev]`, installs all
four local plug-in distributions in editable mode, and runs the strict full
installation check. The explicit second check is useful in agent logs and
after changing package metadata.

Do not treat a passing source-tree test as proof that the distributions are
installed. The repository test configuration adds sibling `src` directories
to `PYTHONPATH`; `install-check` deliberately ignores that source fallback and
requires real distribution metadata plus all four `taoryx.plugins` entry
points.

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
  -e packages/taoryx-simple-aero \
  -e packages/taoryx-reference-models
taoryx plugins check --profile models
```

The complete official suite:

```bash
python -m pip install \
  -e . \
  -e packages/taoryx-daveml \
  -e packages/taoryx-simple-aero \
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

# Core plus every model package, without reachability
python -m pip install --find-links /path/to/taoryx-wheels taoryx-reference-models==0.1.0a0

# Complete official suite
python -m pip install --find-links /path/to/taoryx-wheels taoryx-reachability==0.1.0a0
```

Keep the five Taoryx distributions on compatible versions. The current alpha
plug-ins require `taoryx>=0.1.0a0,<0.2`; pip should resolve the dependency
graph rather than installing plug-in wheels with `--no-deps`.

## Optional third-party feature dependencies

These are root-package extras, not additional Taoryx model distributions:

| Extra | Command | Adds |
| --- | --- | --- |
| Development | `python -m pip install -e '.[dev]'` | build, plotting, numerical, PDF, test, lint, and type-check tools |
| Sensors | `python -m pip install -e '.[sensors]'` | `imu-error-model==0.1.3` and NumPy |
| SciPy integration | `python -m pip install -e '.[scipy]'` | SciPy for custom model plug-ins or direct use of trim/LQR APIs; `taoryx-reference-models` already requires it |

For a fresh full contributor environment that also exercises the optional
sensor dependency, use:

```bash
python scripts/bootstrap.py --with-sensors
```

## Verify what is actually installed

The profile check loads installed entry points only and does not construct a
vehicle plant:

```bash
taoryx plugins check --profile full
taoryx plugins check --profile full --json
```

Inspect the complete typed advertisement catalog when diagnosing a model,
control, metadata, or execution contribution:

```bash
taoryx plugins list --no-builtin
taoryx plugins inspect taoryx.reference-models --no-builtin
taoryx model list --provider taoryx.registry.mission-composition
taoryx model plan taoryx.registry.mission-composition skywalker_x8 \
  --fidelity point_mass_3dof --realization point_mass_3dof \
  --mission powered_fixed_wing_racetrack_v1
```

After the model profile passes, verify the model-to-mission authoring surface:

```bash
taoryx model list
taoryx model plan \
  taoryx.registry.mission-composition \
  hummingbird \
  --fidelity pseudo_6dof
```

`model list` should include 11 production-registry models and two analytical
reference-provider models in the current official suite. The A320 pseudo-6DOF
entry, Hummingbird pseudo-6DOF entry, and Hummingbird source-hover rotor screen
also advertise registered tuning campaign IDs.
See [Model-to-mission authoring and automation](architecture/model-authoring-automation.md)
for scaffold, compile, and tune commands.

`--no-builtin` is important in a checkout: it disables the convenience source
fallback and shows only installed Python entry points. A full profile is ready
only when the check reports all five distributions and these four plug-in IDs:

```text
taoryx.daveml
taoryx.simple-aero
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
