# taoryx-reachability

Optional reachability workbench for Taoryx. This distribution layers launch
grid studies, terminal criteria, timeout continuation, plotting, X-15 and
HL-20 reachability workflows, passive-release propagation, and their composed
mission adapters over the core host and reference-model packages.

The package publishes `taoryx.reachability` through the standard
`taoryx.plugins` entry-point group. The core language and engine do not depend
on this wheel. Reference models remain installable without it; installing this
wheel adds the reachability provider plus the reach-specific capability,
semantic-preflight, and execution contributions.

Implementation modules continue to occupy the shared `taoryx` namespace so
established imports work when this optional distribution is installed.

## Install from this checkout

Run from the repository root and supply the complete local dependency chain:

```bash
python -m pip install \
  -e . \
  -e packages/taoryx-daveml \
  -e packages/taoryx-simple-aero \
  -e packages/taoryx-reference-models \
  -e packages/taoryx-reachability
taoryx plugins check --profile full
```

See the [installation guide](../../docs/INSTALLATION.md) for release wheels,
minimal profiles, optional dependencies, and contributor setup.
