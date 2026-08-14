# taoryx-reachability

Optional reachability workbench for Taoryx. This distribution layers launch
grid studies, terminal criteria, timeout continuation, plotting, X-15 and
HL-20 reachability workflows, passive-release propagation, and their composed
mission adapters over the core host and the X-15 and HL-20 family packages.

The package publishes `taoryx.reachability` through the standard
`taoryx.plugins` entry-point group. The core language and engine do not depend
on this wheel. Reference models remain installable without it; installing this
wheel adds the reachability provider plus the reach-specific capability,
semantic-preflight, and execution contributions.

Discovery publishes stable overlay identities without importing the X-15/HL-20
source planners, preflights, or executors. Those implementations load only
when the selected mission uses the corresponding adapter, preflight, or
execution factory.

Its staged X-15 route and HL-20 booster-release/glide-energy routes are typed
additive catalog overlays, not vehicle-family claims. Each has a separate
packaged resource root, so selecting X-15 never parses HL-20 overlay data (or
vice versa). A host must select `taoryx.x15` plus `taoryx.reachability` for the
staged X-15 initialization, segments, mission, batch bindings, and witnesses;
it must select `taoryx.hl20` plus `taoryx.reachability` for the corresponding
HL-20 rows and requests. Selecting reachability by itself fails closed instead
of advertising a vehicle family it does not own.

Implementation modules continue to occupy the shared `taoryx` namespace so
established imports work when this optional distribution is installed.

## Install from this checkout

Run from the repository root and supply this overlay's direct local dependency
chain:

```bash
python -m pip install \
  -e . \
  -e packages/taoryx-daveml \
  -e packages/taoryx-x15 \
  -e packages/taoryx-hl20 \
  -e packages/taoryx-reachability
taoryx plugins inspect taoryx.reachability --no-builtin
```

## Focused package boundary

Use the package-owned gate while changing this overlay. It checks the exact
packaged data, direct dependencies, deferred registrations, and an
installed-wheel boundary without executing X-15 or HL-20 studies:

```bash
python tools/dev.py test-reachability
python tools/dev.py check-reachability
```

See the [installation guide](../../docs/INSTALLATION.md) for release wheels,
minimal profiles, optional dependencies, and contributor setup.
