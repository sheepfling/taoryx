# taoryx-x15

The source-backed X-15 family plug-in for Taoryx. It owns the X-15 source
tables, direct-wrench and source-surface local controller screens, controller
campaigns, focused Mission Composition provider, and package-local catalog
fragments.

The direct-wrench screens advertise six bounded body-force/moment coordinates
and explicitly do not claim physical stabilator, rudder, RCS, propulsion, or
navigation allocation. The source-surface screens retain their three named
source coordinates and similarly remain local, frozen-fixture evidence.

Focused discovery retains the direct-wrench screens' identities, six-axis
bounds, cadence, and claim boundaries without importing NumPy, SciPy, source
tables, or the numerical direct-wrench runtime. The package resolves those
implementations only after the matching local screen is selected.

The optional `taoryx-reachability` plug-in depends on this package for the
X-15 source assets used by its separate staged reachability overlay. It is not
required to discover or execute the local X-15 screens.

The X-15 catalog fragment owns only the local-screen composition rows,
bindings, and witnesses. The staged booster/release composition request and
its runnable metadata live in the reachability wheel as a typed catalog
overlay, which is resolved only when a host selects both `taoryx.x15` and
`taoryx.reachability`. Selecting X-15 alone therefore cannot advertise an
endpoint whose reachability capability or execution factory is absent.

```bash
python -m pip install -e . -e packages/taoryx-x15
taoryx model list --provider taoryx.x15.mission-composition
taoryx model plan taoryx.x15.mission-composition x15 \
  --fidelity rigid_body_6dof_direct_wrench \
  --mission x15_local_direct_wrench_screen_v1
python tools/dev.py check-vehicle x15
```

The focused gate begins by confirming that package data is an exact fresh
extract of its canonical X-15 inputs:

```bash
python tools/extract_x15_plugin_assets.py --check
```

The retained source/evidence bundle includes the source assets intentionally
consumed by the optional reachability overlay. It does not carry the overlay's
staged composition requests, execution bindings, or witnesses, and it does
not make reachability a runtime dependency of the local X-15 provider.

The normal X-15 plug-in gate covers only the versioned focused provider, its
local direct-wrench batch/episode witness pair, their parity trace, and the
selected-catalog batch/session contract. It deliberately excludes the
reachability-owned staged overlay and the separate source-surface controller
screens, which retain their own evidence lanes.
