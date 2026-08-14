# taoryx-hl20

The source-backed HL-20 Mod K family plug-in for Taoryx. It owns the
DAVE-ML source fixture, seven-surface local authority and LQI screens,
bounded direct-wrench screens, controller campaigns, focused Mission
Composition provider, trim-evidence binding, and HL-20 catalog fragments.

The direct-wrench screens advertise six bounded body-force/moment coordinates
and do not claim physical surface allocation. The source-surface screens use
all seven named source surfaces but remain fixed-fixture local evidence rather
than a full glide, navigation, or flight-qualification claim.

The base catalog owns only the local-screen initialization, segment, mission,
binding, and witness rows. `taoryx-reachability` depends on this package for
the separate HL-20 booster-release and glide-energy overlays; those rows,
composition requests, and source-release witnesses appear through generic
catalog discovery only when both `taoryx.hl20` and `taoryx.reachability` are
selected. It is not required to discover or execute the local HL-20 screens.

```bash
python -m pip install -e . -e packages/taoryx-daveml -e packages/taoryx-hl20
taoryx model list --provider taoryx.hl20.mission-composition
taoryx model plan taoryx.hl20.mission-composition hl20_mod_k \
  --fidelity rigid_body_6dof_direct_wrench \
  --mission hl20_local_direct_wrench_screen_v1
python tools/dev.py check-vehicle hl20_mod_k
```

The focused gate begins by confirming that package data is an exact fresh
extract of its canonical HL-20 inputs:

```bash
python tools/extract_hl20_plugin_assets.py --check
```

The retained source/evidence bundle includes the HL-20 fixture intentionally
consumed by the optional reachability overlays. The source fixture remains
with this family package; the optional catalog rows live in the reachability
wheel's separate HL-20 overlay resource root. Neither arrangement makes
reachability a runtime dependency of the local HL-20 provider.

The normal HL-20 plug-in gate covers the versioned focused provider, one local
direct-wrench batch/episode witness pair, their parity trace, and the
selected-catalog batch/session contract. It deliberately excludes the
reachability-owned source-release overlays and the separate seven-surface
controller screens, which retain their own evidence lanes.

Its installed-wheel smoke verifies that the package owns and compiles the
primary local composition without source fallback. It intentionally leaves the
numerical batch execution to the focused gate above, so packaging work does not
repeat the local controller evidence.
