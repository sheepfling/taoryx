# TAORYX NESC plug-in

`taoryx-nesc` owns the NASA/NESC Scenario 17 two-stage source-replay family:
its source/evidence fragment, replay and pseudo-6DOF adapters, capability and
preflight contract, batch runtime, and focused Mission Composition provider.

The package deliberately does not claim a participating rocket plant,
guidance, or gimbal allocation. Its optional stage-separation child uses the
separately installed `taoryx-passive-bodies` runtime through an explicit
composition binding; it is not a package dependency.

For the source-free package boundary, run:

```bash
python tools/dev.py check-vehicle reference_nesc_two_stage_rocket
python tools/verify_plugin_wheels.py --plugin nesc
python tools/verify_plugin_wheels.py --plugin cross-plugin-deployment
```

The normal gate begins by verifying the exact NESC package-data extract:

```bash
python tools/extract_nesc_plugin_assets.py --check
```

It retains the named passive-child composition as deployment metadata but does
not install or execute the child runtime; that remains the separate
two-package wheel proof.

The normal NESC gate selects only `taoryx.nesc`, validates the point-mass and
pseudo-6DOF interfaces, executes its two batch-only replay witnesses, and
runs its focused advertisement/compiler/common-batch proof. It deliberately
does not open a session or replay batch/episode parity because the source
replay advertises neither. The passive-child composition remains the separate
two-package wheel test above, so a NESC-only change does not pull that child
runtime or unrelated vehicle tests into its inner loop.

For UI, agent, or cache refresh, the standard discovery surfaces publish both
the package-managed version and a scoped revision fingerprint; the NESC model
also exposes its own version and metadata fingerprint:

```bash
taoryx plugins inspect taoryx.nesc --json
taoryx model list --provider taoryx.nesc.mission-composition
```

Focused hosts should pass their selected plug-in catalog into composition
compilation as well as batch execution. The provider retains that same catalog
through the common batch API, which prevents an NESC authoring or execution
request from falling back to environment-wide plug-in discovery.
