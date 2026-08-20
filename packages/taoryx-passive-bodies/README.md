# Taoryx passive bodies

`taoryx-passive-bodies` owns reusable passive released-body models: tanks,
canisters, spent stages, cones, fairings, and debris-like bodies.  It exposes
the standard `tumbling_body` direct-release witnesses and a typed child-runtime
seam that parent vehicle plug-ins can bind at composition time.

The package deliberately does not import a parent vehicle plug-in.  A parent
supplies an accepted release state through the core deployment contract; this
plug-in independently propagates the selected child and returns its own
telemetry and provenance-bounded result.

The shipped examples are development witnesses, not source-specific
spent-stage aerodynamics or separation qualification.

## Versioned focused runtime

The package/provider release is `0.1.0a0`; the runnable `tumbling_body`
model advertises `1.0.0+composition-v1`. A UI, agent, or cache can compare
`catalog.plugin_revision("taoryx.passive-bodies")` and the model/provider
schema fingerprints to refresh this family without rediscovering sibling
vehicles.

The focused provider retains the exact catalog that created it through normal
batch execution, preflight, lowering, and witness validation. The two public
tiers advertise `no_external_action`: there is no controller, control surface,
thrust, wrench, native live plant, or batch/episode parity claim. Each exact
registered batch path is nevertheless available through the standard
read-only core replay session, with an empty action schema and committed
direct-release truth—position, velocity, drag, projected area, angular rate,
phase, and fixed mass.

Use the focused provider to inspect or execute a direct-release model:

```bash
taoryx model list --provider taoryx.passive-bodies.mission-composition
taoryx model plan taoryx.passive-bodies.mission-composition tumbling_body \
  --fidelity pseudo_6dof --realization pseudo_6dof \
  --mission tumbling_body_release_damping_impact_v1
```

The separate `taoryx.passive-bodies.local-atmosphere-release.v1` runtime is a
typed child-runtime seam. A parent plug-in must explicitly advertise and bind
the child at composition time; the passive package never discovers or imports
the parent. The current NESC synthetic-cylinder integration is therefore a
two-package proof, not a hidden dependency or generic spent-stage claim.

Discovery is intentionally identity-only: it advertises the passive capability,
semantic-preflight translator, direct-release interface factory, and child
runtime ID without importing their planners, truth interface, propagator, or
numerical family-adapter stack. Those implementations load only when the
selected composition requests the corresponding capability, interface,
preflight, family adapter, or explicit parent-child binding. This keeps
passive-body discovery independent of both a parent vehicle and the optional
reachability overlay.

## Focused verification

```bash
python tools/dev.py check-vehicle tumbling_body
python tools/verify_plugin_wheels.py --plugin passive-bodies
python tools/verify_plugin_wheels.py --plugin cross-plugin-deployment
```

The direct-release gate begins by verifying the exact passive-body package-data
extract:

```bash
python tools/extract_passive_bodies_plugin_assets.py --check
```

The first command checks the two no-action direct-release interfaces, their
two endpoint witnesses, and the short focused provider/batch/replay-session
test. It does not claim native control parity. The last command installs
only the selected NESC, DAVE-ML, and passive-body wheels and proves the
explicit parent/child binding from release state through child telemetry.
