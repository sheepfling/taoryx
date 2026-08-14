# taoryx-dual-launch

`taoryx-dual-launch` owns the synthetic Alpha 2 dual-launch glider workflow:
its versioned Mission Composition provider, launch-form configuration grammar,
source-problem lowering, normalized batch result projection, endpoint witness,
and package-owned metadata.

The focused provider is `taoryx.dual-launch.mission-composition`. It exposes
only the executable source-generated point-mass batch route. Both air-release
and attached-booster forms share a post-release waypoint mission; separation is
reported as an event on the primary trajectory, not as an independently
propagated child vehicle. Pseudo-6DOF, rigid-body, and interactive modes are
advertised as unavailable rather than inferred.

The historical `taoryx.registry.mission-composition` aggregate can retain this
model as a compatibility view when its full profile is installed. New UI,
agent, and composition callers should select this focused provider.

From the repository root:

```bash
python -m pip install -e . -e packages/taoryx-dual-launch
python tools/dev.py test-vehicle dual_launch_glider
python tools/verify_plugin_wheels.py --plugin dual-launch --python .venv/bin/python
```

The focused command begins by verifying its exact package-data extract:

```bash
python tools/extract_dual_launch_plugin_assets.py --check
```

That extract contains only the Dual Launch family, maturity, endpoint, and
witness fragments; it is deliberately independent of Simple Aero.
