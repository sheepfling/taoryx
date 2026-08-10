# taoryx-reference-models

Optional source-backed and analytical vehicle models for Taoryx. This bundle
owns the currently executable X-15, HL-20, NESC, passive tumbling-body,
Skywalker X8, B747, A320, F-16, and Hummingbird implementations and their
family-adapter registrations.

It also owns non-reachability family mission-capability planners, semantic
preflight handlers, batch execution factories, batch/episode parity verifiers,
registered controller-tuning campaigns, and the registry-backed Mission
Composition provider/session implementation.
Model assets and source evidence are package resources rather than core-wheel
data.

Reachability envelopes, reach-specific mission translations, and their
execution/preflight overlays live in the separate `taoryx-reachability` wheel.
That wheel depends on this one; this distribution does not depend on it and is
fully discoverable without it.

The implementation occupies the shared `taoryx` namespace so established
imports remain valid while the core Taoryx wheel contains only language,
engine, contracts, registries, and generic host services.

## Install from this checkout

Run from the repository root and supply both local format dependencies:

```bash
python -m pip install \
  -e . \
  -e packages/taoryx-daveml \
  -e packages/taoryx-simple-aero \
  -e packages/taoryx-reference-models
taoryx plugins check --profile models
taoryx model list --provider taoryx.registry.mission-composition
```

NumPy and SciPy are direct dependencies of this distribution because its
executable family adapters and the common trim/linearization/tuning path use
them. The core-only `taoryx` wheel does not acquire those model dependencies.

After installation, use `taoryx model plan` and `taoryx model scaffold` for
every advertised model. The A320 and Hummingbird pseudo-6DOF realizations also
publish campaigns executable with `taoryx model tune`; their results remain
local candidate-design screens rather than vehicle qualification.

See the [installation guide](../../docs/INSTALLATION.md) for release wheels,
reachability, and contributor setup.
