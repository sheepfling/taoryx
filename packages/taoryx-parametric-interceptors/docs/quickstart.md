# Parametric interceptor quickstart

Use this page when the goal is to turn one sparse interceptor record into a
truthful, runnable TAORYX provider without first learning the whole package.
The [package README](../README.md) is the complete authoring reference; the
[model architecture guide](model-architecture.md) explains the evidence,
runtime, sensor, control, and ECEF details behind this short path.

## Choose the right route

| You want to… | Use this route |
| --- | --- |
| Run an app-local study with one new profile | `taoryx-interceptor` file-first path below |
| Preserve ontology/source evidence | Start the same path with `init --format catalogue` |
| Ship a maintained model in this distribution | Add a witnessed profile and use the package verification path |
| Implement a non-TAORYX solver for generic hosts | Use the standalone [trajectory contracts](../../../docs/api/trajectory-contracts.md), not this internal plug-in path |

## Run an app-local profile

The app-local path is the normal first step. It creates an isolated provider;
it does not modify global plug-in discovery or make the profile available to
unrelated TAORYX processes.

```bash
# Create a valid, editable profile. Refuses to overwrite by default.
taoryx-interceptor init my-sam.yaml --interceptor-id my-sam

# Resolve evidence and inspect runnable tiers, controls, diagnostics, and the
# common standard-ECEF output contract before executing anything.
taoryx-interceptor inspect my-sam.yaml --output my-sam-report.json

# Run a deliberately small first witness through the ordinary Mission
# Composition runner.
taoryx-interceptor run my-sam.yaml \
  --set runtime.duration_s=0.2 \
  --set runtime.time_step_s=0.05 \
  --output my-sam-run.json
```

Use `--format catalogue` with `init` when evidence must retain observed,
reported, derived, inferred, unavailable, and interval records. The inspection
report tells you whether the result is executable or only a resolver/provenance
witness. Do not use `--allow-unqualified` unless the intentionally unqualified
surrogate boundary is appropriate for the study.

The same provider is available to Python without registration glue:

```python
from taoryx_parametric_interceptors import ParametricInterceptorMissionCompositionProvider

provider = ParametricInterceptorMissionCompositionProvider.from_yaml("my-sam.yaml")
model = provider.list_models()[0]
configuration = provider.configuration(model.id)
prepared = provider.validate_configuration(configuration)
```

## Promote a profile into the installed package

Only take this route when the model should ship as a maintained, discoverable
plug-in model. Keep the source/evidence seed and claim boundary in
`witnesses.py`, add it to `runnable_prototype_profiles()` only after it is
executable, and add the package-focused vertical test. The model must remain a
normal TAORYX universal model: batchable, steppable, standard-ECEF capable, and
able to build a provider-selected runnable default.

```bash
# See only this distribution's discovery and verification loop.
python tools/dev.py plugin-focus parametric-interceptors

# Fast source-tree gate: direct package, universal batch/step surface,
# preparable default, and standard ECEF contract.
python tools/dev.py check-plugin-contract parametric-interceptors

# Package-owned discovery-to-execution witness.
python tools/dev.py test-parametric-interceptors

# Inspect the built-in default a generic TAORYX host can prepare.
taoryx model default taoryx.parametric-interceptors.mission-composition \
  generic-medium-sam --output generic-medium-sam-default.json
```

For an explicit consumer-level default/streaming check, run:

```bash
python tools/validate_mission_composition_provider_contract.py \
  --plugin taoryx.parametric-interceptors \
  --contract-profile taoryx-universal \
  --execute-defaults
```

## Know the boundaries

This package is a low-fidelity, evidence-aware interceptor surrogate. Its
source-native local-NED/NEU channels remain available, while every public
batch sample and streaming observation also carries the shared ECEF position,
Earth-relative kinematics, body-frame angular velocity, and ECEF-from-body
quaternion. That portable sidecar does not turn a point-mass or response-law
surrogate into a source-qualified CADAC or rigid-body vehicle model.

For target-track and direct-acceleration command examples, calibration,
fitting, thrust curves, response analysis, and the exact provenance rules,
continue in the [package README](../README.md).
