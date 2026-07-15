# Grammar profiles

TAORYX has two language claims. They are profiles, not a silent replacement of
the historical language:

| Profile | Claim | Typical use |
| --- | --- | --- |
| `taos96` | Reconstructed TAOS Version 96.0 syntax and documented semantics | Historical fixtures and manual conformance |
| `taoryx` | `taos96` plus explicitly documented successor extensions | New rigid-body, interactive, and analysis features |

The parser records the selected profile on `ProblemDocument.grammar_profile`:

```python
from taoryx.language import GrammarProfile, parse_problem_file

document = parse_problem_file("mission.prb", profile=GrammarProfile.TAORYX)
```

The default is `taos96` so callers must opt into the successor claim. Existing
TAOS fixtures remain valid and continue to be parsed without a TAORYX profile.
The current dynamics directives are:

```text
*3dof    # point-mass translational mode
*6dof    # rigid-body translational + attitude mode
```

The older explicit extension form remains available for configuration tools:
`*mode point-mass`, `*mode kinematic-6dof`, and `*mode rigid-body-6dof`.

Runtime declarations are successor-only and can be repeated at problem scope:

```text
*runtime control throttle unit=fraction default=0.5 lower=0 upper=1 slew=2
*runtime status altitude source=alt unit=m
*runtime event ground condition=alt<0 action=stop
*runtime output channels=alt,vel interval=0.5 events=true
```

They lower into the shared `ScenarioRuntimeContract` and are available to
interactive stepping and artifact metadata. An API-supplied runtime contract
replaces the file declarations; this avoids silently merging contradictory
control definitions. Batch model-specific actuator application remains an
explicit adapter responsibility.
The new `*3dof`/`*6dof` directives are successor-only and are rejected unless
the `taoryx` profile is selected. Runtime lowering will not claim execution of
`*6dof` until the rigid-body problem builder is connected to the main runtime
kernel.

A future extension must provide all of the following before it is considered
part of the TAORYX profile:

1. A grammar contract and stable syntax.
2. A typed model representation.
3. Positive and negative parser fixtures.
4. Runtime lowering or an explicit unsupported-feature diagnostic.
5. Artifact and verification coverage.

`taos96` and `taoryx` must never be conflated in compatibility reporting.
