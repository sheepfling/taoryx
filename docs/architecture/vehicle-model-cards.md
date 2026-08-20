# Vehicle model cards

`taoryx model overview` produces a résumé-style, evidence-bounded overview for
each installed plug-in model. It is generated from the provider's typed model
metadata, configuration grammar, and registered tuning declarations, rather
than maintained as a second hand-written inventory.

Use a focused provider when reviewing a vehicle package. That avoids seeing the
same family again through the compatibility aggregate:

```bash
taoryx model overview \
  --provider taoryx.hummingbird.mission-composition \
  --model hummingbird \
  --output build/hummingbird-model-card.md

taoryx model overview \
  --provider taoryx.f16.mission-composition \
  --model f16_s119 \
  --format json \
  --output build/f16-model-card.json
```

Omit the filters to render every installed provider and model. That broader
view intentionally includes development fixtures and compatibility providers;
use `taoryx model list` first when choosing the focused provider/model pair.

## What a card records

Each card has the same review structure:

- identity, package/provider revision, model kind, status, and metadata
  fingerprint;
- the full fidelity ladder, including dynamics and input realization,
  promotion status, runnable operations, blockers, and claim boundaries;
- declared data provenance and source-reference scopes, plus an explicit count
  of configuration leaves that do or do not declare provenance;
- every registered common tuning campaign, its strategy, operating points,
  controller method, trim targets, design coordinates, and the command that
  would run it;
- mission templates, segment sequences, compatible fidelities, and per-
  operation availability; and
- initialization and segment parameters with types, units, required/default
  disposition, hard bounds, and provenance.

The Markdown form is meant for a reviewer. The JSON form retains the full
typed control, operation, value-space, and configuration records for a UI,
agent, or release process.

## Reading the evidence boundary

A card reports declarations, not conclusions that those declarations cannot
support. In particular:

- `promotion_status: development` or an available batch/session operation is
  not vehicle qualification;
- a source reference or populated provenance field is not independent source
  validation, data correlation, or numerical-parity evidence;
- a registered LQR/LQI campaign states how a local design screen is configured;
  it does not mean that the campaign has run, yielded gains, or demonstrated
  nonlinear mission performance; and
- parameter bounds and defaults are the advertised authoring contract, not an
  inferred operating envelope.

For the authoritative definitions of the four fidelity tiers, see
[Fidelity tiers and vehicle plug-in requirements](fidelity-data-requirements.md).
For authoring, compiling, executing, and tuning a selected configuration, see
[Model-to-mission authoring and automation](../developer/model-authoring-automation.md).
