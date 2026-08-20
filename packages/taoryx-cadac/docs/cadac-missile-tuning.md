# CADAC missile tuning and reduced-order authoring

This guide is the developer route for the CADAC missile families that are
needed first: AIM5, ADS6 SRBM/ROCKET5, ADS6 SAM, SRAAM6, and AGM6. It covers
source-backed *configuration variants*, not a claim that every source input is
a safe live control or that a lower-order model is a physical six-degree-of-
freedom plant.

## Select the correct model boundary

| Family | Fidelity and realization | Public tuning boundary |
| --- | --- | --- |
| AIM5 | `pseudo_6dof` / `response_law` | alpha/beta limit plus pitch/yaw reduced-order response-law parameters |
| ADS6 SRBM (`ROCKET5`) | `pseudo_6dof` / `response_law` | incidence limit and endo/ascent response-law parameters |
| ADS6 SAM | T4 fin/TVC or T3 aggregate RCS | physical-effector limits and dynamics; caller-owned batch commands |
| SRAAM6 | T4 four-fin | physical-fin, source seeker/filter, and controller parameters |
| AGM6 | T4 four-fin | physical-fin, source seeker/filter, controller, and constant-throttle parameters |

`pseudo_6dof` means that the source integrates translation plus the named
reduced-order response states. It does **not** imply quaternion attitude,
body-rate truth, inertia/moment closure, or individual physical effectors. Do
not advertise those missing states or label a response-law parameter as an
actuator control.

## Developer workflow

1. Bind exactly one local, authorized CADAC source case to its matching
   plug-in/provider. Do not discover a neighboring model and use it as a
   fallback.
2. Inspect the advertised schema first. Every parameter must have a label,
   description, canonical unit when dimensional, role, source provenance, and
   a source-backed domain when the source model declares one.
3. Add a field to the model's `*PluginOverrides`, validate its source-model
   domain, and copy it into a per-run `SourceDefinition`. Never alter the
   installed source definition or deck.
4. Add an alias to `build_default_*_configuration`, a configuration-schema
   group, and `_overrides_from_resolved` routing. Use `role="variant"` for
   settings that define a run before propagation rather than a live command.
5. Add the focused vertical tests: source immutability, typed configuration
   validation/unit handling, and configuration-to-plugin routing. A new
   missile model does not require retesting unrelated CADAC actors.
6. Increment the model metadata version when its public configuration grammar
   changes, and update callers/tests to use the exported version constant.

For the normal host-side inspection route, use:

```bash
taoryx model list
taoryx model plan cadac cadac.aim5.missile --fidelity pseudo_6dof
taoryx model plan cadac cadac.ads6.srbm --fidelity pseudo_6dof
```

The selected source-bound provider exposes the same configuration/output
metadata used by a UI or an agent. The helper builders are useful when writing
tests, services, or reproducible campaigns:

```python
configuration = build_default_aim5_configuration(
    provider,
    overrides={
        "alpha_max_deg": 32.0,
        "rate_loop_time_constant_s": 0.08,
        "proportional_integral_ratio": 1.5,
        "acceleration_loop_gain_rad_s2": 55.0,
    },
)
prepared = provider.validate_configuration(configuration)
```

The equivalent ADS6 SRBM response-law aliases are `alpha_limit_deg`,
`endo_boundary_altitude_m`, and `ascent_normal_bias_g`. Its guidance aliases
remain separately available for source seeker, PN, and spiral behavior.

## Tuning policy

A configuration variant may change the source-compatible behavior; it is not
automatically a calibrated or qualified vehicle. Keep these categories
separate:

- **Variant parameters** configure the plant, response law, seeker/filter, or
  source controller before a run. They remain fixed for that run.
- **Live controls** are explicitly advertised action channels with their own
  requested/achieved readback. ADS6 SAM provides this at its controller-output
  seam; the source-managed AIM5, SRBM, SRAAM6, and AGM6 controllers do not.
- **Sensor configuration** shapes a CADAC source seeker. The typed Taoryx
  `relative-state-track` remains the generic sensor interface and is not
  replaced by a source-private measurement API.

Use only bounds declared by the typed source model. A public lower bound is a
validation/normalization aid; it is not an assertion of an externally validated
upper operating envelope. Any fit to a higher-fidelity model, golden source
trace, or flight data needs its own evidence and promotion record.

## Focused verification

Run only the affected family during the inner loop:

```bash
python -m pytest \
  tests/families/cadac/test_aim5.py \
  tests/families/cadac/test_aim5_plugin.py \
  tests/families/cadac/test_aim5_mission_composition.py \
  tests/families/cadac/test_ads6_srbm.py \
  tests/families/cadac/test_ads6_srbm_plugin.py \
  tests/families/cadac/test_ads6_srbm_mission_composition.py \
  -q -x --strict-markers
```

For physical-fin work, substitute the ADS6 SAM, SRAAM6, or AGM6 triplet. The
focused suite verifies the plug-in's one-model vertical boundary. Reserve
`check-cadac` for intentional CADAC-wide integration changes; it is not the
normal response to a parameter addition in one vehicle family.
