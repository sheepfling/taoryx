# Taoryx parametric interceptors

This independent plug-in turns sparse, provenance-bearing interceptor records
into low-fidelity Taoryx models. It does not copy CADAC code or claim that
archetype defaults are facts. It provides a neutral `point_mass_3dof` kernel,
an opt-in `attitude_response_pseudo_6dof` response-law tier, and one shared
guidance/control kernel. Both tiers support selectable fixed-waypoint,
constant-velocity target-track, and direct local-NEU lateral-acceleration
missions in batch and through the standard Composition session API.

The shortest useful profile is plain Python and is also shaped like a YAML
mapping:

## What this plug-in is for

Use this independent plug-in to turn compact public, synthetic, or
catalogue-derived interceptor data into an honest, selectable low-fidelity
Taoryx vehicle. It owns the evidence-aware profile format, archetype
resolution, point-mass and attitude-response pseudo-6DOF realizations,
standard Composition controls, and the matching output/readback contract.

It is deliberately not a detailed CADAC replacement: it does not import a
CADAC vehicle, coefficient deck, atmosphere, gravity model, or controller;
and a resolved archetype assumption is never advertised as an observed
engineering fact or a qualified performance prediction.

## Choose an authoring route

| Need | Start here | What you get |
| --- | --- | --- |
| Try a new model in an application or study | `taoryx-interceptor init` then `from_yaml(...)` | An isolated provider containing only your profile; no registration code |
| Preserve an ontology or research record | Catalogue YAML then `from_catalogue_yaml(...)` | The same runnable provider with observed, derived, gap, and assumption provenance retained |
| Add a maintained model to this installed plug-in | A named witness/profile factory and focused plug-in test | A reviewed built-in that appears in `taoryx model list` after entry-point discovery |
| Represent a genuinely new vehicle family | A new versioned archetype | Explicit resolver defaults and diagnostics, rather than a hidden reinterpretation of an existing family |

For most developers, the first route is the right one: create a small,
app-local provider and compose it normally. Use the built-in route only when a
profile is intended to be maintained and advertised by this distribution.

## Add or integrate a parametric model

### App-local profile: the normal path

Start with the scaffold, inspect the resolved evidence and available controls,
then construct the provider directly from the same file:

```bash
taoryx-interceptor init my-sam.yaml --interceptor-id my-sam
taoryx-interceptor inspect my-sam.yaml --output my-sam-report.json
```

```python
from taoryx_parametric_interceptors import (
    ParametricInterceptorMissionCompositionProvider,
)

provider = ParametricInterceptorMissionCompositionProvider.from_yaml("my-sam.yaml")
```

`inspect` is the developer preflight: it reports provenance, unresolved
diagnostics, executable fidelity tiers, advertised control modes, standard
outputs, and the response-analysis contract. It is the quickest way to decide
whether the model is ready to run or still a resolver-only research record.

Use a catalogue-shaped file when the source material is already organized as
observed, derived, and missing claims:

```python
provider = ParametricInterceptorMissionCompositionProvider.from_catalogue_yaml(
    "my-interceptor-evidence.yaml"
)
```

Both constructors make an isolated provider; they do not add a global search
path or change what other plug-ins expose.

### Maintained built-in profile

Add a built-in only when it should ship with this distribution. Keep the
source/evidence seed and explicit claim boundary in `witnesses.py`, include it
in `runnable_prototype_profiles()` only after it is executable, and add a
focused vertical test for discovery, advertisement, and the intended fidelity
and mission template. Sparse profiles may remain resolver/provenance witnesses
without being advertised as runnable performance models.

Add a new archetype only when the existing versioned archetypes cannot express
the vehicle family. Its defaults, uncertainty case, parameter usage, and
required diagnostics must stay explicit; do not conceal a new family behind a
different model ID or an observed-value label.

```python
from taoryx_parametric_interceptors import interceptor, resolve_interceptor

profile = interceptor(
    "catalog-medium-sam",
    launch_mass_kg=420.0,
    length_m=5.2,
    body_diameter_m=0.36,
    propulsion_architecture="single_stage",
    burn_time_class="medium",
    thrust_profile_class="boost_sustain",
    aero_archetype="generic_slender_supersonic",
    drag_class="nominal",
    maneuverability_class="high",
    guidance_family="sarh",
    reported_max_speed_mps=1_200.0,
    reported_max_range_m=45_000.0,
    reported_max_altitude_m=24_000.0,
    target_classes=("air_breathing", "cruise_missile"),
)

resolved = resolve_interceptor(profile)
print(resolved.fingerprint)
print(resolved.parameters["drag_coefficient"].origin)
```

Raw values are conservatively labeled `simulation_assumption`. Add factual
provenance without changing the surrounding model shape:

```python
from taoryx_parametric_interceptors import interceptor, observed, reported

profile = interceptor(
    "catalog-medium-sam",
    launch_mass_kg=observed(420.0, unit="kg", source_record_id="ontology:variant-42:mass"),
    length_m=observed(5.2, unit="m", source_record_id="ontology:variant-42:length"),
    reported_max_range_m=reported(
        45_000.0,
        unit="m",
        source_record_id="source:public-range-claim",
    ),
)
```

The Python path exposes the same evidence vocabulary as catalogue YAML. No
low-level Pydantic construction is required:

```python
from taoryx_parametric_interceptors import derived, inferred, unavailable, variant_interval

profile = interceptor(
    "catalog-medium-sam",
    launch_mass_kg=observed(
        161.48,
        unit="kg",
        source_record_ids=("source:navair:mass", "ontology:amraam:mass-claim"),
    ),
    reference_area_m2=derived(
        0.02483,
        unit="m^2",
        method="circular_area_from_body_diameter",
        source_record_id="source:navair:diameter",
    ),
    aero_archetype=inferred(
        "generic_slender_supersonic",
        method="selected_from_public_geometry_and_speed_regime",
    ),
    evidence_gaps=(
        unavailable("reported_max_speed_mps", "classified", unit="m/s"),
    ),
    evidence_intervals=(
        variant_interval(
            "launch_mass_kg",
            157.85,
            162.39,
            unit="kg",
            source_minimum_value=348.0,
            source_maximum_value=358.0,
            source_unit="lb",
        ),
    ),
)
```

`source_record_id` is the short one-source form; `source_record_ids` accepts a
stable ordered sequence for corroborating records. Supplying both, duplicate
IDs, partial source-unit interval bounds, or whitespace-damaged IDs is an
authoring error. These helpers label evidence only—they do not cause inferred
or derived values to become observed facts.

The same profile can live in YAML. A value may be a raw scalar or an evidence
record:

```yaml
interceptor_id: catalog-medium-sam
launch_mass_kg:
  value: 420.0
  unit: kg
  origin: observed
  source_record_ids: [ontology:variant-42:mass]
  confidence: high
length_m: 5.2
body_diameter_m: 0.36
guidance_family: sarh
guidance_archetype: proportional_navigation
navigation_constant: 3.0
target_classes: [air_breathing, cruise_missile]
```

`guidance_family` and `guidance_archetype` intentionally answer different
questions. The family records sourced architecture such as command, SARH,
active radar, or infrared homing; it never silently selects executable code.
The archetype is the simulation assumption that selects either
`waypoint_pursuit` or `proportional_navigation`. `navigation_constant` is a
positive dimensionless tuning value used by proportional navigation. All three
retain independent evidence origins in the resolved profile.

```python
from taoryx_parametric_interceptors import (
    ParametricInterceptorMissionCompositionProvider,
    load_interceptor_profile,
)

profile = load_interceptor_profile("catalog-medium-sam.yaml")
provider = ParametricInterceptorMissionCompositionProvider.from_yaml(
    "catalog-medium-sam.yaml"
)
```

The repository includes a copy-ready starting point at
`examples/parametric_interceptors/generic_medium_sam.yaml`.

## File-first developer command

Installing the distribution adds `taoryx-interceptor`. It closes the gap
between a new evidence file and the standard Composition provider without
requiring a developer to write a registration module:

```bash
taoryx-interceptor init my-sam.yaml --interceptor-id my-sam
taoryx-interceptor schema --format all --output interceptor-authoring-schemas.json
taoryx-interceptor inspect my-sam.yaml --output my-sam-report.json
taoryx-interceptor run my-sam.yaml \
  --set runtime.duration_s=5 \
  --set runtime.time_step_s=0.05 \
  --set navigation.waypoint.north.command=12000 \
  --output my-sam-run.json

taoryx-interceptor run my-sam.yaml \
  --mission-template constant_velocity_target_intercept \
  --set navigation.target.position.north.command=12000 \
  --set navigation.target.position.altitude.command=3000 \
  --set navigation.target.velocity.east.command=150 \
  --output my-sam-target-run.json

taoryx-interceptor run my-sam.yaml \
  --mission-template direct_lateral_acceleration_control \
  --set control.lateral_acceleration.local.east.command=10 \
  --output my-sam-direct-control-run.json

taoryx-interceptor calibrate my-sam.yaml scenario.yaml \
  --output calibration-result.json
taoryx-interceptor compare-cases my-sam.yaml scenario.yaml \
  --output assumption-case-comparison.json
taoryx-interceptor fit my-sam.yaml fit-campaign.yaml \
  --output fit-receipt.json \
  --fitted-profile-output my-sam-fitted.yaml
taoryx-interceptor analyze-response my-sam.yaml response-request.yaml \
  --output response-analysis.json
taoryx-interceptor analyze-operating-response \
  my-sam.yaml operating-point-response.yaml \
  --output operating-point-response-analysis.json
taoryx-interceptor compare-operating-response \
  baseline.yaml candidate.yaml operating-point-response.yaml \
  --output operating-point-response-comparison.json
taoryx-interceptor compare-response baseline.yaml candidate.yaml response-request.yaml \
  --output response-comparison.json
```

The inspection report includes `composition.controller_analysis`: the
discoverable pseudo-6DOF response-analysis contract, its roll/pitch/yaw axes,
provider methods, and copy-ready `analyze-response` and
`analyze-operating-response` commands. These are local surrogate response
metrics, never a physical-controller or robustness qualification.

`init` writes a valid fast-start profile and refuses to overwrite an existing
file unless `--force` is explicit. Use `--format catalogue` for an
ontology-shaped evidence-gap scaffold instead. `inspect` automatically detects
either shape and emits
`taoryx.parametric-interceptor-authoring-report/v1`, including both input and
resolved fingerprints, values grouped by provenance origin, evidence gaps and
intervals, required diagnostics, calibration-target availability, selectable
fidelities, controls, and output channels.

`schema` emits the exact JSON Schema used by the flat and catalogue intake
surfaces. Editors, ontology exporters, CI jobs, and agents can request either
format or the default bundle without loading or running a model. The catalogue
contract forbids unknown keys at the record, evidence-field, derived-field,
interval, and resolver-assumption levels. It also rejects contradictory
value-plus-gap records, duplicate aliases across `evidence` and `derived`, and
unknown resolution statuses. These are hard intake errors: authored data is
never discarded merely because an archetype could fill the corresponding
simulation parameter.

`run` accepts exact advertised mission parameter IDs through repeatable
`--set PARAMETER=VALUE` arguments. It constructs the same provider and invokes
the same `MissionCompositionRunRequest`/runner used by ordinary Taoryx clients;
the command is only a file-intake front end, not another dynamics or streaming
route. The response is the standard discriminated Composition trajectory.

Every `inspect` report also contains a fail-closed parameter-usage manifest.
This prevents a resolved value from looking like an active model input merely
because it appears beside executable values. Each record has exact consumers,
resolved sink IDs, direct-consumption and dynamics-effect flags, and one usage
class:

- `dynamics_input`: read directly by point-mass, pseudo-6DOF, propulsion,
  guidance, or response analysis;
- `resolution_input`: indirectly reaches one or more active dynamics inputs
  through the resolved `depends_on` graph;
- `inactive_resolution_input`: a coarse selector retained for provenance but
  overridden or otherwise inactive in this exact resolved case;
- `runtime_advisory`: read during execution without changing forces or state,
  currently the optional applicability bounds;
- `calibration_target`: compared only in a scenario-qualified calibration;
- `derived_summary`: published for inspection but not consumed by dynamics;
- `evidence_only`: catalogue context with no current low-fidelity equation.

`usage_counts`, `parameters_by_usage`, and the readiness lists make these
distinctions easy to inspect without walking the dependency graph manually.
Classification fails if a newly resolved parameter reaches no executable sink
and has no explicitly declared non-executable role. Composition publishes the
same `taoryx.parametric-interceptors.parameter-usage/v1` manifest and repeats
the usage class, consumers, and active sinks in each resolved property’s
metadata. For example, `guidance_family` remains evidence-only while
`guidance_archetype` is a direct dynamics input; a supplied `control_features`
record is an active resolver input only when it actually selects the resolved
control configuration.
`--mission-template` chooses `fixed_waypoint_intercept` (the default),
`constant_velocity_target_intercept`, or
`direct_lateral_acceleration_control`; all three use the same selected runtime
tier and force-authority implementation. Their command grammars are mutually
exclusive. The common portable configuration retains defaults for all three,
but any non-default control owned by an inactive grammar fails with
`inactive-mission-control`, the exact configuration path, and the mission that
owns that channel. A wrong `--mission-template` therefore cannot silently
discard an otherwise valid-looking command.

`calibrate`, `compare-cases`, `fit`, `analyze-response`,
`analyze-operating-response`, `compare-operating-response`, and
`compare-response` are file-first front ends to the same typed Python APIs
documented below. Calibration emits a versioned result containing the scenario
and exact runtime-dependency identity. `compare-cases` runs that immutable
scenario against conservative, nominal, and optimistic resolution by default;
repeat `--case` to select a subset. Its versioned output binds the sparse source
profile fingerprint, scenario fingerprint, each distinct resolved-profile
fingerprint, full calibration result, and normalized-error ranking.
Fitting emits an immutable receipt and can optionally write a canonical flat
profile whose fitted scale values are explicitly `calibrated`. An unaccepted
candidate is never written unless `--allow-unaccepted-candidate` is explicit.
Response comparison preserves separate speed, overshoot, and authority
tradeoffs and never declares an overall controller winner.

A named, archetype-dominant record with unresolved required diagnostics—PAC-3
MSE is the deliberate witness—is reported as `resolver_only`. The command will
not execute it accidentally. A developer conducting an explicitly unqualified
surrogate study must acknowledge that boundary with `--allow-unqualified`;
the resulting model and trajectory remain labeled as unqualified surrogates.

The entry-point provider's built-in examples also remain available through the
host-wide authoring workflow:

```bash
taoryx model list --provider taoryx.parametric-interceptors.mission-composition
taoryx model plan taoryx.parametric-interceptors.mission-composition generic-medium-sam
taoryx model scaffold taoryx.parametric-interceptors.mission-composition \
  generic-medium-sam --fidelity point_mass_3dof --output mission.yaml
taoryx model compile mission.yaml --output prepared.json
taoryx model run taoryx.parametric-interceptors.mission-composition prepared.json
```

The plug-in-specific command creates or loads a new model profile. The common
`taoryx model` commands author missions for models already registered by an
installed entry point. Keeping those responsibilities separate avoids global
profile search paths or environment-dependent discovery order.

The compact flat profile uses the canonical units encoded in its field names.
When source material is in pounds, feet, inches, knots, degrees, or another
common public-source unit, use the catalogue-shaped route below so conversion
and source retention happen together.

Propulsion uses the same flat authoring surface. Most models need only
`propulsion_architecture`, `burn_time_class`, and `thrust_profile_class`.
Override `burn_time_s` for a known single-pulse duration. A dual-pulse study
can opt into exact program values without writing a motor class:

```yaml
propulsion_architecture: dual_pulse_solid
first_pulse_burn_time_s: 2.0
inter_pulse_coast_time_s: 3.0
second_pulse_burn_time_s: 4.0
second_pulse_thrust_ratio: 0.5
second_pulse_propellant_fraction: 0.6
```

Omitted pulse timing and allocation values are visibly resolved as archetype
assumptions. `burn_time_s` always means total active burn and excludes the
inter-pulse coast. Both runtime tiers consume one shared propulsion program,
so they cannot disagree about mass depletion, pulse transitions, or thrust.

When public evidence or an engineering assumption supplies absolute amplitude,
set `nominal_thrust_n`. It is the active-burn mean before assumption-case and
`thrust_scale` changes. Taoryx retains it as a separate evidence-bearing value;
the executable `thrust_n` and active-burn total impulse are derived and
advertised without modifying the source value.

When a coarse `neutral`, `regressive`, `progressive`, or `boost_sustain` shape
is not enough, add `thrust_profile_schedule`. The points use normalized active
burn fraction and relative multipliers; Taoryx integrates and normalizes them
to unit mean automatically:

```yaml
nominal_thrust_n:
  value: 22500.0
  unit: N
  origin: observed
  source_record_ids: [source:public-motor-figure]
  confidence: medium
thrust_profile_schedule:
  schedule_id: public-curve-digitization-v1
  interpolation: linear
  points:
    - {burn_fraction: 0.0, multiplier: 1.8}
    - {burn_fraction: 0.25, multiplier: 1.1}
    - {burn_fraction: 1.0, multiplier: 0.7}
  origin: reported
  source_record_ids: [source:public-motor-figure]
  confidence: medium
  method: relative curve digitized from the cited public figure
```

The first point must be at `0`, the last at `1`, fractions must increase, and
the integrated multiplier must be positive. `linear` and `step_previous`
interpolation are supported. Observed or reported schedules require source
IDs. The structured schedule replaces the built-in executable pulse shape;
an explicit `nominal_thrust_n` replaces the class-to-thrust-to-weight amplitude
fallback. When both are present, omit `thrust_profile_class` because it would
be unused. When only shape or amplitude is explicit, the class remains visibly
active only for the missing side. Catalogue records may use `mean_thrust`,
`average_thrust`, or `nominal_thrust`; values in `kN` and `lbf` are converted
to `N` with the original scalar retained. `thrust_scale` adjusts executable
amplitude separately. This separation lets developers change amplitude or
timing without hand-normalizing total impulse or altering pulse allocation.
See `examples/parametric_interceptors/custom_thrust_profile_sam.yaml` for a
copy-ready profile.

For the common case where a source provides an absolute time–thrust table,
`thrust_time_curve` removes the remaining normalization work:

```yaml
propulsion_architecture: single_stage_solid
thrust_time_curve:
  curve_id: public-static-test-v1
  time_unit: ms
  thrust_unit: kN
  interpolation: linear
  points:
    - {time: 0.0, thrust: 0.0}
    - {time: 250.0, thrust: 80.0}
    - {time: 1500.0, thrust: 30.0}
    - {time: 4000.0, thrust: 0.0}
  origin: reported
  source_record_ids: [source:public-static-test-plot]
  confidence: medium
  method: digitized from the cited public plot
```

Taoryx converts source units, integrates total impulse, derives active-burn
duration and mean thrust, and generates the unit-mean profile schedule. The
original source-unit points remain fingerprinted and advertised alongside all
derived values. A curve replaces `burn_time_class`, `burn_time_s`,
`thrust_profile_class`, `nominal_thrust_n`, and `thrust_profile_schedule`;
overlaps fail validation. Version 1 intentionally represents one continuous
single-pulse burn and rejects `dual_pulse_solid` rather than treating an
unlabeled zero-thrust interval as a coast. See
`examples/parametric_interceptors/absolute_thrust_curve_sam.yaml`.

For an explicitly dual-pulse motor, use `dual_pulse_thrust_program` instead. It
nests two independently sourced absolute curves around a declared coast:

```yaml
dual_pulse_thrust_program:
  program_id: public-two-pulse-program-v1
  first_pulse:
    curve_id: public-first-pulse-v1
    time_unit: ms
    thrust_unit: kN
    points:
      - {time: 0.0, thrust: 0.0}
      - {time: 500.0, thrust: 80.0}
      - {time: 2000.0, thrust: 0.0}
    origin: reported
    source_record_ids: [source:first-pulse-plot]
  inter_pulse_coast_time: 3000.0
  coast_time_unit: ms
  second_pulse:
    curve_id: public-second-pulse-v1
    time_unit: ms
    thrust_unit: kN
    points:
      - {time: 0.0, thrust: 0.0}
      - {time: 1000.0, thrust: 35.0}
      - {time: 4000.0, thrust: 0.0}
    origin: reported
    source_record_ids: [source:second-pulse-plot]
  origin: reported
  source_record_ids: [source:pulse-timing]
```

The compiler derives dual-pulse architecture, both burn durations, total
active-burn mean thrust, the second/first mean-thrust ratio, and independent
unit-mean schedules. The shared runtime then reconstructs both absolute curves
while preserving total impulse. Coast timing and each curve retain separate
lineage. `second_pulse_propellant_fraction` remains a separate input or visible
archetype assumption: curve impulse cannot determine propellant allocation
without a specific-impulse model. See
`examples/parametric_interceptors/dual_pulse_thrust_curves_sam.yaml`.

When a common effective specific impulse is available, the opt-in mass lowering
removes that last manual step:

```yaml
launch_mass_kg: 320.0
effective_specific_impulse_s:
  value: 250.0
  unit: s
  origin: reported
  source_record_ids: [source:effective-motor-isp]
dual_pulse_thrust_program: # two explicit absolute curves, as above
  # ...
```

`effective_specific_impulse_s` requires explicit launch mass, thrust amplitude,
and burn timing. An absolute single-pulse curve or dual-pulse program supplies
the latter two automatically. The resolver computes nominal propellant mass as
`impulse / (Isp * standard_gravity)`, then derives burnout mass and propellant
fraction. For a dual-pulse program, one common effective Isp also derives the
second-pulse propellant fraction from its nominal impulse share. Per-pulse
efficiency differences are not inferred. Explicit `burnout_mass_kg`,
`propellant_fraction`, or `second_pulse_propellant_fraction` would overdetermine
this route and therefore fail validation.

The mass calculation uses the evidence-bearing nominal impulse, before
assumption-case or fitted `thrust_scale` changes. Conservative and optimistic
cases can bound executable thrust without silently changing wet or dry mass.
Every derived mass value keeps operand source IDs and exact `depends_on`
lineage in Composition. Catalogue records may use the shorter aliases
`specific_impulse` or `effective_specific_impulse`; both canonicalize to
seconds.

## Catalogue-shaped evidence records

The ontology-facing adapter accepts the nested evidence/derived/assumption
shape directly:

```python
from taoryx_parametric_interceptors import (
    ParametricInterceptorMissionCompositionProvider,
    load_catalogue_interceptor_record,
    resolve_interceptor,
)

profile = load_catalogue_interceptor_record(
    "examples/parametric_interceptors/aim120_c5_c7_catalogue_seed.yaml"
)
resolved = resolve_interceptor(profile)

provider = ParametricInterceptorMissionCompositionProvider.from_catalogue_yaml(
    "examples/parametric_interceptors/aim9x_block2_catalogue_seed.yaml",
    "examples/parametric_interceptors/aim120_c5_c7_catalogue_seed.yaml",
)
```

The adapter deliberately preserves distinctions that are easy to lose during
model intake:

- unknown or misplaced fields fail before resolution instead of being ignored;
- a canonical parameter may occur only once across `evidence` and `derived`,
  including aliases such as `length` and `length_m`;
- `value: null` plus a status becomes a typed evidence gap, never a numeric
  parameter;
- common source units are converted to the field's canonical unit while the
  original scalar and unit remain in `source_value`;
- `observed_converted` remains observed while retaining the original source
  value and unit;
- family-level numeric intervals are normalized with their original bounds and
  unit retained, and never overwrite the selected variant value;
- resolver diagnostics and source/archetype dominance are advertised through
  Composition metadata;
- every retained value is labeled as executable, indirect, inactive,
  advisory, calibration-only, summary-only, or evidence-only;
- missing reported speed, range, or altitude is not populated from an
  archetype because these are optional calibration observations, not dynamics
  inputs.

For example, the adapter accepts this directly—no pre-calculation is required:

```yaml
evidence:
  launch_mass: {value: 356, unit: lb, origin: observed}
  length: {value: 12, unit: ft, origin: observed}
  max_bank_angle_rad: {value: 75, unit: deg, origin: simulation_assumption}
  reported_speed: {value: 2500, unit: km/h, origin: reported}
```

The resolved values are `kg`, `m`, `rad`, and `m/s`, respectively. Incompatible
dimensions fail before resolution. `canonicalize_interceptor_value(...)`,
`canonical_unit_for_interceptor_parameter(...)`, and
`supported_interceptor_units(...)` expose the same conversion table for intake
tools and UIs. The complete copy-ready example is
`examples/parametric_interceptors/mixed_unit_authoring_catalogue_seed.yaml`.
Mach is intentionally absent because converting it requires an atmospheric
operating condition rather than a fixed scalar factor.

## Mach-dependent drag without an aerodynamic deck

The fast-start route still needs only `aero_archetype` and `drag_class`. The
resolver turns those two coarse assumptions into a visible, versioned
Mach–drag schedule with a transonic rise and supersonic decay. The schedule is
an archetype assumption—not a hidden CADAC table or sourced coefficient deck.
Conservative, nominal, and optimistic cases scale every ordinate and therefore
produce distinct schedule and profile fingerprints.

When better data or a deliberate engineering surrogate is available, replace
both coarse selectors with one structured schedule:

```yaml
drag_coefficient_schedule:
  schedule_id: my-sam-drag-v1
  origin: simulation_assumption
  confidence: low
  method: Developer-authored generic transonic/supersonic shape.
  points:
    - {mach: 0.0, coefficient: 0.30}
    - {mach: 0.8, coefficient: 0.33}
    - {mach: 1.0, coefficient: 0.49}
    - {mach: 1.2, coefficient: 0.43}
    - {mach: 2.0, coefficient: 0.36}
    - {mach: 4.0, coefficient: 0.32}
```

Mach points must be finite, nonnegative, unique, and strictly increasing;
coefficients must be positive. Runtime interpolation is linear and values
outside the table hold the nearest endpoint. This explicit held-endpoint rule
avoids undocumented high-Mach polynomial extrapolation. An `observed` or
`reported` schedule requires source-record IDs. `drag_scale` remains the only
fit coordinate and scales the complete curve without rewriting its source
record; a fitted curve is labeled `calibrated`.

Both runtime tiers consume the same resolved schedule. Composition advertises
its ID, origin, fingerprint, point count, Mach domain, points, interpolation,
and extrapolation, while `aerodynamics.drag_coefficient` reports the active
interpolated value beside Mach, dynamic pressure, and drag. A complete example
is `examples/parametric_interceptors/custom_mach_drag_sam.yaml`.

## Control authority without an actuator model

Most developers can continue to author only `maneuverability_class`. The
resolver selects a visible `control_configuration` and force-authority defaults
from the archetype. If more control is useful, the complete override is three
fields:

```yaml
control_configuration: mixed  # aerodynamic | thrust_assisted | mixed
normal_force_coefficient_limit: 8.0
max_thrust_vector_angle_rad: 0.1745329252  # 10 deg
# control_allocation_policy: proportional  # optional; see policies below
```

`aerodynamic` uses only the normal-force coefficient and rejects a thrust-vector
angle. `thrust_assisted` uses only the angle and rejects a normal-force
coefficient. `mixed` adds both. Catalogue input may express the angle in
degrees through `max_thrust_vector_angle`; the adapter converts and retains the
source value. A sourced `control_features: [thrust_vectoring]` record infers a
mixed surrogate when no explicit configuration is supplied, as in the AIM-9X
witness. That inference says how this reduced-order simulation should allocate
authority; it does not claim a sourced autopilot, actuator, or control law.

At each accepted boundary both fidelity tiers call the same evaluator:

```text
a_aero = dynamic_pressure * reference_area * Cn_limit / mass
a_tvc  = thrust * sin(max_vector_angle) / mass
a_available = min(enabled component sum, structural maneuver limit)
a_achieved = min(requested lateral acceleration, a_available)
```

The optional allocation policy is useful for a `mixed` sensitivity study:

- `aerodynamic_first` is the default and spends aerodynamic authority before
  vectoring only enough thrust for the remainder;
- `thrust_vector_first` spends TVC authority before aerodynamic authority;
- `proportional` splits achieved demand using the current unclipped
  aerodynamic-to-TVC authority ratio.

Single-component configurations naturally produce the same allocation under
all three names. A mixed policy changes achieved maneuver drag and remaining
axial thrust, so the choice is a dynamics input rather than presentation-only
metadata. The achieved TVC angle and axial force are computed consistently:

```text
theta_achieved = asin(thrust-vector acceleration * mass / thrust)
axial_thrust = thrust * cos(theta_achieved)
```

Thus lateral thrust is never added while retaining the same full axial thrust.
Every policy remains a labeled simulation assumption. The authoring JSON Schema
and Composition property `control_allocation_policy.supported` advertise the
exact choices; unknown names fail at resolution rather than falling back
silently.

Achieved aerodynamic force also carries a load-dependent drag increment:

```text
Cn_achieved = mass * aerodynamic_acceleration_achieved / (q * reference_area)
Cdm = maneuver_drag_factor * Cn_achieved^2
Dm = q * reference_area * Cdm
```

`maneuver_drag_factor` defaults visibly to the low-confidence archetype
assumption `0.1`. A developer may author another nonnegative value, including
zero for an explicit no-penalty sensitivity case; catalogue records may use
the alias `induced_drag_factor`. The term uses only achieved aerodynamic
allocation. Thrust-vector contribution does not create aerodynamic maneuver
drag, and zero lateral demand leaves the existing Mach schedule unchanged.

The runtime preserves that separation in telemetry: the active
`aerodynamics.drag_coefficient` remains the base Mach-schedule value, while
maneuver normal-force coefficient, maneuver factor, maneuver coefficient,
base drag, maneuver drag, total coefficient, and total drag are separate
channels. `drag_scale` continues to scale only the base Mach schedule. This
quadratic term is a transparent reduced-order energy penalty, not an induced
drag polar, angle-of-attack model, CFD result, or coefficient deck.

The structural maneuver limit continues to resolve from
`maneuverability_class` and the assumption case. It is intentionally separate
from instantaneous force authority. Consequently, aerodynamic authority grows
with dynamic pressure, thrust-vector authority becomes zero during inter-pulse
coast and burnout, and neither component can exceed the structural envelope.
`maneuverability_scale` adjusts `normal_force_coefficient_limit`; it does not
rewrite that structural envelope.

Composition advertises the resolved configuration and coefficient/angle as
ordinary model properties. Runtime output separates aerodynamic,
thrust-vector, combined-unclipped, currently available, and structural-limit
status, plus achieved component allocation, achieved vector angle, remaining
axial thrust, and utilization against both current authority and the structural
envelope. Stable limit reasons identify aerodynamic, thrust-vector, or combined
authority saturation. This is a transparent force surrogate, not a physical
aerodynamic deck, actuator model, autopilot, stability proof, or copied CADAC
controller. See the copy-ready
`examples/parametric_interceptors/custom_control_authority_sam.yaml`.

The scalar commanded and achieved lateral accelerations are accompanied by
three components each under
`guidance.lateral_acceleration.{commanded,achieved}.local.{north,east,vertical}`.
These channels use the advertised `local_neu` frame, so `vertical` is explicitly
positive upward. The commanded vector is the unbounded guidance-law request
before force or attitude realization. The achieved vector is the force-limited
result for point mass and the force- plus attitude-response-limited result for
pseudo-6DOF. Each scalar is the Euclidean magnitude of its corresponding vector;
zero authority or unavailable target-track guidance therefore reports a zero
achieved vector without hiding a nonzero valid command.

Three common channels remove another downstream normalization ambiguity:
`guidance.lateral_acceleration.achievement_fraction` is actual achieved
magnitude divided by nonzero command and bounded to `[0, 1]`;
`guidance.lateral_acceleration.direction_error` is the angle between the two
nonzero vectors; and `.direction_error.valid` says whether that angle exists.
A zero command with zero achievement reports fraction `1` because there was no
demand, but its direction error remains invalid and numerically zero. A nonzero
command with no authority or no initial pseudo-6DOF directional response
reports fraction `0`, invalid direction error, and zero error value. This
achievement fraction is distinct from structural-envelope utilization and from
the pseudo-6DOF available-authority support fraction.

The pseudo-6DOF tier also scales its bounded attitude-response acceleration by
the fraction of the current guidance demand that the force model can support.
It publishes `control.attitude_response.authority_available`,
`control.attitude_response.command_support_fraction`, and
`control.attitude_response.authority_limited`. The fraction is current
available authority divided by nonzero commanded lateral acceleration, bounded
to `[0, 1]`; zero demand reports `1` by convention, while the separate
availability flag says whether any force authority exists. With zero authority,
the response law contributes no angular acceleration. An existing angular rate
therefore coasts instead of receiving hidden response-law damping. Point mass
does not advertise or emit these attitude-only channels.

When a target-track packet is unavailable, pseudo-6DOF guidance and attitude
commands both hold the current velocity direction. The reported target truth
remains available for mission range/capture accounting, but it is never used as
a controller fallback.

The same tier projects standard-environment air-relative velocity into its
response body's forward/right/down axes and publishes the three components,
`aerodynamics.angle_of_attack`, `aerodynamics.sideslip_angle`, and an explicit
validity flag. The definitions are:

```text
alpha = atan2(body_down_velocity, body_forward_velocity)
beta  = atan2(body_right_velocity,
              hypot(body_forward_velocity, body_down_velocity))
```

At zero airspeed the direction is undefined, so validity is false and the
components and angles are reported as zero. Density is intentionally not a
validity condition: relative-wind geometry remains defined in a vacuum when
airspeed is nonzero, even though aerodynamic force authority is zero. These
channels expose response-state geometry for plotting, control inspection, and
future model development. They do not add an angle-of-attack state, coefficient
deck, static-margin model, or aerodynamic moment calculation, and they are not
advertised by the point-mass tier.

## Declared applicability is not reported performance

An interceptor profile may optionally state an advisory model domain:

```yaml
applicability_altitude_min_m: 100.0
applicability_altitude_max_m: 25000.0
applicability_mach_min: 0.1
applicability_mach_max: 5.0
```

Every bound is optional. Omitted bounds remain undeclared; the resolver never
fills them from an archetype. Catalogue records may use
`operating_altitude_min`, `operating_altitude_max`, `operating_mach_min`, and
`operating_mach_max`, including ordinary source units such as feet. Conversion
retains the original value, unit, origin, and source IDs.

Both fidelity tiers evaluate the same envelope after sampling the standard
environment and computing air-relative Mach. Composition reports whether an
envelope was declared, one of `not_declared`, `within_declared_envelope`, or
`outside_declared_envelope`, and stable altitude/Mach reason codes. Enforcement
is explicitly `advisory`: an out-of-domain trajectory remains runnable and
labeled rather than terminating, disabling controls, or pretending the
extrapolation is qualified.

These fields describe where the surrogate is intended to apply. They are not
aliases for `reported_max_altitude_m`, reported speed/range, or engagement
range. Those public claims remain scenario-qualified calibration targets and
do not become runtime validity limits automatically. The mixed-unit catalogue
example demonstrates the complete provenance-preserving route.

Three prototype seeds exercise the intended paths:

| Witness | Resolver behavior | Built-in execution |
| --- | --- | --- |
| AIM-9X Block II | populated compact/high-agility profile with classified performance gaps | point mass and pseudo-6DOF |
| AIM-120 C5/C7 | variant-qualified medium-range profile retaining original imperial values and the unresolved family mass interval | point mass and pseudo-6DOF |
| PAC-3 MSE | sparse, archetype-dominant profile with explicit missing-geometry/mass diagnostics | resolver/provenance witness only |

The built-in plug-in registers the first two alongside the assumption-only
generic exemplar. PAC-3 MSE remains available through `pac3_mse_profile()` but
is intentionally not presented as a performance-credible built-in model.

Every omitted field records the archetype ID, assumption case, and fill method.
Optimistic, nominal, and conservative cases produce different immutable
fingerprints:

```python
from taoryx_parametric_interceptors import AssumptionCase, resolve_interceptor

conservative = resolve_interceptor(profile, assumption_case=AssumptionCase.CONSERVATIVE)
nominal = resolve_interceptor(profile)
optimistic = resolve_interceptor(profile, assumption_case=AssumptionCase.OPTIMISTIC)
```

## Scenario-qualified calibration screens

Reported maxima stay inert evidence until a developer explicitly states how a
simulation scenario corresponds to the source claim. The short path promotes
only available reported fields and records unavailable ones:

```python
from taoryx_parametric_interceptors import (
    PointMassMission,
    calibration_scenario_from_reported_profile,
    evaluate_interceptor_calibration,
)

scenario = calibration_scenario_from_reported_profile(
    resolved,
    "public-speed-screen-v1",
    scenario_basis=(
        "The source speed definition and launch condition have been normalized "
        "to this fixed-waypoint prototype screen."
    ),
    mission=PointMassMission(duration_s=20.0),
    relative_tolerance=0.20,
)
result = evaluate_interceptor_calibration(resolved, scenario)

print(result.observables["peak_speed_mps"])
print(result.evaluation.as_dict())
```

A record with classified or unresolved values is never filled just to make a
score. For example, the AIM-9X and AIM-120 prototype records cannot create a
reported-speed/range screen from their classified gaps.

For sensitivity—not model fitting—the same immutable scenario can be evaluated
against all named assumption cases:

```python
from taoryx_parametric_interceptors import compare_assumption_cases

comparison = compare_assumption_cases(profile, scenario)
print(comparison.best_case)
print(comparison.ranking)
```

The same operation requires no Python glue:

```bash
taoryx-interceptor compare-cases interceptor.yaml scenario.yaml \
  --output assumption-case-comparison.json
```

The ranking is normalized target error across the selected cases. It does not
change parameters, promote the winning case to evidence, or claim that the
lowest-error archetype is physically correct. The result embeds Taoryx's
provider-neutral `TrajectoryEvaluation`, including target metrics, waypoint
inputs, remaining propellant, capture outcome, evidence gates, scenario
fingerprint, and an explicit unqualified status. Archetype-dominant
resolver-only profiles still require `--allow-unqualified`; comparison does not
weaken the ordinary execution acknowledgement boundary.

Fully explicit scenarios can be written as YAML and loaded with
`load_interceptor_calibration_scenario(...)`. See
`examples/parametric_interceptors/generic_medium_sam_calibration.yaml`.
An existing Composition provider offers the same path as
`provider.reported_calibration_scenario(...)` followed by
`provider.evaluate_calibration(...)`, so applications do not need to reach
around the selected model registry.

Every scenario binds `sensor_suite_id`, `sensor_suite_version`, and the exact
suite fingerprint in addition to environment and gravity IDs. Results repeat
those fields plus the tier-specific navigation-provider kind and target-track
provider kind. Direct evaluation rejects a nonstandard dependency without its
implementation, and provider evaluation rejects an unregistered or
version/fingerprint-mismatched suite. Thus target-sensor bias/noise used by
guidance cannot be changed behind an otherwise identical calibration
fingerprint.

## Bounded multi-scenario fitting

Fitting is an authoring operation over explicit simulation-only scale factors,
not a mutation of ontology evidence. The initial supported variables are:

- `thrust_scale`
- `drag_scale`
- `maneuverability_scale`
- `guidance_time_constant_scale`

Each defaults to `1.0` and is advertised with its resolved origin. `drag_scale`
scales every ordinate of the resolved Mach-drag schedule, while
`maneuverability_scale` scales the resolved normal-force coefficient used by
instantaneous aerodynamic authority. A fit may
replace only an absent scale, a `simulation_assumption`, or a prior
`calibrated` value. An observed, reported, derived, or inferred scale fails
closed.

```python
from taoryx_parametric_interceptors import (
    InterceptorFitCampaign,
    InterceptorFitVariable,
    apply_interceptor_fit_receipt,
    fit_interceptor_profile,
)

campaign = InterceptorFitCampaign(
    campaign_id="medium-sam-fit-v1",
    scenarios=(speed_scenario, altitude_scenario),
    variables=(
        InterceptorFitVariable(
            parameter_id="thrust_scale",
            lower_bound=0.5,
            upper_bound=1.5,
        ),
        InterceptorFitVariable(
            parameter_id="drag_scale",
            lower_bound=0.5,
            upper_bound=1.5,
        ),
    ),
)
receipt = fit_interceptor_profile(profile, campaign)
fitted_profile = apply_interceptor_fit_receipt(profile, receipt)
fitted_provider = ParametricInterceptorMissionCompositionProvider.from_profiles(
    fitted_profile
)
```

The default `builtin-rqp` backend keeps the workflow usable in an isolated
plug-in installation. Optional Taoryx optimizer backends remain selectable.
The receipt preserves the base-profile and campaign fingerprints, every bound
and initial/fitted value, optimizer status, baseline and fitted scenario
evaluations, objective values, acceptance threshold, and fitted profile
fingerprints.

Each campaign scenario retains the same sensor-suite binding as standalone
calibration. A nonstandard environment, gravity model, or sensor suite must be
passed through the fitting dependency registries; naming one without supplying
it fails closed instead of silently running the portable default.

Numerical convergence and campaign acceptance are separate. Fixed-step
trajectory objectives can become flat enough for an optimizer to report
`stalled` even when every target is inside tolerance. A receipt is accepted
only when its maximum normalized target error is at or below the campaign's
explicit `acceptance_normalized_error`. Receipt application requires acceptance
by default and always verifies the exact base profile fingerprint.

`load_interceptor_fit_campaign(...)`, `write_interceptor_fit_receipt(...)`, and
`load_interceptor_fit_receipt(...)` provide the file workflow. See
`examples/parametric_interceptors/generic_medium_sam_fit_campaign.yaml`.

## Inspect or compare pseudo-6DOF response tuning

The pseudo-6DOF tier exposes its reduced-order response tuning without asking a
developer to construct a state matrix:

```python
from taoryx_parametric_interceptors import (
    Pseudo6ResponseAnalysisRequest,
    analyze_pseudo6_response,
    compare_pseudo6_responses,
)

request = Pseudo6ResponseAnalysisRequest(
    analysis_id="medium-sam-response-v1",
    axis="roll",
    sample_time_s=0.05,
    command_step_rad=0.17453292519943295,  # 10 deg
    command_support_fraction=0.6,  # frozen local authority approximation
)
report = analyze_pseudo6_response(profile, request)
comparison = compare_pseudo6_responses(baseline_profile, candidate_profile, request)
```

When a developer has a concrete force operating point, they do not need to
guess `command_support_fraction`. Capture the four required values directly or
lift them from any pseudo-6DOF runtime sample:

```python
from taoryx_parametric_interceptors import (
    Pseudo6OperatingPointAnalysisRequest,
    Pseudo6ResponseOperatingPoint,
    analyze_pseudo6_response_at_operating_point,
)

operating_point = Pseudo6ResponseOperatingPoint.from_sample(
    run.samples[10],
    operating_point_id="boost-turn-t0p5",
    source_basis="Selected deterministic boost-turn sample.",
)
report = analyze_pseudo6_response_at_operating_point(
    profile,
    operating_point,
    Pseudo6OperatingPointAnalysisRequest(
        analysis_id="boost-turn-local-response",
        axis="yaw",
        sample_time_s=0.05,
    ),
)
print(report.operating_point_resolution.response_support_fraction)
print(report.response_analysis.discrete_status)
```

The resolution recomputes aerodynamic and thrust-vector authority using the
same `q*S*Cn/m` and `thrust*sin(vector_angle)/m` evaluator as both runtime
tiers, applies the same structural clipping, and then freezes the exact
pseudo-6DOF response-support multiplier. It records the operating-point and
profile fingerprints, component authority, parameter provenance, and both the
ordinary command-support fraction and the runtime-equivalent response-support
fraction. At zero command, command support remains one by convention while
response support remains zero when no force authority exists; the two meanings
cannot be silently conflated. The provider exposes the same route as
`provider.analyze_pseudo6_response_at_operating_point(...)`.
The copy-ready case is
`examples/parametric_interceptors/generic_medium_sam_operating_point_response.yaml`;
run it with `taoryx-interceptor analyze-operating-response PROFILE CASE`.
For a matched-condition comparison, use the same case with
`taoryx-interceptor compare-operating-response BASELINE CANDIDATE CASE` or
`provider.compare_pseudo6_responses_at_operating_point(...)`. The comparison
binds both reports to the same force inputs but resolves available authority and
response support separately for each profile. Its deltas distinguish those
authority effects from local response-tuning metrics, never select an overall
winner, and retain the same non-qualification boundary.

The report includes the exact unsaturated continuous poles and exact
semi-implicit-update poles after freezing the runtime's command-support
multiplier at the requested value. It reports resolved and effective natural
frequency/damping, spectral margins, continuous and sampled settling estimates,
analytic step overshoot, pre- and post-support initial acceleration, and
acceleration, rate, and axis-angle headroom. A fraction of `1` is the nominal
full-support response. A fraction of `0` honestly reports a marginal,
unresponsive response with no settling estimate or fabricated finite sampling
limit. The linear response and rate/acceleration limits are shared across roll,
pitch, and yaw; angle topology is explicit and axis-specific: resolved bank
bounds for roll, the runtime's ±89-degree pitch guard, and periodic wrapped
yaw. Parameter values retain their resolver origins and methods. The comparison
uses one identical axis/request, reports
candidate-minus-baseline deltas, and keeps response speed, overshoot, and
authority margins separate instead of inventing an overall winner.

The standalone bounded-step witness accepts the same frozen support fraction
and applies it at the same point in the runtime equation: after pre-support
angular-acceleration limiting and before rate integration. It therefore checks
the local analytic result without inventing an environment or motor state.
Mission propagation still recomputes support from current aerodynamic and
propulsion authority every accepted boundary. A frozen report is local and
does not qualify a changing trajectory, gain schedule, or physical controller.

`provider.analyze_pseudo6_response(...)`,
`provider.compare_pseudo6_responses(...)`, and their operating-point variants
expose the same functions to Composition consumers. Model metadata advertises
the contracts and default time-step result. Pseudo-6DOF configuration validation
rejects a time step when the implemented local discrete response is unstable;
point-mass validation is unchanged. The copy-ready request is
`examples/parametric_interceptors/generic_medium_sam_response_analysis.yaml`.

`run_pseudo6_attitude_step(...)` and the equivalent provider method execute a
single-axis bounded witness through the same acceleration, rate, semi-implicit
update, and angle-limit code used by mission propagation. The trace separates a
limited requested command from acceleration, rate, and state-angle limiting.
This is useful for checking when the unsaturated analytic metrics apply without
running propulsion, atmosphere, translation, guidance, or sensors.

This is analysis of the symmetric, unsaturated second-order response law at one
operating point. It is not evidence that a missile airframe or physical
autopilot is stable, and it makes no nonlinear, scheduled, actuator, or robust
stability claim.

## Composition in a few lines

The convenience builder hides the configuration AST while still returning the
standard typed Taoryx request object:

```python
from taoryx.trajectory.execution_contract import MissionCompositionRunRequest
from taoryx_parametric_interceptors import ParametricInterceptorMissionCompositionProvider

provider = ParametricInterceptorMissionCompositionProvider.from_profiles(profile)
configuration = provider.configuration(
    profile.model_id,
    # Omit this line to keep the faster point-mass default.
    fidelity="attitude_response_pseudo_6dof",
    navigation_waypoint_north_command=20_000.0,
    navigation_waypoint_altitude_command=5_000.0,
    runtime_duration_s=45.0,
)
prepared = provider.validate_configuration(configuration)
response = provider.build_runner().run(
    MissionCompositionRunRequest(
        request_id="interceptor-demo",
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
    )
)
```

The configuration also contains three categorical runtime dependencies:
`runtime.environment_model_id`, `runtime.gravity_model_id`, and
`runtime.sensor_suite_id`. Their portable
defaults select Taoryx's standard exponential atmosphere and inverse-square
gravity equation plus the standard translation-acceleration, ideal-IMU, and
relative-state-track sensor providers. Applications can register additional standard
`EnvironmentProvider` instances and gravity callables under stable IDs when
constructing the provider. Sensor suites reference ordinary Taoryx sensor
provider kinds and validated provider configuration; they do not introduce a
private interceptor sensor API. All selected IDs participate in the schema and
prepared-configuration fingerprints and appear in batch diagnostics and
live-session lowering evidence. Environment, gravity, and exact sensor-suite
identity are also retained by calibration scenarios. A registry ID is therefore a versioned behavior
contract; callers must not reuse one ID for different behavior.

For example, this suite keeps the standard ideal IMU at pseudo-6DOF while
giving the point-mass translation sensor a 250 ms delivery delay:

```python
from taoryx.runtime.sensor_contracts import SensorProviderConfig
from taoryx_parametric_interceptors import InterceptorSensorSuite

suite = InterceptorSensorSuite(
    id="my-org.interceptor-navigation.delayed-v1",
    version="1.0.0",
    point_mass_provider=SensorProviderConfig(
        kind="translation-acceleration",
        config={"delivery_delay_s": 0.25},
    ),
    pseudo6_provider=SensorProviderConfig(kind="ideal"),
    provenance="my-org simulation configuration registry",
    claim_boundary="Simulation integration case; no hardware qualification.",
)
provider = ParametricInterceptorMissionCompositionProvider.from_profiles(
    profile,
    sensor_suites={suite.id: suite},
    default_sensor_suite_id=suite.id,
)
```

Suite construction rejects providers that cannot consume the tier's committed
truth mode, support accepted-boundary sampling, or emit its versioned payload
schema. The surrogate samples at every accepted integration boundary; it does
not silently substitute the sensor manifest's default clock. Each batch
execution and each session reset receives a fresh registered sensor instance. Sensor outputs
include validity, interval, sample time, availability time, delivery latency,
fresh-delivery status, monotonic packet sequence, payload schema ID, and typed
measurement values. A packet is not exposed before `available_at`; between
deliveries the latest delivered packet is held with `delivery_fresh=false`.
Pending packets are part of the session checkpoint. This preserves native
packet timing and checkpoint semantics instead of flattening a sensor into
unlabeled numbers.

Both kernels subtract the provider's ECFC wind from the surrogate's
radial/east/north velocity embedding before computing airspeed, Mach, active
drag coefficient, dynamic pressure, and drag. Discovery publishes density,
pressure, temperature, speed of sound, all three ECFC wind components,
airspeed, Mach, active drag coefficient, dynamic pressure, and drag. Malformed
provider samples—including non-ECFC wind, negative density or
pressure, or nonpositive temperature/speed of sound—fail at the kernel
boundary instead of silently producing misleading aerodynamics.

For a live session, select either fidelity and choose one mission grammar.
Point mass remains the default; add
`fidelity="attitude_response_pseudo_6dof"` for the attitude-response tier:

```python
from taoryx.trajectory.session_contract import (
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionManager,
    MissionCompositionSessionStepRequest,
)

configuration = provider.configuration(
    profile.model_id,
    startup_authority_profile_id="live_waypoint_guidance",
)
prepared = provider.validate_configuration(configuration)
sessions = MissionCompositionSessionManager(provider)
session = sessions.open(
    MissionCompositionOpenSessionRequest(
        session_id="interceptor-live-demo",
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
        integration_step_s=0.05,
    )
)
result = sessions.step(
    MissionCompositionSessionStepRequest(
        session_id=session.session_id,
        action={
            "navigation.waypoint.north.command": 18_000.0,
            "navigation.waypoint.east.command": 2_000.0,
            "navigation.waypoint.altitude.command": 4_500.0,
            "navigation.waypoint.capture_radius.command": 100.0,
        },
        duration_s=0.25,
        expected_sequence=0,
    )
)
```

The moving-target route is the same session API with a different, narrow
authority surface:

```python
from taoryx_parametric_interceptors import TARGET_TRACK_MISSION_TEMPLATE_ID

configuration = provider.configuration(
    profile.model_id,
    mission_template_id=TARGET_TRACK_MISSION_TEMPLATE_ID,
    navigation_target_position_north_command=18_000.0,
    navigation_target_position_east_command=2_000.0,
    navigation_target_position_altitude_command=4_500.0,
    navigation_target_velocity_north_command=120.0,
    navigation_target_velocity_east_command=20.0,
    navigation_target_velocity_vertical_command=0.0,  # positive upward
    navigation_target_capture_radius_command=100.0,
)
prepared = provider.validate_configuration(configuration)
session = sessions.open(
    MissionCompositionOpenSessionRequest(
        session_id="interceptor-target-track-demo",
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
        integration_step_s=0.05,
    )
)
assert session.active_authority_profile_id == "live_target_track_guidance"
```

The active action schema contains exactly the seven target position, velocity,
and capture-radius channels—no waypoint channels. Partial updates are held.
Before a mid-session update, the track is rebased at the current target
position and time, so changing only a velocity component cannot teleport the
target to an old reference epoch. Those channels define target truth. Guidance
does not read that truth directly: the selected registered
`relative-state-track` provider projects it into a versioned relative-state
packet at every accepted boundary. Invalid packets make guidance unavailable
with `guidance.law.mode=target_track_unavailable`; the kernel does not silently
fall back to perfect target truth. Capture and event localization remain
truth-based mission evaluations.

For controller integration, select the direct mission and its three-channel
authority. The command is local north/east/positive-up acceleration in
`m/s^2`; the runtime removes the component parallel to current velocity before
using the same bounded force-authority path as guidance:

```python
from taoryx_parametric_interceptors import DIRECT_ACCELERATION_MISSION_TEMPLATE_ID

configuration = provider.configuration(
    profile.model_id,
    mission_template_id=DIRECT_ACCELERATION_MISSION_TEMPLATE_ID,
    startup_authority_profile_id="live_direct_lateral_acceleration",
)
prepared = provider.validate_configuration(configuration)
session = sessions.open(
    MissionCompositionOpenSessionRequest(
        session_id="interceptor-direct-control-demo",
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
        integration_step_s=0.05,
    )
)
result = sessions.step(
    MissionCompositionSessionStepRequest(
        session_id=session.session_id,
        action={"control.lateral_acceleration.local.east.command": 10.0},
        duration_s=0.1,
        expected_sequence=0,
    )
)
```

Omitted axes remain held. Direct control has an explicit three-stage readback:

- `control.lateral_acceleration.accepted.local.*` is the raw held local-NEU
  command accepted from configuration or the live session;
- `guidance.lateral_acceleration.commanded.local.*` is that vector after the
  runtime removes its component parallel to current velocity;
- `guidance.lateral_acceleration.achieved.local.*` is the force-limited point
  mass or force-and-attitude-limited pseudo-6DOF realization.

Per-control Composition feedback binds to the first stage and therefore means
accepted/held, not physically achieved. The projected and achieved vectors,
achievement fraction, available authority, utilization, saturation reasons,
active law/mode, and standard navigation-sensor outputs remain observable.
Each axis advertises a symmetric interval from the selected model's resolved
`max_lateral_acceleration_mps2`. Configuration, the native binding, and the
live session all publish the same bound. The agent action-space projection maps
that interval affinely to `[-1, 1]` without requiring external statistics.
These are component bounds: a multi-axis vector can have a larger norm, and
instantaneous aerodynamic/TVC authority can be lower, so achieved-command
saturation remains part of the required readback.
This is a reduced-order acceleration interface, not an actuator, control
surface, autopilot, or rigid-body moment model. It does not fabricate waypoint
capture or target-sensor applicability.

Mission selection also scopes batch configuration. Waypoint coordinates are
consumed only by `fixed_waypoint_intercept`, target position/velocity/radius
only by `constant_velocity_target_intercept`, and direct acceleration only by
`direct_lateral_acceleration_control`. Discovery publishes the versioned
`mission_control_scope.contract` model property so a UI or agent can recognize
this fail-closed behavior before authoring a request.

Actions may be partial after startup; omitted coordinates retain their last
accepted values. `result.control_feedback` reports requested, accepted, and
observed setpoint values per channel. The observation adds the objective kind,
accepted waypoint or target-reference state, propagated target position,
target-relative velocity, linear time to closest approach, predicted miss
distance, capture radius, generic objective range/captured status, guidance availability,
the resolved guidance-law ID, active law mode, waypoint closing speed,
line-of-sight rate, navigation constant, commanded/achieved lateral
acceleration and their local north/east/positive-up vectors, the active
command-achievement fraction, validated direction error, acceleration limit,
achieved utilization,
deterministic saturation reasons, `phase.id`, and `vehicle.operational`.
`guidance.lateral_acceleration.utilization` is achieved
acceleration divided by the advertised limit and is always in `[0, 1]`.
`control.limit.reason` is `none` when unrestricted; otherwise it is a stable
`+`-delimited combination of `lateral_acceleration_command_saturation`,
`lateral_acceleration_achieved_saturation`,
`bank_angle_saturation`, `pitch_angle_saturation`,
`attitude_acceleration_saturation`, and `body_rate_saturation` in that order.
All but the first two apply only when the pseudo-6DOF response encounters them.
`proportional_navigation` uses positive target-relative closing speed and the
inertial line-of-sight rate. A live retarget that is opening rather than closing
reports `guidance.law.mode=capture_fallback` and temporarily uses waypoint
pursuit; this prevents a zero-demand deadlock without disguising the fallback
as proportional navigation. At `waypoint_capture` or `target_intercept`,
guidance output is inactive but the live authority remains available so a
caller can retarget and resume. A
terminal `ground_impact` completes the episode, marks the vehicle
non-operational, and causes the common session manager to reject further
commands with standard authority-availability diagnostics. Session checkpoints
bind the complete held waypoint and runtime state to the advertised interface
fingerprint. Pseudo-6DOF checkpoints additionally preserve attitude,
Euler-rate, and the selected registered IMU's accepted-history state, so
restored sessions retain deterministic IMU intervals and increments.
Point-mass checkpoints preserve the selected registered translation sensor's
history for the same reason. Both tiers also checkpoint the registered target
sensor's history and random-generator state; repeated observation never
resamples any sensor. Batch diagnostics and live lowering evidence identify the
suite ID, version, fingerprint, tier-specific navigation provider, and target
provider.

Capture is localized inside each held-derivative integration step. Both tiers
use the exact semi-implicit path implemented by their state update, partition
the resulting squared-range quartic at its stationary points, and find the
first capture-volume entry. A fast vehicle therefore cannot pass through a
small waypoint or target radius merely because both step boundaries are
outside it. Batch execution terminates and samples at that localized boundary;
descending ground contact wins an equal-or-earlier tie. A live session still
honors the caller's requested hold duration, emits `waypoint_captured` or
`target_intercept`, records localized event times in lowering evidence, and
latches `guidance.objective.capture_occurred` until the objective is retargeted.
`guidance.objective.captured` remains the instantaneous in-volume status, so a
consumer can distinguish a completed fly-through from current occupancy.

Discovery advertises exact units, bounds, separate waypoint, target-track, and
direct-acceleration mission schemes, model
and profile versions, requested/achieved steering feedback, control saturation,
its cause, normalized achieved utilization, full standard-environment and
air-relative aerodynamic readback, forces, mass, and propulsion output.
Propulsion readback
includes thrust, binary motor schedule, normalized achieved motor level,
remaining propellant, current thrust availability, phase, and active pulse
index. The binary schedule is internal solid-motor timing, not a caller throttle.
Motor burnout removes thrust but does not by itself remove waypoint authority:
the surrogate may continue guidance during coast, and clients can distinguish
those states through `propulsion.available`, `guidance.available`, and
`observation.control_authority`.
The provider exposes the full resolution record through
`provider.resolved_profile(model_id)` and its machine-readable consumption
classification through `interceptor_parameter_usage(...)` and the
`parameter_usage.manifest` model property.

That is also the complete fidelity switch: profiles do not need subclasses or
separate provider registration. Pseudo-6DOF attitude tuning is optional. Most
authors can rely on `control_bandwidth_class` and `maneuverability_class`; an
advanced profile may directly set `attitude_bandwidth_rad_s`,
`attitude_damping_ratio`, `max_body_rate_rad_s`,
`max_body_acceleration_rad_s2`, and `max_bank_angle_rad`. Every such override
retains its simulation-assumption or evidence label.

## Current boundary

- The built-in generic exemplar is intentionally assumption-only.
- The AIM-9X and AIM-120 built-in witnesses are prototype catalogue-linked
  evidence seeds, not validated performance models or verbatim catalogue
  exports.
- Their observed homing families remain separate from the archetype-assumed
  proportional-navigation runtime; the generic archetype defaults to waypoint
  pursuit. Both fidelity tiers call the same shared guidance module.
- Runtime atmosphere and wind come from a selected standard Taoryx environment
  provider; gravity comes from a selected registered gravity model; navigation
  measurements come from a selected versioned suite of registered Taoryx
  sensor providers. All three IDs are bound into Composition configuration
  fingerprints.
- `constant_velocity_target_intercept` propagates only externally supplied
  local-NED target truth. Guidance consumes the standard registered
  direct-geometry relative-state measurement, while capture remains truth
  based. The route reports both truth and packet status/relative-state outputs,
  but does not implement propagation, signatures, gimbals, a track manager,
  datalink, target dynamics, a physical seeker, fire control, or terminal-hit
  logic.
- Capture-volume intersection is a numerical mission event, not a fuze,
  lethality, probability-of-kill, or damage model. The live latch resets when a
  caller changes the objective.
- Single-stage, shaped-thrust, and dual-pulse cases share one resolved motor
  schedule with explicit coast/relight and burnout feedback.
- Point-mass streaming reuses the batch kernel's absolute propulsion clock,
  standard environment boundary, semi-implicit state transition, and Taoryx
  sensor registry. Its compatible translation provider emits ECI
  delta-velocity intervals without inventing attitude or gyro channels, and
  does not add a seeker, datalink, or private guidance route.
- The pseudo-6DOF tier is an attitude response law, not rigid-body dynamics or
  controller-stability qualification. Its batch and streaming paths share the
  same bounded attitude transition, scaled by current lateral command support;
  session integration steps receive the same discrete-stability preflight as
  batch configurations. Its local response-law poles and discretization
  margins can be evaluated at nominal full support or an explicitly frozen
  support fraction, with zero support reported as marginal and unresponsive.
  These remain local metrics with that narrower claim explicit. Its achieved
  lateral vector can lag and differ in direction from
  the commanded guidance vector; both remain directly observable. Body-axis
  air-relative velocity, angle of attack, and sideslip are
  geometric telemetry over this response attitude, not aerodynamic-deck inputs
  or evidence of moment dynamics.
- The portable sensor suite uses Taoryx's standard `IdealImuAdapter`,
  `TranslationAccelerationAdapter`, and `relative-state-track` provider;
  another suite may select compatible registered providers. Suite identity,
  provider kinds, packet timing, payload schemas, and invalid reasons stay
  explicit and do not establish a physical-seeker, propagation, track-manager,
  sensor-error, or hardware-performance claim.
- A CADAC model ID may be retained as `calibration_reference_model_id`, but
  this package does not depend on or copy `taoryx-cadac`.
