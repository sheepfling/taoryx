# Simple Aero-style specialized segments

The Simple Aero material is valuable as a compact library of maneuver shapes, but
the checked-in problem files are synthetic translations and not a recovered
Simple Aero runtime. The reusable unit is therefore the segment contract, not the
literal `.prb` text.

## Shared phase vocabulary

`src/taoryx/specialized_segments.py` defines the vehicle-neutral contracts:

| Contract | Typical Simple Aero families | Reusable intent |
| --- | --- | --- |
| `powered_ascent` | ballistic, CBCR, crossrange | bounded thrust/boost phase |
| `ballistic_coast` | every launch family | passive or low-control coast |
| `bank_maneuver` | CBCR, crossrange, MARV, slalom, weave | signed bank and lateral steering |
| `alpha_profile` | phugoid, range extension | bounded alpha/energy profile |
| `skip_maneuver` | skip | entry/exit lift modulation |
| `terminal_pronav` | guided families | terminal ProNav handoff |
| `moving_target_intercept` | two-trajectory ProNav | explicit target-track intercept |

The composition layer exposes these as templates. Their `GoalKind` identifies
the phase contract, while the source `.prb` remains responsible for native
syntax such as `*prop`, `*aero`, `*fly`, and `*when`.

The same material is published as the `simple_aero` workflow model through
`RegistryMissionCompositionProvider`. Its portable schema advertises launch
and aimpoint geometry, initial mass and speed, burnout/apogee checkpoints,
fixed-L/D inputs, and ordered per-occurrence segment parameters. Reviewed
mission recipes include ballistic, phugoid, skip, slalom, and weave. This is a
workflow advertisement—not a physical vehicle entry—and only its synthetic
point-mass realization is declared.

## Start with parameters, not problem-file boilerplate

For a reduced-order 3-DOF or fixed-L/D trajectory, use
`taoryx.simple_aero_builder`. The small input surface is intentionally shaped like
the questions an engineer normally has at the start of a case: burnout speed
(`Vbo`), apogee, pitch-over angle, target geometry, heading offset, and fixed
`L/D`.

```python
from taoryx.simple_aero_builder import build_fixed_ld_3dof

build = build_fixed_ld_3dof(
    scenario_id="x8-crossrange-demo",
    vehicle_id="skywalker-x8",
    family="crossrange",
    vbo_m_s=900.0,
    apogee_altitude_m=20_000.0,
    pitch_over_angle_deg=75.0,
    target_range_m=100_000.0,
    target_bearing_deg=90.0,
    initial_heading_offset_deg=8.0,
    lift_to_drag=4.0,
)
build.write("build/x8-crossrange-demo.prb", "build/x8-crossrange-demo.manifest.json")
```

The builder derives a boost checkpoint from `Vbo` and the configured surrogate
acceleration, derives a coast checkpoint from apogee, computes the initial
heading as `target bearing + heading offset`, and emits a two-trajectory
terminal ProNav handoff. Use `boost_duration_s` or `coast_duration_s` when a
vehicle-specific phase time is known. The manifest is the review surface: it
records the derived values and assumptions instead of hiding them in generated
text. The `family` value is provenance metadata for this common
powered-ascent/coast/maneuver/terminal skeleton; it does not silently change a
vehicle model or claim to reproduce every family-specific Simple Aero law.

The generated `.prb` uses numeric `CA`/`CN` values with
`CN = |CA| * L/D`. This is a fixed-coefficient surrogate for segment bring-up,
not an aerodynamic model. Before vehicle promotion, replace it with the
vehicle's bounded aero tables and rerun the convention, force-closure,
time-step, and segment-promotion gates.

## X-15 reuse path

The X-15 is a good reuse target because it already has separate point-mass,
kinematic, and rigid-body evidence. The current focused fixtures are:

| X-15 use | Fixture | What it proves now |
| --- | --- | --- |
| phugoid-like alpha/energy profile | `examples/showcases/x15_rocket_to_hawaii/x15_phugoid_3dof.prb` | bounded signed alpha profile, speed/altitude exchange, source-deck limits |
| multi-cycle weave | `examples/showcases/x15_rocket_to_hawaii/x15_weave_6dof.prb` | two native bank reversals, alpha/beta bounds, force/moment closure |

Run the focused checks with:

```bash
python -m pytest tests/e2e/test_glider_family_validation.py \
  -k 'phugoid or weave' -o addopts=''
```

The phugoid fixture is deliberately named “phugoid-like”: it is an alpha and
energy-profile excitation, not yet proof of the vehicle's natural longitudinal
mode. The next promotion gate is a trim-linearized or independently identified
natural-mode comparison. The weave fixture reuses the already source-anchored
bank-reversal seam, but still needs a crossrange objective and actuator/slew
packet before route-level promotion.

## Choose settled X-15 maneuvers

The selectable X-15 menu is
`verification/x15_maneuver_catalog.yaml`. Load it instead of searching through
example filenames:

```python
from taoryx.x15_maneuvers import load_x15_maneuver_catalog

catalog = load_x15_maneuver_catalog("verification/x15_maneuver_catalog.yaml")
for maneuver in catalog.select("settled"):
    print(maneuver.id, maneuver.mode, maneuver.source_problem)

weave = catalog.get("weave")
assert weave.quality_gates_pass
```

The current settled menu contains powered ascent, release/coast, bank-energy
management, the bounded phugoid-like alpha profile, and the two-cycle weave.
Each row names its source problem, tables, focused test, and segment-quality
gates. `terminal-pronav` remains a `candidate` because terminal miss distance
has not yet been evidenced. Settled means “the focused fixture's declared
segment gates pass”; it does not mean the maneuver is ready for every vehicle,
full-route verification, landing, or flight-performance claims.

## Validate the Simple Aero segment fixtures

Run the isolated Simple Aero ladder with:

```bash
python tools/dev.py test-simple_aero-segments
```

This checks each of the nine simple-aero segment fixtures for TAORYX grammar,
runtime completion, required telemetry, finite values, monotonic time, and a
single isolated segment span. Those are fixture-quality gates. The result also
reports the deferred vehicle gates—such as aero-table bounds, force/energy
closure, actuator limits, and time-step convergence—because the synthetic
simple-aero fixtures do not establish those claims. A passing fixture ladder
therefore makes a segment usable for parser/runtime composition experiments,
not vehicle promotion.

## Reusing a segment on another vehicle

Start with the same phase contract and change the vehicle adapter, tables,
controls, and valid operating envelope. Do not copy a Simple Aero bank value or
time-to-go threshold as though it were universal.

```python
from taoryx.composition import TrajectoryBuilder

builder = TrajectoryBuilder(
    "glider-crossrange",
    vehicle="skywalker_x8",
    family="fixed-wing-uav",
    source_problem="examples/mission.prb",
    mode="point_mass_3dof",
)
builder.use(
    "powered_ascent",
    "boost",
    duration_s=5.0,
    controller="x8-powered-trim",
    actuator_binding="x8-elevons",
)
builder.use(
    "ballistic_coast",
    "coast",
    duration_s=40.0,
    controller="x8-coast",
    actuator_binding="x8-elevons",
)
builder.use(
    "bank_maneuver",
    "crossrange",
    duration_s=35.0,
    controls=("bank_deg",),
    controller="x8-crossrange",
    actuator_binding="x8-elevons",
)
```

The segment is not promoted merely because it composes or parses. The
vehicle-specific evidence must cover the declared mode, state/control
convention, table bounds, signed bank/alpha response, entry and exit events,
time-step convergence, and actuator/slew behavior.

## Simple Aero catalog and evidence ladder

`verification/simple_aero_segment_catalog.yaml` maps each fixture family to its
reusable contract sequence. `fixture-ready` means the translation and source
trace are present; it does not mean `composition-ready` or `vehicle-promoted`.

For each reuse target, run the ladder in this order:

1. Validate the source grammar and preserve the source/fixture provenance.
2. Bind the phase to the vehicle's 3-DOF or pseudo-6-DOF mode.
3. Run data and convention firewall checks, especially signs and table axes.
4. Check plant closure and `dt`/`dt/2` convergence for the isolated segment.
5. Check the maneuver objective: bank direction, alpha profile, skip corridor,
   range closure, or ProNav LOS/closure.
6. Check entry, handoff, exit, termination, and saturation evidence.
7. Add a vehicle-specific promotion row before using the segment in a route.

A later 6-DOF implementation can reuse the same phase contract, but must add
attitude, rate, allocation, inertia, and aerodynamic response gates. A green
3-DOF fixture is an anchor, not a 6-DOF certification.
