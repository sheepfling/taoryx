# Taoryx CADAC plug-in

`taoryx-cadac` is an optional, source-bound Taoryx plug-in for the public
CADAC aerospace vehicle examples. It contributes the CADAC Mission Composition
catalog and the CADAC table importer without copying any upstream input decks,
coefficient values, or generated trajectories into this repository.

The initial release is deliberately discovery-first. Installing it exposes the
phase-aware CADAC actor catalog through the standard `taoryx.plugins` entry
point, but does not scan an environment variable, parse a source archive, or
construct actor schemas while Taoryx is discovering plug-ins. The provider
factory materializes the catalog only when a host selects the CADAC Composition
API. A source case becomes executable only when an exact, caller-owned case
binding is supplied to the corresponding CADAC runtime. There is no fallback to
another vehicle or to a synthetic deck.

## Install

From this source checkout:

```bash
python -m pip install -e . -e packages/taoryx-cadac
taoryx plugins check --profile cadac
taoryx plugins inspect taoryx.cadac
```

The package requires an independently installed `taoryx` host. It does not
download CADAC or assert a license for the upstream source material. Obtain a
local, authorized copy separately, for example from the
[`missiondesignsolutions/CADAC`](https://github.com/missiondesignsolutions/CADAC)
repository.

## Focused maintenance gate

The CADAC inner loop is source-bound and does not require the reference-model
aggregate or another vehicle family:

```bash
python tools/dev.py test-cadac-discovery  # entry point and deferred provider only
python tools/dev.py test-vehicle cadac_aim5  # one selected AIM5 actor boundary
python tools/dev.py check-cadac
```

The AIM5 slice binds only its synthetic source case and creates a provider
scope for `cadac.aim5.missile`, so it does not materialize unrelated actor
schemas as a side effect of one-model development. Use
`check-cadac` only for the intentional multi-actor integration sweep: it first
proves metadata-only discovery leaves the actor catalog unloaded until the
provider API is selected, then exercises explicit test-only AIM5, CRUISE5,
MAGSIX, GHAME3, GHAME6, ROCKET6G, ADS6 SRBM, standalone ADS6 SAM and AIRCRAFT3, AGM6, FALCON6, ADS6 engagement,
and SRAAM6 source bindings. AIM5, ADS6 SRBM, AGM6, ADS6 engagement, and SRAAM6 retain their
source-owned persistent-session APIs, report the catalog provider revision
selected by the caller, and publish the native `relative-state-track` sensor
readback. The standalone ADS6 SAM proof instead verifies all three
caller-owned direct boundaries—cross fins, physical TVC, and aggregate RCS—as
fixed batch controls with requested/achieved output evidence. It explicitly
keeps persistent stepping and `SensorBus` delivery blocked: its source RF/IR
controller requires the live target/radar package scheduler owned by the ADS6
engagement composition. AIRCRAFT3 is also explicitly batch-only: its source
program owns the configured steady/g-turn/escape behavior and publishes its
commanded-versus-achieved bank/load response without promoting it to rigid-body
truth or a native sensor. The ADS6 SRBM proof additionally checks the normal
command/realized-acceleration output pairs used for source-controller
time-domain analysis; AGM6 and SRAAM6 prove source command-to-physical-fin
readback; FALCON6 proves caller-owned direct physical-surface commands plus
requested/achieved position and actuator-limit feedback; the ADS6 package proof
carries that boundary through its
source-scheduled SAM/target/RADAR0 composition. The installed-wheel boundary
can be run independently with:

```bash
python tools/verify_plugin_wheels.py --plugin cadac --python .venv/bin/python
```

Neither check includes a redistributed CADAC deck or scans a workstation for a
source checkout.

For the source-backed variant contract, pseudo-6DoF boundary, and the focused
authoring/test workflow for AIM5, ADS6 SRBM, ADS6 SAM, SRAAM6, and AGM6, see
[CADAC missile tuning and reduced-order authoring](../../docs/cadac-missile-tuning.md).

### Vertical scope boundary

Every source-bound CADAC vertical gate now binds one source case into a matching
one-model provider scope. AIM5, ADS6 SRBM, ADS6 SAM, AIRCRAFT3, AGM6, FALCON6,
and SRAAM6 select their exact actor model. ADS6 engagement selects its
source-owned package model, even though it contains SAM, target, and RADAR0
actors and therefore has no one-actor manifest descriptor. Embedded actors can
still appear as result roots, but never become registered fallback models.

CADAC remains a single coherent distribution. A new source-bound model must add
its own `test-vehicle` vertical slice with this exact boundary, preserving its
declared controls, sensor status, and batch/session availability without
constructing unrelated actor schemas.

CRUISE5 is the current low-fidelity CADAC vertical boundary: its executable
source waypoint/line realization is pseudo-6DOF, with the source program owning
guidance and lagged bank/alpha response. It advertises `source_program_control`
with no caller action channels, returns labeled waypoint, command, response,
propulsion, and geodetic truth outputs through batch, and explicitly blocks a
persistent session. Its point-mass translation phase remains validation-only;
it is never silently lowered from the pseudo-6DOF source runtime. The integration
record labels the source controller and its command outputs, while time-domain
comparison and formal stability remain blocked until the model publishes an
explicit command-to-realized-response mapping and closed-loop state definition.

When a caller binds one local source case, it can construct a selected model
scope rather than materializing every CADAC actor schema:

```python
provider = CadacSourceCaseBindings(cruise5_case_path=case_path).build_provider(
    selected_model_ids=("cadac.cruise5.cruise_vehicle",),
)
```

The scope rejects an omitted explicitly bound case, registers no unselected
actor as a fallback, and is intended for focused model development and testing.
It checks the selected scope before constructing any bound source runtime, so
an out-of-scope eager deck cannot add parsing or Pydantic validation work to a
one-model operation.

MAGSIX uses the same selected boundary for its independently executable
point-mass trajectory and spin model. Its source program is fixed rather than
an externally controllable controller, so it advertises no caller action,
no session, and no controller-analysis claim. The restricted pseudo-6DOF
attitude phase is discoverable and validates its fidelity selection, but stays
explicitly non-executable until an independent attitude runtime exists.

GHAME3 uses a selected source-bound 3-DoF round-Earth batch boundary. Its
source event schedule supplies prescribed angle-of-attack, bank, and hypersonic
propulsion behavior, so those values are labeled telemetry rather than caller
actions or rotational states. The batch result carries geodetic, force,
aerodynamic, propulsion, and source-event readback; persistent stepping,
native sensor delivery, controller comparison, and formal stability remain
blocked until a separate closed-loop state/session contract is implemented.

GHAME6 uses a selected source-bound **composition-package** boundary. HYPER6,
SAT3, and RADAR0 remain independent roots in every result; source release
events change HYPER6's realization without inventing child lineage. Its eleven
physical-surface and aggregate-RCS coordinates are caller-owned fixed-batch
controls with requested/achieved readback, while per-sample phase telemetry
keeps the atmospheric T4 envelope distinct from later aggregate-RCS T3 phases.
Each RADAR0 source update publishes two explicitly separate records: the
source-shaped noisy track and a standard `relative-state-track` packet from
the same committed RADAR0/SAT3 geometry. This does not claim a persistent
`SensorBus`, source GNC/estimation replay, or formal stability analysis.

ROCKET6G uses the selected boundary for one phase-aware three-stage launch
vehicle. Its TVC and aggregate-RCS command coordinates are caller-owned,
fixed-batch controls with labeled requested/achieved nozzle, force, and moment
readback. Per-sample source phase, RCS force-mode, and runtime-fidelity telemetry preserves the
T3 aggregate-RCS versus T4 physical-TVC distinction. The current direct-command
plant supports time-domain trace analysis and controller comparison from those
published pairs, but a persistent session, closed-loop linearization, and
formal stability margins remain blocked.

## Convert local CADAC decks

The included converter accepts `1DIM`, `2DIM`, and `3DIM` CADAC table decks and
writes deterministic, provider-neutral `taoryx.table.v1` resources plus a
hash-checked `taoryx.table_bundle.v1` manifest:

```bash
taoryx-cadac convert-tree /path/to/CADAC --output build/cadac-tables
taoryx-cadac validate-table build/cadac-tables
```

`convert-bundle` converts every physical deck referenced by one `input.asc`;
`convert-deck` converts a single table deck. The output records source paths
and SHA-256 digests so generated resources remain traceable to the exact local
source bytes. Do not commit generated bundles containing upstream coefficient
data unless their redistribution status has been independently confirmed.

## Claim boundary

The plug-in preserves source actor identity, source order, table direction,
interpolation policy, and provenance. Its compatibility runtimes are a Taoryx
reconstruction, not a claim of numerical parity with a compiled CADAC
executable. The CADAC integration documentation states each model's current
fidelity and execution boundary.

The source-compatibility code is under a scoped static-typing exception while
the imported NumPy-heavy prototype is typed incrementally. This does not
promote compiled-CADAC numerical parity.
