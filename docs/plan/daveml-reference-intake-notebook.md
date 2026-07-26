# DAVE-ML reference intake notebook

**Scope:** F-16 S-119, HL-20 Mod K, and the NESC two-stage rocket qualified
reference packages, with the broader canonical catalog tracked separately.

**Claim boundary:** This notebook records source intake and integration work. A
recorded source hash is not the same thing as an executable model in the
checkout. Neither family is promoted to a runnable Taoryx provider until the
declared bytes are present, hash-verified, and replayed through the source
checks.

## Current intake state

| Family | Expected input | Recorded digest | Checkout state | Next gate |
| --- | --- | --- | --- | --- |
| `reference_f16_s119` | Package plus normalized aero source under `qualified-models/f16-s119/` | `eeaeaf17…`; aero `272c647b…` | Package/plant replay passed through Taoryx load contract | Separate overlay and reduction qualification |
| `reference_hl20_mod_k` | Package plus exact source under `qualified-models/hl20-mod-k/` | Package `443e2ed9…`; source `b2ec6260…` | Package/plant replay passed through Taoryx load contract | Separate overlay and reduction qualification |

The machine-readable declarations live in:

- `verification/daveml_reference_integration.yaml`
- `families/reference_f16_s119/qualification/integration-record.yaml`
- `families/reference_hl20_mod_k/qualification/integration-record.yaml`

## Intake command

Put the externally controlled inputs under a local source root. Do not rename
or edit pinned source files. Then run:

```bash
.venv/bin/python tools/verify_daveml_reference_inputs.py \
  --source-root /path/to/taoryx-source-inputs \
  --json /tmp/daveml-reference-intake.json
```

The verifier reports one of these states per declared input:

- `verified_input`: present and exact hash match;
- `blocked_missing_input`: the record exists but the bytes are absent;
- `blocked_hash_mismatch`: bytes are present but are not the recorded source;
- `record_invalid` or `record_incomplete`: the intake declaration itself needs repair.

Use `--strict` in CI or before allowing source replay:

```bash
.venv/bin/python tools/verify_daveml_reference_inputs.py \
  --source-root /path/to/taoryx-source-inputs --strict
```

The corpus archive is `resources/aerospace/daveml/taoryx-corpus-v1.1/corpus.zip` with archive
SHA-256 `9dbb38f23924f2cd2775cb37b1b87476d9ddda672ecab9b84828b253931ebd31`.
Its extracted directory is ignored local input data, not a new source license
or a historical-manual change.

## Integration sequence

Once the payloads are available, work in this order:

1. **Verify distribution integrity.** Check the expected path, exact bytes,
   archive/member hashes if applicable, and source notices.
2. **Inventory the source.** Record variables, tables, equations, units,
   frames, controls, clamps, interpolation, extrapolation, and check cases.
3. **Replay the source evaluator.** Run source/oracle vectors before attaching
   the common runtime.
4. **Build the canonical adapter.** Convert to SI and FRD/NED at one explicit
   boundary, preserving source channels and validity status.
5. **Replay trim and dynamic checks.** Record residuals, tolerances, solver
   settings, and deterministic hashes.
6. **Add overlays separately.** Actuators, SAS, allocation, controllers,
   missions, sensors, reductions, and RL authority are Taoryx layers, not
   silently part of the DAVE-ML plant.
7. **Publish the handoff packet.** Include the source ledger, resolved case,
   plant reports, controls/actuators, telemetry, limitations, and one-command
   reproduction instructions.

## Source replay result

The corpus wheel’s pure DAVE-ML parser/runtime was exercised from an isolated
temporary extraction. The parser and compiler passed for both source files:

| Family | Variables | Breakpoints | Functions | Tables | Embedded checks | Result |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| F-16 S-119 normalized aero | 51 | 4 | 18 | 18 | 17/17 | Parsed, compiled, and accepted |
| HL-20 Mod K exact aero | 361 | 8 | 241 | 169 | 24/24 | Parsed, compiled, and accepted |

This was run with the corpus wheel and `defusedxml` in `/tmp`; neither was
installed into the repository environment. The package/plant replay gap is now
closed for the three qualified packages. The active DAVE-ML gap is
deterministic canonical IR/export and fresh-import comparison, implemented by
`taoryx.trajectory.daveml_semantic` and
`tools/canonical_daveml_roundtrip.py`.

## Standardized library shape

The next ingestion layer now exists under the family directories:

```text
families/reference_f16_s119/
  family.yaml
  plant/source-lock.yaml
  bindings/canonical-controls.yaml
  bindings/canonical-observations.yaml
  bindings/envelope.yaml
  actuators/...
families/reference_hl20_mod_k/
  family.yaml
  plant/source-lock.yaml
  bindings/canonical-controls.yaml
  bindings/canonical-observations.yaml
  bindings/envelope.yaml
  actuators/...
```

`family.yaml` is a source-grounded reference manifest. It records the package
and DAVE-ML hashes, FRD/NED conventions, named `3DOF`, pseudo-`6DOF`, and
rigid-body `6DOF` profiles, evidence per overlay, and the direct source control
and observation schemas. The loader in
`src/taoryx/trajectory/reference_families.py` projects it into the existing
provider-neutral `FamilyPackage`, so the families can be resolved by the same
case contract as synthetic families.

This projection remains metadata-only for plant overlays. The source plant
payload remains immutable and external to the checked-in runtime. Canonical IR
and export are now a separate source round-trip layer; actuator, allocator,
controller, reduction, and mission layers remain separately versioned overlays.

The package-binding preflight is available through
`taoryx.trajectory.inspect_reference_package`. With the local corpus present,
it verifies the package hash, embedded package schema, FRD/NED frames,
quaternion order, reference geometry, validity envelope, and every artifact
hash listed by `manifest.json`. The current corpus reports:

| Family | Package binding | Native stepping |
| --- | --- | --- |
| F-16 S-119 | Verified; 7 embedded artifacts | Load contract and trim-hold evidence replay passed |
| HL-20 Mod K | Verified; 16 embedded artifacts | Load contract and glide-hold evidence replay passed |

This distinction is deliberate: a package can be structurally and
provenance-correct before Taoryx can evaluate its force/moment graph.

## First `.txcollection` build

The F-16 is now the first M0–M2 collection fixture. It can be rebuilt from
the external corpus with:

```text
python tools/build_daveml_collection.py \
  --source-root /tmp/taoryx-aerospace-data-corpus-v1.1/taoryx-aerospace-data-corpus-v1.1 \
  --output build/f16-s119.txcollection
```

The builder embeds the exact three source DAVE-ML documents, the verified
`.txair` runtime artifact, the control/runtime mappings, 29 source validation
cases (17 aerodynamic, 3 mass-property, and 9 propulsion), the trim-hold
trajectory, explicit contribution authorities, transforms, and the dynamics
sidecar. The archive writer fixes member order, timestamps, permissions, and
the checksum ledger; repeated builds currently produce the same archive hash.

The same builder now supports the HL-20 fixture:

```text
python tools/build_daveml_collection.py \
  --family-manifest families/reference_hl20_mod_k/family.yaml \
  --source-root /tmp/taoryx-aerospace-data-corpus-v1.1/taoryx-aerospace-data-corpus-v1.1 \
  --output build/hl20-mod-k.txcollection
```

For either archive, run the source-preserving round-trip check:

```text
python tools/roundtrip_daveml_collection.py \
  --collection build/f16-s119.txcollection \
  --output-dir build/f16-s119-daveml
```

This verifies the collection checksum ledger, exports the embedded DAVE-ML
bytes, and reloads their hashes in a fresh process. It establishes source
identity (L0/L1); it does not yet establish canonical regenerated DAVE-ML or
runtime replay (L2–L4).

The collection is deliberately a derived artifact. The repository retains the
builder, source/hash contracts, and promoted canonical source payload under
`resources/aerospace/daveml/`.

Run the package/runtime replay with:

```text
python tools/replay_daveml_reference.py path/to/f16.txair path/to/hl20.txair
```

The replay verifies the package ledger and source identity, checks the pinned
force/moment evidence where present, exercises the Taoryx rigid-body load
contract, and requires the package trim/glide hold evidence to pass. It does
not claim that Taoryx now contains a general-purpose DAVE-ML compiler.

The same command now passes all three qualified packages from
`resources/aerospace/daveml/nesc-model-catalog-v1.0/qualified/`. The F-16 and HL-20
use trim/glide-hold evidence; the NESC rocket uses benchmark, convergence,
schedule, and source-equivalence acceptance evidence. The adapter records that
evidence family explicitly rather than forcing the rocket through a trim
schema.

For a built collection, replay the declared runtime artifact through the outer
collection checksum and manifest boundary as well:

```text
python tools/replay_daveml_collection.py \
  --json build/f16-s119-collection-replay.json \
  build/f16-s119.txcollection
```

The report identifies the collection, family, runtime member, source identity,
and Taoryx load-contract evidence. This closes the initial package-to-
collection replay handoff without implying canonical DAVE-ML regeneration.

## Family-specific notes

### F-16 S-119

The recorded integration boundary is a nonlinear powered fixed-wing rigid-body
6DOF plant with steady propulsion and source-bound fixed mass properties. The
next overlays are actuator dynamics, SAS, rate/attitude/route control, an
airborne trim-to-arrival mission, and reductions generated from the qualified
parent. Runway, low-speed, fuel-transient, and operational-fidelity claims stay
out of scope until separately qualified.

### HL-20 Mod K

The recorded integration boundary is a nonlinear unpowered lifting-body
rigid-body 6DOF plant with a byte-pinned aerodynamic source and separately
bound fixed mass properties. Preserve the seven direct surfaces, then add
actuator dynamics, logical allocation, body-rate damping, energy-management
guidance, and an airborne release-to-arrival-energy mission. Do not infer
touchdown capability from an aerodynamic source.

## Friction ledger

This ledger separates observed integration pain from untested behavior.

| Status | Observation | Consequence | Resolution |
| --- | --- | --- | --- |
| resolved intake blocker | The archive was initially unavailable locally | F-16/HL-20 source replay could not begin | Archive synced; corpus extraction and hash verification completed |
| resolved integrity gate | Qualified packages were present only inside the corpus archive | Package presence alone was insufficient | Outer corpus ledger, ZIP CRC, package manifests, and embedded member ledgers all pass |
| resolved parser gate | The repository environment did not include the corpus wheel’s `defusedxml` dependency | The first isolated CLI attempt stopped before parsing | Installed the single dependency into `/tmp`; direct F-16/HL-20 parse, compile, and static checks pass |
| important distinction | The corpus also contains A320 OpenAP and JSBSim data | This does not create an A320 DAVE-ML source model | Keep A320 as a separate OpenAP/JSBSim integration path |
| resolved runtime boundary | No Taoryx package replay adapter was present | Payload verification stopped before the rigid-body load contract | `taoryx.trajectory.replay_reference_package` now verifies and replays both reference packages |
| design constraint | Controller and actuator data are not source-plant evidence | Prevents accidental overclaiming | Keep them as versioned overlays with separate evidence classes |
| not yet tested | Canonical DAVE-ML regeneration and semantic round trip | No conclusion yet | Keep in the separate round-trip tranche |
| not yet tested | Controller, event, and reduction overlays in this repository | No conclusion yet | Qualify as separate layers from the replayed parent plant |

New contributors should append rows here when a command, adapter, or regression
reveals a new failure mode. Do not replace a blocker with a guessed value.
