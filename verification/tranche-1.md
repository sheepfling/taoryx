# Numerical verification tranche 1

This tranche is the first concrete slice of the numerical-verification goal.
It focuses on the parts of TAOS 1995 that are most likely to fail silently if
the source is misread: coordinate conventions, sign conventions, exact
constants, branch behavior, and singularity diagnostics.

## Scope

In scope:

- ECFC, ECIC, local geocentric, local geodetic, body, wind, velocity, and
  tangent-plane conventions;
- fixed anchor vectors for sign and handedness checks;
- exact-decimal Earth and atmosphere constants;
- aerodynamic and geodesy branch behavior at documented boundaries;
- source-located diagnostics for malformed or ambiguous cases.

Out of scope for this tranche:

- historical TAOS 96.0 compatibility claims;
- full integrated scenario validation;
- optimization solver equivalence beyond the local equation/branch checks;
- any six-degree-of-freedom runtime behavior.

## Work items

### 1. Coordinate and sign conventions

Prove that the implementation matches the documented frame cards and independent
anchor vectors.

Evidence:

- `verification/spec/frames.yaml`
- `verification/spec/anchors.yaml`
- `tests/unit/test_coordinates.py`
- `tests/unit/test_frames_equations.py`
- `tests/unit/test_verification_baseline.py`

Exit condition:

- fixed anchor vectors fail when a sign, axis, or handedness mutation is
  introduced;
- pole, equator, and zero-speed singularities produce source-located
  diagnostics instead of silent fallback;
- every documented frame has a card with explicit axes, origin, singularities,
  and anchor cases.

### 2. Exact constants and model IDs

Prove that the constant registry preserves source decimals and model identity.

Evidence:

- `verification/spec/constants.yaml`
- `tests/unit/test_gravity.py`
- `tests/unit/test_gravity_equations.py`
- `tests/unit/test_atmosphere.py`
- `tests/unit/test_atmosphere_equations.py`
- `tests/unit/test_verification_baseline.py`

Exit condition:

- WGS, TSAP, GEM-T1, and atmosphere constants remain exact-decimal registry
  entries;
- each constant has a source page, table anchor, and linked tests;
- source-vs-runtime discrepancies remain explicit as ambiguity records.

### 3. Branch behavior and singularities

Prove that special-case branches are explicit at the source level and in the
implementation.

Evidence:

- `manual/chapters/chapter02/01_07_body_fixed.tex`
- `manual/chapters/chapter02/01_09_wind.tex`
- `manual/chapters/chapter02/01_11_tangent_plane.tex`
- `tests/unit/test_frames_equations.py`
- `tests/unit/test_coordinates.py`
- `tests/unit/test_verification_baseline.py`

Exit condition:

- aerodynamic angle branches at `alpha = ±90°` and `alpha_T = 90°` are
  preserved explicitly;
- pole and zero-relative-speed cases do not collapse into ordinary-angle
  identities;
- diagnostics identify the relevant source passage or ambiguity record.

### 4. Requirement and ambiguity traceability

Prove that every numerical rule in this tranche has a stable requirement ID and
an explicit ambiguity record where needed.

Evidence:

- `verification/spec/requirements.yaml`
- `verification/spec/ambiguities.yaml`
- `verification/traceability.yaml`
- `tests/unit/test_verification_baseline.py`

Exit condition:

- every in-scope rule in this tranche is tied to a requirement ID;
- every known discrepancy remains represented as a separate ambiguity record;
- no source correction is silently promoted into normal execution.

## Completion rule for tranche 1

This tranche is done when the repository can demonstrate the above evidence
without depending on historical-compatibility claims or integrated end-to-end
scenarios. The result should be enough to trust the local numerical grammar and
frame/constant behavior before moving into the rest of the derivation stack.

####
