# Analytical aerodynamic table extensions

TAORYX should support analytical or automatically generated aerodynamic tables
as an additive extension to the TAOS 96.0 table language. The extension must
never change the meaning of an existing historical `.tbl` file.

## Boundary

The historical-compatible surface remains an ordinary coefficient table:

```text
(cone-cd)
  table cd(alphat) no-extrap sref=0.785398
  alphat = 0, 5, 10, ...
  cd = ...
```

An analytical source is a generator for that table, not a new runtime force
law. Generation should be deterministic, inspectable, and losslessly lowered
to the same table representation used by measured or CFD data.

The initial shape families are:

- sphere;
- axisymmetric spheroid;
- closed cylinder;
- closed cone, with separate front/rear aerodynamic anchors; and
- triaxial ellipsoid, using `alphat` and `phi`.

The first four can use a one-angle table where appropriate. A triaxial body
requires a two-angle table. The word “four” should therefore mean four
geometric families only if spheroids are intentionally grouped with
ellipsoids; the registry should keep the concrete model IDs distinct.

## Proposed opt-in syntax

The exact grammar is not yet normative, but the intended form is:

```text
(demo-cone-analytic)
  generate cd(alphat) from shape=cone
    radius=0.5 height=2.0
    reference-area=0.7853981634
    model=front-broadside-rear
    cd-front=0.30 cd-broadside=1.10 cd-rear=1.25
    angle start=0 stop=180 step=5
    no-extrap
```

For a triaxial ellipsoid:

```text
(demo-ellipsoid-analytic)
  generate cd(alphat,phi) from shape=triaxial-ellipsoid
    axes=1.0,0.6,0.4 reference-area=0.7539822369
    model=projected-area projected-cd=1.0
    alphat start=0 stop=180 step=5
    phi start=0 stop=360 step=5
    no-extrap
```

This syntax should be introduced behind an extension/profile switch rather
than added to the historical grammar claim. The parser should preserve the
source declaration and the generated ordinary table, including generator
parameters and a provenance fingerprint.

## Semantic rules

1. `reference-area` is required and positive.
2. Shape dimensions are required, positive, and dimensionally compatible.
3. Axis values are finite, monotonic, and within the declared domain.
4. A periodic `phi` axis must define its seam explicitly; `360` may be a
   duplicate endpoint but must not create a second physical period.
5. `projected-area` models are geometry surrogates, not universal drag laws.
6. Front/rear cone coefficients must remain separate from projected geometry.
7. Generated tables must be reproducible from the declaration and generator
   version.
8. The resulting `.tbl` must pass the ordinary table validator and be usable
   wherever a normal `cd` table is accepted.

## Provenance and claims

Every generated table should record:

- source declaration digest;
- generator name and version;
- shape/model ID;
- exact input parameters and decimal strings;
- axis grids and flattening order;
- whether values are geometric, illustrative, calibrated, CFD-derived, or
  measured; and
- the verification claim supported by the result.

The built-in shape models are suitable for parser, table, and sensitivity
fixtures. They do not establish historical TAOS 96.0 behavior or validated
aerodynamic performance.

## Implementation sequence

1. Register the extension syntax and preserve it losslessly.
2. Lower each analytical declaration to an ordinary `cd` table.
3. Validate dimensions, domains, seams, and table cardinality.
4. Add golden fixtures for all five concrete model IDs.
5. Compare generated output against the existing full-angle analysis CLI.
6. Add negative diagnostics for missing dimensions, invalid domains, and
   unsupported model parameters.
7. Only then consider using generated tables in a runtime trajectory case.

The current implementation already provides the model and CLI foundation in
[`src/taoryx/aero_drag_tables.py`](../../src/taoryx/aero_drag_tables.py) and
[`tools/aero_drag_analysis.py`](../../tools/aero_drag_analysis.py). The syntax
extension remains a future language feature until its grammar and provenance
contract are reviewed.
