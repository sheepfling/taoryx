# X-15 coherent 6-DOF research fixture

This fixture preserves the supplied X-15 package and its generated TAORYX
tables. The source package validation report is retained under
`checks/validation_report.json`.

Generated tables:

- `x15_static_6axis.tbl`: Mach/altitude/alpha/beta `CX`, `CY`, `CZ`, `CMX`,
  `CMY`, and `CMZ` tables;
- `x15_symmetric_stabilator_6axis.tbl`;
- `x15_differential_stabilator_6axis.tbl`;
- `x15_rudder_6axis.tbl`.

All angular table axes are radians and altitude is meters. The tables use the
X-15 body convention (+X forward, +Y right, +Z down) and the supplied
18.580608 m² reference area.

This is the strongest current candidate for a native 6-DOF research fixture,
but it is not a flight-qualified database. The source identifies itself as a
beta public research surrogate, has zero-wind atmosphere data, and has no
certified thermal model.

It is not substituted into California-to-Hawaii yet: the X-15 aero envelope
ends at 80,000 ft and the CA–HI mission climbs substantially higher. A valid
comparison requires an X-15-compatible mission profile or an explicitly
documented envelope extension.

Regenerate the translations with:

```bash
python tools/import_x15_tables.py
```
