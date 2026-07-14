# Analytical table extension fixtures

These `.tbl` files exercise the opt-in taoryx analytical-table extension.
They are not historical TAOS 96.0 fixtures. The extension must be explicitly
enabled before parsing them; the ordinary historical table parser should retain
the source as unsupported rather than assigning it historical semantics.
